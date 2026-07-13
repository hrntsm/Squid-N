//! 階(Story)の自動生成。
//!
//! 節点の標高(Z)をクラスタリングして階を推定し、各階に剛床(ダイアフラム)と
//! 地震重量を設定する。地震静的解析(Ai分布)・プッシュオーバー・偏心率計算の
//! 前提データを 1 操作で用意するための機能。
//!
//! 重量は「自重(線材: ρ·A·L·g、壁・シェル: ρ·t·A·g) + 指定荷重ケースの
//! 鉛直下向き荷重」を節点に配分し、階ごとに合計する簡易法(節点支配)による。
//! 自重は左右対称な等分布荷重なので両端 1/2 ずつ、指定荷重ケースの部材荷重は
//! 単純支持梁の静定反力（`static_reactions`）で両端に配分する（RESP-D マニュアル
//! 「地震荷重の計算」の CMoQo による梁せん断力 Q0 に相当。対称荷重では結果的に
//! 自重と同じ 1/2-1/2 になる）。
//!
//! 剛床代表節点は、剛床に含まれる節点の慣性力重心（重量重み付き重心）に
//! 専用の仮想節点として自動生成する（既存節点の流用ではない）。
//! 参考: RESP技術ブログ「剛床に関連する操作や考え方のまとめ」(2026-05-29)。
//! 並進慣性重量は ΣiW、回転慣性重量は ΣiW·ir² となり、スレーブ節点の面内応答は
//! `crates/squid-n-solver/src/constraint.rs` の RigidDiaphragm 縮約で
//! ix = Gx − iry·Gθz, iy = Gy + irx·Gθz として復元される。
//! 回転慣性重量 ΣiW·ir² は質量を代表節点自体に持たせなくても、要素・節点側に残った
//! 質量が Reducer の TᵀMT 縮約（`eigen.rs`）で自動的にマスターへ集約されるため、
//! 代表節点の `mass` は常に `None` とする（二重計上を避ける）。

use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::ids::{LoadCaseId, NodeId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementData, ElementKind, KBraceWeightRule, LoadCfg, MemberLoadKind,
    MiscWallTransfer, Model, Node, Story,
};

/// 重力加速度 [mm/s²]（内部単位系 N-mm-s、質量 ton）。
/// レビュー §1.11: `squid-n-core` 側の定数（`capacity_spectrum.rs` も使用）と
/// ソースオブトゥルースを統一する。
use squid_n_core::units::GRAVITY_MM_S2;

/// 同一階とみなす標高差 [mm]。
const LEVEL_TOL_MM: f64 = 1.0;

/// 生成結果。[`Model`] へ適用するのは呼び出し側（EditCommand 経由）。
#[derive(Clone, Debug, PartialEq)]
pub struct StoryGenResult {
    /// 下から順の階（基部レベルは含まない）。
    pub stories: Vec<Story>,
    /// 各節点の所属階（`model.nodes` と同順。基部レベルは None）。
    /// 長さは `model.nodes.len()`（新規に生成される代表節点は含まない。
    /// 代表節点の所属階は `rep_nodes` 側の `story` フィールドが正）。
    pub node_story: Vec<Option<StoryId>>,
    /// 各階の剛床拘束（`Reducer` が読む `model.constraints` 用）。
    pub constraints: Vec<Constraint>,
    /// 生成・更新される剛床代表節点（座標＝慣性力重心、拘束・所属階設定済み）。
    /// ID が既存の `model.nodes` 範囲内なら置換（再利用）、範囲外なら新規追加。
    pub rep_nodes: Vec<Node>,
    /// 適用後の `model.generated_masters` の全量。
    pub generated_masters: Vec<NodeId>,
}

/// 平面多角形（3D座標、頂点が同一平面上と仮定）の面積。
///
/// Newell の公式 `N = 1/2 Σ(Vi × Vi+1)`, `Area = |N|` による。
/// 凸・非凸いずれも、頂点が境界を一周する順序で与えられていれば成立する。
/// 壁・シェル要素の自重（§1.2）算定に用いる。
fn polygon_area_3d(pts: &[[f64; 3]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let p0 = pts[i];
        let p1 = pts[(i + 1) % n];
        nx += p0[1] * p1[2] - p0[2] * p1[1];
        ny += p0[2] * p1[0] - p0[0] * p1[2];
        nz += p0[0] * p1[1] - p0[1] * p1[0];
    }
    0.5 * (nx * nx + ny * ny + nz * nz).sqrt()
}

/// 2 点間の 3D 距離 [mm]。
fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// 「鉛直材（柱）」判定。両端の水平距離（XY平面）が 1mm 未満なら鉛直とみなす。
/// 仕上げ周長式・雑壁の柱探索・柱脚梁せい付加の判定に共通で用いる。
fn is_vertical_pair(a: [f64; 3], b: [f64; 3]) -> bool {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt() < 1.0
}

/// 鋼材単位体積重量（RESP-D マニュアル γs=77kN/m³）を内部単位系の質量密度
/// [ton/mm³] に換算した値（≈7.85e-9）。ダンパー支持部重量（§ダンパー自重）に用いる。
/// `squid-n-core::units` の単一ソースオブトゥルースから導出する（レビュー §1.11 と同じ方針）。
fn steel_density_ton_mm3() -> f64 {
    squid_n_core::units::to_internal::mass_density_from_unit_weight_kn_m3(
        squid_n_core::units::STEEL_UNIT_WEIGHT_KN_M3,
    )
}

/// 仕上げ周長 φ（マニュアル「柱梁自重」の仕上げ荷重）。
/// 鉛直材（柱）は四周仕上げ `2(b+D)`、それ以外（梁）は三面仕上げ `b+2D`。
/// 断面の `width`/`depth` のいずれかが 0 以下の場合は 0（換算対象外）とする。
fn finish_perimeter(width: f64, depth: f64, is_vertical: bool) -> f64 {
    if width <= 0.0 || depth <= 0.0 {
        return 0.0;
    }
    if is_vertical {
        2.0 * (width + depth)
    } else {
        width + 2.0 * depth
    }
}

/// 自重 1 件分の重量とその帰属。
///
/// 地震用重量の節点集計（[`generate_stories_multi`]）と、長期応力解析用の
/// 自重(自動)荷重ケース（[`crate::self_weight`]）が同じ算定を共有するための
/// 中間表現。重量の算定規則（自重算定長・スラブ厚控除・仕上げ・ダンパー置換等）は
/// [`enumerate_self_weight`] に一元化する。
pub(crate) enum SelfWeightItem {
    /// 線材（柱・梁・ブレース）の自重（総量 [N]）。`elem_idx` は `model.elements` の添字。
    Line { elem_idx: usize, total: f64 },
    /// ダンパー装置＋支持部の重量（総量 [N]）。両端節点（`model.nodes` 添字）へ 1/2 ずつ。
    Damper { ni: usize, nj: usize, total: f64 },
    /// 壁・シェルの自重の頂点配分（`model.nodes` 添字 → [N]）。
    Panel { shares: Vec<(usize, f64)> },
}

/// モデル全要素の自重を列挙する（§柱梁自重・§壁自重・§ダンパー自重）。
///
/// - 線材（柱・梁・ブレース, `ElementKind::Beam`/`Brace`）: ρ·A·L·g。
///   §1.8: 自重算定長 L は、コンクリート材（`mat.fc` あり = RC/SRC）の水平材（梁）は
///   柱面間距離（`len - face_i - face_j`、負にならない範囲）、鉛直材（柱）は
///   床上面から床上面まで（＝節点間距離。フェイス控除しない）、鋼材（S 梁・柱）は
///   節点間距離（マニュアル「柱梁自重」：RC/SRC 大梁は柱面間距離、
///   RC/SRC 柱は床上面から床上面、S 梁・柱は節点間距離）。
///   §1.9: RC/SRC 梁の断面積は梁上部のスラブ厚分 b·t を控除する
///   （w_c = γ·b(D−t)+…。スラブ重量は構造芯間の面積で別途計上されるため、
///   控除しないと梁幅×スラブ厚の体積が二重計上になる）。スラブが定義されて
///   いないモデル（純フレーム等）では控除しない。
///   §柱の長さ: コンクリート柱（鉛直材）で下端節点に別の柱（鉛直 Beam/Brace）が
///   下から接続していない場合、下端節点に取り付く梁（非鉛直 Beam）の最大せいを
///   自重算定長へ加算する（マニュアル「下階に柱がない場合...柱脚に取付く梁の最大せいの
///   長さを柱長さに付加」）。
///   ギャップ対応: 鋼材のみ `load_cfg.effective_steel_factor()`（鉄骨重量割増率）を乗じ、
///   `load_cfg.extra_line_weight`（耐火被覆等の付加線重量 [N/mm]）・
///   `load_cfg.finish_area_weight`（仕上げ面重量 w_f、周長 φ から自動換算）が
///   あれば自重算定長を掛けて加算する。
/// - 壁・シェル（`ElementKind::Wall`/`Shell`, 節点数3以上）: ρ·t·(A−開口面積)·g＋開口重量
///   （§壁自重）を全頂点に等分配。三方スリット壁は最上位標高の頂点へ全量集中
///   （マニュアル「壁に三方スリットが指定されている場合、壁荷重は全て上部の大梁に伝達」）。
///   §1.2: マニュアル「壁の重量を階高の中央で上下階の節点に分配」に対応
///   （矩形壁なら上下2節点ずつに1/4ずつ配分される）。
///   §壁自重: 4 節点の耐震壁は「周辺の柱梁の内法寸法」で面積を評価する
///   （[`wall_clear_area_factor`]。芯々面積に内法係数を乗じる。控除相手の
///   柱・梁が見つからない辺は控除なし＝芯々のまま保守側）。
/// - ダンパー（`load_cfg.dampers` に登録された Beam/Brace 要素）: 断面自重
///   （ρ·A·L·g）は使わず、装置重量＋支持部重量に置き換える（§ダンパー自重。
///   `device_weight=0` かつ `support_area>0` の場合は支持部のみが算入され、
///   マニュアル「自重を考慮しない部材」に対応する）。
pub(crate) fn enumerate_self_weight(model: &Model, load_cfg: &LoadCfg) -> Vec<SelfWeightItem> {
    let mut items = Vec::new();
    for (elem_idx, elem) in model.elements.iter().enumerate() {
        // ダンパー自重（§ダンパー自重）: 対象部材は断面からの自重計算をスキップし、
        // 装置重量＋支持部断面積×(節点間距離−装置長さ)×鋼材単位体積重量で置き換える。
        if matches!(elem.kind, ElementKind::Beam | ElementKind::Brace { .. })
            && elem.nodes.len() >= 2
        {
            if let Some(damper) = load_cfg.dampers.iter().find(|d| d.elem == elem.id) {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let len = dist3(model.nodes[ni].coord, model.nodes[nj].coord);
                let support_len = (len - damper.device_length).max(0.0);
                let w = damper.device_weight
                    + damper.support_area * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
                items.push(SelfWeightItem::Damper { ni, nj, total: w });
                continue;
            }
        }

        let (Some(sec_id), Some(mat_id)) = (elem.section, elem.material) else {
            continue;
        };
        let (Some(sec), Some(mat)) = (
            model.sections.get(sec_id.index()),
            model.materials.get(mat_id.index()),
        ) else {
            continue;
        };

        match elem.kind {
            ElementKind::Beam | ElementKind::Brace { .. } if elem.nodes.len() >= 2 => {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
                let len = dist3(ci, cj);
                let is_vertical = is_vertical_pair(ci, cj);
                let is_concrete = mat.fc.is_some();
                // §1.8: 柱面間距離の控除は水平材（梁）のみ。鉛直材（柱）は
                // 床上面から床上面（＝節点間距離）で算定する（マニュアル「柱の長さ」）。
                let mut eff_len = if is_concrete && !is_vertical {
                    (len - elem.rigid_zone.face_i - elem.rigid_zone.face_j).max(0.0)
                } else {
                    len
                };

                // §柱の長さ: コンクリート造の柱で、下端節点から下に続く柱が無い場合、
                // 下端節点に取り付く梁（非鉛直 Beam）の最大せいを自重算定長へ加算する。
                if is_concrete && is_vertical {
                    let bottom_local = if ci[2] <= cj[2] { 0 } else { 1 };
                    let bottom_id = elem.nodes[bottom_local];
                    let bottom_z = model.nodes[bottom_id.index()].coord[2];
                    let has_column_below = model.elements.iter().any(|e2| {
                        e2.id != elem.id
                            && matches!(e2.kind, ElementKind::Beam | ElementKind::Brace { .. })
                            && e2.nodes.len() >= 2
                            && e2.nodes.contains(&bottom_id)
                            && {
                                let (a, b) = (
                                    model.nodes[e2.nodes[0].index()].coord,
                                    model.nodes[e2.nodes[1].index()].coord,
                                );
                                is_vertical_pair(a, b) && {
                                    let other = if e2.nodes[0] == bottom_id { b } else { a };
                                    other[2] < bottom_z - LEVEL_TOL_MM
                                }
                            }
                    });
                    if !has_column_below {
                        let max_depth = model
                            .elements
                            .iter()
                            .filter(|e2| {
                                e2.kind == ElementKind::Beam
                                    && e2.id != elem.id
                                    && e2.nodes.len() >= 2
                                    && e2.nodes.contains(&bottom_id)
                            })
                            .filter_map(|e2| {
                                let (a, b) = (
                                    model.nodes[e2.nodes[0].index()].coord,
                                    model.nodes[e2.nodes[1].index()].coord,
                                );
                                if is_vertical_pair(a, b) {
                                    None
                                } else {
                                    e2.section
                                        .and_then(|sid| model.sections.get(sid.index()))
                                        .map(|s| s.depth)
                                }
                            })
                            .fold(0.0_f64, f64::max);
                        eff_len += max_depth;
                    }
                }

                let factor = if is_concrete {
                    1.0
                } else {
                    load_cfg.effective_steel_factor()
                };
                // §1.9: RC/SRC 梁（水平材）はスラブ厚分の断面積 b·t を控除する
                // （w_c = γ·b(D−t)+…。スラブ重量が構造芯間の面積で別途計上される
                // ための二重計上防止）。スラブが無いモデルでは控除しない。
                let self_weight_area = if is_concrete
                    && !is_vertical
                    && model.slab_thickness > 0.0
                    && !model.slabs.is_empty()
                {
                    (sec.area - sec.width * model.slab_thickness.min(sec.depth)).max(0.0)
                } else {
                    sec.area
                };
                let mut w = mat.density * self_weight_area * eff_len * GRAVITY_MM_S2 * factor;
                if let Some(&(_, lw)) = load_cfg
                    .extra_line_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    w += lw * eff_len;
                }
                // §仕上げ荷重の自動換算: w_f × 仕上げ周長 φ を自重算定長に乗じて加算する。
                if let Some(&(_, wf)) = load_cfg
                    .finish_area_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    let phi = finish_perimeter(sec.width, sec.depth, is_vertical);
                    w += wf * phi * eff_len;
                }

                items.push(SelfWeightItem::Line { elem_idx, total: w });
            }
            ElementKind::Wall | ElementKind::Shell if elem.nodes.len() >= 3 => {
                let Some(t) = sec.thickness else {
                    continue;
                };
                let pts: Vec<[f64; 3]> = elem
                    .nodes
                    .iter()
                    .map(|n| model.nodes[n.index()].coord)
                    .collect();
                // §壁自重: 耐震壁は周辺柱梁の内法寸法で面積を評価する。
                let area = polygon_area_3d(&pts) * wall_clear_area_factor(model, elem, &pts);

                // §壁自重: 開口控除・開口重量。三方スリットは全量を最上位標高の頂点へ。
                let attr = model.wall_attrs.iter().find(|a| a.elem == elem.id);
                let opening_area = attr.map(|a| a.total_opening_area()).unwrap_or(0.0);
                let opening_weight = attr.map(|a| a.opening_weight).unwrap_or(0.0);
                let three_side_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);
                let net_area = (area - opening_area).max(0.0);
                let w = (mat.density * t * net_area * GRAVITY_MM_S2 + opening_weight).max(0.0);

                let shares = if three_side_slit {
                    // 壁荷重は全て上部の節点（頂点のうち標高最大のもの。同率上位は等分）へ。
                    let max_z = pts.iter().map(|p| p[2]).fold(f64::MIN, f64::max);
                    let top_indices: Vec<usize> = pts
                        .iter()
                        .enumerate()
                        .filter(|(_, p)| (p[2] - max_z).abs() < LEVEL_TOL_MM)
                        .map(|(i, _)| i)
                        .collect();
                    let share = w / top_indices.len() as f64;
                    top_indices
                        .into_iter()
                        .map(|i| (elem.nodes[i].index(), share))
                        .collect()
                } else {
                    let share = w / pts.len() as f64;
                    elem.nodes.iter().map(|n| (n.index(), share)).collect()
                };
                items.push(SelfWeightItem::Panel { shares });
            }
            _ => {}
        }
    }
    items
}

/// 耐震壁の自重面積算定用の**内法係数**（芯々面積に乗じる係数、(0,1]）。
///
/// マニュアル §壁自重「耐震壁の重量は周辺の柱梁の内法寸法で計算します」に対応。
/// 対象は 4 節点の `ElementKind::Wall` のみ（シェル床・多角形壁は 1.0）。
/// 各辺を鉛直辺（側柱候補）・水平辺（上下梁候補）に分類し、辺の節点対に一致する
/// 線材（`ElementKind::Beam`）の断面寸法の半分を芯々寸法から控除する:
/// - 水平辺（上下梁）: 梁せい `sec.depth` の半分を高さから控除
/// - 鉛直辺（側柱）: 平面内の向きが特定できないため `min(width, depth)` の半分を
///   長さから控除（控除を小さくとる保守側の近似）
///
/// ハンチ・セットバック等で柱梁が斜めの場合の個別考慮は行わない。控除相手の
/// 部材が見つからない辺は控除なし（芯々のまま＝保守側）。
fn wall_clear_area_factor(model: &Model, elem: &ElementData, pts: &[[f64; 3]]) -> f64 {
    if elem.kind != ElementKind::Wall || elem.nodes.len() != 4 || pts.len() != 4 {
        return 1.0;
    }
    let n = 4usize;
    let mut l_len = 0.0; // 水平辺（芯々長さ）の合計
    let mut l_cnt = 0u32;
    let mut h_len = 0.0; // 鉛直辺（芯々高さ）の合計
    let mut h_cnt = 0u32;
    let mut l_deduct = 0.0; // 側柱の半幅の和（長さ方向の控除）
    let mut h_deduct = 0.0; // 上下梁の半せいの和（高さ方向の控除）
    for i in 0..n {
        let (a, b) = (elem.nodes[i], elem.nodes[(i + 1) % n]);
        let (pa, pb) = (pts[i], pts[(i + 1) % n]);
        let dz = (pb[2] - pa[2]).abs();
        let dh = ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2)).sqrt();
        let len = (dz * dz + dh * dh).sqrt();
        if len <= 0.0 {
            continue;
        }
        // 辺の節点対に一致する線材（柱・梁）の断面。
        let member_sec = model
            .elements
            .iter()
            .find(|e| {
                e.kind == ElementKind::Beam && e.nodes.len() >= 2 && {
                    let (m0, m1) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
                    (m0 == a && m1 == b) || (m0 == b && m1 == a)
                }
            })
            .and_then(|e| e.section)
            .and_then(|sid| model.sections.get(sid.index()));
        if dz > dh {
            // 鉛直辺 = 側柱候補
            h_len += len;
            h_cnt += 1;
            if let Some(sec) = member_sec {
                l_deduct += sec.width.min(sec.depth).max(0.0) / 2.0;
            }
        } else {
            // 水平辺 = 上下梁候補
            l_len += len;
            l_cnt += 1;
            if let Some(sec) = member_sec {
                h_deduct += sec.depth.max(0.0) / 2.0;
            }
        }
    }
    if l_cnt == 0 || h_cnt == 0 {
        return 1.0;
    }
    let l = l_len / l_cnt as f64;
    let h = h_len / h_cnt as f64;
    if l <= 0.0 || h <= 0.0 {
        return 1.0;
    }
    let fl = ((l - l_deduct) / l).clamp(0.0, 1.0);
    let fh = ((h - h_deduct) / h).clamp(0.0, 1.0);
    (fl * fh).clamp(0.0, 1.0)
}

/// モデル全節点のうち、指定点に最も近い節点。
fn nearest_node(model: &Model, pt: [f64; 3]) -> Option<NodeId> {
    model
        .nodes
        .iter()
        .min_by(|a, b| dist3(a.coord, pt).partial_cmp(&dist3(b.coord, pt)).unwrap())
        .map(|n| n.id)
}

/// 「柱要素」（鉛直な `Beam` 要素）のうち、指定点に最も近い節点を持つ要素。
/// §フレーム外雑壁の「柱」伝達タイプに用いる（最近接の柱節点→その柱要素の上下節点）。
fn nearest_column_element(model: &Model, pt: [f64; 3]) -> Option<&ElementData> {
    let mut best: Option<(&ElementData, f64)> = None;
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() < 2 {
            continue;
        }
        let (a, b) = (
            model.nodes[e.nodes[0].index()].coord,
            model.nodes[e.nodes[1].index()].coord,
        );
        if !is_vertical_pair(a, b) {
            continue;
        }
        let d = dist3(a, pt).min(dist3(b, pt));
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

/// フレーム外雑壁（`Model.misc_walls`）の重量を近傍節点へ集計する（§フレーム外雑壁）。
///
/// 壁を長さ 0.5m（端数含む）ごとの領域に分割し、各領域重量
/// `weight_per_area × height × 領域長` を、領域中心位置（下端線分上の点、
/// 高さ方向に `height/2` を加えた 3D 点）から近い節点へ `transfer` の規則で伝達する。
/// - `Column`: 最も近い柱要素（鉛直 `Beam`）の上下 2 節点へ 1/2 ずつ
///   （柱が見つからない場合はモデル全節点中の最近接節点へ全量）
/// - `Beam`: モデル全節点中の最近接節点へ全量集中
/// - `SelfStanding`: 自立壁の簡易扱い。解析用の専用節点を新設する代わりに
///   モデル全節点中の最近接節点（配置階の代表的な節点）へ全量を伝達する
///   （マニュアルの「配置階の剛床へ伝達」の簡易近似）。
fn accumulate_misc_wall_weight(model: &Model, node_weight: &mut [f64]) {
    for (i, w) in misc_wall_weight_shares(model) {
        node_weight[i] += w;
    }
}

/// [`accumulate_misc_wall_weight`] の配分内容（`model.nodes` 添字 → [N]）を返す。
/// 地震用重量の集計と自重(自動)荷重ケース（[`crate::self_weight`]）で共有する。
pub(crate) fn misc_wall_weight_shares(model: &Model) -> Vec<(usize, f64)> {
    const SEGMENT_LEN: f64 = 500.0;
    let mut shares: Vec<(usize, f64)> = Vec::new();
    for wall in &model.misc_walls {
        let (s, e) = (wall.start, wall.end);
        let (dx, dy, dz) = (e[0] - s[0], e[1] - s[1], e[2] - s[2]);
        let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
        if total_len <= 0.0 {
            continue;
        }
        let mut offset = 0.0;
        while offset < total_len - 1e-9 {
            let seg_len = SEGMENT_LEN.min(total_len - offset);
            let t_center = (offset + seg_len / 2.0) / total_len;
            let center = [
                s[0] + dx * t_center,
                s[1] + dy * t_center,
                s[2] + dz * t_center + wall.height / 2.0,
            ];
            let seg_weight = wall.weight_per_area * wall.height * seg_len;

            match wall.transfer {
                MiscWallTransfer::Column => {
                    if let Some(col) = nearest_column_element(model, center) {
                        let ni = col.nodes[0].index();
                        let nj = col.nodes[1].index();
                        shares.push((ni, seg_weight / 2.0));
                        shares.push((nj, seg_weight / 2.0));
                    } else if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
                MiscWallTransfer::Beam | MiscWallTransfer::SelfStanding => {
                    if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
            }

            offset += SEGMENT_LEN;
        }
    }
    shares
}

/// 単純支持梁（節点間距離 `len` を支間とする静定梁）の等価節点重量（両端反力）。
///
/// §1.4: マニュアル「地震荷重の計算」は地震用節点重量を「大梁の CMoQo の計算で
/// 求めた梁せん断力（＝ Q0、単純梁反力）」と定義する。単純梁の反力は集中荷重・
/// 分布荷重いずれも静定なので、両端 1/2 の一律配分ではなく荷重位置に応じた
/// 反力比で配分する（対称荷重では結果的に 1/2 ずつになる）。
///
/// - 集中荷重 `Point{a,p}`: `R_i = p(L-a)/L`, `R_j = p·a/L`
/// - 分布荷重 `Distributed{a,b,w1,w2}`: 合計 `W=(w1+w2)/2·(b-a)`、
///   重心位置 `x̄ = a + (b-a)(w1+2w2)/(3(w1+w2))`（`w1+w2=0` は区間中点）、
///   `R_j = W·x̄/L`, `R_i = W - R_j`
///
/// 戻り値は `(R_i, R_j)`。`len <= 0` は `(0, 0)`。
fn static_reactions(kind: &MemberLoadKind, len: f64) -> (f64, f64) {
    if len <= 0.0 {
        return (0.0, 0.0);
    }
    match *kind {
        MemberLoadKind::Point { a, p } => {
            let a = a.clamp(0.0, len);
            let ri = p * (len - a) / len;
            let rj = p * a / len;
            (ri, rj)
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            let a = a.max(0.0);
            let b = b.min(len);
            if b <= a {
                return (0.0, 0.0);
            }
            let span = b - a;
            let total = (w1 + w2) / 2.0 * span;
            let sum_w = w1 + w2;
            let xbar = if sum_w.abs() < 1e-12 {
                a + span / 2.0
            } else {
                a + span * (w1 + 2.0 * w2) / (3.0 * sum_w)
            };
            let rj = total * xbar / len;
            let ri = total - rj;
            (ri, rj)
        }
    }
}

/// 節点 Z 座標から階を自動生成する（複数の重力荷重ケースを地震用重量に算入する版）。
///
/// - 最下レベルは基部(支点レベル)とみなし階に含めない
/// - 前回生成した剛床代表節点（`model.generated_masters`）は構造節点の
///   クラスタリング対象から除外する（再生成時に過去の代表節点を混ぜない）
/// - 各階の剛床代表節点は、その階の全構造節点の慣性力重心（重量重み付き重心）に
///   新規生成する（既存の `generated_masters` があれば座標・拘束・所属階を更新して再利用）
/// - `gravity_lcs` に指定した各ケースの鉛直下向き荷重を地震重量に算入する
///   （自重は材料密度から常に算入）。重複 ID は 1 回だけ処理する
///   （固定荷重＋地震用積載荷重など複数ケースの合算に対応する下準備）。
pub fn generate_stories_multi(
    model: &Model,
    gravity_lcs: &[LoadCaseId],
) -> Result<StoryGenResult, String> {
    if model.nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 0. 構造節点の抽出（前回生成分の剛床代表節点を除外） ---
    let generated: std::collections::HashSet<NodeId> =
        model.generated_masters.iter().copied().collect();
    let struct_nodes: Vec<&Node> = model
        .nodes
        .iter()
        .filter(|n| !generated.contains(&n.id))
        .collect();
    if struct_nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 1. Z レベルのクラスタリング（構造節点のみ） ---
    let mut zs: Vec<f64> = struct_nodes.iter().map(|n| n.coord[2]).collect();
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut levels: Vec<f64> = Vec::new();
    for z in zs {
        match levels.last() {
            Some(&last) if (z - last).abs() <= LEVEL_TOL_MM => {}
            _ => levels.push(z),
        }
    }
    if levels.len() < 2 {
        return Err(
            "節点の標高(Z)が 1 レベルしかありません。階を生成するには 2 レベル以上必要です。"
                .into(),
        );
    }

    let level_of = |z: f64| -> usize {
        levels
            .iter()
            .position(|&l| (z - l).abs() <= LEVEL_TOL_MM)
            .unwrap_or(0)
    };

    // --- 2. 節点の重量配分 ---
    let mut node_weight = vec![0.0f64; model.nodes.len()];
    let load_cfg = model.load_cfg.clone().unwrap_or_default();

    // K型ブレースの重量配分（§K型ブレース）に用いる「基準節点」判定。
    // 基準節点＝ Brace 以外の要素が 1 つでも接続する節点。それ以外は「内部節点」。
    let mut is_base_node = vec![false; model.nodes.len()];
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Brace { .. }) {
            for n in &e.nodes {
                if let Some(slot) = is_base_node.get_mut(n.index()) {
                    *slot = true;
                }
            }
        }
    }

    // 自重（算定規則は enumerate_self_weight に一元化。§柱梁自重・§壁自重・§ダンパー自重）。
    // - 線材: 総重量を両端に半分ずつ（対称等分布荷重の静定反力。
    //   K型ブレースは §K型ブレースの規則で再配分）。
    // - ダンパー: 両端節点へ 1/2 ずつ（鉛直配置は上下階へ、水平配置は同一階の
    //   両節点へ、が節点標高から自然に成立する）。
    // - 壁・シェル: 頂点配分（三方スリットは最上位標高の頂点へ全量）。
    for item in enumerate_self_weight(model, &load_cfg) {
        match item {
            SelfWeightItem::Damper { ni, nj, total } => {
                node_weight[ni] += total / 2.0;
                node_weight[nj] += total / 2.0;
            }
            SelfWeightItem::Line { elem_idx, total } => {
                let elem = &model.elements[elem_idx];
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                if matches!(elem.kind, ElementKind::Brace { .. })
                    && load_cfg.k_brace_rule == KBraceWeightRule::BaseNodesOnly
                {
                    // §K型ブレース: 基準節点のみへ配分。両端とも基準節点は 1/2 ずつ、
                    // 片端が内部節点ならその分も基準節点側へ全量、両端とも内部節点は
                    // フォールバックで従来どおり 1/2 ずつ。
                    match (is_base_node[ni], is_base_node[nj]) {
                        (true, false) => node_weight[ni] += total,
                        (false, true) => node_weight[nj] += total,
                        (true, true) | (false, false) => {
                            node_weight[ni] += total / 2.0;
                            node_weight[nj] += total / 2.0;
                        }
                    }
                } else {
                    node_weight[ni] += total / 2.0;
                    node_weight[nj] += total / 2.0;
                }
            }
            SelfWeightItem::Panel { shares } => {
                for (i, w) in shares {
                    node_weight[i] += w;
                }
            }
        }
    }

    // §フレーム外雑壁: 部材としてモデル化しない壁の重量を近傍節点へ集計する。
    accumulate_misc_wall_weight(model, &mut node_weight);

    // 指定荷重ケース（複数可）の鉛直下向き成分。
    // §1.4: 部材荷重は単純梁の静定反力（`static_reactions`）で両端に配分する
    // （マニュアル「地震用節点重量 = 大梁の CMoQo 計算による梁せん断力 Q0」）。
    let mut seen_lcs: std::collections::HashSet<LoadCaseId> = std::collections::HashSet::new();
    for &lc_id in gravity_lcs {
        if !seen_lcs.insert(lc_id) {
            continue;
        }
        let Some(lc) = model.load_cases.iter().find(|c| c.id == lc_id) else {
            continue;
        };
        for nl in &lc.nodal {
            if nl.values[2] < 0.0 {
                node_weight[nl.node.index()] += -nl.values[2];
            }
        }
        for ml in &lc.member {
            let Some(elem) = model
                .elements
                .iter()
                .find(|e| e.id == ml.elem)
                .filter(|e| e.nodes.len() >= 2)
            else {
                continue;
            };
            // 全体座標系の作用方向（正規化済み）の鉛直下向き成分
            let dz = ml.dir[2];
            if dz >= 0.0 {
                continue;
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
            let len = ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2))
                .sqrt();
            let (ri, rj) = static_reactions(&ml.kind, len);
            let scale = -dz;
            node_weight[ni] += ri * scale;
            node_weight[nj] += rj * scale;
        }
    }

    // --- 3. 階の構築（レベル 1 以上、下から順） ---
    let mut stories = Vec::new();
    let mut node_story = vec![None; model.nodes.len()];
    let mut constraints = Vec::new();
    let mut rep_nodes: Vec<Node> = Vec::new();
    let mut generated_masters: Vec<NodeId> = Vec::new();

    // 既存の代表節点は昇順（下の階から順）に再利用し、足りない分は末尾連番で新規生成する。
    let mut reuse_masters = model.generated_masters.iter().copied();
    let mut next_new_id = model.nodes.len() as u32;

    // 剛床代表節点の拘束: 要素が接続しない浮遊節点のため、剛床が拘束しない
    // 3 自由度（Uz, Rx, Ry）を固定しないと特異行列になる。Ux, Uy, Rz は自由。
    let mut rep_restraint = Dof6Mask::FREE;
    rep_restraint.set_fixed(Dof::Uz);
    rep_restraint.set_fixed(Dof::Rx);
    rep_restraint.set_fixed(Dof::Ry);

    for (si, &elev) in levels.iter().enumerate().skip(1) {
        let story_id = StoryId((si - 1) as u32);
        let node_ids: Vec<NodeId> = struct_nodes
            .iter()
            .filter(|n| level_of(n.coord[2]) == si)
            .map(|n| n.id)
            .collect();
        if node_ids.is_empty() {
            continue;
        }

        let weight: f64 = node_ids.iter().map(|n| node_weight[n.index()]).sum();

        // 慣性力重心（重量重み付き重心）。重量が算定できない場合は幾何重心へフォールバック。
        let (gx, gy) = if weight > 0.0 {
            let gx = node_ids
                .iter()
                .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[0])
                .sum::<f64>()
                / weight;
            let gy = node_ids
                .iter()
                .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[1])
                .sum::<f64>()
                / weight;
            (gx, gy)
        } else {
            let gx = node_ids
                .iter()
                .map(|n| model.nodes[n.index()].coord[0])
                .sum::<f64>()
                / node_ids.len() as f64;
            let gy = node_ids
                .iter()
                .map(|n| model.nodes[n.index()].coord[1])
                .sum::<f64>()
                / node_ids.len() as f64;
            (gx, gy)
        };

        // 剛床代表節点（慣性力重心に置く専用の仮想節点）の生成/再利用。
        let master = reuse_masters.next().unwrap_or_else(|| {
            let id = NodeId(next_new_id);
            next_new_id += 1;
            id
        });
        rep_nodes.push(Node {
            id: master,
            coord: [gx, gy, elev],
            restraint: rep_restraint,
            mass: None,
            story: Some(story_id),
        });
        generated_masters.push(master);

        // 当該階の全構造節点がスレーブ（マスターは専用節点のため、既存節点は 1 点でも全てスレーブ）。
        let slaves = node_ids.clone();

        for n in &node_ids {
            node_story[n.index()] = Some(story_id);
        }
        constraints.push(Constraint::RigidDiaphragm {
            story: story_id,
            master,
            slaves: slaves.clone(),
        });
        stories.push(Story {
            id: story_id,
            name: format!("{}F", si),
            elevation: elev,
            node_ids,
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                master,
                slaves,
                rigid: true,
                weight: Some(weight),
            }],
            seismic_weight: Some(weight),
            structure: Default::default(),
            level_kind: Default::default(),
        });
    }

    if stories.is_empty() {
        return Err("階を構成する節点が見つかりませんでした。".into());
    }

    // 階数が減って余った旧代表節点は不活性化する（拘束固定・所属階なし）が、
    // `generated_masters` には残して次回再生成時に再利用できるようにする。
    for id in reuse_masters {
        rep_nodes.push(Node {
            id,
            coord: model.nodes[id.index()].coord,
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        generated_masters.push(id);
    }

    Ok(StoryGenResult {
        stories,
        node_story,
        constraints,
        rep_nodes,
        generated_masters,
    })
}

/// 節点 Z 座標から階を自動生成する（重力荷重ケース単一指定・従来互換の薄いラッパー）。
///
/// 詳細は [`generate_stories_multi`] を参照。`gravity_lc` を `Some` で渡した場合は
/// その 1 ケースのみを地震用重量に算入する（`None` は自重のみ）。
pub fn generate_stories(
    model: &Model,
    gravity_lc: Option<LoadCaseId>,
) -> Result<StoryGenResult, String> {
    let lcs: Vec<LoadCaseId> = gravity_lc.into_iter().collect();
    generate_stories_multi(model, &lcs)
}

#[cfg(test)]
mod tests;
