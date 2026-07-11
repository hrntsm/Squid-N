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
    Constraint, DiaphragmDef, ElementData, ElementKind, KBraceWeightRule, MemberLoadKind,
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

/// 仕上げ周長 φ（RESP-D マニュアル「柱梁自重」の仕上げ荷重）。
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
    const SEGMENT_LEN: f64 = 500.0;
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
                        node_weight[ni] += seg_weight / 2.0;
                        node_weight[nj] += seg_weight / 2.0;
                    } else if let Some(n) = nearest_node(model, center) {
                        node_weight[n.index()] += seg_weight;
                    }
                }
                MiscWallTransfer::Beam | MiscWallTransfer::SelfStanding => {
                    if let Some(n) = nearest_node(model, center) {
                        node_weight[n.index()] += seg_weight;
                    }
                }
            }

            offset += SEGMENT_LEN;
        }
    }
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

    // 自重。
    // - 線材（柱・梁・ブレース, `ElementKind::Beam`/`Brace`）: ρ·A·L·g を両端に半分ずつ
    //   （対称等分布荷重の静定反力。K型ブレースは §K型ブレースの規則で再配分）。
    //   §1.8: 自重算定長 L は、コンクリート材（`mat.fc` あり = RC/SRC 梁）は柱面間距離
    //   （`len - face_i - face_j`、負にならない範囲）、鋼材（S 梁）は従来どおり節点間距離
    //   （マニュアル「柱梁自重」：RC/SRC 大梁は柱面間距離、S 梁は節点間距離）。
    //   §柱の長さ: コンクリート柱（鉛直材）で下端節点に別の柱（鉛直 Beam/Brace）が
    //   下から接続していない場合、下端節点に取り付く梁（非鉛直 Beam）の最大せいを
    //   自重算定長へ加算する（マニュアル「下階に柱がない場合...柱脚に取付く梁の最大せいの
    //   長さを柱長さに付加」）。
    //   ギャップ対応: 鋼材のみ `load_cfg.effective_steel_factor()`（鉄骨重量割増率）を乗じ、
    //   `load_cfg.extra_line_weight`（耐火被覆等の付加線重量 [N/mm]）・
    //   `load_cfg.finish_area_weight`（仕上げ面重量 w_f、周長 φ から自動換算）が
    //   あれば自重算定長を掛けて加算する（両端配分は自重と同じ経路）。
    // - 壁・シェル（`ElementKind::Wall`/`Shell`, 節点数3以上）: ρ·t·(A−開口面積)·g＋開口重量
    //   （§壁自重）を全頂点に等分配。三方スリット壁は最上位標高の頂点へ全量集中。
    //   §1.2: マニュアル「壁の重量を階高の中央で上下階の節点に分配」に対応
    //   （矩形壁なら上下2節点ずつに1/4ずつ配分される）。
    // - ダンパー（`load_cfg.dampers` に登録された Beam/Brace 要素）: 断面自重
    //   （ρ·A·L·g）は使わず、装置重量＋支持部重量に置き換える（§ダンパー自重）。
    for elem in &model.elements {
        // ダンパー自重（§ダンパー自重）: 対象部材は断面からの自重計算をスキップし、
        // 装置重量＋支持部断面積×(節点間距離−装置長さ)×鋼材単位体積重量で置き換える。
        // 両端節点へ 1/2 ずつ（鉛直配置は上下階へ、水平配置は同一階の両節点へ、が
        // 節点標高から自然に成立する）。`device_weight=0` かつ `support_area>0` の
        // 場合は支持部のみが算入される（マニュアル「自重を考慮しない部材」に対応）。
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
                node_weight[ni] += w / 2.0;
                node_weight[nj] += w / 2.0;
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
                let mut eff_len = if is_concrete {
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
                let mut w = mat.density * sec.area * eff_len * GRAVITY_MM_S2 * factor;
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

                if matches!(elem.kind, ElementKind::Brace { .. })
                    && load_cfg.k_brace_rule == KBraceWeightRule::BaseNodesOnly
                {
                    // §K型ブレース: 基準節点のみへ配分。両端とも基準節点は 1/2 ずつ、
                    // 片端が内部節点ならその分も基準節点側へ全量、両端とも内部節点は
                    // フォールバックで従来どおり 1/2 ずつ。
                    match (is_base_node[ni], is_base_node[nj]) {
                        (true, false) => node_weight[ni] += w,
                        (false, true) => node_weight[nj] += w,
                        (true, true) | (false, false) => {
                            node_weight[ni] += w / 2.0;
                            node_weight[nj] += w / 2.0;
                        }
                    }
                } else {
                    node_weight[ni] += w / 2.0;
                    node_weight[nj] += w / 2.0;
                }
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
                let area = polygon_area_3d(&pts);

                // §壁自重: 開口控除・開口重量。三方スリットは全量を最上位標高の頂点へ。
                let attr = model.wall_attrs.iter().find(|a| a.elem == elem.id);
                let opening_area = attr.map(|a| a.total_opening_area()).unwrap_or(0.0);
                let opening_weight = attr.map(|a| a.opening_weight).unwrap_or(0.0);
                let three_side_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);
                let net_area = (area - opening_area).max(0.0);
                let w = (mat.density * t * net_area * GRAVITY_MM_S2 + opening_weight).max(0.0);

                if three_side_slit {
                    // 壁荷重は全て上部の節点（頂点のうち標高最大のもの。同率上位は等分）へ。
                    let max_z = pts.iter().map(|p| p[2]).fold(f64::MIN, f64::max);
                    let top_indices: Vec<usize> = pts
                        .iter()
                        .enumerate()
                        .filter(|(_, p)| (p[2] - max_z).abs() < LEVEL_TOL_MM)
                        .map(|(i, _)| i)
                        .collect();
                    let share = w / top_indices.len() as f64;
                    for i in top_indices {
                        node_weight[elem.nodes[i].index()] += share;
                    }
                } else {
                    let share = w / pts.len() as f64;
                    for n in &elem.nodes {
                        node_weight[n.index()] += share;
                    }
                }
            }
            _ => {}
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
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        DamperSpec, ElementData, EndCondition, ForceRegime, LoadCase, LoadCfg, LocalAxis, Material,
        MemberLoad, MiscWall, MiscWallTransfer, NodalLoad, Node, RigidZone, Section, WallAttr,
    };

    /// 2 層 × 1 スパンの平面ラーメン（各レベル 2 節点）。
    fn two_story_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
            [0.0, 0.0, 7000.0],
            [6000.0, 0.0, 7000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        // 柱4 + 梁2
        let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
        for (i, (a, b)) in conn.iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![NodalLoad {
                node: NodeId(4),
                values: [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
            }],
            member: vec![MemberLoad {
                elem: ElemId(4),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 6000.0,
                    w1: 10.0,
                    w2: 10.0,
                },
            }],
        });
        model
    }

    #[test]
    fn test_generate_two_stories() {
        let model = two_story_model();
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        assert_eq!(gen.stories.len(), 2);
        assert_eq!(gen.stories[0].elevation, 3500.0);
        assert_eq!(gen.stories[1].elevation, 7000.0);
        // 各階 2 節点 → 代表節点(慣性力重心)を新規生成 + スレーブ2（既存節点は全てスレーブ）
        assert_eq!(gen.stories[0].node_ids.len(), 2);
        assert_eq!(gen.stories[0].diaphragms[0].slaves.len(), 2);
        assert_eq!(gen.stories[1].diaphragms[0].slaves.len(), 2);
        assert_eq!(gen.constraints.len(), 2);
        // 基部節点は無所属
        assert_eq!(gen.node_story[0], None);
        assert_eq!(gen.node_story[2], Some(StoryId(0)));
        assert_eq!(gen.node_story[4], Some(StoryId(1)));
        // 重量: 1F = 梁分布荷重 10 N/mm × 6000 = 60 kN + 自重、2F = 節点荷重 50 kN + 自重
        let w1 = gen.stories[0].seismic_weight.unwrap();
        let w2 = gen.stories[1].seismic_weight.unwrap();
        assert!(w1 > 60000.0, "w1={}", w1);
        assert!(w2 > 50000.0, "w2={}", w2);

        // 代表節点は新規生成（既存節点数=6 の末尾連番）。
        assert_eq!(gen.rep_nodes.len(), 2);
        assert_eq!(gen.generated_masters, vec![NodeId(6), NodeId(7)]);
        for rep in &gen.rep_nodes {
            assert_eq!(rep.mass, None, "質量は Reducer 側の TᵀMT 縮約に委ねる");
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Uz));
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Rx));
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Ry));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));
        }
        assert_eq!(gen.rep_nodes[0].story, Some(StoryId(0)));
        assert_eq!(gen.rep_nodes[1].story, Some(StoryId(1)));
        // 1F は左右対称な自重＋分布荷重のみなので慣性力重心の X は中央(3000)になる。
        assert!((gen.rep_nodes[0].coord[0] - 3000.0).abs() < 1e-6);
        // 2F は節点荷重(50kN)が NodeId(4)(x=0)側のみに掛かる非対称配置なので、
        // 慣性力重心は x=0 側へ偏る(単純な幾何重心 3000 とは一致しない)。
        // 手計算(g=9806.65, §1.11): nw4=53656.6546..., nw5=3656.6546...,
        // gx = nw5*6000/(nw4+nw5) = 382.806855936086
        assert!(
            (gen.rep_nodes[1].coord[0] - 382.806855936086).abs() < 1e-6,
            "{}",
            gen.rep_nodes[1].coord[0]
        );
        assert_eq!(gen.rep_nodes[0].coord[2], 3500.0);
        assert_eq!(gen.rep_nodes[1].coord[2], 7000.0);
    }

    #[test]
    fn test_generate_single_level_is_error() {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        assert!(generate_stories(&model, None).is_err());
    }

    /// 重量が非対称な 1 層モデル（自重なし・節点荷重のみで重みを制御）。
    fn asymmetric_weight_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [4000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![
                NodalLoad {
                    node: NodeId(2),
                    values: [0.0, 0.0, -100000.0, 0.0, 0.0, 0.0],
                },
                NodalLoad {
                    node: NodeId(3),
                    values: [0.0, 0.0, -300000.0, 0.0, 0.0, 0.0],
                },
            ],
            member: vec![],
        });
        model
    }

    #[test]
    fn test_generate_weighted_centroid_matches_hand_calc() {
        let model = asymmetric_weight_model();
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        assert_eq!(gen.stories.len(), 1);
        let story = &gen.stories[0];
        // 重量は自重なし・節点荷重のみ: 100kN + 300kN = 400kN
        assert_eq!(story.seismic_weight, Some(400000.0));
        assert_eq!(
            story.diaphragms[0].slaves.len(),
            2,
            "既存節点は全てスレーブ"
        );

        // 手計算: Gx = Σ(iW·ix)/ΣiW = (100000*0 + 300000*4000) / 400000 = 3000
        assert_eq!(gen.rep_nodes.len(), 1);
        let rep = &gen.rep_nodes[0];
        assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
        assert!((rep.coord[1] - 0.0).abs() < 1e-6, "Gy={}", rep.coord[1]);
        assert_eq!(rep.coord[2], 3000.0);
        assert_eq!(rep.mass, None);
        assert_eq!(rep.story, Some(StoryId(0)));
        assert!(rep.restraint.is_fixed(Dof::Uz));
        assert!(rep.restraint.is_fixed(Dof::Rx));
        assert!(rep.restraint.is_fixed(Dof::Ry));
        assert!(!rep.restraint.is_fixed(Dof::Ux));
        assert!(!rep.restraint.is_fixed(Dof::Uy));
        assert!(!rep.restraint.is_fixed(Dof::Rz));
        // 既存節点数=4 の末尾連番で新規生成される。
        assert_eq!(gen.generated_masters, vec![NodeId(4)]);
    }

    #[test]
    fn test_generate_zero_weight_falls_back_to_geometric_centroid() {
        let mut model = Model::default();
        // 幾何重心が非対称になるよう配置（自重・荷重ケースなし → 重量ゼロ）。
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [6000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        let gen = generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories[0].seismic_weight, Some(0.0));
        let rep = &gen.rep_nodes[0];
        // 幾何重心(単純平均) = (0 + 6000) / 2 = 3000
        assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
    }

    /// 基部(z=0, 固定)と上端(z=`len`, 自由)を結ぶ 1 部材の最小モデル。
    /// 面控除・鉄骨割増率・付加線重量など「単一部材の自重」を検証する各テストの共通土台。
    fn single_beam_model(
        len: f64,
        density: f64,
        area: f64,
        fc: Option<f64>,
        rigid_zone: RigidZone,
        load_cfg: Option<LoadCfg>,
    ) -> Model {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, len],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "M".into(),
            young: 205000.0,
            poisson: 0.3,
            density,
            shear: None,
            fc,
            fy: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone,
            plastic_zone: None,
            spring: None,
        });
        model.load_cfg = load_cfg;
        model
    }

    #[test]
    fn test_static_reactions_point_load_hand_calc() {
        // 単純梁 L=4000, a=1000, p=800: Ri=p(L-a)/L=600, Rj=p*a/L=200
        let (ri, rj) = static_reactions(
            &MemberLoadKind::Point {
                a: 1000.0,
                p: 800.0,
            },
            4000.0,
        );
        assert!((ri - 600.0).abs() < 1e-9, "ri={}", ri);
        assert!((rj - 200.0).abs() < 1e-9, "rj={}", rj);
        assert!((ri + rj - 800.0).abs() < 1e-9);
    }

    #[test]
    fn test_static_reactions_symmetric_distributed_is_half_half() {
        let (ri, rj) = static_reactions(
            &MemberLoadKind::Distributed {
                a: 0.0,
                b: 6000.0,
                w1: 10.0,
                w2: 10.0,
            },
            6000.0,
        );
        assert!((ri - 30000.0).abs() < 1e-9, "ri={}", ri);
        assert!((rj - 30000.0).abs() < 1e-9, "rj={}", rj);
    }

    #[test]
    fn test_static_reactions_asymmetric_distributed_hand_calc() {
        // 三角形分布(w1=0→w2=20)、a=0,b=4000,L=4000。
        // W=(0+20)/2*4000=40000, xbar=4000*(0+40)/(3*20)=2666.666...,
        // Rj=W*xbar/L=26666.666..., Ri=W-Rj=13333.333...
        let (ri, rj) = static_reactions(
            &MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 0.0,
                w2: 20.0,
            },
            4000.0,
        );
        assert!((ri - 13333.333333333334).abs() < 1e-6, "ri={}", ri);
        assert!((rj - 26666.666666666668).abs() < 1e-6, "rj={}", rj);
        assert!((ri + rj - 40000.0).abs() < 1e-6);
    }

    #[test]
    fn test_member_load_reaction_distribution_end_to_end() {
        // 自重を持たない(section/material 未設定)部材に非対称な三角形分布荷重を与え、
        // 剛床代表節点の重心が naive な 1/2-1/2 配分(x=2000)ではなく
        // 静定反力配分による偏った位置(x≈2666.67)になることを確認する（§1.4）。
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [4000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(2), NodeId(3)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![],
            member: vec![MemberLoad {
                elem: ElemId(0),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 4000.0,
                    w1: 0.0,
                    w2: 20.0,
                },
            }],
        });
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        let rep = &gen.rep_nodes[0];
        assert!(
            (rep.coord[0] - 2666.666666666667).abs() < 1e-2,
            "Gx={}",
            rep.coord[0]
        );
    }

    #[test]
    fn test_face_reduction_applies_only_to_concrete() {
        // §1.8: RC/SRC 梁(fc あり)は柱面間距離、S 梁(fc なし)は節点間距離。
        let len = 4000.0;
        let area = 90000.0;
        let density = 2.4e-9;
        let rz = RigidZone {
            face_i: 300.0,
            face_j: 300.0,
            ..Default::default()
        };

        let rc_model = single_beam_model(len, density, area, Some(24.0), rz, None);
        let rc = generate_stories(&rc_model, None).unwrap();
        let eff_len_rc = len - 300.0 - 300.0;
        let expected_rc = density * area * eff_len_rc * GRAVITY_MM_S2 / 2.0;
        assert!(
            (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
            "{}",
            rc.stories[0].seismic_weight.unwrap()
        );

        let s_model = single_beam_model(len, density, area, None, rz, None);
        let s = generate_stories(&s_model, None).unwrap();
        let expected_s = density * area * len * GRAVITY_MM_S2 / 2.0;
        assert!(
            (s.stories[0].seismic_weight.unwrap() - expected_s).abs() < 1e-6,
            "{}",
            s.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_steel_weight_factor_applies_only_to_steel() {
        let len = 4000.0;
        let area = 90000.0;
        let density = 7.85e-9;
        let cfg = LoadCfg {
            live_load_reduction: false,
            dampers: Vec::new(),
            finish_area_weight: Vec::new(),
            k_brace_rule: Default::default(),
            steel_weight_factor: 1.3,
            extra_line_weight: vec![],
        };

        let steel_model = single_beam_model(
            len,
            density,
            area,
            None,
            RigidZone::default(),
            Some(cfg.clone()),
        );
        let steel = generate_stories(&steel_model, None).unwrap();
        let expected_steel = density * area * len * GRAVITY_MM_S2 * 1.3 / 2.0;
        assert!(
            (steel.stories[0].seismic_weight.unwrap() - expected_steel).abs() < 1e-6,
            "{}",
            steel.stories[0].seismic_weight.unwrap()
        );

        let rc_model = single_beam_model(
            len,
            density,
            area,
            Some(24.0),
            RigidZone::default(),
            Some(cfg),
        );
        let rc = generate_stories(&rc_model, None).unwrap();
        let expected_rc = density * area * len * GRAVITY_MM_S2 / 2.0;
        assert!(
            (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
            "割増率はコンクリート材に適用しない: {}",
            rc.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_extra_line_weight_adds_to_self_weight() {
        let len = 4000.0;
        let area = 90000.0;
        let density = 7.85e-9;
        let cfg = LoadCfg {
            live_load_reduction: false,
            dampers: Vec::new(),
            finish_area_weight: Vec::new(),
            k_brace_rule: Default::default(),
            steel_weight_factor: 1.0,
            extra_line_weight: vec![(ElemId(0), 5.0)],
        };
        let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
        let gen = generate_stories(&model, None).unwrap();
        let expected = (density * area * len * GRAVITY_MM_S2 + 5.0 * len) / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    /// 矩形壁(4000×3000, t=150)を上下 2 レベルの節点間に張った 1 層モデル。
    fn wall_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "Wall".into(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: [NodeId(0), NodeId(1), NodeId(2), NodeId(3)]
                .into_iter()
                .collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        model
    }

    #[test]
    fn test_wall_self_weight_included_in_story_weight() {
        // §1.2: 壁自重 w=ρ·t·A·g を全頂点に等分配。
        // 基部(z=0)側 2 節点は階に属さないため、階の地震用重量に算入されるのは
        // 上端 2 節点分(w/2)のみになる。
        let model = wall_model();
        let gen = generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories.len(), 1);
        let area = 4000.0 * 3000.0;
        let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
        let expected = w_total / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_generate_stories_multi_sums_multiple_gravity_cases_and_dedupes() {
        let mut model = asymmetric_weight_model();
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "LL".into(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, -10000.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        });

        // DL(400kN) + LL(10kN) = 410kN
        let gen = generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(1)]).unwrap();
        assert_eq!(gen.stories[0].seismic_weight, Some(410000.0));

        // 重複 ID は 1 回だけ処理される（二重計上しない）
        let gen_dup =
            generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(0), LoadCaseId(1)]).unwrap();
        assert_eq!(gen_dup.stories[0].seismic_weight, Some(410000.0));
    }

    // ------------------------------------------------------------------
    // §壁開口・三方スリット
    // ------------------------------------------------------------------

    #[test]
    fn test_wall_opening_deduction_and_opening_weight() {
        let mut model = wall_model();
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 1_000_000.0,
            opening_weight: 5000.0,
            three_side_slit: false,
            openings: vec![],
        });
        let gen = generate_stories(&model, None).unwrap();
        let area = 4000.0 * 3000.0;
        let net_area = area - 1_000_000.0;
        let w_total = (2.4e-9 * 150.0 * net_area * GRAVITY_MM_S2 + 5000.0).max(0.0);
        let expected = w_total / 2.0; // 上端2節点分(4節点等分の半分)
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_wall_opening_deduction_clamped_non_negative() {
        // 開口面積が壁面積を超える極端な入力でも自重が負にならない(clamp)。
        let mut model = wall_model();
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 4000.0 * 3000.0 * 2.0, // 壁面積を超える
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        let gen = generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories[0].seismic_weight, Some(0.0));
    }

    #[test]
    fn test_wall_three_side_slit_transfers_all_to_top_nodes() {
        // §壁自重: 三方スリットは壁荷重を全て上部の節点へ伝達する。
        // wall_model() の頂点は [z=0, z=0, z=3000, z=3000] の順で、上位2節点は
        // どちらも階に属する(基部でない)ため、通常配分(上端2節点でw_total/2)とは異なり
        // 階の地震用重量は w_total 全量になる。
        let mut model = wall_model();
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: true,
            openings: vec![],
        });
        let gen = generate_stories(&model, None).unwrap();
        let area = 4000.0 * 3000.0;
        let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - w_total).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    // ------------------------------------------------------------------
    // §フレーム外雑壁
    // ------------------------------------------------------------------

    #[test]
    fn test_misc_wall_beam_transfer_conserves_total_weight() {
        // 長さ 1200mm(500+500+200 の端数分割)を Beam タイプで最近接節点へ集中。
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.misc_walls.push(MiscWall {
            start: [-600.0, 0.0, 2900.0],
            end: [600.0, 0.0, 2900.0],
            height: 200.0,
            weight_per_area: 1.0e-3,
            transfer: MiscWallTransfer::Beam,
            thickness: None,
        });
        let gen = generate_stories(&model, None).unwrap();
        // 領域中心の z は 2900+200/2=3000 で node1 に一致し、x も node1(x=0)に
        // node0(距離 3000超)より常に近いため、全量が node1(story0)へ集中する。
        let expected = 1.0e-3 * 200.0 * 1200.0; // weight_per_area * height * 壁長
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_misc_wall_column_transfer_splits_to_column_ends() {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "Col".into(),
            area: 0.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.misc_walls.push(MiscWall {
            start: [-100.0, 0.0, 2900.0],
            end: [100.0, 0.0, 2900.0],
            height: 200.0,
            weight_per_area: 1.0e-3,
            transfer: MiscWallTransfer::Column,
            thickness: None,
        });
        let gen = generate_stories(&model, None).unwrap();
        let total = 1.0e-3 * 200.0 * 200.0;
        // 唯一の柱(node0-node1)へ 1/2 ずつ。node0 は基部で階に属さないため、
        // 階の地震用重量に現れるのは node1 側(w/2)のみ。
        let expected = total / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    // ------------------------------------------------------------------
    // §ダンパー自重
    // ------------------------------------------------------------------

    #[test]
    fn test_damper_weight_replaces_section_self_weight() {
        let len = 4000.0;
        let damper = DamperSpec {
            elem: ElemId(0),
            device_weight: 20000.0,
            device_length: 1000.0,
            support_area: 5000.0,
        };
        let cfg = LoadCfg {
            dampers: vec![damper],
            ..Default::default()
        };
        let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
        let gen = generate_stories(&model, None).unwrap();
        let support_len = (len - 1000.0_f64).max(0.0);
        let w = 20000.0 + 5000.0 * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
        let expected = w / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_damper_zero_device_weight_counts_support_only() {
        // 「自重を考慮しない部材」: device_weight=0 かつ support_area>0 は支持部のみ算入。
        let len = 4000.0;
        let damper = DamperSpec {
            elem: ElemId(0),
            device_weight: 0.0,
            device_length: 500.0,
            support_area: 8000.0,
        };
        let cfg = LoadCfg {
            dampers: vec![damper],
            ..Default::default()
        };
        let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
        let gen = generate_stories(&model, None).unwrap();
        let support_len = (len - 500.0_f64).max(0.0);
        let w = 8000.0 * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
        let expected = w / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
        // 断面自重(ρ·A·L·g)は使われない(桁違いに大きい値とは一致しない)。
        let naive = 7.85e-9 * 90000.0 * len * GRAVITY_MM_S2 / 2.0;
        assert!((gen.stories[0].seismic_weight.unwrap() - naive).abs() > 1.0);
    }

    // ------------------------------------------------------------------
    // §仕上げ面重量の自動換算
    // ------------------------------------------------------------------

    #[test]
    fn test_finish_area_weight_column_perimeter_four_side() {
        // single_beam_model は鉛直材(柱)。φ=2(b+D)、b=D=300 (helper内の断面固定値)。
        let len = 4000.0;
        let area = 90000.0;
        let density = 7.85e-9;
        let wf = 0.002;
        let cfg = LoadCfg {
            finish_area_weight: vec![(ElemId(0), wf)],
            ..Default::default()
        };
        let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
        let gen = generate_stories(&model, None).unwrap();
        let phi = 2.0 * (300.0 + 300.0);
        let expected = (density * area * len * GRAVITY_MM_S2 + wf * phi * len) / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_finish_area_weight_beam_perimeter_three_side() {
        // 水平梁(非鉛直)。φ=b+2D の三面仕上げ。
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [6000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "Beam".into(),
            area: 90000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 600.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        let wf = 0.0015;
        model.load_cfg = Some(LoadCfg {
            finish_area_weight: vec![(ElemId(0), wf)],
            ..Default::default()
        });
        let gen = generate_stories(&model, None).unwrap();
        let len = 6000.0;
        let phi = 300.0 + 2.0 * 600.0;
        // 両端(node1, node2)とも z=3000 の同一階に属するため、全量がその階に現れる。
        let expected = 7.85e-9 * 90000.0 * len * GRAVITY_MM_S2 + wf * phi * len;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    // ------------------------------------------------------------------
    // §柱の長さ(下階柱なし時の柱脚梁せい付加)
    // ------------------------------------------------------------------

    #[test]
    fn test_base_column_without_lower_column_adds_max_beam_depth() {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        }); // 柱脚(下階柱なし) & 梁の一端
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }); // 柱頭
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [4000.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        }); // 梁の他端(基部)
        model.sections.push(Section {
            id: SectionId(0),
            name: "Col".into(),
            area: 90000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.sections.push(Section {
            id: SectionId(1),
            name: "Beam".into(),
            area: 0.0, // 自重寄与ゼロにして柱脚梁せい付加のみを検証する
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 600.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.elements.push(ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
            section: Some(SectionId(1)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        let gen = generate_stories(&model, None).unwrap();
        let eff_len = 3000.0 + 600.0; // 柱長さ + 柱脚に取付く梁の最大せい
        let expected = 2.4e-9 * 90000.0 * eff_len * GRAVITY_MM_S2 / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    #[test]
    fn test_base_column_with_lower_column_does_not_add_beam_depth() {
        // 下階に柱がある場合は梁せいを付加しない(誤って常時付加しないことの回帰確認)。
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, -3000.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        }); // 最下層(基部)
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }); // 1F: 下階に柱(node0-node1)があるので梁せい付加なし
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }); // 2F
        model.nodes.push(Node {
            id: NodeId(3),
            coord: [4000.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        }); // 1F 位置に取付く梁の他端
        model.sections.push(Section {
            id: SectionId(0),
            name: "Col".into(),
            area: 90000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.sections.push(Section {
            id: SectionId(1),
            name: "ColLower".into(),
            area: 0.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.sections.push(Section {
            id: SectionId(2),
            name: "Beam".into(),
            area: 0.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 600.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        // 下階の柱(node0-node1)。area=0 で自重寄与ゼロ(有無の判定のみに使う)。
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(1)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        // 検証対象の柱(node1-node2)。
        model.elements.push(ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        // 1F(node1)に取付く梁(area=0、せい付加の誤検出があれば効いてしまう)。
        model.elements.push(ElementData {
            id: ElemId(2),
            kind: ElementKind::Beam,
            nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
            section: Some(SectionId(2)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        let gen = generate_stories(&model, None).unwrap();
        // story0(1F, node1・node3) には下階柱(area0)/2 + 検証対象柱の下半分 + 梁(area0)/2。
        // 梁せい付加が誤って効いていれば eff_len が 3600 になり期待値からずれる。
        let w_upper = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2; // 梁せい付加なし(eff_len=3000)
        let expected_story0 = w_upper / 2.0;
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - expected_story0).abs() < 1e-6,
            "{}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    // ------------------------------------------------------------------
    // §K型ブレースの重量配分
    // ------------------------------------------------------------------

    /// K型ブレース: 基準節点(node2, node3)から内部節点(node4)へ2本のブレースが
    /// 集まる形。ブレース断面積を非対称にして配分規則による重心の違いを検出する。
    fn k_brace_model(rule: KBraceWeightRule) -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [4000.0, 0.0, 3000.0],
            [2000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        // 柱(自重ゼロ、node2/node3 を「基準節点」化するために存在)
        model.sections.push(Section {
            id: SectionId(0),
            name: "Col".into(),
            area: 0.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        // ブレース1(node2-node4)
        model.sections.push(Section {
            id: SectionId(1),
            name: "Brace1".into(),
            area: 10000.0,
            iy: 1.0e6,
            iz: 1.0e6,
            j: 1.0e6,
            depth: 200.0,
            width: 200.0,
            as_y: 1000.0,
            as_z: 1000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        // ブレース2(node3-node4): 面積を2倍にして非対称にする
        model.sections.push(Section {
            id: SectionId(2),
            name: "Brace2".into(),
            area: 20000.0,
            iy: 1.0e6,
            iz: 1.0e6,
            j: 1.0e6,
            depth: 200.0,
            width: 200.0,
            as_y: 1000.0,
            as_z: 1000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        let axis = LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        };
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: axis,
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.elements.push(ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: axis,
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.elements.push(ElementData {
            id: ElemId(2),
            kind: ElementKind::Brace {
                tension_only: false,
            },
            nodes: [NodeId(2), NodeId(4)].into_iter().collect(),
            section: Some(SectionId(1)),
            material: Some(MaterialId(0)),
            local_axis: axis,
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.elements.push(ElementData {
            id: ElemId(3),
            kind: ElementKind::Brace {
                tension_only: false,
            },
            nodes: [NodeId(3), NodeId(4)].into_iter().collect(),
            section: Some(SectionId(2)),
            material: Some(MaterialId(0)),
            local_axis: axis,
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
        model.load_cfg = Some(LoadCfg {
            k_brace_rule: rule,
            ..Default::default()
        });
        model
    }

    #[test]
    fn test_k_brace_base_nodes_only_shifts_centroid_toward_base_nodes() {
        let density = 7.85e-9;
        let len = 2000.0; // node2-node4, node3-node4 とも水平距離2000
        let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
        let w2 = density * 20000.0 * len * GRAVITY_MM_S2;

        let internal = k_brace_model(KBraceWeightRule::InternalNodes);
        let gen_internal = generate_stories(&internal, None).unwrap();
        // 手計算: node2=w1/2(x=0), node3=w2/2(x=4000), node4=(w1+w2)/2(x=2000)
        let expected_internal = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
        assert!(
            (gen_internal.rep_nodes[0].coord[0] - expected_internal).abs() < 1e-2,
            "{}",
            gen_internal.rep_nodes[0].coord[0]
        );

        let base_only = k_brace_model(KBraceWeightRule::BaseNodesOnly);
        let gen_base = generate_stories(&base_only, None).unwrap();
        // 手計算: node2=w1(x=0), node3=w2(x=4000), node4=0
        let expected_base = 4000.0 * w2 / (w1 + w2);
        assert!(
            (gen_base.rep_nodes[0].coord[0] - expected_base).abs() < 1e-2,
            "{}",
            gen_base.rep_nodes[0].coord[0]
        );

        // 両者は明確に異なる(基準節点側、より重いブレースが繋がる node3 側へ寄る)。
        assert!(gen_base.rep_nodes[0].coord[0] > gen_internal.rep_nodes[0].coord[0]);

        // 総重量(層重量)自体は配分規則によらず保存される。
        assert!(
            (gen_internal.stories[0].seismic_weight.unwrap()
                - gen_base.stories[0].seismic_weight.unwrap())
            .abs()
                < 1e-6
        );
    }

    #[test]
    fn test_k_brace_internal_nodes_default_is_half_half() {
        // 既定(InternalNodes)は両端 1/2 ずつ(従来どおり)であることを回帰確認する。
        let mut model = k_brace_model(KBraceWeightRule::InternalNodes);
        model.load_cfg = None; // 既定値(LoadCfg::default())でも InternalNodes になることを確認
        let gen = generate_stories(&model, None).unwrap();
        let density = 7.85e-9;
        let len = 2000.0;
        let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
        let w2 = density * 20000.0 * len * GRAVITY_MM_S2;
        let expected = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
        assert!(
            (gen.rep_nodes[0].coord[0] - expected).abs() < 1e-2,
            "{}",
            gen.rep_nodes[0].coord[0]
        );
    }
}
