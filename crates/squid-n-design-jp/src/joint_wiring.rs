//! 節点単位の断面検定（柱梁接合部・パネルゾーン・冷間成形耐力比・耐震壁）の
//! 入力組み立て（RESP-D マニュアル 04 断面検定）。
//!
//! [`crate::joint`] の純関数群に対し、`Model` と部材内力から入力を組み立てて
//! 一括実行する。squid-n-app（GUI）と squid-n-mcp（ヘッドレス）の両方から
//! 呼ばれる共通経路。
//!
//! # 入力の組み立てにおける簡略化（doc 兼申し送り）
//! - 部材種別は部材軸の鉛直成分による幾何判定（app の `member_kind_of` と同じ規則）。
//! - RC 接合部の形状（十字/T/ト/L）は「上下柱の有無 × 取り付く梁の本数(2 以上/1)」で
//!   判定し、加力方向別の区別はしない（全方向の梁をまとめて扱う）。
//! - 冷間成形角形鋼管の軸力比に用いる存在軸力は当該解析ケースの軸力
//!   （`NL + 1.5·NE` の割増は組合せ分離情報が無いため未対応）。
//! - S 造パネルの梁段違い形式（せい差 150mm 以上）は判別せず標準形式で計算する。
//! - 耐震壁は `SectionShape::RcWall` を割り当てた Wall 要素のみ検定する。
//!   設計用せん断力は等価梁化された壁要素の内力の最大水平せん断成分を用いる
//!   （暫定）。`Model::wall_attrs` に開口面積合計・個別開口寸法・三方スリット
//!   の有無が登録されている場合は以下のとおり配線する。
//!   - まず `Model::multi_opening_mode`（建物一律。既定は `Equivalent`）に
//!     応じて `WallAttr::opening_dims_for(mode)` でモード適用後の個別開口
//!     `(l0,h0)` 列を得る（RESP-D マニュアル計算編02「複数開口の取り扱い」）。
//!     - `Equivalent`: 個別開口をそのまま返す（従来どおり）。
//!     - `Envelope`: 位置（`offset`）を持つ開口全体の包絡矩形 1 つに置換
//!       する。位置不明の開口は包絡できないため個別のまま残る。
//!     - `Auto`: 包絡可能な開口対が無くなるまで繰り返し包絡開口を作成し、
//!       残った開口列を返す。
//!   - モード適用後のリストが要素数 1（単一開口）の場合は開口の実寸法
//!     `(l0,h0)` をそのまま用いる（等価開口への置換はしない）。単一開口は
//!     実寸法そのものが RC規準18条の `γ1=1−l0/l`・`γ3=1−h0/h` に直接効く
//!     ため、面積が同じでも辺長比が壁と異なれば等価開口に置換した場合と
//!     検定比が変わる。
//!   - モード適用後のリストが複数残る場合（`Equivalent` で複数開口のまま、
//!     `Auto` で包絡しきれない対が残る、`Envelope` で位置不明の開口が残る、
//!     のいずれか）は、マニュアル「包絡できなくなった時点の開口状況で
//!     『等価開口とする』と同様の判定を行います」に従い、
//!     [`crate::wall_opening::equivalent_opening`] で面積総和を保つ単一の
//!     等価開口 `(l0′,h0′)` に統合する。
//!   - `opening_dims_for(mode)` が `None`（個別寸法未入力・合計面積のみ）の
//!     場合は従来どおり壁と同じ辺長比を持つ擬似ペア（面積は
//!     `WallAttr::total_opening_area_for(mode)` で評価。個別開口が無い
//!     ためモードによらず合計面積と同値）から
//!     [`crate::wall_opening::equivalent_opening`] で等価開口を復元する
//!     （後方互換）。
//!   - いずれの経路で得た `(l0′,h0′)` も壁寸法 `(l,h)` を超える場合は
//!     安全側にクランプしたうえで、[`crate::joint::rc_wall_shear_check`] の
//!     `RcWallInput.opening` へ供給する（RC規準18条のせん断耐力検定用の
//!     低減係数 `r=min(γ1,γ2,γ3)`）。開口周比 r0（耐震壁判定用、下記）も
//!     このモード適用後の `(l0′,h0′)` から算定するため、判定・検定の双方が
//!     選択したモードに整合する。
//!   - 一方、耐震壁として扱ってよいか（スリットの有無・壁厚・開口周比 r0）は
//!     [`crate::wall_opening::is_seismic_wall`]（RESP-D マニュアル 02 剛性計算）
//!     で判定し、`false` の壁は本検定自体をスキップする（耐震壁ではない
//!     壁に18条検定を適用しない）。
//!
//!   [`crate::wall_opening`] の `r=1−1.25・r0` は剛性計算専用の低減率であり、
//!   上記 18 条の `r=min(γ1,γ2,γ3)` とは準拠する規定も数式も異なる別物
//!   である。02章の r0/r は耐震壁判定・等価開口の算定にのみ用い、18条の
//!   γ式や `Q1,Q2` の計算に流用してはならない（数式が異なるため結果が
//!   変わる）。
//! - SRC 側柱（[`SectionShape::SrcRect`]）を持つ耐震壁は、側柱の内蔵鉄骨の
//!   ウェブせん断断面積 `As = steel_web_thick・(steel_height − 2・steel_flange_thick)`
//!   と、鋼種（`steel_grade`）から [`crate::steel::steel_f_value_prefix`] で
//!   解決した F 値・[`crate::steel::steel_fs`] による許容せん断応力度 `sfs`
//!   から `steel_shear = sfs・As` を算定し、[`WallSideColumn::steel_shear`]
//!   へ供給する（RC 側柱は 0）。F 値の板厚区分にはフランジ厚とウェブ厚の
//!   大きい方を用いる（他のフォールバック箇所と同じ近似）。
//! - SRC 造柱梁接合部（パネルゾーン、[`crate::joint::src_panel_zone_check`]）:
//!   柱断面形状が `SrcRect` の節点で検定する。梁の上下主筋間距離 mBd・柱の
//!   左右主筋間距離 mCd は、既存の RC 接合部配線（`beam_j` に
//!   `d − rc_dt(rebar)` を用いる近似）に合わせ、「梁せい／柱幅 −
//!   2・rc_dt(rebar)」（`rc_dt` はかぶり＋帯筋径＋主筋径/2）で近似する
//!   （鉄筋位置の実配置ではなく主筋かぶり情報からの近似）。梁が S 造の
//!   場合は mBd の代わりに sBd（フランジ板厚中心間距離、S パネルゾーンの
//!   `db` 算定と同じ近似）を用いる。柱鉄骨のフランジ重心間距離 sCd は
//!   `steel_height − steel_flange_thick`、接合部鉄骨ウェブ厚 Jtw は柱の
//!   `steel_web_thick`、ヤング係数比 n は [`crate::rc::young_ratio_n`]。
//!   内法階高/階高比 h′/h は情報が無いため 1.0 固定とする（暫定）。

use crate::joint::{
    box_zp, cold_formed_column_ratio_check, panel_mpp, rc_joint_shear_check, rc_wall_shear_check,
    s_panel_zone_check, src_panel_zone_check, ColdFormedInput, JointShape, PanelSection,
    RcJointInput, RcWallInput, SrcPanelInput, WallSideColumn,
};
use crate::wall_opening::{equivalent_opening, is_seismic_wall, opening_ratio_r0, WallJudgeInput};
use crate::{CheckResult, LoadTerm};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, ElementKind, Material, Model, Section};
use squid_n_core::section_shape::SectionShape;

/// 1 部材分の内力（評価位置と [N,Qy,Qz,Mx,My,Mz]）。
pub type ForcesAt<'a> = &'a [(f64, [f64; 6])];

/// 鋼材判定（app の `is_steel` と同じ規則。鉄筋 SD/SR は RC 扱い）。
fn is_steel(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
        || upper.starts_with("SA")
        || upper.starts_with("BC")
}

/// 冷間成形角形鋼管（BCR/BCP）判定。
fn is_cold_formed(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("BCR") || upper.starts_with("BCP")
}

/// 収集済みの部材情報。
struct MemberInfo<'a> {
    elem: &'a ElementData,
    sec: &'a Section,
    mat: &'a Material,
    forces: ForcesAt<'a>,
    /// 部材軸の鉛直成分（|ez|）。
    ez: f64,
    length: f64,
}

impl MemberInfo<'_> {
    fn is_column(&self) -> bool {
        self.ez >= 0.8
    }
    fn is_beam_horiz(&self) -> bool {
        self.ez <= 0.2
    }
    /// 節点 `nid` 側の端部内力行（pos 0/1 のうち近い方）。
    fn end_forces(&self, nid: NodeId) -> Option<&[f64; 6]> {
        let pos = if self.elem.nodes.first() == Some(&nid) {
            0.0
        } else if self.elem.nodes.get(1) == Some(&nid) {
            1.0
        } else {
            return None;
        };
        self.forces
            .iter()
            .min_by(|a, b| {
                (a.0 - pos)
                    .abs()
                    .partial_cmp(&(b.0 - pos).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(_, f)| f)
    }
}

/// 主筋 1 段の重心位置（引張縁から）k1 = かぶり + 帯筋径 + 主筋径/2。
fn rc_dt(rebar: &squid_n_core::section_shape::RcRebar) -> f64 {
    rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0
}

/// H 形鋼の塑性断面係数（強軸）Zp = B·tf·(H−tf) + tw·(H−2tf)²/4。
fn h_zp(h: f64, b: f64, tw: f64, tf: f64) -> f64 {
    b * tf * (h - tf) + tw * (h - 2.0 * tf).powi(2) / 4.0
}

/// モデルと部材内力から節点単位の検定を一括実行する。
///
/// 戻り値: `(節点, 種別ラベル, 検定結果)` のリスト。
///
/// 冷間成形角形鋼管の存在軸力に `NL + 1.5・NE` の割増を効かせたい場合は
/// [`collect_joint_checks_with_long`] を使う（本関数は割増なし＝当該ケースの
/// 軸力そのまま）。
pub fn collect_joint_checks(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    collect_joint_checks_with_long(model, member_forces, None, term)
}

/// [`collect_joint_checks`] の長期内力付き版。
///
/// `long_member_forces` に長期（G+P）組合せの部材内力を渡すと、冷間成形
/// 角形鋼管の柱梁耐力比チェックの存在軸力を `N = NL + 1.5・NE`
/// （NE = 当該ケースの軸力 − NL。RESP-D マニュアル 04「冷間成形角型鋼管の
/// 断面検定」の Ds/Co = 1.5 割増）で算定する。None の場合は当該ケースの
/// 軸力をそのまま用いる（従来動作）。地震時組合せの結果を渡すことを想定する。
pub fn collect_joint_checks_with_long(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    long_member_forces: Option<&[(ElemId, ForcesAt<'_>)]>,
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    let mut out = Vec::new();

    // 部材情報の収集（2 節点の梁/柱系要素）
    let mut members: Vec<MemberInfo<'_>> = Vec::new();
    for (eid, forces) in member_forces {
        let Some(elem) = model.elements.iter().find(|e| e.id == *eid) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let mat = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid));
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };
        let (Some(p0), Some(p1)) = (
            model.nodes.get(elem.nodes[0].index()).map(|n| n.coord),
            model.nodes.get(elem.nodes[1].index()).map(|n| n.coord),
        ) else {
            continue;
        };
        let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let length = (dx * dx + dy * dy + dz * dz).sqrt();
        if length < 1e-9 {
            continue;
        }
        members.push(MemberInfo {
            elem,
            sec,
            mat,
            forces,
            ez: (dz / length).abs(),
            length,
        });
    }

    // ── 耐震壁（Wall 要素 × RcWall 形状） ────────────────────────
    for (eid, forces) in member_forces {
        let Some(elem) = model.elements.iter().find(|e| e.id == *eid) else {
            continue;
        };
        if elem.kind != ElementKind::Wall {
            continue;
        }
        let Some(sec) = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid))
        else {
            continue;
        };
        let Some(SectionShape::RcWall { thickness, ps }) = sec.shape else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid))
        else {
            continue;
        };
        let fc = mat.fc.unwrap_or(0.0);
        if fc <= 0.0 {
            continue;
        }
        // 壁の平面寸法: 節点群の水平距離の最大 = l、鉛直 extent = h。
        let coords: Vec<[f64; 3]> = elem
            .nodes
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .map(|n| n.coord)
            .collect();
        if coords.len() < 3 {
            continue;
        }
        let mut l = 0.0_f64;
        for i in 0..coords.len() {
            for jj in (i + 1)..coords.len() {
                let dx = coords[i][0] - coords[jj][0];
                let dy = coords[i][1] - coords[jj][1];
                l = l.max((dx * dx + dy * dy).sqrt());
            }
        }
        if l < 1e-9 {
            continue;
        }
        let h = coords.iter().map(|c| c[2]).fold(f64::MIN, f64::max)
            - coords.iter().map(|c| c[2]).fold(f64::MAX, f64::min);

        // 壁自重属性（開口面積合計・個別開口寸法・三方スリット）。未登録の壁は
        // 開口ゼロ・スリット無し（無開口の耐震壁）として扱う。
        let attr = model.wall_attrs.iter().find(|w| w.elem == elem.id);
        let has_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);

        // 開口寸法 (l0',h0') の評価。h・l ≤ 0（寸法不定）の場合は開口ゼロ扱い
        // とする。
        let (mut l0p, mut h0p) = if h > 1e-9 && l > 1e-9 {
            match attr.and_then(|a| a.opening_dims_for(model.multi_opening_mode)) {
                // モード適用後の開口が単一（複数開口の包絡・統合の結果 1 個に
                // なった場合を含む）: 実寸法をそのまま使う（γ1=1-l0/l・
                // γ3=1-h0/h へ実寸法が直接効くため、等価開口への置換はしない）。
                Some(dims) if dims.len() == 1 => dims[0],
                // モード適用後も複数開口が残る場合（Auto で包絡しきれない対
                // が残る・Envelope で位置不明の開口が残る・Equivalent で
                // 複数開口のまま）は、面積総和を保つ単一の等価開口に統合する
                // （RESP-D マニュアル計算編02「複数開口の取り扱い」）。
                Some(dims) => equivalent_opening(&dims, l, h),
                // 個別寸法が未入力（合計面積のみ）の場合は従来どおり、壁と
                // 同じ辺長比を持つ擬似ペアから等価開口を復元する（後方互換）。
                None => {
                    let area = attr
                        .map(|a| a.total_opening_area_for(model.multi_opening_mode))
                        .unwrap_or(0.0);
                    if area > 0.0 {
                        equivalent_opening(&[(area / h, h)], l, h)
                    } else {
                        (0.0, 0.0)
                    }
                }
            }
        } else {
            (0.0, 0.0)
        };
        // 開口寸法が壁寸法を超える場合のガード（実寸法入力の誤り等に対する
        // 安全側処理）。
        l0p = l0p.clamp(0.0, l);
        h0p = h0p.clamp(0.0, h);

        // 耐震壁判定（RESP-D マニュアル 02 剛性計算）。スリットあり・壁厚
        // <120mm・開口周比 r0>0.4 のいずれかに該当する壁は耐震壁として
        // 扱わないため、RC規準18条の耐震壁せん断検定自体を対象外とする。
        let r0 = opening_ratio_r0(h0p, l0p, h, l);
        let judge = WallJudgeInput {
            thickness,
            r0,
            has_slit,
        };
        if !is_seismic_wall(&judge) {
            continue;
        }
        // 側柱: 壁節点のうち 2 節点を両端に持つ鉛直部材。
        let wall_nodes = &elem.nodes;
        let mut side_columns = Vec::new();
        let mut sum_col_depth = 0.0;
        for m in &members {
            if !m.is_column() {
                continue;
            }
            let n0 = m.elem.nodes[0];
            let n1 = m.elem.nodes[1];
            if !(wall_nodes.contains(&n0) && wall_nodes.contains(&n1)) {
                continue;
            }
            // SRC 側柱（内蔵鉄骨あり）はウェブせん断断面積 As と鋼種の F 値から
            // sfs・As を Qc への加算項として算定する（冒頭 doc 参照）。RC 側柱
            // （内蔵鉄骨なし）は 0。
            let steel_shear = match m.sec.shape {
                Some(SectionShape::SrcRect {
                    steel_height,
                    steel_web_thick,
                    steel_flange_thick,
                    ref steel_grade,
                    ..
                }) => {
                    let as_web =
                        (steel_web_thick * (steel_height - 2.0 * steel_flange_thick)).max(0.0);
                    let f = crate::steel::steel_f_value_prefix(
                        steel_grade,
                        steel_flange_thick.max(steel_web_thick),
                    )
                    .unwrap_or(235.0);
                    crate::steel::steel_fs(f, term) * as_web
                }
                _ => 0.0,
            };
            let bd_rebar = match m.sec.shape {
                Some(SectionShape::RcRect { b, d, ref rebar }) => Some((b, d, rebar)),
                Some(SectionShape::SrcRect {
                    b, d, ref rebar, ..
                }) => Some((b, d, rebar)),
                _ => None,
            };
            let Some((b, d, rebar)) = bd_rebar else {
                continue;
            };
            let dt = rc_dt(rebar);
            let pw = if rebar.shear.pitch > 0.0 {
                rebar.shear.legs as f64 * std::f64::consts::PI * rebar.shear.dia * rebar.shear.dia
                    / 4.0
                    / (b * rebar.shear.pitch)
            } else {
                0.0
            };
            side_columns.push(WallSideColumn {
                b,
                d_eff: d - dt,
                pw,
                w_ft: crate::rc::rebar_allowable_shear(&m.mat.name, term == LoadTerm::Long),
                steel_shear,
            });
            sum_col_depth += d;
        }
        let l_clear = (l - sum_col_depth / 2.0).max(0.1 * l);
        // 設計用せん断力: 等価梁化された壁要素内力の最大水平せん断成分（暫定）。
        let q_design = forces
            .iter()
            .map(|(_, f)| f[1].abs().max(f[2].abs()))
            .fold(0.0, f64::max);
        let inp = RcWallInput {
            t: thickness,
            l,
            l_clear,
            fc,
            ps,
            w_ft: crate::rc::rebar_allowable_shear(&mat.name, term == LoadTerm::Long),
            side_columns,
            // 開口寸法 (l0',h0')（単一開口は実寸法・複数開口は等価開口・
            // 面積のみは擬似等価開口）を 18条のγ式（r=min(γ1,γ2,γ3)）へ
            // 供給する（冒頭 doc 参照。02章の r0/r とは別式のため流用しない）。
            opening: if l0p > 1e-9 && h0p > 1e-9 {
                Some((l0p, h0p, h, l))
            } else {
                None
            },
            q_design,
            long_term: term == LoadTerm::Long,
        };
        let cr = rc_wall_shear_check(&inp);
        out.push((elem.nodes[0], "耐震壁(RC)".to_string(), cr));
    }

    // ── 節点単位の接合部検定 ─────────────────────────────────────
    for (ni, node) in model.nodes.iter().enumerate() {
        let nid = node.id;
        let _ = ni;
        let cols: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_column() && m.elem.nodes.contains(&nid))
            .collect();
        let beams: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_beam_horiz() && m.elem.nodes.contains(&nid))
            .collect();
        if cols.is_empty() || beams.is_empty() {
            continue;
        }

        // ── RC 柱梁接合部 ────────────────────────────────────────
        let rc_col = cols.iter().find(|c| {
            matches!(c.sec.shape, Some(SectionShape::RcRect { .. }))
                && c.mat.fc.unwrap_or(0.0) > 0.0
        });
        let rc_beams: Vec<&&MemberInfo> = beams
            .iter()
            .filter(|b| matches!(b.sec.shape, Some(SectionShape::RcRect { .. })))
            .collect();
        if let (Some(col), false) = (rc_col, rc_beams.is_empty()) {
            let shape = match (cols.len() >= 2, rc_beams.len() >= 2) {
                (true, true) => JointShape::Cross,
                (false, true) => JointShape::Tee,
                (true, false) => JointShape::Knee,
                (false, false) => JointShape::Corner,
            };
            let Some(SectionShape::RcRect { .. }) = col.sec.shape else {
                unreachable!()
            };
            let beam0 = rc_beams[0];
            let beam_j = if let Some(SectionShape::RcRect { d, ref rebar, .. }) = beam0.sec.shape {
                7.0 / 8.0 * (d - rc_dt(rebar))
            } else {
                0.8 * beam0.sec.depth
            };
            let sum_beam_moments: f64 = rc_beams
                .iter()
                .filter_map(|b| b.end_forces(nid))
                .map(|f| f[5].abs())
                .sum();
            let col_shear = cols
                .iter()
                .filter_map(|c| c.end_forces(nid))
                .map(|f| f[1].abs().max(f[2].abs()))
                .fold(0.0, f64::max);
            let col_height = cols.iter().map(|c| c.length).sum::<f64>() / cols.len() as f64;
            let beam_span = rc_beams.iter().map(|b| b.length).sum::<f64>() / rc_beams.len() as f64;
            let inp = RcJointInput {
                shape,
                fc: col.mat.fc.unwrap_or(0.0),
                col_depth: col.sec.depth,
                col_width: col.sec.width,
                beam_width: beam0.sec.width,
                beam_j,
                sum_beam_moments,
                col_shear,
                col_height,
                beam_span,
            };
            out.push((nid, "接合部(RC)".to_string(), rc_joint_shear_check(&inp)));
        }

        // ── S 造パネルゾーン ─────────────────────────────────────
        let s_col = cols.iter().find(|c| is_steel(&c.mat.name));
        let s_beams: Vec<&&MemberInfo> = beams.iter().filter(|b| is_steel(&b.mat.name)).collect();
        if let (Some(col), false) = (s_col, s_beams.is_empty()) {
            let panel = match col.sec.shape {
                Some(SectionShape::SteelH {
                    height,
                    width,
                    web_thick,
                    flange_thick,
                }) => Some(PanelSection::H {
                    bc: width,
                    tf: flange_thick,
                    dc: height - flange_thick,
                    tp: web_thick,
                }),
                Some(SectionShape::SteelBox {
                    height,
                    width,
                    thick,
                })
                | Some(SectionShape::CftBox {
                    height,
                    width,
                    thick,
                }) => Some(PanelSection::Box {
                    bc: width,
                    dc: height - thick,
                    tp: thick,
                }),
                Some(SectionShape::SteelPipe { outer_dia, thick })
                | Some(SectionShape::CftPipe { outer_dia, thick }) => Some(PanelSection::Pipe {
                    dc: outer_dia - thick,
                    tp: thick,
                }),
                _ => None,
            };
            if let Some(panel) = panel {
                // 梁フランジ板厚中心間距離 db（最大せいの梁を採用）。
                let db = s_beams
                    .iter()
                    .map(|b| match b.sec.shape {
                        Some(SectionShape::SteelH { flange_thick, .. }) => {
                            b.sec.depth - flange_thick
                        }
                        _ => 0.9 * b.sec.depth,
                    })
                    .fold(0.0, f64::max);
                let t = crate::steel::steel_f_value_prefix(&col.mat.name, 40.0);
                let fy = t.unwrap_or(235.0);
                // 軸力比 n = 圧縮軸力/(F·A)（当該ケースの軸力。引張は 0）。
                let n_axial = col
                    .end_forces(nid)
                    .map(|f| (-f[0]).max(0.0) / (fy * col.sec.area.max(1e-9)))
                    .unwrap_or(0.0);
                let m_left = s_beams
                    .first()
                    .and_then(|b| b.end_forces(nid))
                    .map(|f| f[5].abs())
                    .unwrap_or(0.0);
                let m_right = s_beams
                    .get(1)
                    .and_then(|b| b.end_forces(nid))
                    .map(|f| f[5].abs())
                    .unwrap_or(0.0);
                let mut col_qs: Vec<f64> = cols
                    .iter()
                    .filter(|c| is_steel(&c.mat.name))
                    .filter_map(|c| c.end_forces(nid))
                    .map(|f| f[1].abs().max(f[2].abs()))
                    .collect();
                col_qs.resize(2, 0.0);
                let inp = crate::joint::SPanelInput {
                    section: panel,
                    db,
                    fy,
                    axial_ratio: n_axial,
                    beam_moment_left: m_left,
                    beam_moment_right: m_right,
                    col_shear_upper: col_qs[0],
                    col_shear_lower: col_qs[1],
                };
                out.push((nid, "パネルゾーン(S)".to_string(), s_panel_zone_check(&inp)));
            }
        }

        // ── SRC 造柱梁接合部（パネルゾーン） ─────────────────────
        let src_col = cols.iter().find(|c| {
            matches!(c.sec.shape, Some(SectionShape::SrcRect { .. }))
                && c.mat.fc.unwrap_or(0.0) > 0.0
        });
        if let Some(col) = src_col {
            if let Some(SectionShape::SrcRect {
                ref rebar,
                steel_height,
                steel_web_thick,
                steel_flange_thick,
                ..
            }) = col.sec.shape
            {
                let fc = col.mat.fc.unwrap_or(0.0);
                // mCd（柱の左右主筋間距離）の近似: 柱幅 − 2・rc_dt(rebar)
                // （冒頭 doc 参照。既存 RC 接合部配線の beam_j 近似に合わせる）。
                let m_cd = (col.sec.width - 2.0 * rc_dt(rebar)).max(0.0);
                let s_cd = (steel_height - steel_flange_thick).max(0.0);
                let j_tw = steel_web_thick;

                let beam0 = beams[0];
                let beam_is_steel = is_steel(&beam0.mat.name);
                let m_bd = if beam_is_steel {
                    // 梁が S 造の場合は mBd の代わりに sBd（フランジ板厚中心間
                    // 距離）を渡す（S パネルゾーンの db 算定と同じ近似）。
                    match beam0.sec.shape {
                        Some(SectionShape::SteelH { flange_thick, .. }) => {
                            beam0.sec.depth - flange_thick
                        }
                        _ => 0.9 * beam0.sec.depth,
                    }
                } else {
                    match beam0.sec.shape {
                        Some(SectionShape::RcRect { ref rebar, .. })
                        | Some(SectionShape::SrcRect { ref rebar, .. }) => {
                            (beam0.sec.depth - 2.0 * rc_dt(rebar)).max(0.0)
                        }
                        _ => 0.8 * beam0.sec.depth,
                    }
                };

                // 接合部形状（RC 接合部配線と同じ判定: 柱2本以上×取り付く梁
                // 2本以上で十字形、以下同様）。
                let shape = match (cols.len() >= 2, beams.len() >= 2) {
                    (true, true) => JointShape::Cross,
                    (false, true) => JointShape::Tee,
                    (true, false) => JointShape::Knee,
                    (false, false) => JointShape::Corner,
                };

                let sum_beam_moments: f64 = beams
                    .iter()
                    .filter_map(|b| b.end_forces(nid))
                    .map(|f| f[5].abs())
                    .sum();

                let inp = SrcPanelInput {
                    shape,
                    fc,
                    long_term: term == LoadTerm::Long,
                    col_width: col.sec.width,
                    beam_width: beam0.sec.width,
                    m_bd,
                    m_cd,
                    j_tw,
                    s_cd,
                    beam_is_steel,
                    n_ratio: crate::rc::young_ratio_n(fc),
                    // h′/h（内法階高/階高比、原典図 2026-07-11）は情報が無いため 1.0 固定（暫定、
                    // 冒頭 doc 参照）。
                    h_ratio: 1.0,
                    sum_beam_moments,
                };
                out.push((
                    nid,
                    "柱梁接合部(SRC)".to_string(),
                    src_panel_zone_check(&inp),
                ));
            }
        }

        // ── 冷間成形角形鋼管の柱梁耐力比 ────────────────────────
        let cf_cols: Vec<&&MemberInfo> = cols
            .iter()
            .filter(|c| {
                is_cold_formed(&c.mat.name)
                    && matches!(
                        c.sec.shape,
                        Some(SectionShape::SteelBox { .. }) | Some(SectionShape::CftBox { .. })
                    )
            })
            .collect();
        if !cf_cols.is_empty() {
            // 長期（G+P）の当該部材・当該節点側の軸力 NL [N]（引張正）。
            // 存在軸力 N = NL + 1.5・NE（NE = 当該ケース軸力 − NL）に用いる。
            let long_end_n = |c: &MemberInfo, nid: NodeId| -> Option<f64> {
                let list = long_member_forces?;
                let (_, forces) = list.iter().find(|(id, _)| *id == c.elem.id)?;
                let pos = if c.elem.nodes.first() == Some(&nid) {
                    0.0
                } else if c.elem.nodes.get(1) == Some(&nid) {
                    1.0
                } else {
                    return None;
                };
                forces
                    .iter()
                    .min_by(|a, b| {
                        (a.0 - pos)
                            .abs()
                            .partial_cmp(&(b.0 - pos).abs())
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(_, f)| f[0])
            };
            let zp_f_n = |c: &MemberInfo| -> Option<(f64, f64, f64)> {
                let (h, b, t) = match c.sec.shape {
                    Some(SectionShape::SteelBox {
                        height,
                        width,
                        thick,
                    })
                    | Some(SectionShape::CftBox {
                        height,
                        width,
                        thick,
                    }) => (height, width, thick),
                    _ => return None,
                };
                let f = crate::steel::steel_f_value_prefix(&c.mat.name, t).unwrap_or(295.0);
                let n = c
                    .end_forces(nid)
                    .map(|fr| {
                        // 圧縮正に変換して存在軸力を求める（RESP-D: N = NL+1.5・NE）。
                        let n_cur = -fr[0];
                        let n_exist = match long_end_n(c, nid) {
                            Some(nl_signed) => {
                                let nl = -nl_signed;
                                nl + 1.5 * (n_cur - nl)
                            }
                            None => n_cur,
                        };
                        n_exist.max(0.0) / (f * c.sec.area.max(1e-9))
                    })
                    .unwrap_or(0.0);
                Some((box_zp(h, b, t), f, n))
            };
            let upper = zp_f_n(cf_cols[0]);
            let lower = cf_cols.get(1).and_then(|c| zp_f_n(c)).or(upper);
            if let (Some((zp_u, f_u, n_u)), Some((zp_l, f_l, n_l))) = (upper, lower) {
                // 梁の全塑性モーメント和 Σ(Fyb·Zpb)（H 形鋼の鋼梁のみ算入）。
                let sum_beam_mp: f64 = beams
                    .iter()
                    .filter(|b| is_steel(&b.mat.name))
                    .filter_map(|b| match b.sec.shape {
                        Some(SectionShape::SteelH {
                            height,
                            width,
                            web_thick,
                            flange_thick,
                        }) => {
                            let fb = crate::steel::steel_f_value_prefix(
                                &b.mat.name,
                                flange_thick.max(web_thick),
                            )
                            .unwrap_or(235.0);
                            Some(fb * h_zp(height, width, web_thick, flange_thick))
                        }
                        _ => None,
                    })
                    .sum();
                if sum_beam_mp > 0.0 {
                    // パネル耐力 Mpp（下柱寸法・db は最大梁せい基準）。
                    let (dc, tp) = match cf_cols[0].sec.shape {
                        Some(SectionShape::SteelBox { height, thick, .. })
                        | Some(SectionShape::CftBox { height, thick, .. }) => {
                            (height - thick, thick)
                        }
                        _ => (0.0, 0.0),
                    };
                    let db = beams
                        .iter()
                        .map(|b| match b.sec.shape {
                            Some(SectionShape::SteelH { flange_thick, .. }) => {
                                b.sec.depth - flange_thick
                            }
                            _ => 0.9 * b.sec.depth,
                        })
                        .fold(0.0, f64::max);
                    let mpp = panel_mpp(dc, db, tp, f_l, n_l);
                    let inp = ColdFormedInput {
                        zp_col_upper: zp_u,
                        zp_col_lower: zp_l,
                        f_col_upper: f_u,
                        f_col_lower: f_l,
                        n_upper: n_u,
                        n_lower: n_l,
                        sum_beam_mp,
                        panel_mpp: mpp,
                    };
                    out.push((
                        nid,
                        "冷間成形耐力比".to_string(),
                        cold_formed_column_ratio_check(&inp),
                    ));
                }
            }
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Material, MultiOpeningMode, Node,
        RigidZone, Section, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    /// 矩形壁（4000×3000, t=180）1 枚のみのモデル。側柱なし。
    /// `wall_attr` を指定すると `model.wall_attrs` に登録する。
    fn wall_model(wall_attr: Option<WallAttr>) -> Model {
        wall_model_sized(4000.0, 3000.0, 180.0, wall_attr)
    }

    /// 矩形壁（`l`×`h`, 厚さ `thickness`）1 枚のみのモデル。側柱なし。
    /// `wall_model` の寸法可変版（近接開口・包絡開口のテストで、開口周比 r0
    /// を任意の壁面積に対して調整するために用いる）。
    fn wall_model_sized(l: f64, h: f64, thickness: f64, wall_attr: Option<WallAttr>) -> Model {
        let mut nodes: Vec<Node> = Vec::new();
        let coords = [[0.0, 0.0, 0.0], [l, 0.0, 0.0], [l, 0.0, h], [0.0, 0.0, h]];
        for (i, c) in coords.iter().enumerate() {
            nodes.push(Node {
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
        let sections = vec![Section {
            id: SectionId(0),
            name: "wall".to_string(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(thickness),
            shape: Some(SectionShape::RcWall {
                thickness,
                ps: 0.006,
            }),
        }];
        let materials = vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }];
        let elements = vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: {
                let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                v.push(NodeId(0));
                v.push(NodeId(1));
                v.push(NodeId(2));
                v.push(NodeId(3));
                v
            },
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
        }];
        Model {
            nodes,
            elements,
            sections,
            materials,
            wall_attrs: wall_attr.into_iter().collect(),
            ..Default::default()
        }
    }

    /// 壁要素 ElemId(0) の耐震壁(RC)検定結果（無ければ None）。
    fn wall_check_result(model: &Model, forces: ForcesAt<'_>) -> Option<CheckResult> {
        let member_forces = vec![(ElemId(0), forces)];
        collect_joint_checks(model, &member_forces, LoadTerm::Short)
            .into_iter()
            .find(|(_, label, _)| label == "耐震壁(RC)")
            .map(|(_, _, cr)| cr)
    }

    /// 開口あり（`wall_attrs` に `opening_area>0` を登録）の壁は、無開口より
    /// 検定比が大きくなる（開口低減係数 r<1 で Qa が下がるため）。
    #[test]
    fn wall_with_opening_has_larger_ratio_than_without() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];

        let model_no_attr = wall_model(None);
        let res_no_opening =
            wall_check_result(&model_no_attr, &forces).expect("無開口の壁は検定されるはず");

        // opening_area = 0.1・l・h → r0 ≈ 0.316（<0.4 で耐震壁として扱われる）。
        let model_with_opening = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.1 * 4000.0 * 3000.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        }));
        let res_opening = wall_check_result(&model_with_opening, &forces)
            .expect("小開口は耐震壁のまま検定される");

        assert!(
            res_opening.ratio > res_no_opening.ratio,
            "開口あり ratio={} <= 開口なし ratio={}",
            res_opening.ratio,
            res_no_opening.ratio
        );
    }

    /// 三方スリットが指定された壁は耐震壁として扱われず、耐震壁検定自体が
    /// 出力されない。
    #[test]
    fn wall_with_three_side_slit_is_not_checked() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        let model = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: true,
            openings: vec![],
        }));
        assert!(wall_check_result(&model, &forces).is_none());
    }

    /// 開口周比 r0>0.4 となる大開口の壁も耐震壁として扱われず出力されない。
    #[test]
    fn wall_with_large_opening_ratio_is_not_checked() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        // opening_area = 0.5・l・h → r0 = sqrt(0.5) ≈ 0.707 > 0.4。
        let model = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.5 * 4000.0 * 3000.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        }));
        assert!(wall_check_result(&model, &forces).is_none());
    }

    /// `wall_attrs` に属性が無い壁（厚さ≥120mm）は、従来どおり無開口として
    /// 耐震壁検定される。
    #[test]
    fn wall_without_attr_is_checked_as_no_opening() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        let model = wall_model(None);
        let res = wall_check_result(&model, &forces).expect("属性なしの壁も検定されるはず");
        assert!(res.ratio > 0.0);
    }

    /// 単一の個別開口（縦長: l0=750, h0=2000）と、同面積を合計面積のみで
    /// 与えた場合（壁と同じ辺長比の擬似等価開口に復元される）とで、
    /// γ支配項が変わるため検定比が一致しないこと。
    ///
    /// 面積は共通（750×2000=1,500,000）のため開口周比 r0（耐震壁判定用）は
    /// 両者で等しいが、実寸法は壁（l=4000,h=3000）と辺長比が異なる縦長形状
    /// のため γ3=1−h0/h が支配的になり、擬似等価開口（壁と同じ辺長比）を
    /// 使った場合の γ1=γ2=γ3 とは異なる低減係数 r になる。
    #[test]
    fn wall_single_opening_dims_differs_from_area_only_ratio() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];

        let model_single_dims = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![WallOpening {
                width: 750.0,
                height: 2000.0,
                offset: None,
            }],
        }));
        let res_single_dims = wall_check_result(&model_single_dims, &forces)
            .expect("r0<0.4 の単一開口は耐震壁として検定されるはず");

        let model_area_only = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 750.0 * 2000.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        }));
        let res_area_only = wall_check_result(&model_area_only, &forces)
            .expect("同面積を面積のみで与えた壁も耐震壁として検定されるはず");

        assert!(
            (res_single_dims.ratio - res_area_only.ratio).abs() > 1e-6,
            "個別寸法 ratio={} と面積のみ ratio={} が一致してしまっている",
            res_single_dims.ratio,
            res_area_only.ratio
        );
    }

    /// 複数開口（2個）は [`equivalent_opening`] による等価開口に統合され、
    /// その等価開口を直接 `RcWallInput` へ供給した場合と同じ検定比になる。
    #[test]
    fn wall_multiple_openings_matches_equivalent_opening() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        let dims = [(600.0, 800.0), (500.0, 700.0)];

        let model = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: dims
                .iter()
                .map(|&(w, h)| WallOpening {
                    width: w,
                    height: h,
                    offset: None,
                })
                .collect(),
        }));
        let res =
            wall_check_result(&model, &forces).expect("2個の開口は耐震壁として検定されるはず");

        // 期待値: equivalent_opening を直接呼んで壁と同じ辺長比の等価開口を
        // 構築し、同一の RcWallInput（側柱なし・l_clear=l）で検定した結果。
        let (l, h) = (4000.0_f64, 3000.0_f64);
        let (l0p, h0p) = equivalent_opening(&dims, l, h);
        let inp = RcWallInput {
            t: 180.0,
            l,
            l_clear: l,
            fc: 24.0,
            ps: 0.006,
            w_ft: crate::rc::rebar_allowable_shear("SD345", false),
            side_columns: vec![],
            opening: Some((l0p, h0p, h, l)),
            q_design: 500_000.0,
            long_term: false,
        };
        let expected = rc_wall_shear_check(&inp);

        assert!(
            (res.ratio - expected.ratio).abs() < 1e-9,
            "複数開口 ratio={} と等価開口直接計算 ratio={} が不一致",
            res.ratio,
            expected.ratio
        );
    }

    /// 個別開口の面積和で開口周比 r0>0.4 となる壁は耐震壁として扱われず、
    /// 検定自体が出力されない。
    #[test]
    fn wall_multiple_openings_large_ratio_is_not_checked() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        // 開口2個の面積和 = 2,000,000 + 3,000,000 = 5,000,000
        // → r0 = sqrt(5,000,000 / (4000*3000)) = sqrt(0.41667) ≈ 0.645 > 0.4。
        let model = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![
                WallOpening {
                    width: 2000.0,
                    height: 1000.0,
                    offset: None,
                },
                WallOpening {
                    width: 2000.0,
                    height: 1500.0,
                    offset: None,
                },
            ],
        }));
        assert!(wall_check_result(&model, &forces).is_none());
    }

    /// 近接する2開口（水平純間隔200mm、高さ位置が一致）は、`Auto` モードでは
    /// 包絡可能条件（純間隔が両開口の当該方向寸法の小さい方以下）を満たすため
    /// 幅2000×高2000の単一の包絡開口に統合され、実寸法経路（単一開口）として
    /// 検定される。既定の `Equivalent` モードでは個別開口のまま
    /// `equivalent_opening` で等価開口に統合されるため、両モードで検定比が
    /// 異なる（r0 の判定を通すため、壁は 8000×4000 とやや大きめに取る）。
    #[test]
    fn wall_auto_mode_envelopes_close_openings_and_differs_from_equivalent() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        let openings = vec![
            WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: Some([0.0, 0.0]),
            },
            WallOpening {
                width: 800.0,
                height: 2000.0,
                offset: Some([1200.0, 0.0]),
            },
        ];

        // 既定（Equivalent）モード: 個別開口のまま equivalent_opening で統合。
        let model_equiv = wall_model_sized(
            8000.0,
            4000.0,
            180.0,
            Some(WallAttr {
                elem: ElemId(0),
                opening_area: 0.0,
                opening_weight: 0.0,
                three_side_slit: false,
                openings: openings.clone(),
            }),
        );
        let res_equiv = wall_check_result(&model_equiv, &forces)
            .expect("Equivalent モードは耐震壁として検定されるはず");

        // Auto モード: 純間隔(200)が両開口の幅(800,1000)以下・高さ方向の
        // 純間隔が 0（重なり）のため包絡可能 → 幅2000×高2000の単一開口
        // （実寸法経路）に統合される。
        let mut model_auto = wall_model_sized(
            8000.0,
            4000.0,
            180.0,
            Some(WallAttr {
                elem: ElemId(0),
                opening_area: 0.0,
                opening_weight: 0.0,
                three_side_slit: false,
                openings,
            }),
        );
        model_auto.multi_opening_mode = MultiOpeningMode::Auto;
        let res_auto = wall_check_result(&model_auto, &forces)
            .expect("Auto モードで包絡後も耐震壁として検定されるはず");

        // 期待値: 幅2000×高2000の単一開口を実寸法経路で直接検定した結果。
        let model_single = wall_model_sized(
            8000.0,
            4000.0,
            180.0,
            Some(WallAttr {
                elem: ElemId(0),
                opening_area: 0.0,
                opening_weight: 0.0,
                three_side_slit: false,
                openings: vec![WallOpening {
                    width: 2000.0,
                    height: 2000.0,
                    offset: None,
                }],
            }),
        );
        let res_single = wall_check_result(&model_single, &forces)
            .expect("包絡開口相当の単一開口も耐震壁として検定されるはず");

        assert!(
            (res_auto.ratio - res_single.ratio).abs() < 1e-9,
            "Auto ratio={} と包絡開口(実寸法)直接計算 ratio={} が不一致",
            res_auto.ratio,
            res_single.ratio
        );
        assert!(
            (res_auto.ratio - res_equiv.ratio).abs() > 1e-6,
            "Auto ratio={} と Equivalent ratio={} が一致してしまっている",
            res_auto.ratio,
            res_equiv.ratio
        );
    }

    /// 遠く離れた小開口2つは、既定（Equivalent）モードでは面積和が小さく
    /// 耐震壁として検定されるが、`Envelope` モードでは全開口を包絡した巨大な
    /// 矩形の面積で開口周比 r0 を評価するため r0>0.4 となり、耐震壁として
    /// 扱われず検定自体が出力されない。
    #[test]
    fn wall_envelope_mode_excludes_wall_when_envelope_ratio_too_large() {
        let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
        let openings = vec![
            WallOpening {
                width: 200.0,
                height: 200.0,
                offset: Some([0.0, 0.0]),
            },
            WallOpening {
                width: 200.0,
                height: 200.0,
                offset: Some([3500.0, 2500.0]),
            },
        ];

        // 既定（Equivalent）モード: 面積和 = 200*200*2 = 80,000
        // → r0 = sqrt(80,000 / (4000*3000)) ≈ 0.0816 ≤ 0.4 で耐震壁として検定。
        let model_equiv = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: openings.clone(),
        }));
        assert!(
            wall_check_result(&model_equiv, &forces).is_some(),
            "Equivalent モードでは小開口のため耐震壁として検定されるはず"
        );

        // Envelope モード: 包絡矩形は幅3700×高2700 = 9,990,000
        // → r0 = sqrt(9,990,000 / (4000*3000)) ≈ 0.912 > 0.4 で耐震壁から除外。
        let mut model_envelope = wall_model(Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings,
        }));
        model_envelope.multi_opening_mode = MultiOpeningMode::Envelope;
        assert!(
            wall_check_result(&model_envelope, &forces).is_none(),
            "Envelope モードでは包絡矩形が大きく耐震壁から除外されるはず"
        );
    }
}
