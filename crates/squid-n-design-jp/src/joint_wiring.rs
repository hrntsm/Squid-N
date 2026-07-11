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
//!   （暫定）。`Model::wall_attrs` に開口面積合計・三方スリットの有無が
//!   登録されている場合は以下のとおり配線する。
//!   - 個別開口の寸法データは無く合計面積のみのため、[`crate::wall_opening::equivalent_opening`]
//!     で壁と同じ辺長比を持つ単一の等価開口 `(l0′,h0′)` に復元し、
//!     [`crate::joint::rc_wall_shear_check`] の `RcWallInput.opening` へ
//!     供給する（RC規準18条のせん断耐力検定用の低減係数 `r=min(γ1,γ2,γ3)`）。
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

use crate::joint::{
    box_zp, cold_formed_column_ratio_check, panel_mpp, rc_joint_shear_check, rc_wall_shear_check,
    s_panel_zone_check, ColdFormedInput, JointShape, PanelSection, RcJointInput, RcWallInput,
    WallSideColumn,
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
pub fn collect_joint_checks(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
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

        // 壁自重属性（開口面積合計・三方スリット）。未登録の壁は開口ゼロ・
        // スリット無し（無開口の耐震壁）として扱う。
        let attr = model.wall_attrs.iter().find(|w| w.elem == elem.id);
        let opening_area = attr.map(|a| a.opening_area).unwrap_or(0.0);
        let has_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);

        // 複数開口の寸法データが無く合計面積のみのため、壁と同じ辺長比を持つ
        // 単一の等価開口 (l0',h0') に復元する（Σli・hi = opening_area を保存）。
        // h・l ≤ 0（寸法不定）の場合は復元せず開口ゼロ扱いとする。
        let (l0p, h0p) = if opening_area > 0.0 && h > 1e-9 && l > 1e-9 {
            equivalent_opening(&[(opening_area / h, h)], l, h)
        } else {
            (0.0, 0.0)
        };

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
            if wall_nodes.contains(&n0) && wall_nodes.contains(&n1) {
                if let Some(SectionShape::RcRect { b, d, ref rebar }) = m.sec.shape {
                    let dt = rc_dt(rebar);
                    let pw = if rebar.shear.pitch > 0.0 {
                        rebar.shear.legs as f64
                            * std::f64::consts::PI
                            * rebar.shear.dia
                            * rebar.shear.dia
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
                    });
                    sum_col_depth += d;
                }
            }
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
            // 等価開口 (l0',h0') を 18条のγ式（r=min(γ1,γ2,γ3)）へ供給する
            // （冒頭 doc 参照。02章の r0/r とは別式のため流用しない）。
            opening: if opening_area > 0.0 && h > 1e-9 && l > 1e-9 {
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
                    .map(|fr| (-fr[0]).max(0.0) / (f * c.sec.area.max(1e-9)))
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
        ElementData, EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone, Section,
        WallAttr,
    };
    use squid_n_core::section_shape::SectionShape;

    /// 矩形壁（4000×3000, t=180）1 枚のみのモデル。側柱なし。
    /// `wall_attr` を指定すると `model.wall_attrs` に登録する。
    fn wall_model(wall_attr: Option<WallAttr>) -> Model {
        let mut nodes: Vec<Node> = Vec::new();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ];
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
            thickness: Some(180.0),
            shape: Some(SectionShape::RcWall {
                thickness: 180.0,
                ps: 0.006,
            }),
        }];
        let materials = vec![Material {
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
}
