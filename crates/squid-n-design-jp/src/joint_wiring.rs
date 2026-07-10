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
//!   開口情報が無いため無開口（r=1）とし、設計用せん断力は等価梁化された
//!   壁要素の内力の最大水平せん断成分を用いる（暫定）。

use crate::joint::{
    box_zp, cold_formed_column_ratio_check, panel_mpp, rc_joint_shear_check, rc_wall_shear_check,
    s_panel_zone_check, ColdFormedInput, JointShape, PanelSection, RcJointInput, RcWallInput,
    WallSideColumn,
};
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
        // 壁の平面寸法: 節点群の水平距離の最大 = l、鉛直 extent = h（未使用）。
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
            opening: None,
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
