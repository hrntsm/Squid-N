//! 冷間成形角形鋼管（BCR/BCP）柱の柱梁耐力比検定配線。

use super::common::{is_steel, ForcesAt, MemberInfo};
use crate::steel::cold_formed::{
    box_zp, cold_formed_column_ratio_check, panel_mpp, ColdFormedInput,
};
use crate::CheckResult;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::section_shape::SectionShape;

/// 冷間成形角形鋼管（BCR/BCP）判定。
fn is_cold_formed(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("BCR") || upper.starts_with("BCP")
}

/// H 形鋼の塑性断面係数（強軸）Zp = B·tf·(H−tf) + tw·(H−2tf)²/4。
fn h_zp(h: f64, b: f64, tw: f64, tf: f64) -> f64 {
    b * tf * (h - tf) + tw * (h - 2.0 * tf).powi(2) / 4.0
}

/// 冷間成形角形鋼管（BCR/BCP）柱の柱梁耐力比検定を `out` へ追加する。
pub(super) fn check_cold_formed(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    long_member_forces: Option<&[(ElemId, ForcesAt<'_>)]>,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
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
                    ..
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
                    // 圧縮正に変換して存在軸力を求める（存在軸力 N = NL+1.5・NE）。
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
                    | Some(SectionShape::CftBox { height, thick, .. }) => (height - thick, thick),
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
                // パネル軸力比 n は「上柱、下柱軸力の平均から計算する」
                // （冷間成形角形鋼管設計・施工マニュアル■パネル耐力。片側しか存在しない場合は
                // 存在する柱の値がそのまま平均になる）。
                let n_panel = (n_u + n_l) / 2.0;
                let mpp = panel_mpp(dc, db, tp, f_l, n_panel);
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
