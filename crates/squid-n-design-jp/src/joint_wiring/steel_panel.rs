//! S 造パネルゾーンのせん断検定配線。

use super::common::{is_steel, MemberInfo};
use crate::steel::panel_zone::{s_panel_zone_check, PanelSection, SPanelInput};
use crate::CheckResult;
use squid_n_core::ids::NodeId;
use squid_n_core::section_shape::SectionShape;

/// S 造パネルゾーンの検定を `out` へ追加する。
pub(super) fn check_s_panel(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
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
                ..
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
                    Some(SectionShape::SteelH { flange_thick, .. }) => b.sec.depth - flange_thick,
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
            let inp = SPanelInput {
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
}
