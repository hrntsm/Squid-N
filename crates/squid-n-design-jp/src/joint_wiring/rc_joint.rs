//! RC 柱梁接合部（許容応力度・終局）のせん断検定配線。

use super::common::{rc_dt, MemberInfo};
use crate::rc::joint::{rc_joint_shear_check, JointShape, RcJointInput};
use crate::{CheckComponent, CheckKind, CheckResult};
use squid_n_core::ids::NodeId;
use squid_n_core::section_shape::SectionShape;

/// RC 柱梁接合部（許容応力度・終局）の検定を `out` へ追加する。
pub(super) fn check_rc_joint(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    // ── RC 柱梁接合部 ────────────────────────────────────────
    let rc_col = cols.iter().find(|c| {
        matches!(c.sec.shape, Some(SectionShape::RcRect { .. })) && c.mat.fc.unwrap_or(0.0) > 0.0
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
        // Qdj1 の ΣMy は「大梁の降伏モーメント」の和（技術基準解説書
        // Qdj1 = ΣMy/j・(1−ξ)）。弾性解析の梁端モーメントではなく、
        // 梁の QD1 と同じ略算降伏モーメント（rc_mu_simple、対称配筋・
        // スラブ筋非考慮）を用いる。RcRect でない梁（情報不足）は
        // 従来どおり弾性端モーメントで代用する。
        let sum_beam_moments: f64 = rc_beams
            .iter()
            .map(|b| {
                if let Some(SectionShape::RcRect {
                    b: bw,
                    d,
                    ref rebar,
                    ..
                }) = b.sec.shape
                {
                    let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
                    let dt = rc_dt(rebar);
                    let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                        b: bw,
                        d,
                        at,
                        d_eff: d - dt,
                        sigma_y: crate::material_strength::rebar_sigma_y(b.mat),
                        fc: b.mat.fc.unwrap_or(0.0),
                        pw: 0.0,
                        sigma_wy: 0.0,
                        clear_span: 0.0,
                        sigma_0: 0.0,
                    };
                    squid_n_core::rc_capacity::rc_mu_simple(&mu_inp)
                } else {
                    b.end_forces(nid).map(|f| f[5].abs()).unwrap_or(0.0)
                }
            })
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

        // ── RC 柱梁接合部の終局検定（Vju/Qdu）───────
        // 接合部有効幅 bj = bb + 2·bai。終局検定の bai は bi/2 と D/4 の
        // **小さい方**（許容応力度検定の「大きい方」とは規定が異なる。
        // 終局は靭性保証型指針系の有効幅で、小さい方が安全側）。
        let bi = (col.sec.width - beam0.sec.width) / 2.0;
        let bai = (bi / 2.0).min(col.sec.depth / 4.0).max(0.0);
        let bj = beam0.sec.width + 2.0 * bai;
        // 上端・下端鉄筋引張力 T・T′。梁の main_x（せい方向主筋）を上下対称配筋
        // と仮定し、片側（総断面積の半分）が降伏引張力を負担するとみなす。
        // スラブ筋の寄与は本配線では未加算（モデルに接合部位置のスラブ筋情報が
        // 無いため。T にスラブ筋を含める場合と比べ Qdu を安全側に過小評価しうる）。
        let (t_top, t_bottom) = if let Some(SectionShape::RcRect { rebar, .. }) = &beam0.sec.shape {
            let half_area = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
            let sigma_y = crate::material_strength::rebar_sigma_y(beam0.mat);
            (half_area * sigma_y, half_area * sigma_y)
        } else {
            (0.0, 0.0)
        };
        // 上下柱の存在せん断力の平均 Qcu（存在応力の場合）。
        let col_shears: Vec<f64> = cols
            .iter()
            .filter_map(|c| c.end_forces(nid))
            .map(|f| f[1].abs().max(f[2].abs()))
            .collect();
        let qcu = if col_shears.is_empty() {
            0.0
        } else {
            col_shears.iter().sum::<f64>() / col_shears.len() as f64
        };
        // 直交梁の有無による補正係数 φ（両側直交梁付き=1.0、上記外=0.85）。
        // 節点に取り付く水平梁が 4 本以上（2 方向×両側）なら両側直交梁付きと
        // みなす簡略判定とする。
        let phi = if beams.len() >= 4 { 1.0 } else { 0.85 };
        let u = crate::ultimate::rc_joint_ultimate(&crate::ultimate::RcJointUltimateInput {
            shape,
            phi,
            fc: col.mat.fc.unwrap_or(0.0),
            bj,
            dj: col.sec.depth,
            t_top,
            t_bottom,
            qcu,
            alpha: 1.0,
        });
        let ratio = if u.vju > 0.0 {
            u.qdu / u.vju
        } else {
            f64::INFINITY
        };
        out.push((
                nid,
                "接合部終局(RC)".to_string(),
                CheckResult {
                    ratio,
                    ok: ratio <= 1.0,
                    basis: "靭性保証型指針 柱梁接合部終局(Vju=κ·φ·Fj·bj·Dj)".to_string(),
                    detail: format!(
                        "κ={:.2}, φ={:.2}, Fj={:.3} N/mm², bj={:.1} mm, Dj={:.1} mm, \
                         Vju={:.1} N, T={:.1} N, T′={:.1} N, Qcu={:.1} N, Qdu={:.1} N, 余裕率={:.3}",
                        u.kappa, phi, u.fj, bj, col.sec.depth, u.vju, t_top, t_bottom, qcu, u.qdu, u.margin
                    ),
                    components: vec![CheckComponent {
                        kind: CheckKind::Shear,
                        ratio,
                    }],
                },
            ));
    }
}
