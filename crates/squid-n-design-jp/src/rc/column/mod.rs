//! 鉄筋コンクリート造柱の断面検定（RC 規準14条: 軸力・軸力+曲げ・せん断）。
//!
//! 軸力（M=0）・軸力＋二軸曲げ・二方向せん断を検定する。矩形柱は強軸・弱軸
//! それぞれの N-M 相関曲線を構成して二軸曲げを線形和で評価し、円形柱は
//! 等価矩形断面に置換して同じ手順を適用する。

use super::{
    bar_set_area, circle_axis_props, effective_damage_control, high_strength_w_ft, one_bar_area,
    rc_allow, rebar_allowable_tension, rebar_sigma_y, rect_axis_props_strong, rect_axis_props_weak,
    seismic_design_shear, shear_alpha, shear_capacity_for, AxisProps,
};
use crate::{CheckComponent, CheckKind, CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

mod nm_interaction;

use nm_interaction::*;

// ============================================================================
// DesignCheck 実装（柱）
// ============================================================================

/// 柱の断面検定（RC 規準 14条）。軸力・軸力+二軸曲げ・二方向せん断を扱う。
pub(crate) fn column_check(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    shape: &SectionShape,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();
    let mut allow = rc_allow(fc_raw, mat.concrete_class, grade, long_term);

    // 圧縮を正とする設計軸力（forces.n は引張正・圧縮負）。
    let n_design = -forces.n;

    if let SectionShape::RcCircle { d, rebar } = shape {
        let shear_grade = rebar.shear.grade.as_deref();
        if let Some(g) = shear_grade {
            // 高強度せん断補強筋: w_ft は製品表から求め直す（主筋グレードとは独立）。
            allow.w_ft = high_strength_w_ft(g, long_term);
        }
        let damage_control =
            effective_damage_control(ctx.rc_damage_control, shear_grade, mat.concrete_class);
        let d_full = *d;
        let props = circle_axis_props(d_full, rebar);
        let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

        let gross_area = std::f64::consts::PI * d_full * d_full / 4.0;
        let as_total = rebar.main_x.count as f64 * one_bar_area(rebar.main_x.dia);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis = ColumnAxis {
            props,
            at_perp: 0.0,
            ft,
        };
        let curve = column_nm_curve(&axis, &allow, na);
        let ma = interp_ma(&curve, n_design);

        let ratio_axial = if forces.n < 0.0 && na > 0.0 {
            (-forces.n) / na
        } else {
            0.0
        };
        let ratio_moment = if ma > 0.0 {
            (forces.mz / ma).powi(2) + (forces.my / ma).powi(2)
        } else {
            0.0
        };

        let (m_for_alpha_y, q_for_alpha_y) =
            ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
        let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis.props.d, 1.5);
        let qay = shear_capacity_for(
            &axis.props,
            &allow,
            alpha_y,
            ctx.term,
            damage_control,
            true,
            shear_grade,
            fc_raw,
        );
        // 地震時短期は設計用せん断力 QD = min(ΣcMy/h′, QL+n・QE) を用いる。
        // 円形柱の ΣcMy は等価幅 b_eq = A/D の矩形として柱 Mu 閉形式で近似する。
        let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
            let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                b: gross_area / d_full,
                d: d_full,
                at: axis.props.at,
                d_eff: axis.props.d,
                sigma_y: rebar_sigma_y(mat),
                fc: fc_raw,
                pw: axis.props.pw,
                sigma_wy: 0.0,
                clear_span: 0.0,
                sigma_0: 0.0,
            };
            let sum_mu =
                2.0 * squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design);
            (
                seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu, true),
                seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu, true),
            )
        } else {
            (forces.qy.abs(), forces.qz.abs())
        };
        let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

        let (m_for_alpha_z, q_for_alpha_z) = ctx
            .shear_span_y
            .unwrap_or((forces.my.abs(), forces.qz.abs()));
        let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis.props.d, 1.5);
        let qaz = shear_capacity_for(
            &axis.props,
            &allow,
            alpha_z,
            ctx.term,
            damage_control,
            true,
            shear_grade,
            fc_raw,
        );
        let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

        let ratio = ratio_axial.max(ratio_moment).max(ratio_qy).max(ratio_qz);

        let basis = "RC 規準14条（円形柱、等価矩形近似）".to_string();
        let detail = format!(
            "NA={:.1} N, N={:.1} N, MA={:.1} N·mm（等価矩形近似）, mz={:.1} N·mm, my={:.1} N·mm, \
             QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw={:.5}, at={:.1} mm², d={:.1} mm",
            na,
            n_design,
            ma,
            forces.mz,
            forces.my,
            qay,
            qaz,
            alpha_y,
            alpha_z,
            axis.props.pw,
            axis.props.at,
            axis.props.d
        );

        let components = vec![
            CheckComponent {
                kind: CheckKind::AxialBending,
                ratio: ratio_axial.max(ratio_moment),
            },
            CheckComponent {
                kind: CheckKind::Shear,
                ratio: ratio_qy.max(ratio_qz),
            },
        ];

        return CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis,
            detail,
            components,
        };
    }

    let rebar = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        _ => unreachable!(),
    };
    let shear_grade = rebar.shear.grade.as_deref();
    if let Some(g) = shear_grade {
        // 高強度せん断補強筋: w_ft は製品表から求め直す（主筋グレードとは独立）。
        allow.w_ft = high_strength_w_ft(g, long_term);
    }
    let damage_control =
        effective_damage_control(ctx.rc_damage_control, shear_grade, mat.concrete_class);

    let props_z = rect_axis_props_strong(sec, rebar); // mz 方向
    let props_y = rect_axis_props_weak(sec, rebar); // my 方向
    let ft_z = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);
    let ft_y = rebar_allowable_tension(grade, rebar.main_y.dia, long_term);

    let gross_area = sec.width * sec.depth;
    let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    // NA 用の ft は D29 以上の低減を保守的に反映するため、両方向のうち
    // 大径側（許容応力度が低い方）を採用する。
    let ft_axial =
        rebar_allowable_tension(grade, rebar.main_x.dia.max(rebar.main_y.dia), long_term);
    let na = column_axial_capacity(gross_area, as_total, allow.fc, ft_axial, allow.n_ratio);

    let at_perp_for_z = bar_set_area(&rebar.main_y);
    let at_perp_for_y = bar_set_area(&rebar.main_x);

    let axis_z = ColumnAxis {
        props: props_z,
        at_perp: at_perp_for_z,
        ft: ft_z,
    };
    let axis_y = ColumnAxis {
        props: props_y,
        at_perp: at_perp_for_y,
        ft: ft_y,
    };

    let curve_z = column_nm_curve(&axis_z, &allow, na);
    let curve_y = column_nm_curve(&axis_y, &allow, na);
    let ma_z = interp_ma(&curve_z, n_design);
    let ma_y = interp_ma(&curve_y, n_design);

    let ratio_axial = if forces.n < 0.0 && na > 0.0 {
        (-forces.n) / na
    } else {
        0.0
    };
    let ratio_z = if ma_z > 0.0 {
        forces.mz.abs() / ma_z
    } else {
        0.0
    };
    let ratio_y = if ma_y > 0.0 {
        forces.my.abs() / ma_y
    } else {
        0.0
    };
    let ratio_moment = ratio_z + ratio_y;

    let (m_for_alpha_y, q_for_alpha_y) =
        ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis_z.props.d, 1.5);
    let qay = shear_capacity_for(
        &axis_z.props,
        &allow,
        alpha_y,
        ctx.term,
        damage_control,
        true,
        shear_grade,
        fc_raw,
    );
    // 地震時短期は設計用せん断力 QD = min(QD1, QD2) を用いる
    // （QD1 = ΣcMy/h′、QD2 = QL + n・QE。ctx.seismic_qd が None なら解析値）。
    // ΣcMy は柱頭・柱脚同一断面の仮定で 2・Mu（軸力考慮閉形式）とする。
    let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
        let sigma_y = rebar_sigma_y(mat);
        let mu_of = |b: f64, d_full: f64, props: &AxisProps| {
            let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                b,
                d: d_full,
                at: props.at,
                d_eff: props.d,
                sigma_y,
                fc: fc_raw,
                pw: props.pw,
                sigma_wy: 0.0,
                clear_span: 0.0,
                sigma_0: 0.0,
            };
            squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design)
        };
        let sum_mu_z = 2.0 * mu_of(sec.width, sec.depth, &axis_z.props);
        let sum_mu_y = 2.0 * mu_of(sec.depth, sec.width, &axis_y.props);
        (
            seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu_z, true),
            seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu_y, true),
        )
    } else {
        (forces.qy.abs(), forces.qz.abs())
    };
    let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

    let (m_for_alpha_z, q_for_alpha_z) = ctx
        .shear_span_y
        .unwrap_or((forces.my.abs(), forces.qz.abs()));
    let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis_y.props.d, 1.5);
    let qaz = shear_capacity_for(
        &axis_y.props,
        &allow,
        alpha_z,
        ctx.term,
        damage_control,
        true,
        shear_grade,
        fc_raw,
    );
    let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

    let ratio = ratio_axial.max(ratio_moment).max(ratio_qy).max(ratio_qz);

    let basis = "RC 規準14条（柱、軸力+二軸曲げ+せん断）".to_string();
    let detail = format!(
        "NA={:.1} N, N={:.1} N, MA_z={:.1} N·mm, MA_y={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm, \
         QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw_z={:.5}, pw_y={:.5}",
        na,
        n_design,
        ma_z,
        ma_y,
        forces.mz,
        forces.my,
        qay,
        qaz,
        alpha_y,
        alpha_z,
        axis_z.props.pw,
        axis_y.props.pw
    );

    let components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial.max(ratio_moment),
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: ratio_qy.max(ratio_qz),
        },
    ];

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
        components,
    }
}

// ============================================================================
// テスト（柱の軸力・軸力+曲げ・せん断、RcDesign 経由の柱検定）
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::beam::beam_moment_capacity;
    use crate::rc::tests::{ctx_column, make_material, make_section, rc_rect_shape};
    use crate::DesignCheck;

    #[test]
    fn test_column_axial_capacity_handcalc() {
        let fc = 8.0; // 長期許容圧縮（Fc=24 なら 8.0）
        let ft = 215.0;
        let n_ratio = 15.0;
        let gross = 400.0 * 400.0;
        let as_total = 8.0 * std::f64::consts::PI * (22.0 / 2.0f64).powi(2);
        let na = column_axial_capacity(gross, as_total, fc, ft, n_ratio);

        let ae = gross + (n_ratio - 1.0) * as_total;
        let expected = (fc * ae).min(ft * ae / n_ratio);
        assert!((na - expected).abs() < 1e-6);
    }

    #[test]
    fn test_column_n0_moment_close_to_beam_ma_t() {
        // N=0 のときの柱 MA が、対応する梁の MA_t とおおむね一致すること
        // （j≒7d/8 の近似差程度、20% 程度の許容）を確認する。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(
            24.0,
            squid_n_core::units::ConcreteClass::Normal,
            "SD345",
            true,
        );
        let ft = rebar_allowable_tension("SD345", 22.0, true);

        let props_z = rect_axis_props_strong(&sec, &rebar);
        let bm = beam_moment_capacity(&props_z, ft, allow.fc, allow.n_ratio);

        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);
        let ma_at_n0 = interp_ma(&curve, 0.0);

        let rel_diff = (ma_at_n0 - bm.ma).abs() / bm.ma;
        assert!(
            rel_diff < 0.2,
            "N=0 の柱 MA={ma_at_n0} が梁 MA={} と 20% 以上乖離",
            bm.ma
        );
    }

    #[test]
    fn test_column_moment_increases_then_decreases_with_compression() {
        // 軽配筋（N=0 では引張鉄筋支配）の断面を用いる。RC 規準14条の N-M
        // 相関曲線は一般に「引張支配の隅（大きな引張軸力・小さな M）→
        // 釣合点（M最大）→ 全断面圧縮の隅（N=NA, M=0）」という山型になる。
        // 釣合点（ピーク）の位置は配筋量に依存し、既に N=0 でコンクリート縁
        // 応力が支配する（過大配筋の）断面ではピークが引張側にずれることも
        // あるため、ここではピークが正の圧縮軸力側に来る軽配筋断面で検証する
        // （`test_beam_moment_heavy_reinforcement_compression_governs` が過大
        // 配筋側の挙動を別途カバーする）。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(
            24.0,
            squid_n_core::units::ConcreteClass::Normal,
            "SD345",
            true,
        );
        let ft = rebar_allowable_tension("SD345", 19.0, true);
        let props_z = rect_axis_props_strong(&sec, &rebar);
        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);

        let m_at_0 = interp_ma(&curve, 0.0);
        let m_at_mid = interp_ma(&curve, na * 0.3);
        let m_at_near_na = interp_ma(&curve, na * 0.98);

        assert!(
            m_at_mid > m_at_0,
            "圧縮軸力の増加で MA は一旦増加するはず: m0={m_at_0}, mid={m_at_mid}"
        );
        assert!(
            m_at_near_na < m_at_mid,
            "軸力が NA に近づくと MA は減少するはず: mid={m_at_mid}, near_na={m_at_near_na}"
        );
    }

    #[test]
    fn test_column_biaxial_linear_sum() {
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // まず微小な mz を与えて ratio から MA_z を逆算する。
        let forces_z_only = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1.0,
        };
        let design = crate::rc::RcDesign;
        let r0 = design.check(&forces_z_only, &sec, &mat, &ctx);
        let ma_z_approx = 1.0 / r0.ratio.max(1e-30);

        let mz_test = ma_z_approx * 0.3;
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: mz_test,
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(
            (r.ratio - 0.3).abs() < 0.05,
            "mz 単独 0.3 割合のとき ratio ≒ 0.3 のはず: ratio={}",
            r.ratio
        );
    }

    /// 矩形柱: components に AxialBending・Shear が含まれ、その最大値が
    /// ratio と一致することを確認する。
    #[test]
    fn test_column_check_components_axial_bending_and_shear_max_matches_ratio() {
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -200_000.0,
            qy: 50_000.0,
            qz: 30_000.0,
            my: 10.0e6,
            mz: 20.0e6,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert_eq!(result.components.len(), 2);
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::AxialBending));
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::Shear));
        let max_component = result
            .components
            .iter()
            .map(|c| c.ratio)
            .fold(0.0_f64, f64::max);
        assert_eq!(max_component, result.ratio);
    }
}
