//! 鉄骨鉄筋コンクリート造梁の断面検定（許容応力度検定。
//! SRC 規準 1987 の累加強度式）。
//!
//! 曲げは鉄骨・RC の許容曲げモーメントを単純累加する
//! `MA = sMo + rMA`（`sMo = sZ・sft`、`rMA = at・ft・j`）、せん断は鉄骨・RC
//! への弾性分担後にそれぞれの許容せん断力と比較する（[`super::src_shear_check`]
//! に委譲）。

use super::{ratio_or_large, src_rect_axis_props, src_shear_check, steel_h_props, SrcSeismicCtx};
use crate::material_strength::rebar_sigma_y;
use crate::rc::{concrete_allowable_shear_class, rebar_allowable_shear, rebar_allowable_tension};
use crate::steel::{steel_f_value_prefix, steel_fs, steel_ft};
use crate::{CheckComponent, CheckKind, CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::Material;
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::RcRebar;

/// SRC 梁の断面検定。曲げは `MA = sMo + rMA`（単純累加式）、せん断は
/// 鉄骨・RC への弾性分担＋各許容せん断力の比較で行う。
#[allow(clippy::too_many_arguments)]
pub(crate) fn src_beam_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    b: f64,
    d_full: f64,
    rebar: &RcRebar,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
    steel_grade: &str,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();

    // 軽量コンクリート1種・2種は許容応力度を 0.9 倍に低減（SRC規準1987。
    // `mat.concrete_class` を考慮した class 対応版を使用）。
    let fs = concrete_allowable_shear_class(fc_raw, mat.concrete_class, long_term);
    let shear_grade = rebar
        .shear
        .grade
        .clone()
        .unwrap_or_else(|| grade.to_string());
    let w_ft = rebar_allowable_shear(&shear_grade, long_term);
    let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

    let thickness = steel_web_thick.max(steel_flange_thick);
    let f_value = steel_f_value_prefix(steel_grade, thickness).unwrap_or(235.0);
    let s_ft = steel_ft(f_value, ctx.term);
    let s_fs = steel_fs(f_value, ctx.term);

    let (_sa, sz, _sz_weak) = steel_h_props(
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    );

    let props = src_rect_axis_props(b, d_full, &rebar.main_x, rebar);

    // 累加強度式（SRC 規準1987）: MA = sMo（鉄骨単体の許容曲げモーメント）
    // + rMA（RC 部分の許容曲げモーメント）。
    let s_mo = sz * s_ft;
    let r_ma = props.at * ft * props.j;
    let ma = s_mo + r_ma;

    let ratio_m = ratio_or_large(forces.mz, ma);

    let (m_alpha, q_alpha) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let b_prime = (b - steel_width).max(0.0);
    let dw = steel_height - 2.0 * steel_flange_thick;

    // 地震時短期の設計用せん断力（構造規定方式）: rMu は両端同一断面・
    // 対称配筋（at=ac）の仮定で `rc_mu_simple` により算定する。
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let r_mu = rc_mu_simple(&RcCapacityInput {
        b: props.b,
        d: props.d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y: rebar_sigma_y(mat),
        fc: fc_raw,
        pw: props.pw,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    });
    let seismic = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 1,
        s_ft_short,
        r_mu,
    };

    let shear = src_shear_check(
        forces.qy,
        m_alpha,
        q_alpha,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        b_prime,
        props.pw,
        fs,
        w_ft,
        s_fs,
        steel_web_thick * dw,
        2.0,
        &super::SrcShearMode::Beam,
        &seismic,
    );

    let ratio = ratio_m.max(shear.ratio);

    let basis = "SRC規準(1987) 梁: 累加強度式(曲げ)+ せん断弾性分担".to_string();
    let qd_note = if shear.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let detail = format!(
        "sMo={:.1} N·mm, rMA={:.1} N·mm, MA={:.1} N·mm, |mz|={:.1} N·mm, \
         sQ={:.1} N, rQ={:.1} N, sQA={:.1} N, rQA={:.1} N, α={:.3}, pw={:.5}, \
         設計用せん断力={qd_note}",
        s_mo,
        r_ma,
        ma,
        forces.mz,
        shear.s_q,
        shear.r_q,
        shear.s_qa,
        shear.r_qa,
        shear.alpha,
        shear.pw
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
        components: vec![
            CheckComponent {
                kind: CheckKind::Bending,
                ratio: ratio_m,
            },
            CheckComponent {
                kind: CheckKind::Shear,
                ratio: shear.ratio,
            },
        ],
    }
}

// ============================================================================
// テスト
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::srrc::tests::{ctx_beam, make_material, make_section, src_rect_shape, zero_forces};
    use crate::DesignCheck;
    use squid_n_core::section_shape::SectionShape;

    #[test]
    fn test_src_beam_moment_handcalc() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let sec = make_section(shape.clone());
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);

        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 22.0, true);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft = steel_ft(f_value, LoadTerm::Long);
        let expected_ma = sz * s_ft + props.at * ft * props.j;

        let forces = MemberForcesAt {
            mz: expected_ma * 0.5,
            ..zero_forces()
        };
        let design = crate::SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!((result.ratio - 0.5).abs() < 1e-6, "ratio={}", result.ratio);
        assert!(result.basis.contains("SRC規準"));

        // components に Bending・Shear が入り、最大値が ratio と一致する。
        assert_eq!(result.components.len(), 2);
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::Bending));
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
