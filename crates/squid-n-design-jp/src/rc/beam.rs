//! 鉄筋コンクリート造梁の断面検定（RC 規準13条: 曲げ・せん断・付着）。
//!
//! 強軸曲げ（`mz`）とそれに対のせん断（`qy`）のみを検定する（RC 規準の
//! 梁断面検定の対象と一致）。付着の検定は [`super::bond`] へ委譲する。

use super::{
    circle_axis_props, effective_damage_control, high_strength_w_ft, rc_allow, rc_beam_bond_check,
    rebar_allowable_tension, rebar_sigma_y, rect_axis_props_strong, seismic_design_shear,
    shear_alpha, shear_capacity_for, AxisProps,
};
use crate::{CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

// ============================================================================
// 梁の曲げ耐力（RC 規準 13条）
// ============================================================================

pub(crate) struct BeamMoment {
    /// 引張鉄筋支配の許容曲げモーメント MA_t = at・ft・j。
    pub(crate) ma_t: f64,
    /// 圧縮縁コンクリート支配の許容曲げモーメント MA_c。
    pub(crate) ma_c: f64,
    /// MA = min(MA_t, MA_c)。
    pub(crate) ma: f64,
}

/// 梁の許容曲げモーメント MA を算定する（RC 規準 13条）。
///
/// `MA_t = at・ft・j` は引張鉄筋が ft に達する状態（pt が釣合鉄筋比以下）の
/// 許容曲げモーメント。`MA_c` は複筋断面の弾性（全ひび割れ断面）解析により
/// 圧縮縁コンクリート応力度が fc に達するモーメントで、pt が釣合鉄筋比を
/// 超える（圧縮側支配）場合に効く。中立軸位置 xn を
/// `b・xn²/2 + (n-1)・ac・(xn-dc) = n・at・(d-xn)`（dc=dt）から解き、
/// `Icr = b・xn³/3 + (n-1)・ac・(xn-dc)² + n・at・(d-xn)²`、
/// `MA_c = fc・Icr/xn` とする。
///
/// `MA = min(MA_t, MA_c)` をとることで、RC 規準の
/// 「pt <= pt_balance なら C1（引張支配）、それを超えれば C2（圧縮支配）」
/// という分岐と等価な結果が得られる（過小配筋では MA_c が大きく MA_t が支配、
/// 過大配筋では逆になる）。
pub(crate) fn beam_moment_capacity(
    props: &AxisProps,
    ft: f64,
    fc: f64,
    n_ratio: f64,
) -> BeamMoment {
    let ma_t = props.at * ft * props.j;

    let dc = props.dt;
    let d = props.d;
    let b = props.b;
    let ac = props.ac;
    let at = props.at;

    let a_coef = b / 2.0;
    let b_coef = (n_ratio - 1.0) * ac + n_ratio * at;
    let c_coef = -((n_ratio - 1.0) * ac * dc + n_ratio * at * d);

    let ma_c = if a_coef > 0.0 {
        let disc = b_coef * b_coef - 4.0 * a_coef * c_coef;
        if disc >= 0.0 {
            let xn = (-b_coef + disc.sqrt()) / (2.0 * a_coef);
            if xn > 0.0 {
                let icr = b * xn.powi(3) / 3.0
                    + (n_ratio - 1.0) * ac * (xn - dc).powi(2)
                    + n_ratio * at * (d - xn).powi(2);
                fc * icr / xn
            } else {
                f64::INFINITY
            }
        } else {
            f64::INFINITY
        }
    } else {
        f64::INFINITY
    };

    BeamMoment {
        ma_t,
        ma_c,
        ma: ma_t.min(ma_c),
    }
}

// ============================================================================
// DesignCheck 実装（梁）
// ============================================================================

/// 梁の断面検定（RC 規準 13条）。強軸曲げ mz とそれに対のせん断 qy のみを扱う。
pub(crate) fn beam_check(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    shape: &SectionShape,
    fc_raw: f64,
) -> CheckResult {
    let rebar = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        SectionShape::RcCircle { rebar, .. } => rebar,
        _ => unreachable!(),
    };
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();
    let mut allow = rc_allow(fc_raw, mat.concrete_class, grade, long_term);
    let shear_grade = rebar.shear.grade.as_deref();
    if let Some(g) = shear_grade {
        // 高強度せん断補強筋: w_ft は製品表から求め直す（主筋グレードとは独立）。
        allow.w_ft = high_strength_w_ft(g, long_term);
    }

    let props = if let SectionShape::RcCircle { d, .. } = shape {
        circle_axis_props(*d, rebar)
    } else {
        rect_axis_props_strong(sec, rebar)
    };
    let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

    let bm = beam_moment_capacity(&props, ft, allow.fc, allow.n_ratio);
    let ratio_m = if bm.ma > 0.0 {
        forces.mz.abs() / bm.ma
    } else {
        0.0
    };

    let (m_for_alpha, q_for_alpha) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let alpha = shear_alpha(m_for_alpha, q_for_alpha, props.d, 2.0);
    let damage_control =
        effective_damage_control(ctx.rc_damage_control, shear_grade, mat.concrete_class);
    let qa = shear_capacity_for(
        &props,
        &allow,
        alpha,
        ctx.term,
        damage_control,
        false,
        shear_grade,
        fc_raw,
    );
    // 地震時短期は設計用せん断力 QD = min(QL+ΣBMy/l′, QL+n・QE) を用いる
    // （ctx.seismic_qd が None のときは解析せん断力のまま）。
    // ΣBMy は両端とも同一断面・対称配筋（at=ac）の仮定で 2・Mu とする。
    // Mu にスラブ筋は考慮しない。
    let q_design = if ctx.seismic_qd.is_some() {
        let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
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
        };
        let sum_mu = 2.0 * squid_n_core::rc_capacity::rc_mu_simple(&mu_inp);
        seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu, false)
    } else {
        forces.qy.abs()
    };
    let ratio_q = if qa > 0.0 { q_design / qa } else { 0.0 };

    // (B) RC 梁付着の断面検定（RC 規準1999、通し筋・カットオフ無しを仮定）。
    let bond = rc_beam_bond_check(
        forces.pos,
        ctx.length,
        props.b,
        props.d,
        props.j,
        props.at,
        forces.mz.abs(),
        &rebar.main_x,
        rebar,
        fc_raw,
        long_term,
    );
    let ratio_bond = bond.as_ref().map(|b| b.ratio).unwrap_or(0.0);

    let ratio = ratio_m.max(ratio_q).max(ratio_bond);
    let basis = "RC 規準13条（梁の曲げ・せん断・付着）".to_string();
    let bond_detail = match &bond {
        Some(b) => format!(
            ", ld={:.1} mm, ldb={:.1} mm, K={:.3}, W={:.3} mm, fb={:.3} N/mm², \
             σt={:.1} N/mm², 付着検定比={:.3}（{}）",
            b.ld,
            b.ldb,
            b.k,
            b.w,
            b.fb,
            b.sigma_t,
            b.ratio,
            if b.is_end {
                "端部・上端筋"
            } else {
                "中央・下端筋"
            }
        ),
        None => ", 付着検定: 部材長(柱面間距離)が未設定のため省略".to_string(),
    };
    let shear_grade_detail = match shear_grade {
        Some(g) => format!(", 高強度せん断補強筋={g}, w_ft={:.1} N/mm²", allow.w_ft),
        None => String::new(),
    };
    let detail = format!(
        "MA_t={:.1} N·mm, MA_c={:.1} N·mm, MA={:.1} N·mm, |mz|={:.1} N·mm, \
         QA={:.1} N, |qy|={:.1} N, α={:.3}, pw={:.5}, at={:.1} mm², d={:.1} mm, j={:.1} mm{}{}",
        bm.ma_t,
        bm.ma_c,
        bm.ma,
        forces.mz,
        qa,
        forces.qy,
        alpha,
        props.pw,
        props.at,
        props.d,
        props.j,
        shear_grade_detail,
        bond_detail,
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

// ============================================================================
// テスト（梁の曲げ・せん断・付着の統合、RcDesign 経由の梁検定）
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::tests::{
        ctx_beam, make_material, make_section, rc_rect_shape, rc_rect_shape_with_shear_grade,
    };
    use crate::DesignCheck;
    use squid_n_core::section_shape::SectionShape;
    use squid_n_core::units::ConcreteClass;

    fn make_material_class(fc: f64, grade: &str, class: ConcreteClass) -> Material {
        Material {
            concrete_class: class,
            ..make_material(fc, grade)
        }
    }

    #[test]
    fn test_beam_check_seismic_qd_increases_shear_ratio() {
        use crate::{QdMethod, SeismicQd};
        // 短期・地震時: QL=20kN、当該組合せ Q=60kN → QE=40kN、
        // QD2 = 20+1.5×40 = 80kN（QD1 は ΣMy が大きく効かないよう長スパン）。
        let mat = make_material(24.0, "SD345");
        let shape = rc_rect_shape(400.0, 700.0, 6, 22.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape.clone());
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 60_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 10.0e6,
        };
        let ctx_plain = DesignCtx {
            term: LoadTerm::Short,
            kind: crate::MemberKind::Beam,
            length: 6000.0,
            ..Default::default()
        };
        let base = beam_check(&forces, &sec, &mat, &ctx_plain, &shape, 24.0);
        let ctx_qd = DesignCtx {
            term: LoadTerm::Short,
            kind: crate::MemberKind::Beam,
            length: 6000.0,
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, 20_000.0, 0.0, 0.0, 0.0, 0.0])],
                n_factor: 1.5,
                clear_length: 6000.0,
                method: QdMethod::Qd2,
            }),
            ..Default::default()
        };
        let with_qd = beam_check(&forces, &sec, &mat, &ctx_qd, &shape, 24.0);
        // QD=80kN > 解析値 60kN なのでせん断検定比が 4/3 倍になる
        // （曲げ・付着が支配しない前提の内力設定）。
        assert!(
            with_qd.ratio > base.ratio,
            "with_qd={} <= base={}",
            with_qd.ratio,
            base.ratio
        );
    }

    // ------------------------------------------------------------------
    // 梁の曲げ
    // ------------------------------------------------------------------

    #[test]
    fn test_beam_moment_light_reinforcement_tension_governs() {
        // 軽配筋（1段筋）: MA_t が支配するはず。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = super::super::rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 19.0, true);
        let fc = super::super::concrete_allowable_compression(24.0, true);
        let n_ratio = super::super::young_ratio_n(24.0);
        let bm = beam_moment_capacity(&props, ft, fc, n_ratio);

        let expected_ma_t = props.at * ft * props.j;
        assert!((bm.ma_t - expected_ma_t).abs() < 1e-6);
        assert!(bm.ma_t <= bm.ma_c, "軽配筋では MA_t が支配するはず");
        assert!((bm.ma - bm.ma_t).abs() < 1e-6);
    }

    #[test]
    fn test_beam_moment_heavy_reinforcement_compression_governs() {
        // 過大配筋（多段・多本数）: MA_c が MA_t を下回り支配するはず。
        let shape = rc_rect_shape(300.0, 600.0, 20, 32.0, 4, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = super::super::rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 32.0, true);
        let fc = super::super::concrete_allowable_compression(24.0, true);
        let n_ratio = super::super::young_ratio_n(24.0);
        let bm = beam_moment_capacity(&props, ft, fc, n_ratio);

        assert!(bm.ma_c < bm.ma_t, "過大配筋では MA_c が支配するはず");
        assert!((bm.ma - bm.ma_c).abs() < 1e-6);
    }

    #[test]
    fn test_beam_check_via_design_check_trait() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 20_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 30_000_000.0,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio > 0.0);
        assert!(result.basis.contains("13条"));
    }

    #[test]
    fn test_beam_check_high_strength_grade_reflected_in_detail() {
        let shape =
            rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "KH785");
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Short);
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 20_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 5_000_000.0,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.detail.contains("KH785"));
        assert!(result.detail.contains("w_ft=590"));
    }

    /// 軽量1種の RcDesign 検定は、普通コンクリートより検定比が大きくなる
    /// （fc・fs の 0.9 倍低減が `mat.concrete_class` 経由で効いている）。
    #[test]
    fn test_beam_check_lightweight_reduces_capacity() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat_n = make_material(24.0, "SD345");
        let mat_l = make_material_class(24.0, "SD345", ConcreteClass::Lightweight1);
        let ctx = ctx_beam(LoadTerm::Short);
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 100_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 5_000_000.0,
        };
        let design = crate::rc::RcDesign;
        let r_n = design.check(&forces, &sec, &mat_n, &ctx);
        let r_l = design.check(&forces, &sec, &mat_l, &ctx);
        assert!(
            r_l.ratio > r_n.ratio,
            "軽量1種は許容応力度低減により検定比が大きくなるはず: normal={}, light={}",
            r_n.ratio,
            r_l.ratio
        );
    }

    /// 軽量 + 高強度フープの梁検定は、損傷制御指定でも安全確保式で算定される
    /// （damage_control=true/false で結果が一致する）。普通コンクリートでは
    /// 両者は異なる（回帰）。
    #[test]
    fn test_beam_check_lightweight_high_strength_forces_safety_formula() {
        let shape =
            rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "KH785");
        let sec = make_section(shape);
        let mat_l = make_material_class(24.0, "SD345", ConcreteClass::Lightweight1);
        let mat_n = make_material(24.0, "SD345");
        let mut ctx_damage = ctx_beam(LoadTerm::Short);
        ctx_damage.rc_damage_control = true;
        let mut ctx_safety = ctx_beam(LoadTerm::Short);
        ctx_safety.rc_damage_control = false;
        // せん断支配になるよう大きな qy を与える。
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 300_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 1_000_000.0,
        };
        let design = crate::rc::RcDesign;

        let r_l_damage = design.check(&forces, &sec, &mat_l, &ctx_damage);
        let r_l_safety = design.check(&forces, &sec, &mat_l, &ctx_safety);
        assert!(
            (r_l_damage.ratio - r_l_safety.ratio).abs() < 1e-12,
            "軽量+高強度は損傷制御指定でも安全確保式: damage={}, safety={}",
            r_l_damage.ratio,
            r_l_safety.ratio
        );

        let r_n_damage = design.check(&forces, &sec, &mat_n, &ctx_damage);
        let r_n_safety = design.check(&forces, &sec, &mat_n, &ctx_safety);
        assert!(
            (r_n_damage.ratio - r_n_safety.ratio).abs() > 1e-9,
            "普通コンクリートでは損傷制御式と安全確保式は異なるはず（回帰）"
        );
    }

    // ------------------------------------------------------------------
    // (B) RC 梁付着の断面検定（beam_check 経由の統合確認、詳細は
    // rc/bond.rs のテストを参照）
    // ------------------------------------------------------------------

    #[test]
    fn test_beam_check_via_design_check_trait_includes_bond_detail() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let mut ctx = ctx_beam(LoadTerm::Short);
        ctx.length = 3000.0;
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 20_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 30_000_000.0,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.detail.contains("ld="));
        assert!(result.detail.contains("付着検定比"));
    }

    #[test]
    fn test_beam_check_bond_skipped_without_length_regression() {
        // ctx.length（Lo 代用値）が既定の 0.0 のままなら付着検定は省略され、
        // 既存の（付着検定導入前の）挙動から曲げ・せん断比が変化しないこと。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1_000_000.0,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.detail.contains("省略"));
    }
}
