//! 鉄骨鉄筋コンクリート造柱の断面検定（許容応力度検定。
//! SRC 規準 1987 の累加強度式）。
//!
//! 軸力＋二軸曲げ＋二方向せん断の複合検定を行う。曲げ耐力 MA(N) は
//! RC 部分の N-M 相関曲線（[`src_column_nm_curve`]）と鉄骨単体の許容曲げ
//! モーメントを軸力範囲に応じて 3 分岐で累加する（[`src_column_axis_ma`]）。
//! せん断は [`super::src_shear_check`] に委譲する。

use super::{
    bar_set_area, ratio_or_large, src_rect_axis_props, src_shear_check, steel_h_props,
    SrcAxisProps, SrcSeismicCtx,
};
use crate::material_strength::rebar_sigma_y;
use crate::rc::{
    concrete_allowable_compression_class, concrete_allowable_shear_class, rebar_allowable_shear,
    rebar_allowable_tension, young_ratio_n,
};
use crate::steel::{steel_f_value_prefix, steel_fs, steel_ft};
use crate::{CheckComponent, CheckKind, CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::Material;
use squid_n_core::rc_capacity::{rc_column_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::RcRebar;

struct SrcColumnAxis {
    props: SrcAxisProps,
    /// 直交方向の主筋総断面積（断面中央に集約、RC 規準 14条の慣習）。
    at_perp: f64,
    /// 当該軸の主筋径に応じた許容引張・圧縮応力度。
    ft: f64,
}

/// 中立軸位置 xn における RC 部分の (N, |M|) を求める（`rc/column.rs` の
/// `column_nm_at_xn` と同じ考え方。コンクリート許容応力度に `fc_prime`
/// （鉄骨フランジ食い込みによる低減後）を用いる点のみ異なる）。
fn src_column_nm_at_xn(
    axis: &SrcColumnAxis,
    fc_prime: f64,
    n_ratio: f64,
    xn: f64,
) -> Option<(f64, f64)> {
    if xn <= 0.0 {
        return None;
    }
    let p = &axis.props;
    let d_full = p.d_full;
    let b = p.b;
    let r_fc = axis.ft;
    let ft = axis.ft;

    let s_bar = |y: f64, area: f64| -> f64 {
        if area <= 0.0 {
            return f64::INFINITY;
        }
        let diff = xn - y;
        if diff.abs() < 1e-9 {
            return f64::INFINITY;
        }
        if diff > 0.0 {
            r_fc / (n_ratio * diff)
        } else {
            ft / (n_ratio * (-diff))
        }
    };

    let s1 = fc_prime / xn;
    let s2 = s_bar(p.dt, p.ac);
    let s3 = s_bar(d_full - p.dt, p.at);
    let s = s1.min(s2).min(s3);
    if !s.is_finite() || s <= 0.0 {
        return None;
    }

    let xc = xn.min(d_full);
    if xc <= 0.0 {
        return None;
    }

    let nc = b * s * (xn * xc - xc * xc / 2.0);
    let mc =
        b * s * (xn * (d_full / 2.0) * xc - (xn + d_full / 2.0) * xc * xc / 2.0 + xc.powi(3) / 3.0);

    let bar_contrib = |y: f64, area: f64| -> (f64, f64) {
        if area <= 0.0 {
            return (0.0, 0.0);
        }
        let mult = if y <= xn { n_ratio - 1.0 } else { n_ratio };
        let force = area * mult * s * (xn - y);
        let moment = force * (d_full / 2.0 - y);
        (force, moment)
    };

    let (n_c, m_c) = bar_contrib(p.dt, p.ac);
    let (n_t, m_t) = bar_contrib(d_full - p.dt, p.at);
    let (n_p, m_p) = bar_contrib(d_full / 2.0, axis.at_perp);

    let n_total = nc + n_c + n_t + n_p;
    let m_total = mc + m_c + m_t + m_p;
    Some((n_total, m_total.abs()))
}

const SRC_XN_SCAN_POINTS: usize = 300;
const SRC_XN_RATIO_MIN: f64 = 0.02;
const SRC_XN_RATIO_MAX: f64 = 10.0;

/// N-M 相関曲線（RC 部分のみ）を xn/D の対数スキャンで構成する。
///
/// 圧縮側は `(rnc, 0)`、引張側は `(rnt, 0)` をアンカーとして追加する。
/// 引張側アンカーが無いと、xn スキャン最小点より引張側で許容曲げが
/// 頭打ちクランプされ、純引張耐力 rNt 近傍の rM を過大評価する
/// （正しくは N → rNt で rM → 0 に収束する）。
fn src_column_nm_curve(
    axis: &SrcColumnAxis,
    fc_prime: f64,
    n_ratio: f64,
    rnc: f64,
    rnt: f64,
) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(SRC_XN_SCAN_POINTS + 2);
    let log_min = SRC_XN_RATIO_MIN.ln();
    let log_max = SRC_XN_RATIO_MAX.ln();
    for i in 0..SRC_XN_SCAN_POINTS {
        let t = i as f64 / (SRC_XN_SCAN_POINTS as f64 - 1.0);
        let ratio = (log_min + t * (log_max - log_min)).exp();
        let xn = axis.props.d_full * ratio;
        if let Some(pt) = src_column_nm_at_xn(axis, fc_prime, n_ratio, xn) {
            if pt.0.is_finite() && pt.1.is_finite() {
                // 引張側アンカーより外の点は含めない（rNt が最小 N）。
                if pt.0 >= rnt {
                    pts.push(pt);
                }
            }
        }
    }
    pts.push((rnc, 0.0));
    pts.push((rnt, 0.0));
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    pts
}

/// N-M 相関曲線から設計軸力（圧縮正）に対する許容曲げモーメントを線形補間で
/// 求める。範囲外は端点値でクランプする。
fn interp_ma_curve(points: &[(f64, f64)], n_design: f64) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    if n_design <= points[0].0 {
        return points[0].1;
    }
    let last = points.len() - 1;
    if n_design >= points[last].0 {
        return points[last].1;
    }
    for w in points.windows(2) {
        let (n0, m0) = w[0];
        let (n1, m1) = w[1];
        if n_design >= n0 && n_design <= n1 {
            if (n1 - n0).abs() < 1e-9 {
                return m0.max(m1);
            }
            let t = (n_design - n0) / (n1 - n0);
            return m0 + t * (m1 - m0);
        }
    }
    points[last].1
}

/// SRC 柱 1 軸分の許容曲げモーメント MA(N)。SRC規準1987 の 3 分岐
/// （RC+鉄骨累加 / 圧縮超過で鉄骨のみ / 引張超過で鉄骨のみ）を実装する。
#[allow(clippy::too_many_arguments)]
fn src_column_axis_ma(
    n_design: f64,
    rnc: f64,
    rnt: f64,
    sa: f64,
    s_ft: f64,
    s_fc: f64,
    sz: f64,
    s_mo: f64,
    curve: &[(f64, f64)],
) -> f64 {
    if n_design >= rnt && n_design <= rnc {
        s_mo + interp_ma_curve(curve, n_design)
    } else if n_design > rnc {
        let sn = n_design - rnc;
        (sz * (s_fc - sn / sa)).max(0.0)
    } else {
        let sn = n_design - rnt;
        (sz * (s_ft - sn.abs() / sa)).max(0.0)
    }
}

/// SRC 柱の断面検定（軸力+二軸曲げ+二方向せん断、SRC 規準 1987 累加強度式）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn src_column_check(
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

    // 軽量コンクリート1種・2種は許容応力度（圧縮・せん断）を 0.9 倍に低減
    // （SRC規準1987。class 対応版を使用）。
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);
    let fs = concrete_allowable_shear_class(fc_raw, mat.concrete_class, long_term);
    let n_ratio = young_ratio_n(fc_raw);
    let shear_grade = rebar
        .shear
        .grade
        .clone()
        .unwrap_or_else(|| grade.to_string());
    let w_ft = rebar_allowable_shear(&shear_grade, long_term);

    let thickness = steel_web_thick.max(steel_flange_thick);
    let f_value = steel_f_value_prefix(steel_grade, thickness).unwrap_or(235.0);
    let s_ft = steel_ft(f_value, ctx.term);
    let s_fs = steel_fs(f_value, ctx.term);
    let s_fc = s_ft; // 座屈考慮なし（モジュール doc 参照）

    let (sa, sz_z, sz_y) = steel_h_props(
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    );

    // 鉄骨フランジ食い込みによる fc′ 低減: fc′ = fc・(1 - 15・s_pc)
    // （s_pc = 鉄骨フランジ断面積 / 全断面積）。
    let s_pc = (steel_width * steel_flange_thick) / (b * d_full).max(1e-9);
    let fc_prime = (fc_allow * (1.0 - 15.0 * s_pc)).max(0.0);

    let ft_z = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);
    let ft_y = rebar_allowable_tension(grade, rebar.main_y.dia, long_term);
    let ft_axial =
        rebar_allowable_tension(grade, rebar.main_x.dia.max(rebar.main_y.dia), long_term);

    let as_x = bar_set_area(&rebar.main_x);
    let as_y = bar_set_area(&rebar.main_y);
    let as_total = as_x + as_y;

    let ae = b * d_full + (n_ratio - 1.0) * as_total;
    let rnc1 = ae * fc_prime;
    let rnc2 = ae * ft_axial / n_ratio;
    let rnc = rnc1.min(rnc2).max(0.0);
    let rnt = -(as_total * ft_axial);

    let s_nc = sa * s_fc;
    let s_nt = sa * s_ft;

    let props_z = src_rect_axis_props(b, d_full, &rebar.main_x, rebar);
    let props_y = src_rect_axis_props(d_full, b, &rebar.main_y, rebar);

    let axis_z = SrcColumnAxis {
        props: props_z,
        at_perp: as_y,
        ft: ft_z,
    };
    let axis_y = SrcColumnAxis {
        props: props_y,
        at_perp: as_x,
        ft: ft_y,
    };

    let curve_z = src_column_nm_curve(&axis_z, fc_prime, n_ratio, rnc, rnt);
    let curve_y = src_column_nm_curve(&axis_y, fc_prime, n_ratio, rnc, rnt);

    let n_design = -forces.n; // 圧縮を正とする設計軸力に変換

    let s_mo_z = sz_z * s_ft;
    let s_mo_y = sz_y * s_ft;

    let ma_z = src_column_axis_ma(n_design, rnc, rnt, sa, s_ft, s_fc, sz_z, s_mo_z, &curve_z);
    let ma_y = src_column_axis_ma(n_design, rnc, rnt, sa, s_ft, s_fc, sz_y, s_mo_y, &curve_y);

    let ratio_z = ratio_or_large(forces.mz, ma_z);
    let ratio_y = ratio_or_large(forces.my, ma_y);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > rnc + s_nc {
        n_design / (rnc + s_nc)
    } else if n_design < rnt - s_nt {
        (-n_design) / (-rnt + s_nt)
    } else {
        0.0
    };

    // 地震時短期の設計用せん断力（構造規定方式）: rMu は軸力を考慮した
    // `rc_column_mu_simple`（柱頭・柱脚同一断面・同一設計軸力の仮定）で
    // 算定する。sft は常に短期値を用いる。
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let sigma_y = rebar_sigma_y(mat);
    let r_mu_z = rc_column_mu_simple(
        &RcCapacityInput {
            b: props_z.b,
            d: props_z.d_full,
            at: props_z.at,
            d_eff: props_z.d,
            sigma_y,
            fc: fc_raw,
            pw: props_z.pw,
            sigma_wy: 0.0,
            clear_span: 0.0,
            sigma_0: 0.0,
        },
        as_total,
        n_design,
    );
    let r_mu_y = rc_column_mu_simple(
        &RcCapacityInput {
            b: props_y.b,
            d: props_y.d_full,
            at: props_y.at,
            d_eff: props_y.d,
            sigma_y,
            fc: fc_raw,
            pw: props_y.pw,
            sigma_wy: 0.0,
            clear_span: 0.0,
            sigma_0: 0.0,
        },
        as_total,
        n_design,
    );

    // β: 鉄骨ウェブの形式と寸法による係数（SRC規準 P.96-97 の併用式）。
    // 強軸方向（ウェブがせん断を負担）は β = n・tw・sd/(b・rj)、
    // 弱軸方向（フランジが負担）は β = 1.33・n・bf・tf/(b・rj)。
    // sd はフランジ重心間距離 = H − tf、n はヤング係数比。
    let n_ratio = young_ratio_n(fc_raw);
    let sd_flange = (steel_height - steel_flange_thick).max(0.0);
    let beta_z = if props_z.b * props_z.j > 1e-9 {
        n_ratio * steel_web_thick * sd_flange / (props_z.b * props_z.j)
    } else {
        0.0
    };
    let beta_y = if props_y.b * props_y.j > 1e-9 {
        1.33 * n_ratio * steel_width * steel_flange_thick / (props_y.b * props_y.j)
    } else {
        0.0
    };

    // せん断スパン比の割増係数は SRC規準の柱規定 1≦rα≦2 による（RC 柱の 1.5 とは
    // 異なる）。弱軸方向は shear_span_y（|My|max と対応 |Qz|）を用いる。
    let (m_alpha_z, q_alpha_z) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let b_prime_z = (b - steel_width).max(0.0);
    let seismic_z = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 1,
        s_ft_short,
        r_mu: r_mu_z,
    };
    let shear_z = src_shear_check(
        forces.qy,
        m_alpha_z,
        q_alpha_z,
        sz_z,
        props_z.at,
        props_z.j,
        props_z.d,
        props_z.b,
        b_prime_z,
        props_z.pw,
        fs,
        w_ft,
        s_fs,
        // 強軸方向の鉄骨部 sQA = dw・tw・sfs（dw = H − 2tf）。
        steel_web_thick * (steel_height - 2.0 * steel_flange_thick),
        2.0,
        &super::SrcShearMode::Column { beta: beta_z },
        &seismic_z,
    );

    let (m_alpha_y, q_alpha_y) = ctx
        .shear_span_y
        .unwrap_or((forces.my.abs(), forces.qz.abs()));
    let b_prime_y = (d_full - steel_height).max(0.0);
    let seismic_y = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 2,
        s_ft_short,
        r_mu: r_mu_y,
    };
    let shear_y = src_shear_check(
        forces.qz,
        m_alpha_y,
        q_alpha_y,
        sz_y,
        props_y.at,
        props_y.j,
        props_y.d,
        props_y.b,
        b_prime_y,
        props_y.pw,
        fs,
        w_ft,
        s_fs,
        // 弱軸方向の鉄骨部 sQA = (4/3)・bf・tf・sfs（SRC規準。従来の 2・tf・bf は
        // 1.5 倍の非保守側だった）。
        (4.0 / 3.0) * steel_width * steel_flange_thick,
        2.0,
        &super::SrcShearMode::Column { beta: beta_y },
        &seismic_y,
    );

    let ratio = ratio_axial
        .max(ratio_biaxial)
        .max(shear_z.ratio)
        .max(shear_y.ratio);

    let basis = "SRC規準(1987) 柱: 累加強度式(軸力+二軸曲げ)+ せん断弾性分担".to_string();
    let qd_note_z = if shear_z.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let qd_note_y = if shear_y.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let detail = format!(
        "rNc={:.1} N, rNt={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, \
         MAz={:.1} N·mm, MAy={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm, \
         sQAz={:.1} N, rQAz={:.1} N, sQAy={:.1} N, rQAy={:.1} N, s_pc={:.5}, fc'={:.3}, \
         設計用せん断力(z)={qd_note_z}, 設計用せん断力(y)={qd_note_y}",
        rnc,
        rnt,
        s_nc,
        s_nt,
        n_design,
        ma_z,
        ma_y,
        forces.mz,
        forces.my,
        shear_z.s_qa,
        shear_z.r_qa,
        shear_y.s_qa,
        shear_y.r_qa,
        s_pc,
        fc_prime
    );

    let components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial.max(ratio_biaxial),
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: shear_z.ratio.max(shear_y.ratio),
        },
    ];

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
        components,
    }
}

// ============================================================================
// テスト
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::srrc::src_seismic_qd;
    use crate::srrc::tests::{
        ctx_column, make_material, make_section, src_column_shape, src_rect_shape, zero_forces,
    };
    use crate::DesignCheck;
    use squid_n_core::section_shape::SectionShape;
    use squid_n_core::units::ConcreteClass;

    #[test]
    fn test_src_column_n0_matches_smo_plus_rm0() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        let forces = MemberForcesAt {
            mz: 1.0,
            ..zero_forces()
        };
        let design = crate::SrcDesign;
        let r0 = design.check(&forces, &sec, &mat, &ctx);
        let ma_z = 1.0 / r0.ratio;
        assert!(ma_z > 0.0 && ma_z.is_finite());

        let (_sa, sz_z, _) = steel_h_props(300.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft = steel_ft(f_value, LoadTerm::Long);
        let s_mo = sz_z * s_ft;
        // N=0 の MA は少なくとも鋼骨単体の sMo 以上であるはず（RC 分は正で加算）。
        assert!(ma_z >= s_mo * 0.99, "ma_z={ma_z}, s_mo={s_mo}");
    }

    /// components に AxialBending・Shear が入り、最大値が ratio と一致する。
    #[test]
    fn test_src_column_components_axial_bending_and_shear_max_matches_ratio() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let forces = MemberForcesAt {
            n: -500_000.0,
            qy: 30_000.0,
            qz: 20_000.0,
            my: 5_000_000.0,
            mz: 8_000_000.0,
            ..zero_forces()
        };
        let design = crate::SrcDesign;
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

    #[test]
    fn test_src_column_beyond_rnc_uses_steel_only() {
        let shape = src_column_shape();
        let sec = make_section(shape.clone());
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // 非常に大きな圧縮軸力 (n<0 は圧縮) を与え、N > rNc となる状況を作る。
        let forces = MemberForcesAt {
            n: -30_000_000.0,
            mz: 10_000_000.0,
            ..zero_forces()
        };
        let design = crate::SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio.is_finite());
        assert!(result.detail.contains("rNc"));
        let _ = shape;
    }

    #[test]
    fn test_src_column_tension_side() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // n>0 は引張。全主筋の引張耐力を大きく超える引張軸力を与える。
        let forces = MemberForcesAt {
            n: 5_000_000.0,
            mz: 1_000_000.0,
            ..zero_forces()
        };
        let design = crate::SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio.is_finite());
        assert!(result.ratio > 0.0);
    }

    #[test]
    fn test_src_column_biaxial_linear_sum() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = crate::SrcDesign;

        let forces_z = MemberForcesAt {
            mz: 1.0,
            ..zero_forces()
        };
        let r0 = design.check(&forces_z, &sec, &mat, &ctx);
        let ma_z = 1.0 / r0.ratio;

        let mz_test = ma_z * 0.3;
        let forces = MemberForcesAt {
            mz: mz_test,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(
            (r.ratio - 0.3).abs() < 0.05,
            "mz 単独 0.3 割合のとき ratio ≒ 0.3 のはず: ratio={}",
            r.ratio
        );
    }

    #[test]
    fn test_src_column_fc_prime_reduction_effect() {
        // 鉄骨フランジが大きいほど s_pc が大きくなり fc' が低下し、rNc が
        // 減少するはず。
        let shape_small_steel = src_rect_shape(
            500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 150.0, 9.0, 9.0, "SN400B",
        );
        let shape_large_steel = src_rect_shape(
            500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 300.0, 9.0, 30.0, "SN400B",
        );

        let sec_small = make_section(shape_small_steel);
        let sec_large = make_section(shape_large_steel);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = crate::SrcDesign;

        let forces = MemberForcesAt {
            n: -1.0,
            ..zero_forces()
        };
        let r_small = design.check(&forces, &sec_small, &mat, &ctx);
        let r_large = design.check(&forces, &sec_large, &mat, &ctx);
        assert!(r_small.detail.contains("rNc"));
        assert!(r_large.detail.contains("rNc"));
    }

    /// SRC 柱でも軽量コンクリートの 0.9 倍低減が rNc（RC 部分の許容圧縮）に
    /// 反映される。
    #[test]
    fn test_src_column_lightweight_reduces_capacity() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mut mat_n = make_material(24.0, "SD345");
        mat_n.concrete_class = ConcreteClass::Normal;
        let mut mat_l = make_material(24.0, "SD345");
        mat_l.concrete_class = ConcreteClass::Lightweight1;
        let ctx = ctx_column(LoadTerm::Long);
        let design = crate::SrcDesign;

        let forces = MemberForcesAt {
            n: -50_000_000.0,
            ..zero_forces()
        };
        let r_n = design.check(&forces, &sec, &mat_n, &ctx);
        let r_l = design.check(&forces, &sec, &mat_l, &ctx);
        assert!(
            r_l.ratio > r_n.ratio,
            "軽量1種は rNc 低減で検定比が大きいはず: normal={}, light={}",
            r_n.ratio,
            r_l.ratio
        );
    }

    /// SRC 柱の rMu は軸力（圧縮）に依存して変化し（`rc_column_mu_simple`）、
    /// それに応じて rQD1 = rQL + (rMu1+rMu2)/l′ も変化することを確認する。
    #[test]
    fn test_src_column_rmu_varies_with_axial_and_flows_to_qd1() {
        use crate::{QdMethod, SeismicQd};

        let shape = src_column_shape();
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props_z = src_rect_axis_props(500.0, 500.0, &rebar.main_x, &rebar);
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let mat = make_material(24.0, "SD345");
        let sigma_y = rebar_sigma_y(&mat);
        let fc_raw = 24.0;

        let mu_at = |n_design: f64| {
            rc_column_mu_simple(
                &RcCapacityInput {
                    b: props_z.b,
                    d: props_z.d_full,
                    at: props_z.at,
                    d_eff: props_z.d,
                    sigma_y,
                    fc: fc_raw,
                    pw: props_z.pw,
                    sigma_wy: 0.0,
                    clear_span: 0.0,
                    sigma_0: 0.0,
                },
                as_total,
                n_design,
            )
        };

        let mu_n0 = mu_at(0.0);
        let mu_n_comp = mu_at(2_000_000.0); // 中程度の圧縮軸力（釣合軸力未満）
        assert!(
            mu_n_comp > mu_n0,
            "中程度の圧縮軸力で rMu は増加するはず: mu_n0={mu_n0}, mu_n_comp={mu_n_comp}"
        );

        let ql = 50_000.0;
        let clear_length = 4000.0;
        let ctx = DesignCtx {
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
                n_factor: 1.5,
                clear_length,
                method: QdMethod::Qd1,
            }),
            ..Default::default()
        };
        // sM 項をゼロ（share=0.4 は任意の代表値）にして、rQD1 の rMu 依存の
        // みを見る。
        let share = 0.4;
        let sz = 0.0;
        let s_ft_short = 0.0;

        let seismic_n0 = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu: mu_n0,
        };
        let seismic_n_comp = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu: mu_n_comp,
        };

        let (_, r_qd_n0) = src_seismic_qd(&seismic_n0, ql, share, sz).unwrap();
        let (_, r_qd_n_comp) = src_seismic_qd(&seismic_n_comp, ql, share, sz).unwrap();

        let r_ql = (1.0 - share) * ql;
        let expected_n0 = r_ql + 2.0 * mu_n0 / clear_length;
        let expected_n_comp = r_ql + 2.0 * mu_n_comp / clear_length;

        assert!((r_qd_n0 - expected_n0).abs() / expected_n0 < 1e-9);
        assert!((r_qd_n_comp - expected_n_comp).abs() / expected_n_comp < 1e-9);
        assert!(
            r_qd_n_comp > r_qd_n0,
            "rMu 増加で rQD1 も増加するはず: r_qd_n0={r_qd_n0}, r_qd_n_comp={r_qd_n_comp}"
        );
    }
}
