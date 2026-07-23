use super::*;
use crate::rc::{concrete_allowable_shear, rebar_allowable_shear};
use crate::steel::{steel_f_value_prefix, steel_fs, steel_ft};
use crate::{LoadTerm, SeismicQd};
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

pub(crate) fn make_material(fc: f64, grade: &str) -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: Some(fc),
        fy: None,
    }
}

pub(crate) fn make_material_no_fc(grade: &str) -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn src_rect_shape(
    b: f64,
    d: f64,
    main_count: u32,
    main_dia: f64,
    main_layers: u32,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
    steel_grade: &str,
) -> SectionShape {
    SectionShape::SrcRect {
        b,
        d,
        rebar: RcRebar {
            main_x: BarSet {
                count: main_count,
                dia: main_dia,
                layers: main_layers,
            },
            main_y: BarSet {
                count: main_count,
                dia: main_dia,
                layers: main_layers,
            },
            cover,
            shear: ShearBar {
                dia: shear_dia,
                pitch: shear_pitch,
                legs: shear_legs,
                grade: None,
            },
        },
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
        steel_grade: steel_grade.to_string(),
    }
}

/// SRC 柱の標準テスト断面（500x500、8-D22 主筋、内蔵鉄骨 300x200 H形）。
pub(crate) fn src_column_shape() -> SectionShape {
    src_rect_shape(
        500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 200.0, 9.0, 14.0, "SN400B",
    )
}

pub(crate) fn make_section(shape: SectionShape) -> Section {
    shape.to_section(SectionId(0), "test".to_string())
}

pub(crate) fn zero_forces() -> MemberForcesAt {
    MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    }
}

pub(crate) fn ctx_beam(term: LoadTerm) -> DesignCtx {
    DesignCtx {
        term,
        kind: MemberKind::Beam,
        ..Default::default()
    }
}

pub(crate) fn ctx_column(term: LoadTerm) -> DesignCtx {
    DesignCtx {
        term,
        kind: MemberKind::Column,
        ..Default::default()
    }
}

// ------------------------------------------------------------------
// せん断の鉄骨/RC 弾性分担（src_shear_check）
// ------------------------------------------------------------------

#[test]
fn test_src_beam_shear_split_handcalc() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);

    let q = 200_000.0;
    let expected_s_q = sz / (sz + props.at * props.j) * q;

    let fs = concrete_allowable_shear(24.0, true);
    let w_ft = rebar_allowable_shear("SD345", true);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Long);

    let ctx = ctx_beam(LoadTerm::Long);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );
    assert!(!shear.used_qd);
    assert!((shear.s_q - expected_s_q).abs() / expected_s_q < 1e-9);
    assert!((shear.s_q + shear.r_q - q).abs() < 1e-6);
}

/// SRC の pw 上限は SRC 規準1987 準拠で長短期とも 0.6%
/// （「pw が 0.6% を超える場合は 0.6% として算定する」）。
#[test]
fn test_src_shear_pw_capped_at_0_6_percent_both_terms() {
    // 過大なせん断補強筋比（pw > 0.6%）を与え、算定に使われる pw が
    // 0.6% に頭打ちされることを確認する。
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 30.0, 4, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    for long_term in [true, false] {
        let fs = concrete_allowable_shear(24.0, long_term);
        let w_ft = rebar_allowable_shear("SD345", long_term);
        let term = if long_term {
            LoadTerm::Long
        } else {
            LoadTerm::Short
        };
        let s_fs = steel_fs(f_value, term);
        let ctx = ctx_beam(term);
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short: 0.0,
            r_mu: 0.0,
        };
        let shear = src_shear_check(
            100_000.0,
            0.0,
            100_000.0,
            0.0, // 鉄骨寄与を 0 として RC 側の pw の効果だけを見る
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            0.0,
            2.0,
            &SrcShearMode::Beam,
            &seismic,
        );
        assert!(
            (shear.pw - 0.006).abs() < 1e-12,
            "long_term={long_term}: pw={} は 0.6% に頭打ちされるはず",
            shear.pw
        );
    }
}

/// SRC 柱の短期 RC 部許容せん断力 rQAS1 は α を含まない
/// （SRC規準。rQAS1 = b・rj・(fs + 0.5・pw・wft)）。
#[test]
fn test_src_column_short_rc_allowable_has_no_alpha() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);
    let ctx = ctx_column(LoadTerm::Short);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    // m_for_alpha=0 → α は上限側（=2）に張り付く条件。α が式に入っていれば
    // rQA1 が 2 倍近く動くが、柱の短期 rQAS1 は α 非依存であること。
    let shear = src_shear_check(
        100_000.0,
        0.0,
        100_000.0,
        0.0, // 鉄骨寄与 0 で RC 側のみを見る
        props.at,
        props.j,
        props.d,
        props.b,
        390.0, // b′ を大きく取り rQA2 を支配させない
        props.pw,
        fs,
        w_ft,
        0.0,
        0.0,
        2.0,
        &SrcShearMode::Column { beta: 0.0 },
        &seismic,
    );
    let pw = props.pw.min(0.006);
    let expected_rqa1 = props.b * props.j * (fs + 0.5 * pw * w_ft);
    let expected_rqa2 = props.b * props.j * (2.0 * (390.0 / props.b) * fs + pw * w_ft);
    let expected = expected_rqa1.min(expected_rqa2);
    assert!(
        (shear.r_qa - expected).abs() / expected < 1e-9,
        "rQA={} expected={}（α を含まない短期式）",
        shear.r_qa,
        expected
    );
}

/// SRC 柱の長期は併用式 QA = (1+β)・b・rj・a′・fs を全せん断力と比較する
/// （SRC規準 P.96-97。a′ = rα（b′/b ≥ rα/3 のとき）または 3b′/b）。
#[test]
fn test_src_column_long_combined_formula() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let fs = concrete_allowable_shear(24.0, true);
    let ctx = ctx_column(LoadTerm::Long);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    let beta = 0.25;
    let q = 150_000.0;
    // m=0 → α=2（上限）。b′/b=0.5 < α/3=2/3 なので a′=3b′/b=1.5 が採用される。
    let b_prime = 0.5 * props.b;
    let shear = src_shear_check(
        q,
        0.0,
        q,
        1.0e6,
        props.at,
        props.j,
        props.d,
        props.b,
        b_prime,
        props.pw,
        fs,
        0.0,
        100.0,
        1000.0,
        2.0,
        &SrcShearMode::Column { beta },
        &seismic,
    );
    let a_prime = 1.5;
    let qa = (1.0 + beta) * props.b * props.j * a_prime * fs;
    assert!(
        (shear.r_qa - qa).abs() / qa < 1e-9,
        "QA={} expected={}",
        shear.r_qa,
        qa
    );
    assert!((shear.ratio - q / qa).abs() < 1e-9, "ratio={}", shear.ratio);
    assert!(!shear.used_qd);
}

// ------------------------------------------------------------------
// DesignCheck 振り分けの共通経路（Fc未設定・断面形状不一致）
// ------------------------------------------------------------------

#[test]
fn test_src_fc_missing_skip() {
    let shape = src_column_shape();
    let sec = make_section(shape);
    let mat = make_material_no_fc("SD345");
    let ctx = ctx_column(LoadTerm::Long);
    let design = SrcDesign;
    let result = design.check(&zero_forces(), &sec, &mat, &ctx);
    assert!(result.ok);
    assert_eq!(result.ratio, 0.0);
    assert!(result.basis.contains("Fc"));
    assert!(
        result.components.is_empty(),
        "Fc 未設定の退化ケースは components が空のはず"
    );
}

#[test]
fn test_src_shape_mismatch_skip() {
    let sec = Section {
        id: SectionId(0),
        name: "no-shape".to_string(),
        area: 1.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 500.0,
        width: 500.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_column(LoadTerm::Long);
    let design = SrcDesign;
    let result = design.check(&zero_forces(), &sec, &mat, &ctx);
    assert!(result.ok);
    assert!(result.basis.contains("断面形状不一致"));
    assert!(
        result.components.is_empty(),
        "断面形状不一致の退化ケースは components が空のはず"
    );
}

// ------------------------------------------------------------------
// 地震時短期の設計用せん断力（構造規定方式）: SRC 梁
// ------------------------------------------------------------------

/// rQD2 = max(0, n・(|Q|−sQD)) が支配するケース（rMu=0 で rQD1 を無効化し
/// QdMethod::Qd2 で明示的に検証する）。
#[test]
fn test_src_beam_seismic_qd2_handcalc() {
    use crate::QdMethod;
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);

    let ql = 50_000.0; // 長期せん断力
    let q = 200_000.0; // 当該組合せの短期せん断力
    let n_factor = 1.5;
    let clear_length = 4000.0;

    let ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
            n_factor,
            clear_length,
            method: QdMethod::Qd2,
        }),
        ..Default::default()
    };
    // r_mu=0 とすることで rQD1 を無効化し（doc 参照）、rQD2 のみを検証する。
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short,
        r_mu: 0.0,
    };

    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );

    let denom = sz + props.at * props.j;
    let share = sz / denom;
    let s_ql = share * ql;
    let sum_s_m = 2.0 * sz * s_ft_short;
    let s_qd_expected = s_ql + sum_s_m / clear_length;
    let r_qd2_expected = (n_factor * (q - s_qd_expected)).max(0.0);

    assert!(shear.used_qd);
    assert!(
        (shear.s_q - s_qd_expected).abs() / s_qd_expected < 1e-9,
        "sQD={}, expected={}",
        shear.s_q,
        s_qd_expected
    );
    assert!(
        (shear.r_q - r_qd2_expected).abs() / r_qd2_expected.max(1.0) < 1e-9,
        "rQD={}, expected(rQD2)={}",
        shear.r_q,
        r_qd2_expected
    );
}

/// rQD1 = rQL + (rMu1+rMu2)/l′ が支配するケース（QdMethod::Qd1 で
/// 明示的に検証する）。
#[test]
fn test_src_beam_seismic_qd1_handcalc() {
    use crate::QdMethod;
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);

    let ql = 50_000.0;
    let q = 200_000.0;
    let n_factor = 1.5;
    let clear_length = 4000.0;
    // rc_mu_simple で機械的に算定した rMu（部材端 1 箇所分）。
    let r_mu = rc_mu_simple(&RcCapacityInput {
        b: props.b,
        d: props.d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y: 345.0,
        fc: 24.0,
        pw: props.pw,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    });
    assert!(r_mu > 0.0, "テストの前提として rMu>0 が必要");

    let ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
            n_factor,
            clear_length,
            method: QdMethod::Qd1,
        }),
        ..Default::default()
    };
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short,
        r_mu,
    };

    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );

    let denom = sz + props.at * props.j;
    let share = sz / denom;
    let s_ql = share * ql;
    let r_ql = (ql - s_ql).max(0.0);
    let r_qd1_expected = r_ql + 2.0 * r_mu / clear_length;

    assert!(shear.used_qd);
    assert!(
        (shear.r_q - r_qd1_expected).abs() / r_qd1_expected < 1e-9,
        "rQD={}, expected(rQD1)={}",
        shear.r_q,
        r_qd1_expected
    );
}

/// ctx.seismic_qd が None のときは従来どおり弾性分担のみとなり、
/// used_qd=false（回帰確認）。
#[test]
fn test_src_beam_seismic_qd_none_falls_back_to_elastic_share() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
    );
    let rebar = match &shape {
        SectionShape::SrcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Long);
    let fs = concrete_allowable_shear(24.0, true);
    let w_ft = rebar_allowable_shear("SD345", true);

    let q = 200_000.0;
    let ctx = ctx_beam(LoadTerm::Long); // seismic_qd = None（Default）
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );

    let denom = sz + props.at * props.j;
    let expected_s_q = sz / denom * q;
    assert!(!shear.used_qd);
    assert!((shear.s_q - expected_s_q).abs() / expected_s_q < 1e-9);
    assert!((shear.s_q + shear.r_q - q).abs() < 1e-6);
}
