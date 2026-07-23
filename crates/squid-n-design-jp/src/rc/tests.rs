use super::*;
use squid_n_core::ids::{MaterialId, SectionId};
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

#[allow(clippy::too_many_arguments)]
pub(crate) fn rc_rect_shape(
    b: f64,
    d: f64,
    main_count: u32,
    main_dia: f64,
    main_layers: u32,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
) -> SectionShape {
    SectionShape::RcRect {
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
    }
}

/// `rc_rect_shape` のせん断補強筋に高強度品の `grade` を付与した版。
#[allow(clippy::too_many_arguments)]
pub(crate) fn rc_rect_shape_with_shear_grade(
    b: f64,
    d: f64,
    main_count: u32,
    main_dia: f64,
    main_layers: u32,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
    shear_grade: &str,
) -> SectionShape {
    match rc_rect_shape(
        b,
        d,
        main_count,
        main_dia,
        main_layers,
        cover,
        shear_dia,
        shear_pitch,
        shear_legs,
    ) {
        SectionShape::RcRect { b, d, mut rebar } => {
            rebar.shear.grade = Some(shear_grade.to_string());
            SectionShape::RcRect { b, d, rebar }
        }
        _ => unreachable!(),
    }
}

pub(crate) fn make_section(shape: SectionShape) -> Section {
    shape.to_section(SectionId(0), "test".to_string())
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
// 地震時短期の設計用せん断力 QD = min(QD1, QD2)
// ------------------------------------------------------------------

#[test]
fn test_seismic_design_shear_min_of_qd1_qd2() {
    use crate::{QdMethod, SeismicQd};
    let mut ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, 50_000.0, 0.0, 0.0, 0.0, 0.0])],
            n_factor: 1.5,
            clear_length: 4000.0,
            method: QdMethod::Min,
        }),
        ..Default::default()
    };
    // 当該組合せ Q=150kN、QL=50kN → QE=100kN、QD2 = 50+1.5×100 = 200kN。
    // ΣMy=400kN·m → 梁 QD1 = 50+400e6/4000 = 150kN → min = 150kN。
    let q_beam = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q_beam - 150_000.0).abs() < 1e-6, "q_beam={q_beam}");
    // 柱 QD1 = ΣcMy/h′ = 100kN（QL を加算しない）→ min(100, 200) = 100kN。
    let q_col = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, true);
    assert!((q_col - 100_000.0).abs() < 1e-6, "q_col={q_col}");
    // QD2 単独選択。
    ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Qd2;
    let q2 = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q2 - 200_000.0).abs() < 1e-6, "q2={q2}");
    // ΣMy<=0（終局曲げ不明）のとき QD1 は無効で QD2 のみ。
    ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Min;
    let q_no_mu = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 0.0, false);
    assert!((q_no_mu - 200_000.0).abs() < 1e-6);
    // 評価位置が長期内力に無い場合・文脈なしの場合は解析値のまま。
    let q_missing = seismic_design_shear(&ctx, 0.5, 150_000.0, 1, 400.0e6, false);
    assert!((q_missing - 150_000.0).abs() < 1e-6);
    ctx.seismic_qd = None;
    let q_none = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q_none - 150_000.0).abs() < 1e-6);
}

// ------------------------------------------------------------------
// 許容応力度（RC 造検定でのみ使う独自カバレッジ分。他は material_strength.rs 側で検証）
// ------------------------------------------------------------------

#[test]
fn test_concrete_young_modulus_plausible() {
    // Fc=21, γ=23 で AIJ 表の目安値（約 2.0〜2.3 × 10^4 N/mm²）に近い。
    let ec = concrete_young_modulus(21.0, Some(23.0));
    assert!(ec > 20000.0 && ec < 23000.0, "Ec={ec}");
}

#[test]
fn test_rebar_allowable_tension_table() {
    assert!((rebar_allowable_tension("SR235", 16.0, true) - 155.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR235", 16.0, false) - 235.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR295", 16.0, true) - 155.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR295", 16.0, false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD295A", 16.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 22.0, true) - 215.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 32.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 22.0, false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD490", 22.0, false) - 490.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("UNKNOWN", 22.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("UNKNOWN", 22.0, false) - 295.0).abs() < 1e-9);
}

#[test]
fn test_rebar_allowable_shear_table() {
    assert!((rebar_allowable_shear("SR235", true) - 155.0).abs() < 1e-9);
    // 短期は基準強度 F=235（令90条表）。フォールバック 295 に落ちて F 値を
    // 超過していた回帰の防止。
    assert!((rebar_allowable_shear("SR235", false) - 235.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SR295", false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD345", true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD295A", false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD345", false) - 345.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD390", false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD490", false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("UNKNOWN", false) - 295.0).abs() < 1e-9);
}

// ------------------------------------------------------------------
// dt（引張筋重心）
// ------------------------------------------------------------------

#[test]
fn test_tension_dt_single_layer() {
    let bar = BarSet {
        count: 4,
        dia: 22.0,
        layers: 1,
    };
    let dt = tension_dt(40.0, 10.0, &bar);
    assert!((dt - (40.0 + 10.0 + 11.0)).abs() < 1e-9);
}

#[test]
fn test_tension_dt_two_layers() {
    let bar = BarSet {
        count: 8,
        dia: 22.0,
        layers: 2,
    };
    let cover = 40.0;
    let shear_dia = 10.0;
    let k1 = cover + shear_dia + bar.dia / 2.0;
    let k_prime = 25.0_f64.max(1.5 * bar.dia);
    let k2 = k1 + bar.dia / 2.0 + k_prime + bar.dia / 2.0;
    let expected = (k1 + k2) / 2.0;
    let dt = tension_dt(cover, shear_dia, &bar);
    assert!((dt - expected).abs() < 1e-6);
}

// ------------------------------------------------------------------
// せん断スパン比 α・せん断耐力（普通強度・高強度せん断補強筋とも）
// ------------------------------------------------------------------

#[test]
fn test_shear_alpha_clamp_at_upper_bound() {
    // M/(Q・d) = 1 -> α = 4/2 = 2.0（上限に一致）
    let d = 500.0;
    let q = 100_000.0;
    let m = q * d * 1.0;
    let alpha = shear_alpha(m, q, d, 2.0);
    assert!((alpha - 2.0).abs() < 1e-9);
}

#[test]
fn test_shear_alpha_clamp_at_lower_bound() {
    // M/(Q・d) = 3 -> α = 4/4 = 1.0（下限に一致）
    let d = 500.0;
    let q = 100_000.0;
    let m = q * d * 3.0;
    let alpha = shear_alpha(m, q, d, 2.0);
    assert!((alpha - 1.0).abs() < 1e-9);
}

#[test]
fn test_shear_alpha_clamp_engages_beyond_bounds() {
    let d = 500.0;
    let q = 100_000.0;
    // M/(Q・d)=0 -> 素の α=4.0 は上限 2.0 にクランプされる。
    let alpha_hi = shear_alpha(0.0, q, d, 2.0);
    assert!((alpha_hi - 2.0).abs() < 1e-9);
    // M/(Q・d)=10 -> 素の α=4/11≈0.364 は下限 1.0 にクランプされる。
    let m = q * d * 10.0;
    let alpha_lo = shear_alpha(m, q, d, 2.0);
    assert!((alpha_lo - 1.0).abs() < 1e-9);
}

#[test]
fn test_shear_alpha_intermediate_value() {
    // M/(Q・d) = 1 と 3 の中間、M/(Q・d)=2 -> α = 4/3 ≈ 1.333
    let d = 500.0;
    let q = 100_000.0;
    let m = q * d * 2.0;
    let alpha = shear_alpha(m, q, d, 2.0);
    assert!((alpha - 4.0 / 3.0).abs() < 1e-9);
}

#[test]
fn test_pw_ratio_capped_long_term() {
    // 過大なせん断補強筋比を作り、長期は 0.6% に制限されることを確認する。
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 30.0, 4);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    let alpha = 1.5;
    let qa_capped = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, false);

    // 手計算: pw を 0.6% に制限した式と一致すること。
    let pw_term = 0.5 * allow.w_ft * (0.006 - 0.002);
    let expected = props.b * props.j * (alpha * allow.fs + pw_term);
    assert!((qa_capped - expected).abs() / expected < 1e-6);
}

#[test]
fn test_beam_shear_damage_control_vs_safety() {
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    let alpha = 1.4;

    let qa_damage = shear_capacity(&props, &allow, alpha, LoadTerm::Short, true, false);
    let qa_safety = shear_capacity(&props, &allow, alpha, LoadTerm::Short, false, false);

    let pw_term = if props.pw < 0.002 {
        0.0
    } else {
        0.5 * allow.w_ft * (props.pw.min(0.012) - 0.002)
    };
    let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term);
    let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term);

    assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
    assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
    assert!(
        qa_damage < qa_safety,
        "損傷制御式は安全確保式より小さいはず"
    );
}

#[test]
fn test_column_shear_alpha_upper_bound_1_5() {
    let d = 400.0;
    let q = 50_000.0;
    // M/(Q・d)=0 -> 素の α=4.0 は柱の上限 1.5 にクランプされる。
    let alpha = shear_alpha(0.0, q, d, 1.5);
    assert!((alpha - 1.5).abs() < 1e-9);
}

#[test]
fn test_column_safety_check_excludes_alpha() {
    let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props_strong(&make_section(shape), &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);

    let qa_alpha_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, false, true);
    let qa_alpha_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, false, true);
    // 柱の「安全確保のための検討」式は α を含まないため、α を変えても
    // QA は変化しない。
    assert!((qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6);

    // 損傷制御式は α に依存するため異なる値になる。
    let qa_damage_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, true, true);
    let qa_damage_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, true, true);
    assert!((qa_damage_1 - qa_damage_1_5).abs() > 1e-6);
}

#[test]
fn test_column_long_term_shear_has_no_rebar_term() {
    let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 60.0, 4);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props_strong(&make_section(shape), &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    let alpha = 1.3;
    let qal = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, true);
    let expected = props.b * props.j * alpha * allow.fs;
    assert!((qal - expected).abs() / expected < 1e-9);
}

#[test]
fn test_high_strength_shear_capacity_offset_0_001_beam() {
    // 高強度せん断補強筋の暫定対応式（短期）は pw オフセットが 0.001。
    let shape =
        rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "KH785");
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    assert!(props.pw > 0.001, "テストの前提として pw > 0.1% が必要");

    let mut allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    allow.w_ft = high_strength_w_ft("KH785", false);
    let alpha = 1.4;

    let qa_damage = shear_capacity_high_strength(
        &props,
        &allow,
        alpha,
        LoadTerm::Short,
        true,
        false,
        "KH785",
        24.0,
    );
    let qa_safety = shear_capacity_high_strength(
        &props,
        &allow,
        alpha,
        LoadTerm::Short,
        false,
        false,
        "KH785",
        24.0,
    );

    let pw_cap_damage = high_strength_pw_cap("KH785", LoadTerm::Short, true, 24.0);
    let pw_cap_safety = high_strength_pw_cap("KH785", LoadTerm::Short, false, 24.0);
    let pw_term_damage = 0.5 * allow.w_ft * (props.pw.min(pw_cap_damage) - 0.001);
    let pw_term_safety = 0.5 * allow.w_ft * (props.pw.min(pw_cap_safety) - 0.001);
    let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term_damage);
    let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term_safety);

    assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
    assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
}

#[test]
fn test_high_strength_offset_differs_from_normal_short_term() {
    // pw を 0.001 < pw < 0.002 の範囲に設定する。普通強度式（offset=0.002）
    // では pw 項が 0 のままだが、高強度式（短期 offset=0.001）では
    // pw 項が有効になり QA が普通強度より大きくなることを確認する。
    let shape =
        rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 600.0, 2, "KH785");
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    assert!(
        props.pw > 0.001 && props.pw < 0.002,
        "テストの前提として 0.001 < pw < 0.002 が必要: pw={}",
        props.pw
    );

    let allow_normal = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    let mut allow_hs = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    allow_hs.w_ft = high_strength_w_ft("KH785", false);
    let alpha = 1.3;

    let qa_normal = shear_capacity(&props, &allow_normal, alpha, LoadTerm::Short, true, false);
    let qa_hs = shear_capacity_high_strength(
        &props,
        &allow_hs,
        alpha,
        LoadTerm::Short,
        true,
        false,
        "KH785",
        24.0,
    );

    assert!(
        qa_hs > qa_normal,
        "高強度式は pw 項が有効になり普通強度式より大きいはず: normal={qa_normal}, hs={qa_hs}"
    );
}

#[test]
fn test_high_strength_shear_capacity_long_term_matches_normal_formula() {
    // 長期は普通強度と同じ式（offset=0.002, pw 上限 0.6%）で、
    // w_ft も高強度テーブル値=195 と SD345 長期値=195 が一致するため、
    // 高強度パスと普通強度パスの結果は一致するはず。
    let shape =
        rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "UB785");
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);

    let mut allow_hs = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    allow_hs.w_ft = high_strength_w_ft("UB785", true);
    let allow_normal = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    let alpha = 1.3;

    let qa_hs = shear_capacity_high_strength(
        &props,
        &allow_hs,
        alpha,
        LoadTerm::Long,
        true,
        false,
        "UB785",
        24.0,
    );
    let qa_normal = shear_capacity(&props, &allow_normal, alpha, LoadTerm::Long, true, false);

    assert!((qa_hs - qa_normal).abs() / qa_normal < 1e-9);
}

#[test]
fn test_high_strength_column_safety_check_excludes_alpha() {
    let shape =
        rc_rect_shape_with_shear_grade(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, "SHD685");
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props_strong(&make_section(shape.clone()), &rebar);
    let mut allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    allow.w_ft = high_strength_w_ft("SHD685", false);

    let qa_alpha_1 = shear_capacity_high_strength(
        &props,
        &allow,
        1.0,
        LoadTerm::Short,
        false,
        true,
        "SHD685",
        24.0,
    );
    let qa_alpha_1_5 = shear_capacity_high_strength(
        &props,
        &allow,
        1.5,
        LoadTerm::Short,
        false,
        true,
        "SHD685",
        24.0,
    );
    // 柱の安全確保のための検討式は高強度でも α を含まない。
    assert!((qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6);
}

#[test]
fn test_shear_capacity_for_none_delegates_to_normal_regression() {
    // grade=None のディスパッチが普通強度の既存関数と完全に一致すること
    // （既存挙動の回帰確認）。
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);

    let via_dispatch = shear_capacity_for(
        &props,
        &allow,
        1.3,
        LoadTerm::Short,
        true,
        false,
        None,
        24.0,
    );
    let via_direct = shear_capacity(&props, &allow, 1.3, LoadTerm::Short, true, false);
    assert!((via_dispatch - via_direct).abs() < 1e-12);
}

// ------------------------------------------------------------------
// 軽量コンクリート（許容応力度 0.9 倍・高強度フープとの併用）
// ------------------------------------------------------------------

#[test]
fn test_effective_damage_control_lightweight_high_strength() {
    // 高強度フープ + 軽量 → 強制 false。
    assert!(!effective_damage_control(
        true,
        Some("KH785"),
        ConcreteClass::Lightweight1
    ));
    assert!(!effective_damage_control(
        true,
        Some("UB785"),
        ConcreteClass::Lightweight2
    ));
    // 高強度フープ + 普通 → 指定どおり。
    assert!(effective_damage_control(
        true,
        Some("KH785"),
        ConcreteClass::Normal
    ));
    // 普通強度フープ（grade=None）は軽量でも指定どおり。
    assert!(effective_damage_control(
        true,
        None,
        ConcreteClass::Lightweight1
    ));
}

// ------------------------------------------------------------------
// フォールバック・RcDesign 統合（振り分け全体の確認）
// ------------------------------------------------------------------

#[test]
fn test_fc_missing_fallback() {
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let sec = make_section(shape);
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SD345".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let ctx = ctx_beam(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let design = RcDesign;
    let result = design.check(&forces, &sec, &mat, &ctx);
    assert!(result.ok);
    assert_eq!(result.ratio, 0.0);
    assert!(result.basis.contains("Fc"));
    assert!(
        result.components.is_empty(),
        "Fc 未設定の退化ケースは components が空のはず"
    );
}

#[test]
fn test_shape_missing_fallback() {
    // shape を持たない Section（数値直入力等）。
    let sec = Section {
        id: SectionId(0),
        name: "no-shape".to_string(),
        area: 300.0 * 600.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 600.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_beam(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let design = RcDesign;
    let result = design.check(&forces, &sec, &mat, &ctx);
    assert!(result.ok);
    assert_eq!(result.ratio, 0.0);
    assert!(result.basis.contains("配筋情報なし"));
    assert!(
        result.components.is_empty(),
        "配筋情報なしの退化ケースは components が空のはず"
    );
}

#[test]
fn test_rc_circle_beam_and_column_smoke() {
    let shape = SectionShape::RcCircle {
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 12,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 12,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 1,
                grade: None,
            },
        },
    };
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let design = RcDesign;

    let forces = MemberForcesAt {
        pos: 0.0,
        n: -200_000.0,
        qy: 30_000.0,
        qz: 20_000.0,
        my: 10_000_000.0,
        mz: 20_000_000.0,
    };

    let ctx_col = ctx_column(LoadTerm::Short);
    let r_col = design.check(&forces, &sec, &mat, &ctx_col);
    assert!(r_col.ratio.is_finite() && r_col.ratio >= 0.0);
    assert!(r_col.basis.contains("円形柱"));

    let ctx_b = ctx_beam(LoadTerm::Short);
    let r_beam = design.check(&forces, &sec, &mat, &ctx_b);
    assert!(r_beam.ratio.is_finite() && r_beam.ratio >= 0.0);
}
