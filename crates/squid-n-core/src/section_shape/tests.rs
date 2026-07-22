use super::*;

#[test]
fn test_steel_h_shape() {
    let shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "H-300x300x10x15".into());
    assert!(sec.area > 0.0);
    assert!(sec.iy > sec.iz);
    assert!(sec.j > 0.0);
}

#[test]
fn test_steel_box() {
    let shape = SectionShape::SteelBox {
        height: 200.0,
        width: 200.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let sec = shape.to_section(SectionId(0), "BOX-200x200x12".into());
    assert!(sec.area > 0.0);
    assert!((sec.iy - sec.iz).abs() < 1.0);
}

#[test]
fn test_steel_pipe() {
    let shape = SectionShape::SteelPipe {
        outer_dia: 216.3,
        thick: 8.2,
    };
    let sec = shape.to_section(SectionId(0), "PIPE-216.3x8.2".into());
    assert!(sec.area > 0.0);
    assert!((sec.iy - sec.iz).abs() < 1e-6);
    assert!(sec.j > sec.iy);
}

#[test]
fn test_steel_flat_bar() {
    // 中実矩形 B=100 × t=12。A=1200、iy=b·d³/12=100·12³/12、iz=d·b³/12=12·100³/12。
    let shape = SectionShape::SteelFlatBar {
        width: 100.0,
        thick: 12.0,
    };
    let sec = shape.to_section(SectionId(0), "FB-100x12".into());
    assert!((sec.area - 1200.0).abs() < 1e-6);
    assert!((sec.iy - 100.0 * 12.0_f64.powi(3) / 12.0).abs() < 1e-6);
    assert!((sec.iz - 12.0 * 100.0_f64.powi(3) / 12.0).abs() < 1e-6);
    assert!(
        sec.iz > sec.iy,
        "幅 > せい なので弱軸せい方向より iz が大きい"
    );
    assert!(sec.j > 0.0 && sec.as_y > 0.0 && sec.as_z > 0.0);
    assert_eq!(sec.depth, 12.0);
    assert_eq!(sec.width, 100.0);
}

#[test]
fn test_steel_round_bar() {
    // 中実円 D=30。A=πD²/4、iy=iz=πD⁴/64、J=πD⁴/32=2·iy。
    let shape = SectionShape::SteelRoundBar { dia: 30.0 };
    let sec = shape.to_section(SectionId(0), "RB-30".into());
    let pi = std::f64::consts::PI;
    assert!((sec.area - pi * 30.0 * 30.0 / 4.0).abs() < 1e-6);
    assert!((sec.iy - pi * 30.0_f64.powi(4) / 64.0).abs() < 1e-6);
    assert!((sec.iy - sec.iz).abs() < 1e-9, "軸対称");
    assert!((sec.j - 2.0 * sec.iy).abs() < 1e-6);
    assert_eq!(sec.depth, 30.0);
    assert_eq!(sec.width, 30.0);
}

#[test]
fn test_steel_lip_channel() {
    // リップ溝形 H=150, B=75, C=20, t=2.3。矩形分解で独立計算した値と一致するか。
    let h = 150.0;
    let b = 75.0;
    let c = 20.0;
    let t = 2.3;
    let shape = SectionShape::SteelLipChannel {
        height: h,
        width: b,
        lip: c,
        thick: t,
    };
    let sec = shape.to_section(SectionId(0), "LipC-150x75x20x2.3".into());
    // 断面積 = ウェブ + 2 フランジ + 2 リップ（重なり無し分解）。
    let area_expect = t * h + 2.0 * (b - t) * t + 2.0 * t * (c - t);
    assert!(
        (sec.area - area_expect).abs() < 1e-6,
        "A={} 期待 {}",
        sec.area,
        area_expect
    );
    // 強軸 iy（上下対称、y=H/2 まわり）を独立計算。
    let i_web = t * h.powi(3) / 12.0;
    let a_f = (b - t) * t;
    let i_f = (b - t) * t.powi(3) / 12.0 + a_f * ((h - t) / 2.0).powi(2);
    let a_l = t * (c - t);
    let i_l = t * (c - t).powi(3) / 12.0 + a_l * ((h - c - t) / 2.0).powi(2);
    let iy_expect = i_web + 2.0 * i_f + 2.0 * i_l;
    assert!(
        (sec.iy - iy_expect).abs() < 1e-3,
        "iy={} 期待 {}",
        sec.iy,
        iy_expect
    );
    // 溝形なので強軸 > 弱軸、いずれも正、J・せん断有効断面も正。
    assert!(sec.iy > sec.iz && sec.iz > 0.0);
    assert!(sec.j > 0.0 && sec.as_y > 0.0 && sec.as_z > 0.0);
    assert_eq!(sec.depth, h);
    assert_eq!(sec.width, b);
}

#[test]
fn test_steel_built_h_equal_flanges_matches_rolled_h() {
    // 上下フランジが同一なら通常の SteelH と断面性能が一致する（分解の妥当性確認）。
    let built = SectionShape::SteelBuiltH {
        height: 400.0,
        upper_width: 200.0,
        upper_thick: 12.0,
        lower_width: 200.0,
        lower_thick: 12.0,
        web_thick: 8.0,
    };
    let rolled = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 12.0,
    };
    let sb = built.to_section(SectionId(0), "BH".into());
    let sr = rolled.to_section(SectionId(0), "H".into());
    assert!((sb.area - sr.area).abs() < 1e-6, "A 一致");
    assert!(
        (sb.iy - sr.iy).abs() < 1e-3,
        "iy 一致 {} vs {}",
        sb.iy,
        sr.iy
    );
    assert!((sb.iz - sr.iz).abs() < 1e-3, "iz 一致");
    assert!((sb.j - sr.j).abs() < 1e-3, "J 一致");
}

#[test]
fn test_steel_built_h_asymmetric() {
    // 下フランジが大きい非対称 H。図心は下寄り、強軸 iy は独立計算と一致。
    let h = 500.0;
    let uw = 150.0;
    let ut = 9.0;
    let lw = 300.0;
    let lt = 19.0;
    let tw = 9.0;
    let shape = SectionShape::SteelBuiltH {
        height: h,
        upper_width: uw,
        upper_thick: ut,
        lower_width: lw,
        lower_thick: lt,
        web_thick: tw,
    };
    let sec = shape.to_section(SectionId(0), "BH".into());
    // 面積 = 上フランジ + 下フランジ + ウェブ。
    let hw = h - ut - lt;
    let a_expect = uw * ut + lw * lt + tw * hw;
    assert!((sec.area - a_expect).abs() < 1e-6);
    // 図心 y（下端起点）を独立計算し、iy を平行軸で検算。
    let a_uf = uw * ut;
    let a_lf = lw * lt;
    let a_w = tw * hw;
    let y_uf = h - ut / 2.0;
    let y_lf = lt / 2.0;
    let y_w = lt + hw / 2.0;
    let y_bar = (a_uf * y_uf + a_lf * y_lf + a_w * y_w) / a_expect;
    let iy_expect = uw * ut.powi(3) / 12.0
        + a_uf * (y_uf - y_bar).powi(2)
        + lw * lt.powi(3) / 12.0
        + a_lf * (y_lf - y_bar).powi(2)
        + tw * hw.powi(3) / 12.0
        + a_w * (y_w - y_bar).powi(2);
    assert!(
        (sec.iy - iy_expect).abs() < 1e-3,
        "iy={} 期待 {}",
        sec.iy,
        iy_expect
    );
    assert!(sec.iy > sec.iz && sec.iz > 0.0);
    assert!(sec.j > 0.0 && sec.as_y > 0.0 && sec.as_z > 0.0);
    assert_eq!(sec.depth, h);
    assert_eq!(sec.width, lw, "せい幅は広い方（下フランジ）");
}

#[test]
fn test_rc_rect() {
    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 16.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 4,
                dia: 16.0,
                layers: 2,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    let sec = shape.to_section(SectionId(0), "RC-500x500".into());
    assert!(sec.area > 0.0);
    assert!(sec.as_y > 0.0);
    assert!(sec.iz > 0.0);
}

#[test]
fn test_rc_circle() {
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
                dia: 6.0,
                pitch: 80.0,
                legs: 1,
                grade: None,
            },
        },
    };
    let sec = shape.to_section(SectionId(0), "RC-600".into());
    assert!(sec.area > 0.0);
    assert!(sec.as_y > 0.0);
    assert!(sec.as_z > 0.0);
}

#[test]
fn test_steel_l_angle() {
    let shape = SectionShape::SteelAngle {
        leg_a: 150.0,
        leg_b: 100.0,
        thick: 12.0,
    };
    let sec = shape.to_section(SectionId(0), "L-150x100x12".into());
    assert!(sec.area > 0.0);
    assert!(sec.iy > 0.0);
    assert!(sec.iz > 0.0);
}

#[test]
fn test_steel_tee() {
    let shape = SectionShape::SteelTee {
        height: 200.0,
        width: 200.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "T-200x200x10x15".into());
    assert!(sec.area > 0.0);
    assert!(sec.iy > 0.0);
    assert!(sec.iz > 0.0);
}

#[test]
fn test_steel_channel() {
    let shape = SectionShape::SteelChannel {
        height: 250.0,
        width: 90.0,
        web_thick: 7.5,
        flange_thick: 12.0,
    };
    let sec = shape.to_section(SectionId(0), "C-250x90x7.5x12".into());
    assert!(sec.area > 0.0);
    assert!(sec.iy > sec.iz);
}

#[test]
fn test_section_roundtrip_serde() {
    let shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let json = serde_json::to_string(&shape).unwrap();
    let restored: SectionShape = serde_json::from_str(&json).unwrap();
    assert_eq!(shape, restored);
}

#[test]
fn test_rc_rebar_serde_roundtrip() {
    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 2,
                dia: 16.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    let json = serde_json::to_string(&shape).unwrap();
    let restored: SectionShape = serde_json::from_str(&json).unwrap();
    assert_eq!(shape, restored);
}

#[test]
fn test_rc_rect_area() {
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 19.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 2,
                dia: 13.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    assert!((shape.calc_area() - 240_000.0).abs() < 1e-9);
    let iy = shape.calc_iy();
    let iz = shape.calc_iz();
    assert!((iy - 400.0 * 600.0_f64.powi(3) / 12.0).abs() < 1e-6);
    assert!((iz - 600.0 * 400.0_f64.powi(3) / 12.0).abs() < 1e-6);
}

#[test]
fn test_rc_rect_shear_area_is_gross_over_kappa() {
    // 材料力学: RC の As = B·D/κ（κ=1.2）。鉄筋断面積ではない。
    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    let sec = shape.to_section(SectionId(0), "RC-500x500".into());
    let expected = 500.0 * 500.0 / 1.2;
    assert!((sec.as_y - expected).abs() < 1e-6);
    assert!((sec.as_z - expected).abs() < 1e-6);
}

#[test]
fn test_steel_h_shear_area_is_web_and_flange() {
    // 材料力学: S の As = Aw/κ（κ=1.0）。強軸側(as_z)=ウェブ、弱軸側(as_y)=フランジ。
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 9.0,
        flange_thick: 12.0,
    };
    let sec = shape.to_section(SectionId(0), "H-400x200x9x12".into());
    assert!((sec.as_z - 400.0 * 9.0).abs() < 1e-9);
    assert!((sec.as_y - 2.0 * 200.0 * 12.0).abs() < 1e-9);
}

#[test]
fn test_rc_rect_torsion_matches_manual_formula() {
    // J = (b³h/16)[16/3 − 3.36(b/h)(1 − (1/12)(b/h)⁴)]。細長比によらず同一式。
    let rebar = RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    // 正方形 500×500
    let sq = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: rebar.clone(),
    };
    let b: f64 = 500.0;
    let expected = b.powi(3) * b / 16.0 * (16.0 / 3.0 - 3.36 * (1.0 - 1.0 / 12.0));
    assert!((sq.calc_j() - expected).abs() / expected < 1e-12);
    // 細長断面 100×2000（旧実装は r≥10 で β=1/3 に切替え約+6.7%乖離していた）
    let slender = SectionShape::RcRect {
        b: 100.0,
        d: 2000.0,
        rebar,
    };
    let (bs, h) = (100.0_f64, 2000.0_f64);
    let c = bs / h;
    let expected2 = bs.powi(3) * h / 16.0 * (16.0 / 3.0 - 3.36 * c * (1.0 - c.powi(4) / 12.0));
    assert!((slender.calc_j() - expected2).abs() / expected2 < 1e-12);
}

#[test]
fn test_src_axial_stiffness_area_accumulates_steel() {
    // An = rcAn + sAn·(ns−1)。calc_area（質量用）はコンクリート全断面のまま。
    let shape = SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 12.0,
        steel_grade: "SN400B".into(),
    };
    let s_a = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 9.0;
    assert!((shape.calc_area() - 360_000.0).abs() < 1e-9);
    let expected = 360_000.0 + (N_S_EQ - 1.0) * s_a;
    assert!((shape.calc_axial_stiffness_area() - expected).abs() < 1e-9);
    // せん断断面積も RC 部 A/1.2 + 鉄骨等価分が累加される
    let sec = shape.to_section(SectionId(0), "SRC-600".into());
    let rc_as = 360_000.0 / 1.2;
    assert!((sec.as_z - (rc_as + (N_S_EQ - 1.0) * 400.0 * 9.0)).abs() < 1e-9);
    assert!((sec.as_y - (rc_as + (N_S_EQ - 1.0) * 2.0 * 200.0 * 12.0)).abs() < 1e-9);
}

fn make_src_600() -> SectionShape {
    SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 12.0,
        steel_grade: "SN400B".into(),
    }
}

#[test]
fn test_wall_shear_shape_factor_rectangle_limit() {
    // ξ=1(側柱なし=矩形)は η によらず κ=1.2
    for eta in [0.1, 0.5, 1.0] {
        let k = wall_shear_shape_factor(1.0, eta);
        assert!((k - KAPPA_RC).abs() < 1e-12, "eta={eta} k={k}");
    }
    // 側柱付き(ξ<1)は有限・正の値
    let k = wall_shear_shape_factor(0.8, 0.3);
    assert!(k.is_finite() && k > 0.0);
    // 退化入力でも非有限値・負値は返さない
    let k0 = wall_shear_shape_factor(0.0, 0.0);
    assert!(k0.is_finite() && k0 > 0.0);
}

#[test]
fn test_concrete_young_modulus_formula() {
    // Ec = 3.35e4·(γ/24)²·(Fc/60)^(1/3)、γ=23。Fc=60 で (Fc/60)^(1/3)=1。
    let expected = 3.35e4 * (23.0_f64 / 24.0).powi(2);
    assert!((concrete_young_modulus(60.0) - expected).abs() < 1e-9);
    assert_eq!(concrete_young_modulus(0.0), 0.0);
}

#[test]
fn test_src_equivalent_props_uses_material_ns() {
    // ns = Es/Ec を材料から算定（N_S_EQ=15 固定ではない）。
    let shape = make_src_600();
    let ec = 23000.0;
    let p = shape.src_equivalent_props(ec, 0.2).unwrap();
    let ns = E_STEEL / ec;
    let s_a = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 9.0;
    assert!((p.area_ax - (360_000.0 + (ns - 1.0) * s_a)).abs() < 1e-6);
    // Iy = Ic + (ns−1)·sIy
    let s_iy = (200.0 * 400.0_f64.powi(3) - (200.0 - 9.0) * 376.0_f64.powi(3)) / 12.0;
    let expected_iy = 600.0 * 600.0_f64.powi(3) / 12.0 + (ns - 1.0) * s_iy;
    assert!((p.iy - expected_iy).abs() / expected_iy < 1e-12);
    // ngs = ns·(1+νc)/(1+νs)
    let ngs = ns * 1.2 / 1.3;
    let rc_as = 360_000.0 / 1.2;
    assert!((p.as_z - (rc_as + (ngs - 1.0) * 400.0 * 9.0)).abs() < 1e-6);
    // Ec≤0 は None（既定 N_S_EQ へのフォールバック用）
    assert!(shape.src_equivalent_props(0.0, 0.2).is_none());
}

#[test]
fn test_cft_equivalent_props_adds_concrete() {
    // 鋼管基準の 1/n 換算で充填コンクリートを累加（A・I・J とも鋼管のみより増える）。
    let shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };
    let es = 205000.0;
    let fc = 36.0;
    let p = shape.cft_equivalent_props(es, 0.3, fc).unwrap();
    let n = es / concrete_young_modulus(fc);
    let (bi, hi) = (400.0 - 24.0, 400.0 - 24.0);
    assert!((p.area_ax - (shape.calc_area() + bi * hi / n)).abs() < 1e-6);
    let expected_iy = shape.calc_iy() + bi * hi.powi(3) / 12.0 / n;
    assert!((p.iy - expected_iy).abs() / expected_iy < 1e-12);
    assert!(p.j > shape.calc_j());
    assert!(p.as_z > 2.0 * 12.0 * hi);
    // 円形も同様
    let pipe = SectionShape::CftPipe {
        outer_dia: 400.0,
        thick: 12.0,
    };
    let pp = pipe.cft_equivalent_props(es, 0.3, fc).unwrap();
    assert!(pp.area_ax > pipe.calc_area());
    assert!(pp.iy > pipe.calc_iy());
    // 非CFT形状は None
    assert!(make_src_600().cft_equivalent_props(es, 0.3, fc).is_none());
}

#[test]
fn test_rc_circle_area() {
    let shape = SectionShape::RcCircle {
        d: 800.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 16,
                dia: 25.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 16,
                dia: 25.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 1,
                grade: None,
            },
        },
    };
    let expected_area = std::f64::consts::PI * 800.0_f64.powi(2) / 4.0;
    assert!((shape.calc_area() - expected_area).abs() < 1e-6);
    let iy = shape.calc_iy();
    assert!((iy - std::f64::consts::PI * 800.0_f64.powi(4) / 64.0).abs() < 1e-6);
    assert!((shape.calc_iy() - shape.calc_iz()).abs() < 1e-6);
}
