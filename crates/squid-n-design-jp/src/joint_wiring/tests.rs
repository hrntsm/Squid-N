use super::*;
use smallvec::SmallVec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, EndCondition, ForceRegime, LocalAxis, Material, MultiOpeningMode, Node, RigidZone,
    Section, WallAttr, WallOpening,
};
use squid_n_core::section_shape::SectionShape;

/// 矩形壁（4000×3000, t=180）1 枚のみのモデル。側柱なし。
/// `wall_attr` を指定すると `model.wall_attrs` に登録する。
fn wall_model(wall_attr: Option<WallAttr>) -> Model {
    wall_model_sized(4000.0, 3000.0, 180.0, wall_attr)
}

/// 矩形壁（`l`×`h`, 厚さ `thickness`）1 枚のみのモデル。側柱なし。
/// `wall_model` の寸法可変版（近接開口・包絡開口のテストで、開口周比 r0
/// を任意の壁面積に対して調整するために用いる）。
fn wall_model_sized(l: f64, h: f64, thickness: f64, wall_attr: Option<WallAttr>) -> Model {
    let mut nodes: Vec<Node> = Vec::new();
    let coords = [[0.0, 0.0, 0.0], [l, 0.0, 0.0], [l, 0.0, h], [0.0, 0.0, h]];
    for (i, c) in coords.iter().enumerate() {
        nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
        });
    }
    let sections = vec![Section {
        id: SectionId(0),
        name: "wall".to_string(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: Some(thickness),
        shape: Some(SectionShape::RcWall {
            thickness,
            ps: 0.006,
        }),
    }];
    let materials = vec![Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SD345".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    }];
    let elements = vec![ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(0));
            v.push(NodeId(1));
            v.push(NodeId(2));
            v.push(NodeId(3));
            v
        },
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    }];
    Model {
        nodes,
        elements,
        sections,
        materials,
        wall_attrs: wall_attr.into_iter().collect(),
        ..Default::default()
    }
}

/// 壁要素 ElemId(0) の耐震壁(RC)検定結果（無ければ None）。
fn wall_check_result(model: &Model, forces: ForcesAt<'_>) -> Option<CheckResult> {
    let member_forces = vec![(ElemId(0), forces)];
    collect_joint_checks(model, &member_forces, LoadTerm::Short)
        .into_iter()
        .find(|(_, label, _)| label == "耐震壁(RC)")
        .map(|(_, _, cr)| cr)
}

/// 開口あり（`wall_attrs` に `opening_area>0` を登録）の壁は、無開口より
/// 検定比が大きくなる（開口低減係数 r<1 で Qa が下がるため）。
#[test]
fn wall_with_opening_has_larger_ratio_than_without() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];

    let model_no_attr = wall_model(None);
    let res_no_opening =
        wall_check_result(&model_no_attr, &forces).expect("無開口の壁は検定されるはず");

    // opening_area = 0.1・l・h → r0 ≈ 0.316（<0.4 で耐震壁として扱われる）。
    let model_with_opening = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.1 * 4000.0 * 3000.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    }));
    let res_opening =
        wall_check_result(&model_with_opening, &forces).expect("小開口は耐震壁のまま検定される");

    assert!(
        res_opening.ratio > res_no_opening.ratio,
        "開口あり ratio={} <= 開口なし ratio={}",
        res_opening.ratio,
        res_no_opening.ratio
    );
}

/// 三方スリットが指定された壁は耐震壁として扱われず、耐震壁検定自体が
/// 出力されない。
#[test]
fn wall_with_three_side_slit_is_not_checked() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    let model = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: true,
        openings: vec![],
    }));
    assert!(wall_check_result(&model, &forces).is_none());
}

/// 開口周比 r0>0.4 となる大開口の壁も耐震壁として扱われず出力されない。
#[test]
fn wall_with_large_opening_ratio_is_not_checked() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    // opening_area = 0.5・l・h → r0 = sqrt(0.5) ≈ 0.707 > 0.4。
    let model = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.5 * 4000.0 * 3000.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    }));
    assert!(wall_check_result(&model, &forces).is_none());
}

/// `wall_attrs` に属性が無い壁（厚さ≥120mm）は、従来どおり無開口として
/// 耐震壁検定される。
#[test]
fn wall_without_attr_is_checked_as_no_opening() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    let model = wall_model(None);
    let res = wall_check_result(&model, &forces).expect("属性なしの壁も検定されるはず");
    assert!(res.ratio > 0.0);
}

/// 単一の個別開口（縦長: l0=750, h0=2000）と、同面積を合計面積のみで
/// 与えた場合（壁と同じ辺長比の擬似等価開口に復元される）とで、
/// γ支配項が変わるため検定比が一致しないこと。
///
/// 面積は共通（750×2000=1,500,000）のため開口周比 r0（耐震壁判定用）は
/// 両者で等しいが、実寸法は壁（l=4000,h=3000）と辺長比が異なる縦長形状
/// のため γ3=1−h0/h が支配的になり、擬似等価開口（壁と同じ辺長比）を
/// 使った場合の γ1=γ2=γ3 とは異なる低減係数 r になる。
#[test]
fn wall_single_opening_dims_differs_from_area_only_ratio() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];

    let model_single_dims = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![WallOpening {
            width: 750.0,
            height: 2000.0,
            offset: None,
        }],
    }));
    let res_single_dims = wall_check_result(&model_single_dims, &forces)
        .expect("r0<0.4 の単一開口は耐震壁として検定されるはず");

    let model_area_only = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 750.0 * 2000.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    }));
    let res_area_only = wall_check_result(&model_area_only, &forces)
        .expect("同面積を面積のみで与えた壁も耐震壁として検定されるはず");

    assert!(
        (res_single_dims.ratio - res_area_only.ratio).abs() > 1e-6,
        "個別寸法 ratio={} と面積のみ ratio={} が一致してしまっている",
        res_single_dims.ratio,
        res_area_only.ratio
    );
}

/// 複数開口（2個）は [`equivalent_opening`] による等価開口に統合され、
/// その等価開口を直接 `RcWallInput` へ供給した場合と同じ検定比になる。
#[test]
fn wall_multiple_openings_matches_equivalent_opening() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    let dims = [(600.0, 800.0), (500.0, 700.0)];

    let model = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: dims
            .iter()
            .map(|&(w, h)| WallOpening {
                width: w,
                height: h,
                offset: None,
            })
            .collect(),
    }));
    let res = wall_check_result(&model, &forces).expect("2個の開口は耐震壁として検定されるはず");

    // 期待値: equivalent_opening を直接呼んで壁と同じ辺長比の等価開口を
    // 構築し、同一の RcWallInput（側柱なし・l_clear=l）で検定した結果。
    let (l, h) = (4000.0_f64, 3000.0_f64);
    let (l0p, h0p) = equivalent_opening(&dims, l, h);
    let inp = RcWallInput {
        t: 180.0,
        l,
        l_clear: l,
        fc: 24.0,
        ps: 0.006,
        w_ft: crate::rc::rebar_allowable_shear("SD345", false),
        side_columns: vec![],
        opening: Some((l0p, h0p, h, l)),
        q_design: 500_000.0,
        long_term: false,
    };
    let expected = rc_wall_shear_check(&inp);

    assert!(
        (res.ratio - expected.ratio).abs() < 1e-9,
        "複数開口 ratio={} と等価開口直接計算 ratio={} が不一致",
        res.ratio,
        expected.ratio
    );
}

/// 個別開口の面積和で開口周比 r0>0.4 となる壁は耐震壁として扱われず、
/// 検定自体が出力されない。
#[test]
fn wall_multiple_openings_large_ratio_is_not_checked() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    // 開口2個の面積和 = 2,000,000 + 3,000,000 = 5,000,000
    // → r0 = sqrt(5,000,000 / (4000*3000)) = sqrt(0.41667) ≈ 0.645 > 0.4。
    let model = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![
            WallOpening {
                width: 2000.0,
                height: 1000.0,
                offset: None,
            },
            WallOpening {
                width: 2000.0,
                height: 1500.0,
                offset: None,
            },
        ],
    }));
    assert!(wall_check_result(&model, &forces).is_none());
}

/// 近接する2開口（水平純間隔200mm、高さ位置が一致）は、`Auto` モードでは
/// 包絡可能条件（純間隔が両開口の当該方向寸法の小さい方以下）を満たすため
/// 幅2000×高2000の単一の包絡開口に統合され、実寸法経路（単一開口）として
/// 検定される。既定の `Equivalent` モードでは個別開口のまま
/// `equivalent_opening` で等価開口に統合されるため、両モードで検定比が
/// 異なる（r0 の判定を通すため、壁は 8000×4000 とやや大きめに取る）。
#[test]
fn wall_auto_mode_envelopes_close_openings_and_differs_from_equivalent() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    let openings = vec![
        WallOpening {
            width: 1000.0,
            height: 2000.0,
            offset: Some([0.0, 0.0]),
        },
        WallOpening {
            width: 800.0,
            height: 2000.0,
            offset: Some([1200.0, 0.0]),
        },
    ];

    // 既定（Equivalent）モード: 個別開口のまま equivalent_opening で統合。
    let model_equiv = wall_model_sized(
        8000.0,
        4000.0,
        180.0,
        Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: openings.clone(),
        }),
    );
    let res_equiv = wall_check_result(&model_equiv, &forces)
        .expect("Equivalent モードは耐震壁として検定されるはず");

    // Auto モード: 純間隔(200)が両開口の幅(800,1000)以下・高さ方向の
    // 純間隔が 0（重なり）のため包絡可能 → 幅2000×高2000の単一開口
    // （実寸法経路）に統合される。
    let mut model_auto = wall_model_sized(
        8000.0,
        4000.0,
        180.0,
        Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings,
        }),
    );
    model_auto.multi_opening_mode = MultiOpeningMode::Auto;
    let res_auto = wall_check_result(&model_auto, &forces)
        .expect("Auto モードで包絡後も耐震壁として検定されるはず");

    // 期待値: 幅2000×高2000の単一開口を実寸法経路で直接検定した結果。
    let model_single = wall_model_sized(
        8000.0,
        4000.0,
        180.0,
        Some(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![WallOpening {
                width: 2000.0,
                height: 2000.0,
                offset: None,
            }],
        }),
    );
    let res_single = wall_check_result(&model_single, &forces)
        .expect("包絡開口相当の単一開口も耐震壁として検定されるはず");

    assert!(
        (res_auto.ratio - res_single.ratio).abs() < 1e-9,
        "Auto ratio={} と包絡開口(実寸法)直接計算 ratio={} が不一致",
        res_auto.ratio,
        res_single.ratio
    );
    assert!(
        (res_auto.ratio - res_equiv.ratio).abs() > 1e-6,
        "Auto ratio={} と Equivalent ratio={} が一致してしまっている",
        res_auto.ratio,
        res_equiv.ratio
    );
}

/// 遠く離れた小開口2つは、既定（Equivalent）モードでは面積和が小さく
/// 耐震壁として検定されるが、`Envelope` モードでは全開口を包絡した巨大な
/// 矩形の面積で開口周比 r0 を評価するため r0>0.4 となり、耐震壁として
/// 扱われず検定自体が出力されない。
#[test]
fn wall_envelope_mode_excludes_wall_when_envelope_ratio_too_large() {
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [0.0, 500_000.0, 0.0, 0.0, 0.0, 0.0])];
    let openings = vec![
        WallOpening {
            width: 200.0,
            height: 200.0,
            offset: Some([0.0, 0.0]),
        },
        WallOpening {
            width: 200.0,
            height: 200.0,
            offset: Some([3500.0, 2500.0]),
        },
    ];

    // 既定（Equivalent）モード: 面積和 = 200*200*2 = 80,000
    // → r0 = sqrt(80,000 / (4000*3000)) ≈ 0.0816 ≤ 0.4 で耐震壁として検定。
    let model_equiv = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: openings.clone(),
    }));
    assert!(
        wall_check_result(&model_equiv, &forces).is_some(),
        "Equivalent モードでは小開口のため耐震壁として検定されるはず"
    );

    // Envelope モード: 包絡矩形は幅3700×高2700 = 9,990,000
    // → r0 = sqrt(9,990,000 / (4000*3000)) ≈ 0.912 > 0.4 で耐震壁から除外。
    let mut model_envelope = wall_model(Some(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings,
    }));
    model_envelope.multi_opening_mode = MultiOpeningMode::Envelope;
    assert!(
        wall_check_result(&model_envelope, &forces).is_none(),
        "Envelope モードでは包絡矩形が大きく耐震壁から除外されるはず"
    );
}

/// 側柱付き耐震壁は、せん断非線形トリリニア骨格（Qc/βu/Qu、RESP-D「05 非線形
/// モデル」）が算定・出力される（付帯柱の主筋量が得られる壁のみ）。
#[test]
fn wall_with_side_columns_emits_nonlinear_shear_trilinear() {
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

    let mut model = wall_model(None);
    // 両側の鉛直辺（節点 0-3・1-2）に 600×600 RC 側柱を追加する。
    let col_shape = SectionShape::RcRect {
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
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(1), "C600".into()));
    for (eid, n0, n1) in [(1u32, 0u32, 3u32), (2u32, 1u32, 2u32)] {
        let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
        v.push(NodeId(n0));
        v.push(NodeId(n1));
        model.elements.push(ElementData {
            id: ElemId(eid),
            kind: ElementKind::Beam,
            nodes: v,
            section: Some(SectionId(1)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    // 圧縮軸力 1000kN・水平せん断 800kN・曲げ 1000kN·m（壁）。
    let forces: [(f64, [f64; 6]); 1] = [(0.0, [-1_000_000.0, 800_000.0, 0.0, 0.0, 0.0, 1.0e9])];
    // 側柱の内力（実アプリではソルバ結果に全要素が含まれる。側柱の主筋量を
    // 集計するため member_forces に側柱の内力エントリも渡す）。
    let col_forces: [(f64, [f64; 6]); 1] = [(0.0, [-500_000.0, 0.0, 0.0, 0.0, 0.0, 0.0])];
    let member_forces = vec![
        (ElemId(0), forces.as_slice()),
        (ElemId(1), col_forces.as_slice()),
        (ElemId(2), col_forces.as_slice()),
    ];
    let checks = collect_joint_checks(&model, &member_forces, LoadTerm::Short);

    let nl = checks
        .iter()
        .find(|(_, label, _)| label == "耐震壁(RC)せん断非線形");
    assert!(
        nl.is_some(),
        "側柱付き壁でせん断非線形トリリニアが出力される"
    );
    let (_, _, cr) = nl.unwrap();
    assert!(
        cr.detail.contains("Qc=") && cr.detail.contains("βu=") && cr.detail.contains("Qu="),
        "detail にトリリニア諸元が含まれる: {}",
        cr.detail
    );
    assert!(cr.ratio > 0.0, "Qu 検定比が正: {}", cr.ratio);

    // 側柱の無い壁（主筋量ゼロ）ではトリリニアは出力されない。
    let plain = collect_joint_checks(&wall_model(None), &member_forces, LoadTerm::Short);
    assert!(
        !plain
            .iter()
            .any(|(_, label, _)| label == "耐震壁(RC)せん断非線形"),
        "側柱の無い壁はトリリニア対象外"
    );
}
