use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{EndCondition, LocalAxis, Material, Node, Section};

fn make_diaphragm_model() -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [5000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        constraints: vec![squid_n_core::model::Constraint::RigidDiaphragm {
            story: squid_n_core::ids::StoryId(0),
            master: NodeId(2),
            slaves: vec![NodeId(1)],
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "sec".into(),
            area: 100.0,
            iy: 833.33,
            iz: 833.33,
            j: 100.0,
            depth: 10.0,
            width: 10.0,
            as_y: 83.33,
            as_z: 83.33,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young: 20000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_resolve_force_regime_explicit() {
    let model = make_diaphragm_model();
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::UniaxialBendingShear,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    assert!(matches!(
        resolve_force_regime(&elem, &model),
        ResolvedRegime::ConcentratedSpring
    ));
}

#[test]
fn test_resolve_force_regime_auto() {
    let model = make_diaphragm_model();
    // 水平部材＋剛床あり → ConcentratedSpring
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    assert!(matches!(
        resolve_force_regime(&beam, &model),
        ResolvedRegime::ConcentratedSpring
    ));

    // 鉛直部材 → Fiber
    let col = ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    assert!(matches!(
        resolve_force_regime(&col, &model),
        ResolvedRegime::Fiber
    ));
}

#[test]
fn test_build_behavior_concentrated_spring_uses_spring_beam() {
    let model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let (behavior, _state) = build_behavior(&beam, &model);
    // ConcentratedSpringBeam は recover_forces を override していないので None
    assert!(
        behavior.recover_forces(&[0.0; 12]).is_none(),
        "ConcentratedSpringBeam should return None for recover_forces"
    );
    // snapshot_state で ConcentratedSpringBeam 固有型を確認
    let snap = behavior.snapshot_state();
    let is_spring = snap
        .downcast_ref::<(
            Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>,
            f64,
            f64,
            f64,
            f64,
        )>()
        .is_some();
    assert!(
        is_spring,
        "should be ConcentratedSpringBeam by snapshot type"
    );
}

#[test]
fn test_build_behavior_fiber_still_fiber() {
    let model = make_diaphragm_model();
    let col = ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let (behavior, _state) = build_behavior(&col, &model);
    // Fiber 分岐は暫定 BeamElement（線形解析）→ recover_forces は Some
    assert!(
        behavior.recover_forces(&[0.0; 12]).is_some(),
        "Fiber regime should use BeamElement for linear analysis"
    );
    assert_eq!(behavior.n_dof(), 12);
}

#[test]
fn test_build_nonlinear_behavior_concentrated_spring_uses_spring_beam() {
    let model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let (behavior, _state) = build_nonlinear_behavior(&beam, &model);
    let snap = behavior.snapshot_state();
    let is_spring = snap
        .downcast_ref::<(
            Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>,
            f64,
            f64,
            f64,
            f64,
        )>()
        .is_some();
    assert!(
        is_spring,
        "nonlinear ConcentratedSpring should be ConcentratedSpringBeam"
    );
}

#[test]
fn test_build_nonlinear_behavior_fiber_uses_fiber_beam() {
    let model = make_diaphragm_model();
    let col = ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let (behavior, _state) = build_nonlinear_behavior(&col, &model);
    let snap = behavior.snapshot_state();
    let is_fiber = snap
        .downcast_ref::<(
            [f64; 12],
            [f64; 12],
            Vec<Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>>,
        )>()
        .is_some();
    assert!(is_fiber, "nonlinear Fiber should be FiberBeam");
}

/// ブレース要素の生成モデル用（2 節点・軸方向 4000mm・断面積 2000mm2）。
fn make_brace_model(tension_only: bool) -> (Model, ElementData) {
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [4000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "brace".into(),
            area: 2000.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Brace { tension_only },
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    (model, elem)
}

/// 一般ブレース（引張専用でない）: build_behavior は factor=1.0 の TrussElement
/// を生成し、軸剛性 K = E·A/L に一致する（RESP-D マニュアル計算編02）。
#[test]
fn test_build_behavior_brace_normal_full_stiffness() {
    let (model, elem) = make_brace_model(false);
    let (behavior, state) = build_behavior(&elem, &model);
    let ctx = crate::behavior::Ctx { model: &model };
    let k = behavior.tangent_stiffness(&state, &ctx);
    let ea_l = 205000.0 * 2000.0 / 4000.0;
    assert!((k.get(0, 0) - ea_l).abs() < 1e-6, "k00={}", k.get(0, 0));
}

/// 引張専用ブレース: 弾性解析（build_behavior）では剛性を1/2にモデル化する
/// （マニュアル「引張と圧縮が対で存在するとみなし、弾性解析では剛性を1/2」）。
#[test]
fn test_build_behavior_brace_tension_only_half_stiffness() {
    let (model, elem) = make_brace_model(true);
    let (behavior, state) = build_behavior(&elem, &model);
    let ctx = crate::behavior::Ctx { model: &model };
    let k = behavior.tangent_stiffness(&state, &ctx);
    let ea_l = 205000.0 * 2000.0 / 4000.0;
    assert!(
        (k.get(0, 0) - 0.5 * ea_l).abs() < 1e-6,
        "k00={}",
        k.get(0, 0)
    );
}

/// 引張専用ブレース: 弾塑性解析（build_nonlinear_behavior）では初期剛性を
/// 1倍とする（マニュアル「弾塑性解析の場合は初期剛性は1倍とする」）。
#[test]
fn test_build_nonlinear_behavior_brace_tension_only_full_stiffness() {
    let (model, elem) = make_brace_model(true);
    let (behavior, state) = build_nonlinear_behavior(&elem, &model);
    let ctx = crate::behavior::Ctx { model: &model };
    let k = behavior.tangent_stiffness(&state, &ctx);
    let ea_l = 205000.0 * 2000.0 / 4000.0;
    assert!((k.get(0, 0) - ea_l).abs() < 1e-6, "k00={}", k.get(0, 0));
}

/// 壁要素の開口低減: wall_attrs の開口面積からせん断剛性が低減されること
/// （RESP-D 計算編 02「剛性計算」耐震壁の開口低減 r=1−1.25·√(開口面積/壁面積)）。
#[test]
fn test_build_behavior_wall_opening_reduces_shear_stiffness() {
    use squid_n_core::model::WallAttr;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "wall".into(),
            area: 150.0 * 1000.0,
            iy: 1.0e9,
            iz: 1.0e9,
            j: 1.0e9,
            depth: 1000.0,
            width: 150.0,
            as_y: 125_000.0,
            as_z: 125_000.0,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        ..Default::default()
    };
    let wall = ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    // 壁エレメント(24自由度)の面内せん断・鉛直軸のエネルギーパターン。
    // 内部節点順は z ソート([0,1] 下辺, [3,2] 上辺)のため、上辺の
    // スロットは 2(node3)・3(node2)。
    let shear_pattern = |k: &crate::behavior::LocalMat| -> f64 {
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0;
        u[3 * 6] = 1.0;
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    };
    let axial_pattern = |k: &crate::behavior::LocalMat| -> f64 {
        let mut u = [0.0; 24];
        u[2 * 6 + 2] = 1.0;
        u[3 * 6 + 2] = 1.0;
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    };

    // 開口なし
    let (b_no, state) = build_behavior(&wall, &model);
    let ctx = crate::behavior::Ctx { model: &model };
    let k_no = b_no.tangent_stiffness(&state, &ctx);

    // 開口 10%（壁 4000×3000=12e6 mm² に対し 1.2e6 mm²）→ r0=0.316(耐震壁
    // 成立のまま)、r=1−1.25·0.316=0.605
    model.wall_attrs.push(WallAttr {
        elem: ElemId(0),
        opening_area: 1.2e6,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    });
    let (b_open, state2) = build_behavior(&wall, &model);
    let ctx2 = crate::behavior::Ctx { model: &model };
    let k_open = b_open.tangent_stiffness(&state2, &ctx2);

    // 個別開口(合計 1.2e6 mm²)は面積のみ指定と同じ低減率になる。
    // また opening_area(古い値)より個別開口が優先される。
    model.wall_attrs[0] = WallAttr {
        elem: ElemId(0),
        opening_area: 1.0, // 無視される(個別開口が優先)
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![
            squid_n_core::model::WallOpening {
                width: 1000.0,
                height: 800.0,
                offset: Some([0.0, 500.0]),
            },
            squid_n_core::model::WallOpening {
                width: 500.0,
                height: 800.0,
                offset: Some([1800.0, 500.0]),
            },
        ],
    };
    let (b_dims, state3) = build_behavior(&wall, &model);
    let ctx3 = crate::behavior::Ctx { model: &model };
    let k_dims = b_dims.tangent_stiffness(&state3, &ctx3);
    assert!(
        (shear_pattern(&k_dims) - shear_pattern(&k_open)).abs() < 1e-6,
        "個別開口(Σ1.2e6)と面積のみ(1.2e6)の低減が一致しない: {} vs {}",
        shear_pattern(&k_dims),
        shear_pattern(&k_open)
    );

    // 包絡モード: 離れた2開口の包絡矩形(2300×800=1.84e6、r0=0.39≦0.4 で
    // 耐震壁成立のまま)により低減がさらに大きくなる
    model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Envelope;
    let (b_env, state4) = build_behavior(&wall, &model);
    let ctx4 = crate::behavior::Ctx { model: &model };
    let k_env = b_env.tangent_stiffness(&state4, &ctx4);
    assert!(
        shear_pattern(&k_env) < shear_pattern(&k_dims) * 0.999,
        "包絡モードで低減が強まらない: env={} eq={}",
        shear_pattern(&k_env),
        shear_pattern(&k_dims)
    );
    model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Equivalent;

    // せん断剛性の低減で面内せん断が小さくなる（鉛直軸剛性 EA/h は不変）
    assert!(
        shear_pattern(&k_open) < shear_pattern(&k_no) * 0.999,
        "shear open={} no={}",
        shear_pattern(&k_open),
        shear_pattern(&k_no)
    );
    assert!((axial_pattern(&k_open) - axial_pattern(&k_no)).abs() < 1e-6);
}

#[test]
fn test_resolve_member_hysteresis_and_flexural_springs() {
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    fn rebar() -> RcRebar {
        RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
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
        }
    }

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());

    // 断面形状なし → 非 RC → 標準型（バイリニア、N-M 相関対象）。
    assert!(!is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model),
        HysteresisModel::Standard
    );
    let (_i, _j, use_mn) = build_flexural_springs(&beam, &model, HysteresisModel::Standard);
    assert!(use_mn);

    // RcRect + Fc → RC 系 → 既定=武田型（履歴材料、N-M 相関対象外）。
    model.sections[0].shape = Some(SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    });
    model.sections[0].depth = 700.0;
    model.sections[0].width = 400.0;
    model.sections[0].iz = 400.0 * 700.0f64.powi(3) / 12.0;
    model.materials[0].fc = Some(24.0);
    model.materials[0].fy = Some(345.0);
    assert!(is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model),
        HysteresisModel::Takeda
    );
    let (_i, _j, use_mn) = build_flexural_springs(&beam, &model, HysteresisModel::Takeda);
    assert!(!use_mn, "武田型(履歴材料)は N-M 相関(set_yield)対象外");

    // SteelH → 非 RC → 標準型。
    model.sections[0].shape = Some(SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    });
    assert!(!is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model),
        HysteresisModel::Standard
    );

    // 個別指定は既定表に優先する。
    model.set_member_hysteresis(ElemId(0), HysteresisModel::MaxPointOriented);
    assert_eq!(
        resolve_member_hysteresis(&beam, &model),
        HysteresisModel::MaxPointOriented
    );
    let (_i, _j, use_mn) = build_flexural_springs(&beam, &model, HysteresisModel::MaxPointOriented);
    assert!(!use_mn);
}

#[test]
fn test_rc_beam_flexural_spring_exhibits_takeda_degradation() {
    // RC 梁の材端バネが解析で実際に武田型（除荷剛性が初期剛性より低下）で
    // 応答することを、返却された復元力材料を直接駆動して確認する。
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());
    model.sections[0].shape = Some(SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
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
    });
    model.sections[0].depth = 700.0;
    model.sections[0].width = 400.0;
    model.sections[0].iz = 400.0 * 700.0f64.powi(3) / 12.0;
    model.materials[0].young = 25_000.0;
    model.materials[0].fc = Some(24.0);
    model.materials[0].fy = Some(345.0);

    let rule = resolve_member_hysteresis(&beam, &model);
    assert_eq!(rule, HysteresisModel::Takeda);
    let (mut si, _sj, use_mn) = build_flexural_springs(&beam, &model, rule);
    assert!(!use_mn);

    // 初期（弾性）接線。
    let (_m0, k0) = si.trial(1e-8);
    si.commit();
    assert!(k0 > 0.0);

    // 十分大きい回転で降伏させ、スケルトン上のモーメントを得る。
    let big = 0.02_f64;
    let (m_peak, _) = si.trial(big);
    si.commit();
    assert!(m_peak > 0.0, "should carry positive moment at peak");

    // 除荷: 武田型の除荷剛性は初期剛性より小さい（剛性低下）。
    let (m1, _) = si.trial(big * 0.95);
    let (m2, _) = si.trial(big * 0.90);
    let ku = (m1 - m2) / (big * 0.05);
    assert!(
        ku < k0 * 0.999,
        "Takeda unloading stiffness ({ku}) must be below initial ({k0})"
    );
    assert!(ku > 0.0, "unloading stiffness must stay positive");
}

#[test]
fn test_steel_beam_flexural_spring_buckling_degrades() {
    // 鉄骨梁に座屈考慮型を個別指定 → 材端バネが最大耐力後に耐力劣化することを、
    // 返却された復元力材料を直接駆動して確認する。
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::SectionShape;

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());
    model.sections[0].shape = Some(SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    });
    model.sections[0].depth = 400.0;
    model.sections[0].width = 200.0;
    model.sections[0].iz = 200.0 * 400.0f64.powi(3) / 12.0;
    model.materials[0].fy = Some(325.0);
    model.set_member_hysteresis(ElemId(0), HysteresisModel::SteelBuckling);

    let rule = resolve_member_hysteresis(&beam, &model);
    assert_eq!(rule, HysteresisModel::SteelBuckling);
    let (mut si, _sj, use_mn) = build_flexural_springs(&beam, &model, rule);
    assert!(use_mn, "座屈考慮型は set_yield 対応で N-M 相関適用可");

    let (_m0, k0) = si.trial(1e-9);
    si.commit();
    assert!(k0 > 0.0);
    // 単調載荷でピーク → さらに大変形で耐力劣化。
    let theta_y = {
        // My は spring 内部だが、θy≈small。大きめの回転で骨格の各域を通過させる。
        1e-3
    };
    let mut m_max = 0.0_f64;
    let mut m_last = 0.0_f64;
    for i in 1..=200 {
        let th = theta_y * i as f64 * 0.5;
        let (m, _) = si.trial(th);
        si.commit();
        m_max = m_max.max(m);
        m_last = m;
    }
    assert!(m_max > 0.0);
    assert!(
        m_last < m_max * 0.999,
        "buckling degradation: last M ({m_last}) must fall below peak ({m_max})"
    );
}
