use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, SectionId};
use squid_n_core::model::{
    DamperSpec, ElementData, EndCondition, ForceRegime, LoadCase, LoadCfg, LocalAxis, Material,
    MemberLoad, MiscWall, MiscWallTransfer, NodalLoad, Node, RigidZone, Section, WallAttr,
};

/// 2 層 × 1 スパンの平面ラーメン（各レベル 2 節点）。
fn two_story_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
        [0.0, 0.0, 7000.0],
        [6000.0, 0.0, 7000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    model.sections.push(Section {
        id: SectionId(0),
        name: "S".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    // 柱4 + 梁2
    let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
    for (i, (a, b)) in conn.iter().enumerate() {
        model.elements.push(ElementData {
            id: ElemId(i as u32),
            kind: ElementKind::Beam,
            nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![NodalLoad {
            node: NodeId(4),
            values: [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
        }],
        member: vec![MemberLoad {
            elem: ElemId(4),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 6000.0,
                w1: 10.0,
                w2: 10.0,
            },
        }],
    });
    model
}

#[test]
fn test_generate_two_stories() {
    let model = two_story_model();
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories.len(), 2);
    assert_eq!(gen.stories[0].elevation, 3500.0);
    assert_eq!(gen.stories[1].elevation, 7000.0);
    // 各階 2 節点 → 代表節点(慣性力重心)を新規生成 + スレーブ2（既存節点は全てスレーブ）
    assert_eq!(gen.stories[0].node_ids.len(), 2);
    assert_eq!(gen.stories[0].diaphragms[0].slaves.len(), 2);
    assert_eq!(gen.stories[1].diaphragms[0].slaves.len(), 2);
    assert_eq!(gen.constraints.len(), 2);
    // 基部節点は無所属
    assert_eq!(gen.node_story[0], None);
    assert_eq!(gen.node_story[2], Some(StoryId(0)));
    assert_eq!(gen.node_story[4], Some(StoryId(1)));
    // 重量: 1F = 梁分布荷重 10 N/mm × 6000 = 60 kN + 自重、2F = 節点荷重 50 kN + 自重
    let w1 = gen.stories[0].seismic_weight.unwrap();
    let w2 = gen.stories[1].seismic_weight.unwrap();
    assert!(w1 > 60000.0, "w1={}", w1);
    assert!(w2 > 50000.0, "w2={}", w2);

    // 代表節点は新規生成（既存節点数=6 の末尾連番）。
    assert_eq!(gen.rep_nodes.len(), 2);
    assert_eq!(gen.generated_masters, vec![NodeId(6), NodeId(7)]);
    for rep in &gen.rep_nodes {
        assert_eq!(rep.mass, None, "質量は Reducer 側の TᵀMT 縮約に委ねる");
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Uz));
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Rx));
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Ry));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));
    }
    assert_eq!(gen.rep_nodes[0].story, Some(StoryId(0)));
    assert_eq!(gen.rep_nodes[1].story, Some(StoryId(1)));
    // 1F は左右対称な自重＋分布荷重のみなので慣性力重心の X は中央(3000)になる。
    assert!((gen.rep_nodes[0].coord[0] - 3000.0).abs() < 1e-6);
    // 2F は節点荷重(50kN)が NodeId(4)(x=0)側のみに掛かる非対称配置なので、
    // 慣性力重心は x=0 側へ偏る(単純な幾何重心 3000 とは一致しない)。
    // 手計算(g=9806.65, §1.11): nw4=53656.6546..., nw5=3656.6546...,
    // gx = nw5*6000/(nw4+nw5) = 382.806855936086
    assert!(
        (gen.rep_nodes[1].coord[0] - 382.806855936086).abs() < 1e-6,
        "{}",
        gen.rep_nodes[1].coord[0]
    );
    assert_eq!(gen.rep_nodes[0].coord[2], 3500.0);
    assert_eq!(gen.rep_nodes[1].coord[2], 7000.0);
}

#[test]
fn test_generate_single_level_is_error() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    assert!(generate_stories(&model, None).is_err());
}

/// 重量が非対称な 1 層モデル（自重なし・節点荷重のみで重みを制御）。
fn asymmetric_weight_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![
            NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, -100000.0, 0.0, 0.0, 0.0],
            },
            NodalLoad {
                node: NodeId(3),
                values: [0.0, 0.0, -300000.0, 0.0, 0.0, 0.0],
            },
        ],
        member: vec![],
    });
    model
}

#[test]
fn test_generate_weighted_centroid_matches_hand_calc() {
    let model = asymmetric_weight_model();
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories.len(), 1);
    let story = &gen.stories[0];
    // 重量は自重なし・節点荷重のみ: 100kN + 300kN = 400kN
    assert_eq!(story.seismic_weight, Some(400000.0));
    assert_eq!(
        story.diaphragms[0].slaves.len(),
        2,
        "既存節点は全てスレーブ"
    );

    // 手計算: Gx = Σ(iW·ix)/ΣiW = (100000*0 + 300000*4000) / 400000 = 3000
    assert_eq!(gen.rep_nodes.len(), 1);
    let rep = &gen.rep_nodes[0];
    assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
    assert!((rep.coord[1] - 0.0).abs() < 1e-6, "Gy={}", rep.coord[1]);
    assert_eq!(rep.coord[2], 3000.0);
    assert_eq!(rep.mass, None);
    assert_eq!(rep.story, Some(StoryId(0)));
    assert!(rep.restraint.is_fixed(Dof::Uz));
    assert!(rep.restraint.is_fixed(Dof::Rx));
    assert!(rep.restraint.is_fixed(Dof::Ry));
    assert!(!rep.restraint.is_fixed(Dof::Ux));
    assert!(!rep.restraint.is_fixed(Dof::Uy));
    assert!(!rep.restraint.is_fixed(Dof::Rz));
    // 既存節点数=4 の末尾連番で新規生成される。
    assert_eq!(gen.generated_masters, vec![NodeId(4)]);
}

#[test]
fn test_generate_zero_weight_falls_back_to_geometric_centroid() {
    let mut model = Model::default();
    // 幾何重心が非対称になるよう配置（自重・荷重ケースなし → 重量ゼロ）。
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories[0].seismic_weight, Some(0.0));
    let rep = &gen.rep_nodes[0];
    // 幾何重心(単純平均) = (0 + 6000) / 2 = 3000
    assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
}

/// 基部(z=0, 固定)と上端(z=`len`, 自由)を結ぶ 1 部材の最小モデル。
/// 面控除・鉄骨割増率・付加線重量など「単一部材の自重」を検証する各テストの共通土台。
fn single_beam_model(
    len: f64,
    density: f64,
    area: f64,
    fc: Option<f64>,
    rigid_zone: RigidZone,
    load_cfg: Option<LoadCfg>,
) -> Model {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, len],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "S".into(),
        area,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "M".into(),
        young: 205000.0,
        poisson: 0.3,
        density,
        shear: None,
        fc,
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone,
        plastic_zone: None,
        spring: None,
    });
    model.load_cfg = load_cfg;
    model
}

#[test]
fn test_static_reactions_point_load_hand_calc() {
    // 単純梁 L=4000, a=1000, p=800: Ri=p(L-a)/L=600, Rj=p*a/L=200
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Point {
            a: 1000.0,
            p: 800.0,
        },
        4000.0,
    );
    assert!((ri - 600.0).abs() < 1e-9, "ri={}", ri);
    assert!((rj - 200.0).abs() < 1e-9, "rj={}", rj);
    assert!((ri + rj - 800.0).abs() < 1e-9);
}

#[test]
fn test_static_reactions_symmetric_distributed_is_half_half() {
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Distributed {
            a: 0.0,
            b: 6000.0,
            w1: 10.0,
            w2: 10.0,
        },
        6000.0,
    );
    assert!((ri - 30000.0).abs() < 1e-9, "ri={}", ri);
    assert!((rj - 30000.0).abs() < 1e-9, "rj={}", rj);
}

#[test]
fn test_static_reactions_asymmetric_distributed_hand_calc() {
    // 三角形分布(w1=0→w2=20)、a=0,b=4000,L=4000。
    // W=(0+20)/2*4000=40000, xbar=4000*(0+40)/(3*20)=2666.666...,
    // Rj=W*xbar/L=26666.666..., Ri=W-Rj=13333.333...
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Distributed {
            a: 0.0,
            b: 4000.0,
            w1: 0.0,
            w2: 20.0,
        },
        4000.0,
    );
    assert!((ri - 13333.333333333334).abs() < 1e-6, "ri={}", ri);
    assert!((rj - 26666.666666666668).abs() < 1e-6, "rj={}", rj);
    assert!((ri + rj - 40000.0).abs() < 1e-6);
}

#[test]
fn test_member_load_reaction_distribution_end_to_end() {
    // 自重を持たない(section/material 未設定)部材に非対称な三角形分布荷重を与え、
    // 剛床代表節点の重心が naive な 1/2-1/2 配分(x=2000)ではなく
    // 静定反力配分による偏った位置(x≈2666.67)になることを確認する（§1.4）。
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(2), NodeId(3)].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![],
        member: vec![MemberLoad {
            elem: ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 0.0,
                w2: 20.0,
            },
        }],
    });
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    let rep = &gen.rep_nodes[0];
    assert!(
        (rep.coord[0] - 2666.666666666667).abs() < 1e-2,
        "Gx={}",
        rep.coord[0]
    );
}

#[test]
fn test_face_reduction_applies_only_to_concrete() {
    // §1.8: RC/SRC の柱（鉛直材）は床上面から床上面（＝節点間距離。フェイス控除
    // しない）、S 柱も節点間距離。single_beam_model は鉛直材（柱）なので、
    // fc の有無によらず全長で算定される。
    let len = 4000.0;
    let area = 90000.0;
    let density = 2.4e-9;
    let rz = RigidZone {
        face_i: 300.0,
        face_j: 300.0,
        ..Default::default()
    };

    let rc_model = single_beam_model(len, density, area, Some(24.0), rz, None);
    let rc = generate_stories(&rc_model, None).unwrap();
    let expected_rc = density * area * len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
        "{}",
        rc.stories[0].seismic_weight.unwrap()
    );

    let s_model = single_beam_model(len, density, area, None, rz, None);
    let s = generate_stories(&s_model, None).unwrap();
    let expected_s = density * area * len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (s.stories[0].seismic_weight.unwrap() - expected_s).abs() < 1e-6,
        "{}",
        s.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_face_reduction_applies_to_horizontal_concrete_beam() {
    // §1.8: RC/SRC の水平材（梁）は柱面間距離（len − face_i − face_j）で算定する。
    // 鉛直材（柱）は同じフェイス値でも控除しない（前テストで検証）。
    let len = 6000.0;
    let area = 400.0 * 700.0;
    let density = 2.4e-9;
    let mut model = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [len, 0.0, 3000.0],
        [len, 0.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if c[2] == 0.0 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "RC".into(),
        area,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 22000.0,
        poisson: 0.2,
        density,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 水平梁（節点1→2）のみ断面・材料を持たせ、フェイス控除を検証する。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone {
            face_i: 400.0,
            face_j: 400.0,
            ..Default::default()
        },
        plastic_zone: None,
        spring: None,
    });

    let gen = generate_stories(&model, None).unwrap();
    let eff_len = len - 400.0 - 400.0;
    let expected = density * area * eff_len * GRAVITY_MM_S2;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "w={} expected={}",
        gen.stories[0].seismic_weight.unwrap(),
        expected
    );
}

#[test]
fn test_steel_weight_factor_applies_only_to_steel() {
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let cfg = LoadCfg {
        live_load_reduction: false,
        dampers: Vec::new(),
        finish_area_weight: Vec::new(),
        k_brace_rule: Default::default(),
        steel_weight_factor: 1.3,
        extra_line_weight: vec![],
    };

    let steel_model = single_beam_model(
        len,
        density,
        area,
        None,
        RigidZone::default(),
        Some(cfg.clone()),
    );
    let steel = generate_stories(&steel_model, None).unwrap();
    let expected_steel = density * area * len * GRAVITY_MM_S2 * 1.3 / 2.0;
    assert!(
        (steel.stories[0].seismic_weight.unwrap() - expected_steel).abs() < 1e-6,
        "{}",
        steel.stories[0].seismic_weight.unwrap()
    );

    let rc_model = single_beam_model(
        len,
        density,
        area,
        Some(24.0),
        RigidZone::default(),
        Some(cfg),
    );
    let rc = generate_stories(&rc_model, None).unwrap();
    let expected_rc = density * area * len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
        "割増率はコンクリート材に適用しない: {}",
        rc.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_extra_line_weight_adds_to_self_weight() {
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let cfg = LoadCfg {
        live_load_reduction: false,
        dampers: Vec::new(),
        finish_area_weight: Vec::new(),
        k_brace_rule: Default::default(),
        steel_weight_factor: 1.0,
        extra_line_weight: vec![(ElemId(0), 5.0)],
    };
    let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let expected = (density * area * len * GRAVITY_MM_S2 + 5.0 * len) / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

/// 矩形壁(4000×3000, t=150)を上下 2 レベルの節点間に張った 1 層モデル。
fn wall_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [4000.0, 0.0, 3000.0],
        [0.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    model.sections.push(Section {
        id: SectionId(0),
        name: "Wall".into(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: Some(150.0),
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: [NodeId(0), NodeId(1), NodeId(2), NodeId(3)]
            .into_iter()
            .collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    model
}

#[test]
fn test_wall_self_weight_included_in_story_weight() {
    // §1.2: 壁自重 w=ρ·t·A·g を全頂点に等分配。
    // 基部(z=0)側 2 節点は階に属さないため、階の地震用重量に算入されるのは
    // 上端 2 節点分(w/2)のみになる。
    let model = wall_model();
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories.len(), 1);
    let area = 4000.0 * 3000.0;
    let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
    let expected = w_total / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_wall_self_weight_uses_clear_dimensions_of_boundary_members() {
    // §壁自重: 耐震壁の重量は周辺の柱梁の内法寸法で計算する。
    // 側柱 500 角 ×2、上下梁 400×700 を壁の 4 辺に配置すると、
    // 内法係数 = (L−500/2×2)/L × (H−700/2×2)/H が芯々面積に乗じられる。
    let mut model = wall_model();
    // 側柱・上下梁用の断面（線材）。
    model.sections.push(Section {
        id: SectionId(1),
        name: "C500".into(),
        area: 0.0, // 自重 0（壁重量のみを観測するため）
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
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "G400x700".into(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    let line = |id: u32, sec: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(n0), NodeId(n1)].into_iter().collect(),
        section: Some(SectionId(sec)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    // 壁節点順は [0,1,2,3] = 下辺(0-1)・右柱(1-2)・上辺(2-3)・左柱(3-0)。
    model.elements.push(line(1, 1, 1, 2)); // 右側柱
    model.elements.push(line(2, 1, 3, 0)); // 左側柱
    model.elements.push(line(3, 2, 0, 1)); // 下梁
    model.elements.push(line(4, 2, 2, 3)); // 上梁

    let gen = generate_stories(&model, None).unwrap();
    let (l, h) = (4000.0_f64, 3000.0_f64);
    let factor = ((l - 2.0 * 250.0) / l) * ((h - 2.0 * 350.0) / h);
    let w_total = 2.4e-9 * 150.0 * (l * h * factor) * GRAVITY_MM_S2;
    let expected = w_total / 2.0; // 上端2節点分のみ階重量に算入
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "got={}, expected={}",
        gen.stories[0].seismic_weight.unwrap(),
        expected
    );
}

#[test]
fn test_generate_stories_multi_sums_multiple_gravity_cases_and_dedupes() {
    let mut model = asymmetric_weight_model();
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(1),
        name: "LL".into(),
        nodal: vec![NodalLoad {
            node: NodeId(2),
            values: [0.0, 0.0, -10000.0, 0.0, 0.0, 0.0],
        }],
        member: vec![],
    });

    // DL(400kN) + LL(10kN) = 410kN
    let gen = generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(1)]).unwrap();
    assert_eq!(gen.stories[0].seismic_weight, Some(410000.0));

    // 重複 ID は 1 回だけ処理される（二重計上しない）
    let gen_dup =
        generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(0), LoadCaseId(1)]).unwrap();
    assert_eq!(gen_dup.stories[0].seismic_weight, Some(410000.0));
}

// ------------------------------------------------------------------
// §壁開口・三方スリット
// ------------------------------------------------------------------

#[test]
fn test_wall_opening_deduction_and_opening_weight() {
    let mut model = wall_model();
    model.wall_attrs.push(WallAttr {
        elem: ElemId(0),
        opening_area: 1_000_000.0,
        opening_weight: 5000.0,
        three_side_slit: false,
        openings: vec![],
    });
    let gen = generate_stories(&model, None).unwrap();
    let area = 4000.0 * 3000.0;
    let net_area = area - 1_000_000.0;
    let w_total = (2.4e-9 * 150.0 * net_area * GRAVITY_MM_S2 + 5000.0).max(0.0);
    let expected = w_total / 2.0; // 上端2節点分(4節点等分の半分)
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_wall_opening_deduction_clamped_non_negative() {
    // 開口面積が壁面積を超える極端な入力でも自重が負にならない(clamp)。
    let mut model = wall_model();
    model.wall_attrs.push(WallAttr {
        elem: ElemId(0),
        opening_area: 4000.0 * 3000.0 * 2.0, // 壁面積を超える
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    });
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories[0].seismic_weight, Some(0.0));
}

#[test]
fn test_wall_three_side_slit_transfers_all_to_top_nodes() {
    // §壁自重: 三方スリットは壁荷重を全て上部の節点へ伝達する。
    // wall_model() の頂点は [z=0, z=0, z=3000, z=3000] の順で、上位2節点は
    // どちらも階に属する(基部でない)ため、通常配分(上端2節点でw_total/2)とは異なり
    // 階の地震用重量は w_total 全量になる。
    let mut model = wall_model();
    model.wall_attrs.push(WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: true,
        openings: vec![],
    });
    let gen = generate_stories(&model, None).unwrap();
    let area = 4000.0 * 3000.0;
    let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - w_total).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §フレーム外雑壁
// ------------------------------------------------------------------

#[test]
fn test_misc_wall_beam_transfer_conserves_total_weight() {
    // 長さ 1200mm(500+500+200 の端数分割)を Beam タイプで最近接節点へ集中。
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.misc_walls.push(MiscWall {
        start: [-600.0, 0.0, 2900.0],
        end: [600.0, 0.0, 2900.0],
        height: 200.0,
        weight_per_area: 1.0e-3,
        transfer: MiscWallTransfer::Beam,
        thickness: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // 領域中心の z は 2900+200/2=3000 で node1 に一致し、x も node1(x=0)に
    // node0(距離 3000超)より常に近いため、全量が node1(story0)へ集中する。
    let expected = 1.0e-3 * 200.0 * 1200.0; // weight_per_area * height * 壁長
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_misc_wall_column_transfer_splits_to_column_ends() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
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
    model.misc_walls.push(MiscWall {
        start: [-100.0, 0.0, 2900.0],
        end: [100.0, 0.0, 2900.0],
        height: 200.0,
        weight_per_area: 1.0e-3,
        transfer: MiscWallTransfer::Column,
        thickness: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    let total = 1.0e-3 * 200.0 * 200.0;
    // 唯一の柱(node0-node1)へ 1/2 ずつ。node0 は基部で階に属さないため、
    // 階の地震用重量に現れるのは node1 側(w/2)のみ。
    let expected = total / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §ダンパー自重
// ------------------------------------------------------------------

#[test]
fn test_damper_weight_replaces_section_self_weight() {
    let len = 4000.0;
    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 20000.0,
        device_length: 1000.0,
        support_area: 5000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let support_len = (len - 1000.0_f64).max(0.0);
    let w = 20000.0 + 5000.0 * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
    let expected = w / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_damper_zero_device_weight_counts_support_only() {
    // 「自重を考慮しない部材」: device_weight=0 かつ support_area>0 は支持部のみ算入。
    let len = 4000.0;
    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 0.0,
        device_length: 500.0,
        support_area: 8000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let support_len = (len - 500.0_f64).max(0.0);
    let w = 8000.0 * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
    let expected = w / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
    // 断面自重(ρ·A·L·g)は使われない(桁違いに大きい値とは一致しない)。
    let naive = 7.85e-9 * 90000.0 * len * GRAVITY_MM_S2 / 2.0;
    assert!((gen.stories[0].seismic_weight.unwrap() - naive).abs() > 1.0);
}

// ------------------------------------------------------------------
// §仕上げ面重量の自動換算
// ------------------------------------------------------------------

#[test]
fn test_finish_area_weight_column_perimeter_four_side() {
    // single_beam_model は鉛直材(柱)。φ=2(b+D)、b=D=300 (helper内の断面固定値)。
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let wf = 0.002;
    let cfg = LoadCfg {
        finish_area_weight: vec![(ElemId(0), wf)],
        ..Default::default()
    };
    let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let phi = 2.0 * (300.0 + 300.0);
    let expected = (density * area * len * GRAVITY_MM_S2 + wf * phi * len) / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_finish_area_weight_beam_perimeter_three_side() {
    // 水平梁(非鉛直)。φ=b+2D の三面仕上げ。
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [6000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "Beam".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
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
    });
    let wf = 0.0015;
    model.load_cfg = Some(LoadCfg {
        finish_area_weight: vec![(ElemId(0), wf)],
        ..Default::default()
    });
    let gen = generate_stories(&model, None).unwrap();
    let len = 6000.0;
    let phi = 300.0 + 2.0 * 600.0;
    // 両端(node1, node2)とも z=3000 の同一階に属するため、全量がその階に現れる。
    let expected = 7.85e-9 * 90000.0 * len * GRAVITY_MM_S2 + wf * phi * len;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §柱の長さ(下階柱なし時の柱脚梁せい付加)
// ------------------------------------------------------------------

#[test]
fn test_base_column_without_lower_column_adds_max_beam_depth() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    }); // 柱脚(下階柱なし) & 梁の一端
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    }); // 柱頭
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    }); // 梁の他端(基部)
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "Beam".into(),
        area: 0.0, // 自重寄与ゼロにして柱脚梁せい付加のみを検証する
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
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
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    let eff_len = 3000.0 + 600.0; // 柱長さ + 柱脚に取付く梁の最大せい
    let expected = 2.4e-9 * 90000.0 * eff_len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_base_column_with_lower_column_does_not_add_beam_depth() {
    // 下階に柱がある場合は梁せいを付加しない(誤って常時付加しないことの回帰確認)。
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, -3000.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    }); // 最下層(基部)
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    }); // 1F: 下階に柱(node0-node1)があるので梁せい付加なし
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    }); // 2F
    model.nodes.push(Node {
        id: NodeId(3),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    }); // 1F 位置に取付く梁の他端
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "ColLower".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "Beam".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 下階の柱(node0-node1)。area=0 で自重寄与ゼロ(有無の判定のみに使う)。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
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
    // 検証対象の柱(node1-node2)。
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
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
    // 1F(node1)に取付く梁(area=0、せい付加の誤検出があれば効いてしまう)。
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
        section: Some(SectionId(2)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // story0(1F, node1・node3) には下階柱(area0)/2 + 検証対象柱の下半分 + 梁(area0)/2。
    // 梁せい付加が誤って効いていれば eff_len が 3600 になり期待値からずれる。
    let w_upper = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2; // 梁せい付加なし(eff_len=3000)
    let expected_story0 = w_upper / 2.0;
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - expected_story0).abs() < 1e-6,
        "{}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §K型ブレースの重量配分
// ------------------------------------------------------------------

/// K型ブレース: 基準節点(node2, node3)から内部節点(node4)へ2本のブレースが
/// 集まる形。ブレース断面積を非対称にして配分規則による重心の違いを検出する。
fn k_brace_model(rule: KBraceWeightRule) -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [2000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
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
    // 柱(自重ゼロ、node2/node3 を「基準節点」化するために存在)
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    // ブレース1(node2-node4)
    model.sections.push(Section {
        id: SectionId(1),
        name: "Brace1".into(),
        area: 10000.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 200.0,
        width: 200.0,
        as_y: 1000.0,
        as_z: 1000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    // ブレース2(node3-node4): 面積を2倍にして非対称にする
    model.sections.push(Section {
        id: SectionId(2),
        name: "Brace2".into(),
        area: 20000.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 200.0,
        width: 200.0,
        as_y: 1000.0,
        as_z: 1000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    let axis = LocalAxis {
        ref_vector: [0.0, 0.0, 1.0],
    };
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Brace {
            tension_only: false,
        },
        nodes: [NodeId(2), NodeId(4)].into_iter().collect(),
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(3),
        kind: ElementKind::Brace {
            tension_only: false,
        },
        nodes: [NodeId(3), NodeId(4)].into_iter().collect(),
        section: Some(SectionId(2)),
        material: Some(MaterialId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.load_cfg = Some(LoadCfg {
        k_brace_rule: rule,
        ..Default::default()
    });
    model
}

#[test]
fn test_k_brace_base_nodes_only_shifts_centroid_toward_base_nodes() {
    let density = 7.85e-9;
    let len = 2000.0; // node2-node4, node3-node4 とも水平距離2000
    let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
    let w2 = density * 20000.0 * len * GRAVITY_MM_S2;

    let internal = k_brace_model(KBraceWeightRule::InternalNodes);
    let gen_internal = generate_stories(&internal, None).unwrap();
    // 手計算: node2=w1/2(x=0), node3=w2/2(x=4000), node4=(w1+w2)/2(x=2000)
    let expected_internal = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
    assert!(
        (gen_internal.rep_nodes[0].coord[0] - expected_internal).abs() < 1e-2,
        "{}",
        gen_internal.rep_nodes[0].coord[0]
    );

    let base_only = k_brace_model(KBraceWeightRule::BaseNodesOnly);
    let gen_base = generate_stories(&base_only, None).unwrap();
    // 手計算: node2=w1(x=0), node3=w2(x=4000), node4=0
    let expected_base = 4000.0 * w2 / (w1 + w2);
    assert!(
        (gen_base.rep_nodes[0].coord[0] - expected_base).abs() < 1e-2,
        "{}",
        gen_base.rep_nodes[0].coord[0]
    );

    // 両者は明確に異なる(基準節点側、より重いブレースが繋がる node3 側へ寄る)。
    assert!(gen_base.rep_nodes[0].coord[0] > gen_internal.rep_nodes[0].coord[0]);

    // 総重量(層重量)自体は配分規則によらず保存される。
    assert!(
        (gen_internal.stories[0].seismic_weight.unwrap()
            - gen_base.stories[0].seismic_weight.unwrap())
        .abs()
            < 1e-6
    );
}

#[test]
fn test_k_brace_internal_nodes_default_is_half_half() {
    // 既定(InternalNodes)は両端 1/2 ずつ(従来どおり)であることを回帰確認する。
    let mut model = k_brace_model(KBraceWeightRule::InternalNodes);
    model.load_cfg = None; // 既定値(LoadCfg::default())でも InternalNodes になることを確認
    let gen = generate_stories(&model, None).unwrap();
    let density = 7.85e-9;
    let len = 2000.0;
    let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
    let w2 = density * 20000.0 * len * GRAVITY_MM_S2;
    let expected = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
    assert!(
        (gen.rep_nodes[0].coord[0] - expected).abs() < 1e-2,
        "{}",
        gen.rep_nodes[0].coord[0]
    );
}
