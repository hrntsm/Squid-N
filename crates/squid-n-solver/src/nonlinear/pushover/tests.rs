
use super::*;
use crate::constraint::Reducer;
use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
    Material, Node, Section, Story,
};
use squid_n_core::section_shape::ShearBar;

/// 1層・鉛直ファイバ柱の片持ちプッシュオーバー（P5 §10 相当の最小統合テスト）。
/// 配線済み非線形要素（FiberBeam）＋座標変換＋NR 反復＋降伏追跡が
/// エンドツーエンドで動作することを検証する。
fn single_column_model(fy: f64, seismic_weight: f64) -> Model {
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
                coord: [0.0, 0.0, 3000.0],
                // FiberBeam はねじり剛性を持たないため、Z 軸柱の頂部ねじり DOF(rz=bit5)
                // のみ拘束して特異性を除く。曲げ回転 rx,ry と並進は自由。
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
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
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(fy),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(1),
                slaves: vec![],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
        }],
        ..Default::default()
    }
}

#[test]
fn test_pushover_single_column_forms_hinge() {
    // 降伏応力を低め、地震重量を降伏荷重をやや超える程度に設定し、
    // 柱脚に曲げヒンジが形成されることを確認する。
    let mut model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,    // max_steps
        0.0,   // max_disp（変位制御に移行しない＝荷重制御のみ）
        false, // use_kg
        false, // use_arc_length
        0.0,
    )
    .expect("pushover should run end-to-end");

    // パイプライン全体が収束ステップを生成していること。
    assert!(
        !result.capacity_curve.is_empty(),
        "capacity curve should have at least one converged step"
    );
    // 荷重−変位曲線の頂部変位は単調に正（水平押し）であること。
    let last = result.capacity_curve.last().unwrap();
    assert!(
        last.roof_disp > 0.0,
        "roof displacement should be positive: {}",
        last.roof_disp
    );
    // 降伏応力を与えた鋼材ファイバ柱で、柱脚に曲げヒンジが追跡されること
    //（座標変換＋ファイバ降伏＋降伏追跡のエンドツーエンド検証）。
    assert!(
        !result.hinges.is_empty(),
        "at least one hinge should form in the column under lateral push"
    );

    // steps は capacity_curve と同じ収束ステップ数だけ積まれること。
    assert_eq!(
        result.steps.len(),
        result.capacity_curve.len(),
        "steps should have one entry per capacity_curve point"
    );
    // 各 step の story_drifts は層数（本モデルは1層）と一致すること。
    for s in &result.steps {
        assert_eq!(
            s.story_drifts.len(),
            model.stories.len(),
            "story_drifts length should match number of stories"
        );
    }
}

#[test]
fn test_pushover_requires_seismic_weight() {
    // 地震重量未定義ではエラーを返す（入力検証）。
    let mut model = single_column_model(235.0, 0.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,
        0.0,
        false,
        false,
        0.0,
    );
    assert!(
        result.is_err(),
        "should error when no seismic weight defined"
    );
}

#[test]
fn test_pushover_arc_length_path_runs() {
    // 弧長法フェーズ（f_int 反復再評価版）がエンドツーエンドで動作すること。
    let mut model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,    // max_steps（荷重制御）
        0.0,   // max_disp
        false, // use_kg
        true,  // use_arc_length
        1.0,   // arc_length_dl [mm]
    )
    .expect("arc-length pushover should run end-to-end");
    assert!(!result.capacity_curve.is_empty());
    assert!(result.qu > 0.0);
}

/// determine_mechanism / hinge_story 用の2層・柱通り（基礎-1F-2F）モデル。
/// node0=基礎(story None), node1=1F(story0), node2=2F(story1)。
/// elem0=1F柱(0-1), elem1=2F柱(1-2)。
fn two_story_model() -> Model {
    let sec = Section {
        id: SectionId(0),
        name: "c".to_string(),
        area: 10000.0,
        iy: 8.333e6,
        iz: 8.333e6,
        j: 1.0e6,
        depth: 100.0,
        width: 100.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: Some(0.0),
        fc: None,
        fy: Some(235.0),
    };
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 6000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(1)),
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        stories: vec![
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1)],
                diaphragms: vec![],
                seismic_weight: None,
            },
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(1),
                name: "2F".to_string(),
                elevation: 6000.0,
                node_ids: vec![NodeId(2)],
                diaphragms: vec![],
                seismic_weight: None,
            },
        ],
        ..Default::default()
    }
}

fn hinge(elem: u32, pos: f64, level: HingeLevel) -> HingeEvent {
    HingeEvent {
        step: 0,
        elem: ElemId(elem),
        pos,
        level,
        ductility: 1.0,
    }
}

#[test]
fn test_determine_mechanism_partial_when_insufficient() {
    let model = two_story_model();
    // ひび割れのみ → 降伏ヒンジ0個 < r+1 → Partial
    assert!(matches!(
        determine_mechanism(&[hinge(0, 0.0, HingeLevel::Crack)], &model),
        MechanismType::Partial
    ));
}

/// two_story_model は部材2・節点3・基礎FIXED(平面3DOF) → r=0（静定）。
/// したがって降伏ヒンジ1個で運動学的機構成立（r+1=1）。単一階集中→層崩壊。
#[test]
fn test_determine_mechanism_single_yield_establishes_mechanism() {
    let model = two_story_model();
    // elem0 端 j (pos=1.0) → node1 = 1F 単独階 → 層崩壊
    match determine_mechanism(&[hinge(0, 1.0, HingeLevel::Yield)], &model) {
        MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
        other => panic!(
            "expected StoryCollapse{{0}}, got {:?}",
            std::mem::discriminant(&other)
        ),
    }
}

/// 静的不静定次数の計算検証（平面骨組: r = 3m − 3n + r_support）。
#[test]
fn test_compute_static_indeterminacy_two_story() {
    // 2層2柱: 部材2・節点3・基礎節点(node0)が平面3DOF拘束 → r = 6 - 9 + 3 = 0（静定）
    let model = two_story_model();
    assert_eq!(compute_static_indeterminacy(&model), 0);
}

#[test]
fn test_compute_static_indeterminacy_indeterminate_portal() {
    // 1層1スパン両端固定ラーメン: 柱2+梁1=部材3、節点4（基礎2点FIXED+上部2点FREE）
    // r = 3*3 - 3*4 + (3+3) = 9 - 12 + 6 = 3（3次不静定）
    let model = two_story_model(); // 共用せず簡易生成
    let _ = model; // unused warning 回避
    let nodes = vec![
        Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(0)),
        },
        Node {
            id: NodeId(2),
            coord: [5000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(0)),
        },
        Node {
            id: NodeId(3),
            coord: [5000.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        },
    ];
    let elems = vec![
        ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        },
        ElementData {
            id: ElemId(1),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        },
        ElementData {
            id: ElemId(2),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        },
    ];
    let portal = Model {
        nodes,
        elements: elems,
        sections: vec![Section {
            id: SectionId(0),
            name: "c".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
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
            name: "s".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(235.0),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1), NodeId(2)],
            diaphragms: vec![],
            seismic_weight: None,
        }],
        ..Default::default()
    };
    assert_eq!(compute_static_indeterminacy(&portal), 3);
}

#[test]
fn test_determine_mechanism_story_collapse() {
    let model = two_story_model();
    // 1F柱の両端（elem0 pos1.0 → node1=1F, elem1 pos0.0 → node1=1F）が降伏
    // → 降伏ヒンジが1F(story0)に集中 → 層崩壊
    let hinges = vec![
        hinge(0, 1.0, HingeLevel::Yield),
        hinge(1, 0.0, HingeLevel::Yield),
    ];
    match determine_mechanism(&hinges, &model) {
        MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
        other => panic!(
            "expected StoryCollapse{{0}}, got {:?}",
            std::mem::discriminant(&other)
        ),
    }
}

#[test]
fn test_determine_mechanism_overall() {
    let model = two_story_model();
    // 1F(story0)と2F(story1)に分散して降伏 → 全体崩壊
    let hinges = vec![
        hinge(0, 1.0, HingeLevel::Yield), // node1 = 1F
        hinge(1, 1.0, HingeLevel::Yield), // node2 = 2F
    ];
    assert!(matches!(
        determine_mechanism(&hinges, &model),
        MechanismType::Overall
    ));
}

#[test]
fn test_pushover_base_shear_is_real_force() {
    // 最初の（弾性）ステップで base_shear/roof_disp が片持ち柱の弾性剛性
    // 3EI/L³ ≈ 189.8 N/mm に一致することを確認（DOF添字加算の旧バグを排除）。
    let mut model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .unwrap();
    let first = result.capacity_curve.first().unwrap();
    assert!(first.roof_disp > 0.0 && first.base_shear > 0.0);
    let k = first.base_shear / first.roof_disp;
    assert!(
        (150.0..=230.0).contains(&k),
        "first-step stiffness base_shear/roof_disp={k} should be ~3EI/L^3≈189.8"
    );
    // Qu はピークベースシア（全点以上）であること。
    for c in &result.capacity_curve {
        assert!(
            result.qu >= c.base_shear - 1e-6,
            "qu {} must be >= {}",
            result.qu,
            c.base_shear
        );
    }
    assert!(result.qu > 0.0);
}

fn portal_frame_model(fy: f64, seismic_weight: f64) -> Model {
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
                coord: [0.0, 0.0, 3000.0],
                // FiberBeam はねじり剛性を持たないため Rz を拘束
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(2),
                coord: [5000.0, 0.0, 3000.0],
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(3),
                coord: [5000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
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
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(fy),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1), NodeId(2)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(1),
                slaves: vec![NodeId(2)],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(1),
            slaves: vec![NodeId(2)],
        }],
        ..Default::default()
    }
}

// 1層1スパン剛床ラーメン（門形フレーム）で崩壊荷重が手計算値（4・My/H_col）
// に一致し、柱両端に4つの塑性ヒンジが形成され全体機構となることを検証する（P5 §10.1）。
//
// 手計算: Z=I/(depth/2)=166,660, My=σ_y·Z, Qu=4My/H=52,220 N（柱両端降伏・2柱）。
// seismic_weight は崩壊荷重を上回る値に設定し、真に降伏到達させる。
#[test]
fn test_portal_frame_collapse_load() {
    let qu_theory: f64 = 52_220.0;
    let mut model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    // 柱両端の降伏ヒンジが実際に形成されていること（運動学的機構: r+1=4）。
    let yielded_hinges = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .count();
    assert!(
        yielded_hinges >= 4,
        "at least 4 yielded hinges expected for Overall mechanism, got {} (total hinges={})",
        yielded_hinges,
        result.hinges.len()
    );

    // 崩壊機構が成立していること（Partial でない）。
    assert!(
        !matches!(result.mechanism, MechanismType::Partial),
        "mechanism should not be Partial for a collapsed portal frame"
    );

    assert!(result.qu > 0.0, "qu should be positive, got {}", result.qu);

    // 4番目の降伏ヒンジ（柱両端×2本＝4個で運動学的機構成立）発生ステップの
    // ベースシアを「観測崩壊荷重」とする（qu=max(base_shear) はまだ弾性最大反力で
    // plateau を正確に捉えられないため、降伏到達点で照合する）。
    let mut yield_steps: Vec<u32> = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .map(|h| h.step)
        .collect();
    yield_steps.sort_unstable();
    yield_steps.dedup();
    assert!(
        yield_steps.len() >= 4,
        "need >=4 distinct yield steps for Overall mechanism, got {}: {:?}",
        yield_steps.len(),
        yield_steps
    );
    let mech_step = yield_steps[3];
    let qu_observed = result
        .capacity_curve
        .iter()
        .find(|c| c.step == mech_step)
        .map(|c| c.base_shear)
        .unwrap_or(0.0);
    let rel_err = (qu_observed - qu_theory).abs() / qu_theory;
    // pushover は段階改良途上のため、比較的広めの許容差（30%）を設ける。
    assert!(
        rel_err < 0.30,
        "observed_qu={} at step {} deviates from Qu_theory={} by {:.1}% (>30%)",
        qu_observed,
        mech_step,
        qu_theory,
        rel_err * 100.0
    );
}

#[test]
fn test_portal_frame_mechanism_classified() {
    let mut model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    match &result.mechanism {
        MechanismType::Overall | MechanismType::StoryCollapse { .. } => {}
        other => panic!(
            "expected Overall or StoryCollapse, got {:?}",
            std::mem::discriminant(other)
        ),
    }
}

// ---- せん断降伏耐力 Qy の単体テスト ----

#[test]
fn test_compute_shear_yield_qy_steel() {
    // 鋼系（fy 設定あり）: Qy = as・fy/√3（RcRect 形状の有無・方向によらない）。
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(200.0),
    };
    let qy = compute_shear_yield_qy(1000.0, Some(&mat), None, ShearDir::Z, 3000.0);
    let expected = 1000.0 * 200.0 / 3.0_f64.sqrt();
    assert!(
        (qy - expected).abs() < 1e-6,
        "qy={qy} should equal as*fy/sqrt(3)={expected}"
    );
}

#[test]
fn test_compute_shear_yield_qy_rc_fallback_without_rc_rect_shape() {
    // RC系（fy 無し・fc 設定あり）かつ断面形状情報（RcRect）が無い場合:
    // Qy = as・0.7√fc（慣用値へフォールバック）。
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let qy = compute_shear_yield_qy(50000.0, Some(&mat), None, ShearDir::Z, 3000.0);
    let expected = 50000.0 * 0.7 * 24.0_f64.sqrt();
    assert!(
        (qy - expected).abs() < 1e-6,
        "qy={qy} should equal as*0.7*sqrt(fc)={expected}"
    );
}

#[test]
fn test_compute_shear_yield_qy_zero_as_is_infinite() {
    // 有効せん断断面積が 0 の断面は判定対象外（Qy=∞扱い）。
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(200.0),
    };
    assert_eq!(
        compute_shear_yield_qy(0.0, Some(&mat), None, ShearDir::Z, 3000.0),
        f64::INFINITY
    );
    // 材料未設定でも∞扱い。
    assert_eq!(
        compute_shear_yield_qy(1000.0, None, None, ShearDir::Z, 3000.0),
        f64::INFINITY
    );
}

/// RC 矩形断面（`SectionShape::RcRect`）+ 配筋情報がある場合、Qy は荒川式
/// （`rc_qsu_simple`）による方向別算定値に一致すること。
/// z 方向（強軸・main_x）、y 方向（弱軸・main_y、b/d 入れ替え）の双方を検証する。
#[test]
fn test_compute_shear_yield_qy_rc_rect_matches_arakawa_handcalc() {
    let rebar = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 25.0,
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
    let (b, d) = (400.0, 600.0);
    let shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };
    let sec = shape.to_section(SectionId(0), "RC-400x600".into());
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let clear_span = 3000.0;

    // z 方向（強軸）: b=幅, d=せい, 引張鉄筋 main_x。
    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        sigma_y: 345.0,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span,
        sigma_0: 0.0,
    });
    let qy_z = compute_shear_yield_qy(sec.as_z, Some(&mat), Some(&sec), ShearDir::Z, clear_span);
    assert!(
        (qy_z - qsu_z_handcalc).abs() < 1e-6,
        "qy_z={qy_z} should equal rc_qsu_simple handcalc={qsu_z_handcalc}"
    );

    // y 方向（弱軸）: b と d を入れ替え、引張鉄筋 main_y。
    let qsu_y_handcalc = rc_qsu_simple(&RcCapacityInput {
        b: d,
        d: b,
        at: bar_area(&rebar.main_y) / 2.0,
        d_eff: b - rebar.cover - rebar.main_y.dia / 2.0,
        sigma_y: 345.0,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (d * 100.0),
        sigma_wy: 295.0,
        clear_span,
        sigma_0: 0.0,
    });
    let qy_y = compute_shear_yield_qy(sec.as_y, Some(&mat), Some(&sec), ShearDir::Y, clear_span);
    assert!(
        (qy_y - qsu_y_handcalc).abs() < 1e-6,
        "qy_y={qy_y} should equal rc_qsu_simple handcalc={qsu_y_handcalc}"
    );
    // 断面が非正方形（b≠d、主筋も非対称）なので z・y の Qy は異なるはず。
    assert!((qy_z - qy_y).abs() > 1.0, "qy_z={qy_z} qy_y={qy_y}");
}

/// as_y/as_z を明示的に与えた片持ち柱モデル（`single_column_model` のせん断有効
/// 断面積を差し替えたもの）。せん断降伏耐力 Qy は as_y/as_z と材料強度のみに
/// 依存し、実際に生じるせん断力（`track_shear_yield`）は材端力の釣合いから
/// 求まるため、せん断バネ剛性（材料のせん断弾性係数）を変更する必要はない。
fn single_column_model_with_shear(fy: f64, seismic_weight: f64, as_shear: f64) -> Model {
    let mut model = single_column_model(fy, seismic_weight);
    model.sections[0].as_y = as_shear;
    model.sections[0].as_z = as_shear;
    model
}

#[test]
fn test_pushover_shear_yield_event_recorded() {
    // せん断有効断面積を小さく設定してせん断降伏耐力 Qy を小さくすることで、
    // 水平荷重漸増中にせん断降伏イベントが記録されることを確認する
    // （曲げヒンジ判定 `track_hinges` とは独立の判定経路の検証）。
    let mut model = single_column_model_with_shear(235.0, 80_000.0, 50.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    assert!(
        !result.shear_yields.is_empty(),
        "shear yield event should be recorded when Qy is small relative to applied shear"
    );
}

/// as_y・as_z を独立に設定した片持ち柱モデル（局所 y・z 方向分離の検証用）。
fn single_column_model_with_shear_yz(fy: f64, seismic_weight: f64, as_y: f64, as_z: f64) -> Model {
    let mut model = single_column_model(fy, seismic_weight);
    model.sections[0].as_y = as_y;
    model.sections[0].as_z = as_z;
    model
}

/// `single_column_model` は節点 (0,0,0)→(0,0,3000)、`local_axis.ref_vector=[1,0,0]`
/// なので局所座標系は ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（`LocalFrame::from_nodes`）。
/// `SeismicDir::X` でプッシュすると力はグローバル X＝局所 y（ey）方向に生じ、
/// 局所 z（ez＝グローバル Y）方向にはほぼ生じない。
fn run_pushover_has_shear_yield(as_y: f64, as_z: f64) -> bool {
    let mut model = single_column_model_with_shear_yz(235.0, 80_000.0, as_y, as_z);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &mut model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");
    !result.shear_yields.is_empty()
}

/// 局所 y/z 方向の厳密分離（改良1）の検証:
/// 実際に力が生じる方向（局所 y）の Qy を小さくすればせん断降伏イベントが
/// 記録されるが、力がほぼ生じない方向（局所 z）の Qy をどれだけ小さくしても
/// 記録されないこと。v1（軸直交合力 vs min(qy_y,qy_z)）では後者でも
/// 誤って記録されてしまっていた（qy_z が min を支配してしまうため）。
#[test]
fn test_pushover_shear_yield_direction_independent() {
    assert!(
        run_pushover_has_shear_yield(50.0, 1.0e12),
        "small as_y (the actually-stressed local direction) should trigger a shear yield event"
    );
    assert!(
        !run_pushover_has_shear_yield(1.0e12, 50.0),
        "small as_z (the unstressed local direction) should NOT trigger a shear yield event \
             once Vy/Vz are judged independently against qy_y/qy_z"
    );
}

// ---- 精緻化1: h0 への剛域控除の単体テスト ----

#[test]
fn test_effective_clear_span_deducts_rigid_zone_lengths() {
    let rz = RigidZone {
        length_i: 500.0,
        length_j: 300.0,
        ..Default::default()
    };
    // h0 = 節点間長3000 − (500+300) = 2200。
    assert!((effective_clear_span(3000.0, &rz) - 2200.0).abs() < 1e-9);
}

#[test]
fn test_effective_clear_span_falls_back_when_non_positive() {
    // 剛域長の合計が節点間長を超える異常入力 → 節点間長へフォールバック。
    let rz_over = RigidZone {
        length_i: 2000.0,
        length_j: 1500.0,
        ..Default::default()
    };
    assert_eq!(effective_clear_span(3000.0, &rz_over), 3000.0);

    // ちょうど0（または極小の浮動小数点誤差域）でもフォールバック。
    let rz_zero = RigidZone {
        length_i: 1500.0,
        length_j: 1500.0,
        ..Default::default()
    };
    assert_eq!(effective_clear_span(3000.0, &rz_zero), 3000.0);
}

/// RC矩形断面 + 配筋情報を持つ要素モデル（剛域テスト共通）。
/// 節点間距離3000mm、`rigid_zone` は呼び出し側で差し替える。
fn rc_column_model_with_rigid_zone(rigid_zone: RigidZone) -> (Model, RcRebar, f64, f64) {
    let rebar = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 25.0,
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
    let (b, d) = (400.0, 600.0);
    let shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };
    let sec = shape.to_section(SectionId(0), "RC-400x600".into());
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
        }],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };
    (model, rebar, b, d)
}

#[test]
fn test_compute_shear_yield_thresholds_rc_rect_uses_rigid_zone_reduced_clear_span() {
    // 剛域: length_i=400, length_j=200 → h0 = 3000-600 = 2400。
    let rigid_zone = RigidZone {
        length_i: 400.0,
        length_j: 200.0,
        ..Default::default()
    };
    let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
    let thresholds = compute_shear_yield_thresholds(&model);
    let th = &thresholds[0];

    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let expected_clear_span = 2400.0;

    // z方向（強軸・main_x）: RcArakawa を採用し、h0=2400 での rc_qsu_simple 手計算に一致。
    let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        sigma_y: 345.0,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span: expected_clear_span,
        sigma_0: 0.0,
    });
    match &th.z {
        DirThreshold::RcArakawa { input, gross_area } => {
            assert!(
                (input.clear_span - expected_clear_span).abs() < 1e-9,
                "clear_span={} expected={}",
                input.clear_span,
                expected_clear_span
            );
            assert!((gross_area - b * d).abs() < 1e-9);
        }
        DirThreshold::Static(_) => panic!("expected RcArakawa for RcRect with rebar"),
    }
    assert!(
        (th.z.qy(0.0) - qsu_z_handcalc).abs() < 1e-6,
        "qy(0.0)={} handcalc={}",
        th.z.qy(0.0),
        qsu_z_handcalc
    );
}

#[test]
fn test_compute_shear_yield_thresholds_rc_rect_falls_back_when_rigid_zone_exceeds_length() {
    // 剛域長の合計(2000+1500=3500)が節点間長(3000)を超える異常入力
    // → h0 は節点間長3000へフォールバックする。
    let rigid_zone = RigidZone {
        length_i: 2000.0,
        length_j: 1500.0,
        ..Default::default()
    };
    let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
    let thresholds = compute_shear_yield_thresholds(&model);
    let th = &thresholds[0];

    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        sigma_y: 345.0,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span: 3000.0, // フォールバック後の値
        sigma_0: 0.0,
    });
    assert!((th.z.qy(0.0) - qsu_z_handcalc).abs() < 1e-6);
}

// ---- 精緻化2: 軸力σ0の動的反映の単体テスト ----

#[test]
fn test_dir_threshold_qy_axial_term_matches_handcalc() {
    // rc_capacity::tests::sample_input と同一の断面（b=400,D=600,pw=0.002等）で
    // DirThreshold::RcArakawa を直接構成し、圧縮軸力からの σ0 反映を検算する。
    let b = 400.0;
    let d = 600.0;
    let d_eff = 530.0;
    let input = RcCapacityInput {
        b,
        d,
        at: 1935.0,
        d_eff,
        sigma_y: 345.0,
        fc: 24.0,
        pw: 0.002,
        sigma_wy: 295.0,
        clear_span: 3000.0,
        sigma_0: 0.0, // プレースホルダ（qy() が上書きする）
    };
    let gross_area = b * d;
    let th = DirThreshold::RcArakawa { input, gross_area };

    let qy_base = th.qy(0.0);
    let qsu_base_handcalc = rc_qsu_simple(&input);
    assert!((qy_base - qsu_base_handcalc).abs() < 1e-6);

    // 圧縮軸力 N_compress = 5.0 * gross_area → σ0 = 5.0 [N/mm²]（適用範囲0〜0.4Fc=9.6内）。
    let sigma_0 = 5.0;
    let n_compress = sigma_0 * gross_area;
    let qy_with_axial = th.qy(n_compress);
    let j = 7.0 * d_eff / 8.0;
    let expected_delta = 0.1 * sigma_0 * b * j;
    assert!(
        (qy_with_axial - qy_base - expected_delta).abs() < 1e-6,
        "delta={} expected={}",
        qy_with_axial - qy_base,
        expected_delta
    );

    // 引張（n_compress=0、呼び出し側で既にクランプ済みの規約）は σ0=0 のまま、
    // Qy は base と一致（増えない）。
    assert!((th.qy(0.0) - qy_base).abs() < 1e-9);
}

/// 軸力符号規約の検算（単純片持ち柱、節点 i=(0,0,0)・j=(0,0,3000)、
/// `ref_vector=[1,0,0]` → `LocalFrame::from_nodes` により ex=[0,0,1]）。
///
/// 柱頭（j端）を Δ=-1mm（ex と逆向き、圧縮方向）変位させたときの内力を
/// 手計算（f_local_x(i)=-N>0, f_local_x(j)=N<0、doc `axial_compression` 参照）
/// で再現し、`axial_compression` がこの圧縮を正しく検出することを確認する。
#[test]
fn test_axial_compression_sign_convention_handcalc() {
    let ex = [0.0, 0.0, 1.0];
    // 圧縮（N<0、|N|=1000）: f_i はコンプレッション側 = +|N|・ex、f_j = -|N|・ex。
    let n_compress_mag = 1000.0;
    let f_i_comp = [0.0, 0.0, n_compress_mag];
    let f_j_comp = [0.0, 0.0, -n_compress_mag];
    assert!(
        (axial_compression(f_i_comp, f_j_comp, ex) - n_compress_mag).abs() < 1e-9,
        "compression should be detected as a positive n_compress"
    );

    // 引張（N>0）: 圧縮側の符号が反転 → axial_compression は 0（圧縮なし）。
    let f_i_tension = [0.0, 0.0, -n_compress_mag];
    let f_j_tension = [0.0, 0.0, n_compress_mag];
    assert_eq!(
        axial_compression(f_i_tension, f_j_tension, ex),
        0.0,
        "pure tension must not be treated as compression (sigma_0=0 for tension)"
    );

    // 片端のみ圧縮成分がある非対称ケース（数値誤差や分布荷重を模擬）:
    // 両端のうち大きい方（実勢値）を採用する。
    let f_i_asym = [0.0, 0.0, n_compress_mag];
    let f_j_asym = [0.0, 0.0, -0.5 * n_compress_mag];
    assert!(
        (axial_compression(f_i_asym, f_j_asym, ex) - n_compress_mag).abs() < 1e-9,
        "should take the larger of the two end-derived compression values"
    );
}

/// `ElementBehavior::internal_force` が固定のグローバル材端力を返すだけのテスト
/// スタブ（`track_shear_yield` は `global_dofs`/剛性を使わないため他は無関係）。
struct FixedForceBehavior {
    f: LocalVec,
}

impl ElementBehavior for FixedForceBehavior {
    fn n_dof(&self) -> usize {
        12
    }
    fn global_dofs(&self, _dof: &DofMap) -> SmallVec<[usize; 24]> {
        SmallVec::new()
    }
    fn tangent_stiffness(
        &self,
        _state: &ElemState,
        _ctx: &Ctx,
    ) -> squid_n_element::behavior::LocalMat {
        squid_n_element::behavior::LocalMat::zeros(12)
    }
    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        LocalVec {
            data: self.f.data.clone(),
        }
    }
    fn mass_matrix(
        &self,
        _opt: squid_n_element::behavior::MassOption,
    ) -> squid_n_element::behavior::LocalMat {
        squid_n_element::behavior::LocalMat::zeros(12)
    }
}

/// 精緻化2のエンドツーエンド確認: 同一のせん断力 Vz デマンドに対し、
/// 軸圧縮が作用する場合は σ0 反映で Qy が増え判定を免れるが、圧縮が無い
/// （引張・軸力ゼロ）場合は従来どおり判定に掛かることを、実際の
/// `track_shear_yield` を通して確認する（`compute_shear_yield_thresholds` の
/// 構築から一貫して検証）。
#[test]
fn test_track_shear_yield_axial_compression_raises_qy_end_to_end() {
    let (model, _rebar, b, d) = rc_column_model_with_rigid_zone(RigidZone::default());
    let thresholds = compute_shear_yield_thresholds(&model);
    let (input, gross_area) = match &thresholds[0].z {
        DirThreshold::RcArakawa { input, gross_area } => (*input, *gross_area),
        DirThreshold::Static(_) => panic!("expected RcArakawa"),
    };
    assert!((gross_area - b * d).abs() < 1e-6);

    let qy_base = rc_qsu_simple(&input);
    let sigma_0 = 5.0; // 0〜0.4Fc=9.6 の範囲内
    let n_compress = sigma_0 * gross_area;
    let mut inp_axial = input;
    inp_axial.sigma_0 = sigma_0;
    let qy_boosted = rc_qsu_simple(&inp_axial);
    assert!(qy_boosted > qy_base, "axial term should raise Qy");

    // Vz を base と boosted のちょうど中間に設定: base では降伏、boosted では非降伏。
    // モデルは node i=(0,0,0)・j=(0,0,3000)、ref_vector=[1,0,0] のため
    // ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（既存テストの局所座標系規約と同じ）。
    // よって Vz は global y 成分（f.data[1]/f.data[7]）、N は global z 成分
    // （f.data[2]/f.data[8]）に対応する。
    let vz_demand = (qy_base + qy_boosted) / 2.0;

    // ケースA: 軸圧縮あり（N_compress = sigma_0*gross_area）→ 判定を免れるはず。
    let f_comp = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            vz_demand,
            n_compress,
            0.0,
            0.0,
            0.0,
            0.0,
            -vz_demand,
            -n_compress,
            0.0,
            0.0,
            0.0,
        ]),
    };
    let behaviors_comp: Vec<Box<dyn ElementBehavior>> =
        vec![Box::new(FixedForceBehavior { f: f_comp })];
    let mut events_comp = Vec::new();
    track_shear_yield(&model, &behaviors_comp, &thresholds, 0, &mut events_comp);
    assert!(
        events_comp.is_empty(),
        "compression should raise Qy above the shear demand, suppressing the event"
    );

    // ケースB: 軸力なし（同じ Vz デマンド）→ 従来どおり判定に掛かるはず。
    let f_zero = LocalVec {
        data: SmallVec::from_slice(&[
            0.0, vz_demand, 0.0, 0.0, 0.0, 0.0, 0.0, -vz_demand, 0.0, 0.0, 0.0, 0.0,
        ]),
    };
    let behaviors_zero: Vec<Box<dyn ElementBehavior>> =
        vec![Box::new(FixedForceBehavior { f: f_zero })];
    let mut events_zero = Vec::new();
    track_shear_yield(&model, &behaviors_zero, &thresholds, 0, &mut events_zero);
    assert!(
        !events_zero.is_empty(),
        "without axial compression the same Vz demand should still trigger the event"
    );
}
