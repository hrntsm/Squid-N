use super::*;
use crate::analysis::SeismicDir;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    DiaphragmDef, ElementData, ElementKind, ForceRegime, LocalAxis, Material, Node, Section, Story,
};

// ---- FEMA テーブル関数の単体テスト ----

fn base_params() -> FemaBeamParams {
    FemaBeamParams {
        b: 300.0,
        d: 450.0,
        depth_d: 500.0,
        rho: 0.01,
        rho_prime: 0.01,
        rho_bal: 0.02,
        s: 100.0,
        vs: 200_000.0,
        v_yield: 100_000.0,
        fc_prime: 24.0,
    }
}

#[test]
fn test_fema_is_c_true() {
    // s=100 <= D/3=166.7、Vs=200k >= 0.75*V=75k → C
    let p = base_params();
    assert!(fema_is_c(&p));
}

#[test]
fn test_fema_is_c_false_when_spacing_wide() {
    let mut p = base_params();
    p.s = 400.0; // > D/3
    assert!(!fema_is_c(&p));
}

#[test]
fn test_fema_is_c_false_when_shear_weak() {
    let mut p = base_params();
    p.vs = 50_000.0; // < 0.75*V
    assert!(!fema_is_c(&p));
}

/// テーブル4隅（C 側）を正確に再現すること。
#[test]
fn test_fema_plastic_rotation_corners_c() {
    // (ρ−ρ′)/ρbal <= 0.0 → rho=rho_prime とする。
    let mut p = base_params();
    p.rho = p.rho_prime; // ratio = 0.0
    p.b = 1.0;
    p.d = 1.0;
    p.fc_prime = 1.0;

    // V/(b*d*sqrt(fc')) <= 0.25 となるよう v_yield を設定。
    p.v_yield = 0.2; // C: s<=D/3, Vs>=0.75V は base_params の値から保持
    p.vs = 1.0e9; // 十分大きく C 判定を維持
    assert!((fema_plastic_rotation(&p) - 0.025).abs() < 1e-9);

    p.v_yield = 0.6; // vn=0.6 >= 0.5
    assert!((fema_plastic_rotation(&p) - 0.02).abs() < 1e-9);

    // ratio >= 0.5 側
    p.rho = p.rho_prime + 0.5 * p.rho_bal;
    p.v_yield = 0.2;
    assert!((fema_plastic_rotation(&p) - 0.02).abs() < 1e-9);

    p.v_yield = 0.6;
    assert!((fema_plastic_rotation(&p) - 0.015).abs() < 1e-9);
}

/// テーブル4隅（NC 側）を正確に再現すること。
#[test]
fn test_fema_plastic_rotation_corners_nc() {
    let mut p = base_params();
    p.s = 1000.0; // NC 確定
    p.b = 1.0;
    p.d = 1.0;
    p.fc_prime = 1.0;
    p.rho = p.rho_prime; // ratio=0.0

    p.v_yield = 0.2; // vn<=0.25
    assert!((fema_plastic_rotation(&p) - 0.02).abs() < 1e-9);

    p.v_yield = 0.6; // vn>=0.5
    assert!((fema_plastic_rotation(&p) - 0.01).abs() < 1e-9);

    p.rho = p.rho_prime + 0.5 * p.rho_bal; // ratio=0.5
    p.v_yield = 0.2;
    assert!((fema_plastic_rotation(&p) - 0.01).abs() < 1e-9);

    p.v_yield = 0.6;
    assert!((fema_plastic_rotation(&p) - 0.005).abs() < 1e-9);
}

/// 中間値の線形補間を検証する（ratio, vn ともに中間点で中間値になること）。
#[test]
fn test_fema_plastic_rotation_interpolation_midpoint() {
    let mut p = base_params();
    p.s = 100.0;
    p.depth_d = 500.0; // C 判定
    p.vs = 1.0e9;
    p.b = 1.0;
    p.d = 1.0;
    p.fc_prime = 1.0;
    // ratio の中間 (0.25) と vn の中間 (0.375) は C の4隅 (0.025,0.02,0.02,0.015) の
    // 双線形補間で中央値 (0.025+0.02+0.02+0.015)/4 = 0.02 となる。
    p.rho = p.rho_prime + 0.25 * p.rho_bal;
    p.v_yield = 0.375;
    let a = fema_plastic_rotation(&p);
    assert!(
        (a - 0.02).abs() < 1e-9,
        "midpoint bilinear interpolation should be 0.02, got {a}"
    );
}

// ---- staged_strength_loss の構造的検証 ----

/// 1層1スパン剛床ラーメン（門形フレーム）。柱脚に曲げ降伏が生じ、
/// 十分な変位で耐力喪失変形角にも到達するモデル。
fn portal_frame_model(fy: f64, seismic_weight: f64) -> Model {
    let sec = Section {
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
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: Some(0.0),
        fc: None,
        fy: Some(fy),
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
        sections: vec![sec],
        materials: vec![mat],
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
        constraints: vec![squid_n_core::model::Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(1),
            slaves: vec![NodeId(2)],
        }],
        ..Default::default()
    }
}

/// 耐力喪失変形角を極小に設定し、降伏後ただちに耐力喪失が検出されるようにした
/// 上で、staged_strength_loss が複数パス実行され、喪失部材が記録され、
/// 包絡線の頂部変位軸が単調であることを確認する（数値の厳密照合はしない）。
#[test]
fn test_staged_strength_loss_runs_multiple_passes() {
    let model = portal_frame_model(235.0, 600_000.0);
    let criterion = LossCriterion::DriftRange {
        start: 0.0,
        end: 1.0e-4,
    };

    let result = staged_strength_loss(
        &model,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
        &criterion,
        10,
    )
    .expect("staged strength loss should run end-to-end");

    assert!(
        result.passes.len() >= 2,
        "expected at least 2 reloading passes, got {}",
        result.passes.len()
    );
    assert!(
        !result.removed.is_empty(),
        "at least one member should be recorded as removed"
    );
    // 除去された部材はいずれか実在の部材であること。
    for (pass_idx, elem_id) in &result.removed {
        assert!(*pass_idx < result.passes.len());
        assert!(elem_id.index() < model.elements.len());
    }
    // 包絡線の頂部変位軸は単調非減少であること。
    for w in result.envelope.windows(2) {
        assert!(
            w[0].0 <= w[1].0 + 1e-9,
            "envelope roof_disp axis should be non-decreasing: {} then {}",
            w[0].0,
            w[1].0
        );
    }
    assert!(!result.envelope.is_empty());
}

/// 耐力喪失変形角を実務的にあり得ないほど大きく設定すると、
/// 喪失が一度も発生せずパス1回で終了すること。
#[test]
fn test_staged_strength_loss_single_pass_when_no_loss() {
    let model = portal_frame_model(235.0, 600_000.0);
    let criterion = LossCriterion::DriftRange {
        start: 10.0,
        end: 10.0,
    };

    let result = staged_strength_loss(
        &model,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
        &criterion,
        10,
    )
    .expect("staged strength loss should run end-to-end");

    assert_eq!(result.passes.len(), 1);
    assert!(result.removed.is_empty());
}

// ---- せん断降伏イベントに基づく耐力喪失判定の単体テスト ----

/// 曲げヒンジが1件も無くても、`PushoverResult::shear_yields` にせん断降伏
/// イベントが記録されていれば、それを「降伏後」として耐力喪失判定に用いる
/// ことを確認する（原典の「せん断降伏後、耐力喪失変形角に達した部材」判定の検証）。
#[test]
fn test_detect_strength_loss_uses_shear_yield_when_present() {
    use crate::pushover::{CapacityPoint, MechanismType, ShearYieldEvent};

    let model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let elem0 = model.elements[0].id;

    let n_active = dofmap.n_active();
    // node1（elem0 の j 端）の水平変位(dof0)を 100mm 与え、elem0 の変形角
    // ≈100/3000 ≈ 0.0333 rad が耐力喪失変形角(end=0.01)を超えるようにする。
    let mut disp1 = vec![0.0; n_active];
    if let Some(a) = dofmap.active(NodeId(1).index() * 6) {
        disp1[a as usize] = 100.0;
    }

    let result = PushoverResult {
        steps: vec![
            PushoverStep {
                load_factor: 0.5,
                top_disp: 0.0,
                base_shear: 0.0,
                story_drifts: vec![0.0],
                node_disp: Some(vec![0.0; n_active]),
            },
            PushoverStep {
                load_factor: 1.0,
                top_disp: 100.0,
                base_shear: 100.0,
                story_drifts: vec![100.0],
                node_disp: Some(disp1),
            },
        ],
        capacity_curve: vec![
            CapacityPoint {
                step: 0,
                roof_disp: 0.0,
                base_shear: 0.0,
                story_shear: vec![0.0],
                story_drift: vec![0.0],
            },
            CapacityPoint {
                step: 1,
                roof_disp: 100.0,
                base_shear: 100.0,
                story_shear: vec![100.0],
                story_drift: vec![100.0],
            },
        ],
        hinges: vec![], // 曲げ降伏イベントは無し（曲げヒンジ非依存であることの検証）
        shear_yields: vec![ShearYieldEvent {
            step: 0,
            elem: elem0,
        }],
        mechanism: MechanismType::Partial,
        qu: 100.0,
        member_response: vec![],
    };

    let criterion = LossCriterion::DriftRange {
        start: 0.0,
        end: 0.01,
    };
    let detected = detect_strength_loss(&model, &dofmap, &result, &criterion, &HashSet::new());
    assert!(
        detected.is_some(),
        "shear-yielded member exceeding the loss drift threshold should be detected"
    );
    let (step_no, elems) = detected.unwrap();
    assert_eq!(step_no, 1);
    assert!(
        elems.contains(&elem0),
        "elem0 (shear-yielded) should be among the removed elements"
    );
}

/// せん断降伏イベントが1件も無いモデルでは、曲げ降伏（`HingeLevel::Yield`
/// 以降）で代用するフォールバックが機能することを確認する。
#[test]
fn test_detect_strength_loss_falls_back_to_bending_yield_without_shear_events() {
    use crate::pushover::{CapacityPoint, HingeEvent, MechanismType};

    let model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let elem0 = model.elements[0].id;

    let n_active = dofmap.n_active();
    let mut disp1 = vec![0.0; n_active];
    if let Some(a) = dofmap.active(NodeId(1).index() * 6) {
        disp1[a as usize] = 100.0;
    }

    let result = PushoverResult {
        steps: vec![
            PushoverStep {
                load_factor: 0.5,
                top_disp: 0.0,
                base_shear: 0.0,
                story_drifts: vec![0.0],
                node_disp: Some(vec![0.0; n_active]),
            },
            PushoverStep {
                load_factor: 1.0,
                top_disp: 100.0,
                base_shear: 100.0,
                story_drifts: vec![100.0],
                node_disp: Some(disp1),
            },
        ],
        capacity_curve: vec![
            CapacityPoint {
                step: 0,
                roof_disp: 0.0,
                base_shear: 0.0,
                story_shear: vec![0.0],
                story_drift: vec![0.0],
            },
            CapacityPoint {
                step: 1,
                roof_disp: 100.0,
                base_shear: 100.0,
                story_shear: vec![100.0],
                story_drift: vec![100.0],
            },
        ],
        // 曲げ降伏ヒンジのみ記録（せん断降伏イベントは無し）。
        hinges: vec![HingeEvent {
            step: 0,
            elem: elem0,
            pos: 1.0,
            level: HingeLevel::Yield,
            ductility: 1.0,
        }],
        shear_yields: vec![],
        mechanism: MechanismType::Partial,
        qu: 100.0,
        member_response: vec![],
    };

    let criterion = LossCriterion::DriftRange {
        start: 0.0,
        end: 0.01,
    };
    let detected = detect_strength_loss(&model, &dofmap, &result, &criterion, &HashSet::new());
    assert!(
        detected.is_some(),
        "bending-yielded member exceeding the loss drift threshold should be detected \
             via the fallback path when no shear yield events exist"
    );
    let (step_no, elems) = detected.unwrap();
    assert_eq!(step_no, 1);
    assert!(elems.contains(&elem0));
}
