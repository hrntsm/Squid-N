use super::*;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, Haunch, JointKind, LocalAxis, MemberDetailAttr, MemberJoint, Node,
    Section,
};

fn sample_model() -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: squid_n_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(squid_n_core::ids::StoryId(0)),
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "H-400".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 2000.0,
            j: 50.0,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_query_model_nodes() {
    let m = sample_model();
    let items = query_model(&m, "node", None);
    assert_eq!(items.len(), 2);
    assert_eq!(items[0]["id"], 0);
    assert_eq!(items[1]["story"], 0);
}

#[test]
fn test_query_model_elements_and_sections() {
    let m = sample_model();
    assert_eq!(query_model(&m, "member", None).len(), 1);
    let secs = query_model(&m, "section", None);
    assert_eq!(secs.len(), 1);
    assert_eq!(secs[0]["name"], "H-400");
}

#[test]
fn test_query_model_filter() {
    let m = sample_model();
    // 名前で絞り込み（断面名 H-400 を含むものだけ）。
    assert_eq!(query_model(&m, "section", Some("H-400")).len(), 1);
    assert_eq!(query_model(&m, "section", Some("RC")).len(), 0);
}

#[test]
fn test_query_model_unknown_kind() {
    let m = sample_model();
    assert!(query_model(&m, "bogus", None).is_empty());
}

/// 部材付帯情報（ハンチ・継手位置）が登録された部材は、`query_model` の
/// member/elements 出力に `haunch_i`/`haunch_j`/`joints` が含まれる。
/// 付帯情報が無い部材（本テストには含めない）は従来どおりのフィールドのみとなる
/// （`test_query_model_elements_and_sections` で確認済み）。
#[test]
fn test_query_model_elements_with_member_detail() {
    let mut m = sample_model();
    m.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: Some(Haunch {
            length: 500.0,
            depth_increase: 150.0,
            width_increase: 50.0,
        }),
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Shop,
        }],
    });
    let items = query_model(&m, "elements", None);
    assert_eq!(items.len(), 1);
    let e = &items[0];
    assert_eq!(e["haunch_i"]["length"], 700.0);
    assert_eq!(e["haunch_i"]["depth_increase"], 200.0);
    assert_eq!(e["haunch_j"]["width_increase"], 50.0);
    let joints = e["joints"].as_array().expect("joints 配列");
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0]["distance"], 1000.0);
    assert_eq!(joints[0]["kind"], "Shop");
}

/// RC 矩形の片持ち柱モデル（終局検定ジョブ用）。長期荷重ケース 1 つ。
fn rc_column_model() -> Model {
    use squid_n_core::model::{LoadCase, Material, NodalLoad};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 8,
            dia: 25.0,
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
    let shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar,
    };
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: squid_n_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(squid_n_core::ids::StoryId(0)),
            },
        ],
        sections: vec![shape.to_section(SectionId(0), "C600".into())],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: Some(345.0),
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: squid_n_core::ids::LoadCaseId(0),
            name: "長期".into(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [0.0, 0.0, -500_000.0, 0.0, 0.0, 0.0],
            }],
            member: Vec::new(),
        }],
        ..Default::default()
    }
}

#[test]
fn test_compute_ultimate_check_job() {
    let model = rc_column_model();
    let outcome = compute_job(&model, JobKind::UltimateCheck, &JobParams::default())
        .expect("終局検定ジョブは成功するはず");
    match outcome {
        JobOutcome::UltimateCheck { summary } => {
            assert_eq!(summary["kind"], "UltimateCheck");
            assert_eq!(summary["n_checks"], 1);
            // 柱 1 本のせん断余裕度・耐力が算定されている。
            let members = summary["members"].as_array().expect("members 配列");
            assert_eq!(members.len(), 1);
            assert!(members[0]["qsu"].as_f64().unwrap() > 0.0);
            assert!(members[0]["shear_margin"].as_f64().unwrap() > 0.0);
            // CFT 集計キーが存在する（本モデルは CFT 柱なしなので 0）。
            assert_eq!(summary["n_cft_checks"], 0);
            assert!(summary["cft_members"].is_array());
        }
        _ => panic!("expected UltimateCheck outcome"),
    }
}

/// DesignCheck ジョブは既定では危険断面位置（柱フェイス [face=0 につき節点芯]・
/// 中央）の 3 断面のみを検定する（付帯情報なし）。
#[test]
fn test_compute_design_check_job_default_positions() {
    let model = rc_column_model();
    let outcome = compute_job(&model, JobKind::DesignCheck, &JobParams::default())
        .expect("断面検定ジョブは成功するはず");
    match outcome {
        JobOutcome::DesignCheck { summary, .. } => {
            assert_eq!(summary["kind"], "DesignCheck");
            assert_eq!(summary["n_checks"], 3);
        }
        _ => panic!("expected DesignCheck outcome"),
    }
}

/// 部材付帯情報（継手位置）が登録された部材は、継手位置でも断面力が評価され
/// （squid-n-element の `eval_sections` 拡張）、DesignCheck の検定位置にも
/// 継手位置が加わる（既定 3 断面 + 継手 1 = 4 検定）。
#[test]
fn test_compute_design_check_job_member_detail_joint() {
    let mut model = rc_column_model();
    // 節点間距離 3000mm の柱に、始端から 1000mm（正規化 1/3）の現場継手を追加する。
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: None,
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Site,
        }],
    });
    let outcome = compute_job(&model, JobKind::DesignCheck, &JobParams::default())
        .expect("断面検定ジョブは成功するはず");
    match outcome {
        JobOutcome::DesignCheck {
            member_force_rows,
            summary,
            ..
        } => {
            assert_eq!(summary["kind"], "DesignCheck");
            // 継手位置 1000/3000 の断面力行が追加されている。
            assert!(member_force_rows
                .iter()
                .any(|(_, pos, _)| (pos - 1000.0 / 3000.0).abs() < 1e-6));
            // 継手位置分だけ検定数が増える（3 -> 4）。
            assert_eq!(summary["n_checks"], 4);
        }
        _ => panic!("expected DesignCheck outcome"),
    }
}

#[test]
fn test_job_registry_lifecycle() {
    let mut reg = JobRegistry::new();
    let id = reg.register(JobKind::LinearStatic);
    assert!(matches!(reg.get(&id).unwrap().status, JobStatus::Queued));
    reg.update(&id, JobStatus::Running { progress: 0.5 });
    assert!(matches!(
        reg.get(&id).unwrap().status,
        JobStatus::Running { progress } if (progress - 0.5).abs() < 1e-6
    ));
    reg.update(
        &id,
        JobStatus::Done {
            result_ref: "r1".into(),
        },
    );
    assert!(matches!(
        &reg.get(&id).unwrap().status,
        JobStatus::Done { result_ref } if result_ref == "r1"
    ));
    // 異なる ID は別ジョブ。
    let id2 = reg.register(JobKind::Eigen);
    assert_ne!(id, id2);
    assert!(reg.get("nonexistent").is_none());
}

#[test]
fn test_quantity_takeoff_json_column() {
    let model = rc_column_model();
    // 部位別（既定）: RC 柱 1 本 → 0.6×0.6×3.0 = 1.08 m³。
    let v = quantity_takeoff_json(&model, None);
    let rows = v["rows"].as_array().unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["category"], "柱");
    assert!((rows[0]["concrete_m3"].as_f64().unwrap() - 1.08).abs() < 1e-9);
    // 明細: 部材 1 件。合計と注記も返る。
    let detail = quantity_takeoff_json(&model, Some("detail"));
    assert_eq!(detail["rows"].as_array().unwrap().len(), 1);
    assert!(detail["totals"]["rebar_t"].as_f64().unwrap() > 0.0);
    assert!(!detail["notes"].as_array().unwrap().is_empty());
    // 鉄筋径別: D25（主筋）と D10（フープ）。
    let rebar = quantity_takeoff_json(&model, Some("rebar"));
    assert_eq!(rebar["rows"].as_array().unwrap().len(), 2);
}
