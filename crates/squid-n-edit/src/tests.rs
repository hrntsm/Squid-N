use super::*;
use smallvec::smallvec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_core::ids::*;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node};

fn empty_model() -> Model {
    Model::default()
}

#[test]
fn test_set_node_coord_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    let mut stack = UndoStack::new();

    let cmd = SetNodeCoord {
        node: NodeId(0),
        coord: [1000.0, 2000.0, 0.0],
    };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);

    stack.undo(&mut model);
    assert_eq!(model.nodes[0].coord, [0.0, 0.0, 0.0]);

    stack.redo(&mut model);
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);
}

#[test]
fn test_set_node_coord_invalid_id_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeCoord {
            node: NodeId(99),
            coord: [1.0, 2.0, 3.0],
        }),
    );
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.nodes.is_empty());
}

#[test]
fn test_set_node_restraint_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetNodeRestraint {
            node: NodeId(0),
            restraint: Dof6Mask::PINNED,
        }),
    );
    assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);

    stack.undo(&mut model);
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);

    stack.redo(&mut model);
    assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);
}

#[test]
fn test_set_node_restraint_invalid_id_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeRestraint {
            node: NodeId(99),
            restraint: Dof6Mask::FIXED,
        }),
    );
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.nodes.is_empty());
}

#[test]
fn test_add_node_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(AddNode {
            coord: [1000.0, 2000.0, 3000.0],
            restraint: Dof6Mask::FREE,
        }),
    );
    assert_eq!(model.nodes.len(), 1);
    assert_eq!(model.nodes[0].id, NodeId(0));
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 3000.0]);

    stack.undo(&mut model);
    assert_eq!(model.nodes.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 1);
    assert_eq!(model.nodes[0].id, NodeId(0));
}

#[test]
fn test_add_node_id_equals_index() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    for i in 0..3 {
        stack.run(
            &mut model,
            Box::new(AddNode {
                coord: [i as f64, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
            }),
        );
    }
    for (i, node) in model.nodes.iter().enumerate() {
        assert_eq!(node.id, NodeId(i as u32));
    }
}

#[test]
fn test_delete_node_middle_renumbers_and_roundtrips() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    for i in 0..3 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    // 末尾の節点（N2）を使う部材を用意し、中間節点（N1）削除後に
    // 参照が N1 へ繰り上がることを確認する。
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(1) }));
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.nodes[0].id, NodeId(0));
    assert_eq!(model.nodes[1].id, NodeId(1));
    assert_eq!(model.nodes[1].coord, [2.0, 0.0, 0.0]);
    // 元 N2 だった部材参照は N1 に繰り上がる
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    for (i, node) in model.nodes.iter().enumerate() {
        assert_eq!(node.id, NodeId(i as u32));
        assert_eq!(node.coord, [i as f64, 0.0, 0.0]);
    }
    assert_eq!(model.elements[0].nodes.to_vec(), vec![NodeId(0), NodeId(2)]);
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());
}

#[test]
fn test_delete_node_in_use_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    // 部材に使われている節点の削除は Noop（先に部材を削除する必要がある）
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
    assert_eq!(model.nodes.len(), 2);
    assert!(model.validate().is_ok());
}

#[test]
fn test_add_delete_member_load_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mut model = empty_model();
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![],
    });
    let mut stack = UndoStack::new();
    let load = MemberLoad {
        elem: squid_n_core::ids::ElemId(0),
        dir: [0.0, 0.0, -1.0],
        kind: MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 2.0,
            w2: 2.0,
        },
    };
    stack.run(
        &mut model,
        Box::new(AddMemberLoad {
            lc: LoadCaseId(0),
            load: load.clone(),
        }),
    );
    assert_eq!(model.load_cases[0].member.len(), 1);
    assert_eq!(model.load_cases[0].member[0], load);

    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 1);

    // 削除と復元（位置保持）
    stack.run(
        &mut model,
        Box::new(DeleteMemberLoad {
            lc: LoadCaseId(0),
            index: 0,
        }),
    );
    assert_eq!(model.load_cases[0].member.len(), 0);
    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 1);
    assert_eq!(model.load_cases[0].member[0], load);
}

#[test]
fn test_delete_member_undo_preserves_member_load_order() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mk_elem = |id: u32| ElementData {
        id: squid_n_core::ids::ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mk_load = |elem: u32, w: f64| MemberLoad {
        elem: squid_n_core::ids::ElemId(elem),
        dir: [0.0, 0.0, -1.0],
        kind: MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: w,
            w2: w,
        },
    };
    let w1_of = |l: &MemberLoad| match l.kind {
        MemberLoadKind::Distributed { w1, .. } => w1,
        _ => 0.0,
    };

    let mut model = empty_model();
    model.elements = vec![mk_elem(0), mk_elem(1), mk_elem(2)];
    // 荷重順 [e0:1, e1:2, e2:3, e1:4]。elem1 に 2 つの荷重が離れて存在する。
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![
            mk_load(0, 1.0),
            mk_load(1, 2.0),
            mk_load(2, 3.0),
            mk_load(1, 4.0),
        ],
    });

    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(1) }));
    // elem1 の荷重（w=2,4）が除去され、残りは [1,3]。
    let after: Vec<f64> = model.load_cases[0].member.iter().map(w1_of).collect();
    assert_eq!(after, vec![1.0, 3.0]);

    stack.undo(&mut model);
    // undo は削除前の順序 [1,2,3,4] を厳密に復元しなければならない
    // （前方順挿入では [1,2,4,3] と入れ替わっていた）。
    let restored: Vec<f64> = model.load_cases[0].member.iter().map(w1_of).collect();
    assert_eq!(
        restored,
        vec![1.0, 2.0, 3.0, 4.0],
        "undo は部材荷重の順序を厳密に復元する"
    );
}

#[test]
fn test_add_delete_member_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    let elem = ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    stack.run(&mut model, Box::new(AddMember { elem }));
    assert_eq!(model.elements.len(), 1);
    stack.undo(&mut model);
    assert_eq!(model.elements.len(), 0);
    stack.redo(&mut model);
    assert_eq!(model.elements.len(), 1);
}

#[test]
fn test_add_section_shape_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let cmd = AddSectionShape {
        shape,
        new_id: SectionId(0),
        name: "H-300x300x10x15".into(),
    };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));
}

#[test]
fn test_edit_section_shape_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape1 = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape1.to_section(SectionId(0), "H-300".into());
    let area_h = sec.area;
    model.sections.push(sec);

    let shape2 = squid_n_section::shape::SectionShape::SteelBox {
        height: 200.0,
        width: 200.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let cmd = EditSectionShape {
        section: SectionId(0),
        new_shape: shape2,
    };
    stack.run(&mut model, Box::new(cmd));
    assert!((model.sections[0].area - 9024.0).abs() < 1.0);

    stack.undo(&mut model);
    assert!((model.sections[0].area - area_h).abs() < 1.0);

    stack.redo(&mut model);
    assert!((model.sections[0].area - 9024.0).abs() < 1.0);
}

#[test]
fn test_duplicate_section_for_member_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "H-300".into());
    model.sections.push(sec);

    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let cmd = DuplicateSectionForMember { member: ElemId(0) };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 2);
    assert_eq!(model.elements[0].section, Some(SectionId(1)));

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.elements[0].section, Some(SectionId(0)));

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 2);
    assert_eq!(model.elements[0].section, Some(SectionId(1)));
}

#[test]
fn test_edit_section_shape_invalid_id_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelBox {
        height: 200.0,
        width: 200.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let cmd = EditSectionShape {
        section: SectionId(99),
        new_shape: shape,
    };
    stack.run(&mut model, Box::new(cmd));
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.sections.is_empty());
}

#[test]
fn test_delete_add_section_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "H-300".into());
    model.sections.push(sec);

    let cmd = DeleteSection { id: SectionId(0) };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 0);

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 0);
}

#[test]
fn test_duplicate_section_no_section_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let cmd = DuplicateSectionForMember { member: ElemId(0) };
    stack.run(&mut model, Box::new(cmd));
    assert!(stack.can_undo());
    stack.undo(&mut model);
}

/// 2 節点 + 部材 2 本のモデル（部材削除・再採番テスト用）。
fn two_member_model() -> Model {
    let mut model = empty_model();
    for (i, x) in [0.0, 1000.0, 2000.0].iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [*x, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    for i in 0..2u32 {
        model.elements.push(ElementData {
            id: ElemId(i),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(i), NodeId(i + 1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model
}

#[test]
fn test_delete_member_middle_renumbers_and_roundtrips() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mut model = two_member_model();
    // 両方の部材に部材荷重を付ける
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![
            MemberLoad {
                elem: ElemId(0),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Point { a: 500.0, p: 1.0 },
            },
            MemberLoad {
                elem: ElemId(1),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Point { a: 500.0, p: 2.0 },
            },
        ],
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 先頭（中間）の部材を削除 → 後続 ID が繰り上がり、関連荷重も消える
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    assert_eq!(model.elements[0].id, ElemId(0));
    assert!(model.validate().is_ok());
    assert_eq!(model.load_cases[0].member.len(), 1);
    // 残った荷重は旧 ElemId(1) → 新 ElemId(0) を指す
    assert_eq!(model.load_cases[0].member[0].elem, ElemId(0));

    // undo で完全復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());
}

#[test]
fn test_delete_section_in_use_is_noop_and_renumbers() {
    use squid_n_core::model::Section;
    let mut model = two_member_model();
    for i in 0..2u32 {
        model.sections.push(Section {
            id: SectionId(i),
            name: format!("S{}", i),
            area: 100.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 10.0,
            width: 10.0,
            as_y: 80.0,
            as_z: 80.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
    }
    // 部材 0 に断面 1 を割当（断面 0 は未使用）
    model.elements[0].section = Some(SectionId(1));
    let mut stack = UndoStack::new();

    // 使用中の断面 1 は削除できない
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) }));
    assert_eq!(model.sections.len(), 2);

    // 未使用の断面 0 は削除でき、断面 1 → 0 に繰り上がり参照も追随
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));
    assert_eq!(model.elements[0].section, Some(SectionId(0)));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

/// 小梁（JoistLine.section）が参照する断面は削除ガードで守られ、別断面の削除では
/// 参照が繰り上がって追随する（床設計用の断面参照が陳腐化しない。レビュー指摘）。
#[test]
fn test_delete_section_referenced_by_joist() {
    use squid_n_core::model::{DistributionMethod, JoistLine, Node, Section};
    let mut model = empty_model();
    for i in 0..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    for i in 0..2u32 {
        model.sections.push(Section {
            id: SectionId(i),
            name: format!("S{}", i),
            area: 100.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 10.0,
            width: 10.0,
            as_y: 80.0,
            as_z: 80.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
    }
    // 小梁が断面 1 のみを参照するスラブ（要素は断面を参照しない）。
    model.slabs.push(squid_n_core::model::Slab {
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![JoistLine {
            dir: [1.0, 0.0],
            spacing: 900.0,
            support: [NodeId(0), NodeId(1)],
            section: Some(SectionId(1)),
            pinned_onto: None,
        }],
        loads: vec![],
        method: DistributionMethod::TriTrapezoid,
        kind: Default::default(),
        one_way: None,
        edge_supported: None,
        usage: None,
        thickness: None,
    });
    let mut stack = UndoStack::new();

    // 小梁が参照中の断面 1 は削除できない（要素だけでなく小梁も参照チェック対象）。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) }));
    assert_eq!(model.sections.len(), 2, "小梁参照中の断面は削除されない");

    // 未参照の断面 0 を削除 → 断面 1 が 0 へ繰り上がり、小梁参照も追随する。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(
        model.slabs[0].joists[0].section,
        Some(SectionId(0)),
        "小梁の断面参照が繰り上がりに追随"
    );
    assert!(model.validate().is_ok());
}

#[test]
fn test_add_delete_material_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            fc: None,
            fy: Some(235.0),
        }),
    );
    assert_eq!(model.materials.len(), 1);
    assert_eq!(model.materials[0].id, MaterialId(0));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert_eq!(model.materials.len(), 0);
    stack.redo(&mut model);
    assert_eq!(model.materials.len(), 1);
}

#[test]
fn test_delete_material_in_use_is_noop() {
    let mut model = two_member_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            fc: None,
            fy: Some(235.0),
        }),
    );
    model.elements[0].material = Some(MaterialId(0));
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(model.materials.len(), 1, "使用中の材料は削除できない");
}

#[test]
fn test_delete_material_middle_renumbers() {
    let mut model = two_member_model();
    let mut stack = UndoStack::new();
    for name in ["A", "B"] {
        stack.run(
            &mut model,
            Box::new(AddMaterial {
                name: name.into(),
                young: 1.0,
                poisson: 0.3,
                density: 0.0,
                fc: None,
                fy: None,
            }),
        );
    }
    model.elements[0].material = Some(MaterialId(1));
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(model.materials.len(), 1);
    assert_eq!(model.materials[0].name, "B");
    assert_eq!(model.elements[0].material, Some(MaterialId(0)));
    assert!(model.validate().is_ok());
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

#[test]
fn test_set_material_field_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "Fc21".into(),
            young: 21500.0,
            poisson: 0.2,
            density: 2.3e-9,
            fc: Some(21.0),
            fy: None,
        }),
    );
    stack.run(
        &mut model,
        Box::new(SetMaterialField {
            id: MaterialId(0),
            field: MaterialField::Fc,
            value: Some(24.0),
        }),
    );
    assert_eq!(model.materials[0].fc, Some(24.0));
    stack.undo(&mut model);
    assert_eq!(model.materials[0].fc, Some(21.0));
}

#[test]
fn test_add_delete_load_case_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    stack.run(&mut model, Box::new(AddLoadCase { name: "LL".into() }));
    assert_eq!(model.load_cases.len(), 2);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[1].id, LoadCaseId(1));

    // 先頭を削除 → 後続 ID 繰り上がり
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[0].name, "LL");

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

#[test]
fn test_delete_load_case_referenced_by_combo_is_noop() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    model.combinations.push(LoadCombination {
        name: "combo".into(),
        terms: vec![(LoadCaseId(0), 1.0)],
    });
    stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
    assert_eq!(model.load_cases.len(), 1, "組合せ参照中のケースは削除不可");
}

#[test]
fn test_add_delete_combination_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    model.combinations.push(LoadCombination {
        name: "既存".into(),
        terms: vec![(LoadCaseId(0), 1.0)],
    });
    let mut stack = UndoStack::new();

    let combo = LoadCombination {
        name: "1.0DL+1.0LL".into(),
        terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
    };
    stack.run(
        &mut model,
        Box::new(AddCombination {
            combo: combo.clone(),
        }),
    );
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[1], combo);

    stack.undo(&mut model);
    assert_eq!(model.combinations.len(), 1);

    stack.redo(&mut model);
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[1], combo);
}

#[test]
fn test_delete_combination_roundtrip_restores_position() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    for (name, coef) in [("A", 1.0), ("B", 2.0), ("C", 3.0)] {
        model.combinations.push(LoadCombination {
            name: name.into(),
            terms: vec![(LoadCaseId(0), coef)],
        });
    }
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 中間（B）を削除
    stack.run(&mut model, Box::new(DeleteCombination { index: 1 }));
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[0].name, "A");
    assert_eq!(model.combinations[1].name, "C");

    // undo で元の位置（index 1）に復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.combinations[1].name, "B");

    stack.redo(&mut model);
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[0].name, "A");
    assert_eq!(model.combinations[1].name, "C");
}

#[test]
fn test_delete_combination_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteCombination { index: 0 }));
    assert!(model.combinations.is_empty());
    assert!(stack.can_undo());
    // Noop の undo でも状態は変わらない
    stack.undo(&mut model);
    assert!(model.combinations.is_empty());
}

#[test]
fn test_add_delete_slab_roundtrip() {
    use squid_n_core::model::DistributionMethod;
    let mut model = empty_model();
    for i in 0..4 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            usage: None,
        }),
    );
    assert_eq!(model.slabs.len(), 1);
    assert_eq!(model.slabs[0].id, SlabId(0));
    assert!(model.validate().is_ok());

    // 採番の確認：2 枚目は SlabId(1)
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::OneWay,
            usage: None,
        }),
    );
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].id, SlabId(1));

    stack.undo(&mut model);
    assert_eq!(model.slabs.len(), 1);
    assert_eq!(model.slabs[0].id, SlabId(0));

    stack.redo(&mut model);
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].id, SlabId(1));
}

#[test]
fn test_delete_slab_middle_renumbers_and_roundtrips() {
    use squid_n_core::model::{AreaLoad, DistributionMethod};
    let mut model = empty_model();
    for i in 0..3 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    let mut stack = UndoStack::new();
    for (i, kind) in ["A", "B", "C"].iter().enumerate() {
        stack.run(
            &mut model,
            Box::new(AddSlab {
                boundary: vec![NodeId(i as u32)],
                joists: vec![],
                loads: vec![AreaLoad {
                    kind: kind.to_string(),
                    value: 1.0,
                }],
                method: DistributionMethod::TributaryArea,
                usage: None,
            }),
        );
    }
    assert_eq!(model.slabs.len(), 3);
    let before = model.clone();

    // 中間（SlabId(1) = "B"）を削除 → 後続 ID が繰り上がる
    stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(1) }));
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[0].id, SlabId(0));
    assert_eq!(model.slabs[0].loads[0].kind, "A");
    assert_eq!(model.slabs[1].id, SlabId(1));
    assert_eq!(model.slabs[1].loads[0].kind, "C");
    assert!(model.validate().is_ok());

    // undo で完全復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].loads[0].kind, "C");
}

#[test]
fn test_delete_slab_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(0) }));
    assert!(model.slabs.is_empty());
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.slabs.is_empty());
}

fn make_story(id: u32, weight: Option<f64>) -> squid_n_core::model::Story {
    squid_n_core::model::Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: StoryId(id),
        name: format!("{}F", id + 1),
        elevation: id as f64 * 3000.0,
        node_ids: vec![],
        diaphragms: vec![],
        seismic_weight: weight,
    }
}

#[test]
fn test_set_story_weight_roundtrip() {
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(0),
            weight: Some(1234.5),
        }),
    );
    assert_eq!(model.stories[0].seismic_weight, Some(1234.5));

    stack.undo(&mut model);
    assert_eq!(model.stories[0].seismic_weight, None);

    stack.redo(&mut model);
    assert_eq!(model.stories[0].seismic_weight, Some(1234.5));
}

#[test]
fn test_set_story_weight_invalid_id_is_noop() {
    let mut model = empty_model();
    model.stories.push(make_story(0, Some(999.0)));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(99),
            weight: Some(1.0),
        }),
    );
    assert_eq!(model.stories[0].seismic_weight, Some(999.0));
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert_eq!(model.stories[0].seismic_weight, Some(999.0));
}

/// `ApplyStories` が剛床代表節点(`rep_nodes`/`generated_masters`)込みで適用され、
/// undo で節点数・`generated_masters`・stories・constraints が完全に元へ戻ること
/// （`eq_ignoring_dofmap` で比較）。redo も確認する。
#[test]
fn test_apply_stories_roundtrip_with_generated_masters() {
    use squid_n_core::dof::Dof;
    use squid_n_core::model::{Constraint, DiaphragmDef, Story};

    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    let before = model.clone();
    let mut stack = UndoStack::new();

    let mut rep_restraint = Dof6Mask::FREE;
    rep_restraint.set_fixed(Dof::Uz);
    rep_restraint.set_fixed(Dof::Rx);
    rep_restraint.set_fixed(Dof::Ry);
    let rep_node = Node {
        id: NodeId(2),
        coord: [500.0, 0.0, 3000.0],
        restraint: rep_restraint,
        mass: None,
        story: Some(StoryId(0)),
    };
    let cmd = ApplyStories {
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".into(),
            elevation: 3000.0,
            node_ids: vec![NodeId(0), NodeId(1)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(2),
                slaves: vec![NodeId(0), NodeId(1)],
                rigid: true,
            }],
            seismic_weight: Some(1000.0),
        }],
        node_story: vec![Some(StoryId(0)), Some(StoryId(0))],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(2),
            slaves: vec![NodeId(0), NodeId(1)],
        }],
        rep_nodes: vec![rep_node],
        generated_masters: vec![NodeId(2)],
    };

    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.generated_masters, vec![NodeId(2)]);
    assert_eq!(model.stories.len(), 1);
    assert_eq!(model.constraints.len(), 1);
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.generated_masters.is_empty());
    assert!(model.stories.is_empty());
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.generated_masters, vec![NodeId(2)]);
    assert_eq!(model.stories.len(), 1);
    assert_eq!(model.constraints.len(), 1);
}

/// 階数が減って不活性化された剛床代表節点（`generated_masters` には残るが
/// `restraint=FIXED`/`story=None`）の `DeleteNode` → undo(`InsertNode`) で
/// `generated_masters` が（ID 繰り上げ込みで）正しく維持されること。
#[test]
fn test_delete_leftover_generated_master_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    });
    // 不活性化された旧代表節点(story_gen.rs の仕様どおり restraint=FIXED, story=None)。
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [3000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FIXED,
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
    model.generated_masters = vec![NodeId(1)];
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 不活性節点は何にも参照されていないため削除できる。
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(1) }));
    assert_eq!(model.nodes.len(), 2);
    assert!(model.generated_masters.is_empty());
    // 旧 NodeId(2) だった部材参照は NodeId(1) に繰り上がる
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.generated_masters, vec![NodeId(1)]);
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 2);
    assert!(model.generated_masters.is_empty());
}

#[test]
fn test_set_load_case_kind_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCaseKind;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Other);

    stack.run(
        &mut model,
        Box::new(SetLoadCaseKind {
            id: LoadCaseId(0),
            kind: LoadCaseKind::Dead,
        }),
    );
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);

    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Other);

    stack.redo(&mut model);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);
}

#[test]
fn test_set_load_case_kind_invalid_id_is_noop() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCaseKind;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetLoadCaseKind {
            id: LoadCaseId(0),
            kind: LoadCaseKind::Dead,
        }),
    );
    assert!(model.load_cases.is_empty());
}

#[test]
fn test_sync_slab_loads_to_case_creates_new_case() {
    use squid_n_core::ids::{ElemId, LoadCaseId};
    use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind, NodalLoad};
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let member = vec![MemberLoad {
        elem: ElemId(0),
        dir: [0.0, 0.0, -1.0],
        kind: MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 1.0,
            w2: 1.0,
        },
    }];
    let nodal = vec![NodalLoad {
        node: NodeId(0),
        values: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
    }];

    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: nodal.clone(),
            member: member.clone(),
        }),
    );
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[0].name, "床荷重(自動)");
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);
    assert_eq!(model.load_cases[0].member, member);
    assert_eq!(model.load_cases[0].nodal, nodal);

    // undo → 新規作成したケースごと消える(DeleteLoadCase を再利用した逆操作)。
    stack.undo(&mut model);
    assert!(model.load_cases.is_empty());

    stack.redo(&mut model);
    assert_eq!(model.load_cases.len(), 1);
}

#[test]
fn test_sync_slab_loads_to_case_replaces_existing_case() {
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind};
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    // 既存の同名ケース(手動でユーザーが編集した中身を想定)。
    stack.run(
        &mut model,
        Box::new(AddLoadCase {
            name: "床荷重(自動)".into(),
        }),
    );
    stack.run(
        &mut model,
        Box::new(AddMemberLoad {
            lc: LoadCaseId(0),
            load: MemberLoad {
                elem: ElemId(0),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 500.0,
                    w1: 9.0,
                    w2: 9.0,
                },
            },
        }),
    );
    let before = model.clone();
    assert_eq!(model.load_cases[0].member.len(), 1);

    let new_member = vec![MemberLoad {
        elem: ElemId(1),
        dir: [0.0, 0.0, -1.0],
        kind: MemberLoadKind::Distributed {
            a: 0.0,
            b: 2000.0,
            w1: 3.0,
            w2: 3.0,
        },
    }];
    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: Vec::new(),
            member: new_member.clone(),
        }),
    );
    // 全置換: 個数は増えず(重複せず)、内容が新しい値に入れ替わる。
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].member, new_member);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);

    // 再同期しても個数は変わらない(全置換なので重複しない)。
    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: Vec::new(),
            member: new_member.clone(),
        }),
    );
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].member, new_member);

    // undo を2回 → 元の手動入力内容に戻る。
    stack.undo(&mut model);
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

#[test]
fn test_set_load_cfg_roundtrip() {
    use squid_n_core::model::LoadCfg;
    let mut model = empty_model();
    assert!(model.load_cfg.is_none());
    let mut stack = UndoStack::new();

    let cfg = LoadCfg {
        steel_weight_factor: 1.05,
        ..Default::default()
    };
    stack.run(
        &mut model,
        Box::new(SetLoadCfg {
            cfg: Some(cfg.clone()),
        }),
    );
    assert_eq!(model.load_cfg, Some(cfg));

    stack.undo(&mut model);
    assert!(model.load_cfg.is_none());

    stack.redo(&mut model);
    assert_eq!(model.load_cfg.as_ref().unwrap().steel_weight_factor, 1.05);
}

#[test]
fn test_set_wall_attr_add_replace_and_remove_roundtrip() {
    use squid_n_core::model::WallAttr;
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let attr1 = WallAttr {
        elem: ElemId(0),
        opening_area: 100.0,
        opening_weight: 50.0,
        three_side_slit: false,
        openings: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetWallAttr {
            attr: attr1.clone(),
        }),
    );
    assert_eq!(model.wall_attrs, vec![attr1.clone()]);

    // 既存エントリを置換
    let attr2 = WallAttr {
        elem: ElemId(0),
        opening_area: 200.0,
        opening_weight: 80.0,
        three_side_slit: true,
        openings: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetWallAttr {
            attr: attr2.clone(),
        }),
    );
    assert_eq!(model.wall_attrs, vec![attr2.clone()]);

    stack.undo(&mut model);
    assert_eq!(model.wall_attrs, vec![attr1.clone()]);

    // 削除
    stack.run(&mut model, Box::new(RemoveWallAttr { elem: ElemId(0) }));
    assert!(model.wall_attrs.is_empty());

    stack.undo(&mut model);
    assert_eq!(model.wall_attrs, vec![attr1]);
}

#[test]
fn test_remove_wall_attr_missing_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(RemoveWallAttr { elem: ElemId(0) }));
    assert!(model.wall_attrs.is_empty());
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.wall_attrs.is_empty());
}

fn sample_misc_wall(weight: f64) -> squid_n_core::model::MiscWall {
    squid_n_core::model::MiscWall {
        start: [0.0, 0.0, 0.0],
        end: [3000.0, 0.0, 0.0],
        height: 3000.0,
        weight_per_area: weight,
        transfer: Default::default(),
        thickness: None,
    }
}

#[test]
fn test_add_delete_misc_wall_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(AddMiscWall {
            wall: sample_misc_wall(1.0),
        }),
    );
    stack.run(
        &mut model,
        Box::new(AddMiscWall {
            wall: sample_misc_wall(2.0),
        }),
    );
    assert_eq!(model.misc_walls.len(), 2);

    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteMiscWall { index: 0 }));
    assert_eq!(model.misc_walls.len(), 1);
    assert_eq!(model.misc_walls[0].weight_per_area, 2.0);

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));

    stack.redo(&mut model);
    assert_eq!(model.misc_walls.len(), 1);
}

#[test]
fn test_set_misc_wall_roundtrip() {
    let mut model = empty_model();
    model.misc_walls.push(sample_misc_wall(1.0));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMiscWall {
            index: 0,
            wall: sample_misc_wall(9.0),
        }),
    );
    assert_eq!(model.misc_walls[0].weight_per_area, 9.0);

    stack.undo(&mut model);
    assert_eq!(model.misc_walls[0].weight_per_area, 1.0);
}

#[test]
fn test_set_misc_wall_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetMiscWall {
            index: 0,
            wall: sample_misc_wall(1.0),
        }),
    );
    assert!(model.misc_walls.is_empty());
}

#[test]
fn test_set_story_structure_roundtrip() {
    use squid_n_core::model::StoryStructure;
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    assert_eq!(model.stories[0].structure, StoryStructure::Rc);
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryStructure {
            story: StoryId(0),
            structure: StoryStructure::S,
        }),
    );
    assert_eq!(model.stories[0].structure, StoryStructure::S);

    stack.undo(&mut model);
    assert_eq!(model.stories[0].structure, StoryStructure::Rc);

    stack.redo(&mut model);
    assert_eq!(model.stories[0].structure, StoryStructure::S);
}

#[test]
fn test_set_story_level_kind_roundtrip() {
    use squid_n_core::model::StoryLevelKind;
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryLevelKind {
            story: StoryId(0),
            level_kind: StoryLevelKind::Penthouse { k: 0.5 },
        }),
    );
    assert_eq!(
        model.stories[0].level_kind,
        StoryLevelKind::Penthouse { k: 0.5 }
    );

    stack.undo(&mut model);
    assert_eq!(model.stories[0].level_kind, StoryLevelKind::Normal);
}

#[test]
fn test_set_story_structure_invalid_id_is_noop() {
    use squid_n_core::model::StoryStructure;
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetStoryStructure {
            story: StoryId(99),
            structure: StoryStructure::S,
        }),
    );
    assert_eq!(model.stories[0].structure, StoryStructure::Rc);
}

#[test]
fn test_set_slab_kind_and_one_way_roundtrip() {
    use squid_n_core::model::{DistributionMethod, OneWayDir, SlabKind};
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            usage: None,
        }),
    );
    assert_eq!(model.slabs[0].kind, SlabKind::Interior);
    assert_eq!(model.slabs[0].one_way, None);

    stack.run(
        &mut model,
        Box::new(SetSlabKind {
            id: SlabId(0),
            kind: SlabKind::Cantilever,
        }),
    );
    assert_eq!(model.slabs[0].kind, SlabKind::Cantilever);
    stack.undo(&mut model);
    assert_eq!(model.slabs[0].kind, SlabKind::Interior);

    stack.run(
        &mut model,
        Box::new(SetSlabOneWay {
            id: SlabId(0),
            one_way: Some(OneWayDir::X),
        }),
    );
    assert_eq!(model.slabs[0].one_way, Some(OneWayDir::X));
    stack.undo(&mut model);
    assert_eq!(model.slabs[0].one_way, None);
}

#[test]
fn test_set_slab_joists_roundtrip() {
    use squid_n_core::model::{DistributionMethod, JoistLine};
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            usage: None,
        }),
    );
    assert!(model.slabs[0].joists.is_empty());

    let joists = vec![JoistLine {
        dir: [0.0, 1.0],
        spacing: 900.0,
        support: [NodeId(0), NodeId(3)],
        section: None,
        pinned_onto: None,
    }];
    stack.run(
        &mut model,
        Box::new(SetSlabJoists {
            id: SlabId(0),
            joists: joists.clone(),
        }),
    );
    assert_eq!(model.slabs[0].joists, joists);

    // undo で元の空 joists に戻る（対称逆操作）。
    stack.undo(&mut model);
    assert!(model.slabs[0].joists.is_empty());
    stack.redo(&mut model);
    assert_eq!(model.slabs[0].joists, joists);

    // 存在しない SlabId は Noop（モデル不変・undo スタックも安全）。
    stack.run(
        &mut model,
        Box::new(SetSlabJoists {
            id: SlabId(99),
            joists: vec![],
        }),
    );
    assert_eq!(model.slabs[0].joists, joists);
}

#[test]
fn test_materialize_slab_joists_creates_beams() {
    use squid_n_core::model::{DistributionMethod, ElementKind, EndCondition, JoistLine};
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![JoistLine {
                dir: [0.0, 1.0],
                spacing: 900.0,
                support: [NodeId(0), NodeId(3)],
                section: None,
                pinned_onto: None,
            }],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            usage: None,
        }),
    );
    let before = model.elements.len();

    stack.run(
        &mut model,
        Box::new(MaterializeSlabJoists { slab: SlabId(0) }),
    );
    assert_eq!(model.elements.len(), before + 1, "小梁1本が実部材化される");
    let beam = model.elements.last().unwrap();
    assert_eq!(beam.kind, ElementKind::Beam);
    assert_eq!(beam.nodes.len(), 2);
    assert!(
        (beam.nodes[0] == NodeId(0) && beam.nodes[1] == NodeId(3))
            || (beam.nodes[0] == NodeId(3) && beam.nodes[1] == NodeId(0)),
        "支持2節点を両端に持つ"
    );
    assert_eq!(
        beam.end_cond,
        [EndCondition::Pinned, EndCondition::Pinned],
        "小梁は両端ピン"
    );

    // 再実行しても既存の実部材があるので新規生成しない（冪等）。
    stack.run(
        &mut model,
        Box::new(MaterializeSlabJoists { slab: SlabId(0) }),
    );
    assert_eq!(
        model.elements.len(),
        before + 1,
        "実部材化済みは重複生成しない"
    );

    // undo で生成した実部材が末尾から除去される。
    stack.undo(&mut model); // 2回目（冪等 no-op）を戻す
    assert_eq!(model.elements.len(), before + 1);
    stack.undo(&mut model); // 1回目の生成を戻す
    assert_eq!(model.elements.len(), before, "undo で実部材化を取り消す");
}

#[test]
fn test_set_multi_opening_mode_roundtrip() {
    use squid_n_core::model::MultiOpeningMode;
    let mut model = empty_model();
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Envelope,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Auto,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Auto);

    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);
    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);

    stack.redo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);
    stack.redo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Auto);
}

/// 同じモードへの再設定でも既存の値置換系コマンドと同様に処理される
/// （同値判定による分岐なし。undo すれば必ず変更前の値へ戻る）。
#[test]
fn test_set_multi_opening_mode_same_value_is_symmetric() {
    use squid_n_core::model::MultiOpeningMode;
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Equivalent,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);

    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);
}

#[test]
fn test_set_member_hysteresis_roundtrip() {
    use squid_n_core::model::HysteresisModel;
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0; 3],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
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
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetMemberHysteresis {
            elem: ElemId(0),
            rule: HysteresisModel::Takeda,
        }),
    );
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );
    stack.undo(&mut model);
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
    stack.redo(&mut model);
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );

    // 存在しない部材は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(SetMemberHysteresis {
            elem: ElemId(99),
            rule: HysteresisModel::Standard,
        }),
    );
    assert_eq!(model.member_hysteresis(ElemId(99)), None);
}

#[test]
fn test_add_damper_creates_element_and_attr_roundtrip() {
    use squid_n_core::model::{DamperKind, DamperProps};
    let mut model = two_member_model();
    let before = model.clone();
    let new_id = ElemId(model.elements.len() as u32);
    let props = DamperProps {
        kind: DamperKind::Maxwell,
        kd: 120_000.0,
        c0: 2_000.0,
        alpha: 0.5,
        ..Default::default()
    };
    let elem = ElementData {
        id: new_id,
        kind: ElementKind::Damper,
        nodes: smallvec![NodeId(0), NodeId(2)],
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
    };
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddDamper { elem, props }));
    // 要素と特性が原子的に追加される。
    assert_eq!(model.elements.len(), 3);
    assert_eq!(model.elements[2].kind, ElementKind::Damper);
    assert_eq!(model.damper_props(new_id), Some(props));

    // undo で要素・特性ともに消える（完全復元）。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.damper_props(new_id), None);

    // redo で再生成。
    stack.redo(&mut model);
    assert_eq!(model.damper_props(new_id), Some(props));
}

#[test]
fn test_set_damper_props_roundtrip() {
    use squid_n_core::model::DamperProps;
    let mut model = two_member_model();
    let e = ElemId(1);
    let p1 = DamperProps {
        kd: 100.0,
        c0: 10.0,
        alpha: 1.0,
        ..Default::default()
    };
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: e,
            props: Some(p1),
        }),
    );
    assert_eq!(model.damper_props(e), Some(p1));
    // 解除。
    stack.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: e,
            props: None,
        }),
    );
    assert_eq!(model.damper_props(e), None);
    stack.undo(&mut model);
    assert_eq!(model.damper_props(e), Some(p1));
    stack.undo(&mut model);
    assert_eq!(model.damper_props(e), None);

    // 存在しない部材は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: ElemId(99),
            props: Some(p1),
        }),
    );
    assert_eq!(model.damper_props(ElemId(99)), None);
}

#[test]
fn test_set_member_detail_attr_add_replace_and_remove_roundtrip() {
    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let attr1 = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetMemberDetailAttr {
            attr: attr1.clone(),
        }),
    );
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    stack.undo(&mut model);
    assert!(model.member_detail_attrs.is_empty());

    stack.redo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    // 既存エントリを置換
    let attr2 = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: None,
        haunch_j: Some(Haunch {
            length: 500.0,
            depth_increase: 150.0,
            width_increase: 0.0,
        }),
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Shop,
        }],
    };
    stack.run(
        &mut model,
        Box::new(SetMemberDetailAttr {
            attr: attr2.clone(),
        }),
    );
    assert_eq!(model.member_detail_attrs, vec![attr2.clone()]);

    stack.undo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    // 削除
    stack.run(
        &mut model,
        Box::new(RemoveMemberDetailAttr { elem: ElemId(0) }),
    );
    assert!(model.member_detail_attrs.is_empty());

    stack.undo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1]);
}

#[test]
fn test_remove_member_detail_attr_missing_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(RemoveMemberDetailAttr { elem: ElemId(0) }),
    );
    assert!(model.member_detail_attrs.is_empty());
    assert!(stack.can_undo());
    stack.undo(&mut model);
    assert!(model.member_detail_attrs.is_empty());
}

#[test]
fn test_delete_member_shifts_and_restores_side_table_attrs() {
    use squid_n_core::model::{DamperProps, HysteresisModel};
    let mut model = two_member_model();
    // 部材0に履歴則、部材1にダンパー特性を付与。
    model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
    let props = DamperProps {
        kd: 90_000.0,
        c0: 1_500.0,
        alpha: 0.4,
        ..Default::default()
    };
    model.set_damper_props(ElemId(1), Some(props));
    let before = model.clone();

    let mut stack = UndoStack::new();
    // 部材0を削除 → 部材1が ElemId(0) へ繰り上がり、その側テーブル参照も追従する。
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    // 削除された部材0の履歴則は消える。
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
    // 元・部材1のダンパー特性は新 ElemId(0) を指す（参照整合）。
    assert_eq!(model.damper_props(ElemId(0)), Some(props));
    assert!(model.validate().is_ok());

    // undo で側テーブル属性も含め完全復元。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );
    assert_eq!(model.damper_props(ElemId(1)), Some(props));

    // redo で再削除・再整合。
    stack.redo(&mut model);
    assert_eq!(model.damper_props(ElemId(0)), Some(props));
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
}

/// `DeleteMember` が `member_detail_attrs`（ハンチ・継手位置）も
/// `take_elem_attrs`/`restore_elem_attrs` 経由で退避・復元すること
/// （`ElemAttrs.detail` の配線の検証）。
#[test]
fn test_delete_member_restores_member_detail_attr() {
    use squid_n_core::model::{Haunch, MemberDetailAttr};
    let mut model = two_member_model();
    // 部材0にハンチ付帯情報を付与。
    let attr = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![],
    };
    model.member_detail_attrs.push(attr.clone());
    let before = model.clone();

    let mut stack = UndoStack::new();
    // 部材0を削除 → 付帯情報も連動して消える。
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    assert!(model.member_detail(ElemId(0)).is_none());
    assert!(model.validate().is_ok());

    // undo で付帯情報も含め完全復元。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.member_detail(ElemId(0)), Some(&attr));

    // redo で再削除。
    stack.redo(&mut model);
    assert!(model.member_detail(ElemId(0)).is_none());
}
