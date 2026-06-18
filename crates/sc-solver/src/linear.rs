use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use sc_core::dof::DofMap;
use sc_core::ids::LoadCaseId;
use sc_core::model::Model;
use sc_element::beam::MemberForces;
use sc_element::factory::build_behavior;
use sc_math::solver::{make_solver, SolveError, SolverBackend};

pub struct StaticOnce {
    pub disp: Vec<[f64; 6]>,
    pub member_forces: Vec<(sc_core::ids::ElemId, MemberForces)>,
}

pub fn linear_static_once(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        });
    }

    let k_free = assemble_global_k(model, &dofmap);
    let f_free = assemble_global_f(model, &dofmap, lc);

    let reducer = Reducer::build(model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let f_red = reducer.reduce_f(&f_free);
    let n_indep = reducer.n_indep;

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    if n_indep > 0 {
        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; model.nodes.len()];
        for ni in 0..model.nodes.len() {
            for d in 0..sc_core::dof::DOF_PER_NODE {
                let g = ni * sc_core::dof::DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    let val = u_free[active as usize];
                    match d {
                        0 => disp[ni][0] = val,
                        1 => disp[ni][1] = val,
                        2 => disp[ni][2] = val,
                        3 => disp[ni][3] = val,
                        4 => disp[ni][4] = val,
                        _ => disp[ni][5] = val,
                    }
                }
            }
        }

        let mut member_forces = Vec::new();
        let _ctx = sc_element::behavior::Ctx { model };
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let n_gdofs = gdofs.len();
            let mut u_elem = vec![0.0; n_gdofs];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }

            if let Some(forces) = behavior.recover_forces(&u_elem) {
                member_forces.push((elem.id, forces));
            }
        }

        Ok(StaticOnce {
            disp,
            member_forces,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::dof::Dof6Mask;
    use sc_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
    use sc_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
        NodalLoad, Node, Section,
    };

    fn make_axial_cantilever() -> Model {
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
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".to_string(),
                area: 100.0,
                iy: 1000.0,
                iz: 1000.0,
                j: 100.0,
                depth: 100.0,
                width: 100.0,
                as_y: 83.33,
                as_z: 83.33,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".to_string(),
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "axial".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }],
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_linear_static_axial_cantilever() {
        let model = make_axial_cantilever();
        let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
        assert!(
            (result.disp[1][0] - 10.0).abs() < 1e-6,
            "ux={}",
            result.disp[1][0]
        );
        assert!(result.member_forces.len() == 1);
        let forces = &result.member_forces[0].1;
        let fx_i = forces.at[0].1[0];
        assert!((fx_i + 1000.0).abs() < 1e-6, "fx_i={}", fx_i);
    }

    /// X 軸上の片持ち梁に「グローバル Y 方向」の先端荷重をかける。
    /// 参照ベクトル [0,0,1] では local z = global −y となるので、たわみは
    /// **iy** で決まる（iz ではない）。to_global を欠くと iz を使ってしまい誤る。
    /// よって iy≠iz の断面で、δ = PL³/(3E·iy) に一致することを確認する。
    #[test]
    fn test_beam_to_global_transverse_uses_correct_inertia() {
        // 現実的な鋼材大断面（iz=1e9 級）を用いる：to_global 修正の検証に加え、
        // 端ばね静縮約のペナルティが大断面でも非正定値化しないこと（堅牢性）も同時に確認。
        let e = 205000.0_f64;
        let l = 1000.0_f64; // make_axial_cantilever の節点間距離
        let iy = 2.0e9_f64;
        let iz = 1.0e9_f64; // iy≠iz：取り違えが顕在化する
        let p = 10000.0_f64;
        let mut model = make_axial_cantilever();
        model.materials[0].young = e;
        model.sections[0].iy = iy;
        model.sections[0].iz = iz;
        model.sections[0].as_y = 1.0e9; // せん断たわみを十分小さく
        model.sections[0].as_z = 1.0e9;
        model.load_cases[0].nodal[0].values = [0.0, p, 0.0, 0.0, 0.0, 0.0];

        let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
        let uy = result.disp[1][1];
        let expected = p * l.powi(3) / (3.0 * e * iy); // 曲げ支配（iy 使用）
        let buggy = p * l.powi(3) / (3.0 * e * iz); // 誤った値=iz 使用（2倍）
                                                    // iy ベースの値に一致し、iz ベース(2倍)を明確に排除する。
        assert!(
            (uy - expected).abs() / expected < 1e-3,
            "uy={} expected(iy)={} buggy(iz)={}",
            uy,
            expected,
            buggy
        );
    }

    /// 剛域がモデル→解析へ接続され、結果に効くことのエンドツーエンド確認。
    /// 同一片持ち梁で、基部に大きな剛域（可とう長を短縮）を入れると、
    /// 先端たわみが明確に小さく（剛く）なる。
    #[test]
    fn test_rigid_zone_affects_analysis() {
        let mut base = make_axial_cantilever();
        base.sections[0].iy = 1.0e7;
        base.sections[0].iz = 1.0e7;
        base.sections[0].as_y = 1.0e8;
        base.sections[0].as_z = 1.0e8;
        base.load_cases[0].nodal[0].values = [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0]; // global Z 載荷

        // 剛域なし
        let r0 = linear_static_once(&base, LoadCaseId(1)).unwrap();
        let uz0 = r0.disp[1][2];

        // 基部に剛域 λ_i=800（可とう長 200）
        let mut rigid = base.clone();
        rigid.elements[0].rigid_zone.length_i = 800.0;
        let r1 = linear_static_once(&rigid, LoadCaseId(1)).unwrap();
        let uz1 = r1.disp[1][2];

        assert!(
            uz0.abs() > 0.0 && uz1.abs() > 0.0,
            "uz0={} uz1={}",
            uz0,
            uz1
        );
        assert!(
            uz1.abs() < 0.5 * uz0.abs(),
            "剛域で剛くなるはず: uz_norigid={} uz_rigid={}",
            uz0,
            uz1
        );
    }

    #[test]
    fn test_linear_static_shell_element() {
        // Cantilever plate: bottom edge fixed (nodes 0,1), top edge free (nodes 2,3)
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
                    coord: [100.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [100.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(3),
                    coord: [0.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(1),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "shell".to_string(),
                area: 0.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 0.0,
                width: 0.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: Some(10.0),
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".to_string(),
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "shell_load".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(2),
                    values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                }],
            }],
            ..Default::default()
        };
        let result = linear_static_once(&model, LoadCaseId(1));
        assert!(result.is_ok(), "solver failed: {:?}", result.err());
        let result = result.unwrap();
        // Top edge should displace upward (positive z) under positive z point load
        assert!(
            result.disp[2][2] > 0.0,
            "loaded node should displace upward: {}",
            result.disp[2][2]
        );
        assert!(
            result.disp[3][2] > 0.0,
            "free node should also displace upward: {}",
            result.disp[3][2]
        );
    }

    #[test]
    fn test_linear_static_deterministic() {
        let model = make_axial_cantilever();
        let first = linear_static_once(&model, LoadCaseId(1)).unwrap();
        for _ in 0..99 {
            let cur = linear_static_once(&model, LoadCaseId(1)).unwrap();
            assert_eq!(first.disp, cur.disp);
            assert_eq!(first.member_forces.len(), cur.member_forces.len());
            for (a, b) in first.member_forces.iter().zip(cur.member_forces.iter()) {
                assert_eq!(a.0, b.0);
                assert_eq!(a.1.at, b.1.at);
            }
        }
    }

    #[test]
    fn test_shell_membrane_patch_test() {
        // Distorted 2x2 patch: corners pinned, midsides+interior free.
        // Sanity check that the patch assembles and solves without singularity.

        let e = 1000.0;
        let nu = 0.3;
        let t = 10.0;

        // 9 nodes: 4 corners, 4 midsides, 1 interior (offset from center)
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
                coord: [1000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [1000.0, 1000.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 1000.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(4),
                coord: [500.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(5),
                coord: [1000.0, 500.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(6),
                coord: [500.0, 1000.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(7),
                coord: [0.0, 500.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(8),
                coord: [450.0, 550.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ];

        // Apply boundary displacements as fixed restraints + prescribed displacements
        // We model this by making boundary nodes free and applying nodal loads that
        // produce the target displacements. Simpler: fix all boundary DOFs to zero and
        // apply the linear field as loads is non-trivial. Instead we directly set
        // boundary node displacements via MPC-like fixed values: set boundary nodes
        // to FIXED and then apply the corresponding displacement via load is not possible.
        //
        // Workaround: make boundary nodes free but apply large penalty springs to enforce
        // target displacements. This is complex.
        //
        // Alternative patch test: just verify the assembled element gives constant strain
        // when boundary nodes have linear displacements. We do this element-directly in
        // sc-element tests already. Here we only check that a free patch solves.
        //
        // For a meaningful solver test, pin the corners and leave midsides+interior free.
        // This is a simple sanity check that the patch does not become singular.

        let model = Model {
            nodes,
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Shell,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(4), NodeId(8), NodeId(7)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Shell,
                    nodes: smallvec::smallvec![NodeId(4), NodeId(1), NodeId(5), NodeId(8)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                },
                ElementData {
                    id: ElemId(2),
                    kind: ElementKind::Shell,
                    nodes: smallvec::smallvec![NodeId(8), NodeId(5), NodeId(2), NodeId(6)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                },
                ElementData {
                    id: ElemId(3),
                    kind: ElementKind::Shell,
                    nodes: smallvec::smallvec![NodeId(7), NodeId(8), NodeId(6), NodeId(3)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "shell".to_string(),
                area: 0.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 0.0,
                width: 0.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: Some(t),
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".to_string(),
                young: e,
                poisson: nu,
                density: 0.0,
                shear: None,
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "patch".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(8),
                    values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                }],
            }],
            ..Default::default()
        };

        let result = linear_static_once(&model, LoadCaseId(1));
        assert!(result.is_ok(), "patch solve failed: {:?}", result.err());
    }

    #[test]
    fn test_shell_membrane_off_no_diaphragm() {
        // Sanity: single shell element with membrane manually off, no diaphragm constraints.
        let mut model = Model {
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
                    coord: [100.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [100.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(3),
                    coord: [0.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "shell".to_string(),
                area: 0.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 0.0,
                width: 0.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: Some(10.0),
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".to_string(),
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "shell_load".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(2),
                    values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                }],
            }],
            ..Default::default()
        };
        // Put a rigid diaphragm in the story so ShellElement::new sets membrane_active=false,
        // but do NOT add a model.constraints entry, so the global DOFs remain free.
        use sc_core::model::{DiaphragmDef, Story};
        model.stories.push(Story {
            id: StoryId(0),
            name: "floor".to_string(),
            elevation: 0.0,
            node_ids: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            diaphragms: vec![DiaphragmDef {
                master: NodeId(0),
                slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
                rigid: true,
            }],
            seismic_weight: None,
        });
        let result = linear_static_once(&model, LoadCaseId(1));
        assert!(result.is_ok(), "solver failed: {:?}", result.err());
    }

    #[test]
    fn test_shell_rigid_floor_membrane_off() {
        // Rigid floor story: master node fully fixed, slaves follow master in-plane via
        // RigidDiaphragm constraint. Shell membrane is off for this story, but bending remains.
        use sc_core::model::{Constraint, DiaphragmDef, Story};

        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(1),
                    coord: [100.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(2),
                    coord: [100.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(3),
                    coord: [0.0, 100.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "shell".to_string(),
                area: 0.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 0.0,
                width: 0.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: Some(10.0),
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".to_string(),
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
            }],
            stories: vec![Story {
                id: StoryId(0),
                name: "floor".to_string(),
                elevation: 0.0,
                node_ids: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                diaphragms: vec![DiaphragmDef {
                    master: NodeId(0),
                    slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
                    rigid: true,
                }],
                seismic_weight: None,
            }],
            constraints: vec![Constraint::RigidDiaphragm {
                story: StoryId(0),
                master: NodeId(0),
                slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "load".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(2),
                    values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                }],
            }],
            ..Default::default()
        };

        let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
        // Slaves have no in-plane displacement because master is fixed and diaphragm constrains them.
        assert!(
            res.disp[1][0].abs() < 1e-12 && res.disp[1][1].abs() < 1e-12,
            "slave should not move in-plane: {:?}",
            [res.disp[1][0], res.disp[1][1]]
        );
        // Shell bending allows out-of-plane displacement under vertical load.
        assert!(
            res.disp[2][2].abs() > 1e-12,
            "shell should deflect vertically: {}",
            res.disp[2][2]
        );
    }
}
