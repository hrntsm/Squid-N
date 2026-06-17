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

        // Recover forces for each element
        let mut member_forces = Vec::new();
        let _ctx = sc_element::behavior::Ctx { model };
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let mut u_elem = [0.0_f64; 12];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() && k < 12 {
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
    use sc_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
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
        // u = F L / (E A) = 1000 * 1000 / (1000 * 100) = 10
        assert!(
            (result.disp[1][0] - 10.0).abs() < 1e-6,
            "ux={}",
            result.disp[1][0]
        );
        assert!(result.member_forces.len() == 1);
        let forces = &result.member_forces[0].1;
        // 部材力: i端反力 ≈ -1000
        let fx_i = forces.at[0].1[0];
        assert!((fx_i + 1000.0).abs() < 1e-6, "fx_i={}", fx_i);
    }
}
