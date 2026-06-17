use faer::sparse::SparseColMat;
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::LoadCaseId;
use sc_core::model::Model;
use sc_element::factory::build_behavior;
use sc_math::sparse::assemble_csc;

pub fn assemble_global_k(model: &Model, dofmap: &DofMap) -> SparseColMat<usize, f64> {
    let ctx = sc_element::behavior::Ctx { model };
    let mut all_triplets = Vec::new();

    for elem in &model.elements {
        let (behavior, state) = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(dofmap);
        let k_local = behavior.tangent_stiffness(&state, &ctx);
        let triplets = k_local.to_triplets(&gdofs);
        all_triplets.extend(triplets);
    }

    assemble_csc(dofmap.n_active(), all_triplets)
}

pub fn assemble_global_m(
    model: &Model,
    dofmap: &DofMap,
    opt: sc_element::behavior::MassOption,
) -> SparseColMat<usize, f64> {
    let mut all_triplets = Vec::new();
    for elem in &model.elements {
        let (behavior, _state) = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(dofmap);
        let m_local = behavior.mass_matrix(opt);
        let triplets = m_local.to_triplets(&gdofs);
        all_triplets.extend(triplets);
    }
    assemble_csc(dofmap.n_active(), all_triplets)
}

pub fn assemble_global_f(model: &Model, dofmap: &DofMap, lc: LoadCaseId) -> Vec<f64> {
    let n_active = dofmap.n_active();
    let mut f = vec![0.0; n_active];

    // Find the load case
    if let Some(lc_data) = model.load_cases.iter().find(|l| l.id == lc) {
        for nodal_load in &lc_data.nodal {
            let ni = nodal_load.node.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    f[active as usize] += nodal_load.values[d];
                }
            }
        }
    }

    f
}
