use faer::sparse::SparseColMat;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::Model;
use squid_n_element::factory::build_behavior;
use squid_n_math::sparse::{assemble_csc, Triplet};

pub fn assemble_global_k(model: &Model, dofmap: &DofMap) -> SparseColMat<usize, f64> {
    let ctx = squid_n_element::behavior::Ctx { model };
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
    opt: squid_n_element::behavior::MassOption,
) -> SparseColMat<usize, f64> {
    let mut all_triplets = Vec::new();
    for elem in &model.elements {
        let (behavior, _state) = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(dofmap);
        let m_local = behavior.mass_matrix(opt);
        let triplets = m_local.to_triplets(&gdofs);
        all_triplets.extend(triplets);
    }

    // 節点集中質量（Node.mass）を対角へ加算する。
    // 床荷重→質量化した層質量や、集中質量モデルの質量はここで反映される。
    // これを欠くと固有値・有効質量比（P2 DoD #2）が物理的に誤る。
    for (ni, node) in model.nodes.iter().enumerate() {
        if let Some(mass) = node.mass {
            for (d, &mval) in mass.iter().enumerate() {
                if mval == 0.0 {
                    continue;
                }
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    all_triplets.push(Triplet {
                        row: active as usize,
                        col: active as usize,
                        val: mval,
                    });
                }
            }
        }
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

        // 部材（梁）荷重 → 等価節点力（consistent load vector）を全体系へ加算。
        add_member_loads(model, dofmap, &lc_data.member, &mut f);
    }

    f
}

/// 部材荷重の等価節点力を local で計算し、全体系へ回して荷重ベクトルへ散布する。
fn add_member_loads(
    model: &Model,
    dofmap: &DofMap,
    member_loads: &[squid_n_core::model::MemberLoad],
    f: &mut [f64],
) {
    use squid_n_element::transform::LocalFrame;

    for elem in &model.elements {
        // この部材に作用する荷重だけ収集
        let loads: Vec<squid_n_core::model::MemberLoad> = member_loads
            .iter()
            .filter(|ml| ml.elem == elem.id)
            .cloned()
            .collect();
        if loads.is_empty() || elem.nodes.len() < 2 {
            continue;
        }
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        if ni >= model.nodes.len() || nj >= model.nodes.len() {
            continue;
        }
        let p_i = model.nodes[ni].coord;
        let p_j = model.nodes[nj].coord;
        let length = {
            let dx = p_j[0] - p_i[0];
            let dy = p_j[1] - p_i[1];
            let dz = p_j[2] - p_i[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        if length < 1e-9 {
            continue;
        }
        let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
        let q_local = squid_n_element::member_load::consistent_load_local(&loads, &frame, length);
        let q_global = frame.rotate_to_global(&q_local);
        // q_global: [i:0..6, j:6..12] を各節点 DOF へ散布
        for (local_node, &node_idx) in [ni, nj].iter().enumerate() {
            for d in 0..DOF_PER_NODE {
                let g = node_idx * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    f[active as usize] += q_global[local_node * DOF_PER_NODE + d];
                }
            }
        }
    }
}
