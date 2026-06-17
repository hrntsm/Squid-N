use crate::behavior::{ElemState, LocalMat, LocalVec, MassOption};
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::NodeId;
use sc_core::model::Model;
use smallvec::SmallVec;

pub struct MsElement {
    pub nodes: [NodeId; 2],
    pub n_springs: usize,
    pub spring_areas: Vec<f64>,
    pub spring_coords: Vec<f64>,
    pub e: f64,
    pub length: f64,
}

impl MsElement {
    pub fn new(data: &sc_core::model::ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = model.nodes[n0.index()].coord;
        let p1 = model.nodes[n1.index()].coord;
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let n_springs = 10;
        let spring_areas = vec![data.section.map(|_| 10.0).unwrap_or(0.0); n_springs];
        let half = (n_springs - 1) as f64 / 2.0;
        let spring_coords: Vec<f64> = (0..n_springs).map(|i| (i as f64 - half) / half).collect();

        MsElement {
            nodes: [n0, n1],
            n_springs,
            spring_areas,
            spring_coords,
            e: model.materials.first().map(|m| m.young).unwrap_or(0.0),
            length: len,
        }
    }
}

impl crate::behavior::ElementBehavior for MsElement {
    fn n_dof(&self) -> usize {
        12
    }
    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }
    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &crate::behavior::Ctx) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let ka = self.e * self.spring_areas.iter().sum::<f64>() / self.length;
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }
    fn internal_force(&self, _state: &ElemState, _ctx: &crate::behavior::Ctx) -> LocalVec {
        LocalVec {
            data: smallvec::smallvec![0.0; 12],
        }
    }
    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }
}
