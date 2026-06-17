use sc_core::dof::DofMap;
use sc_core::model::Model;
use smallvec::SmallVec;

pub struct LocalMat {
    pub n: usize,
    pub data: Vec<f64>,
}

pub struct LocalVec {
    pub data: SmallVec<[f64; 24]>,
}

pub struct Ctx<'a> {
    pub model: &'a Model,
}

#[derive(Clone, Debug, Default)]
pub struct ElemState {}

pub enum MassOption {
    Lumped,
    Consistent,
}

impl LocalMat {
    pub fn zeros(n: usize) -> Self {
        Self {
            n,
            data: vec![0.0; n * n],
        }
    }

    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i * self.n + j]
    }

    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        self.data[i * self.n + j] = v;
    }

    pub fn to_triplets(&self, gdofs: &[usize]) -> Vec<sc_math::sparse::Triplet> {
        let mut out = Vec::with_capacity(self.n * self.n);
        for i in 0..self.n {
            let gi = gdofs[i];
            if gi == usize::MAX {
                continue;
            }
            for (j, &gj) in gdofs.iter().enumerate().take(self.n) {
                if gj == usize::MAX {
                    continue;
                }
                let v = self.get(i, j);
                if v != 0.0 {
                    out.push(sc_math::sparse::Triplet {
                        row: gi,
                        col: gj,
                        val: v,
                    });
                }
            }
        }
        out
    }
}

pub trait ElementBehavior {
    fn n_dof(&self) -> usize;
    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]>;
    fn tangent_stiffness(&self, state: &ElemState, ctx: &Ctx) -> LocalMat;
    fn internal_force(&self, state: &ElemState, ctx: &Ctx) -> LocalVec;
    fn update_state(&mut self, _du: &LocalVec, _commit: bool, _ctx: &Ctx) {}
    fn mass_matrix(&self, opt: MassOption) -> LocalMat;
    fn recover_forces(&self, _u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        None
    }
}
