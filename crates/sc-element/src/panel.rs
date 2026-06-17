use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::NodeId;
use sc_core::model::{ElementData, Model};
use smallvec::SmallVec;

pub enum PanelStiffnessModel {
    RigidZoneApprox,
    ElasticShearPanel,
}

pub struct PanelZone {
    pub dc: f64,
    pub db: f64,
    pub tp: f64,
    pub g: f64,
    pub kind: PanelStiffnessModel,
    pub center_node: NodeId,
    pub connected_nodes: Vec<NodeId>,
}

pub struct PanelConnection {
    pub ml_b: f64,
    pub mr_b: f64,
    pub bql: f64,
    pub bqr: f64,
    pub bnl: f64,
    pub bnr: f64,
    pub ml_c: f64,
    pub mu_c: f64,
    pub cql: f64,
    pub cqu: f64,
}

pub struct PanelResult {
    pub b_ml: f64,
    pub b_mr: f64,
    pub c_ml: f64,
    pub c_mu: f64,
    pub pqc: f64,
    pub pqb: f64,
    pub tau: f64,
}

impl PanelZone {
    pub fn new(data: &ElementData, model: &Model) -> Self {
        let center = data.nodes[0];
        let connected: Vec<NodeId> = data.nodes.iter().skip(1).copied().collect();

        let mut dc = 0.0;
        let mut db = 0.0;
        let mut tp = 0.0;
        let mut g = 0.0;

        // 接合部中心を含む梁・柱をモデルから探し、寸法を推定
        for elem in &model.elements {
            if elem.nodes.len() < 2 || !elem.nodes.contains(&center) {
                continue;
            }
            let p0 = model.nodes[elem.nodes[0].index()].coord;
            let p1 = model.nodes[elem.nodes[1].index()].coord;
            let dx = p1[0] - p0[0];
            let dy = p1[1] - p0[1];
            let dz = p1[2] - p0[2];
            let l = (dx * dx + dy * dy + dz * dz).sqrt();
            if l < 1e-12 {
                continue;
            }
            let axis = [dx / l, dy / l, dz / l];
            let is_horizontal = axis[2].abs() < 0.707;

            if let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) {
                if is_horizontal {
                    if sec.depth > dc {
                        dc = sec.depth;
                    }
                } else if sec.depth > db {
                    db = sec.depth;
                }
                if tp == 0.0 {
                    tp = sec.panel_thickness.unwrap_or(0.0);
                }
            }
            if let Some(mat) = elem
                .material
                .and_then(|mid| model.materials.get(mid.index()))
            {
                if g == 0.0 {
                    g = mat.shear_modulus();
                }
            }
        }

        Self {
            dc,
            db,
            tp,
            g,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: center,
            connected_nodes: connected,
        }
    }

    pub fn evaluate(&self, conn: &PanelConnection) -> PanelResult {
        let dc2 = self.dc / 2.0;
        let db2 = self.db / 2.0;

        let b_ml = conn.ml_b - conn.bql * dc2;
        let b_mr = conn.mr_b - conn.bqr * dc2;
        let c_ml = conn.ml_c - conn.cql * db2;
        let c_mu = conn.mu_c - conn.cqu * db2;

        let pqc = ((b_ml + b_mr) - (conn.cql + conn.cqu) * db2) / self.db;
        let pqb = ((c_mu + c_ml) - (conn.bql + conn.bqr) * dc2) / self.dc;
        let tau = if self.tp > 0.0 {
            pqc / (self.dc * self.tp)
        } else {
            0.0
        };
        PanelResult {
            b_ml,
            b_mr,
            c_ml,
            c_mu,
            pqc,
            pqb,
            tau,
        }
    }
}

impl ElementBehavior for PanelZone {
    fn n_dof(&self) -> usize {
        self.connected_nodes.len() * DOF_PER_NODE + DOF_PER_NODE
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in std::iter::once(&self.center_node).chain(self.connected_nodes.iter()) {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        match self.kind {
            PanelStiffnessModel::RigidZoneApprox => LocalMat::zeros(self.n_dof()),
            PanelStiffnessModel::ElasticShearPanel => {
                let kp = self.g * self.tp * self.dc;
                let n = self.n_dof();
                let mut k = LocalMat::zeros(n);
                if kp > 0.0 && n >= 2 {
                    k.set(0, 0, kp);
                    k.set(1, 1, kp);
                }
                k
            }
        }
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        LocalVec {
            data: SmallVec::from_elem(0.0, self.n_dof()),
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(self.n_dof())
    }
}
