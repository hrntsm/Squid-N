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

#[cfg(test)]
mod tests {
    use super::*;

    /// パネルゾーンの検証:
    /// 整合条件 pqc·db = pqb·dc は、節点のモーメント釣り合い ml_b + mr_b = ml_c + mu_c
    /// が成立するとき自動的に満たされる（添付資料『パネルゾーンの力学』式(4)）。
    /// 釣り合いを満たす入力を与え、正しい pQc/pQb/τ が計算されることを確認する。
    #[test]
    fn test_panel_zone_equilibrium_consistency() {
        let dc = 500.0;
        let db = 800.0;
        let tp = 19.0;
        let pz = PanelZone {
            dc,
            db,
            tp,
            g: 80_000.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        };

        // 節点モーメント釣り合い: ml_b + mr_b = ml_c + mu_c
        let ml_b = 500_000.0;
        let mr_b = 300_000.0;
        let ml_c = 400_000.0;
        let mu_c = 400_000.0; // 500+300 = 400+400 = 800 ✓

        let conn = PanelConnection {
            ml_b,
            mr_b,
            bql: 150.0,
            bqr: 100.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c,
            mu_c,
            cql: 120.0,
            cqu: 130.0,
        };

        let res = pz.evaluate(&conn);

        // 整合条件: pqc·db = pqb·dc
        let lhs = res.pqc * db;
        let rhs = res.pqb * dc;
        assert!(
            (lhs - rhs).abs() < 1e-9,
            "pqc*db ({}) != pqb*dc ({})",
            lhs,
            rhs
        );

        // τ = pqc / (dc·tp)
        assert!(
            (res.tau - res.pqc / (dc * tp)).abs() < 1e-12,
            "tau mismatch"
        );

        // フェースモーメントが正しく計算されていること
        let dc2 = dc / 2.0;
        let db2 = db / 2.0;
        assert!((res.b_ml - (ml_b - conn.bql * dc2)).abs() < 1e-9);
        assert!((res.b_mr - (mr_b - conn.bqr * dc2)).abs() < 1e-9);
        assert!((res.c_ml - (ml_c - conn.cql * db2)).abs() < 1e-9);
        assert!((res.c_mu - (mu_c - conn.cqu * db2)).abs() < 1e-9);
    }

    /// 方式A（剛域近似）は tangent_stiffness がゼロを返す（二重計上防止）
    #[test]
    fn test_panel_rigid_zone_approx_zero_stiffness() {
        let pz = PanelZone {
            dc: 500.0,
            db: 700.0,
            tp: 12.0,
            g: 80_000.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        };
        let model = sc_core::model::Model::default();
        let k = pz.tangent_stiffness(&ElemState {}, &Ctx { model: &model });
        for v in k.data.as_slice() {
            assert_eq!(*v, 0.0, "RigidZoneApprox must return zero stiffness");
        }
    }

    /// 方式B（弾性せん断パネル）は非ゼロ剛性を返す
    #[test]
    fn test_panel_elastic_shear_panel_nonzero_stiffness() {
        let pz = PanelZone {
            dc: 500.0,
            db: 700.0,
            tp: 12.0,
            g: 80_000.0,
            kind: PanelStiffnessModel::ElasticShearPanel,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1)],
        };
        let model = sc_core::model::Model::default();
        let k = pz.tangent_stiffness(&ElemState {}, &Ctx { model: &model });
        // Kp = G*tp*dc 相当の剛性が出ていること
        let kp_expected = pz.g * pz.tp * pz.dc;
        assert!(k.data.as_slice().iter().any(|&v| v > 0.0));
        // 対角成分が Kp と一致
        assert!((k.data[0] - kp_expected).abs() < 1e-6);
    }

    /// L型接合部（右梁欠落）のテスト: 欠落部材の項を 0 に
    #[test]
    fn test_panel_l_shape() {
        let pz = PanelZone {
            dc: 500.0,
            db: 700.0,
            tp: 12.0,
            g: 80_000.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2)],
        };
        // L型: 右梁(mr_b, bqr, bnr)と上柱(mu_c, cqu)が欠落 → すべて0
        // 節点モーメント釣り合い条件: ml_b + 0 = ml_c + 0 → ml_b = ml_c
        let conn = PanelConnection {
            ml_b: 300_000.0,
            mr_b: 0.0,
            bql: 100.0,
            bqr: 0.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 300_000.0, // ml_b = ml_c に設定
            mu_c: 0.0,
            cql: 80.0,
            cqu: 0.0,
        };
        let res = pz.evaluate(&conn);
        assert!(
            res.pqc.is_finite(),
            "pqc should be finite for L-shape joint"
        );
        assert!(
            res.pqb.is_finite(),
            "pqb should be finite for L-shape joint"
        );
        // 整合条件（釣り合い条件 ml_b + 0 = ml_c + 0 より成立）
        let lhs = res.pqc * pz.db;
        let rhs = res.pqb * pz.dc;
        assert!(
            (lhs - rhs).abs() < 1e-9,
            "pqc*db ({}) != pqb*dc ({}) for L-shape",
            lhs,
            rhs
        );
    }

    /// ト型接合部（下柱欠落）のテスト
    #[test]
    fn test_panel_t_shape() {
        let pz = PanelZone {
            dc: 500.0,
            db: 700.0,
            tp: 12.0,
            g: 80_000.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3)],
        };
        // ト型: 下柱(ml_c, cql)が欠落 → 0
        // 釣り合い条件: ml_b + mr_b = 0 + mu_c
        let conn = PanelConnection {
            ml_b: 200_000.0,
            mr_b: 100_000.0, // 200+100 = 300
            bql: 80.0,
            bqr: 60.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 0.0,
            mu_c: 300_000.0, // = 300
            cql: 0.0,
            cqu: 90.0,
        };
        let res = pz.evaluate(&conn);
        assert!(
            res.pqc.is_finite(),
            "pqc should be finite for T-shape joint"
        );
        let lhs = res.pqc * pz.db;
        let rhs = res.pqb * pz.dc;
        assert!(
            (lhs - rhs).abs() < 1e-9,
            "pqc*db ({}) != pqb*dc ({}) for T-shape",
            lhs,
            rhs
        );
    }

    /// 十字型接合部（全方向あり）の対称ケース
    #[test]
    fn test_panel_cross_symmetric() {
        let pz = PanelZone {
            dc: 600.0,
            db: 800.0,
            tp: 22.0,
            g: 80_000.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        };
        // 左右対称 + 上下対称 → 釣り合い成立
        let conn = PanelConnection {
            ml_b: 450_000.0,
            mr_b: 450_000.0,
            bql: 120.0,
            bqr: 120.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 500_000.0,
            mu_c: 400_000.0, // 450+450 = 500+400 = 900 ✓
            cql: 100.0,
            cqu: 100.0,
        };
        let res = pz.evaluate(&conn);
        assert!(res.pqc.is_finite());
        let lhs = res.pqc * pz.db;
        let rhs = res.pqb * pz.dc;
        assert!(
            (lhs - rhs).abs() < 1e-9,
            "pqc*db ({}) != pqb*dc ({}) for cross-shape",
            lhs,
            rhs
        );
    }
}
