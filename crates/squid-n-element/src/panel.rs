use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};

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
                // dc はパネルの柱せい方向寸法（鉛直材＝柱の depth）、
                // db は梁せい方向寸法（水平材＝梁の depth）。evaluate() では
                // dc/2 を梁フェイス距離、db/2 を柱フェイス距離として使うため、
                // 水平材→db / 鉛直材→dc の対応を取り違えないこと。
                if is_horizontal {
                    if sec.depth > db {
                        db = sec.depth;
                    }
                } else if sec.depth > dc {
                    dc = sec.depth;
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

    /// PanelZone::new のモデルからの寸法推定: dc=柱（鉛直材）せい、db=梁（水平材）せい。
    /// evaluate() は dc/2 を梁フェイス距離・db/2 を柱フェイス距離として使うため、
    /// 水平材→db / 鉛直材→dc の対応が入れ替わると全結果が誤る（回帰テスト）。
    #[test]
    fn test_panel_zone_new_assigns_column_depth_to_dc() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, MaterialId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        };

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let make_sec = |id: u32, depth: f64| squid_n_core::model::Section {
            id: SectionId(id),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth,
            width: depth,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: Some(12.0),
            thickness: None,
            shape: None,
        };
        let make_elem = |id: u32, n0: u32, n1: u32, sec: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };

        let model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 3000.0]),    // 接合部中心
                make_node(1, [5000.0, 0.0, 3000.0]), // 梁の先端
                make_node(2, [0.0, 0.0, 0.0]),       // 柱脚
            ],
            sections: vec![make_sec(0, 500.0), make_sec(1, 700.0)], // 0: 梁, 1: 柱
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: String::new(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            elements: vec![
                make_elem(0, 0, 1, 0), // 水平材（梁, せい500）
                make_elem(1, 2, 0, 1), // 鉛直材（柱, せい700）
            ],
            ..Default::default()
        };

        let panel_data = ElementData {
            id: ElemId(2),
            kind: ElementKind::PanelZone,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };

        let pz = PanelZone::new(&panel_data, &model);
        assert!((pz.dc - 700.0).abs() < 1e-9, "dc は柱せい: {}", pz.dc);
        assert!((pz.db - 500.0).abs() < 1e-9, "db は梁せい: {}", pz.db);
    }

    /// 添付資料『パネルゾーンの力学』(小野瀬, 2009) ケース1 の数値例照合（仕様 §11.5）。
    /// 単位系は資料に合わせ kN, m, kN·m。確定値:
    ///   pQc = 851.135 kN, pQb = 1702.273 kN, τc = τb（整合）。
    /// 入力（資料 図18 のM図より）:
    ///   柱せい方向 dc=0.2m, 梁せい方向 db=0.4m。
    ///   梁: ML*b=218.182, MR*b=181.818 kNm, bQL=bQR=72.727 kN。
    ///   柱: ML*c=150, MU*c=250 kNm, cQL=100, cQU=125 kN。
    #[test]
    fn test_panel_zone_reference_case1() {
        let dc = 0.2_f64; // 柱せい [m]
        let db = 0.4_f64; // 梁せい [m]
        let tp = 1.0_f64; // 板厚（pQ には無関係。τ 整合確認用）
        let pz = PanelZone {
            dc,
            db,
            tp,
            g: 0.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        };
        let conn = PanelConnection {
            ml_b: 218.182,
            mr_b: 181.818,
            bql: 72.727,
            bqr: 72.727,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 150.0,
            mu_c: 250.0,
            cql: 100.0,
            cqu: 125.0,
        };
        let res = pz.evaluate(&conn);

        // フェースモーメント（資料: bML=210.909, bMR=174.545, cML=130.0, cMU=225.0）
        assert!((res.b_ml - 210.909).abs() < 1e-3, "bML={}", res.b_ml);
        assert!((res.b_mr - 174.545).abs() < 1e-3, "bMR={}", res.b_mr);
        assert!((res.c_ml - 130.0).abs() < 1e-9, "cML={}", res.c_ml);
        assert!((res.c_mu - 225.0).abs() < 1e-9, "cMU={}", res.c_mu);

        // パネルせん断（資料の確定値）
        assert!(
            (res.pqc - 851.135).abs() < 0.05,
            "pQc={} (期待 851.135)",
            res.pqc
        );
        assert!(
            (res.pqb - 1702.273).abs() < 0.05,
            "pQb={} (期待 1702.273)",
            res.pqb
        );

        // 整合条件 τc = τb（= pQc/(dc·tp) = pQb/(db·tp)、資料 "c=b o.k"）
        let tau_b = res.pqb / (db * tp);
        assert!(
            (res.tau - tau_b).abs() / res.tau.abs() < 1e-4,
            "τc={} != τb={}",
            res.tau,
            tau_b
        );
    }

    /// 添付資料 ケース2（ト型＝梁が片側のみ）。欠落部材の項を 0 として同一式で
    /// 計算できることの数値照合（仕様 §7.3 / §7.5 / 資料 図23）。
    /// 確定値: pQc=854.168 kN, pQb=1708.334 kN。
    /// 入力: ML*b=400, MR*b=0（欠落）, bQL=133.333, bQR=0;
    ///       ML*c=150, MU*c=250, cQL=100, cQU=125。
    #[test]
    fn test_panel_zone_reference_case2_t_joint() {
        let (dc, db, tp) = (0.2_f64, 0.4_f64, 1.0_f64);
        let pz = PanelZone {
            dc,
            db,
            tp,
            g: 0.0,
            kind: PanelStiffnessModel::RigidZoneApprox,
            center_node: NodeId(0),
            connected_nodes: vec![NodeId(1), NodeId(2), NodeId(3)],
        };
        let conn = PanelConnection {
            ml_b: 400.0,
            mr_b: 0.0, // 欠落部材 → 0
            bql: 133.333,
            bqr: 0.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 150.0,
            mu_c: 250.0,
            cql: 100.0,
            cqu: 125.0,
        };
        let res = pz.evaluate(&conn);
        assert!(
            (res.pqc - 854.168).abs() < 0.05,
            "pQc={} (期待 854.168)",
            res.pqc
        );
        assert!(
            (res.pqb - 1708.334).abs() < 0.05,
            "pQb={} (期待 1708.334)",
            res.pqb
        );
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
        let model = squid_n_core::model::Model::default();
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
        let model = squid_n_core::model::Model::default();
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
