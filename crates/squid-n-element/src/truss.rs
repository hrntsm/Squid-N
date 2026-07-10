use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, Material, Model, Section};

/// 一般ブレース要素（RESP-D マニュアル計算編02「剛性計算」§一般ブレースの剛性）。
///
/// 剛性 KB = factor·E·A/L（L: 芯々間の長さ、A: 降伏部の断面積）。
/// 軸剛性のみを持ち、曲げ・せん断・ねじりはゼロ（トラス要素）。
/// `factor` は弾性解析における引張専用ブレースの低減（1/2）と、通常ブレース・
/// 弾塑性解析の初期剛性（1倍）を切り替えるために生成時に指定する
/// （マニュアル「引張専用ブレースは引張と圧縮が対で存在するとみなし、弾性解析では
/// 剛性を1/2にモデル化する。ただし、弾塑性解析の場合は初期剛性は1倍とする」）。
#[derive(Clone)]
pub struct TrussElement {
    pub id: ElemId,
    pub e: f64,
    /// 軸剛性用断面積（降伏部の断面積）。
    pub a: f64,
    /// 質量算定用の断面積（既定は `a` と同じ。将来 SRC 等価換算が必要になれば分離）。
    pub a_mass: f64,
    /// 剛性倍率（引張専用ブレースの弾性解析: 0.5、それ以外: 1.0）。
    pub factor: f64,
    pub length: f64,
    pub density: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    /// 確定変位（線形要素の内力計算用。グローバル座標系で蓄積）。
    pub committed_disp: [f64; 12],
}

fn get_section(model: &Model, sid: Option<squid_n_core::ids::SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section {
        id: squid_n_core::ids::SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    })
}

fn get_material(model: &Model, mid: Option<squid_n_core::ids::MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        id: squid_n_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    })
}

impl TrussElement {
    /// `factor`: 弾性剛性倍率（引張専用の弾性解析なら 0.5、それ以外は 1.0）。
    pub fn new(data: &ElementData, model: &Model, factor: f64) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = if n0.index() < model.nodes.len() {
            model.nodes[n0.index()].coord
        } else {
            [0.0; 3]
        };
        let p1 = if n1.index() < model.nodes.len() {
            model.nodes[n1.index()].coord
        } else {
            [0.0; 3]
        };
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, data.material);

        Self {
            id: data.id,
            e: mat.young,
            a: sec.area,
            a_mass: sec.area,
            factor,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            committed_disp: [0.0; 12],
        }
    }

    /// 局所座標系での 12×12 剛性行列。軸方向（ux, ux_j）成分のみ非ゼロ。
    /// k = factor·E·A/L（RESP-D マニュアル計算編02「一般ブレースの剛性」KB = E·A/L）。
    pub fn local_stiffness(&self) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        if self.length < 1e-12 {
            return k;
        }
        let ka = self.factor * self.e * self.a / self.length;
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }
}

impl ElementBehavior for TrussElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
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
        // ElementBehavior::tangent_stiffness は全体系を返す契約（beam.rs 参照）。
        // 部材軸方向ベクトル t による K = k·(t·tᵀ) 展開は、ローカル軸剛性を
        // 回転行列で全体系へ回すことと等価（t = axis.rot[0]）。
        self.axis.to_global(&self.local_stiffness())
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let k = self.axis.to_global(&self.local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.committed_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        if commit {
            for i in 0..12 {
                self.committed_disp[i] += du.data[i];
            }
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
            }
            MassOption::Consistent => {
                // 軸方向のみ整合質量（m/6·[2,1;1,2]）。並進の他成分（uy, uz）は
                // Lumped と同等（節点集中）で足りる（beam.rs のブロック分割方針を踏襲）。
                let c1 = m / 6.0;
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                for d in [1, 2, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        let u_local = self.axis.rotate_to_local(&arr);
        let k_local = self.local_stiffness();
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }
        // 軸力 N（引張正）のみ。他要素の慣習（beam.rs）に合わせ i 端側は -f_local[0]。
        let n = -f_local[0];
        Some(crate::beam::MemberForces {
            at: vec![
                (0.0, [n, 0.0, 0.0, 0.0, 0.0, 0.0]),
                (1.0, [n, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, RigidZone,
    };

    fn make_model(p0: [f64; 3], p1: [f64; 3]) -> (Model, ElementData) {
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: p0,
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: p1,
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "brace".to_string(),
                area: 2000.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Brace {
                tension_only: false,
            },
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Pinned, EndCondition::Pinned],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
        };
        (model, data)
    }

    #[test]
    fn test_axial_local_stiffness_matches_ea_over_l() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let truss = TrussElement::new(&data, &model, 1.0);
        let k = truss.local_stiffness();
        let ea_l = truss.e * truss.a / truss.length;
        assert!((k.get(0, 0) - ea_l).abs() < 1e-9);
        assert!((k.get(6, 6) - ea_l).abs() < 1e-9);
        assert!((k.get(0, 6) + ea_l).abs() < 1e-9);
        assert!((k.get(6, 0) + ea_l).abs() < 1e-9);
    }

    /// 斜め配置でも全体系剛性が軸方向ベクトル t による t·tᵀ 展開に一致すること
    /// （K_global = k·(t·tᵀ) をブロックごとに検証。t = 部材軸方向単位ベクトル）。
    #[test]
    fn test_global_stiffness_matches_t_tt_projection() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [3000.0, 0.0, 4000.0]);
        let truss = TrussElement::new(&data, &model, 1.0);
        let ctx = Ctx { model: &model };
        let k_global = truss.tangent_stiffness(&ElemState::default(), &ctx);

        let l = truss.length;
        let t = [3000.0 / l, 0.0, 4000.0 / l];
        let k = truss.e * truss.a / l;

        // ii ブロック（0..3, 0..3）= k·t·tᵀ
        for i in 0..3 {
            for j in 0..3 {
                let expected = k * t[i] * t[j];
                assert!(
                    (k_global.get(i, j) - expected).abs() < 1e-6,
                    "K[{i}][{j}]: {} vs {}",
                    k_global.get(i, j),
                    expected
                );
                // jj ブロック（6..9, 6..9）も同じ
                assert!((k_global.get(i + 6, j + 6) - expected).abs() < 1e-6);
                // ij ブロック（0..3, 6..9）は符号反転
                assert!((k_global.get(i, j + 6) + expected).abs() < 1e-6);
            }
        }
        // 回転・せん断自由度はゼロ
        for i in 3..6 {
            for j in 0..12 {
                assert_eq!(k_global.get(i, j), 0.0);
                assert_eq!(k_global.get(j, i), 0.0);
            }
        }
    }

    #[test]
    fn test_tension_only_elastic_half_stiffness() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let normal = TrussElement::new(&data, &model, 1.0);
        let tension_only_elastic = TrussElement::new(&data, &model, 0.5);
        let k_normal = normal.local_stiffness();
        let k_half = tension_only_elastic.local_stiffness();
        assert!((k_half.get(0, 0) - 0.5 * k_normal.get(0, 0)).abs() < 1e-9);
    }

    #[test]
    fn test_stiffness_matrix_symmetric() {
        let (model, data) = make_model([1000.0, 500.0, 0.0], [5000.0, 2500.0, 3000.0]);
        let truss = TrussElement::new(&data, &model, 1.0);
        let ctx = Ctx { model: &model };
        let k = truss.tangent_stiffness(&ElemState::default(), &ctx);
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                    "K[{i}][{j}] != K[{j}][{i}]"
                );
            }
        }
    }

    /// 剛体移動（両節点を同一量だけ並進）を与えると内力がゼロになること。
    #[test]
    fn test_rigid_body_translation_gives_zero_force() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [3000.0, 4000.0, 0.0]);
        let mut truss = TrussElement::new(&data, &model, 1.0);
        // 両端に同一の並進変位を与える（剛体移動）
        let du = LocalVec {
            data: SmallVec::from_vec(vec![
                1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0,
            ]),
        };
        let ctx = Ctx { model: &model };
        truss.update_state(&du, true, &ctx);
        let f = truss.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            assert!(f.data[i].abs() < 1e-6, "f[{i}]={}", f.data[i]);
        }
    }
}
