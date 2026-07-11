use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, Material, Model, Section};
use std::any::Any;

/// 引張専用非線形モードにおける圧縮側（スラック）の剛性倍率。
/// ゼロにすると特異になり得るため、数値安定用に EA/L の 1e-6 倍を残す。
const NONLINEAR_COMPRESSION_FACTOR: f64 = 1e-6;

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
    /// 確定変位（線形モードの内力計算用。グローバル座標系で蓄積）。
    pub committed_disp: [f64; 12],
    /// 引張専用非線形モードフラグ（true: 弾塑性解析用。圧縮側は剛性を実質ゼロとする）。
    pub nonlinear: bool,
    /// 非線形モードの確定軸伸び（部材軸方向、引張正、mm）。
    committed_elongation: f64,
    /// 非線形モードの試行軸伸び（未確定、mm）。
    trial_elongation: f64,
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
        concrete_class: Default::default(),
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
            nonlinear: false,
            committed_elongation: 0.0,
            trial_elongation: 0.0,
        }
    }

    /// 引張専用ブレースの弾塑性解析用コンストラクタ（RESP-D マニュアル計算編02
    /// 「一般ブレースの剛性」）。弾塑性解析では初期剛性を1倍（factor=1.0）とし、
    /// 圧縮側では軸力を負担しない非線形挙動（真のスラック挙動）を持たせる。
    pub fn new_tension_only_nonlinear(data: &ElementData, model: &Model) -> Self {
        let mut elem = Self::new(data, model, 1.0);
        elem.nonlinear = true;
        elem
    }

    /// 局所座標系での 12×12 剛性行列。軸方向（ux, ux_j）成分のみ非ゼロ。
    /// k = factor·E·A/L（RESP-D マニュアル計算編02「一般ブレースの剛性」KB = E·A/L）。
    pub fn local_stiffness(&self) -> LocalMat {
        self.local_stiffness_with_factor(self.factor)
    }

    /// 剛性倍率を明示指定した局所剛性行列（非線形モードの現在剛性計算用）。
    fn local_stiffness_with_factor(&self, factor: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        if self.length < 1e-12 {
            return k;
        }
        let ka = factor * self.e * self.a / self.length;
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }

    /// 非線形モードにおける現在剛性倍率。伸び（引張正）がゼロ以上（初期状態
    /// 含む）なら EA/L（factor=1.0）、負（圧縮・スラック）なら数値安定用の
    /// 微小剛性とする。
    fn nonlinear_stiffness_factor(elongation: f64) -> f64 {
        // 伸びゼロ(初期状態)は初期剛性1倍(マニュアル「弾塑性解析の場合は
        // 初期剛性は1倍としてモデル化されます」)。圧縮に入った時点でスラック。
        if elongation >= 0.0 {
            1.0
        } else {
            NONLINEAR_COMPRESSION_FACTOR
        }
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
        if self.nonlinear {
            let factor = Self::nonlinear_stiffness_factor(self.trial_elongation);
            self.axis
                .to_global(&self.local_stiffness_with_factor(factor))
        } else {
            self.axis.to_global(&self.local_stiffness())
        }
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        if self.nonlinear {
            let mut f = LocalVec {
                data: SmallVec::from_elem(0.0, 12),
            };
            // 圧縮（伸び<=0、スラック）では軸力を負担しない。
            if self.trial_elongation > 0.0 && self.length >= 1e-12 {
                let n = self.e * self.a / self.length * self.trial_elongation;
                let t = self.axis.rot[0];
                for k in 0..3 {
                    f.data[k] = -n * t[k];
                    f.data[6 + k] = n * t[k];
                }
            }
            f
        } else {
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
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        if self.nonlinear {
            // 全体系 du を部材軸方向へ射影して軸伸びの増分を取り出す
            // （concentrated.rs の committed/trial パターンに準拠）。
            let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
            let du_local = self.axis.rotate_to_local(&du_global);
            let delong = du_local[6] - du_local[0];
            if commit {
                self.committed_elongation += delong;
                self.trial_elongation = self.committed_elongation;
            } else {
                self.trial_elongation = self.committed_elongation + delong;
            }
        } else if commit {
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

    /// 非線形モードの軸伸び（確定・試行）をスナップショット
    /// （concentrated.rs の committed/trial パターンに準拠）。
    fn snapshot_state(&self) -> Box<dyn Any> {
        Box::new((self.committed_elongation, self.trial_elongation))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some(&(committed, trial)) = state.downcast_ref::<(f64, f64)>() {
            self.committed_elongation = committed;
            self.trial_elongation = trial;
        }
    }

    fn commit_state(&mut self) {
        if self.nonlinear {
            self.committed_elongation = self.trial_elongation;
        }
    }

    fn revert_state(&mut self) {
        if self.nonlinear {
            self.trial_elongation = self.committed_elongation;
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct TrussCheckpoint {
            committed_elongation: f64,
            trial_elongation: f64,
        }
        let cp = TrussCheckpoint {
            committed_elongation: self.committed_elongation,
            trial_elongation: self.trial_elongation,
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(&mut self, data: &[u8]) {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct TrussCheckpoint {
            committed_elongation: f64,
            trial_elongation: f64,
        }
        if let Ok(cp) = bincode::deserialize::<TrussCheckpoint>(data) {
            self.committed_elongation = cp.committed_elongation;
            self.trial_elongation = cp.trial_elongation;
        }
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
                concrete_class: Default::default(),
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
            spring: None,
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

    /// j端の軸方向変位を与える du ベクトル（他自由度はゼロ）。
    fn axial_du(dj: f64) -> LocalVec {
        LocalVec {
            data: SmallVec::from_vec(vec![
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dj, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        }
    }

    /// 引張専用非線形モード: 引張側（伸び>0）では内力が EA/L×伸び に一致し、
    /// 接線剛性が EA/L に一致すること。
    #[test]
    fn test_tension_only_nonlinear_tension_side_matches_linear() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let mut truss = TrussElement::new_tension_only_nonlinear(&data, &model);
        let ctx = Ctx { model: &model };
        let ea_l = truss.e * truss.a / truss.length;

        let du = axial_du(1.0);
        truss.update_state(&du, true, &ctx);

        let k = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k.get(0, 0) - ea_l).abs() < 1e-6, "k00={}", k.get(0, 0));
        assert!((k.get(6, 6) - ea_l).abs() < 1e-6, "k66={}", k.get(6, 6));

        let f = truss.internal_force(&ElemState::default(), &ctx);
        let n = ea_l * 1.0;
        assert!((f.data[6] - n).abs() < 1e-6, "f[6]={}", f.data[6]);
        assert!((f.data[0] + n).abs() < 1e-6, "f[0]={}", f.data[0]);
    }

    /// 引張専用非線形モード: 圧縮側（伸び<=0、スラック）では内力がほぼゼロ、
    /// 接線剛性が数値安定用の微小値（EA/L×1e-6）まで低下すること。
    #[test]
    fn test_tension_only_nonlinear_compression_side_is_slack() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let mut truss = TrussElement::new_tension_only_nonlinear(&data, &model);
        let ctx = Ctx { model: &model };
        let ea_l = truss.e * truss.a / truss.length;

        let du = axial_du(-1.0);
        truss.update_state(&du, true, &ctx);

        let k = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!(
            (k.get(0, 0) - ea_l * NONLINEAR_COMPRESSION_FACTOR).abs() < 1e-9,
            "k00={}",
            k.get(0, 0)
        );

        let f = truss.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            assert!(f.data[i].abs() < 1e-9, "f[{i}]={}", f.data[i]);
        }
    }

    /// 引張→圧縮→引張のサイクルで commit/revert・snapshot/restore が正しく働くこと。
    #[test]
    fn test_tension_only_nonlinear_cycle_commit_revert_snapshot() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let mut truss = TrussElement::new_tension_only_nonlinear(&data, &model);
        let ctx = Ctx { model: &model };
        let ea_l = truss.e * truss.a / truss.length;

        // 1. 引張側で確定（伸び +2.0）
        truss.update_state(&axial_du(2.0), true, &ctx);
        assert!((truss.committed_elongation - 2.0).abs() < 1e-12);
        let f = truss.internal_force(&ElemState::default(), &ctx);
        assert!((f.data[6] - ea_l * 2.0).abs() < 1e-6);

        // 2. 確定基準から -5.0 の試行変位 → 伸び -3.0（圧縮）。まだ未確定。
        truss.update_state(&axial_du(-5.0), false, &ctx);
        assert!((truss.trial_elongation - (-3.0)).abs() < 1e-12);
        let k_compress = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k_compress.get(0, 0) - ea_l * NONLINEAR_COMPRESSION_FACTOR).abs() < 1e-9);
        let f_compress = truss.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            assert!(f_compress.data[i].abs() < 1e-9);
        }

        // 3. スナップショット（確定=2.0引張／試行=-3.0圧縮）を保存
        let snap = truss.snapshot_state();

        // 4. revert すると試行が確定値（引張2.0）へ戻る
        truss.revert_state();
        assert!((truss.trial_elongation - 2.0).abs() < 1e-12);
        assert!((truss.committed_elongation - 2.0).abs() < 1e-12);
        let k_reverted = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k_reverted.get(0, 0) - ea_l).abs() < 1e-6);

        // 5. restore で圧縮の試行状態を復元できる（往復確認）
        truss.restore_state(&*snap);
        assert!((truss.committed_elongation - 2.0).abs() < 1e-12);
        assert!((truss.trial_elongation - (-3.0)).abs() < 1e-12);

        // 6. その圧縮試行を確定する
        truss.commit_state();
        assert!((truss.committed_elongation - (-3.0)).abs() < 1e-12);

        // 7. 圧縮確定状態から再び引張へ（+10.0 で伸び +7.0 に確定）
        truss.update_state(&axial_du(10.0), true, &ctx);
        assert!((truss.committed_elongation - 7.0).abs() < 1e-12);
        let k_final = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k_final.get(0, 0) - ea_l).abs() < 1e-6);
        let f_final = truss.internal_force(&ElemState::default(), &ctx);
        assert!((f_final.data[6] - ea_l * 7.0).abs() < 1e-6);
    }

    /// 非線形モードのチェックポイント直列化/復元の往復確認。
    #[test]
    fn test_tension_only_nonlinear_checkpoint_roundtrip() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let mut truss = TrussElement::new_tension_only_nonlinear(&data, &model);
        let ctx = Ctx { model: &model };

        truss.update_state(&axial_du(2.0), true, &ctx);
        truss.update_state(&axial_du(-5.0), false, &ctx);
        let checkpoint = truss.serialize_checkpoint();

        let mut restored = TrussElement::new_tension_only_nonlinear(&data, &model);
        restored.deserialize_checkpoint(&checkpoint);
        assert!((restored.committed_elongation - 2.0).abs() < 1e-12);
        assert!((restored.trial_elongation - (-3.0)).abs() < 1e-12);
    }

    /// 線形モード（factor 指定）は非線形フィールドの影響を受けず、
    /// 既存挙動（factor·EA/L の一定剛性）のまま変わらないこと。
    #[test]
    fn test_linear_mode_unaffected_by_nonlinear_fields() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]);
        let mut truss = TrussElement::new(&data, &model, 1.0);
        assert!(!truss.nonlinear);
        let ctx = Ctx { model: &model };
        let ea_l = truss.e * truss.a / truss.length;

        // 圧縮方向の変位を与えても線形モードでは剛性は変化しない
        truss.update_state(&axial_du(-1.0), true, &ctx);
        let k = truss.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k.get(0, 0) - ea_l).abs() < 1e-6);
        let f = truss.internal_force(&ElemState::default(), &ctx);
        assert!((f.data[6] + ea_l).abs() < 1e-6);
    }
}
