//! 節点バネ要素（構造力学。部材の変形と自由度）。
//!
//! 構造力学上、部材の変形と自由度を整理すると、節点バネは
//! θX（ねじり） = ―（非考慮）、θY = ○、θZ = ○、γY（Y方向せん断）= ○、
//! γZ（Z方向せん断）= ○、δX（軸方向）= ○ の変形成分を持ちうる。
//! つまりねじり以外の曲げ・せん断・軸の各自由度に独立なバネ剛性を
//! 定義できる 2 節点要素である。
//!
//! ## モデル化
//!
//! 局所座標系の各自由度 d ∈ {ux, uy, uz, rx, ry, rz} ごとに独立なバネ定数
//! `k = [kx, ky, kz, krx, kry, krz]`（軸[N/mm]・せん断[N/mm]・回転[N·mm/rad]）を
//! 持つ。標準的な 2 節点バネ剛性として、局所自由度 d（0..6）・j 端側 d+6 に対し
//!
//! ```text
//! K[d][d]     = +k[d]
//! K[d+6][d+6] = +k[d]
//! K[d][d+6]   = K[d+6][d] = -k[d]
//! ```
//!
//! を組む（軸方向バネ・せん断バネ・回転バネのいずれも同形）。
//!
//! θX（ねじり）を非考慮とする扱いは `krx = 0` を既定とすることで対応する
//! （`ElementData::spring` は 6 成分すべてを入力可能とし、θX 方向にも
//! バネ定数を与えることを妨げない。既定値として 0 を渡せば非考慮と等価）。
//!
//! ## 局所座標系
//!
//! 2 節点が同一座標（零長バネ）の場合、`LocalFrame::from_nodes` は方向ベクトルが
//! 定義できないため、全体座標系＝局所座標系（単位回転）とみなして扱う
//! （零長バネは主に鉛直な独立要素として用いられ、軸の傾きを持たないため）。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, Model};

/// 節点バネ要素本体。
#[derive(Clone)]
pub struct NodalSpringElement {
    pub id: ElemId,
    pub nodes: [NodeId; 2],
    /// 局所軸バネ定数 `[kx, ky, kz, krx, kry, krz]`。`spring` 未指定時は全 0
    /// （剛性ゼロ。パニックせず安全にフォールバックする）。
    pub k: [f64; 6],
    /// 局所座標系。2 節点が同一座標（零長バネ）の場合は単位回転（全体座標系＝局所座標系）。
    pub axis: LocalFrame,
    /// 確定変位（線形要素の内力計算用。グローバル座標系で蓄積）。
    pub committed_disp: [f64; 12],
}

/// 2 節点間距離が実質ゼロとみなす閾値 [mm]。
const ZERO_LENGTH_EPS: f64 = 1e-9;

impl NodalSpringElement {
    pub fn new(data: &ElementData, model: &Model) -> Self {
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

        // 零長バネ（2 節点が同一座標）は方向ベクトルが定義できないため、
        // 全体座標系＝局所座標系（単位回転）とする。
        let axis = if len < ZERO_LENGTH_EPS {
            LocalFrame {
                rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            }
        } else {
            LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector)
        };

        // spring 未指定は剛性ゼロへ安全にフォールバック（パニックしない）。
        let k = data.spring.unwrap_or([0.0; 6]);

        Self {
            id: data.id,
            nodes: [n0, n1],
            k,
            axis,
            committed_disp: [0.0; 12],
        }
    }

    /// 局所座標系での 12×12 剛性行列。
    /// 各自由度 d（0..6）について K[d][d]=+k, K[d+6][d+6]=+k, K[d][d+6]=K[d+6][d]=-k。
    pub fn local_stiffness(&self) -> LocalMat {
        let mut m = LocalMat::zeros(12);
        for (d, &kd) in self.k.iter().enumerate() {
            if kd == 0.0 {
                continue;
            }
            m.set(d, d, kd);
            m.set(d + 6, d + 6, kd);
            m.set(d, d + 6, -kd);
            m.set(d + 6, d, -kd);
        }
        m
    }
}

impl ElementBehavior for NodalSpringElement {
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
        self.axis.to_global(&self.local_stiffness())
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // 線形弾性: f = K_global · u（truss.rs の committed_disp 方式に倣う）。
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

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // 節点バネは質量を持たない（質量規定は設けない。既存要素の質量は
        // 接続する節点・部材側で評価される想定）。
        LocalMat::zeros(12)
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
        // 両端 2 評価点の [N, Qy, Qz, Mx, My, Mz]（バネ力。beam.rs/truss.rs と同じ
        // 局所軸・符号の約束: i 端 = f_local[0..6]、j 端は反力として符号反転）。
        let i_end = [
            f_local[0], f_local[1], f_local[2], f_local[3], f_local[4], f_local[5],
        ];
        let j_end = [
            -f_local[6],
            -f_local[7],
            -f_local[8],
            -f_local[9],
            -f_local[10],
            -f_local[11],
        ];
        Some(crate::beam::MemberForces {
            at: vec![(0.0, i_end), (1.0, j_end)],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, RigidZone,
    };

    fn make_model_with_k(
        p0: [f64; 3],
        p1: [f64; 3],
        ref_vec: [f64; 3],
        k: [f64; 6],
    ) -> (Model, ElementData) {
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
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::NodalSpring,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: ref_vec,
            },
            end_cond: [EndCondition::Pinned, EndCondition::Pinned],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: Some(k),
        };
        (model, data)
    }

    fn make_model(p0: [f64; 3], p1: [f64; 3], ref_vec: [f64; 3]) -> (Model, ElementData) {
        make_model_with_k(p0, p1, ref_vec, [1000.0, 2000.0, 3000.0, 0.0, 5.0e6, 6.0e6])
    }

    /// 1) 軸バネ: 一端固定・他端に軸力 P を与えたときの変位 δ = P/kx が剛性行列（局所 0,0 / 6,6 / 0,6 / 6,0 成分）から手計算どおりに再現されること。
    #[test]
    fn test_axial_spring_local_stiffness_and_displacement() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let spring = NodalSpringElement::new(&data, &model);
        let k = spring.local_stiffness();
        let kx = 1000.0;
        assert!((k.get(0, 0) - kx).abs() < 1e-9);
        assert!((k.get(6, 6) - kx).abs() < 1e-9);
        assert!((k.get(0, 6) + kx).abs() < 1e-9);
        assert!((k.get(6, 0) + kx).abs() < 1e-9);

        // i 端固定・j 端自由の 1DOF 系: K·u = f → kx·u = P → u = P/kx。
        // 縮約: [kx, -kx; -kx, kx] のうち i端固定行を除いた j端の式は
        // kx·u_j = P + kx·u_i(=0) なので u_j = P/kx。
        let p = 500.0;
        let u_j = p / kx;
        assert!((k.get(6, 6) * u_j - p).abs() < 1e-9);
    }

    /// 2) 回転バネ kry/krz の剛性行列成分の照合（局所 4,4/5,5 と j 端対応成分）。
    #[test]
    fn test_rotational_spring_stiffness_components() {
        let (model, data) = make_model([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let spring = NodalSpringElement::new(&data, &model);
        let k = spring.local_stiffness();
        let kry = 5.0e6;
        let krz = 6.0e6;
        assert!((k.get(4, 4) - kry).abs() < 1e-6);
        assert!((k.get(10, 10) - kry).abs() < 1e-6);
        assert!((k.get(4, 10) + kry).abs() < 1e-6);
        assert!((k.get(10, 4) + kry).abs() < 1e-6);

        assert!((k.get(5, 5) - krz).abs() < 1e-6);
        assert!((k.get(11, 11) - krz).abs() < 1e-6);
        assert!((k.get(5, 11) + krz).abs() < 1e-6);
        assert!((k.get(11, 5) + krz).abs() < 1e-6);

        // θX（ねじり）は既定 0（非考慮）。
        assert_eq!(k.get(3, 3), 0.0);
        assert_eq!(k.get(9, 9), 0.0);
    }

    /// 3) 零長バネ（2 節点が同一座標）が全体座標系＝局所座標系として機能すること。
    #[test]
    fn test_zero_length_spring_uses_global_as_local() {
        let (model, data) = make_model(
            [100.0, 200.0, 300.0],
            [100.0, 200.0, 300.0],
            [1.0, 0.0, 0.0],
        );
        let spring = NodalSpringElement::new(&data, &model);
        // 単位回転であること
        assert_eq!(
            spring.axis.rot,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        );

        let ctx = Ctx { model: &model };
        let k_global = spring.tangent_stiffness(&ElemState::default(), &ctx);
        let k_local = spring.local_stiffness();
        for i in 0..12 {
            for j in 0..12 {
                assert!((k_global.get(i, j) - k_local.get(i, j)).abs() < 1e-9);
            }
        }
    }

    /// 4) 局所軸が傾いた配置（45°）での座標変換の照合。局所 x 軸（軸バネ方向）を全体座標へ回転させたとき、全体剛性の ii ブロックが k·(t·tᵀ) に一致すること（truss.rs と同じ検証手法）。
    #[test]
    fn test_tilted_axis_global_stiffness_matches_projection() {
        // 軸バネ成分のみ（ky=kz=kr*=0）にして、他自由度の寄与を排除した上で
        // truss.rs と同じ t·tᵀ 射影照合を行う。
        let l = (2000.0_f64 * 2000.0 + 2000.0 * 2000.0).sqrt(); // 45°、水平面内
        let (model, data) = make_model_with_k(
            [0.0, 0.0, 0.0],
            [2000.0, 2000.0, 0.0],
            [0.0, 0.0, 1.0],
            [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let spring = NodalSpringElement::new(&data, &model);
        let ctx = Ctx { model: &model };
        let k_global = spring.tangent_stiffness(&ElemState::default(), &ctx);

        let t = [2000.0 / l, 2000.0 / l, 0.0];
        let kx = 1000.0;
        for i in 0..3 {
            for j in 0..3 {
                let expected = kx * t[i] * t[j];
                assert!(
                    (k_global.get(i, j) - expected).abs() < 1e-6,
                    "K[{i}][{j}]: {} vs {}",
                    k_global.get(i, j),
                    expected
                );
                assert!((k_global.get(i + 6, j + 6) - expected).abs() < 1e-6);
                assert!((k_global.get(i, j + 6) + expected).abs() < 1e-6);
            }
        }
    }

    /// 5) `spring: None` の NodalSpring は剛性ゼロ（安全なフォールバック）となり、パニックしないこと。
    #[test]
    fn test_spring_none_falls_back_to_zero_stiffness_no_panic() {
        let (model, mut data) = make_model([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        data.spring = None;
        let spring = NodalSpringElement::new(&data, &model);
        assert_eq!(spring.k, [0.0; 6]);
        let ctx = Ctx { model: &model };
        let k_global = spring.tangent_stiffness(&ElemState::default(), &ctx);
        for i in 0..12 {
            for j in 0..12 {
                assert_eq!(k_global.get(i, j), 0.0);
            }
        }
        // internal_force / recover_forces もパニックしないこと
        let f = spring.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            assert_eq!(f.data[i], 0.0);
        }
        assert!(spring.recover_forces(&[0.0; 12]).is_some());
    }

    /// 剛性行列は対称であること（軸・せん断・回転すべての成分が独立対角なので当然だが、
    /// 全体座標変換後も対称性が保たれることを確認する）。
    #[test]
    fn test_global_stiffness_symmetric() {
        let (model, data) =
            make_model([500.0, 100.0, 0.0], [1800.0, 900.0, 700.0], [0.0, 0.0, 1.0]);
        let spring = NodalSpringElement::new(&data, &model);
        let ctx = Ctx { model: &model };
        let k = spring.tangent_stiffness(&ElemState::default(), &ctx);
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                    "K[{i}][{j}] != K[{j}][{i}]"
                );
            }
        }
    }
}
