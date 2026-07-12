//! 免震支承材要素（RESP-D マニュアル「計算編 05 非線形モデル」免震支承材）。
//!
//! 2 節点要素。局所 x 軸（部材軸＝鉛直）は弾性軸ばね Kv、局所 y・z 軸（水平）は
//! 非線形せん断ばねでモデル化する。回転自由度は剛（モーメントを剛に伝達）とする。
//!
//! ## 水平せん断ばねのモデル
//! - **積層ゴム系（`IsolatorKind::LaminatedRubber`）**: 各水平方向を独立な
//!   バイリニア（初期剛性 K1・二次剛性 K2・特性耐力 Qd）でモデル化する。
//!   マルチシアスプリングの各方向独立性（一方向加力で直交方向は剛性を保持）に対応。
//! - **弾性すべり支承（`IsolatorKind::ElasticSliding`）**: 摩擦ばね。滑り出しを
//!   水平 2 方向の**合力ベクトル**で判定し（`|Q| ≥ Qmax=μ·N`）、滑り後は合力を
//!   Qmax に保つ 2 次元摩擦モデル（曲げは伝達しない）。
//!
//! ## 座標系・状態
//! 内部状態（`trial_disp`）は局所系で保持し、トレイト境界でグローバル系へ回転する
//! （`FiberBeam` と同じ規約）。せん断バイリニアの履歴は `squid_n_material::Bilinear`
//! （変位=ひずみ、力=応力とみなす）で追跡する。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, IsolatorKind, IsolatorProps, Model};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};
use std::any::Any;

/// 回転自由度に与える剛剛性 [N·mm/rad]（免震支承はモーメントを剛に伝達）。
const RIGID_ROT: f64 = 1.0e12;
/// 摩擦ばねの滑り後（塑性域）の残留せん断剛性比（数値安定用の微小値）。
const FRICTION_POST_SLIP_RATIO: f64 = 1.0e-4;
/// 2 節点間距離が実質ゼロとみなす閾値 [mm]。
const ZERO_LENGTH_EPS: f64 = 1e-9;

/// 水平せん断ばねの状態。
enum ShearModel {
    /// 積層ゴム系: 各方向独立バイリニア（局所 y, z）。
    Laminated { sy: Bilinear, sz: Bilinear },
    /// 弾性すべり支承: 2 次元摩擦（合力で滑り判定）。塑性変位 (committed, trial)。
    Friction {
        k1: f64,
        qmax: f64,
        pl_y: f64,
        pl_z: f64,
        tr_pl_y: f64,
        tr_pl_z: f64,
        tr_fy: f64,
        tr_fz: f64,
        tr_slip: bool,
    },
    /// 歪依存積層ゴム系（LRB 統一型・高減衰ゴム）: 特性耐力 Qd(γ)・二次剛性 K2(γ) が
    /// 歪 γ=|δ|/H に依存する各方向独立バイリニア。RESP-D「07」歪依存特性。
    StrainDependent {
        sy: StrainBilinear,
        sz: StrainBilinear,
        /// 基準特性耐力 Qd0 [N]。
        qd0: f64,
        /// 基準二次剛性 K2_0 [N/mm]。
        k2_0: f64,
        /// ゴム総厚 H [mm]（γ=|δ|/H）。
        h: f64,
        /// CQd(γ)=c0+c1·γ+c2·γ² 係数。
        cqd: [f64; 3],
        /// CKd(γ)=c0+c1·γ+c2·γ² 係数。
        ckd: [f64; 3],
    },
}

/// 歪依存 1 次元 kinematic バイリニア。降伏耐力 `qd`・二次剛性 `k2` を毎ステップ外部から
/// 与える（歪依存）。塑性変位 `pl`（累積・符号付き）を状態に保持する。
#[derive(Clone, Copy, Debug, Default)]
struct StrainBilinear {
    k1: f64,
    /// 確定塑性変位。
    pl: f64,
    /// 試行塑性変位。
    tr_pl: f64,
    /// 試行力・接線。
    tr_f: f64,
    tr_t: f64,
}

impl StrainBilinear {
    fn new(k1: f64) -> Self {
        Self {
            k1: k1.max(1e-9),
            pl: 0.0,
            tr_pl: 0.0,
            tr_f: 0.0,
            tr_t: k1.max(1e-9),
        }
    }

    /// 変位 `u`・降伏耐力 `qd`・二次剛性 `k2` に対する試行（kinematic 戻し写像）。
    fn trial(&mut self, u: f64, qd: f64, k2: f64) {
        let qd = qd.max(1e-9);
        let k2 = k2.clamp(0.0, self.k1 * 0.999);
        // 塑性係数 Hp: 降伏後接線 = k1·Hp/(k1+Hp) = k2 → Hp = k1·k2/(k1−k2)。
        let hp = if self.k1 > k2 {
            self.k1 * k2 / (self.k1 - k2)
        } else {
            1e18
        };
        let alpha = hp * self.pl; // 背応力（kinematic hardening）。
        let f_tr = self.k1 * (u - self.pl);
        let phi = (f_tr - alpha).abs() - qd;
        if phi <= 0.0 {
            self.tr_f = f_tr;
            self.tr_t = self.k1;
            self.tr_pl = self.pl;
        } else {
            let s = (f_tr - alpha).signum();
            let dpl = phi / (self.k1 + hp);
            self.tr_pl = self.pl + s * dpl;
            self.tr_f = self.k1 * (u - self.tr_pl);
            self.tr_t = if hp >= 1e17 {
                0.0
            } else {
                self.k1 * hp / (self.k1 + hp)
            };
        }
    }

    fn commit(&mut self) {
        self.pl = self.tr_pl;
    }
    fn revert(&mut self) {
        self.tr_pl = self.pl;
    }
}

/// γ の 2 次多項式 `c0+c1·γ+c2·γ²`（歪依存係数）。負値は 0 にクランプ。
fn poly3(c: &[f64; 3], gamma: f64) -> f64 {
    (c[0] + c[1] * gamma + c[2] * gamma * gamma).max(0.0)
}

/// 免震支承材要素。
pub struct IsolatorElement {
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub props: IsolatorProps,
    shear: ShearModel,
    committed_disp: [f64; 12],
    trial_disp: [f64; 12],
}

impl IsolatorElement {
    pub fn new(data: &ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = model
            .nodes
            .get(n0.index())
            .map(|n| n.coord)
            .unwrap_or([0.0; 3]);
        let p1 = model
            .nodes
            .get(n1.index())
            .map(|n| n.coord)
            .unwrap_or([0.0; 3]);
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let axis = if len < ZERO_LENGTH_EPS {
            // 零長支承は鉛直（局所 x=全体 z）を既定とする。
            LocalFrame::from_nodes(p0, [p0[0], p0[1], p0[2] + 1.0], data.local_axis.ref_vector)
        } else {
            LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector)
        };

        let props = model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == data.id)
            .map(|a| a.props)
            .unwrap_or_default();

        let shear = match props.kind {
            // 積層ゴム系（天然・鉛・高減衰）。ゴム総厚 H>0 かつ歪依存係数が非自明なら
            // 歪依存バイリニア、そうでなければ従来の定数バイリニア。
            IsolatorKind::LaminatedRubber
            | IsolatorKind::LeadRubber
            | IsolatorKind::HighDampingRubber => {
                let k1 = props.k1.max(1e-9);
                let strain_dep = props.total_rubber_thickness > 0.0
                    && (props.ckd_gamma != [1.0, 0.0, 0.0] || props.cqd_gamma != [1.0, 0.0, 0.0]);
                if strain_dep {
                    ShearModel::StrainDependent {
                        sy: StrainBilinear::new(k1),
                        sz: StrainBilinear::new(k1),
                        qd0: props.qd.max(1e-9),
                        k2_0: props.k2.max(0.0),
                        h: props.total_rubber_thickness,
                        cqd: props.cqd_gamma,
                        ckd: props.ckd_gamma,
                    }
                } else {
                    let hardening = if props.k1 > 0.0 {
                        (props.k2 / props.k1).clamp(0.0, 0.999)
                    } else {
                        0.0
                    };
                    ShearModel::Laminated {
                        sy: Bilinear::new(k1, props.qd.max(1e-9), hardening),
                        sz: Bilinear::new(k1, props.qd.max(1e-9), hardening),
                    }
                }
            }
            IsolatorKind::ElasticSliding => ShearModel::Friction {
                k1: props.k1.max(1e-9),
                qmax: (props.mu.max(0.0) * props.n_long.max(0.0)).max(0.0),
                pl_y: 0.0,
                pl_z: 0.0,
                tr_pl_y: 0.0,
                tr_pl_z: 0.0,
                tr_fy: 0.0,
                tr_fz: 0.0,
                tr_slip: false,
            },
        };

        Self {
            nodes: [n0, n1],
            axis,
            props,
            shear,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        }
    }

    /// 局所系のせん断力 (fy, fz) と接線 (ty, tz) を現在の trial_disp から求める。
    /// 摩擦は 2 次元合力で滑り判定するため、両方向を同時に評価する。
    fn shear_forces(&self) -> ((f64, f64), (f64, f64)) {
        let uy = self.trial_disp[7] - self.trial_disp[1];
        let uz = self.trial_disp[8] - self.trial_disp[2];
        match &self.shear {
            ShearModel::Laminated { sy, sz } => {
                // Bilinear は committed 状態を保持するので、非破壊評価のため複製して trial。
                let mut sy2 = sy.clone();
                let mut sz2 = sz.clone();
                let (fy, ty) = sy2.trial(uy);
                let (fz, tz) = sz2.trial(uz);
                ((fy, fz), (ty, tz))
            }
            ShearModel::Friction {
                tr_fy,
                tr_fz,
                k1,
                tr_slip,
                ..
            } => {
                let t = if *tr_slip {
                    *k1 * FRICTION_POST_SLIP_RATIO
                } else {
                    *k1
                };
                ((*tr_fy, *tr_fz), (t, t))
            }
            ShearModel::StrainDependent {
                sy,
                sz,
                qd0,
                k2_0,
                h,
                cqd,
                ckd,
            } => {
                // 非破壊評価: 複製して現在変位で試行。
                let gamma = ((uy * uy + uz * uz).sqrt() / h.max(1e-9)).abs();
                let qd = qd0 * poly3(cqd, gamma);
                let k2 = k2_0 * poly3(ckd, gamma);
                let mut sy2 = *sy;
                let mut sz2 = *sz;
                sy2.trial(uy, qd, k2);
                sz2.trial(uz, qd, k2);
                ((sy2.tr_f, sz2.tr_f), (sy2.tr_t, sz2.tr_t))
            }
        }
    }
}

impl ElementBehavior for IsolatorElement {
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

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let kv = self.props.kv.max(0.0);
        let (_, (ty, tz)) = self.shear_forces();
        // 局所系ばね: 軸(0)=Kv、せん断(1,2)=接線、回転(3,4,5)=剛。
        let k_local = [kv, ty, tz, RIGID_ROT, RIGID_ROT, RIGID_ROT];
        let mut m = LocalMat::zeros(12);
        for (d, &kd) in k_local.iter().enumerate() {
            if kd == 0.0 {
                continue;
            }
            m.set(d, d, kd);
            m.set(d + 6, d + 6, kd);
            m.set(d, d + 6, -kd);
            m.set(d + 6, d, -kd);
        }
        self.axis.to_global(&m)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let kv = self.props.kv.max(0.0);
        let ((fy, fz), _) = self.shear_forces();
        // 局所相対変位。
        let rel = |d: usize| self.trial_disp[d + 6] - self.trial_disp[d];
        let fx = kv * rel(0);
        let mrx = RIGID_ROT * rel(3);
        let mry = RIGID_ROT * rel(4);
        let mrz = RIGID_ROT * rel(5);
        // 局所系内力（i 端 = −f, j 端 = +f）。
        let f_local = [-fx, -fy, -fz, -mrx, -mry, -mrz, fx, fy, fz, mrx, mry, mrz];
        let f_global = self.axis.rotate_to_global(&f_local);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        for i in 0..12 {
            self.trial_disp[i] += du_local[i];
        }
        let uy = self.trial_disp[7] - self.trial_disp[1];
        let uz = self.trial_disp[8] - self.trial_disp[2];
        match &mut self.shear {
            ShearModel::Laminated { sy, sz } => {
                sy.trial(uy);
                sz.trial(uz);
                if commit {
                    sy.commit();
                    sz.commit();
                }
            }
            ShearModel::Friction {
                k1,
                qmax,
                pl_y,
                pl_z,
                tr_pl_y,
                tr_pl_z,
                tr_fy,
                tr_fz,
                tr_slip,
            } => {
                // 弾性予測（前回確定塑性変位から）。
                let fy_pred = *k1 * (uy - *pl_y);
                let fz_pred = *k1 * (uz - *pl_z);
                let norm = (fy_pred * fy_pred + fz_pred * fz_pred).sqrt();
                if norm <= *qmax || norm < 1e-12 {
                    // 固着（弾性）。
                    *tr_fy = fy_pred;
                    *tr_fz = fz_pred;
                    *tr_pl_y = *pl_y;
                    *tr_pl_z = *pl_z;
                    *tr_slip = false;
                } else {
                    // 滑り: 合力を Qmax に射影し、塑性変位を更新。
                    let scale = *qmax / norm;
                    *tr_fy = fy_pred * scale;
                    *tr_fz = fz_pred * scale;
                    *tr_pl_y = uy - *tr_fy / *k1;
                    *tr_pl_z = uz - *tr_fz / *k1;
                    *tr_slip = true;
                }
                if commit {
                    *pl_y = *tr_pl_y;
                    *pl_z = *tr_pl_z;
                }
            }
            ShearModel::StrainDependent {
                sy,
                sz,
                qd0,
                k2_0,
                h,
                cqd,
                ckd,
            } => {
                let gamma = ((uy * uy + uz * uz).sqrt() / h.max(1e-9)).abs();
                let qd = *qd0 * poly3(cqd, gamma);
                let k2 = *k2_0 * poly3(ckd, gamma);
                sy.trial(uy, qd, k2);
                sz.trial(uz, qd, k2);
                if commit {
                    sy.commit();
                    sz.commit();
                }
            }
        }
        if commit {
            self.committed_disp = self.trial_disp;
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let shear = match &self.shear {
            ShearModel::Laminated { sy, sz } => (Some((sy.clone(), sz.clone())), None, None),
            ShearModel::Friction { pl_y, pl_z, .. } => (None, Some((*pl_y, *pl_z)), None),
            ShearModel::StrainDependent { sy, sz, .. } => (None, None, Some((sy.pl, sz.pl))),
        };
        Box::new((self.trial_disp, self.committed_disp, shear))
    }

    #[allow(clippy::type_complexity)]
    fn restore_state(&mut self, state: &dyn Any) {
        if let Some((trial, committed, shear)) = state.downcast_ref::<(
            [f64; 12],
            [f64; 12],
            (
                Option<(Bilinear, Bilinear)>,
                Option<(f64, f64)>,
                Option<(f64, f64)>,
            ),
        )>() {
            self.trial_disp = *trial;
            self.committed_disp = *committed;
            match (&mut self.shear, &shear.0, &shear.1, &shear.2) {
                (ShearModel::Laminated { sy, sz }, Some((sy0, sz0)), _, _) => {
                    *sy = sy0.clone();
                    *sz = sz0.clone();
                }
                (
                    ShearModel::Friction {
                        pl_y,
                        pl_z,
                        tr_pl_y,
                        tr_pl_z,
                        ..
                    },
                    _,
                    Some((py, pz)),
                    _,
                ) => {
                    *pl_y = *py;
                    *pl_z = *pz;
                    *tr_pl_y = *py;
                    *tr_pl_z = *pz;
                }
                (ShearModel::StrainDependent { sy, sz, .. }, _, _, Some((py, pz))) => {
                    sy.pl = *py;
                    sy.tr_pl = *py;
                    sz.pl = *pz;
                    sz.tr_pl = *pz;
                }
                _ => {}
            }
        }
    }

    fn commit_state(&mut self) {
        match &mut self.shear {
            ShearModel::Laminated { sy, sz } => {
                sy.commit();
                sz.commit();
            }
            ShearModel::Friction {
                pl_y,
                pl_z,
                tr_pl_y,
                tr_pl_z,
                ..
            } => {
                *pl_y = *tr_pl_y;
                *pl_z = *tr_pl_z;
            }
            ShearModel::StrainDependent { sy, sz, .. } => {
                sy.commit();
                sz.commit();
            }
        }
        self.committed_disp = self.trial_disp;
    }

    fn revert_state(&mut self) {
        match &mut self.shear {
            ShearModel::Laminated { sy, sz } => {
                sy.revert();
                sz.revert();
            }
            ShearModel::Friction {
                pl_y,
                pl_z,
                tr_pl_y,
                tr_pl_z,
                ..
            } => {
                *tr_pl_y = *pl_y;
                *tr_pl_z = *pl_z;
            }
            ShearModel::StrainDependent { sy, sz, .. } => {
                sy.revert();
                sz.revert();
            }
        }
        self.trial_disp = self.committed_disp;
    }
}

#[cfg(test)]
mod tests;
