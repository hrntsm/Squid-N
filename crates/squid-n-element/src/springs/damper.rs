//! 制振ダンパー要素（RESP-D「07 非線形解析（動的解析）」制振要素）。
//!
//! # マクスウェル要素
//! バネ剛性 `Kd` と粘性ダッシュポット（力 `Fc = C0·sign(V)·|V|^α`）を直列に接続した
//! 2 節点軸方向要素。連結点変位 `Ud` を挟み、要素力 = バネ力 `Fk = Kd(Uij − Ud)` が
//! ダッシュポット力と釣り合う（`Fk = Fc`）。時刻歴では後退 Euler で `Ud` を毎ステップ
//! 更新し、`V = (Ud − Ud_前) / Δt` として釣合いを解く。
//!
//! - 線形（α=1）: `Ud = (C0·Ud_前 + Δt·Kd·Uij) / (C0 + Δt·Kd)`（閉形式）。
//! - 非線形（α≠1）: 上式を初期値としてスカラー Newton 法で `Ud` を求める。
//!
//! 減衰要素の要素力は節点力として運動方程式へ与えられる（RESP-D）。本実装は
//! 収束用に整合接線 `∂Fk/∂Uij` を接線剛性へ与えるが、収束解は要素力の釣合いに
//! 一致するため結果は原典と等価。`Δt<=0`（静的・線形解析）では不活性（力・剛性 0）。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};
use std::any::Any;

/// マクスウェルダンパー要素（2 節点・軸方向）。
#[derive(Clone)]
pub struct MaxwellDamperElement {
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    /// バネ剛性 Kd [N/mm]。
    pub kd: f64,
    /// 粘性係数 C0 [N·(s/mm)^α]。
    pub c0: f64,
    /// 速度指数 α。
    pub alpha: f64,
    /// 時間刻み Δt [s]（0 以下で不活性）。
    dt: f64,
    /// 確定軸伸び Uij [mm]（引張正）。
    committed_elong: f64,
    /// 試行軸伸び Uij [mm]。
    trial_elong: f64,
    /// 確定連結点変位 Ud [mm]。
    committed_ud: f64,
}

impl MaxwellDamperElement {
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
        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let props = model.damper_props(data.id).unwrap_or_default();
        Self {
            nodes: [n0, n1],
            axis,
            kd: props.kd.max(0.0),
            c0: props.c0.max(0.0),
            alpha: if props.alpha > 0.0 { props.alpha } else { 1.0 },
            dt: 0.0,
            committed_elong: 0.0,
            trial_elong: 0.0,
            committed_ud: 0.0,
        }
    }

    /// 与えた軸伸び `elong` に対する連結点変位 `Ud` を後退 Euler で解く。
    /// `Δt<=0` では `Ud=elong`（バネ力 0 = 不活性）。
    fn solve_ud(&self, elong: f64) -> f64 {
        if self.dt <= 0.0 || self.kd <= 0.0 {
            return elong;
        }
        let ud0 = self.committed_ud;
        // 線形（α=1）閉形式を初期値に。
        let mut ud = (self.c0 * ud0 + self.dt * self.kd * elong) / (self.c0 + self.dt * self.kd);
        if (self.alpha - 1.0).abs() < 1e-9 || self.c0 <= 0.0 {
            return ud;
        }
        // 非線形（α≠1）: g(Ud) = Kd(elong−Ud) − C0·sign(V)·|V|^α = 0、V=(Ud−ud0)/Δt。
        for _ in 0..30 {
            let v = (ud - ud0) / self.dt;
            let fc = self.c0 * v.signum() * v.abs().powf(self.alpha);
            let g = self.kd * (elong - ud) - fc;
            // g'(Ud) = −Kd − C0·α·|V|^(α−1)/Δt
            let dfc = self.c0 * self.alpha * v.abs().powf(self.alpha - 1.0) / self.dt;
            let gp = -self.kd - dfc;
            if gp.abs() < 1e-30 {
                break;
            }
            let step = g / gp;
            ud -= step;
            if step.abs() < 1e-12 * (1.0 + ud.abs()) {
                break;
            }
        }
        ud
    }

    /// 現在の軸力 N [N]（引張正）= Kd(Uij − Ud)。`Δt<=0` は 0（不活性）。
    fn axial_force(&self, elong: f64) -> f64 {
        if self.dt <= 0.0 {
            return 0.0;
        }
        self.kd * (elong - self.solve_ud(elong))
    }

    /// 整合接線軸剛性 K_eff = Kd·C'/(Δt·Kd + C')、C'=C0·α·|V|^(α−1)（現在速度で評価）。
    /// `Δt<=0` は 0（不活性）。
    fn axial_tangent(&self) -> f64 {
        if self.dt <= 0.0 || self.kd <= 0.0 {
            return 0.0;
        }
        let ud = self.solve_ud(self.trial_elong);
        let v = (ud - self.committed_ud) / self.dt;
        let c_prime = if (self.alpha - 1.0).abs() < 1e-9 {
            self.c0
        } else {
            self.c0 * self.alpha * v.abs().max(1e-12).powf(self.alpha - 1.0)
        };
        let denom = self.dt * self.kd + c_prime;
        if denom <= 0.0 {
            0.0
        } else {
            self.kd * c_prime / denom
        }
    }

    fn local_stiffness(&self, ka: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }
}

impl ElementBehavior for MaxwellDamperElement {
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
        self.axis
            .to_global(&self.local_stiffness(self.axial_tangent()))
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let n = self.axial_force(self.trial_elong);
        let t = self.axis.rot[0];
        for k in 0..3 {
            f.data[k] = -n * t[k];
            f.data[6 + k] = n * t[k];
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        let delong = du_local[6] - du_local[0];
        if commit {
            let elong = self.committed_elong + delong;
            self.committed_ud = self.solve_ud(elong);
            self.committed_elong = elong;
            self.trial_elong = elong;
        } else {
            self.trial_elong = self.committed_elong + delong;
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // ダンパー要素は構造質量を持たない（自重はダンパー諸元・荷重側で扱う）。
        LocalMat::zeros(12)
    }

    fn commit_state(&mut self) {
        self.committed_ud = self.solve_ud(self.trial_elong);
        self.committed_elong = self.trial_elong;
    }

    fn revert_state(&mut self) {
        self.trial_elong = self.committed_elong;
    }

    fn set_time_step(&mut self, dt: f64) {
        self.dt = dt;
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        Box::new((self.committed_elong, self.committed_ud, self.trial_elong))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some(&(ce, cud, te)) = state.downcast_ref::<(f64, f64, f64)>() {
            self.committed_elong = ce;
            self.committed_ud = cud;
            self.trial_elong = te;
        }
    }
}

// ──────────────────────────── 履歴型（弾塑性バイリニア）ダンパー ────────────────────────────

/// 履歴型（弾塑性バイリニア）ダンパー要素（2 節点・軸方向）。
/// 鋼材系ダンパー（SUB／アンボンドブレース／二重鋼管座屈補剛ブレース／鉛／U 型等、
/// RESP-D「07」制振要素の標準型バイリニア）。初期軸剛性 `k1`・降伏軸力 `qy`・第2剛性 `k2`
/// の弾塑性軸ばね。変位依存のため静的・動的いずれの解析でも作用する（`dt` 不要）。
#[derive(Clone)]
pub struct HystereticDamperElement {
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    /// 軸力–伸び関係の弾塑性材料（Bilinear を力–変位として流用）。
    mat: squid_n_material::Bilinear,
    committed_elong: f64,
    trial_elong: f64,
}

impl HystereticDamperElement {
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
        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let props = model.damper_props(data.id).unwrap_or_default();
        let k1 = props.kd.max(1e-9);
        let qy = props.qy.max(1e-9);
        let hardening = props.k2_ratio.clamp(0.0, 0.999);
        Self {
            nodes: [n0, n1],
            axis,
            // Bilinear(e=k1, fy=qy, hardening=k2/k1) を軸力–伸びの弾塑性則として用いる。
            mat: squid_n_material::Bilinear::new(k1, qy, hardening),
            committed_elong: 0.0,
            trial_elong: 0.0,
        }
    }

    fn axial(&mut self, elong: f64) -> (f64, f64) {
        // Bilinear::trial は (応力, 接線) を返すが、ここでは (軸力, 軸剛性) に相当。
        use squid_n_material::UniaxialMaterial;
        self.mat.trial(elong)
    }

    fn local_stiffness(&self, ka: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }
}

impl ElementBehavior for HystereticDamperElement {
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
        // trial を汚さないよう複製して接線のみ取得。
        use squid_n_material::UniaxialMaterial;
        let mut m = self.mat.clone();
        let (_f, k) = m.trial(self.trial_elong);
        self.axis.to_global(&self.local_stiffness(k))
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        use squid_n_material::UniaxialMaterial;
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let mut m = self.mat.clone();
        let (n, _k) = m.trial(self.trial_elong);
        let t = self.axis.rot[0];
        for k in 0..3 {
            f.data[k] = -n * t[k];
            f.data[6 + k] = n * t[k];
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        let delong = du_local[6] - du_local[0];
        self.trial_elong = self.committed_elong + delong;
        let _ = self.axial(self.trial_elong);
        if commit {
            use squid_n_material::UniaxialMaterial;
            self.mat.commit();
            self.committed_elong = self.trial_elong;
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }

    fn commit_state(&mut self) {
        use squid_n_material::UniaxialMaterial;
        let _ = self.axial(self.trial_elong);
        self.mat.commit();
        self.committed_elong = self.trial_elong;
    }

    fn revert_state(&mut self) {
        use squid_n_material::UniaxialMaterial;
        self.mat.revert();
        self.trial_elong = self.committed_elong;
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        use squid_n_material::UniaxialMaterial;
        Box::new((
            self.committed_elong,
            self.trial_elong,
            self.mat.serialize_state(),
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        use squid_n_material::UniaxialMaterial;
        if let Some((ce, te, ms)) = state.downcast_ref::<(f64, f64, Vec<u8>)>() {
            self.committed_elong = *ce;
            self.trial_elong = *te;
            self.mat.deserialize_state(ms);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn damper(kd: f64, c0: f64, alpha: f64, dt: f64) -> MaxwellDamperElement {
        MaxwellDamperElement {
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            kd,
            c0,
            alpha,
            dt,
            committed_elong: 0.0,
            trial_elong: 0.0,
            committed_ud: 0.0,
        }
    }

    #[test]
    fn test_maxwell_inert_when_dt_zero() {
        // Δt<=0（静的・線形）は不活性（力・剛性 0）。
        let d = damper(100.0, 1000.0, 1.0, 0.0);
        assert_eq!(d.axial_force(5.0), 0.0);
        assert_eq!(d.axial_tangent(), 0.0);
    }

    #[test]
    fn test_maxwell_locks_at_fast_step_then_relaxes() {
        // 速い載荷（1 ステップ）ではダッシュポットがほぼロックしバネ力 ≈ Kd·Uij。
        // 変形一定で時間が進むとダッシュポットが緩和し力 → 0。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        d.trial_elong = 1.0;
        let f0 = d.axial_force(1.0);
        assert!(
            f0 > 90.0,
            "fast step should be near-locked (Kd·Uij), got {f0}"
        );
        // 変形を 1.0 に保ったまま多数ステップ commit（緩和）。
        for _ in 0..5000 {
            d.commit_state();
        }
        let f1 = d.axial_force(1.0);
        assert!(
            f1 < f0 * 0.1,
            "dashpot should relax force toward 0: f0={f0}, f1={f1}"
        );
    }

    #[test]
    fn test_maxwell_tangent_matches_finite_difference() {
        // 整合接線が有限差分と一致（α=1）。K_eff = Kd·C0/(C0+Δt·Kd)。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        d.trial_elong = 0.5;
        let kt = d.axial_tangent();
        let h = 1e-6;
        let f1 = d.axial_force(0.5 + h);
        let f2 = d.axial_force(0.5 - h);
        let fd = (f1 - f2) / (2.0 * h);
        assert!((kt - fd).abs() < 1e-3 * kt.max(1.0), "kt={kt}, fd={fd}");
        let expect = 100.0 * 1000.0 / (1000.0 + 0.01 * 100.0);
        assert!((kt - expect).abs() < 1e-6, "kt={kt}, expect={expect}");
    }

    #[test]
    fn test_maxwell_nonlinear_alpha_solves() {
        // α≠1（非線形粘性）でも Ud が釣合い（Kd(elong−Ud)=Fc(V)）を満たす。
        let mut d = damper(100.0, 500.0, 0.5, 0.02);
        d.committed_ud = 0.0;
        let elong = 2.0;
        let ud = d.solve_ud(elong);
        let v = (ud - 0.0) / d.dt;
        let fc = d.c0 * v.signum() * v.abs().powf(d.alpha);
        let fk = d.kd * (elong - ud);
        assert!(
            (fk - fc).abs() < 1e-6 * fk.abs().max(1.0),
            "fk={fk}, fc={fc}"
        );
    }

    #[test]
    fn test_maxwell_element_drives_axial_force() {
        // update_state で節点変位を与え、internal_force が軸方向へ力を返す。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        // 節点1 の ux に 1.0mm（軸方向 = グローバル X）。
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        du.data[6] = 1.0;
        let model = Model::default();
        let ctx = Ctx { model: &model };
        d.update_state(&du, false, &ctx);
        assert!((d.trial_elong - 1.0).abs() < 1e-9);
        let f = d.internal_force(&ElemState {}, &ctx);
        let n = d.axial_force(1.0);
        assert!(n > 0.0);
        assert!((f.data[0] + n).abs() < 1e-9); // 節点0 ux = −N
        assert!((f.data[6] - n).abs() < 1e-9); // 節点1 ux = +N
    }

    fn hyst_damper(k1: f64, qy: f64, k2ratio: f64) -> HystereticDamperElement {
        HystereticDamperElement {
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            mat: squid_n_material::Bilinear::new(k1, qy, k2ratio),
            committed_elong: 0.0,
            trial_elong: 0.0,
        }
    }

    /// 節点1 の軸方向（グローバル X）へ伸び elong を与えて commit。
    fn drive(d: &mut HystereticDamperElement, elong: f64, model: &Model) {
        let prev = d.committed_elong;
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        du.data[6] = elong - prev;
        let ctx = Ctx { model };
        d.update_state(&du, true, &ctx);
    }

    #[test]
    fn test_hysteretic_bilinear_elastic_then_yield() {
        // 弾性域: N=k1·elong。降伏後: N≈qy+k2·(elong−δy)。
        let k1 = 1000.0;
        let qy = 100.0;
        let mut d = hyst_damper(k1, qy, 0.02);
        let model = Model::default();
        let dy = qy / k1; // 0.1
        drive(&mut d, 0.5 * dy, &model);
        let ctx = Ctx { model: &model };
        let f_el = d.internal_force(&ElemState {}, &ctx).data[6];
        assert!((f_el - k1 * 0.5 * dy).abs() < 1e-6, "elastic: {f_el}");
        // 降伏後（5δy）。
        drive(&mut d, 5.0 * dy, &model);
        let f_pl = d.internal_force(&ElemState {}, &ctx).data[6];
        let expect = qy + 0.02 * k1 * (5.0 * dy - dy);
        assert!(
            (f_pl - expect).abs() < 1.0,
            "plastic: {f_pl}, expect={expect}"
        );
    }

    #[test]
    fn test_hysteretic_active_in_static_no_dt() {
        // 履歴型は dt 不要（set_time_step を呼ばずとも力を発生＝静的で作用）。
        let mut d = hyst_damper(1000.0, 100.0, 0.02);
        let model = Model::default();
        let ctx = Ctx { model: &model };
        drive(&mut d, 0.05, &model);
        let n = d.internal_force(&ElemState {}, &ctx).data[6];
        assert!(n > 0.0, "hysteretic damper must be active in static: {n}");
        let kt = d.tangent_stiffness(&ElemState {}, &ctx).get(6, 6);
        assert!(kt > 0.0);
    }

    #[test]
    fn test_hysteretic_dissipates_energy() {
        // 1 サイクルの履歴ループ面積（散逸エネルギー）が正。
        let k1 = 1000.0;
        let qy = 100.0;
        let mut d = hyst_damper(k1, qy, 0.02);
        let model = Model::default();
        let ctx = Ctx { model: &model };
        let dy = qy / k1;
        let amp = 4.0 * dy;
        let mut energy = 0.0;
        let mut prev = (0.0, 0.0);
        for i in 0..=80 {
            let phase = i as f64 / 20.0 * std::f64::consts::PI;
            let elong = amp * phase.sin();
            drive(&mut d, elong, &model);
            let n = d.internal_force(&ElemState {}, &ctx).data[6];
            energy += 0.5 * (prev.1 + n) * (elong - prev.0);
            prev = (elong, n);
        }
        assert!(
            energy > 0.0,
            "hysteretic loop must dissipate energy: {energy}"
        );
    }
}
