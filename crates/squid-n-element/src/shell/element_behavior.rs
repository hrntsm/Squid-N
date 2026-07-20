//! [`ElementBehavior`](crate::behavior::ElementBehavior) 実装。
//!
//! - 自由度写像・接線剛性・内力・質量行列・断面力回復を要素インターフェースへ接続

use super::element::ShellElement;
use super::geom::element_area;
use super::shape::{jacobian, jacobian_det, shape_2d, GAUSS_PTS_2};
use crate::behavior::{ElemState, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};

// ---------------------------------------------------------------------------
// ElementBehavior implementation
// ---------------------------------------------------------------------------
impl crate::behavior::ElementBehavior for ShellElement {
    fn n_dof(&self) -> usize {
        24
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
        let mut k_local = self.local_stiffness();
        self.apply_rigid_floor_membrane_off(&mut k_local);
        self.frame.to_global(&k_local)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &crate::behavior::Ctx) -> LocalVec {
        // 線形弾性: f = K_global · u（トライアル追従。beam/behavior.rs と同じ規約）。
        // 接線剛性と同じ構成（剛床時の面内剛性無効化を含む）で評価し、
        // K・u の整合を保つ。従来は恒常的にゼロを返しており、非線形解析で
        // シェルが復元力を全く負担していなかった。
        // 注: 剛床所属（membrane_active=false）時の面内成分は
        // apply_rigid_floor_membrane_off が特異回避のため対角へ入れるダミー剛性
        // 1.0 に由来する微小な寄生力を持ち、厳密なゼロにはならない（実剛性より
        // 約 10 桁小さく実害なし。接線剛性と同一の K を使うため数学的には整合）。
        let mut k_local = self.local_stiffness();
        self.apply_rigid_floor_membrane_off(&mut k_local);
        let k = self.frame.to_global(&k_local);
        let mut f = LocalVec {
            data: smallvec::smallvec![0.0; 24],
        };
        for i in 0..24 {
            let mut s = 0.0;
            for j in 0..24 {
                s += k.get(i, j) * self.trial_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &crate::behavior::Ctx) {
        for i in 0..24.min(du.data.len()) {
            self.trial_disp[i] += du.data[i];
        }
        if commit {
            self.committed_disp = self.trial_disp;
        }
    }

    fn commit_state(&mut self) {
        self.committed_disp = self.trial_disp;
    }

    fn revert_state(&mut self) {
        self.trial_disp = self.committed_disp;
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        Box::new((self.committed_disp, self.trial_disp))
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        if let Some((committed, trial)) = state.downcast_ref::<([f64; 24], [f64; 24])>() {
            self.committed_disp = *committed;
            self.trial_disp = *trial;
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        bincode::serialize(&(self.committed_disp, self.trial_disp)).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        let (committed, trial): ([f64; 24], [f64; 24]) = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_disp = committed;
        self.trial_disp = trial;
        Ok(())
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let area = element_area(&self.coords);
        let m_total = self.density * self.t * area;
        let mut mm = LocalMat::zeros(24);
        match opt {
            MassOption::Lumped => {
                let m_node = m_total / 4.0;
                for i in 0..4 {
                    let bo = i * 6;
                    mm.set(bo, bo, m_node);
                    mm.set(bo + 1, bo + 1, m_node);
                    mm.set(bo + 2, bo + 2, m_node);
                }
            }
            MassOption::Consistent => {
                // Consistent mass uses 2×2 Gauss integration of NᵀρtN
                let lc = self.local_coords();
                for gi in 0..2 {
                    for gj in 0..2 {
                        let gp = gi * 2 + gj;
                        let xi = GAUSS_PTS_2[gp].0;
                        let eta = GAUSS_PTS_2[gp].1;
                        let det_j = jacobian_det(&jacobian(xi, eta, &lc));
                        let weight = det_j;
                        let n = shape_2d(xi, eta);
                        let rho_t = self.density * self.t;
                        for a in 0..4 {
                            let bo_a = a * 6;
                            let na = n[a];
                            for b in 0..4 {
                                let bo_b = b * 6;
                                let nb = n[b];
                                let contrib = na * nb * rho_t * weight;
                                for d in 0..3 {
                                    let ia = bo_a + d;
                                    let ib = bo_b + d;
                                    mm.set(ia, ib, mm.get(ia, ib) + contrib);
                                }
                            }
                        }
                    }
                }
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        let mut arr = [0.0; 24];
        arr.copy_from_slice(&u_elem[..24]);
        let resultants = self.recover_resultants(&arr);
        let data: Vec<(f64, [f64; 6])> = resultants
            .into_iter()
            .map(|(pt, r)| (pt[0], [r.nx, r.ny, r.nxy, r.mx, r.my, r.mxy]))
            .collect();
        Some(crate::beam::MemberForces { at: data })
    }
}
