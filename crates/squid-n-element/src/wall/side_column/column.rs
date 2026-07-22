//! 側柱の要素本体（面内両端ピンの柱）。
//!
//! 静的縮約による剛性計算と `ElementBehavior` 実装、および解放曲げ面を表す
//! `ReleaseAxis`。

use crate::beam::{invert_small, BeamElement};
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;

/// 解放する局所曲げ面（回転自由度）。
///
/// - `LocalY`: 局所 y 軸回りの回転（ry, 要素ローカル自由度 4・10）を解放。
///   曲げ面は局所 x-z 面（たわみ方向 = 局所 z 軸）。
/// - `LocalZ`: 局所 z 軸回りの回転（rz, 要素ローカル自由度 5・11）を解放。
///   曲げ面は局所 x-y 面（たわみ方向 = 局所 y 軸）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReleaseAxis {
    LocalY,
    LocalZ,
}

/// 面内方向のみ両端ピンとした側柱（耐震壁の側柱）。
///
/// 内部に通常の柱と同じ `BeamElement` を持ち、剛性計算時に指定曲げ面の両端回転
/// 自由度を静的縮約で消去した 12×12 を用いる。軸・ねじり・面外曲げは `inner` と
/// 変わらない。
pub struct InPlaneReleasedColumn {
    pub(super) inner: BeamElement,
    release_axis: ReleaseAxis,
}

impl InPlaneReleasedColumn {
    pub fn new(inner: BeamElement, release_axis: ReleaseAxis) -> Self {
        Self {
            inner,
            release_axis,
        }
    }

    /// 解放対象の局所自由度（両端の回転自由度2個）。
    fn release_dofs(&self) -> [usize; 2] {
        match self.release_axis {
            ReleaseAxis::LocalY => [4, 10],
            ReleaseAxis::LocalZ => [5, 11],
        }
    }

    /// `inner.local_stiffness()` から解放曲げ面の両端回転自由度を静的縮約した局所 12×12。
    ///
    /// K* = Kaa − Kab·Kbb⁻¹·Kba（a: 残す10自由度、b: 解放する2自由度）。
    /// 縮約後の b 自由度の行・列は 0（その回転自由度に剛性を持たない＝ピン）。
    /// 軸・ねじり・他方向曲げは元の局所剛性で b 自由度と非連成のため影響を受けない。
    fn released_local_stiffness(&self) -> LocalMat {
        let k = self.inner.local_stiffness();
        let b = self.release_dofs();
        let n = k.n;

        // Kbb（2×2）とその逆行列
        let kbb = vec![
            k.get(b[0], b[0]),
            k.get(b[0], b[1]),
            k.get(b[1], b[0]),
            k.get(b[1], b[1]),
        ];
        let kbb_inv = invert_small(&kbb, 2);

        let mut out = LocalMat::zeros(n);
        for i in 0..n {
            if b.contains(&i) {
                continue;
            }
            let kai = [k.get(i, b[0]), k.get(i, b[1])];
            for j in 0..n {
                if b.contains(&j) {
                    continue;
                }
                let kbj = [k.get(b[0], j), k.get(b[1], j)];
                let mut corr = 0.0;
                for p in 0..2 {
                    for q in 0..2 {
                        corr += kai[p] * kbb_inv[p * 2 + q] * kbj[q];
                    }
                }
                out.set(i, j, k.get(i, j) - corr);
            }
        }
        out
    }

    /// 縮約後の局所剛性を用いた断面力の復元（`BeamElement::recover_forces` と同じ規約）。
    /// `BeamElement::recover_forces` は自身の（非解放の）`local_stiffness()` を用いるため、
    /// ここでは解放後の局所剛性で同じ算定式を再実装する。
    fn recover_forces_released(&self, u_elem_global: &[f64; 12]) -> crate::beam::MemberForces {
        let u_local = self.inner.axis.rotate_to_local(u_elem_global);
        let k_local = self.released_local_stiffness();
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        let length = self.inner.length;
        let mut at = Vec::new();
        for &xi in &self.inner.eval_sections {
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = -f_local[0];
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = -f_local[3];
                let my = -f_local[4] - f_local[2] * xi * length;
                let mz = -f_local[5] + f_local[1] * xi * length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[6];
                let qy = -f_local[7];
                let qz = -f_local[8];
                let mx = f_local[9];
                let my = f_local[10] - f_local[8] * (1.0 - xi) * length;
                let mz = f_local[11] + f_local[7] * (1.0 - xi) * length;
                (n, qy, qz, mx, my, mz)
            };
            at.push((xi, [n, qy, qz, mx, my, mz]));
        }
        crate::beam::MemberForces { at }
    }
}

impl ElementBehavior for InPlaneReleasedColumn {
    fn n_dof(&self) -> usize {
        self.inner.n_dof()
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.inner.global_dofs(dof)
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        self.inner.axis.to_global(&self.released_local_stiffness())
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // trial_disp はグローバル系で蓄積される（BeamElement と同じトライアル追従規約）
        // ため、解放後の局所剛性をグローバルへ回した K で内力を評価する。
        let k = self.inner.axis.to_global(&self.released_local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.inner.trial_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, ctx: &Ctx) {
        self.inner.update_state(du, commit, ctx);
    }

    fn commit_state(&mut self) {
        self.inner.commit_state();
    }

    fn revert_state(&mut self) {
        self.inner.revert_state();
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        self.inner.snapshot_state()
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        self.inner.restore_state(state);
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        self.inner.serialize_checkpoint()
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        self.inner.deserialize_checkpoint(data)
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.inner.mass_matrix(opt)
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        self.inner.geometric_stiffness(n)
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces_released(&arr))
    }
}
