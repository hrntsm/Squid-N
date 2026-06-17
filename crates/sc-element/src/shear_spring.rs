use crate::behavior::{ElemState, LocalMat, MassOption};
use sc_core::model::{ElementData, Model};
use sc_skeleton::MemberSkeleton;

/// 独立せん断ばね（設計書 §6.3/R26）。
/// 軸–曲げと別系統のせん断変形を表現。スケルトンは P4 sc-skeleton から供給。
pub struct ShearSpring {
    pub skeleton: MemberSkeleton,
}

impl ShearSpring {
    pub fn new(_data: &ElementData, _model: &Model) -> Self {
        ShearSpring {
            skeleton: MemberSkeleton::default(),
        }
    }

    /// 12×12 剛性行列（せん断成分のみ。将来非線形化）
    pub fn stiffness_12x12(&self) -> LocalMat {
        LocalMat::zeros(12)
    }

    pub fn tangent_stiffness(&self, _state: &ElemState) -> LocalMat {
        LocalMat::zeros(12)
    }

    pub fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }
}
