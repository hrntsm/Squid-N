pub mod allowable_stress;

// 容量スペクトル法（限界耐力計算）は P12 のスコープ。P7 とは別フェーズなので p12 で分離。
#[cfg(feature = "p12")]
pub mod capacity_spectrum;
#[cfg(feature = "p7")]
pub mod ds;
#[cfg(feature = "p7")]
pub mod eccentricity;
#[cfg(feature = "p7")]
pub mod holding_capacity;
#[cfg(feature = "p7")]
pub mod panel_shear;

pub use allowable_stress::*;

use sc_core::model::{Material, Section};

/// ある評価位置 1 点の内力。
///
/// 単位は以下に統一する（プログラム全体と共通）:
/// - `n`: 軸力 [N]
/// - `q`: せん断力 [N]
/// - `m`: 曲げモーメント [N·mm]
/// - `pos`: 部材軸方向の無次元位置 (0.0=始端, 1.0=終端)
///
/// 許容応力度は [N/mm²] で与えられるため、応力算定は
/// `σ = M[N·mm] / Z[mm³]` のように単位を N·mm 系で揃えること。
pub struct MemberForcesAt {
    pub pos: f64,
    pub n: f64,
    pub q: f64,
    pub m: f64,
}

pub struct CheckResult {
    pub ratio: f64,
    pub ok: bool,
    pub basis: String,
    pub detail: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadTerm {
    Long,
    Short,
}

pub struct DesignCtx {
    pub term: LoadTerm,
}

pub trait DesignCheck {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult;
}
