//! 断面算定（許容応力度検定）と二次設計の日本基準実装。
//!
//! 一次設計（許容応力度検定）は RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」の計算方法に準拠する:
//! - 鋼構造: `steel`（S梁・S柱・鉄骨ブレース、鋼構造設計規準 1973）
//! - RC 造: `rc`（RC梁・RC柱、RC規準 1999/2010・構造規定）
pub mod joint;
pub mod joint_wiring;
pub mod rc;
pub mod steel;

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
#[cfg(feature = "p7")]
pub mod rc_capacity;

pub use rc::RcDesign;
pub use steel::{steel_f_value, steel_f_value_prefix, SteelDesign};

use squid_n_core::model::{Material, Section};

/// ある評価位置 1 点の内力。
///
/// 単位は以下に統一する（プログラム全体と共通）:
/// - `n`: 軸力 [N]（**引張を正、圧縮を負**とする）
/// - `qy`, `qz`: 部材局所 y/z 方向のせん断力 [N]
/// - `my`, `mz`: 部材局所 y/z 軸まわりの曲げモーメント [N·mm]
///   （`mz` が強軸まわり＝`Section.iy` に対応する曲げ、`my` が弱軸まわり）
/// - `pos`: 部材軸方向の無次元位置 (0.0=始端, 1.0=終端)
///
/// 許容応力度は [N/mm²] で与えられるため、応力算定は
/// `σ = M[N·mm] / Z[mm³]` のように単位を N·mm 系で揃えること。
pub struct MemberForcesAt {
    pub pos: f64,
    pub n: f64,
    pub qy: f64,
    pub qz: f64,
    pub my: f64,
    pub mz: f64,
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

/// 部材種別。検定式の選択に用いる（RESP-D マニュアル 04 断面検定）。
///
/// - `Beam`: 梁（強軸曲げ＋せん断。鋼は横座屈を考慮した fb）
/// - `Column`: 柱（軸力＋二軸曲げの複合検定＋せん断）
/// - `Brace`: ブレース（軸力のみ。圧縮は座屈を考慮した fc）
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemberKind {
    Beam,
    Column,
    Brace,
}

/// 検定コンテキスト（部材単位で一定の情報）。
pub struct DesignCtx {
    pub term: LoadTerm,
    pub kind: MemberKind,
    /// 部材長 [mm]。座屈長さ lk・横座屈長さ lb の既定値として用いる。
    pub length: f64,
    /// 圧縮フランジの支点間距離（横座屈長さ）lb [mm]。None なら `length`。
    pub lb: Option<f64>,
    /// 座屈長さ lk [mm]。None なら `length`（座屈長さ係数 K=1 相当）。
    pub lk: Option<f64>,
    /// せん断スパン比 M/(Q·d) 算定用の部材代表値 `(|M|max, 対応する |Q|)`。
    /// マニュアルの規定（モーメントが最大となる検定位置の値を採用）に対応する。
    /// None の場合は当該評価位置の |M|, |Q| を使う。
    pub shear_span: Option<(f64, f64)>,
    /// RC 短期許容せん断力で「損傷制御のための検討」式（2/3·α）を使うか。
    /// false の場合は「安全確保のための検討」式。
    pub rc_damage_control: bool,
    /// 部材両端の強軸まわり曲げモーメント `(M_i端, M_j端)` [N·mm]（符号付き）。
    /// 鋼の横座屈修正係数 C（複曲率正/単曲率負）とたわみ検定に用いる。
    /// None の場合は C=1.0（安全側）となり、たわみ検定は省略される。
    pub end_moments_z: Option<(f64, f64)>,
    /// 部材中央（pos=0.5）の強軸まわり曲げモーメント [N·mm]（符号付き）。
    /// たわみ検定の単純梁中央モーメント M0 の復元と、横座屈 C 係数の
    /// 「中央部の曲げモーメントが端部より大きい場合 C=1.0」判定に用いる。
    pub mid_moment_z: Option<f64>,
}

impl Default for DesignCtx {
    fn default() -> Self {
        DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            lb: None,
            lk: None,
            shear_span: None,
            rc_damage_control: true,
            end_moments_z: None,
            mid_moment_z: None,
        }
    }
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
