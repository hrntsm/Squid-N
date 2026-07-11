//! 断面算定（許容応力度検定）と二次設計の日本基準実装。
//!
//! 一次設計（許容応力度検定）は RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」の計算方法に準拠する。**マニュアルの章立てと本クレートの
//! モジュール構成を対応させ、原典との照合を容易にしている**:
//!
//! | RESP-D マニュアル 04 断面検定 の節 | モジュール |
//! |---|---|
//! | 材料強度・許容応力度 | [`material`] |
//! | 鉄筋コンクリート造梁の断面検定 | `rc::beam` |
//! | 鉄筋コンクリート造柱の断面検定 | `rc::column` |
//! | 鉄筋コンクリート造耐震壁の断面検定 | [`rc::wall`] |
//! | 鉄筋コンクリート造柱梁接合部の断面検定 | [`rc::joint`] |
//! | 鉄筋コンクリート造梁付着の断面検定 | `rc::bond` |
//! | 鉄筋コンクリート造水平接合面の検討 | [`rc::horizontal_joint`] |
//! | 鉄骨造梁の断面検定 | `steel::beam` |
//! | 鉄骨造柱の断面検定 | `steel::column` |
//! | 鉄骨造パネルゾーンの断面検定 | [`steel::panel_zone`] |
//! | 冷間成形角型鋼管の断面検定 | [`steel::cold_formed`] |
//! | 鉄骨の断面検定における断面性能 | `steel::section` |
//! | 鉄骨ブレースの断面検定 | `steel::brace` |
//! | 鉄骨造柱の座屈長さ係数 K | [`steel::buckling`] |
//! | CFT柱の断面検定 | [`cft`] |
//! | 鉄骨鉄筋コンクリート造梁の断面検定 | `srrc::beam` |
//! | 鉄骨鉄筋コンクリート造柱の断面検定 | `srrc::column` |
//! | 鉄骨鉄筋コンクリート造パネルゾーンの断面検定 | [`srrc::panel_zone`] |
//! | JFEシビル二重鋼管座屈補剛ブレース／日鉄アンボンドブレースの断面検定 | [`brb`] |
//!
//! 節点単位の検定（接合部・パネル・耐震壁・水平接合面）の入力組み立ては
//! [`joint_wiring`] が担い、上記の純関数群を `Model` から呼び出す。
//! 二次設計（保有水平耐力・偏心率・部材ランク等）は `p7` フィーチャ配下の
//! `ds`/`holding_capacity`/`eccentricity` 等に分離している。
pub mod brb;
pub mod cft;
pub mod joint_wiring;
/// 材料強度・許容応力度（RESP-D「材料強度・許容応力度」節）。材種横断の
/// 許容応力度・材料定数を集約する。
pub mod material;
pub mod rc;
pub mod srrc;
pub mod steel;
pub mod wall_opening;

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
pub mod principal_axis;
#[cfg(feature = "p7")]
pub mod rc_capacity;
#[cfg(feature = "p7")]
pub mod story_metrics;

pub use cft::CftDesign;
pub use material::{steel_f_value, steel_f_value_prefix};
pub use rc::RcDesign;
pub use srrc::SrcDesign;
pub use steel::SteelDesign;

use squid_n_core::model::{Material, Section, SteelDesignAttr};

/// 地震時短期の設計用せん断力 QD の決定方法（RESP-D マニュアル 04 断面検定。
/// ユーザー選択により QD1・QD2 のいずれか、または小さいほう）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum QdMethod {
    /// QD1（部材両端の終局曲げモーメントによる値）のみ。
    Qd1,
    /// QD2 = QL + n・QE のみ。
    Qd2,
    /// min(QD1, QD2)（既定）。
    #[default]
    Min,
}

/// 地震時短期の設計用せん断力 QD = min(QD1, QD2) の算定に用いる文脈
/// （RESP-D マニュアル 04 断面検定、RC 梁・柱）。
///
/// - 梁: `QD1 = QL + ΣBMy/l′`、柱: `QD1 = ΣcMy/h′`
/// - `QD2 = QL + n・QE`（`QE` = 当該組合せのせん断力 − 長期せん断力）
///
/// `None`（長期・積雪時・暴風時、または長期結果が未解析）の場合は
/// 解析せん断力をそのまま設計用せん断力とする。積雪時・暴風時の
/// `QD = QL + Qsn／QL + Qw` は組合せ解析の弾性せん断力そのものに一致する
/// ため、本文脈は地震時組合せでのみ与えることを想定する。
pub struct SeismicQd {
    /// 長期（G+P）の部材内力（評価位置, [N,Qy,Qz,Mx,My,Mz]）。
    /// 当該部材の長期組合せ解析結果をそのまま渡す。
    pub long_at: Vec<(f64, [f64; 6])>,
    /// 水平荷重時せん断力の割増係数 n（柱は 1.5 以上。既定 1.5）。
    pub n_factor: f64,
    /// 内法長さ l′／h′（剛域控除後）[mm]。0 以下なら QD1 は省略する。
    pub clear_length: f64,
    /// QD の決定方法。
    pub method: QdMethod,
}

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
    /// 地震時短期の設計用せん断力 QD = min(QD1, QD2) の算定文脈（RC）。
    /// None の場合は解析せん断力をそのまま用いる（従来動作）。
    pub seismic_qd: Option<SeismicQd>,
    /// S 造部材の断面検定属性（継手・スカラップ欠損率、横座屈長さ入力）。
    /// `Model::steel_design_attrs` 由来。None は欠損なし・lb 自動。
    pub steel_attr: Option<SteelDesignAttr>,
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
            seismic_qd: None,
            steel_attr: None,
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
