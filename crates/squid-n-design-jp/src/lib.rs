//! 断面算定（許容応力度検定）と二次設計の日本基準実装。
//!
//! 一次設計（許容応力度検定）は令82条および各構造規準（RC規準・鋼構造設計
//! 規準・SRC規準）に準拠し、構造種別ごとにモジュールへ分割する
//! （材種ごとに `rc`/`steel`/`cft`/`srrc`、材料強度・許容応力度は
//! `material_strength`、節点単位の検定の入力組み立ては `joint_wiring`）。
//!
//! 二次設計（保有水平耐力計算）は `p7` フィーチャ配下の [`secondary`] モジュール
//! （部材ランク・層 Ds・保有水平耐力・剛性率・偏心率・主軸）に分離する。
pub mod brb;
pub mod cft;
pub mod floor;
/// 免震支承材のマルチシアスプリング低減率・摩擦力（各免震部材指針）。
pub mod isolator;
pub mod joint_wiring;
/// 材料強度・許容応力度（各構造規準の材料強度・許容応力度）。材種横断の
/// 許容応力度・材料定数を集約する。構成則モデルの `squid-n-material`
/// クレートとは別物（本モジュールは設計規準の許容応力度）。
pub mod material_strength;
/// 数量積算（部位別の概算数量集計）。
pub mod quantity;
pub mod rc;
pub mod srrc;
pub mod steel;
/// 終局検定（靭性保証型指針・技術基準解説書）。荷重増分解析後の各部材の終局せん断強度
/// （塑性理論式）・付着割裂耐力・軸終局耐力に対する余裕度を検定する。
pub mod ultimate;
pub mod wall_opening;

// 容量スペクトル法（限界耐力計算）は P12 のスコープ。P7 とは別フェーズなので p12 で分離。
#[cfg(feature = "p12")]
pub mod capacity_spectrum;
#[cfg(feature = "p7")]
pub mod secondary;

pub use cft::CftDesign;
pub use material_strength::{steel_f_value, steel_f_value_prefix};
pub use rc::RcDesign;
pub use srrc::SrcDesign;
pub use steel::SteelDesign;

use squid_n_core::model::{Material, Section, SteelDesignAttr};

/// 鋼梁の許容曲げ応力度 fb の算定式（旧基準 1973 / 新基準 AIJ-ASD19）。
///
/// - `Old`: 鋼構造設計規準 1973（`steel_fb_h`）。既定値。
/// - `New`: AIJ 鋼構造許容応力度設計規準 2019（`steel_fb_h_new` 相当。
///   限界細長比 λb による全塑性・非弾性・弾性の 3 領域式）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SteelFbRule {
    #[default]
    Old,
    New,
}

/// 地震時短期の設計用せん断力 QD の決定方法（RC規準。
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
/// （RC規準、RC 梁・柱）。
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

/// 部材種別。検定式の選択に用いる（RC規準・鋼構造設計規準の断面検定）。
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
    /// 強軸まわり座屈長さ lk_y [mm]（断面二次半径 i_y=√(Iy/A) と対）。
    /// None なら `length`（座屈長さ係数 K=1 相当）。[`effective_slenderness`] 参照。
    pub lk_y: Option<f64>,
    /// 弱軸まわり座屈長さ lk_z [mm]（断面二次半径 i_z=√(Iz/A) と対）。
    /// None なら `length`（座屈長さ係数 K=1 相当）。[`effective_slenderness`] 参照。
    pub lk_z: Option<f64>,
    /// せん断スパン比 M/(Q·d) 算定用の部材代表値 `(|Mz|max, 対応する |Qy|)`
    /// （強軸曲げ方向）。「モーメントが最大となる検定位置の値を採用」の規定に
    /// 対応する。None の場合は当該評価位置の |Mz|, |Qy| を使う。
    pub shear_span: Option<(f64, f64)>,
    /// せん断スパン比の弱軸曲げ方向代表値 `(|My|max, 対応する |Qz|)`。
    /// 柱の二方向せん断検定で qz 方向の α に用いる（加力方向ごとに
    /// せん断スパン比を評価する規定）。None の場合は当該評価位置の
    /// |My|, |Qz| を使う（強軸側の値を流用しない）。
    pub shear_span_y: Option<(f64, f64)>,
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
    /// 鋼梁の許容曲げ応力度 fb の算定式（旧基準 / 新基準）。既定は `Old`
    /// （従来挙動を維持）。
    pub steel_fb_rule: SteelFbRule,
}

impl Default for DesignCtx {
    fn default() -> Self {
        DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            lb: None,
            lk_y: None,
            lk_z: None,
            shear_span: None,
            shear_span_y: None,
            rc_damage_control: true,
            end_moments_z: None,
            mid_moment_z: None,
            seismic_qd: None,
            steel_attr: None,
            steel_fb_rule: SteelFbRule::default(),
        }
    }
}

/// 強軸・弱軸の座屈長さを個別に扱った有効細長比 λ の算定
/// （鋼構造設計規準・SRC規準の柱・梁・ブレース・CFT 柱で共用）。
///
/// `λ = max(λ_y, λ_z)`（`λ_y = lk_y/i_y`、`λ_z = lk_z/i_z`）。
/// - `i_y = √(max(Iy,0)/A)`、`i_z = √(max(Iz,0)/A)`（`iy`/`iz`/`area` は
///   呼び出し側が渡す断面二次モーメント・断面積。CFT 柱は鋼管単体の値を渡す
///   ことで従来の「鋼管単体の i で評価」の流儀を維持できる）。
/// - `lk_y`/`lk_z` が `None` の場合は `length` を用いる（座屈長さ係数 K=1 相当）。
/// - 各軸の `i` が極小、または対応する座屈長さが 0 以下の場合は、その軸の
///   λ を 0（座屈無視）とする。
///
/// 両軸とも `None`（=`length` 共通）の場合、`λ = max(length/i_y, length/i_z)
/// = length/min(i_y, i_z)` となり、軸別座屈長さ導入前の `λ = lk/i_min`
/// （`i_min = √(min(Iy,Iz)/A)`）と一致する。
pub fn effective_slenderness(
    iy: f64,
    iz: f64,
    area: f64,
    length: f64,
    lk_y: Option<f64>,
    lk_z: Option<f64>,
) -> f64 {
    let axis_lambda = |i_sq: f64, lk: Option<f64>| -> f64 {
        let i = if area > 1e-9 {
            (i_sq.max(0.0) / area).sqrt()
        } else {
            0.0
        };
        let lk_val = lk.unwrap_or(length);
        if i > 1e-9 && lk_val > 1e-9 {
            lk_val / i
        } else {
            0.0
        }
    };
    axis_lambda(iy, lk_y).max(axis_lambda(iz, lk_z))
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
