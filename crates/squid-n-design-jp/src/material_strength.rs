//! 材料強度・許容応力度（許容応力度検定で用いる材料の許容応力度・材料定数）。
//!
//! 断面検定で用いる材料の許容応力度・材料定数を、材種横断でまとめる:
//! - コンクリート（許容圧縮・許容せん断・ヤング係数・ヤング係数比 n・付着）
//! - 鉄筋（異形鉄筋の許容引張/圧縮・せん断補強筋・降伏点）
//! - 高強度せん断補強筋（製品別 w_ft・pw 上限表）
//! - 鋼材（F 値・許容引張/圧縮/曲げ/せん断）
//!
//! 準拠する規準:
//! - コンクリート・鉄筋の許容応力度・ヤング係数比: 2010年版 RC 規準・構造規定
//!   （建築基準法施行令 第91条・第90/96条）
//! - 鋼材の F 値・許容応力度: 鋼構造設計規準 1973・構造規定（令90条・令98条/告示）

use crate::LoadTerm;
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;
use squid_n_core::units::ConcreteClass;

// ============================================================================
// 1. コンクリート
// ============================================================================

/// コンクリートの許容圧縮応力度 fc [N/mm²]。
///
/// `長期 = Fc/3`、`短期 = 長期 × 2`（令91条）。
pub fn concrete_allowable_compression(fc: f64, long_term: bool) -> f64 {
    let long = fc / 3.0;
    if long_term {
        long
    } else {
        long * 2.0
    }
}

/// コンクリートの許容せん断応力度 fs [N/mm²]。
///
/// `長期 = min(Fc/30, 0.49 + Fc/100)`、`短期 = 長期 × 1.5`
/// （圧縮の ×2 と異なり、せん断は ×1.5 である点に注意）。
pub fn concrete_allowable_shear(fc: f64, long_term: bool) -> f64 {
    let long = (fc / 30.0).min(0.49 + fc / 100.0);
    if long_term {
        long
    } else {
        long * 1.5
    }
}

/// コンクリート種類による許容応力度の低減係数。
///
/// 軽量コンクリート1種・2種の許容応力度（圧縮・せん断）は普通コンクリートの
/// `0.9 倍`（RC規準・構造規定）。
fn concrete_class_factor(class: ConcreteClass) -> f64 {
    match class {
        ConcreteClass::Normal => 1.0,
        ConcreteClass::Lightweight1 | ConcreteClass::Lightweight2 => 0.9,
    }
}

/// コンクリートの許容圧縮応力度 fc [N/mm²]（コンクリート種類対応版）。
///
/// `fc = concrete_allowable_compression × concrete_class_factor`。
/// 軽量コンクリートの 0.9 倍低減を適用する。`class=Normal` のときは
/// [`concrete_allowable_compression`] と完全に一致する。
pub fn concrete_allowable_compression_class(fc: f64, class: ConcreteClass, long_term: bool) -> f64 {
    concrete_allowable_compression(fc, long_term) * concrete_class_factor(class)
}

/// コンクリートの許容せん断応力度 fs [N/mm²]（コンクリート種類対応版）。
///
/// `fs = concrete_allowable_shear × concrete_class_factor`
/// （軽量コンクリートの 0.9 倍低減を適用）。
pub fn concrete_allowable_shear_class(fc: f64, class: ConcreteClass, long_term: bool) -> f64 {
    concrete_allowable_shear(fc, long_term) * concrete_class_factor(class)
}

/// 断面算定用のヤング係数比 n（Fc に応じた区分値）。
///
/// `Fc≤27→15`, `≤36→13`, `≤48→11`, `≤60→9`, `それ超→7`。
pub fn young_ratio_n(fc: f64) -> f64 {
    if fc <= 27.0 {
        15.0
    } else if fc <= 36.0 {
        13.0
    } else if fc <= 48.0 {
        11.0
    } else if fc <= 60.0 {
        9.0
    } else {
        // 60 < Fc <= 120 の区分値をそれ以上にも代表値として適用する。
        7.0
    }
}

/// コンクリートのヤング係数 Ec [N/mm²]（参考実装）。
///
/// `Ec = 3.35×10⁴・(γ/24)²・(Fc/60)^(1/3)`、γ は単位容積重量 [kN/m³]（既定 23）。
pub fn concrete_young_modulus(fc: f64, gamma_kn_m3: Option<f64>) -> f64 {
    let gamma = gamma_kn_m3.unwrap_or(23.0);
    3.35e4 * (gamma / 24.0).powi(2) * (fc / 60.0).powf(1.0 / 3.0)
}

/// コンクリートの付着許容応力度 fa [N/mm²]（異形鉄筋。RC 規準 1991 方式の
/// τa 検定用）。
///
/// - `長期・上端筋 = min(Fc/15, 0.9 + 2/75・Fc)`
/// - `長期・その他 = min(Fc/10, 1.35 + Fc/25)`
/// - `短期 = 長期 × 1.5`
///
/// 丸鋼（4/100・Fc かつ 0.9 以下等）はモデルに丸鋼の区分が無いため未対応
/// （異形鉄筋のみ）。
pub fn concrete_allowable_bond(fc: f64, top_bar: bool, long_term: bool) -> f64 {
    let long = if top_bar {
        (fc / 15.0).min(0.9 + 2.0 / 75.0 * fc)
    } else {
        (fc / 10.0).min(1.35 + fc / 25.0)
    };
    if long_term {
        long
    } else {
        long * 1.5
    }
}

// ============================================================================
// 2. 鉄筋
// ============================================================================

/// 異形鉄筋の許容引張・圧縮応力度 ft [N/mm²]。
///
/// SD345/SD390/SD490 は径 D29 以上（`dia >= 29.0`）で長期値が低減される
/// （215→195）。USD685（主筋として使う場合の高強度異形棒鋼）は技術評定値
/// どおり長期 215（径によらず、D29 以上の低減対象外）・短期 685 とする。
pub fn rebar_allowable_tension(grade: &str, dia: f64, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 215.0 } else { 685.0 };
    }
    if long_term {
        if g == "SR235" || g == "SR295" {
            155.0
        } else if g.starts_with("SD295") {
            195.0
        } else if g == "SD345" || g == "SD390" || g == "SD490" {
            if dia >= 29.0 {
                195.0
            } else {
                215.0
            }
        } else {
            195.0
        }
    } else if g == "SR235" {
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        490.0
    } else {
        295.0
    }
}

/// せん断補強筋の許容引張応力度 w_ft [N/mm²]。
///
/// USD685 は技術評定値どおり長期 195・短期 590。SD490 短期はせん断のみ
/// F=390 に頭打ち。
pub fn rebar_allowable_shear(grade: &str, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 195.0 } else { 590.0 };
    }
    if long_term {
        if g == "SR235" {
            155.0
        } else {
            195.0
        }
    } else if g == "SR235" {
        // 短期は基準強度 F=235（令90条表。従来はフォールバック 295 に落ちて
        // F 値を 25% 超過する非保守側の誤りだった）。
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        // F 値スケーリング: SD490 短期はせん断のみ F=390 に頭打ち。
        390.0
    } else {
        295.0
    }
}

/// 主筋の降伏点 σy [N/mm²]（終局曲げ ΣMy 算定用）。
///
/// `Material.fy` があればそれを、無ければ材料名（鉄筋グレード名）の数値部
/// （例 "SD345"→345）を、どちらも無ければ 345（SD345 相当）を用いる。
pub fn rebar_sigma_y(mat: &Material) -> f64 {
    if let Some(fy) = mat.fy {
        if fy > 0.0 {
            return fy;
        }
    }
    let digits: String = mat.name.chars().filter(|c| c.is_ascii_digit()).collect();
    digits
        .parse::<f64>()
        .ok()
        .filter(|v| *v > 0.0)
        .unwrap_or(345.0)
}

// ============================================================================
// 3. 高強度せん断補強筋（各製品の大臣認定・技術評定値による許容せん断応力度・pw 上限）
// ============================================================================
//
// `ShearBar.grade` に製品名/規格名（例 "UB785", "KH785", "SBPD1275" 等）が
// 設定されている場合、通常鋼材（SD295〜SD490）の許容せん断応力度表とは別の
// 高強度品用テーブルを用いる。
//
// # 簡略化・注意事項
// - 高強度せん断補強筋の精算式は製品ごとに異なる（例: ウルボン1275 の √ を含む式、
//   KH785 系の βc を用いる式など）が、本実装では未実装。「上記以外の高強度
//   せん断補強筋の場合」に相当する一般化した暫定対応式
//   （[`crate::rc::shear_capacity_high_strength`]）を全高強度製品に一律適用する。
// - pw の上限値は各製品の大臣認定・技術評定値に基づく製品グループごとの定数表とし、
//   グループ判別ができない（未知の高強度品名の）場合は安全側の 0.8% を用いる。

/// 高強度せん断補強筋の製品グループ（pw 上限値の判定用）。
///
/// 製品別 pw 上限表（短期。各製品の技術評定値。2026-07-11 原典図で照合済み）:
/// - ウルボン系（ウルボン785=UB785, ウルボン1275=SBPD1275）・SPR785:
///   1.2%（損傷制御）/1.0%（安全確保）、Fc 非依存。
/// - リバーボン785(KW785)・スミフープ等(KSS785)・HDC685: 0.8%、Fc 非依存。
/// - スーパーフープ KH785: `min(1.2%, 1.0%・Fc/27)`。
/// - スーパーフープ KH685・パワーリング SPR685: `min(1.2%, 1.2%・Fc/27)`。
/// - UHYフープ SHD685・エムケーフープ MK785: 1.2%（損傷制御・安全確保とも）、Fc 非依存。
/// - 上記以外（判別不能な高強度品）: 安全側に 0.8%。
///
/// 長期は全製品 0.6% で共通（[`high_strength_pw_cap`] 側で分岐）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HighStrengthGroup {
    /// ウルボン系（ウルボン785=UB785, ウルボン1275=SBPD1275）・SPR785。
    /// 短期上限 1.2%（損傷制御）/1.0%（安全確保）、Fc 非依存。
    UlbonSeries,
    /// リバーボン785(KW785)・スミフープ等(KSS785)・HDC685。
    /// 短期上限 0.8%（損傷制御・安全確保とも）、Fc 非依存。
    Kw785Series,
    /// スーパーフープ KH785。短期上限 `min(1.2%, 1.0%・Fc/27)`。
    Kh785,
    /// スーパーフープ KH685・パワーリング SPR685。
    /// 短期上限 `min(1.2%, 1.2%・Fc/27)`。
    Kh685Series,
    /// UHYフープ SHD685・エムケーフープ MK785。短期上限 1.2%（損傷制御・
    /// 安全確保とも）、Fc 非依存。
    Shd685OrMk785,
    /// 上記以外（判別不能な高強度品）。安全側に短期上限 0.8% とする。
    Other,
}

/// grade 文字列（大文字化・前方一致）から高強度せん断補強筋の製品グループ
/// を判定する。
pub fn high_strength_group(grade: &str) -> HighStrengthGroup {
    let g = grade.trim().to_uppercase();
    let matches_any = |candidates: &[&str]| {
        candidates
            .iter()
            .any(|c| g.starts_with(c.to_uppercase().as_str()))
    };

    if matches_any(&[
        "UB785",
        "SBPD1275",
        "ｳﾙﾎﾞﾝ785",
        "ｳﾙﾎﾞﾝ1275",
        "ウルボン785",
        "ウルボン1275",
        "SPR785",
    ]) {
        HighStrengthGroup::UlbonSeries
    } else if matches_any(&["KW785", "KSS785", "HDC685"]) {
        HighStrengthGroup::Kw785Series
    } else if matches_any(&["KH785"]) {
        HighStrengthGroup::Kh785
    } else if matches_any(&["KH685", "SPR685"]) {
        HighStrengthGroup::Kh685Series
    } else if matches_any(&["SHD685", "MK785"]) {
        HighStrengthGroup::Shd685OrMk785
    } else {
        HighStrengthGroup::Other
    }
}

/// 高強度せん断補強筋の許容せん断応力度 w_ft [N/mm²]（製品表）。
///
/// 長期は全製品 195。短期は SBPD1275（ウルボン1275）のみ 585、他は全て 590
/// （未知の高強度品名を含む「その他」も 590 とする）。
pub fn high_strength_w_ft(grade: &str, long_term: bool) -> f64 {
    if long_term {
        return 195.0;
    }
    let g = grade.trim().to_uppercase();
    let is_sbpd1275 = g.starts_with("SBPD1275")
        || g.starts_with("ｳﾙﾎﾞﾝ1275".to_uppercase().as_str())
        || g.starts_with("ウルボン1275");
    if is_sbpd1275 {
        585.0
    } else {
        590.0
    }
}

/// 高強度せん断補強筋使用時の pw 上限値（製品グループ・長短期・
/// 損傷制御/安全確保・Fc に応じた表）。
///
/// 長期は全製品 0.6%（Fc 非依存）。`fc` は Fc(raw) [N/mm²]。スーパーフープ
/// KH785/KH685・パワーリング SPR685 は短期上限が Fc に依存する
/// （[`HighStrengthGroup`] の doc 参照）。
pub fn high_strength_pw_cap(grade: &str, term: LoadTerm, damage_control: bool, fc: f64) -> f64 {
    if term == LoadTerm::Long {
        return 0.006;
    }
    match high_strength_group(grade) {
        HighStrengthGroup::UlbonSeries => {
            if damage_control {
                0.012
            } else {
                0.010
            }
        }
        HighStrengthGroup::Kw785Series => 0.008,
        // スーパーフープ KH785: min(1.2%, 1.0%・Fc/27)。
        HighStrengthGroup::Kh785 => (0.012_f64).min(0.010 * fc / 27.0),
        // スーパーフープ KH685・パワーリング SPR685: min(1.2%, 1.2%・Fc/27)。
        HighStrengthGroup::Kh685Series => (0.012_f64).min(0.012 * fc / 27.0),
        // UHYフープ SHD685・エムケーフープ MK785: Fc に依存せず一律 1.2%。
        HighStrengthGroup::Shd685OrMk785 => 0.012,
        HighStrengthGroup::Other => 0.008,
    }
}

// ============================================================================
// 3b. 高強度せん断補強筋の終局検定用 σwy・ν0・pw 上限
//     （「06 終局検定」の製品別表。各製品の技術評定値）
// ============================================================================

/// 終局検定用の高強度せん断補強筋の製品判別（大文字化・前方一致）。
/// 1275 級（ウルボン1275=SBPD1275/1420・リバーボン1275=SBPDN1275/1420）なら true。
fn is_ultimate_hoop_1275_class(grade: &str) -> bool {
    let g = grade.trim().to_uppercase();
    [
        "SBPD1275",
        "SBPDN1275",
        "ウルボン1275",
        "リバーボン1275",
        "ｳﾙﾎﾞﾝ1275",
        "ﾘﾊﾞｰﾎﾞﾝ1275",
    ]
    .iter()
    .any(|c| g.starts_with(c.to_uppercase().as_str()))
}

/// 高強度せん断補強筋の**終局検定用** σwy（せん断補強筋の降伏強度算定用強度）
/// [N/mm²]。製品別の技術評定値の表による。`fc` は Fc(raw) [N/mm²]。
///
/// | 製品 | σwy |
/// |---|---|
/// | ウルボン1275(SBPD1275/1420)・リバーボン1275(SBPDN1275/1420) | min(25·Fc, 1275) |
/// | ウルボン785(UB785)・リバーボン785(KW785)・スミフープ/ストロングフープ/デーフープ(KSS785) | min(25·Fc, 785) |
/// | UHYフープ(SHD685)・パワーリング685(SPR685) | min(25·Fc, 685) |
/// | エヌエスハイデック685H(HDC685) | 685 |
/// | スーパーフープ(KH785) | 25·Fc (Fc＜27.4) / 785 (27.4≦Fc) |
/// | パワーリング785(SPR785) | 25·Fc (Fc＜32.0) / 785 (32.0≦Fc) |
/// | エムケーフープ785(MK785) | 25·Fc (Fc＜31.4) / 785 (31.4≦Fc) |
///
/// 判別できない製品名は `None`（呼び出し側の既定 σwy を用いる）。
/// 許容応力度検定用の w_ft（[`high_strength_w_ft`]）とは別物であることに注意。
pub fn ultimate_hoop_sigma_wy(grade: &str, fc: f64) -> Option<f64> {
    let g = grade.trim().to_uppercase();
    let m = |cands: &[&str]| {
        cands
            .iter()
            .any(|c| g.starts_with(c.to_uppercase().as_str()))
    };
    let v = if is_ultimate_hoop_1275_class(grade) {
        (25.0 * fc).min(1275.0)
    } else if m(&[
        "UB785",
        "KW785",
        "KSS785",
        "ウルボン785",
        "リバーボン785",
        "ｳﾙﾎﾞﾝ785",
        "ﾘﾊﾞｰﾎﾞﾝ785",
        "スミフープ",
        "ストロングフープ",
        "デーフープ",
    ]) {
        (25.0 * fc).min(785.0)
    } else if m(&["SHD685", "SPR685"]) {
        (25.0 * fc).min(685.0)
    } else if m(&["HDC685"]) {
        685.0
    } else if m(&["KH785"]) {
        if fc < 27.4 {
            25.0 * fc
        } else {
            785.0
        }
    } else if m(&["SPR785"]) {
        if fc < 32.0 {
            25.0 * fc
        } else {
            785.0
        }
    } else if m(&["MK785"]) {
        if fc < 31.4 {
            25.0 * fc
        } else {
            785.0
        }
    } else {
        return None;
    };
    Some(v)
}

/// 高強度せん断補強筋使用時のコンクリート圧縮強度有効係数 ν0（終局検定・
/// 塑性理論式用の製品別式）。
///
/// - 1275 級（ウルボン1275・リバーボン1275）: `ν0 = 0.7·(1.0 − Fc/140)`
///   （恒等的に標準式 `0.7 − Fc/200` と一致する。0.7/140 = 1/200）
/// - その他の判別できた高強度製品（785/685 級）: `ν0 = 0.7·(0.7 − Fc/200)`
/// - 判別できない製品名は `None`（標準式 `ν0 = 0.7 − Fc/200` を用いる）
pub fn ultimate_hoop_nu0(grade: &str, fc: f64) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some((0.7 * (1.0 - fc / 140.0)).max(0.0))
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some((0.7 * (0.7 - fc / 200.0)).max(0.0))
    } else {
        None
    }
}

/// 高強度せん断補強筋の**終局検定用** pw 上限（小数）。
///
/// 1275 級は柱かつ Fc＜27 N/mm² のとき 0.8%、それ以外 1.2%。その他の判別できた
/// 高強度製品は 1.2%。判別できない製品名は `None`（上限なし＝pw·σwy ≤ ν·Fc/2 の
/// 一般制約のみ）。許容応力度検定用の [`high_strength_pw_cap`] とは別物。
pub fn ultimate_hoop_pw_cap(grade: &str, fc: f64, is_column: bool) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some(if is_column && fc < 27.0 { 0.008 } else { 0.012 })
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some(0.012)
    } else {
        None
    }
}

// ============================================================================
// 4. 鋼材（鋼構造設計規準 1973・構造規定。板厚 [mm] 区分対応の F 値）
// ============================================================================

/// 板厚 2 区分（`t<=40` / `40<t<=100`）の F 値を返す。
fn bucket2(t: f64, le40: f64, gt40: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else {
        gt40
    }
}

/// 板厚 3 区分（`t<=40` / `40<t<=75` / `75<t<=100`）の F 値を返す（SM520 専用）。
fn bucket3(t: f64, le40: f64, le75: f64, gt75: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else if t <= 75.0 {
        le75
    } else {
        gt75
    }
}

/// 鋼材グレード一覧（前方一致の探索対象。長い記号を優先するため
/// [`steel_f_value_prefix`] 側で文字数最大のものを選ぶ）。
const STEEL_GRADES: &[&str] = &[
    "SS400", "SS490", "SM400", "SM490", "SM520", "SN400", "SN490", "SA440", "STKR400", "STKR490",
    "BCR295", "BCP235", "BCP325",
];

/// 鋼材の F 値 [N/mm²]（完全一致、板厚 [mm] 区分対応）。
///
/// 厚さ 40mm 以下 / 40mm 超 100mm 以下の 2 区分（SM520 のみ 3 区分）。
/// 100mm を超える板厚は規定が無いため、本実装では最終区分値をそのまま用いる
/// （非保守的になり得るため実運用では要確認）。
///
/// 戻り値は F 値。長期許容引張・圧縮・曲げ `ft = F/1.5`、
/// 長期許容せん断 `fs = F/(1.5·√3)`。短期は長期の 1.5 倍（=F, F/√3）。
pub fn steel_f_value(grade: &str, thickness: f64) -> Option<f64> {
    match grade {
        "SS400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SS490" => Some(bucket2(thickness, 275.0, 255.0)),
        "SM400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SM490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SM520" => Some(bucket3(thickness, 355.0, 335.0, 325.0)),
        "SN400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SN490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SA440" => Some(440.0),
        "STKR400" => Some(bucket2(thickness, 235.0, 215.0)),
        "STKR490" => Some(bucket2(thickness, 325.0, 295.0)),
        "BCR295" => Some(295.0),
        "BCP235" => Some(235.0),
        "BCP325" => Some(325.0),
        _ => None,
    }
}

/// 鋼材の F 値 [N/mm²]（前方一致、板厚 [mm] 区分対応）。
///
/// 材料名が板厚区分や強度記号等の接尾辞を伴う場合（例 "SN400B"→235、
/// "SM490A"→325）に対応するため、[`STEEL_GRADES`] の記号と材料名の前方一致で
/// 判定する。複数の記号が前方一致しうる場合は最も長い（＝最も具体的な）
/// ものを優先する（例: "SN490B" は "SN400" ではなく "SN490" に一致する）。
pub fn steel_f_value_prefix(name: &str, thickness: f64) -> Option<f64> {
    STEEL_GRADES
        .iter()
        .filter(|g| name.starts_with(*g))
        .max_by_key(|g| g.len())
        .and_then(|g| steel_f_value(g, thickness))
}

/// F 値の板厚区分判定に用いる最大板厚 [mm]。
///
/// `Section.shape` があれば形状の最大板厚、無ければ `Section.thickness`、
/// いずれも無ければ 40mm 以下区分として扱う。
pub fn plate_thickness(sec: &Section) -> f64 {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return web_thick.max(flange_thick),
            SectionShape::SteelBox { thick, .. }
            | SectionShape::SteelAngle { thick, .. }
            | SectionShape::SteelPipe { thick, .. }
            | SectionShape::CftBox { thick, .. }
            | SectionShape::CftPipe { thick, .. } => return thick,
            SectionShape::SrcRect {
                steel_web_thick,
                steel_flange_thick,
                ..
            } => return steel_web_thick.max(steel_flange_thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. } => {}
        }
    }
    sec.thickness.unwrap_or(40.0)
}

/// 長期・短期許容引張／曲げ（座屈無視）応力度 ft [N/mm²]。
///
/// `長期 = F/1.5`、`短期 = F`。
pub fn steel_ft(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / 1.5,
        LoadTerm::Short => f,
    }
}

/// 長期・短期許容せん断応力度 fs [N/mm²]。
///
/// `長期 = F/(1.5·√3)`、`短期 = F/√3`。
pub fn steel_fs(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / (1.5 * 3.0_f64.sqrt()),
        LoadTerm::Short => f / 3.0_f64.sqrt(),
    }
}

/// 限界細長比 `Λ = 1500/√(F/1.5)`。
pub fn big_lambda(f: f64) -> f64 {
    1500.0 / (f / 1.5).sqrt()
}

/// 長期・短期許容圧縮応力度 fc [N/mm²]（座屈考慮、鋼構造設計規準 1973）。
///
/// 有効細長比 `λ = lk/i`、限界細長比 `Λ = 1500/√(F/1.5)`、
/// 安全率 `ν = 3/2 + (2/3)(λ/Λ)²` として:
/// - `λ ≤ Λ`: `fcL = F·(1 − 0.4(λ/Λ)²) / ν`
/// - `λ > Λ`: `fcL = (18/65)·F/(λ/Λ)²`
///
/// 短期は長期の 1.5 倍。`λ=0` のとき `fcL = F/1.5`（= ft 長期）と一致し、
/// `λ=Λ` で両分岐は連続する（`fcL = (18/65)F`）。
pub fn steel_fc(f: f64, lambda: f64, term: LoadTerm) -> f64 {
    let big_l = big_lambda(f);
    let r = if big_l > 1e-9 { lambda / big_l } else { 0.0 };
    let fc_long = if lambda <= big_l {
        let nu = 1.5 + (2.0 / 3.0) * r * r;
        f * (1.0 - 0.4 * r * r) / nu
    } else {
        (18.0 / 65.0) * f / (r * r)
    };
    match term {
        LoadTerm::Long => fc_long,
        LoadTerm::Short => fc_long * 1.5,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------
    // コンクリート
    // ------------------------------------------------------------------

    #[test]
    fn test_concrete_shear_long_term_min_branch() {
        // Fc=21: Fc/30=0.7, 0.49+Fc/100=0.7 で同値。
        assert!((concrete_allowable_shear(21.0, true) - 0.7).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_shear_short_term_is_1_5x_long() {
        let long = concrete_allowable_shear(24.0, true);
        let short = concrete_allowable_shear(24.0, false);
        assert!((short - long * 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_compression_short_is_2x_long() {
        let long = concrete_allowable_compression(24.0, true);
        assert!((long - 8.0).abs() < 1e-9);
        assert!((concrete_allowable_compression(24.0, false) - 16.0).abs() < 1e-9);
    }

    #[test]
    fn test_lightweight_concrete_is_0_9x() {
        let normal = concrete_allowable_shear_class(24.0, ConcreteClass::Normal, false);
        let light = concrete_allowable_shear_class(24.0, ConcreteClass::Lightweight1, false);
        assert!((light - normal * 0.9).abs() < 1e-12);
        let normal_c = concrete_allowable_compression_class(24.0, ConcreteClass::Normal, true);
        let light_c = concrete_allowable_compression_class(24.0, ConcreteClass::Lightweight2, true);
        assert!((light_c - normal_c * 0.9).abs() < 1e-12);
    }

    #[test]
    fn test_young_ratio_n_buckets() {
        assert_eq!(young_ratio_n(24.0), 15.0);
        assert_eq!(young_ratio_n(27.0), 15.0);
        assert_eq!(young_ratio_n(30.0), 13.0);
        assert_eq!(young_ratio_n(42.0), 11.0);
        assert_eq!(young_ratio_n(60.0), 9.0);
        assert_eq!(young_ratio_n(80.0), 7.0);
    }

    #[test]
    fn test_concrete_allowable_bond_table() {
        // Fc=24 上端筋: min(24/15, 0.9+2/75×24) = min(1.6, 1.54) = 1.54
        assert!((concrete_allowable_bond(24.0, true, true) - 1.54).abs() < 1e-9);
        // Fc=24 その他: min(24/10, 1.35+24/25) = min(2.4, 2.31) = 2.31
        assert!((concrete_allowable_bond(24.0, false, true) - 2.31).abs() < 1e-9);
        assert!(
            (concrete_allowable_bond(24.0, true, false)
                - concrete_allowable_bond(24.0, true, true) * 1.5)
                .abs()
                < 1e-9
        );
        // 低強度側の分岐: Fc=15 上端筋 min(1.0, 1.3) = 1.0（Fc/15 側が支配）。
        assert!((concrete_allowable_bond(15.0, true, true) - 1.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 鉄筋
    // ------------------------------------------------------------------

    #[test]
    fn test_rebar_tension_sd345_d29_reduction() {
        assert!((rebar_allowable_tension("SD345", 25.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 29.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 25.0, false) - 345.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_usd685() {
        assert!((rebar_allowable_tension("USD685", 32.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("USD685", 32.0, false) - 685.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("USD685", true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("USD685", false) - 590.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_sigma_y_sources() {
        let mut m = Material {
            concrete_class: Default::default(),
            id: squid_n_core::ids::MaterialId(0),
            name: "SD390".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        assert!((rebar_sigma_y(&m) - 390.0).abs() < 1e-9);
        m.fy = Some(400.0);
        assert!((rebar_sigma_y(&m) - 400.0).abs() < 1e-9);
        m.fy = None;
        m.name = "unknown".to_string();
        assert!((rebar_sigma_y(&m) - 345.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 高強度せん断補強筋
    // ------------------------------------------------------------------

    #[test]
    fn test_high_strength_pw_cap_groups() {
        // ウルボン系(UB785)・SPR785: 短期 1.2%(損傷制御)/1.0%(安全確保)。
        assert!((high_strength_pw_cap("UB785", LoadTerm::Short, true, 24.0) - 0.012).abs() < 1e-9);
        assert!((high_strength_pw_cap("UB785", LoadTerm::Short, false, 24.0) - 0.010).abs() < 1e-9);
        // KW785/KSS785/HDC685: 0.8%。
        assert!((high_strength_pw_cap("KW785", LoadTerm::Short, true, 24.0) - 0.008).abs() < 1e-9);
        // SHD685・MK785: 1.2% 固定。
        assert!((high_strength_pw_cap("SHD685", LoadTerm::Short, true, 24.0) - 0.012).abs() < 1e-9);
        assert!((high_strength_pw_cap("MK785", LoadTerm::Short, false, 24.0) - 0.012).abs() < 1e-9);
        // KH785: min(1.2%, 1.0%・Fc/27)。Fc=24 → 0.010×24/27≈0.008889。
        assert!(
            (high_strength_pw_cap("KH785", LoadTerm::Short, false, 24.0) - 0.010 * 24.0 / 27.0)
                .abs()
                < 1e-9
        );
        // KH685/SPR685: min(1.2%, 1.2%・Fc/27)。Fc=36 → 頭打ち 1.2%。
        assert!((high_strength_pw_cap("KH685", LoadTerm::Short, true, 36.0) - 0.012).abs() < 1e-9);
        assert!(
            (high_strength_pw_cap("SPR685", LoadTerm::Short, false, 24.0) - 0.012 * 24.0 / 27.0)
                .abs()
                < 1e-9
        );
        // 未知品・長期。
        assert!((high_strength_pw_cap("XYZ999", LoadTerm::Short, true, 24.0) - 0.008).abs() < 1e-9);
        assert!((high_strength_pw_cap("UB785", LoadTerm::Long, true, 24.0) - 0.006).abs() < 1e-9);
    }

    #[test]
    fn test_high_strength_w_ft() {
        assert!((high_strength_w_ft("SBPD1275", false) - 585.0).abs() < 1e-9);
        assert!((high_strength_w_ft("UB785", false) - 590.0).abs() < 1e-9);
        assert!((high_strength_w_ft("KH785", true) - 195.0).abs() < 1e-9);
    }

    #[test]
    fn test_ultimate_hoop_sigma_wy_products() {
        // min(25Fc, 上限) 系: Fc=24 → 25·24=600 が支配。Fc=60 → 上限が支配。
        assert!((ultimate_hoop_sigma_wy("SBPD1275", 24.0).unwrap() - 600.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SBPD1275", 60.0).unwrap() - 1275.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SBPDN1275/1420", 60.0).unwrap() - 1275.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("UB785", 60.0).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("KSS785", 24.0).unwrap() - 600.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SHD685", 60.0).unwrap() - 685.0).abs() < 1e-9);
        // HDC685 は Fc 非依存の 685。
        assert!((ultimate_hoop_sigma_wy("HDC685", 24.0).unwrap() - 685.0).abs() < 1e-9);
        // しきい値切替系: KH785 は Fc=27.4 で 25Fc→785 に跳ぶ。
        assert!((ultimate_hoop_sigma_wy("KH785", 27.0).unwrap() - 675.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("KH785", 27.4).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SPR785", 31.0).unwrap() - 775.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SPR785", 32.0).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("MK785", 31.0).unwrap() - 775.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("MK785", 31.4).unwrap() - 785.0).abs() < 1e-9);
        // 未知製品は None。
        assert!(ultimate_hoop_sigma_wy("SD295", 24.0).is_none());
        assert!(ultimate_hoop_sigma_wy("XYZ999", 24.0).is_none());
    }

    #[test]
    fn test_ultimate_hoop_nu0_and_pw_cap() {
        // 1275 級: ν0 = 0.7·(1.0−Fc/140)。
        let nu = ultimate_hoop_nu0("SBPD1275", 24.0).unwrap();
        assert!((nu - 0.7 * (1.0 - 24.0 / 140.0)).abs() < 1e-12);
        // 785/685 級: ν0 = 0.7·(0.7−Fc/200)。
        let nu2 = ultimate_hoop_nu0("UB785", 24.0).unwrap();
        assert!((nu2 - 0.7 * (0.7 - 24.0 / 200.0)).abs() < 1e-12);
        assert!(ultimate_hoop_nu0("SD295", 24.0).is_none());
        // pw 上限: 1275 級の柱かつ Fc<27 のみ 0.8%、それ以外 1.2%。
        assert!((ultimate_hoop_pw_cap("SBPD1275", 24.0, true).unwrap() - 0.008).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("SBPD1275", 24.0, false).unwrap() - 0.012).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("SBPD1275", 30.0, true).unwrap() - 0.012).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("KH785", 24.0, true).unwrap() - 0.012).abs() < 1e-12);
        assert!(ultimate_hoop_pw_cap("SD295", 24.0, true).is_none());
    }

    // ------------------------------------------------------------------
    // 鋼材
    // ------------------------------------------------------------------

    #[test]
    fn test_f_value_buckets() {
        assert!((steel_f_value("SS400", 40.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value("SS400", 40.1).unwrap() - 215.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 75.0).unwrap() - 335.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 76.0).unwrap() - 325.0).abs() < 1e-9);
    }

    #[test]
    fn test_f_value_prefix_longest_match() {
        assert!((steel_f_value_prefix("SN400B", 30.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value_prefix("SN490B", 30.0).unwrap() - 325.0).abs() < 1e-9);
        assert!(steel_f_value_prefix("UNKNOWN999", 30.0).is_none());
    }

    #[test]
    fn test_steel_ft_fs_short_is_1_5x() {
        assert!((steel_ft(235.0, LoadTerm::Long) - 235.0 / 1.5).abs() < 1e-9);
        assert!((steel_ft(235.0, LoadTerm::Short) - 235.0).abs() < 1e-9);
        assert!(
            (steel_fs(235.0, LoadTerm::Short) - steel_fs(235.0, LoadTerm::Long) * 1.5).abs() < 1e-9
        );
    }

    #[test]
    fn test_steel_fc_continuous_at_lambda() {
        // λ=0 で fc = F/1.5（=ft長期）、λ=Λ で両分岐が連続。
        let f = 235.0;
        assert!((steel_fc(f, 0.0, LoadTerm::Long) - f / 1.5).abs() < 1e-6);
        let big_l = big_lambda(f);
        let below = steel_fc(f, big_l - 1e-6, LoadTerm::Long);
        let above = steel_fc(f, big_l + 1e-6, LoadTerm::Long);
        assert!((below - above).abs() < 1e-3);
        assert!((below - (18.0 / 65.0) * f).abs() < 1e-2);
    }
}
