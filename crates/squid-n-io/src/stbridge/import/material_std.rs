//! ST-Bridge のグレード名（`Fc21`・`SN400B`・`SD345` 等）から材料物性を一意に解決する。
//!
//! ST-Bridge 2.0 の `StbModel` は材料テーブル（E・ν・密度）を持たず、材料は断面に付く
//! グレード名の文字列で表す。日本の建築構造材料は法令・JIS で規格化されており、
//! グレード名が決まれば物性（ヤング係数・ポアソン比・単位体積重量・基準強度）は一意に定まる。
//! 本モジュールは代表的な規格材のグレード名から [`StdMat`] を返す（未知の名前は `None`）。

use squid_n_core::section_shape::{concrete_young_modulus, E_STEEL};
use squid_n_core::units::to_internal::mass_density_from_unit_weight_kn_m3;

/// グレード名から解決した標準材料物性（内部単位系 N-mm-s。密度は ton/mm³）。
pub(super) struct StdMat {
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    pub fc: Option<f64>,
    pub fy: Option<f64>,
}

/// 鋼材の単位体積重量 γs = 77 kN/m³（固定荷重）。
const STEEL_UNIT_WEIGHT_KN_M3: f64 = 77.0;
/// 鉄筋コンクリートの単位体積重量 γrc = 24 kN/m³（固定荷重）。
const RC_UNIT_WEIGHT_KN_M3: f64 = 24.0;

/// グレード名から標準材料物性を解決する。認識できない名前は `None`。
///
/// - コンクリート `FcXX`（`Fc21`・`Fc24` 等）→ 圧縮強度 Fc=XX、Ec は RC 規準式で算定。
/// - 構造用鋼材（`SN400B`・`SS400`・`STKR400`・`SM490` 等）→ E=205000、基準強度 F を降伏点に。
/// - 鉄筋（`SD295A`・`SD345`・`SD390` 等）→ E=205000、規格降伏点。
pub(super) fn resolve_grade(name: &str) -> Option<StdMat> {
    let n = name.trim();
    if n.is_empty() {
        return None;
    }
    if let Some(fc) = parse_concrete_fc(n) {
        return Some(StdMat {
            young: concrete_young_modulus(fc),
            poisson: 0.2,
            density: mass_density_from_unit_weight_kn_m3(RC_UNIT_WEIGHT_KN_M3),
            fc: Some(fc),
            fy: None,
        });
    }
    steel_yield_strength(n).map(|fy| StdMat {
        young: E_STEEL,
        poisson: 0.3,
        density: mass_density_from_unit_weight_kn_m3(STEEL_UNIT_WEIGHT_KN_M3),
        fc: None,
        fy: Some(fy),
    })
}

/// コンクリートのグレード名 `FcXX` から設計基準強度 Fc [N/mm²] を取り出す。
/// 大文字小文字を問わず `Fc` で始まり、続く数値を Fc とする（`Fc21`→21）。
fn parse_concrete_fc(name: &str) -> Option<f64> {
    let rest = name
        .strip_prefix("Fc")
        .or_else(|| name.strip_prefix("FC"))
        .or_else(|| name.strip_prefix("fc"))?;
    let digits: String = rest
        .trim()
        .chars()
        .take_while(|c| c.is_ascii_digit() || *c == '.')
        .collect();
    digits.parse::<f64>().ok().filter(|v| *v > 0.0)
}

/// 構造用鋼材・鉄筋のグレード名から規格降伏点（設計用 F [N/mm²]）を返す。
/// 代表的な規格材を網羅する（板厚 40mm 以下の基準強度）。未知は `None`。
fn steel_yield_strength(name: &str) -> Option<f64> {
    // 鉄筋（異形 SD・丸鋼 SR）は末尾の数値が規格降伏点を表す（SD295A・SD345・SR235 等）。
    if let Some(rest) = name.strip_prefix("SD").or_else(|| name.strip_prefix("SR")) {
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if let Ok(v) = digits.parse::<f64>() {
            if v > 0.0 {
                return Some(v);
            }
        }
    }
    // 構造用鋼材は規格ごとに F が定まる（板厚 40mm 以下）。
    let f = match name {
        // 400 N/mm² 級（F=235）
        "SS400" | "SN400A" | "SN400B" | "SN400C" | "SM400A" | "SM400B" | "SM400C" | "SM400"
        | "STK400" | "STKR400" | "STKN400W" | "STKN400B" | "SSC400" | "SWH400" | "BCP235"
        | "BCR235" => 235.0,
        // SS490 級（F=275）
        "SS490" => 275.0,
        // 冷間成形角形鋼管 BCR295（F=295）
        "BCR295" => 295.0,
        // 490 N/mm² 級（F=325）
        "SM490A" | "SM490B" | "SM490C" | "SM490" | "SM490YA" | "SM490YB" | "SN490B" | "SN490C"
        | "STK490" | "STKR490" | "STKN490B" | "BCP325" => 325.0,
        // 520 N/mm² 級（F=355）
        "SM520B" | "SM520C" | "SM520" | "SN490" => 355.0,
        _ => return None,
    };
    Some(f)
}
