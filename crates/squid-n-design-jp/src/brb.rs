//! 座屈補剛ブレース（BRB: Buckling Restrained Brace）の断面検定
//! （RESP-D マニュアル「計算編 04 断面検定」JFEシビル二重鋼管座屈補剛ブレース／
//! 日鉄アンボンドブレースの断面検定）。
//!
//! BRB は芯材が座屈拘束されているため一般の鋼構造ブレースのような座屈を
//! 考慮した許容圧縮応力度 fc（[`crate::steel::steel_fc`]）は用いず、メーカーが
//! 性能評価・大臣認定等で定めた製品固有の許容値を直接用いる。検定項目は
//! 次の 2 つ:
//!
//! 1. 短期許容軸力 ≧ 設計軸力（軸力検定）
//! 2. 座屈長さ ≦ 限界座屈長さ（座屈長さ検定）
//!
//! 座屈長さは芯材の全長からブレース端部の座屈長さ低減距離を差し引いた
//! `Lk = L − 2・L1` で求める（`L1` はブレース上下端それぞれの低減距離
//! `L1上`, `L1下` の平均値 `(L1上+L1下)/2` として入力する）。

use crate::CheckResult;
use squid_n_core::model::BrbAttr;

/// BRB の断面検定（軸力・座屈長さの 2 項目）。
///
/// - 軸力検定比 = `|N_design| / Na`。`Na` は短期許容軸力
///   （[`BrbAttr::allowable_axial_short`]）を用い、`long_term=true`（長期）の
///   場合は `Na = 短期許容軸力 / 1.5` とする（メーカー資料によっては長期の
///   許容軸力が別途定められている場合があり、その際は本近似ではなく
///   メーカー値をそのまま用いるべきである）。
/// - 座屈長さ検定比 = `Lk / 限界座屈長さ`。`Lk = member_length − 2・L1`
///   （[`BrbAttr::length_reduction`]）。
/// - 総合検定比 `ratio` は両者の大きい方（`max`）。
///
/// `n_design` は部材軸力 [N]（符号は問わず絶対値で評価する）、
/// `member_length` は芯材全長 [mm]。
pub fn brb_check(
    attr: &BrbAttr,
    n_design: f64,
    member_length: f64,
    long_term: bool,
) -> CheckResult {
    let na = if long_term {
        attr.allowable_axial_short / 1.5
    } else {
        attr.allowable_axial_short
    };
    let na_safe = if na.abs() > 1e-9 { na } else { 1e-9 };
    let ratio_axial = n_design.abs() / na_safe;

    let lk = (member_length - 2.0 * attr.length_reduction).max(0.0);
    let critical_safe = if attr.critical_length.abs() > 1e-9 {
        attr.critical_length
    } else {
        1e-9
    };
    let ratio_length = lk / critical_safe;

    let ratio = ratio_axial.max(ratio_length);
    let term_label = if long_term { "長期" } else { "短期" };

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis: "座屈補剛ブレース（メーカー許容値による検定）".to_string(),
        detail: format!(
            "{}: |N|={:.4} N, Na={:.4} N, N/Na={:.4} / Lk={:.4} mm, 限界座屈長={:.4} mm, Lk/限界座屈長={:.4}",
            term_label, n_design.abs(), na, ratio_axial, lk, attr.critical_length, ratio_length
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::ElemId;

    fn attr() -> BrbAttr {
        BrbAttr {
            elem: ElemId(1),
            allowable_axial_short: 1_000_000.0,
            critical_length: 4000.0,
            length_reduction: 300.0,
        }
    }

    /// 手計算: N=500,000N（短期）、Na=1,000,000N → N/Na=0.5。
    /// L=5000, L1=300 → Lk=5000-600=4400, 限界=4000 → Lk/限界=1.1。
    /// ratio = max(0.5, 1.1) = 1.1 → NG。
    #[test]
    fn test_brb_check_hand_calc_length_governs() {
        let a = attr();
        let result = brb_check(&a, 500_000.0, 5000.0, false);
        assert!((result.ratio - 1.1).abs() < 1e-6, "ratio={}", result.ratio);
        assert!(!result.ok);
    }

    /// 軸力が支配するケース: N=1,200,000N（短期許容 1,000,000N 超過）、
    /// L=3000（Lk=2400 < 4000 なので座屈長さは余裕あり）。
    /// N/Na=1.2、Lk/限界=2400/4000=0.6 → ratio=1.2 → NG。
    #[test]
    fn test_brb_check_axial_governs_and_fails() {
        let a = attr();
        let result = brb_check(&a, 1_200_000.0, 3000.0, false);
        assert!((result.ratio - 1.2).abs() < 1e-6, "ratio={}", result.ratio);
        assert!(!result.ok);
    }

    /// L1（座屈長さ低減距離）を大きくすると Lk が短くなり、座屈長さ検定比が
    /// 下がる（安全側に効く）ことを確認する。
    #[test]
    fn test_brb_check_length_reduction_effect() {
        let mut a = attr();
        let n = 100_000.0; // 軸力は十分小さく座屈長さ検定が支配するようにする
        let l = 5000.0;

        a.length_reduction = 300.0;
        let ratio_small_l1 = brb_check(&a, n, l, false).ratio;

        a.length_reduction = 800.0;
        let ratio_large_l1 = brb_check(&a, n, l, false).ratio;

        assert!(
            ratio_large_l1 < ratio_small_l1,
            "L1 が大きいほど座屈長さ検定比は小さくなるはず: large={} small={}",
            ratio_large_l1,
            ratio_small_l1
        );
    }

    /// OK 判定: 軸力・座屈長さとも余裕がある場合。
    #[test]
    fn test_brb_check_passes_when_within_limits() {
        let a = attr();
        // Lk = 3000 - 600 = 2400 < 4000（余裕）、N/Na = 400,000/1,000,000=0.4。
        let result = brb_check(&a, 400_000.0, 3000.0, false);
        assert!(result.ratio <= 1.0, "ratio={}", result.ratio);
        assert!(result.ok);
    }

    /// 長期は短期許容値の 1/1.5 を用いる: 同じ N でも長期の方が検定比が
    /// 1.5 倍厳しくなる（座屈長さ検定比は変わらないため軸力支配ケースで検証）。
    #[test]
    fn test_brb_check_long_term_uses_1_5_divisor() {
        let a = attr();
        let n = 300_000.0;
        // Lk=1000-600=400、座屈長さ検定比=400/4000=0.1 は軸力検定比より
        // 十分小さく保ち、軸力検定が支配するようにする。
        let l = 1000.0;
        let ratio_short = brb_check(&a, n, l, false).ratio;
        let ratio_long = brb_check(&a, n, l, true).ratio;
        assert!(
            (ratio_short - 0.3).abs() < 1e-9,
            "ratio_short={}",
            ratio_short
        );
        assert!(
            (ratio_long - 0.45).abs() < 1e-9,
            "ratio_long={}",
            ratio_long
        );
    }
}
