//! 冷間成形角形鋼管柱の柱梁耐力比チェック（許容応力度検定。
//! 冷間成形角形鋼管設計・施工マニュアルの冷間成形角形鋼管柱部分に準拠）。
//!
//! # 位置付け
//! このモジュールは `squid_n_core`（モデル）や要素（`squid_n_element`）に依存せず、
//! 呼び出し側（節点まわりの応力集計・断面形状の解決を担当する別モジュール）が
//! 用意した数値入力を受け取る**純関数**として実装する。
//!
//! 準拠する規準: 2008年版「冷間成形角形鋼管の柱に用いる角形鋼管設計・施工
//! マニュアル」
//!
//! # 式の簡略化について（重要）
//! 冷間成形角形鋼管の耐力低減係数 ν、パネル耐力 Mpp の軸力依存項は同マニュアルの
//! 分岐式をそのまま用いるが、退化域（|n|≧1 等）は安全側にクランプする処理を
//! 追加している。

use crate::CheckResult;

/// 冷間成形角形鋼管柱の柱梁耐力比チェックの入力。
pub struct ColdFormedInput {
    /// 上柱の塑性断面係数 Zp [mm³]。
    pub zp_col_upper: f64,
    /// 下柱の塑性断面係数 Zp [mm³]。
    pub zp_col_lower: f64,
    /// 上柱の基準強度 F [N/mm²]。
    pub f_col_upper: f64,
    /// 下柱の基準強度 F [N/mm²]。
    pub f_col_lower: f64,
    /// 上柱の軸力比 n = N/(F・A)（存在軸力 N = NL + 1.5・NE は呼び出し側で算定）。
    pub n_upper: f64,
    /// 下柱の軸力比 n = N/(F・A)。
    pub n_lower: f64,
    /// 梁の全塑性モーメント和 Σ(Fyb・Zpb) [N·mm]。
    pub sum_beam_mp: f64,
    /// パネル耐力 Mpp [N·mm]。0（または負）の場合は要求値の min 判定の対象外とする。
    pub panel_mpp: f64,
}

/// 冷間成形角形鋼管柱の柱梁耐力比チェック
/// （2008年版 冷間成形角形鋼管の柱に用いる角形鋼管設計・施工マニュアル）。
///
/// ## 柱の耐力低減係数 ν
/// - `n ≤ 0.5`: `ν = 1 − 4n²/3`
/// - `n > 0.5`: `ν = 4(1 − n)/3`
///
/// ここで `n` は軸力比の絶対値。`n ≥ 1`（軸力が全塑性軸耐力以上）の場合は
/// 柱の曲げ耐力に余裕がないとみなし `ν = 0` にクランプする。
///
/// ## 柱梁耐力比
/// `ΣMpc = νu・Fu・Zpu + νl・Fl・Zpl`
///
/// 要求値 = `min(1.5・ΣMpb, 1.3・Mpp)`。`Mpp ≤ 0`（未評価・対象外）の場合は
/// `1.5・ΣMpb` のみを要求値とする。
///
/// 検定比 = `要求値 / ΣMpc`（1.0 以下で OK）。
///
/// **注記**: 同マニュアルでは、この検定を満たさない（NG の）場合でも、他の多くの
/// 保有耐力接合検定のように部材耐力を直接低減する再計算は行わない
/// （柱梁耐力比が確保できない状況として設計者に警告する位置付け）。
/// 本関数もその方針に従い、`ok=false` を返すのみで耐力の再計算は行わない。
pub fn cold_formed_column_ratio_check(inp: &ColdFormedInput) -> CheckResult {
    let nu_upper = nu_factor(inp.n_upper);
    let nu_lower = nu_factor(inp.n_lower);

    let sum_mpc = nu_upper * inp.f_col_upper * inp.zp_col_upper
        + nu_lower * inp.f_col_lower * inp.zp_col_lower;

    let beam_req = 1.5 * inp.sum_beam_mp;
    let required = if inp.panel_mpp > 0.0 {
        beam_req.min(1.3 * inp.panel_mpp)
    } else {
        beam_req
    };

    let ratio = if sum_mpc > 0.0 {
        required / sum_mpc
    } else {
        f64::INFINITY
    };
    let ok = ratio <= 1.0;

    let basis =
        "2008年版冷間成形角形鋼管設計・施工マニュアル 柱梁耐力比（NG時も耐力低減なし）".to_string();
    let detail = format!(
        "nu_upper={:.4}, nu_lower={:.4}, SumMpc={:.1} N*mm, 1.5*SumMpb={:.1} N*mm, 1.3*Mpp={:.1} N*mm, required={:.1} N*mm, ratio={:.4}",
        nu_upper,
        nu_lower,
        sum_mpc,
        beam_req,
        1.3 * inp.panel_mpp,
        required,
        ratio
    );

    CheckResult {
        ratio,
        ok,
        basis,
        detail,
    }
}

/// 柱の耐力低減係数 ν（`n` は符号付きでよく、内部で絶対値を用いる）。
fn nu_factor(n: f64) -> f64 {
    let n = n.abs();
    if n >= 1.0 {
        0.0
    } else if n <= 0.5 {
        1.0 - 4.0 * n * n / 3.0
    } else {
        4.0 * (1.0 - n) / 3.0
    }
}

/// 角形鋼管の塑性断面係数 Zp [mm³]（中空矩形断面、軸まわり曲げ）。
///
/// `Zp = b・h²/4 − (b − 2t)・(h − 2t)²/4`
///
/// - `h`: 曲げ軸方向のせい [mm]
/// - `b`: 曲げ軸と直交する方向の幅 [mm]
/// - `t`: 管厚 [mm]
pub fn box_zp(h: f64, b: f64, t: f64) -> f64 {
    b * h * h / 4.0 - (b - 2.0 * t) * (h - 2.0 * t) * (h - 2.0 * t) / 4.0
}

/// パネル耐力 Mpp [N·mm]（角形鋼管柱パネルの全塑性モーメント、軸力の影響を考慮）。
///
/// `Ve = 2・dc・db・tp`
/// - `n ≤ 0.5`: `Mpp = Ve・F/√3`
/// - `n > 0.5`: `Mpp = Ve・F/√3・2・√(n・(1 − n))`
///
/// `n` は軸力比（絶対値を用いる）。`n・(1 − n)` が負になりうる `n > 1` の
/// 領域は安全側として 0 にクランプする。
pub fn panel_mpp(dc: f64, db: f64, tp: f64, f: f64, n: f64) -> f64 {
    let n = n.abs();
    let ve = 2.0 * dc * db * tp;
    let base = ve * f / 3f64.sqrt();
    if n <= 0.5 {
        base
    } else {
        let inner = (n * (1.0 - n)).max(0.0);
        base * 2.0 * inner.sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_formed_nu_branches_at_n_0_3_and_0_7() {
        let nu_03 = nu_factor(0.3);
        let expected_03 = 1.0 - 4.0 * 0.3 * 0.3 / 3.0;
        assert!((nu_03 - expected_03).abs() < 1e-9);

        let nu_07 = nu_factor(0.7);
        let expected_07 = 4.0 * (1.0 - 0.7) / 3.0;
        assert!((nu_07 - expected_07).abs() < 1e-9);

        // 連続性の確認（境界 n=0.5 付近で急変しない）。
        assert!((nu_factor(0.5) - (1.0 - 4.0 * 0.25 / 3.0)).abs() < 1e-9);
    }

    #[test]
    fn cold_formed_nu_clamped_at_and_above_one() {
        assert_eq!(nu_factor(1.0), 0.0);
        assert_eq!(nu_factor(1.2), 0.0);
        assert_eq!(nu_factor(-1.5), 0.0);
    }

    #[test]
    fn box_zp_hand_calc() {
        // H=B=400, t=19 の角形鋼管。
        let h = 400.0;
        let b = 400.0;
        let t = 19.0;
        let zp = box_zp(h, b, t);
        let expected = b * h * h / 4.0 - (b - 2.0 * t) * (h - 2.0 * t) * (h - 2.0 * t) / 4.0;
        assert!((zp - expected).abs() < 1e-6);
        // 400x400x19 の Zp はおよそ 4.1e6 mm^3（手計算: 400*400^2/4 - 362*362^2/4 ≈ 4,140,518）。
        assert!(zp > 3.5e6 && zp < 4.5e6, "zp={}", zp);
    }

    #[test]
    fn panel_mpp_branches_at_n_0_5() {
        let dc = 400.0;
        let db = 500.0;
        let tp = 12.0;
        let f = 235.0;
        let ve = 2.0 * dc * db * tp;
        let base = ve * f / 3f64.sqrt();

        let mpp_low = panel_mpp(dc, db, tp, f, 0.3);
        assert!((mpp_low - base).abs() < 1e-6);

        let mpp_high = panel_mpp(dc, db, tp, f, 0.8);
        let expected_high = base * 2.0 * (0.8_f64 * 0.2).sqrt();
        assert!((mpp_high - expected_high).abs() < 1e-6);
        assert!(mpp_high < base);
    }

    #[test]
    fn cold_formed_ratio_check_uses_min_of_beam_and_panel_requirement() {
        let zp = box_zp(400.0, 400.0, 19.0);
        let f = 325.0;
        let mpp = panel_mpp(400.0, 500.0, 12.0, 235.0, 0.3);
        let inp = ColdFormedInput {
            zp_col_upper: zp,
            zp_col_lower: zp,
            f_col_upper: f,
            f_col_lower: f,
            n_upper: 0.3,
            n_lower: 0.3,
            sum_beam_mp: 300_000_000.0,
            panel_mpp: mpp,
        };
        let res = cold_formed_column_ratio_check(&inp);
        let nu = nu_factor(0.3);
        let sum_mpc = nu * f * zp * 2.0;
        let expected_required = (1.5 * inp.sum_beam_mp).min(1.3 * mpp);
        let expected_ratio = expected_required / sum_mpc;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    #[test]
    fn cold_formed_ratio_check_ignores_panel_when_mpp_is_zero() {
        let zp = box_zp(400.0, 400.0, 19.0);
        let f = 325.0;
        let inp = ColdFormedInput {
            zp_col_upper: zp,
            zp_col_lower: zp,
            f_col_upper: f,
            f_col_lower: f,
            n_upper: 0.3,
            n_lower: 0.3,
            sum_beam_mp: 300_000_000.0,
            panel_mpp: 0.0,
        };
        let res = cold_formed_column_ratio_check(&inp);
        let nu = nu_factor(0.3);
        let sum_mpc = nu * f * zp * 2.0;
        let expected_ratio = (1.5 * inp.sum_beam_mp) / sum_mpc;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }
}
