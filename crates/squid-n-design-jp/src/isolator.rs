//! 免震支承材の**マルチシアスプリング低減率・摩擦力**（RESP-D マニュアル「計算編
//! 05 非線形モデル」免震支承材）。
//!
//! # 位置付け
//! マルチシアスプリング要素は、各方向の特性を等方にするため水平ばねを n 本の
//! 放射配置ばねに分割してモデル化する。RESP-D では n=8（天然ゴム系積層ゴムは n=2）。
//! 本モジュールは 1 本あたりの剛性・耐力の低減率と、摩擦ばねの最大摩擦力を算定する
//! 純関数群である（要素の合計特性は入力目標値に一致し、本低減率は 1 本分の値の
//! 導出・照合に用いる）。
//!
//! # 準拠する式（RESP-D「05 非線形モデル」）
//! - 剛性低減率 = 2/n
//! - 耐力低減率 = 1 / Σ_{i=1}^{n/2}（cos(π/n·(i−1)) + sin(π/n·(i−1))）
//! - 摩擦ばね: Qmax = μ·N（μ=摩擦係数、N=長期軸力）

/// マルチシアスプリング 1 本あたりの剛性低減率 = 2/n。
/// `n < 1` は 1.0 を返す。
pub fn multi_shear_stiffness_reduction(n: u32) -> f64 {
    if n < 1 {
        return 1.0;
    }
    2.0 / n as f64
}

/// マルチシアスプリング 1 本あたりの耐力低減率
/// = 1 / Σ_{i=1}^{n/2}（cos(π/n·(i−1)) + sin(π/n·(i−1))）。
/// `n < 2` は 1.0 を返す。
pub fn multi_shear_strength_reduction(n: u32) -> f64 {
    if n < 2 {
        return 1.0;
    }
    let nf = n as f64;
    let mut sum = 0.0;
    for i in 1..=(n / 2) {
        let ang = std::f64::consts::PI / nf * (i as f64 - 1.0);
        sum += ang.cos() + ang.sin();
    }
    if sum > 0.0 {
        1.0 / sum
    } else {
        1.0
    }
}

/// 摩擦ばね（弾性すべり支承）の最大摩擦力 Qmax [N] = μ·N。
/// `N`（長期軸力、圧縮正）が負（引張・浮上り）の場合は 0（摩擦力を生じない）。
pub fn friction_max_force(mu: f64, axial_n: f64) -> f64 {
    mu.max(0.0) * axial_n.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stiffness_reduction_matches_respd_table() {
        // 本数 2/4/6/8/16 → 1.0/0.5/0.33333/0.25/0.125。
        for (n, expect) in [(2, 1.0), (4, 0.5), (6, 1.0 / 3.0), (8, 0.25), (16, 0.125)] {
            assert!(
                (multi_shear_stiffness_reduction(n) - expect).abs() < 1e-5,
                "n={n}: {} vs {expect}",
                multi_shear_stiffness_reduction(n)
            );
        }
    }

    #[test]
    fn test_strength_reduction_matches_respd_table() {
        // 本数 2/4/6/8/16 → 1.0/0.41421/0.26795/0.19891/0.09849。
        for (n, expect) in [
            (2, 1.0),
            (4, 0.41421),
            (6, 0.26795),
            (8, 0.19891),
            (16, 0.09849),
        ] {
            assert!(
                (multi_shear_strength_reduction(n) - expect).abs() < 1e-4,
                "n={n}: {} vs {expect}",
                multi_shear_strength_reduction(n)
            );
        }
    }

    #[test]
    fn test_friction_max_force() {
        assert!((friction_max_force(0.1, 1_000_000.0) - 100_000.0).abs() < 1e-6);
        // 引張軸力（浮上り）は摩擦力ゼロ。
        assert_eq!(friction_max_force(0.1, -500_000.0), 0.0);
        assert_eq!(friction_max_force(-0.1, 1_000_000.0), 0.0);
    }

    #[test]
    fn test_reductions_monotonic_in_n() {
        // 本数が増えると 1 本あたりの低減率は単調減少。
        let ns = [2u32, 4, 6, 8, 16];
        for w in ns.windows(2) {
            assert!(multi_shear_stiffness_reduction(w[0]) > multi_shear_stiffness_reduction(w[1]));
            assert!(multi_shear_strength_reduction(w[0]) > multi_shear_strength_reduction(w[1]));
        }
    }
}
