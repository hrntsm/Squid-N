//! 免震支承材の**マルチシアスプリング低減率・摩擦力**（各免震部材指針の
//! 免震支承材モデル）。
//!
//! # 位置付け
//! マルチシアスプリング要素は、各方向の特性を等方にするため水平ばねを n 本の
//! 放射配置ばねに分割してモデル化する。本実装では n=8（天然ゴム系積層ゴムは n=2）を既定とする。
//! 本モジュールは 1 本あたりの剛性・耐力の低減率と、摩擦ばねの最大摩擦力を算定する
//! 純関数群である（要素の合計特性は入力目標値に一致し、本低減率は 1 本分の値の
//! 導出・照合に用いる）。
//!
//! # 準拠する式（各免震部材指針）
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

// ──────────────────── 鉛プラグ挿入型積層ゴム LRB 統一型（歪依存バイリニア） ───────────
// 免震支承材「鉛プラグ挿入型積層ゴム a) LRB 統一型」（各免震部材の製品技術資料）。

/// LRB 統一型の降伏後剛性のひずみ依存修正係数 `CKd(γ)`。
/// `CKd = { 0.779·γ^−0.43 (γ<0.25); γ^−0.25 (0.25≤γ<1.0); γ^−0.12 (1.0≤γ) }`。
///
/// 第1分岐の指数は **−0.43**（負）。分岐点 γ=0.25 で
/// `0.779·0.25^−0.43 = 1.4144 ≒ 0.25^−0.25 = 1.4142` と連続になることから
/// 確定した（+0.41 としていた従来実装は γ<0.25 域で CKd を最大 1/7 に
/// 過小評価し、γ=0.25 で約 3.2 倍の不連続を生じる誤りだった。低ひずみ域で
/// 剛性が単調に増加するのが積層ゴムの実挙動で、他分岐の負指数とも整合する）。
pub fn lrb_stiffness_strain_factor(gamma: f64) -> f64 {
    let g = gamma.abs().max(1e-9);
    if g < 0.25 {
        0.779 * g.powf(-0.43)
    } else if g < 1.0 {
        g.powf(-0.25)
    } else {
        g.powf(-0.12)
    }
}

/// LRB 統一型の切片荷重のひずみ依存修正係数 `CQd(γ)`。
/// `CQd = { 2.036·γ^0.41 (γ≤0.1); 1.106·γ^0.145 (0.1<γ<0.5); 1 (0.5≤γ) }`。
pub fn lrb_intercept_strain_factor(gamma: f64) -> f64 {
    let g = gamma.abs().max(1e-9);
    if g <= 0.1 {
        2.036 * g.powf(0.41)
    } else if g < 0.5 {
        1.106 * g.powf(0.145)
    } else {
        1.0
    }
}

/// 温度換算（20℃基準）: 降伏後剛性 `Kd(t0)=Kd(20)·exp(−0.00271·(t0−20))`。
pub fn lrb_stiffness_at_temperature(kd20: f64, t0_celsius: f64) -> f64 {
    kd20 * (-0.00271 * (t0_celsius - 20.0)).exp()
}

/// 温度換算（20℃基準）: 切片荷重 `Qd(t0)=Qd(20)·exp(−0.00879·(t0−20))`。
pub fn lrb_intercept_at_temperature(qd20: f64, t0_celsius: f64) -> f64 {
    qd20 * (-0.00879 * (t0_celsius - 20.0)).exp()
}

/// バイリニア免震材の等価水平剛性 `keq = Qd/δ + Kd`（δ=設計変位、Kd=降伏後剛性）。
pub fn equivalent_stiffness(kd: f64, qd: f64, disp: f64) -> f64 {
    let d = disp.abs();
    if d < 1e-12 {
        kd
    } else {
        qd / d + kd
    }
}

/// バイリニア免震材の等価粘性減衰定数 `Heq`。
/// `Heq = (2/π)·Qd·(δ − Qd/((β−1)·Kd)) / (keq·δ²)`、`β = K1/Kd`（初期剛性/降伏後剛性）。
pub fn equivalent_damping(k1: f64, kd: f64, qd: f64, disp: f64) -> f64 {
    let d = disp.abs();
    if d < 1e-12 || kd <= 0.0 || qd <= 0.0 {
        return 0.0;
    }
    let beta = k1 / kd;
    if beta <= 1.0 {
        return 0.0;
    }
    let keq = qd / d + kd;
    if keq <= 0.0 {
        return 0.0;
    }
    (2.0 / std::f64::consts::PI) * qd * (d - qd / ((beta - 1.0) * kd)) / (keq * d * d)
}

// ──────────────────── 転がり支承（標準型バイリニア） ────────────────────
// 免震支承材「転がり支承」（各免震部材の製品技術資料）。

/// 転がり支承の摩擦係数 `μ = (1.2 + 7.8·Pv/Po)/1000`。
/// `Pv`=長期軸力、`Po`=静定格圧縮荷重。
pub fn rolling_bearing_friction(pv: f64, po: f64) -> f64 {
    if po.abs() < 1e-12 {
        return 0.0;
    }
    (1.2 + 7.8 * pv / po) / 1000.0
}

/// 転がり支承の折れ点耐力 `Q1 = μ·Pv`（軸力一定・長期軸力と摩擦係数から）。
pub fn rolling_bearing_yield_force(pv: f64, po: f64) -> f64 {
    rolling_bearing_friction(pv, po) * pv.max(0.0)
}

// ──────────────────── 球面すべり支承（速度・面圧依存摩擦） ────────────────────
// 免震支承材「球面すべり支承」（各免震部材の製品技術資料）。

/// 球面すべり支承 MN タイプの標準摩擦係数 `μ0`（面圧依存）。
/// `μ0 = 0.043·(2.03·σ^−0.19 + 0.068)`（σ=長期支持面圧）。
pub fn spherical_bearing_mu0_mn(sigma: f64) -> f64 {
    let s = sigma.max(1e-9);
    0.043 * (2.03 * s.powf(-0.19) + 0.068)
}

/// 球面すべり支承 LN タイプの標準摩擦係数 `μ0`（面圧依存）。
/// `μ0 = 0.013·(20·σ^−0.9 + 0.5)`。
pub fn spherical_bearing_mu0_ln(sigma: f64) -> f64 {
    let s = sigma.max(1e-9);
    0.013 * (20.0 * s.powf(-0.9) + 0.5)
}

/// 球面すべり支承 MN タイプの速度依存摩擦係数 `μ = μ0·(1.0 − 0.55·e^(−0.019·|V|))`。
/// `v`=層間速度 [mm/s]。
pub fn spherical_bearing_mu_mn(mu0: f64, v: f64) -> f64 {
    mu0 * (1.0 - 0.55 * (-0.019 * v.abs()).exp())
}

// ──────────────────── 高減衰ゴム系積層ゴム（ブリヂストン） ────────────────────
// 免震支承材「高減衰ゴム系積層ゴム ブリヂストン」（各免震部材の製品技術資料）。等価せん断弾性係数
// Geq・等価粘性減衰定数 Heq・降伏荷重特性値 U を歪 γ の多項式で与える。

fn poly(coeffs: &[f64], x: f64) -> f64 {
    // 昇冪（coeffs[0] + coeffs[1]x + …）。
    coeffs.iter().rev().fold(0.0, |acc, &c| acc * x + c)
}

/// ブリヂストン高減衰ゴム E6 タイプの `(Geq, Heq, U)`（歪 γ）。
pub fn hdr_bridgestone_e6(gamma: f64) -> (f64, f64, f64) {
    let g = poly(&[2.309, -4.327, 4.456, -2.379, 0.630, -0.0649], gamma);
    let h = poly(&[0.1894, 0.0664, -0.0353, 0.0041], gamma);
    let u = poly(&[0.3726, 0.0956, -0.0741, 0.0113], gamma);
    (g, h, u)
}

/// ブリヂストン高減衰ゴム E4 タイプの `(Geq, Heq, U)`（歪 γ）。
pub fn hdr_bridgestone_e4(gamma: f64) -> (f64, f64, f64) {
    let g = poly(&[1.308, -2.438, 2.640, -1.483, 0.4086, -0.043], gamma);
    let h = poly(&[0.227, 0.0120, -0.0088, 0.0037], gamma);
    let u = poly(&[0.379, 0.0069, -0.0046, 0.0026], gamma);
    (g, h, u)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stiffness_reduction_matches_reference_table() {
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
    fn test_strength_reduction_matches_reference_table() {
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

    #[test]
    fn test_lrb_stiffness_strain_factor_handcalc() {
        // 3 区間の代表値（第1分岐の指数は −0.43）。
        assert!((lrb_stiffness_strain_factor(0.1) - 0.779 * 0.1f64.powf(-0.43)).abs() < 1e-9);
        assert!((lrb_stiffness_strain_factor(0.5) - 0.5f64.powf(-0.25)).abs() < 1e-9);
        assert!((lrb_stiffness_strain_factor(2.0) - 2.0f64.powf(-0.12)).abs() < 1e-9);
        // 単調傾向（大歪で剛性低下）を確認。
        assert!(lrb_stiffness_strain_factor(2.0) < lrb_stiffness_strain_factor(0.5));
        // 分岐点 γ=0.25 での連続性（−0.43 の根拠。相対誤差 0.1% 以内）。
        let lo = lrb_stiffness_strain_factor(0.25 - 1e-9);
        let hi = lrb_stiffness_strain_factor(0.25 + 1e-9);
        assert!((lo - hi).abs() / hi < 1e-3, "CKd 不連続: {lo} vs {hi}");
    }

    #[test]
    fn test_lrb_intercept_strain_factor_handcalc() {
        assert!((lrb_intercept_strain_factor(0.05) - 2.036 * 0.05f64.powf(0.41)).abs() < 1e-9);
        assert!((lrb_intercept_strain_factor(0.3) - 1.106 * 0.3f64.powf(0.145)).abs() < 1e-9);
        assert_eq!(lrb_intercept_strain_factor(0.6), 1.0);
        assert_eq!(lrb_intercept_strain_factor(1.5), 1.0);
    }

    #[test]
    fn test_lrb_temperature_conversion() {
        // 30℃: Kd·exp(−0.00271·10)、Qd·exp(−0.00879·10)。高温で低下。
        assert!(
            (lrb_stiffness_at_temperature(100.0, 30.0) - 100.0 * (-0.0271f64).exp()).abs() < 1e-9
        );
        assert!(
            (lrb_intercept_at_temperature(100.0, 30.0) - 100.0 * (-0.0879f64).exp()).abs() < 1e-9
        );
        // 20℃ 基準では不変。
        assert!((lrb_stiffness_at_temperature(100.0, 20.0) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_equivalent_stiffness_and_damping_handcalc() {
        // keq = Qd/δ + Kd = 100/200 + 1 = 1.5。
        let keq = equivalent_stiffness(1.0, 100.0, 200.0);
        assert!((keq - 1.5).abs() < 1e-9, "keq={keq}");
        // Heq = (2/π)·Qd·(δ − Qd/((β−1)Kd)) / (keq·δ²), β=K1/Kd=10。
        // = (0.63662)·100·(200 − 100/9) / (1.5·40000) ≈ 0.2004。
        let heq = equivalent_damping(10.0, 1.0, 100.0, 200.0);
        let expect = (2.0 / std::f64::consts::PI) * 100.0 * (200.0 - 100.0 / 9.0) / (1.5 * 40000.0);
        assert!((heq - expect).abs() < 1e-9, "heq={heq}");
        assert!((heq - 0.2004).abs() < 5e-4, "heq≈0.2004, got {heq}");
    }

    #[test]
    fn test_rolling_bearing_handcalc() {
        // Pv=1e6, Po=5e6 → μ=(1.2+7.8·0.2)/1000=0.00276、Q1=μ·Pv=2760。
        let mu = rolling_bearing_friction(1.0e6, 5.0e6);
        assert!((mu - 0.00276).abs() < 1e-9, "mu={mu}");
        assert!((rolling_bearing_yield_force(1.0e6, 5.0e6) - 2760.0).abs() < 1e-6);
    }

    #[test]
    fn test_spherical_bearing_handcalc() {
        // μ0(MN, σ=18) = 0.043·(2.03·18^−0.19 + 0.068)。
        let mu0 = spherical_bearing_mu0_mn(18.0);
        let expect = 0.043 * (2.03 * 18.0f64.powf(-0.19) + 0.068);
        assert!((mu0 - expect).abs() < 1e-12, "mu0={mu0}");
        // V=0 で μ = μ0·(1−0.55) = 0.45·μ0（静止時）。
        assert!((spherical_bearing_mu_mn(mu0, 0.0) - 0.45 * mu0).abs() < 1e-12);
        // 高速では μ → μ0（速度依存項が減衰）。
        assert!(spherical_bearing_mu_mn(mu0, 1000.0) > spherical_bearing_mu_mn(mu0, 0.0));
        // LN タイプ μ0 も正。
        assert!(spherical_bearing_mu0_ln(10.0) > 0.0);
    }

    #[test]
    fn test_hdr_bridgestone_e6_handcalc() {
        // γ=1.0 での多項式値（昇冪係数の総和）。
        let (g, h, u) = hdr_bridgestone_e6(1.0);
        assert!(
            (g - (2.309 - 4.327 + 4.456 - 2.379 + 0.630 - 0.0649)).abs() < 1e-9,
            "G={g}"
        );
        assert!(
            (h - (0.1894 + 0.0664 - 0.0353 + 0.0041)).abs() < 1e-9,
            "H={h}"
        );
        assert!(
            (u - (0.3726 + 0.0956 - 0.0741 + 0.0113)).abs() < 1e-9,
            "U={u}"
        );
        // Heq は正の減衰定数。
        assert!(h > 0.0 && g > 0.0);
        // E4 も評価可能。
        let (g4, _, _) = hdr_bridgestone_e4(1.0);
        assert!(g4 > 0.0);
    }
}
