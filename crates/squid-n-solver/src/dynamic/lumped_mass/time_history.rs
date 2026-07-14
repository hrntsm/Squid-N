//! 質点系（串団子）時刻歴応答（せん断型多質点系、構造力学）。
//!
//! 各層トリリニアの最大点指向型履歴で、Newmark-β 平均加速度法・Newton-Raphson により
//! 非線形時刻歴応答を解く。減衰は初期剛性比例。
//!
//! - [`StickResponse`] — 質点系（せん断型）時刻歴応答解析の結果。
//! - [`lumped_mass_time_history`] — 質点系モデルの非線形時刻歴応答解析。

use super::model::{LumpedMassModel, StoryTrilinear};
use squid_n_material::{HysteresisMaterial, HysteresisRule, UniaxialMaterial};

/// 質点系（せん断型）時刻歴応答解析の結果。
#[derive(Clone, Debug)]
pub struct StickResponse {
    /// 時刻列 [s]。
    pub time: Vec<f64>,
    /// 最上階（頂部）質点の絶対変位時刻歴 [mm]。
    pub roof_disp: Vec<f64>,
    /// 各層の最大層間変形 [mm]（下層→上層）。
    pub story_peak_drift: Vec<f64>,
    /// 各層の最大層せん断力 [N]。
    pub story_peak_shear: Vec<f64>,
    /// 各層の最大塑性率 μ = δmax/δ1（δ1=第1折点変形。δ1≤0 は 0）。
    pub story_ductility: Vec<f64>,
}

/// 三重対角系 `A·x=b` を Thomas 法で解く（`a`=下副対角, `b_diag`=主対角, `c`=上副対角）。
pub(crate) fn solve_tridiagonal(a: &[f64], b_diag: &[f64], c: &[f64], d: &[f64]) -> Vec<f64> {
    let n = b_diag.len();
    if n == 0 {
        return Vec::new();
    }
    let mut cp = vec![0.0; n];
    let mut dp = vec![0.0; n];
    cp[0] = c[0] / b_diag[0];
    dp[0] = d[0] / b_diag[0];
    for i in 1..n {
        let m = b_diag[i] - a[i] * cp[i - 1];
        let m = if m.abs() < 1e-30 { 1e-30 } else { m };
        cp[i] = c[i] / m;
        dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
    }
    let mut x = vec![0.0; n];
    x[n - 1] = dp[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = dp[i] - cp[i] * x[i + 1];
    }
    x
}

/// 初期剛性の三重対角せん断系の 1 次固有円振動数 ω1 を逆反復で求める。
/// `m`=質量, `k`=各層初期剛性（`k[i]`=層 i の K1）。
pub(crate) fn fundamental_omega(m: &[f64], k: &[f64]) -> f64 {
    let n = m.len();
    if n == 0 {
        return 0.0;
    }
    // 初期剛性の三重対角 K（せん断型: K[i][i]=k_i+k_{i+1}, 副対角=−k_{i+1}）。
    let mut diag = vec![0.0; n];
    let mut lower = vec![0.0; n];
    let mut upper = vec![0.0; n];
    for i in 0..n {
        let ki = k[i];
        let ki1 = if i + 1 < n { k[i + 1] } else { 0.0 };
        diag[i] = ki + ki1;
        if i + 1 < n {
            upper[i] = -ki1;
            lower[i + 1] = -ki1;
        }
    }
    // 逆反復: K x = M x_prev。
    let mut x = vec![1.0; n];
    let mut omega2 = 0.0;
    for _ in 0..50 {
        let b: Vec<f64> = (0..n).map(|i| m[i] * x[i]).collect();
        let y = solve_tridiagonal(&lower, &diag, &upper, &b);
        // 正規化（M ノルム）。
        let ynorm: f64 = (0..n).map(|i| m[i] * y[i] * y[i]).sum::<f64>().sqrt();
        if ynorm < 1e-30 {
            break;
        }
        let xn: Vec<f64> = y.iter().map(|v| v / ynorm).collect();
        // Rayleigh 商 ω² = xᵀKx / xᵀMx。
        let kx_diag: Vec<f64> = (0..n)
            .map(|i| {
                let mut s = diag[i] * xn[i];
                if i > 0 {
                    s += lower[i] * xn[i - 1];
                }
                if i + 1 < n {
                    s += upper[i] * xn[i + 1];
                }
                s
            })
            .collect();
        let num: f64 = (0..n).map(|i| xn[i] * kx_diag[i]).sum();
        let den: f64 = (0..n).map(|i| m[i] * xn[i] * xn[i]).sum();
        omega2 = if den > 0.0 { num / den } else { 0.0 };
        x = xn;
    }
    omega2.max(0.0).sqrt()
}

/// 層のトリリニア骨格から最大点指向型（Clough 系トリリニア）の履歴材料を作る。
fn story_spring(sk: &StoryTrilinear) -> HysteresisMaterial {
    HysteresisMaterial::new(HysteresisRule::MaxPointOriented {
        crack: (sk.q1.max(1.0), sk.d1.max(1e-6)),
        yield_point: (sk.q2.max(sk.q1 + 1.0), sk.d2.max(sk.d1 * 1.0001)),
        ultimate: (sk.q3.max(sk.q2 + 1.0), sk.d3.max(sk.d2 * 1.0001)),
    })
}

/// 質点系（せん断型）モデルの非線形時刻歴応答解析（Newmark-β 平均加速度法・
/// Newton-Raphson）。せん断型多質点系の復元力特性（各層トリリニア、構造力学）。
///
/// - `lm`: 串団子モデル（各層の質量・トリリニア骨格）。
/// - `accel`: 地動加速度 [mm/s²]。`dt`: 刻み [s]。`h`: 減衰定数（初期剛性比例）。
///
/// 各層の復元力は最大点指向型トリリニア。減衰は初期剛性比例
/// `C=(2h/ω1)·K_init`（ω1=1 次固有円振動数）。
pub fn lumped_mass_time_history(
    lm: &LumpedMassModel,
    accel: &[f64],
    dt: f64,
    h: f64,
) -> StickResponse {
    let n = lm.stories.len();
    if n == 0 || dt <= 0.0 || accel.is_empty() {
        return StickResponse {
            time: Vec::new(),
            roof_disp: Vec::new(),
            story_peak_drift: vec![0.0; n],
            story_peak_shear: vec![0.0; n],
            story_ductility: vec![0.0; n],
        };
    }
    let mass: Vec<f64> = lm.stories.iter().map(|s| s.mass.max(1e-9)).collect();
    let k_init: Vec<f64> = lm.stories.iter().map(|s| s.skeleton.k1.max(1e-9)).collect();
    let mut springs: Vec<HysteresisMaterial> = lm
        .stories
        .iter()
        .map(|s| story_spring(&s.skeleton))
        .collect();

    // 初期剛性比例減衰係数 a1=2h/ω1。
    let omega1 = fundamental_omega(&mass, &k_init);
    let a1 = if omega1 > 0.0 { 2.0 * h / omega1 } else { 0.0 };

    // Newmark 平均加速度（β=1/4, γ=1/2）。
    let beta = 0.25;
    let gamma = 0.5;
    let c1 = 1.0 / (beta * dt * dt); // a = c1·Δu − ...
    let c2 = gamma / (beta * dt); // v = c2·Δu − ...

    let mut u = vec![0.0; n];
    let mut v = vec![0.0; n];
    let mut a = vec![0.0; n];

    let mut time: Vec<f64> = Vec::with_capacity(accel.len());
    let mut roof: Vec<f64> = Vec::with_capacity(accel.len());
    let mut peak_drift: Vec<f64> = vec![0.0; n];
    let mut peak_shear: Vec<f64> = vec![0.0; n];

    // 層ドリフト δ_i = u_i − u_{i-1}（u_0=base=0）。
    let drift = |u: &[f64], i: usize| if i == 0 { u[0] } else { u[i] - u[i - 1] };

    for (step, &ag) in accel.iter().enumerate() {
        // 外力（地動慣性力）。
        let p: Vec<f64> = mass.iter().map(|&mi| -mi * ag).collect();
        // 予測子（変位一定, du=0 から Newton）。
        let u_prev = u.clone();
        let v_prev = v.clone();
        let a_prev = a.clone();
        let mut u_tr = u_prev.clone();

        for _iter in 0..30 {
            // 層せん断・接線（各 spring を drift で試行）。
            let mut q = vec![0.0; n];
            let mut kt = vec![0.0; n];
            for i in 0..n {
                let (qi, ki) = springs[i].trial(drift(&u_tr, i));
                q[i] = qi;
                kt[i] = ki.max(1e-6);
            }
            // 内力 f_int[i]=Q_i−Q_{i+1}。
            let mut f_int = vec![0.0; n];
            for i in 0..n {
                let q_above = if i + 1 < n { q[i + 1] } else { 0.0 };
                f_int[i] = q[i] - q_above;
            }
            // Newmark の a, v（u_tr に対応）。
            let a_tr: Vec<f64> = (0..n)
                .map(|i| {
                    c1 * (u_tr[i] - u_prev[i])
                        - (1.0 / (beta * dt)) * v_prev[i]
                        - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
                })
                .collect();
            let v_tr: Vec<f64> = (0..n)
                .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_tr[i]))
                .collect();
            // 減衰力 C·v、C=a1·K_t（三重対角）。C·v を層剛性から直接計算。
            let cv = tridiag_stiffness_matvec(&kt, &v_tr, a1);
            // 残差 r = p − M·a − C·v − f_int。
            let mut r = vec![0.0; n];
            let mut rnorm = 0.0;
            for i in 0..n {
                r[i] = p[i] - mass[i] * a_tr[i] - cv[i] - f_int[i];
                rnorm += r[i] * r[i];
            }
            if rnorm.sqrt() < 1e-6 * (1.0 + p.iter().map(|x| x * x).sum::<f64>().sqrt()) {
                // 収束。
                break;
            }
            // 有効接線 Keff = c1·M + c2·C + K_t（三重対角）。
            let (low, diag, up) = effective_tridiagonal(&mass, &kt, a1, c1, c2);
            let du = solve_tridiagonal(&low, &diag, &up, &r);
            for i in 0..n {
                u_tr[i] += du[i];
            }
        }

        // 確定。
        for s in springs.iter_mut() {
            s.commit();
        }
        // a, v を確定値へ更新。
        let a_new: Vec<f64> = (0..n)
            .map(|i| {
                c1 * (u_tr[i] - u_prev[i])
                    - (1.0 / (beta * dt)) * v_prev[i]
                    - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
            })
            .collect();
        let v_new: Vec<f64> = (0..n)
            .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_new[i]))
            .collect();
        u = u_tr;
        v = v_new;
        a = a_new;

        // 応答の記録。
        for i in 0..n {
            let d = drift(&u, i).abs();
            peak_drift[i] = peak_drift[i].max(d);
            let (qi, _) = {
                let mut sp = springs[i].clone();
                sp.trial(drift(&u, i))
            };
            peak_shear[i] = peak_shear[i].max(qi.abs());
        }
        time.push(step as f64 * dt);
        roof.push(u[n - 1]);
    }

    let ductility: Vec<f64> = lm
        .stories
        .iter()
        .zip(peak_drift.iter())
        .map(|(s, &d)| {
            if s.skeleton.d1 > 1e-9 {
                d / s.skeleton.d1
            } else {
                0.0
            }
        })
        .collect();

    StickResponse {
        time,
        roof_disp: roof,
        story_peak_drift: peak_drift,
        story_peak_shear: peak_shear,
        story_ductility: ductility,
    }
}

/// せん断型三重対角剛性 `K(kt)` と `a1·K` の和は使わず、`(scale·K)·x` を直接計算する。
/// `scale` は減衰係数 a1。せん断型: `K[i][i]=kt_i+kt_{i+1}`, 副対角 `−kt_{i+1}`。
fn tridiag_stiffness_matvec(kt: &[f64], x: &[f64], scale: f64) -> Vec<f64> {
    let n = kt.len();
    let mut y = vec![0.0; n];
    for i in 0..n {
        let ki = kt[i];
        let ki1 = if i + 1 < n { kt[i + 1] } else { 0.0 };
        let mut s = (ki + ki1) * x[i];
        if i > 0 {
            s += -ki * x[i - 1];
        }
        if i + 1 < n {
            s += -ki1 * x[i + 1];
        }
        y[i] = scale * s;
    }
    y
}

/// 有効接線 `Keff = c1·M + c2·(a1·K) + K` の三重対角成分（下・主・上）。
fn effective_tridiagonal(
    mass: &[f64],
    kt: &[f64],
    a1: f64,
    c1: f64,
    c2: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = kt.len();
    let mut low = vec![0.0; n];
    let mut diag = vec![0.0; n];
    let mut up = vec![0.0; n];
    // 剛性倍率: K 自身は係数 1、減衰 C=a1·K は係数 c2 → 合計 (1 + c2·a1)。
    let ks = 1.0 + c2 * a1;
    for i in 0..n {
        let ki = kt[i];
        let ki1 = if i + 1 < n { kt[i + 1] } else { 0.0 };
        diag[i] = c1 * mass[i] + ks * (ki + ki1);
        if i + 1 < n {
            up[i] = -ks * ki1;
            low[i + 1] = -ks * ki1;
        }
    }
    (low, diag, up)
}
