//! 質点系（串団子）モデルの生成（RESP-D「07 非線形解析（動的解析）」質点系解析モデル
//! の非線形特性）。
//!
//! 立体フレームのプッシュオーバー（漸増静的）結果から、層ごとの層せん断力 Q・層間変形 δ
//! 関係（Q-δ 曲線）を抽出し、**等包絡面積則**でトリリニア骨格へ縮約した串団子モデルを
//! 生成する。
//!
//! - 初期剛性 K1: プッシュオーバー第1ステップの荷重-変形勾配。
//! - 第3折点（終局）: Q-δ 曲線の終端。第3勾配 K3: 終端の接線勾配。
//! - 第1折点: 割線剛性が K1 の指定比率以下となった変位（ルール1）、第1勾配は K1。
//! - 第2折点: 0→第3折点の包絡面積が実曲線と等しくなるよう自動決定。
//!
//! 詳細なルール1/2/3の分岐（降伏部材比率等）は簡略化しており、第1折点の判定は
//! 割線剛性比率（`secant_ratio`）で行う。

use crate::pushover::PushoverResult;
use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;
use squid_n_core::units::GRAVITY_MM_S2;

/// 層のトリリニア骨格（Q-δ）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoryTrilinear {
    /// 初期剛性 K1 [N/mm]。
    pub k1: f64,
    /// 第1折点 (δ1[mm], Q1[N])。
    pub d1: f64,
    pub q1: f64,
    /// 第2折点 (δ2, Q2)。
    pub d2: f64,
    pub q2: f64,
    /// 第3折点＝終局 (δ3, Q3)。
    pub d3: f64,
    pub q3: f64,
}

impl StoryTrilinear {
    /// 第2勾配 K2 = (Q2−Q1)/(δ2−δ1)。
    pub fn k2(&self) -> f64 {
        if self.d2 > self.d1 {
            (self.q2 - self.q1) / (self.d2 - self.d1)
        } else {
            0.0
        }
    }
    /// 第3勾配 K3 = (Q3−Q2)/(δ3−δ2)。
    pub fn k3(&self) -> f64 {
        if self.d3 > self.d2 {
            (self.q3 - self.q2) / (self.d3 - self.d2)
        } else {
            0.0
        }
    }
}

/// 串団子モデルの1質点（層）。
#[derive(Clone, Copy, Debug)]
pub struct StoryStick {
    pub story: StoryId,
    /// 質量 [t]（= 地震重量 W / g）。
    pub mass: f64,
    /// 階高 [mm]。
    pub height: f64,
    /// 層の復元力特性（トリリニア）。
    pub skeleton: StoryTrilinear,
}

/// モデル化タイプ（RESP-D「07」モデル化タイプ）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LumpedMassType {
    /// 等価せん断型（曲げ剛性を剛とする）。
    #[default]
    EquivalentShear,
    /// 等価曲げせん断型（曲げ剛性を梁要素として考慮）。
    EquivalentBendingShear,
    /// 曲げせん断分離型（曲げ剛性を回転ばねとして考慮）。
    BendingShearSeparated,
}

impl LumpedMassType {
    pub fn label(&self) -> &'static str {
        match self {
            LumpedMassType::EquivalentShear => "等価せん断型",
            LumpedMassType::EquivalentBendingShear => "等価曲げせん断型",
            LumpedMassType::BendingShearSeparated => "曲げせん断分離型",
        }
    }
}

/// 串団子モデル。層ごとの質点と復元力特性を保持する。
pub struct LumpedMassModel {
    pub model_type: LumpedMassType,
    pub stories: Vec<StoryStick>,
}

/// 台形則で (0,0) から曲線終端までの包絡面積を求める。
fn envelope_area(pts: &[(f64, f64)]) -> f64 {
    let mut a = 0.0;
    let (mut pd, mut pq) = (0.0, 0.0);
    for &(d, q) in pts {
        a += 0.5 * (pq + q) * (d - pd);
        pd = d;
        pq = q;
    }
    a
}

/// 層 Q-δ 曲線（δ 昇順・正値）を等包絡面積則でトリリニアへ縮約する。
/// `secant_ratio`（0..1）: 第1折点＝割線剛性が K1 のこの比率以下となる変位。
pub fn fit_story_trilinear(curve: &[(f64, f64)], secant_ratio: f64) -> StoryTrilinear {
    // 正の変形のみ・δ 昇順に整える。
    let mut pts: Vec<(f64, f64)> = curve.iter().copied().filter(|&(d, _)| d > 0.0).collect();
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);

    if pts.is_empty() {
        return StoryTrilinear {
            k1: 0.0,
            d1: 0.0,
            q1: 0.0,
            d2: 0.0,
            q2: 0.0,
            d3: 0.0,
            q3: 0.0,
        };
    }
    let (d_first, q_first) = pts[0];
    let (d3, q3) = *pts.last().unwrap();
    let k1 = if d_first > 0.0 {
        q_first / d_first
    } else {
        0.0
    };
    if k1 <= 0.0 || d3 <= d_first {
        // 単調1点・剛性不定は弾性トリリニア（折点なし）で返す。
        return StoryTrilinear {
            k1,
            d1: d3,
            q1: q3,
            d2: d3,
            q2: q3,
            d3,
            q3,
        };
    }
    // 第3勾配 K3 = 終端接線（[0, K1] にクランプ）。
    let k3 = if pts.len() >= 2 {
        let (dp, qp) = pts[pts.len() - 2];
        if d3 > dp {
            ((q3 - qp) / (d3 - dp)).clamp(0.0, k1)
        } else {
            0.0
        }
    } else {
        (q3 / d3).clamp(0.0, k1)
    };
    // 第1折点 δ1: 接線勾配が secant_ratio·K1 を初めて下回る直前の変位（弾性限）。
    // 第1勾配は K1。接線基準は割線基準より弾性限（折れ点）を鋭く捉える（降伏後剛性が
    // 小さい場合でも Q1=K1·δ1 が過大にならない）。
    let thr = secant_ratio * k1;
    let mut d1 = d3 * 0.5;
    let mut prev = (0.0, 0.0);
    let mut found = false;
    for &(d, q) in &pts {
        let tan = if d > prev.0 {
            (q - prev.1) / (d - prev.0)
        } else {
            k1
        };
        if tan < thr && prev.0 > 0.0 {
            d1 = prev.0;
            found = true;
            break;
        }
        prev = (d, q);
    }
    if !found {
        d1 = d3 * 0.5;
    }
    let d1 = d1.clamp(d_first, d3 * 0.9);
    let q1 = k1 * d1;

    // 等包絡面積: A_tri(δ2)=A_actual を解く。Q2 は第3勾配直線上 Q2=Q3−K3(δ3−δ2)。
    // A_tri は δ2 について線形（∂A/∂δ2 = ½[(Q1−Q3)+K3(δ3−δ1)] 一定）なので直接解ける。
    let a_actual = envelope_area(&pts);
    let a_tri = |d2: f64| {
        let q2 = q3 - k3 * (d3 - d2);
        0.5 * d1 * q1 + 0.5 * (q1 + q2) * (d2 - d1) + 0.5 * (q2 + q3) * (d3 - d2)
    };
    let slope = 0.5 * ((q1 - q3) + k3 * (d3 - d1));
    let d2 = if slope.abs() < 1e-30 {
        0.5 * (d1 + d3)
    } else {
        (d1 + (a_actual - a_tri(d1)) / slope).clamp(d1, d3)
    };
    let q2 = q3 - k3 * (d3 - d2);

    StoryTrilinear {
        k1,
        d1,
        q1,
        d2,
        q2,
        d3,
        q3,
    }
}

/// プッシュオーバー結果から串団子モデル（層ごとの質点・復元力特性）を生成する。
/// `secant_ratio`: 第1折点判定の割線剛性比（既定 0.75 程度）。
pub fn build_lumped_mass_model(
    model: &Model,
    pushover: &PushoverResult,
    model_type: LumpedMassType,
    secant_ratio: f64,
) -> LumpedMassModel {
    let n_story = model.stories.len();
    let mut sticks = Vec::with_capacity(n_story);
    for (i, story) in model.stories.iter().enumerate() {
        // 層 i の Q-δ 曲線（各キャパシティ点の層せん断・層間変形）。
        let curve: Vec<(f64, f64)> = pushover
            .capacity_curve
            .iter()
            .filter_map(|cp| {
                let d = cp.story_drift.get(i).copied()?.abs();
                let q = cp.story_shear.get(i).copied()?.abs();
                Some((d, q))
            })
            .collect();
        let skeleton = fit_story_trilinear(&curve, secant_ratio);

        // 質量 = 地震重量 / g（未設定なら節点質量の合計）。
        let mass = match story.seismic_weight {
            Some(w) if w > 0.0 => w / GRAVITY_MM_S2,
            _ => story
                .node_ids
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .filter_map(|n| n.mass)
                .map(|m| m[0].max(m[1]))
                .sum(),
        };
        // 階高 = 当該階標高 − 直下階標高（最下階は標高そのもの）。
        let below = if i > 0 {
            model.stories[i - 1].elevation
        } else {
            0.0
        };
        let height = (story.elevation - below).max(0.0);

        sticks.push(StoryStick {
            story: story.id,
            mass,
            height,
            skeleton,
        });
    }
    LumpedMassModel {
        model_type,
        stories: sticks,
    }
}

// ──────────────────────────── 質点系（串団子）時刻歴応答解析 ────────────────────────────

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
fn solve_tridiagonal(a: &[f64], b_diag: &[f64], c: &[f64], d: &[f64]) -> Vec<f64> {
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
fn fundamental_omega(m: &[f64], k: &[f64]) -> f64 {
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
/// Newton-Raphson）。RESP-D「07」質点系解析モデルの復元力特性（各層トリリニア）。
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fit_trilinear_equal_area_and_endpoints() {
        // 実曲線: 折れ点のあるなめらかな軟化曲線を細かくサンプル。
        // 0→(1,100) K1=100、(1,100)→(3,140) K2=20、(3,140)→(6,155) K3=5。
        let mut curve = Vec::new();
        for step in 1..=60 {
            let d = step as f64 * 0.1;
            let q = if d <= 1.0 {
                100.0 * d
            } else if d <= 3.0 {
                100.0 + 20.0 * (d - 1.0)
            } else {
                140.0 + 5.0 * (d - 3.0)
            };
            curve.push((d, q));
        }
        let tri = fit_story_trilinear(&curve, 0.9);
        // K1 = 初期剛性 100。
        assert!((tri.k1 - 100.0).abs() < 1.0, "k1={}", tri.k1);
        // 終端 (6, 155)。
        assert!((tri.d3 - 6.0).abs() < 1e-6 && (tri.q3 - 155.0).abs() < 1e-6);
        // 折点は昇順・耐力単調増加。
        assert!(tri.d1 < tri.d2 && tri.d2 <= tri.d3);
        assert!(tri.q1 <= tri.q2 + 1e-9 && tri.q2 <= tri.q3 + 1e-9);
        // 等包絡面積: トリリニアの面積 = 実曲線の面積。
        let a_actual = envelope_area(&curve);
        let a_tri = 0.5 * tri.d1 * tri.q1
            + 0.5 * (tri.q1 + tri.q2) * (tri.d2 - tri.d1)
            + 0.5 * (tri.q2 + tri.q3) * (tri.d3 - tri.d2);
        assert!(
            (a_tri - a_actual).abs() < 1e-3 * a_actual,
            "equal-area: a_tri={a_tri}, a_actual={a_actual}"
        );
    }

    #[test]
    fn test_fit_trilinear_k2_k3_helpers() {
        // 3勾配（K1=80 > K2=30 > K3=8）の軟化曲線。
        let curve: Vec<(f64, f64)> = (1..=50)
            .map(|s| {
                let d = s as f64 * 0.1;
                let q = if d <= 1.0 {
                    80.0 * d
                } else if d <= 2.5 {
                    80.0 + 30.0 * (d - 1.0)
                } else {
                    125.0 + 8.0 * (d - 2.5)
                };
                (d, q)
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!(
            tri.d1 < tri.d2 && tri.d2 < tri.d3,
            "distinct folds: {tri:?}"
        );
        assert!(
            tri.k1 >= tri.k2() && tri.k2() >= tri.k3() - 1e-6,
            "K1>=K2>=K3: k1={}, k2={}, k3={}",
            tri.k1,
            tri.k2(),
            tri.k3()
        );
        assert!(tri.k3() >= 0.0 && tri.k3() <= tri.k1);
    }

    #[test]
    fn test_fit_trilinear_bilinear_input_reduces_gracefully() {
        // バイリニア入力（K1=50→K=5）はトリリニアが縮退（d1≈d2）しても panic せず妥当。
        let curve: Vec<(f64, f64)> = (1..=30)
            .map(|s| {
                let d = s as f64 * 0.1;
                (d, 50.0 * d.min(2.0) + 5.0 * (d - 2.0).max(0.0))
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!((tri.k1 - 50.0).abs() < 1.0);
        assert!(tri.d1 <= tri.d2 && tri.d2 <= tri.d3);
        assert!((tri.d3 - 3.0).abs() < 1e-6 && (tri.q3 - 105.0).abs() < 1e-6);
    }

    #[test]
    fn test_fit_trilinear_empty_and_degenerate() {
        let tri = fit_story_trilinear(&[], 0.75);
        assert_eq!(tri.k1, 0.0);
        // 1点のみ（弾性）。
        let tri1 = fit_story_trilinear(&[(2.0, 200.0)], 0.75);
        assert!((tri1.k1 - 100.0).abs() < 1e-9);
    }

    fn stick(mass: f64, k1: f64, d1: f64, d2: f64, q2: f64, d3: f64, q3: f64) -> StoryStick {
        StoryStick {
            story: StoryId(0),
            mass,
            height: 3000.0,
            skeleton: StoryTrilinear {
                k1,
                d1,
                q1: k1 * d1,
                d2,
                q2,
                d3,
                q3,
            },
        }
    }

    #[test]
    fn test_solve_tridiagonal_identity() {
        // 単位行列: x=b。
        let x = solve_tridiagonal(
            &[0.0, 0.0, 0.0],
            &[1.0, 1.0, 1.0],
            &[0.0, 0.0, 0.0],
            &[3.0, 5.0, 7.0],
        );
        assert!(
            (x[0] - 3.0).abs() < 1e-12 && (x[1] - 5.0).abs() < 1e-12 && (x[2] - 7.0).abs() < 1e-12
        );
    }

    #[test]
    fn test_fundamental_omega_sdof() {
        // 1 質点: ω1=√(k/m)。
        let w = fundamental_omega(&[2.0], &[800.0]);
        assert!((w - (800.0_f64 / 2.0).sqrt()).abs() < 1e-6, "w={w}");
    }

    #[test]
    fn test_stick_th_zero_input_zero_response() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![stick(1.0, 1000.0, 0.1, 0.3, 140.0, 1.0, 160.0)],
        };
        let res = lumped_mass_time_history(&lm, &vec![0.0; 200], 0.01, 0.02);
        assert!(res.roof_disp.iter().all(|&v| v.abs() < 1e-9));
        assert_eq!(res.story_ductility[0], 0.0);
    }

    #[test]
    fn test_stick_th_responds_and_bounded() {
        // 正弦地動で応答が非ゼロかつ有限。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![
                stick(1.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.0, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let dt = 0.01;
        let accel: Vec<f64> = (0..300)
            .map(|i| 2000.0 * (2.0 * std::f64::consts::PI * 1.5 * i as f64 * dt).sin())
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.03);
        assert_eq!(res.time.len(), 300);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.roof_disp.iter().any(|&v| v.abs() > 1e-3),
            "should show nonzero roof response"
        );
        assert_eq!(res.story_peak_drift.len(), 2);
    }

    #[test]
    fn test_stick_th_yields_under_strong_input() {
        // 強い地動で層が降伏（塑性率 μ>1）。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![stick(2.0, 1000.0, 0.5, 2.0, 700.0, 8.0, 800.0)],
        };
        let dt = 0.01;
        // 一定方向の強い引き込みで大変形。
        let accel: Vec<f64> = (0..400)
            .map(|i| {
                let t = i as f64 * dt;
                3000.0 * (2.0 * std::f64::consts::PI * 0.8 * t).sin()
            })
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.02);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.story_ductility[0] > 1.0,
            "strong input should yield the story: μ={}",
            res.story_ductility[0]
        );
    }
}
