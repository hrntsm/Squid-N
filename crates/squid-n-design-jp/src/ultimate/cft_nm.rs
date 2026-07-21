//! コンクリート充填鋼管（CFT）柱の**N-M 相互作用（曲げを伴う終局耐力）**
//! （コンクリート充填鋼管構造設計指針（CFT 指針）に基づく）。
//!
//! # 位置付け
//! [`super::cft`] が軸方向終局耐力（Ncu/Ntu）を扱うのに対し、本モジュールは軸方向力と
//! 曲げモーメントを同時に受ける柱の終局曲げ耐力 `Mu(N)` を CFT 指針に基づき算定する。
//! - **短柱**（[`cft_short_column_mu`]）: 中立軸位置をパラメータとする耐力曲線を軸力 N に
//!   整合させ、中立軸がコンクリート断面外の場合は Ncu1・Ntu との直線補間で求める。
//! - **中柱・長柱**（[`cft_long_medium_column_mu`]）: 座屈による曲げ低減 R=(1−cNcu/Nk) を
//!   考慮し、コンクリート放物線 cMu＋鋼管の曲げ耐力（低減後）を重ね合わせる。
//!
//! # 準拠・出典（要・原典照合、`dev_docs/specs/原典照合リスト.md`）
//! - 日本建築学会「コンクリート充填鋼管構造設計指針」短柱の終局曲げ耐力。
//!
//! # 角形 sMu の第 2 項について（原典照合メモ）
//! 抽出した原文では角形の `sMu = D·t·(D−t)·Fy + 2t·(cD−xn)·xn·Fc` と末尾が `Fc` だが、
//! 第 2 項は中立軸 xn におけるウェブ 2 枚の全塑性モーメント
//! `2·∫ t·Fy·|中立軸からの距離| = 2t·xn·(cD−xn)·Fy` に一致するため、`Fy` を採用する
//! （`Fc` は OCR 誤りと判断。第 1 項 `D·t·(D−t)·Fy` はフランジ 2 枚の全塑性モーメント）。

use std::f64::consts::PI;

/// CFT 短柱の N-M 相互作用の算定入力。
#[derive(Clone, Copy, Debug)]
pub struct CftBendingInput {
    /// 円形断面なら true（角型なら false）。
    pub circular: bool,
    /// 鋼管のせい D [mm]（円形は外径）。
    pub d_steel: f64,
    /// 鋼管の幅 B [mm]（円形は外径と同値）。
    pub b_steel: f64,
    /// コンクリートのせい cD [mm]（= D − 2t）。
    pub c_d: f64,
    /// コンクリートの幅 cB [mm]（円形は cD と同値）。
    pub c_b: f64,
    /// 鋼管の板厚 t [mm]。
    pub t: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 鋼管の降伏強さ Fy [N/mm²]。
    pub fy: f64,
}

/// 角形短柱: 中立軸深さ `xn`（圧縮縁からの距離 [mm]）における (Nu, Mu)（**圧縮正**）。
///
/// ```text
/// cNu = xn·cB·Fc,  cMu = (1/2)·xn·cB·(cD − xn)·Fc
/// sNu = 2t·(2xn − cD)·Fy
/// sMu = B·t·(D − t)·Fy + 2t·xn·(cD − xn)·Fy   （第2項はウェブ全塑性、Fy）
/// ```
fn angular_nu_mu(inp: &CftBendingInput, xn: f64) -> (f64, f64) {
    let c_nu = xn * inp.c_b * inp.fc;
    let c_mu = 0.5 * xn * inp.c_b * (inp.c_d - xn) * inp.fc;
    let s_nu = 2.0 * inp.t * (2.0 * xn - inp.c_d) * inp.fy;
    let s_mu = inp.b_steel * inp.t * (inp.d_steel - inp.t) * inp.fy
        + 2.0 * inp.t * xn * (inp.c_d - xn) * inp.fy;
    (c_nu + s_nu, c_mu + s_mu)
}

/// 円形短柱: パラメータ角 `θ`（[0, π]、`θ = cos⁻¹(1 − 2xn/cD)`）における (Nu, Mu)。
///
/// ```text
/// cσcB = Fc + 0.78·(2t/(D−2t))·Fy,  r1 = cD/2,  r2 = (D−t)/2
/// cNu = r1²·(θ − sinθcosθ)·cσcB,     cMu = (2/3)·r1³·sin³θ·cσcB
/// sNu = 2·r2·t·(β1·θ − β2·(θ−π))·Fy,  sMu = 2·r2²·t·(β1 − β2)·sinθ·Fy
/// β1 = 0.89,  β2 = −1.08
/// ```
fn circular_nu_mu(inp: &CftBendingInput, theta: f64) -> (f64, f64) {
    let r1 = inp.c_d / 2.0;
    let r2 = (inp.d_steel - inp.t) / 2.0;
    let denom = inp.d_steel - 2.0 * inp.t;
    let c_sigma = if denom > 0.0 {
        inp.fc + 0.78 * (2.0 * inp.t / denom) * inp.fy
    } else {
        inp.fc
    };
    let (b1, b2) = (0.89_f64, -1.08_f64);
    let c_nu = r1 * r1 * (theta - theta.sin() * theta.cos()) * c_sigma;
    let c_mu = (2.0 / 3.0) * r1.powi(3) * theta.sin().powi(3) * c_sigma;
    let s_nu = 2.0 * r2 * inp.t * (b1 * theta - b2 * (theta - PI)) * inp.fy;
    let s_mu = 2.0 * r2 * r2 * inp.t * (b1 - b2) * theta.sin() * inp.fy;
    (c_nu + s_nu, c_mu + s_mu)
}

/// パラメータ p（角形は xn∈[0,cD]、円形は θ∈[0,π]）における (Nu, Mu)。
fn nu_mu_at(inp: &CftBendingInput, p: f64) -> (f64, f64) {
    if inp.circular {
        circular_nu_mu(inp, p)
    } else {
        angular_nu_mu(inp, p)
    }
}

/// CFT **短柱**の N-M 相互作用による終局曲げ耐力 `Mu` [N·mm]（CFT 指針）。
///
/// `n_design`: 設計軸力 [N]（**圧縮正**）。`ncu1`: 短柱の軸圧縮終局耐力 [N]、
/// `ntu`: 軸引張終局耐力の**大きさ** [N]（引張は −ntu に対応）。
///
/// 中立軸をパラメータとする耐力曲線（角形 xn∈[0,cD]、円形 θ∈[0,π]）を軸力に整合させて
/// Mu を求め、曲線の N 範囲外（中立軸がコンクリート断面外）は端点と (Ncu1, 0)・(−Ntu, 0) を
/// 直線補間する。不正入力（せい・板厚・Fc・Fy のいずれか 0 以下）は 0.0。
pub fn cft_short_column_mu(inp: &CftBendingInput, n_design: f64, ncu1: f64, ntu: f64) -> f64 {
    if inp.d_steel <= 0.0 || inp.c_d <= 0.0 || inp.t <= 0.0 || inp.fc <= 0.0 || inp.fy <= 0.0 {
        return 0.0;
    }
    let p_max = if inp.circular { PI } else { inp.c_d };
    let (n_lo, m_lo) = nu_mu_at(inp, 0.0); // 圧縮縁ゼロ（最小軸力側）
    let (n_hi, m_hi) = nu_mu_at(inp, p_max); // 全圧縮側（最大軸力側）

    if n_design >= n_hi {
        // 曲線上端 → (Ncu1, 0) を直線補間。
        if ncu1 > n_hi {
            (m_hi * (ncu1 - n_design) / (ncu1 - n_hi)).max(0.0)
        } else {
            0.0
        }
    } else if n_design <= n_lo {
        // 曲線下端 → (−Ntu, 0) を直線補間。
        let n_tension = -ntu;
        if n_lo > n_tension {
            (m_lo * (n_design - n_tension) / (n_lo - n_tension)).max(0.0)
        } else {
            0.0
        }
    } else {
        // 曲線内: Nu(p)=n_design となる p を二分法で求める（Nu は p に単調増加）。
        let mut lo = 0.0;
        let mut hi = p_max;
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            if nu_mu_at(inp, mid).0 < n_design {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (_, mu) = nu_mu_at(inp, 0.5 * (lo + hi));
        mu.max(0.0)
    }
}

/// 座屈補助軸力 `Nk = π²·(cE'·cI/5 + sE·sI)/lk²`（CFT 指針、長柱）。
/// `cE' = (3.32·√Fc + 6.90)×10³`。`lk ≤ 0` は `f64::INFINITY`（座屈しない）。
pub fn cft_nk(c_inertia: f64, s_inertia: f64, s_young: f64, fc: f64, lk: f64) -> f64 {
    if lk <= 0.0 {
        return f64::INFINITY;
    }
    let c_young = (3.32 * fc.max(0.0).sqrt() + 6.90) * 1.0e3;
    PI * PI * (c_young * c_inertia.max(0.0) / 5.0 + s_young.max(0.0) * s_inertia.max(0.0))
        / (lk * lk)
}

/// CFT **中柱・長柱**の N-M 相互作用の算定入力（CFT 指針、中柱・長柱）。
#[derive(Clone, Copy, Debug)]
pub struct CftLongMediumInput {
    /// 短柱 N-M と同じ断面諸元。
    pub bending: CftBendingInput,
    /// 長柱なら true（座屈長さが断面せいの 12 倍超）、中柱なら false。
    pub is_long: bool,
    /// 充填コンクリートの座屈軸耐力 cNcr [N]（[`super::cft::cft_concrete_buckling_axial`]）。
    pub c_ncr: f64,
    /// 充填コンクリートの規準化細長比 cλ1（[`super::cft::cft_concrete_slenderness`]）。
    pub c_lambda1: f64,
    /// 座屈補助軸力 Nk [N]（[`cft_nk`]）。
    pub nk: f64,
    /// 当該分類の軸圧縮終局耐力 [N]（中柱 Ncu2 / 長柱 Ncu3。Case2 の線形補間に用いる）。
    pub ncu_axial: f64,
    /// 軸引張終局耐力の大きさ Ntu [N]。
    pub ntu: f64,
}

/// CFT **中柱・長柱**の N-M 相互作用による終局曲げ耐力 `Mu` [N·mm]
/// （CFT 指針、長柱 / 中柱）。
///
/// ```text
/// R    = (1 − cNcu/Nk)^(1/CM)         （CM=1、長柱の曲げ低減。0 未満は 0）
/// sMu0 = 鋼管の曲げのみ終局耐力       （円形 4r2²t·Fy / 角形 B·t(D−t)Fy + t·cD²/2·Fy）
/// cMu  = max(0, 4·cN/(0.9cNcr)·(1 − cN/(0.9cNcr))·cMmax)
/// cMmax= Cb/(Cb + cλ1²)·cMmax0,  Cb = 0.923 − 0.0045·Fc
/// cMmax0 = Fc·cD³/8（角形） / Fc·cD³/12（円形）
/// Case1 (N ≤ cNcr):  Mu = cMu(N) + sMu0·R
/// Case2 (N > cNcr):
///   長柱・円形: θ = π/2 + (N−cNcr)/(4r2t·Fy), Mu = 4r2²t·sinθ·Fy·R
///   中柱・角形長柱: Mu = sMu0·(1 − (N−cNcu)/(Ncu−cNcu))·R   （Ncu = Ncu2/Ncu3）
/// ```
/// 中立軸がコンクリート断面外（N < 0）は (0, sMu0·R) と (−Ntu, 0) を直線補間する。
/// 不正入力（せい・板厚・Fc・Fy のいずれか 0 以下）は 0.0。
pub fn cft_long_medium_column_mu(inp: &CftLongMediumInput, n_design: f64) -> f64 {
    let b = &inp.bending;
    if b.d_steel <= 0.0 || b.c_d <= 0.0 || b.t <= 0.0 || b.fc <= 0.0 || b.fy <= 0.0 {
        return 0.0;
    }
    // 長柱の曲げ低減係数 R = (1 − cNcr/Nk)（CM=1）。
    let r_factor = if inp.nk.is_finite() && inp.nk > 0.0 {
        (1.0 - inp.c_ncr / inp.nk).max(0.0)
    } else {
        1.0 // Nk=∞（座屈しない）は低減なし
    };
    // 鋼管の曲げのみ終局耐力 sMu0。
    let s_mu0 = if b.circular {
        let r2 = (b.d_steel - b.t) / 2.0;
        4.0 * r2 * r2 * b.t * b.fy
    } else {
        b.b_steel * b.t * (b.d_steel - b.t) * b.fy + b.t * b.c_d * b.c_d / 2.0 * b.fy
    };
    // 充填コンクリートの最大曲げ耐力 cMmax。
    let cb = 0.923 - 0.0045 * b.fc;
    let cmmax0 = if b.circular {
        b.fc * b.c_d.powi(3) / 12.0
    } else {
        b.fc * b.c_d.powi(3) / 8.0
    };
    let denom_cb = cb + inp.c_lambda1 * inp.c_lambda1;
    let cmmax = if denom_cb > 0.0 {
        cb / denom_cb * cmmax0
    } else {
        0.0
    };
    // 充填コンクリートの曲げ耐力（軸力 cn における放物線）。
    let c_ncr09 = 0.9 * inp.c_ncr;
    let cmu = |cn: f64| -> f64 {
        if c_ncr09 <= 0.0 {
            return 0.0;
        }
        (4.0 * cn / c_ncr09 * (1.0 - cn / c_ncr09) * cmmax).max(0.0)
    };

    if n_design < 0.0 {
        // 引張側: (0, sMu0·R) → (−Ntu, 0) を直線補間。
        let mu0 = s_mu0 * r_factor;
        let n_tension = -inp.ntu;
        if 0.0 > n_tension {
            (mu0 * (n_design - n_tension) / (0.0 - n_tension)).max(0.0)
        } else {
            0.0
        }
    } else if n_design <= inp.c_ncr {
        // Case 1: コンクリート放物線 + 鋼管の曲げのみ耐力（低減後）。
        (cmu(n_design) + s_mu0 * r_factor).max(0.0)
    } else if inp.is_long && b.circular {
        // Case 2（長柱・円形）: 鋼管長柱の θ パラメトリック。
        let r2 = (b.d_steel - b.t) / 2.0;
        let denom = 4.0 * r2 * b.t * b.fy;
        if denom <= 0.0 {
            return 0.0;
        }
        let theta = (PI / 2.0 + (n_design - inp.c_ncr) / denom).min(PI);
        (4.0 * r2 * r2 * b.t * theta.sin() * b.fy * r_factor).max(0.0)
    } else {
        // Case 2（中柱、または角形長柱）: Ncu への線形低減。
        if inp.ncu_axial > inp.c_ncr {
            (s_mu0 * (1.0 - (n_design - inp.c_ncr) / (inp.ncu_axial - inp.c_ncr)) * r_factor)
                .max(0.0)
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 角形 CFT-□400×400×12, Fc=30, Fy=325。cD=cB=376。
    fn box_input() -> CftBendingInput {
        CftBendingInput {
            circular: false,
            d_steel: 400.0,
            b_steel: 400.0,
            c_d: 376.0,
            c_b: 376.0,
            t: 12.0,
            fc: 30.0,
            fy: 325.0,
        }
    }

    /// 円形 CFT-φ400×12, Fc=30, Fy=325。cD=376。
    fn pipe_input() -> CftBendingInput {
        CftBendingInput {
            circular: true,
            d_steel: 400.0,
            b_steel: 400.0,
            c_d: 376.0,
            c_b: 376.0,
            t: 12.0,
            fc: 30.0,
            fy: 325.0,
        }
    }

    #[test]
    fn test_angular_nu_mu_endpoints_handcalc() {
        let inp = box_input();
        // xn=cD/2（中立軸中央）: sNu=0（軸力は主にコンクリート）。
        let (nu, mu) = angular_nu_mu(&inp, inp.c_d / 2.0);
        let xn = inp.c_d / 2.0;
        let c_nu = xn * inp.c_b * inp.fc;
        let s_nu = 2.0 * inp.t * (2.0 * xn - inp.c_d) * inp.fy; // =0
        assert!((nu - (c_nu + s_nu)).abs() < 1e-3);
        assert!(s_nu.abs() < 1e-9);
        // Mu は正（フランジ＋ウェブ＋コンクリート）。
        assert!(mu > 0.0);
    }

    #[test]
    fn test_angular_web_term_uses_fy() {
        // ウェブ項 2t·xn·(cD−xn)·Fy が sMu に含まれることを確認（Fc ではなく Fy）。
        let inp = box_input();
        let xn = inp.c_d / 3.0;
        let (_, mu) = angular_nu_mu(&inp, xn);
        let c_mu = 0.5 * xn * inp.c_b * (inp.c_d - xn) * inp.fc;
        let s_mu_flange = inp.b_steel * inp.t * (inp.d_steel - inp.t) * inp.fy;
        let s_mu_web = 2.0 * inp.t * xn * (inp.c_d - xn) * inp.fy;
        assert!((mu - (c_mu + s_mu_flange + s_mu_web)).abs() < 1e-3);
    }

    #[test]
    fn test_circular_nu_mu_symmetry() {
        // θ=π/2（中立軸が中心）: cNu = r1²·(π/2)·cσcB、sNu = 2r2t·(β1·π/2 − β2·(−π/2))·Fy。
        let inp = pipe_input();
        let (nu, mu) = circular_nu_mu(&inp, PI / 2.0);
        assert!(nu.is_finite() && mu > 0.0);
        // θ=0（圧縮なし）で cNu=cMu=0。
        let (n0, m0) = circular_nu_mu(&inp, 0.0);
        assert!(m0.abs() < 1e-6 || m0 >= 0.0);
        assert!(n0 < nu, "θ=0 の軸力は θ=π/2 より小さい");
    }

    #[test]
    fn test_cft_short_column_mu_curve_and_interp() {
        let inp = box_input();
        let ncu1 = inp.c_d * inp.c_b * inp.fc + (400.0 * 400.0 - inp.c_d * inp.c_b) * inp.fy;
        let ntu = (400.0 * 400.0 - inp.c_d * inp.c_b) * inp.fy;

        // 中央付近の軸力で Mu 正。
        let n_mid = 0.3 * ncu1;
        let mu_mid = cft_short_column_mu(&inp, n_mid, ncu1, ntu);
        assert!(mu_mid > 0.0);

        // 中心圧縮（N=Ncu1）で Mu→0。
        let mu_at_ncu1 = cft_short_column_mu(&inp, ncu1, ncu1, ntu);
        assert!(mu_at_ncu1.abs() < 1e-3, "Mu(Ncu1)={mu_at_ncu1}");

        // 中心引張（N=−Ntu）で Mu→0。
        let mu_at_ntu = cft_short_column_mu(&inp, -ntu, ncu1, ntu);
        assert!(mu_at_ntu.abs() < 1e-3, "Mu(−Ntu)={mu_at_ntu}");

        // 高圧縮域は中央より Mu 小（相関曲線の山型）。
        let mu_high = cft_short_column_mu(&inp, 0.85 * ncu1, ncu1, ntu);
        assert!(mu_high < mu_mid, "mu_high={mu_high} mu_mid={mu_mid}");
    }

    #[test]
    fn test_cft_short_column_mu_circular_positive() {
        let inp = pipe_input();
        let c_area = PI * inp.c_d * inp.c_d / 4.0;
        let s_area = PI * (400.0 * 400.0 - inp.c_d * inp.c_d) / 4.0;
        let ncu1 = c_area * inp.fc + (1.0 + 0.27) * s_area * inp.fy;
        let ntu = s_area * inp.fy;
        let mu = cft_short_column_mu(&inp, 0.3 * ncu1, ncu1, ntu);
        assert!(mu > 0.0);
    }

    #[test]
    fn test_cft_short_column_mu_invalid_zero() {
        let mut bad = box_input();
        bad.fc = 0.0;
        assert_eq!(cft_short_column_mu(&bad, 1000.0, 1.0e6, 1.0e6), 0.0);
    }

    #[test]
    fn test_cft_nk_handcalc() {
        // Nk = π²(cE'·cI/5 + sE·sI)/lk²、cE'=(3.32√30+6.90)×10³。
        let (c_i, s_i, s_e, fc, lk) = (1.0e9, 5.0e8, 205000.0, 30.0, 5000.0);
        let nk = cft_nk(c_i, s_i, s_e, fc, lk);
        let c_e = (3.32 * 30.0_f64.sqrt() + 6.90) * 1.0e3;
        let hand = PI * PI * (c_e * c_i / 5.0 + s_e * s_i) / (lk * lk);
        assert!((nk - hand).abs() / hand < 1e-9, "Nk={nk} vs {hand}");
        // lk=0 は無限大（座屈しない）。
        assert!(cft_nk(c_i, s_i, s_e, fc, 0.0).is_infinite());
    }

    fn long_input(circular: bool, is_long: bool) -> (CftLongMediumInput, f64, f64) {
        let bending = if circular { pipe_input() } else { box_input() };
        // 代表値（cNcr は座屈で短柱 cNc より小さい）。
        let c_area = if circular {
            PI * bending.c_d * bending.c_d / 4.0
        } else {
            bending.c_d * bending.c_b
        };
        let s_area = if circular {
            PI * (400.0 * 400.0 - bending.c_d * bending.c_d) / 4.0
        } else {
            400.0 * 400.0 - bending.c_d * bending.c_b
        };
        let c_ncr = 0.6 * c_area * bending.fc; // 座屈低減の代表
        let ncu_axial = c_ncr + 0.8 * s_area * bending.fy;
        let ntu = s_area * bending.fy;
        let nk = 3.0 * c_ncr; // Nk > cNcr（R>0）
        (
            CftLongMediumInput {
                bending,
                is_long,
                c_ncr,
                c_lambda1: 1.0,
                nk,
                ncu_axial,
                ntu,
            },
            c_ncr,
            ntu,
        )
    }

    #[test]
    fn test_cft_long_medium_mu_continuity_at_cncr() {
        // Case1/Case2 の境界 N=cNcr で連続（いずれも sMu0·R）。
        for (circular, is_long) in [(true, true), (false, true), (false, false), (true, false)] {
            let (inp, c_ncr, _) = long_input(circular, is_long);
            let lo = cft_long_medium_column_mu(&inp, c_ncr - 1.0);
            let hi = cft_long_medium_column_mu(&inp, c_ncr + 1.0);
            assert!(
                (lo - hi).abs() / hi.max(1.0) < 1e-2,
                "境界不連続 circular={circular} is_long={is_long}: lo={lo} hi={hi}"
            );
        }
    }

    #[test]
    fn test_cft_long_medium_mu_endpoints_and_reduction() {
        let (inp, _, ntu) = long_input(true, true);
        // 中心引張 N=−Ntu で Mu→0。
        assert!(cft_long_medium_column_mu(&inp, -ntu).abs() < 1e-3);
        // 圧縮側で Mu 正。
        assert!(cft_long_medium_column_mu(&inp, 0.3 * inp.ncu_axial) > 0.0);
        // Nk を小さくする（座屈が厳しい）と R が下がり Mu が減少する。
        let mut stiff = inp;
        stiff.nk = 10.0 * inp.c_ncr; // R 大
        let mut slender = inp;
        slender.nk = 1.2 * inp.c_ncr; // R 小
        let n = 0.2 * inp.c_ncr;
        assert!(
            cft_long_medium_column_mu(&slender, n) < cft_long_medium_column_mu(&stiff, n),
            "細長い方が Mu 小のはず"
        );
    }

    #[test]
    fn test_cft_medium_less_than_short() {
        // 中柱の N-M 曲げ耐力は座屈低減により短柱より小さい（同一断面・同一 N）。
        let inp = box_input();
        let c_area = inp.c_d * inp.c_b;
        let s_area = 400.0 * 400.0 - c_area;
        let ncu1 = c_area * inp.fc + s_area * inp.fy;
        let ntu = s_area * inp.fy;
        let short = cft_short_column_mu(&inp, 0.3 * ncu1, ncu1, ntu);

        let (med_inp, _, _) = long_input(false, false);
        let medium = cft_long_medium_column_mu(&med_inp, 0.3 * med_inp.ncu_axial);
        assert!(
            medium > 0.0 && medium < short,
            "medium={medium} short={short}"
        );
    }
}
