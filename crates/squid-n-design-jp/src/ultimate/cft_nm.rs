//! コンクリート充填鋼管（CFT）柱の**短柱の N-M 相互作用（曲げを伴う終局耐力）**
//! （RESP-D マニュアル「計算編 06 終局検定」CFT 柱の終局耐力 (3)A）。
//!
//! # 位置付け
//! [`super::cft`] が軸方向終局耐力（Ncu/Ntu）を扱うのに対し、本モジュールは軸方向力と
//! 曲げモーメントを同時に受ける**短柱**（座屈長さ ≤ 断面せいの 4 倍）の終局曲げ耐力
//! `Mu(N)` を CFT 指針に基づき算定する。中立軸位置をパラメータとする耐力曲線を軸力 N に
//! 整合させ、中立軸がコンクリート断面外の場合は軸圧縮耐力 Ncu1・軸引張耐力 Ntu との
//! 直線補間で求める。長柱・中柱の N-M 相互作用（Nk・CM・cMmax を用いる式）は今後の課題。
//!
//! # 準拠・出典（要・原典照合、`specs/原典照合リスト.md`）
//! - 日本建築学会「コンクリート充填鋼管構造設計指針」短柱の終局曲げ耐力。
//!
//! # 角形 sMu の第 2 項について（原典照合メモ）
//! マニュアル抽出では角形の `sMu = D·t·(D−t)·Fy + 2t·(cD−xn)·xn·Fc` と末尾が `Fc` だが、
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

/// CFT **短柱**の N-M 相互作用による終局曲げ耐力 `Mu` [N·mm]（RESP-D「06 終局検定」）。
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
}
