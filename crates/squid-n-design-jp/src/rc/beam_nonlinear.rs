//! 鉄筋コンクリート造梁の**非線形復元力特性**（RESP-D マニュアル「計算編 05
//! 非線形モデル」梁の曲げ・せん断・軸復元力特性）。
//!
//! # 位置付け
//! [`super::beam`] が許容応力度検定を扱うのに対し、本モジュールは非線形解析の
//! 材端剛塑性回転バネ（曲げトリリニア）・せん断ばね・軸ばねの骨格諸元を算定する
//! 純関数群である。曲げは (ひび割れ Mc, 降伏 My＋剛性低下率 αy) のトリリニア、
//! せん断はひび割れ Qc、軸は (引張ひび割れ Nct, 引張降伏 Nut, 圧縮降伏 Nuc)。
//!
//! # 準拠する規準・出典
//! - 曲げひび割れ Mc=κ·Fc·Ze（κ=0.56）: 技術基準解説書 P.621-623。
//! - 曲げ降伏時剛性低下率 αy（菅野式）: 梅村魁『鉄筋コンクリート建物の動的
//!   耐震設計法』P.106-108（[`squid_n_core::rc_capacity::rc_alpha_y_sugano`]）。
//! - 曲げ終局 My=0.9·at·σy·j: 技術基準解説書 P.623（[`squid_n_core::rc_capacity::rc_mu_simple`]）。
//! - せん断ひび割れ Qc=(0.061·(Fc+49)/(M/(Q·d)+1.7))·b·j: RESP-D 非線形モデル。
//! - 軸: Nct=κ·Fc·Ac（κ=0.56）, Nut=at·σy, Nuc=at·σy+Fc·(Ac−at)。

use squid_n_core::rc_capacity::{rc_alpha_y_sugano, rc_mu_simple, RcCapacityInput};

/// RC 梁の曲げひび割れ強度 Mc [N·mm]（RESP-D 非線形モデル）。
/// `Mc = κ·Fc·Ze`（κ=0.56、Ze=鉄筋を考慮した引張側断面係数 Ie/(D−g) または Ie/g）。
/// 不正入力（Fc・Ze のいずれかが 0 以下）は 0.0。
pub fn rc_beam_crack_moment(fc: f64, ze: f64) -> f64 {
    if fc <= 0.0 || ze <= 0.0 {
        return 0.0;
    }
    0.56 * fc * ze
}

/// RC 梁のせん断ひび割れ強度 Qc [N]（RESP-D 非線形モデル、トリリニア用）。
/// `Qc = (0.061·(Fc+49)/(M/(Q·d)+1.7))·b·j`。
/// 不正入力（Fc・b・j のいずれかが 0 以下）は 0.0。
pub fn rc_beam_shear_crack(fc: f64, m_over_qd: f64, b: f64, j: f64) -> f64 {
    if fc <= 0.0 || b <= 0.0 || j <= 0.0 {
        return 0.0;
    }
    (0.061 * (fc + 49.0) / (m_over_qd.max(0.0) + 1.7)) * b * j
}

/// RC 梁の軸復元力特性（引張ひび割れ・引張降伏・圧縮降伏）[N]（RESP-D 非線形モデル）。
/// - 引張ひび割れ `Nct = κ·Fc·Ac`（κ=0.56、引張正）
/// - 引張降伏 `Nut = at·σy`
/// - 圧縮降伏 `Nuc = at·σy + Fc·(Ac − at)`
#[derive(Clone, Copy, Debug)]
pub struct RcAxial {
    pub tension_crack: f64,
    pub tension_yield: f64,
    pub compression_yield: f64,
}

/// RC 梁の軸復元力特性を算定する。`ac`: コンクリート断面積、`at`: 鉄筋断面積、
/// `sigma_y`: 鉄筋降伏、`fc`: コンクリート強度。
pub fn rc_beam_axial(fc: f64, ac: f64, at: f64, sigma_y: f64) -> RcAxial {
    let nct = if fc > 0.0 && ac > 0.0 {
        0.56 * fc * ac
    } else {
        0.0
    };
    let nut = (at.max(0.0)) * sigma_y.max(0.0);
    let nuc = nut + fc.max(0.0) * (ac - at).max(0.0);
    RcAxial {
        tension_crack: nct,
        tension_yield: nut,
        compression_yield: nuc,
    }
}

/// RC 梁の曲げトリリニア骨格の算定入力。
#[derive(Clone, Copy, Debug)]
pub struct RcBeamBendingInput {
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 引張側断面係数 Ze [mm³]（鉄筋考慮、Ie/(D−g) 等。Mc 用）。
    pub ze: f64,
    /// 引張鉄筋断面積 at [mm²]（片側）。
    pub at: f64,
    /// 引張鉄筋比 pt（小数、αy 用）。
    pub pt: f64,
    /// 有効せい d [mm]。
    pub d_eff: f64,
    /// 全せい D [mm]（αy の d/D 用）。
    pub d_full: f64,
    /// 主筋降伏強度 σy [N/mm²]。
    pub sigma_y: f64,
    /// せん断スパン a = l0/2 [mm]（αy の a/D 用）。
    pub a_shear_span: f64,
    /// 鉄筋ヤング係数 Es [N/mm²]。
    pub es: f64,
    /// コンクリートヤング係数 Ec [N/mm²]（n=Es/Ec）。
    pub ec: f64,
}

/// RC 梁の曲げトリリニア骨格諸元。
#[derive(Clone, Copy, Debug)]
pub struct RcBeamBending {
    /// 曲げひび割れ強度 Mc [N·mm]。
    pub mc: f64,
    /// 曲げ降伏（＝終局）強度 My [N·mm]（=0.9·at·σy·j）。
    pub my: f64,
    /// 曲げ降伏時剛性低下率 αy（菅野式、無次元）。
    pub alpha_y: f64,
}

/// RC 梁の曲げトリリニア骨格（Mc・My・αy）を算定する（RESP-D 非線形モデル）。
pub fn rc_beam_bending(inp: &RcBeamBendingInput) -> RcBeamBending {
    let mc = rc_beam_crack_moment(inp.fc, inp.ze);
    let cap = RcCapacityInput {
        b: 1.0,
        d: inp.d_full,
        at: inp.at,
        d_eff: inp.d_eff,
        sigma_y: inp.sigma_y,
        fc: inp.fc.max(1e-9),
        pw: 0.0,
        sigma_wy: 0.0,
        clear_span: 1.0,
        sigma_0: 0.0,
    };
    let my = rc_mu_simple(&cap);
    let n = if inp.ec > 0.0 { inp.es / inp.ec } else { 15.0 };
    let a_over_d = if inp.d_full > 0.0 {
        inp.a_shear_span / inp.d_full
    } else {
        3.0
    };
    let d_over_full = if inp.d_full > 0.0 {
        inp.d_eff / inp.d_full
    } else {
        0.9
    };
    let alpha_y = rc_alpha_y_sugano(inp.pt, a_over_d, d_over_full, n);
    RcBeamBending { mc, my, alpha_y }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rc_beam_crack_moment() {
        // Mc = 0.56·24·(300·600²/6) = 0.56·24·1.8e7
        let ze = 300.0 * 600.0_f64.powi(2) / 6.0;
        let mc = rc_beam_crack_moment(24.0, ze);
        assert!((mc - 0.56 * 24.0 * ze).abs() < 1e-3);
        assert_eq!(rc_beam_crack_moment(0.0, ze), 0.0);
    }

    #[test]
    fn test_rc_beam_shear_crack_matches_handcalc() {
        let qc = rc_beam_shear_crack(24.0, 1.5, 300.0, 500.0);
        let hand = (0.061 * (24.0 + 49.0) / (1.5 + 1.7)) * 300.0 * 500.0;
        assert!((qc - hand).abs() < 1e-6, "Qc={qc} vs {hand}");
        assert!(qc > 0.0);
    }

    #[test]
    fn test_rc_beam_axial_matches_handcalc() {
        let ax = rc_beam_axial(24.0, 180_000.0, 2000.0, 345.0);
        assert!((ax.tension_crack - 0.56 * 24.0 * 180_000.0).abs() < 1e-3);
        assert!((ax.tension_yield - 2000.0 * 345.0).abs() < 1e-6);
        assert!(
            (ax.compression_yield - (2000.0 * 345.0 + 24.0 * (180_000.0 - 2000.0))).abs() < 1e-3
        );
        // 圧縮降伏 > 引張降伏（コンクリート寄与）。
        assert!(ax.compression_yield > ax.tension_yield);
    }

    #[test]
    fn test_rc_beam_bending_trilinear() {
        let inp = RcBeamBendingInput {
            fc: 24.0,
            ze: 300.0 * 600.0_f64.powi(2) / 6.0,
            at: 1935.0,
            pt: 0.008,
            d_eff: 540.0,
            d_full: 600.0,
            sigma_y: 345.0,
            a_shear_span: 1500.0,
            es: 205000.0,
            ec: 21000.0,
        };
        let b = rc_beam_bending(&inp);
        // My = 0.9·at·σy·(7·d_eff/8) の手計算一致。
        let j = 7.0 * 540.0 / 8.0;
        assert!((b.my - 0.9 * 1935.0 * 345.0 * j).abs() < 1e-3);
        // ひび割れ < 降伏（健全な折れ点順序）。
        assert!(b.mc > 0.0 && b.mc < b.my, "Mc={} My={}", b.mc, b.my);
        // αy は 0〜1 の妥当値。
        assert!(b.alpha_y > 0.0 && b.alpha_y < 1.0, "αy={}", b.alpha_y);
    }
}
