//! NewRC コンクリート構成則（RESP-D マニュアル「計算編 05 非線形モデル」
//! 柱ファイバーモデルのコンクリート応力-ひずみ関係）。
//!
//! # 構成則
//! 圧縮側は NewRC（有理式）モデル:
//! ```text
//! σc/σcB = (A·X + (D−1)·X²) / (1 + (A−2)·X + D·X²),  X = εc/εc0
//! εc0 = εo = 0.5243·(σB)^(1/4) × 10⁻³
//! Ec  = 4k·(σB/1000)^(1/3) × 10⁵ × (γ/2.4)²   （k=1.0）
//! A   = Ec·εc0/σcB,   D = α + β·σB   （α=1.50, β=1.68×10⁻³）
//! σcB = σp = 1.0·σB   （コンファインド効果は考慮しない）
//! ```
//! 上式は**工学単位系（kg/cm²）**で与えられるため、σB=Fc[N/mm²]×10.19716 として
//! A・D・εc0 を評価する。σc/σcB は無次元のため σc[N/mm²]=(比)·Fc[N/mm²] とし、
//! 初期接線は Ec[N/mm²]=Ec[kg/cm²]×0.0980665 に一致する。
//!
//! 引張側は Ec の弾性 → ひび割れ（ε_cr=ft/Ec）後は応力を保持しない脆性型とする。
//! 除荷・再載荷は最大経験ひずみへの原点指向割線（静的単調増加のプッシュオーバーでは
//! 履歴の影響は小さい）。RESP-D の Fc60 超・ユーザー定義は Bilinear 骨格へ切替える
//! 規定のため、本モデルは適用範囲（Fc≤60）でのみ用いること（呼び出し側で判定）。

use crate::uniaxial::UniaxialMaterial;

/// N/mm² → kgf/cm²（1 kgf/cm² = 0.0980665 N/mm²）。
const NMM2_TO_KGFCM2: f64 = 1.0 / 0.0980665;

/// コンクリート履歴の除荷則（RESP-D「05 非線形モデル」）。
/// 静的解析は逆行型（包絡線を可逆に辿る）、動的解析は原点指向型（最大経験ひずみ点
/// から原点への割線で除荷・再載荷）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum ConcreteHysteresis {
    /// 原点指向型（動的解析）。既定。
    #[default]
    OriginOriented,
    /// 逆行型（静的解析）。除荷・再載荷が圧縮包絡線を辿る。
    Retrace,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct NewRcState {
    strain: f64,
    stress: f64,
    tangent: f64,
    max_comp_strain: f64,
    max_tens_strain: f64,
    is_cracked: bool,
}

/// NewRC コンクリート構成則（圧縮 NewRC 有理式＋引張脆性）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ConcreteNewRc {
    /// コンクリート強度 Fc（σB）[N/mm²]。
    pub fc: f64,
    /// 引張強度 ft [N/mm²]。
    pub ft: f64,
    /// 初期接線 Ec [N/mm²]。
    pub ec: f64,
    /// NewRC の係数 A（無次元）。
    a: f64,
    /// NewRC の係数 D（無次元）。
    d_coef: f64,
    /// 圧縮強度時ひずみ εc0（正）。
    eps_c0: f64,
    /// 除荷則（静的=逆行型／動的=原点指向型）。
    #[serde(default)]
    hysteresis: ConcreteHysteresis,
    committed: NewRcState,
    trial: NewRcState,
}

impl ConcreteNewRc {
    /// `fc`,`ft` [N/mm²]、`gamma` は気乾単位体積重量 [t/m³]（既定 2.4 → (γ/2.4)²=1）。
    pub fn new(fc: f64, ft: f64) -> Self {
        Self::with_gamma(fc, ft, 2.4)
    }

    pub fn with_gamma(fc: f64, ft: f64, gamma: f64) -> Self {
        let sigma_b = fc.max(1e-6) * NMM2_TO_KGFCM2; // kg/cm²
        let eps_c0 = 0.5243 * sigma_b.powf(0.25) * 1e-3;
        let ec_kgf = 4.0 * 1.0 * (sigma_b / 1000.0).powf(1.0 / 3.0) * 1e5 * (gamma / 2.4).powi(2);
        let sigma_cb = sigma_b; // コンファインドなし
        let a = ec_kgf * eps_c0 / sigma_cb;
        let d_coef = 1.50 + 1.68e-3 * sigma_b;
        let ec = ec_kgf * 0.0980665; // N/mm²
        let init = NewRcState {
            strain: 0.0,
            stress: 0.0,
            tangent: ec,
            max_comp_strain: 0.0,
            max_tens_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ft,
            ec,
            a,
            d_coef,
            eps_c0,
            hysteresis: ConcreteHysteresis::default(),
            committed: init.clone(),
            trial: init,
        }
    }

    /// NewRC 圧縮包絡線の応力比 σc/σcB とその微分 d(比)/dX（X=εc/εc0）。
    fn envelope_ratio(&self, x: f64) -> (f64, f64) {
        let a = self.a;
        let d = self.d_coef;
        let num = a * x + (d - 1.0) * x * x;
        let den = 1.0 + (a - 2.0) * x + d * x * x;
        let ratio = num / den;
        let num_p = a + 2.0 * (d - 1.0) * x;
        let den_p = (a - 2.0) + 2.0 * d * x;
        let dratio = (num_p * den - num * den_p) / (den * den);
        (ratio, dratio)
    }

    /// 圧縮包絡線（strain ≤ 0）。(σ[N/mm²]（圧縮負）, 接線[N/mm²])。
    fn envelope_compression(&self, strain: f64) -> (f64, f64) {
        let x = (-strain) / self.eps_c0; // ≥0
        let (ratio, dratio) = self.envelope_ratio(x);
        let stress = -ratio * self.fc;
        // dσ/dε = (fc/εc0)·d(ratio)/dX（初期は Ec）。
        let tangent = (self.fc / self.eps_c0) * dratio;
        (stress, tangent.max(0.0))
    }

    fn eps_cr(&self) -> f64 {
        if self.ec > 0.0 {
            self.ft / self.ec
        } else {
            0.0
        }
    }
}

impl UniaxialMaterial for ConcreteNewRc {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = &self.committed;
        let (stress, tangent, max_comp, max_tens, cracked) = if strain <= 0.0 {
            // 圧縮側
            let mut max_comp = c.max_comp_strain;
            let (s, t) = if self.hysteresis == ConcreteHysteresis::Retrace {
                // 逆行型（静的解析）: 除荷・再載荷も圧縮包絡線を可逆に辿る。
                if strain < max_comp {
                    max_comp = strain;
                }
                self.envelope_compression(strain)
            } else if strain < c.max_comp_strain {
                // 原点指向型（動的）: 包絡線上（新最大圧縮ひずみ）。
                max_comp = strain;
                self.envelope_compression(strain)
            } else if c.max_comp_strain < 0.0 {
                // 原点指向型 除荷・再載荷: 最大経験圧縮ひずみ点への割線。
                let (sig_m, _) = self.envelope_compression(c.max_comp_strain);
                let ku = sig_m / c.max_comp_strain;
                (ku * strain, ku)
            } else {
                (self.ec * strain, self.ec)
            };
            (s, t, max_comp, c.max_tens_strain, c.is_cracked)
        } else {
            // 引張側（弾性→ひび割れ後は応力ゼロ・脆性）
            let eps_cr = self.eps_cr();
            let mut cracked = c.is_cracked;
            let mut max_tens = c.max_tens_strain;
            let (s, t) = if !cracked && strain <= eps_cr {
                (self.ec * strain, self.ec)
            } else {
                cracked = true;
                max_tens = max_tens.max(strain);
                (0.0, 0.0)
            };
            (s, t, c.max_comp_strain, max_tens, cracked)
        };
        self.trial = NewRcState {
            strain,
            stress,
            tangent,
            max_comp_strain: max_comp,
            max_tens_strain: max_tens,
            is_cracked: cracked,
        };
        (stress, tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }

    fn clone_box(&self) -> Box<dyn UniaxialMaterial> {
        Box::new(self.clone())
    }

    fn serialize_state(&self) -> Vec<u8> {
        bincode::serialize(self).expect("material serialize")
    }

    fn deserialize_state(&mut self, data: &[u8]) {
        if let Ok(de) = bincode::deserialize::<Self>(data) {
            *self = de;
        }
    }

    fn reference_stress(&self) -> f64 {
        self.fc
    }

    fn reference_strain(&self) -> f64 {
        self.eps_c0
    }

    fn set_concrete_hysteresis(&mut self, dynamic: bool) {
        self.hysteresis = if dynamic {
            ConcreteHysteresis::OriginOriented
        } else {
            ConcreteHysteresis::Retrace
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_newrc_peak_at_ec0() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        // ピーク（εc0）で σ=-fc（比=1）。
        let (stress, _) = c.envelope_compression(-c.eps_c0);
        assert_relative_eq!(stress, -30.0, max_relative = 1e-6);
    }

    #[test]
    fn test_newrc_initial_tangent_is_ec() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        let (_, t) = c.envelope_compression(-1e-9);
        // ε=0 近傍の接線は Ec。
        assert_relative_eq!(t, c.ec, max_relative = 1e-3);
        // Ec は常識的な範囲（普通コンクリート 2〜3×10⁴ N/mm² 程度）。
        assert!(c.ec > 2.0e4 && c.ec < 3.5e4, "Ec={}", c.ec);
    }

    #[test]
    fn test_newrc_eps_c0_reasonable() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        // εc0 は 0.002 前後（普通強度コンクリート）。
        assert!(c.eps_c0 > 0.0015 && c.eps_c0 < 0.0030, "εc0={}", c.eps_c0);
    }

    #[test]
    fn test_newrc_softening_after_peak() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        let (s_peak, _) = c.trial(-c.eps_c0);
        c.commit();
        let (s_post, _) = c.trial(-2.0 * c.eps_c0);
        // ピーク後は |σ| が低下（軟化）。
        assert!(
            s_post > s_peak,
            "post-peak stress should reduce magnitude: peak={s_peak}, post={s_post}"
        );
    }

    #[test]
    fn test_newrc_tension_cracks() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        let eps_cr = c.eps_cr();
        let (s_el, _) = c.trial(eps_cr * 0.5);
        assert!(s_el > 0.0);
        c.commit();
        let (s_cr, _) = c.trial(eps_cr * 2.0);
        // ひび割れ後は応力ゼロ（脆性）。
        assert_relative_eq!(s_cr, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_newrc_commit_revert() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        c.trial(-0.001);
        c.commit();
        c.trial(-0.003);
        c.revert();
        let (stress, _) = c.trial(-0.0005);
        assert!(stress < 0.0);
    }

    #[test]
    fn test_newrc_reference_values() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        assert_eq!(c.reference_stress(), 30.0);
        assert_relative_eq!(c.reference_strain(), c.eps_c0);
    }
}
