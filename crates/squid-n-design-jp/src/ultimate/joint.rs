//! 鉄筋コンクリート造柱梁接合部の**終局耐力**
//! （靭性保証型耐震設計指針 RC 柱梁接合部の終局耐力）。
//!
//! # 位置付け
//! [`crate::rc::joint`] が許容応力度検定（RC 規準 15 条）を扱うのに対し、本モジュールは
//! 終局検定（06 章）における接合部の終局せん断耐力 `Vju` と設計用接合部せん断力
//! `Qdu` の比（余裕率 `Vju/Qdu`）を算定する純関数である。節点まわりの応力集計
//! （どの梁・柱を対象とするか、T・T′・Qcu の算定）は呼び出し側
//! （[`crate::joint_wiring`]）の責務とする。
//!
//! # 準拠する規準・出典（要・原典照合、`specs/原典照合リスト.md`）
//! - `Vju = κ·φ·Fj·bj·Dj`, `Fj = 0.8·Fc^0.7` [N/mm²], `Qdu = α·(T + T′ − Qcu)`:
//!   日本建築学会「鉄筋コンクリート造建物の靭性保証型耐震設計指針・同解説」
//!   （RC 柱梁接合部の終局耐力）。

use crate::rc::joint::JointShape;

/// 接合部形状による係数 κ（靭性保証型耐震設計指針）。
///
/// - 十字形（[`JointShape::Cross`]）: 1.0
/// - ト形・T形（[`JointShape::Knee`]・[`JointShape::Tee`]）: 0.7
/// - L形（[`JointShape::Corner`]）: 0.4
pub fn joint_kappa(shape: JointShape) -> f64 {
    match shape {
        JointShape::Cross => 1.0,
        JointShape::Tee | JointShape::Knee => 0.7,
        JointShape::Corner => 0.4,
    }
}

/// 接合部コンクリートのせん断終局強度 Fj = 0.8·Fc^0.7 [N/mm²]（靭性指針）。
/// `Fc ≤ 0` の不正入力は 0 を返す。
pub fn joint_fj(fc: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    0.8 * fc.powf(0.7)
}

/// RC 柱梁接合部の終局検定の入力（靭性保証型耐震設計指針）。
#[derive(Clone, Copy, Debug)]
pub struct RcJointUltimateInput {
    /// 接合部の形状区分（κ の算定に用いる）。
    pub shape: JointShape,
    /// 直交梁の有無による補正係数 φ（両側直交梁付き=1.0、上記外=0.85）。
    pub phi: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]（Fj の算定に用いる）。
    pub fc: f64,
    /// 接合部有効幅 bj [mm]（= bb + ba1 + ba2）。
    pub bj: f64,
    /// 下側柱せい Dj [mm]。
    pub dj: f64,
    /// 上端鉄筋引張力 T [N]（スラブ筋を含む）。
    pub t_top: f64,
    /// 下端鉄筋引張力 T′ [N]。
    pub t_bottom: f64,
    /// 上下柱の存在せん断力の平均 Qcu [N]。
    pub qcu: f64,
    /// 接合部せん断力の割増率 α（既定 1.0）。
    pub alpha: f64,
}

/// RC 柱梁接合部の終局検定の結果。
#[derive(Clone, Copy, Debug)]
pub struct RcJointUltimateResult {
    /// 形状から計算される終局せん断耐力 Vju [N]。
    pub vju: f64,
    /// 設計用接合部せん断力 Qdu [N]。
    pub qdu: f64,
    /// 余裕率 Vju/Qdu（≥ 1.0 で OK）。`Qdu ≤ 0` のときは `f64::INFINITY`。
    pub margin: f64,
    /// κ（形状係数）。
    pub kappa: f64,
    /// Fj（接合部せん断終局強度）[N/mm²]。
    pub fj: f64,
}

/// RC 柱梁接合部の終局耐力 `Vju` と設計用せん断力 `Qdu` を算定する
/// （靭性保証型耐震設計指針）。
///
/// ```text
/// Vju = κ·φ·Fj·bj·Dj
/// Fj  = 0.8·Fc^0.7
/// Qdu = α·(T + T′ − Qcu)
/// 余裕率 = Vju/Qdu
/// ```
/// `Qdu ≤ 0`（設計用せん断力が非正）のときは余裕率を `f64::INFINITY` とする。
pub fn rc_joint_ultimate(inp: &RcJointUltimateInput) -> RcJointUltimateResult {
    let kappa = joint_kappa(inp.shape);
    let fj = joint_fj(inp.fc);
    let vju = (kappa * inp.phi * fj * inp.bj.max(0.0) * inp.dj.max(0.0)).max(0.0);
    let qdu = (inp.alpha * (inp.t_top + inp.t_bottom - inp.qcu)).max(0.0);
    let margin = if qdu > 0.0 { vju / qdu } else { f64::INFINITY };
    RcJointUltimateResult {
        vju,
        qdu,
        margin,
        kappa,
        fj,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_kappa_by_shape() {
        assert_eq!(joint_kappa(JointShape::Cross), 1.0);
        assert_eq!(joint_kappa(JointShape::Tee), 0.7);
        assert_eq!(joint_kappa(JointShape::Knee), 0.7);
        assert_eq!(joint_kappa(JointShape::Corner), 0.4);
    }

    #[test]
    fn test_joint_fj_matches_handcalc() {
        assert!((joint_fj(24.0) - 0.8 * 24.0_f64.powf(0.7)).abs() < 1e-9);
        assert_eq!(joint_fj(0.0), 0.0);
    }

    fn sample() -> RcJointUltimateInput {
        RcJointUltimateInput {
            shape: JointShape::Cross,
            phi: 1.0,
            fc: 24.0,
            bj: 600.0,
            dj: 600.0,
            t_top: 1_200_000.0,
            t_bottom: 1_200_000.0,
            qcu: 300_000.0,
            alpha: 1.0,
        }
    }

    #[test]
    fn test_rc_joint_ultimate_matches_handcalc() {
        let inp = sample();
        let r = rc_joint_ultimate(&inp);
        let fj = 0.8 * 24.0_f64.powf(0.7);
        let vju = 1.0 * 1.0 * fj * 600.0 * 600.0;
        let qdu = 1.0 * (1_200_000.0 + 1_200_000.0 - 300_000.0);
        assert!((r.vju - vju).abs() < 1e-3, "Vju={} vs {}", r.vju, vju);
        assert!((r.qdu - qdu).abs() < 1e-3, "Qdu={} vs {}", r.qdu, qdu);
        assert!((r.margin - vju / qdu).abs() < 1e-9);
    }

    #[test]
    fn test_rc_joint_ultimate_shape_affects_vju() {
        // 十字形（κ=1.0）は L 形（κ=0.4）より Vju が大きい。
        let cross = rc_joint_ultimate(&sample());
        let mut l = sample();
        l.shape = JointShape::Corner;
        let corner = rc_joint_ultimate(&l);
        assert!(cross.vju > corner.vju);
        assert!((corner.vju / cross.vju - 0.4).abs() < 1e-9);
    }

    #[test]
    fn test_rc_joint_ultimate_nonpositive_qdu_is_infinite_margin() {
        let mut inp = sample();
        // T+T′ < Qcu → Qdu クランプで 0 → 余裕率無限大。
        inp.qcu = 5_000_000.0;
        let r = rc_joint_ultimate(&inp);
        assert_eq!(r.qdu, 0.0);
        assert!(r.margin.is_infinite());
    }
}
