//! 材料に関する換算関数。
//!
//! - [`concrete_young_modulus`] — コンクリート強度 Fc からヤング係数 Ec を算定
//! - [`wall_shear_shape_factor`] — 耐震壁のせん断形状係数

use super::constants::{GAMMA_CONCRETE, KAPPA_RC};

/// コンクリート強度 Fc [N/mm²] からヤング係数 Ec [N/mm²] を算定する
/// （RC 規準の Ec=3.35·10⁴·(γ/24)²·(Fc/60)^(1/3)、γ=23 固定）。
pub fn concrete_young_modulus(fc: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    3.35e4 * (GAMMA_CONCRETE / 24.0).powi(2) * (fc / 60.0).powf(1.0 / 3.0)
}

/// 耐震壁（壁板＋両側柱＝平面 I 形断面）のせん断形状係数
/// （側柱付き壁を I 形断面とみなしたせん断形状係数。材料力学）。
///
/// κ = 3(1+ξ)/(5·(1−ξ³(1−η))²)·[η + ξ(1−η)·((15/8)(1−ξ²)² − ξ⁴·η)]
///
/// ξ・η の定義は原典ページに明示がないため、
/// ξ=壁板内法長さ/全長（側柱外面間）、η=壁厚/側柱幅 と仮定する
/// （式の読み・記号定義とも dev_docs/specs/原典照合リスト.md に要照合として登録）。
/// ξ=1（側柱なし＝矩形断面）で κ=1.2（=`KAPPA_RC`）に一致する。
/// 退化（非有限・非正）時は矩形の 1.2 にフォールバックする。
pub fn wall_shear_shape_factor(xi: f64, eta: f64) -> f64 {
    let xi = xi.clamp(0.0, 1.0);
    let eta = eta.clamp(1e-6, 1.0);
    let denom = 5.0 * (1.0 - xi.powi(3) * (1.0 - eta)).powi(2);
    if denom <= 1e-12 {
        return KAPPA_RC;
    }
    let bracket =
        eta + xi * (1.0 - eta) * ((15.0 / 8.0) * (1.0 - xi * xi).powi(2) - xi.powi(4) * eta);
    let k = 3.0 * (1.0 + xi) / denom * bracket;
    if k.is_finite() && k > 0.0 {
        k
    } else {
        KAPPA_RC
    }
}
