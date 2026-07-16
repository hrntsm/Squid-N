//! 異形棒鋼（鉄筋）の呼び径と単位質量。
//!
//! - [`rebar_unit_mass_kg_per_m`] — 呼び径ごとの単位質量 [kg/m]
//! - [`rebar_weight_t`] — 鉄筋長さ → 重量 [t] の換算

/// 異形棒鋼の単位質量表 (呼び径 [mm], 単位質量 [kg/m])（JIS G 3112）。
const UNIT_MASS_TABLE: &[(f64, f64)] = &[
    (10.0, 0.560),
    (13.0, 0.995),
    (16.0, 1.56),
    (19.0, 2.25),
    (22.0, 3.04),
    (25.0, 3.98),
    (29.0, 5.04),
    (32.0, 6.23),
    (35.0, 7.51),
    (38.0, 8.95),
    (41.0, 10.5),
    (51.0, 15.9),
];

/// 呼び径 `dia` [mm] の異形棒鋼の単位質量 [kg/m]。
///
/// JIS G 3112 の単位質量表（D10〜D51）から呼び径 ±1mm 以内の一致を探し、
/// 表に無い径（高強度せん断補強筋の中間径等）は公称断面積
/// `π/4·d²` × 鋼材密度 7.85e-3 [kg/(mm²·m)] で計算する。
pub fn rebar_unit_mass_kg_per_m(dia: f64) -> f64 {
    for &(d, m) in UNIT_MASS_TABLE {
        if (dia - d).abs() <= 1.0 {
            return m;
        }
    }
    std::f64::consts::PI / 4.0 * dia * dia * 7.85e-3
}

/// 鉄筋の総長さ [mm] × 呼び径 [mm] → 重量 [t]。
pub fn rebar_weight_t(total_length_mm: f64, dia: f64) -> f64 {
    // kg/m × m → kg → t
    rebar_unit_mass_kg_per_m(dia) * (total_length_mm / 1_000.0) / 1_000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unit_mass_table() {
        assert_eq!(rebar_unit_mass_kg_per_m(13.0), 0.995);
        assert_eq!(rebar_unit_mass_kg_per_m(22.0), 3.04);
        // 表に無い径は計算値（D14 相当: π/4·14²·7.85e-3 ≈ 1.208）
        let m = rebar_unit_mass_kg_per_m(14.5);
        assert!((m - std::f64::consts::PI / 4.0 * 14.5 * 14.5 * 7.85e-3).abs() < 1e-12);
    }

    #[test]
    fn test_weight_conversion() {
        // D13 × 1000m = 995 kg = 0.995 t
        let w = rebar_weight_t(1_000_000.0, 13.0);
        assert!((w - 0.995).abs() < 1e-9);
    }
}
