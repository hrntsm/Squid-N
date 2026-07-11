pub const GRAVITY_MM_S2: f64 = 9_806.65;

/// コンクリートの種類（単位体積重量表の行。RESP-D マニュアル「柱梁自重」）。
/// 「04 断面検定」の許容応力度低減（軽量1種・2種は普通コンクリートの 0.9 倍）にも用いる。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ConcreteClass {
    #[default]
    Normal,
    Lightweight1,
    Lightweight2,
}

/// コンクリート系構造の区分（γC/γRC/γSRC の列に対応）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConcreteComposition {
    /// 無筋（気乾単位体積重量 γC）
    Plain,
    /// 鉄筋コンクリート（γRC = γC + 1.0）
    #[default]
    Rc,
    /// 鉄骨鉄筋コンクリート（γSRC = γC + 2.0）
    Src,
}

/// コンクリートの単位体積重量 [kN/m³]。
/// RESP-D マニュアル「柱梁自重」の表（設計基準強度 Fc・種類・構造区分ごと）による。
/// 軽量コンクリートで表の範囲を超える Fc は最上段の値で頭打ちとする。
pub fn concrete_unit_weight_kn_m3(fc: f64, class: ConcreteClass, comp: ConcreteComposition) -> f64 {
    let gamma_c = match class {
        ConcreteClass::Normal => {
            if fc <= 36.0 {
                23.0
            } else if fc <= 48.0 {
                23.5
            } else if fc <= 120.0 {
                24.0
            } else {
                24.5
            }
        }
        ConcreteClass::Lightweight1 => {
            if fc <= 27.0 {
                19.0
            } else {
                20.0
            }
        }
        ConcreteClass::Lightweight2 => 17.0,
    };
    // 軽量1種 27<Fc≦36 は γRC=22.0（+2.0）と表の増分が他と異なるため個別に扱う。
    match (class, comp) {
        (ConcreteClass::Lightweight1, ConcreteComposition::Rc) if fc > 27.0 => 22.0,
        (ConcreteClass::Lightweight1, ConcreteComposition::Src) if fc > 27.0 => 23.0,
        (_, ConcreteComposition::Plain) => gamma_c,
        (_, ConcreteComposition::Rc) => gamma_c + 1.0,
        (_, ConcreteComposition::Src) => gamma_c + 2.0,
    }
}

/// 鋼材の単位体積重量 [kN/m³]（RESP-D マニュアル: γs = 77 kN/m³）。
pub const STEEL_UNIT_WEIGHT_KN_M3: f64 = 77.0;

pub mod to_internal {
    pub fn length_m(m: f64) -> f64 {
        m * 1_000.0
    }
    pub fn force_kn(kn: f64) -> f64 {
        kn * 1_000.0
    }
    pub fn line_load_kn_per_m(v: f64) -> f64 {
        v
    }
    pub fn area_load_kn_per_m2(v: f64) -> f64 {
        v / 1_000.0
    }
    pub fn stress_n_per_mm2(v: f64) -> f64 {
        v
    }
    pub fn mass_density_g_per_cm3(v: f64) -> f64 {
        v * 1.0e-9
    }
    pub fn unit_weight_kn_per_m3(v: f64) -> f64 {
        v * 1.0e-6
    }
    pub fn weight_n_to_mass(w_n: f64) -> f64 {
        w_n / super::GRAVITY_MM_S2
    }
    /// 単位体積重量 [kN/m³] → 質量密度 [ton/mm³]（内部単位系 N-mm-s）。
    /// 例: γRC=24.0 kN/m³ → 2.4473e-9 ton/mm³。
    pub fn mass_density_from_unit_weight_kn_m3(v: f64) -> f64 {
        unit_weight_kn_per_m3(v) / super::GRAVITY_MM_S2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_unit_conversions() {
        assert_relative_eq!(to_internal::length_m(6.0), 6000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::line_load_kn_per_m(10.0),
            10.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(to_internal::force_kn(50.0), 50000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::stress_n_per_mm2(24.0),
            24.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::mass_density_g_per_cm3(2.4),
            2.4e-9,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::unit_weight_kn_per_m3(24.0),
            2.4e-5,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::weight_n_to_mass(1.0e6),
            101.971_621_297_792_82,
            max_relative = 1e-12
        );
    }

    #[test]
    fn test_concrete_unit_weight_table() {
        use ConcreteClass::*;
        use ConcreteComposition::*;
        // 普通コンクリート（マニュアル表の代表値）
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Plain), 23.0);
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Rc), 24.0);
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Src), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(42.0, Normal, Rc), 24.5);
        assert_eq!(concrete_unit_weight_kn_m3(60.0, Normal, Rc), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(100.0, Normal, Rc), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(150.0, Normal, Rc), 25.5);
        // 軽量コンクリート
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Lightweight1, Rc), 20.0);
        assert_eq!(concrete_unit_weight_kn_m3(30.0, Lightweight1, Rc), 22.0);
        assert_eq!(concrete_unit_weight_kn_m3(30.0, Lightweight1, Src), 23.0);
        assert_eq!(concrete_unit_weight_kn_m3(21.0, Lightweight2, Rc), 18.0);
    }

    #[test]
    fn test_mass_density_from_unit_weight() {
        // γRC=24 kN/m³ → 24e-6 N/mm³ / 9806.65 mm/s² ≈ 2.4473e-9 t/mm³
        let rho = to_internal::mass_density_from_unit_weight_kn_m3(24.0);
        assert_relative_eq!(rho, 24.0e-6 / GRAVITY_MM_S2, max_relative = 1e-12);
        assert!((rho - 2.4473e-9).abs() / rho < 1e-3);
    }
}
