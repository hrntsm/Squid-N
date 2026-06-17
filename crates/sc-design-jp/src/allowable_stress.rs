use sc_core::ids::ElemId;

pub struct AllowableStressResult {
    pub elem: ElemId,
    pub long_term_ratio: f64,
    pub short_term_ratio: f64,
    pub ok_long: bool,
    pub ok_short: bool,
}

pub fn allowable_steel_stress(grade: &str, long_term: bool) -> f64 {
    let base = match grade {
        "SS400" | "SS490" => 235.0,
        "SN400" => 235.0,
        "SN490" => 325.0,
        _ => 235.0,
    };
    if long_term {
        base / 1.5
    } else {
        base
    }
}

pub fn allowable_concrete_stress(fc: f64, long_term: bool) -> f64 {
    let fc_n = fc / 1.0;
    if long_term {
        fc_n / 3.0
    } else {
        fc_n / 1.5
    }
}

/// Combined stress ratio: (axial/allowable_axial) + (bending/allowable_bending) ≤ 1.0
pub fn combined_stress_ratio(
    axial_stress: f64,
    bending_stress: f64,
    allowable_axial: f64,
    allowable_bending: f64,
) -> f64 {
    let axial_ratio = if allowable_axial != 0.0 {
        axial_stress / allowable_axial
    } else {
        0.0
    };
    let bend_ratio = if allowable_bending != 0.0 {
        bending_stress / allowable_bending
    } else {
        0.0
    };
    axial_ratio + bend_ratio
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_ss400_allowable() {
        let fa = allowable_steel_stress("SS400", true);
        assert!((fa - 235.0 / 1.5).abs() < 1e-6);
    }
}
