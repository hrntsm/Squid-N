pub struct CapacityCurve {
    pub points: Vec<(f64, f64)>,
}

pub struct ResponseSpectrum {
    pub periods: Vec<f64>,
    pub accelerations: Vec<f64>,
}

pub struct PerformancePoint {
    pub sd: f64,
    pub sa: f64,
    pub ok_damage: bool,
    pub ok_safety: bool,
}

const G_MM_S2: f64 = 9806.65;

pub fn to_equivalent_sdof(
    base_shear: &[f64],
    top_disp: &[f64],
    story_masses: &[f64],
) -> CapacityCurve {
    let total_mass: f64 = story_masses.iter().sum();
    if total_mass == 0.0 {
        return CapacityCurve { points: vec![] };
    }

    let mef = 0.75;
    let equiv_mass = total_mass * mef;
    let points: Vec<(f64, f64)> = base_shear
        .iter()
        .zip(top_disp.iter())
        .map(|(&v, &d)| {
            let sa = v / equiv_mass / G_MM_S2;
            let sd = d / mef;
            (sd, sa)
        })
        .collect();

    CapacityCurve { points }
}

pub fn demand_spectrum(rt_val: f64, _tc: f64) -> ResponseSpectrum {
    let periods: Vec<f64> = (1..50).map(|i| i as f64 * 0.05).collect();
    let accelerations: Vec<f64> = periods
        .iter()
        .map(|&t| {
            if t < 0.1 {
                0.1 / t.max(0.02)
            } else {
                rt_val / t
            }
        })
        .collect();
    ResponseSpectrum {
        periods,
        accelerations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_equivalent_sdof() {
        let curve = to_equivalent_sdof(&[1000.0], &[10.0], &[500.0, 500.0]);
        assert_eq!(curve.points.len(), 1);
    }
}
