pub struct NewmarkCfg {
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
}

impl NewmarkCfg {
    pub fn average_accel(dt: f64) -> Self {
        Self {
            beta: 0.25,
            gamma: 0.5,
            dt,
        }
    }
    pub fn linear_accel(dt: f64) -> Self {
        Self {
            beta: 1.0 / 6.0,
            gamma: 0.5,
            dt,
        }
    }
}

pub struct HhtCfg {
    pub alpha: f64,
    pub dt: f64,
}

impl HhtCfg {
    pub fn new(dt: f64) -> Self {
        Self { alpha: -0.1, dt }
    }
}

pub struct GroundMotion {
    pub dt: f64,
    pub accel_x: Vec<f64>,
    pub accel_y: Option<Vec<f64>>,
}

pub struct RayleighDamping {
    pub alpha_m: f64,
    pub beta_k: f64,
}

impl RayleighDamping {
    pub fn from_ratios(omega1: f64, omega2: f64, h1: f64, h2: f64) -> Self {
        let d = omega2 * omega2 - omega1 * omega1;
        let beta_k = 2.0 * (h2 * omega2 - h1 * omega1) / d;
        let alpha_m = 2.0 * omega1 * omega2 * (h1 * omega2 - h2 * omega1) / d;
        Self { alpha_m, beta_k }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_rayleigh() {
        let d = RayleighDamping::from_ratios(10.0, 100.0, 0.05, 0.05);
        let omega1 = 10.0;
        let h_actual = (d.alpha_m / omega1 + d.beta_k * omega1) / 2.0;
        assert!((h_actual - 0.05).abs() < 1e-6);
    }
}
