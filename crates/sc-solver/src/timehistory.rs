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

    /// 時刻歴ソルバの決定性は P6 実装後に追加予定（R28）。
    /// 現状は Newmark/HHT 設定の決定性のみ確認。
    #[test]
    fn test_timehistory_config_deterministic() {
        let cfg1 = NewmarkCfg::average_accel(0.01);
        let cfg2 = NewmarkCfg::linear_accel(0.02);
        let cfg3 = HhtCfg::new(0.005);
        for _ in 0..10 {
            let c1 = NewmarkCfg::average_accel(0.01);
            assert_eq!(cfg1.beta.to_bits(), c1.beta.to_bits());
            assert_eq!(cfg1.gamma.to_bits(), c1.gamma.to_bits());
            assert_eq!(cfg1.dt.to_bits(), c1.dt.to_bits());
            let c2 = NewmarkCfg::linear_accel(0.02);
            assert_eq!(cfg2.beta.to_bits(), c2.beta.to_bits());
            let c3 = HhtCfg::new(0.005);
            assert_eq!(cfg3.alpha.to_bits(), c3.alpha.to_bits());
            assert_eq!(cfg3.dt.to_bits(), c3.dt.to_bits());
        }
    }
}
