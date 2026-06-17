pub enum SoilClass {
    I,
    II,
    III,
}

pub struct AiDistribution {
    pub alpha: Vec<f64>,
    pub ai: Vec<f64>,
    pub ci: Vec<f64>,
    pub qi: Vec<f64>,
    pub pi: Vec<f64>,
    pub t_used: f64,
}

pub fn rt(t: f64, tc: f64) -> f64 {
    if t < tc {
        1.0
    } else if t < 2.0 * tc {
        1.0 - 0.2 * (t / tc - 1.0).powi(2)
    } else {
        1.6 * tc / t
    }
}

pub fn tc_of(soil: SoilClass) -> f64 {
    match soil {
        SoilClass::I => 0.4,
        SoilClass::II => 0.6,
        SoilClass::III => 0.8,
    }
}

pub fn ai_distribution(
    stories_weight_bottom_to_top: &[f64],
    z: f64,
    rt_val: f64,
    c0: f64,
    t: f64,
) -> AiDistribution {
    let total_w: f64 = stories_weight_bottom_to_top.iter().sum();
    let n = stories_weight_bottom_to_top.len();
    let mut alpha = Vec::with_capacity(n);
    let mut cumulative = 0.0;
    for i in (0..n).rev() {
        cumulative += stories_weight_bottom_to_top[i];
        alpha.push(cumulative / total_w);
    }
    alpha.reverse();

    let t_factor = 2.0 * t / (1.0 + 3.0 * t);
    let ai: Vec<f64> = alpha
        .iter()
        .map(|a| 1.0 + ((1.0 / a.sqrt()) - a) * t_factor)
        .collect();
    let ci: Vec<f64> = ai.iter().map(|a| z * rt_val * a * c0).collect();
    let qi: Vec<f64> = ci
        .iter()
        .zip(stories_weight_bottom_to_top.iter())
        .map(|(c, w)| c * w)
        .collect();
    let mut pi = Vec::with_capacity(n);
    for i in 0..n {
        let p = if i < n - 1 { qi[i] - qi[i + 1] } else { qi[i] };
        pi.push(p.max(0.0));
    }

    AiDistribution {
        alpha,
        ai,
        ci,
        qi,
        pi,
        t_used: t,
    }
}

pub fn approx_t(height_m: f64, steel_ratio: f64) -> f64 {
    height_m * (0.02 + 0.01 * steel_ratio)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rt_values() {
        assert!((rt(0.2, 0.6) - 1.0).abs() < 1e-12);
        let r = rt(0.8, 0.6);
        assert!(r < 1.0 && r > 0.0);
        let r = rt(2.0, 0.6);
        assert!((r - 1.6 * 0.6 / 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_ai_distribution_3story() {
        let weights = vec![1000.0, 1000.0, 1000.0];
        let result = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        assert_eq!(result.alpha.len(), 3);
        assert!((result.alpha[0] - 1.0).abs() < 1e-3);
        assert!((result.alpha[1] - 2.0 / 3.0).abs() < 1e-3);
        assert!((result.alpha[2] - 1.0 / 3.0).abs() < 1e-3);
        assert!((result.ai[2] - result.ai[1]) > 0.0);
        assert!(result.pi.iter().all(|&p| p >= 0.0));
    }

    #[test]
    fn test_ai_spec_values() {
        let weights = vec![1.0, 1.0, 1.0];
        let result = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        assert!(
            (result.alpha[0] - 1.0).abs() < 1e-3,
            "alpha[0]={}",
            result.alpha[0]
        );
        assert!(
            (result.alpha[1] - 0.6667).abs() < 1e-3,
            "alpha[1]={}",
            result.alpha[1]
        );
        assert!(
            (result.alpha[2] - 0.3333).abs() < 1e-3,
            "alpha[2]={}",
            result.alpha[2]
        );
        assert!((result.ai[0] - 1.000).abs() < 1e-2, "A0={}", result.ai[0]);
        assert!((result.ai[1] - 1.156).abs() < 1e-2, "A1={}", result.ai[1]);
        assert!((result.ai[2] - 1.390).abs() < 1e-2, "A2={}", result.ai[2]);
    }
}
