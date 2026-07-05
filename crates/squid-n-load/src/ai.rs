#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    // 層せん断力 Qi = Ci · Wi（Wi＝当該層以上の累積重量＝令88条）。
    // αi = Wi/total_w なので Wi = αi·total_w。
    // （旧実装は Ci·単層重量 で、Pi=Qi[i]-Qi[i+1] が下層で負になり max(0) で 0 に
    //   潰れ、地震力が最上層にしか載らない重大なバグだった。）
    let qi: Vec<f64> = ci
        .iter()
        .zip(alpha.iter())
        .map(|(c, a)| c * a * total_w)
        .collect();
    // 各層の水平外力 Pi = Qi − Qi+1（最上層は Pi = Qi）。
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

    #[test]
    fn test_story_shear_uses_cumulative_weight() {
        // Qi = Ci·Wi（Wi=累積重量）。等重量3層 w=[1,1,1], Z=Rt=1, C0=0.2, T=0.24。
        // alpha=[1, 2/3, 1/3], Ai≈[1.0,1.156,1.390], Ci=0.2·Ai。
        // Wi=[3,2,1] → Qi=[0.6, 0.4624, 0.278]（近似）。
        let weights = vec![1.0, 1.0, 1.0];
        let r = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        // 基部せん断 Q0 = C0·Ai0·total = 0.2·1.0·3 = 0.6。
        assert!((r.qi[0] - 0.6).abs() < 1e-2, "Q0={}", r.qi[0]);
        assert!((r.qi[1] - 0.4624).abs() < 2e-2, "Q1={}", r.qi[1]);
        assert!((r.qi[2] - 0.278).abs() < 2e-2, "Q2={}", r.qi[2]);
        // Qi は下から上へ単調減少（正常な地震時層せん断）。
        assert!(r.qi[0] > r.qi[1] && r.qi[1] > r.qi[2]);
        // Pi = Qi−Qi+1 はすべて正（最上層に全部寄る旧バグの否定）。
        assert!(r.pi.iter().all(|&p| p > 0.0), "pi={:?}", r.pi);
        // ΣPi = Q0（基部せん断）。
        let sum_pi: f64 = r.pi.iter().sum();
        assert!(
            (sum_pi - r.qi[0]).abs() < 1e-9,
            "ΣPi={} Q0={}",
            sum_pi,
            r.qi[0]
        );
    }
}
