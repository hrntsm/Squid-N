//! 鉄骨梁端部の累積損傷度（レインフロー法（ASTM E1049-85）・Miner 則）。
//!
//! 梁端曲げ塑性率 μ の時刻歴から、以下の方法で累積損傷度 D を算定する純関数群。
//!
//! - **レインフロー法**: μ 振幅をレインフロー計数（ASTM E1049-85 3 点法）し、
//!   各サイクルの片振幅を μ として `Nf = (μ/C)^(−1/β)`（破断寿命）、`Di = Nei/Nfi`、
//!   `D = Σ Di`（Miner 則）。振幅は振れ幅（peak-to-peak）としてカウントされるため、
//!   片振幅としての μ は振れ幅の 1/2 とする。
//! - **累積塑性変形倍率（最大振幅）**: `D = η/(4·(μmax−1))·(μmax/C)^(1/β)`。

/// 鉄骨疲労特性（`Nf = (μ/C)^(−1/β)`）。
///
/// `c`・`beta` は要原典照合の暫定既定値（鋼種・接合形式に依存）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FatigueParams {
    /// 疲労強度係数 C。
    pub c: f64,
    /// 疲労指数 β。
    pub beta: f64,
}

impl Default for FatigueParams {
    fn default() -> Self {
        // 暫定既定（要原典照合）。
        Self { c: 20.0, beta: 0.5 }
    }
}

/// レインフローで抽出した 1 サイクル。`range` は振れ幅（peak-to-peak）、
/// `count` は回数（全サイクル=1.0、半サイクル=0.5）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RainflowCycle {
    pub range: f64,
    pub count: f64,
}

/// 時系列を折返し点（peaks/valleys）の列へ縮約する。
fn turning_points(series: &[f64]) -> Vec<f64> {
    // 連続する等値を除去。
    let mut pts: Vec<f64> = Vec::with_capacity(series.len());
    for &v in series {
        if pts.last().map(|&l| l != v).unwrap_or(true) {
            pts.push(v);
        }
    }
    if pts.len() < 2 {
        return pts;
    }
    let mut tp = Vec::with_capacity(pts.len());
    tp.push(pts[0]);
    for i in 1..pts.len() - 1 {
        let a = pts[i] - pts[i - 1];
        let b = pts[i + 1] - pts[i];
        // 勾配の符号が反転する点のみ折返し点。
        if a * b < 0.0 {
            tp.push(pts[i]);
        }
    }
    tp.push(pts[pts.len() - 1]);
    tp
}

/// レインフロー計数（ASTM E1049-85 3 点法）。振れ幅（range）のサイクル列を返す。
pub fn rainflow_cycles(series: &[f64]) -> Vec<RainflowCycle> {
    let tp = turning_points(series);
    let mut stack: Vec<f64> = Vec::new();
    let mut cycles: Vec<RainflowCycle> = Vec::new();
    for x in tp {
        stack.push(x);
        while stack.len() >= 3 {
            let n = stack.len();
            let y = (stack[n - 2] - stack[n - 3]).abs();
            let xr = (stack[n - 1] - stack[n - 2]).abs();
            if xr < y {
                break;
            }
            if n == 3 {
                // 先頭を含む → 半サイクル。先頭点を除去。
                cycles.push(RainflowCycle {
                    range: y,
                    count: 0.5,
                });
                stack.remove(0);
            } else {
                // 内側の閉サイクル → 全サイクル。中間 2 点（末尾を残す）を除去。
                cycles.push(RainflowCycle {
                    range: y,
                    count: 1.0,
                });
                stack.remove(n - 2);
                stack.remove(n - 3);
            }
        }
    }
    // 残差（スタックに残る連続区間）は半サイクル。
    for i in 0..stack.len().saturating_sub(1) {
        cycles.push(RainflowCycle {
            range: (stack[i + 1] - stack[i]).abs(),
            count: 0.5,
        });
    }
    cycles
}

/// レインフロー法による累積損傷度 `D = Σ Nei/Nfi`。
/// 各サイクルの片振幅 `μ = range/2` に対し `Nf = (μ/C)^(−1/β)`。
pub fn cumulative_damage_rainflow(ductility_series: &[f64], p: FatigueParams) -> f64 {
    if p.c <= 0.0 || p.beta <= 0.0 {
        return 0.0;
    }
    let mut d = 0.0;
    for cyc in rainflow_cycles(ductility_series) {
        let mu = cyc.range * 0.5;
        if mu <= 0.0 {
            continue;
        }
        // 1/Nf = (μ/C)^(1/β)
        d += cyc.count * (mu / p.c).powf(1.0 / p.beta);
    }
    d
}

/// 累積塑性変形倍率（最大振幅）による累積損傷度
/// `D = η/(4·(μmax−1))·(μmax/C)^(1/β)`。`μmax≤1` は 0。
/// （(μmax−1) は分母。分子に乗じていた従来実装は μmax が大きいほど
/// D を過大、1 に近いほど過小に評価する誤りだった。）
pub fn cumulative_damage_max_amplitude(mu_max: f64, eta: f64, p: FatigueParams) -> f64 {
    if p.c <= 0.0 || p.beta <= 0.0 || mu_max <= 1.0 {
        return 0.0;
    }
    eta / (4.0 * (mu_max - 1.0)) * (mu_max / p.c).powf(1.0 / p.beta)
}

/// 累積損傷度の算定方式（レインフロー法（ASTM E1049-85）・Miner 則）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DamageMethod {
    /// レインフロー法。
    Rainflow,
    /// 累積塑性変形倍率（最大振幅）。
    MaxAmplitude,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_turning_points_filters_intermediate() {
        // 単調増加中の中間点は折返しでない。
        let tp = turning_points(&[0.0, 1.0, 2.0, 1.0, 3.0]);
        assert_eq!(tp, vec![0.0, 2.0, 1.0, 3.0]);
    }

    #[test]
    fn test_rainflow_single_half_cycle() {
        let c = rainflow_cycles(&[0.0, 2.0, 0.0]);
        // 1 つの半サイクル（範囲 2）。
        assert_eq!(c.len(), 2); // 0->2 と 2->0 の半サイクル 2 本 = 全 1 回相当
        let total: f64 = c.iter().map(|x| x.count).sum();
        assert!((total - 1.0).abs() < 1e-9);
        assert!(c.iter().all(|x| (x.range - 2.0).abs() < 1e-9));
    }

    #[test]
    fn test_rainflow_nested_cycle() {
        // 内側の小サイクル(範囲2)が全サイクル、外側(範囲4)が半サイクル2回。
        let c = rainflow_cycles(&[0.0, 3.0, 1.0, 4.0, 0.0]);
        let full2 = c
            .iter()
            .filter(|x| (x.range - 2.0).abs() < 1e-9 && (x.count - 1.0).abs() < 1e-9)
            .count();
        assert_eq!(full2, 1, "nested range-2 full cycle expected: {c:?}");
        let half4: f64 = c
            .iter()
            .filter(|x| (x.range - 4.0).abs() < 1e-9)
            .map(|x| x.count)
            .sum();
        assert!((half4 - 1.0).abs() < 1e-9, "range-4 total count: {half4}");
    }

    #[test]
    fn test_cumulative_damage_rainflow_handcalc() {
        // [0,4,0]: 半サイクル range4 ×2（合計1回）、μ=2。C=20,β=0.5。
        // 1/Nf = (2/20)^(1/0.5) = 0.1^2 = 0.01。D = 1.0·0.01 = 0.01。
        let p = FatigueParams { c: 20.0, beta: 0.5 };
        let d = cumulative_damage_rainflow(&[0.0, 4.0, 0.0], p);
        assert!((d - 0.01).abs() < 1e-9, "D={d}");
    }

    #[test]
    fn test_cumulative_damage_max_amplitude_handcalc() {
        // D = η/(4·(μmax−1))·(μmax/C)^(1/β)。μmax=4,η=1,C=20,β=0.5:
        // 1/(4·3)·(0.2)^2 = (1/12)·0.04 = 0.003333…。
        let p = FatigueParams { c: 20.0, beta: 0.5 };
        let d = cumulative_damage_max_amplitude(4.0, 1.0, p);
        assert!((d - 0.04 / 12.0).abs() < 1e-9, "D={d}");
        // μmax<1 は 0。
        assert_eq!(cumulative_damage_max_amplitude(0.5, 1.0, p), 0.0);
    }

    #[test]
    fn test_damage_increases_with_amplitude() {
        let p = FatigueParams::default();
        let small = cumulative_damage_rainflow(&[0.0, 2.0, 0.0, 2.0, 0.0], p);
        let large = cumulative_damage_rainflow(&[0.0, 6.0, 0.0, 6.0, 0.0], p);
        assert!(large > small, "larger amplitude → larger damage");
    }
}
