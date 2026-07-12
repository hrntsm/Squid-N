//! 位相差入力解析（RESP-D「07 非線形解析（動的解析）」その他の解析機能「位相差入力解析」）。
//!
//! 地震動が有限の見かけ速度で矩形基礎を通過することで、基礎の両端に到達時間差（位相遅れ）が
//! 生じ、剛基礎にねじれ加振が加わる。本モジュールは位相遅れ時間と、それに基づく
//! ねじれ地動加速度時刻歴を生成する純関数を提供する。生成した時刻歴は並進加速度と
//! 同時に加振する（[`crate::timehistory::GroundMotion::accel_theta`]）。

/// 位相遅れ時間 `t = (L·sinθ)/Vs`（RESP-D 原典式）。
///
/// - `l_m`: 矩形基礎の長さ L [m]（位相遅れ方向 X なら Lx、Y なら Ly）。
/// - `theta_deg`: 入射角 θ [°]。
/// - `vs_m_s`: せん断波速度 Vs [m/s]。
///
/// 返り値は位相遅れ時間 [s]（`Vs<=0` は 0）。
pub fn phase_lag_time(l_m: f64, theta_deg: f64, vs_m_s: f64) -> f64 {
    if vs_m_s <= 0.0 {
        return 0.0;
    }
    let theta = theta_deg.to_radians();
    (l_m * theta.sin() / vs_m_s).abs()
}

/// 並進加速度時刻歴 `base` と位相遅れ時間から、剛基礎のねじれ地動加速度時刻歴
/// `θ̈_g(t)` [rad/s²] を生成する。
///
/// 位相遅れ方向に距離 `l_mm` [mm] だけ離れた基礎両端が、同一波形を位相遅れ時間 `lag_s`
/// だけずれて受けると考え、剛基礎の回転加速度を両端の並進加速度差 ÷ 距離で近似する:
///
/// `θ̈(k) = (base[k] − base[k − shift]) / l_mm`,  `shift = round(lag_s/dt)`。
///
/// `base` は並進加速度 [mm/s²]、`l_mm` は基礎長さ [mm]（並進加速度と同じ長さ単位）。
/// `shift=0`（位相遅れなし）や `l_mm<=0` の場合はゼロ列（ねじれ加振なし）を返す。
pub fn torsional_accel_series(base: &[f64], dt: f64, lag_s: f64, l_mm: f64) -> Vec<f64> {
    let n = base.len();
    if n == 0 || l_mm <= 0.0 || dt <= 0.0 {
        return vec![0.0; n];
    }
    let shift = (lag_s / dt).round() as usize;
    if shift == 0 {
        return vec![0.0; n];
    }
    (0..n)
        .map(|k| {
            let delayed = if k >= shift { base[k - shift] } else { 0.0 };
            (base[k] - delayed) / l_mm
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_phase_lag_time_formula() {
        // t = L·sinθ/Vs。L=20m, θ=30°, Vs=200m/s → 20·0.5/200 = 0.05s。
        let t = phase_lag_time(20.0, 30.0, 200.0);
        assert!((t - 0.05).abs() < 1e-9, "t={t}");
        // θ=0 → 位相遅れ 0。
        assert_eq!(phase_lag_time(20.0, 0.0, 200.0), 0.0);
        // Vs<=0 → 0。
        assert_eq!(phase_lag_time(20.0, 30.0, 0.0), 0.0);
    }

    #[test]
    fn test_torsional_series_zero_without_lag() {
        // 位相遅れ 0（shift=0）ならねじれ加振なし。
        let base = vec![1.0, 2.0, 3.0, 4.0];
        let s = torsional_accel_series(&base, 0.01, 0.0, 1000.0);
        assert!(s.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_torsional_series_difference_over_length() {
        // shift=1, L=10mm。θ̈(k) = (base[k]−base[k−1])/10。
        let base = vec![0.0, 10.0, 30.0, 30.0];
        let dt = 0.01;
        let lag = dt; // shift=1
        let s = torsional_accel_series(&base, dt, lag, 10.0);
        assert_eq!(s.len(), 4);
        assert!((s[0] - 0.0).abs() < 1e-12); // k<shift → delayed=0 → 0/10
        assert!((s[1] - 1.0).abs() < 1e-12); // (10−0)/10
        assert!((s[2] - 2.0).abs() < 1e-12); // (30−10)/10
        assert!((s[3] - 0.0).abs() < 1e-12); // (30−30)/10
    }

    #[test]
    fn test_torsional_series_nonzero_with_lag() {
        // 位相遅れがあり波形が変化するとねじれ加振が生じる。
        let base: Vec<f64> = (0..50).map(|k| (k as f64 * 0.3).sin()).collect();
        let s = torsional_accel_series(&base, 0.01, 0.03, 5000.0);
        assert!(s.iter().any(|&v| v.abs() > 0.0), "torsion must be nonzero");
    }
}
