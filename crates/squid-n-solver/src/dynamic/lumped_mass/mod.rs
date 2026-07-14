//! 質点系（串団子）モデルの生成（せん断型多質点系、構造力学）。
//!
//! 立体フレームのプッシュオーバー（漸増静的）結果から、層ごとの層せん断力 Q・層間変形 δ
//! 関係（Q-δ 曲線）を抽出し、**等包絡面積則**でトリリニア骨格へ縮約した串団子モデルを
//! 生成する。
//!
//! - 初期剛性 K1: プッシュオーバー第1ステップの荷重-変形勾配。
//! - 第3折点（終局）: Q-δ 曲線の終端。第3勾配 K3: 終端の接線勾配。
//! - 第1折点: 接線勾配が K1 の指定比率（`secant_ratio`）を初めて下回る直前の変位、
//!   第1勾配は K1（ルール1「割線剛性比率」の変形。接線基準の意図は実装コメント参照）。
//! - 第2折点: 0→第3折点の包絡面積が実曲線と等しくなるよう自動決定。
//!
//! 詳細なルール1/2/3の分岐（降伏部材比率等）は簡略化しており、第1折点の判定は
//! 割線剛性比率（`secant_ratio`）で行う。

mod model;
mod time_history;

pub use model::{
    build_lumped_mass_model, fit_story_trilinear, LumpedMassModel, LumpedMassType, StoryStick,
    StoryTrilinear,
};
pub use time_history::{lumped_mass_time_history, StickResponse};

// tests は両サブモジュールの非公開項目（`pub(crate)`）を `super::*` で参照するため、
// テストビルド時のみ mod.rs 名前空間へ取り込む。
#[cfg(test)]
use model::envelope_area;
#[cfg(test)]
use squid_n_core::ids::StoryId;
#[cfg(test)]
use time_history::{fundamental_omega, solve_tridiagonal};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fit_trilinear_equal_area_and_endpoints() {
        // 実曲線: 折れ点のあるなめらかな軟化曲線を細かくサンプル。
        // 0→(1,100) K1=100、(1,100)→(3,140) K2=20、(3,140)→(6,155) K3=5。
        let mut curve = Vec::new();
        for step in 1..=60 {
            let d = step as f64 * 0.1;
            let q = if d <= 1.0 {
                100.0 * d
            } else if d <= 3.0 {
                100.0 + 20.0 * (d - 1.0)
            } else {
                140.0 + 5.0 * (d - 3.0)
            };
            curve.push((d, q));
        }
        let tri = fit_story_trilinear(&curve, 0.9);
        // K1 = 初期剛性 100。
        assert!((tri.k1 - 100.0).abs() < 1.0, "k1={}", tri.k1);
        // 終端 (6, 155)。
        assert!((tri.d3 - 6.0).abs() < 1e-6 && (tri.q3 - 155.0).abs() < 1e-6);
        // 折点は昇順・耐力単調増加。
        assert!(tri.d1 < tri.d2 && tri.d2 <= tri.d3);
        assert!(tri.q1 <= tri.q2 + 1e-9 && tri.q2 <= tri.q3 + 1e-9);
        // 等包絡面積: トリリニアの面積 = 実曲線の面積。
        let a_actual = envelope_area(&curve);
        let a_tri = 0.5 * tri.d1 * tri.q1
            + 0.5 * (tri.q1 + tri.q2) * (tri.d2 - tri.d1)
            + 0.5 * (tri.q2 + tri.q3) * (tri.d3 - tri.d2);
        assert!(
            (a_tri - a_actual).abs() < 1e-3 * a_actual,
            "equal-area: a_tri={a_tri}, a_actual={a_actual}"
        );
    }

    #[test]
    fn test_fit_trilinear_k2_k3_helpers() {
        // 3勾配（K1=80 > K2=30 > K3=8）の軟化曲線。
        let curve: Vec<(f64, f64)> = (1..=50)
            .map(|s| {
                let d = s as f64 * 0.1;
                let q = if d <= 1.0 {
                    80.0 * d
                } else if d <= 2.5 {
                    80.0 + 30.0 * (d - 1.0)
                } else {
                    125.0 + 8.0 * (d - 2.5)
                };
                (d, q)
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!(
            tri.d1 < tri.d2 && tri.d2 < tri.d3,
            "distinct folds: {tri:?}"
        );
        assert!(
            tri.k1 >= tri.k2() && tri.k2() >= tri.k3() - 1e-6,
            "K1>=K2>=K3: k1={}, k2={}, k3={}",
            tri.k1,
            tri.k2(),
            tri.k3()
        );
        assert!(tri.k3() >= 0.0 && tri.k3() <= tri.k1);
    }

    #[test]
    fn test_fit_trilinear_bilinear_input_reduces_gracefully() {
        // バイリニア入力（K1=50→K=5）はトリリニアが縮退（d1≈d2）しても panic せず妥当。
        let curve: Vec<(f64, f64)> = (1..=30)
            .map(|s| {
                let d = s as f64 * 0.1;
                (d, 50.0 * d.min(2.0) + 5.0 * (d - 2.0).max(0.0))
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!((tri.k1 - 50.0).abs() < 1.0);
        assert!(tri.d1 <= tri.d2 && tri.d2 <= tri.d3);
        assert!((tri.d3 - 3.0).abs() < 1e-6 && (tri.q3 - 105.0).abs() < 1e-6);
    }

    #[test]
    fn test_fit_trilinear_empty_and_degenerate() {
        let tri = fit_story_trilinear(&[], 0.75);
        assert_eq!(tri.k1, 0.0);
        // 1点のみ（弾性）。
        let tri1 = fit_story_trilinear(&[(2.0, 200.0)], 0.75);
        assert!((tri1.k1 - 100.0).abs() < 1e-9);
    }

    fn stick(mass: f64, k1: f64, d1: f64, d2: f64, q2: f64, d3: f64, q3: f64) -> StoryStick {
        StoryStick {
            story: StoryId(0),
            mass,
            height: 3000.0,
            skeleton: StoryTrilinear {
                k1,
                d1,
                q1: k1 * d1,
                d2,
                q2,
                d3,
                q3,
            },
        }
    }

    #[test]
    fn test_solve_tridiagonal_identity() {
        // 単位行列: x=b。
        let x = solve_tridiagonal(
            &[0.0, 0.0, 0.0],
            &[1.0, 1.0, 1.0],
            &[0.0, 0.0, 0.0],
            &[3.0, 5.0, 7.0],
        );
        assert!(
            (x[0] - 3.0).abs() < 1e-12 && (x[1] - 5.0).abs() < 1e-12 && (x[2] - 7.0).abs() < 1e-12
        );
    }

    #[test]
    fn test_fundamental_omega_sdof() {
        // 1 質点: ω1=√(k/m)。
        let w = fundamental_omega(&[2.0], &[800.0]);
        assert!((w - (800.0_f64 / 2.0).sqrt()).abs() < 1e-6, "w={w}");
    }

    #[test]
    fn test_stick_th_zero_input_zero_response() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![stick(1.0, 1000.0, 0.1, 0.3, 140.0, 1.0, 160.0)],
        };
        let res = lumped_mass_time_history(&lm, &vec![0.0; 200], 0.01, 0.02);
        assert!(res.roof_disp.iter().all(|&v| v.abs() < 1e-9));
        assert_eq!(res.story_ductility[0], 0.0);
    }

    #[test]
    fn test_stick_th_responds_and_bounded() {
        // 正弦地動で応答が非ゼロかつ有限。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![
                stick(1.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.0, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let dt = 0.01;
        let accel: Vec<f64> = (0..300)
            .map(|i| 2000.0 * (2.0 * std::f64::consts::PI * 1.5 * i as f64 * dt).sin())
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.03);
        assert_eq!(res.time.len(), 300);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.roof_disp.iter().any(|&v| v.abs() > 1e-3),
            "should show nonzero roof response"
        );
        assert_eq!(res.story_peak_drift.len(), 2);
    }

    #[test]
    fn test_stick_th_yields_under_strong_input() {
        // 強い地動で層が降伏（塑性率 μ>1）。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![stick(2.0, 1000.0, 0.5, 2.0, 700.0, 8.0, 800.0)],
        };
        let dt = 0.01;
        // 一定方向の強い引き込みで大変形。
        let accel: Vec<f64> = (0..400)
            .map(|i| {
                let t = i as f64 * dt;
                3000.0 * (2.0 * std::f64::consts::PI * 0.8 * t).sin()
            })
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.02);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.story_ductility[0] > 1.0,
            "strong input should yield the story: μ={}",
            res.story_ductility[0]
        );
    }
}
