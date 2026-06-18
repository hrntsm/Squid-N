//! 履歴則のスナップショットテスト（仕様書 §8.1）。
//! 規定の繰り返し変位履歴に対する (θ, M) ループを insta で固定し、回帰を検出する。

use sc_material::{HysteresisMaterial, HysteresisRule, UniaxialMaterial};

/// 規定の繰り返し変位履歴: 段階的に振幅を増やす正負交番。
fn cyclic_history(theta_y: f64) -> Vec<f64> {
    let mut hist = vec![0.0];
    for amp in [1.0, 2.0, 3.0, 4.0] {
        hist.extend([theta_y * amp, 0.0, -theta_y * amp, 0.0]);
    }
    hist
}

fn run_loop(mat: &mut HysteresisMaterial, history: &[f64]) -> String {
    let mut out = String::from("theta,M\n");
    for &theta in history {
        let (m, _) = mat.trial(theta);
        out.push_str(&format!("{:.6},{:.6}\n", theta, m));
        mat.commit();
    }
    out
}

#[test]
fn snapshot_takeda_loop() {
    let rule = HysteresisRule::Takeda {
        crack: (40.0, 0.002),
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
        alpha: 0.4,
    };
    let mut mat = HysteresisMaterial::new(rule);
    let hist = cyclic_history(0.01);
    let csv = run_loop(&mut mat, &hist);
    insta::assert_snapshot!("takeda_loop", csv);
}

#[test]
fn snapshot_origin_oriented_loop() {
    let rule = HysteresisRule::OriginOriented {
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
    };
    let mut mat = HysteresisMaterial::new(rule);
    let hist = cyclic_history(0.01);
    let csv = run_loop(&mut mat, &hist);
    insta::assert_snapshot!("origin_oriented_loop", csv);
}

#[test]
fn snapshot_slip_loop() {
    let rule = HysteresisRule::Slip {
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
        slip_factor: 0.5,
    };
    let mut mat = HysteresisMaterial::new(rule);
    let hist = cyclic_history(0.01);
    let csv = run_loop(&mut mat, &hist);
    insta::assert_snapshot!("slip_loop", csv);
}
