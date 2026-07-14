//! 主軸の計算（構造力学）。
//!
//! 指定された荷重条件で X・Y 各方向の弾性応力解析を行い、水平力が X 軸と
//! 角度 Θ をなして作用するときに水平力がなす仕事
//!
//! ```text
//! W = ½ Pᵗ (ux·cos²Θ + (uy + vx)·sinΘ·cosΘ + vy·sin²Θ)
//! ```
//!
//! が極値をとる角度 Θ（建物の主軸方向）を求める:
//!
//! ```text
//! dW/dΘ = ½ Pᵗ ((vy − ux)·sin2Θ + (uy + vx)·cos2Θ) = 0
//! → tan2Θ = −Pᵗ(uy + vx) / Pᵗ(vy − ux)
//! ```
//!
//! - `P` : 各節点へ作用する水平力のベクトル
//! - `ux(uy)` : X(Y)方向加力時の X 方向節点移動量
//! - `vx(vy)` : X(Y)方向加力時の Y 方向節点移動量

use squid_n_core::model::Model;
use squid_n_solver::linear::StaticOnce;

/// 主軸角 Θ [rad]。`tan2Θ = −Pᵗ(uy+vx) / Pᵗ(vy−ux)` を `atan2` で解く。
///
/// 各ベクトルは同一節点順で対応していること。全体が等方（分母分子とも 0）の
/// 場合は 0 を返す（任意の方向が主軸）。返り値は (−π/4, π/4] を中心とする
/// `0.5·atan2` の範囲 (−π/2, π/2]。
pub fn principal_axis_angle(p: &[f64], ux: &[f64], vx: &[f64], uy: &[f64], vy: &[f64]) -> f64 {
    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };
    let num = -(dot(p, uy) + dot(p, vx));
    let den = dot(p, vy) - dot(p, ux);
    if num == 0.0 && den == 0.0 {
        return 0.0;
    }
    0.5 * num.atan2(den)
}

/// X・Y 加力の弾性解析結果から主軸角 Θ [rad] を求める。
///
/// `p` は各節点の水平力の大きさ（`model.nodes` と同順。X 加力・Y 加力とも
/// 同じ分布で作用させた前提。載荷しない節点は 0）。
pub fn principal_axis_from_results(
    model: &Model,
    p: &[f64],
    res_x: &StaticOnce,
    res_y: &StaticOnce,
) -> f64 {
    let n = model.nodes.len();
    let pick = |res: &StaticOnce, dof: usize| -> Vec<f64> {
        (0..n)
            .map(|i| res.disp.get(i).map(|u| u[dof]).unwrap_or(0.0))
            .collect()
    };
    let ux = pick(res_x, 0);
    let vx = pick(res_x, 1);
    let uy = pick(res_y, 0);
    let vy = pick(res_y, 1);
    principal_axis_angle(p, &ux, &vx, &uy, &vy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_symmetric_structure_theta_zero() {
        // 対称構造: 交差変位が無い（uy=vx=0）→ Θ = 0
        let p = vec![1.0, 2.0];
        let ux = vec![1.0, 1.0];
        let vy = vec![2.0, 2.0];
        let zero = vec![0.0, 0.0];
        let th = principal_axis_angle(&p, &ux, &zero, &zero, &vy);
        assert!(th.abs() < 1e-12, "Θ={th}");
    }

    #[test]
    fn test_known_angle() {
        // Pᵗ(uy+vx) = 1, Pᵗ(vy−ux) = 1 → tan2Θ = −1 → Θ = −π/8
        let p = vec![1.0];
        let ux = vec![1.0];
        let vy = vec![2.0];
        let uy = vec![0.5];
        let vx = vec![0.5];
        let th = principal_axis_angle(&p, &ux, &vx, &uy, &vy);
        assert!((th - (-std::f64::consts::PI / 8.0)).abs() < 1e-12, "Θ={th}");
    }

    #[test]
    fn test_rotated_frame_recovers_angle() {
        // 主軸剛性 kx≠ky の系を角度 φ だけ回した剛性行列に単位荷重を与えた
        // ときの変位から Θ = φ が復元されることを確認する。
        // K(φ) = R·diag(kx,ky)·Rᵗ, u = K⁻¹·e1（X加力）, v = K⁻¹·e2（Y加力）
        let (kx, ky) = (2.0, 1.0);
        let phi = 0.3_f64;
        let (c, s) = (phi.cos(), phi.sin());
        // K⁻¹ = R·diag(1/kx,1/ky)·Rᵗ
        let f11 = c * c / kx + s * s / ky;
        let f12 = c * s / kx - c * s / ky;
        let f22 = s * s / kx + c * c / ky;
        // X 加力 (P=[1,0]): ux=f11, vx=f12 / Y 加力 (P=[0,1]): uy=f12, vy=f22
        let p = vec![1.0];
        let th = principal_axis_angle(&p, &[f11], &[f12], &[f12], &[f22]);
        // tan2Θ = −2·f12/(f22−f11)。f12, f22−f11 の符号から Θ = −φ 側の解
        // （柔性行列の主軸＝剛性行列の主軸なので |Θ| = φ を確認）。
        assert!((th.abs() - phi).abs() < 1e-12, "Θ={th}, expected ±{phi}");
    }

    #[test]
    fn test_degenerate_isotropic() {
        let p = vec![1.0];
        let th = principal_axis_angle(&p, &[1.0], &[0.0], &[0.0], &[1.0]);
        assert_eq!(th, 0.0);
    }
}
