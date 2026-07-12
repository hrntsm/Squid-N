use faer::sparse::SparseColMat;
use squid_n_math::sparse::{assemble_csc, sparse_matvec, weighted_sum_csc, Triplet};

type SparseMat = SparseColMat<usize, f64>;

/// モード別減衰の 1 モード分（RESP-D「07 非線形解析（動的解析）」減衰マトリクス
/// 「モード別減衰」）。`shape` は質量正規化した固有ベクトル（φᵀMφ=1、縮約空間）。
#[derive(Clone, Debug)]
pub struct ModalDampingMode {
    /// 固有円振動数 ω [rad/s]。
    pub omega: f64,
    /// このモードの減衰比 h。
    pub ratio: f64,
    /// 質量正規化固有ベクトル φ（縮約空間、φᵀMφ=1）。
    pub shape: Vec<f64>,
}

/// 減衰モデル（設計書 §10.5 / R9、RESP-D「07」減衰マトリクス）。
pub enum Damping {
    /// 質量比例減衰 C = a0·M。a0 = 2·h·ω で対象振動数 ω に減衰比 h を与える。
    /// 低次モードを強く減衰させる（高次は残る）。
    MassProportional { h: f64, omega: f64 },
    /// 剛性比例減衰 C = a1·K。a1 = 2·h/ω で対象振動数 ω に減衰比 h を与える。
    /// 日本の建築慣行の既定（1次モード剛性比例）。高次モードを強く減衰させる。
    /// `basis=Tangent` は瞬間（接線）剛性比例（α1 一定）で、非線形解析では
    /// 毎ステップ接線剛性から C を再構成する。
    StiffnessProportional {
        h: f64,
        omega: f64,
        basis: StiffnessKind,
    },
    /// Rayleigh 減衰 C = a0·M + a1·K。2つの振動数で目標減衰比を与える。
    Rayleigh { h1: f64, w1: f64, h2: f64, w2: f64 },
    /// モード別減衰。各モードに独立の減衰比 h_i を与える。
    /// C = Σ_i 2·h_i·ω_i·(M φ_i)(M φ_i)ᵀ（質量正規化モード）。
    Modal { modes: Vec<ModalDampingMode> },
    /// 瞬間剛性比例・h1 一定（RESP-D「07」減衰マトリクス「h1 一定」）。
    /// C = 2·h1/ω1·[S]、ω1 = ω1e·√(uᵀ[S]u / uᵀ[Se]u)（[S]=瞬間剛性, [Se]=初期剛性）。
    /// 非線形解析で減衰比 h1 を一定に保つよう ω1 を毎ステップ更新する。
    TangentStiffnessConstantH { h1: f64, omega1e: f64 },
}

/// 剛性比例減衰がどの剛性を基準にするか。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StiffnessKind {
    /// 初期剛性比例（[C]=α1[Se]、α1=2h/ω）。
    Initial,
    /// 瞬間（接線）剛性比例（α1 一定。[C]=α1[S]）。
    Tangent,
}

/// 減衰力の評価方式（RESP-D「07 非線形解析（動的解析）」減衰マトリクス
/// 「累積型・非累積型」）。減衰行列 C が時々刻々変化する（接線比例・モード別で
/// 剛性が変化する）場合に両者は異なる。C 一定なら両者は一致する。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DampingAccumulation {
    /// 非累積型: 減衰力 = 瞬間減衰マトリクス × 速度（`{Cn}=[Cn]{ẋn}`）。
    #[default]
    NonCumulative,
    /// 累積型: 増分減衰力を積分（`{Cn}={Cn−1}+[Cn]{Δẋn}`）。C が変化する場合に
    /// 履歴依存となり、瞬間 C×速度とは異なる。
    Cumulative,
}

impl Damping {
    /// 減衰行列 C を組み立てる。`m`, `k` は縮約済みまたは自由度版の質量・剛性行列。
    pub fn assemble_c(&self, m: &SparseMat, k: &SparseMat) -> SparseMat {
        let n = m.ncols();
        match self {
            Damping::MassProportional { h, omega } => {
                let a0 = 2.0 * h * omega;
                weighted_sum_csc(n, &[(a0, m)])
            }
            Damping::StiffnessProportional { h, omega, .. } => {
                let a1 = 2.0 * h / omega;
                weighted_sum_csc(n, &[(a1, k)])
            }
            Damping::Rayleigh { h1, w1, h2, w2 } => {
                let (a0, a1) = Self::rayleigh_coeffs(*w1, *w2, *h1, *h2);
                weighted_sum_csc(n, &[(a0, m), (a1, k)])
            }
            Damping::Modal { modes } => modal_c(m, modes),
            Damping::TangentStiffnessConstantH { h1, omega1e } => {
                // 初期近似（ω1=ω1e）: C = 2h1/ω1e·K。非線形解析はループ側で ω1 を更新。
                let a1 = if *omega1e > 0.0 {
                    2.0 * h1 / omega1e
                } else {
                    0.0
                };
                weighted_sum_csc(n, &[(a1, k)])
            }
        }
    }

    /// 接線（瞬間）剛性に依存する減衰か（非線形解析で毎ステップ C を再構成する要否）。
    pub fn is_tangent_based(&self) -> bool {
        matches!(
            self,
            Damping::StiffnessProportional {
                basis: StiffnessKind::Tangent,
                ..
            } | Damping::TangentStiffnessConstantH { .. }
        )
    }

    /// 瞬間剛性 `k_t`・初期剛性 `k_e`・現在変位 `u` から接線減衰行列 C を再構成する
    /// （RESP-D「07」剛性変更に伴う減衰項の変更）。接線比例でない場合は初期 C を返す。
    pub fn assemble_c_tangent(
        &self,
        m: &SparseMat,
        k_t: &SparseMat,
        k_e: &SparseMat,
        u: &[f64],
    ) -> SparseMat {
        let n = m.ncols();
        match self {
            // α1 一定: C = (2h/ω)·K_t（α1=2h/ω は初期振動数から定める一定値）。
            Damping::StiffnessProportional {
                h,
                omega,
                basis: StiffnessKind::Tangent,
            } => {
                let a1 = if *omega > 0.0 { 2.0 * h / omega } else { 0.0 };
                weighted_sum_csc(n, &[(a1, k_t)])
            }
            // h1 一定: ω1 = ω1e·√(uᵀK_t u / uᵀK_e u)、C = (2h1/ω1)·K_t。
            Damping::TangentStiffnessConstantH { h1, omega1e } => {
                let num = quad_form(k_t, u);
                let den = quad_form(k_e, u);
                let omega1 = if den > 0.0 && num > 0.0 {
                    omega1e * (num / den).sqrt()
                } else {
                    *omega1e
                };
                let a1 = if omega1 > 0.0 { 2.0 * h1 / omega1 } else { 0.0 };
                weighted_sum_csc(n, &[(a1, k_t)])
            }
            // 接線比例でない場合は初期 C を返す（毎ステップ不変）。
            _ => self.assemble_c(m, k_e),
        }
    }

    /// 質量正規化モード（φᵀMφ=1）とモード別減衰比からモード別減衰を構成する。
    pub fn modal(shapes: &[Vec<f64>], omegas: &[f64], ratios: &[f64]) -> Self {
        let modes = shapes
            .iter()
            .zip(omegas.iter())
            .zip(ratios.iter())
            .map(|((shape, &omega), &ratio)| ModalDampingMode {
                omega,
                ratio,
                shape: shape.clone(),
            })
            .collect();
        Damping::Modal { modes }
    }

    /// Rayleigh 減衰の係数 (α_m, β_k) を、2つの振動数と目標減衰比から計算する。
    /// モード i の減衰比: h_i = α_m/(2ω_i) + β_k·ω_i/2。
    /// ω1 で h1、ω2 で h2 を満たす (α_m, β_k) を連立方程式から解く。
    pub fn rayleigh_coeffs(omega1: f64, omega2: f64, h1: f64, h2: f64) -> (f64, f64) {
        let d = omega2 * omega2 - omega1 * omega1;
        let beta_k = 2.0 * (h2 * omega2 - h1 * omega1) / d;
        let alpha_m = 2.0 * omega1 * omega2 * (h1 * omega2 - h2 * omega1) / d;
        (alpha_m, beta_k)
    }
}

/// モード別減衰行列 C = Σ_i 2·h_i·ω_i·(Mφ_i)(Mφ_i)ᵀ（質量正規化モード φᵀMφ=1）。
/// このとき φ_jᵀ C φ_j = 2·h_j·ω_j（他モードとは直交）となり、各モードに独立の
/// 減衰比 h_i を与える。縮約空間で密行列となるため、質点系・縮約モデル向け。
fn modal_c(m: &SparseMat, modes: &[ModalDampingMode]) -> SparseMat {
    let n = m.ncols();
    let mut dense = vec![0.0f64; n * n];
    for mode in modes {
        if mode.shape.len() != n {
            continue;
        }
        let mphi = sparse_matvec(m, &mode.shape); // M·φ
        let coef = 2.0 * mode.ratio * mode.omega;
        if coef == 0.0 {
            continue;
        }
        for a in 0..n {
            let va = coef * mphi[a];
            if va == 0.0 {
                continue;
            }
            for b in 0..n {
                dense[a * n + b] += va * mphi[b];
            }
        }
    }
    let mut trips = Vec::new();
    for a in 0..n {
        for b in 0..n {
            let v = dense[a * n + b];
            if v.abs() > 1e-30 {
                trips.push(Triplet {
                    row: a,
                    col: b,
                    val: v,
                });
            }
        }
    }
    assemble_csc(n, trips)
}

/// 二次形式 uᵀ·A·u。
fn quad_form(a: &SparseMat, u: &[f64]) -> f64 {
    let au = sparse_matvec(a, u);
    u.iter().zip(au.iter()).map(|(&ui, &aui)| ui * aui).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_math::sparse::{assemble_csc, Triplet};

    fn diag_csc(n: usize, vals: &[f64]) -> SparseMat {
        let triplets: Vec<Triplet> = vals
            .iter()
            .enumerate()
            .map(|(i, &v)| Triplet {
                row: i,
                col: i,
                val: v,
            })
            .collect();
        assemble_csc(n, triplets)
    }

    #[test]
    fn test_damping_rayleigh_coeffs() {
        let (alpha_m, beta_k) = Damping::rayleigh_coeffs(10.0, 100.0, 0.05, 0.05);
        let omega1 = 10.0;
        let h_actual = (alpha_m / omega1 + beta_k * omega1) / 2.0;
        assert!((h_actual - 0.05).abs() < 1e-6);
        let omega2 = 100.0;
        let h2_actual = (alpha_m / omega2 + beta_k * omega2) / 2.0;
        assert!((h2_actual - 0.05).abs() < 1e-6);
    }

    #[test]
    fn test_mass_proportional_assemble_c() {
        // C = a0·M, a0 = 2·h·ω = 2·0.05·10 = 1.0
        let m = diag_csc(2, &[10.0, 20.0]);
        let damping = Damping::MassProportional {
            h: 0.05,
            omega: 10.0,
        };
        let c = damping.assemble_c(&m, &m);
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 10.0).abs() < 1e-12);
        assert!((*c.get(1, 1).unwrap_or(&0.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn test_stiffness_proportional_assemble_c() {
        // C = a1·K, a1 = 2·h/ω = 2·0.05/10 = 0.01
        let m = diag_csc(2, &[1.0, 1.0]);
        let k = diag_csc(2, &[1000.0, 2000.0]);
        let damping = Damping::StiffnessProportional {
            h: 0.05,
            omega: 10.0,
            basis: StiffnessKind::Initial,
        };
        let c = damping.assemble_c(&m, &k);
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 10.0).abs() < 1e-12);
        assert!((*c.get(1, 1).unwrap_or(&0.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn test_rayleigh_assemble_c() {
        // C = a0·M + a1·K
        let m = diag_csc(2, &[1.0, 1.0]);
        let k = diag_csc(2, &[100.0, 400.0]);
        let damping = Damping::Rayleigh {
            h1: 0.05,
            w1: 10.0,
            h2: 0.05,
            w2: 20.0,
        };
        let (a0, a1) = Damping::rayleigh_coeffs(10.0, 20.0, 0.05, 0.05);
        let c = damping.assemble_c(&m, &k);
        assert!(
            (*c.get(0, 0).unwrap_or(&0.0) - (a0 * 1.0 + a1 * 100.0)).abs() < 1e-9,
            "c00={}",
            *c.get(0, 0).unwrap_or(&0.0)
        );
        assert!(
            (*c.get(1, 1).unwrap_or(&0.0) - (a0 * 1.0 + a1 * 400.0)).abs() < 1e-9,
            "c11={}",
            *c.get(1, 1).unwrap_or(&0.0)
        );
    }

    fn quad(c: &SparseMat, x: &[f64], y: &[f64]) -> f64 {
        let mut s = 0.0;
        for i in 0..x.len() {
            for j in 0..y.len() {
                s += x[i] * *c.get(i, j).unwrap_or(&0.0) * y[j];
            }
        }
        s
    }

    #[test]
    fn test_modal_damping_reproduces_ratios() {
        // 質量正規化モード φᵀMφ=1 → φ_iᵀCφ_i = 2h_iω_i、φ_iᵀCφ_j = 0（直交）。
        let m = diag_csc(2, &[2.0, 3.0]);
        let phi1 = vec![1.0 / 2.0f64.sqrt(), 0.0];
        let phi2 = vec![0.0, 1.0 / 3.0f64.sqrt()];
        let d = Damping::Modal {
            modes: vec![
                ModalDampingMode {
                    omega: 10.0,
                    ratio: 0.05,
                    shape: phi1.clone(),
                },
                ModalDampingMode {
                    omega: 20.0,
                    ratio: 0.03,
                    shape: phi2.clone(),
                },
            ],
        };
        let c = d.assemble_c(&m, &m);
        assert!((quad(&c, &phi1, &phi1) - 2.0 * 0.05 * 10.0).abs() < 1e-9);
        assert!((quad(&c, &phi2, &phi2) - 2.0 * 0.03 * 20.0).abs() < 1e-9);
        assert!(
            quad(&c, &phi1, &phi2).abs() < 1e-9,
            "modes must be orthogonal"
        );
    }

    #[test]
    fn test_modal_constructor_from_eigen_shapes() {
        let m = diag_csc(1, &[1.0]);
        let d = Damping::modal(&[vec![1.0]], &[5.0], &[0.02]);
        let c = d.assemble_c(&m, &m);
        // φᵀCφ = 2·0.02·5 = 0.2、φ=1。
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 0.2).abs() < 1e-9);
    }

    #[test]
    fn test_tangent_stiffness_constant_alpha1() {
        // α1 一定（StiffnessProportional Tangent）: C = 2h/ω·K_t。
        let m = diag_csc(2, &[1.0, 1.0]);
        let ke = diag_csc(2, &[1000.0, 2000.0]);
        let kt = diag_csc(2, &[500.0, 1000.0]);
        let d = Damping::StiffnessProportional {
            h: 0.05,
            omega: 10.0,
            basis: StiffnessKind::Tangent,
        };
        assert!(d.is_tangent_based());
        let c = d.assemble_c_tangent(&m, &kt, &ke, &[1.0, 0.0]);
        let a1 = 2.0 * 0.05 / 10.0; // 0.01
        assert!((*c.get(0, 0).unwrap_or(&0.0) - a1 * 500.0).abs() < 1e-9);
        assert!((*c.get(1, 1).unwrap_or(&0.0) - a1 * 1000.0).abs() < 1e-9);
    }

    #[test]
    fn test_tangent_stiffness_constant_h1() {
        // h1 一定: ω1 = ω1e·√(uᵀKt u / uᵀKe u)、C = 2h1/ω1·Kt。
        let m = diag_csc(2, &[1.0, 1.0]);
        let ke = diag_csc(2, &[1000.0, 2000.0]);
        let kt = diag_csc(2, &[250.0, 500.0]); // 剛性 1/4 → ω1 半分
        let d = Damping::TangentStiffnessConstantH {
            h1: 0.05,
            omega1e: 10.0,
        };
        assert!(d.is_tangent_based());
        let u = vec![1.0, 0.0];
        // uᵀKt u=250, uᵀKe u=1000 → ω1=10·√0.25=5。C=2·0.05/5·Kt=0.02·Kt。
        let c = d.assemble_c_tangent(&m, &kt, &ke, &u);
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 0.02 * 250.0).abs() < 1e-9);
    }

    #[test]
    fn test_non_tangent_is_not_tangent_based() {
        assert!(!Damping::Rayleigh {
            h1: 0.05,
            w1: 1.0,
            h2: 0.05,
            w2: 2.0
        }
        .is_tangent_based());
        assert!(!Damping::StiffnessProportional {
            h: 0.05,
            omega: 10.0,
            basis: StiffnessKind::Initial
        }
        .is_tangent_based());
    }
}
