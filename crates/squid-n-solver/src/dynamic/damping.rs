use faer::sparse::SparseColMat;
use squid_n_math::sparse::weighted_sum_csc;

type SparseMat = SparseColMat<usize, f64>;

/// 減衰モデル（設計書 §10.5 / R9）。
pub enum Damping {
    /// 質量比例減衰 C = a0·M。a0 = 2·h·ω で対象振動数 ω に減衰比 h を与える。
    /// 低次モードを強く減衰させる（高次は残る）。
    MassProportional { h: f64, omega: f64 },
    /// 剛性比例減衰 C = a1·K。a1 = 2·h/ω で対象振動数 ω に減衰比 h を与える。
    /// 日本の建築慣行の既定（1次モード剛性比例）。高次モードを強く減衰させる。
    StiffnessProportional {
        h: f64,
        omega: f64,
        basis: StiffnessKind,
    },
    /// Rayleigh 減衰 C = a0·M + a1·K。2つの振動数で目標減衰比を与える。
    Rayleigh { h1: f64, w1: f64, h2: f64, w2: f64 },
}

/// 剛性比例減衰がどの剛性を基準にするか。
pub enum StiffnessKind {
    Initial,
    Tangent,
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
        }
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
}
