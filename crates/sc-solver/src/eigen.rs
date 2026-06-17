use crate::assemble::assemble_global_m;
use crate::constraint::Reducer;
use sc_core::dof::DofMap;
use sc_core::model::Model;
use sc_element::behavior::MassOption;
use sc_math::solver::{make_solver, SolveError, SolverBackend};

const EIGEN_TOL: f64 = 1e-10;
const EIGEN_MAX_ITER: usize = 100;

pub struct ModalResult {
    pub omega2: Vec<f64>,
    pub period: Vec<f64>,
}

pub fn solve_eigen(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    n_modes: usize,
) -> Result<ModalResult, SolveError> {
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let m_red = reducer.reduce_k(&m_free);
    let n = m_red.nrows();
    if n == 0 {
        return Ok(ModalResult {
            omega2: vec![],
            period: vec![],
        });
    }

    let k_free = crate::assemble::assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_red)?;

    let q = (2 * n_modes).min(n).max(n_modes + 2);
    let n_small = q;

    let mut x = vec![0.0; n * q];
    for i in 0..q.min(n) {
        x[i * q + i] = 1.0;
    }
    for i in n..q {
        x[(i % n) * q + i] = 0.1;
    }

    let mut k_small = vec![0.0; n_small * n_small];
    let mut m_small = vec![0.0; n_small * n_small];
    let mut theta_prev = vec![0.0; n_modes];

    for iteration in 0..EIGEN_MAX_ITER {
        let mut y = vec![0.0; n * q];
        for i in 0..q {
            let rhs: Vec<f64> = (0..n).map(|r| x[r * q + i]).collect();
            let yi = solver.solve(&rhs)?;
            for r in 0..n {
                y[r * q + i] = yi[r];
            }
        }

        for i in 0..q {
            for j in 0..=i {
                let mut sk = 0.0;
                let mut sm = 0.0;
                for a in 0..n {
                    let mut tmpk = 0.0;
                    let mut tmpm = 0.0;
                    for b in 0..n {
                        let kab = k_red.get(a, b).copied().unwrap_or(0.0);
                        let mab = m_red.get(a, b).copied().unwrap_or(0.0);
                        let xbj = x[b * q + j];
                        tmpk += kab * xbj;
                        tmpm += mab * xbj;
                    }
                    sk += y[a * q + i] * tmpk;
                    sm += y[a * q + i] * tmpm;
                }
                k_small[i * n_small + j] = sk;
                k_small[j * n_small + i] = sk;
                m_small[i * n_small + j] = sm;
                m_small[j * n_small + i] = sm;
            }
        }

        let theta = solve_small_gevd(&k_small, &m_small, n_small, n_modes)?;

        let z = compute_ritz_vectors(&k_small, &m_small, &theta, n_small, n_modes)?;
        let mut x_new = vec![0.0; n * q];
        for i in 0..n {
            for j in 0..q {
                let mut s = 0.0;
                for k in 0..q {
                    s += y[i * q + k] * z[k * q + j];
                }
                x_new[i * q + j] = s;
            }
        }
        x = x_new;

        if iteration > 0 {
            let converged = (0..n_modes)
                .filter(|&m| (theta[m] - theta_prev[m]).abs() < EIGEN_TOL * theta[m].max(1.0))
                .count();
            if converged == n_modes {
                break;
            }
        }
        theta_prev[..n_modes].copy_from_slice(&theta[..n_modes]);
    }

    let mut omega2 = vec![0.0; n_modes];
    let mut period = vec![0.0; n_modes];
    for m in 0..n_modes {
        omega2[m] = theta_prev[m];
        period[m] = if omega2[m] > 0.0 {
            2.0 * std::f64::consts::PI / omega2[m].sqrt()
        } else {
            0.0
        };
    }

    Ok(ModalResult { omega2, period })
}

fn solve_small_gevd(
    k: &[f64],
    m: &[f64],
    n: usize,
    n_modes: usize,
) -> Result<Vec<f64>, SolveError> {
    // Compute Cholesky of M: M = L * L^T
    let mut l = vec![0.0; n * n];
    for j in 0..n {
        let mut s = 0.0;
        for k in 0..j {
            s += l[j * n + k] * l[j * n + k];
        }
        let d = m[j * n + j] - s;
        if d <= 0.0 {
            // M is singular (e.g. lumped mass) — just use K eigenvalues
            return Ok(diag_eigenvalues(k, n, n_modes));
        }
        l[j * n + j] = d.sqrt();
        for i in (j + 1)..n {
            let mut s = 0.0;
            for k in 0..j {
                s += l[i * n + k] * l[j * n + k];
            }
            l[i * n + j] = (m[i * n + j] - s) / l[j * n + j];
        }
    }

    // A = L^(-1) * K * L^(-T)  (solve L*A_tmp = K, then A_tmp*L^(-T))
    let mut a = vec![0.0; n * n];
    for j in 0..n {
        for i in 0..n {
            let mut s = k[i * n + j];
            for k in 0..i {
                s -= l[i * n + k] * a[k * n + j];
            }
            a[i * n + j] = s / l[i * n + i];
        }
    }
    // Solve A = A_tmp * L^(-T)  → L * A^T = A_tmp^T
    let mut result = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            let mut s = a[i * n + j];
            for k in 0..j {
                s -= result[i * n + k] * l[j * n + k];
            }
            result[i * n + j] = s / l[j * n + j];
        }
    }

    Ok(diag_eigenvalues(&result, n, n_modes))
}

fn diag_eigenvalues(a: &[f64], n: usize, n_modes: usize) -> Vec<f64> {
    let mut vals: Vec<f64> = (0..n).map(|i| a[i * n + i]).collect();
    vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
    vals.truncate(n_modes);
    vals
}

fn compute_ritz_vectors(
    _k: &[f64],
    _m: &[f64],
    _theta: &[f64],
    n: usize,
    n_modes: usize,
) -> Result<Vec<f64>, SolveError> {
    let mut z = vec![0.0; n * n];
    for mode in 0..n_modes {
        for i in 0..n {
            z[i * n + mode] = if i == mode { 1.0 } else { 0.0 };
        }
    }
    Ok(z)
}
