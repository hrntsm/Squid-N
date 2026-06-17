use crate::solver::{LinearSolver, SolveError};
use faer::sparse::SparseColMat;

pub struct PcgSolver {
    a: Option<CsrF32>,
    diag: Option<Vec<f32>>,
    tol: f64,
    max_iter: usize,
    n: usize,
}

struct CsrF32 {
    n: usize,
    row_ptr: Vec<u32>,
    col_idx: Vec<u32>,
    val: Vec<f32>,
}

fn csc_to_csr_f32(mat: &SparseColMat<usize, f64>) -> CsrF32 {
    let n = mat.nrows();
    let col_ptr = mat.col_ptr();
    let row_idx = mat.row_idx();
    let values = mat.val();
    let nnz = values.len();

    let mut row_nnz = vec![0u32; n];
    for &r in row_idx.iter() {
        row_nnz[r] += 1;
    }

    let mut row_ptr = vec![0u32; n + 1];
    let mut prefix = 0u32;
    for r in 0..n {
        row_ptr[r] = prefix;
        prefix += row_nnz[r];
    }
    row_ptr[n] = prefix;

    let mut col_idx = vec![0u32; nnz];
    let mut val = vec![0.0f32; nnz];
    let mut cur = row_ptr[..n].to_vec();

    for c in 0..n {
        let start = col_ptr[c];
        let end = col_ptr[c + 1];
        for pos in start..end {
            let r = row_idx[pos];
            let dest = cur[r] as usize;
            col_idx[dest] = c as u32;
            val[dest] = values[pos] as f32;
            cur[r] += 1;
        }
    }

    CsrF32 {
        n,
        row_ptr,
        col_idx,
        val,
    }
}

fn jacobi_preconditioner(csr: &CsrF32) -> Vec<f32> {
    let mut d = vec![1.0f32; csr.n];
    for (r, di) in d.iter_mut().enumerate() {
        let start = csr.row_ptr[r] as usize;
        let end = csr.row_ptr[r + 1] as usize;
        for k in start..end {
            if csr.col_idx[k] == r as u32 {
                let aii = csr.val[k];
                if aii.abs() > 1e-30 {
                    *di = 1.0 / aii;
                }
                break;
            }
        }
    }
    d
}

fn csr_spmv(csr: &CsrF32, x: &[f32], y: &mut [f32]) {
    for (r, yi) in y.iter_mut().enumerate() {
        let start = csr.row_ptr[r] as usize;
        let end = csr.row_ptr[r + 1] as usize;
        let mut acc = 0.0f32;
        for k in start..end {
            acc += csr.val[k] * x[csr.col_idx[k] as usize];
        }
        *yi = acc;
    }
}

fn dot_f32(a: &[f32], b: &[f32]) -> f32 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

fn axpy_f32(y: &mut [f32], alpha: f32, x: &[f32]) {
    for (yi, xi) in y.iter_mut().zip(x.iter()) {
        *yi += alpha * xi;
    }
}

fn copy_f32(dst: &mut [f32], src: &[f32]) {
    dst.copy_from_slice(src);
}

impl PcgSolver {
    pub fn new(tol: f64, max_iter: usize) -> Self {
        Self {
            a: None,
            diag: None,
            tol,
            max_iter,
            n: 0,
        }
    }
}

impl LinearSolver for PcgSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        self.n = k.nrows();
        let csr = csc_to_csr_f32(k);
        self.diag = Some(jacobi_preconditioner(&csr));
        self.a = Some(csr);
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let csr = self.a.as_ref().ok_or(SolveError::NotFactorized)?;
        let diag = self.diag.as_ref().ok_or(SolveError::NotFactorized)?;
        if rhs.len() != self.n {
            return Err(SolveError::DimMismatch {
                k: self.n,
                rhs: rhs.len(),
            });
        }

        let n = self.n;
        let tol = self.tol as f32;
        let max_iter = self.max_iter;

        let mut x = vec![0.0f32; n];
        let mut r = vec![0.0f32; n];
        let mut z = vec![0.0f32; n];
        let mut p = vec![0.0f32; n];
        let mut q = vec![0.0f32; n];

        for i in 0..n {
            r[i] = rhs[i] as f32;
        }

        csr_spmv(csr, &x, &mut q);
        for i in 0..n {
            r[i] -= q[i];
        }

        let b_norm = (0..n).map(|i| (rhs[i] as f32).powi(2)).sum::<f32>().sqrt();
        if b_norm < 1e-30 {
            return Ok(vec![0.0f64; n]);
        }

        for i in 0..n {
            z[i] = diag[i] * r[i];
        }
        copy_f32(&mut p, &z);
        let mut rho = dot_f32(&r, &z);

        for _iter in 0..max_iter {
            csr_spmv(csr, &p, &mut q);
            let pq = dot_f32(&p, &q);
            if pq.abs() < 1e-30 {
                break;
            }
            let alpha = rho / pq;

            axpy_f32(&mut x, alpha, &p);
            axpy_f32(&mut r, -alpha, &q);

            let r_norm = dot_f32(&r, &r).sqrt();
            if r_norm / b_norm < tol {
                break;
            }

            for i in 0..n {
                z[i] = diag[i] * r[i];
            }
            let rho_next = dot_f32(&r, &z);
            let beta = rho_next / rho;
            rho = rho_next;

            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }
        }

        Ok((0..n).map(|i| x[i] as f64).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_pcg_2dof_spring() {
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 300.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -200.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -200.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 200.0,
                },
            ],
        );
        let mut solver = PcgSolver::new(1e-6, 1000);
        solver.factorize(&k).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-4);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-4);
    }

    #[test]
    fn test_pcg_not_factorized() {
        let solver = PcgSolver::new(1e-6, 100);
        let result = solver.solve(&[1.0, 2.0]);
        assert!(matches!(result, Err(SolveError::NotFactorized)));
    }

    #[test]
    fn test_pcg_dim_mismatch() {
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 1.0,
                },
            ],
        );
        let mut solver = PcgSolver::new(1e-6, 100);
        solver.factorize(&k).unwrap();
        let result = solver.solve(&[1.0]);
        assert!(matches!(result, Err(SolveError::DimMismatch { .. })));
    }

    #[test]
    fn test_pcg_agrees_with_direct() {
        use crate::cholesky::CholeskySolver;
        let k = assemble_csc(
            3,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 200.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -100.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -100.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 200.0,
                },
                Triplet {
                    row: 1,
                    col: 2,
                    val: -100.0,
                },
                Triplet {
                    row: 2,
                    col: 1,
                    val: -100.0,
                },
                Triplet {
                    row: 2,
                    col: 2,
                    val: 100.0,
                },
            ],
        );

        let mut direct = CholeskySolver::default();
        direct.factorize(&k).unwrap();
        let x_direct = direct.solve(&[100.0, 0.0, 0.0]).unwrap();

        let mut pcg = PcgSolver::new(1e-5, 1000);
        pcg.factorize(&k).unwrap();
        let x_pcg = pcg.solve(&[100.0, 0.0, 0.0]).unwrap();

        for i in 0..3 {
            approx::assert_relative_eq!(x_pcg[i], x_direct[i], max_relative = 1e-3);
        }
    }
}
