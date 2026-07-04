use crate::solver::{LinearSolver, SolveError};
use faer::linalg::solvers::Solve;
use faer::sparse::linalg::solvers::{Lu, SymbolicLu};
use faer::sparse::SparseColMat;
use faer::Mat;

/// 疎 LU 直接法ソルバ。対称正定値でない系（非対称化した剛性、ラグランジュ
/// 乗数付き拘束など）にも使えるフォールバック。
#[derive(Default)]
pub struct LuSolver {
    factor: Option<Lu<usize, f64>>,
    n: usize,
}

impl LinearSolver for LuSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        self.n = k.nrows();
        let symbolic = SymbolicLu::try_new(k.symbolic())
            .map_err(|e| SolveError::Backend(format!("symbolic: {e:?}")))?;
        let lu = Lu::try_new_with_symbolic(symbolic, k.as_ref())
            .map_err(|e| SolveError::Backend(format!("LU factorize: {e:?}")))?;
        self.factor = Some(lu);
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let lu = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        if rhs.len() != self.n {
            return Err(SolveError::DimMismatch {
                k: self.n,
                rhs: rhs.len(),
            });
        }
        let b = Mat::from_fn(self.n, 1, |i, _| rhs[i]);
        let x = lu.solve(b.as_ref());
        Ok((0..self.n).map(|i| x[(i, 0)]).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_lu_2dof_spring() {
        faer::set_global_parallelism(faer::Par::Seq);
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
        let mut solver = LuSolver::default();
        solver.factorize(&k).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }

    /// 非対称行列も解ける（Cholesky では対象外のケース）。
    #[test]
    fn test_lu_unsymmetric() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 2.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: 0.5,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 3.0,
                },
            ],
        );
        let mut solver = LuSolver::default();
        solver.factorize(&k).unwrap();
        // [2 1; 0.5 3] x = [4; 6.5] -> x = [1; 2]
        let x = solver.solve(&[4.0, 6.5]).unwrap();
        approx::assert_relative_eq!(x[0], 1.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 2.0, max_relative = 1e-9);
    }

    #[test]
    fn test_lu_not_factorized() {
        let solver = LuSolver::default();
        assert!(matches!(
            solver.solve(&[1.0]),
            Err(SolveError::NotFactorized)
        ));
    }
}
