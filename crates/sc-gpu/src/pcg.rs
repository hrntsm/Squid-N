use faer::sparse::SparseColMat;
use sc_math::solver::{LinearSolver, SolveError};

use crate::spmv;

pub struct PcgGpu {
    inner: Option<spmv::CpuSpMv>,
    tol: f64,
    max_iter: usize,
    n: usize,
}

impl PcgGpu {
    pub fn new(_ctx: &super::GpuContext, tol: f64, max_iter: usize) -> Self {
        Self {
            inner: None,
            tol,
            max_iter,
            n: 0,
        }
    }
}

impl LinearSolver for PcgGpu {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        use sc_math::pcg::PcgSolver;
        let mut cpu = PcgSolver::new(self.tol, self.max_iter);
        cpu.factorize(k)?;
        self.n = k.nrows();
        self.inner = Some(spmv::CpuSpMv::from_csc(k));
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let _spmv = self.inner.as_ref().ok_or(SolveError::NotFactorized)?;
        use sc_math::pcg::PcgSolver;
        let mut cpu = PcgSolver::new(self.tol, self.max_iter);
        cpu.factorize(
            &faer::sparse::SparseColMat::<usize, f64>::try_new_from_triplets(self.n, self.n, &[])
                .map_err(|e| SolveError::Backend(format!("{e:?}")))?,
        )?;
        cpu.solve(rhs)
    }
}
