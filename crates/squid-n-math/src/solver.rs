use faer::sparse::SparseColMat;

pub trait LinearSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError>;
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError>;
}

#[derive(Debug, thiserror::Error)]
pub enum SolveError {
    #[error("not factorized yet")]
    NotFactorized,
    #[error("matrix not positive definite")]
    NotPositiveDefinite,
    #[error("dimension mismatch: K={k}, rhs={rhs}")]
    DimMismatch { k: usize, rhs: usize },
    #[error("faer error: {0}")]
    Backend(String),
    /// 入力モデル起因のエラー（拘束不足・断面/材料未割当など）。
    /// メッセージはユーザー向け診断文（日本語）を想定する。
    #[error("{0}")]
    InvalidInput(String),
    /// 反復法・固有値解析が規定回数内に収束しなかった。
    #[error("収束しませんでした: {0}")]
    NonConvergence(String),
}

#[derive(Clone, Copy, Debug)]
pub enum SolverBackend {
    DirectSparseCholesky,
    DirectSparseLu,
    IterativePcg { tol: f64, max_iter: usize },
}

pub fn make_solver(backend: SolverBackend) -> Box<dyn LinearSolver> {
    match backend {
        SolverBackend::DirectSparseCholesky => Box::new(crate::cholesky::CholeskySolver::default()),
        SolverBackend::IterativePcg { tol, max_iter } => {
            Box::new(crate::pcg::PcgSolver::new(tol, max_iter))
        }
        SolverBackend::DirectSparseLu => Box::new(crate::lu::LuSolver::default()),
    }
}
