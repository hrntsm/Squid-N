use faer::sparse::SparseColMat;

pub trait SpMv {
    fn spmv(&self, x: &[f32]) -> Vec<f32>;
    fn nrows(&self) -> usize;
    fn ncols(&self) -> usize;
}

#[derive(Clone, Debug)]
pub struct CsrMatrix {
    pub nrows: usize,
    pub ncols: usize,
    pub row_ptr: Vec<u32>,
    pub col_idx: Vec<u32>,
    pub values: Vec<f32>,
}

pub struct CpuSpMv {
    csr: CsrMatrix,
}

impl CpuSpMv {
    pub fn from_csc(mat: &SparseColMat<usize, f64>) -> Self {
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
        let mut vals = vec![0.0f32; nnz];
        let mut cur = row_ptr[..n].to_vec();

        for c in 0..n {
            let start = col_ptr[c];
            let end = col_ptr[c + 1];
            for pos in start..end {
                let r = row_idx[pos];
                let dest = cur[r] as usize;
                col_idx[dest] = c as u32;
                vals[dest] = values[pos] as f32;
                cur[r] += 1;
            }
        }

        Self {
            csr: CsrMatrix {
                nrows: n,
                ncols: mat.ncols(),
                row_ptr,
                col_idx,
                values: vals,
            },
        }
    }

    pub fn csr(&self) -> &CsrMatrix {
        &self.csr
    }
}

impl SpMv for CpuSpMv {
    fn spmv(&self, x: &[f32]) -> Vec<f32> {
        let mut y = vec![0.0f32; self.csr.nrows];
        for (r, yi) in y.iter_mut().enumerate() {
            let start = self.csr.row_ptr[r] as usize;
            let end = self.csr.row_ptr[r + 1] as usize;
            let mut acc = 0.0f32;
            for k in start..end {
                acc += self.csr.values[k] * x[self.csr.col_idx[k] as usize];
            }
            *yi = acc;
        }
        y
    }

    fn nrows(&self) -> usize {
        self.csr.nrows
    }

    fn ncols(&self) -> usize {
        self.csr.ncols
    }
}

#[cfg(feature = "gpu")]
pub struct GpuSpMv {
    nrows: usize,
    ncols: usize,
}

#[cfg(feature = "gpu")]
impl GpuSpMv {
    pub fn from_csc(_mat: &SparseColMat<usize, f64>, _ctx: &super::GpuContext) -> Self {
        Self {
            nrows: _mat.nrows(),
            ncols: _mat.ncols(),
        }
    }
}

#[cfg(feature = "gpu")]
impl SpMv for GpuSpMv {
    fn spmv(&self, _x: &[f32]) -> Vec<f32> {
        unimplemented!("GPU SpMV kernel - T1 to be implemented with cubecl")
    }
    fn nrows(&self) -> usize {
        self.nrows
    }
    fn ncols(&self) -> usize {
        self.ncols
    }
}

#[cfg(feature = "gpu")]
pub fn make_spmv_gpu(mat: &SparseColMat<usize, f64>, ctx: &super::GpuContext) -> Box<dyn SpMv> {
    Box::new(GpuSpMv::from_csc(mat, ctx))
}

pub fn make_spmv(mat: &SparseColMat<usize, f64>) -> Box<dyn SpMv> {
    Box::new(CpuSpMv::from_csc(mat))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_math::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_cpu_spmv_identity() {
        let mat = assemble_csc(
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
        let spmv = CpuSpMv::from_csc(&mat);
        let y = spmv.spmv(&[3.0, 5.0]);
        assert!((y[0] - 3.0).abs() < 1e-6);
        assert!((y[1] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_cpu_spmv_2x2() {
        let mat = assemble_csc(
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
        let spmv = CpuSpMv::from_csc(&mat);
        let y = spmv.spmv(&[2.0, 3.0]);
        assert!((y[0] - 0.0).abs() < 1e-6);
        assert!((y[1] - 200.0).abs() < 1e-6);
    }

    #[test]
    fn test_make_spmv_fallback() {
        let mat = assemble_csc(
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
        let spmv = make_spmv(&mat);
        let y = spmv.spmv(&[7.0, 11.0]);
        assert!((y[0] - 7.0).abs() < 1e-6);
        assert!((y[1] - 11.0).abs() < 1e-6);
    }
}
