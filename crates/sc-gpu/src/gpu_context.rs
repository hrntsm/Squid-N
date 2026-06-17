use crate::spmv::{make_spmv_gpu, SpMv};
use faer::sparse::SparseColMat;

pub struct GpuContext {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

impl GpuContext {
    pub async fn try_new() -> Option<Self> {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await
            .ok()?;
        Some(Self { device, queue })
    }

    pub fn make_spmv(&self, mat: &SparseColMat<usize, f64>) -> Box<dyn SpMv> {
        make_spmv_gpu(mat, self)
    }
}
