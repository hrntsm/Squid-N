use sc_core::ids::ElemId;
use sc_core::model::{DistributionMethod, Model, Slab};

pub enum LoadShape {
    Uniform { w: f64 },
    Trapezoid { w0: f64, a: f64, b: f64 },
    Triangle { w0: f64 },
    Point { p: f64, x: f64 },
}

pub struct Cmq {
    pub c_i: f64,
    pub c_j: f64,
    pub q_i: f64,
    pub q_j: f64,
}

pub struct BeamLoad {
    pub elem: ElemId,
    pub shape: LoadShape,
    pub cmq: Cmq,
}

#[allow(clippy::let_and_return, unused_mut)]
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    let _ = model;
    let loads = Vec::new();
    for load in &slab.loads {
        let total_w = load.value;
        let _area = slab_area(slab);
        match slab.method {
            DistributionMethod::TriTrapezoid => {
                // Two-way slab: 45° distribution
                // Find supporting beams from model
                let _ = total_w;
            }
            DistributionMethod::OneWay => {
                let _ = total_w;
            }
            DistributionMethod::TributaryArea => {
                let _ = total_w;
            }
        }
    }
    loads
}

#[allow(dead_code, unused_variables)]
fn slab_area(slab: &Slab) -> f64 {
    if slab.boundary.len() < 3 {
        return 0.0;
    }
    0.0
}

#[allow(dead_code)]
fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

#[allow(dead_code)]
fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fem_uniform() {
        let cmq = fem_uniform(10.0, 4000.0);
        let expected = 10.0 * 4000.0_f64.powi(2) / 12.0;
        assert!((cmq.c_i - expected).abs() < 1e-6);
        assert_eq!(cmq.q_i, 10.0 * 4000.0 / 2.0);
    }

    #[test]
    fn test_fem_triangle() {
        let cmq = fem_triangle(10.0, 4000.0);
        let expected = 5.0 * 10.0 * 4000.0_f64.powi(2) / 96.0;
        assert!((cmq.c_i - expected).abs() < 1e-6);
    }
}
