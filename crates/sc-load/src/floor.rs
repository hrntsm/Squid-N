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

/// スラブの面荷重を大梁へ分配する。
/// 正方形スラブの場合、4辺すべてに三角形分配（45°法）。
/// 矩形スラブの場合、短辺方向に三角形、長辺方向に台形。
/// ※ slab.boundary は節点IDの並び。サポート梁の特定にはモデル参照が必要だが、
///   ここでは分配された荷重強度とCMQを計算し、呼び出し側が節点→部材を対応づける。
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    let (lx, ly) = slab_dimensions(model, slab);
    if lx <= 0.0 || ly <= 0.0 {
        return loads;
    }

    for area_load in &slab.loads {
        let w = area_load.value;

        match slab.method {
            DistributionMethod::TriTrapezoid => {
                let is_square = (lx - ly).abs() < 1e-6;
                if is_square {
                    let w0 = w * lx / 2.0;
                    for i in 0..4 {
                        let l = if i % 2 == 0 { lx } else { ly };
                        loads.push(BeamLoad {
                            elem: ElemId(i as u32),
                            shape: LoadShape::Triangle { w0 },
                            cmq: fem_triangle(w0, l),
                        });
                    }
                } else {
                    let short = lx.min(ly);
                    let long = lx.max(ly);
                    let w0 = w * short / 2.0;
                    let a = short / 2.0;
                    let b = long - 2.0 * a;

                    for i in 0..4 {
                        let l = if i % 2 == 0 { lx } else { ly };
                        let is_short_side = (l - short).abs() < 1e-6;
                        if is_short_side {
                            loads.push(BeamLoad {
                                elem: ElemId(i as u32),
                                shape: LoadShape::Triangle { w0 },
                                cmq: fem_triangle(w0, l),
                            });
                        } else {
                            loads.push(BeamLoad {
                                elem: ElemId(i as u32),
                                shape: LoadShape::Trapezoid { w0, a, b },
                                cmq: fem_trapezoid(w0, a, b, l),
                            });
                        }
                    }
                }
            }
            DistributionMethod::OneWay => {
                let w_line = w * ly;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    if (l - lx).abs() < 1e-6 {
                        loads.push(BeamLoad {
                            elem: ElemId(i as u32),
                            shape: LoadShape::Uniform { w: w_line },
                            cmq: fem_uniform(w_line, l),
                        });
                    }
                }
            }
            DistributionMethod::TributaryArea => {
                let w_line = w * lx / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    loads.push(BeamLoad {
                        elem: ElemId(i as u32),
                        shape: LoadShape::Uniform { w: w_line },
                        cmq: fem_uniform(w_line, l),
                    });
                }
            }
        }
    }

    loads
}

fn slab_dimensions(model: &Model, slab: &Slab) -> (f64, f64) {
    if slab.boundary.len() < 4 {
        return (0.0, 0.0);
    }
    let p0 = model.nodes.get(slab.boundary[0].index()).map(|n| n.coord);
    let p1 = model.nodes.get(slab.boundary[1].index()).map(|n| n.coord);
    let p3 = model.nodes.get(slab.boundary[3].index()).map(|n| n.coord);
    match (p0, p1, p3) {
        (Some(c0), Some(c1), Some(c3)) => {
            let lx = ((c1[0] - c0[0]).powi(2) + (c1[1] - c0[1]).powi(2) + (c1[2] - c0[2]).powi(2))
                .sqrt();
            let ly = ((c3[0] - c0[0]).powi(2) + (c3[1] - c0[1]).powi(2) + (c3[2] - c0[2]).powi(2))
                .sqrt();
            (lx, ly)
        }
        _ => (0.0, 0.0),
    }
}

fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

#[allow(unused_variables)]
fn fem_trapezoid(w0: f64, a: f64, b: f64, l: f64) -> Cmq {
    let cmq_uni = fem_uniform(w0, b);
    let cmq_tri = fem_triangle(w0, 2.0 * a);
    Cmq {
        c_i: cmq_tri.c_i + cmq_uni.c_i,
        c_j: -(cmq_tri.c_i.abs() + cmq_uni.c_i.abs()),
        q_i: cmq_tri.q_i + cmq_uni.q_i,
        q_j: cmq_tri.q_j + cmq_uni.q_j,
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
    fn test_fem_triangle_spec() {
        let w0 = 10.0_f64;
        let l = 4000.0_f64;
        let cmq = fem_triangle(w0, l);
        let expected = 5.0 * w0 * l.powi(2) / 96.0;
        assert!(
            (cmq.c_i - expected).abs() < 1e-3,
            "FEM={} expected={}",
            cmq.c_i,
            expected
        );
        assert!((expected - 8.3333e6).abs() < 1.0e3, "expected={}", expected);
    }
}
