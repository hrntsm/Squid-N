use sc_section::SectionShape;

pub struct SkeletonPoint {
    pub deformation: f64,
    pub force: f64,
}

pub struct SkeletonCurve {
    pub points: Vec<SkeletonPoint>,
    pub yield_deformation: f64,
    pub yield_force: f64,
    pub ultimate_deformation: f64,
    pub ultimate_force: f64,
}

/// 履歴則の種別（設計書 §8）
#[derive(Clone, Debug)]
pub enum HysteresisRule {
    Elastic,
    Bilinear,
    Takeda,
    OriginOriented,
    Slip,
}

/// 軸力相互作用の定義（設計書 §8）
#[derive(Clone, Debug)]
pub struct AxialInteraction {
    pub n_yield: f64,
    pub n_balance: f64,
}

/// 部材のスケルトン＋履歴パラメータ（設計書 §8 / P5 §3）
#[derive(Clone, Debug)]
pub struct MemberSkeleton {
    pub points: Vec<(f64, f64)>,
    pub hysteresis: HysteresisRule,
    pub axial_dependency: AxialInteraction,
}

impl Default for MemberSkeleton {
    fn default() -> Self {
        MemberSkeleton {
            points: vec![(0.0, 0.0), (1.0, 100.0)],
            hysteresis: HysteresisRule::Elastic,
            axial_dependency: AxialInteraction {
                n_yield: 0.0,
                n_balance: 0.0,
            },
        }
    }
}

pub fn generate_flexural_skeleton(
    _shape: &SectionShape,
    _axial_force: f64,
    _length: f64,
) -> SkeletonCurve {
    SkeletonCurve {
        points: vec![
            SkeletonPoint {
                deformation: 0.0,
                force: 0.0,
            },
            SkeletonPoint {
                deformation: 1.0,
                force: 100.0,
            },
        ],
        yield_deformation: 1.0,
        yield_force: 100.0,
        ultimate_deformation: 4.0,
        ultimate_force: 80.0,
    }
}

pub fn generate_shear_skeleton(_shape: &SectionShape, _axial_force: f64) -> SkeletonCurve {
    SkeletonCurve {
        points: vec![
            SkeletonPoint {
                deformation: 0.0,
                force: 0.0,
            },
            SkeletonPoint {
                deformation: 1.0,
                force: 200.0,
            },
        ],
        yield_deformation: 1.0,
        yield_force: 200.0,
        ultimate_deformation: 3.0,
        ultimate_force: 160.0,
    }
}
