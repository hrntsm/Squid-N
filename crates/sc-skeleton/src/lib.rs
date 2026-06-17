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
