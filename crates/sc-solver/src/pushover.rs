use crate::analysis::{AiMode, Analysis, SeismicDir};
use sc_math::solver::SolveError;

pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
}

pub fn pushover_analysis(
    analysis: &Analysis,
    dir: SeismicDir,
    _max_steps: usize,
    _max_disp: f64,
) -> Result<PushoverResult, SolveError> {
    let result = analysis.seismic_static(dir, AiMode::Approx)?;
    let top_disp = result
        .disp
        .last()
        .map(|d| match dir {
            SeismicDir::X => d[0],
            SeismicDir::Y => d[1],
        })
        .unwrap_or(0.0);
    let base_shear = result
        .member_forces
        .iter()
        .flat_map(|(_, f)| f.at.first())
        .map(|(_, f)| f[0].abs())
        .sum();

    Ok(PushoverResult {
        steps: vec![PushoverStep {
            load_factor: 1.0,
            top_disp,
            base_shear,
            story_drifts: vec![],
        }],
    })
}
