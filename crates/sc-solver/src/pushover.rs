use crate::analysis::{AiMode, Analysis, SeismicDir};
use sc_core::ids::{ElemId, StoryId};
use sc_math::solver::SolveError;

/// 性能曲線の1点（P5 §7.4）
pub struct CapacityPoint {
    pub step: u32,
    pub roof_disp: f64,
    pub base_shear: f64,
    pub story_shear: Vec<f64>,
    pub story_drift: Vec<f64>,
}

/// ヒンジ発生事象（P5 §7.4）
pub struct HingeEvent {
    pub step: u32,
    pub elem: ElemId,
    pub pos: f64,
    pub level: HingeLevel,
    pub ductility: f64,
}

/// ヒンジレベル（P5 §7.4）
pub enum HingeLevel {
    Crack,
    Yield,
    Ultimate,
}

/// 崩壊機構種別（P5 §7.4）
pub enum MechanismType {
    Overall,
    StoryCollapse { story: StoryId },
    Partial,
}

/// プッシュオーバー解析結果（P5 §7.4）
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
}

/// プッシュオーバー解析（現在は弾性1ステップのスタブ）
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
        capacity_curve: vec![CapacityPoint {
            step: 0,
            roof_disp: top_disp,
            base_shear,
            story_shear: vec![],
            story_drift: vec![],
        }],
        hinges: vec![],
        mechanism: crate::pushover::MechanismType::Partial,
        qu: base_shear,
    })
}
