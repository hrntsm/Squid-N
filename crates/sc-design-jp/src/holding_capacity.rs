use sc_core::ids::{ElemId, StoryId};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MemberRank {
    FA,
    FB,
    FC,
    FD,
}

pub struct StoryCheck {
    pub story: StoryId,
    pub rs: f64,
    pub re: f64,
    pub ds: f64,
    pub fes: f64,
    pub qu: f64,
    pub qud: f64,
    pub qun: f64,
    pub drift_angle: f64,
    pub ok: bool,
}

pub struct HoldingCapacityResult {
    pub stories: Vec<StoryCheck>,
    pub member_ranks: Vec<(ElemId, MemberRank)>,
}

pub fn check_story_drift(story_height: f64, interstory_drift: f64) -> bool {
    let angle = interstory_drift / story_height;
    angle <= 1.0 / 200.0
}

pub fn stiffness_ratio(story_stiffness: f64, upper_stiffness: f64, lower_stiffness: f64) -> f64 {
    let avg = (upper_stiffness + lower_stiffness) / 2.0;
    if avg == 0.0 {
        return 1.0;
    }
    let rs = story_stiffness / avg;
    rs.clamp(0.6, 1.0)
}

pub fn eccentricity_ratio(e: f64, r: f64) -> f64 {
    if r == 0.0 {
        return 0.0;
    }
    (e / r).abs()
}

pub fn ds_from_member_ranks(ranks: &[MemberRank]) -> f64 {
    let worst = ranks
        .iter()
        .max_by_key(|r| match *r {
            MemberRank::FD => 4,
            MemberRank::FC => 3,
            MemberRank::FB => 2,
            MemberRank::FA => 1,
        })
        .unwrap_or(&MemberRank::FA);
    match worst {
        MemberRank::FA => 0.25,
        MemberRank::FB => 0.30,
        MemberRank::FC => 0.35,
        MemberRank::FD => 0.40,
    }
}

pub fn fes(rs: f64, re: f64) -> f64 {
    let f_rs = if rs >= 0.6 { 1.0 } else { 1.0 / rs.sqrt() };
    let f_re = if re <= 0.15 {
        1.0
    } else {
        (1.0 + 2.0 * re).sqrt()
    };
    (f_rs * f_re).clamp(1.0, 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_ds_fa() {
        assert!((ds_from_member_ranks(&[MemberRank::FA]) - 0.25).abs() < 1e-6);
    }
    #[test]
    fn test_fes_default() {
        let f = fes(0.8, 0.1);
        assert!(f >= 1.0);
    }
}
