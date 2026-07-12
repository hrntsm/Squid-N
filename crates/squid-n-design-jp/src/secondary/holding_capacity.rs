use squid_n_core::ids::{ElemId, StoryId};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

// ===== T1: 剛性率 Rs・層間変形角 (§4) =====

pub fn check_story_drift(story_height: f64, interstory_drift: f64) -> bool {
    let angle = interstory_drift / story_height;
    angle <= 1.0 / 200.0
}

/// 全層の剛性率 Rs_i を計算する。
/// Ks_i = h_i / δ_i,  Rs_i = Ks_i / mean(Ks)
pub fn stiffness_ratios(story_heights: &[f64], story_drifts: &[f64]) -> Vec<f64> {
    let ks: Vec<f64> = story_heights
        .iter()
        .zip(story_drifts)
        .map(|(h, d)| if *d == 0.0 { 0.0 } else { h / d })
        .collect();
    let n = ks.len() as f64;
    if n == 0.0 {
        return vec![];
    }
    let mean = ks.iter().sum::<f64>() / n;
    if mean == 0.0 {
        return vec![1.0; ks.len()];
    }
    ks.iter().map(|k| k / mean).collect()
}

// ===== T2: 偏心率 Re (§5.2) =====

/// 偏心距離 e を弾力半径 r で割った偏心率。
pub fn eccentricity_ratio(e: f64, r: f64) -> f64 {
    if r == 0.0 {
        return 0.0;
    }
    (e / r).abs()
}

// ===== T3: Fs / Fe / Fes (§5.3) =====

/// 剛性率 Rs から Fs を算定（告示1792）。
/// Rs ≥ 0.6 → Fs = 1.0
/// Rs < 0.6 → Fs = 2.0 − Rs/0.6
pub fn fs(rs: f64) -> f64 {
    if rs >= 0.6 {
        1.0
    } else {
        2.0 - rs / 0.6
    }
}

/// 偏心率 Re から Fe を算定（告示1792）。
/// Re ≤ 0.15 → Fe = 1.0
/// Re > 0.15 → Fe = 1.0 + 0.5·(Re − 0.15)/0.15（最大 1.5）
pub fn fe(re: f64) -> f64 {
    if re <= 0.15 {
        1.0
    } else {
        (1.0 + 0.5 * (re - 0.15) / 0.15).min(1.5)
    }
}

/// 形状係数 Fes = Fs · Fe。
pub fn fes(rs: f64, re: f64) -> f64 {
    fs(rs) * fe(re)
}

// ===== T4: Ds 自動分類 (§7) =====

/// 架構種別。Ds 表の行を選ぶ。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameType {
    RcFrame,
    RcWall,
    SteelFrame,
    SteelBrace,
}

/// 告示1792 の Ds 値。
pub fn ds_value(frame: FrameType, rank: MemberRank) -> f64 {
    use FrameType::*;
    use MemberRank::*;
    match (frame, rank) {
        (RcFrame, FA) => 0.30,
        (RcFrame, FB) => 0.35,
        (RcFrame, FC) => 0.40,
        (RcFrame, FD) => 0.45,
        (RcWall, FA) => 0.35,
        (RcWall, FB) => 0.40,
        (RcWall, FC) => 0.45,
        (RcWall, FD) => 0.55,
        (SteelFrame, FA) => 0.25,
        (SteelFrame, FB) => 0.30,
        (SteelFrame, FC) => 0.35,
        (SteelFrame, FD) => 0.40,
        (SteelBrace, FA) => 0.30,
        (SteelBrace, FB) => 0.35,
        (SteelBrace, FC) => 0.40,
        (SteelBrace, FD) => 0.50,
    }
}

// ===== T2: Qud ヘルパー (§2) =====

/// 二次設計の地震時層せん断 Qud（**C0 = 1.0** の Ai 分布層せん断 Qi）。仕様 §2。
///
/// - `story_weights_bottom_to_top`: 下→上の各層の地震用重量。
/// - `z`: 地域係数。
/// - `rt`: 振動特性係数 Rt。
/// - `t`: 設計用一次固有周期 [s]。
///
/// 戻り値は層せん断 Qi（下→上インデックス）。
pub fn qud_by_story(story_weights_bottom_to_top: &[f64], z: f64, rt: f64, t: f64) -> Vec<f64> {
    squid_n_load::ai::ai_distribution(story_weights_bottom_to_top, z, rt, 1.0, t).qi
}

// ===== T6: Qun 比較・判定・統合 (§3) =====

use squid_n_solver::pushover::PushoverResult;

/// 二次設計（保有水平耐力）の層チェックを統合する。
///
/// **Qu（保有水平耐力）は P5 プッシュオーバーから取得する**（DoD §0.2-1）。
/// `pushover.capacity_curve` の最終点（崩壊機構形成時）の層せん断 `story_shear[i]` を
/// 層 i の Qu、`story_drift[i]` を層間変位とする。capacity_curve が空なら Qu=0。
///
/// 他の量は各タスクで算定した層別配列を渡す:
/// - `qud_by_story`: 二次設計用の地震時層せん断（**Ai 分布・C0=1.0** で算定したもの。§2）。
/// - `ds_by_story`: 層 Ds（[`crate::secondary::member_rank::story_ds`]）。
/// - `fes_by_story`: 形状係数 Fes（[`fes`]）。
/// - `rs_by_story` / `re_by_story`: 剛性率（[`stiffness_ratios`]）・偏心率
///   （[`crate::secondary::eccentricity`]）。
/// - `story_heights`: 階高（層間変形角＝層間変位/階高 の算定に使用）。
/// - `member_ranks`: 部材ランク一覧（出力にそのまま格納）。
///
/// 層数 n は `qud_by_story.len()`。判定は `ok = (Qu ≥ Qun)`, `Qun = Ds·Fes·Qud`。
#[allow(clippy::too_many_arguments)]
pub fn check_holding_capacity(
    pushover: &PushoverResult,
    qud_by_story: &[f64],
    ds_by_story: &[f64],
    fes_by_story: &[f64],
    rs_by_story: &[f64],
    re_by_story: &[f64],
    story_heights: &[f64],
    member_ranks: Vec<(ElemId, MemberRank)>,
) -> HoldingCapacityResult {
    let last_point = pushover.capacity_curve.last();
    let n = qud_by_story.len();

    let stories: Vec<StoryCheck> = (0..n)
        .map(|i| {
            let story = StoryId(i as u32);
            // Qu・層間変位は P5 プッシュオーバー最終点から。
            let qu = last_point
                .and_then(|p| p.story_shear.get(i))
                .copied()
                .unwrap_or(0.0);
            let drift = last_point
                .and_then(|p| p.story_drift.get(i))
                .copied()
                .unwrap_or(0.0);
            let height = *story_heights.get(i).unwrap_or(&1.0);
            let drift_angle = if height == 0.0 { 0.0 } else { drift / height };

            let qud = *qud_by_story.get(i).unwrap_or(&0.0);
            let ds = *ds_by_story.get(i).unwrap_or(&0.0);
            let f = *fes_by_story.get(i).unwrap_or(&1.0);
            let rs = *rs_by_story.get(i).unwrap_or(&0.0);
            let re = *re_by_story.get(i).unwrap_or(&0.0);
            let qun = ds * f * qud;
            let ok = qu >= qun;
            StoryCheck {
                story,
                rs,
                re,
                ds,
                fes: f,
                qu,
                qud,
                qun,
                drift_angle,
                ok,
            }
        })
        .collect();

    HoldingCapacityResult {
        stories,
        member_ranks,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- T1 ----
    #[test]
    fn test_stiffness_ratios_example() {
        let heights = vec![1.0, 1.0, 1.0];
        let drifts = vec![1.0 / 200.0, 1.0 / 150.0, 1.0 / 250.0];
        let rs = stiffness_ratios(&heights, &drifts);
        assert_eq!(rs.len(), 3);
        for (r, expected) in rs.iter().zip([1.0, 0.75, 1.25].iter()) {
            assert!((r - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn test_stiffness_ratios_empty() {
        assert!(stiffness_ratios(&[], &[]).is_empty());
    }

    #[test]
    fn test_stiffness_ratios_zero_drift() {
        let rs = stiffness_ratios(&[1.0], &[0.0]);
        assert_eq!(rs, vec![1.0]);
    }

    #[test]
    fn test_check_story_drift_ok() {
        assert!(check_story_drift(3.0, 0.01));
        assert!(!check_story_drift(3.0, 0.02));
    }

    // ---- T2 ----
    #[test]
    fn test_eccentricity_ratio_basic() {
        assert!((eccentricity_ratio(1.5, 3.0) - 0.5).abs() < 1e-9);
        assert_eq!(eccentricity_ratio(1.5, 0.0), 0.0);
    }

    // ---- T3 ----
    #[test]
    fn test_fs_ge_06() {
        assert!((fs(0.6) - 1.0).abs() < 1e-9);
        assert!((fs(1.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fs_lt_06() {
        assert!((fs(0.3) - 1.5).abs() < 1e-9);
        assert!((fs(0.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_fe_le_015() {
        assert!((fe(0.15) - 1.0).abs() < 1e-9);
        assert!((fe(0.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fe_gt_015() {
        assert!((fe(0.30) - 1.5).abs() < 1e-9);
        assert!((fe(0.45) - 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_fes_default() {
        let f = fes(0.8, 0.1);
        assert!((f - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_fes_example() {
        assert!((fes(0.3, 0.30) - 2.25).abs() < 1e-9);
    }

    // ---- T4 ----
    #[test]
    fn test_ds_value_rc_frame() {
        assert!((ds_value(FrameType::RcFrame, MemberRank::FA) - 0.30).abs() < 1e-9);
        assert!((ds_value(FrameType::RcFrame, MemberRank::FD) - 0.45).abs() < 1e-9);
    }

    #[test]
    fn test_ds_value_steel_frame() {
        assert!((ds_value(FrameType::SteelFrame, MemberRank::FA) - 0.25).abs() < 1e-9);
        assert!((ds_value(FrameType::SteelFrame, MemberRank::FD) - 0.40).abs() < 1e-9);
    }

    #[test]
    fn test_ds_value_all_combinations() {
        for (f, r, expected) in [
            (FrameType::RcFrame, MemberRank::FA, 0.30),
            (FrameType::RcFrame, MemberRank::FB, 0.35),
            (FrameType::RcFrame, MemberRank::FC, 0.40),
            (FrameType::RcFrame, MemberRank::FD, 0.45),
            (FrameType::RcWall, MemberRank::FA, 0.35),
            (FrameType::RcWall, MemberRank::FD, 0.55),
            (FrameType::SteelFrame, MemberRank::FA, 0.25),
            (FrameType::SteelFrame, MemberRank::FD, 0.40),
            (FrameType::SteelBrace, MemberRank::FA, 0.30),
            (FrameType::SteelBrace, MemberRank::FD, 0.50),
        ] {
            assert!(
                (ds_value(f, r) - expected).abs() < 1e-9,
                "ds_value({:?}, {:?}) should be {}, got {}",
                f,
                r,
                expected,
                ds_value(f, r)
            );
        }
    }

    // ---- T6 ----
    /// Qu を持つ capacity_curve 1点だけの PushoverResult を作る。
    fn pushover_with_qu(story_shear: Vec<f64>, story_drift: Vec<f64>) -> PushoverResult {
        use squid_n_solver::pushover::{CapacityPoint, MechanismType};
        PushoverResult {
            steps: vec![],
            capacity_curve: vec![CapacityPoint {
                step: 0,
                roof_disp: 0.0,
                base_shear: story_shear.first().copied().unwrap_or(0.0),
                story_shear,
                story_drift,
            }],
            hinges: vec![],
            shear_yields: vec![],
            mechanism: MechanismType::Overall,
            qu: 0.0,
            member_response: vec![],
        }
    }

    #[test]
    fn test_check_holding_capacity_basic() {
        // Qu は pushover 最終点から取得（[100,200]）。
        let pushover = pushover_with_qu(vec![100.0, 200.0], vec![15.0, 12.0]);
        let qud = vec![80.0, 180.0];
        let ds = vec![0.30, 0.35];
        let fes = vec![1.0, 1.0];
        let rs = vec![1.0, 0.75];
        let re = vec![0.05, 0.10];
        let heights = vec![3000.0, 3000.0];
        let result = check_holding_capacity(&pushover, &qud, &ds, &fes, &rs, &re, &heights, vec![]);
        assert_eq!(result.stories.len(), 2);
        assert!(result.stories[0].ok); // Qu=100 ≥ Qun=24
        assert!(result.stories[1].ok); // Qu=200 ≥ Qun=63
        assert!((result.stories[0].qu - 100.0).abs() < 1e-9);
        assert!((result.stories[0].qun - 24.0).abs() < 1e-9);
        assert!((result.stories[1].qun - 63.0).abs() < 1e-9);
        // Rs/Re が出力に反映されている（旧実装は 0.0 固定だった）。
        assert!((result.stories[1].rs - 0.75).abs() < 1e-9);
        assert!((result.stories[0].re - 0.05).abs() < 1e-9);
        // 層間変形角 = drift/height = 15/3000 = 1/200。
        assert!((result.stories[0].drift_angle - 15.0 / 3000.0).abs() < 1e-9);
    }

    #[test]
    fn test_check_holding_capacity_ng() {
        let pushover = pushover_with_qu(vec![20.0, 50.0], vec![10.0, 10.0]);
        let qud = vec![80.0, 180.0];
        let ds = vec![0.30, 0.35];
        let fes = vec![1.0, 1.0];
        let rs = vec![1.0, 1.0];
        let re = vec![0.0, 0.0];
        let heights = vec![3000.0, 3000.0];
        let result = check_holding_capacity(&pushover, &qud, &ds, &fes, &rs, &re, &heights, vec![]);
        assert!(!result.stories[0].ok); // Qu=20 < Qun=24
        assert!(!result.stories[1].ok); // Qu=50 < Qun=63
    }

    /// 境界（Qu = Qun ちょうど）→ ok=true（DoD §3）。
    #[test]
    fn test_check_holding_capacity_boundary() {
        // Qun = Ds·Fes·Qud = 0.5·1.0·100 = 50。Qu = 50 ちょうど。
        let pushover = pushover_with_qu(vec![50.0], vec![5.0]);
        let qud = vec![100.0];
        let ds = vec![0.5];
        let fes = vec![1.0];
        let rs = vec![1.0];
        let re = vec![0.0];
        let heights = vec![3000.0];
        let result = check_holding_capacity(&pushover, &qud, &ds, &fes, &rs, &re, &heights, vec![]);
        assert!((result.stories[0].qun - 50.0).abs() < 1e-9);
        assert!(result.stories[0].ok, "Qu=Qun は ok（≥）であるべき");
    }

    // ---- Qud ヘルパー ----

    /// 等重量3層で qud_by_story の基部層せん断が C0=1.0 版 = C0=0.2 版の 5 倍であること。
    #[test]
    fn test_qud_by_story_linearity_in_c0() {
        use squid_n_load::ai::ai_distribution;
        let weights = &[1.0_f64, 1.0, 1.0];
        let z = 1.0;
        let rt = 1.0;
        let t = 0.24;
        let qud = qud_by_story(weights, z, rt, t);
        let qi_c0_02 = ai_distribution(weights, z, rt, 0.2, t).qi;
        // C0 に線形なので qud[0] = 5 · qi_c0_02[0]。
        assert!(
            (qud[0] - 5.0 * qi_c0_02[0]).abs() < 1e-9,
            "qud[0]={} vs 5×qi0.2[0]={}",
            qud[0],
            5.0 * qi_c0_02[0]
        );
        // 基部層せん断 ≈ C0·Ai0·総重量 = 1.0·1.0·3.0 = 3.0。
        assert!(
            (qud[0] - 3.0).abs() < 1e-9,
            "qud[0]={} (expected ≈3.0)",
            qud[0]
        );
    }

    /// 部材ランクが出力に格納される。
    #[test]
    fn test_check_holding_capacity_member_ranks() {
        let pushover = pushover_with_qu(vec![100.0], vec![5.0]);
        let ranks = vec![(ElemId(0), MemberRank::FA), (ElemId(1), MemberRank::FC)];
        let result = check_holding_capacity(
            &pushover,
            &[80.0],
            &[0.3],
            &[1.0],
            &[1.0],
            &[0.0],
            &[3000.0],
            ranks,
        );
        assert_eq!(result.member_ranks.len(), 2);
        assert_eq!(result.member_ranks[1].1, MemberRank::FC);
    }
}
