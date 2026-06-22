//! T4: 部材ランク（FA..FD）と層 Ds の自動分類。仕様 specs/P7_二次設計.md §7。
//! しきい値表（RC 耐力比境界・S 幅厚比限界）は AIJ 規準（Category B）の外部データ。
//! 本モジュールは判定ロジックのみを持ち、しきい値は RankCriteria で外部入力する。
use crate::holding_capacity::{ds_value, FrameType, MemberRank};
use sc_solver::pushover::MechanismType;

/// AIJ 規準のしきい値（外部入力）。
///
/// # 注意
/// フィールドの代表値はあくまで仮の値であり、原典照合が必要（要・原典照合リスト）。
/// - RC せん断余裕度 Qsu/Qmu の境界値: rc_ratio_fa, rc_ratio_fb, rc_ratio_fc
/// - S 最大幅厚比の上限: s_wt_fa, s_wt_fb, s_wt_fc
pub struct RankCriteria {
    /// RC: Qsu/Qmu の FA/FB 境界（要・原典照合）
    pub rc_ratio_fa: f64,
    /// RC: Qsu/Qmu の FB/FC 境界（要・原典照合）
    pub rc_ratio_fb: f64,
    /// RC: Qsu/Qmu の FC/FD 境界（要・原典照合）
    pub rc_ratio_fc: f64,
    /// S: FA の最大幅厚比上限（要・原典照合）
    pub s_wt_fa: f64,
    /// S: FB の最大幅厚比上限（要・原典照合）
    pub s_wt_fb: f64,
    /// S: FC の最大幅厚比上限（要・原典照合）
    pub s_wt_fc: f64,
}

impl Default for RankCriteria {
    /// 代表値（要・原典照合リスト）。
    fn default() -> Self {
        Self {
            rc_ratio_fa: 1.3, // 要・原典照合
            rc_ratio_fb: 1.1, // 要・原典照合
            rc_ratio_fc: 1.0, // 要・原典照合
            s_wt_fa: 9.0,    // 要・原典照合
            s_wt_fb: 11.0,   // 要・原典照合
            s_wt_fc: 13.0,   // 要・原典照合
        }
    }
}

/// ランクを 0(FA)..3(FD) の整数インデックスに変換する。
fn rank_index(r: MemberRank) -> u8 {
    match r {
        MemberRank::FA => 0,
        MemberRank::FB => 1,
        MemberRank::FC => 2,
        MemberRank::FD => 3,
    }
}

/// 整数インデックスをランクに変換する。インデックスが 3 を超える場合は FD を返す。
fn index_rank(i: u8) -> MemberRank {
    match i {
        0 => MemberRank::FA,
        1 => MemberRank::FB,
        2 => MemberRank::FC,
        _ => MemberRank::FD,
    }
}

/// RC 部材ランク判定。
///
/// `qsu`: せん断耐力、`qmu`: 曲げ耐力（qmu <= 0 なら FD を返す）。
/// せん断余裕度 r = Qsu/Qmu の大小でランクを決定（大きいほど靭性的＝良い）。
pub fn rc_member_rank(qsu: f64, qmu: f64, c: &RankCriteria) -> MemberRank {
    if qmu <= 0.0 {
        return MemberRank::FD;
    }
    let r = qsu / qmu;
    if r >= c.rc_ratio_fa {
        MemberRank::FA
    } else if r >= c.rc_ratio_fb {
        MemberRank::FB
    } else if r >= c.rc_ratio_fc {
        MemberRank::FC
    } else {
        MemberRank::FD
    }
}

/// S 部材ランク判定。
///
/// `max_width_thickness`: 最大幅厚比（小さいほど良い）。
pub fn s_member_rank(max_width_thickness: f64, c: &RankCriteria) -> MemberRank {
    let wt = max_width_thickness;
    if wt <= c.s_wt_fa {
        MemberRank::FA
    } else if wt <= c.s_wt_fb {
        MemberRank::FB
    } else if wt <= c.s_wt_fc {
        MemberRank::FC
    } else {
        MemberRank::FD
    }
}

/// 層 Ds 値を計算する。
///
/// # 規則
/// 1. 層の代表ランク = `ranks` 中で最も不利（FD 寄り）な部材ランク。
///    `ranks` が空の場合は FA を使用する。
/// 2. 崩壊機構補正:
///    - [`MechanismType::StoryCollapse`] または [`MechanismType::Partial`] の場合、
///      代表ランクを 1 段階不利側へ移動（FA→FB→FC→FD、FD は据え置き）。
///    - [`MechanismType::Overall`] は補正なし。
/// 3. 補正後のランクと `frame` を [`ds_value`] に渡して返す。
pub fn story_ds(ranks: &[MemberRank], frame: FrameType, mechanism: &MechanismType) -> f64 {
    // 代表ランク: ranks が空なら FA とみなす
    let worst_index = ranks
        .iter()
        .map(|r| rank_index(*r))
        .max()
        .unwrap_or(0);

    // 崩壊機構補正: StoryCollapse または Partial → 1段階不利
    let corrected_index = match mechanism {
        MechanismType::StoryCollapse { .. } | MechanismType::Partial => {
            (worst_index + 1).min(3)
        }
        MechanismType::Overall => worst_index,
    };

    let representative = index_rank(corrected_index);
    ds_value(frame, representative)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::ids::StoryId;

    // ===== rc_member_rank テスト =====

    #[test]
    fn test_rc_member_rank_fa() {
        let c = RankCriteria::default();
        assert_eq!(rc_member_rank(1.5 * 1.0, 1.0, &c), MemberRank::FA);
    }

    #[test]
    fn test_rc_member_rank_fb() {
        let c = RankCriteria::default();
        // r=1.2 → >= 1.1(FB境界) かつ < 1.3(FA境界) → FB
        assert_eq!(rc_member_rank(1.2, 1.0, &c), MemberRank::FB);
    }

    #[test]
    fn test_rc_member_rank_fc() {
        let c = RankCriteria::default();
        // r=1.05 → >= 1.0(FC境界) かつ < 1.1(FB境界) → FC
        assert_eq!(rc_member_rank(1.05, 1.0, &c), MemberRank::FC);
    }

    #[test]
    fn test_rc_member_rank_fd() {
        let c = RankCriteria::default();
        // r=0.9 → < 1.0(FC境界) → FD
        assert_eq!(rc_member_rank(0.9, 1.0, &c), MemberRank::FD);
    }

    #[test]
    fn test_rc_member_rank_zero_qmu() {
        let c = RankCriteria::default();
        // qmu=0 → FD
        assert_eq!(rc_member_rank(1.5, 0.0, &c), MemberRank::FD);
    }

    #[test]
    fn test_rc_member_rank_negative_qmu() {
        let c = RankCriteria::default();
        // qmu<0 → FD
        assert_eq!(rc_member_rank(1.5, -1.0, &c), MemberRank::FD);
    }

    // ===== s_member_rank テスト =====

    #[test]
    fn test_s_member_rank_fa() {
        let c = RankCriteria::default();
        // wt=8 <= 9(s_wt_fa) → FA
        assert_eq!(s_member_rank(8.0, &c), MemberRank::FA);
    }

    #[test]
    fn test_s_member_rank_fb() {
        let c = RankCriteria::default();
        // wt=10 → > 9 かつ <= 11 → FB
        assert_eq!(s_member_rank(10.0, &c), MemberRank::FB);
    }

    #[test]
    fn test_s_member_rank_fc() {
        let c = RankCriteria::default();
        // wt=12 → > 11 かつ <= 13 → FC
        assert_eq!(s_member_rank(12.0, &c), MemberRank::FC);
    }

    #[test]
    fn test_s_member_rank_fd() {
        let c = RankCriteria::default();
        // wt=15 → > 13 → FD
        assert_eq!(s_member_rank(15.0, &c), MemberRank::FD);
    }

    // ===== story_ds テスト =====

    /// ranks=[FA,FC,FB], RcFrame, Overall → 代表 FC → ds_value(RcFrame,FC) = 0.40
    #[test]
    fn test_story_ds_rc_frame_overall() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        assert!(
            (ds - 0.40).abs() < 1e-9,
            "expected 0.40, got {}",
            ds
        );
    }

    /// 同上で StoryCollapse → 代表 FC → FD → ds_value(RcFrame,FD) = 0.45
    #[test]
    fn test_story_ds_rc_frame_story_collapse() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse {
                story: StoryId(0),
            },
        );
        assert!(
            (ds - 0.45).abs() < 1e-9,
            "expected 0.45, got {}",
            ds
        );
    }

    /// ranks=[FA], SteelFrame, Overall → 代表 FA → ds_value(SteelFrame,FA) = 0.25
    #[test]
    fn test_story_ds_steel_frame_fa_overall() {
        let ranks = vec![MemberRank::FA];
        let ds = story_ds(&ranks, FrameType::SteelFrame, &MechanismType::Overall);
        assert!(
            (ds - 0.25).abs() < 1e-9,
            "expected 0.25, got {}",
            ds
        );
    }

    /// 空 ranks → FA 扱い → ds_value(RcFrame, FA) = 0.30
    #[test]
    fn test_story_ds_empty_ranks() {
        let ds = story_ds(&[], FrameType::RcFrame, &MechanismType::Overall);
        assert!(
            (ds - 0.30).abs() < 1e-9,
            "expected 0.30 for empty ranks, got {}",
            ds
        );
    }

    /// Partial でも1段階不利になる: [FA,FC,FB], RcFrame, Partial → FC → FD → 0.45
    #[test]
    fn test_story_ds_partial_downgrades_one_step() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Partial);
        assert!(
            (ds - 0.45).abs() < 1e-9,
            "expected 0.45, got {}",
            ds
        );
    }

    /// FD は据え置き（StoryCollapse でも FD → FD）
    #[test]
    fn test_story_ds_fd_stays_fd() {
        let ranks = vec![MemberRank::FD];
        let ds_overall = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        let ds_collapse = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse {
                story: StoryId(0),
            },
        );
        // FD は最悪なので補正後も FD のまま
        assert!(
            (ds_overall - 0.45).abs() < 1e-9,
            "FD Overall expected 0.45, got {}",
            ds_overall
        );
        assert!(
            (ds_collapse - 0.45).abs() < 1e-9,
            "FD StoryCollapse expected 0.45 (FD stays FD), got {}",
            ds_collapse
        );
    }
}
