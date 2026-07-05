//! T4: 部材ランク（FA..FD）と層 Ds の自動分類。仕様 specs/P7_二次設計.md §7。
//! しきい値表（RC 耐力比境界・S 幅厚比限界）は AIJ 規準（Category B）の外部データ。
//! 本モジュールは判定ロジックのみを持ち、しきい値は RankCriteria で外部入力する。
use crate::holding_capacity::{ds_value, FrameType, MemberRank};
use squid_n_solver::pushover::MechanismType;

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
            s_wt_fa: 9.0,     // 要・原典照合
            s_wt_fb: 11.0,    // 要・原典照合
            s_wt_fc: 13.0,    // 要・原典照合
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

/// S 部材ランク判定（F 値スケーリング付き）。
///
/// `max_width_thickness`: 最大幅厚比（小さいほど良い）。
/// `f_value`: 鋼材の F 値 [N/mm²]（0 以下は 235 とみなす）。
///
/// 幅厚比境界は F=235 基準の代表値（`RankCriteria`）を √(235/F) 倍して用いる
/// （鋼構造規準の幅厚比区分（FA〜FD の限界幅厚比）が√(235/F)に比例して定められる
/// 規定に倣う簡易スケーリング。要・原典照合）。F=235 のときは `s_member_rank` と一致する。
pub fn s_member_rank_scaled(
    max_width_thickness: f64,
    f_value: f64,
    c: &RankCriteria,
) -> MemberRank {
    let f = if f_value <= 0.0 { 235.0 } else { f_value };
    let scale = (235.0 / f).sqrt();
    let wt = max_width_thickness;
    if wt <= c.s_wt_fa * scale {
        MemberRank::FA
    } else if wt <= c.s_wt_fb * scale {
        MemberRank::FB
    } else if wt <= c.s_wt_fc * scale {
        MemberRank::FC
    } else {
        MemberRank::FD
    }
}

/// 鋼断面の代表最大幅厚比を形状寸法から算定する（UI-13、specs/UI設計.md §9.3）。
///
/// # 採用式（要・原典照合。簡易法であり AIJ 精算式そのものではない）
/// - H形: フランジ片持ち部 `b/(2·tf)`（半幅/板厚）とウェブ内法 `(h-2·tf)/tw` の大きい方。
/// - 箱形: 内法平板幅を板厚で割った値 `(h-2t)/t`, `(b-2t)/t` の大きい方（4辺同厚前提）。
/// - 溝形: H形に準じるが、フランジは片側のみが自由端の片持ち版のため全幅がそのまま
///   張出し長さとなる（半幅ではない）→ `b/tf`。ウェブは上下フランジに挟まれる点は
///   H形と同じなので `(h-2·tf)/tw`。
/// - T形: フランジは片側（上端）のみの片持ち版 → `b/tf`。ウェブは上端のフランジのみを
///   差し引いた `(h-tf)/tw`（下端は自由端のため 2 枚分は引かない）。
/// - 山形: 単板が直交する形状のため `max(leg_a, leg_b)/thick`。
/// - 円形鋼管: 径厚比 `D/t` は幅厚比と規準体系（座屈モード）が異なるため対象外（`None`）。
/// - RC 断面: 幅厚比の概念がないため `None`。
///
/// 板厚が 0 以下、または板要素の内法寸法が 0 未満になる不正な寸法の場合は `None` を返す。
pub fn max_width_thickness(shape: &squid_n_core::section_shape::SectionShape) -> Option<f64> {
    use squid_n_core::section_shape::SectionShape;

    /// 板厚が正で内法寸法が非負なら比を返す。不正な寸法は None。
    fn ratio(clear: f64, thick: f64) -> Option<f64> {
        if thick <= 0.0 || clear < 0.0 {
            None
        } else {
            Some(clear / thick)
        }
    }

    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, 2.0 * flange_thick)?;
            let web = ratio(height - 2.0 * flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
        } => {
            let hi = ratio(height - 2.0 * thick, thick)?;
            let wi = ratio(width - 2.0 * thick, thick)?;
            Some(hi.max(wi))
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, flange_thick)?;
            let web = ratio(height - 2.0 * flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, flange_thick)?;
            let web = ratio(height - flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => ratio(leg_a.max(leg_b), thick),
        SectionShape::SteelPipe { .. } => None,
        SectionShape::RcRect { .. } | SectionShape::RcCircle { .. } => None,
    }
}

/// 複数の部材ランクのうち最も不利（FD 寄り）なものを返す。`ranks` が空なら `None`。
///
/// 保有水平耐力（ルート3）の層ランク自動判定（UI-13）で、1 層に属する複数の
/// 鋼部材ランクから層の代表ランクを選ぶために使う。
pub fn worst_rank(ranks: &[MemberRank]) -> Option<MemberRank> {
    ranks.iter().map(|r| rank_index(*r)).max().map(index_rank)
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
    let worst_index = ranks.iter().map(|r| rank_index(*r)).max().unwrap_or(0);

    // 崩壊機構補正: StoryCollapse または Partial → 1段階不利
    let corrected_index = match mechanism {
        MechanismType::StoryCollapse { .. } | MechanismType::Partial => (worst_index + 1).min(3),
        MechanismType::Overall => worst_index,
    };

    let representative = index_rank(corrected_index);
    ds_value(frame, representative)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::StoryId;

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

    // ===== s_member_rank_scaled テスト =====

    /// F=235（基準値）では scale=1.0 なので `s_member_rank` と完全に一致する。
    #[test]
    fn test_s_member_rank_scaled_matches_unscaled_at_f235() {
        let c = RankCriteria::default();
        for wt in [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 15.0] {
            assert_eq!(
                s_member_rank_scaled(wt, 235.0, &c),
                s_member_rank(wt, &c),
                "wt={} で不一致",
                wt
            );
        }
    }

    /// f_value<=0 は 235 とみなす（F=0 と F=235 が一致）。
    #[test]
    fn test_s_member_rank_scaled_nonpositive_f_defaults_to_235() {
        let c = RankCriteria::default();
        assert_eq!(
            s_member_rank_scaled(10.0, 0.0, &c),
            s_member_rank_scaled(10.0, 235.0, &c)
        );
        assert_eq!(
            s_member_rank_scaled(10.0, -100.0, &c),
            s_member_rank_scaled(10.0, 235.0, &c)
        );
    }

    /// F=325（SN490 相当）では境界が √(235/325)≈0.850340 倍に厳しくなる。
    /// wt=8.0 は F=235 では FA(<=9.0) だが、F=325 では
    /// FA境界=9.0*0.850340=7.653<8.0、FB境界=11.0*0.850340=9.354>=8.0 → FB。
    #[test]
    fn test_s_member_rank_scaled_f325_boundary_tightens() {
        let c = RankCriteria::default();
        assert_eq!(s_member_rank_scaled(8.0, 235.0, &c), MemberRank::FA);
        assert_eq!(s_member_rank_scaled(8.0, 325.0, &c), MemberRank::FB);
    }

    // ===== worst_rank テスト =====

    #[test]
    fn test_worst_rank_picks_fd_leaning() {
        let ranks = [MemberRank::FA, MemberRank::FC, MemberRank::FB];
        assert_eq!(worst_rank(&ranks), Some(MemberRank::FC));
    }

    #[test]
    fn test_worst_rank_empty_is_none() {
        assert_eq!(worst_rank(&[]), None);
    }

    // ===== max_width_thickness テスト =====

    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    fn dummy_rebar() -> RcRebar {
        RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 16.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 16.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }

    /// H-300x200x10x16: flange=200/(2*16)=6.25, web=(300-32)/10=26.8 → max=26.8
    #[test]
    fn test_max_width_thickness_steel_h() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 16.0,
        };
        let wt = max_width_thickness(&shape).unwrap();
        assert!((wt - 26.8).abs() < 1e-9, "expected 26.8, got {}", wt);
    }

    /// BOX-200x150x9: hi=(200-18)/9=20.2222, wi=(150-18)/9=14.6667 → max=20.2222
    #[test]
    fn test_max_width_thickness_steel_box() {
        let shape = SectionShape::SteelBox {
            height: 200.0,
            width: 150.0,
            thick: 9.0,
        };
        let wt = max_width_thickness(&shape).unwrap();
        assert!(
            (wt - 182.0 / 9.0).abs() < 1e-9,
            "expected {}, got {}",
            182.0 / 9.0,
            wt
        );
    }

    /// C-200x90x8x12: flange=90/12=7.5, web=(200-24)/8=22.0 → max=22.0
    #[test]
    fn test_max_width_thickness_steel_channel() {
        let shape = SectionShape::SteelChannel {
            height: 200.0,
            width: 90.0,
            web_thick: 8.0,
            flange_thick: 12.0,
        };
        let wt = max_width_thickness(&shape).unwrap();
        assert!((wt - 22.0).abs() < 1e-9, "expected 22.0, got {}", wt);
    }

    /// T-200x200x10x15: flange=200/15=13.333, web=(200-15)/10=18.5 → max=18.5
    #[test]
    fn test_max_width_thickness_steel_tee() {
        let shape = SectionShape::SteelTee {
            height: 200.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let wt = max_width_thickness(&shape).unwrap();
        assert!((wt - 18.5).abs() < 1e-9, "expected 18.5, got {}", wt);
    }

    /// L-150x100x12: max(150,100)/12=12.5
    #[test]
    fn test_max_width_thickness_steel_angle() {
        let shape = SectionShape::SteelAngle {
            leg_a: 150.0,
            leg_b: 100.0,
            thick: 12.0,
        };
        let wt = max_width_thickness(&shape).unwrap();
        assert!((wt - 12.5).abs() < 1e-9, "expected 12.5, got {}", wt);
    }

    /// 円形鋼管: 径厚比は規準体系が異なるため対象外 → None
    #[test]
    fn test_max_width_thickness_steel_pipe_is_none() {
        let shape = SectionShape::SteelPipe {
            outer_dia: 216.3,
            thick: 8.2,
        };
        assert!(max_width_thickness(&shape).is_none());
    }

    /// RC 断面は幅厚比の概念がないため None
    #[test]
    fn test_max_width_thickness_rc_is_none() {
        let rect = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: dummy_rebar(),
        };
        assert!(max_width_thickness(&rect).is_none());
        let circle = SectionShape::RcCircle {
            d: 600.0,
            rebar: dummy_rebar(),
        };
        assert!(max_width_thickness(&circle).is_none());
    }

    /// 板厚 0 は不正 → None
    #[test]
    fn test_max_width_thickness_zero_thickness_is_none() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 200.0,
            web_thick: 0.0,
            flange_thick: 16.0,
        };
        assert!(max_width_thickness(&shape).is_none());
    }

    /// 板厚が負は不正 → None
    #[test]
    fn test_max_width_thickness_negative_thickness_is_none() {
        let shape = SectionShape::SteelBox {
            height: 200.0,
            width: 150.0,
            thick: -9.0,
        };
        assert!(max_width_thickness(&shape).is_none());
    }

    // ===== story_ds テスト =====

    /// ranks=[FA,FC,FB], RcFrame, Overall → 代表 FC → ds_value(RcFrame,FC) = 0.40
    #[test]
    fn test_story_ds_rc_frame_overall() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        assert!((ds - 0.40).abs() < 1e-9, "expected 0.40, got {}", ds);
    }

    /// 同上で StoryCollapse → 代表 FC → FD → ds_value(RcFrame,FD) = 0.45
    #[test]
    fn test_story_ds_rc_frame_story_collapse() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse { story: StoryId(0) },
        );
        assert!((ds - 0.45).abs() < 1e-9, "expected 0.45, got {}", ds);
    }

    /// ranks=[FA], SteelFrame, Overall → 代表 FA → ds_value(SteelFrame,FA) = 0.25
    #[test]
    fn test_story_ds_steel_frame_fa_overall() {
        let ranks = vec![MemberRank::FA];
        let ds = story_ds(&ranks, FrameType::SteelFrame, &MechanismType::Overall);
        assert!((ds - 0.25).abs() < 1e-9, "expected 0.25, got {}", ds);
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
        assert!((ds - 0.45).abs() < 1e-9, "expected 0.45, got {}", ds);
    }

    /// FD は据え置き（StoryCollapse でも FD → FD）
    #[test]
    fn test_story_ds_fd_stays_fd() {
        let ranks = vec![MemberRank::FD];
        let ds_overall = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        let ds_collapse = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse { story: StoryId(0) },
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
