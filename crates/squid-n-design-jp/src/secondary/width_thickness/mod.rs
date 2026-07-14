//! 鉄骨部材の幅厚比による部材ランク判定（昭55建告1792号・技術基準解説書の
//! 「幅厚比の検討（部材ランク）」表）。2007年版建築物の構造関係技術基準解説書＝
//! 構造規定の表に対応する。一次資料: `crate::secondary::member_rank`（後方互換の
//! 簡易判定）に対し、本モジュールは構造規定の表そのものを実装した正式版。
//!
//! # モジュール構成
//! - [`ratio`] — 鋼断面の代表最大幅厚比の算定（[`max_width_thickness`]）。
//! - [`member_rank`] — 構造規定の幅厚比表による S 部材ランク判定（[`s_member_rank_by_kihon`]）。

mod member_rank;
mod ratio;

pub use member_rank::{s_member_rank_by_kihon, SteelMemberUse};
pub use ratio::max_width_thickness;

// tests は `use super::*` で MemberRank を参照する。抽出により mod.rs 本体では
// 未使用となるため、テストビルドでのみ再エクスポートして super::* からの解決を維持する。
#[cfg(test)]
use super::holding_capacity::MemberRank;

#[cfg(test)]
mod tests {
    use super::*;

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
                grade: None,
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

    // ===== s_member_rank_by_kihon テスト =====

    /// 柱 H形フランジのみが効く形状を作る（ウェブは常に FA になるよう十分厚くする）。
    /// flange_wt = width / (2*flange_thick)。web_wt = (height-2*flange_thick)/web_thick
    /// は height=220, flange_thick=10, web_thick=60 で 200/60≈3.33（常に FA）。
    fn steel_h_flange_only(width: f64) -> SectionShape {
        SectionShape::SteelH {
            height: 220.0,
            width,
            web_thick: 60.0,
            flange_thick: 10.0,
        }
    }

    /// 柱 H形 400級 フランジ: b/t=9.5（境界） → FA。
    #[test]
    fn test_s_member_rank_by_kihon_column_h_flange_fa_boundary() {
        // width/(2*10)=9.5 -> width=190
        let shape = steel_h_flange_only(190.0);
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        assert_eq!(rank, MemberRank::FA);
    }

    /// 柱 H形 400級 フランジ: b/t=9.6（FA境界超え） → FB。
    #[test]
    fn test_s_member_rank_by_kihon_column_h_flange_fb() {
        // width/(2*10)=9.6 -> width=192
        let shape = steel_h_flange_only(192.0);
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        assert_eq!(rank, MemberRank::FB);
    }

    /// 柱 H形 400級 フランジ: b/t=15.5（FC境界） → FC。
    #[test]
    fn test_s_member_rank_by_kihon_column_h_flange_fc_boundary() {
        // width/(2*10)=15.5 -> width=310
        let shape = steel_h_flange_only(310.0);
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        assert_eq!(rank, MemberRank::FC);
    }

    /// 柱 H形 400級 フランジ: b/t=15.6（FC境界超え） → FD。
    #[test]
    fn test_s_member_rank_by_kihon_column_h_flange_fd() {
        // width/(2*10)=15.6 -> width=312
        let shape = steel_h_flange_only(312.0);
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        assert_eq!(rank, MemberRank::FD);
    }

    /// フランジは FA だがウェブが FC → 悪い方の FC が採用される（worst 合成）。
    /// かつ、構造規定表の取り方（フランジ b/(2*t2)、ウェブ (H-2*t2)/t1）を
    /// 内部で使っていることの検証も兼ねる:
    /// SteelH { height:400, width:200, web_thick:8, flange_thick:13 } では
    /// flange_wt = (200/2)/13 ≈ 7.69（柱400級 FA=9.5 以下 → FA）、
    /// web_wt = (400-2*13)/8 = 46.75（柱400級 FB=45 < 46.75 <= FC=48 → FC）。
    /// もし誤って d=H（全せい、内法を取らない）を使うと web_wt=400/8=50 となり
    /// FD になってしまうため、この境界値は式の取り方を検証できる。
    #[test]
    fn test_s_member_rank_by_kihon_h_worst_of_flange_and_web() {
        let shape = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        assert_eq!(rank, MemberRank::FC);
    }

    /// BCR295 角形鋼管（d=H, 全せい）: 幅厚比 30（境界） → FA。
    #[test]
    fn test_s_member_rank_by_kihon_bcr295_box_fa_boundary() {
        // height/thick = 300/10 = 30
        let shape = SectionShape::SteelBox {
            height: 300.0,
            width: 300.0,
            thick: 10.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "BCR295").expect("Some");
        assert_eq!(rank, MemberRank::FA);
    }

    /// BCR295 角形鋼管: 幅厚比 43（FC境界） → FC。
    #[test]
    fn test_s_member_rank_by_kihon_bcr295_box_fc_boundary() {
        // height/thick = 430/10 = 43
        let shape = SectionShape::SteelBox {
            height: 430.0,
            width: 430.0,
            thick: 10.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "BCR295").expect("Some");
        assert_eq!(rank, MemberRank::FC);
    }

    /// BCR295 角形鋼管: 幅厚比 43.1（FC境界超え） → FD。
    #[test]
    fn test_s_member_rank_by_kihon_bcr295_box_fd() {
        // height/thick = 431/10 = 43.1
        let shape = SectionShape::SteelBox {
            height: 431.0,
            width: 431.0,
            thick: 10.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "BCR295").expect("Some");
        assert_eq!(rank, MemberRank::FD);
    }

    /// 角形鋼管は d=H（全せい）を使う。内法（H-2t）ではないことを確認する。
    /// H=400, t=12, STKR400（FA=33, FB=37, FC=48） → wt=400/12≈33.33 → FB。
    /// もし誤って内法 (H-2t)/t=(400-24)/12≈31.33 を使うと FA になってしまうため、
    /// この境界値は d=H（全せい）を使っていることを検証できる。
    #[test]
    fn test_s_member_rank_by_kihon_box_uses_full_height_not_clear() {
        let shape = SectionShape::SteelBox {
            height: 400.0,
            width: 1000.0, // 幅は d の算定に使わないことを示すため、あえて全く違う値にする
            thick: 12.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "STKR400").expect("Some");
        assert_eq!(rank, MemberRank::FB);
    }

    /// 円形鋼管 490級: 径厚比 36（境界） → FA。
    #[test]
    fn test_s_member_rank_by_kihon_pipe_490_fa_boundary() {
        // outer_dia/thick = 360/10 = 36
        let shape = SectionShape::SteelPipe {
            outer_dia: 360.0,
            thick: 10.0,
        };
        // SM490A: F=325 (>=295) -> 490級
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SM490A").expect("Some");
        assert_eq!(rank, MemberRank::FA);
    }

    /// 円形鋼管 490級: 径厚比 73（FC境界） → FC。
    #[test]
    fn test_s_member_rank_by_kihon_pipe_490_fc_boundary() {
        // outer_dia/thick = 730/10 = 73
        let shape = SectionShape::SteelPipe {
            outer_dia: 730.0,
            thick: 10.0,
        };
        let rank = s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SM490A").expect("Some");
        assert_eq!(rank, MemberRank::FC);
    }

    /// 梁の円形鋼管・角形鋼管は柱の行を準用する（構造規定表に梁の行が無いため）。
    #[test]
    fn test_s_member_rank_by_kihon_beam_pipe_and_box_use_column_row() {
        let pipe = SectionShape::SteelPipe {
            outer_dia: 360.0,
            thick: 10.0,
        };
        let column_pipe_rank =
            s_member_rank_by_kihon(&pipe, SteelMemberUse::Column, "SM490A").unwrap();
        let beam_pipe_rank = s_member_rank_by_kihon(&pipe, SteelMemberUse::Beam, "SM490A").unwrap();
        assert_eq!(column_pipe_rank, beam_pipe_rank);

        let box_shape = SectionShape::SteelBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let column_box_rank =
            s_member_rank_by_kihon(&box_shape, SteelMemberUse::Column, "STKR400").unwrap();
        let beam_box_rank =
            s_member_rank_by_kihon(&box_shape, SteelMemberUse::Beam, "STKR400").unwrap();
        assert_eq!(column_box_rank, beam_box_rank);
    }

    /// CftBox・CftPipe も鋼管部分として同様に扱われる（形状違いのみ）。
    #[test]
    fn test_s_member_rank_by_kihon_cft_treated_as_steel() {
        let steel_box = SectionShape::SteelBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let cft_box = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        assert_eq!(
            s_member_rank_by_kihon(&steel_box, SteelMemberUse::Column, "STKR400"),
            s_member_rank_by_kihon(&cft_box, SteelMemberUse::Column, "STKR400")
        );
    }

    /// 未知の鋼種名は F 値が解決できないため 490級（安全側）にフォールバックする。
    /// H形フランジで b/t=9.5 は 400級なら FA だが 490級だと FB になる境界値を使う。
    #[test]
    fn test_s_member_rank_by_kihon_unresolvable_grade_falls_back_to_490() {
        let shape = steel_h_flange_only(190.0); // flange_wt=9.5
        let rank_unknown =
            s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "UNKNOWN999").expect("Some");
        let rank_400 =
            s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SN400B").expect("Some");
        let rank_490 =
            s_member_rank_by_kihon(&shape, SteelMemberUse::Column, "SM490A").expect("Some");
        assert_eq!(rank_unknown, rank_490);
        assert_ne!(rank_unknown, rank_400);
    }

    /// RC・角形以外の対象外形状は None。
    #[test]
    fn test_s_member_rank_by_kihon_unsupported_shape_is_none() {
        let rc = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: dummy_rebar(),
        };
        assert!(s_member_rank_by_kihon(&rc, SteelMemberUse::Column, "SN400B").is_none());

        let angle = SectionShape::SteelAngle {
            leg_a: 150.0,
            leg_b: 100.0,
            thick: 12.0,
        };
        assert!(s_member_rank_by_kihon(&angle, SteelMemberUse::Column, "SN400B").is_none());
    }
}
