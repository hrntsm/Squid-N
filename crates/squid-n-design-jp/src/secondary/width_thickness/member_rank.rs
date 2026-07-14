//! 構造規定の幅厚比表による S 部材ランク判定（昭55建告1792号・技術基準解説書の
//! 「幅厚比の検討（部材ランク）」表。2007年版建築物の構造関係技術基準解説書＝
//! 構造規定の表に対応）。

use crate::secondary::holding_capacity::MemberRank;
use crate::secondary::member_rank::worst_rank;
use squid_n_core::section_shape::SectionShape;

/// 部材の用途（幅厚比ランク表の行の選択）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SteelMemberUse {
    /// 柱
    Column,
    /// 梁
    Beam,
}

/// FA/FB/FC の幅厚比限界値（超えると FD）。
#[derive(Clone, Copy, Debug)]
struct WtLimits {
    fa: f64,
    fb: f64,
    fc: f64,
}

/// 幅厚比 `wt` を [`WtLimits`] と比較してランクを返す。
/// `wt <= fa` → FA、`<= fb` → FB、`<= fc` → FC、それ以外 → FD。
fn rank_from_limits(wt: f64, limits: &WtLimits) -> MemberRank {
    if wt <= limits.fa {
        MemberRank::FA
    } else if wt <= limits.fb {
        MemberRank::FB
    } else if wt <= limits.fc {
        MemberRank::FC
    } else {
        MemberRank::FD
    }
}

/// H形フランジの幅厚比限界（部材用途・鋼種級ごと）。
fn h_flange_limits(member_use: SteelMemberUse, is_490: bool) -> WtLimits {
    match (member_use, is_490) {
        (SteelMemberUse::Column, false) => WtLimits {
            fa: 9.5,
            fb: 12.0,
            fc: 15.5,
        },
        (SteelMemberUse::Column, true) => WtLimits {
            fa: 8.0,
            fb: 10.0,
            fc: 13.2,
        },
        (SteelMemberUse::Beam, false) => WtLimits {
            fa: 9.0,
            fb: 11.0,
            fc: 15.5,
        },
        (SteelMemberUse::Beam, true) => WtLimits {
            fa: 7.5,
            fb: 9.5,
            fc: 13.2,
        },
    }
}

/// H形ウェブの幅厚比限界（部材用途・鋼種級ごと）。
fn h_web_limits(member_use: SteelMemberUse, is_490: bool) -> WtLimits {
    match (member_use, is_490) {
        (SteelMemberUse::Column, false) => WtLimits {
            fa: 43.0,
            fb: 45.0,
            fc: 48.0,
        },
        (SteelMemberUse::Column, true) => WtLimits {
            fa: 37.0,
            fb: 39.0,
            fc: 41.0,
        },
        (SteelMemberUse::Beam, false) => WtLimits {
            fa: 60.0,
            fb: 65.0,
            fc: 71.0,
        },
        (SteelMemberUse::Beam, true) => WtLimits {
            fa: 51.0,
            fb: 55.0,
            fc: 61.0,
        },
    }
}

/// 円形鋼管（径厚比 D/t）の幅厚比限界。
///
/// 構造規定表には柱の行のみが定義されている。梁の円形鋼管は同表に
/// 独立の行が無いため、柱の行を準用する（呼び出し側では `member_use` を
/// 見ずに本関数を呼ぶ）。
fn pipe_limits(is_490: bool) -> WtLimits {
    if is_490 {
        WtLimits {
            fa: 36.0,
            fb: 50.0,
            fc: 73.0,
        }
    } else {
        WtLimits {
            fa: 50.0,
            fb: 70.0,
            fc: 100.0,
        }
    }
}

/// 角形鋼管の鋼種区分（材料名の前方一致で判定する専用行）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BoxGrade {
    Stkr400,
    Stkr490,
    Bcr295,
    Bcp235,
    Bcp325,
}

/// 角形鋼管の専用グレード名（前方一致）。複数一致しうる場合は最長一致を採用する
/// （[`crate::steel::steel_f_value_prefix`] と同様の方針。実際にはこれらの記号に
/// 接頭辞の重複はない）。
const BOX_GRADE_NAMES: &[(&str, BoxGrade)] = &[
    ("STKR400", BoxGrade::Stkr400),
    ("STKR490", BoxGrade::Stkr490),
    ("BCR295", BoxGrade::Bcr295),
    ("BCP235", BoxGrade::Bcp235),
    ("BCP325", BoxGrade::Bcp325),
];

fn box_grade_from_name(name: &str) -> Option<BoxGrade> {
    BOX_GRADE_NAMES
        .iter()
        .filter(|(prefix, _)| name.starts_with(prefix))
        .max_by_key(|(prefix, _)| prefix.len())
        .map(|(_, g)| *g)
}

/// 角形鋼管（角形鋼管・BOX）の幅厚比限界（d=H、部位は柱の行のみ。梁の角形鋼管も
/// 構造規定表に独立の行が無いため柱の行を準用する）。
fn box_limits(g: BoxGrade) -> WtLimits {
    match g {
        BoxGrade::Stkr400 => WtLimits {
            fa: 33.0,
            fb: 37.0,
            fc: 48.0,
        },
        BoxGrade::Stkr490 => WtLimits {
            fa: 27.0,
            fb: 32.0,
            fc: 41.0,
        },
        BoxGrade::Bcr295 => WtLimits {
            fa: 30.0,
            fb: 34.0,
            fc: 43.0,
        },
        BoxGrade::Bcp235 => WtLimits {
            fa: 33.0,
            fb: 37.0,
            fc: 48.0,
        },
        BoxGrade::Bcp325 => WtLimits {
            fa: 27.0,
            fb: 32.0,
            fc: 41.0,
        },
    }
}

/// 角形鋼管の限界値を鋼種名から解決する。`grade_name` が
/// STKR400/STKR490/BCR295/BCP235/BCP325 のいずれにも前方一致しない場合は、
/// F 値ベースの鋼種級判定（[`is_490_class`]）により STKR400/STKR490 の限界値へ
/// フォールバックする。
fn box_limits_for(grade_name: &str, thickness: f64) -> WtLimits {
    match box_grade_from_name(grade_name) {
        Some(g) => box_limits(g),
        None => {
            if is_490_class(grade_name, thickness) {
                box_limits(BoxGrade::Stkr490)
            } else {
                box_limits(BoxGrade::Stkr400)
            }
        }
    }
}

/// 鋼種級（400N/mm²級 or 490N/mm²級）を判定する。
///
/// 鋼種級は引張強さによる区分であり、まず鋼種名の数値部で判定する
/// （…490/520 系 → 490 級、…400 系 → 400 級。SS490 は F=275 だが
/// 引張強さ 490N/mm² 級であり、F 値のみの判定では 400 級に誤分類され
/// 限界幅厚比が緩くなる非保守側の誤りとなる）。名称で判定できない場合は
/// [`crate::steel::steel_f_value_prefix`] を板厚で呼び F≧295 なら 490 級と
/// するフォールバックを用い、それも解決できなければ限界幅厚比がより厳しい
/// （小さい）490 級を安全側として採用する。
fn is_490_class(grade_name: &str, thickness: f64) -> bool {
    let name = grade_name.trim().to_ascii_uppercase();
    // 名称中の3桁数値（引張強さ表記）による判定を優先する。
    for (needle, is_490) in [("490", true), ("520", true), ("550", true), ("400", false)] {
        if name.contains(needle) {
            return is_490;
        }
    }
    match crate::steel::steel_f_value_prefix(grade_name, thickness) {
        Some(f) => f >= 295.0,
        None => true,
    }
}

/// 構造規定の幅厚比表による S 部材ランク判定（昭55建告1792号・技術基準解説書の
/// 「幅厚比の検討（部材ランク）」表）。
///
/// # 対象形状
/// - `SteelH`: フランジ `b/t2`（`b`=B/2、フランジ半幅／フランジ厚）と
///   ウェブ `d/t1`（`d`=H−2·t2、内法せい／ウェブ厚）をそれぞれ判定し、
///   悪い方（FA<FB<FC<FD の順で不利な方）を採用する。
/// - `SteelBox`（`CftBox` は鋼管部分として同様に扱う）: `d/t`（`d`=H、全せい。
///   内法寸法ではない点に注意）。
/// - `SteelPipe`（`CftPipe` は鋼管部分として同様に扱う）: `d/t`（`d`=D、外径）。
/// - 上記以外（RC・溝形・T形・山形等）は `None`。
///
/// # 部材用途と表の行
/// `member_use` で柱／梁の行を選ぶ（H形はフランジ・ウェブとも柱・梁で異なる
/// 限界値を持つ）。角形鋼管・円形鋼管は構造規定表に梁の行が無いため、
/// `member_use` によらず柱の行を準用する。
///
/// # 鋼種の判定
/// `grade_name`（例 "SN400B", "SM490A", "BCR295"）は前方一致で判定する。
/// - `BCR295`/`BCP235`/`BCP325`/`STKR400`/`STKR490` は角形鋼管専用の行。
/// - それ以外は [`crate::steel::steel_f_value_prefix`] の F 値により
///   400N/mm²級／490N/mm²級を判定する（詳細は [`is_490_class`]）。
///
/// # 戻り値
/// 対象外の形状、または板厚が 0 以下・内法寸法が負になる不正な寸法の場合は
/// `None`。各板要素の幅厚比が FC 限界も超える場合は `FD`。
pub fn s_member_rank_by_kihon(
    shape: &SectionShape,
    member_use: SteelMemberUse,
    grade_name: &str,
) -> Option<MemberRank> {
    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            if flange_thick <= 0.0 || web_thick <= 0.0 {
                return None;
            }
            let web_clear = height - 2.0 * flange_thick;
            if web_clear < 0.0 {
                return None;
            }
            let flange_wt = (width / 2.0) / flange_thick;
            let web_wt = web_clear / web_thick;

            let flange_is_490 = is_490_class(grade_name, flange_thick);
            let web_is_490 = is_490_class(grade_name, web_thick);
            let flange_rank =
                rank_from_limits(flange_wt, &h_flange_limits(member_use, flange_is_490));
            let web_rank = rank_from_limits(web_wt, &h_web_limits(member_use, web_is_490));
            worst_rank(&[flange_rank, web_rank])
        }
        SectionShape::SteelBox { height, thick, .. }
        | SectionShape::CftBox { height, thick, .. } => {
            if thick <= 0.0 {
                return None;
            }
            let wt = height / thick;
            Some(rank_from_limits(wt, &box_limits_for(grade_name, thick)))
        }
        SectionShape::SteelPipe { outer_dia, thick }
        | SectionShape::CftPipe { outer_dia, thick } => {
            if thick <= 0.0 {
                return None;
            }
            let wt = outer_dia / thick;
            let is_490 = is_490_class(grade_name, thick);
            Some(rank_from_limits(wt, &pipe_limits(is_490)))
        }
        SectionShape::SteelChannel { .. }
        | SectionShape::SteelTee { .. }
        | SectionShape::SteelAngle { .. }
        | SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::SrcRect { .. }
        | SectionShape::RcWall { .. } => None,
    }
}
