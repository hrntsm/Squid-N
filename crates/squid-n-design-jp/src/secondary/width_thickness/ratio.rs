//! 鋼断面の代表最大幅厚比の算定（形状寸法からの簡易法）。

use squid_n_core::section_shape::SectionShape;

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
pub fn max_width_thickness(shape: &SectionShape) -> Option<f64> {
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
        // CFT 角形: 鋼管部分の幅厚比（充填効果による緩和は未考慮＝安全側）。
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            let hi = ratio(height - 2.0 * thick, thick)?;
            let wi = ratio(width - 2.0 * thick, thick)?;
            Some(hi.max(wi))
        }
        SectionShape::CftPipe { .. } => None,
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::SrcRect { .. }
        | SectionShape::RcWall { .. } => None,
    }
}
