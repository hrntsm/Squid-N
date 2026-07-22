//! 形鋼ライブラリ要素（`StbSecRoll-*` / `StbSecBuild-*` / `StbSecPipe`）からの断面形状復元。

use super::xml::get_f64_any;
use squid_n_core::section_shape::SectionShape;
use std::collections::HashMap;

/// 形鋼ライブラリ要素（`StbSecRoll-H` 等）と属性から [`SectionShape`] を復元する。
pub(super) fn steel_shape_from(tag: &str, a: &HashMap<String, String>) -> Option<SectionShape> {
    // 形鋼の寸法属性は A(せい/長辺)・B(幅/短辺)・t1(ウェブ)・t2(フランジ) を基本とする。
    let a_ = |keys: &[&str]| get_f64_any(a, keys).ok();
    match tag {
        t if t.ends_with("-H") => {
            let height = a_(&["A"])?;
            let web_thick = a_(&["t1"])?;
            let upper_width = a_(&["B"])?;
            let upper_thick = a_(&["t2"])?;
            // 下フランジの方言属性があれば非対称組立 H、無ければ対称 H。
            match (a_(&["B2", "B_lower"]), a_(&["t2_lower", "t2_2"])) {
                (Some(lower_width), Some(lower_thick)) => Some(SectionShape::SteelBuiltH {
                    height,
                    upper_width,
                    upper_thick,
                    lower_width,
                    lower_thick,
                    web_thick,
                }),
                _ => Some(SectionShape::SteelH {
                    height,
                    width: upper_width,
                    web_thick,
                    flange_thick: upper_thick,
                }),
            }
        }
        t if t.ends_with("-BOX") => {
            let thick = a_(&["t", "t1"])?;
            // 角部外半径 r（StbSecRoll-BOX の r 属性）。未指定は角部直角（0.0）。
            let corner_r = a_(&["r"]).unwrap_or(0.0);
            Some(SectionShape::SteelBox {
                height: a_(&["A"])?,
                width: a_(&["B"])?,
                thick,
                corner_r,
            })
        }
        // 鋼管。Squid 方言の `StbSecPipe` に加え、実 ST-Bridge の形鋼ライブラリ名
        // （`StbSecRoll-Pipe`／冷間成形の `StbSecBuild-Pipe`）も受ける。いずれも外径 D・
        // 板厚 t を持つ（別名 A/t1 も許容）。これが無いと他社ファイルの鋼管柱・梁の
        // 形鋼参照が解決できず、物性ゼロの断面になってしまう。
        t if t == "StbSecPipe" || t.ends_with("-Pipe") => Some(SectionShape::SteelPipe {
            outer_dia: a_(&["D", "A"])?,
            thick: a_(&["t", "t1"])?,
        }),
        t if t.ends_with("-L") => Some(SectionShape::SteelAngle {
            leg_a: a_(&["A"])?,
            leg_b: a_(&["B"])?,
            thick: a_(&["t1", "t"])?,
        }),
        t if t.ends_with("-C") => Some(SectionShape::SteelChannel {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        t if t.ends_with("-T") => Some(SectionShape::SteelTee {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        // 平鋼・鋼板（中実矩形）。幅 B・板厚 t。
        t if t.ends_with("-FlatBar") => Some(SectionShape::SteelFlatBar {
            width: a_(&["B", "A", "width"])?,
            thick: a_(&["t", "t1"])?,
        }),
        // 中実丸鋼。直径 D（半径 R のみの場合は 2R）。
        t if t.ends_with("-RoundBar") => {
            let dia = a_(&["D", "A"]).or_else(|| a_(&["R"]).map(|r| r * 2.0))?;
            Some(SectionShape::SteelRoundBar { dia })
        }
        // リップ溝形鋼（冷間成形）。せい A・幅 B・リップ C・板厚 t。
        t if t.ends_with("-LipC") => Some(SectionShape::SteelLipChannel {
            height: a_(&["A", "H"])?,
            width: a_(&["B"])?,
            lip: a_(&["C"])?,
            thick: a_(&["t", "t1"])?,
        }),
        _ => None,
    }
}
