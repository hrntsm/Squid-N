//! ST-Bridge の配筋（`StbSecBarArrangement*`）属性の解析（best-effort）。

use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
use std::collections::HashMap;

/// ST-Bridge 標準断面（幾何のみ）から復元する RC 断面の既定配筋（無筋相当）。
/// 弾性断面性能は b・d のみで決まり配筋に依存しないため、往復での剛性は保たれる。
/// 配筋検定を要する場合は取り込み後に別途入力する必要がある。
pub(super) fn default_rebar() -> RcRebar {
    let zero = BarSet {
        count: 0,
        dia: 0.0,
        layers: 0,
    };
    RcRebar {
        main_x: zero.clone(),
        main_y: zero,
        cover: 0.0,
        shear: ShearBar {
            dia: 0.0,
            pitch: 0.0,
            legs: 0,
            grade: None,
        },
    }
}

/// 鉄筋径の文字列を mm へ解釈する。数値ならそのまま、`D22`/`D10` のような呼び名は
/// 先頭の `D`/`d` を除いた数値を径とする（best-effort。厳密な JIS 公称径ではない）。
fn parse_bar_dia(v: &str) -> Option<f64> {
    if let Ok(x) = v.parse::<f64>() {
        return Some(x);
    }
    let t = v.trim();
    if let Some(rest) = t.strip_prefix(['D', 'd']) {
        return rest.trim().parse::<f64>().ok();
    }
    None
}

/// `StbSecBarArrangement*` の子要素の属性から [`RcRebar`] を復元する。
/// Squid-N の書き出し属性（`count_main_X`・`dia_main_X` 等）を優先しつつ、実 ST-Bridge で
/// 使われる名前（`D_main`・`N_main_X_1st`・`D_band` 等）や呼び名径（`D22`）も best-effort で
/// 拾う。欠落した属性は 0（無筋相当）を既定にする。弾性性能は b・d のみで決まるため、
/// 配筋の欠落・近似は往復での剛性に影響しない。
///
/// 段別本数（`N_main_X_1st`/`_2nd`/`_3rd`、梁は `N_main_top`/`_bottom` の各段）は合算して
/// 総本数とし、非ゼロの段数を `layers` に反映する。呼び名→公称径の正確な対応や、梁の
/// 上端/下端 ↔ 内部 `main_x`/`main_y`（せい/幅方向）の厳密な意味対応は今後の課題。
pub(super) fn parse_rebar(a: &HashMap<String, String>) -> RcRebar {
    let f = |keys: &[&str]| -> f64 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Ok(x) = v.parse::<f64>() {
                    return x;
                }
            }
        }
        0.0
    };
    // 径（数値 or 呼び名 `D22`）。
    let dia = |keys: &[&str]| -> f64 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Some(x) = parse_bar_dia(v) {
                    return x;
                }
            }
        }
        0.0
    };
    let u = |keys: &[&str]| -> u32 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Ok(x) = v.parse::<u32>() {
                    return x;
                }
            }
        }
        0
    };
    // 主筋本数と段数を求める。Squid 出力の合計本数キー（`count_main_*`）があれば
    // それを最優先で使う（往復での本数一致を保つ）。無ければ実 ST-Bridge の段別本数
    // （`N_main_*_1st`/`_2nd`/`_3rd`）を合算する（他社ファイルは段別にしか本数を持たず、
    // 1 段目だけ読むと下端筋の 2 段目等を取りこぼす）。段数は明示キー
    // （`count_main_layers_*`）を優先し、無ければ非ゼロの段数を数える。
    // 各引数: totals=合計本数キー, layers=段ごとの候補キー列, layer_attr=明示段数キー。
    let count_and_layers = |totals: &[&str], stages: &[&[&str]], layer_attr: &str| -> (u32, u32) {
        for k in totals {
            if let Some(x) = a.get(*k).and_then(|v| v.parse::<u32>().ok()) {
                let l = a
                    .get(layer_attr)
                    .and_then(|v| v.parse::<u32>().ok())
                    .filter(|&l| l > 0)
                    .unwrap_or(1);
                return (x, l);
            }
        }
        let mut sum = 0u32;
        let mut nonzero_stages = 0u32;
        for stage in stages {
            if let Some(x) = stage
                .iter()
                .find_map(|k| a.get(*k).and_then(|v| v.parse::<u32>().ok()))
            {
                if x > 0 {
                    sum += x;
                    nonzero_stages += 1;
                }
            }
        }
        let layers = a
            .get(layer_attr)
            .and_then(|v| v.parse::<u32>().ok())
            .filter(|&l| l > 0)
            .unwrap_or_else(|| nonzero_stages.max(1));
        (sum, layers)
    };
    // せい方向（X）／梁上端。合計本数キーが無いときは 1〜3 段目を合算する。
    let (count_x, layers_x) = count_and_layers(
        &["count_main_X", "count_main_top"],
        &[
            &["N_main_X_1st", "N_main_top_1st", "N_main_top"],
            &["N_main_X_2nd", "N_main_top_2nd"],
            &["N_main_X_3rd", "N_main_top_3rd"],
        ],
        "count_main_layers_X",
    );
    // 幅方向（Y）／梁下端。
    let (count_y, layers_y) = count_and_layers(
        &["count_main_Y", "count_main_bottom"],
        &[
            &["N_main_Y_1st", "N_main_bottom_1st", "N_main_bottom"],
            &["N_main_Y_2nd", "N_main_bottom_2nd"],
            &["N_main_Y_3rd", "N_main_bottom_3rd"],
        ],
        "count_main_layers_Y",
    );
    RcRebar {
        main_x: BarSet {
            count: count_x,
            dia: dia(&["dia_main_X", "dia_main", "D_main"]),
            layers: layers_x,
        },
        main_y: BarSet {
            count: count_y,
            dia: dia(&["dia_main_Y", "dia_main", "D_main"]),
            layers: layers_y,
        },
        cover: f(&["cover", "kaburi"]),
        shear: ShearBar {
            // 柱は帯筋 `D_band`、梁はあばら筋 `D_stirrup`（実 ST-Bridge 標準名）。
            dia: dia(&["D_band", "D_stirrup", "dia_band", "dia_stirrup", "dia_hoop"]),
            pitch: f(&["pitch_band", "pitch_stirrup", "pitch_hoop"]),
            legs: u(&[
                "N_band_direction_X",
                "N_stirrup",
                "count_band",
                "count_stirrup",
                "count_hoop",
            ]),
            grade: a
                .get("strength_band")
                .or_else(|| a.get("strength_stirrup"))
                .or_else(|| a.get("strength_bar_band"))
                .or_else(|| a.get("strength_main_band"))
                .cloned(),
        },
    }
}
