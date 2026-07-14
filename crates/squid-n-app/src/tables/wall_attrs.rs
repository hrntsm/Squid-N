//! 壁属性（`Model.wall_attrs` = `WallAttr`: 開口面積・開口部重量・三方スリット）
//! の編集 UI。対象は `ElementKind::Wall`/`Shell` の部材のみ。
//! 編集は `squid_n_edit::{SetWallAttr, RemoveWallAttr}` 経由（undo 対応）。
//! 併せて、建物一律の複数開口の取り扱い（`Model.multi_opening_mode`）を
//! `squid_n_edit::SetMultiOpeningMode` 経由で編集する（undo 対応）。

use crate::app::App;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, MultiOpeningMode, WallAttr, WallOpening};
use squid_n_edit::{RemoveWallAttr, SetMultiOpeningMode, SetWallAttr};

/// 複数開口の取り扱い（`MultiOpeningMode`）の選択肢一覧（UI 表示順）。
const MULTI_OPENING_MODES: [MultiOpeningMode; 3] = [
    MultiOpeningMode::Equivalent,
    MultiOpeningMode::Envelope,
    MultiOpeningMode::Auto,
];

/// `MultiOpeningMode` の表示ラベル（RC規準「耐震壁の開口」の用語）。
fn multi_opening_mode_label(mode: MultiOpeningMode) -> &'static str {
    match mode {
        MultiOpeningMode::Equivalent => "等価開口とする",
        MultiOpeningMode::Envelope => "包絡する",
        MultiOpeningMode::Auto => "包絡開口・等価開口自動判定",
    }
}

/// 壁属性フォームのドラフト状態（GUI 専用）。
/// 対象壁を選択すると `synced_for` の壁の現在値でバッファを初期化し、
/// 「適用」で `SetWallAttr` を発行する。
#[derive(Clone, Debug, Default)]
pub struct WallAttrDraft {
    /// 編集対象の壁要素。
    pub elem: Option<ElemId>,
    /// バッファを初期化した対象（`elem` と異なれば model 値で再同期する）。
    pub synced_for: Option<ElemId>,
    /// 開口面積 [mm²] の入力バッファ（`openings` が空の場合のみ有効）。
    pub opening_area: String,
    /// 開口部重量 [N] の入力バッファ。
    pub opening_weight: String,
    /// 三方スリット。
    pub three_side_slit: bool,
    /// 個別開口寸法の入力バッファ。1行1開口または「,」区切りで
    /// `幅x高さ` または `幅x高さ@x,z`（位置指定付き）を入力する。
    /// 空文字列は「個別開口なし（`opening_area` を使用）」を表す。
    pub openings: String,
}

/// 個別開口の入力バッファ1件分の書式エラー。
/// `parse_openings` が返すメッセージには不正箇所の文字列を含める。
fn parse_single_opening(entry: &str) -> Result<WallOpening, String> {
    let (dims, offset) = match entry.split_once('@') {
        Some((d, o)) => (d, Some(o)),
        None => (entry, None),
    };
    let (w_str, h_str) = dims
        .split_once(['x', 'X'])
        .ok_or_else(|| format!("不正な開口指定「{entry}」: '幅x高さ' 形式で入力してください"))?;
    let width: f64 = w_str
        .trim()
        .parse()
        .map_err(|_| format!("不正な幅「{}」: 数値ではありません", w_str.trim()))?;
    let height: f64 = h_str
        .trim()
        .parse()
        .map_err(|_| format!("不正な高さ「{}」: 数値ではありません", h_str.trim()))?;
    if width <= 0.0 || height <= 0.0 {
        return Err(format!(
            "開口寸法「{entry}」: 幅・高さは正の値で入力してください"
        ));
    }
    let offset = match offset {
        Some(o) => {
            let (x_str, z_str) = o
                .split_once(',')
                .ok_or_else(|| format!("不正な位置指定「{o}」: 'x,z' 形式で入力してください"))?;
            let x: f64 = x_str
                .trim()
                .parse()
                .map_err(|_| format!("不正な位置x「{}」: 数値ではありません", x_str.trim()))?;
            let z: f64 = z_str
                .trim()
                .parse()
                .map_err(|_| format!("不正な位置z「{}」: 数値ではありません", z_str.trim()))?;
            Some([x, z])
        }
        None => None,
    };
    Ok(WallOpening {
        width,
        height,
        offset,
    })
}

/// 個別開口入力バッファ（1行1開口または「,」区切り、`幅x高さ` / `幅x高さ@x,z`）を
/// パースする（egui 非依存の純関数）。
///
/// 開口の位置指定 `@x,z` 自体がカンマを含むため、単純な「,」split だけでは
/// 「幅x高さ」を含まないトークン（位置の z 座標）を直前のトークンへ結合することで
/// 「,」区切りの開口列と「@x,z」内の「,」を区別する。
pub fn parse_openings(s: &str) -> Result<Vec<WallOpening>, String> {
    let normalized = s.replace('\n', ",");
    let mut entries: Vec<String> = Vec::new();
    for token in normalized.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        if token.contains(['x', 'X']) {
            entries.push(token.to_string());
        } else {
            match entries.last_mut() {
                Some(last) => {
                    last.push(',');
                    last.push_str(token);
                }
                None => {
                    return Err(format!(
                        "不正な開口指定「{token}」: '幅x高さ' 形式で入力してください"
                    ));
                }
            }
        }
    }
    entries.iter().map(|e| parse_single_opening(e)).collect()
}

/// 個別開口リストを入力バッファ書式（1行1開口、`幅x高さ` または `幅x高さ@x,z`）へ
/// 整形する（`parse_openings` の逆変換）。既存値をフォームへ読み込む際に使用する。
pub fn format_openings(openings: &[WallOpening]) -> String {
    openings
        .iter()
        .map(|o| match o.offset {
            Some([x, z]) => format!("{}x{}@{},{}", o.width, o.height, x, z),
            None => format!("{}x{}", o.width, o.height),
        })
        .collect::<Vec<_>>()
        .join("\n")
}

pub fn wall_attrs_table(ui: &mut egui::Ui, app: &mut App) {
    // ── 複数開口の取り扱い（建物一律） ─────────────────────────
    ui.horizontal(|ui| {
        ui.label("複数開口の取り扱い(建物一律):");
        let current = app.model.multi_opening_mode;
        let combo = egui::ComboBox::from_id_salt("multi_opening_mode")
            .selected_text(multi_opening_mode_label(current))
            .show_ui(ui, |ui| {
                for mode in MULTI_OPENING_MODES {
                    if ui
                        .selectable_label(current == mode, multi_opening_mode_label(mode))
                        .clicked()
                        && current != mode
                    {
                        app.undo
                            .run(&mut app.model, Box::new(SetMultiOpeningMode { mode }));
                        app.staleness.mark_edited();
                    }
                }
            });
        combo
            .response
            .on_hover_text("自動判定は開口間距離 l が l<1.5h または l<1m のとき包絡開口とみなします(h: 包絡開口とした場合の高さ。RC規準「耐震壁の開口」)。");
    });
    ui.label(
        "このモードは剛性の開口低減・耐震壁判定・検定の開口評価に適用されます\
         （自重控除は常に実開口面積を用います）。",
    );
    ui.separator();

    ui.label(
        "壁要素(Wall/Shell)の自重算定属性（開口控除・開口部重量・三方スリット）を設定します。",
    );
    ui.separator();

    let wall_elems: Vec<ElemId> = app
        .model
        .elements
        .iter()
        .filter(|e| matches!(e.kind, ElementKind::Wall | ElementKind::Shell))
        .map(|e| e.id)
        .collect();

    if wall_elems.is_empty() {
        ui.label("壁要素(Wall/Shell)がありません。");
        return;
    }

    // ── 既存の壁属性一覧 ─────────────────────────────────
    let mut pending_remove: Option<ElemId> = None;
    let mut pending_edit: Option<WallAttr> = None;
    if app.model.wall_attrs.is_empty() {
        ui.label("設定済みの壁属性はありません（未設定の壁は開口なしとして扱われます）。");
    } else {
        for attr in &app.model.wall_attrs {
            ui.horizontal(|ui| {
                let opening_desc = if attr.openings.is_empty() {
                    format!("開口 {:.0} mm²", attr.opening_area)
                } else {
                    format!(
                        "開口 {}個 Σ{:.2e} mm²",
                        attr.openings.len(),
                        attr.total_opening_area()
                    )
                };
                ui.label(format!(
                    "壁#{}: {} / 開口部重量 {:.0} N / 三方スリット: {}",
                    attr.elem.0,
                    opening_desc,
                    attr.opening_weight,
                    if attr.three_side_slit {
                        "あり"
                    } else {
                        "なし"
                    }
                ));
                if ui
                    .button("✏")
                    .on_hover_text("フォームへ読み込んで編集")
                    .clicked()
                {
                    pending_edit = Some(attr.clone());
                }
                if ui.button("🗑").on_hover_text("この壁属性を削除").clicked() {
                    pending_remove = Some(attr.elem);
                }
            });
        }
    }
    if let Some(attr) = pending_edit {
        app.wall_attr_draft.elem = Some(attr.elem);
        app.wall_attr_draft.synced_for = Some(attr.elem);
        app.wall_attr_draft.opening_area = format!("{:.0}", attr.opening_area);
        app.wall_attr_draft.opening_weight = format!("{:.0}", attr.opening_weight);
        app.wall_attr_draft.three_side_slit = attr.three_side_slit;
        app.wall_attr_draft.openings = format_openings(&attr.openings);
    }
    if let Some(elem) = pending_remove {
        app.undo
            .run(&mut app.model, Box::new(RemoveWallAttr { elem }));
        app.staleness.mark_edited();
    }

    ui.separator();
    ui.strong("壁属性を設定");

    // 対象壁の選択（変更時に model 値でバッファを再同期）
    ui.horizontal(|ui| {
        ui.label("対象壁:");
        let text = app
            .wall_attr_draft
            .elem
            .map(|e| format!("壁#{}", e.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("wall_attr_elem")
            .selected_text(text)
            .show_ui(ui, |ui| {
                for &eid in &wall_elems {
                    if ui
                        .selectable_label(
                            app.wall_attr_draft.elem == Some(eid),
                            format!("壁#{}", eid.0),
                        )
                        .clicked()
                    {
                        app.wall_attr_draft.elem = Some(eid);
                    }
                }
            });
    });
    if app.wall_attr_draft.elem != app.wall_attr_draft.synced_for {
        if let Some(eid) = app.wall_attr_draft.elem {
            let existing = app.model.wall_attrs.iter().find(|a| a.elem == eid);
            let (area, weight, slit, openings) = existing
                .map(|a| {
                    (
                        a.opening_area,
                        a.opening_weight,
                        a.three_side_slit,
                        a.openings.clone(),
                    )
                })
                .unwrap_or((0.0, 0.0, false, Vec::new()));
            app.wall_attr_draft.opening_area = format!("{:.0}", area);
            app.wall_attr_draft.opening_weight = format!("{:.0}", weight);
            app.wall_attr_draft.three_side_slit = slit;
            app.wall_attr_draft.openings = format_openings(&openings);
            app.wall_attr_draft.synced_for = Some(eid);
        }
    }

    ui.horizontal(|ui| {
        ui.label("開口面積[mm²]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.wall_attr_draft.opening_area).desired_width(90.0),
        );
        ui.label("開口部重量[N]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.wall_attr_draft.opening_weight).desired_width(90.0),
        );
        ui.checkbox(&mut app.wall_attr_draft.three_side_slit, "三方スリット")
            .on_hover_text("有効にすると壁自重は上下分配せず全て壁頂部の節点へ伝達されます");
    });

    ui.label("個別開口（任意・耐震壁判定/剛性計算の複数開口寸法）:");
    ui.add(
        egui::TextEdit::multiline(&mut app.wall_attr_draft.openings)
            .desired_rows(3)
            .desired_width(320.0)
            .hint_text(
                "幅x高さ または 幅x高さ@x,z（位置指定）\n\
                 複数開口は改行または「,」区切りで入力\n\
                 例: 1000x2000, 800x900@3000,500",
            ),
    )
    .on_hover_text(
        "1行1開口または「,」区切りで '幅x高さ' もしくは位置付き '幅x高さ@x,z' を入力します。\
         空欄の場合は開口面積[mm²]の入力値がそのまま使われます。",
    );

    let parsed_openings = parse_openings(&app.wall_attr_draft.openings);
    match &parsed_openings {
        Ok(openings) if !openings.is_empty() => {
            let sum_area: f64 = openings.iter().map(WallOpening::area).sum();
            ui.label(format!(
                "個別開口 {}個 Σ{:.2e} mm²（開口面積[mm²]の入力値は無視され、\
                 個別開口の面積和が優先されます）",
                openings.len(),
                sum_area
            ));
        }
        Ok(_) => {}
        Err(e) => {
            ui.colored_label(
                crate::theme::ERROR_RED,
                format!("個別開口の書式エラー: {e}"),
            );
        }
    }

    let parsed_area = app.wall_attr_draft.opening_area.trim().parse::<f64>();
    let parsed_weight = app.wall_attr_draft.opening_weight.trim().parse::<f64>();
    let can_apply = app.wall_attr_draft.elem.is_some()
        && parsed_area.is_ok()
        && parsed_weight.is_ok()
        && parsed_openings.is_ok();
    if ui
        .add_enabled(can_apply, egui::Button::new("✔ 適用"))
        .on_hover_text("選択した壁に開口・スリット属性を設定します（undo可）")
        .clicked()
    {
        if let (Some(elem), Ok(opening_area), Ok(opening_weight), Ok(openings)) = (
            app.wall_attr_draft.elem,
            parsed_area,
            parsed_weight,
            parsed_openings,
        ) {
            app.undo.run(
                &mut app.model,
                Box::new(SetWallAttr {
                    attr: WallAttr {
                        elem,
                        opening_area,
                        opening_weight,
                        three_side_slit: app.wall_attr_draft.three_side_slit,
                        openings,
                    },
                }),
            );
            app.staleness.mark_edited();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 空文字列は「個別開口なし」を表し、空の Vec になること。
    #[test]
    fn test_parse_openings_empty_is_empty_vec() {
        assert_eq!(parse_openings("").unwrap(), Vec::new());
        assert_eq!(parse_openings("   ").unwrap(), Vec::new());
    }

    /// 課題例の「,」区切り＋位置指定付き開口が正しくパースされること
    /// （offset のカンマと開口区切りのカンマの曖昧性を解消できているか）。
    #[test]
    fn test_parse_openings_comma_separated_with_offset() {
        let openings = parse_openings("1000x2000, 800x900@3000,500").unwrap();
        assert_eq!(
            openings,
            vec![
                WallOpening {
                    width: 1000.0,
                    height: 2000.0,
                    offset: None,
                },
                WallOpening {
                    width: 800.0,
                    height: 900.0,
                    offset: Some([3000.0, 500.0]),
                },
            ]
        );
    }

    /// 改行区切り（1行1開口）でも同じ結果になること。
    #[test]
    fn test_parse_openings_newline_separated() {
        let openings = parse_openings("1000x2000\n800x900@3000,500").unwrap();
        assert_eq!(
            openings,
            vec![
                WallOpening {
                    width: 1000.0,
                    height: 2000.0,
                    offset: None,
                },
                WallOpening {
                    width: 800.0,
                    height: 900.0,
                    offset: Some([3000.0, 500.0]),
                },
            ]
        );
    }

    /// 大文字 'X' や前後の空白を許容すること。
    #[test]
    fn test_parse_openings_tolerates_whitespace_and_uppercase_x() {
        let openings = parse_openings("  1000 X 2000  ").unwrap();
        assert_eq!(
            openings,
            vec![WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: None
            }]
        );
    }

    /// 'x' を含まない不正な書式はエラーになること。
    #[test]
    fn test_parse_openings_rejects_missing_x_separator() {
        let err = parse_openings("1000,2000").unwrap_err();
        assert!(err.contains("1000"), "err={err}");
    }

    /// 数値でない幅・高さはエラーになること。
    #[test]
    fn test_parse_openings_rejects_non_numeric() {
        assert!(parse_openings("abcxdef").is_err());
    }

    /// 幅・高さが 0 以下はエラーになること。
    #[test]
    fn test_parse_openings_rejects_non_positive_dims() {
        assert!(parse_openings("0x2000").is_err());
        assert!(parse_openings("1000x-5").is_err());
    }

    /// 位置指定の書式が 'x,z' でない場合はエラーになること。
    #[test]
    fn test_parse_openings_rejects_malformed_offset() {
        assert!(parse_openings("1000x2000@3000").is_err());
    }

    /// format_openings は parse_openings の逆変換になっていること（往復一致）。
    #[test]
    fn test_format_openings_roundtrip() {
        let openings = vec![
            WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: None,
            },
            WallOpening {
                width: 800.0,
                height: 900.0,
                offset: Some([3000.0, 500.0]),
            },
        ];
        let formatted = format_openings(&openings);
        assert_eq!(parse_openings(&formatted).unwrap(), openings);
    }

    /// 空リストは空文字列へ整形されること。
    #[test]
    fn test_format_openings_empty() {
        assert_eq!(format_openings(&[]), "");
    }
}
