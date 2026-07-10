//! 壁属性（`Model.wall_attrs` = `WallAttr`: 開口面積・開口部重量・三方スリット）
//! の編集 UI。対象は `ElementKind::Wall`/`Shell` の部材のみ。
//! 編集は `squid_n_edit::{SetWallAttr, RemoveWallAttr}` 経由（undo 対応）。

use crate::app::App;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, WallAttr};
use squid_n_edit::{RemoveWallAttr, SetWallAttr};

/// 壁属性フォームのドラフト状態（GUI 専用）。
/// 対象壁を選択すると `synced_for` の壁の現在値でバッファを初期化し、
/// 「適用」で `SetWallAttr` を発行する。
#[derive(Clone, Debug, Default)]
pub struct WallAttrDraft {
    /// 編集対象の壁要素。
    pub elem: Option<ElemId>,
    /// バッファを初期化した対象（`elem` と異なれば model 値で再同期する）。
    pub synced_for: Option<ElemId>,
    /// 開口面積 [mm²] の入力バッファ。
    pub opening_area: String,
    /// 開口部重量 [N] の入力バッファ。
    pub opening_weight: String,
    /// 三方スリット。
    pub three_side_slit: bool,
}

pub fn wall_attrs_table(ui: &mut egui::Ui, app: &mut App) {
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
                ui.label(format!(
                    "壁#{}: 開口 {:.0} mm² / 開口部重量 {:.0} N / 三方スリット: {}",
                    attr.elem.0,
                    attr.opening_area,
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
            let (area, weight, slit) = existing
                .map(|a| (a.opening_area, a.opening_weight, a.three_side_slit))
                .unwrap_or((0.0, 0.0, false));
            app.wall_attr_draft.opening_area = format!("{:.0}", area);
            app.wall_attr_draft.opening_weight = format!("{:.0}", weight);
            app.wall_attr_draft.three_side_slit = slit;
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

    let parsed_area = app.wall_attr_draft.opening_area.trim().parse::<f64>();
    let parsed_weight = app.wall_attr_draft.opening_weight.trim().parse::<f64>();
    let can_apply =
        app.wall_attr_draft.elem.is_some() && parsed_area.is_ok() && parsed_weight.is_ok();
    if ui
        .add_enabled(can_apply, egui::Button::new("✔ 適用"))
        .on_hover_text("選択した壁に開口・スリット属性を設定します（undo可）")
        .clicked()
    {
        if let (Some(elem), Ok(opening_area), Ok(opening_weight)) =
            (app.wall_attr_draft.elem, parsed_area, parsed_weight)
        {
            app.undo.run(
                &mut app.model,
                Box::new(SetWallAttr {
                    attr: WallAttr {
                        elem,
                        opening_area,
                        opening_weight,
                        three_side_slit: app.wall_attr_draft.three_side_slit,
                    },
                }),
            );
            app.staleness.mark_edited();
        }
    }
}
