//! 荷重計算条件（`Model.load_cfg` = `LoadCfg`）の編集 UI。
//!
//! 鉄骨重量割増率・K型ブレース配分規則・積載荷重低減の考慮と、部材別の
//! 付加線重量／仕上げ面重量／ダンパー諸元の簡易テーブルを提供する。
//! 全ての編集は `squid_n_edit::SetLoadCfg`（全置換・undo 対応）経由で行う。

use crate::app::App;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{DamperSpec, KBraceWeightRule, LoadCfg};
use squid_n_edit::SetLoadCfg;

/// 荷重計算条件フォームのドラフト状態（GUI 専用）。
///
/// 鉄骨重量割増率は `story_weight_edit` と同じ「操作中は model 値で上書きしない」
/// パターン（DragValue + active フラグ）。各テーブルの追加フォームは文字列
/// バッファで、「+ 追加」押下時にのみ `SetLoadCfg` を発行するため同期問題はない。
#[derive(Clone, Debug)]
pub struct LoadCfgDraft {
    /// 鉄骨重量割増率の編集バッファ。
    pub steel_factor: f64,
    /// `steel_factor` が現在操作中（ドラッグ中またはフォーカス中）か。
    pub steel_factor_active: bool,
    /// 付加線重量の追加フォーム: 値 [N/mm]。
    pub extra_value: String,
    /// 仕上げ面重量の追加フォーム: 値 [N/mm²]。
    pub finish_value: String,
    /// ダンパー諸元の追加フォーム: 装置重量 [N]・装置長さ [mm]・支持部断面積 [mm²]。
    pub damper_weight: String,
    pub damper_length: String,
    pub damper_area: String,
    /// 各追加フォームで選択中の部材（付加線重量/仕上げ/ダンパー共用）。
    pub sel_elem: Option<ElemId>,
}

impl Default for LoadCfgDraft {
    fn default() -> Self {
        Self {
            steel_factor: 1.0,
            steel_factor_active: false,
            extra_value: "0".into(),
            finish_value: "0".into(),
            damper_weight: "0".into(),
            damper_length: "0".into(),
            damper_area: "0".into(),
            sel_elem: None,
        }
    }
}

fn k_brace_rule_label(rule: KBraceWeightRule) -> &'static str {
    match rule {
        KBraceWeightRule::InternalNodes => "内部節点にも配分",
        KBraceWeightRule::BaseNodesOnly => "基準節点のみ",
    }
}

/// `SetLoadCfg` を発行して undo 可能に全置換する共通ヘルパー。
fn commit(app: &mut App, cfg: LoadCfg) {
    app.undo
        .run(&mut app.model, Box::new(SetLoadCfg { cfg: Some(cfg) }));
    app.staleness.mark_edited();
}

/// (ElemId, f64) リストの表示・削除と追加フォームの共通 UI。
/// 変更後のリストを返す（None = 変更なし）。
fn elem_value_table(
    ui: &mut egui::Ui,
    app_model: &squid_n_core::model::Model,
    id_salt: &str,
    rows: &[(ElemId, f64)],
    value_label: &str,
    sel_elem: &mut Option<ElemId>,
    value_buf: &mut String,
) -> Option<Vec<(ElemId, f64)>> {
    let mut result: Option<Vec<(ElemId, f64)>> = None;
    for (i, (elem, value)) in rows.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("部材#{}: {} {:.4}", elem.0, value_label, value));
            if ui.button("🗑").on_hover_text("この行を削除").clicked() {
                let mut new_rows = rows.to_vec();
                new_rows.remove(i);
                result = Some(new_rows);
            }
        });
    }
    ui.horizontal(|ui| {
        elem_selector(ui, app_model, &format!("{id_salt}_elem"), sel_elem);
        ui.label(value_label);
        ui.add(egui::TextEdit::singleline(value_buf).desired_width(80.0));
        let can_add = sel_elem.is_some() && value_buf.trim().parse::<f64>().is_ok();
        if ui
            .add_enabled(can_add, egui::Button::new("+ 追加"))
            .clicked()
        {
            if let (Some(elem), Ok(value)) = (*sel_elem, value_buf.trim().parse::<f64>()) {
                let mut new_rows = rows.to_vec();
                // 同一部材の既存行は置換（重複行を作らない）。
                if let Some(pos) = new_rows.iter().position(|(e, _)| *e == elem) {
                    new_rows[pos].1 = value;
                } else {
                    new_rows.push((elem, value));
                }
                result = Some(new_rows);
            }
        }
    });
    result
}

/// 部材選択 ComboBox（全部材から選ぶ）。
fn elem_selector(
    ui: &mut egui::Ui,
    model: &squid_n_core::model::Model,
    id_salt: &str,
    selected: &mut Option<ElemId>,
) {
    let text = selected
        .map(|e| format!("部材#{}", e.0))
        .unwrap_or_else(|| "―".to_string());
    egui::ComboBox::from_id_salt(id_salt.to_string())
        .selected_text(text)
        .show_ui(ui, |ui| {
            for elem in &model.elements {
                if ui
                    .selectable_label(
                        *selected == Some(elem.id),
                        format!("部材#{} ({:?})", elem.id.0, elem.kind),
                    )
                    .clicked()
                {
                    *selected = Some(elem.id);
                }
            }
        });
}

/// 荷重計算条件パネル本体。
pub fn load_cfg_panel(ui: &mut egui::Ui, app: &mut App) {
    let cfg = app.model.load_cfg.clone().unwrap_or_default();

    // ── 鉄骨重量割増率 ─────────────────────────────────────
    if !app.load_cfg_draft.steel_factor_active {
        app.load_cfg_draft.steel_factor = cfg.steel_weight_factor;
    }
    ui.horizontal(|ui| {
        ui.label("鉄骨重量割増率 α:");
        let resp = ui
            .add(
                egui::DragValue::new(&mut app.load_cfg_draft.steel_factor)
                    .speed(0.01)
                    .range(0.0..=3.0),
            )
            .on_hover_text("コンクリート材(Fcあり)には適用されません。0以下は1.0として扱われます");
        app.load_cfg_draft.steel_factor_active = resp.dragged() || resp.has_focus();
        if (resp.drag_stopped() || resp.lost_focus())
            && (app.load_cfg_draft.steel_factor - cfg.steel_weight_factor).abs() > 1e-9
        {
            let mut new_cfg = cfg.clone();
            new_cfg.steel_weight_factor = app.load_cfg_draft.steel_factor;
            // ドラッグ終了/フォーカス喪失と同一フレームで他の操作は発生しない
            // （egui の1フレーム1操作）ため、以降の描画が旧 cfg を参照しても
            // 次フレームで model の新値に再同期される。
            commit(app, new_cfg);
        }
    });

    // ── K型ブレース配分規則 ─────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("K型ブレース重量配分:");
        let mut new_rule: Option<KBraceWeightRule> = None;
        egui::ComboBox::from_id_salt("load_cfg_k_brace_rule")
            .selected_text(k_brace_rule_label(cfg.k_brace_rule))
            .show_ui(ui, |ui| {
                for rule in [
                    KBraceWeightRule::InternalNodes,
                    KBraceWeightRule::BaseNodesOnly,
                ] {
                    if ui
                        .selectable_label(cfg.k_brace_rule == rule, k_brace_rule_label(rule))
                        .clicked()
                        && cfg.k_brace_rule != rule
                    {
                        new_rule = Some(rule);
                    }
                }
            });
        if let Some(rule) = new_rule {
            let mut new_cfg = cfg.clone();
            new_cfg.k_brace_rule = rule;
            commit(app, new_cfg);
        }
    });

    // ── 積載荷重低減 ─────────────────────────────────────
    {
        let mut reduction = cfg.live_load_reduction;
        if ui
            .checkbox(
                &mut reduction,
                "柱軸力の積載荷重低減を考慮する（令85条2項）",
            )
            .on_hover_text(
                "支える床の数に応じた低減率を集計します。現状は設計タブでの参考表示のみで、\
                 断面検定の軸力への実適用は未対応（残課題）です",
            )
            .changed()
        {
            let mut new_cfg = cfg.clone();
            new_cfg.live_load_reduction = reduction;
            commit(app, new_cfg);
            return;
        }
    }

    ui.add_space(4.0);

    // ── 付加線重量 ─────────────────────────────────────────
    ui.label(egui::RichText::new("付加線重量（耐火被覆等の直接入力）").strong());
    if let Some(new_rows) = elem_value_table(
        ui,
        &app.model,
        "load_cfg_extra",
        &cfg.extra_line_weight,
        "[N/mm]",
        &mut app.load_cfg_draft.sel_elem,
        &mut app.load_cfg_draft.extra_value,
    ) {
        let mut new_cfg = cfg.clone();
        new_cfg.extra_line_weight = new_rows;
        commit(app, new_cfg);
        return;
    }

    ui.add_space(4.0);

    // ── 仕上げ面重量 ────────────────────────────────────────
    ui.label(egui::RichText::new("仕上げ面重量（断面周長×面重量で線重量に換算）").strong());
    if let Some(new_rows) = elem_value_table(
        ui,
        &app.model,
        "load_cfg_finish",
        &cfg.finish_area_weight,
        "[N/mm²]",
        &mut app.load_cfg_draft.sel_elem,
        &mut app.load_cfg_draft.finish_value,
    ) {
        let mut new_cfg = cfg.clone();
        new_cfg.finish_area_weight = new_rows;
        commit(app, new_cfg);
        return;
    }

    ui.add_space(4.0);

    // ── ダンパー諸元 ────────────────────────────────────────
    ui.label(egui::RichText::new("ダンパー自重諸元（断面自重を装置+支持部重量で置換）").strong());
    let mut new_dampers: Option<Vec<DamperSpec>> = None;
    for (i, d) in cfg.dampers.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!(
                "部材#{}: 装置 {:.0} N / 長さ {:.0} mm / 支持部 {:.0} mm²",
                d.elem.0, d.device_weight, d.device_length, d.support_area
            ));
            if ui
                .button("🗑")
                .on_hover_text("このダンパー諸元を削除")
                .clicked()
            {
                let mut rows = cfg.dampers.clone();
                rows.remove(i);
                new_dampers = Some(rows);
            }
        });
    }
    ui.horizontal(|ui| {
        elem_selector(
            ui,
            &app.model,
            "load_cfg_damper_elem",
            &mut app.load_cfg_draft.sel_elem,
        );
        ui.label("装置[N]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.load_cfg_draft.damper_weight).desired_width(60.0),
        );
        ui.label("長さ[mm]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.load_cfg_draft.damper_length).desired_width(60.0),
        );
        ui.label("支持部[mm²]:");
        ui.add(egui::TextEdit::singleline(&mut app.load_cfg_draft.damper_area).desired_width(60.0));
        let parsed = (
            app.load_cfg_draft.sel_elem,
            app.load_cfg_draft.damper_weight.trim().parse::<f64>(),
            app.load_cfg_draft.damper_length.trim().parse::<f64>(),
            app.load_cfg_draft.damper_area.trim().parse::<f64>(),
        );
        let can_add =
            parsed.0.is_some() && parsed.1.is_ok() && parsed.2.is_ok() && parsed.3.is_ok();
        if ui
            .add_enabled(can_add, egui::Button::new("+ 追加"))
            .clicked()
        {
            if let (Some(elem), Ok(w), Ok(l), Ok(a)) = parsed {
                let spec = DamperSpec {
                    elem,
                    device_weight: w,
                    device_length: l,
                    support_area: a,
                };
                let mut rows = cfg.dampers.clone();
                if let Some(pos) = rows.iter().position(|d| d.elem == elem) {
                    rows[pos] = spec;
                } else {
                    rows.push(spec);
                }
                new_dampers = Some(rows);
            }
        }
    });
    if let Some(rows) = new_dampers {
        let mut new_cfg = cfg.clone();
        new_cfg.dampers = rows;
        commit(app, new_cfg);
    }
}
