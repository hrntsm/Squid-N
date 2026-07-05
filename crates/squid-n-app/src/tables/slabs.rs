use crate::app::App;
use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{AreaLoad, DistributionMethod};
use squid_n_edit::{AddSlab, DeleteSlab};

/// スラブ追加フォームのドラフト状態（GUI 専用）。
/// `nodes` は境界4節点（頂点0→1→2→3→0 の順で外周を辿る）の選択状態。
#[derive(Clone, Debug)]
pub struct SlabDraft {
    pub nodes: [Option<NodeId>; 4],
    /// 荷重種別（既定 "DL"）
    pub load_kind: String,
    /// 荷重値の入力文字列。**UI 表示は kN/m²**（内部格納は ×1e-3 した N/mm²）。
    pub load_value: String,
    pub method: DistributionMethod,
}

impl Default for SlabDraft {
    fn default() -> Self {
        Self {
            nodes: [None; 4],
            load_kind: "DL".to_string(),
            load_value: "0".to_string(),
            method: DistributionMethod::TriTrapezoid,
        }
    }
}

fn method_label(m: DistributionMethod) -> &'static str {
    match m {
        DistributionMethod::TriTrapezoid => "三角/台形(45°法)",
        DistributionMethod::OneWay => "一方向",
        DistributionMethod::TributaryArea => "負担面積",
    }
}

pub fn slabs_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    ui.label(
        "スラブは境界4節点・外周の梁があって初めて機能します（結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと分配結果を確認できます）。",
    );
    ui.separator();

    // ── 一覧表 ──────────────────────────────────────────
    let n = app.model.slabs.len();
    let mut pending_delete: Option<SlabId> = None;

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(140.0))
        .column(Column::initial(200.0))
        .column(Column::initial(140.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "境界節点", "荷重", "分配法", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let slab = &app.model.slabs[i];
                row.col(|ui| {
                    ui.label(slab.id.0.to_string());
                });
                row.col(|ui| {
                    let s = slab
                        .boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    ui.label(s);
                });
                row.col(|ui| {
                    let s = slab
                        .loads
                        .iter()
                        .map(|l| format!("{} {:.2}kN/m²", l.kind, l.value * 1e3))
                        .collect::<Vec<_>>()
                        .join(", ");
                    ui.label(if s.is_empty() { "―".to_string() } else { s });
                });
                row.col(|ui| {
                    ui.label(method_label(slab.method));
                });
                row.col(|ui| {
                    if ui.button("🗑").on_hover_text("このスラブを削除").clicked() {
                        pending_delete = Some(slab.id);
                    }
                });
            });
        });

    if let Some(id) = pending_delete {
        app.undo.run(&mut app.model, Box::new(DeleteSlab { id }));
        app.staleness.mark_edited();
    }

    ui.separator();
    // ── スラブ追加フォーム ──────────────────────────────────
    ui.strong("スラブを追加");

    if app.model.nodes.len() < 4 {
        ui.label("スラブを追加するには節点が4つ以上必要です");
        return;
    }

    // 借用衝突を避けるため、節点一覧は先にローカルへ複製しておく
    // （app.model への参照を保持したまま app.slab_draft を可変参照しないため）。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();

    ui.label(
        "境界節点（頂点0→1→2→3→0 の順で外周を辿り、その辺 i=節点i→節点i+1 を持つ梁を検索します）:",
    );
    ui.horizontal_wrapped(|ui| {
        for k in 0..4 {
            let text = app.slab_draft.nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("slab_draft_node_{}", k))
                .selected_text(format!("頂点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        let label = format!("N{}", nid.0);
                        if ui
                            .selectable_label(app.slab_draft.nodes[k] == Some(nid), &label)
                            .clicked()
                        {
                            app.slab_draft.nodes[k] = Some(nid);
                        }
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("荷重種別:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_kind).desired_width(60.0));
        ui.label("荷重 [kN/m²]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_value).desired_width(80.0));
    });

    ui.horizontal(|ui| {
        ui.label("分配法:");
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TriTrapezoid,
            "三角/台形(45°法)",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::OneWay,
            "一方向",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TributaryArea,
            "負担面積",
        );
    });

    let selected: Vec<NodeId> = app.slab_draft.nodes.iter().filter_map(|n| *n).collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    let can_add = selected.len() == 4 && dedup.len() == 4;

    if ui
        .add_enabled(can_add, egui::Button::new("+ 追加"))
        .on_hover_text("境界節点4つがすべて選択され、かつ重複が無い場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .slab_draft
            .nodes
            .iter()
            .map(|n| n.expect("can_add で4つとも Some を確認済み"))
            .collect();
        let value_kn_m2 = app
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        // kN/m² → N/mm²（内部単位系）。1 kN/m² = 1e-3 N/mm²。
        let value = value_kn_m2 * 1e-3;
        let kind = app.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        app.undo.run(
            &mut app.model,
            Box::new(AddSlab {
                boundary,
                joists: Vec::new(),
                loads: vec![AreaLoad { kind, value }],
                method: app.slab_draft.method,
            }),
        );
        app.staleness.mark_edited();
    }
}
