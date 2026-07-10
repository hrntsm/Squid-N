//! フレーム外雑壁（`Model.misc_walls` = `MiscWall`）の編集 UI。
//! 始点・終点・高さ・面重量・伝達タイプの追加/編集/削除を提供する。
//! 編集は `squid_n_edit::{AddMiscWall, DeleteMiscWall, SetMiscWall}` 経由（undo 対応）。

use crate::app::App;
use squid_n_core::model::{MiscWall, MiscWallTransfer};
use squid_n_edit::{AddMiscWall, DeleteMiscWall, SetMiscWall};

/// 雑壁追加/編集フォームのドラフト状態（GUI 専用）。
#[derive(Clone, Debug)]
pub struct MiscWallDraft {
    /// 始点座標 [mm] の入力バッファ。
    pub start: [String; 3],
    /// 終点座標 [mm] の入力バッファ。
    pub end: [String; 3],
    /// 高さ [mm]。
    pub height: String,
    /// 面重量 [kN/m²]（内部単位 N/mm² へは ×1e-3 で変換。スラブ荷重入力と同じ流儀）。
    pub weight_kn_m2: String,
    /// 伝達タイプ。
    pub transfer: MiscWallTransfer,
    /// 編集中の行（None = 新規追加モード）。
    pub editing: Option<usize>,
}

impl Default for MiscWallDraft {
    fn default() -> Self {
        Self {
            start: ["0".into(), "0".into(), "0".into()],
            end: ["0".into(), "0".into(), "0".into()],
            height: "3000".into(),
            weight_kn_m2: "1.0".into(),
            transfer: MiscWallTransfer::Column,
            editing: None,
        }
    }
}

fn transfer_label(t: MiscWallTransfer) -> &'static str {
    match t {
        MiscWallTransfer::Column => "柱伝達",
        MiscWallTransfer::Beam => "梁伝達",
        MiscWallTransfer::SelfStanding => "自立",
    }
}

pub fn misc_walls_table(ui: &mut egui::Ui, app: &mut App) {
    ui.label("フレーム外雑壁（部材としてモデル化しない壁）を定義します。0.5m 分割規則で近傍の節点へ重量が集計されます。");
    ui.separator();

    // ── 一覧 ───────────────────────────────────────────────
    let mut pending_delete: Option<usize> = None;
    let mut pending_edit: Option<usize> = None;
    if app.model.misc_walls.is_empty() {
        ui.label("雑壁はありません。");
    } else {
        for (i, w) in app.model.misc_walls.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "#{}: ({:.0},{:.0},{:.0})→({:.0},{:.0},{:.0}) h={:.0}mm w={:.2}kN/m² {}",
                    i,
                    w.start[0],
                    w.start[1],
                    w.start[2],
                    w.end[0],
                    w.end[1],
                    w.end[2],
                    w.height,
                    w.weight_per_area * 1e3,
                    transfer_label(w.transfer),
                ));
                if ui
                    .button("✏")
                    .on_hover_text("フォームへ読み込んで編集")
                    .clicked()
                {
                    pending_edit = Some(i);
                }
                if ui.button("🗑").on_hover_text("この雑壁を削除").clicked() {
                    pending_delete = Some(i);
                }
            });
        }
    }
    if let Some(i) = pending_edit {
        if let Some(w) = app.model.misc_walls.get(i) {
            for k in 0..3 {
                app.misc_wall_draft.start[k] = format!("{:.0}", w.start[k]);
                app.misc_wall_draft.end[k] = format!("{:.0}", w.end[k]);
            }
            app.misc_wall_draft.height = format!("{:.0}", w.height);
            app.misc_wall_draft.weight_kn_m2 = format!("{:.3}", w.weight_per_area * 1e3);
            app.misc_wall_draft.transfer = w.transfer;
            app.misc_wall_draft.editing = Some(i);
        }
    }
    if let Some(i) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteMiscWall { index: i }));
        if app.misc_wall_draft.editing == Some(i) {
            app.misc_wall_draft.editing = None;
        }
        app.staleness.mark_edited();
    }

    ui.separator();
    match app.misc_wall_draft.editing {
        Some(i) => ui.strong(format!("雑壁 #{} を編集", i)),
        None => ui.strong("雑壁を追加"),
    };

    ui.horizontal(|ui| {
        ui.label("始点[mm]:");
        for k in 0..3 {
            ui.add(
                egui::TextEdit::singleline(&mut app.misc_wall_draft.start[k]).desired_width(70.0),
            );
        }
        ui.label("終点[mm]:");
        for k in 0..3 {
            ui.add(egui::TextEdit::singleline(&mut app.misc_wall_draft.end[k]).desired_width(70.0));
        }
    });
    ui.horizontal(|ui| {
        ui.label("高さ[mm]:");
        ui.add(egui::TextEdit::singleline(&mut app.misc_wall_draft.height).desired_width(70.0));
        ui.label("面重量[kN/m²]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.misc_wall_draft.weight_kn_m2).desired_width(70.0),
        );
        ui.label("伝達:");
        for t in [
            MiscWallTransfer::Column,
            MiscWallTransfer::Beam,
            MiscWallTransfer::SelfStanding,
        ] {
            ui.selectable_value(&mut app.misc_wall_draft.transfer, t, transfer_label(t));
        }
    });

    // 入力のパース（全て数値になったら追加/更新可能）
    let parse3 = |bufs: &[String; 3]| -> Option<[f64; 3]> {
        let mut out = [0.0; 3];
        for (k, b) in bufs.iter().enumerate() {
            out[k] = b.trim().parse::<f64>().ok()?;
        }
        Some(out)
    };
    let start = parse3(&app.misc_wall_draft.start);
    let end = parse3(&app.misc_wall_draft.end);
    let height = app.misc_wall_draft.height.trim().parse::<f64>().ok();
    let weight = app
        .misc_wall_draft
        .weight_kn_m2
        .trim()
        .parse::<f64>()
        .ok()
        // kN/m² → N/mm²（内部単位系）
        .map(|w| w * 1e-3);
    let can_commit = start.is_some() && end.is_some() && height.is_some() && weight.is_some();

    ui.horizontal(|ui| {
        let label = if app.misc_wall_draft.editing.is_some() {
            "✔ 更新"
        } else {
            "+ 追加"
        };
        if ui
            .add_enabled(can_commit, egui::Button::new(label))
            .clicked()
        {
            if let (Some(start), Some(end), Some(height), Some(weight_per_area)) =
                (start, end, height, weight)
            {
                let wall = MiscWall {
                    start,
                    end,
                    height,
                    weight_per_area,
                    transfer: app.misc_wall_draft.transfer,
                };
                match app.misc_wall_draft.editing {
                    Some(index) if index < app.model.misc_walls.len() => {
                        app.undo
                            .run(&mut app.model, Box::new(SetMiscWall { index, wall }));
                    }
                    _ => {
                        app.undo.run(&mut app.model, Box::new(AddMiscWall { wall }));
                    }
                }
                app.misc_wall_draft.editing = None;
                app.staleness.mark_edited();
            }
        }
        if app.misc_wall_draft.editing.is_some() && ui.button("✖ 編集をやめる").clicked() {
            app.misc_wall_draft.editing = None;
        }
    });
}
