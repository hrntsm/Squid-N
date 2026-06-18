use crate::app::App;
use sc_core::ids::NodeId;
use sc_edit::SetNodeCoord;

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // バッファを model に同期（長さ合わせ + 未編集セルの更新）
    app.sync_node_edit();

    let n = app.model.nodes.len();
    // node_edit を一時的に取り出し、クロージャ内で app.model/undo と共存させる
    let mut node_edit = std::mem::take(&mut app.node_edit);
    // 確定待ちの編集（行、列、パース結果）
    let mut pending: Vec<(usize, usize, f64)> = Vec::new();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .columns(Column::initial(80.0), 3)
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "X", "Y", "Z", "拘束"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let node = &app.model.nodes[i];
                row.col(|ui| {
                    ui.label(node.id.0.to_string());
                });
                for (k, slot) in node_edit[i].iter_mut().enumerate().take(3) {
                    row.col(|ui| {
                        let resp = ui.add(
                            egui::TextEdit::singleline(slot)
                                .desired_width(70.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            if let Ok(val) = slot.trim().parse::<f64>() {
                                if (val - node.coord[k]).abs() > 1e-9 {
                                    pending.push((i, k, val));
                                }
                            }
                        }
                        // 数値以外は赤背景
                        if slot.trim().parse::<f64>().is_err() {
                            ui.painter().rect_filled(
                                resp.rect,
                                0.0,
                                egui::Color32::from_rgba_unmultiplied(200, 50, 50, 60),
                            );
                        }
                    });
                }
                row.col(|ui| {
                    ui.label(format!("{:?}", node.restraint));
                });
            });
        });

    // 確定処理（クロージャ外で app.model と app.undo にアクセス）
    for (i, k, val) in pending {
        let node_id = NodeId(app.model.nodes[i].id.0);
        let mut new_coord = app.model.nodes[i].coord;
        new_coord[k] = val;
        app.undo.run(
            &mut app.model,
            Box::new(SetNodeCoord {
                node: node_id,
                coord: new_coord,
            }),
        );
    }

    // バッファを戻す
    app.node_edit = node_edit;
}
