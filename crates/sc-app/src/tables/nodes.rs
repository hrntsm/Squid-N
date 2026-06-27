use crate::app::App;
use sc_core::ids::NodeId;
use sc_edit::{AddNode, SetNodeCoord, SetNodeRestraint};

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // バッファを model に同期（長さ合わせ + 未編集セルの更新）
    app.sync_node_edit();

    // 節点追加ボタン（node_edit 取り出し前に処理することで借用衝突を回避）
    ui.horizontal(|ui| {
        if ui.button("+ 節点追加").clicked() {
            app.undo.run(
                &mut app.model,
                Box::new(AddNode {
                    coord: [0.0, 0.0, 0.0],
                    restraint: sc_core::dof::Dof6Mask::FREE,
                }),
            );
            app.staleness.mark_edited();
        }
    });
    ui.separator();

    let n = app.model.nodes.len();
    // node_edit を一時的に取り出し、クロージャ内で app.model/undo と共存させる
    let mut node_edit = std::mem::take(&mut app.node_edit);
    // 確定待ちの編集（行、列、パース結果）
    let mut pending: Vec<(usize, usize, f64)> = Vec::new();
    // 確定待ちの拘束変更（行、新マスク）
    let mut pending_restraint: Vec<(usize, sc_core::dof::Dof6Mask)> = Vec::new();

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
                let is_focus = app.nav.focus_node == Some(node.id);
                row.col(|ui| {
                    let text = node.id.0.to_string();
                    // 選択行は blue-500 背景になるため文字は白、非選択は既定色
                    let rich = egui::RichText::new(text).color(if is_focus {
                        crate::theme::WHITE
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui.selectable_label(is_focus, rich).clicked() {
                        app.nav.focus_node = Some(node.id);
                    }
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
                                crate::theme::translucent(crate::theme::ERROR_RED, 60),
                            );
                        }
                    });
                }
                row.col(|ui| {
                    let r = node.restraint;
                    ui.horizontal(|ui| {
                        // プリセットボタン（自由／ピン／固定）
                        if ui.small_button("自由").clicked() {
                            pending_restraint.push((i, sc_core::dof::Dof6Mask::FREE));
                        }
                        if ui.small_button("ピン").clicked() {
                            pending_restraint.push((i, sc_core::dof::Dof6Mask::PINNED));
                        }
                        if ui.small_button("固定").clicked() {
                            pending_restraint.push((i, sc_core::dof::Dof6Mask::FIXED));
                        }
                        ui.separator();
                        // 各成分チェックボックス
                        use sc_core::dof::Dof;
                        for (d, lbl) in [
                            (Dof::Ux, "X"),
                            (Dof::Uy, "Y"),
                            (Dof::Uz, "Z"),
                            (Dof::Rx, "RX"),
                            (Dof::Ry, "RY"),
                            (Dof::Rz, "RZ"),
                        ] {
                            let mut on = r.is_fixed(d);
                            if ui.checkbox(&mut on, lbl).changed() {
                                let mut new_mask = r;
                                new_mask.set(d, on);
                                pending_restraint.push((i, new_mask));
                            }
                        }
                    });
                });
            });
        });

    // 確定処理（クロージャ外で app.model と app.undo にアクセス）
    let had_pending = !pending.is_empty();
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

    let had_restraint = !pending_restraint.is_empty();
    for (i, mask) in pending_restraint {
        let node_id = app.model.nodes[i].id;
        app.undo.run(
            &mut app.model,
            Box::new(SetNodeRestraint {
                node: node_id,
                restraint: mask,
            }),
        );
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_pending || had_restraint {
        app.staleness.mark_edited();
    }

    // バッファを戻す
    app.node_edit = node_edit;
}
