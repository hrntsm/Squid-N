use crate::app::App;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};
use squid_n_edit::{AddMember, DeleteMember, SetElementMaterial, SetElementSection};

pub fn members_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // ── 梁追加フォーム ──────────────────────────────────────────
    if app.model.nodes.len() < 2 {
        ui.label("梁を追加するには節点が2つ以上必要です");
    } else {
        let id_i = egui::Id::new("add_member_sel_i");
        let id_j = egui::Id::new("add_member_sel_j");

        // egui 一時メモリから選択済み節点IDを取得。未設定なら先頭/2番目の節点で初期化。
        let mut sel_i: Option<NodeId> = ui
            .data(|d| d.get_temp::<Option<NodeId>>(id_i))
            .flatten()
            .or_else(|| app.model.nodes.first().map(|n| n.id));
        let mut sel_j: Option<NodeId> = ui
            .data(|d| d.get_temp::<Option<NodeId>>(id_j))
            .flatten()
            .or_else(|| app.model.nodes.get(1).map(|n| n.id));

        let mut do_add = false;
        // 免震支承材の作成フォームは仕様策定中のためプレースホルダ（押すと未実装通知）。
        let mut do_isolator_notice = false;

        ui.horizontal(|ui| {
            ui.label("梁追加:");

            // i 節点 ComboBox
            let i_text = sel_i
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("add_member_i")
                .selected_text(i_text)
                .show_ui(ui, |ui| {
                    for node in &app.model.nodes {
                        let label = format!("N{}", node.id.0);
                        if ui
                            .selectable_label(sel_i == Some(node.id), &label)
                            .clicked()
                        {
                            sel_i = Some(node.id);
                        }
                    }
                });

            // j 節点 ComboBox
            let j_text = sel_j
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("add_member_j")
                .selected_text(j_text)
                .show_ui(ui, |ui| {
                    for node in &app.model.nodes {
                        let label = format!("N{}", node.id.0);
                        if ui
                            .selectable_label(sel_j == Some(node.id), &label)
                            .clicked()
                        {
                            sel_j = Some(node.id);
                        }
                    }
                });

            // i != j のときのみ追加ボタンを有効化
            let enabled = matches!((sel_i, sel_j), (Some(i), Some(j)) if i != j);
            if ui
                .add_enabled(enabled, egui::Button::new("+ 部材追加"))
                .clicked()
            {
                do_add = true;
            }

            ui.separator();
            // 免震支承材の作成フォーム（仕様策定中）。ボタンのみ用意し、押下時は未実装通知。
            if ui
                .button("+ 免震支承材追加")
                .on_hover_text("免震支承材の作成フォームは仕様策定中（未実装）")
                .clicked()
            {
                do_isolator_notice = true;
            }
        });

        // クロージャ終了後に一時メモリ更新（借用の競合を避ける）
        ui.data_mut(|d| d.insert_temp(id_i, sel_i));
        ui.data_mut(|d| d.insert_temp(id_j, sel_j));

        // 追加実行（クロージャ外で app の可変借用を使う）
        if do_add {
            if let (Some(i_node), Some(j_node)) = (sel_i, sel_j) {
                let new_id = ElemId(app.model.elements.len() as u32);
                let elem = ElementData {
                    id: new_id,
                    kind: ElementKind::Beam,
                    nodes: [i_node, j_node].into_iter().collect(),
                    section: None,
                    material: None,
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                };
                app.undo.run(&mut app.model, Box::new(AddMember { elem }));
                app.staleness.mark_edited();
            }
        }

        // 免震支承材の作成フォームは未実装（仕様策定中）。ステータスバーに通知のみ。
        if do_isolator_notice {
            app.last_error = Some("免震支承材の作成フォームは未実装です（仕様策定中）".to_string());
        }
    }
    ui.separator();
    // ── ここまで梁追加フォーム ────────────────────────────────────

    let n = app.model.elements.len();
    let mut pending_section: Vec<(usize, u32)> = Vec::new();
    let mut pending_material: Vec<(usize, u32)> = Vec::new();
    let mut pending_delete: Option<ElemId> = None;

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(70.0))
        .column(Column::initial(80.0))
        .column(Column::initial(90.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "種別", "節点", "断面", "材料", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let elem = &app.model.elements[i];
                let is_focus = app.nav.focus_member == Some(elem.id);
                row.col(|ui| {
                    let text = elem.id.0.to_string();
                    // 選択行は blue-500 背景になるため文字は白、非選択は既定色
                    let rich = egui::RichText::new(text).color(if is_focus {
                        crate::theme::WHITE
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui.selectable_label(is_focus, rich).clicked() {
                        app.nav.focus_member = Some(elem.id);
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:?}", elem.kind));
                });
                row.col(|ui| {
                    let ids: Vec<String> = elem.nodes.iter().map(|n| n.0.to_string()).collect();
                    ui.label(ids.join(","));
                });
                row.col(|ui| {
                    let selected = elem.section.map(|s| s.0).unwrap_or(u32::MAX);
                    let combo = egui::ComboBox::from_id_salt(format!("elem_sec_{}", i))
                        .selected_text(
                            elem.section
                                .map(|s| format!("S{}", s.0))
                                .unwrap_or_else(|| "―".to_string()),
                        );
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                            pending_section.push((i, u32::MAX));
                        }
                        for sec in &app.model.sections {
                            if ui
                                .selectable_label(
                                    selected == sec.id.0,
                                    format!("S{} {}", sec.id.0, sec.name),
                                )
                                .clicked()
                            {
                                pending_section.push((i, sec.id.0));
                            }
                        }
                    });
                });
                row.col(|ui| {
                    let selected = elem.material.map(|m| m.0).unwrap_or(u32::MAX);
                    let combo = egui::ComboBox::from_id_salt(format!("elem_mat_{}", i))
                        .selected_text(
                            elem.material
                                .and_then(|m| app.model.materials.get(m.index()))
                                .map(|m| m.name.clone())
                                .unwrap_or_else(|| "―".to_string()),
                        );
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                            pending_material.push((i, u32::MAX));
                        }
                        for mat in &app.model.materials {
                            if ui
                                .selectable_label(selected == mat.id.0, &mat.name)
                                .clicked()
                            {
                                pending_material.push((i, mat.id.0));
                            }
                        }
                    });
                });
                row.col(|ui| {
                    if ui
                        .button("🗑")
                        .on_hover_text("部材を削除（関連する部材荷重も削除されます）")
                        .clicked()
                    {
                        pending_delete = Some(elem.id);
                    }
                });
            });
        });

    // 確定処理
    let had_pending =
        !pending_section.is_empty() || !pending_material.is_empty() || pending_delete.is_some();
    for (i, sec_id) in pending_section {
        let elem_id = app.model.elements[i].id;
        let section = if sec_id == u32::MAX {
            None
        } else {
            // 参照先が存在するか確認
            let sid = SectionId(sec_id);
            if app.model.sections.iter().any(|s| s.id == sid) {
                Some(sid)
            } else {
                None
            }
        };
        app.undo.run(
            &mut app.model,
            Box::new(SetElementSection {
                elem: elem_id,
                section,
            }),
        );
    }
    for (i, mat_id) in pending_material {
        let elem_id = app.model.elements[i].id;
        let material = if mat_id == u32::MAX {
            None
        } else {
            let mid = MaterialId(mat_id);
            if app.model.materials.iter().any(|m| m.id == mid) {
                Some(mid)
            } else {
                None
            }
        };
        app.undo.run(
            &mut app.model,
            Box::new(SetElementMaterial {
                elem: elem_id,
                material,
            }),
        );
    }
    if let Some(elem_id) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteMember { id: elem_id }));
        if app.nav.focus_member == Some(elem_id) {
            app.nav.focus_member = None;
        }
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_pending {
        app.staleness.mark_edited();
    }
}
