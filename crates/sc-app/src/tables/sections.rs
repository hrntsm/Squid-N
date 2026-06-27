use crate::app::App;
use sc_core::ids::SectionId;
use sc_edit::{SectionField, SetSectionField, SetSectionName};

pub fn sections_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    let n = app.model.sections.len();
    let mut pending_name: Vec<(usize, String)> = Vec::new();
    let mut pending_field: Vec<(usize, SectionField, f64)> = Vec::new();

    // 編集バッファ（名称）
    let mut name_buf: Vec<String> = app.model.sections.iter().map(|s| s.name.clone()).collect();
    // 編集バッファ（数値フィールド）
    let mut num_bufs: Vec<[String; 7]> = app
        .model
        .sections
        .iter()
        .map(|s| {
            [
                format!("{:.1}", s.area),
                format!("{:.1}", s.iy),
                format!("{:.1}", s.iz),
                format!("{:.1}", s.j),
                format!("{:.1}", s.depth),
                format!("{:.1}", s.width),
                format!("{:.1}", s.as_y),
            ]
        })
        .collect();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(100.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "A", "Iy", "Iz", "J"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let sec = &app.model.sections[i];
                row.col(|ui| {
                    ui.label(sec.id.0.to_string());
                });
                row.col(|ui| {
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut name_buf[i])
                            .desired_width(90.0)
                            .clip_text(false),
                    );
                    if resp.lost_focus() && resp.changed() {
                        let trimmed = name_buf[i].trim().to_string();
                        if trimmed != sec.name && !trimmed.is_empty() {
                            pending_name.push((i, trimmed));
                        }
                    }
                });
                // A, Iy, Iz, J
                let fields = [
                    SectionField::Area,
                    SectionField::Iy,
                    SectionField::Iz,
                    SectionField::J,
                ];
                for (k, field) in fields.iter().enumerate() {
                    row.col(|ui| {
                        let buf = &mut num_bufs[i][k];
                        let resp = ui.add(
                            egui::TextEdit::singleline(buf)
                                .desired_width(70.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            if let Ok(val) = buf.trim().parse::<f64>() {
                                let old = match field {
                                    SectionField::Area => sec.area,
                                    SectionField::Iy => sec.iy,
                                    SectionField::Iz => sec.iz,
                                    SectionField::J => sec.j,
                                    _ => 0.0,
                                };
                                if (val - old).abs() > 1e-9 {
                                    pending_field.push((i, *field, val));
                                }
                            }
                        }
                        if buf.trim().parse::<f64>().is_err() {
                            ui.painter().rect_filled(
                                resp.rect,
                                0.0,
                                crate::theme::translucent(crate::theme::ERROR_RED, 60),
                            );
                        }
                    });
                }
            });
        });

    // 確定処理
    let (had_name, had_field) = (!pending_name.is_empty(), !pending_field.is_empty());
    for (i, name) in pending_name {
        let sid = SectionId(app.model.sections[i].id.0);
        app.undo
            .run(&mut app.model, Box::new(SetSectionName { id: sid, name }));
    }
    for (i, field, val) in pending_field {
        let sid = SectionId(app.model.sections[i].id.0);
        app.undo.run(
            &mut app.model,
            Box::new(SetSectionField {
                id: sid,
                field,
                value: val,
            }),
        );
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_name || had_field {
        app.staleness.mark_edited();
    }
}
