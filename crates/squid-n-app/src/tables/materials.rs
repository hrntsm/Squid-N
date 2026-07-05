use crate::app::App;
use squid_n_edit::{AddMaterial, DeleteMaterial, MaterialField, SetMaterialField, SetMaterialName};

/// (名称, E [N/mm²], ν, 密度 [ton/mm³], Fc, Fy)
type MaterialPreset = (&'static str, f64, f64, f64, Option<f64>, Option<f64>);

/// 材料プリセット（JIS 主要鋼種と普通コンクリート）。
/// 密度は内部単位系 N-mm-s の質量密度 [ton/mm³]（鋼 7.85e-9、RC 2.4e-9）。
const PRESETS: &[MaterialPreset] = &[
    ("SN400B", 205000.0, 0.3, 7.85e-9, None, Some(235.0)),
    ("SS400", 205000.0, 0.3, 7.85e-9, None, Some(235.0)),
    ("SM490A", 205000.0, 0.3, 7.85e-9, None, Some(325.0)),
    ("Fc21", 21500.0, 0.2, 2.4e-9, Some(21.0), None),
    ("Fc24", 22700.0, 0.2, 2.4e-9, Some(24.0), None),
    ("Fc30", 24800.0, 0.2, 2.4e-9, Some(30.0), None),
];

/// 材料タブ：プリセット追加・カスタム追加・一覧編集・削除。
pub fn materials_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // ── プリセット追加 ─────────────────────────────────────────
    ui.label("プリセット追加:");
    ui.horizontal_wrapped(|ui| {
        for (name, e, nu, rho, fc, fy) in PRESETS {
            if ui.button(*name).clicked() {
                app.undo.run(
                    &mut app.model,
                    Box::new(AddMaterial {
                        name: name.to_string(),
                        young: *e,
                        poisson: *nu,
                        density: *rho,
                        fc: *fc,
                        fy: *fy,
                    }),
                );
                app.staleness.mark_edited();
            }
        }
    });

    // ── カスタム追加フォーム ────────────────────────────────────
    let id_draft = egui::Id::new("material_custom_draft");
    // (名称, E, ν, 密度, Fc, Fy) の文字列ドラフト
    let mut draft: [String; 6] = ui
        .data(|d| d.get_temp::<[String; 6]>(id_draft))
        .unwrap_or_else(|| {
            [
                "新規材料".into(),
                "205000".into(),
                "0.3".into(),
                "7.85e-9".into(),
                String::new(),
                String::new(),
            ]
        });
    let mut do_add_custom = false;
    ui.horizontal(|ui| {
        ui.label("カスタム:");
        ui.add(egui::TextEdit::singleline(&mut draft[0]).desired_width(80.0))
            .on_hover_text("名称");
        for (k, label) in [(1, "E"), (2, "ν"), (3, "ρ"), (4, "Fc"), (5, "Fy")] {
            ui.label(label);
            ui.add(egui::TextEdit::singleline(&mut draft[k]).desired_width(60.0));
        }
        let parsed_e = draft[1].parse::<f64>();
        let parsed_nu = draft[2].parse::<f64>();
        let parsed_rho = draft[3].parse::<f64>();
        let ok = parsed_e.is_ok() && parsed_nu.is_ok() && parsed_rho.is_ok();
        if ui
            .add_enabled(ok, egui::Button::new("+ 追加"))
            .on_hover_text("E・ν・ρ は必須。Fc・Fy は空欄可")
            .clicked()
        {
            do_add_custom = true;
        }
    });
    if do_add_custom {
        let fc = draft[4].parse::<f64>().ok();
        let fy = draft[5].parse::<f64>().ok();
        if let (Ok(e), Ok(nu), Ok(rho)) = (
            draft[1].parse::<f64>(),
            draft[2].parse::<f64>(),
            draft[3].parse::<f64>(),
        ) {
            app.undo.run(
                &mut app.model,
                Box::new(AddMaterial {
                    name: draft[0].clone(),
                    young: e,
                    poisson: nu,
                    density: rho,
                    fc,
                    fy,
                }),
            );
            app.staleness.mark_edited();
        }
    }
    ui.data_mut(|d| d.insert_temp(id_draft, draft));
    ui.separator();

    // ── 一覧テーブル（編集・削除） ──────────────────────────────
    let n = app.model.materials.len();
    ui.label(format!("材料一覧（{} 件）", n));
    let mut pending_name: Option<(u32, String)> = None;
    let mut pending_field: Option<(u32, MaterialField, Option<f64>)> = None;
    let mut pending_delete: Option<u32> = None;

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(90.0))
        .column(Column::initial(70.0))
        .column(Column::initial(45.0))
        .column(Column::initial(70.0))
        .column(Column::initial(50.0))
        .column(Column::initial(50.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "E [N/mm²]", "ν", "ρ [t/mm³]", "Fc", "Fy", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let idx = row.index();
                let mat = &app.model.materials[idx];
                let mat_id = mat.id;
                row.col(|ui| {
                    ui.label(format!("{}", mat_id.0));
                });
                row.col(|ui| {
                    let mut name = mat.name.clone();
                    if ui
                        .add(egui::TextEdit::singleline(&mut name).desired_width(85.0))
                        .lost_focus()
                        && name != mat.name
                    {
                        pending_name = Some((mat_id.0, name));
                    }
                });
                // 数値セル: フォーカス喪失時に確定
                let cells: [(MaterialField, String, bool); 5] = [
                    (MaterialField::Young, format!("{}", mat.young), true),
                    (MaterialField::Poisson, format!("{}", mat.poisson), true),
                    (MaterialField::Density, format!("{:.3e}", mat.density), true),
                    (
                        MaterialField::Fc,
                        mat.fc.map(|v| format!("{}", v)).unwrap_or_default(),
                        false,
                    ),
                    (
                        MaterialField::Fy,
                        mat.fy.map(|v| format!("{}", v)).unwrap_or_default(),
                        false,
                    ),
                ];
                for (field, current, required) in cells {
                    row.col(|ui| {
                        let cell_id = egui::Id::new(("mat_cell", mat_id.0, field as u8));
                        let mut buf = ui
                            .data(|d| d.get_temp::<String>(cell_id))
                            .unwrap_or_else(|| current.clone());
                        let resp = ui.add(egui::TextEdit::singleline(&mut buf).desired_width(60.0));
                        if resp.lost_focus() {
                            let parsed = buf.trim().parse::<f64>().ok();
                            let changed = buf.trim() != current.trim();
                            if changed && (parsed.is_some() || !required) {
                                pending_field = Some((mat_id.0, field, parsed));
                            }
                            ui.data_mut(|d| d.remove::<String>(cell_id));
                        } else if resp.has_focus() {
                            ui.data_mut(|d| d.insert_temp(cell_id, buf));
                        }
                    });
                }
                row.col(|ui| {
                    let in_use = app
                        .model
                        .elements
                        .iter()
                        .any(|e| e.material == Some(mat_id));
                    let btn = ui.add_enabled(!in_use, egui::Button::new("🗑"));
                    if in_use {
                        btn.on_hover_text("部材から参照中のため削除できません");
                    } else if btn.clicked() {
                        pending_delete = Some(mat_id.0);
                    }
                });
            });
        });

    // 確定処理（テーブル描画後に model を可変借用）
    let mut edited = false;
    if let Some((id, name)) = pending_name {
        app.undo.run(
            &mut app.model,
            Box::new(SetMaterialName {
                id: squid_n_core::ids::MaterialId(id),
                name,
            }),
        );
        edited = true;
    }
    if let Some((id, field, value)) = pending_field {
        app.undo.run(
            &mut app.model,
            Box::new(SetMaterialField {
                id: squid_n_core::ids::MaterialId(id),
                field,
                value,
            }),
        );
        edited = true;
    }
    if let Some(id) = pending_delete {
        app.undo.run(
            &mut app.model,
            Box::new(DeleteMaterial {
                id: squid_n_core::ids::MaterialId(id),
            }),
        );
        edited = true;
    }
    if edited {
        app.staleness.mark_edited();
    }
}
