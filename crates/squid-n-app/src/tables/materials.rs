use crate::app::App;
use squid_n_edit::{AddMaterial, DeleteMaterial, MaterialField, SetMaterialField, SetMaterialName};

/// (名称, E [N/mm²], ν, 密度 [ton/mm³], Fc, Fy)
type MaterialPreset = (&'static str, f64, f64, f64, Option<f64>, Option<f64>);

/// コンクリートの質量密度 [ton/mm³] を単位体積重量表（γC/γRC/γSRC）から導出する。
///
/// レビュー §1.9: 旧実装は Fc・構造区分によらず `2.4e-9`（γ≈23.5 kN/m³）固定
/// だったが、マニュアルの表は Fc・普通/軽量・無筋/RC/SRC ごとに
/// 17.0〜26.5 kN/m³ の範囲で規定する。普通コンクリート・Fc≤36 の場合、
/// 気乾単位体積重量 γC=23.0、鉄筋込み γRC=γC+1.0=24.0、
/// 鉄骨鉄筋込み γSRC=γC+2.0=25.0 kN/m³（`squid_n_core::units::concrete_unit_weight_kn_m3`
/// の表そのもの）。Fc21/24/30 はいずれも Fc≤36 帯のため γRC/γSRC は Fc に依らず
/// 同一値になる（Fc>36 で初めて変化する）。
fn concrete_density(fc: f64, comp: squid_n_core::units::ConcreteComposition) -> f64 {
    use squid_n_core::units::{concrete_unit_weight_kn_m3, to_internal, ConcreteClass};
    let gamma = concrete_unit_weight_kn_m3(fc, ConcreteClass::Normal, comp);
    to_internal::mass_density_from_unit_weight_kn_m3(gamma)
}

/// 材料プリセット（JIS 主要鋼種と普通/SRC コンクリート）を実行時に生成する。
/// コンクリートの密度は `concrete_density`（γ表からの導出）を用いるため
/// `const` にできず、呼び出しのたびに構築する（UI 描画1回あたり数件のみで
/// コストは無視できる）。密度は内部単位系 N-mm-s の質量密度 [ton/mm³]
/// （鋼は慣用値 γs=77kN/m³ 相当の 7.85e-9 を維持）。
fn material_presets() -> Vec<MaterialPreset> {
    use squid_n_core::units::ConcreteComposition::{Rc, Src};
    vec![
        ("SN400B", 205000.0, 0.3, 7.85e-9, None, Some(235.0)),
        ("SS400", 205000.0, 0.3, 7.85e-9, None, Some(235.0)),
        ("SM490A", 205000.0, 0.3, 7.85e-9, None, Some(325.0)),
        (
            "Fc21",
            21500.0,
            0.2,
            concrete_density(21.0, Rc),
            Some(21.0),
            None,
        ),
        (
            "Fc24",
            22700.0,
            0.2,
            concrete_density(24.0, Rc),
            Some(24.0),
            None,
        ),
        (
            "Fc30",
            24800.0,
            0.2,
            concrete_density(30.0, Rc),
            Some(30.0),
            None,
        ),
        // SRC 造（鉄骨鉄筋コンクリート）用: γSRC = γC + 2.0（レビュー §1.9）。
        (
            "Fc24(SRC)",
            22700.0,
            0.2,
            concrete_density(24.0, Src),
            Some(24.0),
            None,
        ),
    ]
}

/// 材料タブ：プリセット追加・カスタム追加・一覧編集・削除。
pub fn materials_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // ── プリセット追加 ─────────────────────────────────────────
    ui.label("プリセット追加:");
    ui.horizontal_wrapped(|ui| {
        for (name, e, nu, rho, fc, fy) in material_presets() {
            if ui.button(name).clicked() {
                app.undo.run(
                    &mut app.model,
                    Box::new(AddMaterial {
                        name: name.to_string(),
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

#[cfg(test)]
mod tests {
    use super::*;

    /// レビュー §1.9: コンクリートプリセットの密度が単位体積重量表（γ表）から
    /// 導出されていることを確認する（Fc・種別に応じた 24.0/25.0 kN/m³ 等）。
    #[test]
    fn test_concrete_presets_match_unit_weight_table() {
        use squid_n_core::units::to_internal::mass_density_from_unit_weight_kn_m3;

        let presets = material_presets();
        let find = |name: &str| {
            presets
                .iter()
                .find(|p| p.0 == name)
                .unwrap_or_else(|| panic!("preset {name} not found"))
                .clone()
        };

        // Fc21/24/30 は全て Fc<=36 帯のため γRC=24.0 kN/m³ で共通。
        let rc_density = mass_density_from_unit_weight_kn_m3(24.0);
        for name in ["Fc21", "Fc24", "Fc30"] {
            let p = find(name);
            assert!(
                (p.3 - rc_density).abs() < 1e-15,
                "{name}: density={} expected={}",
                p.3,
                rc_density
            );
        }

        // SRC 用プリセットは γSRC=25.0 kN/m³。
        let src_density = mass_density_from_unit_weight_kn_m3(25.0);
        let src = find("Fc24(SRC)");
        assert!((src.3 - src_density).abs() < 1e-15);

        // 旧実装の固定値 2.4e-9 より正しい値の方が大きい（γ=24.0 が実際の表の値）。
        assert!(rc_density > 2.4e-9);
        assert!(
            (rc_density - 2.4473e-9).abs() < 1e-13,
            "rc_density={rc_density}"
        );

        // 鋼材の密度は据え置き。
        let steel = find("SN400B");
        assert!((steel.3 - 7.85e-9).abs() < 1e-18);
    }
}
