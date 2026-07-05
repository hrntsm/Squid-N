use crate::app::App;

pub fn design_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // ── 一次設計: 部材検定表 ─────────────────────────────────────
    ui.strong("部材検定（許容応力度）");
    if app.staleness.design_stale {
        ui.colored_label(
            crate::theme::BEST_YELLOW,
            "⚠ モデルが編集されました。解析を再実行してください。",
        );
    }
    let checks: Vec<(squid_n_core::ids::ElemId, f64, f64, bool, String)> = app
        .results
        .as_ref()
        .map(|r| {
            r.checks
                .iter()
                .map(|(id, pos, cr)| (*id, *pos, cr.ratio, cr.ok, cr.basis.clone()))
                .collect()
        })
        .unwrap_or_default();

    if checks.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "検定結果がありません。解析タブから静的解析を実行してください（部材に断面と材料の割当が必要です）。",
        );
    } else {
        let ng_count = checks.iter().filter(|(_, _, _, ok, _)| !ok).count();
        ui.label(format!(
            "{} 位置を検定、NG {} 件（部材IDクリックで 3D ビューにハイライト）",
            checks.len(),
            ng_count
        ));
    }

    let n = checks.len();
    let mut focus: Option<squid_n_core::ids::ElemId> = None;
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(80.0))
        .column(Column::initial(60.0))
        .column(Column::initial(200.0))
        .header(20.0, |mut h| {
            for t in &["部材", "位置", "検定比", "判定", "根拠"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();
                let (elem_id, pos, ratio, ok, basis) = &checks[i];
                let is_focus = app.nav.focus_member == Some(*elem_id);
                row.col(|ui| {
                    if ui
                        .selectable_label(is_focus, elem_id.0.to_string())
                        .on_hover_text("クリックで部材を選択（結果タブの3Dビューで確認できます）")
                        .clicked()
                    {
                        focus = Some(*elem_id);
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", pos));
                });
                let ratio_color = crate::theme::status_color(*ratio);
                row.col(|ui| {
                    ui.colored_label(ratio_color, format!("{:.4}", ratio));
                });
                row.col(|ui| {
                    if *ok {
                        ui.label("OK");
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, "NG");
                    }
                });
                row.col(|ui| {
                    ui.label(basis);
                });
            });
        });
    if let Some(id) = focus {
        app.nav.focus_member = Some(id);
    }

    // ── 二次設計: 層指標（層間変形角・剛性率・偏心率） ────────────
    ui.add_space(12.0);
    ui.strong("層指標（二次設計: 層間変形角・剛性率・偏心率）");
    if app.model.stories.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "階が未定義です。解析タブの「階の自動生成」を実行してください。",
        );
        return;
    }
    let Some(st) = app
        .results
        .as_ref()
        .and_then(|r| r.statics.last())
        .map(|(_, st)| st)
    else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "静的解析結果がありません。地震静的(Ai)を実行すると層指標を評価できます。",
        );
        return;
    };
    let metrics =
        crate::summary::compute_story_metrics(&app.model, &st.disp, app.analysis_cfg.seismic_dir);

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(70.0))
        .column(Column::initial(80.0))
        .column(Column::initial(90.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(60.0))
        .header(20.0, |mut h| {
            for t in &[
                "階",
                "階高[mm]",
                "層間変位[mm]",
                "変形角(1/200)",
                "剛性率Rs(≥0.6)",
                "偏心率Re(≤0.15)",
                "Fes",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, metrics.len(), |mut row| {
                let m = &metrics[row.index()];
                row.col(|ui| {
                    ui.label(&m.name);
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", m.height));
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", m.drift));
                });
                row.col(|ui| {
                    let txt = if m.drift_angle > 1e-12 {
                        format!("1/{:.0}", 1.0 / m.drift_angle)
                    } else {
                        "0".to_string()
                    };
                    if m.drift_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    let txt = format!("{:.3}", m.rs);
                    if m.rs_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    let txt = format!("{:.3}", m.re);
                    if m.re_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", m.fes));
                });
            });
        });
}
