use crate::app::App;

pub fn design_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    let checks = app
        .results
        .as_ref()
        .map(|r| r.checks.as_slice())
        .unwrap_or(&[]);

    let n = checks.len();
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
                let (elem_id, pos, cr) = &checks[i];
                row.col(|ui| {
                    ui.label(elem_id.0.to_string());
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", pos));
                });
                let ratio_color = crate::theme::status_color(cr.ratio);
                row.col(|ui| {
                    ui.colored_label(ratio_color, format!("{:.4}", cr.ratio));
                });
                row.col(|ui| {
                    ui.label(if cr.ok { "OK" } else { "NG" });
                });
                row.col(|ui| {
                    ui.label(&cr.basis);
                });
            });
        });
}
