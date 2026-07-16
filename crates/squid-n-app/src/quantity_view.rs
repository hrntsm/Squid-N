//! 数量積算ビュー（設計タブ「数量積算」）。
//!
//! モデルから [`squid_n_design_jp::quantity::compute_quantity_takeoff`] で
//! 概算数量（コンクリート体積・型枠面積・鉄筋重量・鉄骨重量・鉄筋継手個所数）
//! を集計し、部位別・階別・明細・鉄骨種類別・鉄筋径別に表示する。
//! CSV エクスポート（[`crate::summary::build_quantity_csv`]）にも対応する。

use egui_extras::{Column, TableBuilder};
use squid_n_design_jp::quantity::{compute_quantity_takeoff, QuantityCfg, QuantityTotals};

use crate::app::App;

/// 集計単位の切替。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum QuantityGrouping {
    /// 部位別（柱・大梁・小梁・床・壁・ブレース…）の小計。
    #[default]
    ByCategory,
    /// 階別の小計。
    ByStory,
    /// 部材ごとの明細。
    Detail,
    /// 鉄骨の種類別（断面名別）長さ・重量。
    SteelBySection,
    /// 鉄筋の呼び径別長さ・重量。
    RebarByDia,
}

/// 数量積算ビューの状態。
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantityViewState {
    pub grouping: QuantityGrouping,
}

/// 数量積算パネルの描画。
pub fn quantity_panel(ui: &mut egui::Ui, app: &mut App) {
    let takeoff = compute_quantity_takeoff(&app.model, &QuantityCfg::default());

    if takeoff.items.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "集計対象の部材がありません。モデルタブで部材（断面・材料の割当）を作成してください。",
        );
        return;
    }

    // 集計単位の切替とエクスポート。
    ui.horizontal(|ui| {
        for (g, label) in [
            (QuantityGrouping::ByCategory, "部位別"),
            (QuantityGrouping::ByStory, "階別"),
            (QuantityGrouping::Detail, "明細"),
            (QuantityGrouping::SteelBySection, "鉄骨種類別"),
            (QuantityGrouping::RebarByDia, "鉄筋径別"),
        ] {
            if ui
                .selectable_label(app.quantity_view.grouping == g, label)
                .clicked()
            {
                app.quantity_view.grouping = g;
            }
        }
        ui.separator();
        if ui.button("💾 CSV エクスポート…").clicked() {
            let csv = crate::summary::build_quantity_csv(&app.model);
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("CSV", &["csv"])
                .set_file_name("quantity.csv")
                .save_file()
            {
                if let Err(e) = std::fs::write(&path, &csv) {
                    app.last_error = Some(format!("数量 CSV 保存エラー: {}", e));
                }
            }
        }
        if ui.button("📋 クリップボードへコピー").clicked() {
            ui.ctx()
                .copy_text(crate::summary::build_quantity_csv(&app.model));
        }
    });

    // 全体合計のサマリ行。
    let totals = takeoff.totals();
    ui.horizontal(|ui| {
        ui.label(format!("コンクリート {:.1} m³", totals.concrete_m3));
        ui.separator();
        ui.label(format!("型枠 {:.1} m²", totals.formwork_m2));
        ui.separator();
        ui.label(format!("鉄筋 {:.2} t", totals.rebar_t));
        ui.separator();
        ui.label(format!("鉄骨 {:.2} t", totals.steel_t));
        ui.separator();
        ui.label(format!("鉄筋継手 {:.1} 個所", totals.rebar_joints));
    });
    ui.separator();

    let mut focus: Option<squid_n_core::ids::ElemId> = None;
    egui::ScrollArea::vertical().show(ui, |ui| {
        match app.quantity_view.grouping {
            QuantityGrouping::ByCategory => {
                let rows = takeoff.totals_by_category();
                totals_table(
                    ui,
                    "部位",
                    rows.iter()
                        .map(|(c, t)| (c.label().to_string(), *t))
                        .collect(),
                    totals,
                );
            }
            QuantityGrouping::ByStory => {
                let rows = takeoff.totals_by_story();
                totals_table(ui, "階", rows, totals);
            }
            QuantityGrouping::SteelBySection => {
                let rows = takeoff.steel_by_section();
                TableBuilder::new(ui)
                    .striped(true)
                    .column(Column::initial(220.0))
                    .column(Column::initial(100.0))
                    .column(Column::initial(100.0))
                    .header(20.0, |mut h| {
                        for t in &["断面", "長さ [m]", "重量 [t]"] {
                            h.col(|ui| {
                                ui.strong(*t);
                            });
                        }
                    })
                    .body(|body| {
                        body.rows(18.0, rows.len(), |mut row| {
                            let it = &rows[row.index()];
                            row.col(|ui| {
                                ui.label(&it.section_name);
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.2}", it.length_m));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.3}", it.weight_t));
                            });
                        });
                    });
            }
            QuantityGrouping::RebarByDia => {
                let rows = takeoff.rebar_by_dia();
                TableBuilder::new(ui)
                    .striped(true)
                    .column(Column::initial(120.0))
                    .column(Column::initial(120.0))
                    .column(Column::initial(100.0))
                    .header(20.0, |mut h| {
                        for t in &["呼び径", "長さ [m]", "重量 [t]"] {
                            h.col(|ui| {
                                ui.strong(*t);
                            });
                        }
                    })
                    .body(|body| {
                        body.rows(18.0, rows.len(), |mut row| {
                            let (dia, len, w) = rows[row.index()];
                            row.col(|ui| {
                                if dia > 0.0 {
                                    ui.label(format!("D{:.0}", dia));
                                } else {
                                    ui.label("（鉄筋比概算）");
                                }
                            });
                            row.col(|ui| {
                                if len > 0.0 {
                                    ui.label(format!("{:.1}", len));
                                } else {
                                    ui.label("-");
                                }
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.3}", w));
                            });
                        });
                    });
            }
            QuantityGrouping::Detail => {
                TableBuilder::new(ui)
                    .striped(true)
                    .column(Column::initial(60.0)) // ID
                    .column(Column::initial(60.0)) // 階
                    .column(Column::initial(70.0)) // 部位
                    .column(Column::initial(50.0)) // 構造
                    .column(Column::initial(160.0)) // 符号
                    .column(Column::initial(110.0)) // コンクリート
                    .column(Column::initial(90.0)) // 型枠
                    .column(Column::initial(80.0)) // 鉄筋
                    .column(Column::initial(80.0)) // 鉄骨
                    .column(Column::initial(80.0)) // 継手
                    .header(20.0, |mut h| {
                        for t in &[
                            "ID",
                            "階",
                            "部位",
                            "構造",
                            "符号",
                            "ｺﾝｸﾘｰﾄ [m³]",
                            "型枠 [m²]",
                            "鉄筋 [t]",
                            "鉄骨 [t]",
                            "継手 [個所]",
                        ] {
                            h.col(|ui| {
                                ui.strong(*t);
                            });
                        }
                    })
                    .body(|body| {
                        body.rows(18.0, takeoff.items.len(), |mut row| {
                            let it = &takeoff.items[row.index()];
                            row.col(|ui| match it.elem {
                                Some(id) => {
                                    let is_focus = app.nav.focus_member == Some(id);
                                    if ui
                                        .selectable_label(is_focus, id.0.to_string())
                                        .on_hover_text("クリックで部材を選択")
                                        .clicked()
                                    {
                                        focus = Some(id);
                                    }
                                }
                                None => {
                                    ui.label(
                                        it.slab
                                            .map(|s| format!("S{}", s.0))
                                            .unwrap_or_else(|| "-".to_string()),
                                    );
                                }
                            });
                            row.col(|ui| {
                                ui.label(&it.story);
                            });
                            row.col(|ui| {
                                ui.label(it.category.label());
                            });
                            row.col(|ui| {
                                ui.label(it.structure.label());
                            });
                            row.col(|ui| {
                                ui.label(&it.label);
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.3}", it.concrete_m3));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.2}", it.formwork_m2));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.4}", it.rebar_weight_t()));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.4}", it.steel_weight_t()));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.1}", it.rebar_joints));
                            });
                        });
                    });
            }
        }

        // 前提・未対応事項。
        ui.add_space(8.0);
        egui::CollapsingHeader::new("注記（算定の前提・未対応事項）")
            .default_open(false)
            .show(ui, |ui| {
                for n in &takeoff.notes {
                    ui.label(format!("・{}", n));
                }
            });
    });

    if let Some(id) = focus {
        app.nav.focus_member = Some(id);
    }
}

/// 小計テーブル（部位別・階別で共用）＋合計行。
fn totals_table(
    ui: &mut egui::Ui,
    key_label: &str,
    rows: Vec<(String, QuantityTotals)>,
    totals: QuantityTotals,
) {
    let n = rows.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::initial(90.0))
        .column(Column::initial(110.0))
        .column(Column::initial(90.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .header(20.0, |mut h| {
            for t in &[
                key_label,
                "ｺﾝｸﾘｰﾄ [m³]",
                "型枠 [m²]",
                "鉄筋 [t]",
                "鉄骨 [t]",
                "継手 [個所]",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(18.0, n + 1, |mut row| {
                let i = row.index();
                if i < n {
                    let (name, t) = &rows[i];
                    row.col(|ui| {
                        ui.label(name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", t.concrete_m3));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", t.formwork_m2));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.3}", t.rebar_t));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.3}", t.steel_t));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", t.rebar_joints));
                    });
                } else {
                    row.col(|ui| {
                        ui.strong("合計");
                    });
                    row.col(|ui| {
                        ui.strong(format!("{:.2}", totals.concrete_m3));
                    });
                    row.col(|ui| {
                        ui.strong(format!("{:.2}", totals.formwork_m2));
                    });
                    row.col(|ui| {
                        ui.strong(format!("{:.3}", totals.rebar_t));
                    });
                    row.col(|ui| {
                        ui.strong(format!("{:.3}", totals.steel_t));
                    });
                    row.col(|ui| {
                        ui.strong(format!("{:.1}", totals.rebar_joints));
                    });
                }
            });
        });
}
