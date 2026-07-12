//! 終局検定（RESP-D「計算編 06 終局検定」）の結果表示ビュー。
//!
//! RC 矩形部材について、塑性理論式による終局せん断強度 Qsu・付着割裂耐力 Qbu と、
//! 両端ヒンジ時せん断力 Qmu との余裕度（Qsu/Qmu, Qbu/Qmu）を一覧表示する。
//! 算定本体は [`squid_n_design_jp::ultimate`]、部材内力（軸力）の収集は
//! [`crate::app::App::compute_ultimate_checks`] による。

use crate::app::App;
use egui_extras::{Column, TableBuilder};
use squid_n_design_jp::MemberKind;

/// 部材種別ラベル。
fn kind_label(kind: MemberKind) -> &'static str {
    match kind {
        MemberKind::Column => "柱",
        MemberKind::Beam => "梁",
        MemberKind::Brace => "斜材",
    }
}

/// 余裕度セルの色（1.0 未満＝せん断先行で NG を赤系に）。
fn margin_color(margin: f64) -> egui::Color32 {
    if margin.is_finite() {
        // status_color は「需要/耐力」を受けるため、逆数（=Qmu/Qsu）を渡す。
        crate::theme::status_color(if margin > 1e-9 { 1.0 / margin } else { 9.9 })
    } else {
        crate::theme::GOOD_GREEN
    }
}

/// 終局検定表を描画する（設計タブ「終局検定」サブビュー）。
pub fn ultimate_table(ui: &mut egui::Ui, app: &mut App) {
    ui.strong("終局検定（RESP-D 06：塑性理論式による終局せん断・付着余裕度）");
    ui.add_space(4.0);

    // ── 算定条件 ─────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("ヒンジ回転角 Rp:");
        ui.add(
            egui::DragValue::new(&mut app.ultimate_rp)
                .speed(0.001)
                .range(0.0..=0.1)
                .fixed_decimals(3),
        )
        .on_hover_text(
            "終局限界状態でのヒンジ領域の回転角 Rp [rad]。ν=(1−15Rp)ν0、\
             cotφ=2−50Rp に効きます。0 で塑性化前（cotφ=2.0, ν=ν0）の終局強度。",
        );
        ui.separator();
        ui.label("上限強度倍率:");
        ui.add(
            egui::DragValue::new(&mut app.ultimate_upper_factor)
                .speed(0.05)
                .range(0.1..=2.0)
                .fixed_decimals(2),
        )
        .on_hover_text("Qmu = 上限強度倍率·2·Mu/内法 の倍率。");
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.ultimate_lightweight,
            "軽量コンクリート（Qsu・Qbu を 0.9 倍）",
        )
        .on_hover_text("軽量コンクリート使用時のせん断終局耐力 0.9 倍低減（共通事項）。");
        ui.checkbox(&mut app.ultimate_include_bond, "付着割裂 Qbu を検定");
    });
    ui.horizontal(|ui| {
        ui.label("終局せん断強度:");
        ui.selectable_value(&mut app.ultimate_shear_ductility, false, "塑性理論式(Qsu)")
            .on_hover_text(
                "藤井・森田式系の塑性理論式（終局強度型設計指針）で終局せん断強度を算定します。",
            );
        ui.selectable_value(&mut app.ultimate_shear_ductility, true, "靭性指針式(Vu)")
            .on_hover_text(
                "AIJ「靭性保証型耐震設計指針」6.4 の Vu=min(Vu1,Vu2,Vu3)（トラス＋アーチ機構）\
                 で終局せん断信頼強度を算定します。Qsu 列に Vu を表示します。",
            );
    });
    ui.horizontal(|ui| {
        ui.label("柱 Mu 算定:");
        ui.selectable_value(&mut app.ultimate_mu_aci, false, "構造規定式(at式)");
        ui.selectable_value(&mut app.ultimate_mu_aci, true, "ACI規準(平面保持)")
            .on_hover_text("ACI318 等価応力度ブロック法による平面保持解析で柱の Mu を算定します。");
        ui.separator();
        ui.checkbox(&mut app.ultimate_biaxial_shear, "柱を2軸せん断で検定")
            .on_hover_text(
                "RC 柱のせん断余裕度を 2 軸せん断 1/((Qmx/Qux)²+(Qmy/Quy)²)^(1/2) として\
                 検定します（採用応力）。Qsu/Qmu 列は 2 軸合成値を表示します。",
            );
        ui.checkbox(&mut app.ultimate_biaxial_bending, "柱を2軸曲げで検定")
            .on_hover_text(
                "RC 柱の曲げ余裕度を 2 軸曲げ 1/((Mmx/Mux)²+(Mmy/Muy)²)^(1/2) として検定します\
                 （採用応力）。需要曲げは応答（プッシュオーバー／静的）の値を用います。",
            );
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.ultimate_use_pushover,
            "設計用応力・Rp をプッシュオーバー応答から反映",
        )
        .on_hover_text(
            "終局時の設計用せん断 Qmu・需要曲げ・軸力・部材別 Rp をプッシュオーバー\
             最終ステップの部材別応答から直接反映します。未実行時は静的応答＋UI 一律 Rp \
             にフォールバックします。",
        );
        if app.ultimate_use_pushover {
            let has_po = app
                .results
                .as_ref()
                .and_then(|r| r.pushover.as_ref())
                .map(|p| !p.member_response.is_empty())
                .unwrap_or(false);
            if has_po {
                ui.colored_label(crate::theme::GOOD_GREEN, "● 応答反映中");
            } else {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "（プッシュオーバー未実行 → 静的応答で代替）",
                );
            }
        }
    });
    ui.separator();

    match app.compute_ultimate_checks() {
        Err(msg) => {
            ui.colored_label(crate::theme::GRAY_600, &msg);
            let needs_analysis = msg.contains("静的") || msg.contains("解析");
            if needs_analysis && ui.button("▶ 解析タブへ").clicked() {
                app.active_tab = crate::app::Tab::Analysis;
            }
        }
        Ok(checks) => {
            let ng = checks.iter().filter(|c| !c.ok).count();
            ui.horizontal(|ui| {
                ui.label(format!("対象部材 {} 本", checks.len()));
                if ng > 0 {
                    ui.colored_label(crate::theme::ERROR_RED, format!("NG {} 本", ng));
                } else {
                    ui.colored_label(crate::theme::GOOD_GREEN, "全部材 OK");
                }
            });
            ui.add_space(4.0);

            let bond = app.ultimate_include_bond;
            // 終局せん断強度の列見出し（靭性指針式は Vu／付着考慮 Vbu 表記）。
            let (qsu_hdr, ratio_hdr, qbu_hdr, bond_ratio_hdr) = if app.ultimate_shear_ductility {
                ("Vu[kN]", "Vu/Qmu", "Vbu[kN]", "Vbu/Qmu")
            } else {
                ("Qsu[kN]", "Qsu/Qmu", "Qbu[kN]", "Qbu/Qmu")
            };
            TableBuilder::new(ui)
                .id_salt("ultimate_checks")
                .striped(true)
                .column(Column::auto())
                .column(Column::initial(48.0))
                .column(Column::initial(90.0))
                .column(Column::initial(80.0))
                .column(Column::initial(80.0))
                .column(Column::initial(72.0))
                .column(Column::initial(80.0))
                .column(Column::initial(72.0))
                .column(Column::initial(50.0))
                .header(20.0, |mut h| {
                    for t in &[
                        "部材",
                        "種別",
                        "Mu[kN·m]",
                        "Qmu[kN]",
                        qsu_hdr,
                        ratio_hdr,
                        qbu_hdr,
                        bond_ratio_hdr,
                        "判定",
                    ] {
                        h.col(|ui| {
                            ui.strong(*t);
                        });
                    }
                })
                .body(|body| {
                    body.rows(18.0, checks.len(), |mut row| {
                        let i = row.index();
                        let c = &checks[i];
                        row.col(|ui| {
                            ui.label(format!("{}", c.elem.0)).on_hover_text(&c.detail);
                        });
                        row.col(|ui| {
                            ui.label(kind_label(c.kind));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", c.mu / 1.0e6));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", c.qmu / 1000.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", c.qsu / 1000.0));
                        });
                        row.col(|ui| {
                            // 2 軸せん断指定時は合成余裕度を表示（柱のみ Some）。
                            let m = c.biaxial_shear_margin.unwrap_or(c.shear_margin);
                            ui.colored_label(margin_color(m), format!("{m:.2}"));
                        });
                        row.col(|ui| {
                            if bond {
                                ui.label(format!("{:.1}", c.qbu / 1000.0));
                            } else {
                                ui.label("-");
                            }
                        });
                        row.col(|ui| {
                            if bond {
                                ui.colored_label(
                                    margin_color(c.bond_margin),
                                    format!("{:.2}", c.bond_margin),
                                );
                            } else {
                                ui.label("-");
                            }
                        });
                        row.col(|ui| {
                            if c.ok {
                                ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                            } else {
                                ui.colored_label(crate::theme::ERROR_RED, "NG");
                            }
                        });
                    });
                });

            ui.add_space(4.0);
            let (shear_note, bond_note) = if app.ultimate_shear_ductility {
                (
                    "Vu=靭性指針式の終局せん断信頼強度 min(Vu1,Vu2,Vu3)（トラス＋アーチ機構）",
                    "Vbu=付着考慮せん断信頼強度 min(Vbu1,Vbu2)",
                )
            } else {
                ("Qsu=塑性理論式の終局せん断強度", "Qbu=付着割裂耐力")
            };
            let using_po = app.ultimate_use_pushover
                && app
                    .results
                    .as_ref()
                    .and_then(|r| r.pushover.as_ref())
                    .map(|p| !p.member_response.is_empty())
                    .unwrap_or(false);
            let demand_note = if using_po {
                "Qmu・需要曲げ・軸力・Rp はプッシュオーバー終局応答（部材別）を直接反映"
            } else {
                "Qmu=上限強度倍率·2·Mu/内法（両端ヒンジ）、需要曲げ・軸力は静的応答、Rp は一律指定"
            };
            ui.colored_label(
                crate::theme::GRAY_600,
                format!(
                    "{demand_note}。{shear_note}、{bond_note}。\
                     余裕度<1.0（赤）はせん断・付着が曲げ降伏に先行することを示す。\
                     対象は RcRect の RC 矩形部材（強軸）。"
                ),
            );
        }
    }

    // ── CFT 柱の軸終局耐力（RESP-D「06 終局検定」CFT）─────────────
    ui.add_space(12.0);
    ui.strong("CFT柱の軸終局耐力（RESP-D 06：CFT指針）");
    ui.add_space(4.0);
    match app.compute_cft_ultimate_checks() {
        Err(msg) => {
            ui.colored_label(crate::theme::GRAY_600, &msg);
        }
        Ok(checks) => {
            let ng = checks.iter().filter(|c| !c.ok).count();
            ui.horizontal(|ui| {
                ui.label(format!("対象 CFT 柱 {} 本", checks.len()));
                if ng > 0 {
                    ui.colored_label(crate::theme::ERROR_RED, format!("NG {} 本", ng));
                } else {
                    ui.colored_label(crate::theme::GOOD_GREEN, "全柱 OK");
                }
            });
            ui.add_space(4.0);
            TableBuilder::new(ui)
                .id_salt("cft_ultimate_checks")
                .striped(true)
                .column(Column::auto())
                .column(Column::initial(48.0))
                .column(Column::initial(90.0))
                .column(Column::initial(90.0))
                .column(Column::initial(100.0))
                .column(Column::initial(90.0))
                .column(Column::initial(72.0))
                .column(Column::initial(50.0))
                .header(20.0, |mut h| {
                    for t in &[
                        "部材",
                        "分類",
                        "Ncu[kN]",
                        "Ntu[kN]",
                        "Mu(N-M)[kN·m]",
                        "N[kN]",
                        "軸余裕度",
                        "判定",
                    ] {
                        h.col(|ui| {
                            ui.strong(*t);
                        });
                    }
                })
                .body(|body| {
                    body.rows(18.0, checks.len(), |mut row| {
                        let i = row.index();
                        let c = &checks[i];
                        row.col(|ui| {
                            ui.label(format!("{}", c.elem.0)).on_hover_text(&c.detail);
                        });
                        row.col(|ui| {
                            ui.label(cft_class_label(c.class));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", c.ncu / 1000.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", c.ntu / 1000.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", c.mu_nm / 1.0e6));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", c.n_design / 1000.0));
                        });
                        row.col(|ui| {
                            ui.colored_label(
                                margin_color(c.axial_margin),
                                format!("{:.2}", c.axial_margin),
                            );
                        });
                        row.col(|ui| {
                            if c.ok {
                                ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                            } else {
                                ui.colored_label(crate::theme::ERROR_RED, "NG");
                            }
                        });
                    });
                });
            ui.add_space(4.0);
            ui.colored_label(
                crate::theme::GRAY_600,
                "Ncu=軸圧縮終局耐力（短柱=cNc+(1+ξ)sNc、長柱=座屈耐力、中柱=線形補間）、\
                 Ntu=軸引張終局耐力（sA·Fy）。N は長期軸力（圧縮正）。座屈長さは幾何長（K=1）。",
            );
        }
    }
}

/// CFT 柱分類のラベル。
fn cft_class_label(class: squid_n_design_jp::ultimate::CftColumnClass) -> &'static str {
    use squid_n_design_jp::ultimate::CftColumnClass;
    match class {
        CftColumnClass::Short => "短柱",
        CftColumnClass::Medium => "中柱",
        CftColumnClass::Long => "長柱",
    }
}
