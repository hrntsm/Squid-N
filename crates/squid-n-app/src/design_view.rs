use crate::app::App;

/// 柱の積載荷重低減（令85条2項）の参考表示。
/// `Model.load_cfg.live_load_reduction == true` のときのみ表示する。
/// 支持床数・低減率の集計は `crate::app::column_live_load_factors` による。
/// **検定の長期軸力への実適用は残課題**（表示のみ。荷重計算条件のツールチップにも明記）。
fn live_load_reduction_section(ui: &mut egui::Ui, app: &App) {
    if !app
        .model
        .load_cfg
        .as_ref()
        .is_some_and(|c| c.live_load_reduction)
    {
        return;
    }
    egui::CollapsingHeader::new("柱の積載荷重低減（令85条2項・参考表示）")
        .id_salt("live_load_reduction_section")
        .default_open(true)
        .show(ui, |ui| {
            ui.colored_label(
                crate::theme::GRAY_600,
                "支える床の数に応じた低減率の集計値です。断面検定の長期軸力への実適用は未対応（残課題）。",
            );
            let factors = crate::app::column_live_load_factors(&app.model);
            if factors.is_empty() {
                ui.label("柱要素（鉛直材）がありません。階の自動生成後に所属階が設定されると床数を集計できます。");
                return;
            }
            for (elem, floors, factor) in factors {
                ui.label(format!(
                    "柱#{}: 支持床数 {} → 低減率 {:.2}",
                    elem.0, floors, factor
                ));
            }
        });
    ui.add_space(6.0);
}

pub fn design_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    live_load_reduction_section(ui, app);

    // ── 一次設計: 部材検定表 ─────────────────────────────────────
    ui.strong("部材検定（許容応力度）");
    // 断面算定条件（許容応力度設計・令82条）。
    ui.horizontal(|ui| {
        let mut changed = false;
        changed |= ui
            .checkbox(
                &mut app.analysis_cfg.rc_damage_control,
                "RC短期せん断: 損傷制御",
            )
            .on_hover_text(
                "ON: 損傷制御のための検討（2/3・α・fs）、OFF: 安全確保のための検討。\
                 軽量コンクリート×高強度せん断補強筋は常に安全確保式（RC規準）",
            )
            .changed();
        ui.label("QD:");
        for (m, label) in [
            (squid_n_design_jp::QdMethod::Min, "min(QD1,QD2)"),
            (squid_n_design_jp::QdMethod::Qd1, "QD1"),
            (squid_n_design_jp::QdMethod::Qd2, "QD2"),
        ] {
            if ui
                .selectable_label(app.analysis_cfg.qd_method == m, label)
                .on_hover_text(
                    "地震時短期の設計用せん断力の決定方法（QD1=終局曲げベース、\
                     QD2=QL+n・QE。長期組合せ(G+P)を先に解析している場合のみ有効）",
                )
                .clicked()
            {
                app.analysis_cfg.qd_method = m;
                changed = true;
            }
        }
        if changed {
            app.run_design_check();
        }
    });
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
    // 各行の部材に割り当てられている断面（NG部材→断面編集への遷移用）。
    let section_of: Vec<Option<(squid_n_core::ids::SectionId, String)>> = checks
        .iter()
        .map(|(elem_id, _, _, _, _)| {
            app.model
                .elements
                .iter()
                .find(|e| e.id == *elem_id)
                .and_then(|e| e.section)
                .and_then(|sid| {
                    app.model
                        .sections
                        .iter()
                        .find(|s| s.id == sid)
                        .map(|s| (sid, s.name.clone()))
                })
        })
        .collect();

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
    let mut jump_to_section: Option<(squid_n_core::ids::SectionId, squid_n_core::ids::ElemId)> =
        None;
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(80.0))
        .column(Column::initial(60.0))
        .column(Column::initial(200.0))
        .column(Column::initial(90.0))
        .header(20.0, |mut h| {
            for t in &["部材", "位置", "検定比", "判定", "根拠", "断面"] {
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
                row.col(|ui| match &section_of[i] {
                    Some((sid, name)) => {
                        if ui
                            .button(name)
                            .on_hover_text("クリックでモデルタブの断面編集へ移動")
                            .clicked()
                        {
                            jump_to_section = Some((*sid, *elem_id));
                        }
                    }
                    None => {
                        ui.label("-");
                    }
                });
            });
        });
    if let Some(id) = focus {
        app.nav.focus_member = Some(id);
    }
    if let Some((sid, eid)) = jump_to_section {
        app.active_tab = crate::app::Tab::Model;
        app.model_tab = crate::app::ModelTab::Sections;
        app.nav.focus_section = Some(sid);
        app.nav.focus_member = Some(eid);
    }

    // ── 一次設計: 節点単位の検定（柱梁接合部・パネルゾーン・冷間耐力比・耐震壁） ──
    let joint_checks: Vec<(squid_n_core::ids::NodeId, String, f64, bool, String)> = app
        .results
        .as_ref()
        .map(|r| {
            r.joint_checks
                .iter()
                .map(|(id, label, cr)| (*id, label.clone(), cr.ratio, cr.ok, cr.basis.clone()))
                .collect()
        })
        .unwrap_or_default();
    if !joint_checks.is_empty() {
        ui.add_space(12.0);
        ui.strong("接合部・耐震壁の検定");
        let ng = joint_checks.iter().filter(|(_, _, _, ok, _)| !ok).count();
        ui.label(format!("{} 箇所を検定、NG {} 件", joint_checks.len(), ng));
        TableBuilder::new(ui)
            .striped(true)
            .id_salt("joint_checks")
            .column(Column::auto())
            .column(Column::initial(110.0))
            .column(Column::initial(80.0))
            .column(Column::initial(60.0))
            .column(Column::initial(260.0))
            .header(20.0, |mut h| {
                for t in &["節点", "種別", "検定比", "判定", "根拠"] {
                    h.col(|ui| {
                        ui.strong(*t);
                    });
                }
            })
            .body(|body| {
                body.rows(18.0, joint_checks.len(), |mut row| {
                    let (node_id, label, ratio, ok, basis) = &joint_checks[row.index()];
                    row.col(|ui| {
                        ui.label(node_id.0.to_string());
                    });
                    row.col(|ui| {
                        ui.label(label);
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
    }

    // ── 免震支承材の非線形特性 ────────────
    if !app.model.isolator_attrs.is_empty() {
        ui.add_space(12.0);
        ui.strong("免震支承材の非線形特性");
        for a in &app.model.isolator_attrs {
            let p = a.props;
            let ks = squid_n_design_jp::isolator::multi_shear_stiffness_reduction(p.n_springs);
            let qs = squid_n_design_jp::isolator::multi_shear_strength_reduction(p.n_springs);
            use squid_n_core::model::IsolatorKind;
            let desc = match p.kind {
                IsolatorKind::LaminatedRubber
                | IsolatorKind::LeadRubber
                | IsolatorKind::HighDampingRubber => {
                    // 等価水平剛性 keq・等価粘性減衰定数 Heq を設計変位 200mm（参考）で算定
                    // （LRB 統一型 keq=Qd/δ+Kd、Heq=(2/π)Qd(δ−Qd/((β−1)Kd))/(keq·δ²)）。
                    let disp = 200.0;
                    let keq = squid_n_design_jp::isolator::equivalent_stiffness(p.k2, p.qd, disp);
                    let heq =
                        squid_n_design_jp::isolator::equivalent_damping(p.k1, p.k2, p.qd, disp);
                    let kind_label = match p.kind {
                        IsolatorKind::LeadRubber => "鉛プラグ積層ゴム(LRB)",
                        IsolatorKind::HighDampingRubber => "高減衰ゴム(HDR)",
                        _ => "天然ゴム積層ゴム",
                    };
                    let strain_dep = if p.total_rubber_thickness > 0.0
                        && (p.ckd_gamma != [1.0, 0.0, 0.0] || p.cqd_gamma != [1.0, 0.0, 0.0])
                    {
                        format!("／ 歪依存 H={:.0}mm", p.total_rubber_thickness)
                    } else {
                        String::new()
                    };
                    format!(
                        "{} K1={:.0} K2={:.0} Qd={:.0}kN Kv={:.0} ／ δ=200mm時 keq={:.1} Heq={:.3} {}",
                        kind_label,
                        p.k1,
                        p.k2,
                        p.qd / 1000.0,
                        p.kv,
                        keq,
                        heq,
                        strain_dep
                    )
                }
                IsolatorKind::ElasticSliding => format!(
                    "弾性すべり μ={:.3} N={:.0}kN Qmax={:.0}kN Kv={:.0}",
                    p.mu,
                    p.n_long / 1000.0,
                    squid_n_design_jp::isolator::friction_max_force(p.mu, p.n_long) / 1000.0,
                    p.kv
                ),
            };
            ui.label(format!(
                "部材{}: {} ／ マルチシア n={} 剛性低減={:.4} 耐力低減={:.4}",
                a.elem.0, desc, p.n_springs, ks, qs
            ));
        }
    }

    // ── 制振ダンパーの非線形特性 ──
    if !app.model.damper_attrs.is_empty() {
        ui.add_space(12.0);
        ui.strong("制振ダンパーの非線形特性");
        for a in &app.model.damper_attrs {
            let p = a.props;
            match p.kind {
                squid_n_core::model::DamperKind::Maxwell => {
                    // 緩和時間 τ=C0/Kd。線形マクスウェルの損失は ωτ≈1 で最大。
                    let tau = if p.kd > 0.0 { p.c0 / p.kd } else { 0.0 };
                    ui.label(format!(
                        "部材{}: マクスウェル Kd={:.0} C0={:.0} α={:.2} ／ 緩和時間 τ={:.3}s（時刻歴で作用）",
                        a.elem.0, p.kd, p.c0, p.alpha, tau
                    ));
                }
                squid_n_core::model::DamperKind::HystereticBilinear => {
                    // 降伏変位 δy=Qy/k1。
                    let dy = if p.kd > 0.0 { p.qy / p.kd } else { 0.0 };
                    ui.label(format!(
                        "部材{}: 履歴型ﾊﾞｲﾘﾆｱ k1={:.0} Qy={:.0} k2/k1={:.3} ／ 降伏変位 δy={:.2}mm（静的・動的で作用）",
                        a.elem.0, p.kd, p.qy, p.k2_ratio, dy
                    ));
                }
            }
        }
    }

    // ── 非線形解析の材端履歴則 ──
    ui.add_space(12.0);
    egui::CollapsingHeader::new("非線形解析の材端履歴則")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(
                "材端曲げバネの復元力履歴則。既定は RC/SRC/CFT 梁=武田型、S 梁=標準型（部材表で個別指定可）。",
            );
            use std::collections::BTreeMap;
            let mut counts: BTreeMap<&'static str, u32> = BTreeMap::new();
            let mut overrides: Vec<String> = Vec::new();
            for e in &app.model.elements {
                if e.kind != squid_n_core::model::ElementKind::Beam {
                    continue;
                }
                let eff = squid_n_element::factory::resolve_member_hysteresis(e, &app.model);
                *counts.entry(eff.label()).or_default() += 1;
                if let Some(r) = app.model.member_hysteresis(e.id) {
                    overrides.push(format!("部材{}: {}", e.id.0, r.label()));
                }
            }
            if counts.is_empty() {
                ui.label("梁部材がありません。");
            } else {
                for (label, cnt) in &counts {
                    ui.label(format!("{}: {} 部材", label, cnt));
                }
            }
            if !overrides.is_empty() {
                ui.label(format!("個別指定: {}", overrides.join(", ")));
            }
        });

    // ── 二次設計: 層指標（層間変形角・剛性率・偏心率） ────────────
    ui.add_space(12.0);
    ui.strong("層指標（二次設計: 層間変形角・剛性率・偏心率）");
    if app.model.stories.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "階が未定義です。解析タブの「階の自動生成」を実行してください。",
        );
    } else if let Some(st) = app.current_static() {
        // 表示対象はナビゲータの結果ケース選択（→最後に実行した結果）に追従する。
        let ctx = crate::summary::metrics_ctx_from_results(app.results.as_ref());
        let metrics = crate::summary::compute_story_metrics_with(
            &app.model,
            &st.disp,
            app.analysis_cfg.seismic_dir,
            &ctx,
        );

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
                // 変形角の制限値は計算条件（令82条の2: 原則 1/200、緩和時 1/120）に追従する。
                let denom = metrics
                    .first()
                    .map(|m| m.drift_limit_denom)
                    .unwrap_or(app.model.stress_cfg.drift_limit_denom);
                let drift_label = format!("変形角(1/{:.0})", denom);
                for t in [
                    "階",
                    "階高[mm]",
                    "層間変位[mm]",
                    drift_label.as_str(),
                    "剛性率Rs(≥0.6)",
                    "偏心率Re(≤0.15)",
                    "Fes",
                ] {
                    h.col(|ui| {
                        ui.strong(t);
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
    } else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "静的解析結果がありません。地震静的(Ai)を実行すると層指標を評価できます。",
        );
    }

    // ── 二次設計: 保有水平耐力（ルート3） ──────────────────────
    ui.add_space(12.0);
    ui.strong("保有水平耐力（ルート3）");
    ui.horizontal(|ui| {
        use squid_n_design_jp::secondary::holding_capacity::FrameType;
        ui.label("架構種別:");
        ui.selectable_value(&mut app.design_frame, FrameType::RcFrame, "RCラーメン");
        ui.selectable_value(&mut app.design_frame, FrameType::RcWall, "RC壁式");
        ui.selectable_value(&mut app.design_frame, FrameType::SteelFrame, "Sラーメン");
        ui.selectable_value(&mut app.design_frame, FrameType::SteelBrace, "Sブレース");
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.design_rank_auto,
            "自動判定（鋼=幅厚比・RC矩形=Qsu/Qmu）",
        )
        .on_hover_text(
            "鋼部材(断面形状を持つもの)は幅厚比から、RC矩形部材(断面形状 RcRect かつ\
                 コンクリート強度Fc設定済みの材料)はせん断余裕度 Qsu/Qmu の略算から\
                 部材ランクを層ごとに自動判定します。断面形状未設定の部材・幅厚比の対象外\
                 形状(円形鋼管等)・RC円形・Fc未設定材料はスキップされ、1 本も算定できなかった\
                 層は下記の選択値にフォールバックします。RC の Qsu の軸力項に用いる軸力は\
                 先頭荷重ケース（長期相当）の結果を優先し、なければ最後に実行した\
                 静的解析結果を使用する簡易運用です。",
        );
    });
    ui.horizontal(|ui| {
        use squid_n_design_jp::secondary::holding_capacity::MemberRank;
        ui.label(if app.design_rank_auto {
            "部材ランク（フォールバック用）:"
        } else {
            "部材ランク:"
        });
        ui.selectable_value(&mut app.design_rank, MemberRank::FA, "FA");
        ui.selectable_value(&mut app.design_rank, MemberRank::FB, "FB");
        ui.selectable_value(&mut app.design_rank, MemberRank::FC, "FC");
        ui.selectable_value(&mut app.design_rank, MemberRank::FD, "FD");
    });
    if !app.design_rank_auto {
        let ds = squid_n_design_jp::secondary::holding_capacity::ds_value(
            app.design_frame,
            app.design_rank,
        );
        ui.label(format!("Ds = {:.2}（部材ランク選択値による簡易運用）", ds));
    }

    match app.compute_holding_capacity() {
        Err(msg) => {
            ui.colored_label(crate::theme::GRAY_600, &msg);
            let needs_analysis =
                msg.contains("プッシュオーバー") || msg.contains("地震静的") || msg.contains("階");
            if needs_analysis && ui.button("▶ 解析タブへ").clicked() {
                app.active_tab = crate::app::Tab::Analysis;
            }
        }
        Ok((result, story_ranks)) => {
            TableBuilder::new(ui)
                .striped(true)
                .column(Column::auto())
                .column(Column::initial(80.0))
                .column(Column::initial(80.0))
                .column(Column::initial(60.0))
                .column(Column::initial(60.0))
                .column(Column::initial(80.0))
                .column(Column::initial(60.0))
                .column(Column::initial(70.0))
                .header(20.0, |mut h| {
                    for t in &[
                        "階",
                        "Qu[kN]",
                        "Qud[kN]",
                        "Ds",
                        "Fes",
                        "Qun[kN]",
                        "判定",
                        "採用ランク",
                    ] {
                        h.col(|ui| {
                            ui.strong(*t);
                        });
                    }
                })
                .body(|body| {
                    body.rows(18.0, result.stories.len(), |mut row| {
                        let i = row.index();
                        let s = &result.stories[i];
                        let name = app
                            .model
                            .stories
                            .get(i)
                            .map(|st| st.name.clone())
                            .unwrap_or_else(|| format!("{}", s.story.0));
                        row.col(|ui| {
                            ui.label(&name);
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", s.qu / 1000.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", s.qud / 1000.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", s.ds));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", s.fes));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", s.qun / 1000.0));
                        });
                        row.col(|ui| {
                            if s.ok {
                                ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                            } else {
                                ui.colored_label(crate::theme::ERROR_RED, "NG");
                            }
                        });
                        row.col(|ui| {
                            ui.label(story_ranks.get(i).map(|r| rank_label(*r)).unwrap_or("-"));
                        });
                    });
                });
            let note = if app.design_rank_auto {
                "Qu はプッシュオーバー最終ステップの層せん断力。Ds は部材ランク自動判定\
                 （鋼=幅厚比、RC矩形=せん断余裕度 Qsu/Qmu の略算）。形状未設定・RC円形・\
                 Fc未設定材料は選択値フォールバック。"
            } else {
                "Qu はプッシュオーバー最終ステップの層せん断力。Ds は選択値（部材ランク自動判定OFF）。"
            };
            ui.colored_label(crate::theme::GRAY_600, note);
        }
    }

    floor_design_section(ui, app);
}

/// 床の中での小梁・スラブ設計の表示（`ResultsBundle.joist_checks`/`slab_checks`）。
/// 小梁は単純支持梁として曲げ・たわみを検定し、スラブは一方向版として設計曲げ
/// モーメント・必要鉄筋量を表示する（いずれも全体 FEM から独立）。
fn floor_design_section(ui: &mut egui::Ui, app: &App) {
    use egui_extras::{Column, TableBuilder};
    let Some(r) = app.results.as_ref() else {
        return;
    };
    if r.joist_checks.is_empty() && r.slab_checks.is_empty() {
        return;
    }

    ui.add_space(12.0);
    ui.strong("小梁・床の設計（床の中で・単純支持／一方向）");
    ui.colored_label(
        crate::theme::GRAY_600,
        "小梁は大梁を分割せず、床の中で単純支持梁として曲げ・たわみを検定します（反力は\
         大梁へ CMQ として伝達）。スラブは一方向版として設計曲げと必要鉄筋量を算定します。\
         鋼小梁 E=205000・F=235、鉄筋 SD295（長期 ft=195）の既定値を用います。",
    );

    if !r.joist_checks.is_empty() {
        ui.label("小梁（単純支持梁）:");
        TableBuilder::new(ui)
            .id_salt("joist_design_table")
            .striped(true)
            .column(Column::auto())
            .column(Column::auto())
            .column(Column::initial(80.0))
            .column(Column::initial(90.0))
            .column(Column::initial(90.0))
            .column(Column::initial(90.0))
            .column(Column::initial(70.0))
            .column(Column::auto())
            .header(20.0, |mut h| {
                for t in &[
                    "スラブ",
                    "小梁",
                    "スパン[mm]",
                    "M[kN·m]",
                    "Q[kN]",
                    "δ[mm]",
                    "検定比",
                    "判定",
                ] {
                    h.col(|ui| {
                        ui.strong(*t);
                    });
                }
            })
            .body(|mut body| {
                for (sid, ji, jr) in &r.joist_checks {
                    body.row(22.0, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("#{}", sid.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{ji}"));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", jr.span));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", jr.m_max * 1e-6));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", jr.q_max * 1e-3));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2} (δ/L=1/{:.0})", jr.deflection, {
                                if jr.deflection_span_ratio > 0.0 {
                                    1.0 / jr.deflection_span_ratio
                                } else {
                                    f64::INFINITY
                                }
                            }));
                        });
                        row.col(|ui| {
                            ui.colored_label(
                                crate::theme::status_color(jr.ratio),
                                format!("{:.2}", jr.ratio),
                            );
                        });
                        row.col(|ui| {
                            if jr.ok {
                                ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                            } else {
                                ui.colored_label(crate::theme::ERROR_RED, "NG");
                            }
                        });
                    });
                }
            });
    }

    if !r.slab_checks.is_empty() {
        ui.add_space(6.0);
        ui.label("スラブ（一方向版）:");
        TableBuilder::new(ui)
            .id_salt("slab_design_table")
            .striped(true)
            .column(Column::auto())
            .column(Column::initial(80.0))
            .column(Column::initial(90.0))
            .column(Column::initial(100.0))
            .column(Column::initial(80.0))
            .column(Column::initial(120.0))
            .header(20.0, |mut h| {
                for t in &[
                    "スラブ",
                    "スパン[mm]",
                    "w[kN/m²]",
                    "M[kN·m/m]",
                    "t[mm]",
                    "必要As[mm²/m]",
                ] {
                    h.col(|ui| {
                        ui.strong(*t);
                    });
                }
            })
            .body(|mut body| {
                for (sid, sr) in &r.slab_checks {
                    body.row(22.0, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("#{}", sid.0));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", sr.span));
                        });
                        row.col(|ui| {
                            // w[N/mm²] → kN/m²（×1e3）。
                            ui.label(format!("{:.2}", sr.w * 1e3));
                        });
                        row.col(|ui| {
                            // M[N·mm/mm] → kN·m/m（×1e-3）。
                            ui.label(format!("{:.2}", sr.moment * 1e-3));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", sr.thickness));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", sr.as_req_per_m));
                        });
                    });
                }
            });
    }
}

/// `MemberRank` の表示名（FA〜FD）。
fn rank_label(r: squid_n_design_jp::secondary::holding_capacity::MemberRank) -> &'static str {
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    match r {
        MemberRank::FA => "FA",
        MemberRank::FB => "FB",
        MemberRank::FC => "FC",
        MemberRank::FD => "FD",
    }
}
