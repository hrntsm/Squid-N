//! `App` の egui パネル描画メソッド。

use super::*;

impl App {
    /// 「開く…」ダイアログを表示して読み込む。
    pub(crate) fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Squid-N プロジェクト", &["scz"])
            .pick_file()
        {
            self.open_project_from(path);
        }
    }

    /// 保存する。`force_ask` またはパス未設定時はダイアログで保存先を尋ねる。
    pub(crate) fn save_project_dialog(&mut self, force_ask: bool) {
        let path = if force_ask {
            None
        } else {
            self.project_path.clone()
        };
        let path = path.or_else(|| {
            rfd::FileDialog::new()
                .add_filter("Squid-N プロジェクト", &["scz"])
                .set_file_name("model.scz")
                .save_file()
        });
        if let Some(path) = path {
            self.save_project_to(path);
        }
    }

    /// 「ST-Bridge 読込…」ダイアログを表示して読み込む。
    pub(crate) fn import_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .pick_file()
        {
            self.import_stbridge_from(path);
        }
    }

    /// 「ST-Bridge 書出…」ダイアログを表示して保存先を尋ねる。
    pub(crate) fn export_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .set_file_name("model.stb")
            .save_file()
        {
            self.export_stbridge_to(path);
        }
    }

    /// 左ペイン：ナビゲータ（階/部材群/荷重ケース/結果ケースのツリー）。
    pub(crate) fn navigator_panel(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.strong("ナビゲータ");
            ui.separator();

            // 部材グループ（簡易: 材種ごと）
            let header = egui::CollapsingHeader::new("部材グループ")
                .default_open(true)
                .id_salt("nav_groups");
            header.show(ui, |ui| {
                let steel_ids: Vec<ElemId> = self
                    .model
                    .elements
                    .iter()
                    .filter(|e| {
                        e.material
                            .and_then(|mid| self.model.materials.get(mid.index()))
                            .map(|m| is_steel(&m.name))
                            .unwrap_or(false)
                    })
                    .map(|e| e.id)
                    .collect();
                let rc_ids: Vec<ElemId> = self
                    .model
                    .elements
                    .iter()
                    .map(|e| e.id)
                    .filter(|id| !steel_ids.contains(id))
                    .collect();
                // selected 表示は簡易判定（先頭要素が当該グループに属するか）。
                let is_steel_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| steel_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_steel_sel, format!("鋼材部材 ({})", steel_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = steel_ids.clone();
                }
                let is_rc_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| rc_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_rc_sel, format!("RC部材 ({})", rc_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = rc_ids.clone();
                }
            });

            // 荷重ケース
            let header = egui::CollapsingHeader::new("荷重ケース")
                .default_open(true)
                .id_salt("nav_load_cases");
            header.show(ui, |ui| {
                for (i, lc) in self.model.load_cases.iter().enumerate() {
                    let is_sel = self
                        .nav
                        .focus_load_case
                        .map(|id| id == lc.id)
                        .unwrap_or(false);
                    if ui
                        .selectable_label(is_sel, format!("[{}] {}", i, lc.name))
                        .clicked()
                    {
                        self.nav.focus_load_case = Some(lc.id);
                    }
                }
            });

            // 部材リスト（クリックで focus_member を更新 → テーブル/インスペクタに連動）
            let header = egui::CollapsingHeader::new("部材一覧")
                .default_open(false)
                .id_salt("nav_members");
            header.show(ui, |ui| {
                use egui_extras::{Column, TableBuilder};
                TableBuilder::new(ui)
                    .striped(true)
                    .column(Column::auto())
                    .column(Column::remainder())
                    .header(16.0, |mut h| {
                        h.col(|ui| {
                            ui.strong("ID");
                        });
                        h.col(|ui| {
                            ui.strong("種別");
                        });
                    })
                    .body(|body| {
                        let n = self.model.elements.len();
                        body.rows(18.0, n, |mut row| {
                            let idx = row.index();
                            let elem = self.model.elements[idx].clone();
                            let is_focus = self.nav.focus_member == Some(elem.id);
                            row.col(|ui| {
                                if ui
                                    .add(egui::Button::selectable(is_focus, elem.id.0.to_string()))
                                    .clicked()
                                {
                                    self.nav.focus_member = Some(elem.id);
                                }
                            });
                            row.col(|ui| {
                                ui.label(format!("{:?}", elem.kind));
                            });
                        });
                    });
            });

            // 結果ケース：静的解析結果／荷重組合せ結果をクリックで表示対象に選択できる。
            let header = egui::CollapsingHeader::new("結果ケース")
                .default_open(true)
                .id_salt("nav_result_cases");
            header.show(ui, |ui| {
                if let Some(r) = &self.results {
                    if r.statics.is_empty() && r.combos.is_empty() && r.modal.is_none() {
                        ui.label("（未実行）");
                    } else {
                        for (key, _) in r.statics.iter() {
                            let label = match key {
                                StaticCaseKey::User(id) => {
                                    let lc_name = self
                                        .model
                                        .load_cases
                                        .iter()
                                        .find(|lc| lc.id == *id)
                                        .map(|lc| lc.name.as_str())
                                        .unwrap_or("");
                                    format!("静的 LC {} {}", id.0, lc_name)
                                }
                                StaticCaseKey::Seismic(SeismicDir::X) => {
                                    "地震静的 (X方向)".to_string()
                                }
                                StaticCaseKey::Seismic(SeismicDir::Y) => {
                                    "地震静的 (Y方向)".to_string()
                                }
                                StaticCaseKey::Wind(SeismicDir::X) => "風静的 (X方向)".to_string(),
                                StaticCaseKey::Wind(SeismicDir::Y) => "風静的 (Y方向)".to_string(),
                            };
                            let is_sel = self.nav.focus_result == Some(StaticKey::Case(*key));
                            if ui.selectable_label(is_sel, label).clicked() {
                                self.nav.focus_result = Some(StaticKey::Case(*key));
                            }
                        }
                        for (i, (name, _)) in r.combos.iter().enumerate() {
                            let is_sel = self.nav.focus_result == Some(StaticKey::Combo(i));
                            if ui
                                .selectable_label(is_sel, format!("組合せ {}", name))
                                .clicked()
                            {
                                self.nav.focus_result = Some(StaticKey::Combo(i));
                            }
                        }
                        if r.modal.is_some() {
                            ui.label("固有値");
                        }
                    }
                } else {
                    ui.label("（未実行）");
                }
            });

            // 階/レベル（階の自動生成結果を上階→下階順に表示）
            let _ = ui.collapsing("階/レベル", |ui| {
                if self.model.stories.is_empty() {
                    ui.colored_label(crate::theme::GRAY_600, "未定義");
                    if ui.small_button("🏢 解析タブで自動生成").clicked() {
                        self.active_tab = Tab::Analysis;
                    }
                } else {
                    for s in self.model.stories.iter().rev() {
                        ui.label(format!(
                            "{}  Z={:.0}mm  W={:.1}kN",
                            s.name,
                            s.elevation,
                            s.seismic_weight.unwrap_or(0.0) / 1000.0
                        ));
                    }
                }
            });
        });
    }

    /// モデルタブ：サブタブ切替で節点/部材/断面/材料を編集するテーブルを表示。
    pub(crate) fn model_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui
                .button("📄 新規")
                .on_hover_text("現在のモデルを破棄して空のモデルを作成します")
                .clicked()
            {
                self.load_model(squid_n_core::model::Model::default());
            }
            if ui
                .button("🏠 サンプル読込")
                .on_hover_text("現在のモデルを破棄して門型ラーメンのサンプルを読み込みます")
                .clicked()
            {
                self.load_model(crate::sample::portal_frame());
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            let subs = [
                ("節点", ModelTab::Nodes),
                ("境界条件", ModelTab::BoundaryConditions),
                ("部材", ModelTab::Members),
                ("断面", ModelTab::Sections),
                ("材料", ModelTab::Materials),
                ("スラブ", ModelTab::Slabs),
                ("壁属性", ModelTab::WallAttrs),
                ("雑壁", ModelTab::MiscWalls),
                ("部材付帯情報", ModelTab::MemberDetails),
            ];
            for (label, sub) in &subs {
                let sel = self.model_tab == *sub;
                if ui.selectable_label(sel, *label).clicked() {
                    self.model_tab = *sub;
                }
            }
        });
        ui.separator();
        match self.model_tab {
            ModelTab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            ModelTab::BoundaryConditions => {
                crate::tables::nodes::boundary_condition_panel(ui, self)
            }
            ModelTab::Members => crate::tables::members::members_table(ui, self),
            ModelTab::Sections => {
                crate::tables::sections::sections_table(ui, self);
                ui.add_space(8.0);
                crate::section_editor::catalog_section_panel(ui, self);
                ui.add_space(8.0);
                crate::section_editor::section_editor_panel(ui, self);
            }
            ModelTab::Materials => crate::tables::materials::materials_table(ui, self),
            ModelTab::Slabs => crate::tables::slabs::slabs_table(ui, self),
            ModelTab::WallAttrs => crate::tables::wall_attrs::wall_attrs_table(ui, self),
            ModelTab::MiscWalls => crate::tables::misc_walls::misc_walls_table(ui, self),
            ModelTab::MemberDetails => {
                crate::tables::member_details::member_details_table(ui, self)
            }
        }
    }

    /// 解析タブ：種別選択＋実行＋進捗表示。
    pub(crate) fn analysis_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("解析設定");
        ui.separator();

        // バックグラウンドジョブ実行中は全解析ボタンを無効化する（P8 §5）。
        let running = self.job.is_some();

        if let Some(when) = self.staleness.last_run {
            if let Ok(dur) = when.elapsed() {
                ui.label(format!("最終実行: {:.0} 秒前", dur.as_secs_f64()));
            } else {
                ui.label("最終実行: 不明");
            }
        } else {
            ui.label("最終実行: なし");
        }
        if self.staleness.results_stale {
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ モデルが編集されました。結果は再計算が必要です。",
            );
        }
        ui.separator();

        // ── 階の定義（地震系解析の前提） ──────────────────────────
        ui.group(|ui| {
            ui.strong("階の定義");
            if self.model.stories.is_empty() {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "未定義（地震静的・プッシュオーバーには階が必要です）",
                );
            } else {
                use squid_n_core::model::{StoryLevelKind, StoryStructure};
                // model.stories を借用したまま undo.run（model の可変借用）はできないため、
                // 行データを先に複製してから描画・編集確定を行う。
                #[allow(clippy::type_complexity)]
                let story_rows: Vec<(
                    squid_n_core::ids::StoryId,
                    String,
                    f64,
                    usize,
                    Option<f64>,
                    StoryStructure,
                    StoryLevelKind,
                )> = self
                    .model
                    .stories
                    .iter()
                    .map(|s| {
                        (
                            s.id,
                            s.name.clone(),
                            s.elevation,
                            s.node_ids.len(),
                            s.seismic_weight,
                            s.structure,
                            s.level_kind,
                        )
                    })
                    .collect();
                self.story_weight_edit.resize(story_rows.len(), 0.0);
                self.story_weight_active.resize(story_rows.len(), false);
                for (i, (story, name, elevation, n_nodes, weight, structure, level_kind)) in
                    story_rows.into_iter().enumerate()
                {
                    if !self.story_weight_active[i] {
                        self.story_weight_edit[i] = weight.unwrap_or(0.0) / 1000.0;
                    }
                    ui.horizontal(|ui| {
                        ui.label(format!(
                            "{}: 標高 {:.0} mm, 節点 {}",
                            name, elevation, n_nodes
                        ));
                        ui.label("W[kN]:");
                        let resp = ui
                            .add(
                                egui::DragValue::new(&mut self.story_weight_edit[i])
                                    .speed(1.0)
                                    .range(0.0..=1.0e9),
                            )
                            .on_hover_text("地震重量を手動調整します(自動生成値を上書き、undo可)");
                        self.story_weight_active[i] = resp.dragged() || resp.has_focus();
                        if resp.drag_stopped() || resp.lost_focus() {
                            let new_weight = self.story_weight_edit[i] * 1000.0;
                            if (new_weight - weight.unwrap_or(0.0)).abs() > 1e-6 {
                                self.undo.run(
                                    &mut self.model,
                                    Box::new(squid_n_edit::SetStoryWeight {
                                        story,
                                        weight: Some(new_weight),
                                    }),
                                );
                                self.staleness.mark_edited();
                            }
                        }

                        ui.separator();
                        ui.label("構造:");
                        for (label, st) in [
                            ("RC", StoryStructure::Rc),
                            ("S", StoryStructure::S),
                            ("SRC", StoryStructure::Src),
                        ] {
                            if ui.selectable_label(structure == st, label).clicked()
                                && structure != st
                            {
                                self.undo.run(
                                    &mut self.model,
                                    Box::new(squid_n_edit::SetStoryStructure {
                                        story,
                                        structure: st,
                                    }),
                                );
                                self.staleness.mark_edited();
                            }
                        }

                        ui.separator();
                        ui.label("種別:");
                        let level_label = match level_kind {
                            StoryLevelKind::Normal => "一般".to_string(),
                            StoryLevelKind::Penthouse { k } => format!("PH k={:.2}", k),
                            StoryLevelKind::Basement { depth_m } => {
                                format!("地下 depth={:.1}m", depth_m)
                            }
                        };
                        let mut new_level_kind: Option<StoryLevelKind> = None;
                        egui::ComboBox::from_id_salt(("story_level_kind", story.0))
                            .selected_text(level_label)
                            .show_ui(ui, |ui| {
                                if ui
                                    .selectable_label(
                                        matches!(level_kind, StoryLevelKind::Normal),
                                        "一般",
                                    )
                                    .clicked()
                                {
                                    new_level_kind = Some(StoryLevelKind::Normal);
                                }
                                if ui
                                    .selectable_label(
                                        matches!(level_kind, StoryLevelKind::Penthouse { .. }),
                                        "PH(塔屋)",
                                    )
                                    .clicked()
                                {
                                    let k = if let StoryLevelKind::Penthouse { k } = level_kind {
                                        k
                                    } else {
                                        0.5
                                    };
                                    new_level_kind = Some(StoryLevelKind::Penthouse { k });
                                }
                                if ui
                                    .selectable_label(
                                        matches!(level_kind, StoryLevelKind::Basement { .. }),
                                        "地下",
                                    )
                                    .clicked()
                                {
                                    let depth_m =
                                        if let StoryLevelKind::Basement { depth_m } = level_kind {
                                            depth_m
                                        } else {
                                            3.0
                                        };
                                    new_level_kind = Some(StoryLevelKind::Basement { depth_m });
                                }
                            });
                        if let StoryLevelKind::Penthouse { k } = level_kind {
                            let mut kv = k;
                            let resp = ui.add(
                                egui::DragValue::new(&mut kv)
                                    .speed(0.05)
                                    .range(0.0..=2.0)
                                    .prefix("k="),
                            );
                            if (resp.drag_stopped() || resp.lost_focus()) && (kv - k).abs() > 1e-9 {
                                new_level_kind = Some(StoryLevelKind::Penthouse { k: kv });
                            }
                        }
                        if let StoryLevelKind::Basement { depth_m } = level_kind {
                            let mut dv = depth_m;
                            let resp = ui.add(
                                egui::DragValue::new(&mut dv)
                                    .speed(0.1)
                                    .range(0.0..=100.0)
                                    .suffix("m"),
                            );
                            if (resp.drag_stopped() || resp.lost_focus())
                                && (dv - depth_m).abs() > 1e-9
                            {
                                new_level_kind = Some(StoryLevelKind::Basement { depth_m: dv });
                            }
                        }
                        if let Some(lk) = new_level_kind {
                            self.undo.run(
                                &mut self.model,
                                Box::new(squid_n_edit::SetStoryLevelKind {
                                    story,
                                    level_kind: lk,
                                }),
                            );
                            self.staleness.mark_edited();
                        }
                    });
                }
            }
            if ui
                .button("🏢 階の自動生成")
                .on_hover_text(
                    "節点の標高(Z)から階を推定し、剛床と地震重量(自重+先頭荷重ケース)を設定します",
                )
                .clicked()
            {
                self.generate_stories_action();
            }
        });
        ui.add_space(6.0);

        // ── 線形静的 ──────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("線形静的");
            let selected_lc = self
                .nav
                .focus_load_case
                .filter(|id| self.model.load_cases.iter().any(|c| c.id == *id))
                .or_else(|| self.model.load_cases.first().map(|c| c.id));
            ui.horizontal(|ui| {
                ui.label("荷重ケース:");
                let text = selected_lc
                    .and_then(|id| {
                        self.model
                            .load_cases
                            .iter()
                            .find(|c| c.id == id)
                            .map(|c| format!("[{}] {}", c.id.0, c.name))
                    })
                    .unwrap_or_else(|| "（なし）".to_string());
                egui::ComboBox::from_id_salt("analysis_lc")
                    .selected_text(text)
                    .show_ui(ui, |ui| {
                        for lc in &self.model.load_cases {
                            if ui
                                .selectable_label(
                                    selected_lc == Some(lc.id),
                                    format!("[{}] {}", lc.id.0, lc.name),
                                )
                                .clicked()
                            {
                                self.nav.focus_load_case = Some(lc.id);
                            }
                        }
                    });
                if ui
                    .add_enabled(
                        selected_lc.is_some() && !running,
                        egui::Button::new("▶ 実行"),
                    )
                    .clicked()
                {
                    if let Some(lc) = selected_lc {
                        self.run_linear_static(lc);
                        if self.last_error.is_none() {
                            self.active_tab = Tab::Results;
                            self.results_view = ResultsView::Spatial;
                        }
                    }
                }
            });
            if self.model.load_cases.is_empty() {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "荷重ケースがありません。荷重タブで作成してください。",
                );
            }
        });
        ui.add_space(6.0);

        // ── 荷重組合せ ────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("荷重組合せ");
            if self.model.combinations.is_empty() {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "荷重組合せがありません。荷重タブで作成してください。",
                );
            } else {
                if self.analysis_combo_idx >= self.model.combinations.len() {
                    self.analysis_combo_idx = 0;
                }
                ui.horizontal(|ui| {
                    ui.label("組合せ:");
                    let text = self
                        .model
                        .combinations
                        .get(self.analysis_combo_idx)
                        .map(|c| c.name.clone())
                        .unwrap_or_else(|| "（なし）".to_string());
                    egui::ComboBox::from_id_salt("analysis_combo")
                        .selected_text(text)
                        .show_ui(ui, |ui| {
                            for (i, combo) in self.model.combinations.iter().enumerate() {
                                if ui
                                    .selectable_label(
                                        self.analysis_combo_idx == i,
                                        format!("[{}] {}", i, combo.name),
                                    )
                                    .clicked()
                                {
                                    self.analysis_combo_idx = i;
                                }
                            }
                        });
                    if ui
                        .add_enabled(!running, egui::Button::new("▶ 実行"))
                        .clicked()
                    {
                        self.run_combination(self.analysis_combo_idx);
                        if self.last_error.is_none() {
                            self.active_tab = Tab::Results;
                        }
                    }
                });
            }
        });
        ui.add_space(6.0);

        // ── 固有値 ────────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("固有値");
            ui.horizontal(|ui| {
                ui.label("モード数:");
                let mut n = self.analysis_cfg.n_modes;
                ui.add(egui::DragValue::new(&mut n).range(1..=30));
                self.analysis_cfg.n_modes = n;
                if ui
                    .add_enabled(!running, egui::Button::new("▶ 実行"))
                    .clicked()
                {
                    self.run_eigen(self.analysis_cfg.n_modes);
                }
            });
        });
        ui.add_space(6.0);

        // ── 地震静的（Ai 分布） ───────────────────────────────────
        ui.group(|ui| {
            ui.strong("地震静的 (Ai 分布)");
            ui.horizontal(|ui| {
                ui.label("方向:");
                ui.selectable_value(&mut self.analysis_cfg.seismic_dir, SeismicDir::X, "X");
                ui.selectable_value(&mut self.analysis_cfg.seismic_dir, SeismicDir::Y, "Y");
                ui.separator();
                ui.label("T算定:");
                ui.selectable_value(
                    &mut self.analysis_cfg.ai_mode,
                    AiMode::SemiPrecise,
                    "固有値",
                )
                .on_hover_text("固有値解析による 1 次周期");
                ui.selectable_value(&mut self.analysis_cfg.ai_mode, AiMode::Approx, "略算")
                    .on_hover_text("T = h(0.02 + 0.01α) の略算式");
            });
            ui.horizontal(|ui| {
                ui.label("Z:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.z)
                        .speed(0.05)
                        .range(0.7..=1.0),
                );
                ui.label("地盤:");
                use squid_n_load::ai::SoilClass;
                for (label, soil) in [
                    ("第一種", SoilClass::I),
                    ("第二種", SoilClass::II),
                    ("第三種", SoilClass::III),
                ] {
                    ui.selectable_value(&mut self.analysis_cfg.soil, soil, label);
                }
                ui.label("C0:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.c0)
                        .speed(0.05)
                        .range(0.05..=1.0),
                );
            });
            // Z表（告示1793号別表第2、市町村名→Z のCSV）からの参照
            ui.horizontal(|ui| {
                if ui
                    .button("📂 Z表CSV読込…")
                    .on_hover_text("「市町村名,Z値」形式のCSVを読み込みます（#始まりはコメント行）")
                    .clicked()
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Z表 (CSV)", &["csv", "txt"])
                        .pick_file()
                    {
                        match std::fs::read_to_string(&path) {
                            Ok(csv) => self.load_z_table_from_csv(&csv),
                            Err(e) => self.last_error = Some(format!("Z表読込エラー: {}", e)),
                        }
                    }
                }
                match &self.z_table {
                    Some(t) => {
                        ui.label(format!("{} 市町村", t.len()));
                    }
                    None => {
                        ui.colored_label(crate::theme::GRAY_600, "（未読込）");
                    }
                }
                ui.label("市町村:");
                ui.add(
                    egui::TextEdit::singleline(&mut self.z_table_municipality).desired_width(130.0),
                );
                let can_lookup =
                    self.z_table.is_some() && !self.z_table_municipality.trim().is_empty();
                if ui
                    .add_enabled(can_lookup, egui::Button::new("Z参照"))
                    .on_hover_text("市町村名（完全一致）でZ表を引き、Zへ反映します")
                    .clicked()
                {
                    let name = self.z_table_municipality.trim().to_string();
                    self.apply_z_from_municipality(&name);
                }
            });
            if ui
                .add_enabled(!running, egui::Button::new("▶ 実行"))
                .clicked()
            {
                self.run_seismic(self.analysis_cfg.seismic_dir);
                if self.last_error.is_none() {
                    self.active_tab = Tab::Results;
                    self.results_view = ResultsView::Spatial;
                }
            }
        });
        ui.add_space(6.0);

        // ── 風荷重静的 ─────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("風荷重静的");
            ui.horizontal(|ui| {
                ui.label("V0[m/s]:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.v0)
                        .speed(0.5)
                        .range(30.0..=46.0),
                );
                ui.label("粗度区分:");
                use squid_n_load::wind::TerrainRoughness;
                for (label, r) in [
                    ("I", TerrainRoughness::I),
                    ("II", TerrainRoughness::II),
                    ("III", TerrainRoughness::III),
                    ("IV", TerrainRoughness::IV),
                ] {
                    ui.selectable_value(&mut self.analysis_cfg.roughness, r, label);
                }
                ui.label("パラペット[mm]:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.parapet_mm)
                        .speed(10.0)
                        .range(0.0..=5000.0),
                );
            });
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!running, egui::Button::new("▶ 風荷重解析 (X)"))
                    .clicked()
                {
                    self.run_wind(SeismicDir::X);
                    if self.last_error.is_none() {
                        self.active_tab = Tab::Results;
                        self.results_view = ResultsView::Spatial;
                    }
                }
                if ui
                    .add_enabled(!running, egui::Button::new("▶ 風荷重解析 (Y)"))
                    .clicked()
                {
                    self.run_wind(SeismicDir::Y);
                    if self.last_error.is_none() {
                        self.active_tab = Tab::Results;
                        self.results_view = ResultsView::Spatial;
                    }
                }
            });
        });
        ui.add_space(6.0);

        // ── プッシュオーバー ──────────────────────────────────────
        ui.group(|ui| {
            ui.strong("プッシュオーバー");
            ui.horizontal(|ui| {
                ui.label("方向:");
                ui.selectable_value(&mut self.analysis_cfg.push_dir, SeismicDir::X, "X");
                ui.selectable_value(&mut self.analysis_cfg.push_dir, SeismicDir::Y, "Y");
                ui.label("ステップ:");
                ui.add(egui::DragValue::new(&mut self.analysis_cfg.push_steps).range(1..=100));
                ui.label("目標変位[mm]:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.push_max_disp)
                        .speed(10.0)
                        .range(1.0..=10000.0),
                );
            });
            ui.horizontal(|ui| {
                use squid_n_solver::pushover::DuctilityMethod;
                ui.label("塑性率方式:")
                    .on_hover_text("ファイバーモデルの塑性率（構造力学）");
                egui::ComboBox::from_id_salt("ductility_method")
                    .selected_text(match self.analysis_cfg.ductility_method {
                        DuctilityMethod::ReferenceStrain => "基点歪み",
                        DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                        DuctilityMethod::FirstYield => "降伏時",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.analysis_cfg.ductility_method,
                            DuctilityMethod::ReferenceStrain,
                            "基点歪み（RC:引張0.01/圧縮0.005・鉄骨0.01）",
                        );
                        ui.selectable_value(
                            &mut self.analysis_cfg.ductility_method,
                            DuctilityMethod::WeightedAverageJm,
                            "重み付け平均塑性率 Jm≥1",
                        );
                        ui.selectable_value(
                            &mut self.analysis_cfg.ductility_method,
                            DuctilityMethod::FirstYield,
                            "降伏発生時（塑性率1）",
                        );
                    });
            });
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!running, egui::Button::new("▶ 実行"))
                    .clicked()
                {
                    self.start_pushover_job();
                }
                if self
                    .job
                    .as_ref()
                    .is_some_and(|j| j.label == "プッシュオーバー")
                {
                    ui.spinner();
                }
            });
        });
        ui.add_space(6.0);

        // ── 時刻歴応答 ────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("時刻歴応答（線形）");
            ui.horizontal(|ui| {
                ui.label("方向:");
                ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::X, "X");
                ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::Y, "Y");
                ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::Xy, "X+Y")
                    .on_hover_text("同一波形を両方向へ同時入力(CSV は2列)");
                ui.separator();
                ui.label("積分法:");
                ui.selectable_value(
                    &mut self.analysis_cfg.th_integrator,
                    ThIntegrator::NewmarkBeta,
                    "Newmark-β",
                );
                ui.selectable_value(
                    &mut self.analysis_cfg.th_integrator,
                    ThIntegrator::HhtAlpha,
                    "HHT-α(α=-0.1)",
                );
            });
            ui.horizontal(|ui| {
                ui.label("減衰:");
                ui.selectable_value(
                    &mut self.analysis_cfg.th_damping_model,
                    ThDampingModel::StiffnessProportional,
                    "剛性比例",
                );
                ui.selectable_value(
                    &mut self.analysis_cfg.th_damping_model,
                    ThDampingModel::Rayleigh,
                    "Rayleigh",
                );
                ui.selectable_value(
                    &mut self.analysis_cfg.th_damping_model,
                    ThDampingModel::Modal,
                    "モード別",
                )
                .on_hover_text("各モードに減衰比 h を与える（非線形は初期剛性モード）");
                ui.selectable_value(
                    &mut self.analysis_cfg.th_damping_model,
                    ThDampingModel::TangentAlpha1,
                    "接線(α1一定)",
                )
                .on_hover_text("瞬間剛性比例。C=2h/ω1e·Kt を毎ステップ再構成");
                ui.selectable_value(
                    &mut self.analysis_cfg.th_damping_model,
                    ThDampingModel::TangentH1,
                    "接線(h1一定)",
                )
                .on_hover_text("瞬間剛性比例。ω1 を毎ステップ更新し減衰比 h1 を保つ");
                ui.separator();
                ui.label(match self.analysis_cfg.th_damping_model {
                    ThDampingModel::StiffnessProportional
                    | ThDampingModel::TangentAlpha1
                    | ThDampingModel::TangentH1 => "減衰比 h:",
                    ThDampingModel::Modal => "減衰比 h(全モード):",
                    ThDampingModel::Rayleigh => "h1(1次):",
                });
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_damping)
                        .speed(0.005)
                        .range(0.0..=0.3),
                );
                if self.analysis_cfg.th_damping_model == ThDampingModel::Rayleigh {
                    ui.label("h2(2次):");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_h2)
                            .speed(0.005)
                            .range(0.0..=0.3),
                    );
                }
            });
            ui.horizontal(|ui| {
                ui.label("サンプル波: dt[s]");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_dt)
                        .speed(0.001)
                        .range(0.001..=0.1),
                );
                ui.label("継続[s]");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_duration)
                        .speed(0.5)
                        .range(1.0..=120.0),
                );
                ui.label("周期[s]");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_period)
                        .speed(0.05)
                        .range(0.05..=5.0),
                );
                ui.label("振幅[mm/s²]");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_amp)
                        .speed(50.0)
                        .range(10.0..=10000.0),
                );
            });
            // 位相差入力（ねじれ加振）。構造動力学の位相差入力解析 t=(L·sinθ)/Vs。
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.analysis_cfg.phase_diff_enabled, "位相差入力")
                    .on_hover_text(
                        "見かけ速度で地震動が矩形基礎を通過する位相差からねじれ加振を生成",
                    );
                ui.add_enabled_ui(self.analysis_cfg.phase_diff_enabled, |ui| {
                    ui.label("Vs[m/s]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.phase_diff_vs)
                            .speed(10.0)
                            .range(50.0..=2000.0),
                    );
                    ui.label("L[m]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.phase_diff_length_m)
                            .speed(1.0)
                            .range(1.0..=500.0),
                    );
                    ui.label("θ[°]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.phase_diff_incidence_deg)
                            .speed(1.0)
                            .range(0.0..=90.0),
                    );
                    ui.selectable_value(&mut self.analysis_cfg.phase_diff_dir_y, false, "X");
                    ui.selectable_value(&mut self.analysis_cfg.phase_diff_dir_y, true, "Y");
                    let lag = squid_n_solver::phase_diff::phase_lag_time(
                        self.analysis_cfg.phase_diff_length_m,
                        self.analysis_cfg.phase_diff_incidence_deg,
                        self.analysis_cfg.phase_diff_vs,
                    );
                    ui.label(format!("位相遅れ {:.4}s", lag));
                });
            });
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!running, egui::Button::new("▶ サンプル波で実行"))
                    .on_hover_text("正弦減衰波を生成して時刻歴解析を実行します")
                    .clicked()
                {
                    let wave = Self::sample_wave(&self.analysis_cfg);
                    self.start_time_history_job(wave);
                }
                if ui
                    .add_enabled(!running, egui::Button::new("📂 波形CSVを開いて実行…"))
                    .on_hover_text(
                        "1 行 1 値(加速度 gal)の CSV/テキスト。dt は上の設定値を使用します",
                    )
                    .clicked()
                {
                    self.run_time_history_from_csv();
                }
                if self.job.as_ref().is_some_and(|j| j.label == "時刻歴応答") {
                    ui.spinner();
                }
            });
            ui.label(
                egui::RichText::new("応答グラフは入力の大きい方向を記録")
                    .small()
                    .color(crate::theme::GRAY_600),
            );
        });
    }

    /// 波形 CSV（X/Y: 1 行 1 値、X+Y: 1 行 2 列、いずれも gal 単位）を選択して
    /// 時刻歴解析をジョブ実行する。
    pub(crate) fn run_time_history_from_csv(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("波形 (CSV/テキスト)", &["csv", "txt", "dat"])
            .pick_file()
        else {
            return;
        };
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(e) => {
                self.last_error = Some(format!("波形読込エラー: {}", e));
                return;
            }
        };
        let dir = self.analysis_cfg.th_dir;
        let (col1, col2) = match parse_wave_csv(&content, dir) {
            Ok(v) => v,
            Err(e) => {
                self.last_error = Some(e);
                return;
            }
        };
        let wave = match dir {
            // X/Y は単一列を方向へ振り分ける（従来仕様、build_ground_motion 共用）。
            ThDir::X | ThDir::Y => Self::build_ground_motion(self.analysis_cfg.th_dt, dir, col1),
            // X+Y は CSV の 2 列がそのまま X・Y の入力になる
            // （build_ground_motion の Xy 分岐は「同一波形を複製」する仕様のため、
            // 別波形の 2 列読込はここで直接 GroundMotion を組み立てる）。
            ThDir::Xy => squid_n_solver::timehistory::GroundMotion {
                dt: self.analysis_cfg.th_dt,
                accel_x: col1,
                accel_y: col2,
                accel_theta: None,
            },
        };
        self.start_time_history_job(wave);
    }

    /// 結果タブ：3Dビューア と 時刻歴グラフを切替。
    pub(crate) fn results_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let sel_spatial = self.results_view == ResultsView::Spatial;
            let sel_th = self.results_view == ResultsView::TimeHistory;
            let sel_po = self.results_view == ResultsView::Pushover;
            let sel_lm = self.results_view == ResultsView::LumpedMass;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.results_view = ResultsView::TimeHistory;
            }
            if ui.selectable_label(sel_po, "プッシュオーバー").clicked() {
                self.results_view = ResultsView::Pushover;
            }
            if ui.selectable_label(sel_lm, "質点系モデル").clicked() {
                self.results_view = ResultsView::LumpedMass;
            }
            ui.separator();
            // 結果サマリ
            if let Some(r) = &self.results {
                ui.label(format!("静的ケース数: {}", r.statics.len()));
                if let Some(m) = &r.modal {
                    let t1 = m.period.first().copied().unwrap_or(0.0);
                    ui.label(format!("固有周期 T1: {:.3} s", t1));
                }
                ui.label(format!("検定結果数: {}", r.checks.len()));
            } else {
                ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
            }
        });
        ui.separator();
        match self.results_view {
            ResultsView::Spatial => crate::viewer::viewer_panel(ui, self),
            ResultsView::TimeHistory => crate::time_history_view::time_history_panel(ui, self),
            ResultsView::Pushover => self.pushover_panel(ui),
            ResultsView::LumpedMass => self.lumped_mass_panel(ui),
        }
    }

    /// 設計タブ：検定表（許容応力度・保有水平耐力）と MN 相関曲面ビューを切り替える。
    pub(crate) fn design_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let sel_table = self.design_view == DesignView::Table;
            let sel_ult = self.design_view == DesignView::Ultimate;
            let sel_mn = self.design_view == DesignView::MnSurface;
            let sel_qty = self.design_view == DesignView::Quantities;
            if ui.selectable_label(sel_table, "検定表").clicked() {
                self.design_view = DesignView::Table;
            }
            if ui.selectable_label(sel_ult, "終局検定").clicked() {
                self.design_view = DesignView::Ultimate;
            }
            if ui.selectable_label(sel_mn, "MN相関曲面").clicked() {
                self.design_view = DesignView::MnSurface;
            }
            if ui.selectable_label(sel_qty, "数量積算").clicked() {
                self.design_view = DesignView::Quantities;
            }
        });
        ui.separator();
        match self.design_view {
            DesignView::Table => crate::design_view::design_table(ui, self),
            DesignView::Ultimate => crate::ultimate_view::ultimate_table(ui, self),
            DesignView::MnSurface => crate::mn_view::mn_surface_panel(ui, self),
            DesignView::Quantities => crate::quantity_view::quantity_panel(ui, self),
        }
    }

    /// プッシュオーバー結果（性能曲線・ヒンジ・崩壊機構）の表示。
    pub(crate) fn pushover_panel(&mut self, ui: &mut egui::Ui) {
        let Some(po) = self.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "プッシュオーバー結果がありません。解析タブから実行してください。",
            );
            return;
        };

        ui.horizontal(|ui| {
            ui.label(format!("保有水平耐力 Qu = {:.1} kN", po.qu / 1000.0));
            ui.separator();
            let mech = match &po.mechanism {
                squid_n_solver::pushover::MechanismType::Overall => "全体崩壊形".to_string(),
                squid_n_solver::pushover::MechanismType::StoryCollapse { story } => {
                    format!("層崩壊形 (Story {})", story.0)
                }
                squid_n_solver::pushover::MechanismType::Partial => "部分崩壊形".to_string(),
            };
            ui.label(format!("崩壊機構: {}", mech));
            ui.separator();
            ui.label(format!("ヒンジ発生 {} 件", po.hinges.len()));
        });
        // 塑性率（構造力学）の方式と最大値。
        ui.horizontal(|ui| {
            use squid_n_solver::pushover::DuctilityMethod;
            let method = match self.analysis_cfg.ductility_method {
                DuctilityMethod::ReferenceStrain => "基点歪み",
                DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                DuctilityMethod::FirstYield => "降伏時",
            };
            let max_mu = po
                .hinges
                .iter()
                .map(|h| h.ductility)
                .fold(0.0_f64, f64::max);
            ui.label(format!("塑性率方式: {method}"));
            ui.separator();
            ui.label(format!("最大部材塑性率 μmax = {:.2}", max_mu));
        });

        // 性能曲線（頂部変位 - ベースシア）
        let points: Vec<[f64; 2]> = po
            .capacity_curve
            .iter()
            .map(|p| [p.roof_disp, p.base_shear / 1000.0])
            .collect();
        egui_plot::Plot::new("pushover_curve")
            .x_axis_label("頂部変位 [mm]")
            .y_axis_label("ベースシア [kN]")
            .height(ui.available_height() * 0.6)
            .show(ui, |plot_ui| {
                plot_ui.line(
                    egui_plot::Line::new("capacity", egui_plot::PlotPoints::from(points))
                        .color(crate::theme::DATA_BLUE)
                        .width(2.0),
                );
            });

        // ヒンジ発生履歴（先頭 20 件）
        ui.separator();
        ui.strong("ヒンジ発生履歴");
        egui::ScrollArea::vertical().show(ui, |ui| {
            for h in po.hinges.iter().take(20) {
                let level = match h.level {
                    squid_n_solver::pushover::HingeLevel::Crack => "ひび割れ",
                    squid_n_solver::pushover::HingeLevel::Yield => "降伏",
                    squid_n_solver::pushover::HingeLevel::Ultimate => "終局",
                };
                ui.label(format!(
                    "step {}: 部材 {} pos={:.2} {} (μ={:.2})",
                    h.step, h.elem.0, h.pos, level, h.ductility
                ));
            }
            if po.hinges.len() > 20 {
                ui.label(format!("... 他 {} 件", po.hinges.len() - 20));
            }
        });
    }

    /// 質点系（串団子）モデルの表示。プッシュオーバー結果から層 Q-δ を
    /// トリリニア縮約し、層ごとの質量・階高・復元力特性を一覧する
    /// （構造動力学の質点系解析モデル）。
    pub(crate) fn lumped_mass_panel(&mut self, ui: &mut egui::Ui) {
        use squid_n_solver::lumped_mass::{build_lumped_mass_model, LumpedMassType};

        let Some(po) = self.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "プッシュオーバー結果がありません。質点系モデルは\
                 プッシュオーバー結果から生成します。解析タブから実行してください。",
            );
            return;
        };

        // モデル化タイプ・第1折点判定の割線剛性比を選択。
        ui.horizontal(|ui| {
            ui.label("モデル化タイプ:");
            let cur = self.analysis_cfg.lumped_mass_type;
            egui::ComboBox::from_id_salt("lumped_mass_type")
                .selected_text(cur.label())
                .show_ui(ui, |ui| {
                    for t in [
                        LumpedMassType::EquivalentShear,
                        LumpedMassType::EquivalentBendingShear,
                        LumpedMassType::BendingShearSeparated,
                    ] {
                        ui.selectable_value(&mut self.analysis_cfg.lumped_mass_type, t, t.label());
                    }
                });
            ui.separator();
            ui.label("第1折点 割線比:");
            ui.add(
                egui::DragValue::new(&mut self.analysis_cfg.lumped_secant_ratio)
                    .speed(0.01)
                    .range(0.3..=0.95),
            );
        });
        ui.separator();

        // プッシュオーバーから串団子モデルを生成（軽量なので毎フレーム再構成）。
        let lm = build_lumped_mass_model(
            &self.model,
            po,
            self.analysis_cfg.lumped_mass_type,
            self.analysis_cfg.lumped_secant_ratio,
        );

        let total_mass: f64 = lm.stories.iter().map(|s| s.mass).sum();
        ui.horizontal(|ui| {
            ui.label(format!("質点数: {}", lm.stories.len()));
            ui.separator();
            ui.label(format!("総質量: {:.1} t", total_mass));
            ui.separator();
            ui.label(format!("モデル: {}", lm.model_type.label()));
        });
        ui.separator();

        // 層ごとの質点・復元力特性（トリリニア）を一覧。上層から順に表示。
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::Grid::new("lumped_mass_stories")
                .num_columns(9)
                .striped(true)
                .show(ui, |ui| {
                    ui.strong("階");
                    ui.strong("質量[t]");
                    ui.strong("階高[mm]");
                    ui.strong("K1[kN/mm]");
                    ui.strong("K2[kN/mm]");
                    ui.strong("K3[kN/mm]");
                    ui.strong("第1折点 δ1/Q1");
                    ui.strong("第2折点 δ2/Q2");
                    ui.strong("第3折点 δ3/Q3");
                    ui.end_row();

                    // model.stories と stick は同順（build_lumped_mass_model が順に生成）。
                    for (i, stick) in lm.stories.iter().enumerate().rev() {
                        let name = self
                            .model
                            .stories
                            .get(i)
                            .map(|s| s.name.as_str())
                            .unwrap_or("-");
                        let sk = &stick.skeleton;
                        ui.label(name);
                        ui.label(format!("{:.2}", stick.mass));
                        ui.label(format!("{:.0}", stick.height));
                        ui.label(format!("{:.1}", sk.k1 / 1000.0));
                        ui.label(format!("{:.1}", sk.k2() / 1000.0));
                        ui.label(format!("{:.1}", sk.k3() / 1000.0));
                        ui.label(format!("{:.2} / {:.0}", sk.d1, sk.q1 / 1000.0));
                        ui.label(format!("{:.2} / {:.0}", sk.d2, sk.q2 / 1000.0));
                        ui.label(format!("{:.2} / {:.0}", sk.d3, sk.q3 / 1000.0));
                        ui.end_row();
                    }
                });

            ui.add_space(6.0);
            ui.colored_label(
                crate::theme::GRAY_600,
                "K は [kN/mm]、Q は [kN]、δ は [mm]。骨格はプッシュオーバー層 Q-δ を\
                 等包絡面積則でトリリニア縮約したもの。",
            );
        });

        // ── 質点系（せん断型）時刻歴応答解析 ──────────────────────────
        ui.separator();
        let mut run_stick = false;
        let mut clear_stick = false;
        ui.horizontal(|ui| {
            if ui
                .button("▶ 質点系時刻歴を実行")
                .on_hover_text(
                    "サンプル波（解析設定の dt/継続/周期/振幅・減衰比）で串団子モデルの\
                     非線形時刻歴（Newmark-β、各層トリリニア）を実行します",
                )
                .clicked()
            {
                run_stick = true;
            }
            if self.stick_response.is_some() && ui.button("結果クリア").clicked() {
                clear_stick = true;
            }
        });
        if run_stick {
            let accel = Self::sample_wave(&self.analysis_cfg).accel_x;
            let res = squid_n_solver::lumped_mass::lumped_mass_time_history(
                &lm,
                &accel,
                self.analysis_cfg.th_dt,
                self.analysis_cfg.th_damping,
            );
            self.stick_response = Some(res);
        }
        if clear_stick {
            self.stick_response = None;
        }
        if let Some(res) = &self.stick_response {
            let roof_peak = res
                .roof_disp
                .iter()
                .cloned()
                .fold(0.0f64, |m, v| m.max(v.abs()));
            let mu_max = res.story_ductility.iter().cloned().fold(0.0f64, f64::max);
            ui.horizontal(|ui| {
                ui.label(format!("頂部最大変位: {:.2} mm", roof_peak));
                ui.separator();
                ui.label(format!("最大層塑性率 μ: {:.2}", mu_max));
            });
            egui::Grid::new("stick_th_result")
                .striped(true)
                .num_columns(4)
                .show(ui, |ui| {
                    ui.strong("階");
                    ui.strong("最大層間変形[mm]");
                    ui.strong("最大層せん断[kN]");
                    ui.strong("塑性率μ");
                    ui.end_row();
                    for i in (0..res.story_peak_drift.len()).rev() {
                        let name = self
                            .model
                            .stories
                            .get(i)
                            .map(|s| s.name.as_str())
                            .unwrap_or("-");
                        ui.label(name);
                        ui.label(format!("{:.2}", res.story_peak_drift[i]));
                        ui.label(format!("{:.0}", res.story_peak_shear[i] / 1000.0));
                        ui.label(format!("{:.2}", res.story_ductility[i]));
                        ui.end_row();
                    }
                });
            let pts: Vec<[f64; 2]> = res
                .time
                .iter()
                .zip(res.roof_disp.iter())
                .map(|(&t, &d)| [t, d])
                .collect();
            egui_plot::Plot::new("stick_roof_plot")
                .height(160.0)
                .x_axis_label("時間[s]")
                .y_axis_label("頂部変位[mm]")
                .show(ui, |pu| {
                    pu.line(
                        egui_plot::Line::new("roof", egui_plot::PlotPoints::from(pts))
                            .color(crate::theme::DATA_BLUE),
                    );
                });
        }
    }

    /// レポートタブ：CSV レポートのプレビューとエクスポート。
    pub(crate) fn report_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("レポート");
        if !crate::summary::has_report_content(&self.results) {
            ui.colored_label(
                crate::theme::GRAY_600,
                "解析結果がありません。解析タブから実行するとレポートを生成できます。",
            );
            return;
        }
        let csv = crate::summary::build_report_csv(self);
        ui.horizontal(|ui| {
            if ui.button("💾 CSV エクスポート…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CSV", &["csv"])
                    .set_file_name("report.csv")
                    .save_file()
                {
                    if let Err(e) = std::fs::write(&path, &csv) {
                        self.last_error = Some(format!("レポート保存エラー: {}", e));
                    }
                }
            }
            if ui.button("📋 クリップボードへコピー").clicked() {
                ui.ctx().copy_text(csv.clone());
            }
        });
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add(
                egui::TextEdit::multiline(&mut csv.as_str())
                    .font(egui::TextStyle::Monospace)
                    .desired_width(f32::INFINITY),
            );
        });
    }

    /// 右ペイン：選択要素のインスペクタ。
    /// 3D/ナビゲータ/テーブルの選択（現時点では focus_*）を表示。断面編集は UI-4 で拡充。
    pub(crate) fn inspector_panel(&mut self, ui: &mut egui::Ui) {
        // 遅延アクション（借用チェーン回避：UI 内で self.model を immutable borrow 中に
        // mut borrow できないため、複製ボタンクリックは一旦 here に保存）
        let mut duplicate_member = None;
        let mut highlight_section_members: Option<Vec<ElemId>> = None;
        ui.group(|ui| {
            ui.strong("インスペクタ");
            ui.separator();

            // 選択された部材の諸元
            if let Some(elem_id) = self.nav.focus_member {
                if let Some(e) = self.model.elements.iter().find(|e| e.id == elem_id) {
                    ui.label(format!("部材 ID: {}", e.id.0));
                    let n0 = e.nodes.first().map(|n| n.0).unwrap_or(0);
                    let n1 = e.nodes.get(1).map(|n| n.0).unwrap_or(0);
                    ui.label(format!("節点 I/J: {} / {}", n0, n1));
                    if let Some(sec_id) = e.section {
                        if let Some(sec) = self
                            .model
                            .sections
                            .get(sec_id.index())
                            .filter(|s| s.id == sec_id)
                        {
                            ui.label(format!("断面: {} ({})", sec.name, sec_id.0));
                            ui.label(format!("  A = {:.3e} mm²", sec.area));
                            ui.label(format!("  Iy= {:.3e} mm⁴", sec.iy));
                            ui.label(format!("  Iz= {:.3e} mm⁴", sec.iz));
                            // 影響数: 同一断面を使う部材数
                            let n_used = self
                                .model
                                .elements
                                .iter()
                                .filter(|o| o.section == Some(sec_id))
                                .count();
                            ui.colored_label(
                                crate::theme::BLUE_500,
                                format!("この断面を使う {} 部材に影響", n_used),
                            );
                            // UI-4: 複製ボタン（UI設計 §3）。同断面を新規IDで複製し、
                            // 当該部材のみ新断面に割当。
                            if ui.button("📋 複製してこの部材だけ別断面に").clicked()
                            {
                                duplicate_member = Some(elem_id);
                            }
                        }
                    } else {
                        ui.label("断面: 未割当");
                    }
                    if let Some(mat_id) = e.material {
                        if let Some(mat) = self
                            .model
                            .materials
                            .get(mat_id.index())
                            .filter(|m| m.id == mat_id)
                        {
                            ui.label(format!("材料: {} ({})", mat.name, mat_id.0));
                            ui.label(format!("  E = {:.1} N/mm²", mat.young));
                            if let Some(fc) = mat.fc {
                                ui.label(format!("  Fc = {:.1} N/mm²", fc));
                            }
                        }
                    }
                    ui.separator();
                    // 検定結果サマリ（同一部材）
                    if let Some(r) = &self.results {
                        let my_checks: Vec<_> = r
                            .checks
                            .iter()
                            .filter(|(id, _, _)| *id == elem_id)
                            .collect();
                        ui.label(format!("検定結果（{} 位置）", my_checks.len()));
                        for (_, pos, cr) in my_checks.iter().take(8) {
                            let ratio = cr.ratio;
                            let color = crate::theme::status_color(ratio);
                            ui.colored_label(
                                color,
                                format!("  pos={:.2} 検定比={:.3}", pos, ratio),
                            );
                        }
                        if my_checks.len() > 8 {
                            ui.label(format!("  ... 他 {} 件", my_checks.len() - 8));
                        }
                    }
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "部材を選択してください");
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(150, 150, 150),
                    "部材を選択してください",
                );
            }

            // 選択された断面の諸元（断面テーブルの行選択と連動）
            if let Some(sec_id) = self.nav.focus_section {
                if let Some(sec) = self.model.sections.iter().find(|s| s.id == sec_id) {
                    ui.separator();
                    ui.strong("断面（選択中）");
                    ui.label(format!("名前: {} ({})", sec.name, sec_id.0));
                    ui.label(format!("  A = {:.3e} mm²", sec.area));
                    ui.label(format!("  Iy= {:.3e} mm⁴", sec.iy));
                    ui.label(format!("  Iz= {:.3e} mm⁴", sec.iz));
                    let used: Vec<ElemId> = self
                        .model
                        .elements
                        .iter()
                        .filter(|e| e.section == Some(sec_id))
                        .map(|e| e.id)
                        .collect();
                    ui.label(format!("使用部材数: {}", used.len()));
                    if ui.button("🔍 使用部材を3Dハイライト").clicked() {
                        highlight_section_members = Some(used);
                    }
                }
            }

            ui.separator();
            // 選択された節点の諸元
            if let Some(node_id) = self.nav.focus_node {
                if let Some(node) = self.model.nodes.iter().find(|n| n.id == node_id) {
                    ui.label(format!("節点 ID: {}", node.id.0));
                    ui.label(format!(
                        "座標: ({:.3}, {:.3}, {:.3})",
                        node.coord[0], node.coord[1], node.coord[2]
                    ));
                    // 拘束情報
                    let is_fixed = node.restraint.0 != 0;
                    if is_fixed {
                        ui.label("拘束: あり");
                    } else {
                        ui.label("拘束: なし");
                    }
                }
            }
        });

        // 遅延実行: 複製ボタンが押されていたら EditCommand を叩く
        if let Some(member) = duplicate_member {
            self.undo.run(
                &mut self.model,
                Box::new(squid_n_edit::DuplicateSectionForMember { member }),
            );
            self.staleness.mark_edited();
        }
        // 遅延実行: 断面の使用部材ハイライトボタン
        if let Some(members) = highlight_section_members {
            self.selection.members = members;
        }
    }

    /// 下部ステータスバー。
    pub(crate) fn status_bar(&self, ui: &mut egui::Ui) {
        ui.separator();
        ui.horizontal(|ui| {
            // プロジェクトファイル名 + 未保存マーカー
            let file_label = self
                .project_path
                .as_ref()
                .and_then(|p| p.file_name())
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_else(|| "(未保存プロジェクト)".to_string());
            let marker = if self.staleness.unsaved_changes {
                " ●"
            } else {
                ""
            };
            ui.label(format!("{}{}", file_label, marker));
            ui.separator();
            // バックグラウンド解析ジョブの実行状況
            if let Some(job) = &self.job {
                let elapsed = job.started.elapsed().unwrap_or_default().as_secs_f64();
                ui.colored_label(
                    crate::theme::GOOD_GREEN,
                    format!("⏳ {} 実行中… {:.0}s", job.label, elapsed),
                );
                ui.separator();
            }
            // stale アイコン
            if self.staleness.results_stale {
                ui.colored_label(crate::theme::BEST_YELLOW, "⚠ stale");
            } else if self.results.is_some() {
                ui.colored_label(crate::theme::GOOD_GREEN, "✓ 最新");
            } else {
                ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
            }
            ui.separator();
            if let Some(err) = &self.last_error {
                ui.colored_label(crate::theme::ERROR_RED, format!("⚠ {}", err));
            }
            ui.separator();
            ui.label(format!(
                "部材 {}. 節点 {}. 断面 {}.",
                self.model.elements.len(),
                self.model.nodes.len(),
                self.model.sections.len()
            ));
        });
    }
}
