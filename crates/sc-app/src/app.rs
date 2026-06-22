use std::time::SystemTime;

use sc_core::ids::{ElemId, LoadCaseId, NodeId, SectionId};
use sc_design_jp::{DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, RcDesign, SteelDesign};
use sc_edit::UndoStack;
use sc_solver::analysis::{AiMode, Analysis, SeismicDir};

/// 工程タブ（UI設計 §1.1）。進行ロックしない。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Model,
    Loads,
    Analysis,
    Results,
    Design,
    Report,
}

/// モデルタブ内のサブタブ。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ModelTab {
    #[default]
    Nodes,
    Members,
    Sections,
    Materials,
}

/// 結果タブ内の切替（3D 各種図と時刻歴グラフ）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ResultsView {
    #[default]
    Spatial,
    TimeHistory,
}

/// ナビゲータ（左ペイン）。階/部材群/ケース系の選択状態。
#[derive(Default)]
pub struct Navigator {
    pub expanded_floors: bool,
    pub expanded_groups: bool,
    pub expanded_load_cases: bool,
    pub expanded_result_cases: bool,
    pub focus_node: Option<NodeId>,
    pub focus_member: Option<ElemId>,
    pub focus_section: Option<SectionId>,
    pub focus_load_case: Option<LoadCaseId>,
}

/// stale（要再計算）状態（UI設計 §5）。
#[derive(Clone, Debug, Default)]
pub struct Staleness {
    pub results_stale: bool,
    pub design_stale: bool,
    pub last_run: Option<SystemTime>,
}

impl Staleness {
    /// モデル/荷重が編集された → 下流を stale にする。
    pub fn mark_edited(&mut self) {
        self.results_stale = true;
        self.design_stale = true;
    }
    /// 解析が完了 → 最新化する。
    pub fn mark_fresh(&mut self) {
        self.results_stale = false;
        self.design_stale = false;
        self.last_run = Some(SystemTime::now());
    }
}

#[derive(Default)]
pub struct Selection {
    pub nodes: Vec<sc_core::ids::NodeId>,
    pub members: Vec<sc_core::ids::ElemId>,
}

pub struct ResultsBundle {
    pub statics: Vec<(LoadCaseId, sc_solver::linear::StaticOnce)>,
    pub modal: Option<sc_solver::eigen::ModalResult>,
    pub member_forces: Vec<(ElemId, sc_element::beam::MemberForces)>,
    pub checks: Vec<(ElemId, f64, sc_design_jp::CheckResult)>,
}

pub struct App {
    pub model: sc_core::model::Model,
    pub results: Option<ResultsBundle>,
    pub selection: Selection,
    pub undo: UndoStack,
    pub active_tab: Tab,
    /// 設計検定の荷重継続性区分（長期／短期）
    pub design_term: LoadTerm,
    /// 最後に実行した荷重ケース ID
    pub last_lc: Option<LoadCaseId>,
    /// 解析実行中のエラーメッセージ
    pub last_error: Option<String>,
    /// 節点座標の編集バッファ（model.nodes に同期）
    pub node_edit: Vec<[String; 3]>,
    /// stale（要再計算）状態と最終実行時刻
    pub staleness: Staleness,
    /// ナビゲータ（左ペイン）状態
    pub nav: Navigator,
    /// モデルタブ内のサブタブ
    pub model_tab: ModelTab,
    /// 結果タブ内の表示切替（3D / 時刻歴）
    #[cfg(feature = "gui")]
    pub results_view: ResultsView,
    /// ビューアの表示モード
    #[cfg(feature = "gui")]
    pub view_mode: crate::viewer::ViewMode,
    /// 変形図・モード形の倍率スライダー値
    #[cfg(feature = "gui")]
    pub deform_scale: f32,
    /// モード形の表示インデックス
    #[cfg(feature = "gui")]
    pub view_mode_idx: usize,
    /// ビューアのカメラ状態
    #[cfg(feature = "gui")]
    pub camera: crate::viewer::CameraState,
    /// 床荷重分配の CMQ 結果（P2 §5.1）。描画用。
    pub beam_loads: Vec<sc_load::floor::BeamLoad>,
    /// 時刻歴応答データ（描画用）
    #[cfg(feature = "gui")]
    pub time_history_data: crate::time_history_view::TimeHistoryData,
    /// 時刻歴グラフの表示項目選択
    #[cfg(feature = "gui")]
    pub time_history_source: crate::time_history_view::TimeHistorySource,
    /// 断面作成UI のドラフト（UI-3）
    #[cfg(feature = "gui")]
    pub section_draft: crate::section_editor::SectionEditorDraft,
}

impl Default for App {
    fn default() -> Self {
        Self {
            model: sc_core::model::Model::default(),
            results: None,
            selection: Selection::default(),
            undo: UndoStack::new(),
            active_tab: Tab::Model,
            design_term: LoadTerm::Long,
            last_lc: None,
            last_error: None,
            node_edit: Vec::new(),
            staleness: Staleness::default(),
            nav: Navigator::default(),
            model_tab: ModelTab::default(),
            #[cfg(feature = "gui")]
            results_view: ResultsView::default(),
            #[cfg(feature = "gui")]
            view_mode: crate::viewer::ViewMode::default(),
            #[cfg(feature = "gui")]
            deform_scale: 100.0,
            #[cfg(feature = "gui")]
            view_mode_idx: 0,
            #[cfg(feature = "gui")]
            camera: crate::viewer::CameraState::default(),
            beam_loads: Vec::new(),
            #[cfg(feature = "gui")]
            time_history_data: crate::time_history_view::dummy_time_history(),
            #[cfg(feature = "gui")]
            time_history_source: crate::time_history_view::TimeHistorySource::default(),
            #[cfg(feature = "gui")]
            section_draft: crate::section_editor::SectionEditorDraft::default(),
        }
    }
}

/// 日本語フォントを egui に登録する（UI のラベルが豆腐□にならないように）。
///
/// egui の既定フォント（Ubuntu/Hack）は日本語グリフを持たないため、
/// OS のシステムフォントから日本語対応フォントを探して読み込む。
/// 見つからない場合は何もしない（英数字は既定フォントで表示される）。
pub fn install_japanese_fonts(ctx: &egui::Context) {
    // OS ごとの代表的な日本語フォント候補（先に見つかったものを使用）。
    const CANDIDATES: &[&str] = &[
        // Windows
        "C:/Windows/Fonts/meiryo.ttc",
        "C:/Windows/Fonts/YuGothR.ttc",
        "C:/Windows/Fonts/YuGothM.ttc",
        "C:/Windows/Fonts/msgothic.ttc",
        // macOS
        "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        "/Library/Fonts/Osaka.ttf",
        // Linux (Noto / IPA / VL)
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf",
        "/usr/share/fonts/truetype/vlgothic/VL-Gothic-Regular.ttf",
    ];

    let Some((path, bytes)) = CANDIDATES
        .iter()
        .find_map(|p| std::fs::read(p).ok().map(|b| (*p, b)))
    else {
        eprintln!(
            "[warn] 日本語フォントが見つかりませんでした。UI が文字化けする可能性があります。"
        );
        return;
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "jp".to_owned(),
        std::sync::Arc::new(egui::FontData::from_owned(bytes)),
    );
    // プロポーショナル・等幅の両ファミリーで日本語フォントを最優先にする。
    if let Some(family) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
        family.insert(0, "jp".to_owned());
    }
    if let Some(family) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
        family.push("jp".to_owned());
    }
    ctx.set_fonts(fonts);
    eprintln!("[info] 日本語フォントを読み込みました: {path}");
}

impl App {
    /// 節点編集バッファを model.nodes に同期する。
    /// 編集中でない（フォーカス外）セルのみ model 値で更新する。
    pub fn sync_node_edit(&mut self) {
        self.node_edit.resize(
            self.model.nodes.len(),
            ["0".to_string(), "0".to_string(), "0".to_string()],
        );
        for (i, node) in self.model.nodes.iter().enumerate() {
            for (k, slot) in self.node_edit[i].iter_mut().enumerate().take(3) {
                *slot = format!("{:.3}", node.coord[k]);
            }
        }
    }

    /// T3: 線形静的解析を実行し、結果を `self.results` に格納する。
    /// 指定した荷重ケースが存在しない場合はエラーメッセージをセット。
    pub fn run_linear_static(&mut self, lc: LoadCaseId) {
        self.last_error = None;
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.linear_static(lc) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or(ResultsBundle {
                        statics: Vec::new(),
                        modal: None,
                        member_forces: Vec::new(),
                        checks: Vec::new(),
                    });
                    bundle.statics.retain(|(id, _)| *id != lc);
                    bundle.statics.push((lc, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_lc = Some(lc);
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("線形静的解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// T3: 固有値解析を実行し、結果を `self.results` に格納する。
    pub fn run_eigen(&mut self, n_modes: usize) {
        self.last_error = None;
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.eigen(n_modes) {
                Ok(modal) => {
                    let mut bundle = self.results.take().unwrap_or(ResultsBundle {
                        statics: Vec::new(),
                        modal: None,
                        member_forces: Vec::new(),
                        checks: Vec::new(),
                    });
                    bundle.modal = Some(modal);
                    self.results = Some(bundle);
                    // 固有値のみの更新では設計は更新されないが、最新実行時刻は更新
                    self.staleness.last_run = Some(SystemTime::now());
                }
                Err(e) => self.last_error = Some(format!("固有値解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// T3: 地震静的解析（Ai一気通貫）を実行し、結果を `self.results` に格納する。
    pub fn run_seismic(&mut self, dir: SeismicDir) {
        self.last_error = None;
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.seismic_static(dir, AiMode::SemiPrecise) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or(ResultsBundle {
                        statics: Vec::new(),
                        modal: None,
                        member_forces: Vec::new(),
                        checks: Vec::new(),
                    });
                    bundle.statics.retain(|(id, _)| *id != LoadCaseId(0));
                    bundle.statics.push((LoadCaseId(0), res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_lc = Some(LoadCaseId(0));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("地震解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// T7: 解析結果の member_forces から検定結果を生成する。
    /// 危険断面位置（eval_sections）の内力に対し、材種に応じて
    /// SteelDesign / RcDesign を適用する。
    pub fn run_design_check(&mut self) {
        let Some(results) = &self.results else {
            return;
        };
        let ctx = DesignCtx {
            term: self.design_term,
        };
        let mut checks: Vec<(ElemId, f64, sc_design_jp::CheckResult)> = Vec::new();
        for (elem_id, mf) in &results.member_forces {
            let elem = self.model.elements.iter().find(|e| e.id == *elem_id);
            let Some(elem) = elem else {
                continue;
            };
            let sec = elem
                .section
                .and_then(|sid| self.model.sections.get(sid.index()))
                .filter(|s| s.id == elem.section.unwrap());
            let mat = elem
                .material
                .and_then(|mid| self.model.materials.get(mid.index()))
                .filter(|m| m.id == elem.material.unwrap());
            let (Some(sec), Some(mat)) = (sec, mat) else {
                continue;
            };

            let checker: Box<dyn DesignCheck> = if is_steel(&mat.name) {
                Box::new(SteelDesign)
            } else {
                Box::new(RcDesign)
            };

            for (pos, forces) in &mf.at {
                // [N, Qy, Qz, Mx, My, Mz] -> MemberForcesAt
                // 暫定: 強軸まわりとして Mz[5] と Qy[1] を使用
                let mfa = MemberForcesAt {
                    pos: *pos,
                    n: forces[0],
                    q: forces[1],
                    m: forces[5],
                };
                let cr = checker.check(&mfa, sec, mat, &ctx);
                checks.push((*elem_id, *pos, cr));
            }
        }
        if let Some(bundle) = self.results.as_mut() {
            bundle.checks = checks;
        }
    }
}

/// 鋼材判定（Material.name に "S" で始まる JIS 鋼種名が含まれるか）。
fn is_steel(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
}

#[cfg(feature = "gui")]
impl eframe::App for App {
    #![allow(deprecated)] // 4ペイン分割に allocate_ui_at_rect を使用（deprecation は cosmetic）
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // 上部ツールバー: 工程タブ（自由遷移）+ Undo/Redo
        ui.horizontal(|ui| {
            let tabs = [
                ("モデル", Tab::Model),
                ("荷重", Tab::Loads),
                ("解析", Tab::Analysis),
                ("結果", Tab::Results),
                ("設計", Tab::Design),
                ("レポート", Tab::Report),
            ];
            for (label, tab) in &tabs {
                let selected = self.active_tab == *tab;
                let stale_marker = match *tab {
                    // 進行中の下流タブに stale バッジを付与（§5）
                    Tab::Results | Tab::Design if self.staleness.results_stale => "⚠",
                    _ => "",
                };
                let label_str = format!("{} {}", label, stale_marker);
                if ui.selectable_label(selected, label_str).clicked() {
                    self.active_tab = *tab;
                }
            }
            ui.separator();
            let can_undo = self.undo.can_undo();
            let can_redo = self.undo.can_redo();
            if ui
                .add_enabled(can_undo, egui::Button::new("↶ Undo"))
                .clicked()
            {
                self.undo.undo(&mut self.model);
                self.staleness.mark_edited();
            }
            if ui
                .add_enabled(can_redo, egui::Button::new("↷ Redo"))
                .clicked()
            {
                self.undo.redo(&mut self.model);
                self.staleness.mark_edited();
            }
            ui.separator();
            // 荷重継続性区分は設計タブと関係するが、共有コントロールとして上部に残置
            ui.label("荷重:");
            let term = self.design_term;
            if ui
                .selectable_label(term == LoadTerm::Long, "長期")
                .clicked()
            {
                self.design_term = LoadTerm::Long;
                self.run_design_check();
            }
            if ui
                .selectable_label(term == LoadTerm::Short, "短期")
                .clicked()
            {
                self.design_term = LoadTerm::Short;
                self.run_design_check();
            }
        });
        ui.separator();

        // 4ペイン：左ナビゲータ / 中央 / 右インスペクタ / 下ステータス
        // 下パネルは描画の都合上最後に置く。左右は egui::SidePanel を模して available_rect で分割。
        let available = ui.available_rect_before_wrap();
        let nav_width = 180.0;
        let inspector_width = 240.0;
        let status_height = 22.0;

        let nav_rect = egui::Rect {
            min: available.min,
            max: egui::pos2(available.min.x + nav_width, available.max.y - status_height),
        };
        let inspector_rect = egui::Rect {
            min: egui::pos2(available.max.x - inspector_width, available.min.y),
            max: egui::pos2(available.max.x, available.max.y - status_height),
        };
        let central_rect = egui::Rect {
            min: egui::pos2(nav_rect.max.x, available.min.y),
            max: egui::pos2(inspector_rect.min.x, available.max.y - status_height),
        };
        let status_rect = egui::Rect {
            min: egui::pos2(available.min.x, available.max.y - status_height),
            max: available.max,
        };

        // 左：ナビゲータ
        ui.allocate_ui_at_rect(nav_rect, |ui| {
            self.navigator_panel(ui);
        });

        // 中央：工程タブの内容
        ui.allocate_ui_at_rect(central_rect, |ui| match self.active_tab {
            Tab::Model => self.model_tab_panel(ui),
            Tab::Loads => crate::tables::loads::loads_table(ui, self),
            Tab::Analysis => self.analysis_tab_panel(ui),
            Tab::Results => self.results_tab_panel(ui),
            Tab::Design => crate::design_view::design_table(ui, self),
            Tab::Report => self.report_tab_panel(ui),
        });

        // 右：インスペクタ
        ui.allocate_ui_at_rect(inspector_rect, |ui| {
            self.inspector_panel(ui);
        });

        // 下：ステータスバー
        ui.allocate_ui_at_rect(status_rect, |ui| {
            self.status_bar(ui);
        });
    }
}

#[cfg(feature = "gui")]
impl App {
    /// 左ペイン：ナビゲータ（階/部材群/荷重ケース/結果ケースのツリー）。
    fn navigator_panel(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.strong("ナビゲータ");
            ui.separator();

            // 部材グループ（簡易: 材種ごと）
            let header = egui::CollapsingHeader::new("部材グループ")
                .default_open(true)
                .id_salt("nav_groups");
            header.show(ui, |ui| {
                let steel_count = self
                    .model
                    .elements
                    .iter()
                    .filter(|e| {
                        e.material
                            .and_then(|mid| self.model.materials.get(mid.index()))
                            .map(|m| is_steel(&m.name))
                            .unwrap_or(false)
                    })
                    .count();
                let rc_count = self.model.elements.len().saturating_sub(steel_count);
                if ui
                    .selectable_label(
                        self.nav.focus_member.is_none(),
                        format!("鋼材部材 ({})", steel_count),
                    )
                    .clicked()
                {
                    self.nav.focus_member = None;
                }
                if ui
                    .selectable_label(
                        self.nav.focus_member.is_none(),
                        format!("RC部材 ({})", rc_count),
                    )
                    .clicked()
                {
                    self.nav.focus_member = None;
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

            // 結果メモ（簡易：静的ケース数を列挙）
            let header = egui::CollapsingHeader::new("結果ケース")
                .default_open(true)
                .id_salt("nav_result_cases");
            header.show(ui, |ui| {
                if let Some(r) = &self.results {
                    if r.statics.is_empty() && r.modal.is_none() {
                        ui.label("（未実行）");
                    } else {
                        for (i, (id, _)) in r.statics.iter().enumerate() {
                            ui.label(format!("静的 #{} LC {}", i, id.0));
                        }
                        if r.modal.is_some() {
                            ui.label("固有値");
                        }
                    }
                } else {
                    ui.label("（未実行）");
                }
            });

            // 階/レベル（未実装だがグループ表示のみ用意）
            let _ = ui.collapsing("階/レベル", |ui| {
                ui.label("（未実装: P4 以降）");
            });
        });
    }

    /// モデルタブ：サブタブ切替で節点/部材/断面/材料を編集するテーブルを表示。
    fn model_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let subs = [
                ("節点", ModelTab::Nodes),
                ("部材", ModelTab::Members),
                ("断面", ModelTab::Sections),
                ("材料", ModelTab::Materials),
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
            ModelTab::Members => crate::tables::members::members_table(ui, self),
            ModelTab::Sections => {
                crate::tables::sections::sections_table(ui, self);
                ui.add_space(8.0);
                crate::section_editor::section_editor_panel(ui, self);
            }
            ModelTab::Materials => materials_panel(ui, self),
        }
    }

    /// 解析タブ：種別選択＋実行＋進捗表示。
    fn analysis_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("解析設定");
        ui.separator();

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
                egui::Color32::from_rgb(220, 140, 0),
                "⚠ モデルが編集されました。結果は再計算が必要です。",
            );
        }
        ui.separator();

        ui.label("種別:");
        if ui.button("線形静的").clicked() {
            if let Some(lc) = self.model.load_cases.first().map(|c| c.id) {
                self.run_linear_static(lc);
            } else {
                self.last_error = Some("荷重ケースがありません".to_string());
            }
        }
        if ui.button("固有値").clicked() {
            self.run_eigen(3);
        }
        if ui.button("地震(Ai)").clicked() {
            self.run_seismic(SeismicDir::X);
        }
    }

    /// 結果タブ：3Dビューア と 時刻歴グラフを切替。
    fn results_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let sel_spatial = self.results_view == ResultsView::Spatial;
            let sel_th = self.results_view == ResultsView::TimeHistory;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.results_view = ResultsView::TimeHistory;
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
                ui.colored_label(egui::Color32::from_rgb(150, 150, 150), "▷ 未実行");
            }
        });
        ui.separator();
        match self.results_view {
            ResultsView::Spatial => crate::viewer::viewer_panel(ui, self),
            ResultsView::TimeHistory => crate::time_history_view::time_history_panel(ui, self),
        }
    }

    /// レポートタブ：P9 で実装予定（プレースホルダー）。
    fn report_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("レポート");
        ui.label("P9 仕上げフェーズで実装予定（PDF / Excel / CSV の章選択出力）");
    }

    /// 右ペイン：選択要素のインスペクタ。
    /// 3D/ナビゲータ/テーブルの選択（現時点では focus_*）を表示。断面編集は UI-4 で拡充。
    fn inspector_panel(&mut self, ui: &mut egui::Ui) {
        // 遅延アクション（借用チェーン回避：UI 内で self.model を immutable borrow 中に
        // mut borrow できないため、複製ボタンクリックは一旦 here に保存）
        let mut duplicate_member = None;
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
                                egui::Color32::from_rgb(70, 110, 200),
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
                            let color = if ratio <= 0.8 {
                                egui::Color32::from_rgb(60, 160, 60)
                            } else if ratio <= 1.0 {
                                egui::Color32::from_rgb(200, 180, 40)
                            } else {
                                egui::Color32::from_rgb(220, 60, 60)
                            };
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
                    ui.colored_label(
                        egui::Color32::from_rgb(150, 150, 150),
                        "部材を選択してください",
                    );
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(150, 150, 150),
                    "部材を選択してください",
                );
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
                Box::new(sc_edit::DuplicateSectionForMember { member }),
            );
            self.staleness.mark_edited();
        }
    }

    /// 下部ステータスバー。
    fn status_bar(&self, ui: &mut egui::Ui) {
        ui.separator();
        ui.horizontal(|ui| {
            // stale アイコン
            if self.staleness.results_stale {
                ui.colored_label(egui::Color32::from_rgb(220, 140, 0), "⚠ stale");
            } else if self.results.is_some() {
                ui.colored_label(egui::Color32::from_rgb(60, 160, 60), "✓ 最新");
            } else {
                ui.colored_label(egui::Color32::from_rgb(150, 150, 150), "▷ 未実行");
            }
            ui.separator();
            if let Some(err) = &self.last_error {
                ui.colored_label(egui::Color32::RED, format!("⚠ {}", err));
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

/// 材料テーブル（既存の beginners テーブル相当。簡易表示のみ）。
#[cfg(feature = "gui")]
fn materials_panel(ui: &mut egui::Ui, app: &mut App) {
    let n = app.model.materials.len();
    ui.label(format!("材料一覧（{} 件）", n));
    ui.separator();
    use egui_extras::{Column, TableBuilder};
    TableBuilder::new(ui)
        .striped(true)
        .resizable(true)
        .column(Column::auto().resizable(true))
        .column(Column::auto())
        .column(Column::remainder())
        .header(20.0, |mut header| {
            header.col(|ui| {
                ui.strong("ID");
            });
            header.col(|ui| {
                ui.strong("名称");
            });
            header.col(|ui| {
                ui.strong("E [N/mm²]");
            });
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let idx = row.index();
                let mat = &app.model.materials[idx];
                row.col(|ui| {
                    ui.label(format!("{}", mat.id.0));
                });
                row.col(|ui| {
                    ui.label(&mat.name);
                });
                row.col(|ui| {
                    ui.label(format!("{:.1}", mat.young));
                });
            });
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_steel() {
        assert!(is_steel("SN400"));
        assert!(is_steel("SS400"));
        assert!(is_steel("SM490"));
        assert!(!is_steel("SD345"));
        assert!(!is_steel(" Concrete"));
    }

    #[test]
    fn test_run_design_check_empty_model() {
        let mut app = App::default();
        app.run_design_check();
        assert!(app.results.is_none() || app.results.as_ref().unwrap().checks.is_empty());
    }

    #[test]
    fn test_staleness_mark_edited_marks_downstream() {
        let mut s = Staleness::default();
        assert!(!s.results_stale);
        s.mark_edited();
        assert!(s.results_stale);
        assert!(s.design_stale);
        let now = SystemTime::now();
        s.last_run = Some(now);
        s.mark_fresh();
        assert!(!s.results_stale);
        assert!(!s.design_stale);
        assert!(s.last_run.is_some());
    }

    #[test]
    fn test_tab_default_is_model() {
        assert_eq!(Tab::Model, Tab::default());
    }
}
