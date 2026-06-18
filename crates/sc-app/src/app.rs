use sc_core::ids::{ElemId, LoadCaseId};
use sc_design_jp::{DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, RcDesign, SteelDesign};
use sc_edit::UndoStack;
use sc_solver::analysis::{AiMode, Analysis, SeismicDir};

pub enum Tab {
    Nodes,
    Members,
    Sections,
    Loads,
    Viewer,
    Design,
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
}

impl Default for App {
    fn default() -> Self {
        Self {
            model: sc_core::model::Model::default(),
            results: None,
            selection: Selection::default(),
            undo: UndoStack::new(),
            active_tab: Tab::Nodes,
            design_term: LoadTerm::Long,
            last_lc: None,
            last_error: None,
            node_edit: Vec::new(),
        }
    }
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
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // 上部ツールバー: タブ切替 + Undo/Redo + 解析実行 + 荷重継続性
        ui.horizontal(|ui| {
            let tabs = [
                ("節点", Tab::Nodes),
                ("部材", Tab::Members),
                ("断面", Tab::Sections),
                ("荷重", Tab::Loads),
                ("3D", Tab::Viewer),
                ("設計", Tab::Design),
            ];
            for (label, tab) in &tabs {
                let selected =
                    std::mem::discriminant(&self.active_tab) == std::mem::discriminant(tab);
                if ui.selectable_label(selected, *label).clicked() {
                    self.active_tab = discriminant_to_tab(tab);
                }
            }
            ui.separator();
            // T2: Undo/Redo
            let can_undo = self.undo.can_undo();
            let can_redo = self.undo.can_redo();
            if ui
                .add_enabled(can_undo, egui::Button::new("↶ Undo"))
                .clicked()
            {
                self.undo.undo(&mut self.model);
            }
            if ui
                .add_enabled(can_redo, egui::Button::new("↷ Redo"))
                .clicked()
            {
                self.undo.redo(&mut self.model);
            }
            ui.separator();
            // T3: 解析実行
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
            ui.separator();
            // 荷重継続性区分
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

        // エラーメッセージ
        if let Some(err) = &self.last_error {
            ui.colored_label(egui::Color32::RED, format!("⚠ {}", err));
            ui.separator();
        }

        // 解析結果サマリ
        if let Some(r) = &self.results {
            ui.horizontal(|ui| {
                ui.label(format!("静的ケース数: {}", r.statics.len()));
                if let Some(m) = &r.modal {
                    let t1 = m.period.first().copied().unwrap_or(0.0);
                    ui.label(format!("固有周期 T1: {:.3} s", t1));
                }
                ui.label(format!("検定結果数: {}", r.checks.len()));
            });
            ui.separator();
        }

        // 各タブの内容
        egui::ScrollArea::vertical().show(ui, |ui| match self.active_tab {
            Tab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            Tab::Members => crate::tables::members::members_table(ui, self),
            Tab::Sections => crate::tables::sections::sections_table(ui, self),
            Tab::Loads => crate::tables::loads::loads_table(ui, self),
            Tab::Viewer => crate::viewer::viewer_panel(ui, self),
            Tab::Design => crate::design_view::design_table(ui, self),
        });
    }
}

#[cfg(feature = "gui")]
fn discriminant_to_tab(tab: &Tab) -> Tab {
    match tab {
        Tab::Nodes => Tab::Nodes,
        Tab::Members => Tab::Members,
        Tab::Sections => Tab::Sections,
        Tab::Loads => Tab::Loads,
        Tab::Viewer => Tab::Viewer,
        Tab::Design => Tab::Design,
    }
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
}
