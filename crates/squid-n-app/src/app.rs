use std::time::SystemTime;

use squid_n_core::ids::{ElemId, LoadCaseId, NodeId, SectionId};
use squid_n_design_jp::{DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, RcDesign, SteelDesign};
use squid_n_edit::UndoStack;
use squid_n_solver::analysis::{AiMode, Analysis, SeismicDir};

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
    BoundaryConditions,
    Members,
    Sections,
    Materials,
}

/// 結果タブ内の切替（3D 各種図・時刻歴グラフ・プッシュオーバー曲線）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ResultsView {
    #[default]
    Spatial,
    TimeHistory,
    Pushover,
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
    /// ファイル保存後に編集があったか（タイトル/ステータスの未保存マーカー用）。
    /// `mark_fresh`（解析完了）ではクリアされず、保存/読込時のみクリアする。
    pub unsaved_changes: bool,
}

impl Staleness {
    /// モデル/荷重が編集された → 下流を stale にする。
    pub fn mark_edited(&mut self) {
        self.results_stale = true;
        self.design_stale = true;
        self.unsaved_changes = true;
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
    pub nodes: Vec<squid_n_core::ids::NodeId>,
    pub members: Vec<squid_n_core::ids::ElemId>,
}

#[derive(Default)]
pub struct ResultsBundle {
    pub statics: Vec<(LoadCaseId, squid_n_solver::linear::StaticOnce)>,
    pub modal: Option<squid_n_solver::eigen::ModalResult>,
    pub member_forces: Vec<(ElemId, squid_n_element::beam::MemberForces)>,
    pub checks: Vec<(ElemId, f64, squid_n_design_jp::CheckResult)>,
    pub pushover: Option<squid_n_solver::pushover::PushoverResult>,
    pub time_history: Option<squid_n_solver::timehistory::ResponseResult>,
}

/// 解析タブの設定値（GUI 非依存。テストからも使う）。
#[derive(Clone, Copy, Debug)]
pub struct AnalysisSettings {
    /// 固有値解析のモード数
    pub n_modes: usize,
    /// 地震静的(Ai)の方向・Ai算定法・地域係数・地盤種別・標準せん断力係数
    pub seismic_dir: SeismicDir,
    pub ai_mode: AiMode,
    pub z: f64,
    pub soil: squid_n_load::ai::SoilClass,
    pub c0: f64,
    /// プッシュオーバー: 方向・最大ステップ・目標変位 [mm]
    pub push_dir: SeismicDir,
    pub push_steps: usize,
    pub push_max_disp: f64,
    /// 時刻歴: 減衰比・サンプル波の刻み/継続時間/周期/振幅 [mm/s²]
    pub th_damping: f64,
    pub th_dt: f64,
    pub th_duration: f64,
    pub th_period: f64,
    pub th_amp: f64,
}

impl Default for AnalysisSettings {
    fn default() -> Self {
        Self {
            n_modes: 3,
            seismic_dir: SeismicDir::X,
            ai_mode: AiMode::SemiPrecise,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
            push_dir: SeismicDir::X,
            push_steps: 50,
            push_max_disp: 500.0,
            th_damping: 0.02,
            th_dt: 0.01,
            th_duration: 10.0,
            th_period: 0.5,
            th_amp: 1000.0,
        }
    }
}

pub struct App {
    pub model: squid_n_core::model::Model,
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
    /// 節点追加フォームの入力中座標（境界条件の編集とは別の独立 UI）
    pub node_draft: [String; 3],
    /// stale（要再計算）状態と最終実行時刻
    pub staleness: Staleness,
    /// ナビゲータ（左ペイン）状態
    pub nav: Navigator,
    /// モデルタブ内のサブタブ
    pub model_tab: ModelTab,
    /// 左ペインの幅（px）。ドラッグで調整可能（180–520 にクランプ）。
    #[cfg(feature = "gui")]
    pub left_panel_width: f32,
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
    pub beam_loads: Vec<squid_n_load::floor::BeamLoad>,
    /// 時刻歴応答データ（描画用）
    #[cfg(feature = "gui")]
    pub time_history_data: crate::time_history_view::TimeHistoryData,
    /// 時刻歴グラフの表示項目選択
    #[cfg(feature = "gui")]
    pub time_history_source: crate::time_history_view::TimeHistorySource,
    /// 断面作成UI のドラフト（UI-3）
    #[cfg(feature = "gui")]
    pub section_draft: crate::section_editor::SectionEditorDraft,
    /// 断面カタログ選択UI のドラフト（Shape→Family→Name）
    #[cfg(feature = "gui")]
    pub catalog_draft: crate::section_editor::CatalogDraft,
    /// ビューアの梁作成モード（ON 中はクリックで節点を選び 2 点で梁を作る）
    #[cfg(feature = "gui")]
    pub beam_draw_mode: bool,
    /// 梁作成モードで選択済みの始点節点（2 点目で梁を生成しリセット）
    #[cfg(feature = "gui")]
    pub beam_draw_first: Option<squid_n_core::ids::NodeId>,
    /// ビューアの壁作成モード（ON 中はクリックで節点を選び 4 点で壁を作る）
    #[cfg(feature = "gui")]
    pub wall_draw_mode: bool,
    /// 壁作成モードで選択済みの節点（4 点目で壁を生成しリセット）
    #[cfg(feature = "gui")]
    pub wall_draw_nodes: Vec<squid_n_core::ids::NodeId>,
    /// 現在のプロジェクトファイル（.scz）パス。未保存なら None。
    pub project_path: Option<std::path::PathBuf>,
    /// 解析タブの設定値
    pub analysis_cfg: AnalysisSettings,
}

impl Default for App {
    fn default() -> Self {
        Self {
            model: squid_n_core::model::Model::default(),
            results: None,
            selection: Selection::default(),
            undo: UndoStack::new(),
            active_tab: Tab::Model,
            design_term: LoadTerm::Long,
            last_lc: None,
            last_error: None,
            node_edit: Vec::new(),
            node_draft: ["0".to_string(), "0".to_string(), "0".to_string()],
            staleness: Staleness::default(),
            nav: Navigator::default(),
            model_tab: ModelTab::default(),
            #[cfg(feature = "gui")]
            left_panel_width: 280.0,
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
            time_history_data: crate::time_history_view::TimeHistoryData::default(),
            #[cfg(feature = "gui")]
            time_history_source: crate::time_history_view::TimeHistorySource::default(),
            #[cfg(feature = "gui")]
            section_draft: crate::section_editor::SectionEditorDraft::default(),
            #[cfg(feature = "gui")]
            catalog_draft: crate::section_editor::CatalogDraft::default(),
            #[cfg(feature = "gui")]
            beam_draw_mode: false,
            #[cfg(feature = "gui")]
            beam_draw_first: None,
            #[cfg(feature = "gui")]
            wall_draw_mode: false,
            #[cfg(feature = "gui")]
            wall_draw_nodes: Vec::new(),
            project_path: None,
            analysis_cfg: AnalysisSettings::default(),
        }
    }
}

/// 日本語フォントを egui に登録する（UI のラベルが豆腐□にならないように）。
///
/// egui の既定フォント（Ubuntu/Hack）は日本語グリフを持たないため、
/// OS のシステムフォントから日本語対応フォントを探して読み込む。
/// 見つからない場合は何もしない（英数字は既定フォントで表示される）。
#[cfg(feature = "gui")]
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
    /// モデルを丸ごと差し替える（新規作成・サンプル読込・ファイル読込で共用）。
    /// undo 履歴・結果・選択・stale 状態をすべてリセットする。
    pub fn load_model(&mut self, model: squid_n_core::model::Model) {
        self.model = model;
        self.results = None;
        self.selection = Selection::default();
        self.undo = UndoStack::new();
        self.nav = Navigator::default();
        self.last_lc = None;
        self.last_error = None;
        self.beam_loads.clear();
        self.staleness = Staleness::default();
        self.sync_node_edit();
    }

    /// プロジェクトを指定パスへ保存する。成功時は project_path と未保存フラグを更新。
    pub fn save_project_to(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::scz::save_scz(&path, &self.model) {
            Ok(()) => {
                self.project_path = Some(path);
                self.staleness.unsaved_changes = false;
            }
            Err(e) => self.last_error = Some(format!("保存エラー: {}", e)),
        }
    }

    /// プロジェクトを指定パスから読み込む。成功時はモデルを差し替える。
    pub fn open_project_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::scz::load_scz(&path) {
            Ok(model) => {
                if let Err(e) = model.validate() {
                    self.last_error = Some(format!("読込モデルの検証エラー: {:?}", e));
                    return;
                }
                self.load_model(model);
                self.project_path = Some(path);
            }
            Err(e) => self.last_error = Some(format!("読込エラー: {}", e)),
        }
    }

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
                    let mut bundle = self.results.take().unwrap_or_default();
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
                    let mut bundle = self.results.take().unwrap_or_default();
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

    /// 階(Story)を節点標高から自動生成して適用する（undo 可能）。
    /// 地震重量には最初の荷重ケースの鉛直下向き荷重＋自重を用いる。
    pub fn generate_stories_action(&mut self) {
        self.last_error = None;
        let gravity_lc = self.model.load_cases.first().map(|c| c.id);
        match squid_n_load::story_gen::generate_stories(&self.model, gravity_lc) {
            Ok(gen) => {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::ApplyStories {
                        stories: gen.stories,
                        node_story: gen.node_story,
                        constraints: gen.constraints,
                    }),
                );
                self.staleness.mark_edited();
            }
            Err(e) => self.last_error = Some(format!("階の自動生成エラー: {}", e)),
        }
    }

    /// T3: 地震静的解析（Ai一気通貫）を実行し、結果を `self.results` に格納する。
    /// 方向・Ai算定法・Z・地盤種別・C0 は `analysis_cfg` を用いる。
    pub fn run_seismic(&mut self, dir: SeismicDir) {
        self.last_error = None;
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.seismic_static_with(cfg) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
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

    /// プッシュオーバー解析を実行する。モデルは複製の上で解析する
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    pub fn run_pushover(&mut self) {
        self.last_error = None;
        if let Err(e) = Analysis::prepare(&self.model) {
            self.last_error = Some(format!("解析準備エラー: {}", e));
            return;
        }
        let mut work = self.model.clone();
        let dofmap = squid_n_core::dof::DofMap::build(&work);
        let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
        let cfg = &self.analysis_cfg;
        match squid_n_solver::pushover::pushover_analysis(
            &mut work,
            &dofmap,
            &reducer,
            cfg.push_dir,
            cfg.push_steps,
            cfg.push_max_disp,
            false,
            false,
            0.0,
        ) {
            Ok(res) => {
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.pushover = Some(res);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
            }
            Err(e) => self.last_error = Some(format!("プッシュオーバー解析エラー: {}", e)),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰は 1 次固有周期に対する剛性比例。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.last_error = None;
        let analysis = match Analysis::prepare(&self.model) {
            Ok(a) => a,
            Err(e) => {
                self.last_error = Some(format!("解析準備エラー: {}", e));
                return;
            }
        };
        // 1 次固有円振動数（減衰の基準）
        let omega1 = match analysis.eigen(1) {
            Ok(modal) => match modal.omega2.first() {
                Some(&w2) if w2 > 0.0 => w2.sqrt(),
                _ => {
                    self.last_error = Some("固有値が得られず減衰を設定できません。".to_string());
                    return;
                }
            },
            Err(e) => {
                self.last_error = Some(format!("固有値解析エラー: {}", e));
                return;
            }
        };
        let damping = squid_n_solver::damping::Damping::StiffnessProportional {
            h: self.analysis_cfg.th_damping,
            omega: omega1,
            basis: squid_n_solver::damping::StiffnessKind::Initial,
        };
        let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
        match analysis.time_history(&wave, newmark, damping) {
            Ok(res) => {
                #[cfg(feature = "gui")]
                {
                    self.time_history_data = crate::time_history_view::TimeHistoryData {
                        time: res.time.clone(),
                        node_disp: res.history.node_disp_x.clone(),
                        story_shear: res.history.base_shear_x.clone(),
                        story_drift_angle: res.history.top_drift_angle.clone(),
                        node: res.history.node,
                    };
                }
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.time_history = Some(res);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
            }
            Err(e) => self.last_error = Some(format!("時刻歴解析エラー: {}", e)),
        }
    }

    /// 正弦減衰のサンプル地震波を生成して時刻歴解析を実行する
    /// （外部波形ファイルなしで機能を試せる導線）。
    pub fn run_time_history_sample(&mut self) {
        let cfg = &self.analysis_cfg;
        let n = ((cfg.th_duration / cfg.th_dt).ceil() as usize).max(2);
        let omega = 2.0 * std::f64::consts::PI / cfg.th_period.max(1e-6);
        let accel_x: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * cfg.th_dt;
                cfg.th_amp * (omega * t).sin() * (-0.3 * t).exp()
            })
            .collect();
        let wave = squid_n_solver::timehistory::GroundMotion {
            dt: cfg.th_dt,
            accel_x,
            accel_y: None,
        };
        self.run_time_history(wave);
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
        let mut checks: Vec<(ElemId, f64, squid_n_design_jp::CheckResult)> = Vec::new();
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
    // eframe のデフォルトは (12,12,12) ≒ 黒なので、テーマに合わせた白灰色で上書きする
    fn clear_color(&self, _visuals: &egui::Visuals) -> [f32; 4] {
        crate::theme::GRAY_100.to_normalized_gamma_f32()
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // 上部ツールバー: ファイルメニュー + 工程タブ（自由遷移）+ Undo/Redo
        ui.horizontal(|ui| {
            ui.menu_button("ファイル", |ui| {
                if ui.button("📄 新規").clicked() {
                    self.load_model(squid_n_core::model::Model::default());
                    self.project_path = None;
                    ui.close();
                }
                if ui.button("🏠 サンプル(門型ラーメン)").clicked() {
                    self.load_model(crate::sample::portal_frame());
                    self.project_path = None;
                    ui.close();
                }
                ui.separator();
                if ui.button("📂 開く…").clicked() {
                    self.open_project_dialog();
                    ui.close();
                }
                if ui.button("💾 保存").clicked() {
                    self.save_project_dialog(false);
                    ui.close();
                }
                if ui.button("💾 名前を付けて保存…").clicked() {
                    self.save_project_dialog(true);
                    ui.close();
                }
            });
            ui.separator();
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

        // 4ペイン：左ナビゲータ(+モデル/荷重設定) / 中央 / 右インスペクタ / 下ステータス
        // 下パネルは描画の都合上最後に置く。左右は egui::SidePanel を模して available_rect で分割。
        // 左ペインの幅はドラッグで調整可能（self.left_panel_width）。
        let available = ui.available_rect_before_wrap();
        let nav_width = self.left_panel_width.clamp(180.0, 520.0);
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

        // 左：ナビゲータ（常時）＋ モデル/荷重タブ選択中はその設定編集を併設。
        // モデル作成状況は中央の3Dビューで常時確認できるようにし、設定操作はここに集約する。
        #[allow(deprecated)]
        ui.allocate_ui_at_rect(nav_rect, |ui| {
            egui::ScrollArea::both()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    self.navigator_panel(ui);
                    if matches!(self.active_tab, Tab::Model | Tab::Loads) {
                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);
                        match self.active_tab {
                            Tab::Model => self.model_tab_panel(ui),
                            Tab::Loads => crate::tables::loads::loads_table(ui, self),
                            _ => unreachable!(),
                        }
                    }
                });
        });

        // 左ペインの右端：ドラッグで幅調整するハンドル
        let resize_rect = egui::Rect::from_min_max(
            egui::pos2(nav_rect.max.x - 3.0, nav_rect.min.y),
            egui::pos2(nav_rect.max.x + 3.0, nav_rect.max.y),
        );
        let resize_id = ui.id().with("left_panel_resize");
        let resize_response = ui.interact(resize_rect, resize_id, egui::Sense::drag());
        if resize_response.dragged() {
            self.left_panel_width =
                (self.left_panel_width + resize_response.drag_delta().x).clamp(180.0, 520.0);
        }
        if resize_response.hovered() || resize_response.dragged() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        }
        ui.painter().vline(
            nav_rect.max.x,
            nav_rect.y_range(),
            ui.visuals().widgets.noninteractive.bg_stroke,
        );

        // 中央：モデル/荷重タブでは常に3Dビュー（作成状況を即座に確認できるようにする）。
        // それ以外の工程タブは従来通りの内容を表示する。
        #[allow(deprecated)]
        ui.allocate_ui_at_rect(central_rect, |ui| match self.active_tab {
            Tab::Model | Tab::Loads => crate::viewer::viewer_panel(ui, self),
            Tab::Analysis => self.analysis_tab_panel(ui),
            Tab::Results => self.results_tab_panel(ui),
            Tab::Design => crate::design_view::design_table(ui, self),
            Tab::Report => self.report_tab_panel(ui),
        });

        // 右：インスペクタ
        #[allow(deprecated)]
        ui.allocate_ui_at_rect(inspector_rect, |ui| {
            self.inspector_panel(ui);
        });

        // 下：ステータスバー
        #[allow(deprecated)]
        ui.allocate_ui_at_rect(status_rect, |ui| {
            self.status_bar(ui);
        });
    }
}

#[cfg(feature = "gui")]
impl App {
    /// 「開く…」ダイアログを表示して読み込む。
    fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Squid-N プロジェクト", &["scz"])
            .pick_file()
        {
            self.open_project_from(path);
        }
    }

    /// 保存する。`force_ask` またはパス未設定時はダイアログで保存先を尋ねる。
    fn save_project_dialog(&mut self, force_ask: bool) {
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
                for s in &self.model.stories {
                    ui.label(format!(
                        "{}: 標高 {:.0} mm, 節点 {}, W = {:.1} kN",
                        s.name,
                        s.elevation,
                        s.node_ids.len(),
                        s.seismic_weight.unwrap_or(0.0) / 1000.0
                    ));
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
                    .add_enabled(selected_lc.is_some(), egui::Button::new("▶ 実行"))
                    .clicked()
                {
                    if let Some(lc) = selected_lc {
                        self.run_linear_static(lc);
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

        // ── 固有値 ────────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("固有値");
            ui.horizontal(|ui| {
                ui.label("モード数:");
                let mut n = self.analysis_cfg.n_modes;
                ui.add(egui::DragValue::new(&mut n).range(1..=30));
                self.analysis_cfg.n_modes = n;
                if ui.button("▶ 実行").clicked() {
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
            if ui.button("▶ 実行").clicked() {
                self.run_seismic(self.analysis_cfg.seismic_dir);
            }
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
            if ui.button("▶ 実行").clicked() {
                self.run_pushover();
                if self.last_error.is_none() {
                    self.active_tab = Tab::Results;
                    self.results_view = ResultsView::Pushover;
                }
            }
        });
        ui.add_space(6.0);

        // ── 時刻歴応答 ────────────────────────────────────────────
        ui.group(|ui| {
            ui.strong("時刻歴応答（線形 Newmark-β）");
            ui.horizontal(|ui| {
                ui.label("減衰比 h:");
                ui.add(
                    egui::DragValue::new(&mut self.analysis_cfg.th_damping)
                        .speed(0.005)
                        .range(0.0..=0.3),
                );
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
            ui.horizontal(|ui| {
                if ui
                    .button("▶ サンプル波で実行")
                    .on_hover_text("正弦減衰波を生成して時刻歴解析を実行します")
                    .clicked()
                {
                    self.run_time_history_sample();
                    if self.last_error.is_none() {
                        self.active_tab = Tab::Results;
                        self.results_view = ResultsView::TimeHistory;
                    }
                }
                if ui
                    .button("📂 波形CSVを開いて実行…")
                    .on_hover_text(
                        "1 行 1 値(加速度 gal)の CSV/テキスト。dt は上の設定値を使用します",
                    )
                    .clicked()
                {
                    self.run_time_history_from_csv();
                    if self.last_error.is_none() {
                        self.active_tab = Tab::Results;
                        self.results_view = ResultsView::TimeHistory;
                    }
                }
            });
        });
    }

    /// 波形 CSV（1 行 1 値、gal 単位）を選択して時刻歴解析を実行する。
    fn run_time_history_from_csv(&mut self) {
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
        // 1 行 1 値（カンマ区切りなら最後の列）を gal → mm/s² (×10) で読む
        let accel_x: Vec<f64> = content
            .lines()
            .filter_map(|l| {
                let field = l.split(',').next_back()?.trim();
                field.parse::<f64>().ok()
            })
            .map(|gal| gal * 10.0)
            .collect();
        if accel_x.len() < 2 {
            self.last_error = Some(
                "波形データが読み取れませんでした（数値が 2 点未満）。1 行 1 値の CSV を指定してください。"
                    .to_string(),
            );
            return;
        }
        let wave = squid_n_solver::timehistory::GroundMotion {
            dt: self.analysis_cfg.th_dt,
            accel_x,
            accel_y: None,
        };
        self.run_time_history(wave);
    }

    /// 結果タブ：3Dビューア と 時刻歴グラフを切替。
    fn results_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let sel_spatial = self.results_view == ResultsView::Spatial;
            let sel_th = self.results_view == ResultsView::TimeHistory;
            let sel_po = self.results_view == ResultsView::Pushover;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.results_view = ResultsView::TimeHistory;
            }
            if ui.selectable_label(sel_po, "プッシュオーバー").clicked() {
                self.results_view = ResultsView::Pushover;
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
        }
    }

    /// プッシュオーバー結果（性能曲線・ヒンジ・崩壊機構）の表示。
    fn pushover_panel(&mut self, ui: &mut egui::Ui) {
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

    /// レポートタブ：CSV レポートのプレビューとエクスポート。
    fn report_tab_panel(&mut self, ui: &mut egui::Ui) {
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
    }

    /// 下部ステータスバー。
    fn status_bar(&self, ui: &mut egui::Ui) {
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

    #[test]
    fn test_seismic_flow_requires_then_uses_stories() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());

        // 階なし → 明示エラー（サイレントゼロ結果ではない）
        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_some());

        // 階の自動生成 → 地震静的が成功する
        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        assert_eq!(app.model.stories.len(), 1);
        assert!(app.model.stories[0].seismic_weight.unwrap() > 0.0);

        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let r = app.results.as_ref().unwrap();
        let (_, res) = r.statics.first().unwrap();
        // 柱頭が X 方向へ変位している
        assert!(res.disp[2][0].abs() > 1e-3, "{}", res.disp[2][0]);

        // undo で階定義が戻る
        app.undo.undo(&mut app.model);
        assert!(app.model.stories.is_empty());
    }

    #[test]
    fn test_time_history_sample_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.analysis_cfg.th_duration = 2.0;
        app.run_time_history_sample();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
        assert!(th.history.node_disp_x.len() > 100);
        assert!(
            th.history.node_disp_x.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
        assert!(th.history.node.is_some());
    }

    #[test]
    fn test_pushover_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        app.analysis_cfg.push_steps = 10;
        app.run_pushover();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
        assert!(!po.capacity_curve.is_empty());
    }

    #[test]
    fn test_save_and_open_project_roundtrip() {
        let dir = std::env::temp_dir().join("squid_n_app_test_scz");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("roundtrip.scz");

        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.staleness.mark_edited();
        app.save_project_to(path.clone());
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        assert!(!app.staleness.unsaved_changes);
        assert_eq!(app.project_path.as_ref(), Some(&path));

        let saved_model = app.model.clone();
        let mut app2 = App::default();
        app2.open_project_from(path.clone());
        assert!(app2.last_error.is_none(), "{:?}", app2.last_error);
        assert!(app2.model.eq_ignoring_dofmap(&saved_model));
        assert_eq!(app2.project_path.as_ref(), Some(&path));

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_open_project_missing_file_sets_error() {
        let mut app = App::default();
        app.open_project_from(std::path::PathBuf::from(
            "/nonexistent/dir/does_not_exist.scz",
        ));
        assert!(app.last_error.is_some());
    }
}
