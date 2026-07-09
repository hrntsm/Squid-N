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
    Slabs,
}

/// 結果タブ内の切替（3D 各種図・時刻歴グラフ・プッシュオーバー曲線）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ResultsView {
    #[default]
    Spatial,
    TimeHistory,
    Pushover,
}

/// 設計タブ内の切替（検定表・MN相関曲面ビュー）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DesignView {
    #[default]
    Table,
    MnSurface,
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
    /// ナビゲータで選択中の結果表示対象（静的ケース／荷重組合せ）
    pub focus_result: Option<StaticKey>,
}

/// 静的解析結果の格納キー。ユーザー荷重ケースと地震静的(Ai)を型で区別し、
/// LoadCaseId(0) の二重使用(ユーザーケース0と地震結果の同居)を解消する。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaticCaseKey {
    /// ユーザー定義の荷重ケース
    User(LoadCaseId),
    /// 地震静的(Ai 分布)。方向別に共存できる
    Seismic(SeismicDir),
}

/// 表示対象の静的解析結果を指すキー。荷重ケース単体（ユーザー／地震静的）か
/// 荷重組合せかを区別する。
///
/// `Case` は `StaticCaseKey` をそのままネストする形を採る（`User`/`Seismic`/`Combo`
/// にフラット化する案もあったが、`current_static` やナビゲータでの
/// `bundle.statics` 引き当ては User/Seismic を区別する必要がなく
/// `StaticCaseKey` の等値比較 1 本で完結するため、ネストの方が呼び出し側の
/// 分岐が増えずシンプルに書ける）。
///
/// `Combo` のインデックスは **`ResultsBundle.combos` 上の位置**
/// （`model.combinations` のインデックスではない）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaticKey {
    Case(StaticCaseKey),
    Combo(usize),
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
    pub statics: Vec<(StaticCaseKey, squid_n_solver::linear::StaticOnce)>,
    /// 荷重組合せの解析結果（組合せ名で保持）
    pub combos: Vec<(String, squid_n_solver::linear::StaticOnce)>,
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
    /// 時刻歴の入力方向(サンプル波・CSV波形の作用方向)
    pub th_dir: ThDir,
    /// 時刻歴の減衰モデル
    pub th_damping_model: ThDampingModel,
    /// Rayleigh の2次モード減衰比(1次は th_damping を使用)
    pub th_h2: f64,
    /// 時刻歴の積分法
    pub th_integrator: ThIntegrator,
}

/// 時刻歴の入力方向選択（UI 用）。X・Y に加え、同一波形を両方向へ同時入力する
/// 「X+Y」を持つ（`SeismicDir` は静的地震荷重・プッシュオーバー共用のため
/// 拡張せず、時刻歴専用にこの型を新設する）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThDir {
    X,
    Y,
    Xy,
}

/// 時刻歴の減衰モデル選択（UI 用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThDampingModel {
    StiffnessProportional,
    Rayleigh,
}

/// 時刻歴の積分法選択（UI 用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThIntegrator {
    NewmarkBeta,
    HhtAlpha,
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
            th_dir: ThDir::X,
            th_damping_model: ThDampingModel::StiffnessProportional,
            th_h2: 0.02,
            th_integrator: ThIntegrator::NewmarkBeta,
        }
    }
}

/// バックグラウンド解析ジョブ（プッシュオーバー／時刻歴）が送る結果。
pub enum JobResult {
    Pushover(Result<squid_n_solver::pushover::PushoverResult, String>),
    TimeHistory(Result<squid_n_solver::timehistory::ResponseResult, String>),
}

/// バックグラウンド解析ジョブ。重い解析(プッシュオーバー・時刻歴)を
/// UI スレッドから逃がす(P8 §5)。結果は poll_job で受け取り適用する。
pub struct AnalysisJob {
    pub label: &'static str,
    pub started: std::time::SystemTime,
    rx: std::sync::mpsc::Receiver<JobResult>,
    /// ジョブ成功時に自動遷移する結果タブ・表示切替（GUI 専用）。
    #[cfg(feature = "gui")]
    pub jump_on_success: Option<(Tab, ResultsView)>,
}

pub struct App {
    pub model: squid_n_core::model::Model,
    pub results: Option<ResultsBundle>,
    pub selection: Selection,
    pub undo: UndoStack,
    pub active_tab: Tab,
    /// 設計検定の荷重継続性区分（長期／短期）
    pub design_term: LoadTerm,
    /// 最後に実行した静的解析結果（荷重ケース／荷重組合せ）
    pub last_static: Option<StaticKey>,
    /// 解析実行中のエラーメッセージ
    pub last_error: Option<String>,
    /// 実行中のバックグラウンド解析ジョブ（プッシュオーバー・時刻歴、P8 §5）。
    /// 完了は `poll_job` で検知して結果を適用する。
    pub job: Option<AnalysisJob>,
    /// 節点座標の編集バッファ（model.nodes に同期）
    pub node_edit: Vec<[String; 3]>,
    /// 節点追加フォームの入力中座標（境界条件の編集とは別の独立 UI）
    pub node_draft: [String; 3],
    /// 節点追加時に既存節点と同一座標だった場合の追加保留座標。
    /// セットされている間は確認ダイアログを表示し、ユーザの判断を待つ。
    pub pending_duplicate_node_coord: Option<[f64; 3]>,
    /// stale（要再計算）状態と最終実行時刻
    pub staleness: Staleness,
    /// ナビゲータ（左ペイン）状態
    pub nav: Navigator,
    /// モデルタブ内のサブタブ
    pub model_tab: ModelTab,
    /// 保有水平耐力（ルート3）判定の架構種別（Ds 表の行選択）
    pub design_frame: squid_n_design_jp::holding_capacity::FrameType,
    /// 保有水平耐力（ルート3）判定の部材ランク（Ds 表の列選択）。
    /// `design_rank_auto == true` の場合はフォールバック用（幅厚比を算定できない
    /// 層のみに適用される）。
    pub design_rank: squid_n_design_jp::holding_capacity::MemberRank,
    /// 保有水平耐力（ルート3）の部材ランクを鋼部材の幅厚比から自動判定するか（UI-13）。
    /// true の場合、鋼部材かつ断面形状(`Section.shape`)を持つ部材について
    /// `squid_n_design_jp::ds::max_width_thickness` → `s_member_rank` で算定し、
    /// 算定できなかった層のみ `design_rank`（選択値）にフォールバックする。
    pub design_rank_auto: bool,
    /// 左ペインの幅（px）。ドラッグで調整可能（180–520 にクランプ）。
    #[cfg(feature = "gui")]
    pub left_panel_width: f32,
    /// 結果タブ内の表示切替（3D / 時刻歴）
    #[cfg(feature = "gui")]
    pub results_view: ResultsView,
    /// 設計タブ内の表示切替（検定表 / MN相関曲面）
    #[cfg(feature = "gui")]
    pub design_view: DesignView,
    /// MN 相関曲面ビューの状態（断面選択・材料強度・表示切替・カメラ等）
    #[cfg(feature = "gui")]
    pub mn_view: crate::mn_view::MnViewState,
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
    /// 解析タブ「荷重組合せ」で選択中の組合せインデックス（model.combinations）
    #[cfg(feature = "gui")]
    pub analysis_combo_idx: usize,
    /// 荷重タブ「荷重組合せ」自動生成 UI のドラフト状態
    #[cfg(feature = "gui")]
    pub combo_draft: ComboDraft,
    /// モデルタブ「スラブ」追加フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub slab_draft: crate::tables::slabs::SlabDraft,
    /// 解析タブ「階の定義」W[kN] 編集バッファ（kN、model.stories と同じ並び）。
    /// ドラッグ／フォーカス中でない行のみ model 値で上書きする。
    #[cfg(feature = "gui")]
    pub story_weight_edit: Vec<f64>,
    /// `story_weight_edit` の各行が現在操作中（ドラッグ中またはフォーカス中）か。
    /// true の間は model 値での上書きを止めて入力中の値を保つ。
    #[cfg(feature = "gui")]
    pub story_weight_active: Vec<bool>,
}

/// 荷重組合せ自動生成 UI のドラフト（GUI 専用）。DL/LL は必須、地震X/Y・積雪は任意。
#[cfg(feature = "gui")]
#[derive(Clone, Debug, Default)]
pub struct ComboDraft {
    pub dl: Option<LoadCaseId>,
    pub ll: Option<LoadCaseId>,
    pub seismic_x: Option<LoadCaseId>,
    pub seismic_y: Option<LoadCaseId>,
    pub snow: Option<LoadCaseId>,
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
            last_static: None,
            last_error: None,
            job: None,
            node_edit: Vec::new(),
            node_draft: ["0".to_string(), "0".to_string(), "0".to_string()],
            pending_duplicate_node_coord: None,
            staleness: Staleness::default(),
            nav: Navigator::default(),
            model_tab: ModelTab::default(),
            // サンプル(門型ラーメン)が鋼構造のため既定は S ラーメン
            design_frame: squid_n_design_jp::holding_capacity::FrameType::SteelFrame,
            design_rank: squid_n_design_jp::holding_capacity::MemberRank::FA,
            design_rank_auto: false,
            #[cfg(feature = "gui")]
            left_panel_width: 280.0,
            #[cfg(feature = "gui")]
            results_view: ResultsView::default(),
            #[cfg(feature = "gui")]
            design_view: DesignView::default(),
            #[cfg(feature = "gui")]
            mn_view: crate::mn_view::MnViewState::default(),
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
            #[cfg(feature = "gui")]
            analysis_combo_idx: 0,
            #[cfg(feature = "gui")]
            combo_draft: ComboDraft::default(),
            #[cfg(feature = "gui")]
            slab_draft: crate::tables::slabs::SlabDraft::default(),
            #[cfg(feature = "gui")]
            story_weight_edit: Vec::new(),
            #[cfg(feature = "gui")]
            story_weight_active: Vec::new(),
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
        self.last_static = None;
        self.last_error = None;
        self.staleness = Staleness::default();
        self.sync_node_edit();
        self.refresh_beam_loads();
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

    /// ST-Bridge（XML, サブセット）ファイルを読み込む。
    /// Squid-N プロジェクト（.scz）とは別物なので project_path はクリアする。
    pub fn import_stbridge_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        let xml = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                self.last_error = Some(format!("ST-Bridge読込エラー: {}", e));
                return;
            }
        };
        match squid_n_io::stbridge::import_stbridge(&xml) {
            Ok(model) => {
                if let Err(e) = model.validate() {
                    self.last_error = Some(format!("ST-Bridge読込モデルの検証エラー: {:?}", e));
                    return;
                }
                self.load_model(model);
                self.project_path = None;
            }
            Err(e) => self.last_error = Some(format!("ST-Bridge読込エラー: {}", e)),
        }
    }

    /// モデルを ST-Bridge（XML, サブセット）として指定パスへ書き出す。
    pub fn export_stbridge_to(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::stbridge::export_stbridge(&self.model) {
            Ok(xml) => {
                if let Err(e) = std::fs::write(&path, xml) {
                    self.last_error = Some(format!("ST-Bridge書出エラー: {}", e));
                }
            }
            Err(e) => self.last_error = Some(format!("ST-Bridge書出エラー: {}", e)),
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
                    let key = StaticCaseKey::User(lc);
                    bundle.statics.retain(|(id, _)| *id != key);
                    bundle.statics.push((key, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Case(key));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("線形静的解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// T7: 荷重組合せ解析を実行し、結果を `bundle.combos` に格納する。
    /// 指定インデックスの荷重組合せが存在しない場合はエラーメッセージをセット。
    pub fn run_combination(&mut self, index: usize) {
        self.last_error = None;
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.last_error = Some(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.linear_combination(&combo) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
                    // StaticKey::Combo は bundle.combos 上の位置を指す規約
                    // （current_static・ナビゲータと共有）。再実行時は既存位置を
                    // その場で差し替え、他の組合せ結果のキーを無効化しない。
                    let pos = match bundle
                        .combos
                        .iter()
                        .position(|(name, _)| *name == combo.name)
                    {
                        Some(pos) => {
                            bundle.combos[pos].1 = res;
                            pos
                        }
                        None => {
                            bundle.combos.push((combo.name.clone(), res));
                            bundle.combos.len() - 1
                        }
                    };
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Combo(pos));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("荷重組合せ解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// 表示対象の静的解析結果を解決する。優先順: ナビゲータ選択 → 最後に実行した結果。
    pub fn current_static(&self) -> Option<&squid_n_solver::linear::StaticOnce> {
        let bundle = self.results.as_ref()?;
        let resolve = |key: StaticKey| -> Option<&squid_n_solver::linear::StaticOnce> {
            match key {
                StaticKey::Case(case_key) => bundle
                    .statics
                    .iter()
                    .find(|(k, _)| *k == case_key)
                    .map(|(_, s)| s),
                StaticKey::Combo(idx) => bundle.combos.get(idx).map(|(_, s)| s),
            }
        };
        self.nav
            .focus_result
            .and_then(resolve)
            .or_else(|| self.last_static.and_then(resolve))
    }

    /// 保有水平耐力の層別判定を行う。前提データが不足していれば Err(案内文)。
    ///
    /// 戻り値の第 2 要素は層ごとに採用された部材ランク（`design_rank_auto` が
    /// true の場合は幅厚比からの自動判定、算定できなかった層は `design_rank`
    /// へフォールバック。false の場合は全層 `design_rank`）。
    #[allow(clippy::type_complexity)]
    pub fn compute_holding_capacity(
        &self,
    ) -> Result<
        (
            squid_n_design_jp::holding_capacity::HoldingCapacityResult,
            Vec<squid_n_design_jp::holding_capacity::MemberRank>,
        ),
        String,
    > {
        use squid_n_core::section_shape::SectionShape;
        use squid_n_design_jp::ds::{
            max_width_thickness, rc_member_rank, s_member_rank_scaled, worst_rank, RankCriteria,
        };
        use squid_n_design_jp::holding_capacity::{
            check_holding_capacity, ds_value, qud_by_story, MemberRank,
        };
        use squid_n_design_jp::rc_capacity::{rc_qmu_simple, rc_qsu_simple};
        use squid_n_design_jp::steel_f_value_prefix;

        if self.model.stories.is_empty() {
            return Err(
                "階が未定義です。解析タブの「階の自動生成」を実行してください。".to_string(),
            );
        }
        let po = self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .ok_or_else(|| {
                "プッシュオーバー未実行です。解析タブからプッシュオーバーを実行してください。"
                    .to_string()
            })?;
        let st = self.current_static().ok_or_else(|| {
            "静的解析結果がありません。地震静的(Ai)を実行してください。".to_string()
        })?;

        let metrics = crate::summary::compute_story_metrics(
            &self.model,
            &st.disp,
            self.analysis_cfg.seismic_dir,
        );

        // 地震重量: 下階→上階順（model.stories は生成時から下階→上階順に格納される）。
        let weights: Vec<f64> = self
            .model
            .stories
            .iter()
            .map(|s| s.seismic_weight.unwrap_or(0.0))
            .collect();
        if weights.iter().any(|w| *w <= 0.0) {
            return Err(
                "地震重量が未設定です。解析タブの「階の自動生成」を実行してください。".to_string(),
            );
        }

        // T(1 次周期): 固有値解析があればそれを使用、なければ略算式。
        let t = self
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.period.first().copied())
            .unwrap_or_else(|| {
                let height_m = self
                    .model
                    .stories
                    .last()
                    .map(|s| s.elevation)
                    .unwrap_or(0.0)
                    / 1000.0;
                squid_n_load::ai::approx_t(height_m, 0.0)
            });
        let rt = squid_n_load::ai::rt(t, squid_n_load::ai::tc_of(self.analysis_cfg.soil));
        let qud = qud_by_story(&weights, self.analysis_cfg.z, rt, t);

        let n_stories = weights.len();
        let (story_ranks, member_ranks): (Vec<MemberRank>, Vec<(ElemId, MemberRank)>) =
            if self.design_rank_auto {
                // 鋼部材は幅厚比、RC 矩形部材はせん断余裕度 Qsu/Qmu の略算から
                // ランクを算定し、所属階ごとに集計する。
                //
                // 所属階の規則: 部材の節点のうち最も高い階(story index 最大)。
                // story_gen::generate_stories は各節点をその節点自身の標高が属する
                // レベルへ割り当てる（柱下端は下階または基部=None、柱上端は上階、
                // 梁は両端とも同一階）ため、柱は自動的に上端側の階（＝各節点の
                // story のうち最大値）に算入される。
                let mut per_story: Vec<Vec<MemberRank>> = vec![Vec::new(); n_stories];
                let mut computed: Vec<(ElemId, MemberRank)> = Vec::new();
                // 長期軸力の簡易近似として使う先頭荷重ケースの id
                // （`generate_stories_action` の gravity_lc と同じ規則）。
                let gravity_lc = self.model.load_cases.first().map(|c| c.id);
                for elem in &self.model.elements {
                    let Some(sec) = elem
                        .section
                        .and_then(|sid| self.model.sections.get(sid.index()))
                    else {
                        continue;
                    };
                    let Some(mat) = elem
                        .material
                        .and_then(|mid| self.model.materials.get(mid.index()))
                    else {
                        continue;
                    };
                    let rank = if is_steel(&mat.name) {
                        // 鋼部材: 形状情報がない断面(カタログ数値直入力等)・
                        // 円形鋼管等の幅厚比対象外形状はスキップ。
                        let Some(shape) = sec.shape.as_ref() else {
                            continue;
                        };
                        let Some(wt) = max_width_thickness(shape) else {
                            continue;
                        };
                        // F 値は材料名の前方一致で引く(例 "SN400B"→235)。引けなければ 235。
                        let f_value = steel_f_value_prefix(&mat.name).unwrap_or(235.0);
                        s_member_rank_scaled(wt, f_value, &RankCriteria::default())
                    } else {
                        // RC 部材: RcRect のみ対応。RcCircle・形状未設定・
                        // コンクリート強度(fc)未設定の材料はスキップ(選択値へフォールバック)。
                        let Some(SectionShape::RcRect { b, d, rebar }) = sec.shape.as_ref() else {
                            continue;
                        };
                        // 内法スパン = 幾何長 - 両端剛域長。剛域長の合計が幾何長以上になる
                        // (不整合な入力)場合は下限0を割り込むため、幾何長のままとする。
                        let geom_len = elem_geometric_length(elem, &self.model);
                        let rz_len = elem.rigid_zone.length_i + elem.rigid_zone.length_j;
                        let clear_span = if geom_len - rz_len > 0.0 {
                            geom_len - rz_len
                        } else {
                            geom_len
                        };
                        let Some(mut input) =
                            rc_capacity_input_from_rect(*b, *d, rebar, mat, clear_span)
                        else {
                            continue;
                        };
                        // σ0: 長期軸力の簡易近似として先頭荷重ケース(gravity_lc)の
                        // 静的解析結果を優先し、無ければ最後に実行した静的解析結果
                        // (self.results.member_forces)から当該部材の軸力を引き、
                        // 圧縮のときのみ設定する。
                        let sigma_0 = self
                            .results
                            .as_ref()
                            .map(|r| {
                                rc_sigma_0_from_gravity_or_last_static(
                                    &r.statics,
                                    &r.member_forces,
                                    gravity_lc,
                                    elem.id,
                                    *b,
                                    *d,
                                )
                            })
                            .unwrap_or(0.0);
                        input.sigma_0 = sigma_0;
                        let qmu = rc_qmu_simple(&input);
                        let qsu = rc_qsu_simple(&input);
                        rc_member_rank(qsu, qmu, &RankCriteria::default())
                    };
                    // 節点が階を持たない部材（両端とも基部）はスキップ。
                    let Some(story_idx) = elem
                        .nodes
                        .iter()
                        .filter_map(|nid| self.model.nodes.get(nid.index()))
                        .filter_map(|n| n.story)
                        .max()
                    else {
                        continue;
                    };
                    let idx = story_idx.index();
                    if idx >= n_stories {
                        continue;
                    }
                    per_story[idx].push(rank);
                    computed.push((elem.id, rank));
                }
                // 階ごとの代表ランク = 算定できた部材ランクの最悪値。
                // 1 本も算定できなかった層は手動選択ランクへフォールバック。
                let ranks: Vec<MemberRank> = per_story
                    .into_iter()
                    .map(|rs| worst_rank(&rs).unwrap_or(self.design_rank))
                    .collect();
                (ranks, computed)
            } else {
                (vec![self.design_rank; n_stories], Vec::new())
            };

        let ds_vec: Vec<f64> = story_ranks
            .iter()
            .map(|r| ds_value(self.design_frame, *r))
            .collect();
        let heights: Vec<f64> = metrics.iter().map(|m| m.height).collect();
        let rs: Vec<f64> = metrics.iter().map(|m| m.rs).collect();
        let re: Vec<f64> = metrics.iter().map(|m| m.re).collect();
        let fes: Vec<f64> = metrics.iter().map(|m| m.fes).collect();

        let result =
            check_holding_capacity(po, &qud, &ds_vec, &fes, &rs, &re, &heights, member_ranks);
        Ok((result, story_ranks))
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
    /// 結果は `StaticCaseKey::Seismic(dir)` に格納するため、X/Y 双方の地震静的結果
    /// および任意のユーザー荷重ケースの結果と衝突せず共存できる。
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
                    let key = StaticCaseKey::Seismic(dir);
                    bundle.statics.retain(|(id, _)| *id != key);
                    bundle.statics.push((key, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Case(key));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("地震解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// プッシュオーバー解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_pushover_job`）からも呼び出せる。
    /// モデルは呼び出し側で複製したものを渡す
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    fn compute_pushover(
        model: squid_n_core::model::Model,
        cfg: AnalysisSettings,
    ) -> Result<squid_n_solver::pushover::PushoverResult, String> {
        Analysis::prepare(&model).map_err(|e| format!("解析準備エラー: {}", e))?;
        let mut work = model;
        let dofmap = squid_n_core::dof::DofMap::build(&work);
        let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
        squid_n_solver::pushover::pushover_analysis(
            &mut work,
            &dofmap,
            &reducer,
            cfg.push_dir,
            cfg.push_steps,
            cfg.push_max_disp,
            false,
            false,
            0.0,
        )
        .map_err(|e| format!("プッシュオーバー解析エラー: {}", e))
    }

    /// `compute_pushover` の結果を適用する（bundle 格納・最終実行時刻更新・エラー設定）。
    fn apply_pushover_result(
        &mut self,
        res: Result<squid_n_solver::pushover::PushoverResult, String>,
    ) {
        match res {
            Ok(result) => {
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.pushover = Some(result);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// プッシュオーバー解析を実行する。モデルは複製の上で解析する
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    pub fn run_pushover(&mut self) {
        self.last_error = None;
        let res = Self::compute_pushover(self.model.clone(), self.analysis_cfg);
        self.apply_pushover_result(res);
    }

    /// プッシュオーバー解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_pushover_job(&mut self) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.last_error = None;
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_pushover(model, cfg)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::Pushover(result));
        });
        self.job = Some(AnalysisJob {
            label: "プッシュオーバー",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: Some((Tab::Results, ResultsView::Pushover)),
        });
    }

    /// 線形時刻歴応答解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_time_history_job`）からも呼び出せる。
    /// 減衰モデル・積分法は `cfg` に従う（剛性比例／Rayleigh、Newmark-β／HHT-α）。
    fn compute_time_history(
        model: squid_n_core::model::Model,
        cfg: AnalysisSettings,
        wave: squid_n_solver::timehistory::GroundMotion,
    ) -> Result<squid_n_solver::timehistory::ResponseResult, String> {
        let analysis = Analysis::prepare(&model).map_err(|e| format!("解析準備エラー: {}", e))?;
        let damping = match cfg.th_damping_model {
            ThDampingModel::StiffnessProportional => {
                // 1 次固有円振動数（減衰の基準）
                let omega1 = match analysis.eigen(1) {
                    Ok(modal) => match modal.omega2.first() {
                        Some(&w2) if w2 > 0.0 => w2.sqrt(),
                        _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
                    },
                    Err(e) => return Err(format!("固有値解析エラー: {}", e)),
                };
                squid_n_solver::damping::Damping::StiffnessProportional {
                    h: cfg.th_damping,
                    omega: omega1,
                    basis: squid_n_solver::damping::StiffnessKind::Initial,
                }
            }
            ThDampingModel::Rayleigh => {
                // 1次・2次の固有円振動数（Rayleigh 減衰の基準）
                let modal = match analysis.eigen(2) {
                    Ok(m) => m,
                    Err(e) => return Err(format!("固有値解析エラー: {}", e)),
                };
                let (w1, w2) = match (modal.omega2.first(), modal.omega2.get(1)) {
                    (Some(&a), Some(&b)) if a > 0.0 && b > 0.0 => (a.sqrt(), b.sqrt()),
                    _ => {
                        return Err(
                            "Rayleigh 減衰には 2 次までの固有値が必要です（モード数を確保できませんでした）。"
                                .to_string(),
                        );
                    }
                };
                squid_n_solver::damping::Damping::Rayleigh {
                    h1: cfg.th_damping,
                    w1,
                    h2: cfg.th_h2,
                    w2,
                }
            }
        };
        let result = match cfg.th_integrator {
            ThIntegrator::NewmarkBeta => {
                let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
                analysis.time_history(&wave, newmark, damping)
            }
            ThIntegrator::HhtAlpha => {
                let hht = squid_n_solver::timehistory::HhtCfg::new(wave.dt);
                analysis.time_history_hht(&wave, hht, damping)
            }
        };
        result.map_err(|e| format!("時刻歴解析エラー: {}", e))
    }

    /// `compute_time_history` の結果を適用する
    /// （bundle 格納・time_history_data 更新(gui)・最終実行時刻更新・エラー設定）。
    fn apply_time_history_result(
        &mut self,
        res: Result<squid_n_solver::timehistory::ResponseResult, String>,
    ) {
        match res {
            Ok(res) => {
                #[cfg(feature = "gui")]
                {
                    self.time_history_data = crate::time_history_view::TimeHistoryData {
                        time: res.time.clone(),
                        node_disp: res.history.node_disp.clone(),
                        story_shear: res.history.base_shear.clone(),
                        story_drift_angle: res.history.top_drift_angle.clone(),
                        node: res.history.node,
                    };
                }
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.time_history = Some(res);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰モデル・積分法は `analysis_cfg` に従う
    /// （剛性比例／Rayleigh、Newmark-β／HHT-α）。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.last_error = None;
        let res = Self::compute_time_history(self.model.clone(), self.analysis_cfg, wave);
        self.apply_time_history_result(res);
    }

    /// 時刻歴応答解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_time_history_job(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.last_error = None;
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_time_history(model, cfg, wave)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::TimeHistory(result));
        });
        self.job = Some(AnalysisJob {
            label: "時刻歴応答",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: Some((Tab::Results, ResultsView::TimeHistory)),
        });
    }

    /// 実行中のジョブの完了を確認し、完了していれば結果を適用する。
    /// 成功/失敗いずれかで結果を受信できた場合、またはスレッド異常終了時は
    /// `job` を `None` に戻し `true` を返す。まだ実行中なら `false` を返す。
    pub fn poll_job(&mut self) -> bool {
        let recv = match &self.job {
            Some(job) => job.rx.try_recv(),
            None => return false,
        };
        match recv {
            Ok(result) => {
                #[cfg(feature = "gui")]
                let jump = self.job.take().and_then(|j| j.jump_on_success);
                #[cfg(not(feature = "gui"))]
                {
                    self.job = None;
                }
                match result {
                    JobResult::Pushover(res) => self.apply_pushover_result(res),
                    JobResult::TimeHistory(res) => self.apply_time_history_result(res),
                }
                #[cfg(feature = "gui")]
                {
                    if self.last_error.is_none() {
                        if let Some((tab, view)) = jump {
                            self.active_tab = tab;
                            self.results_view = view;
                        }
                    }
                }
                true
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.job = None;
                self.last_error = Some(
                    "解析スレッドが異常終了しました（結果を受信できませんでした）。".to_string(),
                );
                true
            }
        }
    }

    /// 正弦減衰のサンプル地震波を `cfg` から組み立てる
    /// （外部波形ファイルなしで機能を試せる導線。同期実行・ジョブ実行の双方で使う）。
    fn sample_wave(cfg: &AnalysisSettings) -> squid_n_solver::timehistory::GroundMotion {
        let n = ((cfg.th_duration / cfg.th_dt).ceil() as usize).max(2);
        let omega = 2.0 * std::f64::consts::PI / cfg.th_period.max(1e-6);
        let accel: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * cfg.th_dt;
                cfg.th_amp * (omega * t).sin() * (-0.3 * t).exp()
            })
            .collect();
        Self::build_ground_motion(cfg.th_dt, cfg.th_dir, accel)
    }

    /// 正弦減衰のサンプル地震波を生成して時刻歴解析を実行する（同期）。
    pub fn run_time_history_sample(&mut self) {
        let wave = Self::sample_wave(&self.analysis_cfg);
        self.run_time_history(wave);
    }

    /// 方向 `dir` に加速度列 `accel` を割り当てた `GroundMotion` を組み立てる。
    /// X なら accel_x、Y なら accel_y に入れ、他方はゼロ列にする。
    /// Xy（X+Y 同時入力）は同一波形を accel_x・accel_y の両方にそのまま入れる
    /// 簡易仕様（位相差・別波形の指定はサポートしない。CSV 2 列入力は
    /// `parse_wave_csv` が別々の列を返すため、その場合は本関数を経由せず
    /// 直接 `GroundMotion` を組み立てる）。
    fn build_ground_motion(
        dt: f64,
        dir: ThDir,
        accel: Vec<f64>,
    ) -> squid_n_solver::timehistory::GroundMotion {
        match dir {
            ThDir::X => squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: accel,
                accel_y: None,
            },
            ThDir::Y => {
                let n = accel.len();
                squid_n_solver::timehistory::GroundMotion {
                    dt,
                    accel_x: vec![0.0; n],
                    accel_y: Some(accel),
                }
            }
            ThDir::Xy => squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: accel.clone(),
                accel_y: Some(accel),
            },
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

    /// 全スラブの床荷重を大梁へ分配し、辺インデックスを実部材IDへ対応付けて
    /// `self.beam_loads` を更新する。対応する梁が無い辺の荷重は捨てる。
    ///
    /// `squid_n_load::floor::distribute_slab` が返す `BeamLoad.elem` は実部材ID
    /// ではなく、スラブ境界の辺インデックス（`ElemId(i)`, i=0..4、辺 i は
    /// `boundary[i]` → `boundary[(i+1)%4]`）である。ここでその節点対を両端に
    /// 持つ `Beam` 要素を探し、実 ElemId に置き換える（ノード順は不問）。
    pub fn refresh_beam_loads(&mut self) {
        let mut beam_loads = Vec::new();
        for slab in &self.model.slabs {
            if slab.boundary.len() < 4 {
                continue;
            }
            for mut bl in squid_n_load::floor::distribute_slab(&self.model, slab) {
                let k = bl.elem.0 as usize;
                if k >= 4 {
                    continue;
                }
                let n0 = slab.boundary[k];
                let n1 = slab.boundary[(k + 1) % 4];
                let found = self.model.elements.iter().find(|e| {
                    e.kind == squid_n_core::model::ElementKind::Beam
                        && e.nodes.len() == 2
                        && ((e.nodes[0] == n0 && e.nodes[1] == n1)
                            || (e.nodes[0] == n1 && e.nodes[1] == n0))
                });
                let Some(elem) = found else { continue };
                bl.elem = elem.id;
                beam_loads.push(bl);
            }
        }
        self.beam_loads = beam_loads;
    }
}

/// 波形 CSV/テキストの内容を解析する（ヘッドレステスト可能な純粋関数）。
///
/// - `ThDir::X` / `ThDir::Y`: 1 行 1 値（カンマ区切りなら最後の列）を加速度(gal)として
///   読む（従来仕様）。数値化できない行は無視する。戻り値の第 2 要素は常に `None`。
/// - `ThDir::Xy`: 1 行をカンマ区切り 2 列（1 列目 X、2 列目 Y、ともに gal）として読む。
///   2 列に満たない行があればエラーを返す（「X+Y には2列のCSVが必要です」）。
///   数値化できない行（ヘッダ等）は無視する。
///
/// いずれも gal → mm/s²（内部単位系）へ ×10 で変換して返す。
/// 有効なデータ点が 2 点未満の場合はエラーを返す。
///
/// 呼び出し元（`run_time_history_from_csv`）は GUI 専用のため、非 GUI ビルドでは
/// テストからのみ使用される（`cfg(any(test, feature = "gui"))` で dead_code 警告を回避）。
#[cfg(any(test, feature = "gui"))]
fn parse_wave_csv(content: &str, dir: ThDir) -> Result<(Vec<f64>, Option<Vec<f64>>), String> {
    match dir {
        ThDir::X | ThDir::Y => {
            let accel: Vec<f64> = content
                .lines()
                .filter_map(|l| {
                    let field = l.split(',').next_back()?.trim();
                    field.parse::<f64>().ok()
                })
                .map(|gal| gal * 10.0)
                .collect();
            if accel.len() < 2 {
                return Err(
                    "波形データが読み取れませんでした（数値が 2 点未満）。1 行 1 値の CSV を指定してください。"
                        .to_string(),
                );
            }
            Ok((accel, None))
        }
        ThDir::Xy => {
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            for line in content.lines() {
                let line = line.trim();
                if line.is_empty() {
                    continue;
                }
                let fields: Vec<&str> = line.split(',').collect();
                if fields.len() < 2 {
                    return Err("X+Y には2列のCSVが必要です".to_string());
                }
                let (Ok(x), Ok(y)) = (
                    fields[0].trim().parse::<f64>(),
                    fields[1].trim().parse::<f64>(),
                ) else {
                    // ヘッダ行等、数値化できない行は無視する。
                    continue;
                };
                xs.push(x * 10.0);
                ys.push(y * 10.0);
            }
            if xs.len() < 2 {
                return Err(
                    "波形データが読み取れませんでした（数値が 2 点未満）。X+Y には2列のCSVが必要です。"
                        .to_string(),
                );
            }
            Ok((xs, Some(ys)))
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

/// `SectionShape::RcRect` の配筋情報から RC 終局耐力算定（rank-auto）用の入力を組み立てる。
///
/// # 変換規則
/// - 曲げ・せん断は強軸（せい=d）まわりを想定する。引張側主筋量 `at` は上下対称配筋を
///   仮定し、`main_x`（せい方向主筋）の総断面積の半分とする（非対称配筋の場合は別途検討）。
/// - `d_eff` = d - かぶり - 主筋径/2。
/// - `pw` = せん断補強筋 1 組の断面積(π/4・dia²)×組数 / (b・ピッチ)。ピッチが 0 以下なら 0。
/// - `sigma_y`: 材料の `fy` があればそれを使用し、なければ 345 N/mm²（SD345 相当、要・原典照合）。
/// - `sigma_wy`: 295 N/mm² 固定（SD295 相当、要・原典照合。せん断補強筋の材質はモデル上
///   部材材料と区別されないため代表値を用いる）。
/// - `fc`: 材料の `fc`（コンクリート設計基準強度）が未設定の場合は `None` を返し、
///   ランク算定の対象外（呼び出し側で選択値へフォールバック）とする。
fn rc_capacity_input_from_rect(
    b: f64,
    d: f64,
    rebar: &squid_n_core::section_shape::RcRebar,
    mat: &squid_n_core::model::Material,
    clear_span: f64,
) -> Option<squid_n_design_jp::rc_capacity::RcCapacityInput> {
    let fc = mat.fc?;
    let bar_area = |bs: &squid_n_core::section_shape::BarSet| -> f64 {
        bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia
    };
    // 上下対称配筋を仮定し、引張側主筋量は main_x 総断面積の半分。
    let at = bar_area(&rebar.main_x) / 2.0;
    let d_eff = d - rebar.cover - rebar.main_x.dia / 2.0;
    let shear_area =
        std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia * rebar.shear.legs as f64;
    let pw = if rebar.shear.pitch > 0.0 {
        shear_area / (b * rebar.shear.pitch)
    } else {
        0.0
    };
    Some(squid_n_design_jp::rc_capacity::RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y: mat.fy.unwrap_or(345.0), // SD345 相当、要・原典照合
        fc,
        pw,
        sigma_wy: 295.0, // SD295 相当、要・原典照合
        clear_span,
        // 軸方向圧縮応力度は呼び出し側(compute_holding_capacity)が既知の場合に上書きする。
        // ここでは既定値 0(軸力なし・安全側)とする。
        sigma_0: 0.0,
    })
}

/// 長期軸力の簡易近似として先頭荷重ケース(`model.load_cases.first()`)の結果を優先し、
/// `bundle.statics` に無ければ従来どおり最後に実行した静的解析結果(`member_forces`)を
/// 用いて部材の軸力を取得する。圧縮のときのみ σ0 \[N/mm²\]（= |N|/(b・D)）を返す。
/// 引張・軸力なし・対象部材の結果が無い場合は 0.0（安全側）。
///
/// `statics` は `StaticCaseKey` をキーとするため、ユーザー荷重ケースの結果
/// (`StaticCaseKey::User`)と地震静的の結果(`StaticCaseKey::Seismic`)は別々に
/// 格納される（旧実装では両者とも `LoadCaseId(0)` を共有し、後から実行した方が
/// 先頭荷重ケースの結果を上書きしてしまう問題があったが、型で区別したことで解消済み）。
/// 先頭荷重ケースが `statics` に無い（未実行）場合のみ `fallback_member_forces`
/// （最後に実行した静的解析の内力）を用いる。
///
/// # 符号規約（要確認済み・推測ではない）
/// `squid_n_element::beam::BeamElement::recover_forces` は局所剛性 K・u を
/// そのまま評価値とするため、始端(pos=0.0、`eval_sections`\[0\])では
/// `n = f_local[0] = -N`（N は引張正）となる。これは
/// `squid_n_solver::linear::test_linear_static_axial_cantilever` で
/// N=+1000N（引張）を与えたとき `forces.at[0].1[0]` ≈ -1000 になることで
/// 確認済み（すなわち f_local\[0\] は「圧縮正」）。よって `mf.at.first()`
/// (= pos=0.0、始端)の n は「圧縮正」（n>0 のとき圧縮）であり、
/// n<=0（引張または軸力なし）なら σ0=0（安全側）とする。
fn rc_sigma_0_from_gravity_or_last_static(
    statics: &[(StaticCaseKey, squid_n_solver::linear::StaticOnce)],
    fallback_member_forces: &[(ElemId, squid_n_element::beam::MemberForces)],
    gravity_lc: Option<LoadCaseId>,
    elem_id: ElemId,
    b: f64,
    d: f64,
) -> f64 {
    let member_forces = gravity_lc
        .and_then(|lc| {
            statics
                .iter()
                .find(|(id, _)| *id == StaticCaseKey::User(lc))
        })
        .map(|(_, s)| s.member_forces.as_slice())
        .unwrap_or(fallback_member_forces);

    member_forces
        .iter()
        .find(|(id, _)| *id == elem_id)
        .and_then(|(_, mf)| mf.at.first())
        .map(|(_, f)| f[0])
        .filter(|n| *n > 0.0)
        .map(|n| n / (b * d))
        .unwrap_or(0.0)
}

/// 部材両端節点間の幾何長 \[mm\]（内法補正なしの簡易値。剛域等は考慮しない）。
fn elem_geometric_length(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(feature = "gui")]
impl eframe::App for App {
    // eframe のデフォルトは (12,12,12) ≒ 黒なので、テーマに合わせた白灰色で上書きする
    fn clear_color(&self, _visuals: &egui::Visuals) -> [f32; 4] {
        crate::theme::GRAY_100.to_normalized_gamma_f32()
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // バックグラウンド解析ジョブ（P8 §5）: 完了していれば結果を適用し、
        // 実行中は完了検知のため再描画を要求し続ける。
        if self.job.is_some() {
            self.poll_job();
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(100));
        }

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
                ui.separator();
                if ui
                    .button("📥 ST-Bridge 読込…")
                    .on_hover_text(
                        "ST-Bridge 2.0 サブセット（節点・部材・断面・材料・節点荷重）。支点・部材荷重・組合せは含まれません",
                    )
                    .clicked()
                {
                    self.import_stbridge_dialog();
                    ui.close();
                }
                if ui
                    .button("📤 ST-Bridge 書出…")
                    .on_hover_text(
                        "ST-Bridge 2.0 サブセット（節点・部材・断面・材料・節点荷重）。支点・部材荷重・組合せは含まれません",
                    )
                    .clicked()
                {
                    self.export_stbridge_dialog();
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
            Tab::Design => self.design_tab_panel(ui),
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

    /// 「ST-Bridge 読込…」ダイアログを表示して読み込む。
    fn import_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .pick_file()
        {
            self.import_stbridge_from(path);
        }
    }

    /// 「ST-Bridge 書出…」ダイアログを表示して保存先を尋ねる。
    fn export_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .set_file_name("model.stb")
            .save_file()
        {
            self.export_stbridge_to(path);
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
                ("スラブ", ModelTab::Slabs),
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
        }
    }

    /// 解析タブ：種別選択＋実行＋進捗表示。
    fn analysis_tab_panel(&mut self, ui: &mut egui::Ui) {
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
                // model.stories を借用したまま undo.run（model の可変借用）はできないため、
                // 行データを先に複製してから描画・編集確定を行う。
                let story_rows: Vec<(squid_n_core::ids::StoryId, String, f64, usize, Option<f64>)> =
                    self.model
                        .stories
                        .iter()
                        .map(|s| {
                            (
                                s.id,
                                s.name.clone(),
                                s.elevation,
                                s.node_ids.len(),
                                s.seismic_weight,
                            )
                        })
                        .collect();
                self.story_weight_edit.resize(story_rows.len(), 0.0);
                self.story_weight_active.resize(story_rows.len(), false);
                for (i, (story, name, elevation, n_nodes, weight)) in
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
                ui.separator();
                ui.label(match self.analysis_cfg.th_damping_model {
                    ThDampingModel::StiffnessProportional => "減衰比 h:",
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
            },
        };
        self.start_time_history_job(wave);
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

    /// 設計タブ：検定表（許容応力度・保有水平耐力）と MN 相関曲面ビューを切り替える。
    fn design_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let sel_table = self.design_view == DesignView::Table;
            let sel_mn = self.design_view == DesignView::MnSurface;
            if ui.selectable_label(sel_table, "検定表").clicked() {
                self.design_view = DesignView::Table;
            }
            if ui.selectable_label(sel_mn, "MN相関曲面").clicked() {
                self.design_view = DesignView::MnSurface;
            }
        });
        ui.separator();
        match self.design_view {
            DesignView::Table => crate::design_view::design_table(ui, self),
            DesignView::MnSurface => crate::mn_view::mn_surface_panel(ui, self),
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

        // ユーザー荷重ケース0("長期")を先に実行しておく。
        app.run_linear_static(LoadCaseId(0));
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let user_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        // 地震静的の結果は StaticCaseKey::Seismic(X) に格納され、直前に実行した
        // ユーザーケース0(StaticCaseKey::User)の結果を上書きしない
        // (旧実装ではどちらも LoadCaseId(0) を共有し、後勝ちで上書きされていた)。
        let r = app.results.as_ref().unwrap();
        assert_eq!(
            r.statics.len(),
            2,
            "ユーザーケース0と地震静的Xの結果が両方残っているはず"
        );
        let seismic_disp = r
            .statics
            .iter()
            .find(|(k, _)| *k == StaticCaseKey::Seismic(SeismicDir::X))
            .expect("地震静的Xの結果が残っているはず")
            .1
            .disp
            .clone();
        let kept_user_disp = r
            .statics
            .iter()
            .find(|(k, _)| *k == StaticCaseKey::User(LoadCaseId(0)))
            .expect("ユーザーケース0の結果が地震静的実行後も残っているはず")
            .1
            .disp
            .clone();
        assert_eq!(
            kept_user_disp, user_disp,
            "ユーザーケース0の結果は地震静的の実行後も変わらないはず（衝突していない）"
        );
        // 柱頭が X 方向へ変位している(地震静的の結果)
        assert!(seismic_disp[2][0].abs() > 1e-3, "{}", seismic_disp[2][0]);

        // ナビゲータでそれぞれのキーを選択すれば current_static が個別に引ける
        app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
        assert_eq!(app.current_static().unwrap().disp, kept_user_disp);
        app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::Seismic(SeismicDir::X)));
        assert_eq!(app.current_static().unwrap().disp, seismic_disp);

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
        assert!(th.history.node_disp.len() > 100);
        assert!(
            th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
        assert!(th.history.node.is_some());
    }

    #[test]
    fn test_time_history_y_direction_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.analysis_cfg.th_duration = 2.0;
        app.analysis_cfg.th_dir = ThDir::Y;
        app.run_time_history_sample();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
        assert!(
            th.history.record_dir_y,
            "th_dir=Y なのに代表応答の記録方向が X のままです"
        );
        assert!(
            th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
    }

    #[test]
    fn test_build_ground_motion_routes_by_direction() {
        // wave 構築のみを検証する純粋関数のテスト（th_dir=Y でも accel_x 側に
        // 誤って入らないことを確認する）。
        let accel = vec![1.0, 2.0, 3.0];
        let wave_x = App::build_ground_motion(0.01, ThDir::X, accel.clone());
        assert_eq!(wave_x.accel_x, accel);
        assert!(wave_x.accel_y.is_none());

        let wave_y = App::build_ground_motion(0.01, ThDir::Y, accel.clone());
        assert_eq!(wave_y.accel_x, vec![0.0; accel.len()]);
        assert_eq!(wave_y.accel_y, Some(accel.clone()));
    }

    /// ThDir::Xy: 同一波形を accel_x・accel_y の両方に入れる（簡易仕様）。
    #[test]
    fn test_build_ground_motion_xy_duplicates_wave() {
        let accel = vec![1.0, 2.0, 3.0];
        let wave = App::build_ground_motion(0.01, ThDir::Xy, accel.clone());
        assert_eq!(wave.accel_x, accel);
        assert_eq!(wave.accel_y, Some(accel));
    }

    // ===== parse_wave_csv テスト =====

    #[test]
    fn test_parse_wave_csv_single_column_x_or_y() {
        let content = "10.0\n20.0\n30.0\n";
        let (accel, second) = parse_wave_csv(content, ThDir::X).unwrap();
        assert_eq!(accel, vec![100.0, 200.0, 300.0]); // gal→mm/s²(×10)
        assert!(second.is_none());

        // カンマ区切りなら最後の列を使う（従来仕様）。
        let content_csv = "0.0,10.0\n0.01,20.0\n0.02,30.0\n";
        let (accel, second) = parse_wave_csv(content_csv, ThDir::Y).unwrap();
        assert_eq!(accel, vec![100.0, 200.0, 300.0]);
        assert!(second.is_none());
    }

    #[test]
    fn test_parse_wave_csv_single_column_too_few_points_is_err() {
        assert!(parse_wave_csv("10.0\n", ThDir::X).is_err());
        assert!(parse_wave_csv("", ThDir::X).is_err());
    }

    #[test]
    fn test_parse_wave_csv_xy_two_columns() {
        let content = "10.0,5.0\n20.0,15.0\n30.0,25.0\n";
        let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
        assert_eq!(xs, vec![100.0, 200.0, 300.0]);
        assert_eq!(ys, Some(vec![50.0, 150.0, 250.0]));
    }

    #[test]
    fn test_parse_wave_csv_xy_header_line_is_skipped() {
        // ヘッダ行（数値化不可）は無視され、残りの2行が (X, Y) として読める。
        let content = "x,y\n10.0,5.0\n20.0,15.0\n";
        let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
        assert_eq!(xs, vec![100.0, 200.0]);
        assert_eq!(ys, Some(vec![50.0, 150.0]));
    }

    #[test]
    fn test_parse_wave_csv_xy_insufficient_columns_is_err() {
        let content = "10.0,5.0\n20.0\n30.0,25.0\n";
        let err = parse_wave_csv(content, ThDir::Xy).unwrap_err();
        assert_eq!(err, "X+Y には2列のCSVが必要です");
    }

    #[test]
    fn test_parse_wave_csv_xy_too_few_points_is_err() {
        assert!(parse_wave_csv("10.0,5.0\n", ThDir::Xy).is_err());
    }

    #[test]
    fn test_time_history_xy_sample_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.analysis_cfg.th_duration = 2.0;
        app.analysis_cfg.th_dir = ThDir::Xy;
        app.run_time_history_sample();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
        assert!(
            th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
    }

    /// 2 層等質量等剛性せん断モデル（軸ばね 2 本の直列、Ux 方向のみ自由）。
    /// portal_frame は平面骨組で弱軸・面外方向の縮約後自由度が多く、
    /// 固有値解析(部分空間反復)が n_modes=2 で不安定になりやすいため、
    /// Rayleigh 減衰(1次・2次固有値が必要)のテストには本モデルを用いる。
    fn shear_2dof_model() -> squid_n_core::model::Model {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::MaterialId;
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
            Section,
        };
        const FREE_UX: Dof6Mask = Dof6Mask(0b111110);
        let k = 1000.0_f64;
        let m = 1.0_f64;
        let young = k * 1000.0; // EA/L = young*1/1000 = k
        let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
            id: NodeId(id),
            coord: [x, 0.0, 0.0],
            restraint,
            mass,
            story: None,
        };
        let beam = |id: u32, a: u32, b: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        Model {
            nodes: vec![
                node(0, 0.0, Dof6Mask::FIXED, None),
                node(1, 1000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
                node(2, 2000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
            ],
            elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
            sections: vec![Section {
                id: SectionId(0),
                name: "spring".into(),
                area: 1.0,
                iy: 1.0,
                iz: 1.0,
                j: 1.0,
                depth: 1.0,
                width: 1.0,
                as_y: 1.0,
                as_z: 1.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".into(),
                young,
                poisson: 0.0,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_time_history_rayleigh_and_hht() {
        let mut app = App::default();
        app.load_model(shear_2dof_model());
        app.analysis_cfg.th_duration = 2.0;
        app.analysis_cfg.th_damping_model = ThDampingModel::Rayleigh;
        app.analysis_cfg.th_integrator = ThIntegrator::HhtAlpha;
        app.run_time_history_sample();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
        assert!(!th.history.node_disp.is_empty());
        assert!(
            th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
    }

    #[test]
    fn test_set_story_weight_via_ui_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let story_id = app.model.stories[0].id;
        let old_weight = app.model.stories[0].seismic_weight;

        app.undo.run(
            &mut app.model,
            Box::new(squid_n_edit::SetStoryWeight {
                story: story_id,
                weight: Some(12345.0),
            }),
        );
        assert_eq!(app.model.stories[0].seismic_weight, Some(12345.0));

        app.undo.undo(&mut app.model);
        assert_eq!(app.model.stories[0].seismic_weight, old_weight);
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

    /// `poll_job` が完了するまで待つ（タイムアウト5秒でパニック、10ms 間隔でポーリング）。
    fn wait_for_job(app: &mut App) {
        let start = std::time::Instant::now();
        while !app.poll_job() {
            assert!(
                start.elapsed() < std::time::Duration::from_secs(5),
                "ジョブが時間内に完了しませんでした"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    #[test]
    fn test_async_pushover_job_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        app.analysis_cfg.push_steps = 10;

        app.start_pushover_job();
        assert!(app.job.is_some());

        wait_for_job(&mut app);

        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        assert!(app.job.is_none());
        let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
        assert!(!po.capacity_curve.is_empty());
    }

    #[test]
    fn test_async_time_history_job_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.analysis_cfg.th_duration = 2.0;
        let wave = App::sample_wave(&app.analysis_cfg);

        app.start_time_history_job(wave);
        assert!(app.job.is_some());

        wait_for_job(&mut app);

        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        assert!(app.job.is_none());
        let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
        assert!(th.history.node_disp.len() > 100);
        assert!(
            th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
            "応答がゼロのままです"
        );
    }

    #[test]
    fn test_start_job_while_running_is_rejected() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        app.analysis_cfg.push_steps = 10;

        app.start_pushover_job();
        assert!(app.job.is_some());

        // 実行中に再度 start しても2つ目は無視され、job は上書きされない。
        app.start_time_history_job(App::sample_wave(&app.analysis_cfg));
        assert!(app.job.is_some());
        assert_eq!(app.job.as_ref().unwrap().label, "プッシュオーバー");

        wait_for_job(&mut app);

        assert!(app.job.is_none());
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        assert!(app.results.as_ref().unwrap().pushover.is_some());
        assert!(app.results.as_ref().unwrap().time_history.is_none());
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

    #[test]
    fn test_export_and_import_stbridge_roundtrip() {
        let dir = std::env::temp_dir().join("squid_n_app_test_stbridge");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("roundtrip.stb");

        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        let original = app.model.clone();
        app.export_stbridge_to(path.clone());
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        let mut app2 = App::default();
        app2.import_stbridge_from(path.clone());
        assert!(app2.last_error.is_none(), "{:?}", app2.last_error);
        assert!(app2.model.validate().is_ok());
        // ST-Bridge プロジェクト(.scz)とは別物なので project_path は更新されない。
        assert!(app2.project_path.is_none());

        // サブセットのため完全一致(eq_ignoring_dofmap)は求めない
        // （拘束条件・部材荷重は ST-Bridge の対象外で失われる）が、
        // 節点数・部材数はまず一致するはず。
        assert_eq!(app2.model.nodes.len(), original.nodes.len());
        assert_eq!(app2.model.elements.len(), original.elements.len());

        // 座標・部材の接続関係（節点参照・断面・材料・部材軸）はこの門型ラーメンでは
        // 完全にビット一致する（列/梁の判定に依らず節点順序が保たれるケース）。
        for (a, b) in app2.model.nodes.iter().zip(original.nodes.iter()) {
            assert_eq!(a.coord, b.coord);
        }
        assert_eq!(app2.model.elements, original.elements);

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_import_stbridge_missing_file_sets_error() {
        let mut app = App::default();
        app.import_stbridge_from(std::path::PathBuf::from(
            "/nonexistent/dir/does_not_exist.stb",
        ));
        assert!(app.last_error.is_some());
    }

    #[test]
    fn test_combination_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());

        let combo = squid_n_core::model::LoadCombination {
            name: "G+Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        };
        app.undo.run(
            &mut app.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );

        app.run_combination(0);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let bundle = app.results.as_ref().unwrap();
        assert_eq!(bundle.combos.len(), 1);
        assert!(!bundle.checks.is_empty());
        assert_eq!(app.last_static, Some(StaticKey::Combo(0)));
    }

    #[test]
    fn test_current_static_priority() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.run_linear_static(LoadCaseId(0));
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let expected_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

        // ナビゲータで存在しない Combo を選択していても last_static にフォールバックする
        app.nav.focus_result = Some(StaticKey::Combo(9));
        let fallback = app
            .current_static()
            .expect("無効な選択時は last_static にフォールバックするはず");
        assert_eq!(fallback.disp, expected_disp);

        // Case を選択すれば該当ケースの結果が返る
        app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
        let by_case = app
            .current_static()
            .expect("Case 選択時は該当ケースの結果が返るはず");
        assert_eq!(by_case.disp, expected_disp);
    }

    #[test]
    fn test_holding_capacity_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());

        // 階が未定義 → Err
        assert!(app.compute_holding_capacity().is_err());

        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        // プッシュオーバー未実行 → Err
        assert!(app.compute_holding_capacity().is_err());

        app.analysis_cfg.push_steps = 10;
        app.run_pushover();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        let (result, story_ranks) = app
            .compute_holding_capacity()
            .expect("前提が揃えば Ok のはず");
        assert_eq!(result.stories.len(), 1);
        assert!(result.stories[0].qun > 0.0);
        // Qu はプッシュオーバー最終点の層せん断（capacity_curve.story_shear）から取得される。
        assert!(result.stories[0].qu > 0.0, "{}", result.stories[0].qu);
        // design_rank_auto=false（既定）→ 全層フォールバック（選択値 design_rank）。
        assert_eq!(story_ranks, vec![app.design_rank]);
        assert!(result.member_ranks.is_empty());
    }

    /// UI-13: `design_rank_auto = true` で鋼部材の幅厚比から部材ランクを自動判定する。
    /// portal_frame の柱(H-300x300x10x15)・梁(H-400x200x8x13)の幅厚比を手計算し、
    /// `s_member_rank` の結果と一致することを確認する。
    #[test]
    fn test_holding_capacity_rank_auto_from_width_thickness() {
        use squid_n_design_jp::ds::{max_width_thickness, s_member_rank, worst_rank, RankCriteria};

        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.analysis_cfg.push_steps = 10;
        app.run_pushover();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.design_rank_auto = true;
        let (result, story_ranks) = app
            .compute_holding_capacity()
            .expect("shape 付き鋼断面があれば Ok のはず");

        assert!(
            !result.member_ranks.is_empty(),
            "鋼部材の幅厚比からランクが算定されているはず"
        );

        // 柱 H-300x300x10x15: flange=300/(2*15)=10.0, web=(300-30)/10=27.0 → max=27.0
        let col_wt = max_width_thickness(app.model.sections[0].shape.as_ref().unwrap()).unwrap();
        let col_rank = s_member_rank(col_wt, &RankCriteria::default());
        // 梁 H-400x200x8x13: flange=200/(2*13)=7.69, web=(400-26)/8=46.75 → max=46.75
        let beam_wt = max_width_thickness(app.model.sections[1].shape.as_ref().unwrap()).unwrap();
        let beam_rank = s_member_rank(beam_wt, &RankCriteria::default());

        for (elem_id, rank) in &result.member_ranks {
            let expected = if elem_id.0 == 2 { beam_rank } else { col_rank };
            assert_eq!(
                *rank, expected,
                "ElemId({}) のランクが手計算値と一致しません",
                elem_id.0
            );
        }
        // 唯一の層の代表ランクは柱・梁のうち最悪値（FD 寄り）。
        assert_eq!(story_ranks.len(), 1);
        assert_eq!(story_ranks[0], worst_rank(&[col_rank, beam_rank]).unwrap());
    }

    /// SectionShape::RcRect の配筋情報から `rc_capacity_input_from_rect` で
    /// `RcCapacityInput` を組み立てる経路そのものを検証する（RcRect→入力構築）。
    /// 得られた入力から `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank` の結果が、
    /// 同じ式を独立に書き下した手計算と一致することを確認する。
    #[test]
    fn test_rc_capacity_input_from_rect_matches_handcalc() {
        use squid_n_core::ids::MaterialId;
        use squid_n_core::model::Material;
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        use squid_n_design_jp::ds::{rc_member_rank, RankCriteria};
        use squid_n_design_jp::rc_capacity::{rc_qmu_simple, rc_qsu_simple};

        let b = 400.0;
        let d = 600.0;
        let rebar = RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 150.0,
                legs: 2,
            },
        };
        // 材料名は "FC24"（is_steel が false になる、かつ fc 設定あり）を想定。
        let mat = Material {
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None, // 未設定 → sigma_y は 345(SD345相当)にフォールバックするはず
        };
        let clear_span = 3000.0;

        let input = rc_capacity_input_from_rect(b, d, &rebar, &mat, clear_span)
            .expect("fc が設定されているので Some のはず");

        // 変換規則の確認: at=main_x総断面積の半分、d_eff=d-cover-dia/2、
        // pw=せん断補強筋断面積・組数/(b・ピッチ)、sigma_y は fy 未設定なので 345 固定、
        // sigma_wy は常に 295 固定。
        let main_area = 8.0 * std::f64::consts::PI / 4.0 * 22.0 * 22.0;
        let at_expected = main_area / 2.0;
        let d_eff_expected = 600.0 - 40.0 - 22.0 / 2.0;
        let shear_area = std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0;
        let pw_expected = shear_area / (400.0 * 150.0);
        assert!((input.at - at_expected).abs() < 1e-9);
        assert!((input.d_eff - d_eff_expected).abs() < 1e-9);
        assert!((input.pw - pw_expected).abs() < 1e-12);
        assert_eq!(input.sigma_y, 345.0);
        assert_eq!(input.sigma_wy, 295.0);
        assert_eq!(input.fc, 24.0);
        assert_eq!(input.clear_span, clear_span);

        // rc_qsu_simple/rc_qmu_simple の結果を、式を独立に書き下した手計算と照合する。
        let j = 7.0 * d_eff_expected / 8.0;
        let mu_handcalc = 0.9 * at_expected * 345.0 * j;
        let qmu_handcalc = 2.0 * mu_handcalc / clear_span;
        let pt = 100.0 * at_expected / (400.0 * d_eff_expected);
        let shear_span_ratio = (clear_span / (2.0 * d_eff_expected)).clamp(1.0, 3.0);
        let pw_clamped = pw_expected.clamp(0.0, 0.012);
        let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (shear_span_ratio + 0.12);
        let hoop_term = 0.85 * (pw_clamped * 295.0_f64).sqrt();
        let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;

        let qmu = rc_qmu_simple(&input);
        let qsu = rc_qsu_simple(&input);
        assert!(
            (qmu - qmu_handcalc).abs() < 1e-3,
            "Qmu={} vs handcalc={}",
            qmu,
            qmu_handcalc
        );
        assert!(
            (qsu - qsu_handcalc).abs() < 1e-3,
            "Qsu={} vs handcalc={}",
            qsu,
            qsu_handcalc
        );

        let rank = rc_member_rank(qsu, qmu, &RankCriteria::default());
        let rank_handcalc = rc_member_rank(qsu_handcalc, qmu_handcalc, &RankCriteria::default());
        assert_eq!(rank, rank_handcalc);
        // Qsu/Qmu ≈ 2.12（曲げ降伏が十分先行する健全な配筋）なので FA になるはず。
        assert_eq!(rank, squid_n_design_jp::holding_capacity::MemberRank::FA);
    }

    /// UI-13(RC): SectionShape::RcRect + fc 付き材料（コンクリート、is_steel=false）を
    /// 持つ小さな門型ラーメンを組み、rank-auto で member_ranks に RC 部材のランクが入り、
    /// `rc_capacity_input_from_rect` → `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank`
    /// の手計算と一致することを確認する。
    #[test]
    fn test_holding_capacity_rank_auto_rc_rect_from_shape() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::MaterialId;
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
            MemberLoad, MemberLoadKind, Model, NodalLoad, Node,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
        use squid_n_design_jp::ds::{rc_member_rank, RankCriteria};
        use squid_n_design_jp::rc_capacity::{rc_qmu_simple, rc_qsu_simple};

        let rebar = RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 150.0,
                legs: 2,
            },
        };
        let rc_shape = SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: rebar.clone(),
        };

        let mut model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [4000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(3),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
            materials: vec![Material {
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let members = [
            (0u32, 0u32, 2u32, [1.0, 0.0, 0.0]),
            (1, 1, 3, [1.0, 0.0, 0.0]),
            (2, 2, 3, [0.0, 0.0, 1.0]),
        ];
        for (id, i, j, ref_vector) in members {
            model.elements.push(ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis { ref_vector },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            });
        }
        model.load_cases.push(LoadCase {
            id: LoadCaseId(0),
            name: "長期".into(),
            nodal: Vec::new(),
            member: vec![MemberLoad {
                elem: ElemId(2),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 4000.0,
                    w1: 10.0,
                    w2: 10.0,
                },
            }],
        });
        model.load_cases.push(LoadCase {
            id: LoadCaseId(1),
            name: "地震X".into(),
            nodal: vec![
                NodalLoad {
                    node: NodeId(2),
                    values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
                NodalLoad {
                    node: NodeId(3),
                    values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
            ],
            member: Vec::new(),
        });

        let mut app = App::default();
        app.load_model(model);
        app.generate_stories_action();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        // RC の簡易断面・fy 未設定材料はヒンジ耐力の既定値(鋼材既定 235N/mm²)を用いる
        // 都合上、既定の push_max_disp=500mm では機構形成後に特異行列となり得るため、
        // 微小変位のみを対象とする(ここではランク判定経路の配線確認が目的で、
        // 崩壊形の精算は対象外)。
        app.analysis_cfg.push_steps = 3;
        app.analysis_cfg.push_max_disp = 3.0;
        app.run_pushover();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        app.design_rank_auto = true;
        let (result, _story_ranks) = app
            .compute_holding_capacity()
            .expect("RC 矩形 + fc 付き材料があれば Ok のはず");

        assert!(
            !result.member_ranks.is_empty(),
            "RC 部材(RcRect+fc)のせん断余裕度からランクが算定されているはず"
        );

        // 柱: 節点間距離 3000mm、梁: 節点間距離 4000mm。それぞれ手計算で
        // rc_capacity_input_from_rect → rc_qsu/qmu_simple → rc_member_rank を再現する。
        //
        // σ0 は実運用と同じ規則(rc_sigma_0_from_gravity_or_last_static)で個別に反映する。
        // このテストでは run_linear_static(先頭ケース="長期")を実行していないため、
        // gravity_lc=LoadCaseId(0) は statics 内の StaticCaseKey::User(LoadCaseId(0))
        // として見つからず、フォールバック(bundle.member_forces = 直近実行した
        // run_seismic の内力)が使われる(= 最後の静的解析結果と同じ)。地震水平力による
        // 柱の転倒モーメント抵抗で柱0・柱1の軸力は一方が圧縮・他方が引張(または
        // 大きさが異なる)になり得るため、部材ごとに算定する(柱を一括りにしない)。
        let mat = &app.model.materials[0];
        let statics = &app.results.as_ref().unwrap().statics;
        let member_forces = &app.results.as_ref().unwrap().member_forces;
        let gravity_lc = app.model.load_cases.first().map(|c| c.id);
        let expected_rank_for = |elem_id: ElemId, clear_span: f64| {
            let mut input = rc_capacity_input_from_rect(400.0, 600.0, &rebar, mat, clear_span)
                .expect("fc 設定済みなので Some");
            input.sigma_0 = rc_sigma_0_from_gravity_or_last_static(
                statics,
                member_forces,
                gravity_lc,
                elem_id,
                400.0,
                600.0,
            );
            let qmu = rc_qmu_simple(&input);
            let qsu = rc_qsu_simple(&input);
            rc_member_rank(qsu, qmu, &RankCriteria::default())
        };
        let col0_rank = expected_rank_for(ElemId(0), 3000.0);
        let col1_rank = expected_rank_for(ElemId(1), 3000.0);
        let beam_rank = expected_rank_for(ElemId(2), 4000.0);

        for (elem_id, rank) in &result.member_ranks {
            let expected = match elem_id.0 {
                2 => beam_rank,
                1 => col1_rank,
                _ => col0_rank,
            };
            assert_eq!(
                *rank, expected,
                "ElemId({}) のランクが手計算値と一致しません",
                elem_id.0
            );
        }
    }

    /// `rc_sigma_0_from_gravity_or_last_static`: 圧縮軸力から σ0 が正しく算定されることを、
    /// 実際に静的解析を実行して確認する。
    ///
    /// モデル: 鉛直片持ち柱（節点0=基部, 固定, z=0 / 節点1=先端, 自由, z=3000）に
    /// RC矩形断面 400x600 を設定し、先端節点へ下向き(圧縮)集中荷重 P=100,000N を
    /// 与える。軸力のみが生じる単純な釣合いなので、内力の軸力の大きさは
    /// 弾性係数・断面性能によらず厳密に P と一致する。
    ///
    /// 符号規約の確認: squid-n-solver::linear::test_linear_static_axial_cantilever
    /// で N=+1000N(引張)のとき forces.at[0].1[0]≈-1000 であることを確認済みなので、
    /// 圧縮(先端を下向きに押す)では forces.at[0].1[0]≈+P（正）になるはず。
    #[test]
    fn test_rc_sigma_0_from_compression_axial_force() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::MaterialId;
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
            Model, NodalLoad, Node,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let b = 400.0;
        let d = 600.0;
        let rebar = RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 150.0,
                legs: 2,
            },
        };
        let rc_shape = SectionShape::RcRect {
            b,
            d,
            rebar: rebar.clone(),
        };

        let p = 100_000.0; // 圧縮荷重 [N]
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
            materials: vec![Material {
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(0),
                name: "圧縮".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 0.0, -p, 0.0, 0.0, 0.0],
                }],
                member: Vec::new(),
            }],
            ..Default::default()
        };

        let mut app = App::default();
        app.load_model(model);
        app.run_linear_static(LoadCaseId(0));
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        let member_forces = &app.results.as_ref().unwrap().member_forces;
        let (_, mf) = member_forces
            .iter()
            .find(|(id, _)| *id == ElemId(0))
            .expect("elem 0 の内力があるはず");
        let n_raw = mf.at.first().expect("eval_sections[0] があるはず").1[0];
        // 圧縮なので符号規約上「圧縮正」の n_raw は正で、大きさは P と厳密に一致する。
        assert!((n_raw - p).abs() < 1e-6, "n_raw={} (expected {})", n_raw, p);

        let statics = &app.results.as_ref().unwrap().statics;
        let gravity_lc = app.model.load_cases.first().map(|c| c.id);
        let sigma_0 = rc_sigma_0_from_gravity_or_last_static(
            statics,
            member_forces,
            gravity_lc,
            ElemId(0),
            b,
            d,
        );
        let expected_sigma_0 = p / (b * d);
        assert!(
            (sigma_0 - expected_sigma_0).abs() < 1e-9,
            "sigma_0={} expected={}",
            sigma_0,
            expected_sigma_0
        );
    }

    /// `rc_sigma_0_from_gravity_or_last_static`: 先頭荷重ケース(gravity_lc)の静的解析結果が
    /// `bundle.statics` にあれば、最後に実行した(かつ結果が異なる)静的解析ではなく
    /// 先頭荷重ケースの結果が優先されることを確認する。
    ///
    /// モデル: `test_rc_sigma_0_from_compression_axial_force` と同じ片持ち柱に、
    /// 先頭荷重ケース(id=0,"長期")として圧縮荷重 P1、2番目のケース(id=1,"地震")として
    /// 引張荷重 P2 を設定する。両ケースをこの順に実行すると
    /// `bundle.member_forces`(=最後に実行したケース)は引張(id=1)の結果になり、
    /// これをそのまま使うと σ0=0(引張は 0 とみなす安全側処理)になってしまう。
    /// 優先順位が正しく効いていれば、`bundle.statics` 内の id=0(長期)の圧縮軸力から
    /// σ0=P1/(b・D) (>0) が算定される。
    #[test]
    fn test_rc_sigma_0_prefers_gravity_load_case_over_last_static() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::MaterialId;
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
            Model, NodalLoad, Node,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let b = 400.0;
        let d = 600.0;
        let rebar = RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 2,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 150.0,
                legs: 2,
            },
        };
        let rc_shape = SectionShape::RcRect {
            b,
            d,
            rebar: rebar.clone(),
        };

        let p1 = 100_000.0; // 先頭ケース(長期)の圧縮荷重 [N]
        let p2 = 60_000.0; // 2番目のケース(地震想定)の引張荷重 [N]
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
            materials: vec![Material {
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            load_cases: vec![
                LoadCase {
                    id: LoadCaseId(0),
                    name: "長期".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, -p1, 0.0, 0.0, 0.0], // 下向き=圧縮
                    }],
                    member: Vec::new(),
                },
                LoadCase {
                    id: LoadCaseId(1),
                    name: "地震".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, p2, 0.0, 0.0, 0.0], // 上向き=引張
                    }],
                    member: Vec::new(),
                },
            ],
            ..Default::default()
        };

        let mut app = App::default();
        app.load_model(model);
        // 先頭ケース(長期,圧縮)→2番目のケース(地震,引張)の順に実行し、
        // 「最後に実行した静的解析結果」は引張(id=1)になるようにする。
        app.run_linear_static(LoadCaseId(0));
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        app.run_linear_static(LoadCaseId(1));
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        let bundle = app.results.as_ref().unwrap();
        // 最後に実行した静的解析結果(bundle.member_forces)は引張なので、
        // これをそのまま使うと σ0=0 になってしまうことの確認(比較対象)。
        let sigma_0_last_only = rc_sigma_0_from_gravity_or_last_static(
            &[],
            &bundle.member_forces,
            None,
            ElemId(0),
            b,
            d,
        );
        assert_eq!(sigma_0_last_only, 0.0, "引張のみなら σ0=0 のはず(比較対象)");

        // 優先順位が正しく効いていれば、先頭ケース(長期,id=0)の圧縮軸力から
        // σ0=P1/(b・D) (>0) が算定される。
        let gravity_lc = app.model.load_cases.first().map(|c| c.id);
        assert_eq!(gravity_lc, Some(LoadCaseId(0)));
        let sigma_0 = rc_sigma_0_from_gravity_or_last_static(
            &bundle.statics,
            &bundle.member_forces,
            gravity_lc,
            ElemId(0),
            b,
            d,
        );
        let expected_sigma_0 = p1 / (b * d);
        assert!(
            (sigma_0 - expected_sigma_0).abs() < 1e-9,
            "sigma_0={} expected={}(先頭ケースの圧縮軸力が優先されるはず)",
            sigma_0,
            expected_sigma_0
        );
    }

    /// Z=0 平面の矩形（4000×6000）+外周4本の梁 + スラブ1枚（TriTrapezoid）を持つモデルを作る。
    /// 辺 i = boundary[i] → boundary[(i+1)%4] の順に梁を並べる（refresh_beam_loads の対応付けと一致）。
    fn make_slab_test_model() -> squid_n_core::model::Model {
        use squid_n_core::ids::SlabId;
        use squid_n_core::model::{
            AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
            LocalAxis, Node, Slab,
        };

        let mk_node = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, 4000.0, 0.0),
            mk_node(2, 4000.0, 6000.0),
            mk_node(3, 0.0, 6000.0),
        ];
        let mk_beam = |id: u32, i: u32, j: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        let elements = vec![
            mk_beam(0, 0, 1),
            mk_beam(1, 1, 2),
            mk_beam(2, 2, 3),
            mk_beam(3, 3, 0),
        ];
        let slab = Slab {
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: 0.005,
            }],
            method: DistributionMethod::TriTrapezoid,
        };
        squid_n_core::model::Model {
            nodes,
            elements,
            slabs: vec![slab],
            ..Default::default()
        }
    }

    #[test]
    fn test_refresh_beam_loads_maps_edges_to_members() {
        let model = make_slab_test_model();
        model
            .validate()
            .expect("テストモデルは validate を通るはず");

        let mut app = App {
            model,
            ..App::default()
        };
        app.refresh_beam_loads();

        assert_eq!(app.beam_loads.len(), 4, "外周4辺すべてに荷重が対応付くはず");
        for bl in &app.beam_loads {
            let elem = app
                .model
                .elements
                .iter()
                .find(|e| e.id == bl.elem)
                .expect("beam_loads.elem は実在する部材IDを指すはず");
            assert_eq!(elem.kind, squid_n_core::model::ElementKind::Beam);
            assert!(
                bl.cmq.c_i.abs() > 1e-9 || bl.cmq.q_i.abs() > 1e-9,
                "CMQ が非ゼロのはず: {:?} {:?}",
                bl.cmq.c_i,
                bl.cmq.q_i
            );
        }

        // 梁が1本欠けたモデルでは、対応する辺の荷重が捨てられ3件になる
        let mut missing = app.model.clone();
        missing.elements.pop();
        app.model = missing;
        app.refresh_beam_loads();
        assert_eq!(app.beam_loads.len(), 3);
    }
}
