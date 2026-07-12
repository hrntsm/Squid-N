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
    /// 壁属性（開口・三方スリット）
    WallAttrs,
    /// フレーム外雑壁
    MiscWalls,
}

/// 結果タブ内の切替（3D 各種図・時刻歴グラフ・プッシュオーバー曲線）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ResultsView {
    #[default]
    Spatial,
    TimeHistory,
    Pushover,
}

/// 設計タブ内の切替（検定表・終局検定表・MN相関曲面ビュー）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DesignView {
    #[default]
    Table,
    /// 終局検定（RESP-D「06 終局検定」塑性理論式による終局せん断・付着余裕度）。
    Ultimate,
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
    /// 風荷重静的解析。方向別に共存できる
    Wind(SeismicDir),
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
    /// 節点単位の検定結果（柱梁接合部・パネルゾーン・冷間成形耐力比など）。
    /// ラベルは「接合部(RC)」等の種別表示用。
    pub joint_checks: Vec<(
        squid_n_core::ids::NodeId,
        String,
        squid_n_design_jp::CheckResult,
    )>,
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
    /// プッシュオーバー: 塑性率（ductility）の算定方式（RESP-D「05 非線形モデル」）。
    pub ductility_method: squid_n_solver::pushover::DuctilityMethod,
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
    /// 荷重組合せ自動生成（種別ベース）の多雪区域フラグ（施行令86条・82条）。
    pub heavy_snow_zone: bool,
    /// 多雪区域の積雪荷重低減係数 δ1（長期 G+P+δ1・S。既定 0.7）。
    pub snow_delta1: f64,
    /// 同 δ2（暴風時 G+P+δ2・S±W。既定 0.35）。
    pub snow_delta2: f64,
    /// 同 δ3（地震時 G+P+δ3・S±K。既定 0.35）。
    pub snow_delta3: f64,
    /// RC 短期許容せん断力の「損傷制御のための検討」（false=安全確保のための検討）。
    /// RESP-D マニュアル 04「断面算定条件 RC造」に対応。
    pub rc_damage_control: bool,
    /// 地震時短期の設計用せん断力 QD の決定方法（QD1/QD2/min）。
    pub qd_method: squid_n_design_jp::QdMethod,
    /// 風荷重静的解析の基準風速 V0 [m/s]。
    pub v0: f64,
    /// 風荷重静的解析の地表面粗度区分。
    pub roughness: squid_n_load::wind::TerrainRoughness,
    /// 風荷重静的解析のパラペット高さ [mm]。
    pub parapet_mm: f64,
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
            ductility_method: squid_n_solver::pushover::DuctilityMethod::default(),
            th_damping: 0.02,
            th_dt: 0.01,
            th_duration: 10.0,
            th_period: 0.5,
            th_amp: 1000.0,
            th_dir: ThDir::X,
            th_damping_model: ThDampingModel::StiffnessProportional,
            th_h2: 0.02,
            th_integrator: ThIntegrator::NewmarkBeta,
            heavy_snow_zone: false,
            snow_delta1: 0.7,
            snow_delta2: 0.35,
            snow_delta3: 0.35,
            rc_damage_control: true,
            qd_method: squid_n_design_jp::QdMethod::Min,
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            parapet_mm: 0.0,
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
    pub design_frame: squid_n_design_jp::secondary::holding_capacity::FrameType,
    /// 保有水平耐力（ルート3）判定の部材ランク（Ds 表の列選択）。
    /// `design_rank_auto == true` の場合はフォールバック用（幅厚比を算定できない
    /// 層のみに適用される）。
    pub design_rank: squid_n_design_jp::secondary::holding_capacity::MemberRank,
    /// 保有水平耐力（ルート3）の部材ランクを鋼部材の幅厚比から自動判定するか（UI-13）。
    /// true の場合、鋼部材かつ断面形状(`Section.shape`)を持つ部材について
    /// `squid_n_design_jp::secondary::width_thickness::max_width_thickness` →
    /// `s_member_rank` で算定し、
    /// 算定できなかった層のみ `design_rank`（選択値）にフォールバックする。
    pub design_rank_auto: bool,
    /// 終局検定（RESP-D「06 終局検定」）のヒンジ回転角 Rp [rad]（ν・cotφ 用。既定 0）。
    pub ultimate_rp: f64,
    /// 終局検定で軽量コンクリートのせん断終局耐力 0.9 倍低減を適用するか。
    pub ultimate_lightweight: bool,
    /// 終局検定で付着割裂耐力 Qbu の余裕度を算定するか。
    pub ultimate_include_bond: bool,
    /// 終局検定の上限強度倍率（Qmu = 上限強度倍率·2·Mu/内法。既定 1.0）。
    pub ultimate_upper_factor: f64,
    /// 終局検定で柱の Mu を ACI 規準（平面保持）で算定するか（false は at 式）。
    pub ultimate_mu_aci: bool,
    /// 終局検定の終局せん断強度に靭性指針式 Vu を用いるか（false=塑性理論式 Qsu）。
    pub ultimate_shear_ductility: bool,
    /// 終局検定で柱のせん断を 2 軸せん断として検定するか（RC 柱の 2 軸せん断余裕度）。
    pub ultimate_biaxial_shear: bool,
    /// 終局検定で柱の曲げを 2 軸曲げとして検定するか（RC 柱の 2 軸曲げ余裕度）。
    pub ultimate_biaxial_bending: bool,
    /// 終局検定の設計用応力（Qmu・需要曲げ）と部材別 Rp をプッシュオーバー応答から
    /// 直接反映するか（false は静的解析応答＋UI 一律 Rp）。プッシュオーバー未実行時は
    /// 自動的に静的応答へフォールバックする。
    pub ultimate_use_pushover: bool,
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
    /// 地震地域係数 Z の市町村別ローダ（CSV読込結果）。ヘッドレスでも使うため
    /// gui 限定にしない（`load_z_table_from_csv`/`apply_z_from_municipality` から参照）。
    pub z_table: Option<squid_n_load::z_table::ZTable>,
    /// Z表 CSV 読込 UI の市町村名入力バッファ。
    #[cfg(feature = "gui")]
    pub z_table_municipality: String,
    /// モデルタブ「壁属性」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub wall_attr_draft: crate::tables::wall_attrs::WallAttrDraft,
    /// モデルタブ「雑壁」追加フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub misc_wall_draft: crate::tables::misc_walls::MiscWallDraft,
    /// 荷重タブ「荷重計算条件」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub load_cfg_draft: crate::tables::load_cfg::LoadCfgDraft,
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
            design_frame: squid_n_design_jp::secondary::holding_capacity::FrameType::SteelFrame,
            design_rank: squid_n_design_jp::secondary::holding_capacity::MemberRank::FA,
            design_rank_auto: false,
            ultimate_rp: 0.0,
            ultimate_lightweight: false,
            ultimate_include_bond: true,
            ultimate_upper_factor: 1.0,
            ultimate_mu_aci: false,
            ultimate_shear_ductility: false,
            ultimate_biaxial_shear: false,
            ultimate_biaxial_bending: false,
            ultimate_use_pushover: false,
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
            z_table: None,
            #[cfg(feature = "gui")]
            z_table_municipality: String::new(),
            #[cfg(feature = "gui")]
            wall_attr_draft: crate::tables::wall_attrs::WallAttrDraft::default(),
            #[cfg(feature = "gui")]
            misc_wall_draft: crate::tables::misc_walls::MiscWallDraft::default(),
            #[cfg(feature = "gui")]
            load_cfg_draft: crate::tables::load_cfg::LoadCfgDraft::default(),
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

/// `sync_slab_loads_action` が同期先とする専用荷重ケース名（レビュー §1.1）。
pub const SLAB_AUTO_LOAD_CASE_NAME: &str = "床荷重(自動)";

/// 節点ペアが鉛直材（柱）かどうかを判定する。両端の水平距離（XY平面）が
/// 1mm 未満なら鉛直とみなす（`squid_n_load::story_gen::is_vertical_pair` と
/// 同じ判定規則。あちらは非公開のためここで同じ規則を再実装する）。
fn is_vertical_pair(a: [f64; 3], b: [f64; 3]) -> bool {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt() < 1.0
}

/// 柱要素ごとの「支持する床数」と「積載荷重低減率」（令85条2項）を一覧する。
///
/// `Model.load_cfg.live_load_reduction == true` のときに UI 側で参考表示する
/// ための集計ヘルパー（`squid_n_load::live_load::{floors_supported_by_column,
/// column_live_load_reduction}` を薄くラップする）。**断面検定の長期軸力への
/// 実適用はまだ行っていない**（解析パイプラインへの侵襲が大きいため残課題。
/// 本関数は表示用の集計のみを提供する）。
///
/// 柱の判定は `ElementKind::Beam` の部材のうち両端節点の水平距離が 1mm 未満
/// （鉛直材）のものとする。所属階は `Node.story`（`ApplyStories`/階の自動生成で
/// 設定される）を用いるため、階が未生成のモデルでは全部材が 0 層扱いになる。
pub fn column_live_load_factors(model: &squid_n_core::model::Model) -> Vec<(ElemId, usize, f64)> {
    use squid_n_core::model::ElementKind;

    let node_story: Vec<Option<squid_n_core::ids::StoryId>> =
        model.nodes.iter().map(|n| n.story).collect();

    model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam && e.nodes.len() >= 2)
        .filter_map(|e| {
            let ni = e.nodes[0].index();
            let nj = e.nodes[1].index();
            if ni >= model.nodes.len() || nj >= model.nodes.len() {
                return None;
            }
            if !is_vertical_pair(model.nodes[ni].coord, model.nodes[nj].coord) {
                return None;
            }
            let floors = squid_n_load::live_load::floors_supported_by_column(model, e, &node_story);
            let factor = squid_n_load::live_load::column_live_load_reduction(floors);
            Some((e.id, floors, factor))
        })
        .collect()
}

/// 地震用重量に算入する重力ケースを `LoadCaseKind` から選択する（レビュー §1.7）。
///
/// - `kind == Dead` の全ケースを対象とする。
/// - `kind == LiveSeismic`（地震用積載）のケースがあれば併せて対象とする。
///   無ければ `kind == Live`（長期用積載）で代用する
///   （マニュアル「床の積載荷重は地震用の値とします」の趣旨。地震用の値が
///   個別に定義されていなければ長期用の値をそのまま使う）。
/// - いずれのケースも `kind` が設定されていない（全ケースが既定値 `Other`）
///   場合は、旧スキーマ・後方互換のため先頭ケースのみを返す
///   （並び順に依存する旧規約。新規モデルは kind 設定を推奨）。
fn gravity_cases_for_seismic_weight(model: &squid_n_core::model::Model) -> Vec<LoadCaseId> {
    use squid_n_core::model::LoadCaseKind;

    let any_kind_set = model
        .load_cases
        .iter()
        .any(|lc| lc.kind != LoadCaseKind::Other);
    if !any_kind_set {
        return model.load_cases.first().map(|c| c.id).into_iter().collect();
    }

    let mut result: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::Dead)
        .map(|lc| lc.id)
        .collect();

    let live_seismic: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::LiveSeismic)
        .map(|lc| lc.id)
        .collect();
    if !live_seismic.is_empty() {
        result.extend(live_seismic);
    } else {
        result.extend(
            model
                .load_cases
                .iter()
                .filter(|lc| lc.kind == LoadCaseKind::Live)
                .map(|lc| lc.id),
        );
    }
    result
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

/// 鋼材判定（Material.name が JIS 鋼種名で始まるか）。
/// 鉄筋（SD/SR）は RC 扱いのため含めない。
fn is_steel(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
        || upper.starts_with("SA")
        || upper.starts_with("BC")
}

/// 部材種別判定（部材軸の鉛直成分による幾何判定）。
///
/// - |ez| ≥ 0.8: 柱（軸力＋二軸曲げの複合検定）
/// - |ez| ≤ 0.2: 梁（強軸曲げ＋せん断）
/// - それ以外: ブレース（軸力検定）
fn member_kind_of(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> squid_n_design_jp::MemberKind {
    use squid_n_design_jp::MemberKind;
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (dz / len).abs();
    if ez >= 0.8 {
        squid_n_design_jp::MemberKind::Column
    } else if ez <= 0.2 {
        squid_n_design_jp::MemberKind::Beam
    } else {
        squid_n_design_jp::MemberKind::Brace
    }
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
) -> Option<squid_n_design_jp::secondary::rc_capacity::RcCapacityInput> {
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
    Some(squid_n_design_jp::secondary::rc_capacity::RcCapacityInput {
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

    // 軸力 N は引張正の部材内力（beam.rs recover_forces）。圧縮（N<0）のみ
    // σ0 に反映し、引張は 0 とする（安全側）。
    member_forces
        .iter()
        .find(|(id, _)| *id == elem_id)
        .and_then(|(_, mf)| mf.at.first())
        .map(|(_, f)| f[0])
        .filter(|n| *n < 0.0)
        .map(|n| -n / (b * d))
        .unwrap_or(0.0)
}

/// 鋼断面形状の最大板厚 [mm]（F 値の板厚区分判定用）。
/// 形状情報のない断面・RC 断面は 0（板厚 40mm 以下の区分扱い）。
fn steel_max_thickness(shape: &squid_n_core::section_shape::SectionShape) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    match *shape {
        SectionShape::SteelH {
            web_thick,
            flange_thick,
            ..
        }
        | SectionShape::SteelChannel {
            web_thick,
            flange_thick,
            ..
        }
        | SectionShape::SteelTee {
            web_thick,
            flange_thick,
            ..
        } => web_thick.max(flange_thick),
        SectionShape::SteelBox { thick, .. }
        | SectionShape::SteelAngle { thick, .. }
        | SectionShape::SteelPipe { thick, .. }
        | SectionShape::CftBox { thick, .. }
        | SectionShape::CftPipe { thick, .. } => thick,
        SectionShape::SrcRect {
            steel_web_thick,
            steel_flange_thick,
            ..
        } => steel_web_thick.max(steel_flange_thick),
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::RcWall { .. } => 0.0,
    }
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

/// 一本部材グループ 1 本分の検定文脈（RESP-D マニュアル 04 断面検定
/// 「採用応力 ■一本部材指定時の採用応力」）。
struct BeamGroupOverride {
    /// 一本部材の全長 L [mm]（分割部材長の総和）。
    length: f64,
    /// 一本部材両端の強軸曲げ (M_i端, M_j端) [N·mm]。
    end_moments_z: Option<(f64, f64)>,
    /// 一本部材中央の強軸曲げ Mc [N·mm]。A 式（M0=(Q1+Q2)L/8 による復元値）と
    /// B 式（中央に位置する分割部材の内力）の大きい方を採用する（マニュアル）。
    mid_moment_z: Option<f64>,
    /// グループ内 |Mz| 最大位置の (|M|, |Q|)（せん断スパン比の代表値）。
    shear_span: Option<(f64, f64)>,
    /// 一本部材の内法長（両外端の剛域控除後）[mm]。
    clear_length: f64,
}

/// `Model.beam_groups` の各グループについて検定文脈の合成値を求め、
/// 所属要素 ID → 合成値の対応表を返す。
///
/// - グループは軸方向に連続する梁要素の ID を**並び順**で持つ前提
///   （幾何学的な連続性・共線性の検証は行わない。並び順が実際の配置と
///   異なる場合、端部モーメント等の対応がずれる）。
/// - 要素または内力が欠けるグループ・要素数 2 未満のグループは無視する。
/// - 中央モーメントは、A 式 `Mc_A = (|Q1|+|Q2|)・L/8 − (|M1|+|M2|)/2`
///   （端部せん断と釣り合う等分布荷重の単純梁中央モーメントから端部
///   モーメントの平均を差し引いた復元値）と、B 式（グループ中央位置を
///   含む分割部材の、中央位置に最も近い評価行のモーメント）の絶対値の
///   大きい方（符号は B 式に合わせる）。
fn beam_group_overrides(
    model: &squid_n_core::model::Model,
    member_forces: &[(ElemId, squid_n_element::beam::MemberForces)],
) -> std::collections::HashMap<ElemId, std::rc::Rc<BeamGroupOverride>> {
    use std::collections::HashMap;
    use std::rc::Rc;
    let mut out: HashMap<ElemId, Rc<BeamGroupOverride>> = HashMap::new();

    for group in &model.beam_groups {
        if group.len() < 2 {
            continue;
        }
        // 各分割部材の (要素, 内力, 長さ) を並び順に収集。欠けがあればスキップ。
        let mut parts: Vec<(
            &squid_n_core::model::ElementData,
            &squid_n_element::beam::MemberForces,
            f64,
        )> = Vec::with_capacity(group.len());
        let mut ok = true;
        for id in group {
            let elem = model.elements.iter().find(|e| e.id == *id);
            let mf = member_forces
                .iter()
                .find(|(mid, _)| mid == id)
                .map(|(_, m)| m);
            match (elem, mf) {
                (Some(e), Some(m)) if !m.at.is_empty() => {
                    let l = elem_geometric_length(e, model);
                    if l <= 1e-9 {
                        ok = false;
                        break;
                    }
                    parts.push((e, m, l));
                }
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok || parts.len() < 2 {
            continue;
        }

        let total: f64 = parts.iter().map(|p| p.2).sum();
        let row_at = |m: &squid_n_element::beam::MemberForces, target: f64| -> Option<[f64; 6]> {
            m.at.iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| *f)
        };
        let first = parts[0].1;
        let last = parts[parts.len() - 1].1;
        let end_i = row_at(first, 0.0);
        let end_j = row_at(last, 1.0);
        let end_moments_z = match (end_i, end_j) {
            (Some(a), Some(b)) => Some((a[5], b[5])),
            _ => None,
        };

        // A 式: M0 = (Q1+Q2)・L/8（端部せん断と釣り合う等分布仮定）。
        let q1 = end_i.map(|f| f[1].abs()).unwrap_or(0.0);
        let q2 = end_j.map(|f| f[1].abs()).unwrap_or(0.0);
        let m0_a = (q1 + q2) * total / 8.0;
        let m_ends_avg = end_moments_z
            .map(|(a, b)| (a.abs() + b.abs()) / 2.0)
            .unwrap_or(0.0);
        let mc_a = m0_a - m_ends_avg;

        // B 式: グループ中央位置を含む分割部材の、中央位置に最も近い評価行。
        let target_s = total / 2.0;
        let mut acc = 0.0;
        let mut mc_b: Option<f64> = None;
        for (_, m, l) in &parts {
            if target_s <= acc + l + 1e-9 {
                let xi = ((target_s - acc) / l).clamp(0.0, 1.0);
                mc_b =
                    m.at.iter()
                        .min_by(|a, b| {
                            (a.0 - xi)
                                .abs()
                                .partial_cmp(&(b.0 - xi).abs())
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(_, f)| f[5]);
                break;
            }
            acc += l;
        }
        let mid_moment_z = mc_b.map(|b| {
            let sign = if b >= 0.0 { 1.0 } else { -1.0 };
            sign * b.abs().max(mc_a)
        });

        let shear_span = parts
            .iter()
            .flat_map(|(_, m, _)| m.at.iter())
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let face_sum = parts[0].0.rigid_zone.face_i + parts[parts.len() - 1].0.rigid_zone.face_j;
        let clear_length = if total - face_sum > 0.0 {
            total - face_sum
        } else {
            total
        };

        let ov = Rc::new(BeamGroupOverride {
            length: total,
            end_moments_z,
            mid_moment_z,
            shear_span,
            clear_length,
        });
        for (e, _, _) in &parts {
            out.insert(e.id, ov.clone());
        }
    }
    out
}

/// 危険断面位置（§6.2.3、既定は柱フェイスと中央）を正規化座標 \[0,1\] で算定する。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` 算定と同じ規則
/// （xi_i は \[0.0, 0.5) へ、xi_j は (0.5, 1.0\] へクランプ）で face_i/face_j から
/// 求める。face=0（直交材が無い端）では節点芯（0.0/1.0）と一致する。
fn design_positions(elem: &squid_n_core::model::ElementData, geom_len: f64) -> [f64; 3] {
    if geom_len > 1e-12 {
        let xi_i = (elem.rigid_zone.face_i / geom_len).clamp(0.0, 0.5 - 1e-9);
        let xi_j = (1.0 - elem.rigid_zone.face_j / geom_len).clamp(0.5 + 1e-9, 1.0);
        [xi_i, 0.5, xi_j]
    } else {
        [0.0, 0.5, 1.0]
    }
}

/// `pos` が `positions` のいずれかと 1e-6 以内で一致するか判定する。
fn is_near_design_position(pos: f64, positions: &[f64; 3]) -> bool {
    positions.iter().any(|p| (p - pos).abs() < 1e-6)
}

mod actions;

#[cfg(feature = "gui")]
mod panels;

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

#[cfg(test)]
mod tests;
