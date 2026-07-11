use squid_n_core::model::Model;
use squid_n_edit::UndoStack;
// `FsResultStore` はメソッド(`writer`/`query`/`manifest`)をトレイト経由でのみ提供するため、
// 具象型への `.writer()` 等のドット呼び出しにはこのトレイトのインポートが要る
// （`&dyn ResultStore` 経由の呼び出しはトレイトオブジェクト自体がトレイトを表すため不要）。
use squid_n_io::results::ResultStore;
use std::collections::HashMap;
use std::path::PathBuf;

pub type JobId = String;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, schemars::JsonSchema)]
pub enum JobStatus {
    Queued,
    Running { progress: f32 },
    Done { result_ref: String },
    Failed { error: String },
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, schemars::JsonSchema,
)]
pub enum JobKind {
    LinearStatic,
    Eigen,
    Pushover,
    TimeHistory,
    DesignCheck,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Job {
    pub id: JobId,
    pub kind: JobKind,
    pub status: JobStatus,
}

#[derive(Debug, Default)]
pub struct JobRegistry {
    jobs: HashMap<JobId, Job>,
    next_id: u64,
}

impl JobRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, kind: JobKind) -> JobId {
        let id = format!("job-{}", self.next_id);
        self.next_id += 1;
        let job = Job {
            id: id.clone(),
            kind,
            status: JobStatus::Queued,
        };
        self.jobs.insert(id.clone(), job);
        id
    }

    pub fn get(&self, id: &str) -> Option<&Job> {
        self.jobs.get(id)
    }

    pub fn update(&mut self, id: &str, status: JobStatus) {
        if let Some(job) = self.jobs.get_mut(id) {
            job.status = status;
        }
    }
}

/// 結果ストア本体。
///
/// `ResultStore` トレイトは `writer(&mut self)`/`query(&self)`/`manifest(&self)` のみを
/// 持ち、`FsResultStore::sync`（保留エントリを manifest へ反映する）を含まない
/// （`squid-n-io` 側のトレイト定義であり本クレートからは変更しない）。
/// ジョブ完了直後に `manifest()`/`query()` で書き込み結果を参照できる必要があるため
/// （`result_get` が manifest 存在確認で結果を見つけられないと困る）、
/// `ServerState` は `Box<dyn ResultStore>` ではなく具象型 `FsResultStore` を直接保持し、
/// 書き込み後は明示的に `sync()` を呼ぶ設計とする（`Box<dyn ResultStore>` のままだと
/// `sync()` を呼ぶ手段が無く、ダウンキャストするにはトレイトに `Any` を足す必要が
/// あるが、それは squid-n-io 側の変更になってしまうため避けた）。
pub struct ServerState {
    pub model: Model,
    pub undo: UndoStack,
    pub jobs: JobRegistry,
    pub results: squid_n_io::results::FsResultStore,
}

impl ServerState {
    /// 結果ストアのディレクトリを明示して `ServerState` を構築する。
    pub fn with_fs_store(model: Model, dir: impl Into<PathBuf>) -> std::io::Result<Self> {
        Ok(Self {
            model,
            undo: UndoStack::new(),
            jobs: JobRegistry::new(),
            results: squid_n_io::results::FsResultStore::open(dir)?,
        })
    }
}

/// 結果ストアの既定ディレクトリ。環境変数 `SQUID_N_RESULT_DIR` があれば優先し、
/// 無ければ OS 一時ディレクトリ配下の `squid-n-mcp-results` を使う。
pub fn default_result_dir() -> PathBuf {
    std::env::var_os("SQUID_N_RESULT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("squid-n-mcp-results"))
}

#[cfg(feature = "mcp")]
pub mod server {
    use super::*;
    // rmcp 1.x では `tool_router`/`tool_handler` マクロはクレートルート（rmcp_macros の再エクスポート）にあり、
    // `Parameters` は `handler::server::tool` 内で private import されているだけなので
    // 実体を持つ `handler::server::wrapper` から取る必要がある。
    use rmcp::handler::server::tool::ToolRouter;
    use rmcp::handler::server::wrapper::Parameters;
    use rmcp::model::{CallToolResult, Content, Implementation, ServerInfo};
    use rmcp::transport::stdio;
    use rmcp::{tool, tool_handler, tool_router, ErrorData, ServerHandler, ServiceExt};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    #[derive(Clone)]
    pub struct SquidNServer {
        state: Arc<Mutex<ServerState>>,
        /// rmcp の `#[tool_handler]` が生成するディスパッチ用ルータ。
        /// rmcp 1.7 のマクロ展開はこのフィールドを直接読まない経路を取り得るため
        /// dead_code を明示的に許可する（rmcp の標準パターンに従った保持）。
        #[allow(dead_code)]
        tool_router: ToolRouter<Self>,
    }

    impl SquidNServer {
        pub fn new(state: ServerState) -> Self {
            Self {
                state: Arc::new(Mutex::new(state)),
                tool_router: Self::tool_router(),
            }
        }
    }

    #[tool_router]
    impl SquidNServer {
        #[tool(description = "節点・部材・断面を検索する")]
        pub async fn model_query(
            &self,
            Parameters(args): Parameters<QueryArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            // 実クエリは feature 非依存の query_model に委譲（テスト済み）。
            let items = super::query_model(&st.model, &args.kind, args.filter.as_deref());
            let result = QueryResult { items };
            Ok(CallToolResult::success(vec![Content::json(result)?]))
        }

        #[tool(description = "解析を非同期で実行する")]
        pub async fn analysis_run(
            &self,
            Parameters(args): Parameters<AnalysisRunArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            // 引数の妥当性（dir 等）はジョブ登録前に確認する。失敗が確定している
            // ジョブを登録しないため。
            let params = args
                .to_job_params()
                .map_err(|e| ErrorData::invalid_params(e, None))?;
            let kind = args.kind;

            // ジョブ登録・実行中への遷移・モデルの複製はロック保持中に行い、
            // ロックを解放してから spawn_blocking へ渡す
            // （Mutex ガードをスレッドへ持ち込まないため）。
            let (id, model) = {
                let mut st = self.state.lock().await;
                let id = st.jobs.register(kind);
                st.jobs.update(&id, JobStatus::Running { progress: 0.0 });
                (id, st.model.clone())
            };

            let state = self.state.clone();
            let job_id = id.clone();
            // 解析（CPU バウンド）は spawn_blocking で実行する。その完了待ちと、結果ストアへの
            // 永続化・ジョブ状態の更新は別タスク（tokio::spawn）で行うことで、本ツール呼び出し
            // 自体は job_id を即時返し、応答をブロックしない（非同期ジョブとしての仕様どおり）。
            // 結果ストアへの書き込みとジョブ状態の Done への遷移は、同じロック保持区間内で
            // 行う（result_get が「Done なのに manifest に無い」状態を観測しないため）。
            tokio::spawn(async move {
                let outcome =
                    tokio::task::spawn_blocking(move || super::compute_job(&model, kind, &params))
                        .await;
                match outcome {
                    Ok(Ok(job_outcome)) => {
                        let mut st = state.lock().await;
                        let summary = super::persist_job_outcome(&mut st.results, job_outcome);
                        st.jobs.update(
                            &job_id,
                            JobStatus::Done {
                                result_ref: summary,
                            },
                        );
                    }
                    Ok(Err(e)) => {
                        let mut st = state.lock().await;
                        st.jobs.update(&job_id, JobStatus::Failed { error: e });
                    }
                    // spawn_blocking 内で panic した場合。JoinError を利用者向けメッセージに変換する。
                    Err(join_err) => {
                        let mut st = state.lock().await;
                        st.jobs.update(
                            &job_id,
                            JobStatus::Failed {
                                error: format!("解析タスクが異常終了しました: {join_err}"),
                            },
                        );
                    }
                }
            });

            Ok(CallToolResult::success(vec![Content::json(
                serde_json::json!({ "job_id": id }),
            )?]))
        }

        /// 結果ストアから結果を取得する。`kind` は "NodalDisp"/"MemberForce"/
        /// "Modal"/"TimeHistory" のいずれか。`case`+`kind` の組が manifest に無い
        /// 場合はエラーを返す（analysis_run で該当ジョブを先に実行すること）。
        #[tool(description = "解析結果ストアから結果を取得する")]
        pub async fn result_get(
            &self,
            Parameters(args): Parameters<ResultGetArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let step_range = match &args.step_range {
                None => None,
                Some(v) if v.len() == 2 => Some((v[0], v[1])),
                Some(_) => {
                    return Err(ErrorData::invalid_params(
                        "step_range は [start, end) の2要素で指定してください",
                        None,
                    ));
                }
            };
            let st = self.state.lock().await;
            let result = super::result_get_json(
                &st.results,
                args.case,
                &args.kind,
                args.node_ids.clone(),
                args.member_ids.clone(),
                step_range,
            )
            .map_err(|e| ErrorData::invalid_params(e, None))?;
            Ok(CallToolResult::success(vec![Content::json(result)?]))
        }

        /// ジョブの現在状態を返す。`status` は `Queued`/`Running{progress}`/
        /// `Done{result_ref}`/`Failed{error}` のいずれか。`Done` の場合
        /// `result_ref` に解析結果（変位ベクトルの JSON 文字列）を保持している。
        #[tool(description = "ジョブの状態を取得する")]
        pub async fn analysis_status(
            &self,
            Parameters(args): Parameters<AnalysisStatusArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            let job = st.jobs.get(&args.job_id);
            match job {
                Some(j) => Ok(CallToolResult::success(vec![Content::json(j)?])),
                // rmcp 1.x では CallToolResult::error は content のみを取り、
                // JSON-RPC レベルのエラーコードは Err(ErrorData) 側で表現する。
                None => Err(ErrorData::invalid_params("job not found", None)),
            }
        }
    }

    #[tool_handler]
    impl ServerHandler for SquidNServer {
        fn get_info(&self) -> ServerInfo {
            // rmcp 1.x では ServerInfo(=InitializeResult)・Implementation は #[non_exhaustive] のため
            // 構造体リテラルで組み立てられない。ビルダーメソッド経由で組み立てる。
            ServerInfo::default().with_server_info(Implementation::new("squid-n-mcp", "0.1.0"))
        }
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct QueryArgs {
        pub kind: String,
        pub filter: Option<String>,
    }

    #[derive(Debug, serde::Serialize, schemars::JsonSchema)]
    pub struct QueryResult {
        pub items: Vec<serde_json::Value>,
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct AnalysisRunArgs {
        pub kind: JobKind,
        /// LinearStatic/DesignCheck: 対象荷重ケース ID（未指定なら先頭ケース）。
        pub load_case: Option<u32>,
        /// Eigen: モード数（既定 3）。
        pub n_modes: Option<usize>,
        /// Pushover/TimeHistory: 加力・入力方向 "X"/"Y"（既定 "X"）。
        pub dir: Option<String>,
        /// Pushover: 最大ステップ数（既定 50）。
        pub steps: Option<usize>,
        /// Pushover: 目標変位 [mm]（既定 500）。
        pub max_disp: Option<f64>,
        /// TimeHistory: サンプル波の時間刻み [s]（既定 0.01）。
        pub dt: Option<f64>,
        /// TimeHistory: サンプル波の継続時間 [s]（既定 2.0）。
        pub duration: Option<f64>,
        /// TimeHistory: サンプル波の周期 [s]（既定 0.5）。
        pub period: Option<f64>,
        /// TimeHistory: サンプル波の振幅 [mm/s²]（既定 1000）。
        pub amp: Option<f64>,
    }

    impl AnalysisRunArgs {
        /// 任意パラメータを `super::JobParams`（既定値込み）へ変換する。
        /// `dir` に "X"/"Y" 以外の文字列が指定された場合のみエラーを返す
        /// （ジョブ登録前に検証することで、失敗が確定しているジョブを作らない）。
        fn to_job_params(&self) -> Result<super::JobParams, String> {
            let dir = match self.dir.as_deref() {
                None => super::JobDir::X,
                Some("X") => super::JobDir::X,
                Some("Y") => super::JobDir::Y,
                Some(other) => {
                    return Err(format!("不明な方向: {other}（\"X\" または \"Y\"）"));
                }
            };
            let d = super::JobParams::default();
            Ok(super::JobParams {
                load_case: self.load_case,
                n_modes: self.n_modes.unwrap_or(d.n_modes),
                dir,
                steps: self.steps.unwrap_or(d.steps),
                max_disp: self.max_disp.unwrap_or(d.max_disp),
                dt: self.dt.unwrap_or(d.dt),
                duration: self.duration.unwrap_or(d.duration),
                period: self.period.unwrap_or(d.period),
                amp: self.amp.unwrap_or(d.amp),
            })
        }
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct AnalysisStatusArgs {
        pub job_id: String,
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct ResultGetArgs {
        pub case: u32,
        /// "NodalDisp" | "MemberForce" | "Modal" | "TimeHistory"
        pub kind: String,
        pub node_ids: Option<Vec<u32>>,
        pub member_ids: Option<Vec<u32>>,
        /// \[start, end)。ちょうど2要素で指定する。
        pub step_range: Option<Vec<u64>>,
    }

    pub async fn run_stdio_server(state: ServerState) -> Result<(), Box<dyn std::error::Error>> {
        let service = SquidNServer::new(state).serve(stdio()).await?;
        service.waiting().await?;
        Ok(())
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
        use squid_n_core::model::{
            DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis,
            Material, NodalLoad, Node, Section, Story,
        };
        use std::path::{Path, PathBuf};
        use std::time::Duration;

        /// テスト用の結果ストアディレクトリを用意する（テストごとに固有の名前を渡すこと）。
        /// 前回実行の残骸を消してから使う（実ストア=ファイルシステムを使うため）。
        fn test_store_dir(name: &str) -> PathBuf {
            let dir = std::env::temp_dir().join(format!("squid_n_mcp_test_{name}"));
            let _ = std::fs::remove_dir_all(&dir);
            dir
        }

        /// 実ストア（`FsResultStore`）を使う `ServerState` を組み立てる。
        fn make_state(model: Model, dir: &Path) -> ServerState {
            ServerState::with_fs_store(model, dir).expect("FsResultStore::open が失敗しないこと")
        }

        /// 片持ち梁（node0 固定・node1 自由）+ 荷重ケース1つの、解析が完走できる最小モデル。
        /// LinearStatic/Eigen/TimeHistory/DesignCheck の各ジョブテストで共有する:
        /// - 先端に質量を与えている(Eigen/TimeHistory が固有値解析できるように)。
        /// - 材料名を鋼材(SN400)にし、断面係数(iz)を小さくしている
        ///   (DesignCheck で NG が出ることを確認できるように、わざと過大応力にしている)。
        /// - 荷重は全体座標系 Z 方向のせん断力とし、曲げモーメントが生じるようにしている。
        ///   `ref_vector=[0,0,1]`(=全体Z)の梁は局所 y 軸が全体 Z に一致するため
        ///   （`LocalFrame::from_nodes` 参照）、全体 Z 方向の力が局所 Qy/Mz
        ///   （`compute_design_check_job` が強軸として見る成分）に載る。
        ///   全体 X 方向の軸力だけでは M・Q が実質ゼロで検定に影響しないため使わない。
        fn cantilever_with_load_case() -> Model {
            Model {
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
                        coord: [1000.0, 0.0, 0.0],
                        restraint: Dof6Mask::FREE,
                        mass: Some([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]),
                        story: None,
                    },
                ],
                elements: vec![ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                }],
                sections: vec![Section {
                    id: SectionId(0),
                    name: "beam".into(),
                    area: 100.0,
                    iy: 833.33,
                    iz: 10.0,
                    j: 100.0,
                    depth: 10.0,
                    width: 10.0,
                    as_y: 83.33,
                    as_z: 83.33,
                    panel_thickness: None,
                    thickness: None,
                    shape: None,
                }],
                materials: vec![Material {
                    concrete_class: Default::default(),
                    id: MaterialId(0),
                    name: "SN400".into(),
                    young: 20000.0,
                    poisson: 0.3,
                    density: 0.0,
                    shear: None,
                    fc: None,
                    fy: Some(235.0),
                }],
                load_cases: vec![LoadCase {
                    id: LoadCaseId(1),
                    name: "case1".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                }],
                ..Default::default()
            }
        }

        /// 上と同じモデルから荷重ケースだけを抜いたもの（LinearStatic ジョブが
        /// "no load cases" で失敗する経路を確認するため）。
        fn cantilever_without_load_case() -> Model {
            Model {
                load_cases: vec![],
                ..cantilever_with_load_case()
            }
        }

        /// 1層・鉛直柱モデル（Pushover ジョブ用）。
        /// squid-n-app の `sample::portal_frame`（Beam 要素・SN400B）と同じ材料構成をベースに
        /// 単純な片持ち柱 + Story(地震重量) を組み立てる
        /// （`Analysis::prepare` の線形剛性検証を通す必要があるため、
        /// ねじり剛性を持たない Fiber 要素ではなく Beam 要素を使う。
        /// squid-n-solver 側の `pushover::tests::single_column_model` は Fiber 要素かつ
        /// `Analysis::prepare` を経由しないテストのため、そのままでは
        /// `App::compute_pushover`/本ジョブの「まず prepare で検証する」流儀に合わない）。
        fn pushover_model() -> Model {
            Model {
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
                        story: Some(StoryId(0)),
                    },
                ],
                elements: vec![ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [1.0, 0.0, 0.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                }],
                sections: vec![Section {
                    id: SectionId(0),
                    name: "col".into(),
                    area: 10000.0,
                    iy: 8.333e6,
                    iz: 8.333e6,
                    j: 1.0e6,
                    depth: 100.0,
                    width: 100.0,
                    as_y: 0.0,
                    as_z: 0.0,
                    panel_thickness: None,
                    thickness: None,
                    shape: None,
                }],
                materials: vec![Material {
                    concrete_class: Default::default(),
                    id: MaterialId(0),
                    name: "steel".into(),
                    young: 205000.0,
                    poisson: 0.3,
                    density: 0.0,
                    // squid-n-solver 側の `single_column_model`（Fiber 要素、ねじり無視）は
                    // shear=Some(0.0) だが、ここは Beam 要素（弾性ねじり剛性 GJ を持つ）なので
                    // shear=None にして young/poisson から G を導出させる
                    // （G=0 のままだと頂部のねじり自由度の剛性が 0 になり特異行列になる）。
                    shear: None,
                    fc: None,
                    fy: Some(235.0),
                }],
                stories: vec![Story {
                    id: StoryId(0),
                    name: "1F".into(),
                    elevation: 3000.0,
                    node_ids: vec![NodeId(1)],
                    diaphragms: vec![DiaphragmDef {
                        master: NodeId(1),
                        slaves: vec![],
                        rigid: true,
                    }],
                    seismic_weight: Some(80_000.0),
                }],
                ..Default::default()
            }
        }

        /// `AnalysisRunArgs` を最小限のフィールド指定で組み立てる
        /// （他は全て既定値=None）。
        fn run_args(kind: JobKind) -> AnalysisRunArgs {
            AnalysisRunArgs {
                kind,
                load_case: None,
                n_modes: None,
                dir: None,
                steps: None,
                max_disp: None,
                dt: None,
                duration: None,
                period: None,
                amp: None,
            }
        }

        /// `CallToolResult`（`analysis_run` の戻り値）から `job_id` を取り出す。
        fn extract_job_id(result: &CallToolResult) -> String {
            let text = result.content[0]
                .raw
                .as_text()
                .expect("analysis_run の応答は text content のはず")
                .text
                .clone();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            value["job_id"].as_str().unwrap().to_string()
        }

        /// ジョブが `Done`/`Failed` のいずれかの終端状態に達するまでポーリングする。
        async fn wait_for_terminal(server: &SquidNServer, job_id: &str) -> JobStatus {
            for _ in 0..400 {
                {
                    let st = server.state.lock().await;
                    if let Some(job) = st.jobs.get(job_id) {
                        match &job.status {
                            JobStatus::Done { .. } | JobStatus::Failed { .. } => {
                                return job.status.clone();
                            }
                            _ => {}
                        }
                    }
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            panic!("job did not reach a terminal state in time");
        }

        /// `Done` 状態の `result_ref`（サマリ JSON 文字列）を取り出してパースする。
        fn done_summary(status: &JobStatus) -> serde_json::Value {
            match status {
                JobStatus::Done { result_ref } => {
                    serde_json::from_str(result_ref).expect("summary は JSON のはず")
                }
                other => panic!("expected Done, got {other:?}"),
            }
        }

        #[tokio::test]
        async fn test_analysis_run_completes_for_valid_model() {
            let dir = test_store_dir("linear_static_basic");
            let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
            let result = server
                .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            assert!(
                matches!(status, JobStatus::Done { .. }),
                "expected Done, got {status:?}"
            );
        }

        #[tokio::test]
        async fn test_analysis_run_fails_without_load_case() {
            let dir = test_store_dir("linear_static_no_case");
            let server = SquidNServer::new(make_state(cantilever_without_load_case(), &dir));
            let result = server
                .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            match status {
                JobStatus::Failed { error } => {
                    assert!(error.contains("no load cases"), "unexpected error: {error}");
                }
                other => panic!("expected Failed, got {other:?}"),
            }
        }

        /// LinearStatic ジョブ → Done → manifest に NodalDisp/MemberForce が載る →
        /// result_get(NodalDisp, node_ids 指定) が該当行を返す。
        #[tokio::test]
        async fn test_linear_static_job_persists_and_result_get_filters_nodes() {
            let dir = test_store_dir("linear_static_result_get");
            let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
            let result = server
                .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            let summary = done_summary(&status);
            assert_eq!(summary["store"]["case"], 1);
            let kinds: Vec<String> = summary["store"]["kinds"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_str().unwrap().to_string())
                .collect();
            assert!(kinds.contains(&"NodalDisp".to_string()));
            assert!(kinds.contains(&"MemberForce".to_string()));

            {
                let st = server.state.lock().await;
                let manifest = st.results.manifest();
                assert!(manifest
                    .entries
                    .iter()
                    .any(|e| e.case == 1 && e.kind == squid_n_io::results::ResultKind::NodalDisp));
                assert!(
                    manifest
                        .entries
                        .iter()
                        .any(|e| e.case == 1
                            && e.kind == squid_n_io::results::ResultKind::MemberForce)
                );
            }

            let got = server
                .result_get(Parameters(ResultGetArgs {
                    case: 1,
                    kind: "NodalDisp".to_string(),
                    node_ids: Some(vec![1]),
                    member_ids: None,
                    step_range: None,
                }))
                .await
                .unwrap();
            let text = got.content[0].raw.as_text().unwrap().text.clone();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            let rows = value["rows"].as_array().unwrap();
            assert_eq!(rows.len(), 1, "node_ids=[1] で絞り込んだ行数");
            assert_eq!(rows[0]["node_id"], 1);
            assert_eq!(value["truncated"], false);
        }

        /// Eigen ジョブ → Done（周期がサマリに含まれる）→
        /// result_get(Modal) が n_modes 行返す。
        #[tokio::test]
        async fn test_eigen_job_persists_and_result_get_modal() {
            let dir = test_store_dir("eigen_result_get");
            let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
            let mut args = run_args(JobKind::Eigen);
            args.n_modes = Some(1);
            let result = server.analysis_run(Parameters(args)).await.unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            let summary = done_summary(&status);
            assert_eq!(summary["n_modes"], 1);
            assert!(summary["period"].as_array().unwrap()[0].as_f64().unwrap() > 0.0);
            assert_eq!(summary["store"]["case"], 0);

            let got = server
                .result_get(Parameters(ResultGetArgs {
                    case: 0,
                    kind: "Modal".to_string(),
                    node_ids: None,
                    member_ids: None,
                    step_range: None,
                }))
                .await
                .unwrap();
            let text = got.content[0].raw.as_text().unwrap().text.clone();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            let rows = value["rows"].as_array().unwrap();
            assert_eq!(rows.len(), 1, "n_modes=1 なので1行返るはず");
        }

        /// Pushover ジョブ（stories 付きモデル）→ Done でサマリに qu[kN] が含まれる。
        #[tokio::test]
        async fn test_pushover_job_completes_with_qu_in_summary() {
            let dir = test_store_dir("pushover_basic");
            let server = SquidNServer::new(make_state(pushover_model(), &dir));
            let mut args = run_args(JobKind::Pushover);
            // 既定(steps=50, max_disp=500mm)だと機構形成後に特異行列となり得るため
            // (squid-n-solver 側の同種テストと同じ配慮)、小さめの値にする。
            args.steps = Some(10);
            args.max_disp = Some(30.0);
            let result = server.analysis_run(Parameters(args)).await.unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            let summary = done_summary(&status);
            assert!(summary["qu_kN"].as_f64().unwrap() > 0.0);
            assert!(
                summary.get("store").is_none(),
                "Pushover はストアへ書かない"
            );
        }

        /// DesignCheck ジョブ → Done でサマリに NG 数が含まれる。
        #[tokio::test]
        async fn test_design_check_job_reports_ng_count() {
            let dir = test_store_dir("design_check_basic");
            let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
            let result = server
                .analysis_run(Parameters(run_args(JobKind::DesignCheck)))
                .await
                .unwrap();
            let job_id = extract_job_id(&result);
            let status = wait_for_terminal(&server, &job_id).await;
            let summary = done_summary(&status);
            assert_eq!(summary["case"], 1);
            assert!(summary["n_checks"].as_u64().unwrap() > 0);
            assert!(
                summary["n_ng"].as_u64().unwrap() > 0,
                "断面係数を小さくしてあるので過大応力で NG になるはず: {summary}"
            );
        }

        /// result_get: 存在しない case を指定すると invalid_params エラーになる。
        #[tokio::test]
        async fn test_result_get_missing_case_is_invalid_params() {
            let dir = test_store_dir("result_get_missing");
            let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
            let err = server
                .result_get(Parameters(ResultGetArgs {
                    case: 999,
                    kind: "NodalDisp".to_string(),
                    node_ids: None,
                    member_ids: None,
                    step_range: None,
                }))
                .await
                .expect_err("manifest に無い case は Err のはず");
            assert!(
                err.message.contains("結果がありません"),
                "エラーメッセージに『結果がありません』が含まれるはず: {err:?}"
            );
        }
    }
}

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
}

/// `model.query` の中核ロジック（feature 非依存・テスト可能）。
///
/// `kind` で `node`/`member`(=element)/`section` を選び、各要素を JSON 化して返す。
/// `filter` が与えられたときは、各 JSON を文字列化した中に部分一致するものだけを残す
/// （簡易フィルタ。名前・ID 等での絞り込み用）。MCP ツール `model_query` はこれを呼ぶ。
pub fn query_model(model: &Model, kind: &str, filter: Option<&str>) -> Vec<serde_json::Value> {
    use serde_json::json;
    let items: Vec<serde_json::Value> = match kind {
        "node" | "nodes" => model
            .nodes
            .iter()
            .map(|n| {
                json!({
                    "id": n.id.0,
                    "coord": n.coord,
                    "story": n.story.map(|s| s.0),
                })
            })
            .collect(),
        "member" | "members" | "element" | "elements" => model
            .elements
            .iter()
            .map(|e| {
                json!({
                    "id": e.id.0,
                    "kind": format!("{:?}", e.kind),
                    "nodes": e.nodes.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "section": e.section.map(|s| s.0),
                    "material": e.material.map(|m| m.0),
                })
            })
            .collect(),
        "section" | "sections" => model
            .sections
            .iter()
            .map(|s| {
                json!({
                    "id": s.id.0,
                    "name": s.name,
                    "area": s.area,
                    "iy": s.iy,
                    "iz": s.iz,
                })
            })
            .collect(),
        _ => vec![],
    };
    match filter {
        Some(f) if !f.is_empty() => items
            .into_iter()
            .filter(|v| v.to_string().contains(f))
            .collect(),
        _ => items,
    }
}

/// 解析の実処理（feature 非依存・テスト可能）。`Model` の参照だけを受け取るため、
/// `ServerState` のロックを取らずに（= ロック解放後に）呼び出せる。
/// 現状は先頭の荷重ケースに対する線形静的解析のみ（他ジョブ種別は将来対応）。
///
/// `analysis_run`（MCP ツール）は `state.model.clone()` を取ってロックを落としてから
/// `spawn_blocking` 内でこの関数を呼ぶことで、CPU バウンドな解析中も `ServerState` の
/// ミューテックスを他ツール呼び出しのためにブロックしない。
pub fn analyze_model(model: &Model) -> Result<String, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1「剛域」は標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    if let Some(lc) = model.load_cases.first() {
        let result = analysis
            .linear_static(lc.id)
            .map_err(|e| format!("solve failed: {e}"))?;
        Ok(serde_json::to_string(&result.disp).unwrap_or_default())
    } else {
        Err("no load cases".into())
    }
}

/// `analyze_model` の `ServerState` 経由の薄いラッパ（後方互換用）。
pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    analyze_model(&state.model)
}

// ============================================================================
// 全 JobKind の実処理（feature 非依存・テスト可能）。
//
// `analysis_run`（MCP ツール、mod server）は「ロック保持中にモデルを複製 →
// ロック解放 → spawn_blocking でこの節の compute_* を呼ぶ → 再度ロックして
// 結果ストアへ永続化 + ジョブ状態更新」という流れを取る（P8 の既存方針を踏襲）。
// compute_* はいずれも GUI（squid-n-app）非依存の純関数（&Model か Model の
// クローンだけで完結）とし、squid-n-app の同等ロジック（compute_pushover /
// compute_time_history / sample_wave / run_design_check）と重複する箇所は
// コメントで明記する（squid-n-mcp は squid-n-app に依存しないため複製が必要）。
// ============================================================================

/// `analysis_run` の任意パラメータの解決後の値（`AnalysisRunArgs` から変換する）。
/// 既定値は GUI (`squid_n_app::app::AnalysisSettings`) の既定に合わせる。
/// ただし `duration` は GUI 既定の 10.0 秒だと MCP 経由の応答待ちが長くなるため、
/// 動作確認がしやすい 2.0 秒を既定とする（呼び出し側で明示すれば変更可）。
#[derive(Debug, Clone, Copy)]
pub struct JobParams {
    /// LinearStatic/DesignCheck: 対象荷重ケース ID（未指定なら先頭ケース）。
    pub load_case: Option<u32>,
    /// Eigen: モード数。
    pub n_modes: usize,
    /// Pushover/TimeHistory: 加力・入力方向。
    pub dir: JobDir,
    /// Pushover: 最大ステップ数。
    pub steps: usize,
    /// Pushover: 目標変位 [mm]。
    pub max_disp: f64,
    /// TimeHistory: サンプル波の時間刻み [s]。
    pub dt: f64,
    /// TimeHistory: サンプル波の継続時間 [s]。
    pub duration: f64,
    /// TimeHistory: サンプル波の周期 [s]。
    pub period: f64,
    /// TimeHistory: サンプル波の振幅 [mm/s²]。
    pub amp: f64,
}

impl Default for JobParams {
    fn default() -> Self {
        Self {
            load_case: None,
            n_modes: 3,
            dir: JobDir::X,
            steps: 50,
            max_disp: 500.0,
            dt: 0.01,
            duration: 2.0,
            period: 0.5,
            amp: 1000.0,
        }
    }
}

/// Pushover/TimeHistory の方向（"X"/"Y"）。X+Y 同時入力（GUI の `ThDir::Xy`）は
/// MCP 経由では対応しない（仕様どおり "X"/"Y" のみ）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobDir {
    X,
    Y,
}

/// 各 JobKind の compute 結果。結果ストアへ書くべき生データ（あれば）とサマリ
/// （`JobStatus::Done::result_ref` に格納する JSON）の両方を保持する。
/// ストアへの書き込みは `persist_job_outcome` が担う（`ServerState` のロック内で
/// 呼ぶ必要があるため、compute 側とは分離している）。
pub enum JobOutcome {
    LinearStatic {
        case: u32,
        node_ids: Vec<u32>,
        disp: Vec<[f64; 6]>,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
    Eigen {
        period: Vec<f64>,
        omega2: Vec<f64>,
        participation: Vec<[f64; 3]>,
        effective_mass: Vec<[f64; 3]>,
        summary: serde_json::Value,
    },
    Pushover {
        summary: serde_json::Value,
    },
    TimeHistory {
        summary: serde_json::Value,
    },
    DesignCheck {
        case: u32,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
}

/// `kind` に応じて対応する compute_* 関数へ振り分ける。
pub fn compute_job(model: &Model, kind: JobKind, params: &JobParams) -> Result<JobOutcome, String> {
    match kind {
        JobKind::LinearStatic => compute_linear_static_job(model, params.load_case),
        JobKind::Eigen => compute_eigen_job(model, params.n_modes),
        JobKind::Pushover => {
            compute_pushover_job(model.clone(), params.dir, params.steps, params.max_disp)
        }
        JobKind::TimeHistory => compute_time_history_job(
            model,
            params.dir,
            params.dt,
            params.duration,
            params.period,
            params.amp,
        ),
        JobKind::DesignCheck => compute_design_check_job(model, params.load_case),
    }
}

/// `load_case` 指定があればそれを、無ければ先頭の荷重ケースを返す。
/// 荷重ケースが1つも無いモデルでは "no load cases" を返す
/// （既存の `analyze_model` と同じ文言。P8 のテストが this を確認している）。
fn resolve_load_case(
    model: &Model,
    load_case: Option<u32>,
) -> Result<&squid_n_core::model::LoadCase, String> {
    match load_case {
        Some(id) => model
            .load_cases
            .iter()
            .find(|c| c.id.0 == id)
            .ok_or_else(|| format!("荷重ケース {id} が存在しません")),
        None => model
            .load_cases
            .first()
            .ok_or_else(|| "no load cases".to_string()),
    }
}

/// LinearStatic ジョブの純粋計算部分。
fn compute_linear_static_job(model: &Model, load_case: Option<u32>) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let node_ids: Vec<u32> = model.nodes.iter().map(|n| n.id.0).collect();
    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }
    }
    let max_abs_disp = result
        .disp
        .iter()
        .flat_map(|d| d.iter())
        .fold(0.0_f64, |m, v| m.max(v.abs()));

    let summary = serde_json::json!({
        "kind": "LinearStatic",
        "case": lc_id,
        "n_nodes": node_ids.len(),
        "n_member_force_rows": member_force_rows.len(),
        "max_abs_disp": max_abs_disp,
    });
    Ok(JobOutcome::LinearStatic {
        case: lc_id,
        node_ids,
        disp: result.disp,
        member_force_rows,
        summary,
    })
}

/// Eigen ジョブの純粋計算部分。
fn compute_eigen_job(model: &Model, n_modes: usize) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let modal = analysis
        .eigen(n_modes)
        .map_err(|e| format!("eigen failed: {e}"))?;
    let summary = serde_json::json!({
        "kind": "Eigen",
        "n_modes": modal.period.len(),
        "period": modal.period,
    });
    Ok(JobOutcome::Eigen {
        period: modal.period,
        omega2: modal.omega2,
        participation: modal.participation,
        effective_mass: modal.effective_mass,
        summary,
    })
}

/// Pushover ジョブの純粋計算部分。
/// squid-n-app の `App::compute_pushover`（app.rs）と同じ流れ
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
/// モデルは所有権を取って複製したものを渡す前提
/// （プッシュオーバーは非線形状態を模型に書き戻すため）。
fn compute_pushover_job(
    model: Model,
    dir: JobDir,
    steps: usize,
    max_disp: f64,
) -> Result<JobOutcome, String> {
    let mut work = model;
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut work,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    squid_n_solver::analysis::Analysis::prepare(&work)
        .map_err(|e| format!("解析準備エラー: {e}"))?;
    let dofmap = squid_n_core::dof::DofMap::build(&work);
    let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
    let seismic_dir = match dir {
        JobDir::X => squid_n_solver::analysis::SeismicDir::X,
        JobDir::Y => squid_n_solver::analysis::SeismicDir::Y,
    };
    let result = squid_n_solver::pushover::pushover_analysis(
        &mut work,
        &dofmap,
        &reducer,
        seismic_dir,
        steps,
        max_disp,
        false,
        false,
        0.0,
    )
    .map_err(|e| format!("プッシュオーバー解析エラー: {e}"))?;

    let mechanism = match result.mechanism {
        squid_n_solver::pushover::MechanismType::Overall => "Overall".to_string(),
        squid_n_solver::pushover::MechanismType::StoryCollapse { story } => {
            format!("StoryCollapse(story={})", story.0)
        }
        squid_n_solver::pushover::MechanismType::Partial => "Partial".to_string(),
    };
    // qu は N 単位（squid_n_solver::pushover::PushoverResult）。GUI(app.rs/summary.rs)と
    // 同様に kN 表示にするため /1000.0 する。
    let summary = serde_json::json!({
        "kind": "Pushover",
        "qu_kN": result.qu / 1000.0,
        "mechanism": mechanism,
        "n_steps": result.steps.len(),
    });
    Ok(JobOutcome::Pushover { summary })
}

/// TimeHistory ジョブの純粋計算部分。
/// サンプル波の生成式は squid-n-app の `App::sample_wave`/`build_ground_motion`
/// （app.rs）と同一（squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 減衰は剛性比例減衰 h=0.02（1次固有円振動数を使用）固定
/// （`App::compute_time_history` の `ThDampingModel::StiffnessProportional` 経路と同じ）。
fn compute_time_history_job(
    model: &Model,
    dir: JobDir,
    dt: f64,
    duration: f64,
    period: f64,
    amp: f64,
) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("解析準備エラー: {e}"))?;

    let n = ((duration / dt).ceil() as usize).max(2);
    let omega = 2.0 * std::f64::consts::PI / period.max(1e-6);
    let accel: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            amp * (omega * t).sin() * (-0.3 * t).exp()
        })
        .collect();
    let wave = match dir {
        JobDir::X => squid_n_solver::timehistory::GroundMotion {
            dt,
            accel_x: accel,
            accel_y: None,
        },
        JobDir::Y => {
            let n = accel.len();
            squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: vec![0.0; n],
                accel_y: Some(accel),
            }
        }
    };

    let omega1 = match analysis.eigen(1) {
        Ok(modal) => match modal.omega2.first() {
            Some(&w2) if w2 > 0.0 => w2.sqrt(),
            _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
        },
        Err(e) => return Err(format!("固有値解析エラー: {e}")),
    };
    let damping = squid_n_solver::damping::Damping::StiffnessProportional {
        h: 0.02,
        omega: omega1,
        basis: squid_n_solver::damping::StiffnessKind::Initial,
    };
    let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
    let result = analysis
        .time_history(&wave, newmark, damping)
        .map_err(|e| format!("時刻歴解析エラー: {e}"))?;

    let peak_disp = result
        .history
        .node_disp
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let summary = serde_json::json!({
        "kind": "TimeHistory",
        "peak_disp": peak_disp,
        "record_dir_y": result.history.record_dir_y,
        "n_steps": result.time.len(),
    });
    Ok(JobOutcome::TimeHistory { summary })
}

/// 鋼材判定（Material.name が JIS 鋼種名で始まるか。鉄筋 SD/SR は RC 扱い）。
/// squid-n-app の `is_steel`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
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
/// squid-n-app の `member_kind_of`（app.rs）と同じロジック。
fn member_kind_of(
    elem: &squid_n_core::model::ElementData,
    model: &Model,
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
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 \[mm\]（内法補正なしの簡易値。剛域等は考慮しない）。
/// squid-n-app の `elem_geometric_length`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn elem_geometric_length(elem: &squid_n_core::model::ElementData, model: &Model) -> f64 {
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

/// 危険断面位置（§6.2.3、既定は柱フェイスと中央）を正規化座標 \[0,1\] で算定する。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` 算定と同じ規則
/// （xi_i は \[0.0, 0.5) へ、xi_j は (0.5, 1.0\] へクランプ）で face_i/face_j から
/// 求める。face=0（直交材が無い端）では節点芯（0.0/1.0）と一致する。
/// squid-n-app の `design_positions`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
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

/// DesignCheck ジョブの純粋計算部分。
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// squid-n-app の `App::run_design_check`（app.rs）と同じ判定
/// （材料名先頭文字で鋼/RC を判定し SteelDesign/RcDesign を適用）を行う
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 検定条件（長期/短期）は既定で長期（`LoadTerm::Long`）とする。
fn compute_design_check_job(model: &Model, load_case: Option<u32>) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    // face_i/face_j は危険断面位置（§6.2.3）の算定にも使う。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    let mut n_checks = 0usize;
    let mut n_ng = 0usize;
    let mut max_ratio = 0.0_f64;

    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }

        let Some(elem) = model.elements.iter().find(|e| e.id == *elem_id) else {
            continue;
        };
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let mat = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid));
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };

        // 部材種別・部材長・せん断スパン比代表値（app.rs の run_design_check と同じ規則）。
        let kind = member_kind_of(elem, model);
        let length = {
            let coords: Vec<[f64; 3]> = elem
                .nodes
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .map(|n| n.coord)
                .take(2)
                .collect();
            match (coords.first(), coords.get(1)) {
                (Some(p0), Some(p1)) => {
                    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
                    (dx * dx + dy * dy + dz * dz).sqrt()
                }
                _ => 0.0,
            }
        };
        let shear_span = mf
            .at
            .iter()
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // 端部・中央の強軸曲げ（横座屈 C 係数・たわみ検定用）。
        let m_at = |target: f64| {
            mf.at
                .iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| f[5])
        };
        let end_moments_z = match (m_at(0.0), m_at(1.0)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        };
        // 柱の座屈長さ lk = K・h（app.rs run_design_check と同じ規則）。
        let lk = if kind == squid_n_design_jp::MemberKind::Column {
            squid_n_design_jp::steel::buckling::steel_column_k(model, elem).map(|k| k * length)
        } else {
            None
        };
        // S 造部材の断面検定属性（欠損率・横座屈長さ）。単一ケース・長期検定の
        // ため seismic_qd（地震時 QD 割増）は常に None。
        let steel_attr = model
            .steel_design_attrs
            .iter()
            .find(|a| a.elem == *elem_id)
            .cloned();
        let ctx = squid_n_design_jp::DesignCtx {
            term: squid_n_design_jp::LoadTerm::Long,
            kind,
            length,
            lb: None,
            lk,
            shear_span,
            rc_damage_control: true,
            end_moments_z,
            mid_moment_z: m_at(0.5),
            seismic_qd: None,
            steel_attr,
        };

        // 検定器の選択: 複合断面（SRC/CFT）は形状優先、それ以外は材料名で鋼/RC
        // （app.rs の run_design_check と同じ規則）。
        let checker: Box<dyn squid_n_design_jp::DesignCheck> = match sec.shape {
            Some(squid_n_core::section_shape::SectionShape::SrcRect { .. }) => {
                Box::new(squid_n_design_jp::SrcDesign)
            }
            Some(squid_n_core::section_shape::SectionShape::CftBox { .. })
            | Some(squid_n_core::section_shape::SectionShape::CftPipe { .. }) => {
                Box::new(squid_n_design_jp::CftDesign)
            }
            _ if is_steel(&mat.name) => Box::new(squid_n_design_jp::SteelDesign),
            _ => Box::new(squid_n_design_jp::RcDesign),
        };
        // 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力のみ検定する。
        // 節点芯は剛域が有る場合は検定対象外（app.rs の run_design_check と同じ規則）。
        let geom_len = elem_geometric_length(elem, model);
        let positions = design_positions(elem, geom_len);
        for (pos, forces) in &mf.at {
            if !is_near_design_position(*pos, &positions) {
                continue;
            }
            // [N, Qy, Qz, Mx, My, Mz] -> MemberForcesAt（N は引張正の部材内力）
            let mfa = squid_n_design_jp::MemberForcesAt {
                pos: *pos,
                n: forces[0],
                qy: forces[1],
                qz: forces[2],
                my: forces[4],
                mz: forces[5],
            };
            // BRB 属性が登録された部材はメーカー許容値による BRB 検定に差し替える
            // （app.rs run_design_check と同じ規則）。
            let cr = if let Some(brb) = model.brb_attrs.iter().find(|a| a.elem == *elem_id) {
                squid_n_design_jp::brb::brb_check(brb, mfa.n, length, true)
            } else {
                checker.check(&mfa, sec, mat, &ctx)
            };
            n_checks += 1;
            if !cr.ok {
                n_ng += 1;
            }
            if cr.ratio > max_ratio {
                max_ratio = cr.ratio;
            }
        }
    }

    // 節点単位の検定（RC 柱梁接合部・S パネルゾーン・冷間成形耐力比・耐震壁）。
    let mf_slices: Vec<(
        squid_n_core::ids::ElemId,
        squid_n_design_jp::joint_wiring::ForcesAt,
    )> = result
        .member_forces
        .iter()
        .map(|(id, mf)| (*id, mf.at.as_slice()))
        .collect();
    // PCa 水平接合面の検定（PcaBeamAttr が登録された梁のみ。単一ケース＝長期扱い）。
    for (_, _, cr) in
        squid_n_design_jp::rc::horizontal_joint::collect_pca_checks(model, &mf_slices, true)
    {
        n_checks += 1;
        if !cr.ok {
            n_ng += 1;
        }
        if cr.ratio > max_ratio {
            max_ratio = cr.ratio;
        }
    }
    let joint_checks = squid_n_design_jp::joint_wiring::collect_joint_checks(
        model,
        &mf_slices,
        squid_n_design_jp::LoadTerm::Long,
    );
    let n_joint_checks = joint_checks.len();
    let n_joint_ng = joint_checks.iter().filter(|(_, _, cr)| !cr.ok).count();
    for (_, _, cr) in &joint_checks {
        if cr.ratio > max_ratio {
            max_ratio = cr.ratio;
        }
    }

    let summary = serde_json::json!({
        "kind": "DesignCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        "n_joint_checks": n_joint_checks,
        "n_joint_ng": n_joint_ng,
        "max_ratio": max_ratio,
    });
    Ok(JobOutcome::DesignCheck {
        case: lc_id,
        member_force_rows,
        summary,
    })
}

/// `summary`（JSON オブジェクト）に、結果ストアへ書き込んだ場所を示す
/// `"store": {"case": .., "kinds": [..]}` を追記する。
fn attach_store_info(summary: &mut serde_json::Value, case: u32, kinds: &[&str]) {
    if let serde_json::Value::Object(map) = summary {
        map.insert(
            "store".to_string(),
            serde_json::json!({ "case": case, "kinds": kinds }),
        );
    }
}

/// `JobOutcome` を結果ストアへ永続化し、`JobStatus::Done::result_ref` に格納する
/// サマリ JSON 文字列を返す。`ServerState` のロックを保持したまま
/// （= ジョブ状態更新と同じロック内で）呼び出すこと。
///
/// ## 対応表（JobKind → 書き込む ResultKind）
/// - LinearStatic: NodalDisp（全節点変位）+ MemberForce（評価位置ごとの断面力）。
///   case = 使用した荷重ケース ID。
/// - DesignCheck: MemberForce のみ（検定の元データ）。検定結果自体（OK/NG・検定比）は
///   専用の ResultKind が無いためサマリ JSON にのみ含める。case = 使用した荷重ケース ID。
/// - Eigen: Modal のみ。case は固定で 0 を使う。固有値解析は荷重ケースに依存しない
///   1系統の結果のため、実在する荷重ケース番号と衝突しないダミー値を使う設計とした。
///   manifest のキーは (case, kind) の組であり、Modal は NodalDisp/MemberForce とは
///   別の ResultKind（＝別の名前空間）なので、仮に実際の荷重ケースが `case=0` を
///   使っていても NodalDisp/MemberForce の case=0 エントリとは衝突しない
///   （LoadCaseId(0) を実荷重ケースとしても二重利用してしまう設計は避けている）。
/// - Pushover/TimeHistory: 対応する ResultKind スキーマが無いため
///   （TimeHistory 結果 `ResponseResult` は代表1節点の応答のみを保持し、
///   `ResultKind::TimeHistory` が要求する全節点×全ステップの変位を持たない）
///   ストアへは書き込まず、サマリ JSON のみを返す。
pub fn persist_job_outcome(
    store: &mut squid_n_io::results::FsResultStore,
    outcome: JobOutcome,
) -> String {
    use squid_n_io::results::{member_force_batch, modal_batch, nodal_disp_batch, ResultKind};

    match outcome {
        JobOutcome::LinearStatic {
            case,
            node_ids,
            disp,
            member_force_rows,
            mut summary,
        } => {
            let mut kinds: Vec<&str> = Vec::new();
            {
                let mut w = store.writer(case, ResultKind::NodalDisp);
                if let Ok(batch) = nodal_disp_batch(&node_ids, &disp) {
                    w.write_rows(&batch);
                }
                w.finish();
            }
            kinds.push("NodalDisp");
            if !member_force_rows.is_empty() {
                let mut w = store.writer(case, ResultKind::MemberForce);
                if let Ok(batch) = member_force_batch(&member_force_rows) {
                    w.write_rows(&batch);
                }
                w.finish();
                kinds.push("MemberForce");
            }
            let _ = store.sync();
            attach_store_info(&mut summary, case, &kinds);
            summary.to_string()
        }
        JobOutcome::DesignCheck {
            case,
            member_force_rows,
            mut summary,
        } => {
            let mut kinds: Vec<&str> = Vec::new();
            if !member_force_rows.is_empty() {
                let mut w = store.writer(case, ResultKind::MemberForce);
                if let Ok(batch) = member_force_batch(&member_force_rows) {
                    w.write_rows(&batch);
                }
                w.finish();
                kinds.push("MemberForce");
            }
            let _ = store.sync();
            if !kinds.is_empty() {
                attach_store_info(&mut summary, case, &kinds);
            }
            summary.to_string()
        }
        JobOutcome::Eigen {
            period,
            omega2,
            participation,
            effective_mass,
            mut summary,
        } => {
            // LoadCaseId(0) の二重使用を避けるための設計は上記ドキュメントコメント参照。
            let case = 0u32;
            {
                let mut w = store.writer(case, ResultKind::Modal);
                if let Ok(batch) = modal_batch(&period, &omega2, &participation, &effective_mass) {
                    w.write_rows(&batch);
                }
                w.finish();
            }
            let _ = store.sync();
            attach_store_info(&mut summary, case, &["Modal"]);
            summary.to_string()
        }
        JobOutcome::Pushover { summary } | JobOutcome::TimeHistory { summary } => {
            summary.to_string()
        }
    }
}

/// 結果 1 回あたりの `result_get` 応答に含める行数の上限。
/// MCP 応答（JSON-RPC のテキストコンテンツ）が肥大化して呼び出し側（LLM）の
/// コンテキストを圧迫するのを防ぐための安全弁。超過分は "truncated": true で通知する。
const RESULT_GET_ROW_LIMIT: usize = 10_000;

/// 結果種別名（"NodalDisp" 等）を `ResultKind` へ変換する。
fn parse_result_kind(s: &str) -> Result<squid_n_io::results::ResultKind, String> {
    use squid_n_io::results::ResultKind;
    match s {
        "NodalDisp" => Ok(ResultKind::NodalDisp),
        "MemberForce" => Ok(ResultKind::MemberForce),
        "Modal" => Ok(ResultKind::Modal),
        "TimeHistory" => Ok(ResultKind::TimeHistory),
        other => Err(format!(
            "不明な結果種別: {other}（NodalDisp/MemberForce/Modal/TimeHistory のいずれか）"
        )),
    }
}

/// `RecordBatch` を JSON 行配列へ変換する（arrow::json は使わず、既知の列型
/// （UInt32/UInt64/Float64。P8 の4スキーマはすべてこのいずれか）だけを手動で
/// `serde_json::Value` に変換する）。`row_limit` を超える行は切り詰め、
/// 2つ目の戻り値で打ち切ったかどうかを返す。
fn batch_to_json_rows(
    batch: &arrow::record_batch::RecordBatch,
    row_limit: usize,
) -> (Vec<serde_json::Value>, bool) {
    use arrow::array::{Float64Array, UInt32Array, UInt64Array};
    use arrow::datatypes::DataType;

    let schema = batch.schema();
    let total = batch.num_rows();
    let n = total.min(row_limit);
    let mut rows = Vec::with_capacity(n);
    for r in 0..n {
        let mut obj = serde_json::Map::new();
        for (c, field) in schema.fields().iter().enumerate() {
            let col = batch.column(c);
            let value = match field.data_type() {
                DataType::UInt32 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<UInt32Array>()
                    .expect("UInt32 列のはず")
                    .value(r)),
                DataType::UInt64 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<UInt64Array>()
                    .expect("UInt64 列のはず")
                    .value(r)),
                DataType::Float64 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<Float64Array>()
                    .expect("Float64 列のはず")
                    .value(r)),
                // P8 の4スキーマ（NodalDisp/MemberForce/Modal/TimeHistory）に
                // 現れない型。将来スキーマが増えたら対応を追加する。
                other => {
                    let _ = other;
                    serde_json::Value::Null
                }
            };
            obj.insert(field.name().clone(), value);
        }
        rows.push(serde_json::Value::Object(obj));
    }
    (rows, total > row_limit)
}

/// `result_get` ツールの中核ロジック（feature 非依存・テスト可能）。
/// manifest に該当エントリが無ければエラー文字列を返す
/// （呼び出し側は MCP の `invalid_params` へマップする）。
pub fn result_get_json(
    store: &dyn squid_n_io::results::ResultStore,
    case: squid_n_io::results::CaseId,
    kind_str: &str,
    node_ids: Option<Vec<u32>>,
    member_ids: Option<Vec<u32>>,
    step_range: Option<(u64, u64)>,
) -> Result<serde_json::Value, String> {
    let kind = parse_result_kind(kind_str)?;
    let exists = store
        .manifest()
        .entries
        .iter()
        .any(|e| e.case == case && e.kind == kind);
    if !exists {
        return Err(format!(
            "結果がありません（case={case}, kind={kind_str}）。analysis_run で解析を実行してから呼び出してください。"
        ));
    }

    let node_filter = node_ids.map(|ids| ids.into_iter().map(squid_n_core::ids::NodeId).collect());
    let member_filter =
        member_ids.map(|ids| ids.into_iter().map(squid_n_core::ids::ElemId).collect());
    let query = squid_n_io::results::ResultQuery {
        case,
        kind,
        node_filter,
        member_filter,
        step_range,
    };
    let result = store.query(&query);
    let (rows, truncated) = batch_to_json_rows(&result.batch, RESULT_GET_ROW_LIMIT);
    Ok(serde_json::json!({
        "case": case,
        "kind": kind_str,
        "rows": rows,
        "truncated": truncated,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, LocalAxis, Node, Section};

    fn sample_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: squid_n_core::dof::Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: squid_n_core::dof::Dof6Mask::FREE,
                    mass: None,
                    story: Some(squid_n_core::ids::StoryId(0)),
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "H-400".to_string(),
                area: 100.0,
                iy: 1000.0,
                iz: 2000.0,
                j: 50.0,
                depth: 400.0,
                width: 200.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [
                    squid_n_core::model::EndCondition::Fixed,
                    squid_n_core::model::EndCondition::Fixed,
                ],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_query_model_nodes() {
        let m = sample_model();
        let items = query_model(&m, "node", None);
        assert_eq!(items.len(), 2);
        assert_eq!(items[0]["id"], 0);
        assert_eq!(items[1]["story"], 0);
    }

    #[test]
    fn test_query_model_elements_and_sections() {
        let m = sample_model();
        assert_eq!(query_model(&m, "member", None).len(), 1);
        let secs = query_model(&m, "section", None);
        assert_eq!(secs.len(), 1);
        assert_eq!(secs[0]["name"], "H-400");
    }

    #[test]
    fn test_query_model_filter() {
        let m = sample_model();
        // 名前で絞り込み（断面名 H-400 を含むものだけ）。
        assert_eq!(query_model(&m, "section", Some("H-400")).len(), 1);
        assert_eq!(query_model(&m, "section", Some("RC")).len(), 0);
    }

    #[test]
    fn test_query_model_unknown_kind() {
        let m = sample_model();
        assert!(query_model(&m, "bogus", None).is_empty());
    }

    #[test]
    fn test_job_registry_lifecycle() {
        let mut reg = JobRegistry::new();
        let id = reg.register(JobKind::LinearStatic);
        assert!(matches!(reg.get(&id).unwrap().status, JobStatus::Queued));
        reg.update(&id, JobStatus::Running { progress: 0.5 });
        assert!(matches!(
            reg.get(&id).unwrap().status,
            JobStatus::Running { progress } if (progress - 0.5).abs() < 1e-6
        ));
        reg.update(
            &id,
            JobStatus::Done {
                result_ref: "r1".into(),
            },
        );
        assert!(matches!(
            &reg.get(&id).unwrap().status,
            JobStatus::Done { result_ref } if result_ref == "r1"
        ));
        // 異なる ID は別ジョブ。
        let id2 = reg.register(JobKind::Eigen);
        assert_ne!(id, id2);
        assert!(reg.get("nonexistent").is_none());
    }
}
