use sc_core::model::Model;
use sc_edit::UndoStack;
use std::collections::HashMap;

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

pub struct ServerState {
    pub model: Model,
    pub undo: UndoStack,
    pub jobs: JobRegistry,
    pub results: Box<dyn sc_io::results::ResultStore>,
}

#[cfg(feature = "mcp")]
pub mod server {
    use super::*;
    use rmcp::handler::server::router::tool::ToolRouter as _;
    use rmcp::handler::server::tool::{tool_router, Parameters, ToolRouter};
    use rmcp::model::{CallToolResult, Content, ServerInfo};
    use rmcp::transport::stdio;
    use rmcp::{tool, tool_handler, ErrorData, ServerHandler, ServiceExt};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    #[derive(Clone)]
    pub struct StructCalcServer {
        state: Arc<Mutex<ServerState>>,
        tool_router: ToolRouter<Self>,
    }

    impl StructCalcServer {
        pub fn new(state: ServerState) -> Self {
            Self {
                state: Arc::new(Mutex::new(state)),
                tool_router: Self::tool_router(),
            }
        }
    }

    #[tool_router]
    impl StructCalcServer {
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
            let mut st = self.state.lock().await;
            let id = st.jobs.register(args.kind);
            Ok(CallToolResult::success(vec![Content::json(
                serde_json::json!({ "job_id": id }),
            )?]))
        }

        #[tool(description = "ジョブの状態を取得する")]
        pub async fn analysis_status(
            &self,
            Parameters(args): Parameters<AnalysisStatusArgs>,
        ) -> Result<CallToolResult, ErrorData> {
            let st = self.state.lock().await;
            let job = st.jobs.get(&args.job_id);
            match job {
                Some(j) => Ok(CallToolResult::success(vec![Content::json(j)?])),
                None => Ok(CallToolResult::error(
                    rmcp::model::ErrorCode::InvalidParams,
                    "job not found",
                )),
            }
        }
    }

    #[tool_handler]
    impl ServerHandler for StructCalcServer {
        fn get_info(&self) -> ServerInfo {
            ServerInfo {
                name: "sc-mcp".into(),
                version: "0.1.0".into(),
                ..Default::default()
            }
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
    }

    #[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
    pub struct AnalysisStatusArgs {
        pub job_id: String,
    }

    pub async fn run_stdio_server(state: ServerState) -> Result<(), Box<dyn std::error::Error>> {
        let service = StructCalcServer::new(state).serve(stdio()).await?;
        service.waiting().await?;
        Ok(())
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
pub fn query_model(
    model: &Model,
    kind: &str,
    filter: Option<&str>,
) -> Vec<serde_json::Value> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use sc_core::model::{ElementData, ElementKind, LocalAxis, Node, Section};

    fn sample_model() -> Model {
        let mut m = Model::default();
        m.nodes = vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: sc_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: sc_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(sc_core::ids::StoryId(0)),
            },
        ];
        m.sections = vec![Section {
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
        }];
        m.elements = vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [
                sc_core::model::EndCondition::Fixed,
                sc_core::model::EndCondition::Fixed,
            ],
            force_regime: sc_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
        }];
        m
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

pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    let analysis = sc_solver::analysis::Analysis::prepare(&state.model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    if let Some(lc) = state.model.load_cases.first() {
        let result = analysis
            .linear_static(lc.id)
            .map_err(|e| format!("solve failed: {e}"))?;
        Ok(serde_json::to_string(&result.disp).unwrap_or_default())
    } else {
        Err("no load cases".into())
    }
}
