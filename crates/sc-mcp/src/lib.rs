use sc_core::model::Model;

pub struct ServerState {
    pub model: Model,
    pub job_counter: u64,
}

#[allow(dead_code)]
pub struct JobRegistry {
    jobs: Vec<JobInfo>,
}

pub struct JobInfo {
    pub id: u64,
    pub status: JobStatus,
}

pub enum JobStatus {
    Pending,
    Running,
    Completed,
    Failed(String),
}

#[cfg(feature = "mcp")]
pub mod server {
    use super::*;
    use std::sync::{Arc, Mutex};

    pub async fn run_stdio_server(
        state: Arc<Mutex<ServerState>>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let _ = state;
        Ok(())
    }
}

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
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
