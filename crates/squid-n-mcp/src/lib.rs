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
    /// 終局検定（靭性保証型耐震設計指針）: RC 矩形部材の塑性理論式による終局
    /// せん断・付着・軸余裕度を算定する。
    UltimateCheck,
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
pub mod server;

mod job;
mod persist;
mod query;

pub use job::*;
pub use persist::*;
pub use query::*;

#[cfg(test)]
mod tests;
