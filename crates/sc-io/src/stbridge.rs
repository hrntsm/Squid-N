use sc_core::model::Model;

#[derive(Debug, thiserror::Error)]
pub enum StbError {
    #[error("xml parse: {0}")]
    Parse(String),
    #[error("unsupported version: {0}")]
    Version(String),
    #[error("unmappable element: {0}")]
    Unmappable(String),
}

pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    let _ = xml;
    Err(StbError::Version(
        "ST-Bridge 2.0 import not yet implemented".into(),
    ))
}

pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    let _ = model;
    Err(StbError::Version(
        "ST-Bridge 2.0 export not yet implemented".into(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    // 現状の ST-Bridge は未実装で、import/export とも Version エラーを返す。
    // 実装着手時にこのテストを「意味的往復」の検証へ置き換える（P8 DoD §8.3）。
    #[test]
    fn test_import_unimplemented() {
        let r = import_stbridge("<StbModel/>");
        assert!(matches!(r, Err(StbError::Version(_))), "現状は未実装エラー");
    }

    #[test]
    fn test_export_unimplemented() {
        let r = export_stbridge(&Model::default());
        assert!(matches!(r, Err(StbError::Version(_))), "現状は未実装エラー");
    }
}
