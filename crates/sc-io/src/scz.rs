use crate::manifest::Manifest;
use crate::migrate::migrate;
use sc_core::model::Model;
use sha2::{Digest, Sha256};
use std::io::{Read, Write};
use std::path::Path;

pub const CURRENT_SCHEMA_VERSION: u32 = 4;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("zip: {0}")]
    Zip(String),
    #[error("decode: {0}")]
    Decode(String),
    #[error("hash mismatch for entry {0}")]
    HashMismatch(String),
    #[error("unsupported schema version: {0}")]
    UnsupportedVersion(u32),
}

fn sha256_of(data: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(data);
    format!("{:x}", hasher.finalize())
}

pub fn save_scz(path: &Path, model: &Model) -> Result<(), IoError> {
    let tmp_path = path.with_extension("scz.tmp");

    let model_bytes = rmp_serde::to_vec(model).map_err(|e| IoError::Decode(e.to_string()))?;
    let settings_bytes = serde_json::to_vec_pretty(&serde_json::json!({
        "code": "JIS B 0001",
        "created_at": "",
    }))
    .map_err(|e| IoError::Decode(e.to_string()))?;

    let model_hash = sha256_of(&model_bytes);
    let settings_hash = sha256_of(&settings_bytes);

    let manifest = Manifest {
        schema_version: CURRENT_SCHEMA_VERSION,
        units: "internal: N-mm-s".to_string(),
        created_by: "sc-io 0.0.1".to_string(),
        entries: vec![
            crate::manifest::EntryHash {
                name: "model.msgpack".to_string(),
                sha256: model_hash,
            },
            crate::manifest::EntryHash {
                name: "settings.json".to_string(),
                sha256: settings_hash,
            },
        ],
    };

    let manifest_bytes =
        serde_json::to_vec_pretty(&manifest).map_err(|e| IoError::Decode(e.to_string()))?;

    {
        let f = std::fs::File::create(&tmp_path)?;
        let mut zip = zip::ZipWriter::new(f);
        let opts = zip::write::FileOptions::<()>::default()
            .compression_method(zip::CompressionMethod::Deflated);

        zip.start_file("manifest.json", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&manifest_bytes)?;

        zip.start_file("model.msgpack", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&model_bytes)?;

        zip.start_file("settings.json", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&settings_bytes)?;

        zip.finish().map_err(|e| IoError::Zip(e.to_string()))?;
    }

    std::fs::rename(&tmp_path, path)?;
    Ok(())
}

pub fn load_scz(path: &Path) -> Result<Model, IoError> {
    let f = std::fs::File::open(path)?;
    let mut archive = zip::ZipArchive::new(f).map_err(|e| IoError::Zip(e.to_string()))?;

    let mut manifest_bytes = Vec::new();
    archive
        .by_name("manifest.json")
        .map_err(|e| IoError::Zip(e.to_string()))?
        .read_to_end(&mut manifest_bytes)?;
    let manifest: Manifest =
        serde_json::from_slice(&manifest_bytes).map_err(|e| IoError::Decode(e.to_string()))?;

    // 未知版（サポート範囲外）のみここで弾く。既知の旧版（1..=現行）は
    // model.msgpack 読込後に migrate() で最新版へ移行する。
    // 「現行版以外を一律弾く」と migrate の旧版分岐が到達不能になり、
    // 後方互換（§5.1）が機能しなくなるため、ここでは範囲チェックに留める。
    if !(1..=CURRENT_SCHEMA_VERSION).contains(&manifest.schema_version) {
        return Err(IoError::UnsupportedVersion(manifest.schema_version));
    }

    for entry in &manifest.entries {
        let mut data = Vec::new();
        archive
            .by_name(&entry.name)
            .map_err(|e| IoError::Zip(format!("missing entry {}: {}", entry.name, e)))?
            .read_to_end(&mut data)?;
        let actual_hash = sha256_of(&data);
        if actual_hash != entry.sha256 {
            return Err(IoError::HashMismatch(entry.name.clone()));
        }
    }

    let mut model_data = Vec::new();
    archive
        .by_name("model.msgpack")
        .map_err(|e| IoError::Zip(e.to_string()))?
        .read_to_end(&mut model_data)?;

    let model_data = migrate(manifest.schema_version, model_data)?;

    let model: Model =
        rmp_serde::from_slice(&model_data).map_err(|e| IoError::Decode(e.to_string()))?;

    Ok(model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::dof::Dof6Mask;
    use sc_core::ids::*;
    use sc_core::model::*;

    fn make_3node_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [2000.0, 0.0, 0.0],
                    restraint: Dof6Mask::PINNED,
                    mass: None,
                    story: None,
                },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_roundtrip() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p.scz");
        save_scz(&path, &model).unwrap();
        let back = load_scz(&path).unwrap();
        assert_eq!(model.nodes.len(), back.nodes.len());
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_hash_mismatch() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p_hash.scz");
        save_scz(&path, &model).unwrap();

        // corrupt model.msgpack by writing bad data into a new zip
        let bad_manifest = Manifest {
            schema_version: CURRENT_SCHEMA_VERSION,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![crate::manifest::EntryHash {
                name: "model.msgpack".to_string(),
                sha256: "badhash".to_string(),
            }],
        };
        let bad_bytes = serde_json::to_vec(&bad_manifest).unwrap();
        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&bad_bytes).unwrap();
            zip.start_file("model.msgpack", opts).unwrap();
            zip.write_all(&[0u8; 4]).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, &path).unwrap();

        let result = load_scz(&path);
        assert!(result.is_err());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_unsupported_version() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p_ver.scz");
        save_scz(&path, &model).unwrap();

        let bad_manifest = Manifest {
            schema_version: 999,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![],
        };
        let bad_bytes = serde_json::to_vec(&bad_manifest).unwrap();
        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&bad_bytes).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, &path).unwrap();

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::UnsupportedVersion(999))));
        let _ = std::fs::remove_file(&path);
    }

    /// 旧版（v2）の .scz が migrate 経由で読めること（後方互換 §5.1）。
    /// 現行フォーマットの model.msgpack のまま manifest の版だけ 2 に書換えても、
    /// migrate(2,..) が round-trip して読み込めることを確認する
    /// （= migrate の旧版分岐が到達可能であることの回帰テスト）。
    #[test]
    fn test_migrate_old_version_loads() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p_migrate_v2.scz");
        // まず通常保存し、その model.msgpack / settings.json を取り出す。
        save_scz(&path, &model).unwrap();
        let (model_bytes, settings_bytes) = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut mb = Vec::new();
            ar.by_name("model.msgpack")
                .unwrap()
                .read_to_end(&mut mb)
                .unwrap();
            let mut sb = Vec::new();
            ar.by_name("settings.json")
                .unwrap()
                .read_to_end(&mut sb)
                .unwrap();
            (mb, sb)
        };

        // 版を 2 と偽った manifest（entries のハッシュは実バイトに一致させる）。
        let manifest = Manifest {
            schema_version: 2,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![
                crate::manifest::EntryHash {
                    name: "model.msgpack".to_string(),
                    sha256: sha256_of(&model_bytes),
                },
                crate::manifest::EntryHash {
                    name: "settings.json".to_string(),
                    sha256: sha256_of(&settings_bytes),
                },
            ],
        };
        let manifest_bytes = serde_json::to_vec_pretty(&manifest).unwrap();

        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&manifest_bytes).unwrap();
            zip.start_file("model.msgpack", opts).unwrap();
            zip.write_all(&model_bytes).unwrap();
            zip.start_file("settings.json", opts).unwrap();
            zip.write_all(&settings_bytes).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, &path).unwrap();

        let back = load_scz(&path).expect("v2 は migrate 経由で読めるべき");
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }
}
