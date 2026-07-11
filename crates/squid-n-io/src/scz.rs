use crate::manifest::Manifest;
use crate::migrate::migrate;
use sha2::{Digest, Sha256};
use squid_n_core::model::Model;
use std::io::{Read, Write};
use std::path::Path;

// 未リリースのため後方互換なし。リリース前のスキーマ変更は版を上げずにこのまま 1 とする。
pub const CURRENT_SCHEMA_VERSION: u32 = 1;

/// manifest への記載が必須な zip エントリ。ここに無い名前はハッシュ検証されないまま
/// 読み込まれてしまうため、読込時に存在を強制する。
const REQUIRED_ENTRIES: [&str; 2] = ["model.msgpack", "settings.json"];

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
    #[error("manifest missing required entry {0}")]
    MissingEntry(String),
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
        created_by: "squid-n-io 0.0.1".to_string(),
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

        // rename 前に内容をディスクへ永続化する。fsync を挟まないと rename が
        // 原子的でも電源断で新ファイルが空・破損になり得る。
        let f = zip.finish().map_err(|e| IoError::Zip(e.to_string()))?;
        f.sync_all()?;
    }

    std::fs::rename(&tmp_path, path)?;
    sync_parent_dir(path)?;
    Ok(())
}

/// rename というディレクトリエントリ変更自体を永続化する（Unix のみ。
/// Windows はディレクトリを fsync できないため no-op）。
#[cfg(unix)]
fn sync_parent_dir(path: &Path) -> std::io::Result<()> {
    let parent = match path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p,
        _ => Path::new("."),
    };
    std::fs::File::open(parent)?.sync_all()
}

#[cfg(not(unix))]
fn sync_parent_dir(_path: &Path) -> std::io::Result<()> {
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

    // 未リリースのため後方互換なし。現行版以外は弾く（migrate は将来の版上げ用の骨子）。
    if manifest.schema_version != CURRENT_SCHEMA_VERSION {
        return Err(IoError::UnsupportedVersion(manifest.schema_version));
    }

    // 必須エントリが manifest に列挙されていることを強制する。これが無いと、
    // manifest から model.msgpack を落としたファイルがハッシュ未検証のまま読めてしまう。
    for required in REQUIRED_ENTRIES {
        if !manifest.entries.iter().any(|e| e.name == required) {
            return Err(IoError::MissingEntry(required.to_string()));
        }
    }

    let mut model_data = None;
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
        if entry.name == "model.msgpack" {
            model_data = Some(data);
        }
    }

    // REQUIRED_ENTRIES チェック済みなので必ず Some だが、防御的に扱う。
    let model_data = model_data.ok_or_else(|| IoError::MissingEntry("model.msgpack".into()))?;

    let model_data = migrate(manifest.schema_version, model_data)?;

    let model: Model =
        rmp_serde::from_slice(&model_data).map_err(|e| IoError::Decode(e.to_string()))?;

    Ok(model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::*;
    use squid_n_core::model::*;

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
        let settings_bytes = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut sb = Vec::new();
            ar.by_name("settings.json")
                .unwrap()
                .read_to_end(&mut sb)
                .unwrap();
            sb
        };

        // model.msgpack を改竄相当のバイト列に差し替え、manifest のハッシュと食い違わせる。
        // settings.json は正しいハッシュにして、必須エントリチェックではなく
        // ハッシュ検証そのものでエラーになることを確認する。
        let bad_manifest = Manifest {
            schema_version: CURRENT_SCHEMA_VERSION,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![
                crate::manifest::EntryHash {
                    name: "model.msgpack".to_string(),
                    sha256: "badhash".to_string(),
                },
                crate::manifest::EntryHash {
                    name: "settings.json".to_string(),
                    sha256: sha256_of(&settings_bytes),
                },
            ],
        };
        write_zip_with_manifest(&path, &bad_manifest, &[0u8; 4], &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::HashMismatch(ref name)) if name == "model.msgpack"));
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

    /// 未リリースのため後方互換なし: 現行版以外（例: 旧版 2 を名乗るファイル）は
    /// UnsupportedVersion で拒否されること。
    #[test]
    fn test_old_version_rejected() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p_old_ver.scz");
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
        write_zip_with_manifest(&path, &manifest, &model_bytes, &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::UnsupportedVersion(2))));
        let _ = std::fs::remove_file(&path);
    }

    /// manifest.entries から model.msgpack を落としたファイルが、ハッシュ未検証のまま
    /// 読めてしまわないこと（MissingEntry で拒否される）。
    #[test]
    fn test_manifest_missing_required_entry_rejected() {
        let model = make_3node_model();
        let dir = std::env::temp_dir();
        let path = dir.join("p_missing_entry.scz");
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

        // model.msgpack のエントリだけを manifest から落とす（zip 内には実体を残す）。
        let manifest = Manifest {
            schema_version: CURRENT_SCHEMA_VERSION,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![crate::manifest::EntryHash {
                name: "settings.json".to_string(),
                sha256: sha256_of(&settings_bytes),
            }],
        };
        write_zip_with_manifest(&path, &manifest, &model_bytes, &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::MissingEntry(ref name)) if name == "model.msgpack"));
        let _ = std::fs::remove_file(&path);
    }

    /// テスト用: 指定 manifest と実バイトで .scz を書き直す。
    fn write_zip_with_manifest(
        path: &Path,
        manifest: &Manifest,
        model_bytes: &[u8],
        settings_bytes: &[u8],
    ) {
        let manifest_bytes = serde_json::to_vec_pretty(manifest).unwrap();
        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&manifest_bytes).unwrap();
            zip.start_file("model.msgpack", opts).unwrap();
            zip.write_all(model_bytes).unwrap();
            zip.start_file("settings.json", opts).unwrap();
            zip.write_all(settings_bytes).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, path).unwrap();
    }

    /// UI設計 §4.2: Section は SectionShape の派生。`to_section` で生成した断面を
    /// 持つモデルを保存→読込しても `shape` が失われず、Some のまま完全一致することを確認する。
    #[test]
    fn test_roundtrip_preserves_section_shape() {
        use squid_n_core::section_shape::SectionShape;

        let shape = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "H-400x200x9x12".to_string());
        assert!(section.shape.is_some());

        let mut model = make_3node_model();
        model.sections.push(section.clone());

        let dir = std::env::temp_dir();
        let path = dir.join("p_shape_roundtrip.scz");
        save_scz(&path, &model).unwrap();
        let back = load_scz(&path).unwrap();

        assert_eq!(back.sections.len(), 1);
        assert_eq!(back.sections[0].shape, Some(shape));
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }
}
