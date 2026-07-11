//! チェックポイント／再開（設計書 §12.4）。
//! 長時間の非線形／時刻歴の再開・巻き戻しのため、解析状態をバイナリ保存する。

use sha2::{Digest, Sha256};
use std::io;
use std::path::Path;

/// P5 §6 の StateSnapshot を直列化したバイト列（全 ElemState・全材料 committed）。
/// 線形時刻歴では空配列でよい。
#[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
pub struct StateBlob {
    pub element_states: Vec<Vec<u8>>,
}

/// チェックポイント内容（設計書 §12.4）。
#[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
pub struct Checkpoint {
    pub schema_version: u32,
    pub model_hash: String,
    pub step: u64,
    pub time: f64,
    pub disp: Vec<f64>,
    pub vel: Vec<f64>,
    pub accel: Vec<f64>,
    pub state: StateBlob,
}

const CHECKPOINT_DIR: &str = "checkpoint";
const CHECKPOINT_FILE: &str = "checkpoint.bin";
const CHECKPOINT_TMP: &str = "checkpoint.tmp";

/// チェックポイントを高速バイナリ形式（bincode）で保存する。
/// 原子的書込のため、一時ファイルに書いてからリネームする。
pub fn save_checkpoint(dir: &Path, cp: &Checkpoint) -> io::Result<()> {
    let cp_dir = dir.join(CHECKPOINT_DIR);
    std::fs::create_dir_all(&cp_dir)?;

    let tmp_path = cp_dir.join(CHECKPOINT_TMP);
    let final_path = cp_dir.join(CHECKPOINT_FILE);

    let encoded = bincode::serialize(cp).map_err(io::Error::other)?;
    std::fs::write(&tmp_path, &encoded)?;
    std::fs::rename(&tmp_path, &final_path)?;

    Ok(())
}

/// チェックポイントを高速バイナリ形式から読み込む。
pub fn load_checkpoint(dir: &Path) -> io::Result<Checkpoint> {
    let path = dir.join(CHECKPOINT_DIR).join(CHECKPOINT_FILE);
    let bytes = std::fs::read(&path)?;
    bincode::deserialize(&bytes).map_err(io::Error::other)
}

/// チェックポイントの model_hash と期待値が一致するか検証する。
pub fn verify_model_hash(cp: &Checkpoint, expected_hash: &str) -> Result<(), String> {
    if cp.model_hash != expected_hash {
        return Err(format!(
            "model hash mismatch: checkpoint={} expected={}",
            cp.model_hash, expected_hash
        ));
    }
    Ok(())
}

/// モデルの安定なハッシュ（SHA-256）を計算する。
/// JSON 文字列化してからハッシュ化することで、フィールド順序が安定する。
pub fn compute_model_hash(model: &squid_n_core::model::Model) -> String {
    let json = serde_json::to_string(model).expect("Model serialization must not fail");
    let mut hasher = Sha256::new();
    hasher.update(json.as_bytes());
    format!("{:x}", hasher.finalize())
}

/// 非線形解析の状態からチェックポイントを作成・保存する。
/// behaviors の serialize_checkpoint を呼んで要素状態を StateBlob に格納。
#[allow(clippy::too_many_arguments)]
pub fn save_nonlinear_checkpoint(
    dir: &Path,
    model: &squid_n_core::model::Model,
    step: u64,
    time: f64,
    disp: &[f64],
    vel: &[f64],
    accel: &[f64],
    behaviors: &[Box<dyn squid_n_element::behavior::ElementBehavior>],
) -> io::Result<()> {
    let model_hash = compute_model_hash(model);
    let element_states: Vec<Vec<u8>> = behaviors.iter().map(|b| b.serialize_checkpoint()).collect();
    let cp = Checkpoint {
        schema_version: 1,
        model_hash,
        step,
        time,
        disp: disp.to_vec(),
        vel: vel.to_vec(),
        accel: accel.to_vec(),
        state: StateBlob { element_states },
    };
    save_checkpoint(dir, &cp)
}

/// チェックポイントから非線形解析状態を復元する。
/// behaviors の deserialize_checkpoint を呼んで要素状態を復元。
pub fn load_nonlinear_checkpoint(
    dir: &Path,
    model: &squid_n_core::model::Model,
    behaviors: &mut [Box<dyn squid_n_element::behavior::ElementBehavior>],
) -> io::Result<Checkpoint> {
    let cp = load_checkpoint(dir)?;
    let expected_hash = compute_model_hash(model);
    verify_model_hash(&cp, &expected_hash)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    for (b, state_bytes) in behaviors.iter_mut().zip(&cp.state.element_states) {
        b.deserialize_checkpoint(state_bytes);
    }
    Ok(cp)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_checkpoint(model_hash: &str, step: u64) -> Checkpoint {
        Checkpoint {
            schema_version: 1,
            model_hash: model_hash.to_string(),
            step,
            time: step as f64 * 0.01,
            disp: vec![0.0, 0.0, 0.0],
            vel: vec![0.0, 0.0, 0.0],
            accel: vec![0.0, 0.0, 0.0],
            state: StateBlob {
                element_states: vec![],
            },
        }
    }

    #[test]
    fn test_checkpoint_save_load_roundtrip() {
        let dir = std::env::temp_dir().join("cp_roundtrip_test");
        let _ = std::fs::remove_dir_all(&dir);
        let cp = make_checkpoint("abc123", 42);

        save_checkpoint(&dir, &cp).unwrap();
        let loaded = load_checkpoint(&dir).unwrap();

        assert_eq!(loaded, cp);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_model_hash_mismatch_rejected() {
        let cp = make_checkpoint("abc123", 0);
        let result = verify_model_hash(&cp, "xyz789");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("model hash mismatch"));
    }

    #[test]
    fn test_model_hash_same_model_consistent() {
        let model1 = squid_n_core::model::Model::default();
        let model2 = squid_n_core::model::Model::default();
        let h1 = compute_model_hash(&model1);
        let h2 = compute_model_hash(&model2);
        assert_eq!(h1, h2);
    }

    #[test]
    fn test_load_nonexistent_returns_error() {
        let dir = std::env::temp_dir().join("cp_nonexistent_test");
        let _ = std::fs::remove_dir_all(&dir);
        // dir は存在するが checkpoint/ は存在しない → NotFound
        std::fs::create_dir_all(&dir).unwrap();
        let result = load_checkpoint(&dir);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::NotFound);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// 非線形チェックポイントのラウンドトリップテスト（P6 §5 §6）。
    /// FiberBeam 1要素の状態を保存→復元し、直列化バイト列が一致することを確認。
    #[test]
    fn test_nonlinear_checkpoint_roundtrip() {
        use smallvec::smallvec;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
            Section,
        };
        use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};
        use squid_n_element::factory::build_nonlinear_behavior;

        let k = 1000.0_f64;
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
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask(0b111110),
                    mass: Some([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec![NodeId(0), NodeId(1)],
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
                name: "spring".into(),
                area: 1.0,
                iy: 1.0,
                iz: 1.0,
                j: 1.0,
                depth: 1.0,
                width: 1.0,
                as_y: 1e12,
                as_z: 1e12,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".into(),
                young: k * 1000.0 / 1.0,
                poisson: 0.0,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        };

        // 要素ビヘイビアを構築し、状態変化を加える
        let (mut behavior, _) = build_nonlinear_behavior(&model.elements[0], &model);
        let ctx = Ctx { model: &model };
        let du = LocalVec {
            data: smallvec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        behavior.update_state(&du, false, &ctx);
        behavior.commit_state();

        let dir = std::env::temp_dir().join("cp_nonlinear_roundtrip_test");
        let _ = std::fs::remove_dir_all(&dir);

        let behaviors: Vec<Box<dyn ElementBehavior>> = vec![behavior];
        let saved_bytes: Vec<Vec<u8>> =
            behaviors.iter().map(|b| b.serialize_checkpoint()).collect();

        save_nonlinear_checkpoint(&dir, &model, 42, 0.42, &[1.0], &[0.1], &[0.0], &behaviors)
            .unwrap();

        // 新しいビヘイビアを初期状態で作成し、チェックポイントから復元
        let (new_behavior, _) = build_nonlinear_behavior(&model.elements[0], &model);
        let mut new_behaviors: Vec<Box<dyn ElementBehavior>> = vec![new_behavior];

        let cp = load_nonlinear_checkpoint(&dir, &model, &mut new_behaviors).unwrap();

        let restored_bytes: Vec<Vec<u8>> = new_behaviors
            .iter()
            .map(|b| b.serialize_checkpoint())
            .collect();
        assert_eq!(restored_bytes, saved_bytes);
        assert_eq!(cp.step, 42);
        assert!((cp.time - 0.42).abs() < 1e-12);

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ハッシュ不一致時に load_nonlinear_checkpoint がエラーを返すことの確認。
    #[test]
    fn test_nonlinear_checkpoint_hash_mismatch() {
        use smallvec::smallvec;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
            Section,
        };
        use squid_n_element::behavior::ElementBehavior;
        use squid_n_element::factory::build_nonlinear_behavior;

        let make_model = |k: f64| -> Model {
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
                        restraint: Dof6Mask(0b111110),
                        mass: Some([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                        story: None,
                    },
                ],
                elements: vec![ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Fiber,
                    nodes: smallvec![NodeId(0), NodeId(1)],
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
                    name: "spring".into(),
                    area: 1.0,
                    iy: 1.0,
                    iz: 1.0,
                    j: 1.0,
                    depth: 1.0,
                    width: 1.0,
                    as_y: 1e12,
                    as_z: 1e12,
                    panel_thickness: None,
                    thickness: None,
                    shape: None,
                }],
                materials: vec![Material {
                    concrete_class: Default::default(),
                    id: MaterialId(0),
                    name: "mat".into(),
                    young: k * 1000.0 / 1.0,
                    poisson: 0.0,
                    density: 0.0,
                    shear: None,
                    fc: None,
                    fy: None,
                }],
                ..Default::default()
            }
        };

        let model_a = make_model(1000.0);
        let model_b = make_model(2000.0); // 異なるヤング率 → ハッシュ不一致

        let (behavior, _) = build_nonlinear_behavior(&model_a.elements[0], &model_a);
        let dir = std::env::temp_dir().join("cp_hash_mismatch_test");
        let _ = std::fs::remove_dir_all(&dir);

        let behaviors: Vec<Box<dyn ElementBehavior>> = vec![behavior];
        save_nonlinear_checkpoint(&dir, &model_a, 0, 0.0, &[0.0], &[0.0], &[0.0], &behaviors)
            .unwrap();

        let (new_behavior, _) = build_nonlinear_behavior(&model_b.elements[0], &model_b);
        let mut new_behaviors: Vec<Box<dyn ElementBehavior>> = vec![new_behavior];

        let result = load_nonlinear_checkpoint(&dir, &model_b, &mut new_behaviors);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::InvalidData);

        let _ = std::fs::remove_dir_all(&dir);
    }
}
