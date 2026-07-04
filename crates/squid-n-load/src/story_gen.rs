//! 階(Story)の自動生成。
//!
//! 節点の標高(Z)をクラスタリングして階を推定し、各階に剛床(ダイアフラム)と
//! 地震重量を設定する。地震静的解析(Ai分布)・プッシュオーバー・偏心率計算の
//! 前提データを 1 操作で用意するための機能。
//!
//! 重量は「自重(ρ·A·L·g) + 指定荷重ケースの鉛直下向き荷重」を節点に配分し、
//! 階ごとに合計する簡易法(節点支配)による。

use squid_n_core::ids::{LoadCaseId, NodeId, StoryId};
use squid_n_core::model::{Constraint, DiaphragmDef, ElementKind, MemberLoadKind, Model, Story};

/// 重力加速度 [mm/s²]（内部単位系 N-mm-s、質量 ton）。
const GRAVITY_MM_S2: f64 = 9800.0;

/// 同一階とみなす標高差 [mm]。
const LEVEL_TOL_MM: f64 = 1.0;

/// 生成結果。[`Model`] へ適用するのは呼び出し側（EditCommand 経由）。
#[derive(Clone, Debug, PartialEq)]
pub struct StoryGenResult {
    /// 下から順の階（基部レベルは含まない）。
    pub stories: Vec<Story>,
    /// 各節点の所属階（`model.nodes` と同順。基部レベルは None）。
    pub node_story: Vec<Option<StoryId>>,
    /// 各階の剛床拘束（`Reducer` が読む `model.constraints` 用）。
    pub constraints: Vec<Constraint>,
}

/// 節点 Z 座標から階を自動生成する。
///
/// - 最下レベルは基部(支点レベル)とみなし階に含めない
/// - 各階の剛床マスターは水平重心に最も近い節点
/// - `gravity_lc` を指定すると、そのケースの鉛直下向き荷重を地震重量に算入する
///   （自重は材料密度から常に算入）
pub fn generate_stories(
    model: &Model,
    gravity_lc: Option<LoadCaseId>,
) -> Result<StoryGenResult, String> {
    if model.nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 1. Z レベルのクラスタリング ---
    let mut zs: Vec<f64> = model.nodes.iter().map(|n| n.coord[2]).collect();
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut levels: Vec<f64> = Vec::new();
    for z in zs {
        match levels.last() {
            Some(&last) if (z - last).abs() <= LEVEL_TOL_MM => {}
            _ => levels.push(z),
        }
    }
    if levels.len() < 2 {
        return Err(
            "節点の標高(Z)が 1 レベルしかありません。階を生成するには 2 レベル以上必要です。"
                .into(),
        );
    }

    let level_of = |z: f64| -> usize {
        levels
            .iter()
            .position(|&l| (z - l).abs() <= LEVEL_TOL_MM)
            .unwrap_or(0)
    };

    // --- 2. 節点の重量配分 ---
    let mut node_weight = vec![0.0f64; model.nodes.len()];

    // 自重: ρ·A·L·g を両端に半分ずつ
    for elem in &model.elements {
        if !matches!(elem.kind, ElementKind::Beam) || elem.nodes.len() < 2 {
            continue;
        }
        let (Some(sec_id), Some(mat_id)) = (elem.section, elem.material) else {
            continue;
        };
        let (Some(sec), Some(mat)) = (
            model.sections.get(sec_id.index()),
            model.materials.get(mat_id.index()),
        ) else {
            continue;
        };
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
        let len =
            ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2)).sqrt();
        let w = mat.density * sec.area * len * GRAVITY_MM_S2;
        node_weight[ni] += w / 2.0;
        node_weight[nj] += w / 2.0;
    }

    // 指定荷重ケースの鉛直下向き成分
    if let Some(lc_id) = gravity_lc {
        if let Some(lc) = model.load_cases.iter().find(|c| c.id == lc_id) {
            for nl in &lc.nodal {
                if nl.values[2] < 0.0 {
                    node_weight[nl.node.index()] += -nl.values[2];
                }
            }
            for ml in &lc.member {
                let Some(elem) = model
                    .elements
                    .iter()
                    .find(|e| e.id == ml.elem)
                    .filter(|e| e.nodes.len() >= 2)
                else {
                    continue;
                };
                // 全体座標系の作用方向の鉛直下向き成分
                let dz = ml.dir[2];
                if dz >= 0.0 {
                    continue;
                }
                let total = match ml.kind {
                    MemberLoadKind::Point { p, .. } => p,
                    MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                };
                let w = total * (-dz);
                node_weight[elem.nodes[0].index()] += w / 2.0;
                node_weight[elem.nodes[1].index()] += w / 2.0;
            }
        }
    }

    // --- 3. 階の構築（レベル 1 以上、下から順） ---
    let mut stories = Vec::new();
    let mut node_story = vec![None; model.nodes.len()];
    let mut constraints = Vec::new();

    for (si, &elev) in levels.iter().enumerate().skip(1) {
        let story_id = StoryId((si - 1) as u32);
        let node_ids: Vec<NodeId> = model
            .nodes
            .iter()
            .filter(|n| level_of(n.coord[2]) == si)
            .map(|n| n.id)
            .collect();
        if node_ids.is_empty() {
            continue;
        }

        // 剛床マスター: 水平重心に最も近い節点
        let cx = node_ids
            .iter()
            .map(|n| model.nodes[n.index()].coord[0])
            .sum::<f64>()
            / node_ids.len() as f64;
        let cy = node_ids
            .iter()
            .map(|n| model.nodes[n.index()].coord[1])
            .sum::<f64>()
            / node_ids.len() as f64;
        let master = *node_ids
            .iter()
            .min_by(|a, b| {
                let da = (model.nodes[a.index()].coord[0] - cx).powi(2)
                    + (model.nodes[a.index()].coord[1] - cy).powi(2);
                let db = (model.nodes[b.index()].coord[0] - cx).powi(2)
                    + (model.nodes[b.index()].coord[1] - cy).powi(2);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap();
        let slaves: Vec<NodeId> = node_ids.iter().copied().filter(|n| *n != master).collect();

        let weight: f64 = node_ids.iter().map(|n| node_weight[n.index()]).sum();

        for n in &node_ids {
            node_story[n.index()] = Some(story_id);
        }
        if !slaves.is_empty() {
            constraints.push(Constraint::RigidDiaphragm {
                story: story_id,
                master,
                slaves: slaves.clone(),
            });
        }
        stories.push(Story {
            id: story_id,
            name: format!("{}F", si),
            elevation: elev,
            node_ids,
            diaphragms: vec![DiaphragmDef {
                master,
                slaves,
                rigid: true,
            }],
            seismic_weight: Some(weight),
        });
    }

    if stories.is_empty() {
        return Err("階を構成する節点が見つかりませんでした。".into());
    }

    Ok(StoryGenResult {
        stories,
        node_story,
        constraints,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, MemberLoad,
        NodalLoad, Node, Section,
    };

    /// 2 層 × 1 スパンの平面ラーメン（各レベル 2 節点）。
    fn two_story_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
            [0.0, 0.0, 7000.0],
            [6000.0, 0.0, 7000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
        });
        model.materials.push(Material {
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        // 柱4 + 梁2
        let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
        for (i, (a, b)) in conn.iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            });
        }
        model.load_cases.push(LoadCase {
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![NodalLoad {
                node: NodeId(4),
                values: [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
            }],
            member: vec![MemberLoad {
                elem: ElemId(4),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 6000.0,
                    w1: 10.0,
                    w2: 10.0,
                },
            }],
        });
        model
    }

    #[test]
    fn test_generate_two_stories() {
        let model = two_story_model();
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        assert_eq!(gen.stories.len(), 2);
        assert_eq!(gen.stories[0].elevation, 3500.0);
        assert_eq!(gen.stories[1].elevation, 7000.0);
        // 各階 2 節点 → マスター1 + スレーブ1
        assert_eq!(gen.stories[0].node_ids.len(), 2);
        assert_eq!(gen.stories[0].diaphragms[0].slaves.len(), 1);
        assert_eq!(gen.constraints.len(), 2);
        // 基部節点は無所属
        assert_eq!(gen.node_story[0], None);
        assert_eq!(gen.node_story[2], Some(StoryId(0)));
        assert_eq!(gen.node_story[4], Some(StoryId(1)));
        // 重量: 1F = 梁分布荷重 10 N/mm × 6000 = 60 kN + 自重、2F = 節点荷重 50 kN + 自重
        let w1 = gen.stories[0].seismic_weight.unwrap();
        let w2 = gen.stories[1].seismic_weight.unwrap();
        assert!(w1 > 60000.0, "w1={}", w1);
        assert!(w2 > 50000.0, "w2={}", w2);
    }

    #[test]
    fn test_generate_single_level_is_error() {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        assert!(generate_stories(&model, None).is_err());
    }
}
