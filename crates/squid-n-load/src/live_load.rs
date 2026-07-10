//! 令85条2項: 支える床の数に応じた柱の積載荷重低減。
//!
//! 柱の軸力算定に用いる積載荷重は、その柱が支える床の数が多いほど、各層の
//! 積載荷重が同時に最大となる確率が下がるという考え方から、表の低減率を
//! 乗じてよい（建築基準法施行令85条2項）。本モジュールは低減率の算定
//! （[`column_live_load_reduction`]）と、柱要素が支える床の数の算定
//! （[`floors_supported_by_column`]）の API を提供する。低減率を実際の
//! 柱軸力算定へ適用する処理は別タスク（本モジュールは API 提供までがスコープ）。
//!
//! `Model.load_cfg.live_load_reduction`（既定 false = 低減を考慮しない、
//! RESP-D と同じ既定）が有効な場合にのみ、呼び出し側でこの低減率を適用する
//! ことを想定する。

use squid_n_core::ids::StoryId;
use squid_n_core::model::{ElementData, Model};

/// 支える床の数に応じた柱の積載荷重低減率（令85条2項）。
///
/// | 支える床の数 | 低減率 |
/// |---|---|
/// | 1 以下 | 1.00 |
/// | 2 | 0.95 |
/// | 3 | 0.90 |
/// | 4 | 0.85 |
/// | 5 | 0.80 |
/// | 6 | 0.75 |
/// | 7 | 0.70 |
/// | 8 | 0.65 |
/// | 9 以上 | 0.60 |
pub fn column_live_load_reduction(n_floors_supported: usize) -> f64 {
    match n_floors_supported {
        0 | 1 => 1.0,
        2 => 0.95,
        3 => 0.90,
        4 => 0.85,
        5 => 0.80,
        6 => 0.75,
        7 => 0.70,
        8 => 0.65,
        _ => 0.60,
    }
}

/// 柱要素（`elem`）が支える床の数。
///
/// 柱の上端節点の所属階（`node_story`）から、モデル中で最も高い階（＝最上階、
/// `StoryId` の値が最大の階。`generate_stories_multi` は下から順に昇順の
/// `StoryId` を割り当てるため、数値の大小が高さの順序と一致する）までの
/// 階数として算定する。上端節点がどの階にも属さない場合（基部節点など）は 0。
pub fn floors_supported_by_column(
    model: &Model,
    elem: &ElementData,
    node_story: &[Option<StoryId>],
) -> usize {
    if elem.nodes.len() < 2 {
        return 0;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    if ni >= model.nodes.len() || nj >= model.nodes.len() {
        return 0;
    }
    let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
    // 上端節点 = Z 座標が大きい方。
    let top_idx = if ci[2] >= cj[2] { ni } else { nj };

    let Some(Some(top_story)) = node_story.get(top_idx).copied() else {
        return 0;
    };
    let Some(max_story) = node_story.iter().filter_map(|s| *s).max() else {
        return 0;
    };
    if max_story.0 < top_story.0 {
        return 0;
    }
    (max_story.0 - top_story.0) as usize + 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::story_gen::generate_stories;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone,
        Section,
    };

    #[test]
    fn test_column_live_load_reduction_table() {
        assert_eq!(column_live_load_reduction(0), 1.0);
        assert_eq!(column_live_load_reduction(1), 1.0);
        assert_eq!(column_live_load_reduction(2), 0.95);
        assert_eq!(column_live_load_reduction(3), 0.90);
        assert_eq!(column_live_load_reduction(4), 0.85);
        assert_eq!(column_live_load_reduction(5), 0.80);
        assert_eq!(column_live_load_reduction(6), 0.75);
        assert_eq!(column_live_load_reduction(7), 0.70);
        assert_eq!(column_live_load_reduction(8), 0.65);
        assert_eq!(column_live_load_reduction(9), 0.60);
        assert_eq!(column_live_load_reduction(100), 0.60);
    }

    /// 3層(4レベル)の1本柱を、複数の柱要素(各階1本)に分けたモデル。
    /// 最下階の柱は3層分、最上階の柱は1層分の床を支える。
    fn three_story_column_model() -> Model {
        let mut model = Model::default();
        let zs = [0.0, 3000.0, 6000.0, 9000.0];
        for (i, z) in zs.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [0.0, 0.0, *z],
                restraint: if i == 0 {
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
            name: "Col".into(),
            area: 90000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        // 柱を3要素(各階1本)に分割: 0-1, 1-2, 2-3
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3)].iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
            });
        }
        model
    }

    #[test]
    fn test_floors_supported_by_column_3story() {
        let model = three_story_column_model();
        let gen = generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories.len(), 3);

        // 最下階の柱(節点0-1、上端は1F=story0): 3層分(1F,2F,3F)を支える。
        let bottom = &model.elements[0];
        assert_eq!(
            floors_supported_by_column(&model, bottom, &gen.node_story),
            3
        );
        // 中間階の柱(節点1-2、上端は2F=story1): 2層分(2F,3F)を支える。
        let mid = &model.elements[1];
        assert_eq!(floors_supported_by_column(&model, mid, &gen.node_story), 2);
        // 最上階の柱(節点2-3、上端は3F=story2): 1層分(3F)のみを支える。
        let top = &model.elements[2];
        assert_eq!(floors_supported_by_column(&model, top, &gen.node_story), 1);

        // 低減率も対応して確認。
        assert_eq!(
            column_live_load_reduction(floors_supported_by_column(&model, bottom, &gen.node_story)),
            0.90
        );
        assert_eq!(
            column_live_load_reduction(floors_supported_by_column(&model, top, &gen.node_story)),
            1.0
        );
    }

    #[test]
    fn test_floors_supported_by_column_unassigned_node_is_zero() {
        // 基部節点(所属階なし)が上端になる異常なケースは 0 を返す(安全側フォールバック)。
        let model = three_story_column_model();
        let node_story: Vec<Option<StoryId>> = vec![None; model.nodes.len()];
        let elem = &model.elements[0];
        assert_eq!(floors_supported_by_column(&model, elem, &node_story), 0);
    }
}
