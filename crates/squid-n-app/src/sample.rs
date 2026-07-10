//! サンプルモデル生成（導線の自己検証と新規ユーザーの出発点）。
//! GUI 依存なし（テストからも呼べる）。

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, MemberLoad,
    MemberLoadKind, Model, NodalLoad,
};
use squid_n_section::shape::SectionShape;

/// 鋼構造の門型ラーメン（スパン 6m × 高さ 3.5m、柱脚固定）を生成する。
///
/// - 柱: H-300x300x10x15、梁: H-400x200x8x13（いずれも SN400B）
/// - LC0「長期」: 梁全長に等分布荷重 w = 10 N/mm（鉛直下向き）
/// - LC1「地震X」: 梁レベル両節点に水平 20 kN
pub fn portal_frame() -> Model {
    let mut model = Model::default();

    // 節点: 柱脚 2 + 柱頭 2
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(squid_n_core::model::Node {
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

    // 断面: 柱 H-300x300、梁 H-400x200
    let col_shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let beam_shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱 H-300x300x10x15".into()));
    model
        .sections
        .push(beam_shape.to_section(SectionId(1), "梁 H-400x200x8x13".into()));

    // 材料: SN400B
    model.materials.push(Material {
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    });

    // 部材: 柱 2 本 + 梁 1 本
    let members = [
        (0u32, 1u32, 2u32, 0u32), // id, i, j, section
        (1, 1, 3, 0),
        (2, 2, 3, 1),
    ];
    for (id, i, j, sec) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                // 柱は +X、梁は +Z を参照ベクトルにする
                ref_vector: if sec == 0 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 1.0]
                },
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
        });
    }

    // 荷重ケース
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "長期".into(),
        nodal: Vec::new(),
        member: vec![MemberLoad {
            elem: ElemId(2),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 6000.0,
                w1: 10.0,
                w2: 10.0,
            },
        }],
    });
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(1),
        name: "地震X".into(),
        nodal: vec![
            NodalLoad {
                node: NodeId(2),
                values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
            NodalLoad {
                node: NodeId(3),
                values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ],
        member: Vec::new(),
    });

    model
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_portal_frame_is_valid() {
        let model = portal_frame();
        assert!(model.validate().is_ok());
        assert_eq!(model.nodes.len(), 4);
        assert_eq!(model.elements.len(), 3);
        assert_eq!(model.load_cases.len(), 2);
    }

    #[test]
    fn test_portal_frame_solves() {
        let model = portal_frame();
        let analysis = squid_n_solver::analysis::Analysis::prepare(&model).unwrap();
        // 長期（等分布）: 梁中央がたわむ → 柱頭に鉛直変位が生じる
        let r0 = analysis
            .linear_static(squid_n_core::ids::LoadCaseId(0))
            .unwrap();
        assert!(r0.member_forces.len() == 3);
        // 地震X: 柱頭が +X に流れる
        let r1 = analysis
            .linear_static(squid_n_core::ids::LoadCaseId(1))
            .unwrap();
        assert!(
            r1.disp[2][0] > 0.1,
            "柱頭の水平変位が小さすぎる: {}",
            r1.disp[2][0]
        );
    }
}
