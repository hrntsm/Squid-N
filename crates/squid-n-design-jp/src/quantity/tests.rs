//! 数量積算のモデル走査テスト。

use smallvec::SmallVec;

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, SlabId};
use squid_n_core::model::{
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
    Model, Node, RigidZone, Section, Slab, SlabKind,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

use super::*;

fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, y, z],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    }
}

fn line_elem(id: u32, n0: u32, n1: u32, sec: u32, mat: u32) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(n0));
            v.push(NodeId(n1));
            v
        },
        section: Some(SectionId(sec)),
        material: Some(MaterialId(mat)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    }
}

fn rc_rebar() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 0,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 200.0,
            legs: 2,
            grade: None,
        },
    }
}

fn rc_girder_section(id: u32) -> Section {
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 800.0,
        rebar: rc_rebar(),
    };
    shape.to_section(SectionId(id), format!("G{id}"))
}

fn rc_column_section(id: u32) -> Section {
    let rebar = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 13.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let shape = SectionShape::RcRect {
        b: 700.0,
        d: 700.0,
        rebar,
    };
    shape.to_section(SectionId(id), format!("C{id}"))
}

fn rc_material(id: u32) -> Material {
    Material {
        id: MaterialId(id),
        name: "Fc24".to_string(),
        young: 22_700.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
        concrete_class: Default::default(),
    }
}

fn steel_material(id: u32) -> Material {
    Material {
        id: MaterialId(id),
        name: "SN400B".to_string(),
        young: 205_000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
        concrete_class: Default::default(),
    }
}

/// 2 柱＋上部大梁＋基礎梁の 1 スパン RC ラーメン。
fn rc_portal_model() -> Model {
    let nodes = vec![
        node(0, 0.0, 0.0, 0.0),
        node(1, 6_000.0, 0.0, 0.0),
        node(2, 0.0, 0.0, 3_500.0),
        node(3, 6_000.0, 0.0, 3_500.0),
    ];
    let mut girder = line_elem(2, 2, 3, 0, 0);
    girder.rigid_zone.face_i = 350.0;
    girder.rigid_zone.face_j = 350.0;
    let mut fg = line_elem(3, 0, 1, 0, 0);
    fg.rigid_zone.face_i = 350.0;
    fg.rigid_zone.face_j = 350.0;
    let elements = vec![
        line_elem(0, 0, 2, 1, 0), // 柱
        line_elem(1, 1, 3, 1, 0), // 柱
        girder,                   // 大梁
        fg,                       // 基礎梁
    ];
    Model {
        nodes,
        elements,
        sections: vec![rc_girder_section(0), rc_column_section(1)],
        materials: vec![rc_material(0)],
        ..Default::default()
    }
}

#[test]
fn test_rc_portal_categories_and_concrete() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    assert_eq!(q.items.len(), 4);

    let cols: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::Column)
        .collect();
    let girders: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::Girder)
        .collect();
    let fgs: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::FoundationGirder)
        .collect();
    assert_eq!(cols.len(), 2);
    assert_eq!(girders.len(), 1);
    assert_eq!(fgs.len(), 1);

    // 柱: 0.7×0.7×3.5 = 1.715 m³、型枠 2×(0.7+0.7)×3.5 = 9.8 m²
    assert!((cols[0].concrete_m3 - 1.715).abs() < 1e-9);
    assert!((cols[0].formwork_m2 - 9.8).abs() < 1e-9);

    // 大梁: 内法 L=6000−350×2=5300 → 0.4×0.8×5.3 = 1.696 m³
    assert!((girders[0].concrete_m3 - 1.696).abs() < 1e-9);
    // 型枠（スラブなし）: 側面 0.8×2×5.3 + 底面 0.4×5.3 = 10.6 m²
    assert!((girders[0].formwork_m2 - (0.8 * 2.0 + 0.4) * 5.3).abs() < 1e-9);

    // 基礎梁の型枠: 側面 1 面＋底面 = (0.8+0.4)×5.3 = 6.36 m²
    assert!((fgs[0].formwork_m2 - 1.2 * 5.3).abs() < 1e-9);
}

#[test]
fn test_rc_girder_main_bars_and_stirrups() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();

    // 主筋: 両端とも連続梁なし → 両外端（タイプ7a）
    // 1 本長さ = 5300 + 35×25×2 = 7050mm、8 本 → 56.4 m
    let main = girder
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::MainBar)
        .unwrap();
    assert!((main.total_length_m - 8.0 * 7.05).abs() < 1e-9);
    // D25 = 3.98 kg/m → 56.4 m = 224.472 kg
    assert!((main.weight_t - 8.0 * 7.05 * 3.98 / 1_000.0).abs() < 1e-9);

    // スターラップ: 一組 2×400+2×800 = 2400、本数 5300/200 = 26.5
    let st = girder
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::Stirrup)
        .unwrap();
    assert!((st.total_length_m - 2.4 * 26.5).abs() < 1e-9);

    // 継手: 8 本 × (0.5 + 0.5×floor(5.3/5)) = 8 × 1.0
    assert!((girder.rebar_joints - 8.0).abs() < 1e-9);
}

#[test]
fn test_rc_column_rebar() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let col = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Column)
        .unwrap();

    // 主筋: X 6 本 + Y 4 本、各 H=3.5m
    let total_main_len: f64 = col
        .rebar
        .iter()
        .filter(|r| r.usage == RebarUsage::MainBar)
        .map(|r| r.total_length_m)
        .sum();
    assert!((total_main_len - 10.0 * 3.5).abs() < 1e-9);

    // フープ: 一組 2×700+2×700 = 2800、本数 3500/100 = 35 → 98 m
    let hoop = col
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::Hoop)
        .unwrap();
    assert!((hoop.total_length_m - 2.8 * 35.0).abs() < 1e-9);

    // 継手: 10 本 × 1 個所（階高 < 7m）
    assert!((col.rebar_joints - 10.0).abs() < 1e-9);
}

#[test]
fn test_girder_formwork_slab_deduction() {
    // 大梁の両側にスラブが取り付く場合、側面せいからスラブ厚を控除する。
    let mut model = rc_portal_model();
    model.slab_thickness = 150.0;
    // 大梁（節点 2-3）の両側（y=+5000 と y=−5000）に床パネルを配置する。
    model.nodes.push(node(4, 0.0, 5_000.0, 3_500.0));
    model.nodes.push(node(5, 6_000.0, 5_000.0, 3_500.0));
    model.nodes.push(node(6, 0.0, -5_000.0, 3_500.0));
    model.nodes.push(node(7, 6_000.0, -5_000.0, 3_500.0));
    for (sid, (a, b)) in [(0u32, (5u32, 4u32)), (1u32, (6u32, 7u32))] {
        model.slabs.push(Slab {
            id: SlabId(sid),
            boundary: vec![NodeId(2), NodeId(3), NodeId(a), NodeId(b)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            kind: SlabKind::Interior,
            one_way: None,
            edge_supported: None,
        });
    }

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();
    // 側面 (0.8−0.15)×2×5.3 + 底面 0.4×5.3
    let expected = ((0.8 - 0.15) * 2.0 + 0.4) * 5.3;
    assert!((girder.formwork_m2 - expected).abs() < 1e-9);
}

#[test]
fn test_joist_by_ratio() {
    // 柱に取り付かない水平梁は小梁として鉄筋比で概算する。
    let mut model = rc_portal_model();
    model.nodes.push(node(4, 2_000.0, 0.0, 3_500.0));
    model.nodes.push(node(5, 2_000.0, 3_000.0, 3_500.0));
    model.elements.push(line_elem(4, 4, 5, 0, 0));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let joist = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Joist)
        .unwrap();
    // 0.4×0.8×3.0 = 0.96 m³、型枠 (0.4+1.6)×3.0 = 6.0 m²
    assert!((joist.concrete_m3 - 0.96).abs() < 1e-9);
    assert!((joist.formwork_m2 - 6.0).abs() < 1e-9);
    // 主筋 0.8% → 0.96×0.008×7.85 t
    let main = joist
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::JoistMain)
        .unwrap();
    assert!((main.weight_t - 0.96 * 0.008 * 7.85).abs() < 1e-9);
}

#[test]
fn test_steel_member_weight() {
    // S 造: W = L×A×7.85（t/m³）。
    let mut model = rc_portal_model();
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    let a = shape.calc_area();
    let sec = shape.to_section(SectionId(2), "H-400x200x8x13".to_string());
    model.sections.push(sec);
    model.materials.push(steel_material(1));
    // 大梁を S 断面へ差し替え。
    model.elements[2].section = Some(SectionId(2));
    model.elements[2].material = Some(MaterialId(1));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();
    assert_eq!(girder.structure, StructureKind::S);
    assert!(girder.concrete_m3 == 0.0 && girder.formwork_m2 == 0.0);
    let steel = girder.steel.as_ref().unwrap();
    // 鉄骨は節点間距離（6.0m）で算定。
    assert!((steel.length_m - 6.0).abs() < 1e-9);
    assert!((steel.weight_t - a * 6_000.0 * 7.85e-9).abs() < 1e-12);
}

#[test]
fn test_brace_length_and_weight() {
    // ブレース長さは節点間距離 LB=√(L²+H²)。
    let mut model = rc_portal_model();
    let shape = SectionShape::SteelAngle {
        leg_a: 90.0,
        leg_b: 90.0,
        thick: 7.0,
    };
    let a = shape.calc_area();
    let sec = shape.to_section(SectionId(2), "L-90x90x7".to_string());
    model.sections.push(sec);
    model.materials.push(steel_material(1));
    let mut brace = line_elem(4, 0, 3, 2, 1);
    brace.kind = ElementKind::Brace {
        tension_only: false,
    };
    model.elements.push(brace);

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let br = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Brace)
        .unwrap();
    let lb = (6_000.0_f64.powi(2) + 3_500.0_f64.powi(2)).sqrt();
    let steel = br.steel.as_ref().unwrap();
    assert!((steel.length_m - lb / 1_000.0).abs() < 1e-9);
    assert!((steel.weight_t - a * lb * 7.85e-9).abs() < 1e-12);
}

#[test]
fn test_slab_quantity() {
    let mut model = rc_portal_model();
    model.slab_thickness = 150.0;
    // 6m×5m の床パネル。
    model.nodes.push(node(4, 0.0, 5_000.0, 3_500.0));
    model.nodes.push(node(5, 6_000.0, 5_000.0, 3_500.0));
    model.slabs.push(Slab {
        id: SlabId(0),
        boundary: vec![NodeId(2), NodeId(3), NodeId(5), NodeId(4)],
        joists: vec![],
        loads: vec![],
        method: DistributionMethod::TriTrapezoid,
        kind: SlabKind::Interior,
        one_way: None,
        edge_supported: None,
    });

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let slab = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Slab)
        .unwrap();
    // 面積 30 m²、体積 30×0.15 = 4.5 m³、型枠 30 m²
    assert!((slab.formwork_m2 - 30.0).abs() < 1e-6);
    assert!((slab.concrete_m3 - 4.5).abs() < 1e-6);
    // 床筋: 4.5×1.0%×7.85 t
    let bar = slab.rebar.first().unwrap();
    assert!((bar.weight_t - 4.5 * 0.01 * 7.85).abs() < 1e-9);
}

#[test]
fn test_wall_quantity_with_opening() {
    let mut model = rc_portal_model();
    // 柱・梁で囲まれた 4 節点壁（節点 0-1-3-2）。
    let shape = SectionShape::RcWall {
        thickness: 200.0,
        ps: 0.0025,
    };
    let sec = shape.to_section(SectionId(2), "W20".to_string());
    model.sections.push(sec);
    let mut wall = line_elem(4, 0, 1, 2, 0);
    wall.kind = ElementKind::Wall;
    wall.nodes = {
        let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
        for n in [0, 1, 3, 2] {
            v.push(NodeId(n));
        }
        v
    };
    model.elements.push(wall);
    model.wall_attrs.push(squid_n_core::model::WallAttr {
        elem: ElemId(4),
        opening_area: 2.0e6, // 2 m²
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    });

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let w = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Wall)
        .unwrap();
    // 内法: L = 6000 − 700/2×2 = 5300（柱 700 角）、H = 3500 − 800/2×2 = 2700（梁せい 800）
    let net = 5.3_f64 * 2.7 - 2.0;
    assert!((w.concrete_m3 - net * 0.2).abs() < 1e-6);
    assert!((w.formwork_m2 - net * 2.0).abs() < 1e-6);
    // 壁筋: 横筋・縦筋の 2 件が計上される。
    assert_eq!(w.rebar.len(), 2);
    assert!(w.rebar_weight_t() > 0.0);
}

#[test]
fn test_totals_aggregation() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let totals = q.totals();
    let sum: f64 = q.items.iter().map(|i| i.concrete_m3).sum();
    assert!((totals.concrete_m3 - sum).abs() < 1e-12);
    // 部位別小計に柱・大梁・基礎梁が現れる。
    let by_cat = q.totals_by_category();
    assert_eq!(by_cat.len(), 3);
}
