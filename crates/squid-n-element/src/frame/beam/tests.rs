use super::*;
use crate::transform::LocalFrame;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, LocalAxis, Material, Model, Node, RigidZone, Section,
};

fn make_test_beam() -> BeamElement {
    BeamElement {
        id: ElemId(0),
        e: 205000.0,
        g: 78846.15,
        a: 80000.0,
        a_mass: 80000.0,
        iy: 1.0666667e9,
        iz: 1.0666667e9,
        j: 0.0,
        as_y: 66666.67,
        as_z: 66666.67,
        length: 3000.0,
        density: 0.0,
        nodes: [NodeId(0), NodeId(1)],
        axis: LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        rigid: RigidZone::default(),
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        eval_sections: vec![0.0, 0.5, 1.0],
        section: None,
        material: None,
        committed_disp: [0.0; 12],
        trial_disp: [0.0; 12],
    }
}

/// SRC/CFT の複合換算が要素生成へ配線されていること（SRC規準の考え方・ヤング係数比による等価換算）。
#[test]
fn test_beam_new_src_cft_composite_props() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Model};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar, E_STEEL, N_S_EQ};

    let src_shape = SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 12.0,
        steel_grade: "SN400B".into(),
    };
    let cft_shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };

    let mut model = Model {
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![
            src_shape.to_section(SectionId(0), "SRC-600".into()),
            cft_shape.to_section(SectionId(1), "CFT-400".into()),
        ],
        materials: vec![
            Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            },
            Material {
                concrete_class: Default::default(),
                id: MaterialId(1),
                name: "BCR295(充填FC36)".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: Some(36.0),
                fy: Some(295.0),
            },
        ],
        ..Default::default()
    };
    let make_elem = |sec: u32, mat: u32| ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(squid_n_core::ids::SectionId(sec)),
        material: Some(squid_n_core::ids::MaterialId(mat)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    // SRC + コンクリート材料: ns=Es/Ec による等価断面性能
    let src_beam = BeamElement::new(&make_elem(0, 0), &model);
    let p = src_shape.src_equivalent_props(23000.0, 0.2).unwrap();
    assert!((src_beam.a - p.area_ax).abs() < 1e-6);
    // 断面レイヤの iy（強軸）・as_z（ウェブ）は要素座標系では iz・as_y に入る
    assert!((src_beam.iz - p.iy).abs() / p.iy < 1e-12);
    assert!((src_beam.j - p.j).abs() / p.j < 1e-12);
    assert!((src_beam.as_y - p.as_z).abs() < 1e-6);
    // ns=205000/23000≈8.91 は既定 N_S_EQ=15 と異なる値になること
    let ns = E_STEEL / 23000.0;
    assert!((ns - N_S_EQ).abs() > 1.0);
    // 質量用断面積は幾何断面(コンクリート全断面)のまま
    assert!((src_beam.a_mass - 360_000.0).abs() < 1e-9);

    // CFT + 鋼材料(fc=充填強度): 充填コンクリートの 1/n 換算累加
    let cft_beam = BeamElement::new(&make_elem(1, 1), &model);
    let pc = cft_shape.cft_equivalent_props(205000.0, 0.3, 36.0).unwrap();
    assert!((cft_beam.a - pc.area_ax).abs() < 1e-6);
    assert!((cft_beam.iz - pc.iy).abs() / pc.iy < 1e-12);
    assert!((cft_beam.j - pc.j).abs() / pc.j < 1e-12);

    // SRC + fc の無い材料: 既定 N_S_EQ の軸剛性累加へフォールバック
    model.materials[0].fc = None;
    let src_fallback = BeamElement::new(&make_elem(0, 0), &model);
    assert!((src_fallback.a - src_shape.calc_axial_stiffness_area()).abs() < 1e-6);
    assert!((src_fallback.iz - model.sections[0].iy).abs() < 1e-6);
}

/// スラブ協力幅による強軸剛性増大（RC規準8条）。
#[test]
fn test_beam_new_slab_cooperation_width_amplifies_iy() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, EndCondition, ForceRegime, LocalAxis, Model, Slab,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let shape = SectionShape::RcRect {
        b: 300.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 2500.0, 3000.0]),
            make_node(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![shape.to_section(SectionId(0), "RC-300x600".into())],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        slabs: vec![Slab {
            usage: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            kind: Default::default(),
            one_way: None,
            edge_supported: None,
            thickness: None,
        }],
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    // 期待値: a は隣接平行梁との内法距離（RC規準8条の a）。軸間 2500 から
    // 自梁の幅/2 と相手梁の幅/2（向かい側に梁要素が無いため自梁と同幅の
    // フォールバック）を控除して a=2500−150−150=2200 < l/2=3000
    // → ba=(0.5−0.6·2200/6000)·2200=616(片側のみ)
    let (b, d, t, l) = (300.0_f64, 600.0_f64, 150.0_f64, 6000.0_f64);
    let a_clear = 2500.0 - b / 2.0 - b / 2.0;
    let ba = (0.5 - 0.6 * a_clear / l) * a_clear;
    assert!((ba - 616.0).abs() < 1e-9);
    let bf = b + ba;
    let (aw, af) = (b * d, (bf - b) * t);
    let g = (aw * d / 2.0 + af * (d - t / 2.0)) / (aw + af);
    let i0 = b * d.powi(3) / 12.0;
    let ie = i0
        + aw * (g - d / 2.0).powi(2)
        + (bf - b) * t.powi(3) / 12.0
        + af * (d - t / 2.0 - g).powi(2);

    let beam = BeamElement::new(&elem, &model);
    // 強軸（鉛直曲げ）は要素座標系では iz（Mz 面）
    assert!(
        (beam.iz - ie).abs() / ie < 1e-12,
        "iz={} ie={}",
        beam.iz,
        ie
    );
    assert!(beam.iz / i0 > 1.3, "増大率が小さすぎる: {}", beam.iz / i0);
    // 弱軸（要素座標系では iy）は増大しない
    assert!((beam.iy - model.sections[0].iz).abs() < 1e-9);

    // 床厚 0(既定)では従来どおり
    model.slab_thickness = 0.0;
    let beam0 = BeamElement::new(&elem, &model);
    assert!((beam0.iz - i0).abs() < 1e-9);
}

/// S 造合成梁の剛性（スラブ考慮換算断面と鉄骨単独の平均。計算編 02「合成梁の
/// 断面性能」）。
#[test]
fn test_beam_new_composite_steel_beam_averages_stiffness() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, EndCondition, ForceRegime, LocalAxis, Model, Slab,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 2500.0, 3000.0]),
            make_node(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![shape.to_section(SectionId(0), "H-400x200".into())],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        slabs: vec![Slab {
            usage: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            kind: Default::default(),
            one_way: None,
            edge_supported: None,
            thickness: None,
        }],
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    // 期待値: 協力幅 bf = b + ba（片側のみ）。a=2500−100−100=2300 < l/2
    // → ba=(0.5−0.6·2300/6000)·2300=621。合成断面（スラブ上端基準・Hd=0・
    // スラブ Fc21）と鉄骨単独の平均。
    let sec = &model.sections[0];
    let (sa, si, sh) = (sec.area, sec.iy, 400.0_f64);
    let (es, t, l) = (205000.0_f64, 150.0_f64, 6000.0_f64);
    let a_clear = 2500.0 - 100.0 - 100.0;
    let ba = (0.5 - 0.6 * a_clear / l) * a_clear;
    assert!((ba - 621.0).abs() < 1e-9);
    let bf = 200.0 + ba;
    let ec = squid_n_core::section_shape::concrete_young_modulus(21.0);
    let ca = bf * t;
    let g = (ec * ca * (t / 2.0) + es * sa * (t + sh / 2.0)) / (ec * ca + es * sa);
    let i_comp = (ec / es) * (bf * t.powi(3) / 12.0 + ca * (g - t / 2.0).powi(2))
        + si
        + sa * (g - t - sh / 2.0).powi(2);
    let expected = (i_comp + si) / 2.0;

    let beam = BeamElement::new(&elem, &model);
    // 強軸（鉛直曲げ）は要素座標系では iz（Mz 面）
    assert!(
        (beam.iz - expected).abs() / expected < 1e-12,
        "iz={} expected={}",
        beam.iz,
        expected
    );
    // 平均法: 鉄骨単独 < 採用剛性 < 完全合成
    assert!(beam.iz > si && beam.iz < i_comp);
    // 弱軸（要素座標系では iy）は増大しない
    assert!((beam.iy - model.sections[0].iz).abs() < 1e-9);

    // 床厚 0(既定)では鉄骨単独のまま
    model.slab_thickness = 0.0;
    let beam0 = BeamElement::new(&elem, &model);
    assert!((beam0.iz - si).abs() < 1e-9);
}

#[test]
fn test_local_stiffness_symmetric() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                k.get(i, j),
                k.get(j, i)
            );
        }
    }
}

#[test]
fn test_phi_zero_converges_to_bernoulli() {
    // As → ∞ => phi → 0 => Timoshenko → Bernoulli
    let mut beam = make_test_beam();
    beam.as_y = 1e30;
    beam.as_z = 1e30;
    let k_timo = beam.local_stiffness_raw();

    // Bernoulli reference: same beam with phi=0
    let e = beam.e;
    let iz = beam.iz;
    let iy = beam.iy;
    let a = beam.a;
    let l = beam.length;
    let g = beam.g;
    let jj = beam.j;

    let az = e * iz / (l * l * l);
    let ay = e * iy / (l * l * l);

    for i in 0..12 {
        for j in 0..12 {
            let norm_pair = if i <= j { (i, j) } else { (j, i) };
            let bernoulli = match norm_pair {
                (0, 0) | (6, 6) => e * a / l,
                (0, 6) => -e * a / l,
                (3, 3) | (9, 9) => g * jj / l,
                (3, 9) => -g * jj / l,
                (1, 1) | (7, 7) => 12.0 * az,
                (1, 7) => -12.0 * az,
                (1, 5) | (1, 11) => 6.0 * az * l,
                (5, 7) | (7, 11) => -6.0 * az * l,
                (5, 5) | (11, 11) => 4.0 * az * l * l,
                (5, 11) => 2.0 * az * l * l,
                (2, 2) | (8, 8) => 12.0 * ay,
                (2, 8) => -12.0 * ay,
                (2, 4) | (2, 10) => -6.0 * ay * l,
                (4, 8) | (8, 10) => 6.0 * ay * l,
                (4, 4) | (10, 10) => 4.0 * ay * l * l,
                (4, 10) => 2.0 * ay * l * l,
                _ => 0.0,
            };
            let timo = k_timo.get(i, j);
            assert!(
                (timo - bernoulli).abs() < 1e-6,
                "K[{i}][{j}]: timo={timo}, bernoulli={bernoulli}"
            );
        }
    }
}

#[test]
fn test_beam_axial_stiffness() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    let ea_l = beam.e * beam.a / beam.length;
    assert!((k.get(0, 0) - ea_l).abs() < 1e-9);
    assert!((k.get(0, 6) + ea_l).abs() < 1e-9);
    assert!((k.get(6, 0) + ea_l).abs() < 1e-9);
    assert!((k.get(6, 6) - ea_l).abs() < 1e-9);
}

#[test]
fn test_beam_torsion_stiffness() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    let gj_l = beam.g * beam.j / beam.length;
    assert!((k.get(3, 3) - gj_l).abs() < 1e-9);
    assert!((k.get(9, 9) - gj_l).abs() < 1e-9);
    assert!((k.get(3, 9) + gj_l).abs() < 1e-9);
}

#[test]
fn test_rigid_zone_preserves_rigid_body_rotation() {
    // 剛域変換は剛体運動不変性を保たねばならない: 要素全体を剛体回転させると
    // 可撓部にひずみは生じず、節点力はゼロでなければならない。
    // 剛域腕の運動学 u_flex = u_node + θ×r（i端 r=+li·ex, j端 r=-lj·ex）より
    // uy_i'=uy_i+li·rz_i, uz_i'=uz_i-li·ry_i, uy_j'=uy_j-lj·rz_j, uz_j'=uz_j+lj·ry_j。
    // 従来はこの 4 項の符号がすべて逆で、剛体回転に対し偽の材端モーメント・
    // せん断（~1e7 オーダ）を生じていた。
    let mut beam = make_test_beam();
    beam.j = 5.0e8; // ねじり剛性を与える（回転自由度の一般性確保）
    beam.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let k = beam.local_stiffness();

    // 局所 z 軸まわりに節点 i を中心とする剛体回転 θ:
    // uy_j = θ·L、rz_i = rz_j = θ、その他 0。
    let theta = 1.0; // 線形剛性なので大きさは任意（剛体モードは厳密に核）
    let l = beam.length;
    let u = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        theta, // 節点 i
        0.0,
        theta * l,
        0.0,
        0.0,
        0.0,
        theta, // 節点 j
    ];
    // f = K·u（剛体回転なので ≈ 0 でなければならない）。
    let mut fmax = 0.0_f64;
    for i in 0..12 {
        let mut fi = 0.0;
        for j in 0..12 {
            fi += k.get(i, j) * u[j];
        }
        fmax = fmax.max(fi.abs());
    }
    // 代表剛性スケール（曲げ対角）に対して十分小さいこと。
    let scale = k.get(1, 1).abs().max(1.0);
    assert!(
        fmax / scale < 1e-9,
        "rigid-body z-rotation must produce ~zero nodal force: fmax={fmax}, scale={scale}"
    );

    // 同様に局所 y 軸まわりの剛体回転（uz_j = -θ·L、ry_i=ry_j=θ）。
    let u_y = [
        0.0,
        0.0,
        0.0,
        0.0,
        theta,
        0.0, // 節点 i
        0.0,
        0.0,
        -theta * l,
        0.0,
        theta,
        0.0, // 節点 j
    ];
    let mut fmax_y = 0.0_f64;
    for i in 0..12 {
        let mut fi = 0.0;
        for j in 0..12 {
            fi += k.get(i, j) * u_y[j];
        }
        fmax_y = fmax_y.max(fi.abs());
    }
    assert!(
        fmax_y / scale < 1e-9,
        "rigid-body y-rotation must produce ~zero nodal force: fmax_y={fmax_y}, scale={scale}"
    );
}

#[test]
fn test_torsion_not_stiffened_by_rigid_zone() {
    // ねじりは剛域で増大させない（軸剛性と同じく節点間長 L 基準 GJ/L）。
    // 剛域を入れても剛性は GJ/l_flex ではなく GJ/L のまま。
    let mut beam = make_test_beam();
    beam.j = 5.0e8;
    beam.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let k = beam.local_stiffness();
    let gj_l = beam.g * beam.j / beam.length; // 全長 3000 基準（可撓長 2400 ではない）
    assert!(
        (k.get(3, 3) - gj_l).abs() / gj_l < 1e-9,
        "ねじりは GJ/L: got {}, want {}",
        k.get(3, 3),
        gj_l
    );
    assert!((k.get(3, 9) + gj_l).abs() / gj_l < 1e-9);
}

#[test]
fn test_geometric_stiffness_consistent_with_rigid_zone() {
    use crate::behavior::ElementBehavior;
    let n = 1000.0;
    // 剛域なし: 従来どおり全長 L 基準（回帰なしを確認）。
    let kg = make_test_beam().geometric_stiffness(n);
    let expected_full = n / 3000.0 * 6.0 / 5.0;
    assert!((kg.get(1, 1) - expected_full).abs() / expected_full < 1e-9);

    // 剛域あり: 可撓長基準となり弾性剛性と整合（並進対角 N/l_flex·6/5 が増える）。
    let mut beam_rz = make_test_beam();
    beam_rz.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let kg_rz = beam_rz.geometric_stiffness(n);
    let expected_flex = n / 2400.0 * 6.0 / 5.0; // 可撓長 2400
    assert!(
        (kg_rz.get(1, 1) - expected_flex).abs() / expected_flex < 1e-9,
        "剛域ありは可撓長基準: got {}, want {}",
        kg_rz.get(1, 1),
        expected_flex
    );
    assert!(kg_rz.get(1, 1) > kg.get(1, 1));
}

#[test]
fn test_pinned_end_releases_moment() {
    // i端をピンにすると、i端回転行/列がほぼゼロになり剛性が低下
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let k = beam.local_stiffness();
    // i端の My, Mz 対角成分が Fixed 時より大幅に小さい
    let k_fixed = make_test_beam().local_stiffness();
    assert!(k.get(4, 4) < k_fixed.get(4, 4) * 1e-6);
    assert!(k.get(5, 5) < k_fixed.get(5, 5) * 1e-6);
}

#[test]
fn test_fixed_ends_exact_equals_raw() {
    // 両端剛接は raw 剛性そのもの（ペナルティばね近似を用いない厳密な扱い）。
    // 剛域なし・両端固定なので local_stiffness は raw と厳密に一致する。
    let beam = make_test_beam();
    let k = beam.local_stiffness();
    let raw = beam.local_stiffness_raw();
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - raw.get(i, j)).abs() < 1e-9,
                "K[{i},{j}] {} != raw {}",
                k.get(i, j),
                raw.get(i, j)
            );
        }
    }
}

#[test]
fn test_pinned_end_rotation_stiffness_exactly_zero() {
    // ピン端の節点回転への当要素の寄与は「厳密に 0」（従来のペナルティでは ~1e-8 残っていた）。
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let k = beam.local_stiffness();
    for r in [3usize, 4, 5] {
        for c in 0..12 {
            assert_eq!(
                k.get(r, c),
                0.0,
                "released rot DOF {r} row must be exactly 0 at col {c}"
            );
            assert_eq!(
                k.get(c, r),
                0.0,
                "released rot DOF {r} col must be exactly 0 at row {c}"
            );
        }
    }
}

#[test]
fn test_auto_rigid_zone_standard_formula() {
    // 柱せい 600, 梁せい 700 の T 字接合
    // 梁端 λ = 柱せい/2 - 梁せい/4 = 300 - 175 = 125
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    let col_sec = Section {
        id: SectionId(0),
        name: "col".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 600.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 700.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!((zone.length_i - 125.0).abs() < 1e-9);
    // フェイス距離 face_i = D_orth/2 = 柱せい/2 = 300（低減率は掛けない）。
    assert!((zone.face_i - 300.0).abs() < 1e-9, "face_i={}", zone.face_i);
}

/// apply_auto_rigid_zones が ElementData::rigid_zone に反映され、
/// Manual 端が保護されることを確認する（剛域がモデル→解析へ接続されたこと）。
#[test]
fn test_apply_auto_rigid_zones_and_manual_protection() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementKind, ZoneSource};

    let mk_sec = |id: u32, depth: f64| Section {
        id: SectionId(id),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mk_node = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let mk_beam = |id: u32, a: u32, b: u32, sec: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(sec)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: squid_n_core::model::ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let mut model = Model {
        nodes: vec![
            mk_node(0, [0.0, 0.0, 0.0]),
            mk_node(1, [0.0, 0.0, 3000.0]),
            mk_node(2, [4000.0, 0.0, 3000.0]),
        ],
        elements: vec![mk_beam(0, 0, 1, 0), mk_beam(1, 1, 2, 1)], // 柱(せい600)・梁(せい700)
        sections: vec![mk_sec(0, 600.0), mk_sec(1, 700.0)],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: String::new(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };

    // 既定では剛域長 0（未適用）。
    assert_eq!(model.elements[1].rigid_zone.length_i, 0.0);

    apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
    // 梁端（接合部側）に λ = 柱せい/2 − 梁せい/4 = 300 − 175 = 125 が入る。
    assert!(
        (model.elements[1].rigid_zone.length_i - 125.0).abs() < 1e-9,
        "λ_i={}",
        model.elements[1].rigid_zone.length_i
    );

    // 手動端は再適用で保護される。
    model.elements[1].rigid_zone.source_i = ZoneSource::Manual;
    model.elements[1].rigid_zone.length_i = 999.0;
    model.elements[1].rigid_zone.face_i = 0.0;
    apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
    assert_eq!(
        model.elements[1].rigid_zone.length_i, 999.0,
        "Manual 端が上書きされた"
    );
    // face_i は剛域長の Manual/Auto フラグとは無関係な幾何量なので、
    // Manual 端でも常に再算定される（設計書 §6.2.1）。
    assert!(
        (model.elements[1].rigid_zone.face_i - 300.0).abs() < 1e-9,
        "Manual 端でも face_i は再算定されるべき: face_i={}",
        model.elements[1].rigid_zone.face_i
    );
}

/// 危険断面位置（§6.2.3）: face_i/face_j から評価断面リストを算定する。
/// face=0（直交材なし）の端では従来どおり [0.0, 0.5, 1.0] と完全一致する。
#[test]
fn test_eval_sections_from_face_distance() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementKind, RigidZone};

    let sec = Section {
        id: SectionId(0),
        name: String::new(),
        area: 100.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: String::new(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [4000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: RigidZone {
                face_i: 300.0,
                face_j: 250.0,
                ..Default::default()
            },
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };

    let beam = BeamElement::new(&model.elements[0], &model);
    let expected = [0.0, 0.075, 0.5, 0.9375, 1.0];
    assert_eq!(beam.eval_sections.len(), expected.len());
    for (a, b) in beam.eval_sections.iter().zip(expected.iter()) {
        assert!(
            (a - b).abs() < 1e-9,
            "eval_sections={:?}",
            beam.eval_sections
        );
    }

    // face=0 の端では従来どおり [0.0, 0.5, 1.0] と完全一致。
    let mut model_zero = model.clone();
    model_zero.elements[0].rigid_zone = RigidZone::default();
    let beam_zero = BeamElement::new(&model_zero.elements[0], &model_zero);
    assert_eq!(beam_zero.eval_sections, vec![0.0, 0.5, 1.0]);

    // 部材付帯情報（ハンチ・継手位置）があれば、ハンチ端・継手位置も評価断面に
    // 加わる（§6.2.3 の追加検定位置。剛性には影響しない）。
    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};
    let mut model_detail = model.clone();
    model_detail.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 3000.0,
            kind: JointKind::Site,
        }],
    });
    let beam_detail = BeamElement::new(&model_detail.elements[0], &model_detail);
    // face_i=300, ハンチ長 700 → (300+700)/4000 = 0.25、継手 3000/4000 = 0.75
    let expected_detail = [0.0, 0.075, 0.25, 0.5, 0.75, 0.9375, 1.0];
    assert_eq!(beam_detail.eval_sections.len(), expected_detail.len());
    for (a, b) in beam_detail.eval_sections.iter().zip(expected_detail.iter()) {
        assert!(
            (a - b).abs() < 1e-9,
            "eval_sections={:?}",
            beam_detail.eval_sections
        );
    }

    // 付帯情報を付けても剛性行列は不変（剛性には影響しない）。
    let beam_base = BeamElement::new(&model.elements[0], &model);
    assert_eq!(
        beam_base.local_stiffness().data,
        beam_detail.local_stiffness().data,
        "付帯情報の有無で剛性行列が変わってはならない"
    );
}

/// 剛域算定用の RC 配筋（本数・径は最小限のダミー値。断面性能の絶対値は無関係）。
fn simple_rc_rebar() -> squid_n_core::section_shape::RcRebar {
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
    RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 16.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 16.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    }
}

/// S造仕口（柱・梁とも鋼材形状）: 直交する RC/SRC 系の柱（梁）が存在しないため、
/// 仕口部に接続する柱(梁)がすべてＳの場合は剛域長さ0（λ=0）になる。
#[test]
fn test_auto_rigid_zone_steel_joint_is_zero() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
    .to_section(SectionId(0), "col-H400".to_string());
    let beam_sec = SectionShape::SteelH {
        height: 500.0,
        width: 200.0,
        web_thick: 10.0,
        flange_thick: 16.0,
    }
    .to_section(SectionId(1), "beam-H500".to_string());
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert_eq!(
        zone.length_i, 0.0,
        "S造仕口の剛域長は0のはず: length_i={}",
        zone.length_i
    );
}

/// S梁 + RC柱: Ｓ・ＣＦＴ柱の場合はＲＣ・ＳＲＣ大梁のうち最大せいの梁
/// フェイスまでの長さとなり、λ = 柱せい/2（D/4控除なし・reductionも掛けない）。
#[test]
fn test_auto_rigid_zone_steel_beam_rc_column() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: simple_rc_rebar(),
    }
    .to_section(SectionId(0), "col-RC600".to_string());
    let beam_sec = SectionShape::SteelH {
        height: 500.0,
        width: 200.0,
        web_thick: 10.0,
        flange_thick: 16.0,
    }
    .to_section(SectionId(1), "beam-H500".to_string());
    let rc_mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "concrete".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let s_mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![rc_mat, s_mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 300.0).abs() < 1e-9,
        "S梁+RC柱: λ_i={} (期待値=柱せい/2=300)",
        zone.length_i
    );
}

/// RC梁 + S柱のみ: 直交する RC/SRC 系の柱が無いため D_orth_rc=0 となり、
/// 従来式 λ=reduction·(0/2−梁せい/4) は負となって 0 にクランプされる。
#[test]
fn test_auto_rigid_zone_rc_beam_steel_column_only_is_zero() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
    .to_section(SectionId(0), "col-H400".to_string());
    let beam_sec = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: simple_rc_rebar(),
    }
    .to_section(SectionId(1), "beam-RC600".to_string());
    let s_mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    let rc_mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "concrete".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![s_mat, rc_mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert_eq!(
        zone.length_i, 0.0,
        "RC梁+S柱のみ: 剛域長は0のはず（RC/SRC直交材が無い）。length_i={}",
        zone.length_i
    );
}

/// 耐震壁要素（ElementKind::Wall）が節点に接続していても、直交せい探索の対象は
/// Beam 要素のみなので結果に影響しない（耐震壁周辺の柱・梁の剛域は
/// 考慮しない扱い）。壁を追加しても標準ケース（柱600・梁700 → λ=125）と同じ結果。
#[test]
fn test_auto_rigid_zone_wall_does_not_affect_orthogonal_search() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    let col_sec = Section {
        id: SectionId(0),
        name: "col".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 600.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 700.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    // 壁のせい（名目値）を柱・梁より大きくし、混入すれば結果が変わることを検証可能にする。
    let wall_sec = Section {
        id: SectionId(2),
        name: "wall".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 1000.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 4000.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            // 節点1に接続する壁要素（節点1-3）。梁と直交するがWall kindなので無視される。
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Wall,
                nodes: smallvec::smallvec![NodeId(1), NodeId(3)],
                section: Some(SectionId(2)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec, wall_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 125.0).abs() < 1e-9,
        "壁のせいが紛れ込んでいないはず: λ_i={}",
        zone.length_i
    );
    assert!(
        (zone.face_i - 300.0).abs() < 1e-9,
        "壁のせいが紛れ込んでいないはず: face_i={}",
        zone.face_i
    );
}

/// 壁エレメントモデルの上下大梁の剛性倍率（壁エレメント置換モデルの上下大梁の断面性能）。
/// 4節点 Wall 要素の下辺2節点を結ぶ水平梁は iy/a が既定倍率（100倍）になる。
#[test]
fn test_beam_new_wall_girder_bottom_edge_scales_stiffness() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "beam".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
    };

    // 壁なしモデル（基準）
    let model_no_wall = Model {
        nodes: nodes.clone(),
        elements: vec![beam_elem.clone()],
        sections: vec![sec.clone()],
        materials: vec![mat.clone()],
        ..Default::default()
    };
    let beam_no_wall = BeamElement::new(&beam_elem, &model_no_wall);

    // 壁ありモデル: 節点0-1が下辺、2-3が上辺の4節点壁
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model_with_wall = Model {
        nodes,
        elements: vec![beam_elem.clone(), wall_elem],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };
    let beam_with_wall = BeamElement::new(&beam_elem, &model_with_wall);

    assert!(
        (beam_with_wall.iy / beam_no_wall.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "iy倍率が既定100倍でない: with={} without={}",
        beam_with_wall.iy,
        beam_no_wall.iy
    );
    assert!(
        (beam_with_wall.a / beam_no_wall.a - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "a倍率が既定100倍でない: with={} without={}",
        beam_with_wall.a,
        beam_no_wall.a
    );
    // 質量用断面積（a_mass）は倍率の対象外
    assert!(
        (beam_with_wall.a_mass - beam_no_wall.a_mass).abs() < 1e-9,
        "a_massは変更されないはず"
    );
}

/// 壁の節点を1つしか共有しない梁（壁の上辺・下辺ではない）には倍率が掛からない。
#[test]
fn test_beam_new_wall_girder_requires_both_nodes_shared() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "beam".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    // 節点1は壁の隅、節点4は壁に属さない別節点（梁は壁の外へ伸びる）
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
        make_node(4, [8000.0, 0.0, 0.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(1), NodeId(4)],
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
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![beam_elem.clone(), wall_elem],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };
    let beam = BeamElement::new(&beam_elem, &model);
    assert!(
        (beam.iy - sec.iy).abs() < 1e-9,
        "壁節点を1つしか共有しない梁には倍率が掛からないはず: iy={}",
        beam.iy
    );
}

/// 鉛直材（柱）は壁節点を2つ共有していても水平材ではないため倍率は掛からない。
#[test]
fn test_beam_new_wall_girder_vertical_member_not_scaled() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "column".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    // 左辺（節点0-3）を結ぶ鉛直材。両端とも壁の節点だが鉛直材なので対象外。
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![column_elem.clone(), wall_elem],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };
    let column = BeamElement::new(&column_elem, &model);
    assert!(
        (column.iy - sec.iy).abs() < 1e-9,
        "鉛直材は水平材ではないため倍率が掛からないはず: iy={}",
        column.iy
    );
}

/// フレーム内雑壁（耐震壁不成立）の柱への袖壁算入（RC規準の耐震壁規定・
/// フレーム内雑壁のモデル化）。大開口(r0=√(3.6e6/12e6)=0.548>0.4)の壁は
/// 耐震壁不成立となり、側柱（左辺=節点0-3）に袖壁として断面性能算入される。
/// 面内（iz・as_y）は平行軸の定理による合成値と一致し、面外（iy・as_z）は不変。
#[test]
fn test_beam_new_misc_wall_wing_augments_column_inplane_stiffness() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 90_000.0,
        iy: 3.0e9,
        iz: 2.0e9,
        j: 1.0e7,
        depth: 300.0,
        width: 300.0,
        as_y: 50_000.0,
        as_z: 60_000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut model = Model {
        nodes,
        elements: vec![column_elem.clone(), wall_elem],
        sections: vec![
            col_sec.clone(),
            wall_shape.to_section(SectionId(1), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };
    model.wall_attrs.push(WallAttr {
        elem: ElemId(1),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![WallOpening {
            width: 2400.0,
            height: 1500.0,
            offset: Some([800.0, 750.0]),
        }],
    });

    let column = BeamElement::new(&column_elem, &model);

    // 手計算（misc_wall::tests::test_collect_misc_walls_and_lengths と同じ壁形状）:
    // wing_length(side=0)=800、lww=800-300/2=650、Aw=150*650=97500。
    let d_col: f64 = 300.0;
    let lww = 650.0_f64;
    let aw = 150.0 * lww;
    let ac = col_sec.area;
    let e_i = -(d_col / 2.0 + lww / 2.0);
    let g = (aw * e_i) / (ac + aw);
    let self_i = 150.0 * lww.powi(3) / 12.0;
    // 要素座標系では断面 iy（強軸）が elem.iz、断面 as_z が elem.as_y に入る
    // （construct.rs のクロス変換）。面内合成のベースはその値。
    let expected_iz = col_sec.iy + ac * g * g + self_i + aw * (e_i - g).powi(2);

    assert!(
        (column.a - (ac + aw)).abs() < 1e-6,
        "a={} expected={}",
        column.a,
        ac + aw
    );
    assert!(
        (column.iz - expected_iz).abs() / expected_iz < 1e-9,
        "iz={} expected={}",
        column.iz,
        expected_iz
    );
    assert!(
        (column.as_y - (col_sec.as_z + aw / 1.2)).abs() < 1e-6,
        "as_y={}",
        column.as_y
    );
    // 面外（iy・as_z）は袖壁算入の影響を受けない
    assert!((column.iy - col_sec.iz).abs() < 1e-6, "iy={}", column.iy);
    assert!(
        (column.as_z - col_sec.as_y).abs() < 1e-6,
        "as_z={}",
        column.as_z
    );
}

/// 同じ大開口壁の下辺梁（節点0-1）への腰壁算入。鉛直曲げ（要素座標系では
/// iz・as_y）へ平行軸の定理で合成され、耐震壁不成立のため上下大梁100倍は掛からない。
#[test]
fn test_beam_new_misc_wall_strip_augments_girder_iy_without_100x() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let beam_sec = Section {
        id: SectionId(0),
        name: "beam".into(),
        area: 200_000.0,
        iy: 5.0e9,
        iz: 1.0e9,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 70_000.0,
        as_z: 70_000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut model = Model {
        nodes,
        elements: vec![beam_elem.clone(), wall_elem],
        sections: vec![
            beam_sec.clone(),
            wall_shape.to_section(SectionId(1), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };
    model.wall_attrs.push(WallAttr {
        elem: ElemId(1),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![WallOpening {
            width: 2400.0,
            height: 1500.0,
            offset: Some([800.0, 750.0]),
        }],
    });

    let beam = BeamElement::new(&beam_elem, &model);

    // 手計算: strip_height(top=false)=750（lw/2=2000 は開口 x:[800,3200] 内）、
    // hw=750-600/2=450、Aw=150*450=67500。下辺の梁なので壁は上に載る(+方向)。
    let d_beam: f64 = 600.0;
    let hw = 450.0_f64;
    let aw = 150.0 * hw;
    let ac = beam_sec.area;
    let e_i = d_beam / 2.0 + hw / 2.0;
    let g = (aw * e_i) / (ac + aw);
    let self_i = 150.0 * hw.powi(3) / 12.0;
    // 鉛直曲げは要素座標系では iz（ベースは断面 iy=強軸）、対のせん断は as_y
    // （ベースは断面 as_z）に入る（construct.rs のクロス変換）。
    let expected_iz = beam_sec.iy + ac * g * g + self_i + aw * (e_i - g).powi(2);

    assert!(
        (beam.a - (ac + aw)).abs() < 1e-6,
        "a={} expected={}",
        beam.a,
        ac + aw
    );
    assert!(
        (beam.iz - expected_iz).abs() / expected_iz < 1e-9,
        "iz={} expected={}",
        beam.iz,
        expected_iz
    );
    assert!(
        (beam.as_y - (beam_sec.as_z + aw / 1.2)).abs() < 1e-6,
        "as_y={}",
        beam.as_y
    );
    // 耐震壁不成立のため上下大梁100倍は掛からない（合成値は元の強軸値の高々数倍）
    assert!(
        beam.iz < beam_sec.iy * 10.0,
        "100倍が誤って適用されている可能性: iz={} base={}",
        beam.iz,
        beam_sec.iy
    );
    // 弱軸（要素座標系では iy・as_z）は腰壁算入の影響を受けない
    assert!((beam.iy - beam_sec.iz).abs() < 1e-6, "iy={}", beam.iy);
    assert!(
        (beam.as_z - beam_sec.as_y).abs() < 1e-6,
        "as_z={}",
        beam.as_z
    );
}

/// 耐震壁が成立する壁（無開口・t=150）の周辺部材: 柱・梁とも雑壁算入されず、
/// 上下大梁は従来どおり100倍（`WALL_GIRDER_STIFF_FACTOR`）のままとなる
/// （雑壁算入と上下大梁100倍は排他: `collect_misc_walls` は不成立壁のみ返す）。
#[test]
fn test_beam_new_seismic_wall_no_misc_wall_augmentation() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 90_000.0,
        iy: 3.0e9,
        iz: 2.0e9,
        j: 1.0e7,
        depth: 300.0,
        width: 300.0,
        as_y: 50_000.0,
        as_z: 60_000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".into(),
        area: 200_000.0,
        iy: 5.0e9,
        iz: 1.0e9,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 70_000.0,
        as_z: 70_000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let beam_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(2),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(2)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    // 開口なし（wall_attrs 未設定）・t=150 → 耐震壁成立
    let model = Model {
        nodes,
        elements: vec![column_elem.clone(), beam_elem.clone(), wall_elem],
        sections: vec![
            col_sec.clone(),
            beam_sec.clone(),
            wall_shape.to_section(SectionId(2), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };

    let column = BeamElement::new(&column_elem, &model);
    assert!(
        (column.iz - col_sec.iy).abs() < 1e-6,
        "耐震壁成立時は柱に袖壁算入されないはず: iz={}",
        column.iz
    );
    assert!((column.a - col_sec.area).abs() < 1e-6, "a={}", column.a);
    assert!(
        (column.as_y - col_sec.as_z).abs() < 1e-6,
        "as_y={}",
        column.as_y
    );

    let beam = BeamElement::new(&beam_elem, &model);
    assert!(
        (beam.iz / beam_sec.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "耐震壁成立時は従来どおり上下大梁100倍のはず: iz={} base={}",
        beam.iz,
        beam_sec.iy
    );
    assert!(
        (beam.a / beam_sec.area - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "a={} base={}",
        beam.a,
        beam_sec.area
    );
}

/// 断面レイヤ→要素座標系のクロス変換の回帰テスト（軸名の取り違え防止）。
///
/// 断面レイヤは「iy=強軸（せい方向 D³ 系）・as_z=ウェブ」の規約だが、要素座標系は
/// せい方向＝ローカル y のため、梁の鉛直たわみ（uy、Mz 面）の剛性は断面の強軸値
/// iy・as_z で、水平たわみ（uz、My 面）は弱軸値 iz・as_y で組み立てられなければ
/// ならない。クロス変換（construct.rs）を外すと本テストが失敗する。
#[test]
fn test_vertical_bending_stiffness_uses_section_strong_axis() {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::ForceRegime;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    // H-400x200 相当の非対称断面（iy=強軸 ≫ iz=弱軸、as_z=ウェブ、as_y=フランジ）
    let sec = Section {
        id: SectionId(0),
        name: "H-400x200".into(),
        area: 8_412.0,
        iy: 2.37e8,
        iz: 1.60e7,
        j: 5.0e5,
        depth: 400.0,
        width: 200.0,
        as_y: 5_200.0,
        as_z: 3_200.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
    };
    let model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [6000.0, 0.0, 0.0]),
        ],
        elements: vec![elem.clone()],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };

    let beam = BeamElement::new(&elem, &model);
    // 水平梁（ref=[0,0,1]）: ey=+Z（鉛直上向き）を確認
    assert!((beam.axis.rot[1][2] - 1.0).abs() < 1e-12);

    let k = beam.local_stiffness_raw();
    let (e, g, l) = (beam.e, beam.g, beam.length);

    // 鉛直たわみ（uy、DOF1）: 断面の強軸 iy とウェブ as_z が支配する
    let phi_v = 12.0 * e * sec.iy / (g * sec.as_z * l * l);
    let expected_v = 12.0 * e * sec.iy / ((1.0 + phi_v) * l.powi(3));
    assert!(
        (k.get(1, 1) - expected_v).abs() / expected_v < 1e-12,
        "鉛直たわみ剛性が強軸値で組まれていない: k11={} expected={}",
        k.get(1, 1),
        expected_v
    );

    // 水平たわみ（uz、DOF2）: 断面の弱軸 iz とフランジ as_y が支配する
    let phi_h = 12.0 * e * sec.iz / (g * sec.as_y * l * l);
    let expected_h = 12.0 * e * sec.iz / ((1.0 + phi_h) * l.powi(3));
    assert!(
        (k.get(2, 2) - expected_h).abs() / expected_h < 1e-12,
        "水平たわみ剛性が弱軸値で組まれていない: k22={} expected={}",
        k.get(2, 2),
        expected_h
    );

    // 鉛直曲げ剛性 > 水平曲げ剛性（強軸 ≫ 弱軸）であること
    assert!(k.get(1, 1) > k.get(2, 2) * 5.0);
}

/// トライアル追従の回帰テスト: update_state(du, commit=false) が internal_force に
/// 反映され、commit_state で確定、revert_state / restore_state でロールバック
/// できること。旧実装は commit=false の du を捨てており、非線形ドライバの規律
/// 「反復中 update_state(du,false) → 収束時 commit_state()」では弾性要素の内力が
/// 一切更新されなかった（Newton 収束の劣化と、弾性要素が復元力を負担しない
/// 誤った釣合いの原因）。
#[test]
fn test_beam_trial_displacement_tracking() {
    use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
    let mut beam = make_test_beam();
    let model = Model::default();
    let ctx = Ctx { model: &model };
    let state = ElemState::default();

    // 初期状態: 内力ゼロ
    assert!(beam
        .internal_force(&state, &ctx)
        .data
        .iter()
        .all(|v| v.abs() < 1e-12));

    let mut du = LocalVec {
        data: smallvec::SmallVec::from_elem(0.0, 12),
    };
    du.data[6] = 1.0; // j端 軸方向 1mm
    let snap = beam.snapshot_state();
    beam.update_state(&du, false, &ctx);

    // commit 前でも内力へ反映される（トライアル追従）
    let f = beam.internal_force(&state, &ctx);
    let ea_over_l = beam.e * beam.a / beam.length;
    assert!(
        (f.data[6] - ea_over_l).abs() / ea_over_l < 1e-12,
        "f6={} expected EA/L={}",
        f.data[6],
        ea_over_l
    );

    // commit_state で確定
    beam.commit_state();
    assert!((beam.committed_disp[6] - 1.0).abs() < 1e-15);

    // さらに反復 → revert_state で確定値へ戻る
    beam.update_state(&du, false, &ctx);
    assert!((beam.trial_disp[6] - 2.0).abs() < 1e-15);
    beam.revert_state();
    assert!((beam.trial_disp[6] - 1.0).abs() < 1e-15);

    // restore_state でスナップショット時点（初期状態）へ完全ロールバック
    beam.restore_state(&*snap);
    assert!(beam
        .internal_force(&state, &ctx)
        .data
        .iter()
        .all(|v| v.abs() < 1e-12));
    assert!(beam.committed_disp.iter().all(|v| *v == 0.0));
    assert!(beam.trial_disp.iter().all(|v| *v == 0.0));
}

/// recover_forces の内力場が i/j 分岐（ξ=0.5）をまたいで連続・整合であること。
/// スパン荷重なしでは N/Qy/Qz/Mx は全断面で一定、Mz/My は
/// dMz/dx = Qy・dMy/dx = −Qz を満たす単一の線形場になる。
/// （旧実装は i 端側分岐で節点モーメントの符号を反転せず出力しており、
/// 端部モーメント非ゼロの部材で M 図が ξ=0.5 でジャンプしていた。）
#[test]
fn test_recover_forces_moment_field_continuous_across_half() {
    let mut beam = make_test_beam();
    beam.j = 1.0e8; // ねじり剛性を与えて Mx も検証する
    beam.eval_sections = vec![0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0];
    // 全自由度を励起する任意の端部変位（局所=グローバルの恒等軸）
    let u = [
        0.1, 2.0, -1.5, 0.004, 0.002, -0.003, //
        -0.2, -1.0, 0.5, -0.002, 0.004, 0.001,
    ];
    let mf = beam.recover_forces(&u);
    let l = beam.length;
    let f0 = mf.at.first().unwrap().1;
    for &(xi, f) in &mf.at {
        // N・Qy・Qz・Mx は一定
        for (k, name) in [(0, "N"), (1, "Qy"), (2, "Qz"), (3, "Mx")] {
            let tol = 1e-6 * f0[k].abs().max(1.0);
            assert!(
                (f[k] - f0[k]).abs() < tol,
                "xi={xi} {name}={} が一定でない (端={})",
                f[k],
                f0[k]
            );
        }
        // Mz(ξ) = Mz(0) + Qy·ξL、My(ξ) = My(0) − Qz·ξL の線形場
        let mz_expected = f0[5] + f0[1] * xi * l;
        let my_expected = f0[4] - f0[2] * xi * l;
        let tol_mz = 1e-6 * mz_expected.abs().max(1.0);
        let tol_my = 1e-6 * my_expected.abs().max(1.0);
        assert!(
            (f[5] - mz_expected).abs() < tol_mz,
            "xi={xi} Mz={} expected={mz_expected}",
            f[5]
        );
        assert!(
            (f[4] - my_expected).abs() < tol_my,
            "xi={xi} My={} expected={my_expected}",
            f[4]
        );
    }
}

/// 純曲げ（両端逆向き回転 θ, −θ・並進ゼロ）では Qy=0 で Mz が全断面一定になる。
/// たわみ形 v=θx(1−x/L) は v''=−2θ/L（上に凸＝上端引張）で、下端引張正の
/// 断面力規約では Mz = EI·v'' = −2EIθ/L（負）となる符号まで検証する。
#[test]
fn test_recover_forces_pure_bending_constant_negative_moment() {
    let mut beam = make_test_beam();
    beam.eval_sections = vec![0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0];
    let theta = 1.0e-3;
    let mut u = [0.0; 12];
    u[5] = theta; // rz_i
    u[11] = -theta; // rz_j
    let mf = beam.recover_forces(&u);
    let expected = -2.0 * beam.e * beam.iz * theta / beam.length;
    for &(xi, f) in &mf.at {
        assert!(
            (f[5] - expected).abs() < expected.abs() * 1e-6,
            "xi={xi} Mz={} expected={expected}",
            f[5]
        );
        assert!(f[1].abs() < 1e-6, "xi={xi} 純曲げで Qy={} が生じた", f[1]);
    }
}
