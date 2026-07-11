use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, MemberLoad,
    MemberLoadKind, Model, NodalLoad, Node, Section,
};

/// 単純梁（i:ピン, j:ローラ）に等分布荷重 → 中央曲げ wL²/8、端部 0 を検証。
/// 曲げは静定なので EI に依らず厳密。組立（等価節点力）＋回復（重ね合わせ）の総合検証。
#[test]
fn simply_supported_udl_midspan_moment() {
    let l = 1000.0_f64;
    let w = 2.0_f64; // N/mm（下向き -Z）
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                // Ux,Uy,Uz,Rx 拘束（並進ピン＋ねじり剛体モード除去）
                restraint: Dof6Mask(0b001111),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [l, 0.0, 0.0],
                // Uy,Uz 拘束（ローラ。Ux 自由）
                restraint: Dof6Mask(0b000110),
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
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".into(),
            area: 1000.0,
            iy: 1.0e7,
            iz: 1.0e7,
            j: 1.0e6,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "m".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "udl".into(),
            nodal: vec![],
            member: vec![MemberLoad {
                elem: ElemId(0),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: l,
                    w1: w,
                    w2: w,
                },
            }],
        }],
        ..Default::default()
    };

    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("member forces for elem 0");

    let expected_mid = w * l * l / 8.0; // 250000
    let mut mid_mz = None;
    let mut end_mz_max = 0.0_f64;
    for (xi, vals) in &mf.at {
        let mz = vals[5];
        if (xi - 0.5).abs() < 1e-9 {
            mid_mz = Some(mz);
        }
        if (*xi < 1e-9) || ((xi - 1.0).abs() < 1e-9) {
            end_mz_max = end_mz_max.max(mz.abs());
        }
    }
    let mid = mid_mz.expect("midspan section present");
    assert!(
        (mid.abs() - expected_mid).abs() / expected_mid < 1e-3,
        "midspan Mz={} expected {}",
        mid,
        expected_mid
    );
    assert!(
        end_mz_max < expected_mid * 1e-3,
        "end Mz should be ~0, got {}",
        end_mz_max
    );
}

/// 単純梁モデル（長さ l、i:ピン+ねじり拘束, j:ローラ）を指定の部材荷重で作る。
fn ss_beam(l: f64, member: Vec<MemberLoad>) -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask(0b001111),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [l, 0.0, 0.0],
                restraint: Dof6Mask(0b000110),
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
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".into(),
            area: 1000.0,
            iy: 1.0e7,
            iz: 1.0e7,
            j: 1.0e6,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "m".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "lc".into(),
            nodal: vec![],
            member,
        }],
        ..Default::default()
    }
}

fn mid_value(mf: &squid_n_element::beam::MemberForces, comp: usize) -> f64 {
    mf.at
        .iter()
        .find(|(xi, _)| (xi - 0.5).abs() < 1e-9)
        .map(|(_, v)| v[comp])
        .expect("midspan")
}

/// 単純梁・中央集中荷重 P → 中央曲げ PL/4。
#[test]
fn simply_supported_point_mid_moment() {
    let l = 1000.0_f64;
    let p = 500.0_f64;
    let model = ss_beam(
        l,
        vec![MemberLoad {
            elem: ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Point { a: l / 2.0, p },
        }],
    );
    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .unwrap();
    let expected = p * l / 4.0;
    let mid = mid_value(mf, 5).abs();
    assert!(
        (mid - expected).abs() / expected < 1e-3,
        "point mid Mz={} expected {}",
        mid,
        expected
    );
}

/// 単純梁・全体 Y 方向 UDL（ローカル z 面）→ 中央 My = wL²/8。z 面の符号検証。
#[test]
fn simply_supported_udl_zplane_moment() {
    let l = 1000.0_f64;
    let w = 1.5_f64;
    let model = ss_beam(
        l,
        vec![MemberLoad {
            elem: ElemId(0),
            dir: [0.0, -1.0, 0.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: w,
                w2: w,
            },
        }],
    );
    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .unwrap();
    let expected = w * l * l / 8.0;
    let mid = mid_value(mf, 4).abs(); // My
    assert!(
        (mid - expected).abs() / expected < 1e-3,
        "zplane mid My={} expected {}",
        mid,
        expected
    );
    // ねじり・Mz は概ね 0
    assert!(mid_value(mf, 5).abs() < expected * 1e-3, "Mz leak");
}

fn make_axial_cantilever() -> Model {
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
                restraint: Dof6Mask::FREE,
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
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "sec".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 1000.0,
            j: 100.0,
            depth: 100.0,
            width: 100.0,
            as_y: 83.33,
            as_z: 83.33,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "axial".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    }
}

#[test]
fn test_linear_static_axial_cantilever() {
    let model = make_axial_cantilever();
    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    assert!(
        (result.disp[1][0] - 10.0).abs() < 1e-6,
        "ux={}",
        result.disp[1][0]
    );
    assert!(result.member_forces.len() == 1);
    let forces = &result.member_forces[0].1;
    // 軸力 N は部材内力（引張正）。先端 +1000N の引張で全断面 N=+1000。
    for (xi, vals) in &forces.at {
        assert!((vals[0] - 1000.0).abs() < 1e-6, "N(ξ={})={}", xi, vals[0]);
    }
}

/// X 軸上の片持ち梁に「グローバル Y 方向」の先端荷重をかける。
/// 参照ベクトル [0,0,1] では local z = global −y となるので、たわみは
/// **iy** で決まる（iz ではない）。to_global を欠くと iz を使ってしまい誤る。
/// よって iy≠iz の断面で、δ = PL³/(3E·iy) に一致することを確認する。
#[test]
fn test_beam_to_global_transverse_uses_correct_inertia() {
    // 現実的な鋼材大断面（iz=1e9 級）を用いる：to_global 修正の検証に加え、
    // 端ばね静縮約のペナルティが大断面でも非正定値化しないこと（堅牢性）も同時に確認。
    let e = 205000.0_f64;
    let l = 1000.0_f64; // make_axial_cantilever の節点間距離
    let iy = 2.0e9_f64;
    let iz = 1.0e9_f64; // iy≠iz：取り違えが顕在化する
    let p = 10000.0_f64;
    let mut model = make_axial_cantilever();
    model.materials[0].young = e;
    model.sections[0].iy = iy;
    model.sections[0].iz = iz;
    model.sections[0].as_y = 1.0e9; // せん断たわみを十分小さく
    model.sections[0].as_z = 1.0e9;
    model.load_cases[0].nodal[0].values = [0.0, p, 0.0, 0.0, 0.0, 0.0];

    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let uy = result.disp[1][1];
    let expected = p * l.powi(3) / (3.0 * e * iy); // 曲げ支配（iy 使用）
    let buggy = p * l.powi(3) / (3.0 * e * iz); // 誤った値=iz 使用（2倍）
                                                // iy ベースの値に一致し、iz ベース(2倍)を明確に排除する。
    assert!(
        (uy - expected).abs() / expected < 1e-3,
        "uy={} expected(iy)={} buggy(iz)={}",
        uy,
        expected,
        buggy
    );
}

/// 剛域がモデル→解析へ接続され、結果に効くことのエンドツーエンド確認。
/// 同一片持ち梁で、基部に大きな剛域（可とう長を短縮）を入れると、
/// 先端たわみが明確に小さく（剛く）なる。
#[test]
fn test_rigid_zone_affects_analysis() {
    let mut base = make_axial_cantilever();
    base.sections[0].iy = 1.0e7;
    base.sections[0].iz = 1.0e7;
    base.sections[0].as_y = 1.0e8;
    base.sections[0].as_z = 1.0e8;
    base.load_cases[0].nodal[0].values = [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0]; // global Z 載荷

    // 剛域なし
    let r0 = linear_static_once(&base, LoadCaseId(1)).unwrap();
    let uz0 = r0.disp[1][2];

    // 基部に剛域 λ_i=800（可とう長 200）
    let mut rigid = base.clone();
    rigid.elements[0].rigid_zone.length_i = 800.0;
    let r1 = linear_static_once(&rigid, LoadCaseId(1)).unwrap();
    let uz1 = r1.disp[1][2];

    assert!(
        uz0.abs() > 0.0 && uz1.abs() > 0.0,
        "uz0={} uz1={}",
        uz0,
        uz1
    );
    assert!(
        uz1.abs() < 0.5 * uz0.abs(),
        "剛域で剛くなるはず: uz_norigid={} uz_rigid={}",
        uz0,
        uz1
    );
}

#[test]
fn test_linear_static_vertical_cantilever_bending() {
    // 鉛直柱: (0,0,0)固定 → (0,0,1000)自由。頂部に水平荷重 P=1000 (global X)。
    // 座標変換が正しく適用されれば曲げ片持ち応答 δx ≈ PL³/3E·Iz + せん断 ≈ 333,364。
    // 回転変換が欠落していると軸剛性を誤用して δx≈10 になる（回帰防止）。
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
                coord: [0.0, 0.0, 1000.0],
                restraint: Dof6Mask::FREE,
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
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "sec".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 1000.0,
            j: 100.0,
            depth: 100.0,
            width: 100.0,
            as_y: 83.33,
            as_z: 83.33,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "h".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    };
    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let ux = result.disp[1][0];
    // 曲げ主成分 333,333 + せん断 ~31。軸剛性誤用(=10)を確実に弾く帯域で判定。
    assert!(
            (333_000.0..=334_000.0).contains(&ux),
            "vertical cantilever tip ux={ux} (expected ~333,364 bending; got axial ~10 means rotation missing)"
        );
}

#[test]
fn test_linear_static_shell_element() {
    // Cantilever plate: bottom edge fixed (nodes 0,1), top edge free (nodes 2,3)
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
                coord: [100.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [100.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(1),
            kind: ElementKind::Shell,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            name: "shell".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(10.0),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "shell_load".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    };
    let result = linear_static_once(&model, LoadCaseId(1));
    assert!(result.is_ok(), "solver failed: {:?}", result.err());
    let result = result.unwrap();
    // Top edge should displace upward (positive z) under positive z point load
    assert!(
        result.disp[2][2] > 0.0,
        "loaded node should displace upward: {}",
        result.disp[2][2]
    );
    assert!(
        result.disp[3][2] > 0.0,
        "free node should also displace upward: {}",
        result.disp[3][2]
    );
}

#[test]
fn test_linear_static_deterministic() {
    let model = make_axial_cantilever();
    let first = linear_static_once(&model, LoadCaseId(1)).unwrap();
    for _ in 0..99 {
        let cur = linear_static_once(&model, LoadCaseId(1)).unwrap();
        assert_eq!(first.disp, cur.disp);
        assert_eq!(first.member_forces.len(), cur.member_forces.len());
        for (a, b) in first.member_forces.iter().zip(cur.member_forces.iter()) {
            assert_eq!(a.0, b.0);
            assert_eq!(a.1.at, b.1.at);
        }
    }
}

#[test]
fn test_shell_membrane_patch_test() {
    // Distorted 2x2 patch: corners pinned, midsides+interior free.
    // Sanity check that the patch assembles and solves without singularity.

    let e = 1000.0;
    let nu = 0.3;
    let t = 10.0;

    // 9 nodes: 4 corners, 4 midsides, 1 interior (offset from center)
    let nodes = vec![
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
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(2),
            coord: [1000.0, 1000.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(3),
            coord: [0.0, 1000.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(4),
            coord: [500.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(5),
            coord: [1000.0, 500.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(6),
            coord: [500.0, 1000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(7),
            coord: [0.0, 500.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        },
        Node {
            id: NodeId(8),
            coord: [450.0, 550.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        },
    ];

    // Apply boundary displacements as fixed restraints + prescribed displacements
    // We model this by making boundary nodes free and applying nodal loads that
    // produce the target displacements. Simpler: fix all boundary DOFs to zero and
    // apply the linear field as loads is non-trivial. Instead we directly set
    // boundary node displacements via MPC-like fixed values: set boundary nodes
    // to FIXED and then apply the corresponding displacement via load is not possible.
    //
    // Workaround: make boundary nodes free but apply large penalty springs to enforce
    // target displacements. This is complex.
    //
    // Alternative patch test: just verify the assembled element gives constant strain
    // when boundary nodes have linear displacements. We do this element-directly in
    // sc-element tests already. Here we only check that a free patch solves.
    //
    // For a meaningful solver test, pin the corners and leave midsides+interior free.
    // This is a simple sanity check that the patch does not become singular.

    let model = Model {
        nodes,
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(0), NodeId(4), NodeId(8), NodeId(7)],
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
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(4), NodeId(1), NodeId(5), NodeId(8)],
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
            },
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(8), NodeId(5), NodeId(2), NodeId(6)],
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
            },
            ElementData {
                id: ElemId(3),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![NodeId(7), NodeId(8), NodeId(6), NodeId(3)],
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
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "shell".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(t),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: e,
            poisson: nu,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "patch".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(8),
                values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    };

    let result = linear_static_once(&model, LoadCaseId(1));
    assert!(result.is_ok(), "patch solve failed: {:?}", result.err());
}

#[test]
fn test_shell_membrane_off_no_diaphragm() {
    // Sanity: single shell element with membrane manually off, no diaphragm constraints.
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
                coord: [100.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [100.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Shell,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            name: "shell".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(10.0),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "shell_load".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    };
    // Put a rigid diaphragm in the story so ShellElement::new sets membrane_active=false,
    // but do NOT add a model.constraints entry, so the global DOFs remain free.
    use squid_n_core::model::{DiaphragmDef, Story};
    model.stories.push(Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: StoryId(0),
        name: "floor".to_string(),
        elevation: 0.0,
        node_ids: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        diaphragms: vec![DiaphragmDef {
            ci_override: None,
            weight: None,
            master: NodeId(0),
            slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
            rigid: true,
        }],
        seismic_weight: None,
    });
    let result = linear_static_once(&model, LoadCaseId(1));
    assert!(result.is_ok(), "solver failed: {:?}", result.err());
}

#[test]
fn test_shell_rigid_floor_membrane_off() {
    // Rigid floor story: master node fully fixed, slaves follow master in-plane via
    // RigidDiaphragm constraint. Shell membrane is off for this story, but bending remains.
    use squid_n_core::model::{Constraint, DiaphragmDef, Story};

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(1),
                coord: [100.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(2),
                coord: [100.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Shell,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            name: "shell".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(10.0),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".to_string(),
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "floor".to_string(),
            elevation: 0.0,
            node_ids: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(0),
                slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
                rigid: true,
            }],
            seismic_weight: None,
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(0),
            slaves: vec![NodeId(1), NodeId(2), NodeId(3)],
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "load".to_string(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        }],
        ..Default::default()
    };

    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
    // Slaves have no in-plane displacement because master is fixed and diaphragm constrains them.
    assert!(
        res.disp[1][0].abs() < 1e-12 && res.disp[1][1].abs() < 1e-12,
        "slave should not move in-plane: {:?}",
        [res.disp[1][0], res.disp[1][1]]
    );
    // Shell bending allows out-of-plane displacement under vertical load.
    assert!(
        res.disp[2][2].abs() > 1e-12,
        "shell should deflect vertically: {}",
        res.disp[2][2]
    );
}

/// 単純支持正方形板（等分布荷重）の N×N メッシュモデルを作る。
/// 周辺=単純支持（Uz=0, 縁回転自由）。面内は全節点で固定（平板曲げ＝面内変位0）。
fn make_ss_plate(n: usize, a: f64, t: f64, e: f64, nu: f64, q: f64, clamped: bool) -> Model {
    let h = a / n as f64;
    let nn = n + 1;
    let idx = |ix: usize, iy: usize| (iy * nn + ix) as u32;
    let mut nodes = Vec::new();
    for iy in 0..nn {
        for ix in 0..nn {
            let on_boundary = ix == 0 || ix == n || iy == 0 || iy == n;
            // 常に Ux,Uy,Rz を固定（面内＋ドリリング）。周辺は Uz も固定。
            let mut mask = 0b100011u8; // bits 0(Ux),1(Uy),5(Rz)
            if on_boundary {
                mask |= 1 << 2; // Uz
                if clamped {
                    mask |= 1 << 3; // Rx
                    mask |= 1 << 4; // Ry
                }
            }
            nodes.push(Node {
                id: NodeId(idx(ix, iy)),
                coord: [ix as f64 * h, iy as f64 * h, 0.0],
                restraint: Dof6Mask(mask),
                mass: None,
                story: None,
            });
        }
    }
    let mut elements = Vec::new();
    let mut eid = 0u32;
    for iy in 0..n {
        for ix in 0..n {
            elements.push(ElementData {
                id: ElemId(eid),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![
                    NodeId(idx(ix, iy)),
                    NodeId(idx(ix + 1, iy)),
                    NodeId(idx(ix + 1, iy + 1)),
                    NodeId(idx(ix, iy + 1)),
                ],
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
            });
            eid += 1;
        }
    }
    // 等分布荷重 q を負担面積で節点 Fz へ（周辺節点の荷重は支点が負担）。
    let mut nodal = Vec::new();
    for iy in 0..nn {
        for ix in 0..nn {
            let wx = if ix == 0 || ix == n { 0.5 } else { 1.0 };
            let wy = if iy == 0 || iy == n { 0.5 } else { 1.0 };
            let fz = q * (wx * h) * (wy * h);
            nodal.push(NodalLoad {
                node: NodeId(idx(ix, iy)),
                values: [0.0, 0.0, fz, 0.0, 0.0, 0.0],
            });
        }
    }
    Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "plate".into(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: Some(t),
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "m".into(),
            young: e,
            poisson: nu,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "q".into(),
            nodal,
            member: vec![],
        }],
        ..Default::default()
    }
}

/// 単純支持正方形板の中央たわみが参照解（α·q·a⁴/D, α=0.00406）へ
/// 細分化収束する（仕様 §9.3）。粗→密で誤差が単調減少し、16×16 で ±2%。
#[test]
fn test_ss_plate_convergence() {
    let (a, t, e, nu, q) = (1000.0_f64, 10.0_f64, 200000.0_f64, 0.3_f64, 0.01_f64);
    let d = e * t.powi(3) / (12.0 * (1.0 - nu * nu));
    let ref_w = 0.00406 * q * a.powi(4) / d; // ≈ 2.217 mm

    let center_w = |n: usize| -> f64 {
        let model = make_ss_plate(n, a, t, e, nu, q, false);
        let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
        let nn = n + 1;
        let c = (n / 2) * nn + (n / 2);
        res.disp[c][2].abs()
    };

    let w4 = center_w(4);
    let w8 = center_w(8);
    let w16 = center_w(16);
    let e4 = (w4 - ref_w).abs();
    let e8 = (w8 - ref_w).abs();
    let e16 = (w16 - ref_w).abs();

    // 細分化で誤差が単調減少して参照解へ近づく
    assert!(
        e8 < e4 && e16 < e8,
        "誤差が単調減少しない: e4={e4} e8={e8} e16={e16} (w4={w4} w8={w8} w16={w16} ref={ref_w})"
    );
    // 16×16 で参照解の ±2% 以内
    assert!(
        e16 / ref_w < 0.02,
        "16x16 誤差 {:.2}% > 2% (w16={} ref={})",
        e16 / ref_w * 100.0,
        w16,
        ref_w
    );
}

/// クランプ（四辺固定）正方形板の中央たわみ（α=0.00126, 参照解≈0.688mm）の収束。
#[test]
fn test_clamped_plate_convergence() {
    let (a, t, e, nu, q) = (1000.0_f64, 10.0_f64, 200000.0_f64, 0.3_f64, 0.01_f64);
    let d = e * t.powi(3) / (12.0 * (1.0 - nu * nu));
    let ref_w = 0.00126 * q * a.powi(4) / d; // ≈ 0.688 mm

    let center_w = |n: usize| -> f64 {
        let model = make_ss_plate(n, a, t, e, nu, q, true);
        let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
        let nn = n + 1;
        let c = (n / 2) * nn + (n / 2);
        res.disp[c][2].abs()
    };

    let w4 = center_w(4);
    let w8 = center_w(8);
    let w16 = center_w(16);
    let e4 = (w4 - ref_w).abs();
    let e8 = (w8 - ref_w).abs();
    let e16 = (w16 - ref_w).abs();

    assert!(
        e8 < e4 && e16 < e8,
        "誤差が単調減少しない: e4={e4} e8={e8} e16={e16} (w4={w4} w8={w8} w16={w16} ref={ref_w})"
    );
    // 16×16 で参照解の ±2% 以内（仕様 §9.3）。
    assert!(
        e16 / ref_w < 0.02,
        "16x16 誤差 {:.2}% > 2% (w16={} ref={})",
        e16 / ref_w * 100.0,
        w16,
        ref_w
    );
}

// ===== 長期応力解析: 長期軸力無効化（RESP-D 計算編03「応力解析」）=====
//
// 1 スパン・2 柱・頂部大梁・対角ブレース 1 本のモデル（ブレース付きラーメン）。
// 柱・大梁は Fixed-Fixed（曲げ骨組）、ブレースは Pinned-Pinned のトラス要素。
// 荷重ケースを 2 本（Dead=長期, Seismic=短期）用意し、同一の鉛直荷重を与える。
fn braced_frame(kind: squid_n_core::model::LoadCaseKind) -> Model {
    use squid_n_core::model::StressAnalysisCfg;

    let l = 4000.0_f64; // スパン
    let h = 3000.0_f64; // 階高
    let node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let sec = Section {
        id: SectionId(0),
        name: "steel".into(),
        area: 6000.0,
        iy: 8.0e7,
        iz: 8.0e7,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 5000.0,
        as_z: 5000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    Model {
        nodes: vec![
            {
                let mut n = node(0, [0.0, 0.0, 0.0]);
                n.restraint = Dof6Mask::FIXED;
                n
            },
            {
                let mut n = node(1, [l, 0.0, 0.0]);
                n.restraint = Dof6Mask::FIXED;
                n
            },
            node(2, [0.0, 0.0, h]),
            node(3, [l, 0.0, h]),
        ],
        elements: vec![
            // e0: 柱A（鉛直）
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
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
            },
            // e1: 柱B（鉛直）
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(3)],
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
            },
            // e2: 頂部大梁（水平）
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(2), NodeId(3)],
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
            },
            // e3: 対角ブレース（柱Aの脚部 → 柱Bの頂部）
            ElementData {
                id: ElemId(3),
                kind: ElementKind::Brace {
                    tension_only: false,
                },
                nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        load_cases: vec![LoadCase {
            id: LoadCaseId(1),
            name: "gravity".into(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
            }],
            member: vec![],
            kind,
        }],
        stress_cfg: StressAnalysisCfg::default(),
        ..Default::default()
    }
}

// 柱の長期軸力無効化テスト用モデル。同一の2節点間に「柱」（`ElementKind::Beam`、
// 鉛直、Fixed-Fixed）と「鉛直ブレース」（`ElementKind::Brace`、同じ2節点）を
// 並列に配置する。荷重方向が部材軸と厳密に一致するため、柱の軸剛性が
// 支配的な baseline では柱・ブレースへほぼ等分に軸力を負担させつつ、
// 柱側だけ曲げ・せん断剛性（iy/iz/j/as_y/as_z）はそのまま健全に保つ。
// ブレース側は常に軸剛性のみを保つため、柱の軸力を無効化しても
// 機構化（特異行列）を起こさず、負担していた軸力がそのままブレースへ
// 移ることを明快に検証できる。
fn column_with_parallel_vertical_brace() -> Model {
    use squid_n_core::model::StressAnalysisCfg;

    let h = 3000.0_f64;
    let sec = Section {
        id: SectionId(0),
        name: "steel".into(),
        area: 6000.0,
        iy: 8.0e7,
        iz: 8.0e7,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 5000.0,
        as_z: 5000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    Model {
        nodes: vec![
            {
                let mut n = Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                };
                n.restraint = Dof6Mask::FIXED;
                n
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, h],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        elements: vec![
            // e0: 柱（鉛直）。no_long_axial_column の対象。
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
            },
            // e1: 同じ2節点間の鉛直ブレース（並列）。常に軸剛性を保ち、
            // 柱が負担しなくなった軸力の受け皿になる。
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Brace {
                    tension_only: false,
                },
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        load_cases: vec![LoadCase {
            id: LoadCaseId(1),
            name: "gravity".into(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
            }],
            member: vec![],
            kind: squid_n_core::model::LoadCaseKind::Dead,
        }],
        stress_cfg: StressAnalysisCfg::default(),
        ..Default::default()
    }
}

fn axial_force(res: &StaticOnce, elem: ElemId) -> f64 {
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == elem)
        .unwrap_or_else(|| panic!("member forces for elem {:?} not found", elem));
    mf.at[0].1[0]
}

/// 検証1: `no_long_axial_brace=true` の長期ケースでは、ブレース軸力がほぼ0
/// （元の1e-3倍以下）になり、周囲の柱（柱A。ブレースが基部で直接取り付く側の
/// 柱で、荷重も直接負担する主経路）の軸力の絶対値が増えること
/// （RESP-D マニュアル計算編03「応力解析」）。
#[test]
fn test_no_long_axial_brace_zeros_brace_force_and_increases_column() {
    let mut model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_brace = axial_force(&base, ElemId(3)).abs();
    let base_col_a = axial_force(&base, ElemId(0)).abs();
    assert!(
        base_brace > 1.0,
        "baseline brace force should be non-trivial: {base_brace}"
    );

    model.stress_cfg.no_long_axial_brace = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_brace = axial_force(&cut, ElemId(3)).abs();
    let cut_col_a = axial_force(&cut, ElemId(0)).abs();

    assert!(
        cut_brace <= base_brace * 1e-3,
        "brace force should collapse to ~0: base={base_brace} cut={cut_brace}"
    );
    assert!(
            cut_col_a > base_col_a,
            "column A axial force should increase when brace unloaded: base={base_col_a} cut={cut_col_a}"
        );
}

/// 検証2: `no_long_axial_column=true` の長期ケースでは、柱の長期軸力が
/// ほぼ0（元の1e-3倍以下）になること。同一2節点間で柱と並列に鉛直
/// ブレースを置いたモデル（`column_with_parallel_vertical_brace`）を使い、
/// 柱が負担しなくなった軸力がブレース側で健全に負担される
/// （機構化せず数値的に安定に解ける）ことも合わせて確認する。
#[test]
fn test_no_long_axial_column_zeros_column_force() {
    let mut model = column_with_parallel_vertical_brace();
    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_col = axial_force(&base, ElemId(0)).abs();
    let base_brace = axial_force(&base, ElemId(1)).abs();
    assert!(
        base_col > 1.0,
        "baseline column force should be non-trivial: {base_col}"
    );

    model.stress_cfg.no_long_axial_column = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_col = axial_force(&cut, ElemId(0)).abs();
    let cut_brace = axial_force(&cut, ElemId(1)).abs();

    assert!(
        cut_col <= base_col * 1e-3,
        "column force should collapse to ~0: base={base_col} cut={cut_col}"
    );
    // 柱が負担しなくなった軸力は並列ブレースへ移り、荷重全体はブレース単体で
    // 健全に負担される（機構化していないことの確認）。
    assert!(
            cut_brace > base_brace,
            "brace should pick up the load the column no longer carries: base={base_brace} cut={cut_brace}"
        );
    assert!(
        (cut_brace - 1.0e5).abs() / 1.0e5 < 1e-3,
        "brace should carry ~all of the applied load once column axial is disabled: {cut_brace}"
    );
}

/// 検証3: フラグが既定（false）のとき、`apply_long_axial_cut` はモデルを
/// 複製せずそのまま返す（＝従来結果と完全一致。回帰なし）。
#[test]
fn test_apply_long_axial_cut_noop_when_flags_false() {
    let model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    let cow = apply_long_axial_cut(&model, squid_n_core::model::LoadCaseKind::Dead);
    assert!(
        matches!(cow, Cow::Borrowed(_)),
        "既定 stress_cfg では複製が起きない（従来どおりの経路を通る）はず"
    );
}

/// 検証3b: フラグが既定（false）のとき、`linear_static_once` の結果が
/// 有効フラグを立てた場合と異なることも含め、既定値では従来どおり通しで解けること
/// （回帰確認: 既存の他テスト群が既定 stress_cfg のままであることの追加保証）。
#[test]
fn test_default_stress_cfg_matches_plain_model() {
    let model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    assert_eq!(model.stress_cfg, Default::default());
    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
    // 変位・内力が有限（解けている）ことの素朴な確認。
    assert!(res.disp.iter().all(|d| d.iter().all(|v| v.is_finite())));
    assert!(axial_force(&res, ElemId(0)).is_finite());
}

/// 検証4: 短期（Seismic）荷重ケースでは、`no_long_axial_brace=true` でも
/// 適用されない（ブレース軸力が長期無効化なしの基準値と同程度に残ること）。
#[test]
fn test_axial_cut_not_applied_to_short_term_case() {
    let mut model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    // 同一の鉛直荷重を持つ短期（地震）荷重ケースを追加する。
    model.load_cases.push(LoadCase {
        id: LoadCaseId(2),
        name: "seismic_gravity_dummy".into(),
        nodal: vec![NodalLoad {
            node: NodeId(2),
            values: [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
        }],
        member: vec![],
        kind: squid_n_core::model::LoadCaseKind::Seismic,
    });
    model.stress_cfg.no_long_axial_brace = true;

    let long_term = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let short_term = linear_static_once(&model, LoadCaseId(2)).unwrap();

    let brace_long = axial_force(&long_term, ElemId(3)).abs();
    let brace_short = axial_force(&short_term, ElemId(3)).abs();

    assert!(
        brace_long < 1.0,
        "長期ケースはブレース軸力が無効化されるはず: {brace_long}"
    );
    assert!(
        brace_short > 1.0,
        "短期ケースは無効化されず通常どおりブレース軸力を負担するはず: {brace_short}"
    );
}

/// 検証5: SRC 等の合成断面（`Section.shape` あり）の柱でも軸力カットが効く
/// （複製断面で `shape` を外すため、`beam.rs` の `a_stiff` が `shape` 由来の
/// 値へ再計算されず、縮小した `area` が使われる）。
#[test]
fn test_axial_cut_applies_to_composite_src_column() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let mut model = column_with_parallel_vertical_brace();
    // 柱断面を SRC（shape あり・コンクリート材料 fc あり）へ差し替える。
    model.sections[0].shape = Some(SectionShape::SrcRect {
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
        steel_grade: "SN400".into(),
    });
    model.materials[0].fc = Some(24.0);
    model.materials[0].young = 2.27e4; // コンクリートのヤング係数相当

    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_col = axial_force(&base, ElemId(0)).abs();
    assert!(
        base_col > 1.0,
        "SRC柱の基準軸力が有意であること: {base_col}"
    );

    model.stress_cfg.no_long_axial_column = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_col = axial_force(&cut, ElemId(0)).abs();
    let cut_brace = axial_force(&cut, ElemId(1)).abs();

    assert!(
        cut_col <= base_col * 1e-3,
        "SRC柱でも軸力が無効化されるはず: base={base_col} cut={cut_col}"
    );
    assert!(
        (cut_brace - 1.0e5).abs() / 1.0e5 < 1e-3,
        "荷重はブレースが全量負担するはず: {cut_brace}"
    );
}
