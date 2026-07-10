use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};

pub struct StaticOnce {
    pub disp: Vec<[f64; 6]>,
    pub member_forces: Vec<(squid_n_core::ids::ElemId, MemberForces)>,
}

pub fn linear_static_once(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        });
    }

    let k_free = assemble_global_k(model, &dofmap);
    let f_free = assemble_global_f(model, &dofmap, lc);

    let reducer = Reducer::build(model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let f_red = reducer.reduce_f(&f_free);
    let n_indep = reducer.n_indep;

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    if n_indep > 0 {
        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; model.nodes.len()];
        for ni in 0..model.nodes.len() {
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    let val = u_free[active as usize];
                    match d {
                        0 => disp[ni][0] = val,
                        1 => disp[ni][1] = val,
                        2 => disp[ni][2] = val,
                        3 => disp[ni][3] = val,
                        4 => disp[ni][4] = val,
                        _ => disp[ni][5] = val,
                    }
                }
            }
        }

        let mut member_forces = Vec::new();
        let _ctx = squid_n_element::behavior::Ctx { model };
        // 解析対象荷重ケースの部材荷重（内力回復の重ね合わせ用）
        let member_loads: &[squid_n_core::model::MemberLoad] = model
            .load_cases
            .iter()
            .find(|l| l.id == lc)
            .map(|l| l.member.as_slice())
            .unwrap_or(&[]);
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let n_gdofs = gdofs.len();
            let mut u_elem = vec![0.0; n_gdofs];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }

            if let Some(mut forces) = behavior.recover_forces(&u_elem) {
                superpose_member_loads(model, elem, member_loads, &mut forces);
                member_forces.push((elem.id, forces));
            }
        }

        Ok(StaticOnce {
            disp,
            member_forces,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        })
    }
}

/// 部材荷重の固定端内力を、`K·u` 由来の回復内力へ各断面で重ね合わせる。
/// 線形重ね合わせ: 実内力 = （等価節点力に対する応答 K·u）＋（両端固定梁のスパン内力）。
fn superpose_member_loads(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    member_loads: &[squid_n_core::model::MemberLoad],
    forces: &mut squid_n_element::beam::MemberForces,
) {
    use squid_n_element::transform::LocalFrame;

    if elem.nodes.len() < 2 {
        return;
    }
    let loads: Vec<squid_n_core::model::MemberLoad> = member_loads
        .iter()
        .filter(|ml| ml.elem == elem.id)
        .cloned()
        .collect();
    if loads.is_empty() {
        return;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    if ni >= model.nodes.len() || nj >= model.nodes.len() {
        return;
    }
    let p_i = model.nodes[ni].coord;
    let p_j = model.nodes[nj].coord;
    let dx = p_j[0] - p_i[0];
    let dy = p_j[1] - p_i[1];
    let dz = p_j[2] - p_i[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
    if length < 1e-9 {
        return;
    }
    let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
    for (xi, vals) in forces.at.iter_mut() {
        let fixed = squid_n_element::member_load::fixed_internal_local(&loads, &frame, length, *xi);
        for k in 0..6 {
            vals[k] += fixed[k];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MemberLoad, MemberLoadKind, Model, NodalLoad, Node, Section,
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
}
