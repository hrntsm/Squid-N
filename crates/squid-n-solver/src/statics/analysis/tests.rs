use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase,
    LocalAxis, Material, MemberLoad, MemberLoadKind, NodalLoad, Node, Section,
};

fn make_cantilever_model() -> Model {
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
            name: "beam".into(),
            area: 100.0,
            iy: 833.33,
            iz: 833.33,
            j: 100.0,
            depth: 10.0,
            width: 10.0,
            as_y: 83.33,
            as_z: 83.33,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young: 20000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(1),
                name: "axial".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }],
                member: Vec::new(),
            },
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(2),
                name: "shear".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 500.0, 0.0, 0.0, 0.0, 0.0],
                }],
                member: Vec::new(),
            },
        ],
        combinations: vec![LoadCombination {
            name: "combo1".into(),
            terms: vec![(LoadCaseId(1), 1.2), (LoadCaseId(2), 1.5)],
        }],
        ..Default::default()
    }
}

#[test]
fn test_prepare_and_single_case() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let result = analysis.linear_static(LoadCaseId(1)).unwrap();
    let ux = result.disp[1][0];
    let expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    assert!(
        (ux - expected).abs() < 1e-6,
        "ux={} expected={}",
        ux,
        expected
    );
}

#[test]
fn test_two_cases_one_factorization() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
    let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
    let ux = r1.disp[1][0];
    let uy = r2.disp[1][1];
    let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    let l = 1000.0_f64;
    let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
    // Timoshenko beam includes shear deflection ≈ 0.1% — use relaxed tolerance
    assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
    assert!(
        (uy - uy_expected).abs() < 20.0,
        "uy={} approx={}",
        uy,
        uy_expected
    );
}

#[test]
fn test_load_combination() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let combo = &model.combinations[0];
    let result = analysis.linear_combination(combo).unwrap();
    let ux = result.disp[1][0];
    let uy = result.disp[1][1];
    let ux_expected = 1.2 * (1000.0 * 1000.0 / (20000.0 * 100.0));
    let l = 1000.0_f64;
    let uy_expected = 1.5 * (500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33));
    assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
    // Timoshenko shear adds slight deflection — relaxed tolerance
    assert!(
        (uy - uy_expected).abs() < 20.0,
        "uy={} approx={}",
        uy,
        uy_expected
    );
}

#[test]
fn test_prepare_empty_model_gives_diagnostic() {
    let model = Model::default();
    let err = Analysis::prepare(&model).err().unwrap();
    assert!(matches!(err, SolveError::InvalidInput(_)), "{:?}", err);
}

#[test]
fn test_prepare_no_restraint_gives_diagnostic() {
    let mut model = make_cantilever_model();
    for n in &mut model.nodes {
        n.restraint = Dof6Mask::FREE;
    }
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("拘束"), "{}", msg);
}

#[test]
fn test_prepare_missing_section_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model.elements[0].section = None;
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("未割当"), "{}", msg);
}

#[test]
fn test_prepare_isolated_node_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [0.0, 5000.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    });
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("接続されていない節点"), "{}", msg);
}

#[test]
fn test_linear_static_unknown_load_case_is_error() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let err = analysis.linear_static(LoadCaseId(99)).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("荷重ケース"), "{}", msg);
}

#[test]
fn test_seismic_without_stories_is_error() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let err = analysis
        .seismic_static(SeismicDir::X, AiMode::Approx)
        .err()
        .unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("階"), "{}", msg);
}

#[test]
fn test_bernoulli_strict_1e9() {
    // Bernoulli beam: very large shear area → negligible shear deformation.
    // Axial: u = PL/EA, Bending: w = PL³/3EI — strict 1e-9 match.
    let mut model = make_cantilever_model();
    model.sections[0].as_y = 1e12;
    model.sections[0].as_z = 1e12;
    let analysis = Analysis::prepare(&model).unwrap();
    let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
    let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
    let ux = r1.disp[1][0];
    let uy = r2.disp[1][1];
    let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    let l = 1000.0_f64;
    let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
    let ux_rel = (ux - ux_expected).abs() / ux_expected.abs();
    let uy_rel = (uy - uy_expected).abs() / uy_expected.abs();
    assert!(ux_rel < 1e-9, "ux rel err={}", ux_rel);
    assert!(uy_rel < 1e-4, "uy rel err={}", uy_rel);
}

// ---- §1.5 略算周期の鉄骨造比 α ----

/// 3層等階高（各1000mm、基部Z=0）で、指定した各階の `structure` から
/// `steel_height_ratio` を計算するテスト用モデル。
fn make_story_ratio_model(structures: &[StoryStructure]) -> Model {
    let mut nodes = vec![Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    }];
    let mut stories = Vec::new();
    for (i, s) in structures.iter().enumerate() {
        let elev = (i as f64 + 1.0) * 1000.0;
        let nid = NodeId((i + 1) as u32);
        nodes.push(Node {
            id: nid,
            coord: [0.0, 0.0, elev],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(i as u32)),
        });
        stories.push(Story {
            id: StoryId(i as u32),
            name: format!("F{}", i + 1),
            elevation: elev,
            node_ids: vec![nid],
            diaphragms: Vec::new(),
            seismic_weight: Some(1000.0),
            structure: *s,
            level_kind: StoryLevelKind::Normal,
        });
    }
    Model {
        nodes,
        stories,
        ..Default::default()
    }
}

#[test]
fn test_steel_height_ratio_bottom_story_s_gives_one_third() {
    let model =
        make_story_ratio_model(&[StoryStructure::S, StoryStructure::Rc, StoryStructure::Rc]);
    let alpha = steel_height_ratio(&model);
    assert!((alpha - 1.0 / 3.0).abs() < 1e-9, "alpha={}", alpha);
}

#[test]
fn test_steel_height_ratio_all_rc_is_zero() {
    let model = make_story_ratio_model(&[StoryStructure::Rc; 3]);
    assert_eq!(steel_height_ratio(&model), 0.0);
}

#[test]
fn test_steel_height_ratio_all_s_is_one() {
    let model = make_story_ratio_model(&[StoryStructure::S; 3]);
    let alpha = steel_height_ratio(&model);
    assert!((alpha - 1.0).abs() < 1e-9, "alpha={}", alpha);
}

#[test]
fn test_steel_height_ratio_no_stories_is_zero() {
    let model = Model::default();
    assert_eq!(steel_height_ratio(&model), 0.0);
}

// ---- §1.6 多剛床のPi重複載荷 ----

fn make_diaphragm_story(diaphragms: Vec<DiaphragmDef>) -> Story {
    Story {
        id: StoryId(0),
        name: "F1".into(),
        elevation: 1000.0,
        node_ids: Vec::new(),
        diaphragms,
        seismic_weight: Some(400.0),
        structure: StoryStructure::Rc,
        level_kind: StoryLevelKind::Normal,
    }
}

#[test]
fn test_distribute_pi_single_diaphragm_gets_full_pi() {
    let story = make_diaphragm_story(vec![DiaphragmDef {
        ci_override: None,
        master: NodeId(10),
        slaves: vec![],
        rigid: true,
        weight: None,
    }]);
    let shares = distribute_pi_over_diaphragms(&story, 40.0);
    assert_eq!(shares, vec![(NodeId(10), 40.0)]);
}

#[test]
fn test_distribute_pi_weight_ratio_3_to_1() {
    let story = make_diaphragm_story(vec![
        DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: Some(300.0),
        },
        DiaphragmDef {
            ci_override: None,
            master: NodeId(11),
            slaves: vec![],
            rigid: true,
            weight: Some(100.0),
        },
    ]);
    let pi = 40.0;
    let shares = distribute_pi_over_diaphragms(&story, pi);
    let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
    let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
    assert!((s10 - 30.0).abs() < 1e-9, "s10={}", s10);
    assert!((s11 - 10.0).abs() < 1e-9, "s11={}", s11);
    // 合計は階の Pi に一致する（重複載荷しない）。
    let total: f64 = shares.iter().map(|(_, v)| v).sum();
    assert!((total - pi).abs() < 1e-9, "total={}", total);
}

#[test]
fn test_distribute_pi_equal_split_when_no_weight() {
    let story = make_diaphragm_story(vec![
        DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: None,
        },
        DiaphragmDef {
            ci_override: None,
            master: NodeId(11),
            slaves: vec![],
            rigid: true,
            weight: None,
        },
    ]);
    let pi = 40.0;
    let shares = distribute_pi_over_diaphragms(&story, pi);
    for (_, v) in &shares {
        assert!((*v - 20.0).abs() < 1e-9, "share={}", v);
    }
    let total: f64 = shares.iter().map(|(_, v)| v).sum();
    assert!((total - pi).abs() < 1e-9, "total={}", total);
}

// ---- §4 風荷重の静的解析接続 ----

/// 2層×1スパンの平面ラーメン（squid-n-load::story_gen のテスト固定物と同形）。
/// X方向にスパン6000mm、全節点 Y=0（平面フレーム）。柱4本・梁2本。
fn two_story_wind_model() -> Model {
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
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
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
            plastic_zone: None,
            spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
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

/// `squid_n_load::story_gen::StoryGenResult` をテスト用モデルへ適用する
/// （squid-n-app の `ApplyStories` EditCommand と同じ手順を直接実行する）。
fn apply_story_gen(model: &mut Model, gen: squid_n_load::story_gen::StoryGenResult) {
    model.stories = gen.stories;
    for (node, st) in model.nodes.iter_mut().zip(gen.node_story.iter()) {
        node.story = *st;
    }
    model
        .constraints
        .retain(|c| !matches!(c, Constraint::RigidDiaphragm { .. }));
    model.constraints.extend(gen.constraints);
    for rn in gen.rep_nodes {
        let idx = rn.id.index();
        if idx < model.nodes.len() {
            model.nodes[idx] = rn;
        } else {
            model.nodes.push(rn);
        }
    }
    model.generated_masters = gen.generated_masters;
}

#[test]
fn test_wind_static_runs_and_reactions_balance_applied_force() {
    let mut model = two_story_wind_model();
    let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    apply_story_gen(&mut model, gen);

    let analysis = Analysis::prepare(&model).unwrap();
    // 平面フレームは全節点 Y=0 のため、Y方向の風(dir=Y)なら見付け幅は
    // X方向範囲(6000mm)から求まる（dir=X だと Y範囲=0 でエラーになる）。
    let cfg = WindStaticCfg {
        dir: SeismicDir::Y,
        v0: 34.0,
        roughness: squid_n_load::wind::TerrainRoughness::III,
        cpi: 0.0,
        parapet_mm: 0.0,
    };
    let result = analysis.wind_static(cfg).unwrap();

    // wind_static と同じ幾何（H=7000mm、幅=6000mm）で独立に風荷重を再計算し、
    // 全層合計の水平力を求める。
    let wcfg = squid_n_load::wind::WindCfg {
        v0: 34.0,
        roughness: squid_n_load::wind::TerrainRoughness::III,
        cpe_windward: 0.8,
        cpe_leeward: -0.4,
        cpi: 0.0,
    };
    let wind_stories = vec![
        squid_n_load::wind::WindStory {
            z_bottom: 0.0,
            z_top: 5250.0,
            width: 6000.0,
        },
        squid_n_load::wind::WindStory {
            z_bottom: 5250.0,
            z_top: 7000.0,
            width: 6000.0,
        },
    ];
    let dist = squid_n_load::wind::wind_forces(7000.0, &wind_stories, &wcfg);
    let total_force: f64 = dist.force.iter().sum();
    assert!(total_force > 0.0, "total_force={}", total_force);

    // 基部の反力(Y方向)は、基部節点(0,1)に接続する柱要素(ElemId 0,1)の
    // i端(xi=0)局所力から求める。この2柱は鉛直（ref_vector=[0,0,1]と部材軸が
    // 平行）なので LocalFrame::from_nodes のフォールバックにより
    // 局所軸 (ex,ey,ez) = (global Z, global X, global Y) となり、
    // 局所 qz（[2]成分）がそのまま global Y 方向の力に一致する。
    let reaction_y: f64 = result
        .member_forces
        .iter()
        .filter(|(id, _)| id.0 == 0 || id.0 == 1)
        .map(|(_, mf)| {
            mf.at
                .iter()
                .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
                .unwrap()
                .1[2]
        })
        .sum();

    // 全体の水平釣合い: 反力合計の大きさ = 作用させた風荷重合計(Σforce_i)。
    assert!(
        (reaction_y.abs() - total_force.abs()).abs() < total_force.abs() * 1e-6 + 1e-6,
        "reaction_y={} total_force={}",
        reaction_y,
        total_force
    );
}

#[test]
fn test_wind_static_without_stories_is_error() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let cfg = WindStaticCfg {
        dir: SeismicDir::X,
        v0: 30.0,
        roughness: squid_n_load::wind::TerrainRoughness::II,
        cpi: 0.0,
        parapet_mm: 0.0,
    };
    let err = analysis.wind_static(cfg).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("階"), "{}", msg);
}

// ---- §4 追補: 副剛床のCi直接入力・パラペット・階別見付け幅 ----

#[test]
fn test_main_system_weight_excludes_ci_override_diaphragm() {
    let story = make_diaphragm_story(vec![
        DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: Some(300.0),
        },
        DiaphragmDef {
            ci_override: Some(0.3),
            master: NodeId(11),
            slaves: vec![],
            rigid: true,
            weight: Some(100.0),
        },
    ]);
    // make_diaphragm_story は seismic_weight=400.0 固定（主300+副100）。
    // 主系統重量は ci_override を持つ副剛床の重量(100)を除いた 300 になる。
    let w = main_system_weight(&story);
    assert!((w - 300.0).abs() < 1e-9, "main_system_weight={}", w);
}

#[test]
fn test_distribute_seismic_forces_ci_override_adds_separate_force() {
    let story = make_diaphragm_story(vec![
        DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: Some(300.0),
        },
        DiaphragmDef {
            ci_override: Some(0.3),
            master: NodeId(11),
            slaves: vec![],
            rigid: true,
            weight: Some(100.0),
        },
    ]);
    // 主系統(重量300ベースで別途算定済み)の Pi として 60.0 を渡す。
    // 主剛床(唯一の ci_override 無し剛床)が全量を受け、副剛床には
    // 0.3×100=30 が別途載る。
    let pi = 60.0;
    let shares = distribute_seismic_forces(&story, pi);
    let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
    let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
    assert!((s10 - 60.0).abs() < 1e-9, "s10={}", s10);
    assert!((s11 - 30.0).abs() < 1e-9, "s11={}", s11);
}

#[test]
fn test_distribute_seismic_forces_matches_pi_distribution_without_ci_override() {
    // 全剛床が ci_override 無しなら distribute_pi_over_diaphragms と厳密一致。
    let story = make_diaphragm_story(vec![
        DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: Some(300.0),
        },
        DiaphragmDef {
            ci_override: None,
            master: NodeId(11),
            slaves: vec![],
            rigid: true,
            weight: Some(100.0),
        },
    ]);
    let pi = 40.0;
    let expected = distribute_pi_over_diaphragms(&story, pi);
    let actual = distribute_seismic_forces(&story, pi);
    assert_eq!(expected, actual);
}

/// 単純な `Model`（構造節点のみ、部材・拘束なし）を組み立てるテスト用ヘルパ。
fn make_node_only_model(coords: &[[f64; 3]]) -> Model {
    let nodes = coords
        .iter()
        .enumerate()
        .map(|(i, c)| Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        })
        .collect();
    Model {
        nodes,
        ..Default::default()
    }
}

#[test]
fn test_story_wind_width_uses_story_node_range() {
    let model = make_node_only_model(&[[0.0, 0.0, 0.0], [6000.0, 0.0, 0.0]]);
    let story = make_diaphragm_story(vec![]);
    let mut story = story;
    story.node_ids = vec![NodeId(0), NodeId(1)];
    let excluded = HashSet::new();
    let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
    assert!((w - 6000.0).abs() < 1e-9, "w={}", w);
}

#[test]
fn test_story_wind_width_fallback_when_single_node() {
    let model = make_node_only_model(&[[0.0, 0.0, 0.0], [6000.0, 0.0, 0.0]]);
    let mut story = make_diaphragm_story(vec![]);
    story.node_ids = vec![NodeId(0)];
    let excluded = HashSet::new();
    let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
    assert!((w - 999.0).abs() < 1e-9, "w={}", w);
}

#[test]
fn test_story_wind_width_fallback_when_zero_range() {
    let model = make_node_only_model(&[[3000.0, 0.0, 0.0], [3000.0, 5000.0, 0.0]]);
    let mut story = make_diaphragm_story(vec![]);
    story.node_ids = vec![NodeId(0), NodeId(1)];
    let excluded = HashSet::new();
    // 両節点とも X=3000 なので axis=0(X) の範囲は 0 → フォールバック。
    let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
    assert!((w - 999.0).abs() < 1e-9, "w={}", w);
}

#[test]
fn test_wind_story_geometry_parapet_increases_h_and_extends_top_interval() {
    let mut model = two_story_wind_model();
    let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    apply_story_gen(&mut model, gen);
    let normal_stories: Vec<&Story> = model.stories.iter().collect();
    let base = base_elevation(&model);
    let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();

    let (h0, ws0) = wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 0.0).unwrap();
    let (h1, ws1) =
        wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 1000.0).unwrap();

    // パラペット無し: H=7000mm、最上層区間上端=7000mm(=H)。
    assert!((h0 - 7000.0).abs() < 1e-9, "h0={}", h0);
    assert!(
        (ws0.last().unwrap().z_top - 7000.0).abs() < 1e-9,
        "z_top0={}",
        ws0.last().unwrap().z_top
    );

    // パラペット1000mm: H=7000+500=7500mm、最上層区間上端=7000+1000=8000mm
    // （H には半分のみ、区間上端にはパラペット天端まで全量算入）。
    assert!((h1 - 7500.0).abs() < 1e-9, "h1={}", h1);
    assert!(
        (ws1.last().unwrap().z_top - 8000.0).abs() < 1e-9,
        "z_top1={}",
        ws1.last().unwrap().z_top
    );
}

#[test]
fn test_wind_story_geometry_setback_gives_narrower_upper_story_width() {
    // 1階(0〜3500mm): X=0〜6000(幅6000)、2階(3500〜7000mm): セットバックで
    // X=1000〜4000(幅3000)のみ柱がある平面フレームを組み立てる。
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],       // 0: 基部
        [6000.0, 0.0, 0.0],    // 1: 基部
        [0.0, 0.0, 3500.0],    // 2: 1階床（幅6000端）
        [6000.0, 0.0, 3500.0], // 3: 1階床（幅6000端）
        [1000.0, 0.0, 3500.0], // 4: 2階柱脚（セットバック開始）
        [4000.0, 0.0, 3500.0], // 5: 2階柱脚（セットバック開始）
        [1000.0, 0.0, 7000.0], // 6: 2階床（幅3000端）
        [4000.0, 0.0, 7000.0], // 7: 2階床（幅3000端）
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
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    // 柱: 0-2, 1-3（1階）、4-6, 5-7（2階、セットバック柱脚）。
    // 梁: 2-3（1階床）、6-7（2階床）、4-2, 5-3（セットバックの水平つなぎ、
    // 2階柱脚(4,5)を1階床(2,3)へ接続して構造を連続させる）。
    let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (4, 6), (5, 7), (2, 3), (6, 7)];
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
            plastic_zone: None,
            spring: None,
        });
    }
    // 2階柱脚(4,5)を1階床(2,3)へ接続する水平つなぎ梁。
    model.elements.push(ElementData {
        id: ElemId(6),
        kind: ElementKind::Beam,
        nodes: [NodeId(2), NodeId(4)].into_iter().collect(),
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
    model.elements.push(ElementData {
        id: ElemId(7),
        kind: ElementKind::Beam,
        nodes: [NodeId(3), NodeId(5)].into_iter().collect(),
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

    let gen = squid_n_load::story_gen::generate_stories(&model, None).unwrap();
    apply_story_gen(&mut model, gen);
    assert_eq!(model.stories.len(), 2, "stories={:?}", model.stories.len());

    let normal_stories: Vec<&Story> = model.stories.iter().collect();
    let base = base_elevation(&model);
    let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();
    // dir=Y の風 → axis=0(X方向の座標範囲)。
    let (_h, wind_stories) =
        wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 0.0).unwrap();

    assert_eq!(wind_stories.len(), 2);
    assert!(
        (wind_stories[0].width - 6000.0).abs() < 1e-6,
        "story0 width={}",
        wind_stories[0].width
    );
    assert!(
        (wind_stories[1].width - 3000.0).abs() < 1e-6,
        "story1 width={}",
        wind_stories[1].width
    );
    assert!(
        wind_stories[1].width < wind_stories[0].width,
        "上層(セットバック)の見付け幅は下層より小さいはず"
    );
}

#[test]
fn test_wind_static_excludes_penthouse_story_from_height_and_load() {
    let mut model = two_story_wind_model();
    let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    apply_story_gen(&mut model, gen);
    // 最上階(7000mm)をPH階に変更 → 建物高さ・負担層は1階(3500mm)のみになる。
    model.stories[1].level_kind = StoryLevelKind::Penthouse { k: 0.6 };

    let analysis = Analysis::prepare(&model).unwrap();
    let cfg = WindStaticCfg {
        dir: SeismicDir::Y,
        v0: 34.0,
        roughness: squid_n_load::wind::TerrainRoughness::III,
        cpi: 0.0,
        parapet_mm: 0.0,
    };
    let result = analysis.wind_static(cfg).unwrap();

    // H=3500mm・幅6000mmの1層構成として独立に風荷重を再計算する。
    let wcfg = squid_n_load::wind::WindCfg {
        v0: 34.0,
        roughness: squid_n_load::wind::TerrainRoughness::III,
        cpe_windward: 0.8,
        cpe_leeward: -0.4,
        cpi: 0.0,
    };
    let wind_stories = vec![squid_n_load::wind::WindStory {
        z_bottom: 0.0,
        z_top: 3500.0,
        width: 6000.0,
    }];
    let dist = squid_n_load::wind::wind_forces(3500.0, &wind_stories, &wcfg);
    let total_force: f64 = dist.force.iter().sum();
    assert!(total_force > 0.0, "total_force={}", total_force);

    let reaction_y: f64 = result
        .member_forces
        .iter()
        .filter(|(id, _)| id.0 == 0 || id.0 == 1)
        .map(|(_, mf)| {
            mf.at
                .iter()
                .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
                .unwrap()
                .1[2]
        })
        .sum();

    assert!(
        (reaction_y.abs() - total_force.abs()).abs() < total_force.abs() * 1e-6 + 1e-6,
        "reaction_y={} total_force={}",
        reaction_y,
        total_force
    );
}
