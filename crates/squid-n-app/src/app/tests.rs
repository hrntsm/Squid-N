use super::*;

#[test]
fn test_is_steel() {
    assert!(is_steel("SN400"));
    assert!(is_steel("SS400"));
    assert!(is_steel("SM490"));
    assert!(!is_steel("SD345"));
    assert!(!is_steel(" Concrete"));
}

#[test]
fn test_run_design_check_empty_model() {
    let mut app = App::default();
    app.run_design_check();
    assert!(app.results.is_none() || app.results.as_ref().unwrap().checks.is_empty());
}

/// 一本部材指定（beam_groups）: 2 分割梁のグループ合成値
/// （全長・端部/中央モーメント・せん断スパン代表値）の手計算照合。
#[test]
fn test_beam_group_overrides_combines_members() {
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node, RigidZone,
    };
    use squid_n_element::beam::MemberForces;

    let node = |id: u32, x: f64| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let beam = |id: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(n0));
            v.push(NodeId(n1));
            v
        },
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes: vec![node(0, 0.0), node(1, 3000.0), node(2, 6000.0)],
        elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
        beam_groups: vec![vec![ElemId(0), ElemId(1)]],
        ..Default::default()
    };
    let mf = |rows: Vec<(f64, f64, f64)>| MemberForces {
        at: rows
            .into_iter()
            .map(|(p, q, m)| (p, [0.0, q, 0.0, 0.0, 0.0, m]))
            .collect(),
    };
    let member_forces = vec![
        (
            ElemId(0),
            mf(vec![
                (0.0, 50_000.0, -200.0e6),
                (0.5, 30_000.0, 20.0e6),
                (1.0, 10_000.0, 100.0e6),
            ]),
        ),
        (
            ElemId(1),
            mf(vec![
                (0.0, -10_000.0, 100.0e6),
                (0.5, -30_000.0, 20.0e6),
                (1.0, -50_000.0, -200.0e6),
            ]),
        ),
    ];

    let overrides = beam_group_overrides(&model, &member_forces);
    let ov = overrides.get(&ElemId(0)).expect("グループ所属");
    // 両要素が同じ合成値を共有する。
    assert!(std::rc::Rc::ptr_eq(ov, overrides.get(&ElemId(1)).unwrap()));
    // 全長 = 3000+3000。
    assert!((ov.length - 6000.0).abs() < 1e-9);
    // 端部モーメントは外端（要素0の pos0、要素1の pos1）。
    assert_eq!(ov.end_moments_z, Some((-200.0e6, -200.0e6)));
    // A式: M0 = (50k+50k)・6000/8 = 75e6、Mc_A = 75e6 − 200e6 < 0。
    // B式: グループ中央(3000mm)＝要素0の pos=1.0 の行 → +100e6。
    // 中央モーメント = max(|B|, Mc_A) に B の符号 → +100e6。
    assert!((ov.mid_moment_z.unwrap() - 100.0e6).abs() < 1e-3);
    // せん断スパン代表値: |M| 最大 200e6 の行の (200e6, 50e3)。
    let (m_rep, q_rep) = ov.shear_span.unwrap();
    assert!((m_rep - 200.0e6).abs() < 1e-3);
    assert!((q_rep - 50_000.0).abs() < 1e-6);
    // 剛域なし → 内法長 = 全長。
    assert!((ov.clear_length - 6000.0).abs() < 1e-9);

    // グループ未指定なら空。
    let mut model2 = model;
    model2.beam_groups.clear();
    assert!(beam_group_overrides(&model2, &member_forces).is_empty());
}

/// 剛域自動算定・危険断面フィルタのテスト用モデル。
/// `sample::portal_frame`（対角材を含む変則的な接続）と異なり、
/// 柱(node0-node1)・梁(node1-node2)・柱(node2-node3)が各節点で厳密に直交する
/// 素直なポータルフレーム（柱 H-300x300x10x15・梁 H-400x200x8x13、SN400B）。
/// - node0(柱1脚部)・node3(柱2脚部): 他要素と接続しない → face=0（節点芯のまま）
/// - node1(柱1頭部/梁始端)・node2(梁終端/柱2頭部): 柱・梁が直交 → face>0
fn aligned_portal_frame() -> squid_n_core::model::Model {
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MemberLoad, MemberLoadKind, Model, Node,
    };
    use squid_n_section::shape::SectionShape;

    let mut model = Model::default();

    let coords = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [4000.0, 0.0, 0.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i == 0 || i == 3 {
                squid_n_core::dof::Dof6Mask::FIXED
            } else {
                squid_n_core::dof::Dof6Mask::FREE
            },
            mass: None,
            story: None,
        });
    }

    // RC 造ラーメン（S 造は剛域長 0 となるため、
    // 剛域自動算定の配管検証には RC 断面を用いる）。
    let rebar = squid_n_core::section_shape::RcRebar {
        main_x: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        main_y: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        cover: 40.0,
        shear: squid_n_core::section_shape::ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let col_shape = SectionShape::RcRect {
        b: 300.0,
        d: 300.0,
        rebar: rebar.clone(),
    };
    let beam_shape = SectionShape::RcRect {
        b: 200.0,
        d: 400.0,
        rebar,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱 RC-300x300".into()));
    model
        .sections
        .push(beam_shape.to_section(SectionId(1), "梁 RC-200x400".into()));

    model.materials.push(Material {
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });

    let members = [
        (0u32, 0u32, 1u32, 0u32, [1.0, 0.0, 0.0]),
        (1, 1, 2, 1, [0.0, 0.0, 1.0]),
        (2, 2, 3, 0, [1.0, 0.0, 0.0]),
    ];
    for (id, i, j, sec, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(sec)),
            material: Some(squid_n_core::ids::MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
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
        name: "長期".into(),
        nodal: Vec::new(),
        member: vec![MemberLoad {
            elem: ElemId(1),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 10.0,
                w2: 10.0,
            },
        }],
    });

    model
}

/// 剛域自動算定が解析パイプラインへ接続されていること（設計書 §6.2.1、標準実装）。
/// 解析エントリ(`run_linear_static`)を通す前は既定の 0（未適用）のままだが、
/// 通した後は `apply_rigid_zones_for_analysis` により `elem.rigid_zone` が
/// 自動算定値へ更新される。
#[test]
fn test_run_linear_static_applies_auto_rigid_zones() {
    let mut app = App::default();
    app.load_model(aligned_portal_frame());

    // 適用前は既定の 0（apply_auto_rigid_zones 未実行）。
    assert_eq!(app.model.elements[1].rigid_zone.length_i, 0.0);
    assert_eq!(app.model.elements[1].rigid_zone.face_i, 0.0);

    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 梁(id=1)の i端(node1, 柱と直交)。
    // λ_i = D_orth/2 − D_self/4 = 柱せい/2 − 梁せい/4 = 150 − 100 = 50
    // face_i = D_orth/2 = 柱せい/2 = 150
    let beam = &app.model.elements[1];
    assert!(
        (beam.rigid_zone.length_i - 50.0).abs() < 1e-9,
        "length_i={}",
        beam.rigid_zone.length_i
    );
    assert!(
        (beam.rigid_zone.face_i - 150.0).abs() < 1e-9,
        "face_i={}",
        beam.rigid_zone.face_i
    );

    // 柱(id=0)の j端(node1, 梁と直交)。
    // λ_j = D_orth/2 − D_self/4 = 梁せい/2 − 柱せい/4 = 200 − 75 = 125
    // face_j = D_orth/2 = 梁せい/2 = 200
    let col = &app.model.elements[0];
    assert!(
        (col.rigid_zone.length_j - 125.0).abs() < 1e-9,
        "length_j={}",
        col.rigid_zone.length_j
    );
    assert!(
        (col.rigid_zone.face_j - 200.0).abs() < 1e-9,
        "face_j={}",
        col.rigid_zone.face_j
    );
    // 柱脚(node0)は他要素と接続しないため face_i は 0 のまま。
    assert_eq!(col.rigid_zone.face_i, 0.0);
}

/// `run_design_check` が危険断面位置（§6.2.3、既定は柱フェイスと中央）のみを
/// 検定し、剛域が有る端の節点芯は検定対象外になることを確認する。
/// 剛域が無い端（face=0）では従来どおり節点芯が検定対象に残る。
#[test]
fn test_run_design_check_filters_to_design_positions() {
    let mut app = App::default();
    app.load_model(aligned_portal_frame());
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let checks = &app.results.as_ref().unwrap().checks;
    assert!(!checks.is_empty());

    // 梁(id=1): 両端とも柱と直交(face>0)のため、節点芯 0.0/1.0 は検定対象外。
    let beam_positions: Vec<f64> = checks
        .iter()
        .filter(|(id, _, _)| *id == ElemId(1))
        .map(|(_, pos, _)| *pos)
        .collect();
    assert!(
        !beam_positions.iter().any(|p| *p < 1e-6),
        "梁の節点芯(i端)が検定対象に残っている: {:?}",
        beam_positions
    );
    assert!(
        !beam_positions.iter().any(|p| (*p - 1.0).abs() < 1e-6),
        "梁の節点芯(j端)が検定対象に残っている: {:?}",
        beam_positions
    );
    assert!(
        beam_positions.iter().any(|p| (*p - 0.5).abs() < 1e-6),
        "梁の中央が検定対象から抜けている: {:?}",
        beam_positions
    );

    // 柱(id=0): 脚部(node0)は他要素と接続しない(face_i=0)ため節点芯 0.0 のままが
    // 危険断面位置に一致し、検定対象に残る(従来挙動と一致)。
    // 頭部(node1)は梁と直交(face_j>0)のため節点芯 1.0 は検定対象外になる。
    let col_positions: Vec<f64> = checks
        .iter()
        .filter(|(id, _, _)| *id == ElemId(0))
        .map(|(_, pos, _)| *pos)
        .collect();
    assert!(
        col_positions.iter().any(|p| *p < 1e-6),
        "剛域の無い柱脚(節点芯)が検定対象から抜けている: {:?}",
        col_positions
    );
    assert!(
        !col_positions.iter().any(|p| (*p - 1.0).abs() < 1e-6),
        "柱頭の節点芯が検定対象に残っている: {:?}",
        col_positions
    );
}

/// 部材付帯情報（`MemberDetailAttr`）を持つ部材で設計検定を実行すると、
/// ハンチ端・継手位置の検定結果が `checks` に含まれること
/// （`design_positions` が `Model::member_detail` の追加検定位置を
/// 取り込んでいるかの確認。§6.2.3「位置はユーザが追加・変更可能」）。
#[test]
fn test_run_design_check_includes_member_detail_positions() {
    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};

    let mut model = aligned_portal_frame();
    // 梁(id=1, i端-j端: node1-node2, 全長4000mm)にハンチ(i端)と継手を追加する。
    // i端は柱と直交(face_i=150、自動剛域算定後)のため、ハンチ端位置は
    // (150+700)/4000 = 0.2125、継手位置は 1000/4000 = 0.25 になる。
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(1),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 100.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Site,
        }],
    });

    let mut app = App::default();
    app.load_model(model);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let checks = &app.results.as_ref().unwrap().checks;
    let beam_positions: Vec<f64> = checks
        .iter()
        .filter(|(id, _, _)| *id == ElemId(1))
        .map(|(_, pos, _)| *pos)
        .collect();

    assert!(
        beam_positions.iter().any(|p| (*p - 0.2125).abs() < 1e-6),
        "ハンチ端の検定位置が抜けている: {:?}",
        beam_positions
    );
    assert!(
        beam_positions.iter().any(|p| (*p - 0.25).abs() < 1e-6),
        "継手位置の検定位置が抜けている: {:?}",
        beam_positions
    );
}

#[test]
fn test_staleness_mark_edited_marks_downstream() {
    let mut s = Staleness::default();
    assert!(!s.results_stale);
    s.mark_edited();
    assert!(s.results_stale);
    assert!(s.design_stale);
    let now = SystemTime::now();
    s.last_run = Some(now);
    s.mark_fresh();
    assert!(!s.results_stale);
    assert!(!s.design_stale);
    assert!(s.last_run.is_some());
}

#[test]
fn test_tab_default_is_model() {
    assert_eq!(Tab::Model, Tab::default());
}

#[test]
fn test_seismic_flow_requires_then_uses_stories() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階なし → 明示エラー（サイレントゼロ結果ではない）
    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_some());

    // 階の自動生成 → 地震静的が成功する
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.model.stories.len(), 1);
    assert!(app.model.stories[0].seismic_weight.unwrap() > 0.0);

    // ユーザー荷重ケース0("長期")を先に実行しておく。
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let user_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 地震静的の結果は StaticCaseKey::Seismic(X) に格納され、直前に実行した
    // ユーザーケース0(StaticCaseKey::User)の結果を上書きしない
    // (旧実装ではどちらも LoadCaseId(0) を共有し、後勝ちで上書きされていた)。
    let r = app.results.as_ref().unwrap();
    assert_eq!(
        r.statics.len(),
        2,
        "ユーザーケース0と地震静的Xの結果が両方残っているはず"
    );
    let seismic_disp = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::Seismic(SeismicDir::X))
        .expect("地震静的Xの結果が残っているはず")
        .1
        .disp
        .clone();
    let kept_user_disp = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::User(LoadCaseId(0)))
        .expect("ユーザーケース0の結果が地震静的実行後も残っているはず")
        .1
        .disp
        .clone();
    assert_eq!(
        kept_user_disp, user_disp,
        "ユーザーケース0の結果は地震静的の実行後も変わらないはず（衝突していない）"
    );
    // 柱頭が X 方向へ変位している(地震静的の結果)
    assert!(seismic_disp[2][0].abs() > 1e-3, "{}", seismic_disp[2][0]);

    // ナビゲータでそれぞれのキーを選択すれば current_static が個別に引ける
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
    assert_eq!(app.current_static().unwrap().disp, kept_user_disp);
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::Seismic(SeismicDir::X)));
    assert_eq!(app.current_static().unwrap().disp, seismic_disp);

    // undo で自重(自動)の同期 → 階定義の順に戻る
    // （run_linear_static が自重(自動)ケースの同期を undo 履歴に積むため 2 回）
    app.undo.undo(&mut app.model);
    assert!(app
        .model
        .load_cases
        .iter()
        .all(|lc| lc.name != SELF_WEIGHT_AUTO_LOAD_CASE_NAME));
    app.undo.undo(&mut app.model);
    assert!(app.model.stories.is_empty());
}

/// 剛床代表節点は慣性力重心に自動生成される。再度自動生成しても
/// 既存の代表節点を再利用するため節点数が増えないことを確認する
/// （story_gen + edit の統合: `generate_stories` → `ApplyStories` の往復）。
#[test]
fn test_generate_stories_action_reuses_rep_node_on_regenerate() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n0 = app.model.nodes.len();
    assert!(app.model.generated_masters.is_empty());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let n1 = app.model.nodes.len();
    assert_eq!(n1, n0 + 1, "剛床代表節点が 1 つ新規生成される");
    assert_eq!(app.model.generated_masters.len(), 1);
    let master_after_first = app.model.generated_masters[0];
    assert!(app.model.validate().is_ok());

    // 再生成しても代表節点は再利用され、節点数は増えない。
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(
        app.model.nodes.len(),
        n1,
        "再生成でノード数が増えてはいけない（代表節点の再利用）"
    );
    assert_eq!(app.model.generated_masters, vec![master_after_first]);
    assert!(app.model.validate().is_ok());

    // 固有値解析・地震静的解析が正常に動作する（生成された剛床を含む縮約の統合確認）。
    app.run_eigen(1);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
}

#[test]
fn test_time_history_sample_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(th.history.node_disp.len() > 100);
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
    assert!(th.history.node.is_some());
}

#[test]
fn test_time_history_y_direction_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_dir = ThDir::Y;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(
        th.history.record_dir_y,
        "th_dir=Y なのに代表応答の記録方向が X のままです"
    );
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_build_ground_motion_routes_by_direction() {
    // wave 構築のみを検証する純粋関数のテスト（th_dir=Y でも accel_x 側に
    // 誤って入らないことを確認する）。
    let accel = vec![1.0, 2.0, 3.0];
    let wave_x = App::build_ground_motion(0.01, ThDir::X, accel.clone());
    assert_eq!(wave_x.accel_x, accel);
    assert!(wave_x.accel_y.is_none());

    let wave_y = App::build_ground_motion(0.01, ThDir::Y, accel.clone());
    assert_eq!(wave_y.accel_x, vec![0.0; accel.len()]);
    assert_eq!(wave_y.accel_y, Some(accel.clone()));
}

/// ThDir::Xy: 同一波形を accel_x・accel_y の両方に入れる（簡易仕様）。
#[test]
fn test_build_ground_motion_xy_duplicates_wave() {
    let accel = vec![1.0, 2.0, 3.0];
    let wave = App::build_ground_motion(0.01, ThDir::Xy, accel.clone());
    assert_eq!(wave.accel_x, accel);
    assert_eq!(wave.accel_y, Some(accel));
}

// ===== parse_wave_csv テスト =====

#[test]
fn test_parse_wave_csv_single_column_x_or_y() {
    let content = "10.0\n20.0\n30.0\n";
    let (accel, second) = parse_wave_csv(content, ThDir::X).unwrap();
    assert_eq!(accel, vec![100.0, 200.0, 300.0]); // gal→mm/s²(×10)
    assert!(second.is_none());

    // カンマ区切りなら最後の列を使う（従来仕様）。
    let content_csv = "0.0,10.0\n0.01,20.0\n0.02,30.0\n";
    let (accel, second) = parse_wave_csv(content_csv, ThDir::Y).unwrap();
    assert_eq!(accel, vec![100.0, 200.0, 300.0]);
    assert!(second.is_none());
}

#[test]
fn test_parse_wave_csv_single_column_too_few_points_is_err() {
    assert!(parse_wave_csv("10.0\n", ThDir::X).is_err());
    assert!(parse_wave_csv("", ThDir::X).is_err());
}

#[test]
fn test_parse_wave_csv_xy_two_columns() {
    let content = "10.0,5.0\n20.0,15.0\n30.0,25.0\n";
    let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
    assert_eq!(xs, vec![100.0, 200.0, 300.0]);
    assert_eq!(ys, Some(vec![50.0, 150.0, 250.0]));
}

#[test]
fn test_parse_wave_csv_xy_header_line_is_skipped() {
    // ヘッダ行（数値化不可）は無視され、残りの2行が (X, Y) として読める。
    let content = "x,y\n10.0,5.0\n20.0,15.0\n";
    let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
    assert_eq!(xs, vec![100.0, 200.0]);
    assert_eq!(ys, Some(vec![50.0, 150.0]));
}

#[test]
fn test_parse_wave_csv_xy_insufficient_columns_is_err() {
    let content = "10.0,5.0\n20.0\n30.0,25.0\n";
    let err = parse_wave_csv(content, ThDir::Xy).unwrap_err();
    assert_eq!(err, "X+Y には2列のCSVが必要です");
}

#[test]
fn test_parse_wave_csv_xy_too_few_points_is_err() {
    assert!(parse_wave_csv("10.0,5.0\n", ThDir::Xy).is_err());
}

#[test]
fn test_time_history_xy_sample_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_dir = ThDir::Xy;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

/// 2 層等質量等剛性せん断モデル（軸ばね 2 本の直列、Ux 方向のみ自由）。
/// portal_frame は平面骨組で弱軸・面外方向の縮約後自由度が多く、
/// 固有値解析(部分空間反復)が n_modes=2 で不安定になりやすいため、
/// Rayleigh 減衰(1次・2次固有値が必要)のテストには本モデルを用いる。
fn shear_2dof_model() -> squid_n_core::model::Model {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        Section,
    };
    const FREE_UX: Dof6Mask = Dof6Mask(0b111110);
    let k = 1000.0_f64;
    let m = 1.0_f64;
    let young = k * 1000.0; // EA/L = young*1/1000 = k
    let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint,
        mass,
        story: None,
    };
    let beam = |id: u32, a: u32, b: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
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
    Model {
        nodes: vec![
            node(0, 0.0, Dof6Mask::FIXED, None),
            node(1, 1000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
            node(2, 2000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
        ],
        elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
        sections: vec![Section {
            id: SectionId(0),
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_time_history_rayleigh_and_hht() {
    let mut app = App::default();
    app.load_model(shear_2dof_model());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_damping_model = ThDampingModel::Rayleigh;
    app.analysis_cfg.th_integrator = ThIntegrator::HhtAlpha;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(!th.history.node_disp.is_empty());
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_set_story_weight_via_ui_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let story_id = app.model.stories[0].id;
    let old_weight = app.model.stories[0].seismic_weight;

    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetStoryWeight {
            story: story_id,
            weight: Some(12345.0),
        }),
    );
    assert_eq!(app.model.stories[0].seismic_weight, Some(12345.0));

    app.undo.undo(&mut app.model);
    assert_eq!(app.model.stories[0].seismic_weight, old_weight);
}

#[test]
fn test_pushover_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
    assert!(!po.capacity_curve.is_empty());
}

/// プッシュオーバー結果から質点系（串団子）モデルを生成する配線の end-to-end 確認。
#[test]
fn test_lumped_mass_model_from_pushover() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();

    let lm = squid_n_solver::lumped_mass::build_lumped_mass_model(
        &app.model,
        po,
        app.analysis_cfg.lumped_mass_type,
        app.analysis_cfg.lumped_secant_ratio,
    );
    // 層数分の質点が生成され、各層のトリリニア骨格が妥当（K1>0・折点昇順）。
    assert_eq!(lm.stories.len(), app.model.stories.len());
    assert!(!lm.stories.is_empty());
    for stick in &lm.stories {
        let sk = &stick.skeleton;
        assert!(sk.k1 > 0.0, "K1>0: {sk:?}");
        assert!(sk.d1 <= sk.d2 && sk.d2 <= sk.d3, "折点昇順: {sk:?}");
        assert!(stick.mass >= 0.0);
    }
}

/// 制振ダンパーの作成→諸元変更→削除を app の undo スタック経由で確認する
/// （部材表 UI が発行する編集コマンドの統合確認）。
#[test]
fn test_damper_create_edit_delete_via_undo() {
    use squid_n_core::model::{
        DamperProps, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
    };
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n = app.model.nodes.len();
    assert!(n >= 2);
    let (i_node, j_node) = (app.model.nodes[0].id, app.model.nodes[1].id);
    let new_id = squid_n_core::ids::ElemId(app.model.elements.len() as u32);
    let elem = ElementData {
        id: new_id,
        kind: ElementKind::Damper,
        nodes: [i_node, j_node].into_iter().collect(),
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
    // 作成。
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::AddDamper {
            elem,
            props: DamperProps::default(),
        }),
    );
    assert_eq!(app.model.damper_props(new_id), Some(DamperProps::default()));
    // 諸元変更。
    let edited = DamperProps {
        kd: 150_000.0,
        c0: 3_000.0,
        alpha: 0.35,
        ..Default::default()
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetDamperProps {
            elem: new_id,
            props: Some(edited),
        }),
    );
    assert_eq!(app.model.damper_props(new_id), Some(edited));
    // 削除（要素も特性も消える）。
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::DeleteMember { id: new_id }),
    );
    assert_eq!(app.model.damper_props(new_id), None);
    assert!(app.model.elements.iter().all(|e| e.id != new_id));
}

/// `poll_job` が完了するまで待つ（タイムアウト5秒でパニック、10ms 間隔でポーリング）。
fn wait_for_job(app: &mut App) {
    let start = std::time::Instant::now();
    while !app.poll_job() {
        assert!(
            start.elapsed() < std::time::Duration::from_secs(5),
            "ジョブが時間内に完了しませんでした"
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[test]
fn test_async_pushover_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.analysis_cfg.push_steps = 10;

    app.start_pushover_job();
    assert!(app.job.is_some());

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
    assert!(!po.capacity_curve.is_empty());
}

#[test]
fn test_async_time_history_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    let wave = App::sample_wave(&app.analysis_cfg);

    app.start_time_history_job(wave);
    assert!(app.job.is_some());

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(th.history.node_disp.len() > 100);
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_start_job_while_running_is_rejected() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;

    app.start_pushover_job();
    assert!(app.job.is_some());

    // 実行中に再度 start しても2つ目は無視され、job は上書きされない。
    app.start_time_history_job(App::sample_wave(&app.analysis_cfg));
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "プッシュオーバー");

    wait_for_job(&mut app);

    assert!(app.job.is_none());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.results.as_ref().unwrap().pushover.is_some());
    assert!(app.results.as_ref().unwrap().time_history.is_none());
}

#[test]
fn test_save_and_open_project_roundtrip() {
    let dir = std::env::temp_dir().join("squid_n_app_test_scz");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.scz");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.staleness.mark_edited();
    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(!app.staleness.unsaved_changes);
    assert_eq!(app.project_path.as_ref(), Some(&path));

    let saved_model = app.model.clone();
    let mut app2 = App::default();
    app2.open_project_from(path.clone());
    assert!(app2.last_error.is_none(), "{:?}", app2.last_error);
    assert!(app2.model.eq_ignoring_dofmap(&saved_model));
    assert_eq!(app2.project_path.as_ref(), Some(&path));

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_open_project_missing_file_sets_error() {
    let mut app = App::default();
    app.open_project_from(std::path::PathBuf::from(
        "/nonexistent/dir/does_not_exist.scz",
    ));
    assert!(app.last_error.is_some());
}

#[test]
fn test_export_and_import_stbridge_roundtrip() {
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let original = app.model.clone();
    app.export_stbridge_to(path.clone(), squid_n_io::stbridge::SectionExportMode::Raw);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    assert!(app2.last_error.is_none(), "{:?}", app2.last_error);
    assert!(app2.model.validate().is_ok());
    // ST-Bridge プロジェクト(.scz)とは別物なので project_path は更新されない。
    assert!(app2.project_path.is_none());

    // サブセットのため完全一致(eq_ignoring_dofmap)は求めない
    // （拘束条件・部材荷重は ST-Bridge の対象外で失われる）が、
    // 節点数・部材数はまず一致するはず。
    assert_eq!(app2.model.nodes.len(), original.nodes.len());
    assert_eq!(app2.model.elements.len(), original.elements.len());

    // 座標・部材の接続関係（節点参照・断面・材料・部材軸）はこの門型ラーメンでは
    // 完全にビット一致する（列/梁の判定に依らず節点順序が保たれるケース）。
    for (a, b) in app2.model.nodes.iter().zip(original.nodes.iter()) {
        assert_eq!(a.coord, b.coord);
    }
    assert_eq!(app2.model.elements, original.elements);

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_export_stbridge_standard_mode_writes_steel_library() {
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_std");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("standard.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.export_stbridge_to(
        path.clone(),
        squid_n_io::stbridge::SectionExportMode::Standard,
    );
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let xml = std::fs::read_to_string(&path).unwrap();
    // 門型ラーメンのサンプルは鋼 H 断面（柱・梁）を持つため、標準モードでは
    // 標準断面要素と形鋼ライブラリが書き出される。
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱は StbSecColumn_S");
    assert!(xml.contains("<StbSecBeam_S "), "鋼梁は StbSecBeam_S");
    assert!(xml.contains("<StbSecSteel>"), "形鋼ライブラリを出す");
    assert!(
        !xml.contains("<StbSecRaw "),
        "形状を持つ断面は Raw にしない"
    );

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_stbridge_standard_mode_roundtrip_through_app() {
    // 断面形状モードで書き出したファイルを GUI 経路（import_stbridge_from）で
    // 読み戻せる（検証エラーなくモデルが差し替わり、断面形状が復元される）。
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_std_rt");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("standard_rt.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n_sections = app.model.sections.len();
    app.export_stbridge_to(
        path.clone(),
        squid_n_io::stbridge::SectionExportMode::Standard,
    );
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    assert!(
        app2.last_error.is_none(),
        "標準モードのファイルを読み戻せる: {:?}",
        app2.last_error
    );
    assert!(app2.model.validate().is_ok());
    assert_eq!(app2.model.sections.len(), n_sections);
    // サンプルは鋼 H 断面のみ。読み戻した断面も H 形鋼として復元される。
    assert!(
        app2.model.sections.iter().all(|s| matches!(
            s.shape,
            Some(squid_n_core::section_shape::SectionShape::SteelH { .. })
        )),
        "断面形状が H 形鋼として復元される"
    );

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_import_stbridge_missing_file_sets_error() {
    let mut app = App::default();
    app.import_stbridge_from(std::path::PathBuf::from(
        "/nonexistent/dir/does_not_exist.stb",
    ));
    assert!(app.last_error.is_some());
}

#[test]
fn test_combination_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    let combo = squid_n_core::model::LoadCombination {
        name: "G+Kx".into(),
        terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::AddCombination { combo }),
    );

    app.run_combination(0);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let bundle = app.results.as_ref().unwrap();
    assert_eq!(bundle.combos.len(), 1);
    assert!(!bundle.checks.is_empty());
    assert_eq!(app.last_static, Some(StaticKey::Combo(0)));
}

/// `run_all_combinations` は個別に `run_combination` を実行した場合と
/// 同じ結果（combos の名前・変位）を与える（並列/一括経路と単発経路の一致確認）。
/// 決定性のため `threads=1`（Deterministic）を明示する。
#[test]
fn test_run_all_combinations_matches_individual_runs() {
    let combos = vec![
        squid_n_core::model::LoadCombination {
            name: "G+Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        },
        squid_n_core::model::LoadCombination {
            name: "G-Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), -1.0)],
        },
    ];

    let mut app_batch = App::default();
    app_batch.load_model(crate::sample::portal_frame());
    app_batch.analysis_cfg.threads = 1;
    for combo in combos.clone() {
        app_batch.undo.run(
            &mut app_batch.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );
    }
    app_batch.run_all_combinations();
    assert!(app_batch.last_error.is_none(), "{:?}", app_batch.last_error);

    let mut app_each = App::default();
    app_each.load_model(crate::sample::portal_frame());
    app_each.analysis_cfg.threads = 1;
    for combo in combos {
        app_each.undo.run(
            &mut app_each.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );
    }
    app_each.run_combination(0);
    assert!(app_each.last_error.is_none(), "{:?}", app_each.last_error);
    app_each.run_combination(1);
    assert!(app_each.last_error.is_none(), "{:?}", app_each.last_error);

    let bundle_batch = app_batch.results.as_ref().unwrap();
    let bundle_each = app_each.results.as_ref().unwrap();
    assert_eq!(bundle_batch.combos.len(), bundle_each.combos.len());
    for ((name_b, res_b), (name_e, res_e)) in
        bundle_batch.combos.iter().zip(bundle_each.combos.iter())
    {
        assert_eq!(name_b, name_e);
        assert_eq!(res_b.disp, res_e.disp);
    }
    assert_eq!(app_batch.last_static, Some(StaticKey::Combo(1)));
}

/// 荷重組合せが 1 件も無い場合はエラーメッセージを設定し、結果は変更しない。
#[test]
fn test_run_all_combinations_no_combos_is_error() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert!(app.model.combinations.is_empty());

    app.run_all_combinations();
    assert!(app.last_error.is_some());
    assert!(app.results.is_none());
}

#[test]
fn test_current_static_priority() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let expected_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

    // ナビゲータで存在しない Combo を選択していても last_static にフォールバックする
    app.nav.focus_result = Some(StaticKey::Combo(9));
    let fallback = app
        .current_static()
        .expect("無効な選択時は last_static にフォールバックするはず");
    assert_eq!(fallback.disp, expected_disp);

    // Case を選択すれば該当ケースの結果が返る
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
    let by_case = app
        .current_static()
        .expect("Case 選択時は該当ケースの結果が返るはず");
    assert_eq!(by_case.disp, expected_disp);
}

#[test]
fn test_holding_capacity_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階が未定義 → Err
    assert!(app.compute_holding_capacity().is_err());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // プッシュオーバー未実行 → Err
    assert!(app.compute_holding_capacity().is_err());

    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let (result, story_ranks) = app
        .compute_holding_capacity()
        .expect("前提が揃えば Ok のはず");
    assert_eq!(result.stories.len(), 1);
    assert!(result.stories[0].qun > 0.0);
    // Qu はプッシュオーバー最終点の層せん断（capacity_curve.story_shear）から取得される。
    assert!(result.stories[0].qu > 0.0, "{}", result.stories[0].qu);
    // design_rank_auto=false（既定）→ 全層フォールバック（選択値 design_rank）。
    assert_eq!(story_ranks, vec![app.design_rank]);
    assert!(result.member_ranks.is_empty());
}

/// UI-13: `design_rank_auto = true` で鋼部材の幅厚比から部材ランクを自動判定する。
/// portal_frame の柱(H-300x300x10x15)・梁(H-400x200x8x13)を、構造規定の
/// 幅厚比表（鋼構造設計規準、`s_member_rank_by_kihon`）で
/// 手計算した結果と一致することを確認する。
/// - 柱(SN400B=400級): フランジ 150/15=10.0（>9.5 → FB）、ウェブ 27.0（≦43 → FA）→ FB
/// - 梁(400級): フランジ 100/13≈7.69（≦9 → FA）、ウェブ 46.75（≦60 → FA）→ FA
#[test]
fn test_holding_capacity_rank_auto_from_width_thickness() {
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    use squid_n_design_jp::secondary::member_rank::worst_rank;
    use squid_n_design_jp::secondary::width_thickness::{s_member_rank_by_kihon, SteelMemberUse};

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.design_rank_auto = true;
    let (result, story_ranks) = app
        .compute_holding_capacity()
        .expect("shape 付き鋼断面があれば Ok のはず");

    assert!(
        !result.member_ranks.is_empty(),
        "鋼部材の幅厚比からランクが算定されているはず"
    );

    // 柱 H-300x300x10x15（SN400B）: フランジ 10.0 → FB が支配。
    let col_rank = s_member_rank_by_kihon(
        app.model.sections[0].shape.as_ref().unwrap(),
        SteelMemberUse::Column,
        "SN400B",
    )
    .unwrap();
    assert_eq!(col_rank, MemberRank::FB);
    // 梁 H-400x200x8x13（SN400B）: フランジ・ウェブとも FA。
    let beam_rank = s_member_rank_by_kihon(
        app.model.sections[1].shape.as_ref().unwrap(),
        SteelMemberUse::Beam,
        "SN400B",
    )
    .unwrap();
    assert_eq!(beam_rank, MemberRank::FA);

    for (elem_id, rank) in &result.member_ranks {
        let expected = if elem_id.0 == 2 { beam_rank } else { col_rank };
        assert_eq!(
            *rank, expected,
            "ElemId({}) のランクが手計算値と一致しません",
            elem_id.0
        );
    }
    // 唯一の層の代表ランクは柱・梁のうち最悪値（FD 寄り）。
    assert_eq!(story_ranks.len(), 1);
    assert_eq!(story_ranks[0], worst_rank(&[col_rank, beam_rank]).unwrap());
}

/// SectionShape::RcRect の配筋情報から `rc_capacity_input_from_rect` で
/// `RcCapacityInput` を組み立てる経路そのものを検証する（RcRect→入力構築）。
/// 得られた入力から `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank` の結果が、
/// 同じ式を独立に書き下した手計算と一致することを確認する。
#[test]
fn test_rc_capacity_input_from_rect_matches_handcalc() {
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::Material;
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
    use squid_n_design_jp::secondary::member_rank::{rc_member_rank, RankCriteria};
    use squid_n_design_jp::secondary::rc_capacity::{rc_qmu_simple, rc_qsu_simple};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    // 材料名は "FC24"（is_steel が false になる、かつ fc 設定あり）を想定。
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None, // 未設定 → sigma_y は 345(SD345相当)にフォールバックするはず
    };
    let clear_span = 3000.0;

    let input = rc_capacity_input_from_rect(b, d, &rebar, &mat, clear_span)
        .expect("fc が設定されているので Some のはず");

    // 変換規則の確認: at=main_x総断面積の半分、d_eff=d-cover-dia/2、
    // pw=せん断補強筋断面積・組数/(b・ピッチ)、sigma_y は fy 未設定なので 345 固定、
    // sigma_wy は常に 295 固定。
    let main_area = 8.0 * std::f64::consts::PI / 4.0 * 22.0 * 22.0;
    let at_expected = main_area / 2.0;
    let d_eff_expected = 600.0 - 40.0 - 22.0 / 2.0;
    let shear_area = std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0;
    let pw_expected = shear_area / (400.0 * 150.0);
    assert!((input.at - at_expected).abs() < 1e-9);
    assert!((input.d_eff - d_eff_expected).abs() < 1e-9);
    assert!((input.pw - pw_expected).abs() < 1e-12);
    assert_eq!(input.sigma_y, 345.0);
    assert_eq!(input.sigma_wy, 295.0);
    assert_eq!(input.fc, 24.0);
    assert_eq!(input.clear_span, clear_span);

    // rc_qsu_simple/rc_qmu_simple の結果を、式を独立に書き下した手計算と照合する。
    // Mu = 0.9·at·σy·d（技術基準解説書 P.623。d = 有効せい）。
    let j = 7.0 * d_eff_expected / 8.0;
    let mu_handcalc = 0.9 * at_expected * 345.0 * d_eff_expected;
    let qmu_handcalc = 2.0 * mu_handcalc / clear_span;
    let pt = 100.0 * at_expected / (400.0 * d_eff_expected);
    let shear_span_ratio = (clear_span / (2.0 * d_eff_expected)).clamp(1.0, 3.0);
    let pw_clamped = pw_expected.clamp(0.0, 0.012);
    let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (shear_span_ratio + 0.12);
    let hoop_term = 0.85 * (pw_clamped * 295.0_f64).sqrt();
    let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;

    let qmu = rc_qmu_simple(&input);
    let qsu = rc_qsu_simple(&input);
    assert!(
        (qmu - qmu_handcalc).abs() < 1e-3,
        "Qmu={} vs handcalc={}",
        qmu,
        qmu_handcalc
    );
    assert!(
        (qsu - qsu_handcalc).abs() < 1e-3,
        "Qsu={} vs handcalc={}",
        qsu,
        qsu_handcalc
    );

    let rank = rc_member_rank(qsu, qmu, &RankCriteria::default());
    let rank_handcalc = rc_member_rank(qsu_handcalc, qmu_handcalc, &RankCriteria::default());
    assert_eq!(rank, rank_handcalc);
    // Qsu/Qmu ≈ 2.12（曲げ降伏が十分先行する健全な配筋）なので FA になるはず。
    assert_eq!(
        rank,
        squid_n_design_jp::secondary::holding_capacity::MemberRank::FA
    );
}

/// UI-13(RC): SectionShape::RcRect + fc 付き材料（コンクリート、is_steel=false）を
/// 持つ小さな門型ラーメンを組み、rank-auto で member_ranks に RC 部材のランクが入り、
/// `rc_capacity_input_from_rect` → `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank`
/// の手計算と一致することを確認する。
#[test]
fn test_holding_capacity_rank_auto_rc_rect_from_shape() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MemberLoad, MemberLoadKind, Model, NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    use squid_n_design_jp::secondary::member_rank::{rc_member_rank, RankCriteria};
    use squid_n_design_jp::secondary::rc_capacity::{rc_qmu_simple, rc_qsu_simple};

    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: rebar.clone(),
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
                coord: [4000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
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
        ..Default::default()
    };
    let members = [
        (0u32, 0u32, 2u32, [1.0, 0.0, 0.0]),
        (1, 1, 3, [1.0, 0.0, 0.0]),
        (2, 2, 3, [0.0, 0.0, 1.0]),
    ];
    for (id, i, j, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
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
        name: "長期".into(),
        nodal: Vec::new(),
        member: vec![MemberLoad {
            elem: ElemId(2),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
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

    let mut app = App::default();
    app.load_model(model);
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // RC の簡易断面・fy 未設定材料はヒンジ耐力の既定値(鋼材既定 235N/mm²)を用いる
    // 都合上、既定の push_max_disp=500mm では機構形成後に特異行列となり得るため、
    // 微小変位のみを対象とする(ここではランク判定経路の配線確認が目的で、
    // 崩壊形の精算は対象外)。
    app.analysis_cfg.push_steps = 3;
    app.analysis_cfg.push_max_disp = 3.0;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.design_rank_auto = true;
    let (result, _story_ranks) = app
        .compute_holding_capacity()
        .expect("RC 矩形 + fc 付き材料があれば Ok のはず");

    assert!(
        !result.member_ranks.is_empty(),
        "RC 部材(RcRect+fc)のせん断余裕度からランクが算定されているはず"
    );

    // 柱: 節点間距離 3000mm、梁: 節点間距離 4000mm。それぞれ手計算で
    // rc_capacity_input_from_rect → rc_qsu/qmu_simple → rc_member_rank を再現する。
    //
    // σ0 は実運用と同じ規則(rc_sigma_0_from_gravity_or_last_static)で個別に反映する。
    // このテストでは run_linear_static(先頭ケース="長期")を実行していないため、
    // gravity_lc=LoadCaseId(0) は statics 内の StaticCaseKey::User(LoadCaseId(0))
    // として見つからず、フォールバック(bundle.member_forces = 直近実行した
    // run_seismic の内力)が使われる(= 最後の静的解析結果と同じ)。地震水平力による
    // 柱の転倒モーメント抵抗で柱0・柱1の軸力は一方が圧縮・他方が引張(または
    // 大きさが異なる)になり得るため、部材ごとに算定する(柱を一括りにしない)。
    let mat = &app.model.materials[0];
    let statics = &app.results.as_ref().unwrap().statics;
    let member_forces = &app.results.as_ref().unwrap().member_forces;
    let gravity_lc = app.model.load_cases.first().map(|c| c.id);
    let expected_rank_for = |elem_id: ElemId, clear_span: f64| {
        let mut input = rc_capacity_input_from_rect(400.0, 600.0, &rebar, mat, clear_span)
            .expect("fc 設定済みなので Some");
        input.sigma_0 = rc_sigma_0_from_gravity_or_last_static(
            statics,
            member_forces,
            gravity_lc,
            elem_id,
            400.0,
            600.0,
        );
        let qmu = rc_qmu_simple(&input);
        let qsu = rc_qsu_simple(&input);
        rc_member_rank(qsu, qmu, &RankCriteria::default())
    };
    let col0_rank = expected_rank_for(ElemId(0), 3000.0);
    let col1_rank = expected_rank_for(ElemId(1), 3000.0);
    let beam_rank = expected_rank_for(ElemId(2), 4000.0);

    for (elem_id, rank) in &result.member_ranks {
        let expected = match elem_id.0 {
            2 => beam_rank,
            1 => col1_rank,
            _ => col0_rank,
        };
        assert_eq!(
            *rank, expected,
            "ElemId({}) のランクが手計算値と一致しません",
            elem_id.0
        );
    }
}

/// `rc_sigma_0_from_gravity_or_last_static`: 圧縮軸力から σ0 が正しく算定されることを、
/// 実際に静的解析を実行して確認する。
///
/// モデル: 鉛直片持ち柱（節点0=基部, 固定, z=0 / 節点1=先端, 自由, z=3000）に
/// RC矩形断面 400x600 を設定し、先端節点へ下向き(圧縮)集中荷重 P=100,000N を
/// 与える。軸力のみが生じる単純な釣合いなので、内力の軸力の大きさは
/// 弾性係数・断面性能によらず厳密に P と一致する。
///
/// 符号規約の確認: squid-n-solver::linear::test_linear_static_axial_cantilever
/// で N=+1000N(引張)のとき forces.at[0].1[0]≈-1000 であることを確認済みなので、
/// 圧縮(先端を下向きに押す)では forces.at[0].1[0]≈+P（正）になるはず。
#[test]
fn test_rc_sigma_0_from_compression_axial_force() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
        NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };

    let p = 100_000.0; // 圧縮荷重 [N]
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
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
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
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
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "圧縮".into(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [0.0, 0.0, -p, 0.0, 0.0, 0.0],
            }],
            member: Vec::new(),
        }],
        ..Default::default()
    };

    let mut app = App::default();
    app.load_model(model);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let member_forces = &app.results.as_ref().unwrap().member_forces;
    let (_, mf) = member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("elem 0 の内力があるはず");
    let n_raw = mf.at.first().expect("eval_sections[0] があるはず").1[0];
    // 軸力は引張正の部材内力なので、圧縮 P に対して n_raw = -P となる。
    assert!(
        (n_raw + p).abs() < 1e-6,
        "n_raw={} (expected {})",
        n_raw,
        -p
    );

    let statics = &app.results.as_ref().unwrap().statics;
    let gravity_lc = app.model.load_cases.first().map(|c| c.id);
    let sigma_0 =
        rc_sigma_0_from_gravity_or_last_static(statics, member_forces, gravity_lc, ElemId(0), b, d);
    let expected_sigma_0 = p / (b * d);
    assert!(
        (sigma_0 - expected_sigma_0).abs() < 1e-9,
        "sigma_0={} expected={}",
        sigma_0,
        expected_sigma_0
    );
}

/// `rc_sigma_0_from_gravity_or_last_static`: 先頭荷重ケース(gravity_lc)の静的解析結果が
/// `bundle.statics` にあれば、最後に実行した(かつ結果が異なる)静的解析ではなく
/// 先頭荷重ケースの結果が優先されることを確認する。
///
/// モデル: `test_rc_sigma_0_from_compression_axial_force` と同じ片持ち柱に、
/// 先頭荷重ケース(id=0,"長期")として圧縮荷重 P1、2番目のケース(id=1,"地震")として
/// 引張荷重 P2 を設定する。両ケースをこの順に実行すると
/// `bundle.member_forces`(=最後に実行したケース)は引張(id=1)の結果になり、
/// これをそのまま使うと σ0=0(引張は 0 とみなす安全側処理)になってしまう。
/// 優先順位が正しく効いていれば、`bundle.statics` 内の id=0(長期)の圧縮軸力から
/// σ0=P1/(b・D) (>0) が算定される。
#[test]
fn test_rc_sigma_0_prefers_gravity_load_case_over_last_static() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
        NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };

    let p1 = 100_000.0; // 先頭ケース(長期)の圧縮荷重 [N]
    let p2 = 60_000.0; // 2番目のケース(地震想定)の引張荷重 [N]
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
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
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
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
        load_cases: vec![
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(0),
                name: "長期".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 0.0, -p1, 0.0, 0.0, 0.0], // 下向き=圧縮
                }],
                member: Vec::new(),
            },
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(1),
                name: "地震".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 0.0, p2, 0.0, 0.0, 0.0], // 上向き=引張
                }],
                member: Vec::new(),
            },
        ],
        ..Default::default()
    };

    let mut app = App::default();
    app.load_model(model);
    // 先頭ケース(長期,圧縮)→2番目のケース(地震,引張)の順に実行し、
    // 「最後に実行した静的解析結果」は引張(id=1)になるようにする。
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.run_linear_static(LoadCaseId(1));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let bundle = app.results.as_ref().unwrap();
    // 最後に実行した静的解析結果(bundle.member_forces)は引張なので、
    // これをそのまま使うと σ0=0 になってしまうことの確認(比較対象)。
    let sigma_0_last_only =
        rc_sigma_0_from_gravity_or_last_static(&[], &bundle.member_forces, None, ElemId(0), b, d);
    assert_eq!(sigma_0_last_only, 0.0, "引張のみなら σ0=0 のはず(比較対象)");

    // 優先順位が正しく効いていれば、先頭ケース(長期,id=0)の圧縮軸力から
    // σ0=P1/(b・D) (>0) が算定される。
    let gravity_lc = app.model.load_cases.first().map(|c| c.id);
    assert_eq!(gravity_lc, Some(LoadCaseId(0)));
    let sigma_0 = rc_sigma_0_from_gravity_or_last_static(
        &bundle.statics,
        &bundle.member_forces,
        gravity_lc,
        ElemId(0),
        b,
        d,
    );
    let expected_sigma_0 = p1 / (b * d);
    assert!(
        (sigma_0 - expected_sigma_0).abs() < 1e-9,
        "sigma_0={} expected={}(先頭ケースの圧縮軸力が優先されるはず)",
        sigma_0,
        expected_sigma_0
    );
}

/// Z=0 平面の矩形（4000×6000）+外周4本の梁 + スラブ1枚（TriTrapezoid）を持つモデルを作る。
/// 辺 i = boundary[i] → boundary[(i+1)%4] の順に梁を並べる（refresh_beam_loads の対応付けと一致）。
fn make_slab_test_model() -> squid_n_core::model::Model {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 4000.0, 0.0),
        mk_node(2, 4000.0, 6000.0),
        mk_node(3, 0.0, 6000.0),
    ];
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
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
    let elements = vec![
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ];
    let slab = Slab {
        usage: None,
        edge_supported: None,
        kind: Default::default(),
        one_way: None,
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
    };
    squid_n_core::model::Model {
        nodes,
        elements,
        slabs: vec![slab],
        ..Default::default()
    }
}

#[test]
fn test_refresh_beam_loads_maps_edges_to_members() {
    let model = make_slab_test_model();
    model
        .validate()
        .expect("テストモデルは validate を通るはず");

    let mut app = App {
        model,
        ..App::default()
    };
    app.refresh_beam_loads();

    assert_eq!(app.beam_loads.len(), 4, "外周4辺すべてに荷重が対応付くはず");
    for bl in &app.beam_loads {
        let elem = app
            .model
            .elements
            .iter()
            .find(|e| e.id == bl.elem)
            .expect("beam_loads.elem は実在する部材IDを指すはず");
        assert_eq!(elem.kind, squid_n_core::model::ElementKind::Beam);
        assert!(
            bl.cmq.c_i.abs() > 1e-9 || bl.cmq.q_i.abs() > 1e-9,
            "CMQ が非ゼロのはず: {:?} {:?}",
            bl.cmq.c_i,
            bl.cmq.q_i
        );
    }

    // 梁が1本欠けたモデルでは、対応する辺の荷重が捨てられ3件になる
    let mut missing = app.model.clone();
    missing.elements.pop();
    app.model = missing;
    app.refresh_beam_loads();
    assert_eq!(app.beam_loads.len(), 3);
}

/// 正方形スラブ（4000×4000）+ 外周4本の梁を持つモデル
/// （`make_slab_test_model` の正方形版。正方形は `TriTrapezoid` で全辺
/// 三角形分布になるため §1.1 のスラブ→荷重ケース同期の検算がしやすい）。
fn make_square_slab_test_model() -> squid_n_core::model::Model {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 4000.0, 0.0),
        mk_node(2, 4000.0, 4000.0),
        mk_node(3, 0.0, 4000.0),
    ];
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
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
    let elements = vec![
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ];
    let slab = Slab {
        usage: None,
        edge_supported: None,
        kind: Default::default(),
        one_way: None,
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
    };
    squid_n_core::model::Model {
        nodes,
        elements,
        slabs: vec![slab],
        ..Default::default()
    }
}

/// レビュー §1.1（最重要）: スラブ荷重が `sync_slab_loads_action` で
/// 「床荷重(自動)」荷重ケースへ実際に書き込まれ、応力解析から参照可能に
/// なることを確認する。正方形スラブは全辺三角形分布（2区間）になるため
/// `MemberLoadKind::Distributed` への変換規則を直接検算できる。
#[test]
fn test_sync_slab_loads_action_square_slab_triangle_distribution() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind};

    let model = make_square_slab_test_model();
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };

    app.sync_slab_loads_action();

    let case = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME)
        .expect("床荷重(自動)ケースが作られるはず");
    assert_eq!(case.kind, LoadCaseKind::Dead);
    assert_eq!(case.member.len(), 8, "4辺 × 2区間（三角形分布）= 8件");
    assert!(case.nodal.is_empty(), "小梁が無いので節点荷重は空のはず");

    // 各梁にちょうど2区間ずつ入っていることを確認
    for elem_id in 0..4u32 {
        let n_segs = case
            .member
            .iter()
            .filter(|m| m.elem == ElemId(elem_id))
            .count();
        assert_eq!(n_segs, 2, "梁#{elem_id} には三角形分布の2区間が入るはず");
        for m in case.member.iter().filter(|m| m.elem == ElemId(elem_id)) {
            assert_eq!(m.dir, [0.0, 0.0, -1.0], "作用方向は鉛直下向き固定のはず");
        }
    }

    // 鉛直合計 = w × 面積（保存則）
    let total: f64 = case
        .member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    let expected = 0.005 * 4000.0 * 4000.0;
    assert!(
        (total - expected).abs() < 1e-6,
        "total={total} expected={expected}"
    );

    // 再同期しても重複しない（全置換）
    app.sync_slab_loads_action();
    let cases: Vec<_> = app
        .model
        .load_cases
        .iter()
        .filter(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME)
        .collect();
    assert_eq!(cases.len(), 1, "再同期でケースが重複してはいけない");
    assert_eq!(cases[0].member.len(), 8, "再同期で荷重が重複してはいけない");

    // undo で元に戻る（新規作成だったケースが丸ごと消える）
    app.undo.undo(&mut app.model);
    assert!(
        !app.model
            .load_cases
            .iter()
            .any(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME),
        "undo で「床荷重(自動)」ケースが消えるはず"
    );
}

/// 床 Phase A-2: 用途（`SlabUsage`）を設定したスラブは、`sync_slab_loads_action`
/// で固定荷重（DL・「床荷重(自動)」kind=Dead）と積載荷重（LL・「床積載(自動)」
/// kind=Live）の 2 ケースに分離されることを確認する。LL は令別表第1 の
/// **骨組用**積載（事務室=1800 N/m²）を用い、DL とは独立に総和保存する。
#[test]
fn test_sync_slab_loads_action_separates_dead_and_live() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind, SlabUsage};

    let mut model = make_square_slab_test_model();
    // 事務室用途を設定（骨組用 LL = 1800 N/m² = 1.8e-3 N/mm²）。
    model.slabs[0].usage = Some(SlabUsage::Office);
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };

    app.sync_slab_loads_action();

    fn sum_vertical(model: &squid_n_core::model::Model, name: &str) -> f64 {
        model
            .load_cases
            .iter()
            .find(|lc| lc.name == name)
            .map(|c| {
                c.member
                    .iter()
                    .map(|m| match m.kind {
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                        MemberLoadKind::Point { p, .. } => p,
                    })
                    .sum()
            })
            .unwrap_or(0.0)
    }

    // DL ケース: 従来どおり loads(0.005) を分配。
    let dl = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME)
        .expect("床荷重(自動)ケースが作られるはず");
    assert_eq!(dl.kind, LoadCaseKind::Dead);
    let area = 4000.0 * 4000.0;
    assert!((sum_vertical(&app.model, SLAB_AUTO_LOAD_CASE_NAME) - 0.005 * area).abs() < 1e-6);

    // LL ケース: 骨組用積載 1.8e-3 N/mm² を分配（DL とは別ケース・kind=Live）。
    let ll = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_LIVE_AUTO_LOAD_CASE_NAME)
        .expect("床積載(自動)ケースが作られるはず");
    assert_eq!(ll.kind, LoadCaseKind::Live);
    assert!((sum_vertical(&app.model, SLAB_LIVE_AUTO_LOAD_CASE_NAME) - 1.8e-3 * area).abs() < 1e-6);

    // 用途を外すと LL ケースは空同期され、寄与が無くなる（新規なら作られない）。
    app.model.slabs[0].usage = None;
    app.sync_slab_loads_action();
    assert!(
        (sum_vertical(&app.model, SLAB_LIVE_AUTO_LOAD_CASE_NAME)).abs() < 1e-12,
        "用途を外したら積載寄与は 0 になるはず"
    );
}

/// 床 Phase A-3: 用途を設定したスラブは地震用積載（LiveSeismic）ケースも同期され、
/// 地震用重量の重力ケース選択（`gravity_cases_for_seismic_weight`）が
/// 骨組用 Live ではなく地震用 LiveSeismic を採用することを確認する
/// （令85条1項の地震用値〔事務室=800 N/m²〕を地震用重量に用いる）。
#[test]
fn test_sync_slab_loads_action_seismic_live_case() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind, SlabUsage};

    let mut model = make_square_slab_test_model();
    model.slabs[0].usage = Some(SlabUsage::Office);
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };
    app.sync_slab_loads_action();

    let sum_vertical = |name: &str| -> f64 {
        app.model
            .load_cases
            .iter()
            .find(|lc| lc.name == name)
            .map(|c| {
                c.member
                    .iter()
                    .map(|m| match m.kind {
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                        MemberLoadKind::Point { p, .. } => p,
                    })
                    .sum()
            })
            .unwrap_or(0.0)
    };

    let area = 4000.0 * 4000.0;
    // 地震用積載ケース: 地震用値 800 N/m² = 8e-4 N/mm²。
    let ls = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_LIVE_SEISMIC_AUTO_LOAD_CASE_NAME)
        .expect("床地震用積載(自動)ケースが作られるはず");
    assert_eq!(ls.kind, LoadCaseKind::LiveSeismic);
    assert!((sum_vertical(SLAB_LIVE_SEISMIC_AUTO_LOAD_CASE_NAME) - 8e-4 * area).abs() < 1e-6);

    // 地震用重量の重力ケース: DL(床荷重) と LiveSeismic(床地震用積載) を含み、
    // 骨組用 Live(床積載) は含まない（LiveSeismic 優先）。
    let dl_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME)
        .unwrap()
        .id;
    let live_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_LIVE_AUTO_LOAD_CASE_NAME)
        .unwrap()
        .id;
    let ls_id = ls.id;
    let gravity = gravity_cases_for_seismic_weight(&app.model);
    assert!(gravity.contains(&dl_id), "DL(床荷重)は地震用重量に含むはず");
    assert!(
        gravity.contains(&ls_id),
        "LiveSeismic(床地震用積載)は地震用重量に含むはず"
    );
    assert!(
        !gravity.contains(&live_id),
        "骨組用 Live(床積載)は地震用重量に含めないはず（LiveSeismic 優先）"
    );
}

/// 床 Phase A-3 レビュー指摘: 用途の地震用値が明示的に 0（骨組用のみ正）の場合、
/// 地震用積載(LiveSeismic)ケースは生成されないが、骨組用 Live(床積載)ケースへ
/// フォールバックして地震用重量が過大にならないことを確認する
/// （自動生成の骨組用 Live ケースは地震用重量の代用対象から除外される）。
#[test]
fn test_gravity_cases_excludes_auto_frame_live_when_no_seismic() {
    use squid_n_core::model::SlabUsage;

    let mut model = make_square_slab_test_model();
    // 骨組用のみ正・地震用 0 の用途（serde 由来を想定した異常系）。
    model.slabs[0].usage = Some(SlabUsage::Custom {
        floor: 3e-3,
        frame: 2e-3,
        seismic: 0.0,
    });
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };
    app.sync_slab_loads_action();

    // 地震用値 0 なので LiveSeismic ケースは生成されない。
    assert!(
        !app.model
            .load_cases
            .iter()
            .any(|lc| lc.name == SLAB_LIVE_SEISMIC_AUTO_LOAD_CASE_NAME),
        "地震用値 0 なら床地震用積載(自動)は作られないはず"
    );
    // 骨組用 Live ケースは生成される。
    let live_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == SLAB_LIVE_AUTO_LOAD_CASE_NAME)
        .expect("床積載(自動)は作られるはず")
        .id;
    // 地震用重量の重力ケースに、骨組用 Live(床積載)を含めてはならない。
    let gravity = gravity_cases_for_seismic_weight(&app.model);
    assert!(
        !gravity.contains(&live_id),
        "自動生成の骨組用 Live(床積載)は地震用重量にフォールバックしてはならない"
    );
}

/// レビュー §1.7: 地震用重量に使う荷重ケースの選択が、並び順ではなく
/// `LoadCaseKind` に基づくことを確認する（Dead+LiveSeismic 優先、
/// LiveSeismic が無ければ Dead+Live、種別が一つも設定されていなければ
/// 従来互換で先頭ケースのみ）。
#[test]
fn test_gravity_cases_for_seismic_weight_selection() {
    use squid_n_core::model::{LoadCase, LoadCaseKind};

    let mk_lc = |i: u32, name: &str, kind: LoadCaseKind| LoadCase {
        id: LoadCaseId(i),
        name: name.to_string(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    };

    // 種別が一つも設定されていない（全て既定値 Other） → 先頭ケースのみ
    let model_no_kind = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "LC0", LoadCaseKind::Other),
            mk_lc(1, "LC1", LoadCaseKind::Other),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_no_kind),
        vec![LoadCaseId(0)],
        "種別未設定モデルは従来互換で先頭ケースのみ"
    );

    // LiveSeismic が無い → Dead + Live
    let model_dead_live = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定", LoadCaseKind::Dead),
            mk_lc(1, "積載(長期)", LoadCaseKind::Live),
            mk_lc(2, "積雪", LoadCaseKind::Snow),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_dead_live),
        vec![LoadCaseId(0), LoadCaseId(1)],
        "LiveSeismic が無ければ Dead+Live"
    );

    // LiveSeismic があれば Live ではなく LiveSeismic を優先
    let model_dead_live_seismic = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定", LoadCaseKind::Dead),
            mk_lc(1, "積載(長期)", LoadCaseKind::Live),
            mk_lc(2, "積載(地震用)", LoadCaseKind::LiveSeismic),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_dead_live_seismic),
        vec![LoadCaseId(0), LoadCaseId(2)],
        "LiveSeismic があれば Live ではなく LiveSeismic を採用"
    );

    // 複数 Dead ケースも全て対象
    let model_multi_dead = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定1", LoadCaseKind::Dead),
            mk_lc(1, "固定2", LoadCaseKind::Dead),
            mk_lc(2, "地震荷重", LoadCaseKind::Seismic),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_multi_dead),
        vec![LoadCaseId(0), LoadCaseId(1)],
        "複数の Dead ケースは全て対象、Seismic は対象外"
    );
}

/// テスト用の荷重ケース（種別付き）を作る。
fn kind_lc(
    i: u32,
    name: &str,
    kind: squid_n_core::model::LoadCaseKind,
) -> squid_n_core::model::LoadCase {
    squid_n_core::model::LoadCase {
        id: LoadCaseId(i),
        name: name.to_string(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    }
}

/// 種別から組合せを自動生成: Dead/Live/Snow/Wind の種別を設定したモデルで
/// 標準組合せ（長期・短期積雪・短期暴風±）が undo 可能に一括生成されること。
#[test]
fn test_auto_generate_combinations_from_kinds() {
    use squid_n_core::model::LoadCaseKind;

    let mut app = App::default();
    app.model.load_cases = vec![
        kind_lc(0, "固定", LoadCaseKind::Dead),
        kind_lc(1, "積載", LoadCaseKind::Live),
        kind_lc(2, "積雪", LoadCaseKind::Snow),
        kind_lc(3, "風", LoadCaseKind::Wind),
    ];

    app.auto_generate_combinations_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 多雪区域=false: G+P(1) + G+P+S(1) + 風±(2) = 4 ケース
    // （地震(Kx/Ky)は kind だけでは方向を判別できないため対象外の仕様）。
    let names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(
        names,
        vec!["G + P", "G + P + S", "G + P + Wx", "G + P - Wx"]
    );

    // G+P の中身は Dead(0)+Live(1) を各1.0で参照する。
    assert_eq!(
        app.model.combinations[0].terms,
        vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)]
    );

    // 各組合せは個別コマンドで追加されているため、全 undo で消える。
    for _ in 0..app.model.combinations.len() {
        app.undo.undo(&mut app.model);
    }
    assert!(app.model.combinations.is_empty());
}

/// 多雪区域フラグ（AnalysisSettings::heavy_snow_zone）を立てると
/// 0.7S・0.35S 系の組合せも生成されること。
#[test]
fn test_auto_generate_combinations_heavy_snow() {
    use squid_n_core::model::LoadCaseKind;

    let mut app = App::default();
    app.analysis_cfg.heavy_snow_zone = true;
    app.model.load_cases = vec![
        kind_lc(0, "固定", LoadCaseKind::Dead),
        kind_lc(1, "積載", LoadCaseKind::Live),
        kind_lc(2, "積雪", LoadCaseKind::Snow),
        kind_lc(3, "風", LoadCaseKind::Wind),
    ];

    app.auto_generate_combinations_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert!(names.contains(&"G + P + 0.7S"), "{names:?}");
    assert!(names.contains(&"G + P + 0.35S + Wx"), "{names:?}");
    assert!(names.contains(&"G + P + 0.35S - Wx"), "{names:?}");
}

/// Dead ケースが無い場合はエラーメッセージが設定され、組合せは生成されないこと。
/// Live 欠如も同様。
#[test]
fn test_auto_generate_combinations_missing_dead_or_live_is_error() {
    use squid_n_core::model::LoadCaseKind;

    // Dead 無し
    let mut app = App::default();
    app.model.load_cases = vec![kind_lc(0, "積載", LoadCaseKind::Live)];
    app.auto_generate_combinations_action();
    assert!(app.last_error.as_deref().unwrap().contains("固定荷重"));
    assert!(app.model.combinations.is_empty());

    // Live 無し
    let mut app = App::default();
    app.model.load_cases = vec![kind_lc(0, "固定", LoadCaseKind::Dead)];
    app.auto_generate_combinations_action();
    assert!(app.last_error.as_deref().unwrap().contains("積載荷重"));
    assert!(app.model.combinations.is_empty());
}

/// SetLoadCfg が App の undo スタック経由で機能すること
/// （荷重計算条件タブの編集経路のヘッドレス確認）。
#[test]
fn test_set_load_cfg_via_app_undo() {
    use squid_n_core::model::{KBraceWeightRule, LoadCfg};

    let mut app = App::default();
    assert!(app.model.load_cfg.is_none());

    let cfg = LoadCfg {
        steel_weight_factor: 1.1,
        k_brace_rule: KBraceWeightRule::BaseNodesOnly,
        live_load_reduction: true,
        ..Default::default()
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetLoadCfg {
            cfg: Some(cfg.clone()),
        }),
    );
    assert_eq!(app.model.load_cfg, Some(cfg));

    app.undo.undo(&mut app.model);
    assert!(app.model.load_cfg.is_none());
}

/// 3層1本柱のモデルで `column_live_load_factors` が
/// 支持床数（3,2,1）と低減率（0.90,0.95,1.00）を返すこと（令85条2項）。
#[test]
fn test_column_live_load_factors_three_story() {
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    let mut model = squid_n_core::model::Model::default();
    // 4節点(z=0,3000,6000,9000)。z>0 の節点に所属階(1F=story0..3F=story2)を設定。
    for (i, z) in [0.0, 3000.0, 6000.0, 9000.0].iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [0.0, 0.0, *z],
            restraint: if i == 0 {
                squid_n_core::dof::Dof6Mask::FIXED
            } else {
                squid_n_core::dof::Dof6Mask::FREE
            },
            mass: None,
            story: if i == 0 {
                None
            } else {
                Some(StoryId(i as u32 - 1))
            },
        });
    }
    // 柱3本（各階1本）＋ 水平の梁1本（柱でないため集計対象外の確認用）
    let mut push_elem = |id: u32, a: u32, b: u32| {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    };
    push_elem(0, 0, 1);
    push_elem(1, 1, 2);
    push_elem(2, 2, 3);
    // 水平材（同一 Z の節点を追加して繋ぐ）
    model.nodes.push(Node {
        id: NodeId(4),
        coord: [4000.0, 0.0, 9000.0],
        restraint: squid_n_core::dof::Dof6Mask::FREE,
        mass: None,
        story: Some(StoryId(2)),
    });
    model.elements.push(ElementData {
        id: ElemId(3),
        kind: ElementKind::Beam,
        nodes: [NodeId(3), NodeId(4)].into_iter().collect(),
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
    });

    let factors = column_live_load_factors(&model);
    // 水平梁(ElemId(3))は含まれない。
    assert_eq!(
        factors,
        vec![
            (ElemId(0), 3, 0.90),
            (ElemId(1), 2, 0.95),
            (ElemId(2), 1, 1.00),
        ]
    );
}

/// Z表 CSV の読込と市町村名参照 → analysis_cfg.z への反映（ヘッドレス）。
#[test]
fn test_z_table_load_and_apply() {
    let mut app = App::default();

    // 未読込での参照はエラー
    assert!(!app.apply_z_from_municipality("那覇市"));
    assert!(app.last_error.as_deref().unwrap().contains("Z表"));

    // 不正な Z 値（0.85 は告示1793号の値でない）はエラー
    app.load_z_table_from_csv("変な市,0.85\n");
    assert!(app.last_error.is_some());
    assert!(app.z_table.is_none());

    // 正常読込 → 参照で z が反映される
    app.load_z_table_from_csv("# 出典: 告示1793号 別表第2\n東京都千代田区,1.0\n沖縄県那覇市,0.7\n");
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.z_table.as_ref().unwrap().len(), 2);

    assert!(app.apply_z_from_municipality("沖縄県那覇市"));
    assert_eq!(app.analysis_cfg.z, 0.7);

    // 見つからない市町村はエラー、z は変わらない
    assert!(!app.apply_z_from_municipality("存在しない市"));
    assert!(app
        .last_error
        .as_deref()
        .unwrap()
        .contains("見つかりません"));
    assert_eq!(app.analysis_cfg.z, 0.7);
}

/// 風荷重静的解析（run_wind）: 階の定義後に実行でき、結果が
/// `StaticCaseKey::Wind(dir)` に格納されること。
///
/// サンプルの門型ラーメンは XZ 平面内の平面架構のため、Y 方向の風
/// （見付け幅 = X 方向の座標範囲 4000mm）のみ解析できる。X 方向の風は
/// 見付け幅（Y 範囲）が 0 のため明示エラーになることも併せて確認する。
#[test]
fn test_run_wind_static() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階なし → 明示エラー
    app.run_wind(SeismicDir::Y);
    assert!(app.last_error.is_some());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 平面架構の面外（X風）は見付け幅 0 の明示エラー
    app.run_wind(SeismicDir::X);
    assert!(app.last_error.is_some());

    app.run_wind(SeismicDir::Y);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let r = app.results.as_ref().unwrap();
    let wind = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::Wind(SeismicDir::Y))
        .expect("風静的Yの結果が格納されるはず");
    // 柱頭が Y 方向へ変位している（風方向の水平力が作用した証拠）
    assert!(wind.1.disp[2][1].abs() > 1e-9, "{}", wind.1.disp[2][1]);
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::Wind(SeismicDir::Y)))
    );
}

/// 終局検定（靭性保証型耐震設計指針）の App 経由の一括算定を検証する。
/// RC 矩形の柱・梁について `compute_ultimate_checks` が部材別の終局せん断・付着
/// 余裕度を返し、柱には軸終局耐力（Nuc/Nut）が付くことを確認する。
#[test]
fn test_compute_ultimate_checks_rc_frame() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    use squid_n_design_jp::MemberKind;

    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let col_shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar: rebar.clone(),
    };
    let beam_rebar = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        ..rebar
    };
    let beam_shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: beam_rebar,
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
            Node {
                id: NodeId(2),
                coord: [6000.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![
            col_shape.to_section(SectionId(0), "C600".into()),
            beam_shape.to_section(SectionId(1), "B400x700".into()),
        ],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: Some(345.0),
        }],
        ..Default::default()
    };
    let members = [
        (0u32, 0u32, 1u32, 0u32, [1.0, 0.0, 0.0]), // 柱（鉛直）
        (1, 1, 2, 1, [0.0, 0.0, 1.0]),             // 梁（水平）
    ];
    for (id, i, j, sec, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    let mut app = App::default();
    app.load_model(model);
    let checks = app
        .compute_ultimate_checks()
        .expect("RC 矩形部材があれば Ok のはず");
    assert_eq!(checks.len(), 2, "柱・梁の 2 部材が検定される");

    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert_eq!(col.kind, MemberKind::Column);
    assert_eq!(beam.kind, MemberKind::Beam);
    // 各耐力・余裕度が正常に算定される。
    assert!(col.qsu > 0.0 && col.qmu > 0.0 && col.shear_margin > 0.0);
    assert!(col.axial.is_some(), "柱は軸終局耐力を持つ");
    assert!(beam.axial.is_none(), "梁は軸終局耐力なし");
    // 付着検定 ON（既定）なので Qbu も算定される。
    assert!(col.qbu > 0.0 && col.bond_margin.is_finite());
}

/// CFT 柱の軸終局検定を App 経由で算定する。
#[test]
fn test_compute_cft_ultimate_checks() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
    };
    use squid_n_core::section_shape::SectionShape;

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
        sections: vec![cft_shape.to_section(SectionId(0), "CFT400".into())],
        materials: vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "BCR295".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: Some(30.0),
            fy: None,
        }],
        ..Default::default()
    };
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
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
    });

    let mut app = App::default();
    app.load_model(model);
    let checks = app
        .compute_cft_ultimate_checks()
        .expect("CFT 柱があれば Ok のはず");
    assert_eq!(checks.len(), 1);
    assert!(checks[0].ncu > 0.0 && checks[0].ntu > 0.0);
}
