use super::*;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, IsolatorAttr, IsolatorKind, IsolatorProps,
    LocalAxis, Node,
};

fn iso_model(props: IsolatorProps) -> Model {
    Model {
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
                coord: [0.0, 0.0, 1000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Isolator,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
        }],
        sections: vec![],
        materials: vec![],
        isolator_attrs: vec![IsolatorAttr {
            elem: ElemId(0),
            props,
        }],
        ..Default::default()
    }
}

fn laminated() -> IsolatorProps {
    IsolatorProps {
        kind: IsolatorKind::LaminatedRubber,
        k1: 2000.0,
        k2: 200.0,
        qd: 100_000.0,
        kv: 5_000_000.0,
        mu: 0.0,
        n_long: 0.0,
        n_springs: 8,
        ..Default::default()
    }
}

fn friction() -> IsolatorProps {
    IsolatorProps {
        kind: IsolatorKind::ElasticSliding,
        k1: 10_000.0,
        k2: 0.0,
        qd: 0.0,
        kv: 5_000_000.0,
        mu: 0.1,
        n_long: 1_000_000.0, // Qmax = 0.1×1e6 = 100kN
        n_springs: 8,
        ..Default::default()
    }
}

/// 高減衰ゴム（歪依存）: ゴム総厚 H=200mm、γ が大きいほど二次剛性・特性耐力が低下する
/// 歪依存係数（CKd=1−0.5γ、CQd=1−0.3γ）。
fn hdr_strain_dependent() -> IsolatorProps {
    IsolatorProps {
        kind: IsolatorKind::HighDampingRubber,
        k1: 2000.0,
        k2: 200.0,
        qd: 100_000.0,
        kv: 5_000_000.0,
        mu: 0.0,
        n_long: 0.0,
        n_springs: 2,
        total_rubber_thickness: 200.0,
        ckd_gamma: [1.0, -0.5, 0.0],
        cqd_gamma: [1.0, -0.3, 0.0],
    }
}

/// 節点 j に水平（global X）変位 δ を与えたときの局所内力を返す。
fn push_horizontal(elem: &mut IsolatorElement, ctx: &Ctx, delta: f64) -> LocalVec {
    let mut du = LocalVec {
        data: smallvec::smallvec![0.0; 12],
    };
    du.data[6] = delta; // node j, global ux
    elem.update_state(&du, true, ctx);
    elem.internal_force(&ElemState::default(), ctx)
}

fn horiz_resultant(f: &LocalVec) -> f64 {
    (f.data[6] * f.data[6] + f.data[7] * f.data[7]).sqrt()
}

#[test]
fn test_laminated_elastic_horizontal_stiffness() {
    let model = iso_model(laminated());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    // 弾性域（降伏変位 δy=Qd/K1=50mm 未満）。δ=10mm → |F|=K1·δ=20kN。
    let f = push_horizontal(&mut elem, &ctx, 10.0);
    assert!(
        (horiz_resultant(&f) - 2000.0 * 10.0).abs() < 1.0,
        "elastic |F|={} expected {}",
        horiz_resultant(&f),
        2000.0 * 10.0
    );
}

#[test]
fn test_laminated_yields_past_qd() {
    let model = iso_model(laminated());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    // δy=Qd/K1=50mm。δ=150mm（降伏後）。二次剛性 K2=200 なので
    // |F|=Qd+K2·(δ−δy)=100000+200·100=120kN < 弾性外挿 K1·δ=300kN。
    let f = push_horizontal(&mut elem, &ctx, 150.0);
    let fr = horiz_resultant(&f);
    assert!(fr < 2000.0 * 150.0, "降伏で剛性低下: {fr}");
    assert!(
        (fr - 120_000.0).abs() < 500.0,
        "バイリニア降伏後 |F|={fr} 期待 120kN"
    );
}

#[test]
fn test_vertical_axial_elastic() {
    let model = iso_model(laminated());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    let mut du = LocalVec {
        data: smallvec::smallvec![0.0; 12],
    };
    du.data[8] = 2.0; // node j, global uz（鉛直）
    elem.update_state(&du, true, &ctx);
    let f = elem.internal_force(&ElemState::default(), &ctx);
    // 鉛直軸力 = Kv·δ = 5e6·2 = 1e7 N。
    assert!(
        (f.data[8].abs() - 5_000_000.0 * 2.0).abs() < 1.0,
        "鉛直軸剛性 Fz={}",
        f.data[8]
    );
}

#[test]
fn test_friction_slips_at_qmax() {
    let model = iso_model(friction());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    // Qmax=μN=100kN, K1=10000 → 滑り出し変位 = 10mm。
    // 弾性: δ=5mm → |F|=50kN。
    let f1 = push_horizontal(&mut elem, &ctx, 5.0);
    assert!(
        (horiz_resultant(&f1) - 50_000.0).abs() < 10.0,
        "摩擦弾性 |F|={}",
        horiz_resultant(&f1)
    );
    // 滑り: δ=50mm → |F| は Qmax=100kN で頭打ち。
    let mut elem2 = IsolatorElement::new(&model.elements[0], &model);
    let f2 = push_horizontal(&mut elem2, &ctx, 50.0);
    assert!(
        (horiz_resultant(&f2) - 100_000.0).abs() < 100.0,
        "摩擦滑り |F|={} 期待 Qmax=100kN",
        horiz_resultant(&f2)
    );
}

#[test]
fn test_commit_revert_roundtrip() {
    let model = iso_model(laminated());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    let f_committed = push_horizontal(&mut elem, &ctx, 80.0); // 降伏後をコミット

    // さらに trial して revert → コミット状態へ戻る。
    let mut du = LocalVec {
        data: smallvec::smallvec![0.0; 12],
    };
    du.data[6] = 40.0;
    elem.update_state(&du, false, &ctx);
    elem.revert_state();
    let zero = LocalVec {
        data: smallvec::smallvec![0.0; 12],
    };
    elem.update_state(&zero, false, &ctx);
    let f_reverted = elem.internal_force(&ElemState::default(), &ctx);
    for i in 0..12 {
        approx::assert_relative_eq!(
            f_committed.data[i],
            f_reverted.data[i],
            epsilon = 1e-3,
            max_relative = 1e-6
        );
    }
}

#[test]
fn test_hdr_strain_dependent_softens_with_strain() {
    // 高減衰ゴム（歪依存）: 弾性域は K1·δ、降伏後は γ 依存で剛性・耐力が低下する。
    let model = iso_model(hdr_strain_dependent());
    let ctx = Ctx { model: &model };
    let mut elem = IsolatorElement::new(&model.elements[0], &model);
    // 弾性域（δ=10mm < δy=50mm）: 歪依存でも弾性は K1·δ=20kN。
    let f_el = push_horizontal(&mut elem, &ctx, 10.0);
    assert!(
        (horiz_resultant(&f_el) - 2000.0 * 10.0).abs() < 1.0,
        "elastic |F|={}",
        horiz_resultant(&f_el)
    );

    // 降伏後（δ=100mm, γ=0.5）: 歪依存で耐力が定数バイリニアより低下。
    let f_sd = {
        let mut e = IsolatorElement::new(&model.elements[0], &model);
        horiz_resultant(&push_horizontal(&mut e, &ctx, 100.0))
    };
    let model_c = iso_model(laminated()); // 同 K1/K2/Qd の定数バイリニア。
    let ctx_c = Ctx { model: &model_c };
    let f_const = {
        let mut e = IsolatorElement::new(&model_c.elements[0], &model_c);
        horiz_resultant(&push_horizontal(&mut e, &ctx_c, 100.0))
    };
    assert!(
        f_sd < f_const,
        "歪依存で耐力低下すべき: 歪依存 {f_sd} < 定数 {f_const}"
    );
    assert!(f_sd > 0.0);
}
