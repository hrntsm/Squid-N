use crate::behavior::{ElemState, ElementBehavior};
use squid_n_core::model::{ElementData, ElementKind, ForceRegime, Model};
use squid_n_material::uniaxial::Bilinear;

/// ForceRegime の自動選択結果（P5 §5）
pub enum ResolvedRegime {
    ConcentratedSpring,
    Fiber,
}

/// ForceRegime::Auto をトポロジから判定する（P5 §5）
/// 剛床所属の階かつ梁で軸力変動が小 → ConcentratedSpring
/// それ以外 → Fiber
pub fn resolve_force_regime(data: &ElementData, model: &Model) -> ResolvedRegime {
    if data.force_regime != ForceRegime::Auto {
        return match data.force_regime {
            ForceRegime::UniaxialBendingShear => ResolvedRegime::ConcentratedSpring,
            ForceRegime::AxialBendingInteract => ResolvedRegime::Fiber,
            ForceRegime::Auto => unreachable!(),
        };
    }

    // Auto の判定ロジック（ヒューリスティック）
    // 剛床に所属する梁（= 鉛直軸でない部材）は集中ばね
    let is_vertical = is_vertical_member(data, model);
    let on_rigid_diaphragm = is_on_rigid_diaphragm(data, model);

    if on_rigid_diaphragm && !is_vertical {
        ResolvedRegime::ConcentratedSpring
    } else {
        ResolvedRegime::Fiber
    }
}

fn is_vertical_member(data: &ElementData, model: &Model) -> bool {
    if data.nodes.len() < 2 {
        return false;
    }
    let n0 = &model.nodes.get(data.nodes[0].index());
    let n1 = &model.nodes.get(data.nodes[1].index());
    match (n0, n1) {
        (Some(n0), Some(n1)) => {
            let dz = (n1.coord[2] - n0.coord[2]).abs();
            let dx = (n1.coord[0] - n0.coord[0]).abs();
            let dy = (n1.coord[1] - n0.coord[1]).abs();
            dz > (dx + dy) * 0.5
        }
        _ => false,
    }
}

fn is_on_rigid_diaphragm(data: &ElementData, model: &Model) -> bool {
    let elem_nodes: Vec<squid_n_core::ids::NodeId> = data.nodes.iter().copied().collect();
    for story in &model.stories {
        for dia in &story.diaphragms {
            if elem_nodes
                .iter()
                .any(|n| *n == dia.master || dia.slaves.contains(n))
            {
                return true;
            }
        }
    }
    for c in &model.constraints {
        if let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c {
            if elem_nodes
                .iter()
                .any(|n| *n == *master || slaves.contains(n))
            {
                return true;
            }
        }
    }
    false
}

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => {
            // ForceRegime に基づいて要素種別を選択（P5 §5）
            let regime = resolve_force_regime(data, model);
            match regime {
                ResolvedRegime::ConcentratedSpring => {
                    let elem = crate::beam::BeamElement::new(data, model);
                    let (spring_i, spring_j) = build_rotational_springs(data, model);
                    (
                        Box::new(
                            crate::concentrated::ConcentratedSpringBeam::new_one_component(
                                elem, spring_i, spring_j,
                            ),
                        ),
                        ElemState::default(),
                    )
                }
                ResolvedRegime::Fiber => {
                    // T2: FiberBeam が実装されるまでの暫定 BeamElement
                    let elem = crate::beam::BeamElement::new(data, model);
                    (Box::new(elem), ElemState::default())
                }
            }
        }
        ElementKind::PanelZone => (
            Box::new(crate::panel::PanelZone::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Shell => (
            Box::new(crate::shell::ShellElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Ms => (
            Box::new(crate::ms::MsElement::new(data, model)),
            ElemState::default(),
        ),
        // Fiber 要素：将来 FiberBeam が実装されるまでの暫定 BeamElement
        ElementKind::Fiber => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        // Wall 要素：将来 TvlemWall が実装されるまでの暫定 BeamElement
        ElementKind::Wall => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        // Brace 要素：軸材。剛性は当面 BeamElement スタブ（両端条件で軸材化は呼び出し側）
        ElementKind::Brace => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
    }
}

/// 非線形解析（pushover）用の要素生成。`ForceRegime` に基づき非線形要素を構築する（P5 §5）。
///
/// 線形弾性解析は従来どおり [`build_behavior`]（弾性 `BeamElement`）を使う。両者を分けるのは、
/// `resolve_force_regime` が剛床に乗らない梁も Fiber へ振り分けるため、共通化すると
/// 線形解析の弾性梁まで非線形要素に置き換わってしまうため。
///
/// 注意（既知の制約）: `ConcentratedSpringBeam` は端ばねスケルトン（降伏モーメント）が必要だが、
/// 現状 `Model` に降伏応力／スケルトン供給経路が無いため、軸-曲げ連成を扱う `FiberBeam` に
/// フォールバックしている（P5 §5 の本来意図は集中ばね梁）。また鋼材はファイバ材料が
/// `Bilinear(My=1e20)` で実質弾性のため、真の降伏は `fc` を持つコンクリート断面でのみ生じる。
/// 鋼材の降伏・集中ばね梁の実体化には Model への降伏応力／スケルトン追加が前提（follow-up）。
pub fn build_nonlinear_behavior(
    data: &ElementData,
    model: &Model,
) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => match resolve_force_regime(data, model) {
            ResolvedRegime::ConcentratedSpring => {
                let elem = crate::beam::BeamElement::new(data, model);
                let (spring_i, spring_j) = build_rotational_springs(data, model);
                // 端バネの N-M 相関（2バネ連成: M_lim = My0·(1-|N|/N許容)）。
                // My0 はバネ生成と同じ弾性断面係数ベース、N許容 = σy·A。
                let (my0, n_allow) = yield_moment_and_axial(data, model);
                (
                    Box::new(
                        crate::concentrated::ConcentratedSpringBeam::new_one_component(
                            elem, spring_i, spring_j,
                        )
                        .with_mn_interaction(my0, n_allow),
                    ),
                    ElemState::default(),
                )
            }
            ResolvedRegime::Fiber => (Box::new(build_fiber(data, model)), ElemState::default()),
        },
        ElementKind::Fiber => (Box::new(build_fiber(data, model)), ElemState::default()),
        // MS 要素: 端部バネ断面 + 中央弾性の非線形要素（P5.5 §3）
        ElementKind::Ms => (
            Box::new(crate::ms::MsElement::new(data, model)),
            ElemState::default(),
        ),
        // PanelZone / Shell / Wall は現状の挙動（弾性ベース）を踏襲。
        _ => build_behavior(data, model),
    }
}

/// ファイバー梁の生成。既定で塑性化域考慮モデル（端部 Lp 区間にファイバー断面、
/// 中央弾性）とし、Lp は `plastic_zone` 指定値、未指定なら断面せいの 0.5 倍
/// （MS 要素と同じ既定。0.5D は既往検討で標準的に用いられる値）。
fn build_fiber(data: &ElementData, model: &Model) -> crate::fiber_elem::FiberBeam {
    let depth = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .map(|s| s.depth)
        .filter(|d| *d > 0.0)
        .unwrap_or(200.0);
    let lp = data.plastic_zone.unwrap_or(0.5 * depth);
    crate::fiber_elem::FiberBeam::with_plastic_zone(data, model, lp)
}

/// 集中バネの降伏モーメント My0 と軸許容耐力 N許容 = σy·A（MN 相関用）。
fn yield_moment_and_axial(data: &ElementData, model: &Model) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let fy_sigma = mat.and_then(|m| m.fy).unwrap_or(235.0);
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let z = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    let area = sec.map(|s| s.area).unwrap_or(1.0e4);
    (fy_sigma * z, fy_sigma * area)
}

fn build_rotational_springs(
    data: &ElementData,
    model: &Model,
) -> (
    Box<dyn squid_n_material::uniaxial::UniaxialMaterial>,
    Box<dyn squid_n_material::uniaxial::UniaxialMaterial>,
) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let e = mat.map(|m| m.young).unwrap_or(205000.0);
    let fy_sigma = mat.and_then(|m| m.fy).unwrap_or(235.0);
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let z = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    let my = fy_sigma * z;

    let n0 = &model.nodes[data.nodes[0].index()];
    let n1 = &model.nodes[data.nodes[1].index()];
    let l = ((n1.coord[0] - n0.coord[0]).powi(2)
        + (n1.coord[1] - n0.coord[1]).powi(2)
        + (n1.coord[2] - n0.coord[2]).powi(2))
    .sqrt();
    // 集中ばねの初期剛性は可とう長 L'（= L − 剛域長。§6.2.1）基準で評価する。
    // 剛域があるのに節点間長 L で評価すると初期剛性を過小評価するため、
    // L' ≤ 0（剛域が全長を占める異常値）の場合のみ L にフォールバックする。
    let l_flex = l - data.rigid_zone.length_i - data.rigid_zone.length_j;
    let l_eff = if l_flex > 0.0 { l_flex } else { l };
    let k_rot = if l_eff > 0.0 {
        6.0 * e * iz / l_eff
    } else {
        1.0e12
    };

    let spring_i = Box::new(Bilinear::new(k_rot, my, 0.01));
    let spring_j = Box::new(Bilinear::new(k_rot, my, 0.01));
    (spring_i, spring_j)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{EndCondition, LocalAxis, Material, Node, Section};

    fn make_diaphragm_model() -> Model {
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
                    coord: [5000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
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
            ],
            constraints: vec![squid_n_core::model::Constraint::RigidDiaphragm {
                story: squid_n_core::ids::StoryId(0),
                master: NodeId(2),
                slaves: vec![NodeId(1)],
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".into(),
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
                id: MaterialId(0),
                name: "mat".into(),
                young: 20000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_resolve_force_regime_explicit() {
        let model = make_diaphragm_model();
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
            force_regime: ForceRegime::UniaxialBendingShear,
            rigid_zone: Default::default(),
            plastic_zone: None,
        };
        assert!(matches!(
            resolve_force_regime(&elem, &model),
            ResolvedRegime::ConcentratedSpring
        ));
    }

    #[test]
    fn test_resolve_force_regime_auto() {
        let model = make_diaphragm_model();
        // 水平部材＋剛床あり → ConcentratedSpring
        let beam = ElementData {
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
        };
        assert!(matches!(
            resolve_force_regime(&beam, &model),
            ResolvedRegime::ConcentratedSpring
        ));

        // 鉛直部材 → Fiber
        let col = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
        };
        assert!(matches!(
            resolve_force_regime(&col, &model),
            ResolvedRegime::Fiber
        ));
    }

    #[test]
    fn test_build_behavior_concentrated_spring_uses_spring_beam() {
        let model = make_diaphragm_model();
        let beam = ElementData {
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
        };
        let (behavior, _state) = build_behavior(&beam, &model);
        // ConcentratedSpringBeam は recover_forces を override していないので None
        assert!(
            behavior.recover_forces(&[0.0; 12]).is_none(),
            "ConcentratedSpringBeam should return None for recover_forces"
        );
        // snapshot_state で ConcentratedSpringBeam 固有型を確認
        let snap = behavior.snapshot_state();
        let is_spring = snap
            .downcast_ref::<(
                Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>,
                f64,
                f64,
                f64,
                f64,
            )>()
            .is_some();
        assert!(
            is_spring,
            "should be ConcentratedSpringBeam by snapshot type"
        );
    }

    #[test]
    fn test_build_behavior_fiber_still_fiber() {
        let model = make_diaphragm_model();
        let col = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
        };
        let (behavior, _state) = build_behavior(&col, &model);
        // Fiber 分岐は暫定 BeamElement（線形解析）→ recover_forces は Some
        assert!(
            behavior.recover_forces(&[0.0; 12]).is_some(),
            "Fiber regime should use BeamElement for linear analysis"
        );
        assert_eq!(behavior.n_dof(), 12);
    }

    #[test]
    fn test_build_nonlinear_behavior_concentrated_spring_uses_spring_beam() {
        let model = make_diaphragm_model();
        let beam = ElementData {
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
        };
        let (behavior, _state) = build_nonlinear_behavior(&beam, &model);
        let snap = behavior.snapshot_state();
        let is_spring = snap
            .downcast_ref::<(
                Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>,
                f64,
                f64,
                f64,
                f64,
            )>()
            .is_some();
        assert!(
            is_spring,
            "nonlinear ConcentratedSpring should be ConcentratedSpringBeam"
        );
    }

    #[test]
    fn test_build_nonlinear_behavior_fiber_uses_fiber_beam() {
        let model = make_diaphragm_model();
        let col = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
        };
        let (behavior, _state) = build_nonlinear_behavior(&col, &model);
        let snap = behavior.snapshot_state();
        let is_fiber = snap
            .downcast_ref::<(
                [f64; 12],
                [f64; 12],
                Vec<Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>>,
            )>()
            .is_some();
        assert!(is_fiber, "nonlinear Fiber should be FiberBeam");
    }
}
