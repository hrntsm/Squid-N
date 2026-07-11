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

/// 壁要素のせん断剛性に乗じる開口低減率 r = 1 − 1.25·√(開口面積/壁面積)
/// （RESP-D 計算編 02「剛性計算」耐震壁の開口低減。式の原典実装は
/// `squid-n-design-jp::wall_opening::opening_reduction_r`。element は design-jp に
/// 依存できないため、面積比による同値式をここで評価する）。
///
/// 壁面積は節点群の包絡寸法（最大水平距離 × 鉛直高さ）で近似する。
/// `Model::wall_attrs` に該当が無い・開口ゼロ・寸法不定では 1.0（低減なし）。
pub(crate) fn wall_opening_reduction(data: &ElementData, model: &Model) -> f64 {
    let Some(attr) = model.wall_attrs.iter().find(|w| w.elem == data.id) else {
        return 1.0;
    };
    // 複数開口の取り扱い（等価/包絡/自動判定）を適用した開口面積。
    // 包絡系モードでは包絡矩形の面積となり、生の面積和より大きくなり得る。
    let opening_area = attr.total_opening_area_for(model.multi_opening_mode);
    if opening_area <= 0.0 {
        return 1.0;
    }
    let coords: Vec<[f64; 3]> = data
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .collect();
    if coords.len() < 3 {
        return 1.0;
    }
    let mut l = 0.0_f64;
    for i in 0..coords.len() {
        for j in (i + 1)..coords.len() {
            let dx = coords[i][0] - coords[j][0];
            let dy = coords[i][1] - coords[j][1];
            l = l.max((dx * dx + dy * dy).sqrt());
        }
    }
    let zs = coords.iter().map(|c| c[2]);
    let h = zs.clone().fold(f64::MIN, f64::max) - zs.fold(f64::MAX, f64::min);
    if l <= 0.0 || h <= 0.0 {
        return 1.0;
    }
    let ratio = (opening_area / (l * h)).clamp(0.0, 1.0);
    (1.0 - 1.25 * ratio.sqrt()).max(0.0)
}

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => {
            // RC 耐震壁の側柱: 面内方向は両端ピンのためモーメント・せん断を
            // 負担しない（RESP-D 計算編 02「側柱の断面性能」）。該当する柱は
            // 面内曲げ面のみ端部回転を静的縮約した要素へ差し替える。
            if let Some(axis) = crate::side_column::wall_side_column_release(data, model) {
                let elem = crate::beam::BeamElement::new(data, model);
                return (
                    Box::new(crate::side_column::InPlaneReleasedColumn::new(elem, axis)),
                    ElemState::default(),
                );
            }
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
        // Wall 要素：壁エレメントモデル（RESP-D 計算編 02。壁柱＋両端ピン剛梁の
        // 4 節点 24 自由度要素）。開口低減率 r は要素内部で考慮される。
        // 耐震壁不成立（フレーム内雑壁）の壁は剛性を周辺の柱・梁の断面性能へ
        // 算入する（beam.rs）ため、壁要素自体は質量のみ保持し剛性は実質ゼロ。
        // 4 節点未満・断面/材料未設定などで構築できない場合は従来の
        // 暫定等価梁にフォールバックする（開口低減 r はせん断剛性に乗じる）。
        ElementKind::Wall => {
            let stiffness_scale = if crate::misc_wall::wall_is_seismic(data, model) {
                1.0
            } else {
                1e-9
            };
            match crate::wall_panel::WallPanelElement::try_new_scaled(data, model, stiffness_scale)
            {
                Some(panel) => (Box::new(panel), ElemState::default()),
                None => {
                    let mut elem = crate::beam::BeamElement::new(data, model);
                    // r=0（開口が壁の 64% 以上）でせん断断面積が 0 になると
                    // ティモシェンコの φ 項が ∞×0 で NaN になるため、微小値を下限とする
                    // （このような壁は本来 RC 耐震壁判定でも不成立となる）。
                    let r = wall_opening_reduction(data, model).max(1e-6);
                    elem.as_y *= r;
                    elem.as_z *= r;
                    elem.a *= stiffness_scale;
                    elem.iy *= stiffness_scale;
                    elem.iz *= stiffness_scale;
                    elem.j *= stiffness_scale;
                    (Box::new(elem), ElemState::default())
                }
            }
        }
        // 一般ブレース：KB = factor·E·A/L（RESP-D マニュアル計算編02）。
        // 引張専用ブレースは弾性解析で剛性を1/2にモデル化する（factor=0.5）。
        ElementKind::Brace { tension_only } => {
            let factor = if tension_only { 0.5 } else { 1.0 };
            (
                Box::new(crate::truss::TrussElement::new(data, model, factor)),
                ElemState::default(),
            )
        }
        // 節点バネ：RESP-D マニュアル計算編03「応力解析」§部材の変形と自由度。
        // 局所軸ごとに独立な弾性バネ（軸・せん断・曲げ回転。ねじりは既定 0）。
        ElementKind::NodalSpring => (
            Box::new(crate::spring::NodalSpringElement::new(data, model)),
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
        // 一般ブレース(弾塑性): 初期剛性1倍(RESP-D計算編02)。引張専用は
        // 圧縮側の剛性・軸力を実質ゼロとするスラック挙動でモデル化する。
        ElementKind::Brace { tension_only } => {
            let truss = if tension_only {
                crate::truss::TrussElement::new_tension_only_nonlinear(data, model)
            } else {
                crate::truss::TrussElement::new(data, model, 1.0)
            };
            (Box::new(truss), ElemState::default())
        }
        // PanelZone / Shell / Wall / NodalSpring は現状の挙動（弾性ベース）を踏襲。
        // 節点バネは非線形解析でも常に弾性のまま（スケルトン未対応）。
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
            spring: None,
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
            spring: None,
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
            spring: None,
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
            spring: None,
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
            spring: None,
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
            spring: None,
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
            spring: None,
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

    /// ブレース要素の生成モデル用（2 節点・軸方向 4000mm・断面積 2000mm2）。
    fn make_brace_model(tension_only: bool) -> (Model, ElementData) {
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
                    coord: [4000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "brace".into(),
                area: 2000.0,
                iy: 0.0,
                iz: 0.0,
                j: 0.0,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            ..Default::default()
        };
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Brace { tension_only },
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
        };
        (model, elem)
    }

    /// 一般ブレース（引張専用でない）: build_behavior は factor=1.0 の TrussElement
    /// を生成し、軸剛性 K = E·A/L に一致する（RESP-D マニュアル計算編02）。
    #[test]
    fn test_build_behavior_brace_normal_full_stiffness() {
        let (model, elem) = make_brace_model(false);
        let (behavior, state) = build_behavior(&elem, &model);
        let ctx = crate::behavior::Ctx { model: &model };
        let k = behavior.tangent_stiffness(&state, &ctx);
        let ea_l = 205000.0 * 2000.0 / 4000.0;
        assert!((k.get(0, 0) - ea_l).abs() < 1e-6, "k00={}", k.get(0, 0));
    }

    /// 引張専用ブレース: 弾性解析（build_behavior）では剛性を1/2にモデル化する
    /// （マニュアル「引張と圧縮が対で存在するとみなし、弾性解析では剛性を1/2」）。
    #[test]
    fn test_build_behavior_brace_tension_only_half_stiffness() {
        let (model, elem) = make_brace_model(true);
        let (behavior, state) = build_behavior(&elem, &model);
        let ctx = crate::behavior::Ctx { model: &model };
        let k = behavior.tangent_stiffness(&state, &ctx);
        let ea_l = 205000.0 * 2000.0 / 4000.0;
        assert!(
            (k.get(0, 0) - 0.5 * ea_l).abs() < 1e-6,
            "k00={}",
            k.get(0, 0)
        );
    }

    /// 引張専用ブレース: 弾塑性解析（build_nonlinear_behavior）では初期剛性を
    /// 1倍とする（マニュアル「弾塑性解析の場合は初期剛性は1倍とする」）。
    #[test]
    fn test_build_nonlinear_behavior_brace_tension_only_full_stiffness() {
        let (model, elem) = make_brace_model(true);
        let (behavior, state) = build_nonlinear_behavior(&elem, &model);
        let ctx = crate::behavior::Ctx { model: &model };
        let k = behavior.tangent_stiffness(&state, &ctx);
        let ea_l = 205000.0 * 2000.0 / 4000.0;
        assert!((k.get(0, 0) - ea_l).abs() < 1e-6, "k00={}", k.get(0, 0));
    }

    /// 壁要素の開口低減: wall_attrs の開口面積からせん断剛性が低減されること
    /// （RESP-D 計算編 02「剛性計算」耐震壁の開口低減 r=1−1.25·√(開口面積/壁面積)）。
    #[test]
    fn test_build_behavior_wall_opening_reduces_shear_stiffness() {
        use squid_n_core::model::WallAttr;

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let mut model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "wall".into(),
                area: 150.0 * 1000.0,
                iy: 1.0e9,
                iz: 1.0e9,
                j: 1.0e9,
                depth: 1000.0,
                width: 150.0,
                as_y: 125_000.0,
                as_z: 125_000.0,
                panel_thickness: None,
                thickness: Some(150.0),
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 0.0,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let wall = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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

        // 壁エレメント(24自由度)の面内せん断・鉛直軸のエネルギーパターン。
        // 内部節点順は z ソート([0,1] 下辺, [3,2] 上辺)のため、上辺の
        // スロットは 2(node3)・3(node2)。
        let shear_pattern = |k: &crate::behavior::LocalMat| -> f64 {
            let mut u = [0.0; 24];
            u[2 * 6] = 1.0;
            u[3 * 6] = 1.0;
            let mut s = 0.0;
            for i in 0..24 {
                for j in 0..24 {
                    s += u[i] * k.get(i, j) * u[j];
                }
            }
            s
        };
        let axial_pattern = |k: &crate::behavior::LocalMat| -> f64 {
            let mut u = [0.0; 24];
            u[2 * 6 + 2] = 1.0;
            u[3 * 6 + 2] = 1.0;
            let mut s = 0.0;
            for i in 0..24 {
                for j in 0..24 {
                    s += u[i] * k.get(i, j) * u[j];
                }
            }
            s
        };

        // 開口なし
        let (b_no, state) = build_behavior(&wall, &model);
        let ctx = crate::behavior::Ctx { model: &model };
        let k_no = b_no.tangent_stiffness(&state, &ctx);

        // 開口 10%（壁 4000×3000=12e6 mm² に対し 1.2e6 mm²）→ r0=0.316(耐震壁
        // 成立のまま)、r=1−1.25·0.316=0.605
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 1.2e6,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        let (b_open, state2) = build_behavior(&wall, &model);
        let ctx2 = crate::behavior::Ctx { model: &model };
        let k_open = b_open.tangent_stiffness(&state2, &ctx2);

        // 個別開口(合計 1.2e6 mm²)は面積のみ指定と同じ低減率になる。
        // また opening_area(古い値)より個別開口が優先される。
        model.wall_attrs[0] = WallAttr {
            elem: ElemId(0),
            opening_area: 1.0, // 無視される(個別開口が優先)
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![
                squid_n_core::model::WallOpening {
                    width: 1000.0,
                    height: 800.0,
                    offset: Some([0.0, 500.0]),
                },
                squid_n_core::model::WallOpening {
                    width: 500.0,
                    height: 800.0,
                    offset: Some([1800.0, 500.0]),
                },
            ],
        };
        let (b_dims, state3) = build_behavior(&wall, &model);
        let ctx3 = crate::behavior::Ctx { model: &model };
        let k_dims = b_dims.tangent_stiffness(&state3, &ctx3);
        assert!(
            (shear_pattern(&k_dims) - shear_pattern(&k_open)).abs() < 1e-6,
            "個別開口(Σ1.2e6)と面積のみ(1.2e6)の低減が一致しない: {} vs {}",
            shear_pattern(&k_dims),
            shear_pattern(&k_open)
        );

        // 包絡モード: 離れた2開口の包絡矩形(2300×800=1.84e6、r0=0.39≦0.4 で
        // 耐震壁成立のまま)により低減がさらに大きくなる
        model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Envelope;
        let (b_env, state4) = build_behavior(&wall, &model);
        let ctx4 = crate::behavior::Ctx { model: &model };
        let k_env = b_env.tangent_stiffness(&state4, &ctx4);
        assert!(
            shear_pattern(&k_env) < shear_pattern(&k_dims) * 0.999,
            "包絡モードで低減が強まらない: env={} eq={}",
            shear_pattern(&k_env),
            shear_pattern(&k_dims)
        );
        model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Equivalent;

        // せん断剛性の低減で面内せん断が小さくなる（鉛直軸剛性 EA/h は不変）
        assert!(
            shear_pattern(&k_open) < shear_pattern(&k_no) * 0.999,
            "shear open={} no={}",
            shear_pattern(&k_open),
            shear_pattern(&k_no)
        );
        assert!((axial_pattern(&k_open) - axial_pattern(&k_no)).abs() < 1e-6);
    }
}
