use crate::behavior::{ElemState, ElementBehavior};
use squid_n_core::model::{
    default_member_hysteresis, ElementData, ElementKind, ForceRegime, HysteresisModel, Model,
};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};
use squid_n_material::{HysteresisMaterial, HysteresisRule, SteelBuckling, TsujiYamada};

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
/// （RC規準（耐震壁）の開口低減。式の原典実装は
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
            // 負担しない（RC規準の耐震壁規定・側柱の断面性能）。該当する柱は
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
        // Wall 要素：壁エレメントモデル（壁エレメント置換モデル。壁柱＋両端ピン剛梁の
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
        // 一般ブレース：KB = factor·E·A/L（材料力学・トラス要素）。
        // 引張専用ブレースは弾性解析で剛性を1/2にモデル化する（factor=0.5）。
        ElementKind::Brace { tension_only } => {
            let factor = if tension_only { 0.5 } else { 1.0 };
            (
                Box::new(crate::truss::TrussElement::new(data, model, factor)),
                ElemState::default(),
            )
        }
        // 節点バネ：構造力学（部材の変形と自由度）。
        // 局所軸ごとに独立な弾性バネ（軸・せん断・曲げ回転。ねじりは既定 0）。
        ElementKind::NodalSpring => (
            Box::new(crate::spring::NodalSpringElement::new(data, model)),
            ElemState::default(),
        ),
        // 免震支承材：各免震部材指針・製品技術資料（Category B）。
        // 水平は非線形せん断（積層ゴム系バイリニア／摩擦ばね）、鉛直は弾性軸。
        ElementKind::Isolator => (
            Box::new(crate::isolator::IsolatorElement::new(data, model)),
            ElemState::default(),
        ),
        // 制振ダンパー（制振部材の力学モデル）。種別で要素を切替える。
        // - マクスウェル（速度依存型）: 静的・線形では Δt=0 で不活性、時刻歴で活性化。
        // - 履歴型バイリニア（鋼材系）: 変位依存の弾塑性軸ばね（静的・動的で作用）。
        ElementKind::Damper => {
            use squid_n_core::model::DamperKind;
            let kind = model.damper_props(data.id).unwrap_or_default().kind;
            let beh: Box<dyn ElementBehavior> = match kind {
                DamperKind::Maxwell => {
                    Box::new(crate::damper::MaxwellDamperElement::new(data, model))
                }
                DamperKind::HystereticBilinear => {
                    Box::new(crate::damper::HystereticDamperElement::new(data, model))
                }
            };
            (beh, ElemState::default())
        }
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
                // 履歴則を解決（部材個別指定 → 構造種別ごとの既定表。本実装の既定の
                // 非線形特性は各履歴則の原典に基づく）。RC/SRC/CFT 梁は
                // 武田型トリリニア、S 梁は標準型（kinematic バイリニア）を材端バネに用いる。
                let rule = resolve_member_hysteresis(data, model);
                let (spring_i, spring_j, use_mn) = build_flexural_springs(data, model, rule);
                let beam = crate::concentrated::ConcentratedSpringBeam::new_one_component(
                    elem, spring_i, spring_j,
                );
                // 端バネの N-M 相関（M_lim = My0·(1-|N|/N許容)）はバイリニア（標準型）
                // のみ適用（`set_yield` 対応）。武田型等の履歴材料は骨格固定のため対象外。
                let beam = if use_mn {
                    let (my0, n_allow) = yield_moment_and_axial(data, model);
                    beam.with_mn_interaction(my0, n_allow)
                } else {
                    beam
                };
                (Box::new(beam), ElemState::default())
            }
            ResolvedRegime::Fiber => (Box::new(build_fiber(data, model)), ElemState::default()),
        },
        ElementKind::Fiber => (Box::new(build_fiber(data, model)), ElemState::default()),
        // MS 要素: 端部バネ断面 + 中央弾性の非線形要素（P5.5 §3）
        ElementKind::Ms => (
            Box::new(crate::ms::MsElement::new(data, model)),
            ElemState::default(),
        ),
        // 一般ブレース(弾塑性): 初期剛性1倍(材料力学・トラス要素)。引張専用は
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

/// 部材の曲げ終局（降伏）モーメント My [N·mm]（技術基準解説書の曲げ終局強度）。
/// RC=0.9·at·σy·j（[`squid_n_core::rc_capacity::rc_mu_simple`]）、鉄骨=Zp·σy（全塑性 Mp）、
/// それ以外（複合断面・形状不明）は σy·Z弾性でフォールバックする。
/// 従来の材端バネは σy·Z弾性を用いていたが、規準の曲げ終局強度へ改良する。
fn flexural_yield_moment(data: &ElementData, model: &Model) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let ze = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    let fy = mat.and_then(|m| m.fy);
    match sec.and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::RcRect { rebar, d, .. }) | Some(SectionShape::RcCircle { rebar, d }) => {
            let sy = fy.unwrap_or(345.0);
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
            let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
            let my = squid_n_core::rc_capacity::rc_mu_simple(
                &squid_n_core::rc_capacity::RcCapacityInput {
                    b: 1.0,
                    d: *d,
                    at,
                    d_eff,
                    sigma_y: sy,
                    fc: fc.max(1e-9),
                    pw: 0.0,
                    sigma_wy: 0.0,
                    clear_span: 1.0,
                    sigma_0: 0.0,
                },
            );
            if my > 0.0 {
                my
            } else {
                sy * ze
            }
        }
        Some(shape) => {
            let sy = fy.unwrap_or(235.0);
            match shape.plastic_modulus_strong() {
                Some(zp) => sy * zp,
                None => sy * ze,
            }
        }
        None => fy.unwrap_or(235.0) * ze,
    }
}

/// 集中バネの降伏モーメント My0 と軸許容耐力 N許容 = σy·A（MN 相関用）。
fn yield_moment_and_axial(data: &ElementData, model: &Model) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let fy_sigma = mat.and_then(|m| m.fy).unwrap_or(235.0);
    let area = sec.map(|s| s.area).unwrap_or(1.0e4);
    (flexural_yield_moment(data, model), fy_sigma * area)
}

/// 部材の可撓長さ [mm]（= 節点間長 − 両端剛域長。剛域控除後が非正なら全長）。
fn flexible_length(data: &ElementData, model: &Model) -> f64 {
    let n0 = &model.nodes[data.nodes[0].index()];
    let n1 = &model.nodes[data.nodes[1].index()];
    let l = ((n1.coord[0] - n0.coord[0]).powi(2)
        + (n1.coord[1] - n0.coord[1]).powi(2)
        + (n1.coord[2] - n0.coord[2]).powi(2))
    .sqrt();
    let l_flex = l - data.rigid_zone.length_i - data.rigid_zone.length_j;
    if l_flex > 0.0 {
        l_flex
    } else {
        l
    }
}

/// 材端曲げバネの初期回転剛性 k_rot [N·mm/rad] と降伏モーメント My [N·mm]。
/// k_rot は可とう長 L'（= L − 剛域長。§6.2.1）基準で評価する。
fn rotational_spring_params(data: &ElementData, model: &Model) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let e = mat.map(|m| m.young).unwrap_or(205000.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    // 材端バネの降伏モーメントは規準の曲げ終局強度（RC=0.9·at·σy·d、鉄骨=Zp·σy）を用いる。
    let my = flexural_yield_moment(data, model);

    let l_eff = flexible_length(data, model);
    let k_rot = if l_eff > 0.0 {
        6.0 * e * iz / l_eff
    } else {
        1.0e12
    };
    (k_rot, my)
}

fn build_rotational_springs(
    data: &ElementData,
    model: &Model,
) -> (Box<dyn UniaxialMaterial>, Box<dyn UniaxialMaterial>) {
    let (k_rot, my) = rotational_spring_params(data, model);
    let spring_i = Box::new(Bilinear::new(k_rot, my, 0.01));
    let spring_j = Box::new(Bilinear::new(k_rot, my, 0.01));
    (spring_i, spring_j)
}

/// 断面形状が RC/SRC/CFT（コンクリート系）か否か（既定履歴則の判定用）。
fn is_rc_like_section(data: &ElementData, model: &Model) -> bool {
    use squid_n_core::section_shape::SectionShape;
    matches!(
        data.section
            .and_then(|sid| model.sections.get(sid.index()))
            .and_then(|s| s.shape.as_ref()),
        Some(
            SectionShape::RcRect { .. }
                | SectionShape::RcCircle { .. }
                | SectionShape::SrcRect { .. }
                | SectionShape::CftBox { .. }
                | SectionShape::CftPipe { .. }
                | SectionShape::RcWall { .. }
        )
    )
}

/// 部材の履歴則を解決する（属性 override → 構造種別ごとの既定表。本実装の既定の
/// 非線形特性は各履歴則の原典に基づく）。`HysteresisModel::Auto` は
/// 構造種別ごとの既定（RC/SRC/CFT=武田型、S=標準型）へ解決される。UI 表示にも用いる。
pub fn resolve_member_hysteresis(data: &ElementData, model: &Model) -> HysteresisModel {
    match model.member_hysteresis(data.id) {
        Some(r) if r != HysteresisModel::Auto => r,
        _ => default_member_hysteresis(is_rc_like_section(data, model)),
    }
}

/// 材端曲げバネのひび割れモーメント Mc [N·mm]。RC 系は Mc=0.56·√Fc·Ze
/// （Fc [N/mm²]、Ze=断面係数。技術基準解説書 P.621-623）、それ以外は My/3 で
/// 近似する。
fn crack_moment(data: &ElementData, model: &Model, my: f64) -> f64 {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let ze = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    match (is_rc_like_section(data, model), mat.and_then(|m| m.fc)) {
        (true, Some(fc)) if fc > 0.0 && ze > 0.0 => {
            (0.56 * fc.sqrt() * ze).clamp(my * 0.1, my * 0.9)
        }
        _ => my / 3.0,
    }
}

/// 材端曲げバネの降伏時剛性低下率 αy。
///
/// RC 矩形断面の梁（水平材）は菅野式
/// （[`squid_n_core::rc_capacity::rc_alpha_y_sugano`]、梅村魁『鉄筋コンクリート
/// 建物の動的耐震設計法』P.106-108）で算定する:
/// - `pt` = at/(b·D)（at=main_x の半分を引張側と仮定）
/// - `a` = 可撓長さ/2（せん断スパン）、`a/D` は式側で [1,5] にクランプ
/// - `d` = 有効せい（D − かぶり − 主筋半径）
/// - `n` = Es/Ec（部材材料のヤング係数を Ec とみなす）
///
/// 柱（鉛直材）は菅野式に軸力項を要するため対象外（柱の既定はファイバー
/// モデルで、本バネ経路に乗る場合は従来既定 0.3）。鉄骨・SRC・CFT・情報不足も
/// 従来既定 0.3 を用いる。
fn flexural_alpha_y(data: &ElementData, model: &Model) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    const DEFAULT_ALPHA_Y: f64 = 0.3;
    if is_vertical_member(data, model) {
        return DEFAULT_ALPHA_Y;
    }
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let Some(SectionShape::RcRect { b, d, rebar }) = sec.and_then(|s| s.shape.as_ref()) else {
        return DEFAULT_ALPHA_Y;
    };
    if *b <= 0.0 || *d <= 0.0 {
        return DEFAULT_ALPHA_Y;
    }
    let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
    let pt = at / (b * d);
    let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
    let ec = data
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.young)
        .unwrap_or(0.0);
    let n = if ec > 0.0 {
        squid_n_core::section_shape::E_STEEL / ec
    } else {
        15.0
    };
    let a = flexible_length(data, model) / 2.0;
    let ay = squid_n_core::rc_capacity::rc_alpha_y_sugano(pt, a / d, d_eff / d, n);
    if ay.is_finite() && ay > 1e-6 {
        ay.min(1.0)
    } else {
        DEFAULT_ALPHA_Y
    }
}

/// 材端曲げバネの復元力材料を履歴則に応じて構築する（各履歴則の原典）。
/// 戻り値の bool は N-M 相関（`set_yield`）を適用可能か（バイリニアのみ true）。
/// 標準型・降伏モーメント不定は従来の kinematic バイリニアを用い、武田型/逆行型/
/// 原点指向型/最大点指向型は [`HysteresisMaterial`] のトリリニア（原点指向はバイ
/// リニア）を用いる。
fn build_flexural_springs(
    data: &ElementData,
    model: &Model,
    rule: HysteresisModel,
) -> (Box<dyn UniaxialMaterial>, Box<dyn UniaxialMaterial>, bool) {
    let (k_rot, my) = rotational_spring_params(data, model);
    // 標準型・降伏モーメント不定は従来の kinematic バイリニア（＝標準型相当）。
    if my <= 0.0 || k_rot <= 0.0 || rule == HysteresisModel::Standard {
        let my = my.max(1.0);
        return (
            Box::new(Bilinear::new(k_rot, my, 0.01)),
            Box::new(Bilinear::new(k_rot, my, 0.01)),
            true,
        );
    }
    // 辻・山田型（バイリニア＋β 混合硬化）。K2=0.01·k_rot、β=0.5（既定）。
    // set_yield 対応のため N-M 相関を適用可能。
    if rule == HysteresisModel::TsujiYamada {
        let k2 = 0.01 * k_rot;
        let mk = || Box::new(TsujiYamada::new(k_rot, my, k2, 0.5)) as Box<dyn UniaxialMaterial>;
        return (mk(), mk(), true);
    }
    // 座屈考慮型（耐力劣化型＋RO 除荷）。既定 Mu=1.1·My（座屈細長比の精算は今後の課題。
    // 断面の λb・κ・WF が得られる場合は lateral_buckling_mu_ratio で Mu/Mp を算定可）。
    // set_yield 対応（Mu も比率を保持）のため N-M 相関を適用可能。
    if rule == HysteresisModel::SteelBuckling {
        let mk =
            || Box::new(SteelBuckling::with_defaults(k_rot, my, 1.1)) as Box<dyn UniaxialMaterial>;
        return (mk(), mk(), true);
    }
    // トリリニア折れ点: ひび割れ Mc/θc（初期勾配 k_rot）、降伏 My/θy（降伏時剛性
    // 低下率 αy。RC 矩形梁は菅野式、その他は既定 0.3 = [`flexural_alpha_y`]）、
    // 終局 Mu=1.1·My/θu（塑性率 4）。
    let mc = crack_moment(data, model, my);
    let tc = (mc / k_rot).max(1e-9);
    let alpha_y = flexural_alpha_y(data, model);
    let ty = (my / (alpha_y * k_rot)).max(tc * 1.5);
    let mu = 1.1 * my;
    let tu = ty * 4.0;
    let alpha = 0.4;
    let mk =
        |r: HysteresisRule| -> Box<dyn UniaxialMaterial> { Box::new(HysteresisMaterial::new(r)) };
    let make_pair = |r: HysteresisRule| (mk(r.clone()), mk(r));
    let (a, b) = match rule {
        HysteresisModel::Retrograde => make_pair(HysteresisRule::Retrograde {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        HysteresisModel::OriginOriented => make_pair(HysteresisRule::OriginOriented {
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        HysteresisModel::MaxPointOriented => make_pair(HysteresisRule::MaxPointOriented {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        // Takeda（RC 既定）とその他は武田型トリリニア。
        _ => make_pair(HysteresisRule::Takeda {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
            alpha,
        }),
    };
    (a, b, false)
}

#[cfg(test)]
mod tests;
