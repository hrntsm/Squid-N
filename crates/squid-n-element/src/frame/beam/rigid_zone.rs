//! 剛域（rigid zone）の自動算定。
//!
//! モデルのトポロジ（部材種別・接続断面）から各部材端の剛域長を算定し、
//! `ElementData::rigid_zone` へ反映する前処理を提供する。剛性・内力を計算する
//! [`BeamElement`](super::BeamElement) とは独立しており、解析前に一度だけ適用する。

use squid_n_core::model::{Model, RigidZone, ZoneSource};

pub struct RigidZoneRule {
    pub reduction: f64,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self { reduction: 1.0 }
    }
}

/// 部材の構造種別（技術基準解説書「剛域の計算」の RC/SRC 系・S 系区分）。
/// 剛域長の算定式（後述 `auto_rigid_zones`）を部材種別で切り替えるための分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MemberKind {
    /// RC・SRC 系（RC 造柱・梁・耐震壁、SRC 造柱・梁）。
    RcSrc,
    /// S・CFT 系（ＣＦＴはＳ造と同様に扱う）。
    Steel,
}

/// 要素の構造種別を判定する。
///
/// `Section.shape` があれば形状で判定する（RC/SRC 形状 → RcSrc、鋼材・CFT 形状 → Steel）。
/// `shape` が無い（カタログ数値直入力等）場合は材料で判定する: `Material.fc`（コンクリート
/// 設計基準強度）があれば RcSrc、`fy`（降伏応力）のみあれば Steel。どちらも無い場合は
/// 判定材料が無いため RcSrc 扱い（剛域式を変えない＝従来挙動を維持する既定）。
fn member_kind(model: &Model, e: &squid_n_core::model::ElementData) -> MemberKind {
    use squid_n_core::section_shape::SectionShape;

    let sec = e.section.and_then(|sid| model.sections.get(sid.index()));
    if let Some(shape) = sec.and_then(|s| s.shape.as_ref()) {
        return match shape {
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. }
            | SectionShape::SrcRect { .. } => MemberKind::RcSrc,
            SectionShape::SteelH { .. }
            | SectionShape::SteelBox { .. }
            | SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
            | SectionShape::SteelPipe { .. }
            | SectionShape::CftBox { .. }
            | SectionShape::CftPipe { .. } => MemberKind::Steel,
        };
    }

    let mat = e.material.and_then(|mid| model.materials.get(mid.index()));
    if let Some(mat) = mat {
        if mat.fc.is_some() {
            return MemberKind::RcSrc;
        }
        if mat.fy.is_some() {
            return MemberKind::Steel;
        }
    }
    MemberKind::RcSrc
}

pub fn auto_rigid_zones(
    model: &squid_n_core::model::Model,
    elem_id: squid_n_core::ids::ElemId,
    rule: &RigidZoneRule,
) -> RigidZone {
    let elem = match model.elements.iter().find(|e| e.id == elem_id) {
        Some(e) => e,
        None => {
            return RigidZone {
                reduction: rule.reduction,
                ..Default::default()
            }
        }
    };

    let nodes = &elem.nodes;
    if nodes.len() < 2 {
        return RigidZone {
            reduction: rule.reduction,
            ..Default::default()
        };
    }

    let self_sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    let d_self = self_sec.map(|s| s.depth).unwrap_or(0.0);

    // 節点 → 接続要素のマップ（直交せい探索の対象は柱・梁＝Beam 要素のみ。
    // 耐震壁・シェル等が混入すると「耐震壁周辺の柱・梁の剛域は考慮しません」
    // という方針に反し、壁の名目せい等が誤って直交材に紛れ込む）。
    let mut node_to_elems: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for (ei, e) in model.elements.iter().enumerate() {
        if e.nodes.len() >= 2 && matches!(e.kind, squid_n_core::model::ElementKind::Beam) {
            for n in &e.nodes {
                node_to_elems.entry(n.index()).or_default().push(ei);
            }
        }
    }

    fn elem_axis(model: &Model, e: &squid_n_core::model::ElementData) -> [f64; 3] {
        if e.nodes.len() < 2 {
            return [0.0, 0.0, 0.0];
        }
        let p0 = model.nodes[e.nodes[0].index()].coord;
        let p1 = model.nodes[e.nodes[1].index()].coord;
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        if l < 1e-12 {
            [0.0, 0.0, 0.0]
        } else {
            [dx / l, dy / l, dz / l]
        }
    }

    // `only_rc_src` を true にすると、RC/SRC 系の直交 Beam 要素だけを対象に最大せいを探す
    // （剛域長 λ 用。仕口部に接続する柱(梁)がすべてＳの場合、剛域長さは0
    // ＝ S 系直交材は無視することで自然に d_max=0 となる）。false なら種別を問わず全直交
    // Beam 要素が対象（危険断面位置 face 用。§6.2.3 は幾何量であり種別を区別しない）。
    fn max_orth_depth(
        model: &Model,
        node_idx: usize,
        target_axis: [f64; 3],
        target_elem_idx: usize,
        node_to_elems: &std::collections::HashMap<usize, Vec<usize>>,
        only_rc_src: bool,
    ) -> f64 {
        let mut d_max = 0.0;
        if let Some(elems) = node_to_elems.get(&node_idx) {
            for &ei in elems {
                if ei == target_elem_idx {
                    continue;
                }
                let e = &model.elements[ei];
                if e.nodes.len() < 2 {
                    continue;
                }
                if only_rc_src && member_kind(model, e) != MemberKind::RcSrc {
                    continue;
                }
                let axis = elem_axis(model, e);
                let dot = (axis[0] * target_axis[0]
                    + axis[1] * target_axis[1]
                    + axis[2] * target_axis[2])
                    .abs();
                if dot < 0.707 {
                    // 概ね直交（45°以上）
                    if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                        if sec.depth > d_max {
                            d_max = sec.depth;
                        }
                    }
                }
            }
        }
        d_max
    }

    let target_axis = elem_axis(model, elem);
    let target_elem_idx = model
        .elements
        .iter()
        .position(|e| e.id == elem_id)
        .unwrap_or(0);

    // face 用: 種別を問わない直交 Beam 要素の最大せい（従来どおりの幾何量）。
    let d_orth_face_i = max_orth_depth(
        model,
        nodes[0].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        false,
    );
    let d_orth_face_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        false,
    );
    // λ 用: RC/SRC 系の直交 Beam 要素だけの最大せい。
    let d_orth_rc_i = max_orth_depth(
        model,
        nodes[0].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        true,
    );
    let d_orth_rc_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        true,
    );

    // 剛域長 λ は自部材の構造種別で式を切り替える（技術基準解説書「剛域の計算」）。
    // - RC/SRC 造: λ = reduction·(D_orth_rc/2 − D_self/4)（従来式。負は 0 クランプ）。
    // - Ｓ・ＣＦＴ造: λ = D_orth_rc/2（D_self/4 の控除なし・reduction も掛けない。
    //   RC/SRC 大梁のうち最大せいの梁フェイスまでの長さ＝仕口部を除いた長さ）。
    //   直交する RC/SRC 系の梁（柱）が無ければ D_orth_rc=0 なので λ=0
    //   （Ｓ造の剛域長さは0となる）。
    let self_kind = member_kind(model, elem);
    let lambda = |d_orth_rc: f64| -> f64 {
        match self_kind {
            MemberKind::RcSrc => {
                let v = rule.reduction * (d_orth_rc / 2.0 - d_self / 4.0);
                if v < 0.0 {
                    0.0
                } else {
                    v
                }
            }
            MemberKind::Steel => d_orth_rc / 2.0,
        }
    };
    // フェイス距離 = D_orth/2 は剛性用剛域の低減率（慣用調整）と無関係な幾何量なので
    // reduction を掛けない（設計書 §6.2.1「設計位置との区別」）。
    // λ が負→0 にクランプされる場合でも face はそのまま D_orth/2 を保持する。
    let face = |d_orth: f64| -> f64 { d_orth / 2.0 };

    RigidZone {
        length_i: lambda(d_orth_rc_i),
        length_j: lambda(d_orth_rc_j),
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        reduction: rule.reduction,
        face_i: face(d_orth_face_i),
        face_j: face(d_orth_face_j),
    }
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
    }
    // フェイス距離は剛域長の Manual/Auto フラグとは独立な幾何量（接続関係から
    // 一意に決まる §6.2.1）。手動で剛域長を保護しているときも、モデルの接続情報
    // が変われば危険断面位置は追従すべきなので、Manual 保護の対象外として常に
    // 再算定値で更新する。
    zone.face_i = recomputed.face_i;
    zone.face_j = recomputed.face_j;
}

/// モデル全要素の剛域を自動算定し、`ElementData::rigid_zone` を更新する前処理。
/// `source` が `Auto` の端のみ更新し、`Manual` 端は保護する（設計書 §6.2.1）。
/// 解析前に1回呼ぶことで剛域が組立に反映される（既定では剛域長 0 のまま
/// ＝呼ばなければ従来挙動。明示的に有効化する設計）。
///
/// `auto_rigid_zones` を要素ごとに呼ぶと隣接マップ構築が O(E²) になるため、
/// ここでは梁要素の集合に対し各端の剛域を算定して一括反映する。
pub fn apply_auto_rigid_zones(model: &mut Model, rule: &RigidZoneRule) {
    // 要素 id ごとに算定（auto_rigid_zones は内部で隣接を構築するが、
    // 呼び出しは「解析前1回」を想定。大規模最適化は将来）。
    let recomputed: Vec<(usize, RigidZone)> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e.kind, squid_n_core::model::ElementKind::Beam))
        .map(|(i, e)| (i, auto_rigid_zones(model, e.id, rule)))
        .collect();

    for (i, rz) in recomputed {
        let zone = &mut model.elements[i].rigid_zone;
        recompute_auto_zones(zone, &rz);
        // reduction も Auto 算定値を反映（手動端の length は保持済み）。
        zone.reduction = rz.reduction;
    }
}
