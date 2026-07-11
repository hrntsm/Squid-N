//! S 造柱の座屈長さ係数 K（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」鉄骨造柱の断面検定、根拠は鋼構造塑性設計指針
//! (6.65)〜(6.67) 式、水平移動が拘束されない場合）。
//!
//! ```text
//! (GA・GB・(π/K)² − 36) / (6・(GA + GB)) = (π/K) / tan(π/K)
//! G = Σ(Ic/lc) / Σ(Ig/lg)
//! ```
//!
//! マニュアルの規定（本実装で対応するもの）:
//! - 柱端がピン接合の場合は G=10。
//! - 節点に接する梁が無い場合は G=10。
//! - 混合構造（RC/SRC 部材が節点に接する場合）はその部材の剛性をヤング係数比
//!   により補正する → 本実装は `Σ(E・I/L)` の比で G を計算するため、各部材の
//!   実ヤング係数がそのまま補正として効く。
//! - 節点に接する部材の角度は考慮しない（マニュアルと同じ簡略化）。
//! - 梁の結合状態・支点の状態は考慮しない（同上）。
//!
//! # 本実装の追加的な簡略化
//! - 断面二次モーメントは強軸 `Section.iy` を全部材で用いる（加力方向別の
//!   使い分けはしない。マニュアル自身が部材角度を考慮しないため同水準の近似）。
//! - `EndCondition::SemiRigid` はピンとみなさず G の計算値をそのまま用いる。
//! - 剛域・特殊形状による材長補正は行わず、節点間の幾何学的長さを用いる。

use squid_n_core::model::{ElementData, ElementKind, EndCondition, Model};

/// ピン端・梁無し節点に用いる剛度比 G の規定値（マニュアル既定）。
const G_PIN: f64 = 10.0;

/// 水平移動が拘束されない場合（sway 骨組）の座屈長さ係数 K を、
/// 鋼構造塑性設計指針 (6.65) 式
/// `(GA・GB・x² − 36)/(6(GA+GB)) = x/tan(x)`（`x = π/K`）から数値的に解く。
///
/// - `ga`, `gb`: 柱両端の剛度比 G（負値は 0 に丸める）。
/// - 戻り値は K ≥ 1.0（sway 骨組の理論下限。GA=GB=0 の完全固定端で K=1）。
///
/// 左辺は x について単調増加、右辺 `x/tan(x)` は (0, π) で単調減少であり、
/// `f(x) = 左辺 − 右辺` は単調増加かつ `f(0+) = −6/(GA+GB) − 1 < 0`、
/// `f(π−) → +∞` なので (0, π) に唯一の根を持つ。二分法で求める。
pub fn sway_buckling_k(ga: f64, gb: f64) -> f64 {
    let ga = ga.max(0.0);
    let gb = gb.max(0.0);
    let sum = ga + gb;
    if sum <= 1e-12 {
        // 両端とも G=0（梁が無限剛）: K=1。
        return 1.0;
    }
    let f = |x: f64| (ga * gb * x * x - 36.0) / (6.0 * sum) - x / x.tan();

    let mut lo = 1e-9_f64;
    let mut hi = std::f64::consts::PI - 1e-9;
    // 数値端点の符号を確認（理論上 f(lo)<0, f(hi)>0）。万一崩れていたら K=1 に退避。
    if !(f(lo) < 0.0 && f(hi) > 0.0) {
        return 1.0;
    }
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let x = 0.5 * (lo + hi);
    (std::f64::consts::PI / x).max(1.0)
}

/// 線材（`ElementKind::Beam`）の幾何学的長さと軸方向余弦の鉛直成分 |ez|。
fn line_geometry(model: &Model, elem: &ElementData) -> Option<(f64, f64)> {
    let p0 = model.nodes.get(elem.nodes.first()?.index())?.coord;
    let p1 = model.nodes.get(elem.nodes.get(1)?.index())?.coord;
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return None;
    }
    Some((len, (dz / len).abs()))
}

/// 部材の曲げ剛度 E・I/L（強軸 `iy`）。断面・材料・長さが解決できない場合 None。
fn flexural_stiffness(model: &Model, elem: &ElementData, length: f64) -> Option<f64> {
    let sec = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let mat = elem
        .material
        .and_then(|mid| model.materials.get(mid.index()))?;
    if sec.iy <= 0.0 || mat.young <= 0.0 || length <= 0.0 {
        return None;
    }
    Some(mat.young * sec.iy / length)
}

/// 節点 `node_idx`（`elem.nodes` の 0/1）まわりの剛度比 G を求める。
///
/// `G = Σ(E・I/L)_柱 / Σ(E・I/L)_梁`。部材種別は部材軸の鉛直成分による
/// 幾何判定（|ez| ≥ 0.8 柱、|ez| ≤ 0.2 梁。それ以外＝斜材は無視）で、
/// `member_kind_of`（app/mcp）と同じ規則。
///
/// - 当該柱端の `EndCondition` が `Pinned` の場合は G=10（マニュアル既定）。
/// - 節点に接する梁が無い場合（Σ梁 = 0）は G=10（同上）。
fn g_ratio_at(model: &Model, column: &ElementData, node_idx: usize) -> f64 {
    if matches!(column.end_cond.get(node_idx), Some(EndCondition::Pinned)) {
        return G_PIN;
    }
    let Some(node_id) = column.nodes.get(node_idx) else {
        return G_PIN;
    };

    let mut sum_col = 0.0_f64;
    let mut sum_beam = 0.0_f64;
    for other in &model.elements {
        if other.kind != ElementKind::Beam {
            continue;
        }
        if !other.nodes.iter().take(2).any(|n| n == node_id) {
            continue;
        }
        let Some((len, ez)) = line_geometry(model, other) else {
            continue;
        };
        let Some(ei_l) = flexural_stiffness(model, other, len) else {
            continue;
        };
        if ez >= 0.8 {
            sum_col += ei_l;
        } else if ez <= 0.2 {
            sum_beam += ei_l;
        }
    }

    if sum_beam <= 1e-12 {
        return G_PIN;
    }
    sum_col / sum_beam
}

/// 柱 `elem` の座屈長さ係数 K（水平移動が拘束されない場合）を、モデルの
/// 節点まわり剛度比から算定する。
///
/// 柱でない（幾何判定で |ez| < 0.8）、または線材でない場合は None。
/// 呼び出し側は `lk = K・L` を [`crate::DesignCtx::lk`] に渡すことを想定する。
pub fn steel_column_k(model: &Model, elem: &ElementData) -> Option<f64> {
    if elem.kind != ElementKind::Beam {
        return None;
    }
    let (_, ez) = line_geometry(model, elem)?;
    if ez < 0.8 {
        return None;
    }
    let ga = g_ratio_at(model, elem, 0);
    let gb = g_ratio_at(model, elem, 1);
    Some(sway_buckling_k(ga, gb))
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        RigidZone, Section,
    };

    // ------------------------------------------------------------------
    // sway_buckling_k（純関数）: 鋼構造塑性設計指針のアラインメントチャート
    // （水平移動非拘束）の代表値と照合する。
    // ------------------------------------------------------------------

    /// 求めた K が (6.65) 式を満たすことを直接確認するヘルパ。
    fn residual(ga: f64, gb: f64, k: f64) -> f64 {
        let x = std::f64::consts::PI / k;
        (ga * gb * x * x - 36.0) / (6.0 * (ga + gb)) - x / x.tan()
    }

    #[test]
    fn sway_k_fixed_ends_is_one() {
        assert_eq!(sway_buckling_k(0.0, 0.0), 1.0);
        assert_eq!(sway_buckling_k(-1.0, 0.0), 1.0);
    }

    #[test]
    fn sway_k_symmetric_g1() {
        // G_A=G_B=1 → K ≈ 1.31〜1.32（チャート代表値）。
        let k = sway_buckling_k(1.0, 1.0);
        assert!((1.28..=1.35).contains(&k), "k={k}");
        assert!(residual(1.0, 1.0, k).abs() < 1e-6);
    }

    #[test]
    fn sway_k_symmetric_g10() {
        // G_A=G_B=10 → K ≈ 3.0（チャート代表値）。
        let k = sway_buckling_k(10.0, 10.0);
        assert!((2.9..=3.1).contains(&k), "k={k}");
        assert!(residual(10.0, 10.0, k).abs() < 1e-6);
    }

    #[test]
    fn sway_k_asymmetric_g0_g10() {
        // G_A=0（固定）・G_B=10（ほぼピン）→ K ≈ 1.65〜1.75。
        let k = sway_buckling_k(0.0, 10.0);
        assert!((1.6..=1.8).contains(&k), "k={k}");
        assert!(residual(1e-12, 10.0, k).abs() < 1e-3);
    }

    #[test]
    fn sway_k_monotone_and_symmetric() {
        let k11 = sway_buckling_k(1.0, 1.0);
        let k22 = sway_buckling_k(2.0, 2.0);
        assert!(k22 > k11);
        let k15 = sway_buckling_k(1.0, 5.0);
        let k51 = sway_buckling_k(5.0, 1.0);
        assert!((k15 - k51).abs() < 1e-9);
        // sway 骨組では常に K ≥ 1。
        assert!(k11 >= 1.0 && k15 >= 1.0);
    }

    // ------------------------------------------------------------------
    // steel_column_k（モデル配線）
    // ------------------------------------------------------------------

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }
    }

    fn line_elem(id: u32, n0: u32, n1: u32) -> ElementData {
        ElementData {
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
        }
    }

    fn section(iy: f64) -> Section {
        Section {
            id: SectionId(0),
            name: "H-400x200x8x13".to_string(),
            area: 8_000.0,
            iy,
            iz: iy / 10.0,
            j: 1.0,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }
    }

    fn steel_material() -> Material {
        Material {
            id: MaterialId(0),
            name: "SN400B".to_string(),
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
            concrete_class: Default::default(),
        }
    }

    /// 柱1本（節点0-1）+ 上下端に同一断面・同一長さの梁が1本ずつ:
    /// G = (E·I/Lc)/(E·I/Lg) が両端で等しくなるモデル。
    fn portal_model(col_len: f64, beam_len: f64) -> Model {
        let nodes = vec![
            node(0, 0.0, 0.0, 0.0),
            node(1, 0.0, 0.0, col_len),
            node(2, beam_len, 0.0, 0.0),
            node(3, beam_len, 0.0, col_len),
        ];
        let elements = vec![
            line_elem(0, 0, 1), // 柱
            line_elem(1, 0, 2), // 下端の梁
            line_elem(2, 1, 3), // 上端の梁
        ];
        Model {
            nodes,
            elements,
            sections: vec![section(2.0e8)],
            materials: vec![steel_material()],
            ..Default::default()
        }
    }

    #[test]
    fn steel_column_k_matches_hand_g() {
        // 柱長 4000・梁長 8000、同一断面 → G = (I/4000)/(I/8000) = 2 が両端。
        let model = portal_model(4000.0, 8000.0);
        let k = steel_column_k(&model, &model.elements[0]).expect("柱として判定される");
        let expected = sway_buckling_k(2.0, 2.0);
        assert!((k - expected).abs() < 1e-9, "k={k}, expected={expected}");
        // G=2,2 のチャート代表値はおよそ 1.6。
        assert!((1.5..=1.7).contains(&k), "k={k}");
    }

    #[test]
    fn steel_column_k_no_beam_uses_g10() {
        // 梁を取り除くと両端 G=10（マニュアル既定）→ K ≈ 3.0。
        let mut model = portal_model(4000.0, 8000.0);
        model.elements.truncate(1);
        let k = steel_column_k(&model, &model.elements[0]).unwrap();
        let expected = sway_buckling_k(10.0, 10.0);
        assert!((k - expected).abs() < 1e-9);
    }

    #[test]
    fn steel_column_k_pinned_end_uses_g10() {
        let mut model = portal_model(4000.0, 8000.0);
        model.elements[0].end_cond[1] = EndCondition::Pinned;
        let k = steel_column_k(&model, &model.elements[0]).unwrap();
        let expected = sway_buckling_k(2.0, 10.0);
        assert!((k - expected).abs() < 1e-9);
    }

    #[test]
    fn steel_column_k_none_for_beam() {
        let model = portal_model(4000.0, 8000.0);
        // elements[1] は水平材（梁）なので None。
        assert!(steel_column_k(&model, &model.elements[1]).is_none());
    }

    #[test]
    fn steel_column_k_rc_beam_young_ratio_correction() {
        // 梁を RC（E=1/10）にすると Σ(EI/L)_梁 が 1/10 になり G が 10 倍 → K 増大。
        let mut model = portal_model(4000.0, 8000.0);
        let mut rc = steel_material();
        rc.id = MaterialId(1);
        rc.name = "Fc24".to_string();
        rc.young = 20_500.0;
        model.materials.push(rc);
        for e in &mut model.elements[1..] {
            e.material = Some(MaterialId(1));
        }
        let k_rc = steel_column_k(&model, &model.elements[0]).unwrap();
        let steel_model = portal_model(4000.0, 8000.0);
        let k_steel = steel_column_k(&steel_model, &steel_model.elements[0]).unwrap();
        assert!(
            k_rc > k_steel,
            "RC梁で G が大きくなり K も大きくなる: {k_rc} <= {k_steel}"
        );
        let expected = sway_buckling_k(20.0, 20.0);
        assert!((k_rc - expected).abs() < 1e-9);
    }
}
