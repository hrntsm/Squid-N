//! S 造柱の座屈長さ係数 K（鉄骨造柱の許容応力度検定、
//! 根拠は鋼構造塑性設計指針
//! (6.65)〜(6.67) 式、水平移動が拘束されない場合）。
//!
//! ```text
//! (GA・GB・(π/K)² − 36) / (6・(GA + GB)) = (π/K) / tan(π/K)
//! G = Σ(Ic/lc) / Σ(Ig/lg)
//! ```
//!
//! 座屈長さ算定で本実装が対応する規定:
//! - 柱端がピン接合の場合は G=10。
//! - 節点に接する梁が無い場合は G=10。
//! - 混合構造（RC/SRC 部材が節点に接する場合）はその部材の剛性をヤング係数比
//!   により補正する → 本実装は `Σ(E・I/L)` の比で G を計算するため、各部材の
//!   実ヤング係数がそのまま補正として効く。
//! - 梁の結合状態・支点の状態は、軸別評価版（下記）のみ節点側の梁端の
//!   ピン接合を考慮する。それ以外（方向無差別簡略版・支点の状態・梁の
//!   遠端の結合状態）は考慮しない（本実装の簡略化）。
//!
//! # 軸別評価（[`steel_column_k_axes_with_index`]）
//! 強軸まわり K_y・弱軸まわり K_z を個別に評価する版は、節点に接する部材の
//! 角度を [`squid_n_element::transform::LocalFrame`] の局所軸から求め、次の
//! 重み付けで G を軸ごとに集計する:
//! - 梁: 梁材軸の水平投影と評価方向（たわみ方向）のなす角の余弦の 2 乗
//!   `cos²θ` を `E・iy/L'` に乗じる（面内（鉛直面）曲げは強軸 `iy` とする
//!   仮定は維持し、角度のみ重み付けする）。当該節点側の梁端が
//!   `EndCondition::Pinned` の場合、その梁は節点回転を拘束しないため
//!   Σ梁 に算入しない（`SemiRigid` は従来通り剛接合とみなす。梁の遠端の
//!   結合状態は考慮しない）。
//! - 柱（対象柱自身を含む）: その柱自身の強軸たわみ方向と評価方向のなす角
//!   `cos²β` により `I_eff = iy・cos²β + iz・(1−cos²β)` へ断面二次モーメントを
//!   投影する。
//! - `L'` は柱・梁とも剛域控除後の内法長 `L' = 節点間長さ − rigid_zone.length_i
//!   − rigid_zone.length_j`（[`clear_length`]。`L' ≤ 0` になる場合は節点間の
//!   幾何学的長さにフォールバックする）。
//!
//! 評価方向の水平投影が縮退する（部材がほぼ鉛直で水平方向が定まらない）軸は、
//! 方向を区別しない従来の集計（[`g_ratio_at_with_index`]、下記の方向無差別
//! 簡略版と同じ扱い）にフォールバックする。
//!
//! # 方向無差別簡略版（[`steel_column_k_with_index`]・[`steel_column_k`]）の簡略化
//! - 節点に接する部材の角度は考慮しない（互換用の簡略版。軸別精緻化は
//!   [`steel_column_k_axes_with_index`] を使うこと）。
//! - 断面二次モーメントは強軸 `Section.iy` を全部材で用いる（加力方向別の
//!   使い分けはしない。部材角度を考慮しないため同水準の近似）。
//! - 梁の結合状態は考慮しない（ピン接合の梁も剛接合として算入する）。
//! - 剛域による材長補正は行わず、節点間の幾何学的長さをそのまま用いる
//!   （互換維持のため軸別評価版のみ精緻化する）。
//!
//! # 両版に共通する簡略化
//! - `EndCondition::SemiRigid` はピンとみなさず G の計算値をそのまま用いる。
//! - 支点（節点の境界条件）の状態は考慮しない。
//! - 斜材（水平・鉛直いずれでもない部材）は無視する。

use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, Material, Model, Section};
use squid_n_element::transform::LocalFrame;
use std::collections::HashMap;

/// ピン端・梁無し節点に用いる剛度比 G の規定値（本実装の既定値）。
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

/// 線材（両端 2 節点）の始端・終端座標。
fn node_coords(model: &Model, elem: &ElementData) -> Option<([f64; 3], [f64; 3])> {
    let p0 = model.nodes.get(elem.nodes.first()?.index())?.coord;
    let p1 = model.nodes.get(elem.nodes.get(1)?.index())?.coord;
    Some((p0, p1))
}

/// 線材（`ElementKind::Beam`）の幾何学的長さと軸方向余弦の鉛直成分 |ez|。
fn line_geometry(model: &Model, elem: &ElementData) -> Option<(f64, f64)> {
    let (p0, p1) = node_coords(model, elem)?;
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return None;
    }
    Some((len, (dz / len).abs()))
}

/// 部材の断面・材料 `(Section, Material)`。いずれか解決できない場合 None。
fn section_material<'m>(
    model: &'m Model,
    elem: &ElementData,
) -> Option<(&'m Section, &'m Material)> {
    let sec = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let mat = elem
        .material
        .and_then(|mid| model.materials.get(mid.index()))?;
    Some((sec, mat))
}

/// 部材の曲げ剛度 E・I/L（強軸 `iy`）。断面・材料・長さが解決できない場合 None。
fn flexural_stiffness(model: &Model, elem: &ElementData, length: f64) -> Option<f64> {
    let (sec, mat) = section_material(model, elem)?;
    if sec.iy <= 0.0 || mat.young <= 0.0 || length <= 0.0 {
        return None;
    }
    Some(mat.young * sec.iy / length)
}

/// 水平面（xy 平面）への正射影を単位ベクトルへ正規化する。投影長が `1e-6`
/// 未満（鉛直に近く水平方向を定義できない）場合は `None`。
fn horizontal_unit(v: [f64; 3]) -> Option<[f64; 2]> {
    let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if len < 1e-6 {
        None
    } else {
        Some([v[0] / len, v[1] / len])
    }
}

/// 部材 `elem` の局所軸フレームにおける `rot[axis]`（0=ex 材軸, 1=ey 強軸たわみ
/// 方向, 2=ez 弱軸たわみ方向）の水平投影単位ベクトル。幾何が解決できない、
/// または投影が縮退する場合は `None`。
fn member_horizontal_axis(model: &Model, elem: &ElementData, axis: usize) -> Option<[f64; 2]> {
    let (p0, p1) = node_coords(model, elem)?;
    let frame = LocalFrame::from_nodes(p0, p1, elem.local_axis.ref_vector);
    horizontal_unit(frame.rot[axis])
}

/// 部材 `elem` の `node_id` 側の端番号（`elem.nodes` の 0/1）。
/// 見つからない場合は `None`（`node_id` が `elem` の端点でない）。
fn end_index_at(elem: &ElementData, node_id: NodeId) -> Option<usize> {
    elem.nodes.iter().take(2).position(|n| *n == node_id)
}

/// 剛域控除後の内法長 `L' = len − rigid_zone.length_i − rigid_zone.length_j`
/// （squid_n_element の可とう長と同じ式）。`L' ≤ 0` になる場合は
/// 幾何学的長さ `len` にフォールバックする。
fn clear_length(elem: &ElementData, len: f64) -> f64 {
    let l = len - elem.rigid_zone.length_i - elem.rigid_zone.length_j;
    if l > 0.0 {
        l
    } else {
        len
    }
}

/// 節点ID → その節点に接続する線材（`ElementKind::Beam`）要素への参照インデックス。
///
/// `g_ratio_at_with_index`（延いては `steel_column_k_with_index`）が節点まわりの
/// 部材を求める際に、モデル全要素を毎回線形走査するのを避けるために使う。
/// 判定ロジック自体（どの要素を G の柱側/梁側に数えるか）は一切変更しない
/// （ここで絞り込むのは従来の `other.kind == ElementKind::Beam &&
/// other.nodes[..2].contains(node_id)` と同じ集合）。
pub struct BeamNodeIndex<'m> {
    by_node: HashMap<NodeId, Vec<&'m ElementData>>,
}

impl<'m> BeamNodeIndex<'m> {
    /// モデル全体から一度だけ構築する。複数回の `steel_column_k_with_index`
    /// 呼び出し（部材数ぶん）で使い回すことを想定する。
    pub fn build(model: &'m Model) -> Self {
        let mut by_node: HashMap<NodeId, Vec<&'m ElementData>> = HashMap::new();
        for elem in &model.elements {
            if elem.kind != ElementKind::Beam {
                continue;
            }
            for node_id in elem.nodes.iter().take(2) {
                by_node.entry(*node_id).or_default().push(elem);
            }
        }
        Self { by_node }
    }

    fn beams_at(&self, node_id: NodeId) -> &[&'m ElementData] {
        self.by_node
            .get(&node_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }
}

/// 節点 `node_idx`（`elem.nodes` の 0/1）まわりの剛度比 G を求める（インデックス版）。
///
/// `G = Σ(E・I/L)_柱 / Σ(E・I/L)_梁`。部材種別は部材軸の鉛直成分による
/// 幾何判定（|ez| ≥ 0.8 柱、|ez| ≤ 0.2 梁。それ以外＝斜材は無視）で、
/// `member_kind_of`（app/mcp）と同じ規則。
///
/// - 当該柱端の `EndCondition` が `Pinned` の場合は G=10（本実装の既定値）。
/// - 節点に接する梁が無い場合（Σ梁 = 0）は G=10（同上）。
fn g_ratio_at_with_index(
    model: &Model,
    index: &BeamNodeIndex<'_>,
    column: &ElementData,
    node_idx: usize,
) -> f64 {
    if matches!(column.end_cond.get(node_idx), Some(EndCondition::Pinned)) {
        return G_PIN;
    }
    let Some(node_id) = column.nodes.get(node_idx) else {
        return G_PIN;
    };

    let mut sum_col = 0.0_f64;
    let mut sum_beam = 0.0_f64;
    for other in index.beams_at(*node_id) {
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

/// 柱 `elem` の座屈長さ係数 K（水平移動が拘束されない場合、方向を考慮しない
/// 簡略版・互換用）を、モデルの節点まわり剛度比から算定する（インデックス版）。
///
/// 柱でない（幾何判定で |ez| < 0.8）、または線材でない場合は None。
/// 呼び出し側は `lk = K・L` を [`crate::DesignCtx::lk_y`]/[`crate::DesignCtx::lk_z`]
/// （両軸同値）に渡すことを想定する。強軸・弱軸を個別に評価する場合は
/// [`steel_column_k_axes_with_index`] を使うこと。
///
/// `index` は事前に `BeamNodeIndex::build(model)` で一度だけ構築し、
/// 全部材ぶんのループで使い回すこと（全部材×全要素の O(n²) 線形探索を回避）。
pub fn steel_column_k_with_index(
    model: &Model,
    index: &BeamNodeIndex<'_>,
    elem: &ElementData,
) -> Option<f64> {
    if elem.kind != ElementKind::Beam {
        return None;
    }
    let (_, ez) = line_geometry(model, elem)?;
    if ez < 0.8 {
        return None;
    }
    let ga = g_ratio_at_with_index(model, index, elem, 0);
    let gb = g_ratio_at_with_index(model, index, elem, 1);
    Some(sway_buckling_k(ga, gb))
}

/// 柱 `elem` の座屈長さ係数 K（水平移動が拘束されない場合、方向を考慮しない
/// 簡略版・互換用）を、モデルの節点まわり剛度比から算定する。
///
/// 柱でない（幾何判定で |ez| < 0.8）、または線材でない場合は None。
/// 呼び出し側は `lk = K・L` を [`crate::DesignCtx::lk_y`]/[`crate::DesignCtx::lk_z`]
/// （両軸同値）に渡すことを想定する。
///
/// 単発呼び出し用の互換 API（内部で `BeamNodeIndex` を都度構築する）。
/// 部材数ぶんループで呼ぶ場合は `BeamNodeIndex::build` を一度だけ構築して
/// [`steel_column_k_with_index`] を使うこと（O(n²) を避けられる）。
pub fn steel_column_k(model: &Model, elem: &ElementData) -> Option<f64> {
    let index = BeamNodeIndex::build(model);
    steel_column_k_with_index(model, &index, elem)
}

// ---------------------------------------------------------------------
// 軸別評価（強軸 K_y・弱軸 K_z を個別に算定する精緻化版）
// ---------------------------------------------------------------------

/// 節点 `node_idx` まわりの軸別剛度比 G_a を求める（評価方向 `d_a` に対する
/// 重み付け版）。
///
/// `d_a` は評価対象のたわみ方向（対象柱自身の `ey`（強軸）または `ez`（弱軸）の
/// 水平投影単位ベクトル）。`G_a = Σ(E・I_eff/L')_柱 / Σ(E・iy・cos²θ/L')_梁`
/// （`L'` は剛域控除後の内法長。[`clear_length`]）:
///
/// - 柱（対象柱自身を含む。|ez| ≥ 0.8）: その柱自身の強軸たわみ方向の水平投影
///   `d_c` を求め、`cos²β = (d_c・d_a)²` により `I_eff = iy・cos²β + iz・(1−cos²β)`
///   を負担剛度とする。`d_c` が縮退（求まらない）場合は `cos²β=1`（`iy` を採用）。
/// - 梁（|ez| ≤ 0.2）: 梁材軸の水平投影単位ベクトル `e_h` と `d_a` のなす角の
///   余弦の 2 乗 `cos²θ = (e_h・d_a)²` を重みとして `E・iy/L'` に乗じる（面内
///   （鉛直面）曲げは強軸 `iy` とする従来仮定を維持）。`e_h` が縮退する場合は
///   寄与 0（水平方向を定義できない部材は回転拘束に寄与しないとみなす）。
///   当該節点側の端が `Pinned` の梁は、その梁端が節点回転を拘束しないため
///   Σ梁 に算入しない（`SemiRigid` は従来通り剛接合とみなす）。
/// - 斜材（0.2 < |ez| < 0.8）は従来通り無視する。
///
/// 当該柱端が `Pinned`、または節点に接する梁が無い（Σ梁 ≤ 0）場合は G=10。
fn g_ratio_axis_at(
    model: &Model,
    index: &BeamNodeIndex<'_>,
    column: &ElementData,
    node_idx: usize,
    d_a: [f64; 2],
) -> f64 {
    if matches!(column.end_cond.get(node_idx), Some(EndCondition::Pinned)) {
        return G_PIN;
    }
    let Some(node_id) = column.nodes.get(node_idx) else {
        return G_PIN;
    };

    let mut sum_col = 0.0_f64;
    let mut sum_beam = 0.0_f64;
    for other in index.beams_at(*node_id) {
        let Some((raw_len, ez)) = line_geometry(model, other) else {
            continue;
        };
        let len = clear_length(other, raw_len);
        if ez >= 0.8 {
            let Some((sec, mat)) = section_material(model, other) else {
                continue;
            };
            if mat.young <= 0.0 || len <= 0.0 {
                continue;
            }
            let (iy, iz) = (sec.iy.max(0.0), sec.iz.max(0.0));
            let cos2 = match member_horizontal_axis(model, other, 1) {
                Some(d_c) => {
                    let dot = d_c[0] * d_a[0] + d_c[1] * d_a[1];
                    dot * dot
                }
                None => 1.0,
            };
            let i_eff = iy * cos2 + iz * (1.0 - cos2);
            sum_col += mat.young * i_eff / len;
        } else if ez <= 0.2 {
            // ピン接合の梁端（当該節点側）は節点回転を拘束しないため不算入。
            if let Some(end_idx) = end_index_at(other, *node_id) {
                if matches!(other.end_cond.get(end_idx), Some(EndCondition::Pinned)) {
                    continue;
                }
            }
            let Some(ei_l) = flexural_stiffness(model, other, len) else {
                continue;
            };
            let cos2 = match member_horizontal_axis(model, other, 0) {
                Some(e_h) => {
                    let dot = e_h[0] * d_a[0] + e_h[1] * d_a[1];
                    dot * dot
                }
                None => 0.0,
            };
            sum_beam += ei_l * cos2;
        }
    }

    if sum_beam <= 1e-12 {
        return G_PIN;
    }
    sum_col / sum_beam
}

/// 柱 `elem` の軸別座屈長さ係数（強軸まわり K_y・弱軸まわり K_z）を、モデルの
/// 節点まわり剛度比から算定する（水平移動が拘束されない場合）。
///
/// 対象柱の局所軸（[`LocalFrame::from_nodes`]、`rot=[ex,ey,ez]`）の `ey`
/// （強軸曲げのたわみ方向）・`ez`（弱軸曲げのたわみ方向）それぞれについて
/// [`g_ratio_axis_at`] で軸別の G を求め、`sway_buckling_k` に渡す
/// （モジュール doc の「軸別評価」節参照）。評価方向の水平投影が縮退する軸は、
/// 方向を区別しない従来の集計（[`g_ratio_at_with_index`]、
/// [`steel_column_k_with_index`] と同じ値）にフォールバックする。
///
/// 柱でない（幾何判定で |ez| < 0.8）、または線材でない場合は None。
/// 呼び出し側は `lk_y = K_y・L`・`lk_z = K_z・L` を
/// [`crate::DesignCtx::lk_y`]/[`crate::DesignCtx::lk_z`] に渡すことを想定する。
///
/// `index` は事前に `BeamNodeIndex::build(model)` で一度だけ構築し、
/// 全部材ぶんのループで使い回すこと。
pub fn steel_column_k_axes_with_index(
    model: &Model,
    index: &BeamNodeIndex<'_>,
    elem: &ElementData,
) -> Option<(f64, f64)> {
    if elem.kind != ElementKind::Beam {
        return None;
    }
    let (p0, p1) = node_coords(model, elem)?;
    let (_, ez_comp) = line_geometry(model, elem)?;
    if ez_comp < 0.8 {
        return None;
    }
    let frame = LocalFrame::from_nodes(p0, p1, elem.local_axis.ref_vector);
    let d_y = horizontal_unit(frame.rot[1]);
    let d_z = horizontal_unit(frame.rot[2]);

    let k_for = |d: Option<[f64; 2]>| -> f64 {
        let (ga, gb) = match d {
            Some(d_a) => (
                g_ratio_axis_at(model, index, elem, 0, d_a),
                g_ratio_axis_at(model, index, elem, 1, d_a),
            ),
            None => (
                g_ratio_at_with_index(model, index, elem, 0),
                g_ratio_at_with_index(model, index, elem, 1),
            ),
        };
        sway_buckling_k(ga, gb)
    };
    Some((k_for(d_y), k_for(d_z)))
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
        // 梁を取り除くと両端 G=10（本実装の既定値）→ K ≈ 3.0。
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

    // ------------------------------------------------------------------
    // steel_column_k_axes_with_index（軸別精緻化版）
    // ------------------------------------------------------------------

    /// 平面ポータルフレーム（X-Z 面、柱・梁とも `ref_vector=[0,0,1]`）:
    /// 柱は ref_vector が材軸と平行なため `LocalFrame` の縮退フォールバックで
    /// `ey≈X`（強軸たわみ方向）・`ez≈Y`（弱軸たわみ方向）となる。梁は X 方向に
    /// しか無いため、K_y は従来の G=2（`steel_column_k_matches_hand_g` と同じ）
    /// と一致し、K_z（Y 方向に梁が無い）は `sway_buckling_k(10,10)` になる。
    #[test]
    fn steel_column_k_axes_plane_portal_matches_hand_g() {
        let model = portal_model(4000.0, 8000.0);
        let index = BeamNodeIndex::build(&model);
        let (k_y, k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");
        let expected_ky = sway_buckling_k(2.0, 2.0);
        let expected_kz = sway_buckling_k(10.0, 10.0);
        assert!(
            (k_y - expected_ky).abs() < 1e-9,
            "k_y={k_y}, expected={expected_ky}"
        );
        assert!(
            (k_z - expected_kz).abs() < 1e-9,
            "k_z={k_z}, expected={expected_kz}"
        );
    }

    /// 直交 2 方向（X・Y）に梁がある立体モデル: 柱は鉛直（`ey≈X`・`ez≈Y`）。
    /// X 方向梁（長さ 6000）は K_y のみに、Y 方向梁（長さ 9000）は K_z のみに
    /// 効くことを、両端対称のモデルで手計算照合する
    /// （`G_y=Lx/Lc=6000/3000=2`、`G_z=(iz/Lc)/(iy/Ly)=(iz/iy)・(Ly/Lc)=0.3`）。
    #[test]
    fn steel_column_k_axes_orthogonal_beams_pick_matching_axis() {
        let nodes = vec![
            node(0, 0.0, 0.0, 0.0),       // 柱下端
            node(1, 0.0, 0.0, 3000.0),    // 柱上端
            node(2, 6000.0, 0.0, 0.0),    // X 方向梁（下端側）の遠端
            node(3, 0.0, 9000.0, 0.0),    // Y 方向梁（下端側）の遠端
            node(4, 6000.0, 0.0, 3000.0), // X 方向梁（上端側）の遠端
            node(5, 0.0, 9000.0, 3000.0), // Y 方向梁（上端側）の遠端
        ];
        let elements = vec![
            line_elem(0, 0, 1), // 柱
            line_elem(1, 0, 2), // X 方向梁（下端）
            line_elem(2, 0, 3), // Y 方向梁（下端）
            line_elem(3, 1, 4), // X 方向梁（上端）
            line_elem(4, 1, 5), // Y 方向梁（上端）
        ];
        let model = Model {
            nodes,
            elements,
            sections: vec![section(2.0e8)],
            materials: vec![steel_material()],
            ..Default::default()
        };
        let index = BeamNodeIndex::build(&model);
        let (k_y, k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");
        let expected_ky = sway_buckling_k(2.0, 2.0);
        let expected_kz = sway_buckling_k(0.3, 0.3);
        assert!(
            (k_y - expected_ky).abs() < 1e-9,
            "k_y={k_y}, expected={expected_ky}"
        );
        assert!(
            (k_z - expected_kz).abs() < 1e-6,
            "k_z={k_z}, expected={expected_kz}"
        );
    }

    /// 45° 斜め梁（水平投影が強軸たわみ方向と 45°）: `cos²θ=0.5` の重みにより、
    /// 梁が完全に整列する場合（G=beam_len/col_len=2）に対して G が 2 倍
    /// （=4）になることを確認する。上端は梁が無く G=10。
    #[test]
    fn steel_column_k_axes_diagonal_beam_half_weight() {
        let diag = 8000.0 / std::f64::consts::SQRT_2;
        let nodes = vec![
            node(0, 0.0, 0.0, 0.0),
            node(1, 0.0, 0.0, 4000.0),
            node(2, diag, diag, 0.0),
        ];
        let elements = vec![
            line_elem(0, 0, 1), // 柱（ey≈X・ez≈Y）
            line_elem(1, 0, 2), // 45°梁（水平投影 (1,1,0)/√2 方向、長さ8000）
        ];
        let model = Model {
            nodes,
            elements,
            sections: vec![section(2.0e8)],
            materials: vec![steel_material()],
            ..Default::default()
        };
        let index = BeamNodeIndex::build(&model);
        let (k_y, _k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");
        // 整列時 G=8000/4000=2 に対し cos²θ=0.5 で G=2/0.5=4。上端は梁が無く G=10。
        let expected = sway_buckling_k(4.0, 10.0);
        assert!(
            (k_y - expected).abs() < 1e-6,
            "k_y={k_y}, expected={expected}"
        );
    }

    /// 節点に直交配置の他柱がある場合: 他柱自身の強軸たわみ方向 `ey` と評価方向
    /// のなす角 `cos²β` により、他柱の `I_eff` が `iy`/`iz` へ投影されて
    /// 柱側の集計に加わることを確認する（対象柱の下端に、強軸を Y に向けた
    /// 直交他柱と X 方向梁が取り付く。他柱は評価軸 y に対して cos²β=0 と
    /// なるため iz のみを負担する）。
    #[test]
    fn steel_column_k_axes_orthogonal_other_column_projects_i() {
        let nodes = vec![
            node(0, 0.0, 0.0, 0.0),     // 共有節点（対象柱の下端・他柱の上端）
            node(1, 0.0, 0.0, 4000.0),  // 対象柱の上端
            node(2, 0.0, 0.0, -3000.0), // 他柱の下端
            node(3, 6000.0, 0.0, 0.0),  // X 方向梁の遠端
        ];
        let mut other_col = line_elem(1, 2, 0);
        // 強軸たわみ方向を Y に回転（垂直材は ey=ref_vector の水平成分）。
        other_col.local_axis.ref_vector = [0.0, 1.0, 0.0];
        let elements = vec![
            line_elem(0, 0, 1), // 対象柱（ey≈X・ez≈Y）
            other_col,          // 直交配置の他柱（ey≈Y・ez≈X）
            line_elem(2, 0, 3), // X 方向梁
        ];
        let model = Model {
            nodes,
            elements,
            sections: vec![section(2.0e8)],
            materials: vec![steel_material()],
            ..Default::default()
        };
        let index = BeamNodeIndex::build(&model);
        let (k_y, k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");

        // 下端 G_y: 対象柱自身(iy、cos²β=1)＋他柱(iz、cos²β=0 のため iz のみ)を
        // 柱側に、X 方向梁(iy)を梁側に集計。上端は取付部材が無く G=10。
        let e = 205_000.0;
        let (iy, iz) = (2.0e8, 2.0e7);
        let sum_col_y = e * iy / 4000.0 + e * iz / 3000.0;
        let sum_beam_y = e * iy / 6000.0;
        let expected_ky = sway_buckling_k(sum_col_y / sum_beam_y, 10.0);
        assert!(
            (k_y - expected_ky).abs() < 1e-6,
            "k_y={k_y}, expected={expected_ky}"
        );

        // Z 方向には整列する梁が無い（X 方向梁の cos²θ=0）ため両端 G=10。
        let expected_kz = sway_buckling_k(10.0, 10.0);
        assert!(
            (k_z - expected_kz).abs() < 1e-9,
            "k_z={k_z}, expected={expected_kz}"
        );
    }

    /// 節点側の梁端が `Pinned` の場合、その梁は Σ梁 に不算入となり G=10 相当
    /// （柱側のみで梁側 Σ=0 のため）になることを、軸別評価版
    /// （[`g_ratio_axis_at`] を通る [`steel_column_k_axes_with_index`]）で確認する。
    /// 下端の梁 `line_elem(1, 0, 2)` の節点0側（`end_cond[0]`）を Pinned にする。
    #[test]
    fn steel_column_k_axes_pinned_beam_end_excluded_from_sum() {
        let mut model = portal_model(4000.0, 8000.0);
        model.elements[1].end_cond[0] = EndCondition::Pinned;
        let index = BeamNodeIndex::build(&model);
        let (k_y, k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");
        // 下端: 梁が不算入となり Σ梁=0 → G=10。上端は従来通り G=2。
        let expected_ky = sway_buckling_k(10.0, 2.0);
        // 平面ポータルモデルは Y 方向（弱軸）に梁が無いため元々両端 G=10 のまま。
        let expected_kz = sway_buckling_k(10.0, 10.0);
        assert!(
            (k_y - expected_ky).abs() < 1e-9,
            "k_y={k_y}, expected={expected_ky}"
        );
        assert!(
            (k_z - expected_kz).abs() < 1e-9,
            "k_z={k_z}, expected={expected_kz}"
        );
    }

    /// 梁に剛域を与えると内法長 L' が短くなり Σ梁（E・I/L'）が増大 → G が
    /// 減少（K も減少）することを、軸別評価版で確認する。下端の梁
    /// （長さ8000）に length_i=2000 を与え、L'=6000 になる場合と比較する。
    #[test]
    fn steel_column_k_axes_rigid_zone_shortens_clear_length() {
        let mut model = portal_model(4000.0, 8000.0);
        model.elements[1].rigid_zone = RigidZone {
            length_i: 2000.0,
            ..RigidZone::default()
        };
        let index = BeamNodeIndex::build(&model);
        let (k_y, _k_z) = steel_column_k_axes_with_index(&model, &index, &model.elements[0])
            .expect("柱として判定される");
        // 下端 G_y = (E・iy/4000)/(E・iy/(8000-2000)) = 6000/4000 = 1.5（剛域控除前は 2.0）。
        // 上端は剛域を与えていないため従来通り G=2。
        let expected = sway_buckling_k(1.5, 2.0);
        assert!(
            (k_y - expected).abs() < 1e-6,
            "k_y={k_y}, expected={expected}"
        );
        // 剛域控除前（L=8000 のまま）の G=2 より小さいことを確認（L' 短縮で G 減少）。
        let baseline = sway_buckling_k(2.0, 2.0);
        assert!(k_y < baseline, "k_y={k_y}, baseline={baseline}");
    }
}
