//! 層間変形角・剛性率（層間変形角は令82条の2、剛性率は令82条の6・昭55建告1792号）。
//!
//! - **層間変形角（確認用）:** 立体解析のため層間変形角は位置ごとに異なる。
//!   その階の**柱の層間変形角の最大値**を用いて確認する（斜め柱は除外）:
//!   `1/irs = max(δ1, δ2, …, δn) / iH`（δ: 柱頭の変位−柱脚の変位（加力方向の水平変位））
//! - **剛性率算出用の層間変形角:** 重心位置の層間変位 δg を用いる:
//!   `1/irs = iδg / iH`, `Rs = rs / r̄s ≧ 0.6`（r̄s は相加平均）
//!
//! 剛性率の式自体（Ks=H/δ, Rs=Ks/mean）は
//! `crate::secondary::holding_capacity::stiffness_ratios` を再利用する。
//! 本モジュールは「δ に何を使うか」を上記の実務的取扱いに沿って揃える層である。

use squid_n_core::ids::{ElemId, StoryId};
use squid_n_core::model::{ElementKind, Model, Node};

/// 柱 1 本の層間変位（加力方向）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColumnDrift {
    pub elem: ElemId,
    /// 柱頭の平面位置 (x, y) [mm]。
    pub pos: [f64; 2],
    /// 層間変位 δ = |柱頭変位 − 柱脚変位| [mm]（加力方向の水平成分）。
    pub drift: f64,
}

/// 斜め柱判定の許容平面ズレ [mm]。柱頭・柱脚の平面位置がこれを超えて
/// 異なる柱は「斜め柱」として層間変形角の確認から除外する（本実装の扱い）。
const INCLINED_PLAN_TOL: f64 = 1.0;

/// 鉛直部材（柱）判定の方向余弦しきい値（`eccentricity::column_stiffnesses` と同じ）。
const VERTICAL_COS_TOL: f64 = 0.707;

/// 当該層に帰属する柱を列挙して `f(elem_id, 柱頭節点, 柱脚節点)` を呼ぶ。
/// 柱の判定: 2 節点 `Beam` かつ部材軸の z 方向余弦 > 0.707。
/// 層帰属: 柱頭（z 大）節点の `story == Some(story)`。
fn for_each_story_column(model: &Model, story: StoryId, mut f: impl FnMut(ElemId, &Node, &Node)) {
    for elem in &model.elements {
        if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let n0 = &model.nodes[elem.nodes[0].index()];
        let n1 = &model.nodes[elem.nodes[1].index()];
        let d = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-12 || (d[2] / l).abs() <= VERTICAL_COS_TOL {
            continue;
        }
        let (top, bot) = if n0.coord[2] < n1.coord[2] {
            (n1, n0)
        } else {
            (n0, n1)
        };
        if top.story != Some(story) {
            continue;
        }
        f(elem.id, top, bot);
    }
}

/// 当該層の各柱の層間変位（加力方向）。**斜め柱は除外**する
/// （柱頭・柱脚の平面位置が `INCLINED_PLAN_TOL` を超えてずれる柱）。
///
/// `disp` は節点変位（`model.nodes` と同順）、`dir` は加力方向（0=X, 1=Y）。
pub fn column_drifts(
    model: &Model,
    disp: &[[f64; 6]],
    dir: usize,
    story: StoryId,
) -> Vec<ColumnDrift> {
    let mut out = Vec::new();
    for_each_story_column(model, story, |eid, top, bot| {
        let plan_off =
            ((top.coord[0] - bot.coord[0]).powi(2) + (top.coord[1] - bot.coord[1]).powi(2)).sqrt();
        if plan_off > INCLINED_PLAN_TOL {
            return; // 斜め柱は除外
        }
        let (Some(ut), Some(ub)) = (disp.get(top.id.index()), disp.get(bot.id.index())) else {
            return;
        };
        out.push(ColumnDrift {
            elem: eid,
            pos: [top.coord[0], top.coord[1]],
            drift: (ut[dir] - ub[dir]).abs(),
        });
    });
    out
}

/// 当該層の柱の層間変位の最大値 `max(δ1, δ2, …, δn)` [mm]（層間変形角の式の分子）。
/// 柱が無い（または全て斜め柱の）層は `None`。
pub fn max_column_drift(
    model: &Model,
    disp: &[[f64; 6]],
    dir: usize,
    story: StoryId,
) -> Option<ColumnDrift> {
    column_drifts(model, disp, dir, story)
        .into_iter()
        .max_by(|a, b| a.drift.total_cmp(&b.drift))
}

/// 当該層の重心位置の水平変位 [mm]（加力方向）。
///
/// 剛床の並進＋回転を仮定すると質量重み付き平均変位は重心位置の変位に一致する
/// （u_i = u_g + θ×(r_i − r_g) の質量加重平均は u_g）。質量未定義の節点は重み 0、
/// 層の全質量が 0 の場合は単純平均で代用する。
pub fn cog_horizontal_disp(model: &Model, disp: &[[f64; 6]], dir: usize, story: StoryId) -> f64 {
    let nodes: Vec<&Node> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(story))
        .collect();
    if nodes.is_empty() {
        return 0.0;
    }
    let mass = |n: &Node| n.mass.map(|m| m[0]).unwrap_or(0.0);
    let total: f64 = nodes.iter().map(|n| mass(n)).sum();
    let get = |n: &Node| disp.get(n.id.index()).map(|u| u[dir]).unwrap_or(0.0);
    if total > 0.0 {
        nodes.iter().map(|n| mass(n) * get(n)).sum::<f64>() / total
    } else {
        nodes.iter().map(|n| get(n)).sum::<f64>() / nodes.len() as f64
    }
}

/// 全層の「重心位置の層間変位」δg [mm]（下階→上階順）。
/// `iδg = 当該層の重心位置変位 − 直下層の重心位置変位`（最下層は基部変位 0 とみなす）。
/// 剛性率 Rs の算出（`crate::secondary::holding_capacity::stiffness_ratios`）に
/// この δg を渡す。
pub fn cog_story_drifts(model: &Model, disp: &[[f64; 6]], dir: usize) -> Vec<f64> {
    let disp_g: Vec<f64> = model
        .stories
        .iter()
        .map(|s| cog_horizontal_disp(model, disp, dir, s.id))
        .collect();
    disp_g
        .iter()
        .enumerate()
        .map(|(i, d)| {
            let below = if i == 0 { 0.0 } else { disp_g[i - 1] };
            (d - below).abs()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Node, RigidZone, Story,
    };

    /// 1 層 3 本柱（2 本鉛直・1 本斜め）のテストモデル。
    /// 柱1: (0,0), 柱2: (6000,0), 斜め柱: 脚(3000,0)→頭(3000+1500,0)。
    fn build_model() -> (Model, StoryId) {
        let s0 = StoryId(0);
        let mut nodes = Vec::new();
        // 柱脚（z=0, 拘束）
        let feet = [[0.0, 0.0], [6000.0, 0.0], [3000.0, 0.0]];
        for (i, &[x, y]) in feet.iter().enumerate() {
            nodes.push(Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            });
        }
        // 柱頭（z=3000, story=S0）。斜め柱の頭は x を 1500 ずらす。
        let heads = [[0.0, 0.0], [6000.0, 0.0], [4500.0, 0.0]];
        let masses = [2.0, 1.0, 1.0]; // 重心を柱1側へ寄せる
        for (i, &[x, y]) in heads.iter().enumerate() {
            nodes.push(Node {
                id: NodeId((i + 3) as u32),
                coord: [x, y, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: Some([masses[i], masses[i], masses[i], 0.0, 0.0, 0.0]),
                story: Some(s0),
            });
        }
        let elements: Vec<ElementData> = (0..3)
            .map(|i| ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: {
                    let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                    v.push(NodeId(i as u32));
                    v.push(NodeId((i + 3) as u32));
                    v
                },
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
                spring: None,
            })
            .collect();
        let story = Story {
            id: s0,
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(3), NodeId(4), NodeId(5)],
            diaphragms: vec![],
            seismic_weight: None,
            structure: Default::default(),
            level_kind: Default::default(),
        };
        let model = Model {
            nodes,
            elements,
            stories: vec![story],
            ..Default::default()
        };
        (model, s0)
    }

    /// 変位場: 柱頭 X 変位 = [10, 20, 15]、柱脚 0。
    fn disp_field() -> Vec<[f64; 6]> {
        let mut d = vec![[0.0; 6]; 6];
        d[3][0] = 10.0;
        d[4][0] = 20.0;
        d[5][0] = 15.0;
        d
    }

    #[test]
    fn test_max_column_drift_excludes_inclined() {
        let (model, s0) = build_model();
        let disp = disp_field();
        let drifts = column_drifts(&model, &disp, 0, s0);
        // 斜め柱（ElemId 2, δ=15）は除外され、鉛直 2 本のみ。
        assert_eq!(drifts.len(), 2);
        let max = max_column_drift(&model, &disp, 0, s0).unwrap();
        assert_eq!(max.elem, ElemId(1));
        assert!((max.drift - 20.0).abs() < 1e-12);
    }

    #[test]
    fn test_cog_disp_mass_weighted() {
        let (model, s0) = build_model();
        let disp = disp_field();
        // δg = (2·10 + 1·20 + 1·15) / 4 = 13.75（斜め柱の節点も質量として重心には寄与）
        let dg = cog_horizontal_disp(&model, &disp, 0, s0);
        assert!((dg - 13.75).abs() < 1e-12, "δg={dg}");
        let drifts = cog_story_drifts(&model, &disp, 0);
        assert_eq!(drifts.len(), 1);
        assert!((drifts[0] - 13.75).abs() < 1e-12);
    }

    #[test]
    fn test_cog_disp_unmassed_falls_back_to_average() {
        let (mut model, s0) = build_model();
        for n in &mut model.nodes {
            n.mass = None;
        }
        let disp = disp_field();
        let dg = cog_horizontal_disp(&model, &disp, 0, s0);
        assert!((dg - 15.0).abs() < 1e-12, "単純平均 (10+20+15)/3, got {dg}");
    }
}
