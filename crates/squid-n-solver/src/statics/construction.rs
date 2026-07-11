//! 施工時解析（施工段階解析）。RESP-D マニュアル計算編 03「応力解析 §施工時解析」。
//!
//! 原典の規定:
//! - 指定により、長期応力解析を施工段階解析とすることができる。
//! - 下層から順に1層ずつ部材を生成する。
//! - 1層部材を生成するごとに、その層の固定荷重を載荷する。
//! - 部材生成時には、これから生成される層の節点は初期座標に存在するものとして
//!   部材生成する。したがって部材生成時には節点変位が0の状態から解析が開始し、
//!   下階の鉛直変位は累積されない変位となる。
//! - すべての部材生成および固定荷重を載荷した後、積載荷重を載荷する。
//!
//! ## 実装方針
//! [`crate::linear::linear_static_once`] の機械（DofMap 構築 → 剛性・荷重組立 →
//! Reducer による拘束低減 → 求解 → 変位展開 → 部材内力回収）を、層（ステージ）ごとに
//! 呼び出す。各ステージでは
//! - 剛性: そのステージまでに生成された部材のみからなる部分構造（未生成の部材は
//!   モデルから除外）。
//! - 荷重: 固定荷重ケースのうち、そのステージで新たに生成された節点・部材に
//!   作用する分のみを増分として載荷する（既に載荷済みの荷重は再載荷しない）。
//! - 未生成の節点（まだ存在しない、将来のステージで生成される節点）は全自由度を
//!   拘束し解析対象から除外する（RESP-D では自重は既に節点/部材荷重へ変換済みで
//!   あり、剛性・荷重組立コード（`assemble_global_f`）は密度から自動で体積力を
//!   組み立てていないため、荷重ケースの nodal/member フィルタのみで施工増分を
//!   再現できる）。
//!
//! 各ステージの解（変位・部材内力の「増分」）を、節点/部材がそれぞれ生成された
//! ステージ以降について足し合わせることで最終的な累積値を得る。未生成の節点は
//! 該当ステージで全自由度拘束（変位0）となるため、単純に全ステージの増分を
//! 加算するだけで「生成前のステージは寄与しない」という規則が自動的に満たされる。
//!
//! 全ステージ完了後（＝完成形の全部材が生成された状態）に、指定があれば積載荷重
//! ケースを完成形へ載荷し、同様に累積する。

use crate::linear::{linear_static_once, StaticOnce};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId};
use squid_n_core::model::{LoadCase, LoadCaseKind, Model};
use squid_n_element::beam::MemberForces;
use squid_n_math::solver::SolveError;
use std::collections::HashMap;

/// 施工時解析（施工段階解析）の結果。
///
/// `disp` / `member_forces` は全ステージ（＋積載荷重）を通じた最終的な累積値であり、
/// [`crate::linear::StaticOnce`] と同じ形式（節点順 `[f64; 6]`、
/// `Vec<(ElemId, MemberForces)>`）で取り出せる。呼び出し側で長期応力解析結果を
/// 本結果へ置き換えるだけで済むようにするため。
pub struct ConstructionResult {
    /// 節点変位（節点順、`[Ux,Uy,Uz,Rx,Ry,Rz]`）。各節点の値は、その節点が
    /// 生成されたステージ以降の増分変位の和（生成前のステージの増分は0であり
    /// 寄与しない）。
    pub disp: Vec<[f64; 6]>,
    /// 部材内力。部材が生成されたステージ以降の増分内力の和。
    pub member_forces: Vec<(ElemId, MemberForces)>,
    /// 層生成に分割したステージ数（積載荷重の載荷ステージは含まない）。
    pub n_stages: usize,
}

impl ConstructionResult {
    /// `StaticOnce` 互換の形へ変換する（長期解析結果の置き換え用）。
    pub fn as_static_once(&self) -> StaticOnce {
        StaticOnce {
            disp: self.disp.clone(),
            member_forces: self.member_forces.clone(),
        }
    }
}

/// 施工段階解析を実行する。
///
/// `dead` は固定荷重ケース（各ステージで新規生成分のみを増分載荷）、`live` は
/// 完成形へ最後に載荷する積載荷重ケース（`None` なら積載荷重は載荷しない）。
///
/// 部材の帰属層は、部材の最高節点 z 座標が属する層（`elevation` 以下で最も近い層）
/// で決める。該当する層が無い部材（全層の `elevation` より下にある部材、例えば
/// 基礎梁など）は最初のステージに含める。節点の帰属層も同じ規則を節点自身の
/// z 座標に適用して決める（部材の帰属層は、その部材を構成する節点の帰属層の
/// 最大値と一致する）。
pub fn construction_stage_analysis(
    model: &Model,
    dead: LoadCaseId,
    live: Option<LoadCaseId>,
) -> Result<ConstructionResult, SolveError> {
    let n_nodes = model.nodes.len();

    // 層を elevation 昇順に並べ替える（原典: 下層から順に生成）。
    let mut elevations: Vec<f64> = model.stories.iter().map(|s| s.elevation).collect();
    elevations.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n_stages = elevations.len().max(1);

    // z 座標から帰属ステージ（0 始まり）を決める規則。
    // 「elevation 以下で最も近い層」＝ elevations は昇順なので、
    // e[i] <= z を満たす最大の i。該当が無ければ 0（＝最初のステージに含める）。
    const EPS: f64 = 1e-6;
    let stage_of_z = |z: f64| -> usize {
        let mut found = None;
        for (i, &e) in elevations.iter().enumerate() {
            if e <= z + EPS {
                found = Some(i);
            } else {
                break;
            }
        }
        found.unwrap_or(0)
    };

    let node_stage: Vec<usize> = model.nodes.iter().map(|n| stage_of_z(n.coord[2])).collect();
    let elem_stage: HashMap<ElemId, usize> = model
        .elements
        .iter()
        .map(|e| {
            let z_max = e
                .nodes
                .iter()
                .map(|nid| model.nodes[nid.index()].coord[2])
                .fold(f64::MIN, f64::max);
            (e.id, stage_of_z(z_max))
        })
        .collect();

    let dead_case = model
        .load_cases
        .iter()
        .find(|lc| lc.id == dead)
        .ok_or_else(|| {
            SolveError::InvalidInput(format!("固定荷重ケース {:?} が見つかりません", dead))
        })?;

    // 既存の LoadCaseId と衝突しない一時 ID（ステージごとの増分荷重ケース用）。
    let synth_base = model.load_cases.iter().map(|lc| lc.id.0).max().unwrap_or(0) + 1;

    let mut disp_acc = vec![[0.0_f64; 6]; n_nodes];
    let mut forces_acc: Vec<(ElemId, MemberForces)> = Vec::new();

    for stage in 0..n_stages {
        // このステージまでに生成された部材のみを残した部分構造モデル。
        let mut partial = model.clone();
        partial.elements.retain(|e| elem_stage[&e.id] <= stage);

        // まだ生成されていない節点は全自由度拘束し解析対象から除外する
        // （原典: 生成前の節点は初期座標に存在するのみで、変位自由度を持たない）。
        for (ni, node) in partial.nodes.iter_mut().enumerate() {
            if node_stage[ni] > stage {
                node.restraint = Dof6Mask::FIXED;
            }
        }

        // このステージで新規生成された節点・部材にのみ作用する固定荷重の増分。
        let nodal_inc: Vec<_> = dead_case
            .nodal
            .iter()
            .filter(|nl| node_stage[nl.node.index()] == stage)
            .cloned()
            .collect();
        let member_inc: Vec<_> = dead_case
            .member
            .iter()
            .filter(|ml| elem_stage.get(&ml.elem).copied() == Some(stage))
            .cloned()
            .collect();

        let synth_id = LoadCaseId(synth_base + stage as u32);
        partial.load_cases.push(LoadCase {
            id: synth_id,
            name: format!("__construction_stage_{stage}"),
            nodal: nodal_inc,
            member: member_inc,
            kind: LoadCaseKind::Dead,
        });

        let stage_res = linear_static_once(&partial, synth_id)?;
        accumulate(&mut disp_acc, &mut forces_acc, &stage_res);
    }

    // 全ステージ完了後（＝完成形）に積載荷重を載荷する。
    if let Some(live_id) = live {
        let live_res = linear_static_once(model, live_id)?;
        accumulate(&mut disp_acc, &mut forces_acc, &live_res);
    }

    Ok(ConstructionResult {
        disp: disp_acc,
        member_forces: forces_acc,
        n_stages,
    })
}

/// ステージ（または積載荷重）1回分の解を、累積アキュムレータへ加算する。
/// 変位は節点順にそのまま加算（未生成の節点は当該ステージで拘束済み＝増分0）。
/// 部材内力は断面位置 `xi` が一致する項同士を加算する。
fn accumulate(
    disp_acc: &mut [[f64; 6]],
    forces_acc: &mut Vec<(ElemId, MemberForces)>,
    stage_res: &StaticOnce,
) {
    for (ni, d) in stage_res.disp.iter().enumerate() {
        for k in 0..6 {
            disp_acc[ni][k] += d[k];
        }
    }
    for (eid, mf) in &stage_res.member_forces {
        if let Some((_, acc_mf)) = forces_acc.iter_mut().find(|(id, _)| id == eid) {
            for (xi, vals) in &mf.at {
                if let Some((_, acc_vals)) = acc_mf
                    .at
                    .iter_mut()
                    .find(|(axi, _)| (*axi - *xi).abs() < 1e-6)
                {
                    for k in 0..6 {
                        acc_vals[k] += vals[k];
                    }
                } else {
                    acc_mf.at.push((*xi, *vals));
                }
            }
        } else {
            forces_acc.push((*eid, mf.clone()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind,
        LocalAxis, Material, Model, NodalLoad, Node, Section, Story, StoryLevelKind,
        StoryStructure,
    };

    /// 2 階建ての鉛直柱列（軸力のみが卓越する片持ち柱チェーン）を作る。
    /// node0(基部, z=0, 固定) - elem0 - node1(z=h, story0) - elem1 - node2(z=2h, story1)
    /// 各層の固定荷重は各層の節点に作用する鉛直下向き集中荷重として与える。
    fn two_story_column(p1: f64, p2: f64) -> (Model, LoadCaseId) {
        let h = 3000.0_f64;
        let section = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 400.0,
            width: 400.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let material = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let mk_elem = |id: u32, ni: u32, nj: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(ni), NodeId(nj)],
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
        };
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
                    coord: [0.0, 0.0, h],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 2.0 * h],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(1)),
                },
            ],
            elements: vec![mk_elem(0, 0, 1), mk_elem(1, 1, 2)],
            sections: vec![section],
            materials: vec![material],
            stories: vec![
                Story {
                    id: StoryId(0),
                    name: "1F".to_string(),
                    elevation: h,
                    node_ids: vec![NodeId(1)],
                    diaphragms: vec![DiaphragmDef {
                        ci_override: None,
                        weight: None,
                        master: NodeId(1),
                        slaves: vec![],
                        rigid: false,
                    }],
                    seismic_weight: None,
                    structure: StoryStructure::default(),
                    level_kind: StoryLevelKind::default(),
                },
                Story {
                    id: StoryId(1),
                    name: "2F".to_string(),
                    elevation: 2.0 * h,
                    node_ids: vec![NodeId(2)],
                    diaphragms: vec![DiaphragmDef {
                        ci_override: None,
                        weight: None,
                        master: NodeId(2),
                        slaves: vec![],
                        rigid: false,
                    }],
                    seismic_weight: None,
                    structure: StoryStructure::default(),
                    level_kind: StoryLevelKind::default(),
                },
            ],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "dead".to_string(),
                nodal: vec![
                    NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, -p1, 0.0, 0.0, 0.0],
                    },
                    NodalLoad {
                        node: NodeId(2),
                        values: [0.0, 0.0, -p2, 0.0, 0.0, 0.0],
                    },
                ],
                member: vec![],
                kind: LoadCaseKind::Dead,
            }],
            ..Default::default()
        };
        (model, LoadCaseId(1))
    }

    fn axial_force(mf: &MemberForces) -> f64 {
        // 部材内力の N（軸力）は断面位置によらず一定（自重・軸荷重のみ）。
        // 代表として最初の断面の値を用いる。
        mf.at[0].1[0]
    }

    /// 施工段階解析の柱軸力（長期）が一括解析と一致すること。
    /// 直列の柱チェーンに鉛直荷重のみを与えた静定に近いモデルでは、各部材の
    /// 軸力はその部材が支える上方荷重の総和で決まり、施工順序に依存しない。
    #[test]
    fn axial_force_matches_batch_analysis() {
        let (model, dead) = two_story_column(1000.0, 500.0);

        let staged = construction_stage_analysis(&model, dead, None).expect("staged solve");
        assert_eq!(staged.n_stages, 2);

        let batch = linear_static_once(&model, dead).expect("batch solve");

        for eid in [ElemId(0), ElemId(1)] {
            let (_, mf_staged) = staged
                .member_forces
                .iter()
                .find(|(id, _)| *id == eid)
                .expect("staged member forces");
            let (_, mf_batch) = batch
                .member_forces
                .iter()
                .find(|(id, _)| *id == eid)
                .expect("batch member forces");
            let n_staged = axial_force(mf_staged);
            let n_batch = axial_force(mf_batch);
            assert!(
                (n_staged - n_batch).abs() < 1e-6 * n_batch.abs().max(1.0),
                "elem {:?}: staged N={} batch N={}",
                eid,
                n_staged,
                n_batch
            );
        }
    }

    /// 上層節点の鉛直変位は、一括解析より小さくなる（下階の鉛直変位は
    /// 累積されないため）。
    #[test]
    fn upper_node_displacement_smaller_than_batch() {
        let (model, dead) = two_story_column(1000.0, 500.0);

        let staged = construction_stage_analysis(&model, dead, None).expect("staged solve");
        let batch = linear_static_once(&model, dead).expect("batch solve");

        let uz_staged = staged.disp[2][2];
        let uz_batch = batch.disp[2][2];
        assert!(uz_staged < 0.0 && uz_batch < 0.0, "both should settle down");
        assert!(
            uz_staged.abs() < uz_batch.abs(),
            "staged top displacement should be smaller in magnitude: staged={} batch={}",
            uz_staged,
            uz_batch
        );

        // node1（下層節点）が支える総荷重は施工順序に依存しないため、node1 自身の
        // 変位は一括解析と一致する（node1 の上に積み増される荷重は node1 生成時点
        // では未確定だが、後続ステージの増分荷重は結局 node1 を含む部分構造へ
        // 作用し node1 の変位にも寄与するため、最終的な力の釣合は一括解析と同じ）。
        let uz1_staged = staged.disp[1][2];
        let uz1_batch = batch.disp[1][2];
        assert!(
            (uz1_staged - uz1_batch).abs() < 1e-6 * uz1_batch.abs().max(1.0),
            "node1 staged={} batch={}",
            uz1_staged,
            uz1_batch
        );
    }

    /// 1 層モデル（単一ステージ）では、施工時解析は一括解析（固定荷重＋積載荷重の
    /// 重ね合わせ）と完全に一致する。
    #[test]
    fn single_story_matches_batch_with_live_load() {
        let h = 3000.0_f64;
        let section = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 400.0,
            width: 400.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let material = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let dead_id = LoadCaseId(1);
        let live_id = LoadCaseId(2);
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
                    coord: [0.0, 0.0, h],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
            sections: vec![section],
            materials: vec![material],
            stories: vec![Story {
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: h,
                node_ids: vec![NodeId(1)],
                diaphragms: vec![],
                seismic_weight: None,
                structure: StoryStructure::default(),
                level_kind: StoryLevelKind::default(),
            }],
            load_cases: vec![
                LoadCase {
                    id: dead_id,
                    name: "dead".to_string(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, -1000.0, 0.0, 0.0, 0.0],
                    }],
                    member: vec![],
                    kind: LoadCaseKind::Dead,
                },
                LoadCase {
                    id: live_id,
                    name: "live".to_string(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 0.0, -300.0, 0.0, 0.0, 0.0],
                    }],
                    member: vec![],
                    kind: LoadCaseKind::Live,
                },
            ],
            ..Default::default()
        };

        let staged =
            construction_stage_analysis(&model, dead_id, Some(live_id)).expect("staged solve");
        assert_eq!(staged.n_stages, 1);

        let dead_only = linear_static_once(&model, dead_id).expect("dead solve");
        let live_only = linear_static_once(&model, live_id).expect("live solve");

        for ni in 0..model.nodes.len() {
            for k in 0..6 {
                let expected = dead_only.disp[ni][k] + live_only.disp[ni][k];
                assert!(
                    (staged.disp[ni][k] - expected).abs() < 1e-9 * expected.abs().max(1.0),
                    "node {} dof {}: staged={} expected={}",
                    ni,
                    k,
                    staged.disp[ni][k],
                    expected
                );
            }
        }

        let (_, mf_staged) = staged
            .member_forces
            .iter()
            .find(|(id, _)| *id == ElemId(0))
            .expect("staged member forces");
        let (_, mf_dead) = dead_only
            .member_forces
            .iter()
            .find(|(id, _)| *id == ElemId(0))
            .expect("dead member forces");
        let (_, mf_live) = live_only
            .member_forces
            .iter()
            .find(|(id, _)| *id == ElemId(0))
            .expect("live member forces");
        for (xi, vals) in &mf_staged.at {
            let (_, dvals) = mf_dead
                .at
                .iter()
                .find(|(dxi, _)| (dxi - xi).abs() < 1e-9)
                .expect("dead xi");
            let (_, lvals) = mf_live
                .at
                .iter()
                .find(|(lxi, _)| (lxi - xi).abs() < 1e-9)
                .expect("live xi");
            for k in 0..6 {
                let expected = dvals[k] + lvals[k];
                assert!(
                    (vals[k] - expected).abs() < 1e-9 * expected.abs().max(1.0),
                    "xi={} k={}: staged={} expected={}",
                    xi,
                    k,
                    vals[k],
                    expected
                );
            }
        }
    }
}
