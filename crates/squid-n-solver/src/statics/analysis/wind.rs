//! 風荷重の静的解析（令87条・平12建告1454号）。
//!
//! 建物高さ・各層の負担区間・階別見付け幅を算定し、層の水平力を階内の剛床へ
//! 重量比で按分して載荷する。

use std::collections::HashSet;

use squid_n_core::ids::NodeId;
use squid_n_core::model::{Model, Story, StoryLevelKind};
use squid_n_math::solver::SolveError;

use super::config::{SeismicDir, WindStaticCfg};
use super::seismic::{distribute_pi_over_diaphragms, ground_elevation};
use super::Analysis;
use crate::linear::StaticOnce;

/// 階の見付け幅（風向直交方向の座標範囲）。その階の構造節点（`node_ids`、
/// `generated_masters` 除く）の座標範囲(max−min)を用いる（令87条・
/// 平12建告1454号の見付面積算定に対応する階別の精緻化）。
///
/// 該当する構造節点が 1 点以下、または座標範囲が 0（全節点が同一座標）の
/// 階は、階別の見付け幅を決定できないため `fallback`（建物全体の構造節点
/// 座標範囲）を用いる。
pub(super) fn story_wind_width(
    story: &Story,
    model: &Model,
    axis: usize,
    excluded: &HashSet<NodeId>,
    fallback: f64,
) -> f64 {
    let mut min_c = f64::INFINITY;
    let mut max_c = f64::NEG_INFINITY;
    let mut count = 0usize;
    for nid in &story.node_ids {
        if excluded.contains(nid) {
            continue;
        }
        if let Some(n) = model.nodes.get(nid.index()) {
            let c = n.coord[axis];
            min_c = min_c.min(c);
            max_c = max_c.max(c);
            count += 1;
        }
    }
    if count <= 1 {
        return fallback;
    }
    let w = max_c - min_c;
    if w <= 0.0 {
        fallback
    } else {
        w
    }
}

/// 風荷重の建物高さ H・各層の負担区間（`squid_n_load::wind::WindStory`）を
/// 算定する（令87条・平成12年建設省告示第1454号の運用。`Analysis::wind_static`
/// から solve 抜きでテストできるよう分離）。
///
/// - `H`（建物高さ）= GL（地盤面）からPH階を除く最上階の床高さ + `parapet_mm`/2
///   （「建築物の高さと軒の高さとの平均」）。
/// - 最上層の負担区間上端は、`H` とは別に最上階床高さ + `parapet_mm`
///   （パラペット天端）まで延長し、見付面積に算入する（実壁はパラペット
///   天端まで存在するため。`H` には定義どおり半分のみ算入する）。
/// - 見付け幅は [`story_wind_width`] により階ごとに算定する（フォールバックは
///   建物全体の構造節点座標範囲）。
///
/// `normal_stories` は PH階・地下階を除いた地上一般階を下から上へ並べたもの
/// （呼び出し側でフィルタ済みであること。地下階は地盤内にあり風圧力を受けない）。
/// 空の場合は呼び出し側で弾く前提。`base` には GL（[`ground_elevation`]）を渡す。
pub(super) fn wind_story_geometry(
    model: &Model,
    normal_stories: &[&Story],
    base: f64,
    axis: usize,
    excluded: &HashSet<NodeId>,
    parapet_mm: f64,
) -> Result<(f64, Vec<squid_n_load::wind::WindStory>), SolveError> {
    let top_floor_mm = normal_stories.last().unwrap().elevation - base;
    let h_mm = top_floor_mm + parapet_mm / 2.0;
    if h_mm <= 0.0 {
        return Err(SolveError::InvalidInput(
            "建物高さが 0 以下です。階の標高(elevation)設定を確認してください。".into(),
        ));
    }

    // 建物全体の構造節点座標範囲（階別見付け幅が決定できない場合のフォールバック）。
    let mut min_c = f64::INFINITY;
    let mut max_c = f64::NEG_INFINITY;
    for n in &model.nodes {
        if excluded.contains(&n.id) {
            continue;
        }
        let c = n.coord[axis];
        min_c = min_c.min(c);
        max_c = max_c.max(c);
    }
    if !min_c.is_finite() || !max_c.is_finite() {
        return Err(SolveError::InvalidInput(
            "見付け幅を算定できる構造節点がありません。".into(),
        ));
    }
    let fallback_width = max_c - min_c;
    if fallback_width <= 0.0 {
        return Err(SolveError::InvalidInput(
            "見付け幅が 0 以下です。構造節点の座標を確認してください。".into(),
        ));
    }

    // 層の負担高さ区間（GL＝基部レベル基準）。層 i の負担区間は
    // [中間高さ(i-1,i), 中間高さ(i,i+1)]、最下層は基部から、最上層は
    // パラペット天端まで。
    let elevations: Vec<f64> = normal_stories.iter().map(|s| s.elevation - base).collect();
    let n = elevations.len();
    let parapet_top_mm = top_floor_mm + parapet_mm;
    let wind_stories: Vec<squid_n_load::wind::WindStory> = (0..n)
        .map(|i| {
            let z_bottom = if i == 0 {
                0.0
            } else {
                0.5 * (elevations[i - 1] + elevations[i])
            };
            let z_top = if i == n - 1 {
                parapet_top_mm
            } else {
                0.5 * (elevations[i] + elevations[i + 1])
            };
            let width = story_wind_width(normal_stories[i], model, axis, excluded, fallback_width);
            squid_n_load::wind::WindStory {
                z_bottom,
                z_top,
                width,
            }
        })
        .collect();

    Ok((h_mm, wind_stories))
}

impl Analysis<'_> {
    /// 風荷重の静的解析（令87条・平成12年建設省告示第1454号の運用）。
    ///
    /// - 建物高さ H・各層の負担区間・見付け幅は [`wind_story_geometry`] を
    ///   参照（パラペット割増し・階別見付け幅の詳細はそちらのドキュメント）。
    /// - PH階は建物高さの算定・風荷重の負担層のいずれからも除外する
    ///   （PH階への風荷重接続は未対応。残課題）。
    /// - 地下階は地盤内にあり受風面を持たないため負担層から除外し、
    ///   高さ z は GL（地盤面）基準で測る（従来は最下節点レベル基準で
    ///   地下階の壁面まで見付面積へ算入していた）。
    /// - 層の水平力は §1.6 と同じ規則で階内の剛床へ重量比按分する。
    pub fn wind_static(&self, cfg: WindStaticCfg) -> Result<StaticOnce, SolveError> {
        let model = self.model;
        if model.stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "階(Story)が定義されていません。風荷重には階の定義・剛床(ダイアフラム)が必要です。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }

        let normal_stories: Vec<&Story> = model
            .stories
            .iter()
            .filter(|s| matches!(s.level_kind, StoryLevelKind::Normal))
            .collect();
        if normal_stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "風荷重の対象となる階(PH階・地下階を除く地上一般階)が定義されていません。".into(),
            ));
        }

        let base = ground_elevation(model);
        let axis = match cfg.dir {
            SeismicDir::X => 1, // X方向の風 → 見付け幅はY方向の座標範囲
            SeismicDir::Y => 0,
        };
        let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();
        let (h_mm, wind_stories) = wind_story_geometry(
            model,
            &normal_stories,
            base,
            axis,
            &excluded,
            cfg.parapet_mm,
        )?;

        let wcfg = squid_n_load::wind::WindCfg {
            v0: cfg.v0,
            roughness: cfg.roughness,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: cfg.cpi,
        };
        let dist = squid_n_load::wind::wind_forces(h_mm, &wind_stories, &wcfg);

        let dir_vec = match cfg.dir {
            SeismicDir::X => [1.0, 0.0, 0.0],
            SeismicDir::Y => [0.0, 1.0, 0.0],
        };

        // 各層の水平力を剛床へ重量比按分して作用させる（§1.6 と同じ規則）。
        let mut nodal: Vec<squid_n_core::model::NodalLoad> = Vec::new();
        for (story, &force) in normal_stories.iter().zip(dist.force.iter()) {
            if force == 0.0 {
                continue;
            }
            for (master, share) in distribute_pi_over_diaphragms(story, force) {
                let f = [dir_vec[0] * share, dir_vec[1] * share, 0.0, 0.0, 0.0, 0.0];
                nodal.push(squid_n_core::model::NodalLoad {
                    node: master,
                    values: f,
                });
            }
        }

        if nodal.is_empty() {
            return Err(SolveError::InvalidInput(
                "風荷重を作用させる剛床(ダイアフラム)が階に定義されていません。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }

        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }

        let f_free = self.assemble_f_free_from_nodal(&nodal);
        self.solve_and_recover(&f_free)
    }
}
