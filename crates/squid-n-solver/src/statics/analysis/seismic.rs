//! 地震静的解析（Ai 分布）の荷重生成と解析メソッド。
//!
//! 略算周期・鉄骨造比、階水平力の剛床への分配（多剛床・副剛床の Ci 直接入力）、
//! Ai 分布による層せん断力から節点水平力の荷重ケースを構築する。

use std::collections::HashSet;

use squid_n_core::ids::{LoadCaseId, NodeId};
use squid_n_core::model::{DiaphragmDef, Model, Story, StoryLevelKind, StoryStructure};
use squid_n_math::solver::SolveError;

use super::config::{AiMode, SeismicCfg, SeismicDir};
use super::Analysis;
use crate::eigen;
use crate::linear::StaticOnce;

/// 建物の基部レベル（elevation の基準 0）を求める。
///
/// 全構造節点（`generated_masters`＝階自動生成が作る剛床代表節点を除く）の
/// 最小 Z 座標を基部とする（レビュー §1.5・§1.7 が参照する「基部レベル」の
/// 共通定義。剛床代表節点は慣性力重心に置かれる仮想節点であり、実際の
/// 構造高さには寄与しないため除外する）。
pub(super) fn base_elevation(model: &Model) -> f64 {
    let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();
    let base = model
        .nodes
        .iter()
        .filter(|n| !excluded.contains(&n.id))
        .map(|n| n.coord[2])
        .fold(f64::INFINITY, f64::min);
    if base.is_finite() {
        base
    } else {
        0.0
    }
}

/// 地盤面（GL）レベル [mm] を求める。
///
/// 地下階（`StoryLevelKind::Basement`）が定義されているモデルでは、
/// 各地下階の「床レベル + 地盤面からの深さ depth_m」から GL を復元する
/// （深さの定義より各地下階で同一値になる想定。数値ずれに備え最大値を採る）。
/// 地下階が無ければ [`base_elevation`]（最下構造節点レベル）を GL とみなす。
///
/// 建築物の高さ（令2条・令88条の略算周期 T の h）や風荷重の受風範囲
/// （地上部分のみ）は、基部レベルではなくこの GL を基準に測る。
pub fn ground_elevation(model: &Model) -> f64 {
    let gl = model
        .stories
        .iter()
        .filter_map(|s| match s.level_kind {
            StoryLevelKind::Basement { depth_m } => Some(s.elevation + depth_m * 1000.0),
            _ => None,
        })
        .fold(f64::NEG_INFINITY, f64::max);
    if gl.is_finite() {
        gl
    } else {
        base_elevation(model)
    }
}

/// 建築物の高さ h [mm]（略算周期 T = h(0.02+0.01α) の h。令88条・
/// 昭和55年建設省告示第1793号）。
///
/// GL（[`ground_elevation`]）から、PH（塔屋）階を除く最上の一般階の
/// 床レベルまでの高さとする。地下階の深さ・塔屋の高さは h に算入しない。
/// 一般階が無い場合は最上階レベルで代用し、負値は 0 にクランプする。
pub fn building_height_mm(model: &Model) -> f64 {
    let gl = ground_elevation(model);
    let top_normal = model
        .stories
        .iter()
        .filter(|s| matches!(s.level_kind, StoryLevelKind::Normal))
        .map(|s| s.elevation)
        .fold(f64::NEG_INFINITY, f64::max);
    let top = if top_normal.is_finite() {
        top_normal
    } else {
        model.stories.last().map(|s| s.elevation).unwrap_or(gl)
    };
    (top - gl).max(0.0)
}

/// 略算周期 T = h(0.02 + 0.01α) の鉄骨造比 α（令88条・昭和55年建設省告示第1793号）。
///
/// 「柱及び梁の大部分が鉄骨造である階（地階を除く）の高さの合計の h（建築物の高さ）
/// に対する比」。`Story.structure` が `S`（鉄骨造）である地上一般階の階高の合計 ÷
/// 建築物の高さ（[`building_height_mm`]）。RC・SRC は分子に算入せず、告示の
/// 「地階を除く」に従い地下階は分子・分母とも算入しない。PH（塔屋）階も
/// 建築物の高さに算入しない扱いに合わせて対象外とする。
///
/// 階高 h_i = elevation_i − elevation_{i−1}（最下の地上一般階は GL を
/// elevation_{-1} とみなす）。階が定義されていない、または建築物の高さが 0 以下の
/// 場合は 0.0 を返す（レビュー §1.5：従来はこの α を常に 0.0 にハードコード
/// していたバグの修正。地下階・PH 階の除外は照合レビューによる是正）。
pub fn steel_height_ratio(model: &Model) -> f64 {
    if model.stories.is_empty() {
        return 0.0;
    }
    let gl = ground_elevation(model);
    let total_h = building_height_mm(model);
    if total_h <= 0.0 {
        return 0.0;
    }
    let mut prev_elev = gl;
    let mut steel_h = 0.0;
    for story in &model.stories {
        if !matches!(story.level_kind, StoryLevelKind::Normal) {
            continue;
        }
        let h = story.elevation - prev_elev;
        prev_elev = story.elevation;
        if h <= 0.0 {
            continue;
        }
        if matches!(story.structure, StoryStructure::S) {
            steel_h += h;
        }
    }
    (steel_h / total_h).clamp(0.0, 1.0)
}

/// [`distribute_pi_over_diaphragms`] の中核ロジック。剛床定義の列（階全体、
/// または主系統サブセットのいずれか）を受け取り Pi を重量比で分配する。
fn distribute_pi_over_slice(diaphragms: &[DiaphragmDef], pi: f64) -> Vec<(NodeId, f64)> {
    let n = diaphragms.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(diaphragms[0].master, pi)];
    }
    let total_weight: f64 = diaphragms.iter().filter_map(|d| d.weight).sum();
    if total_weight > 0.0 {
        diaphragms
            .iter()
            .map(|d| (d.master, pi * d.weight.unwrap_or(0.0) / total_weight))
            .collect()
    } else {
        let share = pi / n as f64;
        diaphragms.iter().map(|d| (d.master, share)).collect()
    }
}

/// 階の水平力 Pi を階内の剛床（ダイアフラム）ごとに分配する
/// （多剛床の設計用せん断力：水平力を剛床ごとの重量比で分配する）。
///
/// - 剛床が 1 つの階: 従来どおり Pi を全量その剛床へ載せる。
/// - 剛床が複数の階: `DiaphragmDef.weight` の比で按分する（`None` は 0 扱い）。
///   重量の合計が 0（すべて未設定を含む）の場合は等分割する
///   （レビュー §1.6：従来は各剛床へ Pi をそのまま重複して載せており、
///   多剛床の階では地震力が剛床数倍に水増しされるバグだった）。
///
/// `ci_override`（副剛床の Ci 直接入力）は考慮しない。風荷重など Ci の
/// 概念が無い荷重ケースはこの関数をそのまま使う。地震荷重は
/// [`distribute_seismic_forces`] を使う。
pub(crate) fn distribute_pi_over_diaphragms(story: &Story, pi: f64) -> Vec<(NodeId, f64)> {
    distribute_pi_over_slice(&story.diaphragms, pi)
}

/// 階の主系統（Ai 分布：昭55建告1793号）に用いる地震用重量（副剛床の Ci を
/// 直接入力した場合の扱い）。`ci_override` を持つ剛床の重量は主系統の Ai 分布から
/// 除外する（主剛床は全剛床の Ci に従うが、副剛床は指定 Ci で別途計算するため）。
/// `ci_override` を持つ剛床が無ければ `story.seismic_weight` をそのまま返す
/// （既存挙動と厳密一致）。
pub(super) fn main_system_weight(story: &Story) -> f64 {
    let total = story.seismic_weight.unwrap_or(0.0);
    let ci_override_weight: f64 = story
        .diaphragms
        .iter()
        .filter(|d| d.ci_override.is_some())
        .map(|d| d.weight.unwrap_or(0.0))
        .sum();
    total - ci_override_weight
}

/// 階の水平力 Pi を剛床へ分配する（地震荷重版。副剛床の Ci 直接入力に対応）。
///
/// - `ci_override` を持たない剛床（主系統）: Pi を重量比で分配する
///   （[`distribute_pi_over_diaphragms`] と同じ規則）。
/// - `ci_override` を持つ剛床（副剛床）: Pi の分配対象から除外し、代わりに
///   水平力 = `ci_override` × その剛床の `weight`（`weight=None` なら 0）を
///   別途作用させる（等価震度扱い。上階に同一系統の剛床が積み上がらない
///   副剛床を想定。副剛床の Ci を直接入力した場合の扱い）。
///
/// 全剛床が `ci_override` 無しなら [`distribute_pi_over_diaphragms`] と
/// 厳密に一致する。
pub(crate) fn distribute_seismic_forces(story: &Story, pi: f64) -> Vec<(NodeId, f64)> {
    let main_diaphragms: Vec<DiaphragmDef> = story
        .diaphragms
        .iter()
        .filter(|d| d.ci_override.is_none())
        .cloned()
        .collect();
    let mut result = distribute_pi_over_slice(&main_diaphragms, pi);
    for d in &story.diaphragms {
        if let Some(ci) = d.ci_override {
            result.push((d.master, ci * d.weight.unwrap_or(0.0)));
        }
    }
    result
}

impl Analysis<'_> {
    /// Run seismic static analysis: approx or semi-precise Ai distribution.
    /// SemiPrecise uses eigen T, Approx uses approximate formula.
    ///
    /// 階(Story)・地震重量・剛床が未定義の場合は黙ってゼロ結果を返さず、
    /// 何をすべきかを含むエラーを返す。
    pub fn seismic_static(&self, dir: SeismicDir, mode: AiMode) -> Result<StaticOnce, SolveError> {
        self.seismic_static_with(SeismicCfg {
            dir,
            mode,
            ..SeismicCfg::default()
        })
    }

    /// 地震静的解析（設定指定版）。Z・地盤種別・C0 を UI から与える。
    pub fn seismic_static_with(&self, cfg: SeismicCfg) -> Result<StaticOnce, SolveError> {
        let lc = self.build_seismic_load_case(cfg)?;

        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }

        let f_free = self.assemble_f_free_from_nodal(&lc.nodal);
        self.solve_and_recover(&f_free)
    }

    /// 地震荷重ケース構築に用いる設計用固有周期 T [s] を算定する。
    ///
    /// - `AiMode::Approx`: 略算式 T = h(0.02+0.01α)（令88条・昭和55年建設省
    ///   告示第1793号）。h は建築物の高さ（GL〜PH 階を除く最上階。地下深さ・
    ///   塔屋は含めない。従来の「最上階の生 Z 標高」は、基部が Z=0 に無い
    ///   モデルや地下階付きモデルで h を誤っていた）。
    /// - `AiMode::SemiPrecise`: 固有値解析（1 次モード）による周期。
    ///
    /// T は加力方向（X/Y）によらず同一の値になるため、[`Self::build_seismic_load_case`]
    /// が X・Y 双方向で呼ばれる場合に本関数を 1 回だけ呼び、その結果を
    /// [`Self::build_seismic_load_case_with_period`] へ渡すことで、固有値解析
    /// （部分空間反復）の重複実行を避けられる。
    pub fn seismic_period(&self, mode: AiMode) -> Result<f64, SolveError> {
        match mode {
            AiMode::Approx => {
                let height_m = building_height_mm(self.model) / 1000.0;
                let steel_ratio = steel_height_ratio(self.model);
                Ok(squid_n_load::ai::approx_t(height_m, steel_ratio))
            }
            AiMode::SemiPrecise => {
                // 固有周期は載荷方向に依存しないため、同一 Analysis 上での
                // 2 回目以降（EX→EY 等）はキャッシュを返し固有値解析を省く。
                if let Some(&t) = self.semi_precise_t.get() {
                    return Ok(t);
                }
                let modal = eigen::solve_eigen_with_solver(
                    self.model,
                    &self.dofmap,
                    &self.reducer,
                    1,
                    &*self.solver,
                )?;
                let t = modal.period.first().copied().unwrap_or(0.3);
                let _ = self.semi_precise_t.set(t);
                Ok(t)
            }
        }
    }

    /// 地震静的解析の水平力（Ai 分布）を荷重ケースとして構築して返す。
    /// `seismic_static_with` の載荷部分を切り出したもので、主軸の計算
    /// （主軸の計算（構造力学）の P ベクトル）にも用いる。
    ///
    /// 内部的には [`Self::seismic_period`] で T を求め、
    /// [`Self::build_seismic_load_case_with_period`] へ委譲する（挙動は従来と同一）。
    pub fn build_seismic_load_case(
        &self,
        cfg: SeismicCfg,
    ) -> Result<squid_n_core::model::LoadCase, SolveError> {
        if self.model.stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "階(Story)が定義されていません。地震荷重(Ai分布)には階の定義・地震重量・剛床(ダイアフラム)が必要です。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }
        let t = self.seismic_period(cfg.mode)?;
        self.build_seismic_load_case_with_period(cfg, t)
    }

    /// 設計用固有周期 T [s]（[`Self::seismic_period`] 参照）を受け取り、
    /// Ai 分布・階水平力の算定から荷重ケース構築までを行う。
    ///
    /// X・Y 両方向の地震荷重ケースを構築する際、T は方向によらず同一なので、
    /// 呼び出し側が [`Self::seismic_period`] を 1 回だけ呼んだ結果をここへ
    /// 渡すことで、固有値解析（SemiPrecise モード）の重複実行を避けられる。
    pub fn build_seismic_load_case_with_period(
        &self,
        cfg: SeismicCfg,
        t: f64,
    ) -> Result<squid_n_core::model::LoadCase, SolveError> {
        build_seismic_load_case_from_model(self.model, cfg, t)
    }

    /// 地震静的解析（設計用固有周期 T 指定版）。
    ///
    /// [`Self::seismic_static_with`] と同じ求解を行うが、T を呼び出し側から
    /// 受け取るため固有値解析（SemiPrecise モード）を内部で実行しない。
    /// UI 側で「固有値解析は明示実行のみ・その結果の周期を再利用する」方針を
    /// 実現するための入口（暗黙の固有値解析を避ける）。
    pub fn seismic_static_with_period(
        &self,
        cfg: SeismicCfg,
        t: f64,
    ) -> Result<StaticOnce, SolveError> {
        let lc = self.build_seismic_load_case_with_period(cfg, t)?;

        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }

        let f_free = self.assemble_f_free_from_nodal(&lc.nodal);
        self.solve_and_recover(&f_free)
    }
}

/// 地震静的解析の水平力（Ai 分布）荷重ケースを、モデルと設計用固有周期 T から
/// 構築する（[`Analysis`] を要しないモデル単独版）。
///
/// Ai 分布・階水平力の算定は剛性行列・自由度構成に依存しないため、
/// `Analysis::prepare`（K 組立・拘束縮約・分解）なしで呼び出せる。UI 側の
/// 「EX/EY ケースへの荷重同期」のように、解析準備前に荷重ケースだけを
/// 構築したい場合に用いる（暗黙のフル解析準備を避ける）。
pub fn build_seismic_load_case_from_model(
    model: &Model,
    cfg: SeismicCfg,
    t: f64,
) -> Result<squid_n_core::model::LoadCase, SolveError> {
    let SeismicCfg {
        dir,
        mode,
        z,
        soil,
        c0,
    } = cfg;
    let stories = &model.stories;
    if stories.is_empty() {
        return Err(SolveError::InvalidInput(
                "階(Story)が定義されていません。地震荷重(Ai分布)には階の定義・地震重量・剛床(ダイアフラム)が必要です。解析タブの「階の自動生成」を実行してください。".into(),
            ));
    }

    let tc = squid_n_load::ai::tc_of(soil);
    let rt_val = squid_n_load::ai::rt(t, tc);

    let story_weights: Vec<f64> = stories
        .iter()
        .map(|s| s.seismic_weight.unwrap_or(0.0))
        .collect();

    if story_weights.is_empty() || story_weights.iter().all(|&w| w == 0.0) {
        return Err(SolveError::InvalidInput(
            "階の地震重量(seismic_weight)がすべて 0 です。各階の重量を設定してください。".into(),
        ));
    }

    // PH（塔屋）階・地下階を含む階種別ごとの層せん断力算定式に対応する
    // （seismic_shear_distribution。全階 Normal なら ai_distribution と厳密一致）。
    // 主系統の重量（Qi 用）は ci_override（副剛床の Ci 直接入力）を持つ剛床の
    // 重量を除外する（main_system_weight。§副剛床のCi直接入力）。
    // α・Ai・Ci は「全剛床の場合の Ci」に従うため、副剛床を含む階全体の
    // 重量（ci_weight = seismic_weight）から算定する。
    let specs: Vec<squid_n_load::ai::StorySeismicSpec> = stories
        .iter()
        .map(|s| squid_n_load::ai::StorySeismicSpec {
            weight: main_system_weight(s),
            ci_weight: s.seismic_weight.unwrap_or(0.0),
            level_kind: s.level_kind,
        })
        .collect();
    let ai = squid_n_load::ai::seismic_shear_distribution(&specs, z, rt_val, c0, t);

    // Create a load case from the Ai distribution horizontal forces
    let lc_id = LoadCaseId(1001);
    let dir_vec = match dir {
        SeismicDir::X => [1.0, 0.0, 0.0],
        SeismicDir::Y => [0.0, 1.0, 0.0],
    };

    // Attach Pi forces to master nodes of each story's diaphragms（多剛床の階
    // では重量比で按分し、ci_override を持つ副剛床には指定 Ci による力を
    // 別途作用させる。§1.6・distribute_seismic_forces 参照）。
    let mut lc = squid_n_core::model::LoadCase {
        kind: Default::default(),
        id: lc_id,
        name: format!("seismic_{:?}_{:?}", dir, mode),
        nodal: Vec::new(),
        member: Vec::new(),
    };

    for (i, story) in stories.iter().enumerate() {
        let pi = ai.pi.get(i).copied().unwrap_or(0.0);
        for (master, share) in distribute_seismic_forces(story, pi) {
            if share == 0.0 {
                continue;
            }
            let f = [dir_vec[0] * share, dir_vec[1] * share, 0.0, 0.0, 0.0, 0.0];
            lc.nodal.push(squid_n_core::model::NodalLoad {
                node: master,
                values: f,
            });
        }
    }

    if lc.nodal.is_empty() {
        return Err(SolveError::InvalidInput(
                "地震力を作用させる剛床(ダイアフラム)が階に定義されていません。解析タブの「階の自動生成」を実行してください。".into(),
            ));
    }

    Ok(lc)
}

impl Analysis<'_> {
    /// 各節点の地震静的水平力の大きさ P [N]（`model.nodes` と同順）。
    /// 主軸の計算 `tan2Θ = −Pᵗ(uy+vx)/Pᵗ(vy−ux)` の P ベクトル用
    /// （Ai 分布は加力方向によらないため、X・Y 加力とも同じ分布）。
    pub fn seismic_nodal_force_magnitudes(&self, cfg: SeismicCfg) -> Result<Vec<f64>, SolveError> {
        let lc = self.build_seismic_load_case(cfg)?;
        let mut p = vec![0.0_f64; self.model.nodes.len()];
        for nl in &lc.nodal {
            let i = nl.node.index();
            if i < p.len() {
                p[i] += (nl.values[0].powi(2) + nl.values[1].powi(2)).sqrt();
            }
        }
        Ok(p)
    }
}
