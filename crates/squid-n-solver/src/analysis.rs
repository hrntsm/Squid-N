use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::eigen::{self, ModalResult};
use crate::linear::StaticOnce;
use crate::timehistory::{GroundMotion, NewmarkCfg, ResponseResult};
use std::collections::HashSet;

pub type StaticResult = StaticOnce;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::{LoadCaseId, NodeId};
use squid_n_core::model::{
    DiaphragmDef, LoadCombination, Model, Story, StoryLevelKind, StoryStructure,
};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeismicDir {
    X,
    Y,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiMode {
    Approx,
    SemiPrecise,
}

/// 地震静的解析(Ai分布)の設定。
#[derive(Debug, Clone, Copy)]
pub struct SeismicCfg {
    pub dir: SeismicDir,
    pub mode: AiMode,
    /// 地域係数 Z（令88条）。
    pub z: f64,
    /// 地盤種別（Tc の決定に使用）。
    pub soil: squid_n_load::ai::SoilClass,
    /// 標準せん断力係数 C0（一次設計 0.2、保有 1.0）。
    pub c0: f64,
}

impl Default for SeismicCfg {
    fn default() -> Self {
        Self {
            dir: SeismicDir::X,
            mode: AiMode::SemiPrecise,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
        }
    }
}

/// 風荷重の静的解析（`wind_static`）の設定。
#[derive(Debug, Clone, Copy)]
pub struct WindStaticCfg {
    pub dir: SeismicDir,
    /// 基準風速 V0 [m/s]。
    pub v0: f64,
    /// 地表面粗度区分。
    pub roughness: squid_n_load::wind::TerrainRoughness,
    /// 内圧係数 Cpi（現行の `wind_forces` 実装では風上・風下合算で相殺され
    /// 結果に影響しない。将来の片面評価用に保持する）。
    pub cpi: f64,
    /// パラペット高さ [mm]（既定 0）。マニュアル「建築物の高さと軒の高さとの
    /// 平均」= GLからPH階を除く最上階の床高さ + パラペット高さの半分、に
    /// 対応する。建物高さ H にはこの半分のみを算入するが、見付面積の算定では
    /// 最上層の負担区間上端をパラペット天端（最上階床高さ + `parapet_mm`）まで
    /// 延長する（実壁はパラペット天端まで存在するため）。
    pub parapet_mm: f64,
}

/// 建物の基部レベル（elevation の基準 0）を求める。
///
/// 全構造節点（`generated_masters`＝階自動生成が作る剛床代表節点を除く）の
/// 最小 Z 座標を基部とする（レビュー §1.5・§1.7 が参照する「基部レベル」の
/// 共通定義。剛床代表節点は慣性力重心に置かれる仮想節点であり、実際の
/// 構造高さには寄与しないため除外する）。
fn base_elevation(model: &Model) -> f64 {
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

/// 略算周期 T = h(0.02 + 0.01α) の鉄骨造比 α（令88条・平成12年建設省告示第1793号）。
///
/// 「柱及び梁の大部分が鉄骨造である階の高さの合計の建築物の高さに対する比」。
/// `Story.structure` が `S`（鉄骨造）である階の階高の合計 ÷ 建物全高。
/// RC・SRC は分子に算入しない（マニュアルの定義がS造階のみを対象とするため）。
///
/// 階高 h_i = elevation_i − elevation_{i−1}（最下階は `base_elevation` を
/// elevation_{-1} とみなす）。階が定義されていない、または建物全高が 0 以下の
/// 場合は 0.0 を返す（レビュー §1.5：従来はこの α を常に 0.0 にハードコード
/// していたバグの修正）。
pub fn steel_height_ratio(model: &Model) -> f64 {
    if model.stories.is_empty() {
        return 0.0;
    }
    let base = base_elevation(model);
    let mut prev_elev = base;
    let mut total_h = 0.0;
    let mut steel_h = 0.0;
    for story in &model.stories {
        let h = story.elevation - prev_elev;
        prev_elev = story.elevation;
        total_h += h;
        if matches!(story.structure, StoryStructure::S) {
            steel_h += h;
        }
    }
    if total_h <= 0.0 {
        0.0
    } else {
        (steel_h / total_h).clamp(0.0, 1.0)
    }
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
/// （RESP-D マニュアル「多剛床の設計用せん断力」：水平力を剛床ごとの重量比で
/// 分配する）。
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

/// 階の主系統（Ai 分布）に用いる地震用重量（RESP-D マニュアル「副剛床の Ci を
/// 直接入力した場合」）。`ci_override` を持つ剛床の重量は主系統の Ai 分布から
/// 除外する（主剛床は全剛床の Ci に従うが、副剛床は指定 Ci で別途計算するため）。
/// `ci_override` を持つ剛床が無ければ `story.seismic_weight` をそのまま返す
/// （既存挙動と厳密一致）。
fn main_system_weight(story: &Story) -> f64 {
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
///   副剛床を想定。RESP-D マニュアル「副剛床の Ci を直接入力した場合」）。
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

/// 階の見付け幅（風向直交方向の座標範囲）。その階の構造節点（`node_ids`、
/// `generated_masters` 除く）の座標範囲(max−min)を用いる（マニュアル
/// 「風荷重の計算」の見付面積算定に対応する階別の精緻化）。
///
/// 該当する構造節点が 1 点以下、または座標範囲が 0（全節点が同一座標）の
/// 階は、階別の見付け幅を決定できないため `fallback`（建物全体の構造節点
/// 座標範囲）を用いる。
fn story_wind_width(
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
/// 算定する（RESP-D マニュアル「風荷重の計算」節。`Analysis::wind_static` から
/// solve 抜きでテストできるよう分離）。
///
/// - `H`（建物高さ）= GLからPH階を除く最上階の床高さ + `parapet_mm`/2
///   （マニュアル「建築物の高さと軒の高さとの平均」）。
/// - 最上層の負担区間上端は、`H` とは別に最上階床高さ + `parapet_mm`
///   （パラペット天端）まで延長し、見付面積に算入する（実壁はパラペット
///   天端まで存在するため。`H` にはマニュアルの定義どおり半分のみ算入する）。
/// - 見付け幅は [`story_wind_width`] により階ごとに算定する（フォールバックは
///   建物全体の構造節点座標範囲）。
///
/// `normal_stories` は PH階を除いた一般階・地下階を下から上へ並べたもの
/// （呼び出し側でフィルタ済みであること）。空の場合は呼び出し側で弾く前提。
fn wind_story_geometry(
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

pub struct Analysis<'m> {
    model: &'m Model,
    dofmap: DofMap,
    reducer: Reducer,
    solver: Box<dyn LinearSolver>,
    n_indep: usize,
}

impl<'m> Analysis<'m> {
    /// Build DofMap, assemble global K, apply constraint reduction, and factorize.
    /// After this, `linear_static` and `linear_combination` can be called
    /// multiple times reusing the factorized K.
    ///
    /// 解析前にモデルの静的検証（参照整合・拘束・断面/材料割当・孤立節点）を行い、
    /// 問題があればユーザー向けの日本語診断メッセージ付きでエラーを返す。
    pub fn prepare(model: &'m Model) -> Result<Self, SolveError> {
        faer::set_global_parallelism(faer::Par::Seq);
        model
            .validate()
            .map_err(|e| SolveError::InvalidInput(format!("モデル検証エラー: {:?}", e)))?;
        precheck_model(model)?;
        let dofmap = DofMap::build(model);
        let n_active = dofmap.n_active();

        if n_active == 0 {
            return Ok(Self {
                model,
                dofmap,
                reducer: Reducer {
                    t_rows: vec![],
                    n_indep: 0,
                    n_free: 0,
                },
                solver: make_solver(SolverBackend::DirectSparseCholesky),
                n_indep: 0,
            });
        }

        let k_free = assemble_global_k(model, &dofmap);
        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        let k_red = reducer.reduce_k(&k_free);

        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        if n_indep > 0 {
            solver.factorize(&k_red).map_err(|e| match e {
                SolveError::NotPositiveDefinite => {
                    SolveError::InvalidInput(singular_diagnosis(model))
                }
                other => other,
            })?;
        }

        Ok(Self {
            model,
            dofmap,
            reducer,
            solver,
            n_indep,
        })
    }

    /// 全自由度ゼロの結果（有効自由度なしのモデル用）。
    fn zero_result(&self) -> StaticOnce {
        StaticOnce {
            disp: vec![[0.0; 6]; self.model.nodes.len()],
            member_forces: Vec::new(),
        }
    }

    /// 自由 DOF 空間の荷重ベクトルを縮約 → 解 → 展開し、
    /// 節点変位と部材断面力を復元する（線形静的系の共通経路）。
    fn solve_and_recover(&self, f_free: &[f64]) -> Result<StaticOnce, SolveError> {
        let f_red = self.reducer.reduce_f(f_free);
        let u_indep = self.solver.solve(&f_red)?;
        let u_free = self.reducer.expand_u(&u_indep);
        Ok(StaticOnce {
            disp: self.expand_disp(&u_free),
            member_forces: self.recover_member_forces(&u_free),
        })
    }

    /// 自由 DOF ベクトルを節点 6 成分配列へ展開する。
    fn expand_disp(&self, u_free: &[f64]) -> Vec<[f64; 6]> {
        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; self.model.nodes.len()];
        for (ni, d6) in disp.iter_mut().enumerate() {
            for (d, slot) in d6.iter_mut().enumerate() {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    *slot = u_free[active as usize];
                }
            }
        }
        disp
    }

    /// 自由 DOF ベクトルから全部材の断面力を復元する。
    fn recover_member_forces(
        &self,
        u_free: &[f64],
    ) -> Vec<(
        squid_n_core::ids::ElemId,
        squid_n_element::beam::MemberForces,
    )> {
        let mut member_forces = Vec::new();
        for elem in &self.model.elements {
            let (behavior, _state) = build_behavior(elem, self.model);
            let gdofs = behavior.global_dofs(&self.dofmap);
            let mut u_elem = vec![0.0; gdofs.len()];
            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }
            if let Some(forces) = behavior.recover_forces(&u_elem) {
                member_forces.push((elem.id, forces));
            }
        }
        member_forces
    }

    /// Solve a single load case (back-substitution only, factorized K is reused).
    pub fn linear_static(&self, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        if !self.model.load_cases.iter().any(|c| c.id == lc) {
            return Err(SolveError::InvalidInput(format!(
                "荷重ケース {} が存在しません",
                lc.0
            )));
        }
        let f_free = assemble_global_f(self.model, &self.dofmap, lc);
        self.solve_and_recover(&f_free)
    }

    /// Solve eigenvalue problem (subspace iteration) for n_modes lowest modes.
    pub fn eigen(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, n_modes)
    }

    /// Solve a load combination by assembling the weighted sum of load case
    /// force vectors, then solving with the already factorized K.
    pub fn linear_combination(&self, combo: &LoadCombination) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for (lc_id, factor) in &combo.terms {
            let f_lc = assemble_global_f(self.model, &self.dofmap, *lc_id);
            for (fi, &v) in f_lc.iter().enumerate() {
                f_free[fi] += v * factor;
            }
        }
        self.solve_and_recover(&f_free)
    }

    /// 時刻歴応答解析（Newmark-β / HHT-α、減衰込み）。
    /// 線形専用ラッパ。非線形時刻歴は `timehistory::linear_time_history_analysis`
    /// と同じパターンのフリー関数で実装予定（§4、現在は線形のみ）。
    pub fn time_history(
        &self,
        wave: &GroundMotion,
        newmark: NewmarkCfg,
        damping: Damping,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_time_history_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &newmark,
            &damping,
            &init,
            &init,
            false,
        )
    }

    /// 時刻歴応答解析（HHT-α 法、線形）。α=0 で Newmark-β（平均加速度法）に一致。
    pub fn time_history_hht(
        &self,
        wave: &GroundMotion,
        hht: crate::timehistory::HhtCfg,
        damping: Damping,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_hht_alpha_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &hht,
            &damping,
            &init,
            &init,
            false,
        )
    }

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

    /// 地震静的解析の水平力（Ai 分布）を荷重ケースとして構築して返す。
    /// `seismic_static_with` の載荷部分を切り出したもので、主軸の計算
    /// （RESP-D 計算編03「応力解析 §主軸の計算」の P ベクトル）にも用いる。
    pub fn build_seismic_load_case(
        &self,
        cfg: SeismicCfg,
    ) -> Result<squid_n_core::model::LoadCase, SolveError> {
        let SeismicCfg {
            dir,
            mode,
            z,
            soil,
            c0,
        } = cfg;
        let stories = &self.model.stories;
        if stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "階(Story)が定義されていません。地震荷重(Ai分布)には階の定義・地震重量・剛床(ダイアフラム)が必要です。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }

        let (t, _) = match mode {
            AiMode::Approx => {
                let height_m = stories.last().map(|s| s.elevation).unwrap_or(0.0) / 1000.0;
                let steel_ratio = steel_height_ratio(self.model);
                (squid_n_load::ai::approx_t(height_m, steel_ratio), 0)
            }
            AiMode::SemiPrecise => {
                let modal = eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, 1)?;
                let t = modal.period.first().copied().unwrap_or(0.3);
                (t, 0)
            }
        };

        let tc = squid_n_load::ai::tc_of(soil);
        let rt_val = squid_n_load::ai::rt(t, tc);

        let story_weights: Vec<f64> = stories
            .iter()
            .map(|s| s.seismic_weight.unwrap_or(0.0))
            .collect();

        if story_weights.is_empty() || story_weights.iter().all(|&w| w == 0.0) {
            return Err(SolveError::InvalidInput(
                "階の地震重量(seismic_weight)がすべて 0 です。各階の重量を設定してください。"
                    .into(),
            ));
        }

        // PH（塔屋）階・地下階を含む階種別ごとの層せん断力算定式に対応する
        // （seismic_shear_distribution。全階 Normal なら ai_distribution と厳密一致）。
        // 主系統の重量は ci_override（副剛床の Ci 直接入力）を持つ剛床の重量を
        // 除外する（main_system_weight。§副剛床のCi直接入力）。
        let specs: Vec<squid_n_load::ai::StorySeismicSpec> = stories
            .iter()
            .map(|s| squid_n_load::ai::StorySeismicSpec {
                weight: main_system_weight(s),
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

    /// 風荷重の静的解析（RESP-D マニュアル「風荷重の計算」節）。
    ///
    /// - 建物高さ H・各層の負担区間・見付け幅は [`wind_story_geometry`] を
    ///   参照（パラペット割増し・階別見付け幅の詳細はそちらのドキュメント）。
    /// - PH階は建物高さの算定・風荷重の負担層のいずれからも除外する
    ///   （PH階への風荷重接続は未対応。残課題）。
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
            .filter(|s| !matches!(s.level_kind, StoryLevelKind::Penthouse { .. }))
            .collect();
        if normal_stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "風荷重の対象となる階(PH階を除く一般階・地下階)が定義されていません。".into(),
            ));
        }

        let base = base_elevation(model);
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

    /// LoadCase の節点荷重リストから自由 DOF 空間の荷重ベクトルを組み立てる
    /// （地震荷重・風荷重など静的荷重ケースの共通処理）。
    fn assemble_f_free_from_nodal(&self, nodal: &[squid_n_core::model::NodalLoad]) -> Vec<f64> {
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for nodal_load in nodal {
            let ni = nodal_load.node.index();
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    f_free[active as usize] += nodal_load.values[d];
                }
            }
        }
        f_free
    }
}

/// 解析前のモデル静的検証。よくあるモデリングミスを特異行列エラーの前に検出し、
/// 「何をすれば直るか」を含むメッセージで返す。
fn precheck_model(model: &Model) -> Result<(), SolveError> {
    use squid_n_core::model::ElementKind;

    if model.nodes.is_empty() {
        return Err(SolveError::InvalidInput(
            "節点がありません。モデルタブで節点を追加してください。".into(),
        ));
    }
    if model.elements.is_empty() {
        return Err(SolveError::InvalidInput(
            "部材がありません。モデルタブで部材を追加してください。".into(),
        ));
    }
    if !model.nodes.iter().any(|n| n.restraint.0 != 0) {
        return Err(SolveError::InvalidInput(
            "拘束(支点)が 1 つもありません。境界条件タブで支点を設定してください。".into(),
        ));
    }

    // 梁要素の断面・材料未割当
    let missing: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam) && (e.section.is_none() || e.material.is_none())
        })
        .map(|e| e.id.0)
        .collect();
    if !missing.is_empty() {
        let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
        let more = if missing.len() > 5 {
            format!(" 他{}件", missing.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "断面または材料が未割当の部材があります: ID {}{}。部材タブで割り当ててください。",
            head.join(", "),
            more
        )));
    }

    // 孤立節点（要素・拘束・剛床から参照されず、完全固定でもない）
    // → 剛性ゼロの自由 DOF となり特異行列の典型原因
    let mut referenced = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            referenced[n.index()] = true;
        }
    }
    for c in &model.constraints {
        use squid_n_core::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, slaves, .. }
            | Constraint::RigidLink { master, slaves, .. } => {
                referenced[master.index()] = true;
                for s in slaves {
                    referenced[s.index()] = true;
                }
            }
            Constraint::Mpc { master, terms } => {
                referenced[master.index()] = true;
                for (n, _, _) in terms {
                    referenced[n.index()] = true;
                }
            }
        }
    }
    for story in &model.stories {
        for d in &story.diaphragms {
            referenced[d.master.index()] = true;
            for s in &d.slaves {
                referenced[s.index()] = true;
            }
        }
    }
    let isolated: Vec<u32> = model
        .nodes
        .iter()
        .filter(|n| !referenced[n.id.index()] && n.restraint != squid_n_core::dof::Dof6Mask::FIXED)
        .map(|n| n.id.0)
        .collect();
    if !isolated.is_empty() {
        let head: Vec<String> = isolated.iter().take(5).map(|id| id.to_string()).collect();
        let more = if isolated.len() > 5 {
            format!(" 他{}件", isolated.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "どの部材にも接続されていない節点があります: ID {}{}。削除するか完全固定にしてください(剛性ゼロの自由度は解析できません)。",
            head.join(", "),
            more
        )));
    }

    Ok(())
}

/// 剛性行列の分解に失敗した（特異・非正定値）ときの診断メッセージ。
fn singular_diagnosis(model: &Model) -> String {
    let n_restrained = model.nodes.iter().filter(|n| n.restraint.0 != 0).count();
    format!(
        "剛性行列が特異(非正定値)です。構造が機構(不安定)になっている可能性があります。\
         考えられる原因: (1) 拘束が不足している(現在 {} 節点に拘束あり)、\
         (2) ピン接合が連続し回転が拘束されない部材がある、\
         (3) 断面性能(A・I)が 0 の断面がある。",
        n_restrained
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        Constraint, DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase,
        LocalAxis, Material, MemberLoad, MemberLoadKind, NodalLoad, Node, Section,
    };

    fn make_cantilever_model() -> Model {
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
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "beam".into(),
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
            load_cases: vec![
                LoadCase {
                    kind: Default::default(),
                    id: LoadCaseId(1),
                    name: "axial".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                },
                LoadCase {
                    kind: Default::default(),
                    id: LoadCaseId(2),
                    name: "shear".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 500.0, 0.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                },
            ],
            combinations: vec![LoadCombination {
                name: "combo1".into(),
                terms: vec![(LoadCaseId(1), 1.2), (LoadCaseId(2), 1.5)],
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_prepare_and_single_case() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let result = analysis.linear_static(LoadCaseId(1)).unwrap();
        let ux = result.disp[1][0];
        let expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        assert!(
            (ux - expected).abs() < 1e-6,
            "ux={} expected={}",
            ux,
            expected
        );
    }

    #[test]
    fn test_two_cases_one_factorization() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
        let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
        let ux = r1.disp[1][0];
        let uy = r2.disp[1][1];
        let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        let l = 1000.0_f64;
        let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
        // Timoshenko beam includes shear deflection ≈ 0.1% — use relaxed tolerance
        assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
        assert!(
            (uy - uy_expected).abs() < 20.0,
            "uy={} approx={}",
            uy,
            uy_expected
        );
    }

    #[test]
    fn test_load_combination() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let combo = &model.combinations[0];
        let result = analysis.linear_combination(combo).unwrap();
        let ux = result.disp[1][0];
        let uy = result.disp[1][1];
        let ux_expected = 1.2 * (1000.0 * 1000.0 / (20000.0 * 100.0));
        let l = 1000.0_f64;
        let uy_expected = 1.5 * (500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33));
        assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
        // Timoshenko shear adds slight deflection — relaxed tolerance
        assert!(
            (uy - uy_expected).abs() < 20.0,
            "uy={} approx={}",
            uy,
            uy_expected
        );
    }

    #[test]
    fn test_prepare_empty_model_gives_diagnostic() {
        let model = Model::default();
        let err = Analysis::prepare(&model).err().unwrap();
        assert!(matches!(err, SolveError::InvalidInput(_)), "{:?}", err);
    }

    #[test]
    fn test_prepare_no_restraint_gives_diagnostic() {
        let mut model = make_cantilever_model();
        for n in &mut model.nodes {
            n.restraint = Dof6Mask::FREE;
        }
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("拘束"), "{}", msg);
    }

    #[test]
    fn test_prepare_missing_section_gives_diagnostic() {
        let mut model = make_cantilever_model();
        model.elements[0].section = None;
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("未割当"), "{}", msg);
    }

    #[test]
    fn test_prepare_isolated_node_gives_diagnostic() {
        let mut model = make_cantilever_model();
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [0.0, 5000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("接続されていない節点"), "{}", msg);
    }

    #[test]
    fn test_linear_static_unknown_load_case_is_error() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let err = analysis.linear_static(LoadCaseId(99)).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("荷重ケース"), "{}", msg);
    }

    #[test]
    fn test_seismic_without_stories_is_error() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let err = analysis
            .seismic_static(SeismicDir::X, AiMode::Approx)
            .err()
            .unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("階"), "{}", msg);
    }

    #[test]
    fn test_bernoulli_strict_1e9() {
        // Bernoulli beam: very large shear area → negligible shear deformation.
        // Axial: u = PL/EA, Bending: w = PL³/3EI — strict 1e-9 match.
        let mut model = make_cantilever_model();
        model.sections[0].as_y = 1e12;
        model.sections[0].as_z = 1e12;
        let analysis = Analysis::prepare(&model).unwrap();
        let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
        let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
        let ux = r1.disp[1][0];
        let uy = r2.disp[1][1];
        let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        let l = 1000.0_f64;
        let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
        let ux_rel = (ux - ux_expected).abs() / ux_expected.abs();
        let uy_rel = (uy - uy_expected).abs() / uy_expected.abs();
        assert!(ux_rel < 1e-9, "ux rel err={}", ux_rel);
        assert!(uy_rel < 1e-4, "uy rel err={}", uy_rel);
    }

    // ---- §1.5 略算周期の鉄骨造比 α ----

    /// 3層等階高（各1000mm、基部Z=0）で、指定した各階の `structure` から
    /// `steel_height_ratio` を計算するテスト用モデル。
    fn make_story_ratio_model(structures: &[StoryStructure]) -> Model {
        let mut nodes = vec![Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        }];
        let mut stories = Vec::new();
        for (i, s) in structures.iter().enumerate() {
            let elev = (i as f64 + 1.0) * 1000.0;
            let nid = NodeId((i + 1) as u32);
            nodes.push(Node {
                id: nid,
                coord: [0.0, 0.0, elev],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(i as u32)),
            });
            stories.push(Story {
                id: StoryId(i as u32),
                name: format!("F{}", i + 1),
                elevation: elev,
                node_ids: vec![nid],
                diaphragms: Vec::new(),
                seismic_weight: Some(1000.0),
                structure: *s,
                level_kind: StoryLevelKind::Normal,
            });
        }
        Model {
            nodes,
            stories,
            ..Default::default()
        }
    }

    #[test]
    fn test_steel_height_ratio_bottom_story_s_gives_one_third() {
        let model =
            make_story_ratio_model(&[StoryStructure::S, StoryStructure::Rc, StoryStructure::Rc]);
        let alpha = steel_height_ratio(&model);
        assert!((alpha - 1.0 / 3.0).abs() < 1e-9, "alpha={}", alpha);
    }

    #[test]
    fn test_steel_height_ratio_all_rc_is_zero() {
        let model = make_story_ratio_model(&[StoryStructure::Rc; 3]);
        assert_eq!(steel_height_ratio(&model), 0.0);
    }

    #[test]
    fn test_steel_height_ratio_all_s_is_one() {
        let model = make_story_ratio_model(&[StoryStructure::S; 3]);
        let alpha = steel_height_ratio(&model);
        assert!((alpha - 1.0).abs() < 1e-9, "alpha={}", alpha);
    }

    #[test]
    fn test_steel_height_ratio_no_stories_is_zero() {
        let model = Model::default();
        assert_eq!(steel_height_ratio(&model), 0.0);
    }

    // ---- §1.6 多剛床のPi重複載荷 ----

    fn make_diaphragm_story(diaphragms: Vec<DiaphragmDef>) -> Story {
        Story {
            id: StoryId(0),
            name: "F1".into(),
            elevation: 1000.0,
            node_ids: Vec::new(),
            diaphragms,
            seismic_weight: Some(400.0),
            structure: StoryStructure::Rc,
            level_kind: StoryLevelKind::Normal,
        }
    }

    #[test]
    fn test_distribute_pi_single_diaphragm_gets_full_pi() {
        let story = make_diaphragm_story(vec![DiaphragmDef {
            ci_override: None,
            master: NodeId(10),
            slaves: vec![],
            rigid: true,
            weight: None,
        }]);
        let shares = distribute_pi_over_diaphragms(&story, 40.0);
        assert_eq!(shares, vec![(NodeId(10), 40.0)]);
    }

    #[test]
    fn test_distribute_pi_weight_ratio_3_to_1() {
        let story = make_diaphragm_story(vec![
            DiaphragmDef {
                ci_override: None,
                master: NodeId(10),
                slaves: vec![],
                rigid: true,
                weight: Some(300.0),
            },
            DiaphragmDef {
                ci_override: None,
                master: NodeId(11),
                slaves: vec![],
                rigid: true,
                weight: Some(100.0),
            },
        ]);
        let pi = 40.0;
        let shares = distribute_pi_over_diaphragms(&story, pi);
        let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
        let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
        assert!((s10 - 30.0).abs() < 1e-9, "s10={}", s10);
        assert!((s11 - 10.0).abs() < 1e-9, "s11={}", s11);
        // 合計は階の Pi に一致する（重複載荷しない）。
        let total: f64 = shares.iter().map(|(_, v)| v).sum();
        assert!((total - pi).abs() < 1e-9, "total={}", total);
    }

    #[test]
    fn test_distribute_pi_equal_split_when_no_weight() {
        let story = make_diaphragm_story(vec![
            DiaphragmDef {
                ci_override: None,
                master: NodeId(10),
                slaves: vec![],
                rigid: true,
                weight: None,
            },
            DiaphragmDef {
                ci_override: None,
                master: NodeId(11),
                slaves: vec![],
                rigid: true,
                weight: None,
            },
        ]);
        let pi = 40.0;
        let shares = distribute_pi_over_diaphragms(&story, pi);
        for (_, v) in &shares {
            assert!((*v - 20.0).abs() < 1e-9, "share={}", v);
        }
        let total: f64 = shares.iter().map(|(_, v)| v).sum();
        assert!((total - pi).abs() < 1e-9, "total={}", total);
    }

    // ---- §4 風荷重の静的解析接続 ----

    /// 2層×1スパンの平面ラーメン（squid-n-load::story_gen のテスト固定物と同形）。
    /// X方向にスパン6000mm、全節点 Y=0（平面フレーム）。柱4本・梁2本。
    fn two_story_wind_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
            [0.0, 0.0, 7000.0],
            [6000.0, 0.0, 7000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
        for (i, (a, b)) in conn.iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![NodalLoad {
                node: NodeId(4),
                values: [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
            }],
            member: vec![MemberLoad {
                elem: ElemId(4),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 6000.0,
                    w1: 10.0,
                    w2: 10.0,
                },
            }],
        });
        model
    }

    /// `squid_n_load::story_gen::StoryGenResult` をテスト用モデルへ適用する
    /// （squid-n-app の `ApplyStories` EditCommand と同じ手順を直接実行する）。
    fn apply_story_gen(model: &mut Model, gen: squid_n_load::story_gen::StoryGenResult) {
        model.stories = gen.stories;
        for (node, st) in model.nodes.iter_mut().zip(gen.node_story.iter()) {
            node.story = *st;
        }
        model
            .constraints
            .retain(|c| !matches!(c, Constraint::RigidDiaphragm { .. }));
        model.constraints.extend(gen.constraints);
        for rn in gen.rep_nodes {
            let idx = rn.id.index();
            if idx < model.nodes.len() {
                model.nodes[idx] = rn;
            } else {
                model.nodes.push(rn);
            }
        }
        model.generated_masters = gen.generated_masters;
    }

    #[test]
    fn test_wind_static_runs_and_reactions_balance_applied_force() {
        let mut model = two_story_wind_model();
        let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        apply_story_gen(&mut model, gen);

        let analysis = Analysis::prepare(&model).unwrap();
        // 平面フレームは全節点 Y=0 のため、Y方向の風(dir=Y)なら見付け幅は
        // X方向範囲(6000mm)から求まる（dir=X だと Y範囲=0 でエラーになる）。
        let cfg = WindStaticCfg {
            dir: SeismicDir::Y,
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            cpi: 0.0,
            parapet_mm: 0.0,
        };
        let result = analysis.wind_static(cfg).unwrap();

        // wind_static と同じ幾何（H=7000mm、幅=6000mm）で独立に風荷重を再計算し、
        // 全層合計の水平力を求める。
        let wcfg = squid_n_load::wind::WindCfg {
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let wind_stories = vec![
            squid_n_load::wind::WindStory {
                z_bottom: 0.0,
                z_top: 5250.0,
                width: 6000.0,
            },
            squid_n_load::wind::WindStory {
                z_bottom: 5250.0,
                z_top: 7000.0,
                width: 6000.0,
            },
        ];
        let dist = squid_n_load::wind::wind_forces(7000.0, &wind_stories, &wcfg);
        let total_force: f64 = dist.force.iter().sum();
        assert!(total_force > 0.0, "total_force={}", total_force);

        // 基部の反力(Y方向)は、基部節点(0,1)に接続する柱要素(ElemId 0,1)の
        // i端(xi=0)局所力から求める。この2柱は鉛直（ref_vector=[0,0,1]と部材軸が
        // 平行）なので LocalFrame::from_nodes のフォールバックにより
        // 局所軸 (ex,ey,ez) = (global Z, global X, global Y) となり、
        // 局所 qz（[2]成分）がそのまま global Y 方向の力に一致する。
        let reaction_y: f64 = result
            .member_forces
            .iter()
            .filter(|(id, _)| id.0 == 0 || id.0 == 1)
            .map(|(_, mf)| {
                mf.at
                    .iter()
                    .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
                    .unwrap()
                    .1[2]
            })
            .sum();

        // 全体の水平釣合い: 反力合計の大きさ = 作用させた風荷重合計(Σforce_i)。
        assert!(
            (reaction_y.abs() - total_force.abs()).abs() < total_force.abs() * 1e-6 + 1e-6,
            "reaction_y={} total_force={}",
            reaction_y,
            total_force
        );
    }

    #[test]
    fn test_wind_static_without_stories_is_error() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let cfg = WindStaticCfg {
            dir: SeismicDir::X,
            v0: 30.0,
            roughness: squid_n_load::wind::TerrainRoughness::II,
            cpi: 0.0,
            parapet_mm: 0.0,
        };
        let err = analysis.wind_static(cfg).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("階"), "{}", msg);
    }

    // ---- §4 追補: 副剛床のCi直接入力・パラペット・階別見付け幅 ----

    #[test]
    fn test_main_system_weight_excludes_ci_override_diaphragm() {
        let story = make_diaphragm_story(vec![
            DiaphragmDef {
                ci_override: None,
                master: NodeId(10),
                slaves: vec![],
                rigid: true,
                weight: Some(300.0),
            },
            DiaphragmDef {
                ci_override: Some(0.3),
                master: NodeId(11),
                slaves: vec![],
                rigid: true,
                weight: Some(100.0),
            },
        ]);
        // make_diaphragm_story は seismic_weight=400.0 固定（主300+副100）。
        // 主系統重量は ci_override を持つ副剛床の重量(100)を除いた 300 になる。
        let w = main_system_weight(&story);
        assert!((w - 300.0).abs() < 1e-9, "main_system_weight={}", w);
    }

    #[test]
    fn test_distribute_seismic_forces_ci_override_adds_separate_force() {
        let story = make_diaphragm_story(vec![
            DiaphragmDef {
                ci_override: None,
                master: NodeId(10),
                slaves: vec![],
                rigid: true,
                weight: Some(300.0),
            },
            DiaphragmDef {
                ci_override: Some(0.3),
                master: NodeId(11),
                slaves: vec![],
                rigid: true,
                weight: Some(100.0),
            },
        ]);
        // 主系統(重量300ベースで別途算定済み)の Pi として 60.0 を渡す。
        // 主剛床(唯一の ci_override 無し剛床)が全量を受け、副剛床には
        // 0.3×100=30 が別途載る。
        let pi = 60.0;
        let shares = distribute_seismic_forces(&story, pi);
        let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
        let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
        assert!((s10 - 60.0).abs() < 1e-9, "s10={}", s10);
        assert!((s11 - 30.0).abs() < 1e-9, "s11={}", s11);
    }

    #[test]
    fn test_distribute_seismic_forces_matches_pi_distribution_without_ci_override() {
        // 全剛床が ci_override 無しなら distribute_pi_over_diaphragms と厳密一致。
        let story = make_diaphragm_story(vec![
            DiaphragmDef {
                ci_override: None,
                master: NodeId(10),
                slaves: vec![],
                rigid: true,
                weight: Some(300.0),
            },
            DiaphragmDef {
                ci_override: None,
                master: NodeId(11),
                slaves: vec![],
                rigid: true,
                weight: Some(100.0),
            },
        ]);
        let pi = 40.0;
        let expected = distribute_pi_over_diaphragms(&story, pi);
        let actual = distribute_seismic_forces(&story, pi);
        assert_eq!(expected, actual);
    }

    /// 単純な `Model`（構造節点のみ、部材・拘束なし）を組み立てるテスト用ヘルパ。
    fn make_node_only_model(coords: &[[f64; 3]]) -> Model {
        let nodes = coords
            .iter()
            .enumerate()
            .map(|(i, c)| Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            })
            .collect();
        Model {
            nodes,
            ..Default::default()
        }
    }

    #[test]
    fn test_story_wind_width_uses_story_node_range() {
        let model = make_node_only_model(&[[0.0, 0.0, 0.0], [6000.0, 0.0, 0.0]]);
        let story = make_diaphragm_story(vec![]);
        let mut story = story;
        story.node_ids = vec![NodeId(0), NodeId(1)];
        let excluded = HashSet::new();
        let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
        assert!((w - 6000.0).abs() < 1e-9, "w={}", w);
    }

    #[test]
    fn test_story_wind_width_fallback_when_single_node() {
        let model = make_node_only_model(&[[0.0, 0.0, 0.0], [6000.0, 0.0, 0.0]]);
        let mut story = make_diaphragm_story(vec![]);
        story.node_ids = vec![NodeId(0)];
        let excluded = HashSet::new();
        let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
        assert!((w - 999.0).abs() < 1e-9, "w={}", w);
    }

    #[test]
    fn test_story_wind_width_fallback_when_zero_range() {
        let model = make_node_only_model(&[[3000.0, 0.0, 0.0], [3000.0, 5000.0, 0.0]]);
        let mut story = make_diaphragm_story(vec![]);
        story.node_ids = vec![NodeId(0), NodeId(1)];
        let excluded = HashSet::new();
        // 両節点とも X=3000 なので axis=0(X) の範囲は 0 → フォールバック。
        let w = story_wind_width(&story, &model, 0, &excluded, 999.0);
        assert!((w - 999.0).abs() < 1e-9, "w={}", w);
    }

    #[test]
    fn test_wind_story_geometry_parapet_increases_h_and_extends_top_interval() {
        let mut model = two_story_wind_model();
        let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        apply_story_gen(&mut model, gen);
        let normal_stories: Vec<&Story> = model.stories.iter().collect();
        let base = base_elevation(&model);
        let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();

        let (h0, ws0) =
            wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 0.0).unwrap();
        let (h1, ws1) =
            wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 1000.0).unwrap();

        // パラペット無し: H=7000mm、最上層区間上端=7000mm(=H)。
        assert!((h0 - 7000.0).abs() < 1e-9, "h0={}", h0);
        assert!(
            (ws0.last().unwrap().z_top - 7000.0).abs() < 1e-9,
            "z_top0={}",
            ws0.last().unwrap().z_top
        );

        // パラペット1000mm: H=7000+500=7500mm、最上層区間上端=7000+1000=8000mm
        // （H には半分のみ、区間上端にはパラペット天端まで全量算入）。
        assert!((h1 - 7500.0).abs() < 1e-9, "h1={}", h1);
        assert!(
            (ws1.last().unwrap().z_top - 8000.0).abs() < 1e-9,
            "z_top1={}",
            ws1.last().unwrap().z_top
        );
    }

    #[test]
    fn test_wind_story_geometry_setback_gives_narrower_upper_story_width() {
        // 1階(0〜3500mm): X=0〜6000(幅6000)、2階(3500〜7000mm): セットバックで
        // X=1000〜4000(幅3000)のみ柱がある平面フレームを組み立てる。
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],       // 0: 基部
            [6000.0, 0.0, 0.0],    // 1: 基部
            [0.0, 0.0, 3500.0],    // 2: 1階床（幅6000端）
            [6000.0, 0.0, 3500.0], // 3: 1階床（幅6000端）
            [1000.0, 0.0, 3500.0], // 4: 2階柱脚（セットバック開始）
            [4000.0, 0.0, 3500.0], // 5: 2階柱脚（セットバック開始）
            [1000.0, 0.0, 7000.0], // 6: 2階床（幅3000端）
            [4000.0, 0.0, 7000.0], // 7: 2階床（幅3000端）
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        // 柱: 0-2, 1-3（1階）、4-6, 5-7（2階、セットバック柱脚）。
        // 梁: 2-3（1階床）、6-7（2階床）、4-2, 5-3（セットバックの水平つなぎ、
        // 2階柱脚(4,5)を1階床(2,3)へ接続して構造を連続させる）。
        let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (4, 6), (5, 7), (2, 3), (6, 7)];
        for (i, (a, b)) in conn.iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        // 2階柱脚(4,5)を1階床(2,3)へ接続する水平つなぎ梁。
        model.elements.push(ElementData {
            id: ElemId(6),
            kind: ElementKind::Beam,
            nodes: [NodeId(2), NodeId(4)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        model.elements.push(ElementData {
            id: ElemId(7),
            kind: ElementKind::Beam,
            nodes: [NodeId(3), NodeId(5)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });

        let gen = squid_n_load::story_gen::generate_stories(&model, None).unwrap();
        apply_story_gen(&mut model, gen);
        assert_eq!(model.stories.len(), 2, "stories={:?}", model.stories.len());

        let normal_stories: Vec<&Story> = model.stories.iter().collect();
        let base = base_elevation(&model);
        let excluded: HashSet<NodeId> = model.generated_masters.iter().copied().collect();
        // dir=Y の風 → axis=0(X方向の座標範囲)。
        let (_h, wind_stories) =
            wind_story_geometry(&model, &normal_stories, base, 0, &excluded, 0.0).unwrap();

        assert_eq!(wind_stories.len(), 2);
        assert!(
            (wind_stories[0].width - 6000.0).abs() < 1e-6,
            "story0 width={}",
            wind_stories[0].width
        );
        assert!(
            (wind_stories[1].width - 3000.0).abs() < 1e-6,
            "story1 width={}",
            wind_stories[1].width
        );
        assert!(
            wind_stories[1].width < wind_stories[0].width,
            "上層(セットバック)の見付け幅は下層より小さいはず"
        );
    }

    #[test]
    fn test_wind_static_excludes_penthouse_story_from_height_and_load() {
        let mut model = two_story_wind_model();
        let gen = squid_n_load::story_gen::generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        apply_story_gen(&mut model, gen);
        // 最上階(7000mm)をPH階に変更 → 建物高さ・負担層は1階(3500mm)のみになる。
        model.stories[1].level_kind = StoryLevelKind::Penthouse { k: 0.6 };

        let analysis = Analysis::prepare(&model).unwrap();
        let cfg = WindStaticCfg {
            dir: SeismicDir::Y,
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            cpi: 0.0,
            parapet_mm: 0.0,
        };
        let result = analysis.wind_static(cfg).unwrap();

        // H=3500mm・幅6000mmの1層構成として独立に風荷重を再計算する。
        let wcfg = squid_n_load::wind::WindCfg {
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let wind_stories = vec![squid_n_load::wind::WindStory {
            z_bottom: 0.0,
            z_top: 3500.0,
            width: 6000.0,
        }];
        let dist = squid_n_load::wind::wind_forces(3500.0, &wind_stories, &wcfg);
        let total_force: f64 = dist.force.iter().sum();
        assert!(total_force > 0.0, "total_force={}", total_force);

        let reaction_y: f64 = result
            .member_forces
            .iter()
            .filter(|(id, _)| id.0 == 0 || id.0 == 1)
            .map(|(_, mf)| {
                mf.at
                    .iter()
                    .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
                    .unwrap()
                    .1[2]
            })
            .sum();

        assert!(
            (reaction_y.abs() - total_force.abs()).abs() < total_force.abs() * 1e-6 + 1e-6,
            "reaction_y={} total_force={}",
            reaction_y,
            total_force
        );
    }
}
