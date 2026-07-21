//! 静的解析のファサード [`Analysis`]。
//!
//! `prepare` で DofMap 構築・全体剛性 K の組立・拘束縮約・分解を一度行い、以降は
//! 分解済み K を再利用して線形静的・荷重組合せ・固有値・時刻歴・地震/風の各解析を
//! 実行する。地震・風の荷重生成は [`seismic`] / [`wind`]、設定型は [`config`]、
//! 解析前のモデル検証は [`precheck`] に分離している。

use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::eigen::{self, ModalResult};
use crate::linear::StaticOnce;
use crate::timehistory::{GroundMotion, NewmarkCfg, ResponseResult};

pub type StaticResult = StaticOnce;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{LoadCombination, Model};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

mod config;
mod precheck;
mod seismic;
mod wind;

pub use config::{AiMode, SeismicCfg, SeismicDir, WindStaticCfg};
pub(crate) use seismic::distribute_pi_over_diaphragms;
pub use seismic::{
    build_seismic_load_case_from_model, building_height_mm, ground_elevation, steel_height_ratio,
};

pub struct Analysis<'m> {
    model: &'m Model,
    dofmap: DofMap,
    reducer: Reducer,
    solver: Box<dyn LinearSolver>,
    n_indep: usize,
    /// SemiPrecise の 1 次固有周期キャッシュ [s]。固有周期は載荷方向に依存しない
    /// ため、同一 `Analysis` 上で X・Y 両方向の地震荷重を構築しても固有値解析は
    /// 1 回で済む（`build_seismic_load_case`）。
    semi_precise_t: std::sync::OnceLock<f64>,
}

/// [`Analysis::prepare`] が構築した DofMap・拘束縮約・分解済みソルバを、
/// モデルへの参照から切り離して保持する入れ物。
///
/// UI 側で「`prepare` → 地震荷重ケース構築 → 荷重ケースだけ更新 →
/// 再度 `prepare`」という流れを取ると、剛性・自由度構成が変わっていないのに
/// 毎回 K を再分解してしまう。荷重ケースの内容（`Model::load_cases`）だけを
/// 差し替える場合は、[`Analysis::into_parts`] で分解結果を取り出しておき、
/// 荷重ケース更新後のモデルに対して [`Analysis::from_parts`] で結合し直せば、
/// 再分解を避けられる。フィールドは非公開で、`Analysis` の内部表現以外の
/// 用途を持たない。
pub struct AnalysisParts {
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
        squid_n_math::parallelism::apply_to_faer();
        model
            .validate()
            .map_err(|e| SolveError::InvalidInput(format!("モデル検証エラー: {:?}", e)))?;
        precheck::precheck_model(model)?;
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
                solver: make_solver(SolverBackend::Auto),
                n_indep: 0,
                semi_precise_t: std::sync::OnceLock::new(),
            });
        }

        let k_free = assemble_global_k(model, &dofmap);
        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        let k_red = reducer.reduce_k(&k_free);

        let mut solver = make_solver(SolverBackend::Auto);
        if n_indep > 0 {
            solver.factorize(&k_red).map_err(|e| match e {
                SolveError::NotPositiveDefinite => {
                    SolveError::InvalidInput(precheck::singular_diagnosis(model))
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
            semi_precise_t: std::sync::OnceLock::new(),
        })
    }

    /// 荷重ケース生成専用の軽量準備。モデル検証・DofMap・拘束縮約の構築までを
    /// 行い、全体剛性 K の組立・分解を省く。
    ///
    /// [`Self::build_seismic_load_case`] は分解済み K を使わない（略算 T は
    /// モデル形状のみ、固有値 T も内部の固有値解析が独自に K・M を組立・分解する）
    /// ため、荷重同期だけが目的ならこちらを使うことで大規模モデルの組立・分解
    /// コスト（`prepare` の支配的コスト）を丸ごと省ける。
    ///
    /// 返した `Analysis` で `linear_static` 等の求解系を呼んではならない
    /// （ソルバが未分解のため求解時にエラーになる）。
    pub fn prepare_load_case_gen(model: &'m Model) -> Result<Self, SolveError> {
        squid_n_math::parallelism::apply_to_faer();
        model
            .validate()
            .map_err(|e| SolveError::InvalidInput(format!("モデル検証エラー: {:?}", e)))?;
        precheck::precheck_model(model)?;
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
                solver: make_solver(SolverBackend::Auto),
                n_indep: 0,
                semi_precise_t: std::sync::OnceLock::new(),
            });
        }

        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        Ok(Self {
            model,
            dofmap,
            reducer,
            solver: make_solver(SolverBackend::Auto),
            n_indep,
            semi_precise_t: std::sync::OnceLock::new(),
        })
    }

    /// DofMap・拘束縮約・分解済みソルバをモデル参照から切り離して取り出す。
    ///
    /// 荷重ケースの内容だけを変えて再度解析したい場合、[`Self::prepare`] を
    /// 呼び直す代わりに、本メソッドで取り出した [`AnalysisParts`] を
    /// [`Self::from_parts`] で新しいモデル参照に結合し直すことで、
    /// K の組立・拘束縮約・分解を再実行せずに済む。
    pub fn into_parts(self) -> AnalysisParts {
        AnalysisParts {
            dofmap: self.dofmap,
            reducer: self.reducer,
            solver: self.solver,
            n_indep: self.n_indep,
        }
    }

    /// [`Self::into_parts`] で取り出した分解済み状態を、モデルへの参照と
    /// 結合して `Analysis` を再構築する。
    ///
    /// **重要**: 節点・要素・拘束・断面/材料の割当（＝剛性と自由度構成）が
    /// `into_parts` を呼んだ時点から変わっていない場合にのみ有効。
    /// 荷重ケース（`Model::load_cases`）の内容の変更のみを想定しており、
    /// `validate`/`precheck` は再実行しない。剛性・自由度構成に影響する変更
    /// （節点座標・要素・拘束・断面/材料割当など）を行った後に呼ぶと、
    /// 古い DofMap・分解済み K のまま解析することになり、誤った結果や
    /// パニックの原因になる。そのような変更を行った場合は必ず
    /// [`Self::prepare`] を呼び直すこと。
    pub fn from_parts(model: &'m Model, parts: AnalysisParts) -> Analysis<'m> {
        Analysis {
            model,
            dofmap: parts.dofmap,
            reducer: parts.reducer,
            solver: parts.solver,
            n_indep: parts.n_indep,
            // SemiPrecise 周期キャッシュは持ち越さない（モデルの荷重内容が
            // 変わっても周期は不変だが、安全側で再算定させる）。
            semi_precise_t: std::sync::OnceLock::new(),
        }
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
    ///
    /// 既に `prepare` で分解済みの `self.solver`（縮約後剛性行列 K_red の分解）を
    /// そのまま再利用する。固有値解析専用の再分解は行わない
    /// （[`eigen::solve_eigen_with_solver`] 参照）。
    pub fn eigen(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        eigen::solve_eigen_with_solver(
            self.model,
            &self.dofmap,
            &self.reducer,
            n_modes,
            &*self.solver,
        )
    }

    /// 複数の荷重ケースを一括で解く（分解済み K を共有）。
    ///
    /// 並列度設定（[`squid_n_math::parallelism`]）が並列（`Auto`/`Threads`）の
    /// 場合は荷重ケース単位に rayon で並列実行する。各ケースの計算はケース間で
    /// 可変状態を共有しない（`&self` のみ）ため実行順に依存せず、結果の順序は
    /// 入力 `lcs` の順で固定される。`Deterministic`（既定）では従来どおり
    /// 逐次実行し、`linear_static` を順に呼んだ場合とビット一致する。
    pub fn linear_static_batch(&self, lcs: &[LoadCaseId]) -> Vec<Result<StaticOnce, SolveError>> {
        self.run_batch(lcs, |lc| self.linear_static(*lc))
    }

    /// 複数の荷重組合せを一括で解く（分解済み K を共有）。
    /// 並列実行の考え方は [`Self::linear_static_batch`] と同じ。
    pub fn linear_combination_batch(
        &self,
        combos: &[LoadCombination],
    ) -> Vec<Result<StaticOnce, SolveError>> {
        self.run_batch(combos, |c| self.linear_combination(c))
    }

    /// バッチ API の共通経路。並列設定時は項目単位に rayon で並列実行する。
    ///
    /// ケース並列（outer）とソルバ内部＝faer の並列（inner）の合計要求が
    /// コア数を超えるとスレッドを奪い合って逆に遅くなるため
    /// （`examples/parallel_bench` で実測）、総枠 `effective_threads()` を
    /// 両者へ自動配分する: ケース数がコア数以上なら inner=1（ケース並列のみ）、
    /// ケース数が少ないときは余りコア（cores/outer）を faer の内部並列へ回す。
    /// 終了後は設定値（`squid_n_math::parallelism`）を faer へ再適用して戻す。
    fn run_batch<T: Sync, R: Send>(
        &self,
        items: &[T],
        f: impl Fn(&T) -> R + Send + Sync,
    ) -> Vec<R> {
        use rayon::prelude::*;
        if squid_n_math::parallelism::is_parallel() && items.len() > 1 {
            let cores = squid_n_math::parallelism::effective_threads();
            let outer = items.len().min(cores);
            let inner = (cores / outer).max(1);
            if inner == 1 {
                faer::set_global_parallelism(faer::Par::Seq);
            } else {
                faer::set_global_parallelism(faer::Par::rayon(inner));
            }
            let out = squid_n_math::parallelism::run(|| items.par_iter().map(f).collect());
            squid_n_math::parallelism::apply_to_faer();
            out
        } else {
            // 1 件のみのバッチは逐次経路（設定どおりの faer 並列で 1 件を解く）
            items.iter().map(f).collect()
        }
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

#[cfg(test)]
mod tests;
