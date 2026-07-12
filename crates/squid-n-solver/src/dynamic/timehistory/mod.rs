//! 時刻歴応答解析（P6 §2〜§4）。
//!
//! Newmark-β 法（平均加速度・線形加速度）による線形時刻歴応答解析。
//! 基盤一様加振（相対変位形式）: `M·ü + C·u̇ + K·u = −M·r·ẍg(t)`。
//! 非線形時刻歴（各ステップ Newton 反復）は pushover.rs と同じ
//! commit/rollback 基盤を使う（§4、将来拡張）。

use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::pushover::{assemble_k, compute_f_int};
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, LocalVec, MassOption};
use squid_n_element::factory::build_nonlinear_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::{sparse_matvec, weighted_sum_csc};

/// Newmark-β 法のパラメータ（§2）。
pub struct NewmarkCfg {
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
}

impl NewmarkCfg {
    /// 平均加速度法（無条件安定）。dt は後で設定する。
    pub fn average_accel() -> Self {
        Self {
            beta: 0.25,
            gamma: 0.5,
            dt: 0.0,
        }
    }
    /// 線形加速度法（条件付安定）。dt は後で設定する。
    pub fn linear_accel() -> Self {
        Self {
            beta: 1.0 / 6.0,
            gamma: 0.5,
            dt: 0.0,
        }
    }
}

/// HHT-α 法のパラメータ（§2）。α ∈ [−1/3, 0]、既定 −0.1。
pub struct HhtCfg {
    pub alpha: f64,
    pub dt: f64,
}

impl HhtCfg {
    pub fn new(dt: f64) -> Self {
        Self { alpha: -0.1, dt }
    }
}

/// 地動加速度入力（基盤一様加振）。水平1〜2方向（R8）。
/// `dt` はサンプリング間隔。`accel_x`/`accel_y` は同長さの時系列。
pub struct GroundMotion {
    pub dt: f64,
    pub accel_x: Vec<f64>,
    pub accel_y: Option<Vec<f64>>,
}

/// 時刻歴応答解析の結果（設計書 §10.5）。
/// 時系列の全量は結果I/O（§6）へストリーミングし、メモリに全保持しない。
/// 例外として UI 描画用の代表応答（1 節点変位・ベースシア・最上階変形角）のみ
/// `history` にステップごとの値を保持する。
pub struct ResponseResult {
    pub time: Vec<f64>,
    pub peak_disp: Vec<[f64; 6]>,
    pub story_drift_angle: Vec<f64>,
    pub cumulative_ductility: Vec<f64>,
    pub history: ResponseHistory,
}

/// UI 描画用の代表応答時刻歴（`time` と同じ長さ）。
/// 記録方向は入力加速度の絶対値和（Σ|ẍg|）が大きい方向を解析開始時に自動選択する
/// （`choose_record_dir_y` 参照）。X・Y いずれの加振でも代表応答がゼロにならない。
#[derive(Clone, Debug, Default)]
pub struct ResponseHistory {
    /// 記録節点（最も標高が高い、記録方向の自由度を持つ節点）。
    pub node: Option<squid_n_core::ids::NodeId>,
    /// 記録方向が Y なら true（X なら false）。
    pub record_dir_y: bool,
    /// 記録節点の記録方向相対変位 [mm]。
    pub node_disp: Vec<f64>,
    /// ベースシア(記録方向) [N]（全慣性力の合計、符号付き）。
    pub base_shear: Vec<f64>,
    /// 最上階の層間変形角 [rad]（符号付き。階が未定義なら 0）。
    pub top_drift_angle: Vec<f64>,
}

/// 記録方向を自動選択する: `accel_y` が Some かつ Σ|accel_y| > Σ|accel_x| なら Y、
/// そうでなければ X（従来互換）。
fn choose_record_dir_y(wave: &GroundMotion) -> bool {
    let sum_x: f64 = wave.accel_x.iter().map(|v| v.abs()).sum();
    let sum_y: f64 = wave
        .accel_y
        .as_ref()
        .map(|a| a.iter().map(|v| v.abs()).sum())
        .unwrap_or(0.0);
    wave.accel_y.is_some() && sum_y > sum_x
}

/// 記録節点を選ぶ: 記録方向（`dir_idx`: 0=X, 1=Y）が自由な節点のうち
/// 最も標高(Z)が高いもの。
fn pick_record_node(
    model: &Model,
    dofmap: &DofMap,
    dir_idx: usize,
) -> Option<squid_n_core::ids::NodeId> {
    model
        .nodes
        .iter()
        .filter(|n| {
            dofmap
                .active(n.id.index() * DOF_PER_NODE + dir_idx)
                .is_some()
        })
        .max_by(|a, b| {
            a.coord[2]
                .partial_cmp(&b.coord[2])
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|n| n.id)
}

/// 最上階の現在の層間変形角（符号付き、記録方向 `dir_idx`）。階が未定義なら 0。
fn current_top_drift(model: &Model, dofmap: &DofMap, u_free: &[f64], dir_idx: usize) -> f64 {
    let Some(si) = model.stories.len().checked_sub(1) else {
        return 0.0;
    };
    let story = &model.stories[si];
    let height_mm = if si == 0 {
        story.elevation
    } else {
        story.elevation - model.stories[si - 1].elevation
    };
    if height_mm <= 0.0 {
        return 0.0;
    }
    let top = story.node_ids.first().copied();
    let bot = if si == 0 {
        model.nodes.iter().find(|n| n.story.is_none()).map(|n| n.id)
    } else {
        model.stories[si - 1].node_ids.first().copied()
    };
    if let (Some(tn), Some(bn)) = (top, bot) {
        (node_disp(u_free, dofmap, tn, dir_idx) - node_disp(u_free, dofmap, bn, dir_idx))
            / height_mm
    } else {
        0.0
    }
}

/// 1 ステップ分の代表応答を記録する。
/// `dir_idx` は記録方向（0=X, 1=Y）、`m_r` は当該方向の M·r、`rmr` は当該方向の
/// rᵀ·M·r（合計質量）、`a_red` は縮約空間の相対加速度、`xg` は当該時刻の
/// 記録方向の地動加速度。
#[allow(clippy::too_many_arguments)]
fn record_history_step(
    history: &mut ResponseHistory,
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir_idx: usize,
    m_r: &[f64],
    rmr: f64,
    u_free: &[f64],
    a_red: &[f64],
    xg: f64,
) {
    let disp = history
        .node
        .map(|n| node_disp(u_free, dofmap, n, dir_idx))
        .unwrap_or(0.0);
    history.node_disp.push(disp);
    let a_free = reducer.expand_u(a_red);
    let ma: f64 = m_r.iter().zip(a_free.iter()).map(|(m, a)| m * a).sum();
    history.base_shear.push(-(ma + xg * rmr));
    history
        .top_drift_angle
        .push(current_top_drift(model, dofmap, u_free, dir_idx));
}

/// rᵀ·M·r （記録方向 `dir_idx` の合計質量）。ベースシア計算に使う。
fn total_mass(m_r: &[f64], dofmap: &DofMap, n_nodes: usize, dir_idx: usize) -> f64 {
    let mut s = 0.0;
    for ni in 0..n_nodes {
        if let Some(a) = dofmap.active(ni * DOF_PER_NODE + dir_idx) {
            s += m_r[a as usize];
        }
    }
    s
}

/// 時刻歴応答の1時点の状態（縮約空間）。チェックポイント／再開で使用。
#[derive(Clone, serde::Serialize, serde::Deserialize, PartialEq, Debug)]
pub struct TimeStepState {
    pub step: u64,
    pub time: f64,
    pub disp_red: Vec<f64>,
    pub vel_red: Vec<f64>,
    pub accel_red: Vec<f64>,
}

/// 線形時刻歴応答解析（Newmark-β 法、基盤一様加振）。
///
/// `initial_disp`/`initial_vel` は縮約空間（n_indep 長）の初期値。
/// 自由振動（地震波なし）の場合は `wave.accel_x` をゼロ埋めして呼ぶ。
/// `newmark.dt == 0.0` のときは `wave.dt` を採用する。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_analysis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
) -> Result<ResponseResult, SolveError> {
    let (result, _state) = linear_time_history_with_state(
        model,
        dofmap,
        reducer,
        wave,
        newmark,
        damping,
        initial_disp,
        initial_vel,
        use_kg,
    )?;
    Ok(result)
}

/// 線形時刻歴応答解析（最終状態付き）。チェックポイント保存用に最終状態を返す。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_with_state(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);

    let dt = if newmark.dt > 0.0 {
        newmark.dt
    } else {
        wave.dt
    };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok((
            ResponseResult {
                time: vec![],
                peak_disp: vec![[0.0; 6]; model.nodes.len()],
                story_drift_angle: vec![0.0; model.stories.len()],
                cumulative_ductility: vec![0.0; model.elements.len()],
                history: ResponseHistory::default(),
            },
            TimeStepState {
                step: 0,
                time: 0.0,
                disp_red: vec![],
                vel_red: vec![],
                accel_red: vec![],
            },
        ));
    }

    // --- 行列組立（縮約空間） ---
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    let _ = use_kg;
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    // --- 影響ベクトルと M·r の事前計算 ---
    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE + 0;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    let m_r_x = sparse_matvec(&m_free, &r_x_free);
    let m_r_y = sparse_matvec(&m_free, &r_y_free);

    // --- Newmark-β 係数 ---
    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    // --- 有効剛性 K^ = K + c2·C + c1・M ---
    let k_eff = squid_n_math::sparse::weighted_sum_csc(
        n_indep,
        &[(1.0, &k_red), (c2, &c_red), (c1, &m_red)],
    );

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_eff)?;

    // --- 初期条件 ---
    let mut u = vec![0.0; n_indep];
    let mut v = vec![0.0; n_indep];
    let n_init_d = n_indep.min(initial_disp.len());
    u[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
    let n_init_v = n_indep.min(initial_vel.len());
    v[..n_init_v].copy_from_slice(&initial_vel[..n_init_v]);

    // 初期加速度: M·a_0 = -C·v_0 - K·u_0 - p_red(0)
    let xg0_x = wave.accel_x.first().copied().unwrap_or(0.0);
    let xg0_y = wave
        .accel_y
        .as_ref()
        .and_then(|a| a.first())
        .copied()
        .unwrap_or(0.0);
    let p_free_0: Vec<f64> = m_r_x
        .iter()
        .zip(m_r_y.iter())
        .map(|(mx, my)| -(mx * xg0_x + my * xg0_y))
        .collect();
    let p_red_0 = reducer.reduce_f(&p_free_0);

    let cv0 = sparse_matvec(&c_red, &v);
    let ku0 = sparse_matvec(&k_red, &u);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = -cv0[i] - ku0[i] - p_red_0[i];
    }
    let a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    // --- 時刻歴ループ（start_step=0 から） ---
    run_steps(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        0,
        &m_r_x,
        &m_r_y,
        &m_red,
        &c_red,
        &mut solver,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        gamma,
        u,
        v,
        a,
    )
}

/// チェックポイントから線形時刻歴を再開する。
/// `state.step` の次のステップから `wave` の終端まで進める。
/// `wave` は全ステップ分の地震波（先頭から）。`state.step` 以降を使用する。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_from_state(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    state: &TimeStepState,
    use_kg: bool,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);

    let dt = if newmark.dt > 0.0 {
        newmark.dt
    } else {
        wave.dt
    };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }

    let n_indep = reducer.n_indep;
    if n_indep == 0 || state.disp_red.len() != n_indep {
        return Err(SolveError::Backend(
            "time history restart: state dimension mismatch".into(),
        ));
    }

    // 行列・係数の再計算（線形なので同一）
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    let _ = use_kg;
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE + 0;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    let m_r_x = sparse_matvec(&m_free, &r_x_free);
    let m_r_y = sparse_matvec(&m_free, &r_y_free);

    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    let k_eff = squid_n_math::sparse::weighted_sum_csc(
        n_indep,
        &[(1.0, &k_red), (c2, &c_red), (c1, &m_red)],
    );
    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_eff)?;

    // チェックポイントから状態を復元
    let u = state.disp_red.clone();
    let v = state.vel_red.clone();
    let a = state.accel_red.clone();

    run_steps(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        state.step,
        &m_r_x,
        &m_r_y,
        &m_red,
        &c_red,
        &mut solver,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        gamma,
        u,
        v,
        a,
    )
}

/// 線形時刻歴応答解析（HHT-α 法、基盤一様加振）。
///
/// β=0.25, γ=0.5（平均加速度法）で固定。
/// `initial_disp`/`initial_vel` は縮約空間（n_indep 長）の初期値。
/// `hht.dt == 0.0` のときは `wave.dt` を採用する。
/// α=0 で標準 Newmark-β（平均加速度法）に一致。
#[allow(clippy::too_many_arguments)]
pub fn linear_hht_alpha_analysis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    hht: &HhtCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
) -> Result<ResponseResult, SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);

    let dt = if hht.dt > 0.0 { hht.dt } else { wave.dt };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }
    let alpha = hht.alpha;

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok(ResponseResult {
            time: vec![],
            peak_disp: vec![[0.0; 6]; model.nodes.len()],
            story_drift_angle: vec![0.0; model.stories.len()],
            cumulative_ductility: vec![0.0; model.elements.len()],
            history: ResponseHistory::default(),
        });
    }

    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    let _ = use_kg;
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE + 0;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    let m_r_x = sparse_matvec(&m_free, &r_x_free);
    let m_r_y = sparse_matvec(&m_free, &r_y_free);

    // HHT-α は β=0.25, γ=0.5 で固定（平均加速度法ベース）
    let beta = 0.25;
    let gamma = 0.5;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    // K^_HHT = (1+α)·K + (1+α)·c2·C + c1·M
    let k_eff = squid_n_math::sparse::weighted_sum_csc(
        n_indep,
        &[
            (1.0 + alpha, &k_red),
            ((1.0 + alpha) * c2, &c_red),
            (c1, &m_red),
        ],
    );

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_eff)?;

    // --- 初期条件 ---
    let mut u = vec![0.0; n_indep];
    let mut v = vec![0.0; n_indep];
    let n_init_d = n_indep.min(initial_disp.len());
    u[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
    let n_init_v = n_indep.min(initial_vel.len());
    v[..n_init_v].copy_from_slice(&initial_vel[..n_init_v]);

    // 初期加速度: Newmark と同じ（M·a_0 = -C·v_0 - K·u_0 - p_0）
    let xg0_x = wave.accel_x.first().copied().unwrap_or(0.0);
    let xg0_y = wave
        .accel_y
        .as_ref()
        .and_then(|a| a.first())
        .copied()
        .unwrap_or(0.0);
    let p_free_0: Vec<f64> = m_r_x
        .iter()
        .zip(m_r_y.iter())
        .map(|(mx, my)| -(mx * xg0_x + my * xg0_y))
        .collect();
    let p_red_0 = reducer.reduce_f(&p_free_0);

    let cv0 = sparse_matvec(&c_red, &v);
    let ku0 = sparse_matvec(&k_red, &u);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = -cv0[i] - ku0[i] - p_red_0[i];
    }
    let a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    let (result, _state) = run_steps_hht(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        0,
        &m_r_x,
        &m_r_y,
        &m_red,
        &c_red,
        &k_red,
        &mut solver,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        alpha,
        p_red_0,
        u,
        v,
        a,
    )?;
    Ok(result)
}

/// 初期加速度 M·a₀ = rhs を解く。
/// 質量行列は回転自由度などに質量ゼロ行を含み特異になり得るため、
/// Cholesky → LU の順に試し、いずれも失敗した場合は
/// rhs≈0（静止開始）なら a₀ = 0 とみなす。
fn solve_initial_accel(
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    rhs: &[f64],
    n_indep: usize,
) -> Result<Vec<f64>, SolveError> {
    let mut chol = make_solver(SolverBackend::DirectSparseCholesky);
    if chol.factorize(m_red).is_ok() {
        return chol.solve(rhs);
    }
    let mut lu = make_solver(SolverBackend::DirectSparseLu);
    if lu.factorize(m_red).is_ok() {
        if let Ok(a) = lu.solve(rhs) {
            if a.iter().all(|v| v.is_finite()) {
                return Ok(a);
            }
        }
    }
    let rhs_norm: f64 = rhs.iter().map(|v| v * v).sum::<f64>().sqrt();
    if rhs_norm < 1e-9 {
        // 静止開始（初期外力ゼロ）なら初期加速度もゼロ
        return Ok(vec![0.0; n_indep]);
    }
    Err(SolveError::InvalidInput(
        "質量行列が特異で初期加速度を計算できません。地震波の先頭を 0 から始めるか、全自由度に質量を与えてください。".into(),
    ))
}

#[allow(clippy::too_many_arguments)]
fn run_steps_hht(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    dt: f64,
    start_step: u64,
    m_r_x: &[f64],
    m_r_y: &[f64],
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    c_red: &faer::sparse::SparseColMat<usize, f64>,
    k_red: &faer::sparse::SparseColMat<usize, f64>,
    solver: &mut Box<dyn squid_n_math::solver::LinearSolver>,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    c5: f64,
    c6: f64,
    alpha: f64,
    mut p_prev: Vec<f64>,
    mut u: Vec<f64>,
    mut v: Vec<f64>,
    mut a: Vec<f64>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    let n_indep = reducer.n_indep;
    let n_free = dofmap.n_active();

    let mut peak_disp_free = vec![0.0f64; n_free];
    let u_free_init = reducer.expand_u(&u);
    for i in 0..n_free {
        peak_disp_free[i] = peak_disp_free[i].max(u_free_init[i].abs());
    }
    let mut story_drift_angle = vec![0.0f64; model.stories.len()];
    update_story_drift(model, dofmap, &u_free_init, &mut story_drift_angle);

    let mut time = Vec::with_capacity(wave.accel_x.len() - start_step as usize + 1);
    time.push(start_step as f64 * dt);

    // UI 用の代表応答記録（記録方向は入力加速度の絶対値和が大きい方を自動選択）
    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record = if record_dir_y { m_r_y } else { m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    let xg_init = if record_dir_y {
        wave.accel_y
            .as_ref()
            .and_then(|a| a.get(start_step as usize).copied())
            .unwrap_or(0.0)
    } else {
        wave.accel_x
            .get(start_step as usize)
            .copied()
            .unwrap_or(0.0)
    };
    record_history_step(
        &mut history,
        model,
        dofmap,
        reducer,
        dir_idx,
        m_r_record,
        rmr_record,
        &u_free_init,
        &a,
        xg_init,
    );

    for n in start_step as usize..wave.accel_x.len() {
        let t_next = (n + 1) as f64 * dt;
        let xg_x = wave.accel_x[n];
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);

        let p_free: Vec<f64> = m_r_x
            .iter()
            .zip(m_r_y.iter())
            .map(|(mx, my)| -(mx * xg_x + my * xg_y))
            .collect();
        let p_red = reducer.reduce_f(&p_free);

        let mut mw = vec![0.0; n_indep];
        let mut cw = vec![0.0; n_indep];
        for i in 0..n_indep {
            mw[i] = c1 * u[i] + c3 * v[i] + c4 * a[i];
            cw[i] = c2 * u[i] + c5 * v[i] + c6 * a[i];
        }
        let m_mw = sparse_matvec(m_red, &mw);
        let c_cw = sparse_matvec(c_red, &cw);
        let c_vn = sparse_matvec(c_red, &v);
        let k_un = sparse_matvec(k_red, &u);

        let mut p_eff = vec![0.0; n_indep];
        for i in 0..n_indep {
            p_eff[i] = (1.0 + alpha) * p_red[i] - alpha * p_prev[i]
                + m_mw[i]
                + (1.0 + alpha) * c_cw[i]
                + alpha * (c_vn[i] + k_un[i]);
        }

        let u_next = solver.solve(&p_eff)?;

        let mut a_next = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_next[i] = c1 * (u_next[i] - u[i]) - c3 * v[i] - c4 * a[i];
        }
        let mut v_next = vec![0.0; n_indep];
        for i in 0..n_indep {
            v_next[i] = v[i] + dt * ((1.0 - 0.5) * a[i] + 0.5 * a_next[i]);
        }

        u = u_next;
        v = v_next;
        a = a_next;
        p_prev = p_red;
        time.push(t_next);

        let u_free = reducer.expand_u(&u);
        for i in 0..n_free {
            peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
        }
        update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
        let xg_next = if record_dir_y {
            wave.accel_y
                .as_ref()
                .and_then(|a| a.get(n + 1).copied())
                .unwrap_or(0.0)
        } else {
            wave.accel_x.get(n + 1).copied().unwrap_or(0.0)
        };
        record_history_step(
            &mut history,
            model,
            dofmap,
            reducer,
            dir_idx,
            m_r_record,
            rmr_record,
            &u_free,
            &a,
            xg_next,
        );
    }

    let final_step = wave.accel_x.len() as u64;
    let final_time = final_step as f64 * dt;
    let final_state = TimeStepState {
        step: final_step,
        time: final_time,
        disp_red: u.clone(),
        vel_red: v.clone(),
        accel_red: a.clone(),
    };

    let mut peak_disp = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                peak_disp[ni][d] = peak_disp_free[a as usize];
            }
        }
    }

    Ok((
        ResponseResult {
            time,
            peak_disp,
            story_drift_angle,
            cumulative_ductility: vec![0.0; model.elements.len()],
            history,
        },
        final_state,
    ))
}

/// 時刻歴ステップを `start_step` から `wave` の終端まで進める内部関数。
/// `start_step` は既に確定した状態（u, v, a は step `start_step` の値）。
/// 次のステップ `start_step` → `start_step+1` は `wave[start_step]` を使う。
#[allow(clippy::too_many_arguments)]
fn run_steps(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    dt: f64,
    start_step: u64,
    m_r_x: &[f64],
    m_r_y: &[f64],
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    c_red: &faer::sparse::SparseColMat<usize, f64>,
    solver: &mut Box<dyn squid_n_math::solver::LinearSolver>,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    c5: f64,
    c6: f64,
    gamma: f64,
    mut u: Vec<f64>,
    mut v: Vec<f64>,
    mut a: Vec<f64>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    let n_indep = reducer.n_indep;
    let n_free = dofmap.n_active();

    let mut peak_disp_free = vec![0.0f64; n_free];
    let u_free_init = reducer.expand_u(&u);
    for i in 0..n_free {
        peak_disp_free[i] = peak_disp_free[i].max(u_free_init[i].abs());
    }
    let mut story_drift_angle = vec![0.0f64; model.stories.len()];
    update_story_drift(model, dofmap, &u_free_init, &mut story_drift_angle);

    let mut time = Vec::with_capacity(wave.accel_x.len() - start_step as usize + 1);
    time.push(start_step as f64 * dt);

    // UI 用の代表応答記録（記録方向は入力加速度の絶対値和が大きい方を自動選択）
    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record = if record_dir_y { m_r_y } else { m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    let xg_init = if record_dir_y {
        wave.accel_y
            .as_ref()
            .and_then(|a| a.get(start_step as usize).copied())
            .unwrap_or(0.0)
    } else {
        wave.accel_x
            .get(start_step as usize)
            .copied()
            .unwrap_or(0.0)
    };
    record_history_step(
        &mut history,
        model,
        dofmap,
        reducer,
        dir_idx,
        m_r_record,
        rmr_record,
        &u_free_init,
        &a,
        xg_init,
    );

    for n in start_step as usize..wave.accel_x.len() {
        let t_next = (n + 1) as f64 * dt;
        let xg_x = wave.accel_x[n];
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);

        let p_free: Vec<f64> = m_r_x
            .iter()
            .zip(m_r_y.iter())
            .map(|(mx, my)| -(mx * xg_x + my * xg_y))
            .collect();
        let p_red = reducer.reduce_f(&p_free);

        let mut mw = vec![0.0; n_indep];
        let mut cw = vec![0.0; n_indep];
        for i in 0..n_indep {
            mw[i] = c1 * u[i] + c3 * v[i] + c4 * a[i];
            cw[i] = c2 * u[i] + c5 * v[i] + c6 * a[i];
        }
        let m_mw = sparse_matvec(m_red, &mw);
        let c_cw = sparse_matvec(c_red, &cw);

        let mut p_eff = vec![0.0; n_indep];
        for i in 0..n_indep {
            p_eff[i] = p_red[i] + m_mw[i] + c_cw[i];
        }

        let u_next = solver.solve(&p_eff)?;

        let mut a_next = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_next[i] = c1 * (u_next[i] - u[i]) - c3 * v[i] - c4 * a[i];
        }
        let mut v_next = vec![0.0; n_indep];
        for i in 0..n_indep {
            v_next[i] = v[i] + dt * ((1.0 - gamma) * a[i] + gamma * a_next[i]);
        }

        u = u_next;
        v = v_next;
        a = a_next;
        time.push(t_next);

        let u_free = reducer.expand_u(&u);
        for i in 0..n_free {
            peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
        }
        update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
        let xg_next = if record_dir_y {
            wave.accel_y
                .as_ref()
                .and_then(|a| a.get(n + 1).copied())
                .unwrap_or(0.0)
        } else {
            wave.accel_x.get(n + 1).copied().unwrap_or(0.0)
        };
        record_history_step(
            &mut history,
            model,
            dofmap,
            reducer,
            dir_idx,
            m_r_record,
            rmr_record,
            &u_free,
            &a,
            xg_next,
        );
    }

    let final_step = wave.accel_x.len() as u64;
    let final_time = final_step as f64 * dt;
    let final_state = TimeStepState {
        step: final_step,
        time: final_time,
        disp_red: u.clone(),
        vel_red: v.clone(),
        accel_red: a.clone(),
    };

    let mut peak_disp = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                peak_disp[ni][d] = peak_disp_free[a as usize];
            }
        }
    }

    Ok((
        ResponseResult {
            time,
            peak_disp,
            story_drift_angle,
            cumulative_ductility: vec![0.0; model.elements.len()],
            history,
        },
        final_state,
    ))
}

/// 層間変形角を更新する（各層の最大値を追跡）。X 方向の水平変位差／階高。
fn update_story_drift(
    model: &Model,
    dofmap: &DofMap,
    u_free: &[f64],
    story_drift_angle: &mut [f64],
) {
    for (si, story) in model.stories.iter().enumerate() {
        if si >= story_drift_angle.len() {
            break;
        }
        let height_mm = if si == 0 {
            story.elevation
        } else {
            story.elevation - model.stories[si - 1].elevation
        };
        if height_mm <= 0.0 {
            continue;
        }
        let top = story.node_ids.first().copied();
        let bot = if si == 0 {
            // 1層目: 基礎節点（story=None の最初の節点）を下端とする
            model.nodes.iter().find(|n| n.story.is_none()).map(|n| n.id)
        } else {
            model.stories[si - 1].node_ids.first().copied()
        };
        if let (Some(tn), Some(bn)) = (top, bot) {
            // 層間変形角は従来通り X 方向（0）で評価する（ResponseHistory の
            // 記録方向とは独立）。
            let du = (node_disp(u_free, dofmap, tn, 0) - node_disp(u_free, dofmap, bn, 0)).abs();
            let angle = du / height_mm;
            if angle > story_drift_angle[si] {
                story_drift_angle[si] = angle;
            }
        }
    }
}

/// 節点の並進自由度 `dir_idx`（0=X, 1=Y, 2=Z）の相対変位を返す。
fn node_disp(
    u_free: &[f64],
    dofmap: &DofMap,
    node_id: squid_n_core::ids::NodeId,
    dir_idx: usize,
) -> f64 {
    let ni = node_id.index();
    let g = ni * DOF_PER_NODE + dir_idx;
    if let Some(a) = dofmap.active(g) {
        u_free[a as usize]
    } else {
        0.0
    }
}

/// 非線形時刻歴応答解析（Newmark-β + Newton反復 + commit/rollback）。
///
/// 各時刻ステップで Newton 反復により内力（非線形復元力）と慣性力・減衰力・
/// 地震外力の動的釣合いを満たす解を求める。収束時は要素状態を commit、
/// 不収束時は Step 開始時の状態へ rollback する。
#[allow(clippy::too_many_arguments)]
pub fn nonlinear_time_history_analysis(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
    max_iter: usize,
    tol: f64,
) -> Result<ResponseResult, SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);

    let dt = if newmark.dt > 0.0 {
        newmark.dt
    } else {
        wave.dt
    };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok(ResponseResult {
            time: vec![],
            peak_disp: vec![[0.0; 6]; model.nodes.len()],
            story_drift_angle: vec![0.0; model.stories.len()],
            cumulative_ductility: vec![0.0; model.elements.len()],
            history: ResponseHistory::default(),
        });
    }

    let mut behaviors = build_behaviors(model);

    // 行列組立（縮約空間）
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    let _ = use_kg;
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    // 影響ベクトルと M·r
    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE + 0;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    let m_r_x = sparse_matvec(&m_free, &r_x_free);
    let m_r_y = sparse_matvec(&m_free, &r_y_free);

    // Newmark-β 係数
    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    // 初期条件
    let mut u = vec![0.0; n_indep];
    let mut v = vec![0.0; n_indep];
    let n_init_d = n_indep.min(initial_disp.len());
    u[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
    let n_init_v = n_indep.min(initial_vel.len());
    v[..n_init_v].copy_from_slice(&initial_vel[..n_init_v]);

    // 初期変位を要素状態に反映
    {
        let u_free_init = reducer.expand_u(&u);
        let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
        for (_elem, b) in model.elements.iter().zip(behaviors.iter_mut()) {
            let gdofs = b.global_dofs(dofmap);
            let mut du_elem = LocalVec {
                data: SmallVec::from_elem(0.0, 12),
            };
            for (i, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free_init.len() {
                    du_elem.data[i] = u_free_init[g];
                }
            }
            let dummy_ctx = Ctx {
                model: unsafe { &*model_ptr },
            };
            b.update_state(&du_elem, false, &dummy_ctx);
        }
        for b in behaviors.iter_mut() {
            b.commit_state();
        }
    }

    // 初期加速度: M·a_0 = -C·v_0 - f_int(u_0) - p_red(0)
    let xg0_x = wave.accel_x.first().copied().unwrap_or(0.0);
    let xg0_y = wave
        .accel_y
        .as_ref()
        .and_then(|a| a.first())
        .copied()
        .unwrap_or(0.0);
    let p_free_0: Vec<f64> = m_r_x
        .iter()
        .zip(m_r_y.iter())
        .map(|(mx, my)| -(mx * xg0_x + my * xg0_y))
        .collect();
    let p_red_0 = reducer.reduce_f(&p_free_0);

    let f_int0_free = compute_f_int(model, dofmap, &behaviors);
    let f_int0_red = reducer.reduce_f(&f_int0_free);
    let cv0 = sparse_matvec(&c_red, &v);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = -cv0[i] - f_int0_red[i] - p_red_0[i];
    }
    let mut a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    // --- 時刻歴ループ ---
    let n_steps = wave.accel_x.len();
    let mut peak_disp_free = vec![0.0f64; n_free];
    {
        let u_free_init = reducer.expand_u(&u);
        for i in 0..n_free {
            peak_disp_free[i] = peak_disp_free[i].max(u_free_init[i].abs());
        }
    }
    let mut story_drift_angle = vec![0.0f64; model.stories.len()];
    {
        let u_free_init = reducer.expand_u(&u);
        update_story_drift(model, dofmap, &u_free_init, &mut story_drift_angle);
    }

    // UI 用の代表応答記録（記録方向は入力加速度の絶対値和が大きい方を自動選択）
    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record: &[f64] = if record_dir_y { &m_r_y } else { &m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    {
        let u_free_init = reducer.expand_u(&u);
        let xg_init = if record_dir_y {
            wave.accel_y
                .as_ref()
                .and_then(|a| a.first().copied())
                .unwrap_or(0.0)
        } else {
            wave.accel_x.first().copied().unwrap_or(0.0)
        };
        record_history_step(
            &mut history,
            model,
            dofmap,
            reducer,
            dir_idx,
            m_r_record,
            rmr_record,
            &u_free_init,
            &a,
            xg_init,
        );
    }
    let mut time = Vec::with_capacity(n_steps + 1);
    time.push(0.0);

    for n in 0..n_steps {
        let t_next = (n + 1) as f64 * dt;

        // スナップショット
        let snap = StateSnapshot::capture(&behaviors);

        // 地震荷重
        let xg_x = wave.accel_x[n];
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);
        let p_free: Vec<f64> = m_r_x
            .iter()
            .zip(m_r_y.iter())
            .map(|(mx, my)| -(mx * xg_x + my * xg_y))
            .collect();
        let p_red = reducer.reduce_f(&p_free);

        // 予測子: Δu = 0 での a, v
        let mut a_trial = vec![0.0; n_indep];
        let mut v_trial = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_trial[i] = -c3 * v[i] - c4 * a[i];
            v_trial[i] = -c5 * v[i] - c6 * a[i];
        }

        let mut du_total = vec![0.0; n_indep];
        let mut converged = false;

        let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;

        for _iter in 0..max_iter {
            // 接線剛性
            let k_t_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
            let k_t_red = reducer.reduce_k(&k_t_free);

            // 有効剛性
            let k_eff = weighted_sum_csc(n_indep, &[(1.0, &k_t_red), (c2, &c_red), (c1, &m_red)]);

            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            solver
                .factorize(&k_eff)
                .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;

            // 内力
            let f_int_free = compute_f_int(model, dofmap, &behaviors);
            let f_int_red = reducer.reduce_f(&f_int_free);

            // C·v と M·a（縮約空間）
            let c_v_red = sparse_matvec(&c_red, &v_trial);
            let m_a_red = sparse_matvec(&m_red, &a_trial);

            // 残差
            let mut r_red = vec![0.0; n_indep];
            for i in 0..n_indep {
                r_red[i] = p_red[i] - f_int_red[i] - c_v_red[i] - m_a_red[i];
            }

            // 収束判定
            let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            let p_norm: f64 = p_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            if r_norm < tol * p_norm.max(1.0) {
                converged = true;
                break;
            }

            // δu を解く
            let du_red = solver.solve(&r_red)?;
            let du_free = reducer.expand_u(&du_red);

            // a, v を更新
            for i in 0..n_indep {
                a_trial[i] += c1 * du_red[i];
                v_trial[i] += c2 * du_red[i];
                du_total[i] += du_red[i];
            }

            // 要素状態を trial 更新
            for (_elem, b) in model.elements.iter().zip(behaviors.iter_mut()) {
                let gdofs = b.global_dofs(dofmap);
                let mut du_elem = LocalVec {
                    data: SmallVec::from_elem(0.0, 12),
                };
                for (i, &g) in gdofs.iter().enumerate() {
                    if g != usize::MAX && g < du_free.len() {
                        du_elem.data[i] = du_free[g];
                    }
                }
                let dummy_ctx = Ctx {
                    model: unsafe { &*model_ptr },
                };
                b.update_state(&du_elem, false, &dummy_ctx);
            }
        }

        if converged {
            for i in 0..n_indep {
                u[i] += du_total[i];
            }
            v.copy_from_slice(&v_trial);
            a.copy_from_slice(&a_trial);

            for b in behaviors.iter_mut() {
                b.commit_state();
            }

            time.push(t_next);

            let u_free = reducer.expand_u(&u);
            for i in 0..n_free {
                peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
            }
            update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
            let xg_next = if record_dir_y {
                wave.accel_y
                    .as_ref()
                    .and_then(|a| a.get(n + 1).copied())
                    .unwrap_or(0.0)
            } else {
                wave.accel_x.get(n + 1).copied().unwrap_or(0.0)
            };
            record_history_step(
                &mut history,
                model,
                dofmap,
                reducer,
                dir_idx,
                m_r_record,
                rmr_record,
                &u_free,
                &a,
                xg_next,
            );
        } else {
            // 不収束: rollback
            model.restore(&snap, &mut behaviors);
            return Err(SolveError::Backend(format!(
                "nonlinear time history: step {} did not converge",
                n
            )));
        }
    }

    let mut peak_disp = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                peak_disp[ni][d] = peak_disp_free[a as usize];
            }
        }
    }

    Ok(ResponseResult {
        time,
        peak_disp,
        story_drift_angle,
        cumulative_ductility: vec![0.0; model.elements.len()],
        history,
    })
}

fn build_behaviors(model: &Model) -> Vec<Box<dyn squid_n_element::behavior::ElementBehavior>> {
    let mut behaviors = Vec::new();
    for elem in &model.elements {
        let (mut b, _) = build_nonlinear_behavior(elem, model);
        // 動的解析: コンクリート履歴は原点指向型（RESP-D「05 非線形モデル」）。
        b.set_concrete_hysteresis(true);
        behaviors.push(b);
    }
    behaviors
}

/// Rayleigh 減衰の係数 (α_m, β_k) を、2つの振動数と目標減衰比から計算する。
pub fn rayleigh_coeffs(omega1: f64, omega2: f64, h1: f64, h2: f64) -> (f64, f64) {
    Damping::rayleigh_coeffs(omega1, omega2, h1, h2)
}

/// 時刻歴ソルバ設定の決定性（R28）: Newmark/HHT 設定のビット一致確認。
#[cfg(test)]
mod tests;
