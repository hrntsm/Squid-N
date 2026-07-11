use crate::analysis::{distribute_pi_over_diaphragms, steel_height_ratio, SeismicDir};
use crate::arc_length::ArcLengthSolver;
use crate::constraint::Reducer;
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::{ElemId, StoryId};
use squid_n_core::model::{ElementData, Material, Model, RigidZone, Section};
use squid_n_core::rc_capacity::{rc_qsu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
use squid_n_element::factory::build_nonlinear_behavior;
use squid_n_element::transform::LocalFrame;
use squid_n_math::solver::{make_solver, SolverBackend};

/// 性能曲線の1点（P5 §7.4）
pub struct CapacityPoint {
    pub step: u32,
    pub roof_disp: f64,
    pub base_shear: f64,
    pub story_shear: Vec<f64>,
    pub story_drift: Vec<f64>,
}

/// ヒンジ発生事象（P5 §7.4）
pub struct HingeEvent {
    pub step: u32,
    pub elem: ElemId,
    pub pos: f64,
    pub level: HingeLevel,
    pub ductility: f64,
}

/// ヒンジレベル（P5 §7.4）
pub enum HingeLevel {
    Crack,
    Yield,
    Ultimate,
}

/// 崩壊機構種別（P5 §7.4）
pub enum MechanismType {
    Overall,
    StoryCollapse { story: StoryId },
    Partial,
}

/// せん断降伏イベント（RESP-D マニュアル計算編03「応力解析」§段階的耐力喪失解析）。
///
/// 部材端のせん断力（局所 Vy・Vz の材端最大値）がせん断降伏耐力 Qy
/// （[`compute_shear_yield_qy`] 参照）を超えたステップを記録する。曲げヒンジ
/// （[`HingeEvent`]）とは独立に判定され、曲げ降伏の有無に関わらず記録される。
pub struct ShearYieldEvent {
    pub step: u32,
    pub elem: ElemId,
}

/// プッシュオーバー解析結果（P5 §7.4）
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    /// せん断降伏イベント履歴（段階的耐力喪失解析の判定に使用、`strength_loss` モジュール参照）。
    pub shear_yields: Vec<ShearYieldEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
    /// 当該ステップ確定時点の全自由節点変位（`DofMap` のアクティブ添字順）。
    /// 段階的耐力喪失解析（`strength_loss` モジュール）が部材変形角を算定するための
    /// 記録で、既定では収集しない（オプトイン、`pushover_analysis_recording` 参照）。
    pub node_disp: Option<Vec<f64>>,
}

pub(crate) fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    prescribed: Option<(usize, f64)>,
) -> faer::sparse::SparseColMat<usize, f64> {
    use squid_n_math::sparse::assemble_csc;
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut triplets = Vec::new();
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let mut k = b.tangent_stiffness(&state, &ctx);
        if use_kg {
            let f = b.internal_force(&state, &ctx);
            let n = f.data.first().copied().unwrap_or(0.0);
            let kg = b.geometric_stiffness(n);
            for i in 0..12 {
                for j in 0..12 {
                    let sum = k.get(i, j) + kg.get(i, j);
                    k.set(i, j, sum);
                }
            }
        }
        triplets.extend(k.to_triplets(&gdofs));
    }
    if let Some((d, _u_val)) = prescribed {
        let penalty = 1e16;
        triplets.push(squid_n_math::sparse::Triplet {
            row: d,
            col: d,
            val: penalty,
        });
    }
    assemble_csc(dofmap.n_active(), triplets)
}

pub(crate) fn compute_f_int(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
) -> Vec<f64> {
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut f = vec![0.0; dofmap.n_active()];
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let f_local = b.internal_force(&state, &ctx);
        for (&g, &v) in gdofs.iter().zip(f_local.data.iter()) {
            if g != usize::MAX {
                f[g] += v;
            }
        }
    }
    f
}

/// ベースシア（層せん断の総和）を内力の釣合いから求める（P5 §7.4）。
///
/// 静的釣合いでは各自由節点の水平内力 = 外力。よって全自由節点の載荷方向
/// 並進 DOF にわたる内力の総和が、構造全体が支持点へ伝える水平力＝ベースシア
/// に等しい。DOF 添字を直接足す旧実装（`f_int[0..roof].sum()`）は誤り。
fn compute_base_shear(model: &Model, dofmap: &DofMap, f_int: &[f64], dir: SeismicDir) -> f64 {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    let mut v = 0.0;
    for node in &model.nodes {
        let g = node.id.index() * 6 + dir_idx;
        if let Some(a) = dofmap.active(g) {
            v += f_int[a as usize];
        }
    }
    v
}

/// 層せん断力を内力の釣合いから求める（P5 §7.4、P7 の Qu 突合に使用）。
///
/// 第 i 層のせん断力 Q_i = 第 i 層以上の階に属する節点へ作用する
/// 載荷方向水平内力の合計（上層から累積）。階に属さない中間節点は
/// 集計対象外（階の自動生成はレベル単位で節点をクラスタリングするため、
/// 通常のフレームでは全自由節点がいずれかの階に属する）。
/// stories が空なら空ベクトルを返す。
fn compute_story_shear(model: &Model, dofmap: &DofMap, f_int: &[f64], dir: SeismicDir) -> Vec<f64> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    let n = model.stories.len();
    let mut level_force = vec![0.0; n];
    for (i, story) in model.stories.iter().enumerate() {
        for nid in &story.node_ids {
            let g = nid.index() * 6 + dir_idx;
            if let Some(a) = dofmap.active(g) {
                if let Some(&v) = f_int.get(a as usize) {
                    level_force[i] += v;
                }
            }
        }
    }
    let mut shear = vec![0.0; n];
    let mut acc = 0.0;
    for i in (0..n).rev() {
        acc += level_force[i];
        shear[i] = acc;
    }
    shear
}

/// 層間変位を剛床マスター節点の水平変位差から求める。
/// 第 i 層の層間変位 = マスター変位(第 i 層) − マスター変位(1 つ下の階)。
/// 最下層は基部（変位 0）との差。マスターが無い／拘束済みの階は変位 0 とみなす。
fn compute_story_drift(
    model: &Model,
    dofmap: &DofMap,
    total_disp: &[f64],
    dir: SeismicDir,
) -> Vec<f64> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    let mut prev = 0.0;
    model
        .stories
        .iter()
        .map(|story| {
            let d = story
                .diaphragms
                .first()
                .and_then(|dia| {
                    let g = dia.master.index() * 6 + dir_idx;
                    dofmap
                        .active(g)
                        .and_then(|a| total_disp.get(a as usize).copied())
                })
                .unwrap_or(0.0);
            let drift = d - prev;
            prev = d;
            drift
        })
        .collect()
}

fn get_roof_disp(total_disp: &[f64], model: &Model, dofmap: &DofMap, dir: SeismicDir) -> f64 {
    if let Some(story) = model.stories.last() {
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let dof_idx = match dir {
                SeismicDir::X => 0,
                SeismicDir::Y => 1,
            };
            let g = ni * 6 + dof_idx;
            if let Some(a) = dofmap.active(g) {
                let idx = a as usize;
                if idx < total_disp.len() {
                    return total_disp[idx];
                }
            }
        }
    }
    0.0
}

/// プッシュオーバー解析（P5 §7）
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
) -> Result<PushoverResult, String> {
    pushover_analysis_recording(
        model,
        dofmap,
        reducer,
        dir,
        max_steps,
        max_disp,
        use_kg,
        use_arc_length,
        arc_length_dl,
        false,
    )
}

/// プッシュオーバー解析（P5 §7）。`record_node_disp` が真の場合、各ステップの
/// `PushoverStep::node_disp` に全自由節点変位を記録する（段階的耐力喪失解析の
/// 部材変形角算定用、`strength_loss` モジュール参照）。既存 API を壊さないよう
/// `pushover_analysis` は本関数に `record_node_disp = false` で委譲する薄いラッパー。
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis_recording(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
    record_node_disp: bool,
) -> Result<PushoverResult, String> {
    let n_active = dofmap.n_active();
    if n_active == 0 {
        return Err("no active DOF".into());
    }

    let mut behaviors: Vec<Box<dyn ElementBehavior>> = Vec::new();
    for elem in &model.elements {
        let (b, _) = build_nonlinear_behavior(elem, model);
        behaviors.push(b);
    }

    let stories = &model.stories;
    if stories.is_empty() {
        return Err("no stories defined".into());
    }
    let height_m = stories.last().map(|s| s.elevation).unwrap_or(0.0) / 1000.0;
    // 略算周期の鉄骨造比 α（レビュー §1.5）。steel_height_ratio は analysis.rs の
    // seismic_static_with と共有する実装。
    let steel_ratio = steel_height_ratio(model);
    let t = squid_n_load::ai::approx_t(height_m, steel_ratio);
    let z = 1.0;
    let tc = squid_n_load::ai::tc_of(squid_n_load::ai::SoilClass::II);
    let rt_val = squid_n_load::ai::rt(t, tc);
    let c0 = 0.2;
    let story_weights: Vec<f64> = stories
        .iter()
        .map(|s| s.seismic_weight.unwrap_or(0.0))
        .collect();
    if story_weights.iter().all(|&w| w == 0.0) {
        return Err("no seismic weight defined".into());
    }
    let ai = squid_n_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

    let dir_vec = match dir {
        SeismicDir::X => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        SeismicDir::Y => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    };
    let mut q = vec![0.0; n_active];
    for (i, story) in stories.iter().enumerate() {
        let pi = ai.pi.get(i).copied().unwrap_or(0.0);
        if pi == 0.0 {
            continue;
        }
        // 多剛床の階では重量比で按分する（レビュー §1.6、analysis.rs と同じ規則。
        // 従来は各剛床へ pi をそのまま重複して載せていた）。
        for (master, share) in distribute_pi_over_diaphragms(story, pi) {
            let ni = master.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                if let Some(a) = dofmap.active(g) {
                    q[a as usize] += dir_vec[d] * share;
                }
            }
        }
    }

    let thresholds = compute_hinge_thresholds(model);
    let shear_thresholds = compute_shear_yield_thresholds(model);
    let mut hinges = Vec::new();
    let mut shear_yields = Vec::new();
    let mut capacity_curve = Vec::new();
    let mut steps: Vec<PushoverStep> = Vec::new();
    let mut total_disp = vec![0.0; n_active];
    let n_steps = max_steps.clamp(1, 100);
    let dlambda = 1.0 / n_steps as f64;

    for step in 0..n_steps {
        let mut current_lambda = (step + 1) as f64 * dlambda;
        let mut step_ok = false;

        for _attempt in 0..5 {
            let snap = StateSnapshot::capture(&behaviors);
            let f_ext: Vec<f64> = q.iter().map(|&qi| qi * current_lambda).collect();
            let mut converged = false;
            let mut last_du_free: Vec<f64> = Vec::new();

            for _iter in 0..20 {
                let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
                let k_red = reducer.reduce_k(&k_free);
                let f_int = compute_f_int(model, dofmap, &behaviors);
                let r_free: Vec<f64> = f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                let r_red = reducer.reduce_f(&r_free);

                let f_ext_red = reducer.reduce_f(&f_ext);
                let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                let f_norm: f64 = f_ext_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                if r_norm < 1e-6 * f_norm.max(1.0) {
                    converged = true;
                    break;
                }

                let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
                solver
                    .factorize(&k_red)
                    .map_err(|e| format!("factor: {:?}", e))?;
                let du_red = solver
                    .solve(&r_red)
                    .map_err(|e| format!("solve: {:?}", e))?;
                let du_free = reducer.expand_u(&du_red);
                last_du_free = du_free.clone();

                let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
                for (_elem, b) in model.elements.iter_mut().zip(behaviors.iter_mut()) {
                    let gdofs = b.global_dofs(dofmap);
                    let mut du_elem = LocalVec {
                        data: SmallVec::from_elem(0.0, 12),
                    };
                    for (i, &g) in gdofs.iter().enumerate() {
                        if g != usize::MAX {
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
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for (&du, td) in last_du_free.iter().zip(total_disp.iter_mut()) {
                    *td += du;
                }
                let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                // ベースシアは内力の釣合いから算定（載荷ベクトル総和でも一致するが、
                // 変位制御フェーズと統一し反力ベースで求める）。
                let f_int_now = compute_f_int(model, dofmap, &behaviors);
                let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                capacity_curve.push(CapacityPoint {
                    step: step as u32,
                    roof_disp: roof,
                    base_shear,
                    story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                    story_drift: story_drift.clone(),
                });
                steps.push(PushoverStep {
                    // 荷重制御フェーズ: 参照外力ベクトル q に対する倍率 current_lambda を
                    // そのまま荷重係数として記録する。
                    load_factor: current_lambda,
                    top_disp: roof,
                    base_shear,
                    story_drifts: story_drift,
                    node_disp: record_node_disp.then(|| total_disp.clone()),
                });
                track_hinges(
                    model,
                    dofmap,
                    &behaviors,
                    &thresholds,
                    step as u32,
                    &mut hinges,
                );
                track_shear_yield(
                    model,
                    &behaviors,
                    &shear_thresholds,
                    step as u32,
                    &mut shear_yields,
                );
                step_ok = true;
                if max_disp > 0.0 && roof >= max_disp {
                    break;
                }
                break;
            } else {
                model.restore(&snap, &mut behaviors);
                current_lambda *= 0.5;
            }
        }
        if !step_ok {
            // 収束に至らなかった step はスキップ
        }
    }

    // 変位制御フェーズ（P5 §7.1）
    if max_disp > 0.0 {
        if let Some(roof_active) = get_roof_dof(model, dofmap, dir) {
            let initial_disp = total_disp[roof_active];
            let n_disp_steps = 10usize;
            let du_target = (max_disp - initial_disp) / n_disp_steps as f64;

            for step in 0..n_disp_steps {
                let target = initial_disp + du_target * (step + 1) as f64;
                let mut step_ok = false;

                for _attempt in 0..5 {
                    let snap = StateSnapshot::capture(&behaviors);
                    let mut converged = false;
                    let mut last_du_free = Vec::new();

                    for _iter in 0..20 {
                        let k_free = assemble_k(
                            model,
                            dofmap,
                            &behaviors,
                            use_kg,
                            Some((roof_active, target)),
                        );
                        let k_red = reducer.reduce_k(&k_free);
                        let f_int = compute_f_int(model, dofmap, &behaviors);

                        let penalty = 1e16;
                        let f_ext: Vec<f64> = (0..n_active)
                            .map(|i| {
                                if i == roof_active {
                                    target * penalty
                                } else {
                                    0.0
                                }
                            })
                            .collect();
                        let r_free: Vec<f64> =
                            f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                        let r_red = reducer.reduce_f(&r_free);

                        let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                        if r_norm < 1e-6 * target.abs().max(1.0) {
                            converged = true;
                            break;
                        }

                        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
                        solver.factorize(&k_red).map_err(|e| format!("{:?}", e))?;
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        let du_free = reducer.expand_u(&du_red);
                        last_du_free = du_free.clone();

                        let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
                        for (_elem, b) in model.elements.iter_mut().zip(behaviors.iter_mut()) {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, 12),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX {
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
                        for b in behaviors.iter_mut() {
                            b.commit_state();
                        }
                        for (&du, td) in last_du_free.iter().zip(total_disp.iter_mut()) {
                            *td += du;
                        }
                        let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                        let f_int_now = compute_f_int(model, dofmap, &behaviors);
                        let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                        let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                        let cstep = (n_steps + 1 + step) as u32;
                        capacity_curve.push(CapacityPoint {
                            step: cstep,
                            roof_disp: roof,
                            base_shear,
                            story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                            story_drift: story_drift.clone(),
                        });
                        steps.push(PushoverStep {
                            // 変位制御フェーズ: 荷重制御フェーズで λ=1 まで到達した後の継続で、
                            // 目標頂部変位をペナルティ法で強制するため比例載荷の λ という概念が
                            // 存在しない。荷重制御完了時点の値(1.0)をそのまま保持して記録する。
                            load_factor: 1.0,
                            top_disp: roof,
                            base_shear,
                            story_drifts: story_drift,
                            node_disp: record_node_disp.then(|| total_disp.clone()),
                        });
                        track_hinges(model, dofmap, &behaviors, &thresholds, cstep, &mut hinges);
                        track_shear_yield(
                            model,
                            &behaviors,
                            &shear_thresholds,
                            cstep,
                            &mut shear_yields,
                        );
                        step_ok = true;
                        break;
                    } else {
                        model.restore(&snap, &mut behaviors);
                    }
                }
                if !step_ok {
                    break;
                }
            }
        }
    }

    if use_arc_length {
        let arc_solver = ArcLengthSolver::new(arc_length_dl);
        let mut prev_du: Vec<f64> = Vec::new();
        let mut arc_lambda = 1.0;

        for _step in 0..20 {
            let snap = StateSnapshot::capture(&behaviors);
            let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
            let k_red = reducer.reduce_k(&k_free);

            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            if solver.factorize(&k_red).is_err() {
                model.restore(&snap, &mut behaviors);
                break;
            }

            // 弧長修正子の各反復で内力を再評価するため、変位増分 δu を要素状態へ
            // 反映して更新後 f_int を返すクロージャを渡す（接線 K はステップ開始時で固定＝修正 Newton）。
            let result = {
                let model_ref: &Model = &*model;
                let behaviors_ref = &mut behaviors;
                arc_solver.step(
                    &q,
                    &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                        let r_red = reducer.reduce_f(r);
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        Ok(reducer.expand_u(&du_red))
                    },
                    &mut |delta_u: &[f64]| -> Result<Vec<f64>, String> {
                        let ctx = Ctx { model: model_ref };
                        for b in behaviors_ref.iter_mut() {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, 12),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX && g < delta_u.len() {
                                    du_elem.data[i] = delta_u[g];
                                }
                            }
                            b.update_state(&du_elem, false, &ctx);
                        }
                        Ok(compute_f_int(model_ref, dofmap, behaviors_ref))
                    },
                    &prev_du,
                    arc_lambda,
                )
            };

            match result {
                Ok(step_result) if step_result.converged => {
                    // 要素状態は eval_fint で既に δu 反映済み。ここでは確定のみ。
                    for b in behaviors.iter_mut() {
                        b.commit_state();
                    }
                    for (&du, td) in step_result.du.iter().zip(total_disp.iter_mut()) {
                        *td += du;
                    }
                    arc_lambda += step_result.dlambda;
                    prev_du = step_result.du;

                    let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                    let f_int_now = compute_f_int(model, dofmap, &behaviors);
                    let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                    let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                    capacity_curve.push(CapacityPoint {
                        step: (n_steps + 1 + _step) as u32,
                        roof_disp: roof,
                        base_shear,
                        story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                        story_drift: story_drift.clone(),
                    });
                    steps.push(PushoverStep {
                        // 弧長法: 各増分後に更新される荷重倍率 arc_lambda をそのまま記録する。
                        load_factor: arc_lambda,
                        top_disp: roof,
                        base_shear,
                        story_drifts: story_drift,
                        node_disp: record_node_disp.then(|| total_disp.clone()),
                    });
                }
                _ => {
                    model.restore(&snap, &mut behaviors);
                    break;
                }
            }
        }
    }

    let mechanism = determine_mechanism(&hinges, model);
    // 保有水平耐力 Qu = 性能曲線上の最大ベースシア（崩壊機構形成時の水平耐力）。
    // 単調載荷では機構形成後に頭打ちとなるため、ピーク値を採る。
    let qu = capacity_curve
        .iter()
        .map(|c| c.base_shear)
        .fold(0.0_f64, f64::max);
    Ok(PushoverResult {
        steps,
        capacity_curve,
        hinges,
        shear_yields,
        mechanism,
        qu,
    })
}

struct HingeThreshold {
    mc: f64,
    my: f64,
    mu: f64,
}

fn compute_hinge_thresholds(model: &Model) -> Vec<HingeThreshold> {
    model
        .elements
        .iter()
        .map(|elem| {
            let (my, mu) = if let Some(sid) = elem.section {
                if let Some(sec) = model.sections.get(sid.index()) {
                    let depth = sec.depth.max(sec.width);
                    let i = sec.iz.max(sec.iy);
                    let z = if depth > 0.0 { i / (depth / 2.0) } else { 0.0 };
                    // 降伏応力は部材材料の fy を優先。未設定なら鋼材既定 235 N/mm²（SN400 級）。
                    let sigma_y = elem
                        .material
                        .and_then(|mid| model.materials.get(mid.index()))
                        .and_then(|m| m.fy)
                        .unwrap_or(235.0);
                    let my = sigma_y * z;
                    (my, my * 1.2)
                } else {
                    (0.0, 0.0)
                }
            } else {
                (0.0, 0.0)
            };
            HingeThreshold {
                mc: my / 3.0,
                my,
                mu,
            }
        })
        .collect()
}

fn track_hinges(
    model: &Model,
    _dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[HingeThreshold],
    step: u32,
    hinges: &mut Vec<HingeEvent>,
) {
    let state = ElemState::default();
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        let f = b.internal_force(&state, &ctx);
        let m_i = f.data[4].abs().max(f.data[5].abs());
        let m_j = f.data[10].abs().max(f.data[11].abs());
        let m_max = m_i.max(m_j);
        let th = &thresholds[i];
        if m_max < th.mc {
            continue;
        }
        let level = if m_max >= th.mu {
            HingeLevel::Ultimate
        } else if m_max >= th.my {
            HingeLevel::Yield
        } else {
            HingeLevel::Crack
        };
        let pos = if m_i >= m_j { 0.0 } else { 1.0 };
        hinges.push(HingeEvent {
            step,
            elem: elem.id,
            pos,
            level,
            ductility: if th.my > 0.0 { m_max / th.my } else { 0.0 },
        });
    }
}

/// せん断降伏耐力 Qy の判定しきい値（部材ごと、局所 y・z 方向、独立）。
///
/// `y` は局所 y 方向せん断力 Vy（弱軸曲げに伴う、`Section.as_y`）、
/// `z` は局所 z 方向せん断力 Vz（強軸曲げに伴う、`Section.as_z`）に対する
/// しきい値であり、`track_shear_yield` で Vy vs `y.qy(..)`・Vz vs `z.qy(..)` を
/// 独立に判定する（v1 のような「合力 vs min(qy_y,qy_z)」の丸めは行わない）。
/// RC矩形（[`DirThreshold::RcArakawa`]）方向は、各ステップの部材軸力（圧縮）
/// から動的に σ0 を反映した Qy を都度算定する（精緻化2、`track_shear_yield` 参照）。
struct ShearThreshold {
    y: DirThreshold,
    z: DirThreshold,
}

/// せん断降伏耐力 Qy の算定方式（方向別）。
///
/// `Static` は解析開始時に一度だけ算定される軸力非依存のしきい値（鋼系、または
/// 配筋情報が無い／算定不能な RC のフォールバック）。`RcArakawa` は RC矩形
/// （`SectionShape::RcRect`）の荒川mean式系の略算式で、σ0 を除く入力一式を
/// 保持しておき、各ステップの軸力から求めた σ0 で上書きして
/// [`rc_qsu_simple`] を呼び直す（精緻化2）。
enum DirThreshold {
    Static(f64),
    RcArakawa {
        /// σ0 抜きの入力一式（`sigma_0` は常に 0.0 のプレースホルダ。
        /// [`DirThreshold::qy`] が呼び出しのたびに軸力由来の値へ差し替える）。
        input: RcCapacityInput,
        /// 全断面積 [mm²]（= b・D。方向によらず同一値。σ0 = 圧縮軸力/gross_area
        /// の算定に用いる）。
        gross_area: f64,
    },
}

impl DirThreshold {
    /// 圧縮軸力 `n_compress`（[N]、0 以上。引張は呼び出し側で 0 として渡す
    /// 規約、`axial_compression` 参照）から Qy [N] を求める。
    ///
    /// `Static` は軸力によらず一定値。`RcArakawa` は σ0 = n_compress/gross_area
    /// （荒川式の適用範囲 0〜0.4Fc へのクランプは [`rc_qsu_simple`] 内で行う）を
    /// 反映した Qsu を都度算定する。
    fn qy(&self, n_compress: f64) -> f64 {
        match self {
            DirThreshold::Static(v) => *v,
            DirThreshold::RcArakawa { input, gross_area } => {
                let sigma_0 = if *gross_area > 0.0 {
                    n_compress / gross_area
                } else {
                    0.0
                };
                let mut inp = *input;
                inp.sigma_0 = sigma_0;
                rc_qsu_simple(&inp)
            }
        }
    }
}

/// せん断降伏耐力 Qy 算定対象の方向（局所座標系）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShearDir {
    /// 局所 y 方向（弱軸曲げに伴うせん断、`Section.as_y`・`RcRebar.main_y` 対応）。
    Y,
    /// 局所 z 方向（強軸曲げに伴うせん断、`Section.as_z`・`RcRebar.main_x` 対応）。
    Z,
}

/// `SectionShape::RcRect` の配筋情報から、指定方向の荒川mean式系の略算式
/// （[`squid_n_core::rc_capacity::rc_qsu_simple`]）用入力一式を組み立てる。
/// σ0 は 0.0 のプレースホルダとし、[`DirThreshold::qy`] が各ステップの軸力から
/// 動的に上書きする（精緻化2。旧実装は σ0=0 固定の安全側簡略化だった）。
///
/// 変換規則は `squid-n-app::app::rc_capacity_input_from_rect` と同一の規約
/// （上下対称配筋を仮定・at=引張側総断面積の半分、σy=fy or 345、σwy=295 固定、
/// せん断補強筋は legs 組数を考慮）に合わせる:
/// - 強軸（局所 z 方向せん断、`dir=Z`）: b=幅, d=せい、引張鉄筋は `rebar.main_x`。
/// - 弱軸（局所 y 方向せん断、`dir=Y`）: b と d を入れ替え、引張鉄筋は `rebar.main_y`。
///
/// `clear_span`（h0）は [`effective_clear_span`] が剛域長を控除して算定した値を
/// 渡す（精緻化1。旧実装は剛域控除を省略し節点間長をそのまま用いる簡略化だった）。
/// `fc` 未設定の場合は None（呼び出し側で慣用値へフォールバックする）。
fn rc_rect_capacity_input(
    b: f64,
    d: f64,
    main: &BarSet,
    rebar: &RcRebar,
    mat: &Material,
    clear_span: f64,
) -> Option<RcCapacityInput> {
    let fc = mat.fc?;
    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    // 上下対称配筋を仮定し、引張側主筋量は総断面積の半分。
    let at = bar_area(main) / 2.0;
    let d_eff = d - rebar.cover - main.dia / 2.0;
    let shear_area =
        std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia * rebar.shear.legs as f64;
    let pw = if rebar.shear.pitch > 0.0 {
        shear_area / (b * rebar.shear.pitch)
    } else {
        0.0
    };
    Some(RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y: mat.fy.unwrap_or(345.0), // SD345 相当、要・原典照合
        fc,
        pw,
        sigma_wy: 295.0, // SD295 相当、要・原典照合
        clear_span,
        sigma_0: 0.0, // プレースホルダ。DirThreshold::qy が軸力から都度上書きする。
    })
}

/// 方向別のせん断降伏耐力しきい値（[`DirThreshold`]）を組み立てる。
///
/// RC矩形（`SectionShape::RcRect`）で `fy` が無く、配筋情報から Qsu(σ0=0) が
/// 算定可能（正の値）な場合のみ [`DirThreshold::RcArakawa`] を採用し、各ステップ
/// で軸力から動的算定した σ0 を反映する。それ以外（鋼系・配筋情報が無い／
/// 算定不能な RC・有効せん断断面積や材料情報が無い場合）は、解析開始時に一度だけ
/// 算定した [`DirThreshold::Static`] を用いる（採用式は下記）:
/// - 鋼系部材（材料に `fy` が設定されている）: Qy = as・fy / √3
///   （純せん断降伏条件 τy = fy/√3（von Mises）に有効せん断断面積を乗じた慣用式）。
/// - RC 系部材で `RcRect` 形状が無い、または Qsu 算定不能な場合: Qy = as・0.7√fc
///   （コンクリートのせん断終局強度に対する簡易慣用値。荒川式等の精算は行わない）。
/// - 有効せん断断面積 `as_area` が 0（未設定）、または材料・強度情報が無い場合は
///   判定対象外として Qy = +∞（その方向のせん断では耐力喪失を判定しない）。
fn build_dir_threshold(
    as_area: f64,
    material: Option<&Material>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> DirThreshold {
    if as_area <= 0.0 {
        return DirThreshold::Static(f64::INFINITY);
    }
    let Some(mat) = material else {
        return DirThreshold::Static(f64::INFINITY);
    };
    if let Some(fy) = mat.fy {
        return DirThreshold::Static(as_area * fy / 3.0_f64.sqrt());
    }
    let Some(fc) = mat.fc else {
        return DirThreshold::Static(f64::INFINITY);
    };
    if let Some(Section {
        shape: Some(SectionShape::RcRect { b, d, rebar }),
        ..
    }) = section
    {
        let input = match dir {
            ShearDir::Z => rc_rect_capacity_input(*b, *d, &rebar.main_x, rebar, mat, clear_span),
            ShearDir::Y => rc_rect_capacity_input(*d, *b, &rebar.main_y, rebar, mat, clear_span),
        };
        if let Some(input) = input {
            if rc_qsu_simple(&input) > 0.0 {
                return DirThreshold::RcArakawa {
                    gross_area: input.b * input.d,
                    input,
                };
            }
        }
    }
    DirThreshold::Static(as_area * 0.7 * fc.sqrt())
}

/// せん断降伏耐力 Qy [N] を算定する（RESP-D マニュアル計算編03「応力解析」
/// §段階的耐力喪失解析のせん断降伏判定に使用）。
///
/// 軸力なし（σ0=0）の静的評価。単体テスト・後方互換用の薄いラッパーで、
/// [`build_dir_threshold`] が返す [`DirThreshold`] を `n_compress=0` で評価する
/// ことと等価（実解析 `track_shear_yield` は各ステップの軸力から動的に σ0 を
/// 反映するため、本関数は呼ばない。テスト専用のため `#[cfg(test)]`）。
#[cfg(test)]
fn compute_shear_yield_qy(
    as_area: f64,
    material: Option<&Material>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> f64 {
    build_dir_threshold(as_area, material, section, dir, clear_span).qy(0.0)
}

/// 部材長（節点間距離）[mm]。節点参照が欠落・退化（長さ0）の場合は None。
/// RC のせん断降伏耐力算定における内法スパン h0 は、この節点間長から
/// [`effective_clear_span`] が剛域長を控除して求める（精緻化1）。
fn elem_length(model: &Model, elem: &ElementData) -> Option<f64> {
    if elem.nodes.len() < 2 {
        return None;
    }
    let pi = model.nodes.get(elem.nodes[0].index())?;
    let pj = model.nodes.get(elem.nodes[1].index())?;
    let dx = pj.coord[0] - pi.coord[0];
    let dy = pj.coord[1] - pi.coord[1];
    let dz = pj.coord[2] - pi.coord[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    (len > 0.0).then_some(len)
}

/// 剛域控除後の内法スパン h0 [mm]（荒川式のせん断スパン比算定に用いる、精緻化1）。
///
/// h0 = 節点間長（`raw_length`） − (`rigid_zone.length_i` + `rigid_zone.length_j`)。
/// 控除後が 0 以下（浮動小数点誤差により実質 0 とみなせる極小値を含む、
/// 1e-6mm 以下）になる異常な剛域指定（剛域長の入力誤りで節点間長を超過する等）
/// では、h0 を過小評価しないよう節点間長そのものへフォールバックする
/// （`rc_qsu_simple` 側でもせん断スパン比 h0/(2d_e) は 1.0〜3.0 にクランプされる
/// ため過大な Qsu には至らないが、異常値の握り潰しではなくフォールバックとして
/// 明示する）。
fn effective_clear_span(raw_length: f64, rigid_zone: &RigidZone) -> f64 {
    let net = raw_length - rigid_zone.length_i - rigid_zone.length_j;
    if net > 1e-6 {
        net
    } else {
        raw_length
    }
}

fn compute_shear_yield_thresholds(model: &Model) -> Vec<ShearThreshold> {
    model
        .elements
        .iter()
        .map(|elem| {
            let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
            let mat = elem
                .material
                .and_then(|mid| model.materials.get(mid.index()));
            let (as_y, as_z) = sec.map(|s| (s.as_y, s.as_z)).unwrap_or((0.0, 0.0));
            let raw_length = elem_length(model, elem).unwrap_or(0.0);
            let clear_span = effective_clear_span(raw_length, &elem.rigid_zone);
            ShearThreshold {
                y: build_dir_threshold(as_y, mat, sec, ShearDir::Y, clear_span),
                z: build_dir_threshold(as_z, mat, sec, ShearDir::Z, clear_span),
            }
        })
        .collect()
}

fn dot3(v: [f64; 3], w: [f64; 3]) -> f64 {
    v[0] * w[0] + v[1] * w[1] + v[2] * w[2]
}

/// 材端力（グローバル、i端 `f_i`・j端 `f_j`）と局所 `ex`（i→j 方向単位ベクトル、
/// グローバル成分）から、部材の軸方向圧縮力 N_compress [N]（圧縮のみ採用、
/// 引張は 0）を算定する（精緻化2、σ0 = N_compress/gross_area の入力）。
///
/// ## 符号規約（単純片持ち柱による検算）
/// 標準的なトラス/梁要素の軸剛性行列は局所座標で
/// `[[EA/L, -EA/L], [-EA/L, EA/L]]`（i端・j端の軸方向 DOF）であり、
/// 引張正のひずみ `eps0 = (u_j − u_i)/L` に対し軸力 `N = EA・eps0`（引張正）
/// を生じる。この行列を軸方向変位 `(u_i, u_j)` に適用すると、i端の局所x方向
/// 内力は `f_local_x(i) = -N`、j端は `f_local_x(j) = +N` となる
/// （`squid-n-element` の `FiberBeam`・`Beam` とも同一の規約。剛性行列／
/// B行列の符号から導出、要素実装のいずれでも一致）。
///
/// 具体例（節点 i=(0,0,0)・節点 j=(0,0,3000)、`ref_vector=[1,0,0]` の片持ち柱、
/// `LocalFrame::from_nodes` により `ex=[0,0,1]`）で軸圧縮を検算する: 柱頭（j端）
/// を Δ=-1mm だけ ex 方向と逆向き（縮む向き）に変位させると、局所x方向変位は
/// `u_i=0, u_j=dot(Δ,ex)=-1`。ひずみ `eps0=(u_j-u_i)/L=-1/L<0`（圧縮）となり
/// `N=EA・eps0<0`。よって `f_local_x(i)=-N>0`、`f_local_x(j)=N<0`。
///
/// `ElementBehavior::internal_force` はグローバル力を返す契約であり、
/// 局所x軸（`ex`）方向の内力成分はグローバル内力を `ex` へ射影すれば得られる
/// （`AxisTransform::rotate_to_local` の定義 `v_local[0]=dot(ex,v_global)` より）。
/// よって `dot(f_i, ex) = f_local_x(i) = -N`、`dot(f_j, ex) = f_local_x(j) = +N`。
///
/// 圧縮（N<0）成分のみ正の値として取り出すため、i端は `dot(f_i, ex)` を、
/// j端は `-dot(f_j, ex)` を、それぞれ 0 未満をクランプ（引張は 0 とみなす）して
/// 採用し、両端のうち大きい方を部材の代表圧縮力とする（安全側の丸めではなく
/// 実勢値を採る規約。プリズマティック部材で軸方向分布荷重が無ければ理論上
/// 両端は一致するが、数値誤差・分布荷重の影響を考慮し大きい方を採用する）。
fn axial_compression(f_i: [f64; 3], f_j: [f64; 3], ex: [f64; 3]) -> f64 {
    let from_i = dot3(f_i, ex).max(0.0);
    let from_j = (-dot3(f_j, ex)).max(0.0);
    from_i.max(from_j)
}

/// せん断降伏イベントの追跡（`track_hinges` と対をなす、曲げとは独立の判定）。
///
/// `ElementBehavior::internal_force` が返す材端節点力はグローバル座標成分
/// （`f.data[0..3]`＝i端, `f.data[6..9]`＝j端）である。要素の局所座標系
/// （`LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector)`、
/// `rot[0]=ex, rot[1]=ey, rot[2]=ez`）の `ey`・`ez` へ材端力を射影することで
/// 局所 Vy・Vz を厳密に分離し、Vy は `qy_y`、Vz は `qy_z` と独立に比較する
/// （v1 の「軸直交合力 vs min(qy_y,qy_z)」から改良）。各材端のうち大きい方を
/// 部材の代表値とし、Vy・Vz のいずれかがしきい値を超えた部材を、当該ステップの
/// せん断降伏イベントとして記録する。
///
/// ## 軸力 σ0 の動的反映（精緻化2）
/// Vy・Vz と同様に材端力を局所 `ex` へ射影し、[`axial_compression`] で部材の
/// 圧縮軸力（引張は 0、両端のうち大きい方を実勢値として採用）を求める。
/// RC矩形の [`DirThreshold::RcArakawa`] 方向は σ0 = 圧縮軸力/(b・D) として
/// [`DirThreshold::qy`] に渡し、`rc_qsu_simple` を呼び直して Qy を都度算定する。
/// 鋼系・フォールバック RC（[`DirThreshold::Static`]）はこの軸力を無視し、
/// 解析開始時の静的値をそのまま用いる。
fn track_shear_yield(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[ShearThreshold],
    step: u32,
    events: &mut Vec<ShearYieldEvent>,
) {
    let state = ElemState::default();
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        if elem.nodes.len() < 2 {
            continue;
        }
        let (Some(pi), Some(pj)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[1].index()),
        ) else {
            continue;
        };
        if elem_length(model, elem).is_none() {
            continue;
        }
        let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
        let ex = frame.rot[0];
        let ey = frame.rot[1];
        let ez = frame.rot[2];

        let f = b.internal_force(&state, &ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let f_j = [f.data[6], f.data[7], f.data[8]];
        let vy = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let vz = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let n_compress = axial_compression(f_i, f_j, ex);

        let th = &thresholds[i];
        let qy_y = th.y.qy(n_compress);
        let qy_z = th.z.qy(n_compress);
        if vy >= qy_y || vz >= qy_z {
            events.push(ShearYieldEvent {
                step,
                elem: elem.id,
            });
        }
    }
}

/// 部材端ヒンジが属する階を返す。ヒンジ位置側の節点 story を優先し、
/// 未割当（基礎節点など story=None）の場合は相手端の節点 story で補完する。
fn hinge_story(model: &Model, h: &HingeEvent) -> Option<StoryId> {
    let elem = model.elements.iter().find(|e| e.id == h.elem)?;
    if elem.nodes.len() < 2 {
        return None;
    }
    let (near, far) = if h.pos < 0.5 {
        (elem.nodes[0], elem.nodes[1])
    } else {
        (elem.nodes[1], elem.nodes[0])
    };
    model
        .nodes
        .get(near.index())
        .and_then(|n| n.story)
        .or_else(|| model.nodes.get(far.index()).and_then(|n| n.story))
}

/// 平面骨組の静的不静定次数 r = 3m − 3n + r_support を算出する（P5 §11.5）。
///
/// - m: 部材数（`model.elements.len()`）
/// - n: 節点数（`model.nodes.len()`）
/// - r_support: 各節点で拘束された平面 DoF (ux, uz, ry) の総数
///
/// 3D 6DOF モデルを pushover 方向の平面骨組と見なして次数を計算する。
/// 機構成立条件は `形成降伏ヒンジ数 >= r + 1`（運動学的判定）。
fn compute_static_indeterminacy(model: &Model) -> usize {
    let m = model.elements.len();
    let n = model.nodes.len();
    // 平面 DoF は ux(0), uz(2), ry(4)。各節点の Dof6Mask で拘束判定。
    let r_support: usize = model
        .nodes
        .iter()
        .map(|node| {
            let bits = node.restraint.0;
            let mut count = 0;
            if bits & (1u8 << 0) != 0 {
                count += 1;
            }
            if bits & (1u8 << 2) != 0 {
                count += 1;
            }
            if bits & (1u8 << 4) != 0 {
                count += 1;
            }
            count
        })
        .sum();
    (3 * m + r_support).saturating_sub(3 * n)
}

/// 崩壊機構の判定（P5 §7.4 / §11.5）。
///
/// 降伏以上（Yield/Ultimate）の塑性ヒンジのみを対象とし、運動学的機構成立判定
/// `形成降伏ヒンジ数 >= 静的不静定次数 + 1` でゲートした上で、階分布から機構種別を分類:
/// - 形成降伏ヒンジ数 < r + 1 → まだ機構未成立（Partial）
/// - 複数階モデルで降伏ヒンジが単一階に集中 → 層崩壊（StoryCollapse）
/// - それ以外（複数階に分布／単一階構造）→ 全体崩壊（Overall）
fn determine_mechanism(hinges: &[HingeEvent], model: &Model) -> MechanismType {
    use std::collections::{BTreeMap, BTreeSet};

    let yielded: Vec<&HingeEvent> = hinges
        .iter()
        .filter(|h| matches!(h.level, HingeLevel::Yield | HingeLevel::Ultimate))
        .collect();

    // 運動学的機構成立ゲート: 形成降伏ヒンジ数 >= r+1
    let distinct_ends: BTreeSet<(u32, u8)> = yielded
        .iter()
        .map(|h| (h.elem.index() as u32, if h.pos < 0.5 { 0u8 } else { 1u8 }))
        .collect();
    let r = compute_static_indeterminacy(model);
    if yielded.is_empty() || distinct_ends.len() < r + 1 {
        return MechanismType::Partial;
    }

    // 降伏ヒンジの階分布を集計。
    let mut per_story: BTreeMap<u32, usize> = BTreeMap::new();
    let mut story_ids: BTreeMap<u32, StoryId> = BTreeMap::new();
    let mut unmapped = 0usize;
    for h in &yielded {
        match hinge_story(model, h) {
            Some(s) => {
                *per_story.entry(s.index() as u32).or_default() += 1;
                story_ids.insert(s.index() as u32, s);
            }
            None => unmapped += 1,
        }
    }

    let n_model_stories = model.stories.len();
    if n_model_stories > 1 && per_story.len() == 1 && unmapped == 0 {
        // 単一階に塑性化が集中 → 層崩壊機構。
        let key = *per_story.keys().next().unwrap();
        MechanismType::StoryCollapse {
            story: story_ids[&key],
        }
    } else {
        // 複数階に分布、または単一階構造 → 全体崩壊機構。
        MechanismType::Overall
    }
}

fn get_roof_dof(model: &Model, dofmap: &DofMap, dir: SeismicDir) -> Option<usize> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    if let Some(story) = model.stories.last() {
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let g = ni * 6 + dir_idx;
            return dofmap.active(g).map(|a| a as usize);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint::Reducer;
    use squid_n_core::dof::{Dof6Mask, DofMap};
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        Constraint, DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
        Material, Node, Section, Story,
    };
    use squid_n_core::section_shape::ShearBar;

    /// 1層・鉛直ファイバ柱の片持ちプッシュオーバー（P5 §10 相当の最小統合テスト）。
    /// 配線済み非線形要素（FiberBeam）＋座標変換＋NR 反復＋降伏追跡が
    /// エンドツーエンドで動作することを検証する。
    fn single_column_model(fy: f64, seismic_weight: f64) -> Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    // FiberBeam はねじり剛性を持たないため、Z 軸柱の頂部ねじり DOF(rz=bit5)
                    // のみ拘束して特異性を除く。曲げ回転 rx,ry と並進は自由。
                    restraint: Dof6Mask(0b100000),
                    mass: None,
                    story: Some(StoryId(0)),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
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
            sections: vec![Section {
                id: SectionId(0),
                name: "col".to_string(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                fy: Some(fy),
            }],
            stories: vec![Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1)],
                diaphragms: vec![DiaphragmDef {
                    ci_override: None,
                    weight: None,
                    master: NodeId(1),
                    slaves: vec![],
                    rigid: true,
                }],
                seismic_weight: Some(seismic_weight),
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_pushover_single_column_forms_hinge() {
        // 降伏応力を低め、地震重量を降伏荷重をやや超える程度に設定し、
        // 柱脚に曲げヒンジが形成されることを確認する。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,    // max_steps
            0.0,   // max_disp（変位制御に移行しない＝荷重制御のみ）
            false, // use_kg
            false, // use_arc_length
            0.0,
        )
        .expect("pushover should run end-to-end");

        // パイプライン全体が収束ステップを生成していること。
        assert!(
            !result.capacity_curve.is_empty(),
            "capacity curve should have at least one converged step"
        );
        // 荷重−変位曲線の頂部変位は単調に正（水平押し）であること。
        let last = result.capacity_curve.last().unwrap();
        assert!(
            last.roof_disp > 0.0,
            "roof displacement should be positive: {}",
            last.roof_disp
        );
        // 降伏応力を与えた鋼材ファイバ柱で、柱脚に曲げヒンジが追跡されること
        //（座標変換＋ファイバ降伏＋降伏追跡のエンドツーエンド検証）。
        assert!(
            !result.hinges.is_empty(),
            "at least one hinge should form in the column under lateral push"
        );

        // steps は capacity_curve と同じ収束ステップ数だけ積まれること。
        assert_eq!(
            result.steps.len(),
            result.capacity_curve.len(),
            "steps should have one entry per capacity_curve point"
        );
        // 各 step の story_drifts は層数（本モデルは1層）と一致すること。
        for s in &result.steps {
            assert_eq!(
                s.story_drifts.len(),
                model.stories.len(),
                "story_drifts length should match number of stories"
            );
        }
    }

    #[test]
    fn test_pushover_requires_seismic_weight() {
        // 地震重量未定義ではエラーを返す（入力検証）。
        let mut model = single_column_model(235.0, 0.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            10,
            0.0,
            false,
            false,
            0.0,
        );
        assert!(
            result.is_err(),
            "should error when no seismic weight defined"
        );
    }

    #[test]
    fn test_pushover_arc_length_path_runs() {
        // 弧長法フェーズ（f_int 反復再評価版）がエンドツーエンドで動作すること。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            10,    // max_steps（荷重制御）
            0.0,   // max_disp
            false, // use_kg
            true,  // use_arc_length
            1.0,   // arc_length_dl [mm]
        )
        .expect("arc-length pushover should run end-to-end");
        assert!(!result.capacity_curve.is_empty());
        assert!(result.qu > 0.0);
    }

    /// determine_mechanism / hinge_story 用の2層・柱通り（基礎-1F-2F）モデル。
    /// node0=基礎(story None), node1=1F(story0), node2=2F(story1)。
    /// elem0=1F柱(0-1), elem1=2F柱(1-2)。
    fn two_story_model() -> Model {
        let sec = Section {
            id: SectionId(0),
            name: "c".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "s".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(235.0),
        };
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 6000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(1)),
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Fiber,
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
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Fiber,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
                },
            ],
            sections: vec![sec],
            materials: vec![mat],
            stories: vec![
                Story {
                    level_kind: Default::default(),
                    structure: Default::default(),
                    id: StoryId(0),
                    name: "1F".to_string(),
                    elevation: 3000.0,
                    node_ids: vec![NodeId(1)],
                    diaphragms: vec![],
                    seismic_weight: None,
                },
                Story {
                    level_kind: Default::default(),
                    structure: Default::default(),
                    id: StoryId(1),
                    name: "2F".to_string(),
                    elevation: 6000.0,
                    node_ids: vec![NodeId(2)],
                    diaphragms: vec![],
                    seismic_weight: None,
                },
            ],
            ..Default::default()
        }
    }

    fn hinge(elem: u32, pos: f64, level: HingeLevel) -> HingeEvent {
        HingeEvent {
            step: 0,
            elem: ElemId(elem),
            pos,
            level,
            ductility: 1.0,
        }
    }

    #[test]
    fn test_determine_mechanism_partial_when_insufficient() {
        let model = two_story_model();
        // ひび割れのみ → 降伏ヒンジ0個 < r+1 → Partial
        assert!(matches!(
            determine_mechanism(&[hinge(0, 0.0, HingeLevel::Crack)], &model),
            MechanismType::Partial
        ));
    }

    /// two_story_model は部材2・節点3・基礎FIXED(平面3DOF) → r=0（静定）。
    /// したがって降伏ヒンジ1個で運動学的機構成立（r+1=1）。単一階集中→層崩壊。
    #[test]
    fn test_determine_mechanism_single_yield_establishes_mechanism() {
        let model = two_story_model();
        // elem0 端 j (pos=1.0) → node1 = 1F 単独階 → 層崩壊
        match determine_mechanism(&[hinge(0, 1.0, HingeLevel::Yield)], &model) {
            MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
            other => panic!(
                "expected StoryCollapse{{0}}, got {:?}",
                std::mem::discriminant(&other)
            ),
        }
    }

    /// 静的不静定次数の計算検証（平面骨組: r = 3m − 3n + r_support）。
    #[test]
    fn test_compute_static_indeterminacy_two_story() {
        // 2層2柱: 部材2・節点3・基礎節点(node0)が平面3DOF拘束 → r = 6 - 9 + 3 = 0（静定）
        let model = two_story_model();
        assert_eq!(compute_static_indeterminacy(&model), 0);
    }

    #[test]
    fn test_compute_static_indeterminacy_indeterminate_portal() {
        // 1層1スパン両端固定ラーメン: 柱2+梁1=部材3、節点4（基礎2点FIXED+上部2点FREE）
        // r = 3*3 - 3*4 + (3+3) = 9 - 12 + 6 = 3（3次不静定）
        let model = two_story_model(); // 共用せず簡易生成
        let _ = model; // unused warning 回避
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(2),
                coord: [5000.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
            },
            Node {
                id: NodeId(3),
                coord: [5000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
        ];
        let elems = vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
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
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
            },
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
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
            },
        ];
        let portal = Model {
            nodes,
            elements: elems,
            sections: vec![Section {
                id: SectionId(0),
                name: "c".to_string(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "s".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                fy: Some(235.0),
            }],
            stories: vec![Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1), NodeId(2)],
                diaphragms: vec![],
                seismic_weight: None,
            }],
            ..Default::default()
        };
        assert_eq!(compute_static_indeterminacy(&portal), 3);
    }

    #[test]
    fn test_determine_mechanism_story_collapse() {
        let model = two_story_model();
        // 1F柱の両端（elem0 pos1.0 → node1=1F, elem1 pos0.0 → node1=1F）が降伏
        // → 降伏ヒンジが1F(story0)に集中 → 層崩壊
        let hinges = vec![
            hinge(0, 1.0, HingeLevel::Yield),
            hinge(1, 0.0, HingeLevel::Yield),
        ];
        match determine_mechanism(&hinges, &model) {
            MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
            other => panic!(
                "expected StoryCollapse{{0}}, got {:?}",
                std::mem::discriminant(&other)
            ),
        }
    }

    #[test]
    fn test_determine_mechanism_overall() {
        let model = two_story_model();
        // 1F(story0)と2F(story1)に分散して降伏 → 全体崩壊
        let hinges = vec![
            hinge(0, 1.0, HingeLevel::Yield), // node1 = 1F
            hinge(1, 1.0, HingeLevel::Yield), // node2 = 2F
        ];
        assert!(matches!(
            determine_mechanism(&hinges, &model),
            MechanismType::Overall
        ));
    }

    #[test]
    fn test_pushover_base_shear_is_real_force() {
        // 最初の（弾性）ステップで base_shear/roof_disp が片持ち柱の弾性剛性
        // 3EI/L³ ≈ 189.8 N/mm に一致することを確認（DOF添字加算の旧バグを排除）。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,
            0.0,
            false,
            false,
            0.0,
        )
        .unwrap();
        let first = result.capacity_curve.first().unwrap();
        assert!(first.roof_disp > 0.0 && first.base_shear > 0.0);
        let k = first.base_shear / first.roof_disp;
        assert!(
            (150.0..=230.0).contains(&k),
            "first-step stiffness base_shear/roof_disp={k} should be ~3EI/L^3≈189.8"
        );
        // Qu はピークベースシア（全点以上）であること。
        for c in &result.capacity_curve {
            assert!(
                result.qu >= c.base_shear - 1e-6,
                "qu {} must be >= {}",
                result.qu,
                c.base_shear
            );
        }
        assert!(result.qu > 0.0);
    }

    fn portal_frame_model(fy: f64, seismic_weight: f64) -> Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    // FiberBeam はねじり剛性を持たないため Rz を拘束
                    restraint: Dof6Mask(0b100000),
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(2),
                    coord: [5000.0, 0.0, 3000.0],
                    restraint: Dof6Mask(0b100000),
                    mass: None,
                    story: Some(StoryId(0)),
                },
                Node {
                    id: NodeId(3),
                    coord: [5000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Fiber,
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
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Fiber,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
                },
                ElementData {
                    id: ElemId(2),
                    kind: ElementKind::Fiber,
                    nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
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
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "col".to_string(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                fy: Some(fy),
            }],
            stories: vec![Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1), NodeId(2)],
                diaphragms: vec![DiaphragmDef {
                    ci_override: None,
                    weight: None,
                    master: NodeId(1),
                    slaves: vec![NodeId(2)],
                    rigid: true,
                }],
                seismic_weight: Some(seismic_weight),
            }],
            constraints: vec![Constraint::RigidDiaphragm {
                story: StoryId(0),
                master: NodeId(1),
                slaves: vec![NodeId(2)],
            }],
            ..Default::default()
        }
    }

    // 1層1スパン剛床ラーメン（門形フレーム）で崩壊荷重が手計算値（4・My/H_col）
    // に一致し、柱両端に4つの塑性ヒンジが形成され全体機構となることを検証する（P5 §10.1）。
    //
    // 手計算: Z=I/(depth/2)=166,660, My=σ_y·Z, Qu=4My/H=52,220 N（柱両端降伏・2柱）。
    // seismic_weight は崩壊荷重を上回る値に設定し、真に降伏到達させる。
    #[test]
    fn test_portal_frame_collapse_load() {
        let qu_theory: f64 = 52_220.0;
        let mut model = portal_frame_model(235.0, 600_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            80,
            0.0,
            false,
            false,
            0.0,
        )
        .expect("pushover should run end-to-end");

        // 柱両端の降伏ヒンジが実際に形成されていること（運動学的機構: r+1=4）。
        let yielded_hinges = result
            .hinges
            .iter()
            .filter(|h| !matches!(h.level, HingeLevel::Crack))
            .count();
        assert!(
            yielded_hinges >= 4,
            "at least 4 yielded hinges expected for Overall mechanism, got {} (total hinges={})",
            yielded_hinges,
            result.hinges.len()
        );

        // 崩壊機構が成立していること（Partial でない）。
        assert!(
            !matches!(result.mechanism, MechanismType::Partial),
            "mechanism should not be Partial for a collapsed portal frame"
        );

        assert!(result.qu > 0.0, "qu should be positive, got {}", result.qu);

        // 4番目の降伏ヒンジ（柱両端×2本＝4個で運動学的機構成立）発生ステップの
        // ベースシアを「観測崩壊荷重」とする（qu=max(base_shear) はまだ弾性最大反力で
        // plateau を正確に捉えられないため、降伏到達点で照合する）。
        let mut yield_steps: Vec<u32> = result
            .hinges
            .iter()
            .filter(|h| !matches!(h.level, HingeLevel::Crack))
            .map(|h| h.step)
            .collect();
        yield_steps.sort_unstable();
        yield_steps.dedup();
        assert!(
            yield_steps.len() >= 4,
            "need >=4 distinct yield steps for Overall mechanism, got {}: {:?}",
            yield_steps.len(),
            yield_steps
        );
        let mech_step = yield_steps[3];
        let qu_observed = result
            .capacity_curve
            .iter()
            .find(|c| c.step == mech_step)
            .map(|c| c.base_shear)
            .unwrap_or(0.0);
        let rel_err = (qu_observed - qu_theory).abs() / qu_theory;
        // pushover は段階改良途上のため、比較的広めの許容差（30%）を設ける。
        assert!(
            rel_err < 0.30,
            "observed_qu={} at step {} deviates from Qu_theory={} by {:.1}% (>30%)",
            qu_observed,
            mech_step,
            qu_theory,
            rel_err * 100.0
        );
    }

    #[test]
    fn test_portal_frame_mechanism_classified() {
        let mut model = portal_frame_model(235.0, 600_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            80,
            0.0,
            false,
            false,
            0.0,
        )
        .expect("pushover should run end-to-end");

        match &result.mechanism {
            MechanismType::Overall | MechanismType::StoryCollapse { .. } => {}
            other => panic!(
                "expected Overall or StoryCollapse, got {:?}",
                std::mem::discriminant(other)
            ),
        }
    }

    // ---- せん断降伏耐力 Qy の単体テスト ----

    #[test]
    fn test_compute_shear_yield_qy_steel() {
        // 鋼系（fy 設定あり）: Qy = as・fy/√3（RcRect 形状の有無・方向によらない）。
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "s".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(200.0),
        };
        let qy = compute_shear_yield_qy(1000.0, Some(&mat), None, ShearDir::Z, 3000.0);
        let expected = 1000.0 * 200.0 / 3.0_f64.sqrt();
        assert!(
            (qy - expected).abs() < 1e-6,
            "qy={qy} should equal as*fy/sqrt(3)={expected}"
        );
    }

    #[test]
    fn test_compute_shear_yield_qy_rc_fallback_without_rc_rect_shape() {
        // RC系（fy 無し・fc 設定あり）かつ断面形状情報（RcRect）が無い場合:
        // Qy = as・0.7√fc（慣用値へフォールバック）。
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "rc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let qy = compute_shear_yield_qy(50000.0, Some(&mat), None, ShearDir::Z, 3000.0);
        let expected = 50000.0 * 0.7 * 24.0_f64.sqrt();
        assert!(
            (qy - expected).abs() < 1e-6,
            "qy={qy} should equal as*0.7*sqrt(fc)={expected}"
        );
    }

    #[test]
    fn test_compute_shear_yield_qy_zero_as_is_infinite() {
        // 有効せん断断面積が 0 の断面は判定対象外（Qy=∞扱い）。
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "s".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(200.0),
        };
        assert_eq!(
            compute_shear_yield_qy(0.0, Some(&mat), None, ShearDir::Z, 3000.0),
            f64::INFINITY
        );
        // 材料未設定でも∞扱い。
        assert_eq!(
            compute_shear_yield_qy(1000.0, None, None, ShearDir::Z, 3000.0),
            f64::INFINITY
        );
    }

    /// RC 矩形断面（`SectionShape::RcRect`）+ 配筋情報がある場合、Qy は荒川式
    /// （`rc_qsu_simple`）による方向別算定値に一致すること。
    /// z 方向（強軸・main_x）、y 方向（弱軸・main_y、b/d 入れ替え）の双方を検証する。
    #[test]
    fn test_compute_shear_yield_qy_rc_rect_matches_arakawa_handcalc() {
        let rebar = RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 25.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        };
        let (b, d) = (400.0, 600.0);
        let shape = SectionShape::RcRect {
            b,
            d,
            rebar: rebar.clone(),
        };
        let sec = shape.to_section(SectionId(0), "RC-400x600".into());
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "rc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let clear_span = 3000.0;

        // z 方向（強軸）: b=幅, d=せい, 引張鉄筋 main_x。
        let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
        let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
            b,
            d,
            at: bar_area(&rebar.main_x) / 2.0,
            d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
            sigma_wy: 295.0,
            clear_span,
            sigma_0: 0.0,
        });
        let qy_z =
            compute_shear_yield_qy(sec.as_z, Some(&mat), Some(&sec), ShearDir::Z, clear_span);
        assert!(
            (qy_z - qsu_z_handcalc).abs() < 1e-6,
            "qy_z={qy_z} should equal rc_qsu_simple handcalc={qsu_z_handcalc}"
        );

        // y 方向（弱軸）: b と d を入れ替え、引張鉄筋 main_y。
        let qsu_y_handcalc = rc_qsu_simple(&RcCapacityInput {
            b: d,
            d: b,
            at: bar_area(&rebar.main_y) / 2.0,
            d_eff: b - rebar.cover - rebar.main_y.dia / 2.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (d * 100.0),
            sigma_wy: 295.0,
            clear_span,
            sigma_0: 0.0,
        });
        let qy_y =
            compute_shear_yield_qy(sec.as_y, Some(&mat), Some(&sec), ShearDir::Y, clear_span);
        assert!(
            (qy_y - qsu_y_handcalc).abs() < 1e-6,
            "qy_y={qy_y} should equal rc_qsu_simple handcalc={qsu_y_handcalc}"
        );
        // 断面が非正方形（b≠d、主筋も非対称）なので z・y の Qy は異なるはず。
        assert!((qy_z - qy_y).abs() > 1.0, "qy_z={qy_z} qy_y={qy_y}");
    }

    /// as_y/as_z を明示的に与えた片持ち柱モデル（`single_column_model` のせん断有効
    /// 断面積を差し替えたもの）。せん断降伏耐力 Qy は as_y/as_z と材料強度のみに
    /// 依存し、実際に生じるせん断力（`track_shear_yield`）は材端力の釣合いから
    /// 求まるため、せん断バネ剛性（材料のせん断弾性係数）を変更する必要はない。
    fn single_column_model_with_shear(fy: f64, seismic_weight: f64, as_shear: f64) -> Model {
        let mut model = single_column_model(fy, seismic_weight);
        model.sections[0].as_y = as_shear;
        model.sections[0].as_z = as_shear;
        model
    }

    #[test]
    fn test_pushover_shear_yield_event_recorded() {
        // せん断有効断面積を小さく設定してせん断降伏耐力 Qy を小さくすることで、
        // 水平荷重漸増中にせん断降伏イベントが記録されることを確認する
        // （曲げヒンジ判定 `track_hinges` とは独立の判定経路の検証）。
        let mut model = single_column_model_with_shear(235.0, 80_000.0, 50.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,
            0.0,
            false,
            false,
            0.0,
        )
        .expect("pushover should run end-to-end");

        assert!(
            !result.shear_yields.is_empty(),
            "shear yield event should be recorded when Qy is small relative to applied shear"
        );
    }

    /// as_y・as_z を独立に設定した片持ち柱モデル（局所 y・z 方向分離の検証用）。
    fn single_column_model_with_shear_yz(
        fy: f64,
        seismic_weight: f64,
        as_y: f64,
        as_z: f64,
    ) -> Model {
        let mut model = single_column_model(fy, seismic_weight);
        model.sections[0].as_y = as_y;
        model.sections[0].as_z = as_z;
        model
    }

    /// `single_column_model` は節点 (0,0,0)→(0,0,3000)、`local_axis.ref_vector=[1,0,0]`
    /// なので局所座標系は ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（`LocalFrame::from_nodes`）。
    /// `SeismicDir::X` でプッシュすると力はグローバル X＝局所 y（ey）方向に生じ、
    /// 局所 z（ez＝グローバル Y）方向にはほぼ生じない。
    fn run_pushover_has_shear_yield(as_y: f64, as_z: f64) -> bool {
        let mut model = single_column_model_with_shear_yz(235.0, 80_000.0, as_y, as_z);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,
            0.0,
            false,
            false,
            0.0,
        )
        .expect("pushover should run end-to-end");
        !result.shear_yields.is_empty()
    }

    /// 局所 y/z 方向の厳密分離（改良1）の検証:
    /// 実際に力が生じる方向（局所 y）の Qy を小さくすればせん断降伏イベントが
    /// 記録されるが、力がほぼ生じない方向（局所 z）の Qy をどれだけ小さくしても
    /// 記録されないこと。v1（軸直交合力 vs min(qy_y,qy_z)）では後者でも
    /// 誤って記録されてしまっていた（qy_z が min を支配してしまうため）。
    #[test]
    fn test_pushover_shear_yield_direction_independent() {
        assert!(
            run_pushover_has_shear_yield(50.0, 1.0e12),
            "small as_y (the actually-stressed local direction) should trigger a shear yield event"
        );
        assert!(
            !run_pushover_has_shear_yield(1.0e12, 50.0),
            "small as_z (the unstressed local direction) should NOT trigger a shear yield event \
             once Vy/Vz are judged independently against qy_y/qy_z"
        );
    }

    // ---- 精緻化1: h0 への剛域控除の単体テスト ----

    #[test]
    fn test_effective_clear_span_deducts_rigid_zone_lengths() {
        let rz = RigidZone {
            length_i: 500.0,
            length_j: 300.0,
            ..Default::default()
        };
        // h0 = 節点間長3000 − (500+300) = 2200。
        assert!((effective_clear_span(3000.0, &rz) - 2200.0).abs() < 1e-9);
    }

    #[test]
    fn test_effective_clear_span_falls_back_when_non_positive() {
        // 剛域長の合計が節点間長を超える異常入力 → 節点間長へフォールバック。
        let rz_over = RigidZone {
            length_i: 2000.0,
            length_j: 1500.0,
            ..Default::default()
        };
        assert_eq!(effective_clear_span(3000.0, &rz_over), 3000.0);

        // ちょうど0（または極小の浮動小数点誤差域）でもフォールバック。
        let rz_zero = RigidZone {
            length_i: 1500.0,
            length_j: 1500.0,
            ..Default::default()
        };
        assert_eq!(effective_clear_span(3000.0, &rz_zero), 3000.0);
    }

    /// RC矩形断面 + 配筋情報を持つ要素モデル（剛域テスト共通）。
    /// 節点間距離3000mm、`rigid_zone` は呼び出し側で差し替える。
    fn rc_column_model_with_rigid_zone(rigid_zone: RigidZone) -> (Model, RcRebar, f64, f64) {
        let rebar = RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 25.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        };
        let (b, d) = (400.0, 600.0);
        let shape = SectionShape::RcRect {
            b,
            d,
            rebar: rebar.clone(),
        };
        let sec = shape.to_section(SectionId(0), "RC-400x600".into());
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "rc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone,
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![sec],
            materials: vec![mat],
            ..Default::default()
        };
        (model, rebar, b, d)
    }

    #[test]
    fn test_compute_shear_yield_thresholds_rc_rect_uses_rigid_zone_reduced_clear_span() {
        // 剛域: length_i=400, length_j=200 → h0 = 3000-600 = 2400。
        let rigid_zone = RigidZone {
            length_i: 400.0,
            length_j: 200.0,
            ..Default::default()
        };
        let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
        let thresholds = compute_shear_yield_thresholds(&model);
        let th = &thresholds[0];

        let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
        let expected_clear_span = 2400.0;

        // z方向（強軸・main_x）: RcArakawa を採用し、h0=2400 での rc_qsu_simple 手計算に一致。
        let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
            b,
            d,
            at: bar_area(&rebar.main_x) / 2.0,
            d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
            sigma_wy: 295.0,
            clear_span: expected_clear_span,
            sigma_0: 0.0,
        });
        match &th.z {
            DirThreshold::RcArakawa { input, gross_area } => {
                assert!(
                    (input.clear_span - expected_clear_span).abs() < 1e-9,
                    "clear_span={} expected={}",
                    input.clear_span,
                    expected_clear_span
                );
                assert!((gross_area - b * d).abs() < 1e-9);
            }
            DirThreshold::Static(_) => panic!("expected RcArakawa for RcRect with rebar"),
        }
        assert!(
            (th.z.qy(0.0) - qsu_z_handcalc).abs() < 1e-6,
            "qy(0.0)={} handcalc={}",
            th.z.qy(0.0),
            qsu_z_handcalc
        );
    }

    #[test]
    fn test_compute_shear_yield_thresholds_rc_rect_falls_back_when_rigid_zone_exceeds_length() {
        // 剛域長の合計(2000+1500=3500)が節点間長(3000)を超える異常入力
        // → h0 は節点間長3000へフォールバックする。
        let rigid_zone = RigidZone {
            length_i: 2000.0,
            length_j: 1500.0,
            ..Default::default()
        };
        let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
        let thresholds = compute_shear_yield_thresholds(&model);
        let th = &thresholds[0];

        let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
        let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
            b,
            d,
            at: bar_area(&rebar.main_x) / 2.0,
            d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
            sigma_wy: 295.0,
            clear_span: 3000.0, // フォールバック後の値
            sigma_0: 0.0,
        });
        assert!((th.z.qy(0.0) - qsu_z_handcalc).abs() < 1e-6);
    }

    // ---- 精緻化2: 軸力σ0の動的反映の単体テスト ----

    #[test]
    fn test_dir_threshold_qy_axial_term_matches_handcalc() {
        // rc_capacity::tests::sample_input と同一の断面（b=400,D=600,pw=0.002等）で
        // DirThreshold::RcArakawa を直接構成し、圧縮軸力からの σ0 反映を検算する。
        let b = 400.0;
        let d = 600.0;
        let d_eff = 530.0;
        let input = RcCapacityInput {
            b,
            d,
            at: 1935.0,
            d_eff,
            sigma_y: 345.0,
            fc: 24.0,
            pw: 0.002,
            sigma_wy: 295.0,
            clear_span: 3000.0,
            sigma_0: 0.0, // プレースホルダ（qy() が上書きする）
        };
        let gross_area = b * d;
        let th = DirThreshold::RcArakawa { input, gross_area };

        let qy_base = th.qy(0.0);
        let qsu_base_handcalc = rc_qsu_simple(&input);
        assert!((qy_base - qsu_base_handcalc).abs() < 1e-6);

        // 圧縮軸力 N_compress = 5.0 * gross_area → σ0 = 5.0 [N/mm²]（適用範囲0〜0.4Fc=9.6内）。
        let sigma_0 = 5.0;
        let n_compress = sigma_0 * gross_area;
        let qy_with_axial = th.qy(n_compress);
        let j = 7.0 * d_eff / 8.0;
        let expected_delta = 0.1 * sigma_0 * b * j;
        assert!(
            (qy_with_axial - qy_base - expected_delta).abs() < 1e-6,
            "delta={} expected={}",
            qy_with_axial - qy_base,
            expected_delta
        );

        // 引張（n_compress=0、呼び出し側で既にクランプ済みの規約）は σ0=0 のまま、
        // Qy は base と一致（増えない）。
        assert!((th.qy(0.0) - qy_base).abs() < 1e-9);
    }

    /// 軸力符号規約の検算（単純片持ち柱、節点 i=(0,0,0)・j=(0,0,3000)、
    /// `ref_vector=[1,0,0]` → `LocalFrame::from_nodes` により ex=[0,0,1]）。
    ///
    /// 柱頭（j端）を Δ=-1mm（ex と逆向き、圧縮方向）変位させたときの内力を
    /// 手計算（f_local_x(i)=-N>0, f_local_x(j)=N<0、doc `axial_compression` 参照）
    /// で再現し、`axial_compression` がこの圧縮を正しく検出することを確認する。
    #[test]
    fn test_axial_compression_sign_convention_handcalc() {
        let ex = [0.0, 0.0, 1.0];
        // 圧縮（N<0、|N|=1000）: f_i はコンプレッション側 = +|N|・ex、f_j = -|N|・ex。
        let n_compress_mag = 1000.0;
        let f_i_comp = [0.0, 0.0, n_compress_mag];
        let f_j_comp = [0.0, 0.0, -n_compress_mag];
        assert!(
            (axial_compression(f_i_comp, f_j_comp, ex) - n_compress_mag).abs() < 1e-9,
            "compression should be detected as a positive n_compress"
        );

        // 引張（N>0）: 圧縮側の符号が反転 → axial_compression は 0（圧縮なし）。
        let f_i_tension = [0.0, 0.0, -n_compress_mag];
        let f_j_tension = [0.0, 0.0, n_compress_mag];
        assert_eq!(
            axial_compression(f_i_tension, f_j_tension, ex),
            0.0,
            "pure tension must not be treated as compression (sigma_0=0 for tension)"
        );

        // 片端のみ圧縮成分がある非対称ケース（数値誤差や分布荷重を模擬）:
        // 両端のうち大きい方（実勢値）を採用する。
        let f_i_asym = [0.0, 0.0, n_compress_mag];
        let f_j_asym = [0.0, 0.0, -0.5 * n_compress_mag];
        assert!(
            (axial_compression(f_i_asym, f_j_asym, ex) - n_compress_mag).abs() < 1e-9,
            "should take the larger of the two end-derived compression values"
        );
    }

    /// `ElementBehavior::internal_force` が固定のグローバル材端力を返すだけのテスト
    /// スタブ（`track_shear_yield` は `global_dofs`/剛性を使わないため他は無関係）。
    struct FixedForceBehavior {
        f: LocalVec,
    }

    impl ElementBehavior for FixedForceBehavior {
        fn n_dof(&self) -> usize {
            12
        }
        fn global_dofs(&self, _dof: &DofMap) -> SmallVec<[usize; 24]> {
            SmallVec::new()
        }
        fn tangent_stiffness(
            &self,
            _state: &ElemState,
            _ctx: &Ctx,
        ) -> squid_n_element::behavior::LocalMat {
            squid_n_element::behavior::LocalMat::zeros(12)
        }
        fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
            LocalVec {
                data: self.f.data.clone(),
            }
        }
        fn mass_matrix(
            &self,
            _opt: squid_n_element::behavior::MassOption,
        ) -> squid_n_element::behavior::LocalMat {
            squid_n_element::behavior::LocalMat::zeros(12)
        }
    }

    /// 精緻化2のエンドツーエンド確認: 同一のせん断力 Vz デマンドに対し、
    /// 軸圧縮が作用する場合は σ0 反映で Qy が増え判定を免れるが、圧縮が無い
    /// （引張・軸力ゼロ）場合は従来どおり判定に掛かることを、実際の
    /// `track_shear_yield` を通して確認する（`compute_shear_yield_thresholds` の
    /// 構築から一貫して検証）。
    #[test]
    fn test_track_shear_yield_axial_compression_raises_qy_end_to_end() {
        let (model, _rebar, b, d) = rc_column_model_with_rigid_zone(RigidZone::default());
        let thresholds = compute_shear_yield_thresholds(&model);
        let (input, gross_area) = match &thresholds[0].z {
            DirThreshold::RcArakawa { input, gross_area } => (*input, *gross_area),
            DirThreshold::Static(_) => panic!("expected RcArakawa"),
        };
        assert!((gross_area - b * d).abs() < 1e-6);

        let qy_base = rc_qsu_simple(&input);
        let sigma_0 = 5.0; // 0〜0.4Fc=9.6 の範囲内
        let n_compress = sigma_0 * gross_area;
        let mut inp_axial = input;
        inp_axial.sigma_0 = sigma_0;
        let qy_boosted = rc_qsu_simple(&inp_axial);
        assert!(qy_boosted > qy_base, "axial term should raise Qy");

        // Vz を base と boosted のちょうど中間に設定: base では降伏、boosted では非降伏。
        // モデルは node i=(0,0,0)・j=(0,0,3000)、ref_vector=[1,0,0] のため
        // ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（既存テストの局所座標系規約と同じ）。
        // よって Vz は global y 成分（f.data[1]/f.data[7]）、N は global z 成分
        // （f.data[2]/f.data[8]）に対応する。
        let vz_demand = (qy_base + qy_boosted) / 2.0;

        // ケースA: 軸圧縮あり（N_compress = sigma_0*gross_area）→ 判定を免れるはず。
        let f_comp = LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                vz_demand,
                n_compress,
                0.0,
                0.0,
                0.0,
                0.0,
                -vz_demand,
                -n_compress,
                0.0,
                0.0,
                0.0,
            ]),
        };
        let behaviors_comp: Vec<Box<dyn ElementBehavior>> =
            vec![Box::new(FixedForceBehavior { f: f_comp })];
        let mut events_comp = Vec::new();
        track_shear_yield(&model, &behaviors_comp, &thresholds, 0, &mut events_comp);
        assert!(
            events_comp.is_empty(),
            "compression should raise Qy above the shear demand, suppressing the event"
        );

        // ケースB: 軸力なし（同じ Vz デマンド）→ 従来どおり判定に掛かるはず。
        let f_zero = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, vz_demand, 0.0, 0.0, 0.0, 0.0, 0.0, -vz_demand, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        let behaviors_zero: Vec<Box<dyn ElementBehavior>> =
            vec![Box::new(FixedForceBehavior { f: f_zero })];
        let mut events_zero = Vec::new();
        track_shear_yield(&model, &behaviors_zero, &thresholds, 0, &mut events_zero);
        assert!(
            !events_zero.is_empty(),
            "without axial compression the same Vz demand should still trigger the event"
        );
    }
}
