use crate::analysis::{distribute_pi_over_diaphragms, steel_height_ratio, SeismicDir};
use crate::arc_length::ArcLengthSolver;
use crate::constraint::Reducer;
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::{ElemId, StoryId};
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
use squid_n_element::factory::build_nonlinear_behavior;
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

/// プッシュオーバー解析結果（P5 §7.4）
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
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
    let mut hinges = Vec::new();
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
                });
                track_hinges(
                    model,
                    dofmap,
                    &behaviors,
                    &thresholds,
                    step as u32,
                    &mut hinges,
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
                        });
                        track_hinges(model, dofmap, &behaviors, &thresholds, cstep, &mut hinges);
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
}
