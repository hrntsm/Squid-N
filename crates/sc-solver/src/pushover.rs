use crate::analysis::SeismicDir;
use crate::arc_length::ArcLengthSolver;
use crate::constraint::Reducer;
use crate::transaction::{StateSnapshot, StatefulModel};
use sc_core::dof::DofMap;
use sc_core::ids::{ElemId, StoryId};
use sc_core::model::Model;
use sc_element::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
use sc_element::factory::build_behavior;
use sc_math::solver::{make_solver, SolverBackend};
use smallvec::SmallVec;

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

fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    prescribed: Option<(usize, f64)>,
) -> faer::sparse::SparseColMat<usize, f64> {
    use sc_math::sparse::assemble_csc;
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
        triplets.push(sc_math::sparse::Triplet {
            row: d,
            col: d,
            val: penalty,
        });
    }
    assemble_csc(dofmap.n_active(), triplets)
}

fn compute_f_int(
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
        let (b, _) = build_behavior(elem, model);
        behaviors.push(b);
    }

    let stories = &model.stories;
    if stories.is_empty() {
        return Err("no stories defined".into());
    }
    let height_m = stories.last().map(|s| s.elevation).unwrap_or(0.0) / 1000.0;
    let t = sc_load::ai::approx_t(height_m, 0.0);
    let z = 1.0;
    let tc = sc_load::ai::tc_of(sc_load::ai::SoilClass::II);
    let rt_val = sc_load::ai::rt(t, tc);
    let c0 = 0.2;
    let story_weights: Vec<f64> = stories
        .iter()
        .map(|s| s.seismic_weight.unwrap_or(0.0))
        .collect();
    if story_weights.iter().all(|&w| w == 0.0) {
        return Err("no seismic weight defined".into());
    }
    let ai = sc_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

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
        for dia in &story.diaphragms {
            let ni = dia.master.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                if let Some(a) = dofmap.active(g) {
                    q[a as usize] += dir_vec[d] * pi;
                }
            }
        }
    }

    let thresholds = compute_hinge_thresholds(model);
    let mut hinges = Vec::new();
    let mut capacity_curve = Vec::new();
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
                let base_shear: f64 = q.iter().map(|&qi| qi * current_lambda).sum();
                capacity_curve.push(CapacityPoint {
                    step: step as u32,
                    roof_disp: roof,
                    base_shear,
                    story_shear: vec![],
                    story_drift: vec![],
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
                        let base_shear: f64 = compute_f_int(model, dofmap, &behaviors)
                            .iter()
                            .take(roof_active)
                            .sum();
                        capacity_curve.push(CapacityPoint {
                            step: (n_steps + 1 + step) as u32,
                            roof_disp: roof,
                            base_shear,
                            story_shear: vec![],
                            story_drift: vec![],
                        });
                        track_hinges(
                            model,
                            dofmap,
                            &behaviors,
                            &thresholds,
                            (n_steps + step) as u32,
                            &mut hinges,
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
            let f_int = compute_f_int(model, dofmap, &behaviors);

            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            if solver.factorize(&k_red).is_err() {
                model.restore(&snap, &mut behaviors);
                break;
            }

            let result = arc_solver.step(
                &q,
                &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                    let r_red = reducer.reduce_f(r);
                    let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                    Ok(reducer.expand_u(&du_red))
                },
                &f_int,
                &prev_du,
                arc_lambda,
            );

            match result {
                Ok(step_result) if step_result.converged => {
                    let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
                    for (_elem, b) in model.elements.iter_mut().zip(behaviors.iter_mut()) {
                        let gdofs = b.global_dofs(dofmap);
                        let mut du_elem = LocalVec {
                            data: SmallVec::from_elem(0.0, 12),
                        };
                        for (i, &g) in gdofs.iter().enumerate() {
                            if g != usize::MAX && g < step_result.du.len() {
                                du_elem.data[i] = step_result.du[g];
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
                    for (&du, td) in step_result.du.iter().zip(total_disp.iter_mut()) {
                        *td += du;
                    }
                    arc_lambda += step_result.dlambda;
                    prev_du = step_result.du;

                    let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                    capacity_curve.push(CapacityPoint {
                        step: (n_steps + 1 + _step) as u32,
                        roof_disp: roof,
                        base_shear: arc_lambda,
                        story_shear: vec![],
                        story_drift: vec![],
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
    let qu = capacity_curve.last().map(|c| c.base_shear).unwrap_or(0.0);
    Ok(PushoverResult {
        steps: vec![],
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
                    let sigma_y = 235.0;
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

fn determine_mechanism(hinges: &[HingeEvent], _model: &Model) -> MechanismType {
    if hinges.is_empty() {
        return MechanismType::Partial;
    }
    let last_step = hinges.iter().map(|h| h.step).max().unwrap_or(0);
    let final_hinges: Vec<&HingeEvent> = hinges.iter().filter(|h| h.step == last_step).collect();
    if final_hinges.len() >= 3 {
        MechanismType::Overall
    } else {
        MechanismType::Partial
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
