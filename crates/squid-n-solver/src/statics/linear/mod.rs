use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{ElementData, ElementKind, LoadCaseKind, Model};
use squid_n_element::beam::MemberForces;
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use std::borrow::Cow;

/// 長期軸力無効化（RESP-D 計算編03「応力解析」）で断面積に乗じる縮小係数。
/// 完全にゼロにすると（ブレースのみで支持される節点等で）浮き自由度による
/// 特異行列を招く恐れがあるため、実務上無視できる微小軸剛性を残す
/// （EA×1e-6 は元の軸力の 1e-6 倍程度に留まり回収内力もほぼ0とみなせる）。
const AXIAL_DISABLE_FACTOR: f64 = 1.0e-6;

/// 部材が「柱」（鉛直な `ElementKind::Beam`）かどうかを判定する。
/// 判定規則は `squid-n-design-jp::eccentricity::column_stiffnesses` の柱判定
/// （部材軸単位ベクトルの ez 成分 |ez|>0.707）に合わせる。
fn is_vertical_column(elem: &ElementData, model: &Model) -> bool {
    if !matches!(elem.kind, ElementKind::Beam) || elem.nodes.len() < 2 {
        return false;
    }
    let (Some(n0), Some(n1)) = (
        model.nodes.get(elem.nodes[0].index()),
        model.nodes.get(elem.nodes[1].index()),
    ) else {
        return false;
    };
    let dx = n1.coord[0] - n0.coord[0];
    let dy = n1.coord[1] - n0.coord[1];
    let dz = n1.coord[2] - n0.coord[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return false;
    }
    (dz / len).abs() > 0.707
}

/// 長期応力解析で軸力を負担させない部材（対象: ブレース／柱）かどうかを、
/// `Model::stress_cfg` の指定に基づいて判定する。
fn is_axial_disabled_target(
    elem: &ElementData,
    model: &Model,
    cfg: &squid_n_core::model::StressAnalysisCfg,
) -> bool {
    match elem.kind {
        ElementKind::Brace { .. } => cfg.no_long_axial_brace,
        ElementKind::Beam => cfg.no_long_axial_column && is_vertical_column(elem, model),
        _ => false,
    }
}

/// 長期応力解析の計算条件（RESP-D 計算編03「応力解析」）を適用したモデルを返す。
///
/// 対象荷重ケースが長期系（`LoadCaseKind::is_long_term`）かつ `stress_cfg` で
/// 軸力無効化が指定されている部材がある場合のみ、対象部材が参照する断面を
/// 複製して断面積を `AXIAL_DISABLE_FACTOR` 倍に縮小したモデルを作る
/// （同じ断面 ID を共有する他部材へは影響しない）。曲げ・せん断・ねじり
/// 関連の断面性能は変更しない。対象が無ければ元のモデルをそのまま返す
/// （既定 `stress_cfg` では常にこちら＝従来どおりの結果に一致する）。
///
/// SRC/CFT 等の合成断面では `beam.rs` の軸剛性用面積 `a_stiff` が `shape` 由来の
/// 値で再計算されるため、複製断面では `shape` を外して数値直入力断面へ落とす。
/// これにより曲げ・せん断は `to_section()` が格納済みの等価換算値のまま、
/// 軸剛性のみ `area × AXIAL_DISABLE_FACTOR` が効く（材料由来の複合換算・
/// スラブ協力幅係数は複製断面では適用されなくなるが、軸力を負担させない
/// 部材の曲げ剛性の微差であり実用上支障ない）。
fn apply_long_axial_cut(model: &Model, lc_kind: LoadCaseKind) -> Cow<'_, Model> {
    let cfg = &model.stress_cfg;
    if !lc_kind.is_long_term() || (!cfg.no_long_axial_brace && !cfg.no_long_axial_column) {
        return Cow::Borrowed(model);
    }

    let targets: Vec<usize> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| is_axial_disabled_target(e, model, cfg) && e.section.is_some())
        .map(|(i, _)| i)
        .collect();
    if targets.is_empty() {
        return Cow::Borrowed(model);
    }

    let mut m = model.clone();
    for i in targets {
        let Some(sid) = m.elements[i].section else {
            continue;
        };
        let Some(orig) = m.sections.get(sid.index()) else {
            continue;
        };
        let mut reduced = orig.clone();
        reduced.area *= AXIAL_DISABLE_FACTOR;
        // 合成断面（SRC/CFT）でも軸剛性カットが効くよう shape を外す（関数 doc 参照）。
        reduced.shape = None;
        reduced.id = squid_n_core::ids::SectionId(m.sections.len() as u32);
        m.elements[i].section = Some(reduced.id);
        m.sections.push(reduced);
    }
    Cow::Owned(m)
}

pub struct StaticOnce {
    pub disp: Vec<[f64; 6]>,
    pub member_forces: Vec<(squid_n_core::ids::ElemId, MemberForces)>,
}

pub fn linear_static_once(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    faer::set_global_parallelism(faer::Par::Seq);
    let lc_kind = model
        .load_cases
        .iter()
        .find(|l| l.id == lc)
        .map(|l| l.kind)
        .unwrap_or_default();
    let model_cow = apply_long_axial_cut(model, lc_kind);
    let model: &Model = &model_cow;
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        });
    }

    let k_free = assemble_global_k(model, &dofmap);
    let f_free = assemble_global_f(model, &dofmap, lc);

    let reducer = Reducer::build(model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let f_red = reducer.reduce_f(&f_free);
    let n_indep = reducer.n_indep;

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    if n_indep > 0 {
        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; model.nodes.len()];
        for ni in 0..model.nodes.len() {
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    let val = u_free[active as usize];
                    match d {
                        0 => disp[ni][0] = val,
                        1 => disp[ni][1] = val,
                        2 => disp[ni][2] = val,
                        3 => disp[ni][3] = val,
                        4 => disp[ni][4] = val,
                        _ => disp[ni][5] = val,
                    }
                }
            }
        }

        let mut member_forces = Vec::new();
        let _ctx = squid_n_element::behavior::Ctx { model };
        // 解析対象荷重ケースの部材荷重（内力回復の重ね合わせ用）
        let member_loads: &[squid_n_core::model::MemberLoad] = model
            .load_cases
            .iter()
            .find(|l| l.id == lc)
            .map(|l| l.member.as_slice())
            .unwrap_or(&[]);
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let n_gdofs = gdofs.len();
            let mut u_elem = vec![0.0; n_gdofs];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }

            if let Some(mut forces) = behavior.recover_forces(&u_elem) {
                superpose_member_loads(model, elem, member_loads, &mut forces);
                member_forces.push((elem.id, forces));
            }
        }

        Ok(StaticOnce {
            disp,
            member_forces,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        })
    }
}

/// 部材荷重の固定端内力を、`K·u` 由来の回復内力へ各断面で重ね合わせる。
/// 線形重ね合わせ: 実内力 = （等価節点力に対する応答 K·u）＋（両端固定梁のスパン内力）。
fn superpose_member_loads(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    member_loads: &[squid_n_core::model::MemberLoad],
    forces: &mut squid_n_element::beam::MemberForces,
) {
    use squid_n_element::transform::LocalFrame;

    if elem.nodes.len() < 2 {
        return;
    }
    let loads: Vec<squid_n_core::model::MemberLoad> = member_loads
        .iter()
        .filter(|ml| ml.elem == elem.id)
        .cloned()
        .collect();
    if loads.is_empty() {
        return;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    if ni >= model.nodes.len() || nj >= model.nodes.len() {
        return;
    }
    let p_i = model.nodes[ni].coord;
    let p_j = model.nodes[nj].coord;
    let dx = p_j[0] - p_i[0];
    let dy = p_j[1] - p_i[1];
    let dz = p_j[2] - p_i[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
    if length < 1e-9 {
        return;
    }
    let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
    for (xi, vals) in forces.at.iter_mut() {
        let fixed = squid_n_element::member_load::fixed_internal_local(&loads, &frame, length, *xi);
        for k in 0..6 {
            vals[k] += fixed[k];
        }
    }
}

#[cfg(test)]
mod tests;
