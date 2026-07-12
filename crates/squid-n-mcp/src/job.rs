//! ジョブ実行（線形静的・固有値・プッシュオーバー・時刻歴・断面算定）関数。

use super::*;

/// `analysis_run` の任意パラメータの解決後の値（`AnalysisRunArgs` から変換する）。
/// 既定値は GUI (`squid_n_app::app::AnalysisSettings`) の既定に合わせる。
/// ただし `duration` は GUI 既定の 10.0 秒だと MCP 経由の応答待ちが長くなるため、
/// 動作確認がしやすい 2.0 秒を既定とする（呼び出し側で明示すれば変更可）。
#[derive(Debug, Clone, Copy)]
pub struct JobParams {
    /// LinearStatic/DesignCheck: 対象荷重ケース ID（未指定なら先頭ケース）。
    pub load_case: Option<u32>,
    /// Eigen: モード数。
    pub n_modes: usize,
    /// Pushover/TimeHistory: 加力・入力方向。
    pub dir: JobDir,
    /// Pushover: 最大ステップ数。
    pub steps: usize,
    /// Pushover: 目標変位 [mm]。
    pub max_disp: f64,
    /// TimeHistory: サンプル波の時間刻み [s]。
    pub dt: f64,
    /// TimeHistory: サンプル波の継続時間 [s]。
    pub duration: f64,
    /// TimeHistory: サンプル波の周期 [s]。
    pub period: f64,
    /// TimeHistory: サンプル波の振幅 [mm/s²]。
    pub amp: f64,
}

impl Default for JobParams {
    fn default() -> Self {
        Self {
            load_case: None,
            n_modes: 3,
            dir: JobDir::X,
            steps: 50,
            max_disp: 500.0,
            dt: 0.01,
            duration: 2.0,
            period: 0.5,
            amp: 1000.0,
        }
    }
}

/// Pushover/TimeHistory の方向（"X"/"Y"）。X+Y 同時入力（GUI の `ThDir::Xy`）は
/// MCP 経由では対応しない（仕様どおり "X"/"Y" のみ）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobDir {
    X,
    Y,
}

/// 各 JobKind の compute 結果。結果ストアへ書くべき生データ（あれば）とサマリ
/// （`JobStatus::Done::result_ref` に格納する JSON）の両方を保持する。
/// ストアへの書き込みは `persist_job_outcome` が担う（`ServerState` のロック内で
/// 呼ぶ必要があるため、compute 側とは分離している）。
pub enum JobOutcome {
    LinearStatic {
        case: u32,
        node_ids: Vec<u32>,
        disp: Vec<[f64; 6]>,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
    Eigen {
        period: Vec<f64>,
        omega2: Vec<f64>,
        participation: Vec<[f64; 3]>,
        effective_mass: Vec<[f64; 3]>,
        summary: serde_json::Value,
    },
    Pushover {
        summary: serde_json::Value,
    },
    TimeHistory {
        summary: serde_json::Value,
    },
    DesignCheck {
        case: u32,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
    UltimateCheck {
        summary: serde_json::Value,
    },
}

/// `kind` に応じて対応する compute_* 関数へ振り分ける。
pub fn compute_job(model: &Model, kind: JobKind, params: &JobParams) -> Result<JobOutcome, String> {
    match kind {
        JobKind::LinearStatic => compute_linear_static_job(model, params.load_case),
        JobKind::Eigen => compute_eigen_job(model, params.n_modes),
        JobKind::Pushover => {
            compute_pushover_job(model.clone(), params.dir, params.steps, params.max_disp)
        }
        JobKind::TimeHistory => compute_time_history_job(
            model,
            params.dir,
            params.dt,
            params.duration,
            params.period,
            params.amp,
        ),
        JobKind::DesignCheck => compute_design_check_job(model, params.load_case),
        JobKind::UltimateCheck => compute_ultimate_check_job(model, params.load_case),
    }
}

/// `load_case` 指定があればそれを、無ければ先頭の荷重ケースを返す。
/// 荷重ケースが1つも無いモデルでは "no load cases" を返す
/// （既存の `analyze_model` と同じ文言。P8 のテストが this を確認している）。
fn resolve_load_case(
    model: &Model,
    load_case: Option<u32>,
) -> Result<&squid_n_core::model::LoadCase, String> {
    match load_case {
        Some(id) => model
            .load_cases
            .iter()
            .find(|c| c.id.0 == id)
            .ok_or_else(|| format!("荷重ケース {id} が存在しません")),
        None => model
            .load_cases
            .first()
            .ok_or_else(|| "no load cases".to_string()),
    }
}

/// LinearStatic ジョブの純粋計算部分。
fn compute_linear_static_job(model: &Model, load_case: Option<u32>) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let node_ids: Vec<u32> = model.nodes.iter().map(|n| n.id.0).collect();
    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }
    }
    let max_abs_disp = result
        .disp
        .iter()
        .flat_map(|d| d.iter())
        .fold(0.0_f64, |m, v| m.max(v.abs()));

    let summary = serde_json::json!({
        "kind": "LinearStatic",
        "case": lc_id,
        "n_nodes": node_ids.len(),
        "n_member_force_rows": member_force_rows.len(),
        "max_abs_disp": max_abs_disp,
    });
    Ok(JobOutcome::LinearStatic {
        case: lc_id,
        node_ids,
        disp: result.disp,
        member_force_rows,
        summary,
    })
}

/// Eigen ジョブの純粋計算部分。
fn compute_eigen_job(model: &Model, n_modes: usize) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let modal = analysis
        .eigen(n_modes)
        .map_err(|e| format!("eigen failed: {e}"))?;
    let summary = serde_json::json!({
        "kind": "Eigen",
        "n_modes": modal.period.len(),
        "period": modal.period,
    });
    Ok(JobOutcome::Eigen {
        period: modal.period,
        omega2: modal.omega2,
        participation: modal.participation,
        effective_mass: modal.effective_mass,
        summary,
    })
}

/// Pushover ジョブの純粋計算部分。
/// squid-n-app の `App::compute_pushover`（app.rs）と同じ流れ
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
/// モデルは所有権を取って複製したものを渡す前提
/// （プッシュオーバーは非線形状態を模型に書き戻すため）。
fn compute_pushover_job(
    model: Model,
    dir: JobDir,
    steps: usize,
    max_disp: f64,
) -> Result<JobOutcome, String> {
    let mut work = model;
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut work,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    squid_n_solver::analysis::Analysis::prepare(&work)
        .map_err(|e| format!("解析準備エラー: {e}"))?;
    let dofmap = squid_n_core::dof::DofMap::build(&work);
    let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
    let seismic_dir = match dir {
        JobDir::X => squid_n_solver::analysis::SeismicDir::X,
        JobDir::Y => squid_n_solver::analysis::SeismicDir::Y,
    };
    let result = squid_n_solver::pushover::pushover_analysis(
        &mut work,
        &dofmap,
        &reducer,
        seismic_dir,
        steps,
        max_disp,
        false,
        false,
        0.0,
    )
    .map_err(|e| format!("プッシュオーバー解析エラー: {e}"))?;

    let mechanism = match result.mechanism {
        squid_n_solver::pushover::MechanismType::Overall => "Overall".to_string(),
        squid_n_solver::pushover::MechanismType::StoryCollapse { story } => {
            format!("StoryCollapse(story={})", story.0)
        }
        squid_n_solver::pushover::MechanismType::Partial => "Partial".to_string(),
    };
    // qu は N 単位（squid_n_solver::pushover::PushoverResult）。GUI(app.rs/summary.rs)と
    // 同様に kN 表示にするため /1000.0 する。
    let summary = serde_json::json!({
        "kind": "Pushover",
        "qu_kN": result.qu / 1000.0,
        "mechanism": mechanism,
        "n_steps": result.steps.len(),
    });
    Ok(JobOutcome::Pushover { summary })
}

/// TimeHistory ジョブの純粋計算部分。
/// サンプル波の生成式は squid-n-app の `App::sample_wave`/`build_ground_motion`
/// （app.rs）と同一（squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 減衰は剛性比例減衰 h=0.02（1次固有円振動数を使用）固定
/// （`App::compute_time_history` の `ThDampingModel::StiffnessProportional` 経路と同じ）。
fn compute_time_history_job(
    model: &Model,
    dir: JobDir,
    dt: f64,
    duration: f64,
    period: f64,
    amp: f64,
) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("解析準備エラー: {e}"))?;

    let n = ((duration / dt).ceil() as usize).max(2);
    let omega = 2.0 * std::f64::consts::PI / period.max(1e-6);
    let accel: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            amp * (omega * t).sin() * (-0.3 * t).exp()
        })
        .collect();
    let wave = match dir {
        JobDir::X => squid_n_solver::timehistory::GroundMotion {
            dt,
            accel_x: accel,
            accel_y: None,
        },
        JobDir::Y => {
            let n = accel.len();
            squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: vec![0.0; n],
                accel_y: Some(accel),
            }
        }
    };

    let omega1 = match analysis.eigen(1) {
        Ok(modal) => match modal.omega2.first() {
            Some(&w2) if w2 > 0.0 => w2.sqrt(),
            _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
        },
        Err(e) => return Err(format!("固有値解析エラー: {e}")),
    };
    let damping = squid_n_solver::damping::Damping::StiffnessProportional {
        h: 0.02,
        omega: omega1,
        basis: squid_n_solver::damping::StiffnessKind::Initial,
    };
    let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
    let result = analysis
        .time_history(&wave, newmark, damping)
        .map_err(|e| format!("時刻歴解析エラー: {e}"))?;

    let peak_disp = result
        .history
        .node_disp
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let summary = serde_json::json!({
        "kind": "TimeHistory",
        "peak_disp": peak_disp,
        "record_dir_y": result.history.record_dir_y,
        "n_steps": result.time.len(),
    });
    Ok(JobOutcome::TimeHistory { summary })
}

/// 鋼材判定（Material.name が JIS 鋼種名で始まるか。鉄筋 SD/SR は RC 扱い）。
/// squid-n-app の `is_steel`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn is_steel(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
        || upper.starts_with("SA")
        || upper.starts_with("BC")
}

/// 部材種別判定（部材軸の鉛直成分による幾何判定）。
/// squid-n-app の `member_kind_of`（app.rs）と同じロジック。
fn member_kind_of(
    elem: &squid_n_core::model::ElementData,
    model: &Model,
) -> squid_n_design_jp::MemberKind {
    use squid_n_design_jp::MemberKind;
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (dz / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 \[mm\]（内法補正なしの簡易値。剛域等は考慮しない）。
/// squid-n-app の `elem_geometric_length`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn elem_geometric_length(elem: &squid_n_core::model::ElementData, model: &Model) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 危険断面位置（§6.2.3、既定は柱フェイスと中央）を正規化座標 \[0,1\] で算定する。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` 算定と同じ規則
/// （xi_i は \[0.0, 0.5) へ、xi_j は (0.5, 1.0\] へクランプ）で face_i/face_j から
/// 求める。face=0（直交材が無い端）では節点芯（0.0/1.0）と一致する。
/// squid-n-app の `design_positions`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn design_positions(elem: &squid_n_core::model::ElementData, geom_len: f64) -> [f64; 3] {
    if geom_len > 1e-12 {
        let xi_i = (elem.rigid_zone.face_i / geom_len).clamp(0.0, 0.5 - 1e-9);
        let xi_j = (1.0 - elem.rigid_zone.face_j / geom_len).clamp(0.5 + 1e-9, 1.0);
        [xi_i, 0.5, xi_j]
    } else {
        [0.0, 0.5, 1.0]
    }
}

/// `pos` が `positions` のいずれかと 1e-6 以内で一致するか判定する。
fn is_near_design_position(pos: f64, positions: &[f64; 3]) -> bool {
    positions.iter().any(|p| (p - pos).abs() < 1e-6)
}

/// DesignCheck ジョブの純粋計算部分。
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// squid-n-app の `App::run_design_check`（app.rs）と同じ判定
/// （材料名先頭文字で鋼/RC を判定し SteelDesign/RcDesign を適用）を行う
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 検定条件（長期/短期）は既定で長期（`LoadTerm::Long`）とする。
fn compute_design_check_job(model: &Model, load_case: Option<u32>) -> Result<JobOutcome, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    // face_i/face_j は危険断面位置（§6.2.3）の算定にも使う。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    let mut n_checks = 0usize;
    let mut n_ng = 0usize;
    let mut max_ratio = 0.0_f64;

    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }

        let Some(elem) = model.elements.iter().find(|e| e.id == *elem_id) else {
            continue;
        };
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let mat = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid));
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };

        // 部材種別・部材長・せん断スパン比代表値（app.rs の run_design_check と同じ規則）。
        let kind = member_kind_of(elem, model);
        let length = {
            let coords: Vec<[f64; 3]> = elem
                .nodes
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .map(|n| n.coord)
                .take(2)
                .collect();
            match (coords.first(), coords.get(1)) {
                (Some(p0), Some(p1)) => {
                    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
                    (dx * dx + dy * dy + dz * dz).sqrt()
                }
                _ => 0.0,
            }
        };
        let shear_span = mf
            .at
            .iter()
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // 端部・中央の強軸曲げ（横座屈 C 係数・たわみ検定用）。
        let m_at = |target: f64| {
            mf.at
                .iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| f[5])
        };
        let end_moments_z = match (m_at(0.0), m_at(1.0)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        };
        // 柱の座屈長さ lk = K・h（app.rs run_design_check と同じ規則）。
        let lk = if kind == squid_n_design_jp::MemberKind::Column {
            squid_n_design_jp::steel::buckling::steel_column_k(model, elem).map(|k| k * length)
        } else {
            None
        };
        // S 造部材の断面検定属性（欠損率・横座屈長さ）。単一ケース・長期検定の
        // ため seismic_qd（地震時 QD 割増）は常に None。
        let steel_attr = model
            .steel_design_attrs
            .iter()
            .find(|a| a.elem == *elem_id)
            .cloned();
        let ctx = squid_n_design_jp::DesignCtx {
            term: squid_n_design_jp::LoadTerm::Long,
            kind,
            length,
            lb: None,
            lk,
            shear_span,
            rc_damage_control: true,
            end_moments_z,
            mid_moment_z: m_at(0.5),
            seismic_qd: None,
            steel_attr,
        };

        // 検定器の選択: 複合断面（SRC/CFT）は形状優先、それ以外は材料名で鋼/RC
        // （app.rs の run_design_check と同じ規則）。
        let checker: Box<dyn squid_n_design_jp::DesignCheck> = match sec.shape {
            Some(squid_n_core::section_shape::SectionShape::SrcRect { .. }) => {
                Box::new(squid_n_design_jp::SrcDesign)
            }
            Some(squid_n_core::section_shape::SectionShape::CftBox { .. })
            | Some(squid_n_core::section_shape::SectionShape::CftPipe { .. }) => {
                Box::new(squid_n_design_jp::CftDesign)
            }
            _ if is_steel(&mat.name) => Box::new(squid_n_design_jp::SteelDesign),
            _ => Box::new(squid_n_design_jp::RcDesign),
        };
        // 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力のみ検定する。
        // 節点芯は剛域が有る場合は検定対象外（app.rs の run_design_check と同じ規則）。
        let geom_len = elem_geometric_length(elem, model);
        let positions = design_positions(elem, geom_len);
        for (pos, forces) in &mf.at {
            if !is_near_design_position(*pos, &positions) {
                continue;
            }
            // [N, Qy, Qz, Mx, My, Mz] -> MemberForcesAt（N は引張正の部材内力）
            let mfa = squid_n_design_jp::MemberForcesAt {
                pos: *pos,
                n: forces[0],
                qy: forces[1],
                qz: forces[2],
                my: forces[4],
                mz: forces[5],
            };
            // BRB 属性が登録された部材はメーカー許容値による BRB 検定に差し替える
            // （app.rs run_design_check と同じ規則）。
            let cr = if let Some(brb) = model.brb_attrs.iter().find(|a| a.elem == *elem_id) {
                squid_n_design_jp::brb::brb_check(brb, mfa.n, length, true)
            } else {
                checker.check(&mfa, sec, mat, &ctx)
            };
            n_checks += 1;
            if !cr.ok {
                n_ng += 1;
            }
            if cr.ratio > max_ratio {
                max_ratio = cr.ratio;
            }
        }
    }

    // 節点単位の検定（RC 柱梁接合部・S パネルゾーン・冷間成形耐力比・耐震壁）。
    let mf_slices: Vec<(
        squid_n_core::ids::ElemId,
        squid_n_design_jp::joint_wiring::ForcesAt,
    )> = result
        .member_forces
        .iter()
        .map(|(id, mf)| (*id, mf.at.as_slice()))
        .collect();
    // PCa 水平接合面の検定（PcaBeamAttr が登録された梁のみ。単一ケース＝長期扱い）。
    for (_, _, cr) in
        squid_n_design_jp::rc::horizontal_joint::collect_pca_checks(model, &mf_slices, true)
    {
        n_checks += 1;
        if !cr.ok {
            n_ng += 1;
        }
        if cr.ratio > max_ratio {
            max_ratio = cr.ratio;
        }
    }
    let joint_checks = squid_n_design_jp::joint_wiring::collect_joint_checks(
        model,
        &mf_slices,
        squid_n_design_jp::LoadTerm::Long,
    );
    let n_joint_checks = joint_checks.len();
    let n_joint_ng = joint_checks.iter().filter(|(_, _, cr)| !cr.ok).count();
    for (_, _, cr) in &joint_checks {
        if cr.ratio > max_ratio {
            max_ratio = cr.ratio;
        }
    }

    let summary = serde_json::json!({
        "kind": "DesignCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        "n_joint_checks": n_joint_checks,
        "n_joint_ng": n_joint_ng,
        "max_ratio": max_ratio,
    });
    Ok(JobOutcome::DesignCheck {
        case: lc_id,
        member_force_rows,
        summary,
    })
}

/// 終局検定ジョブ（RESP-D「06 終局検定」）。RC 矩形部材の塑性理論式による
/// 終局せん断強度 Qsu・付着割裂耐力 Qbu・軸終局耐力に対する余裕度を算定する。
///
/// 柱の曲げ終局強度 Mu・軸余裕度に用いる設計軸力は、`load_case`（未指定なら
/// 先頭ケース＝長期相当）の線形静的解析の軸力（圧縮正）を用いる。
fn compute_ultimate_check_job(model: &Model, load_case: Option<u32>) -> Result<JobOutcome, String> {
    // 剛域（face_i/j）を内法長さに反映するため自動剛域を適用（冪等）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    // 部材需要（軸力[圧縮正、始端]・強軸/弱軸の設計用曲げ[部材内最大絶対値]）。
    let demand: Vec<(
        squid_n_core::ids::ElemId,
        squid_n_design_jp::ultimate::MemberDemand,
    )> = result
        .member_forces
        .iter()
        .filter_map(|(id, mf)| {
            let n_axial = mf.at.first().map(|(_, f)| f[0])?;
            let mz = mf.at.iter().map(|(_, f)| f[5].abs()).fold(0.0, f64::max);
            let my = mf.at.iter().map(|(_, f)| f[4].abs()).fold(0.0, f64::max);
            Some((
                *id,
                squid_n_design_jp::ultimate::MemberDemand {
                    n_axial,
                    mz,
                    my,
                    ..Default::default()
                },
            ))
        })
        .collect();
    // CFT の軸終局検定は軸力のみを用いる。
    let axial: Vec<(squid_n_core::ids::ElemId, f64)> =
        demand.iter().map(|(id, d)| (*id, d.n_axial)).collect();

    let opts = squid_n_design_jp::ultimate::UltimateShearOptions::default();
    let checks = squid_n_design_jp::ultimate::collect_rc_ultimate_checks(model, &demand, &opts);
    // CFT 柱の軸終局検定も同時に算定する。
    let cft_checks = squid_n_design_jp::ultimate::collect_cft_ultimate_checks(model, &axial);

    let n_checks = checks.len();
    let n_ng = checks.iter().filter(|c| !c.ok).count();
    let min_shear_margin = checks
        .iter()
        .map(|c| c.shear_margin)
        .filter(|m| m.is_finite())
        .fold(f64::INFINITY, f64::min);
    let members: Vec<serde_json::Value> = checks
        .iter()
        .map(|c| {
            serde_json::json!({
                "elem": c.elem.0,
                "kind": format!("{:?}", c.kind),
                "mu": c.mu,
                "qmu": c.qmu,
                "qsu": c.qsu,
                "qbu": c.qbu,
                "shear_margin": c.shear_margin,
                "bond_margin": c.bond_margin,
                "ok": c.ok,
            })
        })
        .collect();

    let cft_members: Vec<serde_json::Value> = cft_checks
        .iter()
        .map(|c| {
            serde_json::json!({
                "elem": c.elem.0,
                "class": format!("{:?}", c.class),
                "ncu": c.ncu,
                "ntu": c.ntu,
                "mu_nm": c.mu_nm,
                "n_design": c.n_design,
                "axial_margin": c.axial_margin,
                "ok": c.ok,
            })
        })
        .collect();

    let summary = serde_json::json!({
        "kind": "UltimateCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        "min_shear_margin": if min_shear_margin.is_finite() { serde_json::json!(min_shear_margin) } else { serde_json::Value::Null },
        "members": members,
        "n_cft_checks": cft_checks.len(),
        "n_cft_ng": cft_checks.iter().filter(|c| !c.ok).count(),
        "cft_members": cft_members,
    });
    Ok(JobOutcome::UltimateCheck { summary })
}
