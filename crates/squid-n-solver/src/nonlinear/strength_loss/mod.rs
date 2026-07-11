//! 段階的耐力喪失解析。RESP-D マニュアル計算編 03「応力解析」§段階的耐力喪失解析。
//!
//! ## 原典の規定（要約）
//! 荷重漸増解析（プッシュオーバー）は、荷重が減少するような層としての耐力劣化
//! （せん断降伏後の耐力低下）を直接表現できない。段階的耐力喪失解析は、これを
//! 擬似的に考慮するため次の手順を繰り返す。
//!
//! 1. 耐力喪失変形角（開始・終了）を設定する。
//! 2. せん断降伏後、耐力喪失変形角（終了側）に達する部材が発生するまで荷重漸増を行う。
//! 3. 該当部材が発生したら、当該部材を両端ピン・せん断非負担部材に置き換え、
//!    荷重を 0 から再載荷する。
//! 4. 2〜3 を繰り返し、得られた荷重変形関係を包絡することで耐力劣化を考慮した
//!    曲線を得る。
//!
//! 耐力喪失変形角は直接指定（開始・終了変形角）のほか、FEMA 356 Table 6-7 相当の
//! 大梁の非線形特性（塑性回転角）から算定することもできる（[`LossCriterion::Fema`]）。
//!
//! ## せん断降伏判定
//! `pushover::ShearYieldEvent`（部材端せん断力がせん断降伏耐力 Qy を超えたことを
//! 記録するイベント、`pushover::track_shear_yield` 参照）により、曲げ降伏
//! （`HingeLevel::Yield` 以降）とは独立に「せん断降伏」を判定できるようになった。
//! `detect_strength_loss` は原典の規定どおり、**せん断降伏イベントが発生済みの
//! 部材が耐力喪失変形角（終了側）を超えたこと**を耐力喪失の条件とする。
//!
//! せん断降伏イベントの判定精度（本モジュールの入力）は `pushover` 側で改良済み:
//! 材端力を要素局所座標系（`LocalFrame`）の ey・ez へ射影して局所 Vy・Vz を厳密に
//! 分離し、Vy は qy_y、Vz は qy_z と独立に比較する（軸直交合力を単一値として
//! min(qy_y,qy_z) と比較していた v1 の丸めを解消）。また RC 矩形断面
//! （`SectionShape::RcRect`）で配筋情報が得られる場合、Qy は荒川mean式系の略算式
//! （`squid_n_core::rc_capacity::rc_qsu_simple`）で算定する（配筋情報が無い場合は
//! 従来どおり慣用値 `as・0.7√fc` へフォールバック）。さらに、荒川式のせん断スパン
//! h0 は剛域長（`rigid_zone.length_i/length_j`）を控除した値を用い、軸力項 0.1・σ0
//! は各ステップの部材軸力（圧縮のみ、引張は0）から動的に反映する（旧来の
//! 「h0=節点間長・σ0=0固定」という安全側簡略化は解消済み）。詳細は `pushover`
//! モジュールの `ShearThreshold`・`DirThreshold`・`build_dir_threshold`・
//! `effective_clear_span`・`axial_compression`・`track_shear_yield` を参照。
//!
//! ただし、せん断降伏イベントが解析全体を通じて1件も発生しないモデル
//! （断面にせん断有効断面積 `as_y`/`as_z` が設定されていない、またはせん断余裕が
//! 大きく Qy に到達しないモデル）では、せん断耐力情報が実質的に無いに等しく、
//! せん断降伏の発生を厳密に要求すると耐力喪失解析そのものが機能しなくなる。
//! そのため、その場合に限り従来どおり「曲げ降伏（`HingeLevel::Yield` 以降）後、
//! 耐力喪失変形角を超えたこと」をもって代用するフォールバックを維持する
//! （v1 からの既知の制約。せん断耐力情報を持つモデルではこのフォールバックは
//! 使用されない）。
//!
//! 柱の FEMA 非線形特性（塑性回転角）は未対応（RESP-D 自身も未対応のため本実装も
//! 大梁のみを対象とする）。

use crate::analysis::SeismicDir;
use crate::constraint::Reducer;
use crate::pushover::{pushover_analysis_recording, HingeLevel, PushoverResult, PushoverStep};
use squid_n_core::dof::DofMap;
use squid_n_core::ids::{ElemId, SectionId};
use squid_n_core::model::{ElementData, EndCondition, Model};
use std::collections::{HashMap, HashSet};

/// 耐力喪失変形角の判定基準（原典 §段階的耐力喪失解析）。
#[derive(Clone, Debug)]
pub enum LossCriterion {
    /// 耐力喪失開始変形角・終了変形角の直接指定 [rad]。
    /// 終了変形角 (`end`) を超える部材が発生した際に、開始変形角 (`start`) を
    /// 超えている部材をまとめて両端ピンとして再載荷を行う（原典の「直接指定」）。
    DriftRange { start: f64, end: f64 },
    /// FEMA 356 Table 6-7 相当の非線形特性設定（大梁のみ。柱は未対応）。
    /// 部材ごとの `FemaBeamParams` から塑性回転角 a [rad] を算定し、
    /// 部材変形角が a に達した梁を耐力喪失部材として除去する
    /// （開始・終了の区別は無く、a 到達時点で即座に除去）。
    Fema {
        params: Vec<(ElemId, FemaBeamParams)>,
    },
}

/// FEMA 356 Table 6-7 相当の RC 大梁パラメータ（塑性回転角 a の算定に必要な諸元）。
///
/// 記号は原典引用のとおり: b=梁幅, d=有効せい, D=全せい, ρ=引張鉄筋比,
/// ρ′=圧縮鉄筋比, ρbal=釣り合い鉄筋比, s=せん断補強筋間隔, Vs=せん断耐力,
/// v_yield=両端降伏時せん断力 V, fc_prime=コンクリート強度 fc′ [N/mm²]。
#[derive(Clone, Copy, Debug)]
pub struct FemaBeamParams {
    pub b: f64,
    pub d: f64,
    pub depth_d: f64,
    pub rho: f64,
    pub rho_prime: f64,
    pub rho_bal: f64,
    pub s: f64,
    pub vs: f64,
    pub v_yield: f64,
    pub fc_prime: f64,
}

/// C 判定（せん断補強良好）: s ≤ D/3 かつ Vs ≥ 0.75・V。それ以外は NC。
pub fn fema_is_c(p: &FemaBeamParams) -> bool {
    p.s <= p.depth_d / 3.0 && p.vs >= 0.75 * p.v_yield
}

/// 区間 [x0, x1] の線形補間（x は [x0, x1] にクランプ）。
fn lerp_clamped(x: f64, x0: f64, x1: f64, y0: f64, y1: f64) -> f64 {
    if (x1 - x0).abs() < 1e-12 {
        return y0;
    }
    let t = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0);
    y0 + t * (y1 - y0)
}

/// FEMA 356 Table 6-7 相当の塑性回転角 a [rad] を、(ρ−ρ′)/ρbal と
/// V/(b・d・√fc′) の両軸で双線形補間して求める。
///
/// テーブル4隅（原典引用）:
/// | (ρ−ρ′)/ρbal | C/NC | V/(b・d・√fc′) | a |
/// |---|---|---|---|
/// | ≤0.0 | C  | ≤0.25 | 0.025 |
/// | ≤0.0 | C  | ≥0.5  | 0.02  |
/// | ≥0.5 | C  | ≤0.25 | 0.02  |
/// | ≥0.5 | C  | ≥0.5  | 0.015 |
/// | ≤0.0 | NC | ≤0.25 | 0.02  |
/// | ≤0.0 | NC | ≥0.5  | 0.01  |
/// | ≥0.5 | NC | ≤0.25 | 0.01  |
/// | ≥0.5 | NC | ≥0.5  | 0.005 |
pub fn fema_plastic_rotation(p: &FemaBeamParams) -> f64 {
    let is_c = fema_is_c(p);
    let ratio = if p.rho_bal > 0.0 {
        (p.rho - p.rho_prime) / p.rho_bal
    } else {
        0.0
    };
    let vn = if p.b > 0.0 && p.d > 0.0 && p.fc_prime > 0.0 {
        p.v_yield / (p.b * p.d * p.fc_prime.sqrt())
    } else {
        0.0
    };

    // (is_c, ratio>=0.5, vn>=0.5) の4隅の値。
    let corner = |ratio_hi: bool, vn_hi: bool| -> f64 {
        match (is_c, ratio_hi, vn_hi) {
            (true, false, false) => 0.025,
            (true, false, true) => 0.02,
            (true, true, false) => 0.02,
            (true, true, true) => 0.015,
            (false, false, false) => 0.02,
            (false, false, true) => 0.01,
            (false, true, false) => 0.01,
            (false, true, true) => 0.005,
        }
    };

    let a_ratio_lo = lerp_clamped(vn, 0.25, 0.5, corner(false, false), corner(false, true));
    let a_ratio_hi = lerp_clamped(vn, 0.25, 0.5, corner(true, false), corner(true, true));
    lerp_clamped(ratio, 0.0, 0.5, a_ratio_lo, a_ratio_hi)
}

impl LossCriterion {
    /// 部材の耐力喪失変形角しきい値 (start, end) [rad] を返す。
    /// `Fema` で該当部材のパラメータが無い場合は None（対象外＝喪失判定しない）。
    fn thresholds(&self, elem: ElemId) -> Option<(f64, f64)> {
        match self {
            LossCriterion::DriftRange { start, end } => Some((*start, *end)),
            LossCriterion::Fema { params } => {
                params.iter().find(|(id, _)| *id == elem).map(|(_, p)| {
                    let a = fema_plastic_rotation(p);
                    (a, a)
                })
            }
        }
    }
}

/// 段階的耐力喪失解析の1パス（再載荷ごと）を除いた全体結果。
pub struct StagedStrengthLossResult {
    /// 各再載荷パスのプッシュオーバー結果（耐力喪失部材が発生したステップまでに
    /// 打ち切ったもの。最終パスは正常終了まで含む）。
    pub passes: Vec<PushoverResult>,
    /// 各パスの荷重変形関係を包絡した曲線 (頂部変位, ベースシア)。頂部変位昇順。
    pub envelope: Vec<(f64, f64)>,
    /// 耐力喪失により除去された部材 (パス番号 0-origin, 部材ID)。
    pub removed: Vec<(usize, ElemId)>,
}

/// 部材の変形角（弦回転角相当）を、節点変位から算定する。
///
/// - 鉛直材（柱系、|Δz| が水平成分より大きい）: 材端の水平相対変位 / 材長
///   （層間変形角に相当する近似）。
/// - 水平材（梁系）: 材端の鉛直相対変位 / 材長（弦回転角）。
///
/// `disp` は `PushoverStep::node_disp`（`DofMap` アクティブ添字順の全節点変位）。
fn member_drift_angle(model: &Model, dofmap: &DofMap, disp: &[f64], elem: &ElementData) -> f64 {
    if elem.nodes.len() < 2 {
        return 0.0;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
        return 0.0;
    };
    let dx = pj.coord[0] - pi.coord[0];
    let dy = pj.coord[1] - pi.coord[1];
    let dz = pj.coord[2] - pi.coord[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
    if length <= 0.0 {
        return 0.0;
    }
    let get = |node_index: usize, dof: usize| -> f64 {
        let g = node_index * 6 + dof;
        dofmap
            .active(g)
            .and_then(|a| disp.get(a as usize).copied())
            .unwrap_or(0.0)
    };
    let vertical = dz.abs() > (dx.abs() + dy.abs()) * 0.5;
    if vertical {
        let dux = get(nj, 0) - get(ni, 0);
        let duy = get(nj, 1) - get(ni, 1);
        (dux * dux + duy * duy).sqrt() / length
    } else {
        (get(nj, 2) - get(ni, 2)).abs() / length
    }
}

/// 部材を「両端ピン・せん断非負担」の耐力喪失部材へ置き換える。
///
/// 既存の断面を共有する他部材へ影響しないよう、当該部材専用の縮小断面
/// （断面性能を全て微小係数倍。数値特異を避けるため完全ゼロにはしない）を
/// 新規に追加し、その断面へ差し替える。あわせて `end_cond` を両端ピンとする
/// （集中バネ系要素の回転拘束を解放する。ファイバー要素は end_cond を参照しない
/// ため断面縮小が実質的な除去手段となる）。
fn detach_element(model: &mut Model, elem_id: ElemId) {
    const EPS: f64 = 1.0e-6;
    let Some(elem) = model.elements.iter_mut().find(|e| e.id == elem_id) else {
        return;
    };
    elem.end_cond = [EndCondition::Pinned, EndCondition::Pinned];
    let Some(sid) = elem.section else {
        return;
    };
    let Some(sec) = model.sections.get(sid.index()).cloned() else {
        return;
    };
    let new_id = SectionId(model.sections.len() as u32);
    let mut lost = sec;
    lost.id = new_id;
    lost.name = format!("{}_lost", lost.name);
    lost.area *= EPS;
    lost.iy *= EPS;
    lost.iz *= EPS;
    lost.j *= EPS;
    lost.as_y *= EPS;
    lost.as_z *= EPS;
    model.sections.push(lost);
    if let Some(elem) = model.elements.iter_mut().find(|e| e.id == elem_id) {
        elem.section = Some(new_id);
    }
}

/// 打ち切りステップ番号以下のデータのみを残した `PushoverResult` を返す
/// （耐力喪失部材が発生した時点で当該パスの荷重漸増を終えたものとして扱う）。
fn truncate_result(mut result: PushoverResult, cutoff_step: u32) -> PushoverResult {
    result.capacity_curve.retain(|c| c.step <= cutoff_step);
    result.hinges.retain(|h| h.step <= cutoff_step);
    result.shear_yields.retain(|s| s.step <= cutoff_step);
    // steps は capacity_curve と同数・同順で積まれる（pushover.rs の実装契約）。
    let n = result.capacity_curve.len();
    result.steps.truncate(n);
    result
}

/// 各パスの `capacity_curve` を頂部変位で包絡する。
/// 頂部変位でソートし、同一変位でのベースシア最大値を採る。
fn build_envelope(passes: &[PushoverResult]) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = passes
        .iter()
        .flat_map(|p| p.capacity_curve.iter().map(|c| (c.roof_disp, c.base_shear)))
        .collect();
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut env: Vec<(f64, f64)> = Vec::new();
    for (x, y) in pts {
        if let Some(last) = env.last_mut() {
            if (last.0 - x).abs() < 1e-9 {
                if y > last.1 {
                    last.1 = y;
                }
                continue;
            }
        }
        env.push((x, y));
    }
    env
}

/// 1パス分の解析結果から、耐力喪失部材の発生を検出する。
///
/// 返り値: `Some((cutoff_step, elems))` — 検出できた場合、耐力喪失に至った
/// ステップ番号と、そのステップで除去すべき部材（開始変形角超過分をまとめて）。
/// 検出できなければ `None`（このパスで耐力喪失は発生しなかった＝解析終了）。
fn detect_strength_loss(
    model: &Model,
    dofmap: &DofMap,
    result: &PushoverResult,
    criterion: &LossCriterion,
    already_removed: &HashSet<ElemId>,
) -> Option<(u32, Vec<ElemId>)> {
    // 部材ごとの「降伏後」到達ステップ。原典どおりせん断降伏イベントを優先する。
    // 解析全体を通じてせん断降伏イベントが1件も無いモデル（せん断耐力情報が
    // 実質的に無い、またはせん断余裕が大きい）に限り、従来どおり曲げ降伏
    // （`HingeLevel::Yield` 以降）で代用するフォールバックを行う（モジュール doc 参照）。
    let mut first_yield_step: HashMap<ElemId, u32> = HashMap::new();
    if result.shear_yields.is_empty() {
        for h in &result.hinges {
            if matches!(h.level, HingeLevel::Yield | HingeLevel::Ultimate) {
                first_yield_step
                    .entry(h.elem)
                    .and_modify(|s| *s = (*s).min(h.step))
                    .or_insert(h.step);
            }
        }
    } else {
        for sy in &result.shear_yields {
            first_yield_step
                .entry(sy.elem)
                .and_modify(|s| *s = (*s).min(sy.step))
                .or_insert(sy.step);
        }
    }
    if first_yield_step.is_empty() {
        return None;
    }

    for (step, capacity) in result.capacity_curve.iter().enumerate() {
        let step_no = capacity.step;
        let Some(pstep): Option<&PushoverStep> = result.steps.get(step) else {
            continue;
        };
        let Some(disp) = &pstep.node_disp else {
            continue;
        };
        for elem in &model.elements {
            if already_removed.contains(&elem.id) {
                continue;
            }
            let Some(&y_step) = first_yield_step.get(&elem.id) else {
                continue;
            };
            if y_step > step_no {
                continue; // まだ降伏していない
            }
            let Some((_start, end)) = criterion.thresholds(elem.id) else {
                continue;
            };
            let theta = member_drift_angle(model, dofmap, disp, elem);
            if theta.abs() >= end {
                // 耐力喪失に至った。開始変形角を超える降伏済み部材をまとめて除去する。
                let mut to_remove = Vec::new();
                for e2 in &model.elements {
                    if already_removed.contains(&e2.id) {
                        continue;
                    }
                    let Some(&y2) = first_yield_step.get(&e2.id) else {
                        continue;
                    };
                    if y2 > step_no {
                        continue;
                    }
                    let Some((start2, _end2)) = criterion.thresholds(e2.id) else {
                        continue;
                    };
                    let theta2 = member_drift_angle(model, dofmap, disp, e2);
                    if theta2.abs() >= start2 {
                        to_remove.push(e2.id);
                    }
                }
                return Some((step_no, to_remove));
            }
        }
    }
    None
}

/// 段階的耐力喪失解析（RESP-D マニュアル計算編 03「応力解析」§段階的耐力喪失解析）。
///
/// `model` は変更しない（内部でクローンして再載荷パスごとに部材を除去する）。
/// `max_passes` は再載荷パスの上限（無限ループ防止。目安 10）。
#[allow(clippy::too_many_arguments)]
pub fn staged_strength_loss(
    model: &Model,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
    criterion: &LossCriterion,
    max_passes: usize,
) -> Result<StagedStrengthLossResult, String> {
    // 節点・拘束はパスを通じて不変（除去は断面差し替え・end_cond 変更のみ）なので
    // DofMap／Reducer は最初の一回だけ構築して使い回す。
    let dofmap = DofMap::build(model);
    let reducer = Reducer::build(model, &dofmap);

    let mut current_model = model.clone();
    let mut passes: Vec<PushoverResult> = Vec::new();
    let mut removed: Vec<(usize, ElemId)> = Vec::new();
    let mut removed_set: HashSet<ElemId> = HashSet::new();

    let n_passes = max_passes.max(1);
    for pass_idx in 0..n_passes {
        let mut m = current_model.clone();
        let result = pushover_analysis_recording(
            &mut m,
            &dofmap,
            &reducer,
            dir,
            max_steps,
            max_disp,
            use_kg,
            use_arc_length,
            arc_length_dl,
            true,
        )?;

        match detect_strength_loss(&current_model, &dofmap, &result, criterion, &removed_set) {
            Some((cutoff_step, to_remove)) if !to_remove.is_empty() => {
                let truncated = truncate_result(result, cutoff_step);
                passes.push(truncated);
                for elem_id in to_remove {
                    detach_element(&mut current_model, elem_id);
                    removed_set.insert(elem_id);
                    removed.push((pass_idx, elem_id));
                }
                // 次パスへ（荷重 0 から再載荷）。
            }
            _ => {
                // 耐力喪失部材が新たに発生しなかった＝解析終了。
                passes.push(result);
                break;
            }
        }
    }

    let envelope = build_envelope(&passes);
    Ok(StagedStrengthLossResult {
        passes,
        envelope,
        removed,
    })
}

#[cfg(test)]
mod tests;
