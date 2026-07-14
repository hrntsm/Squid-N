//! RC 造の許容応力度と断面検定（RC 規準13〜18条・令82条による許容応力度計算）。
//!
//! 準拠する規準:
//! - 許容応力度・ヤング係数比: 2010年版 RC 規準・構造規定
//! - 梁の曲げ・せん断検定: RC 規準 13条
//! - 柱の軸力＋曲げ検定: RC 規準 14条
//!
//! # 実装方針（全体）
//! - `Section.shape` が `RcRect`/`RcCircle` でない場合（配筋情報なし）は
//!   検定をスキップし `ok=true` で返す（旧実装と同じフォールバック）。
//! - `Material.fc` が未設定/0 の場合も同様にスキップする。
//! - 梁は強軸曲げ（`mz`）とそれに対のせん断（`qy`）のみを検定する
//!   （RC 規準の梁断面検定の対象と一致）。
//! - 柱は軸力（M=0）・軸力＋二軸曲げ・二方向せん断を検定する。
//! - `MemberKind::Brace` は RC 部材としては未対応のため、梁の検定式で代用する。
//!
//! # モジュール構成（RC 規準の許容応力度検定の対象部材に対応）
//! - 本ファイル（`rc/mod.rs`）: 断面諸元の抽出・許容応力度のまとめ・
//!   せん断スパン比 α・せん断耐力・地震時設計用せん断力・`RcDesign`
//!   （`DesignCheck` 実装、梁/柱への振り分け）。
//! - [`beam`]: 鉄筋コンクリート造梁の断面検定（RC 規準13条の曲げ・せん断）。
//! - [`column`]: 鉄筋コンクリート造柱の断面検定（RC 規準14条の軸力+曲げ）。
//! - [`bond`]: 鉄筋コンクリート造梁付着の断面検定（RC 規準1999/1991 方式）。
//! - [`joint`]: 鉄筋コンクリート造柱梁接合部の断面検定（RC 規準15条）。
//! - [`wall`]: 鉄筋コンクリート造耐震壁の断面検定（RC 規準18条）。

use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
use squid_n_core::units::ConcreteClass;

mod beam;
/// 鉄筋コンクリート造梁の非線形復元力特性（曲げトリリニア・せん断・軸）。
/// 非線形解析の材端バネ骨格に用いる（技術基準解説書「部材の復元力特性」）。
pub mod beam_nonlinear;
mod bond;
mod column;
/// 鉄筋コンクリート造水平接合面の検討（PCa 打継ぎ面のせん断検定）。
pub mod horizontal_joint;
pub mod joint;
pub mod wall;
/// 鉄筋コンクリート造耐震壁のせん断非線形特性（トリリニア Qc/βu/Qu）。
/// 非線形解析のせん断ばね骨格に用いる（技術基準解説書「耐震壁のせん断非線形特性」）。
pub mod wall_nonlinear;

pub use bond::{rc_beam_bond_check, rc_beam_bond_check_1991, Bond1991Result, BondCheckResult};
pub use wall_nonlinear::{
    wall_shear_beta_u, wall_shear_crack, wall_shear_trilinear, wall_shear_ultimate,
    WallShearTrilinear, WallShearTrilinearInput,
};

// 材料強度・許容応力度は `crate::material_strength`（RC 規準の材料強度・許容応力度）へ
// 集約した。RC 造の検定で用いるものを再エクスポートし、従来の
// `crate::rc::concrete_allowable_shear` 等のパスも維持する。
pub use crate::material_strength::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_compression_class,
    concrete_allowable_shear, concrete_allowable_shear_class, concrete_young_modulus,
    high_strength_group, high_strength_pw_cap, high_strength_w_ft, rebar_allowable_shear,
    rebar_allowable_tension, rebar_sigma_y, young_ratio_n, HighStrengthGroup,
};

// ============================================================================
// 1. 断面諸元の抽出
// ============================================================================

/// 検討方向 1 軸分の断面諸元。
struct AxisProps {
    /// 検討方向の幅 [mm]（強軸曲げなら sec.width 等）。
    b: f64,
    /// 検討方向のせい D [mm]。
    d_full: f64,
    /// 引張縁から引張筋重心までの距離 dt [mm]。
    dt: f64,
    /// 有効せい d = D - dt [mm]。
    d: f64,
    /// 引張鉄筋断面積 at [mm²]（片側）。
    at: f64,
    /// 圧縮鉄筋断面積 ac [mm²]（片側、at と同値の対称複筋仮定）。
    ac: f64,
    /// 応力中心間距離 j = 7d/8 [mm]。
    j: f64,
    /// せん断補強筋比 pw。
    pw: f64,
}

/// 主筋 1 本あたりの断面積 [mm²]。
fn one_bar_area(dia: f64) -> f64 {
    let r = dia / 2.0;
    std::f64::consts::PI * r * r
}

/// 主筋セットの総断面積 [mm²]。
fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * one_bar_area(bar.dia)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
///
/// 1 段筋（`layers<=1`）は重心 k1 = cover + shear.dia + main.dia/2。
/// 2 段以上は RC 配筋指針式（2 段の場合）
/// `k2 = k1 + D1/2 + k' + D2/2`（`k' = max(25, 1.5・dia)`, `D1=D2=main.dia`）
/// により `dt = (k1+k2)/2` とする。3 段以上は各段が等間隔 `s = dia + k'` で
/// 並び、各段の本数が等しいと仮定して重心を平均で一般化する:
/// `dt = k1 + (layers-1)/2・s`（layers=2 で上式に一致）。
fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if main.layers <= 1 {
        return k1;
    }
    let k_prime = 25.0_f64.max(1.5 * main.dia);
    let s = main.dia + k_prime;
    k1 + (main.layers as f64 - 1.0) / 2.0 * s
}

/// せん断補強筋比 pw = (legs・π/4・dia²) / (b・pitch)。pitch<=0 のときは 0。
fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw = shear.legs as f64 * std::f64::consts::PI / 4.0 * shear.dia * shear.dia;
    aw / (b * shear.pitch)
}

/// 矩形断面 1 軸分の断面諸元を算定する。
///
/// `width_dir_b`: 検討方向の幅、`depth_dir_d`: 検討方向のせい、
/// `main`: 当該方向の主筋（強軸曲げは main_x、弱軸曲げは main_y）。
fn rect_axis_props(
    width_dir_b: f64,
    depth_dir_d: f64,
    main: &BarSet,
    rebar: &RcRebar,
) -> AxisProps {
    let dt = tension_dt(rebar.cover, rebar.shear.dia, main);
    let d = depth_dir_d - dt;
    let at = bar_set_area(main) / 2.0;
    AxisProps {
        b: width_dir_b,
        d_full: depth_dir_d,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, width_dir_b),
    }
}

/// 強軸曲げ（mz）用の断面諸元。b=sec.width, D=sec.depth, 主筋=main_x。
fn rect_axis_props_strong(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.width, sec.depth, &rebar.main_x, rebar)
}

/// 弱軸曲げ（my）用の断面諸元。b=sec.depth, D=sec.width, 主筋=main_y。
fn rect_axis_props_weak(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.depth, sec.width, &rebar.main_y, rebar)
}

/// 円形柱の等価矩形断面諸元。b=(D/2)√π、せい=D。
/// 引張筋本数 nt = ng/4+1（ng = 全主筋本数、`rebar.main_x.count` を採用）。
/// 対称複筋（at=ac）を仮定する。
fn circle_axis_props(d_full: f64, rebar: &RcRebar) -> AxisProps {
    let b = (d_full / 2.0) * std::f64::consts::PI.sqrt();
    let ng = rebar.main_x.count as f64;
    let nt = ng / 4.0 + 1.0;
    let at = nt * one_bar_area(rebar.main_x.dia);
    let dt = tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x);
    let d = d_full - dt;
    AxisProps {
        b,
        d_full,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, b),
    }
}

// ============================================================================
// 3. 許容応力度のまとめ（部材単位で term 依存の値を 1 回だけ計算する）
// ============================================================================

/// 検定に用いる許容応力度一式（コンクリート・せん断補強筋。ft は主筋径に
/// 依存するため軸別に別途算定する）。
struct RcAllow {
    /// コンクリート許容圧縮応力度 fc [N/mm²]（長期/短期は算定済み）。
    fc: f64,
    /// コンクリート許容せん断応力度 fs [N/mm²]。
    fs: f64,
    /// せん断補強筋許容引張応力度 w_ft [N/mm²]。
    w_ft: f64,
    /// ヤング係数比 n。
    n_ratio: f64,
}

fn rc_allow(fc_raw: f64, class: ConcreteClass, grade: &str, long_term: bool) -> RcAllow {
    RcAllow {
        fc: concrete_allowable_compression_class(fc_raw, class, long_term),
        fs: concrete_allowable_shear_class(fc_raw, class, long_term),
        w_ft: rebar_allowable_shear(grade, long_term),
        n_ratio: young_ratio_n(fc_raw),
    }
}

/// 高強度せん断補強筋使用時の「損傷制御のための検討」の対象可否を反映した
/// 有効 damage_control。各製品の大臣認定（ウルボン1275等）により、高強度
/// せん断補強筋を使用する軽量コンクリート部材は損傷制御のための検討の対象外
/// とし、安全確保のための検討のみを行う（`shear_grade` が `Some` かつ
/// `class` が軽量1種/2種のとき damage_control を強制的に false にする）。
fn effective_damage_control(
    damage_control: bool,
    shear_grade: Option<&str>,
    class: ConcreteClass,
) -> bool {
    if shear_grade.is_some() && class != ConcreteClass::Normal {
        false
    } else {
        damage_control
    }
}

// ============================================================================
// 4. せん断スパン比 α とせん断耐力
// ============================================================================

/// せん断スパン比による割増係数 α = 4/(M/(Q・d)+1)。`max_alpha` でクランプ
/// （梁 2.0、柱 1.5）。下限は共通で 1.0。
///
/// 退化時の規約: Q≈0 かつ M>0 は M/(Q・d)→∞ すなわち α→下限 1.0 を返す
/// （従来は上限 max_alpha を返しており、割増を最大化する非保守側だった）。
/// M も Q も 0（無応力）または d≤0 は中立な α=1.0（割増なし）とする。
fn shear_alpha(m: f64, q: f64, d: f64, max_alpha: f64) -> f64 {
    if q.abs() < 1e-9 || d <= 0.0 {
        return 1.0;
    }
    let mqd = m.abs() / (q.abs() * d);
    let alpha = 4.0 / (mqd + 1.0);
    alpha.clamp(1.0, max_alpha)
}

/// 許容せん断力 QA [N]。
///
/// 梁（`is_column=false`）:
/// - 長期  `QAL = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 0.6% 上限）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 1.2% 上限）
///
/// 柱（`is_column=true`）:
/// - 長期  `QAL = b・j・α・fs`（補強筋項なし）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(fs + 0.5・w_ft・(pw-0.002))`（**α を含まない**）
///
/// いずれも pw<0.002 のときせん断補強筋項は 0（マイナスにしない）。
fn shear_capacity(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
) -> f64 {
    let pw_cap = if term == LoadTerm::Long { 0.006 } else { 0.012 };
    shear_capacity_generic(
        props,
        allow,
        alpha,
        term,
        damage_control,
        is_column,
        pw_cap,
        0.002,
    )
}

/// 許容せん断力 QA の汎用式。`pw_cap`（pw の上限値）・`pw_offset`
/// （せん断補強筋項のオフセット、通常は 0.002）を外部から指定できる。
/// `shear_capacity`（普通強度）はこの関数をオフセット 0.002 固定で呼び出す
/// ラッパーであり、高強度せん断補強筋用の
/// `shear_capacity_high_strength` はオフセット・pw 上限を製品ごとに変えて
/// 呼び出す。
#[allow(clippy::too_many_arguments)]
fn shear_capacity_generic(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
    pw_cap: f64,
    pw_offset: f64,
) -> f64 {
    let pw = props.pw.min(pw_cap);
    let pw_term = if props.pw < pw_offset {
        0.0
    } else {
        0.5 * allow.w_ft * (pw - pw_offset)
    };

    match term {
        LoadTerm::Long => {
            if is_column {
                props.b * props.j * alpha * allow.fs
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
        LoadTerm::Short => {
            if damage_control {
                props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term)
            } else if is_column {
                // 柱の安全確保のための検討式は α を含まない。
                props.b * props.j * (allow.fs + pw_term)
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
    }
}

// ----------------------------------------------------------------------
// 4.1 高強度せん断補強筋（許容せん断応力度検定。各製品の大臣認定に
// 基づく）
// ----------------------------------------------------------------------
//
// `ShearBar.grade` に製品名/規格名（例 "UB785", "KH785", "SBPD1275" 等）が
// 設定されている場合、通常鋼材（SD295〜SD490）の許容せん断応力度表とは
// 別の高強度品用テーブルを用いる。
//
// # 簡略化・注意事項
// - 各製品の大臣認定では製品ごとに精算式（例: ウルボン1275 の √ を含む式、
//   KH785 系の βc を用いる式など）が規定されているが、本実装では未実装。
//   「上記以外の高強度せん断補強筋の場合」に相当する
//   暫定対応式（下記 `shear_capacity_high_strength`）を全高強度製品に
//   一律適用する。より精算値が必要な場合は今後の課題とする。
// - pw の上限値は各高強度せん断補強筋の大臣認定に基づく製品グループごとの定数
//   表とし、グループ判別ができない（未知の高強度品名の）場合は安全側の
//   0.8% を用いる。

/// 高強度せん断補強筋使用時の許容せん断力 QA（「上記以外の
/// 高強度せん断補強筋の場合」に相当する暫定対応式、全高強度製品に適用）。
///
/// - 長期: 普通強度と同一の式（offset=0.002・pw 上限 0.6%）。w_ft のみ
///   高強度品テーブル値（=195、普通強度と同値）を用いる。
/// - 短期: offset=0.001（`pw - 0.001` 項）・pw 上限は製品グループごとの
///   値を用いる。梁は `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.001))`
///   （損傷制御）/ `b・j・(α・fs + 0.5・w_ft・(pw-0.001))`（安全確保）、
///   柱は安全確保式で α を含まない
///   （`QAS = b・j・(fs + 0.5・w_ft・(pw-0.001))`）。
#[allow(clippy::too_many_arguments)]
fn shear_capacity_high_strength(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
    shear_grade: &str,
    fc_raw: f64,
) -> f64 {
    let pw_offset = if term == LoadTerm::Long { 0.002 } else { 0.001 };
    let pw_cap = high_strength_pw_cap(shear_grade, term, damage_control, fc_raw);
    shear_capacity_generic(
        props,
        allow,
        alpha,
        term,
        damage_control,
        is_column,
        pw_cap,
        pw_offset,
    )
}

/// `ShearBar.grade` の有無に応じて普通強度／高強度いずれかの許容せん断力
/// 算定式を選択する。`fc_raw` は高強度せん断補強筋の pw 上限が Fc に依存する
/// 製品（KH785/KH685/SPR685）向けに渡す Fc(raw) [N/mm²]。
#[allow(clippy::too_many_arguments)]
fn shear_capacity_for(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
    shear_grade: Option<&str>,
    fc_raw: f64,
) -> f64 {
    match shear_grade {
        Some(g) => shear_capacity_high_strength(
            props,
            allow,
            alpha,
            term,
            damage_control,
            is_column,
            g,
            fc_raw,
        ),
        None => shear_capacity(props, allow, alpha, term, damage_control, is_column),
    }
}

// ============================================================================
// 4.2 地震時短期の設計用せん断力 QD = min(QD1, QD2)
// （梁/柱の設計用せん断力。RC 規準）
// ============================================================================

/// 地震時短期の設計用せん断力 QD [N]。
///
/// - 梁: `QD1 = QL + ΣBMy/l′`、柱: `QD1 = ΣcMy/h′`
/// - `QD2 = QL + n・QE`（`QE` = 当該組合せのせん断力 − 長期せん断力）
/// - `QD = min(QD1, QD2)`（[`crate::QdMethod`] により QD1/QD2 単独も選択可）
///
/// `ctx.seismic_qd` が None（長期・積雪時・暴風時）、または長期内力に同一
/// 評価位置が見つからない場合は、解析せん断力 `|q_signed|` をそのまま返す
/// （積雪時・暴風時の `QD = QL + Qsn／QL + Qw` は組合せの弾性せん断力に一致）。
///
/// `q_index`: 長期内力配列 `[N,Qy,Qz,Mx,My,Mz]` のせん断成分位置（qy=1, qz=2）。
/// `sum_mu`: 部材両端の終局曲げモーメントの絶対値和 ΣMy [N·mm]。0 以下または
/// `clear_length` が 0 以下の場合、QD1 は無効（QD2 のみ）とする。
pub(crate) fn seismic_design_shear(
    ctx: &DesignCtx,
    pos: f64,
    q_signed: f64,
    q_index: usize,
    sum_mu: f64,
    is_column: bool,
) -> f64 {
    let Some(qd) = &ctx.seismic_qd else {
        return q_signed.abs();
    };
    let Some(ql_signed) = qd
        .long_at
        .iter()
        .find(|(p, _)| (p - pos).abs() < 1e-6)
        .map(|(_, f)| f[q_index])
    else {
        return q_signed.abs();
    };
    let ql = ql_signed.abs();
    let qe = (q_signed - ql_signed).abs();
    let qd2 = ql + qd.n_factor * qe;
    let qd1 = if qd.clear_length > 0.0 && sum_mu > 0.0 {
        if is_column {
            sum_mu / qd.clear_length
        } else {
            ql + sum_mu / qd.clear_length
        }
    } else {
        f64::INFINITY
    };
    match qd.method {
        crate::QdMethod::Qd1 => {
            if qd1.is_finite() {
                qd1
            } else {
                qd2
            }
        }
        crate::QdMethod::Qd2 => qd2,
        crate::QdMethod::Min => qd1.min(qd2),
    }
}

// ============================================================================
// 8. DesignCheck 実装（梁は rc/beam.rs、柱は rc/column.rs へ振り分け）
// ============================================================================

pub struct RcDesign;

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let fc_raw = mat.fc.unwrap_or(0.0);
        if fc_raw <= 0.0 {
            return CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "RC 検定: Fc 未設定".to_string(),
                detail: "Material.fc が None/0 です。コンクリート強度を設定してください。"
                    .to_string(),
            };
        }

        let shape = match &sec.shape {
            Some(s @ SectionShape::RcRect { .. }) => s,
            Some(s @ SectionShape::RcCircle { .. }) => s,
            _ => {
                return CheckResult {
                    ratio: 0.0,
                    ok: true,
                    basis: "RC 検定: 配筋情報なし".to_string(),
                    detail: "Section.shape が RcRect/RcCircle ではないため検定をスキップしました。"
                        .to_string(),
                };
            }
        };

        match ctx.kind {
            MemberKind::Beam | MemberKind::Brace => {
                beam::beam_check(forces, sec, mat, ctx, shape, fc_raw)
            }
            MemberKind::Column => column::column_check(forces, sec, mat, ctx, shape, fc_raw),
        }
    }
}

// ============================================================================
// テスト（断面諸元・許容応力度・せん断耐力・地震時せん断力・RcDesign 統合系）
// ============================================================================

#[cfg(test)]
mod tests;
