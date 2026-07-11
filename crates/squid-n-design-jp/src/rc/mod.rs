//! RC 造の許容応力度と断面検定（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」の RC 造部分に準拠）。
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
//!   （マニュアルの梁断面検定の対象と一致）。
//! - 柱は軸力（M=0）・軸力＋二軸曲げ・二方向せん断を検定する。
//! - `MemberKind::Brace` は RC 部材としては未対応のため、梁の検定式で代用する。
//!
//! # モジュール構成（RESP-D マニュアル「04 断面検定」の章立てに対応）
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
mod bond;
mod column;
/// 鉄筋コンクリート造水平接合面の検討（PCa 打継ぎ面のせん断検定）。
pub mod horizontal_joint;
pub mod joint;
pub mod wall;

pub use bond::{rc_beam_bond_check, rc_beam_bond_check_1991, Bond1991Result, BondCheckResult};

// 材料強度・許容応力度は `crate::material`（RESP-D「材料強度・許容応力度」節）へ
// 集約した。RC 造の検定で用いるものを再エクスポートし、従来の
// `crate::rc::concrete_allowable_shear` 等のパスも維持する。
pub use crate::material::{
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
/// 有効 damage_control。マニュアル（ウルボン1275等の規定）により、高強度
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
fn shear_alpha(m: f64, q: f64, d: f64, max_alpha: f64) -> f64 {
    if q.abs() < 1e-9 || d <= 0.0 {
        return max_alpha;
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
// 4.1 高強度せん断補強筋（RESP-D マニュアル「04 断面検定 (A) 高強度せん断
// 補強筋」）
// ----------------------------------------------------------------------
//
// `ShearBar.grade` に製品名/規格名（例 "UB785", "KH785", "SBPD1275" 等）が
// 設定されている場合、通常鋼材（SD295〜SD490）の許容せん断応力度表とは
// 別の高強度品用テーブルを用いる。
//
// # 簡略化・注意事項
// - マニュアルは製品ごとに精算式（例: ウルボン1275 の √ を含む式、
//   KH785 系の βc を用いる式など）を規定しているが、本実装では未実装。
//   マニュアル自身が「上記以外の高強度せん断補強筋の場合」として記載する
//   暫定対応式（下記 `shear_capacity_high_strength`）を全高強度製品に
//   一律適用する。より精算値が必要な場合は今後の課題とする。
// - pw の上限値は RESP-D マニュアルの記載に基づく製品グループごとの定数
//   表とし、グループ判別ができない（未知の高強度品名の）場合は安全側の
//   0.8% を用いる。

/// 高強度せん断補強筋使用時の許容せん断力 QA（マニュアル「上記以外の
/// 高強度せん断補強筋の場合」の暫定対応式、全高強度製品に適用）。
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
// （RESP-D マニュアル 04 断面検定「梁/柱の設計用せん断力」）
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
fn seismic_design_shear(
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
mod tests {
    use super::*;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    pub(crate) fn make_material(fc: f64, grade: &str) -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: grade.to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: Some(fc),
            fy: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rc_rect_shape(
        b: f64,
        d: f64,
        main_count: u32,
        main_dia: f64,
        main_layers: u32,
        cover: f64,
        shear_dia: f64,
        shear_pitch: f64,
        shear_legs: u32,
    ) -> SectionShape {
        SectionShape::RcRect {
            b,
            d,
            rebar: RcRebar {
                main_x: BarSet {
                    count: main_count,
                    dia: main_dia,
                    layers: main_layers,
                },
                main_y: BarSet {
                    count: main_count,
                    dia: main_dia,
                    layers: main_layers,
                },
                cover,
                shear: ShearBar {
                    dia: shear_dia,
                    pitch: shear_pitch,
                    legs: shear_legs,
                    grade: None,
                },
            },
        }
    }

    /// `rc_rect_shape` のせん断補強筋に高強度品の `grade` を付与した版。
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rc_rect_shape_with_shear_grade(
        b: f64,
        d: f64,
        main_count: u32,
        main_dia: f64,
        main_layers: u32,
        cover: f64,
        shear_dia: f64,
        shear_pitch: f64,
        shear_legs: u32,
        shear_grade: &str,
    ) -> SectionShape {
        match rc_rect_shape(
            b,
            d,
            main_count,
            main_dia,
            main_layers,
            cover,
            shear_dia,
            shear_pitch,
            shear_legs,
        ) {
            SectionShape::RcRect { b, d, mut rebar } => {
                rebar.shear.grade = Some(shear_grade.to_string());
                SectionShape::RcRect { b, d, rebar }
            }
            _ => unreachable!(),
        }
    }

    pub(crate) fn make_section(shape: SectionShape) -> Section {
        shape.to_section(SectionId(0), "test".to_string())
    }

    pub(crate) fn ctx_beam(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Beam,
            ..Default::default()
        }
    }

    pub(crate) fn ctx_column(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Column,
            ..Default::default()
        }
    }

    // ------------------------------------------------------------------
    // 地震時短期の設計用せん断力 QD = min(QD1, QD2)
    // ------------------------------------------------------------------

    #[test]
    fn test_seismic_design_shear_min_of_qd1_qd2() {
        use crate::{QdMethod, SeismicQd};
        let mut ctx = DesignCtx {
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, 50_000.0, 0.0, 0.0, 0.0, 0.0])],
                n_factor: 1.5,
                clear_length: 4000.0,
                method: QdMethod::Min,
            }),
            ..Default::default()
        };
        // 当該組合せ Q=150kN、QL=50kN → QE=100kN、QD2 = 50+1.5×100 = 200kN。
        // ΣMy=400kN·m → 梁 QD1 = 50+400e6/4000 = 150kN → min = 150kN。
        let q_beam = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
        assert!((q_beam - 150_000.0).abs() < 1e-6, "q_beam={q_beam}");
        // 柱 QD1 = ΣcMy/h′ = 100kN（QL を加算しない）→ min(100, 200) = 100kN。
        let q_col = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, true);
        assert!((q_col - 100_000.0).abs() < 1e-6, "q_col={q_col}");
        // QD2 単独選択。
        ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Qd2;
        let q2 = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
        assert!((q2 - 200_000.0).abs() < 1e-6, "q2={q2}");
        // ΣMy<=0（終局曲げ不明）のとき QD1 は無効で QD2 のみ。
        ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Min;
        let q_no_mu = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 0.0, false);
        assert!((q_no_mu - 200_000.0).abs() < 1e-6);
        // 評価位置が長期内力に無い場合・文脈なしの場合は解析値のまま。
        let q_missing = seismic_design_shear(&ctx, 0.5, 150_000.0, 1, 400.0e6, false);
        assert!((q_missing - 150_000.0).abs() < 1e-6);
        ctx.seismic_qd = None;
        let q_none = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
        assert!((q_none - 150_000.0).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // 許容応力度（RC 造検定でのみ使う独自カバレッジ分。他は material.rs 側で検証）
    // ------------------------------------------------------------------

    #[test]
    fn test_concrete_young_modulus_plausible() {
        // Fc=21, γ=23 で AIJ 表の目安値（約 2.0〜2.3 × 10^4 N/mm²）に近い。
        let ec = concrete_young_modulus(21.0, Some(23.0));
        assert!(ec > 20000.0 && ec < 23000.0, "Ec={ec}");
    }

    #[test]
    fn test_rebar_allowable_tension_table() {
        assert!((rebar_allowable_tension("SR235", 16.0, true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR235", 16.0, false) - 235.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR295", 16.0, true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR295", 16.0, false) - 295.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD295A", 16.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 22.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 32.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 22.0, false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD490", 22.0, false) - 490.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("UNKNOWN", 22.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("UNKNOWN", 22.0, false) - 295.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_allowable_shear_table() {
        assert!((rebar_allowable_shear("SR235", true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD345", true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD295A", false) - 295.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD345", false) - 345.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD390", false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD490", false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("UNKNOWN", false) - 295.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // dt（引張筋重心）
    // ------------------------------------------------------------------

    #[test]
    fn test_tension_dt_single_layer() {
        let bar = BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        };
        let dt = tension_dt(40.0, 10.0, &bar);
        assert!((dt - (40.0 + 10.0 + 11.0)).abs() < 1e-9);
    }

    #[test]
    fn test_tension_dt_two_layers() {
        let bar = BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        };
        let cover = 40.0;
        let shear_dia = 10.0;
        let k1 = cover + shear_dia + bar.dia / 2.0;
        let k_prime = 25.0_f64.max(1.5 * bar.dia);
        let k2 = k1 + bar.dia / 2.0 + k_prime + bar.dia / 2.0;
        let expected = (k1 + k2) / 2.0;
        let dt = tension_dt(cover, shear_dia, &bar);
        assert!((dt - expected).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // せん断スパン比 α・せん断耐力（普通強度・高強度せん断補強筋とも）
    // ------------------------------------------------------------------

    #[test]
    fn test_shear_alpha_clamp_at_upper_bound() {
        // M/(Q・d) = 1 -> α = 4/2 = 2.0（上限に一致）
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 1.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_clamp_at_lower_bound() {
        // M/(Q・d) = 3 -> α = 4/4 = 1.0（下限に一致）
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 3.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_clamp_engages_beyond_bounds() {
        let d = 500.0;
        let q = 100_000.0;
        // M/(Q・d)=0 -> 素の α=4.0 は上限 2.0 にクランプされる。
        let alpha_hi = shear_alpha(0.0, q, d, 2.0);
        assert!((alpha_hi - 2.0).abs() < 1e-9);
        // M/(Q・d)=10 -> 素の α=4/11≈0.364 は下限 1.0 にクランプされる。
        let m = q * d * 10.0;
        let alpha_lo = shear_alpha(m, q, d, 2.0);
        assert!((alpha_lo - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_intermediate_value() {
        // M/(Q・d) = 1 と 3 の中間、M/(Q・d)=2 -> α = 4/3 ≈ 1.333
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 2.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 4.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_pw_ratio_capped_long_term() {
        // 過大なせん断補強筋比を作り、長期は 0.6% に制限されることを確認する。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 30.0, 4);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
        let alpha = 1.5;
        let qa_capped = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, false);

        // 手計算: pw を 0.6% に制限した式と一致すること。
        let pw_term = 0.5 * allow.w_ft * (0.006 - 0.002);
        let expected = props.b * props.j * (alpha * allow.fs + pw_term);
        assert!((qa_capped - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_beam_shear_damage_control_vs_safety() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        let alpha = 1.4;

        let qa_damage = shear_capacity(&props, &allow, alpha, LoadTerm::Short, true, false);
        let qa_safety = shear_capacity(&props, &allow, alpha, LoadTerm::Short, false, false);

        let pw_term = if props.pw < 0.002 {
            0.0
        } else {
            0.5 * allow.w_ft * (props.pw.min(0.012) - 0.002)
        };
        let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term);
        let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term);

        assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
        assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
        assert!(
            qa_damage < qa_safety,
            "損傷制御式は安全確保式より小さいはず"
        );
    }

    #[test]
    fn test_column_shear_alpha_upper_bound_1_5() {
        let d = 400.0;
        let q = 50_000.0;
        // M/(Q・d)=0 -> 素の α=4.0 は柱の上限 1.5 にクランプされる。
        let alpha = shear_alpha(0.0, q, d, 1.5);
        assert!((alpha - 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_column_safety_check_excludes_alpha() {
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props_strong(&make_section(shape), &rebar);
        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);

        let qa_alpha_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, false, true);
        let qa_alpha_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, false, true);
        // 柱の「安全確保のための検討」式は α を含まないため、α を変えても
        // QA は変化しない。
        assert!((qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6);

        // 損傷制御式は α に依存するため異なる値になる。
        let qa_damage_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, true, true);
        let qa_damage_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, true, true);
        assert!((qa_damage_1 - qa_damage_1_5).abs() > 1e-6);
    }

    #[test]
    fn test_column_long_term_shear_has_no_rebar_term() {
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 60.0, 4);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props_strong(&make_section(shape), &rebar);
        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
        let alpha = 1.3;
        let qal = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, true);
        let expected = props.b * props.j * alpha * allow.fs;
        assert!((qal - expected).abs() / expected < 1e-9);
    }

    #[test]
    fn test_high_strength_shear_capacity_offset_0_001_beam() {
        // 高強度せん断補強筋の暫定対応式（短期）は pw オフセットが 0.001。
        let shape =
            rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "KH785");
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        assert!(props.pw > 0.001, "テストの前提として pw > 0.1% が必要");

        let mut allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        allow.w_ft = high_strength_w_ft("KH785", false);
        let alpha = 1.4;

        let qa_damage = shear_capacity_high_strength(
            &props,
            &allow,
            alpha,
            LoadTerm::Short,
            true,
            false,
            "KH785",
            24.0,
        );
        let qa_safety = shear_capacity_high_strength(
            &props,
            &allow,
            alpha,
            LoadTerm::Short,
            false,
            false,
            "KH785",
            24.0,
        );

        let pw_cap_damage = high_strength_pw_cap("KH785", LoadTerm::Short, true, 24.0);
        let pw_cap_safety = high_strength_pw_cap("KH785", LoadTerm::Short, false, 24.0);
        let pw_term_damage = 0.5 * allow.w_ft * (props.pw.min(pw_cap_damage) - 0.001);
        let pw_term_safety = 0.5 * allow.w_ft * (props.pw.min(pw_cap_safety) - 0.001);
        let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term_damage);
        let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term_safety);

        assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
        assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
    }

    #[test]
    fn test_high_strength_offset_differs_from_normal_short_term() {
        // pw を 0.001 < pw < 0.002 の範囲に設定する。普通強度式（offset=0.002）
        // では pw 項が 0 のままだが、高強度式（短期 offset=0.001）では
        // pw 項が有効になり QA が普通強度より大きくなることを確認する。
        let shape =
            rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 600.0, 2, "KH785");
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        assert!(
            props.pw > 0.001 && props.pw < 0.002,
            "テストの前提として 0.001 < pw < 0.002 が必要: pw={}",
            props.pw
        );

        let allow_normal = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        let mut allow_hs = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        allow_hs.w_ft = high_strength_w_ft("KH785", false);
        let alpha = 1.3;

        let qa_normal = shear_capacity(&props, &allow_normal, alpha, LoadTerm::Short, true, false);
        let qa_hs = shear_capacity_high_strength(
            &props,
            &allow_hs,
            alpha,
            LoadTerm::Short,
            true,
            false,
            "KH785",
            24.0,
        );

        assert!(
            qa_hs > qa_normal,
            "高強度式は pw 項が有効になり普通強度式より大きいはず: normal={qa_normal}, hs={qa_hs}"
        );
    }

    #[test]
    fn test_high_strength_shear_capacity_long_term_matches_normal_formula() {
        // 長期は普通強度と同じ式（offset=0.002, pw 上限 0.6%）で、
        // w_ft も高強度テーブル値=195 と SD345 長期値=195 が一致するため、
        // 高強度パスと普通強度パスの結果は一致するはず。
        let shape =
            rc_rect_shape_with_shear_grade(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2, "UB785");
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);

        let mut allow_hs = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
        allow_hs.w_ft = high_strength_w_ft("UB785", true);
        let allow_normal = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
        let alpha = 1.3;

        let qa_hs = shear_capacity_high_strength(
            &props,
            &allow_hs,
            alpha,
            LoadTerm::Long,
            true,
            false,
            "UB785",
            24.0,
        );
        let qa_normal = shear_capacity(&props, &allow_normal, alpha, LoadTerm::Long, true, false);

        assert!((qa_hs - qa_normal).abs() / qa_normal < 1e-9);
    }

    #[test]
    fn test_high_strength_column_safety_check_excludes_alpha() {
        let shape = rc_rect_shape_with_shear_grade(
            400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, "SHD685",
        );
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props_strong(&make_section(shape.clone()), &rebar);
        let mut allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        allow.w_ft = high_strength_w_ft("SHD685", false);

        let qa_alpha_1 = shear_capacity_high_strength(
            &props,
            &allow,
            1.0,
            LoadTerm::Short,
            false,
            true,
            "SHD685",
            24.0,
        );
        let qa_alpha_1_5 = shear_capacity_high_strength(
            &props,
            &allow,
            1.5,
            LoadTerm::Short,
            false,
            true,
            "SHD685",
            24.0,
        );
        // 柱の安全確保のための検討式は高強度でも α を含まない。
        assert!((qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6);
    }

    #[test]
    fn test_shear_capacity_for_none_delegates_to_normal_regression() {
        // grade=None のディスパッチが普通強度の既存関数と完全に一致すること
        // （既存挙動の回帰確認）。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);

        let via_dispatch = shear_capacity_for(
            &props,
            &allow,
            1.3,
            LoadTerm::Short,
            true,
            false,
            None,
            24.0,
        );
        let via_direct = shear_capacity(&props, &allow, 1.3, LoadTerm::Short, true, false);
        assert!((via_dispatch - via_direct).abs() < 1e-12);
    }

    // ------------------------------------------------------------------
    // 軽量コンクリート（許容応力度 0.9 倍・高強度フープとの併用）
    // ------------------------------------------------------------------

    #[test]
    fn test_effective_damage_control_lightweight_high_strength() {
        // 高強度フープ + 軽量 → 強制 false。
        assert!(!effective_damage_control(
            true,
            Some("KH785"),
            ConcreteClass::Lightweight1
        ));
        assert!(!effective_damage_control(
            true,
            Some("UB785"),
            ConcreteClass::Lightweight2
        ));
        // 高強度フープ + 普通 → 指定どおり。
        assert!(effective_damage_control(
            true,
            Some("KH785"),
            ConcreteClass::Normal
        ));
        // 普通強度フープ（grade=None）は軽量でも指定どおり。
        assert!(effective_damage_control(
            true,
            None,
            ConcreteClass::Lightweight1
        ));
    }

    // ------------------------------------------------------------------
    // フォールバック・RcDesign 統合（振り分け全体の確認）
    // ------------------------------------------------------------------

    #[test]
    fn test_fc_missing_fallback() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let design = RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("Fc"));
    }

    #[test]
    fn test_shape_missing_fallback() {
        // shape を持たない Section（数値直入力等）。
        let sec = Section {
            id: SectionId(0),
            name: "no-shape".to_string(),
            area: 300.0 * 600.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 600.0,
            width: 300.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let design = RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("配筋情報なし"));
    }

    #[test]
    fn test_rc_circle_beam_and_column_smoke() {
        let shape = SectionShape::RcCircle {
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 1,
                    grade: None,
                },
            },
        };
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let design = RcDesign;

        let forces = MemberForcesAt {
            pos: 0.0,
            n: -200_000.0,
            qy: 30_000.0,
            qz: 20_000.0,
            my: 10_000_000.0,
            mz: 20_000_000.0,
        };

        let ctx_col = ctx_column(LoadTerm::Short);
        let r_col = design.check(&forces, &sec, &mat, &ctx_col);
        assert!(r_col.ratio.is_finite() && r_col.ratio >= 0.0);
        assert!(r_col.basis.contains("円形柱"));

        let ctx_b = ctx_beam(LoadTerm::Short);
        let r_beam = design.check(&forces, &sec, &mat, &ctx_b);
        assert!(r_beam.ratio.is_finite() && r_beam.ratio >= 0.0);
    }
}
