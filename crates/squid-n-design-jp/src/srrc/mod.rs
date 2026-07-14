//! SRC 造の断面検定（許容応力度検定。SRC 規準 1987 の
//! SRC 梁・SRC 柱部分に準拠）。
//!
//! SRC = 鉄骨鉄筋コンクリート造（Steel Reinforced Concrete）。ディレクトリ名を
//! `src` ではなく `srrc` としているのは、Rust の慣例的なソースルート
//! `crates/squid-n-design-jp/src/` と名前が衝突するため（`srrc` = SRC 造の意）。
//!
//! 準拠する規準:
//! - 日本建築学会「鉄骨鉄筋コンクリート構造計算規準・同解説」
//!   （SRC 規準 1987年版）の累加強度式、および構造規定。
//!
//! # 材料の扱い
//! - `SrcRect`: コンクリート強度 = `Material.fc`、主筋グレード = `Material.name`
//!   （RC の慣習を踏襲）、内蔵鉄骨の鋼種 = `SectionShape::SrcRect.steel_grade`。
//! - `Material.fc` が `None`/0 の場合は検定をスキップする（`ok=true`,
//!   `basis` に "Fc未設定" と記載）。
//! - 鋼材グレードが [`crate::steel::steel_f_value_prefix`] で解決できない
//!   場合は SS400 相当（F=235）にフォールバックする（安全側とは限らないため
//!   実運用では鋼種名を確認すること）。
//!
//! # SRC規準1987 からの主な簡略化（doc 内に個別関数でも記載）
//! 1. SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式）は実装済み
//!    （[`src_seismic_qd`] 参照。`ctx.seismic_qd` が Some で当該評価位置の
//!    長期内力が見つかる場合のみ有効。それ以外（長期・積雪時・暴風時、
//!    または長期内力が未提供）は従来どおり弾性分担のみで比較する）。
//!    ただし以下はなお簡略化している:
//!    - SRC 規準 1987 が定める「構造規定方式」と「SRC 規準方式」のうち
//!      構造規定方式のみを実装し、選択機能は設けない。
//!    - `sM1+sM2 = 2・sZ・sft` は部材両端が同一鉄骨・sft（短期許容引張
//!      応力度）に達するとみなす近似（本来は許容"曲げモーメント"
//!      `sMA` を用いるべきだが、SRC 柱の `sMA(N)` は軸力依存の複雑な
//!      3 分岐式であり、部材端ごとに軸力が異なりうるため、鉄骨単体の
//!      全塑性相当値 `sZ・sft` で代替する安全側とは限らない近似とする）。
//!    - `rMu1+rMu2 = 2・rMu` も同様に部材両端同一断面・同一設計軸力の
//!      仮定（[`squid_n_core::rc_capacity::rc_mu_simple`]/
//!      [`squid_n_core::rc_capacity::rc_column_mu_simple`] を使用）。
//! 2. SRC 柱の内蔵鉄骨の `s_fc` は、被覆コンクリートによる拘束で単材座屈が
//!    生じにくいことから `s_fc = s_ft` のままとする（SRC 規準の座屈検討は
//!    別途必要になり得る）。
//! 3. SRC 柱の RC 部分の中立軸圧縮側鉄骨面積 `s_ac`（fc′ 低減用）は
//!    軸に依らず `steel_width・steel_flange_thick` の一つの値を用いる
//!    （本来は曲げ軸ごとに異なりうる）。
//! 4. SRC 柱のせん断は強軸・弱軸を対称的に扱うため、RC 柱検定（`rc/`）と
//!    同様に「b/D 入れ替え」の近似を用いる。
//!
//! # モジュール構成（断面検定の項目に対応）
//! - 本ファイル（`srrc/mod.rs`）: 共通の断面諸元抽出・せん断の鉄骨/RC
//!   弾性分担・地震時短期の設計用せん断力（構造規定方式）・`SrcDesign`
//!   （`DesignCheck` 実装、梁/柱への振り分け）。
//! - [`beam`][]: 鉄骨鉄筋コンクリート造梁の断面検定（累加強度式 MA=sMo+rMA）。
//! - [`column`][]: 鉄骨鉄筋コンクリート造柱の断面検定（累加強度式・fc′低減）。
//! - [`panel_zone`][]: SRC 造柱梁接合部（パネルゾーン）の断面検定（SRC 規準）。

use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind, QdMethod};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

mod beam;
/// 鉄骨鉄筋コンクリート造梁のせん断終局強度（非線形解析のせん断ばね終局耐力）。
/// SRC 梁せん断復元力特性（構造関係技術基準解説書・SRC 規準）。
pub mod beam_nonlinear;
mod column;
pub mod panel_zone;

// ============================================================================
// 0. 共通ヘルパ
// ============================================================================

/// 主筋 1 本あたりの断面積 [mm²]。
fn one_bar_area(dia: f64) -> f64 {
    let r = dia / 2.0;
    std::f64::consts::PI * r * r
}

/// 主筋セットの総断面積 [mm²]。
fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * one_bar_area(bar.dia)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]（`rc/mod.rs` の `tension_dt` と同じ
/// 考え方。private のため自前実装する）。
fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if main.layers <= 1 {
        return k1;
    }
    let k_prime = 25.0_f64.max(1.5 * main.dia);
    let s = main.dia + k_prime;
    k1 + (main.layers as f64 - 1.0) / 2.0 * s
}

/// せん断補強筋比 pw = (legs・π/4・dia²) / (b・pitch)。
fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw = shear.legs as f64 * std::f64::consts::PI / 4.0 * shear.dia * shear.dia;
    aw / (b * shear.pitch)
}

/// せん断スパン比による割増係数 α = 4/(M/(Q・d)+1)（`max_alpha` でクランプ、
/// 下限 1.0）。
fn shear_alpha_src(m: f64, q: f64, d: f64, max_alpha: f64) -> f64 {
    if q.abs() < 1e-9 || d <= 0.0 {
        return max_alpha;
    }
    let mqd = m.abs() / (q.abs() * d);
    (4.0 / (mqd + 1.0)).clamp(1.0, max_alpha)
}

/// MA<=0 の場合に検定比が発散しないよう、大きな有限値で代用する。
fn ratio_or_large(m: f64, ma: f64) -> f64 {
    if ma > 1e-9 {
        m.abs() / ma
    } else if m.abs() > 1e-9 {
        1.0e9
    } else {
        0.0
    }
}

/// 矩形断面 1 軸分の断面諸元（`rc/mod.rs` の `AxisProps`/`rect_axis_props` と
/// 同じ考え方）。
#[derive(Clone, Copy)]
struct SrcAxisProps {
    b: f64,
    d_full: f64,
    dt: f64,
    d: f64,
    at: f64,
    ac: f64,
    j: f64,
    pw: f64,
}

fn src_rect_axis_props(
    width_dir_b: f64,
    depth_dir_d: f64,
    main: &BarSet,
    rebar: &RcRebar,
) -> SrcAxisProps {
    let dt = tension_dt(rebar.cover, rebar.shear.dia, main);
    let d = depth_dir_d - dt;
    let at = bar_set_area(main) / 2.0;
    SrcAxisProps {
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

/// 内蔵鋼材の断面積・断面係数を [`SectionShape`] の断面性能計算を借りて
/// 求める（H 形鋼: `sA`, 強軸 `sZ`, 弱軸 `sZ`）。
fn steel_h_props(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> (f64, f64, f64) {
    let shape = SectionShape::SteelH {
        height,
        width,
        web_thick,
        flange_thick,
    };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let iz = shape.calc_iz();
    let sz_strong = if height > 0.0 { iy * 2.0 / height } else { 0.0 };
    let sz_weak = if width > 0.0 { iz * 2.0 / width } else { 0.0 };
    (a, sz_strong, sz_weak)
}

// ============================================================================
// 1. せん断の鉄骨/RC 弾性分担・地震時短期の設計用せん断力（構造規定方式）
// （SRC 梁・柱の両方から共通利用する）
// ============================================================================

struct SrcShearResult {
    ratio: f64,
    s_q: f64,
    r_q: f64,
    s_qa: f64,
    r_qa: f64,
    alpha: f64,
    pw: f64,
    /// 地震時短期の構造規定方式（[`src_seismic_qd`]）で `s_q`/`r_q` を
    /// 算定したか（false の場合は従来の弾性分担）。
    used_qd: bool,
}

/// SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式）算定に必要な
/// 追加入力（[`src_seismic_qd`] 参照）。
struct SrcSeismicCtx<'a> {
    ctx: &'a DesignCtx,
    /// 評価位置（`ctx.seismic_qd.long_at` 検索用）。
    pos: f64,
    /// 長期内力配列 `[N,Qy,Qz,Mx,My,Mz]` のせん断成分位置（qy=1, qz=2）。
    q_index: usize,
    /// 鋼材の短期許容引張応力度 sft [N/mm²]
    /// （`steel_ft(f_value, LoadTerm::Short)`。長短期どちらの検定でも
    /// QD 割増自体は短期時のみ発動するため、常に短期値を用いる）。
    s_ft_short: f64,
    /// RC 部分の終局曲げモーメント rMu（部材端 1 箇所分）[N·mm]
    /// （[`squid_n_core::rc_capacity::rc_mu_simple`]/
    /// [`squid_n_core::rc_capacity::rc_column_mu_simple`] で算定）。
    /// 0 以下なら rQD1 は無効（rQD2 のみ）とする。
    r_mu: f64,
}

/// SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式、SRC 規準 1987）。
/// `seismic.ctx.seismic_qd` が None、または長期内力に
/// 同一評価位置が見つからない場合は None を返す（呼び出し側は従来の弾性
/// 分担にフォールバックする）。
///
/// - `sQD = sQL + (sM1+sM2)/l′`（`sQL = share・|QL|`、`sM1+sM2 = 2・sZ・sft`）
/// - `rQD1 = rQL + (rMu1+rMu2)/l′`（`rQL = |QL| − sQL`、`rMu1+rMu2 = 2・rMu`）
/// - `rQD2 = max(0, n・(|Q| − sQD))`
///   （SRC規準1987 の `rQD2 = n・(QL+QE−sQD)` を、`QL+QE` = 当該組合せの
///   全せん断力 `|Q|` と読んだもの。`QE` は水平力分のせん断力増分であり、
///   `QL+QE` はその和として組合せ後の全せん断力に一致するとみなした）
/// - `rQD = min(rQD1, rQD2)`（[`QdMethod::Qd1`]/[`QdMethod::Qd2`] 選択時は
///   それぞれ単独。`QdMethod::Qd1` で rQD1 が無効な場合は rQD2 で代替する）
/// - 戻り値は `(sQD, rQD)`。
fn src_seismic_qd(
    seismic: &SrcSeismicCtx,
    q_signed: f64,
    share: f64,
    sz: f64,
) -> Option<(f64, f64)> {
    let qd = seismic.ctx.seismic_qd.as_ref()?;
    let ql_signed = qd
        .long_at
        .iter()
        .find(|(p, _)| (p - seismic.pos).abs() < 1e-6)
        .map(|(_, f)| f[seismic.q_index])?;

    let ql = ql_signed.abs();
    let q = q_signed.abs();

    let s_ql = share * ql;
    let sum_s_m = 2.0 * sz * seismic.s_ft_short;
    let s_qd = if qd.clear_length > 0.0 {
        s_ql + sum_s_m / qd.clear_length
    } else {
        s_ql
    };

    let r_ql = (ql - s_ql).max(0.0);
    let r_qd1 = if qd.clear_length > 0.0 && seismic.r_mu > 0.0 {
        let sum_r_mu = 2.0 * seismic.r_mu;
        r_ql + sum_r_mu / qd.clear_length
    } else {
        f64::INFINITY
    };
    let r_qd2 = (qd.n_factor * (q - s_qd)).max(0.0);

    let r_qd = match qd.method {
        QdMethod::Qd1 => {
            if r_qd1.is_finite() {
                r_qd1
            } else {
                r_qd2
            }
        }
        QdMethod::Qd2 => r_qd2,
        QdMethod::Min => r_qd1.min(r_qd2),
    };

    Some((s_qd, r_qd))
}

/// せん断検定の部材モード（SRC規準 1987 の梁・柱の式の切替）。
enum SrcShearMode {
    /// 梁: 鉄骨・RC の分担検定（長短共通）。
    /// `rQA1 = b·rj·(rα·fs + 0.5·pw·wft)`、`rQA2 = b·rj·(2(b′/b)·fs + pw·wft)`。
    Beam,
    /// 柱（SRC規準 P.96-97）:
    /// - 長期は併用式 `QA = (1+β)·b·rj·a′·fs` を全せん断力と比較する
    ///   （`a′ = rα`（`b′/b ≥ rα/3` のとき）または `3b′/b`、`1 ≤ rα ≤ 2`）。
    /// - 短期は鉄骨部 `sQA`（強軸 `dw·tw·sfs`／弱軸 `(4/3)·bf·tf·sfs`）と
    ///   RC 部 `rQAS1 = b·rj·(fs + 0.5·pw·wft)`（**α を含まない**）・
    ///   `rQAS2 = b·rj·(2(b′/b)·fs + pw·wft)` を分担 `sQD`/`rQD` と比較する。
    ///
    /// `beta` は鉄骨ウェブの形式と寸法による係数
    /// （充腹 `β = n·tw·sd/(b·rj)`、弱軸・非充腹 `β = 1.33·n·bf·tf/(b·rj)`）。
    Column { beta: f64 },
}

/// 全せん断力を鉄骨部分・RC 部分に分担させ、それぞれの許容せん断力と比較する
/// （梁・柱の両方向で共通利用。式の切替は [`SrcShearMode`]）。
/// `seismic.ctx.seismic_qd` が Some で当該評価位置の長期内力が見つかる場合は
/// 地震時短期の構造規定方式（[`src_seismic_qd`]）による設計用せん断力
/// `sQD`/`rQD` を用い、それ以外は弾性分担 `share = sz/(sz+at・rj)` を当該
/// 組合せの全せん断力にそのまま適用して代替する。
#[allow(clippy::too_many_arguments)]
fn src_shear_check(
    q_signed: f64,
    m_for_alpha: f64,
    q_for_alpha: f64,
    sz: f64,
    at: f64,
    rj: f64,
    rd: f64,
    b: f64,
    b_prime: f64,
    pw_raw: f64,
    fs: f64,
    w_ft: f64,
    s_fs: f64,
    steel_shear_area: f64,
    alpha_max: f64,
    mode: &SrcShearMode,
    seismic: &SrcSeismicCtx,
) -> SrcShearResult {
    let alpha = shear_alpha_src(m_for_alpha, q_for_alpha, rd, alpha_max);
    let q = q_signed.abs();

    // SRC 規準1987 準拠: 「pw が 0.6% を超える場合は 0.6% として算定する」
    // （長期・短期の区別は記載されていないため、長短期とも 0.6% を上限とする。
    // RC の短期 1.2% とは異なる点に注意）。
    let pw_cap = 0.006;
    let pw = pw_raw.min(pw_cap);

    let b_ratio = if b > 1e-9 {
        (b_prime / b).max(0.0)
    } else {
        0.0
    };

    // 柱・長期: 併用式 QA = (1+β)·b·rj·a′·fs（SRC規準 P.96-97）。
    // 鉄骨・RC の分担ではなく、β で鉄骨ウェブの寄与を見込んだ全体式を
    // 全せん断力と比較する。
    if let SrcShearMode::Column { beta } = mode {
        if seismic.ctx.term == LoadTerm::Long {
            let a_prime = if b_ratio >= alpha / 3.0 {
                alpha
            } else {
                3.0 * b_ratio
            };
            let qa = (1.0 + beta) * b * rj * a_prime * fs;
            let ratio = if qa > 1e-9 { q / qa } else { 0.0 };
            return SrcShearResult {
                ratio,
                s_q: 0.0,
                r_q: q,
                s_qa: steel_shear_area * s_fs,
                r_qa: qa,
                alpha,
                pw,
                used_qd: false,
            };
        }
    }

    let denom = sz + at * rj;
    let share = if denom > 1e-12 { sz / denom } else { 1.0 };

    let (s_q, r_q, used_qd) = match src_seismic_qd(seismic, q_signed, share, sz) {
        Some((s_qd, r_qd)) => (s_qd, r_qd, true),
        None => {
            let s_q = share * q;
            (s_q, (q - s_q).max(0.0), false)
        }
    };

    let s_qa = steel_shear_area * s_fs;

    // RC 部の許容せん断力。柱の短期 rQAS1 は α を含まない（SRC規準。
    // α を乗じると許容を最大2倍に過大評価する非保守側の誤りとなる）。
    // 梁は長短とも rα（α_L/α_S1/α_S2）を含む。
    let r_qa1 = match mode {
        SrcShearMode::Beam => b * rj * (alpha * fs + 0.5 * pw * w_ft),
        SrcShearMode::Column { .. } => b * rj * (fs + 0.5 * pw * w_ft),
    };
    let r_qa2 = b * rj * (2.0 * b_ratio * fs + pw * w_ft);
    let r_qa = r_qa1.min(r_qa2);

    let ratio_s = if s_qa > 1e-9 { s_q / s_qa } else { 0.0 };
    let ratio_r = if r_qa > 1e-9 { r_q / r_qa } else { 0.0 };

    SrcShearResult {
        ratio: ratio_s.max(ratio_r),
        s_q,
        r_q,
        s_qa,
        r_qa,
        alpha,
        pw,
        used_qd,
    }
}

// ============================================================================
// 2. DesignCheck 実装（梁は srrc/beam.rs、柱は srrc/column.rs へ振り分け）
// ============================================================================

/// SRC 梁・SRC 柱の断面検定（`SectionShape::SrcRect` を対象とする）。
pub struct SrcDesign;

impl DesignCheck for SrcDesign {
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
                basis: "SRC検定: Fc未設定".to_string(),
                detail: "Material.fc が None/0 のため検定をスキップしました。".to_string(),
            };
        }

        let shape = match &sec.shape {
            Some(s @ SectionShape::SrcRect { .. }) => s,
            _ => {
                return CheckResult {
                    ratio: 0.0,
                    ok: true,
                    basis: "SRC検定: 断面形状不一致".to_string(),
                    detail: "Section.shape が SrcRect ではないため検定をスキップしました。"
                        .to_string(),
                };
            }
        };

        let SectionShape::SrcRect {
            b,
            d,
            rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            steel_grade,
        } = shape
        else {
            unreachable!()
        };

        match ctx.kind {
            MemberKind::Beam | MemberKind::Brace => beam::src_beam_check(
                forces,
                mat,
                ctx,
                *b,
                *d,
                rebar,
                *steel_height,
                *steel_width,
                *steel_web_thick,
                *steel_flange_thick,
                steel_grade,
                fc_raw,
            ),
            MemberKind::Column => column::src_column_check(
                forces,
                mat,
                ctx,
                *b,
                *d,
                rebar,
                *steel_height,
                *steel_width,
                *steel_web_thick,
                *steel_flange_thick,
                steel_grade,
                fc_raw,
            ),
        }
    }
}

// ============================================================================
// テスト（共通ヘルパ・せん断弾性分担・地震時設計用せん断力・DesignCheck 振り
// 分けの共通経路）
// ============================================================================

#[cfg(test)]
pub(crate) mod tests;
