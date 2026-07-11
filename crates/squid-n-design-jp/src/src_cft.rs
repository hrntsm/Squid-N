//! SRC 造・CFT 造の断面検定（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」の SRC 梁・SRC 柱・CFT 柱部分に準拠）。
//!
//! 準拠する規準:
//! - SRC 梁・SRC 柱: 日本建築学会「鉄骨鉄筋コンクリート構造計算規準・同解説」
//!   （SRC 規準 1987年版）の累加強度式、および構造規定。
//! - CFT 柱: SRC 規準の累加強度式の考え方を CFT 断面（コンクリート充填鋼管）に
//!   適用したもの。相互拘束効果によるコンクリート強度割増しは考慮しない
//!   （非拘束・安全側の仮定）。
//!
//! # 材料の扱い
//! - `SrcRect`: コンクリート強度 = `Material.fc`、主筋グレード = `Material.name`
//!   （RC の慣習を踏襲）、内蔵鉄骨の鋼種 = `SectionShape::SrcRect.steel_grade`。
//! - `CftBox`/`CftPipe`: 鋼種 = `Material.name`、充填コンクリート強度 =
//!   `Material.fc`。
//! - `Material.fc` が `None`/0 の場合は検定をスキップする（`ok=true`,
//!   `basis` に "Fc未設定" と記載）。
//! - 鋼材グレードが [`crate::steel::steel_f_value_prefix`] で解決できない
//!   場合は SS400 相当（F=235）にフォールバックする（安全側とは限らないため
//!   実運用では鋼種名を確認すること）。
//!
//! # マニュアルからの主な簡略化（doc 内に個別関数でも記載）
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
//! 2. CFT 柱の設計用せん断力は `QD2 = |QL| + n・|Q−QL|` のみ実装し、
//!    `QD1`（複合断面の終局曲げによる算定、`ΣcMy/h′`）は実装しない
//!    （CFT の終局曲げは鋼管・充填コンクリートの複合断面として別途
//!    定式化が必要なため）。`ctx.seismic_qd` が Some の場合は常に QD2 を
//!    用いる（[`cft_q_design`] 参照）。
//! 3. CFT 柱の鋼管部分の許容圧縮応力度 `s_fc` は座屈を考慮する
//!    （λ = lk/i を**鋼管単体**の断面二次半径で評価。充填コンクリートの
//!    剛性寄与を無視するため安全側。[`cft_common_steel`] 参照）。
//!    SRC 柱の内蔵鉄骨の `s_fc` は、被覆コンクリートによる拘束で単材座屈が
//!    生じにくいことから `s_fc = s_ft` のままとする（SRC 規準の座屈検討は
//!    別途必要になり得る）。
//! 4. SRC 柱の RC 部分の中立軸圧縮側鉄骨面積 `s_ac`（fc′ 低減用）は
//!    軸に依らず `steel_width・steel_flange_thick` の一つの値を用いる
//!    （本来は曲げ軸ごとに異なりうる）。
//! 5. SRC 柱・CFT 柱のせん断は強軸・弱軸を対称的に扱うため、RC 柱検定
//!    （`rc.rs`）と同様に「b/D 入れ替え」の近似を用いる。
//! 6. CFT 円形柱の (N,M) 相関は閉形式を用いず、縁応力一定の弾性三角形
//!    分布を断面内で数値積分して求める（矩形の閉形式と同じ弾性仮定）。

use crate::rc::{
    concrete_allowable_compression_class, concrete_allowable_shear_class, rebar_allowable_shear,
    rebar_allowable_tension, young_ratio_n,
};
use crate::steel::{steel_f_value_prefix, steel_fs, steel_ft};
use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind, QdMethod};
use squid_n_core::model::{Material, Section};
use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

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

/// 主筋の降伏点 σy [N/mm²]（終局曲げ算定用。`rc.rs` の private 関数
/// `rebar_sigma_y` と同ロジック）。
///
/// `Material.fy` があればそれを、無ければ材料名（鉄筋グレード名）の数値部
/// （例 "SD345"→345）を、どちらも無ければ 345（SD345 相当）を用いる。
fn rebar_sigma_y(mat: &Material) -> f64 {
    if let Some(fy) = mat.fy {
        if fy > 0.0 {
            return fy;
        }
    }
    let digits: String = mat.name.chars().filter(|c| c.is_ascii_digit()).collect();
    digits
        .parse::<f64>()
        .ok()
        .filter(|v| *v > 0.0)
        .unwrap_or(345.0)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]（`rc.rs` の `tension_dt` と同じ
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

/// 矩形断面 1 軸分の断面諸元（`rc.rs` の `AxisProps`/`rect_axis_props` と
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

/// 内蔵/充填鋼材の断面積・断面係数を [`SectionShape`] の断面性能計算を
/// 借りて求める（H 形鋼: `sA`, 強軸 `sZ`, 弱軸 `sZ`）。
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
// 1. SRC 梁（SRC 規準 1987・累加強度式）
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

/// SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式、RESP-D マニュアル
/// 「04 断面検定」）。`seismic.ctx.seismic_qd` が None、または長期内力に
/// 同一評価位置が見つからない場合は None を返す（呼び出し側は従来の弾性
/// 分担にフォールバックする）。
///
/// - `sQD = sQL + (sM1+sM2)/l′`（`sQL = share・|QL|`、`sM1+sM2 = 2・sZ・sft`）
/// - `rQD1 = rQL + (rMu1+rMu2)/l′`（`rQL = |QL| − sQL`、`rMu1+rMu2 = 2・rMu`）
/// - `rQD2 = max(0, n・(|Q| − sQD))`
///   （マニュアルの `rQD2 = n・(QL+QE−sQD)` を、`QL+QE` = 当該組合せの
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

/// 全せん断力を鉄骨部分・RC 部分に分担させ、それぞれの許容せん断力と比較する
/// （梁・柱の両方向で共通利用）。`seismic.ctx.seismic_qd` が Some で当該評価
/// 位置の長期内力が見つかる場合は地震時短期の構造規定方式（[`src_seismic_qd`]）
/// による設計用せん断力 `sQD`/`rQD` を用い、それ以外は SRC 規準・構造規定の
/// 長期式の一般化（弾性分担 `share = sz/(sz+at・rj)` を当該組合せの全せん断力
/// にそのまま適用）で代替する。
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
    seismic: &SrcSeismicCtx,
) -> SrcShearResult {
    let alpha = shear_alpha_src(m_for_alpha, q_for_alpha, rd, alpha_max);
    let q = q_signed.abs();

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

    // SRC 規準1987 準拠: 「pw が 0.6% を超える場合は 0.6% として算定する」
    // （RESP-D マニュアル「04 断面検定」SRC 部分。長期・短期の区別は記載
    // されていないため、長短期とも 0.6% を上限とする。RC の短期 1.2% とは
    // 異なる点に注意）。
    let pw_cap = 0.006;
    let pw = pw_raw.min(pw_cap);

    let r_qa1 = b * rj * (alpha * fs + 0.5 * pw * w_ft);
    let b_ratio = if b > 1e-9 {
        (b_prime / b).max(0.0)
    } else {
        0.0
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

/// SRC 梁の断面検定。曲げは `MA = sMo + rMA`（単純累加式）、せん断は
/// 鉄骨・RC への弾性分担＋各許容せん断力の比較で行う。
#[allow(clippy::too_many_arguments)]
fn src_beam_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    b: f64,
    d_full: f64,
    rebar: &RcRebar,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
    steel_grade: &str,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();

    // 軽量コンクリート1種・2種は許容応力度を 0.9 倍に低減（マニュアル
    // 「04 断面検定」。`mat.concrete_class` を考慮した class 対応版を使用）。
    let fs = concrete_allowable_shear_class(fc_raw, mat.concrete_class, long_term);
    let shear_grade = rebar
        .shear
        .grade
        .clone()
        .unwrap_or_else(|| grade.to_string());
    let w_ft = rebar_allowable_shear(&shear_grade, long_term);
    let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

    let thickness = steel_web_thick.max(steel_flange_thick);
    let f_value = steel_f_value_prefix(steel_grade, thickness).unwrap_or(235.0);
    let s_ft = steel_ft(f_value, ctx.term);
    let s_fs = steel_fs(f_value, ctx.term);

    let (_sa, sz, _sz_weak) = steel_h_props(
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    );

    let props = src_rect_axis_props(b, d_full, &rebar.main_x, rebar);

    let s_mo = sz * s_ft;
    let r_ma = props.at * ft * props.j;
    let ma = s_mo + r_ma;

    let ratio_m = ratio_or_large(forces.mz, ma);

    let (m_alpha, q_alpha) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let b_prime = (b - steel_width).max(0.0);
    let dw = steel_height - 2.0 * steel_flange_thick;

    // 地震時短期の設計用せん断力（構造規定方式）: rMu は両端同一断面・
    // 対称配筋（at=ac）の仮定で `rc_mu_simple` により算定する。
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let r_mu = rc_mu_simple(&RcCapacityInput {
        b: props.b,
        d: props.d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y: rebar_sigma_y(mat),
        fc: fc_raw,
        pw: props.pw,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    });
    let seismic = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 1,
        s_ft_short,
        r_mu,
    };

    let shear = src_shear_check(
        forces.qy,
        m_alpha,
        q_alpha,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        b_prime,
        props.pw,
        fs,
        w_ft,
        s_fs,
        steel_web_thick * dw,
        2.0,
        &seismic,
    );

    let ratio = ratio_m.max(shear.ratio);

    let basis = "SRC規準(1987) 梁: 累加強度式(曲げ)+ せん断弾性分担".to_string();
    let qd_note = if shear.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let detail = format!(
        "sMo={:.1} N·mm, rMA={:.1} N·mm, MA={:.1} N·mm, |mz|={:.1} N·mm, \
         sQ={:.1} N, rQ={:.1} N, sQA={:.1} N, rQA={:.1} N, α={:.3}, pw={:.5}, \
         設計用せん断力={qd_note}",
        s_mo,
        r_ma,
        ma,
        forces.mz,
        shear.s_q,
        shear.r_q,
        shear.s_qa,
        shear.r_qa,
        shear.alpha,
        shear.pw
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
    }
}

// ============================================================================
// 2. SRC 柱（SRC 規準 1987・累加強度式）
// ============================================================================

struct SrcColumnAxis {
    props: SrcAxisProps,
    /// 直交方向の主筋総断面積（断面中央に集約、RC 規準 14条の慣習）。
    at_perp: f64,
    /// 当該軸の主筋径に応じた許容引張・圧縮応力度。
    ft: f64,
}

/// 中立軸位置 xn における RC 部分の (N, |M|) を求める（`rc.rs` の
/// `column_nm_at_xn` と同じ考え方。コンクリート許容応力度に `fc_prime`
/// （鉄骨フランジ食い込みによる低減後）を用いる点のみ異なる）。
fn src_column_nm_at_xn(
    axis: &SrcColumnAxis,
    fc_prime: f64,
    n_ratio: f64,
    xn: f64,
) -> Option<(f64, f64)> {
    if xn <= 0.0 {
        return None;
    }
    let p = &axis.props;
    let d_full = p.d_full;
    let b = p.b;
    let r_fc = axis.ft;
    let ft = axis.ft;

    let s_bar = |y: f64, area: f64| -> f64 {
        if area <= 0.0 {
            return f64::INFINITY;
        }
        let diff = xn - y;
        if diff.abs() < 1e-9 {
            return f64::INFINITY;
        }
        if diff > 0.0 {
            r_fc / (n_ratio * diff)
        } else {
            ft / (n_ratio * (-diff))
        }
    };

    let s1 = fc_prime / xn;
    let s2 = s_bar(p.dt, p.ac);
    let s3 = s_bar(d_full - p.dt, p.at);
    let s = s1.min(s2).min(s3);
    if !s.is_finite() || s <= 0.0 {
        return None;
    }

    let xc = xn.min(d_full);
    if xc <= 0.0 {
        return None;
    }

    let nc = b * s * (xn * xc - xc * xc / 2.0);
    let mc =
        b * s * (xn * (d_full / 2.0) * xc - (xn + d_full / 2.0) * xc * xc / 2.0 + xc.powi(3) / 3.0);

    let bar_contrib = |y: f64, area: f64| -> (f64, f64) {
        if area <= 0.0 {
            return (0.0, 0.0);
        }
        let mult = if y <= xn { n_ratio - 1.0 } else { n_ratio };
        let force = area * mult * s * (xn - y);
        let moment = force * (d_full / 2.0 - y);
        (force, moment)
    };

    let (n_c, m_c) = bar_contrib(p.dt, p.ac);
    let (n_t, m_t) = bar_contrib(d_full - p.dt, p.at);
    let (n_p, m_p) = bar_contrib(d_full / 2.0, axis.at_perp);

    let n_total = nc + n_c + n_t + n_p;
    let m_total = mc + m_c + m_t + m_p;
    Some((n_total, m_total.abs()))
}

const SRC_XN_SCAN_POINTS: usize = 300;
const SRC_XN_RATIO_MIN: f64 = 0.02;
const SRC_XN_RATIO_MAX: f64 = 10.0;

/// N-M 相関曲線（RC 部分のみ）を xn/D の対数スキャンで構成する。
fn src_column_nm_curve(
    axis: &SrcColumnAxis,
    fc_prime: f64,
    n_ratio: f64,
    rnc: f64,
) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(SRC_XN_SCAN_POINTS + 1);
    let log_min = SRC_XN_RATIO_MIN.ln();
    let log_max = SRC_XN_RATIO_MAX.ln();
    for i in 0..SRC_XN_SCAN_POINTS {
        let t = i as f64 / (SRC_XN_SCAN_POINTS as f64 - 1.0);
        let ratio = (log_min + t * (log_max - log_min)).exp();
        let xn = axis.props.d_full * ratio;
        if let Some(pt) = src_column_nm_at_xn(axis, fc_prime, n_ratio, xn) {
            if pt.0.is_finite() && pt.1.is_finite() {
                pts.push(pt);
            }
        }
    }
    pts.push((rnc, 0.0));
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    pts
}

/// N-M 相関曲線から設計軸力（圧縮正）に対する許容曲げモーメントを線形補間で
/// 求める。範囲外は端点値でクランプする。
fn interp_ma_curve(points: &[(f64, f64)], n_design: f64) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    if n_design <= points[0].0 {
        return points[0].1;
    }
    let last = points.len() - 1;
    if n_design >= points[last].0 {
        return points[last].1;
    }
    for w in points.windows(2) {
        let (n0, m0) = w[0];
        let (n1, m1) = w[1];
        if n_design >= n0 && n_design <= n1 {
            if (n1 - n0).abs() < 1e-9 {
                return m0.max(m1);
            }
            let t = (n_design - n0) / (n1 - n0);
            return m0 + t * (m1 - m0);
        }
    }
    points[last].1
}

/// SRC 柱 1 軸分の許容曲げモーメント MA(N)。マニュアルの 3 分岐
/// （RC+鉄骨累加 / 圧縮超過で鉄骨のみ / 引張超過で鉄骨のみ）を実装する。
#[allow(clippy::too_many_arguments)]
fn src_column_axis_ma(
    n_design: f64,
    rnc: f64,
    rnt: f64,
    sa: f64,
    s_ft: f64,
    s_fc: f64,
    sz: f64,
    s_mo: f64,
    curve: &[(f64, f64)],
) -> f64 {
    if n_design >= rnt && n_design <= rnc {
        s_mo + interp_ma_curve(curve, n_design)
    } else if n_design > rnc {
        let sn = n_design - rnc;
        (sz * (s_fc - sn / sa)).max(0.0)
    } else {
        let sn = n_design - rnt;
        (sz * (s_ft - sn.abs() / sa)).max(0.0)
    }
}

/// SRC 柱の断面検定（軸力+二軸曲げ+二方向せん断、SRC 規準 1987 累加強度式）。
#[allow(clippy::too_many_arguments)]
fn src_column_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    b: f64,
    d_full: f64,
    rebar: &RcRebar,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
    steel_grade: &str,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();

    // 軽量コンクリート1種・2種は許容応力度（圧縮・せん断）を 0.9 倍に低減
    // （マニュアル「04 断面検定」。class 対応版を使用）。
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);
    let fs = concrete_allowable_shear_class(fc_raw, mat.concrete_class, long_term);
    let n_ratio = young_ratio_n(fc_raw);
    let shear_grade = rebar
        .shear
        .grade
        .clone()
        .unwrap_or_else(|| grade.to_string());
    let w_ft = rebar_allowable_shear(&shear_grade, long_term);

    let thickness = steel_web_thick.max(steel_flange_thick);
    let f_value = steel_f_value_prefix(steel_grade, thickness).unwrap_or(235.0);
    let s_ft = steel_ft(f_value, ctx.term);
    let s_fs = steel_fs(f_value, ctx.term);
    let s_fc = s_ft; // 座屈考慮なし（モジュール doc 参照）

    let (sa, sz_z, sz_y) = steel_h_props(
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    );

    let s_pc = (steel_width * steel_flange_thick) / (b * d_full).max(1e-9);
    let fc_prime = (fc_allow * (1.0 - 15.0 * s_pc)).max(0.0);

    let ft_z = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);
    let ft_y = rebar_allowable_tension(grade, rebar.main_y.dia, long_term);
    let ft_axial =
        rebar_allowable_tension(grade, rebar.main_x.dia.max(rebar.main_y.dia), long_term);

    let as_x = bar_set_area(&rebar.main_x);
    let as_y = bar_set_area(&rebar.main_y);
    let as_total = as_x + as_y;

    let ae = b * d_full + (n_ratio - 1.0) * as_total;
    let rnc1 = ae * fc_prime;
    let rnc2 = ae * ft_axial / n_ratio;
    let rnc = rnc1.min(rnc2).max(0.0);
    let rnt = -(as_total * ft_axial);

    let s_nc = sa * s_fc;
    let s_nt = sa * s_ft;

    let props_z = src_rect_axis_props(b, d_full, &rebar.main_x, rebar);
    let props_y = src_rect_axis_props(d_full, b, &rebar.main_y, rebar);

    let axis_z = SrcColumnAxis {
        props: props_z,
        at_perp: as_y,
        ft: ft_z,
    };
    let axis_y = SrcColumnAxis {
        props: props_y,
        at_perp: as_x,
        ft: ft_y,
    };

    let curve_z = src_column_nm_curve(&axis_z, fc_prime, n_ratio, rnc);
    let curve_y = src_column_nm_curve(&axis_y, fc_prime, n_ratio, rnc);

    let n_design = -forces.n; // 圧縮を正とする設計軸力に変換

    let s_mo_z = sz_z * s_ft;
    let s_mo_y = sz_y * s_ft;

    let ma_z = src_column_axis_ma(n_design, rnc, rnt, sa, s_ft, s_fc, sz_z, s_mo_z, &curve_z);
    let ma_y = src_column_axis_ma(n_design, rnc, rnt, sa, s_ft, s_fc, sz_y, s_mo_y, &curve_y);

    let ratio_z = ratio_or_large(forces.mz, ma_z);
    let ratio_y = ratio_or_large(forces.my, ma_y);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > rnc + s_nc {
        n_design / (rnc + s_nc)
    } else if n_design < rnt - s_nt {
        (-n_design) / (-rnt + s_nt)
    } else {
        0.0
    };

    // 地震時短期の設計用せん断力（構造規定方式）: rMu は軸力を考慮した
    // `rc_column_mu_simple`（柱頭・柱脚同一断面・同一設計軸力の仮定）で
    // 算定する。sft は常に短期値を用いる。
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let sigma_y = rebar_sigma_y(mat);
    let r_mu_z = rc_column_mu_simple(
        &RcCapacityInput {
            b: props_z.b,
            d: props_z.d_full,
            at: props_z.at,
            d_eff: props_z.d,
            sigma_y,
            fc: fc_raw,
            pw: props_z.pw,
            sigma_wy: 0.0,
            clear_span: 0.0,
            sigma_0: 0.0,
        },
        as_total,
        n_design,
    );
    let r_mu_y = rc_column_mu_simple(
        &RcCapacityInput {
            b: props_y.b,
            d: props_y.d_full,
            at: props_y.at,
            d_eff: props_y.d,
            sigma_y,
            fc: fc_raw,
            pw: props_y.pw,
            sigma_wy: 0.0,
            clear_span: 0.0,
            sigma_0: 0.0,
        },
        as_total,
        n_design,
    );

    let (m_alpha_z, q_alpha_z) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let b_prime_z = (b - steel_width).max(0.0);
    let seismic_z = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 1,
        s_ft_short,
        r_mu: r_mu_z,
    };
    let shear_z = src_shear_check(
        forces.qy,
        m_alpha_z,
        q_alpha_z,
        sz_z,
        props_z.at,
        props_z.j,
        props_z.d,
        props_z.b,
        b_prime_z,
        props_z.pw,
        fs,
        w_ft,
        s_fs,
        steel_web_thick * (steel_height - 2.0 * steel_flange_thick),
        1.5,
        &seismic_z,
    );

    let (m_alpha_y, q_alpha_y) = ctx.shear_span.unwrap_or((forces.my.abs(), forces.qz.abs()));
    let b_prime_y = (d_full - steel_height).max(0.0);
    let seismic_y = SrcSeismicCtx {
        ctx,
        pos: forces.pos,
        q_index: 2,
        s_ft_short,
        r_mu: r_mu_y,
    };
    let shear_y = src_shear_check(
        forces.qz,
        m_alpha_y,
        q_alpha_y,
        sz_y,
        props_y.at,
        props_y.j,
        props_y.d,
        props_y.b,
        b_prime_y,
        props_y.pw,
        fs,
        w_ft,
        s_fs,
        2.0 * steel_flange_thick * steel_width,
        1.5,
        &seismic_y,
    );

    let ratio = ratio_axial
        .max(ratio_biaxial)
        .max(shear_z.ratio)
        .max(shear_y.ratio);

    let basis = "SRC規準(1987) 柱: 累加強度式(軸力+二軸曲げ)+ せん断弾性分担".to_string();
    let qd_note_z = if shear_z.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let qd_note_y = if shear_y.used_qd {
        "構造規定方式"
    } else {
        "弾性分担"
    };
    let detail = format!(
        "rNc={:.1} N, rNt={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, \
         MAz={:.1} N·mm, MAy={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm, \
         sQAz={:.1} N, rQAz={:.1} N, sQAy={:.1} N, rQAy={:.1} N, s_pc={:.5}, fc'={:.3}, \
         設計用せん断力(z)={qd_note_z}, 設計用せん断力(y)={qd_note_y}",
        rnc,
        rnt,
        s_nc,
        s_nt,
        n_design,
        ma_z,
        ma_y,
        forces.mz,
        forces.my,
        shear_z.s_qa,
        shear_z.r_qa,
        shear_y.s_qa,
        shear_y.r_qa,
        s_pc,
        fc_prime
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
    }
}

// ============================================================================
// 3. CFT 柱（SRC 規準に基づく累加強度式）
// ============================================================================

/// 矩形充填コンクリート部分の (cN, cM) を弾性三角形応力分布の閉形式で求める。
/// `xn`: 中立軸位置（圧縮縁からの距離）[mm]、`cb`/`cd`: 検討方向の充填断面
/// 幅・せい [mm]。
fn cft_rect_cn_cm(cb: f64, cd: f64, fc: f64, xn: f64) -> (f64, f64) {
    if cb <= 0.0 || cd <= 0.0 || fc <= 0.0 || xn <= 0.0 {
        return (0.0, 0.0);
    }
    let xr = xn / cd;
    if xr <= 1.0 {
        let cn = cb * cd * fc * (xr / 2.0);
        let cm = cb * cd * cd * fc * (xr * (3.0 - 2.0 * xr) / 12.0);
        (cn, cm)
    } else {
        let cn = cb * cd * fc * (1.0 - 1.0 / (2.0 * xr));
        let cm = cb * cd * cd * fc * (1.0 / (12.0 * xr));
        (cn, cm)
    }
}

/// 設計軸力 `n_design`（0≤N<cNc）に対する矩形充填コンクリート部分の cM を、
/// 閉形式の逆算（cN(xn)=N となる xn を解く）で求める。
fn cft_rect_ma(cb: f64, cd: f64, fc: f64, n_design: f64) -> f64 {
    let cnc = cb * cd * fc;
    if cnc <= 0.0 || n_design <= 0.0 {
        return 0.0;
    }
    let ratio = (n_design / cnc).clamp(0.0, 1.0 - 1e-9);
    let xr = if ratio <= 0.5 {
        2.0 * ratio
    } else {
        1.0 / (2.0 * (1.0 - ratio))
    };
    let xn = xr * cd;
    let (_, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
    cm
}

/// 縁応力 `fc` 一定・線形分布（コンクリート引張無視）を断面内で数値積分し、
/// 任意断面形状の (cN, cM) を求める汎用ヘルパ。`width_fn(y)` は圧縮縁からの
/// 距離 `y` [mm] における断面幅 [mm] を返す。
fn numeric_cn_cm(cd: f64, fc: f64, xn: f64, width_fn: impl Fn(f64) -> f64) -> (f64, f64) {
    if cd <= 0.0 || fc <= 0.0 || xn <= 0.0 {
        return (0.0, 0.0);
    }
    let y_max = xn.min(cd);
    if y_max <= 0.0 {
        return (0.0, 0.0);
    }
    let n_steps = 400usize;
    let dy = y_max / n_steps as f64;
    let center = cd / 2.0;
    let mut cn = 0.0;
    let mut cm = 0.0;
    for i in 0..n_steps {
        let y = (i as f64 + 0.5) * dy;
        let sigma = (fc * (1.0 - y / xn)).max(0.0);
        let width = width_fn(y);
        let df = sigma * width * dy;
        cn += df;
        cm += df * (center - y);
    }
    (cn, cm.abs())
}

/// 円形充填コンクリート部分の (cN, cM)（数値積分、矩形と同じ弾性仮定）。
/// `dc`: 充填部直径 [mm]。
fn cft_circle_cn_cm(dc: f64, fc: f64, xn: f64) -> (f64, f64) {
    numeric_cn_cm(dc, fc, xn, |y| {
        let r = dc / 2.0;
        2.0 * (r * r - (y - r).powi(2)).max(0.0).sqrt()
    })
}

/// 設計軸力 `n_design`（0≤N<cNc）に対する円形充填コンクリート部分の cM を、
/// 二分法で cN(xn)=N となる xn を求めて算定する。
fn cft_circle_ma(dc: f64, fc: f64, n_design: f64) -> f64 {
    if dc <= 0.0 || fc <= 0.0 || n_design <= 0.0 {
        return 0.0;
    }
    let cnc = std::f64::consts::PI * dc * dc / 4.0 * fc;
    if cnc <= 0.0 {
        return 0.0;
    }
    let target = n_design.min(cnc * (1.0 - 1e-6));
    let mut lo = 1e-6 * dc;
    let mut hi = 50.0 * dc;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let (cn, _) = cft_circle_cn_cm(dc, fc, mid);
        if cn < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let xn = 0.5 * (lo + hi);
    let (_, cm) = cft_circle_cn_cm(dc, fc, xn);
    cm
}

/// CFT 柱 1 軸分の許容曲げモーメント MA(N)。マニュアルの 3 分岐
/// （コンクリート+鋼管累加 / 圧縮超過で鋼管のみ / 引張で鋼管のみ）を実装する。
/// `cm_fn`: 0≤N≤cNc の範囲でコンクリート部分の cM(N) を返す関数
/// （矩形は [`cft_rect_ma`]、円形は [`cft_circle_ma`]）。
fn cft_axis_capacity(
    n_design: f64,
    cnc: f64,
    sa: f64,
    s_ft: f64,
    s_fc: f64,
    sz: f64,
    cm_fn: impl Fn(f64) -> f64,
) -> f64 {
    if n_design < 0.0 {
        (sz * (s_ft - (-n_design) / sa)).max(0.0)
    } else if n_design <= cnc {
        sz * s_ft + cm_fn(n_design)
    } else {
        let sn = n_design - cnc;
        (sz * (s_fc - sn / sa)).max(0.0)
    }
}

/// CFT 柱の鋼管部分の許容応力度 (s_ft, s_fs, s_fc)。
///
/// s_fc は座屈を考慮する（RESP-D マニュアル 04「CFT柱の断面検定」の記号
/// λ=Lk/i・Lk/D に対応）。細長比 λ は**鋼管単体**の断面二次半径で評価する
/// （充填コンクリートの曲げ剛性寄与を無視するため実際より λ が大きく
/// 算定され、安全側）。λ=0（座屈長さ 0）のとき s_fc は長期 F/1.5（=s_ft）
/// に一致し、従来実装（s_fc = s_ft）と連続する。
fn cft_common_steel(f_value: f64, term: LoadTerm, lambda: f64) -> (f64, f64, f64) {
    let s_ft = steel_ft(f_value, term);
    let s_fs = steel_fs(f_value, term);
    let s_fc = crate::steel::steel_fc(f_value, lambda, term);
    (s_ft, s_fs, s_fc)
}

/// 鋼管単体の最小断面二次半径 i_min と座屈長さ lk から細長比 λ を求める。
/// i_min・lk のいずれかが 0 以下なら λ=0（座屈無視）。
fn cft_lambda(i_min: f64, lk: f64) -> f64 {
    if i_min > 1e-9 && lk > 0.0 {
        lk / i_min
    } else {
        0.0
    }
}

fn cft_box_steel_props(height: f64, width: f64, thick: f64) -> (f64, f64, f64) {
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let iz = shape.calc_iz();
    let sz_mz = if height > 0.0 { iy * 2.0 / height } else { 0.0 };
    let sz_my = if width > 0.0 { iz * 2.0 / width } else { 0.0 };
    (a, sz_mz, sz_my)
}

fn cft_pipe_steel_props(outer_dia: f64, thick: f64) -> (f64, f64) {
    let shape = SectionShape::CftPipe { outer_dia, thick };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let sz = if outer_dia > 0.0 {
        iy * 2.0 / outer_dia
    } else {
        0.0
    };
    (a, sz)
}

/// CFT 柱の設計用せん断力成分 `QD2 = |QL| + n・|Q−QL|`（RESP-D マニュアル
/// 04 断面検定。CFT は QD1（複合断面の終局曲げによる算定）を実装しないため
/// 常に QD2 を用いる。モジュール doc「簡略化」参照）。
///
/// `ctx.seismic_qd` が None、または長期内力に同一評価位置が見つからない
/// 場合は解析せん断力 `|q_signed|` をそのまま返す（従来動作）。
fn cft_q_design(ctx: &DesignCtx, pos: f64, q_signed: f64, q_index: usize) -> f64 {
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
    ql_signed.abs() + qd.n_factor * (q_signed - ql_signed).abs()
}

fn cft_box_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    height: f64,
    width: f64,
    thick: f64,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    // 軽量コンクリート1種・2種は許容圧縮応力度を 0.9 倍に低減（class 対応版）。
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);

    let f_value = steel_f_value_prefix(&mat.name, thick).unwrap_or(235.0);
    let (sa, sz_z, sz_y) = cft_box_steel_props(height, width, thick);
    // 鋼管単体の最小断面二次半径（弱軸）で細長比を評価（安全側）。
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let i_min = if sa > 1e-9 {
        (shape.calc_iy().min(shape.calc_iz()) / sa).max(0.0).sqrt()
    } else {
        0.0
    };
    let lambda = cft_lambda(i_min, ctx.lk.unwrap_or(ctx.length));
    let (s_ft, s_fs, s_fc) = cft_common_steel(f_value, ctx.term, lambda);
    let s_nt = sa * s_ft;
    let s_nc = sa * s_fc;

    let c_b_z = (width - 2.0 * thick).max(0.0);
    let c_d_z = (height - 2.0 * thick).max(0.0);
    let c_b_y = c_d_z;
    let c_d_y = c_b_z;

    let c_area = c_b_z * c_d_z;
    let cnc = c_area * fc_allow;

    let n_design = -forces.n;

    let ma_z = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz_z, |n| {
        cft_rect_ma(c_b_z, c_d_z, fc_allow, n)
    });
    let ma_y = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz_y, |n| {
        cft_rect_ma(c_b_y, c_d_y, fc_allow, n)
    });

    let ratio_z = ratio_or_large(forces.mz, ma_z);
    let ratio_y = ratio_or_large(forces.my, ma_y);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > cnc + s_nc {
        n_design / (cnc + s_nc)
    } else if n_design < 0.0 && (-n_design) > s_nt {
        (-n_design) / s_nt
    } else {
        0.0
    };

    let dw = (height - 2.0 * thick).max(0.0);
    let s_aw = 2.0 * thick * dw;
    let s_qa = s_aw * s_fs;
    // 地震時短期は QD2 = |QL| + n・|Q−QL| を qy/qz 各成分に適用してから
    // 大きい方を用いる（ctx.seismic_qd が None なら解析せん断力のまま）。
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2);
    let q_max = q_design_y.max(q_design_z);
    let ratio_shear = if s_qa > 1e-9 { q_max / s_qa } else { 0.0 };

    let ratio = ratio_axial.max(ratio_biaxial).max(ratio_shear);

    let basis = "CFT柱(角形): SRC規準に基づく累加強度式".to_string();
    let detail = format!(
        "cNc={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, MAz={:.1} N·mm, MAy={:.1} N·mm, \
         mz={:.1} N·mm, my={:.1} N·mm, sQA={:.1} N, qy={:.1} N, qz={:.1} N",
        cnc, s_nc, s_nt, n_design, ma_z, ma_y, forces.mz, forces.my, s_qa, forces.qy, forces.qz
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
    }
}

fn cft_pipe_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    outer_dia: f64,
    thick: f64,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    // 軽量コンクリート1種・2種は許容圧縮応力度を 0.9 倍に低減（class 対応版）。
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);

    let f_value = steel_f_value_prefix(&mat.name, thick).unwrap_or(235.0);
    let (sa, sz) = cft_pipe_steel_props(outer_dia, thick);
    // 鋼管単体の断面二次半径で細長比を評価（安全側）。
    let shape = SectionShape::CftPipe { outer_dia, thick };
    let i_min = if sa > 1e-9 {
        (shape.calc_iy() / sa).max(0.0).sqrt()
    } else {
        0.0
    };
    let lambda = cft_lambda(i_min, ctx.lk.unwrap_or(ctx.length));
    let (s_ft, s_fs, s_fc) = cft_common_steel(f_value, ctx.term, lambda);
    let s_nt = sa * s_ft;
    let s_nc = sa * s_fc;

    let dc = (outer_dia - 2.0 * thick).max(0.0);
    let c_area = std::f64::consts::PI * dc * dc / 4.0;
    let cnc = c_area * fc_allow;

    let n_design = -forces.n;

    let ma = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz, |n| {
        cft_circle_ma(dc, fc_allow, n)
    });

    // 円形は等方性のため二軸とも同じ MA を用いる。
    let ratio_z = ratio_or_large(forces.mz, ma);
    let ratio_y = ratio_or_large(forces.my, ma);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > cnc + s_nc {
        n_design / (cnc + s_nc)
    } else if n_design < 0.0 && (-n_design) > s_nt {
        (-n_design) / s_nt
    } else {
        0.0
    };

    let s_aw = sa / 2.0;
    let s_qa = s_aw * s_fs;
    // 地震時短期は QD2 = |QL| + n・|Q−QL| を qy/qz 各成分に適用してから
    // 合成する（ctx.seismic_qd が None なら解析せん断力のまま）。
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2);
    let q_res = (q_design_y.powi(2) + q_design_z.powi(2)).sqrt();
    let ratio_shear = if s_qa > 1e-9 { q_res / s_qa } else { 0.0 };

    let ratio = ratio_axial.max(ratio_biaxial).max(ratio_shear);

    let basis = "CFT柱(円形): SRC規準に基づく累加強度式".to_string();
    let detail = format!(
        "cNc={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, MA={:.1} N·mm, mz={:.1} N·mm, \
         my={:.1} N·mm, sQA={:.1} N, qy={:.1} N, qz={:.1} N",
        cnc, s_nc, s_nt, n_design, ma, forces.mz, forces.my, s_qa, forces.qy, forces.qz
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0 && ratio.is_finite(),
        basis,
        detail,
    }
}

// ============================================================================
// 4. DesignCheck 実装
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
            MemberKind::Beam | MemberKind::Brace => src_beam_check(
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
            MemberKind::Column => src_column_check(
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

/// CFT 柱の断面検定（`SectionShape::CftBox`/`CftPipe` を対象とする）。
/// マニュアルに CFT 梁の規定は無いため、`ctx.kind` に依らず柱の検定式を
/// 適用する。
pub struct CftDesign;

impl DesignCheck for CftDesign {
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
                basis: "CFT検定: Fc未設定".to_string(),
                detail: "Material.fc が None/0 のため検定をスキップしました。".to_string(),
            };
        }

        match &sec.shape {
            Some(SectionShape::CftBox {
                height,
                width,
                thick,
            }) => cft_box_check(forces, mat, ctx, *height, *width, *thick, fc_raw),
            Some(SectionShape::CftPipe { outer_dia, thick }) => {
                cft_pipe_check(forces, mat, ctx, *outer_dia, *thick, fc_raw)
            }
            _ => CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "CFT検定: 断面形状不一致".to_string(),
                detail: "Section.shape が CftBox/CftPipe ではないため検定をスキップしました。"
                    .to_string(),
            },
        }
    }
}

// ============================================================================
// テスト
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::concrete_allowable_shear;
    use crate::SeismicQd;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    use squid_n_core::units::ConcreteClass;

    fn make_material(fc: f64, grade: &str) -> Material {
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

    fn make_material_no_fc(grade: &str) -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: grade.to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn src_rect_shape(
        b: f64,
        d: f64,
        main_count: u32,
        main_dia: f64,
        main_layers: u32,
        cover: f64,
        shear_dia: f64,
        shear_pitch: f64,
        shear_legs: u32,
        steel_height: f64,
        steel_width: f64,
        steel_web_thick: f64,
        steel_flange_thick: f64,
        steel_grade: &str,
    ) -> SectionShape {
        SectionShape::SrcRect {
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
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            steel_grade: steel_grade.to_string(),
        }
    }

    fn make_section(shape: SectionShape) -> Section {
        shape.to_section(SectionId(0), "test".to_string())
    }

    fn zero_forces() -> MemberForcesAt {
        MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        }
    }

    fn ctx_beam(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Beam,
            ..Default::default()
        }
    }

    fn ctx_column(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Column,
            ..Default::default()
        }
    }

    // ------------------------------------------------------------------
    // SRC 梁
    // ------------------------------------------------------------------

    #[test]
    fn test_src_beam_moment_handcalc() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let sec = make_section(shape.clone());
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);

        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 22.0, true);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft = steel_ft(f_value, LoadTerm::Long);
        let expected_ma = sz * s_ft + props.at * ft * props.j;

        let forces = MemberForcesAt {
            mz: expected_ma * 0.5,
            ..zero_forces()
        };
        let design = SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!((result.ratio - 0.5).abs() < 1e-6, "ratio={}", result.ratio);
        assert!(result.basis.contains("SRC規準"));
    }

    #[test]
    fn test_src_beam_shear_split_handcalc() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);

        let q = 200_000.0;
        let expected_s_q = sz / (sz + props.at * props.j) * q;

        let fs = concrete_allowable_shear(24.0, true);
        let w_ft = rebar_allowable_shear("SD345", true);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_fs = steel_fs(f_value, LoadTerm::Long);

        let ctx = ctx_beam(LoadTerm::Long);
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short: 0.0,
            r_mu: 0.0,
        };
        let shear = src_shear_check(
            q,
            0.0,
            q,
            sz,
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            9.0 * (500.0 - 2.0 * 14.0),
            2.0,
            &seismic,
        );
        assert!(!shear.used_qd);
        assert!((shear.s_q - expected_s_q).abs() / expected_s_q < 1e-9);
        assert!((shear.s_q + shear.r_q - q).abs() < 1e-6);
    }

    /// SRC の pw 上限は SRC 規準1987 準拠で長短期とも 0.6%
    /// （「pw が 0.6% を超える場合は 0.6% として算定する」）。
    #[test]
    fn test_src_shear_pw_capped_at_0_6_percent_both_terms() {
        // 過大なせん断補強筋比（pw > 0.6%）を与え、算定に使われる pw が
        // 0.6% に頭打ちされることを確認する。
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 30.0, 4, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        for long_term in [true, false] {
            let fs = concrete_allowable_shear(24.0, long_term);
            let w_ft = rebar_allowable_shear("SD345", long_term);
            let term = if long_term {
                LoadTerm::Long
            } else {
                LoadTerm::Short
            };
            let s_fs = steel_fs(f_value, term);
            let ctx = ctx_beam(term);
            let seismic = SrcSeismicCtx {
                ctx: &ctx,
                pos: 0.0,
                q_index: 1,
                s_ft_short: 0.0,
                r_mu: 0.0,
            };
            let shear = src_shear_check(
                100_000.0, 0.0, 100_000.0,
                0.0, // 鉄骨寄与を 0 として RC 側の pw の効果だけを見る
                props.at, props.j, props.d, props.b, 200.0, props.pw, fs, w_ft, s_fs, 0.0, 2.0,
                &seismic,
            );
            assert!(
                (shear.pw - 0.006).abs() < 1e-12,
                "long_term={long_term}: pw={} は 0.6% に頭打ちされるはず",
                shear.pw
            );
        }
    }

    // ------------------------------------------------------------------
    // SRC 柱
    // ------------------------------------------------------------------

    fn src_column_shape() -> SectionShape {
        src_rect_shape(
            500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 200.0, 9.0, 14.0, "SN400B",
        )
    }

    #[test]
    fn test_src_column_n0_matches_smo_plus_rm0() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        let forces = MemberForcesAt {
            mz: 1.0,
            ..zero_forces()
        };
        let design = SrcDesign;
        let r0 = design.check(&forces, &sec, &mat, &ctx);
        let ma_z = 1.0 / r0.ratio;
        assert!(ma_z > 0.0 && ma_z.is_finite());

        let (_sa, sz_z, _) = steel_h_props(300.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft = steel_ft(f_value, LoadTerm::Long);
        let s_mo = sz_z * s_ft;
        // N=0 の MA は少なくとも鋼骨単体の sMo 以上であるはず（RC 分は正で加算）。
        assert!(ma_z >= s_mo * 0.99, "ma_z={ma_z}, s_mo={s_mo}");
    }

    #[test]
    fn test_src_column_beyond_rnc_uses_steel_only() {
        let shape = src_column_shape();
        let sec = make_section(shape.clone());
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // 非常に大きな圧縮軸力 (n<0 は圧縮) を与え、N > rNc となる状況を作る。
        let forces = MemberForcesAt {
            n: -30_000_000.0,
            mz: 10_000_000.0,
            ..zero_forces()
        };
        let design = SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio.is_finite());
        assert!(result.detail.contains("rNc"));
        let _ = shape;
    }

    #[test]
    fn test_src_column_tension_side() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // n>0 は引張。全主筋の引張耐力を大きく超える引張軸力を与える。
        let forces = MemberForcesAt {
            n: 5_000_000.0,
            mz: 1_000_000.0,
            ..zero_forces()
        };
        let design = SrcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio.is_finite());
        assert!(result.ratio > 0.0);
    }

    #[test]
    fn test_src_column_biaxial_linear_sum() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = SrcDesign;

        let forces_z = MemberForcesAt {
            mz: 1.0,
            ..zero_forces()
        };
        let r0 = design.check(&forces_z, &sec, &mat, &ctx);
        let ma_z = 1.0 / r0.ratio;

        let mz_test = ma_z * 0.3;
        let forces = MemberForcesAt {
            mz: mz_test,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(
            (r.ratio - 0.3).abs() < 0.05,
            "mz 単独 0.3 割合のとき ratio ≒ 0.3 のはず: ratio={}",
            r.ratio
        );
    }

    #[test]
    fn test_src_column_fc_prime_reduction_effect() {
        // 鉄骨フランジが大きいほど s_pc が大きくなり fc' が低下し、rNc が
        // 減少するはず。
        let shape_small_steel = src_rect_shape(
            500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 150.0, 9.0, 9.0, "SN400B",
        );
        let shape_large_steel = src_rect_shape(
            500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 300.0, 9.0, 30.0, "SN400B",
        );

        let sec_small = make_section(shape_small_steel);
        let sec_large = make_section(shape_large_steel);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = SrcDesign;

        let forces = MemberForcesAt {
            n: -1.0,
            ..zero_forces()
        };
        let r_small = design.check(&forces, &sec_small, &mat, &ctx);
        let r_large = design.check(&forces, &sec_large, &mat, &ctx);
        assert!(r_small.detail.contains("rNc"));
        assert!(r_large.detail.contains("rNc"));
    }

    #[test]
    fn test_src_fc_missing_skip() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mat = make_material_no_fc("SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = SrcDesign;
        let result = design.check(&zero_forces(), &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("Fc"));
    }

    #[test]
    fn test_src_shape_mismatch_skip() {
        let sec = Section {
            id: SectionId(0),
            name: "no-shape".to_string(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 500.0,
            width: 500.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let design = SrcDesign;
        let result = design.check(&zero_forces(), &sec, &mat, &ctx);
        assert!(result.ok);
        assert!(result.basis.contains("断面形状不一致"));
    }

    // ------------------------------------------------------------------
    // CFT 矩形: 閉形式
    // ------------------------------------------------------------------

    #[test]
    fn test_cft_rect_xn_half_d() {
        let (cb, cd, fc) = (400.0, 400.0, 8.0);
        let xn = 0.5 * cd;
        let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
        let expected_cn = cb * cd * fc * (0.5 / 2.0);
        let expected_cm = cb * cd * cd * fc * (0.5 * (3.0 - 1.0) / 12.0);
        assert!((cn - expected_cn).abs() / expected_cn < 1e-9);
        assert!((cm - expected_cm).abs() / expected_cm < 1e-9);
    }

    #[test]
    fn test_cft_rect_xn_eq_d_continuity() {
        let (cb, cd, fc) = (400.0, 400.0, 8.0);
        let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, cd);
        // Xn=1 の境界で両分岐が一致することを確認する。
        let expected_cn = cb * cd * fc * 0.5;
        let expected_cm = cb * cd * cd * fc * (1.0 / 12.0);
        assert!((cn - expected_cn).abs() / expected_cn < 1e-6);
        assert!((cm - expected_cm).abs() / expected_cm < 1e-6);
    }

    #[test]
    fn test_cft_rect_xn_2d() {
        let (cb, cd, fc) = (400.0, 400.0, 8.0);
        let xn = 2.0 * cd;
        let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
        let expected_cn = cb * cd * fc * (1.0 - 1.0 / 4.0);
        let expected_cm = cb * cd * cd * fc * (1.0 / 24.0);
        assert!((cn - expected_cn).abs() / expected_cn < 1e-9);
        assert!((cm - expected_cm).abs() / expected_cm < 1e-9);
    }

    #[test]
    fn test_cft_rect_matches_numeric_integration() {
        let (cb, cd, fc) = (350.0, 500.0, 10.0);
        for &xr in &[0.3, 0.8, 1.0, 1.5, 3.0] {
            let xn = xr * cd;
            let (cn_closed, cm_closed) = cft_rect_cn_cm(cb, cd, fc, xn);
            let (cn_num, cm_num) = numeric_cn_cm(cd, fc, xn, |_| cb);
            assert!(
                (cn_closed - cn_num).abs() / cn_closed.max(1.0) < 5e-3,
                "xr={xr}: cn_closed={cn_closed}, cn_num={cn_num}"
            );
            assert!(
                (cm_closed - cm_num).abs() / cm_closed.max(1.0) < 5e-3,
                "xr={xr}: cm_closed={cm_closed}, cm_num={cm_num}"
            );
        }
    }

    // ------------------------------------------------------------------
    // CFT 円形
    // ------------------------------------------------------------------

    #[test]
    fn test_cft_circle_positive_and_small_at_small_xn() {
        let dc = 400.0;
        let fc = 8.0;
        let (cn, cm) = cft_circle_cn_cm(dc, fc, 0.05 * dc);
        assert!(cn > 0.0 && cm > 0.0);
        assert!(cn < std::f64::consts::PI * dc * dc / 4.0 * fc);
    }

    #[test]
    fn test_cft_circle_converges_to_area_times_fc() {
        let dc = 400.0;
        let fc = 8.0;
        let (cn, _) = cft_circle_cn_cm(dc, fc, 1000.0 * dc);
        let ca_fc = std::f64::consts::PI * dc * dc / 4.0 * fc;
        assert!((cn - ca_fc).abs() / ca_fc < 1e-3, "cn={cn}, ca_fc={ca_fc}");
    }

    // ------------------------------------------------------------------
    // CFT 柱: DesignCheck 経由
    // ------------------------------------------------------------------

    fn cft_box_section(height: f64, width: f64, thick: f64) -> Section {
        make_section(SectionShape::CftBox {
            height,
            width,
            thick,
        })
    }

    fn cft_pipe_section(outer_dia: f64, thick: f64) -> Section {
        make_section(SectionShape::CftPipe { outer_dia, thick })
    }

    #[test]
    fn test_cft_box_n0_ma_equals_sm0() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material(24.0, "SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;

        let forces = MemberForcesAt {
            mz: 1.0,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        let ma_z = 1.0 / r.ratio;

        let (_sa, sz_z, _sz_y) = cft_box_steel_props(400.0, 300.0, 9.0);
        let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
        let s_ft = steel_ft(f_value, LoadTerm::Long);
        let s_mo = sz_z * s_ft;
        assert!(
            (ma_z - s_mo).abs() / s_mo < 1e-6,
            "ma_z={ma_z}, s_mo={s_mo}"
        );
    }

    #[test]
    fn test_cft_box_n_exceeds_cnc_steel_only() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material(24.0, "SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;

        let forces = MemberForcesAt {
            n: -20_000_000.0,
            mz: 1_000_000.0,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(r.ratio.is_finite());
        assert!(r.detail.contains("cNc"));
    }

    #[test]
    fn test_cft_pipe_biaxial_smoke() {
        let sec = cft_pipe_section(400.0, 12.0);
        let mat = make_material(24.0, "STKR400");
        let ctx = ctx_column(LoadTerm::Short);
        let design = CftDesign;

        let forces = MemberForcesAt {
            pos: 0.0,
            n: -500_000.0,
            qy: 30_000.0,
            qz: 20_000.0,
            my: 8_000_000.0,
            mz: 15_000_000.0,
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(r.ratio.is_finite() && r.ratio >= 0.0);
        assert!(r.basis.contains("円形"));
    }

    #[test]
    fn test_cft_shear_box() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material(24.0, "SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;

        let (sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
        let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
        let s_fs = steel_fs(f_value, LoadTerm::Long);
        let dw = 400.0 - 2.0 * 9.0;
        let s_aw = 2.0 * 9.0 * dw;
        let s_qa = s_aw * s_fs;
        let _ = sa;

        let forces = MemberForcesAt {
            qy: s_qa * 0.4,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!((r.ratio - 0.4).abs() < 1e-3, "ratio={}", r.ratio);
    }

    /// 軽量コンクリート1種の充填 CFT は cNc が 0.9 倍に低減され、
    /// 圧縮軸力超過時の検定比が普通コンクリートより大きくなる
    /// （`mat.concrete_class` が許容応力度算定に反映されている）。
    #[test]
    fn test_cft_box_lightweight_reduces_cnc() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mut mat_n = make_material(24.0, "SN400B");
        mat_n.concrete_class = ConcreteClass::Normal;
        let mut mat_l = make_material(24.0, "SN400B");
        mat_l.concrete_class = ConcreteClass::Lightweight1;
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;

        // 圧縮容量を大きく超える軸力を与え、ratio_axial = N/(cNc+sNc) を比較する。
        let forces = MemberForcesAt {
            n: -50_000_000.0,
            ..zero_forces()
        };
        let r_n = design.check(&forces, &sec, &mat_n, &ctx);
        let r_l = design.check(&forces, &sec, &mat_l, &ctx);
        assert!(
            r_l.ratio > r_n.ratio,
            "軽量1種は cNc 低減で検定比が大きいはず: normal={}, light={}",
            r_n.ratio,
            r_l.ratio
        );
    }

    /// SRC 柱でも軽量コンクリートの 0.9 倍低減が rNc（RC 部分の許容圧縮）に
    /// 反映される。
    #[test]
    fn test_src_column_lightweight_reduces_capacity() {
        let shape = src_column_shape();
        let sec = make_section(shape);
        let mut mat_n = make_material(24.0, "SD345");
        mat_n.concrete_class = ConcreteClass::Normal;
        let mut mat_l = make_material(24.0, "SD345");
        mat_l.concrete_class = ConcreteClass::Lightweight1;
        let ctx = ctx_column(LoadTerm::Long);
        let design = SrcDesign;

        let forces = MemberForcesAt {
            n: -50_000_000.0,
            ..zero_forces()
        };
        let r_n = design.check(&forces, &sec, &mat_n, &ctx);
        let r_l = design.check(&forces, &sec, &mat_l, &ctx);
        assert!(
            r_l.ratio > r_n.ratio,
            "軽量1種は rNc 低減で検定比が大きいはず: normal={}, light={}",
            r_n.ratio,
            r_l.ratio
        );
    }

    #[test]
    fn test_cft_fc_missing_skip() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material_no_fc("SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;
        let result = design.check(&zero_forces(), &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("Fc"));
    }

    #[test]
    fn test_cft_shape_mismatch_skip() {
        let sec = Section {
            id: SectionId(0),
            name: "no-shape".to_string(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 400.0,
            width: 400.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = make_material(24.0, "SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;
        let result = design.check(&zero_forces(), &sec, &mat, &ctx);
        assert!(result.ok);
        assert!(result.basis.contains("断面形状不一致"));
    }

    // ------------------------------------------------------------------
    // 地震時短期の設計用せん断力（構造規定方式）: SRC 梁
    // ------------------------------------------------------------------

    /// rQD2 = max(0, n・(|Q|−sQD)) が支配するケース（rMu=0 で rQD1 を無効化し
    /// QdMethod::Qd2 で明示的に検証する）。
    #[test]
    fn test_src_beam_seismic_qd2_handcalc() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft_short = steel_ft(f_value, LoadTerm::Short);
        let s_fs = steel_fs(f_value, LoadTerm::Short);
        let fs = concrete_allowable_shear(24.0, false);
        let w_ft = rebar_allowable_shear("SD345", false);

        let ql = 50_000.0; // 長期せん断力
        let q = 200_000.0; // 当該組合せの短期せん断力
        let n_factor = 1.5;
        let clear_length = 4000.0;

        let ctx = DesignCtx {
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
                n_factor,
                clear_length,
                method: QdMethod::Qd2,
            }),
            ..Default::default()
        };
        // r_mu=0 とすることで rQD1 を無効化し（doc 参照）、rQD2 のみを検証する。
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu: 0.0,
        };

        let shear = src_shear_check(
            q,
            0.0,
            q,
            sz,
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            9.0 * (500.0 - 2.0 * 14.0),
            2.0,
            &seismic,
        );

        let denom = sz + props.at * props.j;
        let share = sz / denom;
        let s_ql = share * ql;
        let sum_s_m = 2.0 * sz * s_ft_short;
        let s_qd_expected = s_ql + sum_s_m / clear_length;
        let r_qd2_expected = (n_factor * (q - s_qd_expected)).max(0.0);

        assert!(shear.used_qd);
        assert!(
            (shear.s_q - s_qd_expected).abs() / s_qd_expected < 1e-9,
            "sQD={}, expected={}",
            shear.s_q,
            s_qd_expected
        );
        assert!(
            (shear.r_q - r_qd2_expected).abs() / r_qd2_expected.max(1.0) < 1e-9,
            "rQD={}, expected(rQD2)={}",
            shear.r_q,
            r_qd2_expected
        );
    }

    /// rQD1 = rQL + (rMu1+rMu2)/l′ が支配するケース（QdMethod::Qd1 で
    /// 明示的に検証する）。
    #[test]
    fn test_src_beam_seismic_qd1_handcalc() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_ft_short = steel_ft(f_value, LoadTerm::Short);
        let s_fs = steel_fs(f_value, LoadTerm::Short);
        let fs = concrete_allowable_shear(24.0, false);
        let w_ft = rebar_allowable_shear("SD345", false);

        let ql = 50_000.0;
        let q = 200_000.0;
        let n_factor = 1.5;
        let clear_length = 4000.0;
        // rc_mu_simple で機械的に算定した rMu（部材端 1 箇所分）。
        let r_mu = rc_mu_simple(&RcCapacityInput {
            b: props.b,
            d: props.d_full,
            at: props.at,
            d_eff: props.d,
            sigma_y: 345.0,
            fc: 24.0,
            pw: props.pw,
            sigma_wy: 0.0,
            clear_span: 0.0,
            sigma_0: 0.0,
        });
        assert!(r_mu > 0.0, "テストの前提として rMu>0 が必要");

        let ctx = DesignCtx {
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
                n_factor,
                clear_length,
                method: QdMethod::Qd1,
            }),
            ..Default::default()
        };
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu,
        };

        let shear = src_shear_check(
            q,
            0.0,
            q,
            sz,
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            9.0 * (500.0 - 2.0 * 14.0),
            2.0,
            &seismic,
        );

        let denom = sz + props.at * props.j;
        let share = sz / denom;
        let s_ql = share * ql;
        let r_ql = (ql - s_ql).max(0.0);
        let r_qd1_expected = r_ql + 2.0 * r_mu / clear_length;

        assert!(shear.used_qd);
        assert!(
            (shear.r_q - r_qd1_expected).abs() / r_qd1_expected < 1e-9,
            "rQD={}, expected(rQD1)={}",
            shear.r_q,
            r_qd1_expected
        );
    }

    /// ctx.seismic_qd が None のときは従来どおり弾性分担のみとなり、
    /// used_qd=false（回帰確認）。
    #[test]
    fn test_src_beam_seismic_qd_none_falls_back_to_elastic_share() {
        let shape = src_rect_shape(
            400.0, 700.0, 6, 22.0, 2, 40.0, 10.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0, "SN400B",
        );
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = src_rect_axis_props(400.0, 700.0, &rebar.main_x, &rebar);
        let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
        let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
        let s_fs = steel_fs(f_value, LoadTerm::Long);
        let fs = concrete_allowable_shear(24.0, true);
        let w_ft = rebar_allowable_shear("SD345", true);

        let q = 200_000.0;
        let ctx = ctx_beam(LoadTerm::Long); // seismic_qd = None（Default）
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short: 0.0,
            r_mu: 0.0,
        };
        let shear = src_shear_check(
            q,
            0.0,
            q,
            sz,
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            9.0 * (500.0 - 2.0 * 14.0),
            2.0,
            &seismic,
        );

        let denom = sz + props.at * props.j;
        let expected_s_q = sz / denom * q;
        assert!(!shear.used_qd);
        assert!((shear.s_q - expected_s_q).abs() / expected_s_q < 1e-9);
        assert!((shear.s_q + shear.r_q - q).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // 地震時短期の設計用せん断力（構造規定方式）: SRC 柱
    // ------------------------------------------------------------------

    /// SRC 柱の rMu は軸力（圧縮）に依存して変化し（`rc_column_mu_simple`）、
    /// それに応じて rQD1 = rQL + (rMu1+rMu2)/l′ も変化することを確認する。
    #[test]
    fn test_src_column_rmu_varies_with_axial_and_flows_to_qd1() {
        let shape = src_column_shape();
        let rebar = match &shape {
            SectionShape::SrcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props_z = src_rect_axis_props(500.0, 500.0, &rebar.main_x, &rebar);
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let mat = make_material(24.0, "SD345");
        let sigma_y = rebar_sigma_y(&mat);
        let fc_raw = 24.0;

        let mu_at = |n_design: f64| {
            rc_column_mu_simple(
                &RcCapacityInput {
                    b: props_z.b,
                    d: props_z.d_full,
                    at: props_z.at,
                    d_eff: props_z.d,
                    sigma_y,
                    fc: fc_raw,
                    pw: props_z.pw,
                    sigma_wy: 0.0,
                    clear_span: 0.0,
                    sigma_0: 0.0,
                },
                as_total,
                n_design,
            )
        };

        let mu_n0 = mu_at(0.0);
        let mu_n_comp = mu_at(2_000_000.0); // 中程度の圧縮軸力（釣合軸力未満）
        assert!(
            mu_n_comp > mu_n0,
            "中程度の圧縮軸力で rMu は増加するはず: mu_n0={mu_n0}, mu_n_comp={mu_n_comp}"
        );

        let ql = 50_000.0;
        let clear_length = 4000.0;
        let ctx = DesignCtx {
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
                n_factor: 1.5,
                clear_length,
                method: QdMethod::Qd1,
            }),
            ..Default::default()
        };
        // sM 項をゼロ（share=0.4 は任意の代表値）にして、rQD1 の rMu 依存の
        // みを見る。
        let share = 0.4;
        let sz = 0.0;
        let s_ft_short = 0.0;

        let seismic_n0 = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu: mu_n0,
        };
        let seismic_n_comp = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short,
            r_mu: mu_n_comp,
        };

        let (_, r_qd_n0) = src_seismic_qd(&seismic_n0, ql, share, sz).unwrap();
        let (_, r_qd_n_comp) = src_seismic_qd(&seismic_n_comp, ql, share, sz).unwrap();

        let r_ql = (1.0 - share) * ql;
        let expected_n0 = r_ql + 2.0 * mu_n0 / clear_length;
        let expected_n_comp = r_ql + 2.0 * mu_n_comp / clear_length;

        assert!((r_qd_n0 - expected_n0).abs() / expected_n0 < 1e-9);
        assert!((r_qd_n_comp - expected_n_comp).abs() / expected_n_comp < 1e-9);
        assert!(
            r_qd_n_comp > r_qd_n0,
            "rMu 増加で rQD1 も増加するはず: r_qd_n0={r_qd_n0}, r_qd_n_comp={r_qd_n_comp}"
        );
    }

    // ------------------------------------------------------------------
    // 地震時短期の設計用せん断力割増: CFT 柱
    // ------------------------------------------------------------------

    /// CFT 柱の設計用せん断力を QD2 = |QL| + n・|Q−QL| に置き換えると
    /// （QL=0 のとき）せん断検定比が n 倍になることを確認する。
    #[test]
    fn test_cft_box_seismic_qd2_scales_shear_ratio_by_n() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material(24.0, "SN400B");
        let design = CftDesign;

        let (_sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
        let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
        let s_fs = steel_fs(f_value, LoadTerm::Short);
        let dw = 400.0 - 2.0 * 9.0;
        let s_aw = 2.0 * 9.0 * dw;
        let s_qa = s_aw * s_fs;

        let q_test = s_qa * 0.2;
        let forces = MemberForcesAt {
            qy: q_test,
            ..zero_forces()
        };

        let ctx_none = ctx_column(LoadTerm::Short);
        let r_none = design.check(&forces, &sec, &mat, &ctx_none);

        let n_factor = 1.5;
        let ctx_qd = DesignCtx {
            term: LoadTerm::Short,
            kind: MemberKind::Column,
            seismic_qd: Some(SeismicQd {
                long_at: vec![(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])], // QL=0
                n_factor,
                clear_length: 4000.0,
                method: QdMethod::Min,
            }),
            ..Default::default()
        };
        let r_qd = design.check(&forces, &sec, &mat, &ctx_qd);

        assert!(
            (r_qd.ratio - n_factor * r_none.ratio).abs() / r_none.ratio < 1e-6,
            "ratio_none={}, ratio_qd={}, n={}",
            r_none.ratio,
            r_qd.ratio,
            n_factor
        );
    }

    /// ctx.seismic_qd が None のときは CFT も従来どおり解析せん断力の
    /// ままとなる（回帰確認）。
    #[test]
    fn test_cft_box_seismic_qd_none_uses_raw_shear() {
        let sec = cft_box_section(400.0, 300.0, 9.0);
        let mat = make_material(24.0, "SN400B");
        let ctx = ctx_column(LoadTerm::Long);
        let design = CftDesign;

        let (_sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
        let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
        let s_fs = steel_fs(f_value, LoadTerm::Long);
        let dw = 400.0 - 2.0 * 9.0;
        let s_aw = 2.0 * 9.0 * dw;
        let s_qa = s_aw * s_fs;

        let forces = MemberForcesAt {
            qy: s_qa * 0.4,
            ..zero_forces()
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!((r.ratio - 0.4).abs() < 1e-3, "ratio={}", r.ratio);
    }
}
