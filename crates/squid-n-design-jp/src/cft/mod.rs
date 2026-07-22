//! CFT 造の断面検定（許容応力度検定）。SRC 規準の累加強度式を
//! CFT 柱（コンクリート充填鋼管）に準用する。
//!
//! 準拠する規準:
//! - 日本建築学会「鉄骨鉄筋コンクリート構造計算規準・同解説」（SRC 規準
//!   1987年版）の累加強度式の考え方を CFT 断面（コンクリート充填鋼管）に
//!   適用したもの。相互拘束効果によるコンクリート強度割増しは考慮しない
//!   （非拘束・安全側の仮定）。
//!
//! # 材料の扱い
//! - `CftBox`/`CftPipe`: 鋼種 = `Material.name`、充填コンクリート強度 =
//!   `Material.fc`。
//! - `Material.fc` が `None`/0 の場合は検定をスキップする（`ok=true`,
//!   `basis` に "Fc未設定" と記載）。
//! - 鋼材グレードが [`crate::steel::steel_f_value_prefix`] で解決できない
//!   場合は SS400 相当（F=235）にフォールバックする（安全側とは限らないため
//!   実運用では鋼種名を確認すること）。
//!
//! # 本実装での主な簡略化（doc 内に個別関数でも記載）
//! 1. CFT 柱の設計用せん断力は `QD = min(QD1, QD2)`（`ctx.seismic_qd.method`
//!    に従う）。`QD1 = ΣcMy/h′` の cMy には CFT 指針の N-M 相互作用による
//!    終局曲げ耐力 Mu(N)（[`crate::ultimate::cft_mu_nm`]、柱分類対応）を用い、
//!    柱頭・柱脚同一断面の仮定で `ΣcMy = 2·Mu(N)` とする（[`cft_q_design`]）。
//! 2. CFT 柱の鋼管部分の許容圧縮応力度 `s_fc` は座屈を考慮する
//!    （λ = max(lk_y/i_y, lk_z/i_z) を**鋼管単体**の断面二次半径で評価。
//!    充填コンクリートの剛性寄与を無視するため安全側。[`cft_common_steel`] 参照）。
//! 3. CFT 柱のせん断は強軸・弱軸を対称的に扱うため、RC 柱検定
//!    （`rc/`）と同様に「b/D 入れ替え」の近似を用いる。
//! 4. CFT 円形柱の (N,M) 相関は閉形式を用いず、縁応力一定の弾性三角形
//!    分布を断面内で数値積分して求める（矩形の閉形式と同じ弾性仮定）。
//!
//! # モジュール構成
//! CFT はトップレベルの単一モジュール（`crate::cft`、本ファイル）とし、
//! [`crate::srrc`] の共通ヘルパ（断面諸元抽出等）には依存しない
//! （断面形状・N-M 相関の算定方法が SRC 矩形断面と異なるため）。

use crate::rc::concrete_allowable_compression_class;
use crate::steel::{steel_f_value_prefix, steel_fc, steel_fs, steel_ft};
use crate::{effective_slenderness, CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

// ============================================================================
// 0. 共通ヘルパ（CFT 専用。srrc の共通ヘルパとは独立に保持する）
// ============================================================================

/// MA<=0 の場合に検定比が発散しないよう、大きな有限値で代用する
/// （[`crate::srrc`] 内の同名関数と同ロジック。断面形状の算定方法が異なる
/// ため CFT 側で独立に保持する）。
fn ratio_or_large(m: f64, ma: f64) -> f64 {
    if ma > 1e-9 {
        m.abs() / ma
    } else if m.abs() > 1e-9 {
        1.0e9
    } else {
        0.0
    }
}

// ============================================================================
// 1. CFT 矩形柱の充填コンクリート部分 (cN, cM)
// ============================================================================

/// 矩形充填コンクリート部分の (cN, cM) を弾性三角形応力分布の閉形式で求める。
/// `xn`: 中立軸位置（圧縮縁からの距離）[mm]、`cb`/`cd`: 検討方向の充填断面
/// 幅・せい [mm]。
///
/// 注意（Xn > cD の分枝の cM 式について）: 他の構造計算プログラムの資料では
/// `cM = cb·cd²·Fc·(1 − 1/(12Xn))` と記載される場合があるが、これは誤りであり
/// 正しくは `1/(12Xr)`（本実装）である。根拠:
/// - Xr=1 での連続性: 断面内式 Xr(3−2Xr)/12 は Xr=1 で 1/12 となり、
///   1/(12Xr) も Xr=1 で 1/12 で一致する（1−1/12 = 11/12 では不連続）。
/// - Xn→∞（全断面一様圧縮）で偏心モーメントは 0 に収束すべきで、
///   1/(12Xr)→0 は整合するが 1−1/(12Xr)→1 は発散的で物理的に不合理。
///
/// こうした他資料の記載に合わせる「修正」をしないこと。
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

// ============================================================================
// 2. CFT 円形柱の充填コンクリート部分 (cN, cM)（数値積分）
// ============================================================================

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

// ============================================================================
// 3. 累加強度式・鋼管の許容応力度
// ============================================================================

/// CFT 柱 1 軸分の許容曲げモーメント MA(N)。累加強度式による 3 分岐
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
/// s_fc は座屈を考慮する（SRC 規準準用。細長比の記号
/// λ=Lk/i・Lk/D に対応）。細長比 λ は**鋼管単体**の断面二次半径で評価する
/// （充填コンクリートの曲げ剛性寄与を無視するため実際より λ が大きく
/// 算定され、安全側）。λ=0（座屈長さ 0）のとき s_fc は長期 F/1.5（=s_ft）
/// に一致し、従来実装（s_fc = s_ft）と連続する。
fn cft_common_steel(f_value: f64, term: LoadTerm, lambda: f64) -> (f64, f64, f64) {
    let s_ft = steel_ft(f_value, term);
    let s_fs = steel_fs(f_value, term);
    let s_fc = steel_fc(f_value, lambda, term);
    (s_ft, s_fs, s_fc)
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

/// CFT 柱の設計用せん断力 `QD`（SRC 規準準用）。
///
/// - `QD1 = ΣcMy/h′`。cMy は CFT 指針の N-M 相互作用による終局曲げ耐力
///   Mu(N)（[`crate::ultimate::cft_mu_nm`]、柱分類対応）とし、柱頭・柱脚
///   同一断面の仮定で `ΣcMy = 2·Mu(N)` とする（RC 柱の QD1 と同じ扱い）。
/// - `QD2 = |QL| + n・|Q−QL|`。
/// - `ctx.seismic_qd.method`（QD1/QD2/min）の選択は RC と共通の
///   [`crate::rc::seismic_design_shear`] に委譲する。
///
/// `ctx.seismic_qd` が None、または長期内力に同一評価位置が見つからない
/// 場合は解析せん断力 `|q_signed|` をそのまま返す（従来動作）。
fn cft_q_design(ctx: &DesignCtx, pos: f64, q_signed: f64, q_index: usize, sum_c_my: f64) -> f64 {
    crate::rc::seismic_design_shear(ctx, pos, q_signed, q_index, sum_c_my, true)
}

// ============================================================================
// 4. CFT 角形柱・円形柱の断面検定
// ============================================================================

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
    // 鋼管単体の断面二次モーメントを強軸・弱軸個別に評価し、各軸の座屈長さ
    // lk_y/lk_z と対にして λ=max(λ_y,λ_z) を求める（安全側。充填コンクリート
    // の剛性寄与を無視するため実際より λ が大きく算定される）。
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let lambda = effective_slenderness(
        shape.calc_iy(),
        shape.calc_iz(),
        sa,
        ctx.length,
        ctx.lk_y,
        ctx.lk_z,
    );
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

    // 累加強度式（SRC 規準の考え方を CFT に適用）: MA = sZ・sft + cM(N)。
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

    // せん断有効断面積は方向別（qy: せい方向の側壁 2t(H−2t)、qz: 幅方向の
    // 側壁 2t(B−2t)）。従来は両方向とも H 基準で、H≠B の断面の幅方向せん断を
    // 非保守側に評価していた。
    let s_aw_y = 2.0 * thick * (height - 2.0 * thick).max(0.0);
    let s_aw_z = 2.0 * thick * (width - 2.0 * thick).max(0.0);
    let s_qa_y = s_aw_y * s_fs;
    let s_qa_z = s_aw_z * s_fs;
    // 地震時短期は QD = min(QD1, QD2)（method に従う）を qy/qz 各成分に適用する。
    // QD1 の ΣcMy は N-M 相互作用の終局曲げ Mu(N)（CFT 指針・Fc は raw、Fy は F 値）
    // ×2（柱頭・柱脚同一断面）。ctx.seismic_qd が None なら解析せん断力のまま。
    let (sum_c_my_z, sum_c_my_y) = if ctx.seismic_qd.is_some() {
        let shape = SectionShape::CftBox {
            height,
            width,
            thick,
        };
        // weak_axis=false（強軸側）は lk_y、weak_axis=true（弱軸側）は lk_z を用いる。
        let lk_y = ctx.lk_y.unwrap_or(ctx.length);
        let lk_z = ctx.lk_z.unwrap_or(ctx.length);
        let mu_z = crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk_y, false)
            .unwrap_or(0.0);
        let mu_y = crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk_z, true)
            .unwrap_or(0.0);
        (2.0 * mu_z, 2.0 * mu_y)
    } else {
        (0.0, 0.0)
    };
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1, sum_c_my_z);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2, sum_c_my_y);
    let ratio_shear_y = if s_qa_y > 1e-9 {
        q_design_y / s_qa_y
    } else {
        0.0
    };
    let ratio_shear_z = if s_qa_z > 1e-9 {
        q_design_z / s_qa_z
    } else {
        0.0
    };
    let ratio_shear = ratio_shear_y.max(ratio_shear_z);

    let ratio = ratio_axial.max(ratio_biaxial).max(ratio_shear);

    let basis = "CFT柱(角形): SRC規準に基づく累加強度式".to_string();
    let detail = format!(
        "cNc={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, MAz={:.1} N·mm, MAy={:.1} N·mm, \
         mz={:.1} N·mm, my={:.1} N·mm, sQAy={:.1} N, sQAz={:.1} N, qy={:.1} N, qz={:.1} N",
        cnc,
        s_nc,
        s_nt,
        n_design,
        ma_z,
        ma_y,
        forces.mz,
        forces.my,
        s_qa_y,
        s_qa_z,
        forces.qy,
        forces.qz
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
    // 鋼管単体の断面二次モーメントで細長比を評価（安全側）。円形は等方性の
    // ため iy=iz だが、lk_y/lk_z が異なれば λ=max(λ_y,λ_z) は方向により変わる。
    let shape = SectionShape::CftPipe { outer_dia, thick };
    let iy = shape.calc_iy();
    let lambda = effective_slenderness(iy, iy, sa, ctx.length, ctx.lk_y, ctx.lk_z);
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
    // 地震時短期は QD = min(QD1, QD2)（method に従う）を qy/qz 各成分に適用して
    // から合成する。QD1 の ΣcMy は N-M 相互作用の終局曲げ Mu(N)×2（円形は
    // 方向によらず同値）。ctx.seismic_qd が None なら解析せん断力のまま。
    let sum_c_my = if ctx.seismic_qd.is_some() {
        let shape = SectionShape::CftPipe { outer_dia, thick };
        // 円形は方向によらず同値のため、安全側に大きい方の座屈長さを採用する。
        let lk = ctx
            .lk_y
            .unwrap_or(ctx.length)
            .max(ctx.lk_z.unwrap_or(ctx.length));
        2.0 * crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk, false)
            .unwrap_or(0.0)
    } else {
        0.0
    };
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1, sum_c_my);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2, sum_c_my);
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
// 5. DesignCheck 実装
// ============================================================================

/// CFT 柱の断面検定（`SectionShape::CftBox`/`CftPipe` を対象とする）。
/// 準拠規準に CFT 梁の規定は無いため、`ctx.kind` に依らず柱の検定式を
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
mod tests;
