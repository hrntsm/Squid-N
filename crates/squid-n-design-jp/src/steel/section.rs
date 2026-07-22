//! 鉄骨の断面検定における断面性能（鋼構造設計規準の
//! 鉄骨の断面検定における断面性能）。
//!
//! 横座屈を考慮した許容曲げ応力度 fb（鋼構造設計規準 1973）・断面二次半径・
//! 断面欠損（継手部・スカラップ）を考慮した断面係数 Z'・横座屈長さ lb の
//! 解決を扱う。

use crate::material_strength::{big_lambda, steel_ft};
use crate::{DesignCtx, LoadTerm};
use squid_n_core::model::Section;
use squid_n_core::section_shape::SectionShape;

use super::section_modulus;

// ---------------------------------------------------------------------
// 許容曲げ応力度（横座屈考慮）と断面二次半径（鋼構造設計規準 1973）
// ---------------------------------------------------------------------

/// 「圧縮フランジ＋せいの 1/6 のウェブ」からなる T 形断面の、ウェブ軸まわり
/// 断面二次半径 `i = √(I_T/A_T)`（鋼構造設計規準 1973、横座屈許容曲げ fb1 用）。
///
/// `I_T = tf·B³/12 + max(h/6−tf, 0)·tw³/12`、
/// `A_T = B·tf + max(h/6−tf, 0)·tw`。
///
/// ウェブ側の負担高さは「せいの 1/6」からフランジ厚 `tf` を差し引いた
/// ウェブ純高さ相当（フランジに食い込む分を除く）とし、`h/6 ≤ tf` となる
/// 薄せい・厚フランジの断面ではウェブ寄与分を 0 にガードする。
pub fn steel_i_t(b: f64, tf: f64, h: f64, tw: f64) -> f64 {
    let hw = (h / 6.0 - tf).max(0.0);
    let i_t = tf * b.powi(3) / 12.0 + hw * tw.powi(3) / 12.0;
    let a_t = b * tf + hw * tw;
    if a_t > 1e-12 {
        (i_t / a_t).sqrt()
    } else {
        0.0
    }
}

/// 横座屈用の断面二次半径・圧縮フランジ断面積 `(i, af)` を解決する。
///
/// - `Section.shape` が `SteelH` の場合: `(steel_i_t(B, tf, H, tw), B·tf)`。
/// - `SteelBuiltH`（非対称組立 H）の場合: 上下どちらのフランジが圧縮側か
///   （荷重の向き）によらないよう、上下フランジそれぞれについて
///   [`steel_i_t`] を計算し、断面二次半径 `i` が小さい側（横座屈に対して
///   不利な側）の `(i, af=B·tf)` を採用する。
/// - 上記以外（`shape` 無し等）は従来通り `sec.width` を B、呼び出し側が
///   渡す `tf`/`tw` をそのまま用いた `(steel_i_t(B, tf, H, tw), B·tf)`。
pub(crate) fn steel_lateral_buckling_i_af(sec: &Section, tf: f64, tw: f64) -> (f64, f64) {
    match &sec.shape {
        Some(SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        }) => {
            let i = steel_i_t(*width, *flange_thick, *height, *web_thick);
            (i, width * flange_thick)
        }
        Some(SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        }) => {
            let i_upper = steel_i_t(*upper_width, *upper_thick, *height, *web_thick);
            let i_lower = steel_i_t(*lower_width, *lower_thick, *height, *web_thick);
            if i_upper <= i_lower {
                (i_upper, upper_width * upper_thick)
            } else {
                (i_lower, lower_width * lower_thick)
            }
        }
        _ => {
            let b = sec.width;
            let h = sec.depth;
            (steel_i_t(b, tf, h, tw), b * tf)
        }
    }
}

/// H 形鋼強軸の横座屈を考慮した許容曲げ応力度 fb [N/mm²]（鋼構造設計規準 1973）。
///
/// `fbL = max(fb1, fb2)`（ただし長期許容引張 `ft = F/1.5` を上限とする）:
/// - `fb1 = F·(2/3 − (4/15)·(lb/i)²/(C·Λ²))`
/// - `fb2 = 89000/(lb·h/Af)` [N/mm²]（`lb`,`h`,`Af` は mm, mm, mm²）
///
/// `lb`: 圧縮フランジ支点間距離（横座屈長さ）[mm]、`i`: [`steel_i_t`] の
/// 断面二次半径、`h`: 梁せい [mm]、`af`: 圧縮フランジ断面積 `B·tf` [mm²]。
///
/// 修正係数 `c` は呼び出し側で [`steel_lateral_buckling_c`] を用いて求め、
/// ここでは既に確定した値を受け取る（`c=1.0` は安全側・最も不利な等曲げ
/// 分布相当）。`fb1` の分母 `C·Λ²` が大きくなるほど `fb1` は増加する。
///
/// 短期は長期の 1.5 倍（上限 F）。
pub fn steel_fb_h(f: f64, term: LoadTerm, lb: f64, i: f64, h: f64, af: f64, c: f64) -> f64 {
    let c = if c > 1e-9 { c } else { 1.0 };
    let i = i.max(1e-9);
    let af = af.max(1e-9);
    let big_l = big_lambda(f);
    let big_l2 = (big_l.max(1e-9)).powi(2);

    let fb1 = f * (2.0 / 3.0 - (4.0 / 15.0) * (lb / i).powi(2) / (c * big_l2));
    let denom2 = lb * h / af;
    let fb2 = if denom2 > 0.0 {
        89_000.0 / denom2
    } else {
        f64::INFINITY
    };

    let cap_long = f / 1.5;
    let fb_long = fb1.max(fb2).min(cap_long).max(0.0);
    match term {
        LoadTerm::Long => fb_long,
        LoadTerm::Short => (fb_long * 1.5).min(f),
    }
}

/// 横座屈許容曲げ応力度 fb1 の修正係数 C（鋼構造設計規準 1973）。
///
/// `C = 1.75 + 1.05·(M2/M1) + 0.3·(M2/M1)² ≤ 2.3`
///
/// - `M1`: 座屈区間（横座屈長さ `lb` の区間）端部の強軸曲げモーメントの
///   絶対値が大きい方、`M2`: 小さい方（[`DesignCtx::end_moments_z`]）。
/// - `M2/M1` の符号は「複曲率（部材が S 字に曲がる、反曲点あり）なら正、
///   単曲率（一様な向きに曲がる）なら負」という鋼構造設計規準の定義に従う。
///   **squid-n の内力（断面力）符号規約では、`mz` は部材全長で連続な内力場
///   のため、両端の `mz` が異符号＝モーメント図が軸をまたぐ＝反曲点を持つ
///   複曲率、同符号＝軸をまたがない＝反曲点の無い単曲率に対応する**。
/// - 座屈区間中央部（[`DesignCtx::mid_moment_z`]）の絶対値が両端部の絶対値
///   より大きい場合は、区間内の最大曲げが端部にないため安全側の `C=1.0`
///   とする。
/// - [`DesignCtx::end_moments_z`] が `None` の場合は、端部モーメント比の
///   情報が無いため従来通り `C=1.0`（安全側・最も不利な等曲げ分布相当）
///   とする。
pub(crate) fn steel_lateral_buckling_c(ctx: &DesignCtx) -> f64 {
    let Some((m_i, m_j)) = ctx.end_moments_z else {
        return 1.0;
    };
    let abs_i = m_i.abs();
    let abs_j = m_j.abs();
    let m1 = abs_i.max(abs_j);
    let m2 = abs_i.min(abs_j);

    // 区間中央の曲げが端部より大きければ、区間内最大曲げが端部に無いため C=1.0。
    if let Some(mid) = ctx.mid_moment_z {
        if mid.abs() > m1 + 1e-9 {
            return 1.0;
        }
    }

    if m1 <= 1e-9 {
        // 両端とも曲げがほぼ無い（M2/M1=0 相当）→ C=1.75。
        return 1.75;
    }

    let ratio_abs = m2 / m1;
    // 異符号（反曲点あり＝複曲率）なら正、同符号（単曲率）なら負。
    // 片端がほぼゼロなら ratio_abs=0 で符号は結果に影響しない。
    let double_curvature = m_i * m_j < 0.0;
    let m2_over_m1 = if double_curvature {
        ratio_abs
    } else {
        -ratio_abs
    };

    let c = 1.75 + 1.05 * m2_over_m1 + 0.3 * m2_over_m1 * m2_over_m1;
    c.min(2.3)
}

/// 横座屈修正係数 C の解決（beam.rs・column.rs で共通のロジックに統一）。
///
/// 1. [`DesignCtx::steel_attr`] に `c_direct`（正値）の直接入力があれば、
///    その値をそのまま採用する（参考資料の「入力がある場合は入力値を採用」。
///    自動算定の上限 `2.3` はこの場合適用しない）。
/// 2. 直接入力が無い場合、`lb_is_partial=true`（横補剛等により座屈区間が
///    部材の部分区間となり区間端モーメント比が不明）であれば安全側の `1.0`。
/// 3. それ以外は従来の自動算定 [`steel_lateral_buckling_c`]（`ctx.end_moments_z`/
///    `ctx.mid_moment_z` から `M2/M1` により算定）。
///
/// `c_direct` が `0` 以下の場合は無効な入力（未入力相当）として無視し、
/// 2./3. にフォールバックする。
pub(crate) fn steel_c_factor(ctx: &DesignCtx, lb_is_partial: bool) -> f64 {
    if let Some(c) = ctx.steel_attr.as_ref().and_then(|a| a.c_direct) {
        if c > 1e-9 {
            return c;
        }
    }
    if lb_is_partial {
        return 1.0;
    }
    steel_lateral_buckling_c(ctx)
}

// ---------------------------------------------------------------------
// 許容曲げ応力度 fb（新基準・AIJ 鋼構造許容応力度設計規準 2019）
// ---------------------------------------------------------------------

/// H 形鋼強軸の許容曲げ応力度 fb [N/mm²]（新基準・AIJ 鋼構造許容応力度設計規準
/// 2019）。降伏モーメント My と弾性横座屈モーメント Me から求めた限界細長比
/// λb により、全塑性域・非弾性域（直線補間）・弾性域の 3 領域で式を切り替える。
///
/// - `My = F·Z強軸`（降伏モーメント。`z_strong` は強軸断面係数）
/// - `Me = C・√(π⁴・E・Iz・E・Iw/lb⁴ + π²・E・Iz・G・J/lb²)`（弾性横座屈モーメント。
///   `iz`: 弱軸断面二次モーメント、`iw`: 曲げねじり定数、`j`: サンブナンねじり
///   定数、`e`,`g`: ヤング係数・せん断弾性係数、`c`: 修正係数。[`steel_fb_h`]
///   と同じ [`steel_lateral_buckling_c`] を用いてよい）
/// - `λb = √(My/Me)`（横座屈限界細長比）
/// - `eλb = 1/√0.6`（弾性限界細長比）
/// - `ν = 3/2 + (2/3)・(λb/eλb)²`（安全率）
/// - `λb ≤ pλb`（塑性限界細長比。[`steel_p_lambda_b`]）→ `fb = F/ν`
/// - `pλb < λb ≤ eλb` → `fb = (1 − 0.4・(λb−pλb)/(eλb−pλb))・F/ν`
/// - `eλb < λb` → `fb = F/(2.17・λb²)`（弾性座屈域）
///
/// 上限は長期許容引張 `F/1.5`。短期は長期の 1.5 倍（上限 F）。`lb ≤ 0`
/// （横座屈長さ無し）の場合は横座屈を考慮しない `fb = ft` を返す。
#[allow(clippy::too_many_arguments)]
pub fn steel_fb_h_new(
    f: f64,
    term: LoadTerm,
    lb: f64,
    iz: f64,
    iw: f64,
    j: f64,
    e: f64,
    g: f64,
    z_strong: f64,
    c: f64,
    p_lambda_b: f64,
) -> f64 {
    if lb <= 1e-9 {
        return steel_ft(f, term);
    }
    let c = if c > 1e-9 { c } else { 1.0 };

    let my = f * z_strong;
    let pi2 = std::f64::consts::PI.powi(2);
    let pi4 = std::f64::consts::PI.powi(4);
    let me_sq = pi4 * e * iz * e * iw / lb.powi(4) + pi2 * e * iz * g * j / lb.powi(2);
    let me = (c * me_sq.max(0.0).sqrt()).max(1e-9);

    let lambda_b = (my / me).max(0.0).sqrt();
    let e_lambda_b = 1.0 / 0.6_f64.sqrt();
    let nu = 1.5 + (2.0 / 3.0) * (lambda_b / e_lambda_b).powi(2);

    let fb_long = if lambda_b <= p_lambda_b {
        f / nu
    } else if lambda_b <= e_lambda_b {
        let gap = (e_lambda_b - p_lambda_b).max(1e-9);
        (1.0 - 0.4 * (lambda_b - p_lambda_b) / gap) * f / nu
    } else {
        f / (2.17 * lambda_b.powi(2))
    };
    let cap_long = f / 1.5;
    let fb_long = fb_long.min(cap_long).max(0.0);

    match term {
        LoadTerm::Long => fb_long,
        LoadTerm::Short => (fb_long * 1.5).min(f),
    }
}

/// 塑性限界細長比 pλb（新基準・AIJ 鋼構造許容応力度設計規準 2019）。
///
/// `pλb = 0.6 + 0.3・(M2/M1)`。`M2/M1` の符号規約は [`steel_lateral_buckling_c`]
/// と同じ（`M1`: 座屈区間端部モーメントの絶対値が大きい方、`M2`: 小さい方。
/// 複曲率＝両端モーメント異符号で正、単曲率＝同符号で負）。
///
/// - 座屈区間中央部（[`DesignCtx::mid_moment_z`]）の絶対値が両端部より
///   大きい場合は、区間内の最大曲げが端部に無いため安全側の `pλb=0.3` とする。
/// - [`DesignCtx::end_moments_z`] が `None` の場合も同様に安全側の `pλb=0.3`。
/// - `M1≈0`（両端とも曲げがほぼ無い）のときは `M2/M1=0` 扱いで `pλb=0.6`。
pub(crate) fn steel_p_lambda_b(ctx: &DesignCtx) -> f64 {
    let Some((m_i, m_j)) = ctx.end_moments_z else {
        return 0.3;
    };
    let abs_i = m_i.abs();
    let abs_j = m_j.abs();
    let m1 = abs_i.max(abs_j);
    let m2 = abs_i.min(abs_j);

    // 区間中央の曲げが端部より大きければ、区間内最大曲げが端部に無いため 0.3。
    if let Some(mid) = ctx.mid_moment_z {
        if mid.abs() > m1 + 1e-9 {
            return 0.3;
        }
    }

    if m1 <= 1e-9 {
        // 両端とも曲げがほぼ無い（M2/M1=0 相当）→ 0.6。
        return 0.6;
    }

    let ratio_abs = m2 / m1;
    let double_curvature = m_i * m_j < 0.0;
    let m2_over_m1 = if double_curvature {
        ratio_abs
    } else {
        -ratio_abs
    };

    0.6 + 0.3 * m2_over_m1
}

/// 曲げねじり定数 Iw [mm⁶]（新基準 fb 用。beam.rs・column.rs で共用）。
///
/// - `SteelBuiltH`（非対称組立 H）: 上下フランジの寸法から個別に
///   `I_u=t_u・b_u³/12`、`I_l=t_l・b_l³/12` を求め、`hf=H−(t_u+t_l)/2`
///   （上下フランジ図心間距離）として `Iw=hf²・I_u・I_l/(I_u+I_l)`
///   （上下フランジの曲げ剛性比に応じてねじり中心が偏心する非対称 H 用の式）。
/// - それ以外（`SteelH` および shape 無しのフォールバック）: `Iz・(H−tf)²/4`
///   （上下フランジ対称、`Iz` はフランジ図心間の距離の半分だけ離れた 2 枚の
///   フランジが負担するとみなす近似式）。
pub(crate) fn steel_warping_constant(sec: &Section, tf: f64) -> f64 {
    match &sec.shape {
        Some(SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            ..
        }) => {
            let hf = height - (upper_thick + lower_thick) / 2.0;
            let i_u = upper_thick * upper_width.powi(3) / 12.0;
            let i_l = lower_thick * lower_width.powi(3) / 12.0;
            let denom = i_u + i_l;
            if denom > 1e-12 {
                hf * hf * i_u * i_l / denom
            } else {
                0.0
            }
        }
        _ => {
            let h = sec.depth;
            sec.iz * (h - tf).powi(2) / 4.0
        }
    }
}

// ---------------------------------------------------------------------
// 断面欠損（継手部・スカラップ）と横座屈長さ
// （鋼構造設計規準「鉄骨の断面検定における断面性能」）
// ---------------------------------------------------------------------

/// H形鋼の欠損考慮断面係数 Z'（強軸）。
///
/// 鋼構造設計規準の式:
/// - 継手部: `If' = If・(1−βf/100)`、`Iw' = Iw・(1−βw/100)`
///   （βf/βw: フランジ/ウェブの欠損率 [%]）
/// - スカラップ: `Iw'' = tw・((H−2tf)(1−αw/100))³/12`
///   （αw: スカラップによるウェブ欠損率 [%]）
///
/// フランジ寄与分 `If = (B・H³ − B・(H−2tf)³)/12`、ウェブ寄与分
/// `Iw = tw・(H−2tf)³/12` は、全断面の強軸断面二次モーメント
/// `Iy = (B・H³ − (B−tw)・(H−2tf)³)/12`（[`squid_n_core::section_shape`] の
/// H形と同一の式）を `If + Iw = Iy` と分解したもの。
///
/// `is_end=true`（部材端部、スカラップが生じ得る位置）の場合は、ウェブ寄与分を
/// スカラップ考慮の `Iw''` に置き換えたうえで継手欠損 `βw` も併せて乗じる
/// （端部に継手とスカラップが重なる保守的な扱い）。`is_end=false`（継手部・
/// 中間部）は `Iw'`（βw のみ）を用いる。
///
/// `Z' = (If' + Iw'') / (H/2)`。欠損率が 0 の場合は通常の H形強軸断面係数
/// `Z = Iy/(H/2)` に一致する。
#[allow(clippy::too_many_arguments)]
pub fn steel_h_z_with_loss(
    h: f64,
    b: f64,
    tw: f64,
    tf: f64,
    beta_f: f64,
    beta_w: f64,
    alpha_w: f64,
    is_end: bool,
) -> f64 {
    let hw = (h - 2.0 * tf).max(0.0);
    let i_f = (b * h.powi(3) - b * hw.powi(3)) / 12.0;
    let i_f_prime = i_f * (1.0 - beta_f / 100.0).max(0.0);

    let i_w_prime = if is_end {
        let hw_scallop = hw * (1.0 - alpha_w / 100.0).max(0.0);
        let i_w_scallop = tw * hw_scallop.powi(3) / 12.0;
        i_w_scallop * (1.0 - beta_w / 100.0).max(0.0)
    } else {
        let i_w = tw * hw.powi(3) / 12.0;
        i_w * (1.0 - beta_w / 100.0).max(0.0)
    };

    section_modulus(i_f_prime + i_w_prime, h / 2.0)
}

/// 横座屈長さ lb [mm] の解決（優先順位: 直接入力 > 等間隔横補剛 > 部材長）。
///
/// 1. `lb_direct = Some((始端, 中央, 終端))` が与えられていれば、`pos`
///    （部材軸方向の無次元位置 0.0〜1.0）に応じて該当区間の値を返す:
///    `pos<0.25` は始端、`pos<0.75` は中央、それ以外は終端。
/// 2. 直接入力が無く `brace_count = Some(n)`（等間隔横補剛の本数）が
///    あれば `lb = L/(n+1)`（`n` 本の補剛で部材が `n+1` 等分される）。
/// 3. いずれも無ければ部材長 `length` をそのまま横座屈長さとする
///    （横補剛なし＝全長で座屈）。
pub fn resolve_lb(
    pos: f64,
    length: f64,
    lb_direct: Option<(f64, f64, f64)>,
    brace_count: Option<u32>,
) -> f64 {
    if let Some((start, mid, end)) = lb_direct {
        return if pos < 0.25 {
            start
        } else if pos < 0.75 {
            mid
        } else {
            end
        };
    }
    if let Some(n) = brace_count {
        return length / (n as f64 + 1.0);
    }
    length
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material_strength::steel_ft;
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::SteelDesignAttr;

    // -------------------------------------------------------------
    // fb（横座屈考慮）
    // -------------------------------------------------------------

    #[test]
    fn test_fb_lb_zero_equals_ft() {
        let f = 235.0;
        let fb = steel_fb_h(f, LoadTerm::Long, 0.0, 100.0, 500.0, 4000.0, 1.0);
        let ft = steel_ft(f, LoadTerm::Long);
        assert!((fb - ft).abs() < 1e-6, "fb={} ft={}", fb, ft);
    }

    #[test]
    fn test_fb_large_lb_fb2_governs() {
        let f = 235.0;
        let i = 60.0;
        let h = 500.0;
        let af = 4000.0;
        let lb = 20_000.0; // 十分大きい横座屈長さ
        let fb = steel_fb_h(f, LoadTerm::Long, lb, i, h, af, 1.0);
        let fb2 = 89_000.0 / (lb * h / af);
        assert!(
            (fb - fb2).abs() < 1e-6,
            "fb2 should govern: fb={} fb2={}",
            fb,
            fb2
        );
    }

    #[test]
    fn test_i_t_helper() {
        let i = steel_i_t(200.0, 15.0, 400.0, 10.0);
        // hw = max(h/6 - tf, 0) = max(400/6 - 15, 0) = 51.6666...
        let hw: f64 = (400.0 / 6.0 - 15.0_f64).max(0.0);
        let expected_it = 15.0 * 200.0_f64.powi(3) / 12.0 + hw * 10.0_f64.powi(3) / 12.0;
        let expected_at = 200.0 * 15.0 + hw * 10.0;
        let expected = (expected_it / expected_at).sqrt();
        assert!((i - expected).abs() < 1e-9);
    }

    /// h/6 ≤ tf（薄せい・厚フランジ）の場合はウェブ寄与分が 0 にガードされ、
    /// T 形はフランジ矩形単独の断面二次半径 `i = √((tf·b³/12)/(b·tf)) = b/√12`
    /// に一致する。
    #[test]
    fn test_i_t_helper_hw_clamped_to_zero() {
        // h/6 = 100/6 = 16.67 < tf=20 → hw=0。
        let i = steel_i_t(200.0, 20.0, 100.0, 10.0);
        let expected = 200.0 / 12.0_f64.sqrt();
        assert!((i - expected).abs() < 1e-9, "i={} expected={}", i, expected);
    }

    // -------------------------------------------------------------
    // 横座屈修正係数 C
    // -------------------------------------------------------------

    /// 複曲率（端部モーメント異符号＝反曲点あり）・等モーメント逆向き相当
    /// （M2/M1=+1）で C=1.75+1.05+0.3=3.1 → 上限 2.3 にクランプされる。
    #[test]
    fn test_c_factor_double_curvature_equal_clamps_to_2_3() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, -100.0)),
            ..Default::default()
        };
        let c = steel_lateral_buckling_c(&ctx);
        assert!((c - 2.3).abs() < 1e-9, "c={}", c);
    }

    /// 単曲率（端部モーメント同符号＝反曲点なし）・一様分布相当（M2/M1=-1）で
    /// C=1.75-1.05+0.3=1.0。
    #[test]
    fn test_c_factor_single_curvature_uniform_is_1_0() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, 100.0)),
            ..Default::default()
        };
        let c = steel_lateral_buckling_c(&ctx);
        assert!((c - 1.0).abs() < 1e-9, "c={}", c);
    }

    /// 片端モーメントが 0（M2=0）の場合は M2/M1=0 なので C=1.75。
    #[test]
    fn test_c_factor_m2_zero_is_1_75() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, 0.0)),
            ..Default::default()
        };
        let c = steel_lateral_buckling_c(&ctx);
        assert!((c - 1.75).abs() < 1e-9, "c={}", c);
    }

    /// 座屈区間中央の曲げが両端部より大きい場合は安全側 C=1.0。
    #[test]
    fn test_c_factor_mid_moment_dominant_is_1_0() {
        let ctx = DesignCtx {
            end_moments_z: Some((50.0, 50.0)),
            mid_moment_z: Some(200.0),
            ..Default::default()
        };
        let c = steel_lateral_buckling_c(&ctx);
        assert!((c - 1.0).abs() < 1e-9, "c={}", c);
    }

    /// end_moments_z が None の場合は従来通り C=1.0。
    #[test]
    fn test_c_factor_none_end_moments_is_1_0() {
        let ctx = DesignCtx {
            end_moments_z: None,
            ..Default::default()
        };
        let c = steel_lateral_buckling_c(&ctx);
        assert!((c - 1.0).abs() < 1e-9, "c={}", c);
    }

    /// 符号判定の検証: squid-n の内力（断面力）符号規約では mz(0)/mz(1) が
    /// 異符号なら複曲率扱い（C が大きくなる）、同符号なら単曲率扱い
    /// （C が小さくなる）。|M2/M1| が同じでも符号が異なれば C の値が変わる
    /// ことを確認する。
    #[test]
    fn test_c_factor_diff_sign_gt_same_sign_for_same_ratio() {
        let ctx_diff = DesignCtx {
            end_moments_z: Some((100.0, -30.0)), // 異符号（複曲率扱い）
            ..Default::default()
        };
        let ctx_same = DesignCtx {
            end_moments_z: Some((100.0, 30.0)), // 同符号（単曲率扱い）
            ..Default::default()
        };
        let c_diff = steel_lateral_buckling_c(&ctx_diff);
        let c_same = steel_lateral_buckling_c(&ctx_same);
        assert!(
            c_diff > c_same,
            "異符号(複曲率)の方が C が大きいはず: c_diff={} c_same={}",
            c_diff,
            c_same
        );
    }

    // -------------------------------------------------------------
    // 横座屈修正係数 C の解決（steel_c_factor: 直接入力 > 部分区間なら1.0 >
    // 自動算定）
    // -------------------------------------------------------------

    /// c_direct（正値）があれば、自動算定結果や上限 2.3 を無視してその値を
    /// そのまま採用する（自動算定なら異符号端モーメントで C=2.3 になる状況
    /// でも、c_direct=1.5 が優先されること）。
    #[test]
    fn test_c_factor_direct_input_overrides_auto() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, -100.0)), // 自動算定なら C=2.3（上限）
            steel_attr: Some(SteelDesignAttr {
                elem: ElemId(0),
                joint_flange_loss: 0.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 0.0,
                lb_direct: None,
                lateral_brace_count: None,
                lk_y_direct: None,
                lk_z_direct: None,
                c_direct: Some(1.5),
            }),
            ..Default::default()
        };
        // 自動算定結果（2.3）とは異なることも併せて確認する。
        let auto = steel_lateral_buckling_c(&ctx);
        assert!((auto - 2.3).abs() < 1e-9, "auto={auto}");
        assert_eq!(steel_c_factor(&ctx, false), 1.5);
        // 部分区間（横補剛あり）でも直接入力が優先され C=1.0 に落ちないこと。
        assert_eq!(steel_c_factor(&ctx, true), 1.5);
    }

    /// c_direct が無い場合、部分区間（lb_is_partial=true。横補剛により座屈
    /// 区間が部材の部分区間となり区間端モーメント比が不明）では安全側の
    /// C=1.0 とする（自動算定なら異なる値になる状況でも 1.0 に落ちること）。
    #[test]
    fn test_c_factor_partial_lb_without_direct_is_1_0() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, -100.0)), // 自動算定なら C=2.3
            ..Default::default()
        };
        assert_eq!(steel_c_factor(&ctx, true), 1.0);
        // 部分区間でなければ従来通り自動算定（2.3）になること。
        let auto = steel_c_factor(&ctx, false);
        assert!((auto - 2.3).abs() < 1e-9, "auto={auto}");
    }

    /// c_direct ≤ 0 は無効な入力（未入力相当）として無視し、2./3. の規定へ
    /// フォールバックすること。
    #[test]
    fn test_c_factor_direct_non_positive_falls_back() {
        let ctx_zero = DesignCtx {
            steel_attr: Some(SteelDesignAttr {
                elem: ElemId(0),
                joint_flange_loss: 0.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 0.0,
                lb_direct: None,
                lateral_brace_count: None,
                lk_y_direct: None,
                lk_z_direct: None,
                c_direct: Some(0.0),
            }),
            ..Default::default()
        };
        // c_direct=0 は無視され、lb_is_partial=true なら安全側 1.0。
        assert_eq!(steel_c_factor(&ctx_zero, true), 1.0);
        // lb_is_partial=false なら自動算定（end_moments_z=None → 1.0）。
        assert_eq!(steel_c_factor(&ctx_zero, false), 1.0);

        let ctx_neg = DesignCtx {
            steel_attr: Some(SteelDesignAttr {
                elem: ElemId(0),
                joint_flange_loss: 0.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 0.0,
                lb_direct: None,
                lateral_brace_count: None,
                lk_y_direct: None,
                lk_z_direct: None,
                c_direct: Some(-1.5),
            }),
            ..Default::default()
        };
        assert_eq!(steel_c_factor(&ctx_neg, true), 1.0);
    }

    /// C が増加すると fb1 の分母 `C·Λ²` が大きくなり、fb（=fb1 が支配的な
    /// 場合）が増加することを確認する。
    /// lb=5200 は fb1 が fb2 より大きく（fb1 が支配）、かつ長期許容引張
    /// 上限（cap=F/1.5）未満に収まるよう選定した値（fb2, cap によるクランプ
    /// で C の効果が隠れないようにするため）。
    #[test]
    fn test_fb_increases_with_c() {
        let f = 235.0;
        let i = 100.0;
        let h = 500.0;
        let af = 4000.0;
        let lb = 5200.0;
        let fb_c1 = steel_fb_h(f, LoadTerm::Long, lb, i, h, af, 1.0);
        let fb_c23 = steel_fb_h(f, LoadTerm::Long, lb, i, h, af, 2.3);
        let cap = f / 1.5;
        assert!(
            fb_c1 < cap - 1e-6,
            "fb1 が cap でクランプされていない前提: fb_c1={} cap={}",
            fb_c1,
            cap
        );
        assert!(
            fb_c23 > fb_c1,
            "C が大きいほど fb は増えるはず: fb(C=1.0)={} fb(C=2.3)={}",
            fb_c1,
            fb_c23
        );
    }

    // -------------------------------------------------------------
    // 断面欠損（継手部・スカラップ）と横座屈長さ
    // -------------------------------------------------------------

    /// βf=βw=αw=0 のとき、通常の H形強軸断面係数 Z=Iy/(H/2) に一致する
    /// （`Iy` は [`squid_n_core::section_shape::SectionShape::SteelH`] と同一の
    /// `(B・H³ − (B−tw)・(H−2tf)³)/12` 式）。
    #[test]
    fn test_z_with_loss_zero_matches_normal_z() {
        let (h, b, tw, tf): (f64, f64, f64, f64) = (500.0, 200.0, 9.0, 14.0);
        let hw = h - 2.0 * tf;
        let iy = (b * h.powi(3) - (b - tw) * hw.powi(3)) / 12.0;
        let z_expected = iy / (h / 2.0);

        let z_mid = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 0.0, false);
        let z_end = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 0.0, true);
        assert!(
            (z_mid - z_expected).abs() < 1e-6,
            "z_mid={} expected={}",
            z_mid,
            z_expected
        );
        assert!(
            (z_end - z_expected).abs() < 1e-6,
            "z_end={} expected={}",
            z_end,
            z_expected
        );
    }

    /// βf=100（フランジ全損）ならフランジ寄与 If' が消え、ウェブ寄与のみが
    /// 残る（Z' = Iw'/(H/2)）。
    #[test]
    fn test_z_with_loss_beta_f_100_removes_flange_contribution() {
        let (h, b, tw, tf): (f64, f64, f64, f64) = (500.0, 200.0, 9.0, 14.0);
        let hw = h - 2.0 * tf;
        let i_w = tw * hw.powi(3) / 12.0;
        let z_expected = i_w / (h / 2.0);

        let z = steel_h_z_with_loss(h, b, tw, tf, 100.0, 0.0, 0.0, false);
        assert!(
            (z - z_expected).abs() < 1e-6,
            "z={} expected={}",
            z,
            z_expected
        );
    }

    /// スカラップ αw の 3 乗効果: is_end=true で αw を大きくすると
    /// Iw''=tw・(hw・(1−αw/100))³/12 は (1−αw/100)³ に比例して小さくなる。
    #[test]
    fn test_z_with_loss_scallop_cubic_effect() {
        let (h, b, tw, tf): (f64, f64, f64, f64) = (500.0, 200.0, 9.0, 14.0);
        let z_no_scallop = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 0.0, true);
        let z_scallop = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 50.0, true);

        let hw = h - 2.0 * tf;
        let i_w_no_scallop = tw * hw.powi(3) / 12.0;
        let i_w_scallop = tw * (hw * 0.5).powi(3) / 12.0;
        // (1-0.5)^3 = 0.125 倍。
        assert!((i_w_scallop / i_w_no_scallop - 0.125).abs() < 1e-9);

        assert!(
            z_scallop < z_no_scallop,
            "z_scallop={} z_no_scallop={}",
            z_scallop,
            z_no_scallop
        );
        // is_end=false（スカラップ非適用）は αw の影響を受けない。
        let z_mid_with_alpha = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 50.0, false);
        let z_mid_no_alpha = steel_h_z_with_loss(h, b, tw, tf, 0.0, 0.0, 0.0, false);
        assert!((z_mid_with_alpha - z_mid_no_alpha).abs() < 1e-9);
    }

    /// resolve_lb: 直接入力があれば位置に応じて始端/中央/終端を返す。
    #[test]
    fn test_resolve_lb_direct_input_priority() {
        let direct = Some((1000.0, 2000.0, 3000.0));
        assert_eq!(resolve_lb(0.0, 9000.0, direct, Some(2)), 1000.0);
        assert_eq!(resolve_lb(0.24, 9000.0, direct, Some(2)), 1000.0);
        assert_eq!(resolve_lb(0.25, 9000.0, direct, Some(2)), 2000.0);
        assert_eq!(resolve_lb(0.5, 9000.0, direct, Some(2)), 2000.0);
        assert_eq!(resolve_lb(0.74, 9000.0, direct, Some(2)), 2000.0);
        assert_eq!(resolve_lb(0.75, 9000.0, direct, Some(2)), 3000.0);
        assert_eq!(resolve_lb(1.0, 9000.0, direct, Some(2)), 3000.0);
    }

    /// resolve_lb: 直接入力が無く等間隔横補剛の本数があれば L/(n+1)。
    #[test]
    fn test_resolve_lb_brace_count_when_no_direct() {
        // n=2 本の補剛で 3 等分 → 9000/3=3000。
        assert_eq!(resolve_lb(0.5, 9000.0, None, Some(2)), 3000.0);
        assert_eq!(resolve_lb(0.0, 9000.0, None, Some(2)), 3000.0);
    }

    /// resolve_lb: どちらも無ければ部材長そのまま。
    #[test]
    fn test_resolve_lb_falls_back_to_length() {
        assert_eq!(resolve_lb(0.5, 9000.0, None, None), 9000.0);
    }
}
