//! 鉄骨の断面検定における断面性能（鋼構造設計規準の
//! 鉄骨の断面検定における断面性能）。
//!
//! 横座屈を考慮した許容曲げ応力度 fb（鋼構造設計規準 1973）・断面二次半径・
//! 断面欠損（継手部・スカラップ）を考慮した断面係数 Z'・横座屈長さ lb の
//! 解決を扱う。

use crate::material_strength::big_lambda;
use crate::{DesignCtx, LoadTerm};

use super::section_modulus;

// ---------------------------------------------------------------------
// 許容曲げ応力度（横座屈考慮）と断面二次半径（鋼構造設計規準 1973）
// ---------------------------------------------------------------------

/// 「圧縮フランジ＋せいの 1/6 のウェブ」からなる T 形断面の、ウェブ軸まわり
/// 断面二次半径 `i = √(I_T/A_T)`（鋼構造設計規準 1973、横座屈許容曲げ fb1 用）。
///
/// `I_T = tf·B³/12 + (h/6)·tw³/12`、`A_T = B·tf + (h/6)·tw`。
pub fn steel_i_t(b: f64, tf: f64, h: f64, tw: f64) -> f64 {
    let i_t = tf * b.powi(3) / 12.0 + (h / 6.0) * tw.powi(3) / 12.0;
    let a_t = b * tf + (h / 6.0) * tw;
    if a_t > 1e-12 {
        (i_t / a_t).sqrt()
    } else {
        0.0
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
        let expected_it = 15.0 * 200.0_f64.powi(3) / 12.0 + (400.0 / 6.0) * 10.0_f64.powi(3) / 12.0;
        let expected_at = 200.0 * 15.0 + (400.0 / 6.0) * 10.0;
        let expected = (expected_it / expected_at).sqrt();
        assert!((i - expected).abs() < 1e-9);
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
