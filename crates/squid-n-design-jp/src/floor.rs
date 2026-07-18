//! 床の中での小梁・スラブ（床）の設計。
//!
//! 小梁は大梁を分割せず、床の中で**単純支持梁**として検定する（反力は大梁へ CMQ
//! 荷重として伝達する扱い）。スラブは一方向の曲げとして設計曲げモーメントと必要
//! 鉄筋量を算定する。いずれも全体 FEM から独立した床設計として扱う。
//!
//! 単位は N-mm（面荷重 N/mm²、線荷重 N/mm）。

/// 小梁（単純支持梁）の設計結果。
#[derive(Clone, Debug, PartialEq)]
pub struct JoistDesignResult {
    /// スパン（支持間距離）[mm]。
    pub span: f64,
    /// 等分布荷重 w [N/mm]（＝面荷重 × 負担幅 spacing）。
    pub w: f64,
    /// 最大曲げモーメント M = wL²/8 [N·mm]（中央）。
    pub m_max: f64,
    /// 最大せん断力 Q = wL/2 [N]（端部）。
    pub q_max: f64,
    /// 曲げ応力度 σ = M/Z [N/mm²]。
    pub sigma: f64,
    /// 許容曲げ応力度 [N/mm²]。
    pub sigma_allow: f64,
    /// 曲げ検定比 σ/σ_allow。
    pub bending_ratio: f64,
    /// たわみ δ = 5wL⁴/(384EI) [mm]。
    pub deflection: f64,
    /// たわみ比 δ/L。
    pub deflection_span_ratio: f64,
    /// たわみ検定比 (δ/L)/(1/limit_denom)。
    pub deflection_ratio: f64,
    /// 総合検定比（曲げ・たわみの最大）。
    pub ratio: f64,
    /// 判定（`ratio <= 1`）。
    pub ok: bool,
}

/// 小梁を単純支持梁として設計する（床の中での小梁設計）。
///
/// - `span`: 支持間距離 [mm]、`w`: 等分布荷重 [N/mm]（面荷重 × 負担幅）。
/// - `section_modulus`: 断面係数 Z [mm³]（強軸）、`inertia`: 断面二次モーメント I [mm⁴]。
/// - `young`: ヤング係数 E [N/mm²]、`sigma_allow`: 許容曲げ応力度 [N/mm²]（長期）。
/// - `defl_limit_denom`: たわみ制限の分母（例: 250 なら δ/L ≤ 1/250）。
///
/// `section_modulus`・`inertia`・`young` が 0 以下の場合は該当検定比を 0 とする
/// （断面情報が不足する場合の安全なフォールバック）。
#[allow(clippy::too_many_arguments)]
pub fn design_joist_simple(
    span: f64,
    w: f64,
    section_modulus: f64,
    inertia: f64,
    young: f64,
    sigma_allow: f64,
    defl_limit_denom: f64,
) -> JoistDesignResult {
    let m_max = w * span * span / 8.0;
    let q_max = w * span / 2.0;
    let sigma = if section_modulus > 0.0 {
        m_max / section_modulus
    } else {
        0.0
    };
    let bending_ratio = if sigma_allow > 0.0 {
        sigma / sigma_allow
    } else {
        0.0
    };
    let deflection = if young > 0.0 && inertia > 0.0 {
        5.0 * w * span.powi(4) / (384.0 * young * inertia)
    } else {
        0.0
    };
    let deflection_span_ratio = if span > 0.0 { deflection / span } else { 0.0 };
    let deflection_ratio = if defl_limit_denom > 0.0 {
        deflection_span_ratio * defl_limit_denom
    } else {
        0.0
    };
    let ratio = bending_ratio.max(deflection_ratio);
    JoistDesignResult {
        span,
        w,
        m_max,
        q_max,
        sigma,
        sigma_allow,
        bending_ratio,
        deflection,
        deflection_span_ratio,
        deflection_ratio,
        ratio,
        ok: ratio <= 1.0,
    }
}

/// スラブ（一方向）の設計結果。
#[derive(Clone, Debug, PartialEq)]
pub struct SlabDesignResult {
    /// 設計スパン（短辺）[mm]。
    pub span: f64,
    /// 面荷重 w [N/mm²]。
    pub w: f64,
    /// 単位幅あたり設計曲げモーメント M = wL²/coef [N·mm/mm]。
    pub moment: f64,
    /// 板厚 t [mm]。
    pub thickness: f64,
    /// 有効せい d = t − かぶり [mm]。
    pub effective_depth: f64,
    /// 単位幅あたり必要引張鉄筋量 As [mm²/mm]。
    pub as_req_per_mm: f64,
    /// 1m あたり必要引張鉄筋量 As [mm²/m]（表示用）。
    pub as_req_per_m: f64,
}

/// スラブを一方向版として設計し、設計曲げモーメントと必要鉄筋量を算定する。
///
/// - `span`: 設計スパン（短辺）[mm]、`w`: 面荷重 [N/mm²]。
/// - `moment_coef`: 曲げモーメント係数（M = wL²/coef。単純支持=8、連続端=10 等）。
/// - `thickness`: 板厚 [mm]、`cover`: 圧縮縁から鉄筋重心までのかぶり [mm]。
/// - `ft_rebar`: 鉄筋の許容引張応力度 [N/mm²]（長期）、`j_ratio`: 応力中心距離比 j（≒7/8）。
///
/// 必要鉄筋量 As = M / (ft · j · d)。有効せい d が 0 以下なら As=0。
#[allow(clippy::too_many_arguments)]
pub fn design_slab_oneway(
    span: f64,
    w: f64,
    moment_coef: f64,
    thickness: f64,
    cover: f64,
    ft_rebar: f64,
    j_ratio: f64,
) -> SlabDesignResult {
    let coef = if moment_coef > 0.0 { moment_coef } else { 8.0 };
    let moment = w * span * span / coef; // N·mm per mm width
    let d = (thickness - cover).max(0.0);
    let as_req_per_mm = if ft_rebar > 0.0 && j_ratio > 0.0 && d > 0.0 {
        moment / (ft_rebar * j_ratio * d)
    } else {
        0.0
    };
    SlabDesignResult {
        span,
        w,
        moment,
        thickness,
        effective_depth: d,
        as_req_per_mm,
        as_req_per_m: as_req_per_mm * 1000.0,
    }
}

/// 鋼小梁の既定ヤング係数 [N/mm²]。
pub const STEEL_YOUNG: f64 = 205_000.0;
/// たわみ制限の既定分母（δ/L ≤ 1/250）。
pub const DEFLECTION_LIMIT_DENOM: f64 = 250.0;
/// スラブ設計の既定かぶり [mm]（圧縮縁〜鉄筋重心）。
pub const SLAB_DEFAULT_COVER: f64 = 30.0;
/// スラブ設計の既定 j 比（応力中心距離 / 有効せい）。
pub const SLAB_J_RATIO: f64 = 7.0 / 8.0;
/// 異形鉄筋 SD295 の長期許容引張応力度 [N/mm²]。
pub const REBAR_FT_LONG_SD295: f64 = 195.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joist_simple_bending_and_shear() {
        // w=10 N/mm, L=4000mm → M=wL²/8=2.0e7 N·mm, Q=wL/2=2.0e4 N。
        let r = design_joist_simple(4000.0, 10.0, 1.0e6, 1.0e8, STEEL_YOUNG, 156.0, 250.0);
        assert!((r.m_max - 2.0e7).abs() < 1.0, "M={}", r.m_max);
        assert!((r.q_max - 2.0e4).abs() < 1.0, "Q={}", r.q_max);
        // σ = M/Z = 2e7/1e6 = 20。
        assert!((r.sigma - 20.0).abs() < 1e-9);
        assert!((r.bending_ratio - 20.0 / 156.0).abs() < 1e-9);
        // δ = 5wL⁴/(384EI) = 5·10·4000⁴/(384·205000·1e8)。
        let expect_defl = 5.0 * 10.0 * 4000.0_f64.powi(4) / (384.0 * STEEL_YOUNG * 1.0e8);
        assert!((r.deflection - expect_defl).abs() / expect_defl < 1e-9);
    }

    #[test]
    fn test_joist_zero_section_is_safe() {
        // 断面情報ゼロでもパニックせず、検定比 0。
        let r = design_joist_simple(4000.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(r.bending_ratio, 0.0);
        assert_eq!(r.deflection_ratio, 0.0);
        assert!(r.ok);
    }

    #[test]
    fn test_slab_oneway_moment_and_rebar() {
        // w=0.005 N/mm², L=3000mm, 単純支持(coef=8) → M=wL²/8=5625 N·mm/mm。
        let r = design_slab_oneway(
            3000.0,
            0.005,
            8.0,
            150.0,
            SLAB_DEFAULT_COVER,
            REBAR_FT_LONG_SD295,
            SLAB_J_RATIO,
        );
        assert!((r.moment - 0.005 * 3000.0 * 3000.0 / 8.0).abs() < 1e-6);
        assert!((r.effective_depth - 120.0).abs() < 1e-9);
        // As = M/(ft·j·d)。
        let expect_as = r.moment / (REBAR_FT_LONG_SD295 * SLAB_J_RATIO * 120.0);
        assert!((r.as_req_per_mm - expect_as).abs() / expect_as < 1e-9);
        assert!((r.as_req_per_m - expect_as * 1000.0).abs() / (expect_as * 1000.0) < 1e-9);
    }
}
