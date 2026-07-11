//! RC耐震壁の開口低減・複数開口の等価開口置換・耐震壁判定
//! （RESP-D マニュアル「計算編 02 剛性計算」準拠）。
//!
//! 壁の面内剛性を評価する際に、開口の大きさに応じて剛性を低減するための
//! 開口周比・開口低減率、複数の開口を1つの等価開口に置換する計算、および
//! 壁を耐震壁として扱ってよいかどうかの判定条件をまとめる。
//!
//! 壁要素の開口寸法を保持するデータモデルは本実装時点では未整備のため、
//! 本モジュールは入力を受け取る純関数として提供するに留める。モデルに
//! 開口情報（開口高さ・開口長さ）が追加され次第、呼び出し側からそのまま
//! 利用できる。
//!
//! なお [`crate::rc::wall::rc_wall_shear_check`] が持つ開口低減係数
//! （`r = min(γ1, γ2, γ3)`）は RC規準18条（耐震壁のせん断耐力検定）の
//! 規定であり、本モジュールの剛性計算用の低減率 `r = 1 − 1.25・r0` とは
//! 準拠する規定（マニュアルの章）も算定目的（耐力 vs 剛性）も異なる別物
//! である。両者は数式が異なるため混同・統合しないこと。

/// 開口周比 r0 = √(h0・l0 / (h・l))（RESP-D マニュアル 02 剛性計算）。
///
/// - `h0`, `l0`: 開口部分の高さ・長さ [mm]
/// - `h`, `l`: 壁の内法高さ・長さ [mm]
///
/// `h`,`l` のいずれかが 0 以下（壁寸法が未設定など）の場合は 0 除算を避ける
/// ため 0.0 を返す。
pub fn opening_ratio_r0(h0: f64, l0: f64, h: f64, l: f64) -> f64 {
    if h <= 0.0 || l <= 0.0 {
        return 0.0;
    }
    ((h0 * l0) / (h * l)).max(0.0).sqrt()
}

/// 開口による剛性低減率 r = 1 − 1.25・r0（RESP-D マニュアル 02 剛性計算）。
///
/// `r0`（[`opening_ratio_r0`]）が大きい場合、計算上 r が負になり得るため
/// 安全側として 0 に下限クランプする。開口が無い場合（h0=l0=0）は r=1。
pub fn opening_reduction_r(h0: f64, l0: f64, h: f64, l: f64) -> f64 {
    let r0 = opening_ratio_r0(h0, l0, h, l);
    (1.0 - 1.25 * r0).max(0.0)
}

/// 複数開口の等価開口置換（RESP-D マニュアル 02 剛性計算）。
///
/// 壁面に複数の開口 `(li, hi)`（開口長さ・開口高さ）[mm] がある場合、面積の
/// 総和を保ちつつ、壁の内法幅 `lw` ・内法高さ `hw` と等しい辺長比を持つ
/// 単一の等価開口 `(l0', h0')` に置換する。
///
/// - 面積保存: `l0'・h0' = Σ(li・hi)`
/// - 辺長比: `l0' : h0' = lw : hw`
///
/// 上記2条件を解くと `l0' = lw・√(Σli・hi / (lw・hw))`,
/// `h0' = hw・√(Σli・hi / (lw・hw))` となる。
///
/// `lw`,`hw` のいずれかが 0 以下、または開口が無い（面積総和が 0 以下）場合は
/// `(0.0, 0.0)` を返す。
pub fn equivalent_opening(openings: &[(f64, f64)], lw: f64, hw: f64) -> (f64, f64) {
    if lw <= 0.0 || hw <= 0.0 {
        return (0.0, 0.0);
    }
    let sum_area: f64 = openings.iter().map(|(li, hi)| li * hi).sum();
    if sum_area <= 0.0 {
        return (0.0, 0.0);
    }
    let k = (sum_area / (lw * hw)).sqrt();
    (lw * k, hw * k)
}

/// RC耐震壁として扱えるかどうかの判定入力（RESP-D マニュアル 02 剛性計算）。
pub struct WallJudgeInput {
    /// 壁厚 t [mm]。
    pub thickness: f64,
    /// 開口周比 r0（[`opening_ratio_r0`] 参照）。
    pub r0: f64,
    /// スリット（構造スリット等による縁切り）の有無。
    pub has_slit: bool,
}

/// RC耐震壁の判定（RESP-D マニュアル 02 剛性計算）。
///
/// 次の3条件をすべて満たす場合に耐震壁として扱う:
/// - スリットがない
/// - 壁厚 t ≥ 120mm
/// - 開口周比 r0 ≤ 0.4
///
/// マニュアルでは指定により開口幅比 L0/L・開口高さ比 H0/H を追加条件として
/// 含められるが、本実装では基本の3条件のみを扱う。
pub fn is_seismic_wall(input: &WallJudgeInput) -> bool {
    !input.has_slit && input.thickness >= 120.0 && input.r0 <= 0.4
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------
    // 開口周比・開口低減率
    // ------------------------------------------------------------------

    #[test]
    fn r0_and_r_representative_value() {
        // h0*l0/(h*l) = 200*200/(1000*1000) = 0.04 → r0=0.2, r=1-1.25*0.2=0.75
        let r0 = opening_ratio_r0(200.0, 200.0, 1000.0, 1000.0);
        assert!((r0 - 0.2).abs() < 1e-9);
        let r = opening_reduction_r(200.0, 200.0, 1000.0, 1000.0);
        assert!((r - 0.75).abs() < 1e-9);
    }

    #[test]
    fn r_is_one_when_no_opening() {
        let r0 = opening_ratio_r0(0.0, 0.0, 3000.0, 6000.0);
        assert_eq!(r0, 0.0);
        let r = opening_reduction_r(0.0, 0.0, 3000.0, 6000.0);
        assert_eq!(r, 1.0);
    }

    #[test]
    fn r_is_clamped_to_zero_for_large_opening() {
        // h0*l0/(h*l) = 1000*1000/(100*100) = 100 → r0=10, r=1-12.5=負→0クランプ
        let r0 = opening_ratio_r0(1000.0, 1000.0, 100.0, 100.0);
        assert!(r0 > 0.8);
        let r = opening_reduction_r(1000.0, 1000.0, 100.0, 100.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn r0_guards_against_zero_division() {
        // 壁寸法が 0 の場合は 0 除算せず 0.0 を返す。
        assert_eq!(opening_ratio_r0(100.0, 100.0, 0.0, 1000.0), 0.0);
        assert_eq!(opening_ratio_r0(100.0, 100.0, 1000.0, 0.0), 0.0);
    }

    // ------------------------------------------------------------------
    // 複数開口の等価開口置換
    // ------------------------------------------------------------------

    #[test]
    fn equivalent_opening_matches_area_and_aspect_ratio() {
        // l1*h1=1000*1000, l2*h2=500*500, lw=6000, hw=3000
        let openings = [(1000.0, 1000.0), (500.0, 500.0)];
        let (l0p, h0p) = equivalent_opening(&openings, 6000.0, 3000.0);

        let sum_area: f64 = 1000.0 * 1000.0 + 500.0 * 500.0;
        assert!((sum_area - 1_250_000.0).abs() < 1e-6);

        // 面積保存: l0'・h0' = Σ(li・hi)
        assert!((l0p * h0p - sum_area).abs() < 1e-6);
        // 辺長比: l0'/h0' = lw/hw = 2
        assert!((l0p / h0p - 2.0).abs() < 1e-9);

        // 手計算値: l0'=sqrt(1.25e6*2)≈1581.1, h0'≈790.6
        assert!((l0p - 1_581.138_8).abs() < 1e-3);
        assert!((h0p - 790.569_4).abs() < 1e-3);
    }

    #[test]
    fn equivalent_opening_no_openings_is_zero() {
        let (l0p, h0p) = equivalent_opening(&[], 6000.0, 3000.0);
        assert_eq!((l0p, h0p), (0.0, 0.0));
    }

    // ------------------------------------------------------------------
    // 耐震壁判定
    // ------------------------------------------------------------------

    #[test]
    fn is_seismic_wall_boundary_thickness_ok() {
        let inp = WallJudgeInput {
            thickness: 120.0,
            r0: 0.1,
            has_slit: false,
        };
        assert!(is_seismic_wall(&inp));
    }

    #[test]
    fn is_seismic_wall_boundary_r0_ok() {
        let inp = WallJudgeInput {
            thickness: 150.0,
            r0: 0.4,
            has_slit: false,
        };
        assert!(is_seismic_wall(&inp));
    }

    #[test]
    fn is_seismic_wall_false_when_slit_present() {
        let inp = WallJudgeInput {
            thickness: 150.0,
            r0: 0.1,
            has_slit: true,
        };
        assert!(!is_seismic_wall(&inp));
    }

    #[test]
    fn is_seismic_wall_false_when_thin_or_large_opening() {
        let thin = WallJudgeInput {
            thickness: 119.999,
            r0: 0.1,
            has_slit: false,
        };
        assert!(!is_seismic_wall(&thin));

        let big_opening = WallJudgeInput {
            thickness: 150.0,
            r0: 0.400_001,
            has_slit: false,
        };
        assert!(!is_seismic_wall(&big_opening));
    }
}
