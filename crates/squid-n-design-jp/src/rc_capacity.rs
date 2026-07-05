//! RC 矩形断面の簡易終局耐力算定（部材ランク判定用）。
//!
//! squid-n-skeleton のファイバ解析（`build_rc_member_skeleton`）は Mu を精算できるが、
//! 保有水平耐力の部材ランク自動判定（`ds::rc_member_rank`）は毎フレーム実行されるため
//! 重すぎる。本モジュールは閉形式の簡易式で Qsu・Qmu を算定し、`rc_member_rank` の
//! 入力とする。係数は靭性指針・技術基準解説書等の略算式に基づく代表値であり、
//! 全て要・原典照合（specs/原典照合リスト.md）。

/// RC 矩形断面の簡易終局耐力算定用の入力一式。
pub struct RcCapacityInput {
    /// 断面幅 b \[mm\]
    pub b: f64,
    /// 断面せい D \[mm\]
    pub d: f64,
    /// 引張側主筋の総断面積 at \[mm²\]（片側）
    pub at: f64,
    /// 有効せい d_e \[mm\]（= D - かぶり - 主筋径/2 程度）
    pub d_eff: f64,
    /// 主筋降伏強度 σy \[N/mm²\]
    pub sigma_y: f64,
    /// コンクリート強度 Fc \[N/mm²\]
    pub fc: f64,
    /// せん断補強筋比 pw（= aw・組数/(b・ピッチ)）
    pub pw: f64,
    /// せん断補強筋降伏強度 σwy \[N/mm²\]
    pub sigma_wy: f64,
    /// 内法スパン h0 \[mm\]（反曲点中央を仮定し Qmu = 2Mu/h0）
    pub clear_span: f64,
    /// 軸方向圧縮応力度 σ0 \[N/mm²\]（既定 0）。`rc_qsu_simple` の軸力項
    /// `0.1・σ0・b・j` に用いる。荒川式の適用範囲である 0〜0.4Fc に
    /// `rc_qsu_simple` 内でクランプされる（要・原典照合）。
    pub sigma_0: f64,
}

/// 曲げ終局モーメント Mu ≈ 0.9・at・σy・j（引張鉄筋降伏型の略算式、j = 7・d_e/8）。
///
/// squid-n-skeleton のファイバ解析テスト（`test_rc_skeleton_ultimate_matches_handcalc`）が
/// 照合に使っている規準式 `Mu = 0.9・at・σy・j`（AIJ『非線形解析指針』等の簡易式、
/// 要・原典照合）と同一とする。
///
/// 不正入力（at, d_eff, σy のいずれかが 0 以下）は 0.0 を返す（`ds::rc_member_rank` は
/// qmu<=0 で FD を返す仕様に整合）。
pub fn rc_mu_simple(inp: &RcCapacityInput) -> f64 {
    if inp.at <= 0.0 || inp.d_eff <= 0.0 || inp.sigma_y <= 0.0 {
        return 0.0;
    }
    let j = 7.0 * inp.d_eff / 8.0;
    0.9 * inp.at * inp.sigma_y * j
}

/// 曲げ終局時せん断力 Qmu = 2・Mu / h0（両端曲げ降伏・反曲点中央を仮定）。
///
/// `clear_span`（h0）が 0 以下の場合は 0.0 を返す。
pub fn rc_qmu_simple(inp: &RcCapacityInput) -> f64 {
    if inp.clear_span <= 0.0 {
        return 0.0;
    }
    2.0 * rc_mu_simple(inp) / inp.clear_span
}

/// せん断終局耐力 Qsu \[N\]（荒川mean式系の略算式、要・原典照合）。
///
/// ```text
/// Qsu = { 0.068・pt^0.23・(Fc+18) / (M/(Q・d_e)+0.12) + 0.85・√(pw・σwy) + 0.1・σ0 }・b・j
/// ```
/// - `pt = 100・at/(b・d_e)` \[%\]（引張鉄筋比）
/// - `j = 7・d_e/8`
/// - せん断スパン比 `M/(Q・d_e) = h0/(2・d_e)` は反曲点中央（等曲げ勾配）の仮定から
///   導く略算のため、式の適用範囲である 1.0〜3.0 にクランプする。
/// - `pw` は式の適用範囲の上限 0.012 でクランプする（下限は 0）。
/// - 軸力項 `0.1・σ0`（σ0: 軸方向圧縮応力度）は荒川式の適用範囲である
///   0〜0.4Fc にクランプする（負の σ0（引張）は 0 とみなし、Qsu を低減しない
///   安全側の扱いとする）。
///
/// 全係数は要・原典照合（靭性指針/技術基準解説書等）。
/// 不正入力（b, d_eff, at, Fc, clear_span のいずれかが 0 以下）は 0.0 を返す。
pub fn rc_qsu_simple(inp: &RcCapacityInput) -> f64 {
    if inp.b <= 0.0 || inp.d_eff <= 0.0 || inp.at <= 0.0 || inp.fc <= 0.0 || inp.clear_span <= 0.0 {
        return 0.0;
    }
    let pt = 100.0 * inp.at / (inp.b * inp.d_eff);
    let j = 7.0 * inp.d_eff / 8.0;
    let shear_span_ratio = (inp.clear_span / (2.0 * inp.d_eff)).clamp(1.0, 3.0);
    let pw = inp.pw.clamp(0.0, 0.012);
    let concrete_term = 0.068 * pt.powf(0.23) * (inp.fc + 18.0) / (shear_span_ratio + 0.12);
    let hoop_term = 0.85 * (pw * inp.sigma_wy).max(0.0).sqrt();
    // 軸力項: 適用範囲 0〜0.4Fc にクランプ（荒川式の適用範囲、要・原典照合）。
    let sigma_0 = inp.sigma_0.clamp(0.0, 0.4 * inp.fc);
    let axial_term = 0.1 * sigma_0;
    (concrete_term + hoop_term + axial_term) * inp.b * j
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 代表断面: b=400, D=600, at=1935(D25×3程度), d_eff=530, σy=345, Fc=24,
    /// pw=0.002, σwy=295, h0=3000。
    fn sample_input() -> RcCapacityInput {
        RcCapacityInput {
            b: 400.0,
            d: 600.0,
            at: 1935.0,
            d_eff: 530.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: 0.002,
            sigma_wy: 295.0,
            clear_span: 3000.0,
            sigma_0: 0.0,
        }
    }

    #[test]
    fn test_rc_mu_simple_matches_handcalc() {
        let inp = sample_input();
        // 手計算: j = 7*530/8 = 463.75, Mu = 0.9*1935*345*463.75
        let j = 7.0 * 530.0 / 8.0;
        let mu_handcalc = 0.9 * 1935.0 * 345.0 * j;
        let mu = rc_mu_simple(&inp);
        assert!(
            (mu - mu_handcalc).abs() < 1e-6,
            "Mu={} vs handcalc={}",
            mu,
            mu_handcalc
        );
    }

    #[test]
    fn test_rc_qmu_simple_matches_handcalc() {
        let inp = sample_input();
        let j = 7.0 * 530.0 / 8.0;
        let mu_handcalc = 0.9 * 1935.0 * 345.0 * j;
        let qmu_handcalc = 2.0 * mu_handcalc / 3000.0;
        let qmu = rc_qmu_simple(&inp);
        assert!(
            (qmu - qmu_handcalc).abs() < 1e-6,
            "Qmu={} vs handcalc={}",
            qmu,
            qmu_handcalc
        );
    }

    #[test]
    fn test_rc_qsu_simple_matches_handcalc() {
        let inp = sample_input();
        // 手計算(クランプ域内): pt=100*1935/(400*530)=0.912736%,
        // shear_span_ratio=3000/(2*530)=2.830189(1.0-3.0の範囲内なのでクランプなし),
        // pw=0.002(0.012以下なのでクランプなし)。
        let pt: f64 = 100.0 * 1935.0 / (400.0 * 530.0);
        let j = 7.0 * 530.0 / 8.0;
        let shear_span_ratio: f64 = 3000.0 / (2.0 * 530.0);
        let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (shear_span_ratio + 0.12);
        let hoop_term = 0.85 * (0.002_f64 * 295.0).sqrt();
        let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;

        let qsu = rc_qsu_simple(&inp);
        assert!(
            (qsu - qsu_handcalc).abs() < 1e-6,
            "Qsu={} vs handcalc={}",
            qsu,
            qsu_handcalc
        );
        // 参考: せん断余裕度 Qsu/Qmu ≈ 1.6 程度（曲げ降伏が先行する健全な部材の目安）。
        let qmu = rc_qmu_simple(&inp);
        assert!(qsu / qmu > 1.0, "Qsu/Qmu={}", qsu / qmu);
    }

    #[test]
    fn test_rc_qsu_simple_clamps_shear_span_ratio_low() {
        // h0 を極端に短くすると shear_span_ratio = h0/(2*d_eff) < 1.0 → 1.0 にクランプ。
        let mut inp = sample_input();
        inp.clear_span = 200.0; // 200/(2*530)=0.1887 < 1.0
        let qsu = rc_qsu_simple(&inp);

        let pt: f64 = 100.0 * 1935.0 / (400.0 * 530.0);
        let j = 7.0 * 530.0 / 8.0;
        let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (1.0 + 0.12); // クランプ後 1.0
        let hoop_term = 0.85 * (0.002_f64 * 295.0).sqrt();
        let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;
        assert!(
            (qsu - qsu_handcalc).abs() < 1e-6,
            "Qsu={} vs handcalc(clamped)={}",
            qsu,
            qsu_handcalc
        );
    }

    #[test]
    fn test_rc_qsu_simple_clamps_shear_span_ratio_high() {
        // h0 を極端に長くすると shear_span_ratio = h0/(2*d_eff) > 3.0 → 3.0 にクランプ。
        let mut inp = sample_input();
        inp.clear_span = 6000.0; // 6000/(2*530)=5.660 > 3.0
        let qsu = rc_qsu_simple(&inp);

        let pt: f64 = 100.0 * 1935.0 / (400.0 * 530.0);
        let j = 7.0 * 530.0 / 8.0;
        let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (3.0 + 0.12); // クランプ後 3.0
        let hoop_term = 0.85 * (0.002_f64 * 295.0).sqrt();
        let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;
        assert!(
            (qsu - qsu_handcalc).abs() < 1e-6,
            "Qsu={} vs handcalc(clamped)={}",
            qsu,
            qsu_handcalc
        );
    }

    #[test]
    fn test_rc_qsu_simple_clamps_pw_upper_bound() {
        // pw が適用範囲の上限 0.012 を超える場合は 0.012 にクランプされる。
        let mut inp_over = sample_input();
        inp_over.pw = 0.05;
        let mut inp_clamped = sample_input();
        inp_clamped.pw = 0.012;

        let qsu_over = rc_qsu_simple(&inp_over);
        let qsu_clamped = rc_qsu_simple(&inp_clamped);
        assert!(
            (qsu_over - qsu_clamped).abs() < 1e-9,
            "qsu_over={} vs qsu_clamped={}",
            qsu_over,
            qsu_clamped
        );
        // クランプなしでは pw=0.05 の方が pw=0.002 より Qsu が大きくなるはず。
        assert!(qsu_clamped > rc_qsu_simple(&sample_input()));
    }

    #[test]
    fn test_rc_mu_simple_invalid_inputs_are_zero() {
        let base = sample_input();

        let mut at_zero = sample_input();
        at_zero.at = 0.0;
        assert_eq!(rc_mu_simple(&at_zero), 0.0);

        let mut d_eff_zero = sample_input();
        d_eff_zero.d_eff = 0.0;
        assert_eq!(rc_mu_simple(&d_eff_zero), 0.0);

        let mut sigma_y_zero = sample_input();
        sigma_y_zero.sigma_y = 0.0;
        assert_eq!(rc_mu_simple(&sigma_y_zero), 0.0);

        // 妥当な入力は正の値になることの確認（比較対象）。
        assert!(rc_mu_simple(&base) > 0.0);
    }

    #[test]
    fn test_rc_qmu_simple_zero_clear_span_is_zero() {
        let mut inp = sample_input();
        inp.clear_span = 0.0;
        assert_eq!(rc_qmu_simple(&inp), 0.0);

        let mut inp_neg = sample_input();
        inp_neg.clear_span = -100.0;
        assert_eq!(rc_qmu_simple(&inp_neg), 0.0);
    }

    #[test]
    fn test_rc_qsu_simple_invalid_inputs_are_zero() {
        let mut b_zero = sample_input();
        b_zero.b = 0.0;
        assert_eq!(rc_qsu_simple(&b_zero), 0.0);

        let mut d_eff_zero = sample_input();
        d_eff_zero.d_eff = 0.0;
        assert_eq!(rc_qsu_simple(&d_eff_zero), 0.0);

        let mut at_zero = sample_input();
        at_zero.at = 0.0;
        assert_eq!(rc_qsu_simple(&at_zero), 0.0);

        let mut fc_zero = sample_input();
        fc_zero.fc = 0.0;
        assert_eq!(rc_qsu_simple(&fc_zero), 0.0);

        let mut span_zero = sample_input();
        span_zero.clear_span = 0.0;
        assert_eq!(rc_qsu_simple(&span_zero), 0.0);
    }

    #[test]
    fn test_rc_qsu_simple_sigma_0_zero_matches_original() {
        // sigma_0=0.0（既定）は従来値と一致すること。
        let inp = sample_input();
        assert_eq!(inp.sigma_0, 0.0);
        let qsu = rc_qsu_simple(&inp);
        let pt: f64 = 100.0 * 1935.0 / (400.0 * 530.0);
        let j = 7.0 * 530.0 / 8.0;
        let shear_span_ratio: f64 = 3000.0 / (2.0 * 530.0);
        let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (shear_span_ratio + 0.12);
        let hoop_term = 0.85 * (0.002_f64 * 295.0).sqrt();
        let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;
        assert!((qsu - qsu_handcalc).abs() < 1e-6);
    }

    #[test]
    fn test_rc_qsu_simple_axial_term_adds_01_sigma0_b_j() {
        // 適用範囲内(0〜0.4Fc=9.6)の sigma_0=5.0 のとき、Qsu は
        // sigma_0=0 の場合に対して厳密に 0.1・σ0・b・j 分だけ増える。
        let mut inp = sample_input();
        let qsu_base = rc_qsu_simple(&inp);
        inp.sigma_0 = 5.0;
        let qsu_with_axial = rc_qsu_simple(&inp);
        let j = 7.0 * 530.0 / 8.0;
        let expected_delta = 0.1 * 5.0 * 400.0 * j;
        assert!(
            (qsu_with_axial - qsu_base - expected_delta).abs() < 1e-6,
            "delta={} expected={}",
            qsu_with_axial - qsu_base,
            expected_delta
        );
    }

    #[test]
    fn test_rc_qsu_simple_sigma_0_clamped_to_upper_bound_04fc() {
        // Fc=24.0 → 上限 0.4*24=9.6。これを超える sigma_0=20.0 は 9.6 にクランプされる。
        let mut inp_over = sample_input();
        inp_over.sigma_0 = 20.0;
        let mut inp_clamped = sample_input();
        inp_clamped.sigma_0 = 0.4 * 24.0;
        assert!((rc_qsu_simple(&inp_over) - rc_qsu_simple(&inp_clamped)).abs() < 1e-9);
        // クランプなしでは sigma_0=9.6 の方が sigma_0=0 より Qsu が大きいはず。
        assert!(rc_qsu_simple(&inp_clamped) > rc_qsu_simple(&sample_input()));
    }

    #[test]
    fn test_rc_qsu_simple_sigma_0_negative_is_clamped_to_zero() {
        // 負の sigma_0（引張）は 0 とみなす（Qsu を低減しない安全側）。
        let mut inp_neg = sample_input();
        inp_neg.sigma_0 = -10.0;
        let qsu_neg = rc_qsu_simple(&inp_neg);
        let qsu_zero = rc_qsu_simple(&sample_input());
        assert!((qsu_neg - qsu_zero).abs() < 1e-9);
    }
}
