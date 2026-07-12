//! 鉄骨鉄筋コンクリート造梁の**せん断終局強度**（RESP-D マニュアル「計算編 05
//! 非線形モデル」SRC 梁のせん断復元力特性）。
//!
//! # 位置付け
//! 非線形解析のせん断ばね終局耐力を算定する純関数群。RESP-D は 3 式
//! （SRC規準／SRC診断式／構造関係技術基準解説書）から選択できる。本モジュールは
//! 数式が一意な**構造関係技術基準解説書**式と**SRC規準**式を実装する。
//! いずれも RC 部分 rQu と鉄骨部分 sQu の累加 `Qu = rQu + sQu`。
//!
//! # 準拠する規準・出典
//! - 構造関係技術基準解説書 SRC 梁せん断終局。
//! - SRC 規準 SRC 梁せん断終局。
//! いずれも係数は要・原典照合（`specs/原典照合リスト.md`）。

/// SRC 梁せん断終局の算定入力（累加式 Qu=rQu+sQu 共通）。
#[derive(Clone, Copy, Debug)]
pub struct SrcBeamShearInput {
    /// 梁幅 b [mm]。
    pub b: f64,
    /// RC 部分の応力中心間距離 rj [mm]。
    pub rj: f64,
    /// 梁の有効せい rd [mm]。
    pub rd: f64,
    /// せん断補強筋比 rpw（小数）。
    pub rpw: f64,
    /// せん断補強筋の材料強度 rwσy [N/mm²]。
    pub rw_sigma_y: f64,
    /// 鉄骨フランジ位置のコンクリート有効幅 b' [mm]。
    pub b_prime: f64,
    /// せん断力方向の鉄骨ウェブ断面積 sAw [mm²]。
    pub s_aw: f64,
    /// 鉄骨ウェブの材料強度 sσy [N/mm²]。
    pub s_sigma_y: f64,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 鉄骨部分の両端全塑性モーメントの和 ΣsMu [N·mm]（sQu2=ΣsMu/l'）。
    pub sum_s_mu: f64,
    /// 内法スパン l' [mm]。
    pub clear_span: f64,
    /// せん断スパン比 M/(Q·rd)。
    pub m_over_qrd: f64,
}

/// SRC 梁せん断終局強度 Qu [N]（構造関係技術基準解説書式）。
///
/// ```text
/// Qu = rQu + sQu
/// rQu = min(rQu1, rQu2),  sQu = min(sQu1, sQu2)
/// rQu1 = b·rj·(α·fs + 0.5·rpw·rwσy)
/// rQu2 = b·rj·(2·b'/b·fs + rpw·rwσy)
/// sQu1 = sAw·sσy/√3,  sQu2 = ΣsMu/l'
/// fs   = min(Fc/20, (5 + Fc/100)·1.5)
/// α    = 4/(M/(Q·rd)+1)（1≤α≤2 にクランプ）
/// ```
/// 不正入力（b・rj・Fc・l' のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_ultimate_tech_standard(inp: &SrcBeamShearInput) -> f64 {
    if inp.b <= 0.0 || inp.rj <= 0.0 || inp.fc <= 0.0 || inp.clear_span <= 0.0 {
        return 0.0;
    }
    let fs = (inp.fc / 20.0).min((5.0 + inp.fc / 100.0) * 1.5);
    let alpha = (4.0 / (inp.m_over_qrd.max(0.0) + 1.0)).clamp(1.0, 2.0);
    let rqu1 = inp.b * inp.rj * (alpha * fs + 0.5 * inp.rpw * inp.rw_sigma_y);
    let rqu2 = inp.b * inp.rj * (2.0 * inp.b_prime / inp.b * fs + inp.rpw * inp.rw_sigma_y);
    let rqu = rqu1.min(rqu2);
    let squ1 = inp.s_aw * inp.s_sigma_y / 3.0_f64.sqrt();
    let squ2 = inp.sum_s_mu / inp.clear_span;
    let squ = squ1.min(squ2);
    (rqu + squ).max(0.0)
}

/// SRC 梁せん断終局強度 Qu [N]（SRC 規準式）。
///
/// 技術基準解説書式との差異:
/// - `rQu1 = b·rj·(0.5·α·fs + 0.5·rpw·rwσy)`（fs 係数が 0.5·α）
/// - `rQu2 = b·rj·(b'/b·fs + rpw·rwσy)`（b'/b の係数が 1）
/// - `fs = min(0.15·Fc, (22.5 + 4.5·Fc)/100)`（要・原典照合）
///
/// 不正入力（b・rj・Fc・l' のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_ultimate_src_standard(inp: &SrcBeamShearInput) -> f64 {
    if inp.b <= 0.0 || inp.rj <= 0.0 || inp.fc <= 0.0 || inp.clear_span <= 0.0 {
        return 0.0;
    }
    let fs = (0.15 * inp.fc).min((22.5 + 4.5 * inp.fc) / 100.0);
    let alpha = (4.0 / (inp.m_over_qrd.max(0.0) + 1.0)).clamp(1.0, 2.0);
    let rqu1 = inp.b * inp.rj * (0.5 * alpha * fs + 0.5 * inp.rpw * inp.rw_sigma_y);
    let rqu2 = inp.b * inp.rj * (inp.b_prime / inp.b * fs + inp.rpw * inp.rw_sigma_y);
    let rqu = rqu1.min(rqu2);
    let squ1 = inp.s_aw * inp.s_sigma_y / 3.0_f64.sqrt();
    let squ2 = inp.sum_s_mu / inp.clear_span;
    let squ = squ1.min(squ2);
    (rqu + squ).max(0.0)
}

/// 非充腹 SRC 梁（格子材・ラチス材）のせん断終局強度の算定入力
/// （荒川mean式系、RESP-D「05 非線形モデル」）。
#[derive(Clone, Copy, Debug)]
pub struct SrcNonSolidWebShearInput {
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 梁の有効幅 be [mm]（= ΣAg/D、スラブ断面積を加算した全断面積/全せい）。
    pub be: f64,
    /// 全せい D [mm]（応力中心間距離 j = 0.8·D）。
    pub d_full: f64,
    /// 有効せい d [mm]（せん断スパン比 M/(Q·d) 用）。
    pub d_eff: f64,
    /// 引張鉄筋比 rpt [%]。
    pub rpt: f64,
    /// 引張鉄骨比 spt [%]（格子材の pt=rpt+spt に加算。ラチス材は未使用）。
    pub spt: f64,
    /// 鉄骨フランジ側面のせん断破壊低減係数 kcs（= 0.5+b'/b、1.0 以下）。
    pub kcs: f64,
    /// せん断補強筋比 rpw（小数）。
    pub rpw: f64,
    /// せん断補強筋の降伏点強度 rσwy [N/mm²]。
    pub rw_sigma_y: f64,
    /// 帯板比 spw（小数。格子材のみ）。
    pub spw: f64,
    /// 帯板の降伏点強度 sσwy [N/mm²]（格子材のみ）。
    pub s_band_sigma_y: f64,
    /// せん断スパン比 M/(Q·d)（適用範囲 1.0〜3.0 にクランプ）。
    pub m_over_qd: f64,
    /// 高強度せん断補強筋を用いる場合 true（κ 0.053→0.068）。
    pub high_strength_shear_rebar: bool,
    /// RC 部分の応力中心間距離 rj [mm]（ラチス材のみ）。
    pub rj: f64,
    /// ラチス材 1 本の断面積 DA [mm²]（ラチス材のみ）。
    pub lattice_area: f64,
    /// ラチス材の降伏点強度 sσy [N/mm²]（ラチス材のみ）。
    pub lattice_sigma_y: f64,
    /// ラチス材と材軸のなす角 θ [rad]（ラチス材のみ）。
    pub lattice_angle: f64,
    /// 鉄骨部分の全塑性モーメント sM0 [N·mm]（ラチス材の sQu 用）。
    pub s_m0: f64,
    /// 内法長さ h0 [mm]（ラチス材の sQu = 2·sM0/h0 用）。
    pub clear_span: f64,
}

/// κ（せん断補強筋係数）を返す。
fn nonweb_kappa(high: bool) -> f64 {
    if high {
        0.068
    } else {
        0.053
    }
}

/// 非充腹 SRC 梁（**格子材**）のせん断終局強度 Qsu [N]（RESP-D 非線形モデル）。
///
/// ```text
/// Qsu = { κ·pt^0.23·kcs·(18+Fc)/(M/(Q·d)+0.12) + 0.85·√(rpw·rσwy)
///         + (1/2)·√(spw·sσwy) }·be·j
/// ```
/// - `pt = rpt + spt` [%]、`j = 0.8·D`、`M/(Q·d)` は 1.0〜3.0 にクランプ、
///   `kcs ≤ 1.0`、κ=0.053/0.068。
///
/// 不正入力（Fc・be・D のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_grid(inp: &SrcNonSolidWebShearInput) -> f64 {
    if inp.fc <= 0.0 || inp.be <= 0.0 || inp.d_full <= 0.0 {
        return 0.0;
    }
    let pt = (inp.rpt + inp.spt).max(0.0);
    let kcs = inp.kcs.clamp(0.0, 1.0);
    let ssr = inp.m_over_qd.clamp(1.0, 3.0);
    let j = 0.8 * inp.d_full;
    let k = nonweb_kappa(inp.high_strength_shear_rebar);
    let concrete = k * pt.powf(0.23) * kcs * (18.0 + inp.fc) / (ssr + 0.12);
    let hoop_r = 0.85 * (inp.rpw * inp.rw_sigma_y).max(0.0).sqrt();
    let hoop_s = 0.5 * (inp.spw * inp.s_band_sigma_y).max(0.0).sqrt();
    (concrete + hoop_r + hoop_s) * inp.be * j
}

/// 非充腹 SRC 梁（**ラチス材**）のせん断終局強度 Qsu [N]（RESP-D 非線形モデル）。
///
/// ```text
/// Qsu = { κ·rpt^0.23·kcs·(18+Fc)/(M/(Q·d)+0.12) + 0.85·√(rpw·rσwy) }·be·rj + sQu
/// sQu = min( 2·sM0/h0 ,  DA·sσy·sinθ )
/// ```
/// - RC 部分は `rpt`（引張鉄筋比 [%]）のみ、`rj`（RC 部応力中心間距離）を用いる。
/// - `sQu` はラチス鉄骨のせん断寄与（曲げ降伏または引張降伏の小さい方）。
///
/// 不正入力（Fc・be・rj のいずれかが 0 以下）は 0.0。
pub fn src_beam_shear_lattice(inp: &SrcNonSolidWebShearInput) -> f64 {
    if inp.fc <= 0.0 || inp.be <= 0.0 || inp.rj <= 0.0 {
        return 0.0;
    }
    let kcs = inp.kcs.clamp(0.0, 1.0);
    let ssr = inp.m_over_qd.clamp(1.0, 3.0);
    let k = nonweb_kappa(inp.high_strength_shear_rebar);
    let concrete = k * inp.rpt.max(0.0).powf(0.23) * kcs * (18.0 + inp.fc) / (ssr + 0.12);
    let hoop_r = 0.85 * (inp.rpw * inp.rw_sigma_y).max(0.0).sqrt();
    let rc_part = (concrete + hoop_r) * inp.be * inp.rj;
    // ラチス鉄骨の寄与 sQu。
    let squ_bending = if inp.clear_span > 0.0 {
        2.0 * inp.s_m0 / inp.clear_span
    } else {
        f64::INFINITY
    };
    let squ_tension =
        inp.lattice_area.max(0.0) * inp.lattice_sigma_y.max(0.0) * inp.lattice_angle.sin().abs();
    let squ = squ_bending.min(squ_tension).max(0.0);
    rc_part + squ
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_input() -> SrcBeamShearInput {
        SrcBeamShearInput {
            b: 500.0,
            rj: 600.0,
            rd: 700.0,
            rpw: 0.004,
            rw_sigma_y: 295.0,
            b_prime: 300.0,
            s_aw: 5000.0,
            s_sigma_y: 235.0,
            fc: 24.0,
            sum_s_mu: 4.0e8,
            clear_span: 6000.0,
            m_over_qrd: 2.0,
        }
    }

    #[test]
    fn test_src_beam_shear_tech_standard_matches_handcalc() {
        let inp = base_input();
        let qu = src_beam_shear_ultimate_tech_standard(&inp);
        let fs = (24.0_f64 / 20.0).min((5.0 + 24.0 / 100.0) * 1.5);
        let alpha = (4.0_f64 / (2.0 + 1.0)).clamp(1.0, 2.0);
        let rqu1 = 500.0 * 600.0 * (alpha * fs + 0.5 * 0.004 * 295.0);
        let rqu2 = 500.0 * 600.0 * (2.0 * 300.0 / 500.0 * fs + 0.004 * 295.0);
        let rqu = rqu1.min(rqu2);
        let squ1 = 5000.0 * 235.0 / 3.0_f64.sqrt();
        let squ2 = 4.0e8 / 6000.0;
        let squ = squ1.min(squ2);
        let hand = rqu + squ;
        assert!((qu - hand).abs() < 1e-3, "Qu={qu} vs {hand}");
        assert!(qu > 0.0);
    }

    #[test]
    fn test_src_beam_shear_src_standard_matches_handcalc() {
        let inp = base_input();
        let qu = src_beam_shear_ultimate_src_standard(&inp);
        let fs = (0.15_f64 * 24.0).min((22.5 + 4.5 * 24.0) / 100.0);
        let alpha = (4.0_f64 / (2.0 + 1.0)).clamp(1.0, 2.0);
        let rqu1 = 500.0 * 600.0 * (0.5 * alpha * fs + 0.5 * 0.004 * 295.0);
        let rqu2 = 500.0 * 600.0 * (300.0 / 500.0 * fs + 0.004 * 295.0);
        let rqu = rqu1.min(rqu2);
        let squ1 = 5000.0 * 235.0 / 3.0_f64.sqrt();
        let squ2 = 4.0e8 / 6000.0;
        let squ = squ1.min(squ2);
        let hand = rqu + squ;
        assert!((qu - hand).abs() < 1e-3, "Qu={qu} vs {hand}");
    }

    #[test]
    fn test_src_beam_shear_alpha_clamped() {
        // M/(Q·rd) 大 → α<1 になるところを 1.0 にクランプ。
        let mut inp = base_input();
        inp.m_over_qrd = 10.0; // 4/11=0.36 → 1.0
        let qu_hi = src_beam_shear_ultimate_tech_standard(&inp);
        inp.m_over_qrd = 3.0; // 4/4=1.0
        let qu_at1 = src_beam_shear_ultimate_tech_standard(&inp);
        assert!((qu_hi - qu_at1).abs() < 1e-6);
    }

    #[test]
    fn test_src_beam_shear_steel_contribution_positive() {
        // 鉄骨ウェブ項が正の寄与（sAw を 0 にすると Qu が下がる）。
        let inp = base_input();
        let mut no_steel = base_input();
        no_steel.s_aw = 0.0;
        no_steel.sum_s_mu = 0.0;
        assert!(
            src_beam_shear_ultimate_tech_standard(&inp)
                > src_beam_shear_ultimate_tech_standard(&no_steel)
        );
    }

    #[test]
    fn test_src_beam_shear_invalid_inputs_zero() {
        let mut bad = base_input();
        bad.fc = 0.0;
        assert_eq!(src_beam_shear_ultimate_tech_standard(&bad), 0.0);
        assert_eq!(src_beam_shear_ultimate_src_standard(&bad), 0.0);
    }

    fn nonweb_input() -> SrcNonSolidWebShearInput {
        SrcNonSolidWebShearInput {
            fc: 24.0,
            be: 500.0,
            d_full: 700.0,
            d_eff: 630.0,
            rpt: 0.8,
            spt: 0.5,
            kcs: 0.8,
            rpw: 0.004,
            rw_sigma_y: 295.0,
            spw: 0.003,
            s_band_sigma_y: 235.0,
            m_over_qd: 2.0,
            high_strength_shear_rebar: false,
            rj: 560.0,
            lattice_area: 800.0,
            lattice_sigma_y: 235.0,
            lattice_angle: std::f64::consts::FRAC_PI_4,
            s_m0: 3.0e8,
            clear_span: 6000.0,
        }
    }

    #[test]
    fn test_src_beam_shear_grid_matches_handcalc() {
        let inp = nonweb_input();
        let qu = src_beam_shear_grid(&inp);
        let pt: f64 = 0.8 + 0.5;
        let ssr: f64 = 2.0_f64.clamp(1.0, 3.0);
        let j = 0.8 * 700.0;
        let concrete = 0.053 * pt.powf(0.23) * 0.8 * (18.0 + 24.0) / (ssr + 0.12);
        let hoop_r = 0.85 * (0.004_f64 * 295.0).sqrt();
        let hoop_s = 0.5 * (0.003_f64 * 235.0).sqrt();
        let hand = (concrete + hoop_r + hoop_s) * 500.0 * j;
        assert!((qu - hand).abs() < 1e-3, "grid Qu={qu} vs {hand}");
        assert!(qu > 0.0);
    }

    #[test]
    fn test_src_beam_shear_lattice_matches_handcalc() {
        let inp = nonweb_input();
        let qu = src_beam_shear_lattice(&inp);
        let ssr: f64 = 2.0_f64.clamp(1.0, 3.0);
        let concrete = 0.053 * 0.8_f64.powf(0.23) * 0.8 * (18.0 + 24.0) / (ssr + 0.12);
        let hoop_r = 0.85 * (0.004_f64 * 295.0).sqrt();
        let rc_part = (concrete + hoop_r) * 500.0 * 560.0;
        let squ_bending: f64 = 2.0 * 3.0e8 / 6000.0;
        let squ_tension: f64 = 800.0 * 235.0 * std::f64::consts::FRAC_PI_4.sin();
        let squ = squ_bending.min(squ_tension);
        let hand = rc_part + squ;
        assert!((qu - hand).abs() < 1e-3, "lattice Qu={qu} vs {hand}");
        assert!(qu > 0.0);
    }

    #[test]
    fn test_src_beam_shear_grid_band_plate_adds() {
        // 帯板項が正の寄与（spw=0 で Qsu が下がる）。
        let inp = nonweb_input();
        let mut no_band = nonweb_input();
        no_band.spw = 0.0;
        assert!(src_beam_shear_grid(&inp) > src_beam_shear_grid(&no_band));
    }

    #[test]
    fn test_src_beam_shear_nonweb_high_strength_uses_0068() {
        let mut hi = nonweb_input();
        hi.high_strength_shear_rebar = true;
        assert!(src_beam_shear_grid(&hi) > src_beam_shear_grid(&nonweb_input()));
        assert!(src_beam_shear_lattice(&hi) > src_beam_shear_lattice(&nonweb_input()));
    }

    #[test]
    fn test_src_beam_shear_nonweb_invalid_zero() {
        let mut bad = nonweb_input();
        bad.fc = 0.0;
        assert_eq!(src_beam_shear_grid(&bad), 0.0);
        assert_eq!(src_beam_shear_lattice(&bad), 0.0);
    }
}
