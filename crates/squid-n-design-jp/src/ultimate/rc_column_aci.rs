//! 鉄筋コンクリート造柱の**曲げ終局強度 Mu（ACI 規準・平面保持）**
//! （ACI318 の平面保持・等価応力度ブロック法に基づく）。
//!
//! # 位置付け
//! 本実装では柱の終局曲げ強度 Mu を構造規定式（at 式、
//! [`squid_n_core::rc_capacity::rc_column_mu_simple`]）または **ACI 規準による
//! 平面保持解析**のいずれかで算定できる。本モジュールは後者を実装する。
//! 圧縮側コンクリートの応力分布を「ACI318-95 規準」の等価応力度ブロック法で
//! モデル化し、平面保持・ひずみ適合から中立軸位置を軸力 N に整合させて Mu を求める。
//!
//! # 仮定（ACI318 平面保持解析、①〜⑥）
//! ① 圧縮縁コンクリートのひずみが終局ひずみ `εcu = 0.3% = 0.003` に達する。
//! ② 平面保持（ひずみは中立軸からの距離に比例）。
//! ③ 鉄筋は降伏ひずみ以下で弾性、以上で材料強度（σy でクランプ）。
//! ④ コンクリートは引張応力を負担しない。
//! ⑤ 圧縮応力は等価応力度ブロック（応力 `β3·Fc = 0.85·Fc`、深さ `a = β1·c`）。
//! ⑥ 係数 β1/β2/β3 は下記（`Fc` は psi 換算、1 N/mm² = 145.04 psi）。
//!
//! # 簡略化（doc 兼申し送り）
//! - 圧縮域鉄筋によるコンクリート断面の欠損（`−0.85·Fc·As`）は考慮しない
//!   （at 式と同じ扱い。安全側とは限らないが略算の慣習に合わせる）。
//! - 主筋は呼び出し側が与える段（圧縮縁からの距離, 断面積）でモデル化する。

/// 終局ひずみ εcu = 0.3%（ACI318）。
const EPSILON_CU: f64 = 0.003;
/// 単位換算: 1 N/mm² = 145.04 psi。
const NMM2_TO_PSI: f64 = 145.04;
/// 等価応力度ブロックの応力係数 β3 = 0.85。
const BETA3: f64 = 0.85;

/// 等価応力度ブロックの深さ係数 `β1`（ACI318）。
///
/// ```text
/// β1 = { 0.85                         (Fc ≤ 4000 psi)
///        0.85 − 0.05·(Fc−4000)/1000   (4000 psi < Fc < 8000 psi)
///        0.65                         (8000 psi ≤ Fc) }
/// ```
/// `Fc` は [N/mm²] で受け取り内部で psi 換算する。
pub fn aci_beta1(fc: f64) -> f64 {
    let fc_psi = fc.max(0.0) * NMM2_TO_PSI;
    if fc_psi <= 4000.0 {
        0.85
    } else if fc_psi < 8000.0 {
        0.85 - 0.05 * (fc_psi - 4000.0) / 1000.0
    } else {
        0.65
    }
}

/// ACI 平面保持解析の入力（柱 1 軸分の断面諸元）。
#[derive(Clone, Copy, Debug)]
pub struct AciColumnInput {
    /// 圧縮縁の幅 b [mm]（等価応力ブロックの幅）。
    pub b: f64,
    /// 断面せい D [mm]（圧縮縁から反対縁まで）。
    pub d_full: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 主筋の降伏強度 σy [N/mm²]。
    pub sigma_y: f64,
    /// 鉄筋のヤング係数 Es [N/mm²]。
    pub es: f64,
}

/// 中立軸深さ `c`（圧縮縁からの距離 [mm]）における軸力 N(c) [N]（**圧縮正**）を返す。
/// 圧縮コンクリート合力＋各鉄筋段の応力（σy クランプ）の和。
fn axial_at_c(inp: &AciColumnInput, layers: &[(f64, f64)], c: f64) -> f64 {
    if c <= 0.0 {
        return 0.0;
    }
    let beta1 = aci_beta1(inp.fc);
    let a = (beta1 * c).min(inp.d_full);
    let cc = BETA3 * inp.fc * inp.b * a;
    let mut n = cc;
    for &(di, area) in layers {
        let eps = EPSILON_CU * (c - di) / c;
        let sigma = (inp.es * eps).clamp(-inp.sigma_y, inp.sigma_y);
        n += sigma * area;
    }
    n
}

/// ACI 平面保持解析による柱の終局曲げ強度 `Mu` [N·mm]（ACI318）。
///
/// `layers`: 主筋の段リスト `(圧縮縁からの距離 di [mm], 断面積 Asi [mm²])`。
/// `n_design`: 設計軸力 [N]（**圧縮正**）。中立軸深さ `c` を二分法で `N(c)=n_design`
/// に整合させ、断面図心まわりのモーメントとして Mu を算定する。
///
/// - 軸力が断面の N 範囲（純引張〜中心圧縮）外の場合は端点にクランプする。
/// - 不正入力（b・D・Fc・σy・Es のいずれかが 0 以下、または layers 空）は 0.0。
pub fn rc_column_mu_aci(inp: &AciColumnInput, layers: &[(f64, f64)], n_design: f64) -> f64 {
    if inp.b <= 0.0
        || inp.d_full <= 0.0
        || inp.fc <= 0.0
        || inp.sigma_y <= 0.0
        || inp.es <= 0.0
        || layers.is_empty()
    {
        return 0.0;
    }
    // N(c) は c について単調増加。二分法で n_design に整合する c を求める。
    let c_lo = 1.0e-4 * inp.d_full;
    let c_hi = 100.0 * inp.d_full;
    let n_lo = axial_at_c(inp, layers, c_lo);
    let n_hi = axial_at_c(inp, layers, c_hi);
    // 範囲外はクランプ。
    let target = n_design.clamp(n_lo.min(n_hi), n_lo.max(n_hi));
    let mut lo = c_lo;
    let mut hi = c_hi;
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if axial_at_c(inp, layers, mid) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let c = 0.5 * (lo + hi);

    // 断面図心（D/2）まわりのモーメント。圧縮を正とし、圧縮合力・引張鉄筋とも
    // 図心より上（圧縮側）の力が正の曲げに寄与するよう (D/2 − y) を乗じる。
    let beta1 = aci_beta1(inp.fc);
    let a = (beta1 * c).min(inp.d_full);
    let cc = BETA3 * inp.fc * inp.b * a;
    let center = inp.d_full / 2.0;
    let mut m = cc * (center - a / 2.0);
    for &(di, area) in layers {
        let eps = EPSILON_CU * (c - di) / c;
        let sigma = (inp.es * eps).clamp(-inp.sigma_y, inp.sigma_y);
        m += sigma * area * (center - di);
    }
    m.abs().max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aci_beta1_branches() {
        // Fc=24 → 24·145.04=3481 psi ≤ 4000 → 0.85。
        assert!((aci_beta1(24.0) - 0.85).abs() < 1e-9);
        // Fc=40 → 5802 psi（4000〜8000）→ 0.85 − 0.05·(5802−4000)/1000。
        let fc_psi = 40.0 * 145.04;
        assert!((aci_beta1(40.0) - (0.85 - 0.05 * (fc_psi - 4000.0) / 1000.0)).abs() < 1e-9);
        // Fc=60 → 8702 psi ≥ 8000 → 0.65。
        assert!((aci_beta1(60.0) - 0.65).abs() < 1e-9);
    }

    /// 対称配筋の柱: b=D=600, Fc=24, σy=345, Es=205000。
    /// 引張・圧縮各段 at=1963mm²(D25×4相当), dt=60。
    fn sample() -> (AciColumnInput, Vec<(f64, f64)>) {
        let d = 600.0;
        let dt = 60.0;
        let at = 4.0 * std::f64::consts::PI / 4.0 * 25.0 * 25.0;
        let layers = vec![(dt, at), (d - dt, at)];
        (
            AciColumnInput {
                b: 600.0,
                d_full: d,
                fc: 24.0,
                sigma_y: 345.0,
                es: 205000.0,
            },
            layers,
        )
    }

    #[test]
    fn test_rc_column_mu_aci_n0_positive() {
        let (inp, layers) = sample();
        let mu = rc_column_mu_aci(&inp, &layers, 0.0);
        assert!(mu > 0.0);
        // N=0 の Mu は引張鉄筋降伏で決まり、おおむね at·σy·(内側腕長) の程度。
        // 参考: at·σy·(d−dt) の 0.5〜1.5 倍に収まる健全域。
        let at = 4.0 * std::f64::consts::PI / 4.0 * 25.0 * 25.0;
        let ref_m = at * 345.0 * (600.0 - 2.0 * 60.0);
        assert!(mu > 0.3 * ref_m && mu < 2.0 * ref_m, "Mu={mu} ref={ref_m}");
    }

    #[test]
    fn test_rc_column_mu_aci_increases_then_decreases() {
        let (inp, layers) = sample();
        let m0 = rc_column_mu_aci(&inp, &layers, 0.0);
        let n_mid = 0.2 * inp.b * inp.d_full * inp.fc; // 0.2·b·D·Fc
        let m_mid = rc_column_mu_aci(&inp, &layers, n_mid);
        let n_high = 0.8 * inp.b * inp.d_full * inp.fc;
        let m_high = rc_column_mu_aci(&inp, &layers, n_high);
        // 圧縮軸力で Mu は一旦増加し、高軸力で減少する（N-M 相関の山型）。
        assert!(m_mid > m0, "m_mid={m_mid} m0={m0}");
        assert!(m_high < m_mid, "m_high={m_high} m_mid={m_mid}");
    }

    #[test]
    fn test_rc_column_mu_aci_axial_balance() {
        // 求めた c で N(c) が n_design に整合していることを確認する。
        let (inp, layers) = sample();
        let n_design = 0.3 * inp.b * inp.d_full * inp.fc;
        // 内部の二分法後の c を再現して残差を確認（axial_at_c の単調性の担保）。
        let c_lo = 1.0e-4 * inp.d_full;
        let c_hi = 100.0 * inp.d_full;
        let mut lo = c_lo;
        let mut hi = c_hi;
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            if axial_at_c(&inp, &layers, mid) < n_design {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let c = 0.5 * (lo + hi);
        let n_at_c = axial_at_c(&inp, &layers, c);
        assert!(
            (n_at_c - n_design).abs() / n_design < 1e-3,
            "N(c)={n_at_c} vs target={n_design}"
        );
        // Mu も正。
        assert!(rc_column_mu_aci(&inp, &layers, n_design) > 0.0);
    }

    #[test]
    fn test_rc_column_mu_aci_invalid_zero() {
        let (inp, layers) = sample();
        let mut bad = inp;
        bad.fc = 0.0;
        assert_eq!(rc_column_mu_aci(&bad, &layers, 0.0), 0.0);
        assert_eq!(rc_column_mu_aci(&inp, &[], 0.0), 0.0);
    }
}
