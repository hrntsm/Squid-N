//! コンクリート充填鋼管（CFT）柱の**軸終局耐力**
//! （コンクリート充填鋼管構造設計指針（CFT 指針）に基づく軸終局耐力）。
//!
//! # 位置付け
//! [`crate::cft`] が許容応力度検定（SRC 規準準用）を扱うのに対し、本モジュールは
//! 終局検定（06 章）における CFT 柱の軸方向終局耐力（軸圧縮 Ncu・軸引張 Ntu）を
//! 「コンクリート充填鋼管構造設計指針（CFT 指針）」に基づき算定する純関数群である。
//! 曲げを伴う N-M 相互作用（短柱 cNu/sNu 等）は今後の課題とする。
//!
//! # 準拠する規準・出典（要・原典照合、`specs/原典照合リスト.md`）
//! - 日本建築学会「コンクリート充填鋼管構造設計指針」。角型 CFT は正方形のみを
//!   対象とするが、本実装は計算式を準用して長方形断面にも適用する。
//!
//! # 柱の分類（座屈長さ lk と断面せい D）
//! - `lk ≤ 4·D`: 短柱、`lk > 12·D`: 長柱、`4·D < lk ≤ 12·D`: 中柱。

/// CFT 柱の分類（座屈長さ lk と断面せい D による）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CftColumnClass {
    /// 短柱（lk ≤ 4·D）。
    Short,
    /// 中柱（4·D < lk ≤ 12·D）。
    Medium,
    /// 長柱（lk > 12·D）。
    Long,
}

/// 座屈長さ `lk` と断面せい `d` から柱を分類する（CFT 指針）。
/// `d ≤ 0` の不正入力は短柱扱いとする。
pub fn cft_column_class(lk: f64, d: f64) -> CftColumnClass {
    if d <= 0.0 {
        return CftColumnClass::Short;
    }
    let ratio = lk / d;
    if ratio <= 4.0 {
        CftColumnClass::Short
    } else if ratio <= 12.0 {
        CftColumnClass::Medium
    } else {
        CftColumnClass::Long
    }
}

/// 充填コンクリートの座屈応力度 `cσcr` [N/mm²]（CFT 指針）。
///
/// ```text
/// cσcr/Fc = { 2/(1 + √(cλ1⁴ + 1))          (cλ1 ≤ 1.0)
///             2(√2 − 1)·exp(Cc(1 − cλ1))    (cλ1 ≥ 1.0) }
/// ```
/// 両分岐は `cλ1 = 1.0` で連続する（いずれも `2/(1+√2)·Fc`）。
/// `Fc ≤ 0` の不正入力は 0 を返す。`Cc` は [`cft_cc`]。
pub fn cft_concrete_buckling_stress(fc: f64, c_lambda1: f64, cc: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    let lam = c_lambda1.max(0.0);
    let factor = if lam <= 1.0 {
        2.0 / (1.0 + (lam.powi(4) + 1.0).sqrt())
    } else {
        2.0 * (2.0_f64.sqrt() - 1.0) * (cc * (1.0 - lam)).exp()
    };
    (factor * fc).max(0.0)
}

/// 係数 `Cc = 0.568 + 0.00612·Fc`（CFT 指針）。
pub fn cft_cc(fc: f64) -> f64 {
    0.568 + 0.00612 * fc
}

/// 圧縮強度時ひずみ `εu = 0.93·Fc^(1/4)·10⁻³`（CFT 指針）。
/// `Fc ≤ 0` は 0。
pub fn cft_epsilon_u(fc: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    0.93 * fc.powf(0.25) * 1.0e-3
}

/// 充填コンクリートの規準化細長比 `cλ1 = cλ/π·√εu`（`cλ = lk/ci`, `ci = √(cI/cA)`）。
/// 不正入力（cA・cI・lk のいずれか 0 以下）は 0。
pub fn cft_concrete_slenderness(c_inertia: f64, c_area: f64, fc: f64, lk: f64) -> f64 {
    if c_area <= 0.0 || c_inertia <= 0.0 || lk <= 0.0 {
        return 0.0;
    }
    let ci = (c_inertia / c_area).sqrt();
    if ci <= 0.0 {
        return 0.0;
    }
    let c_lambda = lk / ci;
    c_lambda / std::f64::consts::PI * cft_epsilon_u(fc).sqrt()
}

/// 充填コンクリートの座屈軸耐力 `cNcr = cσcr·cA`（CFT 指針）。
/// 不正入力（cA・cI のいずれか 0 以下）は 0。`lk ≤ 0` は無座屈として `cA·Fc`。
pub fn cft_concrete_buckling_axial(c_inertia: f64, c_area: f64, fc: f64, lk: f64) -> f64 {
    if c_area <= 0.0 || c_inertia <= 0.0 || fc <= 0.0 {
        return 0.0;
    }
    if lk <= 0.0 {
        return c_area * fc;
    }
    let c_lambda1 = cft_concrete_slenderness(c_inertia, c_area, fc, lk);
    cft_concrete_buckling_stress(fc, c_lambda1, cft_cc(fc)) * c_area
}

/// CFT 柱の軸終局耐力算定の入力（CFT 指針）。
#[derive(Clone, Copy, Debug)]
pub struct CftAxialInput {
    /// 円形断面なら true（角型なら false）。ξ の判定に用いる。
    pub circular: bool,
    /// 断面せい D [mm]（柱分類 lk/D に用いる）。
    pub d_section: f64,
    /// 充填コンクリートの断面積 cA [mm²]。
    pub c_area: f64,
    /// 鋼管の断面積 sA [mm²]。
    pub s_area: f64,
    /// 充填コンクリートの断面二次モーメント cI [mm⁴]（座屈細長比 cλ 用、弱軸）。
    pub c_inertia: f64,
    /// 鋼管の断面二次モーメント sI [mm⁴]（座屈細長比 sλ・オイラー荷重用、弱軸）。
    pub s_inertia: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 鋼管の降伏強さ Fy [N/mm²]。
    pub fy: f64,
    /// 鋼管のヤング係数 sE [N/mm²]。
    pub s_young: f64,
    /// 座屈長さ lk [mm]。
    pub lk: f64,
}

/// CFT 柱の軸終局耐力の結果 [N]。
#[derive(Clone, Copy, Debug)]
pub struct CftAxialUltimate {
    /// 柱分類。
    pub class: CftColumnClass,
    /// 軸圧縮終局耐力 Ncu [N]（分類に応じて Ncu1/Ncu2/Ncu3）。
    pub ncu: f64,
    /// 軸引張終局耐力 Ntu [N]（引張正の絶対値。Ntu = sA·Fy）。
    pub ntu: f64,
}

/// 短柱の軸圧縮終局耐力 `Ncu1 = cNc + (1+ξ)·sNc`（CFT 指針）。
/// `ξ = 0.27`（円形）/ `0`（角型）、`cNc = cA·Fc`、`sNc = sA·Fy`。
pub fn cft_ncu1(inp: &CftAxialInput) -> f64 {
    let xi = if inp.circular { 0.27 } else { 0.0 };
    let cnc = inp.c_area.max(0.0) * inp.fc.max(0.0);
    let snc = inp.s_area.max(0.0) * inp.fy.max(0.0);
    (cnc + (1.0 + xi) * snc).max(0.0)
}

/// 長柱（または `lk` を明示指定した中柱補間用）の軸圧縮終局耐力
/// `Ncu3 = cNcr + sNcr`（CFT 指針）を、座屈長さ `lk` で算定する。
///
/// - `cNcr = cσcr·cA`、`cλ1 = cλ/π·√εu`、`cλ = lk/ci`、`ci = √(cI/cA)`。
/// - `sNcr = { sNy (sλ1<0.3); (1−0.545(sλ1−0.3))·sNy (0.3≤sλ1<1.3); sNE/1.3 (sλ1≥1.3) }`、
///   `sNy = sA·Fy`、`sλ1 = sλ/π·√(Fy/sE)`、`sλ = lk/si`、`si = √(sI/sA)`、
///   `sNE = π²·sE·sI/lk²`。
///
/// # 簡略化（doc 兼申し送り）
/// 抽出した原文では `sNcr` の分岐条件が `cλ1` と記載されるが、式本体が
/// `sλ1`・`sNy`・`sNE`（鋼管の座屈）で構成されるため、分岐条件も鋼管細長比
/// `sλ1` で評価する（「鋼構造塑性設計指針」の柱耐力式に整合。要・原典照合）。
fn cft_ncu3_at_lk(inp: &CftAxialInput, lk: f64) -> f64 {
    if lk <= 0.0 {
        // 座屈長さ 0 は短柱の累加耐力に一致（座屈なし）。
        return cft_ncu1(inp);
    }
    // 充填コンクリートの座屈耐力 cNcr。
    let c_ncr = cft_concrete_buckling_axial(inp.c_inertia, inp.c_area, inp.fc, lk);
    // 鋼管の座屈耐力 sNcr。
    let s_ncr = if inp.s_area > 0.0 && inp.s_inertia > 0.0 && inp.s_young > 0.0 {
        let si = (inp.s_inertia / inp.s_area).sqrt();
        let s_lambda = if si > 0.0 { lk / si } else { 0.0 };
        let s_lambda1 = s_lambda / std::f64::consts::PI * (inp.fy / inp.s_young).sqrt();
        let s_ny = inp.s_area * inp.fy;
        let s_ne = std::f64::consts::PI.powi(2) * inp.s_young * inp.s_inertia / (lk * lk);
        if s_lambda1 < 0.3 {
            s_ny
        } else if s_lambda1 < 1.3 {
            (1.0 - 0.545 * (s_lambda1 - 0.3)) * s_ny
        } else {
            s_ne / 1.3
        }
    } else {
        0.0
    };
    (c_ncr + s_ncr).max(0.0)
}

/// CFT 柱の軸終局耐力（圧縮 Ncu・引張 Ntu）を算定する（CFT 指針）。
///
/// ```text
/// 短柱: Ncu = Ncu1 = cNc + (1+ξ)·sNc
/// 長柱: Ncu = Ncu3 = cNcr + sNcr
/// 中柱: Ncu = Ncu2 = Ncu1 − 0.125·(Ncu1 − Ncu3|lk/D=12)·(lk/D − 4)
/// 引張: Ntu = sNt = sA·Fy
/// ```
/// 中柱の `Ncu3` は `lk/D = 12` として算定した値を用いる（CFT 指針）。
///
/// # 簡略化（doc 兼申し送り）
/// 軸引張終局耐力 `Ntu = sA·β2·Fy` の `β2`（引張時の低減係数）は抽出した原文で
/// 定義が不明瞭なため、`β2 = 1.0`（鋼管全断面降伏）として扱う（要・原典照合）。
pub fn cft_axial_ultimate(inp: &CftAxialInput) -> CftAxialUltimate {
    let class = cft_column_class(inp.lk, inp.d_section);
    let ncu = match class {
        CftColumnClass::Short => cft_ncu1(inp),
        CftColumnClass::Long => cft_ncu3_at_lk(inp, inp.lk),
        CftColumnClass::Medium => {
            let ncu1 = cft_ncu1(inp);
            // lk/D=12 として算定した Ncu3。
            let ncu3_at_12 = cft_ncu3_at_lk(inp, 12.0 * inp.d_section);
            let ld = if inp.d_section > 0.0 {
                inp.lk / inp.d_section
            } else {
                4.0
            };
            (ncu1 - 0.125 * (ncu1 - ncu3_at_12) * (ld - 4.0)).max(0.0)
        }
    };
    let ntu = (inp.s_area.max(0.0) * inp.fy.max(0.0)).max(0.0);
    CftAxialUltimate { class, ncu, ntu }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cft_column_class() {
        assert_eq!(cft_column_class(1000.0, 400.0), CftColumnClass::Short); // 2.5D
        assert_eq!(cft_column_class(2400.0, 400.0), CftColumnClass::Medium); // 6D
        assert_eq!(cft_column_class(5000.0, 400.0), CftColumnClass::Long); // 12.5D
                                                                           // 境界: 4D=短柱、12D=中柱。
        assert_eq!(cft_column_class(1600.0, 400.0), CftColumnClass::Short);
        assert_eq!(cft_column_class(4800.0, 400.0), CftColumnClass::Medium);
    }

    #[test]
    fn test_cft_concrete_buckling_stress_continuous_at_1() {
        // cλ1=1 で両分岐が連続（2/(1+√2)·Fc）。
        let fc = 30.0;
        let cc = cft_cc(fc);
        let lo = cft_concrete_buckling_stress(fc, 0.999999, cc);
        let hi = cft_concrete_buckling_stress(fc, 1.000001, cc);
        assert!((lo - hi).abs() < 1e-4, "lo={lo} hi={hi}");
        let expected = 2.0 / (1.0 + 2.0_f64.sqrt()) * fc;
        assert!((cft_concrete_buckling_stress(fc, 1.0, cc) - expected).abs() < 1e-6);
        // cλ1=0 → 2/(1+1)=1 倍 → cσcr=Fc。
        assert!((cft_concrete_buckling_stress(fc, 0.0, cc) - fc).abs() < 1e-9);
    }

    /// 角型 CFT-□400×400×12, Fc=30, Fy=325（BCR295 相当）, sE=205000。
    fn box_input(lk: f64) -> CftAxialInput {
        let (h, w, t) = (400.0_f64, 400.0_f64, 12.0_f64);
        let ch = h - 2.0 * t;
        let cw = w - 2.0 * t;
        let s_area = h * w - ch * cw;
        let c_area = ch * cw;
        let s_inertia = w * h.powi(3) / 12.0 - cw * ch.powi(3) / 12.0;
        let c_inertia = cw * ch.powi(3) / 12.0;
        CftAxialInput {
            circular: false,
            d_section: 400.0,
            c_area,
            s_area,
            c_inertia,
            s_inertia,
            fc: 30.0,
            fy: 325.0,
            s_young: 205000.0,
            lk,
        }
    }

    #[test]
    fn test_cft_ncu1_short_column_matches_handcalc() {
        let inp = box_input(1000.0); // 2.5D → 短柱
        let r = cft_axial_ultimate(&inp);
        assert_eq!(r.class, CftColumnClass::Short);
        // 角型 ξ=0: Ncu1 = cA·Fc + sA·Fy。
        let hand = inp.c_area * 30.0 + inp.s_area * 325.0;
        assert!((r.ncu - hand).abs() < 1e-3, "Ncu={} vs {}", r.ncu, hand);
        // Ntu = sA·Fy。
        assert!((r.ntu - inp.s_area * 325.0).abs() < 1e-3);
    }

    #[test]
    fn test_cft_circular_xi_increases_ncu1() {
        let mut inp = box_input(1000.0);
        let box_ncu = cft_ncu1(&inp);
        inp.circular = true;
        let circ_ncu = cft_ncu1(&inp);
        // 円形は ξ=0.27 の拘束効果で sNc 分だけ大きい。
        assert!(circ_ncu > box_ncu);
        assert!((circ_ncu - (box_ncu + 0.27 * inp.s_area * 325.0)).abs() < 1e-3);
    }

    #[test]
    fn test_cft_long_column_less_than_short() {
        let short = cft_axial_ultimate(&box_input(1000.0));
        let long = cft_axial_ultimate(&box_input(6000.0)); // 15D → 長柱
        assert_eq!(long.class, CftColumnClass::Long);
        // 座屈により長柱の圧縮耐力は短柱の累加耐力より小さい。
        assert!(
            long.ncu < short.ncu,
            "long={} short={}",
            long.ncu,
            short.ncu
        );
        assert!(long.ncu > 0.0);
    }

    #[test]
    fn test_cft_medium_column_between_short_and_long() {
        let short = cft_axial_ultimate(&box_input(1000.0)).ncu;
        let medium = cft_axial_ultimate(&box_input(2400.0)); // 6D → 中柱
        assert_eq!(medium.class, CftColumnClass::Medium);
        // 中柱は短柱と（lk/D=12 の）長柱の間を線形補間するため短柱以下。
        assert!(medium.ncu <= short && medium.ncu > 0.0);
    }
}
