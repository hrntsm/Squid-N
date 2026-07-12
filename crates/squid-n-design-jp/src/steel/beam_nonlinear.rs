//! 鉄骨造梁の**非線形復元力特性**（RESP-D マニュアル「計算編 05 非線形モデル」
//! 鉄骨梁の曲げ・軸復元力特性）。
//!
//! # 位置付け
//! [`super::beam`] が許容応力度検定（横座屈を考慮した fb）を扱うのに対し、本モジュールは
//! 非線形解析の材端バネの骨格諸元（バイリニア）を算定する純関数群である。
//! 曲げは全塑性 Mp と横座屈 Mcr、軸は降伏軸力 Nu。
//!
//! # 準拠する規準・出典
//! - 全塑性モーメント Mp=Zp·σy。
//! - 横座屈モーメント Mcr（鋼構造塑性設計指針、H 形鋼のみ、フランジ材料）:
//!   Mcr/Mp を lb·H/Af により低減する（SN400/SN490/その他の 3 系）。
//! - 軸降伏 Nu=Af·σfy+Aw·σwy。

use squid_n_core::section_shape::SectionShape;

/// 全塑性モーメント Mp [N·mm] = Zp·σy。不正入力は 0.0。
pub fn steel_beam_full_plastic_moment(zp: f64, sigma_y: f64) -> f64 {
    zp.max(0.0) * sigma_y.max(0.0)
}

/// 横座屈モーメント比 Mcr/Mp（無次元、H 形鋼、鋼構造塑性設計指針）。
///
/// `x = lb·H/Af`（lb: 横補剛材間隔、H: 梁せい、Af: 圧縮フランジ断面積）に対し、
/// フランジ材料の降伏強度 σy 別に低減する:
/// - SN400/SS400/SM400（σy=235）: しきい値 300/835、傾き 0.00075、末尾 500/x
/// - SN490/SM490（σy=325）: しきい値 220/605、傾き 0.0010、末尾 363/x
/// - その他: e1=70500/σy, e2=117000/(0.6σy), 傾き=0.4/(e2−e1), 末尾=(117000/σy)/x
///
/// `af <= 0` は 1.0（低減なし）を返す。戻り値は [末尾, 1.0] に収まる。
pub fn steel_beam_lateral_buckling_ratio(lb: f64, h: f64, af: f64, sigma_y: f64) -> f64 {
    if af <= 0.0 || h <= 0.0 {
        return 1.0;
    }
    let x = lb.max(0.0) * h / af;
    // (e1, e2, slope, tail_coeff): 中間域 1−slope·(x−e1)、末尾域 tail_coeff/x。
    let (e1, e2, slope, tail_coeff) = if (sigma_y - 235.0).abs() < 1.0 {
        (300.0, 835.0, 0.00075, 500.0)
    } else if (sigma_y - 325.0).abs() < 1.0 {
        (220.0, 605.0, 0.0010, 363.0)
    } else {
        let e1 = 70500.0 / sigma_y;
        let e2 = 117000.0 / (0.6 * sigma_y);
        let slope = if e2 > e1 { 0.4 / (e2 - e1) } else { 0.0 };
        (e1, e2, slope, 117000.0 / sigma_y)
    };
    if x <= e1 {
        1.0
    } else if x <= e2 {
        (1.0 - slope * (x - e1)).max(0.0)
    } else {
        tail_coeff / x
    }
}

/// 横座屈を考慮した鉄骨梁の曲げ耐力 Mcr [N·mm] = (Mcr/Mp)·Mp。
pub fn steel_beam_lateral_buckling_moment(zp: f64, sigma_y: f64, lb: f64, h: f64, af: f64) -> f64 {
    let mp = steel_beam_full_plastic_moment(zp, sigma_y);
    steel_beam_lateral_buckling_ratio(lb, h, af, sigma_y) * mp
}

/// 鉄骨梁の軸降伏耐力 Nu [N] = Af·σfy + Aw·σwy（引張・圧縮共通）。
/// `af_total`: 全フランジ断面積、`aw`: ウェブ断面積。
pub fn steel_beam_axial_yield(af_total: f64, sigma_fy: f64, aw: f64, sigma_wy: f64) -> f64 {
    af_total.max(0.0) * sigma_fy.max(0.0) + aw.max(0.0) * sigma_wy.max(0.0)
}

/// H 形鋼断面の非線形諸元（強軸曲げ）。
#[derive(Clone, Copy, Debug)]
pub struct SteelHProps {
    /// 塑性断面係数 Zp（強軸）[mm³]。
    pub zp: f64,
    /// 梁せい H [mm]。
    pub h: f64,
    /// 圧縮フランジ断面積 Af=b·tf [mm²]（Mcr 用）。
    pub af_compression: f64,
    /// 全フランジ断面積 2·b·tf [mm²]（Nu 用）。
    pub af_total: f64,
    /// ウェブ断面積 (H−2tf)·tw [mm²]（Nu 用）。
    pub aw: f64,
}

/// `SectionShape::SteelH` から非線形諸元を取り出す（H 形以外は None）。
pub fn steel_h_props(shape: &SectionShape) -> Option<SteelHProps> {
    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let zp = width * flange_thick * (height - flange_thick)
                + web_thick * (height - 2.0 * flange_thick).powi(2) / 4.0;
            Some(SteelHProps {
                zp,
                h: height,
                af_compression: width * flange_thick,
                af_total: 2.0 * width * flange_thick,
                aw: (height - 2.0 * flange_thick).max(0.0) * web_thick,
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// H-500x200x10x16 相当。
    fn h_shape() -> SectionShape {
        SectionShape::SteelH {
            height: 500.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 16.0,
        }
    }

    #[test]
    fn test_steel_h_props_and_mp() {
        let p = steel_h_props(&h_shape()).unwrap();
        // Zp = 200·16·(500−16) + 10·(500−32)²/4
        let zp = 200.0 * 16.0 * (500.0 - 16.0) + 10.0 * (500.0 - 32.0_f64).powi(2) / 4.0;
        assert!((p.zp - zp).abs() < 1e-6);
        let mp = steel_beam_full_plastic_moment(p.zp, 235.0);
        assert!((mp - zp * 235.0).abs() < 1e-3);
        // 全フランジ・ウェブ断面積。
        assert!((p.af_total - 2.0 * 200.0 * 16.0).abs() < 1e-9);
        assert!((p.aw - (500.0 - 32.0) * 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_lateral_buckling_ratio_sn400_branches() {
        let h = 500.0;
        let af = 200.0 * 16.0; // 3200
                               // x = lb·H/Af。x≤300 で 1.0。lb·500/3200≤300 → lb≤1920。
        assert!((steel_beam_lateral_buckling_ratio(1000.0, h, af, 235.0) - 1.0).abs() < 1e-12);
        // 中間域 (300<x≤835): lb=3000 → x=3000·500/3200=468.75、1−0.00075·(468.75−300)。
        let x = 3000.0 * 500.0 / 3200.0;
        let expect = 1.0 - 0.00075 * (x - 300.0);
        assert!((steel_beam_lateral_buckling_ratio(3000.0, h, af, 235.0) - expect).abs() < 1e-9);
        assert!(expect < 1.0 && expect > 0.6);
        // 末尾域 (x>835): lb=6000 → x=937.5>835、500/x。
        let x2 = 6000.0 * 500.0 / 3200.0;
        assert!(
            (steel_beam_lateral_buckling_ratio(6000.0, h, af, 235.0) - 500.0 / x2).abs() < 1e-9
        );
    }

    #[test]
    fn test_lateral_buckling_ratio_continuous_at_e2() {
        // 一般式（σy=355）で e2 前後が連続。
        let sy = 355.0;
        let e2 = 117000.0 / (0.6 * sy);
        let af = 3200.0;
        let h = 500.0;
        // x=e2 となる lb を逆算: lb = e2·af/h。
        let lb = e2 * af / h;
        let below = steel_beam_lateral_buckling_ratio(lb * 0.9999, h, af, sy);
        let above = steel_beam_lateral_buckling_ratio(lb * 1.0001, h, af, sy);
        assert!((below - above).abs() < 1e-3, "不連続: {below} vs {above}");
        // e2 での値は両分岐とも 0.6（1−0.4）。
        assert!((below - 0.6).abs() < 2e-3 && (above - 0.6).abs() < 2e-3);
    }

    #[test]
    fn test_lateral_buckling_reduces_moment() {
        let p = steel_h_props(&h_shape()).unwrap();
        let mp = steel_beam_full_plastic_moment(p.zp, 235.0);
        let mcr_short =
            steel_beam_lateral_buckling_moment(p.zp, 235.0, 1000.0, p.h, p.af_compression);
        let mcr_long =
            steel_beam_lateral_buckling_moment(p.zp, 235.0, 6000.0, p.h, p.af_compression);
        assert!((mcr_short - mp).abs() < 1e-3, "短スパンは Mp と一致");
        assert!(mcr_long < mp, "長スパンは横座屈で低減");
    }

    #[test]
    fn test_steel_beam_axial_yield() {
        let p = steel_h_props(&h_shape()).unwrap();
        let nu = steel_beam_axial_yield(p.af_total, 235.0, p.aw, 235.0);
        assert!((nu - (p.af_total * 235.0 + p.aw * 235.0)).abs() < 1e-3);
        assert!(nu > 0.0);
    }
}
