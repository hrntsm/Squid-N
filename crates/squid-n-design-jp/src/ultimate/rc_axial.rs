//! 鉄筋コンクリート造柱の**軸終局耐力（軸圧縮強度 Nuc・軸引張強度 Nut）**
//! （2007年版建築物の構造関係技術基準解説書 鉄筋コンクリート造柱の終局耐力）。
//!
//! 構造規定（2007年版建築物の構造関係技術基準解説書 P.626-627,629-630）に準じて
//! 算定する。本プログラムでは圧縮軸力を正、引張軸力を負と表す。

/// RC 柱の軸終局耐力（圧縮・引張）[N]。
#[derive(Clone, Copy, Debug)]
pub struct RcAxialUltimate {
    /// 軸圧縮強度 Nuc = b·D·Fc [N]（圧縮正）。
    pub nuc: f64,
    /// 軸引張強度 Nut = −ag·σy [N]（引張負）。
    pub nut: f64,
}

/// RC 柱の軸終局耐力 Nuc・Nut を算定する（技術基準解説書）。
///
/// ```text
/// Nuc = b·D·Fc          （軸圧縮強度、圧縮正）
/// Nut = −ag·σy          （軸引張強度、引張負）
/// ```
/// - `b`,`d`: 柱の幅・せい [mm]、`fc`: Fc [N/mm²]、`ag`: 全主筋断面積 [mm²]、
///   `sigma_y`: 主筋降伏強度 [N/mm²]。
/// - 不正入力（いずれかが負）は 0 にクランプして評価する。
pub fn rc_column_axial_ultimate(b: f64, d: f64, fc: f64, ag: f64, sigma_y: f64) -> RcAxialUltimate {
    let nuc = (b.max(0.0) * d.max(0.0) * fc.max(0.0)).max(0.0);
    let nut = -(ag.max(0.0) * sigma_y.max(0.0));
    RcAxialUltimate { nuc, nut }
}

/// 軸力に対する終局検定の余裕率。
///
/// 技術基準解説書の規定に従い、設計用柱軸力 `n_design`（**圧縮正**）に対し:
/// ```text
/// N ≥ 0（圧縮）: 余裕率 = Nuc / N
/// N < 0（引張）: 余裕率 = |Nut| / |N|
/// ```
/// を返す（余裕率 ≥ 1.0 で OK）。`n_design = 0` のときは軸力の余裕は無限大と
/// みなし `f64::INFINITY` を返す。
pub fn rc_axial_margin(axial: &RcAxialUltimate, n_design: f64) -> f64 {
    if n_design > 0.0 {
        if axial.nuc > 0.0 {
            axial.nuc / n_design
        } else {
            0.0
        }
    } else if n_design < 0.0 {
        let nut_abs = axial.nut.abs();
        if nut_abs > 0.0 {
            nut_abs / (-n_design)
        } else {
            0.0
        }
    } else {
        f64::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rc_column_axial_ultimate_matches_handcalc() {
        // b=600, D=600, Fc=24, ag=8-D25(=8·490.87), σy=345。
        let ag = 8.0 * std::f64::consts::PI / 4.0 * 25.0 * 25.0;
        let ax = rc_column_axial_ultimate(600.0, 600.0, 24.0, ag, 345.0);
        assert!((ax.nuc - 600.0 * 600.0 * 24.0).abs() < 1e-6);
        assert!((ax.nut - (-ag * 345.0)).abs() < 1e-3);
        // 圧縮強度は正、引張強度は負。
        assert!(ax.nuc > 0.0 && ax.nut < 0.0);
    }

    #[test]
    fn test_rc_axial_margin_compression_and_tension() {
        let ax = RcAxialUltimate {
            nuc: 8_640_000.0,
            nut: -1_000_000.0,
        };
        // 圧縮 N=+4,000,000 → 余裕率 = Nuc/N = 2.16。
        assert!((rc_axial_margin(&ax, 4_000_000.0) - 8_640_000.0 / 4_000_000.0).abs() < 1e-9);
        // 引張 N=−500,000 → 余裕率 = |Nut|/|N| = 2.0。
        assert!((rc_axial_margin(&ax, -500_000.0) - 2.0).abs() < 1e-9);
        // N=0 → 無限大。
        assert!(rc_axial_margin(&ax, 0.0).is_infinite());
    }
}
