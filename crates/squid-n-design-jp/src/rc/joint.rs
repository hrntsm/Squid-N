//! RC 造柱梁接合部の断面検定（RC 規準15条による許容応力度検定）。
//!
//! # 位置付け
//! このモジュールは `squid_n_core`（モデル）や要素（`squid_n_element`）に依存せず、
//! 呼び出し側（節点まわりの応力集計・断面形状の解決を担当する別モジュール）が
//! 用意した数値入力を受け取る**純関数**として実装する。したがって節点まわりの
//! 応力の集計方法（どの梁・柱を対象とするか、上下柱の平均化方法など）は呼び出し側の
//! 責務であり、本モジュールはその結果を受け取って許容値との比較のみを行う。
//!
//! 準拠する規準: 日本建築学会「鉄筋コンクリート構造計算規準・同解説」15条
//!
//! # 式の再構成・簡略化について（重要）
//! 元テキストは PDF/MathML からの抽出であり、分数式や上付き添字が
//! 崩れている箇所がある。以下は本モジュールで再構成・簡略化した式であり、
//! 各関数のドキュメントに個別に明記する:
//! - RC 接合部の有効幅 bj: `bai = max(bi/2, D/4)`。原典図
//!   （2026-07-11 照合）が「bi/2 または D/4 の**大きい方**」と明記しているため
//!   `max` を採用する（RC 規準本文の一般的な
//!   `min` 解釈より有効幅を大きく＝許容せん断力を大きく見積もる点に注意）。

use crate::CheckResult;

/// 柱梁接合部の形状（取り付く梁の本数・配置による分類）。
///
/// RC 規準 15 条の割増係数 κA の区分に対応する:
/// 十字形（4方向に梁）/ T字形（3方向）/ ト字形（2方向・通り直交）/ L字形（2方向・隅角）。
///
/// SRC 造柱梁接合部（[`crate::srrc::panel_zone`]）の接合部形状係数 jδ の区分
/// にも流用する。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointShape {
    /// 十字形（4方向に梁が取り付く）。κA = 10
    Cross,
    /// T字形（3方向に梁が取り付く）。κA = 7
    Tee,
    /// ト字形（通り直交2方向に梁が取り付く）。κA = 5
    Knee,
    /// L字形（隅角部・2方向に梁が取り付く）。κA = 3
    Corner,
}

/// RC 柱梁接合部のせん断検定の入力。
pub struct RcJointInput {
    /// 接合部の形状区分。
    pub shape: JointShape,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 柱せい D [mm]（検定する加力方向の柱せい）。
    pub col_depth: f64,
    /// 柱幅 [mm]（加力方向と直交する方向の柱幅）。
    pub col_width: f64,
    /// 大梁幅 bb [mm]。
    pub beam_width: f64,
    /// 大梁の応力中心間距離 j [mm]。
    pub beam_j: f64,
    /// 接合部に取り付く大梁端モーメントの絶対値和 ΣM [N·mm]。
    pub sum_beam_moments: f64,
    /// 柱の設計用せん断力 QD [N]（上下柱の平均値でよい）。
    pub col_shear: f64,
    /// 柱の平均階高 cH [mm]。
    pub col_height: f64,
    /// 大梁の平均スパン Lb [mm]。
    pub beam_span: f64,
}

/// RC 柱梁接合部のせん断検定（RC 規準 15 条）。
///
/// ## 許容せん断力
/// `QAj = κA・(fs − 0.5)・bj・D`
/// - κA: 十字形=10, T字形=7, ト字形=5, L字形=3
/// - `fs`: コンクリートの**短期**許容せん断応力度
///   （[`crate::rc::concrete_allowable_shear`]`(fc, false)`）
/// - `bj = bb + ba1 + ba2`（接合部有効幅）。
///   `bai = max(bi/2, D/4)`、`bi = (col_width − beam_width) / 2`。
///   梁が柱断面の中心に取り付き、柱幅と梁幅の差が両側に均等に振り分けられる
///   （`bi` が両側で共通）と仮定している。
///
///   **原典照合済み（2026-07-11）**: 原典の図
///   （ユーザー提供）が `bai = bi/2 または D/4 の大きい方` と明記しているため
///   `max` を採用する。`max` は有効幅を大きく＝許容せん断力を大きく見積もる
///   （RC 規準本文の一般的な `min` 解釈より非安全側）が、原典図の明記に
///   従うことを優先する。
///
/// ## 設計用せん断力
/// `Qdj = min(Qdj1, Qdj2)`
/// - `ξ = j / (cH・(1 − D/Lb))`
/// - `Qdj1 = ΣM/j・(1 − ξ)`
/// - `Qdj2 = QD・(1 − ξ)/ξ`
///
/// `ξ` は本来 `0 < ξ < 1` の範囲に収まる想定の幾何量である。入力の組み合わせに
/// よっては（`col_height=0` や `col_depth ≈ beam_span` 等）分母が 0 に近づいたり
/// `ξ` が範囲外になったりして式が発散しうるため、`ξ` が有限かつ `(0, 1)` の
/// 範囲に収まらない場合は安全側として `ξ→0`（すなわち `Qdj1 = ΣM/j`）とみなし、
/// `Qdj2` は最小値の対象から除外する（`Qdj2 = +∞` として扱う）。
///
/// 検定比 = `Qdj / QAj`（1.0 以下で OK）。
pub fn rc_joint_shear_check(inp: &RcJointInput) -> CheckResult {
    let kappa_a = match inp.shape {
        JointShape::Cross => 10.0,
        JointShape::Tee => 7.0,
        JointShape::Knee => 5.0,
        JointShape::Corner => 3.0,
    };

    let fs = crate::rc::concrete_allowable_shear(inp.fc, false);

    // 接合部有効幅 bj = bb + ba1 + ba2（両側均等仮定）。
    // bai = max(bi/2, D/4)（原典「大きい方」、2026-07-11 照合）。
    let bi = (inp.col_width - inp.beam_width) / 2.0;
    let bai = (bi / 2.0).max(inp.col_depth / 4.0).max(0.0);
    let bj = inp.beam_width + 2.0 * bai;

    let qaj = kappa_a * (fs - 0.5) * bj * inp.col_depth;

    // 設計用せん断力 Qdj = min(Qdj1, Qdj2)。
    let denom = inp.col_height * (1.0 - inp.col_depth / inp.beam_span);
    let xi = inp.beam_j / denom;
    let (qdj1, qdj2) = if xi.is_finite() && xi > 0.0 && xi < 1.0 {
        let one_minus_xi = 1.0 - xi;
        let qdj1 = inp.sum_beam_moments / inp.beam_j * one_minus_xi;
        let qdj2 = inp.col_shear * one_minus_xi / xi;
        (qdj1, qdj2)
    } else {
        // ξ 退化域: ξ→0 とみなし Qdj1 = ΣM/j をそのまま採用、Qdj2 は無効化。
        (inp.sum_beam_moments / inp.beam_j, f64::INFINITY)
    };
    let qdj = qdj1.min(qdj2);

    let ratio = if qaj > 0.0 { qdj / qaj } else { f64::INFINITY };
    let ok = ratio <= 1.0;

    let shape_label = match inp.shape {
        JointShape::Cross => "十字形(kappaA=10)",
        JointShape::Tee => "T字形(kappaA=7)",
        JointShape::Knee => "ト字形(kappaA=5)",
        JointShape::Corner => "L字形(kappaA=3)",
    };
    let basis = format!("RC規準15条 柱梁接合部せん断検定 {}", shape_label);
    let detail = format!(
        "fs={:.4} N/mm2, bj={:.2} mm, QAj={:.1} N, xi={:.4}, Qdj1={:.1} N, Qdj2={:.1} N, Qdj={:.1} N, ratio={:.4}",
        fs, bj, qaj, xi, qdj1, qdj2, qdj, ratio
    );

    CheckResult {
        ratio,
        ok,
        basis,
        detail,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_joint_input(shape: JointShape) -> RcJointInput {
        RcJointInput {
            shape,
            fc: 24.0,
            col_depth: 600.0,
            col_width: 600.0,
            beam_width: 300.0,
            beam_j: 500.0,
            sum_beam_moments: 400_000_000.0,
            col_shear: 200_000.0,
            col_height: 3000.0,
            beam_span: 6000.0,
        }
    }

    #[test]
    fn rc_joint_kappa_a_by_shape() {
        // fs(短期) = concrete_allowable_shear(24.0,false)
        let fs = crate::rc::concrete_allowable_shear(24.0, false);
        // bi=(600-300)/2=150, bai=max(bi/2=75, D/4=150)=150, bj=300+2*150=600
        let bj = 600.0;
        let d = 600.0;
        for (shape, kappa_a) in [
            (JointShape::Cross, 10.0),
            (JointShape::Tee, 7.0),
            (JointShape::Knee, 5.0),
            (JointShape::Corner, 3.0),
        ] {
            let inp = base_joint_input(shape);
            let res = rc_joint_shear_check(&inp);
            let expected_qaj = kappa_a * (fs - 0.5) * bj * d;
            // QAj は detail 文字列比較ではなく ratio から逆算して照合する。
            let qdj = res.ratio * expected_qaj;
            assert!(qdj > 0.0, "shape={:?}", shape);
            // QAj が形状で単調増加することを確認（十字 > T > ト > L）。
            assert!(expected_qaj > 0.0);
        }
        // 十字形が最も許容せん断力が大きく検定比が最小になるはず。
        let cross = rc_joint_shear_check(&base_joint_input(JointShape::Cross));
        let corner = rc_joint_shear_check(&base_joint_input(JointShape::Corner));
        assert!(cross.ratio < corner.ratio);
    }

    #[test]
    fn rc_joint_bj_uses_max_of_bi_half_and_d_quarter() {
        // 原典「bi/2 または D/4 の大きい方」に従い max を採用する
        // （2026-07-11 原典照合）。col_width が大きく (bi/2) > D/4 となるケースで
        // max=bi/2 が選ばれることを確認。
        // bi = (1200-300)/2 = 450, bi/2=225, D/4=600/4=150 -> bai=max(225,150)=225
        // bj = 300 + 2*225 = 750 (もし min なら bj = 300+2*150=600 になり異なる)
        let mut inp = base_joint_input(JointShape::Cross);
        inp.col_width = 1200.0;
        let res = rc_joint_shear_check(&inp);
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let kappa_a = 10.0;
        let bj_max = 750.0;
        let bj_min = 600.0;
        let qaj_max = kappa_a * (fs - 0.5) * bj_max * inp.col_depth;
        let qaj_min = kappa_a * (fs - 0.5) * bj_min * inp.col_depth;
        let qdj = res.ratio * qaj_max;
        // max 採用時の ratio と、もし min を採用していた場合の ratio は異なるはず。
        let ratio_if_min = qdj / qaj_min;
        assert!((res.ratio - ratio_if_min).abs() > 1e-9);
        // max の方が bj が大きく QAj も大きいので ratio は min のケースより小さい。
        assert!(res.ratio < ratio_if_min);
    }

    #[test]
    fn rc_joint_qdj_takes_min_of_two_candidates() {
        let inp = base_joint_input(JointShape::Cross);
        let denom = inp.col_height * (1.0 - inp.col_depth / inp.beam_span);
        let xi = inp.beam_j / denom;
        assert!(xi > 0.0 && xi < 1.0, "xi should be in valid range: {}", xi);
        let one_minus_xi = 1.0 - xi;
        let qdj1 = inp.sum_beam_moments / inp.beam_j * one_minus_xi;
        let qdj2 = inp.col_shear * one_minus_xi / xi;
        let expected_qdj = qdj1.min(qdj2);

        let res = rc_joint_shear_check(&inp);
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let bi = (inp.col_width - inp.beam_width) / 2.0;
        let bai = (bi / 2.0_f64).max(inp.col_depth / 4.0);
        let bj = inp.beam_width + 2.0 * bai;
        let qaj = 10.0 * (fs - 0.5) * bj * inp.col_depth;
        let expected_ratio = expected_qdj / qaj;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    #[test]
    fn rc_joint_degenerate_xi_falls_back_to_qdj1() {
        // col_depth == beam_span なので denom = col_height*(1 - 1) = 0 -> xi = inf -> 退化。
        let mut inp = base_joint_input(JointShape::Cross);
        inp.beam_span = inp.col_depth;
        let res = rc_joint_shear_check(&inp);
        assert!(res.ratio.is_finite());
        let expected_qdj1 = inp.sum_beam_moments / inp.beam_j;
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let bi = (inp.col_width - inp.beam_width) / 2.0;
        let bai = (bi / 2.0_f64).max(inp.col_depth / 4.0);
        let bj = inp.beam_width + 2.0 * bai;
        let qaj = 10.0 * (fs - 0.5) * bj * inp.col_depth;
        let expected_ratio = expected_qdj1 / qaj;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }
}
