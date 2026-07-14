//! SRC 造柱梁接合部（パネルゾーン）の断面検定（許容応力度検定。
//! SRC 規準の SRC 造柱梁接合部パネルゾーン部分に準拠）。
//!
//! # 位置付け
//! このモジュールは `squid_n_core`（モデル）や要素（`squid_n_element`）に依存せず、
//! 呼び出し側（節点まわりの応力集計・断面形状の解決を担当する別モジュール）が
//! 用意した数値入力を受け取る**純関数**として実装する。
//!
//! 準拠する規準: 日本建築学会「鉄骨鉄筋コンクリート構造計算規準・同解説」
//! （SRC 規準）

use crate::rc::joint::JointShape;
use crate::CheckResult;

/// SRC 造柱梁接合部（パネルゾーン）のせん断検定の入力。
pub struct SrcPanelInput {
    /// 接合部の形状区分（[`JointShape`] を流用）。
    /// 接合部形状係数 jδ: 十字形=3、T字形・ト字形=2、L字形=1。
    pub shape: JointShape,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 長期荷重時の検定かどうか（`true`=長期、`false`=短期）。
    pub long_term: bool,
    /// 柱幅 Cb [mm]。
    pub col_width: f64,
    /// 梁幅 Bb [mm]（`beam_is_steel=true` の場合は未使用）。
    pub beam_width: f64,
    /// 梁の上下主筋間距離 mBd [mm]（`beam_is_steel=true` の場合は梁鉄骨の
    /// フランジ重心間距離 sBd を渡す）。
    pub m_bd: f64,
    /// 柱の左右主筋間距離 mCd [mm]。
    pub m_cd: f64,
    /// 接合部鉄骨ウェブ厚 Jtw [mm]（鉄骨なし=0）。
    pub j_tw: f64,
    /// 柱鉄骨のフランジ重心間距離 sCd [mm]。
    pub s_cd: f64,
    /// 梁が S 造かどうか（`m_bd` に mBd〔SRC/RC〕か sBd〔S〕のどちらが
    /// 渡っているかを示す情報。cV = Cb・m_bd・mCd は両者共通で beam_width は
    /// 使わないため、現在の検定計算では参照しない）。
    pub beam_is_steel: bool,
    /// ヤング係数比 n（[`crate::rc::young_ratio_n`]）。
    pub n_ratio: f64,
    /// 内法階高/階高比 h′/h（不明なら 1.0）。原典図（2026-07-11 照合）の
    /// 右辺係数は `h′/h`。
    pub h_ratio: f64,
    /// 接合部に取り付く大梁端モーメントの絶対値和 BM1+BM2 [N・mm]。
    pub sum_beam_moments: f64,
}

/// SRC 造柱梁接合部（パネルゾーン）のせん断検定（SRC 規準）。
///
/// ## 検定式
/// `cV・jδ・fs・(1+β) ≧ (h′/h)・(BM1+BM2)`
///
/// 左辺が許容値 `Ma`、右辺が設計値 `Md`。検定比 = `Md/Ma`（1.0 以下で OK）。
/// - `fs`: コンクリートの許容せん断応力度（長期/短期、
///   [`crate::rc::concrete_allowable_shear`]）。
/// - `jδ`: 接合部形状係数。十字形=3、T字形・ト字形=2、L字形=1。
///
/// **原典照合済み（2026-07-11）**: 原典図（ユーザー提供）により、
/// 左辺は**全体積 `cV`**（有効体積 `cVe` ではない）、右辺係数は **`h′/h`**
/// （内法階高/階高）であることを確認し、実装を修正した（従来は `cVe`・`h/h′`）。
/// また「3fs」の「3」が接合部形状係数 jδ であること、jδ の値
/// （十字型=3・ト字形/T字形=2・L字形=1）を SRC パネルの原典 PDF の表で確認した。
///
/// ## 諸元
/// - 梁が SRC/RC の場合（`beam_is_steel=false`）: `cV = Cb・mBd・mCd`
/// - 梁が S 造の場合（`beam_is_steel=true`。`m_bd` に sBd を渡す前提）:
///   `cV = Cb・sBd・mCd`
///
/// いずれも `cV = Cb・m_bd・mCd`（`m_bd` に mBd/sBd が入る）で共通のため、
/// `beam_is_steel`・`beam_width` は検定計算では参照しない。
/// `β = n・Jtw・sCd/(Cb・mCd)`（`Cb・mCd ≈ 0` の場合は `β=0`）。
pub fn src_panel_zone_check(inp: &SrcPanelInput) -> CheckResult {
    let j_delta = match inp.shape {
        JointShape::Cross => 3.0,
        JointShape::Tee => 2.0,
        JointShape::Knee => 2.0,
        JointShape::Corner => 1.0,
    };

    // cV = Cb・m_bd・mCd（全体積。m_bd に mBd〔SRC/RC〕か sBd〔S〕が入る）。
    let cv = inp.col_width * inp.m_bd * inp.m_cd;

    let denom = inp.col_width * inp.m_cd;
    let beta = if denom.abs() > 1e-9 {
        inp.n_ratio * inp.j_tw * inp.s_cd / denom
    } else {
        0.0
    };

    let fs = crate::rc::concrete_allowable_shear(inp.fc, inp.long_term);

    let ma = cv * j_delta * fs * (1.0 + beta);
    // 設計用モーメント Md = (h′/h)・(BM1+BM2)（原典図の右辺係数、2026-07-11）。
    let md = inp.h_ratio * inp.sum_beam_moments;

    let ratio = if ma > 0.0 {
        md.abs() / ma
    } else {
        f64::INFINITY
    };
    let ok = ratio <= 1.0;

    let shape_label = match inp.shape {
        JointShape::Cross => "十字形(jdelta=3)",
        JointShape::Tee => "T字形(jdelta=2)",
        JointShape::Knee => "ト字形(jdelta=2)",
        JointShape::Corner => "L字形(jdelta=1)",
    };
    let term_label = if inp.long_term { "長期" } else { "短期" };
    let basis = format!(
        "SRC規準 柱梁接合部（パネル）せん断検定 {} ({})",
        shape_label, term_label
    );
    let detail = format!(
        "cV={:.1} mm3, beta={:.4}, jdelta={:.1}, fs={:.4} N/mm2, Ma={:.1} N*mm, Md={:.1} N*mm, ratio={:.4}",
        cv, beta, j_delta, fs, ma, md, ratio
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

    fn base_src_panel_input() -> SrcPanelInput {
        SrcPanelInput {
            shape: JointShape::Cross,
            fc: 24.0,
            long_term: false,
            col_width: 600.0,
            beam_width: 300.0,
            m_bd: 400.0,
            m_cd: 500.0,
            j_tw: 12.0,
            s_cd: 350.0,
            beam_is_steel: false,
            n_ratio: 15.0,
            h_ratio: 1.0,
            sum_beam_moments: 300_000_000.0,
        }
    }

    #[test]
    fn src_panel_hand_calc() {
        let inp = base_src_panel_input();
        let res = src_panel_zone_check(&inp);

        let cv = inp.col_width * inp.m_bd * inp.m_cd; // 全体積 Cb*mBd*mCd
        let beta = inp.n_ratio * inp.j_tw * inp.s_cd / (inp.col_width * inp.m_cd);
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let ma = cv * 3.0 * fs * (1.0 + beta); // jdelta=3 (十字形)
        let expected_ratio = inp.sum_beam_moments / ma;

        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    #[test]
    fn src_panel_beta_zero_without_steel() {
        let mut inp = base_src_panel_input();
        inp.j_tw = 0.0; // 鉄骨ウェブ厚 0 → β=0
        let res = src_panel_zone_check(&inp);

        let cv = inp.col_width * inp.m_bd * inp.m_cd;
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let ma = cv * 3.0 * fs; // (1+beta) = 1
        let expected_ratio = inp.sum_beam_moments / ma;

        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    #[test]
    fn src_panel_shape_factor_ratio_cross_is_3x_corner() {
        let cross = src_panel_zone_check(&base_src_panel_input());
        let mut corner_input = base_src_panel_input();
        corner_input.shape = JointShape::Corner;
        let corner = src_panel_zone_check(&corner_input);

        // Ma(十字形) = 3・Ma(L字形)（他諸元は同一）なので
        // ratio(L字形) = 3・ratio(十字形)。
        assert!((corner.ratio / cross.ratio - 3.0).abs() < 1e-6);
    }

    #[test]
    fn src_panel_long_term_uses_long_term_fs() {
        let mut inp = base_src_panel_input();
        inp.long_term = true;
        let res = src_panel_zone_check(&inp);

        let cv = inp.col_width * inp.m_bd * inp.m_cd;
        let beta = inp.n_ratio * inp.j_tw * inp.s_cd / (inp.col_width * inp.m_cd);
        let fs_long = crate::rc::concrete_allowable_shear(inp.fc, true);
        let fs_short = crate::rc::concrete_allowable_shear(inp.fc, false);
        assert!(fs_long < fs_short);

        let ma = cv * 3.0 * fs_long * (1.0 + beta);
        let expected_ratio = inp.sum_beam_moments / ma;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);

        // 長期は fs が小さく Ma も小さいため、ratio は短期より大きい。
        let res_short = src_panel_zone_check(&base_src_panel_input());
        assert!(res.ratio > res_short.ratio);
    }

    #[test]
    fn src_panel_cv_uses_full_cb_regardless_of_beam_type() {
        // cV = Cb・m_bd・mCd（全体積、原典図で確認 2026-07-11）は m_bd に
        // mBd/sBd のどちらが入っても同じ式なので、beam_is_steel を切り替えても
        // （m_bd を同値に保てば）検定結果は変わらない。
        let inp_rc = base_src_panel_input();
        let mut inp_s = base_src_panel_input();
        inp_s.beam_is_steel = true;
        let res_rc = src_panel_zone_check(&inp_rc);
        let res_s = src_panel_zone_check(&inp_s);
        assert!((res_rc.ratio - res_s.ratio).abs() < 1e-12);

        let cv = inp_rc.col_width * inp_rc.m_bd * inp_rc.m_cd;
        let beta = inp_rc.n_ratio * inp_rc.j_tw * inp_rc.s_cd / (inp_rc.col_width * inp_rc.m_cd);
        let fs = crate::rc::concrete_allowable_shear(inp_rc.fc, false);
        let ma = cv * 3.0 * fs * (1.0 + beta);
        assert!((res_rc.ratio - inp_rc.sum_beam_moments / ma).abs() < 1e-6);
    }
}
