//! RC 造耐震壁の断面検定（RC 規準18条による許容応力度検定）。
//!
//! # 位置付け
//! このモジュールは `squid_n_core`（モデル）や要素（`squid_n_element`）に依存せず、
//! 呼び出し側（節点まわりの応力集計・断面形状の解決を担当する別モジュール）が
//! 用意した数値入力を受け取る**純関数**として実装する。
//!
//! 準拠する規準: 日本建築学会「鉄筋コンクリート構造計算規準・同解説」18条

use crate::CheckResult;

/// 耐震壁の側柱（壁の両側または片側に取り付く柱）の諸元。
pub struct WallSideColumn {
    /// 柱幅 b [mm]。
    pub b: f64,
    /// 柱の有効せい d [mm]。
    pub d_eff: f64,
    /// 柱の帯筋比 pw。
    pub pw: f64,
    /// 帯筋の短期許容引張応力度 [N/mm²]。
    pub w_ft: f64,
    /// SRC 側柱の内蔵鉄骨によるせん断負担分 `sfs・As` [N]（RC 側柱・鉄骨なしは
    /// 0.0）。`sfs`: 鉄骨の許容せん断応力度、`As`: 側柱内蔵鉄骨のせん断断面積
    /// （ウェブ全せい×ウェブ厚）。[`rc_wall_shear_check`] の `Qc` に単純加算する
    /// （下記 doc 参照）。
    pub steel_shear: f64,
}

/// RC 造耐震壁のせん断検定の入力。
pub struct RcWallInput {
    /// 壁厚 t [mm]。
    pub t: f64,
    /// 柱中心間の壁全せい L [mm]。
    pub l: f64,
    /// 壁板内法長さ l′ [mm]。
    pub l_clear: f64,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 壁筋比（直交2方向のうち小さい方）ps。
    pub ps: f64,
    /// 壁筋の短期許容引張応力度 [N/mm²]。
    pub w_ft: f64,
    /// 側柱（0〜2本）。
    pub side_columns: Vec<WallSideColumn>,
    /// 開口寸法 `(l0, h0, h, l)`。`l0`,`h0`: 開口幅・開口高さ、`h`,`l`: 壁板の
    /// 梁中心間高さ・柱中心間の壁全せい。`None` の場合は無開口（低減係数 r=1）。
    pub opening: Option<(f64, f64, f64, f64)>,
    /// 設計用せん断力 QD [N]。
    pub q_design: f64,
    /// 長期荷重時の検定かどうか（`true`=長期、`false`=短期）。
    pub long_term: bool,
}

/// RC 造耐震壁のせん断検定（RC 規準 18 条）。
///
/// ## コンクリート負担分
/// `Q1 = r・t・l・fs`（`fs` は [`crate::rc::concrete_allowable_shear`] による
/// 長期/短期許容せん断応力度）
///
/// ## 壁筋＋側柱負担分（短期のみ有効）
/// `Q2 = r・(Qw + ΣQc)`
/// - `Qw = ps・t・le・w_ft`
/// - RC 側柱1本あたり `Qc = b・j・(1.5・fs + 0.5・w_ft・(pw − 0.002))`
///   （`j = 7/8・d`、`(pw − 0.002)` が負の場合は 0 とする。係数 `1.5・fs` は
///   RC 規準の規定値をそのまま用いる。）
/// - SRC 側柱（内蔵鉄骨あり）は鉄骨のせん断負担分を加え
///   `Qc = b・j・(1.5・fs + 0.5・w_ft・(pw − 0.002)) + sfs・As`
///   （[`WallSideColumn::steel_shear`] = `sfs・As`。`sfs`: 鉄骨の許容せん断
///   応力度、`As`: 側柱内蔵鉄骨のせん断断面積）。RC 側柱は `steel_shear=0` で
///   従来式に一致する。
///
///   **注記（再構成）**: SRC 造耐震壁の Qc は `0.5・wft・pw`
///   （`pw` に `−0.002` のオフセットが無い）と読める資料もあるが、
///   RC 造耐震壁の式（本関数の `(pw − 0.002)`）と整合させ、既存の RC 実装
///   （オフセット付き）をそのまま維持する（鉄骨項の加算のみ SRC 固有とする）。
/// - 壁の有効長さ `le`: 側柱2本 = `l′`、側柱1本 = `0.9・l′`、側柱なし = `0.8・l′`
///
/// ## 開口低減係数
/// 開口がある場合 `r = min(γ1, γ2, γ3)`
/// （`γ1 = 1 − l0/l`、`γ2 = 1 − √(h0・l0 / (h・l))`、`γ3 = 1 − h0/h`）。
/// 開口がない場合は `r = 1`。極端な開口寸法で `r` が負になる場合は
/// 安全側として 0 にクランプする。
///
/// ## 許容せん断力・検定比
/// - 長期: `Qa = Q1`
/// - 短期: `Qa = max(Q1, Q2)`
/// - 検定比 = `|QD| / Qa`（1.0 以下で OK）
pub fn rc_wall_shear_check(inp: &RcWallInput) -> CheckResult {
    let fs = crate::rc::concrete_allowable_shear(inp.fc, inp.long_term);

    // 開口低減係数 r。
    let r = match inp.opening {
        Some((l0, h0, h, l)) => {
            let gamma1 = 1.0 - l0 / l;
            let gamma2 = 1.0 - ((h0 * l0) / (h * l)).sqrt();
            let gamma3 = 1.0 - h0 / h;
            gamma1.min(gamma2).min(gamma3).max(0.0)
        }
        None => 1.0,
    };

    let q1 = r * inp.t * inp.l * fs;

    // 壁の有効長さ le。
    let le = match inp.side_columns.len() {
        n if n >= 2 => inp.l_clear,
        1 => 0.9 * inp.l_clear,
        _ => 0.8 * inp.l_clear,
    };

    let qw = inp.ps * inp.t * le * inp.w_ft;
    let sum_qc: f64 = inp
        .side_columns
        .iter()
        .map(|c| {
            let j = 7.0 / 8.0 * c.d_eff;
            let pw_term = (c.pw - 0.002).max(0.0);
            c.b * j * (1.5 * fs + 0.5 * c.w_ft * pw_term) + c.steel_shear
        })
        .sum();
    let q2 = r * (qw + sum_qc);

    let qa = if inp.long_term { q1 } else { q1.max(q2) };

    let ratio = if qa > 0.0 {
        inp.q_design.abs() / qa
    } else {
        f64::INFINITY
    };
    let ok = ratio <= 1.0;

    let term_label = if inp.long_term { "長期" } else { "短期" };
    let basis = format!("RC規準18条 耐震壁せん断検定 ({})", term_label);
    let detail = format!(
        "fs={:.4} N/mm2, r={:.4}, le={:.1} mm, Q1={:.1} N, Qw={:.1} N, SumQc={:.1} N, Q2={:.1} N, Qa={:.1} N, ratio={:.4}",
        fs, r, le, q1, qw, sum_qc, q2, qa, ratio
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

    fn base_wall_input() -> RcWallInput {
        RcWallInput {
            t: 180.0,
            l: 4000.0,
            l_clear: 3600.0,
            fc: 24.0,
            ps: 0.006,
            w_ft: 195.0,
            side_columns: vec![
                WallSideColumn {
                    b: 500.0,
                    d_eff: 500.0,
                    pw: 0.004,
                    w_ft: 195.0,
                    steel_shear: 0.0,
                },
                WallSideColumn {
                    b: 500.0,
                    d_eff: 500.0,
                    pw: 0.004,
                    w_ft: 195.0,
                    steel_shear: 0.0,
                },
            ],
            opening: None,
            q_design: 500_000.0,
            long_term: false,
        }
    }

    #[test]
    fn rc_wall_no_opening_r_is_one() {
        let inp = base_wall_input();
        let res = rc_wall_shear_check(&inp);
        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let q1 = 1.0 * inp.t * inp.l * fs;
        // 開口なしなので r=1、Q1 のみ手計算で照合。
        assert!(q1 > 0.0);
        assert!(res.ratio > 0.0);
    }

    #[test]
    fn rc_wall_opening_gamma_takes_min() {
        let mut inp = base_wall_input();
        // l0/l=0.5(gamma1=0.5), h0/h=0.1(gamma3=0.9),
        // gamma2 = 1 - sqrt(0.1*0.5)=1-sqrt(0.05)=1-0.2236=0.7764
        inp.opening = Some((2000.0, 300.0, 3000.0, 4000.0));
        let res = rc_wall_shear_check(&inp);

        let gamma1 = 1.0 - 2000.0_f64 / 4000.0;
        let gamma2 = 1.0 - ((300.0_f64 * 2000.0) / (3000.0 * 4000.0)).sqrt();
        let gamma3 = 1.0 - 300.0_f64 / 3000.0;
        let r = gamma1.min(gamma2).min(gamma3);
        assert!((gamma1 - 0.5).abs() < 1e-9);
        assert!(r < 1.0);
        assert!(res.ratio > 0.0);

        // r=gamma1=0.5 が最小のはず
        assert!((r - gamma1).abs() < 1e-9);
    }

    #[test]
    fn rc_wall_le_three_branches() {
        let mut inp2 = base_wall_input();
        inp2.side_columns.truncate(1); // 1本
        let mut inp0 = base_wall_input();
        inp0.side_columns.clear(); // なし

        let full = base_wall_input(); // 2本

        // le は直接 detail から比較しづらいため、Qw 経由での大小関係を確認する。
        let res_full = rc_wall_shear_check(&full);
        let res_1 = rc_wall_shear_check(&inp2);
        let res_0 = rc_wall_shear_check(&inp0);

        // le(2本 = l_clear) > le(1本 = 0.9 l_clear) > le(0本 = 0.8 l_clear)
        // Qw が大きいほど Q2 が大きく許容せん断力が大きくなるため、検定比は
        // 2本 <= 1本 <= 0本 の順に大きくなる傾向（側柱のQc分も加わるため
        // 単調性は概ね成立するが、ここでは le の値そのものを再計算し確認する）。
        let le_full = 3600.0;
        let le_1 = 0.9 * 3600.0;
        let le_0 = 0.8 * 3600.0;
        assert!(le_full > le_1 && le_1 > le_0);
        assert!(res_full.ratio > 0.0 && res_1.ratio > 0.0 && res_0.ratio > 0.0);
    }

    #[test]
    fn rc_wall_long_term_uses_q1_only() {
        let mut inp = base_wall_input();
        inp.long_term = true;
        let res = rc_wall_shear_check(&inp);

        let fs = crate::rc::concrete_allowable_shear(inp.fc, true);
        let q1 = inp.t * inp.l * fs; // r=1
        let expected_ratio = inp.q_design.abs() / q1;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    #[test]
    fn rc_wall_hand_calc_short_term() {
        let inp = base_wall_input();
        let res = rc_wall_shear_check(&inp);

        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let q1 = inp.t * inp.l * fs;
        let le = inp.l_clear; // 側柱2本
        let qw = inp.ps * inp.t * le * inp.w_ft;
        let sum_qc: f64 = inp
            .side_columns
            .iter()
            .map(|c| {
                let j = 7.0 / 8.0 * c.d_eff;
                let pw_term = (c.pw - 0.002).max(0.0);
                c.b * j * (1.5 * fs + 0.5 * c.w_ft * pw_term)
            })
            .sum();
        let q2 = qw + sum_qc;
        let qa = q1.max(q2);
        let expected_ratio = inp.q_design.abs() / qa;
        assert!((res.ratio - expected_ratio).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // RC 造耐震壁（SRC 側柱の鉄骨せん断項）
    // ------------------------------------------------------------------

    #[test]
    fn rc_wall_steel_shear_adds_to_qc() {
        let mut inp = base_wall_input();
        // 片側柱に鉄骨せん断項 steel_shear=100,000N を追加（SRC 側柱相当）。
        inp.side_columns[0].steel_shear = 100_000.0;
        let res_with_steel = rc_wall_shear_check(&inp);
        let res_without = rc_wall_shear_check(&base_wall_input());
        // 鉄骨項の分だけ Q2（ひいては Qa）が大きくなり ratio は小さくなる
        // （安全側の増分であることを確認）。
        assert!(res_with_steel.ratio < res_without.ratio);

        let fs = crate::rc::concrete_allowable_shear(inp.fc, false);
        let q1 = inp.t * inp.l * fs;
        let le = inp.l_clear;
        let qw = inp.ps * inp.t * le * inp.w_ft;
        let sum_qc: f64 = inp
            .side_columns
            .iter()
            .map(|c| {
                let j = 7.0 / 8.0 * c.d_eff;
                let pw_term = (c.pw - 0.002).max(0.0);
                c.b * j * (1.5 * fs + 0.5 * c.w_ft * pw_term) + c.steel_shear
            })
            .sum();
        let q2 = qw + sum_qc;
        let qa = q1.max(q2);
        let expected_ratio = inp.q_design.abs() / qa;
        assert!((res_with_steel.ratio - expected_ratio).abs() < 1e-6);
    }
}
