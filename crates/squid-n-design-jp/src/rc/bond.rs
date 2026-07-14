//! 鉄筋コンクリート造梁付着の断面検定（RC 規準の梁主筋の付着検討）。
//! 既定の検定経路は RC 規準1999 方式
//! （[`rc_beam_bond_check`]）。RC 規準1991 方式（[`rc_beam_bond_check_1991`]）
//! は選択的に使う代替実装として提供する。

use super::{concrete_allowable_bond, one_bar_area};
use squid_n_core::section_shape::{BarSet, RcRebar};

// ============================================================================
// RC 規準 1991 方式の付着検定（鉄筋コンクリート構造計算規準・同解説 1991 の
// 付着検討方法）
// ============================================================================

/// RC 規準 1991 方式の付着検定結果。
pub struct Bond1991Result {
    /// 検定比 = τa / fa。
    pub ratio: f64,
    /// 設計用付着応力度 τa = Q/(φ・j) [N/mm²]。
    pub tau: f64,
    /// 付着許容応力度 fa [N/mm²]。
    pub fa: f64,
}

/// RC 梁付着の断面検定（RC 規準 1991 方式）: `τa = Q/(φ・j) ≦ fa`。
///
/// - `q_abs`: 設計用せん断力 |Q| [N]、`j`: 応力中心間距離（=7/8・d）[mm]。
/// - `phi`: 引張鉄筋の周長総和 Σφ [mm]（= 本数・π・径）。
/// - `top_bar`: 上端筋なら true（fa が低減される）。
///
/// カットオフ位置・スパン途中の鉄筋端までの距離の検定
/// （`ld ≧ σt・a/(0.8fa・φ) + j`）は、モデルにカットオフ筋の情報が無く
/// 通し筋を仮定するため対象外（検定断面位置の τa 検定のみ）。
/// 既定の検定経路は RC 規準 1999 方式（[`rc_beam_bond_check`]）であり、
/// 本関数は 1991 方式を選択したい呼び出し側向けの代替実装。
pub fn rc_beam_bond_check_1991(
    q_abs: f64,
    j: f64,
    phi: f64,
    fc_raw: f64,
    top_bar: bool,
    long_term: bool,
) -> Option<Bond1991Result> {
    if q_abs < 0.0 || j <= 0.0 || phi <= 0.0 || fc_raw <= 0.0 {
        return None;
    }
    let tau = q_abs / (phi * j);
    let fa = concrete_allowable_bond(fc_raw, top_bar, long_term);
    if fa <= 0.0 {
        return None;
    }
    Some(Bond1991Result {
        ratio: tau / fa,
        tau,
        fa,
    })
}

// ============================================================================
// RC 梁付着の断面検定（RC 規準1999 の付着検討に準拠）
// ============================================================================
//
// # 実装方針・簡略化（重要）
// - モデルにカットオフ筋（途中で切断される主筋）の情報がないため、
//   主筋は全断面にわたり「通し筋」（カットオフ無し）であると仮定する。
//   RC 規準1999 の付着検討の表はカットオフの有無で ld（付着長さ）の値が
//   異なるが、本実装は「カットオフ無し」の行のみを用いる。
// - 検定断面は `check()` が呼ばれる評価位置 `pos`（0.0〜1.0の部材内位置）
//   から、`pos<=0.25` を左端、`0.25<pos<0.75` を中央、それ以外
//   （`pos>=0.75`）を右端として分類する。左端・右端はいずれも「端部」
//   として同じ式（ld=(Lo+d)/2）を用いる。
// - Lo（柱面間距離、内法スパン）はモデルに保持されていないため、
//   `DesignCtx.length`（部材の幾何学的長さ）で代用する。
// - K・W は RC 規準1999 の標準式を 1 段目のあき・かぶりで代表させる
//   （2段目のあき・割裂面の個別評価は行わない）。fb は多段配筋
//   （`BarSet.layers ≥ 2`）のとき 0.6 倍に低減し（RC 規準1999 の
//   「1段筋以外は 0.6 を乗じる」）、必要付着長さが最大となる 2 段目以降の
//   鉄筋列で代表して検定する（1段筋は同 σt・同 K なら常にこれを下回る）。
// - 上端筋/下端筋の判定は、梁端部は負曲げ（上端引張）が生じるものとして
//   「端部＝上端筋（fb×0.8）」「中央＝下端筋（低減なし）」と仮定する
//   （実際の応力分布・配筋詳細に依らない保守側の簡略化）。

/// RC 梁付着の断面検定結果。
pub struct BondCheckResult {
    /// 検定比 = ldb / ld。
    pub ratio: f64,
    /// 付着長さ ld [mm]（カットオフ無しを仮定した表値）。
    pub ld: f64,
    /// 必要付着長さ ldb [mm]。
    pub ldb: f64,
    /// 付着割裂の検討用係数 K。
    pub k: f64,
    /// 横補強筋効果を表す換算長さ W [mm]。
    pub w: f64,
    /// 付着許容応力度 fb [N/mm²]。
    pub fb: f64,
    /// 検定断面の鉄筋引張応力度 σt = |M|/(at・j) [N/mm²]。
    pub sigma_t: f64,
    /// 端部（左端・右端）の検定断面であれば true、中央であれば false。
    pub is_end: bool,
}

/// 主筋 1 本あたりの周長 φ = π・dia [mm]。
fn one_bar_perimeter(dia: f64) -> f64 {
    std::f64::consts::PI * dia
}

/// RC 梁の付着の断面検定（RC 規準1999、通し筋・カットオフ無しを仮定）。
///
/// - `pos`: 検定断面の部材内位置（0.0〜1.0）。
/// - `lo`: 柱面間距離 Lo [mm]（`DesignCtx.length` で代用）。`lo<=0` の
///   場合は付着検定に必要な情報が無いとみなし `None` を返す（検定省略）。
/// - `b`, `d_eff`, `j`, `at`: 検討方向の断面諸元（幅・有効せい・応力中心間
///   距離・引張鉄筋断面積）。
/// - `mz_abs`: 検定断面の |M| [N・mm]。
/// - `main`: 引張側主筋（強軸曲げなら `rebar.main_x`）。
/// - `rebar`: かぶり・せん断補強筋情報の取得用。
/// - `fc_raw`: コンクリート設計基準強度 Fc [N/mm²]。
/// - `long_term`: 長期なら true。
#[allow(clippy::too_many_arguments)]
pub fn rc_beam_bond_check(
    pos: f64,
    lo: f64,
    b: f64,
    d_eff: f64,
    j: f64,
    at: f64,
    mz_abs: f64,
    main: &BarSet,
    rebar: &RcRebar,
    fc_raw: f64,
    long_term: bool,
) -> Option<BondCheckResult> {
    if lo <= 0.0 || main.count == 0 || main.dia <= 0.0 || at <= 0.0 || j <= 0.0 {
        return None;
    }

    let db = main.dia;
    // pos<=0.25: 左端、0.25<pos<0.75: 中央、pos>=0.75: 右端（端部扱いは左右共通）。
    let is_end = !(0.25 < pos && pos < 0.75);

    // 付着長さ ld（通し筋・カットオフ無しの表値）。
    let ld = if is_end { (lo + d_eff) / 2.0 } else { lo / 2.0 };
    if ld <= 0.0 {
        return None;
    }

    // n1 = 1段の本数（=count/layers）。BarSet.count・layers から単純に
    // 求める簡略化（本数が層数で割り切れない場合も比率をそのまま用いる）。
    let layers = (main.layers.max(1)) as f64;
    let n1 = (main.count as f64 / layers).max(1.0);

    // 鉄筋間のあき（1本のときは 5db とみなす）。
    let clear_spacing = if n1 <= 1.0 {
        5.0 * db
    } else {
        (b - 2.0 * (rebar.cover + rebar.shear.dia) - n1 * db) / (n1 - 1.0)
    };
    // C = min(鉄筋間のあき, 3×最小かぶり, 5・db)。
    let c = clear_spacing.min(3.0 * rebar.cover).min(5.0 * db);

    // W = min(20・Ast/(s・N), 2.5・db)。Ast は 1 組のせん断補強筋全断面積。
    let ast =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    let w = if rebar.shear.pitch > 0.0 {
        (20.0 * ast / (rebar.shear.pitch * n1)).min(2.5 * db)
    } else {
        0.0
    };

    // K = 0.3・(C+W)/db + 0.4（短期）/ 0.3・C/db + 0.4（長期、W 項無し）。上限 2.5。
    let k_raw = if long_term {
        0.3 * c / db + 0.4
    } else {
        0.3 * (c + w) / db + 0.4
    };
    let k = k_raw.min(2.5);
    if k <= 0.0 {
        return None;
    }

    // fb: 長期「その他鉄筋」= Fc/60+0.6、上端筋はその 0.8 倍。短期は長期の 1.5 倍。
    // 多段配筋（layers ≥ 2）は 1 段筋以外の 0.6 倍低減を乗じ、必要付着長さが
    // 最大となる 2 段目以降の鉄筋列で代表して検定する（RC 規準1999）。
    let fb_other = fc_raw / 60.0 + 0.6;
    let fb_long = if is_end { 0.8 * fb_other } else { fb_other };
    let fb_row1 = if long_term { fb_long } else { fb_long * 1.5 };
    let fb = if main.layers >= 2 {
        0.6 * fb_row1
    } else {
        fb_row1
    };
    if fb <= 0.0 {
        return None;
    }

    // σt = |M|/(at・j) を鉄筋断面の平均応力度として用いる。
    let sigma_t = mz_abs / (at * j);
    let as_bar = one_bar_area(db);
    let phi = one_bar_perimeter(db);
    let ldb = sigma_t * as_bar / (k * fb * phi);

    Some(BondCheckResult {
        ratio: ldb / ld,
        ld,
        ldb,
        k,
        w,
        fb,
        sigma_t,
        is_end,
    })
}

// ============================================================================
// テスト
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::section_shape::ShearBar;

    // ------------------------------------------------------------------
    // RC 規準 1991 方式の付着検定
    // ------------------------------------------------------------------

    #[test]
    fn test_rc_beam_bond_check_1991_hand_calc() {
        // 4-D22（φ=4×π×22）、j=7/8×540、Q=180kN、Fc=24、上端筋・短期。
        let phi = 4.0 * std::f64::consts::PI * 22.0;
        let j = 7.0 / 8.0 * 540.0;
        let res = rc_beam_bond_check_1991(180_000.0, j, phi, 24.0, true, false)
            .expect("有効入力なら Some");
        let tau = 180_000.0 / (phi * j);
        let fa = 1.54 * 1.5;
        assert!((res.tau - tau).abs() < 1e-9);
        assert!((res.fa - fa).abs() < 1e-9);
        assert!((res.ratio - tau / fa).abs() < 1e-9);
        // 不正入力は None。
        assert!(rc_beam_bond_check_1991(1.0, 0.0, phi, 24.0, true, true).is_none());
    }

    // ------------------------------------------------------------------
    // (B) RC 梁付着の断面検定（RC 規準1999）
    // ------------------------------------------------------------------

    fn bond_test_rebar() -> (BarSet, RcRebar) {
        let main = BarSet {
            count: 6,
            dia: 22.0,
            layers: 1,
        };
        let rebar = RcRebar {
            main_x: main.clone(),
            main_y: main.clone(),
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        };
        (main, rebar)
    }

    #[test]
    fn test_bond_lo_zero_skips_check() {
        let (main, rebar) = bond_test_rebar();
        let result = rc_beam_bond_check(
            0.1,
            0.0,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        );
        assert!(result.is_none(), "Lo<=0 のときは付着検定を省略するはず");
    }

    #[test]
    fn test_bond_ld_end_vs_middle() {
        let (main, rebar) = bond_test_rebar();
        let lo = 3000.0;
        let d_eff = 539.0;
        let end = rc_beam_bond_check(
            0.1,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();
        let mid = rc_beam_bond_check(
            0.5,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();
        let right = rc_beam_bond_check(
            0.9,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();

        assert!((end.ld - (lo + d_eff) / 2.0).abs() < 1e-6);
        assert!((mid.ld - lo / 2.0).abs() < 1e-6);
        assert!((right.ld - (lo + d_eff) / 2.0).abs() < 1e-6);
        assert!(end.is_end);
        assert!(!mid.is_end);
        assert!(right.is_end);
    }

    #[test]
    fn test_bond_fb_top_bar_factor_0_8() {
        // 端部（上端筋想定）の fb は中央（下端筋想定）の 0.8 倍。
        let (main, rebar) = bond_test_rebar();
        let lo = 3000.0;
        let end = rc_beam_bond_check(
            0.1,
            lo,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();
        let mid = rc_beam_bond_check(
            0.5,
            lo,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();
        assert!((end.fb - 0.8 * mid.fb).abs() < 1e-9);
    }

    #[test]
    fn test_bond_c_selects_minimum_of_spacing_cover_5db() {
        let (main, rebar) = bond_test_rebar();
        let b = 300.0;
        let n1 = 6.0;
        let db = 22.0;
        let expected_spacing = (b - 2.0 * (40.0 + 10.0) - n1 * db) / (n1 - 1.0); // = 13.6
        assert!(expected_spacing < 3.0 * 40.0 && expected_spacing < 5.0 * db);

        // 長期は K=0.3・C/db+0.4（W 項なし）なので C を逆算で検証できる。
        let result = rc_beam_bond_check(
            0.1,
            3000.0,
            b,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            true,
        )
        .unwrap();
        let expected_k = (0.3 * expected_spacing / db + 0.4).min(2.5);
        assert!((result.k - expected_k).abs() < 1e-6);
    }

    #[test]
    fn test_bond_k_clamped_at_2_5() {
        // C・W ともに上限（5db・2.5db）近くまで大きくし、K が 2.5 にクランプ
        // されることを確認する。
        let main = BarSet {
            count: 2,
            dia: 10.0,
            layers: 1,
        };
        let rebar = RcRebar {
            main_x: main.clone(),
            main_y: main.clone(),
            cover: 100.0,
            shear: ShearBar {
                dia: 12.0,
                pitch: 50.0,
                legs: 10,
                grade: None,
            },
        };
        let result = rc_beam_bond_check(
            0.1,
            3000.0,
            1000.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &main,
            &rebar,
            24.0,
            false,
        )
        .unwrap();
        assert!((result.k - 2.5).abs() < 1e-9, "K={}", result.k);
    }

    #[test]
    fn test_bond_fb_second_row_factor_0_6() {
        // 多段配筋（layers=2）の fb は 1 段配筋の 0.6 倍（2段目以降の鉄筋列で
        // 代表して検定。RC 規準1999「1段筋以外は 0.6 を乗じる」）。
        // 1段あたり本数 n1=count/layers を 6 本で揃え、K・W を同一にして
        // fb の低減だけを比較する。
        let (main1, rebar1) = bond_test_rebar();
        let main2 = BarSet {
            count: 12,
            layers: 2,
            ..main1.clone()
        };
        let rebar2 = RcRebar {
            main_x: main2.clone(),
            ..rebar1.clone()
        };
        let run = |main: &BarSet, rebar: &RcRebar| {
            rc_beam_bond_check(
                0.1,
                3000.0,
                300.0,
                539.0,
                471.625,
                1140.4,
                30_000_000.0,
                main,
                rebar,
                24.0,
                false,
            )
            .unwrap()
        };
        let one = run(&main1, &rebar1);
        let two = run(&main2, &rebar2);
        assert!((two.k - one.k).abs() < 1e-9, "K は同一条件");
        assert!((two.fb - 0.6 * one.fb).abs() < 1e-9, "fb={}", two.fb);
        assert!(
            (two.ratio - one.ratio / 0.6).abs() / one.ratio < 1e-9,
            "検定比は 1/0.6 倍厳しくなる"
        );
    }

    #[test]
    fn test_bond_ldb_over_ld_handcalc() {
        let (main, rebar) = bond_test_rebar();
        let b = 300.0;
        let d_eff = 539.0;
        let j = 471.625;
        let at = 6.0 * std::f64::consts::PI * (22.0_f64 / 2.0).powi(2) / 2.0;
        let lo = 3000.0;
        let mz_abs = 30_000_000.0;
        let fc = 24.0;

        let result =
            rc_beam_bond_check(0.1, lo, b, d_eff, j, at, mz_abs, &main, &rebar, fc, false).unwrap();

        // 独立に手計算した期待値。
        let n1 = 6.0;
        let db = 22.0;
        let spacing = (b - 2.0 * (40.0 + 10.0) - n1 * db) / (n1 - 1.0);
        let c = spacing.min(3.0 * 40.0).min(5.0 * db);
        let ast = 2.0 * std::f64::consts::PI / 4.0 * 10.0 * 10.0;
        let w = (20.0 * ast / (100.0 * n1)).min(2.5 * db);
        let k = (0.3 * (c + w) / db + 0.4).min(2.5);
        let fb_other = fc / 60.0 + 0.6;
        let fb = 0.8 * fb_other * 1.5; // 端部・短期
        let sigma_t = mz_abs / (at * j);
        let as_bar = std::f64::consts::PI * (db / 2.0).powi(2);
        let phi = std::f64::consts::PI * db;
        let ldb = sigma_t * as_bar / (k * fb * phi);
        let ld = (lo + d_eff) / 2.0;
        let expected_ratio = ldb / ld;

        assert!((result.ldb - ldb).abs() / ldb < 1e-6);
        assert!((result.ld - ld).abs() / ld < 1e-6);
        assert!((result.ratio - expected_ratio).abs() / expected_ratio < 1e-6);
    }
}
