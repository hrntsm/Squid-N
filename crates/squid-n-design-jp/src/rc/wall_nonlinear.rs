//! RC 造耐震壁の**せん断非線形特性（トリリニア）**（RESP-D マニュアル
//! 「計算編 05 非線形モデル」耐震壁のせん断非線形特性）。
//!
//! # 位置付け
//! [`super::wall`] が許容応力度検定（RC 規準18条）を扱うのに対し、本モジュールは
//! 非線形（プッシュオーバー・時刻歴）解析で用いる**せん断ばねの骨格曲線**
//! （トリリニア）を算定する。ひび割れ・降伏（剛性低下）・終局の3点を求める。
//!
//! # 準拠する規準・出典
//! - せん断ひび割れ強度 Qc・せん断降伏時剛性低下率 βu:
//!   国土交通省「2007年版建築物の構造関係技術基準解説書」P.635-637。
//! - 終局せん断強度 Qu（荒川mean式系・耐震壁）:
//!   同 P.281-282, 638-639／日本建築学会「鉄筋コンクリート終局強度設計に関する
//!   資料」P.132。
//! - 開口低減率 r: 同 資料 P.132。
//!
//! # 単位系の注意（要・原典照合）
//! Qc・βu の原式は**工学単位系（kgf/cm²・cm²）**で与えられている。本実装は
//! 入力を SI 系（N/mm²・mm²）で受け取り、Qc は内部で kgf/cm² 系へ換算して
//! 評価し、結果を N へ戻す。βu は σy/Fc の比のため単位に依存しない。Qu は
//! 荒川mean式系（N/mm²・mm）でそのまま評価する（[`squid_n_core::rc_capacity`]
//! の梁・柱 Qsu と同じ単位規約）。

/// 単位換算: 1 kgf/cm² = 0.0980665 N/mm²。N/mm² → kgf/cm² は逆数を乗じる。
const NMM2_TO_KGFCM2: f64 = 1.0 / 0.0980665;
/// 単位換算: 1 kgf = 9.80665 N。
const KGF_TO_N: f64 = 9.80665;

/// RC 造耐震壁のせん断トリリニア算定の入力。
///
/// 単位は SI 系（長さ [mm]・面積 [mm²]・応力 [N/mm²]・軸力 [N]）で統一する。
#[derive(Clone, Copy, Debug)]
pub struct WallShearTrilinearInput {
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 壁体断面積 Aw [mm²]（側柱＋壁板の軸断面積。Qc・σ0 の基準面積）。
    pub aw: f64,
    /// 引張側最端の柱 1 本の主筋量 [mm²]（pg = 100·この値/Aw [%]）。
    pub tension_column_main_area: f64,
    /// 壁の縦筋比 pw（小数。βu 用）。
    pub pw_vertical: f64,
    /// 壁筋（縦筋）の降伏強度 σy [N/mm²]（βu 用）。
    pub sigma_y_wall: f64,
    /// 等価壁厚 te [mm]（I 形断面を等価長方形に置換した幅。壁厚 t の 1.5 倍以下）。
    pub te: f64,
    /// 壁厚 t [mm]（pwh = Pwh·t/te の換算に用いる）。
    pub t: f64,
    /// 付帯柱を含めた耐震壁の全長 D [mm]。
    pub d_wall: f64,
    /// 圧縮側柱のせい Dc [mm]（有効せい d = D − Dc/2）。
    pub dc_compression: f64,
    /// 引張側柱の主筋断面積 at [mm²]（pte = 100·at/(te·d) の分子）。
    pub tension_column_at: f64,
    /// 水平せん断補強筋（横筋）の材料強度 σwh [N/mm²]。
    pub sigma_wh: f64,
    /// 横筋比 Pwh（小数。pwh = Pwh·t/te、1.2% 上限）。
    pub pwh_ratio: f64,
    /// 全断面積に対する平均軸方向応力度 σ0 = N/A [N/mm²]（圧縮正）。
    pub sigma_0: f64,
    /// せん断スパン比 M/(Q·D)（適用範囲 1.0〜3.0 にクランプ）。
    pub shear_span_ratio: f64,
    /// 高強度せん断補強筋を用いる場合 true（Qu 係数 0.053→0.068）。
    pub high_strength_shear_rebar: bool,
    /// 開口（`(l0, h0, h, lw)`。l0・h0: 開口幅・高さ、h: 壁の上下梁中心間高さ、
    /// lw: 付帯柱中心間距離）。`None` は無開口（r=1）。
    pub opening: Option<(f64, f64, f64, f64)>,
}

/// RC 造耐震壁のせん断トリリニア骨格（3 点の耐力・剛性低下率）。
#[derive(Clone, Copy, Debug)]
pub struct WallShearTrilinear {
    /// せん断ひび割れ強度 Qc [N]。
    pub qc: f64,
    /// 終局せん断強度 Qu [N]（開口低減 r 適用後）。
    pub qu: f64,
    /// せん断降伏時剛性低下率 βu（無次元。終局点の割線剛性/初期剛性）。
    pub beta_u: f64,
    /// 開口低減率 r（無次元、Qu に乗算済み）。
    pub r_opening: f64,
}

impl WallShearTrilinear {
    /// せん断トリリニアの (せん断変形角 γ, せん断力 Q) 折れ点を返す。
    ///
    /// `k_elastic` は初期弾性せん断剛性 K1（せん断力 Q [N] と せん断変形角 γ
    /// [rad] の関係 Q = K1·γ、すなわち K1 = G·Aw [N]）。トリリニアは:
    /// - 原点 → ひび割れ点 (γc, Qc)、γc = Qc/K1
    /// - ひび割れ点 → 終局点 (γu, Qu)、γu = Qu/(βu·K1)（割線剛性 βu·K1）
    ///
    /// `k_elastic <= 0` の場合は変形を 0 とした縮退点列を返す（呼び出し側で
    /// 剛性が定義できない異常入力の保護）。
    pub fn skeleton_points(&self, k_elastic: f64) -> [(f64, f64); 3] {
        if k_elastic <= 0.0 {
            return [(0.0, 0.0), (0.0, self.qc), (0.0, self.qu)];
        }
        let gamma_c = self.qc / k_elastic;
        // 終局点は割線剛性 βu·K1 上にある（せん断降伏時剛性低下率の定義）。
        let gamma_u = if self.beta_u > 0.0 {
            self.qu / (self.beta_u * k_elastic)
        } else {
            gamma_c
        };
        // ひび割れ後に変形が戻る（Qu 割線がひび割れ点より内側）異常入力では
        // 少なくとも単調増加を保つよう終局点をひび割れ点の外側へ丸める。
        let gamma_u = gamma_u.max(gamma_c);
        [(0.0, 0.0), (gamma_c, self.qc), (gamma_u, self.qu)]
    }
}

/// せん断ひび割れ強度 Qc [N]（RESP-D 非線形モデル・耐震壁）。
///
/// `Qc = (0.043·pg + 0.051)·√Fc·Aw`（Fc [kgf/cm²]・Aw [cm²]・pg [%] → Qc [kgf]）。
/// `pg = 100·(引張側最端の柱1本の主筋量)/Aw` [%]。
///
/// 入力は SI 系で受け取り、内部で工学単位系へ換算して評価し N へ戻す。
/// 不正入力（Fc・Aw のいずれかが 0 以下）は 0.0 を返す。
pub fn wall_shear_crack(inp: &WallShearTrilinearInput) -> f64 {
    if inp.fc <= 0.0 || inp.aw <= 0.0 {
        return 0.0;
    }
    let fc_kgf = inp.fc * NMM2_TO_KGFCM2;
    let aw_cm2 = inp.aw / 100.0;
    // pg [%]（面積比は単位に依存しないため mm² のまま比を取り 100 倍する）。
    let pg_pct = 100.0 * inp.tension_column_main_area.max(0.0) / inp.aw;
    let qc_kgf = (0.043 * pg_pct + 0.051) * fc_kgf.sqrt() * aw_cm2;
    qc_kgf * KGF_TO_N
}

/// せん断降伏時剛性低下率 βu（無次元、RESP-D 非線形モデル・耐震壁）。
///
/// `βu = 0.46·pw·σy/Fc + 0.14`。σy/Fc は比のため単位に依存しない（N/mm² のまま）。
/// 不正入力（Fc が 0 以下）は 0.14（軸項ゼロの下限）を返す。
pub fn wall_shear_beta_u(inp: &WallShearTrilinearInput) -> f64 {
    if inp.fc <= 0.0 {
        return 0.14;
    }
    0.46 * inp.pw_vertical.max(0.0) * inp.sigma_y_wall.max(0.0) / inp.fc + 0.14
}

/// 開口低減率 r（無次元、RESP-D 非線形モデル・耐震壁）。
///
/// `r = 1 − max(r0, l0/lw, h0/h)`、`r0 = √(h0·l0/(h·lw))`。
/// 無開口（`opening == None`）は 1.0。極端な開口で負になる場合は 0 にクランプする。
pub fn wall_shear_opening_reduction(opening: Option<(f64, f64, f64, f64)>) -> f64 {
    match opening {
        Some((l0, h0, h, lw)) if h > 0.0 && lw > 0.0 => {
            let r0 = (h0 * l0 / (h * lw)).max(0.0).sqrt();
            let reduce = r0.max(l0 / lw).max(h0 / h);
            (1.0 - reduce).clamp(0.0, 1.0)
        }
        _ => 1.0,
    }
}

/// 終局せん断強度 Qu [N]（荒川mean式系・耐震壁、RESP-D 非線形モデル）。
///
/// ```text
/// Qu = { k·pte^0.23·(Fc+18)/(M/(Q·D)+0.12) + 0.85·√(σwh·pwh) + 0.1·σ0 }·te·j·r
/// ```
/// - `k = 0.053`（既定）／`0.068`（高強度せん断補強筋）
/// - `pte = 100·at/(te·d)` [%]（等価引張鉄筋比）
/// - `d = D − Dc/2`、`j = 7/8·d`
/// - `M/(Q·D)` は適用範囲 1.0〜3.0 にクランプ
/// - `pwh = Pwh·t/te`（1.2% 上限）
/// - `σ0` は 0〜0.4Fc にクランプ（引張は 0 とみなす。荒川式の適用範囲）
///
/// 開口低減 r を乗じた値を返す。不正入力（Fc・te・D・at のいずれかが 0 以下、
/// または d ≤ 0）は 0.0 を返す。
pub fn wall_shear_ultimate(inp: &WallShearTrilinearInput) -> f64 {
    let d = inp.d_wall - inp.dc_compression / 2.0;
    if inp.fc <= 0.0
        || inp.te <= 0.0
        || inp.d_wall <= 0.0
        || inp.tension_column_at <= 0.0
        || d <= 0.0
    {
        return 0.0;
    }
    let pte = 100.0 * inp.tension_column_at / (inp.te * d);
    let j = 7.0 / 8.0 * d;
    let shear_span_ratio = inp.shear_span_ratio.clamp(1.0, 3.0);
    let k = if inp.high_strength_shear_rebar {
        0.068
    } else {
        0.053
    };
    // pwh = Pwh·t/te、1.2% 上限。
    let pwh = if inp.te > 0.0 {
        (inp.pwh_ratio.max(0.0) * inp.t / inp.te).min(0.012)
    } else {
        0.0
    };
    let concrete_term = k * pte.powf(0.23) * (inp.fc + 18.0) / (shear_span_ratio + 0.12);
    let hoop_term = 0.85 * (pwh * inp.sigma_wh).max(0.0).sqrt();
    let sigma_0 = inp.sigma_0.clamp(0.0, 0.4 * inp.fc);
    let axial_term = 0.1 * sigma_0;
    let r = wall_shear_opening_reduction(inp.opening);
    (concrete_term + hoop_term + axial_term) * inp.te * j * r
}

/// RC 造耐震壁のせん断トリリニア骨格（Qc・βu・Qu・r）を一括算定する。
pub fn wall_shear_trilinear(inp: &WallShearTrilinearInput) -> WallShearTrilinear {
    WallShearTrilinear {
        qc: wall_shear_crack(inp),
        qu: wall_shear_ultimate(inp),
        beta_u: wall_shear_beta_u(inp),
        r_opening: wall_shear_opening_reduction(inp.opening),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 代表壁: t=180, lw=4000 → Aw≈側柱含み(360000×2)+壁板(720000)=1,440,000mm²、
    /// Fc=24, 側柱主筋 1 本分 3097mm²(8-D22 の半分相当は別途)。
    fn base_input() -> WallShearTrilinearInput {
        WallShearTrilinearInput {
            fc: 24.0,
            aw: 1_440_000.0,
            tension_column_main_area: 3097.0,
            pw_vertical: 0.006,
            sigma_y_wall: 345.0,
            te: 180.0,
            t: 180.0,
            d_wall: 4600.0,
            dc_compression: 600.0,
            tension_column_at: 3097.0,
            sigma_wh: 295.0,
            pwh_ratio: 0.004,
            sigma_0: 1.0,
            shear_span_ratio: 1.5,
            high_strength_shear_rebar: false,
            opening: None,
        }
    }

    #[test]
    fn test_wall_shear_crack_matches_handcalc() {
        let inp = base_input();
        let qc = wall_shear_crack(&inp);
        // 手計算（工学単位換算）:
        let fc_kgf: f64 = 24.0 * (1.0 / 0.0980665);
        let aw_cm2: f64 = 1_440_000.0 / 100.0;
        let pg_pct = 100.0 * 3097.0 / 1_440_000.0;
        let qc_kgf = (0.043 * pg_pct + 0.051) * fc_kgf.sqrt() * aw_cm2;
        let qc_hand = qc_kgf * 9.80665;
        assert!((qc - qc_hand).abs() < 1e-3, "Qc={qc} vs handcalc={qc_hand}");
        assert!(qc > 0.0);
    }

    #[test]
    fn test_wall_shear_beta_u_matches_handcalc() {
        let inp = base_input();
        let beta = wall_shear_beta_u(&inp);
        let hand = 0.46 * 0.006 * 345.0 / 24.0 + 0.14;
        assert!((beta - hand).abs() < 1e-9, "βu={beta} vs {hand}");
        // 代表値は 0.15〜0.25 程度に収まる。
        assert!(beta > 0.14 && beta < 0.5);
    }

    #[test]
    fn test_wall_shear_ultimate_matches_handcalc() {
        let inp = base_input();
        let qu = wall_shear_ultimate(&inp);
        // 手計算:
        let d = 4600.0 - 600.0 / 2.0;
        let pte: f64 = 100.0 * 3097.0 / (180.0 * d);
        let j = 7.0 / 8.0 * d;
        let ssr: f64 = 1.5_f64.clamp(1.0, 3.0);
        let pwh = (0.004_f64 * 180.0 / 180.0).min(0.012);
        let concrete = 0.053 * pte.powf(0.23) * (24.0 + 18.0) / (ssr + 0.12);
        let hoop = 0.85 * (pwh * 295.0_f64).sqrt();
        let axial = 0.1 * 1.0_f64.clamp(0.0, 0.4 * 24.0);
        let qu_hand = (concrete + hoop + axial) * 180.0 * j * 1.0;
        assert!((qu - qu_hand).abs() < 1e-3, "Qu={qu} vs {qu_hand}");
        assert!(qu > 0.0);
    }

    #[test]
    fn test_wall_shear_ultimate_high_strength_uses_0068() {
        let mut inp = base_input();
        inp.high_strength_shear_rebar = true;
        let qu_hi = wall_shear_ultimate(&inp);
        let qu_std = wall_shear_ultimate(&base_input());
        // 0.068 > 0.053 なのでコンクリート項が増え Qu が大きくなる。
        assert!(qu_hi > qu_std, "hi={qu_hi} std={qu_std}");
    }

    #[test]
    fn test_wall_shear_opening_reduction_takes_max() {
        // l0/lw=0.5, h0/h=0.1, r0=√(0.05)=0.2236 → max=0.5、r=0.5。
        let r = wall_shear_opening_reduction(Some((2000.0, 300.0, 3000.0, 4000.0)));
        assert!((r - 0.5).abs() < 1e-9, "r={r}");
        // 無開口は 1.0。
        assert!((wall_shear_opening_reduction(None) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_wall_shear_ultimate_opening_reduces() {
        let mut inp = base_input();
        inp.opening = Some((2000.0, 300.0, 3000.0, 4000.0)); // r=0.5
        let qu_open = wall_shear_ultimate(&inp);
        let qu_solid = wall_shear_ultimate(&base_input());
        assert!(
            (qu_open - 0.5 * qu_solid).abs() < 1e-3,
            "Qu_open={qu_open} should be 0.5·Qu_solid={}",
            0.5 * qu_solid
        );
    }

    #[test]
    fn test_wall_shear_ultimate_shear_span_ratio_clamps() {
        let mut low = base_input();
        low.shear_span_ratio = 0.2; // → 1.0
        let mut at_1 = base_input();
        at_1.shear_span_ratio = 1.0;
        assert!((wall_shear_ultimate(&low) - wall_shear_ultimate(&at_1)).abs() < 1e-6);

        let mut high = base_input();
        high.shear_span_ratio = 9.0; // → 3.0
        let mut at_3 = base_input();
        at_3.shear_span_ratio = 3.0;
        assert!((wall_shear_ultimate(&high) - wall_shear_ultimate(&at_3)).abs() < 1e-6);
    }

    #[test]
    fn test_wall_shear_ultimate_pwh_capped_at_1p2pct() {
        let mut over = base_input();
        over.pwh_ratio = 0.05; // te=t なので pwh=0.05 → 0.012 にクランプ
        let mut capped = base_input();
        capped.pwh_ratio = 0.012;
        assert!((wall_shear_ultimate(&over) - wall_shear_ultimate(&capped)).abs() < 1e-6);
    }

    #[test]
    fn test_trilinear_skeleton_points_monotonic() {
        let tri = wall_shear_trilinear(&base_input());
        let k1 = 8000.0 * 1_440_000.0; // G·Aw 相当
        let pts = tri.skeleton_points(k1);
        assert_eq!(pts[0], (0.0, 0.0));
        // 変形・耐力とも単調増加、終局点は割線剛性 βu·K1 上。
        assert!(pts[1].0 > 0.0 && pts[2].0 > pts[1].0);
        assert!(pts[1].1 > 0.0 && pts[2].1 > pts[1].1);
        let gamma_u = tri.qu / (tri.beta_u * k1);
        assert!((pts[2].0 - gamma_u).abs() < 1e-9);
    }

    #[test]
    fn test_invalid_inputs_return_zero() {
        let mut fc0 = base_input();
        fc0.fc = 0.0;
        assert_eq!(wall_shear_crack(&fc0), 0.0);
        assert_eq!(wall_shear_ultimate(&fc0), 0.0);

        let mut te0 = base_input();
        te0.te = 0.0;
        assert_eq!(wall_shear_ultimate(&te0), 0.0);
    }
}
