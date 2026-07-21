//! 鉄筋コンクリート造梁・柱の**終局せん断強度（塑性理論式）**および
//! **付着割裂による終局せん断耐力**（日本建築学会「鉄筋コンクリート造建物の
//! 靭性保証型耐震設計指針・同解説」）。
//!
//! # 位置付け
//! [`crate::rc`] が許容応力度検定（04 章）、[`crate::rc::beam_nonlinear`] 等が
//! 非線形復元力特性（05 章）を扱うのに対し、本モジュールは**終局検定（06 章）**の
//! 「終局強度型設計指針」を選択した場合の終局せん断強度 `Qsu`（塑性理論式）と
//! 付着割裂によって決定するせん断耐力 `Qbu` を算定する純関数群である。
//!
//! 既存の [`squid_n_core::rc_capacity::rc_qsu_simple`]（荒川mean式）は部材ランク
//! 自動判定・プッシュオーバーのせん断降伏判定用の略算式であり、本モジュールの
//! 塑性理論式（トラス機構＋アーチ機構）とは定式が異なる別物である。本実装では
//! 「終局強度型設計指針」を選択した場合に本式（塑性理論式）を採用する。
//!
//! # 準拠する規準・出典（要・原典照合、`dev_docs/specs/原典照合リスト.md`）
//! - 塑性理論式 `Qsu = b·jt·pw·σwy·cotφ + k1·(1−k2)·b·D·ν·Fc`:
//!   日本建築学会「鉄筋コンクリート造建物の靭性保証型耐震設計指針・同解説」
//!   （終局強度型設計指針、藤井・森田式系）。
//! - 付着割裂 `Qbu = jt·τbu·Σφ + k1·(1−k3)·b·D·ν·Fc`: 同指針 P.175-181。
//! - 軽量コンクリートのせん断終局耐力 0.9 倍低減: 技術基準解説書（共通事項）。

/// コンクリート圧縮強度の有効係数 `ν0`（降伏ヒンジ・潜在ヒンジを計画しない時）。
///
/// `ν0 = 0.7 − Fc/200`（靭性保証型耐震設計指針）。`Fc` は [N/mm²]。
/// 下限は 0（Fc が極端に大きい異常入力で負にならないようクランプ）。
pub fn plastic_nu0(fc: f64) -> f64 {
    (0.7 - fc / 200.0).max(0.0)
}

/// 終局限界状態のヒンジ回転角 `Rp` [rad] に応じたコンクリート圧縮強度の
/// 有効係数 `ν`（靭性保証型耐震設計指針 塑性理論式）。
///
/// ```text
/// ν = { (1.0 − 15·Rp)·ν0   (0 < Rp ≤ 0.05)
///       0.25·ν0            (0.05 < Rp) }
/// ```
/// `Rp ≤ 0`（塑性化前）は `ν = ν0`（`Rp→0` の極限）とする。`(1−15·Rp)` は
/// `Rp=0.05` で 0.25 に一致し、両分岐は連続する。負値にはクランプする。
pub fn plastic_nu(fc: f64, rp: f64) -> f64 {
    plastic_nu_from_nu0(plastic_nu0(fc), rp)
}

/// ν0 を与えて Rp 依存の有効係数 ν を求める（[`plastic_nu`] の一般形。
/// 高強度せん断補強筋の製品別 ν0 上書き用）。
pub fn plastic_nu_from_nu0(nu0: f64, rp: f64) -> f64 {
    let factor = if rp <= 0.0 {
        1.0
    } else if rp <= 0.05 {
        (1.0 - 15.0 * rp).max(0.25)
    } else {
        0.25
    };
    (factor * nu0).max(0.0)
}

/// トラス機構の圧縮束の角度 φ のコタンジェント `cotφ`（靭性保証型耐震設計指針）。
///
/// ```text
/// cotφ = { 2.0 − 50·Rp   (0 < Rp ≤ 0.02)
///          1.0           (0.02 < Rp) }
/// ```
/// `Rp ≤ 0`（塑性化前）は `cotφ = 2.0`（`Rp→0` の極限）とする。両分岐は
/// `Rp=0.02` で 1.0 に一致し連続する。下限は 1.0。
pub fn plastic_cot_phi(rp: f64) -> f64 {
    if rp <= 0.0 {
        2.0
    } else if rp <= 0.02 {
        (2.0 - 50.0 * rp).max(1.0)
    } else {
        1.0
    }
}

/// アーチ機構の係数 `k1 = (√((L/D)² + 1) − (L/D)) / 2`（靭性保証型耐震設計指針）。
///
/// `L`: 内法長さ、`D`: 部材せい [mm]。`D ≤ 0` の不正入力は 0 を返す。
pub fn plastic_k1(l_clear: f64, d_full: f64) -> f64 {
    if d_full <= 0.0 || l_clear < 0.0 {
        return 0.0;
    }
    let ld = l_clear / d_full;
    ((ld * ld + 1.0).sqrt() - ld) / 2.0
}

/// せん断補強筋がトラス機構に寄与する割合 `k2 = 2·pw·σwy/(ν·Fc)`
/// （靭性保証型耐震設計指針）。上限 1.0 でクランプする（`k2 ≤ 1.0`）。
///
/// `ν·Fc ≤ 0` の不正入力は 1.0 を返す（アーチ項を 0 にする安全側）。
pub fn plastic_k2(pw: f64, sigma_wy: f64, nu: f64, fc: f64) -> f64 {
    let denom = nu * fc;
    if denom <= 0.0 {
        return 1.0;
    }
    (2.0 * pw.max(0.0) * sigma_wy.max(0.0) / denom).min(1.0)
}

/// 塑性理論式による終局せん断強度 `Qsu` の算定入力（靭性保証型耐震設計指針）。
#[derive(Clone, Copy, Debug)]
pub struct RcPlasticShearInput {
    /// 部材幅 b [mm]。
    pub b: f64,
    /// 部材せい D [mm]。
    pub d_full: f64,
    /// 主筋の重心間距離 jt [mm]（トラス機構。通常 7/8·d 相当）。
    pub jt: f64,
    /// せん断補強筋比 pw（小数、= aw·組数/(b·ピッチ)）。
    pub pw: f64,
    /// せん断補強筋の降伏強度算定用強度 σwy [N/mm²]。
    pub sigma_wy: f64,
    /// 内法長さ L [mm]（アーチ機構の k1 に用いる）。
    pub l_clear: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν・cotφ に用いる）。
    /// 塑性化前の終局強度を評価する場合は 0 を与える（cotφ=2.0, ν=ν0）。
    pub rp: f64,
    /// 軽量コンクリートを使用する場合 true（せん断終局耐力を 0.9 倍に低減）。
    pub lightweight: bool,
    /// ν0 の製品別上書き（高強度せん断補強筋使用時。
    /// [`crate::material_strength::ultimate_hoop_nu0`]）。`None` は標準式
    /// `ν0 = 0.7 − Fc/200`（[`plastic_nu0`]）。
    pub nu0_override: Option<f64>,
}

/// 塑性理論式による終局せん断強度 `Qsu` [N]（靭性保証型耐震設計指針）。
///
/// ```text
/// Qsu = b·jt·pw·σwy·cotφ + k1·(1−k2)·b·D·ν·Fc
/// k1   = (√((L/D)² + 1) − (L/D)) / 2
/// k2   = 2·pw·σwy/(ν·Fc)                              （上限 1.0）
/// ν    = { (1.0 − 15·Rp)·ν0  (0 < Rp ≤ 0.05); 0.25·ν0 (0.05 < Rp) }
/// ν0   = 0.7 − Fc/200
/// cotφ = { 2.0 − 50·Rp (0 < Rp ≤ 0.02); 1.0 (0.02 < Rp) }
/// ```
/// - 第 1 項はトラス機構、第 2 項はアーチ機構の寄与。
/// - 制約 `pw·σwy ≤ ν·Fc/2` を満たすよう、トラス項・k2 の双方で `pw·σwy` を
///   `ν·Fc/2` で上限クランプする（`k2 ≤ 1.0` と整合し、アーチ項が負にならない）。
/// - `lightweight` が true の場合、算定値を 0.9 倍に低減する（共通事項）。
/// - 不正入力（b・D・jt・Fc のいずれかが 0 以下）は 0.0 を返す。
pub fn rc_shear_qsu_plastic(inp: &RcPlasticShearInput) -> f64 {
    if inp.b <= 0.0 || inp.d_full <= 0.0 || inp.jt <= 0.0 || inp.fc <= 0.0 {
        return 0.0;
    }
    let nu0 = inp.nu0_override.unwrap_or_else(|| plastic_nu0(inp.fc));
    let nu = plastic_nu_from_nu0(nu0, inp.rp);
    let cot_phi = plastic_cot_phi(inp.rp);
    let k1 = plastic_k1(inp.l_clear, inp.d_full);

    // 制約 pw·σwy ≤ ν·Fc/2 をトラス項へ反映（k2 の上限 1.0 と整合）。
    let pw_sigma = (inp.pw.max(0.0) * inp.sigma_wy.max(0.0)).min(nu * inp.fc / 2.0);
    let k2 = plastic_k2(inp.pw, inp.sigma_wy, nu, inp.fc);

    let truss = inp.b * inp.jt * pw_sigma * cot_phi;
    let arch = k1 * (1.0 - k2) * inp.b * inp.d_full * nu * inp.fc;
    let qsu = (truss + arch).max(0.0);

    if inp.lightweight {
        0.9 * qsu
    } else {
        qsu
    }
}

/// 付着割裂による終局せん断耐力 `Qbu` の算定入力（靭性保証型耐震設計指針）。
#[derive(Clone, Copy, Debug)]
pub struct RcBondSplitInput {
    /// 部材幅 b [mm]。
    pub b: f64,
    /// 部材せい D [mm]。
    pub d_full: f64,
    /// 主筋の重心間距離 jt [mm]。
    pub jt: f64,
    /// 1 段目主筋の付着信頼強度 τbu [N/mm²]（[`bond_reliable_strength_deformed`]）。
    pub tau_bu: f64,
    /// 引張鉄筋の周長和 Σφ [mm]（2 段筋・寄筋を含める）。
    pub sum_phi: f64,
    /// 内法長さ L [mm]（アーチ機構の k1 に用いる）。
    pub l_clear: f64,
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν に用いる）。
    pub rp: f64,
    /// 軽量コンクリートを使用する場合 true（0.9 倍低減）。
    pub lightweight: bool,
}

/// 付着割裂によって決定する終局せん断耐力 `Qbu` [N]（靭性保証型耐震設計指針）。
///
/// ```text
/// Qbu = jt·τbu·Σφ + k1·(1−k3)·b·D·ν·Fc
/// k3  = 2·τbu·Σφ/(b·ν·Fc)          （上限 1.0）
/// ```
/// - 第 1 項は付着力によるトラス機構、第 2 項はアーチ機構。
/// - `k3` はコンクリートの一部を付着トラスが負担する割合で、`k1·(1−k3)` により
///   アーチ機構に残るコンクリートを評価する（`Qsu` の `k2` と同型）。
/// - `lightweight` が true の場合 0.9 倍に低減する。
/// - 不正入力（b・D・jt・Fc のいずれかが 0 以下）は 0.0 を返す。
pub fn rc_shear_qbu_bond(inp: &RcBondSplitInput) -> f64 {
    if inp.b <= 0.0 || inp.d_full <= 0.0 || inp.jt <= 0.0 || inp.fc <= 0.0 {
        return 0.0;
    }
    let nu = plastic_nu(inp.fc, inp.rp);
    let k1 = plastic_k1(inp.l_clear, inp.d_full);
    let denom = inp.b * nu * inp.fc;
    let k3 = if denom > 0.0 {
        (2.0 * inp.tau_bu.max(0.0) * inp.sum_phi.max(0.0) / denom).min(1.0)
    } else {
        1.0
    };
    let bond = inp.jt * inp.tau_bu.max(0.0) * inp.sum_phi.max(0.0);
    let arch = k1 * (1.0 - k3) * inp.b * inp.d_full * nu * inp.fc;
    let qbu = (bond + arch).max(0.0);
    if inp.lightweight {
        0.9 * qbu
    } else {
        qbu
    }
}

/// 異径鉄筋（普通の異形鉄筋）の 1 段目主筋の付着信頼強度 τbu [N/mm²] の
/// 算定入力（靭性保証型耐震設計指針 P.175-181）。
#[derive(Clone, Copy, Debug)]
pub struct BondStrengthInput {
    /// コンクリートの設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 部材幅 b [mm]。
    pub b: f64,
    /// 1 段目主筋径（呼び名）db1 [mm]。
    pub db1: f64,
    /// 外側一列の引張鉄筋の本数 N。
    pub n_bars: u32,
    /// 主筋の側面かぶり Cs [mm]。
    pub cover_side: f64,
    /// 主筋の底面（上面）かぶり Cb [mm]。
    pub cover_bottom: f64,
    /// せん断補強筋 1 組の断面積 aw [mm²]（kst 用）。
    pub hoop_area: f64,
    /// せん断補強筋ピッチ x [mm]（kst 用）。
    pub hoop_pitch: f64,
    /// せん断補強筋比 pw（小数、kst 用）。
    pub pw: f64,
    /// 梁の上端主筋の場合 true（付着強度低減係数 αt = 0.75 − Fc/400）。
    pub top_bar: bool,
}

/// 割裂線長さ比 bi = min(bvi, bci, bsi)（靭性保証型耐震設計指針）。
///
/// ```text
/// bvi = √3·(2·Cmin/db1 + 1)
/// bci = √2·((Cs + Cb)/db1 − 1)
/// bsi = b/(N·db1) − 1
/// Cmin = min(Cs, Cb)
/// ```
/// `db1 ≤ 0` または `N = 0` の不正入力は 0 を返す。負値は 0 にクランプする。
pub fn bond_split_ratio(b: f64, db1: f64, n_bars: u32, cover_side: f64, cover_bottom: f64) -> f64 {
    if db1 <= 0.0 || n_bars == 0 {
        return 0.0;
    }
    let n = n_bars as f64;
    let c_min = cover_side.min(cover_bottom);
    let bvi = 3.0_f64.sqrt() * (2.0 * c_min / db1 + 1.0);
    let bci = 2.0_f64.sqrt() * ((cover_side + cover_bottom) / db1 - 1.0);
    let bsi = b / (n * db1) - 1.0;
    bvi.min(bci).min(bsi).max(0.0)
}

/// 異径鉄筋の 1 段目主筋の付着信頼強度 τbu [N/mm²]（靭性保証型耐震設計指針 P.175-181）。
///
/// ```text
/// τbu = αt·((0.085·b1 + 0.10)·√Fc + kst)
/// αt  = { 0.75 − Fc/400  (梁の上端主筋); 1.0 (上記以外) }
/// b1  = 割裂線長さ比 = min(bvi, bci, bsi)
/// kst = 140·Aw/(db1·x)   （横補強筋効果、bci<bsi 相当の代表式）
/// ```
///
/// # 簡略化（doc 兼申し送り）
/// 同指針の `kst` は `(54 + 45·Nw/N1)·(bsi+1)·pw`（bci≥bsi）と
/// `140·Aw/(db·s)`（bci<bsi）の 2 分岐だが、前者の `Nw`（中子筋の本数）・`N1`
/// （外側鉄筋本数）はモデルに保持されないため、本実装では後者の
/// `kst = 140·aw/(db1·x)`（横補強筋の直接的な拘束効果）を代表式として全域に
/// 用いる。`Aw` は 1 組のせん断補強筋断面積、`x` はピッチとする。ピッチが
/// 0 以下なら kst=0。
///
/// 不正入力（Fc・b・db1 のいずれかが 0 以下、または N=0）は 0.0 を返す。
pub fn bond_reliable_strength_deformed(inp: &BondStrengthInput) -> f64 {
    if inp.fc <= 0.0 || inp.b <= 0.0 || inp.db1 <= 0.0 || inp.n_bars == 0 {
        return 0.0;
    }
    let b1 = bond_split_ratio(inp.b, inp.db1, inp.n_bars, inp.cover_side, inp.cover_bottom);
    let alpha_t = if inp.top_bar {
        (0.75 - inp.fc / 400.0).max(0.0)
    } else {
        1.0
    };
    let kst = if inp.hoop_pitch > 0.0 {
        140.0 * inp.hoop_area.max(0.0) / (inp.db1 * inp.hoop_pitch)
    } else {
        0.0
    };
    (alpha_t * ((0.085 * b1 + 0.10) * inp.fc.sqrt() + kst)).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plastic_nu0_and_nu() {
        // ν0 = 0.7 − 24/200 = 0.58。
        assert!((plastic_nu0(24.0) - (0.7 - 24.0 / 200.0)).abs() < 1e-12);
        // Rp=0 → ν = ν0。
        assert!((plastic_nu(24.0, 0.0) - plastic_nu0(24.0)).abs() < 1e-12);
        // Rp=0.02 → (1−0.3)=0.7 倍。
        assert!((plastic_nu(24.0, 0.02) - 0.7 * plastic_nu0(24.0)).abs() < 1e-12);
        // Rp=0.05 → 0.25 倍（両分岐の境界で連続）。
        assert!((plastic_nu(24.0, 0.05) - 0.25 * plastic_nu0(24.0)).abs() < 1e-12);
        // Rp=0.1（>0.05）→ 0.25 倍で頭打ち。
        assert!((plastic_nu(24.0, 0.1) - 0.25 * plastic_nu0(24.0)).abs() < 1e-12);
    }

    #[test]
    fn test_plastic_cot_phi() {
        assert!((plastic_cot_phi(0.0) - 2.0).abs() < 1e-12);
        assert!((plastic_cot_phi(0.01) - (2.0 - 50.0 * 0.01)).abs() < 1e-12); // 1.5
        assert!((plastic_cot_phi(0.02) - 1.0).abs() < 1e-12); // 境界で 1.0
        assert!((plastic_cot_phi(0.05) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_plastic_k1_matches_handcalc() {
        // L/D=2.0 → k1 = (√5 − 2)/2。
        let k1 = plastic_k1(1200.0, 600.0);
        let hand = ((2.0_f64 * 2.0 + 1.0).sqrt() - 2.0) / 2.0;
        assert!((k1 - hand).abs() < 1e-12, "k1={k1} vs {hand}");
        // 不正入力。
        assert_eq!(plastic_k1(1200.0, 0.0), 0.0);
    }

    #[test]
    fn test_plastic_k2_capped_at_one() {
        let nu = plastic_nu0(24.0);
        // 過大な pw·σwy → k2=1.0 にクランプ。
        assert!((plastic_k2(0.1, 1000.0, nu, 24.0) - 1.0).abs() < 1e-12);
        // 通常値。
        let k2 = plastic_k2(0.003, 295.0, nu, 24.0);
        assert!((k2 - (2.0 * 0.003 * 295.0 / (nu * 24.0))).abs() < 1e-12);
        assert!(k2 < 1.0);
    }

    fn sample_qsu() -> RcPlasticShearInput {
        RcPlasticShearInput {
            b: 400.0,
            d_full: 600.0,
            jt: 7.0 * 530.0 / 8.0,
            pw: 0.003,
            sigma_wy: 295.0,
            l_clear: 3000.0,
            fc: 24.0,
            rp: 0.0,
            lightweight: false,
            nu0_override: None,
        }
    }

    #[test]
    fn test_rc_shear_qsu_plastic_nu0_override() {
        // 高強度せん断補強筋の製品別 ν0（785/685 級 0.7·(0.7−Fc/200)）を
        // 与えた場合、標準 ν0=0.7−Fc/200 の結果と一致しないこと、および
        // 手計算と整合すること。なお 1275 級の 0.7·(1.0−Fc/140) は恒等的に
        // 標準式と一致する（0.7/140=1/200）ため差は生じない。
        let base = sample_qsu();
        let nu0 = 0.7 * (0.7 - 24.0 / 200.0);
        let over = RcPlasticShearInput {
            nu0_override: Some(nu0),
            ..base
        };
        let q_base = rc_shear_qsu_plastic(&base);
        let q_over = rc_shear_qsu_plastic(&over);
        assert!(q_base > 0.0 && q_over > 0.0);
        assert!((q_base - q_over).abs() > 1.0, "ν0 上書きが反映されること");
        // 手計算（Rp=0: cotφ=2.0, ν=ν0_override）。
        let cot = 2.0_f64;
        let ld: f64 = 3000.0 / 600.0;
        let k1 = ((ld * ld + 1.0).sqrt() - ld) / 2.0;
        let pw_sigma = (0.003_f64 * 295.0).min(nu0 * 24.0 / 2.0);
        let k2 = (2.0 * 0.003 * 295.0 / (nu0 * 24.0)).min(1.0);
        let truss = 400.0 * (7.0 * 530.0 / 8.0) * pw_sigma * cot;
        let arch = k1 * (1.0 - k2) * 400.0 * 600.0 * nu0 * 24.0;
        assert!((q_over - (truss + arch)).abs() / (truss + arch) < 1e-12);
    }

    #[test]
    fn test_rc_shear_qsu_plastic_matches_handcalc() {
        let inp = sample_qsu();
        let qsu = rc_shear_qsu_plastic(&inp);
        // 手計算（Rp=0: cotφ=2.0, ν=ν0）。
        let nu = 0.7 - 24.0 / 200.0;
        let cot = 2.0_f64;
        let ld: f64 = 3000.0 / 600.0;
        let k1 = ((ld * ld + 1.0).sqrt() - ld) / 2.0;
        let pw_sigma = (0.003_f64 * 295.0).min(nu * 24.0 / 2.0);
        let k2 = (2.0 * 0.003 * 295.0 / (nu * 24.0)).min(1.0);
        let truss = 400.0 * (7.0 * 530.0 / 8.0) * pw_sigma * cot;
        let arch = k1 * (1.0 - k2) * 400.0 * 600.0 * nu * 24.0;
        let hand = truss + arch;
        assert!((qsu - hand).abs() < 1e-3, "Qsu={qsu} vs {hand}");
        assert!(qsu > 0.0);
    }

    #[test]
    fn test_rc_shear_qsu_plastic_rp_reduces() {
        // Rp を増やすと ν・cotφ が下がり Qsu は減少する。
        let mut inp = sample_qsu();
        let q0 = rc_shear_qsu_plastic(&inp);
        inp.rp = 0.03;
        let q_rp = rc_shear_qsu_plastic(&inp);
        assert!(q_rp < q0, "Qsu(Rp=0.03)={q_rp} should be < Qsu(0)={q0}");
    }

    #[test]
    fn test_rc_shear_qsu_plastic_lightweight_09() {
        let mut inp = sample_qsu();
        let q_std = rc_shear_qsu_plastic(&inp);
        inp.lightweight = true;
        let q_lw = rc_shear_qsu_plastic(&inp);
        assert!(
            (q_lw - 0.9 * q_std).abs() < 1e-6,
            "lw={q_lw} vs {}",
            0.9 * q_std
        );
    }

    #[test]
    fn test_rc_shear_qsu_plastic_invalid_zero() {
        let mut bad = sample_qsu();
        bad.fc = 0.0;
        assert_eq!(rc_shear_qsu_plastic(&bad), 0.0);
        bad = sample_qsu();
        bad.b = 0.0;
        assert_eq!(rc_shear_qsu_plastic(&bad), 0.0);
        bad = sample_qsu();
        bad.jt = 0.0;
        assert_eq!(rc_shear_qsu_plastic(&bad), 0.0);
    }

    #[test]
    fn test_rc_shear_qsu_plastic_pw_sigma_capped() {
        // pw·σwy ≤ ν·Fc/2 の制約: 過大な補強で頭打ちになり、k2=1（アーチ項 0）。
        let mut inp = sample_qsu();
        inp.pw = 0.05;
        inp.sigma_wy = 1275.0;
        let qsu = rc_shear_qsu_plastic(&inp);
        // トラス項のみ（アーチ項 0）: Qsu = b·jt·(ν·Fc/2)·cotφ。
        let nu = 0.7 - 24.0 / 200.0;
        let truss = 400.0 * (7.0 * 530.0 / 8.0) * (nu * 24.0 / 2.0) * 2.0;
        assert!(
            (qsu - truss).abs() < 1e-3,
            "Qsu={qsu} vs truss-only {truss}"
        );
    }

    #[test]
    fn test_bond_split_ratio_matches_handcalc() {
        // b=400, db1=25, N=4, Cs=Cb=40。
        let b1 = bond_split_ratio(400.0, 25.0, 4, 40.0, 40.0);
        let bvi = 3.0_f64.sqrt() * (2.0 * 40.0 / 25.0 + 1.0);
        let bci = 2.0_f64.sqrt() * ((40.0 + 40.0) / 25.0 - 1.0);
        let bsi = 400.0 / (4.0 * 25.0) - 1.0;
        let hand = bvi.min(bci).min(bsi);
        assert!((b1 - hand).abs() < 1e-9, "b1={b1} vs {hand}");
        assert_eq!(bond_split_ratio(400.0, 0.0, 4, 40.0, 40.0), 0.0);
    }

    fn sample_bond() -> BondStrengthInput {
        BondStrengthInput {
            fc: 24.0,
            b: 400.0,
            db1: 25.0,
            n_bars: 4,
            cover_side: 40.0,
            cover_bottom: 40.0,
            hoop_area: 2.0 * std::f64::consts::PI / 4.0 * 10.0 * 10.0,
            hoop_pitch: 100.0,
            pw: 0.003,
            top_bar: false,
        }
    }

    #[test]
    fn test_bond_reliable_strength_matches_handcalc() {
        let inp = sample_bond();
        let tau = bond_reliable_strength_deformed(&inp);
        let b1 = bond_split_ratio(400.0, 25.0, 4, 40.0, 40.0);
        let kst = 140.0 * (2.0 * std::f64::consts::PI / 4.0 * 100.0) / (25.0 * 100.0);
        let hand = 1.0 * ((0.085 * b1 + 0.10) * 24.0_f64.sqrt() + kst);
        assert!((tau - hand).abs() < 1e-6, "τbu={tau} vs {hand}");
        assert!(tau > 0.0);
    }

    #[test]
    fn test_bond_reliable_strength_top_bar_reduced() {
        let mut inp = sample_bond();
        let tau_other = bond_reliable_strength_deformed(&inp);
        inp.top_bar = true;
        let tau_top = bond_reliable_strength_deformed(&inp);
        // 上端筋は αt = 0.75 − Fc/400 < 1.0 倍で低減。
        assert!(tau_top < tau_other, "top={tau_top} other={tau_other}");
    }

    #[test]
    fn test_rc_shear_qbu_bond_matches_handcalc() {
        let tau_bu = bond_reliable_strength_deformed(&sample_bond());
        let sum_phi = 4.0 * std::f64::consts::PI * 25.0; // 4-D25 の周長和
        let inp = RcBondSplitInput {
            b: 400.0,
            d_full: 600.0,
            jt: 7.0 * 530.0 / 8.0,
            tau_bu,
            sum_phi,
            l_clear: 3000.0,
            fc: 24.0,
            rp: 0.0,
            lightweight: false,
        };
        let qbu = rc_shear_qbu_bond(&inp);
        let nu = 0.7 - 24.0 / 200.0;
        let ld: f64 = 3000.0 / 600.0;
        let k1 = ((ld * ld + 1.0).sqrt() - ld) / 2.0;
        let k3 = (2.0 * tau_bu * sum_phi / (400.0 * nu * 24.0)).min(1.0);
        let bond = (7.0 * 530.0 / 8.0) * tau_bu * sum_phi;
        let arch = k1 * (1.0 - k3) * 400.0 * 600.0 * nu * 24.0;
        let hand = bond + arch;
        assert!((qbu - hand).abs() < 1e-3, "Qbu={qbu} vs {hand}");
        assert!(qbu > 0.0);
    }

    #[test]
    fn test_rc_shear_qbu_bond_invalid_zero() {
        let inp = RcBondSplitInput {
            b: 0.0,
            d_full: 600.0,
            jt: 400.0,
            tau_bu: 3.0,
            sum_phi: 300.0,
            l_clear: 3000.0,
            fc: 24.0,
            rp: 0.0,
            lightweight: false,
        };
        assert_eq!(rc_shear_qbu_bond(&inp), 0.0);
    }
}
