//! 鉄筋コンクリート造梁・柱の**靭性指針式による終局せん断信頼強度 `Vu`**
//! （日本建築学会「鉄筋コンクリート造建物の靭性保証型耐震設計指針・同解説」(1997)。
//! 部材耐力式として「靭性保証型設計指針」を選択した場合の経路）。
//!
//! # 位置付け
//! [`super::rc_shear`] の塑性理論式 `Qsu`（藤井・森田式系、「終局強度型設計指針」）に対し、
//! 本モジュールは AIJ「鉄筋コンクリート造建物の靭性保証型耐震設計指針・同解説」(1997) の
//! せん断信頼強度式（トラス機構＋アーチ機構の 3 式 min）を実装する。本実装では
//! 部材耐力式として「靭性保証型設計指針」を選択した場合に本式を採用する。
//!
//! # 準拠する式（AIJ 靭性保証型耐震設計指針 P.142-162, 1997.7）
//! ```text
//! Vu  = min(Vu1, Vu2, Vu3)
//! Vu1 = μ·pwe·σwy·be·je + (ν·σB − 5·pwe·σwy/λ)·(b·D/2)·tanθ        (6.4.1)
//! Vu2 = (λ·ν·σB + pwe·σwy)/3 · be·je                               (6.4.2)
//! Vu3 = (λ·ν·σB/2)·be·je                                           (6.4.3)
//! μ   = 2 − 20·Rp                        （トラス機構の角度係数）
//! ν   = (1 − 20·Rp)·ν0,  ν0 = 0.7 − σB/200
//! λ   = 1 − s/(2·je) − bs/(4·je),  bs = be/(Ns+1)                  (6.4.8/6.4.9)
//! tanθ= { 0                       （引張軸力を受ける柱）
//!         0.9·D/(2L)              （L/D ≥ 1.5, 6.4.10）
//!         (√(L²+D²) − L)/D        （L/D < 1.5, 6.4.11） }
//! ```
//!
//! # 原典照合の要点（`tanθ` の符号）
//! 一部の二次資料では (6.4.11) が数式抽出上 `(√(L²−D²)−L)/D` と表示されるが、
//! これは **`√(L²+D²)` が正しい**（`L²−D²` では L>D で √(L²−D²)<L となり tanθ<0 と
//! なって物理的に破綻する）。境界 `L/D=1.5` において (6.4.10) は `0.9/(2·1.5)=0.300`、
//! (6.4.11) を `+` で評価すると `(√(1.5²+1²)−1.5)=0.303` となり両式が連続することから
//! 確定した（[`arch_tan_theta`] のテスト参照）。アーチ機構の圧縮束角度 θ の幾何は
//! `tanθ = √((L/D)²+1) − (L/D)`（束が部材端の圧縮域を結ぶ形状）に一致する。

use super::rc_shear::plastic_nu0;

/// トラス機構の角度係数 `μ = 2 − 20·Rp`（AIJ 靭性指針 6.4）。
///
/// `Rp ≤ 0`（塑性化前）は `μ = 2.0`（`Rp→0` の極限）とする。トラス斜材角 ≤ 45°
/// （`cotφ = μ ≥ 1`）に対応するよう下限 1.0 でクランプする（[`super::rc_shear::plastic_cot_phi`]
/// と同じ扱い。`Rp=0.05` で `μ=1.0`）。
pub fn ductility_mu(rp: f64) -> f64 {
    if rp <= 0.0 {
        2.0
    } else {
        (2.0 - 20.0 * rp).max(1.0)
    }
}

/// コンクリート圧縮強度の有効係数 `ν = (1 − 20·Rp)·ν0`（AIJ 靭性指針 6.4）。
///
/// `ν0 = 0.7 − σB/200`（[`super::rc_shear::plastic_nu0`] と共通）。`Rp ≤ 0`（塑性化前）は
/// `ν = ν0`。`(1 − 20·Rp)` は `Rp=0.05` で 0 に達するため、下限 0 でクランプする
/// （靭性指針式は塑性理論式の `0.25·ν0` 頭打ちを持たない。有効範囲は概ね `Rp ≤ 0.05`）。
pub fn ductility_nu(fc: f64, rp: f64) -> f64 {
    let nu0 = plastic_nu0(fc);
    let factor = if rp <= 0.0 {
        1.0
    } else {
        (1.0 - 20.0 * rp).max(0.0)
    };
    (factor * nu0).max(0.0)
}

/// アーチ機構の圧縮束角度のタンジェント `tanθ`（AIJ 靭性指針 6.4.10/6.4.11）。
///
/// ```text
/// tanθ = { 0                 （引張軸力を受ける柱: tensile_axial=true）
///          0.9·D/(2L)        （L/D ≥ 1.5, 6.4.10）
///          (√(L²+D²) − L)/D  （L/D < 1.5, 6.4.11） }
/// ```
/// `L`: クリアスパン長さ、`D`: 部材せい [mm]。`D ≤ 0` または `L ≤ 0` の不正入力は 0。
///
/// (6.4.11) の平方根内は原典照合により `L²+D²`（モジュールドキュメント参照）。境界
/// `L/D=1.5` で (6.4.10) と連続する（テスト `test_arch_tan_theta_continuity`）。
pub fn arch_tan_theta(l_clear: f64, d_full: f64, tensile_axial: bool) -> f64 {
    if tensile_axial {
        return 0.0;
    }
    if d_full <= 0.0 || l_clear <= 0.0 {
        return 0.0;
    }
    let ld = l_clear / d_full;
    if ld >= 1.5 {
        0.9 * d_full / (2.0 * l_clear)
    } else {
        ((l_clear * l_clear + d_full * d_full).sqrt() - l_clear) / d_full
    }
}

/// トラス機構の有効係数 `λ = 1 − s/(2·je) − bs/(4·je)`（AIJ 靭性指針 6.4.8）。
///
/// `bs = be/(Ns+1)`（6.4.9、`Ns`＝中子筋の本数）。`s`＝横補強筋間隔、`je`＝トラス機構
/// 有効せい、`be`＝トラス機構有効幅 [mm]。`je ≤ 0` の不正入力は 0 を返す。結果は
/// `[0, 1]` にクランプする（横補強筋が密なほど 1 に近づく）。
pub fn truss_lambda(s: f64, je: f64, be: f64, n_s: u32) -> f64 {
    if je <= 0.0 {
        return 0.0;
    }
    let bs = be.max(0.0) / (n_s as f64 + 1.0);
    (1.0 - s.max(0.0) / (2.0 * je) - bs / (4.0 * je)).clamp(0.0, 1.0)
}

/// 靭性指針式による終局せん断信頼強度 `Vu` の算定入力（AIJ 靭性指針 6.4）。
#[derive(Clone, Copy, Debug)]
pub struct RcDuctilityShearInput {
    /// 部材幅 b [mm]（アーチ機構の断面 `b·D/2` に用いる）。
    pub b: f64,
    /// 部材せい D [mm]（アーチ機構・tanθ に用いる）。
    pub d_full: f64,
    /// トラス機構に関与する断面の有効幅 be [mm]（外側横補強筋の芯々間隔）。
    pub be: f64,
    /// トラス機構に関与する断面の有効せい je [mm]。
    pub je: f64,
    /// 有効横補強筋比 pwe（小数、= aw/(be·s)）。トラス機構に用いる。
    pub pwe: f64,
    /// 横補強筋の信頼強度 σwy [N/mm²]。
    pub sigma_wy: f64,
    /// 横補強筋の間隔 s [mm]（λ に用いる）。
    pub s: f64,
    /// 中子筋の本数 Ns（λ の bs=be/(Ns+1) に用いる）。
    pub n_s: u32,
    /// クリアスパン長さ L [mm]（tanθ に用いる）。
    pub l_clear: f64,
    /// コンクリートの圧縮強度 σB [N/mm²]。
    pub fc: f64,
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（μ・ν に用いる）。
    pub rp: f64,
    /// 引張軸力を受ける柱の場合 true（tanθ=0、アーチ機構を無効化）。
    pub tensile_axial: bool,
    /// 軽量コンクリートを使用する場合 true（せん断終局耐力を 0.9 倍に低減）。
    pub lightweight: bool,
}

/// 靭性指針式による終局せん断信頼強度 `Vu = min(Vu1, Vu2, Vu3)` [N]（AIJ 靭性指針 6.4）。
///
/// ```text
/// Vu1 = μ·pwe·σwy·be·je + (ν·σB − 5·pwe·σwy/λ)·(b·D/2)·tanθ        (6.4.1)
/// Vu2 = (λ·ν·σB + pwe·σwy)/3 · be·je                               (6.4.2)
/// Vu3 = (λ·ν·σB/2)·be·je                                           (6.4.3)
/// ```
/// - Vu1 はトラス機構＋アーチ機構、Vu2・Vu3 はコンクリート圧壊で頭打ちの候補。
/// - (6.4.1) 第 2 項（アーチ機構）の応力 `ν·σB − 5·pwe·σwy/λ` は負にならないよう
///   下限 0 でクランプする（横補強筋が過密でアーチ寄与が消える場合。塑性理論式で
///   `k2 ≤ 1` によりアーチ項を 0 以上に保つのと同型）。
/// - `lightweight` が true の場合、算定値を 0.9 倍に低減する（共通事項）。
/// - 不正入力（b・D・be・je・Fc のいずれかが 0 以下）は 0.0 を返す。
pub fn rc_shear_vu_ductility(inp: &RcDuctilityShearInput) -> f64 {
    if inp.b <= 0.0 || inp.d_full <= 0.0 || inp.be <= 0.0 || inp.je <= 0.0 || inp.fc <= 0.0 {
        return 0.0;
    }
    let mu = ductility_mu(inp.rp);
    let nu = ductility_nu(inp.fc, inp.rp);
    let lambda = truss_lambda(inp.s, inp.je, inp.be, inp.n_s);
    let tan_theta = arch_tan_theta(inp.l_clear, inp.d_full, inp.tensile_axial);

    let pw_sigma = inp.pwe.max(0.0) * inp.sigma_wy.max(0.0);
    let nu_sb = nu * inp.fc;

    // (6.4.1) アーチ項の応力（λ>0 のときのみ補強筋控除、負はクランプ）。
    let arch_stress = if lambda > 0.0 {
        (nu_sb - 5.0 * pw_sigma / lambda).max(0.0)
    } else {
        nu_sb
    };
    let vu1 =
        mu * pw_sigma * inp.be * inp.je + arch_stress * (inp.b * inp.d_full / 2.0) * tan_theta;
    let vu2 = (lambda * nu_sb + pw_sigma) / 3.0 * inp.be * inp.je;
    let vu3 = (lambda * nu_sb / 2.0) * inp.be * inp.je;

    let vu = vu1.min(vu2).min(vu3).max(0.0);
    if inp.lightweight {
        0.9 * vu
    } else {
        vu
    }
}

/// 部材単位長さあたりに負担できる付着力 `Tx` [N/mm]（AIJ 靭性指針 6.8.16）。
///
/// ```text
/// Tx = τbu·Σφ1 + τbu2·Σφ2                     （降伏ヒンジを計画しない部材, 6.8.16a）
/// Tx = (1 − 10·Rp)·(τbu·Σφ1 + τbu2·Σφ2)       （降伏ヒンジを計画する部材,   6.8.16b）
/// ```
/// `yield_hinge=true` で (6.8.16b)（ヒンジ域の付着劣化を `(1−10·Rp)` で反映、下限 0）。
/// `τbu`・`τbu2` は 1・2 段目主筋の付着信頼強度、`Σφ1`・`Σφ2` はその周長合計。
pub fn bond_force_tx(
    tau_bu: f64,
    sum_phi1: f64,
    tau_bu2: f64,
    sum_phi2: f64,
    rp: f64,
    yield_hinge: bool,
) -> f64 {
    let base = tau_bu.max(0.0) * sum_phi1.max(0.0) + tau_bu2.max(0.0) * sum_phi2.max(0.0);
    if yield_hinge {
        (1.0 - 10.0 * rp).max(0.0) * base
    } else {
        base
    }
}

/// 付着破壊の影響を考慮した終局せん断信頼強度 `Vbu` の算定入力（AIJ 靭性指針 6.8）。
#[derive(Clone, Copy, Debug)]
pub struct RcVbuInput {
    /// 部材幅 b [mm]。
    pub b: f64,
    /// 部材せい D [mm]。
    pub d_full: f64,
    /// トラス機構に関与する断面の有効幅 be [mm]。
    pub be: f64,
    /// トラス機構に関与する断面の有効せい je [mm]。
    pub je: f64,
    /// 1 段目主筋の付着信頼強度 τbu [N/mm²]。
    pub tau_bu: f64,
    /// 1 段目主筋の周長合計 Σφ1 [mm]。
    pub sum_phi1: f64,
    /// 2 段目主筋の付着信頼強度 τbu2 [N/mm²]（無い場合 0）。
    pub tau_bu2: f64,
    /// 2 段目主筋の周長合計 Σφ2 [mm]（無い場合 0）。
    pub sum_phi2: f64,
    /// 横補強筋の間隔 s [mm]（λ に用いる）。
    pub s: f64,
    /// 中子筋の本数 Ns（λ に用いる）。
    pub n_s: u32,
    /// クリアスパン長さ L [mm]（tanθ に用いる）。
    pub l_clear: f64,
    /// コンクリートの圧縮強度 σB [N/mm²]。
    pub fc: f64,
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν・Tx に用いる）。
    pub rp: f64,
    /// 引張軸力を受ける柱の場合 true（tanθ=0）。
    pub tensile_axial: bool,
    /// 降伏ヒンジを計画する部材の場合 true（Tx に (1−10·Rp) を乗じる, 6.8.16b）。
    pub yield_hinge: bool,
    /// 軽量コンクリートを使用する場合 true（0.9 倍に低減）。
    pub lightweight: bool,
}

/// 付着破壊の影響を考慮した終局せん断信頼強度 `Vbu = min(Vbu1, Vbu2)` [N]
/// （AIJ 靭性指針 6.8）。
///
/// ```text
/// Vbu1 = Tx·je + (ν·σB − 2.5·Tx/(λ·be))·(b·D/2)·tanθ        (6.8.14)
/// Vbu2 = (λ·ν·σB/2)·be·je                                    (6.8.15)
/// ```
/// - トラス機構をせん断補強筋（[`rc_shear_vu_ductility`] の `pwe·σwy`）ではなく付着力
///   `Tx`（[`bond_force_tx`]）で置き換えた式。付着割裂が支配する場合のせん断信頼強度。
/// - Vbu2 は [`rc_shear_vu_ductility`] の Vu3（6.4.3）と同型（コンクリート圧壊の頭打ち）。
/// - (6.8.14) 第 2 項の応力 `ν·σB − 2.5·Tx/(λ·be)` は負にならないよう下限 0 でクランプ。
/// - `lightweight` が true の場合 0.9 倍に低減する。
/// - 不正入力（b・D・be・je・Fc のいずれかが 0 以下）は 0.0 を返す。
pub fn rc_shear_vbu_ductility(inp: &RcVbuInput) -> f64 {
    if inp.b <= 0.0 || inp.d_full <= 0.0 || inp.be <= 0.0 || inp.je <= 0.0 || inp.fc <= 0.0 {
        return 0.0;
    }
    let nu = ductility_nu(inp.fc, inp.rp);
    let lambda = truss_lambda(inp.s, inp.je, inp.be, inp.n_s);
    let tan_theta = arch_tan_theta(inp.l_clear, inp.d_full, inp.tensile_axial);
    let tx = bond_force_tx(
        inp.tau_bu,
        inp.sum_phi1,
        inp.tau_bu2,
        inp.sum_phi2,
        inp.rp,
        inp.yield_hinge,
    );
    let nu_sb = nu * inp.fc;

    let arch_stress = if lambda > 0.0 {
        (nu_sb - 2.5 * tx / (lambda * inp.be)).max(0.0)
    } else {
        nu_sb
    };
    let vbu1 = tx * inp.je + arch_stress * (inp.b * inp.d_full / 2.0) * tan_theta;
    let vbu2 = (lambda * nu_sb / 2.0) * inp.be * inp.je;

    let vbu = vbu1.min(vbu2).max(0.0);
    if inp.lightweight {
        0.9 * vbu
    } else {
        vbu
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ductility_mu() {
        assert!((ductility_mu(0.0) - 2.0).abs() < 1e-12);
        assert!((ductility_mu(0.02) - (2.0 - 20.0 * 0.02)).abs() < 1e-12); // 1.6
        assert!((ductility_mu(0.05) - 1.0).abs() < 1e-12); // 境界で 1.0
        assert!((ductility_mu(0.1) - 1.0).abs() < 1e-12); // 下限 1.0
        assert!((ductility_mu(-0.01) - 2.0).abs() < 1e-12); // Rp≤0 → 2.0
    }

    #[test]
    fn test_ductility_nu() {
        // Rp=0 → ν = ν0 = 0.7 − 24/200 = 0.58。
        assert!((ductility_nu(24.0, 0.0) - plastic_nu0(24.0)).abs() < 1e-12);
        // Rp=0.02 → (1−0.4)=0.6 倍。
        assert!((ductility_nu(24.0, 0.02) - 0.6 * plastic_nu0(24.0)).abs() < 1e-12);
        // Rp=0.05 → 0 に達する。
        assert!(ductility_nu(24.0, 0.05).abs() < 1e-12);
        // Rp=0.1（>0.05）→ 0 でクランプ。
        assert!(ductility_nu(24.0, 0.1).abs() < 1e-12);
    }

    #[test]
    fn test_arch_tan_theta_continuity() {
        // 引張軸力柱 → 0。
        assert_eq!(arch_tan_theta(3000.0, 600.0, true), 0.0);
        // 境界 L/D=1.5 で (6.4.10) と (6.4.11) が連続することを確認。
        let d = 600.0_f64;
        let l = 1.5 * d; // L/D = 1.5（(6.4.10) 側）
        let t_640 = arch_tan_theta(l, d, false);
        // (6.4.11) を境界直下で評価。
        let l_below = 1.4999 * d;
        let t_641 = arch_tan_theta(l_below, d, false);
        assert!(
            (t_640 - t_641).abs() < 5e-3,
            "境界不連続: (6.4.10)={t_640} vs (6.4.11)={t_641}"
        );
        // (6.4.10) の値: 0.9·D/(2L) = 0.9/(2·1.5) = 0.300。
        assert!((t_640 - 0.9 / (2.0 * 1.5)).abs() < 1e-12);
        // (6.4.11) は √(L²+D²) 版（正値）。L/D=1.2 で手計算照合。
        let l12 = 1.2 * d;
        let t = arch_tan_theta(l12, d, false);
        let hand = ((l12 * l12 + d * d).sqrt() - l12) / d;
        assert!((t - hand).abs() < 1e-12 && t > 0.0, "tanθ={t} vs {hand}");
        // 不正入力。
        assert_eq!(arch_tan_theta(0.0, 600.0, false), 0.0);
        assert_eq!(arch_tan_theta(3000.0, 0.0, false), 0.0);
    }

    #[test]
    fn test_truss_lambda_handcalc() {
        // s=100, je=525, be=350, Ns=1 → bs=350/2=175。
        let lam = truss_lambda(100.0, 525.0, 350.0, 1);
        let hand = 1.0 - 100.0 / (2.0 * 525.0) - (350.0 / 2.0) / (4.0 * 525.0);
        assert!((lam - hand).abs() < 1e-12, "λ={lam} vs {hand}");
        assert!((0.0..=1.0).contains(&lam));
        // 不正入力。
        assert_eq!(truss_lambda(100.0, 0.0, 350.0, 1), 0.0);
    }

    fn sample() -> RcDuctilityShearInput {
        RcDuctilityShearInput {
            b: 400.0,
            d_full: 600.0,
            be: 350.0,
            je: 7.0 * 530.0 / 8.0,
            pwe: 0.003,
            sigma_wy: 295.0,
            s: 100.0,
            n_s: 1,
            l_clear: 3000.0,
            fc: 24.0,
            rp: 0.0,
            tensile_axial: false,
            lightweight: false,
        }
    }

    #[test]
    fn test_rc_shear_vu_ductility_matches_handcalc() {
        let inp = sample();
        let vu = rc_shear_vu_ductility(&inp);

        // 手計算（Rp=0: μ=2.0, ν=ν0）。
        let mu = 2.0_f64;
        let nu: f64 = 0.7 - 24.0 / 200.0;
        let je: f64 = 7.0 * 530.0 / 8.0;
        let be: f64 = 350.0;
        let lambda = 1.0 - 100.0 / (2.0 * je) - (be / 2.0) / (4.0 * je);
        let ld = 3000.0 / 600.0; // = 5.0 ≥ 1.5 → (6.4.10)
        let tan_theta = 0.9 * 600.0 / (2.0 * 3000.0);
        let _ = ld;
        let pw_sigma = 0.003 * 295.0;
        let nu_sb = nu * 24.0;
        let arch_stress = (nu_sb - 5.0 * pw_sigma / lambda).max(0.0);
        let vu1 = mu * pw_sigma * be * je + arch_stress * (400.0 * 600.0 / 2.0) * tan_theta;
        let vu2 = (lambda * nu_sb + pw_sigma) / 3.0 * be * je;
        let vu3 = (lambda * nu_sb / 2.0) * be * je;
        let hand = vu1.min(vu2).min(vu3);
        assert!((vu - hand).abs() < 1e-3, "Vu={vu} vs {hand}");
        assert!(vu > 0.0);
    }

    #[test]
    fn test_rc_shear_vu_ductility_min_selects_smallest() {
        // 通常配筋では Vu3（コンクリート圧壊の下限式）が支配することが多い。
        let inp = sample();
        let vu = rc_shear_vu_ductility(&inp);
        let nu: f64 = 0.7 - 24.0 / 200.0;
        let je: f64 = 7.0 * 530.0 / 8.0;
        let be: f64 = 350.0;
        let lambda = 1.0 - 100.0 / (2.0 * je) - (be / 2.0) / (4.0 * je);
        let vu3 = (lambda * nu * 24.0 / 2.0) * be * je;
        assert!(vu <= vu3 + 1e-6, "Vu={vu} は Vu3={vu3} 以下のはず");
    }

    #[test]
    fn test_rc_shear_vu_ductility_lightweight_09() {
        let mut inp = sample();
        let v_std = rc_shear_vu_ductility(&inp);
        inp.lightweight = true;
        let v_lw = rc_shear_vu_ductility(&inp);
        assert!(
            (v_lw - 0.9 * v_std).abs() < 1e-6,
            "lw={v_lw} vs {}",
            0.9 * v_std
        );
    }

    #[test]
    fn test_rc_shear_vu_ductility_tensile_axial_no_arch() {
        // 引張軸力柱は tanθ=0 → Vu1 のアーチ項が消え、トラス項のみ。
        let mut inp = sample();
        inp.tensile_axial = true;
        let vu = rc_shear_vu_ductility(&inp);
        assert!(vu > 0.0);
        // アーチ項が消えても Vu2/Vu3 は tanθ 非依存なので min は不変または低下。
        let vu_ref = rc_shear_vu_ductility(&sample());
        assert!(vu <= vu_ref + 1e-6);
    }

    #[test]
    fn test_rc_shear_vu_ductility_invalid_zero() {
        let mut bad = sample();
        bad.fc = 0.0;
        assert_eq!(rc_shear_vu_ductility(&bad), 0.0);
        bad = sample();
        bad.be = 0.0;
        assert_eq!(rc_shear_vu_ductility(&bad), 0.0);
        bad = sample();
        bad.d_full = 0.0;
        assert_eq!(rc_shear_vu_ductility(&bad), 0.0);
    }

    #[test]
    fn test_bond_force_tx() {
        // 降伏ヒンジ無: Tx = τbu·Σφ1 + τbu2·Σφ2。
        let tx = bond_force_tx(3.0, 300.0, 2.0, 150.0, 0.02, false);
        assert!((tx - (3.0 * 300.0 + 2.0 * 150.0)).abs() < 1e-9);
        // 降伏ヒンジ有: (1−10·Rp) 倍（Rp=0.02 → 0.8 倍）。
        let tx_h = bond_force_tx(3.0, 300.0, 2.0, 150.0, 0.02, true);
        assert!((tx_h - 0.8 * (3.0 * 300.0 + 2.0 * 150.0)).abs() < 1e-9);
        // Rp=0.1 → (1−1.0)=0 でクランプ。
        assert!(bond_force_tx(3.0, 300.0, 0.0, 0.0, 0.1, true).abs() < 1e-9);
    }

    fn sample_vbu() -> RcVbuInput {
        RcVbuInput {
            b: 400.0,
            d_full: 600.0,
            be: 350.0,
            je: 7.0 * 530.0 / 8.0,
            tau_bu: 3.0,
            sum_phi1: 4.0 * std::f64::consts::PI * 25.0, // 4-D25 の周長合計
            tau_bu2: 0.0,
            sum_phi2: 0.0,
            s: 100.0,
            n_s: 1,
            l_clear: 3000.0,
            fc: 24.0,
            rp: 0.0,
            tensile_axial: false,
            yield_hinge: false,
            lightweight: false,
        }
    }

    #[test]
    fn test_rc_shear_vbu_ductility_matches_handcalc() {
        let inp = sample_vbu();
        let vbu = rc_shear_vbu_ductility(&inp);
        let nu: f64 = 0.7 - 24.0 / 200.0;
        let je: f64 = 7.0 * 530.0 / 8.0;
        let be: f64 = 350.0;
        let lambda = 1.0 - 100.0 / (2.0 * je) - (be / 2.0) / (4.0 * je);
        let tan_theta = 0.9 * 600.0 / (2.0 * 3000.0); // L/D=5 ≥ 1.5
        let tx = 3.0 * (4.0 * std::f64::consts::PI * 25.0);
        let nu_sb = nu * 24.0;
        let arch_stress = (nu_sb - 2.5 * tx / (lambda * be)).max(0.0);
        let vbu1 = tx * je + arch_stress * (400.0 * 600.0 / 2.0) * tan_theta;
        let vbu2 = (lambda * nu_sb / 2.0) * be * je;
        let hand = vbu1.min(vbu2);
        assert!((vbu - hand).abs() < 1e-3, "Vbu={vbu} vs {hand}");
        assert!(vbu > 0.0);
    }

    #[test]
    fn test_rc_shear_vbu_ductility_yield_hinge_reduces() {
        // 降伏ヒンジ計画（Rp>0）は Tx 低減で Vbu が下がる（トラス項が支配する範囲）。
        let mut inp = sample_vbu();
        inp.rp = 0.03;
        let v_no = rc_shear_vbu_ductility(&inp);
        inp.yield_hinge = true;
        let v_hinge = rc_shear_vbu_ductility(&inp);
        assert!(v_hinge <= v_no + 1e-6, "hinge={v_hinge} no={v_no}");
    }

    #[test]
    fn test_rc_shear_vbu_ductility_invalid_zero() {
        let mut bad = sample_vbu();
        bad.fc = 0.0;
        assert_eq!(rc_shear_vbu_ductility(&bad), 0.0);
        bad = sample_vbu();
        bad.be = 0.0;
        assert_eq!(rc_shear_vbu_ductility(&bad), 0.0);
    }
}
