//! 風荷重の計算（RESP-D マニュアル「風荷重の計算」節）。
//!
//! 速度圧 q・風力係数 Cf から各層の風圧力・水平力を求める。
//! 数値は建築基準法施行令87条・平成12年建設省告示第1454号に相当する
//! 略算式（マニュアル記載のもの）を用いる。

/// 地表面粗度区分（告示1454号別表）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainRoughness {
    I,
    II,
    III,
    IV,
}

/// 地表面粗度区分ごとの Zb[m]・ZG[m]・α。
fn terrain_params(r: TerrainRoughness) -> (f64, f64, f64) {
    match r {
        TerrainRoughness::I => (5.0, 250.0, 0.10),
        TerrainRoughness::II => (5.0, 350.0, 0.15),
        TerrainRoughness::III => (5.0, 450.0, 0.20),
        TerrainRoughness::IV => (10.0, 550.0, 0.27),
    }
}

/// 構造骨組用ガスト影響係数 Gf（地表面粗度区分・H に応じた表。
/// 10m〜40m は直線補間）。`h_m` は建築物の高さ[m]。
fn gust_factor(r: TerrainRoughness, h_m: f64) -> f64 {
    let (lo, hi) = match r {
        TerrainRoughness::I => (2.0, 1.8),
        TerrainRoughness::II => (2.2, 2.0),
        TerrainRoughness::III => (2.5, 2.1),
        TerrainRoughness::IV => (3.1, 2.3),
    };
    if h_m <= 10.0 {
        lo
    } else if h_m >= 40.0 {
        hi
    } else {
        lo + (hi - lo) * (h_m - 10.0) / (40.0 - 10.0)
    }
}

/// 平均風速の高さ方向の分布を表す係数 Er。
fn er_factor(zb: f64, zg: f64, alpha: f64, h_m: f64) -> f64 {
    if h_m <= zb {
        1.7 * (zb / zg).powf(alpha)
    } else {
        1.7 * (h_m / zg).powf(alpha)
    }
}

/// 層区間 [z0, z1]（m、地盤面基準）における Kz の平均値。
///
/// - H≦Zb: 1.0（呼び出し側で判定済みの前提だが、念のためここでも判定する）。
/// - 区間が Zb 以下に収まる: (Zb/H)^(2α) 一定。
/// - 区間が Zb を超えて存在する: ∫(Z/H)^(2α)dZ / (z1−z0) の閉形式。
/// - 区間が Zb をまたぐ: Zb 以下部分は (Zb/H)^(2α) 一定、Zb 超部分は積分値として
///   区間全体で平均する（マニュアル「２つの式にまたがる場合も平均のKzを求めます」）。
fn kz_for_interval(zb: f64, h_m: f64, alpha: f64, z0: f64, z1: f64) -> f64 {
    if h_m <= zb {
        return 1.0;
    }
    let two_a1 = 2.0 * alpha + 1.0;
    let f = |z: f64| (z / h_m).powf(2.0 * alpha);
    let integral = |z: f64| (z / h_m).powf(two_a1) * h_m / two_a1;

    if z1 <= zb {
        f(zb)
    } else if z0 >= zb {
        (integral(z1) - integral(z0)) / (z1 - z0)
    } else {
        let below_len = zb - z0;
        let below_contrib = f(zb) * below_len;
        let above_contrib = integral(z1) - integral(zb);
        (below_contrib + above_contrib) / (z1 - z0)
    }
}

/// 風荷重の算定条件。
pub struct WindCfg {
    /// 基準風速 V0 [m/s]。
    pub v0: f64,
    /// 地表面粗度区分。
    pub roughness: TerrainRoughness,
    /// 風上壁面の外圧係数 Cpe（既定 0.8。kz を乗じて用いる）。
    pub cpe_windward: f64,
    /// 風下壁面の外圧係数 Cpe（既定 −0.4、高さによらず一定）。
    pub cpe_leeward: f64,
    /// 内圧係数 Cpi（指定による。風上・風下の合算では相殺されるため
    /// 現在の実装では合成 Cf の値に影響しないが、将来の片面評価用に保持する）。
    pub cpi: f64,
}

/// 風荷重を受ける1層分のデータ。`z_bottom`/`z_top` は地盤面(GL)基準の
/// 負担高さ区間[mm]（各層の節点が負担する範囲＝直下の層との中間から
/// 直上の層との中間まで。最上層は建物最上部の高さまで）。`width` は
/// その区間に対するX方向またはY方向の見付け幅[mm]。
pub struct WindStory {
    pub z_bottom: f64,
    pub z_top: f64,
    pub width: f64,
}

/// 風荷重の算定結果。
pub struct WindDistribution {
    /// 速度圧 q [N/m²]。
    pub q: f64,
    /// 係数 E = Er²・Gf。
    pub e: f64,
    /// 平均風速の高さ方向の分布係数 Er。
    pub er: f64,
    /// ガスト影響係数 Gf。
    pub gf: f64,
    /// 層ごとの Kz（風上壁面の外圧係数 0.8kz に用いる高さ方向分布係数）。
    pub kz: Vec<f64>,
    /// 層ごとの風圧力 [N/m²]（風上・風下の合算値。Cpi は相殺されるため
    /// 実質的な合成風力係数は `cpe_windward・kz − cpe_leeward` となる）。
    pub pressure: Vec<f64>,
    /// 層ごとの水平力 [N]（風圧力 × 見付面積）。
    pub force: Vec<f64>,
}

/// 建物高さ `h_mm`[mm] と各層の見付幅・負担高さ区間から、層ごとの
/// 風荷重（層水平力）を求める。
///
/// 風上壁面（Cpe=0.8kz）と風下壁面（Cpe=−0.4）が同時に作用するため、
/// 内圧係数 Cpi は合成の際に相殺される:
/// `Cf_total = (0.8・kz − Cpi) + (Cpi − (−0.4)) = 0.8・kz + 0.4`
/// （`cfg.cpi` は将来の片面評価のために保持するのみで、本関数の結果には
/// 影響しない）。
pub fn wind_forces(
    h_mm: f64,
    stories_bottom_to_top: &[WindStory],
    cfg: &WindCfg,
) -> WindDistribution {
    let h_m = h_mm / 1000.0;
    let (zb, zg, alpha) = terrain_params(cfg.roughness);
    let er = er_factor(zb, zg, alpha, h_m);
    let gf = gust_factor(cfg.roughness, h_m);
    let e = er * er * gf;
    let q = 0.6 * e * cfg.v0 * cfg.v0;

    let mut kz = Vec::with_capacity(stories_bottom_to_top.len());
    let mut pressure = Vec::with_capacity(stories_bottom_to_top.len());
    let mut force = Vec::with_capacity(stories_bottom_to_top.len());

    for s in stories_bottom_to_top {
        let z0 = s.z_bottom / 1000.0;
        let z1 = s.z_top / 1000.0;
        let k = kz_for_interval(zb, h_m, alpha, z0, z1);
        let cf_total = cfg.cpe_windward * k - cfg.cpe_leeward;
        let p = q * cf_total; // N/m^2
                              // 面積計算はモデルの長さ単位(mm)に合わせるため N/mm^2 に換算(×1e-6)する。
        let p_per_mm2 = p * 1e-6;
        let f = p_per_mm2 * s.width * (s.z_top - s.z_bottom);
        kz.push(k);
        pressure.push(p);
        force.push(f);
    }

    WindDistribution {
        q,
        e,
        er,
        gf,
        kz,
        pressure,
        force,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol * b.abs().max(1.0)
    }

    #[test]
    fn test_terrain_table() {
        assert_eq!(terrain_params(TerrainRoughness::I), (5.0, 250.0, 0.10));
        assert_eq!(terrain_params(TerrainRoughness::II), (5.0, 350.0, 0.15));
        assert_eq!(terrain_params(TerrainRoughness::III), (5.0, 450.0, 0.20));
        assert_eq!(terrain_params(TerrainRoughness::IV), (10.0, 550.0, 0.27));
    }

    #[test]
    fn test_gust_factor_boundaries_and_interpolation() {
        // H<=10m: 表(1)の値そのもの。
        assert!((gust_factor(TerrainRoughness::I, 10.0) - 2.0).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::II, 10.0) - 2.2).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::III, 10.0) - 2.5).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::IV, 10.0) - 3.1).abs() < 1e-12);
        // H>=40m: 表(3)の値そのもの。
        assert!((gust_factor(TerrainRoughness::I, 40.0) - 1.8).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::II, 40.0) - 2.0).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::III, 40.0) - 2.1).abs() < 1e-12);
        assert!((gust_factor(TerrainRoughness::IV, 40.0) - 2.3).abs() < 1e-12);
        // 中間高さは直線補間。III で H=20m: 2.5 + (2.1-2.5)*(20-10)/30 = 2.36667。
        let g = gust_factor(TerrainRoughness::III, 20.0);
        assert!(
            approx(g, 2.5 + (2.1 - 2.5) * (20.0 - 10.0) / 30.0, 1e-9),
            "Gf={}",
            g
        );
    }

    #[test]
    fn test_er_and_q_hand_calc_h20_roughness_iii() {
        // H=20m, 粗度III (Zb=5,ZG=450,α=0.20), V0=34m/s
        let h_mm = 20_000.0;
        let cfg = WindCfg {
            v0: 34.0,
            roughness: TerrainRoughness::III,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let stories = vec![WindStory {
            z_bottom: 0.0,
            z_top: h_mm,
            width: 10_000.0,
        }];
        let r = wind_forces(h_mm, &stories, &cfg);

        // 手計算: Er = 1.7*(20/450)^0.2
        let er_expected = 1.7 * (20.0_f64 / 450.0).powf(0.20);
        assert!(
            approx(r.er, er_expected, 1e-9),
            "Er={} expected={}",
            r.er,
            er_expected
        );

        let gf_expected = 2.5 + (2.1 - 2.5) * (20.0 - 10.0) / 30.0;
        assert!(
            approx(r.gf, gf_expected, 1e-9),
            "Gf={} expected={}",
            r.gf,
            gf_expected
        );

        let e_expected = er_expected * er_expected * gf_expected;
        assert!(
            approx(r.e, e_expected, 1e-9),
            "E={} expected={}",
            r.e,
            e_expected
        );

        let q_expected = 0.6 * e_expected * 34.0 * 34.0;
        assert!(
            approx(r.q, q_expected, 1e-6),
            "q={} expected={}",
            r.q,
            q_expected
        );
        // 概算値 ≈ 1365.4 N/m^2
        assert!(approx(r.q, 1365.43, 1e-3));
    }

    #[test]
    fn test_kz_straddling_and_above_zb_hand_calc() {
        // H=20m, 粗度III。層1: 0〜8m（Zb=5をまたぐ）、層2: 8〜20m（Zb超のみ）。
        let h_mm = 20_000.0;
        let cfg = WindCfg {
            v0: 34.0,
            roughness: TerrainRoughness::III,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let stories = vec![
            WindStory {
                z_bottom: 0.0,
                z_top: 8_000.0,
                width: 10_000.0,
            },
            WindStory {
                z_bottom: 8_000.0,
                z_top: 20_000.0,
                width: 10_000.0,
            },
        ];
        let r = wind_forces(h_mm, &stories, &cfg);

        // 手計算値（Python で検証済み）。
        assert!(approx(r.kz[0], 0.5976658125212685, 1e-6), "kz0={}", r.kz[0]);
        assert!(approx(r.kz[1], 0.8604072175451684, 1e-6), "kz1={}", r.kz[1]);

        assert!(
            approx(r.pressure[0], 1199.0330083528388, 1e-4),
            "p0={}",
            r.pressure[0]
        );
        assert!(
            approx(r.pressure[1], 1486.0380454879953, 1e-4),
            "p1={}",
            r.pressure[1]
        );

        assert!(approx(r.force[0], 95_922.64, 1e-3), "f0={}", r.force[0]);
        assert!(approx(r.force[1], 178_324.57, 1e-3), "f1={}", r.force[1]);
    }

    #[test]
    fn test_h_less_than_zb_gives_kz_one() {
        // 粗度IV: Zb=10m。建物高さ 8m <= Zb → Kzは全層1.0。
        let h_mm = 8_000.0;
        let cfg = WindCfg {
            v0: 30.0,
            roughness: TerrainRoughness::IV,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let stories = vec![WindStory {
            z_bottom: 0.0,
            z_top: h_mm,
            width: 5_000.0,
        }];
        let r = wind_forces(h_mm, &stories, &cfg);
        assert!((r.kz[0] - 1.0).abs() < 1e-12);
        // Er = 1.7*(Zb/ZG)^alpha であり H に依存しない。
        let er_expected = 1.7 * (10.0_f64 / 550.0).powf(0.27);
        assert!(approx(r.er, er_expected, 1e-9));
    }

    #[test]
    fn test_cpi_cancels_in_total_cf() {
        // Cpi を変えても pressure/force は変化しない（風上・風下合算で相殺）。
        let h_mm = 15_000.0;
        let stories = vec![WindStory {
            z_bottom: 0.0,
            z_top: h_mm,
            width: 8_000.0,
        }];
        let cfg1 = WindCfg {
            v0: 32.0,
            roughness: TerrainRoughness::II,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.0,
        };
        let cfg2 = WindCfg {
            v0: 32.0,
            roughness: TerrainRoughness::II,
            cpe_windward: 0.8,
            cpe_leeward: -0.4,
            cpi: 0.5,
        };
        let r1 = wind_forces(h_mm, &stories, &cfg1);
        let r2 = wind_forces(h_mm, &stories, &cfg2);
        assert!((r1.pressure[0] - r2.pressure[0]).abs() < 1e-9);
        assert!((r1.force[0] - r2.force[0]).abs() < 1e-9);
    }
}
