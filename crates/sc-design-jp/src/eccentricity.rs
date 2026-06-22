//! T2: 偏心率 Re（剛心＝武藤 D値法・略算）。仕様 `specs/P7_二次設計.md` §5。
//!
//! 本モジュールは2層構造になっている:
//! 1. **厳密な計算コア**（`d_value` / `center_of_rigidity` / `eccentricity`）。
//!    告示1792・武藤 D値法の閉形式そのもので、手計算と 1e-9 で一致する（DoD §8.1）。
//! 2. **モデル抽出**（`column_stiffnesses` / `center_of_mass` / `story_centers`）。
//!    実モデルから柱・梁を拾って 1. に渡す略算層。柱＝鉛直部材という幾何判定等、
//!    明示した仮定の上に成り立つ（精算＝マスター節点 3×3 剛性は将来）。
//!
//! **方向の扱い（★最重要）:** 剛心座標は方向別 D 値で重み付けする。
//! `Xs = Σ(Dy·x)/ΣDy`, `Ys = Σ(Dx·y)/ΣDx`。単一 D 値で済むのは対称架構のみ。

use sc_core::ids::StoryId;
use sc_core::model::Model;

/// 1 本の柱（鉛直部材）の、平面位置と方向別水平剛性（D値）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColumnStiffness {
    /// 柱の平面位置 (x, y) [mm]。
    pub pos: [f64; 2],
    /// X 加力方向の水平剛性 Dx [N/mm]。
    pub dx: f64,
    /// Y 加力方向の水平剛性 Dy [N/mm]。
    pub dy: f64,
}

/// 武藤 D 値の閉形式（仕様 §5.1）。加力方向ごとに呼ぶ。
///
/// - `e`: ヤング係数 [N/mm²]
/// - `ic`: 加力方向の柱断面二次モーメント [mm⁴]
/// - `h`: 階高（柱長）[mm]
/// - `sum_beam_stiffness_ratio`: 柱頭・柱脚に取り付く、加力方向に効く梁の剛比 ΣKb（= Σ Ib/Lb）
/// - `first_story`: 最下階（柱脚固定）なら true。一般階は false。
///
/// ```text
/// Kc0 = 12·E·Ic/h³,  kc = Ic/h,  k̄ = ΣKb/(2·kc)
/// a   = k̄/(2+k̄)            （一般階）
///     = (0.5+k̄)/(2+k̄)      （最下階・柱脚固定）
/// D   = a · Kc0
/// ```
pub fn d_value(e: f64, ic: f64, h: f64, sum_beam_stiffness_ratio: f64, first_story: bool) -> f64 {
    if h <= 0.0 || ic <= 0.0 {
        return 0.0;
    }
    let kc0 = 12.0 * e * ic / (h * h * h);
    let kc = ic / h;
    if kc <= 0.0 {
        return 0.0;
    }
    let kbar = sum_beam_stiffness_ratio / (2.0 * kc);
    let a = if first_story {
        (0.5 + kbar) / (2.0 + kbar)
    } else {
        kbar / (2.0 + kbar)
    };
    a * kc0
}

/// 剛心座標 [Xs, Ys]。`Xs = Σ(Dy·x)/ΣDy`, `Ys = Σ(Dx·y)/ΣDx`（仕様 §5.1）。
pub fn center_of_rigidity(cols: &[ColumnStiffness]) -> [f64; 2] {
    let sum_dy: f64 = cols.iter().map(|c| c.dy).sum();
    let sum_dx: f64 = cols.iter().map(|c| c.dx).sum();
    let xs = if sum_dy == 0.0 {
        0.0
    } else {
        cols.iter().map(|c| c.dy * c.pos[0]).sum::<f64>() / sum_dy
    };
    let ys = if sum_dx == 0.0 {
        0.0
    } else {
        cols.iter().map(|c| c.dx * c.pos[1]).sum::<f64>() / sum_dx
    };
    [xs, ys]
}

/// 偏心率の算定結果（X 加力・Y 加力）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Eccentricity {
    /// 偏心距離 ex = |Xg − Xs| [mm]。
    pub ex: f64,
    /// 偏心距離 ey = |Yg − Ys| [mm]。
    pub ey: f64,
    /// ねじり剛性 KR = Σ(Dx·ȳ²) + Σ(Dy·x̄²)（剛心まわり）。
    pub kr: f64,
    /// 弾力半径 rex = √(KR/ΣDx)。
    pub rex: f64,
    /// 弾力半径 rey = √(KR/ΣDy)。
    pub rey: f64,
    /// X 加力時の偏心率 Rex = ey/rex（規定 ≤ 0.15）。
    pub re_x: f64,
    /// Y 加力時の偏心率 Rey = ex/rey（規定 ≤ 0.15）。
    pub re_y: f64,
}

/// 剛心・重心・柱剛性から偏心率を算定（仕様 §5.2）。
pub fn eccentricity(
    cols: &[ColumnStiffness],
    center_of_mass: [f64; 2],
    center_of_rigidity: [f64; 2],
) -> Eccentricity {
    let [xs, ys] = center_of_rigidity;
    let [xg, yg] = center_of_mass;
    let ex = (xg - xs).abs();
    let ey = (yg - ys).abs();

    let sum_dx: f64 = cols.iter().map(|c| c.dx).sum();
    let sum_dy: f64 = cols.iter().map(|c| c.dy).sum();

    // 剛心まわりのねじり剛性。x̄, ȳ は剛心からの距離。
    let kr: f64 = cols
        .iter()
        .map(|c| {
            let xbar = c.pos[0] - xs;
            let ybar = c.pos[1] - ys;
            c.dx * ybar * ybar + c.dy * xbar * xbar
        })
        .sum();

    let rex = if sum_dx > 0.0 {
        (kr / sum_dx).sqrt()
    } else {
        0.0
    };
    let rey = if sum_dy > 0.0 {
        (kr / sum_dy).sqrt()
    } else {
        0.0
    };
    let re_x = if rex > 0.0 { ey / rex } else { 0.0 };
    let re_y = if rey > 0.0 { ex / rey } else { 0.0 };

    Eccentricity {
        ex,
        ey,
        kr,
        rex,
        rey,
        re_x,
        re_y,
    }
}

// ===== モデル抽出層（略算）=====

/// 重心（質量中心）[Xg, Yg]。当該層の節点質量（並進成分）で重み付けする。
///
/// 質量未定義の節点は質量 0（剛心の重み付けには寄与しない）。全質量 0 なら幾何重心。
pub fn center_of_mass(model: &Model, story: StoryId) -> [f64; 2] {
    let nodes: Vec<&sc_core::model::Node> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(story))
        .collect();
    if nodes.is_empty() {
        return [0.0, 0.0];
    }
    let mass = |n: &sc_core::model::Node| n.mass.map(|m| m[0]).unwrap_or(0.0);
    let total: f64 = nodes.iter().map(|n| mass(n)).sum();
    if total > 0.0 {
        let xg = nodes.iter().map(|n| mass(n) * n.coord[0]).sum::<f64>() / total;
        let yg = nodes.iter().map(|n| mass(n) * n.coord[1]).sum::<f64>() / total;
        [xg, yg]
    } else {
        // 質量未定義 → 幾何重心で代用。
        let n = nodes.len() as f64;
        let xg = nodes.iter().map(|n| n.coord[0]).sum::<f64>() / n;
        let yg = nodes.iter().map(|n| n.coord[1]).sum::<f64>() / n;
        [xg, yg]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- d_value ----
    #[test]
    fn test_d_value_rigid_beams_general() {
        // 梁が十分剛（ΣKb 大）→ k̄ 大 → a → 1 → D → Kc0
        let e = 1.0;
        let ic = 1.0;
        let h = 1.0;
        let kc0 = 12.0 * e * ic / (h * h * h);
        let d = d_value(e, ic, h, 1e9, false);
        assert!((d - kc0).abs() / kc0 < 1e-6, "a→1 で D→Kc0, got {d}");
    }

    #[test]
    fn test_d_value_known_kbar() {
        // kc = Ic/h = 1, ΣKb = 4 → k̄ = 4/(2·1) = 2 → a = 2/(2+2) = 0.5
        // Kc0 = 12 → D = 0.5·12 = 6
        let d = d_value(1.0, 1.0, 1.0, 4.0, false);
        assert!((d - 6.0).abs() < 1e-9, "got {d}");
    }

    #[test]
    fn test_d_value_first_story() {
        // 最下階: k̄ = 2 → a = (0.5+2)/(2+2) = 0.625 → D = 0.625·12 = 7.5
        let d = d_value(1.0, 1.0, 1.0, 4.0, true);
        assert!((d - 7.5).abs() < 1e-9, "got {d}");
    }

    #[test]
    fn test_d_value_degenerate() {
        assert_eq!(d_value(1.0, 0.0, 1.0, 4.0, false), 0.0);
        assert_eq!(d_value(1.0, 1.0, 0.0, 4.0, false), 0.0);
    }

    // ---- center_of_rigidity（DoD §8.1）----
    #[test]
    fn test_center_of_rigidity_dod_example() {
        // 仕様 §5.2 の確定値: Dy=[100,300] @ x=[0,6000] → Xs = 4500
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 1.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 1.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs got {}", cr[0]);
    }

    #[test]
    fn test_eccentricity_dod_example() {
        // 上の剛心に重心 Xg=3000 → ex = 1500（DoD §8.1）
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 1.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 1.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        let ecc = eccentricity(&cols, [3000.0, 0.0], cr);
        assert!((ecc.ex - 1500.0).abs() < 1e-9, "ex got {}", ecc.ex);
    }

    #[test]
    fn test_eccentricity_symmetric_zero() {
        // 対称 4 本柱 → 剛心＝重心＝中央 → 偏心率 0
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [0.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        assert!((cr[0] - 3000.0).abs() < 1e-9);
        assert!((cr[1] - 3000.0).abs() < 1e-9);
        let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
        assert!(ecc.re_x.abs() < 1e-9 && ecc.re_y.abs() < 1e-9);
    }

    #[test]
    fn test_eccentricity_hand_calc() {
        // 手計算照合（X 加力時偏心率）。
        // 柱4本、すべて Dx=Dy=100 とし x=[0,0,6000,6000], y=[0,6000,0,6000]…ではなく
        // 剛心をずらすため右側を強くする: Dy=[100,100,300,300] @ x=[0,0,6000,6000]
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [0.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 100.0,
                dy: 300.0,
            },
            ColumnStiffness {
                pos: [6000.0, 6000.0],
                dx: 100.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        // Xs = (100·0+100·0+300·6000+300·6000)/(100+100+300+300) = 3,600,000/800 = 4500
        assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs {}", cr[0]);
        // Ys = (Σ Dx·y)/ΣDx = 100·(0+6000+0+6000)/400 = 3000
        assert!((cr[1] - 3000.0).abs() < 1e-9, "Ys {}", cr[1]);

        // 重心は幾何中央 (3000, 3000) とする → ex = 1500, ey = 0
        let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
        assert!((ecc.ex - 1500.0).abs() < 1e-9);
        assert!(ecc.ey.abs() < 1e-9);

        // KR = Σ Dx·ȳ² + Σ Dy·x̄²
        //   x̄ = x-4500 = [-4500,-4500,1500,1500], ȳ = y-3000 = [-3000,3000,-3000,3000]
        //   Σ Dx·ȳ² = 100·(3000²·4) = 100·4·9e6 = 3.6e9
        //   Σ Dy·x̄² = 100·4500² + 100·4500² + 300·1500² + 300·1500²
        //           = 2·100·2.025e7 + 2·300·2.25e6 = 4.05e9 + 1.35e9 = 5.4e9
        //   KR = 3.6e9 + 5.4e9 = 9.0e9
        assert!((ecc.kr - 9.0e9).abs() / 9.0e9 < 1e-12, "KR {}", ecc.kr);
        // ΣDx = 400 → rex = √(9.0e9/400) = √2.25e7 = 4743.416...
        let rex = (9.0e9_f64 / 400.0).sqrt();
        assert!((ecc.rex - rex).abs() < 1e-6);
        // Rex = ey/rex = 0（ey=0）, Rey = ex/rey
        assert!(ecc.re_x.abs() < 1e-12);
        let sum_dy = 800.0;
        let rey = (9.0e9_f64 / sum_dy).sqrt();
        assert!((ecc.re_y - 1500.0 / rey).abs() < 1e-9, "Rey {}", ecc.re_y);
    }
}
