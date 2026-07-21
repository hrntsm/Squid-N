//! 偏心率の厳密計算コア（武藤 D 値法の閉形式）。仕様 `dev_docs/specs/P7_二次設計.md` §5.1–5.2。
//!
//! - [`ColumnStiffness`] — 柱1本の平面位置と方向別水平剛性（D値）。
//! - [`d_value`] — 武藤 D 値の閉形式。
//! - [`center_of_rigidity`] — 剛心座標 [Xs, Ys]。
//! - [`Eccentricity`] — 偏心率の算定結果。
//! - [`eccentricity`] — 剛心・重心・柱剛性から偏心率を算定。

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
