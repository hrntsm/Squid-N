//! 幾何ヘルパ（境界座標取得・距離・矩形判定・面積）。
//!
//! - [`boundary_coords`] — スラブ境界節点の座標列を取得する
//! - [`dist3`] — 3次元2点間のユークリッド距離
//! - [`slab_dimensions`] — 矩形（平行四辺形）判定と短辺・長辺寸法 `(lx, ly)`
//! - [`edge_len`] — 多角形の辺 i の長さ
//! - [`polygon_area`] — 平面多角形の面積（シューレース公式）

use squid_n_core::model::{Model, Slab};

pub(crate) fn boundary_coords(model: &Model, slab: &Slab) -> Option<Vec<[f64; 3]>> {
    slab.boundary
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect()
}

pub(crate) fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// スラブ境界が矩形（正確には平行四辺形の閉合条件を満たす4辺形）かどうかを判定しつつ、
/// 短辺・長辺相当の寸法 `(lx, ly)`（= `boundary[0]-[1]` 間、`boundary[0]-[3]` 間の距離）を返す。
///
/// `boundary[2]` が `boundary[0] + (boundary[1]-boundary[0]) + (boundary[3]-boundary[0])`
/// （対角線の閉合＝平行四辺形条件）に相対誤差 1e-6 以内で一致することを確認する
/// （レビュー §1.13 対応）。矩形でない4辺形・5角形以上・境界情報欠損の場合は `None` を返し、
/// 呼び出し側は多角形経路（[`distribute_polygon`]）にフォールバックする。
///
/// 注: この判定は「向かい合う辺が等長・平行」という平行四辺形条件のみを検証しており、
/// 直交性（90°）までは検証しない。実運用では境界は軸直交の矩形である前提のため、
/// 既存の TriTrapezoid/OneWay/TributaryArea の面積計算（`lx*ly`）はその前提の下でのみ厳密。
pub fn slab_dimensions(model: &Model, slab: &Slab) -> Option<(f64, f64)> {
    if slab.boundary.len() != 4 {
        return None;
    }
    let p0 = model.nodes.get(slab.boundary[0].index())?.coord;
    let p1 = model.nodes.get(slab.boundary[1].index())?.coord;
    let p2 = model.nodes.get(slab.boundary[2].index())?.coord;
    let p3 = model.nodes.get(slab.boundary[3].index())?.coord;

    let lx = dist3(p0, p1);
    let ly = dist3(p0, p3);
    if lx <= 1e-9 || ly <= 1e-9 {
        return None;
    }

    let expected = [
        p0[0] + (p1[0] - p0[0]) + (p3[0] - p0[0]),
        p0[1] + (p1[1] - p0[1]) + (p3[1] - p0[1]),
        p0[2] + (p1[2] - p0[2]) + (p3[2] - p0[2]),
    ];
    let scale = lx.max(ly);
    let err = dist3(expected, p2);
    if err / scale > 1e-6 {
        return None;
    }
    Some((lx, ly))
}

pub(crate) fn edge_len(coords: &[[f64; 3]], i: usize) -> f64 {
    let n = coords.len();
    dist3(coords[i], coords[(i + 1) % n])
}

/// 平面多角形の面積（ニュートンの公式＝シューレース公式）。全体座標 XY 平面へ投影して
/// 計算する（床スラブは水平面内にある＝Z一定という前提）。
pub fn polygon_area(coords: &[[f64; 3]]) -> f64 {
    let n = coords.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = coords[i];
        let b = coords[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    (sum / 2.0).abs()
}
