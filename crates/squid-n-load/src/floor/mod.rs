//! スラブ面荷重の大梁・小梁・柱への分配。
//!
//! 責務ごとにサブモジュールへ分割している:
//! - [`types`] — 基本型（[`LoadShape`]・[`Cmq`]・[`LoadTarget`]・[`BeamLoad`]）と辺荷重ヘルパ
//! - [`geometry`] — 幾何ヘルパ（座標取得・距離・矩形判定・[`polygon_area`]）
//! - [`fem`] — 固定端モーメント・せん断（CMQ）の閉形式公式
//! - [`rect`] — 矩形床の分配戦略（三角形・台形／一方向／負担面積／小梁二段階）
//! - [`cantilever`] — 片持ちスラブ・出隅スラブの分配戦略
//! - [`polygon`] — 多角形床の負担面積法（最近接辺グリッドサンプリング）
//! - [`rigid_zone`] — 剛域を考慮した大梁 CMQ（[`cmq_with_rigid_zone`]）
//!
//! 本モジュールにはこれらを束ねるディスパッチャ [`distribute_slab`] を置く。

mod cantilever;
mod fem;
mod geometry;
mod polygon;
mod rect;
mod rigid_zone;
mod types;

pub use geometry::{polygon_area, slab_dimensions};
pub use rigid_zone::{cmq_with_rigid_zone, RigidZoneCmqMode, RigidZoneCmqResult};
pub use types::{BeamLoad, Cmq, LoadShape, LoadTarget};

use cantilever::{distribute_cantilever, distribute_corner};
use geometry::boundary_coords;
use polygon::{distribute_polygon, distribute_polygon_supported};
use rect::{distribute_rect, distribute_rect_with_joists};
use squid_n_core::model::{DistributionMethod, Model, Slab, SlabKind};

#[cfg(test)]
use fem::{fem_trapezoid, fem_triangle, fem_uniform};

/// スラブの面荷重を大梁（および小梁経由の節点反力）へ分配する。
///
/// 分岐は次の優先順で決まる:
/// 1. `slab.kind == Corner` → 出隅の片持ちスラブ経路（[`distribute_corner`]）。
///    荷重伝達方向・片持ち梁の取付きに関わらず、全荷重を柱（`boundary[0]` の節点）への
///    単一の節点荷重として返す。
/// 2. `slab.kind == Cantilever` → 片持ちスラブ経路。
///    - `slab.edge_supported` が `None`（既定）→ 4頂点を想定し、境界辺 0
///      （`boundary[0]`→`boundary[1]`）を取付き辺とする単純な等分布伝達
///      （[`distribute_cantilever`]。従来互換）。
///    - `slab.edge_supported` が `Some` → 片持ち梁・先端リブ小梁の取付きに応じて
///      指定された支持辺のみへ、最近接辺グリッドサンプリングで分割伝達する
///      （[`distribute_polygon_supported`]）。
/// 3. `slab.kind == Interior` かつ `slab.edge_supported` が `Some` → 指定された支持辺
///    のみへの最近接辺グリッドサンプリング経路（[`distribute_polygon_supported`]）。
///    開口際などで一部の辺が非支持となる一般スラブの分配に用いる。
/// 4. 境界が矩形（[`slab_dimensions`] が `Some` を返す）かつ `slab.joists` が
///    非空で `method` が `TriTrapezoid`/`OneWay` → 小梁二段階伝達経路
///    （[`distribute_rect_with_joists`]）。
/// 5. 境界が矩形 → 従来の矩形床経路（[`distribute_rect`]）。`slab.one_way` が
///    `Some` の場合は指定方向（全体座標 X/Y）に伝達し、`None` は従来互換
///    （境界辺 0・2 が負担）。
/// 6. それ以外（矩形でない凸/凹多角形。三角形・台形・五角形など） →
///    多角形の負担面積法経路（[`distribute_polygon`]）。`one_way` 指定があっても
///    非矩形の場合はこの経路にフォールバックする。
///
/// いずれの経路も総和保存（Σ大梁荷重 (+Σ小梁反力・Σ柱集中荷重) = w×面積）を満たすよう
/// 設計している（床スラブは全体座標 XY 平面内（Z一定）にあることを仮定する）。
/// 入隅の片持ちスラブは本実装では未対応。
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    // 従来互換: `slab.loads`（固定荷重 DL）の総和を分配する。
    distribute_slab_w(model, slab, slab.dead_intensity())
}

/// 指定した面荷重強度 `w`（N/mm²）のみをスラブ境界へ分配する。
///
/// 分岐ロジックは [`distribute_slab`] と同一で、荷重源だけを引数 `w` に差し替える。
/// これにより DL（固定荷重）と LL（積載荷重）を別々の荷重ケースへ分配できる
/// （令85条1項の床用/骨組用/地震用の使い分けや、荷重組合せでの DL/LL 係数分けに用いる）。
/// `w == 0.0` の場合は空の分配結果を返す。
pub fn distribute_slab_w(model: &Model, slab: &Slab, w: f64) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    if slab.boundary.len() < 3 || w == 0.0 {
        return loads;
    }
    let Some(coords) = boundary_coords(model, slab) else {
        return loads;
    };

    let rect_dims = slab_dimensions(model, slab);

    match slab.kind {
        SlabKind::Corner => {
            distribute_corner(slab, &coords, w, &mut loads);
            return loads;
        }
        SlabKind::Cantilever => {
            match &slab.edge_supported {
                Some(supported) => distribute_polygon_supported(&coords, w, &mut loads, supported),
                None => distribute_cantilever(&coords, w, &mut loads),
            }
            return loads;
        }
        SlabKind::Interior => {
            if let Some(supported) = &slab.edge_supported {
                distribute_polygon_supported(&coords, w, &mut loads, supported);
                return loads;
            }
        }
    }
    match rect_dims {
        Some((lx, ly)) => {
            let use_joists = !slab.joists.is_empty()
                && matches!(
                    slab.method,
                    DistributionMethod::TriTrapezoid | DistributionMethod::OneWay
                );
            if use_joists {
                distribute_rect_with_joists(model, slab, &coords, w, &mut loads);
            } else {
                distribute_rect(slab, &coords, lx, ly, w, &mut loads);
            }
        }
        None => distribute_polygon(&coords, w, &mut loads),
    }

    loads
}

#[cfg(test)]
mod tests;
