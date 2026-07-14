use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{DistributionMethod, Model, OneWayDir, Slab, SlabKind};

#[derive(Clone, Copy, Debug)]
pub enum LoadShape {
    Uniform { w: f64 },
    Trapezoid { w0: f64, a: f64, b: f64 },
    Triangle { w0: f64 },
    Point { p: f64, x: f64 },
}

#[derive(Clone, Copy, Debug)]
pub struct Cmq {
    pub c_i: f64,
    pub c_j: f64,
    pub q_i: f64,
    pub q_j: f64,
}

/// 荷重の作用対象。
///
/// 既存の `BeamLoad.elem` はスラブ境界の「辺インデックス」(`ElemId(i)`, 辺 i =
/// `boundary[i]`→`boundary[(i+1)%n]`) を格納する規約（呼び出し側 `squid-n-app` の
/// `refresh_beam_loads` が辺→実部材へ対応付ける）。この規約は変更しない。
///
/// 一方、小梁反力のように「特定の辺に載らず、特定の節点へ集中荷重として作用する」
/// 荷重を表現する必要が出てきたため、`target` フィールドを追加した。
/// - `Edge(i)`: 従来どおり境界の辺 i。`elem` フィールドにも同じ値 `ElemId(i as u32)` を
///   設定する（互換のため二重に持つ）。
/// - `Node(id)`: 実節点 `id` への集中荷重。`elem` フィールドは無意味なので
///   `ElemId(u32::MAX)` を番兵として設定する（`squid-n-app` 側は `elem` しか見ないため、
///   境界辺数 4 を必ず超えるこの値により誤って辺荷重として処理されることを防ぐ）。
///   `squid-n-app::refresh_beam_loads` は現状 `target` を解釈しないため、`Node` 荷重を
///   実際に構造モデルへ反映する対応は後続タスク（app 側）で行う。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadTarget {
    Edge(usize),
    Node(NodeId),
}

#[derive(Clone, Copy, Debug)]
pub struct BeamLoad {
    pub elem: ElemId,
    pub target: LoadTarget,
    pub shape: LoadShape,
    pub cmq: Cmq,
}

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
    let mut loads = Vec::new();
    if slab.boundary.len() < 3 {
        return loads;
    }
    let Some(coords) = boundary_coords(model, slab) else {
        return loads;
    };

    let rect_dims = slab_dimensions(model, slab);

    for area_load in &slab.loads {
        let w = area_load.value;
        match slab.kind {
            SlabKind::Corner => {
                distribute_corner(slab, &coords, w, &mut loads);
                continue;
            }
            SlabKind::Cantilever => {
                match &slab.edge_supported {
                    Some(supported) => {
                        distribute_polygon_supported(&coords, w, &mut loads, supported)
                    }
                    None => distribute_cantilever(&coords, w, &mut loads),
                }
                continue;
            }
            SlabKind::Interior => {
                if let Some(supported) = &slab.edge_supported {
                    distribute_polygon_supported(&coords, w, &mut loads, supported);
                    continue;
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
    }

    loads
}

/// 出隅の片持ちスラブの分配（`SlabKind::Corner`）。
///
/// 出隅の片持ちスラブの荷重は、荷重伝達方向および片持ち梁の取付きに関わらず、節点荷重
/// としてすべて柱に伝達する。本実装ではこれに従い、荷重伝達方向（`one_way`）や
/// `slab.edge_supported`（片持ち梁の有無）を一切参照せず、全荷重
/// `W = w × 多角形面積`（[`polygon_area`]。構造芯から出隅先端までの長方形
/// ＝境界そのものの面積）を柱（`boundary[0]` の節点）への
/// 単一の集中荷重として返す。小梁反力・[`distribute_rect_with_joists`] の柱集中荷重と
/// 同じ `LoadTarget::Node` + `LoadShape::Point`（`q_i = W`、`q_j = 0`）の機構を再利用する。
fn distribute_corner(slab: &Slab, coords: &[[f64; 3]], w: f64, loads: &mut Vec<BeamLoad>) {
    let area = polygon_area(coords);
    if area <= 0.0 {
        return;
    }
    let total = w * area;
    loads.push(BeamLoad {
        elem: ElemId(u32::MAX),
        target: LoadTarget::Node(slab.boundary[0]),
        shape: LoadShape::Point { p: total, x: 0.0 },
        cmq: Cmq {
            c_i: 0.0,
            c_j: 0.0,
            q_i: total,
            q_j: 0.0,
        },
    });
}

fn boundary_coords(model: &Model, slab: &Slab) -> Option<Vec<[f64; 3]>> {
    slab.boundary
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect()
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn push_edge(loads: &mut Vec<BeamLoad>, i: usize, shape: LoadShape, cmq: Cmq) {
    loads.push(BeamLoad {
        elem: ElemId(i as u32),
        target: LoadTarget::Edge(i),
        shape,
        cmq,
    });
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
fn slab_dimensions(model: &Model, slab: &Slab) -> Option<(f64, f64)> {
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

fn edge_dir2(coords: &[[f64; 3]], i: usize) -> [f64; 2] {
    let n = coords.len();
    let a = coords[i];
    let b = coords[(i + 1) % n];
    [b[0] - a[0], b[1] - a[1]]
}

fn edge_len(coords: &[[f64; 3]], i: usize) -> f64 {
    let n = coords.len();
    dist3(coords[i], coords[(i + 1) % n])
}

/// 矩形の4辺を、与えられた2D軸ベクトルに対して「軸に平行な2辺 (0,2 または 1,3)」と
/// 「軸に直交する2辺」に分類する。矩形（辺0‖辺2、辺1‖辺3）を仮定する。
/// 戻り値は `(平行な2辺, 直交する2辺)`。
fn classify_rect_edges_by_axis(coords: &[[f64; 3]], axis: [f64; 2]) -> ([usize; 2], [usize; 2]) {
    let d0 = edge_dir2(coords, 0);
    let n0 = (d0[0] * d0[0] + d0[1] * d0[1]).sqrt().max(1e-12);
    let na = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt().max(1e-12);
    let cos0 = (d0[0] * axis[0] + d0[1] * axis[1]).abs() / (n0 * na);
    if cos0 >= 0.5 {
        ([0, 2], [1, 3])
    } else {
        ([1, 3], [0, 2])
    }
}

/// 矩形床の分配（三角形・台形（45°）／一方向／負担面積法）。`coords` は境界4頂点の座標。
fn distribute_rect(
    slab: &Slab,
    coords: &[[f64; 3]],
    lx: f64,
    ly: f64,
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    match slab.method {
        DistributionMethod::TriTrapezoid => {
            let is_square = (lx - ly).abs() < 1e-6;
            if is_square {
                let w0 = w * lx / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    push_edge(loads, i, LoadShape::Triangle { w0 }, fem_triangle(w0, l));
                }
            } else {
                let short = lx.min(ly);
                let long = lx.max(ly);
                let w0 = w * short / 2.0;
                let a = short / 2.0;
                let b = long - 2.0 * a;

                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    let is_short_side = (l - short).abs() < 1e-6;
                    if is_short_side {
                        push_edge(loads, i, LoadShape::Triangle { w0 }, fem_triangle(w0, l));
                    } else {
                        push_edge(
                            loads,
                            i,
                            LoadShape::Trapezoid { w0, a, b },
                            fem_trapezoid(w0, a, b, l),
                        );
                    }
                }
            }
        }
        DistributionMethod::OneWay => {
            if let Some(dir) = slab.one_way {
                distribute_one_way_dir(coords, dir, w, loads);
            } else {
                // 従来互換（`one_way` 未指定）: 辺0・2負担（＝辺1方向スパン）。
                let w_line = w * ly / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    if (l - lx).abs() < 1e-6 {
                        push_edge(
                            loads,
                            i,
                            LoadShape::Uniform { w: w_line },
                            fem_uniform(w_line, l),
                        );
                    }
                }
            }
        }
        DistributionMethod::TributaryArea => {
            // 45°負担面積を等価等分布へ換算（総和保存）。
            let short = lx.min(ly);
            let long = lx.max(ly);
            for i in 0..4 {
                let l = if i % 2 == 0 { lx } else { ly };
                let is_short_side = (l - short).abs() <= (l - long).abs();
                let w_line = if is_short_side {
                    w * short / 4.0
                } else {
                    w * (short * long / 2.0 - short * short / 4.0) / long
                };
                push_edge(
                    loads,
                    i,
                    LoadShape::Uniform { w: w_line },
                    fem_uniform(w_line, l),
                );
            }
        }
    }
}

/// 一方向スラブの荷重伝達方向指定（`slab.one_way = Some(dir)`）に基づく分配（レビュー §1.13）。
///
/// `dir` に対応する全体座標軸（X→[1,0]、Y→[0,1]）を「伝達方向」とし、伝達方向に
/// **直交する**2辺（＝伝達方向と垂直に走る2辺）を負担辺とする。負担辺の線荷重は
/// `w×(スパン長/2)`（スパン長＝伝達方向に平行な辺の長さ）。総和は
/// `2×w×(スパン長/2)×負担辺長 = w×面積` で保存される。
fn distribute_one_way_dir(coords: &[[f64; 3]], dir: OneWayDir, w: f64, loads: &mut Vec<BeamLoad>) {
    let axis = match dir {
        OneWayDir::X => [1.0, 0.0],
        OneWayDir::Y => [0.0, 1.0],
    };
    let (parallel, bearing) = classify_rect_edges_by_axis(coords, axis);
    let span_len = edge_len(coords, parallel[0]);
    let w_line = w * span_len / 2.0;
    for &e in &bearing {
        let l_e = edge_len(coords, e);
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: w_line },
            fem_uniform(w_line, l_e),
        );
    }
}

/// 小梁 (`slab.joists`) による二段階伝達（矩形スラブのみ。仕様書 §5.1「2段階」）。
///
/// **簡易モデルの仮定**（本実装の割り切り。厳密な二方向効果は考慮しない）:
/// - 各 `JoistLine` は1本の物理的な小梁を表し、`dir` 方向に架かり、`spacing` 分の
///   負担幅（トリビュタリ幅）を一様に負担すると仮定する（面荷重×`spacing`＝小梁の等分布荷重）。
/// - 小梁は単純梁として振る舞い、両端 `support` 節点へ反力
///   `R = w·spacing·L_joist/2`（`L_joist` = support間距離）を返す
///   （`LoadTarget::Node(support)` の集中荷重、`LoadShape::Point`）。
/// - 小梁と平行な2辺（境界の大梁。`dir` に平行な辺）は、小梁群がカバーしない残りの幅
///   `remainder = width_total − Σspacing_k`（`width_total` = 小梁と直交する境界辺の長さ）を
///   両辺で折半して負担する（各辺 UDL = `w·remainder/2`）。これは「小梁が
///   `spacing` 間隔で並び、境界からも半間隔 `spacing/2` 離れて配置される」典型例のとき
///   厳密に `w·(spacing/2)` に一致する（境界梁が"仮想的な端の小梁"のように振る舞う）。
///   この remainder 方式により、`spacing` が不均一でも総和保存が厳密に成り立つ。
/// - 小梁位置・本数の実際の配置（`JoistLine.support` の座標）はモデルからそのまま用いる。
fn distribute_rect_with_joists(
    model: &Model,
    slab: &Slab,
    coords: &[[f64; 3]],
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    if slab.joists.is_empty() {
        return;
    }
    let dir = slab.joists[0].dir;
    let dn = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
    let axis = if dn > 1e-12 {
        [dir[0] / dn, dir[1] / dn]
    } else {
        [0.0, 1.0]
    };
    let (parallel_edges, perp_edges) = classify_rect_edges_by_axis(coords, axis);
    let width_total = edge_len(coords, perp_edges[0]);

    let mut spacing_sum = 0.0;
    for j in &slab.joists {
        let (Some(n0), Some(n1)) = (
            model.nodes.get(j.support[0].index()),
            model.nodes.get(j.support[1].index()),
        ) else {
            continue;
        };
        let l_joist = dist3(n0.coord, n1.coord);
        let r = w * j.spacing * l_joist / 2.0;
        loads.push(BeamLoad {
            elem: ElemId(u32::MAX),
            target: LoadTarget::Node(j.support[0]),
            shape: LoadShape::Point { p: r, x: 0.0 },
            cmq: Cmq {
                c_i: 0.0,
                c_j: 0.0,
                q_i: r,
                q_j: 0.0,
            },
        });
        loads.push(BeamLoad {
            elem: ElemId(u32::MAX),
            target: LoadTarget::Node(j.support[1]),
            shape: LoadShape::Point { p: r, x: 0.0 },
            cmq: Cmq {
                c_i: 0.0,
                c_j: 0.0,
                q_i: r,
                q_j: 0.0,
            },
        });
        spacing_sum += j.spacing;
    }

    let remainder = width_total - spacing_sum;
    let boundary_w_line = w * remainder / 2.0;
    for &e in &parallel_edges {
        let l_e = edge_len(coords, e);
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: boundary_w_line },
            fem_uniform(boundary_w_line, l_e),
        );
    }
}

fn point_line_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt();
    if len < 1e-12 {
        return dist3(p, a);
    }
    (ap[0] * ab[1] - ap[1] * ab[0]).abs() / len
}

/// 片持ちスラブの分配（`SlabKind::Cantilever`）。
///
/// 4頂点を想定し、境界辺0（`boundary[0]`→`boundary[1]`）を取付き辺（大梁側）、
/// その対辺2を先端とみなす。出し幅 `d` は辺0の直線から頂点2・3までの垂直距離の平均。
/// 片持ち梁がない場合は全て大梁に伝達する扱いとし、取付き辺へ
/// 等分布荷重 `w_line = w·d`（先端まで一様なスラブの単純片持ち反力に相当）として
/// 集約する（`LoadShape::Uniform` + `fem_uniform`）。
///
/// 片持ち梁・先端リブ小梁がある場合の分割伝達は `slab.edge_supported` を指定することで
/// [`distribute_polygon_supported`] 経路（[`distribute_slab`] 側で分岐）が担う。
/// 出隅の片持ちスラブは `SlabKind::Corner`（[`distribute_corner`]）が別途担う。
///
/// **未対応（残課題）**: 入隅の片持ちスラブ（本実装では非対応）。
fn distribute_cantilever(coords: &[[f64; 3]], w: f64, loads: &mut Vec<BeamLoad>) {
    if coords.len() < 4 {
        return;
    }
    let l_attach = edge_len(coords, 0);
    let d = 0.5
        * (point_line_dist(coords[2], coords[0], coords[1])
            + point_line_dist(coords[3], coords[0], coords[1]));
    if l_attach <= 1e-9 || d <= 1e-9 {
        return;
    }
    let w_line = w * d;
    push_edge(
        loads,
        0,
        LoadShape::Uniform { w: w_line },
        fem_uniform(w_line, l_attach),
    );
}

const POLY_GRID_N: usize = 200;

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

fn bbox2(poly: &[[f64; 2]]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for p in poly {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
    }
    (min_x, max_x, min_y, max_y)
}

/// 点が多角形内部にあるか（レイキャスト法／偶奇則）。
fn point_in_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i][0], poly[i][1]);
        let (xj, yj) = (poly[j][0], poly[j][1]);
        if (yi > p[1]) != (yj > p[1]) {
            let x_int = (xj - xi) * (p[1] - yi) / (yj - yi) + xi;
            if p[0] < x_int {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn point_segment_dist2(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if len2 > 1e-12 {
        ((ap[0] * ab[0] + ap[1] * ab[1]) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let proj = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    let dx = p[0] - proj[0];
    let dy = p[1] - proj[1];
    dx * dx + dy * dy
}

/// 矩形でない凸（または単純な凹）多角形床の分配（レビュー §1.13 ギャップ「多角形床組」対応）。
///
/// 45°法の一般化として「各点を最も近い辺に帰属させる」負担面積法を、多角形の
/// バウンディングボックスを `POLY_GRID_N × POLY_GRID_N`（200×200）に格子分割した
/// 決定的なサンプリングで近似する。各セル中心が多角形内部なら、その中心から最も近い
/// 辺（線分）へセル面積を加算する。辺ごとの負担面積が求まったら、等価等分布
/// `w_line = W_edge / L_edge`（`W_edge = w × 辺の負担面積`）として `LoadShape::Uniform` +
/// `fem_uniform` で返す。
///
/// 荷重保存は「サンプル点の全数帰属」により、格子内部と判定された点の面積の総和について
/// 厳密に成り立つ（Σ辺負担荷重 = w × Σ格子内サンプル面積）。格子内サンプル面積と真の
/// 多角形面積（[`polygon_area`]）との差は格子近似誤差のみで、十分細かい分割（200×200）で
/// 1%未満に収まる（凸多角形で確認）。強く凹んだ（入隅の深い）多角形では近似精度が
/// 低下する可能性がある（未検証・残課題）。
fn distribute_polygon(coords: &[[f64; 3]], w: f64, loads: &mut Vec<BeamLoad>) {
    let n = coords.len();
    if n < 3 {
        return;
    }
    let candidate_edges: Vec<usize> = (0..n).collect();
    let edge_area = polygon_edge_areas(coords, &candidate_edges);
    emit_edge_loads(coords, w, &edge_area, loads);
}

/// 多角形の各辺への負担面積を、格子サンプリングで求める（[`distribute_polygon`] と
/// [`distribute_polygon_supported`] の共通処理）。各セル中心が多角形内部なら、
/// `candidate_edges` の中で最も近い辺（線分）へセル面積を加算する。
/// `candidate_edges` に全辺（`0..n`）を渡せば [`distribute_polygon`] と同じ挙動になり、
/// 部分集合を渡せば非候補の辺には荷重が帰属しなくなる（[`distribute_polygon_supported`]）。
fn polygon_edge_areas(coords: &[[f64; 3]], candidate_edges: &[usize]) -> Vec<f64> {
    let n = coords.len();
    let mut edge_area = vec![0.0_f64; n];
    if candidate_edges.is_empty() {
        return edge_area;
    }
    let poly2: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (min_x, max_x, min_y, max_y) = bbox2(&poly2);
    let dx = (max_x - min_x) / POLY_GRID_N as f64;
    let dy = (max_y - min_y) / POLY_GRID_N as f64;
    if dx <= 0.0 || dy <= 0.0 {
        return edge_area;
    }
    let cell_area = dx * dy;
    for iy in 0..POLY_GRID_N {
        let y = min_y + (iy as f64 + 0.5) * dy;
        for ix in 0..POLY_GRID_N {
            let x = min_x + (ix as f64 + 0.5) * dx;
            let p = [x, y];
            if !point_in_polygon(p, &poly2) {
                continue;
            }
            let mut best_e = candidate_edges[0];
            let mut best_d2 = f64::INFINITY;
            for &e in candidate_edges {
                let a = poly2[e];
                let b = poly2[(e + 1) % n];
                let d2 = point_segment_dist2(p, a, b);
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_e = e;
                }
            }
            edge_area[best_e] += cell_area;
        }
    }
    edge_area
}

/// 辺ごとの負担面積 `edge_area`（[`polygon_edge_areas`] の出力）を、等価等分布
/// `w_line = W_edge / L_edge`（`W_edge = w × edge_area[e]`）の辺荷重として `loads` へ追加する。
fn emit_edge_loads(coords: &[[f64; 3]], w: f64, edge_area: &[f64], loads: &mut Vec<BeamLoad>) {
    for (e, &a_e) in edge_area.iter().enumerate() {
        if a_e <= 0.0 {
            continue;
        }
        let l_e = edge_len(coords, e);
        if l_e <= 1e-9 {
            continue;
        }
        let w_edge = w * a_e;
        let w_line = w_edge / l_e;
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: w_line },
            fem_uniform(w_line, l_e),
        );
    }
}

/// 支持辺指定付きの最近接辺グリッドサンプリング帰属（レビュー残課題「片持ち梁・先端リブ
/// 小梁の分割伝達」「一般スラブの部分支持（開口際等）」対応。片持ち梁がある
/// 場合はスラブと同様のルールにより分割して荷重伝達する扱い）。
///
/// [`distribute_polygon`] と同じ格子サンプリング法（[`polygon_edge_areas`]）だが、各サンプル
/// 点を「支持辺（`supported[i] == true` の辺）のみ」の中から最近接の辺に帰属させる。
/// 非支持辺（`supported[i] == false`）には荷重が帰属しない。
///
/// 呼び出し元（[`distribute_slab`]）の用途は2通り:
/// - `SlabKind::Cantilever` + `edge_supported`: 取付き大梁（辺0）に加え、片持ち梁・先端
///   リブ小梁が取り付く辺を支持辺として指定する（例: 辺0・1・3 支持＝両側に片持ち梁、
///   辺2 も支持に含めれば先端リブ小梁あり）。
/// - `SlabKind::Interior` + `edge_supported`: 開口際などで一部の辺が大梁・小梁に
///   取り付かない一般スラブの分配に用いる（非支持辺には荷重を負担させない一般化）。
///
/// `supported` の長さが `coords.len()` と一致しない、または支持辺が1つも無い
/// （全要素 `false`）場合は、指定が無意味なため安全側（総荷重を捨てない）に倒して
/// 全辺支持へフォールバックする（＝ [`distribute_polygon`] と同じ結果になる）。
fn distribute_polygon_supported(
    coords: &[[f64; 3]],
    w: f64,
    loads: &mut Vec<BeamLoad>,
    supported: &[bool],
) {
    let n = coords.len();
    if n < 3 {
        return;
    }
    let candidate_edges: Vec<usize> = if supported.len() == n && supported.iter().any(|&b| b) {
        (0..n).filter(|&i| supported[i]).collect()
    } else {
        (0..n).collect()
    };
    let edge_area = polygon_edge_areas(coords, &candidate_edges);
    emit_edge_loads(coords, w, &edge_area, loads);
}

fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

/// 対称台形荷重（両端 a 区間で 0→w0 に線形立上り、中央 L−2a 区間は等高 w0）の
/// 両端固定梁の固定端モーメント・せん断。
/// 固定端モーメントは閉形式 FEM = (1/L²)∫₀ᴸ w(x)·x·(L−x)² dx を評価して求める。
/// 検算: a→L/2 で対称三角形 5w0L²/96、a→0 で等分布 w0L²/12 に一致する。
#[allow(unused_variables)]
fn fem_trapezoid(w0: f64, a: f64, b: f64, l: f64) -> Cmq {
    // ∫ x(L-x)² dx の不定積分
    let g = |x: f64| l * l * x * x / 2.0 - 2.0 * l * x * x * x / 3.0 + x.powi(4) / 4.0;
    // 両端の三角形立上り区間（[0,a] と [L-a,L]）の寄与（/a を約分済みの閉形式）
    let i_ends = w0 * l * a * a * (l / 3.0 - a / 4.0);
    // 中央の等分布区間 [a, L-a] の寄与
    let i_mid = w0 * (g(l - a) - g(a));
    let fem = (i_ends + i_mid) / (l * l);
    // 総荷重 = 台形面積（単位幅あたり）= w0·(L−a)。せん断は対称なので両端で W/2。
    let total = w0 * (l - a);
    Cmq {
        c_i: fem,
        c_j: -fem,
        q_i: total / 2.0,
        q_j: total / 2.0,
    }
}

// ---------------------------------------------------------------------------
// 剛域考慮 CMQ（レビュー §1.13 ギャップ「剛域考慮 CMQ」対応）
// ---------------------------------------------------------------------------

const SIMPSON_N: usize = 2000;

/// 合成シンプソン則による定積分（決定的・十分な分割数）。`n` は偶数に丸める。
fn simpson_integrate<F: Fn(f64) -> f64>(f: F, a: f64, b: f64, n: usize) -> f64 {
    let n = if n % 2 == 1 { n + 1 } else { n.max(2) };
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for k in 1..n {
        let x = a + k as f64 * h;
        sum += if k % 2 == 0 { 2.0 } else { 4.0 } * f(x);
    }
    sum * h / 3.0
}

/// 荷重形状 `shape`（節点 i 起点、全長 `l_total` の局所座標 x∈[0,l_total] で定義）の
/// 位置 `x` における荷重強度。`fem_uniform`/`fem_triangle`/`fem_trapezoid` が暗黙に
/// 前提とする形状定義と整合させている
/// （三角形＝中央ピーク対称、台形＝両端 a 区間で 0→w0 立上り・中央等高）。
fn shape_intensity(shape: &LoadShape, l_total: f64, x: f64) -> f64 {
    match shape {
        LoadShape::Uniform { w } => *w,
        LoadShape::Triangle { w0 } => {
            let half = l_total / 2.0;
            if half <= 0.0 {
                return 0.0;
            }
            if x <= half {
                w0 * x / half
            } else {
                w0 * (l_total - x) / half
            }
        }
        LoadShape::Trapezoid { w0, a, .. } => {
            if *a <= 1e-12 {
                *w0
            } else if x < *a {
                w0 * x / a
            } else if x > l_total - a {
                w0 * (l_total - x) / a
            } else {
                *w0
            }
        }
        LoadShape::Point { .. } => 0.0,
    }
}

/// 区間 `[x_lo, x_lo+len]`（`shape` の局所座標系、全長 `l_total`）に作用する荷重の
/// 合力と、その合力作用点までの `x_lo` からの距離（モーメント腕）。
fn zone_load_resultant(shape: &LoadShape, l_total: f64, x_lo: f64, len: f64) -> (f64, f64) {
    if len <= 0.0 {
        return (0.0, 0.0);
    }
    match shape {
        LoadShape::Point { p, x } => {
            if *x >= x_lo && *x <= x_lo + len {
                (*p, x - x_lo)
            } else {
                (0.0, 0.0)
            }
        }
        _ => {
            let total = simpson_integrate(
                |xi| shape_intensity(shape, l_total, x_lo + xi),
                0.0,
                len,
                SIMPSON_N,
            );
            if total.abs() < 1e-12 {
                return (0.0, 0.0);
            }
            let moment = simpson_integrate(
                |xi| shape_intensity(shape, l_total, x_lo + xi) * xi,
                0.0,
                len,
                SIMPSON_N,
            );
            (total, moment / total)
        }
    }
}

/// 可撓区間（長さ `l_flex`、`shape` の局所座標で `[x_start, x_start+l_flex]` に相当）に
/// 切り出した荷重による、その可撓区間だけを長さ `l_flex` の両端固定梁とみなした場合の
/// 固定端モーメント・せん断（`C'`・`Q'`）。一般に非対称（`lam_i ≠ lam_j` で切り出した場合）
/// となるため、対称専用ではない一般公式を用いる:
///   FEM_i = (1/L'²)∫w(ξ)ξ(L'−ξ)²dξ,  FEM_j = (1/L'²)∫w(ξ)ξ²(L'−ξ)dξ
///   Q_i = R_i0 + (FEM_i−FEM_j)/L',   Q_j = R_j0 − (FEM_i−FEM_j)/L'
/// （R_i0, R_j0 は単純梁反力）。`lam_i = lam_j` の対称切り出しでは FEM_i=FEM_j となり
/// 既存の対称専用式（`c_j=−c_i`, `q_i=q_j`）に一致する。
fn cmq_flexible_span(shape: &LoadShape, l_total: f64, x_start: f64, l_flex: f64) -> Cmq {
    match shape {
        LoadShape::Uniform { w } => fem_uniform(*w, l_flex),
        LoadShape::Point { p, x } => {
            let xi = x - x_start;
            if xi < 0.0 || xi > l_flex {
                return Cmq {
                    c_i: 0.0,
                    c_j: 0.0,
                    q_i: 0.0,
                    q_j: 0.0,
                };
            }
            let a = xi;
            let b = l_flex - xi;
            let fem_i = *p * a * b * b / (l_flex * l_flex);
            let fem_j = *p * a * a * b / (l_flex * l_flex);
            let r_i0 = *p * b / l_flex;
            let r_j0 = *p * a / l_flex;
            let delta = (fem_i - fem_j) / l_flex;
            Cmq {
                c_i: fem_i,
                c_j: -fem_j,
                q_i: r_i0 + delta,
                q_j: r_j0 - delta,
            }
        }
        _ => {
            let intensity = |xi: f64| shape_intensity(shape, l_total, x_start + xi);
            let fem_i = simpson_integrate(
                |xi| intensity(xi) * xi * (l_flex - xi).powi(2),
                0.0,
                l_flex,
                SIMPSON_N,
            ) / (l_flex * l_flex);
            let fem_j = simpson_integrate(
                |xi| intensity(xi) * xi * xi * (l_flex - xi),
                0.0,
                l_flex,
                SIMPSON_N,
            ) / (l_flex * l_flex);
            let r_i0 =
                simpson_integrate(|xi| intensity(xi) * (l_flex - xi), 0.0, l_flex, SIMPSON_N)
                    / l_flex;
            let total = simpson_integrate(intensity, 0.0, l_flex, SIMPSON_N);
            let r_j0 = total - r_i0;
            let delta = (fem_i - fem_j) / l_flex;
            Cmq {
                c_i: fem_i,
                c_j: -fem_j,
                q_i: r_i0 + delta,
                q_j: r_j0 - delta,
            }
        }
    }
}

/// 剛域部分の荷重計算方法（3 方式）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidZoneCmqMode {
    /// 「剛域を考慮する（剛域部外力はCMoQに加算する）」: 剛域内の荷重を当該端の C・Q に加算する。
    IncludeInCmq,
    /// 「剛域を考慮する（剛域部外力は柱に伝達する）」: 剛域内の荷重は柱への集中荷重として
    /// `column_loads` に集計し、梁の CMQ には含めない。
    TransferToColumn,
    /// 剛域内の荷重を無視する（簡易評価用）。
    Ignore,
}

pub struct RigidZoneCmqResult {
    pub cmq: Cmq,
    /// 剛域内荷重を柱へ伝達する場合の (i側, j側) 集中荷重。`TransferToColumn` 以外は `(0.0, 0.0)`。
    pub column_loads: (f64, f64),
}

/// 剛域を考慮した大梁の CMQ。
///
/// アルゴリズム:
/// 1. 可撓長 `L' = L − λi − λj` の区間に切り出した荷重で `C'/Q'` を求める
///    （[`cmq_flexible_span`]）。等分布はそのまま同じ強度、三角形・台形は切り出し区間の
///    荷重を数値積分（合成シンプソン則）で評価する。
/// 2. 可撓部の端部応力 `C', Q'` を、剛域を片持ち梁とみなしてその先端（可撓部端）から
///    節点（剛域基部）へ伝達する: `C_i = C'_i + Q'_i·λi`、`Q_i = Q'_i`
///    （j側は符号規約 `c_j=−c_i` に整合させ `C_j = C'_j − Q'_j·λj`、`Q_j = Q'_j`）。
/// 3. 剛域内に直接作用する荷重成分（区間 `[0,λi]`・`[L−λj,L]`）は `mode` により:
///    - `IncludeInCmq`: 剛域内荷重 `W` と荷重重心から節点までのモーメント腕 `x̄` を用いて
///      `C_i += W_i·x̄i`、`Q_i += W_i`（j側も同様、符号は `c_j` の向きに合わせて減算）。
///    - `TransferToColumn`: `column_loads` に集計し、CMQ には加えない。
///    - `Ignore`: 無視する（CMQ にも column_loads にも計上しない＝荷重を捨てる）。
///
/// `λi = λj = 0` のとき、いずれの `mode` でも `cmq` は既存の `fem_uniform`/`fem_triangle`/
/// `fem_trapezoid` と厳密に一致する（剛域内荷重・柱集中荷重ともにゼロになるため）。
pub fn cmq_with_rigid_zone(
    shape: &LoadShape,
    l_total: f64,
    lam_i: f64,
    lam_j: f64,
    mode: RigidZoneCmqMode,
) -> RigidZoneCmqResult {
    let l_flex = l_total - lam_i - lam_j;
    if l_flex <= 0.0 {
        return RigidZoneCmqResult {
            cmq: Cmq {
                c_i: 0.0,
                c_j: 0.0,
                q_i: 0.0,
                q_j: 0.0,
            },
            column_loads: (0.0, 0.0),
        };
    }

    // 1) 可撓部分の C'/Q'。
    let flex = cmq_flexible_span(shape, l_total, lam_i, l_flex);

    // 2) 剛域片持ち梁として節点へ伝達。
    let mut c_i = flex.c_i + flex.q_i * lam_i;
    let mut c_j = flex.c_j - flex.q_j * lam_j;
    let mut q_i = flex.q_i;
    let mut q_j = flex.q_j;

    // 3) 剛域内直接荷重。
    let (w_i, xbar_i) = zone_load_resultant(shape, l_total, 0.0, lam_i);
    let (w_j, xbar_j_from_start) = zone_load_resultant(shape, l_total, l_total - lam_j, lam_j);
    let xbar_j = lam_j - xbar_j_from_start; // j端（節点）からの距離に変換

    let mut column_loads = (0.0, 0.0);
    match mode {
        RigidZoneCmqMode::IncludeInCmq => {
            c_i += w_i * xbar_i;
            q_i += w_i;
            c_j -= w_j * xbar_j;
            q_j += w_j;
        }
        RigidZoneCmqMode::TransferToColumn => {
            column_loads = (w_i, w_j);
        }
        RigidZoneCmqMode::Ignore => {}
    }

    RigidZoneCmqResult {
        cmq: Cmq { c_i, c_j, q_i, q_j },
        column_loads,
    }
}

#[cfg(test)]
mod tests;
