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
/// 入隅の片持ちスラブはマニュアル自体が非対応と明記しており、本実装でも未対応。
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

/// 出隅の片持ちスラブの分配（`SlabKind::Corner`。マニュアル「出隅の片持ちスラブ」）。
///
/// 「出隅の片持ちスラブの荷重は、荷重伝達方向および片持ち梁の取付きに関わらず、節点荷重
/// としてすべて柱に伝達します」との記載どおり、荷重伝達方向（`one_way`）や
/// `slab.edge_supported`（片持ち梁の有無）を一切参照せず、全荷重
/// `W = w × 多角形面積`（[`polygon_area`]。マニュアル「出隅の重量は、構造芯から出隅先端
/// までの長方形について計算します」＝境界そのものの面積）を柱（`boundary[0]` の節点）への
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
/// **簡易モデルの仮定**（本実装の割り切り。マニュアルの厳密な二方向効果は考慮しない）:
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

/// 片持ちスラブの分配（`SlabKind::Cantilever`。マニュアル「片持ちスラブ」）。
///
/// 4頂点を想定し、境界辺0（`boundary[0]`→`boundary[1]`）を取付き辺（大梁側）、
/// その対辺2を先端とみなす。出し幅 `d` は辺0の直線から頂点2・3までの垂直距離の平均。
/// マニュアル「片持ち梁がない場合: 全て大梁に伝達されます」に従い、取付き辺へ
/// 等分布荷重 `w_line = w·d`（先端まで一様なスラブの単純片持ち反力に相当）として
/// 集約する（`LoadShape::Uniform` + `fem_uniform`）。
///
/// 片持ち梁・先端リブ小梁がある場合の分割伝達は `slab.edge_supported` を指定することで
/// [`distribute_polygon_supported`] 経路（[`distribute_slab`] 側で分岐）が担う。
/// 出隅の片持ちスラブは `SlabKind::Corner`（[`distribute_corner`]）が別途担う。
///
/// **未対応（残課題）**: 入隅の片持ちスラブ（マニュアル自体が非対応と明記）。
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
/// 小梁の分割伝達」「一般スラブの部分支持（開口際等）」対応。マニュアル「片持ち梁がある
/// 場合：…スラブと同様のルールにより分割して荷重伝達されます」）。
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
// 剛域考慮 CMQ（レビュー §1.13 ギャップ「剛域考慮 CMQ」対応。マニュアル「剛域の考慮」）
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

/// 剛域部分の荷重計算方法（マニュアル「剛域の考慮」の3方式）。
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

/// 剛域を考慮した大梁の CMQ（マニュアル「剛域の考慮」）。
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
mod tests {
    use super::*;

    #[test]
    fn test_fem_uniform() {
        let cmq = fem_uniform(10.0, 4000.0);
        let expected = 10.0 * 4000.0_f64.powi(2) / 12.0;
        assert!((cmq.c_i - expected).abs() < 1e-6);
        assert_eq!(cmq.q_i, 10.0 * 4000.0 / 2.0);
    }

    #[test]
    fn test_fem_triangle_spec() {
        let w0 = 10.0_f64;
        let l = 4000.0_f64;
        let cmq = fem_triangle(w0, l);
        let expected = 5.0 * w0 * l.powi(2) / 96.0;
        assert!(
            (cmq.c_i - expected).abs() < 1e-3,
            "FEM={} expected={}",
            cmq.c_i,
            expected
        );
        assert!((expected - 8.3333e6).abs() < 1.0e3, "expected={}", expected);
    }

    #[test]
    fn test_fem_trapezoid_limits() {
        let w0 = 10.0_f64;
        let l = 6000.0_f64;
        // a→L/2（中央区間消滅）→ 対称三角形 5w0L²/96
        let tri_limit = fem_trapezoid(w0, l / 2.0, 0.0, l);
        let expected_tri = 5.0 * w0 * l.powi(2) / 96.0;
        assert!(
            (tri_limit.c_i - expected_tri).abs() / expected_tri < 1e-9,
            "三角形極限 c_i={} expected={}",
            tri_limit.c_i,
            expected_tri
        );
        // a→0（立上り消滅）→ 等分布 w0L²/12
        let uni_limit = fem_trapezoid(w0, 0.0, l, l);
        let expected_uni = w0 * l.powi(2) / 12.0;
        assert!(
            (uni_limit.c_i - expected_uni).abs() / expected_uni < 1e-9,
            "等分布極限 c_i={} expected={}",
            uni_limit.c_i,
            expected_uni
        );
    }

    #[test]
    fn test_fem_trapezoid_numeric() {
        // 一般の台形を数値積分と照合: FEM = (1/L²)∫ w(x)·x·(L-x)² dx
        let w0 = 7.0_f64;
        let l = 5000.0_f64;
        let a = 1500.0_f64;
        let cmq = fem_trapezoid(w0, a, l - 2.0 * a, l);
        let n = 2_000_000;
        let dx = l / n as f64;
        let mut integral = 0.0;
        let mut total = 0.0;
        for k in 0..n {
            let x = (k as f64 + 0.5) * dx;
            let wx = if x < a {
                w0 * x / a
            } else if x > l - a {
                w0 * (l - x) / a
            } else {
                w0
            };
            integral += wx * x * (l - x).powi(2) * dx;
            total += wx * dx;
        }
        let fem_num = integral / (l * l);
        assert!(
            (cmq.c_i - fem_num).abs() / fem_num < 1e-4,
            "c_i={} 数値積分={}",
            cmq.c_i,
            fem_num
        );
        // せん断 q_i+q_j = 総荷重
        assert!(
            (cmq.q_i + cmq.q_j - total).abs() / total < 1e-4,
            "Q合計={} 総荷重={}",
            cmq.q_i + cmq.q_j,
            total
        );
    }

    fn make_square_slab_model(side: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
        make_rect_slab_model(side, side, method, w)
    }

    fn make_rect_slab_model(lx: f64, ly: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, Node};
        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, 0.0, 0.0),
                mk(1, lx, 0.0),
                mk(2, lx, ly),
                mk(3, 0.0, ly),
            ],
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: None,
            kind: Default::default(),
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method,
        };
        (model, slab)
    }

    fn total_load(loads: &[BeamLoad]) -> f64 {
        // 鉛直釣合いより、各梁の総荷重 = 端せん断の和 q_i + q_j。
        loads.iter().map(|l| l.cmq.q_i + l.cmq.q_j).sum()
    }

    #[test]
    fn test_slab_conservation_square_triangle() {
        // 設計書 §7.3: 1辺 a=4000, w=0.005 → 総和 = w·a² = 80000 N（厳密）
        let w = 0.005_f64;
        let a = 4000.0_f64;
        let (model, slab) = make_square_slab_model(a, DistributionMethod::TriTrapezoid, w);
        let loads = distribute_slab(&model, &slab);
        let expected = w * a * a;
        assert!(
            (total_load(&loads) - expected).abs() < 1e-6,
            "総和={} expected={}",
            total_load(&loads),
            expected
        );
        // 各大梁ピーク強度 w0 = w·a/2 = 10, FEM = 5·w0·a²/96
        for l in &loads {
            if let LoadShape::Triangle { w0 } = l.shape {
                assert!((w0 - 10.0).abs() < 1e-9, "w0={}", w0);
                let fem = 5.0 * w0 * a * a / 96.0;
                assert!((l.cmq.c_i - fem).abs() < 1e-3, "FEM={}", l.cmq.c_i);
            }
        }
    }

    #[test]
    fn test_slab_conservation_rect_all_methods() {
        let w = 0.005_f64;
        let (lx, ly) = (4000.0_f64, 6000.0_f64);
        let expected = w * lx * ly;
        for method in [
            DistributionMethod::TriTrapezoid,
            DistributionMethod::OneWay,
            DistributionMethod::TributaryArea,
        ] {
            let (model, slab) = make_rect_slab_model(lx, ly, method, w);
            let loads = distribute_slab(&model, &slab);
            assert!(
                (total_load(&loads) - expected).abs() / expected < 1e-9,
                "method={:?} 総和={} expected={}",
                method,
                total_load(&loads),
                expected
            );
        }
    }

    // ------------------------------------------------------------------
    // §1.13: 一方向スラブの伝達方向指定
    // ------------------------------------------------------------------

    #[test]
    fn test_one_way_direction_x_and_y() {
        use squid_n_core::model::OneWayDir;
        let w = 0.004_f64;
        let (lx, ly) = (5000.0_f64, 3000.0_f64);
        let expected = w * lx * ly;

        // one_way=Y: 伝達方向Yに直交する辺0・2（X方向の辺、長さlx）が負担。従来互換と同じ結果。
        let (model, mut slab) = make_rect_slab_model(lx, ly, DistributionMethod::OneWay, w);
        slab.one_way = Some(OneWayDir::Y);
        let loads_y = distribute_slab(&model, &slab);
        assert!((total_load(&loads_y) - expected).abs() / expected < 1e-9);
        for l in &loads_y {
            assert!(matches!(
                l.target,
                LoadTarget::Edge(0) | LoadTarget::Edge(2)
            ));
            if let LoadShape::Uniform { w: wl } = l.shape {
                assert!((wl - w * ly / 2.0).abs() / (w * ly / 2.0) < 1e-9);
            }
        }

        // one_way=X: 伝達方向Xに直交する辺1・3（Y方向の辺、長さly）が負担。
        slab.one_way = Some(OneWayDir::X);
        let loads_x = distribute_slab(&model, &slab);
        assert!((total_load(&loads_x) - expected).abs() / expected < 1e-9);
        for l in &loads_x {
            assert!(matches!(
                l.target,
                LoadTarget::Edge(1) | LoadTarget::Edge(3)
            ));
            if let LoadShape::Uniform { w: wl } = l.shape {
                assert!((wl - w * lx / 2.0).abs() / (w * lx / 2.0) < 1e-9);
            }
        }
    }

    // ------------------------------------------------------------------
    // 多角形床組（矩形でない4辺形・五角形）
    // ------------------------------------------------------------------

    fn mk_node(id: u32, x: f64, y: f64) -> squid_n_core::model::Node {
        use squid_n_core::ids::NodeId;
        squid_n_core::model::Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
        }
    }

    fn polygon_slab_model(pts: &[(f64, f64)], method: DistributionMethod, w: f64) -> (Model, Slab) {
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::AreaLoad;
        let nodes: Vec<_> = pts
            .iter()
            .enumerate()
            .map(|(i, (x, y))| mk_node(i as u32, *x, *y))
            .collect();
        let boundary: Vec<NodeId> = (0..pts.len() as u32).map(NodeId).collect();
        let model = Model {
            nodes,
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: None,
            kind: Default::default(),
            one_way: None,
            id: SlabId(0),
            boundary,
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method,
        };
        (model, slab)
    }

    #[test]
    fn test_polygon_trapezoid_conservation() {
        // 矩形でない台形(4頂点、辺2の閉合条件を満たさない) → 多角形経路
        let pts = [
            (0.0, 0.0),
            (6000.0, 0.0),
            (4000.0, 3000.0),
            (1000.0, 3000.0),
        ];
        let w = 0.003_f64;
        let (model, slab) = polygon_slab_model(&pts, DistributionMethod::TriTrapezoid, w);
        // slab_dimensions が None（多角形経路）になることを確認
        assert!(slab_dimensions(&model, &slab).is_none());
        let loads = distribute_slab(&model, &slab);
        assert!(!loads.is_empty());

        let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
        let sampled_area = total_load(&loads) / w;
        let true_area = polygon_area(&coords);
        assert!(
            (sampled_area - true_area).abs() / true_area < 0.01,
            "sampled={} true={}",
            sampled_area,
            true_area
        );
    }

    #[test]
    fn test_polygon_pentagon_conservation() {
        // 凸五角形
        let pts = [
            (0.0, 0.0),
            (5000.0, 0.0),
            (6000.0, 3000.0),
            (2500.0, 5000.0),
            (-1000.0, 3000.0),
        ];
        let w = 0.0025_f64;
        let (model, slab) = polygon_slab_model(&pts, DistributionMethod::TributaryArea, w);
        let loads = distribute_slab(&model, &slab);
        assert!(!loads.is_empty());
        // 辺インデックスが 0..5 の範囲内。
        for l in &loads {
            match l.target {
                LoadTarget::Edge(e) => assert!(e < 5),
                LoadTarget::Node(_) => panic!("polygon path should not emit node targets"),
            }
        }

        let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
        let sampled_area = total_load(&loads) / w;
        let true_area = polygon_area(&coords);
        assert!(
            (sampled_area - true_area).abs() / true_area < 0.01,
            "sampled={} true={}",
            sampled_area,
            true_area
        );
    }

    #[test]
    fn test_polygon_one_way_fallback() {
        // one_way 指定でも非矩形なら多角形経路にフォールバックする。
        use squid_n_core::model::OneWayDir;
        let pts = [
            (0.0, 0.0),
            (6000.0, 0.0),
            (4000.0, 3000.0),
            (1000.0, 3000.0),
        ];
        let w = 0.002_f64;
        let (model, mut slab) = polygon_slab_model(&pts, DistributionMethod::OneWay, w);
        slab.one_way = Some(OneWayDir::X);
        let loads = distribute_slab(&model, &slab);
        let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
        let sampled_area = total_load(&loads) / w;
        let true_area = polygon_area(&coords);
        assert!((sampled_area - true_area).abs() / true_area < 0.01);
    }

    // ------------------------------------------------------------------
    // 片持ちスラブ
    // ------------------------------------------------------------------

    #[test]
    fn test_cantilever_conservation() {
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, SlabKind};
        let (l_attach, depth) = (4000.0_f64, 1500.0_f64);
        let w = 0.003_f64;
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, l_attach, 0.0),
            mk_node(2, l_attach, depth),
            mk_node(3, 0.0, depth),
        ];
        let model = Model {
            nodes,
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: None,
            kind: SlabKind::Cantilever,
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method: DistributionMethod::TriTrapezoid,
        };
        let loads = distribute_slab(&model, &slab);
        assert_eq!(loads.len(), 1);
        let l = &loads[0];
        assert!(matches!(l.target, LoadTarget::Edge(0)));
        let expected_total = w * l_attach * depth; // 矩形なので厳密に一致
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );
        if let LoadShape::Uniform { w: wl } = l.shape {
            assert!((wl - w * depth).abs() / (w * depth) < 1e-9);
        } else {
            panic!("expected uniform shape");
        }
    }

    // ------------------------------------------------------------------
    // 小梁2段階伝達
    // ------------------------------------------------------------------

    #[test]
    fn test_joist_two_stage_transfer_conservation() {
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, JoistLine};
        // 幅方向(X) 9000mm、小梁はY方向に架かり(L_joist=ly=4000)、spacing=3000で
        // 境界から半間隔ずつ離れた2本の小梁（3000,6000）を配置(9000=3*3000)。
        let (lx, ly) = (9000.0_f64, 4000.0_f64);
        let w = 0.0035_f64;
        let spacing = 3000.0_f64;
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, lx, 0.0),
            mk_node(2, lx, ly),
            mk_node(3, 0.0, ly),
            // 小梁支持節点(辺0上, 辺2上)
            mk_node(4, 3000.0, 0.0),
            mk_node(5, 3000.0, ly),
            mk_node(6, 6000.0, 0.0),
            mk_node(7, 6000.0, ly),
        ];
        let model = Model {
            nodes,
            ..Default::default()
        };
        let joists = vec![
            JoistLine {
                dir: [0.0, 1.0],
                spacing,
                support: [NodeId(4), NodeId(5)],
            },
            JoistLine {
                dir: [0.0, 1.0],
                spacing,
                support: [NodeId(6), NodeId(7)],
            },
        ];
        let slab = Slab {
            edge_supported: None,
            kind: Default::default(),
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method: DistributionMethod::TriTrapezoid,
        };
        let loads = distribute_slab(&model, &slab);

        let expected_total = w * lx * ly;
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );

        // 節点反力(小梁): 4本(2小梁×両端)、各 R = w*spacing*ly/2
        let node_entries: Vec<_> = loads
            .iter()
            .filter(|l| matches!(l.target, LoadTarget::Node(_)))
            .collect();
        assert_eq!(node_entries.len(), 4);
        let expected_r = w * spacing * ly / 2.0;
        for l in &node_entries {
            if let LoadShape::Point { p, .. } = l.shape {
                assert!((p - expected_r).abs() / expected_r < 1e-9, "R={}", p);
            } else {
                panic!("expected point load");
            }
        }

        // 境界辺(辺1・3、小梁と平行)は remainder=lx-2*spacing=3000 を折半 → 各1500 = spacing/2
        let edge_entries: Vec<_> = loads
            .iter()
            .filter(|l| matches!(l.target, LoadTarget::Edge(1) | LoadTarget::Edge(3)))
            .collect();
        assert_eq!(edge_entries.len(), 2);
        for l in &edge_entries {
            if let LoadShape::Uniform { w: wl } = l.shape {
                let expected_wl = w * spacing / 2.0;
                assert!((wl - expected_wl).abs() / expected_wl < 1e-9, "wl={}", wl);
            } else {
                panic!("expected uniform load");
            }
        }
    }

    // ------------------------------------------------------------------
    // 剛域考慮CMQ
    // ------------------------------------------------------------------

    #[test]
    fn test_rigid_zone_zero_lambda_matches_existing() {
        let l = 5000.0_f64;
        for mode in [
            RigidZoneCmqMode::IncludeInCmq,
            RigidZoneCmqMode::TransferToColumn,
            RigidZoneCmqMode::Ignore,
        ] {
            // 等分布
            let w = 8.0_f64;
            let res = cmq_with_rigid_zone(&LoadShape::Uniform { w }, l, 0.0, 0.0, mode);
            let expected = fem_uniform(w, l);
            assert!((res.cmq.c_i - expected.c_i).abs() / expected.c_i.abs() < 1e-9);
            assert!((res.cmq.q_i - expected.q_i).abs() / expected.q_i.abs() < 1e-9);
            assert_eq!(res.column_loads, (0.0, 0.0));

            // 三角形
            let w0 = 12.0_f64;
            let res_t = cmq_with_rigid_zone(&LoadShape::Triangle { w0 }, l, 0.0, 0.0, mode);
            let expected_t = fem_triangle(w0, l);
            assert!(
                (res_t.cmq.c_i - expected_t.c_i).abs() / expected_t.c_i.abs() < 1e-4,
                "triangle c_i={} expected={}",
                res_t.cmq.c_i,
                expected_t.c_i
            );
            assert!((res_t.cmq.q_i - expected_t.q_i).abs() / expected_t.q_i.abs() < 1e-4);

            // 台形
            let a = 1200.0_f64;
            let cmq_expected = fem_trapezoid(w0, a, l - 2.0 * a, l);
            let res_tr = cmq_with_rigid_zone(
                &LoadShape::Trapezoid {
                    w0,
                    a,
                    b: l - 2.0 * a,
                },
                l,
                0.0,
                0.0,
                mode,
            );
            assert!(
                (res_tr.cmq.c_i - cmq_expected.c_i).abs() / cmq_expected.c_i.abs() < 1e-4,
                "trapezoid c_i={} expected={}",
                res_tr.cmq.c_i,
                cmq_expected.c_i
            );
            assert!((res_tr.cmq.q_i - cmq_expected.q_i).abs() / cmq_expected.q_i.abs() < 1e-4);
        }
    }

    #[test]
    fn test_rigid_zone_uniform_symmetric_hand_calc() {
        // 全長Lの等分布w、λi=λj=λ（対称）。手計算導出:
        // C_i = wL²/12 + wLλ/6 − wλ²/6 （IncludeInCmqモード）
        let l = 6000.0_f64;
        let lam = 500.0_f64;
        let w = 6.0_f64;
        let res = cmq_with_rigid_zone(
            &LoadShape::Uniform { w },
            l,
            lam,
            lam,
            RigidZoneCmqMode::IncludeInCmq,
        );
        let expected_c = w * l * l / 12.0 + w * l * lam / 6.0 - w * lam * lam / 6.0;
        assert!(
            (res.cmq.c_i - expected_c).abs() / expected_c < 1e-9,
            "c_i={} expected={}",
            res.cmq.c_i,
            expected_c
        );
        // 対称なので c_j = -c_i
        assert!((res.cmq.c_j + res.cmq.c_i).abs() / expected_c < 1e-9);
        // せん断は剛域の有無に関わらず全荷重の半分ずつ(対称・IncludeInCmqで全荷重保存)
        let expected_q = w * l / 2.0;
        assert!((res.cmq.q_i - expected_q).abs() / expected_q < 1e-9);
        assert!((res.cmq.q_j - expected_q).abs() / expected_q < 1e-9);
    }

    #[test]
    fn test_rigid_zone_mode_conservation() {
        // 非対称な剛域長でも、モードによる荷重保存の恒等式が成り立つことを確認。
        let l = 7000.0_f64;
        let lam_i = 300.0_f64;
        let lam_j = 600.0_f64;
        let w = 5.0_f64;
        let total = w * l;

        let include = cmq_with_rigid_zone(
            &LoadShape::Uniform { w },
            l,
            lam_i,
            lam_j,
            RigidZoneCmqMode::IncludeInCmq,
        );
        assert!(
            ((include.cmq.q_i + include.cmq.q_j) - total).abs() / total < 1e-9,
            "IncludeInCmq: q_i+q_j={} total={}",
            include.cmq.q_i + include.cmq.q_j,
            total
        );
        assert_eq!(include.column_loads, (0.0, 0.0));

        let transfer = cmq_with_rigid_zone(
            &LoadShape::Uniform { w },
            l,
            lam_i,
            lam_j,
            RigidZoneCmqMode::TransferToColumn,
        );
        let transfer_total =
            transfer.cmq.q_i + transfer.cmq.q_j + transfer.column_loads.0 + transfer.column_loads.1;
        assert!(
            (transfer_total - total).abs() / total < 1e-9,
            "TransferToColumn total={} expected={}",
            transfer_total,
            total
        );

        let ignore = cmq_with_rigid_zone(
            &LoadShape::Uniform { w },
            l,
            lam_i,
            lam_j,
            RigidZoneCmqMode::Ignore,
        );
        // Ignore と TransferToColumn は梁側 CMQ が一致する（剛域内荷重を加算しない点で同じ）。
        assert!((ignore.cmq.c_i - transfer.cmq.c_i).abs() < 1e-6);
        assert!((ignore.cmq.q_i - transfer.cmq.q_i).abs() < 1e-9);
        assert_eq!(ignore.column_loads, (0.0, 0.0));
        // Ignore は可撓部分の荷重のみを保存する(剛域内荷重は捨てる)。
        let l_flex = l - lam_i - lam_j;
        let expected_flex_total = w * l_flex;
        assert!(
            ((ignore.cmq.q_i + ignore.cmq.q_j) - expected_flex_total).abs() / expected_flex_total
                < 1e-9
        );
    }

    // ------------------------------------------------------------------
    // 出隅の片持ちスラブ（SlabKind::Corner）
    // ------------------------------------------------------------------

    #[test]
    fn test_corner_slab_all_load_to_column_node() {
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, SlabKind};
        let (lx, ly) = (3000.0_f64, 2000.0_f64);
        let w = 0.004_f64;
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, lx, 0.0),
            mk_node(2, lx, ly),
            mk_node(3, 0.0, ly),
        ];
        let model = Model {
            nodes,
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: None,
            kind: SlabKind::Corner,
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method: DistributionMethod::TriTrapezoid,
        };
        let loads = distribute_slab(&model, &slab);
        // 全荷重が単一の節点荷重（boundary[0] = NodeId(0)）としてのみ現れる。
        assert_eq!(loads.len(), 1);
        let l = &loads[0];
        assert_eq!(l.target, LoadTarget::Node(NodeId(0)));
        let expected_total = w * lx * ly;
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );
        if let LoadShape::Point { p, .. } = l.shape {
            assert!((p - expected_total).abs() / expected_total < 1e-9);
        } else {
            panic!("expected point load");
        }
    }

    #[test]
    fn test_corner_slab_ignores_one_way_and_edge_supported() {
        // マニュアル: 出隅の片持ちスラブは伝達方向・片持ち梁の取付きに関わらず全て節点荷重。
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, OneWayDir, SlabKind};
        let (lx, ly) = (2500.0_f64, 1800.0_f64);
        let w = 0.0035_f64;
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, lx, 0.0),
            mk_node(2, lx, ly),
            mk_node(3, 0.0, ly),
        ];
        let model = Model {
            nodes,
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: Some(vec![true, true, true, true]),
            kind: SlabKind::Corner,
            one_way: Some(OneWayDir::X),
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method: DistributionMethod::OneWay,
        };
        let loads = distribute_slab(&model, &slab);
        assert_eq!(loads.len(), 1);
        assert_eq!(loads[0].target, LoadTarget::Node(NodeId(0)));
        let expected_total = w * lx * ly;
        assert!((total_load(&loads) - expected_total).abs() / expected_total < 1e-9);
    }

    // ------------------------------------------------------------------
    // 支持辺指定付き片持ちスラブ（edge_supported、片持ち梁・先端リブ小梁の分割伝達）
    // ------------------------------------------------------------------

    #[test]
    fn test_cantilever_edge_supported_three_of_four_conservation() {
        // 辺0(取付き大梁)・辺1・辺3(片持ち梁)を支持、辺2(先端)は非支持。
        use squid_n_core::ids::{NodeId, SlabId};
        use squid_n_core::model::{AreaLoad, SlabKind};
        let (l_attach, depth) = (4000.0_f64, 1500.0_f64);
        let w = 0.003_f64;
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, l_attach, 0.0),
            mk_node(2, l_attach, depth),
            mk_node(3, 0.0, depth),
        ];
        let model = Model {
            nodes,
            ..Default::default()
        };
        let slab = Slab {
            edge_supported: Some(vec![true, true, false, true]),
            kind: SlabKind::Cantilever,
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method: DistributionMethod::TriTrapezoid,
        };
        let loads = distribute_slab(&model, &slab);
        assert!(!loads.is_empty());
        let expected_total = w * l_attach * depth;
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-6,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );
        // 非支持辺(辺2)には荷重が付かない。
        for l in &loads {
            assert!(!matches!(l.target, LoadTarget::Edge(2)));
        }
        // 支持辺のみ(辺0・1・3)に付く。
        for l in &loads {
            match l.target {
                LoadTarget::Edge(e) => assert!(e == 0 || e == 1 || e == 3),
                LoadTarget::Node(_) => {
                    panic!("edge-supported cantilever should not emit node targets")
                }
            }
        }
    }

    #[test]
    fn test_interior_edge_supported_partial_conservation() {
        // 一般スラブ(Interior)でも edge_supported 指定時は開口際等の非支持辺を除いて分配する。
        let (lx, ly) = (5000.0_f64, 4000.0_f64);
        let w = 0.0025_f64;
        let (model, mut slab) = make_rect_slab_model(lx, ly, DistributionMethod::TriTrapezoid, w);
        // 辺3(開口際等を想定)を非支持とする。
        slab.edge_supported = Some(vec![true, true, true, false]);
        let loads = distribute_slab(&model, &slab);
        assert!(!loads.is_empty());
        let expected_total = w * lx * ly;
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-6,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );
        for l in &loads {
            assert!(!matches!(l.target, LoadTarget::Edge(3)));
        }
    }

    #[test]
    fn test_edge_supported_no_true_falls_back_to_all_edges() {
        // 支持辺が1つも無い指定(全false)は全辺支持へフォールバックする。
        let (lx, ly) = (4000.0_f64, 3000.0_f64);
        let w = 0.002_f64;
        let (model, mut slab) = make_rect_slab_model(lx, ly, DistributionMethod::TriTrapezoid, w);
        slab.edge_supported = Some(vec![false, false, false, false]);
        let loads = distribute_slab(&model, &slab);
        assert!(!loads.is_empty());
        let expected_total = w * lx * ly;
        assert!(
            (total_load(&loads) - expected_total).abs() / expected_total < 1e-6,
            "総和={} expected={}",
            total_load(&loads),
            expected_total
        );
        // フォールバックにより4辺全てに荷重が付き得る(少なくとも1辺は非ゼロ)。
        assert!(loads
            .iter()
            .all(|l| matches!(l.target, LoadTarget::Edge(_))));
    }
}
