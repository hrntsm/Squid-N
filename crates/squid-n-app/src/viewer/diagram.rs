//! 部材ローカルに沿った N/Q/M 図（内力分布図）の描画。
//!
//! 図は要素ローカル y 軸方向（曲げ平面内）へワールド空間で張り出してから投影する。
//! スクリーン上の部材線の法線ではなくワールドの要素軸を使うため、ビューを回転
//! しても図の張り出し面は要素座標系に追従する（曲げ平面を真横から見ると線に潰れる）。
//!
//! ## 塗りつぶしの方式（凸多角形の扇状分割問題への対策）
//!
//! `egui::Shape::convex_polygon` は頂点0からの扇状三角形分割で塗りつぶすため、
//! 「材軸→各張り出し点→材軸」を 1 つの閉多角形として渡すと、M 図のように非凸・
//! 符号反転（両端 hogging＋中央 sagging 等）を含む形状では誤った領域が塗られる。
//! これを避けるため、隣接サンプル区間ごとに「材軸上の2点＋張り出し点2点」の
//! 台形（符号反転区間はゼロ交差で2つの三角形に分割）を個別に塗る
//! （[`diagram_fill_polygons`]）。台形・三角形は必ず凸なので扇状分割の問題が出ない。
//!
//! 輪郭（実線）は閉じない `egui::Shape::line` の折れ線として描く。塗り＋輪郭を
//! 1 つの閉じたポリゴン（Stroke 付き）にする方式だと、値が極小の部材で図形が
//! 材軸上にほぼ潰れ、材軸と張り出し線が浅い角度で接する折り返し点で epaint の
//! マイター結合が発散し、部材軸方向へ画面外まで伸びるスパイク描画になる
//! （CMQ 図の [`super::paint_diagram_polygon`] と同じ問題・同じ対策）。

use crate::app::App;
use crate::theme;

use super::{diagram_offset_dir, member_len3, BeamDeflection, Projector, ViewMode};

/// 張り出しピークがこの px 未満の図形は描かない。60px 正規化に対して値が
/// 相対的に極小の部材（ほぼ潰れた図形）は、輪郭の折り返し点で epaint のマイター
/// 結合が発散し部材軸方向に画面外まで伸びるスパイク描画になるため、視認不能な
/// 図形は端から描かずスキップする。N/Q/M 図・CMQ 図で共有する。
pub(super) const MIN_DIAGRAM_PX: f32 = 0.5;

/// 輪郭の折れ線で、直前の点とのスクリーン距離がこの px 未満の連続点は間引く
/// （ゼロ長セグメントも epaint のマイター結合発散の原因になるため）。
/// N/Q/M 図・CMQ 図で共有する。
pub(super) const MIN_SEGMENT_PX: f32 = 0.25;

/// コンター表示時、各サンプル区間を細分する分割数（滑らかな色階調のため）。
const CONTOUR_SUBDIV: usize = 8;

/// `(xi, val)` のサンプル列から、塗りつぶし用の凸ポリゴン列を作る（純粋関数）。
///
/// 隣接サンプル区間ごとに `subdiv` 分割し、各細分区間を「材軸上の2点（val=0）＋
/// 張り出し点2点」の台形として返す。区間内で符号が反転する場合は、線形補間で
/// ゼロ交差 `xc = x0 + (x1-x0)*v0/(v0-v1)` を求めて2つの三角形（それぞれ片側符号
/// のみ）に分割する。戻り値の各ポリゴンは xi 昇順の点列で、値の符号は単一
/// （片側符号のみ、または境界でちょうど 0）になる。
///
/// `samples` は呼び出し側の保険として関数内で xi 昇順にソートする。
pub(crate) fn diagram_fill_polygons(samples: &[(f64, f64)], subdiv: usize) -> Vec<Vec<(f64, f64)>> {
    let subdiv = subdiv.max(1);
    if samples.len() < 2 {
        return Vec::new();
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut polys = Vec::new();
    for w in sorted.windows(2) {
        let (x0, v0) = w[0];
        let (x1, v1) = w[1];
        for k in 0..subdiv {
            let ta = k as f64 / subdiv as f64;
            let tb = (k + 1) as f64 / subdiv as f64;
            let xa = x0 + (x1 - x0) * ta;
            let xb = x0 + (x1 - x0) * tb;
            let va = v0 + (v1 - v0) * ta;
            let vb = v0 + (v1 - v0) * tb;
            push_segment_fill(xa, va, xb, vb, &mut polys);
        }
    }
    polys
}

/// 1 細分区間 `(x0,v0)-(x1,v1)` の塗りポリゴンを `out` へ積む。
/// 同符号（またはどちらか一方が 0）なら台形 1 つ、符号反転ならゼロ交差で
/// 2 つの三角形に分割する。両端が 0（面積なし）の場合は何も積まない。
fn push_segment_fill(x0: f64, v0: f64, x1: f64, v1: f64, out: &mut Vec<Vec<(f64, f64)>>) {
    if v0 == 0.0 && v1 == 0.0 {
        return;
    }
    if (v0 >= 0.0 && v1 >= 0.0) || (v0 <= 0.0 && v1 <= 0.0) {
        out.push(vec![(x0, 0.0), (x0, v0), (x1, v1), (x1, 0.0)]);
    } else {
        // 符号反転: 線形補間でゼロ交差を求め、片側符号のみの三角形2つに分割する
        let xc = x0 + (x1 - x0) * v0 / (v0 - v1);
        out.push(vec![(x0, 0.0), (x0, v0), (xc, 0.0)]);
        out.push(vec![(xc, 0.0), (x1, v1), (x1, 0.0)]);
    }
}

/// ポリゴン（[`diagram_fill_polygons`] の1要素）の代表値（中点値）。
/// 材軸上の点（val=0）を除く頂点の値の平均。コンター着色の基準に使う。
fn diagram_poly_repr_val(poly: &[(f64, f64)]) -> f64 {
    let nz: Vec<f64> = poly
        .iter()
        .map(|&(_, v)| v)
        .filter(|v| v.abs() > 1e-15)
        .collect();
    if nz.is_empty() {
        0.0
    } else {
        nz.iter().sum::<f64>() / nz.len() as f64
    }
}

/// コンター配色: `t = val/max_abs ∈ [-1,1]`（範囲外はクランプ）を `map` の色へ写像する。
/// TONMANUAL §3「カラーマップ（連続値）」は Viridis を既定に定めており（[`theme::ColorMap`]
/// の `#[default]`）、UI からは他のカラーマップにも切り替えられる。実体は
/// `map.sample((t+1)/2)` への単純な写像で、独自の配色は持たない（テーマ＝配色の
/// 単一情報源は theme.rs 側に置く）。
pub(crate) fn contour_color(t: f64, map: theme::ColorMap) -> egui::Color32 {
    let t = (t as f32).clamp(-1.0, 1.0);
    map.sample((t + 1.0) * 0.5)
}

/// 部材ローカルに沿って N/Q/M 図を描く。
pub(super) fn draw_force_diagram(
    painter: &egui::Painter,
    app: &App,
    mode: ViewMode,
    coords3: &[[f64; 3]],
    disp: Option<&[[f64; 6]]>,
    deform_scale: f64,
    proj: &Projector,
) {
    let scale = proj.scale();
    let force_idx = match mode {
        ViewMode::N => 0, // N
        ViewMode::Q => 1, // Qy
        ViewMode::M => 5, // Mz
        _ => return,
    };
    let label = match mode {
        ViewMode::N => "N",
        ViewMode::Q => "Q",
        ViewMode::M => "M",
        _ => "",
    };

    let Some(results) = &app.results else {
        return;
    };
    let max_abs = results
        .member_forces
        .iter()
        .flat_map(|(_, mf)| mf.at.iter().map(|(_, f)| f[force_idx].abs()))
        .fold(0.0_f64, f64::max);
    if max_abs < 1e-12 {
        return;
    }
    // 最大値で 60px 相当のワールド長（一様スケール正射影なので px/scale=ワールド長）
    let amp_world = 60.0 / max_abs / scale as f64;

    let contour = app.diagram_contour;
    let colormap = app.contour_colormap;
    // コンター時は色の階調のため各区間を細分する。モノクロ時は単色なので不要。
    let subdiv = if contour { CONTOUR_SUBDIV } else { 1 };
    // 塗りの不透明度: コンターは色そのものが情報を持つためモノクロより濃くする。
    let fill_alpha: u8 = if contour { 160 } else { 60 };
    // 輪郭: コンター時は色と干渉しないよう中立なグレーの細線にする。
    let outline_color = if contour {
        theme::GRAY_600
    } else {
        theme::DATA_BLUE
    };
    let outline_width: f32 = if contour { 1.0 } else { 1.5 };

    for (elem_id, mf) in &results.member_forces {
        let elem = app.model.elements.iter().find(|e| e.id == *elem_id);
        let Some(elem) = elem else { continue };
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= coords3.len() || n1 >= coords3.len() {
            continue;
        }
        let p_i = coords3[n0];
        let p_j = coords3[n1];
        if member_len3(p_i, p_j) < 1e-9 {
            continue; // ゼロ長部材（同一節点間）は材軸が定まらず図を描けない
        }
        let ref_vec = elem.local_axis.ref_vector;
        let ey = diagram_offset_dir(p_i, p_j, ref_vec);
        // 内部たわみ表示が有効な梁は、張り出しの基準線を変形後の Hermite 曲線に
        // する（`disp` が Some＝変形重ね時のみ）。梁の線描画と同じ `BeamDeflection`
        // で評価するため、基準線が梁の描画曲線に厳密一致する。それ以外（梁以外・
        // 内部たわみ OFF・変形重ね無し）は変形後の節点間直線（弦）を基準線にする
        // （従来どおり）。未変形材軸端点から一度だけ前処理する。
        let deflection: Option<BeamDeflection> =
            if app.show_beam_interpolation && elem.kind == squid_n_core::model::ElementKind::Beam {
                disp.and_then(|d| {
                    (n0 < d.len() && n1 < d.len()).then(|| {
                        BeamDeflection::new(
                            app.model.nodes[n0].coord,
                            app.model.nodes[n1].coord,
                            d[n0],
                            d[n1],
                            ref_vec,
                        )
                    })
                })
            } else {
                None
            };
        let p0 = proj.project(p_i);
        let p1 = proj.project(p_j);

        // xi 昇順にソート（保険）
        let mut samples: Vec<(f64, f64)> =
            mf.at.iter().map(|(xi, f)| (*xi, f[force_idx])).collect();
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        if samples.len() < 2 {
            continue;
        }
        // 張り出しピーク px が閾値未満の潰れた図形はスキップ（上記ドキュメント参照）
        let val_max = samples.iter().map(|(_, v)| v.abs()).fold(0.0_f64, f64::max);
        let peak_px = (60.0 * val_max / max_abs) as f32;
        if peak_px < MIN_DIAGRAM_PX {
            continue;
        }

        // (xi, val) → スクリーン座標。val=0 は基準線そのもの（オフセット無し）。
        // 基準線は deflection があれば梁の変形後 Hermite 曲線、無ければ節点間直線。
        let to_screen = |xi: f64, val: f64| -> egui::Pos2 {
            let base3 = match &deflection {
                Some(bd) => bd.point_at(xi, deform_scale),
                None => [
                    p_i[0] + (p_j[0] - p_i[0]) * xi,
                    p_i[1] + (p_j[1] - p_i[1]) * xi,
                    p_i[2] + (p_j[2] - p_i[2]) * xi,
                ],
            };
            if val == 0.0 {
                proj.project(base3)
            } else {
                proj.project_offset(base3, ey, -val * amp_world)
            }
        };

        // --- 塗り: 台形/三角形クワッドを個別に塗る（非凸・符号反転にも正しく対応） ---
        for poly in diagram_fill_polygons(&samples, subdiv) {
            let screen_poly: Vec<egui::Pos2> =
                poly.iter().map(|&(xi, v)| to_screen(xi, v)).collect();
            let fill_color = if contour {
                let repr = diagram_poly_repr_val(&poly) / max_abs;
                theme::translucent(contour_color(repr, colormap), fill_alpha)
            } else {
                theme::translucent(theme::DATA_BLUE, fill_alpha)
            };
            painter.add(egui::Shape::convex_polygon(
                screen_poly,
                fill_color,
                egui::Stroke::NONE,
            ));
        }

        // --- 輪郭: 閉じない折れ線（材軸点→各張り出し点→材軸点。マイター発散対策） ---
        let mut outline_pts: Vec<egui::Pos2> = Vec::with_capacity(samples.len() + 2);
        outline_pts.push(p0);
        let mut last = p0;
        for &(xi, val) in &samples {
            let pt = to_screen(xi, val);
            // 直前の点とスクリーン距離が近すぎるサンプル点は間引く
            if (pt.x - last.x).hypot(pt.y - last.y) < MIN_SEGMENT_PX {
                continue;
            }
            last = pt;
            outline_pts.push(pt);
        }
        outline_pts.push(p1);
        painter.add(egui::Shape::line(
            outline_pts,
            egui::Stroke::new(outline_width, outline_color),
        ));
    }

    // 凡例
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        format!("{}図 (max={:.2})", label, max_abs),
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
    if contour {
        draw_contour_legend(painter, max_abs, colormap);
    }
}

/// コンター表示時、凡例の下にカラーバー（グラデーション）を描く。
/// 横 160px×縦 10px 程度のバーを短冊状に並べて描き、左端 −max・中央 0・右端 +max
/// のラベルを添える。
fn draw_contour_legend(painter: &egui::Painter, max_abs: f64, colormap: theme::ColorMap) {
    const BAR_W: f32 = 160.0;
    const BAR_H: f32 = 10.0;
    const STRIPS: usize = 32;

    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let y0 = rect.min.y + 30.0;

    for i in 0..STRIPS {
        let t = (i as f64 + 0.5) / STRIPS as f64 * 2.0 - 1.0; // 短冊中央の t∈[-1,1]
        let color = contour_color(t, colormap);
        let sx0 = x0 + (i as f32 / STRIPS as f32) * BAR_W;
        let sx1 = x0 + ((i + 1) as f32 / STRIPS as f32) * BAR_W;
        painter.rect_filled(
            egui::Rect::from_min_max(egui::pos2(sx0, y0), egui::pos2(sx1, y0 + BAR_H)),
            0.0,
            color,
        );
    }

    let font = egui::FontId::proportional(11.0);
    painter.text(
        egui::pos2(x0, y0 + BAR_H + 2.0),
        egui::Align2::LEFT_TOP,
        format!("-{:.2}", max_abs),
        font.clone(),
        theme::GRAY_600,
    );
    painter.text(
        egui::pos2(x0 + BAR_W * 0.5, y0 + BAR_H + 2.0),
        egui::Align2::CENTER_TOP,
        "0",
        font.clone(),
        theme::GRAY_600,
    );
    painter.text(
        egui::pos2(x0 + BAR_W, y0 + BAR_H + 2.0),
        egui::Align2::RIGHT_TOP,
        format!("{:.2}", max_abs),
        font,
        theme::GRAY_600,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 単調増加（同符号）の区間は、各細分区間が台形（材軸2点＋張り出し2点）になり、
    /// 符号は常に非負（片側符号のみ）であることを確認する。
    #[test]
    fn diagram_fill_polygons_same_sign_is_trapezoids() {
        let samples = [(0.0, 1.0), (1.0, 3.0)];
        let polys = diagram_fill_polygons(&samples, 1);
        assert_eq!(polys.len(), 1);
        let poly = &polys[0];
        assert_eq!(poly.len(), 4);
        // 材軸上の点（val=0）を含む
        assert!(poly.iter().any(|&(_, v)| v == 0.0));
        // 片側符号のみ（すべて非負）
        assert!(poly.iter().all(|&(_, v)| v >= 0.0));
    }

    /// 符号反転区間は、ゼロ交差で2つの片側符号のみの三角形に分割される。
    #[test]
    fn diagram_fill_polygons_sign_change_splits_at_zero_crossing() {
        // v0=1, v1=-1 → 中点 xi=0.5 でゼロ交差
        let samples = [(0.0, 1.0), (1.0, -1.0)];
        let polys = diagram_fill_polygons(&samples, 1);
        assert_eq!(polys.len(), 2);

        // 1つ目は正側のみ、2つ目は負側のみ
        assert!(polys[0].iter().all(|&(_, v)| v >= 0.0));
        assert!(polys[1].iter().all(|&(_, v)| v <= 0.0));

        // どちらも材軸上の点（val=0）を含む
        assert!(polys[0].iter().any(|&(_, v)| v == 0.0));
        assert!(polys[1].iter().any(|&(_, v)| v == 0.0));

        // ゼロ交差位置 xi=0.5 がどちらのポリゴンにも現れる
        let xc_in_first = polys[0]
            .iter()
            .any(|&(xi, v)| v == 0.0 && (xi - 0.5).abs() < 1e-9);
        let xc_in_second = polys[1]
            .iter()
            .any(|&(xi, v)| v == 0.0 && (xi - 0.5).abs() < 1e-9);
        assert!(xc_in_first && xc_in_second);
    }

    /// 全体が単一符号内で常に正しい向き（片側符号のみ）を保つことを、
    /// 複数区間・複数符号反転を含む折れ線でまとめて確認する。
    #[test]
    fn diagram_fill_polygons_multi_segment_each_poly_single_sign() {
        // 両端 hogging（負）・中央 sagging（正）のような M 図の典型形状
        let samples = [(0.0, -2.0), (0.5, 1.0), (1.0, -2.0)];
        let polys = diagram_fill_polygons(&samples, 4);
        assert!(!polys.is_empty());
        for poly in &polys {
            let has_pos = poly.iter().any(|&(_, v)| v > 1e-12);
            let has_neg = poly.iter().any(|&(_, v)| v < -1e-12);
            // 同一ポリゴン内に正と負が同時に現れてはいけない（片側符号のみ）
            assert!(!(has_pos && has_neg), "poly has mixed sign: {:?}", poly);
        }
    }

    /// サンプル列が xi 降順で渡されても、関数内で昇順ソートされ結果が変わらない。
    #[test]
    fn diagram_fill_polygons_sorts_unsorted_input() {
        let sorted = diagram_fill_polygons(&[(0.0, 1.0), (1.0, 3.0)], 1);
        let unsorted = diagram_fill_polygons(&[(1.0, 3.0), (0.0, 1.0)], 1);
        assert_eq!(sorted, unsorted);
    }

    /// サンプルが2点未満なら空を返す。
    #[test]
    fn diagram_fill_polygons_needs_at_least_two_samples() {
        assert!(diagram_fill_polygons(&[], 1).is_empty());
        assert!(diagram_fill_polygons(&[(0.0, 1.0)], 1).is_empty());
    }

    /// Viridis（既定カラーマップ）で t=-1 は濃紫端（#440154）、t=+1 は黄端（#FDE725）、
    /// t=0 は中央の青緑（#26828E）へ写像される。
    #[test]
    fn contour_color_endpoints_and_neutral() {
        let map = theme::ColorMap::Viridis;
        assert_eq!(
            contour_color(-1.0, map),
            egui::Color32::from_rgb(0x44, 0x01, 0x54)
        );
        assert_eq!(
            contour_color(1.0, map),
            egui::Color32::from_rgb(0xFD, 0xE7, 0x25)
        );
        assert_eq!(
            contour_color(0.0, map),
            egui::Color32::from_rgb(0x26, 0x82, 0x8E)
        );
    }

    /// 範囲外の値はクランプされる（t<-1 は t=-1 と同じ、t>1 は t=1 と同じ）。
    #[test]
    fn contour_color_clamps_out_of_range() {
        let map = theme::ColorMap::Viridis;
        assert_eq!(contour_color(-5.0, map), contour_color(-1.0, map));
        assert_eq!(contour_color(5.0, map), contour_color(1.0, map));
    }

    /// Viridis は t の増加とともに G 成分が単調非減少（濃紫→青緑→黄で緑みが増す）。
    #[test]
    fn contour_color_green_channel_is_monotonic() {
        let map = theme::ColorMap::Viridis;
        let ts = [-1.0, -0.5, 0.0, 0.5, 1.0];
        let greens: Vec<u8> = ts.iter().map(|&t| contour_color(t, map).g()).collect();
        for w in greens.windows(2) {
            assert!(w[0] <= w[1], "G成分が非単調: {:?}", greens);
        }
    }

    /// カラーマップを切り替えると異なる色になる（同じ t でも Viridis と Jet で異なる）。
    #[test]
    fn contour_color_respects_selected_colormap() {
        let viridis = contour_color(0.0, theme::ColorMap::Viridis);
        let jet = contour_color(0.0, theme::ColorMap::Jet);
        assert_ne!(viridis, jet);
    }
}
