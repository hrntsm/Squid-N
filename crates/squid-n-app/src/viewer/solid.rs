//! 断面表示: 線材（梁・柱・ブレース）を断面形状の押し出しソリッドで描画する。
//!
//! 各部材の断面輪郭（局所 y-z 平面の多角形）を材軸に沿って押し出し、
//! 側面を四角形フェイスに分解してカメラ空間の奥行きでソートし（画家の
//! アルゴリズム）、面法線に応じた簡易シェーディングで塗る。egui の
//! ペインタには Z バッファが無いため、フェイス単位の奥行きソートで
//! 前後関係を近似する（凹型断面同士の貫入など厳密でないケースは許容）。
//!
//! 断面の向きは解析と同じ局所座標系（[`LocalFrame`]: ex=材軸,
//! ey=ref_vector 直交化, ez=ex×ey）を用い、輪郭の y がせい方向（ey）、
//! z が幅方向（ez）に対応する。

use crate::theme;
use squid_n_core::model::{ElementKind, Model, Section};
use squid_n_core::section_shape::SectionShape;
use squid_n_element::transform::LocalFrame;

use super::{q_rotate, CameraState};

/// 円形断面（鋼管・RC 円柱等）の輪郭分割数。
const CIRCLE_SEGMENTS: usize = 20;

/// 奥行きソート対象の描画要素。
enum SolidPrim {
    /// 側面の四角形フェイス（塗り＋縁線）
    Quad {
        pts: [egui::Pos2; 4],
        fill: egui::Color32,
        stroke: egui::Color32,
    },
    /// 端面の輪郭線（凹型断面があるため塗らずに線のみ）
    Outline {
        pts: Vec<egui::Pos2>,
        color: egui::Color32,
    },
}

/// カメラ空間座標（r[0]=右, r[1]=上, r[2]=手前）。`project` と同じ回転。
fn cam_space(p: [f64; 3], center3: [f64; 3], cam: &CameraState) -> [f32; 3] {
    let v = [
        (p[0] - center3[0]) as f32,
        (p[1] - center3[1]) as f32,
        (p[2] - center3[2]) as f32,
    ];
    q_rotate(cam.rot, v)
}

/// カメラ空間 → スクリーン座標。`project` の後半と同一の式。
fn to_screen(r: [f32; 3], cam: &CameraState, scale: f32, screen_center: [f32; 2]) -> egui::Pos2 {
    egui::pos2(
        screen_center[0] + cam.pan[0] + r[0] * scale,
        screen_center[1] + cam.pan[1] - r[1] * scale,
    )
}

/// RGB を `shade`（0–1）倍して明度を落とす（アルファは保持）。
fn shaded(c: egui::Color32, shade: f32) -> egui::Color32 {
    let s = shade.clamp(0.0, 1.0);
    egui::Color32::from_rgb(
        (c.r() as f32 * s) as u8,
        (c.g() as f32 * s) as u8,
        (c.b() as f32 * s) as u8,
    )
}

/// フェイス（カメラ空間の四角形）の簡易シェーディング係数。
/// 法線の視線方向成分が大きい（正面向き）ほど明るくする。
fn face_shade(quad: &[[f32; 3]; 4]) -> f32 {
    let e1 = [
        quad[1][0] - quad[0][0],
        quad[1][1] - quad[0][1],
        quad[1][2] - quad[0][2],
    ];
    let e2 = [
        quad[3][0] - quad[0][0],
        quad[3][1] - quad[0][1],
        quad[3][2] - quad[0][2],
    ];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len < 1e-9 {
        return 1.0;
    }
    0.45 + 0.55 * (n[2].abs() / len)
}

/// 断面の基本色。RC/SRC 系はコンクリートのグレー、鋼・CFT 系はスチールブルー。
fn base_color(shape: Option<&SectionShape>) -> egui::Color32 {
    match shape {
        Some(
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. },
        ) => theme::GRAY_300,
        Some(_) => theme::BLUE_300,
        // 形状定義なし（カタログ数値直入力等）は中立グレー
        None => theme::GRAY_300,
    }
}

/// 中心 (0,0)・全せい `d`（y 方向）× 全幅 `b`（z 方向）の矩形輪郭。
fn rect_outline(d: f64, b: f64) -> Vec<[f64; 2]> {
    let hy = d * 0.5;
    let hz = b * 0.5;
    vec![[hy, -hz], [hy, hz], [-hy, hz], [-hy, -hz]]
}

/// 直径 `dia` の円形輪郭（多角形近似）。
fn circle_outline(dia: f64) -> Vec<[f64; 2]> {
    let r = dia * 0.5;
    (0..CIRCLE_SEGMENTS)
        .map(|i| {
            let t = i as f64 / CIRCLE_SEGMENTS as f64 * std::f64::consts::TAU;
            [r * t.cos(), r * t.sin()]
        })
        .collect()
}

/// 断面輪郭を局所 (y, z) 座標 [mm] の閉多角形として返す（末尾は先頭に接続）。
/// y=せい方向（局所 ey）、z=幅方向（局所 ez）。形状定義が無い場合は
/// `depth`×`width` の矩形にフォールバックし、それも無ければ None。
fn section_outline(sec: &Section) -> Option<Vec<[f64; 2]>> {
    let Some(shape) = &sec.shape else {
        // 形状なし: 断面表の depth/width が入っていれば矩形で近似
        return (sec.depth > 0.0 && sec.width > 0.0).then(|| rect_outline(sec.depth, sec.width));
    };
    let outline = match shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let (h, b, tw, tf) = (height * 0.5, width * 0.5, web_thick * 0.5, *flange_thick);
            vec![
                [h, -b],
                [h, b],
                [h - tf, b],
                [h - tf, tw],
                [-h + tf, tw],
                [-h + tf, b],
                [-h, b],
                [-h, -b],
                [-h + tf, -b],
                [-h + tf, -tw],
                [h - tf, -tw],
                [h - tf, -b],
            ]
        }
        SectionShape::SteelBox { height, width, .. } => rect_outline(*height, *width),
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            // 垂直脚 leg_a（y 方向）× 水平脚 leg_b（z 方向）。バウンディングボックス中心合わせ。
            let (a, b, t) = (*leg_a, *leg_b, *thick);
            let (cy, cz) = (a * 0.5, b * 0.5);
            vec![
                [-cy, -cz],
                [-cy, b - cz],
                [t - cy, b - cz],
                [t - cy, t - cz],
                [a - cy, t - cz],
                [a - cy, -cz],
            ]
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            // ウェブを -z 側に置き、開口を +z 側へ向ける
            let (h, b, tw, tf) = (height * 0.5, width * 0.5, *web_thick, *flange_thick);
            vec![
                [h, -b],
                [h, b],
                [h - tf, b],
                [h - tf, -b + tw],
                [-h + tf, -b + tw],
                [-h + tf, b],
                [-h, b],
                [-h, -b],
            ]
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            // フランジを上（+y）に置く
            let (h, b, tw, tf) = (height * 0.5, width * 0.5, web_thick * 0.5, *flange_thick);
            vec![
                [h, -b],
                [h, b],
                [h - tf, b],
                [h - tf, tw],
                [-h, tw],
                [-h, -tw],
                [h - tf, -tw],
                [h - tf, -b],
            ]
        }
        SectionShape::SteelPipe { outer_dia, .. } | SectionShape::CftPipe { outer_dia, .. } => {
            circle_outline(*outer_dia)
        }
        SectionShape::SteelFlatBar { width, thick } => rect_outline(*thick, *width),
        SectionShape::SteelRoundBar { dia } => circle_outline(*dia),
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            let h = height * 0.5;
            let (ub, ut) = (upper_width * 0.5, *upper_thick);
            let (lb, lt) = (lower_width * 0.5, *lower_thick);
            let tw = web_thick * 0.5;
            vec![
                [h, -ub],
                [h, ub],
                [h - ut, ub],
                [h - ut, tw],
                [-h + lt, tw],
                [-h + lt, lb],
                [-h, lb],
                [-h, -lb],
                [-h + lt, -lb],
                [-h + lt, -tw],
                [h - ut, -tw],
                [h - ut, -ub],
            ]
        }
        SectionShape::SteelLipChannel {
            height,
            width,
            lip,
            thick,
        } => {
            // ウェブを -z 側に置き、+z 側フランジ先端のリップは内向き（y 中心向き）
            let (h, b, c, t) = (height * 0.5, width * 0.5, *lip, *thick);
            vec![
                [h, -b],
                [h, b],
                [h - c, b],
                [h - c, b - t],
                [h - t, b - t],
                [h - t, -b + t],
                [-h + t, -b + t],
                [-h + t, b - t],
                [-h + c, b - t],
                [-h + c, b],
                [-h, b],
                [-h, -b],
            ]
        }
        SectionShape::RcRect { b, d, .. } | SectionShape::SrcRect { b, d, .. } => {
            rect_outline(*d, *b)
        }
        SectionShape::RcCircle { d, .. } => circle_outline(*d),
        SectionShape::CftBox { height, width, .. } => rect_outline(*height, *width),
        // 壁は要素側で面ポリゴン表示するため対象外
        SectionShape::RcWall { .. } => return None,
    };
    Some(outline)
}

/// 部材の断面押し出しソリッドを描画する。
///
/// `coords` は表示用の節点座標（変形図では変位を加味済み）で、
/// `model.nodes` と同順であること。断面を描けなかった線材
/// （断面未割当・形状情報なし）の本数を返す。
#[allow(clippy::too_many_arguments)]
pub(super) fn draw_section_solids(
    painter: &egui::Painter,
    model: &Model,
    coords: &[[f64; 3]],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) -> usize {
    // (奥行き, 描画要素)。奥行きはカメラ空間 z（手前が正）の平均。
    let mut prims: Vec<(f32, SolidPrim)> = Vec::new();
    let mut skipped = 0usize;

    for elem in &model.elements {
        let is_line_member = matches!(
            elem.kind,
            ElementKind::Beam
                | ElementKind::Fiber
                | ElementKind::MultiSpring
                | ElementKind::Brace { .. }
        );
        if !is_line_member || elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= coords.len() || n1 >= coords.len() {
            continue;
        }
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let Some(outline) = sec.and_then(section_outline) else {
            skipped += 1;
            continue;
        };
        let p_i = coords[n0];
        let p_j = coords[n1];
        let dx = p_j[0] - p_i[0];
        let dy = p_j[1] - p_i[1];
        let dz = p_j[2] - p_i[2];
        if dx * dx + dy * dy + dz * dz < 1e-6 {
            continue;
        }
        // 解析と同じ局所座標系で断面を配向（ey=せい方向, ez=幅方向）
        let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
        let ey = frame.rot[1];
        let ez = frame.rot[2];
        let ring = |p: [f64; 3]| -> Vec<[f32; 3]> {
            outline
                .iter()
                .map(|&[y, z]| {
                    let w = [
                        p[0] + ey[0] * y + ez[0] * z,
                        p[1] + ey[1] * y + ez[1] * z,
                        p[2] + ey[2] * y + ez[2] * z,
                    ];
                    cam_space(w, center3, cam)
                })
                .collect()
        };
        let ring_i = ring(p_i);
        let ring_j = ring(p_j);
        let base = base_color(sec.and_then(|s| s.shape.as_ref()));

        // 側面フェイス（輪郭の各辺 × 材軸方向の四角形）
        let n = outline.len();
        for k in 0..n {
            let k1 = (k + 1) % n;
            let quad = [ring_i[k], ring_i[k1], ring_j[k1], ring_j[k]];
            let depth = (quad[0][2] + quad[1][2] + quad[2][2] + quad[3][2]) * 0.25;
            let shade = face_shade(&quad);
            let fill = shaded(base, shade);
            prims.push((
                depth,
                SolidPrim::Quad {
                    pts: [
                        to_screen(quad[0], cam, scale, screen_center),
                        to_screen(quad[1], cam, scale, screen_center),
                        to_screen(quad[2], cam, scale, screen_center),
                        to_screen(quad[3], cam, scale, screen_center),
                    ],
                    fill,
                    stroke: shaded(base, shade * 0.6),
                },
            ));
        }
        // 端面は凹型断面（H・溝形等）があるため輪郭線のみ描く
        for ring in [&ring_i, &ring_j] {
            let depth = ring.iter().map(|r| r[2]).sum::<f32>() / n as f32;
            prims.push((
                depth,
                SolidPrim::Outline {
                    pts: ring
                        .iter()
                        .map(|&r| to_screen(r, cam, scale, screen_center))
                        .collect(),
                    color: shaded(base, 0.5),
                },
            ));
        }
    }

    // 奥（カメラ空間 z 小）→ 手前の順に描画
    prims.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    for (_, prim) in prims {
        match prim {
            SolidPrim::Quad { pts, fill, stroke } => {
                painter.add(egui::Shape::convex_polygon(
                    pts.to_vec(),
                    fill,
                    egui::Stroke::new(0.5_f32, stroke),
                ));
            }
            SolidPrim::Outline { pts, color } => {
                painter.add(egui::Shape::closed_line(
                    pts,
                    egui::Stroke::new(1.0_f32, color),
                ));
            }
        }
    }
    skipped
}
