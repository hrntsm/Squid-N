//! ViewCube（ナビゲーションキューブ）。
//!
//! ビューポート右上に表示する立方体。面クリックで標準ビュー（Top/Front/…)へ、
//! コーナークリックでアイソメビューへ即時スナップする（`CameraState::snap_to_direction`）。
//! 投影・当たり判定は egui の描画コンテキストに依存しない純粋計算で、ヘッドレスで
//! テスト可能。描画（`draw`）のみ `egui::Painter` を使う。

use super::{q_rotate, CameraState};
use crate::theme;

/// 立方体の 6 面（外向き法線, ラベル）。
/// Front はワールド -Y 側（Y=奥行き・Z=鉛直の慣例。ラベルは TONMANUAL に従い英語）。
pub(crate) const FACES: [([f32; 3], &str); 6] = [
    ([0.0, 0.0, 1.0], "Top"),
    ([0.0, 0.0, -1.0], "Bottom"),
    ([0.0, -1.0, 0.0], "Front"),
    ([0.0, 1.0, 0.0], "Back"),
    ([1.0, 0.0, 0.0], "Right"),
    ([-1.0, 0.0, 0.0], "Left"),
];

/// 立方体の 8 頂点（±1）。コーナークリックのアイソメビュー方向を兼ねる。
pub(crate) const CORNERS: [[f32; 3]; 8] = [
    [1.0, 1.0, 1.0],
    [1.0, 1.0, -1.0],
    [1.0, -1.0, 1.0],
    [1.0, -1.0, -1.0],
    [-1.0, 1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [-1.0, -1.0, -1.0],
];

/// 面が可視と判定する法線のビュー Z（手前）成分の下限。
/// ほぼ真横を向いた面はクリック面積が細く誤操作のもとになるため除外する。
const FACE_VISIBLE_EPS: f32 = 0.05;
/// コーナーの当たり判定半径（px）。面より優先されるため、押しやすさを優先して広めに取る
const CORNER_HIT_PX: f32 = 10.0;

/// 画面上の配置（中心座標と半辺長のピクセルスケール）。
pub(crate) struct Layout {
    pub center: egui::Pos2,
    pub scale: f32,
}

/// 当たり判定の結果。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Hit {
    Face(usize),
    Corner(usize),
}

/// ヒット箇所のスナップ先ビュー方向（ワールド座標）。
pub(crate) fn hit_direction(hit: Hit) -> [f32; 3] {
    match hit {
        Hit::Face(i) => FACES[i].0,
        Hit::Corner(i) => CORNERS[i],
    }
}

/// キューブ座標 `v` を画面へ投影する。戻り値は（画面位置, ビュー Z=手前成分）。
fn project_cube(cam: &CameraState, v: [f32; 3], layout: &Layout) -> (egui::Pos2, f32) {
    let r = q_rotate(cam.rot, v);
    (
        egui::pos2(
            layout.center.x + r[0] * layout.scale,
            layout.center.y - r[1] * layout.scale,
        ),
        r[2],
    )
}

/// 面の可視判定（外向き法線が手前を向いているか）。
fn face_visible(cam: &CameraState, normal: [f32; 3]) -> bool {
    q_rotate(cam.rot, normal)[2] > FACE_VISIBLE_EPS
}

/// コーナーの可視判定（可視面のいずれかの頂点であるか）。
fn corner_visible(cam: &CameraState, corner: [f32; 3]) -> bool {
    (0..3).any(|k| {
        let mut n = [0.0; 3];
        n[k] = corner[k].signum();
        face_visible(cam, n)
    })
}

/// 面の 4 頂点（キューブ座標）。法線軸以外の 2 軸を ±1 で回る。
fn face_vertices(normal: [f32; 3]) -> [[f32; 3]; 4] {
    let axis = (0..3)
        .find(|&k| normal[k] != 0.0)
        .expect("法線は軸方向のはず");
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let mut quad = [[0.0_f32; 3]; 4];
    for (i, (su, sv)) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
        .iter()
        .enumerate()
    {
        quad[i][axis] = normal[axis];
        quad[i][u] = *su;
        quad[i][v] = *sv;
    }
    quad
}

/// 凸四角形の内外判定（各辺に対する外積の符号が一致するか。回り順に依存しない）。
fn point_in_convex_quad(p: egui::Pos2, quad: &[egui::Pos2; 4]) -> bool {
    let mut sign = 0.0_f32;
    for i in 0..4 {
        let a = quad[i];
        let b = quad[(i + 1) % 4];
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if cross.abs() < 1e-6 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    true
}

/// 画面位置 `pos` の当たり判定。コーナーはターゲットが小さいため面より優先する。
pub(crate) fn hit_test(cam: &CameraState, layout: &Layout, pos: egui::Pos2) -> Option<Hit> {
    for (i, c) in CORNERS.iter().enumerate() {
        if corner_visible(cam, *c) {
            let (p, _) = project_cube(cam, *c, layout);
            if (pos - p).length() <= CORNER_HIT_PX {
                return Some(Hit::Corner(i));
            }
        }
    }
    for (i, (n, _)) in FACES.iter().enumerate() {
        if face_visible(cam, *n) {
            let quad = face_vertices(*n).map(|v| project_cube(cam, v, layout).0);
            if point_in_convex_quad(pos, &quad) {
                return Some(Hit::Face(i));
            }
        }
    }
    None
}

/// ViewCube を描画する。`hover` はハイライト対象（`hit_test` の結果をそのまま渡す）。
pub(crate) fn draw(
    painter: &egui::Painter,
    cam: &CameraState,
    layout: &Layout,
    hover: Option<Hit>,
) {
    // 可視面を奥から描く（正射影の凸立体なので重ならないが、順序を保証しておく）
    let mut faces: Vec<(usize, f32)> = FACES
        .iter()
        .enumerate()
        .filter(|(_, (n, _))| face_visible(cam, *n))
        .map(|(i, (n, _))| (i, q_rotate(cam.rot, *n)[2]))
        .collect();
    faces.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    for (i, nz) in faces {
        let (n, label) = FACES[i];
        let quad = face_vertices(n).map(|v| project_cube(cam, v, layout).0);
        // ホバー色は TONMANUAL §6 のボタンホバー慣例（blue-300）に合わせる
        let fill = if hover == Some(Hit::Face(i)) {
            theme::BLUE_300
        } else {
            theme::translucent(theme::GRAY_100, 230)
        };
        painter.add(egui::Shape::convex_polygon(
            quad.to_vec(),
            fill,
            egui::Stroke::new(1.0_f32, theme::GRAY_600),
        ));
        // ラベルは面が正面に近いほど読めるため、浅い角度では省く
        if nz > 0.35 {
            let (center, _) = project_cube(cam, n, layout);
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(10.0),
                theme::GRAY_700,
            );
        }
    }

    // ホバー中のコーナーを強調（当たり判定 CORNER_HIT_PX より小さい円で描き、
    // 立方体の見た目を隠しすぎないようにする）
    if let Some(Hit::Corner(i)) = hover {
        let (p, _) = project_cube(cam, CORNERS[i], layout);
        painter.circle_filled(p, 5.0, theme::BLUE_500);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layout() -> Layout {
        Layout {
            center: egui::pos2(100.0, 100.0),
            scale: 30.0,
        }
    }

    #[test]
    fn 面クリックのスナップで視線がその面の正面を向く() {
        for (dir, label) in FACES {
            let mut cam = CameraState::default();
            cam.snap_to_direction(dir);
            let v = q_rotate(cam.rot, dir);
            assert!(
                (v[0].abs() < 1e-5) && (v[1].abs() < 1e-5) && (v[2] - 1.0).abs() < 1e-5,
                "{label}: 視線正面を向いていない: {v:?}"
            );
        }
    }

    #[test]
    fn コーナークリックのスナップで視線がその頂点方向を向く() {
        for corner in CORNERS {
            let mut cam = CameraState::default();
            cam.snap_to_direction(corner);
            let n = 3.0_f32.sqrt();
            let d = [corner[0] / n, corner[1] / n, corner[2] / n];
            let v = q_rotate(cam.rot, d);
            assert!(
                (v[0].abs() < 1e-5) && (v[1].abs() < 1e-5) && (v[2] - 1.0).abs() < 1e-5,
                "{corner:?}: 視線正面を向いていない: {v:?}"
            );
            // 俯仰はターンテーブルの可動域内（真上〜真下）に収まる
            assert!((-std::f32::consts::PI..=0.0).contains(&cam.pitch));
        }
    }

    #[test]
    fn 真上真下へのスナップは旋回角を保持する() {
        let mut cam = CameraState::default();
        let yaw0 = cam.yaw;
        cam.snap_to_direction([0.0, 0.0, 1.0]); // Top
        assert_eq!(cam.yaw, yaw0);
        assert!(cam.pitch.abs() < 1e-6);
        cam.snap_to_direction([0.0, 0.0, -1.0]); // Bottom
        assert_eq!(cam.yaw, yaw0);
        assert!((cam.pitch + std::f32::consts::PI).abs() < 1e-6);
    }

    #[test]
    fn 可視面の中心をクリックするとその面にヒットする() {
        let cam = CameraState::default();
        let layout = layout();
        for (i, (n, label)) in FACES.iter().enumerate() {
            if !face_visible(&cam, *n) {
                continue;
            }
            let (center, _) = project_cube(&cam, *n, &layout);
            assert_eq!(
                hit_test(&cam, &layout, center),
                Some(Hit::Face(i)),
                "{label} の面中心でヒットしない"
            );
        }
    }

    #[test]
    fn 裏側の面にはヒットしない() {
        let cam = CameraState::default(); // yaw=45°, pitch=-45°: Bottom/Back/Left が裏
        let layout = layout();
        for (i, (n, label)) in FACES.iter().enumerate() {
            if face_visible(&cam, *n) {
                continue;
            }
            // 画面全域を走査しても裏面がヒットすることはない
            for x in (40..160).step_by(4) {
                for y in (40..160).step_by(4) {
                    let hit = hit_test(&cam, &layout, egui::pos2(x as f32, y as f32));
                    assert_ne!(hit, Some(Hit::Face(i)), "裏面 {label} がヒットした");
                }
            }
        }
    }

    #[test]
    fn 可視コーナーの近傍クリックはコーナーが面より優先される() {
        let cam = CameraState::default();
        let layout = layout();
        // 既定ビューで手前に見える頂点 [1,-1,1]（Top/Front/Right の共有頂点）
        let idx = CORNERS.iter().position(|c| *c == [1.0, -1.0, 1.0]).unwrap();
        let (p, _) = project_cube(&cam, CORNERS[idx], &layout);
        assert_eq!(
            hit_test(&cam, &layout, p),
            Some(Hit::Corner(idx)),
            "コーナーにヒットしない"
        );
    }

    #[test]
    fn コーナーの当たり判定半径内のクリックはコーナーが優先される() {
        let cam = CameraState::default();
        let layout = layout();
        let idx = CORNERS.iter().position(|c| *c == [1.0, -1.0, 1.0]).unwrap();
        let (p, _) = project_cube(&cam, CORNERS[idx], &layout);
        // 半径いっぱいまでずらしてもコーナーにヒットする（押しやすさの担保）
        let off = egui::pos2(p.x + CORNER_HIT_PX - 0.5, p.y);
        assert_eq!(
            hit_test(&cam, &layout, off),
            Some(Hit::Corner(idx)),
            "当たり判定半径の縁でコーナーにヒットしない"
        );
    }
}
