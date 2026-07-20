use crate::app::App;
use crate::theme;

mod viewcube;
use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::ids::SectionId;

mod solid;

/// 3D ビュー上での支持条件の分類。`Dof6Mask` のビットパターンを意味的にまとめる。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SupportKind {
    /// 拘束なし（自由節点）
    Free,
    /// ピン支持（並進 3 自由度を拘束、回転は自由）
    Pinned,
    /// 固定支持（全 6 自由度を拘束）
    Fixed,
    /// ローラー支持（並進の一部のみ拘束、回転は自由）
    Roller,
    /// その他の部分拘束（上記以外の組み合わせ）
    Custom,
}

/// `Dof6Mask` を `SupportKind` へ分類する。
fn support_kind(restraint: Dof6Mask) -> SupportKind {
    const FIXED_BITS: u8 = Dof6Mask::FIXED.0;
    const PINNED_BITS: u8 = Dof6Mask::PINNED.0;
    match restraint.0 {
        0 => SupportKind::Free,
        FIXED_BITS => SupportKind::Fixed,
        PINNED_BITS => SupportKind::Pinned,
        _ => {
            let translational = restraint.0 & 0b000111; // Ux, Uy, Uz
            let rotational = restraint.0 & 0b111000; // Rx, Ry, Rz
            if translational != 0 && rotational == 0 {
                SupportKind::Roller
            } else {
                SupportKind::Custom
            }
        }
    }
}

/// ビューアの表示モード。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum ViewMode {
    /// 形状のみ
    #[default]
    Shape,
    /// 変形図（線形静的結果）
    Deformed,
    /// モード形（固有値結果）
    Mode,
    /// N 図
    N,
    /// Q 図
    Q,
    /// M 図
    M,
    /// CMQ 図（両端固定端モーメント C とせん断 Q）
    Cmq,
}

// ===== クォータニオン（3Dカメラ回転用, [w, x, y, z]）=====
// mn_view（M-N相関曲面ビュー）でも同じ操作感の3Dカメラを実装するため、
// これらのヘルパは pub(crate) として公開し再利用する。
pub(crate) type Quat = [f32; 4];

/// 軸 `axis`（正規化済み想定）まわり `ang` ラジアンの回転クォータニオン。
pub(crate) fn q_axis_angle(axis: [f32; 3], ang: f32) -> Quat {
    let h = ang * 0.5;
    let s = h.sin();
    [h.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// クォータニオン積 a⊗b。
pub(crate) fn q_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// 正規化（数値誤差の累積を抑える）。
pub(crate) fn q_norm(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-9 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

/// ベクトル v をクォータニオン q で回転する。
pub(crate) fn q_rotate(q: Quat, v: [f32; 3]) -> [f32; 3] {
    let qv = [q[1], q[2], q[3]];
    let t = [
        2.0 * (qv[1] * v[2] - qv[2] * v[1]),
        2.0 * (qv[2] * v[0] - qv[0] * v[2]),
        2.0 * (qv[0] * v[1] - qv[1] * v[0]),
    ];
    [
        v[0] + q[0] * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
        v[1] + q[0] * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
        v[2] + q[0] * t[2] + (qv[0] * t[1] - qv[1] * t[0]),
    ]
}

/// 3D→2D 投影（§3-2: ターンテーブル回転 + 正射影）。
///
/// 構造モデルは実寸比が意味を持つため、§3-2 の「各軸を [-1,1] に正規化」は採らず、
/// 全軸一様スケールで投影してプロポーションを保持する。
/// ビュー軸は X=右・Y=上・Z=手前。
///
/// 回転はターンテーブル方式: 水平ドラッグ＝ワールド Z 軸（鉛直軸）まわりの旋回、
/// 垂直ドラッグ＝画面 X 軸まわりの俯仰。ロールが発生しないため、建物の鉛直軸は
/// 常に画面上で縦に保たれる（自由回転のアークボールはロールが蓄積し視点が傾く）。
#[derive(Clone)]
pub struct CameraState {
    /// 回転（クォータニオン）。`yaw`/`pitch` から導出したキャッシュ
    pub(crate) rot: Quat,
    /// ワールド Z 軸まわりの旋回角 [rad]
    pub(crate) yaw: f32,
    /// 画面 X 軸まわりの俯仰角 [rad]。0=真上（平面図）〜 -π/2=正面 〜 -π=真下
    pub(crate) pitch: f32,
    /// 画面パン（px）
    pub(crate) pan: [f32; 2],
    /// ズーム倍率（§3-2: 既定 3.0、範囲 0.5–10.0）
    pub(crate) zoom: f32,
}

impl Default for CameraState {
    fn default() -> Self {
        // 45° の斜めビュー（平面を 45° 振ってから 45° 見下ろす）。
        // XY 平面のグリッドが斜めから見えるようにする。
        let yaw = std::f32::consts::FRAC_PI_4;
        let pitch = -std::f32::consts::FRAC_PI_4;
        Self {
            rot: Self::rot_from(yaw, pitch),
            yaw,
            pitch,
            pan: [0.0, 0.0],
            zoom: 3.0,
        }
    }
}

impl CameraState {
    /// ドラッグ回転の感度 [rad/px]
    const ROT_SENS: f32 = 0.005;

    /// `yaw`/`pitch` からビュー回転を導出する（旋回→俯仰の順で合成）。
    fn rot_from(yaw: f32, pitch: f32) -> Quat {
        q_norm(q_mul(
            q_axis_angle([1.0, 0.0, 0.0], pitch),
            q_axis_angle([0.0, 0.0, 1.0], yaw),
        ))
    }

    /// ドラッグ量（px）によるターンテーブル回転。
    /// 俯仰は真上（0）〜真下（-π）でクランプし、天地の反転を防ぐ。
    pub(crate) fn turntable_drag(&mut self, dx_px: f32, dy_px: f32) {
        self.yaw += dx_px * Self::ROT_SENS;
        self.pitch = (self.pitch + dy_px * Self::ROT_SENS).clamp(-std::f32::consts::PI, 0.0);
        self.rot = Self::rot_from(self.yaw, self.pitch);
    }

    /// 視点方向 `d`（ワールド座標、原点から視点位置へ向かうベクトル）へ即時スナップする。
    /// ViewCube の面・コーナークリックから呼ばれる。`d` が鉛直（真上/真下）の場合、
    /// 旋回角は方位角から定まらないため 0 とし、X 軸が画面右を向く正対の平面ビューにする。
    pub(crate) fn snap_to_direction(&mut self, d: [f32; 3]) {
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if n < 1e-6 {
            return;
        }
        let (dx, dy, dz) = (d[0] / n, d[1] / n, d[2] / n);
        // ターンテーブル rot = R_x(pitch)∘R_z(yaw) で q_rotate(rot, d) = [0,0,1]（視線正面）
        // となる角度: yaw は方位角 φ=atan2(dy,dx) から、pitch は仰角から定まる。
        self.yaw = if dx.abs() > 1e-6 || dy.abs() > 1e-6 {
            -std::f32::consts::FRAC_PI_2 - dy.atan2(dx)
        } else {
            0.0
        };
        self.pitch = dz.clamp(-1.0, 1.0).asin() - std::f32::consts::FRAC_PI_2;
        self.rot = Self::rot_from(self.yaw, self.pitch);
    }
}

/// ワールド座標 `p` を投影する。`center3` はモデル中心（回転中心）、`scale` は px/世界長、
/// `screen_center` は描画領域中心（px）。
pub(crate) fn project(
    p: [f64; 3],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) -> [f32; 2] {
    let v = [
        (p[0] - center3[0]) as f32,
        (p[1] - center3[1]) as f32,
        (p[2] - center3[2]) as f32,
    ];
    let r = q_rotate(cam.rot, v);
    [
        screen_center[0] + cam.pan[0] + r[0] * scale,
        screen_center[1] + cam.pan[1] - r[1] * scale,
    ]
}

/// 3D ベクトルの外積。
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// スクリーン座標上の矢印（線分＋矢頭）を描く。
fn draw_arrow(painter: &egui::Painter, from: egui::Pos2, to: egui::Pos2, color: egui::Color32) {
    let stroke = egui::Stroke::new(2.0_f32, color);
    painter.line_segment([from, to], stroke);
    let dir = to - from;
    let len = dir.length();
    if len < 1e-3 {
        return;
    }
    let ux = dir.x / len;
    let uy = dir.y / len;
    let nx = -uy;
    let ny = ux;
    const HEAD: f32 = 6.0;
    let base = egui::pos2(to.x - ux * HEAD, to.y - uy * HEAD);
    let left = egui::pos2(base.x + nx * HEAD * 0.5, base.y + ny * HEAD * 0.5);
    let right = egui::pos2(base.x - nx * HEAD * 0.5, base.y - ny * HEAD * 0.5);
    painter.line_segment([to, left], stroke);
    painter.line_segment([to, right], stroke);
}

/// 節点を中心に `axis` まわりの回転を示す円弧（全周）を描く。
#[allow(clippy::too_many_arguments)]
fn draw_rotation_arc(
    painter: &egui::Painter,
    center_world: [f64; 3],
    axis: [f64; 3],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
    radius_world: f64,
    color: egui::Color32,
) {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        return;
    }
    let axis = [axis[0] / n, axis[1] / n, axis[2] / n];
    // 軸に直交する面内の直交基底 u, v を作る
    let ref_vec = if axis[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u_raw = cross3(axis, ref_vec);
    let un = (u_raw[0] * u_raw[0] + u_raw[1] * u_raw[1] + u_raw[2] * u_raw[2]).sqrt();
    if un < 1e-12 {
        return;
    }
    let u = [u_raw[0] / un, u_raw[1] / un, u_raw[2] / un];
    let v = cross3(axis, u);

    let stroke = egui::Stroke::new(1.5_f32, color);
    const N: usize = 32;
    let mut prev: Option<egui::Pos2> = None;
    for i in 0..=N {
        let theta = i as f64 / N as f64 * std::f64::consts::TAU;
        let c = theta.cos();
        let s = theta.sin();
        let pt = [
            center_world[0] + radius_world * (c * u[0] + s * v[0]),
            center_world[1] + radius_world * (c * u[1] + s * v[1]),
            center_world[2] + radius_world * (c * u[2] + s * v[2]),
        ];
        let p = project(pt, center3, cam, scale, screen_center);
        let cur = egui::pos2(p[0], p[1]);
        if let Some(p0) = prev {
            painter.line_segment([p0, cur], stroke);
        }
        prev = Some(cur);
    }
}

/// 支持条件シンボルを 3D ビューに描画する。
///
/// 固定されている並進自由度の方向へ軸色の矢印を引き、
/// 固定されている回転自由度の軸まわりに円弧を描く。
/// 軸色は X=赤 / Y=緑 / Z=青（§3-2 規約）で方向を直感的に判別できる。
///
/// 現在は全体座標系（X/Y/Z）の軸方向に描画する。将来的に節点ごとに局所座標系を
/// 導入した際は、この関数が参照する軸ベクトルを局所座標系の軸へ差し替えればよい。
#[allow(clippy::too_many_arguments)]
fn draw_support_symbol(
    painter: &egui::Painter,
    node_coord: [f64; 3],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
    restraint: Dof6Mask,
    arrow_px: f32,
    arc_px: f32,
) {
    if support_kind(restraint) == SupportKind::Free {
        return;
    }
    // スクリーン上で arrow_px / arc_px になるようワールド長を逆算
    let arrow_world = arrow_px as f64 / scale as f64;
    let arc_world = arc_px as f64 / scale as f64;
    let p0 = project(node_coord, center3, cam, scale, screen_center);
    let origin = egui::pos2(p0[0], p0[1]);

    // 並進自由度: 固定方向へ軸色の矢印
    let translational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Ux, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Uy, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Uz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (dof, dir, color) in translational {
        if restraint.is_fixed(dof) {
            let end = [
                node_coord[0] + dir[0] * arrow_world,
                node_coord[1] + dir[1] * arrow_world,
                node_coord[2] + dir[2] * arrow_world,
            ];
            let pe = project(end, center3, cam, scale, screen_center);
            draw_arrow(painter, origin, egui::pos2(pe[0], pe[1]), color);
        }
    }

    // 回転自由度: 軸まわりの円弧
    let rotational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Rx, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Ry, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Rz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (dof, axis, color) in rotational {
        if restraint.is_fixed(dof) {
            draw_rotation_arc(
                painter,
                node_coord,
                axis,
                center3,
                cam,
                scale,
                screen_center,
                arc_world,
                color,
            );
        }
    }
}

/// 支持条件シンボルの凡例をビュー左下に描く。
fn draw_support_legend(painter: &egui::Painter) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let y0 = rect.max.y - 10.0;

    // タイトル
    painter.text(
        egui::pos2(x0, y0 - 30.0),
        egui::Align2::LEFT_BOTTOM,
        "支持条件",
        egui::FontId::proportional(13.0),
        theme::GRAY_700,
    );
    // 並進固定サンプル: 矢印
    let arrow_y = y0 - 16.0;
    draw_arrow(
        painter,
        egui::pos2(x0, arrow_y),
        egui::pos2(x0 + 20.0, arrow_y),
        theme::AXIS_X,
    );
    painter.text(
        egui::pos2(x0 + 28.0, y0 - 12.0),
        egui::Align2::LEFT_BOTTOM,
        "並進固定 (X赤/Y緑/Z青)",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
    // 回転固定サンプル: 円
    let arc_y = y0;
    painter.circle_stroke(
        egui::pos2(x0 + 10.0, arc_y - 6.0),
        7.0,
        egui::Stroke::new(1.5_f32, theme::AXIS_X),
    );
    painter.text(
        egui::pos2(x0 + 28.0, y0),
        egui::Align2::LEFT_BOTTOM,
        "回転固定 (X赤/Y緑/Z青)",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
}

pub fn viewer_panel(ui: &mut egui::Ui, app: &mut App) {
    let mut mode = app.view_mode;
    let mut deform_scale = app.deform_scale;
    let mut mode_idx = app.view_mode_idx;

    // --- コントロール ---
    // 中央パネルが狭い場合（左パネルを広げた時など）にボタン列が右パネルへ
    // はみ出さないよう、折り返し可能なレイアウトにする。
    ui.horizontal_wrapped(|ui| {
        ui.label("表示:");
        ui.selectable_value(&mut mode, ViewMode::Shape, "形状");
        ui.selectable_value(&mut mode, ViewMode::Deformed, "変形");
        ui.selectable_value(&mut mode, ViewMode::Mode, "モード");
        ui.selectable_value(&mut mode, ViewMode::N, "N図");
        ui.selectable_value(&mut mode, ViewMode::Q, "Q図");
        ui.selectable_value(&mut mode, ViewMode::M, "M図");
        ui.selectable_value(&mut mode, ViewMode::Cmq, "CMQ図");
        ui.separator();
        // 断面表示: 部材を断面形状の押し出しソリッドで立体表示（全モードと併用可）
        ui.toggle_value(&mut app.show_sections, "断面表示");
        ui.separator();
        // §3-2 の操作規約をヒント表示（左ドラッグ=回転／スクロール=ズーム）
        ui.add_enabled(
            false,
            egui::Label::new(
                egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム")
                    .size(11.0),
            ),
        );
    });
    if matches!(mode, ViewMode::Deformed | ViewMode::Mode) {
        ui.horizontal(|ui| {
            ui.label("倍率:");
            ui.add(egui::Slider::new(&mut deform_scale, 1.0..=10000.0).logarithmic(true));
        });
    }
    if mode == ViewMode::Mode {
        let n_modes = app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .map(|m| m.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .results
                    .as_ref()
                    .and_then(|r| r.modal.as_ref())
                    .and_then(|m| m.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }

    app.view_mode = mode;
    app.deform_scale = deform_scale;
    app.view_mode_idx = mode_idx;

    // CMQ 図はモデル編集に常に追従させるため、表示中は毎フレーム再計算する
    // （スラブ数は小さい前提）。
    if app.view_mode == ViewMode::Cmq {
        app.refresh_beam_loads();
    }

    // --- 梁作成モード ---
    // ON 中はクリックで節点を選び、2 点目で梁を生成する（OFF 中は部材クリック=断面割当）。
    ui.horizontal(|ui| {
        let beam_was_on = app.beam_draw_mode;
        ui.toggle_value(&mut app.beam_draw_mode, "梁作成モード");
        // 梁作成を ON にしたら壁・スラブ作成は OFF（排他）
        if app.beam_draw_mode && !beam_was_on {
            app.wall_draw_mode = false;
            app.slab_draw_mode = false;
        }
        if app.beam_draw_mode {
            match app.beam_draw_first {
                None => {
                    ui.label("始点の節点をクリック");
                }
                Some(nid) => {
                    ui.label(format!("始点 N{} 選択中 → 終点の節点をクリック", nid.0));
                    if ui.button("キャンセル").clicked() {
                        app.beam_draw_first = None;
                    }
                }
            }
        }
    });
    // モード OFF 時は始点選択をクリア
    if !app.beam_draw_mode {
        app.beam_draw_first = None;
    }

    // --- 壁作成モード ---
    // ON 中はクリックで柱・梁に囲まれた 4 節点を順に選び、4 点目で壁を生成する。
    ui.horizontal(|ui| {
        let wall_was_on = app.wall_draw_mode;
        ui.toggle_value(&mut app.wall_draw_mode, "壁作成モード");
        // 壁作成を ON にしたら梁・スラブ作成は OFF（排他）
        if app.wall_draw_mode && !wall_was_on {
            app.beam_draw_mode = false;
            app.slab_draw_mode = false;
        }
        if app.wall_draw_mode {
            let picked: Vec<String> = app
                .wall_draw_nodes
                .iter()
                .map(|n| format!("N{}", n.0))
                .collect();
            ui.label(format!(
                "節点を4つクリック ({}/4){}",
                app.wall_draw_nodes.len(),
                if picked.is_empty() {
                    String::new()
                } else {
                    format!(": {}", picked.join(", "))
                }
            ));
            if !app.wall_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                app.wall_draw_nodes.clear();
            }
        }
    });
    // モード OFF 時は選択をクリア
    if !app.wall_draw_mode {
        app.wall_draw_nodes.clear();
    }

    // --- スラブ作成モード ---
    // ON 中はクリックで境界節点を外周順に選び、3〜N 節点そろったら「確定」で生成する。
    ui.horizontal(|ui| {
        let slab_was_on = app.slab_draw_mode;
        ui.toggle_value(&mut app.slab_draw_mode, "スラブ作成モード");
        // スラブ作成を ON にしたら梁・壁作成は OFF（排他）
        if app.slab_draw_mode && !slab_was_on {
            app.beam_draw_mode = false;
            app.wall_draw_mode = false;
        }
        if app.slab_draw_mode {
            // 節点削除などで陳腐化した参照（範囲外 id）を毎フレーム除去し、
            // 存在しない節点を境界に含むスラブの生成を防ぐ。
            let node_count = app.model.nodes.len() as u32;
            app.slab_draw_nodes.retain(|n| n.0 < node_count);
            let picked: Vec<String> = app
                .slab_draw_nodes
                .iter()
                .map(|n| format!("N{}", n.0))
                .collect();
            ui.label(format!(
                "境界節点を外周順にクリック ({}){}",
                app.slab_draw_nodes.len(),
                if picked.is_empty() {
                    String::new()
                } else {
                    format!(": {}", picked.join(", "))
                }
            ));
            if app.slab_draw_nodes.len() >= 3 && ui.button("確定").clicked() {
                let boundary = app.slab_draw_nodes.clone();
                app.undo.run(
                    &mut app.model,
                    Box::new(squid_n_edit::AddSlab {
                        boundary,
                        joists: Vec::new(),
                        loads: Vec::new(),
                        method: squid_n_core::model::DistributionMethod::TriTrapezoid,
                        usage: None,
                    }),
                );
                app.staleness.mark_edited();
                app.slab_draw_nodes.clear();
            }
            if !app.slab_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                app.slab_draw_nodes.clear();
            }
        }
    });
    // モード OFF 時は選択をクリア
    if !app.slab_draw_mode {
        app.slab_draw_nodes.clear();
    }

    // --- 断面割当 UI ---
    // focus_member を先にコピーして、後段の可変借用と競合しないようにする
    let focus_id: Option<squid_n_core::ids::ElemId> = app.nav.focus_member;
    // 存在確認もここで行い、ローカルに有効性と現在断面を取得
    let elem_info: Option<(squid_n_core::ids::ElemId, Option<SectionId>)> =
        focus_id.and_then(|eid| {
            app.model
                .elements
                .iter()
                .find(|e| e.id == eid)
                .map(|e| (e.id, e.section))
        });

    let mut pending_assign: Option<Option<SectionId>> = None;

    if let Some((elem_id, current_section)) = elem_info {
        ui.horizontal(|ui| {
            ui.label(format!("選択中の梁 #{}", elem_id.0));
            ui.label("断面:");
            let selected_text = current_section
                .map(|sid| format!("S{}", sid.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("viewer_assign_section")
                .selected_text(selected_text)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(current_section.is_none(), "―")
                        .clicked()
                    {
                        pending_assign = Some(None);
                    }
                    for sec in &app.model.sections {
                        if ui
                            .selectable_label(
                                current_section == Some(sec.id),
                                format!("S{}", sec.id.0),
                            )
                            .clicked()
                        {
                            pending_assign = Some(Some(sec.id));
                        }
                    }
                });
        });
        // クロージャ外で発行（借用ルール）
        if let Some(section) = pending_assign {
            app.undo.run(
                &mut app.model,
                Box::new(squid_n_edit::SetElementSection {
                    elem: elem_id,
                    section,
                }),
            );
            app.staleness.mark_edited();
        }
    } else {
        ui.label("ビューアで梁をクリックすると選択できます");
    }

    ui.separator();

    // --- 描画領域 ---
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height()),
        egui::Sense::click_and_drag(),
    );

    // カメラ操作（§3-2: 左ドラッグ=回転 / スクロール=ズーム）。
    // パンは規約外の補助操作として右ドラッグに割り当てる。
    let mut cam = app.camera.clone();
    if response.dragged_by(egui::PointerButton::Primary) {
        // ターンテーブル回転（鉛直軸を画面上で縦に保つ。CameraState のドキュメント参照）。
        let d = response.drag_delta();
        cam.turntable_drag(d.x, d.y);
    }
    if response.dragged_by(egui::PointerButton::Secondary) {
        let d = response.drag_delta();
        cam.pan[0] += d.x;
        cam.pan[1] += d.y;
    }
    // スクロールズーム（係数 0.01、0.5–10.0 にクランプ）。トラックパッドのピンチも反映。
    let scroll_y = ui.input(|i| i.smooth_scroll_delta.y);
    if scroll_y != 0.0 {
        cam.zoom *= 1.0 + scroll_y * 0.01;
    }
    let pinch = ui.input(|i| i.zoom_delta());
    if pinch != 1.0 {
        cam.zoom *= pinch;
    }
    cam.zoom = cam.zoom.clamp(0.5, 10.0);

    // ViewCube（右上）: 面クリック=標準ビュー / コーナークリック=アイソメへ即時スナップ。
    // モデルより手前の固定 UI のため、当たり判定を部材ピックより先に行い、
    // キューブ上のクリックはピック処理へ流さない。
    let cube_layout = viewcube::Layout {
        center: egui::pos2(rect.max.x - 55.0, rect.min.y + 55.0),
        scale: 22.0,
    };
    let cube_hover = response
        .hover_pos()
        .and_then(|p| viewcube::hit_test(&cam, &cube_layout, p));
    if cube_hover.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    let mut cube_clicked = false;
    if response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some(hit) = viewcube::hit_test(&cam, &cube_layout, pos) {
                cam.snap_to_direction(viewcube::hit_direction(hit));
                cube_clicked = true;
            }
        }
    }

    let painter = ui.painter_at(rect);
    // §3-2: 3D 背景は白を避け淡いグレー（立体感・奥行きのため）
    painter.rect_filled(rect, 0.0, theme::VIEW_BG);

    let center = [rect.center().x, rect.center().y];

    // 投影スケールとモデル中心（回転中心）。一様スケールで実寸比を保持する。
    // モデルが空でもグリッド・軸を描画するため早期 return はしない。
    let (bmin, bmax) = model_bbox(&app.model);
    let center3 = [
        (bmin[0] + bmax[0]) * 0.5,
        (bmin[1] + bmax[1]) * 0.5,
        (bmin[2] + bmax[2]) * 0.5,
    ];
    let model_size = model_bbox_size(&app.model);
    let min_dim = rect.width().min(rect.height());
    let fit = if model_size > 1e-9 {
        0.8 * min_dim / model_size as f32
    } else {
        1.0
    };
    // 既定ズーム 3.0 でモデル対角が描画領域の約 80% に収まるよう基準化。
    let scale = fit * (cam.zoom / 3.0);

    // グリッド・軸（§3-2: 赤=X / 緑=Y / 青=Z）。モデルの背後に先に描く。
    draw_grid_and_axes(&painter, rect, center3, &cam, scale, center);

    // 節点座標（変形・モード時は変位を加味）
    let disp = match mode {
        ViewMode::Deformed => app.current_static().map(|s| s.disp.clone()),
        ViewMode::Mode => app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.shapes.get(mode_idx))
            .map(|shape| {
                // shape は自由度順の平坦ベクトル → [node][6] に分解
                let n = app.model.nodes.len();
                let mut disp = vec![[0.0; 6]; n];
                for (ni, row) in disp.iter_mut().enumerate() {
                    for (d, slot) in row.iter_mut().enumerate().take(6) {
                        let g = ni * 6 + d;
                        if g < shape.len() {
                            *slot = shape[g];
                        }
                    }
                }
                disp
            }),
        _ => None,
    };

    // 変形スケール: 最大変位をモデルサイズの 10% に正規化
    let deform_scale_actual = if disp.is_some() {
        let max_disp = disp
            .as_ref()
            .map(|d| {
                d.iter()
                    .map(|v| v[0].abs().max(v[1].abs()).max(v[2].abs()))
                    .fold(0.0_f64, f64::max)
            })
            .unwrap_or(0.0);
        if max_disp > 1e-12 {
            (model_size * 0.1 / max_disp) * deform_scale as f64
        } else {
            0.0
        }
    } else {
        0.0
    };

    // 表示用の節点 3D 座標（変形図・モード形では変位を加味）。
    // 断面ソリッド描画でも 3D 座標が要るため、投影前の座標を保持する。
    let coords3: Vec<[f64; 3]> = app
        .model
        .nodes
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let mut p = node.coord;
            if let Some(d) = &disp {
                p[0] += d[i][0] * deform_scale_actual;
                p[1] += d[i][1] * deform_scale_actual;
                p[2] += d[i][2] * deform_scale_actual;
            }
            p
        })
        .collect();
    let pts: Vec<[f32; 2]> = coords3
        .iter()
        .map(|&p| project(p, center3, &cam, scale, center))
        .collect();

    // --- クリック処理（ViewCube 上のクリックはスナップ済みのため除外） ---
    if response.clicked() && !cube_clicked {
        if let Some(click_pos) = response.interact_pointer_pos() {
            if app.beam_draw_mode {
                // 梁作成モード：クリック位置に最も近い節点を選ぶ
                let mut best: Option<(usize, f32)> = None;
                for (i, &p) in pts.iter().enumerate() {
                    let d = (click_pos - egui::pos2(p[0], p[1])).length();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        match app.beam_draw_first {
                            None => {
                                // 1 点目：始点として記憶
                                app.beam_draw_first = Some(node_id);
                            }
                            Some(first) => {
                                // 2 点目：始点と異なれば梁を生成。同一節点は無視。
                                if first != node_id {
                                    let new_id =
                                        squid_n_core::ids::ElemId(app.model.elements.len() as u32);
                                    let elem = squid_n_core::model::ElementData {
                                        id: new_id,
                                        kind: squid_n_core::model::ElementKind::Beam,
                                        nodes: [first, node_id].into_iter().collect(),
                                        section: None,
                                        material: None,
                                        local_axis: squid_n_core::model::LocalAxis {
                                            ref_vector: [0.0, 0.0, 1.0],
                                        },
                                        end_cond: [
                                            squid_n_core::model::EndCondition::Fixed,
                                            squid_n_core::model::EndCondition::Fixed,
                                        ],
                                        force_regime: squid_n_core::model::ForceRegime::Auto,
                                        rigid_zone: Default::default(),
                                        plastic_zone: None,
                                        spring: None,
                                    };
                                    app.undo.run(
                                        &mut app.model,
                                        Box::new(squid_n_edit::AddMember { elem }),
                                    );
                                    app.staleness.mark_edited();
                                    app.nav.focus_member = Some(new_id);
                                }
                                // 次の梁に備えて始点をリセット
                                app.beam_draw_first = None;
                            }
                        }
                    }
                }
            } else if app.wall_draw_mode {
                // 壁作成モード：クリック位置に最も近い節点を選ぶ
                let mut best: Option<(usize, f32)> = None;
                for (i, &p) in pts.iter().enumerate() {
                    let d = (click_pos - egui::pos2(p[0], p[1])).length();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        // 同一節点の重複選択は無視
                        if !app.wall_draw_nodes.contains(&node_id) {
                            app.wall_draw_nodes.push(node_id);
                        }
                        // 4 点そろったら壁を生成
                        if app.wall_draw_nodes.len() == 4 {
                            let ordered = order_wall_nodes(&app.model, &app.wall_draw_nodes);
                            let new_id = squid_n_core::ids::ElemId(app.model.elements.len() as u32);
                            let elem = squid_n_core::model::ElementData {
                                id: new_id,
                                kind: squid_n_core::model::ElementKind::Wall,
                                nodes: ordered.into_iter().collect(),
                                section: None,
                                material: None,
                                local_axis: squid_n_core::model::LocalAxis {
                                    ref_vector: [0.0, 0.0, 1.0],
                                },
                                end_cond: [
                                    squid_n_core::model::EndCondition::Fixed,
                                    squid_n_core::model::EndCondition::Fixed,
                                ],
                                force_regime: squid_n_core::model::ForceRegime::Auto,
                                rigid_zone: Default::default(),
                                plastic_zone: None,
                                spring: None,
                            };
                            app.undo
                                .run(&mut app.model, Box::new(squid_n_edit::AddMember { elem }));
                            app.staleness.mark_edited();
                            app.nav.focus_member = Some(new_id);
                            app.wall_draw_nodes.clear();
                        }
                    }
                }
            } else if app.slab_draw_mode {
                // スラブ作成モード：クリック位置に最も近い節点を外周順に追加する。
                let mut best: Option<(usize, f32)> = None;
                for (i, &p) in pts.iter().enumerate() {
                    let d = (click_pos - egui::pos2(p[0], p[1])).length();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        // 同一節点の重複選択は無視（外周は各節点1回）。
                        if !app.slab_draw_nodes.contains(&node_id) {
                            app.slab_draw_nodes.push(node_id);
                        }
                    }
                }
            } else {
                // 通常モード：クリック位置に最も近い部材線分を選び、閾値内なら選択。
                let mut best: Option<(squid_n_core::ids::ElemId, f32)> = None;
                for elem in &app.model.elements {
                    if elem.nodes.len() < 2 {
                        continue;
                    }
                    let n0 = elem.nodes[0].index();
                    let n1 = elem.nodes[1].index();
                    if n0 >= pts.len() || n1 >= pts.len() {
                        continue;
                    }
                    let a = egui::pos2(pts[n0][0], pts[n0][1]);
                    let b = egui::pos2(pts[n1][0], pts[n1][1]);
                    let d = dist_point_to_segment(click_pos, a, b);
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some((elem.id, d));
                    }
                }
                // ピッキング許容距離（px）
                const PICK_THRESHOLD: f32 = 8.0;
                match best {
                    Some((id, d)) if d <= PICK_THRESHOLD => {
                        app.selection.members = vec![id];
                        app.nav.focus_member = Some(id);
                    }
                    _ => {
                        app.selection.members.clear();
                    }
                }
            }
        }
    }

    // --- スラブ・小梁 ---
    // 荷重分配オブジェクト（解析部材ではない）であることが分かるよう、
    // 構造部材（実線・青/グレー系）と異なる暖色半透明フィル＋破線のフォーマットで描く。
    // 部材線・断面ソリッドより先に描き、架構が床の上に重なるようにする。
    draw_slabs_and_joists(&painter, app, &pts);

    // --- 断面ソリッド ---
    // 節点・部材線より先に描き、線・シンボル類は上に重ねる（材軸が見えるように）。
    let mut solids_skipped = 0usize;
    if app.show_sections {
        solids_skipped = solid::draw_section_solids(
            &painter, &app.model, &coords3, center3, &cam, scale, center,
        );
    }

    // 節点（梁/壁作成モードで選択中の節点は強調表示）
    for (i, &p) in pts.iter().enumerate() {
        let node_id = app.model.nodes[i].id;
        let is_first = app.beam_draw_first == Some(node_id);
        let is_wall_pick = app.wall_draw_nodes.contains(&node_id);
        let is_slab_pick = app.slab_draw_nodes.contains(&node_id);
        let (radius, color) = if is_first || is_wall_pick || is_slab_pick {
            // 作成モードで選択中の節点 = 重要（赤）
            (5.0, theme::PARETO_RED)
        } else {
            // 通常の節点 = データ点（青）
            (3.0, theme::DATA_BLUE)
        };
        painter.circle_filled(egui::pos2(p[0], p[1]), radius, color);
    }

    // 部材（線）
    let line_color = if matches!(mode, ViewMode::Deformed | ViewMode::Mode) {
        // 変形図・モード形 = 結果の強調（ハイライト紫）
        theme::HILITE_PURPLE
    } else {
        // 通常の部材 = 沈めたニュートラル（gray-700）
        theme::GRAY_700
    };
    // 断面表示中は中心線を細く淡くし、ソリッドの上に材軸として薄く重ねる
    let line_stroke = if app.show_sections {
        egui::Stroke::new(1.0_f32, theme::translucent(line_color, 110))
    } else {
        egui::Stroke::new(2.0_f32, line_color)
    };
    for elem in &app.model.elements {
        // 壁（面要素）は半透明ポリゴンで描画
        if elem.kind == squid_n_core::model::ElementKind::Wall && elem.nodes.len() >= 3 {
            let poly: Vec<egui::Pos2> = elem
                .nodes
                .iter()
                .filter_map(|n| {
                    let idx = n.index();
                    (idx < pts.len()).then(|| egui::pos2(pts[idx][0], pts[idx][1]))
                })
                .collect();
            if poly.len() == elem.nodes.len() {
                painter.add(egui::Shape::convex_polygon(
                    poly,
                    theme::translucent(theme::DATA_BLUE, 50),
                    egui::Stroke::new(1.5_f32, theme::DATA_BLUE),
                ));
            }
            continue;
        }
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 < pts.len() && n1 < pts.len() {
            painter.line_segment(
                [
                    egui::pos2(pts[n0][0], pts[n0][1]),
                    egui::pos2(pts[n1][0], pts[n1][1]),
                ],
                line_stroke,
            );
        }
    }

    // 二次部材（小梁・間柱）: 解析対象外だが実在部材なので実線で描く
    // （解析対象外を示す暖色アンバー。スラブの暖色と同族で、主架構の
    // 青/グレーと弁別。断面表示中はソリッドが上に描かれているため
    // 材軸線として薄く重ねる）。
    let secondary_stroke = if app.show_sections {
        egui::Stroke::new(1.0_f32, theme::translucent(theme::SECONDARY_AMBER, 110))
    } else {
        egui::Stroke::new(1.5_f32, theme::SECONDARY_AMBER)
    };
    for sm in &app.model.secondary_members {
        let n0 = sm.nodes[0].index();
        let n1 = sm.nodes[1].index();
        if n0 < pts.len() && n1 < pts.len() {
            painter.line_segment(
                [
                    egui::pos2(pts[n0][0], pts[n0][1]),
                    egui::pos2(pts[n1][0], pts[n1][1]),
                ],
                secondary_stroke,
            );
        }
    }

    // 断面を描けなかった線材（断面未割当・形状情報なし）があれば右上に注記
    if app.show_sections && solids_skipped > 0 {
        painter.text(
            egui::pos2(
                painter.clip_rect().max.x - 10.0,
                painter.clip_rect().min.y + 10.0,
            ),
            egui::Align2::RIGHT_TOP,
            format!("断面未定義の部材 {} 本は線のみ表示", solids_skipped),
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
    }

    // --- 応力図（N/Q/M）: 部材ローカルに沿って描画 ---
    if matches!(mode, ViewMode::N | ViewMode::Q | ViewMode::M) {
        draw_force_diagram(&painter, app, mode, &coords3, center3, &cam, scale, center);
    }
    if mode == ViewMode::Cmq {
        draw_cmq_diagram(&painter, app, &coords3, center3, &cam, scale, center);
    }

    // 選択ハイライト
    for &elem_id in &app.selection.members {
        if let Some(elem) = app.model.elements.iter().find(|e| e.id == elem_id) {
            if elem.nodes.len() >= 2 {
                let n0 = elem.nodes[0].index();
                let n1 = elem.nodes[1].index();
                if n0 < pts.len() && n1 < pts.len() {
                    painter.line_segment(
                        [
                            egui::pos2(pts[n0][0], pts[n0][1]),
                            egui::pos2(pts[n1][0], pts[n1][1]),
                        ],
                        egui::Stroke::new(4.0_f32, theme::PARETO_RED),
                    );
                }
            }
        }
    }

    // --- 支持条件シンボル ---
    // 固定方向へ軸色の矢印、回転軸まわりに円弧を描く。
    // 部材・応力図の上に重ねて描き、支持方向を一目で判別できるようにする。
    // スクリーン上で矢印 18px・円弧半径 12px になるようワールド長を逆算する。
    const SUPPORT_ARROW_PX: f32 = 18.0;
    const SUPPORT_ARC_PX: f32 = 12.0;
    let mut has_support = false;
    for node in &app.model.nodes {
        let kind = support_kind(node.restraint);
        if kind == SupportKind::Free {
            continue;
        }
        has_support = true;
        draw_support_symbol(
            &painter,
            node.coord,
            center3,
            &cam,
            scale,
            center,
            node.restraint,
            SUPPORT_ARROW_PX,
            SUPPORT_ARC_PX,
        );
    }
    if has_support {
        draw_support_legend(&painter);
    }

    // 右上に ViewCube、右下にカメラ追従の座標系アイコン（常に手前に表示。
    // 左下は支持条件凡例が使うため、これらは右側へ配置する）
    viewcube::draw(&painter, &cam, &cube_layout, cube_hover);
    draw_axis_gadget(&painter, &cam);

    // カメラ状態を保存
    app.camera = cam;
}

/// 応力図・CMQ 図のオフセット方向（要素ローカル y 軸）をワールド座標で返す。
///
/// N/Qy/Mz はローカル x-y 平面（曲げ平面）の成分のため、図はローカル y 方向へ
/// 張り出す。解析と同じ局所座標系（[`LocalFrame`]: ex=材軸, ey=ref_vector 直交化）
/// を使うことで、ビューを回転しても図が要素座標系に固定される。
fn diagram_offset_dir(p_i: [f64; 3], p_j: [f64; 3], ref_vector: [f64; 3]) -> [f64; 3] {
    squid_n_element::transform::LocalFrame::from_nodes(p_i, p_j, ref_vector).rot[1]
}

/// 部材両端間のワールド距離。ゼロ長部材（材軸が定まらない）の除外判定に使う。
fn member_len3(p_i: [f64; 3], p_j: [f64; 3]) -> f64 {
    ((p_j[0] - p_i[0]).powi(2) + (p_j[1] - p_i[1]).powi(2) + (p_j[2] - p_i[2]).powi(2)).sqrt()
}

/// 3D 位置 `base3` から `dir3` 方向へ `off_world` だけ張り出した点を投影する。
fn project_offset(
    base3: [f64; 3],
    dir3: [f64; 3],
    off_world: f64,
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) -> egui::Pos2 {
    let p = project(
        [
            base3[0] + dir3[0] * off_world,
            base3[1] + dir3[1] * off_world,
            base3[2] + dir3[2] * off_world,
        ],
        center3,
        cam,
        scale,
        screen_center,
    );
    egui::pos2(p[0], p[1])
}

/// 部材ローカルに沿って N/Q/M 図を描く。
///
/// 図は要素ローカル y 軸方向（曲げ平面内）へワールド空間で張り出してから投影する。
/// スクリーン上の部材線の法線ではなくワールドの要素軸を使うため、ビューを回転
/// しても図の張り出し面は要素座標系に追従する（曲げ平面を真横から見ると線に潰れる）。
#[allow(clippy::too_many_arguments)]
fn draw_force_diagram(
    painter: &egui::Painter,
    app: &App,
    mode: ViewMode,
    coords3: &[[f64; 3]],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
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
        let ey = diagram_offset_dir(p_i, p_j, elem.local_axis.ref_vector);
        let p0 = {
            let p = project(p_i, center3, cam, scale, screen_center);
            egui::pos2(p[0], p[1])
        };
        let p1 = {
            let p = project(p_j, center3, cam, scale, screen_center);
            egui::pos2(p[0], p[1])
        };

        // 評価位置の内力をローカル y 方向（負側=梁下側）へプロット
        let mut diagram_pts: Vec<egui::Pos2> = Vec::new();
        for (xi, forces) in &mf.at {
            let val = forces[force_idx];
            let base3 = [
                p_i[0] + (p_j[0] - p_i[0]) * xi,
                p_i[1] + (p_j[1] - p_i[1]) * xi,
                p_i[2] + (p_j[2] - p_i[2]) * xi,
            ];
            diagram_pts.push(project_offset(
                base3,
                ey,
                -val * amp_world,
                center3,
                cam,
                scale,
                screen_center,
            ));
        }
        if diagram_pts.len() >= 2 {
            // 図形（折れ線→ポリゴン）
            let mut poly = Vec::with_capacity(diagram_pts.len() + 2);
            poly.push(p0);
            poly.extend(diagram_pts);
            poly.push(p1);
            painter.add(egui::Shape::convex_polygon(
                poly,
                theme::translucent(theme::DATA_BLUE, 60),
                egui::Stroke::new(1.5_f32, theme::DATA_BLUE),
            ));
        }
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
}

/// 部材ローカルに沿って CMQ 図（両端の固定端モーメント C とせん断 Q）を描く。
///
/// N/Q/M 図と同様、張り出し方向は要素ローカル y 軸（曲げ平面内）をワールド空間で
/// とってから投影する。CMQ は鉛直床荷重による強軸曲げのため、水平梁では鉛直面内の
/// 図となり、ビューを回転しても要素座標系に固定される。
#[allow(clippy::too_many_arguments)]
fn draw_cmq_diagram(
    painter: &egui::Painter,
    app: &App,
    coords3: &[[f64; 3]],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    if app.beam_loads.is_empty() {
        // スラブ自体が無いのか、スラブはあるが床荷重（強度）が 0 なのかを区別して案内する。
        let msg = if app.model.slabs.is_empty() {
            "スラブが未定義です。モデルタブの「スラブ」でスラブと床荷重を定義すると CMQ 図を表示できます"
        } else {
            "スラブの床荷重が 0 です。荷重タブ（スラブ）で固定荷重・用途（積載）を設定すると CMQ 図を表示できます"
        };
        painter.text(
            egui::pos2(
                painter.clip_rect().min.x + 10.0,
                painter.clip_rect().min.y + 30.0,
            ),
            egui::Align2::LEFT_TOP,
            msg,
            egui::FontId::proportional(13.0),
            theme::GRAY_600,
        );
        return;
    }

    let max_c = app
        .beam_loads
        .iter()
        .map(|bl| bl.cmq.c_i.abs().max(bl.cmq.c_j.abs()))
        .fold(0.0_f64, f64::max);
    let max_q = app
        .beam_loads
        .iter()
        .map(|bl| bl.cmq.q_i.abs().max(bl.cmq.q_j.abs()))
        .fold(0.0_f64, f64::max);
    if max_c < 1e-12 && max_q < 1e-12 {
        return;
    }
    // 最大値で 60px 相当のワールド長（一様スケール正射影なので px/scale=ワールド長）
    let c_amp = 60.0 / max_c.max(1e-12) / scale as f64;
    let q_amp = 60.0 / max_q.max(1e-12) / scale as f64;

    for bl in &app.beam_loads {
        // 描画スパンの両端節点: 実部材に解決済みなら部材の両端とその局所座標系、
        // 未解決の節点対（二次部材（小梁）上の辺・大梁の中間区間）は節点対そのものに
        // 沿って描き、既定の参照ベクトル（水平材=鉛直上基準、鉛直材=X 基準）を使う。
        let endpoints = match app.model.elements.iter().find(|e| e.id == bl.elem) {
            Some(elem) if elem.nodes.len() >= 2 => Some((
                elem.nodes[0].index(),
                elem.nodes[1].index(),
                Some(elem.local_axis.ref_vector),
            )),
            _ => match bl.target {
                squid_n_load::floor::LoadTarget::Span([a, b]) => Some((a.index(), b.index(), None)),
                _ => None,
            },
        };
        let Some((n0, n1, ref_vec)) = endpoints else {
            continue;
        };
        if n0 >= coords3.len() || n1 >= coords3.len() {
            continue;
        }
        let p_i = coords3[n0];
        let p_j = coords3[n1];
        if member_len3(p_i, p_j) < 1e-9 {
            continue; // ゼロ長スパン（同一節点間）は材軸が定まらず図を描けない
        }
        let ref_vec = ref_vec.unwrap_or_else(|| {
            let dxy = ((p_j[0] - p_i[0]).powi(2) + (p_j[1] - p_i[1]).powi(2)).sqrt();
            if dxy < 1.0 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 0.0, 1.0]
            }
        });
        let ey = diagram_offset_dir(p_i, p_j, ref_vec);
        let p0 = {
            let p = project(p_i, center3, cam, scale, screen_center);
            egui::pos2(p[0], p[1])
        };
        let p1 = {
            let p = project(p_j, center3, cam, scale, screen_center);
            egui::pos2(p[0], p[1])
        };

        // C 図（モーメント）: 両端の c_i, c_j を結ぶ折れ線ポリゴン（-ey 側=梁下側）
        let c_poly = vec![
            p0,
            project_offset(
                p_i,
                ey,
                -bl.cmq.c_i * c_amp,
                center3,
                cam,
                scale,
                screen_center,
            ),
            project_offset(
                p_j,
                ey,
                -bl.cmq.c_j * c_amp,
                center3,
                cam,
                scale,
                screen_center,
            ),
            p1,
        ];
        // C 図（モーメント）= 通常データ（青）
        painter.add(egui::Shape::convex_polygon(
            c_poly,
            theme::translucent(theme::DATA_BLUE, 60),
            egui::Stroke::new(1.5_f32, theme::DATA_BLUE),
        ));

        // Q 図（せん断）: 両端の q_i, q_j を結ぶ折れ線ポリゴン（反対側 +ey 側に描画）
        let q_poly = vec![
            p0,
            project_offset(
                p_i,
                ey,
                bl.cmq.q_i * q_amp,
                center3,
                cam,
                scale,
                screen_center,
            ),
            project_offset(
                p_j,
                ey,
                bl.cmq.q_j * q_amp,
                center3,
                cam,
                scale,
                screen_center,
            ),
            p1,
        ];
        // Q 図（せん断）= 良好系（緑）。C（青）と弁別する
        painter.add(egui::Shape::convex_polygon(
            q_poly,
            theme::translucent(theme::GOOD_GREEN, 60),
            egui::Stroke::new(1.5_f32, theme::GOOD_GREEN),
        ));
    }

    // 凡例
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        format!("CMQ図 C(max={:.2}) 青／Q(max={:.2}) 緑", max_c, max_q),
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
}

/// スラブ（床）と小梁を描画する。
///
/// スラブは解析部材ではなく荷重分配オブジェクトのため、構造部材（実線・青/グレー系）と
/// 一目で区別できるフォーマットで描く:
/// - スラブ面: 暖色（BEST_YELLOW）の淡い半透明フィル＋破線の輪郭
/// - 小梁（`JoistLine`）: `support` 節点間の破線。実部材化された小梁は部材線
///   （実線）が上から重なるため、破線だけの線＝仮想小梁（荷重分配上の存在）と判別できる
///
/// 節点座標は投影済み `pts` を使うため、変形図・モード形では変位に追従する。
/// 節点削除等で陳腐化した参照（範囲外 id）を含むスラブ・小梁は描かない。
fn draw_slabs_and_joists(painter: &egui::Painter, app: &App, pts: &[[f32; 2]]) {
    /// 破線パターン（描画長 / 間隔, px）
    const DASH: f32 = 6.0;
    const GAP: f32 = 4.0;

    for slab in &app.model.slabs {
        let poly: Vec<egui::Pos2> = slab
            .boundary
            .iter()
            .filter_map(|n| {
                let idx = n.index();
                (idx < pts.len()).then(|| egui::pos2(pts[idx][0], pts[idx][1]))
            })
            .collect();
        if poly.len() == slab.boundary.len() && poly.len() >= 3 {
            // 面: 淡い半透明の暖色フィル（壁の青と弁別）
            painter.add(egui::Shape::convex_polygon(
                poly.clone(),
                theme::translucent(theme::BEST_YELLOW, 28),
                egui::Stroke::NONE,
            ));
            // 輪郭: 破線（実部材の実線と弁別）
            let mut closed = poly.clone();
            closed.push(poly[0]);
            painter.extend(egui::Shape::dashed_line(
                &closed,
                egui::Stroke::new(1.5_f32, theme::translucent(theme::BEST_YELLOW, 220)),
                DASH,
                GAP,
            ));
        }

        // 小梁: support 節点間の破線（ニュートラル色。スラブ輪郭の暖色とも弁別）
        for joist in &slab.joists {
            let i0 = joist.support[0].index();
            let i1 = joist.support[1].index();
            if i0 >= pts.len() || i1 >= pts.len() {
                continue;
            }
            painter.extend(egui::Shape::dashed_line(
                &[
                    egui::pos2(pts[i0][0], pts[i0][1]),
                    egui::pos2(pts[i1][0], pts[i1][1]),
                ],
                egui::Stroke::new(1.5_f32, theme::GRAY_600),
                DASH,
                GAP,
            ));
        }
    }
}

/// 壁の頂点を自己交差しない多角形になるよう並べ替える。
/// クリック順は任意なので、節点の重心まわりの偏角で反時計回りにソートする。
/// 節点が同一平面上にあることを前提に、面内 2 軸へ投影して角度を求める。
fn order_wall_nodes(
    model: &squid_n_core::model::Model,
    node_ids: &[squid_n_core::ids::NodeId],
) -> Vec<squid_n_core::ids::NodeId> {
    // 各節点の座標を取得（見つからなければ並べ替えせず返す）
    let coords: Vec<[f64; 3]> = node_ids
        .iter()
        .map(|id| {
            model
                .nodes
                .iter()
                .find(|n| n.id == *id)
                .map(|n| n.coord)
                .unwrap_or([0.0; 3])
        })
        .collect();
    if coords.len() < 3 {
        return node_ids.to_vec();
    }

    // 重心
    let n = coords.len() as f64;
    let centroid = [
        coords.iter().map(|c| c[0]).sum::<f64>() / n,
        coords.iter().map(|c| c[1]).sum::<f64>() / n,
        coords.iter().map(|c| c[2]).sum::<f64>() / n,
    ];

    // 面の法線（最初の非共線な 3 点の外積）。面内基底 u, v を作る。
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let norm = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    let u = {
        let d = sub(coords[1], coords[0]);
        let len = norm(d);
        if len < 1e-9 {
            [1.0, 0.0, 0.0]
        } else {
            [d[0] / len, d[1] / len, d[2] / len]
        }
    };
    // u に直交し面内に収まる v を、法線×u から作る
    let mut normal = [0.0; 3];
    for c in coords.iter().skip(2) {
        let cand = cross(sub(coords[1], coords[0]), sub(*c, coords[0]));
        if norm(cand) > 1e-9 {
            normal = cand;
            break;
        }
    }
    let v = {
        let cand = cross(normal, u);
        let len = norm(cand);
        if len < 1e-9 {
            // 退化（共線）時は並べ替えしない
            return node_ids.to_vec();
        }
        [cand[0] / len, cand[1] / len, cand[2] / len]
    };

    // 重心からの相対ベクトルを (u, v) に投影し偏角でソート
    let mut indexed: Vec<(usize, f64)> = coords
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let r = sub(*c, centroid);
            let pu = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
            let pv = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
            (i, pv.atan2(pu))
        })
        .collect();
    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    indexed.into_iter().map(|(i, _)| node_ids[i]).collect()
}

/// 点 p から線分 ab までの最短距離（スクリーン座標, px）。
fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len_sq = ab.x * ab.x + ab.y * ab.y;
    if len_sq < 1e-6 {
        return ap.length();
    }
    let t = ((ap.x * ab.x + ap.y * ab.y) / len_sq).clamp(0.0, 1.0);
    let proj = egui::pos2(a.x + ab.x * t, a.y + ab.y * t);
    (p - proj).length()
}

/// モデルのバウンディングボックス（min, max）。空なら原点を返す。
fn model_bbox(model: &squid_n_core::model::Model) -> ([f64; 3], [f64; 3]) {
    if model.nodes.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for n in &model.nodes {
        for k in 0..3 {
            min[k] = min[k].min(n.coord[k]);
            max[k] = max[k].max(n.coord[k]);
        }
    }
    (min, max)
}

/// モデルのバウンディングボックス対角線長。
fn model_bbox_size(model: &squid_n_core::model::Model) -> f64 {
    if model.nodes.is_empty() {
        return 1.0;
    }
    let (min, max) = model_bbox(model);
    ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2) + (max[2] - min[2]).powi(2)).sqrt()
}

/// §3-2 の 3D 規約に沿ってグリッド・座標軸（赤=X / 緑=Y / 青=Z）・原点マーカーを描く。
///
/// グリッド間隔は 1 m（= 1000 mm）固定。XY 平面（z=0）にのみ描画する。
/// 描画範囲はビューポートに映るワールド範囲（`rect` と `scale` から逆算）を
/// 1000 mm の倍数に切り上げて決めるため、モデルのバウンディングボックスに依存しない。
/// 軸線は原点から両方向（正=濃色 / 負=淡色）へ伸ばし、原点位置を一目で判別できるようにする。
/// 軸ラベルの値はワールド座標（実寸）を表示する。
fn draw_grid_and_axes(
    painter: &egui::Painter,
    rect: egui::Rect,
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    let proj = |p: [f64; 3]| {
        let s = project(p, center3, cam, scale, screen_center);
        egui::pos2(s[0], s[1])
    };

    /// グリッド間隔 [mm]（1 m）。
    const STEP: f64 = 1000.0;
    // ダーク半透明・線幅 0.5（淡グレー背景の上で奥行きを示す）
    let grid_stroke = egui::Stroke::new(0.5_f32, egui::Color32::from_black_alpha(36));
    let origin: [f64; 3] = [0.0; 3];

    // ビューポートに映るワールド範囲を計算。対角ピクセル長 / scale で大まかな半径を得て
    // 余裕（1.5 倍）を持たせる（回転で端が見切れないように）。
    let view_radius = (rect.width().hypot(rect.height()) / scale) as f64 * 0.75;

    // 各軸の描画範囲: center3 ± view_radius を STEP の倍数に丸める
    let range = [
        (
            ((center3[0] - view_radius) / STEP).floor() * STEP,
            ((center3[0] + view_radius) / STEP).ceil() * STEP,
        ),
        (
            ((center3[1] - view_radius) / STEP).floor() * STEP,
            ((center3[1] + view_radius) / STEP).ceil() * STEP,
        ),
        (
            ((center3[2] - view_radius) / STEP).floor() * STEP,
            ((center3[2] + view_radius) / STEP).ceil() * STEP,
        ),
    ];

    // XY 平面（z=0）の格子線を描く。a=X, b=Y 方向に原点基準で線を引く。
    let a = 0usize; // X
    let b = 1usize; // Y
    let a_lo = (range[a].0 / STEP).round() as i64;
    let a_hi = (range[a].1 / STEP).round() as i64;
    for k in a_lo..=a_hi {
        let av = k as f64 * STEP;
        let p0 = [av, range[b].0, origin[2]];
        let p1 = [av, range[b].1, origin[2]];
        painter.line_segment([proj(p0), proj(p1)], grid_stroke);
    }
    let b_lo = (range[b].0 / STEP).round() as i64;
    let b_hi = (range[b].1 / STEP).round() as i64;
    for k in b_lo..=b_hi {
        let bv = k as f64 * STEP;
        let q0 = [range[a].0, bv, origin[2]];
        let q1 = [range[a].1, bv, origin[2]];
        painter.line_segment([proj(q0), proj(q1)], grid_stroke);
    }

    // 原点からの座標軸（赤=X / 緑=Y / 青=Z）。正方向=濃色 / 負方向=淡色。
    for (axis, col, name) in [
        (0usize, theme::AXIS_X, "X"),
        (1, theme::AXIS_Y, "Y"),
        (2, theme::AXIS_Z, "Z"),
    ] {
        // 正方向: 原点 → range の上端
        let mut pe = origin;
        pe[axis] = range[axis].1;
        painter.line_segment([proj(origin), proj(pe)], egui::Stroke::new(1.5_f32, col));
        painter.text(
            proj(pe),
            egui::Align2::LEFT_BOTTOM,
            format!("{} ({:.1})", name, range[axis].1),
            egui::FontId::proportional(11.0),
            col,
        );
        // 負方向: 原点 → range の下端（淡色）
        let mut pn = origin;
        pn[axis] = range[axis].0;
        painter.line_segment(
            [proj(origin), proj(pn)],
            egui::Stroke::new(1.0_f32, theme::lighten(col, 0.45)),
        );
        painter.text(
            proj(pn),
            egui::Align2::RIGHT_TOP,
            format!("{:.1}", range[axis].0),
            egui::FontId::proportional(10.0),
            theme::lighten(col, 0.45),
        );
    }

    // 原点マーカー（黒点 + "O" ラベル）
    let op = proj(origin);
    painter.circle_filled(op, 3.0, theme::GRAY_900);
    painter.text(
        egui::pos2(op.x + 6.0, op.y - 6.0),
        egui::Align2::LEFT_BOTTOM,
        "O",
        egui::FontId::proportional(11.0),
        theme::GRAY_900,
    );
}

/// ビューポート右下にカメラの向きへ追従する座標系アイコン（XYZ 軸ガジェット）を描く。
///
/// CAD ソフトで一般的な、画面端に固定された小さな座標系。各軸をカメラの回転
/// クォータニオンで投影し、Z（手前）成分でソートして奥から描くことで
/// 手前の軸が上に重なる。軸色は 3D ビューと同一（赤=X / 緑=Y / 青=Z）。
/// 左下は支持条件凡例、右上は ViewCube が使うため右下に置く。
fn draw_axis_gadget(painter: &egui::Painter, cam: &CameraState) {
    let rect = painter.clip_rect();
    let center = egui::pos2(rect.max.x - 45.0, rect.max.y - 45.0);
    const LEN: f32 = 28.0;

    let axes: [([f32; 3], egui::Color32, &str); 3] = [
        ([1.0, 0.0, 0.0], theme::AXIS_X, "X"),
        ([0.0, 1.0, 0.0], theme::AXIS_Y, "Y"),
        ([0.0, 0.0, 1.0], theme::AXIS_Z, "Z"),
    ];

    // 各軸をカメラ回転で投影。r[0]=右, r[1]=上（画面Yは下向きなので反転）, r[2]=手前
    let mut projected: Vec<(egui::Vec2, egui::Color32, &str, f32)> = axes
        .iter()
        .map(|(v, col, name)| {
            let r = q_rotate(cam.rot, *v);
            (egui::vec2(r[0], -r[1]), *col, *name, r[2])
        })
        .collect();
    // r[2]（手前=正）が小さい（奥）順に描く → 手前の軸が最後に描かれ上に来る
    projected.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal));

    // 背景円（軸が背景と混ざらないよう淡い白）
    painter.circle_filled(center, LEN + 8.0, theme::translucent(theme::WHITE, 200));

    for (dir, col, name, _) in &projected {
        let end = center + *dir * LEN;
        draw_arrow(painter, center, end, *col);
        let label_pos = center + *dir * (LEN + 10.0);
        painter.text(
            label_pos,
            egui::Align2::CENTER_CENTER,
            *name,
            egui::FontId::proportional(12.0),
            *col,
        );
    }
    // 中心点
    painter.circle_filled(center, 2.0, theme::GRAY_900);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ワールド Z 軸（鉛直軸）のビュー空間での向き。
    /// 画面上で縦 ⇔ x 成分が 0、かつ上向き ⇔ y 成分が非負（project は y を反転して描画する）。
    fn world_z_in_view(cam: &CameraState) -> [f32; 3] {
        q_rotate(cam.rot, [0.0, 0.0, 1.0])
    }

    #[test]
    fn 既定ビューで鉛直軸は画面上で縦() {
        let cam = CameraState::default();
        let z = world_z_in_view(&cam);
        assert!(z[0].abs() < 1e-5, "Z 軸が画面上で傾いている: {z:?}");
        assert!(z[1] >= -1e-6, "Z 軸が画面上で下向き: {z:?}");
    }

    #[test]
    fn ドラッグ回転を繰り返しても鉛直軸は傾かない() {
        // アークボール時代の不具合: 斜めドラッグの繰り返しでロールが蓄積し、
        // 鉛直軸が画面上で斜めに傾いていた。ターンテーブルでは起きないことを確認する。
        let mut cam = CameraState::default();
        let drags = [
            (30.0, -20.0),
            (-50.0, 40.0),
            (100.0, 100.0),
            (-15.0, -80.0),
            (200.0, 5.0),
            (-3.0, 60.0),
        ];
        for _ in 0..50 {
            for (dx, dy) in drags {
                cam.turntable_drag(dx, dy);
                let z = world_z_in_view(&cam);
                assert!(z[0].abs() < 1e-4, "Z 軸が画面上で傾いた: {z:?}");
                assert!(z[1] >= -1e-4, "Z 軸が画面上で下向きになった: {z:?}");
            }
        }
    }

    #[test]
    fn 俯仰は真上と真下でクランプされる() {
        let mut cam = CameraState::default();
        cam.turntable_drag(0.0, 1e6); // 大きく下ドラッグ → 真上（平面図）で停止
        assert!((cam.pitch - 0.0).abs() < 1e-6);
        cam.turntable_drag(0.0, -1e6); // 大きく上ドラッグ → 真下で停止
        assert!((cam.pitch + std::f32::consts::PI).abs() < 1e-6);
    }
}
