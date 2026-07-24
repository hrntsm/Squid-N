use crate::app::App;
use crate::theme;

mod viewcube;
use squid_n_core::dof::{Dof, Dof6Mask};

mod check_ratio;
mod diagram;
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
    /// 検定比図（部材検定の最大検定比で着色）
    CheckRatio,
}

/// CMQ 図で表示する成分（C: 固定端モーメント／M: 単純梁中央モーメント／Q: せん断）。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum CmqComponent {
    /// 固定端モーメント C 図
    #[default]
    C,
    /// 単純梁としての曲げモーメント M 図（中央モーメントの目安）
    M,
    /// せん断 Q 図
    Q,
}

/// 検定比図の着色対象（最大＝全式の max、または特定の検定式のみ）。
///
/// `Kind` を選ぶと、部材・節点の色や中点ラベル・位置別マーカーが当該検定式
/// だけの検定比（`CheckResult::components` から抽出）に基づいて決まる。
/// 対象の式が存在しない検定位置は「フィルタ対象外」として着色・マーカー
/// ともに描かない（詳細は `check_ratio.rs` の `ratio_for_filter` を参照）。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum CheckRatioFilter {
    /// 全検定式中の最大検定比（既定）
    #[default]
    Max,
    /// 特定の検定式のみ
    Kind(squid_n_design_jp::CheckKind),
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

/// 3D→2D 投影の文脈（回転中心・カメラ・スケール・描画領域中心）を束ねる。
///
/// 多数の描画関数へ `(center3, cam, scale, screen_center)` を個別に引き回す代わりに、
/// この 1 つの参照で受け渡し、投影数式の単一情報源とする（§3-2: ターンテーブル
/// 回転＋正射影。ビュー軸は X=右・Y=上・Z=手前）。深度ソートや面陰影で回転後の
/// カメラ空間ベクトルが要る箇所のため、`to_cam`（回転まで）と `cam_to_screen`
/// （画面写像）に分けて公開する。
#[derive(Clone, Copy)]
pub(crate) struct Projector<'a> {
    /// モデル中心（回転中心）。
    center3: [f64; 3],
    cam: &'a CameraState,
    /// px/世界長。
    scale: f32,
    /// 描画領域中心（px）。
    screen_center: [f32; 2],
}

impl<'a> Projector<'a> {
    pub(crate) fn new(
        center3: [f64; 3],
        cam: &'a CameraState,
        scale: f32,
        screen_center: [f32; 2],
    ) -> Self {
        Self {
            center3,
            cam,
            scale,
            screen_center,
        }
    }

    /// px/世界長。ワールド長 ⇔ 画面 px の換算に使う。
    pub(crate) fn scale(&self) -> f32 {
        self.scale
    }

    /// モデル中心（回転中心）。グリッド範囲の算定などワールド基準の計算に使う。
    pub(crate) fn center3(&self) -> [f64; 3] {
        self.center3
    }

    /// ワールド座標を、回転中心基準でカメラ回転を掛けたカメラ空間ベクトルへ変換する
    /// （r[0]=右, r[1]=上, r[2]=手前）。深度ソート・面陰影で中間ベクトルが要る。
    pub(crate) fn cam_space(&self, p: [f64; 3]) -> [f32; 3] {
        let v = [
            (p[0] - self.center3[0]) as f32,
            (p[1] - self.center3[1]) as f32,
            (p[2] - self.center3[2]) as f32,
        ];
        q_rotate(self.cam.rot, v)
    }

    /// カメラ空間ベクトルをスクリーン座標へ写す（パン加算・スケール・画面 Y 反転）。
    pub(crate) fn cam_to_screen(&self, r: [f32; 3]) -> egui::Pos2 {
        egui::pos2(
            self.screen_center[0] + self.cam.pan[0] + r[0] * self.scale,
            self.screen_center[1] + self.cam.pan[1] - r[1] * self.scale,
        )
    }

    /// ワールド座標 `p` をスクリーン座標へ投影する。
    pub(crate) fn project(&self, p: [f64; 3]) -> egui::Pos2 {
        self.cam_to_screen(self.cam_space(p))
    }

    /// `base3` から `dir3` 方向へ `off_world` だけ張り出した点を投影する。
    pub(crate) fn project_offset(
        &self,
        base3: [f64; 3],
        dir3: [f64; 3],
        off_world: f64,
    ) -> egui::Pos2 {
        self.project([
            base3[0] + dir3[0] * off_world,
            base3[1] + dir3[1] * off_world,
            base3[2] + dir3[2] * off_world,
        ])
    }
}

/// ワールド座標 `p` を投影する（`[f32; 2]` 版。M-N 相関曲面ビュー用の下位ラッパ）。
/// 投影数式は [`Projector`] を単一情報源とする。
pub(crate) fn project(
    p: [f64; 3],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) -> [f32; 2] {
    let pos = Projector::new(center3, cam, scale, screen_center).project(p);
    [pos.x, pos.y]
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
fn draw_rotation_arc(
    painter: &egui::Painter,
    proj: &Projector,
    center_world: [f64; 3],
    axis: [f64; 3],
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
        let cur = proj.project(pt);
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
fn draw_support_symbol(
    painter: &egui::Painter,
    proj: &Projector,
    node_coord: [f64; 3],
    restraint: Dof6Mask,
    arrow_px: f32,
    arc_px: f32,
) {
    if support_kind(restraint) == SupportKind::Free {
        return;
    }
    // スクリーン上で arrow_px / arc_px になるようワールド長を逆算
    let arrow_world = arrow_px as f64 / proj.scale() as f64;
    let arc_world = arc_px as f64 / proj.scale() as f64;
    let origin = proj.project(node_coord);

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
            draw_arrow(painter, origin, proj.project(end), color);
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
            draw_rotation_arc(painter, proj, node_coord, axis, arc_world, color);
        }
    }
}

/// 支持条件シンボルの凡例をビュー左下に描く。
/// `has_diaphragm` が真のとき、剛床マーク（面内拘束）の説明行を追加する。
fn draw_support_legend(painter: &egui::Painter, has_diaphragm: bool) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y0 = rect.max.y - 10.0;

    // 剛床マークの説明（面内拘束 Ux/Uy/Rz）を最下段へ追加する。
    if has_diaphragm {
        painter.text(
            egui::pos2(x0, y0),
            egui::Align2::LEFT_BOTTOM,
            "剛床マーク: 面内拘束 (Ux/Uy/Rz)",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        // 以降の支持条件凡例を 1 行分上へずらす。
        y0 -= 16.0;
    }

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
    let mut mode_idx = app.view_mode_idx;
    let mut cmq_component = app.cmq_component;
    let mut check_ratio_filter = app.check_ratio_filter;

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
        ui.selectable_value(&mut mode, ViewMode::CheckRatio, "検定比");
        ui.separator();
        // 断面表示: 部材を断面形状の押し出しソリッドで立体表示（全モードと併用可）
        ui.toggle_value(&mut app.show_sections, "断面表示");
        // 床（スラブ・小梁）・二次部材の表示切替（全モードと併用可。
        // CMQ 図は主架構の図のため設定によらず常に非表示）
        ui.toggle_value(&mut app.show_floor_secondary, "床・二次部材");
        // 剛床代表点（重心マスター）の表示切替。剛床がある場合のみ選択肢を出す。
        // ON にすると代表点マーカー・面内拘束マーク・スレーブへの点線を描く。
        let has_diaphragm_constraint = app
            .model
            .constraints
            .iter()
            .any(|c| matches!(c, squid_n_core::model::Constraint::RigidDiaphragm { .. }));
        if has_diaphragm_constraint {
            ui.toggle_value(&mut app.show_diaphragm_master, "剛床代表点");
        }
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
    if mode == ViewMode::Cmq {
        ui.horizontal(|ui| {
            ui.label("成分:");
            ui.selectable_value(&mut cmq_component, CmqComponent::C, "C(モーメント)");
            ui.selectable_value(&mut cmq_component, CmqComponent::M, "M(中央)");
            ui.selectable_value(&mut cmq_component, CmqComponent::Q, "Q(せん断)");
        });
    }
    // N/Q/M 図: 単色塗り／コンター（値に応じた色分け）を切替。
    // コンター ON 時のみカラーマップ選択（既定 Viridis。TONMANUAL §3）を表示する。
    if matches!(mode, ViewMode::N | ViewMode::Q | ViewMode::M) {
        ui.horizontal(|ui| {
            // 応力図に変形図を重ねる（変位は自動倍率で節点座標に加味され、
            // 図も変形後の材軸に沿って描かれる）
            ui.toggle_value(&mut app.overlay_deform, "変形表示");
            ui.toggle_value(&mut app.diagram_contour, "コンター");
            if app.diagram_contour {
                let mut colormap = app.contour_colormap;
                egui::ComboBox::from_id_salt("contour_colormap")
                    .selected_text(colormap.label())
                    .show_ui(ui, |ui| {
                        for cm in [
                            theme::ColorMap::Viridis,
                            theme::ColorMap::Plasma,
                            theme::ColorMap::Turbo,
                            theme::ColorMap::Jet,
                            theme::ColorMap::BlueWhiteRed,
                        ] {
                            ui.selectable_value(&mut colormap, cm, cm.label());
                        }
                    });
                app.contour_colormap = colormap;
            }
        });
    }
    // 検定比図: 検定式フィルタ（最大／式別、結果に現れる式のみ選択肢に出す）と
    // 位置別マーカーの表示切替。
    if mode == ViewMode::CheckRatio {
        fn checked_components(
            outcome: &squid_n_design_jp::CheckOutcome,
        ) -> Option<&[squid_n_design_jp::CheckComponent]> {
            match outcome {
                squid_n_design_jp::CheckOutcome::Checked(cr) => Some(cr.components.as_slice()),
                squid_n_design_jp::CheckOutcome::Skipped { .. } => None,
            }
        }
        let available_kinds = app
            .results
            .as_ref()
            .map(|r| {
                check_ratio::available_check_kinds(
                    r.member_checks
                        .iter()
                        .flat_map(|m| m.positions.iter())
                        .filter_map(|p| checked_components(&p.outcome))
                        .chain(
                            r.joint_checks
                                .iter()
                                .filter_map(|j| checked_components(&j.outcome)),
                        ),
                )
            })
            .unwrap_or_default();
        ui.horizontal_wrapped(|ui| {
            ui.label("検定式:");
            ui.selectable_value(&mut check_ratio_filter, CheckRatioFilter::Max, "最大");
            for k in &available_kinds {
                ui.selectable_value(
                    &mut check_ratio_filter,
                    CheckRatioFilter::Kind(*k),
                    k.label(),
                );
            }
            ui.separator();
            ui.checkbox(&mut app.check_ratio_markers, "位置別マーカー");
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
    // 変形表示オプション行: 変形を表示するモード（変形・モード・応力図の変形重ね）で
    // 表示する。「内部たわみ」トグルで梁の Hermite 曲線表示（＋床・二次部材の曲線
    // 追従）と直線表示（全体の変形）を切り替え、変形倍率スライダーで自動算定倍率への
    // 手動係数を対数調整（「リセット」で 1.0）する。
    let show_deform_options = matches!(mode, ViewMode::Deformed | ViewMode::Mode)
        || (matches!(mode, ViewMode::N | ViewMode::Q | ViewMode::M) && app.overlay_deform);
    if show_deform_options {
        ui.horizontal(|ui| {
            ui.toggle_value(&mut app.show_beam_interpolation, "内部たわみ")
                .on_hover_text(
                    "梁を内部たわみ（Hermite 曲線）で描き、床・二次部材も曲線に追従。\
                     OFF で梁を直線（弦）にし全体の変形を見る",
                );
            ui.separator();
            ui.label("変形倍率:");
            ui.add(
                egui::Slider::new(&mut app.deform_scale_factor, 0.1..=10.0)
                    .logarithmic(true)
                    .text("×（自動比）"),
            );
            if ui.button("リセット").clicked() {
                app.deform_scale_factor = 1.0;
            }
        });
    }

    app.view_mode = mode;
    app.view_mode_idx = mode_idx;
    app.cmq_component = cmq_component;
    app.check_ratio_filter = check_ratio_filter;

    // CMQ 図はモデル編集に常に追従させるため、表示中は毎フレーム再計算する
    // （スラブ数は小さい前提）。
    if app.view_mode == ViewMode::Cmq {
        app.refresh_beam_loads();
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
    // 以降の描画で共有する投影文脈（カメラ確定後に 1 度だけ構築）。
    let proj = Projector::new(center3, &cam, scale, center);

    // グリッド・軸（§3-2: 赤=X / 緑=Y / 青=Z）。モデルの背後に先に描く。
    draw_grid_and_axes(&painter, rect, &proj);

    // 節点座標（変形・モード時と、N/Q/M 図の変形重ね表示時は変位を加味）
    let disp = match mode {
        ViewMode::Deformed => app.current_static().map(|s| s.disp.clone()),
        ViewMode::N | ViewMode::Q | ViewMode::M if app.overlay_deform => {
            app.current_static().map(|s| s.disp.clone())
        }
        // `ModalResult::shapes` は剛床等の縮約後独立自由度座標のため直接は使えない。
        // ソルバが節点×6へ展開済みの `node_shapes` を用いる。
        ViewMode::Mode => app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.node_shapes.get(mode_idx))
            .cloned(),
        _ => None,
    };

    // 主架構要素に接続しない節点（スラブ境界・小梁支持点・二次部材の節点）は
    // 解析自由度が割り当てられず変位が常にゼロのため（`DofMap` 参照）、最寄りの
    // 主架構部材の変位から補間し、床・二次部材を変形へ追従させる。梁に載る節点は
    // 梁の Hermite 変形曲線上へ載る（`interpolate_unreferenced_disp`）。続けて剛床
    // 代表節点の鉛直変位をスレーブ平均で補い、代表点も床の変形へ追従させる。
    let disp = disp.map(|d| display_disp(&app.model, d, app.show_beam_interpolation));

    // 実効表示倍率（自動倍率 × 手動係数）。詳細は [`deform_display_scale`]。
    let deform_scale_actual = deform_display_scale(
        &app.model,
        disp.as_deref(),
        model_size,
        app.show_beam_interpolation,
        app.deform_scale_factor,
    );

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
    let pts: Vec<egui::Pos2> = coords3.iter().map(|&p| proj.project(p)).collect();

    // --- クリック処理（ViewCube 上のクリックはスナップ済みのため除外） ---
    if response.clicked() && !cube_clicked {
        if let Some(click_pos) = response.interact_pointer_pos() {
            if app.beam_draw_mode {
                // 梁作成モード：クリック位置に最も近い節点を選ぶ
                let best = pick_nearest_node(&pts, click_pos);
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
                let best = pick_nearest_node(&pts, click_pos);
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
                let best = pick_nearest_node(&pts, click_pos);
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
                // ピッキング許容距離（px）
                const PICK_THRESHOLD: f32 = 8.0;
                match pick_nearest_member(&app.model, &pts, click_pos) {
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
    // CMQ 図は全体解析（主架構）に関するものなので、小梁・スラブは表示しない。
    // 「床・二次部材」トグル OFF 時も表示しない。
    if mode != ViewMode::Cmq && app.show_floor_secondary {
        draw_slabs_and_joists(&painter, app, &pts);
    }

    // --- 断面ソリッド ---
    // 節点・部材線より先に描き、線・シンボル類は上に重ねる（材軸が見えるように）。
    let mut solids_skipped = 0usize;
    if app.show_sections {
        solids_skipped = solid::draw_section_solids(&painter, &app.model, &coords3, &proj);
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
                    (idx < pts.len()).then(|| pts[idx])
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
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }

        // 変形を表示する全モード（変形図・モード形・応力図の変形重ね）で、「内部
        // たわみ」トグルが ON のとき、梁は端部の並進・回転から Hermite 3 次で曲げ
        // 変形を内挿して曲線描画する（節点間の直線ではたわみが見えないため）。
        // トグル OFF では梁も直線で描き、全体の変形だけを素直に見る。変形を表示
        // していない（`disp` が None）モードでは常に直線。
        let curved_beam =
            app.show_beam_interpolation && elem.kind == squid_n_core::model::ElementKind::Beam;
        if let (true, Some(d)) = (curved_beam, &disp) {
            let p_i = app.model.nodes[n0].coord;
            let p_j = app.model.nodes[n1].coord;
            if member_len3(p_i, p_j) > 1e-9 {
                let poly3 = BeamDeflection::new(p_i, p_j, d[n0], d[n1], elem.local_axis.ref_vector)
                    .polyline(deform_scale_actual, DEFORM_CURVE_SEGMENTS);
                let screen: Vec<egui::Pos2> = poly3.iter().map(|&p| proj.project(p)).collect();
                painter.add(egui::Shape::line(screen, line_stroke));
                continue;
            }
        }

        // 通常（未変形・その他要素・ゼロ長梁）は節点間を直線で結ぶ。
        painter.line_segment([pts[n0], pts[n1]], line_stroke);
    }

    // 二次部材（小梁・間柱）: 解析対象外だが実在部材なので実線で描く
    // （解析対象外を示す暖色アンバー。スラブの暖色と同族で、主架構の
    // 青/グレーと弁別。断面表示中はソリッドが上に描かれているため
    // 材軸線として薄く重ねる）。
    if app.show_floor_secondary {
        let secondary_stroke = if app.show_sections {
            egui::Stroke::new(1.0_f32, theme::translucent(theme::SECONDARY_AMBER, 110))
        } else {
            egui::Stroke::new(1.5_f32, theme::SECONDARY_AMBER)
        };
        for sm in &app.model.secondary_members {
            let n0 = sm.nodes[0].index();
            let n1 = sm.nodes[1].index();
            if n0 < pts.len() && n1 < pts.len() {
                painter.line_segment([pts[n0], pts[n1]], secondary_stroke);
            }
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
    // 変形重ね（`disp` が Some）かつ内部たわみ表示が有効なとき、梁の張り出しは
    // 変形後の Hermite 曲線を基準線に描く。判定に必要な変位と表示倍率を渡す。
    if matches!(mode, ViewMode::N | ViewMode::Q | ViewMode::M) {
        diagram::draw_force_diagram(
            &painter,
            app,
            mode,
            &coords3,
            disp.as_deref(),
            deform_scale_actual,
            &proj,
        );
    }
    if mode == ViewMode::Cmq {
        draw_cmq_diagram(&painter, app, &coords3, &proj);
    }
    if mode == ViewMode::CheckRatio {
        check_ratio::draw_check_ratio(&painter, app, &pts);
        // B-3: ホバー詳細（ViewCube ホバー中は除く。通常モードのクリック選択と
        // 同じ最近傍部材探索・8px 閾値で最寄り部材を求め、ヒットしたらツールチップ表示）。
        if cube_hover.is_none() {
            if let Some(hover_pos) = response.hover_pos() {
                const HOVER_PICK_THRESHOLD: f32 = 8.0;
                if let Some((id, d)) = pick_nearest_member(&app.model, &pts, hover_pos) {
                    if d <= HOVER_PICK_THRESHOLD {
                        check_ratio::show_check_tooltip(ui, app, id);
                    }
                }
            }
        }
    }

    // 変形の実効倍率（自動倍率 × 手動係数）の注記。
    // 実変位を表示している時のみ描く（モード形は固有ベクトルの規模が任意のため
    // 倍率に物理的な意味がなく、表示しない）。
    if deform_scale_actual > 0.0 && mode != ViewMode::Mode {
        // N/Q/M 図の凡例（min.y+10）・コンターバー＋ラベル（min.y+30〜56 程度）と
        // 重ならない位置へ
        let y = match mode {
            ViewMode::N | ViewMode::Q | ViewMode::M if app.diagram_contour => 70.0,
            ViewMode::N | ViewMode::Q | ViewMode::M => 30.0,
            _ => 10.0,
        };
        // 手動係数が 1.0 のときは「自動」、それ以外は「自動×係数」を併記する。
        let note = if (app.deform_scale_factor - 1.0).abs() < 1e-3 {
            format!("変形倍率 ×{:.0}（自動）", deform_scale_actual)
        } else {
            format!(
                "変形倍率 ×{:.0}（自動×{:.2}）",
                deform_scale_actual, app.deform_scale_factor
            )
        };
        painter.text(
            egui::pos2(
                painter.clip_rect().min.x + 10.0,
                painter.clip_rect().min.y + y,
            ),
            egui::Align2::LEFT_TOP,
            note,
            egui::FontId::proportional(12.0),
            theme::GRAY_600,
        );
    }

    // 選択ハイライト
    for &elem_id in &app.selection.members {
        if let Some(elem) = app.model.elements.iter().find(|e| e.id == elem_id) {
            if elem.nodes.len() >= 2 {
                let n0 = elem.nodes[0].index();
                let n1 = elem.nodes[1].index();
                if n0 < pts.len() && n1 < pts.len() {
                    painter.line_segment(
                        [pts[n0], pts[n1]],
                        egui::Stroke::new(4.0_f32, theme::PARETO_RED),
                    );
                }
            }
        }
    }

    // --- 剛床代表点（トグル ON 時）: 代表点マーカーと関連スレーブへの点線 ---
    // 剛床代表節点（重心マスター）は面内挙動を担う仮想節点で実部材に接続しない。
    // 関連付けられたスレーブ節点へ点線を引き、所属関係を可視化する。代表点・
    // スレーブとも変形後座標（`coords3` 由来の `pts`）で描くため変形へ追従する。
    // 点線は節点数が多いと他部材が見づらくなるため、トグルで表示を切り替える。
    if app.show_diaphragm_master {
        const DASH: f32 = 5.0;
        const GAP: f32 = 4.0;
        for c in &app.model.constraints {
            let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c else {
                continue;
            };
            let mi = master.index();
            if mi >= pts.len() {
                continue;
            }
            let mp = pts[mi];
            for sl in slaves {
                let si = sl.index();
                if si >= pts.len() {
                    continue;
                }
                painter.extend(egui::Shape::dashed_line(
                    &[mp, pts[si]],
                    egui::Stroke::new(1.0_f32, theme::translucent(theme::HILITE_PURPLE, 140)),
                    DASH,
                    GAP,
                ));
            }
            // 代表点マーカー（強調リング）。通常の青節点の上に紫リングを重ねる。
            painter.circle_stroke(mp, 6.0, egui::Stroke::new(2.0_f32, theme::HILITE_PURPLE));
        }
    }

    // --- 支持条件シンボル ---
    // 固定方向へ軸色の矢印、回転軸まわりに円弧を描く。
    // 部材・応力図の上に重ねて描き、支持方向を一目で判別できるようにする。
    // スクリーン上で矢印 18px・円弧半径 12px になるようワールド長を逆算する。
    //
    // 剛床（RigidDiaphragm）マスター節点は特別扱いする。マスターに設定される
    // 拘束（Uz/Rx/Ry）は零剛性自由度による特異行列を避けるための数値上の
    // ダミー拘束であり、剛床が物理的に拘束するのは面内自由度（Ux/Uy/Rz）。
    // そのため剛床マークはダミー拘束ではなく面内拘束（Ux/Uy/Rz）を表示する
    // （支点拘束との整合。従来はダミー拘束をそのまま描き、剛床が拘束しない
    // 自由度を表示していた）。
    const SUPPORT_ARROW_PX: f32 = 18.0;
    const SUPPORT_ARC_PX: f32 = 12.0;
    // 剛床マスター節点の index 集合。
    let diaphragm_masters: std::collections::HashSet<usize> = app
        .model
        .constraints
        .iter()
        .filter_map(|c| match c {
            squid_n_core::model::Constraint::RigidDiaphragm { master, .. } => Some(master.index()),
            _ => None,
        })
        .collect();
    // 剛床の面内拘束マスク（Ux, Uy, Rz）。
    let diaphragm_mask = {
        let mut m = Dof6Mask::FREE;
        m.set_fixed(Dof::Ux);
        m.set_fixed(Dof::Uy);
        m.set_fixed(Dof::Rz);
        m
    };
    let mut has_support = false;
    let mut has_diaphragm = false;
    for (i, node) in app.model.nodes.iter().enumerate() {
        let is_master = diaphragm_masters.contains(&i);
        // 剛床マスターの面内拘束マークは代表点トグル ON 時のみ描く（既定は非表示に
        // して他部材を見やすくする。点線・マーカーと表示を一致させる）。
        if is_master && !app.show_diaphragm_master {
            continue;
        }
        // 表示する拘束: 剛床マスターは面内拘束（Ux/Uy/Rz）、それ以外は節点拘束。
        let restraint = if is_master {
            diaphragm_mask
        } else {
            node.restraint
        };
        if support_kind(restraint) == SupportKind::Free {
            continue;
        }
        // 支点シンボルは変形後座標に描く。実支点は変位ゼロで原位置に留まり、
        // 剛床マスターは床の面内変形に追従する（剛床の重心マークが変形へ移動する）。
        let coord = coords3.get(i).copied().unwrap_or(node.coord);
        if is_master {
            has_diaphragm = true;
        } else {
            has_support = true;
        }
        draw_support_symbol(
            &painter,
            &proj,
            coord,
            restraint,
            SUPPORT_ARROW_PX,
            SUPPORT_ARC_PX,
        );
    }
    if has_support || has_diaphragm {
        draw_support_legend(&painter, has_diaphragm);
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

/// 変形図・モード形で梁の曲げ変形曲線を描く際の要素分割数（点数は +1）。
const DEFORM_CURVE_SEGMENTS: usize = 12;

/// 梁要素の Hermite 3 次変形曲線を評価するための前処理データ。
///
/// 端部 6 自由度（節点変位、無倍率）を要素ローカル系へ一度だけ変換して保持し、
/// 材軸パラメータ ξ∈[0,1] での変位・曲線上の点を安価に評価する。曲線描画・応力図
/// の基準線・床節点の追従・変形スケール上限で共有し、「梁の変形後の形」の評価を
/// 一箇所へ集約する（ループでの `LocalFrame` 再構築も避ける）。
///
/// 軸方向は線形内挿、曲げ 2 面は Hermite 3 次形状関数で内挿する（等価節点力
/// [`squid_n_element::member_load`] と同一の形状関数・符号規約。局所 z 面は θy の
/// 符号反転）。ξ=0,1 では回転項が消え端点は節点変位に一致する。本内挿は表示専用
/// であり解析結果（節点変位・内力）は変更しない。要素はせん断変形を含む
/// Timoshenko 梁だが、変形図は Euler–Bernoulli の Hermite 曲線で近似する
/// （変形形状の可視化として実務上標準的）。
struct BeamDeflection {
    /// 要素ローカル系（`rot` 行 = ex, ey, ez）。
    frame: squid_n_element::transform::LocalFrame,
    /// 部材長。
    l: f64,
    /// 未変形材軸の始点・終点（グローバル）。
    p_i: [f64; 3],
    p_j: [f64; 3],
    /// i 端のローカル端部変位 `[ux, uy, uz, ry, rz]`。
    ui: [f64; 5],
    /// j 端のローカル端部変位 `[ux, uy, uz, ry, rz]`。
    uj: [f64; 5],
}

impl BeamDeflection {
    /// 端部変位 `d_i`, `d_j`（節点変位 6 成分、無倍率）から前処理する。
    fn new(
        p_i: [f64; 3],
        p_j: [f64; 3],
        d_i: [f64; 6],
        d_j: [f64; 6],
        ref_vector: [f64; 3],
    ) -> Self {
        let l = member_len3(p_i, p_j);
        let frame = squid_n_element::transform::LocalFrame::from_nodes(p_i, p_j, ref_vector);
        let g = [
            d_i[0], d_i[1], d_i[2], d_i[3], d_i[4], d_i[5], d_j[0], d_j[1], d_j[2], d_j[3], d_j[4],
            d_j[5],
        ];
        let u = frame.rotate_to_local(&g);
        // 端部: 並進(ux,uy,uz)=u[0..3]/u[6..9]、曲げ回転(ry,rz)=u[4..6]/u[10..12]。
        Self {
            frame,
            l,
            p_i,
            p_j,
            ui: [u[0], u[1], u[2], u[4], u[5]],
            uj: [u[6], u[7], u[8], u[10], u[11]],
        }
    }

    /// 材軸パラメータ ξ での「未変形材軸上の点へ加えるグローバル並進変位」（無倍率）。
    /// 床・二次部材の節点を梁曲線へ載せる補間で用いる（描画曲線から浮かないよう
    /// 同じ評価を共有する）。
    fn disp_at(&self, xi: f64) -> [f64; 3] {
        let l = self.l;
        // Hermite 3 次形状関数（N2,N4 は L 倍を含む回転項）。
        let n1 = 1.0 - 3.0 * xi * xi + 2.0 * xi * xi * xi;
        let n2 = l * (xi - 2.0 * xi * xi + xi * xi * xi);
        let n3 = 3.0 * xi * xi - 2.0 * xi * xi * xi;
        let n4 = l * (-xi * xi + xi * xi * xi);
        let [uxi, uyi, uzi, ryi, rzi] = self.ui;
        let [uxj, uyj, uzj, ryj, rzj] = self.uj;
        // ローカル変位場: y 面は θz、z 面は θy（符号反転、member_load の msign=-1 と一致）。
        let ux = (1.0 - xi) * uxi + xi * uxj;
        let uy = n1 * uyi + n2 * rzi + n3 * uyj + n4 * rzj;
        let uz = n1 * uzi - n2 * ryi + n3 * uzj - n4 * ryj;
        // ローカル→グローバル（rot 行 = ex,ey,ez。global = ux·ex + uy·ey + uz·ez）。
        let r = &self.frame.rot;
        [
            r[0][0] * ux + r[1][0] * uy + r[2][0] * uz,
            r[0][1] * ux + r[1][1] * uy + r[2][1] * uz,
            r[0][2] * ux + r[1][2] * uy + r[2][2] * uz,
        ]
    }

    /// 変形後曲線上の点（未変形材軸上の点 + 倍率付き変位）を ξ で返す。
    /// ξ=0,1 では端点＝節点変位（`scale` 倍）に厳密一致する（節点マーカーと連続）。
    fn point_at(&self, xi: f64, scale: f64) -> [f64; 3] {
        let dg = self.disp_at(xi);
        [
            self.p_i[0] + (self.p_j[0] - self.p_i[0]) * xi + dg[0] * scale,
            self.p_i[1] + (self.p_j[1] - self.p_i[1]) * xi + dg[1] * scale,
            self.p_i[2] + (self.p_j[2] - self.p_i[2]) * xi + dg[2] * scale,
        ]
    }

    /// 変形後曲線を両端含む `segments + 1` 点の折れ線で返す（曲線描画用）。
    fn polyline(&self, scale: f64, segments: usize) -> Vec<[f64; 3]> {
        let seg = segments.max(1);
        (0..=seg)
            .map(|k| self.point_at(k as f64 / seg as f64, scale))
            .collect()
    }
}

/// CMQ 図の描画対象となる主架構の大梁か（`ElementKind::Beam` かつ、実部材化された
/// 小梁でない）。実部材化小梁は `slab.joists` の support 節点対に一致する Beam 要素
/// として判定する（`squid-n-load` の `beam_between` と同じ判定規則）。CMQ は全体解析
/// （主架構の応力）に関する図なので、二次部材（小梁・間柱）は対象外とする。
fn is_primary_beam_for_cmq(
    model: &squid_n_core::model::Model,
    elem: &squid_n_core::model::ElementData,
) -> bool {
    if elem.kind != squid_n_core::model::ElementKind::Beam || elem.nodes.len() != 2 {
        return false;
    }
    let (n0, n1) = (elem.nodes[0], elem.nodes[1]);
    let is_materialized_joist = model.slabs.iter().any(|slab| {
        slab.joists.iter().any(|j| {
            (j.support[0] == n0 && j.support[1] == n1) || (j.support[0] == n1 && j.support[1] == n0)
        })
    });
    !is_materialized_joist
}

/// 一つの主架構の大梁（`ElemId`）に載る全 `MemberLoadKind` を束ねたグループ。
struct CmqElemGroup {
    n0: usize,
    n1: usize,
    ref_vec: [f64; 3],
    /// C/M/Q 評価用。グループ内の全 `MemberLoad` の荷重種別（`MemberLoadKind`）。
    loads: Vec<squid_n_core::model::MemberLoadKind>,
}

/// `app.cmq_display_member_loads()`（主架構変換後の部材荷重）を要素（大梁）単位で
/// グループ化する。大梁の中間区間（小梁がとりつく位置）の荷重も同じ `ElemId` に
/// 変換されているため、大梁1本=1グループになる。小梁・柱・スラブには `MemberLoad`
/// が付かない（または実部材化小梁として `is_primary_beam_for_cmq` で除外される）ため
/// 自然に描画対象から外れる。描画順は初出順（`app.beam_loads` に現れた順）で安定する。
fn group_member_loads_by_elem(app: &App) -> Vec<CmqElemGroup> {
    let member_loads = app.cmq_display_member_loads();
    let mut order: Vec<squid_n_core::ids::ElemId> = Vec::new();
    let mut groups: std::collections::HashMap<squid_n_core::ids::ElemId, CmqElemGroup> =
        std::collections::HashMap::new();
    for ml in member_loads {
        let Some(elem) = app.model.elements.iter().find(|e| e.id == ml.elem) else {
            continue;
        };
        if !is_primary_beam_for_cmq(&app.model, elem) {
            continue;
        }
        let group = groups.entry(ml.elem).or_insert_with(|| {
            order.push(ml.elem);
            CmqElemGroup {
                n0: elem.nodes[0].index(),
                n1: elem.nodes[1].index(),
                ref_vec: elem.local_axis.ref_vector,
                loads: Vec::new(),
            }
        });
        group.loads.push(ml.kind);
    }
    order
        .into_iter()
        .filter_map(|id| groups.remove(&id))
        .collect()
}

/// グループ内の全荷重の両端固定端モーメントを合算する（C 図）。
fn sum_fixed_end_moments(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::fixed_end_moments(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ci, cj)| (ai + ci, aj + cj))
}

/// グループ内の全荷重の単純梁反力を合算する（Q 図）。
fn sum_simple_reactions(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::simple_reactions(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ri, rj)| (ai + ri, aj + rj))
}

/// M（単純梁中央モーメント）図の折れ線サンプリング位置 ξ∈[0,1] を返す。
/// 等分割に加え、`loads` に含まれる区間分布荷重の両端 a/L, b/L・集中荷重の a/L を
/// 折れ点として正確に出すため追加する。
fn cmq_m_sample_xis(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> Vec<f64> {
    use squid_n_core::model::MemberLoadKind;
    const N: usize = 32;
    let mut xis: Vec<f64> = (0..=N).map(|k| k as f64 / N as f64).collect();
    if l > 1e-9 {
        for load in loads {
            match *load {
                MemberLoadKind::Point { a, .. } => xis.push((a / l).clamp(0.0, 1.0)),
                MemberLoadKind::Distributed { a, b, .. } => {
                    xis.push((a / l).clamp(0.0, 1.0));
                    xis.push((b / l).clamp(0.0, 1.0));
                }
            }
        }
    }
    xis.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xis.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xis
}

/// ポリゴンを塗り（`convex_polygon`, `Stroke::NONE`）と輪郭（閉じない折れ線
/// `Shape::line`）に分けて描画する。塗り+輪郭を1シェイプにする従来方式（閉路）だと、
/// p0/p1 で軸線と曲線が浅い角度で接する折り返し点の epaint マイター結合が発散し、
/// 部材軸方向に画面外まで伸びるスパイク描画になるため、輪郭は閉じない折れ線にする。
fn paint_diagram_polygon(
    painter: &egui::Painter,
    points: Vec<egui::Pos2>,
    fill: egui::Color32,
    stroke_color: egui::Color32,
) {
    painter.add(egui::Shape::convex_polygon(
        points.clone(),
        fill,
        egui::Stroke::NONE,
    ));
    painter.add(egui::Shape::line(
        points,
        egui::Stroke::new(1.5_f32, stroke_color),
    ));
}

/// 部材ローカルに沿って CMQ 図（両端固定端モーメント C・単純梁中央モーメント M・
/// せん断 Q）を描く。
///
/// N/Q/M 図と同様、張り出し方向は要素ローカル y 軸（曲げ平面内）をワールド空間で
/// とってから投影する。CMQ は鉛直床荷重による強軸曲げのため、水平梁では鉛直面内の
/// 図となり、ビューを回転しても要素座標系に固定される。
///
/// 描画ソースは `app.beam_loads`（スラブ・小梁の生の荷重分配）ではなく、主架構へ
/// 変換後の部材荷重（[`group_member_loads_by_elem`]）。これにより大梁1本=1図形になり
/// （小梁がとりつく大梁で図が分裂しない）、小梁・スラブは自然に描画対象から外れる。
fn draw_cmq_diagram(painter: &egui::Painter, app: &App, coords3: &[[f64; 3]], proj: &Projector) {
    let scale = proj.scale();
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

    // 主架構へ変換後の部材荷重を要素（大梁）単位でグループ化し、座標が有効
    // （範囲内・非ゼロ長）なものだけを対象にする。
    let groups: Vec<CmqElemGroup> = group_member_loads_by_elem(app)
        .into_iter()
        .filter(|g| {
            g.n0 < coords3.len()
                && g.n1 < coords3.len()
                && member_len3(coords3[g.n0], coords3[g.n1]) >= 1e-9
        })
        .collect();

    let max_c = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            let (c_i, c_j) = sum_fixed_end_moments(&g.loads, l);
            c_i.abs().max(c_j.abs())
        })
        .fold(0.0_f64, f64::max);
    let max_q = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            let (q_i, q_j) = sum_simple_reactions(&g.loads, l);
            q_i.abs().max(q_j.abs())
        })
        .fold(0.0_f64, f64::max);
    // M（単純梁中央モーメント）の最大値: スパンをサンプリングして評価する。
    let max_m = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            cmq_m_sample_xis(&g.loads, l)
                .into_iter()
                .fold(0.0_f64, |acc, xi| {
                    acc.max(squid_n_load::floor::simple_beam_moment_at(&g.loads, l, xi * l).abs())
                })
        })
        .fold(0.0_f64, f64::max);
    if max_c < 1e-12 && max_q < 1e-12 && max_m < 1e-12 {
        return;
    }
    // 最大値で 60px 相当のワールド長（一様スケール正射影なので px/scale=ワールド長）
    let c_amp = 60.0 / max_c.max(1e-12) / scale as f64;
    let q_amp = 60.0 / max_q.max(1e-12) / scale as f64;
    let m_amp = 60.0 / max_m.max(1e-12) / scale as f64;

    // 張り出しピーク px が閾値未満の潰れた図形はスキップ（マイター発散対策。
    // N/Q/M 図と共有する `diagram::MIN_DIAGRAM_PX`）。
    for g in &groups {
        let p_i = coords3[g.n0];
        let p_j = coords3[g.n1];
        let l = member_len3(p_i, p_j);
        let ey = diagram_offset_dir(p_i, p_j, g.ref_vec);
        let p0 = proj.project(p_i);
        let p1 = proj.project(p_j);

        match app.cmq_component {
            CmqComponent::C => {
                let (c_i, c_j) = sum_fixed_end_moments(&g.loads, l);
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * c_i.abs().max(c_j.abs()) / max_c.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                // C 図（モーメント）: 両端の合算 c_i, c_j を結ぶ折れ線ポリゴン。M図の規約
                // （引張側に描く。sagging 正=-ey側=下、hogging 負=+ey側=上）に合わせ、
                // 固定端モーメント（hogging=引張は上端）は +ey 側=梁上側に描く。
                // c_i/c_j は固定端モーメントの符号規約上、両端で逆符号（i端+, j端-）で
                // 保持されているため、j 端は符号反転して i 端と同じ側（+ey 側）に描く。
                let c_poly = vec![
                    p0,
                    proj.project_offset(p_i, ey, c_i * c_amp),
                    proj.project_offset(p_j, ey, -c_j * c_amp),
                    p1,
                ];
                // C 図（モーメント）= 通常データ（青）
                paint_diagram_polygon(
                    painter,
                    c_poly,
                    theme::translucent(theme::DATA_BLUE, 60),
                    theme::DATA_BLUE,
                );
            }
            CmqComponent::M => {
                // M 図（単純梁としての中央モーメント）: スパンを分割サンプリングし、
                // グループ内の全荷重の simple_beam_moment_at を合算した値を、N/Q/M 図と
                // 同じ規約（正の sagging モーメントが梁下側=-ey 側）でプロットする。
                // 区間分布荷重の境界・集中荷重は折れ点 ξ=a/L, b/L を含める。
                let xis = cmq_m_sample_xis(&g.loads, l);
                // 先に値と対応するワールド位置を求め、ピーク px を判定してから描画する
                let mut val_max = 0.0_f64;
                let samples: Vec<(f64, [f64; 3])> = xis
                    .into_iter()
                    .map(|xi| {
                        let val = squid_n_load::floor::simple_beam_moment_at(&g.loads, l, xi * l);
                        val_max = val_max.max(val.abs());
                        let base3 = [
                            p_i[0] + (p_j[0] - p_i[0]) * xi,
                            p_i[1] + (p_j[1] - p_i[1]) * xi,
                            p_i[2] + (p_j[2] - p_i[2]) * xi,
                        ];
                        (val, base3)
                    })
                    .collect();
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * val_max / max_m.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                let mut m_poly = Vec::with_capacity(samples.len() + 2);
                m_poly.push(p0);
                // 直前の点とスクリーン距離が近すぎるサンプル点は重複点として除外する
                // （ゼロ長セグメントも epaint のマイター結合発散の原因になるため。
                // N/Q/M 図と共有する `diagram::MIN_SEGMENT_PX`）。p0/p1 は常に残す。
                let mut last = p0;
                for (val, base3) in samples {
                    let pt = proj.project_offset(base3, ey, -val * m_amp);
                    if (pt.x - last.x).hypot(pt.y - last.y) < diagram::MIN_SEGMENT_PX {
                        continue;
                    }
                    last = pt;
                    m_poly.push(pt);
                }
                m_poly.push(p1);
                // M 図（中央モーメント）= 強調紫。C（青）・Q（緑）と弁別する
                paint_diagram_polygon(
                    painter,
                    m_poly,
                    theme::translucent(theme::HILITE_PURPLE, 60),
                    theme::HILITE_PURPLE,
                );
            }
            CmqComponent::Q => {
                let (q_i, q_j) = sum_simple_reactions(&g.loads, l);
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * q_i.abs().max(q_j.abs()) / max_q.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                // Q 図（せん断）: 両端の合算 q_i, q_j を結ぶ折れ線ポリゴン（+ey 側に描画）
                let q_poly = vec![
                    p0,
                    proj.project_offset(p_i, ey, q_i * q_amp),
                    proj.project_offset(p_j, ey, q_j * q_amp),
                    p1,
                ];
                // Q 図（せん断）= 良好系（緑）。C（青）と弁別する
                paint_diagram_polygon(
                    painter,
                    q_poly,
                    theme::translucent(theme::GOOD_GREEN, 60),
                    theme::GOOD_GREEN,
                );
            }
        }
    }

    // 凡例（選択中の成分のみ表示）
    let legend = match app.cmq_component {
        CmqComponent::C => format!("CMQ図 C(max={:.2}) 青", max_c),
        CmqComponent::M => format!("CMQ図 M(max={:.2}) 紫", max_m),
        CmqComponent::Q => format!("CMQ図 Q(max={:.2}) 緑", max_q),
    };
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        legend,
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
fn draw_slabs_and_joists(painter: &egui::Painter, app: &App, pts: &[egui::Pos2]) {
    /// 破線パターン（描画長 / 間隔, px）
    const DASH: f32 = 6.0;
    const GAP: f32 = 4.0;

    for slab in &app.model.slabs {
        let poly: Vec<egui::Pos2> = slab
            .boundary
            .iter()
            .filter_map(|n| {
                let idx = n.index();
                (idx < pts.len()).then(|| pts[idx])
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
                &[pts[i0], pts[i1]],
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

/// スクリーン座標 `pos` に最も近い節点の `(index, 距離px)` を返す（同距離は先勝ち）。
/// ピッキング（節点選択・作成モード）で共有する。
fn pick_nearest_node(pts: &[egui::Pos2], pos: egui::Pos2) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for (i, &p) in pts.iter().enumerate() {
        let d = (pos - p).length();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((i, d));
        }
    }
    best
}

/// スクリーン座標 `pos` に最も近い部材（2 節点線分）の `(ElemId, 距離px)` を返す。
/// 2 節点未満の要素・節点参照が範囲外の要素は対象外。部材ピック・ホバーで共有する。
fn pick_nearest_member(
    model: &squid_n_core::model::Model,
    pts: &[egui::Pos2],
    pos: egui::Pos2,
) -> Option<(squid_n_core::ids::ElemId, f32)> {
    let mut best: Option<(squid_n_core::ids::ElemId, f32)> = None;
    for elem in &model.elements {
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let d = dist_point_to_segment(pos, pts[n0], pts[n1]);
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((elem.id, d));
        }
    }
    best
}

/// 主架構要素（`model.elements`）に接続しない節点の変位を、主架構の変形へ
/// 追従するよう補間で埋める。あくまで描画用の近似であり、解析結果そのものは
/// 変更しない。
///
/// スラブ境界・小梁支持点・二次部材の節点は解析自由度が割り当てられず
/// （`DofMap` は主架構要素が接続しない節点の全自由度を不活性にする）、変位が
/// 常にゼロのため、そのままでは変形図で床・二次部材だけが原位置に残る。
/// 補間は 2 段階で行う:
///
/// 1. **大梁への直付き（線上に載る）節点**: 最寄りの主架構 2 節点要素（線材）へ
///    射影し、垂線距離が許容値（モデル寸法の 0.1%）以内なら、その線分上の射影
///    位置 t で追従させる。梁（`Beam`）に載る場合、`use_beam_hermite` が真なら
///    梁の Hermite 変位（描画曲線と一致）で並進を追従させ（回転は線形補間）、
///    偽なら両端変位の線形補間とする（梁を直線で描く「全体変形」表示に合わせる）。
///    梁以外の線材は常に線形補間。ST-Bridge 取り込みモデルで二次部材の支持点が
///    大梁のスパン中間へ節点共有なしで載る典型ケースを追従する。
/// 2. **大梁に直付きしない二次部材節点**: 二次部材（小梁・間柱）の接続グラフを
///    辿り、最寄りの確定節点（1. のアンカー、または主架構節点）の変位へ剛体的に
///    追従させる（辺長を距離とする Dijkstra 的伝播）。最寄り線分への単純射影では
///    無関係な別の大梁へ張り付いて追従しない先端節点（片持ちの二次部材の先など）を
///    正しく取り付き先へ追従させる。
///
/// どちらでも確定しない孤立節点（大梁にも直付きせず、二次部材でも確定節点に
/// 到達しない床境界節点など）は、最寄り線分への射影変位でフォールバックする。
fn interpolate_unreferenced_disp(
    model: &squid_n_core::model::Model,
    mut disp: Vec<[f64; 6]>,
    use_beam_hermite: bool,
) -> Vec<[f64; 6]> {
    let n = model.nodes.len().min(disp.len());

    // 解析自由度を持ち変位が直接求まる節点（`DofMap::build` の structural 判定と
    // 同じ規則）: 主架構要素が接続する節点、または拘束（剛床・剛リンク・MPC）の
    // マスター節点。剛床代表節点（階自動生成が重心に置く仮想節点）は要素に
    // 接続しないが正しい解析変位を持つため、補間で上書きしてはいけない。
    let mut referenced = vec![false; n];
    for elem in &model.elements {
        for nd in &elem.nodes {
            if let Some(r) = referenced.get_mut(nd.index()) {
                *r = true;
            }
        }
    }
    for c in &model.constraints {
        use squid_n_core::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, .. } | Constraint::RigidLink { master, .. } => {
                if let Some(r) = referenced.get_mut(master.index()) {
                    *r = true;
                }
            }
            // MPC は `master` フィールドがスレーブ節点、`terms` がマスター側。
            Constraint::Mpc { terms, .. } => {
                for (nd, _, _) in terms {
                    if let Some(r) = referenced.get_mut(nd.index()) {
                        *r = true;
                    }
                }
            }
        }
    }
    if referenced.iter().all(|&r| r) {
        return disp;
    }

    // 補間ソースとなる主架構の線材（2 節点要素）。端点は必ず参照済み（正しい解析
    // 変位を持つ）ため、射影補間は他の未参照節点に依存しない。梁（`Beam`）は
    // 変形図で Hermite 3 次曲線として描画されるため、その線上に載る節点は端点変位
    // の線形補間ではなく梁の Hermite 変位で追従させる（描画曲線から浮かないよう
    // 端点回転を含めて評価する）。梁以外（ブレース等）は従来どおり線形補間とする
    // ため、要素種別と局所座標参照ベクトルを保持する。
    struct AnchorSeg {
        a: usize,
        b: usize,
        beam: bool,
        ref_vec: [f64; 3],
    }
    let segments: Vec<AnchorSeg> = model
        .elements
        .iter()
        .filter(|e| e.nodes.len() == 2)
        .map(|e| AnchorSeg {
            a: e.nodes[0].index(),
            b: e.nodes[1].index(),
            beam: e.kind == squid_n_core::model::ElementKind::Beam,
            ref_vec: e.local_axis.ref_vector,
        })
        .filter(|s| s.a < n && s.b < n)
        .collect();

    // 「大梁に直付き（線上に載る）」と判定する許容垂線距離。モデル寸法に対する
    // 相対値（バウンディングボックス対角長の 0.1%）。これより近い射影は主架構への
    // 直付きアンカーとして主架構変位を直接採用し、遠い節点は二次部材の接続を
    // 辿って追従させる。
    let anchor_tol = (model_bbox_size(model) * 1e-3).max(1e-9);

    // 段階 1: 各未参照節点を最寄り線分へ射影し、垂線距離が許容値以内なら主架構
    // 直付きアンカーとして確定する。射影変位は、伝播が届かなかった場合の
    // フォールバックとしても保持する。
    let mut finalized = referenced.clone();
    let mut proj_disp = vec![[0.0_f64; 6]; n];
    let mut proj_ok = vec![false; n];
    for i in 0..n {
        if referenced[i] {
            continue;
        }
        let p = model.nodes[i].coord;
        // 射影点までの距離が最小の線分を探す（射影パラメータ t は [0,1] にクランプ）。
        let mut best: Option<(f64, usize, f64)> = None; // (垂線距離², 線分 index, 射影 t)
        for (si, s) in segments.iter().enumerate() {
            let pa = model.nodes[s.a].coord;
            let pb = model.nodes[s.b].coord;
            let ab = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
            let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            let t = if len2 < 1e-12 {
                0.0
            } else {
                (((p[0] - pa[0]) * ab[0] + (p[1] - pa[1]) * ab[1] + (p[2] - pa[2]) * ab[2]) / len2)
                    .clamp(0.0, 1.0)
            };
            let q = [pa[0] + ab[0] * t, pa[1] + ab[1] * t, pa[2] + ab[2] * t];
            let d2 = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2);
            if best.is_none_or(|(bd, _, _)| d2 < bd) {
                best = Some((d2, si, t));
            }
        }
        if let Some((d2, si, t)) = best {
            let s = &segments[si];
            let (da, db) = (disp[s.a], disp[s.b]);
            // 梁で内部たわみ表示が有効なときのみ Hermite 変位で追従（並進 3 成分は
            // 描画曲線上へ載せ、回転は端点の線形補間で補う）。梁以外、または内部
            // たわみ表示 OFF（梁を直線で描く）のときは全 6 成分を線形補間する。
            let interp: [f64; 6] = if s.beam && use_beam_hermite {
                let hermite = BeamDeflection::new(
                    model.nodes[s.a].coord,
                    model.nodes[s.b].coord,
                    da,
                    db,
                    s.ref_vec,
                )
                .disp_at(t);
                std::array::from_fn(|k| match k {
                    0..=2 => hermite[k],
                    _ => da[k] * (1.0 - t) + db[k] * t,
                })
            } else {
                std::array::from_fn(|k| da[k] * (1.0 - t) + db[k] * t)
            };
            proj_disp[i] = interp;
            proj_ok[i] = true;
            if d2.sqrt() <= anchor_tol {
                disp[i] = interp;
                finalized[i] = true;
            }
        }
    }

    // 段階 2: 二次部材（小梁・間柱）の接続グラフを辿り、大梁に直付きしない節点を
    // 最寄りの確定節点の変位へ追従させる。
    let mut sec_adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for sm in &model.secondary_members {
        let a = sm.nodes[0].index();
        let b = sm.nodes[1].index();
        if a < n && b < n {
            sec_adj[a].push(b);
            sec_adj[b].push(a);
        }
    }
    let node_dist = |a: usize, b: usize| -> f64 {
        let pa = model.nodes[a].coord;
        let pb = model.nodes[b].coord;
        ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt()
    };

    // 追従元候補（確定節点から二次部材でつながる未確定節点への辺長と追従変位）。
    let mut best_dist = vec![f64::INFINITY; n];
    let mut src_disp = vec![[0.0_f64; 6]; n];
    let mut has_source = vec![false; n];
    // 確定節点（参照済み＋主架構直付きアンカー）から隣接未確定節点を緩和する。
    for u in 0..n {
        if !finalized[u] {
            continue;
        }
        for &j in &sec_adj[u] {
            if finalized[j] {
                continue;
            }
            let d = node_dist(u, j);
            if d < best_dist[j] {
                best_dist[j] = d;
                src_disp[j] = disp[u];
                has_source[j] = true;
            }
        }
    }
    // 最寄りの確定節点から順に確定させる（辺長を距離とする Dijkstra 的貪欲法）。
    // 二次部材の連鎖が長くても、主架構に最も近い側から変位が伝播する。
    loop {
        let mut pick: Option<(usize, f64)> = None;
        for i in 0..n {
            if finalized[i] || !has_source[i] {
                continue;
            }
            if pick.is_none_or(|(_, bd)| best_dist[i] < bd) {
                pick = Some((i, best_dist[i]));
            }
        }
        let Some((u, _)) = pick else { break };
        disp[u] = src_disp[u];
        finalized[u] = true;
        // u を追従元として、二次部材でつながる未確定の隣接節点を緩和する。
        for &j in &sec_adj[u] {
            if finalized[j] {
                continue;
            }
            let d = node_dist(u, j);
            if d < best_dist[j] {
                best_dist[j] = d;
                src_disp[j] = disp[u];
                has_source[j] = true;
            }
        }
    }

    // フォールバック: まだ確定しない節点（大梁にも直付きせず、二次部材でも確定
    // 節点に到達しない孤立した床境界節点など）は、最寄り線分への射影変位を採る。
    for i in 0..n {
        if !finalized[i] && proj_ok[i] {
            disp[i] = proj_disp[i];
        }
    }
    disp
}

/// 剛床代表節点（マスター）の鉛直変位（Uz）を、スレーブ節点の鉛直変位の平均で
/// 表示用に補う。あくまで描画専用の近似で、解析結果（`StaticOnce::disp`）は変更
/// しない。
///
/// マスターの面内自由度（Ux・Uy・Rz）は解析結果をそのまま使うため水平変形には
/// 追従するが、面外自由度（Uz・Rx・Ry）は零剛性による特異行列を避けるための数値
/// ダミー拘束で 0 に固定されている（`squid-n-load` の `story_gen`）。そのままだと
/// 変形図で代表点だけが原標高に浮き、床の鉛直変形（重力たわみ・地震の転倒による
/// 床の上下動）へ追従しない。スレーブ節点の Uz 平均を代表点の Uz とすることで、
/// 代表点を変形後の床の平均標高へ載せる。
fn fill_diaphragm_master_disp_for_display(
    model: &squid_n_core::model::Model,
    mut disp: Vec<[f64; 6]>,
) -> Vec<[f64; 6]> {
    let n = model.nodes.len().min(disp.len());
    for c in &model.constraints {
        let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c else {
            continue;
        };
        let mi = master.index();
        if mi >= n {
            continue;
        }
        let mut sum = 0.0_f64;
        let mut cnt = 0.0_f64;
        for sl in slaves {
            let si = sl.index();
            if si < n {
                sum += disp[si][2];
                cnt += 1.0;
            }
        }
        if cnt >= 0.5 {
            disp[mi][2] = sum / cnt;
        }
    }
    disp
}

/// 解析変位を表示用に加工する（いずれも描画専用の近似で、解析結果は変更しない）。
///
/// 1. 主架構に接続しない床・二次部材の節点を主架構の変形へ追従させる
///    （[`interpolate_unreferenced_disp`]。梁に載る節点は内部たわみ表示 ON なら
///    梁の Hermite 曲線上へ、OFF なら弦上へ）。
/// 2. 剛床代表節点の鉛直変位をスレーブ平均で補い、代表点を床の変形へ追従させる
///    （[`fill_diaphragm_master_disp_for_display`]）。
fn display_disp(
    model: &squid_n_core::model::Model,
    raw: Vec<[f64; 6]>,
    use_beam_hermite: bool,
) -> Vec<[f64; 6]> {
    let d = interpolate_unreferenced_disp(model, raw, use_beam_hermite);
    fill_diaphragm_master_disp_for_display(model, d)
}

/// 変形図の実効表示倍率（自動倍率 × 手動係数）を算定する。変位が無い（`None`）・
/// 全並進成分がゼロなら 0 を返す（変形を描かない）。
///
/// 自動倍率は次の小さい方:
/// - **バウンディングボックス基準**: 最大並進変位がモデル対角長の 10% で表示される
///   倍率 `0.1 · model_size / δ_max`。
/// - **梁スパン基準**（`use_beam_interpolation` が真のときのみ）: 各梁の Hermite 内部
///   たわみがスパンの一定割合を超えない上限（[`beam_deflection_scale_limit`]）。
///   内部たわみ OFF（梁を直線で描く）ではふくらみが生じないため併用しない。
///
/// これに手動係数 `factor`（スライダー）を掛けた値を実効倍率とする。
fn deform_display_scale(
    model: &squid_n_core::model::Model,
    disp: Option<&[[f64; 6]]>,
    model_size: f64,
    use_beam_interpolation: bool,
    factor: f32,
) -> f64 {
    let Some(d) = disp else {
        return 0.0;
    };
    let max_disp = d
        .iter()
        .map(|v| v[0].abs().max(v[1].abs()).max(v[2].abs()))
        .fold(0.0_f64, f64::max);
    if max_disp <= 1e-12 {
        return 0.0;
    }
    let bbox_scale = model_size * 0.1 / max_disp;
    let auto = if use_beam_interpolation {
        beam_deflection_scale_limit(model, d).map_or(bbox_scale, |lim| bbox_scale.min(lim))
    } else {
        bbox_scale
    };
    auto * factor as f64
}

/// 梁のスパンに対する内部たわみが過大にならないよう、表示倍率の上限を算定する。
/// 制約する梁が無ければ `None`。
///
/// 変形図の梁は端部 6 自由度からの Hermite 3 次曲線で描くため、端部回転が大きいと
/// 中央のふくらみ（変形後両端を結ぶ弦からの逸脱）がスパンに対して過大になり得る。
/// 各梁について無倍率のたわみ（弦からの最大逸脱）を評価し、
/// `倍率 × たわみ ≤ FRAC × スパン` を満たす倍率上限 `FRAC × スパン / たわみ` の
/// 最小値を返す。バウンディングボックス基準の倍率と併せて小さい方を採ることで、
/// 全体変形も梁のふくらみも過大にならないスケールにする。
fn beam_deflection_scale_limit(
    model: &squid_n_core::model::Model,
    disp: &[[f64; 6]],
) -> Option<f64> {
    /// 梁の内部たわみ（弦からの逸脱）が許容されるスパン比。
    const BEAM_DEFLECTION_DISPLAY_FRAC: f64 = 0.1;
    /// たわみ評価の内部サンプル点数（両端を除く分割）。
    const SAMPLES: usize = 9;

    let n = model.nodes.len().min(disp.len());
    let mut limit: Option<f64> = None;
    for elem in &model.elements {
        if elem.kind != squid_n_core::model::ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let a = elem.nodes[0].index();
        let b = elem.nodes[1].index();
        if a >= n || b >= n {
            continue;
        }
        let p_i = model.nodes[a].coord;
        let p_j = model.nodes[b].coord;
        let l = member_len3(p_i, p_j);
        if l < 1e-9 {
            continue;
        }
        let (d_i, d_j) = (disp[a], disp[b]);
        // 無倍率での弦からの最大逸脱（弦＝端部並進の線形補間、曲線＝Hermite 変位）。
        // 端部 DOF のローカル化は ξ に依らないため、梁ごとに 1 回だけ前処理する。
        let bd = BeamDeflection::new(p_i, p_j, d_i, d_j, elem.local_axis.ref_vector);
        let mut max_dev = 0.0_f64;
        for k in 1..SAMPLES {
            let xi = k as f64 / SAMPLES as f64;
            let h = bd.disp_at(xi);
            let dev = ((h[0] - (d_i[0] * (1.0 - xi) + d_j[0] * xi)).powi(2)
                + (h[1] - (d_i[1] * (1.0 - xi) + d_j[1] * xi)).powi(2)
                + (h[2] - (d_i[2] * (1.0 - xi) + d_j[2] * xi)).powi(2))
            .sqrt();
            max_dev = max_dev.max(dev);
        }
        if max_dev > 1e-12 {
            let lim = BEAM_DEFLECTION_DISPLAY_FRAC * l / max_dev;
            limit = Some(limit.map_or(lim, |cur: f64| cur.min(lim)));
        }
    }
    limit
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
fn draw_grid_and_axes(painter: &egui::Painter, rect: egui::Rect, projector: &Projector) {
    let center3 = projector.center3();
    let scale = projector.scale();
    let proj = |p: [f64; 3]| projector.project(p);

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

    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
        SecondaryMember, SecondaryMemberKind,
    };

    /// 補間テスト用の節点を作る（拘束なし・付加情報なし）。
    fn test_node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }
    }

    /// 補間テスト用の二次部材（小梁）を作る。
    fn test_secondary(i: u32, j: u32) -> SecondaryMember {
        SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(i), NodeId(j)],
            section: None,
            material: None,
            name: String::new(),
        }
    }

    /// 補間テスト用の 2 節点梁要素を作る。
    fn test_beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    #[test]
    fn 主架構に接続する節点の変位は補間で変更されない() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
            [4.0, 5.0, 6.0, 0.4, 0.5, 0.6],
        ];
        let out = interpolate_unreferenced_disp(&model, disp.clone(), true);
        assert_eq!(out, disp);
    }

    #[test]
    fn 大梁スパン中間の未参照節点は梁のエルミート変位で追従する() {
        // 大梁 n0-n1 のスパン 1/4 点に、節点共有なしで載る小梁支持点 n2
        // （ST-Bridge 取り込みモデルの典型）を置く。梁は変形図で Hermite 曲線として
        // 描かれるため、直付き節点は端点の線形補間ではなく Hermite 変位で追従する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [4.0, 8.0, -12.0, 0.0, 0.0, 0.0],
            [0.0; 6], // 未参照節点は解析結果ではゼロ
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // t = 2000/8000 = 0.25。端部回転は 0 のため、軸方向（+X）は線形（0.25·4=1.0）、
        // 材軸直交成分（Y,Z）は Hermite の N3(0.25)=0.15625 倍で追従する
        // （線形補間の 0.25 倍より小さく、梁の描画曲線上に載る）。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "X={}", out[2][0]);
        assert!((out[2][1] - 8.0 * 0.15625).abs() < 1e-12, "Y={}", out[2][1]);
        assert!(
            (out[2][2] + 12.0 * 0.15625).abs() < 1e-12,
            "Z={}",
            out[2][2]
        );
    }

    #[test]
    fn 大梁スパン中間の未参照節点は内部たわみオフで線形補間になる() {
        // 内部たわみ表示 OFF（梁を直線で描く「全体変形」表示）では、直付き節点も
        // 端点の線形補間で追従する（梁の直線＝弦の上に載る）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [4.0, 8.0, -12.0, 0.0, 0.0, 0.0],
            [0.0; 6],
        ];
        let out = interpolate_unreferenced_disp(&model, disp, false);
        // t = 0.25 の線形補間（全成分）。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "X={}", out[2][0]);
        assert!((out[2][1] - 2.0).abs() < 1e-12, "Y={}", out[2][1]);
        assert!((out[2][2] + 3.0).abs() < 1e-12, "Z={}", out[2][2]);
    }

    #[test]
    fn 梁上の未参照節点は梁の描画曲線に厳密一致する() {
        // 端部に回転を与えた梁のスパン中央に未参照節点を置く。その補間変位が、同じ
        // 端部変位で BeamDeflection::polyline を描いた曲線の同一パラメータ位置の変位に
        // 厳密一致すること（床・二次部材の節点が梁の描画たわみ曲線から浮かない）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [3000.0, 0.0, 0.0])); // スパン中央（t=0.5）
        model.elements.push(test_beam(0, 0, 1)); // ref_vector=[0,0,1]

        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let disp = vec![d_i, d_j, [0.0; 6]];
        let out = interpolate_unreferenced_disp(&model, disp, true);

        // 同じ端部変位で梁曲線を無倍率描画し、中央点（12 分割の index 6=ξ0.5）の
        // 変位（曲線点 − 未変形材軸点）を取る。
        let poly = BeamDeflection::new(
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            d_i,
            d_j,
            [0.0, 0.0, 1.0],
        )
        .polyline(1.0, 12);
        let curve_disp = [poly[6][0] - 3000.0, poly[6][1] - 0.0, poly[6][2] - 0.0];
        for k in 0..3 {
            assert!(
                (out[2][k] - curve_disp[k]).abs() < 1e-9,
                "軸 {k}: 補間 {} と曲線 {} が不一致",
                out[2][k],
                curve_disp[k]
            );
        }
        // 端部回転で中央がたわむため、直線（線形補間＝0）とは異なる。
        assert!(
            out[2][1].abs() > 1.0,
            "Hermite たわみが出ていない: {}",
            out[2][1]
        );
    }

    #[test]
    fn 梁軸から外れた未参照節点も最寄り線分の射影位置で補間される() {
        // 大梁からオフセットした位置の節点（床境界の幾何節点など）は、
        // 最寄り線分への射影点（クランプ込み）の変位で追従する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [4000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 500.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![[0.0; 6], [10.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // 射影点は t=0.5 → 5.0
        assert!((out[2][0] - 5.0).abs() < 1e-12);
    }

    #[test]
    fn 主架構の線材が無ければ未参照節点の変位はゼロのまま() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [1000.0, 0.0, 0.0]));
        // 要素なし → 補間ソースがなく、変位はゼロのまま
        let out = interpolate_unreferenced_disp(&model, vec![[0.0; 6]; 2], true);
        assert!(out.iter().all(|v| v.iter().all(|&x| x == 0.0)));
    }

    #[test]
    fn 剛床マスター節点の変位は補間で上書きされない() {
        // 剛床代表節点（階自動生成が重心に置く仮想節点）は要素に接続しないが、
        // 拘束のマスターとして解析自由度を持ち正しい変位が求まる
        // （`DofMap::build` の structural 判定と同じ規則）。補間対象にしてはいけない。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [4000.0, 0.0, 0.0])); // 剛床マスター
        model.elements.push(test_beam(0, 0, 1));
        model
            .constraints
            .push(squid_n_core::model::Constraint::RigidDiaphragm {
                story: squid_n_core::ids::StoryId(0),
                master: NodeId(2),
                slaves: vec![NodeId(0), NodeId(1)],
            });

        let disp = vec![
            [0.0; 6],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [7.0, 0.0, 0.0, 0.0, 0.0, 0.0], // マスターの解析変位（補間値 5.0 とは異なる）
        ];
        let out = interpolate_unreferenced_disp(&model, disp.clone(), true);
        assert_eq!(out, disp);
    }

    #[test]
    fn 変形曲線の端部は節点変位に一致する() {
        // 水平梁（i→j が +X）。両端に異なる並進・回転を与え、ξ=0,1 が
        // 節点変位（scale 倍）に厳密一致することを確認する。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0, 1.0, 0.0, 0.0, 0.0, 0.001];
        let d_j = [2.0, 3.0, 0.0, 0.0, 0.0, -0.002];
        let scale = 2.0;
        let poly = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 0.0, 1.0]).polyline(scale, 12);
        assert_eq!(poly.len(), 13);
        // i 端 = p_i + scale·d_i(並進)
        for k in 0..3 {
            assert!(
                (poly[0][k] - (p_i[k] + scale * d_i[k])).abs() < 1e-6,
                "i端 axis{k}: {}",
                poly[0][k]
            );
            assert!(
                (poly[12][k] - (p_j[k] + scale * d_j[k])).abs() < 1e-6,
                "j端 axis{k}: {}",
                poly[12][k]
            );
        }
    }

    #[test]
    fn 端部回転で中央がたわむ() {
        // 水平梁（i→j が +X）、ref=+Y とすると局所系は全体系と一致
        // （ex=+X, ey=+Y, ez=+Z）。両端の並進を 0、i 端に正・j 端に負の
        // θz（全体=局所 z 軸まわり）を与えると、Hermite 内挿で局所 y(=+Y)へ
        // 中央がふくらむ。直線（節点間）内挿なら中央は原位置のまま（たわみ 0）。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let poly = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 1.0, 0.0]).polyline(1.0, 12);
        let mid = poly[6];
        // 中央の材軸位置は x=500、たわみは局所 y=+Y 方向へ非ゼロ
        assert!((mid[0] - 500.0).abs() < 1e-6, "中央 x={}", mid[0]);
        assert!(
            mid[1].abs() > 1.0,
            "中央のたわみが小さすぎる: dy={}",
            mid[1]
        );
        // 端部は原位置（並進 0・回転のみ）
        assert!(poly[0][1].abs() < 1e-9 && poly[12][1].abs() < 1e-9);
    }

    #[test]
    fn 梁変形後曲線の端点は節点変位に一致し中央は弦から外れる() {
        // 応力図の基準線に使う BeamDeflection::point_at の検証。端点（ξ=0,1）は
        // 節点変位（scale 倍）に一致し、中央（ξ=0.5）は端部回転により弦（端点の
        // 線形補間）から外れてたわむ。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [6000.0, 0.0, 0.0];
        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let scale = 2.0;
        let bd = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 0.0, 1.0]);
        let a = bd.point_at(0.0, scale);
        let b = bd.point_at(1.0, scale);
        for k in 0..3 {
            assert!(
                (a[k] - (p_i[k] + scale * d_i[k])).abs() < 1e-6,
                "端点i k={k}"
            );
            assert!(
                (b[k] - (p_j[k] + scale * d_j[k])).abs() < 1e-6,
                "端点j k={k}"
            );
        }
        let mid = bd.point_at(0.5, scale);
        let chord_mid = [
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        ];
        let dev = ((mid[0] - chord_mid[0]).powi(2)
            + (mid[1] - chord_mid[1]).powi(2)
            + (mid[2] - chord_mid[2]).powi(2))
        .sqrt();
        assert!(dev > 1.0, "中央が弦から外れていない: dev={}", dev);
    }

    #[test]
    fn 大梁に直付きしない二次部材の先端は接続先を辿って追従する() {
        // 大梁 G1(0-1) は節点 1 に大きな水平変位を持つ。node 2 は G1 のスパン上
        // （直付きアンカー）。node 3 は G1 から離れた先端で、二次部材 2-3 で node 2 に
        // つながる。もう 1 本の大梁 G2(4-5)（変位ゼロ）を node 3 の近くに置き、
        // 「最寄り線分へ射影」だけでは node 3 が G2 へ張り付いて追従しないところを、
        // 二次部材経由の追従で取り付き先（node 2）へ揃うことを確認する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0])); // G1 上（直付き）
        model.nodes.push(test_node(3, [2000.0, 4000.0, 0.0])); // 先端（G1 から 4000, G2 から 1000）
        model.nodes.push(test_node(4, [0.0, 5000.0, 0.0])); // G2 端
        model.nodes.push(test_node(5, [8000.0, 5000.0, 0.0])); // G2 端
        model.elements.push(test_beam(0, 0, 1)); // G1
        model.elements.push(test_beam(1, 4, 5)); // G2
        model.secondary_members.push(test_secondary(2, 3)); // 二次部材 2-3

        // G1 は大きく水平移動、G2 は変位ゼロ。
        let disp = vec![
            [0.0; 6],                         // 0
            [100.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 1
            [0.0; 6],                         // 2（未参照）
            [0.0; 6],                         // 3（未参照）
            [0.0; 6],                         // 4
            [0.0; 6],                         // 5
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // node 2 は G1 上 t=0.25 → 25.0
        assert!((out[2][0] - 25.0).abs() < 1e-9, "node2={:?}", out[2]);
        // node 3 は最寄り大梁 G2（変位 0）ではなく、二次部材で node 2 に追従 → 25.0
        assert!((out[3][0] - 25.0).abs() < 1e-9, "node3={:?}", out[3]);
    }

    #[test]
    fn 二次部材の連鎖でも主架構に近い側から順に追従する() {
        // node 1(大梁 G1 上, 直付き) → 二次部材 → node 2 → 二次部材 → node 3 の連鎖。
        // node 3 は変位ゼロの別の大梁 G2 に近く、単純射影では G2 へ張り付くが、
        // 連鎖を辿って node 1 の変位へ揃うことを確認する（伝播が無いと誤る配置）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(1, [4000.0, 0.0, 0.0])); // G1 端（直付きアンカー元）
        model.nodes.push(test_node(2, [4000.0, 2000.0, 0.0])); // 連鎖 1 段目
        model.nodes.push(test_node(3, [4000.0, 4000.0, 0.0])); // 連鎖 2 段目（G2 から 1000）
        model.nodes.push(test_node(4, [0.0, 5000.0, 0.0])); // G2 端
        model.nodes.push(test_node(5, [8000.0, 5000.0, 0.0])); // G2 端
        model.elements.push(test_beam(0, 0, 1)); // G1
        model.elements.push(test_beam(1, 4, 5)); // G2（変位ゼロ）
        model.secondary_members.push(test_secondary(1, 2));
        model.secondary_members.push(test_secondary(2, 3));

        let disp = vec![
            [8.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 0
            [8.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 1（両端同変位＝剛体移動）
            [0.0; 6],                       // 2（未参照）
            [0.0; 6],                       // 3（未参照）
            [0.0; 6],                       // 4
            [0.0; 6],                       // 5
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // node 2, 3 とも連鎖を辿って node 1 の変位 8.0 に追従する。
        assert!((out[2][0] - 8.0).abs() < 1e-9, "node2={:?}", out[2]);
        assert!((out[3][0] - 8.0).abs() < 1e-9, "node3={:?}", out[3]);
    }

    #[test]
    fn 剛床マスターの鉛直変位はスレーブ平均で補完される() {
        // マスター（重心）はダミー拘束で Uz=0。スレーブの Uz 平均で表示用に補完し、
        // 面内（Ux/Uy/Rz）は解析結果のまま維持されることを確認する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 3000.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 3000.0]));
        model.nodes.push(test_node(2, [3000.0, 0.0, 3000.0])); // マスター（重心）
        model
            .constraints
            .push(squid_n_core::model::Constraint::RigidDiaphragm {
                story: squid_n_core::ids::StoryId(0),
                master: NodeId(2),
                slaves: vec![NodeId(0), NodeId(1)],
            });
        let disp = vec![
            [1.0, 0.0, -4.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, -6.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.02], // マスターの面内変位（Uz は 0）
        ];
        let out = fill_diaphragm_master_disp_for_display(&model, disp);
        // Uz は (-4 + -6)/2 = -5 に補完される。
        assert!((out[2][2] + 5.0).abs() < 1e-12, "Uz={}", out[2][2]);
        // 面内（Ux, Rz）は変更されない。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "Ux={}", out[2][0]);
        assert!((out[2][5] - 0.02).abs() < 1e-12, "Rz={}", out[2][5]);
    }

    #[test]
    fn 梁の内部たわみで変形スケール上限が算定される() {
        // 端部に等・逆回転（θz=±0.01）を与えた L=6000 の梁。両端並進 0 のため弦は
        // 直線で、弦からの逸脱＝Hermite たわみ w(ξ)=0.01·L·ξ(1−ξ)。9 分割の内部
        // サンプルでの最大は ξ=4/9,5/9 の 0.01·6000·(20/81)。
        // 上限 = 0.1·L / w_max = 0.1·6000·81 / (0.01·6000·20) = 40.5。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            [0.0, 0.0, 0.0, 0.0, 0.0, -0.01],
        ];
        let limit = beam_deflection_scale_limit(&model, &disp).expect("上限が算定される");
        assert!((limit - 40.5).abs() < 1e-9, "limit={}", limit);
    }

    #[test]
    fn 変位ゼロなら梁スケール上限は無し() {
        // たわみが生じない（全変位ゼロ）と制約する梁が無く None を返す。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        let disp = vec![[0.0; 6], [0.0; 6]];
        assert!(beam_deflection_scale_limit(&model, &disp).is_none());
    }

    #[test]
    fn 表示倍率は変位なし又は全ゼロでゼロ() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [10000.0, 0.0, 0.0]));
        let size = model_bbox_size(&model);
        assert_eq!(deform_display_scale(&model, None, size, true, 1.0), 0.0);
        let zero = vec![[0.0; 6], [0.0; 6]];
        assert_eq!(
            deform_display_scale(&model, Some(&zero), size, true, 1.0),
            0.0
        );
    }

    #[test]
    fn 内部たわみオフの表示倍率はbox基準に手動係数を掛ける() {
        // 梁要素が無く（＝梁スパン基準は無関係）、box 基準 × 手動係数になる。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [10000.0, 0.0, 0.0]));
        let disp = vec![[0.0; 6], [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        let size = model_bbox_size(&model); // 対角 10000
                                            // box 基準 = 0.1·10000 / 100 = 10、手動係数 2 → 20。
        let s = deform_display_scale(&model, Some(&disp), size, false, 2.0);
        assert!((s - 20.0).abs() < 1e-9, "s={}", s);
    }

    #[test]
    fn 内部たわみオンは梁スパン上限で倍率が制限される() {
        // box 基準が梁スパン上限より大きい配置。ON では min(box, 梁スパン) になる。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        // 端部回転で内部たわみを生み、並進は微小にして box 基準を大きくする。
        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            [0.001, 0.0, 0.0, 0.0, 0.0, -0.01],
        ];
        let size = model_bbox_size(&model); // 6000
        let on = deform_display_scale(&model, Some(&disp), size, true, 1.0);
        let off = deform_display_scale(&model, Some(&disp), size, false, 1.0);
        // OFF は box 基準のみ、ON は梁スパン上限（前掲テストの 40.5）も併用。
        assert!(on < off, "on={on} off={off}");
        assert!((on - 40.5).abs() < 1e-6, "on={on}");
    }
}
