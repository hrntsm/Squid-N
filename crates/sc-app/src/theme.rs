//! TONMANUAL（トンマナガイド）に基づく配色・テーマの単一情報源。
//!
//! 色値は本書 §2（カラーパレット）／§3（データビジュアライゼーション）／§3-2（3D ビュー）
//! の値をそのまま定数化したもの。UI 各所はこの定数を参照し、独自色を散らさない。
//! テーマ全体（ライト基準・ブルークローム・角丸）は [`apply_theme`] で egui に適用する。

use egui::{Color32, CornerRadius, FontFamily, FontId, Stroke, TextStyle};

// ===== §2 プライマリ（ブランドブルー） =====
/// ナビゲーション／ツールバー背景
pub const BLUE_200: Color32 = Color32::from_rgb(0xBF, 0xDB, 0xFE);
/// 選択ハイライト、ボタンホバー、ヘッダー
pub const BLUE_300: Color32 = Color32::from_rgb(0x93, 0xC5, 0xFD);
/// アナウンスバー背景
pub const BLUE_400: Color32 = Color32::from_rgb(0x60, 0xA5, 0xFA);
/// メインアクセント。アクティブボタン・枠線・インジケーター
pub const BLUE_500: Color32 = Color32::from_rgb(0x3B, 0x82, 0xF6);
/// アクセントのホバー濃色
pub const BLUE_600: Color32 = Color32::from_rgb(0x25, 0x63, 0xEB);

// ===== §2 セカンダリ（グレースケール） =====
/// 見出し・主要テキスト
pub const GRAY_900: Color32 = Color32::from_rgb(0x11, 0x18, 0x27);
/// ナビゲーションテキスト・サブテキスト
pub const GRAY_700: Color32 = Color32::from_rgb(0x37, 0x41, 0x51);
/// 本文テキスト
pub const GRAY_600: Color32 = Color32::from_rgb(0x4B, 0x55, 0x63);
/// キャンバスのドットグリッド
pub const GRAY_300: Color32 = Color32::from_rgb(0xD1, 0xD5, 0xDB);
/// ボーダー・入力欄枠線・ホバー背景・テーブルストライプ
pub const GRAY_200: Color32 = Color32::from_rgb(0xE5, 0xE7, 0xEB);
/// パネル背景・ウィジェット背景・入力欄背景
pub const GRAY_100: Color32 = Color32::from_rgb(0xF3, 0xF4, 0xF6);
/// メインキャンバス・チャートセル背景
pub const WHITE: Color32 = Color32::WHITE;

// ===== §2 アクション／セマンティック =====
/// 重要操作（実行・確定）アクション
pub const GREEN_500: Color32 = Color32::from_rgb(0x22, 0xC5, 0x5E);
/// アクションのホバー
pub const GREEN_600: Color32 = Color32::from_rgb(0x16, 0xA3, 0x4A);
/// エラー表示（ブランド対象外の固定色）
pub const ERROR_RED: Color32 = Color32::from_rgb(0xEA, 0x43, 0x35);

// ===== §3 データビジュアライゼーション配色 =====
/// データ点・通常 trial（デフォルトの試行点・バー・線・リンク）
pub const DATA_BLUE: Color32 = Color32::from_rgb(0x42, 0x85, 0xF4);
/// パレート解・重要・負方向（高スコア・負のバー）
pub const PARETO_RED: Color32 = Color32::from_rgb(0xEA, 0x43, 0x35);
/// 強調・ベスト・中間
pub const BEST_YELLOW: Color32 = Color32::from_rgb(0xFB, 0xBC, 0x04);
/// 良好・収束・実行中
pub const GOOD_GREEN: Color32 = Color32::from_rgb(0x34, 0xA8, 0x53);
/// ハイライト試行点（選択中の試行点・等高線）
pub const HILITE_PURPLE: Color32 = Color32::from_rgb(0x7C, 0x4D, 0xFF);

// ===== §3-2 3D ビュー =====
/// 3D 背景（2D の白とは異なり淡いグレー。立体感のため意図的に白を避ける）
pub const VIEW_BG: Color32 = Color32::from_rgb(0xF0, 0xF2, 0xF5);
/// X 軸（赤系）
pub const AXIS_X: Color32 = Color32::from_rgb(0xD2, 0x64, 0x64);
/// Y 軸（緑系）
pub const AXIS_Y: Color32 = Color32::from_rgb(0x50, 0xAA, 0x50);
/// Z 軸（青系）
pub const AXIS_Z: Color32 = Color32::from_rgb(0x64, 0x64, 0xC8);

/// パレット色を指定アルファで半透明化する（面要素・応力図の塗りつぶし用）。
pub fn translucent(c: Color32, alpha: u8) -> Color32 {
    Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), alpha)
}

/// 色を白側へ `t`（0.0–1.0）だけ寄せて淡くする（§3-2 軸ラベル負方向端の淡色など）。
pub fn lighten(c: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let mix = |ch: u8| (ch as f32 + (255.0 - ch as f32) * t).round() as u8;
    Color32::from_rgb(mix(c.r()), mix(c.g()), mix(c.b()))
}

/// 検定比などの「状態」を §3 のセマンティック 3 色へ写像する。
/// 良好(≤0.8)=緑／注意(≤1.0)=黄／超過(>1.0)=赤。
pub fn status_color(ratio: f64) -> Color32 {
    if ratio <= 0.8 {
        GOOD_GREEN
    } else if ratio <= 1.0 {
        BEST_YELLOW
    } else {
        PARETO_RED
    }
}

/// TONMANUAL に沿ったテーマ（ライト基準・ブルークローム・角丸 4/6px・タイポスケール）を
/// egui コンテキストへ適用する。アプリ起動時に一度だけ呼ぶ。
pub fn apply_theme(ctx: &egui::Context) {
    let mut style = (*ctx.global_style()).clone();
    let mut v = egui::Visuals::light();

    // 背景の階層（§2）: パネル＝gray-100、入力欄＝gray-100、ストライプ＝gray-200
    v.panel_fill = GRAY_100;
    v.window_fill = GRAY_100;
    v.extreme_bg_color = GRAY_100;
    v.faint_bg_color = GRAY_200;
    v.hyperlink_color = BLUE_500;

    // 選択ハイライト（§6 アクティブ）= blue-500 背景 + 白文字
    v.selection.bg_fill = BLUE_500;
    v.selection.stroke = Stroke::new(1.0, WHITE);

    v.window_corner_radius = CornerRadius::same(6);
    v.menu_corner_radius = CornerRadius::same(6);

    let r4 = CornerRadius::same(4); // 小要素（ボタン）
    let r6 = CornerRadius::same(6); // カード／パネル

    // 静かなクローム: 非対話（パネル・ラベル・カード）= gray-100 / gray-200 枠 / gray-700 文字
    v.widgets.noninteractive.bg_fill = GRAY_100;
    v.widgets.noninteractive.weak_bg_fill = GRAY_100;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, GRAY_200);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, GRAY_700);
    v.widgets.noninteractive.corner_radius = r6;

    // ボタン（rest）: 透明背景・gray-700 文字・gray-200 枠（入力欄/コンボ兼用）
    v.widgets.inactive.bg_fill = Color32::TRANSPARENT;
    v.widgets.inactive.weak_bg_fill = Color32::TRANSPARENT;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, GRAY_200);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, GRAY_700);
    v.widgets.inactive.corner_radius = r4;

    // ホバー: blue-300 背景 + 白文字
    v.widgets.hovered.bg_fill = BLUE_300;
    v.widgets.hovered.weak_bg_fill = BLUE_300;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, BLUE_300);
    v.widgets.hovered.fg_stroke = Stroke::new(1.5, WHITE);
    v.widgets.hovered.corner_radius = r4;

    // アクティブ（押下・選択）: blue-500 背景 + 白文字
    v.widgets.active.bg_fill = BLUE_500;
    v.widgets.active.weak_bg_fill = BLUE_500;
    v.widgets.active.bg_stroke = Stroke::new(1.0, BLUE_500);
    v.widgets.active.fg_stroke = Stroke::new(1.5, WHITE);
    v.widgets.active.corner_radius = r4;

    // コンボボックス展開トリガ: 入力欄相当（gray-100 / gray-200 枠）
    v.widgets.open.bg_fill = GRAY_100;
    v.widgets.open.weak_bg_fill = GRAY_100;
    v.widgets.open.bg_stroke = Stroke::new(1.0, GRAY_200);
    v.widgets.open.fg_stroke = Stroke::new(1.0, GRAY_700);
    v.widgets.open.corner_radius = r4;

    style.visuals = v;

    // タイポグラフィスケール（§4）: 見出し / 13 / 12 / 11
    style.text_styles = [
        (
            TextStyle::Heading,
            FontId::new(18.0, FontFamily::Proportional),
        ),
        (TextStyle::Body, FontId::new(13.0, FontFamily::Proportional)),
        (
            TextStyle::Button,
            FontId::new(13.0, FontFamily::Proportional),
        ),
        (
            TextStyle::Monospace,
            FontId::new(12.0, FontFamily::Monospace),
        ),
        (
            TextStyle::Small,
            FontId::new(11.0, FontFamily::Proportional),
        ),
    ]
    .into();

    ctx.set_global_style(style);
}
