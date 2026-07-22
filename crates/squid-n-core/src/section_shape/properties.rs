//! [`SectionShape`] の基本断面性能（A, Zp, Iy, Iz, J, 軸剛性用断面積）。

use super::constants::N_S_EQ;
use super::geometry::{
    angle_centroid, built_h_centroid_y, lip_channel_centroid_z, rect_torsion_j, tee_centroid,
};
use super::types::SectionShape;

impl SectionShape {
    /// Compute the cross‑sectional area [mm²].
    pub fn calc_area(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => 2.0 * width * flange_thick + (height - 2.0 * flange_thick) * web_thick,
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => width * height - (width - 2.0 * thick) * (height - 2.0 * thick),
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => thick * (leg_a + leg_b - thick),
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => 2.0 * width * flange_thick + (height - 2.0 * flange_thick) * web_thick,
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => width * flange_thick + (height - flange_thick) * web_thick,
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI * (r * r - ri * ri)
            }
            SectionShape::SteelFlatBar { width, thick } => width * thick,
            SectionShape::SteelRoundBar { dia } => std::f64::consts::PI * dia * dia / 4.0,
            SectionShape::SteelLipChannel {
                height,
                width,
                lip,
                thick,
            } => {
                let (_, area) = lip_channel_centroid_z(height, width, lip, thick);
                area
            }
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => {
                let (_, area) = built_h_centroid_y(
                    height,
                    upper_width,
                    upper_thick,
                    lower_width,
                    lower_thick,
                    web_thick,
                );
                area
            }
            SectionShape::RcRect { b, d, .. } => b * d,
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d * d / 4.0,
            // SRC: 質量算定への影響を避けるためコンクリート全断面とする（doc 参照）。
            SectionShape::SrcRect { b, d, .. } => b * d,
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => width * height - (width - 2.0 * thick) * (height - 2.0 * thick),
            SectionShape::CftPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI * (r * r - ri * ri)
            }
            // 壁: 名目値（1m 幅相当の板断面。解析剛性は要素実装側の課題）。
            SectionShape::RcWall { thickness, .. } => thickness * 1000.0,
        }
    }

    /// 鉄骨断面の塑性断面係数 Zp [mm³]（強軸）。H・箱・パイプは閉形式、
    /// それ以外（RC・SRC・CFT・不明形状）は None を返す（鉄骨梁の全塑性
    /// モーメント Mp=Zp·σy の算定に用いる。材料力学）。
    pub fn plastic_modulus_strong(&self) -> Option<f64> {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => Some(
                width * flange_thick * (height - flange_thick)
                    + web_thick * (height - 2.0 * flange_thick).powi(2) / 4.0,
            ),
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => Some(
                width * height * height / 4.0
                    - (width - 2.0 * thick) * (height - 2.0 * thick).powi(2) / 4.0,
            ),
            SectionShape::SteelPipe { outer_dia, thick } => {
                Some((outer_dia.powi(3) - (outer_dia - 2.0 * thick).powi(3)) / 6.0)
            }
            _ => None,
        }
    }

    /// Moment of inertia about the local y‑axis [mm⁴] (strong axis for beams).
    pub fn calc_iy(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (width * height.powi(3) - (width - web_thick) * hw.powi(3)) / 12.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => {
                let hi = height - 2.0 * thick;
                (width * height.powi(3) - (width - 2.0 * thick) * hi.powi(3)) / 12.0
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => {
                let (_, cy, _) = angle_centroid(leg_a, leg_b, thick);
                let a1 = leg_a * thick;
                let y1 = leg_a / 2.0;
                let a2 = (leg_b - thick) * thick;
                let y2 = thick / 2.0;
                let i1 = thick * leg_a.powi(3) / 12.0;
                let i2 = (leg_b - thick) * thick.powi(3) / 12.0;
                (i1 + a1 * (y1 - cy).powi(2)) + (i2 + a2 * (y2 - cy).powi(2))
            }
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (width * height.powi(3) - (width - web_thick) * hw.powi(3)) / 12.0
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let y_bar = tee_centroid(height, width, web_thick, flange_thick);
                let a_f = width * flange_thick;
                let a_w = (height - flange_thick) * web_thick;
                let i_f = width * flange_thick.powi(3) / 12.0
                    + a_f * (height - flange_thick / 2.0 - y_bar).powi(2);
                let i_w = web_thick * (height - flange_thick).powi(3) / 12.0
                    + a_w * ((height - flange_thick) / 2.0 - y_bar).powi(2);
                i_f + i_w
            }
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 4.0 * (r.powi(4) - ri.powi(4))
            }
            // 平鋼は中実矩形（せい d=thick、幅 b=width）。iy は b·d³/12。
            SectionShape::SteelFlatBar { width, thick } => width * thick.powi(3) / 12.0,
            SectionShape::SteelRoundBar { dia } => std::f64::consts::PI * dia.powi(4) / 64.0,
            // リップ溝形: 強軸（せい方向 y=H/2 まわり）。矩形分解＋平行軸。上下対称。
            SectionShape::SteelLipChannel {
                height,
                width,
                lip,
                thick,
            } => {
                let t = thick;
                // ウェブ（幅 t×せい H、図心 y=H/2 でオフセット 0）。
                let i_web = t * height.powi(3) / 12.0;
                // フランジ（幅 (B−t)×厚 t、図心 y=H−t/2 → オフセット (H−t)/2）。2 枚。
                let a_f = (width - t) * t;
                let off_f = (height - t) / 2.0;
                let i_f = (width - t) * t.powi(3) / 12.0 + a_f * off_f.powi(2);
                // リップ（幅 t×せい (C−t)、図心 y=H−(C+t)/2 → オフセット (H−C−t)/2）。2 枚。
                let a_l = t * (lip - t);
                let off_l = (height - lip - t) / 2.0;
                let i_l = t * (lip - t).powi(3) / 12.0 + a_l * off_l.powi(2);
                i_web + 2.0 * i_f + 2.0 * i_l
            }
            // 非対称組立 H: 強軸（図心 y_bar まわり）。上下フランジ＋ウェブの平行軸。
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => {
                let (y_bar, _) = built_h_centroid_y(
                    height,
                    upper_width,
                    upper_thick,
                    lower_width,
                    lower_thick,
                    web_thick,
                );
                let hw = (height - upper_thick - lower_thick).max(0.0);
                let a_uf = upper_width * upper_thick;
                let y_uf = height - upper_thick / 2.0;
                let i_uf = upper_width * upper_thick.powi(3) / 12.0 + a_uf * (y_uf - y_bar).powi(2);
                let a_lf = lower_width * lower_thick;
                let y_lf = lower_thick / 2.0;
                let i_lf = lower_width * lower_thick.powi(3) / 12.0 + a_lf * (y_lf - y_bar).powi(2);
                let a_w = web_thick * hw;
                let y_w = lower_thick + hw / 2.0;
                let i_w = web_thick * hw.powi(3) / 12.0 + a_w * (y_w - y_bar).powi(2);
                i_uf + i_lf + i_w
            }
            SectionShape::RcRect { b, d, .. } => b * d.powi(3) / 12.0,
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d.powi(4) / 64.0,
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => {
                let i_c = b * d.powi(3) / 12.0;
                let hw = steel_height - 2.0 * steel_flange_thick;
                let i_s = (steel_width * steel_height.powi(3)
                    - (steel_width - steel_web_thick) * hw.powi(3))
                    / 12.0;
                i_c + (N_S_EQ - 1.0) * i_s
            }
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => {
                let hi = height - 2.0 * thick;
                (width * height.powi(3) - (width - 2.0 * thick) * hi.powi(3)) / 12.0
            }
            SectionShape::CftPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 4.0 * (r.powi(4) - ri.powi(4))
            }
            SectionShape::RcWall { thickness, .. } => 1000.0 * thickness.powi(3) / 12.0,
        }
    }

    /// Moment of inertia about the local z‑axis [mm⁴] (weak axis for beams).
    pub fn calc_iz(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (2.0 * flange_thick * width.powi(3) + hw * web_thick.powi(3)) / 12.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => {
                let wi = width - 2.0 * thick;
                (height * width.powi(3) - (height - 2.0 * thick) * wi.powi(3)) / 12.0
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => {
                let (cx, _, _) = angle_centroid(leg_a, leg_b, thick);
                let a1 = leg_a * thick;
                let z1 = thick / 2.0;
                let a2 = (leg_b - thick) * thick;
                let z2 = thick + (leg_b - thick) / 2.0;
                let i1 = leg_a * thick.powi(3) / 12.0;
                let i2 = thick * (leg_b - thick).powi(3) / 12.0;
                (i1 + a1 * (z1 - cx).powi(2)) + (i2 + a2 * (z2 - cx).powi(2))
            }
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                let a_f = width * flange_thick;
                let a_w = hw * web_thick;
                let a_total = 2.0 * a_f + a_w;
                let z_bar = if a_total > 0.0 {
                    (2.0 * a_f * width / 2.0 + a_w * web_thick / 2.0) / a_total
                } else {
                    0.0
                };
                // 上下フランジは同一寄与（左右対称）。2 枚分をまとめて計上する。
                let i_f = flange_thick * width.powi(3) / 12.0 + a_f * (width / 2.0 - z_bar).powi(2);
                let i_w = hw * web_thick.powi(3) / 12.0 + a_w * (web_thick / 2.0 - z_bar).powi(2);
                2.0 * i_f + i_w
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let iz = flange_thick * width.powi(3) / 12.0;
                let iz_w = (height - flange_thick) * web_thick.powi(3) / 12.0;
                iz + iz_w
            }
            SectionShape::SteelPipe { .. } => self.calc_iy(),
            // 平鋼は中実矩形。iz は d·b³/12（b=width、d=thick）。
            SectionShape::SteelFlatBar { width, thick } => thick * width.powi(3) / 12.0,
            SectionShape::SteelRoundBar { .. } => self.calc_iy(),
            // リップ溝形: 弱軸（幅方向 z=z_bar まわり）。矩形分解＋平行軸（Z へ偏心）。
            SectionShape::SteelLipChannel {
                height,
                width,
                lip,
                thick,
            } => {
                let t = thick;
                let (z_bar, _) = lip_channel_centroid_z(height, width, lip, thick);
                // ウェブ（z 方向厚 t、y 方向せい H、z 図心 t/2）。
                let a_web = t * height;
                let i_web = height * t.powi(3) / 12.0 + a_web * (t / 2.0 - z_bar).powi(2);
                // フランジ（z 方向 (B−t)、y 方向 t、z 図心 (t+B)/2）。2 枚。
                let a_f = (width - t) * t;
                let z_f = (t + width) / 2.0;
                let i_f = t * (width - t).powi(3) / 12.0 + a_f * (z_f - z_bar).powi(2);
                // リップ（z 方向厚 t、y 方向 (C−t)、z 図心 B−t/2）。2 枚。
                let a_l = t * (lip - t);
                let z_l = width - t / 2.0;
                let i_l = (lip - t) * t.powi(3) / 12.0 + a_l * (z_l - z_bar).powi(2);
                i_web + 2.0 * i_f + 2.0 * i_l
            }
            // 非対称組立 H: 弱軸（z=0、左右対称なので各板の自己慣性のみ）。
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => {
                let hw = (height - upper_thick - lower_thick).max(0.0);
                upper_thick * upper_width.powi(3) / 12.0
                    + lower_thick * lower_width.powi(3) / 12.0
                    + hw * web_thick.powi(3) / 12.0
            }
            SectionShape::RcRect { b, d, .. } => d * b.powi(3) / 12.0,
            SectionShape::RcCircle { .. } => self.calc_iy(),
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => {
                let i_c = d * b.powi(3) / 12.0;
                let hw = steel_height - 2.0 * steel_flange_thick;
                let i_s = (2.0 * steel_flange_thick * steel_width.powi(3)
                    + hw * steel_web_thick.powi(3))
                    / 12.0;
                i_c + (N_S_EQ - 1.0) * i_s
            }
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => {
                let wi = width - 2.0 * thick;
                (height * width.powi(3) - (height - 2.0 * thick) * wi.powi(3)) / 12.0
            }
            SectionShape::CftPipe { .. } => self.calc_iy(),
            // 壁: 面外は薄いため名目的に iy と同値の板剛性を返す。
            SectionShape::RcWall { .. } => self.calc_iy(),
        }
    }

    /// Torsional constant J [mm⁴].
    pub fn calc_j(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                (2.0 * width * flange_thick.powi(3)
                    + (height - 2.0 * flange_thick) * web_thick.powi(3))
                    / 3.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => {
                let a0 = (height - thick) * (width - thick);
                let perim = 2.0 * (height + width - 2.0 * thick);
                4.0 * a0 * a0 * thick / perim
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => ((leg_a + leg_b - thick) * thick.powi(3)) / 3.0,
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                (2.0 * width * flange_thick.powi(3)
                    + (height - 2.0 * flange_thick) * web_thick.powi(3))
                    / 3.0
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => (width * flange_thick.powi(3) + (height - flange_thick) * web_thick.powi(3)) / 3.0,
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 2.0 * (r.powi(4) - ri.powi(4))
            }
            // 平鋼は中実矩形のねじり定数（RC 矩形と同じ閉形式）。
            SectionShape::SteelFlatBar { width, thick } => rect_torsion_j(width, thick),
            SectionShape::SteelRoundBar { dia } => std::f64::consts::PI * dia.powi(4) / 32.0,
            // リップ溝形（薄肉開断面）: J = (1/3)Σ l·t³。矩形分解の板長で近似。
            SectionShape::SteelLipChannel {
                height,
                width,
                lip,
                thick,
            } => {
                let len = height + 2.0 * (width - thick) + 2.0 * (lip - thick);
                len * thick.powi(3) / 3.0
            }
            // 非対称組立 H（開断面）: J = (1/3)Σ b·t³（各板）。
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => {
                let hw = (height - upper_thick - lower_thick).max(0.0);
                (upper_width * upper_thick.powi(3)
                    + lower_width * lower_thick.powi(3)
                    + hw * web_thick.powi(3))
                    / 3.0
            }
            SectionShape::RcRect { b, d, .. } => rect_torsion_j(b, d),
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d.powi(4) / 32.0,
            // ねじりは RC 矩形と同じ扱い（内蔵鉄骨の寄与は無視。
            // 各種合成構造設計指針の J=(sG/cG)·sJ+cJ 複合換算は Material 依存のため今後の課題）。
            SectionShape::SrcRect { b, d, .. } => rect_torsion_j(b, d),
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => {
                let a0 = (height - thick) * (width - thick);
                let perim = 2.0 * (height + width - 2.0 * thick);
                4.0 * a0 * a0 * thick / perim
            }
            SectionShape::CftPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 2.0 * (r.powi(4) - ri.powi(4))
            }
            SectionShape::RcWall { thickness, .. } => 1000.0 * thickness.powi(3) / 3.0,
        }
    }

    /// 軸剛性（EA）算定用の等価断面積 [mm²]。
    ///
    /// SRC は各種合成構造設計指針の
    /// An = rcAn + sAn·(ns−1) に従い鉄骨の等価換算断面を累加する
    /// （ns は暫定的に `N_S_EQ`）。質量算定用の断面積（`calc_area` は
    /// コンクリート全断面）とは区別して用いること。他形状は `calc_area` と同値。
    pub fn calc_axial_stiffness_area(&self) -> f64 {
        match *self {
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => {
                let s_a = 2.0 * steel_width * steel_flange_thick
                    + (steel_height - 2.0 * steel_flange_thick) * steel_web_thick;
                b * d + (N_S_EQ - 1.0) * s_a
            }
            _ => self.calc_area(),
        }
    }
}
