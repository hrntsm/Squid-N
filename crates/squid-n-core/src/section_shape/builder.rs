//! [`SectionShape`] から [`Section`] を生成するビルダ（[`SectionShape::to_section`]）。

use super::constants::{KAPPA_RC, N_S_EQ};
use super::geometry::h_web_shear_area;
use super::types::SectionShape;
use crate::ids::SectionId;
use crate::model::Section;

impl SectionShape {
    /// Build a fully-populated `squid_n_core::Section` from the shape parameters.
    ///
    /// `id` and `name` must be supplied by the caller; all section properties
    /// are computed automatically.
    pub fn to_section(&self, id: SectionId, name: String) -> Section {
        let area = self.calc_area();
        let iy = self.calc_iy();
        let iz = self.calc_iz();
        let j = self.calc_j();
        // せん断変形用断面積 As（材料力学のせん断形状係数）。
        // ペアリング規約（P1 §4.1）: as_z ↔ iy（強軸曲げ→z方向せん断）、
        // as_y ↔ iz（弱軸曲げ→y方向せん断）。
        // - RC/SRC: As = A/κ（κ=1.2）。SRC は鉄骨分 sAs·(ngs−1) を累加
        //   （ngs はせん断弾性係数比。暫定的にヤング係数比 N_S_EQ で代用）。
        // - S: As = Aw/κ（κ=1.0）。強軸側はウェブ、弱軸側はフランジを有効とする。
        let (depth, width, as_y, as_z) = match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                2.0 * width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            }
            | SectionShape::CftBox {
                height,
                width,
                thick,
            } => (
                height,
                width,
                2.0 * thick * (width - 2.0 * thick).max(0.0),
                2.0 * thick * (height - 2.0 * thick).max(0.0),
            ),
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => (
                leg_a.max(leg_b),
                leg_a.min(leg_b),
                leg_b * thick,
                leg_a * thick,
            ),
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                2.0 * width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelPipe { outer_dia, .. } | SectionShape::CftPipe { outer_dia, .. } => {
                (outer_dia, outer_dia, area / 2.0, area / 2.0)
            }
            // 平鋼: 中実矩形（せい d=thick、幅 b=width）。せん断有効断面は矩形の A/κ。
            SectionShape::SteelFlatBar { width, thick } => (
                thick,
                width,
                width * thick / KAPPA_RC,
                width * thick / KAPPA_RC,
            ),
            // 中実丸鋼: せん断形状係数 κ=10/9（As=0.9A）。
            SectionShape::SteelRoundBar { dia } => (dia, dia, area * 0.9, area * 0.9),
            // リップ溝形（溝形鋼と同規約）: 強軸せん断はウェブ、弱軸はフランジが負担。
            SectionShape::SteelLipChannel {
                height,
                width,
                thick,
                ..
            } => (
                height,
                width,
                2.0 * (width - thick) * thick,
                h_web_shear_area(height, thick),
            ),
            // 非対称組立 H（H 形と同規約）: 強軸せん断はウェブ、弱軸は上下フランジが負担。
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => (
                height,
                upper_width.max(lower_width),
                upper_width * upper_thick + lower_width * lower_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::RcRect { b, d, .. } => (d, b, b * d / KAPPA_RC, b * d / KAPPA_RC),
            SectionShape::RcCircle { d, .. } => (d, d, area / KAPPA_RC, area / KAPPA_RC),
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => {
                let rc_as = b * d / KAPPA_RC;
                let s_web = h_web_shear_area(steel_height, steel_web_thick);
                let s_flange = 2.0 * steel_width * steel_flange_thick;
                (
                    d,
                    b,
                    rc_as + (N_S_EQ - 1.0) * s_flange,
                    rc_as + (N_S_EQ - 1.0) * s_web,
                )
            }
            SectionShape::RcWall { thickness, .. } => (
                1000.0,
                thickness,
                1000.0 * thickness / KAPPA_RC,
                1000.0 * thickness / KAPPA_RC,
            ),
        };
        // 板厚系の形状は Section.thickness にも板厚を反映する（検定・表示用）。
        let thickness = match *self {
            SectionShape::CftBox { thick, .. } | SectionShape::CftPipe { thick, .. } => Some(thick),
            SectionShape::RcWall { thickness, .. } => Some(thickness),
            _ => None,
        };
        Section {
            id,
            name,
            area,
            iy,
            iz,
            j,
            depth,
            width,
            as_y,
            as_z,
            panel_thickness: None,
            thickness,
            // UI設計 §4.2: Section は SectionShape の派生。生成元の形状を保持する。
            shape: Some(self.clone()),
        }
    }
}
