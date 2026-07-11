use crate::ids::SectionId;
use crate::model::Section;

/// RC 配筋の主筋セット（方向別）。
///
/// `count`: 本数, `dia`: 径 [mm], `layers`: 段数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BarSet {
    pub count: u32,
    pub dia: f64,
    pub layers: u32,
}

/// RC せん断補強筋。
///
/// `dia`: 径 [mm], `pitch`: ピッチ [mm], `legs`: 組数,
/// `grade`: 材質（None は普通強度＝主筋と同種扱い。高強度せん断補強筋は
/// 製品名/規格名で指定する。例: "UB785"（ウルボン785）, "KH785"（スーパーフープ）,
/// "KSS785", "SHD685", "SPR785", "MK785", "SBPD1275" 等）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShearBar {
    pub dia: f64,
    pub pitch: f64,
    pub legs: u32,
    #[serde(default)]
    pub grade: Option<String>,
}

/// RC 配筋情報。
///
/// `main_x`: せい方向（X）主筋, `main_y`: 幅方向（Y）主筋,
/// `cover`: かぶり [mm], `shear`: せん断補強筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcRebar {
    pub main_x: BarSet,
    pub main_y: BarSet,
    pub cover: f64,
    pub shear: ShearBar,
}

/// Parametric section shape definition.
///
/// Each variant carries the minimal parameters needed to define the geometry.
/// Call `to_section()` to compute the derived section properties (A, Iy, Iz, J, ...)
/// and produce a `squid_n_core::Section`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SectionShape {
    /// Steel H‑shape (H形鋼).
    SteelH {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel rectangular hollow section / box (角形鋼管).
    SteelBox { height: f64, width: f64, thick: f64 },
    /// Steel L‑angle (山形鋼).
    SteelAngle { leg_a: f64, leg_b: f64, thick: f64 },
    /// Steel C‑channel (溝形鋼).
    SteelChannel {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel T‑shape (T形鋼).
    SteelTee {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel round pipe (鋼管).
    SteelPipe { outer_dia: f64, thick: f64 },
    /// Reinforced concrete rectangle (RC 矩形).
    RcRect { b: f64, d: f64, rebar: RcRebar },
    /// Reinforced concrete circle column (RC 円形).
    RcCircle { d: f64, rebar: RcRebar },
    /// SRC 矩形断面（RC 矩形 + 内蔵 H 形鉄骨、SRC 規準 1987）。
    ///
    /// `steel_grade`: 内蔵鉄骨の鋼種（例 "SN400B"）。コンクリート強度は
    /// `Material.fc`、主筋グレードは `Material.name` を用いる既存慣習を踏襲する。
    ///
    /// 解析用断面性能（`to_section`）は、コンクリート断面にヤング係数比
    /// `N_S_EQ`（=15、暫定既定）による鉄骨の等価換算剛性を加えて算定する。
    /// 断面積は質量算定への影響を避けるためコンクリート全断面 `b·d` とする。
    SrcRect {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_height: f64,
        steel_width: f64,
        steel_web_thick: f64,
        steel_flange_thick: f64,
        steel_grade: String,
    },
    /// CFT 角形（角形鋼管 + 充填コンクリート）。
    ///
    /// 解析用断面性能は鋼管部分のみ（充填コンクリートの剛性は暫定的に無視、
    /// 剛性計算編での複合換算は今後の課題）。検定では `Material.fc` の
    /// 充填コンクリート強度を用いる。
    CftBox { height: f64, width: f64, thick: f64 },
    /// CFT 円形（円形鋼管 + 充填コンクリート）。扱いは `CftBox` と同じ。
    CftPipe { outer_dia: f64, thick: f64 },
    /// RC 耐震壁（壁エレメント用）。
    ///
    /// `thickness`: 壁板厚 [mm]、`ps`: 壁板の直交する各方向のせん断補強筋比の
    /// うち小さい方（小数。例 0.0025）。壁の平面寸法は要素の節点座標から得る
    /// ため形状には持たない。`to_section` の断面性能は名目値（壁は暫定的に
    /// 等価梁でモデル化されており、実剛性の評価は要素実装側の課題）。
    RcWall { thickness: f64, ps: f64 },
}

/// SRC 断面の解析剛性算定に用いる鉄骨の等価ヤング係数比（暫定既定）。
pub const N_S_EQ: f64 = 15.0;

/// RC 断面のせん断変形用断面積の形状係数 κ（As = A/κ。RESP-D 計算編 02「剛性計算」）。
pub const KAPPA_RC: f64 = 1.2;

/// 鋼材のヤング係数 [N/mm²]（SRC 内蔵鉄骨・CFT 鋼管の換算用標準値）。
pub const E_STEEL: f64 = 205000.0;

/// 鋼材のポアソン比（せん断弾性係数比 ngs の算定用）。
const NU_STEEL: f64 = 0.3;

/// コンクリートのポアソン比（CFT 充填コンクリートの換算用）。
const NU_CONCRETE: f64 = 0.2;

/// コンクリートの単位体積重量 γ [kN/m³]（普通コンクリートの標準値。
/// ヤング係数式 Ec=3.35e4·(γ/24)²·(Fc/60)^(1/3) に用いる）。
const GAMMA_CONCRETE: f64 = 23.0;

/// コンクリート強度 Fc [N/mm²] からヤング係数 Ec [N/mm²] を算定する
/// （RESP-D 計算編 02 の Ec=3.35·10⁴·(γ/24)²·(Fc/60)^(1/3)、γ=23 固定）。
pub fn concrete_young_modulus(fc: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    3.35e4 * (GAMMA_CONCRETE / 24.0).powi(2) * (fc / 60.0).powf(1.0 / 3.0)
}

/// 耐震壁（壁板＋両側柱＝平面 I 形断面）のせん断形状係数
/// （RESP-D 計算編 02「剛性計算」耐震壁の式）。
///
/// κ = 3(1+ξ)/(5·(1−ξ³(1−η))²)·[η + ξ(1−η)·((15/8)(1−ξ²)² − ξ⁴·η)]
///
/// ξ・η の定義は原典ページに明示がないため、
/// ξ=壁板内法長さ/全長（側柱外面間）、η=壁厚/側柱幅 と仮定する
/// （式の読み・記号定義とも specs/原典照合リスト.md に要照合として登録）。
/// ξ=1（側柱なし＝矩形断面）で κ=1.2（=`KAPPA_RC`）に一致する。
/// 退化（非有限・非正）時は矩形の 1.2 にフォールバックする。
pub fn wall_shear_shape_factor(xi: f64, eta: f64) -> f64 {
    let xi = xi.clamp(0.0, 1.0);
    let eta = eta.clamp(1e-6, 1.0);
    let denom = 5.0 * (1.0 - xi.powi(3) * (1.0 - eta)).powi(2);
    if denom <= 1e-12 {
        return KAPPA_RC;
    }
    let bracket = eta
        + xi * (1.0 - eta) * ((15.0 / 8.0) * (1.0 - xi * xi).powi(2) - xi.powi(4) * eta);
    let k = 3.0 * (1.0 + xi) / denom * bracket;
    if k.is_finite() && k > 0.0 {
        k
    } else {
        KAPPA_RC
    }
}

/// SRC/CFT の複合換算断面性能（要素剛性用。RESP-D 計算編 02「剛性計算」）。
///
/// いずれも要素に割り当てた材料（SRC はコンクリート、CFT は鋼管）を基準とした
/// 等価値。質量算定用の断面積（`calc_area`）とは区別して用いること。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompositeProps {
    /// 軸剛性用断面積 [mm²]
    pub area_ax: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
}

/// 矩形断面の St.Venant ねじり定数（RESP-D 計算編 02「剛性計算」の式）。
///
/// J = (b³·h/16)·[16/3 − 3.36·(b/h)·(1 − (1/12)(b/h)⁴)]（b: 短辺, h: 長辺）
///
/// アスペクト比によらず同一式を適用する（b/h→0 で β→1/3 に漸近）。
fn rect_torsion_j(b: f64, d: f64) -> f64 {
    let bs = b.min(d);
    let h = b.max(d);
    if bs <= 0.0 || h <= 0.0 {
        return 0.0;
    }
    let c = bs / h;
    bs.powi(3) * h / 16.0 * (16.0 / 3.0 - 3.36 * c * (1.0 - c.powi(4) / 12.0))
}

/// H 形（内蔵鉄骨含む）のウェブせん断断面積（ウェブ全せい×ウェブ厚。
/// 設計検定側 `squid-n-design-jp::steel::shear_area` と同一規約）。
fn h_web_shear_area(height: f64, web_thick: f64) -> f64 {
    (height * web_thick).max(0.0)
}

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
                let i_f = flange_thick * width.powi(3) / 12.0 + a_f * (width / 2.0 - z_bar).powi(2);
                let i_f2 =
                    flange_thick * width.powi(3) / 12.0 + a_f * (width / 2.0 - z_bar).powi(2);
                let i_w = hw * web_thick.powi(3) / 12.0 + a_w * (web_thick / 2.0 - z_bar).powi(2);
                i_f + i_f2 + i_w
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
            SectionShape::RcRect { b, d, .. } => rect_torsion_j(b, d),
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d.powi(4) / 32.0,
            // ねじりは RC 矩形と同じ扱い（内蔵鉄骨の寄与は無視。
            // マニュアルの J=(sG/cG)·sJ+cJ 複合換算は Material 依存のため今後の課題）。
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
    /// SRC はマニュアル（RESP-D 計算編 02「剛性計算」）の
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

    /// SRC 断面の等価断面性能を、実際のヤング係数比 ns=Es/Ec から算定する
    /// （RESP-D 計算編 02: An=rcAn+sAn·(ns−1)、Ie=rcIe+sIe·(ns−1)、
    /// As=rcAs+sAs·(ngs−1)、J=cJ+(sG/cG)·sJ）。
    ///
    /// `ec`/`nu_c`: 要素材料（コンクリート）のヤング係数・ポアソン比。
    /// 鉄骨は Es=`E_STEEL`・νs=0.3 とする。ngs=ns·(1+νc)/(1+νs)。
    /// SrcRect 以外、または ec≤0 では None（呼び出し側は `to_section` の
    /// 既定値=N_S_EQ 固定へフォールバックする）。
    pub fn src_equivalent_props(&self, ec: f64, nu_c: f64) -> Option<CompositeProps> {
        let SectionShape::SrcRect {
            b,
            d,
            steel_height: sh,
            steel_width: sw,
            steel_web_thick: tw,
            steel_flange_thick: tf,
            ..
        } = *self
        else {
            return None;
        };
        if ec <= 0.0 {
            return None;
        }
        let ns = E_STEEL / ec;
        let ngs = ns * (1.0 + nu_c) / (1.0 + NU_STEEL);

        let s_a = 2.0 * sw * tf + (sh - 2.0 * tf) * tw;
        let hw = sh - 2.0 * tf;
        let s_iy = (sw * sh.powi(3) - (sw - tw) * hw.powi(3)) / 12.0;
        let s_iz = (2.0 * tf * sw.powi(3) + hw * tw.powi(3)) / 12.0;
        let s_j = (2.0 * sw * tf.powi(3) + hw * tw.powi(3)) / 3.0;

        let rc_as = b * d / KAPPA_RC;
        Some(CompositeProps {
            area_ax: b * d + (ns - 1.0) * s_a,
            iy: b * d.powi(3) / 12.0 + (ns - 1.0) * s_iy,
            iz: d * b.powi(3) / 12.0 + (ns - 1.0) * s_iz,
            j: rect_torsion_j(b, d) + ngs * s_j,
            as_y: rc_as + (ngs - 1.0) * 2.0 * sw * tf,
            as_z: rc_as + (ngs - 1.0) * h_web_shear_area(sh, tw),
        })
    }

    /// CFT 断面（CftBox/CftPipe）の等価断面性能を鋼管基準で算定する
    /// （RESP-D 計算編 02: SRC柱「CFTもこれに準じます」の累加を鋼基準の
    /// 1/n 換算で適用。J は S柱の J=(sG/cG)·sJ+cJ を鋼基準 J=sJ+cJ/ngs に換算）。
    ///
    /// `es`/`nu_s`: 要素材料（鋼管）のヤング係数・ポアソン比、
    /// `fc`: 充填コンクリート強度（`Material.fc`）。
    /// Ec は `concrete_young_modulus`（γ=23）・νc=0.2 とする。
    /// CftBox/CftPipe 以外、または Ec≤0 では None（鋼管のみの既定値へ
    /// フォールバック）。
    pub fn cft_equivalent_props(&self, es: f64, nu_s: f64, fc: f64) -> Option<CompositeProps> {
        let ec = concrete_young_modulus(fc);
        if ec <= 0.0 || es <= 0.0 {
            return None;
        }
        let n = es / ec;
        let ngs = n * (1.0 + NU_CONCRETE) / (1.0 + nu_s);
        match *self {
            SectionShape::CftBox {
                height: h,
                width: w,
                thick: t,
            } => {
                let (bi, hi) = (w - 2.0 * t, h - 2.0 * t);
                if bi <= 0.0 || hi <= 0.0 {
                    return None;
                }
                let c_a = bi * hi;
                let s_as_z = 2.0 * t * hi;
                let s_as_y = 2.0 * t * bi;
                Some(CompositeProps {
                    area_ax: self.calc_area() + c_a / n,
                    iy: self.calc_iy() + bi * hi.powi(3) / 12.0 / n,
                    iz: self.calc_iz() + hi * bi.powi(3) / 12.0 / n,
                    j: self.calc_j() + rect_torsion_j(bi, hi) / ngs,
                    as_y: s_as_y + c_a / KAPPA_RC / ngs,
                    as_z: s_as_z + c_a / KAPPA_RC / ngs,
                })
            }
            SectionShape::CftPipe { outer_dia, thick } => {
                let di = outer_dia - 2.0 * thick;
                if di <= 0.0 {
                    return None;
                }
                let c_a = std::f64::consts::PI * di * di / 4.0;
                let c_i = std::f64::consts::PI * di.powi(4) / 64.0;
                let c_j = std::f64::consts::PI * di.powi(4) / 32.0;
                let s_as = self.calc_area() / 2.0;
                Some(CompositeProps {
                    area_ax: self.calc_area() + c_a / n,
                    iy: self.calc_iy() + c_i / n,
                    iz: self.calc_iz() + c_i / n,
                    j: self.calc_j() + c_j / ngs,
                    as_y: s_as + c_a / KAPPA_RC / ngs,
                    as_z: s_as + c_a / KAPPA_RC / ngs,
                })
            }
            _ => None,
        }
    }

    /// Build a fully-populated `squid_n_core::Section` from the shape parameters.
    ///
    /// `id` and `name` must be supplied by the caller; all section properties
    /// are computed automatically.
    pub fn to_section(&self, id: SectionId, name: String) -> Section {
        let area = self.calc_area();
        let iy = self.calc_iy();
        let iz = self.calc_iz();
        let j = self.calc_j();
        // せん断変形用断面積 As（RESP-D 計算編 02「剛性計算」）。
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

/// 主筋セットの総断面積 [mm²]（本数×πr²。配筋検定・ファイバー生成用）。
pub fn bar_set_area(bs: &BarSet) -> f64 {
    let r = bs.dia / 2.0;
    bs.count as f64 * std::f64::consts::PI * r * r
}

fn angle_centroid(leg_a: f64, leg_b: f64, thick: f64) -> (f64, f64, f64) {
    let a1 = leg_a * thick;
    let a2 = (leg_b - thick) * thick;
    let a_total = a1 + a2;
    if a_total < 1e-30 {
        return (0.0, 0.0, 0.0);
    }
    let cy = (a1 * leg_a / 2.0 + a2 * thick / 2.0) / a_total;
    let cx = (a1 * thick / 2.0 + a2 * (thick + (leg_b - thick) / 2.0)) / a_total;
    (cx, cy, a_total)
}

fn tee_centroid(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> f64 {
    let a_f = width * flange_thick;
    let a_w = (height - flange_thick) * web_thick;
    let a_total = a_f + a_w;
    if a_total < 1e-30 {
        return 0.0;
    }
    (a_f * (height - flange_thick / 2.0) + a_w * (height - flange_thick) / 2.0) / a_total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steel_h_shape() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300x300x10x15".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > sec.iz);
        assert!(sec.j > 0.0);
    }

    #[test]
    fn test_steel_box() {
        let shape = SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "BOX-200x200x12".into());
        assert!(sec.area > 0.0);
        assert!((sec.iy - sec.iz).abs() < 1.0);
    }

    #[test]
    fn test_steel_pipe() {
        let shape = SectionShape::SteelPipe {
            outer_dia: 216.3,
            thick: 8.2,
        };
        let sec = shape.to_section(SectionId(0), "PIPE-216.3x8.2".into());
        assert!(sec.area > 0.0);
        assert!((sec.iy - sec.iz).abs() < 1e-6);
        assert!(sec.j > sec.iy);
    }

    #[test]
    fn test_rc_rect() {
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 16.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 16.0,
                    layers: 2,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        let sec = shape.to_section(SectionId(0), "RC-500x500".into());
        assert!(sec.area > 0.0);
        assert!(sec.as_y > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_rc_circle() {
        let shape = SectionShape::RcCircle {
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 6.0,
                    pitch: 80.0,
                    legs: 1,
                    grade: None,
                },
            },
        };
        let sec = shape.to_section(SectionId(0), "RC-600".into());
        assert!(sec.area > 0.0);
        assert!(sec.as_y > 0.0);
        assert!(sec.as_z > 0.0);
    }

    #[test]
    fn test_steel_l_angle() {
        let shape = SectionShape::SteelAngle {
            leg_a: 150.0,
            leg_b: 100.0,
            thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "L-150x100x12".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_steel_tee() {
        let shape = SectionShape::SteelTee {
            height: 200.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "T-200x200x10x15".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_steel_channel() {
        let shape = SectionShape::SteelChannel {
            height: 250.0,
            width: 90.0,
            web_thick: 7.5,
            flange_thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "C-250x90x7.5x12".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > sec.iz);
    }

    #[test]
    fn test_section_roundtrip_serde() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let json = serde_json::to_string(&shape).unwrap();
        let restored: SectionShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, restored);
    }

    #[test]
    fn test_rc_rebar_serde_roundtrip() {
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 22.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 2,
                    dia: 16.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        let json = serde_json::to_string(&shape).unwrap();
        let restored: SectionShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, restored);
    }

    #[test]
    fn test_rc_rect_area() {
        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 19.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 2,
                    dia: 13.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        assert!((shape.calc_area() - 240_000.0).abs() < 1e-9);
        let iy = shape.calc_iy();
        let iz = shape.calc_iz();
        assert!((iy - 400.0 * 600.0_f64.powi(3) / 12.0).abs() < 1e-6);
        assert!((iz - 600.0 * 400.0_f64.powi(3) / 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_rc_rect_shear_area_is_gross_over_kappa() {
        // RESP-D 計算編 02: RC の As = B·D/κ（κ=1.2）。鉄筋断面積ではない。
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 2,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        let sec = shape.to_section(SectionId(0), "RC-500x500".into());
        let expected = 500.0 * 500.0 / 1.2;
        assert!((sec.as_y - expected).abs() < 1e-6);
        assert!((sec.as_z - expected).abs() < 1e-6);
    }

    #[test]
    fn test_steel_h_shear_area_is_web_and_flange() {
        // RESP-D 計算編 02: S の As = Aw/κ（κ=1.0）。強軸側(as_z)=ウェブ、弱軸側(as_y)=フランジ。
        let shape = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "H-400x200x9x12".into());
        assert!((sec.as_z - 400.0 * 9.0).abs() < 1e-9);
        assert!((sec.as_y - 2.0 * 200.0 * 12.0).abs() < 1e-9);
    }

    #[test]
    fn test_rc_rect_torsion_matches_manual_formula() {
        // J = (b³h/16)[16/3 − 3.36(b/h)(1 − (1/12)(b/h)⁴)]。細長比によらず同一式。
        let rebar = RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        };
        // 正方形 500×500
        let sq = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: rebar.clone(),
        };
        let b: f64 = 500.0;
        let expected = b.powi(3) * b / 16.0 * (16.0 / 3.0 - 3.36 * (1.0 - 1.0 / 12.0));
        assert!((sq.calc_j() - expected).abs() / expected < 1e-12);
        // 細長断面 100×2000（旧実装は r≥10 で β=1/3 に切替え約+6.7%乖離していた）
        let slender = SectionShape::RcRect {
            b: 100.0,
            d: 2000.0,
            rebar,
        };
        let (bs, h) = (100.0_f64, 2000.0_f64);
        let c = bs / h;
        let expected2 = bs.powi(3) * h / 16.0 * (16.0 / 3.0 - 3.36 * c * (1.0 - c.powi(4) / 12.0));
        assert!((slender.calc_j() - expected2).abs() / expected2 < 1e-12);
    }

    #[test]
    fn test_src_axial_stiffness_area_accumulates_steel() {
        // An = rcAn + sAn·(ns−1)。calc_area（質量用）はコンクリート全断面のまま。
        let shape = SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
            steel_grade: "SN400B".into(),
        };
        let s_a = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 9.0;
        assert!((shape.calc_area() - 360_000.0).abs() < 1e-9);
        let expected = 360_000.0 + (N_S_EQ - 1.0) * s_a;
        assert!((shape.calc_axial_stiffness_area() - expected).abs() < 1e-9);
        // せん断断面積も RC 部 A/1.2 + 鉄骨等価分が累加される
        let sec = shape.to_section(SectionId(0), "SRC-600".into());
        let rc_as = 360_000.0 / 1.2;
        assert!((sec.as_z - (rc_as + (N_S_EQ - 1.0) * 400.0 * 9.0)).abs() < 1e-9);
        assert!((sec.as_y - (rc_as + (N_S_EQ - 1.0) * 2.0 * 200.0 * 12.0)).abs() < 1e-9);
    }

    fn make_src_600() -> SectionShape {
        SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
            steel_grade: "SN400B".into(),
        }
    }

    #[test]
    fn test_wall_shear_shape_factor_rectangle_limit() {
        // ξ=1(側柱なし=矩形)は η によらず κ=1.2
        for eta in [0.1, 0.5, 1.0] {
            let k = wall_shear_shape_factor(1.0, eta);
            assert!((k - KAPPA_RC).abs() < 1e-12, "eta={eta} k={k}");
        }
        // 側柱付き(ξ<1)は有限・正の値
        let k = wall_shear_shape_factor(0.8, 0.3);
        assert!(k.is_finite() && k > 0.0);
        // 退化入力でも非有限値・負値は返さない
        let k0 = wall_shear_shape_factor(0.0, 0.0);
        assert!(k0.is_finite() && k0 > 0.0);
    }

    #[test]
    fn test_concrete_young_modulus_formula() {
        // Ec = 3.35e4·(γ/24)²·(Fc/60)^(1/3)、γ=23。Fc=60 で (Fc/60)^(1/3)=1。
        let expected = 3.35e4 * (23.0_f64 / 24.0).powi(2);
        assert!((concrete_young_modulus(60.0) - expected).abs() < 1e-9);
        assert_eq!(concrete_young_modulus(0.0), 0.0);
    }

    #[test]
    fn test_src_equivalent_props_uses_material_ns() {
        // ns = Es/Ec を材料から算定（N_S_EQ=15 固定ではない）。
        let shape = make_src_600();
        let ec = 23000.0;
        let p = shape.src_equivalent_props(ec, 0.2).unwrap();
        let ns = E_STEEL / ec;
        let s_a = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 9.0;
        assert!((p.area_ax - (360_000.0 + (ns - 1.0) * s_a)).abs() < 1e-6);
        // Iy = Ic + (ns−1)·sIy
        let s_iy = (200.0 * 400.0_f64.powi(3) - (200.0 - 9.0) * 376.0_f64.powi(3)) / 12.0;
        let expected_iy = 600.0 * 600.0_f64.powi(3) / 12.0 + (ns - 1.0) * s_iy;
        assert!((p.iy - expected_iy).abs() / expected_iy < 1e-12);
        // ngs = ns·(1+νc)/(1+νs)
        let ngs = ns * 1.2 / 1.3;
        let rc_as = 360_000.0 / 1.2;
        assert!((p.as_z - (rc_as + (ngs - 1.0) * 400.0 * 9.0)).abs() < 1e-6);
        // Ec≤0 は None（既定 N_S_EQ へのフォールバック用）
        assert!(shape.src_equivalent_props(0.0, 0.2).is_none());
    }

    #[test]
    fn test_cft_equivalent_props_adds_concrete() {
        // 鋼管基準の 1/n 換算で充填コンクリートを累加（A・I・J とも鋼管のみより増える）。
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let es = 205000.0;
        let fc = 36.0;
        let p = shape.cft_equivalent_props(es, 0.3, fc).unwrap();
        let n = es / concrete_young_modulus(fc);
        let (bi, hi) = (400.0 - 24.0, 400.0 - 24.0);
        assert!((p.area_ax - (shape.calc_area() + bi * hi / n)).abs() < 1e-6);
        let expected_iy = shape.calc_iy() + bi * hi.powi(3) / 12.0 / n;
        assert!((p.iy - expected_iy).abs() / expected_iy < 1e-12);
        assert!(p.j > shape.calc_j());
        assert!(p.as_z > 2.0 * 12.0 * hi);
        // 円形も同様
        let pipe = SectionShape::CftPipe {
            outer_dia: 400.0,
            thick: 12.0,
        };
        let pp = pipe.cft_equivalent_props(es, 0.3, fc).unwrap();
        assert!(pp.area_ax > pipe.calc_area());
        assert!(pp.iy > pipe.calc_iy());
        // 非CFT形状は None
        assert!(make_src_600().cft_equivalent_props(es, 0.3, fc).is_none());
    }

    #[test]
    fn test_rc_circle_area() {
        let shape = SectionShape::RcCircle {
            d: 800.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 16,
                    dia: 25.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 16,
                    dia: 25.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 1,
                    grade: None,
                },
            },
        };
        let expected_area = std::f64::consts::PI * 800.0_f64.powi(2) / 4.0;
        assert!((shape.calc_area() - expected_area).abs() < 1e-6);
        let iy = shape.calc_iy();
        assert!((iy - std::f64::consts::PI * 800.0_f64.powi(4) / 64.0).abs() < 1e-6);
        assert!((shape.calc_iy() - shape.calc_iz()).abs() < 1e-6);
    }
}
