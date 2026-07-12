//! 鋼構造の許容応力度と断面検定（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」鋼構造部分、根拠規準は鋼構造設計規準 1973・構造規定）。
//!
//! ## 形状情報の取得について
//!
//! 検定式（ウェブせん断面積 `tw·H`、圧縮フランジ断面積 `B·tf` 等）にはフランジ厚
//! `tf`・ウェブ厚 `tw` が必要になる。形状は次の優先順で解決する:
//!
//! 1. `Section.shape`（[`squid_n_core::section_shape::SectionShape`]）があれば
//!    `SteelH`/`SteelBox`/`SteelPipe` の実寸（`flange_thick`/`web_thick`/`thick`）
//!    を用いる（パラメトリック断面の正規経路）。
//! 2. 無ければ `Section.name` の先頭トークン（`"H-..."`, `"BOX-..."`,
//!    `"PIPE-..."`）から形状カテゴリを推定し、板厚は `Section.thickness` の
//!    単一値を `tf ≈ tw` として近似する（カタログ断面等のフォールバック。
//!    フランジとウェブの実厚が異なる断面では誤差を生む）。
//!
//! 命名規則にも合わない場合は `Other`（一般断面フォールバック）として扱い、
//! 横座屈低減なし（fb=ft）・単純 τ/fs 検定になる。
//!
//! ## モジュール構成（RESP-D「04 断面検定」の章立てに対応）
//!
//! - [`section`][]: 鉄骨の断面検定における断面性能（許容曲げ応力度 fb・断面
//!   二次半径・断面欠損・横座屈長さ）。
//! - [`beam`][]: 鉄骨造梁の断面検定（必要横補剛数・たわみを含む）。
//! - [`column`][]: 鉄骨造柱の断面検定。
//! - [`brace`][]: 鉄骨ブレースの断面検定。
//! - [`panel_zone`][]: S 造パネルゾーンの断面検定（鋼構造接合部設計指針）。
//! - [`cold_formed`][]: 冷間成形角形鋼管柱の柱梁耐力比チェック（2008年版
//!   角形鋼管設計・施工マニュアル）。

use crate::{CheckResult, DesignCheck, DesignCtx, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

// 鋼材の F 値・許容応力度（ft/fs/fc・限界細長比 Λ・板厚区分）は
// `crate::material_strength`（RESP-D「材料強度・許容応力度」節）へ集約した。鋼構造の
// 検定で用いるものを再エクスポートし、従来のパスも維持する。
pub use crate::material_strength::{
    big_lambda, plate_thickness, steel_f_value, steel_f_value_prefix, steel_fc, steel_fs, steel_ft,
};

mod beam;
/// 鉄骨造梁の非線形復元力特性（全塑性 Mp・横座屈 Mcr・軸 Nu）。
/// 非線形解析の材端バネ骨格に用いる（RESP-D「05 非線形モデル」）。
pub mod beam_nonlinear;
mod brace;
/// 鉄骨造柱の座屈長さ係数 K（鋼構造塑性設計指針、水平移動非拘束）。
pub mod buckling;
pub mod cold_formed;
mod column;
pub mod panel_zone;
mod section;

// 鉄骨の断面検定における断面性能（fb・断面二次半径・断面欠損・横座屈長さ）は
// `section` サブモジュールへ集約したうえで、従来のパス（`crate::steel::X`）を
// 維持するために再エクスポートする。
pub use section::{resolve_lb, steel_fb_h, steel_h_z_with_loss, steel_i_t};

// ---------------------------------------------------------------------
// 断面形状カテゴリ（`Section.shape` 優先、無ければ `Section.name` から推定。
// 上記モジュール doc 参照）
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShapeCategory {
    H,
    Box,
    Pipe,
    Other,
}

/// `Section.name` の先頭アルファベットトークンから形状カテゴリを推定する。
/// 例: "H-300x300x10x15"→H、"BOX-200x200x12"→Box、"PIPE-216.3x8.2"→Pipe。
/// 該当しない場合は `Other`（一般断面フォールバック）。
fn classify_shape(name: &str) -> ShapeCategory {
    let token: String = name
        .trim()
        .to_uppercase()
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .collect();
    match token.as_str() {
        "H" => ShapeCategory::H,
        "BOX" => ShapeCategory::Box,
        "PIPE" | "P" => ShapeCategory::Pipe,
        _ => ShapeCategory::Other,
    }
}

/// 形状カテゴリと板厚 `(カテゴリ, tf, tw)` を解決する。
///
/// `Section.shape`（パラメトリック断面）があれば実寸のフランジ厚・ウェブ厚を、
/// 無ければ断面名からカテゴリを推定して `Section.thickness` を `tf ≈ tw` の
/// 単一板厚として近似する（モジュール doc 参照）。
fn shape_of(sec: &Section) -> (ShapeCategory, f64, f64) {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::H, flange_thick, web_thick),
            SectionShape::SteelBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::SteelPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::Other, flange_thick, web_thick),
            SectionShape::SteelAngle { thick, .. } => return (ShapeCategory::Other, thick, thick),
            // CFT の鋼管部分は角形/円形鋼管として扱う（検定本体は cft 側で行う）。
            SectionShape::CftBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::CftPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. } => return (ShapeCategory::Other, 0.0, 0.0),
        }
    }
    let t = sec.thickness.unwrap_or(0.0);
    (classify_shape(&sec.name), t, t)
}

/// せん断有効断面積 As [mm²]（梁の H形以外／柱の H形・その他 で共用）。
/// - H: `tw·H`（ウェブ全せい×ウェブ厚）
/// - Box: `2·t·(H−2t)`
/// - Pipe: `A/2`
/// - Other: `as_y>0 ? as_y : area`
fn shear_area(shape: ShapeCategory, sec: &Section, tw: f64) -> f64 {
    let h = sec.depth;
    let t = tw;
    match shape {
        ShapeCategory::H => (t * h).max(0.0),
        ShapeCategory::Box => (2.0 * t * (h - 2.0 * t).max(0.0)).max(0.0),
        ShapeCategory::Pipe => sec.area / 2.0,
        ShapeCategory::Other => {
            if sec.as_y > 0.0 {
                sec.as_y
            } else {
                sec.area
            }
        }
    }
}

/// 分母が極小の場合に安全側デフォルトへ逃がすヘルパー。
fn safe_denom(x: f64) -> f64 {
    if x.abs() > 1e-9 {
        x
    } else {
        1e-9
    }
}

/// 断面係数 Z = I / (半せい)。半せいが極小なら 0（呼び出し側で 1.0 にフォールバック）。
fn section_modulus(i: f64, half_dim: f64) -> f64 {
    if half_dim > 1e-9 {
        i / half_dim
    } else {
        0.0
    }
}

fn nonzero(z: f64) -> f64 {
    if z.abs() > 1e-9 {
        z
    } else {
        1.0
    }
}

pub struct SteelDesign;

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let t = plate_thickness(sec);
        let f = steel_f_value_prefix(&mat.name, t).unwrap_or(235.0);
        let term = ctx.term;

        match ctx.kind {
            MemberKind::Beam => beam::check_beam(forces, sec, mat, ctx, f, term),
            MemberKind::Column => column::check_column(forces, sec, ctx, f, term),
            MemberKind::Brace => brace::check_brace(forces, sec, ctx, f, term),
        }
    }
}

/// テスト共通ヘルパー（`mat`/`rect_section`/`h_section`）。各サブモジュールの
/// テストから `super::super::test_support::*` として共有する。
#[cfg(test)]
pub(crate) mod test_support {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{Material, Section};
    use squid_n_core::section_shape::SectionShape;

    pub(crate) fn mat(name: &str) -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: name.to_string(),
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    pub(crate) fn rect_section(b: f64, d: f64, name: &str) -> Section {
        Section {
            id: SectionId(0),
            name: name.to_string(),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: d * b.powi(3) / 12.0,
            j: 0.0,
            depth: d,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }
    }

    /// `SectionShape::SteelH` 付きの断面（実寸 tf/tw を持つ正規経路の検証用）。
    pub(crate) fn h_section(h: f64, b: f64, tw: f64, tf: f64) -> Section {
        let shape = SectionShape::SteelH {
            height: h,
            width: b,
            web_thick: tw,
            flange_thick: tf,
        };
        shape.to_section(SectionId(0), format!("H-{}x{}x{}x{}", h, b, tw, tf))
    }
}
