//! RC 造の許容応力度と断面検定（RC 規準13〜18条・令82条による許容応力度計算）。
//!
//! 準拠する規準:
//! - 許容応力度・ヤング係数比: 2010年版 RC 規準・構造規定
//! - 梁の曲げ・せん断検定: RC 規準 13条
//! - 柱の軸力＋曲げ検定: RC 規準 14条
//!
//! # 実装方針（全体）
//! - `Section.shape` が `RcRect`/`RcCircle` でない場合（配筋情報なし）は
//!   検定をスキップし `ok=true` で返す（旧実装と同じフォールバック）。
//! - `Material.fc` が未設定/0 の場合も同様にスキップする。
//! - 梁は強軸曲げ（`mz`）とそれに対のせん断（`qy`）のみを検定する
//!   （RC 規準の梁断面検定の対象と一致）。
//! - 柱は軸力（M=0）・軸力＋二軸曲げ・二方向せん断を検定する。
//! - `MemberKind::Brace` は RC 部材としては未対応のため、梁の検定式で代用する。
//!
//! # モジュール構成（RC 規準の許容応力度検定の対象部材に対応）
//! - 本ファイル（`rc/mod.rs`）: 断面諸元の抽出・許容応力度のまとめ・
//!   せん断スパン比 α・せん断耐力・地震時設計用せん断力・`RcDesign`
//!   （`DesignCheck` 実装、梁/柱への振り分け）。
//! - [`beam`]: 鉄筋コンクリート造梁の断面検定（RC 規準13条の曲げ・せん断）。
//! - [`column`]: 鉄筋コンクリート造柱の断面検定（RC 規準14条の軸力+曲げ）。
//! - [`bond`]: 鉄筋コンクリート造梁付着の断面検定（RC 規準1999/1991 方式）。
//! - [`joint`]: 鉄筋コンクリート造柱梁接合部の断面検定（RC 規準15条）。
//! - [`wall`]: 鉄筋コンクリート造耐震壁の断面検定（RC 規準18条）。

use crate::{CheckResult, DesignCheck, DesignCtx, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

mod beam;
/// 鉄筋コンクリート造梁の非線形復元力特性（曲げトリリニア・せん断・軸）。
/// 非線形解析の材端バネ骨格に用いる（技術基準解説書「部材の復元力特性」）。
pub mod beam_nonlinear;
mod bond;
mod column;
/// 鉄筋コンクリート造水平接合面の検討（PCa 打継ぎ面のせん断検定）。
pub mod horizontal_joint;
pub mod joint;
pub mod wall;
/// 鉄筋コンクリート造耐震壁のせん断非線形特性（トリリニア Qc/βu/Qu）。
/// 非線形解析のせん断ばね骨格に用いる（技術基準解説書「耐震壁のせん断非線形特性」）。
pub mod wall_nonlinear;

// 共有ヘルパ（1 ファイル 1 責務で分割した非公開サブモジュール）。
mod allowable;
mod design_shear;
mod section_props;
mod shear_capacity;

pub use bond::{rc_beam_bond_check, rc_beam_bond_check_1991, Bond1991Result, BondCheckResult};
pub use wall_nonlinear::{
    wall_shear_beta_u, wall_shear_crack, wall_shear_trilinear, wall_shear_ultimate,
    WallShearTrilinear, WallShearTrilinearInput,
};

// 材料強度・許容応力度は `crate::material_strength`（RC 規準の材料強度・許容応力度）へ
// 集約した。RC 造の検定で用いるものを再エクスポートし、従来の
// `crate::rc::concrete_allowable_shear` 等のパスも維持する。
pub use crate::material_strength::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_compression_class,
    concrete_allowable_shear, concrete_allowable_shear_class, concrete_young_modulus,
    high_strength_group, high_strength_pw_cap, high_strength_w_ft, rebar_allowable_shear,
    rebar_allowable_tension, rebar_sigma_y, young_ratio_n, HighStrengthGroup,
};

// 分割した共有ヘルパを従来の `crate::rc::X`（他モジュール）・`super::X`
// （rc 直下の兄弟モジュール）パスで参照できるよう再エクスポートする。
pub(crate) use allowable::*;
pub(crate) use design_shear::*;
pub(crate) use section_props::*;
pub(crate) use shear_capacity::*;

// ============================================================================
// 8. DesignCheck 実装（梁は rc/beam.rs、柱は rc/column.rs へ振り分け）
// ============================================================================

pub struct RcDesign;

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let fc_raw = mat.fc.unwrap_or(0.0);
        if fc_raw <= 0.0 {
            return CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "RC 検定: Fc 未設定".to_string(),
                detail: "Material.fc が None/0 です。コンクリート強度を設定してください。"
                    .to_string(),
                components: Vec::new(),
            };
        }

        let shape = match &sec.shape {
            Some(s @ SectionShape::RcRect { .. }) => s,
            Some(s @ SectionShape::RcCircle { .. }) => s,
            _ => {
                return CheckResult {
                    ratio: 0.0,
                    ok: true,
                    basis: "RC 検定: 配筋情報なし".to_string(),
                    detail: "Section.shape が RcRect/RcCircle ではないため検定をスキップしました。"
                        .to_string(),
                    components: Vec::new(),
                };
            }
        };

        match ctx.kind {
            MemberKind::Beam | MemberKind::Brace => {
                beam::beam_check(forces, sec, mat, ctx, shape, fc_raw)
            }
            MemberKind::Column => column::column_check(forces, sec, mat, ctx, shape, fc_raw),
        }
    }
}

// ============================================================================
// テスト（断面諸元・許容応力度・せん断耐力・地震時せん断力・RcDesign 統合系）
// ============================================================================

#[cfg(test)]
mod tests;
