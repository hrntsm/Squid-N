//! 二次設計（保有水平耐力計算）。仕様 `specs/P7_二次設計.md`。
//!
//! 建築基準法施行令の二次設計規定（令82条の2〜82条の6等）に準拠し、二次設計の各節を独立したモジュールに分割する。
//!
//! - [`member_rank`][]: 部材ランク FA〜FD 判定（RC せん断余裕度・S 幅厚比の後方互換
//!   簡易判定、層 Ds 自動分類）。
//! - [`width_thickness`][]: 鋼構造規定の幅厚比表による S 部材ランク判定（構造規定の
//!   表そのものを実装した正式版。[`member_rank`] の簡易判定より優先して使うこと）。
//! - [`holding_capacity`][]: 保有水平耐力の層チェック統合（剛性率 Fs・偏心率 Fe・
//!   形状係数 Fes・Ds 値・Qud・Qun 判定）。
//! - [`stiffness_ratio`][]: 層間変形角・剛性率 Rs 算定用の柱層間変位・重心変位。
//! - [`eccentricity`][]: 偏心率の計算コア（武藤 D 値法の閉形式）＋実モデルからの
//!   略算抽出＋雑壁剛性評価（n 倍法）。
//! - [`eccentricity_analysis`][]: 偏心率の精算層（弾性応力解析結果に基づく柱の
//!   水平剛性・長期軸力による重心）。
//! - [`principal_axis`][]: 主軸角度の算定。
//! - [`rc_capacity`][]: RC 矩形断面の簡易終局耐力算定（`squid_n_core::rc_capacity`
//!   の再エクスポート）。
pub mod eccentricity;
pub mod eccentricity_analysis;
pub mod holding_capacity;
pub mod member_rank;
pub mod principal_axis;
pub mod rc_capacity;
pub mod stiffness_ratio;
pub mod width_thickness;
