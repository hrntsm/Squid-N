//! 解析共通の基盤モジュール。
//!
//! - [`assemble`] —    全体剛性・質量・荷重ベクトルの組み立て
//! - [`constraint`] —  拘束条件（自由度縮約）
//! - [`transaction`] — 全要素確定状態のスナップショット
pub mod assemble;
pub mod constraint;
pub mod transaction;
