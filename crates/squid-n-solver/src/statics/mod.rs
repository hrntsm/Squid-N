//! 静的解析。
//!
//! - [`linear`] —       線形静的解析
//! - [`analysis`] —     地震・風の静的荷重生成と解析設定
//! - [`construction`] — 施工時解析（施工段階解析）
pub mod analysis;
pub mod construction;
pub mod linear;
