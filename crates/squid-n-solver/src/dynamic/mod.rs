//! 動的解析。
//!
//! - [`timehistory`] — 時刻歴応答解析（Newmark-β 法）
//! - [`damping`] —     減衰モデル
//! - [`eigen`] —       固有値（モーダル）解析
pub mod damping;
pub mod eigen;
pub mod timehistory;
