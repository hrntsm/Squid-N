//! 動的解析。
//!
//! - [`timehistory`] — 時刻歴応答解析（Newmark-β 法）
//! - [`damping`] —     減衰モデル
//! - [`eigen`] —       固有値（モーダル）解析
//! - [`lumped_mass`] — 質点系（串団子）モデルの生成
//! - [`phase_diff`] —  位相差入力（ねじれ加振）の生成
pub mod damping;
pub mod eigen;
pub mod lumped_mass;
pub mod phase_diff;
pub mod timehistory;
