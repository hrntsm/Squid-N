#![allow(clippy::needless_range_loop)]
#![allow(clippy::single_match)]
#![allow(clippy::identity_op)]

//! 解析ソルバークレート。
//!
//! 解析の種類ごとにモジュールを階層化している:
//!
//! - [`common`] —    解析共通の基盤（組み立て・拘束・状態スナップショット）
//! - [`statics`] —   静的解析（線形静的・地震/風荷重・施工時）
//! - [`nonlinear`] — 非線形（漸増）静的解析（プッシュオーバー・弧長法・耐力喪失）
//! - [`dynamic`] —   動的解析（時刻歴・減衰・固有値）
//!
//! 階層化前の平坦なモジュールパス（例: `squid_n_solver::pushover`）は、
//! 下記の再エクスポートにより従来どおり利用できる。

mod common;
mod dynamic;
mod nonlinear;
mod statics;

// 階層化後も従来のモジュールパス（例: `squid_n_solver::pushover` や
// クレート内部の `crate::constraint`）を維持するための再エクスポート。
pub use common::{assemble, constraint, transaction};
pub use dynamic::{damping, eigen, timehistory};
pub use nonlinear::{arc_length, pushover, strength_loss};
pub use statics::{analysis, construction, linear};
