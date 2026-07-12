#![allow(clippy::needless_range_loop)]

//! 要素（エレメント）クレート。
//!
//! 要素の種類ごとにモジュールを階層化している:
//!
//! - [`common`] —  要素共通の基盤（振る舞いトレイト・局所行列/ベクトル・座標変換）
//! - [`frame`] —   線材要素（梁・トラス・集中ばね梁・ファイバー・MS・部材荷重）
//! - [`springs`] — ばね・パネル要素
//! - [`wall`] —    壁要素
//! - [`shell`] —   シェル要素
//! - [`factory`] — 要素データから振る舞いを生成するディスパッチャ
//!
//! 階層化前の平坦なモジュールパス（例: `squid_n_element::beam`）は、
//! 下記の再エクスポートにより従来どおり利用できる。

mod common;
mod frame;
mod springs;
mod wall;

pub mod factory;
pub mod shell;

// 階層化後も従来のモジュールパス（例: `squid_n_element::beam` や
// クレート内部の `crate::behavior`）を維持するための再エクスポート。
pub use common::{behavior, transform};
pub use frame::{beam, concentrated, fiber_elem, member_load, ms, truss};
pub use springs::{damper, isolator, panel, shear_spring, spring};
pub use wall::{misc_wall, side_column, wall_panel};

pub use behavior::*;
pub use factory::*;
