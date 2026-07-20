//! ST-Bridge（XML, 2.0 系）入出力。設計書 §12.5 / 仕様 `specs/P8_操作と連携.md` §7.1。
//!
//! # 対応範囲（意味的往復を保証する subset）
//! - **節点**（座標・所属層）、**層**（名称・標高・所属節点）、**材料**（E・ν・密度・Fc・Fy）、
//!   **断面**（物性、または形状＝鋼各種・RC・SRC・CFT）、**部材**（柱・大梁・間柱・ブレース・
//!   スラブ・壁）、**荷重ケース**（節点荷重）。
//! - import→export→再import で上記が意味的に一致する（DoD §8.3）。
//! - 要素ごとの詳細な変換状況は利用者ドキュメント `docs/model_io.md`
//!   「ST-Bridge 要素別 変換状況一覧」を参照（一次資料は ST-Bridge 公式スキーマ 2.0 系）。
//!
//! # 非対応（仕様どおり対象外）
//! - 解析結果・独自属性（§12.5）、拘束条件・質量（ST-Bridge の幾何スコープ外）。
//! - 基礎・杭・開口・パラペット・通り芯、部材/面/温度荷重・荷重組合せ。
//!   取り込み時にデータを欠落させる未対応要素は [`ImportReport`] の警告で必ず通知する
//!   （手動リスト外の未知要素も、部材/断面/荷重の直属子なら要素名で通知。fail-loud）。
//! - **既定（`Raw`）の書き出し断面**は実 ST-Bridge の形鋼ライブラリ参照ではなく、内部モデルの
//!   物性をそのまま持つ `StbSecRaw` で表現する（正準モデルを唯一の真実とする方針）。BIM/他ソフト
//!   向けに標準要素で書き出す `Standard` モードは下記「断面書き出しモード」を参照。import は
//!   `StbSecRaw` と標準断面要素（`StbSecColumn_S` 等）の双方を読み取れる。
//!
//! # 断面書き出しモード
//! [`export_stbridge`] は既定で `StbSecRaw`（物性直持ち・往復可能）を書き出す。
//! [`export_stbridge_with`] に [`SectionExportMode::Standard`] を渡すと、ST-Bridge 標準の
//! 断面要素（`StbSecColumn_S` 等）＋形鋼ライブラリ（`StbSecSteel`）で書き出す（BIM/他ソフト向け）。
//!
//! # モジュール構成（1 ファイル 1 責務）
//! - [`export`] — 直列化（内部モデル → ST-Bridge XML）。
//! - [`section_std`] — 標準フォーマット断面の直列化（`Standard` モード）。
//! - [`import`] — パース（ST-Bridge XML → 内部モデル）。
//! - `tests` — 入出力の統合テスト（`#[cfg(test)]`）。

mod export;
mod import;
mod section_std;

pub use export::{export_stbridge, export_stbridge_with};
pub use import::{import_stbridge, import_stbridge_with_report, read_stbridge_file, ImportReport};

/// ST-Bridge 書き出し時の断面表現モード。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SectionExportMode {
    /// 物性を独自要素 `StbSecRaw` として直接保持する。`import_stbridge` で往復可能。既定。
    #[default]
    Raw,
    /// ST-Bridge 標準の断面要素（鋼 `StbSecColumn_S`/`StbSecBeam_S`、RC `StbSecColumn_RC`/
    /// `StbSecBeam_RC`、CFT `StbSecColumn_CFT`、SRC `StbSecColumn_SRC`/`StbSecBeam_SRC`）＋
    /// 形鋼ライブラリ（`StbSecSteel`）で書き出す。BIM/他ソフトとの連携向け。形状
    /// （`Section.shape`）を持たない断面や耐震壁は `StbSecRaw` へフォールバックする。
    ///
    /// `import_stbridge` は本モードのファイル（および同じ断面表現の他社ファイル）を
    /// 読み戻せる（形鋼名から形状を復元し断面性能を再算定する。RC/SRC は配筋も
    /// `StbSecBarArrangement*` で往復する）。ただし柱・梁で共有していた断面は書き出し時に
    /// 2 断面へ分割される。CFT は柱のみ対応で梁に使うと `StbSecRaw` へ、RC 円形も梁では
    /// `StbSecRaw` へフォールバックする（形状・配筋は往復しない）。配筋を持たない
    /// （幾何のみの）他社ファイルは無筋相当で読む。
    Standard,
}

#[derive(Debug, thiserror::Error)]
pub enum StbError {
    #[error("xml parse: {0}")]
    Parse(String),
    #[error("unsupported version: {0}")]
    Version(String),
    #[error("unmappable element: {0}")]
    Unmappable(String),
    #[error("read: {0}")]
    Io(String),
    #[error("decode: {0}")]
    Decode(String),
}

const STB_VERSION: &str = "2.0.0";

#[cfg(test)]
mod tests;
