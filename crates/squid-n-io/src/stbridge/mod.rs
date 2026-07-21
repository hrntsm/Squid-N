//! ST-Bridge（XML, 2.0.2）入出力。設計書 §12.5 / 仕様 `dev_docs/specs/P8_操作と連携.md` §7.1。
//!
//! 一次資料は ST-Bridge 公式スキーマ 2.0.2（`STBridge_v202.xsd`）。読み書きとも
//! **標準スキーマ準拠**を方針とする（他社の一貫計算・BIM とモデルを受け渡すための
//! フォーマットであり、独自方言は相互運用性を損なうため用いない）。
//!
//! # 対応範囲（意味的往復を保証する subset）
//! - **節点**（座標 `X`/`Y`/`Z`・所属層）、**層**（名称・標高・種別・所属節点）、
//!   **断面**（形状＝鋼各種・RC・SRC・CFT ＋形鋼ライブラリ `StbSecSteel`）、
//!   **部材**（柱・大梁・間柱・ブレース・スラブ・壁。向き `rotate`・端部 `condition_*`）、
//!   **材料**（グレード名。`StbModel` は材料テーブルを持たないため断面の `strength_*` で表す）。
//! - import→export→再import で上記が意味的に一致する（DoD §8.3）。
//! - 要素ごとの詳細な変換状況は利用者ドキュメント `docs/model_io.md`
//!   「ST-Bridge 要素別 変換状況一覧」を参照。
//!
//! # 材料（グレード名）
//! ST-Bridge 2.0 の `StbModel` は材料テーブル（E・ν・密度）を持たない。日本の構造材料は
//! 法令・JIS で規格化されており、グレード名（`Fc21`・`SN400B`・`SD345` 等）が決まれば物性は
//! 一意に定まる。import はグレード名から標準材料表（[`import`] の `material_std`）で物性を
//! 復元し、export は断面へグレード名（鋼 `strength_main`、コンクリート `strength_concrete`）を
//! 付す。材料の E/ν や節点荷重など幾何スコープ外は往復しない（完全一致は `.scz`）。
//!
//! # 非対応（仕様どおり対象外）
//! - 解析結果・独自属性、拘束条件・質量・荷重（ST-Bridge の幾何スコープ外）。
//! - 基礎・杭・開口・パラペット・通り芯（`StbAxes`）。取り込み時にデータを欠落させる
//!   未対応要素は [`ImportReport`] の警告で必ず通知する（未知要素も部材/断面グループの
//!   直属子なら要素名で通知。fail-loud）。
//!
//! # モジュール構成（1 ファイル 1 責務）
//! - [`export`] — 直列化（内部モデル → 標準 ST-Bridge XML）。
//! - [`section_std`] — 標準断面要素の直列化。
//! - [`import`] — パース（ST-Bridge XML → 内部モデル）。
//! - `tests` — 入出力の統合テスト（`#[cfg(test)]`）。

mod export;
mod import;
mod section_std;

pub use export::export_stbridge;
pub use import::{import_stbridge, import_stbridge_with_report, read_stbridge_file, ImportReport};

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

const STB_VERSION: &str = "2.0.2";

#[cfg(test)]
mod tests;
