# Squidcan

日本の建築構造計算一貫プログラム。Rust で実装。

## 免責事項

本ソフトウェアは建築構造設計の補助ツールであり、算定結果の最終確認・判断は
**構造設計を担当する技術者**の責任において行ってください。本ソフトウェアの
利用により生じたいかなる損害についても、作者は責任を負いません。

**本ソフトウェアは v1.0 開発中であり、検証段階のコードを含みます。**
設計実務での使用には十分な検証と技術者の判断が必要です。

## 準拠規準

本ソフトウェアが準拠する日本建築学会規準・国土交通省告示の版は以下を参照:
`docs/v_and_v/README.md`

## アーキテクチャ

14 のクレートから成る階層型アーキテクチャ:

```
Layer 0: sc-core（基本データ構造・DOF管理）, sc-math（疎行列・ソルバ）
Layer 1: sc-material（一軸材料履歴則）
Layer 2: sc-section（断面性能算定）
Layer 3: sc-element（梁・板・パネルゾーン要素）
Layer 4: sc-skeleton（スケルトン曲線）, sc-load（Ai分布・床荷重）, sc-solver（各種解析）
Layer 5: sc-design-jp（日本仕様設計計算）, sc-io（結果I/O）
Layer 6: sc-mcp（MCP サーバ）, sc-app（GUI アプリケーション）
```

依存方向は上層→下層のみ。循環依存は `xtask check-deps` でチェック。

## ビルド

```bash
# ワークスペース全体ビルド
cargo build --workspace

# リリースビルド
cargo build --workspace --release
```

## テスト

```bash
# 全テスト実行
cargo test --workspace

# 決定性テスト（100回ビット一致確認を含む）
cargo test --workspace deterministic

# 依存方向チェック
cargo run -p xtask -- check-deps
```

## 静的解析

```bash
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo fmt --all -- --check
```

## 機能フラグ

| フラグ | 対象クレート | 内容 |
|--------|-------------|------|
| `gui` | sc-app | GUI（egui/eframe） |
| `mcp` | sc-mcp | MCP サーバ |
| `gpu` | sc-gpu | GPU 高速化（v2.0） |
| `ml` | sc-ml | ML 断面提案（v2.0） |

GPU/ML 機能が無くても全解析機能は動作する（CPU フォールバック）。

## V&V

各要素・設計式の検証状況は `docs/v_and_v/README.md` を参照。

## ライセンス

MIT License (see [LICENSE](LICENSE))

Copyright (c) 2026 Hiroaki NATSUME
