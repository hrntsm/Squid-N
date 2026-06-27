# Squid-N

> 読み: スクイッダン

日本の建築構造計算一貫プログラム。
Rust で実装。

## アーキテクチャ

15 のクレートから成る階層型アーキテクチャ:

```
Layer 0: squid-n-core（基本データ構造・DOF管理）、squid-n-math（疎行列・ソルバ）
Layer 1: squid-n-material（一軸材料履歴則）
Layer 2: squid-n-section（断面性能算定）
Layer 3: squid-n-element（梁・板・パネルゾーン要素）
Layer 4: squid-n-skeleton（スケルトン曲線）、squid-n-load（Ai分布、床荷重、荷重組合せ）、squid-n-solver（各種解析）
Layer 5: squid-n-design-jp（日本仕様設計計算）、squid-n-io（結果I/O）、squid-n-edit（編集トランザクション）
Layer 6: squid-n-mcp（MCP サーバ）、squid-n-app（GUI アプリケーション）
```

依存方向は上層から下層のみ。循環依存は `cargo run -p xtask -- check-deps` で検出する。

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
| `gui` | squid-n-app | GUI（egui/eframe） |
| `mcp` | squid-n-mcp | MCP サーバ |
| `gpu` | squid-n-gpu | GPU 行列演算（P10、実装中） |
| `ml` | squid-n-ml | ML 断面提案（P11、未実装） |
| `p7` | squid-n-design-jp | 二次設計（Ds、偏心率、保有耐力、パネルせん断）。既定で有効 |
| `p12` | squid-n-design-jp | 限界耐力計算（容量スペクトル法）。未実装・opt-in |

GPU や ML を無効化しても解析機能は CPU で動作する。

## V&V

各要素・設計式の検証状況は `docs/v_and_v/README.md` を参照。

## ライセンス

MIT License (see [LICENSE](LICENSE))

Copyright (c) 2026 Hiroaki NATSUME
