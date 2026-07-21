# Squid-N

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

## ビルド・開発

ビルド、テスト、静的解析、機能フラグ、ドキュメントの手順は
[CONTRIBUTING.md](CONTRIBUTING.md) を参照。

```bash
# ワークスペース全体ビルド
cargo build --workspace
```

## ドキュメント

利用者向けドキュメント（計算根拠・理論・出典）は、
[mdBook](https://rust-lang.github.io/mdBook/) で構築したドキュメントサイトに集約している
（`main` への push で GitHub Pages に自動デプロイ）。ローカルでのプレビュー手順は
[CONTRIBUTING.md](CONTRIBUTING.md#ドキュメントサイトmdbook) を参照。

開発者向けドキュメントは `dev_docs/` に集約している（ドキュメントサイトには含めない）。
設計仕様は `dev_docs/specs/`、各要素・設計式の検証状況は `dev_docs/v_and_v/README.md`、
ロードマップ・申し送りは `dev_docs/ROADMAP.md`・`dev_docs/handoff/` を参照。

## ライセンス

MIT License (see [LICENSE](LICENSE))

Copyright (c) 2026 Hiroaki NATSUME
