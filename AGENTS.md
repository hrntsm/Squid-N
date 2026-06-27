# Squidcan

Rustで作成する日本の建築構造計算一貫プログラム
日本での利用を想定しており、コミットメッセージやコード中のコメント、プロンプトに対する応答などは日本語で行うこと。

## 開発コマンド

### ビルド

```bash
# ワークスペース全体ビルド
cargo build --workspace

# リリースビルド
cargo build --workspace --release
```

### テスト実行

```bash
# 全テスト実行
cargo test --workspace
```

### 静的解析

コミット前には必ず確認すること。CI と同条件で実行する（`--all-targets` がないとテストコードが clippy の対象外になる）。

```bash
cargo clippy --workspace --all-targets --locked -- -D warnings

cargo fmt --all -- --check
```
