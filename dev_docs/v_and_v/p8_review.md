# P8（操作・連携：MCP・ST-Bridge・GUI拡充）監査レポート

**監査日:** 2026-06-22
**対象:** `crates/squid-n-mcp`・`crates/squid-n-io/src/stbridge.rs`・`crates/squid-n-edit`・`crates/squid-n-app`／`dev_docs/specs/P8_操作と連携.md`
**結論:** P8 は「実装」とコミットされているが、**MCP は `--features mcp` でコンパイル不能**、
ST-Bridge は未実装、MCP ツールの中身は大半がスタブ。DoD §8.1〜§8.3 はいずれも未達。

---

## 1. 完了報告との乖離

- コミット `f1b6c4d "P8 操作・連携 実装と dev_docs/specs/P8 の整合性調整"` は「実装」と称している。
- V&V README の索引には P8 の行が無く（明示の ✅ 虚偽記載は無い）、しかし「実装済み」の体裁で main にある。
- **実態:** 下表のとおり MCP サーバは非デフォルト機能 `mcp` の下でコンパイルすら通らず（13 エラー）、
  通常ビルド・テストで一度も検証されていない（P7 の `p7` と同じ rot 罠）。テストは squid-n-mcp・stbridge とも **0 件**。

---

## 2. タスク別の実装状況（監査時点）

| ID | タスク | 仕様の要求 | 実態 | 判定 |
|----|--------|-----------|------|------|
| T0 | squid-n-mcp 雛形（rmcp） | stdio サーバ起動・tools/list | **`--features mcp` で 13 エラー（コンパイル不能）**。rmcp 1.7 API に対し腐敗。非デフォルトで未検証 | ❌ |
| T1 | 読取系ツール | model.load/query, result.get | `model_query` は**常に空** `items: vec![]` を返すスタブ（引数・モデル未使用）。`result.get`・`model.load` **未実装** | ❌ |
| T2 | 書込系（単一ライタ） | model.edit/save, EditCommand 経由, op_id | **未実装**。`squid-n-edit` の EditCommand/UndoStack 自体は存在（P3 で実装）するが MCP から呼んでいない | ❌ |
| T3 | 非同期ジョブ | run→即JobId→進捗notify→result | `JobRegistry` の型と登録のみ。`analysis_run` は**ジョブを Queued 登録するだけ**で実計算も tokio タスクも進捗通知も無し。ジョブは永遠に Queued | ❌ |
| T4 | design/report ツール | design.check, report.export | **未実装** | ❌ |
| T5 | ST-Bridge 入出力 | import/export 意味的往復 | `import_stbridge`/`export_stbridge` とも **"not yet implemented" エラーを返すだけ** | ❌ |
| T6 | GUI 拡充 | 全解析種別の起動・可視化 | P3 GUI に固有値・時刻歴ビューは一部あり。MCP ジョブ連携・網羅は未了 | 🔶 |

### 2.1 `--features mcp` の主なコンパイルエラー（13件）

- `rmcp::handler::server::tool::tool_router` 等の **import パスが rmcp 1.7 と不一致**（E0432, E0603）。
- `SquidNServer::tool_router()` が見つからない（マクロ展開がエラーで止まっている。E0599）。
- **`ServerState` 内の `Box<dyn ResultStore>`・`Box<dyn EditCommand>` が `Send` でない** ため
  `Arc<Mutex<ServerState>>` を rmcp サーバ（Send 要求）に渡せない（E0277）。設計レベルの問題。
- `ErrorCode::InvalidParams` 不在、`CallToolResult::error` の引数数、`ServerInfo`(=`InitializeResult`)に
  `name`/`version` フィールド無し・非網羅構造体（E0599, E0061, E0560, E0639）。

---

## 3. 実際に動いている部分（壊れていない）

- **`squid-n-edit`（EditCommand/UndoStack）**: P3 フォローアップで `squid-n-edit` に統一済み。488 行・実装あり。
- **`JobRegistry`/`Job`/`JobStatus`/`JobKind`**: 型と register/get/update は健全（ただし誰も実行しない）。
- **`analyze()`/`get_model_json()`**: feature 非依存の自由関数。`analyze` は実際に線形静的を解く（が MCP
  ツールには未接続）。
- **GUI（squid-n-app）**: 固有値・時刻歴の起動とビューは一部存在。

---

## 4. 構造計算プログラムとしての評価

MCP は「AI/他ツールからモデル操作・解析・結果取得」を可能にする外部 I/F で、P8 の中核価値。現状は
**サーバが起動できない（コンパイル不能）**ため、DoD §8.1（Inspector で一連動作）は原理的に未達。
ST-Bridge は国内一貫プログラム/BIM 連携の要だが**完全未実装**。よって P8 は名目「実装」だが
**機能としては未成立**。P7 と同様、(1) 非デフォルト機能の rot、(2) スタブのまま「実装」コミット、
という二重の問題を抱える。

---

## 5. 本ブランチ（feat/p8-verification）での是正

1. 本レポート作成、V&V README に P8（#23/#24/#25）を正直に記載。
2. 動く中核ロジック `query_model`（node/member/section + フィルタ）を feature 非依存関数として
   **実装・テスト**（MCP ツール `model_query` も呼ぶよう更新）。`JobRegistry` のライフサイクルテスト追加。
3. **ST-Bridge 2.0 subset の意味的往復を実装**（オーナー判断で優先）。`export_stbridge`/`import_stbridge`
   が節点・層・材料・断面・部材（柱/大梁）・節点荷重を往復。export 冪等・再import安定・取込モデルの
   `validate()` をテスト（DoD §8.3 を subset 範囲で達成）。
   - 対応: 上記スコープ。**非対応**: 結果・拘束・質量・床/ブレース・剛域。断面は形鋼ライブラリ参照でなく
     物性直持ち（StbSecRaw）の subset。他社ソフトとの完全相互運用は将来。

## 6. なお残る大物（オーナーのコスト判断・後続）

- **`--features mcp` のコンパイル復旧**（rmcp 1.7 API 追従＋`ResultStore`/`EditCommand` の `Send` 化）。
- **MCP ツールの本実装**（model.edit 単一ライタ・result.get・design.check・report.export）。
- **非同期ジョブの実行・進捗通知**（tokio タスク＋notification）。
- ST-Bridge の **形鋼ライブラリ参照（実スキーマ完全準拠）** と床/ブレース等への対応拡大。

> 現状: ST-Bridge は subset で意味的往復 🔶。MCP はコンパイル不能のまま（DoD §8.1〜§8.2 未達 ❌）。
> P8 全体としては「ST-Bridge subset 達成・MCP 未達」。
