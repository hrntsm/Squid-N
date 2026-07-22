# Zed 風レイアウト改修（ドック化＋イベントログ）

作成日: 2026-07-22
対象ブランチ: `feature/zed-style-layout`
関連文書: `../specs/UI設計.md`（§1 を本改修に合わせて改訂済み）

## 1. 目的

GUI の全体構成を Zed エディタ風の「開閉可能ドック＋下部バーのトグル」型へ改修する。

- 下部ステータスバーのアイコンで左右・下のパネルを開閉できる（左端 🗂＝左ドック・📜＝下ドック、右端 🔍＝右ドック）。
- 中央下部に処理ログ（イベントログ）専用の領域を新設し、1行しか出せなかった
  `last_error`/`last_notice` を時系列の履歴として追えるようにする。

## 2. 実装内容

### 2.1 レイアウト機構の置き換え（Stage 1）

`crates/squid-n-app/src/app/mod.rs` の `impl eframe::App for App::ui`:

- `available_rect_before_wrap()` の手動矩形分割＋ `allocate_ui_at_rect`（deprecated）＋
  自前ドラッグハンドルを全廃し、egui 標準パネルの `show_inside` 連続呼び出しへ置換。
  呼び出し順は top（ツールバー）→ bottom（ステータスバー）→ left → right →
  bottom（ログドック）→ CentralPanel。下ドックを左右ドックの間（中央領域の下部）に
  出すため、左右より後に呼ぶ必要がある。
- egui 0.34.3 では `TopBottomPanel`/`SidePanel` および `default_width` 等が deprecated
  （統合 API `egui::Panel` へ移行）のため、`egui::Panel::top/bottom/left/right` と
  `default_size`/`size_range`/`exact_size` を使用（clippy `-D warnings` 対策でもある）。
- `App::left_panel_width` フィールドは廃止（egui パネルの標準リサイズが幅記憶を担う）。
- `App` に `left_dock_open`（初期 true）/`right_dock_open`（初期 true）/
  `bottom_dock_open`（初期 false）を追加（gui feature 限定）。
- `status_bar`（panels.rs）を `&mut self` 化し、ドック開閉トグルを追加。右ゾーンの
  確保幅はサマリ文字幅に 🔍 トグル分（アイコン幅＋ボタン余白）を加える必要がある
  （不足すると左ゾーンのエラー表示と重なる）。

### 2.2 イベントログ（Stage 2）

- `LogLevel`/`LogEntry`/`EventLog` を mod.rs に新設（cfg なし＝ヘッドレスでも保持）。
  上限 1000 件、時刻はアプリ起動からの経過時間（std のみではローカル壁時計の表記が
  できないため。表示「mm:ss」）。
- メッセージ発行を `App::report_error / report_notice / report_info`（actions.rs）へ一元化。
  crate 内の `last_error = Some(...)` / `last_notice = Some(...)` 直接代入は全て置換済み
  （残っているのは report_* 自身の実装 2 箇所のみ）。
- `report_error` は下ドックを自動で開く。ステータスバーの ⚠ 1行表示もクリックで
  下ドックが開く。
- 解析ジョブは開始時に「⏳ {label} を開始」、`poll_job` 成功時に「✅ {label} が完了
  ({:.1}s)」を Info でログ。失敗時は各 `apply_*` が report_error 経由になるため
  完了ログは出さない（二重ログ回避）。
- 単体テスト: `EventLog` の上限動作、`report_error` の last_error／ログ両反映
  （src/app/tests.rs）。

### 2.3 検証

CI 同条件＋ gui feature（非デフォルトのため `--workspace` では検証されない点に注意）:

- `cargo clippy --workspace --all-targets --locked -- -D warnings`
- `cargo clippy -p squid-n-app --features gui --all-targets -- -D warnings`
- `cargo test -p squid-n-app --features gui`（137 件）／ feature なし（96 件）
- `cargo fmt --all`

## 3. 第二弾（Stage 3〜5）の実装内容

1. **左ドックのパネル切替式化**（Stage 3）: `LeftPanel { Navigator, DrawTools }`。
   作成パレット（梁・壁・スラブ作成モード＋断面割当）はビューア上部から移設
   （表示モード・変形倍率などキャンバス直結の表示制御はビューアに残す）。
2. **編集テーブルの下ドック移設**（Stage 3）: `BottomTab { Log, Diagnostics, Model, Loads }`。
   モデル編集（ModelTab サブタブ群、新規/サンプルボタンはファイルメニューと重複のため削除、
   サブタブは horizontal_wrapped 化）と荷重編集を下ドックのタブへ。
   ステータスバーは「アイコン＝パネル」方式（`toggle_dock_icon`: アクティブなら閉じる／
   それ以外は開いてアクティブ化）。
3. **解析設定の右ドック移設**（Stage 4）: `RightPanel { Inspector, AnalysisSettings }`。
   `analysis_tab_panel` → `analysis_settings_panel` に改名し狭幅向けに再構成
   （セクションを CollapsingHeader 化・横並びを horizontal_wrapped 化・階の定義を
   「階名見出し＋折り返しパラメータ行」の 2 段化。ウィジェット・ロジックは不変）。
   右ドックは全体を縦スクロール化し幅 220〜560px。中央の解析タブは 3D ビュー。
4. **工程タブのプリセット化**（Stage 4）: `apply_tab_preset`。タブクリックと
   ジョブ成功時の自動遷移で、各工程に適したドック初期配置へ切り替える
   （以後の手動変更は妨げない）。
5. **診断タブ**（Stage 5）: `run_diagnostics`（モデル検証・支点なし・断面未割当
   [100 件超は集約]・空の地震ケース参照組合せ・空荷重ケース）。
   `Staleness::diagnostics_stale` による遅延評価（タブを開いた時点で stale なら実行）。
   対象付き診断行はクリックで 3D 選択・インスペクタへ反映。

## 4. 敵対的レビュー結果と対応（2026-07-22）

観点別 3 レビュー（状態遷移／移設等価性／egui 0.34 API 整合）を実施。
移設コード自体は変更前と完全一致（等価性レンズで一字単位の照合済み）。検出・修正した問題:

1. **作成モードの残留（高）**: 作成モードのトグル・ヒント・クリア処理が作成パレット内に
   しかなく、パレット非表示（ドック閉・パネル切替・工程タブのプリセット）後もモードが
   残り、3D クリックで無警告に部材が生成された。→ パレット非表示のフレームで
   `reset_draw_modes` により強制解除（可視性と発動可能性を一致させる）。
   `load_model` でも解除（旧モデルの節点 id 残留対策）。
2. **report_error がログタブへ切り替えない（中）**: 下ドックが診断・テーブル表示中だと
   エラー本文が見えなかった。→ `bottom_tab = Log` も設定。
3. **プリセットの再適用（中）**: 選択中タブの再クリックでもプリセットが走り、手動で
   閉じたドックが開き直った。→ 工程が実際に変わったときのみ適用。
4. **ジョブ成功判定の非対称（中）**: 成功時に last_error をクリアする apply_* と
   しないものが混在し、ジョブ実行中の無関係エラー（ファイル保存失敗等）で完了ログ・
   自動遷移が抑止された。→ 完了処理冒頭で last_error をクリアして対称化
   （エラーはイベントログに残る）。
5. **下ドック 4 タブの ScrollArea Id 衝突（中）**: id_salt 未指定のため 4 タブが同一 Id と
   なりスクロール位置が共有された（egui は ScrollArea の Id を出現順でなく
   ui.id×salt で決める）。→ タブごとに一意な id_salt を付与。

未対応（許容と判断）: ScrollArea::both 内の TableBuilder 二重スクロール（改修前からの
既存構成で panic はしない）、狭幅時のステータスバーアイコンのクリップ、
horizontal_wrapped 内の縦 separator の折返し見た目、解析設定の default_open の選定
（線形静的・荷重組合せのみ展開。よく使う実行系を最短で押せることを優先）。

## 5. 残課題

- ドックのドラッグ入替・中央のタブ分割まで狙う場合は `egui_dock` 導入を検討
  （固定ドック＋トグルだけなら標準パネルで足りる）。
- 診断の項目追加（節点座標の重複・宙に浮いた節点・材料未割当など）は
  `run_diagnostics` への追記で拡張できる。
- ステータスバーのアイコンは egui 同梱の絵文字フォント頼みのため、環境によって
  グリフが出ない場合はテキストラベル化を検討する。
