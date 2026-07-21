# P3 最小UI／設計 実装仕様書

**対象フェーズ:** v0.5 プロトタイプ / P3 最小UI/設計（設計書 §18）。**これで v0.5 完結。**
**対象読者:** Rust ジュニアエンジニア（egui/wgpu 経験は不要なように手順化）
**親文書:** `../構造計算一貫プログラム_実装設計書.md`（以降「設計書」。`§x.y` はその章番号）
**先行フェーズ:** [P0](P0_基盤.md) / [P1](P1_線形要素.md) / [P1.5](P1.5_板要素.md) / [P2](P2_線形解析と荷重.md)
**前提環境:** Rust stable 1.8x 以降 / `egui`・`egui_extras`・`eframe` / `wgpu`

---

## 0. このフェーズについて

### 0.1 目的

構造技術者が**表入力でモデルを作り、解析を実行し、結果と一次設計（許容応力度）の判定を確認できる**
最小GUIを作る。設計書 §14 / §11.1。

- **squid-n-app（egui）**：テーブル入力（節点・部材・断面・荷重）、3Dビューア、CMQ図／応力図、解析実行・結果閲覧、検定比の色分け表。
- **squid-n-design-jp**：**許容応力度計算（一次設計）**。危険断面位置（P1 §6.2.3）の内力に対し、長期・短期の許容応力度で**検定比**を出す。

### 0.2 完了像（ゴール）

設計書 §18 P3 完了基準「**一次設計が一通り通る**」を次で判定:

1. **表入力**：節点・部材・断面・荷重をテーブルで作成・編集でき、`squid-n-core::Model` に反映される（§3）。
2. **解析実行**：GUI から P2 の `Analysis`（線形静的・固有値・Ai一気通貫）を起動し結果を閲覧（§4）。
3. **可視化**：3Dで形状・変形図・モード形、部材の N/Q/M 図、大梁の CMQ 図を表示（§5）。
4. **許容応力度**：代表断面で検定比が手計算（規準例題）と一致し、色分け表で一覧（§6, §7）。
5. **編集トランザクション**：表編集が Undo/Redo でき、編集コマンドが MCP 編集API（P8）と共通の経路（§3.3）。

### 0.3 スコープ境界（含む／含まない）

| 項目 | P3 で | 備考 |
|---|---|---|
| egui テーブル入力（節点/部材/断面/荷重） | **含む** | §3 |
| 編集トランザクション（Undo/Redo・コマンド） | **含む** | §3.3。MCP と共通化の土台 |
| 3Dビューア（形状・変形・モード形） | **含む** | §5 |
| 応力図（N/Q/M）・CMQ図 | **含む** | §5.3。設計書 §14/R10 |
| 解析実行・結果閲覧（P2 連携） | **含む** | §4 |
| 許容応力度計算（RC/S 一次設計、検定比） | **含む** | §6。SRC は基本骨格のみ |
| 検定比の色分け一覧表 | **含む** | §7 |
| 保有水平耐力・Ds・限界耐力 | **含まない → P7/v2** | 二次設計は P7 |
| MCP サーバ本体・ST-Bridge | **含まない → P8** | 編集コマンドの**形**だけ P3 で用意 |
| 応力コンター・ヒンジ状態の可視化 | **一部 → P5/P7** | 非線形結果が出てから本格化 |
| 配布・インストーラ | **含まない → P9** | |

> **GUI フレームワーク（設計書 §14）:** 第一候補 **egui + egui_extras**。Tauri は代替として保留。
> P3 は egui で進める（PoC でテーブル編集の操作性・大量行スクロールを確認。判断は UI リード）。

---

## 1. タスク一覧と依存順序

```
T0 squid-n-app 雛形（eframe 起動・タブ枠）(§2)
   └─> T1 テーブル入力（節点/部材/断面/荷重）+ バリデーション (§3.1,§3.2)
          └─> T2 編集トランザクション（コマンド・Undo/Redo）(§3.3)
   └─> T3 解析実行・結果閲覧（P2 Analysis 連携）(§4)
   └─> T4 3Dビューア（wgpu：形状・変形・モード形）(§5.1,§5.2)
          └─> T5 応力図 N/Q/M・CMQ図 (§5.3)
T6 squid-n-design-jp DesignCheck トレイト + 許容応力度（RC/S）(§6)
   └─> T7 検定比 色分け一覧表（GUI）(§7)
T0..T7 ─> T8 テスト・DoD (§8)
```

| ID | タスク | クレート | 依存 |
|---|---|---|---|
| T0 | squid-n-app 雛形（eframe・タブ） | squid-n-app（新規） | P0 |
| T1 | テーブル入力＋バリデーション | squid-n-app | T0 |
| T2 | 編集トランザクション（Undo/Redo） | squid-n-app | T1 |
| T3 | 解析実行・結果閲覧 | squid-n-app | T0, P2 |
| T4 | 3Dビューア（形状・変形・モード形） | squid-n-app | T0, P2 |
| T5 | 応力図 N/Q/M・CMQ図 | squid-n-app | T4, P1/P2 |
| T6 | DesignCheck＋許容応力度（RC/S） | squid-n-design-jp（新規） | P1 §6.2.3 |
| T7 | 検定比 色分け表 | squid-n-app | T6 |
| T8 | テスト・DoD | 全体 | 上記 |

> **★UI 構成は [UI設計.md](UI設計.md) で確定（本フェーズで土台を作る）。** 下記の対応で実装する
> （いつ何をやるかは UI設計 §9.2 のスケジュール）:
> - **UI-D1（T0 の前段・前提）**：`squid-n-core`／`squid-n-section` に `SectionShape`/`RcRebar` 等を新設し
>   `to_section`（鋼H形・RC矩形）で A/I/J/As 算定（UI設計 §4.2）。これが無いと断面作成UIが作れない。
> - **UI-1〜UI-2**：T0 を **4ペイン＋工程タブ**（Model/Loads/Analysis/Results/Design/Report）へ。
>   タブは `Nodes/Members/…` ではなく**工程ベース**（UI設計 §1）。ナビゲータ・3D/表/ナビ連動を追加。
> - **UI-3**：T1 に**断面作成UI**（鋼H形・RC矩形＋配筋・プレビュー、UI設計 §4）。
> - **UI-4**：T2 に**3D選択→断面編集（共有編集＋複製）**（UI設計 §3）。
> - **UI-5**：T1/T2 にコピペ・一括生成・即時バリデーション（UI設計 §7）。
> - **UI-6**：T3 に **stale 表示＋手動再計算トリガ**（UI設計 §5）。
> - **UI-7**：T5/T7 に変形図・N/Q/M・CMQ・**検定比色分け**（UI設計 §6）。

---

## 2. T0: squid-n-app 雛形

`eframe`（egui のアプリ枠）で起動し、タブUIの骨格を作る。

```
squid-n-app/src/
├── main.rs        # eframe 起動
├── app.rs         # App 状態（Model・解析結果・選択状態・Undoスタック）
├── tables/        # 節点/部材/断面/荷重 テーブル（§3）
├── viewer/        # 3Dビューア（§5）
├── command.rs     # 編集トランザクション（§3.3）
└── design_view.rs # 検定比表（§7）
```

```rust
// squid-n-app/src/app.rs
pub struct App {
    pub model: squid_n_core::model::Model,
    pub results: Option<ResultsBundle>,     // P2 の解析結果を保持（下記定義）
    pub selection: Selection,               // 選択中の節点・部材（下記定義）
    pub undo: squid_n_edit::UndoStack,           // §3.3（squid-n-edit クレートに統一）
    pub active_tab: Tab,
}
pub enum Tab { Nodes, Members, Sections, Loads, Viewer, Design }

/// 解析結果のまとめ。GUI が表・図・検定で参照する。
pub struct ResultsBundle {
    /// 荷重ケース/組合せごとの静的結果（P2 §3 の StaticOnce）。
    pub statics: Vec<(squid_n_core::ids::LoadCaseId, squid_n_solver::linear::StaticOnce)>,
    /// 固有値結果（P2 §4 の ModalResult）。固有値未実行なら None。
    pub modal: Option<squid_n_solver::eigen::ModalResult>,
    /// 部材ごと・ケースごとの復元内力（P1 §6.2.3 の MemberForces）。N/Q/M・CMQ 図が読む。
    pub member_forces: Vec<(squid_n_core::ids::ElemId, squid_n_element::beam::MemberForces)>,
    /// 検定結果（P3 §6 の CheckResult）。位置別・組合せ別。検定比表（§7）が読む。
    pub checks: Vec<(squid_n_core::ids::ElemId, f64 /*pos*/, squid_n_design_jp::CheckResult)>,
}

/// 選択状態（3D ピック ↔ テーブル 連動）。
#[derive(Default)]
pub struct Selection { pub nodes: Vec<squid_n_core::ids::NodeId>, pub members: Vec<squid_n_core::ids::ElemId> }

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        // 上部タブ → 各タブ描画。左ペインに3D、右に表、など（レイアウトは UI リード調整可）
    }
}
```

**eframe 0.34.3 の実 API（確認済み・そのまま使える）。** 起動は `eframe::run_native`:

```rust
// squid-n-app/src/main.rs
fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    // 0.34.3: 第3引数のクロージャは Result<Box<dyn App>, DynError> を返す
    eframe::run_native(
        "Squid-N",
        options,
        Box::new(|_cc| Ok(Box::new(squid_n_app::app::App::default()))),
    )
}
```

> **★eframe 0.34.3 の `App` trait変更（重要）:** `update` は deprecated になり、
> **`ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame)` が必須メソッド**になった。
> `ui` メソッドは `&mut egui::Ui` を受け取るため、パネルは `show_inside(ui, ...)` を使う
> （従来の `show(ctx, ...)` は deprecated）。タブ切替・Undo/Redo ボタン・解析実行ボタンは
> `ui.horizontal(|ui| { ... })` で配置し、各タブの内容は `egui::ScrollArea::vertical().show(ui, ...)` で描く。

```rust
// squid-n-app/src/app.rs（eframe::App impl の要点）
#[cfg(feature = "gui")]
impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // 上部ツールバー: タブ + Undo/Redo + 解析実行 + 荷重継続性
        ui.horizontal(|ui| { /* selectable_label / Button */ });
        ui.separator();
        // 各タブの内容
        egui::ScrollArea::vertical().show(ui, |ui| match self.active_tab {
            Tab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            /* ... */
        });
    }
}
```

**DoD（T0）:** `cargo run -p squid-n-app --features gui` でウィンドウが開き、タブ切替できる。
（feature `gui` は P0 §2.4 の opt-in。コア解析は GUI 無しでビルド可能を維持。）
**バイナリ:** `[[bin]] name = "Squid-N" required-features = ["gui"]` を `squid-n-app/Cargo.toml` に設定。

---

## 3. T1/T2: テーブル入力と編集トランザクション

設計書 §14.1/§14.2。**構造技術者が慣れた表（スプレッドシート）入力**を `egui_extras::TableBuilder` で。

### 3.1 テーブル（節点の例）

```rust
// squid-n-app/src/tables/nodes.rs
pub fn nodes_table(ui: &mut egui::Ui, app: &mut crate::app::App) {
    use egui_extras::{TableBuilder, Column};
    let n = app.model.nodes.len();
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())            // ID
        .columns(Column::initial(80.0), 3) // X, Y, Z
        .column(Column::auto())            // 拘束
        .header(20.0, |mut h| {
            for t in ["ID","X","Y","Z","拘束"] { h.col(|ui| { ui.strong(t); }); }
        })
        // ★仮想スクロール（egui_extras 0.34 の確定 API）：表示中の行だけ描く
        .body(|body| {
            body.rows(18.0, n, |mut row| {
                let i = row.index();                   // 0.34: row.index() で行番号
                let node = &app.model.nodes[i];
                row.col(|ui| { ui.label(node.id.0.to_string()); });
                for k in 0..3 { row.col(|ui| { ui.label(format!("{:.1}", node.coord[k])); }); }
                row.col(|ui| { ui.label(format!("{:?}", node.restraint)); });
            });
        });
}
```

### 3.2 入力バリデーション（即時赤表示）

- 各セルの編集時に検証（数値範囲・参照ID存在・重複ID）。不正は**赤背景**で即フィードバック。
- 検証は `squid_n_core::Model::validate`（P0 §3.11）と同じ規則を**セル単位**で適用。

### 3.3 編集トランザクション（コマンド・Undo/Redo）★MCP と共通化の土台

設計書 §14.2：**表編集はすべて `squid-n-core::Model` への「コマンド（編集トランザクション）」として適用**し、
Undo履歴と MCP 編集API（P8）を共通化する。

> **所属クレート:** `EditCommand`/`UndoStack` は **`squid-n-edit` クレート**に置く。
> `squid-n-app`（GUI）と P8 の MCP サーバはいずれも `squid-n-edit` を参照し、同一の `UndoStack::run` 経路で
> モデルを変更する（単一ライタ＝整合。設計書 付録A R29）。`squid-n-app/src/command.rs` は廃止。

```rust
// squid-n-edit/src/lib.rs（GUI・MCP 共通の単一ライタ）
/// モデルへの編集を表すコマンド。apply で適用、invert で逆操作（Undo用）を返す。
pub trait EditCommand {
    fn apply(&self, model: &mut squid_n_core::model::Model) -> Box<dyn EditCommand>; // 返り値＝逆コマンド
    fn label(&self) -> &str;
}

// 例: 節点座標変更 / 部材追加 / 部材削除 …（参照不正時は Noop で安全にフォールバック）
pub struct SetNodeCoord { pub node: squid_n_core::ids::NodeId, pub coord: [f64;3] }
impl EditCommand for SetNodeCoord { /* apply: 旧値を保持した逆コマンドを返す */ }

pub struct UndoStack { done: Vec<Box<dyn EditCommand>>, undone: Vec<Box<dyn EditCommand>> }
impl UndoStack {
    pub fn run(&mut self, model: &mut squid_n_core::model::Model, cmd: Box<dyn EditCommand>) {
        let inv = cmd.apply(model); self.done.push(inv); self.undone.clear();
    }
    pub fn undo(&mut self, model: &mut squid_n_core::model::Model) { /* done.pop().apply → undone へ */ }
    pub fn redo(&mut self, model: &mut squid_n_core::model::Model) { /* 対称 */ }
}
```

> **設計意図:** P8 の MCP `model.edit` も**この `EditCommand` を生成して `UndoStack::run` に流す**。
> こうすると GUI 編集と MCP 編集が同一経路（単一ライタ。設計書 付録A R29）になり、整合する。

**DoD（T1/T2）:** 1万部材テーブルが滑らかにスクロール・編集可能（設計書 §14.2）。不正入力が赤表示。
編集→Undo→Redo でモデルが元に戻る／やり直せる。

---

## 4. T3: 解析実行・結果閲覧

P2 の `Analysis` を GUI から起動する。

```rust
// squid-n-app/src/app.rs（解析実行）
impl crate::app::App {
    fn run_linear_static(&mut self, lc: squid_n_core::ids::LoadCaseId) {
        let mut analysis = squid_n_solver::analysis::Analysis::prepare(&self.model).unwrap();
        let res = analysis.linear_static(lc).unwrap();
        self.results = Some(/* res を ResultsBundle に格納 */);
    }
    fn run_eigen(&mut self, n: usize) { /* analysis.eigen(n) */ }
    fn run_seismic(&mut self, dir: SeismicDir) { /* analysis.seismic_static(dir, AiMode::SemiPrecise) */ }
}
```

- 解析は時間がかかり得るので、**別スレッド＋進捗表示**（簡易で可。本格非同期は P8 の MCP ジョブ）。
- 結果は変位・反力・部材応力・固有周期・層せん断・層間変形をテーブル＋図で閲覧。

**DoD（T3）:** GUI から線形静的・固有値・地震（Ai一気通貫）を実行し、結果が表・図に出る。

---

## 5. T4/T5: 3Dビューア・応力図

設計書 §14.1。3D は **wgpu**（egui と同一GPUコンテキスト）が本格実装だが、
P3 では **egui Painter による簡易 2D 投影描画**（等角投影・ドラッグ回転/パン・ズーム）で
DoD（変形図・モード形・N/Q/M・CMQ 図が見える）を先に満たす。wgpu 本格統合は後続フェーズ。

### 5.1 描画基盤（T4）

- **P3 簡易実装:** `egui::Painter` で節点・部材を 2D 投影描画。カメラ（ドラッグ=パン/回転、ズーム）。
- **後続（wgpu 本格統合）:** `egui_wgpu` のコールバックでカスタム描画。節点・部材のピック（選択 → テーブルと連動）。

### 5.2 変形図・モード形（T4）

- 線形静的の変位、固有値のモード形を**倍率付き**で重ね描き（P3 簡易実装でも対応）。
- モード形はモード選択スライダー＋固有周期表示。

### 5.3 応力図・CMQ図（T5）— 設計書 §14/R10

- 部材ローカルに **N／Q／M 図**を描く（P1 §6.2.3 の応力復元・評価位置を使用）。
  P3 簡易実装: `MemberForces.at` の評価位置を部材軸に沿ってポリゴン描画。
- **CMQ 図**：各大梁の荷重項（両端固定端モーメント C/M・せん断 Q）を描く（P2 §5.1 の `Cmq`）。
  P3 簡易実装: `BeamLoad.cmq` の c_i/c_j（モーメント・青）と q_i/q_j（せん断・橙）をポリゴン描画。
  床荷重分配の検証用。**フェースモーメント位置**（P1 §6.7）も併記（後続）。

**DoD（T4/T5）:** 3Dで変形図・モード形が見える。単純梁で M 図が放物線、CMQ 図が荷重項の理論形。

---

## 6. T6: 許容応力度計算（squid-n-design-jp）

設計書 §11.1/§11.2。**一次設計**。危険断面位置（P1 §6.2.3）の内力に対し、長期・短期の許容応力度で検定。

**許容応力度（施行令90条等。法令＝公有。下記で確定）:**

```
■ 鋼材（令90条。F値に基づく）:
    長期 許容引張・圧縮・曲げ ft = F/1.5,   長期 許容せん断 fs = F/(1.5·√3)
    短期 = 長期 × 1.5（＝ ft_短期 = F,  fs_短期 = F/√3）
    F値[N/mm²]: SN400/SS400 = 235（板厚 t≤40mm）, SN490/SM490 = 325, …（令98条/告示の表）

■ コンクリート（令91条）:
    長期 許容圧縮 = Fc/3,   短期 = 2·Fc/3   （Fc=設計基準強度）
    長期 許容せん断は Fc により別式（令91条）

■ 異形鉄筋（令90条・令96条）:
    長期 許容引張 ft: SD295 = 195, SD345 = 215（D≤25）/195（D>25）, SD390 = 215 …（令90条の表）
    短期 = min(長期 × 1.5, F値)   ※上限は F 値（令96条）。SD295=295, SD345=345, SD390=390
        SD295: 195×1.5=292.5 < 295 ⇒ 292.5
        SD345: 215×1.5=322.5 < 345 ⇒ 322.5
        SD390: 215×1.5=322.5 < 390 ⇒ 322.5
```

> **★F値テーブル（令98条/告示。板厚 t≤40mm の代表値）は本書で確定（公有）:**
> SN400/SS400/SM400 = 235, SS490 = 285, SN490/SM490 = 325, SN520/SM520 = 355, SN550/SM570 = 450 [N/mm²]。
> 鉄筋 F値（令96条）: SD295=295, SD345=345, SD390=390。これらは外部データではなく法令由来で確定可能。
>
> **Category B（AIJ 私的規準）だけ外部データ:** 横座屈・局部座屈の許容曲げ低減、付着・定着の詳細式は
> AIJ 規準（RC計算規準／鋼構造設計規準）にあり著作権の対象 → ライセンス下で外部データ入力。
> 上の**令由来の許容応力度（公有）と σ=M/Z 等の力学量は本書で確定**（調査不要）。

### 6.1 トレイト（設計書 §11.2）

```rust
// squid-n-design-jp/src/lib.rs
pub trait DesignCheck {
    /// 危険断面の内力・断面・材料・文脈から検定結果を返す。
    fn check(&self, forces: &MemberForcesAt, sec: &squid_n_core::model::Section,
             mat: &squid_n_core::model::Material, ctx: &DesignCtx) -> CheckResult;
}

/// ある評価位置1点の内力。
///
/// ★単位規定（プログラム全体と共通・N·mm 系）:
///   n: 軸力 [N], q: せん断力 [N], m: 曲げモーメント [N·mm], pos: 無次元位置 0..1
/// 許容応力度は [N/mm²] で与えられるため、応力算定は σ = M[N·mm] / Z[mm³] のように
/// 単位を N·mm 系で揃えること。M が kN·m で入ってくる場合は 1e6 を掛けて N·mm に変換してから渡す。
pub struct MemberForcesAt { pub pos: f64, pub n: f64, pub q: f64, pub m: f64 }

/// 検定結果＝検定比・判定・適用根拠（条文）。
pub struct CheckResult {
    pub ratio: f64,          // 検定比 demand/capacity
    pub ok: bool,            // ratio <= 1.0
    pub basis: String,       // 適用条文・規準ページ（出典）
    pub detail: String,      // σ, 許容応力度 等の内訳
}

pub struct DesignCtx { pub term: LoadTerm, /* 規準版など外部設定 */ }

/// 荷重の継続性区分（許容応力度が異なる）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadTerm { Long, Short }
```

> **★`Material.fc`（P3 で追加）:** `squid_n_core::model::Material` に `#[serde(default)] pub fc: Option<f64>`
> を追加する。鋼材では `None`。RC 設計（令91条）の許容圧縮・せん断に用いる。
> コンクリート強度 Fc はヤング率 `young` とは別物であり、`young` から Fc を類推してはならない。

### 6.2 実装の分割（材種 × 検定項目）

- **RC**：曲げ・せん断・付着・定着（長期/短期）。
  - **曲げ（暫定式）**: コンクリート縁応力 σc = M / Z（Z は `Section.iz`）を `Fc/3`（長期）・`2Fc/3`（短期）で検定。
    本格な引張鉄筋検定 σs = M / (a_t·j)（j=7d/8）は `Section` に a_t が無いため P4 で `SectionShape` 経路が整ってから実装。
  - **せん断（暫定式）**: τ = Q / (b·j), j = 7d/8 を `Fc/20`（長期, 0.55 上限）・その2倍（短期）で検定。
    αs·fw 等の詳細式は AIJ（Category B）外部データ → P7。
- **S（鋼）**：曲げ・せん断・座屈（横座屈・局部座屈）・接合部。
  - 曲げ σ = M / Z を fb で、せん断 τ = Q / As を fs で検定（As は `Section.as_z`/`as_y`、無ければ `area`）。
- **SRC**：P3 は骨格のみ（本実装は後続）。
- **柱梁接合部パネルのせん断検定（S）** は P1 §6.7 の `τ` を入力に取るが、**本格検定は P7**。P3 は
  許容応力度の部材検定を主対象。

### 6.3 検定位置（設計書 §11.2）

- 検定は P1 §6.2.3 の**危険断面位置（既定：柱フェース・中央）**の内力で行う（節点芯ではない）。
- 位置はユーザ追加・変更可。**全位置・全荷重組合せで検定比**を出す。

### 6.4 検算例（テスト用・厳密部分）

鋼梁の長期曲げ（弾性、SN400・板厚≤40 ⇒ F=235、横座屈なしの単純ケース）:

| 量 | 式 | 値 |
|---|---|---|
| 断面係数 `Z`（矩形 B=200,D=400） | `B·D²/6` | `200·400²/6 = 5.3333e6 mm³` |
| 曲げ応力 `σ`（M=100 kN·m=1e8 N·mm） | `M/Z` | `1e8 / 5.3333e6 = 18.75 N/mm²` |
| 長期許容曲げ `fb` | `F/1.5`（令90条, F=235） | `235/1.5 = 156.67 N/mm²` |
| 検定比 | `σ / fb` | `18.75 / 156.67 = 0.1197` |

- すべて**厳密**（相対 1e-9）。`fb=F/1.5` は令90条（公有）で確定。横座屈低減（AIJ＝Category B）が
  要る断面では `fb` に低減係数を掛ける（その係数表のみ外部データ）。
- ★対応テスト: `squid-n-design-jp` の `test_steel_check_bending_spec_p3_6_4`（`allowable_stress.rs`）。
  `M=1e8 N·mm, Z=5.3333e6, σ=18.75, 検定比=0.1197` に厳密一致することを検証する。

**DoD（T6）:** 代表断面（RC梁曲げ・RC柱せん断・鋼梁曲げ）で検定比が**規準例題と一致**。
境界値（ちょうど 1.0）の判定（OK/NG の境目）を試験。`σ=M/Z` 等の力学量は厳密一致。

---

## 7. T7: 検定比 色分け一覧表（GUI）

- `squid-n-design-jp` の `CheckResult` を、**位置別・荷重組合せ別**にテーブル表示。
- 検定比で**色分け**（例：≤0.8 緑／0.8–1.0 黄／>1.0 赤）。クリックで内訳（σ・許容値・適用条文）。
- 3Dビューアと連動（NG 部材をハイライト）。

**DoD（T7）:** 検定比表が色分け表示され、NG 部材が一目で分かる。内訳に適用根拠（条文）が出る。

---

## 8. T8: テスト・DoD（フェーズ全体）

> **★DoD の種別を区別する（重要）:** GUI には「`cargo test` で自動化できる部分」と「目視確認に
> なる部分」が混在する。ジュニアが「3Dを自動テストできない＝テスト不能」と誤解しないよう、
> 各 DoD に **【自動】**（ウィンドウ不要の純テスト）／**【目視】**（手動受け入れ）を明示する。
> バグが出やすい Undo・検定ロジックは**必ず【自動】**にする。

### 8.1 表入力・編集（→ T1/T2）
- 【目視】1万部材テーブルの仮想スクロール・編集が滑らか。不正入力が赤表示。
- 【自動】**`EditCommand` の apply→invert→再apply の往復でモデルが元に戻る**（command モジュールの
  純 `cargo test`。ウィンドウ不要）。これが §3.3 の正確性の要。Undo/Redo スタックの整合も自動テスト。

### 8.2 解析・可視化（→ T3/T4/T5）
- 【目視】3Dで形状・変形図・モード形が見える。応力図 N/Q/M・CMQ 図が表示される。
- 【自動】GUI から起動する解析パスは、P2 の `Analysis` を直接呼ぶ統合テスト（ヘッドレス）で検証
  （単純梁の M 図データ＝放物線、CMQ データ＝荷重項の理論値）。描画自体は【目視】。

### 8.3 許容応力度（→ T6）★ヘッドレスで自動化
- 【自動】代表断面の検定比が規準例題一致。`σ=M/Z=18.75 N/mm²` 等の力学量は厳密（相対 1e-9）。
  境界値（ちょうど 1.0）の OK/NG 判定を試験。`squid-n-design-jp` は GUI 非依存なので全て自動テスト。

### 8.4 検定比表（→ T7）
- 【目視】色分け表示・内訳・3D連動。（色分けロジック＝検定比→色 の関数は【自動】で単体テスト可）

### 8.5 フェーズ DoD チェックリスト
| # | ゴール（§0.2） | 判定 | 章 |
|---|---|---|---|
| 1 | 表入力でモデル作成・反映 | §8.1 | §3 |
| 2 | GUI から解析実行・結果閲覧 | §8.2 | §4 |
| 3 | 3D形状/変形/モード・N/Q/M・CMQ図 | §8.2 | §5 |
| 4 | 許容応力度 検定比 規準例題一致 | §8.3 | §6 |
| 5 | 編集トランザクション（Undo/Redo・MCP共通） | §8.1 | §3.3 |

> **全チェック緑＝ P3 完了＝ v0.5 プロトタイプ完結**（表入力→解析→一次設計が一通り通る）。
> 次は v1.0：P4（材料・断面）。

---

## 9. 補足（ジュニア向け）

### 9.1 即時モードGUI（egui）の考え方（→ §2,§3）
egui は「毎フレーム UI を関数で描き直す」即時モード。状態は `App` が持ち、`update` で毎回描画する。
保持モード（ウィジェット木を保持）と違い、表示は常に `App` の現在値から再生成される。大量行は
`TableBuilder::body(...).rows(...)` の仮想スクロールで「見えている行だけ」描く。

### 9.2 なぜ編集をコマンドにするか（→ §3.3）
編集を `EditCommand`（apply＋逆操作）に統一すると、(1) Undo/Redo が逆コマンドの push/pop で済み、
(2) GUI と MCP（P8）が同じ経路でモデルを変更でき（単一ライタ＝整合）、(3) 編集履歴を直列化・再現できる。

### 9.3 検定比（demand/capacity）（→ §6）
検定比 = 作用（応力・力）／許容（許容応力度・耐力）。≤1.0 で OK。一次設計（許容応力度）は弾性応力を
許容応力度と比べる。力学量（σ=M/Z 等）は厳密、許容応力度の**令由来分（F/1.5 等）は §6 で確定**。
AIJ の座屈低減等のみ外部データ。

---

## 10. 用語（P3 で増えるもの）
| 用語 | 意味 |
|---|---|
| 即時モードGUI | 毎フレーム描き直す方式の GUI（egui） |
| 仮想スクロール | 表示中の行だけ描画する大量行対策 |
| 編集トランザクション | モデル変更を表すコマンド（apply＋逆操作）。Undo/MCP と共通 |
| 許容応力度計算 | 一次設計。弾性応力を許容応力度と比較 |
| 検定比 | 作用／許容（demand/capacity）。≤1.0 で OK |
| 長期／短期 | 荷重の継続性区分。許容応力度が異なる |
| CMQ図 | 部材荷重の荷重項（固定端モーメント・せん断）の図 |
| フェースモーメント | 柱・梁フェース位置のモーメント（P1 §6.7） |

---

*本仕様書は P3（最小UI／設計）を対象とし、v0.5 プロトタイプを完結させる。次は v1.0 の P4（材料・断面）。*
*許容応力度の規準値・SRC・パネルせん断検定の本格実装は外部データ化＋後続フェーズ（P7）で深掘りする。*
