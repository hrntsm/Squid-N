# P7（二次設計：保有水平耐力）監査レポート

**監査日:** 2026-06-22
**対象:** `crates/squid-n-design-jp`（`holding_capacity.rs` ほか）／`dev_docs/specs/P7_二次設計.md`
**結論:** 完了報告は虚偽。実装は約4割、テストはコンパイル不能の状態で放置されていた。

---

## 1. 完了報告との乖離

V&V 索引（`dev_docs/v_and_v/README.md` #14）は「保有耐力 / holding_capacity.rs / ✅」と記載していたが、
実態は以下のとおりで、**✅ は誤り**である。

| 事象 | 詳細 |
|------|------|
| テストがビルド不能 | `cargo test -p squid-n-design-jp --features p7` が E0063 で失敗。`PushoverResult` に P5/P6 で追加された `capacity_curve` `hinges` `mechanism` `qu` フィールドにテストが追従していなかった。 |
| 通常ビルドで一度も検証されない | `p7` は非デフォルト機能で、他クレートも有効化していない。モジュール全体が `cargo build`／`cargo test`（既定）で一度もコンパイルされず、壊れたまま検出されなかった。 |
| P12 の混入 | `capacity_spectrum.rs`（容量スペクトル法＝限界耐力＝P12）が `p7` feature 配下に置かれていた。フェーズ境界違反。 |

**本監査での是正（コミット1）:** テストを修正してコンパイル可能に／`capacity_spectrum` を `p12` feature へ分離／本レポート作成／README #14 を正直な状態へ更新。

---

## 2. タスク別の実装状況（監査時点）

| ID | タスク | 仕様の要求 | 実態 | 判定 |
|----|--------|-----------|------|------|
| T1 | 剛性率 Rs・層間変形角 | δ/h、Ks=h/δ、Rs=Ks/mean(Ks)、規定比較 | `stiffness_ratios` `check_story_drift` 実装。式は正しい。 | ✅ |
| T2 | 偏心率 Re（D値法剛心） | 剛心・重心・KR・弾力半径・Re をモデルから算定 | `eccentricity_ratio(e,r)` の割り算ヘルパーのみ。**剛心・重心・KR・rex は未実装**。`eccentricity.rs` は存在しない。DoD「剛心・偏心距離 e が手計算一致」を満たすコードが無い。 | ❌ |
| T3 | Fes（Fs·Fe） | 告示1792 の Fs/Fe | `fs` `fe` `fes` 実装。式は正しい。 | ✅ |
| T4 | Ds 自動分類 | 部材ランク判定＋層 Ds 決定 | `ds_value` 値表のみ。**`member_rank`／`story_ds` は未実装**（stubコメント）。`ds.rs` は存在しない。 | 🔶 |
| T5 | パネルせん断検定 | PanelResult.τ に対する検定比 | **完全未実装**（stubコメント）。`panel_shear.rs` は存在しない。 | ❌ |
| T6 | Qun 比較・判定・出力 | Qun=Ds·Fes·Qud、Qu≥Qun | `check_holding_capacity` あり。ただし `rs`/`re` を 0.0 固定、`member_ranks` 空、Qud を呼び出し側任せ（Ai・C0=1.0 から算定していない）。 | 🔶 |

---

## 3. 仕様書（`dev_docs/specs/P7_二次設計.md`）自体の構造工学的な誤り

核心の式（Rs、偏心率 KR/rex/Re、Fs/Fe、Qun、Ds 値表[ラーメン]、武藤 D値法の閉形式）は
告示1792・武藤理論に照らし正しい。ただし以下2点に不正確さがある。

### 3.1 §5.1 剛心の式（要修正）

> `Xs = Σ(Di·xi)/ΣDi,  Ys = Σ(Di·yi)/ΣDi`

単一の D 値を用いているが、剛心座標は方向別の水平剛性で重み付けすべき:

```
Xs = Σ(Dy_i · x_i) / Σ Dy_i      （x 座標は Y 方向 D 値で重み付け）
Ys = Σ(Dx_i · y_i) / Σ Dx_i      （y 座標は X 方向 D 値で重み付け）
```

柱の X 方向 D 値 `Dx`（強軸/弱軸・取付梁が異なる）と Y 方向 D 値 `Dy` は一般に異なるため、
単一 D 値の式は**対称架構でしか成立しない**。実装は方向別 D 値で行う。

### 3.2 §5.2 計算例（要修正）

> 「X方向に構面2つ、水平剛性 Kx=[100,300]、位置 x=[0,6000] … Xs=4500」

「X 方向の構面」は通常 Y 位置に並ぶため、これらから求まるのは Ys であり Xs ではない。
Xs を求めるには Y 方向に効く構面（X 位置に並ぶ）の剛性が要る。数値（剛性重み付き重心＝4500）の
概念は正しいが、方向ラベルが幾何学的に矛盾している。計算例を方向整合に書き直す。

### 3.3 軽微（許容範囲・仕様が明記済み）

- Ds 値表の耐震壁付 RC・筋かい付 S は βu／壁量比による細則行が省略（仕様も「βuにより」と明記）。
- RC 部材ランクを「曲げ/せん断耐力比」に単純化（実際は軸力比・帯筋間隔等も関与＝AIJ規準。v1 骨格として許容）。

---

## 4. 本ブランチでの是正結果

| # | 作業 | 結果 |
|---|------|------|
| 1 | 仕様 §5.1/§5.2 の修正 | 完了（剛心を方向別D値に、計算例を方向整合に） |
| 2 | T2 偏心率 `eccentricity.rs` | 完了。`d_value`／`center_of_rigidity`／`eccentricity`（KR・rex/rey・Re）／`center_of_mass`。DoD §8.1 数値例・手計算 1e-9 照合 |
| 3 | T4 `ds.rs` | 完了。`rc_member_rank`／`s_member_rank`／`story_ds`（機構補正含む）。しきい値は `RankCriteria` で外部化 |
| 4 | T5 `panel_shear.rs` | 完了。`check_panel_shear`（F/√3 短期・F/(1.5√3) 長期、割増 factor 外部化） |
| 5 | T6 `check_holding_capacity` 統合 | 完了。**Qu を P5 `capacity_curve` 最終点から取得**、Rs/Re/部材ランクを出力に反映、境界（Qu=Qun）試験追加 |

**テスト:** `cargo test -p squid-n-design-jp --features p7` → 60 passed / 0 failed。`cargo build --workspace` 緑、clippy 警告なし。

## 5. フォローアップ（feat/p7-followup ブランチ）での是正

| 当初の申し送り | 是正 |
|---|---|
| `p7` が非デフォルト機能で CI 未検証＝rot 源 | **`default = ["p7"]` に変更**。通常 `cargo test`/`cargo build` が常に P7 を検証する |
| モデル全体からの自動算定が未配線 | **`column_stiffnesses`/`story_centers`/`story_eccentricity` を実装**。実柱から方向別D値（局所→全体射影）・武藤a補正・剛心・偏心率をモデルから算定 |
| `qud_by_story` の Ai `qi` が要検証 | **重大バグを発見・修正**（§6）。`qud_by_story(C0=1.0)` ヘルパー追加 |

## 6. 派生して発見・修正した重大バグ（P2 Ai 層せん断）

`squid-n-load::ai::ai_distribution` の層せん断 `qi` が `Ci·単層重量` で計算されていた。正しくは
**`Qi = Ci·Wi`（Wi＝当該層以上の累積重量、令88条）**。旧実装では各層水平力
`Pi = Qi[i] − Qi[i+1]` が下層で負になり `max(0)` で 0 に潰れ、**地震外力が最上層にしか
載らず基部せん断が大幅過小**になっていた（等重量3層で基部せん断が約半分）。

- 影響範囲: 線形地震解析（`analysis.rs`）・プッシュオーバー荷重パターン（`pushover.rs`）・P7 Qud。
- 修正: `qi = ci·αi·total_w`（αi=Wi/total_w）。`test_story_shear_uses_cumulative_weight`
  で ΣPi=基部せん断・単調減少を照合。squid-n-load 11 / squid-n-solver 58 passed。

## 7. なお残る限界（テックリード作業・将来）

- **`RankCriteria` 既定値・パネル割増 factor・部材ランクしきい値は仮値。** AIJ 規準（Category B）
  からの原典照合・サインオフが必要（`dev_docs/v_and_v/原典照合リスト.md`）。実装者は判定ロジックのみ提供。
- **偏心率の精算（剛床マスター節点 3×3 剛性）は未実装。** v1 は D値法（略算）。仕様どおり将来送り。
- **D値法のモデル抽出は明示した仮定の上に成り立つ**（柱＝鉛直部材、層帰属＝上端節点 story、
  梁強軸 iz 使用、最下層＝最小 elevation）。複雑な架構では精査が要る。
- **部材ランクの実部材からの集約**（Qsu/Qmu・幅厚比を断面・耐力から拾う配線）は呼び出し側／後続。

> 結論: P7 の力学コアとモデル自動算定の主要配線は実装・テスト済みで手計算と一致し、
> P2 Ai の重大バグも修正した。残るのは原典照合（テックリード）と偏心率精算（将来）であり、
> V&V #14 は **🔶（一部実装＝原典照合待ち）** が引き続き正しい。
