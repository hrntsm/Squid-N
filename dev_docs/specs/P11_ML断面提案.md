# P11 ML 断面提案 実装仕様書

**対象フェーズ:** v2.0 拡張 / P11 ML（設計書 §18）
**対象読者:** Rust ジュニアエンジニア（ML は §8 で補う）
**親文書:** `構造計算一貫プログラム_実装設計書.md`（以降「設計書」。`§x.y` はその章番号）
**先行フェーズ:** [P0](P0_基盤.md)〜[P9](P9_仕上げ.md)（特に squid-n-design-jp/squid-n-solver を学習・検証に使う）
**前提環境:** Rust stable / `burn = "0.21"`（backend: ndarray/wgpu/cuda を feature 切替）/ `feature = "ml"`

---

## 0. このフェーズについて

### 0.1 目的

設計者の初期断面選定を支援する **ML 提案エンジン**を作る。設計書 §16。

> **★最重要の原則（§16.1 / §20-risk6）:** **ML は「提案」だけ。最終判断と適合性は必ず
> `squid-n-design-jp` の断面算定（P3 許容応力度・P7 保有耐力）で検証する。** ML の出力を無検証で設計に
> 使わせない。本フェーズの成果物には**必ず規準計算による検証ゲートを通す**フローを組み込む。

- **回帰/分類モデル**：スパン・荷重・階数・部材種別・想定軸力・地域係数 → 推奨断面寸法・配筋クラス。
- **サロゲート（代理）モデル**：プッシュオーバー/検定の結果（保有耐力・検定比）を近似し、候補の高速
  スクリーニングに使う。
- **最適化ループ（将来）**：提案 → 規準計算で検証 → フィードバック（ベイズ最適化／簡易 RL）。

### 0.2 完了像（ゴール）

設計書 §18 P11 / §16 DoD:

1. 提案断面が**規準算定で「検定比 ≤ 1.0」を高確率で満たす**（§7.1）。
2. **提案 → 規準検証ループが MCP（P8）から実行可能**（§7.2）。
3. **検証ゲートを通らない提案は採用候補にしない**（無検証使用の禁止が構造で担保。§7.2）。

### 0.3 スコープ境界（含む／含まない）

| 項目 | P11 で | 備考 |
|---|---|---|
| 学習データ自動生成（合成＋過去案件） | **含む** | §3。squid-n-design-jp/squid-n-solver を回す |
| 回帰/分類モデル（断面提案） | **含む** | §4。初期版 |
| サロゲートモデル（検定比近似） | **含む** | §5 |
| 提案→規準検証ループ（MCP 連携） | **含む（必須ゲート）** | §6 |
| 最適化ループ（ベイズ/RL） | **骨格のみ → 将来** | §7（タスク T5） |
| モデル配布（onnx/burn native）・推論API | **含む** | §4 |
| **ML 出力の無検証採用** | **禁止** | §0.1 / §20-risk6 |

---

## 1. タスク一覧と依存順序

```
T0 squid-n-ml 雛形（burn backend・feature gate）(§2)
   └─> T1 学習データ自動生成（squid-n-design-jp/squid-n-solver 合成）(§3)
          ├─> T2 回帰/分類モデル（断面提案）(§4)
          └─> T3 サロゲートモデル（検定比近似）(§5)
T4 提案→規準検証ループ（MCP 連携・必須ゲート）(§6)  ← T2,T3, P3/P7/P8
T5 最適化ループ（骨格）(§7)
T0..T5 ─> T6 テスト・DoD (§7)
```

| ID | タスク | クレート | 依存 |
|---|---|---|---|
| T0 | squid-n-ml 雛形 | squid-n-ml | P0 |
| T1 | 学習データ自動生成 | squid-n-ml | P7, P5 |
| T2 | 回帰/分類モデル | squid-n-ml | T1 |
| T3 | サロゲートモデル | squid-n-ml | T1 |
| T4 | 提案→規準検証ループ | squid-n-ml/squid-n-mcp | T2,T3, P3/P7/P8 |
| T5 | 最適化ループ（骨格） | squid-n-ml | T4 |
| T6 | テスト・DoD | 全体 | 上記 |

---

## 2. T0: squid-n-ml 雛形（burn 0.21）

> **⚠ API版依存:** burn の Module/Tensor/Backend API はマイナー版で変わる。下は burn 0.21 の標準パターン。
> backend を **feature 切替**（ndarray=CPU / wgpu / cuda）。`ml` feature 内に隔離。

```rust
// squid-n-ml/src/lib.rs   （[features] ml = ["dep:burn"]; backend は更に細分）
use burn::prelude::*;
use burn::nn::{Linear, LinearConfig, Relu};

/// 断面提案ネット（入力特徴 → 断面寸法/クラス）。
#[derive(Module, Debug)]
pub struct SectionNet<B: Backend> { l1: Linear<B>, l2: Linear<B>, act: Relu }

impl<B: Backend> SectionNet<B> {
    pub fn new(device: &B::Device, n_in: usize, n_hidden: usize, n_out: usize) -> Self {
        Self {
            l1: LinearConfig::new(n_in, n_hidden).init(device),
            l2: LinearConfig::new(n_hidden, n_out).init(device),
            act: Relu::new(),
        }
    }
    pub fn forward(&self, x: Tensor<B, 2>) -> Tensor<B, 2> {
        let h = self.act.forward(self.l1.forward(x));
        self.l2.forward(h)
    }
}
// backend 例: type B = burn::backend::NdArray;  / burn::backend::Wgpu;
```

**DoD（T0）:** `ml` 無効でビルド可（コアは ML 非依存）。`ml` 有効で `SectionNet` の forward が動く。

---

## 3. T1: 学習データ自動生成

設計書 §16.3。**学習データは `squid-n-design-jp` と `squid-n-solver` を回して自動生成（合成データ）**＋過去案件で増強。

- 入力特徴ベクトル `x`：スパン・荷重・階数・部材種別・想定軸力・地域係数 Z 等（正規化）。
- 教師 `y`：その条件で**規準計算が OK（検定比≤1.0）になる断面**（探索 or 過去設計）。
- 合成：条件をサンプリング → 断面候補を規準計算（P3/P7）で評価 → ラベル付け。

```rust
// squid-n-ml/src/dataset.rs
pub struct Sample { pub features: Vec<f32>, pub target: Vec<f32> }
/// 条件をサンプリングし、規準計算でラベル付けして学習サンプルを作る。
pub fn generate_synthetic(n: usize) -> Vec<Sample> {
    // for _ in 0..n: 条件生成 → 断面候補 → squid_n_design_jp で検定 → OK断面を target に
    todo!()
}
```

**DoD（T1）:** 合成データが生成でき、各サンプルの target 断面が規準計算で検定比≤1.0（ラベルの正しさ）。

---

## 4. T2: 回帰/分類モデル（断面提案）

設計書 §16.2。

- 回帰：断面寸法（連続）を出力。分類：配筋クラス・断面種別（離散）を出力。
- 学習は burn の Optimizer（Adam 等）＋ MSE/CrossEntropy。CPU(ndarray) で学習可、GPU(wgpu) で加速（P10 共有）。
- 配布：burn ネイティブ形式 or onnx 互換。推論は MCP/GUI から呼ぶ。

```rust
// squid-n-ml/src/propose.rs
pub struct Proposal { pub section: squid_n_core::model::Section, pub confidence: f32 }
/// 学習済みモデルで断面を提案する（推論のみ）。
pub fn propose_section(net: &SectionNet<impl burn::tensor::backend::Backend>, features: &[f32]) -> Proposal { todo!() }
```

**DoD（T2）:** 学習後、テスト条件で提案断面が**高確率で検定比≤1.0**（§7.1。規準計算で確認）。

---

## 5. T3: サロゲート（代理）モデル

設計書 §16.2。プッシュオーバー/検定の**結果（保有耐力・検定比）を近似**し、候補断面の高速スクリーニングに使う。

- 入力：断面＋条件、出力：予測検定比/保有耐力。重い解析を回す前に候補を絞る。
- **サロゲートの予測はあくまで足切り**。残った候補は**必ず本物の規準計算**（P3/P7）で確定（§6）。

**DoD（T3）:** サロゲート予測と実規準計算の相関が高い。スクリーニングで候補数を削減できる。

---

## 6. T4: 提案 → 規準検証ループ（★必須ゲート・MCP 連携）

設計書 §16.1 / §16.3 / §20-risk6。**ML 提案を無検証で使わせない構造**をここで保証する。

```rust
// squid-n-ml/src/verify_loop.rs
pub struct VerifiedProposal { pub section: squid_n_core::model::Section, pub check: squid_n_design_jp::CheckResult }

/// 提案 → 規準計算で検証 → 合格のみ返す。不合格は次候補へ（or 最適化ループ §7）。
pub fn propose_and_verify(features: &[f32], model: &squid_n_core::model::Model)
    -> Option<VerifiedProposal>
{
    // 1. ML で候補断面（複数）を提案（§4）/ サロゲートで足切り（§5）
    // 2. 各候補を squid_n_design_jp で検定（P3 許容応力度 / P7 保有耐力）
    // 3. 検定比≤1.0 の最良案のみ VerifiedProposal で返す。無ければ None。
    todo!()
}
```

- **MCP（P8）から実行**：`design.propose`（提案→検証ループ）ツールを追加。返すのは**検証済み**断面のみ。
- ML の生出力（未検証）は API 表面に出さない（出すなら「未検証」と明示し、採用フラグを立てさせない）。

**DoD（T4）:** `propose_and_verify` は検定比≤1.0 の案のみ返す。MCP から提案→検証が一連で動く。
未検証提案が採用経路に乗らない。

---

## 7. T5/T6: 最適化ループ（骨格）・テスト

### 7.0 最適化ループ（骨格・将来本実装）
- 提案 → 規準計算で検証 → フィードバック（ベイズ最適化／簡易 RL）で次候補を改善。P11 では**枠だけ**。

### 7.1 提案品質（→ T2）
- テスト条件群で、提案断面が規準計算で検定比≤1.0 を**高確率**で満たす（合格率を指標化）。

### 7.2 検証ゲート（→ T4）★必須
- `propose_and_verify` が不合格案を返さない。MCP `design.propose` が検証済みのみ返す。
- ML 生出力の無検証採用が構造的に不可能（API・フロー上）。

### 7.3 フォールバック（→ T0）
- `ml` feature 無効で全テスト通過（コアは ML 非依存）。

### 7.4 フェーズ DoD チェックリスト
| # | ゴール（§0.2） | 判定 | 章 |
|---|---|---|---|
| 1 | 提案が高確率で検定比≤1.0 | §7.1 | §4 |
| 2 | 提案→規準検証ループが MCP から動く | §7.2 | §6 |
| 3 | 無検証採用の禁止（構造保証） | §7.2 | §6 |

> **全チェック緑＝ P11 完了。** ML は提案、判定は規準計算。次は P12（限界耐力計算）。

---

## 8. 数式・補足（ジュニア向け）

### 8.1 なぜ ML は「提案」止まりか（→ §0.1, §6）
構造設計の合否は**法令・規準で決まる**（ML の確信度ではない）。ML は経験的に「良さそうな初期案」を
速く出すのが役割で、それが安全かは必ず規準計算で確かめる。だから提案と検証を分離し、検証ゲートを
通った案だけを採用候補にする（無検証採用は事故・法的責任の元、§20-risk6/7）。

### 8.2 サロゲートとスクリーニング（→ §5）
本物の解析（プッシュオーバー等）は重い。サロゲートは「結果を近似する軽いモデル」で、多数の候補を
高速に足切りする。残りを本物で確定するので、速度と正確さを両立する（GPU 概算→CPU 確定と同じ思想）。

### 8.3 合成データ（→ §3）
教師データは過去案件だけだと足りない。条件をサンプリングし、規準計算でラベル付けして**自前で量産**する
（squid-n-design-jp/squid-n-solver が真値を与える）。これで ML を支える教師が揃う。

---

## 9. 用語（P11 で増えるもの）
| 用語 | 意味 |
|---|---|
| 断面提案 | 条件から推奨断面を出す ML 回帰/分類 |
| サロゲートモデル | 重い解析結果を近似する軽量モデル（足切り用） |
| 検証ゲート | 提案を規準計算で検定し、合格のみ採用する関門 |
| 合成データ | 規準計算でラベル付けした自動生成学習データ |
| burn | Rust の深層学習フレームワーク（backend 切替） |
| backend | 計算実行系（ndarray=CPU / wgpu / cuda） |

---

*本仕様書は P11（ML 断面提案）を対象とする。ML は提案のみ・判定は必ず squid-n-design-jp。最適化ループは将来。*
*次は P12（限界耐力計算）で v2.0 を完結。*
