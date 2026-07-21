# P10 GPU高速化 実装仕様書

**対象フェーズ:** v2.0 拡張 / P10 GPU高速化（設計書 §18）
**対象読者:** Rust ジュニアエンジニア（GPU/数値反復は §8 で補う）
**親文書:** `構造計算一貫プログラム_実装設計書.md`（以降「設計書」。`§x.y` はその章番号）
**先行フェーズ:** [P0](P0_基盤.md)〜[P9](P9_仕上げ.md)（CPU で全機能完結が前提）
**前提環境:** Rust stable / `wgpu = "29"` / `cubecl`（burn 0.21 連動版）/ `feature = "gpu"`

---

## 0. このフェーズについて

### 0.1 目的

CPU で全機能が完結していることを前提に、効果の高い処理を **GPU にオフロードする任意機能**を作る。
設計書 §15。

- **疎行列ベクトル積（SpMV）と PCG 反復**：大規模線形・固有値の内部反復を高速化（最効果）。
- **要素剛性・内力の一括計算（ファイバ積分）**：要素数×ファイバ数が大きい非線形で有効。
- すべて **opt-in feature `gpu`**。**GPU が無くても全機能が CPU で動く（CPU フォールバック必須）**。

### 0.2 完了像（ゴール）

設計書 §18 P10 / §15 DoD:

1. **SpMV / PCG が CPU 比で十分高速**（規模依存、大規模で数倍。§7.1）。
2. **GPU 無効ビルド（`--features` に gpu なし）で全テスト通過**（フォールバック。§7.3）。
3. **GPU 経路（f32）の結果が CPU 基準解（f64）と許容差内**（§7.2。決定性は保証外＝値一致で検証）。

### 0.3 スコープ境界（含む／含まない）

| 項目 | P10 で | 備考 |
|---|---|---|
| SpMV（GPU） | **含む** | §3 |
| PCG 反復ソルバ（GPU） | **含む** | §4。`LinearSolver` の `IterativePcg`（P0 §4.2）を実装 |
| 要素剛性・ファイバ積分の一括（GPU） | **含む** | §5 |
| f32 概算モード／CPU f64 確定 | **含む** | §6。精度方針 |
| CPU フォールバック | **含む（必須）** | §6 |
| **疎直接分解の GPU 化** | **含まない** | 実装コスト大。直接法は CPU（faer）に残す（§15） |
| GPU 経路の決定性（ビット一致） | **含まない（保証外）** | 値一致で検証（R28） |
| ML の GPU 実行 | **P11 と共有** | burn の wgpu バックエンド |

> **位置づけ（設計書 §15 / R21）:** GPU は「**高速スクリーニング／最適化ループ用の概算モード**」。
> **最終解は CPU の f64 で確定**する。GPU は反復解法経路に限定し、直接分解は CPU に残す。

---

## 1. タスク一覧と依存順序

```
T0 squid-n-gpu 雛形（wgpu device/queue・feature gate・CPUフォールバック）(§2)
   ├─> T1 SpMV カーネル (§3)
   │     └─> T2 PCG 反復ソルバ（LinearSolver 実装）(§4)
   └─> T3 要素剛性・ファイバ積分 一括カーネル (§5)
T4 精度方針（f32概算/CPU f64確定）・決定性検証 (§6)
T0..T4 ─> T5 テスト・DoD (§7)
```

| ID | タスク | クレート | 依存 |
|---|---|---|---|
| T0 | squid-n-gpu 雛形・フォールバック | squid-n-gpu | P0 |
| T1 | SpMV カーネル | squid-n-gpu | T0 |
| T2 | PCG 反復ソルバ | squid-n-gpu | T1, P0 §4 |
| T3 | 要素/ファイバ一括カーネル | squid-n-gpu | T0, P4/P5 |
| T4 | 精度方針・決定性検証 | squid-n-gpu | T2 |
| T5 | テスト・DoD | 全体 | 上記 |

---

## 2. T0: squid-n-gpu 雛形（wgpu 29・feature gate・フォールバック）

> **⚠ API版依存（GPU は churn が速い）:** wgpu 29 / cubecl の device 取得・バッファ・パイプライン API は
> メジャー版で変わる。下は wgpu 29 の標準パターン。**カーネルは cubecl で書けば wgpu/CUDA を切替**できる。
> ここは `gpu` feature 内に隔離され、無効時は CPU 実装にフォールバックするので、上位は影響を受けない。

```rust
// squid-n-gpu/src/lib.rs   （[features] gpu = ["dep:wgpu", "dep:cubecl"]）
#[cfg(feature = "gpu")]
pub struct GpuContext { device: wgpu::Device, queue: wgpu::Queue }

#[cfg(feature = "gpu")]
impl GpuContext {
    /// wgpu 29: Instance → request_adapter → request_device。失敗時 None（→CPUフォールバック）。
    pub async fn try_new() -> Option<Self> {
        let instance = wgpu::Instance::default();
        let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions::default()).await.ok()?;
        let (device, queue) = adapter.request_device(&wgpu::DeviceDescriptor::default()).await.ok()?;
        Some(Self { device, queue })
    }
}
```

**★フォールバックの構造（必須）:** GPU 経路と CPU 経路を**同じトレイト**の裏に置き、`gpu` feature 無効
または `GpuContext::try_new()==None` のとき自動で CPU を使う。

```rust
pub trait SpMv { fn spmv(&self, x: &[f32]) -> Vec<f32>; }
pub struct CpuSpMv { /* CSR */ }            // 常に存在
#[cfg(feature = "gpu")] pub struct GpuSpMv { /* GPUバッファ */ }
pub fn make_spmv(/* CSR, gpu: Option<&GpuContext> */) -> Box<dyn SpMv> { /* gpu可なら Gpu、無ければ Cpu */ todo!() }
```

**DoD（T0）:** `gpu` 無効でビルド・テストが通る。`gpu` 有効でも GPU 不在環境（CI）では CPU に落ちる。

---

## 3. T1: SpMV（疎行列ベクトル積）カーネル

設計書 §15。CSR 格納の `y = A·x` を GPU で。**cubecl** でカーネルを書く（wgpu/CUDA 切替可）。

- 行ごとに1スレッド（or warp）を割り当て、`row_ptr`/`col_idx`/`val` を読んで内積。
- 入出力は f32（§6）。CSR は CPU で1回構築し GPU バッファへ転送。

```rust
// squid-n-gpu/src/spmv.rs（cubecl カーネルの骨子）
// #[cube(launch)] fn spmv_kernel(row_ptr: &Array<u32>, col: &Array<u32>, val: &Array<f32>,
//                                x: &Array<f32>, y: &mut Array<f32>) {
//     let row = ABSOLUTE_POS;            // 1スレッド=1行
//     let mut acc = 0.0f32;
//     for k in row_ptr[row]..row_ptr[row+1] { acc += val[k] * x[col[k]]; }
//     y[row] = acc;
// }
```

**DoD（T1）:** GPU SpMV が CPU SpMV と f32 許容差内で一致。大規模で CPU 比 高速。

---

## 4. T2: PCG 反復ソルバ（GPU）

設計書 §15 / §5.2。前処理付き共役勾配法。**P0 §4.2 の `SolverBackend::IterativePcg` を実装**。

**PCG アルゴリズム（明示。閉形式。前処理 M）:**
```
r0 = b − A·x0;   z0 = M⁻¹·r0;   p0 = z0;   ρ0 = r0·z0
反復 k = 0,1,...:
   q   = A·p_k                    （SpMV、§3）
   α   = ρk / (p_k·q)
   x   = x + α·p_k
   r   = r − α·q
   if ‖r‖/‖b‖ < tol: 収束・終了
   z   = M⁻¹·r
   ρ_{k+1} = r·z
   β   = ρ_{k+1} / ρk
   p   = z + β·p
```
- **前処理 M**：対角（Jacobi、`M⁻¹=1/diag(A)`）→ 不完全 Cholesky(ic0) の順で用意（設計書 §5.2）。
- 内積・axpy・SpMV はすべて GPU カーネル（cubecl）。反復回数・残差を記録。

```rust
// squid-n-gpu/src/pcg.rs
pub struct PcgGpu { ctx: /* GpuContext */ (), tol: f64, max_iter: usize }
impl squid_n_math::solver::LinearSolver for PcgGpu {
    fn factorize(&mut self, k: &faer::sparse::SparseColMat<usize,f64>) -> Result<(), squid_n_math::solver::SolveError> {
        // CSR(f32) と前処理 M をGPUへ転送（直接分解はしない＝反復法）
        todo!()
    }
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, squid_n_math::solver::SolveError> {
        // 上の PCG を GPU で回す。f32 で解き、f64 にして返す（最終確定は CPU、§6）
        todo!()
    }
}
```

**DoD（T2）:** SPD 系で PCG が CPU 直接解（faer）と許容差内一致。残差が単調減少し収束。

---

## 5. T3: 要素剛性・ファイバ積分の一括計算

設計書 §15。プッシュオーバー/時刻歴で**要素数×ファイバ数**が大きいとき、各ファイバの応力・接線を
**データ並列**で一括計算（P4 `section_response`／P5 ファイバ要素の GPU 版）。

- 1スレッド = 1ファイバ（or 1断面）。材料則（一軸履歴）を GPU で評価し、断面力・接線を集約。
- 履歴状態（commit/rollback、P5 §6）は GPU バッファに保持し、確定/巻き戻しを GPU 側でも行う。

**DoD（T3）:** GPU 一括積分が CPU 版と f32 許容差内一致。要素・ファイバ数が大きい場合に高速。

---

## 6. T4: 精度方針（f32 概算 / CPU f64 確定）・決定性

設計書 §15 / R21。

- **wgpu の f64 は実質非対応 → GPU 経路は f32**（必要に応じ混合精度）。
- GPU は**概算モード**（高速スクリーニング・最適化ループ）。**最終解は CPU の f64 で確定**する
  （例：GPU で候補を絞り、採用案を CPU f64 で再解析）。
- **決定性：GPU 経路は決定的でない**ことを明記（並列順序・f32）。検証は **CPU f64 基準解との許容差**で行う
  （R28：ビット一致は CPU 単一スレッドのみ保証）。

**DoD（T4）:** GPU f32 解が CPU f64 解と規定の相対許容差内。GPU 結果をそのまま最終成果にしない
（確定は CPU）ことがコードフローで担保される。

---

## 7. T5: テスト・DoD（フェーズ全体）

### 7.1 高速化（→ T1/T2/T3）
- 大規模疎系で GPU SpMV/PCG が CPU 比 高速（規模依存。閾値は基準機で定義＝リード提供）。

### 7.2 精度（→ T2/T4）★許容差
- GPU f32 の SpMV/PCG/ファイバ積分が CPU f64 基準解と相対許容差内（例 1e-4〜1e-5、f32 相応）。

### 7.3 フォールバック（→ T0）★必須
- `gpu` feature 無効で**全テスト通過**。GPU 不在環境でも CPU に落ちて動作。

### 7.4 フェーズ DoD チェックリスト
| # | ゴール（§0.2） | 判定 | 章 |
|---|---|---|---|
| 1 | SpMV/PCG が CPU 比 高速 | §7.1 | §3,§4 |
| 2 | GPU f32 が CPU f64 と許容差内 | §7.2 | §4,§6 |
| 3 | GPU無効ビルドで全テスト通過 | §7.3 | §2 |

> **全チェック緑＝ P10 完了。** GPU は概算・最適化用途。最終解は常に CPU f64。

---

## 8. 数式・理論補足（ジュニア向け）

### 8.1 SpMV と CSR（→ §3）
疎行列×ベクトル。CSR（行ごとに非ゼロをまとめた格納）なら各行が独立に内積でき、GPU のデータ並列に
向く。反復解法（PCG）の1反復あたり最も重い演算がこれ。

### 8.2 PCG（前処理付き共役勾配法）（→ §4）
SPD 系 `Ax=b` を反復で解く。残差を直交化しながら近づける。前処理 M（A に近く逆が安い行列）で
収束を速める：対角（Jacobi）は最も安い、ic0（不完全 Cholesky）は強いが構築コスト増。直接法（分解）と
違い分解不要で、巨大疎・GPU 向き。

### 8.3 混合精度と「概算→確定」（→ §6）
GPU は f32 が速い（f64 は実質不可）。f32 は丸め誤差が大きいので、GPU では「候補を高速に絞る」用途に
徹し、**採用案だけ CPU の f64 で正確に解き直す**。これで速度と精度（＝認定に足る確定値）を両立する。

---

## 9. 用語（P10 で増えるもの）
| 用語 | 意味 |
|---|---|
| SpMV | 疎行列ベクトル積。反復解法の主演算 |
| PCG | 前処理付き共役勾配法。SPD 系の反復解法 |
| 前処理 M | 収束を速める近似逆行列（Jacobi/ic0） |
| CSR | 行圧縮疎格納。GPU 並列に好適 |
| cubecl | カーネル記述クレート。wgpu/CUDA バックエンド切替 |
| f32 概算モード | GPU の単精度高速計算。最終解は CPU f64 |
| CPUフォールバック | GPU 無しでも CPU で全機能動作する構造 |

---

*本仕様書は P10（GPU高速化）を対象とする。GPU は任意・概算用途で、最終解は CPU f64。次は P11（ML）。*
*wgpu/cubecl の API は版で変わるため `gpu` feature 内に隔離し、CPU フォールバックを常に保つ。*
