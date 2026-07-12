# 非線形モデル（RESP-D「05 非線形モデル」）照合

**原典:** RESP-D 操作・計算マニュアル 計算編「05. 非線形モデル」（ユーザー提供資料、
2026-07-12 照合）。本ドキュメントは同マニュアルとの照合で追加実装した項目を記録する。

## 実装した項目

### 1. RC 耐震壁のせん断非線形特性（トリリニア Qc/βu/Qu）

**対象:** `squid-n-design-jp/src/rc/wall_nonlinear.rs`（新規）。
非線形解析のせん断ばね骨格に用いるトリリニア（ひび割れ・降伏剛性低下・終局）を算定する。
従来は許容応力度検定（RC規準18条、`rc/wall.rs`）のみで、非線形骨格は未実装だった。

| 諸元 | 式 | 出典 |
|---|---|---|
| せん断ひび割れ強度 Qc | `(0.043·pg+0.051)·√Fc·Aw`（工学単位系 kgf/cm²・cm² で評価し N へ換算） | 技術基準解説書 P.635-637 |
| せん断降伏時剛性低下率 βu | `0.46·pw·σy/Fc+0.14`（σy/Fc は比のため単位非依存） | 同上 |
| 終局せん断強度 Qu | `{k·pte^0.23·(Fc+18)/(M/QD+0.12)+0.85·√(σwh·pwh)+0.1·σ0}·te·j·r`、k=0.053/0.068 | 荒川mean式系・技術基準解説書 P.281-282,638-639 |
| 開口低減率 r | `1−max(r0, l0/lw, h0/h)`、`r0=√(h0·l0/(h·lw))` | RC終局強度設計資料 P.132 |

**配線:** `joint_wiring::collect_joint_checks_with_long` が Wall 要素（RcWall 形状）＋付帯柱
から入力を組み立て、`wall_shear_trilinear` を評価。結果は `joint_checks` に
「耐震壁(RC)せん断非線形」ラベル（Qu 検定比＋Qc/βu/Qu/r を detail 表示）として追加され、
アプリ設計タブ（`design_view.rs`「接合部・耐震壁の検定」）に表示される。

**検証:** `rc/wall_nonlinear.rs` 手計算照合テスト 10 件（Qc・βu・Qu・開口低減・単位換算・
クランプ）＋`joint_wiring/tests.rs::wall_with_side_columns_emits_nonlinear_shear_trilinear`。

### 2. プッシュオーバーのヒンジ判定を実スケルトン化＋部材塑性率3方式

**対象:** `squid-n-solver/src/nonlinear/pushover/mod.rs`。

- **ヒンジ閾値の実スケルトン化:** 従来の粗い仮値（My=σy·Z弾性、Mc=My/3、Mu=My·1.2）を、
  RC=ひび割れ `Mc=κ·Fc·Ze`（κ=0.56）・降伏 `My=0.9·at·σy·j`、鉄骨=全塑性 `Mp=Zp·σy`
  （H/箱/パイプは閉形式 Zp）へ置換（`member_moment_thresholds`）。
- **部材塑性率（ductility）3方式:** RESP-D の 3 方式を実装（`DuctilityMethod`）。
  1. 塑性率基点歪み（RC: 引張0.01/圧縮0.005、鉄骨0.01）
  2. 重み付け平均塑性率 Jm=Σσref·A·|ε|·μi/Σσref·A·|ε|≥1
  3. 降伏発生時（塑性率1超）
  ファイバー要素（`FiberBeam::ductility_probe`）が危険断面の曲率・ひずみを集約し、
  塑性率基点曲率と最大応答曲率から μ=最大応答曲率/基点曲率を算定。降伏後 μ≥`ULTIMATE_DUCTILITY`
  （既定4.0）のヒンジを終局と分類。`HingeEvent.ductility` が実塑性率を持つ。

**配線:** 材料に `reference_stress/reference_strain`（`squid-n-material`）、要素に
`ductility_probe`（`squid-n-element`）を追加。アプリ解析タブに塑性率方式の選択 UI、
結果タブ（プッシュオーバー）に方式・最大部材塑性率 μmax・ヒンジ別 μ を表示。

**検証:** `pushover/tests.rs::test_pushover_computes_member_ductility`（降伏後 μ≥1）・
`test_pushover_ductility_method_selection_changes_reference`（3方式とも妥当な μ を算定）。
既存プッシュオーバー・段階的耐力喪失テストは全て緑（回帰なし）。

### 3. RC・S 梁の非線形復元力特性＋材端バネへの反映

**対象:** `squid-n-design-jp/src/rc/beam_nonlinear.rs`・`steel/beam_nonlinear.rs`（新規）、
`squid-n-core/src/rc_capacity.rs`（`rc_alpha_y_sugano` 追加）、
`squid-n-element/src/factory/mod.rs`（材端バネの降伏モーメント改良）。

- **RC 梁:** 曲げひび割れ `Mc=κ·Fc·Ze`（κ=0.56）、曲げ降伏時剛性低下率 `αy`（菅野式、
  a/D∈[1,5] クランプ・2分岐）、曲げ降伏 `My=0.9·at·σy·j`、せん断ひび割れ
  `Qc=(0.061·(Fc+49)/(M/(Q·d)+1.7))·b·j`、軸（`Nct=κ·Fc·Ac`, `Nut=at·σy`,
  `Nuc=at·σy+Fc·(Ac−at)`）。
- **S 梁:** 全塑性 `Mp=Zp·σy`、横座屈 `Mcr/Mp`（H 形鋼、フランジ材料 σy 別 3 系：
  SN400=300/835、SN490=220/605、その他=70500/σy・117000/(0.6σy) の一般式）、
  軸 `Nu=Af·σfy+Aw·σwy`。`SectionShape::plastic_modulus_strong`（H/箱/パイプ閉形式）を
  `squid-n-core` に追加。
- **配線:** 集中バネ梁要素（剛床梁）の材端バネ降伏モーメントを、従来の σy·Z弾性から
  規準の曲げ終局強度（RC=0.9·at·σy·j、鉄骨=Zp·σy）へ改良（`flexural_yield_moment`）。
  プッシュオーバーの実挙動に反映される。

**検証:** `rc_capacity.rs::test_rc_alpha_y_sugano_matches_handcalc`、
`rc/beam_nonlinear.rs` 4件・`steel/beam_nonlinear.rs` 5件の手計算照合テスト。

### 4. NewRC コンクリート構成則＋ファイバー柱への配線

**対象:** `squid-n-material/src/newrc.rs`（新規 `ConcreteNewRc`）、
`squid-n-element/src/frame/fiber_elem/mod.rs`（テンプレート切替）。

- RESP-D の NewRC 有理式 `σc/σcB=(A·X+(D−1)X²)/(1+(A−2)X+D·X²)`、
  `εc0=0.5243·(σB)^(1/4)×10⁻³`、`Ec=4k·(σB/1000)^(1/3)×10⁵·(γ/2.4)²`、
  `A=Ec·εc0/σcB`、`D=1.50+1.68×10⁻³·σB`（工学単位系 kg/cm² で評価し N/mm² 応力を出力）。
- 従来の放物線モデルに代え、ファイバー柱のコンクリートテンプレートを **Fc≤60 で
  NewRC**、Fc60 超は従来放物線へフォールバック（マニュアルの適用範囲規定）。

**検証:** `newrc.rs` 7件（ピーク＝εc0 で −Fc、初期接線＝Ec、εc0≈0.002、軟化、
引張脆性ひび割れ、commit/revert、参照値）。既存ファイバー・プッシュオーバー
テストは全て緑（回帰なし）。

## 原典照合が必要な埋め込み値（技術リード確認用）

`specs/原典照合リスト.md`「RESP-D 計算編 05 非線形モデル」節を参照。
主要な埋め込み値: κ=0.56（ひび割れ）、Qc/βu/Qu の各係数、開口低減式、
塑性率基点歪み（0.01/0.005）、ULTIMATE_DUCTILITY=4.0、壁筋 σy/σwh 既定 295。

## 実装した項目（続き）

### 5. ファイバー断面の鉄筋分離

**対象:** `squid-n-element/src/frame/fiber_elem/mod.rs`（`build_gauss_fibers`・
`add_rebar_fibers_rect/circle`）。

- RC 断面（RcRect/RcCircle）のファイバー柱を、従来の均質コンクリート断面から
  **コンクリート格子＋主筋点ファイバー（バイリニア鋼材、SD345 既定）**へ分離。
  主筋配置は `mn_surface` の M-N 相関と同じ規則（せい方向主筋を上下面、幅方向主筋を
  側面内分点／円形は円周等配）。引張側鉄筋を無視していた従来の欠陥を解消し、RC 柱の
  曲げ耐力を正しく評価する。
- コンクリートは §4 の NewRC（Fc≤60）を用いる。

**検証:** `fiber_elem/tests.rs::test_rc_fiber_section_includes_separated_rebar`
（主筋 16 本の分離配置・最外縁位置）。既存ファイバー/プッシュオーバーテストは回帰なし。

### 6. 免震支承材（マルチシアスプリング・摩擦ばね）

**対象:** `squid-n-design-jp/src/isolator.rs`（低減率・摩擦力）、`squid-n-core`
（`ElementKind::Isolator`・`IsolatorKind`・`IsolatorProps`・`IsolatorAttr`・
`Model::isolator_attrs`）、`squid-n-element/src/springs/isolator.rs`（要素）、
`factory`（配線）、`design_view`（特性表示）。

- **マルチシアスプリング低減率**（design-jp、RESP-D 表と一致）: 剛性低減率=2/n、
  耐力低減率=1/Σ_{i=1}^{n/2}(cos(π/n(i−1))+sin(π/n(i−1)))。摩擦力 Qmax=μ·N。
- **免震要素**（新 `ElementKind::Isolator`）: 2 節点要素。局所 x=鉛直=弾性軸 Kv、
  水平 2 方向は非線形せん断。積層ゴム系＝各方向独立バイリニア（K1/K2/Qd）、
  弾性すべり支承＝2 次元摩擦（合力で滑り判定 |Q|≥Qmax=μN）。回転は剛。
  commit/rollback・snapshot/restore 対応でプッシュオーバーに乗る。
- **配線**: factory（線形/非線形とも同一要素）、設計タブに免震支承材の特性
  （種別・K1/Qd・Qmax・マルチシア低減率）を表示。

**検証:** `isolator.rs`（design-jp）4件（低減率の表照合・摩擦・単調性）、
`springs/isolator/tests.rs`（element）5件（バイリニア弾性/降伏・鉛直軸・摩擦滑り・
commit/revert）。既存テスト全緑。

> **残（UI）:** 免震部材の GUI 作成フォーム（種別選択・IsolatorProps 入力）は
> 仕様策定中のため未実装。部材テーブルの「梁追加」行に **「+ 免震支承材追加」ボタンを
> プレースホルダとして配置**（押下時はステータスバーに「免震支承材の作成フォームは
> 未実装です（仕様策定中）」を表示）。現状はモデル API／IO 経由で作成し、解析・設計
> 表示で利用できる。

### 7. SRC 梁せん断終局（非充腹：格子材・ラチス材）

**対象:** `squid-n-design-jp/src/srrc/beam_nonlinear.rs`
（`SrcNonSolidWebShearInput`・`src_beam_shear_grid`・`src_beam_shear_lattice`）。
従来は充腹形（ウェブ材）の累加式（技術基準/SRC規準）のみで、非充腹形（格子材・ラチス材）
の荒川mean式系は未実装だった。

| 諸元 | 式 | 出典 |
|---|---|---|
| 格子材 Qsu | `{κ·pt^0.23·kcs·(18+Fc)/(M/Qd+0.12)+0.85·√(rpw·rσwy)+0.5·√(spw·sσwy)}·be·j` | 荒川mean式系・SRC 非充腹 |
| ラチス材 Qsu | `{κ·rpt^0.23·kcs·(18+Fc)/(M/Qd+0.12)+0.85·√(rpw·rσwy)}·be·rj + sQu` | 同上 |
| ラチス鉄骨寄与 sQu | `min(2·sM0/h0, DA·sσy·sinθ)`（曲げ降伏／引張降伏の小） | 同上 |
| κ | 通常 0.053／高強度せん断補強筋 0.068 | 同上 |

- 格子材は `pt=rpt+spt`・`j=0.8·D`・帯板項 `0.5·√(spw·sσwy)` を含む。ラチス材は RC 部を
  `rpt` のみ・応力中心間距離 `rj` で評価し、ラチス鉄骨せん断寄与 `sQu` を累加する。
- `M/(Q·d)` は適用範囲 1.0〜3.0、`kcs`（=0.5+b'/b）は 1.0 以下にクランプ。

**位置付け:** 充腹形の `src_beam_shear_ultimate_*` と同様、非線形せん断ばね骨格の
終局耐力を与える design-jp の純関数（Layer 5）。

**検証:** `srrc/beam_nonlinear.rs` 手計算照合テスト 5 件（格子材・ラチス材の手計算一致、
帯板項の加算、高強度筋 κ=0.068、不正入力 0.0）。

### 8. コンクリート履歴の静的＝逆行型／動的＝原点指向型

**対象:** `squid-n-material/src/newrc.rs`（`ConcreteHysteresis`・`set_concrete_hysteresis`）、
`squid-n-material/src/uniaxial.rs`・`squid-n-element/src/common/behavior.rs`
（トレイト既定メソッド）、`squid-n-element/src/frame/{fiber_elem,ms}.rs`（伝播）、
`squid-n-solver`（プッシュオーバー＝静的／時刻歴＝動的の切替）。

- RESP-D は NewRC コンクリートの除荷則を **静的解析＝逆行型**（除荷・再載荷が圧縮包絡線を
  可逆に辿る）、**動的解析＝原点指向型**（最大経験圧縮ひずみ点への割線で除荷・再載荷）と
  規定する。従来は原点指向割線のみで、静的解析の逆行型を区別していなかった。
- `ConcreteNewRc` に `ConcreteHysteresis{OriginOriented（既定）, Retrace}` を追加し、
  `trial()` の圧縮側を除荷則で分岐。`set_concrete_hysteresis(dynamic: bool)` で切替える。
- **配線:** `UniaxialMaterial`・`ElementBehavior` に既定メソッド
  `set_concrete_hysteresis` を追加し、ファイバー要素（`FiberBeam`）・材端バネ（`MsElement`）が
  内蔵コンクリート材料へ伝播。プッシュオーバー（静的）は `set_concrete_hysteresis(false)`
  ＝逆行型、時刻歴（動的）は `set_concrete_hysteresis(true)`＝原点指向型を、解析器が要素
  構築後に一括設定する。

**検証:** `newrc.rs` 既存 7 件（既定＝原点指向型のピーク／軟化／除荷は不変）。プッシュオーバー・
時刻歴・ファイバーの既存テストは全て緑（回帰なし）。

## 未実装項目（今後の課題）

RESP-D「05 非線形モデル」との照合で確認したが**なお未着手**の主な差異:

- ファイバー断面の対数分割（RC 9 分割）・鉄骨 16/7 分割（現状は等分割。分割数は
  十分細かく数値的影響は小さいが、RESP-D 準拠の分割ロジックは未実装）
- ファイバー第1折点 1.1倍・剛性低下率 α=1/1000 の微細規定
- SRC 梁せん断の SRC診断式（数式抽出が不確定のため保留。非充腹の格子/ラチスは §7 で実装）
- 免震部材の GUI 作成フォーム（要素・特性はエンジン実装済み。GUI は仕様策定中のため
  プレースホルダボタン「+ 免震支承材追加」のみ配置＝押下で未実装通知）

> **実装済み（本照合で追加）:** 耐震壁せん断トリリニア、ヒンジ実スケルトン化、
> 塑性率3方式、RC/S 梁の非線形復元力特性（菅野 αy・Mc・My・Qc・軸・Mp・Mcr・Nu）、
> 材端バネの曲げ終局強度化、NewRC コンクリート構成則（Fc≤60）、SRC 梁せん断終局
> （技術基準/SRC規準・非充腹格子/ラチス）、ファイバー断面の鉄筋分離、免震支承材
> （マルチシアスプリング低減率・摩擦ばね・免震要素）、コンクリート履歴の静的＝逆行型／
> 動的＝原点指向型の切替。
