# V&V レポート（検証・妥当性確認）

本ディレクトリは、Squid-N の各要素・各設計式に対する V&V（Verification & Validation）レポートを格納する。

## V&V の定義

| 用語 | 意味 |
|------|------|
| Verification（検証） | 「式を正しく解いているか」。理論解・手計算・規準例題との一致で確認 |
| Validation（妥当性確認） | 「正しい現象を表しているか」。実験・実測との一致で確認 |

## レポート構造

各エントリは以下の項目を持つ:

- **対象**: 検証対象（例: ティモシェンコ梁 / パネルゾーン / Ai分布 / プッシュオーバー機構）
- **参照解の出典**: 理論式 / 実験 / 商用ソフト / 規準例題 / 添付資料
- **入力モデル**: 再現可能な定義（テストに対応づけ）
- **許容差**: 厳密=1e-9 / 収束=±% / 規準例題照合
- **結果**: 合否・差分・グラフ

## 検証の性格

| 区分 | 該当項目 | 許容差 |
|------|----------|--------|
| 厳密一致 | IIE 梁・CMQ・σ=M/Z・剛性率 | 1e-9 |
| 収束 | MITC4 板・固有値・時刻歴 | ±5% |
| 規準例題照合 | Ai・Ds・許容応力度 | 告示値一致 |

## テスト階層

| レベル | 内容 | ツール |
|--------|------|--------|
| 単体 | 要素剛性・履歴則・断面算定式 | `cargo test`, `approx` |
| 性質 | 剛性対称性・エネルギー保存・パッチテスト | `proptest` |
| 回帰 | 履歴ループ・スケルトン形状 | `insta`（スナップショット） |
| 数値照合 | 理論解（梁・板・SDOF/MDOF） | 専用ベンチ集 |
| ベンチマーク照合 | 既往実験・商用ソフト | 検証レポート |
| 性能 | 速度回帰 | `criterion` + CI 閾値 |
| 決定性 | 同一入力ビット一致 | 専用テスト |

## 索引（要素/設計式 → テスト）

| # | 対象 | クレート | ソースファイル | テスト関数 | フェーズ | 状態 |
|---|------|----------|---------------|-----------|---------|------|
| 1 | ティモシェンコ梁 | squid-n-element | beam.rs | `test_phi_zero_converges_to_bernoulli`, `test_beam_axial_stiffness`, `test_beam_torsion_stiffness` | P1 | ✅ |
| 2 | 剛域あり梁 | squid-n-element | beam.rs | `test_auto_rigid_zone_standard_formula` | P1 | 🔶 |
| 3 | 端部ばね（ピン・半剛） | squid-n-element | beam.rs | `test_pinned_end_releases_moment` | P1 | 🔶 |
| 4 | MITC4 シェル（膜） | squid-n-element | shell.rs | `test_patch_membrane_distorted`（歪みメッシュ・機械精度） | P1.5 | ✅ |
| 5 | MITC4 シェル（曲げ） | squid-n-element | shell.rs | `test_patch_bending_distorted`（歪みメッシュ定曲率・機械精度） | P1.5 | ✅ |
| 6 | MITC4 シェル（せん断/収束） | squid-n-solver | linear.rs | `test_ss_plate_convergence`, `test_clamped_plate_convergence`（板たわみ ±2% 収束＝ロッキングなし） | P1.5 | ✅ |
| 7 | パネルゾーン弾性 | squid-n-element | panel.rs | `test_panel_zone_reference_case1`（pQc=851.135kN 等）, `test_panel_zone_reference_case2_t_joint`（ト型） | P1 | ✅ |
| 8 | 線形静的解析 | squid-n-solver | linear.rs | `test_*`（座標変換回帰 `test_beam_to_global_transverse_uses_correct_inertia` 含む） | P2 | ✅ |
| 9 | 固有値解析 | squid-n-solver | eigen.rs | `test_1dof_period` | P2 | ✅ |
| 10 | Ai分布 | squid-n-load | ai.rs | `test_*` | P2 | ✅ |
| 11 | 床荷重分割 | squid-n-load | floor.rs | `test_*` | P2 | ✅ |
| 12 | 荷重組合せ | squid-n-load | combo.rs | `test_combinations` | P2 | ✅ |
| 13 | 許容応力度設計 | squid-n-design-jp | allowable_stress.rs | `test_steel_check_bending_spec_p3_6_4` 他 | P3 | ✅ |
| 14 | 保有耐力 | squid-n-design-jp | holding_capacity.rs | `test_*` | P7 | 🔶 |
| 15 | プッシュオーバー | squid-n-solver | pushover.rs | — | P5 | 🔶 |
| 16 | 壁（TVLEM） | squid-n-element | — | — | P5.5 | ❌ |
| 17 | 時刻歴 | squid-n-solver | timehistory.rs | — | P6 | ❌ |
| 18 | 限界耐力 | squid-n-design-jp | capacity_spectrum.rs | `test_capacity_spectrum` | P12 | ❌ |
| 19 | 一軸履歴則（Concrete/Bilinear/MP） | squid-n-material | uniaxial.rs | `test_concrete_*`/`test_bilinear_*`/`test_menegotto_pinto_*` | P4 | ✅ |
| 20 | 部材履歴則（武田・原点指向・スリップ） | squid-n-material | hysteresis.rs | `tests/hysteresis_snapshots.rs`/`tests/uniaxial_snapshots.rs` | P4 | ✅ |
| 21 | ファイバ断面（M–φ 積分） | squid-n-section | fiber.rs | `test_section_*` | P4 | ✅ |
| 22 | スケルトン自動算定（M–φ→M–θ） | squid-n-skeleton | lib.rs | `test_rc_skeleton_*` | P4 | ✅ |
| 23 | MCP サーバ（rmcp） | squid-n-mcp | lib.rs | — | P8 | ❌ |
| 24 | ST-Bridge 入出力 | squid-n-io | stbridge.rs | `test_roundtrip_*` | P8 | 🔶 |
| 25 | 編集トランザクション（EditCommand/Undo） | squid-n-edit | lib.rs | `test_*` | P3/P8 | ✅ |

凡例: ✅ 実装済み・🔶 一部実装（要拡張）・❌ 未実装

> P7（二次設計）の監査結果は `docs/v_and_v/p7_review.md` を参照。
> #14: 監査で「✅」が虚偽と判明（テストがコンパイル不能・通常ビルド未検証・T2/T5未実装）。
> その後 T2 偏心率（D値法・モデル自動算定）・T4 部材ランク/層Ds・T5 パネルせん断・T6 統合を実装し、
> `p7` を default feature 化（rot 再発防止）。残るは原典照合（RankCriteria 等）と偏心率精算のため 🔶。
> #10: Ai 層せん断 `qi` が `Ci·単層重量` の重大バグだった（正: `Ci·累積重量`）。地震荷重が最上層に
> 偏り基部せん断が過小だった。P7 フォローアップで修正（`test_story_shear_uses_cumulative_weight`）。
> P8（操作・連携）の監査結果は `docs/v_and_v/p8_review.md` を参照。
> #23: MCP は「実装」コミット済みだが `--features mcp` で **13 エラーでコンパイル不能**（rmcp 1.7 API
> rot＋`ResultStore`/`EditCommand` が Send でない）。ツールも `model_query` が常に空を返す等スタブ。❌。
> #24: ST-Bridge は当初 import/export とも未実装だったが、P8 検証で **2.0 subset の意味的往復を実装**
> （節点・層・材料・断面・部材・節点荷重。export 冪等・再import安定をテスト）。断面は形鋼ライブラリ
> 参照でなく物性直持ち（StbSecRaw）の subset のため 🔶。完全な他社相互運用は将来。
> #25: `squid-n-edit` の EditCommand/UndoStack は P3 で実装済み・健全（MCP からの利用は未配線）。
> #13: 断面検定（許容応力度検定）の RESP-D マニュアル照合結果は
> `docs/v_and_v/断面検定_RESP-D照合.md` を参照（対象: rc.rs/steel.rs/src_cft.rs/
> joint.rs/joint_wiring.rs/combo.rs）。
> P4（材料・断面）の監査結果は `docs/v_and_v/p4_review.md` を参照。
> #19: 包絡線（軟化・接線符号・連続性）・ひび割れ判定・MP 反転検知/ξ 更新を修正し、単軸履歴則 insta スナップショット追加で ✅。
> #20: 武田・原点指向・スリップに `UniaxialMaterial`(trial/commit/revert) を実装。武田内側ルール（ポリゴン則）・TakedaDegrading（ピーク劣化）を本格化。insta スナップショットでループ固定し ✅。
> #21: ファイバごとの独立状態化・弾性域厳密(1e-9)・降伏進展テスト追加で ✅。
> #22: RC ファイバ（鉄筋点）組込み・ひずみイベント抽出・塑性ヒンジ M-θ 変換・せん断変形/鉄筋抜出し加算・手計算 My=at·σy·j・Mu=0.9·at·σy·j 照合追加で ✅。
> #15/#16: RESP-D「05 非線形モデル」照合で (a) RC 耐震壁のせん断非線形トリリニア
> （Qc/βu/Qu＋開口低減）を新規実装し設計/結果表示へ配線、(b) プッシュオーバーの
> ヒンジ閾値を実スケルトン（RC: Mc=κFcZe/My、鉄骨: Mp=Zpσy）化し、部材塑性率
> （ductility）3方式（基点歪み/重み付け平均Jm/降伏時）を実装・UI 配線した。
> 詳細・未実装項目は `docs/v_and_v/非線形モデル_RESP-D照合.md`。

## 1 次参照: 手計算／理論解

本ソフトの V&V は、原則として **手計算／理論解** を一次基準とする。各フェーズの DoD は手計算・理論解・告示式の自己整合・添付資料数値例で合否判定できるよう構成されている。

**実測／商用ソフト照合は補助**（入手できれば追加）であり、無くてもビルド・単体テスト・一次 V&V は通る。

**唯一の例外＝壁（壁谷澤）の妥当性確認** は、モデルの性質上、実験照合が本質的に必須（Category B）。
技術リードが実験データを用意する（R4/R23）。

## パネルゾーン参照解

出典: 添付資料『パネルゾーンの力学』(小野瀬, 2009) 図18–20

| ケース | pQc | pQb | τ |
|--------|-----|-----|---|
| ケース1 | 851.135 kN | 1702.273 kN | 42.557/tp |
| ケース2 | (資料参照) | (資料参照) | (資料参照) |
| ケース3 | (資料参照) | (資料参照) | (資料参照) |

## 決定性テスト

全解析種別で「CPU・単一スレッドでビット一致」を検証（R28）。並列／Parquet書込／GPU は値一致で検証（ビット一致保証外）。

| 解析種別 | 状態 | ファイル |
|----------|------|----------|
| 線形静的 | ✅ | linear.rs |
| 疎行列組立 | ✅ | sparse.rs |
| Cholesky 分解 | ✅ | cholesky.rs |
| 固有値 | ✅ | eigen.rs |
| 時刻歴 | 🔶 | timehistory.rs（P6 実装後に本格化） |
| プッシュオーバー | 🔶 | pushover.rs（P5 実装後に本格化） |

## 性能ベンチマーク

`criterion` による性能ベンチマーク（線形静的・固有値・プッシュオーバー1ステップ・時刻歴1ステップ）の計測は CI 導入時（P9）に整備予定。
