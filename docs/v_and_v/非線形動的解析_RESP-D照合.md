# 非線形解析（動的解析）（RESP-D「07 非線形解析（動的解析）」）照合

**原典:** RESP-D 操作・計算マニュアル 計算編「07. 非線形解析（動的解析）」
（ユーザー提供資料、2026-07-12 照合）。本ドキュメントは同マニュアル「履歴特性」
および「立体解析モデルの非線形特性（既定の非線形特性）」との照合で追加・是正した
項目を記録する。

## 実装した項目

### 1. 履歴特性（履歴則）の拡充 — 逆行型・標準型・最大点指向型

**対象:** `squid-n-material/src/hysteresis.rs`（`HysteresisRule` / `HysteresisMaterial`）。

従来は武田型（`Takeda`）・原点指向型（`OriginOriented`）・スリップ型（`Slip`）のみで、
マニュアル「履歴特性」の以下の名前付き履歴則が欠落していた。

| 履歴則 | 追加 variant | 挙動（原典） |
|---|---|---|
| 逆行型 | `Retrograde` | 常にスケルトンカーブ上を進む（除荷・再載荷を可逆に辿り履歴ループを生じない） |
| 標準型 | `Standard` | 除荷は Masing 則（相似則）。除荷開始剛性=初期剛性 K1、除荷後の第2/第3剛性は骨格の剛性低下率と同様 |
| 最大点指向型 | `MaxPointOriented` | \|δ\|<δy1 は原点勾配 K1。降伏後は戻り点から反対側の最大経験変形点を直線で指向（Clough 系） |

- `Retrograde`: `evaluate` 冒頭で常に `Branch::Skeleton` に短絡。
- `Standard`: 反転時に `Branch::Masing` へ遷移。`Q(θ)=Qr − 2·sgn(θr−θ)·g(|θr−θ|/2)`
  （`g`=スケルトン力）で除荷開始勾配 `g'(0)=K1`、反射点（反対側 \|θ\|≥\|反転点\|）で
  スケルトンへ復帰する。
- `MaxPointOriented`: 反転時（降伏後）に `Branch::PeakOriented` へ遷移。戻り点から
  反対側の最大経験点への直線を辿り、到達でスケルトンへ復帰。

**検証:** `hysteresis.rs` 手計算照合テスト（逆行型のスケルトン可逆性、標準型の除荷開始
剛性=K1・反射点復帰・θ=0 で履歴枝上、最大点指向型のピーク指向補間、新規則の
スケルトン一致）。

### 2. 武田型の除荷剛性式を原典へ是正

**対象:** `squid-n-material/src/hysteresis.rs::unloading_stiffness`。

原典「履歴特性 武田型」は `Kd+ = K0·|δmax/δy2|^(−ν)`（K0=初期勾配、δy2=降伏変形、
ν=除荷指数）を規定するが、従来実装は基準を降伏割線 `Ky=My/θy` としていた（誤り）。
初期勾配 `K0 = Mc/θc` を基準に是正した（既存 `alpha` を指数 ν として流用）。
武田型ループのスナップショット（除荷ゼロ交差点）が是正値に更新される（骨格の折れ点は
不変）。

### 3. 既定の非線形特性表 → 材端曲げバネへの配線

**対象:** `squid-n-core`（`HysteresisModel`・`default_member_hysteresis`・
`Model::member_hysteresis_attrs`）、`squid-n-element/src/factory/mod.rs`、
`squid-n-edit`（`SetMemberHysteresis`）、`squid-n-app`（部材表・設計ビュー）。

原典「立体解析モデルの非線形特性」の既定表（梁の曲げ: RC/SRC/CFT=武田型、S=標準型）を
実装し、材端集中バネ（`ConcentratedSpringBeam`）へ配線した。従来は全ての梁が kinematic
バイリニアで、RC 梁が武田型（剛性低下トリリニア）にならない不整合があった。

- **core:** `HysteresisModel{Auto,Retrograde,Standard,OriginOriented,MaxPointOriented,Takeda}`、
  部材個別指定の側テーブル `member_hysteresis_attrs`（`isolator_attrs` と同方式）、
  既定表 `default_member_hysteresis(rc_like)`。
- **element/factory:** `resolve_member_hysteresis`（個別指定→断面形状による既定表）で
  履歴則を解決し、`build_flexural_springs` が武田型/逆行型/最大点指向型は
  `HysteresisMaterial`（トリリニア）、原点指向型はバイリニア、標準型は従来 kinematic
  バイリニアの材端バネを構築。トリリニア折れ点は Mc=0.56·Fc·Ze（RC）/My/3、
  My=規準の曲げ終局、θy=My/(αy·k_rot)（αy=0.3 既定）、Mu=1.1·My、θu=4·θy。
  武田型等の履歴材料は N-M 相関（`set_yield`）非対応のため相関はバイリニア時のみ適用。
- **edit/UI:** `SetMemberHysteresis`（undo 可）、部材表に「履歴則」列（自動/逆行型/
  標準型/原点指向型/最大点指向型/武田型。梁のみ）、設計タブに材端履歴則の集計表示。

**検証:** `factory/tests.rs`（既定表の解決・RC=武田型/S=標準型・個別指定 override・
RC 材端バネが実際に武田型の除荷剛性低下を示す実挙動テスト）、`edit/tests.rs`
（`SetMemberHysteresis` の undo/redo・不存在部材の Noop）、`core/model/tests.rs`
（既定表・側テーブル round-trip）。既存のプッシュオーバー/時刻歴/段階的耐力喪失
テストは全て緑（回帰なし）。

### 4. 辻・山田モデル（β 混合硬化）

**対象:** `squid-n-material/src/hysteresis.rs`（`TsujiYamada`）、`squid-n-core`
（`HysteresisModel::TsujiYamada`）、`squid-n-element/src/factory/mod.rs`。

バイリニア骨格 + β による等方硬化/移動硬化の混合硬化則。塑性増分応力 Δσ を
等方硬化 `Δσ̄=β|Δσ|`（降伏幅膨張）と移動硬化 `Δᾱ=(1−β)|Δσ|`（中心移動）へ配分。
β=1 等方（降伏耐力が正負同時に膨張）、β=0 移動（バウシンガー効果）。`UniaxialMaterial`
として実装し、`set_yield` 対応のため材端バネでは N-M 相関も適用。部材表・設計ビューの
履歴則選択に「辻・山田型」を追加。

**検証:** 単調バイリニア・β=1 等方膨張・β=0 バウシンガー・commit/revert の4テスト。

### 5. 鉄骨梁端部の累積損傷度（レインフロー法）

**対象:** `squid-n-solver/src/damage.rs`（新規）、`squid-n-app/src/time_history_view.rs`。

RESP-D「その他の解析機能」の累積損傷度計算。ASTM E1049-85 3 点レインフロー計数
（`rainflow_cycles`）、レインフロー法 `D=ΣNei/Nfi`（`Nf=(μ/C)^(−1/β)`、片振幅
μ=振れ幅/2）、累積塑性変形倍率（最大振幅）法 `D=η/4·(μmax−1)·(μmax/C)^(1/β)`。
時刻歴ビューに代表応答のレインフロー等価繰返し数・最大振れ幅を参考表示。

**検証:** 折返し点抽出・入れ子サイクル・手計算照合の6テスト。

### 6. 免震支承材の装置別ひずみ依存モデル

**対象:** `squid-n-design-jp/src/isolator.rs`、`squid-n-app/src/design_view.rs`。

- **LRB 統一型（歪依存バイリニア）:** 降伏後剛性のひずみ依存 `CKd(γ)`、切片荷重の
  ひずみ依存 `CQd(γ)`、温度換算 `Kd(t0)/Qd(t0)`、等価水平剛性 `keq=Qd/δ+Kd`、
  等価粘性減衰定数 `Heq=(2/π)Qd(δ−Qd/((β−1)Kd))/(keq·δ²)`。
- **転がり支承:** `μ=(1.2+7.8·Pv/Po)/1000`、`Q1=μ·Pv`。
- **球面すべり支承:** 面圧依存 `μ0`（MN/LN）、MN 速度依存 `μ=μ0·(1−0.55·e^(−0.019|V|))`。
- **高減衰ゴム系（ブリヂストン E6/E4）:** `Geq/Heq/U` の歪 γ 多項式。
- 設計ビューの免震支承材一覧に積層ゴム系の keq/Heq（δ=200mm 参考）を表示。

**検証:** LRB ひずみ依存・温度換算・等価特性・転がり・球面すべり・高減衰ゴムの
手計算照合8テスト。

### 7. 減衰マトリクスの拡充（モード別減衰・接線剛性比例 α1/h1 一定）

**対象:** `squid-n-solver/src/dynamic/damping.rs`、`squid-n-solver/src/dynamic/timehistory/mod.rs`、
`squid-n-app`（`ThDampingModel`・減衰セレクタ・減衰構成）。

RESP-D「07」減衰マトリクスの減衰タイプを拡充した（従来は剛性比例・Rayleigh のみ）。

- **モード別減衰:** `C = Σ_i 2·h_i·ω_i·(Mφ_i)(Mφ_i)ᵀ`（質量正規化モード φᵀMφ=1）。
  各モードに独立の減衰比 h_i を与える（`φ_jᵀCφ_j=2h_jω_j`、モード間直交）。
  固有ベクトルは既存の固有値解析（縮約空間・質量正規化）から取得。
- **剛性変更に伴う減衰項の変更（接線剛性比例）:** 非線形時刻歴で瞬間（接線）剛性から
  C を毎ステップ再構成する。**α1 一定**（`C=2h/ω1e·Kt`、`StiffnessProportional{Tangent}`
  を実効化）、**h1 一定**（`ω1=ω1e·√(uᵀKt u/uᵀKe u)` を毎ステップ更新し `C=2h1/ω1·Kt`）。
  Newton ループは元より毎反復で有効剛性を再分解するため、C 再構成の追加コストは小さい。
- **累積型/非累積型:** 減衰力の評価方式を選択（`DampingAccumulation`）。**非累積型**は
  瞬間 C×速度（`{Cn}=[Cn]{ẋn}`、既定）、**累積型**は増分減衰力の積分
  （`{Cn}={Cn−1}+[Cn]{Δẋn}`）。C が変化する接線比例・モード別で両者は異なり、C 一定なら
  一致する。非線形時刻歴 `nonlinear_time_history_analysis` に方式引数を追加。
- **配線:** アプリ時刻歴の減衰セレクタに「モード別／接線(α1一定)／接線(h1一定)」を追加。

**検証:** モード別 C が目標減衰比を再現・モード直交、接線 α1/h1 の C 再構成の手計算照合、
非線形時刻歴で 3 モデルとも収束し有限応答を返す end-to-end テスト、累積型/非累積型が
C 一定で一致し接線比例でも収束する検証（計8件）。

### 8. 制振要素（マクスウェルダンパー）

**対象:** `squid-n-core`（`ElementKind::Damper`・`DamperKind::Maxwell`・`DamperProps`・
`Model::damper_attrs`）、`squid-n-element/src/springs/damper.rs`（`MaxwellDamperElement`）、
`squid-n-element/src/common/behavior.rs`（`set_time_step`）、`squid-n-solver`（時刻歴配線）、
`squid-n-app/src/design_view.rs`（諸元表示）。

RESP-D「07」制振要素の**マクスウェル要素**（バネ `Kd` と粘性ダッシュポットの直列）を実装した。
「減衰要素の要素力は節点力として運動方程式へ与えられる」規定に対応する。

- **マクスウェル要素:** 2 節点軸方向要素。連結点変位 `Ud` を後退 Euler で毎ステップ更新し、
  要素力 `N=Kd(Uij−Ud)` を評価。線形（α=1）は閉形式 `Ud=(C0·Ud_前+Δt·Kd·Uij)/(C0+Δt·Kd)`、
  非線形（α≠1、`Fc=C0·sign(V)·|V|^α`）はスカラー Newton。収束用に整合接線
  `∂N/∂Uij=Kd·C'/(Δt·Kd+C')` を接線剛性へ与える（収束解は要素力の釣合いに一致し原典と等価）。
- **時刻歴配線:** `ElementBehavior::set_time_step(Δt)`（既定 no-op）を追加し、非線形時刻歴が
  build 後に全要素へ Δt を通知。`Δt<=0`（静的・線形）ではダンパーは不活性（力・剛性 0）。
  ダンパー要素は減衰・剛性マトリクス（初期 K/C）へは加算されず、要素力・接線として残差・
  接線側に入る。
- **表示:** 設計ビューに制振ダンパー諸元（種別・Kd・C0・α・緩和時間 τ=C0/Kd）を表示。

**検証:** 要素の不活性（Δt<=0）・高速載荷でのロック→緩和・整合接線の有限差分一致・非線形 α の
釣合い・軸方向力の DOF 配置（要素5件）、SDOF+ダンパー非線形時刻歴で自由振動応答が低減する
end-to-end 検証（1件）。

### 9. 質点系（串団子）モデルの生成

**対象:** `squid-n-solver/src/dynamic/lumped_mass.rs`（新規）、`squid-n-app`
（結果タブ「質点系モデル」ビュー・`AnalysisSettings::lumped_mass_type/lumped_secant_ratio`）。

RESP-D「07」の質点系解析モデル（立体骨組を層ごとの質点＋復元力バネへ縮約した
串団子モデル）の**生成機能**を実装した。立体フレームのプッシュオーバー（漸増静的）結果から
層ごとの層せん断力 Q・層間変形 δ 関係を抽出し、**等包絡面積則**でトリリニア骨格へ縮約する。

- **層骨格の縮約 `fit_story_trilinear`:** 層 Q-δ 曲線（`CapacityPoint::story_shear/story_drift`）
  から、初期剛性 K1＝第1点の割線勾配、第3折点（終局）＝曲線終端、第3勾配 K3＝終端接線
  （[0,K1] にクランプ）を定め、第1折点 δ1 は接線勾配が `secant_ratio·K1` を初めて下回る直前の
  変位（弾性限）とする。第2折点 δ2 は 0→終局のトリリニア包絡面積が実曲線の包絡面積に
  一致するよう線形に直接求解（Q2 は第3勾配直線上）。
- **モデル生成 `build_lumped_mass_model`:** 各層の質量＝地震重量 W/g（未設定時は節点質量の
  合計）、階高＝当該階標高−直下階標高、復元力＝上記トリリニア骨格で `StoryStick` を構成。
- **モデル化タイプ:** 等価せん断型／等価曲げせん断型／曲げせん断分離型を `LumpedMassType`
  として保持（現状は骨格縮約が主で、曲げ剛性の分離配線は今後の課題）。
- **配線:** 結果タブに「質点系モデル」ビューを追加。モデル化タイプ・第1折点割線比を選択でき、
  層ごとの質量・階高・K1/K2/K3・第1/第2/第3折点（δ・Q）を一覧表示する。プッシュオーバー
  結果から毎フレーム軽量に再構成する。

**検証:** `lumped_mass.rs`（等包絡面積・折点昇順・終端一致、K1≥K2≥K3 の3勾配縮約、
バイリニア入力の縮退耐性、空/1点の縮退の4テスト）+ app end-to-end（プッシュオーバー→
質点系生成）1件。GUI ビルド緑。

### 10. 制振ダンパーの GUI 作成・編集・削除と側テーブル整合

**対象:** `squid-n-core`（`Model::set_damper_props`・`ElemAttrs`・`take_elem_attrs`／
`restore_elem_attrs`・`shift_elem_attr_refs`）、`squid-n-edit`（`AddDamper`・
`SetDamperProps`・`DeleteMember`／`InsertMember` の側テーブル退避復元）、
`squid-n-app/src/tables/members.rs`（作成ボタン・ダンパー一覧編集）。

§8 でマクスウェルダンパー要素・時刻歴配線・諸元表示を実装したが、**GUI からの作成・編集
手段が無く**（API／IO 経由のみ）、アプリケーションとして未完だった。これを解消し、あわせて
要素キー付き側テーブルの参照整合バグを是正した。

- **作成・編集・削除 UI:** 部材表に「+ 制振ダンパー追加」ボタン（選択2節点間にマクスウェル
  要素を既定諸元で作成）と、制振ダンパー一覧（Kd[kN/mm]・C0・α を `DragValue` で編集、
  行削除）を追加。編集は `AddDamper`／`SetDamperProps`／`DeleteMember` として undo 可能。
- **原子的な作成:** `AddDamper` は要素（`ElementKind::Damper`）と特性（`damper_attrs`）を
  1 コマンドで追加し、逆操作 `DeleteMember` が両者を同時に取り消す。
- **側テーブル参照整合の是正（バグ修正）:** 従来 `shift_elem_ids`（要素削除・挿入時の ID
  繰上げ／繰下げ）は要素本体と部材荷重のみを対象とし、**要素キー付き側テーブル**
  （壁・鉄骨・BRB・PCa・免震・履歴則・ダンパーの 7 種）の `elem` 参照を更新していなかった。
  このため中間部材を削除すると、後続要素に紐づく履歴則・ダンパー等の属性が別要素へずれる／
  宙に浮く不整合が生じていた。`Model::shift_elem_attr_refs` で 7 側テーブルの参照も
  繰上げ／繰下げし、`take_elem_attrs`／`restore_elem_attrs`（`ElemAttrs` スナップショット）で
  削除要素自身の属性を退避・復元するよう是正した。§1・§8 で導入した側テーブル方式
  （`member_hysteresis_attrs`／`damper_attrs`）を含む既存の全側テーブルに一括で効く。

**検証:** edit 3 件（`AddDamper` 要素+特性の原子的追加・undo/redo、`SetDamperProps` 設定/解除/
undo・不存在部材 Noop、部材削除で側テーブル参照が繰上がり・削除属性の undo 復元）+ app
end-to-end 1 件（作成→諸元変更→削除を undo スタック経由）。既存の部材削除/挿入テスト
（`test_delete_member_middle_renumbers_and_roundtrips` 等）は全て緑（回帰なし）。

### 11. 鉄骨大梁の座屈を考慮した履歴（SteelBuckling）

**対象:** `squid-n-material/src/hysteresis.rs`（`SteelBuckling`・`lateral_buckling_mu_ratio`）、
`squid-n-core`（`HysteresisModel::SteelBuckling`）、`squid-n-element/src/factory/mod.rs`、
`squid-n-app`（部材表の履歴則選択）。

RESP-D「07」の鉄骨大梁の座屈を考慮した履歴則（局部座屈＝加藤・秋山・帯、横座屈＝井戸田
ほか 2015、連成座屈＝大谷・井戸田 2019）。除荷は孟・大井・高梨の **RO モデル**（γ=5, Φ=0.5）。

- **最大耐力比 `Mu/Mp`（横座屈）:** `lateral_buckling_mu_ratio(λb, κ, WF, eλb)` を原典式で実装。
  `Λc'=((λb/eλb)+WF/3)^(1/3)`、`h0=αΛ·Λc'(Λc'−1.25)+1`（Λc'≤1.25）、`qκ/r/αΛ` の κ 区分式、
  `cres=0, f=1, kres=0.3, kdef=1` の既定係数。連成座屈も Mu は横座屈と同式。
- **履歴則 `SteelBuckling`:** `M–θ` の `UniaxialMaterial`。骨格は 弾性→全塑性 Mp→歪硬化で
  最大耐力 Mu→劣化開始 θ_static から負勾配で残留耐力 Mu·mu_res へ低下、の耐力劣化型。
  除荷は反転点から初期剛性 k1 で立ち上がる RO 枝（`Δθ=(ΔM/k1)(1+Φ|ΔM/Mp|^(γ−1))` を Newton
  求解）。再載荷は経験最大点指向で骨格へ復帰（原典の完全な繰返し則の簡略化）。
- **配線:** 既定 Mu=1.1·My の座屈考慮型を材端バネへ適用（`build_flexural_springs`）。
  `set_yield` 対応（Mu も比率保持）で N-M 相関を適用可。部材表の履歴則選択に「座屈考慮型」を追加。

**検証:** material 5 件（Mu/Mp が細長比増で低下、骨格ピーク→劣化、RO 除荷初期剛性=k1、
1 サイクル散逸エネルギー>0、commit/revert）+ factory 1 件（鉄骨梁の座屈考慮バネが最大耐力後に
劣化）。

### 12. 制振要素の残装置（履歴型バイリニアダンパー）

**対象:** `squid-n-core`（`DamperKind::HystereticBilinear`・`DamperProps{qy,k2_ratio}`）、
`squid-n-element/src/springs/damper.rs`（`HystereticDamperElement`）、
`squid-n-element/src/factory/mod.rs`（種別分岐）、`squid-n-app`（部材表・設計ビュー）。

§8 で速度依存型（マクスウェル）を実装済み。本節で **履歴型（弾塑性バイリニア）ダンパー**を
追加した。RESP-D「07」制振要素で、鋼材系ダンパー（SUB／アンボンドブレース／二重鋼管座屈
補剛ブレース／鉛ダンパー／U 型ダンパー）はいずれも標準型バイリニアで、JFE LY（SLY100）も
非線形静的では K2 のバイリニアばねとしてモデル化されるため、この 1 要素で鋼材系の大半を
カバーする。

- **履歴型ダンパー要素:** 2 節点軸方向の弾塑性軸ばね。初期軸剛性 `k1`（=Kd）・降伏軸力 `qy`・
  第2剛性比 `k2_ratio` の `Bilinear`（kinematic hardening）を軸力–伸び則として用いる。
  変位依存のため **静的・動的いずれの解析でも作用**する（マクスウェルと異なり Δt 不要）。
- **種別分岐:** `DamperProps.kind` で `build_behavior` が要素を切替える
  （Maxwell→`MaxwellDamperElement`、HystereticBilinear→`HystereticDamperElement`）。
- **UI:** 部材表のダンパー一覧に種別セレクタと種別別諸元（マクスウェル: C0・α／履歴型:
  Qy・k2/k1）を追加。設計ビューに履歴型の k1・Qy・k2/k1・降伏変位 δy を表示。
- **後方互換:** `DamperProps.qy/k2_ratio` は `#[serde(default)]` で既存モデルを読込み可。

**検証:** element 3 件（弾性→降伏のバイリニア力、Δt 無しでも静的で作用、1 サイクル散逸
エネルギー>0）。既存のマクスウェル 5 件・時刻歴テストは全て緑（回帰なし）。

## 原典照合が必要な埋め込み値（技術リード確認用）

- 武田型トリリニアの材端骨格の簡略値: ひび割れ κ=0.56、降伏時剛性低下率 αy=0.3
  （既定）、終局倍率 1.1、塑性率 4。RESP-D 準拠の菅野 αy 精算はファイバ/skeleton 側
  （`build_rc_member_skeleton`）で行う方針であり、材端集中バネは簡略骨格を用いる。
- 武田型除荷指数 ν（`alpha`）既定 0.4。辻・山田型の既定 β=0.5・K2=0.01·k_rot。
- 累積損傷度の疲労特性 C・β（暫定既定 C=20, β=0.5。鋼種・接合形式で要照合）。

## 未実装項目（今後の課題／資料スコープ外）

RESP-D「07 非線形解析（動的解析）」との照合で確認したが**なお未着手**の主な差異
（規模が大きく、それぞれ別スライスとして対応するのが妥当）:

- **免震支承材の残装置・要素配線:** オイレス Tri-Linear/修正 H-D、錫プラグ、eRB、
  U 型ダンパー等の残装置。§6 は design-jp 純関数として算定式を実装したが、要素
  （`IsolatorElement`）は依然バイリニア/摩擦の汎用モデルで、γ 依存を毎ステップ反映する
  配線と、装置種別・ゴム総厚 H 等の入力（`IsolatorKind`/`IsolatorProps` 拡張）は未実装。
- **制振要素の残装置:** §8 でマクスウェル要素、§10 で GUI 作成・編集・削除、§12 で履歴型
  バイリニア（鋼材系＝SUB／アンボンドブレース／二重鋼管座屈補剛／鉛／U 型／JFE LY）を実装。
  残るは製品固有の粘弾性則（TRC、ユニットゴム修正 HD、RDT、GRAST 3 要素、筒状流体の
  温度・振動数依存粘度）で、多くは §8 のマクスウェル・§4 の辻山田・§12 の履歴型バイリニアの
  組合せに製品係数を与えて表現でき、`DamperProps` の係数拡張で対応する。
- **質点系（串団子）モデルの拡張:** §9 で 3D プッシュオーバー結果を層ごとのトリリニア
  stick へ縮約する生成機能（等価せん断型・等包絡面積則）と結果ビューを実装。残るは
  等価曲げせん断型／曲げせん断分離型の曲げ剛性分離配線、P-δ、スウェイ・ロッキング、
  免震層の考慮、生成した stick モデルでの独立した質点系時刻歴解析ステージ。
- **鉄骨大梁の座屈考慮履歴の精緻化:** §11 で SteelBuckling 履歴則（横座屈 Mu 式・耐力劣化
  骨格・RO 除荷）を実装。残るは断面の λb・κ・WF・eλb を自動算定して Mu/Mp を部材ごとに
  精算する配線（現状は既定 Mu=1.1·My）、および局部座屈/連成座屈の θ_static の文献精算。
- **位相差入力解析:** せん断波速度・入射角から位相遅れ時間を算定しねじれ加振を生成。
- **累積損傷度の梁端 μ 収集配線:** §5 のアルゴリズムは実装済み。時刻歴ソルバで梁端
  曲げ塑性率 μ の時刻歴を要素状態から収集し `cumulative_ductility` を実値化する配線が残る。
