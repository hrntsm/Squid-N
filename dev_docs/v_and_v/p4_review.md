# P4 レビュー報告（仕様 vs 実装の乖離・資料上の報告の正確性・仕様自体の妥当性）

本報告は `dev_docs/specs/P4_材料と断面.md`（設計書 §6.3/§7/§8/§18）と現状実装（`squid-n-material`・`squid-n-section`・`squid-n-skeleton`、および後続が依存する `squid-n-element`/`squid-n-design-jp` 周辺）を照合し、
「仕様通りか」「実装が間違っていないか」「資料上の報告が間違っていないか」「仕様自体がこのライブラリの目的（日本の建築構造計算一貫プログラム）に適するか」を審査した結果である。
前提: `cargo build --workspace` は成功。`cargo test` は（下記に示す緩い assert のため）緑になる可能性が高いが、**DoD（§8.4）は未達**。

## 0. サマリ

| 区分 | 件数・状況 |
|------|------|
| 実装の重大バグ | 11件（コンクリート包絡線の軟化逆転・接線符号・MP の ξ 更新非標準・ファイバ状態共有・スケルトン折点ヒューリスティック・配筋未使用 等） |
| 資料上の虚偽報告 | 2件（V&V 索引の P4 欠落／pending_items.md の P4 欠落） |
| 仕様未達（DoD §8.4） | #1 履歴則スナップショット・#2 ファイバ M–φ 厳密・#3 スケルトン手計算一致、いずれも未達 |
| 仕様書自体の不備 | 8件（単位規定・ファイバ状態管理・武田内側ルール・MP の ξ 更新式・スケルトン折点の規準式未記載・係数データ駆動の境界曖昧 等） |

結論：**P4 の完了基準（§0.2「M-φ／スケルトン自動生成」／§8.4 DoD チェックリスト）は未達**。
`squid-n-material` は「トレイトと目玉構造体はあるが、履歴則はデータのみで trial/commit/revert 未実装、コンクリート包絡線は物理的に軟化しない、MP はバウシンガーループ未検証」。
`squid-n-section` は「弾性域の積分は通るが、非線形履歴になると材料状態をファイバ間で共有して破綻」。
`squid-n-skeleton` は「M-φ を数値積分する骨格はあるが、配筋・材料・断面寸法を無視し、折点を規準式でなく勾配ヒューリスティックで拾うため手計算と一致しない」。
P3 と同じく「スケルトンはあるが数値が正しくない」状態であり、P5（非線形要素）・P6（動的）・P7（保有水平耐力）・P12（限界耐力）の土台として**構造技術者が信頼できない**。

---

## 1. 実装の重大バグ（材料則・ファイバ断面・スケルトン）

### 1.1 コンクリート圧縮包絡線が「軟化」せず逆進する ★致命的
`crates/squid-n-material/src/uniaxial.rs:305`

```rust
} else if strain >= self.ecu {
    let slope = 0.15 * self.fc / (self.ec0 - self.ecu);
    let stress = -self.fc + slope * (strain - self.ec0);
    let tangent = slope;
    (stress, tangent)
}
```

- `ec0=-0.002`, `ecu=-0.0035`。`ec0 - ecu = 0.0015 > 0`。`slope = 0.15*30/0.0015 = 3000 > 0`。
- 軟化域（`-0.0035 <= strain < -0.002`）で `strain - ec0 < 0`。`stress = -fc + 正*負 = -fc - ...` で**ピーク（-fc）よりさらに負（応力絶対値が増大）**に進む。これは「軟化」の定義（ピーク後ひずみ進行で応力が 0 側へ減少）と**完全に逆方向**。
- 正しい軟化は `stress = -fc + slope_pos*(strain - ec0)` で `slope_pos < 0`（ひずみ負方向に進むと応力は 0 側へ）。あるいは Popovics/Mander 系の残留式。
- 構造解析ライブラリとして、コンクリート圧縮包絡線が軟化しないと、柱の曲げ終局・プッシュオーバーの頂点以後・限界耐力の性能点が全て出ない。P4 の核心（M-φ→スケルトン）が物理的に無意味になる。

### 1.2 コンクリート圧縮の接線剛性の符号が逆 ★致命的
`crates/squid-n-material/src/uniaxial.rs:303`

```rust
let tangent = -(2.0 - 2.0 * ratio) * self.fc / self.ec0.abs();
```

- 放物線 `σ = -fc(2r - r²)`（`r = ε/εc0`）の接線 `dσ/dε = -fc(2/εc0 - 2ε/εc0²)`。
- `ε=-0.001, εc0=-0.002` で `dσ/dε = -30(2/-0.002 - 2*-0.001/4e-6) = -30(-1000 + 500) = +15000`。
- 実装は `-(1)*30/0.002 = -15000`。**符号が逆**。
- 接線剛性が負になると、ファイバ断面の接線 D（§1.7）の対応成分が負になり、`squid-n-skeleton` のニュートン法（`compute_m_phi_curve:98-110`）が分岐点で発散・誤収束する。構造解析の数値安定性に直結。

### 1.3 コンクリート包絡線が ecu 境界で不連続 ★致命的
`crates/squid-n-material/src/uniaxial.rs:311`

```rust
} else {
    let stress = -self.fc * (1.0 - 0.15 * (strain - self.ecu) / self.ecu);
    (stress, 0.0)
}
```

- `strain = ecu = -0.0035` で軟化式は `-30 + 3000*(-0.0015) = -34.5`。残留式は `-30*(1 - 0.15*0/...) = -30`。**-34.5 → -30 の跳び**。
- 不連続な包絡線は履歴則の収束反復で pivot を生み、ニュートン法が発散する。構造解析用材料則として成立しない。

### 1.4 コンクリート引張ひび割れ判定が即時 ON ★重大
`crates/squid-n-material/src/uniaxial.rs:363, 387`

```rust
let is_cracked = c.is_cracked || c.crack_strain > 0.0;
...
is_cracked: is_cracked || strain > c.crack_strain,
```

- `crack_strain` 初期値 0。最初の引張 trial（`strain > 0`）で `is_cracked = true` になる。
- ひび割れ前の弾性引張域（`0 < ε < ε_cr`）がほぼ再現されない。本来ひび割れは「ひずみが ε_cr=ft/E0 を超過」で発生。
- さらにひび割れ判定基準が `s >= ft*0.9`（line 378）と応力基準。放物線/指数包絡線のピーク ft に達する前（90%）でひび割れと判定するのは早すぎる。
- テスト `test_concrete_tension_crack:459` は `trial(0.00005) → s=0.75`（弾性）、`trial(0.0005) → s=0`（ひび割れ後 crack_e=0）で `stress2 < stress` を確認するだけ。**テンションスティフニング（ひび割れ後の引張保持）が検証されていない**。DoD T1「引張ひび割れ後に剛性低下」は形だけ満たし、§3「テンションスティフニング」は未達。

### 1.5 コンクリート/MP のデフォルト係数ハードコード ★仕様違反
`crates/squid-n-material/src/uniaxial.rs:269-272, 114, 129-131`

```rust
ec0: -0.002,
ecu: -0.0035,
tension_stiffening: 0.5,
...
let b = 0.01;
...
r0: 20.0, a1: 18.5, a2: 0.15,
```

- 仕様 §2.2「コードにハードコードしない。出典（規準・式番号）はコメントに残す」。§3「AIJ 系の応力–ひずみ式を選択可能（データ駆動）」。§4「代表値 R0≈20...（外部設定で可）」。§5「α・スリップ量は外部設定で上書き可」。
- `new()` がこれらを固定値で埋め、`with_params` で上書き可能とはいえ、デフォルト経路がハードコード。出典コメントも無い。
- 構造解析ライブラリでは、AIJ/JSCE 規準の包絡線（Popovics/Mander/Okamura-Higashi 等）を切り替えられるのが前提。固定の指数テンションスティフニング 1 本では、設計者が「どの規準式を採用したか」を追跡できない。

### 1.6 Menegotto–Pinto の反転点更新が 1 ステップ遅れ・ξ 式が非標準 ★致命的
`crates/squid-n-material/src/uniaxial.rs:206-224`

```rust
fn commit(&mut self) {
    let prev = &self.committed;
    let sgn_now = (self.trial.strain - prev.eps_r).signum();
    let sgn_prev = (prev.strain - prev.eps_r).signum();
    if sgn_now != sgn_prev && sgn_prev != 0.0 {
        ...
        let xi_new = prev.xi + deps * dsig.signum() - (self.b / (1.0 - self.b)) * dsig / self.e;
        let xi_new = xi_new.abs().max(0.0);
        ...
    }
    self.committed = self.trial.clone();
}
```

- 標準 MP（Filippou-Ceresa 形）は **trial 内で反転を検知**し、反転点 (εr,σr) と漸近線交点 (ε0,σ0) を更新する。実装は **commit 時に反転を検知**するため、反転ステップの trial 計算は**古い (εr,σr) と (ε0,σ0)** を使ってしまい、反転直後のループ形状が 1 ステップずれる。
- `xi_new = prev.xi + deps*dsig.signum() - (b/(1-b))*dsig/E` は標準の ξ 更新式（`ξ = |εr - εr_prev| / εy` 系）と一致しない。`xi_new.abs().max(0.0)` で abs を取るのも非標準（式が正しければ ξ は元々非負）。
- 漸近線交点の更新 `eps_0 = new_r + deps, sig_0 = new_s + dsig` は「反転点を前の反転点に対して鏡映」した近似。初回反転で `eps_r=0` なら `eps_0 = 2*new_r` となり、反転点が降伏点から離れていると漸近線交点が大きくずれる。
- テスト `test_menegotto_pinto_elastic:445` は**弾性域のみ**（ε=0.001, fy=235, E=205000 → 弾性）。バウシンガー効果のループ検証が**存在しない**。DoD T2「MP がバウシンガー効果を示すループ（参照と一致、§8.1）」完全未達。

### 1.7 ファイバ断面が材料状態をファイバ間で共有して破綮 ★致命的
`crates/squid-n-section/src/fiber.rs:38-76`

```rust
pub fn section_response(
    sec: &FiberSection,
    strain: SectionStrain,
    mats: &mut [Box<dyn UniaxialMaterial>],
) -> (SectionForce, SectionStiffness) {
    ...
    for fiber in &sec.fibers {
        let eps = strain.eps0 - strain.kz * fiber.y + strain.ky * fiber.z;
        let mat = &mut mats[fiber.material];
        let (sigma, et) = mat.trial(eps);
        ...
    }
}
```

- `mats` は「材料インデックス → 単一状態インスタンス」。`rect_fiber_section:79` は全ファイバに `material=0` を割り当てる。`test_section_pure_bending:133` は 20×40=800 ファイバ全て material=0。
- 同じ `mat` を 800 回 `trial(eps)` すると、trial 状態が毎回上書きされ、`commit()` したとき最後のファイバの状態だけ残る。
- 弾性域（Bilinear の弾性分枝）では trial がひずみの線形関数なので上書きでも積分結果は正しい。これがテストが通る理由。
- しかし非線形履歴では trial 応力が**履歴状態に依存**し、共有状態だと「前のファイバの履歴」が次のファイバの trial に混入する。M-φ 計算（`squid-n-skeleton::compute_m_phi_curve`）で κ を増やしながら全ファイバ trial すると、各ファイバの降伏進展が追跡できず、**M-φ が物理的に無意味**になる。
- 正しくは「各ファイバが独自の材料状態インスタンスを持つ」。仕様 §6 の `mats: &mut [Box<dyn UniaxialMaterial>]` を「ファイバ数分の状態配列」とするか、`Fiber` が状態を直接持つ必要。実装は材料IDで共有する設計なので、非線形ファイバ積分が破綮。DoD T4「降伏進展でファイバが順次降伏し M–φ がトリリニア状に折れる」未達。構造解析のファイバ断面としては成立しない。

### 1.8 `Fiber.material` の型が仕様と不一致
`crates/squid-n-section/src/fiber.rs:9`

```rust
pub material: usize,
```

- 仕様 §6 の雛形: `pub material: MaterialId`。実装は `usize`。「ライブラリAPIは調査不要・写譯で埋める」（dev_docs/specs/README §5）の方針に反し、型が違う。`MaterialId` を使えば `model.materials[mat_id.index()]` で参照でき、`squid-n-core` との整合が取れる。

### 1.9 `section_response` の接線剛性は正しいが、単位規定がない
`crates/squid-n-section/src/fiber.rs:38` の doc に単位が無い。`Fiber.y/z` が mm か m か、`area` が mm² か m² か、`SectionStrain.eps0/ky/kz` の単位（無次元・1/mm・1/m）。`SectionForce.n/my/mz` が N・N·mm か kN・kN·m か。
- P3 レビュー §4.1 と同じ問題が P4 にもある。`squid-n-skeleton::MemberData.span` は `[mm]` と明記（§7）するのに、`FiberSection` 側は無指定。N·mm 系（部材長 mm、応力 N/mm²）で統一するなら明記すべき。単位不整合は P3 の 1000 倍バグの再現を許す。

### 1.10 `HysteresisRule` がデータのみで trial/commit/revert 未実装 ★致命的
`crates/squid-n-material/src/hysteresis.rs:5-69`

```rust
pub enum HysteresisRule { Takeda { ... }, TakedaDegrading { ... }, OriginOriented { ... }, Slip { ... } }
impl HysteresisRule {
    pub fn unloading_stiffness(&self, max_deformation: f64) -> Option<f64> { ... }
}
```

- `UniaxialMaterial` トレイト（§2）を実装していない。`unloading_stiffness` の式だけ。
- 仕様 §5「武田モデルの核: スケルトン＋除荷剛性＋再載荷＋内側ループ」。DoD T3「規定の繰り返し変位履歴で各則のループが参照と一致（insta、§8.1）」。
- 再載荷「反対側の最大経験点を指向」、内側ループ「武田の規則」が未実装。`squid-n-element::shear_spring.rs:8` が `MemberSkeleton`（`hysteresis: HysteresisRule`）を保持するが、trial/commit/revert が無いと P5 の集中ばね要素で M-θ 履歴を計算できない。
- DoD T3 完全未達。武田モデルは RC の曲げ履歴則の中核であり、P6（時刻歴）・P7（保有水平耐力）・P12（限界耐力）の精度を左右する。構造解析ライブラリとして「武田が動かない」は致命的。

### 1.11 スケルトン折点が規準式でなく勾配ヒューリスティック ★致命的
`crates/squid-n-skeleton/src/lib.rs:126-167`

```rust
fn extract_trilinear(mphi: &[MPhiPoint]) -> Vec<(f64 /* θ */, f64 /* M */)> {
    ...
    let (crack_idx, _crack_m) = mphi[i_start..].iter().enumerate()
        .find(|(_, p)| p.moment > 0.0 && p.curvature > 1e-12)
        ...
    let yield_idx = (crack_idx + 1..n).find(|&i| {
        let dmdk = ...;
        let init_slope = ...;
        init_slope > 0.0 && dmdk / init_slope < 0.1
    })...
    let ultimate_idx = (yield_idx..n).rev()
        .find(|&i| i > 0 && (mphi[i].moment - mphi[i - 1].moment).abs() < 1.0)
        ...
}
```

- `crack_idx` = 最初の `moment>0 && curvature>1e-12` の点。これは「最初の非零サンプル」であって、ひび割れ点 Mc/φc ではない。
- `yield_idx` = 割線剛性が初期勾配の 10% 未満になった点。「ほぼ平坦」で降伏とは限らない。
- `ultimate_idx` = 逆順で隣接サンプルのモーメント差が 1.0（N·mm? kN·m? 単位依存のマジックナンバー）未満の点。「ほぼ平坦」で終局とは限らない。
- 仕様 §7 フロー2「折れ点の決め方は設計規準準拠」。DoD §8.3「ひび割れ Mc・降伏 My・終局 Mu が規準式の手計算と一致」。実装は M-φ 曲線の形状から勾配で拾うだけ。RC 規準式（Mc=N·(h/2-d)..., My=a_t·ft·j, Mu=0.9·a_t·σy·j 等）を一切使わない。
- テスト `test_member_skeleton_basic:234` は `points が空でない` `last.M >= first.M` のみ。数値一致の検証なし。
- DoD §8.3 完全未達。スケルトン折点が手計算と一致しないと、P5 プッシュオーバーの降伏荷重・P7 保有水平耐力の Qu/Ds・P12 限界耐力の性能点が全て規準と合わない。

### 1.12 スケルトン算定が配筋・材料・断面寸法を無視 ★致命的
`crates/squid-n-skeleton/src/lib.rs:191-226`

```rust
pub fn build_member_skeleton(member: &MemberData, n_axial: f64, mats: &mut [Box<dyn UniaxialMaterial>]) -> MemberSkeleton {
    let max_curvature = 0.01;
    let num_steps = 200;
    let mphi = compute_m_phi_curve(member.fibers, mats, n_axial, max_curvature, num_steps);
    ...
}
```

- `MemberData` に `section`/`reinforcement`/`material` を渡すが、`build_member_skeleton` は `fibers` と `mats`（引数）だけ使い、`section.as_y/as_z/depth/width`・`reinforcement.main_bars`・`material.fc/young` を一切参照しない。
- RC 断面の M-φ は鉄筋の応力を積分に含めなければ出ない。実装は `fibers`（コンクリートのみ想定）で積分し、鉄筋を無視。`Reinforcement::main_bars`（位置・断面積）がスケルトンに反映されない。
- `MemberSkeleton::axial_dependency.skeletons` に自分自身（`points: vec![]` の空スケルトン）を入れる（line 215-224）。N-M 相関の複数 N レベルでのスケルトン保持（仕様 §7「n_axial を変えて複数スケルトン」）が未実装。1 レベルのみ。
- 仕様 §7「断面・配筋・材料・スパン・想定軸力からスケルトン曲線を自動生成」に対し、配筋と材料と断面寸法が抜け、軸力依存も 1 点のみ。**RC 梁・柱のスケルトンが算定できない**。DoD §8.3「代表 RC 梁・柱で…」は前提から成立しない。

### 1.13 `mphi_to_mtheta` がせん断・抜出し・付着を加算しない ★未達
`crates/squid-n-skeleton/src/lib.rs:171-184`

```rust
fn mphi_to_mtheta(mphi_points: &[(f64, f64)], span: f64, inflection_ratio: f64) -> Vec<(f64, f64)> {
    mphi_points.iter().map(|&(phi, m)| {
        if phi.abs() < 1e-15 { (0.0, 0.0) }
        else {
            let l = span * inflection_ratio;
            let theta = phi * l / 3.0;
            (theta, m)
        }
    }).collect()
}
```

- 仕様 §7 フロー4「せん断・付着・抜出しによる回転を加算」。実装は `θ = φ·l/3` のみ（曲率分布を三角形と仮定）。せん断変形・鉄筋抜出し・付着すべりによる回転加算が無い。
- `φ·l/3` は反曲点から端まで曲率が線形分布すると仮定した弾性域の近似。降伏後（塑性ヒンジ）は曲率分布が矩形に近づき、`φ·lp`（lp=塑性ヒンジ長）の項が支配的になる。固定 `l/3` では降伏・終局の θ が過小。
- RC の変形評価では抜出し・せん断が部材回転の主要因（特に短期・限界）。これらを省くと、P6 時刻歴の層間変形・P12 限界耐力の安全限界変形が規準と合わない。

### 1.14 `build_member_skeleton` の係数ハードコード
`crates/squid-n-skeleton/src/lib.rs:196-197, 209`

```rust
let max_curvature = 0.01;
let num_steps = 200;
...
alpha: 0.4,
```

- `max_curvature=0.01`（1/m? rad? 単位依存）、`num_steps=200`、`alpha=0.4` をハードコード。仕様 §2.2/§5「外部設定」違反。`max_curvature` は断面寸法・鉄筋・軸力に依存するはずで固定値は不適。

---

## 2. 仕様 vs 実装の未達項目（DoD §8.4）

### 2.1 DoD #1「履歴則 参照ループ一致（§8.1）」★完全未達
- `insta` はワークスペース依存（`Cargo.toml:25`）に存在するが、`squid-n-material/Cargo.toml` は `approx` のみで `insta` 未追加。
- `crates/` 全体で `insta` を使ったテストは 0 件（grep `insta|snapshot` → squid-n-gpu の wgpu instance の誤ヒットのみ）。
- 各履歴則（Concrete/Bilinear/MP/Takeda）の「規定の繰り返し変位履歴に対するループ」を固定したスナップショットが存在しない。
- 武田・原点指向・スリップは §1.10 で示すとおり trial/commit/revert すら未実装。ループの描画以前の問題。

### 2.2 DoD #2「ファイバ M–φ 理論一致（弾性厳密 1e-9）」★一部未達
- `test_section_axial:111` は `epsilon=1.0`（N 単位の絶対誤差）。仕様 §8.2 は「弾性域は厳密、相対 1e-9」。1.0 N の誤差を許すテストは厳密でない。
- `test_section_pure_bending:132` は `epsilon=expected_my*0.01`（1% 相対）。1e-9 でない。
- 単軸圧縮のテストが存在しない。
- 非線形域「降伏進展でトリリニア状に折れる」のテストが存在しない（§1.7 で破綮するため当然）。

### 2.3 DoD #3「スケルトン折れ点 手計算一致（§8.3）」★完全未達
- 規準式（Mc/My/Mu）を使った手計算との一致テストが存在しない。
- `test_member_skeleton_basic` は「空でない」「単調増加」のみ。
- 軸力を変えて柱のスケルトンが変化する検証（§8.3 後段）も未実装（§1.12 で 1 レベルのみ）。

### 2.4 T0/T1/T2/T3/T4/T5 の完了状況
| タスク | DoD | 状況 |
|---|---|---|
| T0 UniaxialMaterial トレイト | trial/commit/revert 動作 | ✅ トレイトは成立（ダミー線形で確認）。ただし Bilinear の繰り返し挙動は原点指向的に振る舞う可能性（§1 補足）で、履歴則としての妥当性は T2 で検証すべき |
| T1 コンクリート | 包絡線規準式一致・ひび割れ剛性低下・テンションスティフニング・除荷再載荷 | 🔶 包絡線ピーク点だけ一致、軟化逆進・接線符号逆・ひび割れ即時ON・テンションスティフニング未検証 |
| T2 鋼・鉄筋 | バイリニア単調・繰り返し・MP バウシンガー | 🔶 バイリニア単調のみ、MP は弾性域のみテスト、バウシンガー未検証 |
| T3 履歴則 | 武田・原点指向・スリップのループ一致 | ❌ trial/commit/revert 未実装、unloading_stiffness の式のみ |
| T4 ファイバ断面 | 単軸/曲げ理論一致・降伏進展 | 🔶 弾性域は概ね一致（厳密でない）、非線形は破綮・降伏進展未検証 |
| T5 スケルトン | 折点手計算一致・軸力依存 | ❌ 規準式不使用・配筋無視・軸力 1 レベル・せん断抜出し加算なし |
| T6 テスト・DoD | §8.1/§8.2/§8.3 | ❌ insta 0件・非線形 M-φ テスト無・手計算一致テスト無 |

---

## 3. 資料上の報告の正確性

### 3.1 V&V 索引（`dev_docs/v_and_v/README.md`）に P4 項目が存在しない ★過小申告
`dev_docs/v_and_v/README.md:44-64` の索引 #1〜#18 に、P4 の核心（履歴則・ファイバ断面・スケルトン）の行が無い。#15 プッシュオーバー（P5）、#16 壁（P5.5）、#17 時刻歴（P6）、#18 限界耐力（P12）はあるのに、**P4 の「履歴則」「ファイバ断面」「スケルトン自動算定」が欠落**。
- P4 が完了扱いで索引に載せられていないことは、P3 レビュー §3.2 と同じく過小申告。P4 の DoD §8.4 の 3 項目を索引に加え、現状は ❌ または 🔶 と明記すべき。

### 3.2 `pending_items.md` に P4 の未達項目が記載されていない ★過小申告
`dev_docs/v_and_v/pending_items.md` は P3（§7）・P5（§1）・P5.5（§2）・P6（§3）の未達を記録するが、**P4 のセクションが存在しない**。経過表（§経過:143-149）も P3/P5/P5.5/P6 のみで P4 が抜ける。
- P4 が「完了」扱いで未達が記録されていない。P3 レビューを受けて P3 項目は充実したが、P4 は未対応。本レビューの §1/§2 を pending_items.md に P4 セクションとして起票すべき。

### 3.3 `dev_docs/specs/README.md` の状態表は「作成済」で妥当
`dev_docs/specs/README.md:22` の P4 行「作成済」は仕様書の作成状態を指し、実装完了ではない。これは README の文脈（「状態」=仕様書有無）として妥当。ただし実装完了とは別であり、P4 仕様書の §0.2「完了像」が満たされていないことをどこかに明示する必要がある（現状どこにも明記なし）。

---

## 4. 仕様書自体の問題点（構造解析ライブラリとしての妥当性）

### 4.1 単位規定の欠落（P3 §4.1 と同根）★根本
- §6 `Fiber.y/z/area`、`SectionStrain.eps0/ky/kz`、`SectionForce.n/my/mz` に単位が無い。
- §7 `MemberData.span [mm]` だけ明記。ファイバは mm か m か。曲率 ky/kz は 1/mm か 1/m か。モーメント my/mz は N·mm か kN·m か。
- 構造解析ライブラリでは解析系（P1/P2）と材料/断面系（P4）で単位を固定するか明示変換するのが鉄則。P3 で `MemberForcesAt` の単位不備が 1000 倍バグを生んだ（P3 レビュー §1.1）。P4 も同じ穴を抱え、実装 §1.9 で指摘したとおり `FiberSection` に単位 doc が無い。
- **是正案**: §6 に「`y/z: mm`, `area: mm²`, `eps0: 無次元`, `ky/kz: 1/mm`, `n: N`, `my/mz: N·mm`」を明記。P2 `MemberForces` と整合させる。

### 4.2 ファイバの材料状態管理が未規定 ★根本
- §6 `section_response(sec, strain, mats: &mut [Box<dyn UniaxialMaterial>])`。`mats` が「材料種別ごとに 1 インスタンス」か「ファイバごとに 1 インスタンス」か曖昧。
- 非線形履歴では各ファイバが独立した履歴状態を持つ必要（§1.7）。仕様がこれを明記しないため、実装は「材料IDで共有」を作って破綮した。
- **是正案**: §6 を「`mats` は `fibers.len()` と同じ長さで、`fiber.material` は `mats` へのインデックス。各ファイバが独自の状態インスタンスを持つ」と改める。`rect_fiber_section` は全ファイバに同じ材料パラメタで個別インスタンスを生成するよう修正。

### 4.3 `HysteresisRule` が enum データのみで UniaxialMaterial に非準拠 ★根本
- §5 の雛形 `pub enum HysteresisRule { Takeda { ... }, ... }`。これを `UniaxialMaterial` トレイト実装にするのか、データ構造にするのか書かれていない。
- 集中ばね要素（P5 §3）で M-θ 履歴を計算するには、`HysteresisRule` が `UniaxialMaterial` として振る舞う必要。仕様が enum データを示したことが、実装を「データのみ」に誘導した。
- **是正案**: §5 を「`HysteresisRule` は `UniaxialMaterial` を impl する enum（または trait object で保持）。`trial(theta) -> (M, Kt)`、`commit/revert` を実装」と改める。

### 4.4 武田モデルの「内側ループ規則」が未記述 ★根本
- §5「内側ループ: 上記の点を結ぶ（武田の規則）」。「武田の規則」の具体（反転点・最大経験点・降伏点を結ぶポリゴン、スリップ型の再載荷寝かせ量）が書かれていない。
- 実装は `unloading_stiffness` だけで内側ループを省略。仕様の曖昧さが省略を招いた。
- **是正案**: §5 に武田の内側ルール（反転点指向・最大経験点指向・原点指向の使い分け、スリップ係数の適用箇所）を明示。原論文（Takeda et al. 1970）または AIJ『非線形解析指針』の図を引用。

### 4.5 Menegotto–Pinto の ξ 更新式が未記述 ★根本
- §4「曲率: R = R0 − a1·ξ / (a2 + ξ)（ξ=反転後の塑性ひずみ振幅）」。ξ の更新式が無い。
- 実装は `xi_new = prev.xi + deps*dsig.signum() - (b/(1-b))*dsig/E`（§1.6）と独自式を埋めた。標準（Filippou-Ceresa）は `ξ = |εr − εr_prev|/εy` 系。
- **是正案**: §4 に ξ の更新式（Chang & Mander 2004 または Filippou-Ceresa）を明示。反転点 (εr,σr) と漸近線交点 (ε0,σ0) の更新手順も併記。

### 4.6 スケルトン折点の規準式が未記述 ★根本
- §7 フロー2「折れ点の決め方は設計規準準拠」。DoD §8.3「手計算（規準式）と一致」。しかし §7/§8.3 に Mc, My, Mu の具体式が無い。
- 実装はヒューリスティック（§1.11）。仕様が式を書かないため、実装者が「M-φ 曲線の形状から拾う」を作った。
- RC の曲げ耐力式は AIJ 規準・令91条で公有（Mc=引張縁ひび割れモーメント、My=a_t·σy·j、Mu=0.9·a_t·σy·j＋N·(d−x)/2 等）。これらは「外部データ」ではなく本書で確定できる。
- **是正案**: §7 に Mc/My/Mu の基本式と φc/φy/φu の算定式（鉄筋降伏ひずみと断面有効高から、あるいはファイバ積分の降伏・圧壊点）を明記。

### 4.7 係数データ駆動の境界が曖昧 ★仕様違反の温床
- §2.2「コードにハードコードしない」。§4「代表値 R0≈20...（外部設定で可）」。§5「α≈0.4（外部設定）」。
- 「ハードコード禁止」と「代表値を本文に書く」が矛盾せず共存するには、「代表値は外部データのデフォルト値として参照し、`new()` に埋め込まない」という運用規則が必要。仕様がこれを書かないため、実装は `new()` にハードコード（§1.5）。
- **是正案**: §2.2 に「`new()` はデフォルトパラメタを与えない（`with_params` のみ、またはデフォルトは外部データファイルから読込）。本文の代表値はドキュメンテーション用」と明記。

### 4.8 せん断履歴とスケルトン算定の連携が未規定
- §5 に `OriginOriented`/`Slip`（せん断履歴）。§7 フロー4「せん断・付着・抜出しによる回転を加算」。しかし §7 でせん断履歴則をどうスケルトンに組み込むかが未記述。
- 実装はせん断加算をスキップ（§1.13）。仕様の空白が省略を招いた。
- **是正案**: §7 に「曲げスケルトン（M-θ）とせん断スケルトン（Q-δ）を独立に算定し、部材端変形に加算」の経路を明示。

---

## 5. 推奨対応（優先度順）

1. **【最優先】コンクリート包絡線の軟化・接線符号・境界連続性を修正**
   - `envelope_compression` の軟化勾配を負に（ピーク後ひずみ進行で応力 0 側へ）。
   - 放物線の接線 `dσ/dε` の符号を +（圧縮側で dσ/dε > 0）に。
   - ecu 境界で軟化式と残留式が連続するよう残留式の切片を調整。
   - 軟化域のテストを追加（-0.003 で応力が -fc より 0 側にあることを検証）。
2. **【最優先】ファイバ断面の材料状態をファイバごとに独立化**
   - `mats: &mut [Box<dyn UniaxialMaterial>]` を `fibers.len()` に揃える。`Fiber.material` は `mats` へのインデックス。
   - `rect_fiber_section` は同じパラメタで個別インスタンスを生成。
   - 非線形 M-φ テスト（降伏進展でトリリニア状に折れる）を追加。
3. **【最優先】`HysteresisRule` に `UniaxialMaterial` を impl**
   - 武田（スケルトン＋除荷＋再載荷＋内側ルール）・原点指向・スリップの trial/commit/revert を実装。
   - `insta` を `squid-n-material` の dev-dependency に追加し、§8.1 のスナップショットテストを作成。
4. **【最優先】スケルトン折点を規準式で算定**
   - Mc/My/Mu/φc/φy/φu の規準式を §7 に明記のうえ実装。`extract_trilinear` のヒューリスティックを廃止。
   - `Reinforcement.main_bars` と `Material.fc/young` を M-φ 計算に組み込む（鉄筋ファイバを追加）。
   - 代表 RC 梁・柱の手計算例と一致するテストを追加（§8.3）。
5. **MP の反転点・ξ 更新を標準式に**
   - trial 内で反転検知。ξ は Chang & Mander または Filippou-Ceresa の標準式。
   - バウシンガーループのスナップショットテストを追加。
6. **係数のハードコード除去**
   - `Concrete::new`/`MenegottoPinto::new` のデフォルト埋込を廃止。外部データまたは `with_params` 必須化。
   - `build_member_skeleton` の `max_curvature/num_steps/alpha` を外部設定化。
7. **V&V 索引と pending_items.md の補完**
   - `dev_docs/v_and_v/README.md` の索引に「履歴則」「ファイバ断面」「スケルトン自動算定」を追加。現状は ❌。
   - `dev_docs/v_and_v/pending_items.md` に P4 セクションを新設し、§1/§2 の未達を記載。
8. **仕様書の改訂**
   - §6 に単位規定（mm/mm²/1/mm/N/N·mm）を明記。
   - §6 にファイバ状態管理規則（ファイバごとに独立インスタンス）を明記。
   - §5 に武田内側ルール・MP の ξ 更新式を明記。
   - §7 に Mc/My/Mu 規準式とせん断・抜出し加算経路を明記。
   - §2.2 に「`new()` へのハードコード禁止、代表値は外部データ参照」を明記。
9. **P5 移行前に P4 の DoD を再判定**
   - P5（非線形要素）は P4 の `MemberSkeleton`/`FiberSection`/`HysteresisRule` に依存。現状では P5 を実装しても数値が信頼できない。P4 §8.4 の 3 項目が緑になるまで P5 に移行しない。

---

## 6. 結論

P4 は「型とトレイトの骨格はあるが、履歴則はデータのみで未実装、コンクリート包絡線は軟化せず逆進する、ファイバ断面は非線形で状態共有破綮、スケルトンは配筋を無視し折点を規準式でなくヒューリスティックで拾う」状態である。
`cargo test` が緑でも DoD §8.4 の 3 項目（履歴則スナップショット・ファイバ M-φ 厳密・スケルトン手計算一致）はいずれも未達であり、V&V 索引と pending_items.md に P4 の未達が記録されていないことは過小申告にあたる。
P4 は非線形解析（P5/P6/P7/P12）の土台であり、ここが不正確だと日本の建築構造計算一貫プログラムとしての核心（M-φ→スケルトン→保有水平耐力→限界耐力）が通らない。P3 と同じく「実装・テスト・資料報告の三層で整合を取り直す」必要がある。
