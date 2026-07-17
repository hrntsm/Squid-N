# ST-Bridge 完全往復に向けた申し送り

目的: **ST-Bridge（XML 2.0 系）との完全な往復**（他社ソフト・BIM のファイルを読み込み、
書き戻しても幾何・断面・部材・材料・配筋が保存される）。

本 PR で「今直すべき MVP」を実装済み。残りは**バケット2（ライブラリ拡充）**・
**バケット3（パーサ拡充）**として別 PR で対応する。バケット2はモデル（`squid-n-core` /
`squid-n-section`）に新しい表現力を足す必要があるもの、バケット3はモデルは足りているが
`squid-n-io::stbridge` のパーサ/シリアライザだけを足すもの。

---

## バケット1: MVP（本 PR で実装済み）

- **id 正規化**（`import`）: node / material / story / element / load の各 id 空間を
  file id 昇順の 0 始まり連番へ正規化し、全参照（部材の節点・断面・材料、節点荷重の節点、
  節点の所属階）を張り替える。
  - 理由: 実 ST-Bridge は 1 始まり・歯抜けの id が普通で、内部モデルの不変条件
    「配列添字 == id.index()」を満たさず `validate` に弾かれていた。これが無いと
    他社ファイルは一切読めないため MVP とした。
  - 併せて: 断面の標準要素（`StbSecColumn_S` 等＋形鋼ライブラリ）の読取り、
    大文字 X/Y/Z 座標の受容も本 PR までに実装済み。

---

## バケット2: ライブラリ拡充（モデル側の表現力不足）

ST-Bridge にあり Squid のモデルが表現できないもの。**まず `squid-n-core` /
`squid-n-section` に型を足し**、その後パーサで往復させる。

### 断面形状（`SectionShape` に variant 追加）

現状: `SteelH / SteelBox / SteelAngle / SteelChannel / SteelTee / SteelPipe /
RcRect / RcCircle / SrcRect / CftBox / CftPipe / RcWall`。

不足（ST-Bridge の形鋼ライブラリ・RC 図形にあるが未対応）:
- **平鋼・鋼板**（`StbSecRoll-FlatBar` 等）
- **中実丸鋼**（solid round bar）
- **リップ溝形・軽量形鋼**（`StbSecRoll-LipC` 等の冷間成形材）
- **組立断面**: 2L（抱き山形）・2C（抱き溝形）・十字形（cross-H）などの built-up
- **非対称 H**（上下フランジ幅・厚が異なる `StbSecBuild-H`）。現 `SteelH` は左右上下対称前提。
- **RC の T 形・L 形梁**（スラブ一体のフランジ付き断面）、テーパ/ハンチ断面。
  ハンチは `member_detail_attrs` に一部あるが、断面図形としての表現は無い。

### 部材・トポロジ

- **通り芯・軸（`StbAxes`）**: `Model` に grid/axis の概念が無い。`Model.axes` を追加しないと
  通り芯が往復で失われる。
- **基礎・杭・フーチング**（`StbFooting` / `StbPile` / `StbFoundationColumn` /
  `StbStripFooting`）: 基礎系の部材型が無い。
- **間柱（`StbPost`）**: 柱/梁と別の意味を持つが対応する種別が無い（Beam 代用は可だが情報欠落）。

### 材料

- ST-Bridge は材料を鋼材/コンクリート/鉄筋で型分けし、規格名（SN400B・SD345・Fc24 等）で持つ。
  `Material` は `name + fy/fc` のみ。規格名・種別を確実に往復させるなら `Material` に種別/規格の
  フィールドを追加する（当面は `name` で代替可能）。

---

## バケット3: パーサ拡充（モデルは足りている。`squid-n-io::stbridge` のみ）

新しいモデル型は不要で、read/write を足すだけで往復可能になるもの。**「完全往復」への寄与が
大きい順**に並べる。

1. **RC 配筋の往復（最優先）**: ✅ **実装済み（Squid 出力どうし）**（本 PR）。
   `StbSecBarArrangementColumn_RC`（`StbSecBarColumn_RC_RectSame` / `_CircleSame`）・
   `StbSecBarArrangementBeam_RC`（`StbSecBarBeam_RC_Same`）へ主筋（方向別の本数・径・段数）・
   帯筋（径・ピッチ・組数・材質）・かぶりを read/write し、`RcRebar` を完全に往復させる。
   配筋の無い幾何のみのファイルは無筋相当で読む。
   - **残課題（実 ST-Bridge 配筋スキーマ完全準拠）**: 書き出す属性名は Squid 独自
     （`count_main_X`・`dia_main_X` 等）で、実 ST-Bridge の配筋属性（`D_main`・`N_main_X_1st`/
     `_2nd` の段別本数、呼び名→公称径の対応、`kind_corner` 等）とは異なる。import は
     `D_main`/`N_main_X_1st`/呼び名径（`D22`）を best-effort で拾うが、段別本数の合算や
     梁の上端/下端（`N_main_top`/`_bottom`）↔ 内部 `main_x`/`main_y`（せい/幅方向）の
     意味対応は近似。第三者ソフトとの厳密な配筋往復には実スキーマの read/write が必要。
   - **円形梁の非対応**: ST-Bridge に円形梁図形が無いため、`RcCircle` を梁に使う断面は
     `StbSecRaw` にフォールバックし形状・配筋が往復しない（円形柱は往復する）。
   - **未認識図形の断面欠落**: テーパ・ハンチ等 `StbSecColumn_RC_Rect`/`_Circle`・
     `StbSecBeam_RC_Straight` 以外の図形を持つ RC 断面は幾何を復元できず、import で断面が
     欠落する（参照部材は断面なし）。バケット2 の断面型追加＋下記 7（未対応要素の可視化）で対応。
2. **SRC / CFT 断面の標準要素対応**: ✅ **実装済み**（本 PR）。
   `CftBox`/`CftPipe` → `StbSecColumn_CFT`＋充填鋼管の `StbSecSteel` 参照（柱のみ。梁は Raw）。
   `SrcRect` → `StbSecColumn_SRC`/`StbSecBeam_SRC`（コンクリート図形＋内蔵鉄骨の `StbSecSteel`
   参照＋配筋 `StbSecBarArrangement*_SRC`＋鋼種 `strength_steel`）。いずれも形状・配筋・
   内蔵鉄骨・鋼種とも完全一致で往復（テスト済み）。
   - 残課題: CFT 梁（ST-Bridge に定義が無い）、SRC 円形・充腹/非充腹の別、実 STB の配筋
     スキーマ準拠（③-1 と同様）。
3. **材料参照の往復**: 断面と材料の関連（鋼断面の `strength_main`、RC の `id_material`）を
   ST-Bridge の形で書く／読む。現状 import は断面へ材料を結び付けていない。
4. **ブレース・壁・スラブ**: `StbBrace_S` ↔ `ElementKind::Brace`、`StbWall_RC` ↔ 壁、
   `StbSlab_RC` ↔ `slabs`。モデルは持っているのでマッピングのみ。
5. **実 ST-Bridge 構造への準拠**: 現状は自社方言（`StbMaterials` を `StbModel` 直下、
   `StbNode` に `story` 属性、`StbSecRaw` 独自要素）。他社完全互換には
   標準構造（材料は `StbCommon` 配下、node-story は `StbStory` の `StbNodeIdList` 経由、
   `StbAxes`、部材の `kind_structure`、単位系宣言）の read/write を実装する。
6. **テーパ/ハンチ/非一様鋼断面**: `StbSecSteelColumn_S_NotSame` / `_Taper` / `_Joint`、
   梁ハンチ図形の読取り（バケット2 の断面型追加とセット）。
7. **未対応要素の可視化**: 現状 SRC/CFT/基礎などは無警告で欠落する。import 時に
   「未対応でスキップした要素・断面」を集計して呼び出し側へ返し、データ欠損を顕在化させる
   （`Result` に警告リストを添える等）。

---

## 「完全往復」の限界（設計上の非対象）

- Squid 固有の解析・設計属性（`steel_design_attrs` / `brb_attrs` / `stress_cfg` 等）は
  ST-Bridge に対応表現が無く、STB 経由では往復しない（`.scz` を使う）。
- ST-Bridge 側の施工・製作情報等 Squid が使わない属性は取り込まない。
- したがって現実的な到達目標は「**STB→Squid→STB で、両モデルの共通部分（幾何・断面・部材・
  材料・配筋）が保存される**」こと。バケット2・3 を満たせばこの範囲で完全往復に到達する。
