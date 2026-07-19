# モデル入出力（ファイル形式）

Squid-N は構造モデルを 2 つのファイル形式で入出力する。

| 形式 | 拡張子 | 用途 | 往復精度 |
|---|---|---|---|
| Squid-N プロジェクト | `.scz` | Squid-N ネイティブの保存形式。モデルを欠損なく保存・読込する | 完全一致 |
| [ST-Bridge](https://www.building-smart.or.jp/meeting/buildingsmart/st-bridge/) | `.stb` / `.xml` | 他社の一貫計算プログラムや BIM ツールとのモデル受け渡し | サブセットのみ意味的一致 |

いずれも GUI アプリ（`squid-n-app`）のファイルメニューから利用でき、実装は `squid-n-io` クレートにある。**日常的な保存・読込は `.scz`**、**他ツール連携は `.stb`** と使い分ける。

- 実装: `.scz` は `squid-n-io::scz`（`save_scz` / `load_scz`）、ST-Bridge は `squid-n-io::stbridge`（`export_stbridge` / `import_stbridge`）。

---

## Squid-N プロジェクト形式（.scz）

Squid-N ネイティブの保存形式。内部モデルをそのまま格納するため、保存・読込でモデルが**完全一致**する（ST-Bridge のようなサブセット制約は無い）。

### GUI からの操作

ファイルメニューに以下の項目がある。

| メニュー | 動作 |
|---|---|
| 📄 **新規** | 空モデルを開く |
| 🏠 **サンプル(門型ラーメン)** | 内蔵のサンプルモデルを開く |
| 📂 **開く…** | `.scz` プロジェクトを読み込む。読込後、モデルは検証（`validate`）を通ってから差し替わる |
| 💾 **保存** | 現在のプロジェクトを上書き保存する（保存先が未設定なら保存先を尋ねる） |
| 💾 **名前を付けて保存…** | 保存先を指定して `.scz` として保存する |

読み込むとそのファイルが現在のプロジェクトの保存先になり、以降の「保存」は同じファイルへ上書きする。

### ファイル構造

`.scz` は 3 つのエントリを持つ ZIP 書庫である。

| エントリ | 内容 |
|---|---|
| `manifest.json` | スキーマ版、単位系、各エントリの SHA-256 ハッシュ |
| `model.msgpack` | 内部モデル本体（MessagePack でシリアライズ） |
| `settings.json` | 設計コード等の設定 |

内部モデルには節点・部材・断面・材料・荷重・層に加え、断面形状（`SectionShape`）や部材付帯情報（ハンチ・継手位置）まで含まれ、これらは保存・読込で保持される。

### 整合性・安全性

- **ハッシュ検証**: 読込時に各エントリの SHA-256 を `manifest.json` の記載と照合し、不一致ならエラーにする。必須エントリ（`model.msgpack` / `settings.json`）が manifest に列挙されていない場合も拒否する（未検証のまま読み込ませないため）。
- **スキーマ版チェック**: 現行スキーマ版（`1`）以外は拒否する。本ソフトはリリース前のため後方互換は持たない。
- **原子的な保存**: 一時ファイルへ書き出して `fsync` した後に `rename` することで、電源断でファイルが破損しないようにしている。
- **ZIP 爆弾対策**: 1 エントリあたりの展開サイズ上限（512 MiB）を超える書庫は拒否する。

### ライブラリからの利用

```rust
use squid_n_io::scz::{save_scz, load_scz};
use std::path::Path;

// 保存: 内部モデル → .scz
save_scz(Path::new("model.scz"), &model)?;

// 読込: .scz → 内部モデル
let model = load_scz(Path::new("model.scz"))?;
```

---

## ST-Bridge 形式（.stb / .xml）

[ST-Bridge](https://www.building-smart.or.jp/meeting/buildingsmart/st-bridge/)（XML, 2.0 系）で**読み込み・書き出し**できる。他社の一貫計算プログラムや BIM ツールとモデルを受け渡すための入出力経路である。

### GUI からの操作

| メニュー | 動作 |
|---|---|
| 📥 **ST-Bridge 読込…** | `.stb`（または `.xml`）ファイルを選び、内部モデルへ取り込む。取り込んだモデルは検証（`validate`）を通ってから現在のモデルと差し替わる |
| 📤 **ST-Bridge 書出（物性）…** | 断面を物性直持ち（`StbSecRaw`）で書き出す。Squid-N 同士の受け渡し向け（読込で往復可能） |
| 📤 **ST-Bridge 書出（断面形状）…** | 断面を ST-Bridge 標準要素＋形鋼ライブラリで書き出す。BIM・他ソフト向け |

- ファイル選択ダイアログの拡張子フィルタは `.stb` / `.xml`。
- ST-Bridge 読込は `.scz` プロジェクトとは別系統であり、読み込んでもプロジェクトの保存先パスは設定されない（新規モデルとして開く扱い）。上書き保存するとネイティブの `.scz` として保存される。
- 書き出しは 2 つの**断面表現モード**から選べる（下記「断面表現モード」を参照）。

### 対応バージョン

- **ST-Bridge 2.0 系のみ**を受け付ける（ルート要素 `ST_BRIDGE` の `version` 属性が `2.` で始まること）。
- 1.x 系や ST-Bridge でない XML は読み込みエラーになる。

### 対応範囲（意味的往復を保証するサブセット）

読み込み・書き出しの対象は、`import → export → 再 import` でモデルが**意味的に一致する**範囲に限定している。

| 分類 | 対象内容 |
|---|---|
| 節点 | 座標、所属層 |
| 層 | 名称、標高 |
| 材料 | ヤング係数 E、ポアソン比 ν、密度、コンクリート強度 Fc、鋼材強度 Fy |
| 断面 | 面積、断面二次モーメント（Iy・Iz）、ねじり定数 J、せい・幅などの物性 |
| 部材 | 柱（鉛直材）／大梁（水平材）／間柱／ブレース（斜材・引張専用の別）、節点・断面・材料の参照、部材軸（`ref_vector`） |
| 床・壁 | スラブ（境界節点ループ＋厚さ）、壁（境界節点ループ＋厚さ＋材料） |
| 荷重 | 荷重ケース（節点荷重） |

要素ごとの詳細な変換状況（取り込み／書き出し／往復・備考）は、下記の
[ST-Bridge 要素別 変換状況一覧](#st-bridge-要素別-変換状況一覧)を参照。

### 非対応（対象外）

以下は ST-Bridge 入出力の対象に含まれない。読み込み後は既定値になる。

- 解析結果・Squid-N 独自の解析／設計属性。
- 拘束条件（支点）・質量（ST-Bridge の幾何スコープ外）。
- 部材荷重・面荷重・温度荷重・荷重組合せ（節点荷重のみ対応）。
- 基礎・杭・フーチング、開口、パラペット、通り芯（`StbAxes`）。
- 剛域・端部接合・製作情報などの詳細属性。

> これら未対応の要素のうち、**取り込み時にデータを欠落させるもの**（部材・断面・荷重・
> 通り芯）は、取りこぼしを無言で捨てず **[`ImportReport`] の警告として必ず通知**する
> （手動リストに無い ST-Bridge 2.1 の新要素・ベンダー拡張も、部材・断面・荷重の直属子で
> あれば要素名で通知する。fail-loud）。

### ST-Bridge 要素別 変換状況一覧

ST-Bridge の主要要素ごとの変換状況。凡例: **✅ 対応** ／ **⚠️ 一部・近似** ／ **❌ 非対応**。
「取り込み」は他社ファイルを読めるか、「書き出し」は Squid-N が出力するか、「往復・備考」は
`import→export→再import` での保存性と注意点を示す。断面（形鋼・RC 等）は**断面形状モード**
（`Standard`）での状況。物性モード（`Raw`）は全断面が `StbSecRaw` で完全一致往復する。

#### 節点・層・材料

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbNode`（座標・所属層） | ✅ | ✅ | 座標（小文字 `x/y/z`・大文字 `X/Y/Z` 双方可）。拘束・質量は対象外 |
| `StbStory`（名称・標高） | ✅ | ✅ | — |
| `StbStory/StbNodeIdList/StbNodeId`（階の所属節点） | ✅ | ⚠️ | 取り込みで `Node.story`・`Story.node_ids` へ反映。書き出しは `StbNode@story` 属性（方言）で表現 |
| `StbMaterial`（E・ν・密度・Fc・Fy） | ✅ | ✅ | 材料の種別・規格名は `name` のみ（型分けは非対応） |

#### 部材

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbColumn`（柱） | ✅ | ✅ | 鉛直材として往復 |
| `StbGirder` / `StbBeam`（大梁・小梁） | ✅ | ✅ | 水平材として往復 |
| `StbPost`（間柱） | ⚠️ | ⚠️ | 梁部材として取り込む（間柱の別種別が無く情報一部欠落） |
| `StbBrace`（ブレース） | ✅ | ✅ | `tension_only` 含む。取り込み時は両端ピン既定 |
| `StbSlab`（スラブ） | ✅ | ✅ | 境界節点ループ（`StbNodeIdOrder`・子要素 `StbNodeId`・CDATA 可）＋厚さ。荷重・用途・分配法は対象外 |
| `StbWall`（壁） | ✅ | ✅ | 境界節点ループ＋厚さ＋材料。開口（`StbOpen`）は対象外 |
| `StbFooting` / `StbPile` / `StbFoundationColumn` / `StbStripFooting`（基礎系） | ❌ | ❌ | 取り込み時に警告 |
| `StbParapet` / `StbOpen`（パラペット・開口） | ❌ | ❌ | 取り込み時に警告 |

#### 断面 — 鋼（形鋼ライブラリ `StbSecSteel`）

`StbSecColumn_S` / `StbSecBeam_S` / `StbSecBrace_S` が形鋼図形を参照する。

| 形鋼要素 | 取り込み | 書き出し | 内部形状・備考 |
|---|:--:|:--:|---|
| `StbSecRoll-H` / `StbSecBuild-H` | ✅ | ✅ | H 形鋼（`SteelH`） |
| `StbSecBuild-H`（上下フランジ相違） | ✅ | ✅ | 非対称組立 H（`SteelBuiltH`）。下フランジは方言属性 `B2`/`t2_lower`。第三者は上フランジの対称 H として読む |
| `StbSecRoll-BOX` / `StbSecBuild-BOX` | ✅ | ✅ | 角形鋼管（`SteelBox`） |
| `StbSecPipe` / `StbSecRoll-Pipe` / `StbSecBuild-Pipe` | ✅ | ✅ | 鋼管（`SteelPipe`）。書き出しは `StbSecPipe` |
| `StbSecRoll-L` | ✅ | ✅ | 山形鋼（`SteelAngle`） |
| `StbSecRoll-C` | ✅ | ✅ | 溝形鋼（`SteelChannel`） |
| `StbSecRoll-T` / `StbSecBuild-T` | ✅ | ✅ | T 形鋼（`SteelTee`） |
| `StbSecRoll-FlatBar` | ✅ | ✅ | 平鋼・鋼板（`SteelFlatBar`） |
| `StbSecRoll-RoundBar` | ✅ | ✅ | 中実丸鋼（`SteelRoundBar`） |
| `StbSecRoll-LipC` | ✅ | ✅ | リップ溝形鋼（`SteelLipChannel`）。幅厚比・部材ランク検定は対象外 |
| 組立断面（2L・2C・十字）・リップ Z・その他軽量形鋼 | ❌ | ❌ | 未対応。参照解決できず物性ゼロ／断面欠落として警告 |
| テーパ・非一様鋼（`_NotSame` / `_Taper` / `_Joint`） | ❌ | ❌ | 図形を復元できず断面欠落として警告 |

#### 断面 — RC・SRC・CFT

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecColumn_RC`（`_Rect` / `_Circle`） | ✅ | ✅ | RC 矩形・円形柱（`RcRect`/`RcCircle`）＋配筋 |
| `StbSecBeam_RC`（`_Straight`） | ✅ | ✅ | RC 矩形梁＋配筋。円形梁は ST-Bridge に図形が無く物性へフォールバック |
| `StbSecBarArrangement*`（配筋） | ⚠️ | ✅ | 主筋（本数・径・段数、段別本数の合算）・帯筋・かぶりを best-effort。呼び名径 `D22` 可。実スキーマ完全準拠は今後の課題 |
| `StbSecColumn_CFT`（＋充填鋼管） | ✅ | ⚠️ | CFT 角形・円形（`CftBox`/`CftPipe`）。**柱のみ**。梁に使うと物性（`StbSecRaw`）へ |
| `StbSecColumn_SRC` / `StbSecBeam_SRC` | ✅ | ✅ | SRC 矩形（`SrcRect`）＋内蔵鉄骨＋配筋＋鋼種 `strength_steel` |
| RC の T 形・L 形梁、テーパ・ハンチ | ❌ | ❌ | 図形を復元できず断面欠落として警告 |
| `StbSecFoundation_RC` / `StbSecPile_*` / `StbSecParapet_RC` / `StbSecOpen_RC` | ❌ | ❌ | 取り込み時に警告 |

#### 断面 — スラブ・壁

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecSlab_RC`（厚さ） | ✅ | ✅ | 図形要素（`StbSecSlab_RC_Straight` 等）から厚さを取得 |
| `StbSecWall_RC`（厚さ） | ✅ | ✅ | 同上 |
| `StbSecSlab_S` / `StbSecSlabDeck`（鋼・デッキ） | ❌ | ❌ | 取り込み時に警告 |

#### 荷重・その他

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbLoadCase` / `StbNodalLoad`（節点荷重） | ✅ | ✅ | 存在しない節点への荷重は破棄して警告 |
| `StbLoadMember` / 面荷重 / 温度荷重 等 | ❌ | ❌ | 未対応。取り込み時に警告（fail-loud） |
| `StbAxes`（通り芯） | ❌ | ❌ | grid/axis 概念が無く取り込み時に警告 |
| 拘束条件（支点）・質量 | ❌ | ❌ | ST-Bridge の幾何スコープ外 |

> **物性モード（`Raw`）との違い**: 上表の断面欄は**断面形状モード（`Standard`）**の状況。
> 既定の**物性モード**では全断面を `StbSecRaw`（面積・断面二次モーメント等）で書き出すため、
> 形状の種類によらず Squid-N 同士で**完全一致**往復する（形状の種類は失われ、物性のみ残る）。

### 断面表現モード（書き出し）

書き出し時、断面の表現方法を 2 つのモードから選べる。用途に応じて使い分ける。

| モード | 断面の表現 | 主な用途 | 往復（再読込） |
|---|---|---|---|
| **物性**（`Raw`, 既定） | 物性を独自要素 `StbSecRaw`（面積・断面二次モーメント等）で直接持つ | Squid-N 同士の受け渡し | **完全一致**で往復 |
| **断面形状**（`Standard`） | ST-Bridge 標準の断面要素（`StbSecColumn_S` 等）＋形鋼ライブラリ（`StbSecSteel`）で表す | BIM・他社ソフトへの受け渡し | **可能**（断面形状を復元。下記の注記あり） |

**物性モード**は「正準モデル（内部モデル）を唯一の真実とする」方針に沿い、Squid-N 同士なら物性が完全に往復する。一方、他社ソフトや BIM ツールは ST-Bridge 標準の断面要素を期待することがあるため、**断面形状モード**を用意している。

読み込み（import）は、`StbSecRaw`（物性）と ST-Bridge 標準の断面要素（`StbSecColumn_S` 等＋形鋼ライブラリ `StbSecSteel`）の**両方**を解釈する。標準要素は形鋼名から内部の断面形状を復元し、断面性能を再算定する。したがって断面形状モードで書き出したファイルや、同じ断面表現を用いる他社ソフトのファイルを Squid-N に読み戻せる。

> **断面形状モードの往復についての注記**: 断面形状・弾性断面性能（面積・断面二次モーメント等）に加え、
> **RC の配筋**（主筋の本数・径・段数、帯筋の径・ピッチ・組数・材質、かぶり）も
> `StbSecBarArrangement*` として往復する（Squid-N の断面形状モード出力どうし）。ただし次の点に注意。
> - **柱・梁で共有していた断面**は書き出し時に柱用・梁用へ分割されるため、読み戻すと 2 断面になる
>   （形状・配筋・性能は同一）。
> - **RC 円形を梁として使う断面**は ST-Bridge に円形梁の図形が無いため物性（`StbSecRaw`）へ
>   フォールバックし、形状・配筋は往復しない（物性のみ残る。円形柱は配筋も往復する）。
> - 配筋を持たない（幾何のみの）他社ファイルを読むと、その RC 断面は無筋相当になる
>   （配筋検定を行う場合は取り込み後に入力が必要）。他社ファイルの配筋は径の呼び名（`D22`）や
>   一部の標準属性名（`D_main`・`N_main_X_1st` 等）を best-effort で読むが、実 ST-Bridge の
>   配筋スキーマ（段別本数の合算・公称径の厳密対応）への完全準拠は今後の課題。
> - 形鋼ライブラリに定義の無い断面参照、および認識できない図形（テーパ・ハンチ等）の RC 断面は
>   物性ゼロ、または断面欠落（参照部材は断面なし）になる（解析前に要確認）。
>
> Squid-N 固有の解析・設計属性まで含めた完全一致での往復が必要なら**物性モード**または `.scz` を使う。

#### 断面形状モードの対応

内部モデルが持つパラメトリック断面形状（`Section.shape`）を、対応する ST-Bridge 要素へ写像する。

| 内部形状 | 書き出し先 |
|---|---|
| H形鋼・角形鋼管・鋼管・山形鋼・溝形鋼・T形鋼 | 形鋼ライブラリ `StbSecSteel`（`StbSecRoll-H`/`-BOX`/`StbSecPipe`/`-L`/`-C`/`-T`）＋ `StbSecColumn_S` / `StbSecBeam_S` |
| 平鋼・中実丸鋼・リップ溝形鋼 | 形鋼ライブラリ（`StbSecRoll-FlatBar`/`-RoundBar`/`-LipC`）＋ `StbSecColumn_S` / `StbSecBeam_S` |
| 非対称組立 H（上下フランジ相違） | `StbSecBuild-H`（標準 `A/B/t1/t2`＝上フランジ＋下フランジの方言属性 `B2`/`t2_lower`） |
| RC 矩形・円形 | `StbSecColumn_RC` / `StbSecBeam_RC`（断面の幾何 ＋ 配筋 `StbSecBarArrangement*`） |
| CFT 角形・円形 | `StbSecColumn_CFT` ＋ 充填鋼管の `StbSecSteel` 参照（柱のみ。梁に使うと `StbSecRaw`） |
| SRC 矩形 | `StbSecColumn_SRC` / `StbSecBeam_SRC`（コンクリート図形＋内蔵鉄骨 `StbSecSteel` 参照＋配筋＋鋼種 `strength_steel`） |
| 上記以外（耐震壁・形状未定義） | `StbSecRaw`（物性）へフォールバック |

補足:

- ST-Bridge では断面が柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされる。内部モデルで 1 つの断面を柱と梁の両方が共有している場合、書き出し時に柱用・梁用の 2 要素へ分割し、梁用へ新しい断面 id を割り当てる（各部材の断面参照は自動で張り替える）。読み戻すとこの分割がそのまま 2 断面になる。
- 読み込み時は、形鋼名（`StbSecRoll-H` 等）から H形鋼・角形鋼管・鋼管・山形鋼・溝形鋼・T形鋼・平鋼・中実丸鋼・リップ溝形鋼・非対称組立 H を、`StbSecColumn_RC_Rect`/`_Circle`・`StbSecBeam_RC_Straight` から RC 矩形・円形を復元し、`StbSecBarArrangement*` から配筋を復元する。node/material/story/section/element/荷重ケースの id が 1 始まりや歯抜けでも 0 始まり連番へ正規化し、全参照を張り替える。
- 材料は ST-Bridge の慣習に合わせ断面側にも付す（鋼は形鋼参照の `strength_main`＝材料名、RC/CFT/SRC は `id_material`）。読み込み時、部材が `id_material` を持たない他社ファイルでは、断面の材料を部材へ伝播する。

### ライブラリからの利用

```rust
use squid_n_io::stbridge::{
    import_stbridge, import_stbridge_with_report, export_stbridge, export_stbridge_with,
    SectionExportMode,
};

// 読み込み: ST-Bridge XML 文字列 → 内部モデル
let xml = std::fs::read_to_string("model.stb")?;
let model = import_stbridge(&xml)?;

// 読み込み（欠落の報告つき）: 未対応要素のスキップ・断面欠落・参照解決失敗などを警告として得る
let (model, report) = import_stbridge_with_report(&xml)?;
if !report.is_clean() {
    for w in &report.warnings {
        eprintln!("警告: {w}");
    }
}

// 書き出し（物性モード / 既定）: 内部モデル → ST-Bridge XML 文字列
let xml = export_stbridge(&model)?;
std::fs::write("model.stb", xml)?;

// 書き出し（断面形状モード）: BIM・他ソフト向け
let xml = export_stbridge_with(&model, SectionExportMode::Standard)?;
std::fs::write("model_std.stb", xml)?;
```

> **取り込みの欠落を確認する**: `import_stbridge_with_report` は [`ImportReport`] を返す。
> 壁・スラブ・基礎などの未対応要素、テーパ等で図形を認識できない RC/SRC 断面、
> 未解決の形鋼参照、存在しない節点を参照する部材・荷重が、警告として列挙される。
> GUI の「ST-Bridge 読込」は警告があれば「⚠️ 取り込み時の注意」として表示する。

---

## MCP サーバでのモデル入力

MCP サーバ（`squid-n-mcp`）は起動時の第 1 引数でモデルファイルを読み込む。**現状は `.scz` のみ**に対応しており、ST-Bridge ファイルの直接指定には対応していない。ST-Bridge から取り込む場合は、いったん GUI で読み込んで `.scz` として保存し、その `.scz` を MCP サーバに渡す。詳細は [MCP サーバ](./mcp_server.md)を参照。
