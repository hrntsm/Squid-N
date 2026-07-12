//! 壁関連の型（開口・壁属性・雑壁・鉄骨/BRB/PCa 属性など）。

use super::*;

/// 複数開口の取り扱い（RESP-D マニュアル計算編 02「剛性計算」）。
/// 建物全体で一律に選択する（`Model::multi_opening_mode`）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MultiOpeningMode {
    /// 等価開口とする（既定）: l0′·h0′=Σli·hi、l0′:h0′=lw:hw で1開口に置換。
    #[default]
    Equivalent,
    /// 包絡する: 全開口の包絡矩形1つに置換（位置 `offset` が必要。
    /// 位置不明の開口は包絡対象にできず個別のまま残る）。
    Envelope,
    /// 包絡開口・等価開口自動判定: 包絡可能な開口対が無くなるまで繰り返し
    /// 包絡開口を作成し、残った開口で「等価開口とする」と同様の評価を行う。
    Auto,
}

/// 壁の個別開口（RESP-D マニュアル計算編 02「剛性計算」複数開口の取り扱い）。
///
/// 寸法は壁面内で定義する: `width`=壁長さ方向の開口長さ l0 [mm]、
/// `height`=壁高さ方向の開口高さ h0 [mm]。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallOpening {
    /// 開口長さ l0 [mm]（壁長さ方向）。
    pub width: f64,
    /// 開口高さ h0 [mm]（壁高さ方向）。
    pub height: f64,
    /// 開口左下の位置 [mm]（壁面内: [壁始端からの水平距離, 壁下端からの高さ]）。
    /// 包絡開口の作成・開口の位置効果評価（将来対応）用。None は位置不定
    /// （等価開口による面積評価のみに用いられる）。
    #[serde(default)]
    pub offset: Option<[f64; 2]>,
}

impl WallOpening {
    /// 開口面積 [mm²]。
    pub fn area(&self) -> f64 {
        (self.width * self.height).max(0.0)
    }

    /// 壁面内の矩形 (x0, z0, x1, z1)。位置不明（offset=None）は None。
    fn rect(&self) -> Option<[f64; 4]> {
        let [x, z] = self.offset?;
        Some([x, z, x + self.width.max(0.0), z + self.height.max(0.0)])
    }

    /// 2開口の包絡開口（外接矩形）。どちらかの位置が不明なら None。
    pub fn envelope(&self, other: &WallOpening) -> Option<WallOpening> {
        let a = self.rect()?;
        let b = other.rect()?;
        let x0 = a[0].min(b[0]);
        let z0 = a[1].min(b[1]);
        let x1 = a[2].max(b[2]);
        let z1 = a[3].max(b[3]);
        Some(WallOpening {
            width: x1 - x0,
            height: z1 - z0,
            offset: Some([x0, z0]),
        })
    }

    /// 自動判定モードで 2 開口を包絡してよいかの判定
    /// （RESP-D マニュアル計算編 02「複数開口の取り扱い」の判定図）。
    ///
    /// **l < 1.5·h または l < 1m（1000mm）のとき包絡開口とみなす。**
    /// - l: 開口間距離（矩形間の純距離。重なっていれば 0）
    /// - h: 包絡開口とした場合の高さ
    ///
    /// 位置（offset）不明の開口は距離を定義できないため包絡不可。
    pub fn can_envelope(&self, other: &WallOpening) -> bool {
        let (Some(a), Some(b)) = (self.rect(), other.rect()) else {
            return false;
        };
        // 開口間距離 l: 各方向の純間隔（重なっていれば 0）の合成
        let gap_x = (a[0].max(b[0]) - a[2].min(b[2])).max(0.0);
        let gap_z = (a[1].max(b[1]) - a[3].min(b[3])).max(0.0);
        let l = (gap_x * gap_x + gap_z * gap_z).sqrt();
        // 包絡開口とした場合の高さ h
        let h = a[3].max(b[3]) - a[1].min(b[1]);
        l < 1.5 * h || l < 1000.0
    }
}

/// 壁要素（`ElementKind::Wall`/`Shell`）の壁属性
/// （RESP-D マニュアル「壁自重」の開口・三方スリット、および
/// 計算編 02「剛性計算」の開口低減・耐震壁判定に用いる個別開口寸法）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallAttr {
    pub elem: ElemId,
    /// 開口面積の合計 [mm²]。壁自重から ρ·t·開口面積·g を控除する。
    /// `openings`（個別開口）が非空の場合はそちらの面積和を優先し、
    /// 本フィールドは無視される（`total_opening_area` 参照）。
    #[serde(default)]
    pub opening_area: f64,
    /// 開口部（サッシ等）の重量 [N]。控除後に加算する。
    #[serde(default)]
    pub opening_weight: f64,
    /// 三方スリット。true の場合、壁自重は上下分配せず全て上部の節点
    /// （壁頂部の節点）へ伝達する（マニュアル「壁に三方スリットが指定されて
    /// いる場合、壁荷重は全て上部の大梁に伝達され」の節点重量版）。
    #[serde(default)]
    pub three_side_slit: bool,
    /// 個別開口の寸法リスト。非空の場合、開口の面積評価（自重控除・
    /// 開口周比 r0・開口低減率 r）と耐震壁検定の開口供給はこのリストを
    /// 優先する。空の場合は従来どおり `opening_area`（合計面積のみ）で評価する。
    #[serde(default)]
    pub openings: Vec<WallOpening>,
}

impl WallAttr {
    /// 開口の合計面積 [mm²]。個別開口 `openings` が非空ならその面積和、
    /// 空なら `opening_area` を返す（全消費側はこのメソッドを経由すること）。
    pub fn total_opening_area(&self) -> f64 {
        if self.openings.is_empty() {
            self.opening_area.max(0.0)
        } else {
            self.openings.iter().map(WallOpening::area).sum()
        }
    }

    /// 個別開口の (l0, h0) ペア列。個別開口が未入力（面積のみ）なら None。
    /// 面積ゼロの開口は除外する。
    pub fn opening_dims(&self) -> Option<Vec<(f64, f64)>> {
        Self::dims_of(&self.openings)
    }

    /// 複数開口の取り扱い（`mode`）適用後の (l0, h0) ペア列。
    /// 個別開口が未入力（面積のみ）なら None（消費側は `opening_area` で評価）。
    pub fn opening_dims_for(&self, mode: MultiOpeningMode) -> Option<Vec<(f64, f64)>> {
        Self::dims_of(&self.openings_for_mode(mode))
    }

    /// 複数開口の取り扱い（`mode`）適用後の開口合計面積 [mm²]。
    /// 包絡モードでは包絡矩形の面積となるため、生の面積和
    /// （`total_opening_area`、自重控除用）とは異なり得る。
    pub fn total_opening_area_for(&self, mode: MultiOpeningMode) -> f64 {
        if self.openings.is_empty() {
            self.opening_area.max(0.0)
        } else {
            self.openings_for_mode(mode)
                .iter()
                .map(WallOpening::area)
                .sum()
        }
    }

    /// 複数開口の取り扱い（RESP-D 計算編 02）を適用した開口リスト。
    /// - `Equivalent`: 個別開口をそのまま返す（等価開口への統合は消費側の式）。
    /// - `Envelope`: 位置（offset）を持つ開口全体の包絡矩形 1 つに置換。
    ///   位置不明の開口は包絡できないため個別のまま残る。
    /// - `Auto`: 包絡可能（`WallOpening::can_envelope`、l<1.5h または l<1m）な開口対が
    ///   無くなるまで繰り返し包絡開口を作成し、残った開口を返す
    ///   （マニュアル「包絡できなくなった時点の開口状況で『等価開口とする』と
    ///   同様の判定を行います」に対応。等価開口への統合は消費側）。
    pub fn openings_for_mode(&self, mode: MultiOpeningMode) -> Vec<WallOpening> {
        match mode {
            MultiOpeningMode::Equivalent => self.openings.clone(),
            MultiOpeningMode::Envelope => {
                let mut out: Vec<WallOpening> = Vec::new();
                let mut merged: Option<WallOpening> = None;
                for o in &self.openings {
                    if o.rect().is_some() {
                        merged = Some(match merged {
                            Some(m) => m.envelope(o).expect("両者とも位置あり"),
                            None => o.clone(),
                        });
                    } else {
                        out.push(o.clone());
                    }
                }
                if let Some(m) = merged {
                    out.insert(0, m);
                }
                out
            }
            MultiOpeningMode::Auto => {
                let mut list: Vec<WallOpening> = self.openings.clone();
                loop {
                    let mut merged_pair: Option<(usize, usize)> = None;
                    'outer: for i in 0..list.len() {
                        for j in (i + 1)..list.len() {
                            if list[i].can_envelope(&list[j]) {
                                merged_pair = Some((i, j));
                                break 'outer;
                            }
                        }
                    }
                    let Some((i, j)) = merged_pair else {
                        break;
                    };
                    let env = list[i].envelope(&list[j]).expect("can_envelope=位置あり");
                    list.remove(j);
                    list[i] = env;
                }
                list
            }
        }
    }

    fn dims_of(openings: &[WallOpening]) -> Option<Vec<(f64, f64)>> {
        if openings.is_empty() {
            return None;
        }
        let dims: Vec<(f64, f64)> = openings
            .iter()
            .filter(|o| o.area() > 0.0)
            .map(|o| (o.width, o.height))
            .collect();
        if dims.is_empty() {
            None
        } else {
            Some(dims)
        }
    }
}

/// S 造部材の断面検定用属性（RESP-D マニュアル 04 断面検定「鉄骨の断面検定に
/// おける断面性能」）。継手部・スカラップによる断面欠損と横座屈長さの指定に用いる。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SteelDesignAttr {
    pub elem: ElemId,
    /// 継手部のフランジ欠損率 βf [%]（0=欠損なし）
    #[serde(default)]
    pub joint_flange_loss: f64,
    /// 継手部のウェブ欠損率 βw [%]
    #[serde(default)]
    pub joint_web_loss: f64,
    /// スカラップによるウェブ欠損率 αw [%]（端部断面に適用）
    #[serde(default)]
    pub scallop_web_loss: f64,
    /// 横座屈長さの直接入力 (始端, 中央, 終端) [mm]（None=自動）
    #[serde(default)]
    pub lb_direct: Option<(f64, f64, f64)>,
    /// 等間隔横補剛の本数（lb 自動計算: lb = L/(n+1)）
    #[serde(default)]
    pub lateral_brace_count: Option<u32>,
}

/// 座屈補剛ブレース（BRB）の断面検定用属性。許容値はメーカー資料による入力値
/// （RESP-D マニュアル「JFEシビル二重鋼管座屈補剛ブレース／日鉄アンボンド
/// ブレースの断面検定」）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BrbAttr {
    pub elem: ElemId,
    /// 短期許容軸力 [N]（メーカー値）
    pub allowable_axial_short: f64,
    /// 限界座屈長さ [mm]（メーカー値）
    pub critical_length: f64,
    /// 座屈長さ低減距離 L1 [mm]（= (L1上+L1下)/2）
    #[serde(default)]
    pub length_reduction: f64,
}

/// 免震支承材の種別（RESP-D マニュアル「05 非線形モデル」免震支承材）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum IsolatorKind {
    /// 積層ゴム系（マルチシアスプリング、水平バイリニア。鉛・高減衰含む）。
    LaminatedRubber,
    /// 弾性すべり支承（摩擦ばね、Qmax=μ·N）。
    ElasticSliding,
}

/// 免震支承材の特性（`ElementKind::Isolator` 要素の非線形特性、
/// RESP-D「05 非線形モデル」）。`Model::isolator_attrs` に要素 ID と対で保持する。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IsolatorProps {
    /// 支承種別。
    pub kind: IsolatorKind,
    /// 水平初期（弾性）剛性 K1 [N/mm]。
    pub k1: f64,
    /// 水平二次剛性 K2 [N/mm]（積層ゴム系バイリニア。すべり支承は 0）。
    pub k2: f64,
    /// 降伏（特性）耐力 Qd [N]（積層ゴム系バイリニア）。すべり支承は 0。
    pub qd: f64,
    /// 鉛直剛性 Kv [N/mm]。
    pub kv: f64,
    /// 摩擦係数 μ（すべり支承。Qmax=μ·N）。
    pub mu: f64,
    /// 長期軸力 N [N]（摩擦力算定用、圧縮正）。
    pub n_long: f64,
    /// マルチシアスプリング本数 n（既定 8、天然ゴム系 2。表示・低減率照合用）。
    pub n_springs: u32,
}

impl Default for IsolatorProps {
    fn default() -> Self {
        IsolatorProps {
            kind: IsolatorKind::LaminatedRubber,
            k1: 1000.0,
            k2: 100.0,
            qd: 50_000.0,
            kv: 2_000_000.0,
            mu: 0.1,
            n_long: 0.0,
            n_springs: 8,
        }
    }
}

/// 免震支承材の属性（要素 ID と特性の対）。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IsolatorAttr {
    pub elem: ElemId,
    pub props: IsolatorProps,
}

/// PCa（プレキャスト）梁の水平接合面検定用属性（RESP-D マニュアル 04）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PcaBeamAttr {
    pub elem: ElemId,
    /// 水平接合面の摩擦係数 μ
    pub mu: f64,
    /// 接合面を横切る補強筋の体積比合計 p′w（あばら筋+接合面補強筋）
    pub pw_joint: f64,
    /// 補強筋の降伏強度 σy [N/mm²]
    pub sigma_y_joint: f64,
    /// 接合面の位置: 断面上端からの距離 [mm]（例: 後打ちスラブ厚）
    pub joint_depth_from_top: f64,
}

/// フレーム外雑壁の荷重伝達タイプ（RESP-D マニュアル「フレーム外雑壁」）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MiscWallTransfer {
    /// 0.5m 分割した各領域の中心から最も近い柱の上下節点へ 1/2 ずつ伝達。
    #[default]
    Column,
    /// 0.5m 分割した各領域の中心から最も近い大梁・小梁側の節点へ集中伝達。
    Beam,
    /// 自立。配置階の剛床（最も近い節点）へ伝達する簡易扱い。
    SelfStanding,
}

/// フレーム外雑壁（部材としてモデル化しない壁）。始点→終点の直線区間に
/// 高さ・面重量を持ち、0.5m 分割規則で近傍の節点へ重量を集計する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MiscWall {
    /// 壁下端の始点座標 [mm]。
    pub start: [f64; 3],
    /// 壁下端の終点座標 [mm]。
    pub end: [f64; 3],
    /// 壁高さ [mm]。
    pub height: f64,
    /// 面重量 [N/mm²]（仕上げ込み）。
    pub weight_per_area: f64,
    /// 荷重伝達タイプ。
    #[serde(default)]
    pub transfer: MiscWallTransfer,
    /// 壁厚 [mm]。雑壁剛性の n 倍法（`StressAnalysisCfg::misc_wall_n`）で
    /// 断面積 Aw' = 壁長 × 壁厚 の算定に用いる。`None` は剛性評価の対象外
    /// （重量のみ考慮）。
    #[serde(default)]
    pub thickness: Option<f64>,
}
