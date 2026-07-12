use crate::dof::Dof6Mask;
use crate::ids::*;
use smallvec::SmallVec;

mod load;
mod wall;
pub use load::*;
pub use wall::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub coord: [f64; 3],
    pub restraint: Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<StoryId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ElementKind {
    Beam,
    Shell,
    Fiber,
    Ms,
    Wall,
    PanelZone,
    /// 一般ブレース（軸材。RESP-D マニュアル計算編02「剛性計算」§一般ブレースの剛性）。
    /// 剛性は軸剛性のみのトラス要素（KB=E·A/L）で評価する。
    /// K 型ブレースの重量配分規則（`LoadCfg::k_brace_rule`）の適用対象。
    /// `tension_only`: 引張専用ブレースか（true の場合、弾性解析では剛性を1/2に
    /// モデル化する。弾塑性解析では初期剛性は1倍。マニュアル既定の「引張と圧縮が
    /// 対で存在するとみなす」モデル化）。
    Brace {
        tension_only: bool,
    },
    /// 節点バネ要素（RESP-D マニュアル計算編03「応力解析」§部材の変形と自由度）。
    ///
    /// マニュアルの「部材の変形と自由度」表で、節点バネは θX=―（非考慮）、
    /// θY=○, θZ=○, γY=○, γZ=○, δX=○。すなわちねじり以外の曲げ・せん断・
    /// 軸方向の変形成分を独立なバネ剛性として持ちうる 2 節点要素。
    /// 各自由度のバネ定数は `ElementData::spring` に保持する（局所軸 6 成分）。
    NodalSpring,
    /// 免震支承材（RESP-D マニュアル「05 非線形モデル」免震支承材）。
    /// 2 節点要素で、水平は非線形せん断ばね（マルチシアスプリング＝積層ゴム系
    /// バイリニア、または摩擦ばね＝弾性すべり支承 Qmax=μN）、鉛直は弾性軸ばね。
    /// 特性は `Model::isolator_attrs` に要素 ID と対で保持する。
    Isolator,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ForceRegime {
    UniaxialBendingShear,
    AxialBendingInteract,
    Auto,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LocalAxis {
    pub ref_vector: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum EndCondition {
    Fixed,
    Pinned,
    SemiRigid { k_theta: f64 },
}

/// 剛域長の出所。Auto は再算定で上書きされる、Manual は保護される（設計書 §6.2.1）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ZoneSource {
    Auto,
    Manual,
}

/// 部材端の剛域（接合部の有限寸法）。可とう長 L' = L − length_i − length_j。
/// 力学計算は sc-element 側。ここではモデルに保持・永続化するデータ。
///
/// **剛域長（length_i/j）とフェイス距離（face_i/j）は別概念**（設計書 §6.2.1）。
/// - `length_i/j`: 剛性計算に使う剛域長 `λ = D_orth/2 − D_self/4`（低減率 `reduction` を含む）。
/// - `face_i/j`: 断面算定・危険断面位置（§6.2.3）に使う柱フェース距離 `D_orth/2`。
///   剛域長のような低減率調整は行わない幾何量であり、節点から接合する直交部材せいの
///   半分までの距離をそのまま保持する。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RigidZone {
    pub length_i: f64,
    pub length_j: f64,
    pub source_i: ZoneSource,
    pub source_j: ZoneSource,
    pub reduction: f64,
    /// 柱フェース距離 [mm]（節点→フェース、= 接合する直交部材せい/2）。
    /// 直交材が無い端は 0。断面算定の既定危険断面位置に用いる（§6.2.3）。
    #[serde(default)]
    pub face_i: f64,
    /// 柱フェース距離 [mm]（j端）。意味は `face_i` と同様。
    #[serde(default)]
    pub face_j: f64,
}

impl Default for RigidZone {
    fn default() -> Self {
        Self {
            length_i: 0.0,
            length_j: 0.0,
            source_i: ZoneSource::Auto,
            source_j: ZoneSource::Auto,
            reduction: 1.0,
            face_i: 0.0,
            face_j: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ElementData {
    pub id: ElemId,
    pub kind: ElementKind,
    pub nodes: SmallVec<[NodeId; 8]>,
    pub section: Option<SectionId>,
    pub material: Option<MaterialId>,
    pub local_axis: LocalAxis,
    pub end_cond: [EndCondition; 2],
    pub force_regime: ForceRegime,
    /// 部材端の剛域。旧スキーマ（無し）は既定値（剛域長 0）で補完される。
    #[serde(default)]
    pub rigid_zone: RigidZone,
    /// 塑性化領域長さ Lp [mm]（None = 塑性化域を考慮しない従来モデル）。
    /// ファイバー要素では端部 Lp 区間に非線形断面を配置し中央を弾性とする
    /// モデル化（材端剛塑性ばねと適合するファイバーモデル化）に用いる。
    #[serde(default)]
    pub plastic_zone: Option<f64>,
    /// 節点バネ要素（`ElementKind::NodalSpring`）の局所軸バネ定数
    /// `[kx, ky, kz, krx, kry, krz]`（軸[N/mm]・せん断[N/mm]・回転[N·mm/rad]）。
    /// RESP-D マニュアル計算編03「応力解析」§部材の変形と自由度により、節点バネは
    /// ねじり（θX）を非考慮とするのが既定だが、本実装では全 6 成分を入力可能とし、
    /// `krx` を明示的に 0 とすることで既定挙動に合わせる（入力で 0 以外も指定できる）。
    /// `None` は他要素種別、またはバネ定数未指定（剛性ゼロ扱い）。
    #[serde(default)]
    pub spring: Option<[f64; 6]>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DiaphragmDef {
    pub master: NodeId,
    pub slaves: Vec<NodeId>,
    pub rigid: bool,
    /// この剛床が負担する地震用重量 [N]。多剛床の階では層の水平力 Pi を
    /// 剛床ごとの重量比で分配するために用いる（RESP-D マニュアル
    /// 「多剛床の設計用せん断力」）。None は未算定（階に単一剛床なら層重量全量）。
    #[serde(default)]
    pub weight: Option<f64>,
    /// 副剛床の層せん断力係数 Ci の直接入力（RESP-D マニュアル
    /// 「副剛床の Ci を直接入力した場合」）。Some の剛床は主系統の Ai 分布から
    /// 除外され、水平力 = ci_override × 剛床重量（等価震度扱い。上階に同一系統の
    /// 剛床が積み上がらない副剛床を想定）として作用する。None は主系統（Ai 分布）。
    #[serde(default)]
    pub ci_override: Option<f64>,
}

/// 階の主要構造種別。設計用一次固有周期の略算式 T=h(0.02+0.01α) の
/// α（柱梁の大部分が鉄骨造である階の高さ比）の算定に用いる（令88条・告示1793号）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StoryStructure {
    #[default]
    Rc,
    S,
    Src,
}

/// 階の種別。地震層せん断力の算定方法を切り替える
/// （一般階=Ai分布、PH階=震度 k、地下階=水平震度 K）。
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StoryLevelKind {
    #[default]
    Normal,
    /// 塔屋（PH）階。層せん断力 Qi = k·ΣWj（k は 0.5〜1.0 の指定震度）。
    Penthouse { k: f64 },
    /// 地下階。Qi = Q(i+1) + K·Wi、K = 0.1·(1 − H/40)·Z（H は地盤面からの深さ[m]、20m 超は 20m）。
    Basement { depth_m: f64 },
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Story {
    pub id: StoryId,
    pub name: String,
    pub elevation: f64,
    pub node_ids: Vec<NodeId>,
    pub diaphragms: Vec<DiaphragmDef>,
    pub seismic_weight: Option<f64>,
    /// 主要構造種別（略算周期の鉄骨造比 α 算定用）。旧スキーマは RC 扱い。
    #[serde(default)]
    pub structure: StoryStructure,
    /// 階の種別（一般/PH/地下）。旧スキーマは一般階扱い。
    #[serde(default)]
    pub level_kind: StoryLevelKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DistributionMethod {
    TriTrapezoid,
    OneWay,
    TributaryArea,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct JoistLine {
    pub dir: [f64; 2],
    pub spacing: f64,
    pub support: [NodeId; 2],
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AreaLoad {
    pub kind: String,
    pub value: f64,
}

/// スラブの種別。片持ちスラブは境界の辺 0（`boundary[0]`→`boundary[1]`）を
/// 取付き辺（大梁側）とし、荷重は取付き辺へ伝達する（RESP-D マニュアル「片持ちスラブ」）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SlabKind {
    #[default]
    Interior,
    Cantilever,
    /// 出隅の片持ちスラブ。荷重は伝達方向・片持ち梁の有無に関わらず
    /// 全て節点荷重として柱（`boundary[0]` の節点）へ伝達する
    /// （RESP-D マニュアル「出隅の片持ちスラブ」）。
    Corner,
}

/// 一方向スラブの荷重伝達方向（床ごとに指定。RESP-D マニュアル「スラブ荷重」の〔X〕〔Y〕）。
/// `X` は全体座標 X 方向へ伝達（＝X 方向両側の辺が負担）、`Y` は Y 方向へ伝達。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OneWayDir {
    X,
    Y,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Slab {
    pub id: SlabId,
    pub boundary: Vec<NodeId>,
    pub joists: Vec<JoistLine>,
    pub loads: Vec<AreaLoad>,
    pub method: DistributionMethod,
    /// スラブ種別（一般/片持ち）。旧スキーマは一般スラブ扱い。
    #[serde(default)]
    pub kind: SlabKind,
    /// 一方向スラブの伝達方向。`None` は従来互換
    /// （境界辺 0・2 が負担＝辺 1 方向スパン）の暗黙規則。
    #[serde(default)]
    pub one_way: Option<OneWayDir>,
    /// 境界辺ごとの支持有無（`boundary` の辺数と同長）。`None` は既定
    /// （Interior は全辺支持、Cantilever は辺 0 のみ支持）。片持ちスラブに
    /// 片持ち梁・先端リブ小梁が取り付く場合、支持辺を追加指定すると
    /// スラブと同様のルール（最近接支持辺の負担面積）で分割伝達される
    /// （RESP-D マニュアル「片持ちスラブ」の片持ち梁あり/先端リブ小梁ありの場合）。
    #[serde(default)]
    pub edge_supported: Option<Vec<bool>>,
}

use crate::dof::Dof;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    RigidDiaphragm {
        story: StoryId,
        master: NodeId,
        slaves: Vec<NodeId>,
    },
    Mpc {
        master: NodeId,
        terms: Vec<(NodeId, Dof, f64)>,
    },
    RigidLink {
        master: NodeId,
        slaves: Vec<NodeId>,
        dofs: Dof6Mask,
    },
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub id: MaterialId,
    pub name: String,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    #[serde(default)]
    pub shear: Option<f64>,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    /// 鋼材では `None`。RC 設計（令91条）の許容圧縮・せん断に用いる。
    #[serde(default)]
    pub fc: Option<f64>,
    /// 降伏応力 fy [N/mm²]。鋼材の弾塑性挙動（ファイバ材料・端ばねスケルトン）に用いる。
    /// `None` の場合、ファイバ材料は弾性（降伏しない）として扱う（P5 非線形）。
    #[serde(default)]
    pub fy: Option<f64>,
    /// コンクリートの種類（普通/軽量1種/軽量2種）。RESP-D マニュアル「柱梁自重」の
    /// 単位体積重量表・「04 断面検定」の許容応力度低減（軽量コンクリートは
    /// 普通コンクリートの 0.9 倍）に用いる。鋼材では意味を持たない（既定 Normal）。
    /// 旧スキーマ（フィールド無し）は Normal 扱い。
    #[serde(default)]
    pub concrete_class: crate::units::ConcreteClass,
}

impl Material {
    pub fn shear_modulus(&self) -> f64 {
        self.shear
            .unwrap_or_else(|| self.young / (2.0 * (1.0 + self.poisson)))
    }
}

pub fn rect_shear_area(area: f64) -> f64 {
    area * 5.0 / 6.0
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Section {
    pub id: SectionId,
    pub name: String,
    pub area: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    #[serde(default)]
    pub depth: f64,
    #[serde(default)]
    pub width: f64,
    #[serde(default)]
    pub as_y: f64,
    #[serde(default)]
    pub as_z: f64,
    #[serde(default)]
    pub panel_thickness: Option<f64>,
    #[serde(default)]
    pub thickness: Option<f64>,
    /// パラメトリック形状定義（UI設計 §4.2: Section は SectionShape の派生）。
    /// 形状から生成されなかった断面（カタログ数値直入力・ST-Bridge 読込等）は None。
    #[serde(default)]
    pub shape: Option<crate::section_shape::SectionShape>,
}

/// 長期応力解析の計算条件（RESP-D マニュアル計算編03「応力解析」）。
///
/// マニュアル原文:「長期応力解析においては、計算条件の指定により以下の部材について
/// 長期軸力を負担させないことも可能です。― ブレース ― 柱、制振間柱」。
///
/// 制振間柱（damper-equipped mullion column）は本リポジトリに要素種別が未実装のため、
/// 対象外（既知の制約）。ブレースと柱（鉛直部材）のみ対応する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StressAnalysisCfg {
    /// 長期応力解析でブレース（`ElementKind::Brace`）に軸力を負担させない。
    pub no_long_axial_brace: bool,
    /// 長期応力解析で柱（鉛直な `ElementKind::Beam`）に軸力を負担させない。
    pub no_long_axial_column: bool,
    /// 剛性率・偏心率算定時の雑壁剛性の n 倍法係数（RESP-D「(7) 雑壁の剛性評価」
    /// `Kw' = n·Aw'·ΣKc/ΣAc` の n。入力値）。`None` は雑壁剛性を考慮しない。
    #[serde(default)]
    pub misc_wall_n: Option<f64>,
    /// 層間変形角の制限値の分母（令82条の2）。原則 200（1/200）。帳壁・仕上げ等に
    /// 著しい損傷の恐れがない場合は 120（1/120）へ緩和できる。
    #[serde(default = "default_drift_limit_denom")]
    pub drift_limit_denom: f64,
}

fn default_drift_limit_denom() -> f64 {
    200.0
}

impl Default for StressAnalysisCfg {
    fn default() -> Self {
        StressAnalysisCfg {
            no_long_axial_brace: false,
            no_long_axial_column: false,
            misc_wall_n: None,
            drift_limit_denom: default_drift_limit_denom(),
        }
    }
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Model {
    pub nodes: Vec<Node>,
    pub elements: Vec<ElementData>,
    pub sections: Vec<Section>,
    pub materials: Vec<Material>,
    pub stories: Vec<Story>,
    pub slabs: Vec<Slab>,
    pub constraints: Vec<Constraint>,
    pub load_cases: Vec<LoadCase>,
    pub combinations: Vec<LoadCombination>,
    /// 階の自動生成が作る剛床代表節点（慣性力重心に置く仮想節点）の ID。
    /// 構造節点と区別するために保持し、再生成時に再利用する。
    #[serde(default)]
    pub generated_masters: Vec<NodeId>,
    /// 剛性計算用の床スラブ厚 [mm]（建物全体で一律。RESP-D 計算編 02「剛性計算」
    /// 注1 の設定に対応）。0 以下でスラブ協力幅による梁剛性増大を無効化（既定）。
    #[serde(default)]
    pub slab_thickness: f64,
    /// 自重算定の付加設定（鉄骨重量割増率・部材付加線重量）。`None` は既定値。
    #[serde(default)]
    pub load_cfg: Option<LoadCfg>,
    /// 壁要素の自重算定属性（開口・三方スリット）。
    #[serde(default)]
    pub wall_attrs: Vec<WallAttr>,
    /// 複数開口の取り扱い（建物一律。RESP-D 計算編 02「剛性計算」）。
    /// 剛性の開口低減・耐震壁判定・検定への開口供給に適用する
    /// （自重控除は常に生の開口面積和）。既定は「等価開口とする」。
    #[serde(default)]
    pub multi_opening_mode: MultiOpeningMode,
    /// フレーム外雑壁。
    #[serde(default)]
    pub misc_walls: Vec<MiscWall>,
    /// 応力解析の計算条件（RESP-D 計算編03「応力解析」。長期軸力を負担させない部材の指定）。
    #[serde(default)]
    pub stress_cfg: StressAnalysisCfg,
    /// S 造部材の断面検定用属性（継手部・スカラップ欠損、横座屈長さ指定。
    /// RESP-D マニュアル 04 断面検定）。
    #[serde(default)]
    pub steel_design_attrs: Vec<SteelDesignAttr>,
    /// 座屈補剛ブレース（BRB）の断面検定用属性（メーカー許容値。
    /// RESP-D マニュアル 04 断面検定）。
    #[serde(default)]
    pub brb_attrs: Vec<BrbAttr>,
    /// PCa（プレキャスト）梁の水平接合面検定用属性（RESP-D マニュアル 04 断面検定）。
    #[serde(default)]
    pub pca_attrs: Vec<PcaBeamAttr>,
    /// 免震支承材の非線形特性（`ElementKind::Isolator` 要素、RESP-D 05 非線形モデル）。
    #[serde(default)]
    pub isolator_attrs: Vec<IsolatorAttr>,
    /// 一本部材の指定（RESP-D マニュアル 04 断面検定「採用応力 ■一本部材指定時の
    /// 採用応力」）。各エントリは**軸方向に連続する梁要素の ID を並び順**で持ち、
    /// 断面検定の採用応力（端部・中央モーメント、部材長、内法長、せん断スパン比
    /// 代表値）をグループ 1 本の部材として評価する。要素の解析（剛性・内力）は
    /// 分割部材のまま行い、検定の文脈だけを合成する。
    #[serde(default)]
    pub beam_groups: Vec<Vec<ElemId>>,
    #[serde(skip)]
    pub dof_map: crate::dof::DofMap,
}

impl Model {
    pub fn validate(&self) -> Result<(), crate::error::CoreError> {
        use crate::error::CoreError;

        for (i, node) in self.nodes.iter().enumerate() {
            if node.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "nodes[{}] has NodeId({})",
                    i, node.id.0
                )));
            }
        }

        let mut seen_nodes = std::collections::HashSet::new();
        for node in &self.nodes {
            if !seen_nodes.insert(node.id) {
                return Err(CoreError::DuplicateId(format!("NodeId({})", node.id.0)));
            }
        }

        for (i, elem) in self.elements.iter().enumerate() {
            if elem.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "elements[{}] has ElemId({})",
                    i, elem.id.0
                )));
            }
        }

        let mut seen_elems = std::collections::HashSet::new();
        for elem in &self.elements {
            if !seen_elems.insert(elem.id) {
                return Err(CoreError::DuplicateId(format!("ElemId({})", elem.id.0)));
            }
            for &nid in &elem.nodes {
                if nid.index() >= self.nodes.len() || self.nodes[nid.index()].id != nid {
                    return Err(CoreError::DanglingRef(format!(
                        "Elem {} -> Node {}",
                        elem.id.0, nid.0
                    )));
                }
            }
            if let Some(sid) = elem.section {
                if sid.index() >= self.sections.len() || self.sections[sid.index()].id != sid {
                    return Err(CoreError::DanglingRef(format!(
                        "Elem {} -> Section {}",
                        elem.id.0, sid.0
                    )));
                }
            }
            if let Some(mid) = elem.material {
                if mid.index() >= self.materials.len() || self.materials[mid.index()].id != mid {
                    return Err(CoreError::DanglingRef(format!(
                        "Elem {} -> Material {}",
                        elem.id.0, mid.0
                    )));
                }
            }
        }

        for (i, story) in self.stories.iter().enumerate() {
            if story.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "stories[{}] has StoryId({})",
                    i, story.id.0
                )));
            }
        }

        let mut seen_stories = std::collections::HashSet::new();
        for story in &self.stories {
            if !seen_stories.insert(story.id) {
                return Err(CoreError::DuplicateId(format!("StoryId({})", story.id.0)));
            }
        }

        for (i, slab) in self.slabs.iter().enumerate() {
            if slab.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "slabs[{}] has SlabId({})",
                    i, slab.id.0
                )));
            }
        }

        let mut seen_slabs = std::collections::HashSet::new();
        for slab in &self.slabs {
            if !seen_slabs.insert(slab.id) {
                return Err(CoreError::DuplicateId(format!("SlabId({})", slab.id.0)));
            }
        }

        for (i, sec) in self.sections.iter().enumerate() {
            if sec.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "sections[{}] has SectionId({})",
                    i, sec.id.0
                )));
            }
        }

        let mut seen_sections = std::collections::HashSet::new();
        for sec in &self.sections {
            if !seen_sections.insert(sec.id) {
                return Err(CoreError::DuplicateId(format!("SectionId({})", sec.id.0)));
            }
        }

        for (i, mat) in self.materials.iter().enumerate() {
            if mat.id.index() != i {
                return Err(CoreError::IndexMismatch(format!(
                    "materials[{}] has MaterialId({})",
                    i, mat.id.0
                )));
            }
        }

        let mut seen_materials = std::collections::HashSet::new();
        for mat in &self.materials {
            if !seen_materials.insert(mat.id) {
                return Err(CoreError::DuplicateId(format!("MaterialId({})", mat.id.0)));
            }
        }

        Ok(())
    }

    /// 指定した節点が部材・節点荷重・階・床・拘束のいずれかから参照されているかを判定する。
    /// 参照中の節点を削除すると参照が壊れる（ダングリング）ため、削除前にこれで確認する。
    pub fn node_in_use(&self, id: NodeId) -> bool {
        self.elements.iter().any(|e| e.nodes.contains(&id))
            || self
                .load_cases
                .iter()
                .any(|lc| lc.nodal.iter().any(|nl| nl.node == id))
            || self.stories.iter().any(|s| {
                s.node_ids.contains(&id)
                    || s.diaphragms
                        .iter()
                        .any(|d| d.master == id || d.slaves.contains(&id))
            })
            || self.slabs.iter().any(|sl| {
                sl.boundary.contains(&id) || sl.joists.iter().any(|j| j.support.contains(&id))
            })
            || self.constraints.iter().any(|c| match c {
                Constraint::RigidDiaphragm { master, slaves, .. } => {
                    *master == id || slaves.contains(&id)
                }
                Constraint::Mpc { master, terms } => {
                    *master == id || terms.iter().any(|(n, _, _)| *n == id)
                }
                Constraint::RigidLink { master, slaves, .. } => {
                    *master == id || slaves.contains(&id)
                }
            })
    }

    pub fn eq_ignoring_dofmap(&self, other: &Self) -> bool {
        self.nodes == other.nodes
            && self.elements == other.elements
            && self.sections == other.sections
            && self.materials == other.materials
            && self.stories == other.stories
            && self.slabs == other.slabs
            && self.constraints == other.constraints
            && self.load_cases == other.load_cases
            && self.combinations == other.combinations
            && self.generated_masters == other.generated_masters
            && self.load_cfg == other.load_cfg
            && self.wall_attrs == other.wall_attrs
            && self.misc_walls == other.misc_walls
            && self.stress_cfg == other.stress_cfg
            && self.steel_design_attrs == other.steel_design_attrs
            && self.brb_attrs == other.brb_attrs
            && self.pca_attrs == other.pca_attrs
            && self.beam_groups == other.beam_groups
            && self.isolator_attrs == other.isolator_attrs
    }
}

#[cfg(test)]
mod tests;
