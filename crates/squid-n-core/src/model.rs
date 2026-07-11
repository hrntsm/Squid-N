use crate::dof::Dof6Mask;
use crate::ids::*;
use smallvec::SmallVec;

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

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NodalLoad {
    pub node: NodeId,
    pub values: [f64; 6],
}

/// 部材（梁）荷重の種別。位置・強度はすべて部材ローカル x 軸（i→j）に沿った
/// 距離 [mm] と強度で与える。作用方向は `MemberLoad::dir`（全体座標）で指定する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum MemberLoadKind {
    /// 中間集中荷重: i 端から距離 `a` [mm] の位置に大きさ `p` [N]。
    Point { a: f64, p: f64 },
    /// 区間分布荷重: [`a`, `b`] 区間に強度 `w1`→`w2` [N/mm] の線形分布。
    /// 等分布は `w1 == w2`、全長は `a = 0, b = L`、三角形は端の強度を 0 にする。
    Distributed { a: f64, b: f64, w1: f64, w2: f64 },
}

/// 部材に作用する荷重。`dir` は全体座標系での作用方向（内部で正規化）。
/// 既定の重力方向は `[0.0, 0.0, -1.0]`。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MemberLoad {
    pub elem: ElemId,
    pub dir: [f64; 3],
    pub kind: MemberLoadKind,
}

/// 荷重ケースの種別。地震用重量の集計（固定＋地震用積載）や
/// 荷重組合せの自動生成（長期・短期・多雪区域の係数）に用いる。
/// 旧スキーマ・種別未指定は `Other`（従来の「先頭ケースを重力とみなす」
/// フォールバック規則の対象）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadCaseKind {
    /// 固定荷重（自重・仕上げ）
    Dead,
    /// 積載荷重（架構用・長期）
    Live,
    /// 積載荷重（地震用）。地震用重量の集計にはこちらを用いる（令85条）。
    LiveSeismic,
    /// 積雪荷重
    Snow,
    /// 風荷重
    Wind,
    /// 地震荷重（自動生成された水平力など）
    Seismic,
    #[default]
    Other,
}

impl LoadCaseKind {
    /// 長期応力解析の対象となる荷重ケース種別か（RESP-D マニュアル計算編03「応力解析」）。
    ///
    /// 固定・積載・積雪（多雪区域の 0.7S 相当を含む常時荷重として登録される想定）と、
    /// 種別未指定 `Other`（従来の「先頭ケースを重力とみなす」フォールバック）を長期として扱う。
    /// 地震用積載（`LiveSeismic`。重量集計専用）・風・地震は短期側なので対象外。
    pub fn is_long_term(&self) -> bool {
        matches!(
            self,
            LoadCaseKind::Dead | LoadCaseKind::Live | LoadCaseKind::Snow | LoadCaseKind::Other
        )
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCase {
    pub id: LoadCaseId,
    pub name: String,
    pub nodal: Vec<NodalLoad>,
    /// 部材（梁）荷重。既存データとの後方互換のため `#[serde(default)]`。
    #[serde(default)]
    pub member: Vec<MemberLoad>,
    /// 荷重種別。旧スキーマは `Other`。
    #[serde(default)]
    pub kind: LoadCaseKind,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCombination {
    pub name: String,
    pub terms: Vec<(LoadCaseId, f64)>,
}

/// ダンパー装置の自重諸元（RESP-D マニュアル「ダンパー自重」）。
/// 自重 = 装置重量 + 支持部断面積 ×（節点間距離 − 装置長さ）× 鋼材単位体積重量。
/// 両端節点へ 1/2 ずつ伝達（鉛直配置は上下階へ、水平配置は同一階の両節点へ、
/// が節点標高から自然に成立する）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DamperSpec {
    pub elem: ElemId,
    /// 装置重量 [N]（直接入力）。自重を考慮しない装置は 0 を入力する
    /// （マニュアル「自重を考慮しない部材」）。
    pub device_weight: f64,
    /// 装置長さ [mm]。支持部長さ =（節点間距離 − 装置長さ）の算定に用いる。
    pub device_length: f64,
    /// 支持部断面積 [mm²]。0 なら支持部重量なし。
    pub support_area: f64,
}

/// K 型ブレースの重量配分規則（RESP-D 荷重計算条件「K型ブレースの重量配分」）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum KBraceWeightRule {
    /// 内部節点（ブレース同士のみが接続する節点）にも重量を配分する（両端 1/2）。
    #[default]
    InternalNodes,
    /// 基準節点（柱梁が接続する節点）にのみ重量を配分する。
    BaseNodesOnly,
}

/// 自重算定の付加設定（RESP-D マニュアル「柱梁自重」の鉄骨重量割増率・
/// 仕上げ荷重・耐火被覆・ダンパー自重・K型ブレース配分に対応する簡易版）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCfg {
    /// 鉄骨重量割増率 α（デフォルト 1.0）。コンクリート材（`fc` あり）には適用しない。
    /// 0 以下が入力された場合は 1.0 として扱う（RESP-D と同じ規則）。
    pub steel_weight_factor: f64,
    /// 部材ごとの付加線重量 [N/mm]（耐火被覆 γc·Ac 等の直接入力）。
    pub extra_line_weight: Vec<(ElemId, f64)>,
    /// 部材ごとの仕上げ面重量 w_f [N/mm²]。断面寸法から仕上げ周長
    /// （梁: b+2D の三面、柱: 2(b+D) の四周）を求めて線重量 w_f·φ に換算し
    /// 自重へ加算する（RESP-D マニュアル「柱梁自重」の仕上げ荷重）。
    #[serde(default)]
    pub finish_area_weight: Vec<(ElemId, f64)>,
    /// ダンパー装置の自重諸元。対象部材の断面自重（ρ·A·L·g）は使わず、
    /// この諸元による装置+支持部重量で置き換える。
    #[serde(default)]
    pub dampers: Vec<DamperSpec>,
    /// K 型ブレース（`ElementKind::Brace`）の重量配分規則。
    #[serde(default)]
    pub k_brace_rule: KBraceWeightRule,
    /// 支える床の数に応じた柱軸力算定時の積載荷重低減（令85条2項）を考慮するか。
    /// RESP-D と同じくデフォルトは「低減を考慮しない」。
    #[serde(default)]
    pub live_load_reduction: bool,
}

impl Default for LoadCfg {
    fn default() -> Self {
        Self {
            steel_weight_factor: 1.0,
            extra_line_weight: Vec::new(),
            finish_area_weight: Vec::new(),
            dampers: Vec::new(),
            k_brace_rule: KBraceWeightRule::default(),
            live_load_reduction: false,
        }
    }
}

impl LoadCfg {
    /// 有効な鉄骨重量割増率（0 以下の入力は 1.0 とみなす）。
    pub fn effective_steel_factor(&self) -> f64 {
        if self.steel_weight_factor > 0.0 {
            self.steel_weight_factor
        } else {
            1.0
        }
    }
}

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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dof::Dof6Mask;

    fn make_grid_model(n: usize) -> Model {
        let nodes: Vec<Node> = (0..n)
            .map(|i| Node {
                id: NodeId(i as u32),
                coord: [i as f64 * 1000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            })
            .collect();
        Model {
            nodes,
            ..Default::default()
        }
    }

    #[test]
    fn test_10k_node_traverse() {
        let n = 10_000;
        let model = make_grid_model(n);
        let t = std::time::Instant::now();
        let mut s = 0.0;
        for nd in &model.nodes {
            s += nd.coord[0];
        }
        assert!(t.elapsed().as_millis() < 50, "traverse too slow");
        std::hint::black_box(s);
    }

    #[test]
    fn test_validate_ok() {
        let model = make_grid_model(3);
        assert!(model.validate().is_ok());
    }

    #[test]
    fn test_validate_duplicate_node() {
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0; 3],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(0),
                    coord: [1.0; 3],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            ..Default::default()
        };
        assert!(model.validate().is_err());
    }

    #[test]
    fn test_validate_dangling_elem_node() {
        let model = Model {
            nodes: vec![Node {
                id: NodeId(0),
                coord: [0.0; 3],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(5)],
                section: None,
                material: None,
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            ..Default::default()
        };
        assert!(model.validate().is_err());
    }

    #[test]
    fn test_shear_modulus_explicit() {
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Test".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(80000.0),
            fc: None,
            fy: None,
        };
        assert_eq!(mat.shear_modulus(), 80000.0);
    }

    #[test]
    fn test_shear_modulus_derived() {
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Test".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let expected = 205000.0 / (2.0 * (1.0 + 0.3));
        assert!((mat.shear_modulus() - expected).abs() < 1e-9);
    }

    /// 旧スキーマ（concrete_class フィールドが無い JSON）の Material が
    /// 読み込めること（serde 後方互換。既定は Normal）。
    #[test]
    fn test_material_serde_backward_compat_concrete_class() {
        let json = r#"{
            "id": 0,
            "name": "FC24",
            "young": 23000.0,
            "poisson": 0.2,
            "density": 2.4e-9,
            "fc": 24.0
        }"#;
        let mat: Material = serde_json::from_str(json).unwrap();
        assert_eq!(mat.concrete_class, crate::units::ConcreteClass::Normal);
        assert_eq!(mat.fc, Some(24.0));

        // ラウンドトリップ（Lightweight1 が保存・復元できること）。
        let mat2 = Material {
            concrete_class: crate::units::ConcreteClass::Lightweight1,
            ..mat
        };
        let s = serde_json::to_string(&mat2).unwrap();
        let back: Material = serde_json::from_str(&s).unwrap();
        assert_eq!(
            back.concrete_class,
            crate::units::ConcreteClass::Lightweight1
        );
    }

    #[test]
    fn test_rect_shear_area() {
        let area = 80000.0;
        let as_ = rect_shear_area(area);
        assert!((as_ - area * 5.0 / 6.0).abs() < 1e-9);
    }

    /// 個別開口が非空なら面積和を優先し、空なら opening_area にフォールバックする。
    #[test]
    fn test_wall_attr_total_opening_area_prefers_openings() {
        let mut attr = WallAttr {
            elem: ElemId(0),
            opening_area: 999.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![
                WallOpening {
                    width: 1000.0,
                    height: 2000.0,
                    offset: None,
                },
                WallOpening {
                    width: 500.0,
                    height: 800.0,
                    offset: Some([3000.0, 500.0]),
                },
            ],
        };
        assert!((attr.total_opening_area() - (2.0e6 + 4.0e5)).abs() < 1e-9);
        assert_eq!(
            attr.opening_dims(),
            Some(vec![(1000.0, 2000.0), (500.0, 800.0)])
        );

        attr.openings.clear();
        assert!((attr.total_opening_area() - 999.0).abs() < 1e-9);
        assert_eq!(attr.opening_dims(), None);

        // 面積ゼロの開口だけなら寸法列は None(面積のみ扱い)
        attr.openings.push(WallOpening {
            width: 0.0,
            height: 1000.0,
            offset: None,
        });
        assert_eq!(attr.opening_dims(), None);
        assert_eq!(attr.total_opening_area(), 0.0);
    }

    fn op(w: f64, h: f64, offset: Option<[f64; 2]>) -> WallOpening {
        WallOpening {
            width: w,
            height: h,
            offset,
        }
    }

    fn attr_with(openings: Vec<WallOpening>) -> WallAttr {
        WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings,
        }
    }

    /// 包絡モード: 位置を持つ開口は外接矩形1つに統合、位置不明は個別のまま。
    #[test]
    fn test_openings_for_mode_envelope() {
        let attr = attr_with(vec![
            op(1000.0, 1000.0, Some([0.0, 0.0])),
            op(500.0, 800.0, Some([2000.0, 1200.0])),
            op(300.0, 300.0, None), // 位置不明
        ]);
        let out = attr.openings_for_mode(MultiOpeningMode::Envelope);
        assert_eq!(out.len(), 2);
        // 包絡矩形: x0=0,z0=0,x1=2500,z1=2000
        assert!((out[0].width - 2500.0).abs() < 1e-9);
        assert!((out[0].height - 2000.0).abs() < 1e-9);
        assert_eq!(out[0].offset, Some([0.0, 0.0]));
        assert!((out[1].width - 300.0).abs() < 1e-9);
        // 包絡モードの面積は包絡矩形基準(生の面積和より大きい)
        let a_env = attr.total_opening_area_for(MultiOpeningMode::Envelope);
        assert!(a_env > attr.total_opening_area());
    }

    /// 自動判定: 近接対のみ包絡を繰り返し、離れた開口は残る。
    #[test]
    fn test_openings_for_mode_auto_merges_close_pairs_only() {
        // 開口1と2は水平間隔200(≤min幅)で包絡可能。開口3は間隔5000で不可。
        let attr = attr_with(vec![
            op(1000.0, 2000.0, Some([0.0, 0.0])),
            op(800.0, 2000.0, Some([1200.0, 0.0])),
            op(900.0, 2000.0, Some([7000.0, 0.0])),
        ]);
        let out = attr.openings_for_mode(MultiOpeningMode::Auto);
        assert_eq!(out.len(), 2);
        // 包絡結果: 幅 0..2000
        assert!((out[0].width - 2000.0).abs() < 1e-9);
        assert!((out[1].width - 900.0).abs() < 1e-9);
        // 等価モードは元のまま
        assert_eq!(
            attr.openings_for_mode(MultiOpeningMode::Equivalent).len(),
            3
        );
    }

    /// 自動判定の包絡可能条件(RESP-D 計算編02 判定図):
    /// l < 1.5h または l < 1m(l: 開口間距離、h: 包絡開口とした場合の高さ)。
    #[test]
    fn test_can_envelope_boundary() {
        // h(包絡高さ)=2000 → 1.5h=3000
        let a = op(1000.0, 2000.0, Some([0.0, 0.0]));
        // 開口間距離 2999 < 1.5h → 包絡可
        let b = op(1000.0, 2000.0, Some([3999.0, 0.0]));
        assert!(a.can_envelope(&b));
        // 開口間距離 3000 = 1.5h(かつ ≥1m) → 不可
        let c = op(1000.0, 2000.0, Some([4000.0, 0.0]));
        assert!(!a.can_envelope(&c));

        // 低い開口(h=500 → 1.5h=750 < 1m)でも l < 1m なら包絡可
        let e = op(1000.0, 500.0, Some([0.0, 0.0]));
        let f = op(1000.0, 500.0, Some([1999.0, 0.0])); // l=999 < 1000
        assert!(e.can_envelope(&f));
        let g = op(1000.0, 500.0, Some([2000.0, 0.0])); // l=1000(≥1m かつ ≥1.5h)
        assert!(!e.can_envelope(&g));

        // 位置不明は不可
        let d = op(1000.0, 2000.0, None);
        assert!(!a.can_envelope(&d));
    }

    /// 旧スキーマ(openings 無し)の WallAttr が読み込めること(serde 後方互換)。
    #[test]
    fn test_wall_attr_serde_backward_compat() {
        let json = r#"{"elem":3,"opening_area":1200.0,"three_side_slit":true}"#;
        let attr: WallAttr = serde_json::from_str(json).unwrap();
        assert_eq!(attr.elem, ElemId(3));
        assert!(attr.openings.is_empty());
        assert!((attr.total_opening_area() - 1200.0).abs() < 1e-9);
        assert!(attr.three_side_slit);
    }

    #[test]
    fn test_section_new_fields_default() {
        let sec = Section {
            id: SectionId(0),
            name: "Test".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 2000.0,
            j: 500.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        assert_eq!(sec.depth, 0.0);
        assert!(sec.panel_thickness.is_none());
    }

    #[test]
    fn test_element_data_plastic_zone_default_missing_field() {
        // 旧スキーマ（plastic_zone フィールドが無い JSON）からの互換性を確認する。
        let json = r#"{
            "id": 0,
            "kind": "Beam",
            "nodes": [0, 1],
            "section": null,
            "material": null,
            "local_axis": { "ref_vector": [1.0, 0.0, 0.0] },
            "end_cond": ["Fixed", "Fixed"],
            "force_regime": "Auto"
        }"#;
        let elem: ElementData = serde_json::from_str(json).unwrap();
        assert_eq!(elem.plastic_zone, None);
        assert_eq!(elem.rigid_zone, RigidZone::default());
    }

    /// 長期系（固定・積載・積雪・種別未指定）は長期、地震用積載・風・地震は短期
    /// （RESP-D マニュアル計算編03「応力解析」の長期軸力無効化条件の適用範囲）。
    #[test]
    fn test_load_case_kind_is_long_term() {
        assert!(LoadCaseKind::Dead.is_long_term());
        assert!(LoadCaseKind::Live.is_long_term());
        assert!(LoadCaseKind::Snow.is_long_term());
        assert!(LoadCaseKind::Other.is_long_term());
        assert!(!LoadCaseKind::LiveSeismic.is_long_term());
        assert!(!LoadCaseKind::Wind.is_long_term());
        assert!(!LoadCaseKind::Seismic.is_long_term());
    }

    #[test]
    fn test_stress_cfg_default_is_false() {
        let cfg = StressAnalysisCfg::default();
        assert!(!cfg.no_long_axial_brace);
        assert!(!cfg.no_long_axial_column);
        assert_eq!(Model::default().stress_cfg, cfg);
    }

    #[test]
    fn test_model_stress_cfg_default_missing_field() {
        // 旧スキーマ（stress_cfg フィールドが無い JSON）からの互換性を確認する。
        let json = r#"{
            "nodes": [], "elements": [], "sections": [], "materials": [],
            "stories": [], "slabs": [], "constraints": [], "load_cases": [],
            "combinations": []
        }"#;
        let model: Model = serde_json::from_str(json).unwrap();
        assert_eq!(model.stress_cfg, StressAnalysisCfg::default());
    }

    #[test]
    fn test_validate_index_mismatch() {
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0; 3],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(5),
                    coord: [1.0; 3],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            ..Default::default()
        };
        assert!(model.validate().is_err());
    }
}
