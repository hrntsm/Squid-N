//! 荷重関連の型（節点荷重・部材荷重・荷重ケース・荷重条件など）。

use super::*;

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
    /// 長期応力解析の対象となる荷重ケース種別か（令82条の応力解析）。
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

/// ダンパー装置の自重諸元（固定荷重）。
/// 自重 = 装置重量 + 支持部断面積 ×（節点間距離 − 装置長さ）× 鋼材単位体積重量。
/// 両端節点へ 1/2 ずつ伝達（鉛直配置は上下階へ、水平配置は同一階の両節点へ、
/// が節点標高から自然に成立する）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DamperSpec {
    pub elem: ElemId,
    /// 装置重量 [N]（直接入力）。自重を考慮しない装置は 0 を入力する
    /// （自重を考慮しない部材の扱い）。
    pub device_weight: f64,
    /// 装置長さ [mm]。支持部長さ =（節点間距離 − 装置長さ）の算定に用いる。
    pub device_length: f64,
    /// 支持部断面積 [mm²]。0 なら支持部重量なし。
    pub support_area: f64,
}

/// K 型ブレースの重量配分規則（固定荷重の重量配分規則）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum KBraceWeightRule {
    /// 内部節点（ブレース同士のみが接続する節点）にも重量を配分する（両端 1/2）。
    #[default]
    InternalNodes,
    /// 基準節点（柱梁が接続する節点）にのみ重量を配分する。
    BaseNodesOnly,
}

/// 自重算定の付加設定（固定荷重の鉄骨重量割増率・
/// 仕上げ荷重・耐火被覆・ダンパー自重・K型ブレース配分に対応する簡易版）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCfg {
    /// 鉄骨重量割増率 α（デフォルト 1.0）。コンクリート材（`fc` あり）には適用しない。
    /// 0 以下が入力された場合は 1.0 として扱う（本実装の規則）。
    pub steel_weight_factor: f64,
    /// 部材ごとの付加線重量 [N/mm]（耐火被覆 γc·Ac 等の直接入力）。
    pub extra_line_weight: Vec<(ElemId, f64)>,
    /// 部材ごとの仕上げ面重量 w_f [N/mm²]。断面寸法から仕上げ周長
    /// （梁: b+2D の三面、柱: 2(b+D) の四周）を求めて線重量 w_f·φ に換算し
    /// 自重へ加算する（固定荷重の仕上げ荷重）。
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
    /// デフォルトは「低減を考慮しない」。
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
