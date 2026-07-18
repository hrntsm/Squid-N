//! スラブ（床）関連の型。
//!
//! - [`DistributionMethod`] — 床荷重の分配方法。
//! - [`JoistLine`] — 小梁ライン。
//! - [`AreaLoad`] — 面荷重。
//! - [`SlabKind`] — スラブ種別（一般／片持ち／出隅）。
//! - [`OneWayDir`] — 一方向スラブの伝達方向。
//! - [`LoadPurpose`] — 積載荷重の用途（床用／骨組用／地震用。令85条1項）。
//! - [`SlabUsage`] — 室用途（令別表第1 の積載荷重プリセット）。
//! - [`Slab`] — スラブの定義。

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DistributionMethod {
    TriTrapezoid,
    OneWay,
    TributaryArea,
}

/// 積載荷重の用途（令85条1項・令別表第1 の 3 欄）。
/// - `Floor`（床用）: 床スラブ・小梁の設計用。最も大きい。
/// - `Frame`（骨組用）: 大梁・柱・基礎の設計用（長期骨組解析に用いる）。
/// - `Seismic`（地震用）: 地震力（地震用重量）の算定用。最も小さい。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadPurpose {
    Floor,
    Frame,
    Seismic,
}

/// 室の用途（令別表第1 の積載荷重プリセット）。`live_load` で用途別の積載荷重
/// [N/mm²] を返す。`Custom` は 3 欄を直接持つ（内部単位 N/mm²）。
///
/// 出典: 建築基準法施行令 第85条第1項・令別表第1。値は N/m² を内部単位 N/mm²
/// （×1e-6）へ換算して返す。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SlabUsage {
    /// 住宅の居室、住宅以外の建築物における寝室又は病室。
    Residential,
    /// 事務室。
    Office,
    /// 教室。
    Classroom,
    /// 百貨店又は店舗の売場。
    Store,
    /// 劇場・映画館等の客席又は集会室（固定席の場合）。
    AssemblyFixed,
    /// 同上（その他の場合）。
    AssemblyOther,
    /// 廊下・玄関・階段（劇場・集会場・売場等に連絡するもの）。
    Corridor,
    /// 自動車車庫及び自動車通路。
    Garage,
    /// 屋上広場又はバルコニー（住宅系＝令別表第1(一)の数値）。
    RoofResidential,
    /// 屋上広場又はバルコニー（学校・百貨店系＝令別表第1(四)の数値）。
    RoofStore,
    /// 任意入力（床用・骨組用・地震用、いずれも N/mm²）。
    Custom {
        floor: f64,
        frame: f64,
        seismic: f64,
    },
}

impl SlabUsage {
    /// 用途別の積載荷重 [N/mm²]（令別表第1）。
    pub fn live_load(self, purpose: LoadPurpose) -> f64 {
        // プリセットは令別表第1 の [N/m²]。内部単位 N/mm² へ ×1e-6。
        // 返り値の並びは (床用, 骨組用, 地震用)。
        let (floor, frame, seismic) = match self {
            SlabUsage::Residential => (1800.0, 1300.0, 600.0),
            SlabUsage::Office => (2900.0, 1800.0, 800.0),
            SlabUsage::Classroom => (2300.0, 2100.0, 1100.0),
            SlabUsage::Store => (2900.0, 2400.0, 1300.0),
            SlabUsage::AssemblyFixed => (2900.0, 2600.0, 1600.0),
            SlabUsage::AssemblyOther => (3500.0, 3200.0, 2100.0),
            SlabUsage::Corridor => (3500.0, 3200.0, 2100.0),
            SlabUsage::Garage => (5400.0, 3900.0, 2000.0),
            SlabUsage::RoofResidential => (1800.0, 1300.0, 600.0),
            SlabUsage::RoofStore => (2900.0, 2400.0, 1300.0),
            // Custom は内部単位 N/mm² をそのまま返す（×1e-6 しない）。
            SlabUsage::Custom {
                floor,
                frame,
                seismic,
            } => {
                return match purpose {
                    LoadPurpose::Floor => floor,
                    LoadPurpose::Frame => frame,
                    LoadPurpose::Seismic => seismic,
                };
            }
        };
        let v_n_per_m2 = match purpose {
            LoadPurpose::Floor => floor,
            LoadPurpose::Frame => frame,
            LoadPurpose::Seismic => seismic,
        };
        v_n_per_m2 * 1e-6
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct JoistLine {
    pub dir: [f64; 2],
    pub spacing: f64,
    pub support: [NodeId; 2],
    /// 小梁の断面参照（床の中での小梁設計に用いる。単純支持梁として検定する）。
    /// `None`（旧スキーマ・未設定）は断面未割当（設計対象外）。
    #[serde(default)]
    pub section: Option<crate::ids::SectionId>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AreaLoad {
    pub kind: String,
    pub value: f64,
}

/// スラブの種別。片持ちスラブは境界の辺 0（`boundary[0]`→`boundary[1]`）を
/// 取付き辺（大梁側）とし、荷重は取付き辺へ伝達する（片持ちスラブの床荷重分配）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SlabKind {
    #[default]
    Interior,
    Cantilever,
    /// 出隅の片持ちスラブ。荷重は伝達方向・片持ち梁の有無に関わらず
    /// 全て節点荷重として柱（`boundary[0]` の節点）へ伝達する
    /// （出隅の片持ちスラブの床荷重分配）。
    Corner,
}

/// 一方向スラブの荷重伝達方向（床ごとに指定。床荷重の分配における伝達方向〔X〕〔Y〕）。
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
    /// （片持ちスラブに片持ち梁あり/先端リブ小梁ありの場合の床荷重分配）。
    #[serde(default)]
    pub edge_supported: Option<Vec<bool>>,
    /// 室用途（令別表第1）。`Some` のとき積載荷重（LL）を用途別に自動算定する。
    /// `None`（旧スキーマ・未設定）は積載荷重を持たない（`loads` の固定荷重のみ）。
    #[serde(default)]
    pub usage: Option<SlabUsage>,
    /// スラブ厚さ [mm]。ST-Bridge（`StbSecSlab_RC`）往復で個別スラブごとの厚さを
    /// 保持するために用いる。`None`（旧スキーマ・未設定）は建物一律の
    /// `Model::slab_thickness` を用いる従来互換。
    #[serde(default)]
    pub thickness: Option<f64>,
}

impl Slab {
    /// 固定荷重（DL）の面荷重強度 [N/mm²]。`loads`（仕上げ等）の合算。
    pub fn dead_intensity(&self) -> f64 {
        self.loads.iter().map(|l| l.value).sum()
    }

    /// 用途別の積載荷重（LL）の面荷重強度 [N/mm²]。`usage` 未設定なら 0。
    pub fn live_intensity(&self, purpose: LoadPurpose) -> f64 {
        self.usage.map(|u| u.live_load(purpose)).unwrap_or(0.0)
    }

    /// 用途に応じた合成面荷重強度 [N/mm²]（固定 DL ＋ 積載 LL(purpose)）。
    /// 長期骨組解析は `Frame`、地震用重量は `Seismic`、床・小梁設計は `Floor`。
    pub fn intensity(&self, purpose: LoadPurpose) -> f64 {
        self.dead_intensity() + self.live_intensity(purpose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{NodeId, SlabId};

    fn slab_with(usage: Option<SlabUsage>, dead_loads: &[f64]) -> Slab {
        Slab {
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: dead_loads
                .iter()
                .map(|&v| AreaLoad {
                    kind: "DL".into(),
                    value: v,
                })
                .collect(),
            method: DistributionMethod::TriTrapezoid,
            kind: SlabKind::Interior,
            one_way: None,
            edge_supported: None,
            thickness: None,
            usage,
        }
    }

    #[test]
    fn test_usage_table_values_n_per_mm2() {
        // 令別表第1: 事務室 = 床用 2900 / 骨組用 1800 / 地震用 800 [N/m²]。
        let o = SlabUsage::Office;
        assert!((o.live_load(LoadPurpose::Floor) - 2900e-6).abs() < 1e-12);
        assert!((o.live_load(LoadPurpose::Frame) - 1800e-6).abs() < 1e-12);
        assert!((o.live_load(LoadPurpose::Seismic) - 800e-6).abs() < 1e-12);
        // 住宅 = 1800 / 1300 / 600。
        let r = SlabUsage::Residential;
        assert!((r.live_load(LoadPurpose::Floor) - 1800e-6).abs() < 1e-12);
        assert!((r.live_load(LoadPurpose::Frame) - 1300e-6).abs() < 1e-12);
        assert!((r.live_load(LoadPurpose::Seismic) - 600e-6).abs() < 1e-12);
        // 積載は 床用 ≥ 骨組用 ≥ 地震用 の順（全用途で成り立つ）。
        for u in [
            SlabUsage::Residential,
            SlabUsage::Office,
            SlabUsage::Classroom,
            SlabUsage::Store,
            SlabUsage::AssemblyFixed,
            SlabUsage::AssemblyOther,
            SlabUsage::Corridor,
            SlabUsage::Garage,
            SlabUsage::RoofResidential,
            SlabUsage::RoofStore,
        ] {
            let f = u.live_load(LoadPurpose::Floor);
            let g = u.live_load(LoadPurpose::Frame);
            let s = u.live_load(LoadPurpose::Seismic);
            assert!(f >= g && g >= s, "床用≥骨組用≥地震用: {u:?}");
        }
    }

    #[test]
    fn test_usage_custom_is_internal_units() {
        // Custom は内部単位 N/mm² をそのまま返す（換算しない）。
        let c = SlabUsage::Custom {
            floor: 3.0e-3,
            frame: 2.0e-3,
            seismic: 1.0e-3,
        };
        assert_eq!(c.live_load(LoadPurpose::Floor), 3.0e-3);
        assert_eq!(c.live_load(LoadPurpose::Frame), 2.0e-3);
        assert_eq!(c.live_load(LoadPurpose::Seismic), 1.0e-3);
    }

    #[test]
    fn test_slab_intensity_helpers() {
        // DL のみ（usage None）。
        let s = slab_with(None, &[1.0e-3, 0.5e-3]);
        assert!((s.dead_intensity() - 1.5e-3).abs() < 1e-12);
        assert_eq!(s.live_intensity(LoadPurpose::Frame), 0.0);
        assert!((s.intensity(LoadPurpose::Frame) - 1.5e-3).abs() < 1e-12);

        // DL + 用途積載。骨組用の合成 = DL + LL(骨組用)。
        let s = slab_with(Some(SlabUsage::Office), &[1.0e-3]);
        assert!((s.live_intensity(LoadPurpose::Frame) - 1800e-6).abs() < 1e-12);
        assert!((s.intensity(LoadPurpose::Frame) - (1.0e-3 + 1800e-6)).abs() < 1e-12);
        // 地震用は積載が小さい。
        assert!(s.intensity(LoadPurpose::Seismic) < s.intensity(LoadPurpose::Frame));
        assert!(s.intensity(LoadPurpose::Frame) < s.intensity(LoadPurpose::Floor));
    }

    /// 旧スキーマ（usage 欄なし）の JSON が読める（後方互換）。
    #[test]
    fn test_slab_serde_backward_compat_no_usage() {
        let json =
            r#"{"id":0,"boundary":[0,1,2,3],"joists":[],"loads":[],"method":"TriTrapezoid"}"#;
        let s: Slab = serde_json::from_str(json).expect("旧スキーマを読める");
        assert_eq!(s.usage, None);
    }
}
