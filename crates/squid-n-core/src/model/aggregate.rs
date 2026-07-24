//! モデル全体を束ねる集約型。
//!
//! - [`ElemAttrs`] — 要素の側テーブル属性スナップショット（undo 用）。
//! - [`Model`] — 構造モデル全体（節点・要素・断面・材料・階・荷重等）。

use super::*;

/// 1 つの要素に紐づく側テーブル属性のスナップショット。要素の削除・挿入
/// （[`Model::take_elem_attrs`] / [`Model::restore_elem_attrs`]）で属性の
/// 退避・復元に用いる（undo 用の一時保持。直列化はしない）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ElemAttrs {
    pub wall: Option<WallAttr>,
    pub steel_design: Option<SteelDesignAttr>,
    pub brb: Option<BrbAttr>,
    pub pca: Option<PcaBeamAttr>,
    pub isolator: Option<IsolatorAttr>,
    pub hysteresis: Option<MemberHysteresisAttr>,
    pub damper: Option<DamperAttr>,
    pub detail: Option<MemberDetailAttr>,
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
    /// 動的解析の質量モデルの方式（[`MassMethod`]）。階の自動生成が剛床マスターへ
    /// 与える質点質量の算定と、質量行列組立での部材密度質量の要否を規定する。
    /// 旧スキーマは補正質点方式（従来の密度質量＋節点質量と同じ組立）扱い。
    #[serde(default)]
    pub mass_method: MassMethod,
    /// 剛性計算用の床スラブ厚 [mm]（建物全体で一律。スラブ協力幅による梁剛性
    /// 増大の算定に用いる。RC 規準）。0 以下でスラブ協力幅による梁剛性増大を無効化（既定）。
    #[serde(default)]
    pub slab_thickness: f64,
    /// 自重算定の付加設定（鉄骨重量割増率・部材付加線重量）。`None` は既定値。
    #[serde(default)]
    pub load_cfg: Option<LoadCfg>,
    /// 壁要素の自重算定属性（開口・三方スリット）。
    #[serde(default)]
    pub wall_attrs: Vec<WallAttr>,
    /// 複数開口の取り扱い（建物一律。耐震壁の開口。RC 規準）。
    /// 剛性の開口低減・耐震壁判定・検定への開口供給に適用する
    /// （自重控除は常に生の開口面積和）。既定は「等価開口とする」。
    #[serde(default)]
    pub multi_opening_mode: MultiOpeningMode,
    /// フレーム外雑壁。
    #[serde(default)]
    pub misc_walls: Vec<MiscWall>,
    /// 応力解析の計算条件（令82条の応力解析。長期軸力を負担させない部材の指定）。
    #[serde(default)]
    pub stress_cfg: StressAnalysisCfg,
    /// S 造部材の断面検定用属性（継手部・スカラップ欠損、横座屈長さ指定。
    /// 鋼構造設計規準）。
    #[serde(default)]
    pub steel_design_attrs: Vec<SteelDesignAttr>,
    /// 座屈補剛ブレース（BRB）の断面検定用属性（メーカー許容値。
    /// 各メーカーの製品技術資料）。
    #[serde(default)]
    pub brb_attrs: Vec<BrbAttr>,
    /// PCa（プレキャスト）梁の水平接合面検定用属性（水平接合面のせん断摩擦検定）。
    #[serde(default)]
    pub pca_attrs: Vec<PcaBeamAttr>,
    /// 免震支承材の非線形特性（`ElementKind::Isolator` 要素、各免震部材指針）。
    #[serde(default)]
    pub isolator_attrs: Vec<IsolatorAttr>,
    /// 部材の履歴則の個別指定（各履歴則の原典）。
    /// 未指定の部材は構造種別ごとの既定（[`default_member_hysteresis`]）に従う。
    #[serde(default)]
    pub member_hysteresis_attrs: Vec<MemberHysteresisAttr>,
    /// 制振ダンパー要素（`ElementKind::Damper`）の特性（各制振部材の力学モデル）。
    #[serde(default)]
    pub damper_attrs: Vec<DamperAttr>,
    /// 部材の付帯情報（端部ハンチ・継手位置）。剛性・応力解析には影響しない
    /// （設計書 §6.2。剛性は基準断面のまま）。断面算定の検定位置の追加
    /// （ハンチ端・継手位置、§6.2.3）と数量拾いに用いる。
    #[serde(default)]
    pub member_detail_attrs: Vec<MemberDetailAttr>,
    /// 二次部材（小梁・間柱）。全体解析（剛性）には算入せず、床荷重・自重を
    /// 主架構への荷重（CMQ）として伝達する（[`SecondaryMember`]）。
    #[serde(default)]
    pub secondary_members: Vec<SecondaryMember>,
    /// 一本部材の指定（断面検定の採用応力。一本部材指定時の採用応力の扱い）。
    /// 各エントリは**軸方向に連続する梁要素の ID を並び順**で持ち、
    /// 断面検定の採用応力（端部・中央モーメント、部材長、内法長、せん断スパン比
    /// 代表値）をグループ 1 本の部材として評価する。要素の解析（剛性・内力）は
    /// 分割部材のまま行い、検定の文脈だけを合成する。
    #[serde(default)]
    pub beam_groups: Vec<Vec<ElemId>>,
    #[serde(skip)]
    pub dof_map: crate::dof::DofMap,
}

/// コレクション内の id が「配列添字 == id.index()」かつ重複しないことを検証する。
/// `coll` は配列名（例 "nodes"）、`id_name` は id 型名（例 "NodeId"）。
fn check_id_consistency<T>(
    items: &[T],
    coll: &str,
    id_name: &str,
    index_of: impl Fn(&T) -> usize,
    raw_of: impl Fn(&T) -> u32,
) -> Result<(), crate::error::CoreError> {
    use crate::error::CoreError;
    for (i, item) in items.iter().enumerate() {
        if index_of(item) != i {
            return Err(CoreError::IndexMismatch(format!(
                "{coll}[{i}] has {id_name}({})",
                raw_of(item)
            )));
        }
    }
    let mut seen = std::collections::HashSet::new();
    for item in items {
        if !seen.insert(index_of(item)) {
            return Err(CoreError::DuplicateId(format!(
                "{id_name}({})",
                raw_of(item)
            )));
        }
    }
    Ok(())
}

impl Model {
    pub fn validate(&self) -> Result<(), crate::error::CoreError> {
        use crate::error::CoreError;

        check_id_consistency(&self.nodes, "nodes", "NodeId", |n| n.id.index(), |n| n.id.0)?;

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

        check_id_consistency(
            &self.stories,
            "stories",
            "StoryId",
            |s| s.id.index(),
            |s| s.id.0,
        )?;
        check_id_consistency(&self.slabs, "slabs", "SlabId", |s| s.id.index(), |s| s.id.0)?;
        // スラブの境界・小梁が参照する節点が実在すること（陳腐化した参照の検出）。
        for slab in &self.slabs {
            for &nid in &slab.boundary {
                if nid.index() >= self.nodes.len() || self.nodes[nid.index()].id != nid {
                    return Err(CoreError::DanglingRef(format!(
                        "Slab {} boundary -> Node {}",
                        slab.id.0, nid.0
                    )));
                }
            }
            for (ji, j) in slab.joists.iter().enumerate() {
                for &nid in &j.support {
                    if nid.index() >= self.nodes.len() || self.nodes[nid.index()].id != nid {
                        return Err(CoreError::DanglingRef(format!(
                            "Slab {} joist support -> Node {}",
                            slab.id.0, nid.0
                        )));
                    }
                }
                if let Some(sid) = j.section {
                    if sid.index() >= self.sections.len() || self.sections[sid.index()].id != sid {
                        return Err(CoreError::DanglingRef(format!(
                            "Slab {} joist -> Section {}",
                            slab.id.0, sid.0
                        )));
                    }
                }
                // ピン受け/架けの相手小梁インデックスは同一スラブ内の別小梁を指す。
                if let Some(c) = j.pinned_onto {
                    if c >= slab.joists.len() || c == ji {
                        return Err(CoreError::DanglingRef(format!(
                            "Slab {} joist {} pinned_onto -> joist {}",
                            slab.id.0, ji, c
                        )));
                    }
                }
            }
        }
        check_id_consistency(
            &self.sections,
            "sections",
            "SectionId",
            |s| s.id.index(),
            |s| s.id.0,
        )?;
        check_id_consistency(
            &self.materials,
            "materials",
            "MaterialId",
            |m| m.id.index(),
            |m| m.id.0,
        )?;

        // 二次部材（小梁・間柱）の参照整合。
        for (i, sm) in self.secondary_members.iter().enumerate() {
            for &nid in &sm.nodes {
                if nid.index() >= self.nodes.len() || self.nodes[nid.index()].id != nid {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} -> Node {}",
                        i, nid.0
                    )));
                }
            }
            if let Some(sid) = sm.section {
                if sid.index() >= self.sections.len() || self.sections[sid.index()].id != sid {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} -> Section {}",
                        i, sid.0
                    )));
                }
            }
            if let Some(mid) = sm.material {
                if mid.index() >= self.materials.len() || self.materials[mid.index()].id != mid {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} -> Material {}",
                        i, mid.0
                    )));
                }
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
            || self
                .secondary_members
                .iter()
                .any(|sm| sm.nodes.contains(&id))
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
            && self.mass_method == other.mass_method
            && self.load_cfg == other.load_cfg
            && self.wall_attrs == other.wall_attrs
            && self.misc_walls == other.misc_walls
            && self.stress_cfg == other.stress_cfg
            && self.steel_design_attrs == other.steel_design_attrs
            && self.brb_attrs == other.brb_attrs
            && self.pca_attrs == other.pca_attrs
            && self.beam_groups == other.beam_groups
            && self.isolator_attrs == other.isolator_attrs
            && self.member_hysteresis_attrs == other.member_hysteresis_attrs
            && self.damper_attrs == other.damper_attrs
            && self.member_detail_attrs == other.member_detail_attrs
    }

    /// ダンパー要素の特性を返す（`Model::damper_attrs` から要素 ID で検索）。
    pub fn damper_props(&self, elem: ElemId) -> Option<DamperProps> {
        self.damper_attrs
            .iter()
            .find(|a| a.elem == elem)
            .map(|a| a.props)
    }

    /// ダンパー要素の特性を設定／解除する。`None` を渡すと指定を解除する。
    /// 戻り値は変更前の指定（undo 用）。
    pub fn set_damper_props(
        &mut self,
        elem: ElemId,
        props: Option<DamperProps>,
    ) -> Option<DamperProps> {
        let old = self.damper_props(elem);
        self.damper_attrs.retain(|a| a.elem != elem);
        if let Some(p) = props {
            self.damper_attrs.push(DamperAttr { elem, props: p });
        }
        old
    }

    /// 要素に紐づく全ての側テーブル属性（壁・鉄骨・BRB・PCa・免震・履歴則・ダンパー）の
    /// `elem` 参照に `f` を適用する。要素の追加・削除に伴う ID 繰上げ／繰下げで、
    /// 側テーブルの参照整合を保つために用いる。
    pub fn shift_elem_attr_refs(&mut self, mut f: impl FnMut(&mut ElemId)) {
        for a in &mut self.wall_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.steel_design_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.brb_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.pca_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.isolator_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.member_hysteresis_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.damper_attrs {
            f(&mut a.elem);
        }
        for a in &mut self.member_detail_attrs {
            f(&mut a.elem);
        }
    }

    /// 指定要素に紐づく全ての側テーブル属性を取り外して返す（要素削除時の退避用）。
    pub fn take_elem_attrs(&mut self, elem: ElemId) -> ElemAttrs {
        /// `elem` フィールドが一致する最初の要素を取り外して返す。
        fn take_first<T>(v: &mut Vec<T>, get: impl Fn(&T) -> ElemId, elem: ElemId) -> Option<T> {
            v.iter()
                .position(|a| get(a) == elem)
                .map(|pos| v.remove(pos))
        }
        ElemAttrs {
            wall: take_first(&mut self.wall_attrs, |a| a.elem, elem),
            steel_design: take_first(&mut self.steel_design_attrs, |a| a.elem, elem),
            brb: take_first(&mut self.brb_attrs, |a| a.elem, elem),
            pca: take_first(&mut self.pca_attrs, |a| a.elem, elem),
            isolator: take_first(&mut self.isolator_attrs, |a| a.elem, elem),
            hysteresis: take_first(&mut self.member_hysteresis_attrs, |a| a.elem, elem),
            damper: take_first(&mut self.damper_attrs, |a| a.elem, elem),
            detail: take_first(&mut self.member_detail_attrs, |a| a.elem, elem),
        }
    }

    /// 取り外した側テーブル属性を、指定要素 ID へ紐づけ直して復元する
    /// （要素削除の undo 用）。各属性の `elem` は `elem` へ上書きする。
    pub fn restore_elem_attrs(&mut self, elem: ElemId, attrs: ElemAttrs) {
        if let Some(mut a) = attrs.wall {
            a.elem = elem;
            self.wall_attrs.push(a);
        }
        if let Some(mut a) = attrs.steel_design {
            a.elem = elem;
            self.steel_design_attrs.push(a);
        }
        if let Some(mut a) = attrs.brb {
            a.elem = elem;
            self.brb_attrs.push(a);
        }
        if let Some(mut a) = attrs.pca {
            a.elem = elem;
            self.pca_attrs.push(a);
        }
        if let Some(mut a) = attrs.isolator {
            a.elem = elem;
            self.isolator_attrs.push(a);
        }
        if let Some(mut a) = attrs.hysteresis {
            a.elem = elem;
            self.member_hysteresis_attrs.push(a);
        }
        if let Some(mut a) = attrs.damper {
            a.elem = elem;
            self.damper_attrs.push(a);
        }
        if let Some(mut a) = attrs.detail {
            a.elem = elem;
            self.member_detail_attrs.push(a);
        }
    }

    /// 部材の付帯情報（ハンチ・継手位置）を返す（未指定は `None`）。
    pub fn member_detail(&self, elem: ElemId) -> Option<&MemberDetailAttr> {
        self.member_detail_attrs.iter().find(|a| a.elem == elem)
    }

    /// 部材に指定された履歴則を返す（未指定は `None`＝既定に従う）。
    pub fn member_hysteresis(&self, elem: ElemId) -> Option<HysteresisModel> {
        self.member_hysteresis_attrs
            .iter()
            .find(|a| a.elem == elem)
            .map(|a| a.rule)
    }

    /// 部材の履歴則を設定する。`HysteresisModel::Auto` を指定した場合は指定を解除
    /// （既定に従う）。戻り値は変更前の指定（undo 用）。
    pub fn set_member_hysteresis(
        &mut self,
        elem: ElemId,
        rule: HysteresisModel,
    ) -> Option<HysteresisModel> {
        let old = self.member_hysteresis(elem);
        self.member_hysteresis_attrs.retain(|a| a.elem != elem);
        if rule != HysteresisModel::Auto {
            self.member_hysteresis_attrs
                .push(MemberHysteresisAttr { elem, rule });
        }
        old
    }

    /// 標準荷重ケース一式（DL・LL(架構用)・LL(地震用)・EX・EY）と標準荷重組合せ
    /// （長期 DL+LL、短期地震 DL+LL±EX・DL+LL±EY）を持つ空モデルを作る
    /// （新規作成の既定。[`default_load_cases`]・[`default_combinations`] 参照）。
    pub fn with_default_load_cases() -> Self {
        Model {
            load_cases: default_load_cases(),
            combinations: default_combinations(),
            ..Model::default()
        }
    }

    /// 旧スキーマの自動生成荷重ケース名を標準ケース名へ移行する（読込時の後方互換）。
    ///
    /// - 「床荷重(自動)」→「DL」、「床積載(自動)」→「LL(架構用)」、
    ///   「床地震用積載(自動)」→「LL(地震用)」に改名する
    ///   （移行先の名前が既に使われている場合は改名しない）。
    /// - 「自重(自動)」は DL へ統合する（自重は DL の同期内容に含まれるように
    ///   なったため）。DL ケースが存在する場合は「自重(自動)」を削除し、
    ///   荷重組合せの参照は DL へ付け替える（同一組合せが既に DL を参照して
    ///   いる場合は項を除去して二重計上を防ぐ）。DL が無い場合は
    ///   「自重(自動)」自体を「DL」へ改名する。
    ///
    /// ケースの内容は改名/削除のみで書き換えない（自動生成ケースの内容は
    /// 解析実行前の同期アクションが毎回再計算して全置換する）。
    /// 削除時は `LoadCaseId` の「id == 添字」規約を保つよう後続ケースの ID と
    /// 組合せの参照を詰め直す。
    pub fn migrate_legacy_auto_load_cases(&mut self) {
        const LEGACY_SELF_WEIGHT: &str = "自重(自動)";
        let renames = [
            ("床荷重(自動)", DL_CASE_NAME),
            ("床積載(自動)", LL_FRAME_CASE_NAME),
            ("床地震用積載(自動)", LL_SEISMIC_CASE_NAME),
        ];
        for (old, new) in renames {
            if self.load_cases.iter().any(|lc| lc.name == new) {
                continue;
            }
            if let Some(lc) = self.load_cases.iter_mut().find(|lc| lc.name == old) {
                lc.name = new.to_string();
            }
        }

        let Some(sw_idx) = self
            .load_cases
            .iter()
            .position(|lc| lc.name == LEGACY_SELF_WEIGHT)
        else {
            return;
        };
        let sw_id = self.load_cases[sw_idx].id;
        match self
            .load_cases
            .iter()
            .find(|lc| lc.name == DL_CASE_NAME)
            .map(|lc| lc.id)
        {
            None => {
                // DL が無ければ「自重(自動)」を DL として引き継ぐ。
                self.load_cases[sw_idx].name = DL_CASE_NAME.to_string();
                self.load_cases[sw_idx].kind = LoadCaseKind::Dead;
            }
            Some(dl_id) => {
                // 組合せの参照を DL へ付け替え（既に DL を含む組合せでは項を除去）。
                for combo in &mut self.combinations {
                    let has_dl = combo.terms.iter().any(|(id, _)| *id == dl_id);
                    if has_dl {
                        combo.terms.retain(|(id, _)| *id != sw_id);
                    } else {
                        for (id, _) in &mut combo.terms {
                            if *id == sw_id {
                                *id = dl_id;
                            }
                        }
                    }
                }
                // ケースを削除し、id == 添字の規約を保つよう後続 ID を詰める。
                self.load_cases.remove(sw_idx);
                for lc in &mut self.load_cases {
                    if lc.id.0 > sw_id.0 {
                        lc.id.0 -= 1;
                    }
                }
                for combo in &mut self.combinations {
                    for (id, _) in &mut combo.terms {
                        if id.0 > sw_id.0 {
                            id.0 -= 1;
                        }
                    }
                }
            }
        }
    }
}
