//! 断面・材料および荷重（節点/部材荷重）の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 断面プロパティ変更（名称・A・Iy・Iz・J 等）。
pub struct SetSectionField {
    pub id: SectionId,
    pub field: SectionField,
    pub value: f64,
}

/// 編集対象の断面プロパティ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SectionField {
    Area,
    Iy,
    Iz,
    J,
    Depth,
    Width,
    AsY,
    AsZ,
}

impl EditCommand for SetSectionField {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let sec = &mut model.sections[idx];
        let old = match self.field {
            SectionField::Area => {
                let o = sec.area;
                sec.area = self.value;
                o
            }
            SectionField::Iy => {
                let o = sec.iy;
                sec.iy = self.value;
                o
            }
            SectionField::Iz => {
                let o = sec.iz;
                sec.iz = self.value;
                o
            }
            SectionField::J => {
                let o = sec.j;
                sec.j = self.value;
                o
            }
            SectionField::Depth => {
                let o = sec.depth;
                sec.depth = self.value;
                o
            }
            SectionField::Width => {
                let o = sec.width;
                sec.width = self.value;
                o
            }
            SectionField::AsY => {
                let o = sec.as_y;
                sec.as_y = self.value;
                o
            }
            SectionField::AsZ => {
                let o = sec.as_z;
                sec.as_z = self.value;
                o
            }
        };
        Box::new(SetSectionField {
            id: self.id,
            field: self.field,
            value: old,
        })
    }

    fn label(&self) -> &str {
        "断面プロパティ変更"
    }
}

/// 断面名称変更。
pub struct SetSectionName {
    pub id: SectionId,
    pub name: String,
}

impl EditCommand for SetSectionName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.sections[idx].name, self.name.clone());
        Box::new(SetSectionName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "断面名称変更"
    }
}

/// 部材の断面割当変更。
pub struct SetElementSection {
    pub elem: ElemId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetElementSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].section;
        model.elements[idx].section = self.section;
        Box::new(SetElementSection {
            elem: self.elem,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "部材断面割当変更"
    }
}

/// 部材の材料割当変更。
pub struct SetElementMaterial {
    pub elem: ElemId,
    pub material: Option<squid_n_core::ids::MaterialId>,
}

impl EditCommand for SetElementMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].material;
        model.elements[idx].material = self.material;
        Box::new(SetElementMaterial {
            elem: self.elem,
            material: old,
        })
    }

    fn label(&self) -> &str {
        "部材材料割当変更"
    }
}

/// 部材の履歴則（復元力特性）変更（各履歴則の原典による）。
/// `HysteresisModel::Auto` を指定すると個別指定を解除し既定へ戻す。
pub struct SetMemberHysteresis {
    pub elem: ElemId,
    pub rule: squid_n_core::model::HysteresisModel,
}

impl EditCommand for SetMemberHysteresis {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.set_member_hysteresis(self.elem, self.rule);
        Box::new(SetMemberHysteresis {
            elem: self.elem,
            rule: old.unwrap_or(squid_n_core::model::HysteresisModel::Auto),
        })
    }

    fn label(&self) -> &str {
        "部材履歴則変更"
    }
}

/// 制振ダンパーの特性（Kd・C0・α）変更（制振部材の力学モデル: Maxwell モデル等）。
/// `props=None` で指定を解除する。
pub struct SetDamperProps {
    pub elem: ElemId,
    pub props: Option<squid_n_core::model::DamperProps>,
}

impl EditCommand for SetDamperProps {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.set_damper_props(self.elem, self.props);
        Box::new(SetDamperProps {
            elem: self.elem,
            props: old,
        })
    }

    fn label(&self) -> &str {
        "制振ダンパー特性変更"
    }
}

/// 荷重ケース名変更。
pub struct SetLoadCaseName {
    pub id: LoadCaseId,
    pub name: String,
}

impl EditCommand for SetLoadCaseName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.load_cases[idx].name, self.name.clone());
        Box::new(SetLoadCaseName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース名変更"
    }
}

/// 節点荷重値変更（6成分）。
pub struct SetNodalLoad {
    pub lc: LoadCaseId,
    pub node: NodeId,
    pub values: [f64; 6],
}

impl EditCommand for SetNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let lc_idx = self.lc.index();
        if lc_idx >= model.load_cases.len() || model.load_cases[lc_idx].id != self.lc {
            return Box::new(Noop);
        }
        let nodal = &mut model.load_cases[lc_idx].nodal;
        if let Some(entry) = nodal.iter_mut().find(|n| n.node == self.node) {
            let old = entry.values;
            entry.values = self.values;
            Box::new(SetNodalLoad {
                lc: self.lc,
                node: self.node,
                values: old,
            })
        } else {
            nodal.push(squid_n_core::model::NodalLoad {
                node: self.node,
                values: self.values,
            });
            Box::new(DeleteNodalLoad {
                lc: self.lc,
                node: self.node,
            })
        }
    }

    fn label(&self) -> &str {
        "節点荷重変更"
    }
}

/// 節点荷重削除。
pub struct DeleteNodalLoad {
    pub lc: LoadCaseId,
    pub node: NodeId,
}

impl EditCommand for DeleteNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let lc_idx = self.lc.index();
        if lc_idx >= model.load_cases.len() || model.load_cases[lc_idx].id != self.lc {
            return Box::new(Noop);
        }
        let nodal = &mut model.load_cases[lc_idx].nodal;
        if let Some(pos) = nodal.iter().position(|n| n.node == self.node) {
            let removed = nodal.remove(pos);
            Box::new(SetNodalLoad {
                lc: self.lc,
                node: removed.node,
                values: removed.values,
            })
        } else {
            Box::new(Noop)
        }
    }

    fn label(&self) -> &str {
        "節点荷重削除"
    }
}

/// 部材（梁）荷重を荷重ケースへ追加。逆操作は末尾要素の削除。
pub struct AddMemberLoad {
    pub lc: LoadCaseId,
    pub load: squid_n_core::model::MemberLoad,
}

impl EditCommand for AddMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        model.load_cases[idx].member.push(self.load.clone());
        let pos = model.load_cases[idx].member.len() - 1;
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: pos,
        })
    }

    fn label(&self) -> &str {
        "部材荷重追加"
    }
}

/// 部材荷重を index 指定で削除。逆操作は同位置への挿入。
pub struct DeleteMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
}

impl EditCommand for DeleteMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        let member = &mut model.load_cases[idx].member;
        if self.index >= member.len() {
            return Box::new(Noop);
        }
        let removed = member.remove(self.index);
        Box::new(InsertMemberLoad {
            lc: self.lc,
            index: self.index,
            load: removed,
        })
    }

    fn label(&self) -> &str {
        "部材荷重削除"
    }
}

/// 部材荷重を index 位置へ挿入（DeleteMemberLoad の逆操作）。
pub struct InsertMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: squid_n_core::model::MemberLoad,
}

impl EditCommand for InsertMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        let member = &mut model.load_cases[idx].member;
        if self.index > member.len() {
            return Box::new(Noop);
        }
        member.insert(self.index, self.load.clone());
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: self.index,
        })
    }

    fn label(&self) -> &str {
        "部材荷重挿入"
    }
}

/// 断面形状を新規追加（UI-3 の新規断面作成）。
pub struct AddSectionShape {
    pub shape: squid_n_section::shape::SectionShape,
    pub new_id: SectionId,
    pub name: String,
}

impl EditCommand for AddSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let sec = self.shape.to_section(self.new_id, self.name.clone());
        model.sections.push(sec);
        Box::new(DeleteSection { id: self.new_id })
    }

    fn label(&self) -> &str {
        "断面形状追加"
    }
}

/// 断面形状変更。逆操作は RestoreSection。
pub struct EditSectionShape {
    pub section: SectionId,
    pub new_shape: squid_n_section::shape::SectionShape,
}

impl EditCommand for EditSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.section.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.section {
            return Box::new(Noop);
        }
        let old = model.sections[idx].clone();
        let new_sec = self.new_shape.to_section(self.section, old.name.clone());
        model.sections[idx] = new_sec;
        Box::new(RestoreSection { old })
    }

    fn label(&self) -> &str {
        "断面形状変更"
    }
}

/// 断面データを指定した Section で復元する（EditSectionShape の逆操作）。
pub struct RestoreSection {
    pub old: squid_n_core::model::Section,
}

impl EditCommand for RestoreSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.old.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.old.id {
            return Box::new(Noop);
        }
        let replaced = std::mem::replace(&mut model.sections[idx], self.old.clone());
        Box::new(RestoreSection { old: replaced })
    }

    fn label(&self) -> &str {
        "断面復元"
    }
}

/// 部材が参照する断面を複製し、部材に新断面を割り当てる。
pub struct DuplicateSectionForMember {
    pub member: ElemId,
}

impl EditCommand for DuplicateSectionForMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.member.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.member {
            return Box::new(Noop);
        }
        let sid = match model.elements[elem_idx].section {
            Some(s) => s,
            None => return Box::new(Noop),
        };
        let sec_idx = sid.index();
        if sec_idx >= model.sections.len() || model.sections[sec_idx].id != sid {
            return Box::new(Noop);
        }
        let orig = &model.sections[sec_idx];
        let new_id = SectionId(model.sections.len() as u32);
        let mut new_sec = orig.clone();
        new_sec.id = new_id;
        new_sec.name = format!("{}(複製)", orig.name);
        model.sections.push(new_sec);
        model.elements[elem_idx].section = Some(new_id);
        Box::new(RestoreElementSectionAndDeleteSection {
            elem: self.member,
            old_section: Some(sid),
            new_section: new_id,
        })
    }

    fn label(&self) -> &str {
        "部材断面複製"
    }
}

/// DuplicateSectionForMember の逆操作。
pub struct RestoreElementSectionAndDeleteSection {
    pub elem: ElemId,
    pub old_section: Option<SectionId>,
    pub new_section: SectionId,
}

impl EditCommand for RestoreElementSectionAndDeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.elem.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.elem {
            return Box::new(Noop);
        }
        let new_idx = self.new_section.index();
        if new_idx >= model.sections.len() || model.sections[new_idx].id != self.new_section {
            return Box::new(Noop);
        }
        model.elements[elem_idx].section = self.old_section;
        model.sections.remove(new_idx);
        let removed_id = self.new_section;
        shift_section_ids(model, |sid| {
            if sid.0 > removed_id.0 {
                sid.0 -= 1;
            }
        });
        Box::new(DuplicateSectionForMember { member: self.elem })
    }

    fn label(&self) -> &str {
        "部材断面複製解除"
    }
}

/// 断面削除。逆操作は AddSection。
///
/// 部材から参照中の断面は削除すると参照が壊れるため Noop とする
/// （先に割当を解除するか、UI 側でボタンを無効化する）。
/// ID＝配列インデックスの不変条件を保つため、削除後は後続の断面 ID と
/// 部材からの参照を 1 つずつ繰り上げる。
pub struct DeleteSection {
    pub id: SectionId,
}

impl EditCommand for DeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        if model.elements.iter().any(|e| e.section == Some(self.id)) {
            return Box::new(Noop);
        }
        let removed = model.sections.remove(idx);
        shift_section_ids(model, |sid| {
            if sid.0 > self.id.0 {
                sid.0 -= 1;
            }
        });
        Box::new(AddSection {
            old: removed,
            index: idx,
        })
    }

    fn label(&self) -> &str {
        "断面削除"
    }
}

/// 断面追加（DeleteSection の逆操作）。後続の断面 ID・参照を 1 つ繰り下げてから挿入する。
pub struct AddSection {
    pub old: squid_n_core::model::Section,
    pub index: usize,
}

impl EditCommand for AddSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.sections.len() {
            return Box::new(Noop);
        }
        let id = SectionId(self.index as u32);
        shift_section_ids(model, |sid| {
            if sid.0 >= id.0 {
                sid.0 += 1;
            }
        });
        let mut sec = self.old.clone();
        sec.id = id;
        model.sections.insert(self.index, sec);
        Box::new(DeleteSection { id })
    }

    fn label(&self) -> &str {
        "断面追加"
    }
}

/// モデル内の全ての `SectionId` 参照（断面自身の ID を含む）に `f` を適用する。
fn shift_section_ids(model: &mut Model, mut f: impl FnMut(&mut SectionId)) {
    for sec in &mut model.sections {
        f(&mut sec.id);
    }
    for elem in &mut model.elements {
        if let Some(sid) = &mut elem.section {
            f(sid);
        }
    }
}

/// 材料追加。末尾に `MaterialId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は材料削除。
pub struct AddMaterial {
    pub name: String,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    pub fc: Option<f64>,
    pub fy: Option<f64>,
}

impl EditCommand for AddMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = MaterialId(model.materials.len() as u32);
        model.materials.push(squid_n_core::model::Material {
            concrete_class: Default::default(),
            id: new_id,
            name: self.name.clone(),
            young: self.young,
            poisson: self.poisson,
            density: self.density,
            shear: None,
            fc: self.fc,
            fy: self.fy,
        });
        Box::new(DeleteMaterial { id: new_id })
    }

    fn label(&self) -> &str {
        "材料追加"
    }
}

/// 材料削除。部材から参照中の材料は Noop。逆操作は [`InsertMaterial`]。
/// ID＝配列インデックスの不変条件を保つため、後続の材料 ID と部材からの参照を繰り上げる。
pub struct DeleteMaterial {
    pub id: MaterialId,
}

impl EditCommand for DeleteMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        if model.elements.iter().any(|e| e.material == Some(self.id)) {
            return Box::new(Noop);
        }
        let removed = model.materials.remove(idx);
        shift_material_ids(model, |mid| {
            if mid.0 > self.id.0 {
                mid.0 -= 1;
            }
        });
        Box::new(InsertMaterial {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "材料削除"
    }
}

/// 指定インデックスへ材料を再挿入する（[`DeleteMaterial`] の逆操作専用）。
pub struct InsertMaterial {
    pub index: usize,
    pub old: squid_n_core::model::Material,
}

impl EditCommand for InsertMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.materials.len() {
            return Box::new(Noop);
        }
        let id = MaterialId(self.index as u32);
        shift_material_ids(model, |mid| {
            if mid.0 >= id.0 {
                mid.0 += 1;
            }
        });
        let mut mat = self.old.clone();
        mat.id = id;
        model.materials.insert(self.index, mat);
        Box::new(DeleteMaterial { id })
    }

    fn label(&self) -> &str {
        "材料削除の取り消し"
    }
}

/// モデル内の全ての `MaterialId` 参照（材料自身の ID を含む）に `f` を適用する。
fn shift_material_ids(model: &mut Model, mut f: impl FnMut(&mut MaterialId)) {
    for mat in &mut model.materials {
        f(&mut mat.id);
    }
    for elem in &mut model.elements {
        if let Some(mid) = &mut elem.material {
            f(mid);
        }
    }
}

/// 編集対象の材料プロパティ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MaterialField {
    Young,
    Poisson,
    Density,
    Fc,
    Fy,
}

/// 材料プロパティ変更（E・ポアソン比・密度・Fc・Fy）。
pub struct SetMaterialField {
    pub id: MaterialId,
    pub field: MaterialField,
    pub value: Option<f64>,
}

impl EditCommand for SetMaterialField {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        let mat = &mut model.materials[idx];
        let old = match self.field {
            MaterialField::Young => {
                let old = Some(mat.young);
                mat.young = self.value.unwrap_or(mat.young);
                old
            }
            MaterialField::Poisson => {
                let old = Some(mat.poisson);
                mat.poisson = self.value.unwrap_or(mat.poisson);
                old
            }
            MaterialField::Density => {
                let old = Some(mat.density);
                mat.density = self.value.unwrap_or(mat.density);
                old
            }
            MaterialField::Fc => std::mem::replace(&mut mat.fc, self.value),
            MaterialField::Fy => std::mem::replace(&mut mat.fy, self.value),
        };
        Box::new(SetMaterialField {
            id: self.id,
            field: self.field,
            value: old,
        })
    }

    fn label(&self) -> &str {
        "材料プロパティ変更"
    }
}

/// 材料名変更。
pub struct SetMaterialName {
    pub id: MaterialId,
    pub name: String,
}

impl EditCommand for SetMaterialName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.materials[idx].name, self.name.clone());
        Box::new(SetMaterialName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "材料名変更"
    }
}
