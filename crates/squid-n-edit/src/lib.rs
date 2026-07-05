use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, SlabId, StoryId};
use squid_n_core::model::Model;

/// 編集コマンド。`Send` を要求するのは、MCP サーバ(P8)が `UndoStack` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
/// コマンドはモデルデータの断片のみを保持するプレーンな構造体であり、
/// 全実装が自然に `Send` を満たす。
pub trait EditCommand: Send {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand>;
    fn label(&self) -> &str;
}

impl<T: EditCommand + 'static> From<T> for Box<dyn EditCommand> {
    fn from(cmd: T) -> Self {
        Box::new(cmd)
    }
}

pub struct UndoStack {
    done: Vec<Box<dyn EditCommand>>,
    undone: Vec<Box<dyn EditCommand>>,
    max_undo: usize,
}

impl UndoStack {
    pub fn new() -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo: 100,
        }
    }

    pub fn with_max(max_undo: usize) -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo,
        }
    }

    pub fn run(&mut self, model: &mut Model, cmd: Box<dyn EditCommand>) {
        let inv = cmd.apply(model);
        self.done.push(inv);
        if self.done.len() > self.max_undo {
            self.done.remove(0);
        }
        self.undone.clear();
    }

    pub fn undo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.done.pop() {
            let redo_cmd = cmd.apply(model);
            self.undone.push(redo_cmd);
        }
    }

    pub fn redo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.undone.pop() {
            let undo_cmd = cmd.apply(model);
            self.done.push(undo_cmd);
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.done.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.undone.is_empty()
    }

    pub fn undo_label(&self) -> Option<&str> {
        self.done.last().map(|c| c.label())
    }

    pub fn redo_label(&self) -> Option<&str> {
        self.undone.last().map(|c| c.label())
    }
}

impl Default for UndoStack {
    fn default() -> Self {
        Self::new()
    }
}

pub fn push_edit_command(model: &mut Model, stack: &mut UndoStack, cmd: Box<dyn EditCommand>) {
    stack.run(model, cmd);
}

/// 節点座標変更。
pub struct SetNodeCoord {
    pub node: NodeId,
    pub coord: [f64; 3],
}

impl EditCommand for SetNodeCoord {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old_coord = model.nodes[idx].coord;
        model.nodes[idx].coord = self.coord;
        Box::new(SetNodeCoord {
            node: self.node,
            coord: old_coord,
        })
    }

    fn label(&self) -> &str {
        "節点座標変更"
    }
}

/// 節点拘束（支点条件）変更。逆操作は変更前マスクへの復元。
pub struct SetNodeRestraint {
    pub node: NodeId,
    pub restraint: squid_n_core::dof::Dof6Mask,
}

impl EditCommand for SetNodeRestraint {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old = model.nodes[idx].restraint;
        model.nodes[idx].restraint = self.restraint;
        Box::new(SetNodeRestraint {
            node: self.node,
            restraint: old,
        })
    }

    fn label(&self) -> &str {
        "節点拘束変更"
    }
}

/// 節点追加。末尾に `NodeId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は節点削除。
pub struct AddNode {
    pub coord: [f64; 3],
    pub restraint: squid_n_core::dof::Dof6Mask,
}

impl EditCommand for AddNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = NodeId(model.nodes.len() as u32);
        model.nodes.push(squid_n_core::model::Node {
            id: new_id,
            coord: self.coord,
            restraint: self.restraint,
            mass: None,
            story: None,
        });
        Box::new(DeleteNode { id: new_id })
    }

    fn label(&self) -> &str {
        "節点追加"
    }
}

/// 節点削除（末尾以外の中間節点も可）。逆操作は [`InsertNode`]（元の位置に再挿入し、
/// 繰り上がった ID・参照を元に戻す）。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該節点より後ろの
/// 節点 ID と、それを参照する全ての箇所（部材・節点荷重・階・床・拘束）を
/// 1 つずつ繰り上げる。部材などからまだ参照されている節点は削除すると
/// 参照が壊れるため Noop とする（先に参照を解消する必要がある）。
pub struct DeleteNode {
    pub id: NodeId,
}

impl EditCommand for DeleteNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.id {
            return Box::new(Noop);
        }
        if model.node_in_use(self.id) {
            return Box::new(Noop);
        }
        let removed = model.nodes.remove(idx);
        shift_node_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertNode {
            index: idx,
            coord: removed.coord,
            restraint: removed.restraint,
            mass: removed.mass,
            story: removed.story,
        })
    }

    fn label(&self) -> &str {
        "節点削除"
    }
}

/// 指定インデックスへ節点を再挿入し、以降の節点 ID・参照を 1 つ繰り下げる
/// （[`DeleteNode`] の逆操作専用。新規追加は [`AddNode`] を使うこと）。
pub struct InsertNode {
    pub index: usize,
    pub coord: [f64; 3],
    pub restraint: squid_n_core::dof::Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<squid_n_core::ids::StoryId>,
}

impl EditCommand for InsertNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let id = NodeId(self.index as u32);
        shift_node_ids(model, |nid| {
            if nid.0 >= id.0 {
                nid.0 += 1;
            }
        });
        model.nodes.insert(
            self.index,
            squid_n_core::model::Node {
                id,
                coord: self.coord,
                restraint: self.restraint,
                mass: self.mass,
                story: self.story,
            },
        );
        Box::new(DeleteNode { id })
    }

    fn label(&self) -> &str {
        "節点削除の取り消し"
    }
}

/// モデル内の全ての `NodeId` 参照（節点自身の ID を含む）に `f` を適用する。
/// [`DeleteNode`]／[`InsertNode`] の ID 繰り上げ・繰り下げで共用する。
fn shift_node_ids(model: &mut Model, mut f: impl FnMut(&mut NodeId)) {
    for node in &mut model.nodes {
        f(&mut node.id);
    }
    for elem in &mut model.elements {
        for n in &mut elem.nodes {
            f(n);
        }
    }
    for story in &mut model.stories {
        for n in &mut story.node_ids {
            f(n);
        }
        for d in &mut story.diaphragms {
            f(&mut d.master);
            for s in &mut d.slaves {
                f(s);
            }
        }
    }
    for slab in &mut model.slabs {
        for n in &mut slab.boundary {
            f(n);
        }
        for j in &mut slab.joists {
            for n in &mut j.support {
                f(n);
            }
        }
    }
    for c in &mut model.constraints {
        use squid_n_core::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, slaves, .. } => {
                f(master);
                for s in slaves {
                    f(s);
                }
            }
            Constraint::Mpc { master, terms } => {
                f(master);
                for (n, _, _) in terms {
                    f(n);
                }
            }
            Constraint::RigidLink { master, slaves, .. } => {
                f(master);
                for s in slaves {
                    f(s);
                }
            }
        }
    }
    for lc in &mut model.load_cases {
        for nl in &mut lc.nodal {
            f(&mut nl.node);
        }
    }
}

/// 部材追加。逆操作は部材削除。
pub struct AddMember {
    pub elem: squid_n_core::model::ElementData,
}

impl EditCommand for AddMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.elements.push(self.elem.clone());
        Box::new(DeleteMember { id: self.elem.id })
    }

    fn label(&self) -> &str {
        "部材追加"
    }
}

/// 部材削除（中間の部材も可）。逆操作は [`InsertMember`]。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該部材より後ろの
/// 部材 ID と、それを参照する部材荷重の `elem` を 1 つずつ繰り上げる。
/// 当該部材を参照する部材荷重は連動して削除し、undo で復元する。
pub struct DeleteMember {
    pub id: ElemId,
}

impl EditCommand for DeleteMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.id {
            return Box::new(Noop);
        }
        // 当該部材を参照する部材荷重を (荷重ケース index, 荷重 index, 内容) で退避してから削除
        let mut removed_loads = Vec::new();
        for (lci, lc) in model.load_cases.iter_mut().enumerate() {
            let mut li = 0;
            while li < lc.member.len() {
                if lc.member[li].elem == self.id {
                    removed_loads.push((lci, li, lc.member.remove(li)));
                } else {
                    li += 1;
                }
            }
        }
        let removed = model.elements.remove(idx);
        shift_elem_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertMember {
            index: idx,
            elem: removed,
            member_loads: removed_loads,
        })
    }

    fn label(&self) -> &str {
        "部材削除"
    }
}

/// 指定インデックスへ部材を再挿入し、以降の部材 ID・参照を 1 つ繰り下げ、
/// 連動削除された部材荷重を復元する（[`DeleteMember`] の逆操作専用）。
pub struct InsertMember {
    pub index: usize,
    pub elem: squid_n_core::model::ElementData,
    /// (荷重ケース index, 荷重 index, 内容)
    pub member_loads: Vec<(usize, usize, squid_n_core::model::MemberLoad)>,
}

impl EditCommand for InsertMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.elements.len() {
            return Box::new(Noop);
        }
        let id = ElemId(self.index as u32);
        shift_elem_ids(model, |eid| {
            if eid.0 >= id.0 {
                eid.0 += 1;
            }
        });
        let mut elem = self.elem.clone();
        elem.id = id;
        model.elements.insert(self.index, elem);
        for (lci, li, load) in &self.member_loads {
            if let Some(lc) = model.load_cases.get_mut(*lci) {
                let pos = (*li).min(lc.member.len());
                lc.member.insert(pos, load.clone());
            }
        }
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "部材削除の取り消し"
    }
}

/// モデル内の全ての `ElemId` 参照（部材自身の ID を含む）に `f` を適用する。
fn shift_elem_ids(model: &mut Model, mut f: impl FnMut(&mut ElemId)) {
    for elem in &mut model.elements {
        f(&mut elem.id);
    }
    for lc in &mut model.load_cases {
        for ml in &mut lc.member {
            f(&mut ml.elem);
        }
    }
}

/// 何もしないコマンド（参照不正時の安全なフォールバック）。
pub struct Noop;

impl EditCommand for Noop {
    fn apply(&self, _model: &mut Model) -> Box<dyn EditCommand> {
        Box::new(Noop)
    }

    fn label(&self) -> &str {
        "Noop"
    }
}

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

/// 荷重ケース追加。末尾に `LoadCaseId(len)` で追加する。逆操作は荷重ケース削除。
pub struct AddLoadCase {
    pub name: String,
}

impl EditCommand for AddLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = LoadCaseId(model.load_cases.len() as u32);
        model.load_cases.push(squid_n_core::model::LoadCase {
            id: new_id,
            name: self.name.clone(),
            nodal: Vec::new(),
            member: Vec::new(),
        });
        Box::new(DeleteLoadCase { id: new_id })
    }

    fn label(&self) -> &str {
        "荷重ケース追加"
    }
}

/// 荷重ケース削除（中身の節点荷重・部材荷重ごと削除し、undo で復元する）。
/// 荷重組合せから参照中のケースは Noop。
/// ID＝配列インデックスの不変条件を保つため、後続のケース ID と組合せからの参照を繰り上げる。
pub struct DeleteLoadCase {
    pub id: LoadCaseId,
}

impl EditCommand for DeleteLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        if model
            .combinations
            .iter()
            .any(|c| c.terms.iter().any(|(lc, _)| *lc == self.id))
        {
            return Box::new(Noop);
        }
        let removed = model.load_cases.remove(idx);
        shift_load_case_ids(model, |lcid| {
            if lcid.0 > self.id.0 {
                lcid.0 -= 1;
            }
        });
        Box::new(InsertLoadCase {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース削除"
    }
}

/// 指定インデックスへ荷重ケースを再挿入する（[`DeleteLoadCase`] の逆操作専用）。
pub struct InsertLoadCase {
    pub index: usize,
    pub old: squid_n_core::model::LoadCase,
}

impl EditCommand for InsertLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.load_cases.len() {
            return Box::new(Noop);
        }
        let id = LoadCaseId(self.index as u32);
        shift_load_case_ids(model, |lcid| {
            if lcid.0 >= id.0 {
                lcid.0 += 1;
            }
        });
        let mut lc = self.old.clone();
        lc.id = id;
        model.load_cases.insert(self.index, lc);
        Box::new(DeleteLoadCase { id })
    }

    fn label(&self) -> &str {
        "荷重ケース削除の取り消し"
    }
}

/// モデル内の全ての `LoadCaseId` 参照（ケース自身の ID を含む）に `f` を適用する。
fn shift_load_case_ids(model: &mut Model, mut f: impl FnMut(&mut LoadCaseId)) {
    for lc in &mut model.load_cases {
        f(&mut lc.id);
    }
    for combo in &mut model.combinations {
        for (lcid, _) in &mut combo.terms {
            f(lcid);
        }
    }
}

/// 荷重組合せ追加。末尾に追加する。逆操作は末尾の組合せ削除。
///
/// `LoadCombination` は ID を持たず配列インデックスのみで管理されるため、
/// 他の追加系コマンド（[`AddLoadCase`] 等）と異なり ID 採番は発生しない。
/// 参照する `LoadCaseId` の存在チェックは行わない（[`Model::validate`] も
/// 組合せの `LoadCaseId` 参照はダングリングチェックの対象外であり、既存の
/// [`DeleteLoadCase`] が参照側で削除を防ぐことで整合性を保っている）。
pub struct AddCombination {
    pub combo: squid_n_core::model::LoadCombination,
}

impl EditCommand for AddCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.combinations.push(self.combo.clone());
        let index = model.combinations.len() - 1;
        Box::new(DeleteCombination { index })
    }

    fn label(&self) -> &str {
        "荷重組合せ追加"
    }
}

/// 荷重組合せを index 指定で削除。逆操作は [`InsertCombination`]（同じ位置への復元）。
/// 組合せは他のデータから参照されないため ID 再採番は不要。index が範囲外なら Noop。
pub struct DeleteCombination {
    pub index: usize,
}

impl EditCommand for DeleteCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.combinations.len() {
            return Box::new(Noop);
        }
        let removed = model.combinations.remove(self.index);
        Box::new(InsertCombination {
            index: self.index,
            combo: removed,
        })
    }

    fn label(&self) -> &str {
        "荷重組合せ削除"
    }
}

/// 指定インデックスへ荷重組合せを再挿入する（[`DeleteCombination`] の逆操作専用）。
pub struct InsertCombination {
    pub index: usize,
    pub combo: squid_n_core::model::LoadCombination,
}

impl EditCommand for InsertCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.combinations.len() {
            return Box::new(Noop);
        }
        model.combinations.insert(self.index, self.combo.clone());
        Box::new(DeleteCombination { index: self.index })
    }

    fn label(&self) -> &str {
        "荷重組合せ削除の取り消し"
    }
}

/// 階定義の一括適用（階自動生成の結果を反映する）。
///
/// `model.stories`・各節点の所属階・剛床拘束(`Constraint::RigidDiaphragm`)を
/// まとめて差し替える。既存の RigidDiaphragm 拘束は除去し、Mpc / RigidLink は
/// 保持する。逆操作は差し替え前の状態の復元。
pub struct ApplyStories {
    pub stories: Vec<squid_n_core::model::Story>,
    /// `model.nodes` と同順の所属階。長さが合わない分は無視する。
    pub node_story: Vec<Option<squid_n_core::ids::StoryId>>,
    /// 追加する剛床拘束（既存の RigidDiaphragm と置換）。
    pub constraints: Vec<squid_n_core::model::Constraint>,
}

impl EditCommand for ApplyStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::Constraint;
        let old_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        let old_node_story: Vec<Option<squid_n_core::ids::StoryId>> =
            model.nodes.iter().map(|n| n.story).collect();
        for (node, st) in model.nodes.iter_mut().zip(self.node_story.iter()) {
            node.story = *st;
        }
        // RigidDiaphragm のみ差し替え、それ以外の拘束は保持
        let old_constraints = model.constraints.clone();
        model
            .constraints
            .retain(|c| !matches!(c, Constraint::RigidDiaphragm { .. }));
        model.constraints.extend(self.constraints.iter().cloned());
        Box::new(RestoreStories {
            stories: old_stories,
            node_story: old_node_story,
            constraints: old_constraints,
        })
    }

    fn label(&self) -> &str {
        "階定義の適用"
    }
}

/// [`ApplyStories`] の逆操作。拘束リストを丸ごと復元する。
pub struct RestoreStories {
    pub stories: Vec<squid_n_core::model::Story>,
    pub node_story: Vec<Option<squid_n_core::ids::StoryId>>,
    pub constraints: Vec<squid_n_core::model::Constraint>,
}

impl EditCommand for RestoreStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        let new_node_story: Vec<Option<squid_n_core::ids::StoryId>> =
            model.nodes.iter().map(|n| n.story).collect();
        for (node, st) in model.nodes.iter_mut().zip(self.node_story.iter()) {
            node.story = *st;
        }
        let new_constraints = std::mem::replace(&mut model.constraints, self.constraints.clone());
        Box::new(RestoreStories {
            stories: new_stories,
            node_story: new_node_story,
            constraints: new_constraints,
        })
    }

    fn label(&self) -> &str {
        "階定義の復元"
    }
}

/// 床追加。末尾に `SlabId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は床削除。
pub struct AddSlab {
    pub boundary: Vec<NodeId>,
    pub joists: Vec<squid_n_core::model::JoistLine>,
    pub loads: Vec<squid_n_core::model::AreaLoad>,
    pub method: squid_n_core::model::DistributionMethod,
}

impl EditCommand for AddSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = SlabId(model.slabs.len() as u32);
        model.slabs.push(squid_n_core::model::Slab {
            id: new_id,
            boundary: self.boundary.clone(),
            joists: self.joists.clone(),
            loads: self.loads.clone(),
            method: self.method,
        });
        Box::new(DeleteSlab { id: new_id })
    }

    fn label(&self) -> &str {
        "床追加"
    }
}

/// 床削除（中間の床も可）。逆操作は [`InsertSlab`]。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該床より後ろの
/// 床 ID を 1 つずつ繰り上げる。`SlabId` は床自身の ID 以外からは参照されない
/// （`crates` 全体で grep 済み）ため、他データへの追従は不要。
pub struct DeleteSlab {
    pub id: SlabId,
}

impl EditCommand for DeleteSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let removed = model.slabs.remove(idx);
        shift_slab_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertSlab {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "床削除"
    }
}

/// 指定インデックスへ床を再挿入する（[`DeleteSlab`] の逆操作専用）。
pub struct InsertSlab {
    pub index: usize,
    pub old: squid_n_core::model::Slab,
}

impl EditCommand for InsertSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.slabs.len() {
            return Box::new(Noop);
        }
        let id = SlabId(self.index as u32);
        shift_slab_ids(model, |sid| {
            if sid.0 >= id.0 {
                sid.0 += 1;
            }
        });
        let mut slab = self.old.clone();
        slab.id = id;
        model.slabs.insert(self.index, slab);
        Box::new(DeleteSlab { id })
    }

    fn label(&self) -> &str {
        "床削除の取り消し"
    }
}

/// モデル内の全ての `SlabId` 参照（床自身の ID を含む）に `f` を適用する。
fn shift_slab_ids(model: &mut Model, mut f: impl FnMut(&mut SlabId)) {
    for slab in &mut model.slabs {
        f(&mut slab.id);
    }
}

/// 階の地震重量（`seismic_weight`）変更。逆操作は変更前の値への復元。
/// 存在しない `StoryId` は Noop。
pub struct SetStoryWeight {
    pub story: StoryId,
    pub weight: Option<f64>,
}

impl EditCommand for SetStoryWeight {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.story.index();
        if idx >= model.stories.len() || model.stories[idx].id != self.story {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.stories[idx].seismic_weight, self.weight);
        Box::new(SetStoryWeight {
            story: self.story,
            weight: old,
        })
    }

    fn label(&self) -> &str {
        "階地震重量変更"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    fn empty_model() -> Model {
        Model::default()
    }

    #[test]
    fn test_set_node_coord_roundtrip() {
        let mut model = empty_model();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let mut stack = UndoStack::new();

        let cmd = SetNodeCoord {
            node: NodeId(0),
            coord: [1000.0, 2000.0, 0.0],
        };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);

        stack.undo(&mut model);
        assert_eq!(model.nodes[0].coord, [0.0, 0.0, 0.0]);

        stack.redo(&mut model);
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);
    }

    #[test]
    fn test_set_node_coord_invalid_id_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(SetNodeCoord {
                node: NodeId(99),
                coord: [1.0, 2.0, 3.0],
            }),
        );
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.nodes.is_empty());
    }

    #[test]
    fn test_set_node_restraint_roundtrip() {
        let mut model = empty_model();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(SetNodeRestraint {
                node: NodeId(0),
                restraint: Dof6Mask::PINNED,
            }),
        );
        assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);

        stack.undo(&mut model);
        assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);

        stack.redo(&mut model);
        assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);
    }

    #[test]
    fn test_set_node_restraint_invalid_id_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(SetNodeRestraint {
                node: NodeId(99),
                restraint: Dof6Mask::FIXED,
            }),
        );
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.nodes.is_empty());
    }

    #[test]
    fn test_add_node_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(AddNode {
                coord: [1000.0, 2000.0, 3000.0],
                restraint: Dof6Mask::FREE,
            }),
        );
        assert_eq!(model.nodes.len(), 1);
        assert_eq!(model.nodes[0].id, NodeId(0));
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 3000.0]);

        stack.undo(&mut model);
        assert_eq!(model.nodes.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.nodes.len(), 1);
        assert_eq!(model.nodes[0].id, NodeId(0));
    }

    #[test]
    fn test_add_node_id_equals_index() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        for i in 0..3 {
            stack.run(
                &mut model,
                Box::new(AddNode {
                    coord: [i as f64, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                }),
            );
        }
        for (i, node) in model.nodes.iter().enumerate() {
            assert_eq!(node.id, NodeId(i as u32));
        }
    }

    #[test]
    fn test_delete_node_middle_renumbers_and_roundtrips() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        for i in 0..3 {
            model.nodes.push(Node {
                id: NodeId(i),
                coord: [i as f64, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            });
        }
        // 末尾の節点（N2）を使う部材を用意し、中間節点（N1）削除後に
        // 参照が N1 へ繰り上がることを確認する。
        model.elements.push(ElementData {
            id: squid_n_core::ids::ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(2)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        stack.run(&mut model, Box::new(DeleteNode { id: NodeId(1) }));
        assert_eq!(model.nodes.len(), 2);
        assert_eq!(model.nodes[0].id, NodeId(0));
        assert_eq!(model.nodes[1].id, NodeId(1));
        assert_eq!(model.nodes[1].coord, [2.0, 0.0, 0.0]);
        // 元 N2 だった部材参照は N1 に繰り上がる
        assert_eq!(model.elements[0].nodes[1], NodeId(1));
        assert!(model.validate().is_ok());

        stack.undo(&mut model);
        assert_eq!(model.nodes.len(), 3);
        for (i, node) in model.nodes.iter().enumerate() {
            assert_eq!(node.id, NodeId(i as u32));
            assert_eq!(node.coord, [i as f64, 0.0, 0.0]);
        }
        assert_eq!(model.elements[0].nodes.to_vec(), vec![NodeId(0), NodeId(2)]);
        assert!(model.validate().is_ok());

        stack.redo(&mut model);
        assert_eq!(model.nodes.len(), 2);
        assert_eq!(model.elements[0].nodes[1], NodeId(1));
        assert!(model.validate().is_ok());
    }

    #[test]
    fn test_delete_node_in_use_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.elements.push(ElementData {
            id: squid_n_core::ids::ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        // 部材に使われている節点の削除は Noop（先に部材を削除する必要がある）
        stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
        assert_eq!(model.nodes.len(), 2);
        assert!(model.validate().is_ok());
    }

    #[test]
    fn test_add_delete_member_load_roundtrip() {
        use squid_n_core::ids::LoadCaseId;
        use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
        let mut model = empty_model();
        model.load_cases.push(LoadCase {
            id: LoadCaseId(0),
            name: "lc".into(),
            nodal: vec![],
            member: vec![],
        });
        let mut stack = UndoStack::new();
        let load = MemberLoad {
            elem: squid_n_core::ids::ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 1000.0,
                w1: 2.0,
                w2: 2.0,
            },
        };
        stack.run(
            &mut model,
            Box::new(AddMemberLoad {
                lc: LoadCaseId(0),
                load: load.clone(),
            }),
        );
        assert_eq!(model.load_cases[0].member.len(), 1);
        assert_eq!(model.load_cases[0].member[0], load);

        stack.undo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 1);

        // 削除と復元（位置保持）
        stack.run(
            &mut model,
            Box::new(DeleteMemberLoad {
                lc: LoadCaseId(0),
                index: 0,
            }),
        );
        assert_eq!(model.load_cases[0].member.len(), 0);
        stack.undo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 1);
        assert_eq!(model.load_cases[0].member[0], load);
    }

    #[test]
    fn test_add_delete_member_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        let elem = ElementData {
            id: squid_n_core::ids::ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        stack.run(&mut model, Box::new(AddMember { elem }));
        assert_eq!(model.elements.len(), 1);
        stack.undo(&mut model);
        assert_eq!(model.elements.len(), 0);
        stack.redo(&mut model);
        assert_eq!(model.elements.len(), 1);
    }

    #[test]
    fn test_add_section_shape_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        let shape = squid_n_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let cmd = AddSectionShape {
            shape,
            new_id: SectionId(0),
            name: "H-300x300x10x15".into(),
        };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));
    }

    #[test]
    fn test_edit_section_shape_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape1 = squid_n_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape1.to_section(SectionId(0), "H-300".into());
        let area_h = sec.area;
        model.sections.push(sec);

        let shape2 = squid_n_section::shape::SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let cmd = EditSectionShape {
            section: SectionId(0),
            new_shape: shape2,
        };
        stack.run(&mut model, Box::new(cmd));
        assert!((model.sections[0].area - 9024.0).abs() < 1.0);

        stack.undo(&mut model);
        assert!((model.sections[0].area - area_h).abs() < 1.0);

        stack.redo(&mut model);
        assert!((model.sections[0].area - 9024.0).abs() < 1.0);
    }

    #[test]
    fn test_duplicate_section_for_member_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = squid_n_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300".into());
        model.sections.push(sec);

        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        let cmd = DuplicateSectionForMember { member: ElemId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 2);
        assert_eq!(model.elements[0].section, Some(SectionId(1)));

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.elements[0].section, Some(SectionId(0)));

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 2);
        assert_eq!(model.elements[0].section, Some(SectionId(1)));
    }

    #[test]
    fn test_edit_section_shape_invalid_id_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = squid_n_section::shape::SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let cmd = EditSectionShape {
            section: SectionId(99),
            new_shape: shape,
        };
        stack.run(&mut model, Box::new(cmd));
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.sections.is_empty());
    }

    #[test]
    fn test_delete_add_section_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = squid_n_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300".into());
        model.sections.push(sec);

        let cmd = DeleteSection { id: SectionId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 0);

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 0);
    }

    #[test]
    fn test_duplicate_section_no_section_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        let cmd = DuplicateSectionForMember { member: ElemId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert!(stack.can_undo());
        stack.undo(&mut model);
    }

    /// 2 節点 + 部材 2 本のモデル（部材削除・再採番テスト用）。
    fn two_member_model() -> Model {
        let mut model = empty_model();
        for (i, x) in [0.0, 1000.0, 2000.0].iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [*x, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            });
        }
        for i in 0..2u32 {
            model.elements.push(ElementData {
                id: ElemId(i),
                kind: ElementKind::Beam,
                nodes: smallvec![NodeId(i), NodeId(i + 1)],
                section: None,
                material: None,
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            });
        }
        model
    }

    #[test]
    fn test_delete_member_middle_renumbers_and_roundtrips() {
        use squid_n_core::ids::LoadCaseId;
        use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
        let mut model = two_member_model();
        // 両方の部材に部材荷重を付ける
        model.load_cases.push(LoadCase {
            id: LoadCaseId(0),
            name: "lc".into(),
            nodal: vec![],
            member: vec![
                MemberLoad {
                    elem: ElemId(0),
                    dir: [0.0, 0.0, -1.0],
                    kind: MemberLoadKind::Point { a: 500.0, p: 1.0 },
                },
                MemberLoad {
                    elem: ElemId(1),
                    dir: [0.0, 0.0, -1.0],
                    kind: MemberLoadKind::Point { a: 500.0, p: 2.0 },
                },
            ],
        });
        let before = model.clone();
        let mut stack = UndoStack::new();

        // 先頭（中間）の部材を削除 → 後続 ID が繰り上がり、関連荷重も消える
        stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
        assert_eq!(model.elements.len(), 1);
        assert_eq!(model.elements[0].id, ElemId(0));
        assert!(model.validate().is_ok());
        assert_eq!(model.load_cases[0].member.len(), 1);
        // 残った荷重は旧 ElemId(1) → 新 ElemId(0) を指す
        assert_eq!(model.load_cases[0].member[0].elem, ElemId(0));

        // undo で完全復元
        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
        assert!(model.validate().is_ok());
    }

    #[test]
    fn test_delete_section_in_use_is_noop_and_renumbers() {
        use squid_n_core::model::Section;
        let mut model = two_member_model();
        for i in 0..2u32 {
            model.sections.push(Section {
                id: SectionId(i),
                name: format!("S{}", i),
                area: 100.0,
                iy: 1.0,
                iz: 1.0,
                j: 1.0,
                depth: 10.0,
                width: 10.0,
                as_y: 80.0,
                as_z: 80.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            });
        }
        // 部材 0 に断面 1 を割当（断面 0 は未使用）
        model.elements[0].section = Some(SectionId(1));
        let mut stack = UndoStack::new();

        // 使用中の断面 1 は削除できない
        stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) }));
        assert_eq!(model.sections.len(), 2);

        // 未使用の断面 0 は削除でき、断面 1 → 0 に繰り上がり参照も追随
        let before = model.clone();
        stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));
        assert_eq!(model.elements[0].section, Some(SectionId(0)));
        assert!(model.validate().is_ok());

        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
    }

    #[test]
    fn test_add_delete_material_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(AddMaterial {
                name: "SN400B".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                fc: None,
                fy: Some(235.0),
            }),
        );
        assert_eq!(model.materials.len(), 1);
        assert_eq!(model.materials[0].id, MaterialId(0));
        assert!(model.validate().is_ok());

        stack.undo(&mut model);
        assert_eq!(model.materials.len(), 0);
        stack.redo(&mut model);
        assert_eq!(model.materials.len(), 1);
    }

    #[test]
    fn test_delete_material_in_use_is_noop() {
        let mut model = two_member_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(AddMaterial {
                name: "SN400B".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                fc: None,
                fy: Some(235.0),
            }),
        );
        model.elements[0].material = Some(MaterialId(0));
        stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
        assert_eq!(model.materials.len(), 1, "使用中の材料は削除できない");
    }

    #[test]
    fn test_delete_material_middle_renumbers() {
        let mut model = two_member_model();
        let mut stack = UndoStack::new();
        for name in ["A", "B"] {
            stack.run(
                &mut model,
                Box::new(AddMaterial {
                    name: name.into(),
                    young: 1.0,
                    poisson: 0.3,
                    density: 0.0,
                    fc: None,
                    fy: None,
                }),
            );
        }
        model.elements[0].material = Some(MaterialId(1));
        let before = model.clone();
        stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
        assert_eq!(model.materials.len(), 1);
        assert_eq!(model.materials[0].name, "B");
        assert_eq!(model.elements[0].material, Some(MaterialId(0)));
        assert!(model.validate().is_ok());
        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
    }

    #[test]
    fn test_set_material_field_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(AddMaterial {
                name: "Fc21".into(),
                young: 21500.0,
                poisson: 0.2,
                density: 2.3e-9,
                fc: Some(21.0),
                fy: None,
            }),
        );
        stack.run(
            &mut model,
            Box::new(SetMaterialField {
                id: MaterialId(0),
                field: MaterialField::Fc,
                value: Some(24.0),
            }),
        );
        assert_eq!(model.materials[0].fc, Some(24.0));
        stack.undo(&mut model);
        assert_eq!(model.materials[0].fc, Some(21.0));
    }

    #[test]
    fn test_add_delete_load_case_roundtrip() {
        use squid_n_core::ids::LoadCaseId;
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
        stack.run(&mut model, Box::new(AddLoadCase { name: "LL".into() }));
        assert_eq!(model.load_cases.len(), 2);
        assert_eq!(model.load_cases[0].id, LoadCaseId(0));
        assert_eq!(model.load_cases[1].id, LoadCaseId(1));

        // 先頭を削除 → 後続 ID 繰り上がり
        let before = model.clone();
        stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
        assert_eq!(model.load_cases.len(), 1);
        assert_eq!(model.load_cases[0].id, LoadCaseId(0));
        assert_eq!(model.load_cases[0].name, "LL");

        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
    }

    #[test]
    fn test_delete_load_case_referenced_by_combo_is_noop() {
        use squid_n_core::ids::LoadCaseId;
        use squid_n_core::model::LoadCombination;
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
        model.combinations.push(LoadCombination {
            name: "combo".into(),
            terms: vec![(LoadCaseId(0), 1.0)],
        });
        stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
        assert_eq!(model.load_cases.len(), 1, "組合せ参照中のケースは削除不可");
    }

    #[test]
    fn test_add_delete_combination_roundtrip() {
        use squid_n_core::ids::LoadCaseId;
        use squid_n_core::model::LoadCombination;
        let mut model = empty_model();
        model.combinations.push(LoadCombination {
            name: "既存".into(),
            terms: vec![(LoadCaseId(0), 1.0)],
        });
        let mut stack = UndoStack::new();

        let combo = LoadCombination {
            name: "1.0DL+1.0LL".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        };
        stack.run(
            &mut model,
            Box::new(AddCombination {
                combo: combo.clone(),
            }),
        );
        assert_eq!(model.combinations.len(), 2);
        assert_eq!(model.combinations[1], combo);

        stack.undo(&mut model);
        assert_eq!(model.combinations.len(), 1);

        stack.redo(&mut model);
        assert_eq!(model.combinations.len(), 2);
        assert_eq!(model.combinations[1], combo);
    }

    #[test]
    fn test_delete_combination_roundtrip_restores_position() {
        use squid_n_core::ids::LoadCaseId;
        use squid_n_core::model::LoadCombination;
        let mut model = empty_model();
        for (name, coef) in [("A", 1.0), ("B", 2.0), ("C", 3.0)] {
            model.combinations.push(LoadCombination {
                name: name.into(),
                terms: vec![(LoadCaseId(0), coef)],
            });
        }
        let before = model.clone();
        let mut stack = UndoStack::new();

        // 中間（B）を削除
        stack.run(&mut model, Box::new(DeleteCombination { index: 1 }));
        assert_eq!(model.combinations.len(), 2);
        assert_eq!(model.combinations[0].name, "A");
        assert_eq!(model.combinations[1].name, "C");

        // undo で元の位置（index 1）に復元
        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
        assert_eq!(model.combinations[1].name, "B");

        stack.redo(&mut model);
        assert_eq!(model.combinations.len(), 2);
        assert_eq!(model.combinations[0].name, "A");
        assert_eq!(model.combinations[1].name, "C");
    }

    #[test]
    fn test_delete_combination_out_of_range_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(&mut model, Box::new(DeleteCombination { index: 0 }));
        assert!(model.combinations.is_empty());
        assert!(stack.can_undo());
        // Noop の undo でも状態は変わらない
        stack.undo(&mut model);
        assert!(model.combinations.is_empty());
    }

    #[test]
    fn test_add_delete_slab_roundtrip() {
        use squid_n_core::model::DistributionMethod;
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(AddSlab {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
            }),
        );
        assert_eq!(model.slabs.len(), 1);
        assert_eq!(model.slabs[0].id, SlabId(0));
        assert!(model.validate().is_ok());

        // 採番の確認：2 枚目は SlabId(1)
        stack.run(
            &mut model,
            Box::new(AddSlab {
                boundary: vec![NodeId(0), NodeId(1)],
                joists: vec![],
                loads: vec![],
                method: DistributionMethod::OneWay,
            }),
        );
        assert_eq!(model.slabs.len(), 2);
        assert_eq!(model.slabs[1].id, SlabId(1));

        stack.undo(&mut model);
        assert_eq!(model.slabs.len(), 1);
        assert_eq!(model.slabs[0].id, SlabId(0));

        stack.redo(&mut model);
        assert_eq!(model.slabs.len(), 2);
        assert_eq!(model.slabs[1].id, SlabId(1));
    }

    #[test]
    fn test_delete_slab_middle_renumbers_and_roundtrips() {
        use squid_n_core::model::{AreaLoad, DistributionMethod};
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        for (i, kind) in ["A", "B", "C"].iter().enumerate() {
            stack.run(
                &mut model,
                Box::new(AddSlab {
                    boundary: vec![NodeId(i as u32)],
                    joists: vec![],
                    loads: vec![AreaLoad {
                        kind: kind.to_string(),
                        value: 1.0,
                    }],
                    method: DistributionMethod::TributaryArea,
                }),
            );
        }
        assert_eq!(model.slabs.len(), 3);
        let before = model.clone();

        // 中間（SlabId(1) = "B"）を削除 → 後続 ID が繰り上がる
        stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(1) }));
        assert_eq!(model.slabs.len(), 2);
        assert_eq!(model.slabs[0].id, SlabId(0));
        assert_eq!(model.slabs[0].loads[0].kind, "A");
        assert_eq!(model.slabs[1].id, SlabId(1));
        assert_eq!(model.slabs[1].loads[0].kind, "C");
        assert!(model.validate().is_ok());

        // undo で完全復元
        stack.undo(&mut model);
        assert!(model.eq_ignoring_dofmap(&before));
        assert!(model.validate().is_ok());

        stack.redo(&mut model);
        assert_eq!(model.slabs.len(), 2);
        assert_eq!(model.slabs[1].loads[0].kind, "C");
    }

    #[test]
    fn test_delete_slab_out_of_range_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(0) }));
        assert!(model.slabs.is_empty());
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.slabs.is_empty());
    }

    fn make_story(id: u32, weight: Option<f64>) -> squid_n_core::model::Story {
        squid_n_core::model::Story {
            id: StoryId(id),
            name: format!("{}F", id + 1),
            elevation: id as f64 * 3000.0,
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: weight,
        }
    }

    #[test]
    fn test_set_story_weight_roundtrip() {
        let mut model = empty_model();
        model.stories.push(make_story(0, None));
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(SetStoryWeight {
                story: StoryId(0),
                weight: Some(1234.5),
            }),
        );
        assert_eq!(model.stories[0].seismic_weight, Some(1234.5));

        stack.undo(&mut model);
        assert_eq!(model.stories[0].seismic_weight, None);

        stack.redo(&mut model);
        assert_eq!(model.stories[0].seismic_weight, Some(1234.5));
    }

    #[test]
    fn test_set_story_weight_invalid_id_is_noop() {
        let mut model = empty_model();
        model.stories.push(make_story(0, Some(999.0)));
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(SetStoryWeight {
                story: StoryId(99),
                weight: Some(1.0),
            }),
        );
        assert_eq!(model.stories[0].seismic_weight, Some(999.0));
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert_eq!(model.stories[0].seismic_weight, Some(999.0));
    }
}
