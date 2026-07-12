//! 節点・部材の編集コマンド。

use super::*;
use squid_n_core::ids::*;

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
        // 剛床代表節点かどうかを退避し、リストからは先に除去してから ID を繰り上げる。
        let generated_master =
            if let Some(pos) = model.generated_masters.iter().position(|n| *n == self.id) {
                model.generated_masters.remove(pos);
                true
            } else {
                false
            };
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
            generated_master,
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
    /// 削除された節点が `generated_masters`（剛床代表節点）に含まれていたか。
    /// 含まれていた場合、再挿入後の ID を `generated_masters` へ戻す。
    pub generated_master: bool,
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
        if self.generated_master {
            model.generated_masters.push(id);
            model.generated_masters.sort();
        }
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
    for id in &mut model.generated_masters {
        f(id);
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

/// 制振ダンパー要素の追加（RESP-D「07 非線形解析（動的解析）」制振要素）。
/// 要素（`ElementKind::Damper`）と特性（`Model::damper_attrs`）を原子的に追加する。
/// 逆操作は部材削除（`DeleteMember` が側テーブル属性も退避・復元する）。
pub struct AddDamper {
    pub elem: squid_n_core::model::ElementData,
    pub props: squid_n_core::model::DamperProps,
}

impl EditCommand for AddDamper {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let id = self.elem.id;
        model.elements.push(self.elem.clone());
        model.set_damper_props(id, Some(self.props));
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "制振ダンパー追加"
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
        // 側テーブル属性（履歴則・ダンパー・免震等）を退避してから削除（残余は shift で繰上げ）。
        let removed_attrs = model.take_elem_attrs(self.id);
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
            elem_attrs: removed_attrs,
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
    /// 削除時に退避した側テーブル属性（履歴則・ダンパー・免震等）。
    pub elem_attrs: squid_n_core::model::ElemAttrs,
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
        // 退避した側テーブル属性を再挿入 ID へ紐づけ直して復元。
        model.restore_elem_attrs(id, self.elem_attrs.clone());
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "部材削除の取り消し"
    }
}

/// モデル内の全ての `ElemId` 参照（部材自身の ID・部材荷重・要素キー付き側テーブル）に
/// `f` を適用する。要素の削除・挿入に伴う ID 繰上げ／繰下げで参照整合を保つ。
fn shift_elem_ids(model: &mut Model, mut f: impl FnMut(&mut ElemId)) {
    for elem in &mut model.elements {
        f(&mut elem.id);
    }
    for lc in &mut model.load_cases {
        for ml in &mut lc.member {
            f(&mut ml.elem);
        }
    }
    // 壁・鉄骨・BRB・PCa・免震・履歴則・ダンパーの側テーブルも同様に繰上げ／繰下げする。
    model.shift_elem_attr_refs(&mut f);
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
