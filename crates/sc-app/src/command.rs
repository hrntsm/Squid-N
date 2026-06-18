pub trait EditCommand {
    fn apply(&self, model: &mut sc_core::model::Model) -> Box<dyn EditCommand>;
    fn label(&self) -> &str;
}

pub struct SetNodeCoord {
    pub node: sc_core::ids::NodeId,
    pub coord: [f64; 3],
}

impl EditCommand for SetNodeCoord {
    fn apply(&self, model: &mut sc_core::model::Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        let old = if idx < model.nodes.len() && model.nodes[idx].id == self.node {
            model.nodes[idx].coord
        } else {
            [0.0; 3]
        };
        if idx < model.nodes.len() && model.nodes[idx].id == self.node {
            model.nodes[idx].coord = self.coord;
        }
        Box::new(SetNodeCoord {
            node: self.node,
            coord: old,
        })
    }

    fn label(&self) -> &str {
        "SetNodeCoord"
    }
}

pub struct AddMember {
    pub elem: sc_core::model::ElementData,
}

impl EditCommand for AddMember {
    fn apply(&self, model: &mut sc_core::model::Model) -> Box<dyn EditCommand> {
        model.elements.push(self.elem.clone());
        Box::new(DeleteMember { id: self.elem.id })
    }

    fn label(&self) -> &str {
        "AddMember"
    }
}

pub struct DeleteMember {
    pub id: sc_core::ids::ElemId,
}

impl EditCommand for DeleteMember {
    fn apply(&self, model: &mut sc_core::model::Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        let removed = if idx < model.elements.len() && model.elements[idx].id == self.id {
            model.elements.remove(idx)
        } else {
            return Box::new(Noop);
        };
        Box::new(AddMember { elem: removed })
    }

    fn label(&self) -> &str {
        "DeleteMember"
    }
}

pub struct Noop;

impl EditCommand for Noop {
    fn apply(&self, _model: &mut sc_core::model::Model) -> Box<dyn EditCommand> {
        Box::new(Noop)
    }

    fn label(&self) -> &str {
        "Noop"
    }
}

pub struct UndoStack {
    done: Vec<Box<dyn EditCommand>>,
    undone: Vec<Box<dyn EditCommand>>,
}

impl Default for UndoStack {
    fn default() -> Self {
        Self::new()
    }
}

impl UndoStack {
    pub fn new() -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
        }
    }

    pub fn run(&mut self, model: &mut sc_core::model::Model, cmd: Box<dyn EditCommand>) {
        let inv = cmd.apply(model);
        self.done.push(inv);
        self.undone.clear();
    }

    pub fn undo(&mut self, model: &mut sc_core::model::Model) {
        if let Some(inv) = self.done.pop() {
            let redo_inv = inv.apply(model);
            self.undone.push(redo_inv);
        }
    }

    pub fn redo(&mut self, model: &mut sc_core::model::Model) {
        if let Some(redo_inv) = self.undone.pop() {
            let inv = redo_inv.apply(model);
            self.done.push(inv);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::dof::Dof6Mask;
    use sc_core::ids::{ElemId, NodeId};
    use sc_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
    };

    #[test]
    fn test_set_node_coord_roundtrip() {
        let mut model = Model {
            nodes: vec![Node {
                id: NodeId(0),
                coord: [1.0, 2.0, 3.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            }],
            ..Default::default()
        };
        let cmd = SetNodeCoord {
            node: NodeId(0),
            coord: [10.0, 20.0, 30.0],
        };
        let inv = cmd.apply(&mut model);
        assert_eq!(model.nodes[0].coord, [10.0, 20.0, 30.0]);
        let _redo = inv.apply(&mut model);
        assert_eq!(model.nodes[0].coord, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_undo_redo() {
        let mut model = Model::default();
        let mut stack = UndoStack::new();
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
}
