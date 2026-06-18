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
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RigidZone {
    pub length_i: f64,
    pub length_j: f64,
    pub source_i: ZoneSource,
    pub source_j: ZoneSource,
    pub reduction: f64,
}

impl Default for RigidZone {
    fn default() -> Self {
        Self {
            length_i: 0.0,
            length_j: 0.0,
            source_i: ZoneSource::Auto,
            source_j: ZoneSource::Auto,
            reduction: 1.0,
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
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DiaphragmDef {
    pub master: NodeId,
    pub slaves: Vec<NodeId>,
    pub rigid: bool,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Story {
    pub id: StoryId,
    pub name: String,
    pub elevation: f64,
    pub node_ids: Vec<NodeId>,
    pub diaphragms: Vec<DiaphragmDef>,
    pub seismic_weight: Option<f64>,
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

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Slab {
    pub id: SlabId,
    pub boundary: Vec<NodeId>,
    pub joists: Vec<JoistLine>,
    pub loads: Vec<AreaLoad>,
    pub method: DistributionMethod,
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
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NodalLoad {
    pub node: NodeId,
    pub values: [f64; 6],
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCase {
    pub id: LoadCaseId,
    pub name: String,
    pub nodal: Vec<NodalLoad>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCombination {
    pub name: String,
    pub terms: Vec<(LoadCaseId, f64)>,
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
            }],
            ..Default::default()
        };
        assert!(model.validate().is_err());
    }

    #[test]
    fn test_shear_modulus_explicit() {
        let mat = Material {
            id: MaterialId(0),
            name: "Test".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(80000.0),
        };
        assert_eq!(mat.shear_modulus(), 80000.0);
    }

    #[test]
    fn test_shear_modulus_derived() {
        let mat = Material {
            id: MaterialId(0),
            name: "Test".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
        };
        let expected = 205000.0 / (2.0 * (1.0 + 0.3));
        assert!((mat.shear_modulus() - expected).abs() < 1e-9);
    }

    #[test]
    fn test_rect_shear_area() {
        let area = 80000.0;
        let as_ = rect_shear_area(area);
        assert!((as_ - area * 5.0 / 6.0).abs() < 1e-9);
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
        };
        assert_eq!(sec.depth, 0.0);
        assert!(sec.panel_thickness.is_none());
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
