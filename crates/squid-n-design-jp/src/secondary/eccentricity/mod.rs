//! T2: 偏心率 Re（剛心＝武藤 D値法・略算）。仕様 `dev_docs/specs/P7_二次設計.md` §5。
//!
//! 本モジュールは2層構造になっている:
//! 1. **厳密な計算コア**（`d_value` / `center_of_rigidity` / `eccentricity`）。
//!    告示1792・武藤 D値法の閉形式そのもので、手計算と 1e-9 で一致する（DoD §8.1）。
//! 2. **モデル抽出**（`column_stiffnesses` / `center_of_mass` / `story_centers`）。
//!    実モデルから柱・梁を拾って 1. に渡す略算層。柱＝鉛直部材という幾何判定等、
//!    明示した仮定の上に成り立つ（精算＝マスター節点 3×3 剛性は
//!    [`crate::secondary::eccentricity_analysis`] を参照）。
//!
//! さらに雑壁（フレーム外の壁）の剛性を n 倍法で等価剛性要素へ換算し、剛心・
//! ねじり剛性へ寄与させる層（`misc_wall_stiffness` / `append_misc_wall_stiffnesses`）
//! を末尾に持つ。
//!
//! **方向の扱い（★最重要）:** 剛心座標は方向別 D 値で重み付けする。
//! `Xs = Σ(Dy·x)/ΣDy`, `Ys = Σ(Dx·y)/ΣDx`。単一 D 値で済むのは対称架構のみ。

mod core;
mod misc_wall;
mod model_extract;

pub use core::{center_of_rigidity, d_value, eccentricity, ColumnStiffness, Eccentricity};
pub use misc_wall::{append_misc_wall_stiffnesses, misc_wall_stiffness, sum_column_area};
pub use model_extract::{
    center_of_mass, column_stiffnesses, story_centers, story_eccentricity, StoryCenters,
};

#[cfg(test)]
use squid_n_core::ids::StoryId;

/// テスト専用のモデル構築ヘルパー。`crate::secondary::eccentricity` と
/// `crate::secondary::eccentricity_analysis` の双方のテストから共用する。
#[cfg(test)]
pub(crate) mod test_support {
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::Model;

    /// 対称4柱・田の字梁モデルを構築するヘルパー。
    /// 柱: 底 z=0（拘束）・頂 z=3000、story=Some(S0)。
    /// 梁: 上端節点間を X 方向・Y 方向に接続（同一 section）。
    /// 質量: 上端4節点に等質量（mass[0]=1.0）。
    /// section_iy_override: 右側 2 本（x=6000）の柱の iy を指定値に差し替え（None なら全同一）。
    pub(crate) fn build_symmetric_frame(section_iy_override: Option<f64>) -> (Model, StoryId) {
        use smallvec::SmallVec;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node,
            RigidZone, Section, Story,
        };

        // 断面（共通: iy=iz=1.0e6）
        let sec_base = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 100.0,
            iy: 1.0e6,
            iz: 1.0e6,
            j: 1.0e6,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        // 右側柱用 section（iy を上書き）
        let sec_right = Section {
            id: SectionId(1),
            name: "col_right".to_string(),
            iy: section_iy_override.unwrap_or(1.0e6),
            ..sec_base.clone()
        };
        // 梁用 section: iz を非常に大きくして全柱で a ≈ 1（kbar→∞）にする。
        // これにより D ≈ Kc0 = 12EI/h³ ∝ iz となり「Dy 比 = iz 比」が精度良く成立。
        let sec_beam = Section {
            id: SectionId(2),
            name: "beam".to_string(),
            area: 100.0,
            iy: 1.0e12,
            iz: 1.0e12,
            j: 1.0e12,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };

        // 材料（共通）
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 2.05e5,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };

        // 層
        let s0 = StoryId(0);
        let story = Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: s0,
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: None,
        };

        // 節点配置:
        // NodeId 0-3: 底部（z=0、story=None、拘束）
        // NodeId 4-7: 上部（z=3000、story=Some(S0)、質量有り）
        // 平面位置: 0→(0,0), 1→(6000,0), 2→(0,6000), 3→(6000,6000)
        let restraint_fixed = Dof6Mask::FIXED;
        let restraint_free = Dof6Mask::FREE;
        let mass_val: Option<[f64; 6]> = Some([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
        let xy = [
            [0.0_f64, 0.0],
            [6000.0, 0.0],
            [0.0, 6000.0],
            [6000.0, 6000.0],
        ];
        let mut nodes: Vec<Node> = Vec::new();
        for (i, &[x, y]) in xy.iter().enumerate() {
            nodes.push(Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: restraint_fixed,
                mass: None,
                story: None,
            });
        }
        for (i, &[x, y]) in xy.iter().enumerate() {
            nodes.push(Node {
                id: NodeId((i + 4) as u32),
                coord: [x, y, 3000.0],
                restraint: restraint_free,
                mass: mass_val,
                story: Some(s0),
            });
        }

        // 部材構築ヘルパー
        // 柱の ref_vector = [0,1,0] にすると:
        //   ex=[0,0,1], ey=[0,1,0](Y軸), ez=[1,0,0](X軸)
        // model_extract.rs は断面→要素座標系のクロス変換（elem iz ← sec.iy）を行うため:
        //   I_globalX = elem_iz·ey[0]² + elem_iy·ez[0]² = elem_iy·1 = sec.iz → Dx ∝ sec.iz
        //   I_globalY = elem_iz·ey[1]² + elem_iy·ez[1]² = elem_iz·1 = sec.iy → Dy ∝ sec.iy（★意図）
        // これにより「右側柱の iy を 3 倍 → Dy が 3 倍 → 剛心 Xs = 4500」が成立。
        let col_local_axis = LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        };
        let beam_local_axis = LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        };
        let end_fixed = [EndCondition::Fixed, EndCondition::Fixed];

        // 柱: bottom i → top i+4
        // 左側 (x=0): SectionId(0)、右側 (x=6000): SectionId(0 or 1)
        let col_sec = |i: usize| -> SectionId {
            if section_iy_override.is_some() && (xy[i][0] - 6000.0).abs() < 1.0 {
                SectionId(1)
            } else {
                SectionId(0)
            }
        };
        let mut elements: Vec<ElementData> = Vec::new();
        for i in 0..4_usize {
            elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: {
                    let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                    v.push(NodeId(i as u32));
                    v.push(NodeId((i + 4) as u32));
                    v
                },
                section: Some(col_sec(i)),
                material: Some(MaterialId(0)),
                local_axis: col_local_axis,
                end_cond: end_fixed,
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
                spring: None,
            });
        }

        // 梁: X方向（同 y、異なる x）: top0-top1, top2-top3
        // 梁: Y方向（同 x、異なる y）: top0-top2, top1-top3
        // ElemId 4..7
        let beam_pairs: [(usize, usize); 4] = [(4, 5), (6, 7), (4, 6), (5, 7)];
        for (bi, &(na, nb)) in beam_pairs.iter().enumerate() {
            elements.push(ElementData {
                id: ElemId((4 + bi) as u32),
                kind: ElementKind::Beam,
                nodes: {
                    let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                    v.push(NodeId(na as u32));
                    v.push(NodeId(nb as u32));
                    v
                },
                section: Some(SectionId(2)),
                material: Some(MaterialId(0)),
                local_axis: beam_local_axis,
                end_cond: end_fixed,
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
                spring: None,
            });
        }

        let sections = if section_iy_override.is_some() {
            vec![sec_base, sec_right, sec_beam]
        } else {
            vec![
                sec_base,
                Section {
                    id: SectionId(1),
                    ..sec_right
                },
                sec_beam,
            ]
        };

        let model = Model {
            nodes,
            elements,
            sections,
            materials: vec![mat],
            stories: vec![story],
            ..Default::default()
        };
        (model, s0)
    }
}

#[cfg(test)]
mod tests;
