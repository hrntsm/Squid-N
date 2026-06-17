use crate::beam::BeamElement;
use crate::behavior::{ElemState, LocalMat, LocalVec, MassOption};
use crate::shear_spring::ShearSpring;
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::MaterialId;
use sc_core::model::Model;
use smallvec::SmallVec;

/// 軸ばね1本：断面内の位置と材料を保持（P5.5 §3）
pub struct AxialSpring {
    pub y: f64,
    pub z: f64,
    pub material: MaterialId,
}

/// MS（マルチスプリング）要素（P5.5 §3）
/// 部材端の断面を軸方向ばね群で置換し、中央は弾性材で連結。
pub struct MsElement {
    pub springs: Vec<AxialSpring>,
    pub elastic_mid: BeamElement,
    pub shear: ShearSpring,
}

impl MsElement {
    pub fn new(data: &sc_core::model::ElementData, model: &Model) -> Self {
        let n_springs = 10;
        let half = (n_springs - 1) as f64 / 2.0;

        // 軸ばねを等間隔に配置（断面内 y 座標）
        let springs: Vec<AxialSpring> = (0..n_springs)
            .map(|i| {
                let y = (i as f64 - half) / half;
                AxialSpring {
                    y,
                    z: 0.0,
                    material: data.material.unwrap_or(MaterialId(0)),
                }
            })
            .collect();

        let elastic_mid = BeamElement::new(data, model);
        let shear = ShearSpring::new(data, model);

        MsElement {
            springs,
            elastic_mid,
            shear,
        }
    }
}

impl crate::behavior::ElementBehavior for MsElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.elastic_mid.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, state: &ElemState, ctx: &crate::behavior::Ctx) -> LocalMat {
        // 中央弾性部の剛性 + せん断ばね剛性（軸ばね群の寄与は将来）
        let k_elastic = self.elastic_mid.tangent_stiffness(state, ctx);
        let k_shear = self.shear.stiffness_12x12();
        let mut k = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let v = k_elastic.get(i, j) + k_shear.get(i, j);
                k.set(i, j, v);
            }
        }
        k
    }

    fn internal_force(&self, state: &ElemState, ctx: &crate::behavior::Ctx) -> LocalVec {
        self.elastic_mid.internal_force(state, ctx)
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.elastic_mid.mass_matrix(opt)
    }
}
