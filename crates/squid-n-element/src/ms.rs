use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::fiber_elem::FiberBeam;
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::MaterialId;
use squid_n_core::model::Model;
use std::any::Any;

/// 軸ばね1本：断面内の位置と材料を保持（P5.5 §3）
pub struct AxialSpring {
    pub y: f64,
    pub z: f64,
    pub material: MaterialId,
}

/// MS（マルチスプリング）要素（P5.5 §3）
///
/// 部材端の塑性化領域（長さ Lp）の断面を少数の軸方向バネ群（2×5 = 10本の
/// 2次元配置）で置換し、中央は弾性材で連結する。軸バネ群の合力が N、
/// 図心まわりの偶力が M となるため、N-M 相関を自然に表現できる（P5.5 §6.2）。
///
/// 実体は塑性化域考慮ファイバー要素（`FiberBeam::build_plastic_zone`）と
/// 同一の定式化で、端部断面の分割数だけが粗い（バネ=粗いファイバ）。
/// これにより trial/commit/rollback・チェックポイントも FiberBeam と
/// 同じ機構に乗る（P5 §6）。
///
/// 注: 旧実装の1次元バネ配置（y 軸上 10 本）は一軸曲げ専用だったため、
/// 2次元配置（2列×5段）へ一般化した。
pub struct MsElement {
    /// 軸バネ配置（両端共通。断面内座標と負担面積は `inner` の端部断面が保持）
    pub springs: Vec<AxialSpring>,
    /// 実体: 端部バネ断面 + 中央弾性 + せん断バネ
    pub inner: FiberBeam,
}

/// MS 要素の端部バネ断面の分割数（幅方向 × せい方向 = 10 本）
const MS_NW: usize = 2;
const MS_ND: usize = 5;

impl MsElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        // 塑性化領域長さ: 入力があればそれを、なければ断面せいの 0.5 倍
        let depth = data
            .section
            .and_then(|sid| model.sections.get(sid.index()))
            .map(|s| s.depth)
            .filter(|d| *d > 0.0)
            .unwrap_or(200.0);
        let lp = data.plastic_zone.unwrap_or(0.5 * depth);

        let inner = FiberBeam::build_plastic_zone(data, model, lp, MS_NW, MS_ND);

        // 互換用のバネ配置情報（端部断面のファイバ位置と同一）
        let springs = inner
            .gauss_points
            .first()
            .map(|gp| {
                gp.section
                    .fibers
                    .iter()
                    .map(|f| AxialSpring {
                        y: f.y,
                        z: f.z,
                        material: data.material.unwrap_or(MaterialId(0)),
                    })
                    .collect()
            })
            .unwrap_or_default();

        MsElement { springs, inner }
    }
}

impl ElementBehavior for MsElement {
    fn n_dof(&self) -> usize {
        self.inner.n_dof()
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.inner.global_dofs(dof)
    }

    fn tangent_stiffness(&self, state: &ElemState, ctx: &Ctx) -> LocalMat {
        self.inner.tangent_stiffness(state, ctx)
    }

    fn internal_force(&self, state: &ElemState, ctx: &Ctx) -> LocalVec {
        self.inner.internal_force(state, ctx)
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, ctx: &Ctx) {
        self.inner.update_state(du, commit, ctx);
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.inner.mass_matrix(opt)
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        self.inner.geometric_stiffness(n)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        self.inner.snapshot_state()
    }

    fn restore_state(&mut self, state: &dyn Any) {
        self.inner.restore_state(state);
    }

    fn commit_state(&mut self) {
        self.inner.commit_state();
    }

    fn revert_state(&mut self) {
        self.inner.revert_state();
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        self.inner.serialize_checkpoint()
    }

    fn deserialize_checkpoint(&mut self, data: &[u8]) {
        self.inner.deserialize_checkpoint(data);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node, Section,
    };

    /// 鋼断面柱相当のテストモデル（fy 付き材料 → バイリニアバネ）
    fn make_model(fy: Option<f64>, fc: Option<f64>) -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [3000.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Ms,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "ms-test".to_string(),
                area: 250000.0,
                iy: 5.2083e9,
                iz: 5.2083e9,
                j: 0.0,
                depth: 500.0,
                width: 500.0,
                as_y: 208333.0,
                as_z: 208333.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc,
                fy,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_ms_has_2d_spring_layout() {
        let model = make_model(Some(295.0), None);
        let elem = MsElement::new(&model.elements[0], &model);
        assert_eq!(elem.springs.len(), 10);
        // 2次元配置: y も z も複数の異なる座標を持つ（一軸曲げ専用でない）
        let mut ys: Vec<i64> = elem.springs.iter().map(|s| s.y as i64).collect();
        let mut zs: Vec<i64> = elem.springs.iter().map(|s| s.z as i64).collect();
        ys.sort();
        ys.dedup();
        zs.sort();
        zs.dedup();
        assert!(ys.len() >= 2, "バネは幅方向にも分布する");
        assert!(zs.len() >= 2, "バネはせい方向にも分布する");
    }

    #[test]
    fn test_ms_axial_force_reduces_moment_capacity() {
        // N-M 相関: 軸圧縮を与えた状態では端部モーメントの頭打ちが下がる
        let model = make_model(Some(295.0), None);
        let ctx = Ctx { model: &model };

        // i端の端部断面で最外縁ひずみ ≈ 10εy となる回転角
        // （κ(ξ=-1) = (4/L)·θ、εmax = κ·d/2）
        let eps_y = 295.0 / 205000.0;
        let kappa = 20.0 * eps_y / 500.0;
        let theta = kappa * 3000.0 / 4.0;

        // ケース1: 純曲げ
        let mut elem1 = MsElement::new(&model.elements[0], &model);
        let du1 = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, theta, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem1.update_state(&du1, false, &ctx);
        let f1 = elem1.internal_force(&ElemState::default(), &ctx);
        let m1 = f1.data[4].abs();

        // ケース2: 同じ回転 + 軸ひずみ −5εy（中立軸シフト → N/Npl ≈ 0.5 相当）。
        // 軸バネと曲げが非連成のモデルなら軸変位は DOF4 のモーメントに一切影響しない
        // ため、m2 < m1 が N-M 連成の直接の証拠になる。
        let mut elem2 = MsElement::new(&model.elements[0], &model);
        let u_axial = -5.0 * eps_y * 3000.0;
        let du2 = LocalVec {
            data: smallvec::smallvec![
                0.0, 0.0, 0.0, 0.0, theta, 0.0, u_axial, 0.0, 0.0, 0.0, 0.0, 0.0
            ],
        };
        elem2.update_state(&du2, false, &ctx);
        let f2 = elem2.internal_force(&ElemState::default(), &ctx);
        let m2 = f2.data[4].abs();

        // バネ群の全塑性モーメント Mp ≈ Σa·|z|·fy = 2列×25000mm²×(200+100+0+100+200)×295
        // ≈ 8.85e9 N·mm。N/Npl=0.5 相当の低減(≈25%)の一部が現れれば連成は機能している。
        assert!(
            m1 - m2 > 1.0e9,
            "高軸圧縮下で曲げ耐力が低下するはず: m1={m1}, m2={m2}"
        );
    }

    #[test]
    fn test_ms_commit_revert_roundtrip() {
        let model = make_model(Some(295.0), None);
        let ctx = Ctx { model: &model };
        let mut elem = MsElement::new(&model.elements[0], &model);

        // 降伏させてコミット
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, true, &ctx);
        let f_committed = elem.internal_force(&ElemState::default(), &ctx);

        // さらに trial を進めてから revert → コミット状態の内力へ戻る
        // （revert 後の応力キャッシュはソルバ契約どおり次の update_state で更新される
        //   ため、ゼロ増分を与えて再評価してから比較する）
        let du2 = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du2, false, &ctx);
        elem.revert_state();
        let zero = LocalVec {
            data: smallvec::smallvec![0.0; 12],
        };
        elem.update_state(&zero, false, &ctx);
        let f_reverted = elem.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            approx::assert_relative_eq!(
                f_committed.data[i],
                f_reverted.data[i],
                epsilon = 1e-6,
                max_relative = 1e-9
            );
        }
    }

    #[test]
    fn test_ms_checkpoint_roundtrip() {
        let model = make_model(Some(295.0), None);
        let ctx = Ctx { model: &model };
        let mut elem = MsElement::new(&model.elements[0], &model);
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.02, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, true, &ctx);
        let cp = elem.serialize_checkpoint();

        let mut elem2 = MsElement::new(&model.elements[0], &model);
        elem2.deserialize_checkpoint(&cp);
        // 復元後、同じ増分に対する応答が一致する
        let du2 = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du2, false, &ctx);
        elem2.update_state(&du2, false, &ctx);
        let f1 = elem.internal_force(&ElemState::default(), &ctx);
        let f2 = elem2.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            approx::assert_relative_eq!(f1.data[i], f2.data[i], epsilon = 1e-6);
        }
    }
}
