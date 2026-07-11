//! 耐震壁（壁エレメントモデル）の側柱（RESP-D マニュアル計算編 02
//! 「耐震壁(壁エレメントモデル)の剛性」側柱の断面性能）。
//!
//! マニュアル規定:
//! 「軸剛性・曲げ剛性・せん断剛性は通常の柱と同様に計算します。ただし、ＲＣ耐震壁
//! 面内方向は両端ピンのためモーメントを負担しません」「…せん断力を負担しません」。
//!
//! すなわち側柱は、壁面内方向の曲げに対してのみ両端ピン（その曲げ面のモーメント・
//! せん断力を負担しない）とし、面外方向・軸・ねじりは通常の柱と同じ剛性を持つ。
//! 両端ピンの曲げ面は、静的縮約でその面の回転自由度（要素ローカル 12 自由度中の
//! 2 個）を消去することで表現する。剛接梁（`beam.rs` の `condense_end_springs`）と
//! 異なり片端ではなく両端を同時に解放するため、縮約後はその面のせん断（並進）剛性も
//! 厳密にゼロとなる（両端ピン材は面内方向に対し機構となり、面内の相対水平変位・
//! 回転のいずれに対しても内力を生じない）。

use crate::beam::{invert_small, BeamElement};
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_core::section_shape::SectionShape;

/// 解放する局所曲げ面（回転自由度）。
///
/// - `LocalY`: 局所 y 軸回りの回転（ry, 要素ローカル自由度 4・10）を解放。
///   曲げ面は局所 x-z 面（たわみ方向 = 局所 z 軸）。
/// - `LocalZ`: 局所 z 軸回りの回転（rz, 要素ローカル自由度 5・11）を解放。
///   曲げ面は局所 x-y 面（たわみ方向 = 局所 y 軸）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReleaseAxis {
    LocalY,
    LocalZ,
}

/// 面内方向のみ両端ピンとした側柱（耐震壁の側柱）。
///
/// 内部に通常の柱と同じ `BeamElement` を持ち、剛性計算時に指定曲げ面の両端回転
/// 自由度を静的縮約で消去した 12×12 を用いる。軸・ねじり・面外曲げは `inner` と
/// 変わらない。
pub struct InPlaneReleasedColumn {
    inner: BeamElement,
    release_axis: ReleaseAxis,
}

impl InPlaneReleasedColumn {
    pub fn new(inner: BeamElement, release_axis: ReleaseAxis) -> Self {
        Self {
            inner,
            release_axis,
        }
    }

    /// 解放対象の局所自由度（両端の回転自由度2個）。
    fn release_dofs(&self) -> [usize; 2] {
        match self.release_axis {
            ReleaseAxis::LocalY => [4, 10],
            ReleaseAxis::LocalZ => [5, 11],
        }
    }

    /// `inner.local_stiffness()` から解放曲げ面の両端回転自由度を静的縮約した局所 12×12。
    ///
    /// K* = Kaa − Kab·Kbb⁻¹·Kba（a: 残す10自由度、b: 解放する2自由度）。
    /// 縮約後の b 自由度の行・列は 0（その回転自由度に剛性を持たない＝ピン）。
    /// 軸・ねじり・他方向曲げは元の局所剛性で b 自由度と非連成のため影響を受けない。
    fn released_local_stiffness(&self) -> LocalMat {
        let k = self.inner.local_stiffness();
        let b = self.release_dofs();
        let n = k.n;

        // Kbb（2×2）とその逆行列
        let kbb = vec![
            k.get(b[0], b[0]),
            k.get(b[0], b[1]),
            k.get(b[1], b[0]),
            k.get(b[1], b[1]),
        ];
        let kbb_inv = invert_small(&kbb, 2);

        let mut out = LocalMat::zeros(n);
        for i in 0..n {
            if b.contains(&i) {
                continue;
            }
            let kai = [k.get(i, b[0]), k.get(i, b[1])];
            for j in 0..n {
                if b.contains(&j) {
                    continue;
                }
                let kbj = [k.get(b[0], j), k.get(b[1], j)];
                let mut corr = 0.0;
                for p in 0..2 {
                    for q in 0..2 {
                        corr += kai[p] * kbb_inv[p * 2 + q] * kbj[q];
                    }
                }
                out.set(i, j, k.get(i, j) - corr);
            }
        }
        out
    }

    /// 縮約後の局所剛性を用いた断面力の復元（`BeamElement::recover_forces` と同じ規約）。
    /// `BeamElement::recover_forces` は自身の（非解放の）`local_stiffness()` を用いるため、
    /// ここでは解放後の局所剛性で同じ算定式を再実装する。
    fn recover_forces_released(&self, u_elem_global: &[f64; 12]) -> crate::beam::MemberForces {
        let u_local = self.inner.axis.rotate_to_local(u_elem_global);
        let k_local = self.released_local_stiffness();
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        let length = self.inner.length;
        let mut at = Vec::new();
        for &xi in &self.inner.eval_sections {
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = -f_local[0];
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = f_local[3];
                let my = f_local[4] - f_local[2] * xi * length;
                let mz = f_local[5] + f_local[1] * xi * length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[6];
                let qy = -f_local[7];
                let qz = -f_local[8];
                let mx = f_local[9];
                let my = f_local[10] - f_local[8] * (1.0 - xi) * length;
                let mz = f_local[11] + f_local[7] * (1.0 - xi) * length;
                (n, qy, qz, mx, my, mz)
            };
            at.push((xi, [n, qy, qz, mx, my, mz]));
        }
        crate::beam::MemberForces { at }
    }
}

impl ElementBehavior for InPlaneReleasedColumn {
    fn n_dof(&self) -> usize {
        self.inner.n_dof()
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.inner.global_dofs(dof)
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        self.inner.axis.to_global(&self.released_local_stiffness())
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // committed_disp はグローバル系で蓄積される（BeamElement と同じ規約）ため、
        // 解放後の局所剛性をグローバルへ回した K で内力を評価する。
        let k = self.inner.axis.to_global(&self.released_local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.inner.committed_disp[j];
            }
            f.data[i] = s;
        }
        f
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

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces_released(&arr))
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = norm(a);
    if l < 1e-9 {
        None
    } else {
        Some([a[0] / l, a[1] / l, a[2] / l])
    }
}

/// 壁（Section.shape=RcWall、または材料に fc がある）かどうか
/// （マニュアル「ＲＣ耐震壁面内方向は…」＝ RC 耐震壁限定の側柱ピン規定）。
fn is_rc_wall(wall: &ElementData, model: &Model) -> bool {
    let sec_is_rc_wall = wall
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .is_some_and(|s| matches!(s.shape, Some(SectionShape::RcWall { .. })));
    let mat_is_rc = wall
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .is_some_and(|m| m.fc.is_some());
    sec_is_rc_wall || mat_is_rc
}

/// 自部材（`data`）が耐震壁（壁エレメントモデル）の側柱（面内両端ピンの柱）かどうかを
/// 判定し、そうであれば解放すべき局所曲げ面を返す。
///
/// 条件:
/// 1. 自部材が鉛直材であること（dz が dx・dy に対して支配的）。
/// 2. `model.elements` 中に節点数4以上の `ElementKind::Wall` があり、かつ RC 限定
///    （`is_rc_wall`）を満たすこと。
/// 3. その壁の四隅を z で下辺2・上辺2 に分け、下辺の軸方向への射影で上辺と対応付けた
///    （`wall_panel.rs::try_new` と同じロジック）とき、自部材の両端節点が
///    「下辺a-上辺a」または「下辺b-上辺b」のいずれかの鉛直辺の2節点と一致すること。
///
/// 解放曲げ面は、壁面法線（下辺方向×鉛直の外積）と柱の局所 ey・ez の内積絶対値が
/// 大きい方（＝回転軸が壁法線に平行な方）とする。
pub fn wall_side_column_release(data: &ElementData, model: &Model) -> Option<ReleaseAxis> {
    if data.nodes.len() < 2 {
        return None;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[1];
    let node0 = model.nodes.get(n0.index())?;
    let node1 = model.nodes.get(n1.index())?;
    let (p0, p1) = (node0.coord, node1.coord);
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    // 鉛直材の判定（factory.rs::is_vertical_member と同じ規約）
    if dz.abs() <= (dx.abs() + dy.abs()) * 0.5 {
        return None;
    }

    for wall in &model.elements {
        if !matches!(wall.kind, ElementKind::Wall) || wall.nodes.len() < 4 {
            continue;
        }
        if !is_rc_wall(wall, model) {
            continue;
        }
        // 耐震壁が不成立（フレーム内雑壁）の場合、柱は側柱としてピン化せず、
        // 通常の柱として袖壁付きの断面性能算入（`beam.rs`）を受ける
        // （RESP-D 計算編 02「フレーム内雑壁のモデル化」）。
        if !crate::misc_wall::wall_is_seismic(wall, model) {
            continue;
        }

        let ids: Vec<NodeId> = wall.nodes.iter().take(4).copied().collect();
        let Some(coords) = ids
            .iter()
            .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };

        // z で下辺2節点・上辺2節点に分ける（wall_panel.rs::try_new と同じロジック）
        let mut order: Vec<usize> = (0..4).collect();
        order.sort_by(|&a, &b| coords[a][2].partial_cmp(&coords[b][2]).unwrap());
        let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);

        let (pa, pb) = (coords[b0], coords[b1]);
        let Some(ex_bot) = unit(sub(pb, pa)) else {
            continue;
        };
        // 上辺は下辺の a に近い方を a とする（対応付け）
        let (ta, tb) = {
            let d0 = dot(sub(coords[t0], pa), ex_bot).abs();
            let d1 = dot(sub(coords[t1], pa), ex_bot).abs();
            if d0 <= d1 {
                (t0, t1)
            } else {
                (t1, t0)
            }
        };

        // 自部材の両端節点が同一鉛直辺（下辺a-上辺a、または下辺b-上辺b）と一致するか
        let side_a = (ids[b0], ids[ta]);
        let side_b = (ids[b1], ids[tb]);
        let matches_side = |side: (NodeId, NodeId)| -> bool {
            (side.0 == n0 && side.1 == n1) || (side.0 == n1 && side.1 == n0)
        };
        if !(matches_side(side_a) || matches_side(side_b)) {
            continue;
        }

        // 壁面法線 = 下辺方向 × 鉛直
        let up = [0.0, 0.0, 1.0];
        let Some(normal) = unit(cross(ex_bot, up)) else {
            continue;
        };

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let dot_ey = dot(axis.rot[1], normal).abs();
        let dot_ez = dot(axis.rot[2], normal).abs();
        return Some(if dot_ey >= dot_ez {
            ReleaseAxis::LocalY
        } else {
            ReleaseAxis::LocalZ
        });
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone};

    fn make_test_column(release_axis: ReleaseAxis) -> InPlaneReleasedColumn {
        // 識別しやすいよう iy・iz を非対称にした軸方向材（軸=ローカルX、
        // 局所座標系=グローバル座標系のまま。曲げ面判定を単純化するため）。
        let inner = BeamElement {
            id: ElemId(0),
            e: 23000.0,
            g: 9583.33,
            a: 250_000.0,
            a_mass: 250_000.0,
            iy: 5.0e9,
            iz: 3.0e9,
            j: 1.0e8,
            as_y: 200_000.0,
            as_z: 200_000.0,
            length: 3000.0,
            density: 2.4e-9,
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame {
                rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            rigid: RigidZone::default(),
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: None,
            material: None,
            committed_disp: [0.0; 12],
        };
        InPlaneReleasedColumn::new(inner, release_axis)
    }

    /// 通常の柱剛性（両端固定・非解放）から、指定した並進自由度に単位変位を
    /// 与えたときのひずみエネルギ u^T K u を返す（節点0側は完全固定=0とする）。
    fn energy_at(k: &LocalMat, dof_j: usize) -> f64 {
        let mut u = [0.0; 12];
        u[dof_j] = 1.0;
        let mut s = 0.0;
        for i in 0..12 {
            for j in 0..12 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    }

    /// 面内曲げ面（解放対象）は縮約後にせん断・モーメントとも負担しない
    /// （壁面内方向は両端ピン＝機構）ことを、局所=グローバルとなる恒等軸で確認する。
    ///
    /// `BeamElement::local_stiffness` は端部固定条件を巨大ペナルティばねの静的縮約で
    /// 近似する（`condense_end_springs`）ため、縮約前後の値は理論値と厳密には一致せず
    /// 相対 1e-8 程度の誤差を持つ。「ゼロになる」判定はこの誤差の影響を受けないよう、
    /// 非解放面の公称剛性（12·az / 12·ay）を基準とした相対値で行う。
    #[test]
    fn test_release_zeroes_inplane_bending_plane() {
        let model = Model::default();
        let ctx = Ctx { model: &model };

        // 両曲げ面の公称固定端剛性（解放していない通常の柱としての基準値）
        let col_ref = make_test_column(ReleaseAxis::LocalZ);
        let phi_y = 12.0 * col_ref.inner.e * col_ref.inner.iy
            / (col_ref.inner.g * col_ref.inner.as_z * col_ref.inner.length * col_ref.inner.length);
        let ay =
            col_ref.inner.e * col_ref.inner.iy / ((1.0 + phi_y) * col_ref.inner.length.powi(3));
        let expected_uz = 12.0 * ay;
        let phi_z = 12.0 * col_ref.inner.e * col_ref.inner.iz
            / (col_ref.inner.g * col_ref.inner.as_y * col_ref.inner.length * col_ref.inner.length);
        let az =
            col_ref.inner.e * col_ref.inner.iz / ((1.0 + phi_z) * col_ref.inner.length.powi(3));
        let expected_uy = 12.0 * az;

        // LocalZ 解放（rz を消去）→ uy 面（1,5,7,11。iz を用いる面）が縮約でゼロになる。
        let col_z = col_ref;
        let k_z = col_z.tangent_stiffness(&ElemState::default(), &ctx);
        let e_uy_zero = energy_at(&k_z, 7);
        assert!(
            e_uy_zero.abs() / expected_uy < 1e-6,
            "LocalZ解放でuy面のエネルギがゼロでない: {e_uy_zero}"
        );
        // 面外（uz面、iy を用いる。8番）は通常剛性のまま
        let e_uz = energy_at(&k_z, 8);
        assert!(
            (e_uz - expected_uz).abs() / expected_uz < 1e-6,
            "面外剛性が変化した: e_uz={e_uz} expected={expected_uz}"
        );

        // LocalY 解放（ry を消去）→ uz 面（2,4,8,10。iy を用いる面）が縮約でゼロになる。
        let col_y = make_test_column(ReleaseAxis::LocalY);
        let k_y = col_y.tangent_stiffness(&ElemState::default(), &ctx);
        let e_uz_zero = energy_at(&k_y, 8);
        assert!(
            e_uz_zero.abs() / expected_uz < 1e-6,
            "LocalY解放でuz面のエネルギがゼロでない: {e_uz_zero}"
        );
        // 面外（uy面、iz を用いる）は通常剛性のまま
        let e_uy = energy_at(&k_y, 7);
        assert!(
            (e_uy - expected_uy).abs() / expected_uy < 1e-6,
            "面外剛性が変化した: e_uy={e_uy} expected={expected_uy}"
        );
    }

    /// 軸剛性 EA/L は解放の影響を受けない。
    #[test]
    fn test_release_keeps_axial_stiffness() {
        let model = Model::default();
        let ctx = Ctx { model: &model };
        for axis in [ReleaseAxis::LocalY, ReleaseAxis::LocalZ] {
            let col = make_test_column(axis);
            let k = col.tangent_stiffness(&ElemState::default(), &ctx);
            let expected = col.inner.e * col.inner.a / col.inner.length;
            let e_axial = energy_at(&k, 6);
            assert!(
                (e_axial - expected).abs() / expected < 1e-6,
                "axis={axis:?} e_axial={e_axial} expected={expected}"
            );
        }
    }

    /// 剛体移動（両端同一変位・回転0）では解放柱も内力ゼロ。
    #[test]
    fn test_rigid_translation_zero_internal_force() {
        let model = Model::default();
        let ctx = Ctx { model: &model };
        for axis in [ReleaseAxis::LocalY, ReleaseAxis::LocalZ] {
            let mut col = make_test_column(axis);
            for dir in 0..3 {
                let mut u = [0.0; 12];
                u[dir] = 1.0;
                u[6 + dir] = 1.0;
                col.inner.committed_disp = u;
                let f = col.internal_force(&ElemState::default(), &ctx);
                for (i, fi) in f.data.iter().enumerate() {
                    assert!(
                        fi.abs() < 1e-6,
                        "axis={axis:?} dir={dir} dof={i} 剛体移動で内力が生じた: {fi}"
                    );
                }
            }
        }
    }

    /// 縮約後の全体系剛性は対称（浮動小数点丸め誤差程度の相対誤差を許容）。
    #[test]
    fn test_released_stiffness_symmetric() {
        let model = Model::default();
        let ctx = Ctx { model: &model };
        for axis in [ReleaseAxis::LocalY, ReleaseAxis::LocalZ] {
            let col = make_test_column(axis);
            let k = col.tangent_stiffness(&ElemState::default(), &ctx);
            for i in 0..12 {
                for j in 0..12 {
                    let kij = k.get(i, j);
                    let kji = k.get(j, i);
                    let tol = 1e-9 * (kij.abs() + kji.abs() + 1.0);
                    assert!(
                        (kij - kji).abs() < tol,
                        "axis={axis:?} K[{i}][{j}]={kij} != K[{j}][{i}]={kji}"
                    );
                }
            }
        }
    }

    fn make_node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }
    }

    /// 4000×3000 の RC 耐震壁（X-Z 面内、Y=0）を持つモデル。
    /// 節点0=下辺a(0,0,0)、1=下辺b(4000,0,0)、2=上辺(4000,0,3000)、3=上辺(0,0,3000)。
    fn make_wall_model() -> Model {
        let wall_shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![wall_shape.to_section(SectionId(0), "W150".into())],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Wall,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            ..Default::default()
        }
    }

    fn make_column_data(nodes: [u32; 2]) -> ElementData {
        ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(nodes[0]), NodeId(nodes[1])],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 壁の左辺（下辺a-上辺a）を結ぶ柱 → 側柱として Some
    #[test]
    fn test_wall_side_column_release_left_edge_is_some() {
        let model = make_wall_model();
        let data = make_column_data([0, 3]);
        assert!(wall_side_column_release(&data, &model).is_some());
    }

    /// 壁と無関係な柱（節点が壁の節点と一致しない）→ None
    #[test]
    fn test_wall_side_column_release_unrelated_column_is_none() {
        let mut model = make_wall_model();
        model.nodes.push(make_node(4, [100_000.0, 100_000.0, 0.0]));
        model
            .nodes
            .push(make_node(5, [100_000.0, 100_000.0, 3000.0]));
        let data = make_column_data([4, 5]);
        assert!(wall_side_column_release(&data, &model).is_none());
    }

    /// 壁の対角節点（下辺a-上辺b）を結ぶ斜材 → None
    /// （鉛直材の判定は通過するが、同一鉛直辺の対にならないため側柱ではない）
    #[test]
    fn test_wall_side_column_release_diagonal_brace_is_none() {
        let model = make_wall_model();
        let data = make_column_data([0, 2]);
        assert!(wall_side_column_release(&data, &model).is_none());
    }
}
