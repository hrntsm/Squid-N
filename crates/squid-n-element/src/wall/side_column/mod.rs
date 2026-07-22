//! 耐震壁（壁エレメントモデル）の側柱（RC規準の耐震壁規定。
//! 側柱の断面性能）。
//!
//! 側柱の扱い:
//! 軸剛性・曲げ剛性・せん断剛性は通常の柱と同様に計算する。ただし、ＲＣ耐震壁
//! 面内方向は両端ピンのためモーメント・せん断力を負担しない。
//!
//! すなわち側柱は、壁面内方向の曲げに対してのみ両端ピン（その曲げ面のモーメント・
//! せん断力を負担しない）とし、面外方向・軸・ねじりは通常の柱と同じ剛性を持つ。
//! 両端ピンの曲げ面は、静的縮約でその面の回転自由度（要素ローカル 12 自由度中の
//! 2 個）を消去することで表現する。剛接梁（`beam.rs` の `condense_end_springs`）と
//! 異なり片端ではなく両端を同時に解放するため、縮約後はその面のせん断（並進）剛性も
//! 厳密にゼロとなる（両端ピン材は面内方向に対し機構となり、面内の相対水平変位・
//! 回転のいずれに対しても内力を生じない）。
//!
//! - [`InPlaneReleasedColumn`] — 面内両端ピンの側柱要素本体（`column`）
//! - [`wall_side_column_release`] — 自部材が側柱かどうかの幾何判定（`detect`）

mod column;
mod detect;

pub use column::{InPlaneReleasedColumn, ReleaseAxis};
pub use detect::wall_side_column_release;

#[cfg(test)]
use crate::beam::BeamElement;
#[cfg(test)]
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat};
#[cfg(test)]
use crate::transform::LocalFrame;
#[cfg(test)]
use squid_n_core::ids::NodeId;
#[cfg(test)]
use squid_n_core::model::{ElementData, ElementKind, Model};
#[cfg(test)]
use squid_n_core::section_shape::SectionShape;

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
            trial_disp: [0.0; 12],
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
    /// internal_force はトライアル追従（trial_disp 参照）のため trial 側へ変位を
    /// 与える。片端のみの変位（非剛体）では内力が生じることも併せて確認し、
    /// 「trial が空のまま常にゼロ」という無意味な合格を防ぐ。
    #[test]
    fn test_rigid_translation_zero_internal_force() {
        let model = Model::default();
        let ctx = Ctx { model: &model };
        for axis in [ReleaseAxis::LocalY, ReleaseAxis::LocalZ] {
            let mut col = make_test_column(axis);

            // 有意性確認: 片端のみの軸方向伸び（非剛体）では内力が生じること。
            // （横方向の片端変位は解放曲げ面では剛体回転となり内力ゼロが正しい
            // ため、常に内力が出る軸方向で「trial が空のまま恒等的にゼロ」という
            // 無意味な合格を防ぐ。）
            let mut u_axial = [0.0; 12];
            u_axial[0] = 1.0;
            col.inner.trial_disp = u_axial;
            let f_axial = col.internal_force(&ElemState::default(), &ctx);
            assert!(
                f_axial.data.iter().any(|fi| fi.abs() > 1e-3),
                "axis={axis:?} 片端軸方向変位で内力が生じない（テストが無効化されている）"
            );

            for dir in 0..3 {
                // 剛体移動: 内力ゼロ
                let mut u = [0.0; 12];
                u[dir] = 1.0;
                u[6 + dir] = 1.0;
                col.inner.trial_disp = u;
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

    /// recover_forces（縮約剛性版）の内力場が i/j 分岐（ξ=0.5）をまたいで
    /// 連続・整合であること（`BeamElement::recover_forces` と同じ規約）。
    /// N/Qy/Qz/Mx は一定、Mz/My は dMz/dx = Qy・dMy/dx = −Qz の線形場になる。
    #[test]
    fn test_released_recover_forces_field_continuous() {
        for axis in [ReleaseAxis::LocalY, ReleaseAxis::LocalZ] {
            let mut col = make_test_column(axis);
            col.inner.eval_sections = vec![0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0];
            let u = [
                0.1, 2.0, -1.5, 0.004, 0.002, -0.003, //
                -0.2, -1.0, 0.5, -0.002, 0.004, 0.001,
            ];
            let mf = col.recover_forces(&u).unwrap();
            let l = col.inner.length;
            let f0 = mf.at.first().unwrap().1;
            for &(xi, f) in &mf.at {
                for (k, name) in [(0, "N"), (1, "Qy"), (2, "Qz"), (3, "Mx")] {
                    let tol = 1e-6 * f0[k].abs().max(1.0);
                    assert!(
                        (f[k] - f0[k]).abs() < tol,
                        "axis={axis:?} xi={xi} {name}={} が一定でない (端={})",
                        f[k],
                        f0[k]
                    );
                }
                let mz_expected = f0[5] + f0[1] * xi * l;
                let my_expected = f0[4] - f0[2] * xi * l;
                assert!(
                    (f[5] - mz_expected).abs() < 1e-6 * mz_expected.abs().max(1.0),
                    "axis={axis:?} xi={xi} Mz={} expected={mz_expected}",
                    f[5]
                );
                assert!(
                    (f[4] - my_expected).abs() < 1e-6 * my_expected.abs().max(1.0),
                    "axis={axis:?} xi={xi} My={} expected={my_expected}",
                    f[4]
                );
            }
        }
    }
}
