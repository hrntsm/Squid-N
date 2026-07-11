//! 耐震壁（壁エレメントモデル）要素（RESP-D マニュアル計算編 02「剛性計算」）。
//!
//! 鉛直の梁要素（壁柱＝間柱）を両端ピンの剛梁ではさみ込んだ 4 節点 24 自由度要素。
//! 剛梁と壁柱は剛接合、剛梁の両端はピン接合のため、四隅節点の並進のみが
//! 壁柱端の並進（両隅の平均）と回転（剛梁の剛体回転＝両隅の変位差/剛梁長）に
//! 伝達され、四隅節点の回転自由度には剛性を与えない（＝ピン）。
//! 剛梁は実要素ではなく、この変換（剛域変換に相当）で表現する。
//!
//! 壁柱の断面性能:
//! - 軸剛性: 壁板断面積 t·lw に鉄筋剛性を考慮（壁筋比 ps を縦横共通とみなし
//!   (1+(n−1)·ps) を乗じる近似。n=Es/Ec）
//! - 曲げ剛性: 壁板断面の面内断面2次モーメント t·lw³/12（側柱のローカル I は
//!   不算入）に同係数を乗じる
//! - せん断剛性: (壁板断面＋側柱断面)/κ に開口低減率 r を乗じる。
//!   κ は側柱がある場合 I 形断面の形状係数（`wall_shear_shape_factor`、
//!   ξ・η の定義は要原典照合）、無い場合は矩形の 1.2
//!
//! 上下大梁の剛性倍率（既定 100 倍）は梁要素側（`beam.rs`）で扱う。
//! 側柱の面内両端ピン化は未対応（方向別端部解放が必要。照合レポート参照）。

use crate::beam::BeamElement;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};
use squid_n_core::section_shape::{wall_shear_shape_factor, SectionShape, E_STEEL, KAPPA_RC};

/// 耐震壁（壁エレメントモデル）。
pub struct WallPanelElement {
    /// [下辺a, 下辺b, 上辺a, 上辺b]（a→b が剛梁の軸方向。上下で対応付け済み）
    nodes: [NodeId; 4],
    /// 壁柱（仮想中央柱。上下剛梁の中点を結ぶ）
    column: BeamElement,
    /// 壁柱端 12 自由度 ← 四隅 24 自由度 の変換行列 A（row-major 12×24）。
    /// 四隅の回転自由度に対応する列は常に 0（ピン）。
    a_mat: Vec<f64>,
    /// 質量算定用の壁板総質量 [質量単位]
    mass_total: f64,
}

impl WallPanelElement {
    /// 生成。4 節点未満・寸法/断面が不定の場合は None
    /// （呼び出し側は従来の暫定等価梁へフォールバックする）。
    pub fn try_new(data: &ElementData, model: &Model) -> Option<Self> {
        Self::try_new_scaled(data, model, 1.0)
    }

    /// 剛性スケール付き生成。耐震壁不成立（フレーム内雑壁）の壁は剛性を
    /// 周辺部材へ算入するため、壁要素自体は `stiffness_scale`（微小値）で
    /// 実質無剛性とし、質量のみを保持する（RESP-D 計算編 02
    /// 「フレーム内雑壁のモデル化」）。
    pub(crate) fn try_new_scaled(
        data: &ElementData,
        model: &Model,
        stiffness_scale: f64,
    ) -> Option<Self> {
        if data.nodes.len() < 4 {
            return None;
        }
        let ids: Vec<NodeId> = data.nodes.iter().take(4).copied().collect();
        let coords: Vec<[f64; 3]> = ids
            .iter()
            .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
            .collect::<Option<Vec<_>>>()?;

        // z で下辺 2 節点・上辺 2 節点に分ける
        let mut order: Vec<usize> = (0..4).collect();
        order.sort_by(|&a, &b| coords[a][2].partial_cmp(&coords[b][2]).unwrap());
        let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);

        // 下辺の軸方向 a→b
        let (pa, pb) = (coords[b0], coords[b1]);
        let ex_bot = unit(sub(pb, pa))?;
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
        let ex_top = unit(sub(coords[tb], coords[ta]))?;

        let lw_bot = norm(sub(pb, pa));
        let lw_top = norm(sub(coords[tb], coords[ta]));
        let bc = mid(pa, pb);
        let tc = mid(coords[ta], coords[tb]);
        let h = norm(sub(tc, bc));
        if lw_bot <= 0.0 || lw_top <= 0.0 || h <= 0.0 {
            return None;
        }
        let lw = 0.5 * (lw_bot + lw_top);

        // 壁板厚: RcWall 形状 → Section.thickness → Section.width の順で採用
        let sec = data
            .section
            .and_then(|sid| model.sections.get(sid.index()))?;
        let t = match &sec.shape {
            Some(SectionShape::RcWall { thickness, .. }) => *thickness,
            _ => sec.thickness.unwrap_or(sec.width),
        };
        if t <= 0.0 {
            return None;
        }
        let mat = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))?;

        // 開口低減率 r（複数開口モード考慮）。r=0 でせん断断面積が 0 になると
        // φ 項が NaN になるため微小値を下限とする。
        let r = crate::factory::wall_opening_reduction(data, model).max(1e-6);

        // 鉄筋剛性の考慮（壁筋比 ps を縦横共通とみなす近似）: (1+(n−1)·ps)
        let ps = match &sec.shape {
            Some(SectionShape::RcWall { ps, .. }) => (*ps).max(0.0),
            _ => 0.0,
        };
        let rebar_factor = if mat.fc.is_some() && mat.young > 0.0 && ps > 0.0 {
            1.0 + (E_STEEL / mat.young - 1.0) * ps
        } else {
            1.0
        };

        // 側柱（壁の鉛直辺の 2 節点を両端に持つ鉛直 Beam 部材）を収集し、
        // せん断断面への算入と I 形形状係数 κ の算定に用いる。
        let edge_pairs = [[ids[b0], ids[ta]], [ids[b1], ids[tb]]];
        let mut col_area_sum = 0.0;
        let mut col_depth_sum = 0.0; // 沿壁方向せい（両側の和）
        let mut col_width_max: f64 = 0.0;
        for e in &model.elements {
            if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            let (n0, n1) = (e.nodes[0], e.nodes[1]);
            let is_edge = edge_pairs
                .iter()
                .any(|p| (p[0] == n0 && p[1] == n1) || (p[0] == n1 && p[1] == n0));
            if !is_edge {
                continue;
            }
            if let Some(cs) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                col_area_sum += cs.area;
                col_depth_sum += cs.depth.max(cs.width);
                col_width_max = col_width_max.max(cs.width.min(cs.depth).max(t));
            }
        }
        // κ: 側柱があれば I 形断面の形状係数（ξ=内法長さ/外面間全長、η=t/側柱幅。
        // 定義は要原典照合）、無ければ矩形の 1.2。
        let kappa = if col_area_sum > 0.0 && col_width_max > 0.0 {
            let l_total = lw + col_depth_sum / 2.0;
            let l_clear = (lw - col_depth_sum / 2.0).max(0.0);
            wall_shear_shape_factor(l_clear / l_total, (t / col_width_max).min(1.0))
        } else {
            KAPPA_RC
        };

        let area = t * lw;
        let as_gross = area + col_area_sum;
        let column = BeamElement {
            id: data.id,
            e: mat.young * stiffness_scale,
            g: mat.shear_modulus() * stiffness_scale,
            a: area * rebar_factor,
            a_mass: area,
            // 面内曲げ（局所 z 軸まわり）= t·lw³/12、面外 = lw·t³/12
            iy: lw * t.powi(3) / 12.0,
            iz: t * lw.powi(3) / 12.0 * rebar_factor,
            j: lw * t.powi(3) / 3.0,
            // 面内せん断（局所 y 方向）: (壁板+側柱)/κ に開口低減 r を考慮
            as_y: r * as_gross / kappa,
            as_z: area / KAPPA_RC,
            length: h,
            density: mat.density,
            nodes: [ids[b0], ids[ta]],
            axis: LocalFrame::from_nodes(bc, tc, ex_bot),
            rigid: Default::default(),
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
        };

        // 変換行列 A（壁柱端 ← 四隅並進）。
        // 並進: u_c = (u_a + u_b)/2
        // 回転: ω = ex × (u_b − u_a)/lw（剛梁の剛体回転。剛梁軸まわり成分は
        //        ピンのため伝達されず 0）
        let mut a_mat = vec![0.0; 12 * 24];
        let corner_slot = |idx: usize| -> usize {
            // nodes 配列 [b_a, b_b, t_a, t_b] 中の位置 → 24 自由度中のオフセット
            idx * 6
        };
        let node_order = [b0, b1, ta, tb];
        let slot_of = |orig: usize| -> usize {
            node_order
                .iter()
                .position(|&x| x == orig)
                .expect("node_order は 4 節点の並べ替え")
        };
        let mut fill_end = |col_base: usize, ca: usize, cb: usize, ex: [f64; 3], lw_e: f64| {
            let (sa, sb) = (corner_slot(slot_of(ca)), corner_slot(slot_of(cb)));
            for tdof in 0..3 {
                a_mat[(col_base + tdof) * 24 + sa + tdof] += 0.5;
                a_mat[(col_base + tdof) * 24 + sb + tdof] += 0.5;
            }
            // ω_i = Σ_jk ε_ijk・ex_j・(u_b − u_a)_k / lw
            for i in 0..3 {
                for j in 0..3 {
                    for k in 0..3 {
                        let e = levi_civita(i, j, k);
                        if e == 0.0 {
                            continue;
                        }
                        let c = e * ex[j] / lw_e;
                        a_mat[(col_base + 3 + i) * 24 + sb + k] += c;
                        a_mat[(col_base + 3 + i) * 24 + sa + k] -= c;
                    }
                }
            }
        };
        fill_end(0, b0, b1, ex_bot, lw_bot);
        fill_end(6, ta, tb, ex_top, lw_top);

        Some(Self {
            nodes: [ids[b0], ids[b1], ids[ta], ids[tb]],
            column,
            a_mat,
            mass_total: mat.density * t * lw * h,
        })
    }

    /// 全体系 24×24 剛性 K = Aᵀ·K_col·A。
    fn stiffness_24(&self) -> LocalMat {
        let k12 = self.column.axis.to_global(&self.column.local_stiffness());
        let mut k = LocalMat::zeros(24);
        // K = Aᵀ K12 A
        for p in 0..24 {
            for q in 0..24 {
                let mut s = 0.0;
                for i in 0..12 {
                    let aip = self.a_mat[i * 24 + p];
                    if aip == 0.0 {
                        continue;
                    }
                    for j in 0..12 {
                        let ajq = self.a_mat[j * 24 + q];
                        if ajq != 0.0 {
                            s += aip * k12.get(i, j) * ajq;
                        }
                    }
                }
                if s != 0.0 {
                    k.set(p, q, s);
                }
            }
        }
        k
    }

    /// 四隅変位 24 → 壁柱端変位 12（全体系）。
    fn to_column_disp(&self, u24: &[f64]) -> [f64; 12] {
        let mut u12 = [0.0; 12];
        for (i, ui) in u12.iter_mut().enumerate() {
            let mut s = 0.0;
            for p in 0..24 {
                s += self.a_mat[i * 24 + p] * u24[p];
            }
            *ui = s;
        }
        u12
    }
}

impl ElementBehavior for WallPanelElement {
    fn n_dof(&self) -> usize {
        24
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        self.stiffness_24()
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        LocalVec {
            data: smallvec::smallvec![0.0; 24],
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // 壁板質量を四隅の並進へ 1/4 ずつ集中（Consistent 指定も同じ扱い）
        let mut mm = LocalMat::zeros(24);
        let m_node = self.mass_total / 4.0;
        for i in 0..4 {
            let bo = i * 6;
            for d in 0..3 {
                mm.set(bo + d, bo + d, m_node);
            }
        }
        mm
    }

    fn geometric_stiffness(&self, _n: f64) -> LocalMat {
        LocalMat::zeros(24)
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        // 壁柱の断面力（N・Q・M）として復元する
        let u12 = self.to_column_disp(&u_elem[..24]);
        Some(self.column.recover_forces(&u12))
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mid(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
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

fn levi_civita(i: usize, j: usize, k: usize) -> f64 {
    match (i, j, k) {
        (0, 1, 2) | (1, 2, 0) | (2, 0, 1) => 1.0,
        (0, 2, 1) | (2, 1, 0) | (1, 0, 2) => -1.0,
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node};
    use squid_n_core::section_shape::SectionShape;

    /// 4000×3000×t150 の壁（X-Z 面内）を持つモデル。
    fn make_wall_model() -> (Model, ElementData) {
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "W150".into())],
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
            ..Default::default()
        };
        let data = ElementData {
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
        };
        (model, data)
    }

    fn energy(k: &LocalMat, u: &[f64; 24]) -> f64 {
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    }

    #[test]
    fn test_wall_panel_rigid_translation_zero_force() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 全節点に同一並進（剛体移動）→ 力ゼロ
        for dir in 0..3 {
            let mut u = [0.0; 24];
            for n in 0..4 {
                u[n * 6 + dir] = 1.0;
            }
            for i in 0..24 {
                let f: f64 = (0..24).map(|j| k.get(i, j) * u[j]).sum();
                assert!(
                    f.abs() < 1e-6,
                    "剛体移動で内力が生じた: dir={dir} i={i} f={f}"
                );
            }
        }
    }

    #[test]
    fn test_wall_panel_inplane_shear_matches_column() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 上辺 2 節点を面内水平(X)に単位変位（下辺固定・上辺回転 0 = 両端固定柱の
        // せん断変形モード）→ ひずみエネルギ uᵀKu が壁柱の両端固定水平剛性
        // 12EI/((1+φ)h³) と一致する
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0; // 上辺 a の ux
        u[3 * 6] = 1.0; // 上辺 b の ux
        let uku = energy(&k, &u);

        let col = &wall.column;
        let phi = 12.0 * col.e * col.iz / (col.g * col.as_y * col.length * col.length);
        let expected = 12.0 * col.e * col.iz / ((1.0 + phi) * col.length.powi(3));
        assert!(
            (uku - expected).abs() / expected < 1e-6,
            "uKu={uku} expected={expected}"
        );
    }

    #[test]
    fn test_wall_panel_vertical_matches_axial() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 上辺 2 節点を鉛直に単位変位 → EA/h
        let mut u = [0.0; 24];
        u[2 * 6 + 2] = 1.0;
        u[3 * 6 + 2] = 1.0;
        let uku = energy(&k, &u);
        let col = &wall.column;
        let expected = col.e * col.a / col.length;
        assert!(
            (uku - expected).abs() / expected < 1e-6,
            "uKu={uku} expected={expected}"
        );
    }

    #[test]
    fn test_wall_panel_corner_rotations_are_pinned() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 四隅の回転自由度は剛性を持たない（剛梁両端ピン）
        for n in 0..4 {
            for d in 3..6 {
                let idx = n * 6 + d;
                for j in 0..24 {
                    assert!(
                        k.get(idx, j).abs() < 1e-9 && k.get(j, idx).abs() < 1e-9,
                        "回転自由度に剛性: node={n} dof={d}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_wall_panel_opening_reduces_inplane_shear() {
        let (mut model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k_no = wall.stiffness_24();
        model.wall_attrs.push(squid_n_core::model::WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6, // 25%
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        let wall_o = WallPanelElement::try_new(&data, &model).unwrap();
        let k_o = wall_o.stiffness_24();
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0;
        u[3 * 6] = 1.0;
        assert!(
            energy(&k_o, &u) < energy(&k_no, &u) * 0.999,
            "開口低減が面内せん断剛性に効いていない"
        );
    }

    /// 鉄筋剛性の考慮: a = t·lw·(1+(n−1)·ps)、n=Es/Ec。
    #[test]
    fn test_wall_panel_rebar_factor() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let n = squid_n_core::section_shape::E_STEEL / 23000.0;
        let expected = 150.0 * 4000.0 * (1.0 + (n - 1.0) * 0.0025);
        assert!((wall.column.a - expected).abs() < 1e-6);
        // 質量用は幾何断面のまま
        assert!((wall.column.a_mass - 150.0 * 4000.0).abs() < 1e-9);
    }

    /// 側柱があるとせん断断面に算入され、I 形の形状係数 κ が用いられる。
    #[test]
    fn test_wall_panel_side_columns_increase_shear_area() {
        let (mut model, data) = make_wall_model();
        let wall_plain = WallPanelElement::try_new(&data, &model).unwrap();

        // 両側の鉛直辺(節点0-3・1-2)に 600×600 の側柱を追加
        let col_shape = SectionShape::RcRect {
            b: 600.0,
            d: 600.0,
            rebar: squid_n_core::section_shape::RcRebar {
                main_x: squid_n_core::section_shape::BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: squid_n_core::section_shape::BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: squid_n_core::section_shape::ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        model
            .sections
            .push(col_shape.to_section(SectionId(1), "C600".into()));
        for (eid, n0, n1) in [(1u32, 0u32, 3u32), (2, 1, 2)] {
            model.elements.push(ElementData {
                id: ElemId(eid),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        let wall_cols = WallPanelElement::try_new(&data, &model).unwrap();
        // (壁板+側柱2本)/κ(I形) > 壁板/1.2
        assert!(
            wall_cols.column.as_y > wall_plain.column.as_y,
            "側柱算入で as_y が増えない: {} vs {}",
            wall_cols.column.as_y,
            wall_plain.column.as_y
        );
        let a_gross = 150.0 * 4000.0 + 2.0 * 360_000.0;
        // κ = as_gross/as_y(逆算)が矩形の 1.2 と異なる(I形の値)
        let kappa = a_gross / wall_cols.column.as_y;
        assert!(
            (kappa - 1.2).abs() > 1e-3,
            "κ が I 形になっていない: {kappa}"
        );
    }

    #[test]
    fn test_wall_panel_try_new_fallbacks() {
        let (model, mut data) = make_wall_model();
        // 2 節点しか無い場合は None（従来の暫定等価梁へ）
        data.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
        assert!(WallPanelElement::try_new(&data, &model).is_none());
    }
}
