use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{EndCondition, Material, Model, RigidZone, Section, ZoneSource};

pub struct RigidZoneRule {
    pub reduction: f64,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self { reduction: 1.0 }
    }
}

#[derive(Clone, Debug)]
pub struct MemberForces {
    pub at: Vec<(f64, [f64; 6])>,
}

#[derive(Clone)]
pub struct BeamElement {
    pub id: ElemId,
    pub e: f64,
    pub g: f64,
    /// 軸剛性（EA）用断面積。SRC では鉄骨の等価換算断面を累加した値になる。
    pub a: f64,
    /// 質量算定（ρ·A·L）用の幾何断面積。SRC の等価換算で質量が過大に
    /// ならないよう `a` と区別する（RESP-D 計算編 02 の An はあくまで剛性用）。
    pub a_mass: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
    pub length: f64,
    pub density: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub rigid: RigidZone,
    pub end_cond: [EndCondition; 2],
    pub eval_sections: Vec<f64>,
    pub section: Option<squid_n_core::ids::SectionId>,
    pub material: Option<squid_n_core::ids::MaterialId>,
    /// 確定変位（線形要素の内力計算用。非線形では ElemState が保持）
    pub committed_disp: [f64; 12],
}

fn get_section(model: &Model, sid: Option<squid_n_core::ids::SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section {
        id: squid_n_core::ids::SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    })
}

fn get_material(model: &Model, mid: Option<squid_n_core::ids::MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    })
}

/// スラブ協力幅による強軸曲げ剛性の増大率（RESP-D 計算編 02「RC大梁」。
/// 協力幅は RC 規準 8 条による）。
///
/// 対象は水平な RC 矩形梁のみ。梁の両端節点をともに境界節点に含むスラブを
/// 「梁に取り付く床」とみなし、梁軸から左右のスラブ奥行き a を求めて
/// ba=(0.5−0.6·a/l)·a（a≥l/2 のとき 0.1·l）で協力幅を算定する。
/// スラブ（厚さ t=`Model::slab_thickness`、建物一律・上端は梁上端と同面）を
/// 考慮した中立軸による T 形断面の Ie を元断面 I0=b·D³/12 で除した値を返す。
/// t≤0・非水平・取り付く床なしでは 1.0（増大なし）。
/// 連続梁の λ・吹抜け補正・二重スラブ/片持ちスラブの区別は未対応（v1。
/// docs/v_and_v/剛性計算_RESP-D照合.md 参照）。
fn slab_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
    d: f64,
) -> f64 {
    let t = model.slab_thickness;
    if t <= 0.0 || b <= 0.0 || d <= 0.0 || data.nodes.len() < 2 || model.slabs.is_empty() {
        return 1.0;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[data.nodes.len() - 1];
    let (Some(node0), Some(node1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
    else {
        return 1.0;
    };
    let (p0, p1) = (node0.coord, node1.coord);
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let lp = (dx * dx + dy * dy).sqrt();
    // 水平材のみ対象（勾配 5% までは水平とみなす）
    if lp < 1e-9 || dz.abs() > 0.05 * lp {
        return 1.0;
    }
    let l = (lp * lp + dz * dz).sqrt();
    let (ex, ey) = (dx / lp, dy / lp);

    // 梁軸の左右それぞれのスラブ奥行き a（複数スラブは大きい方を採用）
    let mut a_pos: f64 = 0.0;
    let mut a_neg: f64 = 0.0;
    for slab in &model.slabs {
        if !(slab.boundary.contains(&n0) && slab.boundary.contains(&n1)) {
            continue;
        }
        for nid in &slab.boundary {
            let Some(q) = model.nodes.get(nid.index()) else {
                continue;
            };
            // 平面内で梁軸に直交する符号付き距離
            let s = -(q.coord[0] - p0[0]) * ey + (q.coord[1] - p0[1]) * ex;
            a_pos = a_pos.max(s);
            a_neg = a_neg.max(-s);
        }
    }

    // RC 規準 8 条の片側協力幅
    let ba = |a: f64| -> f64 {
        if a <= 0.0 {
            0.0
        } else if a < 0.5 * l {
            (0.5 - 0.6 * a / l) * a
        } else {
            0.1 * l
        }
    };
    let bf = b + ba(a_pos) + ba(a_neg);
    if bf <= b {
        return 1.0;
    }

    // スラブを考慮した中立軸による T 形断面の Ie
    let tf = t.min(d);
    let aw = b * d;
    let af = (bf - b) * tf;
    let g = (aw * d / 2.0 + af * (d - tf / 2.0)) / (aw + af);
    let i0 = b * d.powi(3) / 12.0;
    let ie = i0
        + aw * (g - d / 2.0).powi(2)
        + (bf - b) * tf.powi(3) / 12.0
        + af * (d - tf / 2.0 - g).powi(2);
    (ie / i0).max(1.0)
}

/// 壁エレメントモデルの上下大梁の剛性倍率（RESP-D 計算編 02「壁エレメントモデル
/// (壁エレメントモデル)の剛性」上下大梁の断面性能）。
///
/// 「上下大梁の断面性能: 通常の大梁に対し、倍率を乗じた剛性を採用します。倍率は
/// 剛性計算条件で設定できます。既定値は100倍となります。」
/// 壁エレメントモデルの上下大梁は、壁の剛性を四隅の節点へ正しく伝えるため剛体に
/// 近い扱いとする。剛性計算条件 UI からの倍率変更は将来対応（現状は既定値固定）。
pub const WALL_GIRDER_STIFF_FACTOR: f64 = 100.0;

/// 自部材（両端節点 n0, n1）が壁エレメントモデルの上辺・下辺大梁かどうかを判定する。
///
/// `model.elements` 中に節点数4以上（四隅を持つ）の `ElementKind::Wall` 要素があり、
/// その壁の節点集合に自部材の両端節点がともに含まれていれば、その壁の上辺または
/// 下辺の大梁とみなす（RESP-D 計算編 02「壁エレメントモデルの上下大梁」）。
///
/// ただし対象は耐震壁が成立した壁のみ（`misc_wall::wall_is_seismic`）。不成立の
/// フレーム内雑壁の上下梁は 100 倍せず、代わりに腰壁/垂壁として断面性能へ算入する
/// （`BeamElement::new` 内の雑壁算入処理）。
fn is_wall_top_bottom_girder(model: &Model, n0: NodeId, n1: NodeId) -> bool {
    model.elements.iter().any(|e| {
        matches!(e.kind, squid_n_core::model::ElementKind::Wall)
            && e.nodes.len() >= 4
            && e.nodes.contains(&n0)
            && e.nodes.contains(&n1)
            && crate::misc_wall::wall_is_seismic(e, model)
    })
}

impl BeamElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = if n0.index() < model.nodes.len() {
            model.nodes[n0.index()].coord
        } else {
            [0.0; 3]
        };
        let p1 = if n1.index() < model.nodes.len() {
            model.nodes[n1.index()].coord
        } else {
            [0.0; 3]
        };
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, data.material);
        let g = mat.shear_modulus();

        // 危険断面位置（§6.2.3、既定は柱フェース＝節点から face_i/j）を正規化座標へ変換し、
        // 節点芯 [0.0, 1.0] と部材中央 0.5 に加えて評価断面リストへ含める。
        // face=0（直交材が無い端）では従来どおり [0.0, 0.5, 1.0] と完全一致する。
        let eval_sections = if len > 1e-12 {
            let xi_i = (data.rigid_zone.face_i / len).clamp(0.0, 0.5 - 1e-9);
            let xi_j = (1.0 - data.rigid_zone.face_j / len).clamp(0.5 + 1e-9, 1.0);
            let mut xs = vec![0.0, xi_i, 0.5, xi_j, 1.0];
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            xs
        } else {
            vec![0.0, 0.5, 1.0]
        };

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };

        // SRC/CFT の複合換算断面性能（RESP-D 計算編 02「剛性計算」）。
        // 要素材料からヤング係数比を算定して剛性用の断面性能を上書きする。
        // - SRC: 材料=コンクリート（fc あり）のとき ns=Es/Ec で累加。
        // - CFT: 材料=鋼管の young と充填コンクリート強度 fc から 1/n 換算で累加。
        // 算定不能（fc 無し・Ec≤0 等）なら to_section の既定値
        // （SRC: N_S_EQ 固定、CFT: 鋼管のみ）のまま。質量用 a_mass は常に幾何断面。
        use squid_n_core::section_shape::SectionShape;
        let composite = sec.shape.as_ref().and_then(|shape| match shape {
            SectionShape::SrcRect { .. } => mat
                .fc
                .is_some()
                .then(|| shape.src_equivalent_props(mat.young, mat.poisson))
                .flatten(),
            SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => mat
                .fc
                .and_then(|fc| shape.cft_equivalent_props(mat.young, mat.poisson, fc)),
            _ => None,
        });

        // SRC で材料から算定できない場合も、軸剛性だけは既定 N_S_EQ の累加を維持する。
        let a_stiff = match (&composite, &sec.shape) {
            (Some(p), _) => p.area_ax,
            (None, Some(shape @ SectionShape::SrcRect { .. })) => shape.calc_axial_stiffness_area(),
            _ => sec.area,
        };
        let (iy, iz, j, as_y, as_z) = match &composite {
            Some(p) => (p.iy, p.iz, p.j, p.as_y, p.as_z),
            None => (sec.iy, sec.iz, sec.j, as_y, as_z),
        };

        // スラブ協力幅による強軸剛性増大（RESP-D 計算編 02「RC大梁」）。
        // RC 矩形梁のみ対象。Model::slab_thickness=0（既定）では 1.0 で無効。
        let iy = if matches!(&sec.shape, Some(SectionShape::RcRect { .. })) {
            iy * slab_stiffness_factor(model, data, sec.width, sec.depth)
        } else {
            iy
        };

        // 壁エレメントモデルの上下大梁の剛性倍率（RESP-D 計算編 02「上下大梁の断面性能」）。
        // 対象は水平材（協力幅判定と同様、勾配 5% までは水平とみなす）かつ、両端節点が
        // 四隅を持つ Wall 要素の節点集合に含まれる（=壁の上辺・下辺の大梁）場合のみ。
        // 倍率は剛性用の値（a, iy, iz, j, as_y, as_z）にのみ乗じ、質量用 a_mass は
        // 幾何断面のまま変更しない。
        let lp = (dx * dx + dy * dy).sqrt();
        let is_horizontal = lp > 1e-9 && dz.abs() <= 0.05 * lp;
        let wall_girder_factor = if is_horizontal && is_wall_top_bottom_girder(model, n0, n1) {
            WALL_GIRDER_STIFF_FACTOR
        } else {
            1.0
        };
        let mut a_stiff = a_stiff * wall_girder_factor;
        let mut iy = iy * wall_girder_factor;
        let mut iz = iz * wall_girder_factor;
        let j = j * wall_girder_factor;
        let mut as_y = as_y * wall_girder_factor;
        let mut as_z = as_z * wall_girder_factor;

        // フレーム内雑壁（耐震壁不成立）の周辺部材への断面性能算入
        // （RESP-D 計算編 02「フレーム内雑壁のモデル化」）。柱（鉛直材）には袖壁を、
        // 梁（水平材）には腰壁/垂壁を、平行軸の定理で剛性用断面性能へ合成する。
        // 対象は不成立壁のみ（成立壁は上下大梁100倍で別途考慮済み・排他）。
        let misc_walls = crate::misc_wall::collect_misc_walls(model);
        if !misc_walls.is_empty() {
            // 不変条件の確認: 壁要素と周辺部材（柱・梁）の ElemId は別空間ではなく
            // モデル全体で一意のはずなので、自部材自身が雑壁として収集される
            // （壁要素IDと自部材IDの衝突）ことはない。
            for w in &misc_walls {
                debug_assert_ne!(
                    w.elem, data.id,
                    "壁要素と周辺部材のIDが衝突している（model 不整合）"
                );
            }
            let is_vertical_member = dz.abs() > (dx.abs() + dy.abs()) * 0.5;

            // 自部材の両端節点集合が節点対 b と一致するか（順序不問）
            let same_pair = |a: [NodeId; 2], b: (NodeId, NodeId)| -> bool {
                (a[0] == b.0 && a[1] == b.1) || (a[0] == b.1 && a[1] == b.0)
            };
            // 平行軸の定理による合成: contrib = (合成断面積 Aw, 部材中心からの
            // 符号付き距離 e, 合成断面の自身回りの断面2次モーメント)。
            // 図心 g = Σ(Aw·e)/(Ac+ΣAw) を求めた上で I を再合成する。
            let compose = |i0: f64, ac: f64, contrib: &[(f64, f64, f64)]| -> f64 {
                let sum_aw: f64 = contrib.iter().map(|c| c.0).sum();
                if sum_aw <= 0.0 {
                    return i0;
                }
                let sum_aw_e: f64 = contrib.iter().map(|c| c.0 * c.1).sum();
                let g = sum_aw_e / (ac + sum_aw);
                let mut i_new = i0 + ac * g * g;
                for &(aw, e, self_i) in contrib {
                    i_new += self_i + aw * (e - g).powi(2);
                }
                i_new
            };

            if is_vertical_member {
                // 柱（鉛直材）: 袖壁の算入。面内せいは近似として断面の大きい方の
                // 辺（sec.depth.max(sec.width)）を用いる。
                let d_col = sec.depth.max(sec.width);
                let ac = a_stiff;
                let mut contrib_y: Vec<(f64, f64, f64)> = Vec::new(); // iz・as_y を増強
                let mut contrib_z: Vec<(f64, f64, f64)> = Vec::new(); // iy・as_z を増強
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    for s in 0..2 {
                        let pair = (wall.bottom_pair[s], wall.top_pair[s]);
                        if !same_pair([n0, n1], pair) {
                            continue;
                        }
                        let lww = (wall.wing_length(s) - d_col / 2.0).clamp(0.0, wall.lw);
                        if lww <= 0.0 {
                            continue;
                        }
                        let Some(pa) = model
                            .nodes
                            .get(wall.bottom_pair[0].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let Some(pb) = model
                            .nodes
                            .get(wall.bottom_pair[1].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let wdx = pb[0] - pa[0];
                        let wdy = pb[1] - pa[1];
                        let wl = (wdx * wdx + wdy * wdy).sqrt();
                        if wl < 1e-9 {
                            continue;
                        }
                        // 壁下辺方向の水平単位ベクトルと柱の局所 ey・ez との内積で
                        // 面内たわみ方向（iz↔as_y か iy↔as_z か）を判定する。
                        let e_wall = [wdx / wl, wdy / wl, 0.0];
                        let dot_ey = (axis.rot[1][0] * e_wall[0]
                            + axis.rot[1][1] * e_wall[1]
                            + axis.rot[1][2] * e_wall[2])
                            .abs();
                        let dot_ez = (axis.rot[2][0] * e_wall[0]
                            + axis.rot[2][1] * e_wall[1]
                            + axis.rot[2][2] * e_wall[2])
                            .abs();

                        let aw = wall.t * lww;
                        let e_i = if s == 0 {
                            -(d_col / 2.0 + lww / 2.0)
                        } else {
                            d_col / 2.0 + lww / 2.0
                        };
                        let self_i = wall.t * lww.powi(3) / 12.0;
                        a_add += aw;
                        if dot_ey >= dot_ez {
                            contrib_y.push((aw, e_i, self_i));
                        } else {
                            contrib_z.push((aw, e_i, self_i));
                        }
                    }
                }

                if !contrib_y.is_empty() {
                    iz = compose(iz, ac, &contrib_y);
                    as_y += contrib_y.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                if !contrib_z.is_empty() {
                    iy = compose(iy, ac, &contrib_z);
                    as_z += contrib_z.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            } else if is_horizontal {
                // 梁（水平材）: 腰壁（下辺の梁に載る壁）・垂壁（上辺の梁から垂れる壁）の
                // 算入。鉛直曲げ（iy・as_z）へ合成する。
                let d_beam = sec.depth;
                let ac = a_stiff;
                let mut contrib: Vec<(f64, f64, f64)> = Vec::new();
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    let bottom = (wall.bottom_pair[0], wall.bottom_pair[1]);
                    let top = (wall.top_pair[0], wall.top_pair[1]);
                    // 下辺の梁なら壁は上に載る（腰壁）、上辺の梁なら壁は下に垂れる（垂壁）。
                    let (matched, hw_raw, sign) = if same_pair([n0, n1], bottom) {
                        (true, wall.strip_height(false), 1.0)
                    } else if same_pair([n0, n1], top) {
                        (true, wall.strip_height(true), -1.0)
                    } else {
                        (false, 0.0, 0.0)
                    };
                    if !matched {
                        continue;
                    }
                    let hw = (hw_raw - d_beam / 2.0).clamp(0.0, wall.h);
                    if hw <= 0.0 {
                        continue;
                    }
                    let aw = wall.t * hw;
                    let e_i = sign * (d_beam / 2.0 + hw / 2.0);
                    let self_i = wall.t * hw.powi(3) / 12.0;
                    a_add += aw;
                    contrib.push((aw, e_i, self_i));
                }

                if !contrib.is_empty() {
                    iy = compose(iy, ac, &contrib);
                    as_z += contrib.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            }
        }

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: a_stiff,
            a_mass: sec.area,
            iy,
            iz,
            j,
            as_y,
            as_z,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            rigid: data.rigid_zone,
            end_cond: data.end_cond,
            eval_sections,
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
        }
    }

    pub fn local_stiffness_raw(&self) -> LocalMat {
        let (e, g, a, iy, iz, jj, l) = (
            self.e,
            self.g,
            self.a,
            self.iy,
            self.iz,
            self.j,
            self.length,
        );
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let phiz = 12.0 * e * iz / (g * self.as_y * l * l);
        let phiy = 12.0 * e * iy / (g * self.as_z * l * l);
        let az = e * iz / ((1.0 + phiz) * l * l * l);
        let ay = e * iy / ((1.0 + phiy) * l * l * l);

        let mut k = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            k.set(i, j, v);
            if i != j {
                k.set(j, i, v);
            }
        };

        s(0, 0, e * a / l);
        s(6, 6, e * a / l);
        s(0, 6, -e * a / l);
        s(3, 3, g * jj / l);
        s(9, 9, g * jj / l);
        s(3, 9, -g * jj / l);

        s(1, 1, 12.0 * az);
        s(7, 7, 12.0 * az);
        s(1, 7, -12.0 * az);
        s(1, 5, 6.0 * az * l);
        s(1, 11, 6.0 * az * l);
        s(5, 7, -6.0 * az * l);
        s(7, 11, -6.0 * az * l);
        s(5, 5, (4.0 + phiz) * az * l * l);
        s(11, 11, (4.0 + phiz) * az * l * l);
        s(5, 11, (2.0 - phiz) * az * l * l);

        s(2, 2, 12.0 * ay);
        s(8, 8, 12.0 * ay);
        s(2, 8, -12.0 * ay);
        s(2, 4, -6.0 * ay * l);
        s(2, 10, -6.0 * ay * l);
        s(4, 8, 6.0 * ay * l);
        s(8, 10, 6.0 * ay * l);
        s(4, 4, (4.0 + phiy) * ay * l * l);
        s(10, 10, (4.0 + phiy) * ay * l * l);
        s(4, 10, (2.0 - phiy) * ay * l * l);

        k
    }

    pub(crate) fn apply_rigid_zone_transform(
        &self,
        k_flex: &LocalMat,
        li: f64,
        lj: f64,
    ) -> LocalMat {
        if li.abs() < 1e-12 && lj.abs() < 1e-12 {
            return LocalMat {
                n: k_flex.n,
                data: k_flex.data.clone(),
            };
        }
        // Tr: 12×12 — flex端自由度(i', j') → 節点自由度(i, j)
        // i' = i を li だけずらし, j' = j を lj だけずらす
        // Tr はほとんど単位行列。i端: ux_i'=ux_i, uy_i'=uy_i-li*rz_i, uz_i'=uz_i+li*ry_i,
        //   rx_i'=rx_i, ry_i'=ry_i, rz_i'=rz_i
        // j端: ux_j'=ux_j, uy_j'=uy_j+lj*rz_j, uz_j'=uz_j-lj*ry_j,
        //   rx_j'=rx_j, ry_j'=ry_j, rz_j'=rz_j
        let mut tr = LocalMat::zeros(12);
        for i in 0..12 {
            tr.set(i, i, 1.0);
        }
        // i端 (index 0..5): uy方向(1) ← rz方向(5) の項
        tr.set(1, 5, -li);
        tr.set(2, 4, li);
        // j端 (index 6..11): uy方向(7) ← rz方向(11) の項
        tr.set(7, 11, lj);
        tr.set(8, 10, -lj);

        // K_node = Tr^T * K_flex * Tr
        let mut tmp = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += k_flex.get(i, k) * tr.get(k, j);
                }
                tmp.set(i, j, s);
            }
        }
        let mut kn = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += tr.get(k, i) * tmp.get(k, j);
                }
                kn.set(i, j, s);
            }
        }
        kn
    }

    /// 端部回転ばねを「外部回転＋内部回転」の 18 自由度で表し、
    /// 静縮約で 12×12（節点自由度のみ）に戻す。
    /// 18 並び: [外部 0..11（節点 ux,uy,uz,rx,ry,rz ×2）, 内部 12..17（要素端 rx,ry,rz ×2）]
    fn condense_end_springs(&self, k_elem: &LocalMat) -> LocalMat {
        // 18×18 を組む
        let n = 18;
        let mut k = vec![0.0; n * n];

        // 要素剛性: 並進は外部 DOF、回転は内部 DOF へ配置
        let map18 = |i: usize| -> usize {
            match i {
                0..=2 => i,
                3..=5 => i + 9,
                6..=8 => i,
                9..=11 => i + 6,
                _ => i,
            }
        };
        for i in 0..12 {
            for j in 0..12 {
                k[map18(i) * n + map18(j)] = k_elem.get(i, j);
            }
        }

        // 回転ばね: 外部回転 ↔ 内部回転
        // 剛接ペナルティは「部材回転剛性 E·I/L のスケールに対する倍率」で与える。
        // 係数 1e8 なら剛性比 ~1e8（剛接を 8 桁の精度で再現＝結果への影響 ~1e-8<1e-6）
        // でありながら、静縮約 K*=Kaa−Kab·Kbb⁻¹·Kba の丸め誤差（~ペナルティ·eps）が
        // 他剛性成分を下回るため、現実的な大断面（iz≥1e7）でも全体 K が
        // 非正定値化しない。1e12 だと iz が大きいとき誤差が並進剛性を超えて破綻する。
        let rot_scale = self.e * self.iz.max(self.iy) / self.length.max(1.0);
        let spring_stiffness = |cond: &EndCondition| -> f64 {
            match cond {
                EndCondition::Fixed => 1e8 * rot_scale,
                EndCondition::Pinned => 0.0,
                EndCondition::SemiRigid { k_theta } => *k_theta,
            }
        };

        let ext_rot = [3usize, 4, 5, 9, 10, 11];
        let int_rot = [12usize, 13, 14, 15, 16, 17];
        for (idx, &er) in ext_rot.iter().enumerate() {
            let ir = int_rot[idx];
            let kspring = if idx < 3 {
                spring_stiffness(&self.end_cond[0])
            } else {
                spring_stiffness(&self.end_cond[1])
            };
            k[er * n + er] += kspring;
            k[ir * n + ir] += kspring;
            k[er * n + ir] -= kspring;
            k[ir * n + er] -= kspring;
        }

        // 内部 DOF (12..17) を静縮約
        let na = 12;
        let nb = 6;
        let mut kaa = vec![0.0; na * na];
        let mut kab = vec![0.0; na * nb];
        let mut kba = vec![0.0; nb * na];
        let mut kbb = vec![0.0; nb * nb];

        for i in 0..na {
            for j in 0..na {
                kaa[i * na + j] = k[i * n + j];
            }
            for j in 0..nb {
                kab[i * nb + j] = k[i * n + (na + j)];
                kba[j * na + i] = k[(na + j) * n + i];
            }
        }
        for i in 0..nb {
            for j in 0..nb {
                kbb[i * nb + j] = k[(na + i) * n + (na + j)];
            }
        }

        let kbb_inv = invert_small(&kbb, nb);

        // kab_kbbinv = Kab * Kbb^-1
        let mut kab_kbbinv = vec![0.0; na * nb];
        for i in 0..na {
            for j in 0..nb {
                let mut s = 0.0;
                for l in 0..nb {
                    s += kab[i * nb + l] * kbb_inv[l * nb + j];
                }
                kab_kbbinv[i * nb + j] = s;
            }
        }

        let mut kstar = LocalMat::zeros(na);
        for i in 0..na {
            for j in 0..na {
                let mut s = kaa[i * na + j];
                for l in 0..nb {
                    s -= kab_kbbinv[i * nb + l] * kba[l * na + j];
                }
                kstar.set(i, j, s);
            }
        }
        kstar
    }

    pub fn local_stiffness(&self) -> LocalMat {
        let l_flex = self.length - self.rigid.length_i - self.rigid.length_j;
        let k_raw = if l_flex > 1e-12 {
            let mut beam = BeamElement {
                length: l_flex,
                ..BeamElement {
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    ..self.clone()
                }
            };
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
            beam.local_stiffness_raw()
        } else {
            LocalMat::zeros(12)
        };

        // 剛域を持たない可とう部で端部ばね静縮約 → 12×12
        let k_end = self.condense_end_springs(&k_raw);

        // 剛域変換で節点自由度へ
        let li = self.rigid.length_i;
        let lj = self.rigid.length_j;
        self.apply_rigid_zone_transform(&k_end, li, lj)
    }

    pub fn recover_forces(&self, u_elem_global: &[f64; 12]) -> MemberForces {
        let u_local = self.axis.rotate_to_local(u_elem_global);
        let k_local = self.local_stiffness();
        // f_local = K_local * u_local (in local coords, at node ends)
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        // N, Qy, Qz, Mx, My, Mz at i-end: f_local[0], f_local[1], f_local[2], f_local[3], f_local[4], f_local[5]
        // j-end: f_local[6], f_local[7], f_local[8], f_local[9], f_local[10], f_local[11]

        let mut at = Vec::new();
        for &xi in &self.eval_sections {
            // 軸力 N は部材内力（引張正）。スパン内軸方向荷重が無い限り一定で、
            // i 端側は節点力 f_local[0]（引張時に -N）、j 端側は f_local[6]（+N）。
            // 旧実装の f0·(1-ξ)+f6·ξ は両端で符号が逆の節点力を線形補間しており、
            // 中央で N=0 となる誤りだったため、せん断と同じ端別採用に修正。
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = -f_local[0];
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = f_local[3];
                let my = f_local[4] - f_local[2] * xi * self.length;
                let mz = f_local[5] + f_local[1] * xi * self.length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[6];
                let qy = -f_local[7];
                let qz = -f_local[8];
                let mx = f_local[9];
                let my = f_local[10] - f_local[8] * (1.0 - xi) * self.length;
                let mz = f_local[11] + f_local[7] * (1.0 - xi) * self.length;
                (n, qy, qz, mx, my, mz)
            };
            at.push((xi, [n, qy, qz, mx, my, mz]));
        }

        MemberForces { at }
    }
}

pub(crate) fn invert_small(a: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * n * 2];
    for i in 0..n {
        for j in 0..n {
            aug[i * (2 * n) + j] = a[i * n + j];
        }
        aug[i * (2 * n) + n + i] = 1.0;
    }
    for col in 0..n {
        let mut pivot = aug[col * (2 * n) + col];
        if pivot.abs() < 1e-15 {
            pivot = 1.0;
        }
        for j in 0..2 * n {
            aug[col * (2 * n) + j] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row * (2 * n) + col];
                for j in 0..2 * n {
                    aug[row * (2 * n) + j] -= factor * aug[col * (2 * n) + j];
                }
            }
        }
    }
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * (2 * n) + n + j];
        }
    }
    inv
}

/// 部材の構造種別（RESP-D マニュアル「剛域の計算」の RC/SRC 系・S 系区分）。
/// 剛域長の算定式（後述 `auto_rigid_zones`）を部材種別で切り替えるための分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MemberKind {
    /// RC・SRC 系（RC 造柱・梁・耐震壁、SRC 造柱・梁）。
    RcSrc,
    /// S・CFT 系（マニュアル「柱がＣＦＴの場合についても同様」よりＣＦＴはＳ扱い）。
    Steel,
}

/// 要素の構造種別を判定する。
///
/// `Section.shape` があれば形状で判定する（RC/SRC 形状 → RcSrc、鋼材・CFT 形状 → Steel）。
/// `shape` が無い（カタログ数値直入力等）場合は材料で判定する: `Material.fc`（コンクリート
/// 設計基準強度）があれば RcSrc、`fy`（降伏応力）のみあれば Steel。どちらも無い場合は
/// 判定材料が無いため RcSrc 扱い（剛域式を変えない＝従来挙動を維持する既定）。
fn member_kind(model: &Model, e: &squid_n_core::model::ElementData) -> MemberKind {
    use squid_n_core::section_shape::SectionShape;

    let sec = e.section.and_then(|sid| model.sections.get(sid.index()));
    if let Some(shape) = sec.and_then(|s| s.shape.as_ref()) {
        return match shape {
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. }
            | SectionShape::SrcRect { .. } => MemberKind::RcSrc,
            SectionShape::SteelH { .. }
            | SectionShape::SteelBox { .. }
            | SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
            | SectionShape::SteelPipe { .. }
            | SectionShape::CftBox { .. }
            | SectionShape::CftPipe { .. } => MemberKind::Steel,
        };
    }

    let mat = e.material.and_then(|mid| model.materials.get(mid.index()));
    if let Some(mat) = mat {
        if mat.fc.is_some() {
            return MemberKind::RcSrc;
        }
        if mat.fy.is_some() {
            return MemberKind::Steel;
        }
    }
    MemberKind::RcSrc
}

pub fn auto_rigid_zones(
    model: &squid_n_core::model::Model,
    elem_id: squid_n_core::ids::ElemId,
    rule: &RigidZoneRule,
) -> RigidZone {
    let elem = match model.elements.iter().find(|e| e.id == elem_id) {
        Some(e) => e,
        None => {
            return RigidZone {
                reduction: rule.reduction,
                ..Default::default()
            }
        }
    };

    let nodes = &elem.nodes;
    if nodes.len() < 2 {
        return RigidZone {
            reduction: rule.reduction,
            ..Default::default()
        };
    }

    let self_sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    let d_self = self_sec.map(|s| s.depth).unwrap_or(0.0);

    // 節点 → 接続要素のマップ（直交せい探索の対象は柱・梁＝Beam 要素のみ。
    // 耐震壁・シェル等が混入すると「耐震壁周辺の柱・梁の剛域は考慮しません」
    // というマニュアル規定に反し、壁の名目せい等が誤って直交材に紛れ込む）。
    let mut node_to_elems: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for (ei, e) in model.elements.iter().enumerate() {
        if e.nodes.len() >= 2 && matches!(e.kind, squid_n_core::model::ElementKind::Beam) {
            for n in &e.nodes {
                node_to_elems.entry(n.index()).or_default().push(ei);
            }
        }
    }

    fn elem_axis(model: &Model, e: &squid_n_core::model::ElementData) -> [f64; 3] {
        if e.nodes.len() < 2 {
            return [0.0, 0.0, 0.0];
        }
        let p0 = model.nodes[e.nodes[0].index()].coord;
        let p1 = model.nodes[e.nodes[1].index()].coord;
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        if l < 1e-12 {
            [0.0, 0.0, 0.0]
        } else {
            [dx / l, dy / l, dz / l]
        }
    }

    // `only_rc_src` を true にすると、RC/SRC 系の直交 Beam 要素だけを対象に最大せいを探す
    // （剛域長 λ 用。マニュアル「仕口部に接続する柱(梁)がすべてＳの場合、剛域長さは0」
    // ＝ S 系直交材は無視することで自然に d_max=0 となる）。false なら種別を問わず全直交
    // Beam 要素が対象（危険断面位置 face 用。§6.2.3 は幾何量であり種別を区別しない）。
    fn max_orth_depth(
        model: &Model,
        node_idx: usize,
        target_axis: [f64; 3],
        target_elem_idx: usize,
        node_to_elems: &std::collections::HashMap<usize, Vec<usize>>,
        only_rc_src: bool,
    ) -> f64 {
        let mut d_max = 0.0;
        if let Some(elems) = node_to_elems.get(&node_idx) {
            for &ei in elems {
                if ei == target_elem_idx {
                    continue;
                }
                let e = &model.elements[ei];
                if e.nodes.len() < 2 {
                    continue;
                }
                if only_rc_src && member_kind(model, e) != MemberKind::RcSrc {
                    continue;
                }
                let axis = elem_axis(model, e);
                let dot = (axis[0] * target_axis[0]
                    + axis[1] * target_axis[1]
                    + axis[2] * target_axis[2])
                    .abs();
                if dot < 0.707 {
                    // 概ね直交（45°以上）
                    if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                        if sec.depth > d_max {
                            d_max = sec.depth;
                        }
                    }
                }
            }
        }
        d_max
    }

    let target_axis = elem_axis(model, elem);
    let target_elem_idx = model
        .elements
        .iter()
        .position(|e| e.id == elem_id)
        .unwrap_or(0);

    // face 用: 種別を問わない直交 Beam 要素の最大せい（従来どおりの幾何量）。
    let d_orth_face_i = max_orth_depth(
        model,
        nodes[0].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        false,
    );
    let d_orth_face_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        false,
    );
    // λ 用: RC/SRC 系の直交 Beam 要素だけの最大せい。
    let d_orth_rc_i = max_orth_depth(
        model,
        nodes[0].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        true,
    );
    let d_orth_rc_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
        true,
    );

    // 剛域長 λ は自部材の構造種別で式を切り替える（マニュアル「剛域の計算」）。
    // - RC/SRC 造: λ = reduction·(D_orth_rc/2 − D_self/4)（従来式。負は 0 クランプ）。
    // - Ｓ・ＣＦＴ造: λ = D_orth_rc/2（D_self/4 の控除なし・reduction も掛けない。
    //   RC/SRC 大梁のうち最大せいの梁フェイスまでの長さ＝仕口部を除いた長さ）。
    //   直交する RC/SRC 系の梁（柱）が無ければ D_orth_rc=0 なので λ=0
    //   （マニュアル「Ｓ造の剛域…剛域長さは0とします」）。
    let self_kind = member_kind(model, elem);
    let lambda = |d_orth_rc: f64| -> f64 {
        match self_kind {
            MemberKind::RcSrc => {
                let v = rule.reduction * (d_orth_rc / 2.0 - d_self / 4.0);
                if v < 0.0 {
                    0.0
                } else {
                    v
                }
            }
            MemberKind::Steel => d_orth_rc / 2.0,
        }
    };
    // フェイス距離 = D_orth/2 は剛性用剛域の低減率（慣用調整）と無関係な幾何量なので
    // reduction を掛けない（設計書 §6.2.1「設計位置との区別」）。
    // λ が負→0 にクランプされる場合でも face はそのまま D_orth/2 を保持する。
    let face = |d_orth: f64| -> f64 { d_orth / 2.0 };

    RigidZone {
        length_i: lambda(d_orth_rc_i),
        length_j: lambda(d_orth_rc_j),
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        reduction: rule.reduction,
        face_i: face(d_orth_face_i),
        face_j: face(d_orth_face_j),
    }
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
    }
    // フェイス距離は剛域長の Manual/Auto フラグとは独立な幾何量（接続関係から
    // 一意に決まる §6.2.1）。手動で剛域長を保護しているときも、モデルの接続情報
    // が変われば危険断面位置は追従すべきなので、Manual 保護の対象外として常に
    // 再算定値で更新する。
    zone.face_i = recomputed.face_i;
    zone.face_j = recomputed.face_j;
}

/// モデル全要素の剛域を自動算定し、`ElementData::rigid_zone` を更新する前処理。
/// `source` が `Auto` の端のみ更新し、`Manual` 端は保護する（設計書 §6.2.1）。
/// 解析前に1回呼ぶことで剛域が組立に反映される（既定では剛域長 0 のまま
/// ＝呼ばなければ従来挙動。明示的に有効化する設計）。
///
/// `auto_rigid_zones` を要素ごとに呼ぶと隣接マップ構築が O(E²) になるため、
/// ここでは梁要素の集合に対し各端の剛域を算定して一括反映する。
pub fn apply_auto_rigid_zones(model: &mut Model, rule: &RigidZoneRule) {
    // 要素 id ごとに算定（auto_rigid_zones は内部で隣接を構築するが、
    // 呼び出しは「解析前1回」を想定。大規模最適化は将来）。
    let recomputed: Vec<(usize, RigidZone)> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e.kind, squid_n_core::model::ElementKind::Beam))
        .map(|(i, e)| (i, auto_rigid_zones(model, e.id, rule)))
        .collect();

    for (i, rz) in recomputed {
        let zone = &mut model.elements[i].rigid_zone;
        recompute_auto_zones(zone, &rz);
        // reduction も Auto 算定値を反映（手動端の length は保持済み）。
        zone.reduction = rz.reduction;
    }
}

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        // 要素ローカルの 12×12 を全体系へ回す（K_global = Rᵀ K_local R）。
        // ElementBehavior::tangent_stiffness は全体系を返す契約（シェルと同じ）。
        // これを欠くと、ローカル系とグローバル系が一致しない部材（鉛直柱・
        // 任意方向材・非対称断面 iy≠iz）で組立 K が誤る。
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let l = self.length;
        let c = n / l;
        let mut kg = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        // xy面（uy=1,rz=5 / uy_j=7,rz_j=11）
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        // xz面（uz=2,ry=4 / uz_j=8,ry_j=10）§4.1 規約で並進-回転結合項の符号が逆（ry の向き）
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
        // 幾何剛性もグローバル系へ回転（P-Δ を組立系で正しく加算するため）
        self.axis.to_global(&kg)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // committed_disp はグローバル系で蓄積されるため、グローバル剛性で内力を評価する。
        // f_global = (R^T·K_local·R)·u_global
        let k = self.axis.to_global(&self.local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.committed_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..12 {
            if commit {
                self.committed_disp[i] += du.data[i];
            }
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
            }
            MassOption::Consistent => {
                let c1 = m / 6.0;
                let c2 = m / 420.0;
                let l = self.length;
                let l2 = l * l;
                // Axial (Ux):  indices 0,6
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                // Torsion (Rx): indices 3,9
                let ct = self.density * self.j * l / 6.0;
                mm.set(3, 3, 2.0 * ct);
                mm.set(3, 9, 1.0 * ct);
                mm.set(9, 3, 1.0 * ct);
                mm.set(9, 9, 2.0 * ct);
                // Bending: Hermite 梁の一貫質量（4x4 ブロック）。
                // DOF は連続ではないためインデックス配列で指定する。
                //   Uy-Rz 面: [Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11]
                //   Uz-Ry 面: [Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10]（回転符号は逆）
                let b4 = |mm: &mut LocalMat, idx: [usize; 4], sign: f64| {
                    let [d0, r0, d1, r1] = idx;
                    // 並進-並進
                    mm.set(d0, d0, 156.0 * c2);
                    mm.set(d0, d1, 54.0 * c2);
                    mm.set(d1, d0, 54.0 * c2);
                    mm.set(d1, d1, 156.0 * c2);
                    // 並進-回転
                    mm.set(d0, r0, 22.0 * l * c2 * sign);
                    mm.set(r0, d0, 22.0 * l * c2 * sign);
                    mm.set(d0, r1, -13.0 * l * c2 * sign);
                    mm.set(r1, d0, -13.0 * l * c2 * sign);
                    mm.set(d1, r0, 13.0 * l * c2 * sign);
                    mm.set(r0, d1, 13.0 * l * c2 * sign);
                    mm.set(d1, r1, -22.0 * l * c2 * sign);
                    mm.set(r1, d1, -22.0 * l * c2 * sign);
                    // 回転-回転
                    mm.set(r0, r0, 4.0 * l2 * c2);
                    mm.set(r0, r1, -3.0 * l2 * c2);
                    mm.set(r1, r0, -3.0 * l2 * c2);
                    mm.set(r1, r1, 4.0 * l2 * c2);
                };
                b4(&mut mm, [1, 5, 7, 11], 1.0);
                b4(&mut mm, [2, 4, 8, 10], -1.0);
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{ElementData, ElementKind, LocalAxis, Material, Node, Section};

    fn make_test_beam() -> BeamElement {
        BeamElement {
            id: ElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 80000.0,
            a_mass: 80000.0,
            iy: 1.0666667e9,
            iz: 1.0666667e9,
            j: 0.0,
            as_y: 66666.67,
            as_z: 66666.67,
            length: 3000.0,
            density: 0.0,
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
        }
    }

    /// SRC/CFT の複合換算が要素生成へ配線されていること（RESP-D 計算編 02）。
    #[test]
    fn test_beam_new_src_cft_composite_props() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{MaterialId, SectionId};
        use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Model};
        use squid_n_core::section_shape::{
            BarSet, RcRebar, SectionShape, ShearBar, E_STEEL, N_S_EQ,
        };

        let src_shape = SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
            steel_grade: "SN400B".into(),
        };
        let cft_shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };

        let mut model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            sections: vec![
                src_shape.to_section(SectionId(0), "SRC-600".into()),
                cft_shape.to_section(SectionId(1), "CFT-400".into()),
            ],
            materials: vec![
                Material {
                    concrete_class: Default::default(),
                    id: MaterialId(0),
                    name: "FC24".into(),
                    young: 23000.0,
                    poisson: 0.2,
                    density: 2.4e-9,
                    shear: None,
                    fc: Some(24.0),
                    fy: None,
                },
                Material {
                    concrete_class: Default::default(),
                    id: MaterialId(1),
                    name: "BCR295(充填FC36)".into(),
                    young: 205000.0,
                    poisson: 0.3,
                    density: 7.85e-9,
                    shear: None,
                    fc: Some(36.0),
                    fy: Some(295.0),
                },
            ],
            ..Default::default()
        };
        let make_elem = |sec: u32, mat: u32| ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(squid_n_core::ids::SectionId(sec)),
            material: Some(squid_n_core::ids::MaterialId(mat)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };

        // SRC + コンクリート材料: ns=Es/Ec による等価断面性能
        let src_beam = BeamElement::new(&make_elem(0, 0), &model);
        let p = src_shape.src_equivalent_props(23000.0, 0.2).unwrap();
        assert!((src_beam.a - p.area_ax).abs() < 1e-6);
        assert!((src_beam.iy - p.iy).abs() / p.iy < 1e-12);
        assert!((src_beam.j - p.j).abs() / p.j < 1e-12);
        assert!((src_beam.as_z - p.as_z).abs() < 1e-6);
        // ns=205000/23000≈8.91 は既定 N_S_EQ=15 と異なる値になること
        let ns = E_STEEL / 23000.0;
        assert!((ns - N_S_EQ).abs() > 1.0);
        // 質量用断面積は幾何断面(コンクリート全断面)のまま
        assert!((src_beam.a_mass - 360_000.0).abs() < 1e-9);

        // CFT + 鋼材料(fc=充填強度): 充填コンクリートの 1/n 換算累加
        let cft_beam = BeamElement::new(&make_elem(1, 1), &model);
        let pc = cft_shape.cft_equivalent_props(205000.0, 0.3, 36.0).unwrap();
        assert!((cft_beam.a - pc.area_ax).abs() < 1e-6);
        assert!((cft_beam.iy - pc.iy).abs() / pc.iy < 1e-12);
        assert!((cft_beam.j - pc.j).abs() / pc.j < 1e-12);

        // SRC + fc の無い材料: 既定 N_S_EQ の軸剛性累加へフォールバック
        model.materials[0].fc = None;
        let src_fallback = BeamElement::new(&make_elem(0, 0), &model);
        assert!((src_fallback.a - src_shape.calc_axial_stiffness_area()).abs() < 1e-6);
        assert!((src_fallback.iy - model.sections[0].iy).abs() < 1e-6);
    }

    /// スラブ協力幅による強軸剛性増大（RESP-D 計算編 02「RC大梁」・RC規準8条）。
    #[test]
    fn test_beam_new_slab_cooperation_width_amplifies_iy() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{MaterialId, SectionId, SlabId};
        use squid_n_core::model::{
            DistributionMethod, EndCondition, ForceRegime, LocalAxis, Model, Slab,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let shape = SectionShape::RcRect {
            b: 300.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        let mut model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 3000.0]),
                make_node(1, [6000.0, 0.0, 3000.0]),
                make_node(2, [6000.0, 2500.0, 3000.0]),
                make_node(3, [0.0, 2500.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "RC-300x600".into())],
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
            slabs: vec![Slab {
                id: SlabId(0),
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
                kind: Default::default(),
                one_way: None,
                edge_supported: None,
            }],
            slab_thickness: 150.0,
            ..Default::default()
        };
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
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
        };

        // 期待値: a=2500 < l/2=3000 → ba=(0.5−0.6·2500/6000)·2500=625(片側のみ)
        let (b, d, t, l) = (300.0_f64, 600.0_f64, 150.0_f64, 6000.0_f64);
        let ba = (0.5 - 0.6 * 2500.0 / l) * 2500.0;
        assert!((ba - 625.0).abs() < 1e-9);
        let bf = b + ba;
        let (aw, af) = (b * d, (bf - b) * t);
        let g = (aw * d / 2.0 + af * (d - t / 2.0)) / (aw + af);
        let i0 = b * d.powi(3) / 12.0;
        let ie = i0
            + aw * (g - d / 2.0).powi(2)
            + (bf - b) * t.powi(3) / 12.0
            + af * (d - t / 2.0 - g).powi(2);

        let beam = BeamElement::new(&elem, &model);
        assert!(
            (beam.iy - ie).abs() / ie < 1e-12,
            "iy={} ie={}",
            beam.iy,
            ie
        );
        assert!(beam.iy / i0 > 1.3, "増大率が小さすぎる: {}", beam.iy / i0);
        // 弱軸は増大しない
        assert!((beam.iz - model.sections[0].iz).abs() < 1e-9);

        // 床厚 0(既定)では従来どおり
        model.slab_thickness = 0.0;
        let beam0 = BeamElement::new(&elem, &model);
        assert!((beam0.iy - i0).abs() < 1e-9);
    }

    #[test]
    fn test_local_stiffness_symmetric() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                    "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                    k.get(i, j),
                    k.get(j, i)
                );
            }
        }
    }

    #[test]
    fn test_phi_zero_converges_to_bernoulli() {
        // As → ∞ => phi → 0 => Timoshenko → Bernoulli
        let mut beam = make_test_beam();
        beam.as_y = 1e30;
        beam.as_z = 1e30;
        let k_timo = beam.local_stiffness_raw();

        // Bernoulli reference: same beam with phi=0
        let e = beam.e;
        let iz = beam.iz;
        let iy = beam.iy;
        let a = beam.a;
        let l = beam.length;
        let g = beam.g;
        let jj = beam.j;

        let az = e * iz / (l * l * l);
        let ay = e * iy / (l * l * l);

        for i in 0..12 {
            for j in 0..12 {
                let norm_pair = if i <= j { (i, j) } else { (j, i) };
                let bernoulli = match norm_pair {
                    (0, 0) | (6, 6) => e * a / l,
                    (0, 6) => -e * a / l,
                    (3, 3) | (9, 9) => g * jj / l,
                    (3, 9) => -g * jj / l,
                    (1, 1) | (7, 7) => 12.0 * az,
                    (1, 7) => -12.0 * az,
                    (1, 5) | (1, 11) => 6.0 * az * l,
                    (5, 7) | (7, 11) => -6.0 * az * l,
                    (5, 5) | (11, 11) => 4.0 * az * l * l,
                    (5, 11) => 2.0 * az * l * l,
                    (2, 2) | (8, 8) => 12.0 * ay,
                    (2, 8) => -12.0 * ay,
                    (2, 4) | (2, 10) => -6.0 * ay * l,
                    (4, 8) | (8, 10) => 6.0 * ay * l,
                    (4, 4) | (10, 10) => 4.0 * ay * l * l,
                    (4, 10) => 2.0 * ay * l * l,
                    _ => 0.0,
                };
                let timo = k_timo.get(i, j);
                assert!(
                    (timo - bernoulli).abs() < 1e-6,
                    "K[{i}][{j}]: timo={timo}, bernoulli={bernoulli}"
                );
            }
        }
    }

    #[test]
    fn test_beam_axial_stiffness() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        let ea_l = beam.e * beam.a / beam.length;
        assert!((k.get(0, 0) - ea_l).abs() < 1e-9);
        assert!((k.get(0, 6) + ea_l).abs() < 1e-9);
        assert!((k.get(6, 0) + ea_l).abs() < 1e-9);
        assert!((k.get(6, 6) - ea_l).abs() < 1e-9);
    }

    #[test]
    fn test_beam_torsion_stiffness() {
        let beam = make_test_beam();
        let k = beam.local_stiffness_raw();
        let gj_l = beam.g * beam.j / beam.length;
        assert!((k.get(3, 3) - gj_l).abs() < 1e-9);
        assert!((k.get(9, 9) - gj_l).abs() < 1e-9);
        assert!((k.get(3, 9) + gj_l).abs() < 1e-9);
    }

    #[test]
    fn test_pinned_end_releases_moment() {
        // i端をピンにすると、i端回転行/列がほぼゼロになり剛性が低下
        let mut beam = make_test_beam();
        beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
        let k = beam.local_stiffness();
        // i端の My, Mz 対角成分が Fixed 時より大幅に小さい
        let k_fixed = make_test_beam().local_stiffness();
        assert!(k.get(4, 4) < k_fixed.get(4, 4) * 1e-6);
        assert!(k.get(5, 5) < k_fixed.get(5, 5) * 1e-6);
    }

    #[test]
    fn test_auto_rigid_zone_standard_formula() {
        // 柱せい 600, 梁せい 700 の T 字接合
        // 梁端 λ = 柱せい/2 - 梁せい/4 = 300 - 175 = 125
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        let col_sec = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 600.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let beam_sec = Section {
            id: SectionId(1),
            name: "beam".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 700.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };

        let model = Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(1)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![col_sec, beam_sec],
            materials: vec![mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert!((zone.length_i - 125.0).abs() < 1e-9);
        // フェイス距離 face_i = D_orth/2 = 柱せい/2 = 300（低減率は掛けない）。
        assert!((zone.face_i - 300.0).abs() < 1e-9, "face_i={}", zone.face_i);
    }

    /// apply_auto_rigid_zones が ElementData::rigid_zone に反映され、
    /// Manual 端が保護されることを確認する（剛域がモデル→解析へ接続されたこと）。
    #[test]
    fn test_apply_auto_rigid_zones_and_manual_protection() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{ElementKind, ZoneSource};

        let mk_sec = |id: u32, depth: f64| Section {
            id: SectionId(id),
            name: String::new(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mk_node = |id: u32, c: [f64; 3]| Node {
            id: NodeId(id),
            coord: c,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let mk_beam = |id: u32, a: u32, b: u32, sec: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };

        let mut model = Model {
            nodes: vec![
                mk_node(0, [0.0, 0.0, 0.0]),
                mk_node(1, [0.0, 0.0, 3000.0]),
                mk_node(2, [4000.0, 0.0, 3000.0]),
            ],
            elements: vec![mk_beam(0, 0, 1, 0), mk_beam(1, 1, 2, 1)], // 柱(せい600)・梁(せい700)
            sections: vec![mk_sec(0, 600.0), mk_sec(1, 700.0)],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: String::new(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        };

        // 既定では剛域長 0（未適用）。
        assert_eq!(model.elements[1].rigid_zone.length_i, 0.0);

        apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
        // 梁端（接合部側）に λ = 柱せい/2 − 梁せい/4 = 300 − 175 = 125 が入る。
        assert!(
            (model.elements[1].rigid_zone.length_i - 125.0).abs() < 1e-9,
            "λ_i={}",
            model.elements[1].rigid_zone.length_i
        );

        // 手動端は再適用で保護される。
        model.elements[1].rigid_zone.source_i = ZoneSource::Manual;
        model.elements[1].rigid_zone.length_i = 999.0;
        model.elements[1].rigid_zone.face_i = 0.0;
        apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
        assert_eq!(
            model.elements[1].rigid_zone.length_i, 999.0,
            "Manual 端が上書きされた"
        );
        // face_i は剛域長の Manual/Auto フラグとは無関係な幾何量なので、
        // Manual 端でも常に再算定される（設計書 §6.2.1）。
        assert!(
            (model.elements[1].rigid_zone.face_i - 300.0).abs() < 1e-9,
            "Manual 端でも face_i は再算定されるべき: face_i={}",
            model.elements[1].rigid_zone.face_i
        );
    }

    /// 危険断面位置（§6.2.3）: face_i/face_j から評価断面リストを算定する。
    /// face=0（直交材なし）の端では従来どおり [0.0, 0.5, 1.0] と完全一致する。
    #[test]
    fn test_eval_sections_from_face_distance() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{ElementKind, RigidZone};

        let sec = Section {
            id: SectionId(0),
            name: String::new(),
            area: 100.0,
            iy: 1.0e6,
            iz: 1.0e6,
            j: 1.0e6,
            depth: 300.0,
            width: 300.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: String::new(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let model = Model {
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
                    coord: [4000.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: RigidZone {
                    face_i: 300.0,
                    face_j: 250.0,
                    ..Default::default()
                },
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![sec],
            materials: vec![mat],
            ..Default::default()
        };

        let beam = BeamElement::new(&model.elements[0], &model);
        let expected = [0.0, 0.075, 0.5, 0.9375, 1.0];
        assert_eq!(beam.eval_sections.len(), expected.len());
        for (a, b) in beam.eval_sections.iter().zip(expected.iter()) {
            assert!(
                (a - b).abs() < 1e-9,
                "eval_sections={:?}",
                beam.eval_sections
            );
        }

        // face=0 の端では従来どおり [0.0, 0.5, 1.0] と完全一致。
        let mut model_zero = model.clone();
        model_zero.elements[0].rigid_zone = RigidZone::default();
        let beam_zero = BeamElement::new(&model_zero.elements[0], &model_zero);
        assert_eq!(beam_zero.eval_sections, vec![0.0, 0.5, 1.0]);
    }

    /// 剛域算定用の RC 配筋（本数・径は最小限のダミー値。断面性能の絶対値は無関係）。
    fn simple_rc_rebar() -> squid_n_core::section_shape::RcRebar {
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 16.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 16.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        }
    }

    /// S造仕口（柱・梁とも鋼材形状）: 直交する RC/SRC 系の柱（梁）が存在しないため、
    /// マニュアル「仕口部に接続する柱(梁)がすべてＳの場合、剛域長さは0」どおり λ=0 になる。
    #[test]
    fn test_auto_rigid_zone_steel_joint_is_zero() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::ElementKind;
        use squid_n_core::section_shape::SectionShape;

        let col_sec = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        }
        .to_section(SectionId(0), "col-H400".to_string());
        let beam_sec = SectionShape::SteelH {
            height: 500.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 16.0,
        }
        .to_section(SectionId(1), "beam-H500".to_string());
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        };

        let model = Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(1)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![col_sec, beam_sec],
            materials: vec![mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert_eq!(
            zone.length_i, 0.0,
            "S造仕口の剛域長は0のはず: length_i={}",
            zone.length_i
        );
    }

    /// S梁 + RC柱: マニュアル「Ｓ・ＣＦＴ柱の場合…ＲＣ・ＳＲＣ大梁のうち最大せいの梁
    /// フェイスまでの長さ」どおり、λ = 柱せい/2（D/4控除なし・reductionも掛けない）。
    #[test]
    fn test_auto_rigid_zone_steel_beam_rc_column() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::ElementKind;
        use squid_n_core::section_shape::SectionShape;

        let col_sec = SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: simple_rc_rebar(),
        }
        .to_section(SectionId(0), "col-RC600".to_string());
        let beam_sec = SectionShape::SteelH {
            height: 500.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 16.0,
        }
        .to_section(SectionId(1), "beam-H500".to_string());
        let rc_mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "concrete".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let s_mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(1),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        };

        let model = Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(1)),
                    material: Some(MaterialId(1)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![col_sec, beam_sec],
            materials: vec![rc_mat, s_mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert!(
            (zone.length_i - 300.0).abs() < 1e-9,
            "S梁+RC柱: λ_i={} (期待値=柱せい/2=300)",
            zone.length_i
        );
    }

    /// RC梁 + S柱のみ: 直交する RC/SRC 系の柱が無いため D_orth_rc=0 となり、
    /// 従来式 λ=reduction·(0/2−梁せい/4) は負となって 0 にクランプされる。
    #[test]
    fn test_auto_rigid_zone_rc_beam_steel_column_only_is_zero() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::ElementKind;
        use squid_n_core::section_shape::SectionShape;

        let col_sec = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        }
        .to_section(SectionId(0), "col-H400".to_string());
        let beam_sec = SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: simple_rc_rebar(),
        }
        .to_section(SectionId(1), "beam-RC600".to_string());
        let s_mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        };
        let rc_mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(1),
            name: "concrete".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };

        let model = Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(1)),
                    material: Some(MaterialId(1)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![col_sec, beam_sec],
            materials: vec![s_mat, rc_mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert_eq!(
            zone.length_i, 0.0,
            "RC梁+S柱のみ: 剛域長は0のはず（RC/SRC直交材が無い）。length_i={}",
            zone.length_i
        );
    }

    /// 耐震壁要素（ElementKind::Wall）が節点に接続していても、直交せい探索の対象は
    /// Beam 要素のみなので結果に影響しない（マニュアル「耐震壁周辺の柱・梁の剛域は
    /// 考慮しません」）。壁を追加しても標準ケース（柱600・梁700 → λ=125）と同じ結果。
    #[test]
    fn test_auto_rigid_zone_wall_does_not_affect_orthogonal_search() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        let col_sec = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 600.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let beam_sec = Section {
            id: SectionId(1),
            name: "beam".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 700.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        // 壁のせい（名目値）を柱・梁より大きくし、混入すれば結果が変わることを検証可能にする。
        let wall_sec = Section {
            id: SectionId(2),
            name: "wall".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 1000.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };

        let model = Model {
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
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [4000.0, 0.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(3),
                    coord: [0.0, 4000.0, 3000.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(1)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                // 節点1に接続する壁要素（節点1-3）。梁と直交するがWall kindなので無視される。
                ElementData {
                    id: ElemId(2),
                    kind: ElementKind::Wall,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(3)],
                    section: Some(SectionId(2)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![col_sec, beam_sec, wall_sec],
            materials: vec![mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert!(
            (zone.length_i - 125.0).abs() < 1e-9,
            "壁のせいが紛れ込んでいないはず: λ_i={}",
            zone.length_i
        );
        assert!(
            (zone.face_i - 300.0).abs() < 1e-9,
            "壁のせいが紛れ込んでいないはず: face_i={}",
            zone.face_i
        );
    }

    /// 壁エレメントモデルの上下大梁の剛性倍率（RESP-D 計算編 02「上下大梁の断面性能」）。
    /// 4節点 Wall 要素の下辺2節点を結ぶ水平梁は iy/a が既定倍率（100倍）になる。
    #[test]
    fn test_beam_new_wall_girder_bottom_edge_scales_stiffness() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

        let sec = Section {
            id: SectionId(0),
            name: "beam".to_string(),
            area: 60000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 600.0,
            width: 300.0,
            as_y: 50000.0,
            as_z: 50000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "conc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ];
        let beam_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };

        // 壁なしモデル（基準）
        let model_no_wall = Model {
            nodes: nodes.clone(),
            elements: vec![beam_elem.clone()],
            sections: vec![sec.clone()],
            materials: vec![mat.clone()],
            ..Default::default()
        };
        let beam_no_wall = BeamElement::new(&beam_elem, &model_no_wall);

        // 壁ありモデル: 節点0-1が下辺、2-3が上辺の4節点壁
        let wall_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let model_with_wall = Model {
            nodes,
            elements: vec![beam_elem.clone(), wall_elem],
            sections: vec![sec],
            materials: vec![mat],
            ..Default::default()
        };
        let beam_with_wall = BeamElement::new(&beam_elem, &model_with_wall);

        assert!(
            (beam_with_wall.iy / beam_no_wall.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
            "iy倍率が既定100倍でない: with={} without={}",
            beam_with_wall.iy,
            beam_no_wall.iy
        );
        assert!(
            (beam_with_wall.a / beam_no_wall.a - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
            "a倍率が既定100倍でない: with={} without={}",
            beam_with_wall.a,
            beam_no_wall.a
        );
        // 質量用断面積（a_mass）は倍率の対象外
        assert!(
            (beam_with_wall.a_mass - beam_no_wall.a_mass).abs() < 1e-9,
            "a_massは変更されないはず"
        );
    }

    /// 壁の節点を1つしか共有しない梁（壁の上辺・下辺ではない）には倍率が掛からない。
    #[test]
    fn test_beam_new_wall_girder_requires_both_nodes_shared() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

        let sec = Section {
            id: SectionId(0),
            name: "beam".to_string(),
            area: 60000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 600.0,
            width: 300.0,
            as_y: 50000.0,
            as_z: 50000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "conc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        // 節点1は壁の隅、節点4は壁に属さない別節点（梁は壁の外へ伸びる）
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
            make_node(4, [8000.0, 0.0, 0.0]),
        ];
        let beam_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(1), NodeId(4)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let wall_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let model = Model {
            nodes,
            elements: vec![beam_elem.clone(), wall_elem],
            sections: vec![sec.clone()],
            materials: vec![mat],
            ..Default::default()
        };
        let beam = BeamElement::new(&beam_elem, &model);
        assert!(
            (beam.iy - sec.iy).abs() < 1e-9,
            "壁節点を1つしか共有しない梁には倍率が掛からないはず: iy={}",
            beam.iy
        );
    }

    /// 鉛直材（柱）は壁節点を2つ共有していても水平材ではないため倍率は掛からない。
    #[test]
    fn test_beam_new_wall_girder_vertical_member_not_scaled() {
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

        let sec = Section {
            id: SectionId(0),
            name: "column".to_string(),
            area: 60000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 600.0,
            width: 300.0,
            as_y: 50000.0,
            as_z: 50000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "conc".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ];
        // 左辺（節点0-3）を結ぶ鉛直材。両端とも壁の節点だが鉛直材なので対象外。
        let column_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let wall_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let model = Model {
            nodes,
            elements: vec![column_elem.clone(), wall_elem],
            sections: vec![sec.clone()],
            materials: vec![mat],
            ..Default::default()
        };
        let column = BeamElement::new(&column_elem, &model);
        assert!(
            (column.iy - sec.iy).abs() < 1e-9,
            "鉛直材は水平材ではないため倍率が掛からないはず: iy={}",
            column.iy
        );
    }

    /// フレーム内雑壁（耐震壁不成立）の柱への袖壁算入（RESP-D 計算編 02
    /// 「フレーム内雑壁のモデル化」）。大開口(r0=√(3.6e6/12e6)=0.548>0.4)の壁は
    /// 耐震壁不成立となり、側柱（左辺=節点0-3）に袖壁として断面性能算入される。
    /// 面内（iz・as_y）は平行軸の定理による合成値と一致し、面外（iy・as_z）は不変。
    #[test]
    fn test_beam_new_misc_wall_wing_augments_column_inplane_stiffness() {
        use squid_n_core::ids::{ElemId, MaterialId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
        };
        use squid_n_core::section_shape::SectionShape;

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let col_sec = Section {
            id: SectionId(0),
            name: "col".into(),
            area: 90_000.0,
            iy: 3.0e9,
            iz: 2.0e9,
            j: 1.0e7,
            depth: 300.0,
            width: 300.0,
            as_y: 50_000.0,
            as_z: 60_000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let wall_shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ];
        let column_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let wall_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(1)),
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
        let mut model = Model {
            nodes,
            elements: vec![column_elem.clone(), wall_elem],
            sections: vec![
                col_sec.clone(),
                wall_shape.to_section(SectionId(1), "W150".into()),
            ],
            materials: vec![mat],
            ..Default::default()
        };
        model.wall_attrs.push(WallAttr {
            elem: ElemId(1),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![WallOpening {
                width: 2400.0,
                height: 1500.0,
                offset: Some([800.0, 750.0]),
            }],
        });

        let column = BeamElement::new(&column_elem, &model);

        // 手計算（misc_wall::tests::test_collect_misc_walls_and_lengths と同じ壁形状）:
        // wing_length(side=0)=800、lww=800-300/2=650、Aw=150*650=97500。
        let d_col: f64 = 300.0;
        let lww = 650.0_f64;
        let aw = 150.0 * lww;
        let ac = col_sec.area;
        let e_i = -(d_col / 2.0 + lww / 2.0);
        let g = (aw * e_i) / (ac + aw);
        let self_i = 150.0 * lww.powi(3) / 12.0;
        let expected_iz = col_sec.iz + ac * g * g + self_i + aw * (e_i - g).powi(2);

        assert!(
            (column.a - (ac + aw)).abs() < 1e-6,
            "a={} expected={}",
            column.a,
            ac + aw
        );
        assert!(
            (column.iz - expected_iz).abs() / expected_iz < 1e-9,
            "iz={} expected={}",
            column.iz,
            expected_iz
        );
        assert!(
            (column.as_y - (col_sec.as_y + aw / 1.2)).abs() < 1e-6,
            "as_y={}",
            column.as_y
        );
        // 面外（iy・as_z）は袖壁算入の影響を受けない
        assert!((column.iy - col_sec.iy).abs() < 1e-6, "iy={}", column.iy);
        assert!(
            (column.as_z - col_sec.as_z).abs() < 1e-6,
            "as_z={}",
            column.as_z
        );
    }

    /// 同じ大開口壁の下辺梁（節点0-1）への腰壁算入。鉛直曲げ（iy・as_z）へ
    /// 平行軸の定理で合成され、耐震壁不成立のため上下大梁100倍は掛からない。
    #[test]
    fn test_beam_new_misc_wall_strip_augments_girder_iy_without_100x() {
        use squid_n_core::ids::{ElemId, MaterialId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
        };
        use squid_n_core::section_shape::SectionShape;

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let beam_sec = Section {
            id: SectionId(0),
            name: "beam".into(),
            area: 200_000.0,
            iy: 5.0e9,
            iz: 1.0e9,
            j: 1.0e7,
            depth: 600.0,
            width: 300.0,
            as_y: 70_000.0,
            as_z: 70_000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let wall_shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ];
        let beam_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let wall_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(1)),
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
        let mut model = Model {
            nodes,
            elements: vec![beam_elem.clone(), wall_elem],
            sections: vec![
                beam_sec.clone(),
                wall_shape.to_section(SectionId(1), "W150".into()),
            ],
            materials: vec![mat],
            ..Default::default()
        };
        model.wall_attrs.push(WallAttr {
            elem: ElemId(1),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![WallOpening {
                width: 2400.0,
                height: 1500.0,
                offset: Some([800.0, 750.0]),
            }],
        });

        let beam = BeamElement::new(&beam_elem, &model);

        // 手計算: strip_height(top=false)=750（lw/2=2000 は開口 x:[800,3200] 内）、
        // hw=750-600/2=450、Aw=150*450=67500。下辺の梁なので壁は上に載る(+方向)。
        let d_beam: f64 = 600.0;
        let hw = 450.0_f64;
        let aw = 150.0 * hw;
        let ac = beam_sec.area;
        let e_i = d_beam / 2.0 + hw / 2.0;
        let g = (aw * e_i) / (ac + aw);
        let self_i = 150.0 * hw.powi(3) / 12.0;
        let expected_iy = beam_sec.iy + ac * g * g + self_i + aw * (e_i - g).powi(2);

        assert!(
            (beam.a - (ac + aw)).abs() < 1e-6,
            "a={} expected={}",
            beam.a,
            ac + aw
        );
        assert!(
            (beam.iy - expected_iy).abs() / expected_iy < 1e-9,
            "iy={} expected={}",
            beam.iy,
            expected_iy
        );
        assert!(
            (beam.as_z - (beam_sec.as_z + aw / 1.2)).abs() < 1e-6,
            "as_z={}",
            beam.as_z
        );
        // 耐震壁不成立のため上下大梁100倍は掛からない（合成値は元の iy の高々数倍）
        assert!(
            beam.iy < beam_sec.iy * 10.0,
            "100倍が誤って適用されている可能性: iy={} base={}",
            beam.iy,
            beam_sec.iy
        );
        // 弱軸（iz・as_y）は腰壁算入の影響を受けない
        assert!((beam.iz - beam_sec.iz).abs() < 1e-6, "iz={}", beam.iz);
        assert!(
            (beam.as_y - beam_sec.as_y).abs() < 1e-6,
            "as_y={}",
            beam.as_y
        );
    }

    /// 耐震壁が成立する壁（無開口・t=150）の周辺部材: 柱・梁とも雑壁算入されず、
    /// 上下大梁は従来どおり100倍（`WALL_GIRDER_STIFF_FACTOR`）のままとなる
    /// （雑壁算入と上下大梁100倍は排他: `collect_misc_walls` は不成立壁のみ返す）。
    #[test]
    fn test_beam_new_seismic_wall_no_misc_wall_augmentation() {
        use squid_n_core::ids::{ElemId, MaterialId, SectionId};
        use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};
        use squid_n_core::section_shape::SectionShape;

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let col_sec = Section {
            id: SectionId(0),
            name: "col".into(),
            area: 90_000.0,
            iy: 3.0e9,
            iz: 2.0e9,
            j: 1.0e7,
            depth: 300.0,
            width: 300.0,
            as_y: 50_000.0,
            as_z: 60_000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let beam_sec = Section {
            id: SectionId(1),
            name: "beam".into(),
            area: 200_000.0,
            iy: 5.0e9,
            iz: 1.0e9,
            j: 1.0e7,
            depth: 600.0,
            width: 300.0,
            as_y: 70_000.0,
            as_z: 70_000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let wall_shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let nodes = vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ];
        let column_elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let beam_elem = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(1)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let wall_elem = ElementData {
            id: ElemId(2),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(2)),
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
        // 開口なし（wall_attrs 未設定）・t=150 → 耐震壁成立
        let model = Model {
            nodes,
            elements: vec![column_elem.clone(), beam_elem.clone(), wall_elem],
            sections: vec![
                col_sec.clone(),
                beam_sec.clone(),
                wall_shape.to_section(SectionId(2), "W150".into()),
            ],
            materials: vec![mat],
            ..Default::default()
        };

        let column = BeamElement::new(&column_elem, &model);
        assert!(
            (column.iz - col_sec.iz).abs() < 1e-6,
            "耐震壁成立時は柱に袖壁算入されないはず: iz={}",
            column.iz
        );
        assert!((column.a - col_sec.area).abs() < 1e-6, "a={}", column.a);
        assert!(
            (column.as_y - col_sec.as_y).abs() < 1e-6,
            "as_y={}",
            column.as_y
        );

        let beam = BeamElement::new(&beam_elem, &model);
        assert!(
            (beam.iy / beam_sec.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
            "耐震壁成立時は従来どおり上下大梁100倍のはず: iy={} base={}",
            beam.iy,
            beam_sec.iy
        );
        assert!(
            (beam.a / beam_sec.area - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
            "a={} base={}",
            beam.a,
            beam_sec.area
        );
    }
}
