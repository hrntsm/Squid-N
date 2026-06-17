use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use sc_core::dof::{DofMap, DOF_PER_NODE};
use sc_core::ids::{ElemId, NodeId};
use sc_core::model::{EndCondition, Material, Model, Section};
use smallvec::SmallVec;

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub enum ZoneSource {
    Auto,
    Manual,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
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
    pub a: f64,
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
    pub section: Option<sc_core::ids::SectionId>,
    pub material: Option<sc_core::ids::MaterialId>,
    /// 確定変位（線形要素の内力計算用。非線形では ElemState が保持）
    pub committed_disp: [f64; 12],
}

fn get_section(model: &Model, sid: Option<sc_core::ids::SectionId>) -> Section {
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
        id: sc_core::ids::SectionId(0),
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
    })
}

fn get_material(model: &Model, mid: Option<sc_core::ids::MaterialId>) -> Material {
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
        id: sc_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
    })
}

impl BeamElement {
    pub fn new(data: &sc_core::model::ElementData, model: &Model) -> Self {
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

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            sc_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            sc_core::model::rect_shear_area(sec.area)
        };

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: sec.area,
            iy: sec.iy,
            iz: sec.iz,
            j: sec.j,
            as_y,
            as_z,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            rigid: RigidZone::default(),
            end_cond: data.end_cond,
            eval_sections: vec![0.0, 0.5, 1.0],
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

    fn apply_rigid_zone_transform(&self, k_flex: &LocalMat, li: f64, lj: f64) -> LocalMat {
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
        let spring_stiffness = |cond: &EndCondition| -> f64 {
            match cond {
                EndCondition::Fixed => 1e12 * self.e * self.iz / self.length.max(1.0),
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
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = f_local[0] * (1.0 - xi) + f_local[6] * xi;
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = f_local[3];
                let my = f_local[4] - f_local[2] * xi * self.length;
                let mz = f_local[5] + f_local[1] * xi * self.length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[0] * (1.0 - xi) + f_local[6] * xi;
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

#[allow(dead_code)]
fn invert_small(a: &[f64], n: usize) -> Vec<f64> {
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

pub fn auto_rigid_zones(
    model: &sc_core::model::Model,
    elem_id: sc_core::ids::ElemId,
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

    // 節点 → 接続要素のマップ
    let mut node_to_elems: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for (ei, e) in model.elements.iter().enumerate() {
        if e.nodes.len() >= 2 {
            for n in &e.nodes {
                node_to_elems.entry(n.index()).or_default().push(ei);
            }
        }
    }

    fn elem_axis(model: &Model, e: &sc_core::model::ElementData) -> [f64; 3] {
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

    fn max_orth_depth(
        model: &Model,
        node_idx: usize,
        target_axis: [f64; 3],
        target_elem_idx: usize,
        node_to_elems: &std::collections::HashMap<usize, Vec<usize>>,
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

    let d_orth_i = max_orth_depth(
        model,
        nodes[0].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
    );
    let d_orth_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1].index(),
        target_axis,
        target_elem_idx,
        &node_to_elems,
    );

    let lambda = |d_orth: f64| -> f64 {
        let v = rule.reduction * (d_orth / 2.0 - d_self / 4.0);
        if v < 0.0 {
            0.0
        } else {
            v
        }
    };

    RigidZone {
        length_i: lambda(d_orth_i),
        length_j: lambda(d_orth_j),
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        reduction: rule.reduction,
    }
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
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
        self.local_stiffness()
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let k = self.local_stiffness();
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
        let m = self.density * self.a * self.length;
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
                // Bending Uy-Rz: indices 1,5,7,11
                //   local order: [Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11]
                let b4 = |mm: &mut LocalMat, i0: usize, j0: usize, sign: f64| {
                    // row0 (Uy_i)
                    mm.set(i0, j0, 156.0 * c2);
                    mm.set(i0, j0 + 1, 22.0 * l * c2 * sign);
                    mm.set(i0, j0 + 2, 54.0 * c2);
                    mm.set(i0, j0 + 3, -13.0 * l * c2 * sign);
                    // row1 (Rz_i / Ry_i)
                    mm.set(i0 + 1, j0, 22.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 1, 4.0 * l2 * c2);
                    mm.set(i0 + 1, j0 + 2, 13.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 3, -3.0 * l2 * c2);
                    // row2 (Uy_j)
                    mm.set(i0 + 2, j0, 54.0 * c2);
                    mm.set(i0 + 2, j0 + 1, 13.0 * l * c2 * sign);
                    mm.set(i0 + 2, j0 + 2, 156.0 * c2);
                    mm.set(i0 + 2, j0 + 3, -22.0 * l * c2 * sign);
                    // row3 (Rz_j / Ry_j)
                    mm.set(i0 + 3, j0, -13.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 1, -3.0 * l2 * c2);
                    mm.set(i0 + 3, j0 + 2, -22.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 3, 4.0 * l2 * c2);
                };
                // Uy-Rz plane (sign = +1)
                b4(&mut mm, 1, 1, 1.0);
                // Uz-Ry plane (sign = -1)
                // local order: [Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10]
                b4(&mut mm, 2, 2, -1.0);
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
    use sc_core::ids::{ElemId, NodeId};
    use sc_core::model::{ElementData, ElementKind, LocalAxis, Material, Node, Section};

    fn make_test_beam() -> BeamElement {
        BeamElement {
            id: ElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 80000.0,
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
        use sc_core::ids::{ElemId, MaterialId, NodeId, SectionId};
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
        };
        let mat = Material {
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
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
                    force_regime: sc_core::model::ForceRegime::Auto,
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
                    force_regime: sc_core::model::ForceRegime::Auto,
                },
            ],
            sections: vec![col_sec, beam_sec],
            materials: vec![mat],
            ..Default::default()
        };

        let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
        assert!((zone.length_i - 125.0).abs() < 1e-9);
    }
}
