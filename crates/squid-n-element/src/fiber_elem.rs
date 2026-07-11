use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_material::uniaxial::UniaxialMaterial;
use squid_n_section::fiber::FiberSection;
use std::any::Any;

pub struct GaussPoint {
    pub xi: f64,
    pub weight: f64,
    pub section: FiberSection,
    pub mats: Vec<Box<dyn UniaxialMaterial>>,
    pub trial_stress: Vec<f64>,
    pub trial_et: Vec<f64>,
}

impl GaussPoint {
    pub fn new(
        xi: f64,
        weight: f64,
        section: FiberSection,
        mut mats: Vec<Box<dyn UniaxialMaterial>>,
    ) -> Self {
        let n = section.fibers.len();
        // 接線キャッシュを各ファイバの初期弾性接線で初期化する。
        // 未初期化（0）のままだと、最初の update_state より前に tangent_stiffness を
        // 呼ぶ経路（pushover の初回 assemble_k）で剛性が 0 になり特異化する。
        let trial_et: Vec<f64> = mats.iter_mut().map(|m| m.trial(0.0).1).collect();
        GaussPoint {
            xi,
            weight,
            section,
            mats,
            trial_stress: vec![0.0; n],
            trial_et,
        }
    }
}

pub struct FiberBeam {
    pub length: f64,
    pub nodes: [NodeId; 2],
    pub gauss_points: Vec<GaussPoint>,
    pub shear: crate::shear_spring::ShearSpring,
    pub density: f64,
    /// ねじり定数 J [mm⁴]（Section.j から取得）。
    /// Saint-Venant ねじり剛性 G·J/L の計算に用いる。
    pub torsion_j: f64,
    /// せん断弾性係数 G [N/mm²]（Material.shear_modulus）。
    /// せん断ばねおよびねじり剛性の計算に用いる。
    pub g: f64,
    /// 要素ローカル系→グローバル系の回転（柱・斜材で必須）。
    /// 内部状態（trial_disp 等）はローカル系で保持し、トレイト境界で回転する。
    pub axis: crate::transform::LocalFrame,
    /// 塑性化域考慮モデルの中央弾性部剛性（ローカル系 12×12）。
    /// None = 従来の全長ファイバー積分モデル。
    pub k_mid: Option<LocalMat>,
    pub committed_disp: [f64; 12],
    pub trial_disp: [f64; 12],
}

impl FiberBeam {
    pub fn new(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
    ) -> Self {
        let n0 = &model.nodes[data.nodes[0].index()];
        let n1 = &model.nodes[data.nodes[1].index()];
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let dz = n1.coord[2] - n0.coord[2];
        let length = (dx * dx + dy * dy + dz * dz).sqrt();

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let density = mat_ref.map(|m| m.density).unwrap_or(0.0);
        let e = mat_ref.map(|m| m.young).unwrap_or(205000.0);
        let g = mat_ref.map(|m| m.shear_modulus()).unwrap_or(78846.0);
        let area = sec.map(|s| s.area).unwrap_or(0.0);
        let width = sec.map(|s| s.width).unwrap_or(100.0);
        let depth = sec.map(|s| s.depth).unwrap_or(200.0);
        let torsion_j = sec.map(|s| s.j).unwrap_or(0.0);
        let as_y = sec.map(|s| s.as_y).unwrap_or(area * 5.0 / 6.0);
        let as_z = sec.map(|s| s.as_z).unwrap_or(area * 5.0 / 6.0);

        let shear = crate::shear_spring::ShearSpring::new(length, g, as_y, as_z);

        let nw = 12;
        let nd = 20;
        let n_fibers = nw * nd;

        let template: Box<dyn UniaxialMaterial> = if let Some(fc) = mat_ref.and_then(|m| m.fc) {
            Box::new(squid_n_material::uniaxial::Concrete::new(fc, 2.0))
        } else {
            // 鋼材：降伏応力 fy が与えられれば弾塑性、無ければ実質弾性（fy=1e20）。
            let fy = mat_ref.and_then(|m| m.fy).unwrap_or(1e20);
            Box::new(squid_n_material::uniaxial::Bilinear::new(e, fy, 0.01))
        };

        let gauss_points = vec![
            GaussPoint::new(
                -0.5773502691896257,
                1.0,
                squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, 0),
                squid_n_section::fiber::uniform_fiber_mats(&*template, n_fibers),
            ),
            GaussPoint::new(
                0.5773502691896257,
                1.0,
                squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, 0),
                squid_n_section::fiber::uniform_fiber_mats(&*template, n_fibers),
            ),
        ];

        let axis = crate::transform::LocalFrame::from_nodes(
            n0.coord,
            n1.coord,
            data.local_axis.ref_vector,
        );

        FiberBeam {
            length,
            nodes: [data.nodes[0], data.nodes[1]],
            gauss_points,
            shear,
            density,
            torsion_j,
            g,
            axis,
            k_mid: None,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        }
    }

    /// 塑性化域考慮のファイバー要素（材端剛塑性ばねモデルと適合する
    /// ファイバーモデル化）。端部の塑性化領域（長さ `lp`）にファイバー断面を
    /// 配置（積分点 ξ=∓1、重み Lp）し、中央 [Lp, L−Lp] は断面諸元
    /// （EA・EIy・EIz）による弾性剛性として厳密に B 積分する。
    pub fn with_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
    ) -> Self {
        Self::build_plastic_zone(data, model, lp, 12, 20)
    }

    /// 塑性化域考慮要素の実体。`nw × nd` は端部断面のファイバ分割数
    /// （マルチファイバー: 12×20、マルチスプリング: 2×5 の粗い配置）。
    pub(crate) fn build_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
        nw: usize,
        nd: usize,
    ) -> Self {
        let mut fb = Self::new(data, model);
        let l = fb.length;
        if l <= 0.0 {
            return fb;
        }
        // Lp は部材長の 45% までにクランプ（両端合計で全長を超えない）
        let lp = lp.clamp(1.0e-6 * l, 0.45 * l);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let e = mat_ref.map(|m| m.young).unwrap_or(205000.0);
        let width = sec.map(|s| s.width).unwrap_or(100.0);
        let depth = sec.map(|s| s.depth).unwrap_or(200.0);
        let area = sec.map(|s| s.area).unwrap_or(width * depth);
        let iy = sec.map(|s| s.iy).unwrap_or(1.0);
        let iz = sec.map(|s| s.iz).unwrap_or(1.0);

        let template: Box<dyn UniaxialMaterial> = if let Some(fc) = mat_ref.and_then(|m| m.fc) {
            Box::new(squid_n_material::uniaxial::Concrete::new(fc, 2.0))
        } else {
            let fy = mat_ref.and_then(|m| m.fy).unwrap_or(1e20);
            Box::new(squid_n_material::uniaxial::Bilinear::new(e, fy, 0.01))
        };

        // 端部積分点: ξ=∓1、重み w·(L/2) = Lp → w = 2Lp/L
        let w_end = 2.0 * lp / l;
        let n_fibers = nw * nd;
        fb.gauss_points = vec![
            GaussPoint::new(
                -1.0,
                w_end,
                squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, 0),
                squid_n_section::fiber::uniform_fiber_mats(&*template, n_fibers),
            ),
            GaussPoint::new(
                1.0,
                w_end,
                squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, 0),
                squid_n_section::fiber::uniform_fiber_mats(&*template, n_fibers),
            ),
        ];

        // 中央弾性部 [Lp, L−Lp] の剛性: B(ξ)ᵀ·diag(EA,EIy,EIz)·B(ξ) を
        // 2点 Gauss（区間 [−h, h]、h = 1−2Lp/L）で厳密積分（被積分関数は ξ の2次）
        let h = 1.0 - 2.0 * lp / l;
        let d_el = [e * area, e * iy, e * iz];
        let mut k_mid = LocalMat::zeros(12);
        for sgn in [-1.0, 1.0] {
            let xi = sgn * h / 3.0_f64.sqrt();
            let w_phys = h * l / 2.0;
            let b = Self::compute_b_matrix(xi, l);
            for i in 0..12 {
                for j in 0..12 {
                    let mut val = 0.0;
                    for (p, dp) in d_el.iter().enumerate() {
                        val += b[p][i] * dp * b[p][j];
                    }
                    if val != 0.0 {
                        k_mid.set(i, j, k_mid.get(i, j) + val * w_phys);
                    }
                }
            }
        }
        fb.k_mid = Some(k_mid);
        fb
    }

    fn beam_global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &n in &self.nodes {
            let ni = n.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn section_response_from_cache(gp: &GaussPoint) -> ([f64; 3], [[f64; 3]; 3]) {
        let mut force = [0.0; 3];
        let mut stiff = [[0.0; 3]; 3];
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let a = fiber.area;
            let sigma = gp.trial_stress[i];
            let et = gp.trial_et[i];
            force[0] += sigma * a;
            force[1] += sigma * a * fiber.z;
            force[2] += -sigma * a * fiber.y;
            stiff[0][0] += et * a;
            stiff[0][1] += et * a * fiber.z;
            stiff[0][2] += -et * a * fiber.y;
            stiff[1][1] += et * a * fiber.z * fiber.z;
            stiff[1][2] += -et * a * fiber.y * fiber.z;
            stiff[2][2] += et * a * fiber.y * fiber.y;
        }
        stiff[1][0] = stiff[0][1];
        stiff[2][0] = stiff[0][2];
        stiff[2][1] = stiff[1][2];
        (force, stiff)
    }

    fn compute_b_matrix(xi: f64, l: f64) -> [[f64; 12]; 3] {
        let inv_l = 1.0 / l;
        let inv_l2 = 1.0 / (l * l);
        let mut b = [[0.0; 12]; 3];
        b[0][0] = -inv_l;
        b[0][6] = inv_l;
        b[1][2] = 6.0 * xi * inv_l2;
        b[1][4] = (1.0 - 3.0 * xi) * inv_l;
        b[1][8] = -6.0 * xi * inv_l2;
        b[1][10] = -(1.0 + 3.0 * xi) * inv_l;
        b[2][1] = -6.0 * xi * inv_l2;
        b[2][5] = (1.0 - 3.0 * xi) * inv_l;
        b[2][7] = 6.0 * xi * inv_l2;
        b[2][11] = -(1.0 + 3.0 * xi) * inv_l;
        b
    }
}

impl ElementBehavior for FiberBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.beam_global_dofs(dof)
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let l = self.length;
        if l <= 0.0 {
            return k;
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (_, d) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = Self::compute_b_matrix(gp.xi, l);

            for i in 0..12 {
                for p in 0..3 {
                    let bpi = b[p][i];
                    if bpi == 0.0 {
                        continue;
                    }
                    for j in 0..12 {
                        let mut val = 0.0;
                        for q in 0..3 {
                            val += d[p][q] * b[q][j];
                        }
                        if val != 0.0 {
                            let old = k.get(i, j);
                            k.set(i, j, old + bpi * val * w);
                        }
                    }
                }
            }
        }

        // 塑性化域考慮モデル: 中央弾性部の剛性を加算
        if let Some(km) = &self.k_mid {
            for i in 0..12 {
                for j in 0..12 {
                    let old = k.get(i, j);
                    k.set(i, j, old + km.get(i, j));
                }
            }
        }

        let ks = self.shear.tangent_stiffness(&ElemState::default());
        for i in 0..12 {
            for j in 0..12 {
                let old = k.get(i, j);
                k.set(i, j, old + ks.get(i, j));
            }
        }

        // ねじり剛性（Saint-Venant）を rx DOF (index 3, 9) に付加
        if self.torsion_j > 0.0 && l > 0.0 {
            let kt = self.g * self.torsion_j / l;
            k.set(3, 3, k.get(3, 3) + kt);
            k.set(9, 9, k.get(9, 9) + kt);
            k.set(3, 9, k.get(3, 9) - kt);
            k.set(9, 3, k.get(9, 3) - kt);
        }

        // ローカル接線剛性をグローバル節点系へ回転（R^T·K·R）
        self.axis.to_global(&k)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let l = self.length;
        if l <= 0.0 {
            return f;
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (force, _) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = Self::compute_b_matrix(gp.xi, l);
            let n = force[0];
            let my = force[1];
            let mz = force[2];

            for i in 0..12 {
                let val = b[0][i] * n + b[1][i] * my + b[2][i] * mz;
                f.data[i] += val * w;
            }
        }

        // 塑性化域考慮モデル: 中央弾性部の内力（線形: K_mid·u）を加算
        if let Some(km) = &self.k_mid {
            for i in 0..12 {
                let mut si = 0.0;
                for j in 0..12 {
                    si += km.get(i, j) * self.trial_disp[j];
                }
                f.data[i] += si;
            }
        }

        let ks = self.shear.tangent_stiffness(&ElemState::default());
        for i in 0..12 {
            let mut si = 0.0;
            for j in 0..12 {
                si += ks.get(i, j) * self.trial_disp[j];
            }
            f.data[i] += si;
        }

        // ねじり内力（Saint-Venant）
        if self.torsion_j > 0.0 && l > 0.0 {
            let kt = self.g * self.torsion_j / l;
            let drx = self.trial_disp[3] - self.trial_disp[9];
            f.data[3] += kt * drx;
            f.data[9] -= kt * drx;
        }

        // ローカル内力をグローバル系へ回転（committed/trial はローカル保持のため）
        let f_local: [f64; 12] = std::array::from_fn(|i| f.data[i]);
        let f_global = self.axis.rotate_to_global(&f_local);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        // 入力 du はグローバル系。内部状態（trial_disp, B行列ひずみ）はローカル系で
        // 扱うため、まずローカル系へ回転してから累積する。
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        for i in 0..12 {
            self.trial_disp[i] += du_local[i];
        }
        let l = self.length;
        if l <= 0.0 {
            return;
        }

        for gp in &mut self.gauss_points {
            let b = Self::compute_b_matrix(gp.xi, l);
            let eps0 = b[0][0] * self.trial_disp[0] + b[0][6] * self.trial_disp[6];
            let ky = b[1][2] * self.trial_disp[2]
                + b[1][4] * self.trial_disp[4]
                + b[1][8] * self.trial_disp[8]
                + b[1][10] * self.trial_disp[10];
            let kz = b[2][1] * self.trial_disp[1]
                + b[2][5] * self.trial_disp[5]
                + b[2][7] * self.trial_disp[7]
                + b[2][11] * self.trial_disp[11];
            for (i, fiber) in gp.section.fibers.iter().enumerate() {
                let eps = eps0 - kz * fiber.y + ky * fiber.z;
                let (sigma, et) = gp.mats[i].trial(eps);
                gp.trial_stress[i] = sigma;
                gp.trial_et[i] = et;
            }
        }
        if commit {
            for gp in &mut self.gauss_points {
                for mat in &mut gp.mats {
                    mat.commit();
                }
            }
            self.committed_disp = self.trial_disp;
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let total_area: f64 = self
            .gauss_points
            .first()
            .map(|gp| gp.section.fibers.iter().map(|f| f.area).sum())
            .unwrap_or(0.0);
        let total_mass = self.density * total_area * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, total_mass / 2.0);
                }
            }
            MassOption::Consistent => {
                let c1 = total_mass / 6.0;
                let c2 = total_mass / 420.0;
                let l = self.length;
                let l2 = l * l;
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                let b4 = |mm: &mut LocalMat, i0: usize, j0: usize, sign: f64| {
                    mm.set(i0, j0, 156.0 * c2);
                    mm.set(i0, j0 + 1, 22.0 * l * c2 * sign);
                    mm.set(i0, j0 + 2, 54.0 * c2);
                    mm.set(i0, j0 + 3, -13.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0, 22.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 1, 4.0 * l2 * c2);
                    mm.set(i0 + 1, j0 + 2, 13.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 3, -3.0 * l2 * c2);
                    mm.set(i0 + 2, j0, 54.0 * c2);
                    mm.set(i0 + 2, j0 + 1, 13.0 * l * c2 * sign);
                    mm.set(i0 + 2, j0 + 2, 156.0 * c2);
                    mm.set(i0 + 2, j0 + 3, -22.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0, -13.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 1, -3.0 * l2 * c2);
                    mm.set(i0 + 3, j0 + 2, -22.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 3, 4.0 * l2 * c2);
                };
                b4(&mut mm, 1, 1, 1.0);
                b4(&mut mm, 2, 2, -1.0);
            }
        }
        mm
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
        // 幾何剛性もグローバル系へ回転
        self.axis.to_global(&kg)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let gauss_data: Vec<Vec<Box<dyn UniaxialMaterial>>> = self
            .gauss_points
            .iter()
            .map(|gp| gp.mats.iter().map(|m| m.clone_box()).collect())
            .collect();
        Box::new((self.trial_disp, self.committed_disp, gauss_data))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some((trial, committed, mats_data)) =
            state.downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
        {
            self.trial_disp = *trial;
            self.committed_disp = *committed;
            for (gp, gp_mats) in self.gauss_points.iter_mut().zip(mats_data) {
                for (mat, new_mat) in gp.mats.iter_mut().zip(gp_mats) {
                    *mat = new_mat.clone_box();
                }
            }
        }
    }

    fn commit_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.commit();
            }
        }
        self.committed_disp = self.trial_disp;
    }

    fn revert_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.revert();
            }
        }
        self.trial_disp = self.committed_disp;
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct FiberBeamCheckpoint {
            trial_disp: [f64; 12],
            committed_disp: [f64; 12],
            gauss_points: Vec<Vec<Vec<u8>>>,
        }
        let gauss_points: Vec<Vec<Vec<u8>>> = self
            .gauss_points
            .iter()
            .map(|gp| {
                gp.mats
                    .iter()
                    .map(|m| m.serialize_state())
                    .collect::<Vec<_>>()
            })
            .collect();
        let cp = FiberBeamCheckpoint {
            trial_disp: self.trial_disp,
            committed_disp: self.committed_disp,
            gauss_points,
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(&mut self, data: &[u8]) {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct FiberBeamCheckpoint {
            trial_disp: [f64; 12],
            committed_disp: [f64; 12],
            gauss_points: Vec<Vec<Vec<u8>>>,
        }
        if let Ok(cp) = bincode::deserialize::<FiberBeamCheckpoint>(data) {
            self.trial_disp = cp.trial_disp;
            self.committed_disp = cp.committed_disp;
            for (gp, gp_mats) in self.gauss_points.iter_mut().zip(cp.gauss_points) {
                for (mat, mat_bytes) in gp.mats.iter_mut().zip(gp_mats) {
                    mat.deserialize_state(&mat_bytes);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::behavior::Ctx;
    use approx::assert_relative_eq;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        Section,
    };

    fn make_test_fiber_beam(shear_mod: Option<f64>) -> FiberBeam {
        let model = build_test_model(shear_mod);
        FiberBeam::new(&model.elements[0], &model)
    }

    fn make_test_beam_element(as_val: f64) -> crate::beam::BeamElement {
        crate::beam::BeamElement {
            id: ElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 20000.0,
            a_mass: 20000.0,
            iy: 66666666.66666667,
            iz: 16666666.66666667,
            j: 0.0,
            as_y: as_val,
            as_z: as_val,
            length: 3000.0,
            density: 0.0,
            nodes: [NodeId(0), NodeId(1)],
            axis: crate::transform::LocalFrame {
                rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            rigid: squid_n_core::model::RigidZone::default(),
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            eval_sections: vec![],
            section: None,
            material: None,
            committed_disp: [0.0; 12],
        }
    }

    fn build_test_model(shear_mod: Option<f64>) -> Model {
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
                kind: ElementKind::Fiber,
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
                name: "test".to_string(),
                area: 20000.0,
                iy: 66666666.66666667,
                iz: 16666666.66666667,
                j: 0.0,
                depth: 200.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
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
                shear: shear_mod,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    /// 指定した2節点座標・参照ベクトルで FiberBeam を生成するヘルパ（座標変換テスト用）。
    fn make_oriented_fiber(p0: [f64; 3], p1: [f64; 3], ref_vec: [f64; 3]) -> FiberBeam {
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: p0,
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: p1,
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: ref_vec,
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "s".to_string(),
                area: 20000.0,
                iy: 66666666.66666667,
                iz: 16666666.66666667,
                j: 0.0,
                depth: 200.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
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
                fc: None,
                fy: None,
            }],
            ..Default::default()
        };
        FiberBeam::new(&model.elements[0], &model)
    }

    /// 降伏応力 fy を指定した鋼材ファイバ梁（X 整列・恒等フレーム）を生成するヘルパ。
    fn make_steel_fiber_with_fy(fy: Option<f64>) -> FiberBeam {
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
                    coord: [3000.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
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
                name: "s".to_string(),
                area: 20000.0,
                iy: 66666666.66666667,
                iz: 16666666.66666667,
                j: 0.0,
                depth: 200.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
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
                fc: None,
                fy,
            }],
            ..Default::default()
        };
        FiberBeam::new(&model.elements[0], &model)
    }

    /// ねじり剛性テスト用の FiberBeam を生成する。
    /// 既知の G, J, L で Saint-Venant ねじり剛性を検証するため。
    fn make_torsion_fiber_beam(g: f64, j: f64) -> FiberBeam {
        let mut model = build_test_model(Some(g));
        model.sections[0].j = j;
        FiberBeam::new(&model.elements[0], &model)
    }

    /// 降伏データ検証: Material.fy を与えた鋼材ファイバは、同一の大曲率変形に対して
    /// 弾性材（fy 無し＝1e20）より小さい曲げ内力を示す（＝実際に降伏している）。
    #[test]
    fn test_fiber_steel_yields_with_fy() {
        let ctx = Ctx {
            model: &Model::default(),
        };
        // 端部 ry に十分大きな逆対称回転を与え、曲げで降伏させる。
        let big = 0.1;
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, 0.0, 0.0, -big, 0.0],
        };

        let mut yielding = make_steel_fiber_with_fy(Some(235.0));
        yielding.update_state(&du, true, &ctx);
        let f_y = yielding.internal_force(&ElemState::default(), &ctx);

        let mut elastic = make_steel_fiber_with_fy(None);
        elastic.update_state(&du, true, &ctx);
        let f_e = elastic.internal_force(&ElemState::default(), &ctx);

        // 曲げモーメント DOF(ry_i = index 4) で比較。降伏材は弾性材より明確に小さいこと。
        assert!(
            f_e.data[4].abs() > 1.0,
            "elastic bending moment must be non-trivial (test sanity): {}",
            f_e.data[4]
        );
        assert!(
            f_y.data[4].abs() < f_e.data[4].abs() * 0.5,
            "yielding moment {} should be well below elastic {} (fy plumbing inactive?)",
            f_y.data[4],
            f_e.data[4]
        );
    }

    /// 座標変換の検証: 軸方向（X 整列）と鉛直柱（Z 整列）でグローバル接線剛性を比較し、
    /// 軸剛性・曲げ剛性が正しいグローバル DOF へ写像されることを確認する。
    /// 回転変換が欠落していると鉛直柱の水平 DOF に軸剛性が誤って現れる。
    #[test]
    fn test_global_rotation_vertical_column() {
        let l = 3000.0;
        let ctx = Ctx {
            model: &Model::default(),
        };
        let zero_du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        // X 整列（ref [0,1,0] で恒等フレーム）: local x = global X(軸), local y = global Y(曲げ)
        let mut fx = make_oriented_fiber([0.0, 0.0, 0.0], [l, 0.0, 0.0], [0.0, 1.0, 0.0]);
        fx.update_state(&zero_du, false, &ctx); // 初期接線（弾性係数）をキャッシュへ
        let kx = fx.tangent_stiffness(&ElemState::default(), &ctx);
        // Z 整列（鉛直柱, ref [1,0,0]）: local x = global Z(軸), local y = global X(曲げ)
        let mut fz = make_oriented_fiber([0.0, 0.0, 0.0], [0.0, 0.0, l], [1.0, 0.0, 0.0]);
        fz.update_state(&zero_du, false, &ctx);
        let kz = fz.tangent_stiffness(&ElemState::default(), &ctx);

        // 軸剛性: X 整列の ux_i (DOF0) == Z 整列の uz_i (DOF2)
        assert_relative_eq!(kz.get(2, 2), kx.get(0, 0), epsilon = 1.0);
        // 曲げ剛性: X 整列の uy_i (DOF1, local 曲げ) == Z 整列の ux_i (DOF0, local 曲げ)
        assert_relative_eq!(kz.get(0, 0), kx.get(1, 1), epsilon = 1.0);
        // 鉛直柱の水平 DOF は曲げ剛性（小）であって軸剛性（大）ではないこと
        assert!(
            kz.get(0, 0) < kz.get(2, 2),
            "vertical column horizontal DOF must be bending (small), not axial (large): ux={}, uz={}",
            kz.get(0, 0),
            kz.get(2, 2)
        );
    }

    #[test]
    fn test_elastic_stiffness_matches_beam() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let beam = make_test_beam_element(1e30);

        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let state = ElemState::default();

        let u = [
            1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
        ];
        let du = LocalVec {
            data: SmallVec::from_slice(&u),
        };
        fiber.update_state(&du, true, &ctx);

        let k_fiber = fiber.tangent_stiffness(&state, &ctx);
        let k_beam = beam.local_stiffness_raw();

        for i in 0..12 {
            for j in 0..12 {
                let expected = k_beam.get(i, j);
                let actual = k_fiber.get(i, j);
                if expected.abs() > 1e-6 {
                    assert_relative_eq!(actual, expected, max_relative = 0.01);
                } else {
                    assert!(
                        actual.abs() < 1e-3,
                        "K[{i}][{j}] zero expected, got {actual}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_elastic_stiffness_symmetric() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let state = ElemState::default();

        let u = [
            1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
        ];
        let du = LocalVec {
            data: SmallVec::from_slice(&u),
        };
        fiber.update_state(&du, true, &ctx);

        let k = fiber.tangent_stiffness(&state, &ctx);
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
    fn test_axial_response() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let state = ElemState::default();

        let eps0 = 0.001;
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                eps0 * 3000.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);

        let f = fiber.internal_force(&state, &ctx);
        let a_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area)
            .sum();
        let expected_n = eps0 * 205000.0 * a_disc;
        assert_relative_eq!(f.data[0], -expected_n, epsilon = 1.0);
        assert_relative_eq!(f.data[6], expected_n, epsilon = 1.0);
    }

    #[test]
    fn test_pure_bending_mphi() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let state = ElemState::default();

        let ky = 1e-6;
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                0.0,
                0.0,
                0.0,
                ky * 3000.0 / 2.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -ky * 3000.0 / 2.0,
                0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);

        let f = fiber.internal_force(&state, &ctx);
        let iy_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area * f.z * f.z)
            .sum();
        let expected_my = ky * 205000.0 * iy_disc;
        assert_relative_eq!(f.data[4], expected_my, epsilon = 1.0);
        assert_relative_eq!(f.data[10], -expected_my, epsilon = 1.0);
    }

    #[test]
    fn test_n_m_interaction() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let state = ElemState::default();

        let eps0 = 0.0005;
        let ky = 1e-6;
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                0.0,
                0.0,
                0.0,
                ky * 3000.0 / 2.0,
                0.0,
                eps0 * 3000.0,
                0.0,
                0.0,
                0.0,
                -ky * 3000.0 / 2.0,
                0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);

        let f = fiber.internal_force(&state, &ctx);
        let a_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area)
            .sum();
        let iy_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area * f.z * f.z)
            .sum();
        let expected_n = eps0 * 205000.0 * a_disc;
        let expected_my = ky * 205000.0 * iy_disc;
        assert_relative_eq!(f.data[0], -expected_n, epsilon = 1.0);
        assert_relative_eq!(f.data[4], expected_my, epsilon = 1.0);
    }

    #[test]
    fn test_yield_progression() {
        let mut fiber = {
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
                        coord: [3000.0, 0.0, 0.0],
                        restraint: Default::default(),
                        mass: None,
                        story: None,
                    },
                ],
                elements: vec![ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Fiber,
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
                    name: "yield_test".to_string(),
                    area: 20000.0,
                    iy: 66666666.66666667,
                    iz: 16666666.66666667,
                    j: 0.0,
                    depth: 200.0,
                    width: 100.0,
                    as_y: 0.0,
                    as_z: 0.0,
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
                    fc: None,
                    fy: None,
                }],
                ..Default::default()
            };
            FiberBeam::new(&model.elements[0], &model)
        };

        let ctx = Ctx {
            model: &Model::default(),
        };
        let state = ElemState::default();

        let eps_y = 235.0 / 205000.0;
        let z_max = 100.0;
        let ky_y = eps_y / z_max;
        let ky_final = ky_y * 3.0;

        let mut last_my = 0.0;
        let n_steps = 50;
        let mut prev_ky = 0.0;
        for i in 1..=n_steps {
            let ky_curr = ky_final * (i as f64) / (n_steps as f64);
            let dky = ky_curr - prev_ky;
            prev_ky = ky_curr;
            let du = LocalVec {
                data: SmallVec::from_slice(&[
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    dky * 3000.0 / 2.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    -dky * 3000.0 / 2.0,
                    0.0,
                ]),
            };
            fiber.update_state(&du, true, &ctx);

            let f = fiber.internal_force(&state, &ctx);
            last_my = f.data[4];
        }

        let iy_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area * f.z * f.z)
            .sum();
        let elastic_pred = ky_final * 205000.0 * iy_disc;
        assert!(
            last_my < elastic_pred,
            "post-yield My ({}) must be below elastic prediction ({})",
            last_my,
            elastic_pred
        );
    }

    #[test]
    fn test_commit_revert() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };

        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };

        fiber.update_state(&du, false, &ctx);
        assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
        assert_relative_eq!(fiber.committed_disp[4], 0.0, epsilon = 1e-12);
        fiber.revert_state();
        assert_relative_eq!(fiber.trial_disp[4], 0.0, epsilon = 1e-12);
        assert_relative_eq!(fiber.committed_disp[4], 0.0, epsilon = 1e-12);

        fiber.update_state(&du, false, &ctx);
        fiber.commit_state();
        assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
        assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);

        let du2 = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fiber.update_state(&du2, false, &ctx);
        assert_relative_eq!(fiber.trial_disp[4], 0.003, epsilon = 1e-12);
        fiber.revert_state();
        assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
        assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);
    }

    #[test]
    fn test_snapshot_restore() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };

        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);
        let snap = fiber.snapshot_state();

        let du2 = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fiber.update_state(&du2, false, &ctx);
        assert_relative_eq!(fiber.trial_disp[4], 0.003, epsilon = 1e-12);

        fiber.restore_state(&*snap);
        assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
        assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);
    }

    #[test]
    fn test_geometric_stiffness() {
        let fiber = make_test_fiber_beam(Some(0.0));
        let n = 100000.0;
        let kg = fiber.geometric_stiffness(n);
        let l = fiber.length;
        let c = n / l;
        assert_relative_eq!(kg.get(1, 1), c * 6.0 / 5.0, epsilon = 1e-9);
        assert_relative_eq!(kg.get(5, 5), c * 2.0 * l * l / 15.0, epsilon = 1e-9);
        assert_relative_eq!(kg.get(4, 4), c * 2.0 * l * l / 15.0, epsilon = 1e-9);
        assert_relative_eq!(kg.get(2, 4), -c * l / 10.0, epsilon = 1e-9);
    }

    #[test]
    fn test_internal_force_zero_at_zero_disp() {
        let fiber = make_test_fiber_beam(None);
        let f = fiber.internal_force(
            &ElemState::default(),
            &Ctx {
                model: &Model::default(),
            },
        );
        for v in f.data.iter() {
            assert!(v.abs() < 1e-12, "zero disp should give zero force, got {v}");
        }
    }

    #[test]
    fn test_fiber_section_area_matches_section() {
        let fiber = make_test_fiber_beam(None);
        let a_disc: f64 = fiber.gauss_points[0]
            .section
            .fibers
            .iter()
            .map(|f| f.area)
            .sum();
        let expected = 100.0 * 200.0;
        assert_relative_eq!(a_disc, expected, max_relative = 0.01);
    }

    #[test]
    fn test_update_state_trial_stress_nonzero() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };

        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fiber.update_state(&du, false, &ctx);

        for gp in &fiber.gauss_points {
            for &s in &gp.trial_stress {
                assert!(
                    s.abs() > 0.0,
                    "trial_stress should be nonzero after axial disp"
                );
            }
        }
    }

    #[test]
    fn test_different_gp_have_independent_mats() {
        let fiber = make_test_fiber_beam(Some(0.0));
        let gp0_ptr = &fiber.gauss_points[0].mats[0] as *const _;
        let gp1_ptr = &fiber.gauss_points[1].mats[0] as *const _;
        assert_ne!(gp0_ptr, gp1_ptr, "GP mats must be independent instances");
    }

    #[test]
    fn test_torsional_stiffness() {
        let g = 78846.0;
        let j = 1.0e6;
        let l = 3000.0;
        let expected_kt = g * j / l;

        let mut fiber = make_torsion_fiber_beam(g, j);
        let ctx = Ctx {
            model: &build_test_model(Some(g)),
        };
        // 接線キャッシュを初期化（ゼロ変位で update_state）
        let zero_du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        fiber.update_state(&zero_du, false, &ctx);

        let k = fiber.tangent_stiffness(&ElemState::default(), &ctx);
        assert!(
            (k.get(3, 3) - expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
            "K[3][3] should be G*J/L: expected {}, got {}",
            expected_kt,
            k.get(3, 3)
        );
        assert!(
            (k.get(9, 9) - expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
            "K[9][9] should be G*J/L: expected {}, got {}",
            expected_kt,
            k.get(9, 9)
        );
        assert!(
            (k.get(3, 9) + expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
            "K[3][9] should be -G*J/L: expected {}, got {}",
            -expected_kt,
            k.get(3, 9)
        );
        assert!(
            (k.get(9, 3) + expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
            "K[9][3] should be -G*J/L: expected {}, got {}",
            -expected_kt,
            k.get(9, 3)
        );
    }

    #[test]
    fn test_torsional_internal_force() {
        let g = 78846.0;
        let j = 1.0e6;
        let l = 3000.0;
        let kt = g * j / l;

        let mut fiber = make_torsion_fiber_beam(g, j);
        let ctx = Ctx {
            model: &build_test_model(Some(g)),
        };
        let theta_i = 0.01;
        let theta_j = -0.005;
        let du = LocalVec {
            data: smallvec::smallvec![
                0.0, 0.0, 0.0, theta_i, 0.0, 0.0, 0.0, 0.0, 0.0, theta_j, 0.0, 0.0,
            ],
        };
        fiber.update_state(&du, true, &ctx);
        let f = fiber.internal_force(&ElemState::default(), &ctx);

        let expected_mx_i = kt * (theta_i - theta_j);
        assert!(
            (f.data[3] - expected_mx_i).abs() < 1e-6 * expected_mx_i.abs().max(1.0),
            "Mx_i should be kt*(θ_i - θ_j): expected {}, got {}",
            expected_mx_i,
            f.data[3]
        );
        assert!(
            (f.data[9] + expected_mx_i).abs() < 1e-6 * expected_mx_i.abs().max(1.0),
            "Mx_j should be -Mx_i: expected {}, got {}",
            -expected_mx_i,
            f.data[9]
        );
    }

    /// 鉛直柱（Z整列）でねじり剛性 GJ 追加後、グローバル rz DOF (index 5, 11) が
    /// 特異でない（非ゼロの対角成分を持つ）ことを確認する回帰テスト。
    /// 以前は rz 拘束が無いと特異化していた。
    #[test]
    fn test_vertical_column_rz_nonsingular() {
        let g = 78846.0;
        let j = 1.0e6;
        let l = 3000.0;
        let expected_kt = g * j / l;

        // Z 整列（鉛直柱）: local x = global Z
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
                    coord: [0.0, 0.0, l],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
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
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "col".to_string(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
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
                shear: Some(g),
                fc: None,
                fy: None,
            }],
            ..Default::default()
        };

        let mut fiber = FiberBeam::new(&model.elements[0], &model);
        let ctx = Ctx {
            model: &Model::default(),
        };
        let zero_du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        fiber.update_state(&zero_du, false, &ctx);

        let k = fiber.tangent_stiffness(&ElemState::default(), &ctx);
        // 鉛直柱では local rx が global rz に回転される。
        // global rz は節点自由度 index 5 (i端) と index 11 (j端)。
        let k55 = k.get(5, 5);
        let k11_11 = k.get(11, 11);
        assert!(
            k55 > 0.0,
            "global rz_i (k[5][5]) must be > 0 with torsion stiffness, got {}",
            k55
        );
        assert!(
            k11_11 > 0.0,
            "global rz_j (k[11][11]) must be > 0 with torsion stiffness, got {}",
            k11_11
        );
        // ねじり剛性が回転後も正しく伝わっていることの緩い確認
        let _ = expected_kt;
    }

    #[test]
    fn test_fiber_beam_checkpoint_roundtrip() {
        let mut fiber = make_test_fiber_beam(Some(0.0));
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0005, 0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);

        let snap_before = fiber.snapshot_state();
        let checkpoint = fiber.serialize_checkpoint();

        let mut restored = make_test_fiber_beam(Some(0.0));
        restored.deserialize_checkpoint(&checkpoint);
        let snap_after = restored.snapshot_state();

        let before = snap_before
            .downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
            .unwrap();
        let after = snap_after
            .downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
            .unwrap();
        for i in 0..12 {
            assert_relative_eq!(before.0[i], after.0[i], epsilon = 1e-12);
            assert_relative_eq!(before.1[i], after.1[i], epsilon = 1e-12);
        }
    }
    /// plastic_zone 付きのテストモデルから塑性化域考慮 FiberBeam を生成する。
    fn make_plastic_zone_fiber(lp: f64, fy: Option<f64>) -> FiberBeam {
        let mut model = build_test_model(Some(0.0));
        model.elements[0].plastic_zone = Some(lp);
        model.materials[0].fy = fy;
        FiberBeam::with_plastic_zone(&model.elements[0], &model, lp)
    }

    #[test]
    fn test_plastic_zone_axial_stiffness_exact() {
        // 軸剛性は端部ファイバ(2Lp) + 中央弾性(L-2Lp) の合成で EA/L に厳密一致する
        let fb = make_plastic_zone_fiber(300.0, None);
        let ctx = Ctx {
            model: &build_test_model(Some(0.0)),
        };
        let k = fb.tangent_stiffness(&ElemState::default(), &ctx);
        let ea_over_l = 205000.0 * 20000.0 / 3000.0;
        assert_relative_eq!(k.get(0, 0), ea_over_l, max_relative = 1e-9);
    }

    #[test]
    fn test_plastic_zone_elastic_stiffness_close_to_full_fiber() {
        // Lp が小さければ弾性剛性は全長ファイバー積分（=弾性梁）に漸近する。
        // 端部の1点矩形則による誤差は O(Lp/L)（曲率分布の勾配×区間幅）で、
        // Lp = L/20 なら数%以内に収まる。
        let model = build_test_model(Some(0.0));
        let ctx = Ctx { model: &model };
        let full = FiberBeam::new(&model.elements[0], &model);
        let k_full = full.tangent_stiffness(&ElemState::default(), &ctx);

        let pz = make_plastic_zone_fiber(150.0, None); // Lp = L/20
        let k_pz = pz.tangent_stiffness(&ElemState::default(), &ctx);
        for (i, j) in [(1usize, 1usize), (2, 2), (4, 4), (5, 5), (1, 5), (2, 4)] {
            assert_relative_eq!(k_pz.get(i, j), k_full.get(i, j), max_relative = 5e-2);
        }
    }

    #[test]
    fn test_plastic_zone_yield_reduces_stiffness() {
        // 端部断面が降伏すると接線剛性が低下する（中央は弾性のまま）
        let mut fb = make_plastic_zone_fiber(300.0, Some(235.0));
        let model = build_test_model(Some(0.0));
        let ctx = Ctx { model: &model };
        let k0 = fb.tangent_stiffness(&ElemState::default(), &ctx);

        // i端に大回転 → 端部断面降伏
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fb.update_state(&du, false, &ctx);
        let k1 = fb.tangent_stiffness(&ElemState::default(), &ctx);
        assert!(
            k1.get(4, 4) < 0.9 * k0.get(4, 4),
            "降伏後の回転剛性は低下するはず: k0={}, k1={}",
            k0.get(4, 4),
            k1.get(4, 4)
        );
        // 中央弾性部があるため完全にゼロにはならない
        assert!(k1.get(4, 4) > 0.0);
    }

    #[test]
    fn test_plastic_zone_checkpoint_roundtrip() {
        let mut fb = make_plastic_zone_fiber(300.0, Some(235.0));
        let model = build_test_model(Some(0.0));
        let ctx = Ctx { model: &model };
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.02, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fb.update_state(&du, true, &ctx);
        let cp = fb.serialize_checkpoint();

        let mut fb2 = make_plastic_zone_fiber(300.0, Some(235.0));
        fb2.deserialize_checkpoint(&cp);
        let du2 = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fb.update_state(&du2, false, &ctx);
        fb2.update_state(&du2, false, &ctx);
        let f1 = fb.internal_force(&ElemState::default(), &ctx);
        let f2 = fb2.internal_force(&ElemState::default(), &ctx);
        for i in 0..12 {
            assert_relative_eq!(f1.data[i], f2.data[i], epsilon = 1e-6);
        }
    }
}
