use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
use sc_core::dof::DofMap;
use sc_core::model::Model;
use sc_element::behavior::MassOption;
use sc_math::solver::{make_solver, SolveError, SolverBackend};

const EIGEN_TOL: f64 = 1e-10;
const EIGEN_MAX_ITER: usize = 200;

pub struct ModalResult {
    pub omega2: Vec<f64>,
    pub period: Vec<f64>,
    pub shapes: Vec<Vec<f64>>,
    pub participation: Vec<[f64; 3]>,
    pub effective_mass: Vec<[f64; 3]>,
}

pub fn solve_eigen(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    n_modes: usize,
) -> Result<ModalResult, SolveError> {
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let m_red = reducer.reduce_k(&m_free);
    let n = m_red.nrows();
    // 自由度数（縮約後）を超えるモードは存在しないので上限で抑える。
    let n_modes = n_modes.min(n);
    if n == 0 || n_modes == 0 {
        return Ok(ModalResult {
            omega2: vec![],
            period: vec![],
            shapes: vec![],
            participation: vec![],
            effective_mass: vec![],
        });
    }

    let k_free = assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_red)?;

    // 部分空間サイズ q ≈ min(2n_modes, n_modes+8)。ただし行列次元 n を超えない。
    let q = (2 * n_modes).max(n_modes + 2).min(n);

    let mut x = init_subspace(n, q);

    let mut theta_prev = vec![f64::MAX; n_modes];
    let mut eigvecs_q = vec![0.0; q * q];

    for _iteration in 0..EIGEN_MAX_ITER {
        let mut y = vec![0.0; n * q];
        for col in 0..q {
            let rhs: Vec<f64> = (0..n).map(|r| x[r * q + col]).collect();
            let yi = solver.solve(&rhs)?;
            for r in 0..n {
                y[r * q + col] = yi[r];
            }
        }

        let k_bar = proj_yty(&y, &k_red, n, q);
        let m_bar = proj_yty(&y, &m_red, n, q);

        let (eigenvalues, eigvecs) = gevd_jacobi(&k_bar, &m_bar, q)?;

        eigvecs_q = eigvecs;

        let mut x_new = vec![0.0; n * q];
        for i in 0..n {
            for j in 0..q {
                let mut s = 0.0;
                for k in 0..q {
                    s += y[i * q + k] * eigvecs_q[k * q + j];
                }
                x_new[i * q + j] = s;
            }
        }
        x = x_new;

        let mut converged = 0;
        for m in 0..n_modes {
            let th = eigenvalues[m];
            if (th - theta_prev[m]).abs() < EIGEN_TOL * th.max(1.0) {
                converged += 1;
            }
            theta_prev[m] = th;
        }
        if converged == n_modes {
            break;
        }
    }

    let mut omega2 = vec![0.0; n_modes];
    let mut period = vec![0.0; n_modes];
    let mut shapes = Vec::with_capacity(n_modes);

    for m in 0..n_modes {
        omega2[m] = theta_prev[m];
        period[m] = if omega2[m] > 0.0 {
            2.0 * std::f64::consts::PI / omega2[m].sqrt()
        } else {
            0.0
        };

        let mut phi = vec![0.0; n];
        for i in 0..n {
            phi[i] = x[i * q + m];
        }
        let norm2 = m_norm(&phi, &m_red, n);
        if norm2 > 0.0 {
            let inv = 1.0 / norm2.sqrt();
            for v in &mut phi {
                *v *= inv;
            }
        }
        shapes.push(phi);
    }

    let (participation, effective_mass) =
        compute_participation(&shapes, &m_free, &m_red, reducer, dofmap, model);

    Ok(ModalResult {
        omega2,
        period,
        shapes,
        participation,
        effective_mass,
    })
}

fn init_subspace(n: usize, q: usize) -> Vec<f64> {
    let mut x = vec![0.0; n * q];
    for i in 0..q.min(n) {
        x[i * q + i] = 1.0;
    }
    for col in n..q {
        x[(col % n) * q + col] = 0.1;
    }
    x
}

fn proj_yty(
    y: &[f64],
    mat_red: &faer::sparse::SparseColMat<usize, f64>,
    n: usize,
    q: usize,
) -> Vec<f64> {
    let mut result = vec![0.0; q * q];
    for i in 0..q {
        for j in 0..=i {
            let mut s = 0.0;
            for a in 0..n {
                let mut tmp = 0.0;
                for b in 0..n {
                    tmp += mat_red.get(a, b).copied().unwrap_or(0.0) * y[b * q + j];
                }
                s += y[a * q + i] * tmp;
            }
            result[i * q + j] = s;
            result[j * q + i] = s;
        }
    }
    result
}

fn m_norm(phi: &[f64], m_red: &faer::sparse::SparseColMat<usize, f64>, n: usize) -> f64 {
    let mut norm2 = 0.0;
    for a in 0..n {
        let mut tmp = 0.0;
        for b in 0..n {
            tmp += m_red.get(a, b).copied().unwrap_or(0.0) * phi[b];
        }
        norm2 += phi[a] * tmp;
    }
    norm2
}

/// Generalized eigenvalue problem K*z = θ*M*z via Cholesky transform + Jacobi.
/// Returns (eigenvalues ascending, eigenvectors as columns).
fn gevd_jacobi(k: &[f64], m: &[f64], n: usize) -> Result<(Vec<f64>, Vec<f64>), SolveError> {
    let mut l = vec![0.0; n * n];
    for j in 0..n {
        let mut s = 0.0;
        for k in 0..j {
            s += l[j * n + k] * l[j * n + k];
        }
        let d = m[j * n + j] - s;
        if d <= 1e-30 {
            return Ok(diag_fallback(k, m, n));
        }
        l[j * n + j] = d.sqrt();
        for i in (j + 1)..n {
            let mut s = 0.0;
            for k in 0..j {
                s += l[i * n + k] * l[j * n + k];
            }
            l[i * n + j] = (m[i * n + j] - s) / l[j * n + j];
        }
    }

    let a = transform_to_standard(k, &l, n);

    let (eigvals_w, eigvecs_w) = jacobi_evd(&a, n);

    let mut z = vec![0.0; n * n];
    for j in 0..n {
        for i in 0..n {
            let mut s = 0.0;
            for k in 0..n {
                s += l[i * n + k] * eigvecs_w[k * n + j];
            }
            z[i * n + j] = s;
        }
    }

    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| eigvals_w[a].partial_cmp(&eigvals_w[b]).unwrap());

    let mut sorted_vals = vec![0.0; n];
    let mut sorted_vecs = vec![0.0; n * n];
    for (new_pos, &orig) in idx.iter().enumerate() {
        sorted_vals[new_pos] = eigvals_w[orig];
        for i in 0..n {
            sorted_vecs[i * n + new_pos] = z[i * n + orig];
        }
    }

    Ok((sorted_vals, sorted_vecs))
}

fn diag_fallback(k: &[f64], m: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut vals = vec![0.0; n];
    let mut vecs = vec![0.0; n * n];
    for i in 0..n {
        vals[i] = if m[i * n + i].abs() > 1e-30 {
            k[i * n + i] / m[i * n + i]
        } else {
            f64::MAX
        };
        vecs[i * n + i] = 1.0;
    }
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| vals[a].partial_cmp(&vals[b]).unwrap());
    let mut sv = vec![0.0; n];
    let mut sve = vec![0.0; n * n];
    for (np, &orig) in idx.iter().enumerate() {
        sv[np] = vals[orig];
        for i in 0..n {
            sve[i * n + np] = vecs[i * n + orig];
        }
    }
    (sv, sve)
}

/// A = L^(-1) * K * L^(-T)
fn transform_to_standard(k: &[f64], l: &[f64], n: usize) -> Vec<f64> {
    let mut tmp = vec![0.0; n * n];
    for j in 0..n {
        for i in 0..n {
            let mut s = k[i * n + j];
            for kk in 0..i {
                s -= l[i * n + kk] * tmp[kk * n + j];
            }
            tmp[i * n + j] = s / l[i * n + i];
        }
    }
    let mut a = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            let mut s = tmp[i * n + j];
            for kk in 0..j {
                s -= a[i * n + kk] * l[j * n + kk];
            }
            a[i * n + j] = s / l[j * n + j];
        }
    }
    for i in 0..n {
        for j in (i + 1)..n {
            a[i * n + j] = a[j * n + i];
        }
    }
    a
}

/// Classical Jacobi eigenvalue decomposition for symmetric matrix.
/// Returns (eigenvalues, eigenvectors as columns).
fn jacobi_evd(a_in: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut a = a_in.to_vec();
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }

    const MAX_SWEEPS: usize = 100;
    const EPS: f64 = 1e-14;

    for _ in 0..MAX_SWEEPS {
        let mut off = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                off += a[i * n + j].abs();
            }
        }
        if off < EPS {
            break;
        }

        for p in 0..n {
            for q in (p + 1)..n {
                let apq = a[p * n + q];
                if apq.abs() < EPS {
                    continue;
                }
                let app = a[p * n + p];
                let aqq = a[q * n + q];
                let theta = (aqq - app) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (1.0 + theta * theta).sqrt());
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = t * c;

                for i in 0..n {
                    let aip = a[i * n + p];
                    let aiq = a[i * n + q];
                    a[i * n + p] = c * aip - s * aiq;
                    a[i * n + q] = s * aip + c * aiq;
                }
                for i in 0..n {
                    let api = a[p * n + i];
                    let aqi = a[q * n + i];
                    a[p * n + i] = c * api - s * aqi;
                    a[q * n + i] = s * api + c * aqi;
                }
                for i in 0..n {
                    let vip = v[i * n + p];
                    let viq = v[i * n + q];
                    v[i * n + p] = c * vip - s * viq;
                    v[i * n + q] = s * vip + c * viq;
                }
            }
        }
    }

    let eigvals: Vec<f64> = (0..n).map(|i| a[i * n + i]).collect();
    (eigvals, v)
}

fn compute_participation(
    shapes: &[Vec<f64>],
    m_free: &faer::sparse::SparseColMat<usize, f64>,
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    reducer: &Reducer,
    dofmap: &DofMap,
    model: &Model,
) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let n_modes = shapes.len();
    let mut participation = vec![[0.0; 3]; n_modes];
    let mut effective_mass = vec![[0.0; 3]; n_modes];

    let n_free = dofmap.n_active();
    let n_nodes = model.nodes.len();

    for dir_idx in 0..3 {
        let mut r_free = vec![0.0; n_free];
        for ni in 0..n_nodes {
            let g = ni * sc_core::dof::DOF_PER_NODE + dir_idx;
            if let Some(active) = dofmap.active(g) {
                r_free[active as usize] = 1.0;
            }
        }

        for (m_idx, phi_red) in shapes.iter().enumerate() {
            let phi_free = reducer.expand_u(phi_red);

            let mut m_phi = vec![0.0; n_free];
            for a in 0..n_free {
                let mut s = 0.0;
                for b in 0..n_free {
                    s += m_free.get(a, b).copied().unwrap_or(0.0) * phi_free[b];
                }
                m_phi[a] = s;
            }

            let mut phi_m_phi = 0.0;
            for a in 0..n_free {
                phi_m_phi += phi_free[a] * m_phi[a];
            }

            let mut phi_m_r = 0.0;
            for a in 0..n_free {
                phi_m_r += m_phi[a] * r_free[a];
            }

            if phi_m_phi.abs() > 1e-30 {
                participation[m_idx][dir_idx] = phi_m_r / phi_m_phi;
                effective_mass[m_idx][dir_idx] = phi_m_r * phi_m_r / phi_m_phi;
            }
        }
    }

    let _ = m_red;
    (participation, effective_mass)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint::Reducer;
    use sc_core::dof::{Dof6Mask, DofMap};
    use sc_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use sc_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        Section,
    };

    /// Ux のみ自由（並進1方向）にするマスク。
    const FREE_UX: Dof6Mask = Dof6Mask(0b111110);

    /// 軸ばね 1 本（剛性 k=EA/L）＋先端質量 m の 1 自由度モデル。
    /// node0 固定、node1 は Ux のみ自由で質量 m を持つ。
    /// 理論固有周期 T = 2π√(m/k)。
    fn make_1dof_spring_model() -> Model {
        let k = 1000.0_f64;
        let m = 1.0_f64;
        Model {
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
                    coord: [1000.0, 0.0, 0.0],
                    restraint: FREE_UX,
                    mass: Some([m, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(1),
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
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "spring".into(),
                area: 1.0,
                iy: 1.0,
                iz: 1.0,
                j: 1.0,
                depth: 1.0,
                width: 1.0,
                as_y: 1.0,
                as_z: 1.0,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".into(),
                young: k * 1000.0 / 1.0, // EA/L = young*1/1000 = k
                poisson: 0.0,
                density: 0.0,
                shear: None,
            }],
            ..Default::default()
        }
    }

    /// 2層等質量等剛性せん断モデル（軸ばね2本の直列）。
    /// node0 固定、node1/node2 は Ux のみ自由で各質量 m。
    /// K=[[2k,-k],[-k,k]], M=mI。λ=(k/m)(3∓√5)/2。
    fn make_shear_2dof_model() -> Model {
        let k = 1000.0_f64;
        let m = 1.0_f64;
        let young = k * 1000.0; // EA/L = young*1/1000 = k
        let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
            id: NodeId(id),
            coord: [x, 0.0, 0.0],
            restraint,
            mass,
            story: None,
        };
        let beam = |id: u32, a: u32, b: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        Model {
            nodes: vec![
                node(0, 0.0, Dof6Mask::FIXED, None),
                node(1, 1000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
                node(2, 2000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
            ],
            elements: vec![beam(1, 0, 1), beam(2, 1, 2)],
            sections: vec![Section {
                id: SectionId(0),
                name: "spring".into(),
                area: 1.0,
                iy: 1.0,
                iz: 1.0,
                j: 1.0,
                depth: 1.0,
                width: 1.0,
                as_y: 1.0,
                as_z: 1.0,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".into(),
                young,
                poisson: 0.0,
                density: 0.0,
                shear: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_jacobi_2x2() {
        let a = vec![2.0, 1.0, 1.0, 3.0];
        let (vals, vecs) = jacobi_evd(&a, 2);
        let mut sorted = vals.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let expected_1 = (5.0 - 5.0_f64.sqrt()) / 2.0;
        let expected_2 = (5.0 + 5.0_f64.sqrt()) / 2.0;
        assert!((sorted[0] - expected_1).abs() < 1e-10, "val0={}", sorted[0]);
        assert!((sorted[1] - expected_2).abs() < 1e-10, "val1={}", sorted[1]);
        for j in 0..2 {
            let mut norm = 0.0;
            for i in 0..2 {
                norm += vecs[i * 2 + j] * vecs[i * 2 + j];
            }
            assert!((norm - 1.0).abs() < 1e-10, "vec{} not normalized", j);
        }
    }

    #[test]
    fn test_1dof_period() {
        let k = 1000.0_f64;
        let m = 1.0_f64;
        let expected_omega2 = k / m;
        let expected_t = 2.0 * std::f64::consts::PI / expected_omega2.sqrt();

        let model = make_1dof_spring_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();

        // 質量・自由度が正しく組まれていれば 1 モードが得られる。
        assert_eq!(result.omega2.len(), 1, "1 モードが解けていない");
        assert!(result.omega2[0] > 0.0, "omega2={}", result.omega2[0]);
        // 反復解法だが SPD 1 自由度なので高精度に収束する（理論一致・許容差）。
        assert!(
            (result.omega2[0] - expected_omega2).abs() / expected_omega2 < 1e-8,
            "omega2={} expected={}",
            result.omega2[0],
            expected_omega2
        );
        // 設計書 §7.2 の例: T = 0.198692 s
        assert!(
            (result.period[0] - expected_t).abs() / expected_t < 1e-8,
            "T={} expected={}",
            result.period[0],
            expected_t
        );
        assert!(
            (result.period[0] - 0.198692).abs() < 1e-5,
            "T={} 設計書例 0.198692 と不一致",
            result.period[0]
        );
    }

    /// 2層せん断モデル: T1=0.32150, T2=0.12280 へ収束し、
    /// 2 モードで有効質量比合計 ≈100%（設計書 §7.2）。
    #[test]
    fn test_2dof_shear_period_and_mass() {
        let model = make_shear_2dof_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = solve_eigen(&model, &dofmap, &reducer, 2).unwrap();

        assert_eq!(result.omega2.len(), 2);
        let k = 1000.0_f64;
        let m = 1.0_f64;
        let lam1 = (k / m) * (3.0 - 5.0_f64.sqrt()) / 2.0; // ≈382.0
        let lam2 = (k / m) * (3.0 + 5.0_f64.sqrt()) / 2.0; // ≈2618.0

        assert!(
            (result.omega2[0] - lam1).abs() / lam1 < 1e-6,
            "λ1={} expected={}",
            result.omega2[0],
            lam1
        );
        assert!(
            (result.omega2[1] - lam2).abs() / lam2 < 1e-6,
            "λ2={} expected={}",
            result.omega2[1],
            lam2
        );

        let t1 = 2.0 * std::f64::consts::PI / result.omega2[0].sqrt();
        let t2 = 2.0 * std::f64::consts::PI / result.omega2[1].sqrt();
        assert!((t1 - 0.32150).abs() < 1e-4, "T1={}", t1);
        assert!((t2 - 0.12280).abs() < 1e-4, "T2={}", t2);

        // X 方向有効質量の合計が全質量 2m に一致（有効質量比合計 ≈100%）。
        let total_mass = 2.0 * m;
        let eff_sum: f64 = result.effective_mass.iter().map(|e| e[0]).sum();
        assert!(
            (eff_sum - total_mass).abs() / total_mass < 1e-6,
            "有効質量合計={} 全質量={}",
            eff_sum,
            total_mass
        );
        // モード1 が支配的。理論値は閉形式から求める（このKでは ≈94.7%）。
        // 1次モード形 φ=[1, k/(k−λ1)] より Meff1 = (Σφ)²/(Σφ²)。
        let s = k / (k - lam1); // φ2/φ1
        let meff1_theory = (1.0 + s).powi(2) / (1.0 + s * s);
        let ratio1 = result.effective_mass[0][0] / total_mass;
        assert!(
            (ratio1 - meff1_theory / total_mass).abs() < 1e-6,
            "mode1 有効質量比={} 理論={}",
            ratio1,
            meff1_theory / total_mass
        );
    }

    /// 決定性テスト: 固有値解析を10回実行しビット一致を確認
    #[test]
    fn test_eigen_deterministic() {
        let model = make_1dof_spring_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let first = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();
        for _ in 0..9 {
            let cur = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();
            assert_eq!(first.omega2.len(), cur.omega2.len());
            for (a, b) in first.omega2.iter().zip(cur.omega2.iter()) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
            for (a, b) in first.period.iter().zip(cur.period.iter()) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
            for (s_a, s_b) in first.shapes.iter().zip(cur.shapes.iter()) {
                assert_eq!(s_a.len(), s_b.len());
                for (va, vb) in s_a.iter().zip(s_b.iter()) {
                    assert_eq!(va.to_bits(), vb.to_bits());
                }
            }
        }
    }
}
