use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::MassOption;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};

const EIGEN_TOL: f64 = 1e-10;
const EIGEN_MAX_ITER: usize = 200;
/// 部分空間内で射影質量行列 M̄ の固有値をこの相対値未満とみなしたら
/// 「質量を持たない方向」として扱う（質量ランク判定の相対許容誤差）。
const MASS_RANK_REL_TOL: f64 = 1e-9;

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

    // 質量ゼロ（密度・節点質量とも未設定）の検出。
    // M ≈ 0 のまま進めると GEVD が対角フォールバックし周期 0 の無意味な結果になる。
    let mass_trace: f64 = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .sum();
    if mass_trace <= 0.0 {
        return Err(SolveError::InvalidInput(
            "質量がゼロです。材料の密度(ρ)を設定するか、節点質量を与えてください。".into(),
        ));
    }

    let k_free = assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_red)?;

    // 部分空間サイズ q: Bathe の定石 q = min(2p, p+8) にならい、要求モード数 p に対して
    // オーバーサンプリングする（p が大きいときに q が際限なく増えて計算コストが
    // 爆発しないよう +8 側で頭打ちにする）。ただし少なくとも p+1 は確保し
    // （p=1 でも部分空間に余裕を持たせ収束を安定させる）、行列次元 n は超えない。
    let q = ((2 * n_modes).min(n_modes + 8)).max(n_modes + 1).min(n);

    // 開始ベクトルは Bathe の部分空間反復の定石に従い、質量情報を使って選ぶ
    // （単純に自由度番号の若い順に単位ベクトルを選ぶと、回転自由度など質量ゼロの
    // 自由度ばかりを拾ってしまい、水平質点系モデルのように質量を持つ自由度が
    // 少数・偏在するモデルで q が実際の質量ランクより小さいと、質量を持つ自由度が
    // 開始部分空間に一本も入らず反復が正しい低次モードへ収束できないことがある）。
    let k_diag: Vec<f64> = (0..n)
        .map(|i| k_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let m_diag: Vec<f64> = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let mut x = init_subspace(n, q, &k_diag, &m_diag);

    let mut theta_prev = vec![f64::MAX; n_modes];
    let mut is_converged = false;
    // 質量ランク不足の判定に使う: 最後に計算した部分空間内の固有値（昇順、
    // 質量ゼロ方向は +∞ になる）。
    let mut last_eigenvalues = vec![f64::MAX; q];

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

        let (eigenvalues, eigvecs_q) = gevd_jacobi(&k_bar, &m_bar, q);

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
            // 質量ゼロ方向（θ=+∞）が2回連続で現れた場合も「安定した」とみなし、
            // 無限大同士の減算で NaN になって収束判定が永久に false になるのを防ぐ。
            let same = if th.is_finite() && theta_prev[m].is_finite() {
                (th - theta_prev[m]).abs() < EIGEN_TOL * th.max(1.0)
            } else {
                th == theta_prev[m]
            };
            if same {
                converged += 1;
            }
            theta_prev[m] = th;
        }
        last_eigenvalues = eigenvalues;
        if converged == n_modes {
            is_converged = true;
            break;
        }
    }

    if !is_converged {
        return Err(SolveError::NonConvergence(format!(
            "固有値解析(部分空間反復)が {} 回で収束しませんでした。モデルの質量・剛性の分布を確認してください。",
            EIGEN_MAX_ITER
        )));
    }

    // 質量ランク不足チェック: 要求モード数 n_modes に対し、質量が有効な
    // （θ が有限な）方向が n_modes 個に満たない場合、f64::MAX 等を結果に混ぜず
    // 明示エラーとする。gevd_jacobi は質量ゼロ方向の θ を昇順の末尾に +∞ として
    // 返すため、theta_prev の先頭 n_modes 個のうち有限な個数がそのまま
    // （この部分空間内で判定できた）質量ランクになる。
    let mass_rank = last_eigenvalues.iter().filter(|v| v.is_finite()).count();
    if theta_prev.iter().any(|v| !v.is_finite()) {
        return Err(SolveError::InvalidInput(format!(
            "固有値解析: 要求モード数({n_modes})に対し、質量が有効な独立自由度が{mass_rank}個しか見つかりませんでした。\
node.mass や材料の密度(ρ)で並進質量を追加するか、要求モード数を{mass_rank}以下に減らしてください。"
        )));
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

/// 部分空間反復の開始ベクトルを Bathe の定石に従って選ぶ。
///
/// 1本目は質量分布に比例した変位パターン（各自由度の集中質量そのもの）。
/// 残り q-1 本は、剛性/質量比 k_ii/m_ii が小さい（＝質量が相対的に効いていて
/// 低次モードに寄与しやすい）自由度から順に単位ベクトルを割り当てる。
/// 質量ゼロの自由度は比を +∞ とみなし、質量を持つ自由度が尽きない限り選ばれない。
/// こうすることで、q が要求モード数程度に小さくても、質量を持つ自由度が
/// 少数・偏在するモデル（例: 水平質点系モデル化）で開始部分空間から
/// 質量を持つ方向が漏れることを防ぐ。
fn init_subspace(n: usize, q: usize, k_diag: &[f64], m_diag: &[f64]) -> Vec<f64> {
    let mut x = vec![0.0; n * q];
    if q == 0 {
        return x;
    }
    for i in 0..n {
        x[i * q] = m_diag[i];
    }
    let mut ratios: Vec<(usize, f64)> = (0..n)
        .map(|i| {
            let r = if m_diag[i] > 0.0 {
                k_diag[i] / m_diag[i]
            } else {
                f64::INFINITY
            };
            (i, r)
        })
        .collect();
    ratios.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    for col in 1..q {
        let dof = ratios[col - 1].0;
        x[dof * q + col] = 1.0;
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

/// Generalized eigenvalue problem K*z = θ*M*z。
///
/// M は理論上は半正定値だが、部分空間反復の作業次元 q が実際の質量ランク r を
/// 超える場合（回転自由度など質量を持たない自由度が混在するモデルでは一般的）、
/// 射影質量行列 M̄ は必ずランク落ち（半正定値だが正定値でない）になる。
/// Cholesky 分解はこの場合ピボットが 0 に潰れて失敗するため、以前の実装は
/// 「対角成分のみで θ=k/m を計算する」という数学的に誤った近似
/// （非対角の結合を無視し、質量ゼロ方向には f64::MAX を注入する）にフォールバック
/// していた。これは q が要求モード数よりオーバーサンプリングされている限り
/// ほぼ必ず発生し、結果に f64::MAX が混入する原因になっていた。
///
/// 正しい扱いは、M̄ 自体を固有分解して「質量を持つ部分空間」（固有値 > 0）と
/// 「質量を持たない部分空間」（固有値 ≈ 0）に分離し、質量を持つ部分空間内だけで
/// 標準固有値問題に変換して解くこと。質量を持たない方向には物理的な固有振動数が
/// 存在しないため、θ=+∞（有限な f64::MAX ではなく明示的な無限大）を割り当て、
/// 呼び出し側で「要求モード数に対して質量ランクが不足している」ことを検出できる
/// ようにする。
///
/// Returns (eigenvalues ascending, +∞ が質量ゼロ方向; eigenvectors as columns).
fn gevd_jacobi(k: &[f64], m: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let (mu, u) = jacobi_evd(m, n);
    let mu_max = mu.iter().cloned().fold(0.0_f64, f64::max);

    let tol = MASS_RANK_REL_TOL * mu_max;
    let mut kept: Vec<usize> = Vec::new();
    let mut dropped: Vec<usize> = Vec::new();
    for i in 0..n {
        if mu_max > 0.0 && mu[i] > tol {
            kept.push(i);
        } else {
            dropped.push(i);
        }
    }
    let r = kept.len();

    let mut vals = vec![f64::INFINITY; n];
    let mut vecs = vec![0.0; n * n];

    if r > 0 {
        // W の列 = M̄-固有ベクトル / sqrt(質量固有値)。W^T M̄ W = I_r となる
        // （質量に関して正規直交な）基底で、質量を持つ部分空間だけを張る。
        let mut w = vec![0.0; n * r];
        for (col, &ki) in kept.iter().enumerate() {
            let inv_sqrt = 1.0 / mu[ki].sqrt();
            for row in 0..n {
                w[row * r + col] = u[row * n + ki] * inv_sqrt;
            }
        }

        // A = W^T K W（r×r 対称行列）。この基底では一般化固有値問題が
        // A z' = θ z' という標準固有値問題になる。
        let a = mat_wt_k_w(k, &w, n, r);
        let (theta_r, v_r) = jacobi_evd(&a, r);

        // 元の q 次元部分空間へ戻す: Z = W * V。
        for col in 0..r {
            vals[col] = theta_r[col];
            for row in 0..n {
                let mut s = 0.0;
                for l in 0..r {
                    s += w[row * r + l] * v_r[l * r + col];
                }
                vecs[row * n + col] = s;
            }
        }
    }

    // 質量ゼロ方向は θ=+∞。固有ベクトルは M̄ の（質量的に意味を持たない）
    // 対応する固有ベクトルをそのまま使う（直交性は保たれ、次の反復の
    // K^-1 変換に使っても数値的に安全）。
    for (offset, &di) in dropped.iter().enumerate() {
        let col = r + offset;
        for row in 0..n {
            vecs[row * n + col] = u[row * n + di];
        }
    }

    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| vals[a].partial_cmp(&vals[b]).unwrap());

    let mut sorted_vals = vec![0.0; n];
    let mut sorted_vecs = vec![0.0; n * n];
    for (new_pos, &orig) in idx.iter().enumerate() {
        sorted_vals[new_pos] = vals[orig];
        for i in 0..n {
            sorted_vecs[i * n + new_pos] = vecs[i * n + orig];
        }
    }

    (sorted_vals, sorted_vecs)
}

/// A = W^T K W （n×r の W と n×n の K から r×r 対称行列を作る）。
fn mat_wt_k_w(k: &[f64], w: &[f64], n: usize, r: usize) -> Vec<f64> {
    // kw = K * W (n×r)
    let mut kw = vec![0.0; n * r];
    for i in 0..n {
        for j in 0..r {
            let mut s = 0.0;
            for l in 0..n {
                s += k[i * n + l] * w[l * r + j];
            }
            kw[i * r + j] = s;
        }
    }
    // a = W^T * kw (r×r)
    let mut a = vec![0.0; r * r];
    for i in 0..r {
        for j in 0..r {
            let mut s = 0.0;
            for l in 0..n {
                s += w[l * r + i] * kw[l * r + j];
            }
            a[i * r + j] = s;
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
            let g = ni * squid_n_core::dof::DOF_PER_NODE + dir_idx;
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
    use squid_n_core::dof::{Dof6Mask, DofMap};
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
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
                plastic_zone: None,
                spring: None,
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
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".into(),
                young: k * 1000.0 / 1.0, // EA/L = young*1/1000 = k
                poisson: 0.0,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
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
            plastic_zone: None,
            spring: None,
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
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".into(),
                young,
                poisson: 0.0,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    /// 門型ラーメン相当モデル（柱2本＋梁1本、柱脚固定、柱頭2節点は6自由度すべて自由）。
    /// crates/squid-n-app の sample::portal_frame() を模したジオメトリだが、
    /// 材料密度は 0 とし、質量は柱頭2節点の水平(Ux)自由度のみに集中質量として与える
    /// （実務でよく使う「水平質点系」モデル化）。
    /// 縮約後自由度は 12(=2節点×6) あるが、質量を持つ自由度はそのうち 2 つだけなので、
    /// 縮約後質量行列 M_red のランクは厳密に 2 になる（10自由度は完全に質量ゼロ）。
    fn make_portal_frame_like_model(top_mass: f64) -> Model {
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
        ];
        let nodes = coords
            .iter()
            .enumerate()
            .map(|(i, c)| Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: if i >= 2 && top_mass > 0.0 {
                    Some([top_mass, 0.0, 0.0, 0.0, 0.0, 0.0])
                } else {
                    None
                },
                story: None,
            })
            .collect();
        let col_section = Section {
            id: SectionId(0),
            name: "col".into(),
            area: 11980.0,
            iy: 2.04e8,
            iz: 6.75e7,
            j: 3.54e6,
            depth: 300.0,
            width: 300.0,
            as_y: 6000.0,
            as_z: 6000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let beam_section = Section {
            id: SectionId(1),
            name: "beam".into(),
            area: 8337.0,
            iy: 2.37e8,
            iz: 2.62e7,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 4000.0,
            as_z: 4000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let members = [(0u32, 1u32, 2u32, 0u32), (1, 1, 3, 0), (2, 2, 3, 1)];
        let elements = members
            .iter()
            .map(|&(id, i, j, sec)| ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
                section: Some(SectionId(sec)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: if sec == 0 {
                        [1.0, 0.0, 0.0]
                    } else {
                        [0.0, 0.0, 1.0]
                    },
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            })
            .collect();
        Model {
            nodes,
            elements,
            sections: vec![col_section, beam_section],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "SN400B".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0, // 質量は集中質量のみで与える（水平質点系モデル化）
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            ..Default::default()
        }
    }

    /// 再現テスト: 質量ランク(=2) が要求モード数(2)ちょうどでも、
    /// 部分空間反復の作業次元 q(>2) の中で射影質量行列がランク落ちするため、
    /// 修正前の実装は Cholesky 分解失敗 → 対角フォールバック(diag_fallback)に落ち、
    /// 質量ゼロ方向の θ=k/m を素朴に計算して f64::MAX を混入させていた
    /// （diag_fallback は projected な q×q 行列を「対角」とみなす近似で、
    /// 非対角の結合を無視するため、q>質量ランクでは必ず不正確になる）。
    /// 修正後は質量固有分解によりランク落ちを正しく分離し、2 つの有限な固有値が
    /// 得られることを確認する。
    #[test]
    fn test_eigen_portal_frame_like_mass_rank_equals_n_modes() {
        let model = make_portal_frame_like_model(1.0e-3);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = solve_eigen(&model, &dofmap, &reducer, 2)
            .expect("質量ランクが要求モード数以上なら解けるべき");

        assert_eq!(result.omega2.len(), 2);
        for (i, &w2) in result.omega2.iter().enumerate() {
            assert!(
                w2.is_finite() && w2 > 0.0,
                "mode{}: omega2={} は有限な正値であるべき(f64::MAX 混入は禁止)",
                i,
                w2
            );
        }
        for (i, &t) in result.period.iter().enumerate() {
            assert!(
                t.is_finite() && t > 0.0,
                "mode{}: period={} は有限な正値であるべき",
                i,
                t
            );
        }
        // 周期は昇順のモードで降順（1次が最長周期）。
        assert!(
            result.period[0] > result.period[1],
            "T1={} T2={} は T1>T2 であるべき",
            result.period[0],
            result.period[1]
        );
    }

    /// 質量ランク不足(ランク1)で2モードを要求した場合は、f64::MAX を混ぜて返さず、
    /// 日本語の明示エラーを返すことを確認する。
    #[test]
    fn test_eigen_mass_rank_deficient_returns_explicit_error() {
        let mut model = make_portal_frame_like_model(1.0e-3);
        // node3(柱頭2つ目)の質量を落とし、質量ランクを 1 にする。
        model.nodes[3].mass = None;
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = solve_eigen(&model, &dofmap, &reducer, 2);
        let err = match result {
            Err(e) => e,
            Ok(r) => panic!(
                "質量ランク(1) < 要求モード数(2) はエラーになるべきだが omega2={:?} が返った",
                r.omega2
            ),
        };
        let msg = err.to_string();
        assert!(
            msg.contains("質量"),
            "エラーメッセージは質量ランク不足を説明すべき: {}",
            msg
        );
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

    /// crates/squid-n-app の sample::portal_frame() と等価なモデル
    /// （柱2本＋梁1本、柱脚固定、H形断面、材料密度あり・節点集中質量なし）。
    /// 断面性能は H-300x300x10x15（柱）・H-400x200x8x13（梁）の実断面計算値。
    /// 質量は一貫質量行列(consistent mass)のみから生じ、並進DOFに比べ
    /// 回転DOFの質量ははるかに小さい（質量行列が病的に悪条件）。
    /// この状態で eigen(1)〜eigen(3) が f64::MAX を返さず、妥当な周期を
    /// 返すことを確認する（実際に発生した不具合の再現・回帰テスト）。
    fn make_portal_frame_density_mass_model() -> Model {
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
        ];
        let nodes = coords
            .iter()
            .enumerate()
            .map(|(i, c)| Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            })
            .collect();
        let col_section = Section {
            id: SectionId(0),
            name: "col".into(),
            area: 11700.0,
            iy: 1.993275e8,
            iz: 6.75225e7,
            j: 7.65e5,
            depth: 300.0,
            width: 300.0,
            as_y: 6000.0,
            as_z: 6000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let beam_section = Section {
            id: SectionId(1),
            name: "beam".into(),
            area: 8192.0,
            iy: 2.2965e8,
            iz: 1.735e7,
            j: 3.568e5,
            depth: 400.0,
            width: 200.0,
            as_y: 4000.0,
            as_z: 4000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let members = [(0u32, 1u32, 2u32, 0u32), (1, 1, 3, 0), (2, 2, 3, 1)];
        let elements = members
            .iter()
            .map(|&(id, i, j, sec)| ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
                section: Some(SectionId(sec)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: if sec == 0 {
                        [1.0, 0.0, 0.0]
                    } else {
                        [0.0, 0.0, 1.0]
                    },
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            })
            .collect();
        Model {
            nodes,
            elements,
            sections: vec![col_section, beam_section],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "SN400B".into(),
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_eigen_portal_frame_density_mass_two_modes() {
        let model = make_portal_frame_density_mass_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = solve_eigen(&model, &dofmap, &reducer, 2)
            .expect("密度由来の質量でも eigen(2) は解けるべき");
        assert_eq!(result.omega2.len(), 2);
        for (i, &w2) in result.omega2.iter().enumerate() {
            assert!(
                w2.is_finite() && w2 > 0.0,
                "mode{}: omega2={} は有限な正値であるべき(f64::MAX 混入は禁止)",
                i,
                w2
            );
        }
        assert!(
            result.period[0] > result.period[1],
            "T1={} T2={} は T1>T2 であるべき",
            result.period[0],
            result.period[1]
        );
        // ラーメン(高さ3.5m・スパン6mの鋼構造)としては数百ms〜1s台が妥当なオーダー。
        assert!(
            result.period[0] > 0.01 && result.period[0] < 5.0,
            "T1={} は非物理的なオーダー",
            result.period[0]
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
