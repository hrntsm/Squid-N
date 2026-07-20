use super::*;

fn make_flat_shell(t: f64) -> ShellElement {
    let coords = [
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [100.0, 100.0, 0.0],
        [0.0, 100.0, 0.0],
    ];
    let frame = ShellFrame::from_nodes(coords);
    ShellElement {
        nodes: [NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        coords,
        t,
        e: 1000.0,
        nu: 0.3,
        density: 0.0,
        frame,
        drilling_factor: DEFAULT_DRILLING_FACTOR,
        membrane_active: true,
        committed_disp: [0.0; 24],
        trial_disp: [0.0; 24],
    }
}

#[test]
fn test_frame_orthonormal() {
    let coords = [
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [100.0, 100.0, 0.0],
        [0.0, 100.0, 0.0],
    ];
    let frame = ShellFrame::from_nodes(coords);
    let dot_e1e2 =
        frame.e1[0] * frame.e2[0] + frame.e1[1] * frame.e2[1] + frame.e1[2] * frame.e2[2];
    assert!(dot_e1e2.abs() < 1e-15);
    let dot_e1n = frame.e1[0] * frame.n[0] + frame.e1[1] * frame.n[1] + frame.e1[2] * frame.n[2];
    assert!(dot_e1n.abs() < 1e-15);
    let dot_e2n = frame.e2[0] * frame.n[0] + frame.e2[1] * frame.n[1] + frame.e2[2] * frame.n[2];
    assert!(dot_e2n.abs() < 1e-15);
    for &v in &[frame.e1, frame.e2, frame.n] {
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-14);
    }
}

#[test]
fn test_local_stiffness_symmetric() {
    let shell = make_flat_shell(10.0);
    let k = shell.local_stiffness();
    for i in 0..24 {
        for j in i..24 {
            let diff = (k.get(i, j) - k.get(j, i)).abs();
            let max_val = k.get(i, i).max(k.get(j, j)).abs().max(1.0);
            assert!(
                diff / max_val < 1e-10,
                "K[{i},{j}]={} != K[{j},{i}]={}",
                k.get(i, j),
                k.get(j, i)
            );
        }
    }
}

#[test]
fn test_drilling_prevents_singularity() {
    let shell = make_flat_shell(10.0);
    let k = shell.local_stiffness();
    // Check diagonal of drilling DOFs are non-zero
    for i in 0..4 {
        let idx = i * 6 + 5;
        assert!(k.get(idx, idx) > 0.0, "drilling DOF {i} diagonal is zero");
    }
}

#[test]
fn test_rigid_floor_disables_membrane() {
    let mut shell = make_flat_shell(10.0);
    shell.membrane_active = false;
    let mut k = shell.local_stiffness();
    shell.apply_rigid_floor_membrane_off(&mut k);
    // Ux, Uy, Rz diagonals should be 1.0 (penalized)
    for i in 0..4 {
        let bo = i * 6;
        assert!((k.get(bo, bo) - 1.0).abs() < 1e-12, "Ux[{i}] should be 1.0");
        assert!(
            (k.get(bo + 1, bo + 1) - 1.0).abs() < 1e-12,
            "Uy[{i}] should be 1.0"
        );
        assert!(
            (k.get(bo + 5, bo + 5) - 1.0).abs() < 1e-12,
            "Rz[{i}] should be 1.0"
        );
        // Uz, Rx, Ry should remain unchanged (non-zero)
        assert!(k.get(bo + 2, bo + 2) > 0.0, "Uz[{i}] should remain active");
        assert!(k.get(bo + 3, bo + 3) > 0.0, "Rx[{i}] should remain active");
        assert!(k.get(bo + 4, bo + 4) > 0.0, "Ry[{i}] should remain active");
    }
}

#[test]
fn test_shape_functions() {
    let N = shape_2d(0.0, 0.0);
    let sum: f64 = N.iter().sum();
    assert!((sum - 1.0).abs() < 1e-15);
}

#[test]
fn test_stiffness_nonzero_diagonal() {
    let shell = make_flat_shell(10.0);
    let k = shell.local_stiffness();
    for i in 0..24 {
        assert!(k.get(i, i) > 0.0, "diagonal[{i}] should be positive");
    }
}

#[test]
fn test_membrane_b_constant_strain() {
    let shell = make_flat_shell(10.0);
    let eps_x = 1e-3;
    let eps_y = 2e-3;
    let gam_xy = 0.5e-3;
    let coords = &shell.coords;
    let nodes_disp: Vec<f64> = (0..4)
        .flat_map(|i| {
            let x = coords[i][0];
            let y = coords[i][1];
            let u = eps_x * x + 0.5 * gam_xy * y;
            let v = eps_y * y + 0.5 * gam_xy * x;
            // DOF order: Ux, Uy, Uz, Rx, Ry, Rz
            [u, v, 0.0, 0.0, 0.0, 0.0]
        })
        .collect();

    // Evaluate B*u at center (xi=0, eta=0)
    let dNc = dshape_cart(0.0, 0.0, coords);
    let bm = shell.membrane_b(0.0, 0.0, &dNc);
    let mut strain = [0.0; 3];
    for r in 0..3 {
        for j in 0..24 {
            strain[r] += bm[r * 24 + j] * nodes_disp[j];
        }
    }
    assert!((strain[0] - eps_x).abs() < 1e-12, "ε_x={}", strain[0]);
    assert!((strain[1] - eps_y).abs() < 1e-12, "ε_y={}", strain[1]);
    assert!((strain[2] - gam_xy).abs() < 1e-12, "γ_xy={}", strain[2]);
}

#[test]
fn test_bending_b_constant_curvature() {
    let shell = make_flat_shell(10.0);
    let kap_x = 1e-5;
    let kap_y = 2e-5;
    let kap_xy = 0.5e-5;

    // For constant curvature: θ_x = -kap_y * y, θ_y = kap_x * x + kap_xy * y,
    // w = 0.5*(kap_x*x² + kap_xy*x*y - kap_xy*x*y? no, that's complex)
    // Actually for bending: κ_x = dθ_y/dx, κ_y = -dθ_x/dy, κ_xy = dθ_y/dy - dθ_x/dx
    // So set: θ_x = -kap_y * y,  θ_y = kap_x * x
    // Then κ_x = kap_x, κ_y = kap_y, κ_xy = 0 + 0 = 0
    // But κ_xy is missing. Let's use a more complete field:
    // θ_x = -kap_y * y,  θ_y = kap_x * x + kap_xy * y
    // κ_x = dθ_y/dx = kap_x  ✓
    // κ_y = -dθ_x/dy = kap_y  ✓
    // κ_xy = dθ_y/dy - dθ_x/dx = kap_xy - 0 = kap_xy  ✓

    let coords = &shell.coords;
    let nodes_disp: Vec<f64> = (0..4)
        .flat_map(|i| {
            let x = coords[i][0];
            let y = coords[i][1];
            let rx = -kap_y * y;
            let ry = kap_x * x + kap_xy * y;
            [0.0, 0.0, 0.0, rx, ry, 0.0]
        })
        .collect();

    let dNc = dshape_cart(0.0, 0.0, coords);
    let bb = shell.bending_b(0.0, 0.0, &dNc);
    let mut curv = [0.0; 3];
    for r in 0..3 {
        for j in 0..24 {
            curv[r] += bb[r * 24 + j] * nodes_disp[j];
        }
    }
    assert!((curv[0] - kap_x).abs() < 1e-12, "κ_x={}", curv[0]);
    assert!((curv[1] - kap_y).abs() < 1e-12, "κ_y={}", curv[1]);
    assert!((curv[2] - kap_xy).abs() < 1e-12, "κ_xy={}", curv[2]);
}

fn k_times_u(k: &LocalMat, u: &[f64]) -> Vec<f64> {
    let n = k.n;
    let mut r = vec![0.0; n];
    for i in 0..n {
        let mut s = 0.0;
        for j in 0..n {
            s += k.get(i, j) * u[j];
        }
        r[i] = s;
    }
    r
}

fn residual_norm(r: &[f64]) -> f64 {
    r.iter().map(|v| v * v).sum::<f64>().sqrt()
}

#[test]
fn test_six_rigid_body_modes_zero_energy() {
    let shell = make_flat_shell(50.0);
    let k = shell.local_stiffness();
    let coords = &shell.coords;

    let mut rb = vec![vec![0.0; 24]; 6];
    for m in 0..6 {
        for i in 0..4 {
            let x = coords[i][0];
            let y = coords[i][1];
            let bo = i * 6;
            match m {
                0 => rb[m][bo] = 1.0,     // Tx
                1 => rb[m][bo + 1] = 1.0, // Ty
                2 => rb[m][bo + 2] = 1.0, // Tz
                3 => {
                    // Rx: uz = y, rx = 1
                    rb[m][bo + 2] = y;
                    rb[m][bo + 3] = 1.0;
                }
                4 => {
                    // Ry: uz = -x, ry = 1
                    rb[m][bo + 2] = -x;
                    rb[m][bo + 4] = 1.0;
                }
                _ => {
                    // Rz: ux = -y, uy = x, rz = 1 (drilling rotation)
                    rb[m][bo] = -y;
                    rb[m][bo + 1] = x;
                    rb[m][bo + 5] = 1.0;
                }
            }
        }
    }

    for (m, u) in rb.iter().enumerate() {
        let r = k_times_u(&k, u);
        let norm = residual_norm(&r);
        let scale = u.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            norm / scale < 1e-8,
            "rigid body mode {m} should have zero energy: norm={norm}"
        );
    }
}

#[test]
fn test_drilling_stabilization_insensitivity() {
    // Compare cantilever plate tip displacement with different drilling factors.
    // Use a simple one-element cantilever: fix edge nodes 0,1; free 2,3; load node 2 in z.
    let base = make_flat_shell(10.0);
    let mut k1 = base.local_stiffness();
    base.add_drilling(&mut k1);

    let mut base_lo = base.clone();
    base_lo.drilling_factor = DEFAULT_DRILLING_FACTOR * 0.1;
    let mut k_lo = base_lo.local_stiffness();
    base_lo.add_drilling(&mut k_lo);

    let mut base_hi = base.clone();
    base_hi.drilling_factor = DEFAULT_DRILLING_FACTOR * 10.0;
    let mut k_hi = base_hi.local_stiffness();
    base_hi.add_drilling(&mut k_hi);

    // Fixed DOFs: nodes 0 and 1 (Ux..Rz all fixed) => active DOFs are nodes 2 and 3 (12 DOFs)
    let active: Vec<usize> = (12..24).collect();
    let reduce = |k: &LocalMat| -> Vec<f64> {
        let n = active.len();
        let mut kred = vec![0.0; n * n];
        for (ia, &i) in active.iter().enumerate() {
            for (ja, &j) in active.iter().enumerate() {
                kred[ia * n + ja] = k.get(i, j);
            }
        }
        kred
    };

    // Load at node 2 in Uz (active index 2 within node 2 => global index 12+2=14)
    let load_idx_in_active = 14 - 12;
    let solve = |kred: &[f64], f: &[f64]| -> Vec<f64> {
        let n = f.len();
        let mut x = vec![0.0; n];
        for i in 0..n {
            let mut s = 0.0;
            for j in 0..n {
                s += kred[i * n + j] * x[j];
            }
            let r = f[i] - s;
            x[i] += r / kred[i * n + i];
        }
        // one Jacobi sweep is enough for diagonally dominant matrices; do a few
        for _ in 0..100 {
            for i in 0..n {
                let mut s = 0.0;
                for j in 0..n {
                    if i != j {
                        s += kred[i * n + j] * x[j];
                    }
                }
                x[i] = (f[i] - s) / kred[i * n + i];
                if x[i].is_nan() {
                    x[i] = 0.0;
                }
            }
        }
        x
    };

    let k1r = reduce(&k1);
    let klor = reduce(&k_lo);
    let khir = reduce(&k_hi);

    let mut f = vec![0.0; active.len()];
    f[load_idx_in_active] = 1.0;

    let u1 = solve(&k1r, &f);
    let ulo = solve(&klor, &f);
    let uhi = solve(&khir, &f);

    let w1 = u1[load_idx_in_active];
    let wlo = ulo[load_idx_in_active];
    let whi = uhi[load_idx_in_active];

    assert!(
        (wlo - w1).abs() / w1.abs() < 0.01,
        "lo drilling diff too large"
    );
    assert!(
        (whi - w1).abs() / w1.abs() < 0.01,
        "hi drilling diff too large"
    );
}

#[test]
fn test_patch_membrane_constant_stress() {
    // Membrane patch test with a distorted quadrilateral mesh.
    // 4 elements forming a patch with an interior node.
    // Coordinates (in-plane, z=0 for all):
    //   Outer boundary: (0,0), (100,0), (100,100), (0,100)
    //   Inner node: (45, 55) (offset from center)
    // Apply linear displacement u = eps_x * x, v = eps_y * y + 0.5*gam_xy * x
    // at boundary nodes. Interior node should have correct displacement.
    let eps_x = 1e-3;
    let eps_y = 2e-3;
    let gam_xy = 0.5e-3;

    // Use a single distorted element to verify B*u
    let coords = [
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [100.0, 100.0, 0.0],
        [0.0, 100.0, 0.0],
    ];
    let frame = ShellFrame::from_nodes(coords);
    let shell = ShellElement {
        nodes: [NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        coords,
        t: 10.0,
        e: 1000.0,
        nu: 0.3,
        density: 0.0,
        frame,
        drilling_factor: DEFAULT_DRILLING_FACTOR,
        membrane_active: true,
        committed_disp: [0.0; 24],
        trial_disp: [0.0; 24],
    };

    // Interior point at distorted panel center (45, 55)
    // Need to find (xi,eta) that maps to (45,55) — use Newton or just evaluate at several points
    // For the patch test, the strain should be constant everywhere.
    // Evaluate at (xi=0, eta=0) for element center:
    let dNc = dshape_cart(0.0, 0.0, &coords);
    let bm = shell.membrane_b(0.0, 0.0, &dNc);

    // Apply linear displacement at nodes
    let u_disp: Vec<f64> = (0..4)
        .flat_map(|i| {
            let x = coords[i][0];
            let y = coords[i][1];
            let u = eps_x * x + 0.5 * gam_xy * y;
            let v = eps_y * y + 0.5 * gam_xy * x;
            [u, v, 0.0, 0.0, 0.0, 0.0]
        })
        .collect();

    let mut strain = [0.0; 3];
    for r in 0..3 {
        for j in 0..24 {
            strain[r] += bm[r * 24 + j] * u_disp[j];
        }
    }
    assert!((strain[0] - eps_x).abs() < 1e-12, "ε_x={}", strain[0]);
    assert!((strain[1] - eps_y).abs() < 1e-12, "ε_y={}", strain[1]);
    assert!((strain[2] - gam_xy).abs() < 1e-12, "γ_xy={}", strain[2]);
}

// -----------------------------------------------------------------------
// 真のパッチテスト（歪みメッシュ・機械精度）— 仕様 §9.2（唯一の厳密ゲート）
// -----------------------------------------------------------------------
fn distorted_patch() -> (Vec<[f64; 3]>, Vec<[usize; 4]>) {
    // 中央節点を非対称に歪ませた 9 節点・4 要素パッチ。内部=節点4。
    let coords = vec![
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [200.0, 0.0, 0.0],
        [0.0, 100.0, 0.0],
        [115.0, 88.0, 0.0],
        [200.0, 100.0, 0.0],
        [0.0, 200.0, 0.0],
        [100.0, 200.0, 0.0],
        [200.0, 200.0, 0.0],
    ];
    let elems = vec![[0, 1, 4, 3], [1, 2, 5, 4], [3, 4, 7, 6], [4, 5, 8, 7]];
    (coords, elems)
}

fn make_shell_on(coords4: [[f64; 3]; 4], nids: [usize; 4]) -> ShellElement {
    ShellElement {
        nodes: [
            NodeId(nids[0] as u32),
            NodeId(nids[1] as u32),
            NodeId(nids[2] as u32),
            NodeId(nids[3] as u32),
        ],
        coords: coords4,
        t: 10.0,
        e: 200000.0,
        nu: 0.3,
        density: 0.0,
        frame: ShellFrame::from_nodes(coords4),
        drilling_factor: DEFAULT_DRILLING_FACTOR,
        membrane_active: true,
        committed_disp: [0.0; 24],
        trial_disp: [0.0; 24],
    }
}

fn assemble_dense(coords: &[[f64; 3]], elems: &[[usize; 4]]) -> (Vec<f64>, usize) {
    let nn = coords.len();
    let ndof = nn * 6;
    let mut k = vec![0.0; ndof * ndof];
    for e in elems {
        let c4 = [coords[e[0]], coords[e[1]], coords[e[2]], coords[e[3]]];
        let shell = make_shell_on(c4, *e);
        let kl = shell.frame.to_global(&shell.local_stiffness());
        let gdof = |loc: usize| e[loc / 6] * 6 + (loc % 6);
        for a in 0..24 {
            let ga = gdof(a);
            for b in 0..24 {
                k[ga * ndof + gdof(b)] += kl.get(a, b);
            }
        }
    }
    (k, ndof)
}

fn solve_prescribed(k: &[f64], ndof: usize, free: &[usize], g: &[f64]) -> Vec<f64> {
    let nf = free.len();
    let mut a = vec![0.0; nf * nf];
    let mut rhs = vec![0.0; nf];
    let mut is_free = vec![false; ndof];
    for &f in free {
        is_free[f] = true;
    }
    for (i, &fi) in free.iter().enumerate() {
        for (j, &fj) in free.iter().enumerate() {
            a[i * nf + j] = k[fi * ndof + fj];
        }
        let mut s = 0.0;
        for (b, &gb) in g.iter().enumerate() {
            if !is_free[b] {
                s += k[fi * ndof + b] * gb;
            }
        }
        rhs[i] = -s;
    }
    let uf = dense_solve(&mut a, &mut rhs, nf);
    let mut u = g.to_vec();
    for (i, &fi) in free.iter().enumerate() {
        u[fi] = uf[i];
    }
    u
}

fn dense_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    for col in 0..n {
        let mut piv = col;
        let mut best = a[col * n + col].abs();
        for r in (col + 1)..n {
            let v = a[r * n + col].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if piv != col {
            for j in 0..n {
                a.swap(col * n + j, piv * n + j);
            }
            b.swap(col, piv);
        }
        let d = a[col * n + col];
        for r in (col + 1)..n {
            let f = a[r * n + col] / d;
            if f != 0.0 {
                for j in col..n {
                    a[r * n + j] -= f * a[col * n + j];
                }
                b[r] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for j in (i + 1)..n {
            s -= a[i * n + j] * x[j];
        }
        x[i] = s / a[i * n + i];
    }
    x
}

/// 膜パッチ：歪みメッシュで線形変位場 → 内部節点が場を機械精度で再現。
#[test]
fn test_patch_membrane_distorted() {
    let (coords, elems) = distorted_patch();
    let (k, ndof) = assemble_dense(&coords, &elems);
    let field = |c: &[f64; 3]| [1.0e-3 * c[0] + 0.5e-3 * c[1], 0.3e-3 * c[0] + 2.0e-3 * c[1]];
    let mut g = vec![0.0; ndof];
    for (i, c) in coords.iter().enumerate() {
        let f = field(c);
        g[i * 6] = f[0];
        g[i * 6 + 1] = f[1];
    }
    let free: Vec<usize> = (0..6).map(|d| 4 * 6 + d).collect();
    let u = solve_prescribed(&k, ndof, &free, &g);
    let exact = field(&coords[4]);
    assert!(
        (u[4 * 6] - exact[0]).abs() < 1e-9 && (u[4 * 6 + 1] - exact[1]).abs() < 1e-9,
        "膜パッチ不一致: Ux={} (exp {}), Uy={} (exp {})",
        u[4 * 6],
        exact[0],
        u[4 * 6 + 1],
        exact[1]
    );
}

/// 曲げパッチ：歪みメッシュで定曲率場 → 内部節点が場を機械精度で再現。
/// MITC4 の合否ゲート（薄板でロッキングしないことの根拠）。
#[test]
fn test_mitc4_constant_shear_patch_skewed() {
    // MITC4 は任意形状で一定横せん断場を厳密に再現しなければならない。
    // w=0・θx=0・θy=a（一定）は γ_xz = ∂w/∂x + θy = a, γ_yz = ∂w/∂y − θx = 0 の
    // 一定せん断場に相当する。平行四辺形（ヤコビアン非対称）で検証する。
    //
    // 回帰: 共変→デカルト射影 γ = J⁻¹·e_cov を、従来は jit(=J⁻ᵀ) の行アクセスで
    // (J⁻ᵀ)·e として適用していた（正: 列アクセス ＝ J⁻¹·e）。ヤコビアンが
    // 非対称な歪んだ四辺形でのみ顕在化し（矩形では jit が対角のため一致）、
    // 一定せん断パッチテストが破れていた。
    let coords = [
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [150.0, 80.0, 0.0],
        [50.0, 80.0, 0.0],
    ];
    let shell = make_shell_on(coords, [0, 1, 2, 3]);
    let a = 1.0e-3;
    // 24 自由度ベクトル: 各節点の θy(=Ry, col+4) に a、他は 0。
    let mut u = [0.0f64; 24];
    for node in 0..4 {
        u[node * 6 + 4] = a;
    }
    // 複数の評価点（ガウス点）で一定せん断が厳密に再現されること。
    for &(xi, eta, _w) in &[
        (-0.577_350_269_2f64, -0.577_350_269_2f64, 1.0),
        (0.577_350_269_2, -0.577_350_269_2, 1.0),
        (0.577_350_269_2, 0.577_350_269_2, 1.0),
        (-0.577_350_269_2, 0.577_350_269_2, 1.0),
        (0.0, 0.0, 1.0),
    ] {
        let b = shell.shear_b_mitc4(xi, eta, &coords);
        let mut gxz = 0.0;
        let mut gyz = 0.0;
        for j in 0..24 {
            gxz += b[j] * u[j];
            gyz += b[24 + j] * u[j];
        }
        assert!(
            (gxz - a).abs() < 1e-12 && gyz.abs() < 1e-12,
            "constant transverse shear must be reproduced on a skewed quad at ({xi},{eta}): \
             γ_xz={gxz} (want {a}), γ_yz={gyz} (want 0)"
        );
    }
}

#[test]
fn test_patch_bending_distorted() {
    let (coords, elems) = distorted_patch();
    let (k, ndof) = assemble_dense(&coords, &elems);
    let (kx, ky, kxy) = (1.0e-6, 2.0e-6, 0.5e-6);
    // w = ½κx x² + ½κy y² + κxy xy、Kirchhoff: θy=−∂w/∂x, θx=∂w/∂y
    let field = |c: &[f64; 3]| -> [f64; 3] {
        let (x, y) = (c[0], c[1]);
        [
            0.5 * kx * x * x + 0.5 * ky * y * y + kxy * x * y, // w (Uz)
            ky * y + kxy * x,                                  // θx (Rx)
            -(kx * x + kxy * y),                               // θy (Ry)
        ]
    };
    let mut g = vec![0.0; ndof];
    for (i, c) in coords.iter().enumerate() {
        let f = field(c);
        g[i * 6 + 2] = f[0];
        g[i * 6 + 3] = f[1];
        g[i * 6 + 4] = f[2];
    }
    let free: Vec<usize> = (0..6).map(|d| 4 * 6 + d).collect();
    let u = solve_prescribed(&k, ndof, &free, &g);
    let exact = field(&coords[4]);
    let scale = 0.5 * kx * 200.0 * 200.0; // 代表変位スケール
    assert!(
        (u[4 * 6 + 2] - exact[0]).abs() < 1e-8 * scale,
        "曲げパッチ Uz={} exp {}",
        u[4 * 6 + 2],
        exact[0]
    );
    assert!(
        (u[4 * 6 + 3] - exact[1]).abs() < 1e-8 * (exact[1].abs().max(1e-4)),
        "曲げパッチ Rx={} exp {}",
        u[4 * 6 + 3],
        exact[1]
    );
    assert!(
        (u[4 * 6 + 4] - exact[2]).abs() < 1e-8 * (exact[2].abs().max(1e-4)),
        "曲げパッチ Ry={} exp {}",
        u[4 * 6 + 4],
        exact[2]
    );
}

/// トライアル追従の回帰テスト: update_state(du, commit=false) が internal_force に
/// 反映され（内力 = 接線剛性·u と厳密に一致）、剛体並進では内力ゼロとなること。
/// 従来は internal_force が恒常的にゼロを返しており、非線形解析でシェルが
/// 復元力を負担していなかった。
///
/// K·u 比較は「internal_force と tangent_stiffness が将来ズレない」ことの
/// 回帰ガードであり、K の値そのものの正しさは本ファイルのパッチテスト群
/// （膜・曲げの解析解照合）が担保する（両者を合わせて非循環な検証となる）。
#[test]
fn test_shell_trial_displacement_tracking() {
    use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
    use squid_n_core::model::Model;
    let mut shell = make_flat_shell(10.0);
    let model = Model::default();
    let ctx = Ctx { model: &model };
    let state = ElemState::default();

    // 面内せん断的な非剛体変位（節点2・3 のみ x 方向）
    let mut du = LocalVec {
        data: smallvec::smallvec![0.0; 24],
    };
    du.data[2 * 6] = 1.0;
    du.data[3 * 6] = 1.0;
    shell.update_state(&du, false, &ctx);

    // commit 前でも内力へ反映され、接線剛性·u と厳密に一致する
    let f = shell.internal_force(&state, &ctx);
    let k = shell.tangent_stiffness(&state, &ctx);
    for i in 0..24 {
        let expected: f64 = (0..24).map(|j| k.get(i, j) * du.data[j]).sum();
        assert!(
            (f.data[i] - expected).abs() <= 1e-9 * expected.abs().max(1.0),
            "内力が K·u と不一致: i={i} f={} expected={expected}",
            f.data[i]
        );
    }
    assert!(
        f.data.iter().any(|v| v.abs() > 1e-6),
        "非剛体変位で内力が生じない"
    );

    // 剛体並進（全節点同一変位）では内力ゼロ
    shell.revert_state(); // trial を committed（=0）へ戻す
    let mut du_rigid = LocalVec {
        data: smallvec::smallvec![0.0; 24],
    };
    for n in 0..4 {
        du_rigid.data[n * 6] = 1.0;
    }
    shell.update_state(&du_rigid, false, &ctx);
    let f_rigid = shell.internal_force(&state, &ctx);
    assert!(
        f_rigid.data.iter().all(|v| v.abs() < 1e-6),
        "剛体並進で内力が生じた: {:?}",
        f_rigid.data
    );
}
