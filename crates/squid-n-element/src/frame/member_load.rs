//! 部材（梁）スパン荷重の等価節点力（consistent load vector）と、
//! 両端固定梁としての固定端内力（重ね合わせ用）を計算する。
//!
//! # 規約
//! - ローカル 12 自由度の並びは beam.rs と同一:
//!   i 端 [N, Vy, Vz, Mx, My, Mz] = index 0..6、j 端 = index 6..12。
//! - `MemberLoad::dir` は全体座標の作用方向。ローカル軸 (ex, ey, ez) へ分解して
//!   軸方向 (x) と 2 つの曲げ面 (y, z) の成分に分けて扱う。
//! - 等価節点力 `Q`（local）は構造系の荷重ベクトルへ `R^T·Q` で加算する。
//! - 内力回復では、`K·u` 由来の内力に本モジュールの「固定端内力」を重ね合わせる。

use crate::transform::LocalFrame;
use squid_n_core::model::{MemberLoad, MemberLoadKind};

/// ローカル 1 軸へ分解した成分荷重。`mag` は成分係数 (dir·e_axis) を乗じ済み。
#[derive(Clone, Copy, Debug)]
enum Comp {
    /// i 端から距離 a に集中荷重 p（成分係数込み）。
    Point { a: f64, p: f64 },
    /// [a,b] 区間に強度 w1→w2 の線形分布（成分係数込み）。
    Dist { a: f64, b: f64, w1: f64, w2: f64 },
}

/// 1 つの部材荷重をローカル 3 軸 (x,y,z) の成分へ分解する。
/// 返り値 [Option<Comp>;3]: index 0=軸(x), 1=曲げ面y, 2=曲げ面z。
fn resolve(load: &MemberLoad, frame: &LocalFrame) -> [Option<Comp>; 3] {
    // dir を正規化
    let d = load.dir;
    let dl = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if dl < 1e-12 {
        return [None, None, None];
    }
    let d = [d[0] / dl, d[1] / dl, d[2] / dl];
    // ローカル成分係数 c_axis = d · e_axis（rot 行が ex,ey,ez）
    let c = [
        d[0] * frame.rot[0][0] + d[1] * frame.rot[0][1] + d[2] * frame.rot[0][2],
        d[0] * frame.rot[1][0] + d[1] * frame.rot[1][1] + d[2] * frame.rot[1][2],
        d[0] * frame.rot[2][0] + d[1] * frame.rot[2][1] + d[2] * frame.rot[2][2],
    ];
    let mut out = [None, None, None];
    for (axis, out_slot) in out.iter_mut().enumerate() {
        let ck = c[axis];
        if ck.abs() < 1e-15 {
            continue;
        }
        *out_slot = Some(match load.kind {
            MemberLoadKind::Point { a, p } => Comp::Point { a, p: p * ck },
            MemberLoadKind::Distributed { a, b, w1, w2 } => Comp::Dist {
                a,
                b,
                w1: w1 * ck,
                w2: w2 * ck,
            },
        });
    }
    out
}

// --- Hermite 形状関数（ξ = s/L）。曲げ面の等価節点力に用いる ---
fn n_vi(xi: f64) -> f64 {
    1.0 - 3.0 * xi * xi + 2.0 * xi * xi * xi
}
fn n_ti(xi: f64, l: f64) -> f64 {
    l * (xi - 2.0 * xi * xi + xi * xi * xi)
}
fn n_vj(xi: f64) -> f64 {
    3.0 * xi * xi - 2.0 * xi * xi * xi
}
fn n_tj(xi: f64, l: f64) -> f64 {
    l * (-xi * xi + xi * xi * xi)
}

/// [a,b] 区間の強度 w1→w2 線形分布に対し、被積分関数 f(s) を 3 点 Gauss で積分。
/// w が線形・f が高々 3 次までなら正確（4 次まで可）。
fn gauss_dist<F: Fn(f64) -> f64>(a: f64, b: f64, w1: f64, w2: f64, f: F) -> f64 {
    if (b - a).abs() < 1e-12 {
        return 0.0;
    }
    // 3 点 Gauss-Legendre [-1,1]
    const G: [f64; 3] = [-0.7745966692414834, 0.0, 0.7745966692414834];
    const W: [f64; 3] = [0.5555555555555556, 0.8888888888888888, 0.5555555555555556];
    let mid = 0.5 * (a + b);
    let half = 0.5 * (b - a);
    let mut s = 0.0;
    for k in 0..3 {
        let x = mid + half * G[k];
        let t = (x - a) / (b - a);
        let w = w1 + (w2 - w1) * t; // 強度
        s += W[k] * w * f(x);
    }
    s * half
}

/// 部材の全スパン荷重に対する等価節点力ベクトル（local 12）。
/// 構造系へは `frame.rotate_to_global(&q)` を加算する。
pub fn consistent_load_local(loads: &[MemberLoad], frame: &LocalFrame, length: f64) -> [f64; 12] {
    let l = length.max(1e-9);
    let mut q = [0.0; 12];
    for load in loads {
        let comps = resolve(load, frame);
        for (axis, comp) in comps.iter().enumerate() {
            let Some(comp) = comp else { continue };
            match axis {
                0 => add_axial_consistent(&mut q, comp, l),
                1 => add_bending_consistent(&mut q, comp, l, 1),
                _ => add_bending_consistent(&mut q, comp, l, 2),
            }
        }
    }
    q
}

/// 軸方向（線形形状関数 1-ξ, ξ）。q[0]=N_i, q[6]=N_j。
fn add_axial_consistent(q: &mut [f64; 12], comp: &Comp, l: f64) {
    match *comp {
        Comp::Point { a, p } => {
            let xi = (a / l).clamp(0.0, 1.0);
            q[0] += p * (1.0 - xi);
            q[6] += p * xi;
        }
        Comp::Dist { a, b, w1, w2 } => {
            q[0] += gauss_dist(a, b, w1, w2, |s| 1.0 - s / l);
            q[6] += gauss_dist(a, b, w1, w2, |s| s / l);
        }
    }
}

/// 曲げ面（plane=1: y面 → Vy,Mz / plane=2: z面 → Vz,My）。
/// z 面はモーメント自由度の符号が反転する（右手系 θy と θz の差）。
fn add_bending_consistent(q: &mut [f64; 12], comp: &Comp, l: f64, plane: usize) {
    let (iv, im, jv, jm, msign) = if plane == 1 {
        (1usize, 5usize, 7usize, 11usize, 1.0)
    } else {
        (2usize, 4usize, 8usize, 10usize, -1.0)
    };
    match *comp {
        Comp::Point { a, p } => {
            let xi = (a / l).clamp(0.0, 1.0);
            q[iv] += p * n_vi(xi);
            q[im] += msign * p * n_ti(xi, l);
            q[jv] += p * n_vj(xi);
            q[jm] += msign * p * n_tj(xi, l);
        }
        Comp::Dist { a, b, w1, w2 } => {
            q[iv] += gauss_dist(a, b, w1, w2, |s| n_vi(s / l));
            q[im] += msign * gauss_dist(a, b, w1, w2, |s| n_ti(s / l, l));
            q[jv] += gauss_dist(a, b, w1, w2, |s| n_vj(s / l));
            q[jm] += msign * gauss_dist(a, b, w1, w2, |s| n_tj(s / l, l));
        }
    }
}

/// 区間 [lo,hi] における合力 ∫ w ds（成分荷重 1 つ分）。
fn comp_resultant(comp: &Comp, lo: f64, hi: f64) -> f64 {
    if hi <= lo {
        return 0.0;
    }
    match *comp {
        Comp::Point { a, p } => {
            if a >= lo && a < hi {
                p
            } else {
                0.0
            }
        }
        Comp::Dist { a, b, w1, w2 } => {
            let l = lo.max(a);
            let h = hi.min(b);
            if h <= l {
                return 0.0;
            }
            // [l,h] 区間の強度を 2 点 Gauss で積分（w 線形 → 正確）
            integ2(a, b, w1, w2, l, h, |_s| 1.0)
        }
    }
}

/// 区間 [lo,hi] における断面 xref まわりの 1 次モーメント ∫ w(s)(xref−s) ds。
fn comp_moment(comp: &Comp, lo: f64, hi: f64, xref: f64) -> f64 {
    if hi <= lo {
        return 0.0;
    }
    match *comp {
        Comp::Point { a, p } => {
            if a >= lo && a < hi {
                p * (xref - a)
            } else {
                0.0
            }
        }
        Comp::Dist { a, b, w1, w2 } => {
            let l = lo.max(a);
            let h = hi.min(b);
            if h <= l {
                return 0.0;
            }
            integ2(a, b, w1, w2, l, h, |s| xref - s)
        }
    }
}

/// 強度 w1→w2（[a,b] 上線形）の分布に対し被積分 w(s)·f(s) を区間 [l,h] で 2 点 Gauss 積分。
fn integ2<F: Fn(f64) -> f64>(a: f64, b: f64, w1: f64, w2: f64, l: f64, h: f64, f: F) -> f64 {
    const G: [f64; 2] = [-0.5773502691896257, 0.5773502691896257];
    let mid = 0.5 * (l + h);
    let half = 0.5 * (h - l);
    let denom = b - a;
    let mut s_sum = 0.0;
    for &g in &G {
        let s = mid + half * g;
        let t = if denom.abs() < 1e-12 {
            0.0
        } else {
            (s - a) / denom
        };
        let w = w1 + (w2 - w1) * t;
        s_sum += w * f(s);
    }
    s_sum * half
}

/// 断面 xi（i 端からの正規化位置 0..1）における「両端固定梁としての固定端内力」を
/// local 内力 [N, Qy, Qz, Mx, My, Mz] で返す。`K·u` 由来の内力（part A）へ重ね合わせる。
///
/// beam.rs の `recover_forces` の符号規約・i/j 分岐を厳密にミラーする。
/// `fFEF = -Q`（固定端力 = 等価節点力の符号反転）を端部力として用い、
/// 分布荷重のスパン自由体項を beam の式形に合わせて加える。
pub fn fixed_internal_local(
    loads: &[MemberLoad],
    frame: &LocalFrame,
    length: f64,
    xi: f64,
) -> [f64; 6] {
    let l = length.max(1e-9);
    let x = xi * l;
    let xr = (1.0 - xi) * l;
    let q = consistent_load_local(loads, frame, l);
    let ff = q.map(|v| -v); // fFEF = -Q

    // 各ローカル軸（y,z,x）の成分荷重を集約
    let mut comps_x: Vec<Comp> = Vec::new();
    let mut comps_y: Vec<Comp> = Vec::new();
    let mut comps_z: Vec<Comp> = Vec::new();
    for load in loads {
        let r = resolve(load, frame);
        if let Some(c) = r[0] {
            comps_x.push(c);
        }
        if let Some(c) = r[1] {
            comps_y.push(c);
        }
        if let Some(c) = r[2] {
            comps_z.push(c);
        }
    }
    // [0,x] の合力 / x まわりモーメント、[x,L] の合力 / x まわりモーメント
    let res_i = |comps: &[Comp]| comps.iter().map(|c| comp_resultant(c, 0.0, x)).sum::<f64>();
    let mom_i = |comps: &[Comp]| comps.iter().map(|c| comp_moment(c, 0.0, x, x)).sum::<f64>();
    let mom_jx = |comps: &[Comp]| comps.iter().map(|c| comp_moment(c, x, l, x)).sum::<f64>();

    let sy_i = res_i(&comps_y);
    let sz_i = res_i(&comps_z);

    let mut f = [0.0; 6];
    // 軸力（端部反力の線形内挿。分布軸荷重の中間値は近似）
    f[0] = ff[0] * (1.0 - xi) + ff[6] * xi;
    // せん断（i 側自由体の単一式: Q = fFEF_i + ∫₀ˣ w ds）
    f[1] = ff[1] + sy_i;
    f[2] = ff[2] + sz_i;

    if xi < 0.5 {
        // i 端基準（beam: mz=−f5+f1·x, my=−f4−f2·x）。節点モーメントは断面内力と
        // 符号が逆のため反転し、スパン荷重の自由体項も j 側と対称な符号で加える。
        f[3] = -ff[3];
        f[5] = -ff[5] + ff[1] * x + mom_i(&comps_y);
        f[4] = -ff[4] - ff[2] * x - mom_i(&comps_z);
    } else {
        // j 端基準（beam: mz=f11+f7·xr, my=f10−f8·xr）
        f[3] = ff[9];
        f[5] = ff[11] + ff[7] * xr - mom_jx(&comps_y);
        f[4] = ff[10] - ff[8] * xr + mom_jx(&comps_z);
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    // 水平梁（i→j が +X）、参照ベクトルで ey が +Z 上向きになるよう構成。
    fn horiz_frame() -> LocalFrame {
        // ex=+X。ref=+Z → ey=+Z, ez = ex×ey = +X×+Z = -Y
        LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    }

    fn udl(w: f64, l: f64) -> MemberLoad {
        // 下向き(-Z)等分布。ey=+Z なので成分 cy = dir·ey = -1。
        MemberLoad {
            elem: ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: w,
                w2: w,
            },
        }
    }

    #[test]
    fn udl_fixed_end_moment_is_wl2_over_12() {
        let l = 1000.0;
        let w = 2.0; // N/mm
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        let q = consistent_load_local(&loads, &frame, l);
        // y 面のせん断（i,j）= wL/2、符号は成分 cy=-1 を反映
        let expected_shear = w * l / 2.0;
        assert!((q[1].abs() - expected_shear).abs() < 1e-6, "q1={}", q[1]);
        assert!((q[7].abs() - expected_shear).abs() < 1e-6, "q7={}", q[7]);
        // 固定端モーメント = wL²/12
        let fem = w * l * l / 12.0;
        assert!((q[5].abs() - fem).abs() < 1e-3, "q5={} fem={}", q[5], fem);
        assert!((q[11].abs() - fem).abs() < 1e-3, "q11={}", q[11]);
    }

    #[test]
    fn udl_clamped_midspan_moment_is_wl2_over_24() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        let mid = fixed_internal_local(&loads, &frame, l, 0.5);
        let expected = w * l * l / 24.0;
        assert!(
            (mid[5].abs() - expected).abs() < 1e-2,
            "mid Mz={} expected={}",
            mid[5],
            expected
        );
    }

    #[test]
    fn point_mid_fixed_end_moment_is_pl_over_8() {
        let l = 1000.0;
        let p = 100.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad {
            elem: ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Point { a: l / 2.0, p },
        }];
        let q = consistent_load_local(&loads, &frame, l);
        // 中央集中の固定端モーメント = PL/8、せん断 = P/2
        let fem = p * l / 8.0;
        assert!((q[5].abs() - fem).abs() < 1e-6, "q5={} fem={}", q[5], fem);
        assert!((q[1].abs() - p / 2.0).abs() < 1e-6, "q1={}", q[1]);
    }

    /// 両端固定梁の UDL に対する固定端内力場が、i/j 分岐（ξ=0.5）をまたいで
    /// 連続かつ符号付き理論解と一致すること。下向き荷重（下端引張正の規約）で
    /// M(ξ) = wL²(6ξ−6ξ²−1)/12（端部 −wL²/12・中央 +wL²/24）、
    /// Q(ξ) = wL(1−2ξ)/2。
    /// （旧実装は i 端側分岐の符号が j 側と不整合で、ξ=0.5 の前後で値が
    /// ジャンプしていた。）
    #[test]
    fn udl_clamped_internal_field_continuous_and_signed() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        for &xi in &[0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0] {
            let f = fixed_internal_local(&loads, &frame, l, xi);
            let m_expected = w * l * l * (6.0 * xi - 6.0 * xi * xi - 1.0) / 12.0;
            let q_expected = w * l * (1.0 - 2.0 * xi) / 2.0;
            assert!(
                (f[5] - m_expected).abs() < 1e-2,
                "xi={xi} Mz={} expected={m_expected}",
                f[5]
            );
            assert!(
                (f[1] - q_expected).abs() < 1e-6,
                "xi={xi} Qy={} expected={q_expected}",
                f[1]
            );
        }
    }

    #[test]
    fn triangle_via_trapezoid_matches_known_fem() {
        // 対称三角形（端 0、中央ピーク）は台形では表せないが、
        // 片側三角形 [0,L] で w1=0→w2=w の固定端モーメントを検算。
        // FEM_i = wL²/30, FEM_j = wL²/20（i 端が荷重小側）。
        let l = 1000.0;
        let w = 3.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad {
            elem: ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: 0.0,
                w2: w,
            },
        }];
        let q = consistent_load_local(&loads, &frame, l);
        let fem_i = w * l * l / 30.0;
        let fem_j = w * l * l / 20.0;
        assert!(
            (q[5].abs() - fem_i).abs() < 1e-2,
            "q5={} fem_i={}",
            q[5],
            fem_i
        );
        assert!(
            (q[11].abs() - fem_j).abs() < 1e-2,
            "q11={} fem_j={}",
            q[11],
            fem_j
        );
    }
}
