//! 3次元 M-N 相関曲面（降伏曲面）の算定。
//!
//! 部材の降伏判定に用いる N–My–Mz 空間の相関曲面を、モデル化手法別に算定する:
//!
//! - **端部単純降伏バネ** (`SimpleSpring`): 軸バネと回転バネの2バネ連成を
//!   線形相関 |N|/N許容 + M/M許容 = 1 で考慮する。曲面は N 軸を頂点とする
//!   双錐（N-M 平面内では直線）になり、ファイバ積分による曲面の
//!   ふくらみ（特に RC の圧縮側での耐力上昇）は表現できない。
//! - **マルチスプリング** (`MultiSpring`): 断面を少数の軸バネ群で置換したモデル。
//!   N-M 相関は表現できるが、バネ本数が少ないため曲面は多面体状（ファセット状）になる。
//! - **マルチファイバー** (`MultiFiber`): 断面を多数のファイバに細分割したモデル。
//!   滑らかで精度の高い相関曲面が得られる。
//!
//! 算定は剛塑性（全塑性応力分布）の支持点法による。平面保持のひずみ速度方向
//! (ε̇0, κ̇y, κ̇z) を単位球面上で掃引し、各方向でひずみ符号に応じた限界応力
//! （鋼: ±fy、コンクリート: 圧縮 -Fc / 引張 0）を積分した断面力 (N, My, Mz) が
//! 曲面上の支持点となる。マルチスプリング/マルチファイバーはバネ・ファイバ配置の
//! 解像度だけが異なり、同一の積分で評価する。
//!
//! 単位: 長さ [mm], 応力 [N/mm²], 軸力 [N], モーメント [N·mm]。
//! 座標・符号規約はファイバ断面（`fiber.rs`）と同一: ε = ε0 − κz·y + κy·z。
//!
//! 注: 既存の `MsElement`（P5.5 §3）は断面内 y 軸上の1次元バネ配置で一軸曲げのみを
//! 対象とするが、本モジュールの `MultiSpring` は3次元相関を表現するため
//! 2次元配置（粗い格子）へ一般化している。

use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

/// 全塑性計算用のファイバ（またはバネ）。引張/圧縮の限界応力を保持する。
#[derive(Clone, Debug)]
pub struct PlasticFiber {
    /// 断面内 y 座標 [mm]（幅方向）
    pub y: f64,
    /// 断面内 z 座標 [mm]（せい方向）
    pub z: f64,
    /// 負担断面積 [mm²]
    pub area: f64,
    /// 引張限界応力 [N/mm²]（≥0。コンクリートは 0）
    pub sigma_t: f64,
    /// 圧縮限界応力 [N/mm²]（≤0）
    pub sigma_c: f64,
}

/// 降伏判定のモデル化手法。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum YieldModelKind {
    /// 部材端の単純降伏バネ（2バネ連成: |N|/N許容 + M/M許容 = 1 の線形相関）
    SimpleSpring,
    /// マルチスプリング（粗いバネ群、N-M 相関あり・多面体状）
    MultiSpring,
    /// マルチファイバー（細分割、滑らかな相関曲面）
    MultiFiber,
}

impl YieldModelKind {
    /// 表示用ラベル（日本語）。
    pub fn label(&self) -> &'static str {
        match self {
            YieldModelKind::SimpleSpring => "単純降伏バネ",
            YieldModelKind::MultiSpring => "マルチスプリング",
            YieldModelKind::MultiFiber => "マルチファイバー",
        }
    }
}

/// 材料強度パラメータ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrengthParams {
    /// 鋼材（形鋼）降伏強度 [N/mm²]
    pub steel_fy: f64,
    /// 鉄筋降伏強度 [N/mm²]
    pub rebar_fy: f64,
    /// コンクリート圧縮強度 [N/mm²]（正値で与える）
    pub concrete_fc: f64,
}

impl Default for StrengthParams {
    fn default() -> Self {
        StrengthParams {
            steel_fy: 235.0,
            rebar_fy: 345.0,
            concrete_fc: 24.0,
        }
    }
}

/// M-N 相関曲面。`grid[i][j]` は経線方向 i（引張極 α=0 → 圧縮極 α=π）、
/// 周方向 j（曲げ方向 β、閉曲線）でパラメータ化した曲面上の点 [N, My, Mz]。
pub struct MnSurface {
    pub kind: YieldModelKind,
    /// (n_alpha+1) × n_beta の格子点 [N, My, Mz]（周方向は j=0 と j=n_beta が接続）
    pub grid: Vec<Vec<[f64; 3]>>,
    /// 圧縮軸耐力 [N]（負値）
    pub n_comp: f64,
    /// 引張軸耐力 [N]（正値）
    pub n_tens: f64,
    /// N=0 での y 軸まわり全塑性モーメント [N·mm]
    pub mp_y: f64,
    /// N=0 での z 軸まわり全塑性モーメント [N·mm]
    pub mp_z: f64,
}

/// 全塑性応力分布による断面力（支持点）。
/// ひずみ速度方向 (e0, ky, kz) の符号のみで各ファイバの応力が決まる。
pub fn plastic_point(fibers: &[PlasticFiber], e0: f64, ky: f64, kz: f64) -> [f64; 3] {
    let mut n = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    for f in fibers {
        let eps = e0 - kz * f.y + ky * f.z;
        let sigma = if eps > 0.0 {
            f.sigma_t
        } else if eps < 0.0 {
            f.sigma_c
        } else {
            0.0
        };
        let sa = sigma * f.area;
        n += sa;
        my += sa * f.z;
        mz += -sa * f.y;
    }
    [n, my, mz]
}

/// 軸耐力 (圧縮 Nc ≤ 0, 引張 Nt ≥ 0)。
pub fn axial_capacity(fibers: &[PlasticFiber]) -> (f64, f64) {
    let nc: f64 = fibers.iter().map(|f| f.sigma_c * f.area).sum();
    let nt: f64 = fibers.iter().map(|f| f.sigma_t * f.area).sum();
    (nc, nt)
}

/// 曲げ方向 (ky, kz) を固定し、軸力が `n_target` となる全塑性中立軸位置での
/// モーメント (My, Mz) を返す。`n_target` が軸耐力範囲外なら `None`。
///
/// ファイバを中立軸からの距離順にソートし、圧縮側から引張側へ順次反転させて
/// 軸力を合わせる（中立軸上のファイバは部分的に応力を負担する）。
/// これは離散ファイバ集合の厳密な凸包（降伏曲面）上の点を与える。
pub fn plastic_moment_at_n(
    fibers: &[PlasticFiber],
    ky: f64,
    kz: f64,
    n_target: f64,
) -> Option<[f64; 2]> {
    let (nc, nt) = axial_capacity(fibers);
    if n_target < nc || n_target > nt || fibers.is_empty() {
        return None;
    }

    // 中立軸からのてこ距離 d = ky·z − kz·y。d が大きいファイバほど先に引張へ反転する。
    let mut order: Vec<usize> = (0..fibers.len()).collect();
    order.sort_by(|&a, &b| {
        let da = ky * fibers[a].z - kz * fibers[a].y;
        let db = ky * fibers[b].z - kz * fibers[b].y;
        db.partial_cmp(&da).unwrap_or(std::cmp::Ordering::Equal)
    });

    // 全断面圧縮から開始し、順に引張へ反転して n_target に到達させる。
    let mut n = nc;
    let mut my: f64 = fibers.iter().map(|f| f.sigma_c * f.area * f.z).sum();
    let mut mz: f64 = fibers.iter().map(|f| -f.sigma_c * f.area * f.y).sum();

    for &i in &order {
        let f = &fibers[i];
        let dn = (f.sigma_t - f.sigma_c) * f.area;
        if n + dn >= n_target {
            // このファイバが部分的に応力を負担して軸力が一致する
            let t = if dn > 0.0 { (n_target - n) / dn } else { 0.0 };
            let ds = t * (f.sigma_t - f.sigma_c) * f.area;
            my += ds * f.z;
            mz += -ds * f.y;
            return Some([my, mz]);
        }
        n += dn;
        my += (f.sigma_t - f.sigma_c) * f.area * f.z;
        mz += -(f.sigma_t - f.sigma_c) * f.area * f.y;
    }
    Some([my, mz])
}

/// 軸力一定 (`n_target`) での My–Mz 相関曲線を `n_pts` 点で返す（閉曲線、始点は繰り返さない）。
/// 軸耐力範囲外なら空。
pub fn slice_at_n(fibers: &[PlasticFiber], n_target: f64, n_pts: usize) -> Vec<[f64; 2]> {
    let mut pts = Vec::with_capacity(n_pts);
    for j in 0..n_pts {
        let beta = 2.0 * std::f64::consts::PI * j as f64 / n_pts as f64;
        let (ky, kz) = (beta.cos(), beta.sin());
        if let Some(m) = plastic_moment_at_n(fibers, ky, kz, n_target) {
            pts.push(m);
        }
    }
    pts
}

/// ファイバ群の特性半径（重心からの最大距離）。曲率方向の無次元化に用いる。
fn char_radius(fibers: &[PlasticFiber]) -> f64 {
    fibers
        .iter()
        .map(|f| (f.y * f.y + f.z * f.z).sqrt())
        .fold(0.0, f64::max)
        .max(1.0)
}

/// 支持点法で M-N 相関曲面を構築する（マルチスプリング/マルチファイバー用）。
///
/// - `n_alpha`: 経線方向（引張極→圧縮極）の分割数
/// - `n_beta`: 周方向（曲げ方向）の分割数
pub fn build_surface(
    fibers: &[PlasticFiber],
    kind: YieldModelKind,
    n_alpha: usize,
    n_beta: usize,
) -> MnSurface {
    let c = char_radius(fibers);
    let (nc, nt) = axial_capacity(fibers);
    let mp_y = plastic_moment_at_n(fibers, 1.0, 0.0, 0.0).map_or(0.0, |m| m[0]);
    let mp_z = plastic_moment_at_n(fibers, 0.0, 1.0, 0.0).map_or(0.0, |m| m[1]);

    let mut grid = Vec::with_capacity(n_alpha + 1);
    for i in 0..=n_alpha {
        let alpha = std::f64::consts::PI * i as f64 / n_alpha as f64;
        let e0 = alpha.cos();
        let k_mag = alpha.sin() / c;
        let mut row = Vec::with_capacity(n_beta);
        for j in 0..n_beta {
            let beta = 2.0 * std::f64::consts::PI * j as f64 / n_beta as f64;
            row.push(plastic_point(
                fibers,
                e0,
                k_mag * beta.cos(),
                k_mag * beta.sin(),
            ));
        }
        grid.push(row);
    }

    MnSurface {
        kind,
        grid,
        n_comp: nc,
        n_tens: nt,
        mp_y,
        mp_z,
    }
}

/// 単純降伏バネモデルの曲面（軸バネと回転バネの2バネ連成）を構築する。
///
/// バネの降伏値は細分割ファイバから算定した軸耐力 Nc/Nt と N=0 の全塑性モーメント
/// Mp_y/Mp_z とし、軸バネと回転バネの連成を線形相関
///
/// ```text
/// |N|/N許容 + M/M許容 = 1
/// ```
///
/// で考慮する（N許容は引張側 Nt / 圧縮側 |Nc| を使い分け、M/M許容は
/// 正規化モーメントの合成値 √((My/Mp_y)² + (Mz/Mp_z)²)）。
/// 曲面は N 軸を頂点とする双錐（N-M 平面内では直線）になる。
/// 描画の都合上、他モデルと同じ (α, β) 格子トポロジで返す。
pub fn build_simple_spring_surface(
    fibers_fine: &[PlasticFiber],
    n_alpha: usize,
    n_beta: usize,
) -> MnSurface {
    let (nc, nt) = axial_capacity(fibers_fine);
    let mp_y = plastic_moment_at_n(fibers_fine, 1.0, 0.0, 0.0).map_or(0.0, |m| m[0]);
    let mp_z = plastic_moment_at_n(fibers_fine, 0.0, 1.0, 0.0).map_or(0.0, |m| m[1]);

    // 耐力 0 の軸があると退化するため微小値で下駄を履かせる
    let n_tens_ref = nt.max(1.0);
    let n_comp_ref = nc.abs().max(1.0);
    let my_ref = mp_y.abs().max(1.0);
    let mz_ref = mp_z.abs().max(1.0);

    let mut grid = Vec::with_capacity(n_alpha + 1);
    for i in 0..=n_alpha {
        let alpha = std::f64::consts::PI * i as f64 / n_alpha as f64;
        // α ∈ [0, π/2) は引張側、(π/2, π] は圧縮側の N 許容値を使う
        let n_ref = if alpha.cos() >= 0.0 {
            n_tens_ref
        } else {
            n_comp_ref
        };
        // 正規化空間の方向 (cosα, sinα·cosβ, sinα·sinβ) に対し
        // |N|/N許容 + √((My/Mp_y)² + (Mz/Mp_z)²) = t·(|cosα| + sinα) = 1
        // となるよう倍率 t を定める（sinα ≥ 0）。
        let t = 1.0 / (alpha.cos().abs() + alpha.sin());
        let mut row = Vec::with_capacity(n_beta);
        for j in 0..n_beta {
            let beta = 2.0 * std::f64::consts::PI * j as f64 / n_beta as f64;
            row.push([
                t * alpha.cos() * n_ref,
                t * alpha.sin() * beta.cos() * my_ref,
                t * alpha.sin() * beta.sin() * mz_ref,
            ]);
        }
        grid.push(row);
    }

    MnSurface {
        kind: YieldModelKind::SimpleSpring,
        grid,
        n_comp: nc,
        n_tens: nt,
        mp_y,
        mp_z,
    }
}

// ---------------------------------------------------------------------------
// 断面形状 → ファイバ/バネ配置
// ---------------------------------------------------------------------------

/// 矩形領域（中心 `center = [cy, cz]`、幅 w × 高さ h）を目標寸法 `target` 以下の
/// ファイバに等分割して追加する。`limits = (引張限界応力, 圧縮限界応力)`。
fn mesh_rect(
    fibers: &mut Vec<PlasticFiber>,
    center: [f64; 2],
    w: f64,
    h: f64,
    target: f64,
    limits: (f64, f64),
) {
    let [cy, cz] = center;
    let (sigma_t, sigma_c) = limits;
    let ny = (w / target).ceil().max(1.0) as usize;
    let nz = (h / target).ceil().max(1.0) as usize;
    let dy = w / ny as f64;
    let dz = h / nz as f64;
    for i in 0..ny {
        for j in 0..nz {
            fibers.push(PlasticFiber {
                y: cy - w / 2.0 + (i as f64 + 0.5) * dy,
                z: cz - h / 2.0 + (j as f64 + 0.5) * dz,
                area: dy * dz,
                sigma_t,
                sigma_c,
            });
        }
    }
}

/// 円環領域（外径 do、厚 t）を周方向 `n_theta`・径方向 `n_r` に分割して追加する。
fn mesh_annulus(
    fibers: &mut Vec<PlasticFiber>,
    outer_dia: f64,
    thick: f64,
    n_theta: usize,
    n_r: usize,
    sigma_t: f64,
    sigma_c: f64,
) {
    let ro = outer_dia / 2.0;
    let ri = (ro - thick).max(0.0);
    let dr = (ro - ri) / n_r as f64;
    for ir in 0..n_r {
        let r_mid = ri + (ir as f64 + 0.5) * dr;
        let r_in = ri + ir as f64 * dr;
        let r_out = r_in + dr;
        let ring_area = std::f64::consts::PI * (r_out * r_out - r_in * r_in);
        let a = ring_area / n_theta as f64;
        for it in 0..n_theta {
            let th = 2.0 * std::f64::consts::PI * (it as f64 + 0.5) / n_theta as f64;
            fibers.push(PlasticFiber {
                y: r_mid * th.cos(),
                z: r_mid * th.sin(),
                area: a,
                sigma_t,
                sigma_c,
            });
        }
    }
}

/// 主筋1セット分のバネを追加する。
///
/// - `main_x`（せい方向主筋）: 上下面（z = ±(d/2 − cover)）に各 `count` 本を幅方向へ等配。
/// - `main_y`（幅方向主筋）: 側面（y = ±(b/2 − cover)）に各 `count` 本をせい方向の
///   内側区間へ等配（隅角部は main_x 側に含める）。
/// - `layers`: 2段目以降は 2.5×径 ずつ内側へ配置する。
fn rebar_fibers_rect(fibers: &mut Vec<PlasticFiber>, rebar: &RcRebar, b: f64, d: f64, fy: f64) {
    let bar = |set: &BarSet| -> f64 { std::f64::consts::PI * set.dia * set.dia / 4.0 };

    // せい方向主筋（上下面）
    let set = &rebar.main_x;
    if set.count > 0 {
        let a = bar(set);
        for layer in 0..set.layers.max(1) {
            let z0 = d / 2.0 - rebar.cover - layer as f64 * 2.5 * set.dia;
            let span = b - 2.0 * rebar.cover;
            for i in 0..set.count {
                let y = if set.count == 1 {
                    0.0
                } else {
                    -span / 2.0 + span * i as f64 / (set.count - 1) as f64
                };
                for zsign in [1.0, -1.0] {
                    fibers.push(PlasticFiber {
                        y,
                        z: zsign * z0,
                        area: a,
                        sigma_t: fy,
                        sigma_c: -fy,
                    });
                }
            }
        }
    }

    // 幅方向主筋（側面、内側区間）
    let set = &rebar.main_y;
    if set.count > 0 {
        let a = bar(set);
        for layer in 0..set.layers.max(1) {
            let y0 = b / 2.0 - rebar.cover - layer as f64 * 2.5 * set.dia;
            let span = d - 2.0 * rebar.cover;
            for i in 0..set.count {
                // 端点（隅角部）を除いた内分点に配置
                let z = -span / 2.0 + span * (i as f64 + 1.0) / (set.count + 1) as f64;
                for ysign in [1.0, -1.0] {
                    fibers.push(PlasticFiber {
                        y: ysign * y0,
                        z,
                        area: a,
                        sigma_t: fy,
                        sigma_c: -fy,
                    });
                }
            }
        }
    }
}

/// RC 円形断面の主筋バネ（main_x + main_y の合計本数を円周上へ等配）。
fn rebar_fibers_circle(fibers: &mut Vec<PlasticFiber>, rebar: &RcRebar, d: f64, fy: f64) {
    let total = (rebar.main_x.count + rebar.main_y.count) as usize;
    if total == 0 {
        return;
    }
    let dia = if rebar.main_x.count > 0 {
        rebar.main_x.dia
    } else {
        rebar.main_y.dia
    };
    let a = std::f64::consts::PI * dia * dia / 4.0;
    let r = d / 2.0 - rebar.cover;
    for i in 0..total {
        let th = 2.0 * std::f64::consts::PI * i as f64 / total as f64;
        fibers.push(PlasticFiber {
            y: r * th.cos(),
            z: r * th.sin(),
            area: a,
            sigma_t: fy,
            sigma_c: -fy,
        });
    }
}

/// 断面形状からファイバ/バネ配置を生成する。
///
/// `kind` により解像度が変わる:
/// - `MultiFiber` / `SimpleSpring`: 細分割（最大寸法の 1/40 目安）。
///   単純降伏バネの耐力算定にも細分割ファイバを用いる。
/// - `MultiSpring`: 粗い配置（最大寸法の 1/4 目安、鋼管・円形は周 8 分割）。
///   主筋は本数が少ないためどちらも1本ずつバネとして配置する。
///
/// 非対称断面（山形・溝形・T形）は生成後に断面積重心へ座標を平行移動する。
pub fn plastic_fibers(
    shape: &SectionShape,
    strength: &StrengthParams,
    kind: YieldModelKind,
) -> Vec<PlasticFiber> {
    let fine = !matches!(kind, YieldModelKind::MultiSpring);
    let fy = strength.steel_fy;
    let fc = strength.concrete_fc;
    let mut fibers = Vec::new();

    // 最大寸法に対する目標ファイバ寸法
    let max_dim = match *shape {
        SectionShape::SteelH { height, width, .. }
        | SectionShape::SteelBox { height, width, .. }
        | SectionShape::SteelChannel { height, width, .. }
        | SectionShape::SteelTee { height, width, .. } => height.max(width),
        SectionShape::SteelAngle { leg_a, leg_b, .. } => leg_a.max(leg_b),
        SectionShape::SteelPipe { outer_dia, .. } => outer_dia,
        SectionShape::RcRect { b, d, .. } => b.max(d),
        SectionShape::RcCircle { d, .. } => d,
    };
    let target = if fine { max_dim / 40.0 } else { max_dim / 4.0 };

    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let hw = height - 2.0 * flange_thick;
            mesh_rect(
                &mut fibers,
                [0.0, (height - flange_thick) / 2.0],
                width,
                flange_thick,
                target,
                (fy, -fy),
            );
            mesh_rect(
                &mut fibers,
                [0.0, -(height - flange_thick) / 2.0],
                width,
                flange_thick,
                target,
                (fy, -fy),
            );
            mesh_rect(&mut fibers, [0.0, 0.0], web_thick, hw, target, (fy, -fy));
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
        } => {
            let hw = height - 2.0 * thick;
            mesh_rect(
                &mut fibers,
                [0.0, (height - thick) / 2.0],
                width,
                thick,
                target,
                (fy, -fy),
            );
            mesh_rect(
                &mut fibers,
                [0.0, -(height - thick) / 2.0],
                width,
                thick,
                target,
                (fy, -fy),
            );
            for ysign in [1.0, -1.0] {
                mesh_rect(
                    &mut fibers,
                    [ysign * (width - thick) / 2.0, 0.0],
                    thick,
                    hw,
                    target,
                    (fy, -fy),
                );
            }
        }
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            // 縦脚 leg_a（z 方向）× 厚、横脚 leg_b（y 方向）× 厚（重なりは縦脚に含める）
            mesh_rect(
                &mut fibers,
                [thick / 2.0, leg_a / 2.0],
                thick,
                leg_a,
                target,
                (fy, -fy),
            );
            mesh_rect(
                &mut fibers,
                [thick + (leg_b - thick) / 2.0, thick / 2.0],
                leg_b - thick,
                thick,
                target,
                (fy, -fy),
            );
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let hw = height - 2.0 * flange_thick;
            // ウェブを y=0 起点に置き、後で重心補正する
            mesh_rect(
                &mut fibers,
                [web_thick / 2.0, 0.0],
                web_thick,
                hw,
                target,
                (fy, -fy),
            );
            for zsign in [1.0, -1.0] {
                mesh_rect(
                    &mut fibers,
                    [width / 2.0, zsign * (height - flange_thick) / 2.0],
                    width,
                    flange_thick,
                    target,
                    (fy, -fy),
                );
            }
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let hw = height - flange_thick;
            mesh_rect(
                &mut fibers,
                [0.0, (height - flange_thick) / 2.0],
                width,
                flange_thick,
                target,
                (fy, -fy),
            );
            mesh_rect(
                &mut fibers,
                [
                    0.0,
                    (height - flange_thick) / 2.0 - flange_thick / 2.0 - hw / 2.0,
                ],
                web_thick,
                hw,
                target,
                (fy, -fy),
            );
        }
        SectionShape::SteelPipe { outer_dia, thick } => {
            let n_theta = if fine { 48 } else { 8 };
            let n_r = if fine { 4 } else { 1 };
            mesh_annulus(&mut fibers, outer_dia, thick, n_theta, n_r, fy, -fy);
        }
        SectionShape::RcRect { b, d, ref rebar } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, (0.0, -fc));
            rebar_fibers_rect(&mut fibers, rebar, b, d, strength.rebar_fy);
        }
        SectionShape::RcCircle { d, ref rebar } => {
            // 中実円 = 厚 d/2 の円環
            let n_theta = if fine { 48 } else { 8 };
            let n_r = if fine { 12 } else { 2 };
            mesh_annulus(&mut fibers, d, d / 2.0, n_theta, n_r, 0.0, -fc);
            rebar_fibers_circle(&mut fibers, rebar, d, strength.rebar_fy);
        }
    }

    // 非対称断面は断面積重心まわりへ座標補正（曲げの基準軸を図心に取る）
    if matches!(
        shape,
        SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
    ) {
        let a_sum: f64 = fibers.iter().map(|f| f.area).sum();
        if a_sum > 0.0 {
            let cy: f64 = fibers.iter().map(|f| f.area * f.y).sum::<f64>() / a_sum;
            let cz: f64 = fibers.iter().map(|f| f.area * f.z).sum::<f64>() / a_sum;
            for f in &mut fibers {
                f.y -= cy;
                f.z -= cz;
            }
        }
    }

    fibers
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use squid_n_core::section_shape::ShearBar;

    fn steel_rect_fibers(b: f64, d: f64, fy: f64, n: usize) -> Vec<PlasticFiber> {
        let mut fibers = Vec::new();
        mesh_rect(&mut fibers, [0.0, 0.0], b, d, d / n as f64, (fy, -fy));
        fibers
    }

    #[test]
    fn test_axial_capacity_steel_rect() {
        let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 40);
        let (nc, nt) = axial_capacity(&fibers);
        let npl = 100.0 * 200.0 * 235.0;
        assert_relative_eq!(nt, npl, max_relative = 1e-9);
        assert_relative_eq!(nc, -npl, max_relative = 1e-9);
    }

    #[test]
    fn test_plastic_moment_steel_rect() {
        // 矩形鋼断面の全塑性モーメント Mp = fy·b·d²/4
        let (b, d, fy) = (100.0, 200.0, 235.0);
        let fibers = steel_rect_fibers(b, d, fy, 200);
        let m = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap();
        let mp_exact = fy * b * d * d / 4.0;
        assert_relative_eq!(m[0], mp_exact, max_relative = 1e-2);
        assert_relative_eq!(m[1], 0.0, epsilon = mp_exact * 1e-9);
    }

    #[test]
    fn test_mn_interaction_steel_rect() {
        // 矩形鋼断面の厳密解: M/Mp = 1 − (N/Npl)²
        let (b, d, fy) = (100.0, 200.0, 235.0);
        let fibers = steel_rect_fibers(b, d, fy, 400);
        let npl = b * d * fy;
        let mp = fy * b * d * d / 4.0;
        for ratio in [0.25, 0.5, 0.75] {
            let m = plastic_moment_at_n(&fibers, 1.0, 0.0, ratio * npl).unwrap();
            let m_exact = mp * (1.0 - ratio * ratio);
            assert_relative_eq!(m[0], m_exact, max_relative = 1e-2);
        }
    }

    #[test]
    fn test_plastic_moment_outside_range() {
        let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 20);
        let npl = 100.0 * 200.0 * 235.0;
        assert!(plastic_moment_at_n(&fibers, 1.0, 0.0, npl * 1.01).is_none());
        assert!(plastic_moment_at_n(&fibers, 1.0, 0.0, -npl * 1.01).is_none());
    }

    fn sample_rc_shape() -> SectionShape {
        SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 2,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        }
    }

    #[test]
    fn test_rc_rect_capacity() {
        let strength = StrengthParams::default();
        let fibers = plastic_fibers(&sample_rc_shape(), &strength, YieldModelKind::MultiFiber);
        // 引張耐力 = 主筋のみ: (4×2 + 2×2) 本 × a × fy
        let a_bar = std::f64::consts::PI * 22.0 * 22.0 / 4.0;
        let nt_exact = 12.0 * a_bar * strength.rebar_fy;
        let (nc, nt) = axial_capacity(&fibers);
        assert_relative_eq!(nt, nt_exact, max_relative = 1e-9);
        // 圧縮耐力 = コンクリート全断面 + 主筋
        let nc_exact = -(500.0 * 500.0 * strength.concrete_fc + 12.0 * a_bar * strength.rebar_fy);
        assert_relative_eq!(nc, nc_exact, max_relative = 1e-9);
    }

    #[test]
    fn test_rc_moment_increases_with_moderate_compression() {
        // RC 断面は適度な圧縮軸力下で曲げ耐力が増す（相関曲線のふくらみ）
        let strength = StrengthParams::default();
        let fibers = plastic_fibers(&sample_rc_shape(), &strength, YieldModelKind::MultiFiber);
        let (nc, _) = axial_capacity(&fibers);
        let m0 = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap()[0];
        let m_comp = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.3 * nc).unwrap()[0];
        assert!(
            m_comp > m0,
            "M at 0.3Nc ({m_comp}) must exceed M at N=0 ({m0})"
        );
    }

    #[test]
    fn test_build_surface_grid_shape_and_poles() {
        let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 40);
        let surf = build_surface(&fibers, YieldModelKind::MultiFiber, 16, 32);
        assert_eq!(surf.grid.len(), 17);
        assert!(surf.grid.iter().all(|row| row.len() == 32));
        // 極（α=0/π）は純引張/純圧縮で一定
        let npl = 100.0 * 200.0 * 235.0;
        for p in &surf.grid[0] {
            assert_relative_eq!(p[0], npl, max_relative = 1e-9);
        }
        for p in &surf.grid[16] {
            assert_relative_eq!(p[0], -npl, max_relative = 1e-9);
        }
        // 全点が有限値
        assert!(surf
            .grid
            .iter()
            .flatten()
            .all(|p| p.iter().all(|v| v.is_finite())));
    }

    #[test]
    fn test_simple_spring_surface_linear_interaction() {
        let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 100);
        let surf = build_simple_spring_surface(&fibers, 16, 32);
        let npl = 100.0 * 200.0 * 235.0;
        // 極は軸耐力に一致
        for p in &surf.grid[0] {
            assert_relative_eq!(p[0], npl, max_relative = 1e-6);
        }
        for p in &surf.grid[16] {
            assert_relative_eq!(p[0], -npl, max_relative = 1e-6);
        }
        // 全格子点が |N|/N許容 + √((My/Mp_y)² + (Mz/Mp_z)²) = 1 を満たす
        for p in surf.grid.iter().flatten() {
            let n_ref = if p[0] >= 0.0 {
                surf.n_tens
            } else {
                surf.n_comp.abs()
            };
            let f = p[0].abs() / n_ref
                + ((p[1] / surf.mp_y).powi(2) + (p[2] / surf.mp_z).powi(2)).sqrt();
            assert_relative_eq!(f, 1.0, max_relative = 1e-9);
        }
        // 赤道（α=π/2、i=8）は N=0 の全塑性モーメント楕円: β=0 で My = Mp_y
        let equator = &surf.grid[8];
        assert_relative_eq!(equator[0][0], 0.0, epsilon = npl * 1e-12);
        assert_relative_eq!(equator[0][1], surf.mp_y, max_relative = 1e-9);
    }

    #[test]
    fn test_slice_at_n_symmetric() {
        let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 100);
        let pts = slice_at_n(&fibers, 0.0, 16);
        assert_eq!(pts.len(), 16);
        // β=0 は純 y 軸まわり曲げ → My = Mp_y
        let mp_y = 235.0 * 100.0 * 200.0 * 200.0 / 4.0;
        assert_relative_eq!(pts[0][0], mp_y, max_relative = 1e-2);
        // 対称性: β と β+π で符号反転
        assert_relative_eq!(pts[0][0], -pts[8][0], max_relative = 1e-9);
    }

    #[test]
    fn test_multispring_is_coarser_than_fiber() {
        let strength = StrengthParams::default();
        let shape = sample_rc_shape();
        let ms = plastic_fibers(&shape, &strength, YieldModelKind::MultiSpring);
        let fib = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
        assert!(
            ms.len() < fib.len() / 10,
            "MS ({}) must be much coarser than fiber ({})",
            ms.len(),
            fib.len()
        );
        // 軸耐力は離散化によらず一致する（面積保存）
        let (nc_ms, nt_ms) = axial_capacity(&ms);
        let (nc_f, nt_f) = axial_capacity(&fib);
        assert_relative_eq!(nc_ms, nc_f, max_relative = 1e-9);
        assert_relative_eq!(nt_ms, nt_f, max_relative = 1e-9);
    }

    #[test]
    fn test_steel_h_plastic_moment() {
        // H-400×200×8×13 の Mp ≈ fy × Zp（Zp = 手計算）
        let shape = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        };
        let strength = StrengthParams::default();
        let fibers = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
        // Zp = B·tf·(H−tf) + tw·(H−2tf)²/4
        let zp =
            200.0 * 13.0 * (400.0 - 13.0) + 8.0 * (400.0 - 2.0 * 13.0) * (400.0 - 2.0 * 13.0) / 4.0;
        let m = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(m[0], 235.0 * zp, max_relative = 2e-2);
    }

    #[test]
    fn test_tee_centroid_correction() {
        // 非対称断面（T形）: 図心補正後、純軸力で M が出ないこと
        let shape = SectionShape::SteelTee {
            height: 200.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 12.0,
        };
        let strength = StrengthParams::default();
        let fibers = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
        let a_sum: f64 = fibers.iter().map(|f| f.area).sum();
        let cz: f64 = fibers.iter().map(|f| f.area * f.z).sum::<f64>() / a_sum;
        assert_relative_eq!(cz, 0.0, epsilon = 1e-9);
    }
}
