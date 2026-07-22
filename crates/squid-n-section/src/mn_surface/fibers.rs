//! 断面形状（[`SectionShape`]）から全塑性計算用のファイバ/バネ配置を生成する。
//!
//! 矩形・円環メッシュ、H 形／箱形の板メッシュ、主筋バネ配置、形状別ディスパッチ、
//! 非対称断面の図心補正を担う。

use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

use super::types::{concrete_young, PlasticFiber, StrengthParams, YieldModelKind};

/// ファイバ材料（限界応力と弾性係数）。
#[derive(Clone, Copy)]
pub(crate) struct FiberMat {
    pub sigma_t: f64,
    pub sigma_c: f64,
    pub young: f64,
}

/// 矩形領域（中心 `center = [cy, cz]`、幅 w × 高さ h）を目標寸法 `target` 以下の
/// ファイバに等分割して追加する。
pub(crate) fn mesh_rect(
    fibers: &mut Vec<PlasticFiber>,
    center: [f64; 2],
    w: f64,
    h: f64,
    target: f64,
    mat: FiberMat,
) {
    let [cy, cz] = center;
    let FiberMat {
        sigma_t,
        sigma_c,
        young,
    } = mat;
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
                young,
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
    mat: FiberMat,
) {
    let FiberMat {
        sigma_t,
        sigma_c,
        young,
    } = mat;
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
                young,
            });
        }
    }
}

/// H 形（上下フランジ＋ウェブ）を板ごとにメッシュ化して追加する。
/// SteelH と SrcRect 内蔵鉄骨で共用する。
fn mesh_h_plates(
    fibers: &mut Vec<PlasticFiber>,
    height: f64,
    width: f64,
    web_thick: f64,
    flange_thick: f64,
    target: f64,
    mat: FiberMat,
) {
    let hw = height - 2.0 * flange_thick;
    mesh_rect(
        fibers,
        [0.0, (height - flange_thick) / 2.0],
        width,
        flange_thick,
        target,
        mat,
    );
    mesh_rect(
        fibers,
        [0.0, -(height - flange_thick) / 2.0],
        width,
        flange_thick,
        target,
        mat,
    );
    mesh_rect(fibers, [0.0, 0.0], web_thick, hw, target, mat);
}

/// 箱形（角形鋼管）の 4 枚板（上下フランジ＋左右ウェブ）をメッシュ化して追加する。
/// SteelBox と CftBox の鋼管部で共用する。
fn mesh_box_plates(
    fibers: &mut Vec<PlasticFiber>,
    height: f64,
    width: f64,
    thick: f64,
    target: f64,
    mat: FiberMat,
) {
    let hw = height - 2.0 * thick;
    mesh_rect(
        fibers,
        [0.0, (height - thick) / 2.0],
        width,
        thick,
        target,
        mat,
    );
    mesh_rect(
        fibers,
        [0.0, -(height - thick) / 2.0],
        width,
        thick,
        target,
        mat,
    );
    for ysign in [1.0, -1.0] {
        mesh_rect(
            fibers,
            [ysign * (width - thick) / 2.0, 0.0],
            thick,
            hw,
            target,
            mat,
        );
    }
}

/// 主筋1セット分のバネを追加する。
///
/// - `main_x`（せい方向主筋）: 上下面（z = ±(d/2 − cover)）に各 `count` 本を幅方向へ等配。
/// - `main_y`（幅方向主筋）: 側面（y = ±(b/2 − cover)）に各 `count` 本をせい方向の
///   内側区間へ等配（隅角部は main_x 側に含める）。
/// - `layers`: 2段目以降は 2.5×径 ずつ内側へ配置する。
fn rebar_fibers_rect(
    fibers: &mut Vec<PlasticFiber>,
    rebar: &RcRebar,
    b: f64,
    d: f64,
    fy: f64,
    young: f64,
) {
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
                        young,
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
                        young,
                    });
                }
            }
        }
    }
}

/// RC 円形断面の主筋バネ（main_x + main_y の合計本数を円周上へ等配）。
fn rebar_fibers_circle(
    fibers: &mut Vec<PlasticFiber>,
    rebar: &RcRebar,
    d: f64,
    fy: f64,
    young: f64,
) {
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
            young,
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
    let steel = FiberMat {
        sigma_t: fy,
        sigma_c: -fy,
        young: strength.steel_e,
    };
    let conc = FiberMat {
        sigma_t: 0.0,
        sigma_c: -fc,
        young: concrete_young(fc),
    };
    let mut fibers = Vec::new();

    // 最大寸法に対する目標ファイバ寸法
    let max_dim = match *shape {
        SectionShape::SteelH { height, width, .. }
        | SectionShape::SteelBox { height, width, .. }
        | SectionShape::SteelChannel { height, width, .. }
        | SectionShape::SteelTee { height, width, .. } => height.max(width),
        SectionShape::SteelAngle { leg_a, leg_b, .. } => leg_a.max(leg_b),
        SectionShape::SteelPipe { outer_dia, .. } => outer_dia,
        SectionShape::SteelFlatBar { width, thick } => width.max(thick),
        SectionShape::SteelRoundBar { dia } => dia,
        SectionShape::SteelLipChannel { height, width, .. } => height.max(width),
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            lower_width,
            ..
        } => height.max(upper_width).max(lower_width),
        SectionShape::RcRect { b, d, .. } => b.max(d),
        SectionShape::RcCircle { d, .. } => d,
        SectionShape::SrcRect { b, d, .. } => b.max(d),
        SectionShape::CftBox { height, width, .. } => height.max(width),
        SectionShape::CftPipe { outer_dia, .. } => outer_dia,
        SectionShape::RcWall { thickness, .. } => thickness.max(1000.0),
    };
    let target = if fine { max_dim / 40.0 } else { max_dim / 4.0 };

    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            mesh_h_plates(
                &mut fibers,
                height,
                width,
                web_thick,
                flange_thick,
                target,
                steel,
            );
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
            ..
        } => {
            mesh_box_plates(&mut fibers, height, width, thick, target, steel);
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
                steel,
            );
            mesh_rect(
                &mut fibers,
                [thick + (leg_b - thick) / 2.0, thick / 2.0],
                leg_b - thick,
                thick,
                target,
                steel,
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
                steel,
            );
            for zsign in [1.0, -1.0] {
                mesh_rect(
                    &mut fibers,
                    [width / 2.0, zsign * (height - flange_thick) / 2.0],
                    width,
                    flange_thick,
                    target,
                    steel,
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
                steel,
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
                steel,
            );
        }
        SectionShape::SteelPipe { outer_dia, thick } => {
            let n_theta = if fine { 48 } else { 8 };
            let n_r = if fine { 4 } else { 1 };
            mesh_annulus(&mut fibers, outer_dia, thick, n_theta, n_r, steel);
        }
        SectionShape::SteelFlatBar { width, thick } => {
            // 中実矩形（幅 width×せい thick）を鋼ファイバで充填。
            mesh_rect(&mut fibers, [0.0, 0.0], width, thick, target, steel);
        }
        SectionShape::SteelRoundBar { dia } => {
            // 中実円 = 厚 dia/2 の円環を鋼ファイバで充填。
            let n_theta = if fine { 48 } else { 8 };
            let n_r = if fine { 12 } else { 2 };
            mesh_annulus(&mut fibers, dia, dia / 2.0, n_theta, n_r, steel);
        }
        SectionShape::SteelLipChannel {
            height,
            width,
            lip,
            thick,
        } => {
            let t = thick;
            // ウェブ・上下フランジ・上下リップの 5 枚（重なり無し）。座標は後で図心補正。
            mesh_rect(
                &mut fibers,
                [t / 2.0, height / 2.0],
                t,
                height,
                target,
                steel,
            );
            for ysign in [1.0, -1.0] {
                // フランジ（y=±(H−t)/2）
                mesh_rect(
                    &mut fibers,
                    [(t + width) / 2.0, height / 2.0 + ysign * (height - t) / 2.0],
                    width - t,
                    t,
                    target,
                    steel,
                );
                // リップ（y=±(H−C−t)/2）
                mesh_rect(
                    &mut fibers,
                    [
                        width - t / 2.0,
                        height / 2.0 + ysign * (height - lip - t) / 2.0,
                    ],
                    t,
                    lip - t,
                    target,
                    steel,
                );
            }
        }
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            // 上下フランジ（幅が異なる）＋ウェブ。座標は後で図心補正。
            let hw = (height - upper_thick - lower_thick).max(0.0);
            mesh_rect(
                &mut fibers,
                [0.0, height - upper_thick / 2.0],
                upper_width,
                upper_thick,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [0.0, lower_thick / 2.0],
                lower_width,
                lower_thick,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [0.0, lower_thick + hw / 2.0],
                web_thick,
                hw,
                target,
                steel,
            );
        }
        SectionShape::RcRect { b, d, ref rebar } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            rebar_fibers_rect(
                &mut fibers,
                rebar,
                b,
                d,
                strength.rebar_fy,
                strength.steel_e,
            );
        }
        SectionShape::RcCircle { d, ref rebar } => {
            // 中実円 = 厚 d/2 の円環
            let n_theta = if fine { 48 } else { 8 };
            let n_r = if fine { 12 } else { 2 };
            mesh_annulus(&mut fibers, d, d / 2.0, n_theta, n_r, conc);
            rebar_fibers_circle(&mut fibers, rebar, d, strength.rebar_fy, strength.steel_e);
        }
        SectionShape::SrcRect {
            b,
            d,
            ref rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            ..
        } => {
            // RC 部分（コンクリート + 主筋）
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            rebar_fibers_rect(
                &mut fibers,
                rebar,
                b,
                d,
                strength.rebar_fy,
                strength.steel_e,
            );
            // 内蔵 H 形鉄骨（断面中心配置。コンクリートとの重複控除は省略＝
            // 単純累加の近似。鉄骨断面積はコンクリートの数%のため影響軽微）
            mesh_h_plates(
                &mut fibers,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                target,
                steel,
            );
        }
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            // 鋼管部分（SteelBox と同じ 4 枚の板）
            mesh_box_plates(&mut fibers, height, width, thick, target, steel);
            // 充填コンクリート
            mesh_rect(
                &mut fibers,
                [0.0, 0.0],
                width - 2.0 * thick,
                height - 2.0 * thick,
                target,
                conc,
            );
        }
        SectionShape::CftPipe { outer_dia, thick } => {
            let n_theta = if fine { 48 } else { 8 };
            let n_r_s = if fine { 4 } else { 1 };
            let n_r_c = if fine { 12 } else { 2 };
            // 鋼管
            mesh_annulus(&mut fibers, outer_dia, thick, n_theta, n_r_s, steel);
            // 充填コンクリート（中実円 = 厚 di/2 の円環）
            let di = outer_dia - 2.0 * thick;
            if di > 0.0 {
                mesh_annulus(&mut fibers, di, di / 2.0, n_theta, n_r_c, conc);
            }
        }
        SectionShape::RcWall { thickness, .. } => {
            // 名目: 1m 幅の無筋板（壁の MN 曲線は対象外だがパニックさせない）
            mesh_rect(&mut fibers, [0.0, 0.0], 1000.0, thickness, target, conc);
        }
    }

    // 非対称断面は断面積重心まわりへ座標補正（曲げの基準軸を図心に取る）
    if matches!(
        shape,
        SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
            | SectionShape::SteelLipChannel { .. }
            | SectionShape::SteelBuiltH { .. }
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
