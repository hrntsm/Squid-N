//! T2: 偏心率 Re（剛心＝武藤 D値法・略算）。仕様 `specs/P7_二次設計.md` §5。
//!
//! 本モジュールは2層構造になっている:
//! 1. **厳密な計算コア**（`d_value` / `center_of_rigidity` / `eccentricity`）。
//!    告示1792・武藤 D値法の閉形式そのもので、手計算と 1e-9 で一致する（DoD §8.1）。
//! 2. **モデル抽出**（`column_stiffnesses` / `center_of_mass` / `story_centers`）。
//!    実モデルから柱・梁を拾って 1. に渡す略算層。柱＝鉛直部材という幾何判定等、
//!    明示した仮定の上に成り立つ（精算＝マスター節点 3×3 剛性は将来）。
//!
//! **方向の扱い（★最重要）:** 剛心座標は方向別 D 値で重み付けする。
//! `Xs = Σ(Dy·x)/ΣDy`, `Ys = Σ(Dx·y)/ΣDx`。単一 D 値で済むのは対称架構のみ。

use squid_n_core::ids::StoryId;
use squid_n_core::model::{ElementKind, Model};
use squid_n_element::transform::LocalFrame;

/// 1 本の柱（鉛直部材）の、平面位置と方向別水平剛性（D値）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColumnStiffness {
    /// 柱の平面位置 (x, y) [mm]。
    pub pos: [f64; 2],
    /// X 加力方向の水平剛性 Dx [N/mm]。
    pub dx: f64,
    /// Y 加力方向の水平剛性 Dy [N/mm]。
    pub dy: f64,
}

/// 武藤 D 値の閉形式（仕様 §5.1）。加力方向ごとに呼ぶ。
///
/// - `e`: ヤング係数 [N/mm²]
/// - `ic`: 加力方向の柱断面二次モーメント [mm⁴]
/// - `h`: 階高（柱長）[mm]
/// - `sum_beam_stiffness_ratio`: 柱頭・柱脚に取り付く、加力方向に効く梁の剛比 ΣKb（= Σ Ib/Lb）
/// - `first_story`: 最下階（柱脚固定）なら true。一般階は false。
///
/// ```text
/// Kc0 = 12·E·Ic/h³,  kc = Ic/h,  k̄ = ΣKb/(2·kc)
/// a   = k̄/(2+k̄)            （一般階）
///     = (0.5+k̄)/(2+k̄)      （最下階・柱脚固定）
/// D   = a · Kc0
/// ```
pub fn d_value(e: f64, ic: f64, h: f64, sum_beam_stiffness_ratio: f64, first_story: bool) -> f64 {
    if h <= 0.0 || ic <= 0.0 {
        return 0.0;
    }
    let kc0 = 12.0 * e * ic / (h * h * h);
    let kc = ic / h;
    if kc <= 0.0 {
        return 0.0;
    }
    let kbar = sum_beam_stiffness_ratio / (2.0 * kc);
    let a = if first_story {
        (0.5 + kbar) / (2.0 + kbar)
    } else {
        kbar / (2.0 + kbar)
    };
    a * kc0
}

/// 剛心座標 [Xs, Ys]。`Xs = Σ(Dy·x)/ΣDy`, `Ys = Σ(Dx·y)/ΣDx`（仕様 §5.1）。
pub fn center_of_rigidity(cols: &[ColumnStiffness]) -> [f64; 2] {
    let sum_dy: f64 = cols.iter().map(|c| c.dy).sum();
    let sum_dx: f64 = cols.iter().map(|c| c.dx).sum();
    let xs = if sum_dy == 0.0 {
        0.0
    } else {
        cols.iter().map(|c| c.dy * c.pos[0]).sum::<f64>() / sum_dy
    };
    let ys = if sum_dx == 0.0 {
        0.0
    } else {
        cols.iter().map(|c| c.dx * c.pos[1]).sum::<f64>() / sum_dx
    };
    [xs, ys]
}

/// 偏心率の算定結果（X 加力・Y 加力）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Eccentricity {
    /// 偏心距離 ex = |Xg − Xs| [mm]。
    pub ex: f64,
    /// 偏心距離 ey = |Yg − Ys| [mm]。
    pub ey: f64,
    /// ねじり剛性 KR = Σ(Dx·ȳ²) + Σ(Dy·x̄²)（剛心まわり）。
    pub kr: f64,
    /// 弾力半径 rex = √(KR/ΣDx)。
    pub rex: f64,
    /// 弾力半径 rey = √(KR/ΣDy)。
    pub rey: f64,
    /// X 加力時の偏心率 Rex = ey/rex（規定 ≤ 0.15）。
    pub re_x: f64,
    /// Y 加力時の偏心率 Rey = ex/rey（規定 ≤ 0.15）。
    pub re_y: f64,
}

/// 剛心・重心・柱剛性から偏心率を算定（仕様 §5.2）。
pub fn eccentricity(
    cols: &[ColumnStiffness],
    center_of_mass: [f64; 2],
    center_of_rigidity: [f64; 2],
) -> Eccentricity {
    let [xs, ys] = center_of_rigidity;
    let [xg, yg] = center_of_mass;
    let ex = (xg - xs).abs();
    let ey = (yg - ys).abs();

    let sum_dx: f64 = cols.iter().map(|c| c.dx).sum();
    let sum_dy: f64 = cols.iter().map(|c| c.dy).sum();

    // 剛心まわりのねじり剛性。x̄, ȳ は剛心からの距離。
    let kr: f64 = cols
        .iter()
        .map(|c| {
            let xbar = c.pos[0] - xs;
            let ybar = c.pos[1] - ys;
            c.dx * ybar * ybar + c.dy * xbar * xbar
        })
        .sum();

    let rex = if sum_dx > 0.0 {
        (kr / sum_dx).sqrt()
    } else {
        0.0
    };
    let rey = if sum_dy > 0.0 {
        (kr / sum_dy).sqrt()
    } else {
        0.0
    };
    let re_x = if rex > 0.0 { ey / rex } else { 0.0 };
    let re_y = if rey > 0.0 { ex / rey } else { 0.0 };

    Eccentricity {
        ex,
        ey,
        kr,
        rex,
        rey,
        re_x,
        re_y,
    }
}

// ===== モデル抽出層（略算）=====

/// 重心（質量中心）[Xg, Yg]。当該層の節点質量（並進成分）で重み付けする。
///
/// 質量未定義の節点は質量 0（剛心の重み付けには寄与しない）。全質量 0 なら幾何重心。
pub fn center_of_mass(model: &Model, story: StoryId) -> [f64; 2] {
    let nodes: Vec<&squid_n_core::model::Node> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(story))
        .collect();
    if nodes.is_empty() {
        return [0.0, 0.0];
    }
    let mass = |n: &squid_n_core::model::Node| n.mass.map(|m| m[0]).unwrap_or(0.0);
    let total: f64 = nodes.iter().map(|n| mass(n)).sum();
    if total > 0.0 {
        let xg = nodes.iter().map(|n| mass(n) * n.coord[0]).sum::<f64>() / total;
        let yg = nodes.iter().map(|n| mass(n) * n.coord[1]).sum::<f64>() / total;
        [xg, yg]
    } else {
        // 質量未定義 → 幾何重心で代用。
        let n = nodes.len() as f64;
        let xg = nodes.iter().map(|n| n.coord[0]).sum::<f64>() / n;
        let yg = nodes.iter().map(|n| n.coord[1]).sum::<f64>() / n;
        [xg, yg]
    }
}

// ===== モデル自動算定層（column_stiffnesses / StoryCenters / story_centers / story_eccentricity）=====

/// 当該層の各柱について方向別水平剛性（D値）と平面位置を算定して返す（仕様 §5.1）。
///
/// 柱の判定: `ElementKind::Beam` かつ 2節点、部材軸 ez[2].abs() > 0.707 で鉛直判定。
/// 層帰属: 上端節点（z 大）の `story == Some(story)` を当該層とする。
pub fn column_stiffnesses(model: &Model, story: StoryId) -> Vec<ColumnStiffness> {
    // 最下層判定: 当該 story の elevation が全 stories 中で最小なら true。
    let min_elev: f64 = model
        .stories
        .iter()
        .map(|s| s.elevation)
        .fold(f64::INFINITY, f64::min);
    let this_elev = model
        .stories
        .get(story.index())
        .map(|s| s.elevation)
        .unwrap_or(f64::INFINITY);
    let first_story = (this_elev - min_elev).abs() < 1e-9;

    let mut result = Vec::new();

    for elem in &model.elements {
        // 2節点 Beam のみ対象。
        if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let nid0 = elem.nodes[0];
        let nid1 = elem.nodes[1];
        let n0 = &model.nodes[nid0.index()];
        let n1 = &model.nodes[nid1.index()];
        let p0 = n0.coord;
        let p1 = n1.coord;

        // 部材軸単位ベクトル（i→j）。
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        if l < 1e-12 {
            continue;
        }
        let ex_z = dz / l;

        // 鉛直部材（柱）判定。
        if ex_z.abs() <= 0.707 {
            continue;
        }

        // 上端節点（z が大きい方）。
        let (n_top, n_bot, p_top, p_bot) = if p0[2] < p1[2] {
            (n1, n0, p1, p0)
        } else {
            (n0, n1, p0, p1)
        };

        // 層帰属: 上端節点の story が当該層。
        if n_top.story != Some(story) {
            continue;
        }

        // material / section が必須。
        let mid = match elem.material {
            Some(m) => m,
            None => continue,
        };
        let sid = match elem.section {
            Some(s) => s,
            None => continue,
        };
        let mat = &model.materials[mid.index()];
        let sec = &model.sections[sid.index()];
        let e = mat.young;
        let h = (p_top[2] - p_bot[2]).abs();
        if h < 1e-12 {
            continue;
        }

        // 局所座標系から ey, ez を取得。
        let ref_vec = elem.local_axis.ref_vector;
        let frame = LocalFrame::from_nodes(p0, p1, ref_vec);
        let ey = frame.rot[1]; // 局所 y 軸（全体方向への射影に使う）
        let ez = frame.rot[2]; // 局所 z 軸

        // 方向別有効断面二次モーメント（局所→全体の射影）。
        // 全体 X 方向変位に抵抗: 局所 y 方向成分 iz, 局所 z 方向成分 iy。
        let iy = sec.iy;
        let iz = sec.iz;
        let i_global_x = iz * ey[0] * ey[0] + iy * ez[0] * ez[0];
        let i_global_y = iz * ey[1] * ey[1] + iy * ez[1] * ez[1];

        // 梁剛比 ΣKb（武藤 a 補正用）。当該柱の上端・下端節点に取り付く水平梁を探す。
        let (sum_kb_x, sum_kb_y) = {
            let mut skbx = 0.0_f64;
            let mut skby = 0.0_f64;
            for other in &model.elements {
                if other.id == elem.id {
                    continue;
                }
                if other.kind != ElementKind::Beam || other.nodes.len() != 2 {
                    continue;
                }
                // 当該柱の節点（上端または下端）を含む梁か。
                let has_top = other.nodes.contains(&n_top.id);
                let has_bot = other.nodes.contains(&n_bot.id);
                if !has_top && !has_bot {
                    continue;
                }
                // 梁の部材軸単位ベクトル。
                let bn0 = &model.nodes[other.nodes[0].index()];
                let bn1 = &model.nodes[other.nodes[1].index()];
                let bdx = bn1.coord[0] - bn0.coord[0];
                let bdy = bn1.coord[1] - bn0.coord[1];
                let bdz = bn1.coord[2] - bn0.coord[2];
                let bl = (bdx * bdx + bdy * bdy + bdz * bdz).sqrt();
                if bl < 1e-12 {
                    continue;
                }
                let bex = [bdx / bl, bdy / bl, bdz / bl];
                // 水平部材（梁）判定: ez[2].abs() < 0.707
                if bex[2].abs() >= 0.707 {
                    continue;
                }
                // 梁の断面二次モーメント（強軸 iz）と梁剛比。
                let beam_iz = match other.section {
                    Some(s) => model.sections[s.index()].iz,
                    None => continue,
                };
                let kb = beam_iz / bl;
                // X方向に効く梁: 梁軸 bex[0].abs() > 0.707
                if bex[0].abs() > 0.707 {
                    skbx += kb;
                }
                // Y方向に効く梁: 梁軸 bex[1].abs() > 0.707
                if bex[1].abs() > 0.707 {
                    skby += kb;
                }
            }
            (skbx, skby)
        };

        let dx_val = d_value(e, i_global_x, h, sum_kb_x, first_story);
        let dy_val = d_value(e, i_global_y, h, sum_kb_y, first_story);
        let pos = [p_top[0], p_top[1]];
        result.push(ColumnStiffness {
            pos,
            dx: dx_val,
            dy: dy_val,
        });
    }
    result
}

/// 剛心・重心をまとめた構造体。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoryCenters {
    pub center_of_mass: [f64; 2],
    pub center_of_rigidity: [f64; 2],
}

/// 当該層の剛心・重心を算定して返す。
pub fn story_centers(model: &Model, story: StoryId) -> StoryCenters {
    let cols = column_stiffnesses(model, story);
    let com = center_of_mass(model, story);
    let cor = center_of_rigidity(&cols);
    StoryCenters {
        center_of_mass: com,
        center_of_rigidity: cor,
    }
}

/// 当該層の偏心率を算定して返す。
pub fn story_eccentricity(model: &Model, story: StoryId) -> Eccentricity {
    let mut cols = column_stiffnesses(model, story);
    append_misc_wall_stiffnesses(model, story, &mut cols);
    let cor = center_of_rigidity(&cols);
    let com = center_of_mass(model, story);
    eccentricity(&cols, com, cor)
}

// ===== 精算層（応力解析結果に基づく。RESP-D 計算編 03「応力解析 §偏心率」）=====

use squid_n_solver::linear::StaticOnce;

/// 当該層に帰属する柱（鉛直 2 節点 Beam、柱頭節点の story が当該層）を列挙して
/// `f(elem, 柱頭節点, 柱脚節点)` を呼ぶ。判定は `column_stiffnesses` と同一。
fn for_each_story_column(
    model: &Model,
    story: StoryId,
    mut f: impl FnMut(
        &squid_n_core::model::ElementData,
        &squid_n_core::model::Node,
        &squid_n_core::model::Node,
    ),
) {
    for elem in &model.elements {
        if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let n0 = &model.nodes[elem.nodes[0].index()];
        let n1 = &model.nodes[elem.nodes[1].index()];
        let d = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-12 || (d[2] / l).abs() <= 0.707 {
            continue;
        }
        let (top, bot) = if n0.coord[2] < n1.coord[2] {
            (n1, n0)
        } else {
            (n0, n1)
        };
        if top.story != Some(story) {
            continue;
        }
        f(elem, top, bot);
    }
}

/// 部材始端（xi 最小の評価点）の局所内力 [N, Qy, Qz] を全体座標の力ベクトルへ
/// 変換して返す。`ref_vec` は要素の局所軸参照ベクトル。
fn station_force_global(
    p0: [f64; 3],
    p1: [f64; 3],
    ref_vec: [f64; 3],
    local: [f64; 6],
) -> [f64; 3] {
    let frame = LocalFrame::from_nodes(p0, p1, ref_vec);
    let ex = frame.rot[0];
    let ey = frame.rot[1];
    let ez = frame.rot[2];
    let (n, qy, qz) = (local[0], local[1], local[2]);
    [
        n * ex[0] + qy * ey[0] + qz * ez[0],
        n * ex[1] + qy * ey[1] + qz * ez[1],
        n * ex[2] + qy * ey[2] + qz * ez[2],
    ]
}

/// 地震時応力解析結果から柱の方向別水平剛性 `ki = Qi/δi` を算定する（精算。
/// マニュアル「剛心は、その階の柱の水平方向剛性の中心として求めます。各柱の
/// 水平剛性は、地震時応力解析結果のせん断力と層間変位により計算します」）。
///
/// - `res_x` / `res_y`: X / Y 方向加力時の弾性応力解析結果
/// - `kX` は X 加力時の（せん断力 Qx, 層間変位 δx）から、`kY` は Y 加力時から。
///   δ がほぼ 0 の柱は剛性 0 とする。
pub fn column_stiffnesses_from_analysis(
    model: &Model,
    story: StoryId,
    res_x: &StaticOnce,
    res_y: &StaticOnce,
) -> Vec<ColumnStiffness> {
    use std::collections::HashMap;
    let fx: HashMap<_, _> = res_x.member_forces.iter().map(|(id, f)| (*id, f)).collect();
    let fy: HashMap<_, _> = res_y.member_forces.iter().map(|(id, f)| (*id, f)).collect();

    let mut out = Vec::new();
    for_each_story_column(model, story, |elem, top, bot| {
        let p0 = model.nodes[elem.nodes[0].index()].coord;
        let p1 = model.nodes[elem.nodes[1].index()].coord;
        let k_of =
            |res: &StaticOnce,
             forces: &HashMap<squid_n_core::ids::ElemId, &squid_n_element::beam::MemberForces>,
             dir: usize|
             -> f64 {
                let (Some(ut), Some(ub)) =
                    (res.disp.get(top.id.index()), res.disp.get(bot.id.index()))
                else {
                    return 0.0;
                };
                let delta = (ut[dir] - ub[dir]).abs();
                if delta < 1e-9 {
                    return 0.0;
                }
                let Some(mf) = forces.get(&elem.id) else {
                    return 0.0;
                };
                let Some(&(_, local)) = mf.at.first() else {
                    return 0.0;
                };
                let g = station_force_global(p0, p1, elem.local_axis.ref_vector, local);
                g[dir].abs() / delta
            };
        out.push(ColumnStiffness {
            pos: [top.coord[0], top.coord[1]],
            dx: k_of(res_x, &fx, 0),
            dy: k_of(res_y, &fy, 1),
        });
    });
    out
}

/// 長期応力解析の柱軸力から重心を算定する（マニュアル「各階の重心は、鉛直荷重を
/// 支持する柱の長期荷重による軸力 N およびその部材の平面座標から計算します。
/// gx = Σ(Ni·xi)/ΣNi」）。
///
/// 軸力は圧縮分を重みに用いる（引張柱は鉛直荷重を「支持」しないため重み 0）。
/// ΣNi ≤ 0（柱なし・全柱引張）の場合は `None`（呼び出し側で質量重心へフォールバック）。
pub fn center_of_gravity_from_axial(
    model: &Model,
    story: StoryId,
    long_term: &StaticOnce,
) -> Option<[f64; 2]> {
    use std::collections::HashMap;
    let forces: HashMap<_, _> = long_term
        .member_forces
        .iter()
        .map(|(id, f)| (*id, f))
        .collect();
    let mut sum_n = 0.0;
    let mut sum_nx = 0.0;
    let mut sum_ny = 0.0;
    for_each_story_column(model, story, |elem, top, _bot| {
        let Some(mf) = forces.get(&elem.id) else {
            return;
        };
        let Some(&(_, local)) = mf.at.first() else {
            return;
        };
        // 局所軸力 n は引張正 → 圧縮 = −n を重みに使う。
        let ni = (-local[0]).max(0.0);
        sum_n += ni;
        sum_nx += ni * top.coord[0];
        sum_ny += ni * top.coord[1];
    });
    if sum_n > 0.0 {
        Some([sum_nx / sum_n, sum_ny / sum_n])
    } else {
        None
    }
}

/// 応力解析結果に基づく当該層の偏心率（精算ルート）。
///
/// - 剛心: `column_stiffnesses_from_analysis`（ki = Qi/δi）
/// - 重心: `center_of_gravity_from_axial`（長期軸力。`long_term` が無い/算定不能なら
///   質量重心 `center_of_mass` にフォールバック）
///
/// マニュアル注記「偏心率の計算は常に弾性解析結果から計算されます」に対応し、
/// `res_x`/`res_y` には弾性解析の結果を渡すこと。
pub fn story_eccentricity_from_analysis(
    model: &Model,
    story: StoryId,
    res_x: &StaticOnce,
    res_y: &StaticOnce,
    long_term: Option<&StaticOnce>,
) -> Eccentricity {
    let mut cols = column_stiffnesses_from_analysis(model, story, res_x, res_y);
    append_misc_wall_stiffnesses(model, story, &mut cols);
    let cor = center_of_rigidity(&cols);
    let com = long_term
        .and_then(|lt| center_of_gravity_from_axial(model, story, lt))
        .unwrap_or_else(|| center_of_mass(model, story));
    eccentricity(&cols, com, cor)
}

// ===== 雑壁の剛性評価（n 倍法。マニュアル「(7) 雑壁の剛性評価」）=====

/// 雑壁 1 枚の等価水平剛性 `Kw' = n·Aw'·ΣKc/ΣAc`。
///
/// - `n`: 雑壁の剛性を柱の剛性から求める場合の係数（入力値）
/// - `aw`: 雑壁の断面積 Aw' [mm²]
/// - `sum_kc`: 当該階の柱の剛性の和 ΣKc
/// - `sum_ac`: 当該階の柱の断面積の和 ΣAc [mm²]（0 の場合は Kw' = 0）
pub fn misc_wall_stiffness(n: f64, aw: f64, sum_kc: f64, sum_ac: f64) -> f64 {
    if sum_ac <= 0.0 {
        return 0.0;
    }
    n * aw * sum_kc / sum_ac
}

/// 当該層の柱の断面積の和 ΣAc [mm²]。
pub fn sum_column_area(model: &Model, story: StoryId) -> f64 {
    let mut sum = 0.0;
    for_each_story_column(model, story, |elem, _top, _bot| {
        if let Some(sid) = elem.section {
            sum += model.sections[sid.index()].area;
        }
    });
    sum
}

/// 当該層に帰属するフレーム外雑壁を n 倍法で等価剛性要素へ換算し、`cols` に
/// 追加する（剛心・ねじり剛性への寄与。マニュアル「(7) 雑壁の剛性評価」）。
///
/// - n 係数は `Model::stress_cfg.misc_wall_n`（`None` なら雑壁剛性を考慮しない）
/// - 帰属層: 壁の中間高さ z が（直下層 elevation, 当該層 elevation] に入る壁
/// - `Aw' = 壁の平面長さ × 壁厚`（`MiscWall::thickness` 未設定の壁は対象外）
/// - 方向別に `Kw'x = n·Aw'·ΣKc,x/ΣAc`, `Kw'y = n·Aw'·ΣKc,y/ΣAc` を求め、
///   壁面内方向の方向余弦 (cx, cy) で `dx = Kw'x·cx²`, `dy = Kw'y·cy²` として
///   壁の平面中点に置く。ΣAc = 0 の場合は Kw' = 0（マニュアル但し書き）。
pub fn append_misc_wall_stiffnesses(
    model: &Model,
    story: StoryId,
    cols: &mut Vec<ColumnStiffness>,
) {
    let Some(n) = model.stress_cfg.misc_wall_n else {
        return;
    };
    if model.misc_walls.is_empty() {
        return;
    }
    let sum_ac = sum_column_area(model, story);
    if sum_ac <= 0.0 {
        return; // ΣAc = 0 → ΣKw' = 0
    }
    let sum_kx: f64 = cols.iter().map(|c| c.dx).sum();
    let sum_ky: f64 = cols.iter().map(|c| c.dy).sum();

    let idx = story.index();
    let Some(elev) = model.stories.get(idx).map(|s| s.elevation) else {
        return;
    };
    let below = if idx == 0 {
        f64::NEG_INFINITY
    } else {
        model.stories[idx - 1].elevation
    };

    for w in &model.misc_walls {
        let Some(t) = w.thickness else {
            continue;
        };
        let z_mid = w.start[2] + w.height * 0.5;
        if !(z_mid > below + 1e-9 && z_mid <= elev + 1e-9) {
            continue;
        }
        let dxw = w.end[0] - w.start[0];
        let dyw = w.end[1] - w.start[1];
        let len = (dxw * dxw + dyw * dyw).sqrt();
        if len <= 0.0 || t <= 0.0 {
            continue;
        }
        let aw = len * t;
        let (cx, cy) = (dxw / len, dyw / len);
        cols.push(ColumnStiffness {
            pos: [(w.start[0] + w.end[0]) * 0.5, (w.start[1] + w.end[1]) * 0.5],
            dx: misc_wall_stiffness(n, aw, sum_kx, sum_ac) * cx * cx,
            dy: misc_wall_stiffness(n, aw, sum_ky, sum_ac) * cy * cy,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- d_value ----
    #[test]
    fn test_d_value_rigid_beams_general() {
        // 梁が十分剛（ΣKb 大）→ k̄ 大 → a → 1 → D → Kc0
        let e = 1.0;
        let ic = 1.0;
        let h = 1.0;
        let kc0 = 12.0 * e * ic / (h * h * h);
        let d = d_value(e, ic, h, 1e9, false);
        assert!((d - kc0).abs() / kc0 < 1e-6, "a→1 で D→Kc0, got {d}");
    }

    #[test]
    fn test_d_value_known_kbar() {
        // kc = Ic/h = 1, ΣKb = 4 → k̄ = 4/(2·1) = 2 → a = 2/(2+2) = 0.5
        // Kc0 = 12 → D = 0.5·12 = 6
        let d = d_value(1.0, 1.0, 1.0, 4.0, false);
        assert!((d - 6.0).abs() < 1e-9, "got {d}");
    }

    #[test]
    fn test_d_value_first_story() {
        // 最下階: k̄ = 2 → a = (0.5+2)/(2+2) = 0.625 → D = 0.625·12 = 7.5
        let d = d_value(1.0, 1.0, 1.0, 4.0, true);
        assert!((d - 7.5).abs() < 1e-9, "got {d}");
    }

    #[test]
    fn test_d_value_degenerate() {
        assert_eq!(d_value(1.0, 0.0, 1.0, 4.0, false), 0.0);
        assert_eq!(d_value(1.0, 1.0, 0.0, 4.0, false), 0.0);
    }

    // ---- center_of_rigidity（DoD §8.1）----
    #[test]
    fn test_center_of_rigidity_dod_example() {
        // 仕様 §5.2 の確定値: Dy=[100,300] @ x=[0,6000] → Xs = 4500
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 1.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 1.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs got {}", cr[0]);
    }

    #[test]
    fn test_eccentricity_dod_example() {
        // 上の剛心に重心 Xg=3000 → ex = 1500（DoD §8.1）
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 1.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 1.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        let ecc = eccentricity(&cols, [3000.0, 0.0], cr);
        assert!((ecc.ex - 1500.0).abs() < 1e-9, "ex got {}", ecc.ex);
    }

    #[test]
    fn test_eccentricity_symmetric_zero() {
        // 対称 4 本柱 → 剛心＝重心＝中央 → 偏心率 0
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [0.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        assert!((cr[0] - 3000.0).abs() < 1e-9);
        assert!((cr[1] - 3000.0).abs() < 1e-9);
        let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
        assert!(ecc.re_x.abs() < 1e-9 && ecc.re_y.abs() < 1e-9);
    }

    #[test]
    fn test_eccentricity_hand_calc() {
        // 手計算照合（X 加力時偏心率）。
        // 柱4本、すべて Dx=Dy=100 とし x=[0,0,6000,6000], y=[0,6000,0,6000]…ではなく
        // 剛心をずらすため右側を強くする: Dy=[100,100,300,300] @ x=[0,0,6000,6000]
        let cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [0.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 100.0,
                dy: 300.0,
            },
            ColumnStiffness {
                pos: [6000.0, 6000.0],
                dx: 100.0,
                dy: 300.0,
            },
        ];
        let cr = center_of_rigidity(&cols);
        // Xs = (100·0+100·0+300·6000+300·6000)/(100+100+300+300) = 3,600,000/800 = 4500
        assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs {}", cr[0]);
        // Ys = (Σ Dx·y)/ΣDx = 100·(0+6000+0+6000)/400 = 3000
        assert!((cr[1] - 3000.0).abs() < 1e-9, "Ys {}", cr[1]);

        // 重心は幾何中央 (3000, 3000) とする → ex = 1500, ey = 0
        let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
        assert!((ecc.ex - 1500.0).abs() < 1e-9);
        assert!(ecc.ey.abs() < 1e-9);

        // KR = Σ Dx·ȳ² + Σ Dy·x̄²
        //   x̄ = x-4500 = [-4500,-4500,1500,1500], ȳ = y-3000 = [-3000,3000,-3000,3000]
        //   Σ Dx·ȳ² = 100·(3000²·4) = 100·4·9e6 = 3.6e9
        //   Σ Dy·x̄² = 100·4500² + 100·4500² + 300·1500² + 300·1500²
        //           = 2·100·2.025e7 + 2·300·2.25e6 = 4.05e9 + 1.35e9 = 5.4e9
        //   KR = 3.6e9 + 5.4e9 = 9.0e9
        assert!((ecc.kr - 9.0e9).abs() / 9.0e9 < 1e-12, "KR {}", ecc.kr);
        // ΣDx = 400 → rex = √(9.0e9/400) = √2.25e7 = 4743.416...
        let rex = (9.0e9_f64 / 400.0).sqrt();
        assert!((ecc.rex - rex).abs() < 1e-6);
        // Rex = ey/rex = 0（ey=0）, Rey = ex/rey
        assert!(ecc.re_x.abs() < 1e-12);
        let sum_dy = 800.0;
        let rey = (9.0e9_f64 / sum_dy).sqrt();
        assert!((ecc.re_y - 1500.0 / rey).abs() < 1e-9, "Rey {}", ecc.re_y);
    }

    // ===== モデル自動算定テスト =====

    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone, Section,
        Story,
    };

    /// 対称4柱・田の字梁モデルを構築するヘルパー。
    /// 柱: 底 z=0（拘束）・頂 z=3000、story=Some(S0)。
    /// 梁: 上端節点間を X 方向・Y 方向に接続（同一 section）。
    /// 質量: 上端4節点に等質量（mass[0]=1.0）。
    /// section_iz_override: 右側 2 本（x=6000）の柱の iz を指定値に差し替え（None なら全同一）。
    fn build_symmetric_frame(
        section_iz_override: Option<f64>,
    ) -> (squid_n_core::model::Model, StoryId) {
        // 断面（共通: iy=iz=1.0e6）
        let sec_base = Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 100.0,
            iy: 1.0e6,
            iz: 1.0e6,
            j: 1.0e6,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        // 右側柱用 section（iz を上書き）
        let sec_right = Section {
            id: SectionId(1),
            name: "col_right".to_string(),
            iz: section_iz_override.unwrap_or(1.0e6),
            ..sec_base.clone()
        };
        // 梁用 section: iz を非常に大きくして全柱で a ≈ 1（kbar→∞）にする。
        // これにより D ≈ Kc0 = 12EI/h³ ∝ iz となり「Dy 比 = iz 比」が精度良く成立。
        let sec_beam = Section {
            id: SectionId(2),
            name: "beam".to_string(),
            area: 100.0,
            iy: 1.0e12,
            iz: 1.0e12,
            j: 1.0e12,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };

        // 材料（共通）
        let mat = Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 2.05e5,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };

        // 層
        let s0 = StoryId(0);
        let story = Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: s0,
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: None,
        };

        // 節点配置:
        // NodeId 0-3: 底部（z=0、story=None、拘束）
        // NodeId 4-7: 上部（z=3000、story=Some(S0)、質量有り）
        // 平面位置: 0→(0,0), 1→(6000,0), 2→(0,6000), 3→(6000,6000)
        let restraint_fixed = Dof6Mask::FIXED;
        let restraint_free = Dof6Mask::FREE;
        let mass_val: Option<[f64; 6]> = Some([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
        let xy = [
            [0.0_f64, 0.0],
            [6000.0, 0.0],
            [0.0, 6000.0],
            [6000.0, 6000.0],
        ];
        let mut nodes: Vec<Node> = Vec::new();
        for (i, &[x, y]) in xy.iter().enumerate() {
            nodes.push(Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: restraint_fixed,
                mass: None,
                story: None,
            });
        }
        for (i, &[x, y]) in xy.iter().enumerate() {
            nodes.push(Node {
                id: NodeId((i + 4) as u32),
                coord: [x, y, 3000.0],
                restraint: restraint_free,
                mass: mass_val,
                story: Some(s0),
            });
        }

        // 部材構築ヘルパー
        // 柱の ref_vector = [0,1,0] にすると:
        //   ex=[0,0,1], ey=[0,1,0](Y軸), ez=[1,0,0](X軸)
        //   I_globalX = iz·ey[0]² + iy·ez[0]² = iz·0 + iy·1 = iy → Dx ∝ iy
        //   I_globalY = iz·ey[1]² + iy·ez[1]² = iz·1 + iy·0 = iz → Dy ∝ iz（★意図）
        // これにより「右側柱の iz を 3 倍 → Dy が 3 倍 → 剛心 Xs = 4500」が成立。
        let col_local_axis = LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        };
        let beam_local_axis = LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        };
        let end_fixed = [EndCondition::Fixed, EndCondition::Fixed];

        // 柱: bottom i → top i+4
        // 左側 (x=0): SectionId(0)、右側 (x=6000): SectionId(0 or 1)
        let col_sec = |i: usize| -> SectionId {
            if section_iz_override.is_some() && (xy[i][0] - 6000.0).abs() < 1.0 {
                SectionId(1)
            } else {
                SectionId(0)
            }
        };
        let mut elements: Vec<ElementData> = Vec::new();
        for i in 0..4_usize {
            elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: {
                    let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                    v.push(NodeId(i as u32));
                    v.push(NodeId((i + 4) as u32));
                    v
                },
                section: Some(col_sec(i)),
                material: Some(MaterialId(0)),
                local_axis: col_local_axis,
                end_cond: end_fixed,
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
                spring: None,
            });
        }

        // 梁: X方向（同 y、異なる x）: top0-top1, top2-top3
        // 梁: Y方向（同 x、異なる y）: top0-top2, top1-top3
        // ElemId 4..7
        let beam_pairs: [(usize, usize); 4] = [(4, 5), (6, 7), (4, 6), (5, 7)];
        for (bi, &(na, nb)) in beam_pairs.iter().enumerate() {
            elements.push(ElementData {
                id: ElemId((4 + bi) as u32),
                kind: ElementKind::Beam,
                nodes: {
                    let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                    v.push(NodeId(na as u32));
                    v.push(NodeId(nb as u32));
                    v
                },
                section: Some(SectionId(2)),
                material: Some(MaterialId(0)),
                local_axis: beam_local_axis,
                end_cond: end_fixed,
                force_regime: ForceRegime::Auto,
                rigid_zone: RigidZone::default(),
                plastic_zone: None,
                spring: None,
            });
        }

        let sections = if section_iz_override.is_some() {
            vec![sec_base, sec_right, sec_beam]
        } else {
            vec![
                sec_base,
                Section {
                    id: SectionId(1),
                    ..sec_right
                },
                sec_beam,
            ]
        };

        let model = squid_n_core::model::Model {
            nodes,
            elements,
            sections,
            materials: vec![mat],
            stories: vec![story],
            ..Default::default()
        };
        (model, s0)
    }

    /// テスト1: 対称フレーム → 偏心率 ≈ 0、剛心 ≈ [3000, 3000]。
    #[test]
    fn test_story_eccentricity_symmetric_zero() {
        let (model, s0) = build_symmetric_frame(None);
        let ecc = story_eccentricity(&model, s0);
        assert!(ecc.re_x.abs() < 1e-6, "re_x={} (should be ~0)", ecc.re_x);
        assert!(ecc.re_y.abs() < 1e-6, "re_y={} (should be ~0)", ecc.re_y);
        // 剛心確認
        let sc = story_centers(&model, s0);
        assert!(
            (sc.center_of_rigidity[0] - 3000.0).abs() < 1.0,
            "Xs={}",
            sc.center_of_rigidity[0]
        );
        assert!(
            (sc.center_of_rigidity[1] - 3000.0).abs() < 1.0,
            "Ys={}",
            sc.center_of_rigidity[1]
        );
    }

    /// テスト2: 右側柱 iz を 3 倍 → 剛心 x ≈ 4500、偏心距離 ex ≈ 1500。
    /// 軸整合フレーム（柱軸=Z）では I_globalY = iz なので Dy ∝ iz。
    /// 梁は全柱で対称なので a 補正は全柱一致 → Dy 比 = iz 比。
    /// Xs = (1·0 + 1·0 + 3·6000 + 3·6000)/(1+1+3+3) = 4500。重心 = 3000 → ex=1500。
    #[test]
    fn test_story_eccentricity_biased_rigidity() {
        let (model, s0) = build_symmetric_frame(Some(3.0e6));
        let sc = story_centers(&model, s0);
        let xs = sc.center_of_rigidity[0];
        assert!((xs - 4500.0).abs() < 1.0, "Xs={} (expected ≈4500)", xs);
        let ecc = story_eccentricity(&model, s0);
        // 重心 x = 3000（等質量 4 点の中央）→ ex = |3000 - 4500| = 1500
        assert!(
            (ecc.ex - 1500.0).abs() < 1.0,
            "ex={} (expected ≈1500)",
            ecc.ex
        );
    }

    /// テスト3: 柱が無い層（story=S1 が存在するが柱の上端は S0）→ 空 Vec、剛心 [0,0]。
    #[test]
    fn test_story_eccentricity_empty_story() {
        let (model, _s0) = build_symmetric_frame(None);
        // S1 は存在しない（stories は S0 のみ）→ column_stiffnesses は空を返す。
        let s1 = StoryId(1);
        let cols = column_stiffnesses(&model, s1);
        assert!(cols.is_empty(), "S1 に柱が無いはず、got {} 本", cols.len());
        let cor = center_of_rigidity(&cols);
        assert_eq!(cor, [0.0, 0.0], "空時の剛心は [0,0]");
    }

    // ===== 精算ルート（応力解析結果ベース）のテスト =====

    use squid_n_element::beam::MemberForces;

    /// 柱 4 本（ElemId 0..4、節点 i(底)→i+4(頂)）へ、指定の局所内力
    /// [n, qy, qz] を持つ member_forces と頂部一様変位 disp を合成する。
    /// 柱の ref_vector=[0,1,0] より ex=[0,0,1], ey=[0,1,0], ez=[1,0,0]。
    /// → 全体 X 方向せん断 = qz、全体 Y 方向せん断 = qy。
    fn fabricate_static(top_disp: [f64; 2], col_local_forces: [[f64; 3]; 4]) -> StaticOnce {
        let mut disp = vec![[0.0; 6]; 8];
        for d in disp.iter_mut().skip(4) {
            d[0] = top_disp[0];
            d[1] = top_disp[1];
        }
        let member_forces = col_local_forces
            .iter()
            .enumerate()
            .map(|(i, &[n, qy, qz])| {
                (
                    ElemId(i as u32),
                    MemberForces {
                        at: vec![(0.0, [n, qy, qz, 0.0, 0.0, 0.0])],
                    },
                )
            })
            .collect();
        StaticOnce {
            disp,
            member_forces,
        }
    }

    /// ki = Qi/δi の精算: X 加力（δ=10, Qx=qz=1000 → kX=100）、
    /// Y 加力（δ=10, 左 qy=1000/右 qy=3000 → kY=100/300）→ 剛心 Xs=4500, Ys=3000。
    #[test]
    fn test_column_stiffnesses_from_analysis() {
        let (model, s0) = build_symmetric_frame(None);
        // xy = [(0,0), (6000,0), (0,6000), (6000,6000)]
        let res_x = fabricate_static([10.0, 0.0], [[0.0, 0.0, 1000.0]; 4]);
        let res_y = fabricate_static(
            [0.0, 10.0],
            [
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
            ],
        );
        let cols = column_stiffnesses_from_analysis(&model, s0, &res_x, &res_y);
        assert_eq!(cols.len(), 4);
        for c in &cols {
            assert!((c.dx - 100.0).abs() < 1e-9, "kX={}", c.dx);
        }
        let cor = center_of_rigidity(&cols);
        assert!((cor[0] - 4500.0).abs() < 1e-9, "Xs={}", cor[0]);
        assert!((cor[1] - 3000.0).abs() < 1e-9, "Ys={}", cor[1]);
    }

    /// 長期軸力による重心: 全柱等圧縮 → 幾何中央 (3000, 3000)。
    /// 右側 2 本を 3 倍圧縮 → gx = 4500。引張柱は重み 0。
    #[test]
    fn test_center_of_gravity_from_axial() {
        let (model, s0) = build_symmetric_frame(None);
        // 圧縮は n 負（引張正の符号規約）
        let uniform = fabricate_static([0.0, 0.0], [[-200.0, 0.0, 0.0]; 4]);
        let g = center_of_gravity_from_axial(&model, s0, &uniform).unwrap();
        assert!((g[0] - 3000.0).abs() < 1e-9 && (g[1] - 3000.0).abs() < 1e-9);

        let biased = fabricate_static(
            [0.0, 0.0],
            [
                [-100.0, 0.0, 0.0],
                [-300.0, 0.0, 0.0],
                [-100.0, 0.0, 0.0],
                [-300.0, 0.0, 0.0],
            ],
        );
        let g = center_of_gravity_from_axial(&model, s0, &biased).unwrap();
        assert!((g[0] - 4500.0).abs() < 1e-9, "gx={}", g[0]);

        // 全柱引張 → None（質量重心へフォールバックさせる）
        let tension = fabricate_static([0.0, 0.0], [[200.0, 0.0, 0.0]; 4]);
        assert!(center_of_gravity_from_axial(&model, s0, &tension).is_none());
    }

    /// 精算ルートの統合: 剛心 Xs=4500・重心 gx=3000 → ex=1500, Rey=ex/rey。
    #[test]
    fn test_story_eccentricity_from_analysis() {
        let (model, s0) = build_symmetric_frame(None);
        let res_x = fabricate_static([10.0, 0.0], [[0.0, 0.0, 1000.0]; 4]);
        let res_y = fabricate_static(
            [0.0, 10.0],
            [
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
            ],
        );
        let long_term = fabricate_static([0.0, 0.0], [[-200.0, 0.0, 0.0]; 4]);
        let ecc = story_eccentricity_from_analysis(&model, s0, &res_x, &res_y, Some(&long_term));
        assert!((ecc.ex - 1500.0).abs() < 1e-9, "ex={}", ecc.ex);
        assert!(ecc.ey.abs() < 1e-9, "ey={}", ecc.ey);
        // KR = ΣkX·ȳ² + ΣkY·x̄²
        //   ȳ = ±3000（kX=100×4）→ 100·9e6·4 = 3.6e9
        //   x̄ = x−4500 = [-4500,1500,-4500,1500] → 100·4500²+300·1500²（×2組）
        //     = 2·(100·2.025e7 + 300·2.25e6) = 2·(2.025e9+0.675e9) = 5.4e9
        let kr_expect = 3.6e9 + 5.4e9;
        assert!(
            (ecc.kr - kr_expect).abs() / kr_expect < 1e-12,
            "KR={}",
            ecc.kr
        );
        let rey = (kr_expect / 800.0_f64).sqrt();
        assert!((ecc.re_y - 1500.0 / rey).abs() < 1e-9, "Rey={}", ecc.re_y);
    }

    // ===== 雑壁の n 倍法 =====

    #[test]
    fn test_misc_wall_stiffness() {
        // Kw' = n·Aw'·ΣKc/ΣAc = 2·1000·400/400 = 2000
        assert!((misc_wall_stiffness(2.0, 1000.0, 400.0, 400.0) - 2000.0).abs() < 1e-12);
        // ΣAc = 0 → Kw' = 0（マニュアル但し書き）
        assert_eq!(misc_wall_stiffness(2.0, 1000.0, 400.0, 0.0), 0.0);
    }

    #[test]
    fn test_sum_column_area() {
        let (model, s0) = build_symmetric_frame(None);
        // 柱 4 本 × area 100
        assert!((sum_column_area(&model, s0) - 400.0).abs() < 1e-12);
    }

    #[test]
    fn test_append_misc_wall_stiffnesses() {
        use squid_n_core::model::{MiscWall, MiscWallTransfer};
        let (mut model, s0) = build_symmetric_frame(None);
        model.stress_cfg.misc_wall_n = Some(2.0);
        // Y 方向の壁 @ x=6000（長さ 6000 × 厚 100 → Aw' = 6e5）、z_mid=1500 → S0 帰属。
        model.misc_walls.push(MiscWall {
            start: [6000.0, 0.0, 0.0],
            end: [6000.0, 6000.0, 0.0],
            height: 3000.0,
            weight_per_area: 1.0e-3,
            transfer: MiscWallTransfer::SelfStanding,
            thickness: Some(100.0),
        });
        // 帯域外の壁（z_mid = 4500 > elevation 3000）→ 無視される。
        model.misc_walls.push(MiscWall {
            start: [0.0, 0.0, 3000.0],
            end: [0.0, 6000.0, 3000.0],
            height: 3000.0,
            weight_per_area: 1.0e-3,
            transfer: MiscWallTransfer::SelfStanding,
            thickness: Some(100.0),
        });
        // 厚さ未設定の壁 → 無視される。
        model.misc_walls.push(MiscWall {
            start: [0.0, 0.0, 0.0],
            end: [0.0, 6000.0, 0.0],
            height: 3000.0,
            weight_per_area: 1.0e-3,
            transfer: MiscWallTransfer::SelfStanding,
            thickness: None,
        });

        let mut cols = vec![
            ColumnStiffness {
                pos: [0.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 0.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [0.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
            ColumnStiffness {
                pos: [6000.0, 6000.0],
                dx: 100.0,
                dy: 100.0,
            },
        ];
        append_misc_wall_stiffnesses(&model, s0, &mut cols);
        assert_eq!(cols.len(), 5, "帯域内かつ厚さ有りの壁 1 枚のみ追加");
        let wall = cols[4];
        // Kw'y = n·Aw'·ΣKy/ΣAc = 2·6e5·400/400 = 1.2e6（cy=1 なので dy へ全量）
        assert!((wall.dy - 1.2e6).abs() < 1e-6, "Kw'y={}", wall.dy);
        assert!(wall.dx.abs() < 1e-12, "cx=0 なので dx は 0");
        assert_eq!(wall.pos, [6000.0, 3000.0]);

        // 剛心が壁側（x=6000）へ寄ることの確認。
        let cor = center_of_rigidity(&cols);
        assert!(cor[0] > 3000.0, "Xs={} は壁側へ寄る", cor[0]);

        // n 未指定なら追加されない。
        let mut model2 = model.clone();
        model2.stress_cfg.misc_wall_n = None;
        let mut cols2 = cols[..4].to_vec();
        append_misc_wall_stiffnesses(&model2, s0, &mut cols2);
        assert_eq!(cols2.len(), 4);
    }
}
