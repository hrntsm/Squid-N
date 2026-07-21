//! スラブ協力幅・合成梁・壁エレメント上下大梁による曲げ剛性の増大率算定。
//!
//! いずれもモデルデータ（[`Model`]・[`ElementData`]）から剛性倍率（無次元）を返す
//! 純関数群で、[`super::construct`] の `BeamElement::new` から呼ばれる。

use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;

/// RC規準8条によるスラブ協力幅 bf = b + ba(左) + ba(右) [mm]。
///
/// 梁の両端節点をともに境界節点に含むスラブを「梁に取り付く床」とみなし、
/// 隣接する平行梁との**内法距離** a（軸間距離から自梁・相手梁の幅の半分ずつを
/// 控除。RC規準8条の図の a）を求めて ba=(0.5−0.6·a/l)·a（a≥l/2 のとき 0.1·l）
/// で片側協力幅を算定する。対象は水平材（勾配 5% までは水平とみなす）のみ。
/// 適用不能（スラブ厚 t≤0・非水平・取り付く床なし・bf≤b）は None。
/// 連続梁の λ・吹抜け補正・二重スラブ/片持ちスラブの区別は未対応（v1。
/// dev_docs/v_and_v/剛性計算_参照実装照合.md 参照）。
fn slab_cooperating_width(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
) -> Option<f64> {
    let t = model.slab_thickness;
    if t <= 0.0 || b <= 0.0 || data.nodes.len() < 2 || model.slabs.is_empty() {
        return None;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[data.nodes.len() - 1];
    let (Some(node0), Some(node1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
    else {
        return None;
    };
    let (p0, p1) = (node0.coord, node1.coord);
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let lp = (dx * dx + dy * dy).sqrt();
    // 水平材のみ対象（勾配 5% までは水平とみなす）
    if lp < 1e-9 || dz.abs() > 0.05 * lp {
        return None;
    }
    let l = (lp * lp + dz * dz).sqrt();
    let (ex, ey) = (dx / lp, dy / lp);
    // 平面内で梁軸に直交する符号付き距離
    let signed_dist =
        |coord: [f64; 3]| -> f64 { -(coord[0] - p0[0]) * ey + (coord[1] - p0[1]) * ex };

    // スラブ境界内で自梁と平行な向かい側の梁（距離 target_s）の幅を探す。
    // 見つからなければ自梁と同幅とみなす（同一符号の梁が並ぶ床組の慣用近似）。
    let far_beam_width = |slab: &squid_n_core::model::Slab, target_s: f64, sign: f64| -> f64 {
        const TOL_MM: f64 = 1.0;
        for e in &model.elements {
            if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            let (m0, m1) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
            if m0 == n0 && m1 == n1 || m0 == n1 && m1 == n0 {
                continue;
            }
            if !(slab.boundary.contains(&m0) && slab.boundary.contains(&m1)) {
                continue;
            }
            let (Some(q0), Some(q1)) = (model.nodes.get(m0.index()), model.nodes.get(m1.index()))
            else {
                continue;
            };
            let s0 = signed_dist(q0.coord) * sign;
            let s1 = signed_dist(q1.coord) * sign;
            if (s0 - target_s).abs() > TOL_MM || (s1 - target_s).abs() > TOL_MM {
                continue;
            }
            if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                if sec.width > 0.0 {
                    return sec.width;
                }
            }
        }
        b
    };

    // 梁軸の左右それぞれの隣接平行梁との内法距離 a（複数スラブは大きい方を採用）。
    // 軸間距離（スラブ境界節点の最大直交距離）から、自梁の幅/2 と相手梁の幅/2 を
    // 控除して内法にする（RC規準8条の a。従来は軸間距離をそのまま用いており
    // 協力幅を過大評価していた）。
    let mut a_pos: f64 = 0.0;
    let mut a_neg: f64 = 0.0;
    for slab in &model.slabs {
        if !(slab.boundary.contains(&n0) && slab.boundary.contains(&n1)) {
            continue;
        }
        let mut s_pos: f64 = 0.0;
        let mut s_neg: f64 = 0.0;
        for nid in &slab.boundary {
            let Some(q) = model.nodes.get(nid.index()) else {
                continue;
            };
            let s = signed_dist(q.coord);
            s_pos = s_pos.max(s);
            s_neg = s_neg.max(-s);
        }
        if s_pos > 0.0 {
            let far_w = far_beam_width(slab, s_pos, 1.0);
            a_pos = a_pos.max((s_pos - b / 2.0 - far_w / 2.0).max(0.0));
        }
        if s_neg > 0.0 {
            let far_w = far_beam_width(slab, s_neg, -1.0);
            a_neg = a_neg.max((s_neg - b / 2.0 - far_w / 2.0).max(0.0));
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
        return None;
    }
    Some(bf)
}

/// スラブ協力幅による強軸曲げ剛性の増大率（協力幅は RC規準 8 条
/// = [`slab_cooperating_width`] による）。
///
/// 対象は水平な RC 矩形梁のみ。スラブ（厚さ t=`Model::slab_thickness`、
/// 建物一律・上端は梁上端と同面）を考慮した中立軸による T 形断面の Ie を
/// 元断面 I0=b·D³/12 で除した値を返す。適用不能時は 1.0（増大なし）。
pub(super) fn slab_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
    d: f64,
) -> f64 {
    if d <= 0.0 {
        return 1.0;
    }
    let Some(bf) = slab_cooperating_width(model, data, b) else {
        return 1.0;
    };
    // スラブを考慮した中立軸による T 形断面の Ie
    let t = model.slab_thickness;
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

/// S 造合成梁の断面性能に用いる床スラブコンクリートの設計基準強度の仮定値
/// [N/mm²]（モデルにスラブ材料が無いための標準仮定。普通コンクリート Fc21）。
const COMPOSITE_SLAB_FC: f64 = 21.0;

/// S 造合成梁の強軸曲げ剛性の増大率。
///
/// スラブが取り付く水平な H 形鋼梁を合成梁とみなし、スラブを考慮した換算断面の
/// 剛性 I と鉄骨梁のみの剛性 sI の**平均**を採用する（各種合成構造設計指針の
/// 完全合成梁の剛性を安全側に丸めた平均法）。スラブ上端からの図心位置 g と
/// 換算断面 I（鉄骨基準）は
///
/// ```text
/// g = (cE·B·t·(t/2) + sE·sA·(t + Hd + sH/2)) / (cE·B·t + sE·sA)
/// I = (cE/sE)·(B·t³/12 + B·t·(g − t/2)²) + sI + sA·(g − t − Hd − sH/2)²
/// ```
///
/// で算定する（B=協力幅 [`slab_cooperating_width`]、t=スラブ厚、sA/sI/sH=鉄骨の
/// 断面積・断面2次モーメント・せい）。返り値は (I+sI)/(2·sI) ≥ 1。
///
/// 簡略化（適用条件とともに doc 固定）:
/// - デッキ高さ Hd は未対応（=0。スラブ下端＝鉄骨上端と仮定）
/// - スラブコンクリートは Fc21 標準仮定（`COMPOSITE_SLAB_FC`。モデルにスラブ
///   材料が無いため）
/// - 頭付きスタッド等の合成条件は判定しない（スラブが取り付けば合成とみなす）
pub(super) fn composite_beam_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    sec: &squid_n_core::model::Section,
    es: f64,
) -> f64 {
    let (sa, si, sh) = (sec.area, sec.iy, sec.depth);
    if sa <= 0.0 || si <= 0.0 || sh <= 0.0 || es <= 0.0 {
        return 1.0;
    }
    let Some(bf) = slab_cooperating_width(model, data, sec.width.max(1.0)) else {
        return 1.0;
    };
    let t = model.slab_thickness;
    let ec = squid_n_core::section_shape::concrete_young_modulus(COMPOSITE_SLAB_FC);
    let hd = 0.0; // デッキ高さ（未対応=0）
    let ca = bf * t;
    let denom = ec * ca + es * sa;
    if denom <= 0.0 {
        return 1.0;
    }
    let g = (ec * ca * (t / 2.0) + es * sa * (t + hd + sh / 2.0)) / denom;
    let i_comp = (ec / es) * (bf * t.powi(3) / 12.0 + ca * (g - t / 2.0).powi(2))
        + si
        + sa * (g - t - hd - sh / 2.0).powi(2);
    ((i_comp + si) / (2.0 * si)).max(1.0)
}

/// 壁エレメントモデルの上下大梁の剛性倍率（壁エレメント置換モデルの上下大梁の
/// 断面性能。RC規準の耐震壁規定）。
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
/// 下辺の大梁とみなす（壁エレメント置換モデルの上下大梁）。
///
/// ただし対象は耐震壁が成立した壁のみ（`misc_wall::wall_is_seismic`）。不成立の
/// フレーム内雑壁の上下梁は 100 倍せず、代わりに腰壁/垂壁として断面性能へ算入する
/// （`BeamElement::new` 内の雑壁算入処理）。
pub(super) fn is_wall_top_bottom_girder(model: &Model, n0: NodeId, n1: NodeId) -> bool {
    model.elements.iter().any(|e| {
        matches!(e.kind, squid_n_core::model::ElementKind::Wall)
            && e.nodes.len() >= 4
            && e.nodes.contains(&n0)
            && e.nodes.contains(&n1)
            && crate::misc_wall::wall_is_seismic(e, model)
    })
}
