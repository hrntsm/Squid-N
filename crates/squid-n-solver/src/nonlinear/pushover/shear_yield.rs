//! せん断降伏耐力 Qy の算定と降伏イベントの追跡（段階的耐力喪失解析用）。
//!
//! - [`ShearThreshold`] / [`DirThreshold`] / [`ShearDir`] — 方向別 Qy しきい値
//! - [`compute_shear_yield_thresholds`] — 全部材のしきい値を組み立て
//! - [`effective_clear_span`] — 剛域控除後の内法スパン h0
//! - [`track_shear_yield`] — 各ステップのせん断降伏を軸力 σ0 を反映して判定

use super::geom::{axial_compression, dot3};
use super::types::ShearYieldEvent;
use squid_n_core::model::{ElementData, Material, Model, RigidZone, Section};
use squid_n_core::rc_capacity::{rc_qsu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};
use squid_n_element::transform::LocalFrame;

/// せん断降伏耐力 Qy の判定しきい値（部材ごと、局所 y・z 方向、独立）。
///
/// 要素座標系はせい方向＝ローカル y（`LocalFrame`: ey=ref_vector 直交化）のため、
/// `y` は局所 y 方向せん断力 Vy（**強軸**曲げ＝Mz 面に伴う。断面レイヤでは
/// `Section.as_z`＝ウェブ）、`z` は局所 z 方向せん断力 Vz（**弱軸**曲げ＝My 面。
/// `Section.as_y`＝フランジ）に対するしきい値であり、`track_shear_yield` で
/// Vy vs `y.qy(..)`・Vz vs `z.qy(..)` を独立に判定する（v1 のような
/// 「合力 vs min(qy_y,qy_z)」の丸めは行わない）。断面→要素座標系のクロス変換は
/// `beam/construct.rs` と同一規約。
/// RC矩形（[`DirThreshold::RcArakawa`]）方向は、各ステップの部材軸力（圧縮）
/// から動的に σ0 を反映した Qy を都度算定する（精緻化2、`track_shear_yield` 参照）。
pub(crate) struct ShearThreshold {
    pub(crate) y: DirThreshold,
    pub(crate) z: DirThreshold,
}

/// せん断降伏耐力 Qy の算定方式（方向別）。
///
/// `Static` は解析開始時に一度だけ算定される軸力非依存のしきい値（鋼系、または
/// 配筋情報が無い／算定不能な RC のフォールバック）。`RcArakawa` は RC矩形
/// （`SectionShape::RcRect`）の荒川mean式系の略算式で、σ0 を除く入力一式を
/// 保持しておき、各ステップの軸力から求めた σ0 で上書きして
/// [`rc_qsu_simple`] を呼び直す（精緻化2）。
pub(crate) enum DirThreshold {
    Static(f64),
    RcArakawa {
        /// σ0 抜きの入力一式（`sigma_0` は常に 0.0 のプレースホルダ。
        /// [`DirThreshold::qy`] が呼び出しのたびに軸力由来の値へ差し替える）。
        input: RcCapacityInput,
        /// 全断面積 [mm²]（= b・D。方向によらず同一値。σ0 = 圧縮軸力/gross_area
        /// の算定に用いる）。
        gross_area: f64,
    },
}

impl DirThreshold {
    /// 圧縮軸力 `n_compress`（[N]、0 以上。引張は呼び出し側で 0 として渡す
    /// 規約、`axial_compression` 参照）から Qy [N] を求める。
    ///
    /// `Static` は軸力によらず一定値。`RcArakawa` は σ0 = n_compress/gross_area
    /// （荒川式の適用範囲 0〜0.4Fc へのクランプは [`rc_qsu_simple`] 内で行う）を
    /// 反映した Qsu を都度算定する。
    pub(crate) fn qy(&self, n_compress: f64) -> f64 {
        match self {
            DirThreshold::Static(v) => *v,
            DirThreshold::RcArakawa { input, gross_area } => {
                let sigma_0 = if *gross_area > 0.0 {
                    n_compress / gross_area
                } else {
                    0.0
                };
                let mut inp = *input;
                inp.sigma_0 = sigma_0;
                rc_qsu_simple(&inp)
            }
        }
    }
}

/// せん断降伏耐力 Qy 算定対象の方向（局所座標系。せい方向＝ローカル y）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ShearDir {
    /// 局所 y 方向（強軸曲げ＝Mz 面に伴うせん断、`Section.as_z`・`RcRebar.main_x` 対応）。
    Y,
    /// 局所 z 方向（弱軸曲げ＝My 面に伴うせん断、`Section.as_y`・`RcRebar.main_y` 対応）。
    Z,
}

/// `SectionShape::RcRect` の配筋情報から、指定方向の荒川mean式系の略算式
/// （[`squid_n_core::rc_capacity::rc_qsu_simple`]）用入力一式を組み立てる。
/// σ0 は 0.0 のプレースホルダとし、[`DirThreshold::qy`] が各ステップの軸力から
/// 動的に上書きする（精緻化2。旧実装は σ0=0 固定の安全側簡略化だった）。
///
/// 変換規則は `squid-n-app::app::rc_capacity_input_from_rect` と同一の規約
/// （上下対称配筋を仮定・at=引張側総断面積の半分、σy=fy or 345、σwy=295 固定、
/// せん断補強筋は legs 組数を考慮）に合わせる:
/// - 強軸（局所 y 方向せん断、`dir=Y`）: b=幅, d=せい、引張鉄筋は `rebar.main_x`。
/// - 弱軸（局所 z 方向せん断、`dir=Z`）: b と d を入れ替え、引張鉄筋は `rebar.main_y`。
///
/// `clear_span`（h0）は [`effective_clear_span`] が剛域長を控除して算定した値を
/// 渡す（精緻化1。旧実装は剛域控除を省略し節点間長をそのまま用いる簡略化だった）。
/// `fc` 未設定の場合は None（呼び出し側で慣用値へフォールバックする）。
fn rc_rect_capacity_input(
    b: f64,
    d: f64,
    main: &BarSet,
    rebar: &RcRebar,
    mat: &Material,
    clear_span: f64,
) -> Option<RcCapacityInput> {
    let fc = mat.fc?;
    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    // 上下対称配筋を仮定し、引張側主筋量は総断面積の半分。
    let at = bar_area(main) / 2.0;
    let d_eff = d - rebar.cover - main.dia / 2.0;
    let shear_area =
        std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia * rebar.shear.legs as f64;
    let pw = if rebar.shear.pitch > 0.0 {
        shear_area / (b * rebar.shear.pitch)
    } else {
        0.0
    };
    Some(RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y: mat.fy.unwrap_or(345.0), // SD345 相当、要・原典照合
        fc,
        pw,
        sigma_wy: 295.0, // SD295 相当、要・原典照合
        clear_span,
        sigma_0: 0.0, // プレースホルダ。DirThreshold::qy が軸力から都度上書きする。
    })
}

/// 方向別のせん断降伏耐力しきい値（[`DirThreshold`]）を組み立てる。
///
/// RC矩形（`SectionShape::RcRect`）で `fy` が無く、配筋情報から Qsu(σ0=0) が
/// 算定可能（正の値）な場合のみ [`DirThreshold::RcArakawa`] を採用し、各ステップ
/// で軸力から動的算定した σ0 を反映する。それ以外（鋼系・配筋情報が無い／
/// 算定不能な RC・有効せん断断面積や材料情報が無い場合）は、解析開始時に一度だけ
/// 算定した [`DirThreshold::Static`] を用いる（採用式は下記）:
/// - 鋼系部材（材料に `fy` が設定されている）: Qy = as・fy / √3
///   （純せん断降伏条件 τy = fy/√3（von Mises）に有効せん断断面積を乗じた慣用式）。
/// - RC 系部材で `RcRect` 形状が無い、または Qsu 算定不能な場合: Qy = as・0.7√fc
///   （コンクリートのせん断終局強度に対する簡易慣用値。荒川式等の精算は行わない）。
/// - 有効せん断断面積 `as_area` が 0（未設定）、または材料・強度情報が無い場合は
///   判定対象外として Qy = +∞（その方向のせん断では耐力喪失を判定しない）。
fn build_dir_threshold(
    as_area: f64,
    material: Option<&Material>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> DirThreshold {
    if as_area <= 0.0 {
        return DirThreshold::Static(f64::INFINITY);
    }
    let Some(mat) = material else {
        return DirThreshold::Static(f64::INFINITY);
    };
    if let Some(fy) = mat.fy {
        return DirThreshold::Static(as_area * fy / 3.0_f64.sqrt());
    }
    let Some(fc) = mat.fc else {
        return DirThreshold::Static(f64::INFINITY);
    };
    if let Some(Section {
        shape: Some(SectionShape::RcRect { b, d, rebar }),
        ..
    }) = section
    {
        let input = match dir {
            ShearDir::Y => rc_rect_capacity_input(*b, *d, &rebar.main_x, rebar, mat, clear_span),
            ShearDir::Z => rc_rect_capacity_input(*d, *b, &rebar.main_y, rebar, mat, clear_span),
        };
        if let Some(input) = input {
            if rc_qsu_simple(&input) > 0.0 {
                return DirThreshold::RcArakawa {
                    gross_area: input.b * input.d,
                    input,
                };
            }
        }
    }
    DirThreshold::Static(as_area * 0.7 * fc.sqrt())
}

/// せん断降伏耐力 Qy [N] を算定する（段階的耐力喪失解析の
/// せん断降伏判定に使用）。
///
/// 軸力なし（σ0=0）の静的評価。単体テスト・後方互換用の薄いラッパーで、
/// [`build_dir_threshold`] が返す [`DirThreshold`] を `n_compress=0` で評価する
/// ことと等価（実解析 `track_shear_yield` は各ステップの軸力から動的に σ0 を
/// 反映するため、本関数は呼ばない。テスト専用のため `#[cfg(test)]`）。
#[cfg(test)]
pub(crate) fn compute_shear_yield_qy(
    as_area: f64,
    material: Option<&Material>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> f64 {
    build_dir_threshold(as_area, material, section, dir, clear_span).qy(0.0)
}

/// 部材長（節点間距離）[mm]。節点参照が欠落・退化（長さ0）の場合は None。
/// RC のせん断降伏耐力算定における内法スパン h0 は、この節点間長から
/// [`effective_clear_span`] が剛域長を控除して求める（精緻化1）。
fn elem_length(model: &Model, elem: &ElementData) -> Option<f64> {
    if elem.nodes.len() < 2 {
        return None;
    }
    let pi = model.nodes.get(elem.nodes[0].index())?;
    let pj = model.nodes.get(elem.nodes[1].index())?;
    let dx = pj.coord[0] - pi.coord[0];
    let dy = pj.coord[1] - pi.coord[1];
    let dz = pj.coord[2] - pi.coord[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    (len > 0.0).then_some(len)
}

/// 剛域控除後の内法スパン h0 [mm]（荒川式のせん断スパン比算定に用いる、精緻化1）。
///
/// h0 = 節点間長（`raw_length`） − (`rigid_zone.length_i` + `rigid_zone.length_j`)。
/// 控除後が 0 以下（浮動小数点誤差により実質 0 とみなせる極小値を含む、
/// 1e-6mm 以下）になる異常な剛域指定（剛域長の入力誤りで節点間長を超過する等）
/// では、h0 を過小評価しないよう節点間長そのものへフォールバックする
/// （`rc_qsu_simple` 側でもせん断スパン比 h0/(2d_e) は 1.0〜3.0 にクランプされる
/// ため過大な Qsu には至らないが、異常値の握り潰しではなくフォールバックとして
/// 明示する）。
pub(crate) fn effective_clear_span(raw_length: f64, rigid_zone: &RigidZone) -> f64 {
    let net = raw_length - rigid_zone.length_i - rigid_zone.length_j;
    if net > 1e-6 {
        net
    } else {
        raw_length
    }
}

pub(crate) fn compute_shear_yield_thresholds(model: &Model) -> Vec<ShearThreshold> {
    model
        .elements
        .iter()
        .map(|elem| {
            let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
            let mat = elem
                .material
                .and_then(|mid| model.materials.get(mid.index()));
            let (as_y, as_z) = sec.map(|s| (s.as_y, s.as_z)).unwrap_or((0.0, 0.0));
            let raw_length = elem_length(model, elem).unwrap_or(0.0);
            let clear_span = effective_clear_span(raw_length, &elem.rigid_zone);
            // 断面→要素座標系のクロス変換（beam/construct.rs と同一規約）:
            // 局所 y（強軸曲げのせん断）には断面 as_z（ウェブ）、
            // 局所 z（弱軸曲げのせん断）には断面 as_y（フランジ）を用いる。
            ShearThreshold {
                y: build_dir_threshold(as_z, mat, sec, ShearDir::Y, clear_span),
                z: build_dir_threshold(as_y, mat, sec, ShearDir::Z, clear_span),
            }
        })
        .collect()
}

/// せん断降伏イベントの追跡（`track_hinges` と対をなす、曲げとは独立の判定）。
///
/// `ElementBehavior::internal_force` が返す材端節点力はグローバル座標成分
/// （`f.data[0..3]`＝i端, `f.data[6..9]`＝j端）である。要素の局所座標系
/// （`LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector)`、
/// `rot[0]=ex, rot[1]=ey, rot[2]=ez`）の `ey`・`ez` へ材端力を射影することで
/// 局所 Vy・Vz を厳密に分離し、Vy は `qy_y`、Vz は `qy_z` と独立に比較する
/// （v1 の「軸直交合力 vs min(qy_y,qy_z)」から改良）。各材端のうち大きい方を
/// 部材の代表値とし、Vy・Vz のいずれかがしきい値を超えた部材を、当該ステップの
/// せん断降伏イベントとして記録する。
///
/// ## 軸力 σ0 の動的反映（精緻化2）
/// Vy・Vz と同様に材端力を局所 `ex` へ射影し、[`axial_compression`] で部材の
/// 圧縮軸力（引張は 0、両端のうち大きい方を実勢値として採用）を求める。
/// RC矩形の [`DirThreshold::RcArakawa`] 方向は σ0 = 圧縮軸力/(b・D) として
/// [`DirThreshold::qy`] に渡し、`rc_qsu_simple` を呼び直して Qy を都度算定する。
/// 鋼系・フォールバック RC（[`DirThreshold::Static`]）はこの軸力を無視し、
/// 解析開始時の静的値をそのまま用いる。
pub(crate) fn track_shear_yield(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[ShearThreshold],
    step: u32,
    events: &mut Vec<ShearYieldEvent>,
) {
    let state = ElemState::default();
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        if elem.nodes.len() < 2 {
            continue;
        }
        let (Some(pi), Some(pj)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[1].index()),
        ) else {
            continue;
        };
        if elem_length(model, elem).is_none() {
            continue;
        }
        let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
        let ex = frame.rot[0];
        let ey = frame.rot[1];
        let ez = frame.rot[2];

        let f = b.internal_force(&state, &ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let f_j = [f.data[6], f.data[7], f.data[8]];
        let vy = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let vz = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let n_compress = axial_compression(f_i, f_j, ex);

        let th = &thresholds[i];
        let qy_y = th.y.qy(n_compress);
        let qy_z = th.z.qy(n_compress);
        if vy >= qy_y || vz >= qy_z {
            events.push(ShearYieldEvent {
                step,
                elem: elem.id,
            });
        }
    }
}
