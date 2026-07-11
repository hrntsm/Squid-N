//! PCa（プレキャスト）梁の水平接合面の検討（RESP-D マニュアル「計算編 04
//! 断面検定（許容応力度検定）」鉄筋コンクリート造水平接合面の検討）。
//!
//! 材軸平行接合部（打継ぎ面）のせん断強度が設計用せん断応力度を上回ることを
//! 確認する。使用限界状態・終局限界状態の 2 種類の検討がある。
//!
//! # 位置付け・簡略化
//! - 本モジュールは [`crate::rc::joint`] と同様の**純関数群**（[`pca_horizontal_joint_service`]・
//!   [`pca_horizontal_joint_ultimate`]・[`moment_zero_distance`]）を中心とし、
//!   [`crate::joint_wiring`] と同様に `Model`（[`squid_n_core::model::PcaBeamAttr`]）と
//!   部材内力から入力を組み立てて一括実行する配線関数 [`collect_pca_checks`] を提供する。
//!
//! # 配線関数（[`collect_pca_checks`]）の簡略化（doc 兼申し送り）
//! - 対象断面は `SectionShape::RcRect { b, d, rebar }` のみ（T形協力幅は未対応。
//!   SRC/PCa 専用形状も未対応で、登録されていてもスキップする）。
//! - 検定対象位置は「鉛直荷重時: 両端上端、地震荷重時: 上端引張となる端部のみ」が
//!   マニュアルの規定だが、内力の符号規約（引張正負の向き）が呼び出し元次第で
//!   一意に定まらないため、**両端で常に検定する保守側の簡略化**とする。
//! - 終局限界の `ΔT`（長期）は `Md/(0.9・d)` の `Md = α・MDL + β・MLL` を
//!   `α=β=1.0` として当該ケースの端部モーメントをそのまま用いる近似
//!   （荷重組合せの分離情報が呼び出し側に無いため）。
//! - 終局限界の `ΔT`（地震時想定＝`long_term=false`）は引張鉄筋の降伏耐力
//!   `at・σy` とし、強度倍率（割増係数）は未考慮。
//! - 終局限界の `Δl`（区間長さ）は [`moment_zero_distance`] により部材 1 本に
//!   つき 1 つだけ求め、両端の検定で共用する（マニュアルの「区間長さ」は
//!   端部から M=0 位置までの距離であり、対称でない分布でも近い方の解を
//!   採用する近似）。M=0 位置が求まらない場合（全長同符号）は `Δl = L/2`
//!   とする。

use crate::CheckResult;
use squid_n_core::ids::ElemId;
use squid_n_core::model::Model;
use squid_n_core::section_shape::SectionShape;

/// モーメント 2 次曲線分布（RESP-D マニュアル「採用応力」）の M=0 となる
/// 端部からの距離（近い方）[mm]。
///
/// `M(x) = M1 + (−M1 − M2 + 4・M0)・x/L − 4・M0・x²/L²`
///
/// - `m1`, `m2`: 端部モーメント（`m1` は下側引張を正、`m2` は上側引張を正）
/// - `m0`: 単純梁の中央モーメント（下側引張を正）
/// - `l`: 部材長 [mm]
///
/// 終局限界状態の検討の区間長さ Δl の算定（端部から M=0 位置まで）に用いる。
/// 実数解が (0, L) に無い場合は None（全長で同符号のモーメント分布）。
pub fn moment_zero_distance(m1: f64, m2: f64, m0: f64, l: f64) -> Option<f64> {
    if l <= 0.0 {
        return None;
    }
    // M(x) = a・x² + b・x + c、a = −4M0/L²、b = (−M1−M2+4M0)/L、c = M1
    let a = -4.0 * m0 / (l * l);
    let b = (-m1 - m2 + 4.0 * m0) / l;
    let c = m1;
    let roots: Vec<f64> = if a.abs() < 1e-12 {
        if b.abs() < 1e-12 {
            return None;
        }
        vec![-c / b]
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        vec![(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
    };
    // (0, L) 内の解のうち、いずれかの端部に最も近いもの（端部からの距離）。
    roots
        .into_iter()
        .filter(|x| *x > 0.0 && *x < l)
        .map(|x| x.min(l - x))
        .min_by(|p, q| p.partial_cmp(q).unwrap_or(std::cmp::Ordering::Equal))
}

/// PCa 水平接合面の使用限界状態の検討の入力。
pub struct PcaServiceInput {
    /// 部材断面に作用するせん断力 Q [N]。
    pub q: f64,
    /// 水平接合面より外側（断面縁側）のコンクリートの、図心位置からの
    /// 断面一次モーメント Sy [mm³]。
    pub s_y: f64,
    /// 断面二次モーメント I [mm⁴]（応力計算で用いる値と同じ）。
    pub i: f64,
    /// 接合面の幅（梁幅）b [mm]。
    pub b: f64,
    /// 水平接合面の摩擦係数 μ。
    pub mu: f64,
    /// 接合面を横切る補強筋の体積比合計 p′w（= あばら筋 pw + 補強筋 rpw）。
    pub pw_total: f64,
    /// 補強筋の降伏強度 σy [N/mm²]（あばら筋・補強筋で異なる場合は
    /// `p′w・σy = pw・σy + rpw・rσy` となるよう等価値を渡す）。
    pub sigma_y: f64,
}

/// PCa 水平接合面・使用限界状態の検討。
///
/// - 設計用せん断応力度 `τxy = Q・Sy/(b・I)`
/// - せん断強度 `τu = 0.5・μ・p′w・σy`
/// - 検定比 = τxy / τu（1.0 以下で OK）
pub fn pca_horizontal_joint_service(inp: &PcaServiceInput) -> CheckResult {
    let denom = inp.b * inp.i;
    let tau_xy = if denom.abs() > 1e-9 {
        (inp.q * inp.s_y / denom).abs()
    } else {
        f64::INFINITY
    };
    let tau_u = 0.5 * inp.mu * inp.pw_total * inp.sigma_y;
    finish("使用限界", tau_xy, tau_u)
}

/// PCa 水平接合面・終局限界状態の検討。
///
/// - 設計用せん断応力度 `τxy = ΔT/(b・Δl)`
///   - `ΔT`: 区間長さにおいて水平接合面より外側に含まれる引張鉄筋の応力変化量
///     [N]。鉛直荷重に対する検討では `ΔT = Md/(0.9・d)`（Md = α・MDL + β・MLL）、
///     地震時荷重に対する検討では引張鉄筋の降伏耐力（強度倍率考慮）とする
///     （いずれも呼び出し側で算定して渡す）。
///   - `Δl`: 区間長さ [mm]（端部から M=0 位置まで。[`moment_zero_distance`]）。
/// - せん断強度 `τu = μ・p′w・σy`（使用限界の 2 倍＝0.5 係数なし）
/// - 検定比 = τxy / τu（1.0 以下で OK）
pub fn pca_horizontal_joint_ultimate(
    delta_t: f64,
    delta_l: f64,
    b: f64,
    mu: f64,
    pw_total: f64,
    sigma_y: f64,
) -> CheckResult {
    let denom = b * delta_l;
    let tau_xy = if denom.abs() > 1e-9 {
        (delta_t / denom).abs()
    } else {
        f64::INFINITY
    };
    let tau_u = mu * pw_total * sigma_y;
    finish("終局限界", tau_xy, tau_u)
}

/// 主筋 1 段の重心位置（引張縁から）k1 = かぶり + あばら筋径 + 主筋径/2。
///
/// `crate::joint_wiring::rc_dt` と同じ定義だが private のためここで再計算する。
fn rc_dt(rebar: &squid_n_core::section_shape::RcRebar) -> f64 {
    rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0
}

/// 引張鉄筋断面積 at [mm²]（主筋 main_x の半数を片側とする仮定。
/// `crate::rc` の `rect_axis_props` と同じ「count/2 が片側」仮定だが
/// private のためここで再計算する）。
fn tension_steel_area(main_x: &squid_n_core::section_shape::BarSet) -> f64 {
    let one_bar = std::f64::consts::PI / 4.0 * main_x.dia * main_x.dia;
    main_x.count as f64 * one_bar / 2.0
}

/// 補強筋の降伏強度 σy [N/mm²]。`Material.fy` があればそれを、無ければ
/// 材料名（鉄筋グレード名）の数値部（例 "SD345"→345）を、どちらも無ければ
/// 345（SD345 相当）を用いる（`crate::rc::rebar_sigma_y` と同じ近似だが
/// private のためここで再計算する）。強度倍率は未考慮。
fn rebar_sigma_y(mat: &squid_n_core::model::Material) -> f64 {
    if let Some(fy) = mat.fy {
        if fy > 0.0 {
            return fy;
        }
    }
    let digits: String = mat.name.chars().filter(|c| c.is_ascii_digit()).collect();
    digits
        .parse::<f64>()
        .ok()
        .filter(|v| *v > 0.0)
        .unwrap_or(345.0)
}

/// 内力リストのうち、評価位置 `pos` に最も近い行を返す。
fn closest_forces(forces: crate::joint_wiring::ForcesAt<'_>, pos: f64) -> Option<&(f64, [f64; 6])> {
    forces.iter().min_by(|a, b| {
        (a.0 - pos)
            .abs()
            .partial_cmp(&(b.0 - pos).abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// PCa 属性が登録された梁部材の水平接合面検定を一括実行する。
///
/// `long_term` は終局限界の設計用引張力 ΔT の算定方法を切り替える（冒頭 doc
/// 参照）。`true`: 鉛直荷重時（`Md/(0.9・d)` 近似）、`false`: 地震時想定
/// （引張鉄筋の降伏耐力）。
///
/// 戻り値: `(要素ID, 評価位置, 検定結果)` のリスト（使用限界・終局限界の
/// 2 行 × 両端部）。属性が登録されていない要素・`RcRect` 以外の断面・
/// 内力が見つからない要素はスキップする。
pub fn collect_pca_checks(
    model: &Model,
    member_forces: &[(ElemId, crate::joint_wiring::ForcesAt<'_>)],
    long_term: bool,
) -> Vec<(ElemId, f64, CheckResult)> {
    let mut out = Vec::new();

    for attr in &model.pca_attrs {
        let Some(elem) = model.elements.iter().find(|e| e.id == attr.elem) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let Some(sec) = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid))
        else {
            continue;
        };
        let Some(SectionShape::RcRect { b, d, ref rebar }) = sec.shape else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid))
        else {
            continue;
        };
        if mat.fc.unwrap_or(0.0) <= 0.0 {
            continue;
        }
        let Some((_, forces)) = member_forces.iter().find(|(id, _)| *id == attr.elem) else {
            continue;
        };
        if forces.is_empty() {
            continue;
        }

        // 断面諸元（矩形。T形協力幅は未対応、冒頭 doc 参照）。
        let i = b * d.powi(3) / 12.0;
        let yj = attr.joint_depth_from_top;
        if yj <= 0.0 || yj >= d {
            continue;
        }
        let s_y = b * yj * (d - yj) / 2.0;
        let d_eff = d - rc_dt(rebar);
        let at = tension_steel_area(&rebar.main_x);

        // 部材長 L（節点座標から算定）。
        let (Some(p0), Some(p1)) = (
            model.nodes.get(elem.nodes[0].index()).map(|n| n.coord),
            model.nodes.get(elem.nodes[1].index()).map(|n| n.coord),
        ) else {
            continue;
        };
        let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let length = (dx * dx + dy * dy + dz * dz).sqrt();
        if length < 1e-9 {
            continue;
        }

        // Δl（区間長さ）は部材 1 本につき 1 つ求め、両端の終局限界検定で共用する。
        let Some(f_end0) = closest_forces(forces, 0.0) else {
            continue;
        };
        let Some(f_end1) = closest_forces(forces, 1.0) else {
            continue;
        };
        let Some(f_mid) = closest_forces(forces, 0.5) else {
            continue;
        };
        let m1 = f_end0.1[5];
        let m2 = -f_end1.1[5];
        let mc = f_mid.1[5];
        let m0_simple = mc.abs() + (m1.abs() + m2.abs()) / 2.0;
        let delta_l =
            moment_zero_distance(-m1.abs(), -m2.abs(), m0_simple, length).unwrap_or(length / 2.0);

        let sigma_y_steel = rebar_sigma_y(mat);

        for (pos, f_end) in [f_end0, f_end1] {
            let q = f_end[1];
            let service_inp = PcaServiceInput {
                q,
                s_y,
                i,
                b,
                mu: attr.mu,
                pw_total: attr.pw_joint,
                sigma_y: attr.sigma_y_joint,
            };
            out.push((attr.elem, *pos, pca_horizontal_joint_service(&service_inp)));

            let delta_t = if long_term {
                let m_end = f_end[5];
                m_end.abs() / (0.9 * d_eff)
            } else {
                at * sigma_y_steel
            };
            let ultimate = pca_horizontal_joint_ultimate(
                delta_t,
                delta_l,
                b,
                attr.mu,
                attr.pw_joint,
                attr.sigma_y_joint,
            );
            out.push((attr.elem, *pos, ultimate));
        }
    }

    out
}

fn finish(state: &str, tau_xy: f64, tau_u: f64) -> CheckResult {
    let ratio = if tau_u > 0.0 {
        tau_xy / tau_u
    } else {
        f64::INFINITY
    };
    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis: format!("PCa 水平接合面（{state}状態）せん断検定"),
        detail: format!("τxy={tau_xy:.4} N/mm², τu={tau_u:.4} N/mm², ratio={ratio:.4}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node,
        PcaBeamAttr, RigidZone,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

    #[test]
    fn moment_zero_distance_symmetric_beam() {
        // 両端 M1=M2=-100（上側引張）、中央 M0=+100 の対称分布:
        // M(x) = -100 + 800・x/L − 400・x²/L²…係数確認は解の対称性で行う。
        // M(0)<0, M(L/2)>0 なので (0, L/2) に M=0 があり、対称ゆえ両端から等距離。
        let l = 6000.0;
        let d = moment_zero_distance(-100.0, -100.0, 100.0, l).expect("解があるはず");
        assert!(d > 0.0 && d < l / 2.0);
        // 手計算: -100 + (100+100+400)x/L−400x²/L² = 0 → ξ=x/L:
        // -1 + 6ξ - 4ξ² = 0 → ξ = (6±√(36-16))/8 = (6±√20)/8 → ξ1≈0.1910
        let xi = (6.0 - 20.0_f64.sqrt()) / 8.0;
        assert!((d - xi * l).abs() < 1e-6, "d={d}, expected={}", xi * l);
    }

    #[test]
    fn moment_zero_distance_no_root_when_same_sign() {
        // 全長で正（下側引張のみ）: 根なし。符号規約により M(0)=M1、M(L)=−M2
        // なので、両端で正となるのは M1>0 かつ M2<0 の場合。
        // M(ξ) = 100 + 400ξ − 400ξ²（頂点 ξ=0.5 で 200 > 0）。
        assert!(moment_zero_distance(100.0, -100.0, 100.0, 6000.0).is_none());
    }

    #[test]
    fn pca_service_hand_calc() {
        // 矩形断面 b=400, D=700 の上端から 150mm の接合面:
        // 図心から接合面まで y1 = 350-150 = 200、外側部分 A=400×150、
        // 重心 y=350-75=275 → Sy = 400×150×275 = 16.5e6 mm³。
        // I = 400×700³/12 = 11.433e9 mm⁴。Q=200kN →
        // τxy = 200e3×16.5e6/(400×11.433e9) = 0.7217 N/mm²
        let inp = PcaServiceInput {
            q: 200_000.0,
            s_y: 16.5e6,
            i: 400.0 * 700.0_f64.powi(3) / 12.0,
            b: 400.0,
            mu: 0.6,
            pw_total: 0.008,
            sigma_y: 345.0,
        };
        let res = pca_horizontal_joint_service(&inp);
        let tau_xy = 200_000.0 * 16.5e6 / (400.0 * 400.0 * 700.0_f64.powi(3) / 12.0);
        let tau_u = 0.5 * 0.6 * 0.008 * 345.0;
        assert!((res.ratio - tau_xy / tau_u).abs() < 1e-9);
    }

    #[test]
    fn pca_ultimate_is_twice_service_strength() {
        // 同一 μ・p′w・σy に対し、終局の τu は使用限界の 2 倍。
        let service = pca_horizontal_joint_service(&PcaServiceInput {
            q: 1.0,
            s_y: 1.0,
            i: 1.0,
            b: 1.0,
            mu: 1.0,
            pw_total: 0.01,
            sigma_y: 300.0,
        });
        let ultimate = pca_horizontal_joint_ultimate(1.0, 1.0, 1.0, 1.0, 0.01, 300.0);
        assert!((service.ratio / ultimate.ratio - 2.0).abs() < 1e-9);
    }

    #[test]
    fn pca_zero_strength_is_ng() {
        let res = pca_horizontal_joint_ultimate(1000.0, 100.0, 10.0, 0.6, 0.0, 345.0);
        assert!(!res.ok);
        assert!(res.ratio.is_infinite());
    }

    // ------------------------------------------------------------------
    // collect_pca_checks（自動配線）
    // ------------------------------------------------------------------

    /// 矩形 RC 梁 1 本（b=400, D=700, L=6000mm, X 軸方向）のモデル。
    /// `pca_attr` を指定すると `model.pca_attrs` に登録する。
    fn pca_beam_model(shape: SectionShape, pca_attr: Option<PcaBeamAttr>) -> Model {
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [6000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
        ];
        let sections = vec![shape.to_section(SectionId(0), "beam".to_string())];
        let materials = vec![Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".to_string(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }];
        let elements = vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: {
                let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                v.push(NodeId(0));
                v.push(NodeId(1));
                v
            },
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        }];
        Model {
            nodes,
            elements,
            sections,
            materials,
            pca_attrs: pca_attr.into_iter().collect(),
            ..Default::default()
        }
    }

    fn rc_rect_shape() -> SectionShape {
        SectionShape::RcRect {
            b: 400.0,
            d: 700.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 19.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 150.0,
                    legs: 2,
                    grade: None,
                },
            },
        }
    }

    fn default_pca_attr() -> PcaBeamAttr {
        PcaBeamAttr {
            elem: ElemId(0),
            mu: 0.6,
            pw_joint: 0.008,
            sigma_y_joint: 345.0,
            joint_depth_from_top: 150.0,
        }
    }

    /// 属性が登録された RcRect 梁は 2 端部 × (使用限界・終局限界) = 4 行返し、
    /// 使用限界の τxy は手計算（Q・Sy/(b・I)）と一致する。
    #[test]
    fn collect_pca_checks_returns_four_rows_and_service_matches_hand_calc() {
        let model = pca_beam_model(rc_rect_shape(), Some(default_pca_attr()));
        let forces: Vec<(f64, [f64; 6])> = vec![
            (0.0, [0.0, 200_000.0, 0.0, 0.0, 0.0, -100.0e6]),
            (0.5, [0.0, 0.0, 0.0, 0.0, 0.0, 50.0e6]),
            (1.0, [0.0, 200_000.0, 0.0, 0.0, 0.0, 80.0e6]),
        ];
        let member_forces = vec![(ElemId(0), forces.as_slice())];

        let results = collect_pca_checks(&model, &member_forces, false);
        assert_eq!(results.len(), 4, "2端部×(使用限界・終局限界)=4行のはず");

        let service_rows: Vec<&(ElemId, f64, CheckResult)> = results
            .iter()
            .filter(|(_, _, cr)| cr.basis.contains("使用限界"))
            .collect();
        assert_eq!(service_rows.len(), 2);

        // 手計算（pca_service_hand_calc と同一の断面・接合面位置・Q）:
        // Sy = 400×150×(700-150)/2 = 16.5e6 mm³、I = 400×700³/12。
        let s_y = 400.0 * 150.0 * (700.0 - 150.0) / 2.0;
        let i = 400.0 * 700.0_f64.powi(3) / 12.0;
        let expected_tau_xy = 200_000.0 * s_y / (400.0 * i);
        let expected_tau_u = 0.5 * 0.6 * 0.008 * 345.0;
        let expected_ratio = expected_tau_xy / expected_tau_u;
        for (_, _, cr) in &service_rows {
            assert!(
                (cr.ratio - expected_ratio).abs() < 1e-6,
                "ratio={} expected={}",
                cr.ratio,
                expected_ratio
            );
        }

        let ultimate_rows: Vec<&(ElemId, f64, CheckResult)> = results
            .iter()
            .filter(|(_, _, cr)| cr.basis.contains("終局限界"))
            .collect();
        assert_eq!(ultimate_rows.len(), 2);
    }

    /// PCa 属性が未登録のモデルは空を返す。
    #[test]
    fn collect_pca_checks_empty_without_attrs() {
        let model = pca_beam_model(rc_rect_shape(), None);
        let forces: Vec<(f64, [f64; 6])> = vec![(0.0, [0.0, 200_000.0, 0.0, 0.0, 0.0, 0.0])];
        let member_forces = vec![(ElemId(0), forces.as_slice())];
        assert!(collect_pca_checks(&model, &member_forces, false).is_empty());
    }

    /// RcRect 以外の断面形状（例: SteelH）は属性が登録されていてもスキップする。
    #[test]
    fn collect_pca_checks_skips_non_rc_rect_shape() {
        let steel_shape = SectionShape::SteelH {
            height: 700.0,
            width: 300.0,
            web_thick: 13.0,
            flange_thick: 24.0,
        };
        let mut model = pca_beam_model(steel_shape, Some(default_pca_attr()));
        // 鋼材扱いにするため fc=None（steel_h では材料の fc は使わない想定だが、
        // RcRect 判定でスキップされることが本テストの主眼）。
        model.materials[0].fc = None;
        let forces: Vec<(f64, [f64; 6])> = vec![
            (0.0, [0.0, 200_000.0, 0.0, 0.0, 0.0, -100.0e6]),
            (0.5, [0.0, 0.0, 0.0, 0.0, 0.0, 50.0e6]),
            (1.0, [0.0, 200_000.0, 0.0, 0.0, 0.0, 80.0e6]),
        ];
        let member_forces = vec![(ElemId(0), forces.as_slice())];
        assert!(collect_pca_checks(&model, &member_forces, false).is_empty());
    }
}
