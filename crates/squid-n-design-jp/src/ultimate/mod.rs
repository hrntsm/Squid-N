//! **終局検定（RESP-D マニュアル「計算編 06 終局検定」）**。
//!
//! 非線形解析（荷重増分解析）で崩壊機構が形成された後、各部材が終局せん断強度・
//! 付着割裂耐力に対して十分な余裕（せん断・付着が曲げに先行して破壊しないこと）を
//! 持つかを検定する。RESP-D で「終局強度型設計指針」を選択した場合の
//! 塑性理論式（[`rc_shear`]）と、柱の軸終局耐力（[`rc_axial`]）を実装する。
//!
//! # 検定の考え方（RESP-D「06 終局検定」採用応力・RC 柱 b) 余裕度）
//! - 両端ヒンジを仮定した終局せん断応力 `Qmu = 上限強度倍率·(Mu上+Mu下)/内法` を
//!   設計用せん断力とし、終局せん断強度 `Qsu`（塑性理論式）・付着割裂耐力 `Qbu`
//!   との比（余裕度 `Qsu/Qmu`, `Qbu/Qmu`）を算定する。
//! - 余裕度 ≥ 1.0（せん断・付着が曲げ降伏に先行しない）で OK。
//!
//! # 曲げ終局強度 Mu
//! 梁は [`squid_n_core::rc_capacity::rc_mu_simple`]（構造規定 at 式）を用いる。
//! 柱は [`MuMethod`] により、軸力を考慮した構造規定 at 式
//! （[`squid_n_core::rc_capacity::rc_column_mu_simple`]）または ACI 規準の平面保持
//! 解析（[`rc_column_aci::rc_column_mu_aci`]）を選択できる。
//!
//! # 適用範囲・簡略化（doc 兼申し送り）
//! - RC 部材の検定対象は `SectionShape::RcRect`（矩形 RC 断面）のみ。円形柱・SRC・鋼は
//!   別途（本モジュールの RC 経路の対象外）。CFT 柱の軸終局耐力は [`cft`]、柱梁接合部の
//!   終局耐力は [`joint`] を参照。
//! - せん断・付着は強軸（せい方向主筋 main_x）を基本とし、柱は指定により 2 軸せん断
//!   （[`biaxial_margin`]）も検定できる。終局せん断強度は [`ShearMethod`] により
//!   塑性理論式（[`rc_shear`]）または靭性指針式 Vu（[`rc_shear_ductility`]）を選択できる。
//! - 主筋は上下対称配筋を仮定し、引張側主筋量は main_x の総断面積の半分とする。

use crate::MemberKind;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, Material, Model, Section};
use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

pub mod cft;
pub mod cft_nm;
pub mod joint;
pub mod rc_axial;
pub mod rc_column_aci;
pub mod rc_shear;
pub mod rc_shear_ductility;

pub use cft::{
    cft_axial_ultimate, cft_column_class, cft_concrete_buckling_axial,
    cft_concrete_buckling_stress, cft_concrete_slenderness, cft_ncu1, CftAxialInput,
    CftAxialUltimate, CftColumnClass,
};
pub use cft_nm::{
    cft_long_medium_column_mu, cft_nk, cft_short_column_mu, CftBendingInput, CftLongMediumInput,
};
pub use joint::{
    joint_fj, joint_kappa, rc_joint_ultimate, RcJointUltimateInput, RcJointUltimateResult,
};
pub use rc_axial::{rc_axial_margin, rc_column_axial_ultimate, RcAxialUltimate};
pub use rc_column_aci::{aci_beta1, rc_column_mu_aci, AciColumnInput};
pub use rc_shear::{
    bond_reliable_strength_deformed, bond_split_ratio, plastic_cot_phi, plastic_k1, plastic_k2,
    plastic_nu, plastic_nu0, rc_shear_qbu_bond, rc_shear_qsu_plastic, BondStrengthInput,
    RcBondSplitInput, RcPlasticShearInput,
};
pub use rc_shear_ductility::{
    arch_tan_theta, bond_force_tx, ductility_mu, ductility_nu, rc_shear_vbu_ductility,
    rc_shear_vu_ductility, truss_lambda, RcDuctilityShearInput, RcVbuInput,
};

/// 部材の設計用需要（終局検定の入力）。**圧縮を正**とする軸力と、強軸・弱軸まわりの
/// 設計用曲げモーメント（応答値。二軸曲げ余裕度に用いる）。
///
/// `shear`・`rp` は**プッシュオーバー応答からの直接反映**（[`MemberDemand::from_pushover`]）
/// に用いる任意項目で、`None`（既定）のときは従来どおり両端ヒンジ `Qmu=2·Mu/内法` と
/// UI 一律指定 Rp（[`UltimateShearOptions::rp`]）を用いる。
#[derive(Clone, Copy, Debug, Default)]
pub struct MemberDemand {
    /// 設計軸力 [N]（**圧縮正**）。柱の Mu・軸余裕度に用いる。
    pub n_axial: f64,
    /// 強軸（せい方向）まわりの設計用曲げモーメント Mmx [N·mm]（符号は内部で abs）。
    pub mz: f64,
    /// 弱軸（幅方向）まわりの設計用曲げモーメント Mmy [N·mm]（符号は内部で abs）。
    pub my: f64,
    /// 設計用せん断力 Qm [N]（プッシュオーバー応答の強軸せん断。`Some` のとき
    /// `Qmu = 上限強度倍率·|Qm|` として両端ヒンジ略算（2·Mu/内法）を置き換える）。
    pub shear: Option<f64>,
    /// 弱軸の設計用せん断力 Qmy [N]（プッシュオーバー応答の弱軸せん断。`Some` のとき
    /// 2 軸せん断余裕度の弱軸需要 `Qmuy = 上限強度倍率·|Qmy|`（両端ヒンジ略算 2·Muy/内法
    /// を置き換える）に用いる。2 軸せん断を検定しない場合は無視される）。
    pub shear_weak: Option<f64>,
    /// 部材別のヒンジ回転角 Rp [rad]（プッシュオーバー終局時の部材変形角）。`Some`
    /// のとき ν・cotφ・μ・tanθ に用いる Rp を [`UltimateShearOptions::rp`] から置き換える。
    pub rp: Option<f64>,
}

impl MemberDemand {
    /// 軸力のみ（曲げ・せん断需要 0、Rp は UI 一律指定に従う）の需要を作る。
    pub fn axial(n_axial: f64) -> Self {
        Self {
            n_axial,
            mz: 0.0,
            my: 0.0,
            shear: None,
            shear_weak: None,
            rp: None,
        }
    }

    /// プッシュオーバー応答から部材需要を作る。軸力（圧縮正）・強軸/弱軸の設計用曲げ・
    /// 強軸/弱軸の設計用せん断・部材別 Rp を直接反映する（`Qmu`・弱軸 Qmuy・Rp を
    /// 応答値で置き換える）。
    #[allow(clippy::too_many_arguments)]
    pub fn from_pushover(
        n_axial: f64,
        mz: f64,
        my: f64,
        shear: f64,
        shear_weak: f64,
        rp: f64,
    ) -> Self {
        Self {
            n_axial,
            mz,
            my,
            shear: Some(shear),
            shear_weak: Some(shear_weak),
            rp: Some(rp),
        }
    }
}

/// 柱の曲げ終局強度 Mu の算定方法（RESP-D「06 終局検定」柱 a)）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MuMethod {
    /// 構造規定式（at 式、軸力考慮の閉形式略算）。
    #[default]
    AtFormula,
    /// ACI 規準による平面保持解析（等価応力度ブロック法）。
    Aci,
}

/// 終局せん断強度の算定方法（RESP-D「06 終局検定」の「終局耐力条件」選択に対応）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ShearMethod {
    /// 塑性理論式（藤井・森田式系、「終局強度型設計指針」）。既定。
    #[default]
    Plastic,
    /// 靭性指針式 Vu=min(Vu1,Vu2,Vu3)（AIJ「靭性保証型耐震設計指針」6.4）。
    Ductility,
}

/// 終局検定（塑性理論式）の算定オプション。
#[derive(Clone, Copy, Debug)]
pub struct UltimateShearOptions {
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν・cotφ に用いる。既定 0）。
    pub rp: f64,
    /// 軽量コンクリートを使用する場合 true（せん断終局耐力を 0.9 倍に低減）。
    pub lightweight: bool,
    /// 上限強度倍率（Qmu = 上限強度倍率·(Mu上+Mu下)/内法。既定 1.0）。
    pub upper_strength_factor: f64,
    /// せん断補強筋の降伏強度算定用強度 σwy [N/mm²]（モデルに材質情報が無い場合の
    /// 代表値。既定 295 = SD295 相当）。
    pub sigma_wy: f64,
    /// 付着割裂の検定を含める場合 true。
    pub include_bond: bool,
    /// 柱の曲げ終局強度 Mu の算定方法（既定 at 式）。
    pub mu_method: MuMethod,
    /// 終局せん断強度の算定方法（既定 塑性理論式）。靭性指針式を選ぶと Qsu 列に
    /// Vu=min(Vu1,Vu2,Vu3)（[`rc_shear_ductility`]）を用いる。
    pub shear_method: ShearMethod,
    /// 柱のせん断を 2 軸せん断として検定する場合 true（RESP-D「06 終局検定」
    /// 採用応力：RC のみ指定により 2 軸せん断）。両軸の Qmu/Qsu を相互作用式
    /// `1/((Qmx/Qux)^αx+(Qmy/Quy)^αy)^(1/α)`（RC は α=2.0）で合成する。
    pub biaxial_shear: bool,
    /// 柱の曲げを 2 軸曲げとして検定する場合 true（RESP-D「06 終局検定」採用応力）。
    /// 両軸の設計用曲げ Mm と終局曲げ強度 Mu を相互作用式
    /// `1/((Mmx/Mux)^αx+(Mmy/Muy)^αy)^(1/α)`（RC は α=2.0）で合成する。
    /// 設計用曲げ需要 Mmx/Mmy は [`MemberDemand`] の mz/my を用いる。
    pub biaxial_bending: bool,
}

impl Default for UltimateShearOptions {
    fn default() -> Self {
        Self {
            rp: 0.0,
            lightweight: false,
            upper_strength_factor: 1.0,
            sigma_wy: 295.0,
            include_bond: true,
            mu_method: MuMethod::default(),
            shear_method: ShearMethod::default(),
            biaxial_shear: false,
            biaxial_bending: false,
        }
    }
}

/// 2 軸相互作用の余裕度 `1/((rx)^α + (ry)^α)^(1/α)`（RESP-D「06 終局検定」採用応力）。
///
/// `rx`,`ry` は各軸の「需要/耐力」比（例: `Qmx/Qux`, `Qmy/Quy`）、`alpha` は相互作用の
/// 指数（RC 柱は 2.0）。ここでは αx=αy=α と等しく扱う。両比が 0 のとき（需要ゼロ）は
/// `f64::INFINITY` を返す。`alpha ≤ 0` の不正入力も `f64::INFINITY`。
pub fn biaxial_margin(rx: f64, ry: f64, alpha: f64) -> f64 {
    if alpha <= 0.0 {
        return f64::INFINITY;
    }
    let rx = rx.max(0.0);
    let ry = ry.max(0.0);
    let s = rx.powf(alpha) + ry.powf(alpha);
    if s <= 0.0 {
        f64::INFINITY
    } else {
        1.0 / s.powf(1.0 / alpha)
    }
}

/// 1 部材分の終局検定結果。
#[derive(Clone, Debug)]
pub struct UltimateCheck {
    /// 部材 ID。
    pub elem: ElemId,
    /// 部材種別（梁/柱）。
    pub kind: MemberKind,
    /// 曲げ終局強度 Mu [N·mm]。
    pub mu: f64,
    /// 両端ヒンジ時せん断力 Qmu = 上限強度倍率·2·Mu/内法 [N]。
    pub qmu: f64,
    /// 塑性理論式による終局せん断強度 Qsu [N]。
    pub qsu: f64,
    /// 付着割裂による終局せん断耐力 Qbu [N]（`include_bond=false` なら 0）。
    pub qbu: f64,
    /// せん断余裕度 Qsu/Qmu（強軸）。
    pub shear_margin: f64,
    /// 2 軸せん断余裕度（柱かつ `biaxial_shear=true` のとき Some）。
    /// `1/((Qmx/Qsux)^2+(Qmy/Qsuy)^2)^(1/2)`。
    pub biaxial_shear_margin: Option<f64>,
    /// 2 軸曲げ余裕度（柱かつ `biaxial_bending=true` のとき Some）。
    /// `1/((Mmx/Mux)^2+(Mmy/Muy)^2)^(1/2)`。設計用曲げ需要が 0 なら `f64::INFINITY`。
    pub biaxial_bending_margin: Option<f64>,
    /// 付着余裕度 Qbu/Qmu（`include_bond=false` なら `f64::INFINITY`）。
    pub bond_margin: f64,
    /// 軸終局耐力（柱のみ Some）。
    pub axial: Option<RcAxialUltimate>,
    /// 判定（せん断余裕度・付着余裕度が共に 1.0 以上で true）。
    pub ok: bool,
    /// 根拠（表示用）。
    pub basis: String,
    /// 詳細（表示用）。
    pub detail: String,
}

/// 主筋セットの総断面積 [mm²]。
fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * std::f64::consts::PI / 4.0 * bar.dia * bar.dia
}

/// せん断補強筋比 pw = (legs·π/4·dia²)/(b·pitch)。pitch ≤ 0 なら 0。
fn hoop_pw(rebar: &RcRebar, b: f64) -> f64 {
    if rebar.shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    aw / (b * rebar.shear.pitch)
}

/// 部材軸の鉛直成分 |ez| から部材種別を判定する（app の `member_kind_of` と同規則）。
fn member_kind(elem: &ElementData, model: &Model) -> MemberKind {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (d[2] / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 [mm]。
fn geometric_length(elem: &ElementData, model: &Model) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// 内法長さ [mm] = 幾何長 − 両端フェイス距離。フェイス合計が幾何長以上の
/// 不整合入力では幾何長のままとする（app の rank-auto と同規則）。
fn clear_span(elem: &ElementData, model: &Model) -> f64 {
    let geom = geometric_length(elem, model);
    let face_sum = elem.rigid_zone.face_i + elem.rigid_zone.face_j;
    if geom - face_sum > 0.0 {
        geom - face_sum
    } else {
        geom
    }
}

/// 指定方向（`b_dir`=幅, `d_dir`=せい, `main`=当該方向主筋）の柱の終局せん断強度
/// `Qsu`（塑性理論式）と両端ヒンジ時せん断力 `Qmu` を算定する（2 軸せん断用）。
#[allow(clippy::too_many_arguments)]
fn column_axis_shear(
    b_dir: f64,
    d_dir: f64,
    main: &BarSet,
    rebar: &RcRebar,
    fc: f64,
    sigma_y: f64,
    ag: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> (f64, f64) {
    let dt = rebar.cover + rebar.shear.dia + main.dia / 2.0;
    let d_eff = d_dir - dt;
    if d_eff <= 0.0 {
        return (0.0, 0.0);
    }
    let jt = 7.0 * d_eff / 8.0;
    let at = bar_set_area(main) / 2.0;
    let pw = hoop_pw(rebar, b_dir);
    let qsu = member_shear_strength(b_dir, d_dir, jt, pw, rebar, fc, n_axial, l_clear, opts);
    let cap = RcCapacityInput {
        b: b_dir,
        d: d_dir,
        at,
        d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = rc_column_mu_simple(&cap, ag, n_axial);
    let qmu = if l_clear > 0.0 {
        opts.upper_strength_factor * 2.0 * mu / l_clear
    } else {
        0.0
    };
    (qsu, qmu)
}

/// 柱の曲げ終局強度 Mu [N·mm]（`mu_method` に応じて at 式 / ACI 平面保持）。
/// `b_dir`=幅, `d_dir`=せい, `dt`=引張縁〜引張筋距離, `at`=引張側主筋, `ag`=全主筋。
#[allow(clippy::too_many_arguments)]
fn column_mu(
    b_dir: f64,
    d_dir: f64,
    dt: f64,
    at: f64,
    ag: f64,
    sigma_y: f64,
    fc: f64,
    n_axial: f64,
    mu_method: MuMethod,
) -> f64 {
    match mu_method {
        MuMethod::Aci => {
            let layers = [(dt, at), (d_dir - dt, at)];
            rc_column_mu_aci(
                &AciColumnInput {
                    b: b_dir,
                    d_full: d_dir,
                    fc,
                    sigma_y,
                    es: 205000.0,
                },
                &layers,
                n_axial,
            )
        }
        MuMethod::AtFormula => {
            let cap = RcCapacityInput {
                b: b_dir,
                d: d_dir,
                at,
                d_eff: (d_dir - dt).max(1.0),
                sigma_y,
                fc,
                pw: 0.0,
                sigma_wy: 0.0,
                clear_span: 1.0,
                sigma_0: 0.0,
            };
            rc_column_mu_simple(&cap, ag, n_axial)
        }
    }
}

/// 靭性指針式による終局せん断信頼強度 `Vu` [N]（[`rc_shear_ductility`]）を断面諸元から
/// 算定する。`b_dir`=幅, `d_dir`=せい, `je`=トラス機構有効せい（`jt` を用いる）。
///
/// # 簡略化（doc 兼申し送り）
/// マニュアルの `be`（トラス機構有効幅＝外側横補強筋の芯々間隔）・`Ns`（中子筋本数）は
/// モデルに直接保持されないため、以下で近似する:
/// - `be = 幅 − 2·(かぶり + 補強筋径/2)`（せん断補強筋のコア芯々幅）。
/// - `pwe = aw/(be·s)`（`aw`＝1 組の補強筋断面積、`s`＝ピッチ）。
/// - `Ns = legs/2 − 1`（2 本脚→Ns=0、4 本脚→Ns=1、…）。
/// - 引張軸力（`n_axial < 0`）の柱は `tanθ=0`（アーチ機構無効）。
#[allow(clippy::too_many_arguments)]
fn member_vu_ductility(
    b_dir: f64,
    d_dir: f64,
    je: f64,
    rebar: &RcRebar,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> f64 {
    let (be, n_s) = ductility_be_ns(b_dir, rebar);
    let s = rebar.shear.pitch;
    let aw =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    let pwe = if s > 0.0 { aw / (be * s) } else { 0.0 };
    rc_shear_vu_ductility(&RcDuctilityShearInput {
        b: b_dir,
        d_full: d_dir,
        be,
        je,
        pwe,
        sigma_wy: opts.sigma_wy,
        s,
        n_s,
        l_clear,
        fc,
        rp: opts.rp,
        tensile_axial: n_axial < 0.0,
        lightweight: opts.lightweight,
    })
}

/// 靭性指針式のトラス機構有効幅 `be`（外側横補強筋の芯々間隔近似）と中子筋本数 `Ns`
/// （`legs/2 − 1` 近似）を断面諸元から求める（[`member_vu_ductility`]・Vbu で共用）。
fn ductility_be_ns(b_dir: f64, rebar: &RcRebar) -> (f64, u32) {
    let be = (b_dir - 2.0 * (rebar.cover + rebar.shear.dia / 2.0)).max(1.0);
    let n_s = (rebar.shear.legs / 2).saturating_sub(1);
    (be, n_s)
}

/// 選択された [`ShearMethod`] に応じた終局せん断強度 `Qsu`/`Vu` [N]。
#[allow(clippy::too_many_arguments)]
fn member_shear_strength(
    b_dir: f64,
    d_dir: f64,
    jt: f64,
    pw: f64,
    rebar: &RcRebar,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> f64 {
    match opts.shear_method {
        ShearMethod::Plastic => rc_shear_qsu_plastic(&RcPlasticShearInput {
            b: b_dir,
            d_full: d_dir,
            jt,
            pw,
            sigma_wy: opts.sigma_wy,
            l_clear,
            fc,
            rp: opts.rp,
            lightweight: opts.lightweight,
        }),
        ShearMethod::Ductility => {
            member_vu_ductility(b_dir, d_dir, jt, rebar, fc, n_axial, l_clear, opts)
        }
    }
}

/// 1 部材の終局検定を実行する（`RcRect` 以外・Fc 未設定は `None`）。
fn check_member(
    elem: &ElementData,
    sec: &Section,
    mat: &Material,
    model: &Model,
    demand: MemberDemand,
    opts: &UltimateShearOptions,
) -> Option<UltimateCheck> {
    let SectionShape::RcRect { b, d, rebar } = sec.shape.as_ref()? else {
        return None;
    };
    let (b, d) = (*b, *d);
    let fc = mat.fc?;
    if fc <= 0.0 || b <= 0.0 || d <= 0.0 {
        return None;
    }
    // 部材別 Rp（プッシュオーバー応答からの直接反映）が与えられていれば UI 一律 Rp を
    // 置き換える。以降 opts.rp を参照する全経路（ν・cotφ・μ・tanθ）に効く。
    let opts_owned;
    let opts = if let Some(rp) = demand.rp {
        opts_owned = UltimateShearOptions {
            rp: rp.max(0.0),
            ..*opts
        };
        &opts_owned
    } else {
        opts
    };
    let kind = member_kind(elem, model);
    let sigma_y = mat.fy.unwrap_or(345.0);
    let l_clear = clear_span(elem, model);

    // 断面諸元（強軸＝せい方向主筋 main_x）。
    let dt = rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0;
    let d_eff = d - dt;
    if d_eff <= 0.0 {
        return None;
    }
    let jt = 7.0 * d_eff / 8.0;
    let at = bar_set_area(&rebar.main_x) / 2.0;
    let ag = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    let pw = hoop_pw(rebar, b);
    let n_axial = demand.n_axial;

    // 曲げ終局強度 Mu（柱は軸力考慮・mu_method 対応、梁は軸力なし）。
    let cap = RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = match kind {
        MemberKind::Column => column_mu(b, d, dt, at, ag, sigma_y, fc, n_axial, opts.mu_method),
        _ => rc_mu_simple(&cap),
    };

    // 設計用せん断力 Qmu。プッシュオーバー応答の設計用せん断が与えられていれば
    // それを直接反映（上限強度倍率を乗じる）、無ければ両端ヒンジ略算 2·Mu/内法。
    let qmu = match demand.shear {
        Some(qm) => opts.upper_strength_factor * qm.abs(),
        None => {
            if l_clear > 0.0 {
                opts.upper_strength_factor * 2.0 * mu / l_clear
            } else {
                0.0
            }
        }
    };

    // 終局せん断強度 Qsu（塑性理論式）または Vu（靭性指針式）。
    let qsu = member_shear_strength(b, d, jt, pw, rebar, fc, n_axial, l_clear, opts);

    // 付着割裂耐力 Qbu。
    let (qbu, tau_bu) = if opts.include_bond {
        // 引張側主筋本数（対称配筋の半分、外側一列を代表）。
        let n_tension = (rebar.main_x.count as f64 / 2.0).max(1.0);
        let tau_bu = bond_reliable_strength_deformed(&BondStrengthInput {
            fc,
            b,
            db1: rebar.main_x.dia,
            n_bars: n_tension.round() as u32,
            cover_side: rebar.cover,
            cover_bottom: rebar.cover,
            hoop_area: rebar.shear.legs as f64 * std::f64::consts::PI / 4.0
                * rebar.shear.dia
                * rebar.shear.dia,
            hoop_pitch: rebar.shear.pitch,
            pw,
            top_bar: false,
        });
        let sum_phi = n_tension * std::f64::consts::PI * rebar.main_x.dia;
        // 塑性理論式は付着割裂耐力 Qbu、靭性指針式は付着考慮せん断信頼強度 Vbu を用いる。
        let qbu = match opts.shear_method {
            ShearMethod::Plastic => rc_shear_qbu_bond(&RcBondSplitInput {
                b,
                d_full: d,
                jt,
                tau_bu,
                sum_phi,
                l_clear,
                fc,
                rp: opts.rp,
                lightweight: opts.lightweight,
            }),
            ShearMethod::Ductility => {
                let (be, n_s) = ductility_be_ns(b, rebar);
                rc_shear_vbu_ductility(&RcVbuInput {
                    b,
                    d_full: d,
                    be,
                    je: jt,
                    tau_bu,
                    sum_phi1: sum_phi,
                    // モデルは 1 段配筋を仮定するため 2 段目主筋（τbu2・Σφ2）は 0。
                    tau_bu2: 0.0,
                    sum_phi2: 0.0,
                    s: rebar.shear.pitch,
                    n_s,
                    l_clear,
                    fc,
                    rp: opts.rp,
                    tensile_axial: n_axial < 0.0,
                    // Rp>0（ヒンジ回転を指定）を降伏ヒンジ計画部材とみなす（6.8.16b）。
                    yield_hinge: opts.rp > 0.0,
                    lightweight: opts.lightweight,
                })
            }
        };
        (qbu, tau_bu)
    } else {
        (0.0, 0.0)
    };

    let shear_margin = if qmu > 0.0 { qsu / qmu } else { f64::INFINITY };
    let bond_margin = if !opts.include_bond {
        f64::INFINITY
    } else if qmu > 0.0 {
        qbu / qmu
    } else {
        f64::INFINITY
    };

    // 2 軸せん断余裕度（柱のみ、指定時）。弱軸（main_y、b↔D 入替）の Qsu/Qmu を
    // 算定し、相互作用式 1/((Qmx/Qsux)^2+(Qmy/Qsuy)^2)^(1/2)（RC は α=2.0）で合成する。
    let biaxial_shear_margin = if kind == MemberKind::Column && opts.biaxial_shear {
        let (qsu_y, qmu_y_hinge) = column_axis_shear(
            d,
            b,
            &rebar.main_y,
            rebar,
            fc,
            sigma_y,
            ag,
            n_axial,
            l_clear,
            opts,
        );
        // 弱軸設計用せん断 Qmuy。プッシュオーバー応答の弱軸せん断が与えられていれば
        // それを直接反映（上限強度倍率を乗じる）、無ければ両端ヒンジ略算 2·Muy/内法。
        let qmu_y = match demand.shear_weak {
            Some(qmy) => opts.upper_strength_factor * qmy.abs(),
            None => qmu_y_hinge,
        };
        let rx = if qsu > 0.0 { qmu / qsu } else { f64::INFINITY };
        let ry = if qsu_y > 0.0 {
            qmu_y / qsu_y
        } else {
            f64::INFINITY
        };
        Some(biaxial_margin(rx, ry, 2.0))
    } else {
        None
    };

    // 2 軸曲げ余裕度（柱のみ、指定時）。強軸 Mux（=mu）・弱軸 Muy（main_y, b↔D 入替）の
    // 終局曲げ強度と設計用曲げ需要 Mmx=|mz|, Mmy=|my| を相互作用式で合成する。
    let biaxial_bending_margin = if kind == MemberKind::Column && opts.biaxial_bending {
        let dt_y = rebar.cover + rebar.shear.dia + rebar.main_y.dia / 2.0;
        let at_y = bar_set_area(&rebar.main_y) / 2.0;
        let mux = mu;
        let muy = column_mu(d, b, dt_y, at_y, ag, sigma_y, fc, n_axial, opts.mu_method);
        let rx = if mux > 0.0 {
            demand.mz.abs() / mux
        } else if demand.mz.abs() > 0.0 {
            f64::INFINITY
        } else {
            0.0
        };
        let ry = if muy > 0.0 {
            demand.my.abs() / muy
        } else if demand.my.abs() > 0.0 {
            f64::INFINITY
        } else {
            0.0
        };
        Some(biaxial_margin(rx, ry, 2.0))
    } else {
        None
    };

    let axial = if kind == MemberKind::Column {
        Some(rc_column_axial_ultimate(b, d, fc, ag, sigma_y))
    } else {
        None
    };

    // せん断判定は 2 軸指定時は 2 軸余裕度、そうでなければ強軸せん断余裕度を用いる。
    let effective_shear_ok = match biaxial_shear_margin {
        Some(m) => m >= 1.0,
        None => shear_margin >= 1.0,
    };
    // 2 軸曲げ指定時は曲げ余裕度も判定に加える。
    let bending_ok = biaxial_bending_margin.map(|m| m >= 1.0).unwrap_or(true);
    let ok = effective_shear_ok && bond_margin >= 1.0 && bending_ok;

    let shear_label = match opts.shear_method {
        ShearMethod::Plastic => "塑性理論式 Qsu",
        ShearMethod::Ductility => "靭性指針式 Vu",
    };
    let basis = match kind {
        MemberKind::Column => format!("RC柱 終局検定（{shear_label}/Qbu）"),
        _ => format!("RC梁 終局検定（{shear_label}/Qbu）"),
    };
    let biaxial_str = match biaxial_shear_margin {
        Some(m) => format!(", 2軸せん断余裕度={m:.3}"),
        None => String::new(),
    };
    let bend_str = match biaxial_bending_margin {
        Some(m) => format!(", 2軸曲げ余裕度={m:.3}"),
        None => String::new(),
    };
    let detail = format!(
        "Mu={:.0} N·mm, Qmu={:.0} N, Qsu={:.0} N, Qbu={:.0} N, τbu={:.3} N/mm², \
         Qsu/Qmu={:.3}, Qbu/Qmu={:.3}{}{}, pw={:.5}, jt={:.1} mm, L={:.0} mm, Rp={:.4}",
        mu,
        qmu,
        qsu,
        qbu,
        tau_bu,
        shear_margin,
        bond_margin,
        biaxial_str,
        bend_str,
        pw,
        jt,
        l_clear,
        opts.rp
    );

    Some(UltimateCheck {
        elem: elem.id,
        kind,
        mu,
        qmu,
        qsu,
        qbu,
        shear_margin,
        biaxial_shear_margin,
        biaxial_bending_margin,
        bond_margin,
        axial,
        ok,
        basis,
        detail,
    })
}

/// モデルの RC 矩形部材について終局検定（塑性理論式）を一括実行する。
///
/// - `demand_by_elem`: 部材の設計用需要（[`MemberDemand`]：圧縮正の軸力と強軸/弱軸の
///   設計用曲げモーメント）。柱の Mu・軸余裕度・2 軸曲げ余裕度に用いる。該当 ID が無い
///   部材は需要 0（安全側）で評価する。軸力は長期（G+P）静的、曲げ需要は当該組合せの
///   応答値を渡すことを想定する。
/// - 対象外（`RcRect` 以外・断面/材料未解決・Fc 未設定・有効せい ≤ 0）の部材は
///   結果に含めない。
pub fn collect_rc_ultimate_checks(
    model: &Model,
    demand_by_elem: &[(ElemId, MemberDemand)],
    opts: &UltimateShearOptions,
) -> Vec<UltimateCheck> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            continue;
        };
        let demand = demand_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, d)| *d)
            .unwrap_or_default();
        if let Some(check) = check_member(elem, sec, mat, model, demand, opts) {
            out.push(check);
        }
    }
    out
}

// ============================================================================
// CFT 柱の軸終局耐力（RESP-D「06 終局検定」CFT）
// ============================================================================

/// 1 CFT 柱の軸終局検定結果。
#[derive(Clone, Debug)]
pub struct CftUltimateCheck {
    /// 部材 ID。
    pub elem: ElemId,
    /// 柱分類（短柱/中柱/長柱）。
    pub class: CftColumnClass,
    /// 軸圧縮終局耐力 Ncu [N]。
    pub ncu: f64,
    /// 軸引張終局耐力 Ntu [N]。
    pub ntu: f64,
    /// 設計軸力における N-M 相互作用の終局曲げ耐力 Mu [N·mm]
    /// （柱分類に応じて短柱／中柱／長柱の式を用いる）。
    pub mu_nm: f64,
    /// 設計軸力 [N]（圧縮正）。
    pub n_design: f64,
    /// 軸余裕度（圧縮 Ncu/N、引張 Ntu/|N|。N=0 は `f64::INFINITY`）。
    pub axial_margin: f64,
    /// 判定（軸余裕度 ≥ 1.0 で true）。
    pub ok: bool,
    /// 詳細（表示用）。
    pub detail: String,
}

/// CFT 断面（角型/円形）の (円形か, 断面せい D, cA, sA, cI(弱軸), sI(弱軸)) を返す。
fn cft_section_props(shape: &SectionShape) -> Option<(bool, f64, f64, f64, f64, f64)> {
    match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            let ch = (height - 2.0 * thick).max(0.0);
            let cw = (width - 2.0 * thick).max(0.0);
            let s_area = shape.calc_area();
            let c_area = ch * cw;
            // 弱軸（せい/幅の小さい方まわり）の断面二次モーメントを座屈用に採用。
            let s_inertia = shape.calc_iy().min(shape.calc_iz());
            let c_iy = cw * ch.powi(3) / 12.0;
            let c_iz = ch * cw.powi(3) / 12.0;
            let c_inertia = c_iy.min(c_iz);
            let d = height.min(width); // 弱軸方向のせい
            Some((false, d, c_area, s_area, c_inertia, s_inertia))
        }
        SectionShape::CftPipe { outer_dia, thick } => {
            let di = (outer_dia - 2.0 * thick).max(0.0);
            let s_area = shape.calc_area();
            let c_area = std::f64::consts::PI * di * di / 4.0;
            let s_inertia = shape.calc_iy();
            let c_inertia = std::f64::consts::PI * di.powi(4) / 64.0;
            Some((true, outer_dia, c_area, s_area, c_inertia, s_inertia))
        }
        _ => None,
    }
}

/// モデルの CFT 柱（`CftBox`/`CftPipe`）について軸終局検定を一括実行する
/// （RESP-D「06 終局検定」CFT）。
///
/// - `axial_by_elem`: 設計軸力 [N]（**圧縮正**）。無ければ軸力 0（安全側）。
/// - 座屈長さ lk は部材の幾何長（K=1 相当）を用いる。鋼管の降伏強さ Fy は
///   材料名の板厚区分から解決した F 値（解決できなければ 235）、ヤング係数は
///   205000 N/mm²（鋼）を用いる。Fc は材料の `fc`（未設定はスキップ）。
pub fn collect_cft_ultimate_checks(
    model: &Model,
    axial_by_elem: &[(ElemId, f64)],
) -> Vec<CftUltimateCheck> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            continue;
        };
        let Some(shape) = sec.shape.as_ref() else {
            continue;
        };
        let Some((circular, d_section, c_area, s_area, c_inertia, s_inertia)) =
            cft_section_props(shape)
        else {
            continue;
        };
        let Some(fc) = mat.fc.filter(|v| *v > 0.0) else {
            continue;
        };
        let thick = match *shape {
            SectionShape::CftBox { thick, .. } | SectionShape::CftPipe { thick, .. } => thick,
            _ => 0.0,
        };
        let fy = crate::material_strength::steel_f_value_prefix(&mat.name, thick).unwrap_or(235.0);
        let lk = geometric_length(elem, model);

        let inp = cft::CftAxialInput {
            circular,
            d_section,
            c_area,
            s_area,
            c_inertia,
            s_inertia,
            fc,
            fy,
            s_young: 205000.0,
            lk,
        };
        let r = cft_axial_ultimate(&inp);
        let n_design = axial_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, n)| *n)
            .unwrap_or(0.0);

        // N-M 相互作用の終局曲げ耐力 Mu(N)。曲げは強軸（せい方向）で評価する。
        let (bd, bb, bcd, bcb) = match *shape {
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => (height, width, height - 2.0 * thick, width - 2.0 * thick),
            SectionShape::CftPipe { outer_dia, thick } => (
                outer_dia,
                outer_dia,
                outer_dia - 2.0 * thick,
                outer_dia - 2.0 * thick,
            ),
            _ => (d_section, d_section, d_section, d_section),
        };
        let bending = CftBendingInput {
            circular,
            d_steel: bd,
            b_steel: bb,
            c_d: bcd,
            c_b: bcb,
            t: thick,
            fc,
            fy,
        };
        // 短柱は短柱 N-M、中柱・長柱は座屈低減を考慮した中柱・長柱 N-M を用いる。
        let mu_nm = match r.class {
            CftColumnClass::Short => {
                let ncu1 = cft_ncu1(&inp);
                cft_short_column_mu(&bending, n_design, ncu1, r.ntu)
            }
            CftColumnClass::Medium | CftColumnClass::Long => cft_long_medium_column_mu(
                &CftLongMediumInput {
                    bending,
                    is_long: r.class == CftColumnClass::Long,
                    c_ncr: cft_concrete_buckling_axial(c_inertia, c_area, fc, lk),
                    c_lambda1: cft_concrete_slenderness(c_inertia, c_area, fc, lk),
                    nk: cft_nk(c_inertia, s_inertia, 205000.0, fc, lk),
                    ncu_axial: r.ncu,
                    ntu: r.ntu,
                },
                n_design,
            ),
        };

        let axial_margin = if n_design > 0.0 {
            if r.ncu > 0.0 {
                r.ncu / n_design
            } else {
                0.0
            }
        } else if n_design < 0.0 {
            if r.ntu > 0.0 {
                r.ntu / (-n_design)
            } else {
                0.0
            }
        } else {
            f64::INFINITY
        };
        let class_label = match r.class {
            CftColumnClass::Short => "短柱",
            CftColumnClass::Medium => "中柱",
            CftColumnClass::Long => "長柱",
        };
        let detail = format!(
            "分類={class_label}, Ncu={:.0} N, Ntu={:.0} N, Mu(N-M)={:.0} N·mm, N={:.0} N, \
             lk={:.0} mm, cA={:.0} mm², sA={:.0} mm², Fc={:.1}, Fy={:.1}, 軸余裕度={:.3}",
            r.ncu, r.ntu, mu_nm, n_design, lk, c_area, s_area, fc, fy, axial_margin
        );
        out.push(CftUltimateCheck {
            elem: elem.id,
            class: r.class,
            ncu: r.ncu,
            ntu: r.ntu,
            mu_nm,
            n_design,
            axial_margin,
            ok: axial_margin >= 1.0,
            detail,
        });
    }
    out
}

#[cfg(test)]
mod tests;
