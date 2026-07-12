use super::*;
use smallvec::SmallVec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone,
    Section,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

/// テスト用の矩形 RC 断面（b×d, main_x=main_y, 帯筋 D10@pitch）。
fn rc_rect_section(id: u32, b: f64, d: f64, main_dia: f64, main_count: u32, pitch: f64) -> Section {
    let rebar = RcRebar {
        main_x: BarSet {
            count: main_count,
            dia: main_dia,
            layers: 1,
        },
        main_y: BarSet {
            count: main_count,
            dia: main_dia,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch,
            legs: 2,
            grade: None,
        },
    };
    Section {
        id: SectionId(id),
        name: format!("RC{id}"),
        area: b * d,
        iy: b * d.powi(3) / 12.0,
        iz: d * b.powi(3) / 12.0,
        j: 1.0,
        depth: d,
        width: b,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: Some(SectionShape::RcRect { b, d, rebar }),
    }
}

fn material() -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SD345".to_string(),
        young: 21000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: Some(345.0),
    }
}

fn node(id: u32, c: [f64; 3]) -> Node {
    Node {
        id: NodeId(id),
        coord: c,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    }
}

fn frame_element(id: u32, sec: u32, n0: u32, n1: u32) -> ElementData {
    let mut nodes: SmallVec<[NodeId; 8]> = SmallVec::new();
    nodes.push(NodeId(n0));
    nodes.push(NodeId(n1));
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes,
        section: Some(SectionId(sec)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    }
}

/// 1 柱（鉛直）+ 1 梁（水平）のモデル。
fn column_and_beam_model() -> Model {
    let nodes = vec![
        node(0, [0.0, 0.0, 0.0]),
        node(1, [0.0, 0.0, 3000.0]),    // 柱: 鉛直
        node(2, [6000.0, 0.0, 3000.0]), // 梁: 水平
    ];
    let sections = vec![
        rc_rect_section(0, 600.0, 600.0, 25.0, 8, 100.0), // 柱断面
        rc_rect_section(1, 400.0, 700.0, 25.0, 6, 100.0), // 梁断面
    ];
    let materials = vec![material()];
    let elements = vec![
        frame_element(0, 0, 0, 1), // 柱
        frame_element(1, 1, 1, 2), // 梁
    ];
    Model {
        nodes,
        elements,
        sections,
        materials,
        ..Default::default()
    }
}

#[test]
fn test_collect_rc_ultimate_checks_column_and_beam() {
    let model = column_and_beam_model();
    let opts = UltimateShearOptions::default();
    // 柱に圧縮軸力 2000kN。
    let axial = vec![(ElemId(0), MemberDemand::axial(2_000_000.0))];
    let checks = collect_rc_ultimate_checks(&model, &axial, &opts);
    assert_eq!(checks.len(), 2, "柱・梁の 2 部材が検定される");

    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();

    assert_eq!(col.kind, MemberKind::Column);
    assert_eq!(beam.kind, MemberKind::Beam);

    // 各耐力が正。
    assert!(col.mu > 0.0 && col.qmu > 0.0 && col.qsu > 0.0 && col.qbu > 0.0);
    assert!(beam.mu > 0.0 && beam.qmu > 0.0 && beam.qsu > 0.0);

    // 柱は軸終局耐力を持つ。Nuc = 600·600·24。
    let ax = col.axial.expect("柱は軸終局耐力を持つ");
    assert!((ax.nuc - 600.0 * 600.0 * 24.0).abs() < 1e-3);
    assert!(ax.nut < 0.0);
    // 梁は軸終局耐力なし。
    assert!(beam.axial.is_none());

    // せん断余裕度 = Qsu/Qmu。
    assert!((col.shear_margin - col.qsu / col.qmu).abs() < 1e-9);
}

#[test]
fn test_ultimate_check_lightweight_reduces_qsu() {
    let model = column_and_beam_model();
    let std = collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default());
    let lw = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            lightweight: true,
            ..Default::default()
        },
    );
    let col_std = std.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_lw = lw.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!((col_lw.qsu - 0.9 * col_std.qsu).abs() < 1e-3);
    assert!((col_lw.qbu - 0.9 * col_std.qbu).abs() < 1e-3);
}

#[test]
fn test_ultimate_check_skips_non_rc() {
    // 鋼断面（shape=None 相当）は検定対象外。
    let mut model = column_and_beam_model();
    model.sections[0].shape = None;
    let checks = collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default());
    // 柱がスキップされ梁のみ。
    assert_eq!(checks.len(), 1);
    assert_eq!(checks[0].elem, ElemId(1));
}

#[test]
fn test_ultimate_check_include_bond_false() {
    let model = column_and_beam_model();
    let checks = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            include_bond: false,
            ..Default::default()
        },
    );
    for c in &checks {
        assert_eq!(c.qbu, 0.0);
        assert!(c.bond_margin.is_infinite());
    }
}

#[test]
fn test_biaxial_margin_handcalc() {
    // rx=ry=0.5, α=2 → 1/√(0.25+0.25)=1/√0.5=√2。
    let m = biaxial_margin(0.5, 0.5, 2.0);
    assert!((m - 2.0_f64.sqrt()).abs() < 1e-9, "m={m}");
    // 片軸のみ需要（ry=0）→ rx=0.5 の逆数=2.0。
    assert!((biaxial_margin(0.5, 0.0, 2.0) - 2.0).abs() < 1e-9);
    // 需要ゼロ → 無限大。
    assert!(biaxial_margin(0.0, 0.0, 2.0).is_infinite());
    // 相互作用が単位に達する（rx²+ry²=1）と余裕度=1.0。
    assert!((biaxial_margin(0.6, 0.8, 2.0) - 1.0).abs() < 1e-9);
}

/// 柱の 2 軸せん断余裕度オプションが機能し、強軸単独より小さい（不利側）になる。
#[test]
fn test_ultimate_check_biaxial_shear() {
    let model = column_and_beam_model();
    let axial = vec![(ElemId(0), MemberDemand::axial(2_000_000.0))];
    let uni = collect_rc_ultimate_checks(&model, &axial, &UltimateShearOptions::default());
    let bi = collect_rc_ultimate_checks(
        &model,
        &axial,
        &UltimateShearOptions {
            biaxial_shear: true,
            ..Default::default()
        },
    );
    let col_uni = uni.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_bi = bi.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 既定では 2 軸余裕度は None。
    assert!(col_uni.biaxial_shear_margin.is_none());
    // 2 軸指定で Some。両軸の需要を合成するため強軸単独の余裕度以下になる。
    let bm = col_bi.biaxial_shear_margin.expect("2軸指定で Some");
    assert!(
        bm > 0.0 && bm <= col_bi.shear_margin + 1e-9,
        "bm={bm} uni={}",
        col_bi.shear_margin
    );
    // 梁は 2 軸せん断の対象外（None のまま）。
    let beam_bi = bi.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!(beam_bi.biaxial_shear_margin.is_none());
}

/// 柱の 2 軸曲げ余裕度オプションが機能する（設計用曲げ需要を与えたとき Some・正）。
#[test]
fn test_ultimate_check_biaxial_bending() {
    let model = column_and_beam_model();
    // 柱に軸力＋強軸/弱軸の設計用曲げ需要を与える。
    let demand = vec![(
        ElemId(0),
        MemberDemand {
            n_axial: 1_500_000.0,
            mz: 2.0e8,
            my: 1.0e8,
            ..Default::default()
        },
    )];
    let uni = collect_rc_ultimate_checks(&model, &demand, &UltimateShearOptions::default());
    let bi = collect_rc_ultimate_checks(
        &model,
        &demand,
        &UltimateShearOptions {
            biaxial_bending: true,
            ..Default::default()
        },
    );
    let col_uni = uni.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_bi = bi.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 既定では None、指定で Some。
    assert!(col_uni.biaxial_bending_margin.is_none());
    let bm = col_bi.biaxial_bending_margin.expect("2軸曲げ指定で Some");
    assert!(bm > 0.0 && bm.is_finite(), "bm={bm}");
    // 手計算照合: 1/√((Mmx/Mux)²+(Mmy/Muy)²)。Mux=col.mu(強軸)、Muy は弱軸 Mu。
    // 強軸 Mux は col_bi.mu と一致（同一軸力）。弱軸は main_y=main_x なので b↔D 入替のみ。
    // rx=Mmx/Mux>0, ry>0 → bm < min(Mux/Mmx, Muy/Mmy)。
    let rx = 2.0e8 / col_bi.mu;
    assert!(rx > 0.0);
    // 需要 0 なら無限大。
    let zero_demand = vec![(ElemId(0), MemberDemand::axial(1_500_000.0))];
    let z = collect_rc_ultimate_checks(
        &model,
        &zero_demand,
        &UltimateShearOptions {
            biaxial_bending: true,
            ..Default::default()
        },
    );
    let col_z = z.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(col_z.biaxial_bending_margin.unwrap().is_infinite());
    // 梁は対象外。
    let beam_bi = bi.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!(beam_bi.biaxial_bending_margin.is_none());
}

/// 柱の Mu を ACI 規準（平面保持）で算定するオプションが機能する。
#[test]
fn test_ultimate_check_mu_method_aci() {
    let model = column_and_beam_model();
    // 圧縮軸力を与えて柱の Mu を評価（ACI と at 式で共に正、健全域で近い桁）。
    let axial = vec![(ElemId(0), MemberDemand::axial(2_000_000.0))];
    let at = collect_rc_ultimate_checks(&model, &axial, &UltimateShearOptions::default());
    let aci = collect_rc_ultimate_checks(
        &model,
        &axial,
        &UltimateShearOptions {
            mu_method: MuMethod::Aci,
            ..Default::default()
        },
    );
    let col_at = at.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_aci = aci.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(col_aci.mu > 0.0 && col_at.mu > 0.0);
    // 梁は Mu 算定方法の影響を受けない（ACI は柱のみ）。
    let beam_at = at.iter().find(|c| c.elem == ElemId(1)).unwrap();
    let beam_aci = aci.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!((beam_at.mu - beam_aci.mu).abs() < 1e-6);
    // 両手法の柱 Mu は同桁（0.4〜2.5 倍）に収まる。
    let ratio = col_aci.mu / col_at.mu;
    assert!(ratio > 0.4 && ratio < 2.5, "Mu(ACI)/Mu(at)={ratio}");
}

/// 終局せん断強度に靭性指針式 Vu を選択するオプションが機能する。
#[test]
fn test_ultimate_check_shear_method_ductility() {
    let model = column_and_beam_model();
    let plastic = collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default());
    let ductility = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            shear_method: ShearMethod::Ductility,
            ..Default::default()
        },
    );
    // 両手法とも柱・梁の Qsu/Vu は正値（別定式なので値は一般に異なる）。
    for c in &plastic {
        assert!(c.qsu > 0.0, "塑性 Qsu>0: elem={:?}", c.elem);
    }
    let col_p = plastic.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_d = ductility.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(col_d.qsu > 0.0, "靭性 Vu>0");
    assert!(
        (col_p.qsu - col_d.qsu).abs() > 1e-3,
        "塑性 Qsu={} と靭性 Vu={} は一般に異なるはず",
        col_p.qsu,
        col_d.qsu
    );
    // basis 文字列に選択した式名が反映される。
    assert!(col_d.basis.contains("靭性指針式"), "basis={}", col_d.basis);
    assert!(col_p.basis.contains("塑性理論式"), "basis={}", col_p.basis);
    // 付着側も靭性指針式は Vbu（付着考慮せん断信頼強度）を用い、塑性の Qbu と異なる。
    assert!(col_d.qbu > 0.0, "靭性 Vbu>0");
    assert!(col_p.qbu > 0.0, "塑性 Qbu>0");
    assert!(
        (col_p.qbu - col_d.qbu).abs() > 1e-3,
        "塑性 Qbu={} と靭性 Vbu={} は一般に異なるはず",
        col_p.qbu,
        col_d.qbu
    );
}

/// プッシュオーバー応答からの部材別 Rp・設計用せん断力の直接反映が機能する。
#[test]
fn test_ultimate_check_pushover_demand() {
    let model = column_and_beam_model();

    // (1) 設計用せん断力 Qm を直接反映すると Qmu は 2·Mu/内法 ではなく Qm になる。
    let qm = 123_456.0_f64;
    let demand = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.0),
    )];
    let checks = collect_rc_ultimate_checks(&model, &demand, &UltimateShearOptions::default());
    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 上限強度倍率=1.0（既定）なので Qmu = |Qm|。
    assert!(
        (col.qmu - qm).abs() < 1e-3,
        "Qmu={} は応答せん断 Qm={} を反映するはず",
        col.qmu,
        qm
    );

    // (2) 部材別 Rp を上げると（塑性理論式）柱の Qsu は低下する（ν・cotφ 低減）。
    let d_rp0 = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.0),
    )];
    let d_rp3 = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.03),
    )];
    let c0 = collect_rc_ultimate_checks(&model, &d_rp0, &UltimateShearOptions::default());
    let c3 = collect_rc_ultimate_checks(&model, &d_rp3, &UltimateShearOptions::default());
    let q0 = c0.iter().find(|c| c.elem == ElemId(0)).unwrap().qsu;
    let q3 = c3.iter().find(|c| c.elem == ElemId(0)).unwrap().qsu;
    assert!(
        q3 < q0,
        "部材別 Rp=0.03 の Qsu={q3} は Rp=0 の Qsu={q0} より小さいはず"
    );

    // (3) shear/rp 未指定（axial のみ）は従来どおり Qmu=2·Mu/内法（Qm 直接反映なし）。
    let d_axial = vec![(ElemId(0), MemberDemand::axial(1_000_000.0))];
    let ca = collect_rc_ultimate_checks(&model, &d_axial, &UltimateShearOptions::default());
    let col_a = ca.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(
        (col_a.qmu - qm).abs() > 1.0,
        "shear 未指定時は Qmu が応答せん断と一致しないはず（両端ヒンジ略算）"
    );
}

/// 2 軸せん断で弱軸の設計用せん断需要（プッシュオーバー弱軸応答）を直接反映する。
#[test]
fn test_ultimate_check_pushover_weak_shear() {
    let model = column_and_beam_model();
    let opts = UltimateShearOptions {
        biaxial_shear: true,
        ..Default::default()
    };
    // 強軸せん断は共通、弱軸せん断需要のみ大小 2 種。
    let qm = 200_000.0_f64;
    let small = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 0.0, 0.0, qm, 10_000.0, 0.0),
    )];
    let large = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 0.0, 0.0, qm, 400_000.0, 0.0),
    )];
    let cs = collect_rc_ultimate_checks(&model, &small, &opts);
    let cl = collect_rc_ultimate_checks(&model, &large, &opts);
    let ms = cs
        .iter()
        .find(|c| c.elem == ElemId(0))
        .unwrap()
        .biaxial_shear_margin
        .expect("2軸指定で Some");
    let ml = cl
        .iter()
        .find(|c| c.elem == ElemId(0))
        .unwrap()
        .biaxial_shear_margin
        .expect("2軸指定で Some");
    // 弱軸の需要せん断が大きいほど 2 軸せん断余裕度は小さくなる（不利側）。
    assert!(
        ml < ms,
        "弱軸需要大の余裕度 {ml} は弱軸需要小 {ms} より小さいはず"
    );
}

/// CFT 角形柱 1 本のモデルで軸終局検定ドライバが Ncu/Ntu・軸余裕度を算定する。
#[test]
fn test_collect_cft_ultimate_checks() {
    let cft_shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };
    let sec = Section {
        id: SectionId(0),
        name: "CFT400".into(),
        area: cft_shape.calc_area(),
        iy: cft_shape.calc_iy(),
        iz: cft_shape.calc_iz(),
        j: 1.0,
        depth: 400.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: Some(cft_shape),
    };
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "BCR295".to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: Some(30.0),
        fy: None,
    };
    let model = Model {
        nodes: vec![node(0, [0.0, 0.0, 0.0]), node(1, [0.0, 0.0, 3000.0])],
        sections: vec![sec],
        materials: vec![mat],
        elements: vec![frame_element(0, 0, 0, 1)],
        ..Default::default()
    };
    // 圧縮軸力 3000kN。
    let axial = vec![(ElemId(0), 3_000_000.0)];
    let checks = collect_cft_ultimate_checks(&model, &axial);
    assert_eq!(checks.len(), 1);
    let c = &checks[0];
    assert!(c.ncu > 0.0 && c.ntu > 0.0);
    assert!((c.axial_margin - c.ncu / 3_000_000.0).abs() < 1e-6);
    // lk=3000, D=400 → lk/D=7.5 → 中柱。
    assert_eq!(c.class, CftColumnClass::Medium);
    // 短柱 N-M 曲げ耐力 Mu(N) が正（圧縮軸力 3000kN 時）。
    assert!(c.mu_nm > 0.0, "mu_nm={}", c.mu_nm);
}
