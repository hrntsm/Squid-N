//! ST-Bridge 入出力の統合テスト（往復・取り込み報告・断面形状・id 正規化など）。

use super::*;
use smallvec::smallvec;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node, Section,
    Story,
};
use squid_n_core::section_shape::SectionShape;

/// 標準グレード名 `SN400B` の材料（物性は `material_std` の標準表と一致させる）。
fn sn400b(id: u32) -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(id),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }
}

/// 標準往復用の代表モデル（鋼 H 断面・標準グレード材料・階所属節点つき）。
/// ST-Bridge の幾何スコープに収まる要素のみで構成する（材料の E/ν・荷重は対象外）。
fn representative_model() -> Model {
    let mut m = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: if i >= 2 { Some(StoryId(0)) } else { None },
        });
    }
    m.stories.push(Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: StoryId(0),
        name: "1F".into(),
        elevation: 3000.0,
        node_ids: vec![NodeId(2), NodeId(3)],
        diaphragms: vec![],
        seismic_weight: None,
    });
    m.materials.push(sn400b(0));
    // 柱用・梁用で別断面（共有断面の分割を避け、意味的往復を単純化）。
    let col_h = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let beam_h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    m.sections
        .push(col_h.to_section(SectionId(0), "C&1<2".into())); // 名前にエスケープ対象
    m.sections
        .push(beam_h.to_section(SectionId(1), "G1".into()));
    // 柱2本（鉛直, section 0）＋大梁1本（水平, section 1）。
    let mk = |id: u32, ni: u32, nj: u32, sec: u32, refv: [f64; 3]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(ni), NodeId(nj)],
        section: Some(SectionId(sec)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis { ref_vector: refv },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    m.elements.push(mk(0, 0, 2, 0, [1.0, 0.0, 0.0]));
    m.elements.push(mk(1, 1, 3, 0, [1.0, 0.0, 0.0]));
    m.elements.push(mk(2, 2, 3, 1, [0.0, 0.0, 1.0]));
    m
}

/// 2 つの参照ベクトルが（浮動小数の往復誤差を許して）ほぼ一致するか。
fn ref_vec_close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

/// 意味的に一致するか（標準 ST-Bridge の幾何スコープのフィールドのみ）。
/// 材料の E/ν や荷重は ST-Bridge の対象外なので比較しない。
fn assert_semantic_eq(a: &Model, b: &Model) {
    assert_eq!(a.nodes.len(), b.nodes.len(), "node count");
    for (x, y) in a.nodes.iter().zip(&b.nodes) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.coord, y.coord, "coord");
        assert_eq!(x.story, y.story, "story");
    }
    assert_eq!(a.stories.len(), b.stories.len());
    for (x, y) in a.stories.iter().zip(&b.stories) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.name, y.name);
        assert_eq!(x.elevation, y.elevation);
    }
    assert_eq!(a.materials.len(), b.materials.len(), "material count");
    for (x, y) in a.materials.iter().zip(&b.materials) {
        assert_eq!(x.name, y.name, "material grade name");
        assert_eq!(x.young, y.young);
        assert_eq!(x.poisson, y.poisson);
        assert_eq!(x.fy, y.fy);
        assert_eq!(x.fc, y.fc);
    }
    assert_eq!(a.sections.len(), b.sections.len(), "section count");
    for (x, y) in a.sections.iter().zip(&b.sections) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.name, y.name, "section name (escape)");
        assert!((x.area - y.area).abs() < 1e-6, "area");
        assert!((x.iy - y.iy).abs().max((x.iz - y.iz).abs()) < 1.0, "iy/iz");
        assert_eq!(x.depth, y.depth);
        assert_eq!(x.width, y.width);
    }
    assert_eq!(a.elements.len(), b.elements.len());
    for (x, y) in a.elements.iter().zip(&b.elements) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.nodes.as_slice(), y.nodes.as_slice(), "connectivity");
        assert_eq!(x.section, y.section);
        assert_eq!(x.material, y.material);
        assert!(
            ref_vec_close(x.local_axis.ref_vector, y.local_axis.ref_vector),
            "ref_vector {:?} vs {:?}",
            x.local_axis.ref_vector,
            y.local_axis.ref_vector
        );
    }
}

#[test]
fn test_roundtrip_semantic() {
    let m = representative_model();
    let xml = export_stbridge(&m).expect("export");
    let m2 = import_stbridge(&xml).expect("import");
    assert_semantic_eq(&m, &m2);
}

#[test]
fn test_roundtrip_twice_stable() {
    // import→export→再import で安定（DoD §8.3）。
    let m = representative_model();
    let xml1 = export_stbridge(&m).unwrap();
    let m2 = import_stbridge(&xml1).unwrap();
    let xml2 = export_stbridge(&m2).unwrap();
    assert_eq!(xml1, xml2, "export は冪等であるべき");
    let m3 = import_stbridge(&xml2).unwrap();
    assert_semantic_eq(&m2, &m3);
}

#[test]
fn test_column_girder_classification() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbColumn "), "鉛直材は StbColumn");
    assert!(xml.contains("<StbGirder "), "水平材は StbGirder");
}

#[test]
fn test_reject_non_stbridge() {
    let r = import_stbridge("<foo/>");
    assert!(matches!(r, Err(StbError::Version(_))));
}

#[test]
fn test_reject_v1() {
    let r = import_stbridge("<ST_BRIDGE version=\"1.4.0\"><StbModel/></ST_BRIDGE>");
    assert!(matches!(r, Err(StbError::Version(_))));
}

#[test]
fn test_read_stbridge_file_shift_jis() {
    use encoding_rs::SHIFT_JIS;
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    // Shift_JIS には変換できない文字（XML 宣言の UTF-8 等）を避けるため、
    // 日本語を含む注釈を付与した上で Shift_JIS へエンコードする。
    let with_jp = format!("<!-- 柱と梁のモデル -->\n{}", xml);
    let (encoded, _, _) = SHIFT_JIS.encode(&with_jp);

    let dir = std::env::temp_dir().join("squid_n_test_stb_sjis");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("shift_jis.stb");
    std::fs::write(&path, encoded.as_ref()).unwrap();

    let decoded = read_stbridge_file(&path).expect("Shift_JIS デコード");
    let m2 = import_stbridge(&decoded).expect("取り込み");
    assert!(m2.validate().is_ok());
    assert_eq!(m2.nodes.len(), m.nodes.len());
}

#[test]
fn test_read_stbridge_file_utf8_bom() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    let bytes = {
        let mut b = vec![0xEF, 0xBB, 0xBF];
        b.extend_from_slice(xml.as_bytes());
        b
    };
    let dir = std::env::temp_dir().join("squid_n_test_stb_bom");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("utf8_bom.stb");
    std::fs::write(&path, bytes).unwrap();

    let decoded = read_stbridge_file(&path).expect("UTF-8 BOM デコード");
    assert!(decoded.starts_with("<?xml") || decoded.starts_with("<!--"));
    let m2 = import_stbridge(&decoded).expect("取り込み");
    assert!(m2.validate().is_ok());
}

#[test]
fn test_imported_model_validates() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    let m2 = import_stbridge(&xml).unwrap();
    assert!(m2.validate().is_ok(), "取り込んだモデルは検証を通る");
}

use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

fn rebar() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 3,
            dia: 22.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 3,
            dia: 22.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    }
}

fn member(id: u32, kind_col: bool, sec: u32) -> ElementData {
    // kind_col=true は鉛直（柱）、false は水平（梁）になるよう節点を選ぶ。
    let (a, b) = if kind_col {
        (NodeId(0), NodeId(2)) // 鉛直
    } else {
        (NodeId(2), NodeId(3)) // 水平
    };
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![a, b],
        section: Some(SectionId(sec)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    }
}

/// 4 節点だけ持つ骨組（部材・断面は各テストで差し込む）。
fn frame_nodes() -> Model {
    let mut m = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
        });
    }
    m.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    });
    m
}

/// Raw モード（既定）は従来どおり StbSecRaw を出力し、標準要素は出さない。
/// 標準モード: 鋼 H 断面が形鋼ライブラリ参照付きの StbSecColumn_S として出力される。
#[test]
fn test_standard_mode_steel_column() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    m.sections.push(h.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱は StbSecColumn_S");
    assert!(xml.contains("<StbSecSteel>"), "形鋼ライブラリを出す");
    assert!(
        xml.contains("<StbSecRoll-H name=\"H-400x200x8x13\""),
        "H 形鋼図形が定義される: {xml}"
    );
    assert!(
        xml.contains("shape=\"H-400x200x8x13\""),
        "断面が図形名を参照する"
    );
    assert!(
        !xml.contains("<StbSecRaw "),
        "形状がある鋼断面は Raw にしない"
    );
}

/// 標準モード: RC 矩形が梁として使われると StbSecBeam_RC（幾何）で出力される。
#[test]
fn test_standard_mode_rc_beam() {
    let mut m = frame_nodes();
    let rc = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    };
    m.sections.push(rc.to_section(SectionId(0), "G1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecBeam_RC "), "RC 梁は StbSecBeam_RC");
    assert!(
        xml.contains("<StbSecBeam_RC_Straight width=\"400\" depth=\"700\"/>"),
        "矩形図形が幅・せいで出力される: {xml}"
    );
}

/// 標準モード: 柱と梁で共有される鋼断面は 2 要素に分割され、部材の id_section が
/// それぞれ別 id を指す。
#[test]
fn test_standard_mode_shared_section_split() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "S1".into()));
    m.elements.push(member(0, true, 0)); // 柱が section 0 を使用
    m.elements.push(member(1, false, 0)); // 梁も section 0 を使用（共有）

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "柱用に StbSecColumn_S");
    assert!(xml.contains("<StbSecBeam_S "), "梁用に StbSecBeam_S");
    // 形鋼図形は 1 つに重複排除される。
    assert_eq!(
        xml.matches("<StbSecRoll-H ").count(),
        1,
        "形鋼図形は重複排除される"
    );
    // id は 1 始まり（positiveInteger）。柱は id_section=1、梁は分割された新 id=2 を参照。
    assert!(
        xml.contains("<StbColumn ") && xml.contains("id_section=\"1\""),
        "柱は元の断面 id を参照: {xml}"
    );
    assert!(
        xml.contains("<StbGirder ") && xml.contains("id_section=\"2\""),
        "梁は分割された新しい断面 id を参照: {xml}"
    );
}

/// 標準モード: 形状を持たない断面（SRC/CFT/未定義含む）は StbSecRaw へフォールバックする。
#[test]
fn test_standard_mode_fallback_raw_for_shapeless() {
    let mut m = frame_nodes();
    m.sections.push(Section {
        id: SectionId(0),
        name: "X1".into(),
        area: 1.0e4,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRaw "),
        "形状の無い断面は Raw にフォールバック"
    );
}

/// 標準モードで書き出したファイルを import で読み戻せる（往復）。
/// 鋼 H（柱）＋ RC 矩形（梁）が形状・断面性能とも復元され、検証を通る。
#[test]
fn test_standard_import_roundtrip_steel_and_rc() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    m.sections.push(h.to_section(SectionId(0), "C1".into()));
    let rc = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    };
    m.sections.push(rc.to_section(SectionId(1), "G1".into()));
    m.elements.push(member(0, true, 0)); // 柱 → 鋼断面
    m.elements.push(member(1, false, 1)); // 梁 → RC 断面

    let xml = export_stbridge(&m).unwrap();
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());

    assert_eq!(back.sections.len(), 2);
    assert!(
        matches!(back.sections[0].shape, Some(SectionShape::SteelH { .. })),
        "鋼柱断面の形状が復元される: {:?}",
        back.sections[0].shape
    );
    assert!(
        matches!(back.sections[1].shape, Some(SectionShape::RcRect { .. })),
        "RC 梁断面の形状が復元される: {:?}",
        back.sections[1].shape
    );
    // 断面性能（弾性）は形状から再算定され、元と一致する。
    assert_eq!(back.sections[0].area, m.sections[0].area);
    assert_eq!(back.sections[0].iy, m.sections[0].iy);
    assert_eq!(back.sections[0].iz, m.sections[0].iz);
    assert_eq!(back.sections[1].area, m.sections[1].area);
    assert_eq!(back.sections[1].iy, m.sections[1].iy);
    // 部材の断面参照が正しく張り替わる。
    assert_eq!(back.elements[0].section, Some(SectionId(0)));
    assert_eq!(back.elements[1].section, Some(SectionId(1)));
}

/// 方向別に異なる本数・径・段数・かぶり・帯筋を持つ配筋（往復ずれ・取り違えを検出）。
/// 標準 ST-Bridge が保存できる配筋（主筋本数は X/Y で別、径は単一 `D_main`、1 段）。
/// ST-Bridge の主筋径は `D_main` 1 つ・段別本数のみのため、X/Y で径を変えたり多段に
/// したりは標準では往復しない（[`super`] モジュールドキュメント参照）。
fn rebar_distinct() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 3,
            dia: 25.0,
            layers: 1,
        },
        cover: 45.0,
        shear: ShearBar {
            dia: 13.0,
            pitch: 150.0,
            legs: 4,
            grade: Some("KH785".into()),
        },
    }
}

/// 標準モード: RC 矩形柱の配筋（主筋・帯筋・かぶり）が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_rect_column_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 600.0,
        d: 700.0,
        rebar: rebar_distinct(),
    };
    m.sections.push(shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarArrangementColumn_RC "),
        "柱配筋要素が書き出される: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    // 形状（b・d・配筋すべて）が完全一致で復元される。
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 矩形柱の配筋が往復で保存される"
    );
}

/// 標準モード: RC 円形柱の配筋が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_circle_column_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcCircle {
        d: 800.0,
        rebar: rebar_distinct(),
    };
    m.sections.push(shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarColumn_RC_CircleSame "),
        "円形配筋要素: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 円形柱の配筋が往復で保存される"
    );
}

/// 標準モード: RC 矩形梁の配筋が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_beam_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar_distinct(),
    };
    m.sections.push(shape.to_section(SectionId(0), "G1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarArrangementBeam_RC "),
        "梁配筋要素が書き出される: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 梁の配筋が往復で保存される"
    );
}

/// file id が重複する STB は fail-loud でエラーにする（無言のジオメトリ破損防止）。
/// 重複 id があると「配列添字 == id.index()」の不変条件が壊れ、部材が別実体の
/// 節点を参照してしまうため、取り込み時に検出してエラーとする。
#[test]
fn test_import_duplicate_node_id_is_error() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="0" X="1000" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
</StbModel></ST_BRIDGE>"#;
    let r = import_stbridge(xml);
    assert!(
        r.is_err(),
        "重複 file id はエラーにすべき（無言のジオメトリ破損を防ぐ）"
    );
}

/// 配筋要素の無い（幾何のみの）RC 断面ファイルも、無筋相当の既定配筋で読める。
#[test]
fn test_import_rc_without_bar_arrangement_uses_default() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C"><StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="500" width_Y="600"/></StbSecFigureColumn_RC></StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(m.sections.len(), 1);
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { b, d, .. }) => {
            assert_eq!(*b, 500.0);
            assert_eq!(*d, 600.0);
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 標準モードで柱・梁に分割された共有鋼断面が、import で 2 断面として復元され
/// 各部材が別 id を参照する（検証を通る）。
#[test]
fn test_standard_import_recovers_split_shared_section() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "S1".into()));
    m.elements.push(member(0, true, 0)); // 柱
    m.elements.push(member(1, false, 0)); // 梁（同じ断面を共有）

    let xml = export_stbridge(&m).unwrap();
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections.len(), 2, "共有断面は柱用・梁用に分割される");
    assert!(
        back.sections
            .iter()
            .all(|s| matches!(s.shape, Some(SectionShape::SteelH { .. }))),
        "両断面とも H 形鋼として復元される"
    );
    assert_eq!(back.elements[0].section, Some(SectionId(0)));
    assert_eq!(back.elements[1].section, Some(SectionId(1)));
}

/// 柱・梁で共有する RC 矩形断面が、配筋ごと 2 断面へ分割・復元される。
#[test]
fn test_standard_roundtrip_shared_rc_rect_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 800.0,
        rebar: rebar_distinct(),
    };
    m.sections
        .push(shape.to_section(SectionId(0), "RC1".into()));
    m.elements.push(member(0, true, 0)); // 柱
    m.elements.push(member(1, false, 0)); // 梁（共有）

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections.len(), 2, "共有 RC 断面は 2 断面へ分割される");
    // 分割された両断面とも、元の形状・配筋が保存されている。
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
    assert_eq!(back.sections[1].shape, m.sections[0].shape);
}

/// grade=None の配筋も完全一致で往復する（strength_band 属性を出力しない経路）。
#[test]
fn test_standard_roundtrip_rc_rebar_grade_none() {
    let mut m = frame_nodes();
    let mut r = rebar_distinct();
    r.shear.grade = None;
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: r,
    };
    m.sections.push(shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
}

/// 非整数の径・ピッチ・かぶりも桁落ちなく往復する。
#[test]
fn test_standard_roundtrip_rc_rebar_non_integer() {
    let mut m = frame_nodes();
    // 主筋径は単一 `D_main`・1 段のみ標準往復する（X/Y で径・段数は変えない）。
    let r = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 12.7,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 12.7,
            layers: 1,
        },
        cover: 40.5,
        shear: ShearBar {
            dia: 6.35,
            pitch: 133.3,
            legs: 2,
            grade: Some("SD295".into()),
        },
    };
    let shape = SectionShape::RcRect {
        b: 450.0,
        d: 650.0,
        rebar: r,
    };
    m.sections.push(shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
}

/// 帯筋グレードにタブ等の制御空白が含まれても往復で保存される（esc の制御文字対策）。
#[test]
fn test_standard_roundtrip_rc_rebar_grade_with_control_chars() {
    let mut m = frame_nodes();
    let mut r = rebar_distinct();
    r.shear.grade = Some("KH\t785\nX".into());
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: r,
    };
    m.sections.push(shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "制御空白を含む grade が往復で保存される"
    );
}

/// 円形 RC を梁に使うと（ST-Bridge に円形梁図形が無いため）StbSecRaw へフォールバックし、
/// 形状・配筋は失われるが物性は残り、検証は通る（ドキュメント化された既知の挙動）。
#[test]
fn test_standard_rc_circle_beam_falls_back_to_raw() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcCircle {
        d: 700.0,
        rebar: rebar_distinct(),
    };
    m.sections
        .push(shape.to_section(SectionId(0), "CB1".into()));
    m.elements.push(member(0, false, 0)); // 梁（水平材）で円形を使う

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecRaw "), "円形梁は Raw にフォールバック");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    // 形状・配筋は失われる（shape=None）が、弾性物性は残る。
    assert!(back.sections[0].shape.is_none(), "円形梁は形状が往復しない");
    assert_eq!(back.sections[0].area, m.sections[0].area, "物性は残る");
}

/// 実 ST-Bridge 風の配筋属性（呼び名径 D22・標準名 D_band/N_main_X_1st）を best-effort で読む。
#[test]
fn test_import_rc_rebar_third_party_names() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C">
      <StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="600" width_Y="600"/></StbSecFigureColumn_RC>
      <StbSecBarArrangementColumn_RC>
        <StbSecBarColumn_RC_RectSame N_main_X_1st="4" N_main_Y_1st="3" D_main="D22" D_band="D10" pitch_band="100"/>
      </StbSecBarArrangementColumn_RC>
    </StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { rebar, .. }) => {
            assert_eq!(rebar.main_x.count, 4);
            assert_eq!(rebar.main_y.count, 3);
            assert_eq!(rebar.main_x.dia, 22.0, "呼び名 D22 → 22mm");
            assert_eq!(rebar.shear.dia, 10.0, "呼び名 D10 → 10mm");
            assert_eq!(rebar.shear.pitch, 100.0);
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の段別主筋本数（`N_main_X_1st`/`_2nd`、梁の `N_main_bottom`/`_2nd`）を
/// 合算し、非ゼロの段数を `layers` に反映することを確認する。
#[test]
fn test_import_rc_rebar_layered_counts() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C">
      <StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="700" width_Y="700"/></StbSecFigureColumn_RC>
      <StbSecBarArrangementColumn_RC>
        <StbSecBarColumn_RC_RectSame N_main_X_1st="4" N_main_X_2nd="3" N_main_Y_1st="5" D_main="D25" D_band="D13" pitch_band="100"/>
      </StbSecBarArrangementColumn_RC>
    </StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { rebar, .. }) => {
            assert_eq!(rebar.main_x.count, 7, "X 方向は 1・2 段目を合算 (4+3)");
            assert_eq!(rebar.main_x.layers, 2, "非ゼロの段数 = 2");
            assert_eq!(rebar.main_y.count, 5, "Y 方向は 1 段目のみ");
            assert_eq!(rebar.main_y.layers, 1, "非ゼロの段数 = 1");
            assert_eq!(rebar.main_x.dia, 25.0, "呼び名 D25 → 25mm");
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の鋼管形鋼ライブラリ名（`StbSecRoll-Pipe`）を取り込み、鋼管柱の
/// 断面性能（物性ゼロでない）を復元できることを確認する。Squid 方言（`StbSecPipe`）
/// だけでなく標準名も受けることの回帰テスト。
#[test]
fn test_import_steel_roll_pipe_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="P1">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="P-267.4x6" strength_main="STKN400B"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-Pipe name="P-267.4x6" D="267.4" t="6"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    // 形鋼参照が解決され、物性ゼロの警告が出ていないこと。
    assert!(
        report.warnings.iter().all(|w| !w.contains("物性ゼロ")),
        "鋼管の形鋼参照が解決されるべき: {:?}",
        report.warnings
    );
    let sec = &m.sections[0];
    assert!(
        sec.area > 0.0,
        "鋼管断面の断面積が復元される: A={}",
        sec.area
    );
    match &sec.shape {
        Some(SectionShape::SteelPipe { outer_dia, thick }) => {
            assert_eq!(*outer_dia, 267.4);
            assert_eq!(*thick, 6.0);
        }
        other => panic!("SteelPipe を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の階所属（`StbStory` 直下 `StbNodeIdList/StbNodeId`）を取り込み、
/// 節点の `story` と `Story.node_ids` の双方へ反映することを確認する。
#[test]
fn test_import_story_node_list() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="0" Y="0" Z="3000"/>
    <StbNode id="3" X="4000" Y="0" Z="3000"/>
  </StbNodes>
  <StbStories>
    <StbStory id="0" name="1F" height="0"/>
    <StbStory id="1" name="2F" height="3000">
      <StbNodeIdList>
        <StbNodeId id="2"/>
        <StbNodeId id="3"/>
      </StbNodeIdList>
    </StbStory>
  </StbStories>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    // 節点 2・3 は 2F（StoryId(1)）に所属し、0・1 はいずれの階にも属さない。
    assert_eq!(m.nodes[2].story, Some(StoryId(1)), "節点2 → 2F");
    assert_eq!(m.nodes[3].story, Some(StoryId(1)), "節点3 → 2F");
    assert_eq!(m.nodes[0].story, None, "節点0 は階リスト外");
    // Story.node_ids へも反映される。
    assert_eq!(
        m.stories[1].node_ids,
        vec![NodeId(2), NodeId(3)],
        "2F の所属節点"
    );
    assert!(m.stories[0].node_ids.is_empty(), "1F は所属節点なし");
}

/// 標準モード: 平鋼（中実矩形）が `StbSecColumn_S`＋`StbSecRoll-FlatBar` として往復する。
#[test]
fn test_standard_roundtrip_flat_bar() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelFlatBar {
        width: 100.0,
        thick: 12.0,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "FB1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱要素: {xml}");
    assert!(xml.contains("<StbSecRoll-FlatBar "), "平鋼の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections[0].shape, m.sections[0].shape, "平鋼が往復");
    // 断面性能（中実矩形）が算定されている。
    assert!((back.sections[0].area - 1200.0).abs() < 1e-6, "A=width·t");
}

/// 標準モード: 中実丸鋼が `StbSecColumn_S`＋`StbSecRoll-RoundBar` として往復する。
#[test]
fn test_standard_roundtrip_round_bar() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelRoundBar { dia: 32.0 };
    m.sections
        .push(shape.to_section(SectionId(0), "RB1".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRoll-RoundBar "),
        "中実丸鋼の形鋼ライブラリ"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "中実丸鋼が往復"
    );
}

/// import: 実 ST-Bridge の平鋼・丸鋼ライブラリ名を直接読み取れる。
#[test]
fn test_import_flat_and_round_bar_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="FB">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="FB-90x9"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecColumn_S id="1" name="RB">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="RB-25"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-FlatBar name="FB-90x9" B="90" t="9"/>
      <StbSecRoll-RoundBar name="RB-25" D="25"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    let shapes: Vec<_> = m.sections.iter().map(|s| s.shape.clone()).collect();
    assert!(
        shapes.contains(&Some(SectionShape::SteelFlatBar {
            width: 90.0,
            thick: 9.0
        })),
        "平鋼が復元される: {shapes:?}"
    );
    assert!(
        shapes.contains(&Some(SectionShape::SteelRoundBar { dia: 25.0 })),
        "中実丸鋼が復元される: {shapes:?}"
    );
}

/// 標準モード: リップ溝形が `StbSecColumn_S`＋`StbSecRoll-LipC` として往復する。
#[test]
fn test_standard_roundtrip_lip_channel() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelLipChannel {
        height: 150.0,
        width: 75.0,
        lip: 20.0,
        thick: 2.3,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "LipC1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRoll-LipC "),
        "リップ溝形の形鋼ライブラリ: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "リップ溝形が往復"
    );
}

/// import: 実 ST-Bridge のリップ溝形ライブラリ名（`StbSecRoll-LipC`）を直接読み取れる。
#[test]
fn test_import_lip_channel_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="LC">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="LipC-200x75x20x3.2"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-LipC name="LipC-200x75x20x3.2" A="200" B="75" C="20" t="3.2"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report.warnings.iter().all(|w| !w.contains("物性ゼロ")),
        "リップ溝形の形鋼参照が解決されるべき: {:?}",
        report.warnings
    );
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelLipChannel {
            height: 200.0,
            width: 75.0,
            lip: 20.0,
            thick: 3.2
        }),
        "リップ溝形が復元される"
    );
    assert!(m.sections[0].area > 0.0);
}

/// 標準モード: 非対称組立 H が `StbSecBuild-H`（下フランジ方言属性付き）として往復する。
#[test]
fn test_standard_roundtrip_built_h() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelBuiltH {
        height: 500.0,
        upper_width: 150.0,
        upper_thick: 9.0,
        lower_width: 300.0,
        lower_thick: 19.0,
        web_thick: 9.0,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "BH1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBuild-H "),
        "組立 H の形鋼ライブラリ: {xml}"
    );
    assert!(xml.contains("B2="), "下フランジの方言属性が付く");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "非対称組立 H が完全往復"
    );
}

/// import: `StbSecBuild-H`（下フランジ属性なし＝第三者の対称 H）は `SteelH` として読む。
#[test]
fn test_import_symmetric_build_h_is_steel_h() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="BH">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="BH-400"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecBuild-H name="BH-400" A="400" B="200" t1="8" t2="12"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 12.0
        }),
        "下フランジ属性が無ければ対称 H"
    );
}

/// 標準モード: 角形鋼管柱の角部外半径 r（`StbSecRoll-BOX` の r 属性）が
/// `SectionShape::SteelBox.corner_r` として完全往復する。
#[test]
fn test_standard_roundtrip_steel_box_corner_r() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
        corner_r: 30.0,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "BOX1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("r=\"30\""), "角部外半径 r が出力される: {xml}");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "角形鋼管の角部外半径 r が完全往復"
    );
}

/// import: `r` 属性が無い `StbSecRoll-BOX` は角部直角（corner_r=0.0）として読む。
#[test]
fn test_import_box_without_r_attr_is_corner_r_zero() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="BOX">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="BOX-300"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-BOX name="BOX-300" type="ELSE" A="300" B="300" t="12"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelBox {
            height: 300.0,
            width: 300.0,
            thick: 12.0,
            corner_r: 0.0,
        }),
        "r 属性が無ければ角部直角（corner_r=0.0）"
    );
}

/// 標準モード: CFT 角形柱が `StbSecColumn_CFT`＋形鋼ライブラリとして往復する。
#[test]
fn test_standard_roundtrip_cft_box() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 16.0,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "CFT1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_CFT "), "CFT 柱要素: {xml}");
    assert!(xml.contains("<StbSecRoll-BOX "), "充填鋼管の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "CFT 角形が往復"
    );
}

/// 標準モード: CFT 円形柱が往復する。
#[test]
fn test_standard_roundtrip_cft_pipe() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftPipe {
        outer_dia: 500.0,
        thick: 12.0,
    };
    m.sections
        .push(shape.to_section(SectionId(0), "CFT2".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_CFT "));
    assert!(xml.contains("<StbSecPipe "));
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "CFT 円形が往復"
    );
}

/// 標準モード: SRC 柱（コンクリート＋内蔵鉄骨＋配筋＋鋼種）が完全に往復する。
#[test]
fn test_standard_roundtrip_src_column() {
    let mut m = frame_nodes();
    let shape = SectionShape::SrcRect {
        b: 800.0,
        d: 800.0,
        rebar: rebar_distinct(),
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 8.0,
        steel_flange_thick: 13.0,
        steel_grade: "SN490B".into(),
    };
    m.sections
        .push(shape.to_section(SectionId(0), "SRC1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_SRC "), "SRC 柱要素: {xml}");
    assert!(
        xml.contains("strength_steel=\"SN490B\""),
        "鋼種が書き出される"
    );
    assert!(xml.contains("<StbSecRoll-H "), "内蔵鉄骨の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "SRC 柱が形状・配筋・内蔵鉄骨・鋼種とも往復する"
    );
}

/// 標準モード: SRC 梁も往復する（`StbSecBeam_SRC`）。
#[test]
fn test_standard_roundtrip_src_beam() {
    let mut m = frame_nodes();
    let shape = SectionShape::SrcRect {
        b: 500.0,
        d: 800.0,
        rebar: rebar_distinct(),
        steel_height: 450.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 14.0,
        steel_grade: "SN400B".into(),
    };
    m.sections
        .push(shape.to_section(SectionId(0), "SG1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecBeam_SRC "), "SRC 梁要素: {xml}");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections[0].shape, m.sections[0].shape, "SRC 梁が往復");
}

/// CFT を梁に使うと（ST-Bridge に CFT 梁が無いため）Raw へフォールバックする。
#[test]
fn test_standard_cft_beam_falls_back_to_raw() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
    };
    m.sections.push(shape.to_section(SectionId(0), "CB".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecRaw "), "CFT 梁は Raw にフォールバック");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert!(back.sections[0].shape.is_none());
}

/// 形鋼ライブラリが断面要素より後ろに現れても解決できる（順序非依存）。
#[test]
fn test_standard_import_steel_library_order_independent() {
    // export は StbSecSteel を末尾に書き出す。これを import できることを確認する。
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 350.0,
        width: 175.0,
        web_thick: 7.0,
        flange_thick: 11.0,
    };
    m.sections.push(h.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    // 形鋼ライブラリが断面要素の後ろにあること（前提の確認）。
    let steel_pos = xml.find("<StbSecSteel>").unwrap();
    let col_pos = xml.find("<StbSecColumn_S").unwrap();
    assert!(col_pos < steel_pos, "前提: 断面要素 → 形鋼ライブラリの順");

    let back = import_stbridge(&xml).expect("import");
    assert!(
        matches!(back.sections[0].shape, Some(SectionShape::SteelH { .. })),
        "後方の形鋼ライブラリを解決して形状復元"
    );
    assert_eq!(back.sections[0].area, m.sections[0].area);
}

/// 他社ファイルでよくある 1 始まり・非連番の id（node/material/section/member）を
/// 0 始まり連番へ正規化し、参照を張り替えて検証を通す。
#[test]
fn test_import_normalizes_noncontiguous_ids() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="11" X="0" Y="0" Z="0"/>
    <StbNode id="12" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="5" name="SN400B" young="205000" poisson="0.3" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_S id="9" name="C1"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbColumn id="7" id_node_bottom="11" id_node_top="12" id_section="9" id_material="5" rx="0" ry="1" rz="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(
        m.validate().is_ok(),
        "非連番 id を正規化して検証を通る: {:?}",
        m.validate()
    );
    assert_eq!(m.nodes.len(), 2);
    assert_eq!(m.nodes[0].id, NodeId(0));
    assert_eq!(m.nodes[1].id, NodeId(1));
    assert_eq!(m.materials[0].id, MaterialId(0));
    assert_eq!(m.sections[0].id, SectionId(0));
    assert_eq!(m.elements[0].id, ElemId(0));
    // 参照が正規化後の index に張り替わっている。
    assert_eq!(m.elements[0].nodes.as_slice(), &[NodeId(0), NodeId(1)]);
    assert_eq!(m.elements[0].section, Some(SectionId(0)));
    assert_eq!(m.elements[0].material, Some(MaterialId(0)));
    assert!(matches!(
        m.sections[0].shape,
        Some(SectionShape::SteelH { .. })
    ));
}

/// ST-Bridge 標準の属性名（大文字 X/Y/Z 座標）の節点も読める。
#[test]
fn test_import_accepts_uppercase_coordinate_attrs() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes><StbNode id="0" X="1000" Y="2000" Z="3000"/></StbNodes>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(m.nodes.len(), 1);
    assert_eq!(m.nodes[0].coord, [1000.0, 2000.0, 3000.0]);
}

/// ブレース（斜材）が `StbBrace` として往復する（Raw / Standard 両モード）。
#[test]
fn test_roundtrip_brace() {
    let mut m = frame_nodes();
    let pipe = SectionShape::SteelPipe {
        outer_dia: 100.0,
        thick: 5.0,
    };
    m.sections.push(pipe.to_section(SectionId(0), "BR".into()));
    // 節点0→3 の斜材（引張専用）。
    m.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Brace { tension_only: true },
        nodes: smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let raw_xml = export_stbridge(&m).unwrap();
    assert!(
        raw_xml.contains("<StbBrace "),
        "ブレースは StbBrace で書き出される"
    );
    for xml in [raw_xml, export_stbridge(&m).unwrap()] {
        let back = import_stbridge(&xml).expect("import");
        assert!(back.validate().is_ok(), "{:?}", back.validate());
        assert_eq!(back.elements.len(), 1);
        assert_eq!(
            back.elements[0].kind,
            ElementKind::Brace { tension_only: true },
            "ブレース種別（tension_only 含む）が往復する"
        );
        assert_eq!(back.elements[0].nodes.as_slice(), &[NodeId(0), NodeId(3)]);
        assert_eq!(back.elements[0].section, Some(SectionId(0)));
        assert_eq!(back.elements[0].material, Some(MaterialId(0)));
    }
}

/// 標準書き出しは断面側にグレード名で材料を付す（鋼は strength_main、RC は strength_concrete）。
#[test]
fn test_standard_writes_section_material() {
    // 鋼柱: strength_main に材料名（グレード）。
    let mut m = frame_nodes(); // 材料 0 = "SN400B"
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "C".into()));
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("strength_main=\"SN400B\""),
        "鋼断面に材料名（strength_main）を付す: {xml}"
    );

    // RC 柱: strength_concrete にコンクリートのグレード名（id は 1 始まり）。
    let mut m2 = frame_nodes();
    let rc = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: rebar(),
    };
    m2.sections.push(rc.to_section(SectionId(0), "C".into()));
    m2.elements.push(member(0, true, 0));
    let xml2 = export_stbridge(&m2).unwrap();
    assert!(
        xml2.contains("<StbSecColumn_RC id=\"1\" name=\"C\" strength_concrete=\"SN400B\""),
        "RC 断面にコンクリートのグレード名を付す: {xml2}"
    );
}

/// 実 STB 相当: 部材が id_material を持たず断面が鋼種（strength_main）を持つファイルで、
/// 断面の材料を部材へ伝播する（材料名で突き合わせ）。
#[test]
fn test_import_propagates_steel_grade_material_to_member() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="0" name="SN400B" young="205000" poisson="0.3" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_S id="0" name="C"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1" strength_main="SN400B"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.elements[0].material,
        Some(MaterialId(0)),
        "部材が id_material を持たなくても断面の鋼種から材料が伝播する"
    );
}

/// 実 STB 相当: RC 断面の id_material を（id_material 無しの）部材へ伝播する。
#[test]
fn test_import_propagates_rc_material_to_member() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="5" name="Fc24" young="21000" poisson="0.2" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_RC id="0" name="C" id_material="5"><StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="500" width_Y="500"/></StbSecFigureColumn_RC></StbSecColumn_RC>
  </StbSections>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    // 材料 id=5 は正規化で index 0 になる。
    assert_eq!(
        m.elements[0].material,
        Some(MaterialId(0)),
        "断面の id_material が部材へ伝播する"
    );
}

/// 対応範囲内のファイルは取り込み報告がクリーン（欠落なし）。
#[test]
fn test_import_report_clean_for_supported_model() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "C".into()));
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    let (_m, report) = import_stbridge_with_report(&xml).expect("import");
    assert!(
        report.is_clean(),
        "対応範囲のモデルは警告なし: {:?}",
        report.warnings
    );
}

/// 未対応要素（基礎・杭）は警告として報告され、無言で欠落しない。
#[test]
fn test_import_report_lists_unsupported_elements() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1"/>
    <StbFooting id="1" name="F1"/>
    <StbFooting id="2" name="F2"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.elements.len(), 1, "対応する柱のみ取り込む");
    assert!(!report.is_clean());
    let joined = report.warnings.join(" | ");
    assert!(
        joined.contains("StbFooting×2"),
        "基礎2件の欠落を報告: {joined}"
    );
}

/// 明示リストに無い未知の部材・断面・荷重要素も「取り込み対象外」として通知される
/// （fail-loud）。一方、形鋼ライブラリのコンテナ StbSecSteel は誤検出しない。
#[test]
fn test_import_report_unknown_elements_are_reported() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
    <StbSecFutureThing id="1" name="X"/>
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="1" id_section="0" kind_structure="S"/>
      <StbNovelMember id="1"/>
    </StbColumns>
  </StbMembers>
  <StbLoads>
    <StbLoadCase id="0" name="L1">
      <StbNodalLoad id_node="1" fz="-5"/>
      <StbLoadMember id="0"/>
    </StbLoadCase>
  </StbLoads>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    let joined = report.warnings.join(" | ");
    // 未知の部材・断面・荷重が名指しで通知される。
    assert!(joined.contains("StbNovelMember×1"), "未知の部材: {joined}");
    assert!(
        joined.contains("StbSecFutureThing×1"),
        "未知の断面: {joined}"
    );
    assert!(joined.contains("StbLoadMember×1"), "未対応の荷重: {joined}");
    // 形鋼ライブラリのコンテナは誤検出しない。
    assert!(
        !joined.contains("StbSecSteel×"),
        "コンテナは誤検出しない: {joined}"
    );
}

/// StbSlab（境界節点ループ StbNodeIdOrder）と StbSecSlab_RC（厚さ）を取り込む。
#[test]
fn test_import_slab_with_node_order_and_thickness() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecSlab_RC id="7" name="S1">
      <StbSecFigureSlab_RC>
        <StbSecSlab_RC_Straight thickness="180"/>
      </StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbSlab id="0" name="S1" id_section="7" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1, "スラブを1件取り込む");
    let s = &m.slabs[0];
    assert_eq!(
        s.boundary,
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界節点ループが順序どおり"
    );
    assert_eq!(s.thickness, Some(180.0), "断面参照から厚さを解決");
    assert!(report.is_clean(), "警告なし: {:?}", report.warnings);
}

/// StbNodeIdOrder が CDATA 形式でも境界を取り込めること。
#[test]
fn test_import_slab_node_order_cdata() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S1" kind_structure="RC">
      <StbNodeIdOrder><![CDATA[0 1 2 3]]></StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1, "CDATA の節点ループを取り込む");
    assert_eq!(
        m.slabs[0].boundary,
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)]
    );
}

/// 自己終了 <StbNodeIdOrder/> の後に無関係な子要素のテキストがあっても、
/// 取り込み窓が閉じられて境界へ誤混入しないこと（レビュー指摘の回帰テスト）。
#[test]
fn test_import_slab_self_closing_node_order_does_not_capture_stray_text() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S1" kind_structure="RC">
      <StbNodeIdOrder/>
      <Foo>999</Foo>
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1);
    // 999 が混入せず、実 StbNodeIdOrder の 0 1 2 3 のみになる。
    assert_eq!(
        m.slabs[0].boundary,
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "自己終了タグ後の無関係テキストを取り込まない"
    );
}

/// StbWall（境界節点ループ）と StbSecWall_RC（厚さ）を壁要素として取り込む。
#[test]
fn test_import_wall_with_node_order_and_thickness() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="0" Z="3000"/>
    <StbNode id="3" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecWall_RC id="9" name="W1">
      <StbSecFigureWall_RC>
        <StbSecWall_RC_Straight thickness="200"/>
      </StbSecFigureWall_RC>
    </StbSecWall_RC>
  </StbSections>
  <StbMembers>
    <StbWall id="0" name="W1" id_section="9" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbWall>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    let walls: Vec<_> = m
        .elements
        .iter()
        .filter(|e| e.kind == squid_n_core::model::ElementKind::Wall)
        .collect();
    assert_eq!(walls.len(), 1, "壁を1件取り込む");
    let w = walls[0];
    assert_eq!(
        w.nodes.as_slice(),
        &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界節点ループが順序どおり"
    );
    let sec = w.section.and_then(|s| m.sections.get(s.index()));
    assert_eq!(
        sec.and_then(|s| s.thickness),
        Some(200.0),
        "壁断面の厚さを解決"
    );
    assert!(report.is_clean(), "警告なし: {:?}", report.warnings);
}

/// 自己終了 <StbSlab/> の後の StbWall の節点ループが、陳腐化したスラブ状態に
/// 取り込まれず正しく壁へ入ること（レビュー指摘の回帰テスト）。
#[test]
fn test_self_closing_slab_does_not_steal_wall_nodes() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="0" Z="3000"/>
    <StbNode id="3" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S0" id_section="1"/>
    <StbWall id="1" name="W1" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbWall>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    let walls: Vec<_> = m
        .elements
        .iter()
        .filter(|e| e.kind == squid_n_core::model::ElementKind::Wall)
        .collect();
    assert_eq!(walls.len(), 1, "壁が取り込まれる（節点を横取りされない）");
    assert_eq!(
        walls[0].nodes.as_slice(),
        &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)]
    );
}

/// 壁（境界＋厚さ）を含むモデルが export→import で往復すること。
#[test]
fn test_wall_roundtrip_export_import() {
    use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};
    let mut model = Model::default();
    for (i, (x, z)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
        .into_iter()
        .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, 0.0, z],
            restraint: Default::default(),
            mass: None,
            story: None,
        });
    }
    // 厚さ 250 の壁断面と、それを参照する壁要素。
    model.sections.push(squid_n_core::model::Section {
        id: SectionId(0),
        name: "W".into(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: Some(250.0),
        shape: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(0)),
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let xml = export_stbridge(&model).expect("export");
    let (m2, _report) = import_stbridge_with_report(&xml).expect("import");
    assert!(m2.validate().is_ok(), "{:?}", m2.validate());
    let walls: Vec<_> = m2
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Wall)
        .collect();
    assert_eq!(walls.len(), 1, "壁1件");
    assert_eq!(
        walls[0].nodes.as_slice(),
        &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界が往復"
    );
    let t = walls[0].section.and_then(|s| m2.sections.get(s.index()));
    assert_eq!(t.and_then(|s| s.thickness), Some(250.0), "厚さが往復");
}

/// スラブ（境界＋厚さ）を含むモデルが export→import で往復すること。
#[test]
fn test_slab_roundtrip_export_import() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{DistributionMethod, Slab};
    let mut model = Model::default();
    for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
        .into_iter()
        .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
        });
    }
    model.slabs.push(Slab {
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: Vec::new(),
        loads: Vec::new(),
        method: DistributionMethod::TriTrapezoid,
        kind: Default::default(),
        one_way: None,
        edge_supported: None,
        usage: None,
        thickness: Some(200.0),
    });
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let xml = export_stbridge(&model).expect("export");
    let (m2, report) = import_stbridge_with_report(&xml).expect("import");
    assert!(m2.validate().is_ok(), "{:?}", m2.validate());
    assert_eq!(m2.slabs.len(), 1, "スラブ1件");
    assert_eq!(
        m2.slabs[0].boundary,
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界が往復"
    );
    assert_eq!(m2.slabs[0].thickness, Some(200.0), "厚さが往復");
    assert!(report.is_clean(), "警告なし {:?}", report.warnings);
}

/// 形鋼ライブラリに定義の無い断面参照は、物性ゼロで取り込みつつ警告する。
#[test]
fn test_import_report_warns_unresolved_steel_ref() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_S id="0" name="C"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="MISSING"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.sections.len(), 1);
    assert!(m.sections[0].shape.is_none(), "未解決参照は物性ゼロ断面");
    assert!(
        report
            .warnings
            .iter()
            .any(|w| w.contains("形鋼参照を解決できず")),
        "未解決の形鋼参照を報告: {:?}",
        report.warnings
    );
}

// ===== レビュー指摘の回帰テスト =====

/// [高] StbPost（間柱, bottom/top）を含むファイルが取り込みエラーで中断せず、
/// 間柱は二次部材（解析対象外・CMQ 用）として取り込まれる。
#[test]
fn test_import_stbpost_bottom_top() {
    use squid_n_core::model::SecondaryMemberKind;
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers><StbPost id="0" id_node_bottom="0" id_node_top="1"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("StbPost で中断しない");
    assert!(m.elements.is_empty(), "間柱は解析要素にしない");
    assert_eq!(m.secondary_members.len(), 1);
    assert_eq!(m.secondary_members[0].kind, SecondaryMemberKind::Post);
    assert_eq!(m.secondary_members[0].nodes, [NodeId(0), NodeId(1)]);
}

/// [高] SRC 内蔵鉄骨の参照が未解決なら警告する（無言のゼロ鉄骨を防ぐ）。
#[test]
fn test_import_report_warns_unresolved_src_steel() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_SRC id="0" name="SC" strength_steel="SN490B">
      <StbSecFigureColumn_SRC><StbSecColumn_SRC_Rect width_X="800" width_Y="800"/></StbSecFigureColumn_SRC>
      <StbSecSteelFigureColumn_SRC><StbSecSteelColumn_SRC_Same shape="MISSING_H"/></StbSecSteelFigureColumn_SRC>
    </StbSecColumn_SRC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report
            .warnings
            .iter()
            .any(|w| w.contains("内蔵鉄骨参照を解決できず")),
        "SRC 内蔵鉄骨の未解決を報告: {:?}",
        report.warnings
    );
}

/// 標準 ST-Bridge では材料は断面のグレード名で表すため、材料未設定の部材も
/// 取り込み時に断面のグレード材料を継承する（名前が材料を一意に定める）。
#[test]
fn test_member_inherits_section_grade_material() {
    let mut m = frame_nodes(); // 材料0="SN400B"
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "S".into()));
    let mut col = member(0, true, 0);
    col.material = Some(MaterialId(0));
    let mut beam = member(1, false, 0);
    beam.material = None;
    m.elements.push(col);
    m.elements.push(beam);

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    // 柱・梁とも断面グレード（SN400B）の材料を持つ。
    assert_eq!(back.elements[0].material, Some(MaterialId(0)), "柱の材料");
    assert_eq!(
        back.elements[1].material,
        Some(MaterialId(0)),
        "梁は断面グレード材料を継承する"
    );
    assert_eq!(back.materials[0].name, "SN400B");
}

/// [中] 柱・梁で異なる材料が同一断面を共有する場合、分割後の各断面に正しい材料を書き出す。
#[test]
fn test_shared_section_role_material() {
    let mut m = frame_nodes();
    m.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "SN490B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(325.0),
    });
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "S".into()));
    let mut col = member(0, true, 0);
    col.material = Some(MaterialId(0));
    let mut beam = member(1, false, 0);
    beam.material = Some(MaterialId(1));
    m.elements.push(col);
    m.elements.push(beam);

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecColumn_S ") && xml.contains("strength_main=\"SN400B\""),
        "柱断面に SN400B: {xml}"
    );
    assert!(
        xml.contains("<StbSecBeam_S ") && xml.contains("strength_main=\"SN490B\""),
        "梁断面に SN490B: {xml}"
    );
}

/// [中] 存在しない断面を参照する部材は、リンクを外しつつ警告する。
#[test]
fn test_import_report_warns_dangling_section_ref() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="99"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.elements[0].section, None);
    assert!(
        report.warnings.iter().any(|w| w.contains("存在しない断面")),
        "ダングリング断面参照を報告: {:?}",
        report.warnings
    );
}

/// [低] 鋼ブレース断面 StbSecBrace_S を取り込み、ブレースが断面を持つ。
#[test]
fn test_import_stbsecbrace_s() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="6000" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecBrace_S id="0" name="BR"><StbSecSteelFigureBrace_S><StbSecSteelBrace_S_Same shape="P1"/></StbSecSteelFigureBrace_S></StbSecBrace_S>
    <StbSecSteel><StbSecPipe name="P1" D="100" t="5"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbBrace id="0" id_node_start="0" id_node_end="1" id_section="0" tension_only="true"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.elements[0].section,
        Some(SectionId(0)),
        "ブレースが断面を持つ"
    );
    assert!(
        matches!(m.sections[0].shape, Some(SectionShape::SteelPipe { .. })),
        "ブレース断面が鋼管として復元"
    );
    assert!(
        report.is_clean(),
        "StbSecBrace_S は未対応ではない: {:?}",
        report.warnings
    );
}

/// [低] esc は XML 1.0 で表現できない制御文字（例: form feed）を除去する。
#[test]
fn test_export_strips_illegal_control_chars() {
    let mut m = frame_nodes();
    let mut sec = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    }
    .to_section(SectionId(0), "S".into());
    sec.name = "A\u{0C}B".into(); // form feed を含む名前
    m.sections.push(sec);
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    assert!(!xml.contains('\u{0C}'), "不正な制御文字が出力に残らない");
    assert!(import_stbridge(&xml).is_ok(), "出力は XML として読み戻せる");
}

/// [低] 未対応要素リストに StbOpen（開口）が含まれ、欠落が報告される。
#[test]
fn test_import_report_lists_stbopen() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbMembers><StbOpen id="0" id_wall="1"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report.warnings.iter().any(|w| w.contains("StbOpen")),
        "StbOpen の欠落を報告: {:?}",
        report.warnings
    );
}

/// ST-Bridge は境界条件（支点）を持たないため、取り込み時に最下レベル
/// （Z 最小、許容差 1mm）の節点をピン支点（並進固定・回転自由）に自動設定し、notes で通知する。
/// notes は欠落警告ではないため `is_clean` には影響しない。
#[test]
fn test_import_auto_fixes_base_level_supports() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="100"/>
    <StbNode id="1" X="6000" Y="0" Z="100.5"/>
    <StbNode id="2" X="0" Y="0" Z="3500"/>
    <StbNode id="3" X="6000" Y="0" Z="3500"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="2" id_section="0" kind_structure="S"/>
      <StbColumn id="1" name="C2" id_node_bottom="1" id_node_top="3" id_section="0" kind_structure="S"/>
    </StbColumns>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");

    use squid_n_core::dof::Dof6Mask;
    // 最下レベル: Z=100 と Z=100.5（許容差 1mm 以内）の 2 節点がピン支点になる。
    assert_eq!(m.nodes[0].restraint, Dof6Mask::PINNED);
    assert_eq!(m.nodes[1].restraint, Dof6Mask::PINNED);
    // 上部節点は自由のまま。
    assert_eq!(m.nodes[2].restraint, Dof6Mask::FREE);
    assert_eq!(m.nodes[3].restraint, Dof6Mask::FREE);
    // notes で通知され、欠落警告（is_clean）には影響しない。
    assert!(
        report
            .notes
            .iter()
            .any(|n| n.contains("ピン支点に設定") && n.contains("2 箇所")),
        "notes: {:?}",
        report.notes
    );
    assert!(report.is_clean(), "warnings: {:?}", report.warnings);
}

/// 小梁（StbBeam）は二次部材（解析対象外・CMQ 用）として取り込まれ、
/// 大梁（StbGirder）は従来どおり解析要素になる。断面・材料（グレード伝播）も
/// 二次部材へ解決される。
#[test]
fn test_import_stbbeam_as_secondary_member() {
    use squid_n_core::model::SecondaryMemberKind;
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="6000" Y="0" Z="0"/>
    <StbNode id="2" X="2000" Y="0" Z="0"/>
    <StbNode id="3" X="2000" Y="4000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="0" name="G">
      <StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1" strength_main="SN400B"/></StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbGirders>
      <StbGirder id="0" name="G1" id_node_start="0" id_node_end="1" id_section="0" kind_structure="S"/>
    </StbGirders>
    <StbBeams>
      <StbBeam id="1" name="B1" id_node_start="2" id_node_end="3" id_section="0" kind_structure="S"/>
    </StbBeams>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.elements.len(), 1, "大梁のみ解析要素");
    assert_eq!(m.secondary_members.len(), 1);
    let sm = &m.secondary_members[0];
    assert_eq!(sm.kind, SecondaryMemberKind::Joist);
    assert_eq!(sm.nodes, [NodeId(2), NodeId(3)]);
    assert!(sm.section.is_some(), "断面参照が解決されるはず");
    assert!(sm.material.is_some(), "グレード材料が伝播されるはず");
    assert!(
        report.notes.iter().any(|n| n.contains("小梁 1 本")),
        "二次部材の取り込みを通知: {:?}",
        report.notes
    );
    assert!(m.validate().is_ok());
}

/// 二次部材（小梁・間柱）が ST-Bridge 書き出し（StbBeam/StbPost）→再取り込みで
/// 保存されること（往復）。
#[test]
fn test_secondary_members_roundtrip() {
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{SecondaryMember, SecondaryMemberKind};

    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    m.sections.push(h.to_section(SectionId(0), "G".into()));
    m.elements.push(member(0, false, 0));
    // 小梁と間柱を 1 本ずつ（節点は既存節点を使う）。
    m.secondary_members.push(SecondaryMember {
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        name: "B1".into(),
    });
    m.secondary_members.push(SecondaryMember {
        kind: SecondaryMemberKind::Post,
        nodes: [NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        name: "P1".into(),
    });
    m.validate().expect("元モデルは validate を通る");

    let xml = export_stbridge(&m).expect("export");
    assert!(xml.contains("<StbBeams>"), "小梁を書き出す: {xml}");
    assert!(xml.contains("<StbPosts>"), "間柱を書き出す: {xml}");

    let (back, _report) = import_stbridge_with_report(&xml).expect("re-import");
    assert_eq!(back.secondary_members.len(), 2);
    let kinds: Vec<SecondaryMemberKind> = back.secondary_members.iter().map(|s| s.kind).collect();
    assert!(kinds.contains(&SecondaryMemberKind::Joist));
    assert!(kinds.contains(&SecondaryMemberKind::Post));
    assert_eq!(back.elements.len(), 1, "大梁は解析要素のまま");
    assert!(back.validate().is_ok());
}

/// 厚さが分かるスラブ（StbSecSlab_RC）には、取り込み時に自重
/// （厚さ×γRC=24kN/m³）が固定荷重として自動設定される（ST-Bridge は
/// 床荷重を持たないため、DL・CMQ・地震用重量への算入の出発点にする）。
#[test]
fn test_import_slab_auto_self_weight_from_thickness() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecSlab_RC id="0" name="S150">
      <StbSecFigureSlab_RC><StbSecSlab_RC_Straight depth="150"/></StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbSlabs>
      <StbSlab id="0" name="S1" id_section="0" kind_structure="RC">
        <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
      </StbSlab>
    </StbSlabs>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.slabs.len(), 1);
    let slab = &m.slabs[0];
    assert_eq!(slab.thickness, Some(150.0));
    assert_eq!(slab.loads.len(), 1, "自重が床荷重として自動設定される");
    // 150 mm × 24 kN/m³ = 3.6 kN/m² = 3.6e-3 N/mm²
    assert!(
        (slab.loads[0].value - 3.6e-3).abs() < 1e-12,
        "value={}",
        slab.loads[0].value
    );
    assert!(
        (slab.dead_intensity() - 3.6e-3).abs() < 1e-12,
        "分配強度に自重が乗る"
    );
    assert!(
        report.notes.iter().any(|n| n.contains("スラブ 1 枚に自重")),
        "自動設定を通知: {:?}",
        report.notes
    );
}
