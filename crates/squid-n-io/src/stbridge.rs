//! ST-Bridge（XML, 2.0 系）入出力。設計書 §12.5 / 仕様 `specs/P8_操作と連携.md` §7.1。
//!
//! # 対応範囲（意味的往復を保証する subset）
//! - **節点**（座標・所属層）、**層**（名称・標高）、**材料**（E・ν・密度・Fc・Fy）、
//!   **断面**（面積・断面二次モーメント等の物性）、**部材**（柱＝鉛直／大梁＝水平、節点・断面・
//!   材料の参照、部材軸 ref_vector）、**荷重ケース**（節点荷重）。
//! - import→export→再import で上記が意味的に一致する（DoD §8.3）。
//!
//! # 非対応（仕様どおり対象外）
//! - 解析結果・独自属性（§12.5）。
//! - 拘束条件・質量（ST-Bridge の幾何スコープ外。import 後は既定値）。
//! - **断面は実 ST-Bridge の形鋼ライブラリ参照（StbSecColumn_S 等）ではなく、内部モデルの物性を
//!   そのまま持つ `StbSecRaw` で表現する**（正準モデルを唯一の真実とする方針）。他社ソフトとの
//!   完全な相互運用は断面形状名のマッピングが要るため将来課題。
//! - 床・ブレース・剛域・端部接合等の詳細。
//!
//! 一次資料: ST-Bridge 公式スキーマ（XML 2.0 系）。要素・属性名はこれに準拠（subset）。

use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
    NodalLoad, Node, Section, Story,
};
use std::collections::HashMap;

#[derive(Debug, thiserror::Error)]
pub enum StbError {
    #[error("xml parse: {0}")]
    Parse(String),
    #[error("unsupported version: {0}")]
    Version(String),
    #[error("unmappable element: {0}")]
    Unmappable(String),
}

const STB_VERSION: &str = "2.0.0";

// ===== Export =====

/// 内部モデルを ST-Bridge 2.0（subset）XML 文字列へ出力する。
pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    let mut s = String::new();
    s.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    s.push_str(&format!("<ST_BRIDGE version=\"{STB_VERSION}\">\n"));
    s.push_str("  <StbModel>\n");

    // 節点
    s.push_str("    <StbNodes>\n");
    for n in &model.nodes {
        let story = n.story.map(|s| s.0 as i64).unwrap_or(-1);
        s.push_str(&format!(
            "      <StbNode id=\"{}\" x=\"{}\" y=\"{}\" z=\"{}\" story=\"{}\"/>\n",
            n.id.0,
            fmt(n.coord[0]),
            fmt(n.coord[1]),
            fmt(n.coord[2]),
            story
        ));
    }
    s.push_str("    </StbNodes>\n");

    // 層
    s.push_str("    <StbStories>\n");
    for st in &model.stories {
        s.push_str(&format!(
            "      <StbStory id=\"{}\" name=\"{}\" height=\"{}\"/>\n",
            st.id.0,
            esc(&st.name),
            fmt(st.elevation)
        ));
    }
    s.push_str("    </StbStories>\n");

    // 材料
    s.push_str("    <StbMaterials>\n");
    for m in &model.materials {
        s.push_str(&format!(
            "      <StbMaterial id=\"{}\" name=\"{}\" young=\"{}\" poisson=\"{}\" density=\"{}\" shear=\"{}\" fc=\"{}\" fy=\"{}\"/>\n",
            m.id.0,
            esc(&m.name),
            fmt(m.young),
            fmt(m.poisson),
            fmt(m.density),
            opt(m.shear),
            opt(m.fc),
            opt(m.fy),
        ));
    }
    s.push_str("    </StbMaterials>\n");

    // 断面（subset: 物性を直接保持）
    s.push_str("    <StbSections>\n");
    for sec in &model.sections {
        s.push_str(&format!(
            "      <StbSecRaw id=\"{}\" name=\"{}\" area=\"{}\" iy=\"{}\" iz=\"{}\" j=\"{}\" depth=\"{}\" width=\"{}\"/>\n",
            sec.id.0,
            esc(&sec.name),
            fmt(sec.area), fmt(sec.iy), fmt(sec.iz), fmt(sec.j),
            fmt(sec.depth), fmt(sec.width),
        ));
    }
    s.push_str("    </StbSections>\n");

    // 部材（柱＝鉛直／大梁＝水平）
    s.push_str("    <StbMembers>\n");
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let n0 = &model.nodes[e.nodes[0].index()];
        let n1 = &model.nodes[e.nodes[1].index()];
        let dz = (n1.coord[2] - n0.coord[2]).abs();
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let is_col = len > 1e-12 && dz / len > 0.707;
        let sec = e.section.map(|s| s.0 as i64).unwrap_or(-1);
        let mat = e.material.map(|m| m.0 as i64).unwrap_or(-1);
        let r = e.local_axis.ref_vector;
        if is_col {
            // 下端→上端で揃える
            let (bot, top) = if n0.coord[2] <= n1.coord[2] {
                (e.nodes[0], e.nodes[1])
            } else {
                (e.nodes[1], e.nodes[0])
            };
            s.push_str(&format!(
                "      <StbColumn id=\"{}\" id_node_bottom=\"{}\" id_node_top=\"{}\" id_section=\"{}\" id_material=\"{}\" rx=\"{}\" ry=\"{}\" rz=\"{}\"/>\n",
                e.id.0, bot.0, top.0, sec, mat, fmt(r[0]), fmt(r[1]), fmt(r[2])
            ));
        } else {
            s.push_str(&format!(
                "      <StbGirder id=\"{}\" id_node_start=\"{}\" id_node_end=\"{}\" id_section=\"{}\" id_material=\"{}\" rx=\"{}\" ry=\"{}\" rz=\"{}\"/>\n",
                e.id.0, e.nodes[0].0, e.nodes[1].0, sec, mat, fmt(r[0]), fmt(r[1]), fmt(r[2])
            ));
        }
    }
    s.push_str("    </StbMembers>\n");

    // 荷重ケース（節点荷重）
    s.push_str("    <StbLoadCases>\n");
    for lc in &model.load_cases {
        s.push_str(&format!(
            "      <StbLoadCase id=\"{}\" name=\"{}\">\n",
            lc.id.0,
            esc(&lc.name)
        ));
        for nl in &lc.nodal {
            let v = nl.values;
            s.push_str(&format!(
                "        <StbNodalLoad id_node=\"{}\" fx=\"{}\" fy=\"{}\" fz=\"{}\" mx=\"{}\" my=\"{}\" mz=\"{}\"/>\n",
                nl.node.0, fmt(v[0]), fmt(v[1]), fmt(v[2]), fmt(v[3]), fmt(v[4]), fmt(v[5])
            ));
        }
        s.push_str("      </StbLoadCase>\n");
    }
    s.push_str("    </StbLoadCases>\n");

    s.push_str("  </StbModel>\n");
    s.push_str("</ST_BRIDGE>\n");
    Ok(s)
}

fn fmt(x: f64) -> String {
    // 整数値は小数点なしで、それ以外は既定の f64 表記で（往復で値が保たれる）。
    if x == x.trunc() && x.is_finite() {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

fn opt(x: Option<f64>) -> String {
    match x {
        Some(v) => fmt(v),
        None => String::new(),
    }
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

// ===== Import =====

/// ST-Bridge 2.0（subset）XML を内部モデルへ取り込む。
pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut model = Model::default();
    let mut load_cases: Vec<LoadCase> = Vec::new();
    let mut version_ok = false;

    loop {
        match reader
            .read_event()
            .map_err(|e| StbError::Parse(e.to_string()))?
        {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                let a = attrs(&e)?;
                match tag.as_str() {
                    "ST_BRIDGE" => {
                        let v = a.get("version").cloned().unwrap_or_default();
                        if !v.starts_with("2.") {
                            return Err(StbError::Version(v));
                        }
                        version_ok = true;
                    }
                    "StbNode" => {
                        let story = match get_i64(&a, "story") {
                            Some(s) if s >= 0 => Some(StoryId(s as u32)),
                            _ => None,
                        };
                        model.nodes.push(Node {
                            id: NodeId(get_u32(&a, "id")?),
                            coord: [get_f64(&a, "x")?, get_f64(&a, "y")?, get_f64(&a, "z")?],
                            restraint: squid_n_core::dof::Dof6Mask::FREE,
                            mass: None,
                            story,
                        });
                    }
                    "StbStory" => {
                        model.stories.push(Story {
                            level_kind: Default::default(),
                            structure: Default::default(),
                            id: StoryId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            elevation: get_f64(&a, "height")?,
                            node_ids: vec![],
                            diaphragms: vec![],
                            seismic_weight: None,
                        });
                    }
                    "StbMaterial" => {
                        model.materials.push(Material {
                            concrete_class: Default::default(),
                            id: MaterialId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            young: get_f64(&a, "young")?,
                            poisson: get_f64(&a, "poisson")?,
                            density: get_f64(&a, "density")?,
                            shear: get_opt_f64(&a, "shear"),
                            fc: get_opt_f64(&a, "fc"),
                            fy: get_opt_f64(&a, "fy"),
                        });
                    }
                    "StbSecRaw" => {
                        model.sections.push(Section {
                            id: SectionId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            area: get_f64(&a, "area")?,
                            iy: get_f64(&a, "iy")?,
                            iz: get_f64(&a, "iz")?,
                            j: get_f64(&a, "j")?,
                            depth: get_f64(&a, "depth").unwrap_or(0.0),
                            width: get_f64(&a, "width").unwrap_or(0.0),
                            as_y: 0.0,
                            as_z: 0.0,
                            panel_thickness: None,
                            thickness: None,
                            // ST-Bridge インポート断面はパラメトリック形状を持たない。
                            shape: None,
                        });
                    }
                    "StbColumn" => {
                        let bot = NodeId(get_u32(&a, "id_node_bottom")?);
                        let top = NodeId(get_u32(&a, "id_node_top")?);
                        model.elements.push(make_member(&a, bot, top)?);
                    }
                    "StbGirder" | "StbBeam" => {
                        let st = NodeId(get_u32(&a, "id_node_start")?);
                        let en = NodeId(get_u32(&a, "id_node_end")?);
                        model.elements.push(make_member(&a, st, en)?);
                    }
                    "StbLoadCase" => {
                        load_cases.push(LoadCase {
                            kind: Default::default(),
                            id: LoadCaseId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            nodal: vec![],
                            member: vec![],
                        });
                    }
                    "StbNodalLoad" => {
                        let nl = NodalLoad {
                            node: NodeId(get_u32(&a, "id_node")?),
                            values: [
                                get_f64(&a, "fx").unwrap_or(0.0),
                                get_f64(&a, "fy").unwrap_or(0.0),
                                get_f64(&a, "fz").unwrap_or(0.0),
                                get_f64(&a, "mx").unwrap_or(0.0),
                                get_f64(&a, "my").unwrap_or(0.0),
                                get_f64(&a, "mz").unwrap_or(0.0),
                            ],
                        };
                        if let Some(lc) = load_cases.last_mut() {
                            lc.nodal.push(nl);
                        } else {
                            return Err(StbError::Parse("StbNodalLoad outside StbLoadCase".into()));
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    if !version_ok {
        return Err(StbError::Version(
            "missing ST_BRIDGE version 2.x root".into(),
        ));
    }

    model.load_cases = load_cases;
    Ok(model)
}

fn make_member(
    a: &HashMap<String, String>,
    n_i: NodeId,
    n_j: NodeId,
) -> Result<ElementData, StbError> {
    use smallvec::smallvec;
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(SectionId(s as u32)),
        _ => None,
    };
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(MaterialId(m as u32)),
        _ => None,
    };
    let r = [
        get_f64(a, "rx").unwrap_or(0.0),
        get_f64(a, "ry").unwrap_or(0.0),
        get_f64(a, "rz").unwrap_or(1.0),
    ];
    Ok(ElementData {
        id: ElemId(get_u32(a, "id")?),
        kind: ElementKind::Beam,
        nodes: smallvec![n_i, n_j],
        section,
        material,
        local_axis: LocalAxis { ref_vector: r },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    })
}

fn attrs(e: &quick_xml::events::BytesStart) -> Result<HashMap<String, String>, StbError> {
    let mut m = HashMap::new();
    for a in e.attributes() {
        let a = a.map_err(|err| StbError::Parse(err.to_string()))?;
        let key = String::from_utf8_lossy(a.key.as_ref()).to_string();
        let val = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| StbError::Parse(err.to_string()))?
            .to_string();
        m.insert(key, val);
    }
    Ok(m)
}

fn get_f64(a: &HashMap<String, String>, k: &str) -> Result<f64, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<f64>()
        .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")))
}

fn get_opt_f64(a: &HashMap<String, String>, k: &str) -> Option<f64> {
    match a.get(k) {
        Some(v) if !v.is_empty() => v.parse::<f64>().ok(),
        _ => None,
    }
}

fn get_u32(a: &HashMap<String, String>, k: &str) -> Result<u32, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<u32>()
        .map_err(|_| StbError::Parse(format!("bad u32 attr {k}")))
}

fn get_i64(a: &HashMap<String, String>, k: &str) -> Option<i64> {
    a.get(k).and_then(|v| v.parse::<i64>().ok())
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;

    fn representative_model() -> Model {
        let mut m = Model::default();
        // 4 節点（底2・上2）。
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
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: None,
        });
        m.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S400".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "C&1<2".into(), // エスケープ確認用
            area: 1.2345e4,
            iy: 1.0e8,
            iz: 2.0e8,
            j: 3.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        // 柱2本（鉛直）＋大梁1本（水平）。
        m.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.elements.push(ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(1), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.elements.push(ElementData {
            id: ElemId(2),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "L1".into(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [10.5, 0.0, -3.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        });
        m
    }

    /// 意味的に一致するか（対象スコープのフィールドのみ）。
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
        assert_eq!(a.materials.len(), b.materials.len());
        for (x, y) in a.materials.iter().zip(&b.materials) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name);
            assert_eq!(x.young, y.young);
            assert_eq!(x.poisson, y.poisson);
            assert_eq!(x.fy, y.fy);
            assert_eq!(x.fc, y.fc);
        }
        assert_eq!(a.sections.len(), b.sections.len());
        for (x, y) in a.sections.iter().zip(&b.sections) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name, "section name (escape)");
            assert_eq!(x.area, y.area);
            assert_eq!(x.iy, y.iy);
            assert_eq!(x.iz, y.iz);
            assert_eq!(x.j, y.j);
            assert_eq!(x.depth, y.depth);
            assert_eq!(x.width, y.width);
        }
        assert_eq!(a.elements.len(), b.elements.len());
        for (x, y) in a.elements.iter().zip(&b.elements) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.nodes.as_slice(), y.nodes.as_slice(), "connectivity");
            assert_eq!(x.section, y.section);
            assert_eq!(x.material, y.material);
            assert_eq!(
                x.local_axis.ref_vector, y.local_axis.ref_vector,
                "ref_vector"
            );
        }
        assert_eq!(a.load_cases.len(), b.load_cases.len());
        for (x, y) in a.load_cases.iter().zip(&b.load_cases) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name);
            assert_eq!(x.nodal.len(), y.nodal.len());
            for (p, q) in x.nodal.iter().zip(&y.nodal) {
                assert_eq!(p.node, q.node);
                assert_eq!(p.values, q.values);
            }
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
    fn test_imported_model_validates() {
        let m = representative_model();
        let xml = export_stbridge(&m).unwrap();
        let m2 = import_stbridge(&xml).unwrap();
        assert!(m2.validate().is_ok(), "取り込んだモデルは検証を通る");
    }
}
