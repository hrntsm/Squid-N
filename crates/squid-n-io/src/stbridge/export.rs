//! ST-Bridge 直列化（Export）。設計書 §12.5。
//!
//! 出力は **ST-Bridge 2.0.2 標準スキーマ準拠**の幾何モデル（他ソフト・BIM が読める形）。
//! - 断面は標準要素（`StbSecColumn_S`/`StbSecBeam_RC` 等）＋形鋼ライブラリ `StbSecSteel`。
//! - 部材は複数形コンテナ（`StbColumns`/`StbGirders`/`StbBeams`/`StbBraces`/`StbSlabs`/
//!   `StbWalls`）に入れ、向きは `rotate`、端部は `condition_*`、ブレースは `feature_brace`。
//! - 材料は ST-Bridge の慣習どおり断面のグレード名（鋼 `strength_main`、RC/SRC/CFT の
//!   コンクリート `strength_concrete`）で表す（`StbModel` は材料テーブルを持たない）。
//! - id は ST-Bridge の `positiveInteger`（1 始まり）に合わせ、内部 0 始まり id に +1 する。
//!
//! ST-Bridge の幾何スコープ外（材料の E/ν・節点荷重・拘束・独自属性）は往復しない。
//! 完全一致の往復が必要な場合はネイティブの `.scz` を使う（`docs/model_io.md`）。
//!
//! - [`export_stbridge`] — 内部モデルを標準 ST-Bridge 2.0.2 XML 文字列へ出力する。
//! - [`fmt`] — 整数値は小数点なし、それ以外は既定の f64 表記で整形する（`pub(super)`）。
//! - [`esc`] — XML 特殊文字をエスケープする（`pub(super)`）。

use super::section_std::standard_sections;
use super::{StbError, STB_VERSION};
use squid_n_core::model::{ElementKind, EndCondition, Model, StoryLevelKind};
use squid_n_core::section_shape::SectionShape;

/// ST-Bridge の id は `positiveInteger`（1 以上）。内部 0 始まり id に +1 して出力する。
fn sid(internal_id: u32) -> u32 {
    internal_id + 1
}

/// 内部モデルを標準 ST-Bridge 2.0.2 XML 文字列へ出力する。
pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    // 標準断面ブロックと、部材参照（id_section）の柱用・梁用張り替えマップ。
    let std = standard_sections(model);
    let (sections_body, steel_lib, col_map, beam_map) =
        (std.sections_xml, std.steel_lib, std.col_map, std.beam_map);

    let mut s = String::new();
    s.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    s.push_str(&format!(
        "<ST_BRIDGE xmlns=\"https://www.building-smart.or.jp/dl\" \
         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" version=\"{STB_VERSION}\">\n"
    ));

    // StbCommon（ルート必須。プロジェクト名・アプリ名は最小限の既定値）。
    s.push_str(
        "  <StbCommon project_name=\"Squid-N\" app_name=\"Squid-N\" app_version=\"0.0.1\"/>\n",
    );

    s.push_str("  <StbModel>\n");

    // 節点（X/Y/Z、kind 必須）。所属部材が不明なので kind=ON_GRID を既定にする。
    s.push_str("    <StbNodes>\n");
    for n in &model.nodes {
        s.push_str(&format!(
            "      <StbNode id=\"{}\" X=\"{}\" Y=\"{}\" Z=\"{}\" kind=\"ON_GRID\"/>\n",
            sid(n.id.0),
            fmt(n.coord[0]),
            fmt(n.coord[1]),
            fmt(n.coord[2]),
        ));
    }
    s.push_str("    </StbNodes>\n");

    // 層（name・height・kind 必須。所属節点は StbNodeIdList で列挙）。所属は各節点の
    // `story`（正）と層の `node_ids` の和集合を、節点 id 昇順・重複なしで書き出す。
    s.push_str("    <StbStories>\n");
    for st in &model.stories {
        let mut members: Vec<u32> = model
            .nodes
            .iter()
            .filter(|n| n.story == Some(st.id))
            .map(|n| n.id.0)
            .collect();
        for nid in &st.node_ids {
            if !members.contains(&nid.0) {
                members.push(nid.0);
            }
        }
        members.sort_unstable();
        s.push_str(&format!(
            "      <StbStory id=\"{}\" name=\"{}\" height=\"{}\" kind=\"{}\">\n",
            sid(st.id.0),
            esc(&st.name),
            fmt(st.elevation),
            story_kind(st.level_kind),
        ));
        if !members.is_empty() {
            s.push_str("        <StbNodeIdList>\n");
            for nid in members {
                s.push_str(&format!("          <StbNodeId id=\"{}\"/>\n", sid(nid)));
            }
            s.push_str("        </StbNodeIdList>\n");
        }
        s.push_str("      </StbStory>\n");
    }
    s.push_str("    </StbStories>\n");

    // 部材（複数形コンテナに種別ごとに束ねる。空コンテナは出力しない）。
    s.push_str("    <StbMembers>\n");
    s.push_str(&members_body(model, &col_map, &beam_map));
    s.push_str("    </StbMembers>\n");

    // 断面（標準要素＋形鋼ライブラリ）＋スラブ断面＋壁断面。
    // スラブ断面 id は既存断面 id（柱・梁。分割で増える）と衝突しない範囲から採番する。
    let slab_sec_base = col_map
        .values()
        .chain(beam_map.values())
        .copied()
        .max()
        .map(|m| m + 1)
        .unwrap_or(0)
        .max(model.sections.len() as u32);
    let wall_sec_base = slab_sec_base + model.slabs.len() as u32;

    // スキーマ順: 柱・梁・ブレース断面 → スラブ断面 → 壁断面 → 形鋼ライブラリ。
    s.push_str("    <StbSections>\n");
    s.push_str(&sections_body);
    s.push_str(&slab_sections(model, slab_sec_base));
    s.push_str(&wall_sections(model, wall_sec_base));
    s.push_str(&steel_lib);
    s.push_str("    </StbSections>\n");

    s.push_str("    <StbJoints/>\n");
    s.push_str("  </StbModel>\n");
    s.push_str("</ST_BRIDGE>\n");
    Ok(s)
}

/// `StbMembers` 本体（柱・大梁・ブレース・スラブ・壁を複数形コンテナに束ねる）。
fn members_body(
    model: &Model,
    col_map: &std::collections::HashMap<u32, u32>,
    beam_map: &std::collections::HashMap<u32, u32>,
) -> String {
    let mut columns = String::new();
    let mut girders = String::new();
    let mut braces = String::new();

    for e in &model.elements {
        let mat_grade = e
            .material
            .and_then(|m| model.materials.get(m.index()))
            .map(|m| m.name.clone())
            .unwrap_or_default();
        match e.kind {
            ElementKind::Beam if e.nodes.len() == 2 => {
                let n0 = &model.nodes[e.nodes[0].index()];
                let n1 = &model.nodes[e.nodes[1].index()];
                let dz = (n1.coord[2] - n0.coord[2]).abs();
                let dx = n1.coord[0] - n0.coord[0];
                let dy = n1.coord[1] - n0.coord[1];
                let len = (dx * dx + dy * dy + dz * dz).sqrt();
                let is_col = len > 1e-12 && dz / len > 0.707;
                let role_map = if is_col { col_map } else { beam_map };
                let sec = e
                    .section
                    .map(|s| role_map.get(&s.0).copied().unwrap_or(s.0))
                    .map(|v| v as i64)
                    .unwrap_or(-1);
                let rot = rotate_of(e, n0.coord, n1.coord);
                let ks = kind_structure(model, e, &mat_grade);
                if is_col {
                    let (bot, top) = if n0.coord[2] <= n1.coord[2] {
                        (e.nodes[0], e.nodes[1])
                    } else {
                        (e.nodes[1], e.nodes[0])
                    };
                    let (cb, ct) = if n0.coord[2] <= n1.coord[2] {
                        (e.end_cond[0], e.end_cond[1])
                    } else {
                        (e.end_cond[1], e.end_cond[0])
                    };
                    columns.push_str(&format!(
                        "        <StbColumn id=\"{}\" name=\"C{}\" id_node_bottom=\"{}\" id_node_top=\"{}\" \
                         rotate=\"{}\" id_section=\"{}\" kind_structure=\"{}\" condition_bottom=\"{}\" condition_top=\"{}\"/>\n",
                        sid(e.id.0), sid(e.id.0), sid(bot.0), sid(top.0),
                        fmt(rot), sec_ref(sec), ks, cond(cb), cond(ct),
                    ));
                } else {
                    girders.push_str(&format!(
                        "        <StbGirder id=\"{}\" name=\"G{}\" id_node_start=\"{}\" id_node_end=\"{}\" \
                         rotate=\"{}\" id_section=\"{}\" kind_structure=\"{}\" isFoundation=\"false\" \
                         condition_start=\"{}\" condition_end=\"{}\"/>\n",
                        sid(e.id.0), sid(e.id.0), sid(e.nodes[0].0), sid(e.nodes[1].0),
                        fmt(rot), sec_ref(sec), ks, cond(e.end_cond[0]), cond(e.end_cond[1]),
                    ));
                }
            }
            ElementKind::Brace { tension_only } if e.nodes.len() == 2 => {
                let sec = e
                    .section
                    .map(|s| {
                        col_map
                            .get(&s.0)
                            .or_else(|| beam_map.get(&s.0))
                            .copied()
                            .unwrap_or(s.0) as i64
                    })
                    .unwrap_or(-1);
                let feature = if tension_only {
                    "TENSION"
                } else {
                    "TENSIONANDCOMPRESSION"
                };
                braces.push_str(&format!(
                    "        <StbBrace id=\"{}\" name=\"BR{}\" id_node_start=\"{}\" id_node_end=\"{}\" \
                     rotate=\"0\" id_section=\"{}\" kind_structure=\"S\" feature_brace=\"{}\" \
                     condition_start=\"PIN\" condition_end=\"PIN\"/>\n",
                    sid(e.id.0), sid(e.id.0), sid(e.nodes[0].0), sid(e.nodes[1].0),
                    sec_ref(sec), feature,
                ));
            }
            _ => {}
        }
    }

    // スラブ（StbSlab）。境界節点ループ＋断面参照。member id は要素 id と別空間なので
    // 要素数の次から採番する（1 始まり）。
    let slab_member_base = model.elements.len() as u32;
    let slab_sec_base = col_map
        .values()
        .chain(beam_map.values())
        .copied()
        .max()
        .map(|m| m + 1)
        .unwrap_or(0)
        .max(model.sections.len() as u32);
    let mut slabs = String::new();
    for slab in &model.slabs {
        let mid = slab_member_base + slab.id.0;
        let sec = slab_sec_base + slab.id.0;
        let order = slab
            .boundary
            .iter()
            .map(|n| sid(n.0).to_string())
            .collect::<Vec<_>>()
            .join(" ");
        let kind_slab = match slab.kind {
            squid_n_core::model::SlabKind::Interior => "NORMAL",
            _ => "CANTI",
        };
        slabs.push_str(&format!(
            "        <StbSlab id=\"{}\" name=\"S{}\" id_section=\"{}\" kind_structure=\"RC\" kind_slab=\"{}\" isFoundation=\"false\">\n",
            sid(mid),
            sid(slab.id.0),
            sid(sec),
            kind_slab,
        ));
        slabs.push_str(&format!(
            "          <StbNodeIdOrder>{order}</StbNodeIdOrder>\n"
        ));
        slabs.push_str("        </StbSlab>\n");
    }

    // 壁（StbWall）。壁要素（Wall/Shell、境界 3〜N 節点）の節点ループ＋断面参照。
    let wall_sec_base = slab_sec_base + model.slabs.len() as u32;
    let mut walls = String::new();
    let mut wall_idx = 0u32;
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Wall | ElementKind::Shell) || e.nodes.len() < 3 {
            continue;
        }
        let order = e
            .nodes
            .iter()
            .map(|n| sid(n.0).to_string())
            .collect::<Vec<_>>()
            .join(" ");
        let sec = wall_sec_base + wall_idx;
        walls.push_str(&format!(
            "        <StbWall id=\"{}\" name=\"W{}\" id_section=\"{}\" kind_structure=\"RC\">\n",
            sid(e.id.0),
            sid(e.id.0),
            sid(sec),
        ));
        walls.push_str(&format!(
            "          <StbNodeIdOrder>{order}</StbNodeIdOrder>\n"
        ));
        walls.push_str("        </StbWall>\n");
        wall_idx += 1;
    }

    // 複数形コンテナはスキーマ上、子を 1 つ以上持つ必要がある。空なら出力しない。
    // 順序はスキーマの sequence（Columns→Girders→Braces→Slabs→Walls）に合わせる。
    let mut body = String::new();
    if !columns.is_empty() {
        body.push_str("      <StbColumns>\n");
        body.push_str(&columns);
        body.push_str("      </StbColumns>\n");
    }
    if !girders.is_empty() {
        body.push_str("      <StbGirders>\n");
        body.push_str(&girders);
        body.push_str("      </StbGirders>\n");
    }
    if !braces.is_empty() {
        body.push_str("      <StbBraces>\n");
        body.push_str(&braces);
        body.push_str("      </StbBraces>\n");
    }
    if !slabs.is_empty() {
        body.push_str("      <StbSlabs>\n");
        body.push_str(&slabs);
        body.push_str("      </StbSlabs>\n");
    }
    if !walls.is_empty() {
        body.push_str("      <StbWalls>\n");
        body.push_str(&walls);
        body.push_str("      </StbWalls>\n");
    }
    body
}

/// 断面参照属性値。負（未参照）は -1、そうでなければ +1 した positiveInteger。
fn sec_ref(sec_internal: i64) -> String {
    if sec_internal < 0 {
        "-1".to_string()
    } else {
        format!("{}", sec_internal as u32 + 1)
    }
}

/// 端部接合条件（FIX/PIN）。
fn cond(c: EndCondition) -> &'static str {
    match c {
        EndCondition::Pinned => "PIN",
        _ => "FIX",
    }
}

/// 層種別を ST-Bridge の `kind`（GENERAL/PENTHOUSE/BASEMENT）へ写す。
fn story_kind(k: StoryLevelKind) -> &'static str {
    match k {
        StoryLevelKind::Penthouse { .. } => "PENTHOUSE",
        StoryLevelKind::Basement { .. } => "BASEMENT",
        StoryLevelKind::Normal => "GENERAL",
    }
}

/// 部材の構造種別（`kind_structure`）を断面形状・材料から推定する。
fn kind_structure(
    model: &squid_n_core::model::Model,
    e: &squid_n_core::model::ElementData,
    mat_grade: &str,
) -> &'static str {
    let shape = e
        .section
        .and_then(|s| model.sections.get(s.index()))
        .and_then(|s| s.shape.as_ref());
    match shape {
        Some(SectionShape::RcRect { .. } | SectionShape::RcCircle { .. }) => "RC",
        Some(SectionShape::SrcRect { .. }) => "SRC",
        // CFT は柱のみ。梁の kind_structure に CFT が無いため、ここでは柱前提で扱う。
        Some(SectionShape::CftBox { .. } | SectionShape::CftPipe { .. }) => "CFT",
        Some(_) => "S",
        // 形状が無い場合は材料グレード名から推定（Fc… はコンクリート）。
        None => {
            if mat_grade.starts_with("Fc") || mat_grade.starts_with("FC") {
                "RC"
            } else {
                "S"
            }
        }
    }
}

/// 部材の ref_vector と軸から `rotate` 角 [deg] を復元する（import の逆変換）。
/// `rotate=0` の基準（水平材は鉛直上、鉛直材はグローバル X）に対する軸まわりの回転角。
fn rotate_of(e: &squid_n_core::model::ElementData, p_i: [f64; 3], p_j: [f64; 3]) -> f64 {
    let axis = {
        let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-9 {
            return 0.0;
        }
        [d[0] / l, d[1] / l, d[2] / l]
    };
    let base = if axis[2].abs() > 0.99 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let bdot = base[0] * axis[0] + base[1] * axis[1] + base[2] * axis[2];
    let ref0 = normalize([
        base[0] - bdot * axis[0],
        base[1] - bdot * axis[1],
        base[2] - bdot * axis[2],
    ]);
    // 現在の ref_vector を軸へ直交化。
    let r = e.local_axis.ref_vector;
    let rdot = r[0] * axis[0] + r[1] * axis[1] + r[2] * axis[2];
    let refv = normalize([
        r[0] - rdot * axis[0],
        r[1] - rdot * axis[1],
        r[2] - rdot * axis[2],
    ]);
    // ref0→refv の軸まわり符号付き角。angle = atan2((ref0×refv)·axis, ref0·refv)。
    let cross = [
        ref0[1] * refv[2] - ref0[2] * refv[1],
        ref0[2] * refv[0] - ref0[0] * refv[2],
        ref0[0] * refv[1] - ref0[1] * refv[0],
    ];
    let sin = cross[0] * axis[0] + cross[1] * axis[1] + cross[2] * axis[2];
    let cos = ref0[0] * refv[0] + ref0[1] * refv[1] + ref0[2] * refv[2];
    if sin.abs() < 1e-9 && cos.abs() < 1e-9 {
        return 0.0;
    }
    sin.atan2(cos).to_degrees()
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l < 1e-9 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / l, v[1] / l, v[2] / l]
    }
}

/// スラブ断面（`StbSecSlab_RC`）ブロックを生成する。各スラブに 1 つの断面を出力し、
/// 厚さは `slab.thickness`（未設定なら建物一律の `model.slab_thickness`）を用いる。
fn slab_sections(model: &Model, base: u32) -> String {
    let mut body = String::new();
    for slab in &model.slabs {
        let s = sid(base + slab.id.0);
        let t = slab.thickness.unwrap_or(model.slab_thickness);
        body.push_str(&format!(
            "      <StbSecSlab_RC id=\"{}\" name=\"{}\" strength_concrete=\"Fc21\">\n",
            s,
            esc(&format!("S{}", sid(slab.id.0))),
        ));
        body.push_str("        <StbSecFigureSlab_RC>\n");
        body.push_str(&format!(
            "          <StbSecSlab_RC_Straight depth=\"{}\"/>\n",
            fmt(t),
        ));
        body.push_str("        </StbSecFigureSlab_RC>\n");
        body.push_str("      </StbSecSlab_RC>\n");
    }
    body
}

/// 壁断面（`StbSecWall_RC`）ブロックを生成する。壁要素ごとに 1 つの断面を出力し、
/// 厚さは壁の断面（`elem.section.thickness`、未設定は 0）を用いる。
fn wall_sections(model: &Model, base: u32) -> String {
    let mut body = String::new();
    let mut idx = 0u32;
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Wall | ElementKind::Shell) || e.nodes.len() < 3 {
            continue;
        }
        let s = sid(base + idx);
        let t = e
            .section
            .and_then(|sc| model.sections.get(sc.index()))
            .and_then(|sc| sc.thickness)
            .unwrap_or(0.0);
        body.push_str(&format!(
            "      <StbSecWall_RC id=\"{}\" name=\"{}\" strength_concrete=\"Fc21\">\n",
            s,
            esc(&format!("W{}", sid(e.id.0))),
        ));
        body.push_str("        <StbSecFigureWall_RC>\n");
        body.push_str(&format!(
            "          <StbSecWall_RC_Straight thickness=\"{}\"/>\n",
            fmt(t),
        ));
        body.push_str("        </StbSecFigureWall_RC>\n");
        body.push_str("      </StbSecWall_RC>\n");
        idx += 1;
    }
    body
}

pub(super) fn fmt(x: f64) -> String {
    // 整数値は小数点なしで、それ以外は既定の f64 表記で（往復で値が保たれる）。
    if x == x.trunc() && x.is_finite() {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

pub(super) fn esc(s: &str) -> String {
    // XML 1.0 で表現できない C0 制御文字（タブ/改行/CR 以外の #x00-#x1F）は文字参照でも
    // 表せないため除去する。これをしないと不正な XML を出力してしまう。
    let cleaned: String = s
        .chars()
        .filter(|&c| c == '\t' || c == '\n' || c == '\r' || (c as u32) >= 0x20)
        .collect();
    // & を最初に置換した後で制御空白を文字参照化する（後段で `&` を再エスケープしないため安全）。
    // タブ/改行/CR を文字参照にしないと、XML 属性値正規化（読込側 normalized_value）で
    // 空白 (#x20) に潰れ、属性値（例: 断面名・帯筋グレード）が往復で変化してしまう。
    cleaned
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\t', "&#9;")
        .replace('\n', "&#10;")
        .replace('\r', "&#13;")
}
