//! ST-Bridge 標準フォーマットに沿った断面書き出し（[`SectionExportMode::Standard`]）。
//!
//! 既定の [`StbSecRaw`](super::export) は内部物性を直接持つ独自要素で往復に強い一方、
//! BIM や他社ソフトは ST-Bridge 標準の断面要素（`StbSecColumn_S` 等）＋形鋼ライブラリ
//! （`StbSecSteel`）を期待することがある。本モジュールは `Section.shape`
//! （[`SectionShape`]）から標準要素を生成する。
//!
//! # 対応形状
//! - 鋼: H形鋼／角形鋼管／鋼管／山形鋼／溝形鋼／T形鋼（`StbSecSteel` 参照）。
//! - RC: 矩形・円形（幾何＋配筋。配筋は `StbSecBarArrangement*` として書き出す）。
//! - CFT: 角形・円形（充填鋼管を `StbSecColumn_CFT`＋`StbSecSteel` 参照で。柱のみ。梁は Raw）。
//! - SRC: 矩形（`StbSecColumn_SRC`/`StbSecBeam_SRC`。コンクリート図形＋内蔵鉄骨＋配筋＋鋼種）。
//! - 上記以外（耐震壁・形状未定義）は `StbSecRaw` へフォールバックする。
//!
//! # 柱／梁の型分けと id 再割当て
//! ST-Bridge では断面が柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされ、
//! 部材はその断面 id を参照する。内部モデルは 1 断面を柱・梁で共有し得るため、
//! 共有断面は柱用・梁用の 2 要素へ分割し、梁用へ新しい id を割り当てる。
//! 呼び出し側（[`super::export`]）は返り値の id マップで部材の `id_section` を張り替える。
//!
//! # 往復について
//! `import_stbridge` は本モードで書き出した標準断面要素（`StbSecColumn_S`/`StbSecColumn_RC`
//! 等）と配筋（`StbSecBarArrangement*`）を解釈して読み戻せる。ただし ST-Bridge に円形梁の
//! 図形が無いため、**円形（`RcCircle`）を梁として使う断面は `StbSecRaw` にフォールバックし、
//! 形状・配筋は往復しない**（物性のみ残る）。Squid-N 固有属性まで含む完全一致の往復が
//! 要る場合は [`SectionExportMode::Raw`] を使う。

use super::export::{esc, fmt as num};
use squid_n_core::model::{ElementKind, Model, Section};
use squid_n_core::section_shape::{RcRebar, SectionShape};
use std::collections::HashMap;

/// 標準モードで生成した断面ブロックと、部材参照の張り替え用 id マップ。
pub(super) struct StandardSections {
    /// `<StbSections>` の中身（断面要素群 ＋ 末尾の `<StbSecSteel>` ライブラリ）。
    pub sections_xml: String,
    /// 内部断面 id → 柱部材が参照すべき ST-Bridge 断面 id。
    pub col_map: HashMap<u32, u32>,
    /// 内部断面 id → 梁部材が参照すべき ST-Bridge 断面 id。
    pub beam_map: HashMap<u32, u32>,
}

/// 各断面が柱／梁のどちらに使われているかを集計する。
/// 返り値は 内部断面 id → (柱で使用, 梁で使用)。
fn section_roles(model: &Model) -> HashMap<u32, (bool, bool)> {
    let mut roles: HashMap<u32, (bool, bool)> = HashMap::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let Some(sec) = e.section else { continue };
        let n0 = &model.nodes[e.nodes[0].index()];
        let n1 = &model.nodes[e.nodes[1].index()];
        let dz = (n1.coord[2] - n0.coord[2]).abs();
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let is_col = len > 1e-12 && dz / len > 0.707;
        let ent = roles.entry(sec.0).or_insert((false, false));
        if is_col {
            ent.0 = true;
        } else {
            ent.1 = true;
        }
    }
    roles
}

/// 形鋼ライブラリ（`StbSecSteel`）。図形名で重複排除しつつ挿入順を保つ。
#[derive(Default)]
struct SteelLibrary {
    names: std::collections::HashSet<String>,
    entries: Vec<String>,
}

impl SteelLibrary {
    fn add(&mut self, name: &str, entry: String) {
        if self.names.insert(name.to_string()) {
            self.entries.push(entry);
        }
    }
    fn render(&self) -> String {
        if self.entries.is_empty() {
            return String::new();
        }
        let mut s = String::from("      <StbSecSteel>\n");
        for e in &self.entries {
            s.push_str("        ");
            s.push_str(e);
            s.push('\n');
        }
        s.push_str("      </StbSecSteel>\n");
        s
    }
}

/// H 形鋼の形鋼図形名と `StbSecSteel` エントリ（鋼断面・SRC 内蔵鉄骨で共用）。
fn h_figure(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> (String, String) {
    let name = format!(
        "H-{}x{}x{}x{}",
        num(height),
        num(width),
        num(web_thick),
        num(flange_thick)
    );
    let body = format!(
        "<StbSecRoll-H name=\"{}\" type=\"H\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r=\"0\"/>",
        esc(&name),
        num(height),
        num(width),
        num(web_thick),
        num(flange_thick)
    );
    (name, body)
}

/// 角形鋼管の形鋼図形名と `StbSecSteel` エントリ（鋼断面・CFT 角形で共用）。
fn box_figure(height: f64, width: f64, thick: f64) -> (String, String) {
    let name = format!("BOX-{}x{}x{}", num(height), num(width), num(thick));
    let body = format!(
        "<StbSecRoll-BOX name=\"{}\" type=\"BOX\" A=\"{}\" B=\"{}\" t=\"{}\" r=\"0\"/>",
        esc(&name),
        num(height),
        num(width),
        num(thick)
    );
    (name, body)
}

/// 鋼管の形鋼図形名と `StbSecSteel` エントリ（鋼断面・CFT 円形で共用）。
fn pipe_figure(outer_dia: f64, thick: f64) -> (String, String) {
    let name = format!("P-{}x{}", num(outer_dia), num(thick));
    let body = format!(
        "<StbSecPipe name=\"{}\" D=\"{}\" t=\"{}\"/>",
        esc(&name),
        num(outer_dia),
        num(thick)
    );
    (name, body)
}

/// 鋼断面 → 形鋼図形名 と `StbSecSteel` エントリ。対応しない形状は `None`。
fn steel_figure(shape: &SectionShape) -> Option<(String, String)> {
    let e = |name: &str, body: String| (name.to_string(), body);
    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => Some(h_figure(height, width, web_thick, flange_thick)),
        SectionShape::SteelBox {
            height,
            width,
            thick,
        } => Some(box_figure(height, width, thick)),
        SectionShape::SteelPipe { outer_dia, thick } => Some(pipe_figure(outer_dia, thick)),
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            let name = format!("L-{}x{}x{}", num(leg_a), num(leg_b), num(thick));
            let body = format!(
                "<StbSecRoll-L name=\"{}\" type=\"L\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(leg_a),
                num(leg_b),
                num(thick),
                num(thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let name = format!(
                "C-{}x{}x{}x{}",
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            let body = format!(
                "<StbSecRoll-C name=\"{}\" type=\"C\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let name = format!(
                "T-{}x{}x{}x{}",
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            let body = format!(
                "<StbSecRoll-T name=\"{}\" type=\"T\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            Some(e(&name, body))
        }
        _ => None,
    }
}

/// 鋼柱断面 `StbSecColumn_S`。
fn steel_column(id: u32, sec: &Section, figure: &str) -> String {
    format!(
        "      <StbSecColumn_S id=\"{}\" name=\"{}\" kind_column=\"COLUMN\">\n\
         \x20       <StbSecSteelFigureColumn_S>\n\
         \x20         <StbSecSteelColumn_S_Same shape=\"{}\"/>\n\
         \x20       </StbSecSteelFigureColumn_S>\n\
         \x20     </StbSecColumn_S>\n",
        id,
        esc(&sec.name),
        esc(figure)
    )
}

/// 鋼梁断面 `StbSecBeam_S`。
fn steel_beam(id: u32, sec: &Section, figure: &str) -> String {
    format!(
        "      <StbSecBeam_S id=\"{}\" name=\"{}\" kind_beam=\"GIRDER\">\n\
         \x20       <StbSecSteelFigureBeam_S>\n\
         \x20         <StbSecSteelBeam_S_Straight shape=\"{}\"/>\n\
         \x20       </StbSecSteelFigureBeam_S>\n\
         \x20     </StbSecBeam_S>\n",
        id,
        esc(&sec.name),
        esc(figure)
    )
}

/// RC 図形 `StbSecFigureColumn_RC` の中身（矩形／円形）。対応しない形状は `None`。
fn rc_column_figure(shape: &SectionShape) -> Option<String> {
    match *shape {
        SectionShape::RcRect { b, d, .. } => Some(format!(
            "<StbSecColumn_RC_Rect width_X=\"{}\" width_Y=\"{}\"/>",
            num(b),
            num(d)
        )),
        SectionShape::RcCircle { d, .. } => {
            Some(format!("<StbSecColumn_RC_Circle D=\"{}\"/>", num(d)))
        }
        _ => None,
    }
}

/// RC 梁図形 `StbSecFigureBeam_RC` の中身（矩形のみ）。対応しない形状は `None`。
fn rc_beam_figure(shape: &SectionShape) -> Option<String> {
    match *shape {
        SectionShape::RcRect { b, d, .. } => Some(format!(
            "<StbSecBeam_RC_Straight width=\"{}\" depth=\"{}\"/>",
            num(b),
            num(d)
        )),
        _ => None,
    }
}

/// 配筋（[`RcRebar`]）を配筋要素の属性文字列へ整形する。
/// `X`/`Y` は内部モデルの `main_x`（せい方向）/`main_y`（幅方向）主筋セットに対応する
/// ラベルであり、ST-Bridge の平面軸（`width_X`/`width_Y`）とは別概念（Squid 往復用）。
/// 方向別に本数・径・段数を、帯筋（せん断補強筋）に径・ピッチ・組数・材質を、
/// 加えてかぶりを持つ。標準リーダ向けに `dia_main`（単一径）も併記する。
fn rebar_attrs(r: &RcRebar) -> String {
    let mut s = format!(
        "count_main_X=\"{}\" count_main_Y=\"{}\" count_main_layers_X=\"{}\" count_main_layers_Y=\"{}\" \
         dia_main_X=\"{}\" dia_main_Y=\"{}\" dia_main=\"{}\" \
         dia_band=\"{}\" pitch_band=\"{}\" count_band=\"{}\" cover=\"{}\"",
        r.main_x.count,
        r.main_y.count,
        r.main_x.layers,
        r.main_y.layers,
        num(r.main_x.dia),
        num(r.main_y.dia),
        num(r.main_x.dia),
        num(r.shear.dia),
        num(r.shear.pitch),
        r.shear.legs,
        num(r.cover),
    );
    if let Some(g) = &r.shear.grade {
        s.push_str(&format!(" strength_band=\"{}\"", esc(g)));
    }
    s
}

/// RC 柱断面の配筋 `StbSecBarArrangementColumn_RC`（矩形/円形）。配筋の無い形状は空文字。
fn rebar_arrangement_column(shape: &SectionShape) -> String {
    let (child, r) = match shape {
        SectionShape::RcRect { rebar, .. } => ("StbSecBarColumn_RC_RectSame", rebar),
        SectionShape::RcCircle { rebar, .. } => ("StbSecBarColumn_RC_CircleSame", rebar),
        _ => return String::new(),
    };
    format!(
        "        <StbSecBarArrangementColumn_RC>\n\
         \x20         <{} {}/>\n\
         \x20       </StbSecBarArrangementColumn_RC>\n",
        child,
        rebar_attrs(r)
    )
}

/// RC 梁断面の配筋 `StbSecBarArrangementBeam_RC`（矩形）。配筋の無い形状は空文字。
fn rebar_arrangement_beam(shape: &SectionShape) -> String {
    let r = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        _ => return String::new(),
    };
    format!(
        "        <StbSecBarArrangementBeam_RC>\n\
         \x20         <StbSecBarBeam_RC_Same {}/>\n\
         \x20       </StbSecBarArrangementBeam_RC>\n",
        rebar_attrs(r)
    )
}

/// RC 柱断面 `StbSecColumn_RC`（図形＋配筋）。
fn rc_column(id: u32, sec: &Section, shape: &SectionShape, figure_body: &str) -> String {
    format!(
        "      <StbSecColumn_RC id=\"{}\" name=\"{}\">\n\
         \x20       <StbSecFigureColumn_RC>\n\
         \x20         {}\n\
         \x20       </StbSecFigureColumn_RC>\n\
         {}\
         \x20     </StbSecColumn_RC>\n",
        id,
        esc(&sec.name),
        figure_body,
        rebar_arrangement_column(shape),
    )
}

/// RC 梁断面 `StbSecBeam_RC`（図形＋配筋）。
fn rc_beam(id: u32, sec: &Section, shape: &SectionShape, figure_body: &str) -> String {
    format!(
        "      <StbSecBeam_RC id=\"{}\" name=\"{}\">\n\
         \x20       <StbSecFigureBeam_RC>\n\
         \x20         {}\n\
         \x20       </StbSecFigureBeam_RC>\n\
         {}\
         \x20     </StbSecBeam_RC>\n",
        id,
        esc(&sec.name),
        figure_body,
        rebar_arrangement_beam(shape),
    )
}

/// CFT 断面の充填鋼管図形（角形/円形）。`SteelLibrary` に登録し、参照名を返す。
/// CFT 以外は `None`。
fn cft_figure(shape: &SectionShape, steel: &mut SteelLibrary) -> Option<String> {
    let (name, body) = match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => box_figure(height, width, thick),
        SectionShape::CftPipe { outer_dia, thick } => pipe_figure(outer_dia, thick),
        _ => return None,
    };
    steel.add(&name, body);
    Some(name)
}

/// CFT 柱断面 `StbSecColumn_CFT`（充填鋼管の形鋼参照）。
fn cft_column(id: u32, sec: &Section, figure: &str) -> String {
    format!(
        "      <StbSecColumn_CFT id=\"{}\" name=\"{}\">\n\
         \x20       <StbSecSteelFigureColumn_CFT>\n\
         \x20         <StbSecSteelColumn_CFT_Same shape=\"{}\"/>\n\
         \x20       </StbSecSteelFigureColumn_CFT>\n\
         \x20     </StbSecColumn_CFT>\n",
        id,
        esc(&sec.name),
        esc(figure)
    )
}

/// SRC 断面の内蔵鉄骨（H 形鋼）図形。`SteelLibrary` に登録し、参照名を返す。SRC 以外は `None`。
fn src_steel_figure(shape: &SectionShape, steel: &mut SteelLibrary) -> Option<String> {
    match *shape {
        SectionShape::SrcRect {
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            ..
        } => {
            let (name, body) = h_figure(
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            );
            steel.add(&name, body);
            Some(name)
        }
        _ => None,
    }
}

/// SRC 柱／梁断面 `StbSecColumn_SRC` / `StbSecBeam_SRC`
/// （コンクリート図形＋内蔵鉄骨参照＋配筋＋鋼種）。
fn src_section(
    id: u32,
    sec: &Section,
    is_beam: bool,
    shape: &SectionShape,
    steel_fig: &str,
) -> String {
    let (b, d, rebar_arrangement, grade) = match shape {
        SectionShape::SrcRect {
            b, d, steel_grade, ..
        } => (
            *b,
            *d,
            rebar_arrangement_generic(shape, is_beam, "SRC"),
            steel_grade.clone(),
        ),
        // 呼び出し側で SrcRect のみ渡す想定。防御的に空で返す。
        _ => return raw(id, sec),
    };
    let (elem, fig_wrap, fig_body, steel_wrap) = if is_beam {
        (
            "StbSecBeam_SRC",
            "StbSecFigureBeam_SRC",
            format!(
                "<StbSecBeam_SRC_Straight width=\"{}\" depth=\"{}\"/>",
                num(b),
                num(d)
            ),
            "StbSecSteelFigureBeam_SRC",
        )
    } else {
        (
            "StbSecColumn_SRC",
            "StbSecFigureColumn_SRC",
            format!(
                "<StbSecColumn_SRC_Rect width_X=\"{}\" width_Y=\"{}\"/>",
                num(b),
                num(d)
            ),
            "StbSecSteelFigureColumn_SRC",
        )
    };
    let steel_same = if is_beam {
        "StbSecSteelBeam_SRC_Same"
    } else {
        "StbSecSteelColumn_SRC_Same"
    };
    format!(
        "      <{elem} id=\"{id}\" name=\"{name}\" strength_steel=\"{grade}\">\n\
         \x20       <{fig_wrap}>\n\
         \x20         {fig_body}\n\
         \x20       </{fig_wrap}>\n\
         \x20       <{steel_wrap}>\n\
         \x20         <{steel_same} shape=\"{steel_fig}\"/>\n\
         \x20       </{steel_wrap}>\n\
         {rebar_arrangement}\
         \x20     </{elem}>\n",
        elem = elem,
        id = id,
        name = esc(&sec.name),
        grade = esc(&grade),
        fig_wrap = fig_wrap,
        fig_body = fig_body,
        steel_wrap = steel_wrap,
        steel_same = steel_same,
        steel_fig = esc(steel_fig),
        rebar_arrangement = rebar_arrangement,
    )
}

/// SRC の配筋要素 `StbSecBarArrangement{Column,Beam}_SRC`。配筋の無い形状は空文字。
/// `kind` は要素名の中置（"SRC"）。
fn rebar_arrangement_generic(shape: &SectionShape, is_beam: bool, kind: &str) -> String {
    let r = match shape {
        SectionShape::SrcRect { rebar, .. } => rebar,
        _ => return String::new(),
    };
    let (wrap, child) = if is_beam {
        (
            format!("StbSecBarArrangementBeam_{kind}"),
            format!("StbSecBarBeam_{kind}_Same"),
        )
    } else {
        (
            format!("StbSecBarArrangementColumn_{kind}"),
            format!("StbSecBarColumn_{kind}_RectSame"),
        )
    };
    format!(
        "        <{}>\n\
         \x20         <{} {}/>\n\
         \x20       </{}>\n",
        wrap,
        child,
        rebar_attrs(r),
        wrap
    )
}

/// `StbSecRaw` フォールバック（既定モードと同一形式）。
fn raw(id: u32, sec: &Section) -> String {
    format!(
        "      <StbSecRaw id=\"{}\" name=\"{}\" area=\"{}\" iy=\"{}\" iz=\"{}\" j=\"{}\" depth=\"{}\" width=\"{}\"/>\n",
        id,
        esc(&sec.name),
        num(sec.area),
        num(sec.iy),
        num(sec.iz),
        num(sec.j),
        num(sec.depth),
        num(sec.width),
    )
}

/// 標準モードの `<StbSections>` 本体と、部材参照の張り替え用 id マップを生成する。
pub(super) fn standard_sections(model: &Model) -> StandardSections {
    let roles = section_roles(model);
    // 梁用の分割断面へ割り当てる追加 id は、既存 id の最大値の次から採番する。
    let mut next_id = model.sections.iter().map(|s| s.id.0).max().unwrap_or(0) + 1;
    let mut alloc = || {
        let v = next_id;
        next_id += 1;
        v
    };

    let mut steel = SteelLibrary::default();
    let mut body = String::new();
    let mut col_map: HashMap<u32, u32> = HashMap::new();
    let mut beam_map: HashMap<u32, u32> = HashMap::new();

    for sec in &model.sections {
        let base = sec.id.0;
        let (used_col, used_beam) = roles.get(&base).copied().unwrap_or((false, false));
        // どの部材からも参照されない断面も出力に残す（既定で柱扱い）。
        let need_col = used_col || !used_beam;
        let need_beam = used_beam;

        // 形状から標準要素を試み、不可なら StbSecRaw へフォールバック。
        let steel_fig = sec.shape.as_ref().and_then(steel_figure);
        if let Some((fig_name, fig_body)) = steel_fig {
            steel.add(&fig_name, fig_body);
            if need_col {
                body.push_str(&steel_column(base, sec, &fig_name));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if need_col { alloc() } else { base };
                body.push_str(&steel_beam(bid, sec, &fig_name));
                beam_map.insert(base, bid);
            }
            continue;
        }

        // CFT（充填鋼管）: 柱として StbSecColumn_CFT。ST-Bridge に CFT 梁が無いため
        // 梁で使われる場合は Raw へフォールバックする。
        if matches!(
            sec.shape,
            Some(SectionShape::CftBox { .. } | SectionShape::CftPipe { .. })
        ) {
            let shape = sec.shape.as_ref().unwrap();
            if need_col {
                let fig = cft_figure(shape, &mut steel).expect("CFT 図形");
                body.push_str(&cft_column(base, sec, &fig));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if col_map.contains_key(&base) {
                    alloc()
                } else {
                    base
                };
                body.push_str(&raw(bid, sec));
                beam_map.insert(base, bid);
            }
            continue;
        }

        // SRC（RC＋内蔵鉄骨）: 柱 StbSecColumn_SRC / 梁 StbSecBeam_SRC。
        if matches!(sec.shape, Some(SectionShape::SrcRect { .. })) {
            let shape = sec.shape.as_ref().unwrap();
            let steel_fig = src_steel_figure(shape, &mut steel).expect("SRC 内蔵鉄骨図形");
            if need_col {
                body.push_str(&src_section(base, sec, false, shape, &steel_fig));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if col_map.contains_key(&base) {
                    alloc()
                } else {
                    base
                };
                body.push_str(&src_section(bid, sec, true, shape, &steel_fig));
                beam_map.insert(base, bid);
            }
            continue;
        }

        let rc_col_fig = sec.shape.as_ref().and_then(rc_column_figure);
        let rc_beam_fig = sec.shape.as_ref().and_then(rc_beam_figure);
        if rc_col_fig.is_some() || rc_beam_fig.is_some() {
            let shape = sec.shape.as_ref().expect("RC 図形がある＝shape は Some");
            if need_col {
                // 円形など梁図形が無い場合も柱としては出力できる。
                if let Some(fig) = &rc_col_fig {
                    body.push_str(&rc_column(base, sec, shape, fig));
                    col_map.insert(base, base);
                }
            }
            if need_beam {
                if let Some(fig) = &rc_beam_fig {
                    let bid = if col_map.contains_key(&base) {
                        alloc()
                    } else {
                        base
                    };
                    body.push_str(&rc_beam(bid, sec, shape, fig));
                    beam_map.insert(base, bid);
                } else {
                    // 梁で使われるが梁図形に落ちない形状（例: RC 円形）は Raw で残す。
                    let bid = if col_map.contains_key(&base) {
                        alloc()
                    } else {
                        base
                    };
                    body.push_str(&raw(bid, sec));
                    beam_map.insert(base, bid);
                }
            }
            // 柱でも梁でも使われない RC 断面は need_col で拾えているが、
            // 梁図形しか無い（RcRect を柱に使わない）ケースでも need_col=true のとき
            // rc_col_fig=Some なので出力済み。念のため未出力なら Raw で残す。
            if !col_map.contains_key(&base) && !beam_map.contains_key(&base) {
                body.push_str(&raw(base, sec));
                col_map.insert(base, base);
                beam_map.insert(base, base);
            }
            continue;
        }

        // フォールバック: SRC・CFT・耐震壁・形状未定義。Raw は柱/梁で型分けされないため
        // 両者とも同一 id を参照する。
        body.push_str(&raw(base, sec));
        col_map.insert(base, base);
        beam_map.insert(base, base);
    }

    let mut sections_xml = body;
    sections_xml.push_str(&steel.render());

    StandardSections {
        sections_xml,
        col_map,
        beam_map,
    }
}
