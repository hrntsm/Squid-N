//! ST-Bridge パース（Import）。設計書 §12.5。
//!
//! [`import_stbridge`] は ST-Bridge 標準スキーマ（2.0.2）の断面要素
//! （`StbSecColumn_S`/`StbSecBeam_S`/`StbSecColumn_RC`/`StbSecBeam_RC`/`StbSecColumn_CFT`/
//! `StbSecColumn_SRC`/`StbSecBeam_SRC`/`StbSecSlab_RC`/`StbSecSlabDeck`/`StbSecWall_RC`）＋
//! 形鋼ライブラリ（`StbSecSteel`）を解釈する。形鋼名から内部の [`SectionShape`] を復元し、
//! 断面性能を再算定する。材料は断面のグレード名（鋼 `strength_main`、RC/SRC/CFT の
//! `strength_concrete`）から標準材料表（[`material_std`]）で物性へ解決する。
//! 後方互換のため、Squid-N が過去に書き出した物性直持ち `StbSecRaw` も読み取れる。
//!
//! 標準断面は柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされ、柱・梁共有断面の
//! 分割などで断面 id が文書順に整列しないことがある。取り込み後に断面 id を整列・再採番し、
//! 部材の断面参照（`id_section`）を張り替える。node/material/story/section/element の id が
//! 1 始まりや歯抜けでも 0 始まり連番へ正規化する。

use super::StbError;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, SlabId, StoryId};
use squid_n_core::model::{
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis,
    Material, Model, NodalLoad, Node, Section, Slab, Story,
};
use squid_n_core::section_shape::{RcRebar, SectionShape};
use std::collections::HashMap;

mod material_std;
mod rebar;
mod steel;
mod xml;

use rebar::{default_rebar, parse_rebar};
use steel::steel_shape_from;
use xml::{attrs, get_f64, get_f64_any, get_i64, get_opt_f64, get_u32, push_node_id_tokens};

/// 断面が持つ材料参照（ST-Bridge は材料を断面側に持つ）。
/// 数値 id（RC/CFT/SRC の `id_material`）または材料名（鋼の `strength_main` グレード）。
#[derive(Clone)]
enum SecMatRef {
    Id(u32),
    Grade(String),
}

/// 取り込み途中の断面（id 整列・形鋼名解決の前）。
struct PendingSec {
    file_id: u32,
    name: String,
    kind: PendingSecKind,
    /// 断面側に付いた材料参照（部材が id_material を持たないとき部材へ伝播する）。
    mat: Option<SecMatRef>,
}

enum PendingSecKind {
    /// 物性直持ち（`StbSecRaw`）。
    Raw {
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
        depth: f64,
        width: f64,
    },
    /// 形状が確定済み（RC 図形など）。
    Shape(SectionShape),
    /// 形鋼ライブラリ参照（後で名前解決する鋼断面）。
    SteelRef(Option<String>),
    /// CFT（充填鋼管）。充填鋼管の形鋼名を後で解決して CftBox/CftPipe を作る。
    CftRef(Option<String>),
    /// SRC（RC＋内蔵鉄骨）。コンクリート寸法・配筋・鋼種は確定済み、内蔵鉄骨は
    /// 形鋼名を後で解決する。
    SrcRef {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_name: Option<String>,
        grade: String,
    },
}

/// 取り込み途中の部材の種別。
enum PendingMemberKind {
    Beam,
    Brace { tension_only: bool },
}

/// 取り込み途中の部材（id 正規化前。参照はすべて file id）。
struct PendingMember {
    kind: PendingMemberKind,
    n_i: u32,
    n_j: u32,
    section: Option<u32>,
    material: Option<u32>,
    /// `id_material` 属性がファイルに存在したか。存在する（=-1 含む）場合は部材が材料を
    /// 明示しているとみなし、断面材料の伝播を行わない（往復で None→Some 化を防ぐ）。
    /// 属性が無い場合のみ断面材料を伝播する。
    has_material_attr: bool,
    /// 部材軸まわりの断面回転角 [deg]（ST-Bridge `rotate`）。ref_vector は節点座標が
    /// 揃う構築時に軸と `rotate` から算出する。
    rotate: f64,
    /// 部材端の接合条件 [i, j]（`condition_bottom`/`top`・`condition_start`/`end`）。
    end_cond: [EndCondition; 2],
}

/// 取り込み途中の二次部材（小梁 `StbBeam`・間柱 `StbPost`。id 正規化前）。
/// 全体解析の対象外で、床荷重・自重を主架構へ CMQ として伝達する部材
/// （`squid_n_core::model::SecondaryMember`）として取り込む。
struct PendingSecondary {
    kind: squid_n_core::model::SecondaryMemberKind,
    n_i: u32,
    n_j: u32,
    section: Option<u32>,
    material: Option<u32>,
    has_material_attr: bool,
    name: String,
}

/// 取り込み途中の節点（id 正規化前）。
struct RawNode {
    file_id: u32,
    coord: [f64; 3],
}

/// 取り込み途中の層（id 正規化前）。
struct RawStory {
    file_id: u32,
    name: String,
    elevation: f64,
    /// `StbStory` 直下 `StbNodeIdList/StbNodeId` が示す所属節点（file node id 列）。
    node_ids: Vec<u32>,
}

/// 取り込み途中の材料（id 正規化前）。
struct RawMaterial {
    file_id: u32,
    name: String,
    young: f64,
    poisson: f64,
    density: f64,
    shear: Option<f64>,
    fc: Option<f64>,
    fy: Option<f64>,
}

/// 取り込み途中の荷重ケース（節点参照は file id）。
struct RawLoadCase {
    name: String,
    nodal: Vec<(u32, [f64; 6])>,
}

/// 取り込み途中のスラブ（節点参照は file id。`StbSlab` + `StbNodeIdOrder`）。
struct RawSlab {
    /// 断面参照（`id_section`。`StbSecSlab_RC` の file id）。負値/未指定は `None`。
    section_fid: Option<u32>,
    /// 境界節点ループ（`StbNodeIdOrder`。file node id 列）。
    boundary: Vec<u32>,
}

/// 取り込み途中の壁（節点参照は file id。`StbWall` + `StbNodeIdOrder`）。
struct RawWall {
    /// 断面参照（`id_section`。`StbSecWall_RC` の file id）。負値/未指定は `None`。
    section_fid: Option<u32>,
    /// 材料参照（`id_material`）。負値/未指定は `None`。
    material_fid: Option<u32>,
    /// 境界節点ループ（`StbNodeIdOrder`。file node id 列）。
    boundary: Vec<u32>,
}

/// RC 断面の図形（配筋と組み合わせて `SectionShape` を確定する）。
enum RcGeom {
    Rect { b: f64, d: f64 },
    Circle { d: f64 },
}

/// 現在パース中の標準断面要素（子の図形・配筋要素を集める）。
enum CurSec {
    None,
    Steel {
        file_id: u32,
        name: String,
        shape_name: Option<String>,
        grade: Option<String>,
    },
    Rc {
        file_id: u32,
        name: String,
        geom: Option<RcGeom>,
        rebar: Option<RcRebar>,
        /// 配筋コンテナ（`StbSecBarArrangement*`）側に付くかぶり [mm]。実 ST-Bridge は
        /// かぶりをコンテナに、本数・径を子の `*_Same` に持つため別枠で控える。
        rebar_cover: Option<f64>,
        /// 断面のコンクリート材料（数値 id または `strength_concrete` グレード名）。
        mat: Option<SecMatRef>,
    },
    Cft {
        file_id: u32,
        name: String,
        steel_name: Option<String>,
        mat: Option<SecMatRef>,
    },
    Src {
        file_id: u32,
        name: String,
        geom: Option<(f64, f64)>,
        rebar: Option<RcRebar>,
        /// 配筋コンテナ側に付くかぶり [mm]（[`CurSec::Rc`] と同じ）。
        rebar_cover: Option<f64>,
        steel_name: Option<String>,
        grade: String,
        mat: Option<SecMatRef>,
    },
    /// RC スラブ断面（`StbSecSlab_RC`）。子の図形要素から厚さを集める。
    Slab {
        file_id: u32,
        thickness: Option<f64>,
    },
    /// RC 壁断面（`StbSecWall_RC`）。子の図形要素から厚さを集める。
    Wall {
        file_id: u32,
        thickness: Option<f64>,
    },
}

/// 取り込み時に欠落・近似した内容の報告（データ欠損を顕在化させる）。
#[derive(Debug, Default, Clone)]
pub struct ImportReport {
    /// 人間可読の警告メッセージ（未対応要素のスキップ、断面欠落、参照解決失敗など）。
    pub warnings: Vec<String>,
    /// 取り込み時に自動補完した仮定の通知（支点の自動設定など）。
    /// データ欠損ではないため [`is_clean`](Self::is_clean) には影響しないが、
    /// ユーザーへ明示すべき内容として呼び出し側で表示する。
    pub notes: Vec<String>,
}

impl ImportReport {
    /// 警告が 1 件も無い（＝取り込みで欠落が無かった）か。
    /// 自動補完の通知（`notes`）は欠落ではないため判定に含めない。
    pub fn is_clean(&self) -> bool {
        self.warnings.is_empty()
    }
}

/// ST-Bridge の要素のうち Squid-N が未対応で、取り込み時に必ず警告対象とするもの。
/// これに加え、部材（`StbMembers`）・断面（`StbSections`）・荷重（`StbLoadCase`）の直属子で
/// 未対応のものは、このリストに無い未知要素であっても警告する（fail-loud。詳細は取り込み
/// ループの `other` 分岐を参照）。本リストは、直属の親からは判別しづらい要素（通り芯など
/// `StbModel` 直下のもの）を確実に拾うために併用する。
const UNSUPPORTED_ELEMENTS: &[&str] = &[
    // 通り芯（Model 直下。grid/axis 概念が無く往復対象外）
    "StbAxes",
    // 部材（面要素・基礎・開口。StbSlab・StbWall は対応済み）
    "StbFooting",
    "StbPile",
    "StbFoundationColumn",
    "StbStripFooting",
    "StbParapet",
    "StbOpen",
    // 断面（基礎・開口。鋼ブレース断面 StbSecBrace_S・StbSecSlab_RC・StbSecWall_RC・
    // デッキ合成スラブ StbSecSlabDeck は対応済み。鋼スラブ StbSecSlab_S は未対応）
    "StbSecSlab_S",
    "StbSecFoundation_RC",
    "StbSecFoundationColumn_RC",
    "StbSecFoundationColumn_SRC",
    "StbSecFoundationColumn_CFT",
    "StbSecPile_RC",
    "StbSecPile_S",
    "StbSecPile_PC",
    "StbSecParapet_RC",
    "StbSecOpen_RC",
];

/// `StbMembers` 直下の部材グループコンテナ（複数形）。実 ST-Bridge は部材を
/// `StbMembers > StbColumns > StbColumn` のように複数形コンテナへ入れ子にする
/// （Squid 方言は `StbMembers > StbColumn` と直下に置く）。これらのコンテナ自体は
/// 単なる入れ物なので未対応警告の対象にせず、その直属子で未対応のものだけを
/// 「取り込み対象外」として拾う（fail-loud。取り込みループの `other` 分岐を参照）。
const MEMBER_GROUP_CONTAINERS: &[&str] = &[
    "StbColumns",
    "StbPosts",
    "StbGirders",
    "StbBeams",
    "StbBraces",
    "StbSlabs",
    "StbWalls",
    "StbFootings",
    "StbStripFootings",
    "StbFoundationColumns",
    "StbPiles",
    "StbParapets",
    "StbOpens",
];

/// ST-Bridge ファイルを読み込み、UTF-8 文字列へデコードする。
///
/// 日本の建築業界では ST-Bridge が Shift_JIS（Windows-31J / CP932）で
/// 保存されるケースが多いため、次の順で判定する（BOM の有無は問わない）。
/// まず UTF-8 BOM 付き、または UTF-8 として妥当なら UTF-8 として扱い、
/// それ以外は Shift_JIS（Windows-31J）としてデコードする。
/// 読み込み自体の失敗 (存在しない等) は [`StbError::Io`] として返す。
pub fn read_stbridge_file(path: &std::path::Path) -> Result<String, StbError> {
    use encoding_rs::SHIFT_JIS;
    let bytes =
        std::fs::read(path).map_err(|e| StbError::Io(format!("{}: {e}", path.display())))?;

    // BOM 付き UTF-8 はそのまま UTF-8 扱い。
    if bytes.starts_with(&[0xEF, 0xBB, 0xBF]) {
        return String::from_utf8(bytes[3..].to_vec())
            .map_err(|e| StbError::Decode(format!("UTF-8 デコードエラー: {e}")));
    }
    // UTF-8 として妥当ならそのまま扱う（ASCII や既存の UTF-8 ファイル互換）。
    if let Ok(s) = String::from_utf8(bytes.clone()) {
        return Ok(s);
    }
    // それ以外は Shift_JIS（Windows-31J / CP932）としてデコードする。
    let (cow, _, had_errors) = SHIFT_JIS.decode(&bytes);
    if had_errors {
        return Err(StbError::Decode(
            "ファイルを UTF-8 または Shift_JIS としてデコードできませんでした".to_string(),
        ));
    }
    Ok(cow.into_owned())
}

/// ST-Bridge 2.0 XML を内部モデルへ取り込む（欠落の報告は破棄する）。
pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    import_stbridge_with_report(xml).map(|(m, _)| m)
}

/// ST-Bridge 2.0 XML を内部モデルへ取り込み、[`ImportReport`]（欠落・近似の警告）も返す。
pub fn import_stbridge_with_report(xml: &str) -> Result<(Model, ImportReport), StbError> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut version_ok = false;
    let mut warnings: Vec<String> = Vec::new();
    // 未対応要素はタグごとに件数を集計し、最後にまとめて 1 行の警告にする。
    // 明示リストに無い未知の要素も、部材/断面/荷重の直属子であれば「取り込み対象外」
    // として拾う（fail-loud。取りこぼしを無言で捨てない）ため、キーは String とする。
    let mut unsupported: HashMap<String, u32> = HashMap::new();
    // 開いている要素のスタック（直属の親要素を知り、未知の部材/断面/荷重を検出するため）。
    let mut container_stack: Vec<String> = Vec::new();

    // 全要素を一旦 file id 付きの中間表現へ集め、パース後に id を 0 始まり連番へ
    // 正規化して参照を張り替える（他社ファイルの 1 始まり・歯抜け id に対応）。
    let mut raw_nodes: Vec<RawNode> = Vec::new();
    let mut raw_stories: Vec<RawStory> = Vec::new();
    let mut raw_materials: Vec<RawMaterial> = Vec::new();
    let mut raw_load_cases: Vec<RawLoadCase> = Vec::new();
    let mut pending_secs: Vec<PendingSec> = Vec::new();
    let mut pending_members: Vec<PendingMember> = Vec::new();
    let mut pending_secondaries: Vec<PendingSecondary> = Vec::new();
    let mut steel_lib: HashMap<String, SectionShape> = HashMap::new();
    let mut cur = CurSec::None;
    // スラブ・壁関連の中間状態。
    let mut raw_slabs: Vec<RawSlab> = Vec::new();
    let mut slab_sec_thickness: HashMap<u32, f64> = HashMap::new();
    let mut cur_slab: Option<RawSlab> = None;
    let mut raw_walls: Vec<RawWall> = Vec::new();
    let mut wall_sec_thickness: HashMap<u32, f64> = HashMap::new();
    let mut cur_wall: Option<RawWall> = None;
    let mut in_node_id_order = false;
    // 実 ST-Bridge の `StbStory`（内部に `StbNodeIdList/StbNodeId` を持つ）を開いている間 true。
    // 開いている `StbNodeId` を直近の階の所属節点として集めるために使う。
    let mut in_story = false;

    loop {
        let ev = reader
            .read_event()
            .map_err(|e| StbError::Parse(e.to_string()))?;
        // 自己終了要素（<Foo/>）は End が来ないためスタックへ積まない。
        let is_empty = matches!(ev, Event::Empty(_));
        match ev {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                let a = attrs(&e)?;
                // この要素の直属の親（未知の部材/断面/荷重の検出に使う）。
                let parent = container_stack.last().map(|s| s.as_str());
                // StbNodeIdOrder のテキストは開始タグ直後の Text/CData のみで届く。
                // 別要素が現れた時点で取り込み窓を閉じる（自己終了タグ
                // <StbNodeIdOrder/> は End が来ずフラグが残るため、この明示リセットで
                // 無関係な子要素のテキストを境界へ誤取り込みするのを防ぐ）。
                if tag != "StbNodeIdOrder" {
                    in_node_id_order = false;
                }
                match tag.as_str() {
                    "ST_BRIDGE" => {
                        let v = a.get("version").cloned().unwrap_or_default();
                        if !v.starts_with("2.") {
                            return Err(StbError::Version(v));
                        }
                        version_ok = true;
                    }
                    "StbNode" => {
                        raw_nodes.push(RawNode {
                            file_id: get_u32(&a, "id")?,
                            // 座標は ST-Bridge 標準の大文字 `X`/`Y`/`Z`。
                            coord: [get_f64(&a, "X")?, get_f64(&a, "Y")?, get_f64(&a, "Z")?],
                        });
                    }
                    "StbStory" => {
                        raw_stories.push(RawStory {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            elevation: get_f64(&a, "height")?,
                            node_ids: Vec::new(),
                        });
                        // 直下の StbNodeIdList/StbNodeId をこの階へ集める窓を開く
                        // （空の <StbStory/> でも害は無い。StbNodeId はスラブ・壁を優先し、
                        // かつ階は通常部材より前に現れるため誤取り込みしない）。
                        in_story = true;
                    }
                    "StbMaterial" => {
                        raw_materials.push(RawMaterial {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            young: get_f64(&a, "young")?,
                            poisson: get_f64(&a, "poisson")?,
                            density: get_f64(&a, "density")?,
                            shear: get_opt_f64(&a, "shear"),
                            fc: get_opt_f64(&a, "fc"),
                            fy: get_opt_f64(&a, "fy"),
                        });
                    }
                    // --- 断面: 物性直持ち（Raw） ---
                    "StbSecRaw" => {
                        pending_secs.push(PendingSec {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            kind: PendingSecKind::Raw {
                                area: get_f64(&a, "area")?,
                                iy: get_f64(&a, "iy")?,
                                iz: get_f64(&a, "iz")?,
                                j: get_f64(&a, "j")?,
                                depth: get_f64(&a, "depth").unwrap_or(0.0),
                                width: get_f64(&a, "width").unwrap_or(0.0),
                            },
                            mat: None,
                        });
                    }
                    // --- 断面: 標準要素（鋼。柱・梁・ブレース） ---
                    "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                        cur = CurSec::Steel {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            shape_name: None,
                            // 鋼種は形鋼参照（下）に付くことが多いが、要素側にあれば拾う。
                            grade: a.get("strength_main").cloned(),
                        };
                    }
                    // 鋼／CFT／SRC 断面の図形参照（`*_Same` / `*_Straight`）。`shape` 系属性から
                    // 形鋼名を、`strength_main` から鋼種を取り、現在の断面種別へ格納する。
                    tag if tag.starts_with("StbSecSteelColumn_")
                        || tag.starts_with("StbSecSteelBeam_")
                        || tag.starts_with("StbSecSteelBrace_") =>
                    {
                        let sname = a
                            .get("shape")
                            .or_else(|| a.get("shape_start"))
                            .or_else(|| a.get("shape_center"))
                            .or_else(|| a.get("shape_main"))
                            .cloned();
                        let gr = a.get("strength_main").cloned();
                        match &mut cur {
                            CurSec::Steel {
                                shape_name, grade, ..
                            } => {
                                if shape_name.is_none() {
                                    *shape_name = sname;
                                }
                                if grade.is_none() {
                                    *grade = gr;
                                }
                            }
                            CurSec::Cft { steel_name, .. } if steel_name.is_none() => {
                                *steel_name = sname
                            }
                            CurSec::Src { steel_name, .. } if steel_name.is_none() => {
                                *steel_name = sname
                            }
                            _ => {}
                        }
                    }
                    // --- 断面: 標準要素（RC） ---
                    "StbSecColumn_RC" | "StbSecBeam_RC" => {
                        cur = CurSec::Rc {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            geom: None,
                            rebar: None,
                            rebar_cover: None,
                            mat: sec_mat_ref_of(&a),
                        };
                    }
                    "StbSecColumn_RC_Rect" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Rect {
                                    b: get_f64_any(&a, &["width_X", "width_x"])?,
                                    d: get_f64_any(&a, &["width_Y", "width_y"])?,
                                });
                            }
                        }
                    }
                    "StbSecColumn_RC_Circle" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Circle {
                                    d: get_f64_any(&a, &["D", "d"])?,
                                });
                            }
                        }
                    }
                    "StbSecBeam_RC_Straight" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Rect {
                                    b: get_f64_any(&a, &["width", "width_X"])?,
                                    d: get_f64_any(&a, &["depth", "width_Y"])?,
                                });
                            }
                        }
                    }
                    // --- 断面: 標準要素（CFT） ---
                    "StbSecColumn_CFT" => {
                        cur = CurSec::Cft {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            steel_name: None,
                            mat: sec_mat_ref_of(&a),
                        };
                    }
                    // --- 断面: 標準要素（SRC） ---
                    "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                        cur = CurSec::Src {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            geom: None,
                            rebar: None,
                            rebar_cover: None,
                            steel_name: None,
                            grade: a
                                .get("strength_steel")
                                .or_else(|| a.get("strength_main_S"))
                                .cloned()
                                .unwrap_or_default(),
                            mat: sec_mat_ref_of(&a),
                        };
                    }
                    "StbSecColumn_SRC_Rect" => {
                        if let CurSec::Src { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some((
                                    get_f64_any(&a, &["width_X", "width_x"])?,
                                    get_f64_any(&a, &["width_Y", "width_y"])?,
                                ));
                            }
                        }
                    }
                    "StbSecBeam_SRC_Straight" => {
                        if let CurSec::Src { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some((
                                    get_f64_any(&a, &["width", "width_X"])?,
                                    get_f64_any(&a, &["depth", "width_Y"])?,
                                ));
                            }
                        }
                    }
                    // 配筋コンテナ（`StbSecBarArrangement*`）。実 ST-Bridge はかぶり
                    // （`depth_cover_*`）を配置コンテナ側に、本数・径を子の `*_Same` 側に持つ。
                    // 本数・径は下の `*_Same` 分岐で拾うため、ここではかぶりのみを控える。
                    tag if tag.starts_with("StbSecBarArrangement") => {
                        if let Ok(c) = get_f64_any(
                            &a,
                            &[
                                "depth_cover",
                                "depth_cover_top",
                                "depth_cover_bottom",
                                "depth_cover_start",
                                "depth_cover_start_X",
                                "depth_cover_end_X",
                                "depth_cover_start_Y",
                                "cover",
                                "kaburi",
                            ],
                        ) {
                            match &mut cur {
                                CurSec::Rc { rebar_cover, .. } => *rebar_cover = Some(c),
                                CurSec::Src { rebar_cover, .. } => *rebar_cover = Some(c),
                                _ => {}
                            }
                        }
                    }
                    // 配筋（RC / SRC の StbSecBar{Column,Beam}_*_Same 子要素）。現在の断面種別へ格納。
                    tag if tag.starts_with("StbSecBarColumn_")
                        || tag.starts_with("StbSecBarBeam_") =>
                    {
                        match &mut cur {
                            CurSec::Rc { rebar, .. } if rebar.is_none() => {
                                *rebar = Some(parse_rebar(&a))
                            }
                            CurSec::Src { rebar, .. } if rebar.is_none() => {
                                *rebar = Some(parse_rebar(&a))
                            }
                            _ => {}
                        }
                    }
                    // --- 形鋼ライブラリ ---
                    _ if tag.starts_with("StbSecRoll-")
                        || tag.starts_with("StbSecBuild-")
                        || tag == "StbSecPipe" =>
                    {
                        if let (Some(nm), Some(shape)) =
                            (a.get("name").cloned(), steel_shape_from(&tag, &a))
                        {
                            steel_lib.entry(nm).or_insert(shape);
                        }
                    }
                    // --- 部材 ---
                    "StbColumn" => {
                        let bot = get_u32(&a, "id_node_bottom")?;
                        let top = get_u32(&a, "id_node_top")?;
                        pending_members.push(make_member(&a, bot, top, PendingMemberKind::Beam)?);
                    }
                    "StbGirder" => {
                        let st = get_u32(&a, "id_node_start")?;
                        let en = get_u32(&a, "id_node_end")?;
                        pending_members.push(make_member(&a, st, en, PendingMemberKind::Beam)?);
                    }
                    // 小梁（StbBeam）は二次部材: 全体解析の対象外とし、床荷重・自重を
                    // 大梁へ CMQ（中間集中荷重）として伝達する部材として取り込む。
                    "StbBeam" => {
                        let st = get_u32(&a, "id_node_start")?;
                        let en = get_u32(&a, "id_node_end")?;
                        pending_secondaries.push(make_secondary(
                            &a,
                            st,
                            en,
                            squid_n_core::model::SecondaryMemberKind::Joist,
                        ));
                    }
                    // 間柱（StbPost）も二次部材（鉛直材。柱と同じく bottom/top を持つ。
                    // start/end も許容）。
                    "StbPost" => {
                        let bot = get_u32(&a, "id_node_bottom")
                            .or_else(|_| get_u32(&a, "id_node_start"))?;
                        let top =
                            get_u32(&a, "id_node_top").or_else(|_| get_u32(&a, "id_node_end"))?;
                        pending_secondaries.push(make_secondary(
                            &a,
                            bot,
                            top,
                            squid_n_core::model::SecondaryMemberKind::Post,
                        ));
                    }
                    "StbBrace" => {
                        let st = get_u32(&a, "id_node_start")?;
                        let en = get_u32(&a, "id_node_end")?;
                        // `feature_brace`（既定 TENSION）。TENSIONANDCOMPRESSION のみ
                        // 引張圧縮両用、それ以外（TENSION・未指定）は引張専用。
                        let tension_only = a
                            .get("feature_brace")
                            .map(|v| v != "TENSIONANDCOMPRESSION")
                            .unwrap_or(true);
                        pending_members.push(make_member(
                            &a,
                            st,
                            en,
                            PendingMemberKind::Brace { tension_only },
                        )?);
                    }
                    "StbLoadCase" => {
                        raw_load_cases.push(RawLoadCase {
                            name: a.get("name").cloned().unwrap_or_default(),
                            nodal: vec![],
                        });
                    }
                    "StbNodalLoad" => {
                        let node = get_u32(&a, "id_node")?;
                        let values = [
                            get_f64(&a, "fx").unwrap_or(0.0),
                            get_f64(&a, "fy").unwrap_or(0.0),
                            get_f64(&a, "fz").unwrap_or(0.0),
                            get_f64(&a, "mx").unwrap_or(0.0),
                            get_f64(&a, "my").unwrap_or(0.0),
                            get_f64(&a, "mz").unwrap_or(0.0),
                        ];
                        if let Some(lc) = raw_load_cases.last_mut() {
                            lc.nodal.push((node, values));
                        } else {
                            return Err(StbError::Parse("StbNodalLoad outside StbLoadCase".into()));
                        }
                    }
                    // --- スラブ断面: RC（StbSecSlab_RC）／デッキ合成（StbSecSlabDeck）。
                    //     厚さ（コンクリート部せい）を図形の子要素から集める。 ---
                    "StbSecSlab_RC" | "StbSecSlabDeck" => {
                        cur = CurSec::Slab {
                            file_id: get_u32(&a, "id")?,
                            thickness: get_f64_any(&a, &["depth", "thickness", "t", "D"]).ok(),
                        };
                    }
                    // スラブ断面の図形（厚さ = `depth`）。RC・デッキ双方の図形要素を受ける。
                    "StbSecSlab_RC_Straight"
                    | "StbSecFigureSlab_RC"
                    | "StbSecSlabDeckStraight"
                    | "StbSecFigureSlabDeck" => {
                        if let CurSec::Slab { thickness, .. } = &mut cur {
                            // 厚さ属性を持つ図形要素なら更新、無ければ既存値を保持。
                            *thickness = get_f64_any(&a, &["depth", "thickness", "t", "D"])
                                .ok()
                                .or(*thickness);
                        }
                    }
                    // --- 壁断面（StbSecWall_RC）: 厚さを子要素から集める ---
                    "StbSecWall_RC" => {
                        cur = CurSec::Wall {
                            file_id: get_u32(&a, "id")?,
                            thickness: get_f64_any(&a, &["thickness", "t", "depth", "D"]).ok(),
                        };
                    }
                    "StbSecWall_RC_Straight" | "StbSecFigureWall_RC" => {
                        if let CurSec::Wall { thickness, .. } = &mut cur {
                            *thickness = get_f64_any(&a, &["thickness", "t", "depth", "D"])
                                .ok()
                                .or(*thickness);
                        }
                    }
                    // --- スラブ（StbSlab）: 境界節点ループを StbNodeIdOrder から集める ---
                    "StbSlab" => {
                        // 自己終了 <StbWall/> 等で残った兄弟状態をクリアし、境界ノードの
                        // 取り違えを防ぐ（StbSlab/StbWall は入れ子にならない）。
                        cur_wall = None;
                        cur_slab = Some(RawSlab {
                            section_fid: match get_i64(&a, "id_section") {
                                Some(s) if s >= 0 => Some(s as u32),
                                _ => None,
                            },
                            boundary: Vec::new(),
                        });
                    }
                    // --- 壁（StbWall）: 境界節点ループを StbNodeIdOrder から集める ---
                    "StbWall" => {
                        cur_slab = None;
                        cur_wall = Some(RawWall {
                            section_fid: match get_i64(&a, "id_section") {
                                Some(s) if s >= 0 => Some(s as u32),
                                _ => None,
                            },
                            material_fid: match get_i64(&a, "id_material") {
                                Some(s) if s >= 0 => Some(s as u32),
                                _ => None,
                            },
                            boundary: Vec::new(),
                        });
                    }
                    "StbNodeIdOrder" => {
                        in_node_id_order = true;
                    }
                    // 節点ループを子要素形式（<StbNodeId id="…"/>）で持つ方言に対応。
                    // スラブ・壁のうち現在開いている方の境界へ追加する。
                    "StbNodeId" => {
                        if let Ok(id) = get_u32(&a, "id") {
                            if let Some(slab) = cur_slab.as_mut() {
                                slab.boundary.push(id);
                            } else if let Some(wall) = cur_wall.as_mut() {
                                wall.boundary.push(id);
                            } else if in_story {
                                if let Some(story) = raw_stories.last_mut() {
                                    story.node_ids.push(id);
                                }
                            }
                        }
                    }
                    // 未対応の要素はデータ欠落として集計する（fail-loud）。明示リストに加え、
                    // 部材グループコンテナ（StbColumns 等）の直属子・断面（StbSections、ただし
                    // 形鋼ライブラリコンテナ StbSecSteel は除く）・荷重（StbLoadCase）の直属子で
                    // 未対応のものは、リスト外の未知要素であっても「取り込み対象外」として拾う。
                    // グループコンテナ自体（StbColumns 等）や StbMembers は入れ物なので拾わない。
                    other => {
                        let is_group_container =
                            other == "StbMembers" || MEMBER_GROUP_CONTAINERS.contains(&other);
                        let parent_is_member_group =
                            parent.is_some_and(|p| MEMBER_GROUP_CONTAINERS.contains(&p));
                        let skipped_data = !is_group_container
                            && (UNSUPPORTED_ELEMENTS.contains(&other)
                                || parent_is_member_group
                                || (matches!(parent, Some("StbSections"))
                                    && other != "StbSecSteel")
                                || matches!(parent, Some("StbLoadCase")));
                        if skipped_data {
                            *unsupported.entry(other.to_string()).or_insert(0) += 1;
                        }
                    }
                }
                // 開始要素はスタックへ積む（自己終了要素は End が来ないため積まない）。
                if !is_empty {
                    container_stack.push(tag);
                }
            }
            Event::End(e) => {
                // 対応する開始要素をスタックから降ろす。
                container_stack.pop();
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                match tag.as_str() {
                    "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                        if let CurSec::Steel {
                            file_id,
                            name,
                            shape_name,
                            grade,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            pending_secs.push(PendingSec {
                                file_id,
                                name,
                                kind: PendingSecKind::SteelRef(shape_name),
                                mat: grade.map(SecMatRef::Grade),
                            });
                        }
                    }
                    "StbSecColumn_RC" | "StbSecBeam_RC" => {
                        if let CurSec::Rc {
                            file_id,
                            name,
                            geom,
                            rebar,
                            rebar_cover,
                            mat,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            match geom {
                                Some(geom) => {
                                    // 配筋が無い（幾何のみの）ファイルは無筋相当の既定配筋で補う。
                                    let mut rebar = rebar.unwrap_or_else(default_rebar);
                                    // かぶりが配筋要素側に無ければ配置コンテナ側の値を採る。
                                    if rebar.cover == 0.0 {
                                        if let Some(c) = rebar_cover {
                                            rebar.cover = c;
                                        }
                                    }
                                    let shape = match geom {
                                        RcGeom::Rect { b, d } => {
                                            SectionShape::RcRect { b, d, rebar }
                                        }
                                        RcGeom::Circle { d } => SectionShape::RcCircle { d, rebar },
                                    };
                                    pending_secs.push(PendingSec {
                                        file_id,
                                        name,
                                        kind: PendingSecKind::Shape(shape),
                                        mat,
                                    });
                                }
                                None => warnings.push(format!(
                                    "RC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした（テーパ・ハンチ等は未対応）"
                                )),
                            }
                        }
                    }
                    "StbSecColumn_CFT" => {
                        if let CurSec::Cft {
                            file_id,
                            name,
                            steel_name,
                            mat,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            pending_secs.push(PendingSec {
                                file_id,
                                name,
                                kind: PendingSecKind::CftRef(steel_name),
                                mat,
                            });
                        }
                    }
                    "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                        if let CurSec::Src {
                            file_id,
                            name,
                            geom,
                            rebar,
                            rebar_cover,
                            steel_name,
                            grade,
                            mat,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            match geom {
                                Some((b, d)) => {
                                    let mut rebar = rebar.unwrap_or_else(default_rebar);
                                    if rebar.cover == 0.0 {
                                        if let Some(c) = rebar_cover {
                                            rebar.cover = c;
                                        }
                                    }
                                    pending_secs.push(PendingSec {
                                        file_id,
                                        name,
                                        mat,
                                        kind: PendingSecKind::SrcRef {
                                            b,
                                            d,
                                            rebar,
                                            steel_name,
                                            grade,
                                        },
                                    });
                                }
                                None => warnings.push(format!(
                                    "SRC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした"
                                )),
                            }
                        }
                    }
                    "StbSecSlab_RC" | "StbSecSlabDeck" => {
                        // 厚さが取れたスラブ断面のみ登録する（cur は必ず None へ戻す）。
                        if let CurSec::Slab {
                            file_id,
                            thickness: Some(t),
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            slab_sec_thickness.insert(file_id, t);
                        }
                    }
                    "StbSecWall_RC" => {
                        if let CurSec::Wall {
                            file_id,
                            thickness: Some(t),
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            wall_sec_thickness.insert(file_id, t);
                        }
                    }
                    "StbNodeIdOrder" => {
                        in_node_id_order = false;
                    }
                    "StbStory" => {
                        in_story = false;
                    }
                    "StbSlab" => {
                        if let Some(slab) = cur_slab.take() {
                            raw_slabs.push(slab);
                        }
                    }
                    "StbWall" => {
                        if let Some(wall) = cur_wall.take() {
                            raw_walls.push(wall);
                        }
                    }
                    _ => {}
                }
            }
            // StbNodeIdOrder のテキスト内容（空白区切りの節点 id 列）を集める。
            // 節点 id は数字と空白のみで XML 実体参照を含まないため、そのまま UTF-8
            // 解釈でよい。CDATA 形式（<![CDATA[0 1 2 3]]>）にも対応する。
            Event::Text(t) if in_node_id_order => {
                let boundary = cur_slab
                    .as_mut()
                    .map(|s| &mut s.boundary)
                    .or(cur_wall.as_mut().map(|w| &mut w.boundary));
                if let Some(b) = boundary {
                    push_node_id_tokens(&String::from_utf8_lossy(t.as_ref()), b);
                }
            }
            Event::CData(t) if in_node_id_order => {
                let boundary = cur_slab
                    .as_mut()
                    .map(|s| &mut s.boundary)
                    .or(cur_wall.as_mut().map(|w| &mut w.boundary));
                if let Some(b) = boundary {
                    push_node_id_tokens(&String::from_utf8_lossy(t.as_ref()), b);
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

    let mut model = Model::default();

    // 各 id 空間を file id 昇順の 0 始まり連番へ正規化する（内部モデルの不変条件
    // 「配列添字 == id.index()」を満たすため）。返り値は file id → 新 index。
    let node_index = build_index(raw_nodes.iter().map(|n| n.file_id));
    let story_index = build_index(raw_stories.iter().map(|s| s.file_id));
    let material_index = build_index(raw_materials.iter().map(|m| m.file_id));

    // file id の一意性を検証する（fail-loud）。build_index は id を重複排除するが、
    // raw_* は排除しないまま push されるため、重複 id があると model.nodes 等の
    // 配列長が index 数を超え「配列添字 == id.index()」の不変条件が壊れ、部材が
    // 別実体の節点/断面/材料を無言で参照する（ジオメトリ破損）。重複はエラーとする。
    check_unique_ids("StbNode", raw_nodes.iter().map(|n| n.file_id), &node_index)?;
    check_unique_ids(
        "StbStory",
        raw_stories.iter().map(|s| s.file_id),
        &story_index,
    )?;
    check_unique_ids(
        "StbMaterial/StbSecColumn_S ほか材料",
        raw_materials.iter().map(|m| m.file_id),
        &material_index,
    )?;

    // 実 ST-Bridge の階所属（StbStory/StbNodeIdList）から file node id → file story id を作る。
    // 節点の所属階は、まず節点自身の `story` 属性（Squid 方言）を優先し、無ければこの表を引く。
    let node_story_from_list: HashMap<u32, u32> = raw_stories
        .iter()
        .flat_map(|s| {
            let sid = s.file_id;
            s.node_ids.iter().map(move |&nid| (nid, sid))
        })
        .collect();

    raw_stories.sort_by_key(|s| s.file_id);
    for s in raw_stories {
        // 階の所属節点を正規化後の NodeId へ解決する（存在しない節点は除外）。
        let node_ids = s
            .node_ids
            .iter()
            .filter_map(|fid| node_index.get(fid).copied().map(NodeId))
            .collect();
        model.stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(story_index[&s.file_id]),
            name: s.name,
            elevation: s.elevation,
            node_ids,
            diaphragms: vec![],
            seismic_weight: None,
        });
    }

    raw_nodes.sort_by_key(|n| n.file_id);
    for n in raw_nodes {
        model.nodes.push(Node {
            id: NodeId(node_index[&n.file_id]),
            coord: n.coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            // 節点の所属階は `StbStory/StbNodeIdList` から引く（標準スキーマ）。
            story: node_story_from_list
                .get(&n.file_id)
                .and_then(|sfid| story_index.get(sfid).copied())
                .map(StoryId),
        });
    }

    // 念のため、節点の story から Story.node_ids を補完する
    // （StbNodeIdList 由来との重複は除く）。
    for node in &model.nodes {
        if let Some(sid) = node.story {
            let list = &mut model.stories[sid.index()].node_ids;
            if !list.contains(&node.id) {
                list.push(node.id);
            }
        }
    }

    raw_materials.sort_by_key(|m| m.file_id);
    for m in raw_materials {
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(material_index[&m.file_id]),
            name: m.name,
            young: m.young,
            poisson: m.poisson,
            density: m.density,
            shear: m.shear,
            fc: m.fc,
            fy: m.fy,
        });
    }

    // ST-Bridge 2.0 の StbModel は材料テーブル（E・ν・密度）を持たず、材料は断面に付く
    // グレード名（コンクリート `Fc21`、鋼種 `SN400B`、鉄筋 `SD345` 等）で表す。日本の
    // 構造材料は規格化されており名前が物性を一意に定めるため、断面が参照するグレード名を
    // 標準材料表で物性へ解決し、同名の材料がまだ無ければ材料として追加する。
    {
        use std::collections::HashSet;
        let mut existing: HashSet<String> =
            model.materials.iter().map(|m| m.name.clone()).collect();
        // 文書順で決定的に列挙し、重複名は最初の 1 回だけ追加する。
        let mut grades: Vec<&str> = Vec::new();
        for p in &pending_secs {
            if let Some(SecMatRef::Grade(name)) = &p.mat {
                if !name.is_empty() && !grades.contains(&name.as_str()) {
                    grades.push(name.as_str());
                }
            }
        }
        for name in grades {
            if existing.contains(name) {
                continue;
            }
            if let Some(std) = material_std::resolve_grade(name) {
                let id = MaterialId(model.materials.len() as u32);
                model.materials.push(Material {
                    concrete_class: Default::default(),
                    id,
                    name: name.to_string(),
                    young: std.young,
                    poisson: std.poisson,
                    density: std.density,
                    shear: None,
                    fc: std.fc,
                    fy: std.fy,
                });
                existing.insert(name.to_string());
            }
        }
    }

    // 断面側の材料参照を、部材への伝播用に file id → 正規化後 material index へ解決する。
    // ST-Bridge は材料を断面に持つため、部材が id_material を持たない（実 STB 相当の）
    // 場合に断面の材料を部材へ伝播する。数値 id は material_index、鋼のグレード名は
    // 同名の材料へ突き合わせる（同名複数は最初の一致）。
    let mut name_to_mat: HashMap<&str, u32> = HashMap::new();
    for mat in &model.materials {
        name_to_mat.entry(mat.name.as_str()).or_insert(mat.id.0);
    }
    let section_material: HashMap<u32, u32> = pending_secs
        .iter()
        .filter_map(|p| {
            let idx = match p.mat.as_ref()? {
                SecMatRef::Id(mid) => material_index.get(mid).copied(),
                SecMatRef::Grade(name) => name_to_mat.get(name.as_str()).copied(),
            };
            idx.map(|i| (p.file_id, i))
        })
        .collect();

    // 断面 id を整列・連番へ再割当てし、形鋼名を解決してモデルへ格納する。
    let section_index = build_sections(&mut model, pending_secs, &steel_lib, &mut warnings);

    // 部材を格納する（節点・断面・材料の参照を正規化後の index に張り替える）。
    // 参照先が存在しない部材はスキップし、断面/材料の欠落は None にしてダングリングを防ぐ。
    let mut skipped_members = 0u32;
    let mut dangling_section = 0u32;
    let mut dangling_material = 0u32;
    for m in pending_members {
        let (Some(&ni), Some(&nj)) = (node_index.get(&m.n_i), node_index.get(&m.n_j)) else {
            skipped_members += 1;
            continue;
        };
        // 断面参照: 実在しない id を指していれば警告して None にする（ダングリング防止）。
        let section = m.section.and_then(|fid| match section_index.get(&fid) {
            Some(&idx) => Some(SectionId(idx)),
            None => {
                dangling_section += 1;
                None
            }
        });
        // 材料は部材自身の id_material を優先。id_material 属性が無い（実 ST-Bridge 相当の）
        // ときのみ断面の材料を伝播する（属性がある部材の None を上書きしない）。
        let own_material = m.material.and_then(|fid| match material_index.get(&fid) {
            Some(&idx) => Some(idx),
            None => {
                dangling_material += 1;
                None
            }
        });
        let material = own_material
            .or_else(|| {
                if m.has_material_attr {
                    None
                } else {
                    m.section
                        .and_then(|sfid| section_material.get(&sfid).copied())
                }
            })
            .map(MaterialId);
        let id = ElemId(model.elements.len() as u32);
        // 梁・柱は端部接合条件（`condition_*`）を尊重し、ブレースは軸材なので両端ピン。
        let (kind, end_cond) = match m.kind {
            PendingMemberKind::Beam => (ElementKind::Beam, m.end_cond),
            PendingMemberKind::Brace { tension_only } => (
                ElementKind::Brace { tension_only },
                [EndCondition::Pinned, EndCondition::Pinned],
            ),
        };
        // ref_vector は部材軸（節点座標）と `rotate` から算出する。
        let ref_vector = ref_vector_from_rotate(
            model.nodes[ni as usize].coord,
            model.nodes[nj as usize].coord,
            m.rotate,
        );
        model.elements.push(ElementData {
            id,
            kind,
            nodes: smallvec::smallvec![NodeId(ni), NodeId(nj)],
            section,
            material,
            local_axis: LocalAxis { ref_vector },
            end_cond,
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    // 二次部材（小梁・間柱）を格納する（節点・断面・材料の参照は部材と同じ規則で
    // 正規化・伝播する）。全体解析の対象外（CMQ 用）のため `model.elements` には
    // 入れず `model.secondary_members` に入れる。
    let mut n_joists = 0usize;
    let mut n_posts = 0usize;
    for s in pending_secondaries {
        let (Some(&ni), Some(&nj)) = (node_index.get(&s.n_i), node_index.get(&s.n_j)) else {
            skipped_members += 1;
            continue;
        };
        let section = s.section.and_then(|fid| match section_index.get(&fid) {
            Some(&idx) => Some(SectionId(idx)),
            None => {
                dangling_section += 1;
                None
            }
        });
        let own_material = s.material.and_then(|fid| match material_index.get(&fid) {
            Some(&idx) => Some(idx),
            None => {
                dangling_material += 1;
                None
            }
        });
        let material = own_material
            .or_else(|| {
                if s.has_material_attr {
                    None
                } else {
                    s.section
                        .and_then(|sfid| section_material.get(&sfid).copied())
                }
            })
            .map(MaterialId);
        match s.kind {
            squid_n_core::model::SecondaryMemberKind::Joist => n_joists += 1,
            squid_n_core::model::SecondaryMemberKind::Post => n_posts += 1,
        }
        model
            .secondary_members
            .push(squid_n_core::model::SecondaryMember {
                kind: s.kind,
                nodes: [NodeId(ni), NodeId(nj)],
                section,
                material,
                name: s.name,
            });
    }

    if skipped_members > 0 {
        warnings.push(format!(
            "存在しない節点を参照する部材を {skipped_members} 件スキップしました"
        ));
    }
    if dangling_section > 0 {
        warnings.push(format!(
            "存在しない断面を参照する部材が {dangling_section} 件あり、断面リンクを外しました"
        ));
    }
    if dangling_material > 0 {
        warnings.push(format!(
            "存在しない材料を参照する部材が {dangling_material} 件あり、材料リンクを外しました"
        ));
    }

    // スラブ（StbSlab）。境界節点を正規化し、断面参照から厚さを解決する。
    // 3 頂点未満・存在しない節点を含むスラブはスキップして報告する。
    //
    // ST-Bridge は床荷重（仕上げ・積載）を持たないため、厚さが分かるスラブには
    // 自重（厚さ × γRC=24kN/m³。デッキ合成もコンクリート主体の近似）を固定荷重
    // として自動設定する（DL・CMQ・地震用重量へスラブ自重が算入される出発点。
    // 仕上げ・用途（積載）は荷重タブでの設定が必要。notes で通知する）。
    let mut skipped_slabs = 0u32;
    let mut slab_self_weight_count = 0usize;
    for rs in raw_slabs {
        let mut boundary = Vec::with_capacity(rs.boundary.len());
        let mut resolved = true;
        for fid in &rs.boundary {
            match node_index.get(fid) {
                Some(&ni) => boundary.push(NodeId(ni)),
                None => {
                    resolved = false;
                    break;
                }
            }
        }
        if !resolved || boundary.len() < 3 {
            skipped_slabs += 1;
            continue;
        }
        let thickness = rs
            .section_fid
            .and_then(|fid| slab_sec_thickness.get(&fid).copied());
        let loads = match thickness {
            Some(t) if t > 0.0 => {
                slab_self_weight_count += 1;
                vec![squid_n_core::model::AreaLoad {
                    kind: "自重(自動)".into(),
                    value: t * squid_n_core::units::to_internal::unit_weight_kn_per_m3(24.0),
                }]
            }
            _ => Vec::new(),
        };
        let new_id = SlabId(model.slabs.len() as u32);
        model.slabs.push(Slab {
            id: new_id,
            boundary,
            joists: Vec::new(),
            loads,
            method: DistributionMethod::TriTrapezoid,
            kind: Default::default(),
            one_way: None,
            edge_supported: None,
            usage: None,
            thickness,
        });
    }
    if skipped_slabs > 0 {
        warnings.push(format!(
            "境界節点が解決できない、または頂点数が不足するスラブを {skipped_slabs} 件スキップしました"
        ));
    }

    // 壁（StbWall）。境界節点を正規化し、壁要素（ElementKind::Wall）として取り込む。
    // 厚さ（StbSecWall_RC）は t>0 のとき厚さ専用の Section を末尾に追加して参照する
    // （壁自重は section.thickness を用いるため）。3頂点未満・存在しない節点を含む壁は
    // スキップして報告する。
    let mut skipped_walls = 0u32;
    for rw in raw_walls {
        let mut boundary: smallvec::SmallVec<[NodeId; 8]> =
            smallvec::SmallVec::with_capacity(rw.boundary.len());
        let mut resolved = true;
        for fid in &rw.boundary {
            match node_index.get(fid) {
                Some(&ni) => boundary.push(NodeId(ni)),
                None => {
                    resolved = false;
                    break;
                }
            }
        }
        if !resolved || boundary.len() < 3 {
            skipped_walls += 1;
            continue;
        }
        // 厚さ >0 のときのみ厚さ専用断面を作成して参照する。
        let section = rw
            .section_fid
            .and_then(|fid| wall_sec_thickness.get(&fid).copied())
            .filter(|t| *t > 0.0)
            .map(|t| {
                let sid = SectionId(model.sections.len() as u32);
                model.sections.push(Section {
                    id: sid,
                    name: format!("Wall t{}", t),
                    area: 0.0,
                    iy: 0.0,
                    iz: 0.0,
                    j: 0.0,
                    depth: 0.0,
                    width: 0.0,
                    as_y: 0.0,
                    as_z: 0.0,
                    panel_thickness: None,
                    thickness: Some(t),
                    shape: None,
                });
                sid
            });
        let material = rw
            .material_fid
            .and_then(|fid| material_index.get(&fid).copied())
            .map(MaterialId);
        let id = ElemId(model.elements.len() as u32);
        model.elements.push(ElementData {
            id,
            kind: ElementKind::Wall,
            nodes: boundary,
            section,
            material,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    if skipped_walls > 0 {
        warnings.push(format!(
            "境界節点が解決できない、または頂点数が不足する壁を {skipped_walls} 件スキップしました"
        ));
    }

    // 荷重ケース（節点参照を正規化。存在しない節点への荷重は破棄）。
    let mut dropped_loads = 0u32;
    for (i, lc) in raw_load_cases.into_iter().enumerate() {
        let nodal = lc
            .nodal
            .into_iter()
            .filter_map(|(fid, values)| match node_index.get(&fid) {
                Some(&ni) => Some(NodalLoad {
                    node: NodeId(ni),
                    values,
                }),
                None => {
                    dropped_loads += 1;
                    None
                }
            })
            .collect();
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(i as u32),
            name: lc.name,
            nodal,
            member: vec![],
        });
    }
    if dropped_loads > 0 {
        warnings.push(format!(
            "存在しない節点への節点荷重を {dropped_loads} 件破棄しました"
        ));
    }

    // 未対応要素の集計を 1 行の警告にまとめる（タグ名昇順で決定的に）。
    if !unsupported.is_empty() {
        let mut items: Vec<(&String, &u32)> = unsupported.iter().collect();
        items.sort_by(|a, b| a.0.cmp(b.0));
        let list = items
            .iter()
            .map(|(tag, n)| format!("{tag}×{n}"))
            .collect::<Vec<_>>()
            .join(", ");
        warnings.push(format!("未対応の要素をスキップしました: {list}"));
    }

    // 支点の自動設定: ST-Bridge は境界条件（支点）を持たないため、支点が 1 つも
    // 無いモデルは最下レベル（Z 最小、許容差 1mm）の節点をピン支点
    // （並進固定・回転自由）に設定する（柱脚ピンの仮定＝基礎の回転拘束を
    // 期待しない安全側の既定。解析可能な出発点にする）。
    // 仮定した内容は notes で通知する。拘束を 1 つでも持つモデル（将来の方言
    // 拡張等で取り込んだ場合）はそのまま尊重して何もしない。
    let mut notes: Vec<String> = Vec::new();
    if n_joists + n_posts > 0 {
        notes.push(format!(
            "小梁 {n_joists} 本・間柱 {n_posts} 本を二次部材として取り込みました\
            （全体解析の対象外。床荷重・自重は大梁への集中荷重（CMQ）として伝達します）"
        ));
    }
    if slab_self_weight_count > 0 {
        notes.push(format!(
            "スラブ {slab_self_weight_count} 枚に自重（厚さ×24kN/m³）を床荷重として設定しました\
            （仕上げ荷重・用途（積載）は荷重タブで設定してください）"
        ));
    }
    {
        use squid_n_core::dof::Dof6Mask;
        if !model.nodes.is_empty() && model.nodes.iter().all(|n| n.restraint == Dof6Mask::FREE) {
            const BASE_LEVEL_TOL_MM: f64 = 1.0;
            let z_min = model
                .nodes
                .iter()
                .map(|n| n.coord[2])
                .fold(f64::INFINITY, f64::min);
            let mut fixed = 0usize;
            for n in &mut model.nodes {
                if (n.coord[2] - z_min).abs() <= BASE_LEVEL_TOL_MM {
                    n.restraint = Dof6Mask::PINNED;
                    fixed += 1;
                }
            }
            notes.push(format!(
                "支点情報が無いため、最下レベル（Z={z_min:.0} mm）の節点 {fixed} 箇所をピン支点に設定しました（モデルタブ→境界条件で変更できます）"
            ));
        }
    }

    Ok((model, ImportReport { warnings, notes }))
}

/// file id が一意であることを検証する（fail-loud）。要素数が重複排除後の
/// index 数を超えていれば重複 id ありとしてエラーを返す。
fn check_unique_ids(
    kind: &str,
    ids: impl Iterator<Item = u32>,
    index: &HashMap<u32, u32>,
) -> Result<(), StbError> {
    let count = ids.count();
    if count > index.len() {
        return Err(StbError::Parse(format!(
            "{kind} の file id が重複しています（{count} 要素に対し一意 id は {} 個）。\
             id は一意である必要があります。",
            index.len()
        )));
    }
    Ok(())
}

/// file id の集合を昇順・重複排除して 0 始まり連番へ写像する（file id → 新 index）。
fn build_index(ids: impl Iterator<Item = u32>) -> HashMap<u32, u32> {
    let mut sorted: Vec<u32> = ids.collect();
    sorted.sort_unstable();
    sorted.dedup();
    sorted
        .into_iter()
        .enumerate()
        .map(|(i, id)| (id, i as u32))
        .collect()
}

/// 保留していた断面を id 昇順に整列・連番へ再割当てし、形鋼名を解決して
/// `model.sections` を構築する。返り値は 元の file id → 再割当て後 index のマップ。
fn build_sections(
    model: &mut Model,
    mut pending: Vec<PendingSec>,
    steel_lib: &HashMap<String, SectionShape>,
    warnings: &mut Vec<String>,
) -> HashMap<u32, u32> {
    // file id 昇順で整列（Standard 書き出しは分割断面を文書順に整列させないため）。
    pending.sort_by_key(|s| s.file_id);

    let mut index_map: HashMap<u32, u32> = HashMap::new();
    for (idx, ps) in pending.into_iter().enumerate() {
        let new_id = SectionId(idx as u32);
        index_map.insert(ps.file_id, idx as u32);
        let section = match ps.kind {
            PendingSecKind::Raw {
                area,
                iy,
                iz,
                j,
                depth,
                width,
            } => Section {
                id: new_id,
                name: ps.name,
                area,
                iy,
                iz,
                j,
                depth,
                width,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            },
            PendingSecKind::Shape(shape) => shape.to_section(new_id, ps.name),
            PendingSecKind::SteelRef(shape_name) => {
                // 形鋼ライブラリに定義が無い参照は物性ゼロの断面として残す
                // （参照する部材の断面リンクを保つため。解析前に要確認）。
                match shape_name.and_then(|nm| steel_lib.get(&nm).cloned()) {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "鋼断面 (name=\"{}\") の形鋼参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::CftRef(steel_name) => {
                // 充填鋼管の形鋼（BOX/Pipe）を CFT 形状へ読み替える。
                let cft = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelBox {
                            height,
                            width,
                            thick,
                            ..
                        } => Some(SectionShape::CftBox {
                            height,
                            width,
                            thick,
                        }),
                        SectionShape::SteelPipe { outer_dia, thick } => {
                            Some(SectionShape::CftPipe { outer_dia, thick })
                        }
                        _ => None,
                    });
                match cft {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "CFT 断面 (name=\"{}\") の充填鋼管参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::SrcRef {
                b,
                d,
                rebar,
                steel_name,
                grade,
            } => {
                // 内蔵鉄骨（H 形鋼）の寸法を解決する。未解決なら 0 とし、形状は保持する。
                let steel_dims = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelH {
                            height,
                            width,
                            web_thick,
                            flange_thick,
                        } => Some((height, width, web_thick, flange_thick)),
                        _ => None,
                    });
                if steel_dims.is_none() {
                    warnings.push(format!(
                        "SRC 断面 (name=\"{}\") の内蔵鉄骨参照を解決できず鉄骨寸法ゼロで取り込みました",
                        ps.name
                    ));
                }
                let (sh, sw, sweb, sfl) = steel_dims.unwrap_or((0.0, 0.0, 0.0, 0.0));
                SectionShape::SrcRect {
                    b,
                    d,
                    rebar,
                    steel_height: sh,
                    steel_width: sw,
                    steel_web_thick: sweb,
                    steel_flange_thick: sfl,
                    steel_grade: grade,
                }
                .to_section(new_id, ps.name)
            }
        };
        model.sections.push(section);
    }
    index_map
}

/// 物性ゼロ・形状なしの断面（形鋼名未解決などのフォールバック。解析前に要確認）。
fn zero_section(id: SectionId, name: String) -> Section {
    Section {
        id,
        name,
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    }
}

/// RC/SRC/CFT 断面のコンクリート材料参照。数値 id（`id_material` 系）を優先し、
/// 無ければ ST-Bridge 標準のグレード名 `strength_concrete`（`Fc21` 等）を採る。
fn sec_mat_ref_of(a: &HashMap<String, String>) -> Option<SecMatRef> {
    let id = get_i64(a, "id_material")
        .or_else(|| get_i64(a, "id_material_concrete"))
        .or_else(|| get_i64(a, "id_material_rc"))
        .filter(|v| *v >= 0);
    if let Some(v) = id {
        return Some(SecMatRef::Id(v as u32));
    }
    a.get("strength_concrete")
        .filter(|s| !s.is_empty())
        .cloned()
        .map(SecMatRef::Grade)
}

fn make_member(
    a: &HashMap<String, String>,
    n_i: u32,
    n_j: u32,
    kind: PendingMemberKind,
) -> Result<PendingMember, StbError> {
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(s as u32),
        _ => None,
    };
    let has_material_attr = a.contains_key("id_material");
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(m as u32),
        _ => None,
    };
    // 断面回転角（`rotate`、既定 0）。ref_vector は構築時に軸から算出する。
    let rotate = get_f64(a, "rotate").unwrap_or(0.0);
    // 端部接合条件（柱は bottom/top、大梁・小梁は start/end。既定は FIX）。
    let end_cond = [
        end_condition_of(a, &["condition_bottom", "condition_start"]),
        end_condition_of(a, &["condition_top", "condition_end"]),
    ];
    Ok(PendingMember {
        kind,
        n_i,
        n_j,
        section,
        material,
        has_material_attr,
        rotate,
        end_cond,
    })
}

/// 二次部材（小梁 `StbBeam`・間柱 `StbPost`）の中間表現を作る
/// （[`make_member`] の二次部材版。端部接合条件・回転角は解析に使わないため持たない）。
fn make_secondary(
    a: &HashMap<String, String>,
    n_i: u32,
    n_j: u32,
    kind: squid_n_core::model::SecondaryMemberKind,
) -> PendingSecondary {
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(s as u32),
        _ => None,
    };
    let has_material_attr = a.contains_key("id_material");
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(m as u32),
        _ => None,
    };
    PendingSecondary {
        kind,
        n_i,
        n_j,
        section,
        material,
        has_material_attr,
        name: a.get("name").cloned().unwrap_or_default(),
    }
}

/// 部材端の接合条件属性（`FIX`/`PIN`）を [`EndCondition`] へ写す。既定・未知は `Fixed`。
fn end_condition_of(a: &HashMap<String, String>, keys: &[&str]) -> EndCondition {
    for k in keys {
        if let Some(v) = a.get(*k) {
            return match v.as_str() {
                "PIN" => EndCondition::Pinned,
                _ => EndCondition::Fixed,
            };
        }
    }
    EndCondition::Fixed
}

/// 部材軸（`p_i`→`p_j`）まわりに断面回転角 `rotate` [deg] を適用した ref_vector を返す。
/// `rotate=0` の基準は、水平材は鉛直上（グローバル Z）、鉛直材はグローバル X 方向。
/// これは従来の既定 ref_vector と同一の局所座標系を与える（水平材で [0,0,1]）。
fn ref_vector_from_rotate(p_i: [f64; 3], p_j: [f64; 3], rotate_deg: f64) -> [f64; 3] {
    let axis = {
        let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-9 {
            [0.0, 0.0, 1.0]
        } else {
            [d[0] / l, d[1] / l, d[2] / l]
        }
    };
    // 軸が鉛直に近ければ基準を X、そうでなければ Z にとる。
    let base = if axis[2].abs() > 0.99 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    // base を軸に直交化して rotate=0 の基準 ref0 を得る。
    let bdot = base[0] * axis[0] + base[1] * axis[1] + base[2] * axis[2];
    let ref0 = {
        let r = [
            base[0] - bdot * axis[0],
            base[1] - bdot * axis[1],
            base[2] - bdot * axis[2],
        ];
        let l = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        if l < 1e-9 {
            base
        } else {
            [r[0] / l, r[1] / l, r[2] / l]
        }
    };
    if rotate_deg.abs() < 1e-9 {
        return ref0;
    }
    // ref0 を軸まわりに rotate 回転（ロドリゲスの回転公式。ref0⊥axis なので簡約形）。
    let th = rotate_deg.to_radians();
    let (s, c) = (th.sin(), th.cos());
    let cross = [
        axis[1] * ref0[2] - axis[2] * ref0[1],
        axis[2] * ref0[0] - axis[0] * ref0[2],
        axis[0] * ref0[1] - axis[1] * ref0[0],
    ];
    [
        ref0[0] * c + cross[0] * s,
        ref0[1] * c + cross[1] * s,
        ref0[2] * c + cross[2] * s,
    ]
}
