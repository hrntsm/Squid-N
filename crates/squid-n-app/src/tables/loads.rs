use crate::app::App;
use squid_n_core::ids::{ElemId, LoadCaseId, NodeId};
use squid_n_core::model::{ElementKind, LoadCaseKind, MemberLoad, MemberLoadKind};
use squid_n_edit::{
    AddCombination, AddLoadCase, AddMemberLoad, DeleteCombination, DeleteLoadCase,
    DeleteMemberLoad, DeleteNodalLoad, SetLoadCaseKind, SetLoadCaseName, SetNodalLoad,
};

/// `LoadCaseKind` の全種別（UI の選択肢一覧・順序を1箇所に集約する）。
const LOAD_CASE_KINDS: [LoadCaseKind; 7] = [
    LoadCaseKind::Dead,
    LoadCaseKind::Live,
    LoadCaseKind::LiveSeismic,
    LoadCaseKind::Snow,
    LoadCaseKind::Wind,
    LoadCaseKind::Seismic,
    LoadCaseKind::Other,
];

/// `LoadCaseKind` の表示ラベル（レビュー §1.7: 地震用重量の集計に使う種別を
/// 荷重タブから明示的に選べるようにする）。
fn load_case_kind_label(kind: LoadCaseKind) -> &'static str {
    match kind {
        LoadCaseKind::Dead => "固定荷重",
        LoadCaseKind::Live => "積載荷重(長期)",
        LoadCaseKind::LiveSeismic => "積載荷重(地震用)",
        LoadCaseKind::Snow => "積雪荷重",
        LoadCaseKind::Wind => "風荷重",
        LoadCaseKind::Seismic => "地震荷重",
        LoadCaseKind::Other => "その他/未設定",
    }
}

#[derive(Clone)]
struct MemberLoadDraft {
    elem_idx: usize, // app.model.elements のインデックス
    kind: u8,        // 0=中間集中, 1=等分布, 2=台形
    dir: u8,         // 0=-Z(鉛直下),1=+Z,2=+X,3=-X,4=+Y,5=-Y
    a: String,
    b: String,
    w1: String,
    w2: String,
    p: String,
}

impl Default for MemberLoadDraft {
    fn default() -> Self {
        Self {
            elem_idx: 0,
            kind: 1,
            dir: 0,
            a: "0".into(),
            b: "0".into(),
            w1: "0".into(),
            w2: "0".into(),
            p: "0".into(),
        }
    }
}

pub fn loads_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // --- スラブ荷重（床荷重）への案内 ---
    ui.label(format!(
        "スラブ: {} 枚（モデルタブの「スラブ」で床荷重を追加できます。分配結果は結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと確認できます）",
        app.model.slabs.len()
    ));
    ui.add_space(4.0);

    // --- 荷重ケース一覧（名称編集・追加・削除・編集対象の選択） ---
    ui.horizontal(|ui| {
        ui.strong("荷重ケース");
        if ui
            .button("+ ケース追加")
            .on_hover_text("新しい荷重ケースを追加します")
            .clicked()
        {
            let name = format!("LC{}", app.model.load_cases.len());
            app.undo.run(&mut app.model, Box::new(AddLoadCase { name }));
            // 追加したケースを編集対象として選択
            app.nav.focus_load_case = app.model.load_cases.last().map(|lc| lc.id);
            app.staleness.mark_edited();
        }
    });
    let n_lc = app.model.load_cases.len();
    let mut pending_name: Vec<(usize, String)> = Vec::new();
    let mut pending_kind: Vec<(LoadCaseId, LoadCaseKind)> = Vec::new();
    let mut pending_delete: Option<LoadCaseId> = None;
    let mut name_bufs: Vec<String> = app
        .model
        .load_cases
        .iter()
        .map(|lc| lc.name.clone())
        .collect();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(120.0))
        .column(Column::initial(130.0))
        .column(Column::initial(60.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "種別", "荷重数", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n_lc, |mut row| {
                let i = row.index();
                let lc = &app.model.load_cases[i];
                let is_sel = app.nav.focus_load_case == Some(lc.id);
                row.col(|ui| {
                    let rich = egui::RichText::new(lc.id.0.to_string()).color(if is_sel {
                        crate::theme::WHITE
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui
                        .selectable_label(is_sel, rich)
                        .on_hover_text("クリックで下の荷重編集の対象にする")
                        .clicked()
                    {
                        app.nav.focus_load_case = Some(lc.id);
                    }
                });
                row.col(|ui| {
                    if i < name_bufs.len() {
                        let resp = ui.add(
                            egui::TextEdit::singleline(&mut name_bufs[i])
                                .desired_width(110.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            let trimmed = name_bufs[i].trim().to_string();
                            if trimmed != lc.name && !trimmed.is_empty() {
                                pending_name.push((i, trimmed));
                            }
                        }
                    }
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("load_case_kind", lc.id.0))
                        .selected_text(load_case_kind_label(lc.kind))
                        .show_ui(ui, |ui| {
                            for kind in LOAD_CASE_KINDS {
                                if ui
                                    .selectable_label(lc.kind == kind, load_case_kind_label(kind))
                                    .clicked()
                                    && lc.kind != kind
                                {
                                    pending_kind.push((lc.id, kind));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    ui.label((lc.nodal.len() + lc.member.len()).to_string());
                });
                row.col(|ui| {
                    let referenced = app
                        .model
                        .combinations
                        .iter()
                        .any(|c| c.terms.iter().any(|(id, _)| *id == lc.id));
                    let btn = ui.add_enabled(!referenced, egui::Button::new("🗑"));
                    if referenced {
                        btn.on_hover_text("荷重組合せから参照中のため削除できません");
                    } else if btn
                        .on_hover_text("ケースと中身の荷重をまとめて削除")
                        .clicked()
                    {
                        pending_delete = Some(lc.id);
                    }
                });
            });
        });

    let had_name = !pending_name.is_empty() || !pending_kind.is_empty() || pending_delete.is_some();
    for (i, name) in pending_name {
        let lc_id = LoadCaseId(app.model.load_cases[i].id.0);
        app.undo.run(
            &mut app.model,
            Box::new(SetLoadCaseName { id: lc_id, name }),
        );
    }
    for (id, kind) in pending_kind {
        app.undo
            .run(&mut app.model, Box::new(SetLoadCaseKind { id, kind }));
    }
    if let Some(lc_id) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteLoadCase { id: lc_id }));
        if app.nav.focus_load_case == Some(lc_id) {
            app.nav.focus_load_case = None;
        }
        if app.last_static
            == Some(crate::app::StaticKey::Case(
                crate::app::StaticCaseKey::User(lc_id),
            ))
        {
            app.last_static = None;
        }
    }
    if had_name {
        app.staleness.mark_edited();
    }

    ui.add_space(8.0);
    combinations_section(ui, app);

    ui.add_space(8.0);
    egui::CollapsingHeader::new("荷重計算条件")
        .id_salt("load_cfg_section")
        .default_open(false)
        .show(ui, |ui| {
            crate::tables::load_cfg::load_cfg_panel(ui, app);
        });

    ui.add_space(8.0);

    // --- 節点荷重詳細（選択中の荷重ケース） ---
    ui.strong("節点荷重");
    if app.model.load_cases.is_empty() {
        ui.label("荷重ケースがありません。「+ ケース追加」で作成してください。");
        return;
    }
    // 編集対象: ナビゲータ/上表で選択したケース → 最後に実行したケース → 先頭
    let lc_idx = app
        .nav
        .focus_load_case
        .and_then(|id| app.model.load_cases.iter().position(|lc| lc.id == id))
        .or_else(|| {
            app.last_static.and_then(|key| match key {
                // 地震静的(Seismic)・風静的(Wind)はユーザー荷重ケースに対応しないため
                // None（呼び出し元のフォールバックで先頭ケースが選ばれる）。
                crate::app::StaticKey::Case(crate::app::StaticCaseKey::User(id)) => {
                    app.model.load_cases.iter().position(|lc| lc.id == id)
                }
                crate::app::StaticKey::Case(crate::app::StaticCaseKey::Seismic(_))
                | crate::app::StaticKey::Case(crate::app::StaticCaseKey::Wind(_))
                | crate::app::StaticKey::Combo(_) => None,
            })
        })
        .unwrap_or(0);
    let lc_id = app.model.load_cases[lc_idx].id;
    ui.label(format!(
        "ケース: {} ({})",
        lc_id.0, app.model.load_cases[lc_idx].name
    ));

    let nodal_count = app.model.load_cases[lc_idx].nodal.len();
    let mut pending_load: Vec<(NodeId, [f64; 6])> = Vec::new();
    let mut pending_nodal_delete: Option<NodeId> = None;
    let mut value_bufs: Vec<[String; 6]> = app.model.load_cases[lc_idx]
        .nodal
        .iter()
        .map(|n| n.values.map(|v| format!("{:.2}", v)))
        .collect();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .columns(Column::initial(70.0), 6)
        .column(Column::auto())
        .header(20.0, |mut h| {
            h.col(|ui| {
                ui.strong("節点");
            });
            for t in &["Fx", "Fy", "Fz", "Mx", "My", "Mz"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
            h.col(|_| {});
        })
        .body(|body| {
            body.rows(22.0, nodal_count, |mut row| {
                let i = row.index();
                let nodal = &app.model.load_cases[lc_idx].nodal[i];
                row.col(|ui| {
                    ui.label(nodal.node.0.to_string());
                });
                for k in 0..6 {
                    row.col(|ui| {
                        let buf = &mut value_bufs[i][k];
                        let resp = ui.add(
                            egui::TextEdit::singleline(buf)
                                .desired_width(60.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            if let Ok(val) = buf.trim().parse::<f64>() {
                                if (val - nodal.values[k]).abs() > 1e-9 {
                                    let mut new_vals = nodal.values;
                                    new_vals[k] = val;
                                    pending_load.push((nodal.node, new_vals));
                                }
                            }
                        }
                        if buf.trim().parse::<f64>().is_err() {
                            ui.painter().rect_filled(
                                resp.rect,
                                0.0,
                                crate::theme::translucent(crate::theme::ERROR_RED, 60),
                            );
                        }
                    });
                }
                row.col(|ui| {
                    if ui.button("🗑").on_hover_text("この節点荷重を削除").clicked() {
                        pending_nodal_delete = Some(nodal.node);
                    }
                });
            });
        });

    let had_load = !pending_load.is_empty() || pending_nodal_delete.is_some();
    for (node, values) in pending_load {
        app.undo.run(
            &mut app.model,
            Box::new(SetNodalLoad {
                lc: lc_id,
                node,
                values,
            }),
        );
    }
    if let Some(node) = pending_nodal_delete {
        app.undo.run(
            &mut app.model,
            Box::new(DeleteNodalLoad { lc: lc_id, node }),
        );
    }
    if had_load {
        app.staleness.mark_edited();
    }

    // --- 部材荷重セクション ---
    ui.add_space(8.0);
    ui.strong("部材荷重");

    // (A) 既存の部材荷重リスト（削除可能）
    let mut pending_delete: Option<usize> = None;
    {
        let member_loads = &app.model.load_cases[lc_idx].member;
        if member_loads.is_empty() {
            ui.label("部材荷重なし");
        } else {
            for (i, ml) in member_loads.iter().enumerate() {
                ui.horizontal(|ui| {
                    let kind_str = match &ml.kind {
                        MemberLoadKind::Point { a, p } => {
                            format!("中間集中 a={:.0} P={:.1}", a, p)
                        }
                        MemberLoadKind::Distributed { a, b, w1, w2 } => {
                            format!("分布 [{:.0},{:.0}] w1={:.2} w2={:.2}", a, b, w1, w2)
                        }
                    };
                    let dir_str =
                        format!("dir=({:.1},{:.1},{:.1})", ml.dir[0], ml.dir[1], ml.dir[2]);
                    ui.label(format!("梁#{} / {} / {}", ml.elem.0, kind_str, dir_str));
                    if ui.button("削除").clicked() {
                        pending_delete = Some(i);
                    }
                });
            }
        }
    }
    if let Some(idx) = pending_delete {
        app.undo.run(
            &mut app.model,
            Box::new(DeleteMemberLoad {
                lc: lc_id,
                index: idx,
            }),
        );
        app.staleness.mark_edited();
    }

    // (B) 追加フォーム
    // 梁要素のインデックス一覧を収集
    let beam_indices: Vec<usize> = app
        .model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| e.kind == ElementKind::Beam)
        .map(|(i, _)| i)
        .collect();

    if beam_indices.is_empty() {
        ui.label("梁がありません");
        return;
    }

    let draft_id = egui::Id::new("member_load_draft");
    let mut draft: MemberLoadDraft = ui
        .data_mut(|d| d.get_temp::<MemberLoadDraft>(draft_id))
        .unwrap_or_default();

    // elem_idx が梁一覧の範囲外なら先頭梁に補正
    if !beam_indices.contains(&draft.elem_idx) {
        draft.elem_idx = beam_indices[0];
    }

    let mut pending_add: Option<MemberLoad> = None;

    ui.add_space(4.0);

    // 対象梁 ComboBox
    ui.horizontal(|ui| {
        ui.label("対象梁:");
        let current_beam_label = app
            .model
            .elements
            .get(draft.elem_idx)
            .map(|e| format!("梁#{}", e.id.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("member_load_beam")
            .selected_text(current_beam_label)
            .show_ui(ui, |ui| {
                for &bi in &beam_indices {
                    if let Some(elem) = app.model.elements.get(bi) {
                        let label = format!("梁#{}", elem.id.0);
                        if ui.selectable_label(draft.elem_idx == bi, &label).clicked() {
                            draft.elem_idx = bi;
                        }
                    }
                }
            });
    });

    // 種別選択
    ui.horizontal(|ui| {
        ui.label("種別:");
        ui.selectable_value(&mut draft.kind, 0u8, "中間集中");
        ui.selectable_value(&mut draft.kind, 1u8, "等分布");
        ui.selectable_value(&mut draft.kind, 2u8, "台形");
    });

    // 方向 ComboBox
    ui.horizontal(|ui| {
        ui.label("方向:");
        let dir_labels = ["鉛直下(-Z)", "鉛直上(+Z)", "X+", "X-", "Y+", "Y-"];
        let current_dir_label = dir_labels.get(draft.dir as usize).copied().unwrap_or("―");
        egui::ComboBox::from_id_salt("member_load_dir")
            .selected_text(current_dir_label)
            .show_ui(ui, |ui| {
                for (idx, label) in dir_labels.iter().enumerate() {
                    if ui
                        .selectable_label(draft.dir == idx as u8, *label)
                        .clicked()
                    {
                        draft.dir = idx as u8;
                    }
                }
            });
    });

    // パラメータ（kind で出し分け）
    match draft.kind {
        0 => {
            // 中間集中
            ui.horizontal(|ui| {
                ui.label("a[mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.a).desired_width(80.0));
                ui.label("P[N]:");
                ui.add(egui::TextEdit::singleline(&mut draft.p).desired_width(80.0));
            });
        }
        1 => {
            // 等分布
            ui.horizontal(|ui| {
                ui.label("w1[N/mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.w1).desired_width(80.0));
            });
        }
        _ => {
            // 台形
            ui.horizontal(|ui| {
                ui.label("a[mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.a).desired_width(80.0));
                ui.label("b[mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.b).desired_width(80.0));
            });
            ui.horizontal(|ui| {
                ui.label("w1[N/mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.w1).desired_width(80.0));
                ui.label("w2[N/mm]:");
                ui.add(egui::TextEdit::singleline(&mut draft.w2).desired_width(80.0));
            });
        }
    }

    // 追加ボタン
    if ui.button("+ 部材荷重追加").clicked() {
        if let Some(elem) = app.model.elements.get(draft.elem_idx) {
            let elem_id: ElemId = elem.id;

            // 梁長を計算（等分布の b 用）
            let len = if elem.nodes.len() >= 2 {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                if ni < app.model.nodes.len() && nj < app.model.nodes.len() {
                    let pi = app.model.nodes[ni].coord;
                    let pj = app.model.nodes[nj].coord;
                    ((pj[0] - pi[0]).powi(2) + (pj[1] - pi[1]).powi(2) + (pj[2] - pi[2]).powi(2))
                        .sqrt()
                } else {
                    0.0
                }
            } else {
                0.0
            };

            // 方向ベクトル
            let dir: [f64; 3] = match draft.dir {
                0 => [0.0, 0.0, -1.0],
                1 => [0.0, 0.0, 1.0],
                2 => [1.0, 0.0, 0.0],
                3 => [-1.0, 0.0, 0.0],
                4 => [0.0, 1.0, 0.0],
                _ => [0.0, -1.0, 0.0],
            };

            let parse = |s: &str| s.trim().parse::<f64>().unwrap_or(0.0);

            let kind = match draft.kind {
                0 => MemberLoadKind::Point {
                    a: parse(&draft.a),
                    p: parse(&draft.p),
                },
                1 => MemberLoadKind::Distributed {
                    a: 0.0,
                    b: len,
                    w1: parse(&draft.w1),
                    w2: parse(&draft.w1),
                },
                _ => MemberLoadKind::Distributed {
                    a: parse(&draft.a),
                    b: parse(&draft.b),
                    w1: parse(&draft.w1),
                    w2: parse(&draft.w2),
                },
            };

            pending_add = Some(MemberLoad {
                elem: elem_id,
                dir,
                kind,
            });
        }
    }

    // draft を書き戻す（クロージャ外）
    ui.data_mut(|d| d.insert_temp(draft_id, draft));

    // (C) 追加コマンド発行（クロージャ外、借用衝突なし）
    if let Some(load) = pending_add {
        app.undo
            .run(&mut app.model, Box::new(AddMemberLoad { lc: lc_id, load }));
        app.staleness.mark_edited();
    }
}

/// 荷重ケース名を引く（見つからなければ空文字）。
fn load_case_name(model: &squid_n_core::model::Model, id: LoadCaseId) -> String {
    model
        .load_cases
        .iter()
        .find(|lc| lc.id == id)
        .map(|lc| lc.name.clone())
        .unwrap_or_default()
}

/// 荷重ケース選択 ComboBox。`allow_none` の場合のみ「（なし）」を選択できる。
fn combo_case_selector(
    ui: &mut egui::Ui,
    id_salt: &str,
    label: &str,
    model: &squid_n_core::model::Model,
    selected: &mut Option<LoadCaseId>,
    allow_none: bool,
) {
    ui.horizontal(|ui| {
        ui.label(label);
        let text = selected
            .and_then(|id| model.load_cases.iter().find(|lc| lc.id == id))
            .map(|lc| format!("[{}] {}", lc.id.0, lc.name))
            .unwrap_or_else(|| "（なし）".to_string());
        egui::ComboBox::from_id_salt(id_salt)
            .selected_text(text)
            .show_ui(ui, |ui| {
                if allow_none
                    && ui
                        .selectable_label(selected.is_none(), "（なし）")
                        .clicked()
                {
                    *selected = None;
                }
                for lc in &model.load_cases {
                    if ui
                        .selectable_label(
                            *selected == Some(lc.id),
                            format!("[{}] {}", lc.id.0, lc.name),
                        )
                        .clicked()
                    {
                        *selected = Some(lc.id);
                    }
                }
            });
    });
}

/// 荷重組合せセクション：既存組合せの一覧・削除と、標準組合せの自動生成 UI。
fn combinations_section(ui: &mut egui::Ui, app: &mut App) {
    ui.strong("荷重組合せ");

    if app.model.load_cases.is_empty() {
        ui.label("荷重ケースがありません。組合せを作成するにはまず荷重ケースを追加してください。");
        return;
    }

    // --- 既存組合せの一覧（内訳表示・削除） ---
    let mut pending_delete: Option<usize> = None;
    if app.model.combinations.is_empty() {
        ui.label("組合せがありません。下の「自動生成」で作成できます。");
    } else {
        for (i, combo) in app.model.combinations.iter().enumerate() {
            ui.horizontal(|ui| {
                let terms_str = combo
                    .terms
                    .iter()
                    .map(|(id, factor)| {
                        format!(
                            "{:.2}×[{}]{}",
                            factor,
                            id.0,
                            load_case_name(&app.model, *id)
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(" + ");
                ui.label(format!("{}: {}", combo.name, terms_str));
                if ui
                    .button("🗑")
                    .on_hover_text("この荷重組合せを削除")
                    .clicked()
                {
                    pending_delete = Some(i);
                }
            });
        }
    }
    if let Some(idx) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteCombination { index: idx }));
        app.staleness.mark_edited();
    }

    // --- 自動生成 ---
    ui.add_space(4.0);
    ui.strong("自動生成");
    combo_case_selector(
        ui,
        "combo_draft_dl",
        "DL用:",
        &app.model,
        &mut app.combo_draft.dl,
        false,
    );
    combo_case_selector(
        ui,
        "combo_draft_ll",
        "LL用:",
        &app.model,
        &mut app.combo_draft.ll,
        false,
    );
    combo_case_selector(
        ui,
        "combo_draft_seismic_x",
        "地震X用:",
        &app.model,
        &mut app.combo_draft.seismic_x,
        true,
    );
    combo_case_selector(
        ui,
        "combo_draft_seismic_y",
        "地震Y用:",
        &app.model,
        &mut app.combo_draft.seismic_y,
        true,
    );
    combo_case_selector(
        ui,
        "combo_draft_snow",
        "積雪用:",
        &app.model,
        &mut app.combo_draft.snow,
        true,
    );

    ui.checkbox(&mut app.analysis_cfg.heavy_snow_zone, "多雪区域")
        .on_hover_text("有効にすると長期 G+P+0.7S、短期地震/暴風 G+P+0.35S±K/W の組合せも生成します（施行令86条・82条）");

    let can_generate = app.combo_draft.dl.is_some() && app.combo_draft.ll.is_some();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(can_generate, egui::Button::new("⚙ 標準組合せを生成"))
            .on_hover_text(
                "DL/LL（必須）と地震X/Y・積雪（任意）から長期・短期の標準組合せを生成します",
            )
            .clicked()
        {
            if let (Some(dl), Some(ll)) = (app.combo_draft.dl, app.combo_draft.ll) {
                let input = squid_n_load::combo::ComboInput {
                    dl,
                    ll,
                    seismic_x: app.combo_draft.seismic_x,
                    seismic_y: app.combo_draft.seismic_y,
                    wind_x: None,
                    wind_y: None,
                    snow: app.combo_draft.snow,
                    heavy_snow_zone: app.analysis_cfg.heavy_snow_zone,
                };
                let combos = squid_n_load::combo::standard_combinations(&input);
                for combo in combos {
                    app.undo
                        .run(&mut app.model, Box::new(AddCombination { combo }));
                }
                app.staleness.mark_edited();
            }
        }
        if ui
            .button("⚙ 種別から自動生成")
            .on_hover_text(
                "荷重ケースの種別から固定(必須)・積載(必須)・積雪・風を各先頭1件選んで標準組合せを生成します。\
                 種別が特定できない場合はエラーになります",
            )
            .clicked()
        {
            app.auto_generate_combinations_action();
        }
    });
    if let Some(err) = &app.last_error {
        ui.colored_label(crate::theme::ERROR_RED, err);
    }
}
