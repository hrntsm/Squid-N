use crate::app::App;
use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{
    AreaLoad, DistributionMethod, JoistLine, OneWayDir, SlabKind, SlabUsage,
};
use squid_n_edit::{AddSlab, DeleteSlab, SetSlabJoists, SetSlabKind, SetSlabOneWay, SetSlabUsage};

/// スラブ追加フォームのドラフト状態（GUI 専用）。
/// `nodes` は境界4節点（頂点0→1→2→3→0 の順で外周を辿る）の選択状態。
#[derive(Clone, Debug)]
pub struct SlabDraft {
    /// 境界節点スロット（外周順。3〜N 個、可変長）。
    pub nodes: Vec<Option<NodeId>>,
    /// 荷重種別（既定 "DL"）
    pub load_kind: String,
    /// 荷重値の入力文字列。**UI 表示は kN/m²**（内部格納は ×1e-3 した N/mm²）。
    pub load_value: String,
    pub method: DistributionMethod,
    /// スラブ用途（積載荷重プリセット。`None` は積載寄与なし）。
    pub usage: Option<SlabUsage>,
    /// 小梁入力の対象スラブ（小梁編集セクション用）。
    pub joist_target: Option<SlabId>,
    /// 小梁の支持節点（両端。小梁が架かる2節点）。
    pub joist_supports: [Option<NodeId>; 2],
    /// 小梁の負担幅 spacing の入力文字列（**UI 表示は mm**、内部も mm）。
    pub joist_spacing: String,
    /// 小梁の断面（床の中での小梁設計用。`None` は断面未割当）。
    pub joist_section: Option<squid_n_core::ids::SectionId>,
}

impl Default for SlabDraft {
    fn default() -> Self {
        Self {
            nodes: vec![None; 4],
            load_kind: "DL".to_string(),
            load_value: "0".to_string(),
            method: DistributionMethod::TriTrapezoid,
            usage: None,
            joist_target: None,
            joist_supports: [None; 2],
            joist_spacing: "0".to_string(),
            joist_section: None,
        }
    }
}

/// 用途選択で提示するプリセット（令別表第1）。`None` は「なし（積載寄与なし）」。
/// `Custom` は UI からは扱わない（モデル/シリアライズでは利用可）。
const USAGE_PRESETS: &[Option<SlabUsage>] = &[
    None,
    Some(SlabUsage::Residential),
    Some(SlabUsage::Office),
    Some(SlabUsage::Classroom),
    Some(SlabUsage::Store),
    Some(SlabUsage::AssemblyFixed),
    Some(SlabUsage::AssemblyOther),
    Some(SlabUsage::Corridor),
    Some(SlabUsage::Garage),
    Some(SlabUsage::RoofResidential),
    Some(SlabUsage::RoofStore),
];

fn usage_label(u: Option<SlabUsage>) -> &'static str {
    match u {
        None => "なし",
        Some(SlabUsage::Residential) => "住宅の居室・寝室・病室",
        Some(SlabUsage::Office) => "事務室",
        Some(SlabUsage::Classroom) => "教室",
        Some(SlabUsage::Store) => "百貨店・店舗の売場",
        Some(SlabUsage::AssemblyFixed) => "集会室・客席（固定席）",
        Some(SlabUsage::AssemblyOther) => "集会室・客席（その他）",
        Some(SlabUsage::Corridor) => "廊下・玄関・階段",
        Some(SlabUsage::Garage) => "自動車車庫・通路",
        Some(SlabUsage::RoofResidential) => "屋上・バルコニー（住宅系）",
        Some(SlabUsage::RoofStore) => "屋上・バルコニー（学校・百貨店系）",
        Some(SlabUsage::Custom { .. }) => "任意入力",
    }
}

fn method_label(m: DistributionMethod) -> &'static str {
    match m {
        DistributionMethod::TriTrapezoid => "三角/台形(45°法)",
        DistributionMethod::OneWay => "一方向",
        DistributionMethod::TributaryArea => "負担面積",
    }
}

fn kind_label(k: SlabKind) -> &'static str {
    match k {
        SlabKind::Interior => "一般",
        SlabKind::Cantilever => "片持ち",
        SlabKind::Corner => "出隅",
    }
}

fn one_way_label(o: Option<OneWayDir>) -> &'static str {
    match o {
        None => "なし",
        Some(OneWayDir::X) => "X",
        Some(OneWayDir::Y) => "Y",
    }
}

pub fn slabs_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    ui.label(
        "スラブは境界4節点・外周の梁があって初めて機能します（結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと分配結果を確認できます）。",
    );
    ui.separator();

    // ── 一覧表 ──────────────────────────────────────────
    let n = app.model.slabs.len();
    let mut pending_delete: Option<SlabId> = None;
    let mut pending_kind: Vec<(SlabId, SlabKind)> = Vec::new();
    let mut pending_one_way: Vec<(SlabId, Option<OneWayDir>)> = Vec::new();
    let mut pending_usage: Vec<(SlabId, Option<SlabUsage>)> = Vec::new();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(140.0))
        .column(Column::initial(200.0))
        .column(Column::initial(140.0))
        .column(Column::initial(90.0))
        .column(Column::initial(90.0))
        .column(Column::initial(180.0))
        .column(Column::initial(60.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &[
                "ID",
                "境界節点",
                "荷重",
                "分配法",
                "種別",
                "一方向",
                "用途",
                "小梁",
                "",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let slab = &app.model.slabs[i];
                row.col(|ui| {
                    ui.label(slab.id.0.to_string());
                });
                row.col(|ui| {
                    let s = slab
                        .boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    ui.label(s);
                });
                row.col(|ui| {
                    let s = slab
                        .loads
                        .iter()
                        .map(|l| format!("{} {:.2}kN/m²", l.kind, l.value * 1e3))
                        .collect::<Vec<_>>()
                        .join(", ");
                    ui.label(if s.is_empty() { "―".to_string() } else { s });
                });
                row.col(|ui| {
                    ui.label(method_label(slab.method));
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_kind", slab.id.0))
                        .selected_text(kind_label(slab.kind))
                        .show_ui(ui, |ui| {
                            for kind in [SlabKind::Interior, SlabKind::Cantilever, SlabKind::Corner]
                            {
                                if ui
                                    .selectable_label(slab.kind == kind, kind_label(kind))
                                    .clicked()
                                    && slab.kind != kind
                                {
                                    pending_kind.push((slab.id, kind));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_one_way", slab.id.0))
                        .selected_text(one_way_label(slab.one_way))
                        .show_ui(ui, |ui| {
                            for ow in [None, Some(OneWayDir::X), Some(OneWayDir::Y)] {
                                if ui
                                    .selectable_label(slab.one_way == ow, one_way_label(ow))
                                    .clicked()
                                    && slab.one_way != ow
                                {
                                    pending_one_way.push((slab.id, ow));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_usage", slab.id.0))
                        .selected_text(usage_label(slab.usage))
                        .show_ui(ui, |ui| {
                            for &u in USAGE_PRESETS {
                                if ui
                                    .selectable_label(slab.usage == u, usage_label(u))
                                    .clicked()
                                    && slab.usage != u
                                {
                                    pending_usage.push((slab.id, u));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    let cnt = slab.joists.len();
                    ui.label(if cnt == 0 {
                        "―".to_string()
                    } else {
                        format!("{cnt}本")
                    });
                });
                row.col(|ui| {
                    if ui.button("🗑").on_hover_text("このスラブを削除").clicked() {
                        pending_delete = Some(slab.id);
                    }
                });
            });
        });

    let had_pending = !pending_kind.is_empty()
        || !pending_one_way.is_empty()
        || !pending_usage.is_empty()
        || pending_delete.is_some();
    for (id, kind) in pending_kind {
        app.undo
            .run(&mut app.model, Box::new(SetSlabKind { id, kind }));
    }
    for (id, one_way) in pending_one_way {
        app.undo
            .run(&mut app.model, Box::new(SetSlabOneWay { id, one_way }));
    }
    for (id, usage) in pending_usage {
        app.undo
            .run(&mut app.model, Box::new(SetSlabUsage { id, usage }));
    }
    if let Some(id) = pending_delete {
        app.undo.run(&mut app.model, Box::new(DeleteSlab { id }));
    }
    if had_pending {
        app.staleness.mark_edited();
    }

    ui.separator();
    // ── スラブ追加フォーム ──────────────────────────────────
    ui.strong("スラブを追加");

    if app.model.nodes.len() < 3 {
        ui.label("スラブを追加するには節点が3つ以上必要です");
        return;
    }

    // 借用衝突を避けるため、節点一覧は先にローカルへ複製しておく
    // （app.model への参照を保持したまま app.slab_draft を可変参照しないため）。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();

    // 境界頂点は 3〜N の可変長。スロット数は +/− ボタンで調整する。
    if app.slab_draft.nodes.len() < 3 {
        app.slab_draft.nodes.resize(3, None);
    }
    ui.label(
        "境界節点（頂点0→1→2→…→0 の順で外周を辿り、その辺 i=節点i→節点i+1 を持つ梁を検索します。3〜N 節点対応）:",
    );
    ui.horizontal(|ui| {
        if ui.button("+ 頂点を追加").clicked() {
            app.slab_draft.nodes.push(None);
        }
        if ui
            .add_enabled(
                app.slab_draft.nodes.len() > 3,
                egui::Button::new("− 頂点を削除"),
            )
            .on_hover_text("末尾の頂点スロットを削除（最小3）")
            .clicked()
        {
            app.slab_draft.nodes.pop();
        }
        ui.label(format!("頂点数: {}", app.slab_draft.nodes.len()));
    });
    ui.horizontal_wrapped(|ui| {
        let n_slots = app.slab_draft.nodes.len();
        for k in 0..n_slots {
            let text = app.slab_draft.nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("slab_draft_node_{}", k))
                .selected_text(format!("頂点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        let label = format!("N{}", nid.0);
                        if ui
                            .selectable_label(app.slab_draft.nodes[k] == Some(nid), &label)
                            .clicked()
                        {
                            app.slab_draft.nodes[k] = Some(nid);
                        }
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("荷重種別:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_kind).desired_width(60.0));
        ui.label("荷重 [kN/m²]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_value).desired_width(80.0));
    });

    ui.horizontal(|ui| {
        ui.label("用途（積載荷重）:")
            .on_hover_text("令別表第1 の積載荷重（骨組用）を「床積載(自動)」ケースへ分配します");
        egui::ComboBox::from_id_salt("slab_draft_usage")
            .selected_text(usage_label(app.slab_draft.usage))
            .show_ui(ui, |ui| {
                for &u in USAGE_PRESETS {
                    ui.selectable_value(&mut app.slab_draft.usage, u, usage_label(u));
                }
            });
        if let Some(u) = app.slab_draft.usage {
            use squid_n_core::model::LoadPurpose;
            // 表示は kN/m²（内部 N/mm² を ×1e3）。
            ui.label(format!(
                "床用 {:.2} / 骨組用 {:.2} / 地震用 {:.2} kN/m²",
                u.live_load(LoadPurpose::Floor) * 1e3,
                u.live_load(LoadPurpose::Frame) * 1e3,
                u.live_load(LoadPurpose::Seismic) * 1e3,
            ));
        }
    });

    ui.horizontal(|ui| {
        ui.label("分配法:");
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TriTrapezoid,
            "三角/台形(45°法)",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::OneWay,
            "一方向",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TributaryArea,
            "負担面積",
        );
    });

    let selected: Vec<NodeId> = app.slab_draft.nodes.iter().filter_map(|n| *n).collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    // 全スロットが埋まり（selected.len == slots）、3頂点以上、重複が無いこと。
    let n_slots = app.slab_draft.nodes.len();
    let can_add = selected.len() == n_slots && n_slots >= 3 && dedup.len() == n_slots;

    if ui
        .add_enabled(can_add, egui::Button::new("+ 追加"))
        .on_hover_text("境界節点が3つ以上すべて選択され、かつ重複が無い場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .slab_draft
            .nodes
            .iter()
            .map(|n| n.expect("can_add で全スロット Some を確認済み"))
            .collect();
        let value_kn_m2 = app
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        // kN/m² → N/mm²（内部単位系）。1 kN/m² = 1e-3 N/mm²。
        let value = value_kn_m2 * 1e-3;
        let kind = app.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        app.undo.run(
            &mut app.model,
            Box::new(AddSlab {
                boundary,
                joists: Vec::new(),
                loads: vec![AreaLoad { kind, value }],
                method: app.slab_draft.method,
                usage: app.slab_draft.usage,
            }),
        );
        app.staleness.mark_edited();
    }

    joists_section(ui, app);
}

/// 小梁（`JoistLine`）の入力セクション。対象スラブを選び、支持2節点＋負担幅
/// `spacing` で小梁を追加/削除する。小梁は矩形スラブの二段階伝達
/// （`distribute_rect_with_joists`）でのみ使われ、分配法が「三角/台形」または
/// 「一方向」のとき有効になる（それ以外の分配法では無視される）。
///
/// 小梁の架かる方向 `dir` は支持2節点の平面（XY）ベクトルから自動算定する。
fn joists_section(ui: &mut egui::Ui, app: &mut App) {
    ui.separator();
    ui.strong("小梁を入力（矩形スラブの二段階伝達）");
    ui.label(
        "対象スラブを選び、小梁が架かる支持2節点と負担幅を指定します。分配法が「三角/台形」または「一方向」のときに有効です。",
    );

    if app.model.slabs.is_empty() {
        ui.label("スラブがありません");
        return;
    }

    // 対象スラブ選択。
    let slab_ids: Vec<SlabId> = app.model.slabs.iter().map(|s| s.id).collect();
    if app
        .slab_draft
        .joist_target
        .is_none_or(|t| !slab_ids.contains(&t))
    {
        app.slab_draft.joist_target = slab_ids.first().copied();
    }
    egui::ComboBox::from_id_salt("joist_target_slab")
        .selected_text(
            app.slab_draft
                .joist_target
                .map(|t| format!("スラブ #{}", t.0))
                .unwrap_or_else(|| "―".to_string()),
        )
        .show_ui(ui, |ui| {
            for &sid in &slab_ids {
                ui.selectable_value(
                    &mut app.slab_draft.joist_target,
                    Some(sid),
                    format!("スラブ #{}", sid.0),
                );
            }
        });

    let Some(target) = app.slab_draft.joist_target else {
        return;
    };
    let Some(slab_idx) = app.model.slabs.iter().position(|s| s.id == target) else {
        return;
    };

    // 変更は借用衝突を避けるため、UI 走査後に SetSlabJoists で一括反映する。
    let mut new_joists: Option<Vec<JoistLine>> = None;

    // 既存小梁の一覧（削除ボタン付き）。
    let joists = app.model.slabs[slab_idx].joists.clone();
    if joists.is_empty() {
        ui.label("この床には小梁がありません");
    } else {
        for (k, j) in joists.iter().enumerate() {
            ui.horizontal(|ui| {
                let sec = j
                    .section
                    .map(|s| format!("S{}", s.0))
                    .unwrap_or_else(|| "断面なし".to_string());
                ui.label(format!(
                    "小梁{}: 支持 N{}–N{}, 負担幅 {:.0} mm, 断面 {}",
                    k, j.support[0].0, j.support[1].0, j.spacing, sec
                ));
                if ui.button("🗑").on_hover_text("この小梁を削除").clicked() {
                    let mut v = joists.clone();
                    v.remove(k);
                    new_joists = Some(v);
                }
            });
        }
    }

    // 小梁の実部材化（実 Beam 要素を生成し、応力解析・断面検定の対象にする）。
    if !joists.is_empty() {
        // 実 Beam が未生成の小梁本数を数える。
        let beam_exists = |a: NodeId, b: NodeId| -> bool {
            app.model.elements.iter().any(|e| {
                e.kind == squid_n_core::model::ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };
        let unmaterialized = joists
            .iter()
            .filter(|j| j.support[0] != j.support[1] && !beam_exists(j.support[0], j.support[1]))
            .count();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    unmaterialized > 0,
                    egui::Button::new("小梁を実部材化"),
                )
                .on_hover_text(
                    "各小梁の支持2節点に実 Beam 要素を生成します。実部材化した小梁には床荷重が等分布荷重として載り、応力解析・断面検定の対象になります。",
                )
                .clicked()
            {
                app.undo.run(
                    &mut app.model,
                    Box::new(squid_n_edit::MaterializeSlabJoists { slab: target }),
                );
                app.staleness.mark_edited();
            }
            ui.label(if unmaterialized > 0 {
                format!("未実部材化: {unmaterialized}本")
            } else {
                "すべて実部材化済み".to_string()
            });
        });
    }

    // 小梁の追加フォーム。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
    ui.horizontal(|ui| {
        for e in 0..2 {
            let text = app.slab_draft.joist_supports[e]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("joist_support_{e}"))
                .selected_text(format!("支持{e}: {text}"))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        if ui
                            .selectable_label(
                                app.slab_draft.joist_supports[e] == Some(nid),
                                format!("N{}", nid.0),
                            )
                            .clicked()
                        {
                            app.slab_draft.joist_supports[e] = Some(nid);
                        }
                    }
                });
        }
        ui.label("負担幅 [mm]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.joist_spacing).desired_width(80.0));
        ui.label("断面:")
            .on_hover_text("床の中での小梁設計（単純支持梁の曲げ・たわみ検定）に用いる断面");
        let sec_text = app
            .slab_draft
            .joist_section
            .map(|s| format!("S{}", s.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("joist_section")
            .selected_text(sec_text)
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut app.slab_draft.joist_section, None, "―");
                for sec in &app.model.sections {
                    ui.selectable_value(
                        &mut app.slab_draft.joist_section,
                        Some(sec.id),
                        format!("S{} {}", sec.id.0, sec.name),
                    );
                }
            });
    });

    let s0 = app.slab_draft.joist_supports[0];
    let s1 = app.slab_draft.joist_supports[1];
    let spacing = app
        .slab_draft
        .joist_spacing
        .trim()
        .parse::<f64>()
        .unwrap_or(0.0);
    // 追加可能な小梁を安全に構成する。両支持節点が現存し（節点削除でドラフトが
    // 陳腐化しても out-of-bounds しないよう `nodes.get` で確認）、平面（XY）方向に
    // 有意な離間がある（`dir≈[0,0]` は分配エンジンが Y 軸へ暗黙フォールバックし
    // 誤分配となるため弾く）場合のみ Some を返す。
    let addable_joist: Option<JoistLine> = (|| {
        let (a, b) = (s0?, s1?);
        if a == b || spacing <= 0.0 {
            return None;
        }
        let ca = app.model.nodes.get(a.index())?.coord;
        let cb = app.model.nodes.get(b.index())?.coord;
        let dir = [cb[0] - ca[0], cb[1] - ca[1]];
        if dir[0].hypot(dir[1]) <= 1e-9 {
            return None; // 平面上で重なる2節点（鉛直に積層等）は小梁として無効。
        }
        Some(JoistLine {
            dir,
            spacing,
            support: [a, b],
            section: app.slab_draft.joist_section,
        })
    })();

    if ui
        .add_enabled(addable_joist.is_some(), egui::Button::new("+ 小梁を追加"))
        .on_hover_text(
            "現存する異なる支持2節点（平面上で離れている）と正の負担幅を指定してください",
        )
        .clicked()
    {
        if let Some(joist) = addable_joist {
            let mut v = joists.clone();
            v.push(joist);
            new_joists = Some(v);
        }
    }

    if let Some(v) = new_joists {
        app.undo.run(
            &mut app.model,
            Box::new(SetSlabJoists {
                id: target,
                joists: v,
            }),
        );
        app.staleness.mark_edited();
    }
}
