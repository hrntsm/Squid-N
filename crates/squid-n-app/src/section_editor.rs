//! UI-3: 断面作成UI（パラメトリック SectionShape）。
//!
//! モデルタブの ModelsTab::Sections で表示される下部パネル。
//! 鋼 H / 箱 / L / C / T / 丸、RC 矩形 / 丸 の寸法を入力すると
//! `SectionShape::to_section` を呼んで `Section` を `model.sections`
//! に新規追加する。インスペクタ内寸法プレビューは後続 UI-4 で統合。

use crate::app::App;
use squid_n_core::ids::SectionId;
use squid_n_edit::{AddSection, AddSectionShape, EditSectionShape};
use squid_n_section::catalog::CatalogShape;
use squid_n_section::shape::{BarSet, RcRebar, SectionShape, ShearBar};

/// 断面作成UIのドラフト状態。App に保持して UI を跨いで維持。
#[derive(Debug, Clone)]
pub struct SectionEditorDraft {
    pub kind: ShapeKind,
    pub name: String,
    // 鋼共通パラメータ
    pub h: f64,
    pub b: f64,
    pub tw: f64,
    pub tf: f64,
    pub t: f64,
    pub r: f64,
    // L 形
    pub leg_a: f64,
    pub leg_b: f64,
    pub leg_thick: f64,
    // 丸鋼管
    pub outer_dia: f64,
    pub thick: f64,
    // RC 共通
    pub rc_b: f64,
    pub rc_d: f64,
    // RC 配筋
    pub main_x_count: u32,
    pub main_x_dia: f64,
    pub main_x_layers: u32,
    pub main_y_count: u32,
    pub main_y_dia: f64,
    pub main_y_layers: u32,
    pub cover: f64,
    pub shear_dia: f64,
    pub shear_pitch: f64,
    pub shear_legs: u32,
}

impl Default for SectionEditorDraft {
    fn default() -> Self {
        Self {
            kind: ShapeKind::SteelH,
            name: "断面1".to_string(),
            h: 400.0,
            b: 200.0,
            tw: 8.0,
            tf: 12.0,
            t: 12.0,
            r: 12.0,
            leg_a: 75.0,
            leg_b: 75.0,
            leg_thick: 9.0,
            outer_dia: 216.0,
            thick: 8.0,
            rc_b: 400.0,
            rc_d: 600.0,
            main_x_count: 6,
            main_x_dia: 19.0,
            main_x_layers: 2,
            main_y_count: 2,
            main_y_dia: 19.0,
            main_y_layers: 1,
            cover: 40.0,
            shear_dia: 13.0,
            shear_pitch: 100.0,
            shear_legs: 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeKind {
    SteelH,
    SteelBox,
    SteelAngle,
    SteelChannel,
    SteelTee,
    SteelPipe,
    RcRect,
    RcCircle,
}

impl ShapeKind {
    pub fn label(self) -> &'static str {
        match self {
            ShapeKind::SteelH => "鋼 H形",
            ShapeKind::SteelBox => "鋼 箱形",
            ShapeKind::SteelAngle => "鋼 L形",
            ShapeKind::SteelChannel => "鋼 C形",
            ShapeKind::SteelTee => "鋼 T形",
            ShapeKind::SteelPipe => "鋼 丸鋼管",
            ShapeKind::RcRect => "RC 矩形",
            ShapeKind::RcCircle => "RC 円形",
        }
    }
    pub const ALL: [ShapeKind; 8] = [
        ShapeKind::SteelH,
        ShapeKind::SteelBox,
        ShapeKind::SteelAngle,
        ShapeKind::SteelChannel,
        ShapeKind::SteelTee,
        ShapeKind::SteelPipe,
        ShapeKind::RcRect,
        ShapeKind::RcCircle,
    ];
}

/// カタログ選択UIの選択状態（Shape→Family→Name の3段階）。
#[derive(Debug, Clone)]
pub struct CatalogDraft {
    pub shape: CatalogShape,
    pub family: Option<String>,
    pub name: Option<String>,
}

impl Default for CatalogDraft {
    fn default() -> Self {
        Self {
            shape: CatalogShape::H,
            family: None,
            name: None,
        }
    }
}

/// 断面カタログ選択パネル（日本国内規格）。Shape → Family → Name の順に絞り込み、
/// 選んだ断面をそのまま `Section` として追加する。パラメトリック作成（[`section_editor_panel`]）
/// とは別の独立した UI。
pub fn catalog_section_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.group(|ui| {
        ui.strong("断面カタログから選択（日本国内規格）");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Shape:");
            for s in CatalogShape::ALL {
                let cur = app.catalog_draft.shape;
                if ui.selectable_label(cur == s, s.label()).clicked() && cur != s {
                    app.catalog_draft.shape = s;
                    app.catalog_draft.family = None;
                    app.catalog_draft.name = None;
                }
            }
        });

        let families = squid_n_section::catalog::families(app.catalog_draft.shape);
        if families.is_empty() {
            ui.label("該当する断面がありません");
            return;
        }
        let family = app
            .catalog_draft
            .family
            .clone()
            .filter(|f| families.contains(&f.as_str()))
            .unwrap_or_else(|| families[0].to_string());
        if app.catalog_draft.family.as_deref() != Some(family.as_str()) {
            app.catalog_draft.family = Some(family.clone());
            app.catalog_draft.name = None;
        }

        ui.horizontal(|ui| {
            ui.label("Family:");
            egui::ComboBox::from_id_salt("catalog_family_select")
                .selected_text(&family)
                .show_ui(ui, |ui| {
                    for f in &families {
                        if ui.selectable_label(family == *f, *f).clicked() {
                            app.catalog_draft.family = Some((*f).to_string());
                            app.catalog_draft.name = None;
                        }
                    }
                });
        });

        let entries = squid_n_section::catalog::entries_in(app.catalog_draft.shape, &family);
        if entries.is_empty() {
            return;
        }
        let name = app
            .catalog_draft
            .name
            .clone()
            .filter(|n| entries.iter().any(|e| &e.name == n))
            .unwrap_or_else(|| entries[0].name.clone());
        if app.catalog_draft.name.as_deref() != Some(name.as_str()) {
            app.catalog_draft.name = Some(name.clone());
        }

        ui.horizontal(|ui| {
            ui.label("Name:");
            egui::ComboBox::from_id_salt("catalog_name_select")
                .selected_text(&name)
                .show_ui(ui, |ui| {
                    for e in &entries {
                        if ui.selectable_label(name == e.name, &e.name).clicked() {
                            app.catalog_draft.name = Some(e.name.clone());
                        }
                    }
                });
        });

        let Some(entry) = entries.iter().find(|e| e.name == name) else {
            return;
        };
        ui.separator();
        ui.label(format!(
            "算定: A = {:.3e} mm²   Iy = {:.3e} mm⁴   Iz = {:.3e} mm⁴   J = {:.3e} mm⁴",
            entry.area, entry.iy, entry.iz, entry.j
        ));

        if ui.button("+ 追加").clicked() {
            let new_id = SectionId(app.model.sections.len() as u32);
            let sec = squid_n_section::catalog::to_section(entry, new_id);
            let index = app.model.sections.len();
            app.undo
                .run(&mut app.model, Box::new(AddSection { old: sec, index }));
            app.staleness.mark_edited();
        }
    });
}

/// 断面作成パネル。モデルタブの断面サブタブに併置。
pub fn section_editor_panel(ui: &mut egui::Ui, app: &mut App) {
    let draft = &mut app.section_draft;

    ui.group(|ui| {
        ui.strong("断面作成");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("種別:");
            for k in ShapeKind::ALL {
                if ui.selectable_label(draft.kind == k, k.label()).clicked() {
                    draft.kind = k;
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("名称:");
            ui.text_edit_singleline(&mut draft.name);
        });
        ui.separator();

        let mut predicted_id = SectionId(app.model.sections.len() as u32);

        // 寸法入力
        match draft.kind {
            ShapeKind::SteelH => {
                steel_h_fields(ui, draft);
            }
            ShapeKind::SteelBox => {
                steel_box_fields(ui, draft);
            }
            ShapeKind::SteelAngle => {
                steel_angle_fields(ui, draft);
            }
            ShapeKind::SteelChannel => {
                steel_channel_fields(ui, draft);
            }
            ShapeKind::SteelTee => {
                steel_tee_fields(ui, draft);
            }
            ShapeKind::SteelPipe => {
                steel_pipe_fields(ui, draft);
            }
            ShapeKind::RcRect => {
                rc_rect_fields(ui, draft);
            }
            ShapeKind::RcCircle => {
                rc_circle_fields(ui, draft);
            }
        }

        ui.separator();

        let shape = build_shape(draft);
        let sec = shape.to_section(
            SectionId(app.model.sections.len() as u32),
            draft.name.clone(),
        );
        // プレビュー：A/Iy/Iz/J を表示
        ui.label(format!(
            "算定: A = {:.3e} mm²   Iy = {:.3e} mm⁴   Iz = {:.3e} mm⁴   J = {:.3e} mm⁴",
            sec.area, sec.iy, sec.iz, sec.j
        ));

        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("+ 追加").clicked() {
                predicted_id = SectionId(app.model.sections.len() as u32);
                app.undo.run(
                    &mut app.model,
                    Box::new(AddSectionShape {
                        shape: shape.clone(),
                        new_id: predicted_id,
                        name: draft.name.clone(),
                    }),
                );
                app.staleness.mark_edited();
                // 生成後、次の断面をすぐ作れるよう名称を更新
                let n = app.model.sections.len();
                draft.name = format!("断面{}", n + 1);
            }
            ui.separator();

            let focus = focused_section_index(app.nav.focus_section, &app.model.sections);
            let apply_resp = ui.add_enabled(focus.is_some(), egui::Button::new("✏ 選択断面へ適用"));
            match focus {
                Some(idx) => {
                    let sid = app.model.sections[idx].id;
                    let name = app.model.sections[idx].name.clone();
                    let used = app
                        .model
                        .elements
                        .iter()
                        .filter(|e| e.section == Some(sid))
                        .count();
                    let apply_resp = apply_resp.on_hover_text(format!(
                        "現在のフォーム内容で断面 {name} の形状を再定義します\
（この断面を使う全 {used} 部材に波及。名称 {name} は維持されます）"
                    ));
                    if apply_resp.clicked() {
                        app.undo.run(
                            &mut app.model,
                            Box::new(EditSectionShape {
                                section: sid,
                                new_shape: shape.clone(),
                            }),
                        );
                        app.staleness.mark_edited();
                    }
                }
                None => {
                    apply_resp.on_hover_text("断面テーブルで対象断面を選択してください");
                }
            }

            ui.separator();
            ui.label(format!("現在: {}/セクション", app.model.sections.len()));
        });
    });
}

/// `focus_section`（ナビゲータで選択中の断面）が現在も存在するか確認し、
/// 存在すれば `sections` 内のインデックスを返す。断面テーブル側での削除等で
/// 参照が古くなっている場合は `None`（ボタン無効化用）。
/// `App` 全体ではなく個別フィールドを引数に取ることで、呼び出し側の
/// `ui.group` クロージャが `app.section_draft` と `app.model` を disjoint に
/// 借用できるようにしている。
fn focused_section_index(
    focus_section: Option<SectionId>,
    sections: &[squid_n_core::model::Section],
) -> Option<usize> {
    let sid = focus_section?;
    let idx = sid.index();
    if idx < sections.len() && sections[idx].id == sid {
        Some(idx)
    } else {
        None
    }
}

fn steel_h_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_box_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("t 板厚:");
        num_field(ui, &mut d.t);
    });
}

fn steel_angle_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("A 脚長:");
        num_field(ui, &mut d.leg_a);
        ui.label("B 脚長:");
        num_field(ui, &mut d.leg_b);
        ui.label("t 厚:");
        num_field(ui, &mut d.leg_thick);
    });
}

fn steel_channel_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_tee_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_pipe_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("D 外径:");
        num_field(ui, &mut d.outer_dia);
        ui.label("t 板厚:");
        num_field(ui, &mut d.thick);
    });
}

fn rc_rect_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("B 幅:");
        num_field(ui, &mut d.rc_b);
        ui.label("D せい:");
        num_field(ui, &mut d.rc_d);
    });
    rc_rebar_fields(ui, d);
}

fn rc_circle_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("D 径:");
        num_field(ui, &mut d.rc_d);
    });
    rc_rebar_fields(ui, d);
}

fn rc_rebar_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.separator();
    ui.strong("配筋");
    ui.horizontal(|ui| {
        ui.label("X主筋 本数:");
        int_field(ui, &mut d.main_x_count);
        ui.label("径:");
        num_field(ui, &mut d.main_x_dia);
        ui.label("段数:");
        int_field(ui, &mut d.main_x_layers);
    });
    ui.horizontal(|ui| {
        ui.label("Y主筋 本数:");
        int_field(ui, &mut d.main_y_count);
        ui.label("径:");
        num_field(ui, &mut d.main_y_dia);
        ui.label("段数:");
        int_field(ui, &mut d.main_y_layers);
    });
    ui.horizontal(|ui| {
        ui.label("かぶり:");
        num_field(ui, &mut d.cover);
    });
    ui.horizontal(|ui| {
        ui.label("せん断補強筋 径:");
        num_field(ui, &mut d.shear_dia);
        ui.label("ピッチ:");
        num_field(ui, &mut d.shear_pitch);
        ui.label("組数:");
        int_field(ui, &mut d.shear_legs);
    });
}

fn num_field(ui: &mut egui::Ui, val: &mut f64) {
    ui.add(
        egui::DragValue::new(val)
            .speed(1.0)
            .range(0.0..=1e6)
            .max_decimals(1),
    );
}

fn int_field(ui: &mut egui::Ui, val: &mut u32) {
    let mut tmp = *val as f64;
    let resp = ui.add(
        egui::DragValue::new(&mut tmp)
            .speed(1.0)
            .range(0.0..=10_000.0)
            .max_decimals(0),
    );
    if resp.changed() {
        let n = tmp.round() as u32;
        if n != *val {
            *val = n;
        }
    }
}

fn build_rebar(d: &SectionEditorDraft) -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: d.main_x_count,
            dia: d.main_x_dia,
            layers: d.main_x_layers,
        },
        main_y: BarSet {
            count: d.main_y_count,
            dia: d.main_y_dia,
            layers: d.main_y_layers,
        },
        cover: d.cover,
        shear: ShearBar {
            dia: d.shear_dia,
            pitch: d.shear_pitch,
            legs: d.shear_legs,
        },
    }
}

fn build_shape(d: &SectionEditorDraft) -> SectionShape {
    match d.kind {
        ShapeKind::SteelH => SectionShape::SteelH {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelBox => SectionShape::SteelBox {
            height: d.h,
            width: d.b,
            thick: d.t,
        },
        ShapeKind::SteelAngle => SectionShape::SteelAngle {
            leg_a: d.leg_a,
            leg_b: d.leg_b,
            thick: d.leg_thick,
        },
        ShapeKind::SteelChannel => SectionShape::SteelChannel {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelTee => SectionShape::SteelTee {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelPipe => SectionShape::SteelPipe {
            outer_dia: d.outer_dia,
            thick: d.thick,
        },
        ShapeKind::RcRect => SectionShape::RcRect {
            b: d.rc_b,
            d: d.rc_d,
            rebar: build_rebar(d),
        },
        ShapeKind::RcCircle => SectionShape::RcCircle {
            d: d.rc_d,
            rebar: build_rebar(d),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_shape_steel_h_uses_draft_fields() {
        let d = SectionEditorDraft {
            h: 500.0,
            b: 250.0,
            tw: 10.0,
            tf: 15.0,
            ..SectionEditorDraft::default()
        };
        let s = build_shape(&d);
        if let SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } = s
        {
            assert_eq!(height, 500.0);
            assert_eq!(width, 250.0);
            assert_eq!(web_thick, 10.0);
            assert_eq!(flange_thick, 15.0);
        } else {
            panic!("expected SteelH");
        }
    }

    #[test]
    fn test_build_shape_rc_rect_includes_rebar() {
        let d = SectionEditorDraft {
            kind: ShapeKind::RcRect,
            rc_b: 400.0,
            rc_d: 800.0,
            main_x_count: 8,
            main_x_dia: 25.0,
            ..SectionEditorDraft::default()
        };
        let s = build_shape(&d);
        if let SectionShape::RcRect { b, d, rebar } = s {
            assert_eq!(b, 400.0);
            assert_eq!(d, 800.0);
            assert_eq!(rebar.main_x.count, 8);
            assert_eq!(rebar.main_x.dia, 25.0);
        } else {
            panic!("expected RcRect");
        }
    }

    #[test]
    fn test_to_section_preview_matches_drafted_shape() {
        let d = SectionEditorDraft::default();
        let s = build_shape(&d);
        let sec = s.to_section(SectionId(0), "test".into());
        // H 400x200x8x12 の A は閉形式
        let expected = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 8.0;
        assert!((sec.area - expected).abs() < 1e-9);
    }

    /// 「選択断面へ適用」ボタンが発行する `EditSectionShape` を undo.run 経由で
    /// 適用すると断面性能（A 等）が再算定され、undo で元の断面形状に戻ることを確認する。
    /// GUI（egui）非依存で、draft→shape 構築 (`build_shape`) と undo スタックのみで検証する。
    #[test]
    fn test_edit_section_shape_via_undo_recomputes_and_reverts() {
        use squid_n_core::model::Model;
        use squid_n_edit::UndoStack;

        // 既存断面（H 400x200x8x12）を用意
        let old_draft = SectionEditorDraft::default();
        let old_shape = build_shape(&old_draft);
        let sid = SectionId(0);
        let old_sec = old_shape.to_section(sid, "既存断面".to_string());
        let old_area = old_sec.area;

        let mut model = Model::default();
        model.sections.push(old_sec);

        // フォーム（draft）で寸法を変更 → 断面編集パネルの「適用」と同じ経路で shape を構築
        let new_draft = SectionEditorDraft {
            h: 500.0,
            b: 250.0,
            tw: 10.0,
            tf: 15.0,
            ..SectionEditorDraft::default()
        };
        let new_shape = build_shape(&new_draft);
        let new_area_expected = new_shape.to_section(sid, "既存断面".into()).area;
        assert!((new_area_expected - old_area).abs() > 1e-6);

        let mut undo = UndoStack::new();
        undo.run(
            &mut model,
            Box::new(EditSectionShape {
                section: sid,
                new_shape,
            }),
        );

        // 再算定された断面性能が反映され、名称は維持される
        assert!((model.sections[0].area - new_area_expected).abs() < 1e-6);
        assert_eq!(model.sections[0].name, "既存断面");

        // undo で元の形状・断面性能に戻る
        undo.undo(&mut model);
        assert!((model.sections[0].area - old_area).abs() < 1e-6);
        assert_eq!(model.sections[0].name, "既存断面");
    }
}
