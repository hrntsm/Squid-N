//! S 造部材の断面検定用属性（`Model.steel_design_attrs` = `SteelDesignAttr`:
//! 継手フランジ／ウェブ欠損率・スカラップ欠損率、横座屈長さ lb の直接入力、
//! 等間隔横補剛本数、座屈長さ lk_y/lk_z の直接入力、横座屈修正係数 C の
//! 直接入力）の編集 UI。
//! 対象は `ElementKind::Beam` の部材のみ（本ソフトでは柱も梁もブレースも
//! `ElementKind::Beam` で表現するフレーム部材のため、実質すべての一般部材が
//! 対象。座屈長さ lk_y/lk_z の直接入力は柱以外の部材にも有効）。
//! 編集は `squid_n_edit::{SetSteelDesignAttr, RemoveSteelDesignAttr}` 経由
//! （undo 対応）。空（すべて既定値）を適用した場合は削除として扱う
//! （[`SteelDesignAttr::is_empty`]）。
//!
//! `member_details.rs` のドラフト同期パターン（`elem`/`synced_for`、選択時に
//! model 値でバッファ初期化、「適用」で Set／空なら Remove）を踏襲する。

use crate::app::App;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, SteelDesignAttr};
use squid_n_edit::{RemoveSteelDesignAttr, SetSteelDesignAttr};

/// S造検定属性フォームのドラフト状態（GUI 専用）。
/// 対象部材を選択すると `synced_for` の部材の現在値でバッファを初期化し、
/// 「適用」で `SetSteelDesignAttr`（空なら `RemoveSteelDesignAttr`）を発行する。
#[derive(Clone, Debug, Default)]
pub struct SteelAttrDraft {
    /// 編集対象の部材要素。
    pub elem: Option<ElemId>,
    /// バッファを初期化した対象（`elem` と異なれば model 値で再同期する）。
    pub synced_for: Option<ElemId>,
    /// 継手フランジ欠損率 βf [%] の入力バッファ（空欄=0）。
    pub joint_flange_loss: String,
    /// 継手ウェブ欠損率 βw [%] の入力バッファ（空欄=0）。
    pub joint_web_loss: String,
    /// スカラップ欠損率 αw [%] の入力バッファ（空欄=0）。
    pub scallop_web_loss: String,
    /// 横座屈長さ lb 直接入力・始端 [mm] の入力バッファ（空欄=自動）。
    pub lb_start: String,
    /// 横座屈長さ lb 直接入力・中央 [mm] の入力バッファ（空欄=自動）。
    pub lb_mid: String,
    /// 横座屈長さ lb 直接入力・終端 [mm] の入力バッファ（空欄=自動）。
    pub lb_end: String,
    /// 等間隔横補剛の本数の入力バッファ（空欄=なし）。
    pub lateral_brace_count: String,
    /// 強軸まわり座屈長さ lk_y の直接入力 [mm]（空欄=自動算定）。
    pub lk_y_direct: String,
    /// 弱軸まわり座屈長さ lk_z の直接入力 [mm]（空欄=自動算定）。
    pub lk_z_direct: String,
    /// 横座屈修正係数 C の直接入力（空欄=自動算定）。
    pub c_direct: String,
}

/// 欠損率入力（βf/βw/αw共通）をパースする。空欄は 0（欠損なし）とする
/// （egui 非依存の純関数）。
fn parse_loss_percent(s: &str, label: &str) -> Result<f64, String> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(0.0);
    }
    s.parse()
        .map_err(|_| format!("不正な{label}「{s}」: 数値ではありません"))
}

/// 横座屈長さ lb 直接入力（始端/中央/終端）をパースする。3 欄すべて空欄なら
/// `None`（自動）、いずれか 1 つでも入力があれば 3 欄とも数値必須とする
/// （egui 非依存の純関数）。
fn parse_lb_direct(start: &str, mid: &str, end: &str) -> Result<Option<(f64, f64, f64)>, String> {
    let (s, m, e) = (start.trim(), mid.trim(), end.trim());
    if s.is_empty() && m.is_empty() && e.is_empty() {
        return Ok(None);
    }
    if s.is_empty() || m.is_empty() || e.is_empty() {
        return Err(
            "横座屈長さ lb の直接入力は始端/中央/終端をすべて入力してください\
             （すべて空欄なら自動）"
                .to_string(),
        );
    }
    let parse_one = |v: &str, label: &str| -> Result<f64, String> {
        v.parse()
            .map_err(|_| format!("不正な{label}側 lb「{v}」: 数値ではありません"))
    };
    Ok(Some((
        parse_one(s, "始端")?,
        parse_one(m, "中央")?,
        parse_one(e, "終端")?,
    )))
}

/// 等間隔横補剛本数をパースする。空欄は `None`（補剛なし）とする
/// （egui 非依存の純関数）。
fn parse_brace_count(s: &str) -> Result<Option<u32>, String> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(None);
    }
    let n: u32 = s
        .parse()
        .map_err(|_| format!("不正な横補剛本数「{s}」: 0以上の整数を入力してください"))?;
    Ok(Some(n))
}

/// 座屈長さ lk_y/lk_z の直接入力をパースする。空欄は `None`（自動算定）、
/// 入力があれば正の値を要求する（egui 非依存の純関数）。
fn parse_lk_direct(s: &str, label: &str) -> Result<Option<f64>, String> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(None);
    }
    let v: f64 = s
        .parse()
        .map_err(|_| format!("不正な{label}「{s}」: 数値ではありません"))?;
    if v <= 0.0 {
        return Err(format!(
            "{label}は正の値で入力してください（空欄=自動算定）"
        ));
    }
    Ok(Some(v))
}

/// 横座屈長さ lb 直接入力の表示文字列。
fn lb_direct_desc(v: Option<(f64, f64, f64)>) -> String {
    match v {
        Some((s, m, e)) => format!("{s:.0}/{m:.0}/{e:.0}"),
        None => "自動".to_string(),
    }
}

/// 座屈長さ lk_y/lk_z 直接入力の表示文字列。
fn opt_len_desc(v: Option<f64>) -> String {
    match v {
        Some(x) => format!("{x:.0}"),
        None => "自動".to_string(),
    }
}

/// 横座屈修正係数 C 直接入力の表示文字列（小数第2位まで）。
fn opt_c_desc(v: Option<f64>) -> String {
    match v {
        Some(x) => format!("{x:.2}"),
        None => "自動".to_string(),
    }
}

/// 等間隔横補剛本数の表示文字列。
fn opt_count_desc(v: Option<u32>) -> String {
    match v {
        Some(n) => n.to_string(),
        None => "なし".to_string(),
    }
}

pub fn steel_attrs_table(ui: &mut egui::Ui, app: &mut App) {
    ui.label(
        "S造部材の断面検定用属性（継手・スカラップ欠損率、横座屈長さ lb の直接入力、\
         等間隔横補剛本数、座屈長さ lk_y/lk_z の直接入力、横座屈修正係数 C の直接入力）\
         を設定します。lk_y/lk_z の直接入力は柱の自動算定（K・部材長）より優先されます。\
         C の直接入力は自動算定（材端モーメント比・上限2.3）より優先されます。",
    );
    ui.separator();

    let target_elems: Vec<ElemId> = app
        .model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam)
        .map(|e| e.id)
        .collect();

    if target_elems.is_empty() {
        ui.label("対象となる部材(柱・梁・ブレース)がありません。");
        return;
    }

    // ── 既存の S造検定属性一覧 ─────────────────────────────
    let mut pending_remove: Option<ElemId> = None;
    let mut pending_edit: Option<SteelDesignAttr> = None;
    if app.model.steel_design_attrs.is_empty() {
        ui.label(
            "設定済みのS造検定属性はありません(未設定の部材は欠損なし・座屈長さ自動として扱われます)。",
        );
    } else {
        for attr in &app.model.steel_design_attrs {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "部材#{}: βf={:.1}% βw={:.1}% αw={:.1}% / lb={} / 横補剛n={} / \
                     lk_y={} lk_z={} / C={}",
                    attr.elem.0,
                    attr.joint_flange_loss,
                    attr.joint_web_loss,
                    attr.scallop_web_loss,
                    lb_direct_desc(attr.lb_direct),
                    opt_count_desc(attr.lateral_brace_count),
                    opt_len_desc(attr.lk_y_direct),
                    opt_len_desc(attr.lk_z_direct),
                    opt_c_desc(attr.c_direct),
                ));
                if ui
                    .button("✏")
                    .on_hover_text("フォームへ読み込んで編集")
                    .clicked()
                {
                    pending_edit = Some(attr.clone());
                }
                if ui
                    .button("🗑")
                    .on_hover_text("このS造検定属性を削除")
                    .clicked()
                {
                    pending_remove = Some(attr.elem);
                }
            });
        }
    }
    if let Some(attr) = pending_edit {
        app.steel_attr_draft.elem = Some(attr.elem);
        app.steel_attr_draft.synced_for = Some(attr.elem);
        app.steel_attr_draft.joint_flange_loss = format!("{:.1}", attr.joint_flange_loss);
        app.steel_attr_draft.joint_web_loss = format!("{:.1}", attr.joint_web_loss);
        app.steel_attr_draft.scallop_web_loss = format!("{:.1}", attr.scallop_web_loss);
        let (lb_s, lb_m, lb_e) = attr
            .lb_direct
            .map(|(s, m, e)| (format!("{s:.0}"), format!("{m:.0}"), format!("{e:.0}")))
            .unwrap_or_default();
        app.steel_attr_draft.lb_start = lb_s;
        app.steel_attr_draft.lb_mid = lb_m;
        app.steel_attr_draft.lb_end = lb_e;
        app.steel_attr_draft.lateral_brace_count = attr
            .lateral_brace_count
            .map(|n| n.to_string())
            .unwrap_or_default();
        app.steel_attr_draft.lk_y_direct = attr
            .lk_y_direct
            .map(|v| format!("{v:.0}"))
            .unwrap_or_default();
        app.steel_attr_draft.lk_z_direct = attr
            .lk_z_direct
            .map(|v| format!("{v:.0}"))
            .unwrap_or_default();
        app.steel_attr_draft.c_direct =
            attr.c_direct.map(|v| format!("{v:.2}")).unwrap_or_default();
    }
    if let Some(elem) = pending_remove {
        app.undo
            .run(&mut app.model, Box::new(RemoveSteelDesignAttr { elem }));
        app.staleness.mark_edited();
    }

    ui.separator();
    ui.strong("S造検定属性を設定");

    // 対象部材の選択(変更時に model 値でバッファを再同期)
    ui.horizontal(|ui| {
        ui.label("対象部材:");
        let text = app
            .steel_attr_draft
            .elem
            .map(|e| format!("部材#{}", e.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("steel_attr_elem")
            .selected_text(text)
            .show_ui(ui, |ui| {
                for &eid in &target_elems {
                    if ui
                        .selectable_label(
                            app.steel_attr_draft.elem == Some(eid),
                            format!("部材#{}", eid.0),
                        )
                        .clicked()
                    {
                        app.steel_attr_draft.elem = Some(eid);
                    }
                }
            });
    });
    if app.steel_attr_draft.elem != app.steel_attr_draft.synced_for {
        if let Some(eid) = app.steel_attr_draft.elem {
            let existing = app
                .model
                .steel_design_attrs
                .iter()
                .find(|a| a.elem == eid)
                .cloned();
            let attr = existing.unwrap_or(SteelDesignAttr {
                elem: eid,
                joint_flange_loss: 0.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 0.0,
                lb_direct: None,
                lateral_brace_count: None,
                lk_y_direct: None,
                lk_z_direct: None,
                c_direct: None,
            });
            app.steel_attr_draft.joint_flange_loss = format!("{:.1}", attr.joint_flange_loss);
            app.steel_attr_draft.joint_web_loss = format!("{:.1}", attr.joint_web_loss);
            app.steel_attr_draft.scallop_web_loss = format!("{:.1}", attr.scallop_web_loss);
            let (lb_s, lb_m, lb_e) = attr
                .lb_direct
                .map(|(s, m, e)| (format!("{s:.0}"), format!("{m:.0}"), format!("{e:.0}")))
                .unwrap_or_default();
            app.steel_attr_draft.lb_start = lb_s;
            app.steel_attr_draft.lb_mid = lb_m;
            app.steel_attr_draft.lb_end = lb_e;
            app.steel_attr_draft.lateral_brace_count = attr
                .lateral_brace_count
                .map(|n| n.to_string())
                .unwrap_or_default();
            app.steel_attr_draft.lk_y_direct = attr
                .lk_y_direct
                .map(|v| format!("{v:.0}"))
                .unwrap_or_default();
            app.steel_attr_draft.lk_z_direct = attr
                .lk_z_direct
                .map(|v| format!("{v:.0}"))
                .unwrap_or_default();
            app.steel_attr_draft.c_direct =
                attr.c_direct.map(|v| format!("{v:.2}")).unwrap_or_default();
            app.steel_attr_draft.synced_for = Some(eid);
        }
    }

    ui.horizontal(|ui| {
        ui.label("βf 継手フランジ欠損率[%]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.joint_flange_loss)
                .desired_width(60.0),
        );
        ui.label("βw 継手ウェブ欠損率[%]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.joint_web_loss)
                .desired_width(60.0),
        );
        ui.label("αw スカラップ欠損率[%]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.scallop_web_loss)
                .desired_width(60.0),
        );
    });

    ui.horizontal(|ui| {
        ui.label("横座屈長さ lb 直接入力[mm] 始端:");
        ui.add(egui::TextEdit::singleline(&mut app.steel_attr_draft.lb_start).desired_width(70.0));
        ui.label("中央:");
        ui.add(egui::TextEdit::singleline(&mut app.steel_attr_draft.lb_mid).desired_width(70.0));
        ui.label("終端:");
        ui.add(egui::TextEdit::singleline(&mut app.steel_attr_draft.lb_end).desired_width(70.0));
    })
    .response
    .on_hover_text("3欄すべて空欄なら自動算定(等間隔横補剛本数、または部材長)。");

    ui.horizontal(|ui| {
        ui.label("等間隔横補剛本数:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.lateral_brace_count)
                .desired_width(50.0),
        )
        .on_hover_text(
            "空欄=補剛なし。lb 直接入力が空欄の場合、lb=部材長/(本数+1) の自動算定に用います。",
        );
    });

    ui.horizontal(|ui| {
        ui.label("座屈長さ lk_y(強軸)直接入力[mm]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.lk_y_direct)
                .desired_width(80.0),
        );
        ui.label("lk_z(弱軸)直接入力[mm]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.steel_attr_draft.lk_z_direct)
                .desired_width(80.0),
        );
    })
    .response
    .on_hover_text(
        "空欄=自動算定(柱はK・部材長、柱以外は部材長)。指定した場合は柱の自動算定より優先されます。",
    );

    ui.horizontal(|ui| {
        ui.label("C 係数直接入力（空欄=自動）:");
        ui.add(egui::TextEdit::singleline(&mut app.steel_attr_draft.c_direct).desired_width(60.0));
    })
    .response
    .on_hover_text(
        "空欄=自動算定(材端モーメント比M2/M1・上限2.3)。指定した場合は自動算定・上限2.3を行わず入力値をそのまま採用します。",
    );

    let parsed_flange = parse_loss_percent(&app.steel_attr_draft.joint_flange_loss, "βf");
    let parsed_web = parse_loss_percent(&app.steel_attr_draft.joint_web_loss, "βw");
    let parsed_scallop = parse_loss_percent(&app.steel_attr_draft.scallop_web_loss, "αw");
    let parsed_lb = parse_lb_direct(
        &app.steel_attr_draft.lb_start,
        &app.steel_attr_draft.lb_mid,
        &app.steel_attr_draft.lb_end,
    );
    let parsed_brace_count = parse_brace_count(&app.steel_attr_draft.lateral_brace_count);
    let parsed_lk_y = parse_lk_direct(&app.steel_attr_draft.lk_y_direct, "lk_y");
    let parsed_lk_z = parse_lk_direct(&app.steel_attr_draft.lk_z_direct, "lk_z");
    let parsed_c = parse_lk_direct(&app.steel_attr_draft.c_direct, "C係数");

    for (label, r) in [
        ("βf", parsed_flange.as_ref().err()),
        ("βw", parsed_web.as_ref().err()),
        ("αw", parsed_scallop.as_ref().err()),
        ("lb", parsed_lb.as_ref().err()),
        ("横補剛本数", parsed_brace_count.as_ref().err()),
        ("lk_y", parsed_lk_y.as_ref().err()),
        ("lk_z", parsed_lk_z.as_ref().err()),
        ("C係数", parsed_c.as_ref().err()),
    ] {
        if let Some(e) = r {
            ui.colored_label(crate::theme::ERROR_RED, format!("{label}の入力エラー: {e}"));
        }
    }

    let can_apply = app.steel_attr_draft.elem.is_some()
        && parsed_flange.is_ok()
        && parsed_web.is_ok()
        && parsed_scallop.is_ok()
        && parsed_lb.is_ok()
        && parsed_brace_count.is_ok()
        && parsed_lk_y.is_ok()
        && parsed_lk_z.is_ok()
        && parsed_c.is_ok();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(can_apply, egui::Button::new("✔ 適用"))
            .on_hover_text(
                "選択した部材にS造検定属性を設定します(すべて既定値の場合は削除します。undo可)",
            )
            .clicked()
        {
            if let (
                Some(elem),
                Ok(joint_flange_loss),
                Ok(joint_web_loss),
                Ok(scallop_web_loss),
                Ok(lb_direct),
                Ok(lateral_brace_count),
                Ok(lk_y_direct),
                Ok(lk_z_direct),
                Ok(c_direct),
            ) = (
                app.steel_attr_draft.elem,
                parsed_flange,
                parsed_web,
                parsed_scallop,
                parsed_lb,
                parsed_brace_count,
                parsed_lk_y,
                parsed_lk_z,
                parsed_c,
            ) {
                let attr = SteelDesignAttr {
                    elem,
                    joint_flange_loss,
                    joint_web_loss,
                    scallop_web_loss,
                    lb_direct,
                    lateral_brace_count,
                    lk_y_direct,
                    lk_z_direct,
                    c_direct,
                };
                if attr.is_empty() {
                    app.undo
                        .run(&mut app.model, Box::new(RemoveSteelDesignAttr { elem }));
                } else {
                    app.undo
                        .run(&mut app.model, Box::new(SetSteelDesignAttr { attr }));
                }
                app.staleness.mark_edited();
            }
        }
        if app.steel_attr_draft.elem.is_some()
            && ui
                .button("🗑 この部材のS造検定属性を削除")
                .on_hover_text("選択した部材のS造検定属性を削除します(undo可)")
                .clicked()
        {
            if let Some(elem) = app.steel_attr_draft.elem {
                app.undo
                    .run(&mut app.model, Box::new(RemoveSteelDesignAttr { elem }));
                app.staleness.mark_edited();
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 空欄は 0（欠損なし）として扱われること。
    #[test]
    fn test_parse_loss_percent_empty_is_zero() {
        assert_eq!(parse_loss_percent("", "βf").unwrap(), 0.0);
        assert_eq!(parse_loss_percent("   ", "βf").unwrap(), 0.0);
    }

    /// 数値でない入力はエラーになること。
    #[test]
    fn test_parse_loss_percent_rejects_non_numeric() {
        let err = parse_loss_percent("abc", "βf").unwrap_err();
        assert!(err.contains("βf"), "err={err}");
    }

    /// 3欄すべて空欄なら None（自動）になること。
    #[test]
    fn test_parse_lb_direct_all_empty_is_none() {
        assert_eq!(parse_lb_direct("", "", "").unwrap(), None);
        assert_eq!(parse_lb_direct("  ", " ", "").unwrap(), None);
    }

    /// 3欄すべて入力されていれば Some のタプルになること。
    #[test]
    fn test_parse_lb_direct_all_filled() {
        assert_eq!(
            parse_lb_direct("1000", "2000", "3000").unwrap(),
            Some((1000.0, 2000.0, 3000.0))
        );
    }

    /// 一部だけ入力されている場合はエラーになること。
    #[test]
    fn test_parse_lb_direct_partial_is_error() {
        assert!(parse_lb_direct("1000", "", "").is_err());
        assert!(parse_lb_direct("", "2000", "3000").is_err());
    }

    /// 数値でない入力はエラーになること。
    #[test]
    fn test_parse_lb_direct_rejects_non_numeric() {
        assert!(parse_lb_direct("abc", "2000", "3000").is_err());
    }

    /// 空欄は None（補剛なし）として扱われること。
    #[test]
    fn test_parse_brace_count_empty_is_none() {
        assert_eq!(parse_brace_count("").unwrap(), None);
    }

    /// 正の整数はそのままパースされること。
    #[test]
    fn test_parse_brace_count_valid() {
        assert_eq!(parse_brace_count("3").unwrap(), Some(3));
        assert_eq!(parse_brace_count("0").unwrap(), Some(0));
    }

    /// 整数でない入力はエラーになること。
    #[test]
    fn test_parse_brace_count_rejects_non_integer() {
        assert!(parse_brace_count("abc").is_err());
        assert!(parse_brace_count("-1").is_err());
        assert!(parse_brace_count("1.5").is_err());
    }

    /// 空欄は None（自動算定）として扱われること。
    #[test]
    fn test_parse_lk_direct_empty_is_none() {
        assert_eq!(parse_lk_direct("", "lk_y").unwrap(), None);
    }

    /// 正の値はそのままパースされること。
    #[test]
    fn test_parse_lk_direct_valid() {
        assert_eq!(parse_lk_direct("3500", "lk_y").unwrap(), Some(3500.0));
    }

    /// 0 以下・数値でない入力はエラーになること。
    #[test]
    fn test_parse_lk_direct_rejects_non_positive_or_invalid() {
        assert!(parse_lk_direct("0", "lk_y").is_err());
        assert!(parse_lk_direct("-100", "lk_y").is_err());
        assert!(parse_lk_direct("abc", "lk_y").is_err());
    }

    /// C 係数直接入力も lk_y/lk_z と同じ正値パーサを再利用できること
    /// （空欄=自動、正の値はそのまま、0以下・非数値はエラー）。
    #[test]
    fn test_parse_c_direct_reuses_parse_lk_direct() {
        assert_eq!(parse_lk_direct("", "C係数").unwrap(), None);
        assert_eq!(parse_lk_direct("1.5", "C係数").unwrap(), Some(1.5));
        assert!(parse_lk_direct("0", "C係数").is_err());
        assert!(parse_lk_direct("-1.0", "C係数").is_err());
    }

    /// 表示用整形（lb 直接入力・座屈長さ・横補剛本数）が None/Some で
    /// 期待通りの文字列になること。
    #[test]
    fn test_desc_helpers() {
        assert_eq!(lb_direct_desc(None), "自動");
        assert_eq!(
            lb_direct_desc(Some((1000.0, 2000.0, 3000.0))),
            "1000/2000/3000"
        );
        assert_eq!(opt_len_desc(None), "自動");
        assert_eq!(opt_len_desc(Some(3500.0)), "3500");
        assert_eq!(opt_count_desc(None), "なし");
        assert_eq!(opt_count_desc(Some(3)), "3");
        assert_eq!(opt_c_desc(None), "自動");
        assert_eq!(opt_c_desc(Some(1.5)), "1.50");
    }
}
