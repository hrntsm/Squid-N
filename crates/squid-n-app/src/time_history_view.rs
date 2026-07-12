use crate::app::App;

/// 時刻歴グラフの描画データ。`App::run_time_history` が
/// ソルバーの `ResponseResult.history` から充填する。
#[derive(Clone, Default)]
pub struct TimeHistoryData {
    pub time: Vec<f64>,
    /// 記録節点の X 方向相対変位 [mm]
    pub node_disp: Vec<f64>,
    /// ベースシア(X) [N]
    pub story_shear: Vec<f64>,
    /// 最上階の層間変形角 [rad]
    pub story_drift_angle: Vec<f64>,
    /// 記録節点
    pub node: Option<squid_n_core::ids::NodeId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum TimeHistorySource {
    #[default]
    NodeDisp,
    StoryShear,
    StoryDriftAngle,
}

pub fn time_history_panel(ui: &mut egui::Ui, app: &mut App) {
    if app.time_history_data.time.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "時刻歴応答データがありません。解析タブの「時刻歴」から実行してください。",
        );
        return;
    }

    let mut source = app.time_history_source;

    ui.horizontal(|ui| {
        ui.label("表示項目:");
        let node_label = app
            .time_history_data
            .node
            .map(|n| format!("節点 N{} 変位", n.0))
            .unwrap_or_else(|| "節点変位".to_string());
        ui.selectable_value(&mut source, TimeHistorySource::NodeDisp, node_label);
        ui.selectable_value(&mut source, TimeHistorySource::StoryShear, "ベースシア");
        ui.selectable_value(
            &mut source,
            TimeHistorySource::StoryDriftAngle,
            "層間変形角(最上階)",
        );
    });

    ui.add_space(4.0);

    if source != app.time_history_source {
        app.time_history_source = source;
    }

    let data = &app.time_history_data;
    let series = match source {
        TimeHistorySource::NodeDisp => &data.node_disp,
        TimeHistorySource::StoryShear => &data.story_shear,
        TimeHistorySource::StoryDriftAngle => &data.story_drift_angle,
    };
    let values: Vec<[f64; 2]> = data
        .time
        .iter()
        .zip(series.iter())
        .map(|(&t, &v)| [t, v])
        .collect();

    // §3 データビジュアライゼーション配色（系列ごとに弁別可能な 3 色）
    let (ylabel, line_color) = match source {
        TimeHistorySource::NodeDisp => ("変位 [mm]", crate::theme::DATA_BLUE),
        TimeHistorySource::StoryShear => ("ベースシア [N]", crate::theme::PARETO_RED),
        TimeHistorySource::StoryDriftAngle => ("層間変形角 [rad]", crate::theme::GOOD_GREEN),
    };

    // ピーク値サマリ
    let peak = series.iter().cloned().fold(0.0f64, |m, v| m.max(v.abs()));
    ui.label(format!("最大絶対値: {:.4e}", peak));

    // レインフロー計数（RESP-D「07 非線形解析（動的解析）」その他の解析機能の
    // 累積損傷度計算で用いる ASTM E1049 3 点法）。表示中の代表応答に対する
    // 等価繰返し数・最大振れ幅を参考表示する（累積損傷度 D の梁端 μ 収集は今後の拡張）。
    let cycles = squid_n_solver::damage::rainflow_cycles(series);
    let neq: f64 = cycles.iter().map(|c| c.count).sum();
    let max_range = cycles.iter().map(|c| c.range).fold(0.0f64, f64::max);
    ui.label(format!(
        "レインフロー(代表応答): 等価繰返し数 {:.1} 回 / 最大振れ幅 {:.4e}",
        neq, max_range
    ))
    .on_hover_text("累積損傷度計算(レインフロー法)の基礎計数（ASTM E1049 3 点法）。");

    // 梁端累積損傷度 D（RESP-D「07」鉄骨梁端部の累積損傷度計算）。非線形時刻歴で
    // 各要素の危険断面塑性率 μ 時刻歴からレインフロー法で算定した値を表示する。
    if let Some(res) = app.results.as_ref().and_then(|r| r.time_history.as_ref()) {
        let dmax = res
            .cumulative_ductility
            .iter()
            .cloned()
            .fold(0.0f64, f64::max);
        let n_damaged = res
            .cumulative_ductility
            .iter()
            .filter(|&&d| d > 0.0)
            .count();
        if dmax > 0.0 {
            // 最大 D の要素 ID。
            let imax = res
                .cumulative_ductility
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);
            ui.label(format!(
                "梁端累積損傷度 D: 最大 {:.3}（部材 {}） / 損傷要素 {} 件（レインフロー法）",
                dmax, imax, n_damaged
            ))
            .on_hover_text(
                "非線形時刻歴で塑性化した要素の危険断面塑性率 μ の時刻歴から算定。\
                 D≥1 で疲労破断（疲労特性 C・β は暫定既定、鋼種・接合形式で要照合）。",
            );
        } else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "梁端累積損傷度 D: 塑性化要素なし（非線形時刻歴で塑性率を収集）。",
            );
        }
    }

    let plot = egui_plot::Plot::new("time_history_plot")
        .legend(egui_plot::Legend::default())
        .x_axis_label("時間 [s]")
        .y_axis_label(ylabel)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("series", egui_plot::PlotPoints::from(values))
                    .color(line_color)
                    .width(1.5),
            );
        });

    // カーソル位置の値を表示
    if let Some(pointer) = plot.response.hover_pos() {
        let pointer_value = plot.transform.value_from_position(pointer);
        let dt = if data.time.len() >= 2 {
            (data.time[data.time.len() - 1] - data.time[0]) / (data.time.len() - 1) as f64
        } else {
            1.0
        };
        let idx = ((pointer_value.x - data.time[0]) / dt).round().max(0.0) as usize;
        if idx < data.time.len() && idx < series.len() {
            let t = data.time[idx];
            let val = series[idx];
            ui.horizontal(|ui| {
                ui.label(format!("t = {:.3} s", t));
                ui.separator();
                ui.label(match source {
                    TimeHistorySource::NodeDisp => format!("変位 = {:.3} mm", val),
                    TimeHistorySource::StoryShear => format!("せん断 = {:.3} N", val),
                    TimeHistorySource::StoryDriftAngle => format!("変形角 = {:.6} rad", val),
                });
            });
        }
    }
}
