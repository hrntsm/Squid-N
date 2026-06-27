use crate::app::App;

#[derive(Clone, Default)]
pub struct TimeHistoryData {
    pub time: Vec<f64>,
    pub node_disp: Vec<f64>,
    pub story_shear: Vec<f64>,
    pub story_drift_angle: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum TimeHistorySource {
    #[default]
    NodeDisp,
    StoryShear,
    StoryDriftAngle,
}

pub fn dummy_time_history() -> TimeHistoryData {
    let n = 200;
    let dt = 0.01;
    TimeHistoryData {
        time: (0..n).map(|i| i as f64 * dt).collect(),
        node_disp: (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (t * 10.0).sin() * (-t * 0.5).exp() * 10.0
            })
            .collect(),
        story_shear: (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (t * 8.0).sin() * (-t * 0.3).exp() * 5000.0
            })
            .collect(),
        story_drift_angle: (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (t * 12.0).sin() * (-t * 0.4).exp() * 0.005
            })
            .collect(),
    }
}

pub fn time_history_panel(ui: &mut egui::Ui, app: &mut App) {
    let mut source = app.time_history_source;

    ui.horizontal(|ui| {
        ui.label("表示項目:");
        ui.selectable_value(&mut source, TimeHistorySource::NodeDisp, "節点変位");
        ui.selectable_value(&mut source, TimeHistorySource::StoryShear, "層せん断");
        ui.selectable_value(
            &mut source,
            TimeHistorySource::StoryDriftAngle,
            "層間変形角",
        );
    });

    ui.add_space(4.0);

    if source != app.time_history_source {
        app.time_history_source = source;
    }

    let data = &app.time_history_data;
    let values: Vec<[f64; 2]> = match source {
        TimeHistorySource::NodeDisp => data
            .time
            .iter()
            .zip(data.node_disp.iter())
            .map(|(&t, &v)| [t, v])
            .collect(),
        TimeHistorySource::StoryShear => data
            .time
            .iter()
            .zip(data.story_shear.iter())
            .map(|(&t, &v)| [t, v])
            .collect(),
        TimeHistorySource::StoryDriftAngle => data
            .time
            .iter()
            .zip(data.story_drift_angle.iter())
            .map(|(&t, &v)| [t, v])
            .collect(),
    };

    // §3 データビジュアライゼーション配色（系列ごとに弁別可能な 3 色）
    let (ylabel, line_color) = match source {
        TimeHistorySource::NodeDisp => ("変位 [mm]", crate::theme::DATA_BLUE),
        TimeHistorySource::StoryShear => ("層せん断 [N]", crate::theme::PARETO_RED),
        TimeHistorySource::StoryDriftAngle => ("層間変形角 [rad]", crate::theme::GOOD_GREEN),
    };

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
        let max_t = data.time.last().copied().unwrap_or(1.0);
        let pointer_value = plot.transform.value_from_position(pointer);
        let idx_time = pointer_value.x.max(0.0).min(max_t);
        let idx_floor = idx_time as usize;
        if idx_floor < data.time.len() {
            let t = data.time[idx_floor];
            let val = match source {
                TimeHistorySource::NodeDisp => data.node_disp[idx_floor],
                TimeHistorySource::StoryShear => data.story_shear[idx_floor],
                TimeHistorySource::StoryDriftAngle => data.story_drift_angle[idx_floor],
            };
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
