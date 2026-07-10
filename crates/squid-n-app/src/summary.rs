//! 層指標（二次設計チェック）とレポート文字列の生成。GUI 非依存。

use squid_n_core::model::Model;
use squid_n_design_jp::eccentricity::story_eccentricity;
use squid_n_design_jp::holding_capacity::{eccentricity_ratio, fes, stiffness_ratios};
use squid_n_solver::analysis::SeismicDir;

use crate::app::{App, ResultsBundle, StaticCaseKey};

/// 層ごとの二次設計指標（層間変形角・剛性率・偏心率・Fes）。
#[derive(Clone, Debug)]
pub struct StoryMetric {
    pub name: String,
    /// 階高 [mm]
    pub height: f64,
    /// 層間変位 [mm]（加力方向）
    pub drift: f64,
    /// 層間変形角 [rad]
    pub drift_angle: f64,
    /// 1/200 以下か（令82条の2）
    pub drift_ok: bool,
    /// 剛性率 Rs
    pub rs: f64,
    /// Rs ≥ 0.6 か（令82条の6）
    pub rs_ok: bool,
    /// 偏心率 Re（加力方向）
    pub re: f64,
    /// Re ≤ 0.15 か（令82条の6）
    pub re_ok: bool,
    /// 形状係数 Fes = Fs·Fe
    pub fes: f64,
}

/// 静的解析の変位から層指標を計算する。
/// `disp` は節点変位（`model.nodes` と同順）。階が未定義なら空を返す。
pub fn compute_story_metrics(
    model: &Model,
    disp: &[[f64; 6]],
    dir: SeismicDir,
) -> Vec<StoryMetric> {
    if model.stories.is_empty() {
        return Vec::new();
    }
    let d = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };

    // 基部レベル: 全節点の最低標高
    let base_z = model
        .nodes
        .iter()
        .map(|n| n.coord[2])
        .fold(f64::INFINITY, f64::min);

    // 各階の平均水平変位
    let avg_disp: Vec<f64> = model
        .stories
        .iter()
        .map(|s| {
            let vals: Vec<f64> = s
                .node_ids
                .iter()
                .filter_map(|n| disp.get(n.index()).map(|u| u[d]))
                .collect();
            if vals.is_empty() {
                0.0
            } else {
                vals.iter().sum::<f64>() / vals.len() as f64
            }
        })
        .collect();

    let mut heights = Vec::with_capacity(model.stories.len());
    let mut drifts = Vec::with_capacity(model.stories.len());
    for (i, s) in model.stories.iter().enumerate() {
        let below_elev = if i == 0 {
            base_z
        } else {
            model.stories[i - 1].elevation
        };
        let below_disp = if i == 0 { 0.0 } else { avg_disp[i - 1] };
        heights.push((s.elevation - below_elev).max(1e-9));
        drifts.push((avg_disp[i] - below_disp).abs());
    }

    let rs_all = stiffness_ratios(&heights, &drifts);

    model
        .stories
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let ecc = story_eccentricity(model, s.id);
            let (e_dist, radius) = match dir {
                SeismicDir::X => (ecc.ey, ecc.rex),
                SeismicDir::Y => (ecc.ex, ecc.rey),
            };
            let re = eccentricity_ratio(e_dist, radius);
            let rs = rs_all.get(i).copied().unwrap_or(1.0);
            let angle = drifts[i] / heights[i];
            StoryMetric {
                name: s.name.clone(),
                height: heights[i],
                drift: drifts[i],
                drift_angle: angle,
                drift_ok: angle <= 1.0 / 200.0,
                rs,
                rs_ok: rs >= 0.6,
                re,
                re_ok: re <= 0.15,
                fes: fes(rs, re),
            }
        })
        .collect()
}

/// 解析・検定結果を CSV テキストにまとめる（レポートタブの出力）。
pub fn build_report_csv(app: &App) -> String {
    let mut out = String::new();
    let model = &app.model;

    out.push_str("# Squid-N レポート\n");
    out.push_str("\n[モデル概要]\n");
    out.push_str(&format!(
        "節点数,{}\n部材数,{}\n断面数,{}\n材料数,{}\n荷重ケース数,{}\n階数,{}\n",
        model.nodes.len(),
        model.elements.len(),
        model.sections.len(),
        model.materials.len(),
        model.load_cases.len(),
        model.stories.len()
    ));

    if !model.stories.is_empty() {
        out.push_str("\n[階]\n階,標高[mm],地震重量[kN]\n");
        for s in &model.stories {
            out.push_str(&format!(
                "{},{:.0},{:.2}\n",
                s.name,
                s.elevation,
                s.seismic_weight.unwrap_or(0.0) / 1000.0
            ));
        }
    }

    let Some(results) = &app.results else {
        out.push_str("\n(解析結果なし)\n");
        return out;
    };

    if let Some(modal) = &results.modal {
        out.push_str("\n[固有値解析]\n次数,周期[s],有効質量比X,有効質量比Y\n");
        for (i, t) in modal.period.iter().enumerate() {
            let em = modal.effective_mass.get(i).copied().unwrap_or([0.0; 3]);
            out.push_str(&format!("{},{:.4},{:.3},{:.3}\n", i + 1, t, em[0], em[1]));
        }
    }

    for (key, st) in &results.statics {
        // ユーザー荷重ケースは「LC {id} {名前}」、地震静的は方向名でラベル付けする
        // （StaticCaseKey により両者は別キーで共存するため、ラベルも区別できる）。
        let label = match key {
            StaticCaseKey::User(lc_id) => model
                .load_cases
                .iter()
                .find(|c| c.id == *lc_id)
                .map(|c| format!("LC {} {}", lc_id.0, c.name))
                .unwrap_or_else(|| format!("LC {}", lc_id.0)),
            StaticCaseKey::Seismic(SeismicDir::X) => "地震静的 X".to_string(),
            StaticCaseKey::Seismic(SeismicDir::Y) => "地震静的 Y".to_string(),
            StaticCaseKey::Wind(SeismicDir::X) => "風静的 X".to_string(),
            StaticCaseKey::Wind(SeismicDir::Y) => "風静的 Y".to_string(),
        };
        let max_d = st
            .disp
            .iter()
            .flat_map(|u| u[..3].iter())
            .fold(0.0f64, |m, v| m.max(v.abs()));
        out.push_str(&format!(
            "\n[静的解析: {}]\n最大変位[mm],{:.4}\n",
            label, max_d
        ));
    }

    // 層指標（最後に実行した静的結果に基づく）
    if let Some((_, st)) = results.statics.last() {
        let metrics = compute_story_metrics(model, &st.disp, app.analysis_cfg.seismic_dir);
        if !metrics.is_empty() {
            out.push_str(
                "\n[層指標(二次設計)]\n階,階高[mm],層間変位[mm],層間変形角,1/200判定,剛性率Rs,Rs判定,偏心率Re,Re判定,Fes\n",
            );
            for m in &metrics {
                out.push_str(&format!(
                    "{},{:.0},{:.3},1/{:.0},{},{:.3},{},{:.3},{},{:.3}\n",
                    m.name,
                    m.height,
                    m.drift,
                    if m.drift_angle > 0.0 {
                        1.0 / m.drift_angle
                    } else {
                        f64::INFINITY
                    },
                    if m.drift_ok { "OK" } else { "NG" },
                    m.rs,
                    if m.rs_ok { "OK" } else { "NG" },
                    m.re,
                    if m.re_ok { "OK" } else { "NG" },
                    m.fes
                ));
            }
        }
    }

    if !results.checks.is_empty() {
        out.push_str("\n[部材検定]\n部材,位置,検定比,判定,根拠\n");
        for (elem_id, pos, cr) in &results.checks {
            out.push_str(&format!(
                "{},{:.3},{:.4},{},{}\n",
                elem_id.0,
                pos,
                cr.ratio,
                if cr.ok { "OK" } else { "NG" },
                cr.basis.replace(',', ";")
            ));
        }
    }

    if let Some(po) = &results.pushover {
        out.push_str(&format!(
            "\n[プッシュオーバー]\n保有水平耐力Qu[kN],{:.2}\nヒンジ数,{}\n",
            po.qu / 1000.0,
            po.hinges.len()
        ));
        out.push_str("step,頂部変位[mm],ベースシア[kN]\n");
        for p in &po.capacity_curve {
            out.push_str(&format!(
                "{},{:.3},{:.2}\n",
                p.step,
                p.roof_disp,
                p.base_shear / 1000.0
            ));
        }
    }

    if let Some(th) = &results.time_history {
        let peak = th
            .history
            .node_disp
            .iter()
            .fold(0.0f64, |m, v| m.max(v.abs()));
        out.push_str(&format!(
            "\n[時刻歴応答]\nステップ数,{}\n記録節点最大変位[mm],{:.4}\n",
            th.time.len(),
            peak
        ));
    }

    out
}

/// ResultsBundle が空でないか（レポートに載せる内容があるか）。
pub fn has_report_content(results: &Option<ResultsBundle>) -> bool {
    results
        .as_ref()
        .map(|r| {
            !r.statics.is_empty()
                || r.modal.is_some()
                || r.pushover.is_some()
                || r.time_history.is_some()
        })
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_report_and_metrics_from_sample_flow() {
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        // 層指標
        let st = &app.results.as_ref().unwrap().statics.last().unwrap().1;
        let metrics = compute_story_metrics(&app.model, &st.disp, SeismicDir::X);
        assert_eq!(metrics.len(), 1);
        assert!(metrics[0].drift > 0.0);
        assert!(metrics[0].rs > 0.0);

        // レポート
        let csv = build_report_csv(&app);
        assert!(csv.contains("[モデル概要]"));
        assert!(csv.contains("[層指標(二次設計)]"));
        assert!(csv.contains("[部材検定]"));
    }

    #[test]
    fn test_report_without_results() {
        let app = App::default();
        let csv = build_report_csv(&app);
        assert!(csv.contains("解析結果なし"));
        assert!(!has_report_content(&app.results));
    }
}
