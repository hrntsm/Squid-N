//! 層指標（二次設計チェック）とレポート文字列の生成。GUI 非依存。

use squid_n_core::model::Model;
use squid_n_design_jp::secondary::eccentricity::story_eccentricity;
use squid_n_design_jp::secondary::eccentricity_analysis::story_eccentricity_from_analysis;
use squid_n_design_jp::secondary::holding_capacity::{eccentricity_ratio, fes, stiffness_ratios};
use squid_n_design_jp::secondary::stiffness_ratio::{cog_story_drifts, max_column_drift};
use squid_n_solver::analysis::SeismicDir;
use squid_n_solver::linear::StaticOnce;

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
    /// 層間変形角の制限値の分母（令82条の2。原則 200、緩和時 120。
    /// `Model::stress_cfg.drift_limit_denom`）
    pub drift_limit_denom: f64,
    /// 1/drift_limit_denom 以下か（令82条の2）
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

/// 層指標算定の追加入力（偏心率の精算・重心の長期軸力算定用）。
/// 無い項目は `None` のままでよく、その場合は略算（D値法・質量重心）へ
/// フォールバックする。
#[derive(Default, Clone, Copy)]
pub struct StoryMetricsCtx<'a> {
    /// X 方向加力の弾性応力解析結果（剛心の精算 ki=Qi/δi 用）
    pub seismic_x: Option<&'a StaticOnce>,
    /// Y 方向加力の弾性応力解析結果（同上）
    pub seismic_y: Option<&'a StaticOnce>,
    /// 長期応力解析結果（重心の長期軸力算定用）
    pub long_term: Option<&'a StaticOnce>,
}

/// 解析結果一式から `StoryMetricsCtx` を組み立てる。
/// 長期は「短期でない荷重組合せ」を優先し、無ければ None。
pub fn metrics_ctx_from_results(results: Option<&ResultsBundle>) -> StoryMetricsCtx<'_> {
    let Some(r) = results else {
        return StoryMetricsCtx::default();
    };
    let find_seismic = |dir: SeismicDir| {
        r.statics
            .iter()
            .find(|(k, _)| *k == StaticCaseKey::Seismic(dir))
            .map(|(_, s)| s)
    };
    let long_term = r
        .combos
        .iter()
        .find(|(name, _)| !squid_n_load::combo::is_short_term_combo(name))
        .map(|(_, s)| s);
    StoryMetricsCtx {
        seismic_x: find_seismic(SeismicDir::X),
        seismic_y: find_seismic(SeismicDir::Y),
        long_term,
    }
}

/// 静的解析の変位から層指標を計算する（略算フォールバック版）。
/// `disp` は節点変位（`model.nodes` と同順）。階が未定義なら空を返す。
pub fn compute_story_metrics(
    model: &Model,
    disp: &[[f64; 6]],
    dir: SeismicDir,
) -> Vec<StoryMetric> {
    compute_story_metrics_with(model, disp, dir, &StoryMetricsCtx::default())
}

/// 静的解析の変位から層指標を計算する（構造力学・弾性応力解析）。
///
/// - **層間変形角**: その階の柱の層間変形角の**最大値**（斜め柱除外。
///   `story_metrics::max_column_drift`）。柱が拾えない層は従来の
///   階平均変位差にフォールバックする。
/// - **剛性率 Rs**: 重心位置の層間変位 δg（質量重み付き平均変位の差。
///   `story_metrics::cog_story_drifts`）から `Rs = rs/r̄s`。
/// - **偏心率 Re**: `ctx` に X/Y 加力の解析結果があれば精算
///   （剛心 ki=Qi/δi・重心=長期軸力）、無ければ D値法（略算）。
pub fn compute_story_metrics_with(
    model: &Model,
    disp: &[[f64; 6]],
    dir: SeismicDir,
    ctx: &StoryMetricsCtx<'_>,
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

    // 各階の平均水平変位（柱が拾えない層の層間変位フォールバック用）
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
        heights.push((s.elevation - below_elev).max(1e-9));
        // 層間変形角の確認用変位: 柱ごとの最大値（1/irs = max(δ)/iH）
        let drift = match max_column_drift(model, disp, d, s.id) {
            Some(cd) => cd.drift,
            None => {
                let below_disp = if i == 0 { 0.0 } else { avg_disp[i - 1] };
                (avg_disp[i] - below_disp).abs()
            }
        };
        drifts.push(drift);
    }

    // 剛性率は重心位置の層間変位 δg で算定（1/irs = iδg/iH）
    let cog_drifts = cog_story_drifts(model, disp, d);
    let rs_all = stiffness_ratios(&heights, &cog_drifts);

    // 層間変形角の制限値（令82条の2。原則 1/200、緩和時 1/120）。
    let denom = if model.stress_cfg.drift_limit_denom > 0.0 {
        model.stress_cfg.drift_limit_denom
    } else {
        200.0
    };

    model
        .stories
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let ecc = match (ctx.seismic_x, ctx.seismic_y) {
                // 精算: 剛心 = 地震時応力解析結果の ki=Qi/δi、重心 = 長期軸力
                (Some(rx), Some(ry)) => {
                    story_eccentricity_from_analysis(model, s.id, rx, ry, ctx.long_term)
                }
                // 略算: D値法
                _ => story_eccentricity(model, s.id),
            };
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
                drift_limit_denom: denom,
                drift_ok: angle <= 1.0 / denom,
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

    // 数量積算（モデルのみから算定できるため解析結果の有無に関わらず出力する）。
    out.push_str(&build_quantity_csv(model));

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
        let ctx = metrics_ctx_from_results(app.results.as_ref());
        let metrics =
            compute_story_metrics_with(model, &st.disp, app.analysis_cfg.seismic_dir, &ctx);
        if !metrics.is_empty() {
            let denom = metrics
                .first()
                .map(|m| m.drift_limit_denom)
                .unwrap_or(200.0);
            out.push_str(&format!(
                "\n[層指標(二次設計)]\n階,階高[mm],層間変位[mm],層間変形角,1/{:.0}判定,剛性率Rs,Rs判定,偏心率Re,Re判定,Fes\n",
                denom
            ));
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

    // 主軸の計算（構造力学）。
    // X・Y 加力の弾性解析結果が揃っている場合のみ、水平力のなす仕事が極値をとる
    // 角度 Θ（tan2Θ = −Pᵗ(uy+vx)/Pᵗ(vy−ux)）を出力する。
    {
        let ctx = metrics_ctx_from_results(app.results.as_ref());
        if let (Some(rx), Some(ry)) = (ctx.seismic_x, ctx.seismic_y) {
            let cfg = squid_n_solver::analysis::SeismicCfg {
                dir: SeismicDir::X,
                mode: app.analysis_cfg.ai_mode,
                z: app.analysis_cfg.z,
                soil: app.analysis_cfg.soil,
                c0: app.analysis_cfg.c0,
            };
            if let Ok(analysis) = squid_n_solver::analysis::Analysis::prepare(model) {
                if let Ok(p) = analysis.seismic_nodal_force_magnitudes(cfg) {
                    let theta =
                        squid_n_design_jp::secondary::principal_axis::principal_axis_from_results(
                            model, &p, rx, ry,
                        );
                    out.push_str(&format!(
                        "\n[主軸の計算]\n主軸角Θ[deg],{:.3}\n",
                        theta.to_degrees()
                    ));
                }
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

/// 数量積算の CSV 文字列を生成する（GUI 非依存）。
///
/// 部位別の概算数量
/// （[`squid_n_design_jp::quantity::compute_quantity_takeoff`]）を、
/// 部位別・階別・鉄骨種類別・鉄筋径別・明細・注記のセクションに整形する。
pub fn build_quantity_csv(model: &Model) -> String {
    use squid_n_design_jp::quantity::{compute_quantity_takeoff, QuantityCfg};

    let q = compute_quantity_takeoff(model, &QuantityCfg::default());
    let mut out = String::new();
    if q.items.is_empty() {
        return out;
    }

    out.push_str(
        "\n[数量積算 部位別]\n部位,コンクリート[m3],型枠[m2],鉄筋[t],鉄骨[t],鉄筋継手[個所]\n",
    );
    for (cat, t) in q.totals_by_category() {
        out.push_str(&format!(
            "{},{:.2},{:.2},{:.3},{:.3},{:.1}\n",
            cat.label(),
            t.concrete_m3,
            t.formwork_m2,
            t.rebar_t,
            t.steel_t,
            t.rebar_joints
        ));
    }
    let totals = q.totals();
    out.push_str(&format!(
        "合計,{:.2},{:.2},{:.3},{:.3},{:.1}\n",
        totals.concrete_m3, totals.formwork_m2, totals.rebar_t, totals.steel_t, totals.rebar_joints
    ));

    out.push_str(
        "\n[数量積算 階別]\n階,コンクリート[m3],型枠[m2],鉄筋[t],鉄骨[t],鉄筋継手[個所]\n",
    );
    for (story, t) in q.totals_by_story() {
        out.push_str(&format!(
            "{},{:.2},{:.2},{:.3},{:.3},{:.1}\n",
            story, t.concrete_m3, t.formwork_m2, t.rebar_t, t.steel_t, t.rebar_joints
        ));
    }

    let steel = q.steel_by_section();
    if !steel.is_empty() {
        out.push_str("\n[数量積算 鉄骨種類別]\n断面,長さ[m],重量[t]\n");
        for s in steel {
            out.push_str(&format!(
                "{},{:.2},{:.3}\n",
                s.section_name, s.length_m, s.weight_t
            ));
        }
    }

    let rebar = q.rebar_by_dia();
    if !rebar.is_empty() {
        out.push_str("\n[数量積算 鉄筋径別]\n呼び径,長さ[m],重量[t]\n");
        for (dia, len, w) in rebar {
            let name = if dia > 0.0 {
                format!("D{:.0}", dia)
            } else {
                "(鉄筋比概算)".to_string()
            };
            out.push_str(&format!("{},{:.1},{:.3}\n", name, len, w));
        }
    }

    out.push_str(
        "\n[数量積算 明細]\nID,階,部位,構造,符号,コンクリート[m3],型枠[m2],鉄筋[t],鉄骨[t],鉄筋継手[個所]\n",
    );
    for it in &q.items {
        let id = it
            .elem
            .map(|e| e.0.to_string())
            .or_else(|| it.slab.map(|s| format!("S{}", s.0)))
            .unwrap_or_else(|| "-".to_string());
        out.push_str(&format!(
            "{},{},{},{},{},{:.3},{:.2},{:.4},{:.4},{:.1}\n",
            id,
            it.story,
            it.category.label(),
            it.structure.label(),
            it.label,
            it.concrete_m3,
            it.formwork_m2,
            it.rebar_weight_t(),
            it.steel_weight_t(),
            it.rebar_joints
        ));
    }

    out.push_str("\n[数量積算 注記]\n");
    for n in &q.notes {
        out.push_str(&format!("{}\n", n));
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
        // 数量積算セクションも常時含まれる。
        assert!(csv.contains("[数量積算 部位別]"));
    }

    #[test]
    fn test_quantity_csv_from_sample_model() {
        // サンプルモデル（門型ラーメン）で数量積算 CSV が生成される
        // エンドツーエンド確認。柱・大梁が分類され、合計行が出力される。
        let model = crate::sample::portal_frame();
        let csv = build_quantity_csv(&model);
        assert!(csv.contains("[数量積算 部位別]"), "{csv}");
        assert!(csv.contains("柱"), "{csv}");
        assert!(csv.contains("大梁"), "{csv}");
        assert!(csv.contains("合計"), "{csv}");
        assert!(csv.contains("[数量積算 明細]"));
        assert!(csv.contains("[数量積算 注記]"));
    }

    #[test]
    fn test_drift_limit_denom_relaxation() {
        // 令82条の2 の緩和（1/120）を計算条件で指定すると判定と表示分母が追従する。
        let mut app = App::default();
        app.load_model(crate::sample::portal_frame());
        app.generate_stories_action();
        app.model.stress_cfg.drift_limit_denom = 120.0;
        app.run_seismic(SeismicDir::X);
        assert!(app.last_error.is_none(), "{:?}", app.last_error);

        let st = &app.results.as_ref().unwrap().statics.last().unwrap().1;
        let metrics = compute_story_metrics(&app.model, &st.disp, SeismicDir::X);
        assert_eq!(metrics[0].drift_limit_denom, 120.0);
        assert_eq!(metrics[0].drift_ok, metrics[0].drift_angle <= 1.0 / 120.0);
        let csv = build_report_csv(&app);
        assert!(csv.contains("1/120判定"), "CSV ヘッダが緩和値に追従する");
    }

    #[test]
    fn test_report_without_results() {
        let app = App::default();
        let csv = build_report_csv(&app);
        assert!(csv.contains("解析結果なし"));
        assert!(!has_report_content(&app.results));
    }
}
