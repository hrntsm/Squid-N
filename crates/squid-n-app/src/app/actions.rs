//! `App` のアクション（解析実行・ファイル入出力・モデル操作）メソッド。

use super::*;

/// 節点対の順不同キー（`(min,max)`）。`beam_elem_map`（節点対→実 `Beam` 要素索引）と
/// `slab_grillage_node_reactions`（実部材化判定）で、ノード順に依存しない同じキーを
/// 使うための共通ヘルパー。
fn beam_key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
    if a.0 <= b.0 {
        (a, b)
    } else {
        (b, a)
    }
}

impl App {
    /// モデルを丸ごと差し替える（新規作成・サンプル読込・ファイル読込で共用）。
    /// undo 履歴・結果・選択・stale 状態をすべてリセットする。
    /// 旧スキーマの自動生成荷重ケース名（「床荷重(自動)」「自重(自動)」等）は
    /// 標準ケース名（DL・LL(架構用)・LL(地震用)）へ移行する。
    pub fn load_model(&mut self, mut model: squid_n_core::model::Model) {
        model.migrate_legacy_auto_load_cases();
        self.model = model;
        self.results = None;
        self.selection = Selection::default();
        self.undo = UndoStack::new();
        self.nav = Navigator::default();
        self.last_static = None;
        self.last_error = None;
        self.last_notice = None;
        self.auto_load_sync_hash = None;
        self.staleness = Staleness::default();
        self.sync_node_edit();
        self.refresh_beam_loads();
    }

    /// プロジェクトを指定パスへ保存する。成功時は project_path と未保存フラグを更新。
    pub fn save_project_to(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::scz::save_scz(&path, &self.model) {
            Ok(()) => {
                self.project_path = Some(path);
                self.staleness.unsaved_changes = false;
            }
            Err(e) => self.last_error = Some(format!("保存エラー: {}", e)),
        }
    }

    /// プロジェクトを指定パスから読み込む。成功時はモデルを差し替える。
    pub fn open_project_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::scz::load_scz(&path) {
            Ok(model) => {
                if let Err(e) = model.validate() {
                    self.last_error = Some(format!("読込モデルの検証エラー: {:?}", e));
                    return;
                }
                self.load_model(model);
                self.project_path = Some(path);
            }
            Err(e) => self.last_error = Some(format!("読込エラー: {}", e)),
        }
    }

    /// ST-Bridge（XML, サブセット）ファイルを読み込む。
    /// Squid-N プロジェクト（.scz）とは別物なので project_path はクリアする。
    /// ファイルが荷重情報（`StbLoadCase`）を持たない場合は、標準荷重ケース
    /// （DL・LL(架構用)・LL(地震用)・EX・EY）を自動作成する（新規モデルと同じ
    /// 出発点。DL の自重・スラブ荷重は解析実行前の同期アクションが自動計算する）。
    pub fn import_stbridge_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        let xml = match squid_n_io::stbridge::read_stbridge_file(&path) {
            Ok(s) => s,
            Err(e) => {
                self.last_error = Some(format!("ST-Bridge読込エラー: {}", e));
                return;
            }
        };
        match squid_n_io::stbridge::import_stbridge_with_report(&xml) {
            Ok((mut model, report)) => {
                if let Err(e) = model.validate() {
                    self.last_error = Some(format!("ST-Bridge読込モデルの検証エラー: {:?}", e));
                    return;
                }
                if model.load_cases.is_empty() {
                    model.load_cases = squid_n_core::model::default_load_cases();
                }
                self.load_model(model);
                self.project_path = None;
                // 欠落・近似（warnings）と自動補完の仮定（notes。支点の自動設定など）が
                // あれば注意として表示する（致命的ではない）。
                let lines: Vec<&str> = report
                    .warnings
                    .iter()
                    .chain(report.notes.iter())
                    .map(String::as_str)
                    .collect();
                if !lines.is_empty() {
                    self.last_error = Some(format!(
                        "⚠️ ST-Bridge 取り込み時の注意:\n- {}",
                        lines.join("\n- ")
                    ));
                }
            }
            Err(e) => self.last_error = Some(format!("ST-Bridge読込エラー: {}", e)),
        }
    }

    /// モデルを標準 ST-Bridge 2.0.2（XML, 幾何サブセット）として指定パスへ書き出す。
    pub fn export_stbridge_to(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::stbridge::export_stbridge(&self.model) {
            Ok(xml) => {
                if let Err(e) = std::fs::write(&path, xml) {
                    self.last_error = Some(format!("ST-Bridge書出エラー: {}", e));
                }
            }
            Err(e) => self.last_error = Some(format!("ST-Bridge書出エラー: {}", e)),
        }
    }

    /// 節点編集バッファを model.nodes に同期する。
    /// 編集中でない（フォーカス外）セルのみ model 値で更新する。
    pub fn sync_node_edit(&mut self) {
        self.node_edit.resize(
            self.model.nodes.len(),
            ["0".to_string(), "0".to_string(), "0".to_string()],
        );
        for (i, node) in self.model.nodes.iter().enumerate() {
            for (k, slot) in self.node_edit[i].iter_mut().enumerate().take(3) {
                *slot = format!("{:.3}", node.coord[k]);
            }
        }
    }

    /// 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1「剛域」は
    /// 標準実装。解析前に1回適用する）。`squid_n_element::beam::apply_auto_rigid_zones`
    /// は `ZoneSource::Auto` の端のみ更新し `Manual` 端を保護するため、
    /// 各解析エントリの先頭で毎回呼んでも冪等で安全。
    fn apply_rigid_zones_for_analysis(&mut self) {
        squid_n_element::beam::apply_auto_rigid_zones(
            &mut self.model,
            &squid_n_element::beam::RigidZoneRule::default(),
        );
    }

    /// `analysis_cfg.threads` を並列度設定（プロセスグローバル）へ反映する。
    /// 各解析エントリの先頭で呼ぶ（バックグラウンドジョブは thread::spawn 前に
    /// 呼べばよい。設定はプロセスグローバルのためジョブ側での再設定は不要）。
    fn apply_parallelism_setting(&self) {
        squid_n_math::parallelism::set_parallelism(
            squid_n_math::parallelism::Parallelism::from_threads(self.analysis_cfg.threads),
        );
    }

    /// T3: 線形静的解析を実行し、結果を `self.results` に格納する。
    /// 指定した荷重ケースが存在しない場合はエラーメッセージをセット。
    ///
    /// 解析準備前にスラブ荷重・躯体自重を「DL」等の標準ケースへ（レビュー §1.1・
    /// 照合レビュー：③梁自重・②壁荷重の CMoQ 経路を長期応力解析へ接続）、
    /// 階が定義済みなら地震荷重を「EX」「EY」ケースへ同期する
    /// （`sync_auto_load_cases_action`。モデル・関連設定が前回同期時から
    /// 変わっていなければ丸ごとスキップする）。
    pub fn run_linear_static(&mut self, lc: LoadCaseId) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let res = Self::compute_linear_static(self.model.clone(), lc);
        self.apply_static_case_result(StaticCaseKey::User(lc), res);
    }

    /// 線形静的解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_linear_static_job`）からも呼び出せる。
    /// 剛域は呼び出し側（`sync_auto_load_cases_action`）で適用済みのモデルを
    /// 渡す前提のため、ここでは再適用しない（二重適用を避ける）。
    fn compute_linear_static(
        model: squid_n_core::model::Model,
        lc: LoadCaseId,
    ) -> Result<squid_n_solver::linear::StaticOnce, String> {
        match Analysis::prepare(&model) {
            Ok(analysis) => analysis
                .linear_static(lc)
                .map_err(|e| format!("線形静的解析エラー: {:?}", e)),
            Err(e) => Err(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// `compute_linear_static`/`compute_seismic`/`compute_wind` に共通の結果適用
    /// （`StaticCaseKey` で区別される単一荷重ケースの静的解析結果）。
    /// bundle への格納・last_static 設定・staleness.mark_fresh・design_check の
    /// 実行はいずれも `run_linear_static`/`run_seismic`/`run_wind` で同一のため、
    /// ここへ集約し同期版・バックグラウンドジョブ双方から使う。
    fn apply_static_case_result(
        &mut self,
        key: StaticCaseKey,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    ) {
        match res {
            Ok(res) => {
                let member_forces = res.member_forces.clone();
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.statics.retain(|(id, _)| *id != key);
                bundle.statics.push((key, res));
                bundle.member_forces = member_forces;
                self.results = Some(bundle);
                self.last_static = Some(StaticKey::Case(key));
                self.staleness.mark_fresh();
                self.run_design_check();
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// 線形静的解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_linear_static_job(&mut self, lc: LoadCaseId) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let model = self.model.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_linear_static(model, lc)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::StaticCase {
                key: StaticCaseKey::User(lc),
                res,
            });
        });
        self.job = Some(AnalysisJob {
            label: "線形静的解析",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
    }

    /// T7: 荷重組合せ解析を実行し、結果を `bundle.combos` に格納する。
    /// 指定インデックスの荷重組合せが存在しない場合はエラーメッセージをセット。
    ///
    /// 解析準備前にスラブ荷重・躯体自重を「DL」等の標準ケースへ、階が定義済み
    /// なら地震荷重を「EX」「EY」ケースへ同期する（レビュー §1.1・照合レビュー、
    /// `sync_auto_load_cases_action`。モデル・関連設定が前回同期時から変わって
    /// いなければ丸ごとスキップする）。
    /// 組合せが空の地震荷重ケースを参照している場合は解かずにエラーで案内する
    /// （地震項が黙って 0 になるのを防ぐ）。
    pub fn run_combination(&mut self, index: usize) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.last_error = Some(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        if let Some(name) = self.empty_seismic_case_in_combo(&combo) {
            self.last_error = Some(format!(
                "荷重組合せ「{}」が参照する地震荷重ケース「{}」が空です。解析タブの「階の自動生成」を実行して地震荷重を生成してください。",
                combo.name, name
            ));
            return;
        }
        let name = combo.name.clone();
        let res = Self::compute_combination(self.model.clone(), combo);
        self.apply_combo_result(name, res);
    }

    /// 荷重組合せ解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_combination_job`）からも呼び出せる。
    fn compute_combination(
        model: squid_n_core::model::Model,
        combo: squid_n_core::model::LoadCombination,
    ) -> Result<squid_n_solver::linear::StaticOnce, String> {
        match Analysis::prepare(&model) {
            Ok(analysis) => analysis
                .linear_combination(&combo)
                .map_err(|e| format!("荷重組合せ解析エラー: {:?}", e)),
            Err(e) => Err(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// `compute_combination` の結果を適用する（bundle.combos への格納・
    /// last_static 設定・design_term 自動判定・design_check の実行）。
    /// `name` は組合せ名（`bundle.combos` 内の名前一致検索・再実行時の位置差替に
    /// 使う。`run_combination`/`start_combination_job` 双方から使う）。
    fn apply_combo_result(
        &mut self,
        name: String,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    ) {
        match res {
            Ok(res) => {
                let member_forces = res.member_forces.clone();
                let mut bundle = self.results.take().unwrap_or_default();
                // StaticKey::Combo は bundle.combos 上の位置を指す規約
                // （current_static・ナビゲータと共有）。再実行時は既存位置を
                // その場で差し替え、他の組合せ結果のキーを無効化しない。
                let pos = match bundle.combos.iter().position(|(n, _)| *n == name) {
                    Some(pos) => {
                        bundle.combos[pos].1 = res;
                        pos
                    }
                    None => {
                        bundle.combos.push((name.clone(), res));
                        bundle.combos.len() - 1
                    }
                };
                bundle.member_forces = member_forces;
                self.results = Some(bundle);
                self.last_static = Some(StaticKey::Combo(pos));
                self.staleness.mark_fresh();
                // 荷重継続性区分（長期/短期）は組合せ内容から自動判定する
                // （令82条の荷重組合せ: G+P=長期、地震・積雪・風入り=短期）。
                self.design_term = if squid_n_load::combo::is_short_term_combo(&name) {
                    LoadTerm::Short
                } else {
                    LoadTerm::Long
                };
                self.run_design_check();
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// 荷重組合せ解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_combination_job(&mut self, index: usize) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.last_error = Some(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        if let Some(name) = self.empty_seismic_case_in_combo(&combo) {
            self.last_error = Some(format!(
                "荷重組合せ「{}」が参照する地震荷重ケース「{}」が空です。解析タブの「階の自動生成」を実行して地震荷重を生成してください。",
                combo.name, name
            ));
            return;
        }
        let model = self.model.clone();
        let name = combo.name.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_combination(model, combo)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::Combo { name, res });
        });
        self.job = Some(AnalysisJob {
            label: "荷重組合せ解析",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
    }

    /// 全荷重組合せを一括解析し、結果を `bundle.combos` へ格納する
    /// （`run_combination` の全件版。`Analysis::prepare` を 1 回だけ行い、
    /// `analysis_cfg.threads` の並列設定に応じて
    /// `Analysis::linear_combination_batch` で組合せ単位に並列解析する）。
    ///
    /// 個別組合せの解析エラーは処理を止めず、件数と最初のエラー内容を
    /// `last_error` にまとめる（他の組合せの結果は失わない）。荷重組合せが
    /// 1 件も無い場合、および 1 件も解けなかった場合は既存の結果を変更せず、
    /// 案内メッセージを `last_error` に設定して return する。
    pub fn run_all_combinations(&mut self) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        if self.model.combinations.is_empty() {
            self.last_error =
                Some("荷重組合せがありません。荷重タブで作成してください。".to_string());
            return;
        }
        // 解析準備前にスラブ荷重・躯体自重を「DL」等の標準ケースへ、階が定義済み
        // なら地震荷重を「EX」「EY」ケースへ同期する（`sync_auto_load_cases_action`。
        // モデル・関連設定が前回同期時から変わっていなければ丸ごとスキップする）。
        self.sync_auto_load_cases_action();
        // 空の地震荷重ケース（未生成の EX/EY 等）を参照する組合せは解かずに
        // エラーへ回す（地震項が黙って 0 になるのを防ぐ）。UI スレッド側の
        // `self.model` を参照するためバックグラウンドジョブでもここで行う。
        let (combos, errors) = self.filter_combos_for_all_combinations();
        let computed = Self::compute_all_combinations(self.model.clone(), combos);
        self.apply_all_combinations_result(computed, errors);
    }

    /// `run_all_combinations`/`start_all_combinations_job` 共通の事前フィルタ。
    /// 空の地震荷重ケースを参照する組合せを除外し、(解析対象の組合せ, エラー文一覧)
    /// を返す。
    fn filter_combos_for_all_combinations(
        &self,
    ) -> (Vec<squid_n_core::model::LoadCombination>, Vec<String>) {
        let mut errors: Vec<String> = Vec::new();
        let combos = self
            .model
            .combinations
            .iter()
            .filter(|combo| match self.empty_seismic_case_in_combo(combo) {
                Some(name) => {
                    errors.push(format!(
                        "[{}] 地震荷重ケース「{}」が空です。解析タブの「階の自動生成」を実行してください。",
                        combo.name, name
                    ));
                    false
                }
                None => true,
            })
            .cloned()
            .collect();
        (combos, errors)
    }

    /// 全荷重組合せ一括解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_all_combinations_job`）からも呼び出せる。
    /// `Analysis::prepare` を 1 回だけ行い、`analysis_cfg.threads` の並列設定に
    /// 応じて `Analysis::linear_combination_batch` で組合せ単位に並列解析する。
    /// `Analysis::prepare` 自体が失敗した場合は `Err` で全体を中断する
    /// （個別組合せへは処理を進めない。既存結果は `apply_all_combinations_result`
    /// 側で変更しない）。
    #[allow(clippy::type_complexity)]
    fn compute_all_combinations(
        model: squid_n_core::model::Model,
        combos: Vec<squid_n_core::model::LoadCombination>,
    ) -> Result<Vec<(String, Result<squid_n_solver::linear::StaticOnce, String>)>, String> {
        let analysis = Analysis::prepare(&model).map_err(|e| format!("解析準備エラー: {:?}", e))?;
        let results = analysis.linear_combination_batch(&combos);
        Ok(combos
            .iter()
            .zip(results)
            .map(|(combo, res)| {
                (
                    combo.name.clone(),
                    res.map_err(|e| format!("[{}] {:?}", combo.name, e)),
                )
            })
            .collect())
    }

    /// `compute_all_combinations` の結果を適用する。個別組合せの解析エラーは
    /// 処理を止めず、件数と最初のエラー内容を `last_error` にまとめる
    /// （他の組合せの結果は失わない）。`pre_errors`（事前フィルタで除外された
    /// 組合せのエラー）と合わせて1件も解けなかった場合、および
    /// `Analysis::prepare` 自体が失敗した場合は既存の結果を変更せず、案内
    /// メッセージを `last_error` に設定して return する。
    #[allow(clippy::type_complexity)]
    fn apply_all_combinations_result(
        &mut self,
        computed: Result<Vec<(String, Result<squid_n_solver::linear::StaticOnce, String>)>, String>,
        mut errors: Vec<String>,
    ) {
        let items = match computed {
            Ok(items) => items,
            Err(e) => {
                self.last_error = Some(e);
                return;
            }
        };

        let mut bundle = self.results.take().unwrap_or_default();
        let mut last_ok: Option<(usize, String)> = None;
        for (name, res) in items {
            match res {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    // StaticKey::Combo は bundle.combos 上の位置を指す規約
                    // （run_combination と同じ「名前一致なら置換、なければ push」）。
                    let pos = match bundle.combos.iter().position(|(n, _)| *n == name) {
                        Some(pos) => {
                            bundle.combos[pos].1 = res;
                            pos
                        }
                        None => {
                            bundle.combos.push((name.clone(), res));
                            bundle.combos.len() - 1
                        }
                    };
                    bundle.member_forces = member_forces;
                    last_ok = Some((pos, name));
                }
                Err(e) => errors.push(e),
            }
        }

        let Some((pos, last_name)) = last_ok else {
            // 1件も解けなかった場合は結果を壊さない。
            self.last_error = Some(format!(
                "全組合せ解析エラー（{} 件すべて失敗）: {}",
                errors.len(),
                errors.first().cloned().unwrap_or_default()
            ));
            return;
        };
        self.results = Some(bundle);
        self.last_static = Some(StaticKey::Combo(pos));
        self.staleness.mark_fresh();
        // 荷重継続性区分（長期/短期）は最後に成功した組合せの内容から自動判定する
        // （令82条の荷重組合せ: G+P=長期、地震・積雪・風入り=短期）。
        self.design_term = if squid_n_load::combo::is_short_term_combo(&last_name) {
            LoadTerm::Short
        } else {
            LoadTerm::Long
        };
        self.run_design_check();

        if !errors.is_empty() {
            self.last_error = Some(format!(
                "{} 件の組合せでエラー: {}",
                errors.len(),
                errors[0]
            ));
        }
    }

    /// 全荷重組合せ一括解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_all_combinations_job(&mut self) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        if self.model.combinations.is_empty() {
            self.last_error =
                Some("荷重組合せがありません。荷重タブで作成してください。".to_string());
            return;
        }
        self.sync_auto_load_cases_action();
        let (combos, pre_errors) = self.filter_combos_for_all_combinations();
        let model = self.model.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let computed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_all_combinations(model, combos)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::AllCombos {
                computed,
                pre_errors,
            });
        });
        self.job = Some(AnalysisJob {
            label: "全組合せ一括解析",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
    }

    /// 表示対象の静的解析結果を解決する。優先順: ナビゲータ選択 → 最後に実行した結果。
    pub fn current_static(&self) -> Option<&squid_n_solver::linear::StaticOnce> {
        let bundle = self.results.as_ref()?;
        let resolve = |key: StaticKey| -> Option<&squid_n_solver::linear::StaticOnce> {
            match key {
                StaticKey::Case(case_key) => bundle
                    .statics
                    .iter()
                    .find(|(k, _)| *k == case_key)
                    .map(|(_, s)| s),
                StaticKey::Combo(idx) => bundle.combos.get(idx).map(|(_, s)| s),
            }
        };
        self.nav
            .focus_result
            .and_then(resolve)
            .or_else(|| self.last_static.and_then(resolve))
    }

    /// 保有水平耐力の層別判定を行う。前提データが不足していれば Err(案内文)。
    ///
    /// 戻り値の第 2 要素は層ごとに採用された部材ランク（`design_rank_auto` が
    /// true の場合は幅厚比からの自動判定、算定できなかった層は `design_rank`
    /// へフォールバック。false の場合は全層 `design_rank`）。
    #[allow(clippy::type_complexity)]
    pub fn compute_holding_capacity(
        &mut self,
    ) -> Result<
        (
            squid_n_design_jp::secondary::holding_capacity::HoldingCapacityResult,
            Vec<squid_n_design_jp::secondary::holding_capacity::MemberRank>,
        ),
        String,
    > {
        use squid_n_core::section_shape::SectionShape;
        use squid_n_design_jp::secondary::holding_capacity::{
            check_holding_capacity, ds_value, qud_by_story, MemberRank,
        };
        use squid_n_design_jp::secondary::member_rank::{
            rc_member_rank, s_member_rank_scaled, worst_rank, RankCriteria,
        };
        use squid_n_design_jp::secondary::rc_capacity::{rc_qmu_simple, rc_qsu_simple};
        use squid_n_design_jp::secondary::width_thickness::max_width_thickness;
        use squid_n_design_jp::steel_f_value_prefix;

        // rigid_zone（剛域長・face_i/j）を読むため、算定前に自動剛域を反映する
        // （設計書 §6.2.1、冪等なので他の解析エントリと重複して呼んでも安全）。
        self.apply_rigid_zones_for_analysis();

        if self.model.stories.is_empty() {
            return Err(
                "階が未定義です。解析タブの「階の自動生成」を実行してください。".to_string(),
            );
        }
        let po = self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .ok_or_else(|| {
                "プッシュオーバー未実行です。解析タブからプッシュオーバーを実行してください。"
                    .to_string()
            })?;
        let st = self.current_static().ok_or_else(|| {
            "静的解析結果がありません。地震静的(Ai)を実行してください。".to_string()
        })?;

        let ctx = crate::summary::metrics_ctx_from_results(self.results.as_ref());
        let metrics = crate::summary::compute_story_metrics_with(
            &self.model,
            &st.disp,
            self.analysis_cfg.seismic_dir,
            &ctx,
        );

        // 地震重量: 下階→上階順（model.stories は生成時から下階→上階順に格納される）。
        let weights: Vec<f64> = self
            .model
            .stories
            .iter()
            .map(|s| s.seismic_weight.unwrap_or(0.0))
            .collect();
        if weights.iter().any(|w| *w <= 0.0) {
            return Err(
                "地震重量が未設定です。解析タブの「階の自動生成」を実行してください。".to_string(),
            );
        }

        // T(1 次周期): 固有値解析があればそれを使用、なければ略算式
        // T = h(0.02+0.01α)。h は建築物の高さ（GL〜PH 階を除く最上階）、
        // α は鉄骨造比（令88条・告示1793号。従来は α=0.0 固定・h は生の
        // 最上階 Z 標高で、S 造モデルや地下階付きモデルの T を誤っていた）。
        let t = self
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.period.first().copied())
            .unwrap_or_else(|| {
                let height_m = squid_n_solver::analysis::building_height_mm(&self.model) / 1000.0;
                let steel_ratio = squid_n_solver::analysis::steel_height_ratio(&self.model);
                squid_n_load::ai::approx_t(height_m, steel_ratio)
            });
        let rt = squid_n_load::ai::rt(t, squid_n_load::ai::tc_of(self.analysis_cfg.soil));
        let qud = qud_by_story(&weights, self.analysis_cfg.z, rt, t);

        let n_stories = weights.len();
        let (story_ranks, member_ranks): (Vec<MemberRank>, Vec<(ElemId, MemberRank)>) =
            if self.design_rank_auto {
                // 鋼部材は幅厚比、RC 矩形部材はせん断余裕度 Qsu/Qmu の略算から
                // ランクを算定し、所属階ごとに集計する。
                //
                // 所属階の規則: 部材の節点のうち最も高い階(story index 最大)。
                // story_gen::generate_stories は各節点をその節点自身の標高が属する
                // レベルへ割り当てる（柱下端は下階または基部=None、柱上端は上階、
                // 梁は両端とも同一階）ため、柱は自動的に上端側の階（＝各節点の
                // story のうち最大値）に算入される。
                let mut per_story: Vec<Vec<MemberRank>> = vec![Vec::new(); n_stories];
                let mut computed: Vec<(ElemId, MemberRank)> = Vec::new();
                // 長期軸力の簡易近似として使う荷重ケースの id
                // （`generate_stories_action` の gravity_lcs と同じ規則。§1.7:
                // kind による選択の先頭を採用。従来の「先頭ケース」規則は
                // 種別が未設定のモデルに対する後方互換フォールバックとして残る）。
                let gravity_lc = gravity_cases_for_seismic_weight(&self.model)
                    .first()
                    .copied();
                for elem in &self.model.elements {
                    let Some(sec) = elem
                        .section
                        .and_then(|sid| self.model.sections.get(sid.index()))
                    else {
                        continue;
                    };
                    let Some(mat) = elem
                        .material
                        .and_then(|mid| self.model.materials.get(mid.index()))
                    else {
                        continue;
                    };
                    let rank = if is_steel(&mat.name) {
                        // 鋼部材: 形状情報がない断面(カタログ数値直入力等)はスキップ。
                        let Some(shape) = sec.shape.as_ref() else {
                            continue;
                        };
                        // 構造規定の幅厚比表（部材種別×断面×部位×鋼種級）で判定
                        // （鋼構造設計規準「幅厚比の検討」）。
                        // 表の対象外形状（溝形・T形・山形等）は旧・単一幅厚比法へ
                        // フォールバックする。
                        let member_use = match member_kind_of(elem, &self.model) {
                        squid_n_design_jp::MemberKind::Column => {
                            squid_n_design_jp::secondary::width_thickness::SteelMemberUse::Column
                        }
                        _ => squid_n_design_jp::secondary::width_thickness::SteelMemberUse::Beam,
                    };
                        match squid_n_design_jp::secondary::width_thickness::s_member_rank_by_kihon(
                            shape, member_use, &mat.name,
                        ) {
                            Some(rank) => rank,
                            None => {
                                let Some(wt) = max_width_thickness(shape) else {
                                    continue;
                                };
                                // F 値は材料名の前方一致で引く(例 "SN400B"→235)。
                                // 引けなければ 235。板厚は形状の最大板厚。
                                let f_value =
                                    steel_f_value_prefix(&mat.name, steel_max_thickness(shape))
                                        .unwrap_or(235.0);
                                s_member_rank_scaled(wt, f_value, &RankCriteria::default())
                            }
                        }
                    } else {
                        // RC 部材: RcRect のみ対応。RcCircle・形状未設定・
                        // コンクリート強度(fc)未設定の材料はスキップ(選択値へフォールバック)。
                        let Some(SectionShape::RcRect { b, d, rebar }) = sec.shape.as_ref() else {
                            continue;
                        };
                        // 内法スパン = 幾何長 − 両端フェイス距離(直交材せい/2)。
                        // 剛域長(D_orth/2 − D_self/4)を引いた可撓長さとは別物
                        // （設計書 §6.2.1）。フェイス距離の合計が幾何長以上になる
                        // (不整合な入力)場合は下限0を割り込むため、幾何長のままとする。
                        let geom_len = elem_geometric_length(elem, &self.model);
                        let face_sum = elem.rigid_zone.face_i + elem.rigid_zone.face_j;
                        let clear_span = if geom_len - face_sum > 0.0 {
                            geom_len - face_sum
                        } else {
                            geom_len
                        };
                        let Some(mut input) =
                            rc_capacity_input_from_rect(*b, *d, rebar, mat, clear_span)
                        else {
                            continue;
                        };
                        // σ0: 長期軸力の簡易近似として先頭荷重ケース(gravity_lc)の
                        // 静的解析結果を優先し、無ければ最後に実行した静的解析結果
                        // (self.results.member_forces)から当該部材の軸力を引き、
                        // 圧縮のときのみ設定する。
                        let sigma_0 = self
                            .results
                            .as_ref()
                            .map(|r| {
                                rc_sigma_0_from_gravity_or_last_static(
                                    &r.statics,
                                    &r.member_forces,
                                    gravity_lc,
                                    elem.id,
                                    *b,
                                    *d,
                                )
                            })
                            .unwrap_or(0.0);
                        input.sigma_0 = sigma_0;
                        let qmu = rc_qmu_simple(&input);
                        let qsu = rc_qsu_simple(&input);
                        rc_member_rank(qsu, qmu, &RankCriteria::default())
                    };
                    // 節点が階を持たない部材（両端とも基部）はスキップ。
                    let Some(story_idx) = elem
                        .nodes
                        .iter()
                        .filter_map(|nid| self.model.nodes.get(nid.index()))
                        .filter_map(|n| n.story)
                        .max()
                    else {
                        continue;
                    };
                    let idx = story_idx.index();
                    if idx >= n_stories {
                        continue;
                    }
                    per_story[idx].push(rank);
                    computed.push((elem.id, rank));
                }
                // 階ごとの代表ランク = 算定できた部材ランクの最悪値。
                // 1 本も算定できなかった層は手動選択ランクへフォールバック。
                let ranks: Vec<MemberRank> = per_story
                    .into_iter()
                    .map(|rs| worst_rank(&rs).unwrap_or(self.design_rank))
                    .collect();
                (ranks, computed)
            } else {
                (vec![self.design_rank; n_stories], Vec::new())
            };

        let ds_vec: Vec<f64> = story_ranks
            .iter()
            .map(|r| ds_value(self.design_frame, *r))
            .collect();
        let heights: Vec<f64> = metrics.iter().map(|m| m.height).collect();
        let rs: Vec<f64> = metrics.iter().map(|m| m.rs).collect();
        let re: Vec<f64> = metrics.iter().map(|m| m.re).collect();
        let fes: Vec<f64> = metrics.iter().map(|m| m.fes).collect();

        let result =
            check_holding_capacity(po, &qud, &ds_vec, &fes, &rs, &re, &heights, member_ranks);
        Ok((result, story_ranks))
    }

    /// 終局検定（靭性保証型耐震設計指針）: RC 矩形部材の終局せん断強度（塑性
    /// 理論式）・付着割裂耐力・軸終局耐力に対する余裕度を算定する。
    ///
    /// 柱の曲げ終局強度 Mu・軸余裕度に用いる設計軸力は、長期（G+P 相当）静的
    /// 解析結果（先頭重力ケースを優先、無ければ最後に実行した静的解析）の軸力
    /// （圧縮正）を用いる。静的解析結果が無い場合は軸力 0（安全側）で評価する。
    ///
    /// 対象 RC 矩形部材が 1 つも無い場合は `Err` を返す（UI 側で案内表示）。
    pub fn compute_ultimate_checks(
        &mut self,
    ) -> Result<Vec<squid_n_design_jp::ultimate::UltimateCheck>, String> {
        use squid_n_core::section_shape::SectionShape;

        // 剛域（face_i/j）を内法長さに反映するため自動剛域を適用（冪等）。
        self.apply_rigid_zones_for_analysis();

        let demand = self.ultimate_demand_by_elem();

        let opts = squid_n_design_jp::ultimate::UltimateShearOptions {
            rp: self.ultimate_rp.max(0.0),
            lightweight: self.ultimate_lightweight,
            upper_strength_factor: self.ultimate_upper_factor.max(0.0),
            sigma_wy: 295.0,
            include_bond: self.ultimate_include_bond,
            mu_method: if self.ultimate_mu_aci {
                squid_n_design_jp::ultimate::MuMethod::Aci
            } else {
                squid_n_design_jp::ultimate::MuMethod::AtFormula
            },
            shear_method: if self.ultimate_shear_ductility {
                squid_n_design_jp::ultimate::ShearMethod::Ductility
            } else {
                squid_n_design_jp::ultimate::ShearMethod::Plastic
            },
            biaxial_shear: self.ultimate_biaxial_shear,
            biaxial_bending: self.ultimate_biaxial_bending,
        };
        let checks =
            squid_n_design_jp::ultimate::collect_rc_ultimate_checks(&self.model, &demand, &opts);

        // RC 矩形部材が無い場合の案内。
        let has_rc_rect = self.model.elements.iter().any(|e| {
            e.section
                .and_then(|sid| self.model.sections.get(sid.index()))
                .and_then(|s| s.shape.as_ref())
                .map(|sh| matches!(sh, SectionShape::RcRect { .. }))
                .unwrap_or(false)
        });
        if checks.is_empty() {
            if has_rc_rect {
                return Err(
                    "RC 矩形部材の終局検定を算定できませんでした（コンクリート強度 Fc の設定・\
                     有効せいを確認してください）。"
                        .to_string(),
                );
            }
            return Err(
                "終局検定の対象（RcRect の RC 矩形部材）がありません。RC 断面を割り当ててください。"
                    .to_string(),
            );
        }
        Ok(checks)
    }

    /// 終局検定用の部材需要（軸力 [N]圧縮正・強軸/弱軸の設計用曲げ [N·mm]）。
    ///
    /// `ultimate_use_pushover` が真でプッシュオーバー応答（部材別応答）が得られる場合は、
    /// 終局時の部材別 Qmu（設計用せん断）・需要曲げ・軸力・Rp を直接反映する
    /// （[`Self::ultimate_demand_from_pushover`]）。それ以外は先頭重力ケース（G+P 相当）の
    /// 静的解析結果を優先し、無ければ最後に実行した静的解析結果を用いる（軸力は始端値、
    /// 曲げは部材内の最大絶対値、Qmu は両端ヒンジ 2·Mu/内法、Rp は UI 一律指定）。
    /// いずれの応答も無ければ空（＝需要 0）。
    fn ultimate_demand_by_elem(&self) -> Vec<(ElemId, squid_n_design_jp::ultimate::MemberDemand)> {
        use squid_n_design_jp::ultimate::MemberDemand;
        // プッシュオーバー応答からの直接反映（優先、指定時かつ応答があれば）。
        if self.ultimate_use_pushover {
            if let Some(demand) = self.ultimate_demand_from_pushover() {
                return demand;
            }
        }
        let gravity_lc = gravity_cases_for_seismic_weight(&self.model)
            .first()
            .copied();
        // 単純梁せん断 Q0（MK785/SPR785/SPR685 使用部材の QL=Q0 読み替え用）。
        let q0_map = gravity_lc
            .map(|lc| simple_beam_q0_by_elem(&self.model, lc))
            .unwrap_or_default();
        self.results
            .as_ref()
            .map(|r| {
                let member_forces: &[(ElemId, squid_n_element::beam::MemberForces)] = gravity_lc
                    .and_then(|lc| {
                        r.statics
                            .iter()
                            .find(|(id, _)| *id == StaticCaseKey::User(lc))
                    })
                    .map(|(_, s)| s.member_forces.as_slice())
                    .unwrap_or(r.member_forces.as_slice());
                member_forces
                    .iter()
                    .filter_map(|(id, mf)| {
                        let n_axial = mf.at.first().map(|(_, f)| f[0])?;
                        let mz = mf.at.iter().map(|(_, f)| f[5].abs()).fold(0.0, f64::max);
                        let my = mf.at.iter().map(|(_, f)| f[4].abs()).fold(0.0, f64::max);
                        // 長期せん断力 QL（余裕率の分子控除 (Qsu−QL)/Qmu 用）。
                        // このケース自体が重力（長期）ケースのため、そのまま採用する。
                        let ql = mf.at.iter().map(|(_, f)| f[1].abs()).fold(0.0, f64::max);
                        Some((
                            *id,
                            MemberDemand {
                                n_axial,
                                mz,
                                my,
                                q_long: Some(ql),
                                q_simple: q0_map.get(id).copied(),
                                ..Default::default()
                            },
                        ))
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default()
    }

    /// プッシュオーバー応答（部材別応答）から終局検定用の部材需要を組み立てる。
    ///
    /// プッシュオーバー最終ステップの部材別応答（[`squid_n_solver::pushover::PushoverMemberResponse`]）
    /// から、軸力（圧縮正）・強軸/弱軸の設計用曲げ・強軸設計用せん断・部材別 Rp を
    /// 反映する。プッシュオーバー未実行、または部材別応答が空（ステップ未確定）の場合は
    /// `None`（呼び出し側が静的応答へフォールバック）。
    fn ultimate_demand_from_pushover(
        &self,
    ) -> Option<Vec<(ElemId, squid_n_design_jp::ultimate::MemberDemand)>> {
        use squid_n_design_jp::ultimate::MemberDemand;
        let po = self.results.as_ref()?.pushover.as_ref()?;
        if po.member_response.is_empty() {
            return None;
        }
        // 長期せん断力 QL（余裕率の分子控除用）を重力ケースの静的結果から引く。
        let gravity_lc = gravity_cases_for_seismic_weight(&self.model)
            .first()
            .copied();
        let long_forces: Option<&[(ElemId, squid_n_element::beam::MemberForces)]> = self
            .results
            .as_ref()
            .and_then(|res| {
                gravity_lc.and_then(|lc| {
                    res.statics
                        .iter()
                        .find(|(id, _)| *id == StaticCaseKey::User(lc))
                })
            })
            .map(|(_, s)| s.member_forces.as_slice());
        let ql_of = |elem: ElemId| -> Option<f64> {
            long_forces?
                .iter()
                .find(|(id, _)| *id == elem)
                .map(|(_, mf)| mf.at.iter().map(|(_, f)| f[1].abs()).fold(0.0, f64::max))
        };
        // 単純梁せん断 Q0（MK785/SPR785/SPR685 使用部材の QL=Q0 読み替え用）。
        let q0_map = gravity_lc
            .map(|lc| simple_beam_q0_by_elem(&self.model, lc))
            .unwrap_or_default();
        Some(
            po.member_response
                .iter()
                .map(|r| {
                    let mut d = MemberDemand::from_pushover(
                        r.axial,
                        r.m_strong,
                        r.m_weak,
                        r.shear_strong,
                        r.shear_weak,
                        r.rp,
                    );
                    d.q_long = ql_of(r.elem);
                    d.q_simple = q0_map.get(&r.elem).copied();
                    (r.elem, d)
                })
                .collect(),
        )
    }

    /// CFT 柱の軸終局検定（CFT指針）: CftBox/CftPipe 柱の
    /// 軸圧縮終局耐力 Ncu・軸引張終局耐力 Ntu に対する軸余裕度を算定する。
    ///
    /// 対象 CFT 柱が 1 つも無い場合は `Err` を返す（UI 側で案内表示）。
    pub fn compute_cft_ultimate_checks(
        &mut self,
    ) -> Result<Vec<squid_n_design_jp::ultimate::CftUltimateCheck>, String> {
        self.apply_rigid_zones_for_analysis();
        // CFT の軸終局検定は軸力のみを用いる（MemberDemand から軸力を取り出す）。
        let axial: Vec<(ElemId, f64)> = self
            .ultimate_demand_by_elem()
            .into_iter()
            .map(|(id, d)| (id, d.n_axial))
            .collect();
        let checks = squid_n_design_jp::ultimate::collect_cft_ultimate_checks(&self.model, &axial);
        if checks.is_empty() {
            return Err(
                "終局検定の対象（CftBox/CftPipe の CFT 柱）がありません。CFT 断面と\
                 コンクリート強度 Fc を設定してください。"
                    .to_string(),
            );
        }
        Ok(checks)
    }

    /// T3: 固有値解析を実行し、結果を `self.results` に格納する。
    pub fn run_eigen(&mut self, n_modes: usize) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.apply_rigid_zones_for_analysis();
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.eigen(n_modes) {
                Ok(modal) => {
                    let mut bundle = self.results.take().unwrap_or_default();
                    bundle.modal = Some(modal);
                    self.results = Some(bundle);
                    // 固有値のみの更新では設計は更新されないが、最新実行時刻は更新
                    self.staleness.last_run = Some(SystemTime::now());
                }
                Err(e) => self.last_error = Some(format!("固有値解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// 階(Story)を節点標高から自動生成して適用する（undo 可能）。
    /// 地震重量には kind=Dead/LiveSeismic（無ければ Dead+Live、種別未設定なら
    /// 先頭ケース）の荷重ケースの鉛直下向き荷重を用いる（レビュー §1.7）。
    /// 先立ってスラブ荷重・躯体自重を「DL」等の標準ケースへ同期する
    /// （レビュー §1.1）ため、面荷重・自重も地震用重量に反映される
    /// （DL に自重が含まれるため、密度からの自重直接算入は DL が無い場合のみ。
    /// `density_self_weight_for_stories`）。
    ///
    /// 階の適用後、地震荷重を「EX」「EY」ケースへ同期する（Ai 分布の水平力。
    /// これで荷重組合せ G+P±K が実行可能になる）。
    pub fn generate_stories_action(&mut self) {
        self.last_error = None;
        self.last_notice = None;
        self.sync_gravity_load_cases_action();
        let gravity_lcs = gravity_cases_for_seismic_weight(&self.model);
        let include_density = density_self_weight_for_stories(&self.model);
        match squid_n_load::story_gen::generate_stories_with_opts(
            &self.model,
            &gravity_lcs,
            include_density,
        ) {
            Ok(gen) => {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::ApplyStories {
                        stories: gen.stories,
                        node_story: gen.node_story,
                        constraints: gen.constraints,
                        rep_nodes: gen.rep_nodes,
                        generated_masters: gen.generated_masters,
                    }),
                );
                self.staleness.mark_edited();
                // 剛域の反映は地震荷重の同期より先に行う（SemiPrecise の固有周期算定が
                // 剛域込みの剛性を用いるようにするため）。
                self.apply_rigid_zones_for_analysis();
                self.sync_seismic_load_cases_action();
                // 直後に run_linear_static 等（`sync_auto_load_cases_action`）が
                // 呼ばれても、いま行った DL/LL/EX/EY の同期を無駄に繰り返さない
                // よう、同期後の状態のハッシュを記録しておく。
                self.auto_load_sync_hash = Some(self.compute_auto_load_sync_hash());
            }
            Err(e) => self.last_error = Some(format!("階の自動生成エラー: {}", e)),
        }
    }

    /// T3: 地震静的解析（Ai一気通貫）を実行し、結果を `self.results` に格納する。
    /// 方向・Ai算定法・Z・地盤種別・C0 は `analysis_cfg` を用いる。
    /// 結果は `StaticCaseKey::Seismic(dir)` に格納するため、X/Y 双方の地震静的結果
    /// および任意のユーザー荷重ケースの結果と衝突せず共存できる。
    /// あわせて同じ水平力を「EX」「EY」ケースへ同期する（荷重組合せ用、
    /// `sync_auto_load_cases_action`。モデル・関連設定が前回同期時から変わって
    /// いなければ丸ごとスキップする）。
    ///
    /// 設計用固有周期 T は `design_seismic_period` で暗黙の解析なしに決定する
    /// （内部で固有値解析を実行しない `Analysis::seismic_static_with_period` を
    /// 使う）。SemiPrecise で固有値解析が未実行の場合は解析せず、実行を促す
    /// メッセージを `last_error` に設定して return する。
    pub fn run_seismic(&mut self, dir: SeismicDir) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => {
                self.last_error = Some(msg);
                return;
            }
        };
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        let res = Self::compute_seismic(self.model.clone(), cfg, t);
        self.apply_static_case_result(StaticCaseKey::Seismic(dir), res);
    }

    /// 地震静的(Ai分布)解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_seismic_job`）からも呼び出せる。
    fn compute_seismic(
        model: squid_n_core::model::Model,
        cfg: squid_n_solver::analysis::SeismicCfg,
        t: f64,
    ) -> Result<squid_n_solver::linear::StaticOnce, String> {
        match Analysis::prepare(&model) {
            Ok(analysis) => analysis
                .seismic_static_with_period(cfg, t)
                .map_err(|e| format!("地震解析エラー: {:?}", e)),
            Err(e) => Err(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// 地震静的解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_seismic_job(&mut self, dir: SeismicDir) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.sync_auto_load_cases_action();
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => {
                self.last_error = Some(msg);
                return;
            }
        };
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        let model = self.model.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_seismic(model, cfg, t)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::StaticCase {
                key: StaticCaseKey::Seismic(dir),
                res,
            });
        });
        self.job = Some(AnalysisJob {
            label: "地震静的解析",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
    }

    /// 風荷重の静的解析を実行し、結果を `StaticCaseKey::Wind(dir)` に格納する
    /// （`run_seismic` と同じパターン。X/Y 双方の結果および他の静的結果と共存できる）。
    /// 基準風速・地表面粗度区分・パラペット高さは `analysis_cfg` を用いる。
    pub fn run_wind(&mut self, dir: SeismicDir) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        let cfg = squid_n_solver::analysis::WindStaticCfg {
            dir,
            v0: self.analysis_cfg.v0,
            roughness: self.analysis_cfg.roughness,
            cpi: 0.0,
            parapet_mm: self.analysis_cfg.parapet_mm,
        };
        self.apply_rigid_zones_for_analysis();
        let res = Self::compute_wind(self.model.clone(), cfg);
        self.apply_static_case_result(StaticCaseKey::Wind(dir), res);
    }

    /// 風荷重静的解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_wind_job`）からも呼び出せる。
    fn compute_wind(
        model: squid_n_core::model::Model,
        cfg: squid_n_solver::analysis::WindStaticCfg,
    ) -> Result<squid_n_solver::linear::StaticOnce, String> {
        match Analysis::prepare(&model) {
            Ok(analysis) => analysis
                .wind_static(cfg)
                .map_err(|e| format!("風荷重解析エラー: {:?}", e)),
            Err(e) => Err(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// 風荷重静的解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_wind_job(&mut self, dir: SeismicDir) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        let cfg = squid_n_solver::analysis::WindStaticCfg {
            dir,
            v0: self.analysis_cfg.v0,
            roughness: self.analysis_cfg.roughness,
            cpi: 0.0,
            parapet_mm: self.analysis_cfg.parapet_mm,
        };
        self.apply_rigid_zones_for_analysis();
        let model = self.model.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_wind(model, cfg)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::StaticCase {
                key: StaticCaseKey::Wind(dir),
                res,
            });
        });
        self.job = Some(AnalysisJob {
            label: "風荷重静的解析",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
    }

    /// Z表 CSV（`squid_n_load::z_table::ZTable::from_csv`）を読み込み `self.z_table`
    /// に格納する（ヘッドレス可、UI 側のファイル選択とは独立にテストできる）。
    pub fn load_z_table_from_csv(&mut self, csv: &str) {
        match squid_n_load::z_table::ZTable::from_csv(csv) {
            Ok(table) => {
                self.z_table = Some(table);
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(format!("Z表読込エラー: {}", e)),
        }
    }

    /// 読み込み済みの Z表（`self.z_table`）から市町村名を引き、`analysis_cfg.z`
    /// へ反映する。Z表が未読込／該当市町村が無い場合は `last_error` を設定して
    /// `false` を返す。
    pub fn apply_z_from_municipality(&mut self, municipality: &str) -> bool {
        let Some(table) = &self.z_table else {
            self.last_error = Some("Z表が読み込まれていません".to_string());
            return false;
        };
        match table.lookup(municipality) {
            Some(z) => {
                self.analysis_cfg.z = z;
                self.last_error = None;
                true
            }
            None => {
                self.last_error = Some(format!("Z表に「{}」が見つかりません", municipality));
                false
            }
        }
    }

    /// 荷重ケースの種別（`LoadCaseKind`）から Dead（必須）/Live（必須）/Snow（任意）/
    /// Wind（任意）を各先頭1件選び、`squid_n_load::combo::standard_combinations` で
    /// 標準組合せを生成し、undo 可能に一括追加する（`AddCombination` を使用）。
    ///
    /// 地震（Seismic 種別）は対象外とする: Kx/Ky の正確な組合せは方向別の地震静的
    /// 解析（`run_seismic`）が別途扱うため、`kind` だけでは方向を判別できない
    /// 単一の LoadCase から機械的に Kx/Ky を割り当てることは行わない
    /// （既存の手動選択 UI [`combinations_section`] が方向を明示して生成する経路を持つ）。
    /// 同じ理由により、Wind も見つかった先頭1件は `wind_x` にのみ割り当てる
    /// （`wind_y` は常に `None`）。
    ///
    /// Dead/Live のいずれかが見つからない場合は組合せを生成せず `last_error` を設定する。
    pub fn auto_generate_combinations_action(&mut self) {
        use squid_n_core::model::LoadCaseKind;

        self.last_error = None;
        let find_first = |kind: LoadCaseKind| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.kind == kind)
                .map(|lc| lc.id)
        };
        let Some(dl) = find_first(LoadCaseKind::Dead) else {
            self.last_error = Some("種別「固定荷重」の荷重ケースが見つかりません".to_string());
            return;
        };
        let Some(ll) = find_first(LoadCaseKind::Live) else {
            self.last_error =
                Some("種別「積載荷重(長期)」の荷重ケースが見つかりません".to_string());
            return;
        };
        let snow = find_first(LoadCaseKind::Snow);
        let wind = find_first(LoadCaseKind::Wind);

        let input = squid_n_load::combo::ComboInput {
            dl,
            ll,
            seismic_x: None,
            seismic_y: None,
            wind_x: wind,
            wind_y: None,
            snow,
            heavy_snow_zone: self.analysis_cfg.heavy_snow_zone,
            snow_factors: Some(squid_n_load::combo::SnowFactors {
                delta1: self.analysis_cfg.snow_delta1,
                delta2: self.analysis_cfg.snow_delta2,
                delta3: self.analysis_cfg.snow_delta3,
            }),
        };
        let combos = squid_n_load::combo::standard_combinations(&input);
        for combo in combos {
            self.undo.run(
                &mut self.model,
                Box::new(squid_n_edit::AddCombination { combo }),
            );
        }
        self.staleness.mark_edited();
    }

    /// プッシュオーバー解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_pushover_job`）からも呼び出せる。
    /// モデルは呼び出し側で複製したものを渡す
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    fn compute_pushover(
        model: squid_n_core::model::Model,
        cfg: AnalysisSettings,
    ) -> Result<squid_n_solver::pushover::PushoverResult, String> {
        let mut work = model;
        // 解析前に剛域を自動算定（設計書 §6.2.1、標準実装）。
        squid_n_element::beam::apply_auto_rigid_zones(
            &mut work,
            &squid_n_element::beam::RigidZoneRule::default(),
        );
        Analysis::prepare(&work).map_err(|e| format!("解析準備エラー: {}", e))?;
        let dofmap = squid_n_core::dof::DofMap::build(&work);
        let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
        squid_n_solver::pushover::pushover_analysis_recording(
            &mut work,
            &dofmap,
            &reducer,
            cfg.push_dir,
            cfg.push_steps,
            cfg.push_max_disp,
            false,
            false,
            0.0,
            false,
            cfg.ductility_method,
        )
        .map_err(|e| format!("プッシュオーバー解析エラー: {}", e))
    }

    /// `compute_pushover` の結果を適用する（bundle 格納・最終実行時刻更新・エラー設定）。
    fn apply_pushover_result(
        &mut self,
        res: Result<squid_n_solver::pushover::PushoverResult, String>,
    ) {
        match res {
            Ok(result) => {
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.pushover = Some(result);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// プッシュオーバー解析を実行する。モデルは複製の上で解析する
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    pub fn run_pushover(&mut self) {
        self.apply_parallelism_setting();
        self.last_error = None;
        let res = Self::compute_pushover(self.model.clone(), self.analysis_cfg);
        self.apply_pushover_result(res);
    }

    /// プッシュオーバー解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_pushover_job(&mut self) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_pushover(model, cfg)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::Pushover(result));
        });
        self.job = Some(AnalysisJob {
            label: "プッシュオーバー",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: Some((Tab::Results, ResultsView::Pushover)),
        });
    }

    /// 線形時刻歴応答解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_time_history_job`）からも呼び出せる。
    /// 減衰モデル・積分法は `cfg` に従う（剛性比例／Rayleigh、Newmark-β／HHT-α）。
    /// 位相差入力（ねじれ加振）を `wave` へ付加する（構造動力学の位相差入力解析）。
    /// `phase_diff_enabled` が false なら `wave` をそのまま返す。位相遅れ時間
    /// `t=(L·sinθ)/Vs` を求め、位相遅れ方向の並進波からねじれ地動加速度を生成する。
    fn apply_phase_diff(
        cfg: &AnalysisSettings,
        mut wave: squid_n_solver::timehistory::GroundMotion,
    ) -> squid_n_solver::timehistory::GroundMotion {
        if !cfg.phase_diff_enabled {
            return wave;
        }
        use squid_n_solver::phase_diff::{phase_lag_time, torsional_accel_series};
        let lag = phase_lag_time(
            cfg.phase_diff_length_m,
            cfg.phase_diff_incidence_deg,
            cfg.phase_diff_vs,
        );
        // 位相遅れ方向の並進加速度を基準波とする。
        let base: Vec<f64> = if cfg.phase_diff_dir_y {
            wave.accel_y.clone().unwrap_or_else(|| wave.accel_x.clone())
        } else {
            wave.accel_x.clone()
        };
        let l_mm = (cfg.phase_diff_length_m * 1000.0).max(1.0);
        let theta = torsional_accel_series(&base, wave.dt, lag, l_mm);
        wave.accel_theta = Some(theta);
        wave
    }

    fn compute_time_history(
        model: squid_n_core::model::Model,
        cfg: AnalysisSettings,
        wave: squid_n_solver::timehistory::GroundMotion,
    ) -> Result<squid_n_solver::timehistory::ResponseResult, String> {
        let mut model = model;
        // 位相差入力（ねじれ加振）を指定時に付加する（構造動力学の位相差入力解析）。
        let wave = Self::apply_phase_diff(&cfg, wave);
        // 解析前に剛域を自動算定（設計書 §6.2.1、標準実装）。
        squid_n_element::beam::apply_auto_rigid_zones(
            &mut model,
            &squid_n_element::beam::RigidZoneRule::default(),
        );
        let analysis = Analysis::prepare(&model).map_err(|e| format!("解析準備エラー: {}", e))?;
        let damping = match cfg.th_damping_model {
            ThDampingModel::StiffnessProportional => {
                // 1 次固有円振動数（減衰の基準）
                let omega1 = match analysis.eigen(1) {
                    Ok(modal) => match modal.omega2.first() {
                        Some(&w2) if w2 > 0.0 => w2.sqrt(),
                        _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
                    },
                    Err(e) => return Err(format!("固有値解析エラー: {}", e)),
                };
                squid_n_solver::damping::Damping::StiffnessProportional {
                    h: cfg.th_damping,
                    omega: omega1,
                    basis: squid_n_solver::damping::StiffnessKind::Initial,
                }
            }
            ThDampingModel::Rayleigh => {
                // 1次・2次の固有円振動数（Rayleigh 減衰の基準）
                let modal = match analysis.eigen(2) {
                    Ok(m) => m,
                    Err(e) => return Err(format!("固有値解析エラー: {}", e)),
                };
                let (w1, w2) = match (modal.omega2.first(), modal.omega2.get(1)) {
                    (Some(&a), Some(&b)) if a > 0.0 && b > 0.0 => (a.sqrt(), b.sqrt()),
                    _ => {
                        return Err(
                            "Rayleigh 減衰には 2 次までの固有値が必要です（モード数を確保できませんでした）。"
                                .to_string(),
                        );
                    }
                };
                squid_n_solver::damping::Damping::Rayleigh {
                    h1: cfg.th_damping,
                    w1,
                    h2: cfg.th_h2,
                    w2,
                }
            }
            ThDampingModel::Modal => {
                // モード別減衰: 得られる低次モードに一律の減衰比 h を与える。
                // 要求モード数はモデルの質量ランクに合わせ 6→1 の順に試行する。
                let mut modal = None;
                for k in (1..=6).rev() {
                    if let Ok(m) = analysis.eigen(k) {
                        if !m.shapes.is_empty() {
                            modal = Some(m);
                            break;
                        }
                    }
                }
                let modal = modal.ok_or("固有値が得られず減衰を設定できません。".to_string())?;
                let omegas: Vec<f64> = modal
                    .omega2
                    .iter()
                    .map(|&w2| if w2 > 0.0 { w2.sqrt() } else { 0.0 })
                    .collect();
                let ratios = vec![cfg.th_damping; modal.shapes.len()];
                squid_n_solver::damping::Damping::modal(&modal.shapes, &omegas, &ratios)
            }
            ThDampingModel::TangentAlpha1 | ThDampingModel::TangentH1 => {
                // 瞬間（接線）剛性比例。基準は初期剛性の 1 次固有円振動数。
                let omega1 = match analysis.eigen(1) {
                    Ok(modal) => match modal.omega2.first() {
                        Some(&w2) if w2 > 0.0 => w2.sqrt(),
                        _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
                    },
                    Err(e) => return Err(format!("固有値解析エラー: {}", e)),
                };
                if cfg.th_damping_model == ThDampingModel::TangentAlpha1 {
                    squid_n_solver::damping::Damping::StiffnessProportional {
                        h: cfg.th_damping,
                        omega: omega1,
                        basis: squid_n_solver::damping::StiffnessKind::Tangent,
                    }
                } else {
                    squid_n_solver::damping::Damping::TangentStiffnessConstantH {
                        h1: cfg.th_damping,
                        omega1e: omega1,
                    }
                }
            }
        };
        let result = match cfg.th_integrator {
            ThIntegrator::NewmarkBeta => {
                let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
                analysis.time_history(&wave, newmark, damping)
            }
            ThIntegrator::HhtAlpha => {
                let hht = squid_n_solver::timehistory::HhtCfg::new(wave.dt);
                analysis.time_history_hht(&wave, hht, damping)
            }
        };
        result.map_err(|e| format!("時刻歴解析エラー: {}", e))
    }

    /// `compute_time_history` の結果を適用する
    /// （bundle 格納・time_history_data 更新(gui)・最終実行時刻更新・エラー設定）。
    fn apply_time_history_result(
        &mut self,
        res: Result<squid_n_solver::timehistory::ResponseResult, String>,
    ) {
        match res {
            Ok(res) => {
                #[cfg(feature = "gui")]
                {
                    self.time_history_data = crate::time_history_view::TimeHistoryData {
                        time: res.time.clone(),
                        node_disp: res.history.node_disp.clone(),
                        story_shear: res.history.base_shear.clone(),
                        story_drift_angle: res.history.top_drift_angle.clone(),
                        node: res.history.node,
                    };
                }
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.time_history = Some(res);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰モデル・積分法は `analysis_cfg` に従う
    /// （剛性比例／Rayleigh、Newmark-β／HHT-α）。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.apply_parallelism_setting();
        self.last_error = None;
        let res = Self::compute_time_history(self.model.clone(), self.analysis_cfg, wave);
        self.apply_time_history_result(res);
    }

    /// 時刻歴応答解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_time_history_job(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        if self.job.is_some() {
            self.last_error = Some("解析実行中です".to_string());
            return;
        }
        self.apply_parallelism_setting();
        self.last_error = None;
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Self::compute_time_history(model, cfg, wave)
            }))
            .unwrap_or_else(|_| {
                Err(
                    "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。"
                        .to_string(),
                )
            });
            let _ = tx.send(JobResult::TimeHistory(result));
        });
        self.job = Some(AnalysisJob {
            label: "時刻歴応答",
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: Some((Tab::Results, ResultsView::TimeHistory)),
        });
    }

    /// 実行中のジョブの完了を確認し、完了していれば結果を適用する。
    /// 成功/失敗いずれかで結果を受信できた場合、またはスレッド異常終了時は
    /// `job` を `None` に戻し `true` を返す。まだ実行中なら `false` を返す。
    pub fn poll_job(&mut self) -> bool {
        let recv = match &self.job {
            Some(job) => job.rx.try_recv(),
            None => return false,
        };
        match recv {
            Ok(result) => {
                #[cfg(feature = "gui")]
                let jump = self.job.take().and_then(|j| j.jump_on_success);
                #[cfg(not(feature = "gui"))]
                {
                    self.job = None;
                }
                match result {
                    JobResult::Pushover(res) => self.apply_pushover_result(res),
                    JobResult::TimeHistory(res) => self.apply_time_history_result(res),
                    JobResult::StaticCase { key, res } => self.apply_static_case_result(key, res),
                    JobResult::Combo { name, res } => self.apply_combo_result(name, res),
                    JobResult::AllCombos {
                        computed,
                        pre_errors,
                    } => self.apply_all_combinations_result(computed, pre_errors),
                }
                #[cfg(feature = "gui")]
                {
                    if self.last_error.is_none() {
                        if let Some((tab, view)) = jump {
                            self.active_tab = tab;
                            self.results_view = view;
                        }
                    }
                }
                true
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.job = None;
                self.last_error = Some(
                    "解析スレッドが異常終了しました（結果を受信できませんでした）。".to_string(),
                );
                true
            }
        }
    }

    /// 正弦減衰のサンプル地震波を `cfg` から組み立てる
    /// （外部波形ファイルなしで機能を試せる導線。同期実行・ジョブ実行の双方で使う）。
    pub(crate) fn sample_wave(cfg: &AnalysisSettings) -> squid_n_solver::timehistory::GroundMotion {
        let n = ((cfg.th_duration / cfg.th_dt).ceil() as usize).max(2);
        let omega = 2.0 * std::f64::consts::PI / cfg.th_period.max(1e-6);
        let accel: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * cfg.th_dt;
                cfg.th_amp * (omega * t).sin() * (-0.3 * t).exp()
            })
            .collect();
        Self::build_ground_motion(cfg.th_dt, cfg.th_dir, accel)
    }

    /// 正弦減衰のサンプル地震波を生成して時刻歴解析を実行する（同期）。
    pub fn run_time_history_sample(&mut self) {
        self.apply_parallelism_setting();
        let wave = Self::sample_wave(&self.analysis_cfg);
        self.run_time_history(wave);
    }

    /// 方向 `dir` に加速度列 `accel` を割り当てた `GroundMotion` を組み立てる。
    /// X なら accel_x、Y なら accel_y に入れ、他方はゼロ列にする。
    /// Xy（X+Y 同時入力）は同一波形を accel_x・accel_y の両方にそのまま入れる
    /// 簡易仕様（位相差・別波形の指定はサポートしない。CSV 2 列入力は
    /// `parse_wave_csv` が別々の列を返すため、その場合は本関数を経由せず
    /// 直接 `GroundMotion` を組み立てる）。
    pub(crate) fn build_ground_motion(
        dt: f64,
        dir: ThDir,
        accel: Vec<f64>,
    ) -> squid_n_solver::timehistory::GroundMotion {
        match dir {
            ThDir::X => squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: accel,
                accel_y: None,
                accel_theta: None,
            },
            ThDir::Y => {
                let n = accel.len();
                squid_n_solver::timehistory::GroundMotion {
                    dt,
                    accel_x: vec![0.0; n],
                    accel_y: Some(accel),
                    accel_theta: None,
                }
            }
            ThDir::Xy => squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: accel.clone(),
                accel_y: Some(accel),
                accel_theta: None,
            },
        }
    }

    /// T7: 解析結果の member_forces から検定結果を生成する。
    /// 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力に対し、
    /// 材種・部材種別に応じた検定を適用する（令82条・各構造設計規準準拠）。
    /// 節点芯は剛域が有る場合は検定対象外（節点芯の応力をそのまま使わない、
    /// 設計書 §6.2.3）。
    ///
    /// - 部材種別は部材軸の鉛直成分から判定（柱/梁/ブレース）。
    /// - せん断スパン比 M/(Q·d) 用の代表値は、モーメントが最大となる
    ///   検定位置の値を採用する方針で部材単位に求める。
    /// - 柱は軸力＋二軸曲げ（n, my, mz）を検定に渡す。
    /// - 検定器は形状優先（SRC/CFT）、それ以外は材料名で鋼/RC を選択する。
    pub fn run_design_check(&mut self) {
        // rigid_zone（face_i/j）から危険断面位置を決めるため、算定前に自動剛域を
        // 反映する（設計書 §6.2.1、冪等なので他の解析エントリと重複して呼んでも安全）。
        self.apply_rigid_zones_for_analysis();
        let Some(results) = &self.results else {
            return;
        };
        // 地震時短期の設計用せん断力 QD = min(QD1, QD2) 用の長期(G+P)内力。
        // 現在の結果が地震時組合せ（名前に K/E を含む）かつ短期のときのみ、
        // 解析済みの長期組合せ（"G + P" 優先、無ければ長期判定の組合せ）を引く。
        // 長期が未解析なら None（QD 割増なし＝従来動作）。
        let is_seismic_combo = match self.last_static {
            Some(StaticKey::Combo(idx)) => results
                .combos
                .get(idx)
                .map(|(n, _)| {
                    let u = n.to_uppercase();
                    u.contains('K') || u.contains('E')
                })
                .unwrap_or(false),
            _ => false,
        };
        let long_member_forces: Option<&Vec<(ElemId, squid_n_element::beam::MemberForces)>> =
            if is_seismic_combo && self.design_term == LoadTerm::Short {
                results
                    .combos
                    .iter()
                    .find(|(n, _)| n == "G + P")
                    .or_else(|| {
                        results
                            .combos
                            .iter()
                            .find(|(n, _)| !squid_n_load::combo::is_short_term_combo(n))
                    })
                    .map(|(_, st)| &st.member_forces)
            } else {
                None
            };
        // 一本部材指定（Model.beam_groups）: グループ単位の採用応力を合成し、
        // 所属部材の検定文脈（部材長・端部/中央モーメント等）を上書きする。
        let group_overrides = beam_group_overrides(&self.model, &results.member_forces);
        // 部材ID→要素の索引（旧: ループ内で `elements.iter().find` していたため
        // O(部材数²) だった）。ループ前に一度だけ構築する。同一IDが重複した場合に
        // `find`（先頭一致）と同じ結果になるよう、forward 走査 + `entry().or_insert`
        // で「先勝ち」にする（`or_insert` は既に埋まっていれば上書きしない）。
        let mut elem_by_id: std::collections::HashMap<ElemId, &squid_n_core::model::ElementData> =
            std::collections::HashMap::with_capacity(self.model.elements.len());
        for e in &self.model.elements {
            elem_by_id.entry(e.id).or_insert(e);
        }
        // 地震時短期 QD 用の長期内力索引（旧: 部材ごとに `list.iter().find` して
        // いたため O(部材数²)）。同上の「先勝ち」ルールで構築する。
        let long_mf_by_id: Option<
            std::collections::HashMap<ElemId, &squid_n_element::beam::MemberForces>,
        > = long_member_forces.map(|list| {
            let mut m = std::collections::HashMap::with_capacity(list.len());
            for (id, mf) in list {
                m.entry(*id).or_insert(mf);
            }
            m
        });
        // 柱の座屈長さ係数 K 用の節点まわり梁索引（`buckling::g_ratio_at` の全要素
        // 走査を避けるため、ループ前に 1 回だけ構築して使い回す）。
        let column_k_index = squid_n_design_jp::steel::buckling::BeamNodeIndex::build(&self.model);
        let mut checks: Vec<(ElemId, f64, squid_n_design_jp::CheckResult)> = Vec::new();
        for (elem_id, mf) in &results.member_forces {
            let elem = elem_by_id.get(elem_id).copied();
            let Some(elem) = elem else {
                continue;
            };
            let sec = elem
                .section
                .and_then(|sid| self.model.sections.get(sid.index()))
                .filter(|s| s.id == elem.section.unwrap());
            let mat = elem
                .material
                .and_then(|mid| self.model.materials.get(mid.index()))
                .filter(|m| m.id == elem.material.unwrap());
            let (Some(sec), Some(mat)) = (sec, mat) else {
                continue;
            };

            let kind = member_kind_of(elem, &self.model);
            let length = elem_geometric_length(elem, &self.model);
            // せん断スパン比 M/(Q·d) の代表値: 加力方向ごとに「モーメントが最大と
            // なる検定位置」の (|M|, |Q|) を採用する（強軸: |Mz|max と対応 |Qy|、
            // 弱軸: |My|max と対応 |Qz|。従来は強軸側の1組を弱軸検定にも流用して
            // おり、弱軸曲げ卓越の柱で α を過大評価していた）。
            let shear_span = mf
                .at
                .iter()
                .map(|(_, f)| (f[5].abs(), f[1].abs()))
                .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let shear_span_y = mf
                .at
                .iter()
                .map(|(_, f)| (f[4].abs(), f[2].abs()))
                .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            // 端部・中央の強軸曲げ（横座屈 C 係数・たわみ検定用）。
            let m_at = |target: f64| {
                mf.at
                    .iter()
                    .find(|(p, _)| (p - target).abs() < 1e-9)
                    .map(|(_, f)| f[5])
            };
            let end_moments_z = match (m_at(0.0), m_at(1.0)) {
                (Some(a), Some(b)) => Some((a, b)),
                _ => None,
            };
            // 柱の座屈長さ lk = K・h（鋼構造塑性設計指針、水平移動が拘束されない
            // 場合。K は節点まわり剛度比 G から算定）。柱以外は None（lk=部材長）。
            // RC 柱の検定は lk を使わないため、柱一律で設定して問題ない。
            // K は現状 1 方向の剛度比から算定しており両軸同値（lk_y=lk_z）。
            // 軸別 K の精緻化は将来課題。
            let lk = if kind == squid_n_design_jp::MemberKind::Column {
                squid_n_design_jp::steel::buckling::steel_column_k_with_index(
                    &self.model,
                    &column_k_index,
                    elem,
                )
                .map(|k| k * length)
            } else {
                None
            };
            // 一本部材グループに属する梁は、部材長・端部/中央モーメント・せん断
            // スパン比代表値をグループ合成値に置き換える（断面検定の採用応力。
            // 一本部材指定時の採用応力）。
            let group = if kind == squid_n_design_jp::MemberKind::Beam {
                group_overrides.get(elem_id)
            } else {
                None
            };
            let (length, shear_span, end_moments_z, mid_moment_z) = match group {
                Some(g) => (g.length, g.shear_span, g.end_moments_z, g.mid_moment_z),
                None => (length, shear_span, end_moments_z, m_at(0.5)),
            };
            // 地震時短期の設計用せん断力 QD の文脈（長期内力・割増係数 n・内法長）。
            // 内法長 l′/h′ は剛域（フェイス距離）控除後の長さとする。
            let seismic_qd =
                long_mf_by_id
                    .as_ref()
                    .and_then(|map| map.get(elem_id))
                    .map(|mf_long| {
                        let face_sum = elem.rigid_zone.face_i + elem.rigid_zone.face_j;
                        let clear_length = match group {
                            // 一本部材は両外端の剛域控除後のグループ内法長。
                            Some(g) => g.clear_length,
                            None if length - face_sum > 0.0 => length - face_sum,
                            None => length,
                        };
                        squid_n_design_jp::SeismicQd {
                            long_at: mf_long.at.clone(),
                            // 割増係数 n（柱は 1.5 以上）。梁・柱とも 1.5。
                            n_factor: 1.5,
                            clear_length,
                            method: self.analysis_cfg.qd_method,
                        }
                    });
            // S 造部材の断面検定属性（欠損率・横座屈長さ）。
            let steel_attr = self
                .model
                .steel_design_attrs
                .iter()
                .find(|a| a.elem == *elem_id)
                .cloned();
            let ctx = DesignCtx {
                term: self.design_term,
                kind,
                length,
                lb: None,
                lk_y: lk,
                lk_z: lk,
                shear_span,
                shear_span_y,
                rc_damage_control: self.analysis_cfg.rc_damage_control,
                end_moments_z,
                mid_moment_z,
                seismic_qd,
                steel_attr,
                steel_fb_rule: Default::default(),
            };

            // 検定器の選択: 複合断面（SRC/CFT）は形状優先、それ以外は材料名で鋼/RC。
            let checker: Box<dyn DesignCheck> = match sec.shape {
                Some(squid_n_core::section_shape::SectionShape::SrcRect { .. }) => {
                    Box::new(squid_n_design_jp::SrcDesign)
                }
                Some(squid_n_core::section_shape::SectionShape::CftBox { .. })
                | Some(squid_n_core::section_shape::SectionShape::CftPipe { .. }) => {
                    Box::new(squid_n_design_jp::CftDesign)
                }
                _ if is_steel(&mat.name) => Box::new(SteelDesign),
                _ => Box::new(RcDesign),
            };

            let detail = self.model.member_detail(*elem_id);
            let positions = design_positions(elem, length, detail);

            for (pos, forces) in &mf.at {
                if !is_near_design_position(*pos, &positions) {
                    continue;
                }
                // [N, Qy, Qz, Mx, My, Mz] -> MemberForcesAt（N は引張正の部材内力）
                let mfa = MemberForcesAt {
                    pos: *pos,
                    n: forces[0],
                    qy: forces[1],
                    qz: forces[2],
                    my: forces[4],
                    mz: forces[5],
                };
                // BRB 属性が登録された部材はメーカー許容値による BRB 検定に
                // 差し替える（座屈補剛ブレースの断面検定）。
                let cr = if let Some(brb) = self.model.brb_attrs.iter().find(|a| a.elem == *elem_id)
                {
                    squid_n_design_jp::brb::brb_check(
                        brb,
                        mfa.n,
                        length,
                        self.design_term == LoadTerm::Long,
                    )
                } else {
                    checker.check(&mfa, sec, mat, &ctx)
                };
                checks.push((*elem_id, *pos, cr));
            }
        }
        // 節点単位の検定（RC 柱梁接合部・S/SRC パネルゾーン・冷間成形耐力比・耐震壁）。
        // 冷間成形の存在軸力 N = NL + 1.5・NE のため、地震時は長期内力も渡す。
        let mf_slices: Vec<(ElemId, squid_n_design_jp::joint_wiring::ForcesAt)> = results
            .member_forces
            .iter()
            .map(|(id, mf)| (*id, mf.at.as_slice()))
            .collect();
        let long_slices: Option<Vec<(ElemId, squid_n_design_jp::joint_wiring::ForcesAt)>> =
            long_member_forces.map(|list| {
                list.iter()
                    .map(|(id, mf)| (*id, mf.at.as_slice()))
                    .collect()
            });
        let joint_checks = squid_n_design_jp::joint_wiring::collect_joint_checks_with_long(
            &self.model,
            &mf_slices,
            long_slices.as_deref(),
            self.design_term,
        );
        // PCa 水平接合面の検定（PcaBeamAttr が登録された梁のみ。使用限界・終局限界）。
        checks.extend(squid_n_design_jp::rc::horizontal_joint::collect_pca_checks(
            &self.model,
            &mf_slices,
            self.design_term == LoadTerm::Long,
        ));
        // 床の中での小梁・スラブ設計（全体 FEM から独立。小梁は大梁を分割しない）。
        let (joist_checks, slab_checks) = self.floor_design_checks();

        if let Some(bundle) = self.results.as_mut() {
            bundle.checks = checks;
            bundle.joint_checks = joint_checks;
            bundle.joist_checks = joist_checks;
            bundle.slab_checks = slab_checks;
        }
    }

    /// 床の中での小梁・スラブ設計を算定する（`run_design_check` から呼ぶ）。
    ///
    /// - 小梁: 支持2節点間を単純支持梁とし、床用積載（令85条1項の床用）＋固定荷重の
    ///   等分布 w·spacing で曲げ・たわみを検定する。反力は大梁へ CMQ として伝達する
    ///   前提のため、小梁は大梁を分割しない。実部材化された小梁（支持間に実 Beam が
    ///   存在）は全体 FEM で検定するため対象外。断面未割当の小梁もスキップする。
    /// - スラブ: 矩形スラブの短辺を設計スパンとし、一方向版として設計曲げモーメントと
    ///   必要鉄筋量を算定する（鋼小梁・SD295 鉄筋の既定値を用いる）。
    pub(crate) fn floor_design_checks(
        &self,
    ) -> (Vec<crate::app::JoistCheck>, Vec<crate::app::SlabCheck>) {
        use squid_n_core::model::LoadPurpose;
        use squid_n_design_jp::floor as fd;

        let mut joist_checks = Vec::new();
        let mut slab_checks = Vec::new();

        let beam_between = |a: NodeId, b: NodeId| -> bool {
            self.model.elements.iter().any(|e| {
                e.kind == squid_n_core::model::ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };

        for slab in &self.model.slabs {
            // 床設計は床用積載（最大）＋固定荷重を用いる。
            let w = slab.intensity(LoadPurpose::Floor);

            let sigma_allow = 235.0 / 1.5; // 鋼の長期許容曲げ応力度 F/1.5（既定 F=235）。
            let z_of = |sid: squid_n_core::ids::SectionId| -> Option<f64> {
                let sec = self.model.sections.get(sid.index())?;
                // 強軸断面係数 Z = Iy / (depth/2)。
                Some(if sec.depth > 0.0 {
                    sec.iy / (sec.depth / 2.0)
                } else {
                    0.0
                })
            };

            // --- 小梁: 交差があれば床格子サブモデル（二方向）で、無ければ単純支持梁で検定 ---
            let grillage = crate::floor_grillage::build_slab_grillage(&self.model, slab, w)
                .and_then(|g| {
                    crate::floor_grillage::solve_grillage(&g.model, LoadCaseId(0))
                        .ok()
                        .map(|sol| (g, sol))
                });
            if let Some((g, sol)) = grillage {
                // 格子 FEM の部材力・たわみで各小梁を検定（十字梁の二方向挙動を反映）。
                for (jidx, span, m, q, defl) in crate::floor_grillage::joist_design_forces(&g, &sol)
                {
                    let Some(j) = slab.joists.get(jidx) else {
                        continue;
                    };
                    let Some(sid) = j.section else { continue };
                    let Some(z) = z_of(sid) else { continue };
                    let r = fd::design_joist_from_forces(
                        span,
                        w * j.spacing,
                        m,
                        q,
                        defl,
                        z,
                        sigma_allow,
                        fd::DEFLECTION_LIMIT_DENOM,
                    );
                    joist_checks.push((slab.id, jidx, r));
                }
            } else {
                // 交差なし: 各小梁を独立した単純支持梁として検定。
                for (ji, j) in slab.joists.iter().enumerate() {
                    let (a, b) = (j.support[0], j.support[1]);
                    if a == b || beam_between(a, b) {
                        // 実部材化済み or 退化した小梁は床設計の対象外。
                        continue;
                    }
                    let Some(sid) = j.section else { continue };
                    let Some(z) = z_of(sid) else { continue };
                    let Some(sec) = self.model.sections.get(sid.index()) else {
                        continue;
                    };
                    let (Some(na), Some(nb)) = (
                        self.model.nodes.get(a.index()),
                        self.model.nodes.get(b.index()),
                    ) else {
                        continue;
                    };
                    let span = {
                        let d = [
                            nb.coord[0] - na.coord[0],
                            nb.coord[1] - na.coord[1],
                            nb.coord[2] - na.coord[2],
                        ];
                        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                    };
                    if span <= 1e-9 {
                        continue;
                    }
                    let r = fd::design_joist_simple(
                        span,
                        w * j.spacing,
                        z,
                        sec.iy,
                        fd::STEEL_YOUNG,
                        sigma_allow,
                        fd::DEFLECTION_LIMIT_DENOM,
                    );
                    joist_checks.push((slab.id, ji, r));
                }
            }

            // --- スラブ（一方向版） ---
            if let Some((lx, ly)) = squid_n_load::floor::slab_dimensions(&self.model, slab) {
                use squid_n_core::model::OneWayDir;
                // 設計スパンは伝達方向に一致させる（分配エンジンと同じ規約: X→lx, Y→ly）。
                // 一方向指定が無い（両方向）場合は安全側に短辺で設計する。
                let span = match slab.one_way {
                    Some(OneWayDir::X) => lx,
                    Some(OneWayDir::Y) => ly,
                    None => lx.min(ly),
                };
                let thickness = slab.thickness.unwrap_or(self.model.slab_thickness);
                if span > 1e-9 && thickness > 0.0 {
                    // 単純支持相当（coef=8）。連続版はより小さい係数だが安全側に 8 を用いる。
                    let r = fd::design_slab_oneway(
                        span,
                        w,
                        8.0,
                        thickness,
                        fd::SLAB_DEFAULT_COVER,
                        fd::REBAR_FT_LONG_SD295,
                        fd::SLAB_J_RATIO,
                    );
                    slab_checks.push((slab.id, r));
                }
            }
        }
        (joist_checks, slab_checks)
    }

    /// 全スラブの床荷重を大梁（および小梁経由の節点反力）へ分配し、
    /// `self.beam_loads` を更新する。対応する梁が無い辺の荷重は捨てる。
    ///
    /// `squid_n_load::floor::distribute_slab` が返す `BeamLoad.target` は
    /// `LoadTarget::Edge(i)`（スラブ境界の辺 i、`boundary[i]` → `boundary[(i+1)%n]`、
    /// n = 境界頂点数。矩形に限らず三角形・五角形以上の多角形にも対応）または
    /// `LoadTarget::Node(id)`（小梁反力などの節点集中荷重）。`Edge` はここで
    /// その節点対を両端に持つ `Beam` 要素を探し、実 `ElemId` に置き換える
    /// （ノード順は不問）。`Node` はそのまま（`elem` は番兵 `ElemId(u32::MAX)`
    /// のまま）保持する（部材マッピング不要。`sync_gravity_load_cases_action` が
    /// `NodalLoad` へ変換する。CMQ 図描画側は `elem` で梁を引くため、この番兵は
    /// 単に描画対象外になるだけで安全）。
    pub fn refresh_beam_loads(&mut self) {
        // CMQ 図表示・従来互換のため `self.beam_loads` には固定荷重（DL）分配を格納する。
        self.beam_loads = self.slab_beam_loads(|slab| slab.dead_intensity());
    }

    /// 交差小梁スラブについて、床格子サブモデル（二方向）の**支点反力**を大梁接続点
    /// への集中荷重（下向き）として返す（床 Phase F-3b）。`None` の場合は呼び出し側が
    /// 既存の平行小梁モデル（`distribute_rect_with_joists` の点反力）を用いる。
    ///
    /// 反力は面荷重強度 `w` に線形なので各荷重ケースの `w` で解き直す。格子の各小梁は
    /// 平行モデルと同じ `w·spacing` を負担するため、支点反力の総和は平行モデルの小梁
    /// 反力総和（`w·Σ spacing·L`）と厳密に一致する（総和保存）。相違は交点での荷重
    /// 分担の精度のみ。実部材化された小梁を含むスラブは、実 Beam が本体 FEM で荷重を
    /// 伝達し二重計上になるため対象外（`None`）とする。
    ///
    /// `beam_map` は節点対（`(min,max)` 順）→実 `Beam` 要素 `ElemId` の索引
    /// （`beam_elem_map` で構築）。実部材化判定をスラブ全要素走査から `HashMap` 参照に
    /// 落とし、呼び出し側（`slab_beam_loads`／`slab_grillage_unit_reactions`）で 1 回
    /// 構築したものを使い回す（性能）。
    pub(crate) fn slab_grillage_node_reactions(
        &self,
        slab: &squid_n_core::model::Slab,
        w: f64,
        beam_map: &std::collections::HashMap<(NodeId, NodeId), ElemId>,
    ) -> Option<Vec<(NodeId, f64)>> {
        // `distribute_slab_w` が小梁二段階伝達（点反力 Node＋境界 Edge）を採るスラブに
        // 限定する。隅・片持ち・辺支持・非矩形・分配法が三角/一方向以外のスラブは
        // 小梁が使われず全面積が Edge/隅集中で分配されるため、格子反力を上乗せすると
        // 二重計上（または隅集中荷重の取りこぼし）になる。
        if !squid_n_load::floor::uses_joist_distribution(&self.model, slab) {
            return None;
        }
        // 実部材化された小梁を含む場合は対象外（本体 FEM と二重計上を避ける）。
        if slab
            .joists
            .iter()
            .any(|j| beam_map.contains_key(&beam_key(j.support[0], j.support[1])))
        {
            return None;
        }
        let g = crate::floor_grillage::build_slab_grillage(&self.model, slab, w)?;
        let sol = crate::floor_grillage::solve_grillage(&g.model, LoadCaseId(0)).ok()?;
        // 支点反力 Fz（上向き正）＝大梁が受け取る下向き荷重の大きさ。
        Some(
            g.support_origin
                .iter()
                .map(|(n, id)| (*id, sol.reactions[*n][2]))
                .collect(),
        )
    }

    /// 全スラブの床格子**単位**（`w=1.0`）支点反力を 1 回だけ解いて返す
    /// （`SlabId` → 支点反力列）。反力は面荷重強度 `w` に線形（`build_slab_grillage`
    /// の等分布荷重が `w·spacing` で唯一 `w` に依存し、以降の剛性解析は `w` に
    /// 無関係）なため、各荷重ケースはこの単位反力を `w` 倍するだけでよく、
    /// 交差小梁スラブ毎の格子 FEM 組立・求解を `sync_gravity_load_cases_action` の
    /// DL/LL(架構用)/LL(地震用) 3 ケースで使い回せる（従来は 3 回solve_grillageを
    /// 実行していた）。格子非対象（`None`）のスラブはキーが存在しない。
    fn slab_grillage_unit_reactions(
        &self,
        beam_map: &std::collections::HashMap<(NodeId, NodeId), ElemId>,
    ) -> std::collections::HashMap<squid_n_core::ids::SlabId, Vec<(NodeId, f64)>> {
        let mut out = std::collections::HashMap::new();
        for slab in &self.model.slabs {
            if let Some(reactions) = self.slab_grillage_node_reactions(slab, 1.0, beam_map) {
                out.insert(slab.id, reactions);
            }
        }
        out
    }

    /// 節点対 (min,max) → 実 `Beam`（2 節点）要素の `ElemId` を引く索引を構築する
    /// （`slab_beam_loads` の辺→部材対応付け・`slab_grillage_node_reactions` の
    /// 実部材化判定で共有。従来は各スラブ・各辺・各交差小梁ごとに `elements.iter().find`
    /// していたため O(スラブ数×辺数×要素数) だったのを、`sync_gravity_load_cases_action`
    /// 1 回あたり O(要素数) の構築＋ O(1) 参照に削減する）。同一節点対に複数の実梁が
    /// ある場合（通常は起きない）は要素順で最初に見つかったものを採用する
    /// （`.find` の挙動を保つため `entry().or_insert` を使う）。
    pub(crate) fn beam_elem_map(&self) -> std::collections::HashMap<(NodeId, NodeId), ElemId> {
        let mut map = std::collections::HashMap::new();
        for e in &self.model.elements {
            if e.kind == squid_n_core::model::ElementKind::Beam && e.nodes.len() == 2 {
                map.entry(beam_key(e.nodes[0], e.nodes[1])).or_insert(e.id);
            }
        }
        map
    }

    /// 各スラブについて面荷重強度 `w_of(slab)`（N/mm²）を境界へ分配し、
    /// `LoadTarget::Edge` を実 `ElemId` に対応付けた `BeamLoad` 列を返す。
    /// 対応する梁が無い辺の荷重は捨てる。`refresh_beam_loads`（DL）と
    /// `sync_gravity_load_cases_action`（LL）の共通経路（令85条1項の DL/LL 分離）。
    ///
    /// 交差小梁スラブ（軸平行・全仮想）は、平行小梁モデルの小梁点反力
    /// （`LoadTarget::Node`）を床格子サブモデルの支点反力で置換する（床 Phase F-3b。
    /// 総和は保存し、交点での荷重分担のみ高精度化）。境界大梁の残り負担
    /// （`LoadTarget::Edge`）や実部材化小梁（`LoadTarget::Span`）はそのまま。
    ///
    /// 単発呼び出し向けの簡便版。節点対索引・格子単位反力をこの呼び出し内だけで
    /// 構築するため、`sync_gravity_load_cases_action` のように同一モデルに対して
    /// 複数回（DL/LL(架構用)/LL(地震用)）呼ぶ場合は、事前に構築したものを
    /// `slab_beam_loads_with` へ渡して使い回すこと（格子 FEM の重複組立・求解を防ぐ）。
    fn slab_beam_loads(
        &self,
        w_of: impl Fn(&squid_n_core::model::Slab) -> f64,
    ) -> Vec<squid_n_load::floor::BeamLoad> {
        let beam_map = self.beam_elem_map();
        let unit_reactions = self.slab_grillage_unit_reactions(&beam_map);
        self.slab_beam_loads_with(w_of, &unit_reactions, &beam_map)
    }

    /// `slab_beam_loads` の本体。`beam_map`（節点対→実 `ElemId` 索引）と
    /// `unit_reactions`（`slab_grillage_unit_reactions` が返す `w=1` 支点反力。
    /// 反力は `w` に線形なのでここで `w` 倍して使う）を呼び出し側から受け取ることで、
    /// `sync_gravity_load_cases_action` の DL/LL(架構用)/LL(地震用) 3 ケースが
    /// 実要素索引の構築・格子 FEM の組立/求解を 1 回だけ共有できる。
    fn slab_beam_loads_with(
        &self,
        w_of: impl Fn(&squid_n_core::model::Slab) -> f64,
        unit_reactions: &std::collections::HashMap<squid_n_core::ids::SlabId, Vec<(NodeId, f64)>>,
        beam_map: &std::collections::HashMap<(NodeId, NodeId), ElemId>,
    ) -> Vec<squid_n_load::floor::BeamLoad> {
        let mut beam_loads = Vec::new();
        for slab in &self.model.slabs {
            let n = slab.boundary.len();
            if n < 3 {
                continue;
            }
            // 節点対 (n0,n1) を両端に持つ実 Beam 要素の ElemId を引く（ノード順不問）。
            let find_beam = |n0: NodeId, n1: NodeId| -> Option<ElemId> {
                beam_map.get(&beam_key(n0, n1)).copied()
            };
            let w = w_of(slab);
            // 交差小梁スラブは格子サブモデルの支点反力（単位反力を w 倍）で
            // 小梁点反力を置換する（F-3b）。反力は w に線形（build_slab_grillage の
            // コメント参照）なので、格子 FEM を w ごとに解き直す必要はない。
            let grillage_reactions: Option<Vec<(NodeId, f64)>> = unit_reactions
                .get(&slab.id)
                .map(|rs| rs.iter().map(|(node, r)| (*node, r * w)).collect());
            for mut bl in squid_n_load::floor::distribute_slab_w(&self.model, slab, w) {
                match bl.target {
                    squid_n_load::floor::LoadTarget::Node(_) => {
                        // 格子が有効なら平行小梁モデルの点反力は捨てる（格子反力で置換）。
                        if grillage_reactions.is_none() {
                            beam_loads.push(bl);
                        }
                    }
                    squid_n_load::floor::LoadTarget::Edge(k) => {
                        if k >= n {
                            continue;
                        }
                        let n0 = slab.boundary[k];
                        let n1 = slab.boundary[(k + 1) % n];
                        match find_beam(n0, n1) {
                            Some(elem) => {
                                bl.elem = elem;
                                beam_loads.push(bl);
                            }
                            None => {
                                // 対応する実梁が無い辺（二次部材（小梁）上の辺・大梁の
                                // 中間区間など）は節点対を保持して渡し、
                                // `slab_load_case_content` が主架構へ変換する
                                // （大梁の部分分布 or 単純梁反力→CMQ）。
                                // Edge の `elem` は辺番号が入っているため、実部材と
                                // 誤解されないよう番兵へ明示的に戻す。
                                bl.elem = ElemId(u32::MAX);
                                bl.target = squid_n_load::floor::LoadTarget::Span([n0, n1]);
                                beam_loads.push(bl);
                            }
                        }
                    }
                    // 実部材化された小梁: 節点対から実 Beam の ElemId を解決して載せる。
                    // 解決できない節点対はそのまま渡し、`slab_load_case_content` が
                    // 主架構へ変換する。
                    squid_n_load::floor::LoadTarget::Span([n0, n1]) => {
                        if let Some(elem) = find_beam(n0, n1) {
                            bl.elem = elem;
                        }
                        beam_loads.push(bl);
                    }
                }
            }
            // 格子反力を大梁接続点への下向き集中荷重として追加（点反力の置換）。
            if let Some(reactions) = grillage_reactions {
                for (node, r) in reactions {
                    if r.abs() <= 1e-9 {
                        continue;
                    }
                    beam_loads.push(squid_n_load::floor::BeamLoad {
                        elem: ElemId(u32::MAX),
                        target: squid_n_load::floor::LoadTarget::Node(node),
                        shape: squid_n_load::floor::LoadShape::Point { p: r, x: 0.0 },
                        cmq: squid_n_load::floor::Cmq {
                            c_i: 0.0,
                            c_j: 0.0,
                            q_i: r,
                            q_j: 0.0,
                        },
                    });
                }
            }
        }
        beam_loads
    }

    /// `self.beam_loads`（`refresh_beam_loads` 適用後の値）を荷重ケースへ書き込める
    /// `NodalLoad`/`MemberLoad` へ変換する（レビュー §1.1）。作用方向は常に
    /// 鉛直下向き `[0,0,-1]`（面荷重は重力方向のみを扱う既存の前提を踏襲）。
    ///
    /// - `LoadShape::Uniform{w}` → 全長等分布 `Distributed{a:0,b:L,w1:w,w2:w}`
    /// - `LoadShape::Triangle{w0}`（中央 `L/2` で頂点を持つ左右対称三角形）→
    ///   2 区間の線形分布`[0,L/2]: 0→w0` / `[L/2,L]: w0→0` に分割
    ///   （`MemberLoadKind::Distributed` は線形区間しか表現できないため）
    /// - `LoadShape::Trapezoid{w0,a,b}`（両端で `a` ずつ立ち上がり、中央 `b` が
    ///   フラット、`2a+b=L`）→ 3 区間 `[0,a]:0→w0` / `[a,a+b]:w0→w0` /
    ///   `[a+b,L]:w0→0`
    /// - `LoadShape::Point{p,x}` → 中間集中荷重 `MemberLoadKind::Point{a:x,p}`
    /// - `LoadTarget::Node(n)`（小梁反力）→ `NodalLoad{node:n, values:[0,0,-p,0,0,0]}`
    ///
    /// `L` は対応する部材の節点間距離（`elem_geometric_length`。剛域補正なしの
    /// 簡易値。仕様上「部材の節点間距離」を使う規則のため、剛域を考慮する
    /// 設計検定側の `clear_span` とは別物）。
    fn slab_load_case_content(
        &self,
        beam_loads: &[squid_n_load::floor::BeamLoad],
    ) -> (
        Vec<squid_n_core::model::NodalLoad>,
        Vec<squid_n_core::model::MemberLoad>,
    ) {
        use squid_n_core::model::{MemberLoad, MemberLoadKind, NodalLoad};
        use squid_n_load::floor::{LoadShape, LoadTarget};
        use squid_n_load::secondary::{beam_span_position, SPAN_TOL_MM};

        const DIR: [f64; 3] = [0.0, 0.0, -1.0];
        let mut nodal = Vec::new();
        let mut member = Vec::new();

        fn push_dist(member: &mut Vec<MemberLoad>, elem: ElemId, a: f64, b: f64, w1: f64, w2: f64) {
            if b - a <= 1e-9 {
                return;
            }
            member.push(MemberLoad {
                elem,
                dir: DIR,
                kind: MemberLoadKind::Distributed { a, b, w1, w2 },
            });
        }

        // 形状を「部材 `elem` の区間 [a0, a0+len_e]」へ載せる（`a0=0`・`len_e=部材長`
        // なら従来の全長スパン）。`flip` は載荷区間の向きが部材軸と逆
        // （n0 が j 端側）の場合に Point の位置を反転する（分布形状は対称なので不変）。
        fn emit_shape(
            member: &mut Vec<MemberLoad>,
            elem: ElemId,
            a0: f64,
            len_e: f64,
            flip: bool,
            shape: &LoadShape,
        ) {
            match *shape {
                LoadShape::Uniform { w } => push_dist(member, elem, a0, a0 + len_e, w, w),
                LoadShape::Triangle { w0 } => {
                    let mid = len_e / 2.0;
                    push_dist(member, elem, a0, a0 + mid, 0.0, w0);
                    push_dist(member, elem, a0 + mid, a0 + len_e, w0, 0.0);
                }
                LoadShape::Trapezoid { w0, a, b } => {
                    push_dist(member, elem, a0, a0 + a, 0.0, w0);
                    push_dist(member, elem, a0 + a, a0 + a + b, w0, w0);
                    push_dist(member, elem, a0 + a + b, a0 + len_e, w0, 0.0);
                }
                LoadShape::Point { p, x } => {
                    let xx = if flip { len_e - x } else { x };
                    member.push(MemberLoad {
                        elem,
                        dir: DIR,
                        kind: MemberLoadKind::Point { a: a0 + xx, p },
                    });
                }
            }
        }

        // 形状の合計荷重と、単純梁とみなした場合の両端反力 (R0, R1)。
        // 分布形状は対称なので折半、Point は載荷位置に応じて按分する。
        fn simple_reactions(shape: &LoadShape, len: f64) -> (f64, f64) {
            match *shape {
                LoadShape::Uniform { w } => {
                    let total = w * len;
                    (total / 2.0, total / 2.0)
                }
                LoadShape::Triangle { w0 } => {
                    let total = w0 * len / 2.0;
                    (total / 2.0, total / 2.0)
                }
                LoadShape::Trapezoid { w0, a, b } => {
                    let total = w0 * (a + b);
                    (total / 2.0, total / 2.0)
                }
                LoadShape::Point { p, x } => {
                    if len <= 1e-9 {
                        (p / 2.0, p / 2.0)
                    } else {
                        let t = (x / len).clamp(0.0, 1.0);
                        (p * (1.0 - t), p * t)
                    }
                }
            }
        }

        for bl in beam_loads {
            match bl.target {
                LoadTarget::Node(n) => {
                    let LoadShape::Point { p, .. } = bl.shape else {
                        continue;
                    };
                    nodal.push(NodalLoad {
                        node: n,
                        values: [0.0, 0.0, -p, 0.0, 0.0, 0.0],
                    });
                }
                // Edge（境界大梁）: bl.elem に解決済みの ElemId が入る。
                LoadTarget::Edge(_) => {
                    let Some(elem) = self.model.elements.iter().find(|e| e.id == bl.elem) else {
                        continue;
                    };
                    let l = elem_geometric_length(elem, &self.model);
                    if l <= 1e-9 {
                        continue;
                    }
                    emit_shape(&mut member, elem.id, 0.0, l, false, &bl.shape);
                }
                // Span（節点対）: 実部材化小梁（解決済み ElemId）はそのまま全長へ。
                // 実梁が無い節点対（二次部材（小梁）上の辺・大梁の中間区間）は
                // 主架構へ変換する:
                // 1. 両節点が同一の大梁スパン上 → その大梁の**部分区間**分布へ
                // 2. それ以外 → 単純梁の両端反力として節点荷重化
                //    （節点が大梁スパン上なら後段で中間集中荷重（CMQ）へ変換）
                LoadTarget::Span([n0, n1]) => {
                    if let Some(elem) = self.model.elements.iter().find(|e| e.id == bl.elem) {
                        let l = elem_geometric_length(elem, &self.model);
                        if l > 1e-9 {
                            emit_shape(&mut member, elem.id, 0.0, l, false, &bl.shape);
                        }
                        continue;
                    }
                    let (Some(node0), Some(node1)) = (
                        self.model.nodes.get(n0.index()),
                        self.model.nodes.get(n1.index()),
                    ) else {
                        continue;
                    };
                    let hit0 = beam_span_position(&self.model, node0.coord, SPAN_TOL_MM);
                    let hit1 = beam_span_position(&self.model, node1.coord, SPAN_TOL_MM);
                    if let (Some((e0, a0)), Some((e1, a1))) = (hit0, hit1) {
                        if e0 == e1 {
                            // 大梁の中間区間に載る辺: 部分区間の分布荷重へ。
                            let start = a0.min(a1);
                            let len_e = (a1 - a0).abs();
                            if len_e > 1e-9 {
                                emit_shape(&mut member, e0, start, len_e, a0 > a1, &bl.shape);
                            }
                            continue;
                        }
                    }
                    // 二次部材（小梁）上の辺など: 単純梁反力として両端節点へ。
                    let len = {
                        let (a, b) = (node0.coord, node1.coord);
                        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2))
                            .sqrt()
                    };
                    let (r0, r1) = simple_reactions(&bl.shape, len);
                    for (n, r) in [(n0, r0), (n1, r1)] {
                        if r.abs() > 1e-9 {
                            nodal.push(NodalLoad {
                                node: n,
                                values: [0.0, 0.0, -r, 0.0, 0.0, 0.0],
                            });
                        }
                    }
                }
            }
        }

        // 要素が接続しない節点への荷重（小梁反力・小梁支持点など）を、載っている
        // 大梁の中間集中荷重（CMQ）へ変換する（二次部材の荷重伝達）。
        let (nodal, extra_member) =
            squid_n_load::secondary::resolve_nodal_to_primary(&self.model, nodal, SPAN_TOL_MM);
        member.extend(extra_member);

        (nodal, member)
    }

    /// CMQ 図（ビューア）の描画ソース: `self.beam_loads`（`refresh_beam_loads` 適用後の
    /// 固定荷重（DL）分配）を `slab_load_case_content` で主架構の部材荷重へ変換し、
    /// `MemberLoad` 側だけを返す（`NodalLoad`＝柱節点などは CMQ 図の描画対象外）。
    /// これにより、小梁の点反力・大梁中間区間の部分分布荷重が主架構の大梁へ集約された
    /// 状態（大梁1本=部材荷重の集合）で描画でき、実部材化された小梁やスラブは
    /// 自然に描画対象から外れる（小梁・柱には `MemberLoad` が付かないため）。
    /// 呼び出し元（ビューア）が gui フィーチャ限定のため、gui 無効時は dead_code になる。
    #[cfg(feature = "gui")]
    pub(crate) fn cmq_display_member_loads(&self) -> Vec<squid_n_core::model::MemberLoad> {
        self.slab_load_case_content(&self.beam_loads).1
    }

    /// 重力系の標準荷重ケース（DL・LL(架構用)・LL(地震用)）へ自動計算値を同期する
    /// （レビュー §1.1: 面荷重→大梁分配の結果を応力解析へ接続する最重要修正／
    /// 床 Phase A-2: 令85条1項の DL/LL 分離／照合レビュー: ③梁自重・②壁荷重の
    /// CMoQ 経路を長期応力解析へ接続）。
    ///
    /// - 「DL」（kind=Dead・[`DL_CASE_NAME`]）: スラブの `loads`（仕上げ等の
    ///   固定荷重）の分配と、躯体自重（柱梁・壁・ダンパー・フレーム外雑壁。
    ///   `squid_n_load::self_weight::self_weight_case_content`）の合算。
    /// - 「LL(架構用)」（kind=Live）: スラブ用途（`SlabUsage`）から令別表第1 の
    ///   **骨組用**積載（LL）を分配（長期骨組解析用。用途未設定のスラブは寄与 0）。
    /// - 「LL(地震用)」（kind=LiveSeismic）: スラブ用途から令別表第1 の地震用積載を
    ///   分配。`gravity_cases_for_seismic_weight` が LiveSeismic を優先採用するため、
    ///   地震用重量にはこの（骨組用より小さい）地震用値が算入される（令85条1項）。
    ///
    /// 各ケースについて現在の自動計算値を求め、既存ケースの内容と一致するなら
    /// 何もしない（undo 履歴・stale フラグを汚さない）。差分があれば
    /// `SyncSlabLoadsToCase`（全置換、undo 対応）を発行する。
    /// 対応するケースが無く内容も空の場合は空ケースを作らない。
    ///
    /// DL に自重を含めるため、階の自動生成（地震用重量）では密度からの自重直接
    /// 算入を無効にして二重計上を防ぐ（`density_self_weight_for_stories`）。
    ///
    /// 解析実行系（`sync_auto_load_cases_action` 経由）・`generate_stories_action`
    /// の入口で毎回呼ぶことを想定した冪等な同期アクション。
    pub fn sync_gravity_load_cases_action(&mut self) {
        use squid_n_core::model::{LoadCaseKind, LoadPurpose};

        // 節点対→実 Beam 索引・床格子の単位（w=1）支点反力を 1 回だけ構築し、
        // DL/LL(架構用)/LL(地震用) の 3 ケースで使い回す（性能。反力は面荷重強度
        // `w` に線形なので格子 FEM を w ごとに解き直す必要はない）。
        let beam_map = self.beam_elem_map();
        let unit_reactions = self.slab_grillage_unit_reactions(&beam_map);

        // CMQ 図表示・従来互換のため `self.beam_loads` には固定荷重（DL）分配を
        // 格納する（`refresh_beam_loads` と同じ結果。ここでは共有済みの
        // `beam_map`/`unit_reactions` を使い回すため直接 `slab_beam_loads_with` を呼ぶ）。
        self.beam_loads =
            self.slab_beam_loads_with(|slab| slab.dead_intensity(), &unit_reactions, &beam_map);

        // DL（固定荷重）: スラブ分配（`self.beam_loads` は上で dead_intensity 分配済み）
        // ＋躯体自重。自重には二次部材（小梁・間柱）の分（支持点への節点荷重）が
        // 含まれるため、要素が接続しない節点への荷重を大梁の中間集中荷重（CMQ）へ
        // 変換してから同期する。
        let dl_beam_loads = self.beam_loads.clone();
        let (mut dl_nodal, mut dl_member) = self.slab_load_case_content(&dl_beam_loads);
        let load_cfg = self.model.load_cfg.clone().unwrap_or_default();
        let (sw_nodal, sw_member) =
            squid_n_load::self_weight::self_weight_case_content(&self.model, &load_cfg);
        dl_nodal.extend(sw_nodal);
        dl_member.extend(sw_member);
        let (dl_nodal, extra_member) = squid_n_load::secondary::resolve_nodal_to_primary(
            &self.model,
            dl_nodal,
            squid_n_load::secondary::SPAN_TOL_MM,
        );
        dl_member.extend(extra_member);
        self.sync_one_auto_case(DL_CASE_NAME, LoadCaseKind::Dead, dl_nodal, dl_member);

        // LL（積載荷重・骨組用）: スラブ用途から令別表第1 の骨組用積載を分配。
        let ll_beam_loads = self.slab_beam_loads_with(
            |slab| slab.live_intensity(LoadPurpose::Frame),
            &unit_reactions,
            &beam_map,
        );
        let (ll_nodal, ll_member) = self.slab_load_case_content(&ll_beam_loads);
        self.sync_one_auto_case(LL_FRAME_CASE_NAME, LoadCaseKind::Live, ll_nodal, ll_member);

        // LL（積載荷重・地震用）: スラブ用途から令別表第1 の地震用積載を分配。
        let ls_beam_loads = self.slab_beam_loads_with(
            |slab| slab.live_intensity(LoadPurpose::Seismic),
            &unit_reactions,
            &beam_map,
        );
        let (ls_nodal, ls_member) = self.slab_load_case_content(&ls_beam_loads);
        self.sync_one_auto_case(
            LL_SEISMIC_CASE_NAME,
            LoadCaseKind::LiveSeismic,
            ls_nodal,
            ls_member,
        );
    }

    /// 地震荷重(Ai分布)の設計用固有周期 T[s] を、暗黙の解析なしで決定する。
    ///
    /// - `AiMode::Approx`: 略算式 T = h(0.02+0.01α)（令88条・昭和55年建設省
    ///   告示第1793号）。即時計算で解析は不要。
    /// - `AiMode::SemiPrecise`: 明示実行済みの固有値解析結果
    ///   （`self.results` の `modal`）の1次周期（`ModalResult::period[0]`）を
    ///   再利用する。固有値解析が未実行（`results` が無い・`modal` が無い・
    ///   `period` が空のいずれか）の場合は `Err`（実行を促す日本語メッセージ）。
    ///
    /// `Analysis::prepare`（剛性行列組立+Cholesky分解）や固有値解析を新たに
    /// 実行することはない（暗黙の重い解析を避けるための入口）。
    fn design_seismic_period(&self) -> Result<f64, String> {
        match self.analysis_cfg.ai_mode {
            AiMode::Approx => {
                let height_m = squid_n_solver::analysis::building_height_mm(&self.model) / 1000.0;
                let steel_ratio = squid_n_solver::analysis::steel_height_ratio(&self.model);
                Ok(squid_n_load::ai::approx_t(height_m, steel_ratio))
            }
            AiMode::SemiPrecise => self
                .results
                .as_ref()
                .and_then(|r| r.modal.as_ref())
                .and_then(|m| m.period.first().copied())
                .ok_or_else(|| {
                    "精算周期(固有値解析)が選択されていますが固有値解析が未実行です。\
                     解析タブの固有値解析を先に実行してください\
                     (EX/EY の地震荷重は更新されません)。"
                        .to_string()
                }),
        }
    }

    /// 地震荷重の標準ケース（EX・EY、kind=Seismic）へ Ai 分布の水平力を同期する。
    ///
    /// 階（`model.stories`）が定義されている場合に、地震静的解析と同じ載荷
    /// （`build_seismic_load_case_from_model`。方向・Ai算定法・Z・地盤種別・C0 は
    /// `analysis_cfg`）を EX/EY ケースへ書き込む。これにより荷重組合せ
    /// （G+P±K など）が EX/EY を参照して解析できる。
    ///
    /// 設計用固有周期 T は `design_seismic_period` で決定する（`Analysis::prepare`
    /// を要しないモデル単独版 `build_seismic_load_case_from_model` を使うため、
    /// 本関数自体は剛性行列組立や固有値解析を一切行わない）。X・Y 双方向で T を
    /// 共有するため `design_seismic_period` の呼び出しは 1 回のみ。
    ///
    /// 階が未定義・地震荷重が構築できない場合は何もしない（既存の EX/EY
    /// ケースは変更しない。組合せ実行時に空の地震ケースを参照していれば
    /// エラーで案内する）。SemiPrecise で固有値解析が未実行の場合も同様に
    /// 何もせず、代わりに `last_notice` へ実行を促すメッセージを設定する
    /// （`last_error` とは別枠。解析自体は継続してよい注意事項のため）。
    /// 冪等な同期アクション（`sync_gravity_load_cases_action` と同じ規約）。
    pub fn sync_seismic_load_cases_action(&mut self) {
        use squid_n_core::model::LoadCaseKind;
        if self.model.stories.is_empty() {
            return;
        }
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => {
                self.last_notice = Some(msg);
                return;
            }
        };
        let built: Vec<(&'static str, squid_n_core::model::LoadCase)> =
            [(SeismicDir::X, EX_CASE_NAME), (SeismicDir::Y, EY_CASE_NAME)]
                .into_iter()
                .filter_map(|(dir, name)| {
                    let cfg = squid_n_solver::analysis::SeismicCfg {
                        dir,
                        mode: self.analysis_cfg.ai_mode,
                        z: self.analysis_cfg.z,
                        soil: self.analysis_cfg.soil,
                        c0: self.analysis_cfg.c0,
                    };
                    squid_n_solver::analysis::build_seismic_load_case_from_model(
                        &self.model,
                        cfg,
                        t,
                    )
                    .ok()
                    .map(|lc| (name, lc))
                })
                .collect();
        for (name, lc) in built {
            self.sync_one_auto_case(name, LoadCaseKind::Seismic, lc.nodal, lc.member);
        }
    }

    /// `sync_auto_load_cases_action` が同期の要否判定に使うハッシュを計算する。
    ///
    /// 荷重同期（DL/LL/EX/EY）の結果に影響し得る入力をすべて含める:
    /// モデル本体（`bincode` でシリアライズしてハッシュ）、地震荷重の
    /// Ai算定法（`ai_mode`）・地域係数 Z・地盤種別・標準せん断力係数 C0、
    /// および SemiPrecise 時は `design_seismic_period` の値（算定できた場合のみ。
    /// `to_bits()` でビット列化してハッシュ。固有値解析が未実行で `Err` の場合は
    /// 含めない＝モデル・設定が同じなら「未実行」状態も同一ハッシュに畳み込む）。
    fn compute_auto_load_sync_hash(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        if let Ok(bytes) = bincode::serialize(&self.model) {
            bytes.hash(&mut hasher);
        }
        std::mem::discriminant(&self.analysis_cfg.ai_mode).hash(&mut hasher);
        self.analysis_cfg.z.to_bits().hash(&mut hasher);
        (self.analysis_cfg.soil as u8).hash(&mut hasher);
        self.analysis_cfg.c0.to_bits().hash(&mut hasher);
        if matches!(self.analysis_cfg.ai_mode, AiMode::SemiPrecise) {
            if let Ok(t) = self.design_seismic_period() {
                t.to_bits().hash(&mut hasher);
            }
        }
        hasher.finish()
    }

    /// 自重・積載・地震荷重の自動同期（`sync_gravity_load_cases_action` ・
    /// `sync_seismic_load_cases_action`）をまとめて行う、解析実行系
    /// （`run_linear_static`/`run_combination`/`run_all_combinations`/
    /// `run_seismic`）共通の入口。
    ///
    /// モデルが交差小梁スラブを含む場合、床荷重分配（DL・LL(架構用)・
    /// LL(地震用)の3系統×床格子サブFEM解析）は重い処理になり得るため、
    /// 前回の同期時からモデル・関連設定（`compute_auto_load_sync_hash`）が
    /// 一切変わっていなければ丸ごとスキップする。
    ///
    /// 手順:
    /// 1. `apply_rigid_zones_for_analysis`（冪等・軽量なので常に実行。
    ///    剛域の反映は地震荷重の同期より先に行う。SemiPrecise の固有周期算定が
    ///    剛域込みの剛性を用いるようにするため）。
    /// 2. 現在のハッシュを計算し、前回保存したハッシュと一致すればスキップ。
    /// 3. 不一致なら `sync_gravity_load_cases_action` → `sync_seismic_load_cases_action`
    ///    の順で実行する。
    /// 4. 同期後（荷重ケースの内容が書き換わった後）のモデルで再度ハッシュを
    ///    計算して保存する（同期前のハッシュを保存すると、次回呼び出しで
    ///    「同期していないのに一致」と誤判定するため、必ず同期後の状態で保存する）。
    pub fn sync_auto_load_cases_action(&mut self) {
        self.apply_rigid_zones_for_analysis();
        let current = self.compute_auto_load_sync_hash();
        if self.auto_load_sync_hash == Some(current) {
            return;
        }
        self.sync_gravity_load_cases_action();
        self.sync_seismic_load_cases_action();
        self.auto_load_sync_hash = Some(self.compute_auto_load_sync_hash());
    }

    /// 名前付き荷重ケースを指定の `kind`・内容へ冪等に同期する
    /// （`sync_gravity_load_cases_action`／`sync_seismic_load_cases_action` の
    /// 各ケース同期の共通処理）。既存ケースの内容と一致すれば何もしない。
    fn sync_one_auto_case(
        &mut self,
        name: &str,
        kind: squid_n_core::model::LoadCaseKind,
        nodal: Vec<squid_n_core::model::NodalLoad>,
        member: Vec<squid_n_core::model::MemberLoad>,
    ) {
        let existing = self.model.load_cases.iter().find(|lc| lc.name == name);
        let needs_create = existing.is_none() && !(nodal.is_empty() && member.is_empty());
        let needs_update = existing
            .map(|lc| lc.kind != kind || lc.nodal != nodal || lc.member != member)
            .unwrap_or(false);
        if !needs_create && !needs_update {
            return;
        }

        self.undo.run(
            &mut self.model,
            Box::new(squid_n_edit::SyncSlabLoadsToCase {
                name: name.to_string(),
                kind,
                nodal,
                member,
            }),
        );
        self.staleness.mark_edited();
    }

    /// 組合せが参照する空の地震荷重ケース（kind=Seismic・内容なし）の名前を返す。
    /// 空の地震ケースを含む組合せをそのまま解くと地震項が黙って 0 になるため、
    /// 実行前のガードに使う（`run_combination`/`run_all_combinations`）。
    fn empty_seismic_case_in_combo(
        &self,
        combo: &squid_n_core::model::LoadCombination,
    ) -> Option<String> {
        combo.terms.iter().find_map(|(id, _)| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.id == *id)
                .filter(|lc| {
                    lc.kind == squid_n_core::model::LoadCaseKind::Seismic
                        && lc.nodal.is_empty()
                        && lc.member.is_empty()
                })
                .map(|lc| lc.name.clone())
        })
    }
}

/// 長期（重力）ケースの部材荷重から、各部材を単純梁支持とした場合の端部
/// せん断力 Q0 [N] を算定する。
///
/// せん断補強筋に MK785/SPR785/SPR685 を使用した部材の終局余裕率では、
/// QL 控除を `QL=Q0` と読み替える（各製品の技術評定の規定。
/// [`squid_n_design_jp::ultimate::MemberDemand`] の `q_simple`）。荷重は部材軸
/// 直交成分の大きさで評価し、Q0 は単純梁の両端反力の大きい方とする。
/// 対象ケースは QL と同じ先頭重力ケース（そのケースに載る部材荷重のみ集計）。
fn simple_beam_q0_by_elem(
    model: &squid_n_core::model::Model,
    lc: LoadCaseId,
) -> std::collections::HashMap<ElemId, f64> {
    use squid_n_core::model::MemberLoadKind;
    let mut acc: std::collections::HashMap<ElemId, (f64, f64)> = Default::default();
    let Some(case) = model.load_cases.iter().find(|c| c.id == lc) else {
        return Default::default();
    };
    for ml in &case.member {
        let Some(elem) = model.elements.iter().find(|e| e.id == ml.elem) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let (Some(n0), Some(n1)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[elem.nodes.len() - 1].index()),
        ) else {
            continue;
        };
        let dx = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let l = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
        if l <= 0.0 {
            continue;
        }
        let e = [dx[0] / l, dx[1] / l, dx[2] / l];
        let dn = (ml.dir[0] * ml.dir[0] + ml.dir[1] * ml.dir[1] + ml.dir[2] * ml.dir[2]).sqrt();
        if dn <= 0.0 {
            continue;
        }
        let d = [ml.dir[0] / dn, ml.dir[1] / dn, ml.dir[2] / dn];
        // 部材軸直交成分の大きさ（重力荷重×水平梁なら 1.0）。
        let ax = d[0] * e[0] + d[1] * e[1] + d[2] * e[2];
        let trans = (1.0 - ax * ax).max(0.0).sqrt();
        if trans <= 1e-12 {
            continue;
        }
        let (w_total, x_bar) = match ml.kind {
            MemberLoadKind::Point { a, p } => (p.abs(), a.clamp(0.0, l)),
            MemberLoadKind::Distributed { a, b, w1, w2 } => {
                let (a, b) = (a.clamp(0.0, l), b.clamp(0.0, l));
                if b <= a {
                    continue;
                }
                let w_sum = w1 + w2;
                let total = w_sum / 2.0 * (b - a);
                // 台形分布の重心（w_sum≈0 の反対称分布は区間中央で代表）。
                let xb = if w_sum.abs() > 1e-12 {
                    a + (b - a) * (w1 + 2.0 * w2) / (3.0 * w_sum)
                } else {
                    (a + b) / 2.0
                };
                (total.abs(), xb)
            }
        };
        let entry = acc.entry(ml.elem).or_insert((0.0, 0.0));
        entry.0 += trans * w_total * (l - x_bar) / l; // 単純梁反力 Ri
        entry.1 += trans * w_total * x_bar / l; // 単純梁反力 Rj
    }
    acc.into_iter()
        .map(|(k, (ri, rj))| (k, ri.max(rj)))
        .collect()
}
