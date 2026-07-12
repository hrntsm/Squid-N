//! `App` のアクション（解析実行・ファイル入出力・モデル操作）メソッド。

use super::*;

impl App {
    /// モデルを丸ごと差し替える（新規作成・サンプル読込・ファイル読込で共用）。
    /// undo 履歴・結果・選択・stale 状態をすべてリセットする。
    pub fn load_model(&mut self, model: squid_n_core::model::Model) {
        self.model = model;
        self.results = None;
        self.selection = Selection::default();
        self.undo = UndoStack::new();
        self.nav = Navigator::default();
        self.last_static = None;
        self.last_error = None;
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
    pub fn import_stbridge_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        let xml = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                self.last_error = Some(format!("ST-Bridge読込エラー: {}", e));
                return;
            }
        };
        match squid_n_io::stbridge::import_stbridge(&xml) {
            Ok(model) => {
                if let Err(e) = model.validate() {
                    self.last_error = Some(format!("ST-Bridge読込モデルの検証エラー: {:?}", e));
                    return;
                }
                self.load_model(model);
                self.project_path = None;
            }
            Err(e) => self.last_error = Some(format!("ST-Bridge読込エラー: {}", e)),
        }
    }

    /// モデルを ST-Bridge（XML, サブセット）として指定パスへ書き出す。
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

    /// T3: 線形静的解析を実行し、結果を `self.results` に格納する。
    /// 指定した荷重ケースが存在しない場合はエラーメッセージをセット。
    ///
    /// 解析準備前にスラブ荷重を「床荷重(自動)」ケースへ同期する（レビュー §1.1）。
    pub fn run_linear_static(&mut self, lc: LoadCaseId) {
        self.last_error = None;
        self.sync_slab_loads_action();
        self.apply_rigid_zones_for_analysis();
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.linear_static(lc) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
                    let key = StaticCaseKey::User(lc);
                    bundle.statics.retain(|(id, _)| *id != key);
                    bundle.statics.push((key, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Case(key));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("線形静的解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// T7: 荷重組合せ解析を実行し、結果を `bundle.combos` に格納する。
    /// 指定インデックスの荷重組合せが存在しない場合はエラーメッセージをセット。
    ///
    /// 解析準備前にスラブ荷重を「床荷重(自動)」ケースへ同期する（レビュー §1.1）。
    pub fn run_combination(&mut self, index: usize) {
        self.last_error = None;
        self.sync_slab_loads_action();
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.last_error = Some(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        self.apply_rigid_zones_for_analysis();
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.linear_combination(&combo) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
                    // StaticKey::Combo は bundle.combos 上の位置を指す規約
                    // （current_static・ナビゲータと共有）。再実行時は既存位置を
                    // その場で差し替え、他の組合せ結果のキーを無効化しない。
                    let pos = match bundle
                        .combos
                        .iter()
                        .position(|(name, _)| *name == combo.name)
                    {
                        Some(pos) => {
                            bundle.combos[pos].1 = res;
                            pos
                        }
                        None => {
                            bundle.combos.push((combo.name.clone(), res));
                            bundle.combos.len() - 1
                        }
                    };
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Combo(pos));
                    self.staleness.mark_fresh();
                    // 荷重継続性区分（長期/短期）は組合せ内容から自動判定する
                    // （マニュアル「荷重の組合せ」: G+P=長期、地震・積雪・風入り=短期）。
                    self.design_term = if squid_n_load::combo::is_short_term_combo(&combo.name) {
                        LoadTerm::Short
                    } else {
                        LoadTerm::Long
                    };
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("荷重組合せ解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
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

        // T(1 次周期): 固有値解析があればそれを使用、なければ略算式。
        let t = self
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.period.first().copied())
            .unwrap_or_else(|| {
                let height_m = self
                    .model
                    .stories
                    .last()
                    .map(|s| s.elevation)
                    .unwrap_or(0.0)
                    / 1000.0;
                squid_n_load::ai::approx_t(height_m, 0.0)
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
                        // （RESP-D マニュアル 04 断面検定「幅厚比の検討」）。
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

    /// T3: 固有値解析を実行し、結果を `self.results` に格納する。
    pub fn run_eigen(&mut self, n_modes: usize) {
        self.last_error = None;
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
    /// 先頭ケース）の荷重ケースの鉛直下向き荷重＋自重を用いる（レビュー §1.7）。
    /// 先立ってスラブ荷重を「床荷重(自動)」ケースへ同期する（レビュー §1.1）ため、
    /// 面荷重も地震用重量に反映される。
    pub fn generate_stories_action(&mut self) {
        self.last_error = None;
        self.sync_slab_loads_action();
        let gravity_lcs = gravity_cases_for_seismic_weight(&self.model);
        match squid_n_load::story_gen::generate_stories_multi(&self.model, &gravity_lcs) {
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
            }
            Err(e) => self.last_error = Some(format!("階の自動生成エラー: {}", e)),
        }
    }

    /// T3: 地震静的解析（Ai一気通貫）を実行し、結果を `self.results` に格納する。
    /// 方向・Ai算定法・Z・地盤種別・C0 は `analysis_cfg` を用いる。
    /// 結果は `StaticCaseKey::Seismic(dir)` に格納するため、X/Y 双方の地震静的結果
    /// および任意のユーザー荷重ケースの結果と衝突せず共存できる。
    pub fn run_seismic(&mut self, dir: SeismicDir) {
        self.last_error = None;
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        self.apply_rigid_zones_for_analysis();
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.seismic_static_with(cfg) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
                    let key = StaticCaseKey::Seismic(dir);
                    bundle.statics.retain(|(id, _)| *id != key);
                    bundle.statics.push((key, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Case(key));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("地震解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// 風荷重の静的解析を実行し、結果を `StaticCaseKey::Wind(dir)` に格納する
    /// （`run_seismic` と同じパターン。X/Y 双方の結果および他の静的結果と共存できる）。
    /// 基準風速・地表面粗度区分・パラペット高さは `analysis_cfg` を用いる。
    pub fn run_wind(&mut self, dir: SeismicDir) {
        self.last_error = None;
        let cfg = squid_n_solver::analysis::WindStaticCfg {
            dir,
            v0: self.analysis_cfg.v0,
            roughness: self.analysis_cfg.roughness,
            cpi: 0.0,
            parapet_mm: self.analysis_cfg.parapet_mm,
        };
        self.apply_rigid_zones_for_analysis();
        match Analysis::prepare(&self.model) {
            Ok(analysis) => match analysis.wind_static(cfg) {
                Ok(res) => {
                    let member_forces = res.member_forces.clone();
                    let mut bundle = self.results.take().unwrap_or_default();
                    let key = StaticCaseKey::Wind(dir);
                    bundle.statics.retain(|(id, _)| *id != key);
                    bundle.statics.push((key, res));
                    bundle.member_forces = member_forces;
                    self.results = Some(bundle);
                    self.last_static = Some(StaticKey::Case(key));
                    self.staleness.mark_fresh();
                    self.run_design_check();
                }
                Err(e) => self.last_error = Some(format!("風荷重解析エラー: {:?}", e)),
            },
            Err(e) => self.last_error = Some(format!("解析準備エラー: {:?}", e)),
        }
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
    fn compute_time_history(
        model: squid_n_core::model::Model,
        cfg: AnalysisSettings,
        wave: squid_n_solver::timehistory::GroundMotion,
    ) -> Result<squid_n_solver::timehistory::ResponseResult, String> {
        let mut model = model;
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
            },
            ThDir::Y => {
                let n = accel.len();
                squid_n_solver::timehistory::GroundMotion {
                    dt,
                    accel_x: vec![0.0; n],
                    accel_y: Some(accel),
                }
            }
            ThDir::Xy => squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: accel.clone(),
                accel_y: Some(accel),
            },
        }
    }

    /// T7: 解析結果の member_forces から検定結果を生成する。
    /// 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力に対し、
    /// 材種・部材種別に応じた検定を適用する（RESP-D マニュアル 04 断面検定準拠）。
    /// 節点芯は剛域が有る場合は検定対象外（節点芯の応力をそのまま使わない、
    /// 設計書 §6.2.3）。
    ///
    /// - 部材種別は部材軸の鉛直成分から判定（柱/梁/ブレース）。
    /// - せん断スパン比 M/(Q·d) 用の代表値は、マニュアルの規定
    ///   「モーメントが最大となる検定位置の値を採用」に従い部材単位で求める。
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
        let mut checks: Vec<(ElemId, f64, squid_n_design_jp::CheckResult)> = Vec::new();
        for (elem_id, mf) in &results.member_forces {
            let elem = self.model.elements.iter().find(|e| e.id == *elem_id);
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
            // せん断スパン比 M/(Q·d) の代表値: |Mz| 最大の検定位置の (|M|, |Q|)。
            let shear_span = mf
                .at
                .iter()
                .map(|(_, f)| (f[5].abs(), f[1].abs()))
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
            let lk = if kind == squid_n_design_jp::MemberKind::Column {
                squid_n_design_jp::steel::buckling::steel_column_k(&self.model, elem)
                    .map(|k| k * length)
            } else {
                None
            };
            // 一本部材グループに属する梁は、部材長・端部/中央モーメント・せん断
            // スパン比代表値をグループ合成値に置き換える（RESP-D マニュアル 04
            // 「採用応力 ■一本部材指定時の採用応力」）。
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
            let seismic_qd = long_member_forces
                .and_then(|list| list.iter().find(|(id, _)| id == elem_id))
                .map(|(_, mf_long)| {
                    let face_sum = elem.rigid_zone.face_i + elem.rigid_zone.face_j;
                    let clear_length = match group {
                        // 一本部材は両外端の剛域控除後のグループ内法長。
                        Some(g) => g.clear_length,
                        None if length - face_sum > 0.0 => length - face_sum,
                        None => length,
                    };
                    squid_n_design_jp::SeismicQd {
                        long_at: mf_long.at.clone(),
                        // 割増係数 n（マニュアル: 柱は 1.5 以上）。梁・柱とも 1.5。
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
                lk,
                shear_span,
                rc_damage_control: self.analysis_cfg.rc_damage_control,
                end_moments_z,
                mid_moment_z,
                seismic_qd,
                steel_attr,
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

            let positions = design_positions(elem, length);

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
                // 差し替える（RESP-D マニュアル 04 座屈補剛ブレースの断面検定）。
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
        if let Some(bundle) = self.results.as_mut() {
            bundle.checks = checks;
            bundle.joint_checks = joint_checks;
        }
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
    /// のまま）保持する（部材マッピング不要。`sync_slab_loads_action` が
    /// `NodalLoad` へ変換する。CMQ 図描画側は `elem` で梁を引くため、この番兵は
    /// 単に描画対象外になるだけで安全）。
    pub fn refresh_beam_loads(&mut self) {
        let mut beam_loads = Vec::new();
        for slab in &self.model.slabs {
            let n = slab.boundary.len();
            if n < 3 {
                continue;
            }
            for mut bl in squid_n_load::floor::distribute_slab(&self.model, slab) {
                match bl.target {
                    squid_n_load::floor::LoadTarget::Node(_) => {
                        beam_loads.push(bl);
                    }
                    squid_n_load::floor::LoadTarget::Edge(k) => {
                        if k >= n {
                            continue;
                        }
                        let n0 = slab.boundary[k];
                        let n1 = slab.boundary[(k + 1) % n];
                        let found = self.model.elements.iter().find(|e| {
                            e.kind == squid_n_core::model::ElementKind::Beam
                                && e.nodes.len() == 2
                                && ((e.nodes[0] == n0 && e.nodes[1] == n1)
                                    || (e.nodes[0] == n1 && e.nodes[1] == n0))
                        });
                        let Some(elem) = found else { continue };
                        bl.elem = elem.id;
                        beam_loads.push(bl);
                    }
                }
            }
        }
        self.beam_loads = beam_loads;
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
    ) -> (
        Vec<squid_n_core::model::NodalLoad>,
        Vec<squid_n_core::model::MemberLoad>,
    ) {
        use squid_n_core::model::{MemberLoad, MemberLoadKind, NodalLoad};
        use squid_n_load::floor::{LoadShape, LoadTarget};

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

        for bl in &self.beam_loads {
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
                LoadTarget::Edge(_) => {
                    let Some(elem) = self.model.elements.iter().find(|e| e.id == bl.elem) else {
                        continue;
                    };
                    let l = elem_geometric_length(elem, &self.model);
                    if l <= 1e-9 {
                        continue;
                    }
                    match bl.shape {
                        LoadShape::Uniform { w } => {
                            push_dist(&mut member, elem.id, 0.0, l, w, w);
                        }
                        LoadShape::Triangle { w0 } => {
                            let mid = l / 2.0;
                            push_dist(&mut member, elem.id, 0.0, mid, 0.0, w0);
                            push_dist(&mut member, elem.id, mid, l, w0, 0.0);
                        }
                        LoadShape::Trapezoid { w0, a, b } => {
                            push_dist(&mut member, elem.id, 0.0, a, 0.0, w0);
                            push_dist(&mut member, elem.id, a, a + b, w0, w0);
                            push_dist(&mut member, elem.id, a + b, l, w0, 0.0);
                        }
                        LoadShape::Point { p, x } => {
                            member.push(MemberLoad {
                                elem: elem.id,
                                dir: DIR,
                                kind: MemberLoadKind::Point { a: x, p },
                            });
                        }
                    }
                }
            }
        }

        (nodal, member)
    }

    /// スラブ荷重を専用の荷重ケース「床荷重(自動)」（kind=Dead）へ同期する
    /// （レビュー §1.1: 面荷重→大梁分配の結果を応力解析へ接続する最重要修正）。
    ///
    /// `refresh_beam_loads` → `slab_load_case_content` で現在のスラブ荷重を
    /// 計算し、既存の「床荷重(自動)」ケースの内容と一致するなら何もしない
    /// （undo 履歴・stale フラグを汚さない）。差分があれば
    /// `SyncSlabLoadsToCase`（全置換、undo 対応）を発行する。
    /// スラブが無く既存ケースも無い場合は空ケースを作らない。
    ///
    /// 解析実行系（`run_linear_static`/`run_combination`）・`generate_stories_action`
    /// の入口で毎回呼ぶことを想定した冪等な同期アクション。
    pub fn sync_slab_loads_action(&mut self) {
        self.refresh_beam_loads();
        let (nodal, member) = self.slab_load_case_content();

        let existing = self
            .model
            .load_cases
            .iter()
            .find(|lc| lc.name == SLAB_AUTO_LOAD_CASE_NAME);
        let needs_create = existing.is_none() && !(nodal.is_empty() && member.is_empty());
        let needs_update = existing
            .map(|lc| {
                lc.kind != squid_n_core::model::LoadCaseKind::Dead
                    || lc.nodal != nodal
                    || lc.member != member
            })
            .unwrap_or(false);
        if !needs_create && !needs_update {
            return;
        }

        self.undo.run(
            &mut self.model,
            Box::new(squid_n_edit::SyncSlabLoadsToCase {
                name: SLAB_AUTO_LOAD_CASE_NAME.to_string(),
                nodal,
                member,
            }),
        );
        self.staleness.mark_edited();
    }
}
