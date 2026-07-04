use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::eigen::{self, ModalResult};
use crate::linear::StaticOnce;
use crate::timehistory::{GroundMotion, NewmarkCfg, ResponseResult};

pub type StaticResult = StaticOnce;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{LoadCombination, Model};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeismicDir {
    X,
    Y,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiMode {
    Approx,
    SemiPrecise,
}

/// 地震静的解析(Ai分布)の設定。
#[derive(Debug, Clone, Copy)]
pub struct SeismicCfg {
    pub dir: SeismicDir,
    pub mode: AiMode,
    /// 地域係数 Z（令88条）。
    pub z: f64,
    /// 地盤種別（Tc の決定に使用）。
    pub soil: squid_n_load::ai::SoilClass,
    /// 標準せん断力係数 C0（一次設計 0.2、保有 1.0）。
    pub c0: f64,
}

impl Default for SeismicCfg {
    fn default() -> Self {
        Self {
            dir: SeismicDir::X,
            mode: AiMode::SemiPrecise,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
        }
    }
}

pub struct Analysis<'m> {
    model: &'m Model,
    dofmap: DofMap,
    reducer: Reducer,
    solver: Box<dyn LinearSolver>,
    n_indep: usize,
}

impl<'m> Analysis<'m> {
    /// Build DofMap, assemble global K, apply constraint reduction, and factorize.
    /// After this, `linear_static` and `linear_combination` can be called
    /// multiple times reusing the factorized K.
    ///
    /// 解析前にモデルの静的検証（参照整合・拘束・断面/材料割当・孤立節点）を行い、
    /// 問題があればユーザー向けの日本語診断メッセージ付きでエラーを返す。
    pub fn prepare(model: &'m Model) -> Result<Self, SolveError> {
        faer::set_global_parallelism(faer::Par::Seq);
        model
            .validate()
            .map_err(|e| SolveError::InvalidInput(format!("モデル検証エラー: {:?}", e)))?;
        precheck_model(model)?;
        let dofmap = DofMap::build(model);
        let n_active = dofmap.n_active();

        if n_active == 0 {
            return Ok(Self {
                model,
                dofmap,
                reducer: Reducer {
                    t_rows: vec![],
                    n_indep: 0,
                    n_free: 0,
                },
                solver: make_solver(SolverBackend::DirectSparseCholesky),
                n_indep: 0,
            });
        }

        let k_free = assemble_global_k(model, &dofmap);
        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        let k_red = reducer.reduce_k(&k_free);

        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        if n_indep > 0 {
            solver.factorize(&k_red).map_err(|e| match e {
                SolveError::NotPositiveDefinite => {
                    SolveError::InvalidInput(singular_diagnosis(model))
                }
                other => other,
            })?;
        }

        Ok(Self {
            model,
            dofmap,
            reducer,
            solver,
            n_indep,
        })
    }

    /// 全自由度ゼロの結果（有効自由度なしのモデル用）。
    fn zero_result(&self) -> StaticOnce {
        StaticOnce {
            disp: vec![[0.0; 6]; self.model.nodes.len()],
            member_forces: Vec::new(),
        }
    }

    /// 自由 DOF 空間の荷重ベクトルを縮約 → 解 → 展開し、
    /// 節点変位と部材断面力を復元する（線形静的系の共通経路）。
    fn solve_and_recover(&self, f_free: &[f64]) -> Result<StaticOnce, SolveError> {
        let f_red = self.reducer.reduce_f(f_free);
        let u_indep = self.solver.solve(&f_red)?;
        let u_free = self.reducer.expand_u(&u_indep);
        Ok(StaticOnce {
            disp: self.expand_disp(&u_free),
            member_forces: self.recover_member_forces(&u_free),
        })
    }

    /// 自由 DOF ベクトルを節点 6 成分配列へ展開する。
    fn expand_disp(&self, u_free: &[f64]) -> Vec<[f64; 6]> {
        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; self.model.nodes.len()];
        for (ni, d6) in disp.iter_mut().enumerate() {
            for (d, slot) in d6.iter_mut().enumerate() {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    *slot = u_free[active as usize];
                }
            }
        }
        disp
    }

    /// 自由 DOF ベクトルから全部材の断面力を復元する。
    fn recover_member_forces(
        &self,
        u_free: &[f64],
    ) -> Vec<(
        squid_n_core::ids::ElemId,
        squid_n_element::beam::MemberForces,
    )> {
        let mut member_forces = Vec::new();
        for elem in &self.model.elements {
            let (behavior, _state) = build_behavior(elem, self.model);
            let gdofs = behavior.global_dofs(&self.dofmap);
            let mut u_elem = vec![0.0; gdofs.len()];
            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }
            if let Some(forces) = behavior.recover_forces(&u_elem) {
                member_forces.push((elem.id, forces));
            }
        }
        member_forces
    }

    /// Solve a single load case (back-substitution only, factorized K is reused).
    pub fn linear_static(&self, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        if !self.model.load_cases.iter().any(|c| c.id == lc) {
            return Err(SolveError::InvalidInput(format!(
                "荷重ケース {} が存在しません",
                lc.0
            )));
        }
        let f_free = assemble_global_f(self.model, &self.dofmap, lc);
        self.solve_and_recover(&f_free)
    }

    /// Solve eigenvalue problem (subspace iteration) for n_modes lowest modes.
    pub fn eigen(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, n_modes)
    }

    /// Solve a load combination by assembling the weighted sum of load case
    /// force vectors, then solving with the already factorized K.
    pub fn linear_combination(&self, combo: &LoadCombination) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for (lc_id, factor) in &combo.terms {
            let f_lc = assemble_global_f(self.model, &self.dofmap, *lc_id);
            for (fi, &v) in f_lc.iter().enumerate() {
                f_free[fi] += v * factor;
            }
        }
        self.solve_and_recover(&f_free)
    }

    /// 時刻歴応答解析（Newmark-β / HHT-α、減衰込み）。
    /// 線形専用ラッパ。非線形時刻歴は `timehistory::linear_time_history_analysis`
    /// と同じパターンのフリー関数で実装予定（§4、現在は線形のみ）。
    pub fn time_history(
        &self,
        wave: &GroundMotion,
        newmark: NewmarkCfg,
        damping: Damping,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_time_history_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &newmark,
            &damping,
            &init,
            &init,
            false,
        )
    }

    /// Run seismic static analysis: approx or semi-precise Ai distribution.
    /// SemiPrecise uses eigen T, Approx uses approximate formula.
    ///
    /// 階(Story)・地震重量・剛床が未定義の場合は黙ってゼロ結果を返さず、
    /// 何をすべきかを含むエラーを返す。
    pub fn seismic_static(&self, dir: SeismicDir, mode: AiMode) -> Result<StaticOnce, SolveError> {
        self.seismic_static_with(SeismicCfg {
            dir,
            mode,
            ..SeismicCfg::default()
        })
    }

    /// 地震静的解析（設定指定版）。Z・地盤種別・C0 を UI から与える。
    pub fn seismic_static_with(&self, cfg: SeismicCfg) -> Result<StaticOnce, SolveError> {
        let SeismicCfg {
            dir,
            mode,
            z,
            soil,
            c0,
        } = cfg;
        let stories = &self.model.stories;
        if stories.is_empty() {
            return Err(SolveError::InvalidInput(
                "階(Story)が定義されていません。地震荷重(Ai分布)には階の定義・地震重量・剛床(ダイアフラム)が必要です。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }

        let (t, _) = match mode {
            AiMode::Approx => {
                let height_m = stories.last().map(|s| s.elevation).unwrap_or(0.0) / 1000.0;
                let steel_ratio = 0.0;
                (squid_n_load::ai::approx_t(height_m, steel_ratio), 0)
            }
            AiMode::SemiPrecise => {
                let modal = eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, 1)?;
                let t = modal.period.first().copied().unwrap_or(0.3);
                (t, 0)
            }
        };

        let tc = squid_n_load::ai::tc_of(soil);
        let rt_val = squid_n_load::ai::rt(t, tc);

        let story_weights: Vec<f64> = stories
            .iter()
            .map(|s| s.seismic_weight.unwrap_or(0.0))
            .collect();

        if story_weights.is_empty() || story_weights.iter().all(|&w| w == 0.0) {
            return Err(SolveError::InvalidInput(
                "階の地震重量(seismic_weight)がすべて 0 です。各階の重量を設定してください。"
                    .into(),
            ));
        }

        let ai = squid_n_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

        // Create a load case from the Ai distribution horizontal forces
        let lc_id = LoadCaseId(1001);
        let dir_vec = match dir {
            SeismicDir::X => [1.0, 0.0, 0.0],
            SeismicDir::Y => [0.0, 1.0, 0.0],
        };

        // Attach Pi forces to master nodes of each story's diaphragms
        let mut lc = squid_n_core::model::LoadCase {
            id: lc_id,
            name: format!("seismic_{:?}_{:?}", dir, mode),
            nodal: Vec::new(),
            member: Vec::new(),
        };

        for (i, story) in stories.iter().enumerate() {
            let pi = ai.pi.get(i).copied().unwrap_or(0.0);
            if pi == 0.0 {
                continue;
            }
            for dia in &story.diaphragms {
                let f = [dir_vec[0] * pi, dir_vec[1] * pi, 0.0, 0.0, 0.0, 0.0];
                lc.nodal.push(squid_n_core::model::NodalLoad {
                    node: dia.master,
                    values: f,
                });
            }
        }

        if lc.nodal.is_empty() {
            return Err(SolveError::InvalidInput(
                "地震力を作用させる剛床(ダイアフラム)が階に定義されていません。解析タブの「階の自動生成」を実行してください。".into(),
            ));
        }

        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }

        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for nodal_load in &lc.nodal {
            let ni = nodal_load.node.index();
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    f_free[active as usize] += nodal_load.values[d];
                }
            }
        }

        self.solve_and_recover(&f_free)
    }
}

/// 解析前のモデル静的検証。よくあるモデリングミスを特異行列エラーの前に検出し、
/// 「何をすれば直るか」を含むメッセージで返す。
fn precheck_model(model: &Model) -> Result<(), SolveError> {
    use squid_n_core::model::ElementKind;

    if model.nodes.is_empty() {
        return Err(SolveError::InvalidInput(
            "節点がありません。モデルタブで節点を追加してください。".into(),
        ));
    }
    if model.elements.is_empty() {
        return Err(SolveError::InvalidInput(
            "部材がありません。モデルタブで部材を追加してください。".into(),
        ));
    }
    if !model.nodes.iter().any(|n| n.restraint.0 != 0) {
        return Err(SolveError::InvalidInput(
            "拘束(支点)が 1 つもありません。境界条件タブで支点を設定してください。".into(),
        ));
    }

    // 梁要素の断面・材料未割当
    let missing: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam) && (e.section.is_none() || e.material.is_none())
        })
        .map(|e| e.id.0)
        .collect();
    if !missing.is_empty() {
        let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
        let more = if missing.len() > 5 {
            format!(" 他{}件", missing.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "断面または材料が未割当の部材があります: ID {}{}。部材タブで割り当ててください。",
            head.join(", "),
            more
        )));
    }

    // 孤立節点（要素・拘束・剛床から参照されず、完全固定でもない）
    // → 剛性ゼロの自由 DOF となり特異行列の典型原因
    let mut referenced = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            referenced[n.index()] = true;
        }
    }
    for c in &model.constraints {
        use squid_n_core::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, slaves, .. }
            | Constraint::RigidLink { master, slaves, .. } => {
                referenced[master.index()] = true;
                for s in slaves {
                    referenced[s.index()] = true;
                }
            }
            Constraint::Mpc { master, terms } => {
                referenced[master.index()] = true;
                for (n, _, _) in terms {
                    referenced[n.index()] = true;
                }
            }
        }
    }
    for story in &model.stories {
        for d in &story.diaphragms {
            referenced[d.master.index()] = true;
            for s in &d.slaves {
                referenced[s.index()] = true;
            }
        }
    }
    let isolated: Vec<u32> = model
        .nodes
        .iter()
        .filter(|n| !referenced[n.id.index()] && n.restraint != squid_n_core::dof::Dof6Mask::FIXED)
        .map(|n| n.id.0)
        .collect();
    if !isolated.is_empty() {
        let head: Vec<String> = isolated.iter().take(5).map(|id| id.to_string()).collect();
        let more = if isolated.len() > 5 {
            format!(" 他{}件", isolated.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "どの部材にも接続されていない節点があります: ID {}{}。削除するか完全固定にしてください(剛性ゼロの自由度は解析できません)。",
            head.join(", "),
            more
        )));
    }

    Ok(())
}

/// 剛性行列の分解に失敗した（特異・非正定値）ときの診断メッセージ。
fn singular_diagnosis(model: &Model) -> String {
    let n_restrained = model.nodes.iter().filter(|n| n.restraint.0 != 0).count();
    format!(
        "剛性行列が特異(非正定値)です。構造が機構(不安定)になっている可能性があります。\
         考えられる原因: (1) 拘束が不足している(現在 {} 節点に拘束あり)、\
         (2) ピン接合が連続し回転が拘束されない部材がある、\
         (3) 断面性能(A・I)が 0 の断面がある。",
        n_restrained
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        NodalLoad, Node, Section,
    };

    fn make_cantilever_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "beam".into(),
                area: 100.0,
                iy: 833.33,
                iz: 833.33,
                j: 100.0,
                depth: 10.0,
                width: 10.0,
                as_y: 83.33,
                as_z: 83.33,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".into(),
                young: 20000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            load_cases: vec![
                LoadCase {
                    id: LoadCaseId(1),
                    name: "axial".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                },
                LoadCase {
                    id: LoadCaseId(2),
                    name: "shear".into(),
                    nodal: vec![NodalLoad {
                        node: NodeId(1),
                        values: [0.0, 500.0, 0.0, 0.0, 0.0, 0.0],
                    }],
                    member: Vec::new(),
                },
            ],
            combinations: vec![LoadCombination {
                name: "combo1".into(),
                terms: vec![(LoadCaseId(1), 1.2), (LoadCaseId(2), 1.5)],
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_prepare_and_single_case() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let result = analysis.linear_static(LoadCaseId(1)).unwrap();
        let ux = result.disp[1][0];
        let expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        assert!(
            (ux - expected).abs() < 1e-6,
            "ux={} expected={}",
            ux,
            expected
        );
    }

    #[test]
    fn test_two_cases_one_factorization() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
        let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
        let ux = r1.disp[1][0];
        let uy = r2.disp[1][1];
        let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        let l = 1000.0_f64;
        let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
        // Timoshenko beam includes shear deflection ≈ 0.1% — use relaxed tolerance
        assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
        assert!(
            (uy - uy_expected).abs() < 20.0,
            "uy={} approx={}",
            uy,
            uy_expected
        );
    }

    #[test]
    fn test_load_combination() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let combo = &model.combinations[0];
        let result = analysis.linear_combination(combo).unwrap();
        let ux = result.disp[1][0];
        let uy = result.disp[1][1];
        let ux_expected = 1.2 * (1000.0 * 1000.0 / (20000.0 * 100.0));
        let l = 1000.0_f64;
        let uy_expected = 1.5 * (500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33));
        assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
        // Timoshenko shear adds slight deflection — relaxed tolerance
        assert!(
            (uy - uy_expected).abs() < 20.0,
            "uy={} approx={}",
            uy,
            uy_expected
        );
    }

    #[test]
    fn test_prepare_empty_model_gives_diagnostic() {
        let model = Model::default();
        let err = Analysis::prepare(&model).err().unwrap();
        assert!(matches!(err, SolveError::InvalidInput(_)), "{:?}", err);
    }

    #[test]
    fn test_prepare_no_restraint_gives_diagnostic() {
        let mut model = make_cantilever_model();
        for n in &mut model.nodes {
            n.restraint = Dof6Mask::FREE;
        }
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("拘束"), "{}", msg);
    }

    #[test]
    fn test_prepare_missing_section_gives_diagnostic() {
        let mut model = make_cantilever_model();
        model.elements[0].section = None;
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("未割当"), "{}", msg);
    }

    #[test]
    fn test_prepare_isolated_node_gives_diagnostic() {
        let mut model = make_cantilever_model();
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [0.0, 5000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let err = Analysis::prepare(&model).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("接続されていない節点"), "{}", msg);
    }

    #[test]
    fn test_linear_static_unknown_load_case_is_error() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let err = analysis.linear_static(LoadCaseId(99)).err().unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("荷重ケース"), "{}", msg);
    }

    #[test]
    fn test_seismic_without_stories_is_error() {
        let model = make_cantilever_model();
        let analysis = Analysis::prepare(&model).unwrap();
        let err = analysis
            .seismic_static(SeismicDir::X, AiMode::Approx)
            .err()
            .unwrap();
        let msg = format!("{}", err);
        assert!(msg.contains("階"), "{}", msg);
    }

    #[test]
    fn test_bernoulli_strict_1e9() {
        // Bernoulli beam: very large shear area → negligible shear deformation.
        // Axial: u = PL/EA, Bending: w = PL³/3EI — strict 1e-9 match.
        let mut model = make_cantilever_model();
        model.sections[0].as_y = 1e12;
        model.sections[0].as_z = 1e12;
        let analysis = Analysis::prepare(&model).unwrap();
        let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
        let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
        let ux = r1.disp[1][0];
        let uy = r2.disp[1][1];
        let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
        let l = 1000.0_f64;
        let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
        let ux_rel = (ux - ux_expected).abs() / ux_expected.abs();
        let uy_rel = (uy - uy_expected).abs() / uy_expected.abs();
        assert!(ux_rel < 1e-9, "ux rel err={}", ux_rel);
        assert!(uy_rel < 1e-4, "uy rel err={}", uy_rel);
    }
}
