//! AD-HOC PERF PROBE (untracked/new file, not part of the normal test suite's
//! intent) -- measures where wall-clock time goes inside
//! `App::generate_stories_action` (crates/squid-n-app/src/app/actions.rs,
//! ~line 845) on synthetic large regular-frame models.
//!
//! `generate_stories_action` has 5 stages:
//!   1. sync_gravity_load_cases_action
//!   2. squid_n_load::story_gen::generate_stories_with_opts
//!   3. undo.run(ApplyStories)
//!   4. apply_rigid_zones_for_analysis (== squid_n_element::beam::apply_auto_rigid_zones)
//!   5. sync_seismic_load_cases_action
//!
//! Stages 1/2/3/5 use methods/functions that are already `pub` on `App` /
//! `squid_n_load`, so they are called directly (not re-implemented). Stage 4's
//! `App::apply_rigid_zones_for_analysis` is a private one-line wrapper around
//! the public `squid_n_element::beam::apply_auto_rigid_zones`, so that public
//! function is called directly instead (byte-for-byte what the private
//! wrapper does). The two private free functions that stage 2 needs as inputs
//! (`gravity_cases_for_seismic_weight`, `density_self_weight_for_stories` in
//! crates/squid-n-app/src/app/mod.rs) are trivial selection logic over
//! `model.load_cases`; they are inlined verbatim below using the *public*
//! `DL_CASE_NAME` / `LL_FRAME_CASE_NAME` / `SELF_WEIGHT_AUTO_LOAD_CASE_NAME`
//! constants re-exported by `squid_n_app::app`, so no private items are
//! touched anywhere in this file.
//!
//! Run (release, single-threaded so wall time is not muddied by test
//! parallelism; timings are printed via --nocapture):
//!
//!   cargo test --release -p squid-n-app --test perf_probe -- --nocapture --test-threads=1
//!
//! This file is new/untracked -- no tracked source file was modified to
//! create it (tests/ is auto-discovered by Cargo, no Cargo.toml edit needed).

use std::time::Instant;

use squid_n_app::app::{App, DL_CASE_NAME, LL_FRAME_CASE_NAME, SELF_WEIGHT_AUTO_LOAD_CASE_NAME};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, SlabId};
use squid_n_core::model::{
    AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
    LoadCaseKind, LocalAxis, Material, Model, Node, Section, Slab,
};
use squid_n_element::beam::{apply_auto_rigid_zones, RigidZoneRule};

/// 節点格子: nx * ny 本の通り芯 x n_stories 階 (+ 基部レベル)。
/// 柱・梁は RC 概算断面。各階各スパンにスラブ (一様 DL) を配置する。
fn build_grid_model(nx: usize, ny: usize, n_stories: usize, with_slabs: bool) -> Model {
    let bay = 6000.0_f64; // mm
    let story_h = 3500.0_f64; // mm

    let mut model = Model::default();

    let node_id = |level: usize, i: usize, j: usize| -> u32 { ((level * ny + j) * nx + i) as u32 };

    for level in 0..=n_stories {
        for j in 0..ny {
            for i in 0..nx {
                let id = node_id(level, i, j);
                model.nodes.push(Node {
                    id: NodeId(id),
                    coord: [i as f64 * bay, j as f64 * bay, level as f64 * story_h],
                    restraint: if level == 0 {
                        Dof6Mask::FIXED
                    } else {
                        Dof6Mask::FREE
                    },
                    mass: None,
                    story: None,
                });
            }
        }
    }

    model.sections.push(Section {
        id: SectionId(0),
        name: "COL 500x500".into(),
        area: 500.0 * 500.0,
        iy: 5.2e9,
        iz: 5.2e9,
        j: 8.8e9,
        depth: 500.0,
        width: 500.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "BEAM 400x700".into(),
        area: 400.0 * 700.0,
        iy: 1.14e10,
        iz: 3.73e9,
        j: 5.0e9,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 22000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });

    let mut elem_id = 0u32;
    let mut push_elem = |model: &mut Model, n0: u32, n1: u32, section: u32, ref_v: [f64; 3]| {
        model.elements.push(ElementData {
            id: ElemId(elem_id),
            kind: ElementKind::Beam,
            nodes: [NodeId(n0), NodeId(n1)].into_iter().collect(),
            section: Some(SectionId(section)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector: ref_v },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        elem_id += 1;
    };

    // 柱: 全通り芯 x 各階
    for level in 1..=n_stories {
        for j in 0..ny {
            for i in 0..nx {
                let n0 = node_id(level - 1, i, j);
                let n1 = node_id(level, i, j);
                push_elem(&mut model, n0, n1, 0, [1.0, 0.0, 0.0]);
            }
        }
    }
    // 梁: 各階の X 通り・Y 通り
    for level in 1..=n_stories {
        for j in 0..ny {
            for i in 0..nx - 1 {
                let n0 = node_id(level, i, j);
                let n1 = node_id(level, i + 1, j);
                push_elem(&mut model, n0, n1, 1, [0.0, 0.0, 1.0]);
            }
        }
        for j in 0..ny - 1 {
            for i in 0..nx {
                let n0 = node_id(level, i, j);
                let n1 = node_id(level, i, j + 1);
                push_elem(&mut model, n0, n1, 1, [0.0, 0.0, 1.0]);
            }
        }
    }

    if with_slabs {
        let mut slab_id = 0u32;
        for level in 1..=n_stories {
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    let n00 = node_id(level, i, j);
                    let n10 = node_id(level, i + 1, j);
                    let n11 = node_id(level, i + 1, j + 1);
                    let n01 = node_id(level, i, j + 1);
                    model.slabs.push(Slab {
                        id: SlabId(slab_id),
                        boundary: vec![NodeId(n00), NodeId(n10), NodeId(n11), NodeId(n01)],
                        joists: vec![],
                        loads: vec![AreaLoad {
                            kind: "DL".into(),
                            value: 0.005,
                        }],
                        method: DistributionMethod::TriTrapezoid,
                        usage: None,
                        edge_supported: None,
                        thickness: None,
                        kind: Default::default(),
                        one_way: None,
                    });
                    slab_id += 1;
                }
            }
        }
    }

    model
        .validate()
        .expect("生成したベンチモデルは validate を通るはず");
    model
}

/// `App::generate_stories_action` の 5 ステージを、同じ公開 API 呼び出しで
/// 再現しつつ個別に計測する（private 関数には一切触れない。詳細はファイル
/// 冒頭のコメント参照）。
fn timed_generate_stories(app: &mut App) -> [std::time::Duration; 5] {
    app.last_error = None;

    // --- stage 1: sync_gravity_load_cases_action (pub) ---
    let t0 = Instant::now();
    app.sync_gravity_load_cases_action();
    let d1 = t0.elapsed();

    // --- inputs to stage 2, replicated verbatim from the private
    // `gravity_cases_for_seismic_weight` / `density_self_weight_for_stories`
    // (crates/squid-n-app/src/app/mod.rs) using only public constants ---
    let gravity_lcs: Vec<LoadCaseId> = {
        let any_kind_set = app
            .model
            .load_cases
            .iter()
            .any(|lc| lc.kind != LoadCaseKind::Other);
        if !any_kind_set {
            app.model.load_cases.first().map(|c| c.id).into_iter().collect()
        } else {
            let mut result: Vec<LoadCaseId> = app
                .model
                .load_cases
                .iter()
                .filter(|lc| {
                    lc.kind == LoadCaseKind::Dead && lc.name != SELF_WEIGHT_AUTO_LOAD_CASE_NAME
                })
                .map(|lc| lc.id)
                .collect();
            let live_seismic: Vec<LoadCaseId> = app
                .model
                .load_cases
                .iter()
                .filter(|lc| lc.kind == LoadCaseKind::LiveSeismic)
                .map(|lc| lc.id)
                .collect();
            if !live_seismic.is_empty() {
                result.extend(live_seismic);
            } else {
                result.extend(
                    app.model
                        .load_cases
                        .iter()
                        .filter(|lc| lc.kind == LoadCaseKind::Live && lc.name != LL_FRAME_CASE_NAME)
                        .map(|lc| lc.id),
                );
            }
            result
        }
    };
    let include_density = !app
        .model
        .load_cases
        .iter()
        .any(|lc| lc.kind == LoadCaseKind::Dead && lc.name == DL_CASE_NAME);

    // --- stage 2: squid_n_load::story_gen::generate_stories_with_opts (pub) ---
    let t0 = Instant::now();
    let gen = squid_n_load::story_gen::generate_stories_with_opts(
        &app.model,
        &gravity_lcs,
        include_density,
    )
    .expect("story generation should succeed on a well-formed grid model");
    let d2 = t0.elapsed();

    // --- stage 3: undo.run(ApplyStories) (pub `undo` field + pub `ApplyStories`) ---
    let t0 = Instant::now();
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::ApplyStories {
            stories: gen.stories,
            node_story: gen.node_story,
            constraints: gen.constraints,
            rep_nodes: gen.rep_nodes,
            generated_masters: gen.generated_masters,
        }),
    );
    let d3 = t0.elapsed();
    app.staleness.mark_edited();

    // --- stage 4: apply_rigid_zones_for_analysis == apply_auto_rigid_zones (pub) ---
    let t0 = Instant::now();
    apply_auto_rigid_zones(&mut app.model, &RigidZoneRule::default());
    let d4 = t0.elapsed();

    // --- stage 5: sync_seismic_load_cases_action (pub) ---
    let t0 = Instant::now();
    app.sync_seismic_load_cases_action();
    let d5 = t0.elapsed();

    [d1, d2, d3, d4, d5]
}

fn fmt_ms(d: std::time::Duration) -> String {
    format!("{:>9.3} ms", d.as_secs_f64() * 1000.0)
}

fn run_case(label: &str, nx: usize, ny: usize, n_stories: usize, with_slabs: bool) {
    {
        use std::io::Write;
        println!(">>> starting {label} grid {nx}x{ny} x {n_stories} (slabs={with_slabs}) ...");
        std::io::stdout().flush().ok();
    }
    let model = build_grid_model(nx, ny, n_stories, with_slabs);
    let n_nodes = model.nodes.len();
    let n_elems = model.elements.len();
    let n_slabs = model.slabs.len();

    let mut app = App::default();
    let t_load0 = Instant::now();
    app.load_model(model);
    let t_load = t_load0.elapsed();

    // 5-stage manual replay (fully instrumented).
    let stages = timed_generate_stories(&mut app);

    // Whole-action cross-check on a fresh copy of the same model (so we can
    // compare sum(stages) against the real `generate_stories_action` black box).
    let model2 = build_grid_model(nx, ny, n_stories, with_slabs);
    let mut app2 = App::default();
    app2.load_model(model2);
    let t_whole0 = Instant::now();
    app2.generate_stories_action();
    let t_whole = t_whole0.elapsed();
    assert!(
        app2.last_error.is_none(),
        "generate_stories_action failed: {:?}",
        app2.last_error
    );

    let sum: std::time::Duration = stages.iter().sum();

    println!(
        "\n=== {label}  grid {nx}x{ny} x {n_stories} stories  (nodes={n_nodes}, elems={n_elems}, slabs={n_slabs}) ==="
    );
    println!("  load_model                              : {}", fmt_ms(t_load));
    println!("  [1] sync_gravity_load_cases_action       : {}", fmt_ms(stages[0]));
    println!("  [2] story_gen::generate_stories_with_opts: {}", fmt_ms(stages[1]));
    println!("  [3] undo.run(ApplyStories)                : {}", fmt_ms(stages[2]));
    println!("  [4] apply_rigid_zones_for_analysis        : {}", fmt_ms(stages[3]));
    println!("  [5] sync_seismic_load_cases_action        : {}", fmt_ms(stages[4]));
    println!("  ---------------------------------------------------------");
    println!("  sum(1..5) manual replay                   : {}", fmt_ms(sum));
    println!("  generate_stories_action (whole, fresh app): {}", fmt_ms(t_whole));
    use std::io::Write;
    std::io::stdout().flush().ok();
}

/// Not a correctness test -- an ad-hoc timing harness. `#[ignore]` so it does
/// not run as part of the normal `cargo test` suite; invoke explicitly with
/// `--ignored` (see module docs for the exact command).
#[test]
#[ignore = "perf probe, not a correctness test -- run explicitly with --release --ignored"]
fn perf_probe_generate_stories_action() {
    // Grid width/depth (nx * ny columns) x story count, with and without slabs.
    run_case("A small     ", 5, 5, 5, true);
    run_case("B medium-w  ", 10, 10, 5, true);
    run_case("C medium-h  ", 10, 10, 10, true);
    run_case("D large     ", 10, 10, 20, true);
    run_case("E wide      ", 15, 15, 10, true);
    run_case("F wide+tall ", 15, 15, 20, true);
    // No-slab variant at the largest size, to isolate the slab-processing cost.
    run_case("G large,noSlab", 10, 10, 20, false);
}

/// Same as above but only the two smallest cases -- used to calibrate how long
/// a single case takes before committing to the full sweep (see task notes).
#[test]
#[ignore = "perf probe calibration subset -- run explicitly with --release --ignored"]
fn perf_probe_calibration_small() {
    run_case("A small     ", 5, 5, 5, true);
    run_case("B medium-w  ", 10, 10, 5, true);
}
