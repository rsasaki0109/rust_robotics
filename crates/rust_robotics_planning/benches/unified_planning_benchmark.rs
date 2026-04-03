use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rust_robotics_core::{Obstacles, Path2D, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::bidirectional_a_star::{
    BidirectionalAStarConfig, BidirectionalAStarPlanner,
};
use rust_robotics_planning::bidirectional_bfs::{BidirectionalBFSConfig, BidirectionalBFSPlanner};
use rust_robotics_planning::breadth_first_search::{BFSConfig, BFSPlanner};
use rust_robotics_planning::depth_first_search::{DFSConfig, DFSPlanner};
use rust_robotics_planning::flow_field::{FlowFieldConfig, FlowFieldPlanner};
use rust_robotics_planning::greedy_best_first_search::{
    GreedyBestFirstConfig, GreedyBestFirstPlanner,
};
use rust_robotics_planning::jps::{
    JPSConfig, JPSCostMismatchMetrics, JPSFallbackReason, JPSInvalidJumpPathDetail, JPSPlanner,
};
use rust_robotics_planning::moving_ai::{MovingAiMap, MovingAiScenario};
use rust_robotics_planning::lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
use rust_robotics_planning::path_smoothing::smooth_path;
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
use std::collections::{HashMap, HashSet};
use std::env;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

// ---------------------------------------------------------------------------
// Scenario helpers
// ---------------------------------------------------------------------------

const GRID_SIZE: i32 = 50;
const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;

/// Open grid: only boundary walls.
fn open_grid_obstacles() -> Obstacles {
    let mut obs = Obstacles::new();
    for i in 0..=GRID_SIZE {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, GRID_SIZE as f64));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(GRID_SIZE as f64, i as f64));
    }
    obs
}

fn draw_vertical_line(obstacles: &mut Obstacles, x: i32, start_y: i32, len: i32) {
    for y in start_y..start_y + len {
        obstacles.push(Point2D::new(x as f64, y as f64));
    }
}

fn draw_horizontal_line(obstacles: &mut Obstacles, start_x: i32, y: i32, len: i32) {
    for x in start_x..start_x + len {
        obstacles.push(Point2D::new(x as f64, y as f64));
    }
}

/// Maze grid adapted from the upstream JPS regression maze.
fn maze_obstacles() -> Obstacles {
    let mut obs = open_grid_obstacles();

    let vertical_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45];
    let vertical_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25];
    let vertical_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15];
    for ((x, y), len) in vertical_x
        .iter()
        .zip(vertical_y.iter())
        .zip(vertical_len.iter())
    {
        draw_vertical_line(&mut obs, *x, *y, *len);
    }

    let horizontal_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40];
    let horizontal_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45];
    let horizontal_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5];
    for ((x, y), len) in horizontal_x
        .iter()
        .zip(horizontal_y.iter())
        .zip(horizontal_len.iter())
    {
        draw_horizontal_line(&mut obs, *x, *y, *len);
    }

    obs
}

/// Dense random-ish obstacles: boundary walls plus ~20% of interior cells
/// blocked with a deterministic pattern (no rand dependency needed).
fn dense_obstacles() -> Obstacles {
    let mut obs = open_grid_obstacles();
    // Simple hash-based deterministic "random" placement.
    for x in 2..GRID_SIZE - 1 {
        for y in 2..GRID_SIZE - 1 {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash < 20 {
                obs.push(Point2D::new(x as f64, y as f64));
            }
        }
    }
    obs
}

struct Scenario {
    name: String,
    obstacles: Obstacles,
    start: Point2D,
    goal: Point2D,
    reference_length: Option<f64>,
}

fn scenarios() -> Vec<Scenario> {
    let mut scenarios = vec![
        Scenario {
            name: "open_50x50".to_string(),
            obstacles: open_grid_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(48.0, 48.0),
            reference_length: None,
        },
        Scenario {
            name: "maze_50x50".to_string(),
            obstacles: maze_obstacles(),
            start: Point2D::new(5.0, 5.0),
            goal: Point2D::new(35.0, 45.0),
            reference_length: None,
        },
        Scenario {
            name: "dense_50x50".to_string(),
            obstacles: dense_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(48.0, 48.0),
            reference_length: None,
        },
    ];
    scenarios.extend(load_moving_ai_scenarios_from_env());
    scenarios
}

// ---------------------------------------------------------------------------
// Summary table collection
// ---------------------------------------------------------------------------

struct RunResult {
    planner: &'static str,
    scenario: String,
    path_length: f64,
    waypoints: usize,
    reference_length: Option<f64>,
}

static RESULTS: Mutex<Vec<RunResult>> = Mutex::new(Vec::new());
struct JPSFallbackRunResult {
    scenario: String,
    used_fallback: bool,
    fallback_reason: Option<JPSFallbackReason>,
    invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
    cost_mismatch_metrics: Option<JPSCostMismatchMetrics>,
}

static JPS_FALLBACK_RESULTS: Mutex<Vec<JPSFallbackRunResult>> = Mutex::new(Vec::new());

fn record(planner: &'static str, scenario: &Scenario, path: &Path2D) {
    RESULTS.lock().unwrap().push(RunResult {
        planner,
        scenario: scenario.name.clone(),
        path_length: path.total_length(),
        waypoints: path.len(),
        reference_length: scenario.reference_length,
    });
}

fn record_jps_fallback(
    scenario: &Scenario,
    used_fallback: bool,
    fallback_reason: Option<JPSFallbackReason>,
    invalid_jump_path_detail: Option<JPSInvalidJumpPathDetail>,
    cost_mismatch_metrics: Option<JPSCostMismatchMetrics>,
) {
    JPS_FALLBACK_RESULTS
        .lock()
        .unwrap()
        .push(JPSFallbackRunResult {
            scenario: scenario.name.clone(),
            used_fallback,
            fallback_reason,
            invalid_jump_path_detail,
            cost_mismatch_metrics,
        });
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PlannerFamily {
    SearchBaseline,
    GridOptimal,
    AnyAngle,
}

fn planner_family(planner: &str) -> PlannerFamily {
    match planner {
        "A*" | "BidirectionalA*" | "BidirectionalBFS" | "JPS" | "FlowField" => {
            PlannerFamily::GridOptimal
        }
        "Theta*" | "LazyTheta*" | "DFS+Smooth" => PlannerFamily::AnyAngle,
        _ => PlannerFamily::SearchBaseline,
    }
}

fn scenario_reference_lengths(results: &[RunResult]) -> HashMap<String, f64> {
    let mut references = HashMap::new();

    for result in results {
        if let Some(reference_length) = result.reference_length {
            references
                .entry(result.scenario.clone())
                .or_insert(reference_length);
        }
    }

    let mut best_grid_lengths = HashMap::new();
    for result in results {
        if planner_family(result.planner) != PlannerFamily::GridOptimal {
            continue;
        }
        best_grid_lengths
            .entry(result.scenario.clone())
            .and_modify(|best| {
                if result.path_length < *best {
                    *best = result.path_length;
                }
            })
            .or_insert(result.path_length);
    }

    for (scenario, best_length) in best_grid_lengths {
        references.entry(scenario).or_insert(best_length);
    }

    references
}

fn print_section(
    title: &str,
    family: PlannerFamily,
    results: &[RunResult],
    references: &HashMap<String, f64>,
) {
    let section_results: Vec<&RunResult> = results
        .iter()
        .filter(|result| planner_family(result.planner) == family)
        .collect();
    if section_results.is_empty() {
        return;
    }

    eprintln!();
    eprintln!("{title}");
    eprintln!(
        "{:<22} {:<32} {:>10} {:>10} {:>10} {:>10}",
        "Planner", "Scenario", "Length", "Delta", "Ref", "Waypoints"
    );
    eprintln!("{:-<98}", "");

    for result in section_results {
        let reference_length = references.get(&result.scenario).copied();
        let delta = reference_length.map(|reference| result.path_length - reference);
        let delta_text = delta.map_or_else(|| "-".to_string(), |value| format!("{value:.2}"));
        let ref_text =
            reference_length.map_or_else(|| "-".to_string(), |value| format!("{value:.2}"));

        eprintln!(
            "{:<22} {:<32} {:>10.2} {:>10} {:>10} {:>10}",
            result.planner,
            result.scenario,
            result.path_length,
            delta_text,
            ref_text,
            result.waypoints
        );
    }
}

fn print_summary() {
    let results = RESULTS.lock().unwrap();
    if results.is_empty() {
        return;
    }
    let references = scenario_reference_lengths(&results);

    eprintln!();
    eprintln!("======================================================================");
    eprintln!("  Unified Planning Benchmark -- Path Summary");
    eprintln!("======================================================================");
    eprintln!("Reference length = MovingAI scenario optimal when available, otherwise best grid-optimal length in this run.");
    eprintln!("Delta = path length - reference length.");
    print_section(
        "Search Baselines",
        PlannerFamily::SearchBaseline,
        &results,
        &references,
    );
    print_section(
        "Grid-Optimal Family",
        PlannerFamily::GridOptimal,
        &results,
        &references,
    );
    print_section(
        "Any-Angle Family",
        PlannerFamily::AnyAngle,
        &results,
        &references,
    );
    print_jps_fallback_summary();
    eprintln!("======================================================================");
}

fn print_jps_fallback_summary() {
    let results = JPS_FALLBACK_RESULTS.lock().unwrap();
    if results.is_empty() {
        return;
    }

    eprintln!();
    eprintln!("JPS Fallback Summary");
    eprintln!(
        "{:<32} {:>14} {:>24} {:>34} {:>12} {:>10}",
        "Scenario", "UsedFallback", "Reason", "Detail", "DeltaLen", "Detours"
    );
    eprintln!("{:-<134}", "");

    for result in results.iter() {
        let reason = result
            .fallback_reason
            .map(JPSFallbackReason::as_str)
            .unwrap_or("-");
        let detail = result
            .invalid_jump_path_detail
            .map(JPSInvalidJumpPathDetail::as_str)
            .unwrap_or("-");
        let delta_len = result.cost_mismatch_metrics.as_ref().map_or_else(
            || "-".to_string(),
            |metrics| format!("{:.2}", metrics.delta),
        );
        let detours = result.cost_mismatch_metrics.as_ref().map_or_else(
            || "-".to_string(),
            |metrics| metrics.detour_segment_count.to_string(),
        );
        eprintln!(
            "{:<32} {:>14} {:>24} {:>34} {:>12} {:>10}",
            result.scenario, result.used_fallback, reason, detail, delta_len, detours
        );
    }

    let total_count = results.len();
    let total_fallback_count = results.iter().filter(|result| result.used_fallback).count();
    let movingai_results: Vec<&JPSFallbackRunResult> = results
        .iter()
        .filter(|result| result.scenario.starts_with("movingai_"))
        .collect();
    let movingai_fallback_count = movingai_results
        .iter()
        .filter(|result| result.used_fallback)
        .count();

    eprintln!();
    eprintln!(
        "JPS fallback rate (all scenarios): {}/{}",
        total_fallback_count, total_count
    );
    if !movingai_results.is_empty() {
        eprintln!(
            "JPS fallback rate (MovingAI scenarios): {}/{}",
            movingai_fallback_count,
            movingai_results.len()
        );
    }

    let mut invalid_jump_path_detail_counts: HashMap<JPSInvalidJumpPathDetail, usize> =
        HashMap::new();
    for result in results.iter() {
        if result.fallback_reason != Some(JPSFallbackReason::InvalidJumpPath) {
            continue;
        }
        if let Some(detail) = result.invalid_jump_path_detail {
            *invalid_jump_path_detail_counts.entry(detail).or_insert(0) += 1;
        }
    }

    if !invalid_jump_path_detail_counts.is_empty() {
        eprintln!("JPS invalid_jump_path breakdown:");
        for detail in [
            JPSInvalidJumpPathDetail::InvalidStepSequence,
            JPSInvalidJumpPathDetail::CostMismatch,
            JPSInvalidJumpPathDetail::InvalidStepSequenceAndCostMismatch,
        ] {
            let count = invalid_jump_path_detail_counts
                .get(&detail)
                .copied()
                .unwrap_or(0);
            eprintln!("  {}: {}", detail.as_str(), count);
        }
    }
}

fn probe_jps_fallback(scenario: &Scenario) {
    let planner = match JPSPlanner::from_obstacle_points(
        &scenario.obstacles,
        JPSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        Ok(planner) => planner,
        Err(err) => {
            eprintln!(
                "[SKIP] JPS fallback probe on {} (planner construction failed: {})",
                scenario.name, err
            );
            return;
        }
    };

    match planner.plan_with_diagnostics(scenario.start, scenario.goal) {
        Ok(result) => record_jps_fallback(
            scenario,
            result.diagnostics.used_fallback,
            result.diagnostics.fallback_reason,
            result.diagnostics.invalid_jump_path_detail,
            result.diagnostics.cost_mismatch_metrics,
        ),
        Err(err) => eprintln!(
            "[SKIP] JPS fallback probe on {} (planning failed: {})",
            scenario.name, err
        ),
    }
}

// ---------------------------------------------------------------------------
// Planner factory: each closure builds the planner and returns a boxed plan fn
// ---------------------------------------------------------------------------

type PlanFn = Box<dyn Fn(Point2D, Point2D) -> Path2D>;

fn build_planners(obstacles: &Obstacles) -> Vec<(&'static str, Option<PlanFn>)> {
    let mut planners: Vec<(&'static str, Option<PlanFn>)> = Vec::new();

    // A*
    if let Ok(p) = AStarPlanner::from_obstacle_points(
        obstacles,
        AStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        planners.push((
            "A*",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("A*", None));
    }

    // BFS
    if let Ok(p) = BFSPlanner::from_obstacle_points(
        obstacles,
        BFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        planners.push((
            "BFS",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("BFS", None));
    }

    // DFS
    if let Ok(p) = DFSPlanner::from_obstacle_points(
        obstacles,
        DFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        planners.push((
            "DFS",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("DFS", None));
    }

    // GreedyBestFirst
    if let Ok(p) = GreedyBestFirstPlanner::from_obstacle_points(
        obstacles,
        GreedyBestFirstConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        planners.push((
            "GreedyBestFirst",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("GreedyBestFirst", None));
    }

    // BidirectionalA*
    if let Ok(p) = BidirectionalAStarPlanner::from_obstacle_points(
        obstacles,
        BidirectionalAStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        planners.push((
            "BidirectionalA*",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("BidirectionalA*", None));
    }

    // BidirectionalBFS
    if let Ok(p) = BidirectionalBFSPlanner::from_obstacle_points(
        obstacles,
        BidirectionalBFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        planners.push((
            "BidirectionalBFS",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("BidirectionalBFS", None));
    }

    // JPS
    if let Ok(p) = JPSPlanner::from_obstacle_points(
        obstacles,
        JPSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        planners.push((
            "JPS",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("JPS", None));
    }

    // Theta*
    if let Ok(p) = ThetaStarPlanner::from_obstacle_points(
        obstacles,
        ThetaStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        planners.push((
            "Theta*",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("Theta*", None));
    }

    // FlowField
    if let Ok(p) = FlowFieldPlanner::from_obstacle_points(
        obstacles,
        FlowFieldConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        planners.push((
            "FlowField",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("FlowField", None));
    }

    // Lazy Theta*
    if let Ok(p) = LazyThetaStarPlanner::from_obstacle_points(
        obstacles,
        LazyThetaStarConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
            heuristic_weight: 1.0,
        },
    ) {
        planners.push((
            "LazyTheta*",
            Some(Box::new(move |s, g| p.plan(s, g).unwrap_or_default())),
        ));
    } else {
        planners.push(("LazyTheta*", None));
    }

    // DFS+Smooth
    if let Ok(p) = DFSPlanner::from_obstacle_points(
        obstacles,
        DFSConfig {
            resolution: RESOLUTION,
            robot_radius: ROBOT_RADIUS,
        },
    ) {
        let grid_map = p.grid_map().clone();
        planners.push((
            "DFS+Smooth",
            Some(Box::new(move |s, g| {
                let raw = p.plan(s, g).unwrap_or_default();
                smooth_path(&raw, &grid_map)
            })),
        ));
    } else {
        planners.push(("DFS+Smooth", None));
    }

    planners
}

fn load_moving_ai_scenarios_from_env() -> Vec<Scenario> {
    let scen_var = match env::var("RUST_ROBOTICS_MOVINGAI_SCEN") {
        Ok(value) => value,
        Err(_) => return Vec::new(),
    };

    let root = env::var("RUST_ROBOTICS_MOVINGAI_ROOT")
        .ok()
        .map(PathBuf::from);
    let per_file_limit = env::var("RUST_ROBOTICS_MOVINGAI_LIMIT")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .filter(|limit| *limit > 0)
        .unwrap_or(3);
    let bucket_filters = parse_bucket_filters();

    let mut all_scenarios = Vec::new();
    for scen_path in scen_var
        .split(',')
        .map(str::trim)
        .filter(|value| !value.is_empty())
    {
        match load_moving_ai_scenarios(
            Path::new(scen_path),
            root.as_deref(),
            per_file_limit,
            bucket_filters.as_deref(),
        ) {
            Ok(mut scenarios) => all_scenarios.append(&mut scenarios),
            Err(err) => {
                eprintln!("[SKIP] failed to load MovingAI scenarios from {scen_path}: {err}")
            }
        }
    }

    all_scenarios
}

fn load_moving_ai_scenarios(
    scen_path: &Path,
    root: Option<&Path>,
    per_file_limit: usize,
    bucket_filters: Option<&[u32]>,
) -> rust_robotics_core::RoboticsResult<Vec<Scenario>> {
    let scenario_rows = MovingAiScenario::from_file(scen_path)?;
    let default_root = scen_path
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .to_path_buf();
    let dataset_root = root.unwrap_or(default_root.as_path());
    let mut map_cache: HashMap<PathBuf, MovingAiMap> = HashMap::new();
    let mut selected = Vec::new();
    let chosen_rows = select_scenario_rows(scenario_rows, per_file_limit, bucket_filters);

    for (scenario_index, row) in chosen_rows.into_iter().enumerate() {
        let map_path = resolve_map_path(dataset_root, &row.map);
        let map = if let Some(existing) = map_cache.get(&map_path) {
            existing.clone()
        } else {
            let loaded = MovingAiMap::from_file(&map_path)?;
            map_cache.insert(map_path.clone(), loaded);
            map_cache
                .get(&map_path)
                .expect("inserted map should be available")
                .clone()
        };

        if map.width != row.width || map.height != row.height {
            return Err(rust_robotics_core::RoboticsError::InvalidParameter(
                format!(
                    "scenario dimensions {}x{} do not match map {}x{} for {}",
                    row.width,
                    row.height,
                    map.width,
                    map.height,
                    map_path.display()
                ),
            ));
        }

        let map_stem = map_path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("movingai_map");
        selected.push(Scenario {
            name: format!("movingai_{map_stem}_b{}_s{}", row.bucket, scenario_index),
            obstacles: map.to_obstacles(),
            start: map.planning_point(row.start_x, row.start_y)?,
            goal: map.planning_point(row.goal_x, row.goal_y)?,
            reference_length: Some(row.optimal_length),
        });
    }

    Ok(selected)
}

fn resolve_map_path(root: &Path, map_field: &str) -> PathBuf {
    let relative_path = Path::new(map_field);
    if relative_path.is_absolute() {
        return relative_path.to_path_buf();
    }
    root.join(relative_path)
}

fn parse_bucket_filters() -> Option<Vec<u32>> {
    let raw = env::var("RUST_ROBOTICS_MOVINGAI_BUCKETS").ok()?;
    let buckets: Vec<u32> = raw
        .split(',')
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .filter_map(|value| value.parse::<u32>().ok())
        .collect();
    if buckets.is_empty() {
        None
    } else {
        Some(buckets)
    }
}

fn select_scenario_rows(
    scenario_rows: Vec<MovingAiScenario>,
    per_file_limit: usize,
    bucket_filters: Option<&[u32]>,
) -> Vec<MovingAiScenario> {
    if let Some(buckets) = bucket_filters {
        let mut rows_by_bucket: HashMap<u32, MovingAiScenario> = HashMap::new();
        for row in scenario_rows {
            rows_by_bucket.entry(row.bucket).or_insert(row);
        }

        let mut selected = Vec::new();
        for bucket in buckets {
            if let Some(row) = rows_by_bucket.get(bucket) {
                selected.push(row.clone());
            }
        }

        let requested: HashSet<u32> = buckets.iter().copied().collect();
        let found: HashSet<u32> = selected.iter().map(|row| row.bucket).collect();
        for missing in requested.difference(&found) {
            eprintln!("[SKIP] requested MovingAI bucket {} was not found", missing);
        }

        return selected;
    }

    scenario_rows.into_iter().take(per_file_limit).collect()
}

// ---------------------------------------------------------------------------
// Benchmark
// ---------------------------------------------------------------------------

fn bench_unified(c: &mut Criterion) {
    for scenario in scenarios() {
        probe_jps_fallback(&scenario);
        let planners = build_planners(&scenario.obstacles);

        let mut group = c.benchmark_group(&scenario.name);
        group.sample_size(20);

        for (name, plan_fn) in &planners {
            if let Some(plan) = plan_fn {
                // Record path info once outside the timed loop.
                let path = plan(scenario.start, scenario.goal);
                record(name, &scenario, &path);

                group.bench_with_input(BenchmarkId::new(*name, ""), &(), |b, _| {
                    b.iter(|| plan(black_box(scenario.start), black_box(scenario.goal)));
                });
            } else {
                eprintln!(
                    "[SKIP] {} on {} (planner construction failed)",
                    name, &scenario.name
                );
            }
        }

        group.finish();
    }
}

fn bench_and_summarize(c: &mut Criterion) {
    bench_unified(c);
    print_summary();
}

criterion_group!(benches, bench_and_summarize);
criterion_main!(benches);
