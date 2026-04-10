//! Grid threshold-planner comparison on shared local fixtures.
//!
//! Compares `A*`, `Fringe Search`, and `IDA*` under the same `PathPlanner`
//! contract on three deterministic grid cases. The benchmark is intentionally
//! small enough to run in unit-test time while still producing repeatable
//! runtime, quality, and search-effort summaries for docs.

use std::collections::HashMap;
use std::hint::black_box;
use std::path::Path;
use std::time::Duration;
use std::time::Instant;

use rust_robotics_core::experiments::{read_source_metrics, VariantDescriptor};
use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner, AStarSearchStats};
use rust_robotics_planning::fringe_search::{
    FringeSearchConfig, FringeSearchPlanner, FringeSearchStats,
};
use rust_robotics_planning::ida_star::{
    ContourStats, IDAStarConfig, IDAStarPlanOutcome, IDAStarPlanner, IDAStarSearchStats,
};
use rust_robotics_planning::moving_ai::{MovingAiMap, MovingAiScenario};

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;
const TIMING_WARMUP_RUNS: usize = 8;
const TIMING_MIN_RUNS: usize = 8;
const TIMING_WINDOW_SAMPLES: usize = 5;
const TIMING_MIN_WINDOW: Duration = Duration::from_millis(100);
const ARENA2_MAP: &str = include_str!("../benchdata/moving_ai/dao/arena2.map");
const ARENA2_SCENARIO: &str = include_str!("../benchdata/moving_ai/dao/arena2.map.scen");
const MAZE_MAP: &str = include_str!("../benchdata/moving_ai/maze/maze512-1-0.map");
const MAZE_SCENARIO: &str = include_str!("../benchdata/moving_ai/maze/maze512-1-0.map.scen");
const ROOM_MAP: &str = include_str!("../benchdata/moving_ai/room/8room_000.map");
const ROOM_SCENARIO: &str = include_str!("../benchdata/moving_ai/room/8room_000.map.scen");
const RANDOM_MAP: &str = include_str!("../benchdata/moving_ai/random/random512-10-0.map");
const RANDOM_SCENARIO: &str = include_str!("../benchdata/moving_ai/random/random512-10-0.map.scen");
const STREET_MAP: &str = include_str!("../benchdata/moving_ai/street/Berlin_0_512.map");
const STREET_SCENARIO: &str = include_str!("../benchdata/moving_ai/street/Berlin_0_512.map.scen");
const LARGER_MOVING_AI_BUCKET: u32 = 5;
const LARGER_MOVING_AI_SLOT: usize = 0;
const HARDER_MOVING_AI_BUCKET: u32 = 12;
const HARDER_MOVING_AI_SLOT: usize = 0;
const MOVING_AI_WINDOW_MARGINS: [usize; 3] = [8, 16, 24];

struct PlannerVariant {
    descriptor: VariantDescriptor,
    evaluate_case: fn(&ComparisonCase) -> CaseObservation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TimingBudget {
    MedianWindows,
    SingleShot,
}

struct ComparisonCase {
    id: &'static str,
    obstacles: Obstacles,
    start: Point2D,
    goal: Point2D,
    timing_budget: TimingBudget,
}

#[derive(Debug, Clone, Copy)]
struct SearchEffort {
    expanded_nodes: usize,
    unique_expanded_nodes: usize,
    reexpanded_nodes: usize,
    generated_nodes: usize,
    pruned_nodes: usize,
    control_rounds: usize,
    peak_live_nodes: usize,
}

#[derive(Debug, Clone)]
struct CaseObservation {
    case_id: &'static str,
    runtime_us: f64,
    path_length: f64,
    waypoint_count: usize,
    length_ratio_vs_astar: f64,
    effort: SearchEffort,
}

#[derive(Debug, Clone, Copy)]
struct BoundedIdaProfile {
    id: &'static str,
    initial_lookahead_depth: usize,
    max_iterations: usize,
    max_expanded_nodes: usize,
}

#[derive(Debug, Clone)]
struct BoundedIdaObservation {
    profile_id: &'static str,
    case_id: &'static str,
    runtime_us: f64,
    outcome: &'static str,
    path_length: Option<f64>,
    effort: SearchEffort,
    waypoint_count: Option<usize>,
    last_searched_threshold: f64,
    next_threshold: Option<f64>,
    contour_history: Vec<ContourStats>,
}

#[derive(Debug, Clone, Copy)]
struct MovingAiSliceSpec {
    id_prefix: &'static str,
    bucket: u32,
    slot: usize,
}

#[derive(Debug, Clone)]
struct VariantSummary {
    id: &'static str,
    design_style: &'static str,
    runtime_us_mean: f64,
    waypoint_mean: f64,
    length_ratio_mean: f64,
    expanded_mean: f64,
    generated_mean: f64,
    pruned_mean: f64,
    rounds_mean: f64,
    peak_live_mean: f64,
    code_lines: usize,
    branch_keywords: usize,
    knob_count: usize,
    observations: Vec<CaseObservation>,
}

fn a_star_config() -> AStarConfig {
    AStarConfig {
        resolution: RESOLUTION,
        robot_radius: ROBOT_RADIUS,
        heuristic_weight: 1.0,
    }
}

fn fringe_search_config() -> FringeSearchConfig {
    FringeSearchConfig {
        resolution: RESOLUTION,
        robot_radius: ROBOT_RADIUS,
        heuristic_weight: 1.0,
    }
}

fn ida_star_config() -> IDAStarConfig {
    IDAStarConfig {
        resolution: RESOLUTION,
        robot_radius: ROBOT_RADIUS,
        heuristic_weight: 1.0,
        initial_lookahead_depth: 2,
        max_iterations: 10_000,
        max_expanded_nodes: None,
    }
}

fn bounded_ida_star_config(
    initial_lookahead_depth: usize,
    max_iterations: usize,
    max_expanded_nodes: usize,
) -> IDAStarConfig {
    IDAStarConfig {
        initial_lookahead_depth,
        max_iterations,
        max_expanded_nodes: Some(max_expanded_nodes),
        ..ida_star_config()
    }
}

fn bounded_ida_profiles() -> [BoundedIdaProfile; 3] {
    [
        BoundedIdaProfile {
            id: "look2-iter1-exp500",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 500,
        },
        BoundedIdaProfile {
            id: "look8-iter2-exp2000",
            initial_lookahead_depth: 8,
            max_iterations: 2,
            max_expanded_nodes: 2_000,
        },
        BoundedIdaProfile {
            id: "look8-iter16-exp80000",
            initial_lookahead_depth: 8,
            max_iterations: 16,
            max_expanded_nodes: 80_000,
        },
    ]
}

fn cheap_budget_floor_profiles() -> [BoundedIdaProfile; 5] {
    [
        BoundedIdaProfile {
            id: "look2-iter1-exp18",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 18,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp20",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 20,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp21",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 21,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp22",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 22,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp24",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 24,
        },
    ]
}

fn harder_slice_cheap_budget_floor_profiles() -> [BoundedIdaProfile; 7] {
    [
        BoundedIdaProfile {
            id: "look2-iter1-exp22",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 22,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp128",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 128,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp129",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 129,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp130",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 130,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp131",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 131,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp132",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 132,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp133",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 133,
        },
    ]
}

fn mid_slice_cheap_budget_floor_profiles() -> [BoundedIdaProfile; 3] {
    [
        BoundedIdaProfile {
            id: "look2-iter1-exp42",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 42,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp43",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 43,
        },
        BoundedIdaProfile {
            id: "look2-iter1-exp44",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 44,
        },
    ]
}

fn threshold_round_floor_profiles() -> [BoundedIdaProfile; 16] {
    [
        BoundedIdaProfile {
            id: "look2-iter1-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter2-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 2,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter3-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 3,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter4-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 4,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter6-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 6,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter8-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 8,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter12-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 12,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter16-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 16,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter24-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 24,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter32-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 32,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter48-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 48,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter64-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 64,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter96-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 96,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter192-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 192,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 100_000,
        },
    ]
}

fn bucket15_expansion_floor_profiles() -> [BoundedIdaProfile; 10] {
    [
        BoundedIdaProfile {
            id: "look2-iter128-exp100000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 100_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp125000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 125_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp150000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 150_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp175000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 175_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp200000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 200_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp250000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 250_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp300000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 300_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp400000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 400_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp600000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 600_000,
        },
        BoundedIdaProfile {
            id: "look2-iter128-exp800000",
            initial_lookahead_depth: 2,
            max_iterations: 128,
            max_expanded_nodes: 800_000,
        },
    ]
}

fn bucket15_high_iteration_expansion_profiles() -> [BoundedIdaProfile; 7] {
    [
        BoundedIdaProfile {
            id: "look2-iter256-exp200000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 200_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp250000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 250_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp300000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 300_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp400000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 400_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp600000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 600_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp800000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 800_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp1200000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 1_200_000,
        },
    ]
}

fn bucket15_ultra_high_expansion_profiles() -> [BoundedIdaProfile; 4] {
    [
        BoundedIdaProfile {
            id: "look2-iter256-exp1500000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 1_500_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp2000000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 2_000_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp2500000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 2_500_000,
        },
        BoundedIdaProfile {
            id: "look2-iter256-exp3000000",
            initial_lookahead_depth: 2,
            max_iterations: 256,
            max_expanded_nodes: 3_000_000,
        },
    ]
}

fn bucket15_ultra_high_iteration_profiles() -> [BoundedIdaProfile; 3] {
    [
        BoundedIdaProfile {
            id: "look2-iter512-exp1500000",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_500_000,
        },
        BoundedIdaProfile {
            id: "look2-iter512-exp2000000",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 2_000_000,
        },
        BoundedIdaProfile {
            id: "look2-iter512-exp3000000",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 3_000_000,
        },
    ]
}

fn bucket15_iter512_floor_profiles() -> [BoundedIdaProfile; 2] {
    [
        BoundedIdaProfile {
            id: "look2-iter512-exp1500000",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_500_000,
        },
        BoundedIdaProfile {
            id: "look2-iter512-exp2000000",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 2_000_000,
        },
    ]
}

fn bucket15_iter512_representative_refine_profiles() -> [BoundedIdaProfile; 2] {
    [
        BoundedIdaProfile {
            id: "look2-iter512-exp1750775",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_750_775,
        },
        BoundedIdaProfile {
            id: "look2-iter512-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_750_776,
        },
    ]
}

fn bucket15_iter512_full_slice_refine_profiles() -> [BoundedIdaProfile; 2] {
    [
        BoundedIdaProfile {
            id: "look2-iter512-exp1750775",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_750_775,
        },
        BoundedIdaProfile {
            id: "look2-iter512-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 512,
            max_expanded_nodes: 1_750_776,
        },
    ]
}

fn bucket15_iteration_boundary_profiles() -> [BoundedIdaProfile; 3] {
    [
        BoundedIdaProfile {
            id: "look2-iter273-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 273,
            max_expanded_nodes: 1_750_776,
        },
        BoundedIdaProfile {
            id: "look2-iter274-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 274,
            max_expanded_nodes: 1_750_776,
        },
        BoundedIdaProfile {
            id: "look2-iter275-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 275,
            max_expanded_nodes: 1_750_776,
        },
    ]
}

fn planner_variants() -> [PlannerVariant; 3] {
    [
        PlannerVariant {
            descriptor: VariantDescriptor {
                id: "a-star",
                design_style: "priority-queue-best-first",
                source_path: "src/a_star.rs",
                knob_count: 1,
                reports_dispersion: false,
            },
            evaluate_case: evaluate_a_star_case,
        },
        PlannerVariant {
            descriptor: VariantDescriptor {
                id: "fringe-search",
                design_style: "dual-fringe-threshold",
                source_path: "src/fringe_search.rs",
                knob_count: 1,
                reports_dispersion: false,
            },
            evaluate_case: evaluate_fringe_case,
        },
        PlannerVariant {
            descriptor: VariantDescriptor {
                id: "ida-star",
                design_style: "recursive-contour-threshold",
                source_path: "src/ida_star.rs",
                knob_count: 3,
                reports_dispersion: false,
            },
            evaluate_case: evaluate_ida_case,
        },
    ]
}

fn simple_barrier_obstacles() -> Obstacles {
    let mut obs = Obstacles::new();

    for i in 0..11 {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, 10.0));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(10.0, i as f64));
    }

    for i in 4..7 {
        obs.push(Point2D::new(5.0, i as f64));
    }

    obs
}

fn open_20x20_obstacles() -> Obstacles {
    let mut obstacles = Obstacles::new();
    for i in 0..=20 {
        obstacles.push(Point2D::new(i as f64, 0.0));
        obstacles.push(Point2D::new(i as f64, 20.0));
        obstacles.push(Point2D::new(0.0, i as f64));
        obstacles.push(Point2D::new(20.0, i as f64));
    }
    obstacles
}

fn windowed_moving_ai_bench_case(
    id: &'static str,
    map_str: &'static str,
    scen_str: &'static str,
    bucket: u32,
    slot: usize,
    margin: usize,
    timing_budget: TimingBudget,
) -> ComparisonCase {
    let map = MovingAiMap::parse_str(map_str).expect("benchdata map should parse");
    let scenario = MovingAiScenario::parse_str(scen_str)
        .expect("benchdata scenario should parse")
        .into_iter()
        .filter(|scenario| scenario.bucket == bucket)
        .nth(slot)
        .unwrap_or_else(|| {
            panic!(
                "benchdata scenario bucket {} slot {} should exist",
                bucket, slot
            )
        });

    let min_x = scenario.start_x.min(scenario.goal_x).saturating_sub(margin);
    let min_y = scenario.start_y.min(scenario.goal_y).saturating_sub(margin);
    let max_x = (scenario.start_x.max(scenario.goal_x) + margin).min(map.width - 1);
    let max_y = (scenario.start_y.max(scenario.goal_y) + margin).min(map.height - 1);
    let window_width = max_x - min_x + 1;
    let window_height = max_y - min_y + 1;

    let mut obstacles = Obstacles::new();
    for x in 0..=window_width + 1 {
        obstacles.push(Point2D::new(x as f64, 0.0));
        obstacles.push(Point2D::new(x as f64, (window_height + 1) as f64));
    }
    for y in 0..=window_height + 1 {
        obstacles.push(Point2D::new(0.0, y as f64));
        obstacles.push(Point2D::new((window_width + 1) as f64, y as f64));
    }
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            if !map
                .is_passable(x, y)
                .expect("window coordinates should stay in bounds")
            {
                obstacles.push(Point2D::new((x - min_x + 1) as f64, (y - min_y + 1) as f64));
            }
        }
    }

    ComparisonCase {
        id,
        obstacles,
        start: Point2D::new(
            (scenario.start_x - min_x + 1) as f64,
            (scenario.start_y - min_y + 1) as f64,
        ),
        goal: Point2D::new(
            (scenario.goal_x - min_x + 1) as f64,
            (scenario.goal_y - min_y + 1) as f64,
        ),
        timing_budget,
    }
}

fn comparison_cases() -> Vec<ComparisonCase> {
    let map = MovingAiMap::parse_str(include_str!("../src/testdata/moving_ai/sample.map"))
        .expect("sample MovingAI map should parse");
    let scenario =
        MovingAiScenario::parse_str(include_str!("../src/testdata/moving_ai/sample.map.scen"))
            .expect("sample MovingAI scenarios should parse")
            .into_iter()
            .next()
            .expect("sample MovingAI scenario should exist");

    vec![
        ComparisonCase {
            id: "simple-barrier",
            obstacles: simple_barrier_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(8.0, 8.0),
            timing_budget: TimingBudget::MedianWindows,
        },
        ComparisonCase {
            id: "movingai-sample",
            obstacles: map.to_obstacles(),
            start: map
                .planning_point(scenario.start_x, scenario.start_y)
                .expect("sample start should be valid"),
            goal: map
                .planning_point(scenario.goal_x, scenario.goal_y)
                .expect("sample goal should be valid"),
            timing_budget: TimingBudget::MedianWindows,
        },
        ComparisonCase {
            id: "open-grid",
            obstacles: open_20x20_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(18.0, 18.0),
            timing_budget: TimingBudget::MedianWindows,
        },
    ]
}

fn larger_moving_ai_cases() -> Vec<ComparisonCase> {
    vec![windowed_moving_ai_bench_case(
        "movingai-arena2-b5-window16",
        ARENA2_MAP,
        ARENA2_SCENARIO,
        LARGER_MOVING_AI_BUCKET,
        LARGER_MOVING_AI_SLOT,
        16,
        TimingBudget::SingleShot,
    )]
}

fn larger_moving_ai_window_sweep_cases() -> Vec<ComparisonCase> {
    moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b5",
        bucket: LARGER_MOVING_AI_BUCKET,
        slot: LARGER_MOVING_AI_SLOT,
    })
}

fn harder_moving_ai_window_sweep_cases() -> Vec<ComparisonCase> {
    moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b12",
        bucket: HARDER_MOVING_AI_BUCKET,
        slot: HARDER_MOVING_AI_SLOT,
    })
}

fn moving_ai_window_sweep_cases(slice: MovingAiSliceSpec) -> Vec<ComparisonCase> {
    MOVING_AI_WINDOW_MARGINS
        .into_iter()
        .map(|margin| {
            let id = match margin {
                8 => "window08",
                16 => "window16",
                24 => "window24",
                _ => panic!("unsupported window margin {}", margin),
            };
            let case_id = match (slice.id_prefix, id) {
                ("movingai-arena2-b5", "window08") => "movingai-arena2-b5-window08",
                ("movingai-arena2-b5", "window16") => "movingai-arena2-b5-window16",
                ("movingai-arena2-b5", "window24") => "movingai-arena2-b5-window24",
                ("movingai-arena2-b12", "window08") => "movingai-arena2-b12-window08",
                ("movingai-arena2-b12", "window16") => "movingai-arena2-b12-window16",
                ("movingai-arena2-b12", "window24") => "movingai-arena2-b12-window24",
                ("movingai-arena2-b10", "window08") => "movingai-arena2-b10-window08",
                ("movingai-arena2-b10", "window16") => "movingai-arena2-b10-window16",
                ("movingai-arena2-b10", "window24") => "movingai-arena2-b10-window24",
                ("movingai-arena2-b11", "window08") => "movingai-arena2-b11-window08",
                ("movingai-arena2-b11", "window16") => "movingai-arena2-b11-window16",
                ("movingai-arena2-b11", "window24") => "movingai-arena2-b11-window24",
                ("movingai-arena2-b15", "window08") => "movingai-arena2-b15-window08",
                ("movingai-arena2-b15", "window16") => "movingai-arena2-b15-window16",
                ("movingai-arena2-b15", "window24") => "movingai-arena2-b15-window24",
                _ => panic!("unsupported slice id {} {}", slice.id_prefix, id),
            };

            windowed_moving_ai_bench_case(
                case_id,
                ARENA2_MAP,
                ARENA2_SCENARIO,
                slice.bucket,
                slice.slot,
                margin,
                TimingBudget::SingleShot,
            )
        })
        .collect()
}

fn a_star_reference_length(case: &ComparisonCase) -> f64 {
    let planner = AStarPlanner::from_obstacle_points(&case.obstacles, a_star_config())
        .expect("A* reference planner should build");
    planner
        .plan(case.start, case.goal)
        .unwrap_or_else(|err| panic!("A* reference failed on {}: {}", case.id, err))
        .total_length()
}

fn benchmark_runtime_us(mut run_once: impl FnMut()) -> f64 {
    for _ in 0..TIMING_WARMUP_RUNS {
        run_once();
    }

    let mut samples = Vec::with_capacity(TIMING_WINDOW_SAMPLES);
    for _ in 0..TIMING_WINDOW_SAMPLES {
        let t0 = Instant::now();
        let mut runs = 0usize;
        while runs < TIMING_MIN_RUNS || t0.elapsed() < TIMING_MIN_WINDOW {
            run_once();
            runs += 1;
        }
        samples.push(t0.elapsed().as_secs_f64() * 1_000_000.0 / runs as f64);
    }
    samples.sort_by(f64::total_cmp);
    samples[TIMING_WINDOW_SAMPLES / 2]
}

fn effort_from_a_star(stats: AStarSearchStats) -> SearchEffort {
    SearchEffort {
        expanded_nodes: stats.expanded_nodes,
        unique_expanded_nodes: stats.expanded_nodes,
        reexpanded_nodes: 0,
        generated_nodes: stats.generated_nodes,
        pruned_nodes: stats.skipped_closed_nodes,
        control_rounds: 1,
        peak_live_nodes: stats.max_frontier_len,
    }
}

fn effort_from_fringe(stats: FringeSearchStats) -> SearchEffort {
    SearchEffort {
        expanded_nodes: stats.expanded_nodes,
        unique_expanded_nodes: stats.expanded_nodes,
        reexpanded_nodes: 0,
        generated_nodes: stats.generated_nodes,
        pruned_nodes: stats.deferred_nodes + stats.stale_nodes,
        control_rounds: stats.threshold_iterations,
        peak_live_nodes: stats.max_active_nodes,
    }
}

fn effort_from_ida(stats: IDAStarSearchStats) -> SearchEffort {
    SearchEffort {
        expanded_nodes: stats.expanded_nodes,
        unique_expanded_nodes: stats.unique_expanded_nodes,
        reexpanded_nodes: stats.reexpanded_nodes,
        generated_nodes: stats.generated_nodes,
        pruned_nodes: stats.threshold_prunes + stats.cycle_prunes + stats.transposition_prunes,
        control_rounds: stats.threshold_iterations,
        peak_live_nodes: stats.max_depth,
    }
}

fn evaluate_a_star_case(case: &ComparisonCase) -> CaseObservation {
    let planner = AStarPlanner::from_obstacle_points(&case.obstacles, a_star_config())
        .expect("A* planner should build");
    let (path, stats, runtime_us) = match case.timing_budget {
        TimingBudget::MedianWindows => {
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("A* failed on {}: {}", case.id, err));
            let runtime_us = benchmark_runtime_us(|| {
                black_box(
                    planner
                        .plan_with_stats(black_box(case.start), black_box(case.goal))
                        .unwrap(),
                );
            });
            (path, stats, runtime_us)
        }
        TimingBudget::SingleShot => {
            let t0 = Instant::now();
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("A* failed on {}: {}", case.id, err));
            let runtime_us = t0.elapsed().as_secs_f64() * 1_000_000.0;
            (path, stats, runtime_us)
        }
    };

    CaseObservation {
        case_id: case.id,
        runtime_us,
        path_length: path.total_length(),
        waypoint_count: path.len(),
        length_ratio_vs_astar: 1.0,
        effort: effort_from_a_star(stats),
    }
}

fn evaluate_fringe_case(case: &ComparisonCase) -> CaseObservation {
    let planner =
        FringeSearchPlanner::from_obstacle_points(&case.obstacles, fringe_search_config())
            .expect("Fringe Search planner should build");
    let reference_length = a_star_reference_length(case);
    let (path, stats, runtime_us) = match case.timing_budget {
        TimingBudget::MedianWindows => {
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("Fringe Search failed on {}: {}", case.id, err));
            let runtime_us = benchmark_runtime_us(|| {
                black_box(
                    planner
                        .plan_with_stats(black_box(case.start), black_box(case.goal))
                        .unwrap(),
                );
            });
            (path, stats, runtime_us)
        }
        TimingBudget::SingleShot => {
            let t0 = Instant::now();
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("Fringe Search failed on {}: {}", case.id, err));
            let runtime_us = t0.elapsed().as_secs_f64() * 1_000_000.0;
            (path, stats, runtime_us)
        }
    };

    CaseObservation {
        case_id: case.id,
        runtime_us,
        path_length: path.total_length(),
        waypoint_count: path.len(),
        length_ratio_vs_astar: path.total_length() / reference_length,
        effort: effort_from_fringe(stats),
    }
}

fn evaluate_ida_case(case: &ComparisonCase) -> CaseObservation {
    let planner = IDAStarPlanner::from_obstacle_points(&case.obstacles, ida_star_config())
        .expect("IDA* planner should build");
    let reference_length = a_star_reference_length(case);
    let (path, stats, runtime_us) = match case.timing_budget {
        TimingBudget::MedianWindows => {
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("IDA* failed on {}: {}", case.id, err));
            let runtime_us = benchmark_runtime_us(|| {
                black_box(
                    planner
                        .plan_with_stats(black_box(case.start), black_box(case.goal))
                        .unwrap(),
                );
            });
            (path, stats, runtime_us)
        }
        TimingBudget::SingleShot => {
            let t0 = Instant::now();
            let (path, stats) = planner
                .plan_with_stats(case.start, case.goal)
                .unwrap_or_else(|err| panic!("IDA* failed on {}: {}", case.id, err));
            let runtime_us = t0.elapsed().as_secs_f64() * 1_000_000.0;
            (path, stats, runtime_us)
        }
    };

    CaseObservation {
        case_id: case.id,
        runtime_us,
        path_length: path.total_length(),
        waypoint_count: path.len(),
        length_ratio_vs_astar: path.total_length() / reference_length,
        effort: effort_from_ida(stats),
    }
}

fn classify_bounded_ida_outcome(outcome: IDAStarPlanOutcome) -> &'static str {
    match outcome {
        IDAStarPlanOutcome::Exact => "exact",
        IDAStarPlanOutcome::MaxIterations => "max_iterations",
        IDAStarPlanOutcome::MaxExpandedNodes => "max_expanded_nodes",
        IDAStarPlanOutcome::NoPath => "no_path",
    }
}

fn evaluate_bounded_ida_case(
    case: &ComparisonCase,
    profile: BoundedIdaProfile,
) -> BoundedIdaObservation {
    let planner = IDAStarPlanner::from_obstacle_points(
        &case.obstacles,
        bounded_ida_star_config(
            profile.initial_lookahead_depth,
            profile.max_iterations,
            profile.max_expanded_nodes,
        ),
    )
    .expect("bounded IDA* planner should build");

    let t0 = Instant::now();
    let report = planner
        .plan_with_report(case.start, case.goal)
        .expect("bounded IDA* diagnostics query should stay valid");
    let runtime_us = t0.elapsed().as_secs_f64() * 1_000_000.0;

    BoundedIdaObservation {
        profile_id: profile.id,
        case_id: case.id,
        runtime_us,
        outcome: classify_bounded_ida_outcome(report.outcome),
        path_length: report.path.as_ref().map(|path| path.total_length()),
        effort: effort_from_ida(report.stats),
        waypoint_count: report.path.as_ref().map(|path| path.len()),
        last_searched_threshold: report.last_searched_threshold,
        next_threshold: report.next_threshold,
        contour_history: report.contour_history,
    }
}

fn mean_usize(values: &[usize]) -> f64 {
    values.iter().map(|&value| value as f64).sum::<f64>() / values.len() as f64
}

fn mean_f64(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn threshold_gap(observation: &BoundedIdaObservation) -> Option<f64> {
    observation
        .next_threshold
        .map(|next_threshold| next_threshold - observation.last_searched_threshold)
}

fn one_iteration_probe_profile() -> BoundedIdaProfile {
    BoundedIdaProfile {
        id: "look2-iter1-exp10000",
        initial_lookahead_depth: 2,
        max_iterations: 1,
        max_expanded_nodes: 10_000,
    }
}

fn evaluate_variant(variant: &PlannerVariant, cases: &[ComparisonCase]) -> VariantSummary {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let source_metrics = read_source_metrics(&manifest_dir.join(variant.descriptor.source_path))
        .expect("source metrics should load");

    let observations: Vec<_> = cases
        .iter()
        .map(|case| (variant.evaluate_case)(case))
        .collect();

    VariantSummary {
        id: variant.descriptor.id,
        design_style: variant.descriptor.design_style,
        runtime_us_mean: observations.iter().map(|obs| obs.runtime_us).sum::<f64>()
            / observations.len() as f64,
        waypoint_mean: observations
            .iter()
            .map(|obs| obs.waypoint_count as f64)
            .sum::<f64>()
            / observations.len() as f64,
        length_ratio_mean: observations
            .iter()
            .map(|obs| obs.length_ratio_vs_astar)
            .sum::<f64>()
            / observations.len() as f64,
        expanded_mean: observations
            .iter()
            .map(|obs| obs.effort.expanded_nodes as f64)
            .sum::<f64>()
            / observations.len() as f64,
        generated_mean: observations
            .iter()
            .map(|obs| obs.effort.generated_nodes as f64)
            .sum::<f64>()
            / observations.len() as f64,
        pruned_mean: observations
            .iter()
            .map(|obs| obs.effort.pruned_nodes as f64)
            .sum::<f64>()
            / observations.len() as f64,
        rounds_mean: observations
            .iter()
            .map(|obs| obs.effort.control_rounds as f64)
            .sum::<f64>()
            / observations.len() as f64,
        peak_live_mean: observations
            .iter()
            .map(|obs| obs.effort.peak_live_nodes as f64)
            .sum::<f64>()
            / observations.len() as f64,
        code_lines: source_metrics.code_lines,
        branch_keywords: source_metrics.branch_keywords,
        knob_count: variant.descriptor.knob_count,
        observations,
    }
}

fn print_case_observations(reports: &[VariantSummary]) {
    println!(
        "{:<16} {:<18} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "planner",
        "case",
        "runtime us",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "peak live",
        "wp"
    );
    println!("{}", "-".repeat(133));
    for report in reports {
        for observation in &report.observations {
            println!(
                "{:<16} {:<18} {:>10.2} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                report.id,
                observation.case_id,
                observation.runtime_us,
                observation.path_length,
                observation.effort.expanded_nodes,
                observation.effort.generated_nodes,
                observation.effort.pruned_nodes,
                observation.effort.control_rounds,
                observation.effort.peak_live_nodes,
                observation.waypoint_count,
            );
        }
    }
    println!();
}

fn assert_report_observations(reports: &[VariantSummary], case_label: &str) {
    for report in reports {
        for observation in &report.observations {
            assert!(
                (observation.length_ratio_vs_astar - 1.0).abs() < 1e-6,
                "{} should match A* path length on {} {}, got ratio {}",
                report.id,
                case_label,
                observation.case_id,
                observation.length_ratio_vs_astar
            );
            assert!(
                observation.effort.expanded_nodes > 0,
                "{} should expand at least one node on {} {}",
                report.id,
                case_label,
                observation.case_id
            );
            assert!(
                observation.effort.peak_live_nodes > 0,
                "{} should report a positive peak live set on {} {}",
                report.id,
                case_label,
                observation.case_id
            );
        }
    }
}

#[test]
fn grid_threshold_planners_report_shared_quality_runtime_and_effort() {
    let cases = comparison_cases();
    let variants = planner_variants();
    let reports: Vec<_> = variants
        .iter()
        .map(|variant| evaluate_variant(variant, &cases))
        .collect();

    println!();
    println!(
        "{:<16} {:<28} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8} {:>10} {:>8} {:>9} {:>6}",
        "planner",
        "style",
        "mean us",
        "mean wp",
        "len/A*",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "peak live",
        "LOC",
        "branches",
        "knobs"
    );
    println!("{}", "-".repeat(163));
    for report in &reports {
        println!(
            "{:<16} {:<28} {:>10.2} {:>10.2} {:>10.4} {:>10.2} {:>10.2} {:>10.2} {:>8.2} {:>10.2} {:>8} {:>9} {:>6}",
            report.id,
            report.design_style,
            report.runtime_us_mean,
            report.waypoint_mean,
            report.length_ratio_mean,
            report.expanded_mean,
            report.generated_mean,
            report.pruned_mean,
            report.rounds_mean,
            report.peak_live_mean,
            report.code_lines,
            report.branch_keywords,
            report.knob_count,
        );
    }
    println!();
    print_case_observations(&reports);
    assert_report_observations(&reports, "local");
}

#[test]
fn grid_threshold_planners_spot_check_larger_moving_ai_windows() {
    let case = larger_moving_ai_cases()
        .into_iter()
        .next()
        .expect("larger MovingAI case should exist");

    let a_star_observation = evaluate_a_star_case(&case);
    let fringe_observation = evaluate_fringe_case(&case);
    let a_star_reference_length = a_star_observation.path_length;

    let ida_observation = evaluate_bounded_ida_case(
        &case,
        BoundedIdaProfile {
            id: "look2-iter1-exp500",
            initial_lookahead_depth: 2,
            max_iterations: 1,
            max_expanded_nodes: 500,
        },
    );

    println!("larger MovingAI spot checks (single-shot runtime):");
    println!(
        "{:<16} {:<18} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "planner",
        "case",
        "runtime us",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "peak live",
        "wp"
    );
    println!("{}", "-".repeat(133));
    println!(
        "{:<16} {:<18} {:>10.2} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "a-star",
        case.id,
        a_star_observation.runtime_us,
        a_star_observation.path_length,
        a_star_observation.effort.expanded_nodes,
        a_star_observation.effort.generated_nodes,
        a_star_observation.effort.pruned_nodes,
        a_star_observation.effort.control_rounds,
        a_star_observation.effort.peak_live_nodes,
        a_star_observation.waypoint_count,
    );
    println!(
        "{:<16} {:<18} {:>10.2} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "fringe-search",
        case.id,
        fringe_observation.runtime_us,
        fringe_observation.path_length,
        fringe_observation.effort.expanded_nodes,
        fringe_observation.effort.generated_nodes,
        fringe_observation.effort.pruned_nodes,
        fringe_observation.effort.control_rounds,
        fringe_observation.effort.peak_live_nodes,
        fringe_observation.waypoint_count,
    );

    match ida_observation.path_length {
        Some(path_length) => {
            let effort = ida_observation.effort;
            println!(
                "{:<16} {:<18} {:>10.2} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                "ida-star-bounded",
                case.id,
                ida_observation.runtime_us,
                path_length,
                effort.expanded_nodes,
                effort.generated_nodes,
                effort.pruned_nodes,
                effort.control_rounds,
                effort.peak_live_nodes,
                ida_observation
                    .waypoint_count
                    .expect("exact bounded IDA* run should report waypoint count"),
            );
            assert!(
                (path_length - a_star_reference_length).abs() < 1e-6,
                "bounded IDA* should still match A* when it completes on {}",
                case.id
            );
        }
        None => {
            println!(
                "{:<16} {:<18} {:>10.2} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                "ida-star-bounded",
                case.id,
                ida_observation.runtime_us,
                "ERR",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
            );
            println!("bounded error: {}", ida_observation.outcome);
            assert!(
                ida_observation.outcome == "max_expanded_nodes"
                    || ida_observation.outcome == "max_iterations",
                "bounded IDA* should stop on a configured cutoff, got {}",
                ida_observation.outcome
            );
        }
    }

    assert!(
        (a_star_observation.path_length - a_star_reference_length).abs() < 1e-6,
        "A* observation should define the larger-case reference on {}",
        case.id
    );
    assert!(
        (fringe_observation.path_length - a_star_reference_length).abs() < 1e-6,
        "Fringe Search should still match A* on larger {}",
        case.id
    );
}

#[test]
fn grid_threshold_planners_sweep_window_margin_and_bounded_ida_cutoffs() {
    let cases = larger_moving_ai_window_sweep_cases();
    let profiles = bounded_ida_profiles();
    let mut exact_count = 0usize;
    let mut bounded_stop_count = 0usize;

    println!("larger MovingAI bounded IDA* sweep (single-shot runtime):");
    println!(
        "{:<18} {:<14} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "wp"
    );
    println!("{}", "-".repeat(138));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let fringe_observation = evaluate_fringe_case(case);
        let a_star_reference_length = a_star_observation.path_length;

        assert!(
            (fringe_observation.path_length - a_star_reference_length).abs() < 1e-6,
            "Fringe Search should still match A* on larger {}",
            case.id
        );

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("exact bounded IDA* run should report waypoint count");
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.generated_nodes,
                        effort.pruned_nodes,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - a_star_reference_length).abs() < 1e-6,
                        "bounded IDA* exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_count += 1;
                }
                None => {
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                    );
                    assert!(
                        observation.outcome == "max_expanded_nodes"
                            || observation.outcome == "max_iterations",
                        "bounded IDA* should stop on a configured cutoff, got {}",
                        observation.outcome
                    );
                    bounded_stop_count += 1;
                }
            }
        }
    }

    println!(
        "bounded sweep summary: exact={}, bounded-stop={}",
        exact_count, bounded_stop_count
    );
    assert!(
        exact_count == cases.len() * profiles.len(),
        "all current bounded IDA* sweep runs should complete exactly"
    );
}

#[test]
fn grid_threshold_planners_find_cheap_bounded_ida_exact_floor() {
    let cases = larger_moving_ai_window_sweep_cases();
    let profiles = cheap_budget_floor_profiles();
    let mut exact_budgets = Vec::new();
    let mut stopped_budgets = Vec::new();

    println!("cheap bounded IDA* budget floor sweep (single-shot runtime):");
    println!(
        "{:<18} {:<14} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "wp"
    );
    println!("{}", "-".repeat(138));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let reference_length = a_star_observation.path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("exact bounded IDA* run should report waypoint count");
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.generated_nodes,
                        effort.pruned_nodes,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "cheap bounded IDA* exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                    );
                    assert_eq!(
                        observation.outcome, "max_expanded_nodes",
                        "cheap budget sweep should stop on expansion budget, got {}",
                        observation.outcome
                    );
                    stopped_budgets.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = *exact_budgets
        .iter()
        .min()
        .expect("cheap budget sweep should include exact runs");
    let max_stop = *stopped_budgets
        .iter()
        .max()
        .expect("cheap budget sweep should include bounded-stop runs");

    println!(
        "cheap budget sweep summary: min exact budget={}, max bounded-stop budget={}",
        exact_floor, max_stop
    );

    assert_eq!(
        exact_floor, 22,
        "current cheap exact floor should be 22 expanded nodes"
    );
    assert_eq!(
        max_stop, 21,
        "current largest bounded-stop budget should be 21 expanded nodes"
    );
}

#[test]
fn grid_threshold_planners_find_harder_slice_cheap_bounded_ida_exact_floor() {
    let cases = harder_moving_ai_window_sweep_cases();
    let profiles = harder_slice_cheap_budget_floor_profiles();
    let mut exact_budgets = Vec::new();
    let mut stopped_budgets = Vec::new();

    println!("harder-slice cheap bounded IDA* budget floor sweep (single-shot runtime):");
    println!(
        "{:<18} {:<14} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "wp"
    );
    println!("{}", "-".repeat(138));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let reference_length = a_star_observation.path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("exact bounded IDA* run should report waypoint count");
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.generated_nodes,
                        effort.pruned_nodes,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "harder-slice cheap bounded IDA* exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                    );
                    assert_eq!(
                        observation.outcome, "max_expanded_nodes",
                        "harder-slice cheap budget sweep should stop on expansion budget, got {}",
                        observation.outcome
                    );
                    stopped_budgets.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = *exact_budgets
        .iter()
        .min()
        .expect("harder-slice cheap budget sweep should include exact runs");
    let max_stop = *stopped_budgets
        .iter()
        .max()
        .expect("harder-slice cheap budget sweep should include bounded-stop runs");

    println!(
        "harder-slice cheap budget summary: min exact budget={}, max bounded-stop budget={}",
        exact_floor, max_stop
    );

    assert_eq!(
        exact_floor, 132,
        "harder-slice cheap exact floor should be 132 expanded nodes"
    );
    assert_eq!(
        max_stop, 131,
        "harder-slice largest bounded-stop budget should be 131 expanded nodes"
    );
}

#[test]
fn grid_threshold_planners_compare_additional_harder_slice_one_iteration_reachability() {
    let slices = [
        ("movingai-arena2-b10", 10u32, 0usize, "max_iterations"),
        ("movingai-arena2-b11", 11u32, 0usize, "exact"),
        ("movingai-arena2-b15", 15u32, 0usize, "max_iterations"),
    ];

    println!("additional harder-slice one-iteration reachability probes:");
    println!(
        "{:<18} {:<18} {:<18} {:>10} {:>10} {:>10} {:>8}",
        "slice", "case", "look2-iter1", "path len", "expanded", "rounds", "wp"
    );
    println!("{}", "-".repeat(98));

    for (id_prefix, bucket, slot, expected_outcome) in slices {
        let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
            id_prefix,
            bucket,
            slot,
        });
        let mut outcomes = Vec::new();

        for case in &cases {
            let one_iter = evaluate_bounded_ida_case(case, one_iteration_probe_profile());
            let exact_reference = evaluate_a_star_case(case);
            println!(
                "{:<18} {:<18} {:<18} {:>10.4} {:>10} {:>10} {:>8}",
                id_prefix,
                case.id,
                one_iter.outcome,
                exact_reference.path_length,
                exact_reference.effort.expanded_nodes,
                exact_reference.effort.control_rounds,
                exact_reference.waypoint_count,
            );
            outcomes.push(one_iter.outcome);
        }

        let unique_outcome = outcomes
            .first()
            .copied()
            .expect("slice should include at least one case");
        assert!(
            outcomes.iter().all(|outcome| *outcome == unique_outcome),
            "additional harder-slice outcomes should stay consistent within {}",
            id_prefix
        );
        println!(
            "slice summary: {} one-iteration outcome={}",
            id_prefix, unique_outcome
        );
        assert_eq!(
            unique_outcome, expected_outcome,
            "update additional harder-slice reachability expectation for {}",
            id_prefix
        );
    }
}

#[test]
fn grid_threshold_planners_find_mid_slice_cheap_bounded_ida_exact_floor() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b11",
        bucket: 11,
        slot: 0,
    });
    let profiles = mid_slice_cheap_budget_floor_profiles();
    let mut exact_budgets = Vec::new();
    let mut stopped_budgets = Vec::new();

    println!("mid-slice cheap bounded IDA* budget floor sweep (single-shot runtime):");
    println!(
        "{:<18} {:<14} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "generated",
        "pruned",
        "rounds",
        "wp"
    );
    println!("{}", "-".repeat(138));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let reference_length = a_star_observation.path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("mid-slice exact bounded IDA* run should report waypoint count");
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.generated_nodes,
                        effort.pruned_nodes,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "mid-slice cheap bounded IDA* exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<14} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                        "-",
                    );
                    assert_eq!(
                        observation.outcome, "max_expanded_nodes",
                        "mid-slice cheap budget sweep should stop on expansion budget, got {}",
                        observation.outcome
                    );
                    stopped_budgets.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = *exact_budgets
        .iter()
        .min()
        .expect("mid-slice cheap budget sweep should include exact runs");
    let max_stop = *stopped_budgets
        .iter()
        .max()
        .expect("mid-slice cheap budget sweep should include bounded-stop runs");

    println!(
        "mid-slice cheap budget summary: min exact budget={}, max bounded-stop budget={}",
        exact_floor, max_stop
    );

    assert_eq!(
        exact_floor, 43,
        "mid-slice cheap exact floor should be 43 expanded nodes"
    );
    assert_eq!(
        max_stop, 42,
        "mid-slice largest bounded-stop budget should be 42 expanded nodes"
    );
}

#[test]
fn grid_threshold_planners_find_threshold_round_exact_floors_on_harder_slices() {
    let slices = [
        (
            "movingai-arena2-b10",
            10u32,
            0usize,
            32usize,
            24usize,
            0usize,
        ),
        (
            "movingai-arena2-b15",
            15u32,
            0usize,
            0usize,
            96usize,
            128usize,
        ),
    ];
    let profiles = threshold_round_floor_profiles();

    println!("threshold-round exact floor sweep on harder slices:");
    println!(
        "{:<18} {:<18} {:<20} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "slice", "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(116));

    for (
        id_prefix,
        bucket,
        slot,
        expected_exact_floor,
        expected_max_iteration_stop,
        expected_min_expansion_stop,
    ) in slices
    {
        let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
            id_prefix,
            bucket,
            slot,
        });
        let mut exact_iterations = Vec::new();
        let mut iteration_stops = Vec::new();
        let mut expansion_stops = Vec::new();

        for case in &cases {
            let a_star_observation = evaluate_a_star_case(case);
            let reference_length = a_star_observation.path_length;

            for profile in profiles {
                let observation = evaluate_bounded_ida_case(case, profile);
                match observation.path_length {
                    Some(path_length) => {
                        let effort = observation.effort;
                        let waypoint_count = observation
                            .waypoint_count
                            .expect("threshold-round exact run should report waypoint count");
                        println!(
                            "{:<18} {:<18} {:<20} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                            id_prefix,
                            observation.case_id,
                            observation.profile_id,
                            observation.runtime_us,
                            observation.outcome,
                            path_length,
                            effort.control_rounds,
                            waypoint_count,
                        );
                        assert!(
                            (path_length - reference_length).abs() < 1e-6,
                            "threshold-round exact run should match A* on {} {}",
                            observation.case_id,
                            observation.profile_id
                        );
                        exact_iterations.push(profile.max_iterations);
                    }
                    None => {
                        println!(
                            "{:<18} {:<18} {:<20} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                            id_prefix,
                            observation.case_id,
                            observation.profile_id,
                            observation.runtime_us,
                            observation.outcome,
                            "-",
                            "-",
                            "-",
                        );
                        assert!(
                            observation.outcome == "max_iterations"
                                || observation.outcome == "max_expanded_nodes",
                            "threshold-round sweep should stop on iteration or expansion budget, got {}",
                            observation.outcome
                        );
                        if observation.outcome == "max_iterations" {
                            iteration_stops.push(profile.max_iterations);
                        } else {
                            expansion_stops.push(profile.max_iterations);
                        }
                    }
                }
            }
        }

        let exact_floor = exact_iterations.iter().min().copied();
        let max_iteration_stop = iteration_stops.iter().max().copied();
        let min_expansion_stop = expansion_stops.iter().min().copied();

        match exact_floor {
            Some(exact_floor) => {
                println!(
                    "threshold-round summary: {} min exact iterations={}, max iteration-stop={}, min expansion-stop={}",
                    id_prefix,
                    exact_floor,
                    max_iteration_stop.unwrap_or(0),
                    min_expansion_stop.unwrap_or(0),
                );
                if expected_exact_floor == 0
                    && expected_max_iteration_stop == 0
                    && expected_min_expansion_stop == 0
                {
                    continue;
                }
                assert_eq!(
                    exact_floor, expected_exact_floor,
                    "update threshold-round exact floor expectation for {}",
                    id_prefix
                );
                assert_eq!(
                    max_iteration_stop.unwrap_or(0),
                    expected_max_iteration_stop,
                    "update threshold-round iteration-stop ceiling for {}",
                    id_prefix
                );
                assert_eq!(
                    min_expansion_stop.unwrap_or(0),
                    expected_min_expansion_stop,
                    "update threshold-round expansion-stop floor for {}",
                    id_prefix
                );
            }
            None => {
                println!(
                    "threshold-round summary: {} no exact run through iteration {}, max iteration-stop={}, min expansion-stop={}",
                    id_prefix,
                    max_iteration_stop.or(min_expansion_stop).unwrap_or(0),
                    max_iteration_stop.unwrap_or(0),
                    min_expansion_stop.unwrap_or(0),
                );
                if expected_exact_floor == 0
                    && expected_max_iteration_stop == 0
                    && expected_min_expansion_stop == 0
                {
                    continue;
                }
                assert_eq!(
                    expected_exact_floor, 0,
                    "threshold-round sweep found no exact run for {}; set placeholder exact floor to 0 until expanded",
                    id_prefix
                );
                assert_eq!(
                    max_iteration_stop.unwrap_or(0),
                    expected_max_iteration_stop,
                    "update threshold-round iteration-stop ceiling for {}",
                    id_prefix
                );
                assert_eq!(
                    min_expansion_stop.unwrap_or(0),
                    expected_min_expansion_stop,
                    "update threshold-round expansion-stop floor for {}",
                    id_prefix
                );
            }
        }
    }
}

#[test]
fn grid_threshold_planners_find_bucket15_expansion_exact_floor_after_iteration_ramp() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    let profiles = bucket15_expansion_floor_profiles();
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 expansion floor after iteration ramp:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let reference_length = a_star_observation.path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("bucket-15 expansion exact run should report waypoint count");
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "bucket-15 expansion exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                    );
                    assert!(
                        matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                        "bucket-15 expansion sweep should stay bounded, got {}",
                        observation.outcome
                    );
                    if observation.outcome == "max_expanded_nodes" {
                        expansion_stops.push(profile.max_expanded_nodes);
                    } else {
                        iteration_stops.push(profile.max_expanded_nodes);
                    }
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = *expansion_stops
        .iter()
        .max()
        .expect("bucket-15 expansion sweep should include expansion-stop runs");
    let min_iteration_stop = *iteration_stops
        .iter()
        .min()
        .expect("bucket-15 expansion sweep should include iteration-stop runs");

    println!(
        "bucket-15 expansion summary: no exact run through budget {}, max expansion-stop budget={}, min iteration-stop budget={}",
        profiles
            .iter()
            .map(|profile| profile.max_expanded_nodes)
            .max()
            .expect("bucket-15 expansion profiles should be non-empty"),
        max_expansion_stop,
        min_iteration_stop,
    );

    assert!(
        exact_floor.is_none(),
        "bucket-15 expansion sweep should not reach exact within iter128 budgets"
    );
    assert_eq!(
        max_expansion_stop, 200_000,
        "bucket-15 max expansion-stop budget should stay at 200000"
    );
    assert_eq!(
        min_iteration_stop, 250_000,
        "bucket-15 min iteration-stop budget should stay at 250000"
    );
}

#[test]
fn grid_threshold_planners_bound_bucket15_high_iteration_expansion_non_exact_band() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    let profiles = bucket15_high_iteration_expansion_profiles();
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 high-iteration expansion sweep:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for case in &cases {
        let a_star_observation = evaluate_a_star_case(case);
        let reference_length = a_star_observation.path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("bucket-15 high-iteration exact run should report waypoint count");
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "bucket-15 high-iteration exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                    );
                    assert!(
                        matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                        "bucket-15 high-iteration sweep should stay bounded, got {}",
                        observation.outcome
                    );
                    if observation.outcome == "max_expanded_nodes" {
                        expansion_stops.push(profile.max_expanded_nodes);
                    } else {
                        iteration_stops.push(profile.max_expanded_nodes);
                    }
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();

    println!(
        "bucket-15 high-iteration summary: min exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        exact_floor,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert!(
        exact_floor.is_none(),
        "bucket-15 high-iteration sweep should stay non-exact through iter256-exp1200000"
    );
    assert_eq!(
        max_expansion_stop,
        Some(1_200_000),
        "bucket-15 high-iteration max expansion-stop budget should stay at 1200000"
    );
    assert!(
        min_iteration_stop.is_none(),
        "bucket-15 high-iteration sweep should not hit max_iterations in the tested band"
    );
}

#[test]
#[ignore = "expensive representative bucket-15 ultra-high expansion probe"]
fn grid_threshold_planners_bound_bucket15_window16_ultra_high_expansion_band() {
    let case = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    })
    .into_iter()
    .find(|case| case.id == "movingai-arena2-b15-window16")
    .expect("bucket-15 representative window16 case should exist");
    let profiles = bucket15_ultra_high_expansion_profiles();
    let reference_length = evaluate_a_star_case(&case).path_length;
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 representative ultra-high expansion probe:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for profile in profiles {
        let observation = evaluate_bounded_ida_case(&case, profile);
        match observation.path_length {
            Some(path_length) => {
                let effort = observation.effort;
                let waypoint_count = observation
                    .waypoint_count
                    .expect("bucket-15 representative exact run should report waypoint count");
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    path_length,
                    effort.control_rounds,
                    waypoint_count,
                );
                assert!(
                    (path_length - reference_length).abs() < 1e-6,
                    "bucket-15 representative exact run should match A* on {}",
                    observation.profile_id
                );
                exact_budgets.push(profile.max_expanded_nodes);
            }
            None => {
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    "-",
                    "-",
                    "-",
                );
                assert!(
                    matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                    "bucket-15 representative ultra-high probe should stay bounded, got {}",
                    observation.outcome
                );
                if observation.outcome == "max_expanded_nodes" {
                    expansion_stops.push(profile.max_expanded_nodes);
                } else {
                    iteration_stops.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();

    println!(
        "bucket-15 representative ultra-high summary: min exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        exact_floor,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert!(
        exact_floor.is_none(),
        "bucket-15 representative ultra-high expansion band should stay non-exact through iter256-exp3000000"
    );
    assert_eq!(
        max_expansion_stop, None,
        "bucket-15 representative ultra-high expansion band should not hit max_expanded_nodes in the tested range"
    );
    assert_eq!(
        min_iteration_stop,
        Some(1_500_000),
        "bucket-15 representative ultra-high expansion band should flip to max_iterations by exp1500000"
    );
}

#[test]
#[ignore = "expensive representative bucket-15 ultra-high iteration probe"]
fn grid_threshold_planners_probe_bucket15_window16_ultra_high_iteration_band() {
    let case = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    })
    .into_iter()
    .find(|case| case.id == "movingai-arena2-b15-window16")
    .expect("bucket-15 representative window16 case should exist");
    let profiles = bucket15_ultra_high_iteration_profiles();
    let reference_length = evaluate_a_star_case(&case).path_length;
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 representative ultra-high iteration probe:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for profile in profiles {
        let observation = evaluate_bounded_ida_case(&case, profile);
        match observation.path_length {
            Some(path_length) => {
                let effort = observation.effort;
                let waypoint_count = observation.waypoint_count.expect(
                    "bucket-15 representative iter512 exact run should report waypoint count",
                );
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    path_length,
                    effort.control_rounds,
                    waypoint_count,
                );
                assert!(
                    (path_length - reference_length).abs() < 1e-6,
                    "bucket-15 representative iter512 exact run should match A* on {}",
                    observation.profile_id
                );
                exact_budgets.push(profile.max_expanded_nodes);
            }
            None => {
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    "-",
                    "-",
                    "-",
                );
                assert!(
                    matches!(
                        observation.outcome,
                        "max_expanded_nodes" | "max_iterations"
                    ),
                    "bucket-15 representative ultra-high iteration probe should stay bounded, got {}",
                    observation.outcome
                );
                if observation.outcome == "max_expanded_nodes" {
                    expansion_stops.push(profile.max_expanded_nodes);
                } else {
                    iteration_stops.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();

    println!(
        "bucket-15 representative ultra-high iteration summary: min exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        exact_floor,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert_eq!(
        exact_floor,
        Some(2_000_000),
        "bucket-15 representative ultra-high iteration exact floor should be exp2000000"
    );
    assert_eq!(
        max_expansion_stop,
        Some(1_500_000),
        "bucket-15 representative ultra-high iteration max expansion-stop budget should be exp1500000"
    );
    assert!(
        min_iteration_stop.is_none(),
        "bucket-15 representative ultra-high iteration probe should not hit max_iterations in the tested range"
    );
}

#[test]
#[ignore = "expensive representative bucket-15 iter512 budget refinement"]
fn grid_threshold_planners_probe_bucket15_window16_iter512_budget_refinement() {
    let case = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    })
    .into_iter()
    .find(|case| case.id == "movingai-arena2-b15-window16")
    .expect("bucket-15 representative window16 case should exist");
    let profiles = bucket15_iter512_representative_refine_profiles();
    let reference_length = evaluate_a_star_case(&case).path_length;
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 representative iter512 budget refinement:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for profile in profiles {
        let observation = evaluate_bounded_ida_case(&case, profile);
        match observation.path_length {
            Some(path_length) => {
                let effort = observation.effort;
                let waypoint_count = observation
                    .waypoint_count
                    .expect("bucket-15 representative iter512 refine exact run should report waypoint count");
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    path_length,
                    effort.control_rounds,
                    waypoint_count,
                );
                assert!(
                    (path_length - reference_length).abs() < 1e-6,
                    "bucket-15 representative iter512 refine exact run should match A* on {}",
                    observation.profile_id
                );
                exact_budgets.push(profile.max_expanded_nodes);
            }
            None => {
                println!(
                    "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                    observation.case_id,
                    observation.profile_id,
                    observation.runtime_us,
                    observation.outcome,
                    "-",
                    "-",
                    "-",
                );
                assert!(
                    matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                    "bucket-15 representative iter512 refine should stay bounded, got {}",
                    observation.outcome
                );
                if observation.outcome == "max_expanded_nodes" {
                    expansion_stops.push(profile.max_expanded_nodes);
                } else {
                    iteration_stops.push(profile.max_expanded_nodes);
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();

    println!(
        "bucket-15 representative iter512 refine summary: min exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        exact_floor,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert_eq!(
        exact_floor,
        Some(1_750_776),
        "bucket-15 representative iter512 refine exact floor should be exp1750776"
    );
    assert_eq!(
        max_expansion_stop,
        Some(1_750_775),
        "bucket-15 representative iter512 refine max expansion-stop budget should be exp1750775"
    );
    assert!(
        min_iteration_stop.is_none(),
        "bucket-15 representative iter512 refine should not hit max_iterations in the tested range"
    );
}

#[test]
#[ignore = "expensive full-slice bucket-15 iter512 floor confirmation"]
fn grid_threshold_planners_bound_bucket15_iter512_budget_floor_on_full_slice() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    let profiles = bucket15_iter512_floor_profiles();
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();

    println!("bucket-15 iter512 floor confirmation:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>8}",
        "case", "ida profile", "runtime us", "outcome", "path len", "rounds", "wp"
    );
    println!("{}", "-".repeat(104));

    for case in &cases {
        let reference_length = evaluate_a_star_case(case).path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("bucket-15 iter512 exact run should report waypoint count");
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "bucket-15 iter512 exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                }
                None => {
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        "-",
                        "-",
                    );
                    assert!(
                        matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                        "bucket-15 iter512 floor confirmation should stay bounded, got {}",
                        observation.outcome
                    );
                    if observation.outcome == "max_expanded_nodes" {
                        expansion_stops.push(profile.max_expanded_nodes);
                    } else {
                        iteration_stops.push(profile.max_expanded_nodes);
                    }
                }
            }
        }
    }

    let exact_floor = exact_budgets.iter().min().copied();
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();

    println!(
        "bucket-15 iter512 floor summary: min exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        exact_floor,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert_eq!(
        exact_floor,
        Some(2_000_000),
        "bucket-15 iter512 floor exact budget should be exp2000000"
    );
    assert_eq!(
        max_expansion_stop,
        Some(1_500_000),
        "bucket-15 iter512 floor max expansion-stop budget should be exp1500000"
    );
    assert!(
        min_iteration_stop.is_none(),
        "bucket-15 iter512 floor confirmation should not hit max_iterations in the tested range"
    );
}

#[test]
#[ignore = "expensive full-slice bucket-15 iter512 budget refinement"]
fn grid_threshold_planners_probe_bucket15_iter512_budget_refinement_on_full_slice() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    let profiles = bucket15_iter512_full_slice_refine_profiles();
    let mut exact_budgets = Vec::new();
    let mut expansion_stops = Vec::new();
    let mut iteration_stops = Vec::new();
    let mut exact_counts_by_budget = HashMap::new();
    let mut non_exact_counts_by_budget = HashMap::new();
    let mut observations_by_budget: HashMap<usize, Vec<BoundedIdaObservation>> = HashMap::new();

    println!("bucket-15 iter512 budget refinement on full slice:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "unique",
        "reexp",
        "rounds",
        "wp"
    );
    println!("{}", "-".repeat(137));

    for case in &cases {
        let reference_length = evaluate_a_star_case(case).path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            observations_by_budget
                .entry(profile.max_expanded_nodes)
                .or_default()
                .push(observation.clone());
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation
                        .waypoint_count
                        .expect("bucket-15 iter512 refine exact run should report waypoint count");
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.unique_expanded_nodes,
                        effort.reexpanded_nodes,
                        effort.control_rounds,
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "bucket-15 iter512 refine exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    exact_budgets.push(profile.max_expanded_nodes);
                    *exact_counts_by_budget
                        .entry(profile.max_expanded_nodes)
                        .or_insert(0usize) += 1;
                }
                None => {
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        observation.effort.expanded_nodes,
                        observation.effort.unique_expanded_nodes,
                        observation.effort.reexpanded_nodes,
                        observation.effort.control_rounds,
                        "-",
                    );
                    assert!(
                        matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                        "bucket-15 iter512 refine should stay bounded, got {}",
                        observation.outcome
                    );
                    if observation.outcome == "max_expanded_nodes" {
                        expansion_stops.push(profile.max_expanded_nodes);
                    } else {
                        iteration_stops.push(profile.max_expanded_nodes);
                    }
                    *non_exact_counts_by_budget
                        .entry(profile.max_expanded_nodes)
                        .or_insert(0usize) += 1;
                }
            }
        }
    }

    println!();
    println!(
        "{:<22} {:>8} {:>12} {:>12} {:>12} {:>10}",
        "ida profile", "exact", "mean exp", "mean unique", "mean reexp", "mean rnds",
    );
    println!("{}", "-".repeat(84));
    for profile in profiles {
        let observations = observations_by_budget
            .get(&profile.max_expanded_nodes)
            .expect("bucket-15 refine observations should be grouped by budget");
        let expanded: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.expanded_nodes)
            .collect();
        let unique: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.unique_expanded_nodes)
            .collect();
        let reexpanded: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.reexpanded_nodes)
            .collect();
        let rounds: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.control_rounds)
            .collect();
        println!(
            "{:<22} {:>8} {:>12.1} {:>12.1} {:>12.1} {:>10.1}",
            profile.id,
            observations
                .iter()
                .filter(|observation| observation.outcome == "exact")
                .count(),
            mean_usize(&expanded),
            mean_usize(&unique),
            mean_usize(&reexpanded),
            mean_usize(&rounds),
        );
    }

    let full_exact_floor = profiles
        .iter()
        .map(|profile| profile.max_expanded_nodes)
        .find(|budget| {
            exact_counts_by_budget
                .get(budget)
                .copied()
                .unwrap_or_default()
                == cases.len()
        });
    let max_expansion_stop = expansion_stops.iter().max().copied();
    let min_iteration_stop = iteration_stops.iter().min().copied();
    let max_non_exact_budget = profiles
        .iter()
        .map(|profile| profile.max_expanded_nodes)
        .filter(|budget| non_exact_counts_by_budget.contains_key(budget))
        .max();

    println!(
        "bucket-15 iter512 refine summary: full-slice exact floor={:?}, max non-exact budget={:?}, max expansion-stop budget={:?}, min iteration-stop budget={:?}",
        full_exact_floor,
        max_non_exact_budget,
        max_expansion_stop,
        min_iteration_stop,
    );

    assert_eq!(
        full_exact_floor,
        Some(1_750_776),
        "bucket-15 iter512 refine full-slice exact floor should be exp1750776"
    );
    assert_eq!(
        max_non_exact_budget,
        Some(1_750_775),
        "bucket-15 iter512 refine max non-exact budget should be exp1750775"
    );
    assert_eq!(
        max_expansion_stop,
        Some(1_750_775),
        "bucket-15 iter512 refine max expansion-stop budget should be exp1750775"
    );
    assert!(
        min_iteration_stop.is_none(),
        "bucket-15 iter512 refine on full slice should not hit max_iterations in the tested range"
    );
}

#[test]
#[ignore = "expensive full-slice bucket-15 iteration boundary probe"]
fn grid_threshold_planners_probe_bucket15_iteration_boundary_on_full_slice() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    let profiles = bucket15_iteration_boundary_profiles();
    let mut exact_counts_by_iteration = HashMap::new();
    let mut non_exact_counts_by_iteration = HashMap::new();
    let mut expansion_stops_by_iteration = HashMap::new();
    let mut iteration_stops_by_iteration = HashMap::new();
    let mut observations_by_iteration: HashMap<usize, Vec<BoundedIdaObservation>> = HashMap::new();

    println!("bucket-15 iteration boundary probe on full slice:");
    println!(
        "{:<18} {:<22} {:>10} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "case",
        "ida profile",
        "runtime us",
        "outcome",
        "path len",
        "expanded",
        "unique",
        "reexp",
        "rounds",
        "next gap",
        "wp"
    );
    println!("{}", "-".repeat(149));

    for case in &cases {
        let reference_length = evaluate_a_star_case(case).path_length;

        for profile in profiles {
            let observation = evaluate_bounded_ida_case(case, profile);
            observations_by_iteration
                .entry(profile.max_iterations)
                .or_default()
                .push(observation.clone());
            match observation.path_length {
                Some(path_length) => {
                    let effort = observation.effort;
                    let waypoint_count = observation.waypoint_count.expect(
                        "bucket-15 iteration-boundary exact run should report waypoint count",
                    );
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10.4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        path_length,
                        effort.expanded_nodes,
                        effort.unique_expanded_nodes,
                        effort.reexpanded_nodes,
                        effort.control_rounds,
                        "-",
                        waypoint_count,
                    );
                    assert!(
                        (path_length - reference_length).abs() < 1e-6,
                        "bucket-15 iteration-boundary exact run should match A* on {} {}",
                        observation.case_id,
                        observation.profile_id
                    );
                    *exact_counts_by_iteration
                        .entry(profile.max_iterations)
                        .or_insert(0usize) += 1;
                }
                None => {
                    println!(
                        "{:<18} {:<22} {:>10.2} {:>12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10.6} {:>8}",
                        observation.case_id,
                        observation.profile_id,
                        observation.runtime_us,
                        observation.outcome,
                        "-",
                        observation.effort.expanded_nodes,
                        observation.effort.unique_expanded_nodes,
                        observation.effort.reexpanded_nodes,
                        observation.effort.control_rounds,
                        threshold_gap(&observation)
                            .expect("iteration-boundary failures should report the next threshold gap"),
                        "-",
                    );
                    assert!(
                        matches!(observation.outcome, "max_expanded_nodes" | "max_iterations"),
                        "bucket-15 iteration-boundary probe should stay bounded, got {}",
                        observation.outcome
                    );
                    *non_exact_counts_by_iteration
                        .entry(profile.max_iterations)
                        .or_insert(0usize) += 1;
                    if observation.outcome == "max_expanded_nodes" {
                        *expansion_stops_by_iteration
                            .entry(profile.max_iterations)
                            .or_insert(0usize) += 1;
                    } else {
                        *iteration_stops_by_iteration
                            .entry(profile.max_iterations)
                            .or_insert(0usize) += 1;
                    }
                }
            }
        }
    }

    println!();
    println!(
        "{:<22} {:>8} {:>12} {:>12} {:>12} {:>10} {:>12}",
        "ida profile", "exact", "mean exp", "mean unique", "mean reexp", "mean rnds", "mean gap",
    );
    println!("{}", "-".repeat(97));
    for profile in profiles {
        let observations = observations_by_iteration
            .get(&profile.max_iterations)
            .expect("bucket-15 boundary observations should be grouped by iteration");
        let expanded: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.expanded_nodes)
            .collect();
        let unique: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.unique_expanded_nodes)
            .collect();
        let reexpanded: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.reexpanded_nodes)
            .collect();
        let rounds: Vec<_> = observations
            .iter()
            .map(|observation| observation.effort.control_rounds)
            .collect();
        let gaps: Vec<_> = observations.iter().filter_map(threshold_gap).collect();
        println!(
            "{:<22} {:>8} {:>12.1} {:>12.1} {:>12.1} {:>10.1} {:>12}",
            profile.id,
            observations
                .iter()
                .filter(|observation| observation.outcome == "exact")
                .count(),
            mean_usize(&expanded),
            mean_usize(&unique),
            mean_usize(&reexpanded),
            mean_usize(&rounds),
            if gaps.is_empty() {
                "-".to_string()
            } else {
                format!("{:.6}", mean_f64(&gaps))
            },
        );
    }

    let full_exact_floor =
        profiles
            .iter()
            .map(|profile| profile.max_iterations)
            .find(|iterations| {
                exact_counts_by_iteration
                    .get(iterations)
                    .copied()
                    .unwrap_or_default()
                    == cases.len()
            });
    let max_non_exact_iteration = profiles
        .iter()
        .map(|profile| profile.max_iterations)
        .filter(|iterations| non_exact_counts_by_iteration.contains_key(iterations))
        .max();
    let max_expansion_stop_iteration = profiles
        .iter()
        .map(|profile| profile.max_iterations)
        .filter(|iterations| expansion_stops_by_iteration.contains_key(iterations))
        .max();
    let max_iteration_stop_iteration = profiles
        .iter()
        .map(|profile| profile.max_iterations)
        .filter(|iterations| iteration_stops_by_iteration.contains_key(iterations))
        .max();

    println!(
        "bucket-15 iteration boundary summary: full-slice exact floor={:?}, max non-exact iteration={:?}, max expansion-stop iteration={:?}, max iteration-stop iteration={:?}",
        full_exact_floor,
        max_non_exact_iteration,
        max_expansion_stop_iteration,
        max_iteration_stop_iteration,
    );

    assert_eq!(
        full_exact_floor,
        Some(275),
        "bucket-15 fixed-budget iteration boundary exact floor should be iter275"
    );
    assert_eq!(
        max_non_exact_iteration,
        Some(274),
        "bucket-15 fixed-budget iteration boundary max non-exact iteration should be iter274"
    );
    assert!(
        max_expansion_stop_iteration.is_none(),
        "bucket-15 fixed-budget iteration boundary should not hit max_expanded_nodes in the tested range"
    );
    assert_eq!(
        max_iteration_stop_iteration,
        Some(274),
        "bucket-15 fixed-budget iteration boundary max iteration-stop iteration should be iter274"
    );
}

#[test]
fn grid_threshold_planners_probe_bucket15_contour_diagnostics() {
    let cases = moving_ai_window_sweep_cases(MovingAiSliceSpec {
        id_prefix: "movingai-arena2-b15",
        bucket: 15,
        slot: 0,
    });
    // Use the representative window08 case (first case) with iter274 (fails)
    // and iter275 (succeeds) to show per-contour breakdown at the boundary.
    let case = &cases[0];

    let profiles = [
        BoundedIdaProfile {
            id: "look2-iter274-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 274,
            max_expanded_nodes: 1_750_776,
        },
        BoundedIdaProfile {
            id: "look2-iter275-exp1750776",
            initial_lookahead_depth: 2,
            max_iterations: 275,
            max_expanded_nodes: 1_750_776,
        },
    ];

    println!(
        "bucket-15 per-contour diagnostics on {} (iter274 vs iter275):",
        case.id
    );
    println!();

    for profile in profiles {
        let observation = evaluate_bounded_ida_case(case, profile);
        let contours = &observation.contour_history;
        let n = contours.len();

        println!(
            "profile={} outcome={} total_contours={} total_expanded={} total_reexpanded={}",
            profile.id,
            observation.outcome,
            n,
            observation.effort.expanded_nodes,
            observation.effort.reexpanded_nodes,
        );

        // Print summary stats: first 5 contours, last 5 contours, and
        // the contour with the most re-expansions.
        println!(
            "  {:>6} {:>12} {:>8} {:>8} {:>10} {:>8} {:>10} {:>8} {:>8} {:>8}",
            "round",
            "threshold",
            "expand",
            "new_uniq",
            "reexpand",
            "gen",
            "thr_prune",
            "cyc_pr",
            "trn_pr",
            "depth",
        );
        println!("  {}", "-".repeat(100));

        let print_contour = |i: usize, c: &ContourStats| {
            println!(
                "  {:>6} {:>12.4} {:>8} {:>8} {:>10} {:>8} {:>10} {:>8} {:>8} {:>8}",
                i + 1,
                c.threshold,
                c.expanded,
                c.new_unique,
                c.reexpanded,
                c.generated,
                c.threshold_prunes,
                c.cycle_prunes,
                c.transposition_prunes,
                c.max_depth,
            );
        };

        let head = 5.min(n);
        let tail_start = if n > 10 { n - 5 } else { head };

        for (i, c) in contours.iter().enumerate().take(head) {
            print_contour(i, c);
        }
        if tail_start > head {
            println!("  ... ({} contours omitted) ...", tail_start - head);
        }
        for (i, c) in contours.iter().enumerate().take(n).skip(tail_start) {
            print_contour(i, c);
        }

        // Aggregated contour growth analysis
        if n >= 2 {
            let total_reexp: usize = contours.iter().map(|c| c.reexpanded).sum();
            let total_exp: usize = contours.iter().map(|c| c.expanded).sum();
            let reexp_ratio = if total_exp > 0 {
                total_reexp as f64 / total_exp as f64
            } else {
                0.0
            };
            let max_reexp_contour = contours
                .iter()
                .enumerate()
                .max_by_key(|(_, c)| c.reexpanded)
                .map(|(i, _)| i);
            let last_expanded = contours.last().map(|c| c.expanded).unwrap_or(0);
            let second_last_expanded = if n >= 2 { contours[n - 2].expanded } else { 0 };
            let growth_ratio = if second_last_expanded > 0 {
                last_expanded as f64 / second_last_expanded as f64
            } else {
                f64::NAN
            };

            println!();
            println!(
                "  reexpansion ratio: {:.4} ({}/{}), peak reexp contour: round {}",
                reexp_ratio,
                total_reexp,
                total_exp,
                max_reexp_contour.map(|i| i + 1).unwrap_or(0),
            );
            println!(
                "  last two contour expansion: round {}={}, round {}={}, growth ratio: {:.4}",
                n - 1,
                second_last_expanded,
                n,
                last_expanded,
                growth_ratio,
            );

            // Threshold progression: gap between successive contour thresholds
            let threshold_gaps: Vec<f64> = contours
                .windows(2)
                .map(|w| w[1].threshold - w[0].threshold)
                .collect();
            let min_gap = threshold_gaps.iter().copied().fold(f64::INFINITY, f64::min);
            let max_gap = threshold_gaps
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            let mean_gap = if threshold_gaps.is_empty() {
                0.0
            } else {
                threshold_gaps.iter().sum::<f64>() / threshold_gaps.len() as f64
            };

            println!(
                "  threshold gap: min={:.6} max={:.6} mean={:.6}",
                min_gap, max_gap, mean_gap,
            );
        }
        println!();
    }
}

struct MultiMapSpec {
    name: &'static str,
    map_str: &'static str,
    scen_str: &'static str,
    bucket: u32,
}

#[test]
fn grid_threshold_planners_multi_map_spot_check() {
    let specs = [
        MultiMapSpec {
            name: "dao-arena2",
            map_str: ARENA2_MAP,
            scen_str: ARENA2_SCENARIO,
            bucket: 5,
        },
        MultiMapSpec {
            name: "maze-512",
            map_str: MAZE_MAP,
            scen_str: MAZE_SCENARIO,
            bucket: 5,
        },
        MultiMapSpec {
            name: "room-8",
            map_str: ROOM_MAP,
            scen_str: ROOM_SCENARIO,
            bucket: 5,
        },
        MultiMapSpec {
            name: "random-512",
            map_str: RANDOM_MAP,
            scen_str: RANDOM_SCENARIO,
            bucket: 5,
        },
        MultiMapSpec {
            name: "street-berlin",
            map_str: STREET_MAP,
            scen_str: STREET_SCENARIO,
            bucket: 5,
        },
    ];

    println!(
        "{:<20} {:<12} {:>10} {:>10} {:>8} {:>8} {:>8} {:>8}",
        "map", "planner", "runtime us", "path len", "expand", "unique", "reexp", "rounds",
    );
    println!("{}", "-".repeat(90));

    for spec in &specs {
        let case = windowed_moving_ai_bench_case(
            spec.name,
            spec.map_str,
            spec.scen_str,
            spec.bucket,
            0,
            16,
            TimingBudget::SingleShot,
        );

        let a_star_obs = evaluate_a_star_case(&case);
        let fringe_obs = evaluate_fringe_case(&case);
        let ida_obs = evaluate_ida_case(&case);

        for (planner_name, obs) in [
            ("a-star", &a_star_obs),
            ("fringe-search", &fringe_obs),
            ("ida-star", &ida_obs),
        ] {
            println!(
                "{:<20} {:<12} {:>10.2} {:>10.4} {:>8} {:>8} {:>8} {:>8}",
                spec.name,
                planner_name,
                obs.runtime_us,
                obs.path_length,
                obs.effort.expanded_nodes,
                obs.effort.unique_expanded_nodes,
                obs.effort.reexpanded_nodes,
                obs.effort.control_rounds,
            );
        }

        // All planners should find paths of equal length
        assert!(
            (a_star_obs.path_length - fringe_obs.path_length).abs() < 1e-6,
            "fringe-search path length should match A* on {}: {} vs {}",
            spec.name,
            fringe_obs.path_length,
            a_star_obs.path_length,
        );
        assert!(
            (a_star_obs.path_length - ida_obs.path_length).abs() < 1e-6,
            "IDA* path length should match A* on {}: {} vs {}",
            spec.name,
            ida_obs.path_length,
            a_star_obs.path_length,
        );
        println!();
    }
}
