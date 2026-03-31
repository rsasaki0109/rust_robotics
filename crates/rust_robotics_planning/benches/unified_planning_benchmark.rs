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
use rust_robotics_planning::greedy_best_first_search::{GreedyBestFirstConfig, GreedyBestFirstPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use rust_robotics_planning::theta_star::{ThetaStarConfig, ThetaStarPlanner};
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

/// Maze grid: boundary walls plus several internal walls with gaps.
fn maze_obstacles() -> Obstacles {
    let mut obs = open_grid_obstacles();

    // Vertical wall at x=12, gap at y=20..25
    for y in 1..GRID_SIZE {
        if !(20..=25).contains(&y) {
            obs.push(Point2D::new(12.0, y as f64));
        }
    }
    // Vertical wall at x=25, gap at y=30..35
    for y in 1..GRID_SIZE {
        if !(30..=35).contains(&y) {
            obs.push(Point2D::new(25.0, y as f64));
        }
    }
    // Horizontal wall at y=15, gap at x=30..35
    for x in 1..GRID_SIZE {
        if !(30..=35).contains(&x) {
            obs.push(Point2D::new(x as f64, 15.0));
        }
    }
    // Vertical wall at x=38, gap at y=5..10
    for y in 1..GRID_SIZE {
        if !(5..=10).contains(&y) {
            obs.push(Point2D::new(38.0, y as f64));
        }
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
            let hash = ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519))
                % 100;
            if hash < 20 {
                obs.push(Point2D::new(x as f64, y as f64));
            }
        }
    }
    obs
}

struct Scenario {
    name: &'static str,
    obstacles: Obstacles,
    start: Point2D,
    goal: Point2D,
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "open_50x50",
            obstacles: open_grid_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(48.0, 48.0),
        },
        Scenario {
            name: "maze_50x50",
            obstacles: maze_obstacles(),
            start: Point2D::new(2.0, 25.0),
            goal: Point2D::new(48.0, 25.0),
        },
        Scenario {
            name: "dense_50x50",
            obstacles: dense_obstacles(),
            start: Point2D::new(2.0, 2.0),
            goal: Point2D::new(48.0, 48.0),
        },
    ]
}

// ---------------------------------------------------------------------------
// Summary table collection
// ---------------------------------------------------------------------------

struct RunResult {
    planner: &'static str,
    scenario: &'static str,
    path_length: f64,
    waypoints: usize,
}

static RESULTS: Mutex<Vec<RunResult>> = Mutex::new(Vec::new());

fn record(planner: &'static str, scenario: &'static str, path: &Path2D) {
    RESULTS.lock().unwrap().push(RunResult {
        planner,
        scenario,
        path_length: path.total_length(),
        waypoints: path.len(),
    });
}

fn print_summary() {
    let results = RESULTS.lock().unwrap();
    if results.is_empty() {
        return;
    }
    eprintln!();
    eprintln!("======================================================================");
    eprintln!("  Unified Planning Benchmark -- Path Summary");
    eprintln!("======================================================================");
    eprintln!(
        "{:<22} {:<16} {:>10} {:>10}",
        "Planner", "Scenario", "Length", "Waypoints"
    );
    eprintln!("{:-<60}", "");
    for r in results.iter() {
        eprintln!(
            "{:<22} {:<16} {:>10.2} {:>10}",
            r.planner, r.scenario, r.path_length, r.waypoints
        );
    }
    eprintln!("======================================================================");
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
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
            Some(Box::new(move |s, g| p.plan(s, g).unwrap())),
        ));
    } else {
        planners.push(("FlowField", None));
    }

    planners
}

// ---------------------------------------------------------------------------
// Benchmark
// ---------------------------------------------------------------------------

fn bench_unified(c: &mut Criterion) {
    for scenario in scenarios() {
        let planners = build_planners(&scenario.obstacles);

        let mut group = c.benchmark_group(scenario.name);
        group.sample_size(20);

        for (name, plan_fn) in &planners {
            if let Some(plan) = plan_fn {
                // Record path info once outside the timed loop.
                let path = plan(scenario.start, scenario.goal);
                record(name, scenario.name, &path);

                group.bench_with_input(BenchmarkId::new(*name, ""), &(), |b, _| {
                    b.iter(|| plan(black_box(scenario.start), black_box(scenario.goal)));
                });
            } else {
                eprintln!("[SKIP] {} on {} (planner construction failed)", name, scenario.name);
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
