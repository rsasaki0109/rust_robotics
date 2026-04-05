//! JPS vs A* Grid Size Crossover Benchmark
//!
//! Tests JPS and A* on grids of increasing size (50, 100, 200, 500, 1000)
//! with open, maze-like, and dense obstacle patterns to find the crossover
//! point where JPS becomes faster than A*.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rust_robotics_core::{Obstacles, Path2D, Point2D};
use rust_robotics_planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics_planning::jps::{JPSConfig, JPSPlanner};
use std::sync::Mutex;

const RESOLUTION: f64 = 1.0;
const ROBOT_RADIUS: f64 = 0.5;

fn boundary_obstacles(size: i32) -> Obstacles {
    let mut obs = Obstacles::new();
    for i in 0..=size {
        obs.push(Point2D::new(i as f64, 0.0));
        obs.push(Point2D::new(i as f64, size as f64));
        obs.push(Point2D::new(0.0, i as f64));
        obs.push(Point2D::new(size as f64, i as f64));
    }
    obs
}

fn dense_obstacles(size: i32) -> Obstacles {
    let mut obs = boundary_obstacles(size);
    for x in 2..size - 1 {
        for y in 2..size - 1 {
            let hash =
                ((x as u64).wrapping_mul(2654435761) ^ (y as u64).wrapping_mul(2246822519)) % 100;
            if hash < 20 {
                obs.push(Point2D::new(x as f64, y as f64));
            }
        }
    }
    obs
}

fn maze_obstacles(size: i32) -> Obstacles {
    let mut obs = boundary_obstacles(size);
    let wall_spacing = (size / 5).max(4);
    let gap_size = 3;
    let mut is_horizontal = true;

    let mut pos = wall_spacing;
    while pos < size - wall_spacing {
        if is_horizontal {
            let gap_start = ((pos as u64).wrapping_mul(2654435761) % (size as u64 - gap_size as u64 - 4)) as i32 + 2;
            for x in 2..size - 1 {
                if x < gap_start || x >= gap_start + gap_size {
                    obs.push(Point2D::new(x as f64, pos as f64));
                }
            }
        } else {
            let gap_start = ((pos as u64).wrapping_mul(2246822519) % (size as u64 - gap_size as u64 - 4)) as i32 + 2;
            for y in 2..size - 1 {
                if y < gap_start || y >= gap_start + gap_size {
                    obs.push(Point2D::new(pos as f64, y as f64));
                }
            }
        }
        is_horizontal = !is_horizontal;
        pos += wall_spacing;
    }
    obs
}

struct CrossoverResult {
    grid_size: i32,
    scenario: &'static str,
    planner: &'static str,
    path_length: f64,
    waypoints: usize,
}

static RESULTS: Mutex<Vec<CrossoverResult>> = Mutex::new(Vec::new());

fn record(grid_size: i32, scenario: &'static str, planner: &'static str, path: &Path2D) {
    RESULTS.lock().unwrap().push(CrossoverResult {
        grid_size,
        scenario,
        planner,
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
    eprintln!("  JPS vs A* Grid Size Crossover -- Path Summary");
    eprintln!("======================================================================");
    eprintln!(
        "{:<8} {:<12} {:<8} {:>12} {:>10}",
        "Size", "Scenario", "Planner", "PathLength", "Waypoints"
    );
    eprintln!("{:-<54}", "");
    for r in results.iter() {
        eprintln!(
            "{:<8} {:<12} {:<8} {:>12.2} {:>10}",
            r.grid_size, r.scenario, r.planner, r.path_length, r.waypoints
        );
    }
    eprintln!("======================================================================");
}

fn bench_crossover(c: &mut Criterion) {
    let sizes = [50, 100, 200, 500, 1000];
    #[allow(clippy::type_complexity)]
    let scenarios: Vec<(&str, Box<dyn Fn(i32) -> Obstacles>)> = vec![
        ("open", Box::new(boundary_obstacles)),
        ("dense", Box::new(dense_obstacles)),
        ("maze", Box::new(maze_obstacles)),
    ];

    for &size in &sizes {
        for (scenario_name, obstacle_fn) in &scenarios {
            let obstacles = obstacle_fn(size);
            let start = Point2D::new(2.0, 2.0);
            // Use (size-3) to avoid hitting a dense-grid obstacle at (size-2, size-2)
            let goal = Point2D::new((size - 3) as f64, (size - 3) as f64);

            let group_name = format!("{scenario_name}_{size}x{size}");
            let mut group = c.benchmark_group(&group_name);
            group.sample_size(10);

            // A*
            if let Ok(planner) = AStarPlanner::from_obstacle_points(
                &obstacles,
                AStarConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                    heuristic_weight: 1.0,
                },
            ) {
                if let Ok(path) = planner.plan(start, goal) {
                    record(size, scenario_name, "A*", &path);
                    group.bench_with_input(BenchmarkId::new("A*", ""), &(), |b, _| {
                        b.iter(|| planner.plan(black_box(start), black_box(goal)));
                    });
                }
            }

            // JPS
            if let Ok(planner) = JPSPlanner::from_obstacle_points(
                &obstacles,
                JPSConfig {
                    resolution: RESOLUTION,
                    robot_radius: ROBOT_RADIUS,
                    heuristic_weight: 1.0,
                },
            ) {
                if let Ok(path) = planner.plan(start, goal) {
                    record(size, scenario_name, "JPS", &path);
                    group.bench_with_input(BenchmarkId::new("JPS", ""), &(), |b, _| {
                        b.iter(|| planner.plan(black_box(start), black_box(goal)));
                    });
                }
            }

            group.finish();
        }
    }
}

fn bench_and_summarize(c: &mut Criterion) {
    bench_crossover(c);
    print_summary();
}

criterion_group!(benches, bench_and_summarize);
criterion_main!(benches);
