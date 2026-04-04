//! Speed comparison benchmark for Rust implementations.
//!
//! Outputs CSV to stdout: algorithm,rust_ms,runs
//!
//! Run with:
//!   cargo run -p rust_robotics --example speed_comparison --features full --release

use std::time::Instant;

use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::planning::a_star::AStarPlanner;
use rust_robotics::planning::cubic_spline_planner::calc_spline_course;
use rust_robotics::planning::rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};

/// Build a 100x100 grid with boundary walls and an internal wall.
fn create_grid_obstacles() -> (Vec<f64>, Vec<f64>) {
    let mut ox = Vec::new();
    let mut oy = Vec::new();

    // Boundary
    for i in 0..=100 {
        ox.push(i as f64);
        oy.push(0.0);
        ox.push(i as f64);
        oy.push(100.0);
        ox.push(0.0);
        oy.push(i as f64);
        ox.push(100.0);
        oy.push(i as f64);
    }

    // Internal wall
    for i in 20..80 {
        ox.push(50.0);
        oy.push(i as f64);
    }

    (ox, oy)
}

/// Benchmark A* pathfinding on a 100x100 grid.
#[allow(deprecated)]
fn bench_a_star(runs: usize) -> f64 {
    let (ox, oy) = create_grid_obstacles();
    let planner = AStarPlanner::from_obstacles(&ox, &oy, 1.0, 0.5);

    let start = Instant::now();
    for _ in 0..runs {
        let _ = planner.planning(10.0, 10.0, 90.0, 90.0);
    }
    start.elapsed().as_secs_f64() * 1000.0 / runs as f64
}

/// Benchmark RRT on a simple obstacle scenario.
fn bench_rrt(runs: usize) -> f64 {
    let obstacles = vec![
        CircleObstacle::new(5.0, 5.0, 1.0),
        CircleObstacle::new(3.0, 6.0, 2.0),
        CircleObstacle::new(3.0, 8.0, 2.0),
        CircleObstacle::new(3.0, 10.0, 2.0),
        CircleObstacle::new(7.0, 5.0, 2.0),
        CircleObstacle::new(9.0, 5.0, 2.0),
        CircleObstacle::new(8.0, 10.0, 1.0),
    ];
    let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
    let config = RRTConfig {
        expand_dis: 3.0,
        path_resolution: 0.5,
        goal_sample_rate: 5,
        max_iter: 500,
        robot_radius: 0.8,
    };

    let start = Instant::now();
    for _ in 0..runs {
        let mut planner =
            RRTPlanner::new(obstacles.clone(), rand_area.clone(), None, config.clone());
        let _ = planner.planning([0.0, 0.0], [6.0, 10.0]);
    }
    start.elapsed().as_secs_f64() * 1000.0 / runs as f64
}

/// Benchmark EKF localization (predict + update steps).
fn bench_ekf(steps: usize) -> f64 {
    let mut ekf = EKFLocalizer::new(EKFConfig::default());
    let control = nalgebra::Vector2::new(1.0, 0.1);
    let dt = 0.1;

    let start = Instant::now();
    for i in 0..steps {
        let t = i as f64 * dt;
        // Simulated GPS measurement: forward motion along x + small drift
        let measurement = nalgebra::Vector2::new(t + 0.01 * i as f64, 0.005 * i as f64);
        let _ = ekf.estimate(&measurement, &control, dt);
    }
    start.elapsed().as_secs_f64() * 1000.0
}

/// Benchmark cubic spline interpolation.
fn bench_cubic_spline(runs: usize) -> f64 {
    let x = vec![-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
    let y = vec![0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0];
    let ds = 0.1;

    let start = Instant::now();
    for _ in 0..runs {
        let _ = calc_spline_course(x.clone(), y.clone(), ds);
    }
    start.elapsed().as_secs_f64() * 1000.0 / runs as f64
}

fn main() {
    eprintln!("Running Rust speed benchmarks...");

    let a_star_runs = 100;
    let rrt_runs = 100;
    let ekf_steps = 1000;
    let spline_runs = 1000;

    let a_star_ms = bench_a_star(a_star_runs);
    eprintln!("  A* done: {:.3} ms/run", a_star_ms);

    let rrt_ms = bench_rrt(rrt_runs);
    eprintln!("  RRT done: {:.3} ms/run", rrt_ms);

    let ekf_ms = bench_ekf(ekf_steps);
    eprintln!("  EKF done: {:.3} ms total ({} steps)", ekf_ms, ekf_steps);

    let spline_ms = bench_cubic_spline(spline_runs);
    eprintln!("  CubicSpline done: {:.3} ms/run", spline_ms);

    // CSV output
    println!("algorithm,rust_ms,runs");
    println!("a_star,{:.6},{}", a_star_ms, a_star_runs);
    println!("rrt,{:.6},{}", rrt_ms, rrt_runs);
    println!("ekf,{:.6},{}", ekf_ms, ekf_steps);
    println!("cubic_spline,{:.6},{}", spline_ms, spline_runs);
}
