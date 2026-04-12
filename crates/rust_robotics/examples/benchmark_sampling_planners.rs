//! Benchmark sampling-based planners on the same 2D environment.
//!
//! Run with:
//!   cargo run -p rust_robotics --example benchmark_sampling_planners --features planning --release

use std::time::Instant;

use rust_robotics::planning::bidirectional_rrt::{
    AreaBounds as BiAreaBounds, BidirectionalRRTConfig, BidirectionalRRTPlanner,
    CircleObstacle as BiCircleObstacle,
};
use rust_robotics::planning::fmt_star::{
    AreaBounds as FmtAreaBounds, CircleObstacle as FmtCircleObstacle, FMTStarConfig, FMTStarPlanner,
};
use rust_robotics::planning::rrg::{
    AreaBounds as RrgAreaBounds, CircleObstacle as RrgCircleObstacle, RRGConfig, RRGPlanner,
};
use rust_robotics::planning::rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};
use rust_robotics::planning::rrt_star::RRTStar;
use rust_robotics::prelude::*;

const RUNS: usize = 20;

#[derive(Debug)]
struct SamplingResult {
    planner: &'static str,
    avg_time_ms: f64,
    avg_path_length: f64,
    success_rate: f64,
}

fn path_length_xy(path: &[[f64; 2]]) -> f64 {
    path.windows(2)
        .map(|w| {
            let dx = w[1][0] - w[0][0];
            let dy = w[1][1] - w[0][1];
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

fn circular_obstacles() -> Vec<(f64, f64, f64)> {
    vec![
        (5.0, 5.0, 1.2),
        (3.2, 6.8, 1.5),
        (3.5, 9.5, 1.4),
        (7.5, 5.0, 1.6),
        (9.0, 7.5, 1.3),
    ]
}

fn main() {
    let start = Point2D::new(0.0, 0.0);
    let goal = Point2D::new(10.0, 11.0);
    let bounds = (-2.0, 15.0, -2.0, 15.0);
    let raw_obs = circular_obstacles();

    let mut results = Vec::new();

    {
        let obstacles: Vec<CircleObstacle> = raw_obs
            .iter()
            .map(|&(x, y, r)| CircleObstacle::new(x, y, r))
            .collect();
        let rand_area = AreaBounds::new(bounds.0, bounds.1, bounds.2, bounds.3);
        let config = RRTConfig {
            max_iter: 1200,
            ..Default::default()
        };

        let mut total_ms = 0.0;
        let mut success = 0usize;
        let mut total_len = 0.0;
        for _ in 0..RUNS {
            let planner =
                RRTPlanner::new(obstacles.clone(), rand_area.clone(), None, config.clone());
            let t0 = Instant::now();
            let path = planner.plan(start, goal);
            total_ms += t0.elapsed().as_secs_f64() * 1000.0;
            if let Ok(path) = path {
                success += 1;
                total_len += path.total_length();
            }
        }
        results.push(SamplingResult {
            planner: "RRT",
            avg_time_ms: total_ms / RUNS as f64,
            avg_path_length: if success > 0 {
                total_len / success as f64
            } else {
                f64::NAN
            },
            success_rate: 100.0 * success as f64 / RUNS as f64,
        });
    }

    {
        let obstacles = raw_obs.clone();
        let mut total_ms = 0.0;
        let mut success = 0usize;
        let mut total_len = 0.0;
        for _ in 0..RUNS {
            let mut planner = RRTStar::new(
                (start.x, start.y),
                (goal.x, goal.y),
                obstacles.clone(),
                (bounds.0, bounds.1),
                2.0,
                0.5,
                20,
                1200,
                50.0,
                false,
                0.8,
            );
            let t0 = Instant::now();
            let path = planner.planning();
            total_ms += t0.elapsed().as_secs_f64() * 1000.0;
            if let Some(path) = path {
                success += 1;
                total_len += path_length_xy(&path);
            }
        }
        results.push(SamplingResult {
            planner: "RRT*",
            avg_time_ms: total_ms / RUNS as f64,
            avg_path_length: if success > 0 {
                total_len / success as f64
            } else {
                f64::NAN
            },
            success_rate: 100.0 * success as f64 / RUNS as f64,
        });
    }

    {
        let obstacles: Vec<BiCircleObstacle> = raw_obs
            .iter()
            .map(|&(x, y, r)| BiCircleObstacle::new(x, y, r))
            .collect();
        let rand_area = BiAreaBounds::new(bounds.0, bounds.1, bounds.2, bounds.3);
        let config = BidirectionalRRTConfig {
            max_iter: 1500,
            ..Default::default()
        };

        let mut total_ms = 0.0;
        let mut success = 0usize;
        let mut total_len = 0.0;
        for _ in 0..RUNS {
            let planner =
                BidirectionalRRTPlanner::new(obstacles.clone(), rand_area.clone(), config.clone());
            let t0 = Instant::now();
            let path = planner.plan(start, goal);
            total_ms += t0.elapsed().as_secs_f64() * 1000.0;
            if let Ok(path) = path {
                success += 1;
                total_len += path.total_length();
            }
        }
        results.push(SamplingResult {
            planner: "Bidirectional RRT",
            avg_time_ms: total_ms / RUNS as f64,
            avg_path_length: if success > 0 {
                total_len / success as f64
            } else {
                f64::NAN
            },
            success_rate: 100.0 * success as f64 / RUNS as f64,
        });
    }

    {
        let obstacles: Vec<RrgCircleObstacle> = raw_obs
            .iter()
            .map(|&(x, y, r)| RrgCircleObstacle::new(x, y, r))
            .collect();
        let rand_area = RrgAreaBounds::new(bounds.0, bounds.1, bounds.2, bounds.3);
        let config = RRGConfig {
            max_iter: 1200,
            ..Default::default()
        };

        let mut total_ms = 0.0;
        let mut success = 0usize;
        let mut total_len = 0.0;
        for _ in 0..RUNS {
            let planner = RRGPlanner::new(obstacles.clone(), rand_area.clone(), config.clone());
            let t0 = Instant::now();
            let path = planner.plan(start, goal);
            total_ms += t0.elapsed().as_secs_f64() * 1000.0;
            if let Ok(path) = path {
                success += 1;
                total_len += path.total_length();
            }
        }
        results.push(SamplingResult {
            planner: "RRG",
            avg_time_ms: total_ms / RUNS as f64,
            avg_path_length: if success > 0 {
                total_len / success as f64
            } else {
                f64::NAN
            },
            success_rate: 100.0 * success as f64 / RUNS as f64,
        });
    }

    {
        let obstacles: Vec<FmtCircleObstacle> = raw_obs
            .iter()
            .map(|&(x, y, r)| FmtCircleObstacle::new(x, y, r))
            .collect();
        let rand_area = FmtAreaBounds::new(bounds.0, bounds.1, bounds.2, bounds.3);
        let config = FMTStarConfig {
            n_samples: 700,
            connection_radius: 3.5,
            robot_radius: 0.8,
        };

        let mut total_ms = 0.0;
        let mut success = 0usize;
        let mut total_len = 0.0;
        for _ in 0..RUNS {
            let planner = FMTStarPlanner::new(obstacles.clone(), rand_area.clone(), config);
            let t0 = Instant::now();
            let path = planner.plan(start, goal);
            total_ms += t0.elapsed().as_secs_f64() * 1000.0;
            if let Ok(path) = path {
                success += 1;
                total_len += path.total_length();
            }
        }
        results.push(SamplingResult {
            planner: "FMT*",
            avg_time_ms: total_ms / RUNS as f64,
            avg_path_length: if success > 0 {
                total_len / success as f64
            } else {
                f64::NAN
            },
            success_rate: 100.0 * success as f64 / RUNS as f64,
        });
    }

    println!("| Planner | Avg Time (ms) | Avg Path Length | Success Rate (%) |");
    println!("|---------|--------------:|----------------:|-----------------:|");
    for row in results {
        println!(
            "| {} | {:.3} | {:.3} | {:.1} |",
            row.planner, row.avg_time_ms, row.avg_path_length, row.success_rate
        );
    }
}
