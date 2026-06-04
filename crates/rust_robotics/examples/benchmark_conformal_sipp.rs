//! CP-SIPP confidence sweep benchmark.
//!
//! Run with:
//!   cargo run -p rust_robotics --example benchmark_conformal_sipp --no-default-features --features planning

use std::time::Instant;

use rust_robotics::planning::conformal_sipp::{
    ConformalSippConfig, ConformalSippPlanner, PredictedObstaclePoint, PredictedObstacleTrajectory,
};
use rust_robotics::planning::sipp::{
    DynamicObstacle, Interval, SippConfig, SippPlanner, TimedWaypoint,
};
use rust_robotics::prelude::*;

const RUNS: usize = 200;

#[derive(Debug)]
struct SweepRow {
    planner: String,
    arrival_t: u64,
    path_length: f64,
    off_center_steps: usize,
    min_confidence: Option<f64>,
    violation_bound: Option<f64>,
    avg_plan_us: f64,
}

fn empty_map(width: i32, height: i32) -> Vec<Vec<bool>> {
    vec![vec![false; height as usize]; width as usize]
}

fn repeated_scores(time_horizon: u64, scores: &[f64]) -> Vec<Vec<f64>> {
    (0..=time_horizon).map(|_| scores.to_vec()).collect()
}

fn prediction() -> PredictedObstacleTrajectory {
    PredictedObstacleTrajectory::new(vec![
        PredictedObstaclePoint::new(4, 4.0, 2.0),
        PredictedObstaclePoint::new(5, 4.0, 2.0),
    ])
}

fn path_length(path: &[TimedWaypoint]) -> f64 {
    path.windows(2)
        .map(|window| {
            let dx = window[1].x as f64 - window[0].x as f64;
            let dy = window[1].y as f64 - window[0].y as f64;
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

fn off_center_steps(path: &[TimedWaypoint], center_y: i32) -> usize {
    path.iter()
        .filter(|waypoint| waypoint.y != center_y)
        .count()
}

fn main() -> RoboticsResult<()> {
    let width = 9;
    let height = 5;
    let start = (0, 2);
    let goal = (8, 2);
    let time_horizon = 16;
    let obstacle_map = empty_map(width, height);
    let calibration_scores = repeated_scores(time_horizon, &[0.10, 0.60, 1.20]);

    let nominal_sipp = SippPlanner::new(SippConfig {
        width,
        height,
        obstacle_map: obstacle_map.clone(),
        dynamic_obstacles: vec![DynamicObstacle {
            x: 4,
            y: 2,
            interval: Interval::new(4, 6),
        }],
        allow_diagonal: false,
    })?;

    let mut rows = Vec::new();
    let mut nominal_plan = Vec::new();
    let start_time = Instant::now();
    for _ in 0..RUNS {
        nominal_plan = nominal_sipp.plan(start.0, start.1, goal.0, goal.1)?;
    }
    let avg_plan_us = start_time.elapsed().as_secs_f64() * 1_000_000.0 / RUNS as f64;
    rows.push(SweepRow {
        planner: "Exact SIPP".to_string(),
        arrival_t: nominal_plan.last().map(|wp| wp.t).unwrap_or_default(),
        path_length: path_length(&nominal_plan),
        off_center_steps: off_center_steps(&nominal_plan, start.1),
        min_confidence: None,
        violation_bound: None,
        avg_plan_us,
    });

    for required_confidence in [0.0, 0.5, 0.7, 1.0] {
        let planner = ConformalSippPlanner::new(ConformalSippConfig {
            width,
            height,
            obstacle_map: obstacle_map.clone(),
            predicted_obstacles: vec![prediction()],
            calibration_errors_by_time: calibration_scores.clone(),
            time_horizon,
            required_confidence,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        })?;

        let mut plan = planner.plan(start.0, start.1, goal.0, goal.1)?;
        let start_time = Instant::now();
        for _ in 0..RUNS {
            plan = planner.plan(start.0, start.1, goal.0, goal.1)?;
        }
        let avg_plan_us = start_time.elapsed().as_secs_f64() * 1_000_000.0 / RUNS as f64;

        rows.push(SweepRow {
            planner: format!("CP-SIPP c={required_confidence:.1}"),
            arrival_t: plan.path.last().map(|wp| wp.t).unwrap_or_default(),
            path_length: path_length(&plan.path),
            off_center_steps: off_center_steps(&plan.path, start.1),
            min_confidence: Some(plan.min_confidence),
            violation_bound: Some(plan.trajectory_violation_bound),
            avg_plan_us,
        });
    }

    println!("| Planner | Arrival t | Path length | Off-center steps | Min confidence | Violation bound | Avg plan us |");
    println!("|---|---:|---:|---:|---:|---:|---:|");
    for row in rows {
        let min_confidence = row
            .min_confidence
            .map(|value| format!("{value:.2}"))
            .unwrap_or_else(|| "-".to_string());
        let violation_bound = row
            .violation_bound
            .map(|value| format!("{value:.2}"))
            .unwrap_or_else(|| "-".to_string());
        println!(
            "| {} | {} | {:.1} | {} | {} | {} | {:.2} |",
            row.planner,
            row.arrival_t,
            row.path_length,
            row.off_center_steps,
            min_confidence,
            violation_bound,
            row.avg_plan_us
        );
    }

    Ok(())
}
