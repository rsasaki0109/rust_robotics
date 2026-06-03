//! Headless CP-SIPP example.
//!
//! Compares exact-obstacle SIPP against conformal SIPP on the same dynamic
//! corridor. CP-SIPP inflates predicted obstacle occupancy from calibration
//! scores, so the returned path avoids low-confidence cells near the prediction.

use rust_robotics::planning::conformal_sipp::{
    ConformalSippConfig, ConformalSippPlanner, PredictedObstaclePoint, PredictedObstacleTrajectory,
};
use rust_robotics::planning::sipp::{DynamicObstacle, Interval, SippConfig, SippPlanner};
use rust_robotics::prelude::*;

fn empty_map(width: i32, height: i32) -> Vec<Vec<bool>> {
    vec![vec![false; height as usize]; width as usize]
}

fn repeated_scores(time_horizon: u64, scores: &[f64]) -> Vec<Vec<f64>> {
    (0..=time_horizon).map(|_| scores.to_vec()).collect()
}

fn print_path(label: &str, path: &[rust_robotics::planning::sipp::TimedWaypoint]) {
    println!(
        "{label}: arrival_t={} waypoints={}",
        path.last().map(|wp| wp.t).unwrap_or_default(),
        path.len()
    );
    for waypoint in path {
        println!(
            "  t={:02} cell=({}, {})",
            waypoint.t, waypoint.x, waypoint.y
        );
    }
}

fn main() -> RoboticsResult<()> {
    let width = 7;
    let height = 3;
    let start = (0, 1);
    let goal = (6, 1);
    let time_horizon = 12;

    let nominal_sipp = SippPlanner::new(SippConfig {
        width,
        height,
        obstacle_map: empty_map(width, height),
        dynamic_obstacles: vec![DynamicObstacle {
            x: 3,
            y: 1,
            interval: Interval::new(3, 5),
        }],
        allow_diagonal: false,
    })?;

    let conformal_sipp = ConformalSippPlanner::new(ConformalSippConfig {
        width,
        height,
        obstacle_map: empty_map(width, height),
        predicted_obstacles: vec![PredictedObstacleTrajectory::new(vec![
            PredictedObstaclePoint::new(3, 3.0, 1.0),
            PredictedObstaclePoint::new(4, 3.0, 1.0),
        ])],
        calibration_errors_by_time: repeated_scores(time_horizon, &[0.10, 1.10]),
        time_horizon,
        required_confidence: 1.0,
        obstacle_radius: 0.0,
        allow_diagonal: false,
    })?;

    let nominal_path = nominal_sipp.plan(start.0, start.1, goal.0, goal.1)?;
    let conformal_plan = conformal_sipp.plan(start.0, start.1, goal.0, goal.1)?;

    print_path("SIPP", &nominal_path);
    print_path("CP-SIPP", &conformal_plan.path);
    println!(
        "CP-SIPP min_confidence={:.2} violation_bound={:.2}",
        conformal_plan.min_confidence, conformal_plan.trajectory_violation_bound
    );
    println!(
        "CP-SIPP safe intervals at (2,1): {:?}",
        conformal_sipp.get_safe_intervals(2, 1)
    );
    println!(
        "CP-SIPP safe intervals at (3,1): {:?}",
        conformal_sipp.get_safe_intervals(3, 1)
    );

    Ok(())
}
