//! Track-progress MPPI example.
//!
//! A waypoint track is converted into a terminal value grid so a short-horizon
//! MPPI controller can follow course progress instead of only chasing one goal.

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D, MppiWaypointTrack2D,
};
use rust_robotics::prelude::*;

const STEPS: usize = 20;

fn track() -> RoboticsResult<MppiWaypointTrack2D> {
    MppiWaypointTrack2D::new(vec![
        (0.0, 0.0),
        (0.65, 0.55),
        (1.35, -0.55),
        (2.05, 0.55),
        (2.8, 0.0),
    ])
}

fn obstacles() -> Vec<MppiCircularObstacle2D> {
    vec![
        MppiCircularObstacle2D::new(0.78, 0.02, 0.24),
        MppiCircularObstacle2D::new(1.48, -0.03, 0.24),
        MppiCircularObstacle2D::new(2.18, 0.02, 0.24),
    ]
}

fn min_clearance(state: MppiState2D, obstacles: &[MppiCircularObstacle2D]) -> f64 {
    obstacles
        .iter()
        .map(|obstacle| {
            let dx = state.x - obstacle.x;
            let dy = state.y - obstacle.y;
            (dx * dx + dy * dy).sqrt() - obstacle.radius
        })
        .fold(f64::INFINITY, f64::min)
}

fn run_label(
    label: &str,
    config: MppiConfig,
    track: &MppiWaypointTrack2D,
    obstacles: &[MppiCircularObstacle2D],
    lookahead: Option<f64>,
) -> RoboticsResult<()> {
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_lateral = 0.0;

    println!("{label}: total_track_length={:.2}", track.total_length());
    for step in 0..STEPS {
        let projection_before = track.project(state.x, state.y)?;
        let goal = if let Some(lookahead) = lookahead {
            track.point_at_progress(projection_before.progress + lookahead)?
        } else {
            track.goal()
        };
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, dt);
        let projection = track.project(state.x, state.y)?;
        let clearance = min_clearance(state, obstacles);
        min_seen_clearance = min_seen_clearance.min(clearance);
        mean_lateral += projection.lateral_error;
        println!(
            "  step {step}: x={:.2} y={:.2} progress={:.2} lateral={:.2} clearance={:.2} lambda={:.2} ess={:.2}",
            state.x,
            state.y,
            projection.progress,
            projection.lateral_error,
            clearance,
            plan.sampling_diagnostics.lambda,
            plan.sampling_diagnostics.normalized_effective_sample_size
        );
    }
    let projection = track.project(state.x, state.y)?;
    println!(
        "{label}: final_progress={:.2}/{:.2} final_lateral={:.2} mean_lateral={:.2} min_clearance={:.2}",
        projection.progress,
        track.total_length(),
        projection.lateral_error,
        mean_lateral / STEPS as f64,
        min_seen_clearance
    );
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let track = track()?;
    let obstacles = obstacles();
    let base = MppiConfig {
        horizon: 15,
        samples: 340,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.08,
        max_lambda: 25.0,
        target_effective_sample_ratio: 0.24,
        adaptive_lambda_iterations: 22,
        noise_sigma: 1.15,
        control_limit: 2.7,
        goal_weight: 0.35,
        terminal_weight: 0.7,
        obstacles: obstacles.clone(),
        constraint_weight: 520.0,
        constraint_discount: 0.9,
        safety_margin: 0.12,
        seed: 223,
        ..MppiConfig::default()
    };
    let track_value_grid = track.terminal_value_grid(92, 72, -0.4, -1.4, 0.05, 7.0, 12.0)?;
    let track_progress = MppiConfig {
        terminal_value_grid: Some(track_value_grid),
        terminal_value_weight: 1.0,
        seed: 229,
        ..base.clone()
    };

    run_label("single-goal", base, &track, &obstacles, None)?;
    run_label(
        "track-progress-value",
        track_progress,
        &track,
        &obstacles,
        Some(0.95),
    )?;

    Ok(())
}
