//! Reference-free racing MPPI example.
//!
//! This reproduces the core gate-progress objective from "Rethinking Reference
//! Trajectories in Agile Drone Racing" in a dependency-light 2-D setting.

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiGateRace2D, MppiRacingGate2D,
    MppiState2D, MppiWaypointTrack2D,
};
use rust_robotics::prelude::*;

const STEPS: usize = 44;

fn gate(
    center: (f64, f64),
    previous: (f64, f64),
    half_width: f64,
) -> RoboticsResult<MppiRacingGate2D> {
    MppiRacingGate2D::new(
        center.0,
        center.1,
        center.0 - previous.0,
        center.1 - previous.1,
        half_width,
    )
}

fn gate_centers() -> Vec<(f64, f64)> {
    vec![
        (0.38, 0.18),
        (0.86, -0.14),
        (1.35, 0.18),
        (1.86, -0.12),
        (2.40, 0.14),
    ]
}

fn track() -> RoboticsResult<MppiWaypointTrack2D> {
    let mut waypoints = vec![(0.0, 0.0)];
    waypoints.extend(gate_centers());
    MppiWaypointTrack2D::new(waypoints)
}

fn race() -> RoboticsResult<MppiGateRace2D> {
    let mut previous = (0.0, 0.0);
    let mut gates = Vec::new();
    for center in gate_centers() {
        gates.push(gate(center, previous, 0.28)?);
        previous = center;
    }
    let mut race = MppiGateRace2D::new(gates)?;
    race.progress_weight = 6.5;
    race.lateral_weight = 0.35;
    race.pass_bonus = 9.0;
    race.miss_penalty = 42.0;
    race.terminal_gate_weight = 1.6;
    Ok(race)
}

fn obstacles() -> Vec<MppiCircularObstacle2D> {
    vec![
        MppiCircularObstacle2D::new(0.66, 0.32, 0.11),
        MppiCircularObstacle2D::new(1.12, -0.34, 0.11),
        MppiCircularObstacle2D::new(1.62, 0.34, 0.11),
        MppiCircularObstacle2D::new(2.16, -0.32, 0.11),
    ]
}

fn base_config(obstacles: Vec<MppiCircularObstacle2D>, seed: u64) -> MppiConfig {
    MppiConfig {
        horizon: 22,
        samples: 520,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.06,
        max_lambda: 28.0,
        target_effective_sample_ratio: 0.22,
        adaptive_lambda_iterations: 24,
        noise_sigma: 1.55,
        control_limit: 4.0,
        velocity_weight: 0.02,
        control_weight: 0.018,
        obstacles,
        constraint_weight: 620.0,
        constraint_discount: 0.9,
        safety_margin: 0.10,
        seed,
        ..MppiConfig::default()
    }
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

fn run_waypoint(
    track: &MppiWaypointTrack2D,
    obstacles: &[MppiCircularObstacle2D],
) -> RoboticsResult<()> {
    let value_grid = track.terminal_value_grid(108, 76, -0.35, -1.05, 0.035, 5.4, 13.0)?;
    let config = MppiConfig {
        goal_weight: 0.25,
        terminal_weight: 0.65,
        terminal_value_grid: Some(value_grid),
        terminal_value_weight: 1.0,
        ..base_config(obstacles.to_vec(), 331)
    };
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_lateral = 0.0;
    let mut mean_ess = 0.0;

    println!("waypoint-reference:");
    for step in 0..STEPS {
        let projection_before = track.project(state.x, state.y)?;
        let goal = track.point_at_progress(projection_before.progress + 0.82)?;
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, dt);
        let projection = track.project(state.x, state.y)?;
        let clearance = min_clearance(state, obstacles);
        min_seen_clearance = min_seen_clearance.min(clearance);
        mean_lateral += projection.lateral_error;
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        println!(
            "  step {step:02}: x={:.2} y={:.2} progress={:.2} lateral={:.2} clearance={:.2} ess={:.2}",
            state.x,
            state.y,
            projection.progress,
            projection.lateral_error,
            clearance,
            plan.sampling_diagnostics.normalized_effective_sample_size
        );
    }
    let projection = track.project(state.x, state.y)?;
    println!(
        "waypoint-reference: progress={:.2}/{:.2} lateral={:.2} mean_lateral={:.2} min_clearance={:.2} mean_ess={:.2}",
        projection.progress,
        track.total_length(),
        projection.lateral_error,
        mean_lateral / STEPS as f64,
        min_seen_clearance,
        mean_ess / STEPS as f64
    );
    Ok(())
}

fn run_gate_progress(
    race: &MppiGateRace2D,
    obstacles: &[MppiCircularObstacle2D],
) -> RoboticsResult<()> {
    let config = MppiConfig {
        goal_weight: 0.0,
        terminal_weight: 0.0,
        velocity_weight: 0.0,
        gate_race: Some(race.clone()),
        ..base_config(obstacles.to_vec(), 337)
    };
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut states = vec![state];
    let mut min_seen_clearance = f64::INFINITY;
    let mut mean_ess = 0.0;
    let mut executed_steps = 0;

    println!("gate-progress-reference-free:");
    for step in 0..STEPS {
        let plan = controller.plan(state, (0.0, 0.0))?;
        state = state.step(plan.first_control, dt);
        states.push(state);
        let report = race.score_trajectory(&states)?;
        let clearance = min_clearance(state, obstacles);
        min_seen_clearance = min_seen_clearance.min(clearance);
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        executed_steps += 1;
        println!(
            "  step {step:02}: x={:.2} y={:.2} gates={}/{} next_gate={} gate_dist={:.2} clearance={:.2} ess={:.2}",
            state.x,
            state.y,
            report.passed_gates,
            race.gate_count(),
            report.active_gate_index,
            report.final_gate_distance,
            clearance,
            plan.sampling_diagnostics.normalized_effective_sample_size
        );
        if report.passed_gates == race.gate_count() {
            break;
        }
    }
    let report = race.score_trajectory(&states)?;
    let mean_ess = if executed_steps > 0 {
        mean_ess / executed_steps as f64
    } else {
        0.0
    };
    println!(
        "gate-progress-reference-free: gates={}/{} active_gate={} gate_distance={:.2} lateral={:.2} min_clearance={:.2} mean_ess={:.2} race_cost={:.2}",
        report.passed_gates,
        race.gate_count(),
        report.active_gate_index,
        report.final_gate_distance,
        report.final_lateral_error,
        min_seen_clearance,
        mean_ess,
        report.cost
    );
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let track = track()?;
    let race = race()?;
    let obstacles = obstacles();

    run_waypoint(&track, &obstacles)?;
    run_gate_progress(&race, &obstacles)?;

    Ok(())
}
