//! Adap-RPF-lite person-following MPPI example.
//!
//! A target walks forward while a pedestrian occupies the fixed trailing point.
//! The adaptive sampler selects a target-centric following point with lower
//! occlusion/proximity cost, then MPPI tracks the predicted following trajectory.

use rust_robotics::control::{
    MppiConfig, MppiController2D, MppiMovingObstacle2D, MppiPersonFollowingConfig2D,
    MppiPersonFollowingSampler2D, MppiState2D,
};
use rust_robotics::prelude::*;

const STEPS: usize = 34;

#[derive(Debug, Clone, Copy)]
struct RunSummary {
    final_distance: f64,
    min_clearance: f64,
    mean_occlusion: f64,
    mean_proximity: f64,
    mean_ess: f64,
}

fn base_config(seed: u64) -> MppiConfig {
    MppiConfig {
        horizon: 16,
        samples: 360,
        dt: 0.1,
        adaptive_lambda: true,
        min_lambda: 0.06,
        max_lambda: 22.0,
        target_effective_sample_ratio: 0.24,
        adaptive_lambda_iterations: 22,
        noise_sigma: 1.15,
        control_limit: 2.7,
        goal_weight: 1.25,
        terminal_weight: 2.8,
        velocity_weight: 0.06,
        control_weight: 0.018,
        constraint_weight: 520.0,
        constraint_discount: 0.92,
        safety_margin: 0.18,
        seed,
        ..MppiConfig::default()
    }
}

fn target_at(step: usize) -> MppiMovingObstacle2D {
    let t = step as f64 * 0.1;
    MppiMovingObstacle2D::new(0.30 * t, 0.0, 0.30, 0.0, 0.30)
}

fn pedestrians_at(step: usize) -> Vec<MppiMovingObstacle2D> {
    let target = target_at(step);
    vec![
        MppiMovingObstacle2D::new(target.x - 0.92, 0.0, target.vx, target.vy, 0.34),
        MppiMovingObstacle2D::new(0.12 + 0.018 * step as f64, 0.72, 0.06, -0.10, 0.28),
    ]
}

fn fixed_goal_trajectory(
    config: MppiPersonFollowingConfig2D,
    target: MppiMovingObstacle2D,
) -> Vec<(f64, f64)> {
    (0..=config.horizon)
        .map(|step| {
            let predicted = target.predict(step as f64 * config.dt);
            (predicted.x - config.desired_distance, predicted.y)
        })
        .collect()
}

fn clearance_to_people(state: MppiState2D, people: &[MppiMovingObstacle2D]) -> f64 {
    people
        .iter()
        .map(|person| {
            let dx = state.x - person.x;
            let dy = state.y - person.y;
            (dx * dx + dy * dy).sqrt() - person.radius
        })
        .fold(f64::INFINITY, f64::min)
}

fn run_fixed_reference(follower_config: MppiPersonFollowingConfig2D) -> RoboticsResult<RunSummary> {
    let mut controller = MppiController2D::new(base_config(407))?;
    let mut state = MppiState2D::new(-1.65, -0.08, 0.0, 0.0);
    let mut min_clearance = f64::INFINITY;
    let mut mean_ess = 0.0;

    println!("fixed-back-point:");
    for step in 0..STEPS {
        let target = target_at(step);
        let pedestrians = pedestrians_at(step);
        controller.set_goal_trajectory(Some(fixed_goal_trajectory(follower_config, target)))?;
        controller.set_moving_obstacles(pedestrians.clone())?;
        let plan = controller.plan(
            state,
            (target.x - follower_config.desired_distance, target.y),
        )?;
        state = state.step(plan.first_control, follower_config.dt);
        min_clearance = min_clearance.min(clearance_to_people(state, &pedestrians));
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        let final_distance = ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt();
        println!(
            "  step {step:02}: x={:.2} y={:.2} target=({:.2},{:.2}) distance={:.2} clearance={:.2} ess={:.2}",
            state.x,
            state.y,
            target.x,
            target.y,
            final_distance,
            min_clearance,
            plan.sampling_diagnostics.normalized_effective_sample_size
        );
    }
    let target = target_at(STEPS);
    Ok(RunSummary {
        final_distance: ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt(),
        min_clearance,
        mean_occlusion: 0.0,
        mean_proximity: 0.0,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn run_adaptive() -> RoboticsResult<RunSummary> {
    let follower_config = MppiPersonFollowingConfig2D::default();
    let mut sampler = MppiPersonFollowingSampler2D::new(follower_config)?;
    let mut controller = MppiController2D::new(base_config(419))?;
    let mut state = MppiState2D::new(-1.65, -0.08, 0.0, 0.0);
    let mut min_clearance = f64::INFINITY;
    let mut mean_occlusion = 0.0;
    let mut mean_proximity = 0.0;
    let mut mean_ess = 0.0;

    println!("adaptive-rpf-lite:");
    for step in 0..STEPS {
        let target = target_at(step);
        let pedestrians = pedestrians_at(step);
        let candidate = sampler.select_following_point(state, target, &pedestrians)?;
        controller.set_goal_trajectory(Some(
            sampler.goal_trajectory_for_candidate(target, candidate),
        ))?;
        controller.set_moving_obstacles(pedestrians.clone())?;
        let plan = controller.plan(state, (candidate.x, candidate.y))?;
        state = state.step(plan.first_control, follower_config.dt);
        min_clearance = min_clearance.min(clearance_to_people(state, &pedestrians));
        mean_occlusion += candidate.visibility_cost;
        mean_proximity += candidate.proximity_cost;
        mean_ess += plan.sampling_diagnostics.normalized_effective_sample_size;
        let final_distance = ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt();
        println!(
            "  step {step:02}: x={:.2} y={:.2} cand=({:.2},{:.2}) distance={:.2} visibility={:.2} proximity={:.2} clearance={:.2} ess={:.2}",
            state.x,
            state.y,
            candidate.x,
            candidate.y,
            final_distance,
            candidate.visibility_cost,
            candidate.proximity_cost,
            min_clearance,
            plan.sampling_diagnostics.normalized_effective_sample_size
        );
    }
    let target = target_at(STEPS);
    Ok(RunSummary {
        final_distance: ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt(),
        min_clearance,
        mean_occlusion: mean_occlusion / STEPS as f64,
        mean_proximity: mean_proximity / STEPS as f64,
        mean_ess: mean_ess / STEPS as f64,
    })
}

fn main() -> RoboticsResult<()> {
    let follower_config = MppiPersonFollowingConfig2D::default();
    let fixed = run_fixed_reference(follower_config)?;
    let adaptive = run_adaptive()?;

    println!(
        "fixed-back-point: final_distance={:.2} min_clearance={:.2} mean_ess={:.2}",
        fixed.final_distance, fixed.min_clearance, fixed.mean_ess
    );
    println!(
        "adaptive-rpf-lite: final_distance={:.2} min_clearance={:.2} mean_occlusion={:.2} mean_proximity={:.2} mean_ess={:.2}",
        adaptive.final_distance,
        adaptive.min_clearance,
        adaptive.mean_occlusion,
        adaptive.mean_proximity,
        adaptive.mean_ess
    );
    Ok(())
}
