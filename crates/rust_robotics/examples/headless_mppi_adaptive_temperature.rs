//! Adaptive MPPI temperature example.
//!
//! Compares a fixed low lambda against an adaptive lambda selected to keep the
//! path-integral weights from collapsing to a single rollout.

use rust_robotics::control::{MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D};
use rust_robotics::prelude::*;

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn run_label(label: &str, config: MppiConfig) -> RoboticsResult<()> {
    let goal = (2.3, 0.4);
    let dt = config.dt;
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut ess_sum = 0.0;

    println!(
        "{label}: initial distance={:.2}",
        distance_to_goal(state, goal)
    );
    for step in 0..7 {
        let plan = controller.plan(state, goal)?;
        let diagnostics = plan.sampling_diagnostics;
        ess_sum += diagnostics.normalized_effective_sample_size;
        state = state.step(plan.first_control, dt);
        println!(
            "  step {step}: lambda={:.2} ess_ratio={:.2} entropy={:.2} ax={:.2} ay={:.2} distance={:.2} best_cost={:.2}",
            diagnostics.lambda,
            diagnostics.normalized_effective_sample_size,
            diagnostics.normalized_weight_entropy,
            plan.first_control.ax,
            plan.first_control.ay,
            distance_to_goal(state, goal),
            plan.best_rollout.cost
        );
    }
    println!(
        "{label}: final distance={:.2} mean_ess_ratio={:.2}",
        distance_to_goal(state, goal),
        ess_sum / 7.0
    );
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let obstacle = MppiCircularObstacle2D::new(1.0, 0.05, 0.35);
    let base = MppiConfig {
        horizon: 16,
        samples: 220,
        dt: 0.1,
        lambda: 0.08,
        noise_sigma: 1.15,
        control_limit: 2.6,
        goal_weight: 0.8,
        terminal_weight: 1.2,
        obstacles: vec![obstacle],
        constraint_weight: 420.0,
        constraint_discount: 0.9,
        safety_margin: 0.12,
        seed: 181,
        ..MppiConfig::default()
    };
    let adaptive = MppiConfig {
        adaptive_lambda: true,
        min_lambda: 0.08,
        max_lambda: 30.0,
        target_effective_sample_ratio: 0.28,
        adaptive_lambda_iterations: 24,
        ..base.clone()
    };

    run_label("fixed-low-lambda", base)?;
    run_label("adaptive-lambda", adaptive)?;

    Ok(())
}
