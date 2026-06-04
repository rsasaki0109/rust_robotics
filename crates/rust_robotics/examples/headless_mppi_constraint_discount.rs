//! Constraint-discounted MPPI example.
//!
//! Compares vanilla MPPI against the same controller with circular obstacle
//! penalties discounted over the rollout horizon.

use rust_robotics::control::{MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D};
use rust_robotics::prelude::*;

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn min_clearance(states: &[MppiState2D], obstacle: MppiCircularObstacle2D) -> f64 {
    states
        .iter()
        .map(|state| {
            let dx = state.x - obstacle.x;
            let dy = state.y - obstacle.y;
            (dx * dx + dy * dy).sqrt() - obstacle.radius
        })
        .fold(f64::INFINITY, f64::min)
}

fn run_label(
    label: &str,
    mut controller: MppiController2D,
    obstacle: MppiCircularObstacle2D,
) -> RoboticsResult<()> {
    let goal = (2.4, 0.0);
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    println!(
        "{label}: initial distance={:.2}",
        distance_to_goal(state, goal)
    );
    for step in 0..6 {
        let plan = controller.plan(state, goal)?;
        let clearance = min_clearance(&plan.best_rollout.states, obstacle);
        println!(
            "  step {step}: ax={:.2} ay={:.2} best_cost={:.2} constraint_cost={:.2} max_violation={:.2} rollout_min_clearance={:.2}",
            plan.first_control.ax,
            plan.first_control.ay,
            plan.best_rollout.cost,
            plan.best_rollout.constraint_cost,
            plan.best_rollout.max_constraint_violation,
            clearance
        );
        state = state.step(plan.first_control, 0.1);
    }
    println!(
        "{label}: final x={:.2} y={:.2} distance={:.2}",
        state.x,
        state.y,
        distance_to_goal(state, goal)
    );
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let obstacle = MppiCircularObstacle2D::new(1.0, 0.0, 0.35);
    let base = MppiConfig {
        horizon: 22,
        samples: 220,
        dt: 0.1,
        control_limit: 2.5,
        noise_sigma: 1.0,
        seed: 41,
        ..MppiConfig::default()
    };
    let constrained = MppiConfig {
        obstacles: vec![obstacle],
        constraint_weight: 500.0,
        constraint_discount: 0.9,
        safety_margin: 0.15,
        ..base.clone()
    };

    run_label("vanilla", MppiController2D::new(base)?, obstacle)?;
    run_label(
        "constraint-discounted",
        MppiController2D::new(constrained)?,
        obstacle,
    )?;

    Ok(())
}
