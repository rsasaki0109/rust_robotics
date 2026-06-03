//! MPPI with a terminal value grid.
//!
//! This reproduces a small TD-style extension: the finite-horizon MPPI rollout
//! can read a sampled cost-to-go estimate at its terminal state.

use rust_robotics::control::{MppiConfig, MppiController2D, MppiState2D, MppiTerminalValueGrid2D};
use rust_robotics::prelude::*;

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn run_label(label: &str, mut controller: MppiController2D) -> RoboticsResult<()> {
    let goal = (2.4, 1.0);
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    println!(
        "{label}: initial distance={:.2}",
        distance_to_goal(state, goal)
    );
    for step in 0..6 {
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, 0.1);
        println!(
            "  step {step}: ax={:.2} ay={:.2} x={:.2} y={:.2} distance={:.2} best_cost={:.2} terminal_value_cost={:.2}",
            plan.first_control.ax,
            plan.first_control.ay,
            state.x,
            state.y,
            distance_to_goal(state, goal),
            plan.best_rollout.cost,
            plan.best_rollout.terminal_value_cost
        );
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
    let goal = (2.4, 1.0);
    let base = MppiConfig {
        horizon: 8,
        samples: 220,
        dt: 0.1,
        control_limit: 2.5,
        noise_sigma: 1.1,
        goal_weight: 0.5,
        terminal_weight: 0.8,
        seed: 71,
        ..MppiConfig::default()
    };
    let value_grid = MppiTerminalValueGrid2D::from_goal_distance(90, 70, -0.5, -1.5, 0.05, goal)?;
    let terminal_value = MppiConfig {
        terminal_value_grid: Some(value_grid),
        terminal_value_weight: 12.0,
        ..base.clone()
    };

    run_label("short-horizon", MppiController2D::new(base)?)?;
    run_label("terminal-value", MppiController2D::new(terminal_value)?)?;

    Ok(())
}
