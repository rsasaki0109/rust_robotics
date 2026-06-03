//! Headless vanilla MPPI example for a 2-D double integrator.
//!
//! This is the baseline controller to build on before TD-CD-MPPI-style terminal
//! value and constraint-discounted extensions.

use rust_robotics::control::{MppiConfig, MppiController2D, MppiState2D};
use rust_robotics::prelude::*;

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn main() -> RoboticsResult<()> {
    let goal = (2.0, 1.0);
    let config = MppiConfig {
        horizon: 18,
        samples: 160,
        dt: 0.1,
        seed: 23,
        ..MppiConfig::default()
    };
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);

    println!(
        "initial: x={:.2} y={:.2} distance={:.2}",
        state.x,
        state.y,
        distance_to_goal(state, goal)
    );
    for step in 0..8 {
        let plan = controller.plan(state, goal)?;
        state = state.step(plan.first_control, 0.1);
        println!(
            "step {step}: ax={:.2} ay={:.2} x={:.2} y={:.2} vx={:.2} vy={:.2} distance={:.2} best_cost={:.2}",
            plan.first_control.ax,
            plan.first_control.ay,
            state.x,
            state.y,
            state.vx,
            state.vy,
            distance_to_goal(state, goal),
            plan.best_rollout.cost
        );
    }

    Ok(())
}
