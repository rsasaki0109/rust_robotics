//! Replay-buffer terminal-value learning for MPPI.
//!
//! Recent best rollouts are reused to update the terminal value grid, giving a
//! lightweight replay version of TD-style MPPI value learning.

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D, MppiTerminalValueGrid2D,
    MppiTerminalValueReplayBuffer2D, MppiTerminalValueUpdateConfig2D, MppiTerminalValueUpdater2D,
};
use rust_robotics::prelude::*;

const EPISODE_STEPS: usize = 12;

fn distance_to_goal(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    (dx * dx + dy * dy).sqrt()
}

fn clearance(state: MppiState2D, obstacle: MppiCircularObstacle2D) -> f64 {
    let dx = state.x - obstacle.x;
    let dy = state.y - obstacle.y;
    (dx * dx + dy * dy).sqrt() - obstacle.radius
}

fn scaled_goal_grid(goal: (f64, f64)) -> RoboticsResult<MppiTerminalValueGrid2D> {
    let mut grid = MppiTerminalValueGrid2D::from_goal_distance(90, 70, -0.5, -1.75, 0.05, goal)?;
    for column in &mut grid.values {
        for value in column {
            *value *= 8.0;
        }
    }
    Ok(grid)
}

fn run_episode(
    episode: usize,
    grid: MppiTerminalValueGrid2D,
    buffer: &mut MppiTerminalValueReplayBuffer2D,
    updater: &MppiTerminalValueUpdater2D,
    obstacle: MppiCircularObstacle2D,
) -> RoboticsResult<MppiTerminalValueGrid2D> {
    let goal = (2.4, 0.0);
    let config = MppiConfig {
        horizon: 9,
        samples: 260,
        dt: 0.1,
        control_limit: 2.6,
        noise_sigma: 1.1,
        goal_weight: 0.45,
        terminal_weight: 0.8,
        obstacles: vec![obstacle],
        constraint_weight: 550.0,
        constraint_discount: 0.9,
        safety_margin: 0.15,
        terminal_value_weight: 1.0,
        terminal_value_grid: Some(grid),
        seed: 131 + episode as u64,
        ..MppiConfig::default()
    };
    let mut controller = MppiController2D::new(config)?;
    let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
    let mut min_clearance = f64::INFINITY;
    let mut mean_delta_sum = 0.0;
    let mut max_delta: f64 = 0.0;
    let mut replay_updates = 0;

    for _ in 0..EPISODE_STEPS {
        let plan = controller.plan(state, goal)?;
        buffer.push(plan.best_rollout.clone())?;
        let replay_report = buffer.update_grid(
            controller
                .terminal_value_grid_mut()
                .expect("terminal value grid is configured"),
            updater,
        )?;
        mean_delta_sum += replay_report.mean_abs_delta;
        max_delta = max_delta.max(replay_report.max_abs_delta);
        replay_updates += replay_report.updates;
        state = state.step(plan.first_control, 0.1);
        min_clearance = min_clearance.min(clearance(state, obstacle));
    }

    let learned_grid = controller
        .terminal_value_grid()
        .expect("terminal value grid is configured")
        .clone();
    println!(
        "episode {episode}: replay_size={} replay_updates={} final_distance={:.2} final_y={:.2} min_clearance={:.2} value_start={:.2} value_frontier={:.2} value_near_obstacle={:.2} mean_replay_delta={:.2} max_delta={:.2}",
        buffer.len(),
        replay_updates,
        distance_to_goal(state, goal),
        state.y,
        min_clearance,
        learned_grid.value_at(0.0, 0.0),
        learned_grid.value_at(0.25, 0.0),
        learned_grid.value_at(1.0, 0.0),
        mean_delta_sum / EPISODE_STEPS as f64,
        max_delta
    );

    Ok(learned_grid)
}

fn main() -> RoboticsResult<()> {
    let goal = (2.4, 0.0);
    let obstacle = MppiCircularObstacle2D::new(1.0, 0.0, 0.35);
    let updater = MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D {
        learning_rate: 0.12,
        discount: 0.97,
    })?;
    let mut buffer = MppiTerminalValueReplayBuffer2D::new(24)?;
    let mut grid = scaled_goal_grid(goal)?;

    println!(
        "initial_grid: value_start={:.2} value_frontier={:.2} value_near_obstacle={:.2}",
        grid.value_at(0.0, 0.0),
        grid.value_at(0.25, 0.0),
        grid.value_at(1.0, 0.0)
    );
    for episode in 0..5 {
        grid = run_episode(episode, grid, &mut buffer, &updater, obstacle)?;
    }

    Ok(())
}
