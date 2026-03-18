//! Headless integration example that combines DWA and EKF.

use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::planning::dwa::{DWAConfig, DWAPlanner, DWAState};
use rust_robotics::prelude::*;

fn create_obstacles() -> Obstacles {
    Obstacles::from_points(vec![Point2D::new(5.0, 5.0), Point2D::new(6.0, 6.0)])
}

fn main() -> RoboticsResult<()> {
    let obstacles = create_obstacles();
    let goal = Point2D::new(3.0, 0.0);

    let mut planner = DWAPlanner::try_new(DWAConfig::default())?;
    planner.try_set_state(DWAState::new(0.0, 0.0, 0.0, 0.0, 0.0))?;
    planner.try_set_goal(goal)?;
    planner.set_obstacles_from_obstacles(&obstacles)?;

    let mut ekf = EKFLocalizer::with_initial_state_2d(
        State2D::new(0.0, 0.0, 0.0, 0.0),
        EKFConfig::default(),
    )?;

    for step in 0..120 {
        if planner.is_goal_reached() {
            break;
        }

        let control = planner.try_step()?;
        let true_state = planner.state_2d();
        let measurement = Point2D::new(
            true_state.x + ((step % 5) as f64 - 2.0) * 0.02,
            true_state.y + (((step * 2) % 5) as f64 - 2.0) * 0.02,
        );
        let estimate = ekf.estimate_state(
            measurement,
            DWAPlanner::control_to_input(&control),
            planner.config().dt,
        )?;

        if step % 25 == 0 {
            println!(
                "step={step:03} true=({:.2}, {:.2}) est=({:.2}, {:.2})",
                true_state.x, true_state.y, estimate.x, estimate.y
            );
        }
    }

    let true_state = planner.state_2d();
    let estimate = ekf.state_2d();
    println!(
        "goal_reached={} true=({:.2}, {:.2}) estimate=({:.2}, {:.2}) distance_to_goal={:.2}",
        planner.is_goal_reached(),
        true_state.x,
        true_state.y,
        estimate.x,
        estimate.y,
        planner.distance_to_goal()
    );

    Ok(())
}
