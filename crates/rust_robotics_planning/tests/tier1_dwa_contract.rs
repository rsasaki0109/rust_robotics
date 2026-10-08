//! Tier 1 contract for DWA, the workspace's only obstacle-aware local planner.
//! It has no role trait yet (see "Local planning: no trait yet" in
//! `docs/api_traits.md`), so this test pins its inherent API instead.

use rust_robotics_core::{Obstacles, Point2D};
use rust_robotics_planning::dwa::{DWAConfig, DWAPlanner, DWAState};

const GOAL: Point2D = Point2D { x: 10.0, y: 10.0 };

fn obstacles() -> Vec<Point2D> {
    [
        (4.0, 2.0),
        (5.0, 4.0),
        (5.0, 5.0),
        (5.0, 6.0),
        (5.0, 9.0),
        (8.0, 9.0),
        (7.0, 9.0),
        (8.0, 10.0),
        (9.0, 11.0),
        (12.0, 13.0),
        (12.0, 12.0),
        (15.0, 15.0),
        (13.0, 13.0),
    ]
    .into_iter()
    .map(|(x, y)| Point2D::new(x, y))
    .collect()
}

fn planner() -> DWAPlanner {
    let mut planner = DWAPlanner::try_new(DWAConfig::default()).expect("default config");
    planner
        .try_set_state(DWAState::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_8,
            0.0,
            0.0,
        ))
        .expect("finite state");
    planner.try_set_goal(GOAL).expect("finite goal");
    planner
        .set_obstacles_from_obstacles(&Obstacles::from_points(obstacles()))
        .expect("finite obstacles");
    planner
}

#[test]
fn planning_a_command_does_not_move_the_robot() {
    let mut planner = planner();
    let before = planner.state_2d();
    let command = planner.try_plan_input().expect("plan");
    let after = planner.state_2d();
    assert_eq!(
        (before.x, before.y, before.yaw),
        (after.x, after.y, after.yaw)
    );
    assert!(command.v.is_finite() && command.omega.is_finite());
    assert!(
        command.v >= 0.0,
        "DWA drives forward from rest: v = {}",
        command.v
    );
}

#[test]
fn stepping_reaches_the_goal_around_obstacles() {
    let mut planner = planner();
    let obstacles = obstacles();
    let mut clearance = f64::INFINITY;
    for _ in 0..1_000 {
        if planner.is_goal_reached() {
            break;
        }
        planner.try_step().expect("step");
        let state = planner.state_2d();
        let position = Point2D::new(state.x, state.y);
        for obstacle in &obstacles {
            clearance = clearance.min(position.distance(obstacle));
        }
    }
    assert!(
        planner.is_goal_reached(),
        "still {:.2} m from the goal",
        planner.distance_to_goal()
    );
    assert!(
        clearance > 0.3,
        "came within {clearance:.2} m of an obstacle"
    );
}

#[test]
fn invalid_inputs_are_rejected_without_panicking() {
    let mut planner = planner();
    assert!(planner.try_set_goal(Point2D::new(f64::NAN, 0.0)).is_err());
    assert!(planner
        .try_set_state(DWAState::new(0.0, f64::INFINITY, 0.0, 0.0, 0.0))
        .is_err());
    assert!(DWAPlanner::try_new(DWAConfig {
        max_speed: -1.0,
        ..DWAConfig::default()
    })
    .is_err());
}
