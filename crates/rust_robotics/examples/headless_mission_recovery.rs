//! Headless mission regression that exercises waypoint recovery without ROS/Gazebo.

use rust_robotics::core::{ControlInput, RoboticsError};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::planning::dwa::{DWAConfig, DWAPlanner};
use rust_robotics::prelude::*;

const WAYPOINT_TOLERANCE: f64 = 0.18;
const STUCK_PROGRESS_DISTANCE: f64 = 0.04;
const STUCK_STEP_LIMIT: usize = 18;
const MAX_RECOVERY_ATTEMPTS: u32 = 2;
const SETTLE_STEPS: usize = 5;
const ROTATE_STEPS: usize = 14;
const BACKOFF_STEPS: usize = 9;
const ROTATE_SPEED: f64 = 0.6;
const BACKOFF_SPEED: f64 = -0.08;
const MAX_SIM_STEPS: usize = 360;

struct MissionProgress {
    current_idx: usize,
    last_progress_pose: Point2D,
    last_progress_step: usize,
    recovery_attempts_for_waypoint: u32,
    total_recoveries: u32,
}

impl MissionProgress {
    fn new(initial_pose: Point2D) -> Self {
        Self {
            current_idx: 0,
            last_progress_pose: initial_pose,
            last_progress_step: 0,
            recovery_attempts_for_waypoint: 0,
            total_recoveries: 0,
        }
    }
}

fn create_obstacles() -> Obstacles {
    Obstacles::from_points(vec![Point2D::new(5.0, 5.0), Point2D::new(6.0, -4.0)])
}

fn waypoint_mission() -> Vec<Point2D> {
    vec![Point2D::new(1.8, 0.0), Point2D::new(1.8, 0.8)]
}

fn create_dwa_config() -> DWAConfig {
    DWAConfig {
        max_speed: 0.6,
        min_speed: 0.0,
        max_yaw_rate: 45.0_f64.to_radians(),
        max_accel: 0.3,
        max_delta_yaw_rate: 60.0_f64.to_radians(),
        v_resolution: 0.02,
        yaw_rate_resolution: 1.0_f64.to_radians(),
        dt: 0.1,
        predict_time: 2.5,
        robot_radius: 0.25,
        goal_threshold: WAYPOINT_TOLERANCE,
        ..Default::default()
    }
}

fn deterministic_measurement(step: usize, state: &State2D) -> Point2D {
    let x_noise = ((step % 5) as f64 - 2.0) * 0.01;
    let y_noise = (((step * 2) % 5) as f64 - 2.0) * 0.01;
    Point2D::new(state.x + x_noise, state.y + y_noise)
}

fn integrate_motion(state: &mut State2D, omega: &mut f64, control: ControlInput, dt: f64) {
    state.yaw += control.omega * dt;
    state.x += control.v * state.yaw.cos() * dt;
    state.y += control.v * state.yaw.sin() * dt;
    state.v = control.v;
    *omega = control.omega;
}

fn apply_control_step(
    true_state: &mut State2D,
    true_omega: &mut f64,
    ekf: &mut EKFLocalizer,
    control: ControlInput,
    dt: f64,
    step: usize,
) -> RoboticsResult<State2D> {
    integrate_motion(true_state, true_omega, control, dt);
    let measurement = deterministic_measurement(step, true_state);
    ekf.estimate_state(measurement, control, dt)
}

fn perform_recovery_sequence(
    true_state: &mut State2D,
    true_omega: &mut f64,
    ekf: &mut EKFLocalizer,
    dt: f64,
    mut step: usize,
    attempt: u32,
) -> RoboticsResult<(usize, State2D)> {
    println!("recovery attempt {attempt}: settle");
    for _ in 0..SETTLE_STEPS {
        let _ = apply_control_step(true_state, true_omega, ekf, ControlInput::zero(), dt, step)?;
        step += 1;
    }

    println!("recovery attempt {attempt}: rotate");
    let rotate_direction = if attempt.is_multiple_of(2) { -1.0 } else { 1.0 };
    for _ in 0..ROTATE_STEPS {
        let _ = apply_control_step(
            true_state,
            true_omega,
            ekf,
            ControlInput::new(0.0, rotate_direction * ROTATE_SPEED),
            dt,
            step,
        )?;
        step += 1;
    }

    println!("recovery attempt {attempt}: backoff");
    let mut estimate = ekf.state_2d();
    for _ in 0..BACKOFF_STEPS {
        estimate = apply_control_step(
            true_state,
            true_omega,
            ekf,
            ControlInput::new(BACKOFF_SPEED, 0.0),
            dt,
            step,
        )?;
        step += 1;
    }

    Ok((step, estimate))
}

fn maybe_mark_progress(progress: &mut MissionProgress, estimated_pose: Point2D, step: usize) {
    if estimated_pose.distance(&progress.last_progress_pose) >= STUCK_PROGRESS_DISTANCE {
        progress.last_progress_pose = estimated_pose;
        progress.last_progress_step = step;
    }
}

fn main() -> RoboticsResult<()> {
    let waypoints = waypoint_mission();
    let obstacles = create_obstacles();
    let config = create_dwa_config();
    let dt = config.dt;

    let mut planner = DWAPlanner::try_new(config.clone())?;
    planner.set_obstacles_from_obstacles(&obstacles)?;

    let mut ekf = EKFLocalizer::with_initial_state_2d(State2D::origin(), EKFConfig::default())?;

    let mut true_state = State2D::origin();
    let mut true_omega = 0.0;
    let mut progress = MissionProgress::new(Point2D::origin());
    let mut step = 0usize;
    let mut recovery_observed = false;

    while step < MAX_SIM_STEPS && progress.current_idx < waypoints.len() {
        let goal = waypoints[progress.current_idx];
        planner.try_set_state_from_2d(&true_state, true_omega)?;
        planner.try_set_goal(goal)?;

        let planned = planner.try_plan_step()?;
        let mut applied = DWAPlanner::control_to_input(&planned);

        let inject_stall =
            progress.current_idx == 0 && progress.total_recoveries == 0 && (12..48).contains(&step);
        if inject_stall {
            applied = ControlInput::zero();
        }

        let estimate = apply_control_step(
            &mut true_state,
            &mut true_omega,
            &mut ekf,
            applied,
            dt,
            step,
        )?;
        let estimated_pose = estimate.position();
        let distance_to_goal = estimated_pose.distance(&goal);

        if distance_to_goal <= WAYPOINT_TOLERANCE {
            println!(
                "reached waypoint {}/{} at step {} -> ({:.2}, {:.2})",
                progress.current_idx + 1,
                waypoints.len(),
                step,
                goal.x,
                goal.y
            );
            progress.current_idx += 1;
            progress.recovery_attempts_for_waypoint = 0;
            progress.last_progress_pose = estimated_pose;
            progress.last_progress_step = step;
            step += 1;
            continue;
        }

        maybe_mark_progress(&mut progress, estimated_pose, step);
        if step.saturating_sub(progress.last_progress_step) >= STUCK_STEP_LIMIT {
            progress.recovery_attempts_for_waypoint += 1;
            progress.total_recoveries += 1;
            recovery_observed = true;

            if progress.recovery_attempts_for_waypoint > MAX_RECOVERY_ATTEMPTS {
                return Err(RoboticsError::ControlError(format!(
                    "mission failed at waypoint {} after {} recovery attempts",
                    progress.current_idx + 1,
                    progress.recovery_attempts_for_waypoint - 1
                )));
            }

            println!(
                "stuck near waypoint {}/{} at step {}; running recovery attempt {}",
                progress.current_idx + 1,
                waypoints.len(),
                step,
                progress.recovery_attempts_for_waypoint
            );

            let (next_step, estimate_after_recovery) = perform_recovery_sequence(
                &mut true_state,
                &mut true_omega,
                &mut ekf,
                dt,
                step + 1,
                progress.recovery_attempts_for_waypoint,
            )?;

            progress.last_progress_pose = estimate_after_recovery.position();
            progress.last_progress_step = next_step;
            println!(
                "reissued waypoint {}/{} after recovery attempt {}",
                progress.current_idx + 1,
                waypoints.len(),
                progress.recovery_attempts_for_waypoint
            );
            step = next_step;
            continue;
        }

        if step.is_multiple_of(40) {
            println!(
                "step={step:03} waypoint={}/{} true=({:.2}, {:.2}) est=({:.2}, {:.2}) dist={:.2}",
                progress.current_idx + 1,
                waypoints.len(),
                true_state.x,
                true_state.y,
                estimate.x,
                estimate.y,
                distance_to_goal
            );
        }

        step += 1;
    }

    if progress.current_idx != waypoints.len() {
        return Err(RoboticsError::PlanningError(format!(
            "mission did not complete in {} steps; stopped at waypoint {}",
            MAX_SIM_STEPS,
            progress.current_idx + 1
        )));
    }
    if !recovery_observed {
        return Err(RoboticsError::ControlError(
            "mission regression never exercised recovery".to_string(),
        ));
    }

    let final_estimate = ekf.state_2d();
    println!(
        "mission complete: recoveries={} true=({:.2}, {:.2}) estimate=({:.2}, {:.2})",
        progress.total_recoveries, true_state.x, true_state.y, final_estimate.x, final_estimate.y
    );

    Ok(())
}
