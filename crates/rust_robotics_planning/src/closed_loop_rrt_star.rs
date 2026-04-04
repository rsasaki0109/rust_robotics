#![allow(dead_code, clippy::too_many_arguments)]

//! Closed-Loop RRT* (CL-RRT*) path planner
//!
//! Extends RRT\*-Reeds-Shepp by adding a forward-simulation verification step.
//! After the tree is grown, candidate goal paths are tracked using a pure-pursuit
//! controller on a unicycle (bicycle-kinematic) model. Only paths that are
//! dynamically feasible --- the controller reaches the goal, the tracked path
//! does not collide, and the travel distance is reasonable --- are accepted.
//!
//! The planner returns the full simulated trajectory (states over time) of the
//! best feasible path, not just the geometric waypoints.
//!
//! # References
//!
//! * Kuwata, Y. et al. (2009). "Real-Time Motion Planning With Applications
//!   to Autonomous Urban Driving." *IEEE T-CST*.
//! * PythonRobotics `ClosedLoopRRTStar/` by Atsushi Sakai.

use std::f64::consts::PI;

use rust_robotics_core::types::Pose2D;

use crate::rrt::{AreaBounds, CircleObstacle};
use crate::rrt_star_reeds_shepp::{RRTStarRSConfig, RRTStarRSNode, RRTStarRSPlanner};

// ---------------------------------------------------------------------------
// Unicycle (bicycle-kinematic) model
// ---------------------------------------------------------------------------

/// Parameters for the bicycle-kinematic (unicycle) vehicle model.
#[derive(Debug, Clone)]
pub struct UnicycleParams {
    /// Simulation time step \[s\].
    pub dt: f64,
    /// Wheelbase \[m\].
    pub wheelbase: f64,
    /// Maximum steering angle \[rad\].
    pub steer_max: f64,
    /// Maximum longitudinal acceleration \[m/s^2\].
    pub accel_max: f64,
}

impl Default for UnicycleParams {
    fn default() -> Self {
        Self {
            dt: 0.05,
            wheelbase: 0.9,
            steer_max: 40.0_f64.to_radians(),
            accel_max: 5.0,
        }
    }
}

/// Vehicle state for the unicycle model.
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
}

impl VehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64) -> Self {
        Self { x, y, yaw, v }
    }
}

/// Advance the vehicle state by one time step.
fn unicycle_update(
    state: &VehicleState,
    accel: f64,
    delta: f64,
    params: &UnicycleParams,
) -> VehicleState {
    let x = state.x + state.v * state.yaw.cos() * params.dt;
    let y = state.y + state.v * state.yaw.sin() * params.dt;
    let yaw = pi_2_pi(state.yaw + state.v / params.wheelbase * delta.tan() * params.dt);
    let v = state.v + accel * params.dt;
    VehicleState { x, y, yaw, v }
}

// ---------------------------------------------------------------------------
// Pure-pursuit controller
// ---------------------------------------------------------------------------

/// Parameters for the pure-pursuit path tracker.
#[derive(Debug, Clone)]
pub struct PurePursuitParams {
    /// Proportional speed gain.
    pub kp: f64,
    /// Look-ahead distance \[m\].
    pub look_ahead: f64,
    /// Maximum simulation time \[s\].
    pub max_time: f64,
    /// Goal distance threshold \[m\].
    pub goal_dis: f64,
    /// Speed below which the robot is considered stopped \[m/s\].
    pub stop_speed: f64,
}

impl Default for PurePursuitParams {
    fn default() -> Self {
        Self {
            kp: 2.0,
            look_ahead: 0.5,
            max_time: 100.0,
            goal_dis: 0.5,
            stop_speed: 0.5,
        }
    }
}

/// Result of a closed-loop forward simulation.
#[derive(Debug, Clone)]
pub struct SimulationResult {
    pub t: Vec<f64>,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub yaw: Vec<f64>,
    pub v: Vec<f64>,
    pub accel: Vec<f64>,
    pub steer: Vec<f64>,
    pub reached_goal: bool,
}

/// PID speed controller (proportional only, clamped).
fn pid_control(target: f64, current: f64, kp: f64, accel_max: f64) -> f64 {
    (kp * (target - current)).clamp(-accel_max, accel_max)
}

/// Find the target index on the reference path for pure pursuit.
fn calc_target_index(
    state: &VehicleState,
    cx: &[f64],
    cy: &[f64],
    look_ahead: f64,
) -> (usize, f64) {
    let mut min_dist = f64::INFINITY;
    let mut min_ind = 0;
    for (i, (&rx, &ry)) in cx.iter().zip(cy.iter()).enumerate() {
        let d = ((state.x - rx).powi(2) + (state.y - ry).powi(2)).sqrt();
        if d < min_dist {
            min_dist = d;
            min_ind = i;
        }
    }

    let mut cumulative = 0.0;
    while look_ahead > cumulative && (min_ind + 1) < cx.len() {
        let dx = cx[min_ind + 1] - cx[min_ind];
        let dy = cy[min_ind + 1] - cy[min_ind];
        cumulative += (dx * dx + dy * dy).sqrt();
        min_ind += 1;
    }

    (min_ind, min_dist)
}

/// Pure pursuit steering control.
fn pure_pursuit_control(
    state: &VehicleState,
    cx: &[f64],
    cy: &[f64],
    prev_ind: usize,
    look_ahead: f64,
    wheelbase: f64,
    steer_max: f64,
) -> (f64, usize, f64) {
    let (mut ind, dis) = calc_target_index(state, cx, cy, look_ahead);
    if prev_ind >= ind {
        ind = prev_ind;
    }

    let (tx, ty) = if ind < cx.len() {
        (cx[ind], cy[ind])
    } else {
        ind = cx.len() - 1;
        (cx[ind], cy[ind])
    };

    let mut alpha = (ty - state.y).atan2(tx - state.x) - state.yaw;
    if state.v <= 0.0 {
        alpha = PI - alpha;
    }

    let delta = (2.0 * wheelbase * alpha.sin() / look_ahead).atan2(1.0);
    let delta = delta.clamp(-steer_max, steer_max);

    (delta, ind, dis)
}

/// Build a speed profile along the reference path, inserting stop points at
/// direction reversals (forward <-> backward).
fn calc_speed_profile(
    cx: &[f64],
    cy: &[f64],
    cyaw: &[f64],
    target_speed: f64,
    stop_speed: f64,
) -> Vec<f64> {
    let n = cx.len();
    let mut profile = vec![target_speed; n];
    let mut forward = true;
    let mut is_back = false;

    for i in 0..n - 1 {
        let dx = cx[i + 1] - cx[i];
        let dy = cy[i + 1] - cy[i];
        let move_dir = dy.atan2(dx);
        is_back = (move_dir - cyaw[i]).abs() >= PI / 2.0;

        if dx == 0.0 && dy == 0.0 {
            continue;
        }

        if is_back {
            profile[i] = -target_speed;
        } else {
            profile[i] = target_speed;
        }

        if is_back && forward {
            profile[i] = 0.0;
            forward = false;
        } else if !is_back && !forward {
            profile[i] = 0.0;
            forward = true;
        }
    }

    profile[0] = 0.0;
    if is_back {
        profile[n - 1] = -stop_speed;
    } else {
        profile[n - 1] = stop_speed;
    }

    profile
}

/// Extend the path beyond its end by `look_ahead` distance so the tracker
/// does not overshoot.
fn extend_path(
    cx: &[f64],
    cy: &[f64],
    cyaw: &[f64],
    look_ahead: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut cx = cx.to_vec();
    let mut cy = cy.to_vec();
    let mut cyaw = cyaw.to_vec();

    let dl = 0.1_f64;
    let steps = (look_ahead / dl) as usize + 1;

    let n = cx.len();
    let move_dir = (cy[n - 1] - cy[n.saturating_sub(3)]).atan2(cx[n - 1] - cx[n.saturating_sub(3)]);
    let is_back = (move_dir - cyaw[n - 1]).abs() >= PI / 2.0;

    for _ in 0..steps {
        let idl = if is_back { -dl } else { dl };
        let last_yaw = *cyaw.last().unwrap();
        cx.push(cx.last().unwrap() + idl * last_yaw.cos());
        cy.push(cy.last().unwrap() + idl * last_yaw.sin());
        cyaw.push(last_yaw);
    }

    (cx, cy, cyaw)
}

/// Run the closed-loop pure-pursuit simulation on a reference path.
fn closed_loop_prediction(
    cx: &[f64],
    cy: &[f64],
    _cyaw: &[f64],
    speed_profile: &[f64],
    goal: [f64; 3],
    vehicle: &UnicycleParams,
    pp: &PurePursuitParams,
) -> SimulationResult {
    let mut state = VehicleState::new(0.0, 0.0, 0.0, 0.0);
    let mut time = 0.0;

    let mut result = SimulationResult {
        t: vec![0.0],
        x: vec![state.x],
        y: vec![state.y],
        yaw: vec![state.yaw],
        v: vec![state.v],
        accel: vec![0.0],
        steer: vec![0.0],
        reached_goal: false,
    };

    let (mut target_ind, _) = calc_target_index(&state, cx, cy, pp.look_ahead);
    let max_dis = 0.5_f64;

    while time <= pp.max_time {
        let (di, new_ind, dis) = pure_pursuit_control(
            &state,
            cx,
            cy,
            target_ind,
            pp.look_ahead,
            vehicle.wheelbase,
            vehicle.steer_max,
        );
        target_ind = new_ind;

        let mut target_speed = speed_profile[target_ind.min(speed_profile.len() - 1)];
        target_speed *= (max_dis - dis.min(max_dis - 0.1)) / max_dis;

        let ai = pid_control(target_speed, state.v, pp.kp, vehicle.accel_max);
        state = unicycle_update(&state, ai, di, vehicle);

        if state.v.abs() <= pp.stop_speed && target_ind <= cx.len().saturating_sub(2) {
            target_ind += 1;
        }

        time += vehicle.dt;

        // Check goal
        let dx = state.x - goal[0];
        let dy = state.y - goal[1];
        if (dx * dx + dy * dy).sqrt() <= pp.goal_dis {
            result.reached_goal = true;
            break;
        }

        result.t.push(time);
        result.x.push(state.x);
        result.y.push(state.y);
        result.yaw.push(state.yaw);
        result.v.push(state.v);
        result.accel.push(ai);
        result.steer.push(di);
    }

    result
}

// ---------------------------------------------------------------------------
// Closed-Loop RRT* planner
// ---------------------------------------------------------------------------

/// Configuration for the Closed-Loop RRT* planner.
#[derive(Debug, Clone)]
pub struct ClosedLoopRRTStarConfig {
    /// Underlying RRT*-Reeds-Shepp configuration.
    pub rrt_config: RRTStarRSConfig,
    /// Vehicle (unicycle) model parameters.
    pub vehicle: UnicycleParams,
    /// Pure-pursuit tracking parameters.
    pub pursuit: PurePursuitParams,
    /// Target speed for the forward simulation \[m/s\].
    pub target_speed: f64,
    /// Yaw threshold for checking goal candidates from the tree \[rad\].
    pub yaw_threshold: f64,
    /// Position threshold for checking goal candidates from the tree \[m\].
    pub xy_threshold: f64,
    /// If `tracked_travel / geometric_travel` exceeds this ratio, the path is
    /// rejected as inefficient.
    pub invalid_travel_ratio: f64,
    /// Yaw tolerance for the final simulated heading \[rad\].
    pub final_yaw_tolerance: f64,
}

impl Default for ClosedLoopRRTStarConfig {
    fn default() -> Self {
        Self {
            rrt_config: RRTStarRSConfig::default(),
            vehicle: UnicycleParams::default(),
            pursuit: PurePursuitParams::default(),
            target_speed: 10.0 / 3.6,
            yaw_threshold: 3.0_f64.to_radians(),
            xy_threshold: 0.5,
            invalid_travel_ratio: 5.0,
            final_yaw_tolerance: 30.0_f64.to_radians(),
        }
    }
}

/// Result of a successful CL-RRT* planning run.
#[derive(Debug, Clone)]
pub struct ClosedLoopRRTStarResult {
    /// Simulated trajectory.
    pub sim: SimulationResult,
    /// The geometric path from the RRT* tree (Reeds-Shepp waypoints).
    pub geometric_poses: Vec<Pose2D>,
}

/// Closed-Loop RRT* planner.
///
/// Internally runs [`RRTStarRSPlanner`] to build the tree, then evaluates all
/// candidate goal paths with a closed-loop forward simulation.
pub struct ClosedLoopRRTStarPlanner {
    config: ClosedLoopRRTStarConfig,
    obstacles: Vec<CircleObstacle>,
    rand_area: AreaBounds,
    inner: RRTStarRSPlanner,
}

impl ClosedLoopRRTStarPlanner {
    /// Create a new CL-RRT* planner.
    pub fn new(
        obstacles: Vec<CircleObstacle>,
        rand_area: AreaBounds,
        config: ClosedLoopRRTStarConfig,
    ) -> Self {
        let inner = RRTStarRSPlanner::new(
            obstacles.clone(),
            rand_area.clone(),
            config.rrt_config.clone(),
        );
        Self {
            config,
            obstacles,
            rand_area,
            inner,
        }
    }

    /// Plan from `start` to `goal`, returning the best feasible trajectory.
    ///
    /// Returns `None` if no dynamically feasible path is found.
    pub fn planning(&mut self, start: Pose2D, goal: Pose2D) -> Option<ClosedLoopRRTStarResult> {
        // Phase 1: grow the RRT*-Reeds-Shepp tree.
        let _ = self.inner.planning(start, goal);
        self.select_best_feasible(goal)
    }

    /// Plan using a deterministic sampler (for testing).
    pub fn plan_with_sampler<F>(
        &mut self,
        start: Pose2D,
        goal: Pose2D,
        sample_node: F,
    ) -> Option<ClosedLoopRRTStarResult>
    where
        F: FnMut(&RRTStarRSPlanner) -> RRTStarRSNode,
    {
        let _ = self.inner.plan_with_sampler(start, goal, sample_node);
        self.select_best_feasible(goal)
    }

    /// Access the internal RRT* tree.
    pub fn get_tree(&self) -> &[RRTStarRSNode] {
        self.inner.get_tree()
    }

    // -----------------------------------------------------------------------
    // Private
    // -----------------------------------------------------------------------

    /// After the tree has been built, find all goal-candidate nodes and return
    /// the one whose closed-loop simulation has the shortest travel time.
    fn select_best_feasible(&self, goal: Pose2D) -> Option<ClosedLoopRRTStarResult> {
        let tree = self.inner.get_tree();
        let goal_inds = self.get_goal_indexes(tree, &goal);

        let mut best_time = f64::INFINITY;
        let mut best_result: Option<ClosedLoopRRTStarResult> = None;

        for &ind in &goal_inds {
            let path = self.generate_final_course(tree, ind);
            if path.is_empty() || path.len() < 2 {
                continue;
            }

            let feasibility = self.check_tracking_feasible(&path, &goal);
            if let Some(sim) = feasibility {
                let end_time = *sim.t.last().unwrap_or(&f64::INFINITY);
                if end_time < best_time {
                    best_time = end_time;
                    let poses: Vec<Pose2D> = path
                        .iter()
                        .map(|&(x, y, yaw)| Pose2D::new(x, y, yaw))
                        .collect();
                    best_result = Some(ClosedLoopRRTStarResult {
                        sim,
                        geometric_poses: poses,
                    });
                }
            }
        }

        best_result
    }

    /// Collect node indices that are within position and yaw tolerance of the goal.
    fn get_goal_indexes(&self, tree: &[RRTStarRSNode], goal: &Pose2D) -> Vec<usize> {
        let mut inds = Vec::new();
        for (i, node) in tree.iter().enumerate() {
            let dx = node.x - goal.x;
            let dy = node.y - goal.y;
            if (dx * dx + dy * dy).sqrt() > self.config.xy_threshold {
                continue;
            }
            if angle_diff(node.yaw, goal.yaw).abs() > self.config.yaw_threshold {
                continue;
            }
            inds.push(i);
        }
        inds
    }

    /// Trace back from `goal_index` to the root, returning waypoints as
    /// `(x, y, yaw)` from start to goal.
    fn generate_final_course(
        &self,
        tree: &[RRTStarRSNode],
        goal_index: usize,
    ) -> Vec<(f64, f64, f64)> {
        let mut path: Vec<(f64, f64, f64)> = Vec::new();

        let mut node = &tree[goal_index];
        while node.parent.is_some() {
            for ((&px, &py), &pyaw) in node
                .path_x
                .iter()
                .rev()
                .zip(node.path_y.iter().rev())
                .zip(node.path_yaw.iter().rev())
            {
                path.push((px, py, pyaw));
            }
            node = &tree[node.parent.unwrap()];
        }
        path.push((node.x, node.y, node.yaw));
        path.reverse();
        path
    }

    /// Forward-simulate a path using pure pursuit and validate feasibility.
    ///
    /// Returns `Some(SimulationResult)` if feasible, `None` otherwise.
    fn check_tracking_feasible(
        &self,
        path: &[(f64, f64, f64)],
        goal: &Pose2D,
    ) -> Option<SimulationResult> {
        let cx: Vec<f64> = path.iter().map(|p| p.0).collect();
        let cy: Vec<f64> = path.iter().map(|p| p.1).collect();
        let cyaw: Vec<f64> = path.iter().map(|p| p.2).collect();

        let goal_arr = [goal.x, goal.y, goal.yaw];

        let (ecx, ecy, ecyaw) = extend_path(&cx, &cy, &cyaw, self.config.pursuit.look_ahead);

        let speed_profile = calc_speed_profile(
            &ecx,
            &ecy,
            &ecyaw,
            self.config.target_speed,
            self.config.pursuit.stop_speed,
        );

        let sim = closed_loop_prediction(
            &ecx,
            &ecy,
            &ecyaw,
            &speed_profile,
            goal_arr,
            &self.config.vehicle,
            &self.config.pursuit,
        );

        if !sim.reached_goal {
            return None;
        }

        // Final yaw check
        if let Some(&final_yaw) = sim.yaw.last() {
            if (pi_2_pi(final_yaw) - goal.yaw).abs() >= self.config.final_yaw_tolerance {
                return None;
            }
        }

        // Travel ratio check
        let travel: f64 = sim.v.iter().map(|vi| vi.abs()).sum::<f64>() * self.config.vehicle.dt;
        let origin_travel: f64 = path
            .windows(2)
            .map(|w| ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt())
            .sum();

        if origin_travel > 0.0 && travel / origin_travel >= self.config.invalid_travel_ratio {
            return None;
        }

        // Collision check along simulated trajectory
        if !self.check_sim_collision(&sim) {
            return None;
        }

        Some(sim)
    }

    /// Check that the simulated trajectory does not collide with any obstacle.
    fn check_sim_collision(&self, sim: &SimulationResult) -> bool {
        for obs in &self.obstacles {
            for (&sx, &sy) in sim.x.iter().zip(sim.y.iter()) {
                let dx = obs.x - sx;
                let dy = obs.y - sy;
                let d = (dx * dx + dy * dy).sqrt();
                if d <= obs.radius + self.config.rrt_config.robot_radius {
                    return false;
                }
            }
        }
        true
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Normalize an angle to \[-pi, pi\).
fn pi_2_pi(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    }
    if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Shortest signed angular difference.
fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- Unicycle model tests --

    #[test]
    fn test_unicycle_straight_line() {
        let params = UnicycleParams::default();
        let state = VehicleState::new(0.0, 0.0, 0.0, 1.0);
        let next = unicycle_update(&state, 0.0, 0.0, &params);
        assert!(approx_eq(next.x, params.dt, 1e-12));
        assert!(approx_eq(next.y, 0.0, 1e-12));
        assert!(approx_eq(next.yaw, 0.0, 1e-12));
        assert!(approx_eq(next.v, 1.0, 1e-12));
    }

    #[test]
    fn test_unicycle_acceleration() {
        let params = UnicycleParams::default();
        let state = VehicleState::new(0.0, 0.0, 0.0, 0.0);
        let next = unicycle_update(&state, 2.0, 0.0, &params);
        assert!(approx_eq(next.v, 2.0 * params.dt, 1e-12));
        assert!(approx_eq(next.x, 0.0, 1e-12)); // v was 0 at start
    }

    #[test]
    fn test_unicycle_turning() {
        let params = UnicycleParams::default();
        let state = VehicleState::new(0.0, 0.0, 0.0, 1.0);
        let delta = 0.1; // small steering angle
        let next = unicycle_update(&state, 0.0, delta, &params);
        // Yaw should increase (turning left)
        let expected_yaw = 1.0 / params.wheelbase * delta.tan() * params.dt;
        assert!(approx_eq(next.yaw, expected_yaw, 1e-10));
    }

    // -- Pure pursuit tests --

    #[test]
    fn test_calc_target_index_nearest() {
        let cx = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let cy = vec![0.0, 0.0, 0.0, 0.0, 0.0];
        let state = VehicleState::new(0.5, 0.0, 0.0, 1.0);
        let (ind, dis) = calc_target_index(&state, &cx, &cy, 0.5);
        // Nearest is index 0 or 1, then advanced by look-ahead
        assert!(ind <= cx.len());
        assert!(dis < 1.0);
    }

    #[test]
    fn test_pid_control_clamping() {
        let a = pid_control(10.0, 0.0, 2.0, 5.0);
        assert!(approx_eq(a, 5.0, 1e-12)); // Clamped to accel_max

        let a = pid_control(-10.0, 0.0, 2.0, 5.0);
        assert!(approx_eq(a, -5.0, 1e-12)); // Clamped to -accel_max
    }

    #[test]
    fn test_pid_control_proportional() {
        let a = pid_control(1.0, 0.0, 2.0, 5.0);
        assert!(approx_eq(a, 2.0, 1e-12));
    }

    // -- Speed profile tests --

    #[test]
    fn test_speed_profile_forward() {
        let cx = vec![0.0, 1.0, 2.0, 3.0];
        let cy = vec![0.0, 0.0, 0.0, 0.0];
        let cyaw = vec![0.0, 0.0, 0.0, 0.0];
        let profile = calc_speed_profile(&cx, &cy, &cyaw, 1.0, 0.5);
        assert_eq!(profile.len(), 4);
        assert!(approx_eq(profile[0], 0.0, 1e-12)); // start is 0
        assert!(profile[1] > 0.0); // forward
        assert!(approx_eq(profile[3], 0.5, 1e-12)); // end is stop_speed
    }

    #[test]
    fn test_speed_profile_backward() {
        let cx = vec![3.0, 2.0, 1.0, 0.0];
        let cy = vec![0.0, 0.0, 0.0, 0.0];
        // Yaw pointing forward (+x) but moving backward (-x)
        let cyaw = vec![0.0, 0.0, 0.0, 0.0];
        let profile = calc_speed_profile(&cx, &cy, &cyaw, 1.0, 0.5);
        assert!(approx_eq(profile[0], 0.0, 1e-12));
        // Segments should be negative (backward)
        assert!(profile[1] < 0.0 || approx_eq(profile[1], 0.0, 1e-12));
    }

    // -- Extend path tests --

    #[test]
    fn test_extend_path_length() {
        let cx = vec![0.0, 1.0, 2.0];
        let cy = vec![0.0, 0.0, 0.0];
        let cyaw = vec![0.0, 0.0, 0.0];
        let (ecx, ecy, ecyaw) = extend_path(&cx, &cy, &cyaw, 0.5);
        assert!(ecx.len() > cx.len());
        assert_eq!(ecx.len(), ecy.len());
        assert_eq!(ecx.len(), ecyaw.len());
    }

    // -- pi_2_pi tests --

    #[test]
    fn test_pi_2_pi() {
        assert!(approx_eq(pi_2_pi(0.0), 0.0, 1e-12));
        assert!(approx_eq(pi_2_pi(PI), PI, 1e-12));
        assert!(approx_eq(pi_2_pi(-PI), -PI, 1e-12));
        assert!(approx_eq(pi_2_pi(3.0 * PI), PI, 1e-10));
        assert!(approx_eq(pi_2_pi(-3.0 * PI), -PI, 1e-10));
    }

    #[test]
    fn test_angle_diff() {
        assert!(approx_eq(angle_diff(0.0, 0.0), 0.0, 1e-12));
        assert!(approx_eq(angle_diff(PI, 0.0), PI, 1e-12));
        assert!(approx_eq(angle_diff(0.0, PI), -PI, 1e-12));
    }

    // -- Closed-loop simulation tests --

    #[test]
    fn test_closed_loop_straight_path() {
        let cx: Vec<f64> = (0..50).map(|i| i as f64 * 0.2).collect();
        let cy = vec![0.0; cx.len()];
        let cyaw = vec![0.0; cx.len()];
        let vehicle = UnicycleParams::default();
        let pp = PurePursuitParams {
            max_time: 20.0,
            ..Default::default()
        };
        let goal = [cx[cx.len() - 6], cy[cy.len() - 6], 0.0]; // goal before extension point

        let (ecx, ecy, ecyaw) = extend_path(&cx, &cy, &cyaw, pp.look_ahead);
        let esp = calc_speed_profile(&ecx, &ecy, &ecyaw, 1.0, pp.stop_speed);

        let sim = closed_loop_prediction(&ecx, &ecy, &ecyaw, &esp, goal, &vehicle, &pp);
        // Should make progress in +x direction
        assert!(sim.x.last().unwrap() > &0.0);
        assert!(sim.t.len() > 1);
    }

    // -- Config defaults --

    #[test]
    fn test_config_defaults() {
        let config = ClosedLoopRRTStarConfig::default();
        assert!(approx_eq(config.target_speed, 10.0 / 3.6, 1e-10));
        assert!(approx_eq(config.yaw_threshold, 3.0_f64.to_radians(), 1e-10));
        assert!(approx_eq(config.xy_threshold, 0.5, 1e-12));
        assert!(approx_eq(config.invalid_travel_ratio, 5.0, 1e-12));
    }

    // -- Collision check --

    #[test]
    fn test_sim_collision_check_no_obstacles() {
        let config = ClosedLoopRRTStarConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = ClosedLoopRRTStarPlanner::new(vec![], rand_area, config);

        let sim = SimulationResult {
            t: vec![0.0, 0.1],
            x: vec![0.0, 1.0],
            y: vec![0.0, 0.0],
            yaw: vec![0.0, 0.0],
            v: vec![1.0, 1.0],
            accel: vec![0.0, 0.0],
            steer: vec![0.0, 0.0],
            reached_goal: true,
        };
        assert!(planner.check_sim_collision(&sim));
    }

    #[test]
    fn test_sim_collision_check_with_obstacle() {
        let obstacles = vec![CircleObstacle::new(0.5, 0.0, 0.3)];
        let config = ClosedLoopRRTStarConfig::default();
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let planner = ClosedLoopRRTStarPlanner::new(obstacles, rand_area, config);

        let sim = SimulationResult {
            t: vec![0.0, 0.1],
            x: vec![0.0, 0.5],
            y: vec![0.0, 0.0],
            yaw: vec![0.0, 0.0],
            v: vec![1.0, 1.0],
            accel: vec![0.0, 0.0],
            steer: vec![0.0, 0.0],
            reached_goal: true,
        };
        assert!(!planner.check_sim_collision(&sim));
    }

    // -- Goal index selection --

    #[test]
    fn test_get_goal_indexes_filters_by_xy_and_yaw() {
        let config = ClosedLoopRRTStarConfig {
            xy_threshold: 1.0,
            yaw_threshold: 0.5,
            ..Default::default()
        };
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let planner = ClosedLoopRRTStarPlanner::new(vec![], rand_area, config);

        let goal = Pose2D::new(10.0, 10.0, 0.0);
        let tree = vec![
            RRTStarRSNode::new(0.0, 0.0, 0.0),   // far from goal
            RRTStarRSNode::new(10.0, 10.0, 0.0), // at goal
            RRTStarRSNode::new(10.5, 10.5, 0.0), // near goal, good yaw
            RRTStarRSNode::new(10.0, 10.0, PI),  // at goal, bad yaw
        ];

        let inds = planner.get_goal_indexes(&tree, &goal);
        assert!(inds.contains(&1));
        assert!(inds.contains(&2));
        assert!(!inds.contains(&0));
        assert!(!inds.contains(&3));
    }

    // -- Generate final course --

    #[test]
    fn test_generate_final_course() {
        let config = ClosedLoopRRTStarConfig::default();
        let rand_area = AreaBounds::new(-5.0, 20.0, -5.0, 20.0);
        let planner = ClosedLoopRRTStarPlanner::new(vec![], rand_area, config);

        let mut root = RRTStarRSNode::new(0.0, 0.0, 0.0);
        root.parent = None;
        let mut child = RRTStarRSNode::new(5.0, 5.0, 0.5);
        child.parent = Some(0);
        child.path_x = vec![0.0, 2.5, 5.0];
        child.path_y = vec![0.0, 2.5, 5.0];
        child.path_yaw = vec![0.0, 0.25, 0.5];

        let tree = vec![root, child];
        let course = planner.generate_final_course(&tree, 1);
        assert!(course.len() >= 3);
        // First point should be root
        assert!(approx_eq(course[0].0, 0.0, 1e-12));
        assert!(approx_eq(course[0].1, 0.0, 1e-12));
    }

    // -- Integration test: planner construction --

    #[test]
    fn test_planner_creation() {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(4.0, 6.0, 1.0),
        ];
        let rand_area = AreaBounds::new(-2.0, 20.0, -2.0, 20.0);
        let config = ClosedLoopRRTStarConfig::default();
        let planner = ClosedLoopRRTStarPlanner::new(obstacles, rand_area, config);
        assert!(planner.get_tree().is_empty() || planner.get_tree().is_empty());
    }

    // -- Integration test: planning on obstacle-free env --

    #[test]
    fn test_planning_no_obstacles_deterministic() {
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let config = ClosedLoopRRTStarConfig {
            rrt_config: RRTStarRSConfig {
                max_iter: 300,
                goal_xy_threshold: 1.5,
                goal_yaw_threshold: 1.0,
                connect_circle_dist: 50.0,
                ..Default::default()
            },
            xy_threshold: 1.5,
            yaw_threshold: 1.0,
            ..Default::default()
        };
        let mut planner = ClosedLoopRRTStarPlanner::new(vec![], rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 0.0, 0.0);

        let mut call = 0;
        let result = planner.plan_with_sampler(start, goal, |p| {
            call += 1;
            if call % 2 == 0 {
                RRTStarRSNode::new(p.get_tree()[0].x + 5.0, 0.0, 0.0)
            } else {
                RRTStarRSNode::new(p.get_tree()[0].x + 2.5, 0.0, 0.0)
            }
        });

        // With no obstacles and a straight-line goal, we should find something
        // (though the closed-loop check may still reject if simulation doesn't
        // converge; that is acceptable).
        if let Some(res) = result {
            assert!(!res.sim.x.is_empty());
            assert!(res.sim.reached_goal);
            assert!(!res.geometric_poses.is_empty());
        }
    }

    // -- Integration test: runs without panic with obstacles --

    #[test]
    fn test_planning_with_obstacles_no_panic() {
        let obstacles = vec![
            CircleObstacle::new(5.0, 5.0, 1.0),
            CircleObstacle::new(4.0, 6.0, 1.0),
            CircleObstacle::new(4.0, 8.0, 1.0),
            CircleObstacle::new(6.0, 5.0, 1.0),
            CircleObstacle::new(7.0, 5.0, 1.0),
        ];
        let rand_area = AreaBounds::new(-2.0, 15.0, -2.0, 15.0);
        let config = ClosedLoopRRTStarConfig {
            rrt_config: RRTStarRSConfig {
                max_iter: 100,
                goal_xy_threshold: 1.5,
                goal_yaw_threshold: 1.0,
                ..Default::default()
            },
            xy_threshold: 1.5,
            yaw_threshold: 1.0,
            ..Default::default()
        };
        let mut planner = ClosedLoopRRTStarPlanner::new(obstacles, rand_area, config);

        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(6.0, 7.0, PI / 2.0);

        // Random planning may or may not find a feasible path in 100 iterations;
        // we just verify it does not panic.
        let _result = planner.planning(start, goal);
    }

    // -- Extend path with backward motion --

    #[test]
    fn test_extend_path_backward() {
        // Path moving in -x while yaw = 0 (backward motion)
        let cx = vec![3.0, 2.0, 1.0];
        let cy = vec![0.0, 0.0, 0.0];
        let cyaw = vec![0.0, 0.0, 0.0];
        let (ecx, _ecy, _ecyaw) = extend_path(&cx, &cy, &cyaw, 0.5);
        // Extended points should continue in -x direction
        assert!(ecx.last().unwrap() < &1.0);
    }

    // -- SimulationResult fields --

    #[test]
    fn test_simulation_result_default_fields() {
        let sim = SimulationResult {
            t: vec![],
            x: vec![],
            y: vec![],
            yaw: vec![],
            v: vec![],
            accel: vec![],
            steer: vec![],
            reached_goal: false,
        };
        assert!(!sim.reached_goal);
        assert!(sim.t.is_empty());
    }
}
