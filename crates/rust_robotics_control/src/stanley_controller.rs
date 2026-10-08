//! Stanley Controller path tracking algorithm
//!
//! A path tracking controller based on Stanley control law that uses
//! heading error and cross-track error for steering computation.
//!
//! Ref:
//!     - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
//!     - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

use crate::spline_course::calc_spline_course;
use alloc::vec::Vec;
use core::f64::consts::PI;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// f64 math via libm on no_std targets; on std hosts the inherent methods win
use num_traits::Float;
use rust_robotics_core::normalize_angle;
use rust_robotics_core::{ControlInput, Path2D, PathTracker, Point2D, State2D};

/// Vehicle state for Stanley Controller
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub wheelbase: f64,
}

impl VehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64) -> Self {
        VehicleState {
            x,
            y,
            yaw,
            v,
            wheelbase,
        }
    }

    pub fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * delta.tan() * dt;
        self.v += a * dt;
    }

    /// Get front axle position
    pub fn front_axle(&self) -> (f64, f64) {
        let fx = self.x + self.wheelbase * self.yaw.cos();
        let fy = self.y + self.wheelbase * self.yaw.sin();
        (fx, fy)
    }

    pub fn to_state2d(&self) -> State2D {
        State2D::new(self.x, self.y, self.yaw, self.v)
    }
}

impl From<State2D> for VehicleState {
    fn from(s: State2D) -> Self {
        VehicleState::new(s.x, s.y, s.yaw, s.v, 2.9) // default wheelbase
    }
}

/// Largest steering angle the controller commands \[rad\] (about 84°).
const MAX_STEER: f64 = 0.5 * PI - 0.1;

/// Configuration for Stanley Controller
#[derive(Debug, Clone)]
pub struct StanleyConfig {
    /// Cross-track error gain (k)
    pub k: f64,
    /// Vehicle wheelbase
    pub wheelbase: f64,
    /// Speed proportional gain
    pub kp: f64,
    /// Goal distance threshold
    pub goal_threshold: f64,
}

impl Default for StanleyConfig {
    fn default() -> Self {
        Self {
            k: 0.5,
            wheelbase: 2.9,
            kp: 1.0,
            goal_threshold: 3.0,
        }
    }
}

/// Stanley path tracking controller
pub struct StanleyController {
    config: StanleyConfig,
    path: Path2D,
    path_yaw: Vec<f64>,
    last_target_idx: usize,
}

impl StanleyController {
    /// Create a new Stanley controller
    pub fn new(config: StanleyConfig) -> Self {
        StanleyController {
            config,
            path: Path2D::new(),
            path_yaw: Vec::new(),
            last_target_idx: 0,
        }
    }

    /// Create with simplified parameters (legacy interface)
    pub fn with_params(k: f64, wheelbase: f64) -> Self {
        let config = StanleyConfig {
            k,
            wheelbase,
            ..Default::default()
        };
        Self::new(config)
    }

    /// Set the reference path
    pub fn set_path(&mut self, path: Path2D) {
        self.path_yaw = self.compute_path_yaw(&path);
        self.path = path;
        self.last_target_idx = 0;
    }

    /// Set the reference path with pre-computed yaw angles
    pub fn set_path_with_yaw(&mut self, path: Path2D, yaw: Vec<f64>) {
        self.path = path;
        self.path_yaw = yaw;
        self.last_target_idx = 0;
    }

    /// Get the current reference path
    pub fn get_path(&self) -> &Path2D {
        &self.path
    }

    /// Compute yaw angles from path points
    fn compute_path_yaw(&self, path: &Path2D) -> Vec<f64> {
        path.yaw_profile()
    }

    /// Find target index and cross-track error
    fn calc_target_index(&self, state: &VehicleState) -> (usize, f64) {
        let (fx, fy) = state.front_axle();
        let query = Point2D::new(fx, fy);
        let min_idx = self
            .path
            .nearest_point_index_forward(query, self.last_target_idx)
            .unwrap_or(0);

        // Calculate cross-track error
        let target_point = &self.path.points[min_idx];
        let diff_x = fx - target_point.x;
        let diff_y = fy - target_point.y;
        let error_front_axle =
            -(state.yaw + 0.5 * PI).cos() * diff_x - (state.yaw + 0.5 * PI).sin() * diff_y;

        (min_idx, error_front_axle)
    }

    /// Compute steering angle using Stanley control law
    pub fn compute_steering(&mut self, state: &VehicleState) -> f64 {
        if self.path.is_empty() {
            return 0.0;
        }

        let (mut target_idx, error_front_axle) = self.calc_target_index(state);

        if self.last_target_idx >= target_idx {
            target_idx = self.last_target_idx;
        }
        self.last_target_idx = target_idx;

        // Heading error
        let theta_e = normalize_angle(self.path_yaw[target_idx] - state.yaw);

        // Cross-track error correction
        let theta_d = (self.config.k * error_front_axle).atan2(state.v.max(0.1));

        // Total steering angle, kept short of ±90°: beyond it tan(δ) (and so
        // the turn rate) flips sign and the vehicle turns away from the path.
        (theta_e + theta_d).clamp(-MAX_STEER, MAX_STEER)
    }

    /// Proportional speed control
    pub fn compute_acceleration(&self, target_speed: f64, current_speed: f64) -> f64 {
        self.config.kp * (target_speed - current_speed)
    }

    /// Check if goal is reached
    pub fn is_goal_reached_vehicle(&self, state: &VehicleState) -> bool {
        if let Some(goal) = self.path.points.last() {
            let dx = state.x - goal.x;
            let dy = state.y - goal.y;
            (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
        } else {
            true
        }
    }

    /// Legacy planning interface
    pub fn planning(
        &mut self,
        waypoints: Vec<(f64, f64)>,
        target_speed: f64,
        ds: f64,
    ) -> Option<Vec<(f64, f64)>> {
        if waypoints.len() < 2 {
            return None;
        }

        // Generate spline path
        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        let (cx, cy, cyaw, _, _) = calc_spline_course(&ax, &ay, ds);

        // Set path
        let path = Path2D::from_points(
            cx.iter()
                .zip(cy.iter())
                .map(|(&x, &y)| Point2D::new(x, y))
                .collect(),
        );
        self.set_path_with_yaw(path, cyaw);

        // Initialize state
        let init_yaw = 20.0_f64.to_radians();
        let mut state = VehicleState::new(
            waypoints[0].0,
            waypoints[0].1 + 5.0,
            init_yaw,
            0.0,
            self.config.wheelbase,
        );

        let mut trajectory = vec![(state.x, state.y)];
        let dt = 0.1;
        let t_max = 100.0;
        let mut time = 0.0;

        while time < t_max {
            let ai = self.compute_acceleration(target_speed, state.v);
            let di = self.compute_steering(&state);
            state.update(ai, di, dt);
            time += dt;

            trajectory.push((state.x, state.y));

            if self.is_goal_reached_vehicle(&state) {
                break;
            }
        }

        Some(trajectory)
    }
}

impl PathTracker for StanleyController {
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput {
        // Adopt a new reference path (comparing contents, not just length)
        if self.path != *path {
            self.set_path(path.clone());
        }

        let vehicle_state = VehicleState::new(
            current_state.x,
            current_state.y,
            current_state.yaw,
            current_state.v,
            self.config.wheelbase,
        );

        let delta = self.compute_steering(&vehicle_state);

        // Compute speed control (assume constant target speed)
        let target_speed = 5.0; // m/s
        let v = current_state.v + self.compute_acceleration(target_speed, current_state.v) * 0.1;

        // Convert steering angle to angular velocity
        let omega = v * delta.tan() / self.config.wheelbase;

        ControlInput::new(v, omega)
    }

    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool {
        let dx = current_state.x - goal.x;
        let dy = current_state.y - goal.y;
        (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_large_heading_error_still_steers_back_toward_the_path() {
        // Path along +x; the vehicle points almost straight up or down.
        // Past ±90° of steering tan() flipped sign and it turned away.
        let path = Path2D::from_points((0..=40).map(|i| Point2D::new(i as f64, 0.0)).collect());
        for (yaw, expected_sign) in [(1.6, -1.0), (1.8, -1.0), (-1.6, 1.0), (-1.8, 1.0)] {
            let mut controller = StanleyController::new(StanleyConfig::default());
            let command = controller.compute_control(&State2D::new(5.0, 0.0, yaw, 2.0), &path);
            assert!(
                command.omega * expected_sign > 0.0,
                "yaw {yaw}: omega {}",
                command.omega
            );
        }
    }

    #[test]
    fn test_stanley_creation() {
        let config = StanleyConfig::default();
        let controller = StanleyController::new(config);
        assert!(controller.path.is_empty());
    }

    #[test]
    fn test_stanley_set_path() {
        let mut controller = StanleyController::with_params(0.5, 2.9);
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ]);
        controller.set_path(path);
        assert_eq!(controller.get_path().len(), 3);
        assert_eq!(controller.path_yaw.len(), 3);
    }

    #[test]
    fn test_stanley_normalize_angle() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 0.01);
        assert!((normalize_angle(-3.0 * PI) + PI).abs() < 0.01);
    }

    #[test]
    fn test_stanley_planning() {
        let mut controller = StanleyController::with_params(0.5, 2.9);
        let waypoints = vec![(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)];

        let result = controller.planning(waypoints, 5.0, 0.5);
        assert!(result.is_some());
        assert!(!result.unwrap().is_empty());
    }
}
