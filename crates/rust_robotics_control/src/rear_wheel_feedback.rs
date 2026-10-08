//! Rear Wheel Feedback path tracking controller
//!
//! A path tracking controller based on rear wheel feedback steering control
//! that uses heading error and lateral error for computing the steering angle.
//!
//! Ref:
//!     - PythonRobotics: <https://github.com/AtsushiSakai/PythonRobotics>
//!     - B. Paden, M. Cap, S. Z. Yong, D. Yershov and E. Frazzoli,
//!       "A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles"

use crate::spline_course::calc_spline_course;
use rust_robotics_core::normalize_angle;
use rust_robotics_core::{ControlInput, Path2D, PathTracker, Point2D, State2D};

/// Vehicle state for Rear Wheel Feedback Controller
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

    /// Update vehicle state using bicycle model kinematics
    pub fn update(&mut self, a: f64, delta: f64, dt: f64) {
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * delta.tan() * dt;
        self.v += a * dt;
    }

    /// Get rear axle position (same as state position for rear wheel reference)
    pub fn rear_axle(&self) -> (f64, f64) {
        (self.x, self.y)
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

/// Configuration for Rear Wheel Feedback Controller
#[derive(Debug, Clone)]
pub struct RearWheelFeedbackConfig {
    /// Heading error gain (KTH)
    pub kth: f64,
    /// Lateral error gain (KE)
    pub ke: f64,
    /// Vehicle wheelbase \[m\]
    pub wheelbase: f64,
    /// Speed proportional gain
    pub kp: f64,
    /// Goal distance threshold \[m\]
    pub goal_threshold: f64,
    /// Maximum steering angle \[rad\]
    pub max_steer: f64,
}

impl Default for RearWheelFeedbackConfig {
    fn default() -> Self {
        Self {
            kth: 1.0,
            ke: 0.5,
            wheelbase: 2.9,
            kp: 1.0,
            goal_threshold: 0.5,
            max_steer: 45.0_f64.to_radians(),
        }
    }
}

/// Rear Wheel Feedback path tracking controller
pub struct RearWheelFeedbackController {
    config: RearWheelFeedbackConfig,
    path: Path2D,
    path_yaw: Vec<f64>,
    path_curvature: Vec<f64>,
    last_target_idx: usize,
}

impl RearWheelFeedbackController {
    /// Create a new Rear Wheel Feedback controller
    pub fn new(config: RearWheelFeedbackConfig) -> Self {
        RearWheelFeedbackController {
            config,
            path: Path2D::new(),
            path_yaw: Vec::new(),
            path_curvature: Vec::new(),
            last_target_idx: 0,
        }
    }

    /// Create with simplified parameters (legacy interface)
    pub fn with_params(kth: f64, ke: f64, wheelbase: f64) -> Self {
        let config = RearWheelFeedbackConfig {
            kth,
            ke,
            wheelbase,
            ..Default::default()
        };
        Self::new(config)
    }

    /// Set the reference path
    pub fn set_path(&mut self, path: Path2D) {
        self.path_yaw = self.compute_path_yaw(&path);
        self.path_curvature = self.compute_path_curvature(&path);
        self.path = path;
        self.last_target_idx = 0;
    }

    /// Set the reference path with pre-computed yaw angles and curvatures
    pub fn set_path_with_info(&mut self, path: Path2D, yaw: Vec<f64>, curvature: Vec<f64>) {
        self.path = path;
        self.path_yaw = yaw;
        self.path_curvature = curvature;
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

    /// Compute curvature from path points
    fn compute_path_curvature(&self, path: &Path2D) -> Vec<f64> {
        let n = path.len();
        if n < 3 {
            return vec![0.0; n];
        }

        let mut curvature = Vec::with_capacity(n);

        // First point
        curvature.push(0.0);

        // Middle points - compute curvature using three consecutive points
        for i in 1..n - 1 {
            let p0 = &path.points[i - 1];
            let p1 = &path.points[i];
            let p2 = &path.points[i + 1];

            let dx1 = p1.x - p0.x;
            let dy1 = p1.y - p0.y;
            let dx2 = p2.x - p1.x;
            let dy2 = p2.y - p1.y;

            let ds1 = (dx1 * dx1 + dy1 * dy1).sqrt();
            let ds2 = (dx2 * dx2 + dy2 * dy2).sqrt();

            if ds1 > 1e-6 && ds2 > 1e-6 {
                let yaw1 = dy1.atan2(dx1);
                let yaw2 = dy2.atan2(dx2);
                let dyaw = normalize_angle(yaw2 - yaw1);
                let ds = (ds1 + ds2) / 2.0;
                curvature.push(dyaw / ds);
            } else {
                curvature.push(0.0);
            }
        }

        // Last point
        curvature.push(*curvature.last().unwrap_or(&0.0));

        curvature
    }

    /// Find target index and compute lateral error
    fn calc_target_index(&self, state: &VehicleState) -> (usize, f64) {
        let (rx, ry) = state.rear_axle();
        let query = Point2D::new(rx, ry);
        let min_idx = self
            .path
            .nearest_point_index_from(query, self.last_target_idx)
            .unwrap_or(0);

        // Calculate lateral error (signed distance from path)
        let target_point = &self.path.points[min_idx];
        let dx = rx - target_point.x;
        let dy = ry - target_point.y;

        // Lateral error with sign (positive = left of path, negative = right)
        let path_yaw = if min_idx < self.path_yaw.len() {
            self.path_yaw[min_idx]
        } else {
            0.0
        };

        // Cross product to determine sign
        let error = -path_yaw.sin() * dx + path_yaw.cos() * dy;

        (min_idx, error)
    }

    /// Compute steering angle using rear wheel feedback control law
    ///
    /// Control law:
    /// omega = v * k * cos(th_e) / (1.0 - k * e) - KTH * |v| * th_e - KE * v * sin(th_e) * e / th_e
    /// delta = atan2(L * omega / v, 1.0)
    pub fn compute_steering(&mut self, state: &VehicleState) -> f64 {
        if self.path.is_empty() {
            return 0.0;
        }

        let (mut target_idx, e) = self.calc_target_index(state);

        // Don't go backwards on path
        if self.last_target_idx >= target_idx {
            target_idx = self.last_target_idx;
        }
        self.last_target_idx = target_idx;

        // Get path yaw and curvature at target point
        let yaw_ref = if target_idx < self.path_yaw.len() {
            self.path_yaw[target_idx]
        } else {
            *self.path_yaw.last().unwrap_or(&0.0)
        };

        let k = if target_idx < self.path_curvature.len() {
            self.path_curvature[target_idx]
        } else {
            *self.path_curvature.last().unwrap_or(&0.0)
        };

        // Heading error
        let th_e = normalize_angle(state.yaw - yaw_ref);

        // Compute angular velocity using rear wheel feedback control law
        let v = state.v;
        let v_abs = v.abs().max(0.1); // Avoid division by zero

        // sin(th_e) / th_e tends to 1 as th_e -> 0; evaluating it naively there
        // would drop the lateral-error term exactly when the heading is aligned.
        let sinc = if th_e.abs() < 1e-6 {
            1.0
        } else {
            th_e.sin() / th_e
        };

        // Rear wheel feedback control formula
        let denom = 1.0 - k * e;
        let denom_safe = if denom.abs() < 0.01 {
            0.01_f64.copysign(denom)
        } else {
            denom
        };

        let omega = v * k * th_e.cos() / denom_safe
            - self.config.kth * v_abs * th_e
            - self.config.ke * v * sinc * e;

        // Compute steering angle
        let delta = if v.abs() > 0.01 {
            (self.config.wheelbase * omega / v).atan()
        } else {
            0.0
        };

        // Clamp to maximum steering angle
        delta.clamp(-self.config.max_steer, self.config.max_steer)
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

    /// Legacy planning interface - simulate path tracking
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
        let (cx, cy, cyaw, ck, _) = calc_spline_course(&ax, &ay, ds);

        // Set path
        let path = Path2D::from_points(
            cx.iter()
                .zip(cy.iter())
                .map(|(&x, &y)| Point2D::new(x, y))
                .collect(),
        );
        self.set_path_with_info(path, cyaw, ck);

        // Initialize state
        let mut state = VehicleState::new(
            waypoints[0].0,
            waypoints[0].1,
            self.path_yaw.first().copied().unwrap_or(0.0),
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

impl PathTracker for RearWheelFeedbackController {
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
    use std::f64::consts::PI;

    #[test]
    fn test_rear_wheel_feedback_creation() {
        let config = RearWheelFeedbackConfig::default();
        let controller = RearWheelFeedbackController::new(config);
        assert!(controller.path.is_empty());
    }

    #[test]
    fn test_rear_wheel_feedback_config_defaults() {
        let config = RearWheelFeedbackConfig::default();
        assert!((config.kth - 1.0).abs() < 1e-10);
        assert!((config.ke - 0.5).abs() < 1e-10);
        assert!((config.wheelbase - 2.9).abs() < 1e-10);
    }

    #[test]
    fn test_rear_wheel_feedback_set_path() {
        let mut controller = RearWheelFeedbackController::with_params(1.0, 0.5, 2.9);
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ]);
        controller.set_path(path);
        assert_eq!(controller.get_path().len(), 3);
        assert_eq!(controller.path_yaw.len(), 3);
        assert_eq!(controller.path_curvature.len(), 3);
    }

    #[test]
    fn test_rear_wheel_feedback_normalize_angle() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 0.01);
        assert!((normalize_angle(-3.0 * PI) + PI).abs() < 0.01);
        assert!((normalize_angle(0.5)).abs() - 0.5 < 0.01);
    }

    #[test]
    fn test_rear_wheel_feedback_steering_straight_path() {
        let mut controller = RearWheelFeedbackController::with_params(1.0, 0.5, 2.9);
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(20.0, 0.0),
            Point2D::new(30.0, 0.0),
        ]);
        controller.set_path(path);

        // Vehicle on path, heading along path
        let state = VehicleState::new(5.0, 0.0, 0.0, 5.0, 2.9);
        let steering = controller.compute_steering(&state);

        // Should have near-zero steering for vehicle on path
        assert!(steering.abs() < 0.1);
    }

    #[test]
    fn test_rear_wheel_feedback_steering_lateral_error() {
        let mut controller = RearWheelFeedbackController::with_params(1.0, 0.5, 2.9);
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(20.0, 0.0),
            Point2D::new(30.0, 0.0),
        ]);
        controller.set_path(path);

        let state = VehicleState::new(5.0, 2.0, 0.1, 5.0, 2.9);
        let steering = controller.compute_steering(&state);

        assert!(
            steering < 0.0,
            "Steering should be negative to correct errors, got: {}",
            steering
        );
    }

    #[test]
    fn test_rear_wheel_feedback_planning() {
        let mut controller = RearWheelFeedbackController::with_params(1.0, 0.5, 2.9);
        let waypoints = vec![(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)];

        let result = controller.planning(waypoints, 5.0, 0.5);
        assert!(result.is_some());
        assert!(!result.unwrap().is_empty());
    }

    #[test]
    fn test_rear_wheel_feedback_curved_path() {
        let mut controller = RearWheelFeedbackController::with_params(1.0, 0.5, 2.9);
        let waypoints = vec![(0.0, 0.0), (10.0, 0.0), (20.0, 5.0), (30.0, 10.0)];

        let result = controller.planning(waypoints, 3.0, 0.5);
        assert!(result.is_some());

        let trajectory = result.unwrap();
        assert!(!trajectory.is_empty());

        // Check that trajectory ends near goal
        let last = trajectory.last().unwrap();
        let dx = last.0 - 30.0;
        let dy = last.1 - 10.0;
        let dist = (dx * dx + dy * dy).sqrt();
        assert!(dist < 2.0, "Final distance to goal: {}", dist);
    }

    #[test]
    fn test_vehicle_state_update() {
        let mut state = VehicleState::new(0.0, 0.0, 0.0, 1.0, 2.9);
        state.update(0.0, 0.0, 1.0);

        // Should move forward 1m with no steering
        assert!((state.x - 1.0).abs() < 0.01);
        assert!(state.y.abs() < 0.01);
        assert!(state.yaw.abs() < 0.01);
    }

    #[test]
    fn test_path_tracker_trait() {
        let mut controller = RearWheelFeedbackController::new(RearWheelFeedbackConfig::default());
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0),
            Point2D::new(20.0, 0.0),
        ]);

        let state = State2D::new(5.0, 0.0, 0.0, 5.0);
        let control = controller.compute_control(&state, &path);

        // Control input should be reasonable
        assert!(control.v.is_finite());
        assert!(control.omega.is_finite());
    }
}
