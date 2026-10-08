//! LQR Steer Control path tracking algorithm
//!
//! A path tracking controller using Linear Quadratic Regulator (LQR)
//! for steering control combined with PID speed control.
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109)

use crate::spline_course::calc_spline_course;
use alloc::vec::Vec;
use core::f64::consts::PI;
use nalgebra::{Matrix1, Matrix1x4, Matrix4, Vector4};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// f64 math via libm on no_std targets; on std hosts the inherent methods win
use num_traits::Float;
use rust_robotics_core::normalize_angle;
use rust_robotics_core::{ControlInput, Path2D, PathTracker, Point2D, State2D};

/// Vehicle state for LQR controller
#[derive(Debug, Clone, Copy)]
pub struct LQRVehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub wheelbase: f64,
    pub max_steer: f64,
}

impl LQRVehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64, max_steer: f64) -> Self {
        LQRVehicleState {
            x,
            y,
            yaw,
            v,
            wheelbase,
            max_steer,
        }
    }

    pub fn update(&mut self, a: f64, mut delta: f64, dt: f64) {
        delta = delta.clamp(-self.max_steer, self.max_steer);
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * delta.tan() * dt;
        self.v += a * dt;
    }

    pub fn to_state2d(&self) -> State2D {
        State2D::new(self.x, self.y, self.yaw, self.v)
    }
}

impl From<State2D> for LQRVehicleState {
    fn from(s: State2D) -> Self {
        LQRVehicleState::new(s.x, s.y, s.yaw, s.v, 0.5, 45.0_f64.to_radians())
    }
}

/// Configuration for LQR Steer Controller
#[derive(Debug, Clone)]
pub struct LQRSteerConfig {
    /// Vehicle wheelbase
    pub wheelbase: f64,
    /// Maximum steering angle \[rad\]
    pub max_steer: f64,
    /// Speed proportional gain
    pub kp: f64,
    /// State cost matrix Q (4x4 diagonal)
    pub q_diag: [f64; 4],
    /// Control cost R
    pub r: f64,
    /// Time step
    pub dt: f64,
    /// Goal distance threshold
    pub goal_threshold: f64,
}

impl Default for LQRSteerConfig {
    fn default() -> Self {
        Self {
            wheelbase: 0.5,
            max_steer: 45.0_f64.to_radians(),
            kp: 1.0,
            q_diag: [1.0, 1.0, 1.0, 1.0],
            r: 1.0,
            dt: 0.1,
            goal_threshold: 0.3,
        }
    }
}

/// LQR Steer path tracking controller
pub struct LQRSteerController {
    config: LQRSteerConfig,
    path: Path2D,
    path_yaw: Vec<f64>,
    path_curvature: Vec<f64>,
    speed_profile: Vec<f64>,
    prev_error: f64,
    prev_theta_error: f64,
    /// Nearest path index found last step; the search continues from it.
    nearest_index: Option<usize>,
}

impl LQRSteerController {
    /// Create a new LQR Steer controller
    pub fn new(config: LQRSteerConfig) -> Self {
        LQRSteerController {
            config,
            path: Path2D::new(),
            path_yaw: Vec::new(),
            path_curvature: Vec::new(),
            speed_profile: Vec::new(),
            prev_error: 0.0,
            prev_theta_error: 0.0,
            nearest_index: None,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(LQRSteerConfig::default())
    }

    /// Set the reference path
    pub fn set_path(&mut self, path: Path2D) {
        let (yaw, curvature) = self.compute_path_derivatives(&path);
        self.path_yaw = yaw;
        self.path_curvature = curvature;
        self.speed_profile = self.calc_speed_profile(&path, 2.78); // default ~10 km/h
        self.path = path;
        self.prev_error = 0.0;
        self.prev_theta_error = 0.0;
        self.nearest_index = None;
    }

    /// Set the reference path with speed
    pub fn set_path_with_speed(&mut self, path: Path2D, target_speed: f64) {
        let (yaw, curvature) = self.compute_path_derivatives(&path);
        self.path_yaw = yaw;
        self.path_curvature = curvature;
        self.speed_profile = self.calc_speed_profile(&path, target_speed);
        self.path = path;
        self.prev_error = 0.0;
        self.prev_theta_error = 0.0;
        self.nearest_index = None;
    }

    /// Get the current reference path
    pub fn get_path(&self) -> &Path2D {
        &self.path
    }

    /// Compute yaw and curvature from path points
    fn compute_path_derivatives(&self, path: &Path2D) -> (Vec<f64>, Vec<f64>) {
        let n = path.len();
        let yaw = path.yaw_profile();
        let mut curvature = Vec::with_capacity(n);

        for i in 0..n {
            // Simple curvature approximation
            if i > 0 && i < n - 1 {
                let dyaw = normalize_angle(yaw[i] - yaw[i - 1]);
                let dx = path.points[i].x - path.points[i - 1].x;
                let dy = path.points[i].y - path.points[i - 1].y;
                let ds = (dx * dx + dy * dy).sqrt();
                curvature.push(if ds > 0.001 { dyaw / ds } else { 0.0 });
            } else {
                curvature.push(0.0);
            }
        }

        (yaw, curvature)
    }

    /// Calculate speed profile based on path curvature
    fn calc_speed_profile(&self, path: &Path2D, target_speed: f64) -> Vec<f64> {
        let n = path.len();
        let mut profile = Vec::with_capacity(n);
        let mut direction = 1.0;

        for i in 0..n {
            if i < n - 1 && i < self.path_yaw.len() - 1 {
                let dyaw = (self.path_yaw[i + 1] - self.path_yaw[i]).abs();
                if (PI / 4.0..PI / 2.0).contains(&dyaw) {
                    direction *= -1.0;
                    profile.push(0.0);
                } else {
                    profile.push(direction * target_speed);
                }
            } else {
                profile.push(0.0);
            }
        }

        profile
    }

    /// Find target index and cross-track error
    ///
    /// Follows the path forward from the previous nearest point (from the
    /// start on the first call), so loops and crossings are not skipped.
    fn calc_target_index(&mut self, state: &LQRVehicleState) -> (usize, f64) {
        if self.path.is_empty() || self.path_yaw.is_empty() {
            return (0, 0.0);
        }
        let query = Point2D::new(state.x, state.y);
        let from = self.nearest_index.unwrap_or(0);
        let min_idx = self
            .path
            .nearest_point_index_forward(query, from)
            .unwrap_or(from);
        self.nearest_index = Some(min_idx);
        let target = &self.path.points[min_idx];
        let min_dist = ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt();

        // Calculate signed cross-track error
        let diff_x = target.x - state.x;
        let diff_y = target.y - state.y;
        let arcang = self.path_yaw[min_idx] - diff_y.atan2(diff_x);
        let angle = normalize_angle(arcang);

        let error = if angle < 0.0 { -min_dist } else { min_dist };
        (min_idx, error)
    }

    /// Solve Discrete Algebraic Riccati Equation
    fn solve_dare(
        a: Matrix4<f64>,
        b: Vector4<f64>,
        q: Matrix4<f64>,
        r: Matrix1<f64>,
    ) -> Matrix4<f64> {
        let mut x = q;
        let max_iter = 150;
        let eps = 0.01;

        for _ in 0..max_iter {
            let bt_x_b = b.transpose() * x * b;
            let inv = (r + bt_x_b).try_inverse().unwrap_or(Matrix1::identity());
            let xn =
                a.transpose() * x * a - a.transpose() * x * b * inv * b.transpose() * x * a + q;

            if (xn - x).abs().max() < eps {
                break;
            }
            x = xn;
        }
        x
    }

    /// Compute LQR gain
    fn dlqr(a: Matrix4<f64>, b: Vector4<f64>, q: Matrix4<f64>, r: Matrix1<f64>) -> Matrix1x4<f64> {
        let x = Self::solve_dare(a, b, q, r);
        let bt_x_b = b.transpose() * x * b;
        let inv = (bt_x_b + r).try_inverse().unwrap_or(Matrix1::identity());
        inv * (b.transpose() * x * a)
    }

    /// Compute LQR steering control
    pub fn compute_steering(&mut self, state: &LQRVehicleState) -> f64 {
        if self.path.is_empty() {
            return 0.0;
        }

        let (ind, e) = self.calc_target_index(state);
        let k = if ind < self.path_curvature.len() {
            self.path_curvature[ind]
        } else {
            0.0
        };

        let th_e = normalize_angle(state.yaw - self.path_yaw[ind]);
        let dt = self.config.dt;

        // Build state-space matrices
        let mut a = Matrix4::zeros();
        a[(0, 0)] = 1.0;
        a[(0, 1)] = dt;
        a[(1, 2)] = state.v;
        a[(2, 2)] = 1.0;
        a[(2, 3)] = dt;

        let mut b = Vector4::zeros();
        b[3] = state.v / state.wheelbase;

        // Build Q and R matrices
        let q = Matrix4::from_diagonal(&Vector4::new(
            self.config.q_diag[0],
            self.config.q_diag[1],
            self.config.q_diag[2],
            self.config.q_diag[3],
        ));
        let r = Matrix1::new(self.config.r);

        let gain = Self::dlqr(a, b, q, r);

        // State error vector
        let x_err = Vector4::new(
            e,
            if dt > 0.0 {
                (e - self.prev_error) / dt
            } else {
                0.0
            },
            th_e,
            if dt > 0.0 {
                (th_e - self.prev_theta_error) / dt
            } else {
                0.0
            },
        );

        // Feed-forward + feedback
        let ff = (state.wheelbase * k).atan2(1.0);
        let fb = normalize_angle((-gain * x_err)[0]);
        let delta = ff + fb;

        self.prev_error = e;
        self.prev_theta_error = th_e;

        delta.clamp(-state.max_steer, state.max_steer)
    }

    /// Proportional speed control
    pub fn compute_acceleration(&self, target_speed: f64, current_speed: f64) -> f64 {
        self.config.kp * (target_speed - current_speed)
    }

    /// Get target speed for current index
    pub fn get_target_speed(&self, index: usize) -> f64 {
        if index < self.speed_profile.len() {
            self.speed_profile[index].abs()
        } else {
            0.0
        }
    }

    /// Check if goal is reached
    pub fn is_goal_reached_vehicle(&self, state: &LQRVehicleState) -> bool {
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
        let (cx, cy, cyaw, ck, _) = calc_spline_course(&ax, &ay, ds);
        // Waypoints closer together than `ds` give an empty course.
        if cx.is_empty() {
            return None;
        }

        // Set path
        let path = Path2D::from_points(
            cx.iter()
                .zip(cy.iter())
                .map(|(&x, &y)| Point2D::new(x, y))
                .collect(),
        );
        self.path_yaw = cyaw;
        self.path_curvature = ck;
        self.speed_profile = self.calc_speed_profile(&path, target_speed);
        self.path = path;
        self.nearest_index = None;

        // Simulate tracking
        let mut state = LQRVehicleState::new(
            0.0,
            0.0,
            0.0,
            0.0,
            self.config.wheelbase,
            self.config.max_steer,
        );

        let mut trajectory = vec![(state.x, state.y)];
        let dt = self.config.dt;
        let t_max = 500.0;
        let mut time = 0.0;

        while time < t_max {
            let (target_idx, _) = self.calc_target_index(&state);
            let target_v = self.get_target_speed(target_idx);

            let ai = self.compute_acceleration(target_v, state.v);
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

impl PathTracker for LQRSteerController {
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput {
        // Adopt a new reference path (comparing contents, not just length)
        if self.path != *path {
            self.set_path(path.clone());
        }

        let vehicle_state = LQRVehicleState::new(
            current_state.x,
            current_state.y,
            current_state.yaw,
            current_state.v,
            self.config.wheelbase,
            self.config.max_steer,
        );

        if self.path.is_empty() {
            // Nothing to track: stop.
            return ControlInput::new(0.0, 0.0);
        }
        let delta = self.compute_steering(&vehicle_state);
        let (target_idx, _) = self.calc_target_index(&vehicle_state);
        let target_v = self.get_target_speed(target_idx);

        let v =
            current_state.v + self.compute_acceleration(target_v, current_state.v) * self.config.dt;
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
    fn the_nearest_point_search_stays_on_the_leg_being_driven() {
        // A hairpin: out along y = 0, back along y = 0.5.
        let mut points: Vec<Point2D> = (0..=100)
            .map(|i| Point2D::new(i as f64 * 0.1, 0.0))
            .collect();
        points.extend((0..=100).map(|i| Point2D::new(10.0 - i as f64 * 0.1, 0.5)));
        let mut controller = LQRSteerController::with_defaults();
        controller.set_path(Path2D::from_points(points));
        let at = |x: f64, y: f64| LQRVehicleState::new(x, y, 0.0, 1.0, 2.5, 0.6);

        let (first, _) = controller.calc_target_index(&at(2.0, 0.1));
        assert_eq!(first, 20);
        // Drifted toward the return leg: still the outbound leg, further on.
        let (next, _) = controller.calc_target_index(&at(3.0, 0.3));
        assert_eq!(next, 30);
    }

    #[test]
    fn an_empty_or_too_short_path_does_not_panic() {
        let mut controller = LQRSteerController::new(LQRSteerConfig::default());
        let command = controller.compute_control(&State2D::new(0.0, 0.0, 0.0, 1.0), &Path2D::new());
        assert_eq!((command.v, command.omega), (0.0, 0.0));
        let mut controller = LQRSteerController::new(LQRSteerConfig::default());
        assert!(controller
            .planning(vec![(0.0, 0.0), (0.3, 0.0)], 1.0, 0.5)
            .is_none());
    }

    #[test]
    fn test_lqr_creation() {
        let config = LQRSteerConfig::default();
        let controller = LQRSteerController::new(config);
        assert!(controller.path.is_empty());
    }

    #[test]
    fn test_lqr_set_path() {
        let mut controller = LQRSteerController::with_defaults();
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ]);
        controller.set_path(path);
        assert_eq!(controller.get_path().len(), 3);
    }

    #[test]
    fn test_lqr_normalize_angle() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 0.01);
        assert!((normalize_angle(-3.0 * PI) + PI).abs() < 0.01);
    }

    #[test]
    fn test_lqr_planning() {
        let mut controller = LQRSteerController::with_defaults();
        let waypoints = vec![(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)];

        let result = controller.planning(waypoints, 2.0, 0.5);
        assert!(result.is_some());
        assert!(!result.unwrap().is_empty());
    }

    #[test]
    fn test_lqr_solve_dare() {
        let a = Matrix4::identity();
        let b = Vector4::new(0.0, 0.0, 0.0, 1.0);
        let q = Matrix4::identity();
        let r = Matrix1::new(1.0);

        let x = LQRSteerController::solve_dare(a, b, q, r);
        // Should return a positive definite matrix
        assert!(x[(0, 0)] >= 0.0);
    }
}
