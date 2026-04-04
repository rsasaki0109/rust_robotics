//! LQR Speed and Steering Control path tracking algorithm
//!
//! Extends LQR steer control with speed control, tracking both lateral and
//! longitudinal errors. The state vector is 5-dimensional:
//!   x = \[e, dot_e, th_e, dot_th_e, delta_v\]
//! and the input vector is 2-dimensional:
//!   u = \[delta, accel\]
//!
//! Reference: PythonRobotics lqr_speed_steer_control
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109)

use nalgebra::{Matrix2, Matrix5, Matrix5x2, SMatrix, SVector, Vector2, Vector5};
use rust_robotics_core::{ControlInput, Path2D, PathTracker, Point2D, State2D};
use std::f64::consts::PI;

/// Vehicle state for LQR speed+steer controller
#[derive(Debug, Clone, Copy)]
pub struct LQRSpeedSteerVehicleState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub wheelbase: f64,
    pub max_steer: f64,
}

impl LQRSpeedSteerVehicleState {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64, max_steer: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            v,
            wheelbase,
            max_steer,
        }
    }

    /// Update vehicle state with bicycle model kinematics
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

impl From<State2D> for LQRSpeedSteerVehicleState {
    fn from(s: State2D) -> Self {
        Self::new(s.x, s.y, s.yaw, s.v, 0.5, 45.0_f64.to_radians())
    }
}

/// Configuration for LQR Speed and Steer Controller
#[derive(Debug, Clone)]
pub struct LQRSpeedSteerConfig {
    /// Vehicle wheelbase \[m\]
    pub wheelbase: f64,
    /// Maximum steering angle \[rad\]
    pub max_steer: f64,
    /// State cost matrix Q diagonal (5 elements: e, dot_e, th_e, dot_th_e, delta_v)
    pub q_diag: [f64; 5],
    /// Control cost matrix R diagonal (2 elements: delta, accel)
    pub r_diag: [f64; 2],
    /// Time step \[s\]
    pub dt: f64,
    /// Goal distance threshold \[m\]
    pub goal_threshold: f64,
}

impl Default for LQRSpeedSteerConfig {
    fn default() -> Self {
        Self {
            wheelbase: 0.5,
            max_steer: 45.0_f64.to_radians(),
            q_diag: [1.0, 1.0, 1.0, 1.0, 1.0],
            r_diag: [1.0, 1.0],
            dt: 0.1,
            goal_threshold: 0.3,
        }
    }
}

/// LQR Speed and Steer path tracking controller
///
/// Unlike the steer-only LQR controller, this controller computes both
/// steering angle and acceleration via a single 5-state LQR formulation,
/// removing the need for a separate PID speed controller.
pub struct LQRSpeedSteerController {
    config: LQRSpeedSteerConfig,
    path: Path2D,
    path_yaw: Vec<f64>,
    path_curvature: Vec<f64>,
    speed_profile: Vec<f64>,
    prev_error: f64,
    prev_theta_error: f64,
}

impl LQRSpeedSteerController {
    /// Create a new LQR Speed+Steer controller
    pub fn new(config: LQRSpeedSteerConfig) -> Self {
        Self {
            config,
            path: Path2D::new(),
            path_yaw: Vec::new(),
            path_curvature: Vec::new(),
            speed_profile: Vec::new(),
            prev_error: 0.0,
            prev_theta_error: 0.0,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(LQRSpeedSteerConfig::default())
    }

    /// Set the reference path with a default target speed (~10 km/h)
    pub fn set_path(&mut self, path: Path2D) {
        let (yaw, curvature) = self.compute_path_derivatives(&path);
        self.path_yaw = yaw;
        self.path_curvature = curvature;
        self.speed_profile = self.calc_speed_profile(&path, 10.0 / 3.6);
        self.path = path;
        self.prev_error = 0.0;
        self.prev_theta_error = 0.0;
    }

    /// Set the reference path with a specific target speed
    pub fn set_path_with_speed(&mut self, path: Path2D, target_speed: f64) {
        let (yaw, curvature) = self.compute_path_derivatives(&path);
        self.path_yaw = yaw;
        self.path_curvature = curvature;
        self.speed_profile = self.calc_speed_profile(&path, target_speed);
        self.path = path;
        self.prev_error = 0.0;
        self.prev_theta_error = 0.0;
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
            if i > 0 && i < n - 1 {
                let dyaw = Self::normalize_angle(yaw[i] - yaw[i - 1]);
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

    /// Calculate speed profile based on path yaw changes
    fn calc_speed_profile(&self, path: &Path2D, target_speed: f64) -> Vec<f64> {
        let n = path.len();
        let mut profile = vec![target_speed; n];
        let mut direction = 1.0;

        #[allow(clippy::needless_range_loop)]
        for i in 0..n.saturating_sub(1) {
            if i < self.path_yaw.len() - 1 {
                let dyaw = (self.path_yaw[i + 1] - self.path_yaw[i]).abs();
                let switch = (PI / 4.0..PI / 2.0).contains(&dyaw);

                if switch {
                    direction *= -1.0;
                }

                if direction != 1.0 {
                    profile[i] = -target_speed;
                } else {
                    profile[i] = target_speed;
                }

                if switch {
                    profile[i] = 0.0;
                }
            }
        }

        // Speed down near the end
        let slowdown_len = 40.min(n);
        for i in 0..slowdown_len {
            let idx = n - 1 - i;
            profile[idx] = target_speed / (50 - i) as f64;
            if profile[idx] <= 1.0 / 3.6 {
                profile[idx] = 1.0 / 3.6;
            }
        }

        profile
    }

    /// Normalize angle to \[-PI, PI\]
    fn normalize_angle(mut angle: f64) -> f64 {
        while angle > PI {
            angle -= 2.0 * PI;
        }
        while angle < -PI {
            angle += 2.0 * PI;
        }
        angle
    }

    /// Find nearest path index and signed cross-track error
    fn calc_nearest_index(&self, state: &LQRSpeedSteerVehicleState) -> (usize, f64) {
        let query = Point2D::new(state.x, state.y);
        let min_idx = self.path.nearest_point_index(query).unwrap_or(0);
        let target = &self.path.points[min_idx];
        let min_dist = ((state.x - target.x).powi(2) + (state.y - target.y).powi(2)).sqrt();

        let diff_x = target.x - state.x;
        let diff_y = target.y - state.y;
        let arcang = self.path_yaw[min_idx] - diff_y.atan2(diff_x);
        let angle = Self::normalize_angle(arcang);

        let error = if angle < 0.0 { -min_dist } else { min_dist };
        (min_idx, error)
    }

    /// Solve Discrete Algebraic Riccati Equation for 5x5 system with 2 inputs
    fn solve_dare(
        a: &Matrix5<f64>,
        b: &Matrix5x2<f64>,
        q: &Matrix5<f64>,
        r: &Matrix2<f64>,
    ) -> Matrix5<f64> {
        let mut x = *q;
        let max_iter = 150;
        let eps = 0.01;

        for _ in 0..max_iter {
            // x_next = A^T X A - A^T X B (R + B^T X B)^{-1} B^T X A + Q
            let at = a.transpose();
            let bt = b.transpose();
            let at_x = at * x;
            let at_x_a = at_x * a;
            let at_x_b = at_x * b;
            let bt_x_b: Matrix2<f64> = bt * x * b;
            let inv = (r + bt_x_b).try_inverse().unwrap_or(Matrix2::identity());
            let bt_x_a: SMatrix<f64, 2, 5> = bt * x * a;
            let x_next = at_x_a - at_x_b * inv * bt_x_a + q;

            let diff: f64 = (x_next - x).abs().max();
            if diff < eps {
                return x_next;
            }
            x = x_next;
        }
        x
    }

    /// Compute discrete LQR gain matrix K (2x5)
    fn dlqr(
        a: &Matrix5<f64>,
        b: &Matrix5x2<f64>,
        q: &Matrix5<f64>,
        r: &Matrix2<f64>,
    ) -> SMatrix<f64, 2, 5> {
        let x = Self::solve_dare(a, b, q, r);
        let bt = b.transpose();
        let bt_x_b: Matrix2<f64> = bt * x * b;
        let inv = (bt_x_b + r).try_inverse().unwrap_or(Matrix2::identity());
        inv * (bt * x * a)
    }

    /// Compute LQR speed and steering control
    ///
    /// Returns (steering_delta, target_index, lateral_error, heading_error, acceleration)
    pub fn compute_control_full(
        &mut self,
        state: &LQRSpeedSteerVehicleState,
    ) -> (f64, usize, f64, f64, f64) {
        if self.path.is_empty() {
            return (0.0, 0, 0.0, 0.0, 0.0);
        }

        let (ind, e) = self.calc_nearest_index(state);
        let tv = if ind < self.speed_profile.len() {
            self.speed_profile[ind]
        } else {
            0.0
        };
        let k = if ind < self.path_curvature.len() {
            self.path_curvature[ind]
        } else {
            0.0
        };

        let v = state.v;
        let th_e = Self::normalize_angle(state.yaw - self.path_yaw[ind]);
        let dt = self.config.dt;
        let l = self.config.wheelbase;

        // Build 5x5 state-space A matrix
        let mut a = Matrix5::zeros();
        a[(0, 0)] = 1.0;
        a[(0, 1)] = dt;
        a[(1, 2)] = v;
        a[(2, 2)] = 1.0;
        a[(2, 3)] = dt;
        a[(4, 4)] = 1.0;

        // Build 5x2 input B matrix
        let mut b = Matrix5x2::zeros();
        b[(3, 0)] = v / l;
        b[(4, 1)] = dt;

        // Build Q and R matrices
        let q = Matrix5::from_diagonal(&Vector5::new(
            self.config.q_diag[0],
            self.config.q_diag[1],
            self.config.q_diag[2],
            self.config.q_diag[3],
            self.config.q_diag[4],
        ));
        let r = Matrix2::from_diagonal(&Vector2::new(self.config.r_diag[0], self.config.r_diag[1]));

        let gain = Self::dlqr(&a, &b, &q, &r);

        // State error vector: [e, dot_e, th_e, dot_th_e, delta_v]
        let x_err = Vector5::new(
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
            v - tv,
        );

        // u* = -K x
        let ustar: SVector<f64, 2> = -gain * x_err;

        // Steering: feedforward + feedback
        let ff = (l * k).atan2(1.0);
        let fb = Self::normalize_angle(ustar[0]);
        let delta = ff + fb;

        // Acceleration from LQR
        let accel = ustar[1];

        self.prev_error = e;
        self.prev_theta_error = th_e;

        let delta_clamped = delta.clamp(-state.max_steer, state.max_steer);
        (delta_clamped, ind, e, th_e, accel)
    }

    /// Compute steering angle only (convenience method)
    pub fn compute_steering(&mut self, state: &LQRSpeedSteerVehicleState) -> f64 {
        self.compute_control_full(state).0
    }

    /// Compute acceleration only (convenience method)
    pub fn compute_acceleration(&mut self, state: &LQRSpeedSteerVehicleState) -> f64 {
        self.compute_control_full(state).4
    }

    /// Get target speed at a given index
    pub fn get_target_speed(&self, index: usize) -> f64 {
        if index < self.speed_profile.len() {
            self.speed_profile[index]
        } else {
            0.0
        }
    }

    /// Check if goal is reached
    pub fn is_goal_reached(&self, state: &LQRSpeedSteerVehicleState) -> bool {
        if let Some(goal) = self.path.points.last() {
            let dx = state.x - goal.x;
            let dy = state.y - goal.y;
            (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
        } else {
            true
        }
    }

    /// Run a full simulation using cubic spline interpolation of waypoints
    pub fn planning(
        &mut self,
        waypoints: Vec<(f64, f64)>,
        target_speed: f64,
        ds: f64,
    ) -> Option<Vec<(f64, f64)>> {
        if waypoints.len() < 2 {
            return None;
        }

        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        let (cx, cy, cyaw, ck, _) = calc_spline_course(&ax, &ay, ds);

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
        self.prev_error = 0.0;
        self.prev_theta_error = 0.0;

        let mut state = LQRSpeedSteerVehicleState::new(
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
        let stop_speed = 0.05;
        let mut time = 0.0;

        while time < t_max {
            let (delta, target_ind, _, _, accel) = self.compute_control_full(&state);

            state.update(accel, delta, dt);
            time += dt;

            if state.v.abs() <= stop_speed && target_ind + 1 < self.path.len() {
                // nudge forward when nearly stopped
            }

            trajectory.push((state.x, state.y));

            if self.is_goal_reached(&state) {
                break;
            }
        }

        Some(trajectory)
    }
}

impl PathTracker for LQRSpeedSteerController {
    fn compute_control(&mut self, current_state: &State2D, path: &Path2D) -> ControlInput {
        if self.path.len() != path.len() {
            self.set_path(path.clone());
        }

        let vehicle_state = LQRSpeedSteerVehicleState::new(
            current_state.x,
            current_state.y,
            current_state.yaw,
            current_state.v,
            self.config.wheelbase,
            self.config.max_steer,
        );

        let (delta, _, _, _, accel) = self.compute_control_full(&vehicle_state);

        let v = current_state.v + accel * self.config.dt;
        let omega = v * delta.tan() / self.config.wheelbase;

        ControlInput::new(v, omega)
    }

    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool {
        let dx = current_state.x - goal.x;
        let dy = current_state.y - goal.y;
        (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
    }
}

// --- Cubic spline helpers (same as in lqr_steer_control) ---

fn calc_spline_course(x: &[f64], y: &[f64], ds: f64) -> SplineCourse {
    let sp = CubicSpline2D::new(x, y);
    let mut s = 0.0;
    let mut course_x = Vec::new();
    let mut course_y = Vec::new();
    let mut course_yaw = Vec::new();
    let mut course_k = Vec::new();
    let mut course_s = Vec::new();

    // sp.s is always non-empty: CubicSpline2D::new initializes s with at least one element
    let s_max = *sp.s.last().expect("spline s is non-empty after construction") - ds;
    while s < s_max {
        let (ix, iy) = sp.calc_position(s);
        let iyaw = sp.calc_yaw(s);
        let ik = sp.calc_curvature(s);
        course_x.push(ix);
        course_y.push(iy);
        course_yaw.push(iyaw);
        course_k.push(ik);
        course_s.push(s);
        s += ds;
    }

    (course_x, course_y, course_yaw, course_k, course_s)
}

struct CubicSpline {
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
    x: Vec<f64>,
}

type SplineCourse = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>);

impl CubicSpline {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        let mut h = vec![0.0; n - 1];
        for i in 0..n - 1 {
            h[i] = x[i + 1] - x[i];
        }

        let mut a = vec![0.0; n];
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];

        a[..n].copy_from_slice(&y[..n]);

        let mut alpha = vec![0.0; n - 1];
        for i in 1..n - 1 {
            alpha[i] = 3.0 * (a[i + 1] - a[i]) / h[i] - 3.0 * (a[i] - a[i - 1]) / h[i - 1];
        }

        let mut l = vec![1.0; n];
        let mut mu = vec![0.0; n];
        let mut z = vec![0.0; n];

        for i in 1..n - 1 {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        for j in (0..n - 1).rev() {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        CubicSpline {
            a,
            b,
            c,
            d,
            x: x.to_vec(),
        }
    }

    fn calc(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return self.a[0];
        } else if t > self.x[self.x.len() - 1] {
            return self.a[self.a.len() - 1];
        }

        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }

        let dx = t - self.x[i];
        self.a[i] + self.b[i] * dx + self.c[i] * dx * dx + self.d[i] * dx * dx * dx
    }

    fn calc_d(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return self.b[0];
        } else if t > self.x[self.x.len() - 1] {
            return self.b[self.b.len() - 1];
        }

        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }

        let dx = t - self.x[i];
        self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx * dx
    }

    fn calc_dd(&self, t: f64) -> f64 {
        if t < self.x[0] {
            return 2.0 * self.c[0];
        } else if t > self.x[self.x.len() - 1] {
            return 2.0 * self.c[self.c.len() - 1];
        }

        let mut i = self.search_index(t);
        if i >= self.x.len() - 1 {
            i = self.x.len() - 2;
        }

        let dx = t - self.x[i];
        2.0 * self.c[i] + 6.0 * self.d[i] * dx
    }

    fn search_index(&self, x: f64) -> usize {
        for i in 0..self.x.len() - 1 {
            if self.x[i] <= x && x <= self.x[i + 1] {
                return i;
            }
        }
        self.x.len() - 2
    }
}

struct CubicSpline2D {
    s: Vec<f64>,
    sx: CubicSpline,
    sy: CubicSpline,
}

impl CubicSpline2D {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let mut s = vec![0.0];
        for i in 1..x.len() {
            let dx = x[i] - x[i - 1];
            let dy = y[i] - y[i - 1];
            s.push(s[i - 1] + (dx * dx + dy * dy).sqrt());
        }

        let sx = CubicSpline::new(&s, x);
        let sy = CubicSpline::new(&s, y);

        CubicSpline2D { s, sx, sy }
    }

    fn calc_position(&self, s: f64) -> (f64, f64) {
        (self.sx.calc(s), self.sy.calc(s))
    }

    fn calc_curvature(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let ddx = self.sx.calc_dd(s);
        let dy = self.sy.calc_d(s);
        let ddy = self.sy.calc_dd(s);
        let denom = (dx * dx + dy * dy).powf(1.5);
        if denom.abs() > 1e-6 {
            (ddy * dx - ddx * dy) / denom
        } else {
            0.0
        }
    }

    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_creation() {
        let config = LQRSpeedSteerConfig::default();
        let controller = LQRSpeedSteerController::new(config);
        assert!(controller.path.is_empty());
    }

    #[test]
    fn test_default_config() {
        let config = LQRSpeedSteerConfig::default();
        assert_eq!(config.q_diag.len(), 5);
        assert_eq!(config.r_diag.len(), 2);
        assert!((config.wheelbase - 0.5).abs() < 1e-9);
        assert!((config.dt - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_set_path() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
            Point2D::new(3.0, 0.0),
        ]);
        controller.set_path(path);
        assert_eq!(controller.get_path().len(), 4);
        assert_eq!(controller.path_yaw.len(), 4);
        assert_eq!(controller.path_curvature.len(), 4);
        assert_eq!(controller.speed_profile.len(), 4);
    }

    #[test]
    fn test_set_path_with_speed() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path = Path2D::from_points(vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.0),
            Point2D::new(2.0, 0.0),
        ]);
        controller.set_path_with_speed(path, 5.0);
        assert_eq!(controller.get_path().len(), 3);
    }

    #[test]
    fn test_normalize_angle() {
        let a = LQRSpeedSteerController::normalize_angle(3.0 * PI);
        assert!((a - PI).abs() < 0.01);
        let b = LQRSpeedSteerController::normalize_angle(-3.0 * PI);
        assert!((b + PI).abs() < 0.01);
        let c = LQRSpeedSteerController::normalize_angle(0.5);
        assert!((c - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_solve_dare_identity() {
        let a = Matrix5::identity();
        let mut b = Matrix5x2::zeros();
        b[(3, 0)] = 1.0;
        b[(4, 1)] = 0.1;
        let q = Matrix5::identity();
        let r = Matrix2::identity();

        let x = LQRSpeedSteerController::solve_dare(&a, &b, &q, &r);
        // Result should be positive semi-definite (diagonal >= 0)
        for i in 0..5 {
            assert!(x[(i, i)] >= 0.0);
        }
    }

    #[test]
    fn test_dlqr_gain() {
        let a = Matrix5::identity();
        let mut b = Matrix5x2::zeros();
        b[(3, 0)] = 1.0;
        b[(4, 1)] = 0.1;
        let q = Matrix5::identity();
        let r = Matrix2::identity();

        let k = LQRSpeedSteerController::dlqr(&a, &b, &q, &r);
        // K should be 2x5
        assert_eq!(k.nrows(), 2);
        assert_eq!(k.ncols(), 5);
    }

    #[test]
    fn test_compute_control_empty_path() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let state = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.78);
        let (delta, ind, e, th_e, accel) = controller.compute_control_full(&state);
        assert_eq!(delta, 0.0);
        assert_eq!(ind, 0);
        assert_eq!(e, 0.0);
        assert_eq!(th_e, 0.0);
        assert_eq!(accel, 0.0);
    }

    #[test]
    fn test_compute_control_straight_path() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path =
            Path2D::from_points((0..50).map(|i| Point2D::new(i as f64 * 0.5, 0.0)).collect());
        controller.set_path_with_speed(path, 2.0);

        let state = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.78);
        let (delta, _ind, _e, _th_e, _accel) = controller.compute_control_full(&state);
        // On a straight path aligned with the vehicle, steering should be near zero
        assert!(delta.abs() < 0.5);
    }

    #[test]
    fn test_vehicle_state_update() {
        let mut state = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.78);
        state.update(0.0, 0.0, 0.1);
        assert!((state.x - 0.1).abs() < 1e-9);
        assert!(state.y.abs() < 1e-9);
    }

    #[test]
    fn test_vehicle_state_update_with_steer() {
        let mut state = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.78);
        state.update(0.0, 0.1, 0.1);
        // Should have moved forward and slightly turned
        assert!(state.x > 0.0);
        assert!(state.yaw.abs() > 0.0);
    }

    #[test]
    fn test_vehicle_state_steer_clamping() {
        let mut state = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.1);
        state.update(0.0, 1.0, 0.1); // delta=1.0 exceeds max_steer=0.1
                                     // yaw change should be limited by max_steer
        let expected_yaw = 1.0 / 0.5 * (0.1_f64).tan() * 0.1;
        assert!((state.yaw - expected_yaw).abs() < 1e-9);
    }

    #[test]
    fn test_from_state2d() {
        let s = State2D::new(1.0, 2.0, 0.5, 3.0);
        let vs = LQRSpeedSteerVehicleState::from(s);
        assert!((vs.x - 1.0).abs() < 1e-9);
        assert!((vs.y - 2.0).abs() < 1e-9);
        assert!((vs.yaw - 0.5).abs() < 1e-9);
        assert!((vs.v - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_to_state2d() {
        let vs = LQRSpeedSteerVehicleState::new(1.0, 2.0, 0.5, 3.0, 0.5, 0.78);
        let s = vs.to_state2d();
        assert!((s.x - 1.0).abs() < 1e-9);
        assert!((s.y - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_is_goal_reached() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path = Path2D::from_points(vec![Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)]);
        controller.set_path(path);

        let far = LQRSpeedSteerVehicleState::new(0.0, 0.0, 0.0, 1.0, 0.5, 0.78);
        assert!(!controller.is_goal_reached(&far));

        let near = LQRSpeedSteerVehicleState::new(5.0, 0.1, 0.0, 0.0, 0.5, 0.78);
        assert!(controller.is_goal_reached(&near));
    }

    #[test]
    fn test_planning_straight_line() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let waypoints = vec![(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)];
        let result = controller.planning(waypoints, 2.0, 0.5);
        assert!(result.is_some());
        let traj = result.unwrap();
        assert!(!traj.is_empty());
        // Final position should be near the goal
        let (fx, fy) = traj.last().unwrap();
        let dist = ((fx - 10.0).powi(2) + fy.powi(2)).sqrt();
        assert!(dist < 2.0, "Final distance to goal: {}", dist);
    }

    #[test]
    fn test_planning_with_curve() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let waypoints = vec![
            (0.0, 0.0),
            (6.0, -3.0),
            (12.5, -5.0),
            (10.0, 6.5),
            (17.5, 3.0),
            (20.0, 0.0),
            (25.0, 0.0),
        ];
        let result = controller.planning(waypoints, 10.0 / 3.6, 0.1);
        assert!(result.is_some());
        let traj = result.unwrap();
        assert!(traj.len() > 10);
    }

    #[test]
    fn test_planning_too_few_waypoints() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let result = controller.planning(vec![(0.0, 0.0)], 2.0, 0.5);
        assert!(result.is_none());
    }

    #[test]
    fn test_path_tracker_trait() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path =
            Path2D::from_points((0..50).map(|i| Point2D::new(i as f64 * 0.5, 0.0)).collect());
        let state = State2D::new(0.0, 0.0, 0.0, 1.0);
        let input = controller.compute_control(&state, &path);
        // Should produce some valid control input
        assert!(input.v.is_finite());
        assert!(input.omega.is_finite());
    }

    #[test]
    fn test_speed_profile_slowdown() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path = Path2D::from_points(
            (0..100)
                .map(|i| Point2D::new(i as f64 * 0.1, 0.0))
                .collect(),
        );
        controller.set_path_with_speed(path, 3.0);
        // Speed near the end should be less than target speed
        let n = controller.speed_profile.len();
        assert!(controller.speed_profile[n - 1] < 3.0);
        assert!(controller.speed_profile[n - 1] > 0.0);
    }

    #[test]
    fn test_get_target_speed() {
        let mut controller = LQRSpeedSteerController::with_defaults();
        let path = Path2D::from_points(vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)]);
        controller.set_path_with_speed(path, 2.0);
        // Valid index
        let s = controller.get_target_speed(0);
        assert!(s.is_finite());
        // Out of bounds should return 0
        assert_eq!(controller.get_target_speed(1000), 0.0);
    }
}
