//! LQR Steer Control path tracking algorithm
//!
//! A path tracking controller using Linear Quadratic Regulator (LQR)
//! for steering control combined with PID speed control.
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109)

use std::f64::consts::PI;
use nalgebra::{Matrix1, Matrix4, Vector4, Matrix1x4};
use crate::common::{Point2D, Path2D, State2D, ControlInput, PathTracker};

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
        LQRVehicleState { x, y, yaw, v, wheelbase, max_steer }
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
    /// Maximum steering angle [rad]
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
    }

    /// Get the current reference path
    pub fn get_path(&self) -> &Path2D {
        &self.path
    }

    /// Compute yaw and curvature from path points
    fn compute_path_derivatives(&self, path: &Path2D) -> (Vec<f64>, Vec<f64>) {
        let n = path.len();
        let mut yaw = Vec::with_capacity(n);
        let mut curvature = Vec::with_capacity(n);

        for i in 0..n {
            if i < n - 1 {
                let dx = path.points[i + 1].x - path.points[i].x;
                let dy = path.points[i + 1].y - path.points[i].y;
                yaw.push(dy.atan2(dx));
            } else if !yaw.is_empty() {
                yaw.push(*yaw.last().unwrap());
            } else {
                yaw.push(0.0);
            }

            // Simple curvature approximation
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

    /// Calculate speed profile based on path curvature
    fn calc_speed_profile(&self, path: &Path2D, target_speed: f64) -> Vec<f64> {
        let n = path.len();
        let mut profile = Vec::with_capacity(n);
        let mut direction = 1.0;

        for i in 0..n {
            if i < n - 1 && i < self.path_yaw.len() - 1 {
                let dyaw = (self.path_yaw[i + 1] - self.path_yaw[i]).abs();
                if PI / 4.0 <= dyaw && dyaw < PI / 2.0 {
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

    /// Normalize angle to [-PI, PI]
    fn normalize_angle(mut angle: f64) -> f64 {
        while angle > PI {
            angle -= 2.0 * PI;
        }
        while angle < -PI {
            angle += 2.0 * PI;
        }
        angle
    }

    /// Find target index and cross-track error
    fn calc_target_index(&self, state: &LQRVehicleState) -> (usize, f64) {
        let mut min_dist = f64::MAX;
        let mut min_idx = 0;

        for (i, p) in self.path.points.iter().enumerate() {
            let dx = state.x - p.x;
            let dy = state.y - p.y;
            let d = (dx * dx + dy * dy).sqrt();
            if d < min_dist {
                min_dist = d;
                min_idx = i;
            }
        }

        // Calculate signed cross-track error
        let target = &self.path.points[min_idx];
        let diff_x = target.x - state.x;
        let diff_y = target.y - state.y;
        let arcang = self.path_yaw[min_idx] - diff_y.atan2(diff_x);
        let angle = Self::normalize_angle(arcang);

        let error = if angle < 0.0 { -min_dist } else { min_dist };
        (min_idx, error)
    }

    /// Solve Discrete Algebraic Riccati Equation
    fn solve_dare(a: Matrix4<f64>, b: Vector4<f64>, q: Matrix4<f64>, r: Matrix1<f64>) -> Matrix4<f64> {
        let mut x = q;
        let max_iter = 150;
        let eps = 0.01;

        for _ in 0..max_iter {
            let bt_x_b = b.transpose() * x * b;
            let inv = (r + bt_x_b).try_inverse().unwrap_or(Matrix1::identity());
            let xn = a.transpose() * x * a
                - a.transpose() * x * b * inv * b.transpose() * x * a
                + q;

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
        let (ind, e) = self.calc_target_index(state);
        let k = if ind < self.path_curvature.len() {
            self.path_curvature[ind]
        } else {
            0.0
        };

        let th_e = Self::normalize_angle(state.yaw - self.path_yaw[ind]);
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
            if dt > 0.0 { (e - self.prev_error) / dt } else { 0.0 },
            th_e,
            if dt > 0.0 { (th_e - self.prev_theta_error) / dt } else { 0.0 },
        );

        // Feed-forward + feedback
        let ff = (state.wheelbase * k).atan2(1.0);
        let fb = Self::normalize_angle((-gain * x_err)[0]);
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
    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64, ds: f64) -> Option<Vec<(f64, f64)>> {
        if waypoints.len() < 2 {
            return None;
        }

        // Generate spline path
        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        let (cx, cy, cyaw, ck, _) = calc_spline_course(&ax, &ay, ds);

        // Set path
        let path = Path2D::from_points(
            cx.iter().zip(cy.iter()).map(|(&x, &y)| Point2D::new(x, y)).collect()
        );
        self.path_yaw = cyaw;
        self.path_curvature = ck;
        self.speed_profile = self.calc_speed_profile(&path, target_speed);
        self.path = path;

        // Simulate tracking
        let mut state = LQRVehicleState::new(
            0.0, 0.0, 0.0, 0.0,
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
        // Set path if different
        if self.path.len() != path.len() {
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

        let delta = self.compute_steering(&vehicle_state);
        let (target_idx, _) = self.calc_target_index(&vehicle_state);
        let target_v = self.get_target_speed(target_idx);

        let v = current_state.v + self.compute_acceleration(target_v, current_state.v) * self.config.dt;
        let omega = v * delta.tan() / self.config.wheelbase;

        ControlInput::new(v, omega)
    }

    fn is_goal_reached(&self, current_state: &State2D, goal: Point2D) -> bool {
        let dx = current_state.x - goal.x;
        let dy = current_state.y - goal.y;
        (dx * dx + dy * dy).sqrt() < self.config.goal_threshold
    }
}

// Cubic spline helper functions for legacy interface

fn calc_spline_course(x: &[f64], y: &[f64], ds: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let sp = CubicSpline2D::new(x, y);
    let mut s = 0.0;
    let mut course_x = Vec::new();
    let mut course_y = Vec::new();
    let mut course_yaw = Vec::new();
    let mut course_k = Vec::new();
    let mut course_s = Vec::new();

    let s_max = *sp.s.last().unwrap() - ds;
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

        for i in 0..n {
            a[i] = y[i];
        }

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

        CubicSpline { a, b, c, d, x: x.to_vec() }
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
        let x = self.sx.calc(s);
        let y = self.sy.calc(s);
        (x, y)
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
        assert!((LQRSteerController::normalize_angle(3.0 * PI) - PI).abs() < 0.01);
        assert!((LQRSteerController::normalize_angle(-3.0 * PI) + PI).abs() < 0.01);
    }

    #[test]
    fn test_lqr_planning() {
        let mut controller = LQRSteerController::with_defaults();
        let waypoints = vec![
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
        ];

        let result = controller.planning(waypoints, 2.0, 0.5);
        assert!(result.is_some());
        assert!(result.unwrap().len() > 0);
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
