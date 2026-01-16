//! Stanley Controller path tracking algorithm
//!
//! A path tracking controller based on Stanley control law that uses
//! heading error and cross-track error for steering computation.
//!
//! Ref:
//!     - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
//!     - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

use std::f64::consts::PI;
use crate::common::{Point2D, Path2D, State2D, ControlInput, PathTracker};

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
        VehicleState { x, y, yaw, v, wheelbase }
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
        // Compute yaw angles for the path
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
        let mut yaw = Vec::with_capacity(path.len());
        for i in 0..path.len() {
            if i < path.len() - 1 {
                let dx = path.points[i + 1].x - path.points[i].x;
                let dy = path.points[i + 1].y - path.points[i].y;
                yaw.push(dy.atan2(dx));
            } else if !yaw.is_empty() {
                yaw.push(*yaw.last().unwrap());
            } else {
                yaw.push(0.0);
            }
        }
        yaw
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
    fn calc_target_index(&self, state: &VehicleState) -> (usize, f64) {
        let (fx, fy) = state.front_axle();

        let mut min_dist = f64::MAX;
        let mut min_idx = 0;

        for (i, p) in self.path.points.iter().enumerate() {
            let dx = fx - p.x;
            let dy = fy - p.y;
            let d = (dx * dx + dy * dy).sqrt();
            if d < min_dist {
                min_dist = d;
                min_idx = i;
            }
        }

        // Calculate cross-track error
        let target_point = &self.path.points[min_idx];
        let diff_x = fx - target_point.x;
        let diff_y = fy - target_point.y;
        let error_front_axle = -(state.yaw + 0.5 * PI).cos() * diff_x
            - (state.yaw + 0.5 * PI).sin() * diff_y;

        (min_idx, error_front_axle)
    }

    /// Compute steering angle using Stanley control law
    pub fn compute_steering(&mut self, state: &VehicleState) -> f64 {
        let (mut target_idx, error_front_axle) = self.calc_target_index(state);

        if self.last_target_idx >= target_idx {
            target_idx = self.last_target_idx;
        }
        self.last_target_idx = target_idx;

        // Heading error
        let theta_e = Self::normalize_angle(self.path_yaw[target_idx] - state.yaw);

        // Cross-track error correction
        let theta_d = (self.config.k * error_front_axle).atan2(state.v.max(0.1));

        // Total steering angle
        theta_e + theta_d
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
    pub fn planning(&mut self, waypoints: Vec<(f64, f64)>, target_speed: f64, ds: f64) -> Option<Vec<(f64, f64)>> {
        if waypoints.len() < 2 {
            return None;
        }

        // Generate spline path
        let ax: Vec<f64> = waypoints.iter().map(|p| p.0).collect();
        let ay: Vec<f64> = waypoints.iter().map(|p| p.1).collect();
        let (cx, cy, cyaw, _, _) = calc_spline_course(&ax, &ay, ds);

        // Set path
        let path = Path2D::from_points(
            cx.iter().zip(cy.iter()).map(|(&x, &y)| Point2D::new(x, y)).collect()
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
        // Set path if different
        if self.path.len() != path.len() {
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
        (ddy * dx - ddx * dy) / (dx * dx + dy * dy).powf(1.5)
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
        assert!((StanleyController::normalize_angle(3.0 * PI) - PI).abs() < 0.01);
        assert!((StanleyController::normalize_angle(-3.0 * PI) + PI).abs() < 0.01);
    }

    #[test]
    fn test_stanley_planning() {
        let mut controller = StanleyController::with_params(0.5, 2.9);
        let waypoints = vec![
            (0.0, 0.0),
            (50.0, 0.0),
            (100.0, 0.0),
        ];

        let result = controller.planning(waypoints, 5.0, 0.5);
        assert!(result.is_some());
        assert!(result.unwrap().len() > 0);
    }
}
