#![allow(dead_code)]

//! Model Predictive Control (MPC) for path tracking
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109)
//!         Rust port
//!
//! This version follows the PythonRobotics structure more closely:
//! - speed profile generation
//! - yaw smoothing
//! - iterative linear MPC around an operational point

use nalgebra::{Matrix2, Matrix4, Matrix4x2, Vector2, Vector4};
use std::f64::consts::PI;

// Vehicle parameters
const WB: f64 = 2.5; // wheelbase [m]
const MAX_STEER: f64 = 45.0 * PI / 180.0; // max steering angle [rad]
const MAX_DSTEER: f64 = 30.0 * PI / 180.0; // max steering rate [rad/s]
const MAX_SPEED: f64 = 55.0 / 3.6; // max speed [m/s]
const MIN_SPEED: f64 = -20.0 / 3.6; // min speed (reverse) [m/s]
const MAX_ACCEL: f64 = 1.0; // max acceleration [m/ss]

// MPC parameters, aligned with PythonRobotics
const T: usize = 5; // prediction horizon
const DT: f64 = 0.2; // time step [s]
const TARGET_SPEED: f64 = 10.0 / 3.6; // [m/s]
const GOAL_DIS: f64 = 1.5; // goal distance threshold
const STOP_SPEED: f64 = 0.5 / 3.6; // stop speed threshold
const MAX_ITER: usize = 3; // iterative linear MPC outer iterations
const DU_TH: f64 = 0.1; // outer-loop convergence threshold
const N_IND_SEARCH: usize = 10; // nearest-index search window
const PATH_RESOLUTION: f64 = 0.5; // [m]
const MAX_SIM_STEPS: usize = 2000;

// Inner projected-gradient solver settings
const QP_MAX_ITERS: usize = 40;
const LINE_SEARCH_ITERS: usize = 8;
const GRAD_TOL: f64 = 1.0e-3;
const COST_TOL: f64 = 1.0e-5;

// Cost weights
const Q: [f64; 4] = [1.0, 1.0, 0.5, 0.5];
const QF: [f64; 4] = Q;
const R: [f64; 2] = [0.01, 0.01];
const RD: [f64; 2] = [0.01, 1.0];

/// Vehicle state
#[derive(Clone, Copy, Debug)]
pub struct State {
    pub x: f64,
    pub y: f64,
    pub v: f64,
    pub yaw: f64,
}

impl State {
    pub fn new(x: f64, y: f64, v: f64, yaw: f64) -> Self {
        Self { x, y, v, yaw }
    }

    pub fn to_vector(self) -> Vector4<f64> {
        Vector4::new(self.x, self.y, self.v, self.yaw)
    }

    pub fn update(&mut self, accel: f64, steer: f64) {
        let steer = steer.clamp(-MAX_STEER, MAX_STEER);

        self.x += self.v * self.yaw.cos() * DT;
        self.y += self.v * self.yaw.sin() * DT;
        self.yaw += self.v / WB * steer.tan() * DT;
        self.yaw = normalize_angle(self.yaw);
        self.v += accel * DT;
        self.v = self.v.clamp(MIN_SPEED, MAX_SPEED);
    }
}

#[derive(Debug)]
pub struct MpcResult {
    pub controls: Vec<Vector2<f64>>,
    pub predicted: Vec<Vector4<f64>>,
}

#[derive(Debug)]
pub struct SimulationResult {
    pub cx: Vec<f64>,
    pub cy: Vec<f64>,
    pub hist_x: Vec<f64>,
    pub hist_y: Vec<f64>,
    pub predicted_x: Vec<f64>,
    pub predicted_y: Vec<f64>,
    pub goal: (f64, f64),
    pub reached_goal: bool,
    pub target_index: usize,
    pub final_index: usize,
    pub final_state: State,
}

pub fn normalize_angle(angle: f64) -> f64 {
    let mut value = angle % (2.0 * PI);
    if value > PI {
        value -= 2.0 * PI;
    } else if value < -PI {
        value += 2.0 * PI;
    }
    value
}

fn angle_diff(a: f64, b: f64) -> f64 {
    normalize_angle(a - b)
}

fn state_error(state: Vector4<f64>, reference: Vector4<f64>) -> Vector4<f64> {
    let mut error = state - reference;
    error[3] = angle_diff(state[3], reference[3]);
    error
}

fn control_cost_weight() -> Matrix2<f64> {
    Matrix2::from_diagonal(&Vector2::new(R[0], R[1]))
}

fn control_rate_weight() -> Matrix2<f64> {
    Matrix2::from_diagonal(&Vector2::new(RD[0], RD[1]))
}

fn state_cost_weight() -> Matrix4<f64> {
    Matrix4::from_diagonal(&Vector4::new(Q[0], Q[1], Q[2], Q[3]))
}

fn terminal_cost_weight() -> Matrix4<f64> {
    Matrix4::from_diagonal(&Vector4::new(QF[0], QF[1], QF[2], QF[3]))
}

fn get_linear_model_matrix(
    v: f64,
    yaw: f64,
    steer: f64,
) -> (Matrix4<f64>, Matrix4x2<f64>, Vector4<f64>) {
    let a = Matrix4::new(
        1.0,
        0.0,
        DT * yaw.cos(),
        -DT * v * yaw.sin(),
        0.0,
        1.0,
        DT * yaw.sin(),
        DT * v * yaw.cos(),
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        DT * steer.tan() / WB,
        1.0,
    );

    let b = Matrix4x2::new(
        0.0,
        0.0,
        0.0,
        0.0,
        DT,
        0.0,
        0.0,
        DT * v / (WB * steer.cos().powi(2)),
    );

    let c = Vector4::new(
        DT * v * yaw.sin() * yaw,
        -DT * v * yaw.cos() * yaw,
        0.0,
        -DT * v * steer / (WB * steer.cos().powi(2)),
    );

    (a, b, c)
}

/// Cubic spline for reference path
struct CubicSpline1D {
    x: Vec<f64>,
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
}

impl CubicSpline1D {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        let a = y.to_vec();
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];

        let h: Vec<f64> = (0..n - 1).map(|i| x[i + 1] - x[i]).collect();

        let mut alpha = vec![0.0; n];
        for i in 1..n - 1 {
            alpha[i] = 3.0 / h[i] * (a[i + 1] - a[i]) - 3.0 / h[i - 1] * (a[i] - a[i - 1]);
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

        Self {
            x: x.to_vec(),
            a,
            b,
            c,
            d,
        }
    }

    fn calc(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        let dx = t - self.x[i];
        self.a[i] + self.b[i] * dx + self.c[i] * dx.powi(2) + self.d[i] * dx.powi(3)
    }

    fn calc_d(&self, t: f64) -> f64 {
        let i = self.search_index(t);
        let dx = t - self.x[i];
        self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx.powi(2)
    }

    fn search_index(&self, t: f64) -> usize {
        for i in 0..self.x.len() - 1 {
            if self.x[i] <= t && t < self.x[i + 1] {
                return i;
            }
        }
        self.x.len() - 2
    }
}

/// 2D Cubic spline path
struct CubicSpline2D {
    s: Vec<f64>,
    sx: CubicSpline1D,
    sy: CubicSpline1D,
}

impl CubicSpline2D {
    fn new(x: &[f64], y: &[f64]) -> Self {
        let mut s = vec![0.0];
        for i in 1..x.len() {
            let ds = ((x[i] - x[i - 1]).powi(2) + (y[i] - y[i - 1]).powi(2)).sqrt();
            s.push(s[i - 1] + ds);
        }

        let sx = CubicSpline1D::new(&s, x);
        let sy = CubicSpline1D::new(&s, y);

        Self { s, sx, sy }
    }

    fn calc_position(&self, s: f64) -> (f64, f64) {
        (self.sx.calc(s), self.sy.calc(s))
    }

    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }
}

fn smooth_yaw(yaw: &mut [f64]) {
    for i in 0..yaw.len().saturating_sub(1) {
        let mut dyaw = yaw[i + 1] - yaw[i];
        while dyaw >= PI / 2.0 {
            yaw[i + 1] -= 2.0 * PI;
            dyaw = yaw[i + 1] - yaw[i];
        }
        while dyaw <= -PI / 2.0 {
            yaw[i + 1] += 2.0 * PI;
            dyaw = yaw[i + 1] - yaw[i];
        }
    }
}

pub fn calc_speed_profile(cx: &[f64], cy: &[f64], cyaw: &[f64], target_speed: f64) -> Vec<f64> {
    let mut speed_profile = vec![target_speed; cx.len()];
    let mut direction = 1.0;

    for i in 0..cx.len().saturating_sub(1) {
        let dx = cx[i + 1] - cx[i];
        let dy = cy[i + 1] - cy[i];
        let move_direction = dy.atan2(dx);

        if dx.abs() > f64::EPSILON && dy.abs() > f64::EPSILON {
            let dangle = angle_diff(move_direction, cyaw[i]).abs();
            direction = if dangle >= PI / 4.0 { -1.0 } else { 1.0 };
        }

        speed_profile[i] = direction * target_speed;
    }

    if let Some(last) = speed_profile.last_mut() {
        *last = 0.0;
    }

    speed_profile
}

fn calc_nearest_index(
    state: &State,
    cx: &[f64],
    cy: &[f64],
    cyaw: &[f64],
    pind: usize,
) -> (usize, f64) {
    let start = pind.min(cx.len().saturating_sub(1));
    let end = (start + N_IND_SEARCH).min(cx.len());

    let mut best_index = start;
    let mut best_distance_sq = f64::INFINITY;

    for i in start..end {
        let dx = state.x - cx[i];
        let dy = state.y - cy[i];
        let distance_sq = dx * dx + dy * dy;
        if distance_sq < best_distance_sq {
            best_distance_sq = distance_sq;
            best_index = i;
        }
    }

    let mut best_distance = best_distance_sq.sqrt();
    let dxl = cx[best_index] - state.x;
    let dyl = cy[best_index] - state.y;
    let angle = angle_diff(cyaw[best_index], dyl.atan2(dxl));
    if angle < 0.0 {
        best_distance *= -1.0;
    }

    (best_index, best_distance)
}

fn calc_ref_trajectory(
    state: &State,
    cx: &[f64],
    cy: &[f64],
    cyaw: &[f64],
    speed_profile: &[f64],
    pind: usize,
) -> (Vec<Vector4<f64>>, usize) {
    let ncourse = cx.len();
    let (mut ind, _) = calc_nearest_index(state, cx, cy, cyaw, pind);
    if pind >= ind {
        ind = pind;
    }

    let mut xref = vec![Vector4::zeros(); T + 1];
    xref[0] = Vector4::new(cx[ind], cy[ind], speed_profile[ind], cyaw[ind]);

    let mut travel = 0.0;
    for point in xref.iter_mut().take(T + 1).skip(1) {
        travel += state.v.abs() * DT;
        let dind = (travel / PATH_RESOLUTION).round() as usize;
        let index = (ind + dind).min(ncourse - 1);
        *point = Vector4::new(cx[index], cy[index], speed_profile[index], cyaw[index]);
    }

    (xref, ind)
}

fn predict_motion(state: State, controls: &[Vector2<f64>]) -> Vec<Vector4<f64>> {
    let mut predicted = vec![Vector4::zeros(); T + 1];
    predicted[0] = state.to_vector();

    let mut current = state;
    for (i, control) in controls.iter().enumerate().take(T) {
        current.update(control[0], control[1]);
        predicted[i + 1] = current.to_vector();
    }

    predicted
}

fn apply_control_constraints(controls: &mut [Vector2<f64>]) {
    for i in 0..controls.len() {
        controls[i][0] = controls[i][0].clamp(-MAX_ACCEL, MAX_ACCEL);
        controls[i][1] = controls[i][1].clamp(-MAX_STEER, MAX_STEER);

        if i > 0 {
            let delta = controls[i][1] - controls[i - 1][1];
            let max_delta = MAX_DSTEER * DT;
            if delta.abs() > max_delta {
                controls[i][1] = controls[i - 1][1] + max_delta * delta.signum();
            }
        }
    }
}

type LinearizedRollout = (
    Vec<Vector4<f64>>,
    Vec<Matrix4<f64>>,
    Vec<Matrix4x2<f64>>,
    Vec<Vector4<f64>>,
);

fn linearized_rollout(
    x0: &State,
    xbar: &[Vector4<f64>],
    controls: &[Vector2<f64>],
) -> LinearizedRollout {
    let mut x = vec![Vector4::zeros(); T + 1];
    let mut a_seq = Vec::with_capacity(T);
    let mut b_seq = Vec::with_capacity(T);
    let mut c_seq = Vec::with_capacity(T);

    x[0] = x0.to_vector();
    for t in 0..T {
        let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
        let mut next = a * x[t] + b * controls[t] + c;
        next[3] = normalize_angle(next[3]);

        a_seq.push(a);
        b_seq.push(b);
        c_seq.push(c);
        x[t + 1] = next;
    }

    (x, a_seq, b_seq, c_seq)
}

fn compute_cost(x: &[Vector4<f64>], xref: &[Vector4<f64>], controls: &[Vector2<f64>]) -> f64 {
    let q = state_cost_weight();
    let qf = terminal_cost_weight();
    let r = control_cost_weight();
    let rd = control_rate_weight();

    let mut cost = 0.0;
    for t in 0..T {
        cost += controls[t].dot(&(r * controls[t]));
        if t != 0 {
            let err = state_error(x[t], xref[t]);
            cost += err.dot(&(q * err));
        }
        if t < T - 1 {
            let du = controls[t + 1] - controls[t];
            cost += du.dot(&(rd * du));
        }
    }

    let terminal_error = state_error(x[T], xref[T]);
    cost + terminal_error.dot(&(qf * terminal_error))
}

fn optimize_linearized_controls(
    xref: &[Vector4<f64>],
    xbar: &[Vector4<f64>],
    state: &State,
    initial_controls: &[Vector2<f64>],
) -> Vec<Vector2<f64>> {
    let q = state_cost_weight();
    let qf = terminal_cost_weight();
    let r = control_cost_weight();
    let rd = control_rate_weight();

    let mut controls = initial_controls.to_vec();
    controls.resize(T, Vector2::zeros());
    apply_control_constraints(&mut controls);

    let mut previous_cost = f64::INFINITY;

    for _ in 0..QP_MAX_ITERS {
        let (x, a_seq, b_seq, _) = linearized_rollout(state, xbar, &controls);
        let current_cost = compute_cost(&x, xref, &controls);

        let mut gradients = [Vector2::zeros(); T];
        let mut lambda = (qf * state_error(x[T], xref[T])) * 2.0;

        for t in (0..T).rev() {
            let mut grad = (r * controls[t]) * 2.0 + b_seq[t].transpose() * lambda;

            if t > 0 {
                grad += (rd * (controls[t] - controls[t - 1])) * 2.0;
            }
            if t < T - 1 {
                grad -= (rd * (controls[t + 1] - controls[t])) * 2.0;
            }

            gradients[t] = grad;

            let lx = if t == 0 {
                Vector4::zeros()
            } else {
                (q * state_error(x[t], xref[t])) * 2.0
            };
            lambda = lx + a_seq[t].transpose() * lambda;
        }

        let gradient_norm = gradients
            .iter()
            .map(Vector2::norm_squared)
            .sum::<f64>()
            .sqrt();

        if gradient_norm <= GRAD_TOL || (previous_cost - current_cost).abs() <= COST_TOL {
            break;
        }

        previous_cost = current_cost;
        let current_controls = controls.clone();
        let mut step = 0.25;
        let mut improved = false;

        for _ in 0..LINE_SEARCH_ITERS {
            let mut candidate = current_controls.clone();
            for t in 0..T {
                candidate[t] -= gradients[t] * step;
            }
            apply_control_constraints(&mut candidate);

            let (candidate_x, _, _, _) = linearized_rollout(state, xbar, &candidate);
            let candidate_cost = compute_cost(&candidate_x, xref, &candidate);
            if candidate_cost < current_cost {
                controls = candidate;
                improved = true;
                break;
            }

            step *= 0.5;
        }

        if !improved {
            break;
        }
    }

    controls
}

pub fn iterative_linear_mpc_control(
    xref: &[Vector4<f64>],
    state: &State,
    warm_start: &[Vector2<f64>],
) -> MpcResult {
    let mut controls = warm_start.to_vec();
    controls.resize(T, Vector2::zeros());
    apply_control_constraints(&mut controls);

    let mut predicted = predict_motion(*state, &controls);
    for _ in 0..MAX_ITER {
        let previous_controls = controls.clone();
        controls = optimize_linearized_controls(xref, &predicted, state, &controls);
        predicted = predict_motion(*state, &controls);

        let du = controls
            .iter()
            .zip(previous_controls.iter())
            .map(|(current, previous)| {
                (current[0] - previous[0]).abs() + (current[1] - previous[1]).abs()
            })
            .sum::<f64>();

        if du <= DU_TH {
            break;
        }
    }

    MpcResult {
        controls,
        predicted,
    }
}

pub fn check_goal(
    state: &State,
    goal: (f64, f64),
    target_index: usize,
    final_index: usize,
) -> bool {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    let distance = (dx * dx + dy * dy).sqrt();

    let near_goal = distance <= GOAL_DIS;
    let near_end = final_index.abs_diff(target_index) < 5;
    let stopped = state.v.abs() <= STOP_SPEED;

    near_goal && near_end && stopped
}

pub fn generate_reference_course(ax: &[f64], ay: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let csp = CubicSpline2D::new(ax, ay);
    let s_max = *csp.s.last().unwrap_or(&0.0);

    let mut cx = Vec::new();
    let mut cy = Vec::new();
    let mut cyaw = Vec::new();

    let mut s = 0.0;
    while s <= s_max {
        let (x, y) = csp.calc_position(s);
        cx.push(x);
        cy.push(y);
        cyaw.push(csp.calc_yaw(s));
        s += PATH_RESOLUTION;
    }

    if cx.is_empty() || cy.is_empty() || cyaw.is_empty() {
        cx.push(ax[0]);
        cy.push(ay[0]);
        cyaw.push(0.0);
    }

    smooth_yaw(&mut cyaw);
    (cx, cy, cyaw)
}

pub fn run_mpc_simulation() -> SimulationResult {
    let ax = vec![0.0, 60.0, 125.0, 50.0, 75.0, 35.0, -10.0];
    let ay = vec![0.0, 0.0, 50.0, 65.0, 30.0, -10.0, -20.0];

    let (cx, cy, cyaw) = generate_reference_course(&ax, &ay);
    let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

    let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
    if state.yaw - cyaw[0] >= PI {
        state.yaw -= 2.0 * PI;
    } else if state.yaw - cyaw[0] <= -PI {
        state.yaw += 2.0 * PI;
    }

    let mut target_index = 0;
    let final_index = cx.len() - 1;
    let goal = (cx[final_index], cy[final_index]);

    let mut warm_start = vec![Vector2::zeros(); T];

    let mut hist_x = vec![state.x];
    let mut hist_y = vec![state.y];
    let mut predicted = vec![state.to_vector()];
    let mut reached_goal = false;

    for _ in 0..MAX_SIM_STEPS {
        let (xref, new_index) =
            calc_ref_trajectory(&state, &cx, &cy, &cyaw, &speed_profile, target_index);
        target_index = new_index;

        let mpc_result = iterative_linear_mpc_control(&xref, &state, &warm_start);
        let accel = mpc_result.controls[0][0];
        let steer = mpc_result.controls[0][1];

        state.update(accel, steer);
        hist_x.push(state.x);
        hist_y.push(state.y);
        predicted = mpc_result.predicted;

        warm_start = mpc_result.controls[1..].to_vec();
        warm_start.push(
            mpc_result
                .controls
                .last()
                .copied()
                .unwrap_or(Vector2::zeros()),
        );

        if check_goal(&state, goal, target_index, final_index) {
            reached_goal = true;
            break;
        }
    }

    SimulationResult {
        cx,
        cy,
        hist_x,
        hist_y,
        predicted_x: predicted.iter().map(|p| p[0]).collect(),
        predicted_y: predicted.iter().map(|p| p[1]).collect(),
        goal,
        reached_goal,
        target_index,
        final_index,
        final_state: state,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_speed_profile_stops_at_goal() {
        let cx = vec![0.0, 1.0, 2.0];
        let cy = vec![0.0, 0.0, 0.0];
        let cyaw = vec![0.0, 0.0, 0.0];
        let speed = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        assert_eq!(speed.len(), 3);
        assert_eq!(speed[2], 0.0);
        assert!(speed[0] > 0.0);
    }

    #[test]
    fn test_mpc_simulation_reaches_goal() {
        let result = run_mpc_simulation();
        let goal_distance = ((result.final_state.x - result.goal.0).powi(2)
            + (result.final_state.y - result.goal.1).powi(2))
        .sqrt();

        assert!(result.reached_goal);
        assert!(goal_distance <= GOAL_DIS + 0.25);
        assert!(result.hist_x.len() < MAX_SIM_STEPS);
    }
}
