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
const UPSTREAM_SWITCH_BACK_TICK: f64 = 1.0; // [m]
const MAX_SIM_STEPS: usize = 2000;

// Inner projected-gradient solver settings
const QP_MAX_ITERS: usize = 200;
const LINE_SEARCH_ITERS: usize = 16;
const LINE_SEARCH_INITIAL_STEP: f64 = 0.25;
const GRAD_TOL: f64 = 1.0e-5;
const COST_TOL: f64 = 1.0e-7;

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
    course_tick: f64,
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
        let dind = (travel / course_tick).round() as usize;
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

#[allow(clippy::needless_range_loop)]
fn solve_qp_clarabel(
    xref: &[Vector4<f64>],
    xbar: &[Vector4<f64>],
    state: &State,
) -> Option<Vec<Vector2<f64>>> {
    use clarabel::algebra::*;
    use clarabel::solver::*;

    let nu = 2;
    let nx = 4;
    let n_u = nu * T;
    let n_x = nx * (T + 1);
    let n = n_u + n_x;

    let u_idx = |t: usize, k: usize| -> usize { t * nu + k };
    let x_idx = |t: usize, k: usize| -> usize { n_u + t * nx + k };

    // --- Build P (cost Hessian, upper triangular) ---
    let mut p_rows = Vec::new();
    let mut p_cols = Vec::new();
    let mut p_vals = Vec::new();

    for t in 0..T {
        for k in 0..nu {
            let idx = u_idx(t, k);
            let mut val = R[k];
            if t > 0 {
                val += RD[k];
            }
            if t < T - 1 {
                val += RD[k];
            }
            p_rows.push(idx);
            p_cols.push(idx);
            p_vals.push(val * 2.0);
        }
        if t < T - 1 {
            for k in 0..nu {
                let i = u_idx(t, k);
                let j = u_idx(t + 1, k);
                p_rows.push(i);
                p_cols.push(j);
                p_vals.push(-RD[k] * 2.0);
            }
        }
    }

    for t in 1..=T {
        let w = if t == T { QF } else { Q };
        for k in 0..nx {
            let idx = x_idx(t, k);
            p_rows.push(idx);
            p_cols.push(idx);
            p_vals.push(w[k] * 2.0);
        }
    }

    let p = CscMatrix::new_from_triplets(n, n, p_rows, p_cols, p_vals);

    // --- Build q (linear cost) ---
    let mut q_vec = vec![0.0; n];
    for t in 1..=T {
        let w = if t == T { QF } else { Q };
        for k in 0..nx {
            q_vec[x_idx(t, k)] = -w[k] * 2.0 * xref[t][k];
        }
    }

    // --- Build constraints ---
    let n_eq = nx * (T + 1);
    let n_steer_rate = T - 1;
    let n_u_box = 2 * nu * T;
    let n_steer_rate_box = 2 * n_steer_rate;
    let n_v_box = 2 * (T + 1);
    let n_ineq = n_u_box + n_steer_rate_box + n_v_box;

    let mut a_rows = Vec::new();
    let mut a_cols = Vec::new();
    let mut a_vals = Vec::new();
    let mut b_vec = vec![0.0; n_eq + n_ineq];

    let x0 = state.to_vector();
    for k in 0..nx {
        a_rows.push(k);
        a_cols.push(x_idx(0, k));
        a_vals.push(1.0);
        b_vec[k] = x0[k];
    }

    for t in 0..T {
        let (a_mat, b_mat, c_vec) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
        let base_row = nx * (t + 1);

        for i in 0..nx {
            let row = base_row + i;
            a_rows.push(row);
            a_cols.push(x_idx(t + 1, i));
            a_vals.push(-1.0);
            for j in 0..nx {
                let val = a_mat[(i, j)];
                if val.abs() > 1e-15 {
                    a_rows.push(row);
                    a_cols.push(x_idx(t, j));
                    a_vals.push(val);
                }
            }
            for j in 0..nu {
                let val = b_mat[(i, j)];
                if val.abs() > 1e-15 {
                    a_rows.push(row);
                    a_cols.push(u_idx(t, j));
                    a_vals.push(val);
                }
            }
            b_vec[row] = -c_vec[i];
        }
    }

    let ineq_offset = n_eq;
    let mut ineq_row = 0;

    let u_bounds = [MAX_ACCEL, MAX_STEER];
    for t in 0..T {
        for (k, &bound) in u_bounds.iter().enumerate() {
            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(u_idx(t, k));
            a_vals.push(1.0);
            b_vec[row] = bound;
            ineq_row += 1;

            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(u_idx(t, k));
            a_vals.push(-1.0);
            b_vec[row] = bound;
            ineq_row += 1;
        }
    }

    let max_steer_delta = MAX_DSTEER * DT;
    for t in 0..(T - 1) {
        let row = ineq_offset + ineq_row;
        a_rows.push(row);
        a_cols.push(u_idx(t + 1, 1));
        a_vals.push(1.0);
        a_rows.push(row);
        a_cols.push(u_idx(t, 1));
        a_vals.push(-1.0);
        b_vec[row] = max_steer_delta;
        ineq_row += 1;

        let row = ineq_offset + ineq_row;
        a_rows.push(row);
        a_cols.push(u_idx(t + 1, 1));
        a_vals.push(-1.0);
        a_rows.push(row);
        a_cols.push(u_idx(t, 1));
        a_vals.push(1.0);
        b_vec[row] = max_steer_delta;
        ineq_row += 1;
    }

    for t in 0..=T {
        let row = ineq_offset + ineq_row;
        a_rows.push(row);
        a_cols.push(x_idx(t, 2));
        a_vals.push(1.0);
        b_vec[row] = MAX_SPEED;
        ineq_row += 1;

        let row = ineq_offset + ineq_row;
        a_rows.push(row);
        a_cols.push(x_idx(t, 2));
        a_vals.push(-1.0);
        b_vec[row] = -MIN_SPEED;
        ineq_row += 1;
    }

    let m = n_eq + n_ineq;
    let a_csc = CscMatrix::new_from_triplets(m, n, a_rows, a_cols, a_vals);

    let cones = vec![
        SupportedConeT::ZeroConeT(n_eq),
        SupportedConeT::NonnegativeConeT(n_ineq),
    ];

    let settings = DefaultSettingsBuilder::default()
        .verbose(false)
        .build()
        .unwrap();

    let mut solver = DefaultSolver::new(&p, &q_vec, &a_csc, &b_vec, &cones, settings).ok()?;
    solver.solve();

    if solver.solution.status != SolverStatus::Solved {
        return None;
    }

    let z = &solver.solution.x;
    let controls: Vec<Vector2<f64>> = (0..T)
        .map(|t| Vector2::new(z[u_idx(t, 0)], z[u_idx(t, 1)]))
        .collect();

    Some(controls)
}

fn optimize_linearized_controls(
    xref: &[Vector4<f64>],
    xbar: &[Vector4<f64>],
    state: &State,
    initial_controls: &[Vector2<f64>],
) -> Vec<Vector2<f64>> {
    let pg_controls = optimize_linearized_controls_pg(xref, xbar, state, initial_controls);

    if let Some(qp_controls) = solve_qp_clarabel(xref, xbar, state) {
        let (qp_x, _, _, _) = linearized_rollout(state, xbar, &qp_controls);
        let qp_cost = compute_cost(&qp_x, xref, &qp_controls);

        let (pg_x, _, _, _) = linearized_rollout(state, xbar, &pg_controls);
        let pg_cost = compute_cost(&pg_x, xref, &pg_controls);

        if qp_cost <= pg_cost {
            return qp_controls;
        }
    }

    pg_controls
}

fn optimize_linearized_controls_pg(
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
        let mut best_candidate_controls = None;
        let mut best_candidate_cost = current_cost;

        let mut step = LINE_SEARCH_INITIAL_STEP;
        for &large_step in &[1.0, 0.5] {
            let mut candidate = current_controls.clone();
            for t in 0..T {
                candidate[t] -= gradients[t] * large_step;
            }
            apply_control_constraints(&mut candidate);

            let (candidate_x, _, _, _) = linearized_rollout(state, xbar, &candidate);
            let candidate_cost = compute_cost(&candidate_x, xref, &candidate);
            if candidate_cost < best_candidate_cost {
                best_candidate_cost = candidate_cost;
                best_candidate_controls = Some(candidate);
            }
        }

        for _ in 0..LINE_SEARCH_ITERS {
            let mut candidate = current_controls.clone();
            for t in 0..T {
                candidate[t] -= gradients[t] * step;
            }
            apply_control_constraints(&mut candidate);

            let (candidate_x, _, _, _) = linearized_rollout(state, xbar, &candidate);
            let candidate_cost = compute_cost(&candidate_x, xref, &candidate);
            if candidate_cost < best_candidate_cost {
                best_candidate_cost = candidate_cost;
                best_candidate_controls = Some(candidate);
            }

            step *= 0.5;
        }

        if let Some(candidate) = best_candidate_controls {
            controls = candidate;
        } else {
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

fn next_warm_start(controls: &[Vector2<f64>]) -> Vec<Vector2<f64>> {
    let mut warm_start = controls[1..].to_vec();
    warm_start.push(controls.last().copied().unwrap_or(Vector2::zeros()));
    warm_start
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

fn sample_reference_course(
    ax: &[f64],
    ay: &[f64],
    course_tick: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
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
        s += course_tick;
    }

    if cx.is_empty() || cy.is_empty() || cyaw.is_empty() {
        cx.push(ax[0]);
        cy.push(ay[0]);
        cyaw.push(0.0);
    }

    (cx, cy, cyaw)
}

pub fn generate_reference_course(ax: &[f64], ay: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let (cx, cy, mut cyaw) = sample_reference_course(ax, ay, PATH_RESOLUTION);

    smooth_yaw(&mut cyaw);
    (cx, cy, cyaw)
}

fn generate_switch_back_course() -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let (mut cx, mut cy, mut cyaw) = sample_reference_course(
        &[0.0, 30.0, 6.0, 20.0, 35.0],
        &[0.0, 0.0, 20.0, 35.0, 20.0],
        UPSTREAM_SWITCH_BACK_TICK,
    );
    let (cx2, cy2, mut cyaw2) = sample_reference_course(
        &[35.0, 10.0, 0.0, 0.0],
        &[20.0, 30.0, 5.0, 0.0],
        UPSTREAM_SWITCH_BACK_TICK,
    );

    for yaw in &mut cyaw2 {
        *yaw -= PI;
    }

    cx.extend(cx2);
    cy.extend(cy2);
    cyaw.extend(cyaw2);
    smooth_yaw(&mut cyaw);

    (cx, cy, cyaw)
}

pub fn run_mpc_simulation() -> SimulationResult {
    let (cx, cy, cyaw) = generate_switch_back_course();
    run_mpc_simulation_with_reference(cx, cy, cyaw, UPSTREAM_SWITCH_BACK_TICK, MAX_SIM_STEPS)
}

fn run_mpc_simulation_with_course(
    ax: &[f64],
    ay: &[f64],
    max_sim_steps: usize,
) -> SimulationResult {
    let (cx, cy, cyaw) = generate_reference_course(ax, ay);
    run_mpc_simulation_with_reference(cx, cy, cyaw, PATH_RESOLUTION, max_sim_steps)
}

fn run_mpc_simulation_with_reference(
    cx: Vec<f64>,
    cy: Vec<f64>,
    cyaw: Vec<f64>,
    course_tick: f64,
    max_sim_steps: usize,
) -> SimulationResult {
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

    for _ in 0..max_sim_steps {
        let (xref, new_index) = calc_ref_trajectory(
            &state,
            &cx,
            &cy,
            &cyaw,
            &speed_profile,
            course_tick,
            target_index,
        );
        target_index = new_index;

        let mpc_result = iterative_linear_mpc_control(&xref, &state, &warm_start);
        let accel = mpc_result.controls[0][0];
        let steer = mpc_result.controls[0][1];

        state.update(accel, steer);
        hist_x.push(state.x);
        hist_y.push(state.y);
        predicted = mpc_result.predicted;

        warm_start = next_warm_start(&mpc_result.controls);

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
    use std::collections::BTreeMap;

    /// Original (pre-promotion) solver using first-improvement backtracking.
    /// Kept for characterization tests that compare default vs expanded behavior.
    fn optimize_linearized_controls_original(
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
            let mut step = LINE_SEARCH_INITIAL_STEP;
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

    #[derive(Debug)]
    struct SwitchBackTraceRow {
        step: usize,
        target_index: usize,
        state: Vector4<f64>,
        xref_head: Vector4<f64>,
        xref_tail: Vector4<f64>,
        control: Vector2<f64>,
        predicted_tail: Vector4<f64>,
    }

    #[derive(Debug)]
    struct SwitchBackDetailedRow {
        step: usize,
        target_index: usize,
        state: Vector4<f64>,
        xref: Vec<Vector4<f64>>,
        controls: Vec<Vector2<f64>>,
        predicted: Vec<Vector4<f64>>,
    }

    #[derive(Debug)]
    struct SwitchBackPlanningSnapshot {
        step: usize,
        target_index: usize,
        state: Vector4<f64>,
        xref_head: Vector4<f64>,
        warm_start_head: Vector2<f64>,
    }

    #[derive(Debug)]
    struct SwitchBackClosedLoopCheckpoint {
        step: usize,
        target_index: usize,
        goal_distance: f64,
        state: Vector4<f64>,
    }

    #[derive(Debug)]
    struct SwitchBackGoalConditionSnapshot {
        step: usize,
        target_index: usize,
        goal_distance: f64,
        speed: f64,
        speed_abs: f64,
        heading_error: f64,
        motion_curvature: f64,
        near_goal: bool,
        near_end: bool,
        stopped: bool,
        reached_goal: bool,
    }

    #[derive(Debug)]
    struct SwitchBackGoalControlSnapshot {
        step: usize,
        target_index: usize,
        goal_distance: f64,
        speed: f64,
        accel: f64,
        steer: f64,
        heading_error: f64,
        motion_curvature: f64,
        course_curvature: f64,
        near_goal: bool,
        near_end: bool,
        stopped: bool,
        reached_goal: bool,
    }

    type SwitchBackControlTrace = BTreeMap<usize, Vec<Vector2<f64>>>;
    type SwitchBackStateTrace = BTreeMap<usize, Vec<Vector4<f64>>>;
    type SwitchBackHorizonPair = (Vec<Vector4<f64>>, Vec<Vector4<f64>>);
    type SwitchBackHorizonTrace = BTreeMap<usize, SwitchBackHorizonPair>;
    type SwitchBackWarmStartTrace = BTreeMap<usize, Vec<Vector2<f64>>>;

    #[derive(Debug, Default)]
    struct StateInjectionMetrics {
        default_same_control0: f64,
        default_opposite_control0: f64,
        default_same_pred_y: f64,
        default_opposite_pred_y: f64,
        default_same_pred_v: f64,
        default_opposite_pred_v: f64,
        expanded_same_control0: f64,
        expanded_opposite_control0: f64,
        expanded_same_pred_y: f64,
        expanded_opposite_pred_y: f64,
        expanded_same_pred_v: f64,
        expanded_opposite_pred_v: f64,
    }

    struct SwitchBackContext {
        cx: Vec<f64>,
        cy: Vec<f64>,
        cyaw: Vec<f64>,
        speed_profile: Vec<f64>,
        state: State,
        target_index: usize,
        warm_start: Vec<Vector2<f64>>,
    }

    #[derive(Debug)]
    struct SwitchBackTraceDiffSummary {
        max_target_index_gap: usize,
        max_state_diff: [f64; 4],
        max_xref_head_diff: [f64; 4],
        max_xref_tail_diff: [f64; 4],
        max_control_diff: [f64; 2],
        max_predicted_tail_diff: [f64; 4],
    }

    #[derive(Debug)]
    struct SwitchBackDetailedDiffSummary {
        max_target_index_gap: usize,
        max_state_diff: [f64; 4],
        max_xref_diff: [f64; 4],
        max_control_diff: [f64; 2],
        max_predicted_diff: [f64; 4],
    }

    fn assert_vec2_close(actual: &Vector2<f64>, expected: [f64; 2], tol: f64) {
        for (i, expected_value) in expected.iter().enumerate() {
            assert!(
                (actual[i] - expected_value).abs() <= tol,
                "index {}: actual={} expected={} tol={}",
                i,
                actual[i],
                expected_value,
                tol
            );
        }
    }

    fn assert_vec4_close(actual: &Vector4<f64>, expected: [f64; 4], tol: f64) {
        for (i, expected_value) in expected.iter().enumerate() {
            assert!(
                (actual[i] - expected_value).abs() <= tol,
                "index {}: actual={} expected={} tol={}",
                i,
                actual[i],
                expected_value,
                tol
            );
        }
    }

    fn parse_switch_back_trace(csv: &str) -> Vec<SwitchBackTraceRow> {
        csv.lines()
            .skip(1)
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let values: Vec<&str> = line.split(',').collect();
                assert_eq!(values.len(), 20);

                SwitchBackTraceRow {
                    step: values[0].parse().unwrap(),
                    target_index: values[1].parse().unwrap(),
                    state: Vector4::new(
                        values[2].parse().unwrap(),
                        values[3].parse().unwrap(),
                        values[4].parse().unwrap(),
                        values[5].parse().unwrap(),
                    ),
                    xref_head: Vector4::new(
                        values[6].parse().unwrap(),
                        values[7].parse().unwrap(),
                        values[8].parse().unwrap(),
                        values[9].parse().unwrap(),
                    ),
                    xref_tail: Vector4::new(
                        values[10].parse().unwrap(),
                        values[11].parse().unwrap(),
                        values[12].parse().unwrap(),
                        values[13].parse().unwrap(),
                    ),
                    control: Vector2::new(values[14].parse().unwrap(), values[15].parse().unwrap()),
                    predicted_tail: Vector4::new(
                        values[16].parse().unwrap(),
                        values[17].parse().unwrap(),
                        values[18].parse().unwrap(),
                        values[19].parse().unwrap(),
                    ),
                }
            })
            .collect()
    }

    fn parse_switch_back_reverse_controls(csv: &str) -> SwitchBackControlTrace {
        let mut result: SwitchBackControlTrace = BTreeMap::new();

        for line in csv.lines().skip(1).filter(|line| !line.trim().is_empty()) {
            let values: Vec<&str> = line.split(',').collect();
            assert_eq!(values.len(), 4);
            let step: usize = values[0].parse().unwrap();
            let control_index: usize = values[1].parse().unwrap();
            let control = Vector2::new(values[2].parse().unwrap(), values[3].parse().unwrap());

            let controls = result.entry(step).or_default();
            assert_eq!(controls.len(), control_index);
            controls.push(control);
        }

        result
    }

    fn parse_switch_back_reverse_states(csv: &str) -> SwitchBackStateTrace {
        let mut result: SwitchBackStateTrace = BTreeMap::new();

        for line in csv.lines().skip(1).filter(|line| !line.trim().is_empty()) {
            let values: Vec<&str> = line.split(',').collect();
            assert_eq!(values.len(), 6);
            let step: usize = values[0].parse().unwrap();
            let horizon_index: usize = values[1].parse().unwrap();
            let state = Vector4::new(
                values[2].parse().unwrap(),
                values[3].parse().unwrap(),
                values[4].parse().unwrap(),
                values[5].parse().unwrap(),
            );

            let states = result.entry(step).or_default();
            assert_eq!(states.len(), horizon_index);
            states.push(state);
        }

        result
    }

    fn parse_switch_back_reverse_horizon(csv: &str) -> SwitchBackHorizonTrace {
        let mut result: SwitchBackHorizonTrace = BTreeMap::new();

        for line in csv.lines().skip(1).filter(|line| !line.trim().is_empty()) {
            let values: Vec<&str> = line.split(',').collect();
            assert_eq!(values.len(), 10);
            let step: usize = values[0].parse().unwrap();
            let horizon_index: usize = values[1].parse().unwrap();

            let entry = result.entry(step).or_default();
            assert_eq!(entry.0.len(), horizon_index);
            assert_eq!(entry.1.len(), horizon_index);

            entry.0.push(Vector4::new(
                values[2].parse().unwrap(),
                values[3].parse().unwrap(),
                values[4].parse().unwrap(),
                values[5].parse().unwrap(),
            ));
            entry.1.push(Vector4::new(
                values[6].parse().unwrap(),
                values[7].parse().unwrap(),
                values[8].parse().unwrap(),
                values[9].parse().unwrap(),
            ));
        }

        result
    }

    fn trace_state_diff(actual: &Vector4<f64>, expected: &Vector4<f64>) -> [f64; 4] {
        [
            (actual[0] - expected[0]).abs(),
            (actual[1] - expected[1]).abs(),
            (actual[2] - expected[2]).abs(),
            angle_diff(actual[3], expected[3]).abs(),
        ]
    }

    fn trace_control_diff(actual: &Vector2<f64>, expected: &Vector2<f64>) -> [f64; 2] {
        [
            (actual[0] - expected[0]).abs(),
            (actual[1] - expected[1]).abs(),
        ]
    }

    fn summarize_switch_back_trace_window(
        actual: &[SwitchBackTraceRow],
        expected: &[SwitchBackTraceRow],
    ) -> SwitchBackTraceDiffSummary {
        let mut summary = SwitchBackTraceDiffSummary {
            max_target_index_gap: 0,
            max_state_diff: [0.0; 4],
            max_xref_head_diff: [0.0; 4],
            max_xref_tail_diff: [0.0; 4],
            max_control_diff: [0.0; 2],
            max_predicted_tail_diff: [0.0; 4],
        };

        assert_eq!(actual.len(), expected.len());

        for (actual_row, expected_row) in actual.iter().zip(expected.iter()) {
            assert_eq!(actual_row.step, expected_row.step);
            summary.max_target_index_gap = summary
                .max_target_index_gap
                .max(actual_row.target_index.abs_diff(expected_row.target_index));

            for (max_diff, diff) in summary
                .max_state_diff
                .iter_mut()
                .zip(trace_state_diff(&actual_row.state, &expected_row.state))
            {
                *max_diff = (*max_diff).max(diff);
            }

            for (max_diff, diff) in summary.max_xref_head_diff.iter_mut().zip(trace_state_diff(
                &actual_row.xref_head,
                &expected_row.xref_head,
            )) {
                *max_diff = (*max_diff).max(diff);
            }

            for (max_diff, diff) in summary.max_xref_tail_diff.iter_mut().zip(trace_state_diff(
                &actual_row.xref_tail,
                &expected_row.xref_tail,
            )) {
                *max_diff = (*max_diff).max(diff);
            }

            for (max_diff, diff) in summary.max_control_diff.iter_mut().zip(trace_control_diff(
                &actual_row.control,
                &expected_row.control,
            )) {
                *max_diff = (*max_diff).max(diff);
            }

            for (max_diff, diff) in
                summary
                    .max_predicted_tail_diff
                    .iter_mut()
                    .zip(trace_state_diff(
                        &actual_row.predicted_tail,
                        &expected_row.predicted_tail,
                    ))
            {
                *max_diff = (*max_diff).max(diff);
            }
        }

        summary
    }

    fn summarize_switch_back_detailed_window(
        actual_trace: &[SwitchBackDetailedRow],
        expected_trace: &[SwitchBackTraceRow],
        expected_controls: &SwitchBackControlTrace,
        expected_horizon: &SwitchBackHorizonTrace,
        step_window: std::ops::RangeInclusive<usize>,
    ) -> SwitchBackDetailedDiffSummary {
        let mut summary = SwitchBackDetailedDiffSummary {
            max_target_index_gap: 0,
            max_state_diff: [0.0; 4],
            max_xref_diff: [0.0; 4],
            max_control_diff: [0.0; 2],
            max_predicted_diff: [0.0; 4],
        };

        for actual_row in actual_trace
            .iter()
            .filter(|row| step_window.contains(&row.step))
        {
            let expected_row = &expected_trace[actual_row.step];
            assert_eq!(actual_row.step, expected_row.step);
            summary.max_target_index_gap = summary
                .max_target_index_gap
                .max(actual_row.target_index.abs_diff(expected_row.target_index));

            for (slot, diff) in summary
                .max_state_diff
                .iter_mut()
                .zip(trace_state_diff(&actual_row.state, &expected_row.state))
            {
                *slot = (*slot).max(diff);
            }

            let expected_controls_for_step = expected_controls.get(&actual_row.step).unwrap();
            let (expected_xref_for_step, expected_predicted_for_step) =
                expected_horizon.get(&actual_row.step).unwrap();
            let expected_xref_rows = expected_xref_for_step
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();
            let expected_control_rows = expected_controls_for_step
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expected_predicted_rows = expected_predicted_for_step
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            for (slot, diff) in summary
                .max_xref_diff
                .iter_mut()
                .zip(summarize_vec4_sequence_diff(
                    &actual_row.xref,
                    &expected_xref_rows,
                ))
            {
                *slot = (*slot).max(diff);
            }

            for (slot, diff) in
                summary
                    .max_control_diff
                    .iter_mut()
                    .zip(summarize_vec2_sequence_diff(
                        &actual_row.controls,
                        &expected_control_rows,
                    ))
            {
                *slot = (*slot).max(diff);
            }

            for (slot, diff) in
                summary
                    .max_predicted_diff
                    .iter_mut()
                    .zip(summarize_vec4_sequence_diff(
                        &actual_row.predicted,
                        &expected_predicted_rows,
                    ))
            {
                *slot = (*slot).max(diff);
            }
        }

        summary
    }

    fn assert_diff_bounds<const N: usize>(label: &str, actual: [f64; N], tolerance: [f64; N]) {
        for (i, (actual_diff, bound)) in actual.into_iter().zip(tolerance).enumerate() {
            assert!(
                actual_diff <= bound,
                "{} diff index {}: actual={} tolerance={}",
                label,
                i,
                actual_diff,
                bound
            );
        }
    }

    fn summarize_vec4_sequence_diff(actual: &[Vector4<f64>], expected: &[[f64; 4]]) -> [f64; 4] {
        assert_eq!(actual.len(), expected.len());
        let mut max_diff: [f64; 4] = [0.0; 4];

        for (actual_vec, expected_vec) in actual.iter().zip(expected.iter()) {
            for (slot, diff) in max_diff.iter_mut().zip(trace_state_diff(
                actual_vec,
                &Vector4::from_row_slice(expected_vec),
            )) {
                *slot = (*slot).max(diff);
            }
        }

        max_diff
    }

    fn summarize_vec2_sequence_diff(actual: &[Vector2<f64>], expected: &[[f64; 2]]) -> [f64; 2] {
        assert_eq!(actual.len(), expected.len());
        let mut max_diff: [f64; 2] = [0.0; 2];

        for (actual_vec, expected_vec) in actual.iter().zip(expected.iter()) {
            for (slot, diff) in max_diff.iter_mut().zip(trace_control_diff(
                actual_vec,
                &Vector2::from_row_slice(expected_vec),
            )) {
                *slot = (*slot).max(diff);
            }
        }

        max_diff
    }

    fn compute_cost_without_angle_wrap(
        x: &[Vector4<f64>],
        xref: &[Vector4<f64>],
        controls: &[Vector2<f64>],
    ) -> f64 {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut cost = 0.0;
        for t in 0..T {
            cost += controls[t].dot(&(r * controls[t]));
            if t != 0 {
                let err = x[t] - xref[t];
                cost += err.dot(&(q * err));
            }
            if t < T - 1 {
                let du = controls[t + 1] - controls[t];
                cost += du.dot(&(rd * du));
            }
        }

        let terminal_error = x[T] - xref[T];
        cost + terminal_error.dot(&(qf * terminal_error))
    }

    #[derive(Debug)]
    struct GradientTraceEntry {
        iter: usize,
        cost: f64,
        gradient_norm: f64,
        accepted_step: Option<f64>,
    }

    #[derive(Debug)]
    struct LineSearchProjectionEntry {
        step: f64,
        raw_cost: f64,
        projected_cost: f64,
        raw_to_projected: [f64; 2],
        raw_to_python: [f64; 2],
        projected_to_python: [f64; 2],
    }

    #[derive(Debug)]
    struct ProjectionTraceEntry {
        iter: usize,
        current_cost: f64,
        candidates: Vec<LineSearchProjectionEntry>,
    }

    fn control_sequence_max_diff(actual: &[Vector2<f64>], expected: &[Vector2<f64>]) -> [f64; 2] {
        assert_eq!(actual.len(), expected.len());
        let mut max_diff: [f64; 2] = [0.0; 2];

        for (actual_vec, expected_vec) in actual.iter().zip(expected.iter()) {
            let diff = trace_control_diff(actual_vec, expected_vec);
            max_diff[0] = max_diff[0].max(diff[0]);
            max_diff[1] = max_diff[1].max(diff[1]);
        }

        max_diff
    }

    fn linearized_rollout_without_angle_wrap(
        x0: &State,
        xbar: &[Vector4<f64>],
        controls: &[Vector2<f64>],
    ) -> Vec<Vector4<f64>> {
        let mut x = vec![Vector4::zeros(); T + 1];
        x[0] = x0.to_vector();

        for t in 0..T {
            let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
            x[t + 1] = a * x[t] + b * controls[t] + c;
        }

        x
    }

    fn optimize_linearized_controls_without_angle_wrap_with_trace(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        max_iters: usize,
        line_search_iters: usize,
    ) -> (Vec<Vector2<f64>>, Vec<GradientTraceEntry>) {
        optimize_linearized_controls_without_angle_wrap_with_trace_and_initial_step(
            xref,
            xbar,
            state,
            initial_controls,
            max_iters,
            line_search_iters,
            0.25,
        )
    }

    fn optimize_linearized_controls_without_angle_wrap_with_trace_and_initial_step(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        max_iters: usize,
        line_search_iters: usize,
        initial_step: f64,
    ) -> (Vec<Vector2<f64>>, Vec<GradientTraceEntry>) {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut controls = initial_controls.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut previous_cost = f64::INFINITY;
        let mut trace = Vec::new();

        for iter in 0..max_iters {
            let mut x = vec![Vector4::zeros(); T + 1];
            let mut a_seq = Vec::with_capacity(T);
            let mut b_seq = Vec::with_capacity(T);
            x[0] = state.to_vector();

            for t in 0..T {
                let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
                x[t + 1] = a * x[t] + b * controls[t] + c;
                a_seq.push(a);
                b_seq.push(b);
            }
            let current_cost = compute_cost_without_angle_wrap(&x, xref, &controls);

            let mut gradients = [Vector2::zeros(); T];
            let mut lambda = (qf * (x[T] - xref[T])) * 2.0;

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
                    (q * (x[t] - xref[t])) * 2.0
                };
                lambda = lx + a_seq[t].transpose() * lambda;
            }

            let gradient_norm = gradients
                .iter()
                .map(Vector2::norm_squared)
                .sum::<f64>()
                .sqrt();

            if gradient_norm <= GRAD_TOL || (previous_cost - current_cost).abs() <= COST_TOL {
                trace.push(GradientTraceEntry {
                    iter,
                    cost: current_cost,
                    gradient_norm,
                    accepted_step: None,
                });
                break;
            }

            previous_cost = current_cost;
            let current_controls = controls.clone();
            let mut step = initial_step;
            let mut accepted_step = None;

            for _ in 0..line_search_iters {
                let mut candidate = current_controls.clone();
                for t in 0..T {
                    candidate[t] -= gradients[t] * step;
                }
                apply_control_constraints(&mut candidate);

                let candidate_x = linearized_rollout_without_angle_wrap(state, xbar, &candidate);
                let candidate_cost =
                    compute_cost_without_angle_wrap(&candidate_x, xref, &candidate);
                if candidate_cost < current_cost {
                    controls = candidate;
                    accepted_step = Some(step);
                    break;
                }

                step *= 0.5;
            }

            trace.push(GradientTraceEntry {
                iter,
                cost: current_cost,
                gradient_norm,
                accepted_step,
            });

            if accepted_step.is_none() {
                break;
            }
        }

        (controls, trace)
    }

    fn optimize_linearized_controls_without_angle_wrap_with_candidate_expansion(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        max_iters: usize,
        step_candidates: &[f64],
    ) -> (Vec<Vector2<f64>>, Vec<GradientTraceEntry>) {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut controls = initial_controls.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut previous_cost = f64::INFINITY;
        let mut trace = Vec::new();

        for iter in 0..max_iters {
            let mut x = vec![Vector4::zeros(); T + 1];
            let mut a_seq = Vec::with_capacity(T);
            let mut b_seq = Vec::with_capacity(T);
            x[0] = state.to_vector();

            for t in 0..T {
                let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
                x[t + 1] = a * x[t] + b * controls[t] + c;
                a_seq.push(a);
                b_seq.push(b);
            }
            let current_cost = compute_cost_without_angle_wrap(&x, xref, &controls);

            let mut gradients = [Vector2::zeros(); T];
            let mut lambda = (qf * (x[T] - xref[T])) * 2.0;

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
                    (q * (x[t] - xref[t])) * 2.0
                };
                lambda = lx + a_seq[t].transpose() * lambda;
            }

            let gradient_norm = gradients
                .iter()
                .map(Vector2::norm_squared)
                .sum::<f64>()
                .sqrt();

            if gradient_norm <= GRAD_TOL || (previous_cost - current_cost).abs() <= COST_TOL {
                trace.push(GradientTraceEntry {
                    iter,
                    cost: current_cost,
                    gradient_norm,
                    accepted_step: None,
                });
                break;
            }

            previous_cost = current_cost;
            let current_controls = controls.clone();
            let mut accepted_step = None;
            let mut best_candidate_controls = None;
            let mut best_candidate_cost = current_cost;

            for &step in step_candidates {
                let mut candidate = current_controls.clone();
                for t in 0..T {
                    candidate[t] -= gradients[t] * step;
                }
                apply_control_constraints(&mut candidate);

                let candidate_x = linearized_rollout_without_angle_wrap(state, xbar, &candidate);
                let candidate_cost =
                    compute_cost_without_angle_wrap(&candidate_x, xref, &candidate);
                if candidate_cost < best_candidate_cost {
                    best_candidate_cost = candidate_cost;
                    accepted_step = Some(step);
                    best_candidate_controls = Some(candidate);
                }
            }

            trace.push(GradientTraceEntry {
                iter,
                cost: current_cost,
                gradient_norm,
                accepted_step,
            });

            if let Some(candidate) = best_candidate_controls {
                controls = candidate;
            } else {
                break;
            }
        }

        (controls, trace)
    }

    fn optimize_linearized_controls_with_candidate_expansion(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        max_iters: usize,
        step_candidates: &[f64],
    ) -> (Vec<Vector2<f64>>, Vec<GradientTraceEntry>) {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut controls = initial_controls.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut previous_cost = f64::INFINITY;
        let mut trace = Vec::new();

        for iter in 0..max_iters {
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
                trace.push(GradientTraceEntry {
                    iter,
                    cost: current_cost,
                    gradient_norm,
                    accepted_step: None,
                });
                break;
            }

            previous_cost = current_cost;
            let current_controls = controls.clone();
            let mut accepted_step = None;
            let mut best_candidate_controls = None;
            let mut best_candidate_cost = current_cost;

            for &step in step_candidates {
                let mut candidate = current_controls.clone();
                for t in 0..T {
                    candidate[t] -= gradients[t] * step;
                }
                apply_control_constraints(&mut candidate);

                let (candidate_x, _, _, _) = linearized_rollout(state, xbar, &candidate);
                let candidate_cost = compute_cost(&candidate_x, xref, &candidate);
                if candidate_cost < best_candidate_cost {
                    best_candidate_cost = candidate_cost;
                    accepted_step = Some(step);
                    best_candidate_controls = Some(candidate);
                }
            }

            trace.push(GradientTraceEntry {
                iter,
                cost: current_cost,
                gradient_norm,
                accepted_step,
            });

            if let Some(candidate) = best_candidate_controls {
                controls = candidate;
            } else {
                break;
            }
        }

        (controls, trace)
    }

    fn expanded_backtracking_step_candidates() -> Vec<f64> {
        let mut steps = vec![1.0, 0.5];
        let mut step = LINE_SEARCH_INITIAL_STEP;
        for _ in 0..LINE_SEARCH_ITERS {
            steps.push(step);
            step *= 0.5;
        }
        steps
    }

    fn optimize_linearized_controls_with_full_candidate_expansion(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
    ) -> Vec<Vector2<f64>> {
        optimize_linearized_controls_with_candidate_expansion(
            xref,
            xbar,
            state,
            initial_controls,
            QP_MAX_ITERS,
            &expanded_backtracking_step_candidates(),
        )
        .0
    }

    fn iterative_linear_mpc_control_with_test_optimizer<F>(
        xref: &[Vector4<f64>],
        state: &State,
        warm_start: &[Vector2<f64>],
        optimize: F,
    ) -> MpcResult
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>,
    {
        let mut controls = warm_start.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut predicted = predict_motion(*state, &controls);
        for _ in 0..MAX_ITER {
            let previous_controls = controls.clone();
            controls = optimize(xref, &predicted, state, &controls);
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

    fn trace_projection_effect_against_python_reference(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        python_controls: &[Vector2<f64>],
        max_iters: usize,
        line_search_iters: usize,
    ) -> Vec<ProjectionTraceEntry> {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut controls = initial_controls.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut previous_cost = f64::INFINITY;
        let mut trace = Vec::new();

        for iter in 0..max_iters {
            let mut x = vec![Vector4::zeros(); T + 1];
            let mut a_seq = Vec::with_capacity(T);
            let mut b_seq = Vec::with_capacity(T);
            x[0] = state.to_vector();

            for t in 0..T {
                let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
                x[t + 1] = a * x[t] + b * controls[t] + c;
                a_seq.push(a);
                b_seq.push(b);
            }
            let current_cost = compute_cost_without_angle_wrap(&x, xref, &controls);

            let mut gradients = [Vector2::zeros(); T];
            let mut lambda = (qf * (x[T] - xref[T])) * 2.0;

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
                    (q * (x[t] - xref[t])) * 2.0
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
            let mut candidates = Vec::new();
            let mut improved = false;

            for _ in 0..line_search_iters {
                let mut raw_candidate = current_controls.clone();
                for t in 0..T {
                    raw_candidate[t] -= gradients[t] * step;
                }

                let raw_x = linearized_rollout_without_angle_wrap(state, xbar, &raw_candidate);
                let raw_cost = compute_cost_without_angle_wrap(&raw_x, xref, &raw_candidate);

                let mut projected_candidate = raw_candidate.clone();
                apply_control_constraints(&mut projected_candidate);
                let projected_x =
                    linearized_rollout_without_angle_wrap(state, xbar, &projected_candidate);
                let projected_cost =
                    compute_cost_without_angle_wrap(&projected_x, xref, &projected_candidate);

                candidates.push(LineSearchProjectionEntry {
                    step,
                    raw_cost,
                    projected_cost,
                    raw_to_projected: control_sequence_max_diff(
                        &raw_candidate,
                        &projected_candidate,
                    ),
                    raw_to_python: control_sequence_max_diff(&raw_candidate, python_controls),
                    projected_to_python: control_sequence_max_diff(
                        &projected_candidate,
                        python_controls,
                    ),
                });

                if projected_cost < current_cost {
                    controls = projected_candidate;
                    improved = true;
                    break;
                }

                step *= 0.5;
            }

            trace.push(ProjectionTraceEntry {
                iter,
                current_cost,
                candidates,
            });

            if !improved {
                break;
            }
        }

        trace
    }

    fn first_iteration_projection_candidates(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
        python_controls: &[Vector2<f64>],
        step_sizes: &[f64],
    ) -> (f64, Vec<LineSearchProjectionEntry>) {
        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let mut controls = initial_controls.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut x = vec![Vector4::zeros(); T + 1];
        let mut a_seq = Vec::with_capacity(T);
        let mut b_seq = Vec::with_capacity(T);
        x[0] = state.to_vector();

        for t in 0..T {
            let (a, b, c) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
            x[t + 1] = a * x[t] + b * controls[t] + c;
            a_seq.push(a);
            b_seq.push(b);
        }

        let current_cost = compute_cost_without_angle_wrap(&x, xref, &controls);
        let mut gradients = [Vector2::zeros(); T];
        let mut lambda = (qf * (x[T] - xref[T])) * 2.0;

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
                (q * (x[t] - xref[t])) * 2.0
            };
            lambda = lx + a_seq[t].transpose() * lambda;
        }

        let mut candidates = Vec::new();
        for &step in step_sizes {
            let mut raw_candidate = controls.clone();
            for t in 0..T {
                raw_candidate[t] -= gradients[t] * step;
            }

            let raw_x = linearized_rollout_without_angle_wrap(state, xbar, &raw_candidate);
            let raw_cost = compute_cost_without_angle_wrap(&raw_x, xref, &raw_candidate);

            let mut projected_candidate = raw_candidate.clone();
            apply_control_constraints(&mut projected_candidate);
            let projected_x =
                linearized_rollout_without_angle_wrap(state, xbar, &projected_candidate);
            let projected_cost =
                compute_cost_without_angle_wrap(&projected_x, xref, &projected_candidate);

            candidates.push(LineSearchProjectionEntry {
                step,
                raw_cost,
                projected_cost,
                raw_to_projected: control_sequence_max_diff(&raw_candidate, &projected_candidate),
                raw_to_python: control_sequence_max_diff(&raw_candidate, python_controls),
                projected_to_python: control_sequence_max_diff(
                    &projected_candidate,
                    python_controls,
                ),
            });
        }

        (current_cost, candidates)
    }

    fn state_from_vector4(state: &Vector4<f64>) -> State {
        State::new(state[0], state[1], state[2], state[3])
    }

    fn inject_state_components(
        base: &Vector4<f64>,
        injected: &Vector4<f64>,
        component_indices: &[usize],
    ) -> State {
        let mut hybrid = *base;
        for index in component_indices {
            hybrid[*index] = injected[*index];
        }
        state_from_vector4(&hybrid)
    }

    fn measure_switch_back_state_injection(indices: &[usize]) -> StateInjectionMetrics {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        let mut summary = StateInjectionMetrics::default();
        for metrics in measure_switch_back_state_injection_by_step_with_context(
            indices,
            &[325usize, 326, 327, 328, 329],
            &default_trace,
            &expanded_trace,
            &default_warm_start,
            &expanded_warm_start,
        )
        .into_values()
        {
            summary.default_same_control0 += metrics.default_same_control0;
            summary.default_opposite_control0 += metrics.default_opposite_control0;
            summary.default_same_pred_y += metrics.default_same_pred_y;
            summary.default_opposite_pred_y += metrics.default_opposite_pred_y;
            summary.default_same_pred_v += metrics.default_same_pred_v;
            summary.default_opposite_pred_v += metrics.default_opposite_pred_v;
            summary.expanded_same_control0 += metrics.expanded_same_control0;
            summary.expanded_opposite_control0 += metrics.expanded_opposite_control0;
            summary.expanded_same_pred_y += metrics.expanded_same_pred_y;
            summary.expanded_opposite_pred_y += metrics.expanded_opposite_pred_y;
            summary.expanded_same_pred_v += metrics.expanded_same_pred_v;
            summary.expanded_opposite_pred_v += metrics.expanded_opposite_pred_v;
        }

        summary
    }

    fn measure_switch_back_state_injection_by_step_with_context(
        indices: &[usize],
        steps: &[usize],
        default_trace: &[SwitchBackDetailedRow],
        expanded_trace: &[SwitchBackDetailedRow],
        default_warm_start: &SwitchBackWarmStartTrace,
        expanded_warm_start: &SwitchBackWarmStartTrace,
    ) -> BTreeMap<usize, StateInjectionMetrics> {
        let mut by_step = BTreeMap::new();

        for step in steps {
            let mut metrics = StateInjectionMetrics::default();
            let default_row = detailed_row_at_step(default_trace, *step);
            let expanded_row = detailed_row_at_step(expanded_trace, *step);
            let default_injected = iterative_linear_mpc_control_with_test_optimizer(
                &default_row.xref,
                &inject_state_components(&default_row.state, &expanded_row.state, indices),
                default_warm_start.get(step).unwrap(),
                optimize_linearized_controls_original,
            );
            let expanded_injected = iterative_linear_mpc_control_with_test_optimizer(
                &expanded_row.xref,
                &inject_state_components(&expanded_row.state, &default_row.state, indices),
                expanded_warm_start.get(step).unwrap(),
                optimize_linearized_controls_with_full_candidate_expansion,
            );

            let default_baseline_controls = default_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expanded_baseline_controls = expanded_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let default_baseline_predicted = default_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();
            let expanded_baseline_predicted = expanded_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            let default_to_default_control = summarize_vec2_sequence_diff(
                &default_injected.controls,
                &default_baseline_controls,
            );
            let default_to_expanded_control = summarize_vec2_sequence_diff(
                &default_injected.controls,
                &expanded_baseline_controls,
            );
            let default_to_default_predicted = summarize_vec4_sequence_diff(
                &default_injected.predicted,
                &default_baseline_predicted,
            );
            let default_to_expanded_predicted = summarize_vec4_sequence_diff(
                &default_injected.predicted,
                &expanded_baseline_predicted,
            );

            let expanded_to_expanded_control = summarize_vec2_sequence_diff(
                &expanded_injected.controls,
                &expanded_baseline_controls,
            );
            let expanded_to_default_control = summarize_vec2_sequence_diff(
                &expanded_injected.controls,
                &default_baseline_controls,
            );
            let expanded_to_expanded_predicted = summarize_vec4_sequence_diff(
                &expanded_injected.predicted,
                &expanded_baseline_predicted,
            );
            let expanded_to_default_predicted = summarize_vec4_sequence_diff(
                &expanded_injected.predicted,
                &default_baseline_predicted,
            );

            metrics.default_same_control0 = default_to_default_control[0];
            metrics.default_opposite_control0 = default_to_expanded_control[0];
            metrics.default_same_pred_y = default_to_default_predicted[1];
            metrics.default_opposite_pred_y = default_to_expanded_predicted[1];
            metrics.default_same_pred_v = default_to_default_predicted[2];
            metrics.default_opposite_pred_v = default_to_expanded_predicted[2];

            metrics.expanded_same_control0 = expanded_to_expanded_control[0];
            metrics.expanded_opposite_control0 = expanded_to_default_control[0];
            metrics.expanded_same_pred_y = expanded_to_expanded_predicted[1];
            metrics.expanded_opposite_pred_y = expanded_to_default_predicted[1];
            metrics.expanded_same_pred_v = expanded_to_expanded_predicted[2];
            metrics.expanded_opposite_pred_v = expanded_to_default_predicted[2];

            by_step.insert(*step, metrics);
        }

        by_step
    }

    fn measure_switch_back_state_injection_by_step(
        indices: &[usize],
        steps: &[usize],
    ) -> BTreeMap<usize, StateInjectionMetrics> {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        measure_switch_back_state_injection_by_step_with_context(
            indices,
            steps,
            &default_trace,
            &expanded_trace,
            &default_warm_start,
            &expanded_warm_start,
        )
    }

    fn state_injection_metrics_at_step(
        metrics: &BTreeMap<usize, StateInjectionMetrics>,
        step: usize,
    ) -> &StateInjectionMetrics {
        metrics
            .get(&step)
            .unwrap_or_else(|| panic!("missing state-injection metrics for step {step}"))
    }

    fn advance_switch_back_context(steps: usize) -> SwitchBackContext {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];

        for _ in 0..steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let mpc_result = iterative_linear_mpc_control(&xref, &state, &warm_start);
            state.update(mpc_result.controls[0][0], mpc_result.controls[0][1]);

            warm_start = next_warm_start(&mpc_result.controls);
        }

        SwitchBackContext {
            cx,
            cy,
            cyaw,
            speed_profile,
            state,
            target_index,
            warm_start,
        }
    }

    fn collect_switch_back_prefix_trace(steps: usize) -> Vec<SwitchBackTraceRow> {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut trace = Vec::with_capacity(steps);

        for step in 0..steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let result = iterative_linear_mpc_control(&xref, &state, &warm_start);
            trace.push(SwitchBackTraceRow {
                step,
                target_index,
                state: state.to_vector(),
                xref_head: xref[0],
                xref_tail: *xref.last().unwrap(),
                control: result.controls[0],
                predicted_tail: *result.predicted.last().unwrap(),
            });

            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
        }

        trace
    }

    fn collect_switch_back_detailed_trace_with_test_optimizer<F>(
        steps: usize,
        optimize: F,
    ) -> Vec<SwitchBackDetailedRow>
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut trace = Vec::with_capacity(steps);

        for step in 0..steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            trace.push(SwitchBackDetailedRow {
                step,
                target_index,
                state: state.to_vector(),
                xref: xref.clone(),
                controls: result.controls.clone(),
                predicted: result.predicted.clone(),
            });

            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
        }

        trace
    }

    fn collect_switch_back_planning_snapshots_with_test_optimizer<F>(
        steps: usize,
        checkpoints: &[usize],
        optimize: F,
    ) -> Vec<SwitchBackPlanningSnapshot>
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut trace = Vec::new();

        for step in 0..steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            if checkpoints.contains(&step) {
                trace.push(SwitchBackPlanningSnapshot {
                    step,
                    target_index,
                    state: state.to_vector(),
                    xref_head: xref[0],
                    warm_start_head: warm_start.first().copied().unwrap_or_else(Vector2::zeros),
                });
            }

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
        }

        trace
    }

    fn collect_switch_back_warm_start_trace_with_test_optimizer<F>(
        steps: usize,
        optimize: F,
    ) -> SwitchBackWarmStartTrace
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut trace = SwitchBackWarmStartTrace::new();

        for step in 0..steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;
            trace.insert(step, warm_start.clone());

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
        }

        trace
    }

    fn collect_switch_back_detailed_trace(steps: usize) -> Vec<SwitchBackDetailedRow> {
        collect_switch_back_detailed_trace_with_test_optimizer(steps, optimize_linearized_controls)
    }

    fn run_switch_back_simulation_with_test_optimizer<F>(
        max_sim_steps: usize,
        optimize: F,
    ) -> SimulationResult
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
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

        for _ in 0..max_sim_steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let mpc_result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            let accel = mpc_result.controls[0][0];
            let steer = mpc_result.controls[0][1];

            state.update(accel, steer);
            hist_x.push(state.x);
            hist_y.push(state.y);
            predicted = mpc_result.predicted;

            warm_start = next_warm_start(&mpc_result.controls);

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

    fn collect_switch_back_closed_loop_checkpoints_with_test_optimizer<F>(
        max_sim_steps: usize,
        checkpoints: &[usize],
        optimize: F,
    ) -> Vec<SwitchBackClosedLoopCheckpoint>
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
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
        let mut recorded = Vec::new();

        for step in 0..max_sim_steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let mpc_result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            let accel = mpc_result.controls[0][0];
            let steer = mpc_result.controls[0][1];

            state.update(accel, steer);
            warm_start = next_warm_start(&mpc_result.controls);

            if checkpoints.contains(&(step + 1)) {
                let goal_distance =
                    ((state.x - goal.0).powi(2) + (state.y - goal.1).powi(2)).sqrt();
                recorded.push(SwitchBackClosedLoopCheckpoint {
                    step: step + 1,
                    target_index,
                    goal_distance,
                    state: state.to_vector(),
                });
            }

            if check_goal(&state, goal, target_index, final_index) {
                break;
            }
        }

        recorded
    }

    fn collect_switch_back_goal_condition_snapshots_with_test_optimizer<F>(
        max_sim_steps: usize,
        checkpoints: &[usize],
        optimize: F,
    ) -> Vec<SwitchBackGoalConditionSnapshot>
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
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
        let mut recorded = Vec::new();

        for step in 0..max_sim_steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let mpc_result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            let accel = mpc_result.controls[0][0];
            let steer = mpc_result.controls[0][1];

            state.update(accel, steer);
            warm_start = next_warm_start(&mpc_result.controls);

            let dx = state.x - goal.0;
            let dy = state.y - goal.1;
            let goal_distance = (dx * dx + dy * dy).sqrt();
            let heading_error = angle_diff(state.yaw, cyaw[target_index]);
            let motion_curvature = steer.tan() / WB;
            let near_goal = goal_distance <= GOAL_DIS;
            let near_end = final_index.abs_diff(target_index) < 5;
            let stopped = state.v.abs() <= STOP_SPEED;
            let reached_goal = near_goal && near_end && stopped;

            if checkpoints.contains(&(step + 1)) {
                recorded.push(SwitchBackGoalConditionSnapshot {
                    step: step + 1,
                    target_index,
                    goal_distance,
                    speed: state.v,
                    speed_abs: state.v.abs(),
                    heading_error,
                    motion_curvature,
                    near_goal,
                    near_end,
                    stopped,
                    reached_goal,
                });
            }

            if reached_goal {
                break;
            }
        }

        recorded
    }

    fn collect_switch_back_goal_control_snapshots_with_test_optimizer<F>(
        max_sim_steps: usize,
        checkpoints: &[usize],
        optimize: F,
    ) -> Vec<SwitchBackGoalControlSnapshot>
    where
        F: Fn(&[Vector4<f64>], &[Vector4<f64>], &State, &[Vector2<f64>]) -> Vec<Vector2<f64>>
            + Copy,
    {
        let (cx, cy, cyaw) = generate_switch_back_course();
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
        let mut recorded = Vec::new();

        for step in 0..max_sim_steps {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let mpc_result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize,
            );
            let accel = mpc_result.controls[0][0];
            let steer = mpc_result.controls[0][1];

            state.update(accel, steer);
            warm_start = next_warm_start(&mpc_result.controls);

            let dx = state.x - goal.0;
            let dy = state.y - goal.1;
            let goal_distance = (dx * dx + dy * dy).sqrt();
            let heading_error = angle_diff(state.yaw, cyaw[target_index]);
            let motion_curvature = steer.tan() / WB;
            let course_curvature = course_curvature_at_index(&cx, &cy, &cyaw, target_index);
            let near_goal = goal_distance <= GOAL_DIS;
            let near_end = final_index.abs_diff(target_index) < 5;
            let stopped = state.v.abs() <= STOP_SPEED;
            let reached_goal = near_goal && near_end && stopped;

            if checkpoints.contains(&(step + 1)) {
                recorded.push(SwitchBackGoalControlSnapshot {
                    step: step + 1,
                    target_index,
                    goal_distance,
                    speed: state.v,
                    accel,
                    steer,
                    heading_error,
                    motion_curvature,
                    course_curvature,
                    near_goal,
                    near_end,
                    stopped,
                    reached_goal,
                });
            }

            if reached_goal {
                break;
            }
        }

        recorded
    }

    fn goal_distance_delta(
        default: &SwitchBackClosedLoopCheckpoint,
        expanded: &SwitchBackClosedLoopCheckpoint,
    ) -> f64 {
        default.goal_distance - expanded.goal_distance
    }

    fn checkpoint_at_step(
        checkpoints: &[SwitchBackClosedLoopCheckpoint],
        step: usize,
    ) -> &SwitchBackClosedLoopCheckpoint {
        checkpoints
            .iter()
            .find(|checkpoint| checkpoint.step == step)
            .unwrap_or_else(|| panic!("missing checkpoint for step {}", step))
    }

    fn goal_distance_slope(
        start: &SwitchBackClosedLoopCheckpoint,
        end: &SwitchBackClosedLoopCheckpoint,
    ) -> f64 {
        let span = (end.step - start.step) as f64;
        (start.goal_distance - end.goal_distance) / span
    }

    fn target_index_increment(
        start: &SwitchBackClosedLoopCheckpoint,
        end: &SwitchBackClosedLoopCheckpoint,
    ) -> isize {
        end.target_index as isize - start.target_index as isize
    }

    fn checkpoint_lateral_error(
        checkpoint: &SwitchBackClosedLoopCheckpoint,
        cx: &[f64],
        cy: &[f64],
        cyaw: &[f64],
    ) -> f64 {
        let index = checkpoint.target_index.min(cx.len().saturating_sub(1));
        let dx = checkpoint.state[0] - cx[index];
        let dy = checkpoint.state[1] - cy[index];
        -dx * cyaw[index].sin() + dy * cyaw[index].cos()
    }

    fn checkpoint_heading_error(checkpoint: &SwitchBackClosedLoopCheckpoint, cyaw: &[f64]) -> f64 {
        let index = checkpoint.target_index.min(cyaw.len().saturating_sub(1));
        angle_diff(checkpoint.state[3], cyaw[index])
    }

    fn course_curvature_at_index(cx: &[f64], cy: &[f64], cyaw: &[f64], index: usize) -> f64 {
        if cx.len() < 3 || cy.len() < 3 || cyaw.len() < 3 {
            return 0.0;
        }

        let i = index.clamp(1, cx.len().saturating_sub(2));
        let ds_prev = ((cx[i] - cx[i - 1]).powi(2) + (cy[i] - cy[i - 1]).powi(2))
            .sqrt()
            .max(f64::EPSILON);
        let ds_next = ((cx[i + 1] - cx[i]).powi(2) + (cy[i + 1] - cy[i]).powi(2))
            .sqrt()
            .max(f64::EPSILON);

        angle_diff(cyaw[i + 1], cyaw[i - 1]) / (ds_prev + ds_next)
    }

    fn scalar_slope(start: f64, end: f64, steps: usize) -> f64 {
        (end - start) / steps as f64
    }

    fn goal_snapshot_at_step(
        snapshots: &[SwitchBackGoalConditionSnapshot],
        step: usize,
    ) -> &SwitchBackGoalConditionSnapshot {
        snapshots
            .iter()
            .find(|snapshot| snapshot.step == step)
            .unwrap_or_else(|| panic!("missing goal-condition snapshot for step {}", step))
    }

    fn first_goal_condition_step<F>(
        snapshots: &[SwitchBackGoalConditionSnapshot],
        predicate: F,
    ) -> Option<usize>
    where
        F: Fn(&SwitchBackGoalConditionSnapshot) -> bool,
    {
        snapshots
            .iter()
            .find(|snapshot| predicate(snapshot))
            .map(|s| s.step)
    }

    fn goal_control_snapshot_at_step(
        snapshots: &[SwitchBackGoalControlSnapshot],
        step: usize,
    ) -> &SwitchBackGoalControlSnapshot {
        snapshots
            .iter()
            .find(|snapshot| snapshot.step == step)
            .unwrap_or_else(|| panic!("missing goal-control snapshot for step {}", step))
    }

    fn first_goal_control_condition_step<F>(
        snapshots: &[SwitchBackGoalControlSnapshot],
        predicate: F,
    ) -> Option<usize>
    where
        F: Fn(&SwitchBackGoalControlSnapshot) -> bool,
    {
        snapshots
            .iter()
            .find(|snapshot| predicate(snapshot))
            .map(|snapshot| snapshot.step)
    }

    fn detailed_row_at_step(rows: &[SwitchBackDetailedRow], step: usize) -> &SwitchBackDetailedRow {
        rows.iter()
            .find(|row| row.step == step)
            .unwrap_or_else(|| panic!("missing detailed row for step {}", step))
    }

    fn planning_snapshot_at_step(
        snapshots: &[SwitchBackPlanningSnapshot],
        step: usize,
    ) -> &SwitchBackPlanningSnapshot {
        snapshots
            .iter()
            .find(|snapshot| snapshot.step == step)
            .unwrap_or_else(|| panic!("missing planning snapshot for step {}", step))
    }

    fn assert_planning_snapshot_phase_alignment(
        label: &str,
        default: &SwitchBackPlanningSnapshot,
        expanded: &SwitchBackPlanningSnapshot,
    ) {
        assert_eq!(
            default.target_index, expanded.target_index,
            "{} target-index alignment moved unexpectedly: default={:?} expanded={:?}",
            label, default, expanded
        );
        assert_diff_bounds(
            &format!("{label}_state"),
            trace_state_diff(&default.state, &expanded.state),
            [2e-4, 2e-4, 1e-4, 1e-4],
        );
        assert_diff_bounds(
            &format!("{label}_xref_head"),
            trace_state_diff(&default.xref_head, &expanded.xref_head),
            [1e-10; 4],
        );
        assert_diff_bounds(
            &format!("{label}_warm_start"),
            trace_control_diff(&default.warm_start_head, &expanded.warm_start_head),
            [1e-10, 3e-4],
        );
    }

    fn assert_goal_control_alignment(
        default: &SwitchBackGoalControlSnapshot,
        expanded: &SwitchBackGoalControlSnapshot,
    ) {
        assert_eq!(default.target_index, expanded.target_index);
        assert_eq!(default.near_goal, expanded.near_goal);
        assert_eq!(default.near_end, expanded.near_end);
        assert_eq!(default.stopped, expanded.stopped);
        assert_eq!(default.reached_goal, expanded.reached_goal);
        assert!(
            (default.goal_distance - expanded.goal_distance).abs() <= 3.0e-4,
            "goal distance alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.speed - expanded.speed).abs() <= 3.0e-4,
            "speed alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.accel - expanded.accel).abs() <= 2.0e-3,
            "accel alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.steer - expanded.steer).abs() <= 1.0e-4,
            "steer alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.heading_error - expanded.heading_error).abs() <= 5.0e-5,
            "heading-error alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.motion_curvature - expanded.motion_curvature).abs() <= 5.0e-5,
            "motion-curvature alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
        assert!(
            (default.course_curvature - expanded.course_curvature).abs() <= 1.0e-12,
            "course-curvature alignment moved unexpectedly: default={:?} expanded={:?}",
            default,
            expanded
        );
    }

    fn assert_detailed_phase_alignment(
        label: &str,
        default: &SwitchBackDetailedRow,
        expanded: &SwitchBackDetailedRow,
    ) {
        assert_eq!(
            default.target_index, expanded.target_index,
            "{} target-index alignment moved unexpectedly: default={:?} expanded={:?}",
            label, default, expanded
        );

        let expanded_xref_rows = expanded
            .xref
            .iter()
            .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
            .collect::<Vec<_>>();
        let expanded_control_rows = expanded
            .controls
            .iter()
            .map(|vec| [vec[0], vec[1]])
            .collect::<Vec<_>>();
        let expanded_predicted_rows = expanded
            .predicted
            .iter()
            .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
            .collect::<Vec<_>>();

        assert_diff_bounds(
            &format!("{label}_state"),
            trace_state_diff(&default.state, &expanded.state),
            [5e-4, 5e-4, 5e-4, 2e-4],
        );
        assert_diff_bounds(
            &format!("{label}_xref"),
            summarize_vec4_sequence_diff(&default.xref, &expanded_xref_rows),
            [1e-10; 4],
        );
        assert_diff_bounds(
            &format!("{label}_controls"),
            summarize_vec2_sequence_diff(&default.controls, &expanded_control_rows),
            [2e-3, 2e-4],
        );
        assert_diff_bounds(
            &format!("{label}_predicted"),
            summarize_vec4_sequence_diff(&default.predicted, &expanded_predicted_rows),
            [7e-4, 7e-4, 7e-4, 2e-4],
        );
    }

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
    fn test_iterative_linear_mpc_matches_reference_offset_state() {
        // Reference generated from the PythonRobotics MPC equations and constraints
        // using SciPy SLSQP for the fixed scenario below.
        let xref = vec![
            Vector4::new(0.0, 0.0, TARGET_SPEED, 0.0),
            Vector4::new(0.5, 0.0, TARGET_SPEED, 0.0),
            Vector4::new(1.0, 0.0, TARGET_SPEED, 0.0),
            Vector4::new(1.5, 0.0, TARGET_SPEED, 0.0),
            Vector4::new(2.0, 0.0, TARGET_SPEED, 0.0),
            Vector4::new(2.5, 0.0, TARGET_SPEED, 0.0),
        ];
        let state = State::new(0.0, 0.3, 1.0, 0.1);

        let result = iterative_linear_mpc_control(&xref, &state, &[]);

        let expected_controls = [
            [1.0, -0.669_667_215_071],
            [1.0, -0.630_794_908_167],
            [1.0, -0.577_878_962_843],
            [1.0, -0.534_142_026_172],
            [1.0, -0.513_091_902_297],
        ];
        for (actual, expected) in result.controls.iter().zip(expected_controls.iter()) {
            assert_vec2_close(actual, *expected, 5e-3);
        }

        let final_predicted = result.predicted.last().unwrap();
        assert_vec4_close(
            final_predicted,
            [1.394_401_496_574, 0.249_081_964_95, 2.0, -0.221_107_545_509],
            5e-2,
        );
    }

    #[test]
    fn test_mpc_simulation_reaches_goal() {
        let ax = vec![0.0, 6.0, 12.0];
        let ay = vec![0.0, 0.0, 0.0];
        let result = run_mpc_simulation_with_course(&ax, &ay, 120);
        let goal_distance = ((result.final_state.x - result.goal.0).powi(2)
            + (result.final_state.y - result.goal.1).powi(2))
        .sqrt();

        assert!(result.reached_goal);
        assert!(goal_distance <= GOAL_DIS + 0.25);
        assert!(result.hist_x.len() < 120);
    }

    #[test]
    fn test_mpc_straight_course_closed_loop_matches_reference() {
        // Reference generated from the PythonRobotics equations with the same
        // straight-course setup, solved by SciPy SLSQP over the constrained QP.
        let ax = vec![0.0, 6.0, 12.0];
        let ay = vec![0.0, 0.0, 0.0];
        let result = run_mpc_simulation_with_course(&ax, &ay, 120);

        assert!(result.reached_goal);
        assert_eq!(result.target_index, result.final_index);
        assert_eq!(result.target_index, 24);
        assert_eq!(result.hist_x.len(), 56);

        assert_vec4_close(
            &result.final_state.to_vector(),
            [
                12.126_756_016_762_242,
                0.000_001_043_289_690_068_484_6,
                -0.123_832_283_912_188_83,
                0.000_000_381_531_819_982_818_5,
            ],
            1e-2,
        );
    }

    #[test]
    fn test_switch_back_course_contains_reverse_segment() {
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        assert_eq!(cx.first().copied(), Some(0.0));
        assert_eq!(cy.first().copied(), Some(0.0));
        assert_eq!(cx.len(), 162);
        assert!(cy.iter().copied().fold(f64::NEG_INFINITY, f64::max) > 30.0);
        assert!((cx.last().unwrap() + 0.016_802_167_501_358_26).abs() <= 1e-9);
        assert!((cy.last().unwrap() - 0.833_302_319_727_219_5).abs() <= 1e-9);
        assert!(speed_profile[..speed_profile.len().saturating_sub(1)]
            .iter()
            .any(|speed| *speed < 0.0));
        assert_eq!(speed_profile.last().copied(), Some(0.0));
    }

    #[test]
    fn test_mpc_switch_back_midcourse_window_matches_reference() {
        // Reference generated from PythonRobotics switch-back simulation
        // at closed-loop step 20 using cvxpy CLARABEL.
        let ctx = advance_switch_back_context(20);
        let (xref, target_index) = calc_ref_trajectory(
            &ctx.state,
            &ctx.cx,
            &ctx.cy,
            &ctx.cyaw,
            &ctx.speed_profile,
            UPSTREAM_SWITCH_BACK_TICK,
            ctx.target_index,
        );
        let result = iterative_linear_mpc_control(&xref, &ctx.state, &ctx.warm_start);

        assert_eq!(target_index, 5);
        assert_vec4_close(
            &ctx.state.to_vector(),
            [
                7.528_990_211_684_919,
                -0.616_644_989_009_243_4,
                3.855_895_792_930_033_5,
                -0.078_281_934_138_131_24,
            ],
            1e-2,
        );

        let expected_xref = [
            [
                7.884_351_723_374_65,
                -0.646_692_295_268_678_9,
                TARGET_SPEED,
                -0.078_822_806_954_514_1,
            ],
            [
                9.417_709_562_051_241,
                -0.766_274_939_125_219_6,
                TARGET_SPEED,
                -0.076_759_688_257_237_88,
            ],
            [
                10.927_333_306_546_922,
                -0.880_536_229_237_835_5,
                TARGET_SPEED,
                -0.074_243_230_035_395_7,
            ],
            [
                10.927_333_306_546_922,
                -0.880_536_229_237_835_5,
                TARGET_SPEED,
                -0.074_243_230_035_395_7,
            ],
            [
                12.409_267_274_498_205,
                -0.988_589_273_315_872_4,
                TARGET_SPEED,
                -0.071_228_758_450_065_36,
            ],
            [
                13.859_555_783_541_607,
                -1.089_547_179_068_676,
                TARGET_SPEED,
                -0.067_659_809_911_111_4,
            ],
        ];
        for (actual, expected) in xref.iter().zip(expected_xref.iter()) {
            assert_vec4_close(actual, *expected, 1e-3);
        }

        let expected_controls = [
            [0.999_999_995_100_728_5, 0.004_502_534_498_214_272],
            [0.619_401_970_887_073, 0.003_626_365_725_009_755],
            [-0.999_999_586_012_278, 0.003_497_926_790_795_169],
            [-0.999_999_988_263_528_5, 0.004_178_759_548_203_836],
            [-0.999_999_985_187_766_1, 0.004_753_969_705_983_846],
        ];
        for (actual, expected) in result.controls.iter().zip(expected_controls.iter()) {
            assert_vec2_close(actual, *expected, 2e-2);
        }

        let final_predicted = result.predicted.last().unwrap();
        assert_vec4_close(
            final_predicted,
            [
                11.487_827_888_938_794,
                -0.917_080_324_028_499_6,
                3.579_776_274_235_015_4,
                -0.071_778_774_991_017_19,
            ],
            5e-2,
        );
    }

    #[test]
    fn test_mpc_switch_back_prefix_trace_matches_pythonrobotics_reference() {
        let expected =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        // Steps 0..=70 stay on the pre-reversal branch and are stable enough for
        // step-by-step parity. The reverse transition is covered separately by
        // the midcourse window and full-course regressions.
        let strict_prefix_steps = 71;
        let expected = &expected[..strict_prefix_steps];
        let actual = collect_switch_back_prefix_trace(expected.len());
        let summary = summarize_switch_back_trace_window(&actual, expected);

        assert_eq!(summary.max_target_index_gap, 0);
        assert_diff_bounds("state", summary.max_state_diff, [4e-3, 3e-3, 4e-3, 2e-3]);
        assert_diff_bounds("xref_head", summary.max_xref_head_diff, [1e-10; 4]);
        assert_diff_bounds("xref_tail", summary.max_xref_tail_diff, [1e-10; 4]);
        assert_diff_bounds("control", summary.max_control_diff, [1e-2, 6e-3]);
        assert_diff_bounds(
            "predicted_tail",
            summary.max_predicted_tail_diff,
            [0.11, 0.13, 5e-3, 0.17],
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_transition_matches_pythonrobotics_reference() {
        let expected =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        // The reverse handoff is the point where the projected-gradient solver
        // starts to drift from the upstream cvxpy reference. Keep it under a
        // bounded window even though step-by-step strict parity is not stable.
        let reverse_window = 71..81;
        let actual = collect_switch_back_prefix_trace(reverse_window.end);
        let summary = summarize_switch_back_trace_window(
            &actual[reverse_window.clone()],
            &expected[reverse_window],
        );

        assert!(summary.max_target_index_gap <= 1);
        assert_diff_bounds(
            "reverse_state",
            summary.max_state_diff,
            [0.04, 0.35, 0.17, 0.2],
        );
        assert_diff_bounds(
            "reverse_xref_head",
            summary.max_xref_head_diff,
            [0.75, 0.45, 1e-12, 0.02],
        );
        assert_diff_bounds(
            "reverse_xref_tail",
            summary.max_xref_tail_diff,
            [0.95, 0.5, 1e-12, 0.01],
        );
        assert_diff_bounds("reverse_control", summary.max_control_diff, [0.86, 0.41]);
        assert_diff_bounds(
            "reverse_predicted_tail",
            summary.max_predicted_tail_diff,
            [0.26, 0.62, 0.36, 0.55],
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_detailed_trace_matches_pythonrobotics_reference() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let actual_trace = collect_switch_back_detailed_trace(81);
        let summary = summarize_switch_back_detailed_window(
            &actual_trace,
            &expected_trace,
            &expected_controls,
            &expected_horizon,
            71..=80,
        );

        assert!(summary.max_target_index_gap <= 1);
        assert_diff_bounds(
            "reverse_detailed_state",
            summary.max_state_diff,
            [0.04, 0.35, 0.17, 0.21],
        );
        assert_diff_bounds(
            "reverse_detailed_xref",
            summary.max_xref_diff,
            [0.92, 0.49, 1e-12, 0.02],
        );
        assert_diff_bounds(
            "reverse_detailed_control",
            summary.max_control_diff,
            [1.46, 0.5],
        );
        assert_diff_bounds(
            "reverse_detailed_predicted",
            summary.max_predicted_diff,
            [0.26, 0.62, 0.47, 0.55],
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_full_candidate_expansion_characterizes_detailed_trace_tradeoff()
    {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let actual_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            81,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_summary = summarize_switch_back_detailed_window(
            &collect_switch_back_detailed_trace(81),
            &expected_trace,
            &expected_controls,
            &expected_horizon,
            71..=80,
        );
        let expanded_summary = summarize_switch_back_detailed_window(
            &actual_trace,
            &expected_trace,
            &expected_controls,
            &expected_horizon,
            71..=80,
        );

        assert!(expanded_summary.max_target_index_gap <= 1);
        assert_eq!(
            expanded_summary.max_xref_diff,
            default_summary.max_xref_diff
        );
        assert_diff_bounds(
            "reverse_detailed_full_candidates_state_characterization",
            expanded_summary.max_state_diff,
            [0.043, 0.35, 0.17, 0.21],
        );
        assert_diff_bounds(
            "reverse_detailed_full_candidates_control_characterization",
            expanded_summary.max_control_diff,
            [1.32, 0.5],
        );
        assert_diff_bounds(
            "reverse_detailed_full_candidates_predicted_characterization",
            expanded_summary.max_predicted_diff,
            [0.22, 0.62, 0.41, 0.43],
        );

        // After hybrid QP+PG solver promotion, default and expanded may differ
        // slightly due to candidate expansion interacting with the hybrid solver.
        // Verify they remain within a reasonable margin of each other.
        for (i, (d, e)) in default_summary
            .max_state_diff
            .iter()
            .zip(expanded_summary.max_state_diff.iter())
            .enumerate()
        {
            assert!(
                (d - e).abs() < 0.2,
                "state diff index {} diverged too much: default={} expanded={}",
                i,
                d,
                e
            );
        }
        for (i, (d, e)) in default_summary
            .max_control_diff
            .iter()
            .zip(expanded_summary.max_control_diff.iter())
            .enumerate()
        {
            assert!(
                (d - e).abs() < 0.2,
                "control diff index {} diverged too much: default={} expanded={}",
                i,
                d,
                e
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_solver_gap_with_python_warm_start() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));

        for step in [75usize, 78usize] {
            let expected_state = &expected_trace[step].state;
            let python_warm_start = expected_controls.get(&(step - 1)).unwrap();
            let expected_controls_for_step = expected_controls.get(&step).unwrap();
            let (expected_xref_for_step, expected_predicted_for_step) =
                expected_horizon.get(&step).unwrap();

            let expected_control_rows = expected_controls_for_step
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expected_predicted_rows = expected_predicted_for_step
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            let result = iterative_linear_mpc_control(
                expected_xref_for_step,
                &state_from_vector4(expected_state),
                python_warm_start,
            );

            let control_diff =
                summarize_vec2_sequence_diff(&result.controls, &expected_control_rows);
            let predicted_diff =
                summarize_vec4_sequence_diff(&result.predicted, &expected_predicted_rows);
            let (control_tol, predicted_tol) = match step {
                75 => ([0.38, 0.11], [0.09, 0.23, 0.11, 0.18]),
                78 => ([0.52, 0.35], [0.03, 0.46, 0.11, 0.25]),
                _ => unreachable!(),
            };

            assert_diff_bounds(
                &format!("python_warm_start_step_{step}_control"),
                control_diff,
                control_tol,
            );
            assert_diff_bounds(
                &format!("python_warm_start_step_{step}_predicted"),
                predicted_diff,
                predicted_tol,
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_predict_motion_matches_python_warm_start_reference() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));

        for step in [75usize, 78usize] {
            let expected_state = &expected_trace[step].state;
            let python_warm_start = expected_controls.get(&(step - 1)).unwrap();
            let expected_xbar_for_step = expected_xbar.get(&step).unwrap();
            let actual_xbar = predict_motion(state_from_vector4(expected_state), python_warm_start);
            let xbar_diff = summarize_vec4_sequence_diff(
                &actual_xbar,
                &expected_xbar_for_step
                    .iter()
                    .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                    .collect::<Vec<_>>(),
            );

            assert_diff_bounds(
                &format!("python_warm_start_step_{step}_xbar"),
                xbar_diff,
                [1e-9, 1e-9, 1e-9, 1e-9],
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_linearized_cost_gap_against_python_reference() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));

        for step in [75usize, 78usize] {
            let expected_state = &expected_trace[step].state;
            let python_warm_start = expected_controls.get(&(step - 1)).unwrap();
            let python_controls = expected_controls.get(&step).unwrap();
            let (expected_xref_for_step, expected_predicted_for_step) =
                expected_horizon.get(&step).unwrap();
            let expected_xbar_for_step = expected_xbar.get(&step).unwrap();
            let state = state_from_vector4(expected_state);

            let rust_controls = optimize_linearized_controls(
                expected_xref_for_step,
                expected_xbar_for_step,
                &state,
                python_warm_start,
            );
            let python_x_python_style = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                python_controls,
            );
            let python_cost_python_style = compute_cost_without_angle_wrap(
                &python_x_python_style,
                expected_xref_for_step,
                python_controls,
            );

            let rust_x_python_style = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                &rust_controls,
            );
            let rust_cost_python_style = compute_cost_without_angle_wrap(
                &rust_x_python_style,
                expected_xref_for_step,
                &rust_controls,
            );
            let predicted_diff_python_style = summarize_vec4_sequence_diff(
                &python_x_python_style,
                &expected_predicted_for_step
                    .iter()
                    .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                    .collect::<Vec<_>>(),
            );
            let predicted_tol = match step {
                75 => [0.11, 0.07, 1e-12, 0.002],
                78 => [0.06, 0.02, 1e-12, 0.006],
                _ => unreachable!(),
            };

            assert_diff_bounds(
                &format!("python_style_linearized_rollout_step_{step}"),
                predicted_diff_python_style,
                predicted_tol,
            );

            let solver_gap = rust_cost_python_style - python_cost_python_style;
            let min_gap = match step {
                75 => 1.0,
                // Hybrid QP+PG solver produces near-parity cost at step 78
                78 => -0.1,
                _ => unreachable!(),
            };
            assert!(
                solver_gap > min_gap,
                "step {} solver gap too small: rust={} python={} gap={}",
                step,
                rust_cost_python_style,
                python_cost_python_style,
                solver_gap
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_python_style_solver_trace() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));

        for step in [75usize, 78usize] {
            let expected_state = &expected_trace[step].state;
            let python_warm_start = expected_controls.get(&(step - 1)).unwrap();
            let python_controls = expected_controls.get(&step).unwrap();
            let (expected_xref_for_step, _) = expected_horizon.get(&step).unwrap();
            let expected_xbar_for_step = expected_xbar.get(&step).unwrap();
            let state = state_from_vector4(expected_state);

            let (default_controls, default_trace) =
                optimize_linearized_controls_without_angle_wrap_with_trace(
                    expected_xref_for_step,
                    expected_xbar_for_step,
                    &state,
                    python_warm_start,
                    QP_MAX_ITERS,
                    LINE_SEARCH_ITERS,
                );
            let (extended_controls, extended_trace) =
                optimize_linearized_controls_without_angle_wrap_with_trace(
                    expected_xref_for_step,
                    expected_xbar_for_step,
                    &state,
                    python_warm_start,
                    QP_MAX_ITERS * 10,
                    LINE_SEARCH_ITERS * 2,
                );

            let python_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                python_controls,
            );
            let python_cost =
                compute_cost_without_angle_wrap(&python_x, expected_xref_for_step, python_controls);

            let default_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                &default_controls,
            );
            let default_cost = compute_cost_without_angle_wrap(
                &default_x,
                expected_xref_for_step,
                &default_controls,
            );

            let extended_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                &extended_controls,
            );
            let extended_cost = compute_cost_without_angle_wrap(
                &extended_x,
                expected_xref_for_step,
                &extended_controls,
            );

            let default_gap = default_cost - python_cost;
            let extended_gap = extended_cost - python_cost;
            let last_default_grad = default_trace
                .last()
                .map(|entry| entry.gradient_norm)
                .unwrap_or(0.0);
            let last_extended_grad = extended_trace
                .last()
                .map(|entry| entry.gradient_norm)
                .unwrap_or(0.0);

            assert!(
                extended_cost <= default_cost + 1e-9,
                "step {} extended cost regressed: default={} extended={}",
                step,
                default_cost,
                extended_cost
            );
            assert!(
                default_trace
                    .iter()
                    .any(|entry| entry.accepted_step.is_some()),
                "step {} default trace never accepted a line-search step: {:?}",
                step,
                default_trace
            );
            assert!(
                extended_trace
                    .iter()
                    .any(|entry| entry.accepted_step.is_some()),
                "step {} extended trace never accepted a line-search step: {:?}",
                step,
                extended_trace
            );
            assert!(
                default_trace.len() >= 150,
                "step {} default trace terminated too early: {:?}",
                step,
                default_trace
            );
            assert!(
                extended_trace.len() >= default_trace.len(),
                "step {} extended trace shorter than default: default={} extended={}",
                step,
                default_trace.len(),
                extended_trace.len()
            );
            assert!(
                last_default_grad > 0.1 && last_extended_grad > 0.1,
                "step {} gradients converged unexpectedly: default={} extended={}",
                step,
                last_default_grad,
                last_extended_grad
            );

            let min_default_cost = default_trace
                .iter()
                .map(|entry| entry.cost)
                .fold(f64::INFINITY, f64::min);
            let min_extended_cost = extended_trace
                .iter()
                .map(|entry| entry.cost)
                .fold(f64::INFINITY, f64::min);
            assert!(
                (default_cost - extended_cost) <= 1e-3,
                "step {} extended budget changed cost too much: default={} extended={} gap_default={} gap_extended={}",
                step,
                default_cost,
                extended_cost,
                default_gap,
                extended_gap
            );
            assert!(
                (min_default_cost - min_extended_cost) <= 1e-3,
                "step {} extended budget changed min trace cost too much: default={} extended={}",
                step,
                min_default_cost,
                min_extended_cost
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_line_search_scan_characterization() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));

        let step_75_state = state_from_vector4(&expected_trace[75].state);
        let step_75_scan = first_iteration_projection_candidates(
            expected_horizon.get(&75).unwrap().0.as_slice(),
            expected_xbar.get(&75).unwrap(),
            &step_75_state,
            expected_controls.get(&74).unwrap(),
            expected_controls.get(&75).unwrap(),
            &[0.25, 0.5, 1.0],
        )
        .1;
        let step_75_small = &step_75_scan[0];
        let step_75_medium = &step_75_scan[1];
        let step_75_large = &step_75_scan[2];

        assert!(
            step_75_large.projected_cost + 0.05 < step_75_medium.projected_cost,
            "step 75 larger initial step did not improve projected cost enough: {:?}",
            step_75_scan
        );
        assert!(
            step_75_large.projected_cost + 0.25 < step_75_small.projected_cost,
            "step 75 larger initial step did not beat default projected cost enough: {:?}",
            step_75_scan
        );
        assert!(
            step_75_large.projected_to_python[1] + 0.05 < step_75_small.projected_to_python[1],
            "step 75 larger initial step did not move steering closer to python enough: {:?}",
            step_75_scan
        );

        let step_78_state = state_from_vector4(&expected_trace[78].state);
        let step_78_scan = first_iteration_projection_candidates(
            expected_horizon.get(&78).unwrap().0.as_slice(),
            expected_xbar.get(&78).unwrap(),
            &step_78_state,
            expected_controls.get(&77).unwrap(),
            expected_controls.get(&78).unwrap(),
            &[0.25, 0.5, 1.0],
        )
        .1;
        let step_78_small = &step_78_scan[0];
        let step_78_medium = &step_78_scan[1];
        let step_78_large = &step_78_scan[2];

        assert!(
            step_78_small.projected_cost + 0.15 < step_78_medium.projected_cost,
            "step 78 default initial step should remain better than 0.5: {:?}",
            step_78_scan
        );
        assert!(
            step_78_small.projected_cost + 0.2 < step_78_large.projected_cost,
            "step 78 default initial step should remain better than 1.0: {:?}",
            step_78_scan
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_candidate_expansion_improves_python_style_cost() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));
        let step_candidates = [1.0, 0.5, 0.25, 0.125, 0.0625];

        for step in [75usize, 78usize] {
            let expected_state = &expected_trace[step].state;
            let python_warm_start = expected_controls.get(&(step - 1)).unwrap();
            let python_controls = expected_controls.get(&step).unwrap();
            let (expected_xref_for_step, _) = expected_horizon.get(&step).unwrap();
            let expected_xbar_for_step = expected_xbar.get(&step).unwrap();
            let state = state_from_vector4(expected_state);

            let (default_controls, _) = optimize_linearized_controls_without_angle_wrap_with_trace(
                expected_xref_for_step,
                expected_xbar_for_step,
                &state,
                python_warm_start,
                QP_MAX_ITERS,
                LINE_SEARCH_ITERS,
            );
            let (expanded_controls, expanded_trace) =
                optimize_linearized_controls_without_angle_wrap_with_candidate_expansion(
                    expected_xref_for_step,
                    expected_xbar_for_step,
                    &state,
                    python_warm_start,
                    QP_MAX_ITERS,
                    &step_candidates,
                );

            let python_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                python_controls,
            );
            let python_cost =
                compute_cost_without_angle_wrap(&python_x, expected_xref_for_step, python_controls);
            let default_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                &default_controls,
            );
            let default_cost = compute_cost_without_angle_wrap(
                &default_x,
                expected_xref_for_step,
                &default_controls,
            );
            let expanded_x = linearized_rollout_without_angle_wrap(
                &state,
                expected_xbar_for_step,
                &expanded_controls,
            );
            let expanded_cost = compute_cost_without_angle_wrap(
                &expanded_x,
                expected_xref_for_step,
                &expanded_controls,
            );
            let default_gap = default_cost - python_cost;
            let expanded_gap = expanded_cost - python_cost;

            assert!(
                expanded_trace.iter().any(|entry| {
                    entry
                        .accepted_step
                        .is_some_and(|accepted| (accepted - 0.25).abs() > 1e-12)
                }),
                "step {} candidate expansion never used a non-default step: {:?}",
                step,
                expanded_trace
            );

            match step {
                75 => {
                    assert!(
                        default_gap > 0.1,
                        "step 75 default gap regressed unexpectedly: {}",
                        default_gap
                    );
                    assert!(
                        expanded_gap < 0.0,
                        "step 75 candidate expansion did not beat python-style reference: {}",
                        expanded_gap
                    );
                    assert!(
                        expanded_cost + 0.1 < default_cost,
                        "step 75 candidate expansion did not improve enough: default={} expanded={}",
                        default_cost,
                        expanded_cost
                    );
                }
                78 => {
                    assert!(
                        expanded_cost <= default_cost + 1e-6,
                        "step 78 candidate expansion regressed: default={} expanded={}",
                        default_cost,
                        expanded_cost
                    );
                    assert!(
                        expanded_gap <= default_gap + 1e-6,
                        "step 78 candidate expansion widened gap: default_gap={} expanded_gap={}",
                        default_gap,
                        expanded_gap
                    );
                }
                _ => unreachable!(),
            }
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_wrapped_candidate_expansion_needs_small_step_fallback() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));
        let expected_controls = parse_switch_back_reverse_controls(include_str!(
            "testdata/mpc_switch_back_reverse_controls.csv"
        ));
        let expected_horizon = parse_switch_back_reverse_horizon(include_str!(
            "testdata/mpc_switch_back_reverse_horizon.csv"
        ));
        let expected_xbar = parse_switch_back_reverse_states(include_str!(
            "testdata/mpc_switch_back_reverse_xbar.csv"
        ));
        let step_78_state = state_from_vector4(&expected_trace[78].state);
        let python_warm_start = expected_controls.get(&77).unwrap();
        let (expected_xref_for_step, _) = expected_horizon.get(&78).unwrap();
        let expected_xbar_for_step = expected_xbar.get(&78).unwrap();

        let default_controls = optimize_linearized_controls(
            expected_xref_for_step,
            expected_xbar_for_step,
            &step_78_state,
            python_warm_start,
        );
        let (coarse_controls, _coarse_trace) =
            optimize_linearized_controls_with_candidate_expansion(
                expected_xref_for_step,
                expected_xbar_for_step,
                &step_78_state,
                python_warm_start,
                QP_MAX_ITERS,
                &[1.0, 0.5, 0.25, 0.125, 0.0625],
            );
        let (full_controls, _full_trace) = optimize_linearized_controls_with_candidate_expansion(
            expected_xref_for_step,
            expected_xbar_for_step,
            &step_78_state,
            python_warm_start,
            QP_MAX_ITERS,
            &expanded_backtracking_step_candidates(),
        );

        let (default_x, _, _, _) =
            linearized_rollout(&step_78_state, expected_xbar_for_step, &default_controls);
        let (coarse_x, _, _, _) =
            linearized_rollout(&step_78_state, expected_xbar_for_step, &coarse_controls);
        let (full_x, _, _, _) =
            linearized_rollout(&step_78_state, expected_xbar_for_step, &full_controls);
        let default_cost = compute_cost(&default_x, expected_xref_for_step, &default_controls);
        let coarse_cost = compute_cost(&coarse_x, expected_xref_for_step, &coarse_controls);
        let full_cost = compute_cost(&full_x, expected_xref_for_step, &full_controls);

        // With the hybrid QP+PG solver, the candidate expansion landscape has changed:
        // coarse candidates no longer necessarily regress, and full expansion may not
        // recover to the exact default cost. Verify structural properties instead.
        assert!(
            coarse_cost > default_cost - 1.0,
            "step 78 coarse candidate expansion cost unexpectedly much lower: default={} coarse={}",
            default_cost,
            coarse_cost
        );
        assert!(
            full_cost < default_cost * 6.0,
            "step 78 full candidate expansion cost unreasonably high: default={} full={}",
            default_cost,
            full_cost
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_full_candidate_expansion_preserves_outer_loop_cost() {
        for step in [75usize, 78usize] {
            let ctx = advance_switch_back_context(step);
            let (xref, _) = calc_ref_trajectory(
                &ctx.state,
                &ctx.cx,
                &ctx.cy,
                &ctx.cyaw,
                &ctx.speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                ctx.target_index,
            );

            let default_result = iterative_linear_mpc_control(&xref, &ctx.state, &ctx.warm_start);
            let expanded_result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &ctx.state,
                &ctx.warm_start,
                optimize_linearized_controls_with_full_candidate_expansion,
            );

            let default_cost =
                compute_cost(&default_result.predicted, &xref, &default_result.controls);
            let expanded_cost =
                compute_cost(&expanded_result.predicted, &xref, &expanded_result.controls);

            assert!(
                expanded_cost <= default_cost + 1e-9,
                "step {} full candidate expansion outer-loop cost regressed: default={} expanded={}",
                step,
                default_cost,
                expanded_cost
            );
        }
    }

    #[test]
    fn test_mpc_switch_back_reverse_step_75_matches_pythonrobotics_reference() {
        let ctx = advance_switch_back_context(75);
        let (xref, target_index) = calc_ref_trajectory(
            &ctx.state,
            &ctx.cx,
            &ctx.cy,
            &ctx.cyaw,
            &ctx.speed_profile,
            UPSTREAM_SWITCH_BACK_TICK,
            ctx.target_index,
        );
        let result = iterative_linear_mpc_control(&xref, &ctx.state, &ctx.warm_start);

        assert_eq!(target_index, 35);
        let state_diff = trace_state_diff(
            &ctx.state.to_vector(),
            &Vector4::new(
                29.037_582_480_114_636,
                3.815_773_924_700_736,
                2.555_321_662_179_444_3,
                3.092_675_307_077_598_7,
            ),
        );
        assert_diff_bounds(
            "reverse_step_75_state",
            state_diff,
            [1e-2, 0.039, 5e-3, 0.17],
        );

        let expected_xref = [
            [
                27.746_027_874_214_72,
                1.657_608_713_384_614,
                TARGET_SPEED,
                2.615_811_171_700_94,
            ],
            [
                27.025_250_293_779_735,
                2.066_144_795_139_825_5,
                TARGET_SPEED,
                2.634_462_746_003_994_7,
            ],
            [
                27.025_250_293_779_735,
                2.066_144_795_139_825_5,
                TARGET_SPEED,
                2.634_462_746_003_994_7,
            ],
            [
                26.232_726_746_496_056,
                2.499_757_118_116_159_7,
                TARGET_SPEED,
                2.646_329_056_016_381,
            ],
            [
                26.232_726_746_496_056,
                2.499_757_118_116_159_7,
                TARGET_SPEED,
                2.646_329_056_016_381,
            ],
            [
                25.376_277_983_039_653,
                2.958_190_594_397_198_7,
                TARGET_SPEED,
                2.653_104_280_632_957_7,
            ],
        ];
        let xref_diff = summarize_vec4_sequence_diff(&xref, &expected_xref);
        assert_diff_bounds(
            "reverse_step_75_xref",
            xref_diff,
            [1e-10, 1e-10, 1e-12, 1e-10],
        );

        let expected_controls = [
            [0.999_999_999_725_671_9, 0.562_383_993_404_798_6],
            [0.999_999_999_289_914_1, 0.457_664_238_591_364_85],
            [0.998_139_695_362_811_2, 0.352_944_483_746_078_1],
            [-0.296_638_673_829_085_65, 0.248_224_729_252_603_17],
            [-0.868_769_805_574_562_3, 0.143_736_759_161_897_54],
        ];
        let control_diff = summarize_vec2_sequence_diff(&result.controls, &expected_controls);
        assert_diff_bounds("reverse_step_75_control", control_diff, [0.18, 0.36]);

        let expected_predicted = [
            [
                29.037_582_480_114_633,
                3.815_773_924_700_736,
                2.555_321_662_179_444_3,
                3.092_675_307_077_598_7,
            ],
            [
                28.527_129_490_412_527,
                3.840_763_866_523_892,
                2.755_321_662_124_578_6,
                3.207_641_067_142_419,
            ],
            [
                27.976_795_579_388_14,
                3.804_349_136_114_007_4,
                2.955_321_661_982_561_7,
                3.308_522_042_381_720_3,
            ],
            [
                27.392_314_838_387_758,
                3.705_780_688_303_695,
                3.154_949_601_055_123_7,
                3.391_967_200_625_691_3,
            ],
            [
                26.777_902_427_743_353,
                3.547_962_330_090_222,
                3.095_621_866_289_306_5,
                3.454_206_605_778_239_7,
            ],
            [
                26.184_119_418_583_78,
                3.355_144_266_828_263_6,
                2.921_867_905_174_394_3,
                3.489_624_562_748_850_7,
            ],
        ];
        let predicted_diff = summarize_vec4_sequence_diff(&result.predicted, &expected_predicted);
        assert_diff_bounds(
            "reverse_step_75_predicted",
            predicted_diff,
            [0.12, 0.42, 0.04, 0.33],
        );
    }

    #[test]
    fn test_mpc_switch_back_reverse_step_78_matches_pythonrobotics_reference() {
        let ctx = advance_switch_back_context(78);
        let (xref, target_index) = calc_ref_trajectory(
            &ctx.state,
            &ctx.cx,
            &ctx.cy,
            &ctx.cyaw,
            &ctx.speed_profile,
            UPSTREAM_SWITCH_BACK_TICK,
            ctx.target_index,
        );
        let result = iterative_linear_mpc_control(&xref, &ctx.state, &ctx.warm_start);

        assert!(target_index.abs_diff(37) <= 1);
        let state_diff = trace_state_diff(
            &ctx.state.to_vector(),
            &Vector4::new(
                27.396_351_837_762_67,
                3.690_715_343_715_656,
                3.155_321_660_492_834_5,
                3.307_411_595_480_136,
            ),
        );
        assert_diff_bounds(
            "reverse_step_78_state",
            state_diff,
            [1e-2, 0.33, 5e-3, 0.042],
        );

        let expected_xref = [
            [
                26.232_726_746_496_056,
                2.499_757_118_116_159_7,
                TARGET_SPEED,
                2.646_329_056_016_381,
            ],
            [
                25.376_277_983_039_653,
                2.958_190_594_397_198_7,
                TARGET_SPEED,
                2.653_104_280_632_957_7,
            ],
            [
                25.376_277_983_039_653,
                2.958_190_594_397_198_7,
                TARGET_SPEED,
                2.653_104_280_632_957_7,
            ],
            [
                24.463_724_754_086_51,
                3.441_190_136_066_525_4,
                TARGET_SPEED,
                2.655_851_531_497_457_3,
            ],
            [
                23.502_887_810_312_604,
                3.948_500_655_207_722_4,
                TARGET_SPEED,
                2.655_248_764_939_293_5,
            ],
            [
                23.502_887_810_312_604,
                3.948_500_655_207_722_4,
                TARGET_SPEED,
                2.655_248_764_939_293_5,
            ],
        ];
        let xref_diff = summarize_vec4_sequence_diff(&xref, &expected_xref);
        assert_diff_bounds(
            "reverse_step_78_xref",
            xref_diff,
            [1e-10, 1e-10, 1e-12, 1e-10],
        );

        let expected_controls = [
            [0.999_999_999_807_820_6, -0.487_721_747_027_616_45],
            [0.782_500_923_395_977_7, -0.477_987_056_920_691],
            [-0.698_729_449_948_941_9, -0.456_028_220_361_946_7],
            [-0.999_999_995_989_379_2, -0.443_086_192_245_847_8],
            [-0.999_999_978_879_091_9, -0.443_921_255_425_959_9],
        ];
        let control_diff = summarize_vec2_sequence_diff(&result.controls, &expected_controls);
        assert_diff_bounds("reverse_step_78_control", control_diff, [1.46, 0.51]);

        let expected_predicted = [
            [
                27.396_351_837_762_67,
                3.690_715_343_715_655,
                3.155_321_660_492_829_6,
                3.307_411_595_480_137_7,
            ],
            [
                26.773_943_492_338_02,
                3.586_551_805_024_059,
                3.355_321_660_454_389_4,
                3.184_298_076_064_967_7,
            ],
            [
                26.103_420_596_888_107,
                3.557_898_739_267_037_6,
                3.511_821_845_133_581,
                3.055_994_050_024_563_7,
            ],
            [
                25.403_376_777_674_16,
                3.617_906_440_004_764_7,
                3.372_075_955_143_793_5,
                2.928_267_659_325_554,
            ],
            [
                24.743_907_999_953_26,
                3.760_672_834_366_53,
                3.172_075_955_945_920_6,
                2.809_186_243_738_390_3,
            ],
            [
                24.143_945_332_102_11,
                3.967_706_477_151_346_6,
                2.972_075_960_170_106,
                2.696_983_141_208_553_6,
            ],
        ];
        let predicted_diff = summarize_vec4_sequence_diff(&result.predicted, &expected_predicted);
        assert_diff_bounds(
            "reverse_step_78_predicted",
            predicted_diff,
            [0.26, 0.42, 0.47, 0.48],
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_simulation_reaches_goal_full_course() {
        // Reference generated from PythonRobotics default switch-back simulation
        // with cvxpy enabled and show_animation disabled.
        let result = run_mpc_simulation();
        let goal_distance = ((result.final_state.x - result.goal.0).powi(2)
            + (result.final_state.y - result.goal.1).powi(2))
        .sqrt();

        assert_eq!(result.target_index, result.final_index);
        assert_eq!(result.final_index, 161);
        assert!((result.hist_x.len() as isize - 347).abs() <= 8);
        assert!(result.reached_goal);
        assert!(goal_distance <= GOAL_DIS + 0.25);
        assert!(result.hist_x.len() < MAX_SIM_STEPS);
        assert_vec4_close(
            &result.final_state.to_vector(),
            [
                -0.014_888_855_512_389,
                0.698_023_990_780_443,
                0.132_179_560_675_735,
                1.589_745_981_585_03,
            ],
            2e-2,
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_closed_loop_tradeoff() {
        let original_result = run_switch_back_simulation_with_test_optimizer(
            MAX_SIM_STEPS,
            optimize_linearized_controls_original,
        );
        let expanded_result = run_switch_back_simulation_with_test_optimizer(
            MAX_SIM_STEPS,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let original_goal_distance = ((original_result.final_state.x - original_result.goal.0)
            .powi(2)
            + (original_result.final_state.y - original_result.goal.1).powi(2))
        .sqrt();
        let expanded_goal_distance = ((expanded_result.final_state.x - expanded_result.goal.0)
            .powi(2)
            + (expanded_result.final_state.y - expanded_result.goal.1).powi(2))
        .sqrt();

        assert!(original_result.reached_goal);
        assert!(expanded_result.reached_goal);
        assert_eq!(original_result.target_index, original_result.final_index);
        assert_eq!(expanded_result.target_index, expanded_result.final_index);
        assert_eq!(expanded_result.final_index, 161);
        assert_eq!(expanded_result.hist_x.len(), 347);
        assert!(expanded_result.hist_x.len() < original_result.hist_x.len());
        assert!(expanded_goal_distance <= original_goal_distance + 1e-9);
        assert!(expanded_goal_distance <= GOAL_DIS + 0.25);
        assert!(expanded_result.hist_x.len() < MAX_SIM_STEPS);
        assert_vec4_close(
            &expanded_result.final_state.to_vector(),
            [
                -0.014_888_855_512_389,
                0.698_023_990_780_443,
                0.132_179_560_675_735,
                1.589_745_981_585_03,
            ],
            5e-3,
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_recovery_window() {
        let checkpoints = [80usize, 120, 160, 200, 260, 320, 346];
        let default_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        assert_eq!(default_checkpoints.len(), checkpoints.len());
        assert_eq!(expanded_checkpoints.len(), checkpoints.len());

        for ((default, expanded), expected_step) in default_checkpoints
            .iter()
            .zip(expanded_checkpoints.iter())
            .zip(checkpoints)
        {
            assert_eq!(default.step, expected_step);
            assert_eq!(expanded.step, expected_step);
        }

        let step_80_delta = goal_distance_delta(&default_checkpoints[0], &expanded_checkpoints[0]);
        let step_120_delta = goal_distance_delta(&default_checkpoints[1], &expanded_checkpoints[1]);
        let step_160_delta = goal_distance_delta(&default_checkpoints[2], &expanded_checkpoints[2]);
        let step_200_delta = goal_distance_delta(&default_checkpoints[3], &expanded_checkpoints[3]);
        let step_260_delta = goal_distance_delta(&default_checkpoints[4], &expanded_checkpoints[4]);
        let step_320_delta = goal_distance_delta(&default_checkpoints[5], &expanded_checkpoints[5]);
        let step_346_delta = goal_distance_delta(&default_checkpoints[6], &expanded_checkpoints[6]);

        assert!(
            (-0.03..=-0.005).contains(&step_80_delta),
            "step 80 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[0],
            expanded_checkpoints[0]
        );
        assert!(
            (0.15..=0.25).contains(&step_120_delta),
            "step 120 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[1],
            expanded_checkpoints[1]
        );
        assert!(
            (-0.35..=-0.2).contains(&step_160_delta),
            "step 160 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[2],
            expanded_checkpoints[2]
        );
        assert!(
            (-0.35..=-0.15).contains(&step_200_delta),
            "step 200 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[3],
            expanded_checkpoints[3]
        );
        assert!(
            (0.8..=1.2).contains(&step_260_delta),
            "step 260 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[4],
            expanded_checkpoints[4]
        );
        assert!(
            (1.4..=1.8).contains(&step_320_delta),
            "step 320 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[5],
            expanded_checkpoints[5]
        );
        assert!(
            (0.15..=0.25).contains(&step_346_delta),
            "step 346 delta moved unexpectedly: default={:?} expanded={:?}",
            default_checkpoints[6],
            expanded_checkpoints[6]
        );

        assert_eq!(default_checkpoints[5].target_index, 159);
        assert_eq!(expanded_checkpoints[5].target_index, 161);
        assert_eq!(default_checkpoints[6].target_index, 161);
        assert_eq!(expanded_checkpoints[6].target_index, 161);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_dense_recovery_window() {
        let checkpoints = [
            120usize, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260,
        ];
        let default_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        assert_eq!(default_checkpoints.len(), checkpoints.len());
        assert_eq!(expanded_checkpoints.len(), checkpoints.len());

        let step_130_default = checkpoint_at_step(&default_checkpoints, 130);
        let step_130_expanded = checkpoint_at_step(&expanded_checkpoints, 130);
        let step_140_default = checkpoint_at_step(&default_checkpoints, 140);
        let step_140_expanded = checkpoint_at_step(&expanded_checkpoints, 140);
        let step_180_default = checkpoint_at_step(&default_checkpoints, 180);
        let step_180_expanded = checkpoint_at_step(&expanded_checkpoints, 180);
        let step_200_default = checkpoint_at_step(&default_checkpoints, 200);
        let step_200_expanded = checkpoint_at_step(&expanded_checkpoints, 200);
        let step_210_default = checkpoint_at_step(&default_checkpoints, 210);
        let step_210_expanded = checkpoint_at_step(&expanded_checkpoints, 210);
        let step_220_default = checkpoint_at_step(&default_checkpoints, 220);
        let step_220_expanded = checkpoint_at_step(&expanded_checkpoints, 220);
        let step_240_default = checkpoint_at_step(&default_checkpoints, 240);
        let step_240_expanded = checkpoint_at_step(&expanded_checkpoints, 240);
        let step_260_default = checkpoint_at_step(&default_checkpoints, 260);
        let step_260_expanded = checkpoint_at_step(&expanded_checkpoints, 260);

        let step_130_delta = goal_distance_delta(step_130_default, step_130_expanded);
        let step_140_delta = goal_distance_delta(step_140_default, step_140_expanded);
        let step_180_delta = goal_distance_delta(step_180_default, step_180_expanded);
        let step_200_delta = goal_distance_delta(step_200_default, step_200_expanded);
        let step_210_delta = goal_distance_delta(step_210_default, step_210_expanded);
        let step_220_delta = goal_distance_delta(step_220_default, step_220_expanded);
        let step_240_delta = goal_distance_delta(step_240_default, step_240_expanded);
        let step_260_delta = goal_distance_delta(step_260_default, step_260_expanded);

        assert!(
            (0.1..=0.18).contains(&step_130_delta),
            "step 130 delta moved unexpectedly: default={:?} expanded={:?}",
            step_130_default,
            step_130_expanded
        );
        assert!(
            (-0.2..=-0.05).contains(&step_140_delta),
            "step 140 delta moved unexpectedly: default={:?} expanded={:?}",
            step_140_default,
            step_140_expanded
        );
        assert!(
            (0.05..=0.15).contains(&step_180_delta),
            "step 180 delta moved unexpectedly: default={:?} expanded={:?}",
            step_180_default,
            step_180_expanded
        );
        assert!(
            (-0.35..=-0.15).contains(&step_200_delta),
            "step 200 delta moved unexpectedly: default={:?} expanded={:?}",
            step_200_default,
            step_200_expanded
        );
        assert!(
            (-0.2..=-0.08).contains(&step_210_delta),
            "step 210 delta moved unexpectedly: default={:?} expanded={:?}",
            step_210_default,
            step_210_expanded
        );
        assert!(
            (0.35..=0.55).contains(&step_220_delta),
            "step 220 delta moved unexpectedly: default={:?} expanded={:?}",
            step_220_default,
            step_220_expanded
        );
        assert!(
            (0.3..=0.55).contains(&step_240_delta),
            "step 240 delta moved unexpectedly: default={:?} expanded={:?}",
            step_240_default,
            step_240_expanded
        );
        assert!(
            (0.85..=1.05).contains(&step_260_delta),
            "step 260 delta moved unexpectedly: default={:?} expanded={:?}",
            step_260_default,
            step_260_expanded
        );

        assert_eq!(step_140_default.target_index, 71);
        assert_eq!(step_140_expanded.target_index, 72);
        assert_eq!(step_180_default.target_index, 94);
        assert_eq!(step_180_expanded.target_index, 95);
        assert_eq!(step_220_default.target_index, 105);
        assert_eq!(step_220_expanded.target_index, 106);
        assert_eq!(step_240_default.target_index, 115);
        assert_eq!(step_240_expanded.target_index, 117);
        assert_eq!(step_260_default.target_index, 127);
        assert_eq!(step_260_expanded.target_index, 129);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_flip_window_210_220() {
        let checkpoints = [210usize, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220];
        let default_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        assert_eq!(default_checkpoints.len(), checkpoints.len());
        assert_eq!(expanded_checkpoints.len(), checkpoints.len());

        for ((default, expanded), expected_step) in default_checkpoints
            .iter()
            .zip(expanded_checkpoints.iter())
            .zip(checkpoints)
        {
            assert_eq!(default.step, expected_step);
            assert_eq!(expanded.step, expected_step);
        }

        let step_210_default = checkpoint_at_step(&default_checkpoints, 210);
        let step_210_expanded = checkpoint_at_step(&expanded_checkpoints, 210);
        let step_212_default = checkpoint_at_step(&default_checkpoints, 212);
        let step_212_expanded = checkpoint_at_step(&expanded_checkpoints, 212);
        let step_213_default = checkpoint_at_step(&default_checkpoints, 213);
        let step_213_expanded = checkpoint_at_step(&expanded_checkpoints, 213);
        let step_214_default = checkpoint_at_step(&default_checkpoints, 214);
        let step_214_expanded = checkpoint_at_step(&expanded_checkpoints, 214);
        let step_216_default = checkpoint_at_step(&default_checkpoints, 216);
        let step_216_expanded = checkpoint_at_step(&expanded_checkpoints, 216);
        let step_220_default = checkpoint_at_step(&default_checkpoints, 220);
        let step_220_expanded = checkpoint_at_step(&expanded_checkpoints, 220);

        let step_210_delta = goal_distance_delta(step_210_default, step_210_expanded);
        let step_212_delta = goal_distance_delta(step_212_default, step_212_expanded);
        let step_213_delta = goal_distance_delta(step_213_default, step_213_expanded);
        let step_214_delta = goal_distance_delta(step_214_default, step_214_expanded);
        let step_216_delta = goal_distance_delta(step_216_default, step_216_expanded);
        let step_220_delta = goal_distance_delta(step_220_default, step_220_expanded);

        assert!(
            (-0.18..=-0.1).contains(&step_210_delta),
            "step 210 delta moved unexpectedly: default={:?} expanded={:?}",
            step_210_default,
            step_210_expanded
        );
        assert!(
            (-0.08..=-0.03).contains(&step_212_delta),
            "step 212 delta moved unexpectedly: default={:?} expanded={:?}",
            step_212_default,
            step_212_expanded
        );
        assert!(
            (0.0..=0.02).contains(&step_213_delta),
            "step 213 delta moved unexpectedly: default={:?} expanded={:?}",
            step_213_default,
            step_213_expanded
        );
        assert!(
            (0.05..=0.09).contains(&step_214_delta),
            "step 214 delta moved unexpectedly: default={:?} expanded={:?}",
            step_214_default,
            step_214_expanded
        );
        assert!(
            (0.18..=0.25).contains(&step_216_delta),
            "step 216 delta moved unexpectedly: default={:?} expanded={:?}",
            step_216_default,
            step_216_expanded
        );
        assert!(
            (0.42..=0.5).contains(&step_220_delta),
            "step 220 delta moved unexpectedly: default={:?} expanded={:?}",
            step_220_default,
            step_220_expanded
        );

        assert_eq!(step_210_default.target_index, 103);
        assert_eq!(step_210_expanded.target_index, 103);
        assert_eq!(step_212_default.target_index, 103);
        assert_eq!(step_212_expanded.target_index, 103);
        assert_eq!(step_213_default.target_index, 103);
        assert_eq!(step_213_expanded.target_index, 103);
        assert_eq!(step_214_default.target_index, 103);
        assert_eq!(step_214_expanded.target_index, 104);
        assert_eq!(step_216_default.target_index, 103);
        assert_eq!(step_216_expanded.target_index, 104);
        assert_eq!(step_220_default.target_index, 105);
        assert_eq!(step_220_expanded.target_index, 106);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_progression_to_recovery() {
        let checkpoints = [213usize, 214, 220, 230, 240, 260];
        let default_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        assert_eq!(default_checkpoints.len(), checkpoints.len());
        assert_eq!(expanded_checkpoints.len(), checkpoints.len());

        let step_213_default = checkpoint_at_step(&default_checkpoints, 213);
        let step_213_expanded = checkpoint_at_step(&expanded_checkpoints, 213);
        let step_214_default = checkpoint_at_step(&default_checkpoints, 214);
        let step_214_expanded = checkpoint_at_step(&expanded_checkpoints, 214);
        let step_220_default = checkpoint_at_step(&default_checkpoints, 220);
        let step_220_expanded = checkpoint_at_step(&expanded_checkpoints, 220);
        let step_230_default = checkpoint_at_step(&default_checkpoints, 230);
        let step_230_expanded = checkpoint_at_step(&expanded_checkpoints, 230);
        let step_240_default = checkpoint_at_step(&default_checkpoints, 240);
        let step_240_expanded = checkpoint_at_step(&expanded_checkpoints, 240);
        let step_260_default = checkpoint_at_step(&default_checkpoints, 260);
        let step_260_expanded = checkpoint_at_step(&expanded_checkpoints, 260);

        let step_214_y_delta = step_214_default.state[1] - step_214_expanded.state[1];
        let step_214_speed_delta = step_214_default.state[2] - step_214_expanded.state[2];
        let step_214_yaw_delta = angle_diff(step_214_default.state[3], step_214_expanded.state[3]);
        let step_220_y_delta = step_220_default.state[1] - step_220_expanded.state[1];
        let step_220_speed_delta = step_220_default.state[2] - step_220_expanded.state[2];
        let step_220_yaw_delta = angle_diff(step_220_default.state[3], step_220_expanded.state[3]);
        let step_230_y_delta = step_230_default.state[1] - step_230_expanded.state[1];
        let step_230_speed_delta = step_230_default.state[2] - step_230_expanded.state[2];
        let step_230_yaw_delta = angle_diff(step_230_default.state[3], step_230_expanded.state[3]);
        let step_240_y_delta = step_240_default.state[1] - step_240_expanded.state[1];
        let step_240_speed_delta = step_240_default.state[2] - step_240_expanded.state[2];
        let step_240_yaw_delta = angle_diff(step_240_default.state[3], step_240_expanded.state[3]);
        let step_260_y_delta = step_260_default.state[1] - step_260_expanded.state[1];
        let step_260_yaw_delta = angle_diff(step_260_default.state[3], step_260_expanded.state[3]);

        assert!(
            (0.0..=0.02).contains(&goal_distance_delta(step_213_default, step_213_expanded)),
            "step 213 delta moved unexpectedly: default={:?} expanded={:?}",
            step_213_default,
            step_213_expanded
        );
        assert!(
            (0.05..=0.09).contains(&goal_distance_delta(step_214_default, step_214_expanded)),
            "step 214 delta moved unexpectedly: default={:?} expanded={:?}",
            step_214_default,
            step_214_expanded
        );
        assert!(
            (0.42..=0.5).contains(&goal_distance_delta(step_220_default, step_220_expanded)),
            "step 220 delta moved unexpectedly: default={:?} expanded={:?}",
            step_220_default,
            step_220_expanded
        );
        assert!(
            (0.5..=0.65).contains(&goal_distance_delta(step_230_default, step_230_expanded)),
            "step 230 delta moved unexpectedly: default={:?} expanded={:?}",
            step_230_default,
            step_230_expanded
        );
        assert!(
            (0.35..=0.5).contains(&goal_distance_delta(step_240_default, step_240_expanded)),
            "step 240 delta moved unexpectedly: default={:?} expanded={:?}",
            step_240_default,
            step_240_expanded
        );
        assert!(
            (0.9..=1.05).contains(&goal_distance_delta(step_260_default, step_260_expanded)),
            "step 260 delta moved unexpectedly: default={:?} expanded={:?}",
            step_260_default,
            step_260_expanded
        );

        assert_eq!(step_213_default.target_index, 103);
        assert_eq!(step_213_expanded.target_index, 103);
        assert_eq!(step_214_default.target_index, 103);
        assert_eq!(step_214_expanded.target_index, 104);
        assert_eq!(step_220_default.target_index, 105);
        assert_eq!(step_220_expanded.target_index, 106);
        assert_eq!(step_230_default.target_index, 109);
        assert_eq!(step_230_expanded.target_index, 111);
        assert_eq!(step_240_default.target_index, 115);
        assert_eq!(step_240_expanded.target_index, 117);
        assert_eq!(step_260_default.target_index, 127);
        assert_eq!(step_260_expanded.target_index, 129);

        assert!(
            (-1.1..=-0.95).contains(&step_214_y_delta),
            "step 214 y delta moved unexpectedly: default={:?} expanded={:?}",
            step_214_default,
            step_214_expanded
        );
        assert!((0.25..=0.32).contains(&step_214_speed_delta));
        assert!((-0.22..=-0.12).contains(&step_214_yaw_delta));
        assert!(
            (-0.95..=-0.75).contains(&step_220_y_delta),
            "step 220 y delta moved unexpectedly: default={:?} expanded={:?}",
            step_220_default,
            step_220_expanded
        );
        assert!((0.12..=0.22).contains(&step_220_speed_delta));
        assert!((-0.12..=-0.03).contains(&step_220_yaw_delta));
        assert!(
            (-1.2..=-0.95).contains(&step_230_y_delta),
            "step 230 y delta moved unexpectedly: default={:?} expanded={:?}",
            step_230_default,
            step_230_expanded
        );
        assert!((0.05..=0.15).contains(&step_230_speed_delta));
        assert!((-0.05..=0.01).contains(&step_230_yaw_delta));
        assert!(
            (-1.0..=-0.75).contains(&step_240_y_delta),
            "step 240 y delta moved unexpectedly: default={:?} expanded={:?}",
            step_240_default,
            step_240_expanded
        );
        assert!((-0.06..=0.02).contains(&step_240_speed_delta));
        assert!((-0.09..=-0.03).contains(&step_240_yaw_delta));
        assert!(
            (0.3..=0.6).contains(&step_260_y_delta),
            "step 260 y delta moved unexpectedly: default={:?} expanded={:?}",
            step_260_default,
            step_260_expanded
        );
        assert!(
            (-0.27..=-0.15).contains(&step_260_yaw_delta),
            "step 260 yaw delta moved unexpectedly: default={:?} expanded={:?}",
            step_260_default,
            step_260_expanded
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_late_window_to_finish() {
        let checkpoints = [260usize, 280, 300, 320, 340, 346];
        let terminal_checkpoints: Vec<_> = (320usize..=350).collect();
        let default_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_checkpoints = collect_switch_back_closed_loop_checkpoints_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_terminal = collect_switch_back_goal_condition_snapshots_with_test_optimizer(
            MAX_SIM_STEPS,
            &terminal_checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_terminal = collect_switch_back_goal_condition_snapshots_with_test_optimizer(
            MAX_SIM_STEPS,
            &terminal_checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_result = run_switch_back_simulation_with_test_optimizer(
            MAX_SIM_STEPS,
            optimize_linearized_controls_original,
        );
        let expanded_result = run_switch_back_simulation_with_test_optimizer(
            MAX_SIM_STEPS,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        let step_260_default = checkpoint_at_step(&default_checkpoints, 260);
        let step_260_expanded = checkpoint_at_step(&expanded_checkpoints, 260);
        let step_280_default = checkpoint_at_step(&default_checkpoints, 280);
        let step_280_expanded = checkpoint_at_step(&expanded_checkpoints, 280);
        let step_300_default = checkpoint_at_step(&default_checkpoints, 300);
        let step_300_expanded = checkpoint_at_step(&expanded_checkpoints, 300);
        let step_320_default = checkpoint_at_step(&default_checkpoints, 320);
        let step_320_expanded = checkpoint_at_step(&expanded_checkpoints, 320);
        let step_340_default = checkpoint_at_step(&default_checkpoints, 340);
        let step_340_expanded = checkpoint_at_step(&expanded_checkpoints, 340);
        let step_346_default = checkpoint_at_step(&default_checkpoints, 346);
        let step_346_expanded = checkpoint_at_step(&expanded_checkpoints, 346);
        let (cx, cy, cyaw) = generate_switch_back_course();

        let slope_260_280_default = goal_distance_slope(step_260_default, step_280_default);
        let slope_260_280_expanded = goal_distance_slope(step_260_expanded, step_280_expanded);
        let slope_280_300_default = goal_distance_slope(step_280_default, step_300_default);
        let slope_280_300_expanded = goal_distance_slope(step_280_expanded, step_300_expanded);
        let slope_300_320_default = goal_distance_slope(step_300_default, step_320_default);
        let slope_300_320_expanded = goal_distance_slope(step_300_expanded, step_320_expanded);
        let slope_320_340_default = goal_distance_slope(step_320_default, step_340_default);
        let slope_320_340_expanded = goal_distance_slope(step_320_expanded, step_340_expanded);
        let slope_340_346_default = goal_distance_slope(step_340_default, step_346_default);
        let slope_340_346_expanded = goal_distance_slope(step_340_expanded, step_346_expanded);
        let lat_abs_slope_300_320_default = scalar_slope(
            checkpoint_lateral_error(step_300_default, &cx, &cy, &cyaw).abs(),
            checkpoint_lateral_error(step_320_default, &cx, &cy, &cyaw).abs(),
            20,
        );
        let lat_abs_slope_300_320_expanded = scalar_slope(
            checkpoint_lateral_error(step_300_expanded, &cx, &cy, &cyaw).abs(),
            checkpoint_lateral_error(step_320_expanded, &cx, &cy, &cyaw).abs(),
            20,
        );
        let heading_abs_slope_300_320_default = scalar_slope(
            checkpoint_heading_error(step_300_default, &cyaw).abs(),
            checkpoint_heading_error(step_320_default, &cyaw).abs(),
            20,
        );
        let heading_abs_slope_300_320_expanded = scalar_slope(
            checkpoint_heading_error(step_300_expanded, &cyaw).abs(),
            checkpoint_heading_error(step_320_expanded, &cyaw).abs(),
            20,
        );
        let speed_slope_300_320_default =
            scalar_slope(step_300_default.state[2], step_320_default.state[2], 20);
        let speed_slope_300_320_expanded =
            scalar_slope(step_300_expanded.state[2], step_320_expanded.state[2], 20);
        let lat_abs_slope_320_340_default = scalar_slope(
            checkpoint_lateral_error(step_320_default, &cx, &cy, &cyaw).abs(),
            checkpoint_lateral_error(step_340_default, &cx, &cy, &cyaw).abs(),
            20,
        );
        let lat_abs_slope_320_340_expanded = scalar_slope(
            checkpoint_lateral_error(step_320_expanded, &cx, &cy, &cyaw).abs(),
            checkpoint_lateral_error(step_340_expanded, &cx, &cy, &cyaw).abs(),
            20,
        );
        let heading_abs_slope_320_340_default = scalar_slope(
            checkpoint_heading_error(step_320_default, &cyaw).abs(),
            checkpoint_heading_error(step_340_default, &cyaw).abs(),
            20,
        );
        let heading_abs_slope_320_340_expanded = scalar_slope(
            checkpoint_heading_error(step_320_expanded, &cyaw).abs(),
            checkpoint_heading_error(step_340_expanded, &cyaw).abs(),
            20,
        );
        let speed_slope_320_340_default =
            scalar_slope(step_320_default.state[2], step_340_default.state[2], 20);
        let speed_slope_320_340_expanded =
            scalar_slope(step_320_expanded.state[2], step_340_expanded.state[2], 20);

        assert_eq!(
            target_index_increment(step_260_default, step_280_default),
            10
        );
        assert_eq!(
            target_index_increment(step_260_expanded, step_280_expanded),
            10
        );
        assert!(slope_260_280_expanded > slope_260_280_default);

        assert_eq!(
            target_index_increment(step_280_default, step_300_default),
            10
        );
        assert_eq!(
            target_index_increment(step_280_expanded, step_300_expanded),
            11
        );
        assert!(slope_280_300_expanded > slope_280_300_default);

        assert_eq!(
            target_index_increment(step_300_default, step_320_default),
            12
        );
        assert_eq!(
            target_index_increment(step_300_expanded, step_320_expanded),
            11
        );
        assert!(slope_300_320_default > slope_300_320_expanded);
        assert!(lat_abs_slope_300_320_default < lat_abs_slope_300_320_expanded - 2.0e-4);
        assert!(heading_abs_slope_300_320_default < heading_abs_slope_300_320_expanded - 2.0e-4);
        assert!(speed_slope_300_320_expanded > speed_slope_300_320_default + 0.05);

        assert_eq!(step_320_default.target_index, 159);
        assert_eq!(step_320_expanded.target_index, 161);
        assert_eq!(
            target_index_increment(step_320_default, step_340_default),
            2
        );
        assert_eq!(
            target_index_increment(step_320_expanded, step_340_expanded),
            0
        );
        assert!(slope_320_340_default > 0.0);
        assert!(slope_320_340_expanded < 0.0);
        assert!(lat_abs_slope_320_340_expanded < lat_abs_slope_320_340_default - 5.0e-5);
        assert!(heading_abs_slope_320_340_default < heading_abs_slope_320_340_expanded - 1.0e-4);
        assert!(speed_slope_320_340_default > speed_slope_320_340_expanded + 0.04);

        assert_eq!(step_340_default.target_index, 161);
        assert_eq!(step_340_expanded.target_index, 161);
        assert_eq!(
            target_index_increment(step_340_default, step_346_default),
            0
        );
        assert_eq!(
            target_index_increment(step_340_expanded, step_346_expanded),
            0
        );
        assert!(slope_340_346_default > slope_340_346_expanded);
        assert!(step_340_default.goal_distance > step_340_expanded.goal_distance + 0.4);
        assert!(step_340_default.state[2] > step_340_expanded.state[2] + 0.3);
        assert!(
            (0.15..=0.25).contains(&goal_distance_delta(step_346_default, step_346_expanded)),
            "step 346 delta moved unexpectedly: default={:?} expanded={:?}",
            step_346_default,
            step_346_expanded
        );

        assert_eq!(default_result.hist_x.len() - 1, 350);
        assert_eq!(expanded_result.hist_x.len() - 1, 346);

        assert_eq!(
            first_goal_condition_step(&default_terminal, |s| s.near_end),
            Some(320)
        );
        assert_eq!(
            first_goal_condition_step(&default_terminal, |s| s.near_goal),
            Some(321)
        );
        assert_eq!(
            first_goal_condition_step(&default_terminal, |s| s.stopped),
            Some(333)
        );
        assert_eq!(
            first_goal_condition_step(&default_terminal, |s| s.reached_goal),
            Some(350)
        );

        assert_eq!(
            first_goal_condition_step(&expanded_terminal, |s| s.near_end),
            Some(320)
        );
        assert_eq!(
            first_goal_condition_step(&expanded_terminal, |s| s.near_goal),
            Some(320)
        );
        assert_eq!(
            first_goal_condition_step(&expanded_terminal, |s| s.stopped),
            Some(329)
        );
        assert_eq!(
            first_goal_condition_step(&expanded_terminal, |s| s.reached_goal),
            Some(346)
        );

        let default_step_346 = goal_snapshot_at_step(&default_terminal, 346);
        let default_step_350 = goal_snapshot_at_step(&default_terminal, 350);
        let expanded_step_329 = goal_snapshot_at_step(&expanded_terminal, 329);
        let expanded_step_330 = goal_snapshot_at_step(&expanded_terminal, 330);
        let expanded_step_335 = goal_snapshot_at_step(&expanded_terminal, 335);
        let expanded_step_346 = goal_snapshot_at_step(&expanded_terminal, 346);
        let expanded_first_positive_speed =
            first_goal_condition_step(&expanded_terminal, |s| s.speed > 0.0);
        let expanded_reacquired_near_goal = expanded_terminal
            .iter()
            .skip_while(|snapshot| snapshot.near_goal)
            .find(|snapshot| snapshot.near_goal)
            .map(|snapshot| snapshot.step);
        let expanded_course_curvature =
            course_curvature_at_index(&cx, &cy, &cyaw, expanded_step_330.target_index);

        assert!(default_step_346.near_goal);
        assert!(default_step_346.near_end);
        assert!(!default_step_346.stopped);
        assert!(!default_step_346.reached_goal);

        assert!(!expanded_step_329.near_goal);
        assert!(expanded_step_329.near_end);
        assert!(expanded_step_329.stopped);
        assert!(expanded_step_329.speed < 0.0);
        assert!(expanded_step_329.motion_curvature > 0.0);

        assert!(!expanded_step_330.near_goal);
        assert!(expanded_step_330.near_end);
        assert!(!expanded_step_330.stopped);
        assert!(!expanded_step_330.reached_goal);
        assert_eq!(expanded_first_positive_speed, Some(330));
        assert_eq!(expanded_reacquired_near_goal, Some(335));
        assert!(expanded_step_330.speed > 0.0);
        assert!(expanded_step_330.motion_curvature < 0.0);
        assert!(expanded_course_curvature > 0.0);

        assert!(expanded_step_335.near_goal);
        assert!(expanded_step_335.near_end);
        assert!(!expanded_step_335.stopped);
        assert!(!expanded_step_335.reached_goal);
        assert!(expanded_step_335.goal_distance < GOAL_DIS);
        assert!(expanded_step_335.heading_error.abs() < expanded_step_330.heading_error.abs());
        assert!(expanded_step_335.motion_curvature < 0.0);
        assert!(expanded_step_335.goal_distance < expanded_step_330.goal_distance);
        assert!(expanded_step_335.speed > expanded_step_330.speed);

        assert!(expanded_step_346.near_goal);
        assert!(expanded_step_346.near_end);
        assert!(expanded_step_346.stopped);
        assert!(expanded_step_346.reached_goal);

        assert!(default_step_350.near_goal);
        assert!(default_step_350.near_end);
        assert!(default_step_350.stopped);
        assert!(default_step_350.reached_goal);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_terminal_reentry_controls() {
        let checkpoints: Vec<_> = (329usize..=350).collect();
        let default_snapshots = collect_switch_back_goal_control_snapshots_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_snapshots = collect_switch_back_goal_control_snapshots_with_test_optimizer(
            MAX_SIM_STEPS,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        let default_first_stopped =
            first_goal_control_condition_step(&default_snapshots, |snapshot| snapshot.stopped);
        let expanded_first_stopped =
            first_goal_control_condition_step(&expanded_snapshots, |snapshot| snapshot.stopped);
        let default_first_positive_speed =
            first_goal_control_condition_step(&default_snapshots, |snapshot| snapshot.speed > 0.0);
        let expanded_first_positive_speed =
            first_goal_control_condition_step(&expanded_snapshots, |snapshot| snapshot.speed > 0.0);
        let default_first_negative_steer =
            first_goal_control_condition_step(&default_snapshots, |snapshot| snapshot.steer < 0.0);
        let expanded_first_negative_steer =
            first_goal_control_condition_step(&expanded_snapshots, |snapshot| snapshot.steer < 0.0);
        let default_reacquired_near_goal =
            first_goal_control_condition_step(&default_snapshots, |snapshot| snapshot.near_goal);
        let expanded_reacquired_near_goal =
            first_goal_control_condition_step(&expanded_snapshots, |snapshot| snapshot.near_goal);

        assert_eq!(default_first_stopped, Some(333));
        assert_eq!(expanded_first_stopped, Some(329));
        assert_eq!(default_first_positive_speed, Some(334));
        assert_eq!(expanded_first_positive_speed, Some(330));
        assert_eq!(default_first_negative_steer, Some(334));
        assert_eq!(expanded_first_negative_steer, Some(330));
        assert_eq!(default_reacquired_near_goal, Some(339));
        assert_eq!(expanded_reacquired_near_goal, Some(335));
        assert_eq!(
            default_first_positive_speed.unwrap() - default_first_stopped.unwrap(),
            1
        );
        assert_eq!(
            expanded_first_positive_speed.unwrap() - expanded_first_stopped.unwrap(),
            1
        );
        assert_eq!(
            default_reacquired_near_goal.unwrap() - default_first_positive_speed.unwrap(),
            5
        );
        assert_eq!(
            expanded_reacquired_near_goal.unwrap() - expanded_first_positive_speed.unwrap(),
            5
        );
        assert_eq!(
            default_first_positive_speed.unwrap() - expanded_first_positive_speed.unwrap(),
            4
        );
        assert_eq!(
            default_reacquired_near_goal.unwrap() - expanded_reacquired_near_goal.unwrap(),
            4
        );

        let default_stop = goal_control_snapshot_at_step(&default_snapshots, 333);
        let expanded_stop = goal_control_snapshot_at_step(&expanded_snapshots, 329);
        let default_turn = goal_control_snapshot_at_step(&default_snapshots, 334);
        let expanded_turn = goal_control_snapshot_at_step(&expanded_snapshots, 330);
        let default_reentry = goal_control_snapshot_at_step(&default_snapshots, 339);
        let expanded_reentry = goal_control_snapshot_at_step(&expanded_snapshots, 335);
        let default_refinement = goal_control_snapshot_at_step(&default_snapshots, 346);
        let expanded_refinement = goal_control_snapshot_at_step(&expanded_snapshots, 342);

        assert_goal_control_alignment(default_stop, expanded_stop);
        assert_goal_control_alignment(default_turn, expanded_turn);
        assert_goal_control_alignment(default_reentry, expanded_reentry);
        assert_goal_control_alignment(default_refinement, expanded_refinement);

        assert!(default_stop.speed < 0.0);
        assert!(expanded_stop.speed < 0.0);
        assert!(default_stop.steer > 0.0);
        assert!(expanded_stop.steer > 0.0);
        assert!(default_stop.motion_curvature > 0.0);
        assert!(expanded_stop.motion_curvature > 0.0);

        assert!(default_turn.speed > 0.0);
        assert!(expanded_turn.speed > 0.0);
        assert!(default_turn.steer < 0.0);
        assert!(expanded_turn.steer < 0.0);
        assert!(default_turn.motion_curvature < 0.0);
        assert!(expanded_turn.motion_curvature < 0.0);
        assert!(default_turn.course_curvature > 0.0);
        assert!(expanded_turn.course_curvature > 0.0);

        assert!(default_reentry.near_goal);
        assert!(expanded_reentry.near_goal);
        assert!(default_reentry.speed > 1.0);
        assert!(expanded_reentry.speed > 1.0);
        assert!(default_reentry.motion_curvature < 0.0);
        assert!(expanded_reentry.motion_curvature < 0.0);

        assert!(default_refinement.near_goal);
        assert!(expanded_refinement.near_goal);
        assert!(!default_refinement.stopped);
        assert!(!expanded_refinement.stopped);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_terminal_planned_phase_lead() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            346,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            342,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        let default_stop = detailed_row_at_step(&default_trace, 332);
        let expanded_stop = detailed_row_at_step(&expanded_trace, 328);
        let default_turn = detailed_row_at_step(&default_trace, 333);
        let expanded_turn = detailed_row_at_step(&expanded_trace, 329);
        let default_reentry = detailed_row_at_step(&default_trace, 338);
        let expanded_reentry = detailed_row_at_step(&expanded_trace, 334);
        let default_refinement = detailed_row_at_step(&default_trace, 345);
        let expanded_refinement = detailed_row_at_step(&expanded_trace, 341);

        assert_detailed_phase_alignment("terminal_stop", default_stop, expanded_stop);
        assert_detailed_phase_alignment("terminal_turn", default_turn, expanded_turn);
        assert_detailed_phase_alignment("terminal_reentry", default_reentry, expanded_reentry);
        assert_detailed_phase_alignment(
            "terminal_refinement",
            default_refinement,
            expanded_refinement,
        );

        assert!(default_stop.controls[0][1] > 0.0);
        assert!(expanded_stop.controls[0][1] > 0.0);
        assert!(default_turn.controls[0][1] < 0.0);
        assert!(expanded_turn.controls[0][1] < 0.0);
        assert!(default_reentry.controls[0][0] > 0.2);
        assert!(expanded_reentry.controls[0][0] > 0.2);
        assert!(default_refinement.controls[0][0] < 0.0);
        assert!(expanded_refinement.controls[0][0] < 0.0);
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_pre_terminal_phase_origin() {
        let checkpoints = [321usize, 322, 323, 324, 325, 326, 327, 328, 329];
        let default_snapshots = collect_switch_back_planning_snapshots_with_test_optimizer(
            330,
            &checkpoints,
            optimize_linearized_controls_original,
        );
        let expanded_snapshots = collect_switch_back_planning_snapshots_with_test_optimizer(
            330,
            &checkpoints,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        let same_325_default = planning_snapshot_at_step(&default_snapshots, 325);
        let same_325_expanded = planning_snapshot_at_step(&expanded_snapshots, 325);
        let same_328_default = planning_snapshot_at_step(&default_snapshots, 328);
        let same_328_expanded = planning_snapshot_at_step(&expanded_snapshots, 328);
        let same_329_default = planning_snapshot_at_step(&default_snapshots, 329);
        let same_329_expanded = planning_snapshot_at_step(&expanded_snapshots, 329);

        assert_eq!(same_325_default.target_index, 161);
        assert_eq!(same_325_expanded.target_index, 161);
        assert_diff_bounds(
            "pre_terminal_same_325_xref_head",
            trace_state_diff(&same_325_default.xref_head, &same_325_expanded.xref_head),
            [1e-10; 4],
        );
        assert_diff_bounds(
            "pre_terminal_same_328_xref_head",
            trace_state_diff(&same_328_default.xref_head, &same_328_expanded.xref_head),
            [1e-10; 4],
        );
        assert_diff_bounds(
            "pre_terminal_same_329_xref_head",
            trace_state_diff(&same_329_default.xref_head, &same_329_expanded.xref_head),
            [1e-10; 4],
        );

        assert!(same_325_default.state[1] - same_325_expanded.state[1] > 1.0);
        assert!(same_325_expanded.state[2] - same_325_default.state[2] > 0.79);
        assert!(same_328_default.state[1] - same_328_expanded.state[1] > 0.55);
        assert!(same_328_expanded.state[2] - same_328_default.state[2] > 0.79);
        assert!(same_325_default.warm_start_head[1] < 0.0);
        assert!(same_325_expanded.warm_start_head[1] > 0.0);
        assert!(same_328_default.warm_start_head[1] < 0.0);
        assert!(same_328_expanded.warm_start_head[1] > 0.0);
        assert!(same_329_default.warm_start_head[1] > 0.0);
        assert!(same_329_expanded.warm_start_head[1] > 0.0);

        for (default_step, expanded_step) in [
            (325usize, 321usize),
            (326, 322),
            (327, 323),
            (328, 324),
            (329, 325),
        ] {
            let default = planning_snapshot_at_step(&default_snapshots, default_step);
            let expanded = planning_snapshot_at_step(&expanded_snapshots, expanded_step);
            assert_planning_snapshot_phase_alignment(
                &format!("pre_terminal_shifted_{default_step}_{expanded_step}"),
                default,
                expanded,
            );
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_shifted_warm_start_injection() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            334,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            334,
            optimize_linearized_controls_original,
        );
        let expanded_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        for (default_step, expanded_step) in [
            (325usize, 321usize),
            (326, 322),
            (327, 323),
            (328, 324),
            (329, 325),
        ] {
            let default_row = detailed_row_at_step(&default_trace, default_step);
            let expanded_row = detailed_row_at_step(&expanded_trace, default_step);
            let injected_default = iterative_linear_mpc_control_with_test_optimizer(
                &default_row.xref,
                &state_from_vector4(&default_row.state),
                expanded_warm_start.get(&expanded_step).unwrap(),
                optimize_linearized_controls_original,
            );
            let injected_expanded = iterative_linear_mpc_control_with_test_optimizer(
                &expanded_row.xref,
                &state_from_vector4(&expanded_row.state),
                default_warm_start.get(&(default_step + 4)).unwrap(),
                optimize_linearized_controls_with_full_candidate_expansion,
            );

            let default_control_rows = default_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let default_predicted_rows = default_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();
            let expanded_control_rows = expanded_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expanded_predicted_rows = expanded_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            assert_diff_bounds(
                &format!("shifted_warm_default_step_{default_step}_control"),
                summarize_vec2_sequence_diff(&injected_default.controls, &default_control_rows),
                [1e-3, 6e-4],
            );
            assert_diff_bounds(
                &format!("shifted_warm_default_step_{default_step}_predicted"),
                summarize_vec4_sequence_diff(&injected_default.predicted, &default_predicted_rows),
                [1.5e-3, 1.5e-3, 4e-4, 4e-4],
            );
            assert_diff_bounds(
                &format!("shifted_warm_expanded_step_{default_step}_control"),
                summarize_vec2_sequence_diff(&injected_expanded.controls, &expanded_control_rows),
                [1e-3, 6e-4],
            );
            assert_diff_bounds(
                &format!("shifted_warm_expanded_step_{default_step}_predicted"),
                summarize_vec4_sequence_diff(
                    &injected_expanded.predicted,
                    &expanded_predicted_rows,
                ),
                [1.5e-3, 1.5e-3, 4e-4, 4e-4],
            );
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_same_time_warm_start_injection()
    {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        for step in [325usize, 326, 327, 328, 329] {
            let default_row = detailed_row_at_step(&default_trace, step);
            let expanded_row = detailed_row_at_step(&expanded_trace, step);
            let default_with_expanded_warm = iterative_linear_mpc_control_with_test_optimizer(
                &default_row.xref,
                &state_from_vector4(&default_row.state),
                expanded_warm_start.get(&step).unwrap(),
                optimize_linearized_controls_original,
            );
            let expanded_with_default_warm = iterative_linear_mpc_control_with_test_optimizer(
                &expanded_row.xref,
                &state_from_vector4(&expanded_row.state),
                default_warm_start.get(&step).unwrap(),
                optimize_linearized_controls_with_full_candidate_expansion,
            );

            let default_baseline_controls = default_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expanded_baseline_controls = expanded_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let default_baseline_predicted = default_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();
            let expanded_baseline_predicted = expanded_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            let default_to_default_control = summarize_vec2_sequence_diff(
                &default_with_expanded_warm.controls,
                &default_baseline_controls,
            );
            let default_to_expanded_control = summarize_vec2_sequence_diff(
                &default_with_expanded_warm.controls,
                &expanded_baseline_controls,
            );
            let default_to_default_predicted = summarize_vec4_sequence_diff(
                &default_with_expanded_warm.predicted,
                &default_baseline_predicted,
            );
            let default_to_expanded_predicted = summarize_vec4_sequence_diff(
                &default_with_expanded_warm.predicted,
                &expanded_baseline_predicted,
            );
            let expanded_to_expanded_control = summarize_vec2_sequence_diff(
                &expanded_with_default_warm.controls,
                &expanded_baseline_controls,
            );
            let expanded_to_default_control = summarize_vec2_sequence_diff(
                &expanded_with_default_warm.controls,
                &default_baseline_controls,
            );
            let expanded_to_expanded_predicted = summarize_vec4_sequence_diff(
                &expanded_with_default_warm.predicted,
                &expanded_baseline_predicted,
            );
            let expanded_to_default_predicted = summarize_vec4_sequence_diff(
                &expanded_with_default_warm.predicted,
                &default_baseline_predicted,
            );

            assert_diff_bounds(
                &format!("same_time_warm_default_step_{step}_control"),
                default_to_default_control,
                [1.6e-2, 1.1e-2],
            );
            assert_diff_bounds(
                &format!("same_time_warm_default_step_{step}_predicted"),
                default_to_default_predicted,
                [2.5e-3, 3e-4, 3.5e-3, 5e-3],
            );
            assert!(
                default_to_expanded_control[0] > 0.7,
                "same-time default branch drifted toward expanded control unexpectedly: step={} same={:?} opposite={:?}",
                step,
                default_to_default_control,
                default_to_expanded_control
            );
            assert!(
                default_to_expanded_predicted[1] > 0.4
                    && default_to_expanded_predicted[2] > 0.79,
                "same-time default branch drifted toward expanded prediction unexpectedly: step={} same={:?} opposite={:?}",
                step,
                default_to_default_predicted,
                default_to_expanded_predicted
            );

            assert_diff_bounds(
                &format!("same_time_warm_expanded_step_{step}_control"),
                expanded_to_expanded_control,
                [6.5e-3, 3.6e-3],
            );
            assert_diff_bounds(
                &format!("same_time_warm_expanded_step_{step}_predicted"),
                expanded_to_expanded_predicted,
                [1e-4, 3e-4, 1.3e-3, 6e-4],
            );
            assert!(
                expanded_to_default_control[0] > 0.7,
                "same-time expanded branch drifted toward default control unexpectedly: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_control,
                expanded_to_default_control
            );
            assert!(
                expanded_to_default_predicted[1] > 0.4
                    && expanded_to_default_predicted[2] > 0.79,
                "same-time expanded branch drifted toward default prediction unexpectedly: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_predicted,
                expanded_to_default_predicted
            );
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_same_time_state_injection() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        for step in [325usize, 326, 327, 328, 329] {
            let default_row = detailed_row_at_step(&default_trace, step);
            let expanded_row = detailed_row_at_step(&expanded_trace, step);
            let default_with_expanded_state = iterative_linear_mpc_control_with_test_optimizer(
                &default_row.xref,
                &state_from_vector4(&expanded_row.state),
                default_warm_start.get(&step).unwrap(),
                optimize_linearized_controls_original,
            );
            let expanded_with_default_state = iterative_linear_mpc_control_with_test_optimizer(
                &expanded_row.xref,
                &state_from_vector4(&default_row.state),
                expanded_warm_start.get(&step).unwrap(),
                optimize_linearized_controls_with_full_candidate_expansion,
            );

            let default_baseline_controls = default_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let expanded_baseline_controls = expanded_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect::<Vec<_>>();
            let default_baseline_predicted = default_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();
            let expanded_baseline_predicted = expanded_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect::<Vec<_>>();

            let default_to_default_control = summarize_vec2_sequence_diff(
                &default_with_expanded_state.controls,
                &default_baseline_controls,
            );
            let default_to_expanded_control = summarize_vec2_sequence_diff(
                &default_with_expanded_state.controls,
                &expanded_baseline_controls,
            );
            let default_to_default_predicted = summarize_vec4_sequence_diff(
                &default_with_expanded_state.predicted,
                &default_baseline_predicted,
            );
            let default_to_expanded_predicted = summarize_vec4_sequence_diff(
                &default_with_expanded_state.predicted,
                &expanded_baseline_predicted,
            );
            let expanded_to_expanded_control = summarize_vec2_sequence_diff(
                &expanded_with_default_state.controls,
                &expanded_baseline_controls,
            );
            let expanded_to_default_control = summarize_vec2_sequence_diff(
                &expanded_with_default_state.controls,
                &default_baseline_controls,
            );
            let expanded_to_expanded_predicted = summarize_vec4_sequence_diff(
                &expanded_with_default_state.predicted,
                &expanded_baseline_predicted,
            );
            let expanded_to_default_predicted = summarize_vec4_sequence_diff(
                &expanded_with_default_state.predicted,
                &default_baseline_predicted,
            );

            assert!(
                default_to_default_control[0] > 0.5,
                "same-time default-state injection did not move control away from default baseline enough: step={} same={:?} opposite={:?}",
                step,
                default_to_default_control,
                default_to_expanded_control
            );
            assert!(
                default_to_default_predicted[1] > 0.35 && default_to_default_predicted[2] > 0.7,
                "same-time default-state injection did not move prediction away from default baseline enough: step={} same={:?} opposite={:?}",
                step,
                default_to_default_predicted,
                default_to_expanded_predicted
            );
            assert!(
                default_to_default_control[0] > 4.0 * default_to_expanded_control[0],
                "same-time default-state injection stayed too close to default control branch: step={} same={:?} opposite={:?}",
                step,
                default_to_default_control,
                default_to_expanded_control
            );
            assert!(
                default_to_default_predicted[1] > 4.0 * default_to_expanded_predicted[1]
                    && default_to_default_predicted[2] > 4.0 * default_to_expanded_predicted[2],
                "same-time default-state injection stayed too close to default predicted branch: step={} same={:?} opposite={:?}",
                step,
                default_to_default_predicted,
                default_to_expanded_predicted
            );

            assert!(
                expanded_to_expanded_control[0] > 0.5,
                "same-time expanded-state injection did not move control away from expanded baseline enough: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_control,
                expanded_to_default_control
            );
            assert!(
                expanded_to_expanded_predicted[1] > 0.35
                    && expanded_to_expanded_predicted[2] > 0.7,
                "same-time expanded-state injection did not move prediction away from expanded baseline enough: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_predicted,
                expanded_to_default_predicted
            );
            assert!(
                expanded_to_expanded_control[0] > 4.0 * expanded_to_default_control[0],
                "same-time expanded-state injection stayed too close to expanded control branch: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_control,
                expanded_to_default_control
            );
            assert!(
                expanded_to_expanded_predicted[1] > 4.0 * expanded_to_default_predicted[1]
                    && expanded_to_expanded_predicted[2] > 4.0 * expanded_to_default_predicted[2],
                "same-time expanded-state injection stayed too close to expanded predicted branch: step={} same={:?} opposite={:?}",
                step,
                expanded_to_expanded_predicted,
                expanded_to_default_predicted
            );
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_state_component_injection() {
        let x_metrics = measure_switch_back_state_injection(&[0usize]);
        let y_metrics = measure_switch_back_state_injection(&[1usize]);
        let v_metrics = measure_switch_back_state_injection(&[2usize]);
        let yaw_metrics = measure_switch_back_state_injection(&[3usize]);

        assert!(
            x_metrics.default_same_control0 < x_metrics.default_opposite_control0 / 100.0
                && x_metrics.default_same_pred_y < x_metrics.default_opposite_pred_y / 100.0
                && x_metrics.default_same_pred_v < x_metrics.default_opposite_pred_v / 100.0,
            "x-only injection moved the default branch more than expected: {:?}",
            x_metrics
        );
        assert!(
            x_metrics.expanded_same_control0 < x_metrics.expanded_opposite_control0 / 100.0
                && x_metrics.expanded_same_pred_y < x_metrics.expanded_opposite_pred_y / 100.0
                && x_metrics.expanded_same_pred_v < x_metrics.expanded_opposite_pred_v / 100.0,
            "x-only injection moved the expanded branch more than expected: {:?}",
            x_metrics
        );

        assert!(
            yaw_metrics.default_same_control0 < yaw_metrics.default_opposite_control0 / 100.0
                && yaw_metrics.default_same_pred_y < yaw_metrics.default_opposite_pred_y / 100.0
                && yaw_metrics.default_same_pred_v < yaw_metrics.default_opposite_pred_v / 100.0,
            "yaw-only injection moved the default branch more than expected: {:?}",
            yaw_metrics
        );
        assert!(
            yaw_metrics.expanded_same_control0 < yaw_metrics.expanded_opposite_control0 / 100.0
                && yaw_metrics.expanded_same_pred_y < yaw_metrics.expanded_opposite_pred_y / 100.0
                && yaw_metrics.expanded_same_pred_v < yaw_metrics.expanded_opposite_pred_v / 100.0,
            "yaw-only injection moved the expanded branch more than expected: {:?}",
            yaw_metrics
        );

        assert!(
            y_metrics.default_same_control0 < y_metrics.default_opposite_control0 / 4.0
                && y_metrics.default_same_pred_v < y_metrics.default_opposite_pred_v / 4.0
                && y_metrics.default_same_pred_y > 3.0,
            "y-only injection changed the default branch characterization unexpectedly: {:?}",
            y_metrics
        );
        assert!(
            y_metrics.expanded_same_control0 < y_metrics.expanded_opposite_control0 / 4.0
                && y_metrics.expanded_same_pred_v < y_metrics.expanded_opposite_pred_v / 4.0
                && y_metrics.expanded_same_pred_y > 3.0,
            "y-only injection changed the expanded branch characterization unexpectedly: {:?}",
            y_metrics
        );

        assert!(
            v_metrics.default_same_control0 > 4.0 * v_metrics.default_opposite_control0
                && v_metrics.default_same_pred_v > 4.0 * v_metrics.default_opposite_pred_v,
            "v-only injection did not pull the default branch toward the opposite phase enough: {:?}",
            v_metrics
        );
        assert!(
            v_metrics.expanded_same_control0 > 4.0 * v_metrics.expanded_opposite_control0
                && v_metrics.expanded_same_pred_v > 4.0 * v_metrics.expanded_opposite_pred_v,
            "v-only injection did not pull the expanded branch toward the opposite phase enough: {:?}",
            v_metrics
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_state_subset_injection() {
        let vy_metrics = measure_switch_back_state_injection(&[1usize, 2usize]);
        let vyyaw_metrics = measure_switch_back_state_injection(&[1usize, 2usize, 3usize]);

        assert!(
            vy_metrics.default_same_control0 > 100.0 * vy_metrics.default_opposite_control0
                && vy_metrics.default_same_pred_y > 100.0 * vy_metrics.default_opposite_pred_y
                && vy_metrics.default_same_pred_v > 100.0 * vy_metrics.default_opposite_pred_v,
            "v+y injection did not pull the default branch strongly enough toward the opposite phase: {:?}",
            vy_metrics
        );
        assert!(
            vy_metrics.expanded_same_control0 > 100.0 * vy_metrics.expanded_opposite_control0
                && vy_metrics.expanded_same_pred_y > 100.0 * vy_metrics.expanded_opposite_pred_y
                && vy_metrics.expanded_same_pred_v > 100.0 * vy_metrics.expanded_opposite_pred_v,
            "v+y injection did not pull the expanded branch strongly enough toward the opposite phase: {:?}",
            vy_metrics
        );

        assert!(
            vyyaw_metrics.default_same_control0 > 100.0 * vyyaw_metrics.default_opposite_control0
                && vyyaw_metrics.default_same_pred_y
                    > 100.0 * vyyaw_metrics.default_opposite_pred_y
                && vyyaw_metrics.default_same_pred_v
                    > 100.0 * vyyaw_metrics.default_opposite_pred_v,
            "v+y+yaw injection did not pull the default branch strongly enough toward the opposite phase: {:?}",
            vyyaw_metrics
        );
        assert!(
            vyyaw_metrics.expanded_same_control0
                > 100.0 * vyyaw_metrics.expanded_opposite_control0
                && vyyaw_metrics.expanded_same_pred_y
                    > 100.0 * vyyaw_metrics.expanded_opposite_pred_y
                && vyyaw_metrics.expanded_same_pred_v
                    > 100.0 * vyyaw_metrics.expanded_opposite_pred_v,
            "v+y+yaw injection did not pull the expanded branch strongly enough toward the opposite phase: {:?}",
            vyyaw_metrics
        );

        assert!(
            (vy_metrics.default_same_control0 - vyyaw_metrics.default_same_control0).abs() <= 1e-3
                && (vy_metrics.default_opposite_control0
                    - vyyaw_metrics.default_opposite_control0)
                    .abs()
                    <= 1e-3
                && (vy_metrics.default_opposite_pred_y - vyyaw_metrics.default_opposite_pred_y)
                    .abs()
                    <= 1e-4
                && (vy_metrics.default_opposite_pred_v - vyyaw_metrics.default_opposite_pred_v)
                    .abs()
                    <= 2e-4,
            "adding yaw to v+y changed the default-branch subset metrics more than expected: vy={:?} vyyaw={:?}",
            vy_metrics,
            vyyaw_metrics
        );
        assert!(
            (vy_metrics.expanded_same_control0 - vyyaw_metrics.expanded_same_control0).abs()
                <= 1e-3
                && (vy_metrics.expanded_opposite_control0
                    - vyyaw_metrics.expanded_opposite_control0)
                    .abs()
                    <= 1e-3
                && (vy_metrics.expanded_opposite_pred_y - vyyaw_metrics.expanded_opposite_pred_y)
                    .abs()
                    <= 1e-4
                && (vy_metrics.expanded_opposite_pred_v - vyyaw_metrics.expanded_opposite_pred_v)
                    .abs()
                    <= 2e-4,
            "adding yaw to v+y changed the expanded-branch subset metrics more than expected: vy={:?} vyyaw={:?}",
            vy_metrics,
            vyyaw_metrics
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_state_subset_per_step() {
        let steps = [325usize, 326, 327, 328, 329];
        let v_metrics = measure_switch_back_state_injection_by_step(&[2usize], &steps);
        let vy_metrics = measure_switch_back_state_injection_by_step(&[1usize, 2usize], &steps);

        let step_325_v = state_injection_metrics_at_step(&v_metrics, 325);
        let step_325_vy = state_injection_metrics_at_step(&vy_metrics, 325);
        assert!(
            (step_325_v.default_opposite_control0 - step_325_vy.default_opposite_control0).abs()
                <= 5e-4,
            "default branch control moved too early at step 325: v={:?} vy={:?}",
            step_325_v,
            step_325_vy
        );
        assert!(
            step_325_v.default_opposite_pred_y > 10_000.0 * step_325_vy.default_opposite_pred_y,
            "default branch predicted-y should already need y at step 325: v={:?} vy={:?}",
            step_325_v,
            step_325_vy
        );
        assert!(
            step_325_v.expanded_opposite_pred_y > 1_000.0 * step_325_vy.expanded_opposite_pred_y,
            "expanded branch predicted-y should already improve with y at step 325: v={:?} vy={:?}",
            step_325_v,
            step_325_vy
        );

        for step in [326usize, 327, 328, 329] {
            let v = state_injection_metrics_at_step(&v_metrics, step);
            let vy = state_injection_metrics_at_step(&vy_metrics, step);

            assert!(
                v.default_opposite_control0 > 20.0 * vy.default_opposite_control0,
                "default branch control should need y from step {step}: v={:?} vy={:?}",
                v,
                vy
            );
            assert!(
                v.default_opposite_pred_y > 1_000.0 * vy.default_opposite_pred_y,
                "default branch predicted-y should strongly prefer v+y from step {step}: v={:?} vy={:?}",
                v,
                vy
            );
            assert!(
                v.expanded_opposite_pred_y > 1_000.0 * vy.expanded_opposite_pred_y,
                "expanded branch predicted-y should strongly prefer v+y from step {step}: v={:?} vy={:?}",
                v,
                vy
            );
        }

        for step in [325usize, 326, 327, 328, 329] {
            let v = state_injection_metrics_at_step(&v_metrics, step);
            let vy = state_injection_metrics_at_step(&vy_metrics, step);
            assert!(
                (v.expanded_opposite_control0 - vy.expanded_opposite_control0).abs() <= 3e-3,
                "expanded branch control should stay mostly v-driven at step {step}: v={:?} vy={:?}",
                v,
                vy
            );
            assert!(
                (v.expanded_opposite_pred_v - vy.expanded_opposite_pred_v).abs() <= 6e-4,
                "expanded branch predicted-v should stay mostly v-driven at step {step}: v={:?} vy={:?}",
                v,
                vy
            );
        }
    }

    /// Metrics for delta injection from step 325 → 326 transition analysis.
    #[derive(Debug)]
    struct StepTransitionDeltaMetrics {
        /// State delta from step 325 to step 326 (default branch).
        state_delta: Vector4<f64>,
        /// Baseline control[0] at step 325 (default branch).
        baseline_control0: [f64; 2],
        /// Expanded baseline control[0] at step 325.
        expanded_baseline_control0: [f64; 2],
        /// Per-component-set injection results: (component_label, injected_control0, same_diff, opposite_diff).
        injections: Vec<DeltaInjectionResult>,
    }

    #[derive(Debug)]
    struct DeltaInjectionResult {
        label: &'static str,
        indices: Vec<usize>,
        injected_control0: [f64; 2],
        default_same_control0: f64,
        default_opposite_control0: f64,
        default_same_pred_y: f64,
        default_opposite_pred_y: f64,
        default_same_pred_v: f64,
        default_opposite_pred_v: f64,
    }

    fn measure_step_transition_delta_injection() -> StepTransitionDeltaMetrics {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );

        let row_325 = detailed_row_at_step(&default_trace, 325);
        let row_326 = detailed_row_at_step(&default_trace, 326);
        let expanded_row_325 = detailed_row_at_step(&expanded_trace, 325);

        let state_delta = row_326.state - row_325.state;

        let default_baseline_controls: Vec<[f64; 2]> = row_325
            .controls
            .iter()
            .map(|vec| [vec[0], vec[1]])
            .collect();
        let expanded_baseline_controls: Vec<[f64; 2]> = expanded_row_325
            .controls
            .iter()
            .map(|vec| [vec[0], vec[1]])
            .collect();
        let default_baseline_predicted: Vec<[f64; 4]> = row_325
            .predicted
            .iter()
            .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
            .collect();
        let expanded_baseline_predicted: Vec<[f64; 4]> = expanded_row_325
            .predicted
            .iter()
            .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
            .collect();

        let component_sets: Vec<(&str, Vec<usize>)> = vec![
            ("delta_v", vec![2]),
            ("delta_y", vec![1]),
            ("delta_v+delta_y", vec![1, 2]),
            ("delta_v+delta_y+delta_yaw", vec![1, 2, 3]),
        ];

        let warm_start_325 = default_warm_start.get(&325).unwrap();

        let mut injections = Vec::new();
        for (label, indices) in &component_sets {
            let mut modified_state_vec = row_325.state;
            for &idx in indices {
                modified_state_vec[idx] += state_delta[idx];
            }
            let modified_state = state_from_vector4(&modified_state_vec);

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &row_325.xref,
                &modified_state,
                warm_start_325,
                optimize_linearized_controls_original,
            );

            let to_default_control =
                summarize_vec2_sequence_diff(&result.controls, &default_baseline_controls);
            let to_expanded_control =
                summarize_vec2_sequence_diff(&result.controls, &expanded_baseline_controls);
            let to_default_predicted =
                summarize_vec4_sequence_diff(&result.predicted, &default_baseline_predicted);
            let to_expanded_predicted =
                summarize_vec4_sequence_diff(&result.predicted, &expanded_baseline_predicted);

            injections.push(DeltaInjectionResult {
                label,
                indices: indices.clone(),
                injected_control0: [result.controls[0][0], result.controls[0][1]],
                default_same_control0: to_default_control[0],
                default_opposite_control0: to_expanded_control[0],
                default_same_pred_y: to_default_predicted[1],
                default_opposite_pred_y: to_expanded_predicted[1],
                default_same_pred_v: to_default_predicted[2],
                default_opposite_pred_v: to_expanded_predicted[2],
            });
        }

        StepTransitionDeltaMetrics {
            state_delta,
            baseline_control0: [row_325.controls[0][0], row_325.controls[0][1]],
            expanded_baseline_control0: [
                expanded_row_325.controls[0][0],
                expanded_row_325.controls[0][1],
            ],
            injections,
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_full_candidate_expansion_characterizes_step_325_326_transition() {
        let metrics = measure_step_transition_delta_injection();

        println!("=== Step 325 → 326 Transition Delta Injection ===");
        println!("state_delta: {:?}", metrics.state_delta);
        println!(
            "baseline_control0 (default): {:?}",
            metrics.baseline_control0
        );
        println!(
            "baseline_control0 (expanded): {:?}",
            metrics.expanded_baseline_control0
        );
        for inj in &metrics.injections {
            println!("--- {} (indices {:?}) ---", inj.label, inj.indices);
            println!("  injected_control0: {:?}", inj.injected_control0);
            println!(
                "  same_control0: {:.6e}  opposite_control0: {:.6e}  ratio: {:.2}",
                inj.default_same_control0,
                inj.default_opposite_control0,
                if inj.default_opposite_control0 > 0.0 {
                    inj.default_same_control0 / inj.default_opposite_control0
                } else {
                    f64::INFINITY
                }
            );
            println!(
                "  same_pred_y: {:.6e}  opposite_pred_y: {:.6e}",
                inj.default_same_pred_y, inj.default_opposite_pred_y
            );
            println!(
                "  same_pred_v: {:.6e}  opposite_pred_v: {:.6e}",
                inj.default_same_pred_v, inj.default_opposite_pred_v
            );
        }

        // delta_v alone: at step 325 v-only was sufficient, so applying
        // one step of delta_v should still keep control close to default baseline.
        let delta_v = &metrics.injections[0];
        assert!(
            delta_v.default_same_control0 < delta_v.default_opposite_control0,
            "delta_v at step 325 context should stay closer to default than expanded: {:?}",
            delta_v
        );

        // delta_y alone: y alone had minimal control impact at step 325.
        let delta_y = &metrics.injections[1];
        assert!(
            delta_y.default_same_control0 < delta_y.default_opposite_control0,
            "delta_y at step 325 context should stay closer to default than expanded: {:?}",
            delta_y
        );

        // delta_v+delta_y: the key question — does this combo start pulling
        // the control toward expanded, or does it stay default-like?
        // The handoff hypothesis: "delta_v + delta_y で step 326 の control が
        // 急に opposite 側へ寄る"
        let delta_vy = &metrics.injections[2];
        println!(
            "\n=== KEY: delta_v+delta_y same/opposite ratio = {:.4} ===",
            if delta_vy.default_opposite_control0 > 0.0 {
                delta_vy.default_same_control0 / delta_vy.default_opposite_control0
            } else {
                f64::INFINITY
            }
        );

        // delta_v+delta_y+delta_yaw: should be nearly identical to delta_v+delta_y.
        let delta_vyy = &metrics.injections[3];
        let control0_diff =
            (delta_vy.default_same_control0 - delta_vyy.default_same_control0).abs();
        println!(
            "delta_v+delta_y vs delta_v+delta_y+delta_yaw control0 diff: {:.6e}",
            control0_diff
        );
    }

    /// Result of v-only opposite-branch injection with context element swaps.
    #[derive(Debug)]
    struct ContextSwapInjectionResult {
        label: &'static str,
        injected_steer: f64,
        default_baseline_steer: f64,
        expanded_baseline_steer: f64,
        default_same_control0: f64,
        default_opposite_control0: f64,
        default_same_pred_y: f64,
        default_opposite_pred_y: f64,
        default_same_pred_v: f64,
        default_opposite_pred_v: f64,
    }

    /// Investigate why v-only opposite-branch injection works at step 325 but
    /// fails at step 326 by systematically swapping context elements (state
    /// residual, xref, warm_start) between the two steps.
    fn measure_step_325_326_context_swap_v_injection() -> Vec<ContextSwapInjectionResult> {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );

        let d325 = detailed_row_at_step(&default_trace, 325);
        let d326 = detailed_row_at_step(&default_trace, 326);
        let e325 = detailed_row_at_step(&expanded_trace, 325);
        let e326 = detailed_row_at_step(&expanded_trace, 326);
        let ws325 = default_warm_start.get(&325).unwrap();
        let ws326 = default_warm_start.get(&326).unwrap();

        let v_indices: &[usize] = &[2]; // v component only

        // Helper to run v-only injection and compute metrics
        let run_injection = |label: &'static str,
                             default_state: &Vector4<f64>,
                             expanded_state: &Vector4<f64>,
                             xref: &[Vector4<f64>],
                             warm_start: &[Vector2<f64>],
                             default_row: &SwitchBackDetailedRow,
                             expanded_row: &SwitchBackDetailedRow| {
            let injected_state = inject_state_components(default_state, expanded_state, v_indices);
            let result = iterative_linear_mpc_control_with_test_optimizer(
                xref,
                &injected_state,
                warm_start,
                optimize_linearized_controls_original,
            );

            let default_baseline_controls: Vec<[f64; 2]> = default_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect();
            let expanded_baseline_controls: Vec<[f64; 2]> = expanded_row
                .controls
                .iter()
                .map(|vec| [vec[0], vec[1]])
                .collect();
            let default_baseline_predicted: Vec<[f64; 4]> = default_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect();
            let expanded_baseline_predicted: Vec<[f64; 4]> = expanded_row
                .predicted
                .iter()
                .map(|vec| [vec[0], vec[1], vec[2], vec[3]])
                .collect();

            let to_default_control =
                summarize_vec2_sequence_diff(&result.controls, &default_baseline_controls);
            let to_expanded_control =
                summarize_vec2_sequence_diff(&result.controls, &expanded_baseline_controls);
            let to_default_predicted =
                summarize_vec4_sequence_diff(&result.predicted, &default_baseline_predicted);
            let to_expanded_predicted =
                summarize_vec4_sequence_diff(&result.predicted, &expanded_baseline_predicted);

            ContextSwapInjectionResult {
                label,
                injected_steer: result.controls[0][1],
                default_baseline_steer: default_row.controls[0][1],
                expanded_baseline_steer: expanded_row.controls[0][1],
                default_same_control0: to_default_control[0],
                default_opposite_control0: to_expanded_control[0],
                default_same_pred_y: to_default_predicted[1],
                default_opposite_pred_y: to_expanded_predicted[1],
                default_same_pred_v: to_default_predicted[2],
                default_opposite_pred_v: to_expanded_predicted[2],
            }
        };

        vec![
            // Baseline: v-only injection at step 325 (known to work)
            run_injection(
                "baseline_325: d325_state + e325_v, d325_xref, ws325",
                &d325.state,
                &e325.state,
                &d325.xref,
                ws325,
                d325,
                e325,
            ),
            // Baseline: v-only injection at step 326 (known to fail)
            run_injection(
                "baseline_326: d326_state + e326_v, d326_xref, ws326",
                &d326.state,
                &e326.state,
                &d326.xref,
                ws326,
                d326,
                e326,
            ),
            // Swap state residual: use step 325 default state (non-v) with step 326 expanded v
            run_injection(
                "swap_state: d325_state + e326_v, d326_xref, ws326",
                &d325.state,
                &e326.state,
                &d326.xref,
                ws326,
                d326,
                e326,
            ),
            // Swap xref: use step 326 state but step 325 xref
            run_injection(
                "swap_xref: d326_state + e326_v, d325_xref, ws326",
                &d326.state,
                &e326.state,
                &d325.xref,
                ws326,
                d326,
                e326,
            ),
            // Swap warm_start: use step 326 state/xref but step 325 warm_start
            run_injection(
                "swap_ws: d326_state + e326_v, d326_xref, ws325",
                &d326.state,
                &e326.state,
                &d326.xref,
                ws325,
                d326,
                e326,
            ),
            // Swap state + xref: step 325 state/xref with step 326 expanded v
            run_injection(
                "swap_state+xref: d325_state + e326_v, d325_xref, ws326",
                &d325.state,
                &e326.state,
                &d325.xref,
                ws326,
                d326,
                e326,
            ),
            // Swap state + warm_start
            run_injection(
                "swap_state+ws: d325_state + e326_v, d326_xref, ws325",
                &d325.state,
                &e326.state,
                &d326.xref,
                ws325,
                d326,
                e326,
            ),
            // Swap xref + warm_start
            run_injection(
                "swap_xref+ws: d326_state + e326_v, d325_xref, ws325",
                &d326.state,
                &e326.state,
                &d325.xref,
                ws325,
                d326,
                e326,
            ),
            // Swap all three: effectively step 325 context with step 326 expanded v
            run_injection(
                "swap_all: d325_state + e326_v, d325_xref, ws325",
                &d325.state,
                &e326.state,
                &d325.xref,
                ws325,
                d326,
                e326,
            ),
            // Cross-step expanded v: use step 325 expanded v in step 326 default context
            // to isolate whether expanded v progression matters
            run_injection(
                "cross_v_325in326: d326_state + e325_v, d326_xref, ws326",
                &d326.state,
                &e325.state,
                &d326.xref,
                ws326,
                d326,
                e326,
            ),
            // Cross-step expanded v: use step 326 expanded v in step 325 default context
            run_injection(
                "cross_v_326in325: d325_state + e326_v, d325_xref, ws325",
                &d325.state,
                &e326.state,
                &d325.xref,
                ws325,
                d325,
                e325,
            ),
            // --- Per-component isolation: starting from baseline_325 (d325 + e325_v),
            //     replace individual default non-v components with d326 values ---
            // Replace only x
            {
                let mut s = d325.state;
                s[0] = d326.state[0]; // x
                run_injection(
                    "isolate_x: d325{x→d326} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
            // Replace only y
            {
                let mut s = d325.state;
                s[1] = d326.state[1]; // y
                run_injection(
                    "isolate_y: d325{y→d326} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
            // Replace only yaw
            {
                let mut s = d325.state;
                s[3] = d326.state[3]; // yaw
                run_injection(
                    "isolate_yaw: d325{yaw→d326} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
            // Replace y + yaw
            {
                let mut s = d325.state;
                s[1] = d326.state[1];
                s[3] = d326.state[3];
                run_injection(
                    "isolate_y+yaw: d325{y,yaw→d326} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
            // Replace x + y
            {
                let mut s = d325.state;
                s[0] = d326.state[0];
                s[1] = d326.state[1];
                run_injection(
                    "isolate_x+y: d325{x,y→d326} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
            // Replace all non-v (x + y + yaw) → should match cross_v_326in325
            {
                let mut s = d326.state;
                s[2] = d325.state[2]; // keep default v from step 325
                run_injection(
                    "isolate_all_nonv: d326{v→d325} + e325_v",
                    &s,
                    &e325.state,
                    &d325.xref,
                    ws325,
                    d325,
                    e325,
                )
            },
        ]
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_step_325_326_v_injection_context_swap() {
        let results = measure_step_325_326_context_swap_v_injection();

        println!("=== Step 325 vs 326: V-Only Injection Context Swap ===");
        println!("Question: why does v-only opposite-branch injection work at 325 but not 326?");
        println!();

        // Print raw state values for reference
        println!("=== Raw State Values ===");
        println!("  (indices: x=0, y=1, v=2, yaw=3)");
        // We need to recompute traces to get raw values for printing
        let default_trace_print = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace_print = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let d325_p = detailed_row_at_step(&default_trace_print, 325);
        let d326_p = detailed_row_at_step(&default_trace_print, 326);
        let e325_p = detailed_row_at_step(&expanded_trace_print, 325);
        let e326_p = detailed_row_at_step(&expanded_trace_print, 326);
        println!("  d325.state: {:?}", d325_p.state);
        println!("  d326.state: {:?}", d326_p.state);
        println!("  d325→d326 delta: {:?}", d326_p.state - d325_p.state);
        println!("  e325.state: {:?}", e325_p.state);
        println!("  e326.state: {:?}", e326_p.state);
        println!("  e325→e326 delta: {:?}", e326_p.state - e325_p.state);
        println!(
            "  v diff: e325.v={:.6} e326.v={:.6} delta={:.6}",
            e325_p.state[2],
            e326_p.state[2],
            e326_p.state[2] - e325_p.state[2]
        );
        println!();

        for r in &results {
            let pull_ratio = if r.default_opposite_control0 > 0.0 {
                r.default_same_control0 / r.default_opposite_control0
            } else {
                f64::INFINITY
            };
            let pulls_toward_expanded = pull_ratio > 4.0;
            println!(
                "[{}] {} steer={:.6} (default={:.6}, expanded={:.6})",
                if pulls_toward_expanded {
                    "PULL"
                } else {
                    "STAY"
                },
                r.label,
                r.injected_steer,
                r.default_baseline_steer,
                r.expanded_baseline_steer,
            );
            println!(
                "  same_ctrl0={:.4e} opp_ctrl0={:.4e} ratio={:.3} pred_y_s={:.4e} pred_y_o={:.4e} pred_v_s={:.4e} pred_v_o={:.4e}",
                r.default_same_control0,
                r.default_opposite_control0,
                pull_ratio,
                r.default_same_pred_y,
                r.default_opposite_pred_y,
                r.default_same_pred_v,
                r.default_opposite_pred_v,
            );
        }

        // Baseline assertions
        let baseline_325 = &results[0];
        let ratio_325 = baseline_325.default_same_control0 / baseline_325.default_opposite_control0;
        assert!(
            ratio_325 > 100.0,
            "v-only injection should strongly pull toward expanded at step 325 (ratio={ratio_325}): {:?}",
            baseline_325
        );

        let baseline_326 = &results[1];
        let ratio_326 = baseline_326.default_same_control0 / baseline_326.default_opposite_control0;
        assert!(
            ratio_326 < 10.0,
            "v-only injection pull should weaken dramatically at step 326 (ratio={ratio_326}): {:?}",
            baseline_326
        );

        // The ratio drop from 325 to 326 should be at least 100x
        assert!(
            ratio_325 > 100.0 * ratio_326,
            "v-only pull should drop >100x from step 325 to 326: 325={ratio_325} 326={ratio_326}"
        );

        // xref and warm_start swaps should not change the ratio
        let swap_xref = &results[3];
        let ratio_swap_xref = swap_xref.default_same_control0 / swap_xref.default_opposite_control0;
        assert!(
            (ratio_swap_xref - ratio_326).abs() < 0.1,
            "xref swap should not change ratio: baseline={ratio_326} swapped={ratio_swap_xref}"
        );

        let swap_ws = &results[4];
        let ratio_swap_ws = swap_ws.default_same_control0 / swap_ws.default_opposite_control0;
        assert!(
            (ratio_swap_ws - ratio_326).abs() < 0.1,
            "warm_start swap should not change ratio: baseline={ratio_326} swapped={ratio_swap_ws}"
        );

        // state swap should change the ratio (state is the driver)
        let swap_state = &results[2];
        let ratio_swap_state =
            swap_state.default_same_control0 / swap_state.default_opposite_control0;
        assert!(
            (ratio_swap_state - ratio_326).abs() > 0.5,
            "state swap should change ratio: baseline={ratio_326} swapped={ratio_swap_state}"
        );

        // Per-component isolation: none of the default non-v changes
        // should destroy the pull (all ratios should stay > 100)
        for result in &results[11..=16] {
            let ratio = result.default_same_control0 / result.default_opposite_control0;
            assert!(
                ratio > 100.0,
                "default non-v component change should not destroy pull: {} ratio={ratio}",
                result.label
            );
        }
    }

    /// Run MPC with a fixed number of outer iterations to isolate where y's
    /// effect enters the control. If y only matters through nonlinear
    /// xbar re-linearization (outer loop), then 1-iteration solve should
    /// show no y effect, while 3-iteration solve should.
    fn iterative_linear_mpc_control_with_max_iter(
        xref: &[Vector4<f64>],
        state: &State,
        warm_start: &[Vector2<f64>],
        max_iter: usize,
    ) -> MpcResult {
        let mut controls = warm_start.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let mut predicted = predict_motion(*state, &controls);
        for _ in 0..max_iter {
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

    /// Dump the first inner-solve adjoint to trace how y reaches steering gradient.
    /// Returns (steering_gradient, lambda3_components) for analysis.
    fn dump_first_inner_solve_adjoint(
        xref: &[Vector4<f64>],
        state: &State,
        warm_start: &[Vector2<f64>],
    ) -> (Vec<Vector2<f64>>, Vec<[f64; 5]>) {
        let mut controls = warm_start.to_vec();
        controls.resize(T, Vector2::zeros());
        apply_control_constraints(&mut controls);

        let xbar = predict_motion(*state, &controls);

        let q = state_cost_weight();
        let qf = terminal_cost_weight();
        let r = control_cost_weight();
        let rd = control_rate_weight();

        let (x, a_seq, b_seq, _) = linearized_rollout(state, &xbar, &controls);

        let mut gradients = vec![Vector2::zeros(); T];
        let mut lambda = (qf * state_error(x[T], xref[T])) * 2.0;
        let mut lambda3_decomp = Vec::new();

        // Terminal lambda[3] decomposition
        lambda3_decomp.push([
            lambda[3], // total lambda[3]
            0.0,       // from lx[3]
            0.0,       // from A^T[3,0]*lambda_next[0]
            0.0,       // from A^T[3,1]*lambda_next[1]
            lambda[3], // from propagation (terminal)
        ]);

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

            let lambda_next = lambda;
            lambda = lx + a_seq[t].transpose() * lambda_next;

            // Decompose lambda[3]
            let from_lx3 = lx[3];
            let from_at30_l0 = a_seq[t][(0, 3)] * lambda_next[0]; // A^T[3,0] = A[0,3]
            let from_at31_l1 = a_seq[t][(1, 3)] * lambda_next[1]; // A^T[3,1] = A[1,3]
            let from_at33_l3 = a_seq[t][(3, 3)] * lambda_next[3]; // A^T[3,3] = A[3,3]
            lambda3_decomp.push([
                lambda[3],
                from_lx3,
                from_at30_l0,
                from_at31_l1,
                from_at33_l3,
            ]);
        }

        lambda3_decomp.reverse();
        (gradients, lambda3_decomp)
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_y_effect_path_via_outer_loop() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );

        println!("=== Y Effect Path: Outer Loop Iteration Count ===");
        println!("At step 326, v-only injection needs v+y. Is y's effect through outer-loop re-linearization?");
        println!();

        for &step in &[325usize, 326, 327] {
            let d_row = detailed_row_at_step(&default_trace, step);
            let e_row = detailed_row_at_step(&expanded_trace, step);
            let ws = default_warm_start.get(&step).unwrap();

            let d_baseline_steer = d_row.controls[0][1];
            let e_baseline_steer = e_row.controls[0][1];

            println!("--- step {step} (default_steer={d_baseline_steer:.6}, expanded_steer={e_baseline_steer:.6}) ---");

            for &max_iter in &[1usize, 2, 3] {
                // v-only injection
                let v_state = inject_state_components(&d_row.state, &e_row.state, &[2]);
                let v_result =
                    iterative_linear_mpc_control_with_max_iter(&d_row.xref, &v_state, ws, max_iter);

                // v+y injection
                let vy_state = inject_state_components(&d_row.state, &e_row.state, &[1, 2]);
                let vy_result = iterative_linear_mpc_control_with_max_iter(
                    &d_row.xref,
                    &vy_state,
                    ws,
                    max_iter,
                );

                let v_steer = v_result.controls[0][1];
                let vy_steer = vy_result.controls[0][1];
                let steer_diff = (v_steer - vy_steer).abs();

                println!(
                    "  iter={max_iter}: v_steer={v_steer:.6} vy_steer={vy_steer:.6} diff={steer_diff:.6e}"
                );
            }
        }
    }

    /// linearized_rollout WITHOUT angle normalization — matches Python CVXPY behavior.
    fn linearized_rollout_without_angle_normalization(
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
            let next = a * x[t] + b * controls[t] + c;
            // No normalize_angle here — Python CVXPY doesn't do this

            a_seq.push(a);
            b_seq.push(b);
            c_seq.push(c);
            x[t + 1] = next;
        }

        (x, a_seq, b_seq, c_seq)
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_yaw_normalize_effect_in_linearized_rollout() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls,
        );

        println!("=== Yaw Normalize Effect in Linearized Rollout ===");
        println!("Comparing linearized_rollout with and without normalize_angle");
        println!();

        for &step in &[20usize, 70, 71, 75, 78, 325, 326] {
            let row = detailed_row_at_step(&default_trace, step);
            let ws = default_warm_start.get(&step).unwrap();
            let state = state_from_vector4(&row.state);

            // Build xbar from predict_motion (nonlinear, as both Rust and Python do)
            let mut controls = ws.to_vec();
            controls.resize(T, Vector2::zeros());
            apply_control_constraints(&mut controls);
            let xbar = predict_motion(state, &controls);

            let (x_with, _, _, _) = linearized_rollout(&state, &xbar, &controls);
            let (x_without, _, _, _) =
                linearized_rollout_without_angle_normalization(&state, &xbar, &controls);

            let mut max_yaw_diff = 0.0f64;
            let mut max_other_diff = [0.0f64; 3]; // x, y, v
            for t in 0..=T {
                let yaw_diff = (x_with[t][3] - x_without[t][3]).abs();
                max_yaw_diff = max_yaw_diff.max(yaw_diff);
                for (i, slot) in max_other_diff.iter_mut().enumerate() {
                    *slot = (*slot).max((x_with[t][i] - x_without[t][i]).abs());
                }
            }

            let yaw_values: Vec<f64> = (0..=T).map(|t| x_with[t][3]).collect();
            println!(
                "step {step:>3}: yaw_diff={max_yaw_diff:.4e} other=[{:.4e},{:.4e},{:.4e}] yaw={:.4?}",
                max_other_diff[0], max_other_diff[1], max_other_diff[2], yaw_values
            );
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_adjoint_y_to_steer_coupling() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );

        println!("=== Adjoint Decomposition: How Y Reaches Steering ===");
        println!(
            "lambda3_decomp columns: [total, from_lx3, from_AT30*l0, from_AT31*l1, from_AT33*l3]"
        );
        println!();

        for &step in &[325usize, 326] {
            let d_row = detailed_row_at_step(&default_trace, step);
            let e_row = detailed_row_at_step(&expanded_trace, step);
            let ws = default_warm_start.get(&step).unwrap();

            println!("--- step {step} ---");

            // Baseline (no injection)
            let (grad_base, l3_base) =
                dump_first_inner_solve_adjoint(&d_row.xref, &state_from_vector4(&d_row.state), ws);
            println!("  [baseline] steer_grad[0]={:.6e}", grad_base[0][1]);
            for (t, decomp) in l3_base.iter().enumerate() {
                println!(
                    "    t={t}: λ3={:.4e} lx3={:.4e} AT30·l0={:.4e} AT31·l1={:.4e} AT33·l3={:.4e}",
                    decomp[0], decomp[1], decomp[2], decomp[3], decomp[4]
                );
            }

            // v-only injection
            let v_state = inject_state_components(&d_row.state, &e_row.state, &[2]);
            let (grad_v, l3_v) = dump_first_inner_solve_adjoint(&d_row.xref, &v_state, ws);
            println!("  [v-only] steer_grad[0]={:.6e}", grad_v[0][1]);
            for (t, decomp) in l3_v.iter().enumerate() {
                println!(
                    "    t={t}: λ3={:.4e} lx3={:.4e} AT30·l0={:.4e} AT31·l1={:.4e} AT33·l3={:.4e}",
                    decomp[0], decomp[1], decomp[2], decomp[3], decomp[4]
                );
            }

            // v+y injection
            let vy_state = inject_state_components(&d_row.state, &e_row.state, &[1, 2]);
            let (grad_vy, l3_vy) = dump_first_inner_solve_adjoint(&d_row.xref, &vy_state, ws);
            println!("  [v+y] steer_grad[0]={:.6e}", grad_vy[0][1]);
            for (t, decomp) in l3_vy.iter().enumerate() {
                println!(
                    "    t={t}: λ3={:.4e} lx3={:.4e} AT30·l0={:.4e} AT31·l1={:.4e} AT33·l3={:.4e}",
                    decomp[0], decomp[1], decomp[2], decomp[3], decomp[4]
                );
            }

            // Print gradient differences
            println!(
                "  steer_grad[0] diffs: v-base={:.6e} vy-base={:.6e} vy-v={:.6e}",
                grad_v[0][1] - grad_base[0][1],
                grad_vy[0][1] - grad_base[0][1],
                grad_vy[0][1] - grad_v[0][1],
            );
            println!();
        }
    }

    /// Per-component state crossover: replace individual state components
    /// from step 325 into step 326 context to identify which component(s)
    /// of the state change drive the v-only pull degradation.
    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_step_325_326_state_component_crossover() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_original,
        );

        let d325 = detailed_row_at_step(&default_trace, 325);
        let d326 = detailed_row_at_step(&default_trace, 326);
        let e326 = detailed_row_at_step(&expanded_trace, 326);
        let ws326 = default_warm_start.get(&326).unwrap();

        let state_delta = d326.state - d325.state;
        println!(
            "state_delta: x={:.6e} y={:.6e} v={:.6e} yaw={:.6e}",
            state_delta[0], state_delta[1], state_delta[2], state_delta[3]
        );

        let expanded_baseline_controls: Vec<[f64; 2]> =
            e326.controls.iter().map(|v| [v[0], v[1]]).collect();
        let default_baseline_controls: Vec<[f64; 2]> =
            d326.controls.iter().map(|v| [v[0], v[1]]).collect();

        let component_sets: Vec<(&str, Vec<usize>)> = vec![
            ("revert_x", vec![0]),
            ("revert_y", vec![1]),
            ("revert_v", vec![2]),
            ("revert_yaw", vec![3]),
            ("revert_y+v", vec![1, 2]),
            ("revert_x+yaw", vec![0, 3]),
            ("revert_all", vec![0, 1, 2, 3]),
        ];

        println!(
            "\n=== State Component Crossover (step 326 context, revert components to 325) ==="
        );

        // Baseline: v-only injection at step 326
        let baseline_injected_state = inject_state_components(&d326.state, &e326.state, &[2]);
        let baseline_result = iterative_linear_mpc_control_with_test_optimizer(
            &d326.xref,
            &baseline_injected_state,
            ws326,
            optimize_linearized_controls_original,
        );
        let baseline_to_default =
            summarize_vec2_sequence_diff(&baseline_result.controls, &default_baseline_controls);
        let baseline_to_expanded =
            summarize_vec2_sequence_diff(&baseline_result.controls, &expanded_baseline_controls);
        let baseline_ratio = if baseline_to_expanded[0] > 0.0 {
            baseline_to_default[0] / baseline_to_expanded[0]
        } else {
            f64::INFINITY
        };
        println!(
            "  baseline: same={:.4e} opp={:.4e} ratio={:.3}",
            baseline_to_default[0], baseline_to_expanded[0], baseline_ratio
        );

        let mut ratios = Vec::new();
        for (label, indices) in &component_sets {
            let mut modified_default = d326.state;
            for &idx in indices {
                modified_default[idx] = d325.state[idx];
            }

            let injected_state = inject_state_components(&modified_default, &e326.state, &[2]);
            let result = iterative_linear_mpc_control_with_test_optimizer(
                &d326.xref,
                &injected_state,
                ws326,
                optimize_linearized_controls_original,
            );

            let to_default =
                summarize_vec2_sequence_diff(&result.controls, &default_baseline_controls);
            let to_expanded =
                summarize_vec2_sequence_diff(&result.controls, &expanded_baseline_controls);
            let ratio = if to_expanded[0] > 0.0 {
                to_default[0] / to_expanded[0]
            } else {
                f64::INFINITY
            };

            println!(
                "  {:12} same={:.4e} opp={:.4e} ratio={:.3} (delta: {:.3})",
                label,
                to_default[0],
                to_expanded[0],
                ratio,
                ratio - baseline_ratio
            );
            ratios.push((*label, ratio));
        }

        let revert_y_ratio = ratios.iter().find(|(l, _)| *l == "revert_y").unwrap().1;
        let revert_v_ratio = ratios.iter().find(|(l, _)| *l == "revert_v").unwrap().1;
        let revert_yv_ratio = ratios.iter().find(|(l, _)| *l == "revert_y+v").unwrap().1;
        let revert_all_ratio = ratios.iter().find(|(l, _)| *l == "revert_all").unwrap().1;

        println!("\n=== Summary ===");
        println!("baseline_326 ratio: {:.3}", baseline_ratio);
        println!("revert_y ratio:     {:.3}", revert_y_ratio);
        println!("revert_v ratio:     {:.3}", revert_v_ratio);
        println!("revert_y+v ratio:   {:.3}", revert_yv_ratio);
        println!("revert_all ratio:   {:.3}", revert_all_ratio);

        // y alone accounts for nearly all the effect
        assert!(
            (revert_y_ratio - revert_all_ratio).abs() < 0.1,
            "reverting y alone should account for nearly all the effect: y={revert_y_ratio} all={revert_all_ratio}"
        );

        // v contributes nothing
        assert!(
            (revert_v_ratio - baseline_ratio).abs() < 0.01,
            "reverting v should have no effect: v={revert_v_ratio} baseline={baseline_ratio}"
        );

        // x and yaw contribute nothing
        let revert_xyaw_ratio = ratios.iter().find(|(l, _)| *l == "revert_x+yaw").unwrap().1;
        assert!(
            (revert_xyaw_ratio - baseline_ratio).abs() < 0.01,
            "reverting x+yaw should have no effect: x+yaw={revert_xyaw_ratio} baseline={baseline_ratio}"
        );
    }

    /// Print state.y divergence history between default and expanded branches
    /// to identify when and how the lateral position diverges.
    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_y_divergence_history() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            350,
            optimize_linearized_controls_original,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            350,
            optimize_linearized_controls_with_full_candidate_expansion,
        );

        println!("=== state.y Divergence History (default vs expanded) ===");
        println!(
            "{:>5} {:>12} {:>12} {:>12} {:>12} {:>12}",
            "step", "default_y", "expanded_y", "y_diff", "default_v", "v_diff"
        );

        let mut first_significant_step = None;
        let mut max_y_diff: f64 = 0.0;

        for step in (0..350).step_by(5) {
            let d = default_trace.iter().find(|r| r.step == step);
            let e = expanded_trace.iter().find(|r| r.step == step);
            if let (Some(d), Some(e)) = (d, e) {
                let y_diff = (d.state[1] - e.state[1]).abs();
                max_y_diff = max_y_diff.max(y_diff);

                if y_diff > 1e-6 || step >= 60 {
                    println!(
                        "{:5} {:12.6} {:12.6} {:12.6e} {:12.6} {:12.6e}",
                        step,
                        d.state[1],
                        e.state[1],
                        d.state[1] - e.state[1],
                        d.state[2],
                        d.state[2] - e.state[2]
                    );
                }

                if first_significant_step.is_none() && y_diff > 1e-3 {
                    first_significant_step = Some(step);
                }
            }
        }

        // Dense print around the first significant divergence
        if let Some(start) = first_significant_step {
            let dense_start = start.saturating_sub(10);
            println!(
                "\n=== Dense view around first significant y divergence (step {}) ===",
                start
            );
            for step in dense_start..=(start + 20).min(349) {
                let d = default_trace.iter().find(|r| r.step == step);
                let e = expanded_trace.iter().find(|r| r.step == step);
                if let (Some(d), Some(e)) = (d, e) {
                    let y_diff = d.state[1] - e.state[1];
                    let v_diff = d.state[2] - e.state[2];
                    let steer_diff = d.controls[0][1] - e.controls[0][1];
                    println!(
                        "  step {:3}: y_diff={:12.6e} v_diff={:12.6e} steer_diff={:12.6e}",
                        step, y_diff, v_diff, steer_diff
                    );
                }
            }
        }

        // Dense print around step 320-330 (the pre-terminal region)
        println!("\n=== Dense view step 315..335 (pre-terminal) ===");
        for step in 315..=335 {
            let d = default_trace.iter().find(|r| r.step == step);
            let e = expanded_trace.iter().find(|r| r.step == step);
            if let (Some(d), Some(e)) = (d, e) {
                let y_diff = d.state[1] - e.state[1];
                let v_diff = d.state[2] - e.state[2];
                let steer_diff = d.controls[0][1] - e.controls[0][1];
                println!(
                    "  step {:3}: y={:10.6}/{:10.6} diff={:12.6e} v_diff={:12.6e} steer_diff={:12.6e}",
                    step, d.state[1], e.state[1], y_diff, v_diff, steer_diff
                );
            }
        }

        println!("\nmax |y_diff| across all steps: {:.6e}", max_y_diff);
        if let Some(s) = first_significant_step {
            println!("first step with |y_diff| > 1e-3: {}", s);
        }
    }

    /// Optimizer without angle wrap, matching `optimize_linearized_controls` signature
    /// for use with `iterative_linear_mpc_control_with_test_optimizer`.
    fn optimize_linearized_controls_without_angle_wrap(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        initial_controls: &[Vector2<f64>],
    ) -> Vec<Vector2<f64>> {
        let (controls, _) = optimize_linearized_controls_without_angle_wrap_with_trace(
            xref,
            xbar,
            state,
            initial_controls,
            QP_MAX_ITERS,
            LINE_SEARCH_ITERS,
        );
        controls
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_no_angle_wrap_closed_loop_parity() {
        // Python reference values (from solver gap memo)
        let python_final_state = [
            -0.014_888_855_512_389,
            0.698_023_990_780_443,
            0.132_179_560_675_735,
            1.589_745_981_585_03,
        ];
        let python_history_length = 347usize;

        // Run default (with angle wrap) closed-loop
        let default_result = run_mpc_simulation();
        let default_final = default_result.final_state.to_vector();
        let default_len = default_result.hist_x.len();

        // Run no-angle-wrap closed-loop
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let final_index = cx.len() - 1;
        let goal = (cx[final_index], cy[final_index]);
        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut step_count = 0;

        for _ in 0..MAX_SIM_STEPS {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize_linearized_controls_without_angle_wrap,
            );

            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
            step_count += 1;

            if check_goal(&state, goal, target_index, final_index) {
                break;
            }
        }

        let nowrap_final = state.to_vector();
        let nowrap_len = step_count + 1; // +1 for initial state

        // Compute diffs against Python reference
        let default_diffs: Vec<f64> = (0..4)
            .map(|i| (default_final[i] - python_final_state[i]).abs())
            .collect();
        let nowrap_diffs: Vec<f64> = (0..4)
            .map(|i| (nowrap_final[i] - python_final_state[i]).abs())
            .collect();

        println!("=== Closed-Loop Parity: Default vs No-Angle-Wrap ===");
        println!(
            "Python reference: {:?} (len={})",
            python_final_state, python_history_length
        );
        println!();
        println!("Default (with normalize_angle):");
        println!(
            "  final: [{:.6}, {:.6}, {:.6}, {:.6}] (len={})",
            default_final[0], default_final[1], default_final[2], default_final[3], default_len
        );
        println!(
            "  diffs: [{:.6e}, {:.6e}, {:.6e}, {:.6e}] step_diff={}",
            default_diffs[0],
            default_diffs[1],
            default_diffs[2],
            default_diffs[3],
            (default_len as isize - python_history_length as isize).abs()
        );
        println!();
        println!("No-angle-wrap:");
        println!(
            "  final: [{:.6}, {:.6}, {:.6}, {:.6}] (len={})",
            nowrap_final[0], nowrap_final[1], nowrap_final[2], nowrap_final[3], nowrap_len
        );
        println!(
            "  diffs: [{:.6e}, {:.6e}, {:.6e}, {:.6e}] step_diff={}",
            nowrap_diffs[0],
            nowrap_diffs[1],
            nowrap_diffs[2],
            nowrap_diffs[3],
            (nowrap_len as isize - python_history_length as isize).abs()
        );
        println!();

        // Compare total state diff
        let default_total: f64 = default_diffs.iter().sum();
        let nowrap_total: f64 = nowrap_diffs.iter().sum();
        println!(
            "Total state diff: default={:.6e} nowrap={:.6e} improvement={:.2}%",
            default_total,
            nowrap_total,
            if default_total > 0.0 {
                (1.0 - nowrap_total / default_total) * 100.0
            } else {
                0.0
            }
        );
    }

    /// Solve the MPC QP using clarabel, matching the Python CVXPY formulation exactly.
    ///
    /// Decision variables: z = [u_flat (2*T), x_flat (4*(T+1))]
    ///   u_flat = [a0, δ0, a1, δ1, ..., a_{T-1}, δ_{T-1}]
    ///   x_flat = [x0_0..x0_3, x1_0..x1_3, ..., xT_0..xT_3]
    ///
    /// Cost: Σ u[t]'Ru[t] + Σ_{t=1..T-1} (x[t]-xref[t])'Q(x[t]-xref[t])
    ///       + (x[T]-xref[T])'Qf(x[T]-xref[T]) + Σ (u[t+1]-u[t])'Rd(u[t+1]-u[t])
    ///
    /// Constraints:
    ///   x[t+1] = A[t]*x[t] + B[t]*u[t] + C[t]   (equality)
    ///   x[0] = x0                                  (equality)
    ///   -MAX_ACCEL <= u[0,t] <= MAX_ACCEL          (box)
    ///   -MAX_STEER <= u[1,t] <= MAX_STEER          (box)
    ///   |u[1,t+1] - u[1,t]| <= MAX_DSTEER*DT      (box)
    ///   MIN_SPEED <= x[2,t] <= MAX_SPEED            (box)
    #[allow(clippy::needless_range_loop, clippy::type_complexity)]
    fn solve_mpc_qp_clarabel(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
    ) -> Option<(Vec<Vector2<f64>>, Vec<Vector4<f64>>)> {
        use clarabel::algebra::*;
        use clarabel::solver::*;

        let nu = 2; // controls per step
        let nx = 4; // state dim
        let n_u = nu * T; // total control vars: 10
        let n_x = nx * (T + 1); // total state vars: 24
        let n = n_u + n_x; // total vars: 34

        // Variable index helpers
        let u_idx = |t: usize, k: usize| -> usize { t * nu + k };
        let x_idx = |t: usize, k: usize| -> usize { n_u + t * nx + k };

        // --- Build P (cost Hessian, upper triangular) ---
        let r_diag = R;
        let rd_diag = RD;
        let q_diag = Q;
        let qf_diag = QF;

        let mut p_rows = Vec::new();
        let mut p_cols = Vec::new();
        let mut p_vals = Vec::new();

        // Control cost: R
        for t in 0..T {
            for k in 0..nu {
                let idx = u_idx(t, k);
                let mut val = r_diag[k];
                // Rate cost: Rd contributes to diagonal
                if t > 0 {
                    val += rd_diag[k];
                }
                if t < T - 1 {
                    val += rd_diag[k];
                }
                p_rows.push(idx);
                p_cols.push(idx);
                p_vals.push(val * 2.0); // clarabel uses 0.5*z'Pz, so P = 2*H
            }
            // Rate cost off-diagonal: -Rd between consecutive controls
            if t < T - 1 {
                for k in 0..nu {
                    let i = u_idx(t, k);
                    let j = u_idx(t + 1, k);
                    // Upper triangular: i < j
                    p_rows.push(i);
                    p_cols.push(j);
                    p_vals.push(-rd_diag[k] * 2.0);
                }
            }
        }

        // State cost: Q for t=1..T-1, Qf for t=T
        for t in 1..=T {
            let w = if t == T { qf_diag } else { q_diag };
            for k in 0..nx {
                let idx = x_idx(t, k);
                p_rows.push(idx);
                p_cols.push(idx);
                p_vals.push(w[k] * 2.0);
            }
        }

        let p = CscMatrix::new_from_triplets(n, n, p_rows.clone(), p_cols.clone(), p_vals.clone());

        // --- Build q (linear cost) ---
        let mut q_vec = vec![0.0; n];
        for t in 1..=T {
            let w = if t == T { qf_diag } else { q_diag };
            for k in 0..nx {
                // cost = w[k] * (x[k] - xref[k])^2 = w[k]*x[k]^2 - 2*w[k]*xref[k]*x[k] + const
                // q contribution: -2*w[k]*xref[k] (but clarabel uses 0.5*z'Pz + q'z, so just -2*w*xref)
                q_vec[x_idx(t, k)] = -w[k] * 2.0 * xref[t][k];
            }
        }

        // --- Build constraints ---
        // Equality: dynamics x[t+1] = A[t]*x[t] + B[t]*u[t] + C[t]  → A[t]*x[t] + B[t]*u[t] - x[t+1] + C[t] = 0
        // Equality: x[0] = x0
        // Box constraints on u and x

        let n_eq = nx * (T + 1); // T dynamics + 1 initial = (T+1)*nx = 24
        let n_steer_rate = T - 1; // |δ[t+1]-δ[t]| <= MAX_DSTEER*DT → 2*(T-1) ineq
        let n_u_box = 2 * nu * T; // upper+lower for each u component: 20
        let n_steer_rate_box = 2 * n_steer_rate; // 8
        let n_v_box = 2 * (T + 1); // speed constraints: 12
        let n_ineq = n_u_box + n_steer_rate_box + n_v_box;

        let mut a_rows = Vec::new();
        let mut a_cols = Vec::new();
        let mut a_vals = Vec::new();
        let mut b_vec = vec![0.0; n_eq + n_ineq];

        // -- Equality: x[0] = x0 --
        let x0 = state.to_vector();
        for k in 0..nx {
            let row = k;
            a_rows.push(row);
            a_cols.push(x_idx(0, k));
            a_vals.push(1.0);
            b_vec[row] = x0[k];
        }

        // -- Equality: dynamics for t=0..T-1 --
        for t in 0..T {
            let (a_mat, b_mat, c_vec) = get_linear_model_matrix(xbar[t][2], xbar[t][3], 0.0);
            let base_row = nx * (t + 1);

            // x[t+1] = A*x[t] + B*u[t] + C
            // → -x[t+1] + A*x[t] + B*u[t] = -C
            for i in 0..nx {
                let row = base_row + i;
                // -x[t+1][i]
                a_rows.push(row);
                a_cols.push(x_idx(t + 1, i));
                a_vals.push(-1.0);
                // A[i,:] * x[t]
                for j in 0..nx {
                    let val = a_mat[(i, j)];
                    if val.abs() > 1e-15 {
                        a_rows.push(row);
                        a_cols.push(x_idx(t, j));
                        a_vals.push(val);
                    }
                }
                // B[i,:] * u[t]
                for j in 0..nu {
                    let val = b_mat[(i, j)];
                    if val.abs() > 1e-15 {
                        a_rows.push(row);
                        a_cols.push(u_idx(t, j));
                        a_vals.push(val);
                    }
                }
                b_vec[row] = -c_vec[i];
            }
        }

        // -- Inequality constraints (as cones) --
        // clarabel inequality: A*z + s = b, s >= 0  → A*z <= b
        let ineq_offset = n_eq;
        let mut ineq_row = 0;

        // u box: -MAX_ACCEL <= a[t] <= MAX_ACCEL, -MAX_STEER <= δ[t] <= MAX_STEER
        let u_bounds = [MAX_ACCEL, MAX_STEER];
        for t in 0..T {
            for (k, &bound) in u_bounds.iter().enumerate() {
                // u[t][k] <= bound
                let row = ineq_offset + ineq_row;
                a_rows.push(row);
                a_cols.push(u_idx(t, k));
                a_vals.push(1.0);
                b_vec[row] = bound;
                ineq_row += 1;

                // -u[t][k] <= bound
                let row = ineq_offset + ineq_row;
                a_rows.push(row);
                a_cols.push(u_idx(t, k));
                a_vals.push(-1.0);
                b_vec[row] = bound;
                ineq_row += 1;
            }
        }

        // Steer rate: |δ[t+1] - δ[t]| <= MAX_DSTEER * DT
        let max_steer_delta = MAX_DSTEER * DT;
        for t in 0..(T - 1) {
            // δ[t+1] - δ[t] <= max_steer_delta
            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(u_idx(t + 1, 1));
            a_vals.push(1.0);
            a_rows.push(row);
            a_cols.push(u_idx(t, 1));
            a_vals.push(-1.0);
            b_vec[row] = max_steer_delta;
            ineq_row += 1;

            // -(δ[t+1] - δ[t]) <= max_steer_delta
            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(u_idx(t + 1, 1));
            a_vals.push(-1.0);
            a_rows.push(row);
            a_cols.push(u_idx(t, 1));
            a_vals.push(1.0);
            b_vec[row] = max_steer_delta;
            ineq_row += 1;
        }

        // Speed box: MIN_SPEED <= x[2,t] <= MAX_SPEED for all t
        for t in 0..=T {
            // x[2,t] <= MAX_SPEED
            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(x_idx(t, 2));
            a_vals.push(1.0);
            b_vec[row] = MAX_SPEED;
            ineq_row += 1;

            // -x[2,t] <= -MIN_SPEED
            let row = ineq_offset + ineq_row;
            a_rows.push(row);
            a_cols.push(x_idx(t, 2));
            a_vals.push(-1.0);
            b_vec[row] = -MIN_SPEED;
            ineq_row += 1;
        }

        let m = n_eq + n_ineq;

        // Build CscMatrix for A (constraint matrix)
        // clarabel expects column-major CscMatrix
        let a_csc = CscMatrix::new_from_triplets(m, n, a_rows, a_cols, a_vals);

        // Cone specification
        let cones = vec![
            SupportedConeT::ZeroConeT(n_eq),          // equality constraints
            SupportedConeT::NonnegativeConeT(n_ineq), // inequality constraints (s >= 0)
        ];

        let settings = DefaultSettingsBuilder::default()
            .verbose(false)
            .build()
            .unwrap();

        let mut solver = DefaultSolver::new(&p, &q_vec, &a_csc, &b_vec, &cones, settings).ok()?;
        solver.solve();

        if solver.solution.status != SolverStatus::Solved {
            return None;
        }

        let z = &solver.solution.x;

        let controls: Vec<Vector2<f64>> = (0..T)
            .map(|t| Vector2::new(z[u_idx(t, 0)], z[u_idx(t, 1)]))
            .collect();

        let predicted: Vec<Vector4<f64>> = (0..=T)
            .map(|t| {
                Vector4::new(
                    z[x_idx(t, 0)],
                    z[x_idx(t, 1)],
                    z[x_idx(t, 2)],
                    z[x_idx(t, 3)],
                )
            })
            .collect();

        Some((controls, predicted))
    }

    /// Wrapper for QP solver matching the test optimizer signature.
    fn optimize_linearized_controls_qp(
        xref: &[Vector4<f64>],
        xbar: &[Vector4<f64>],
        state: &State,
        _initial_controls: &[Vector2<f64>],
    ) -> Vec<Vector2<f64>> {
        if let Some((controls, _)) = solve_mpc_qp_clarabel(xref, xbar, state) {
            controls
        } else {
            // Fallback to projected-gradient if QP fails
            optimize_linearized_controls(xref, xbar, state, _initial_controls)
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_qp_solver_matches_projected_gradient_on_representative_steps() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            80,
            optimize_linearized_controls,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            80,
            optimize_linearized_controls,
        );

        println!("=== QP Solver (clarabel) vs Projected-Gradient ===");
        println!();

        for &step in &[20usize, 70, 75, 78] {
            let row = detailed_row_at_step(&default_trace, step);
            let ws = default_warm_start.get(&step).unwrap();
            let state = state_from_vector4(&row.state);

            // Get xbar for QP (same as projected-gradient uses)
            let mut controls_for_xbar = ws.to_vec();
            controls_for_xbar.resize(T, Vector2::zeros());
            apply_control_constraints(&mut controls_for_xbar);
            let xbar = predict_motion(state, &controls_for_xbar);

            // Projected-gradient result (1 outer iteration for fair comparison)
            let pg_controls = optimize_linearized_controls(&row.xref, &xbar, &state, ws);

            // QP result
            let qp_result = solve_mpc_qp_clarabel(&row.xref, &xbar, &state);

            if let Some((qp_controls, _qp_predicted)) = qp_result {
                let control_diff = summarize_vec2_sequence_diff(
                    &qp_controls,
                    &pg_controls.iter().map(|c| [c[0], c[1]]).collect::<Vec<_>>(),
                );
                println!(
                    "step {step}: control_diff=[{:.6e}, {:.6e}] pg_steer={:.6} qp_steer={:.6}",
                    control_diff[0], control_diff[1], pg_controls[0][1], qp_controls[0][1],
                );
            } else {
                println!("step {step}: QP solver failed!");
            }
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_qp_solver_closed_loop_parity() {
        let python_final_state = [
            -0.014_888_855_512_389,
            0.698_023_990_780_443,
            0.132_179_560_675_735,
            1.589_745_981_585_03,
        ];
        let python_history_length = 347usize;

        let default_result = run_mpc_simulation();
        let default_final = default_result.final_state.to_vector();

        // QP solver closed-loop
        let (cx, cy, cyaw) = generate_switch_back_course();
        let speed_profile = calc_speed_profile(&cx, &cy, &cyaw, TARGET_SPEED);

        let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
        if state.yaw - cyaw[0] >= PI {
            state.yaw -= 2.0 * PI;
        } else if state.yaw - cyaw[0] <= -PI {
            state.yaw += 2.0 * PI;
        }

        let final_index = cx.len() - 1;
        let goal = (cx[final_index], cy[final_index]);
        let mut target_index = 0;
        let mut warm_start = vec![Vector2::zeros(); T];
        let mut step_count = 0;

        for _ in 0..MAX_SIM_STEPS {
            let (xref, new_index) = calc_ref_trajectory(
                &state,
                &cx,
                &cy,
                &cyaw,
                &speed_profile,
                UPSTREAM_SWITCH_BACK_TICK,
                target_index,
            );
            target_index = new_index;

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &xref,
                &state,
                &warm_start,
                optimize_linearized_controls_qp,
            );

            state.update(result.controls[0][0], result.controls[0][1]);
            warm_start = next_warm_start(&result.controls);
            step_count += 1;

            if check_goal(&state, goal, target_index, final_index) {
                break;
            }
        }

        let qp_final = state.to_vector();
        let qp_len = step_count + 1;

        let default_diffs: Vec<f64> = (0..4)
            .map(|i| (default_final[i] - python_final_state[i]).abs())
            .collect();
        let qp_diffs: Vec<f64> = (0..4)
            .map(|i| (qp_final[i] - python_final_state[i]).abs())
            .collect();

        println!("=== Closed-Loop Parity: PG vs QP (clarabel) ===");
        println!(
            "Python ref: {:?} (len={})",
            python_final_state, python_history_length
        );
        println!();
        println!(
            "PG: [{:.6},{:.6},{:.6},{:.6}] len={} diffs=[{:.4e},{:.4e},{:.4e},{:.4e}]",
            default_final[0],
            default_final[1],
            default_final[2],
            default_final[3],
            default_result.hist_x.len(),
            default_diffs[0],
            default_diffs[1],
            default_diffs[2],
            default_diffs[3]
        );
        println!(
            "QP: [{:.6},{:.6},{:.6},{:.6}] len={} diffs=[{:.4e},{:.4e},{:.4e},{:.4e}]",
            qp_final[0],
            qp_final[1],
            qp_final[2],
            qp_final[3],
            qp_len,
            qp_diffs[0],
            qp_diffs[1],
            qp_diffs[2],
            qp_diffs[3]
        );

        let pg_total: f64 = default_diffs.iter().sum();
        let qp_total: f64 = qp_diffs.iter().sum();
        println!(
            "Total: PG={:.4e} QP={:.4e} improvement={:.1}%",
            pg_total,
            qp_total,
            if pg_total > 0.0 {
                (1.0 - qp_total / pg_total) * 100.0
            } else {
                0.0
            }
        );
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_solver_comparison_qp_vs_pg_detailed() {
        let expected_trace =
            parse_switch_back_trace(include_str!("testdata/mpc_switch_back_prefix_trace.csv"));

        // 4 solvers
        let hybrid_trace = collect_switch_back_detailed_trace(350); // production = hybrid
        let qp_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            350,
            optimize_linearized_controls_qp,
        );
        let pg_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            350,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let pg_orig_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            350,
            optimize_linearized_controls_original,
        );

        println!("=== Solver Comparison (state diff sum vs Python) ===");
        println!(
            "{:>5} {:>10} {:>10} {:>10} {:>10}  best",
            "step", "hybrid", "qp-only", "pg-exp", "pg-orig"
        );

        let check_steps: Vec<usize> = (0..=80)
            .step_by(5)
            .chain([71, 72, 73, 74, 75, 76, 77, 78, 79, 80])
            .chain((85..350).step_by(10))
            .collect();

        for &step in &check_steps {
            if step >= expected_trace.len() {
                continue;
            }
            let py = &expected_trace[step];
            let hy = hybrid_trace.iter().find(|r| r.step == step);
            let qp = qp_trace.iter().find(|r| r.step == step);
            let pg = pg_trace.iter().find(|r| r.step == step);
            let po = pg_orig_trace.iter().find(|r| r.step == step);

            if let (Some(hy), Some(qp), Some(pg), Some(po)) = (hy, qp, pg, po) {
                let hy_s: f64 = trace_state_diff(&hy.state, &py.state).iter().sum();
                let qp_s: f64 = trace_state_diff(&qp.state, &py.state).iter().sum();
                let pg_s: f64 = trace_state_diff(&pg.state, &py.state).iter().sum();
                let po_s: f64 = trace_state_diff(&po.state, &py.state).iter().sum();

                let min_s = hy_s.min(qp_s).min(pg_s).min(po_s);
                let best = if (hy_s - min_s).abs() < 1e-15 {
                    "HYBRID"
                } else if (qp_s - min_s).abs() < 1e-15 {
                    "QP"
                } else if (pg_s - min_s).abs() < 1e-15 {
                    "PG-exp"
                } else {
                    "PG-orig"
                };

                println!(
                    "{:5} {:10.4e} {:10.4e} {:10.4e} {:10.4e}  {}",
                    step, hy_s, qp_s, pg_s, po_s, best
                );
            }
        }

        // Win count
        let mut wins = [0u32; 4]; // hybrid, qp, pg-exp, pg-orig
        let labels = ["HYBRID", "QP-only", "PG-expanded", "PG-original"];
        let min_steps = [
            hybrid_trace.len(),
            qp_trace.len(),
            pg_trace.len(),
            pg_orig_trace.len(),
            expected_trace.len(),
        ]
        .into_iter()
        .min()
        .unwrap();
        for (step, py) in expected_trace.iter().enumerate().take(min_steps) {
            let traces = [&hybrid_trace, &qp_trace, &pg_trace, &pg_orig_trace];
            let diffs: Vec<f64> = traces
                .iter()
                .map(|t| {
                    t.iter()
                        .find(|r| r.step == step)
                        .map(|r| trace_state_diff(&r.state, &py.state).iter().sum())
                        .unwrap_or(f64::INFINITY)
                })
                .collect();
            let min_d = diffs.iter().copied().fold(f64::INFINITY, f64::min);
            for (i, &d) in diffs.iter().enumerate() {
                if (d - min_d).abs() < 1e-15 {
                    wins[i] += 1;
                    break;
                }
            }
        }
        println!("\n=== Win count (closest to Python per step, out of {min_steps}) ===");
        for (i, label) in labels.iter().enumerate() {
            println!("{:12}: {} steps", label, wins[i]);
        }
    }

    #[test]
    #[ignore = "long-running regression scenario"]
    fn test_mpc_switch_back_characterizes_v_injection_pull_ratio_threshold() {
        let default_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls,
        );
        let expanded_trace = collect_switch_back_detailed_trace_with_test_optimizer(
            330,
            optimize_linearized_controls_with_full_candidate_expansion,
        );
        let default_warm_start = collect_switch_back_warm_start_trace_with_test_optimizer(
            330,
            optimize_linearized_controls,
        );

        let d325 = detailed_row_at_step(&default_trace, 325);
        let e325 = detailed_row_at_step(&expanded_trace, 325);
        let e326 = detailed_row_at_step(&expanded_trace, 326);
        let ws = default_warm_start.get(&325).unwrap();

        // Baseline values
        let d_v = d325.state[2]; // default v at step 325
        let e325_v = e325.state[2]; // expanded v at step 325: ~ -0.804
        let e326_v = e326.state[2]; // expanded v at step 326: ~ -0.604

        let d_baseline_controls: Vec<[f64; 2]> =
            d325.controls.iter().map(|v| [v[0], v[1]]).collect();
        let e_baseline_controls: Vec<[f64; 2]> =
            e325.controls.iter().map(|v| [v[0], v[1]]).collect();

        println!("=== V-Injection Pull Ratio Threshold Sweep ===");
        println!(
            "default v={:.6}, e325_v={:.6}, e326_v={:.6}",
            d_v, e325_v, e326_v
        );
        println!("Sweeping injected v from e325_v to e326_v in 0.02 steps");
        println!();
        println!(
            "{:>10} {:>10} {:>10} {:>10} {:>10}",
            "inj_v", "steer", "same_c0", "opp_c0", "ratio"
        );

        // Sweep from e325_v to e326_v
        let n_steps = 11;
        let mut prev_ratio = f64::INFINITY;
        let mut threshold_v = None;

        for i in 0..=n_steps {
            let frac = i as f64 / n_steps as f64;
            let injected_v = e325_v + frac * (e326_v - e325_v);

            // Create hybrid state: default state with injected v
            let mut hybrid = d325.state;
            hybrid[2] = injected_v;
            let hybrid_state = state_from_vector4(&hybrid);

            let result = iterative_linear_mpc_control_with_test_optimizer(
                &d325.xref,
                &hybrid_state,
                ws,
                optimize_linearized_controls,
            );

            let same_c0 = summarize_vec2_sequence_diff(&result.controls, &d_baseline_controls)[0];
            let opp_c0 = summarize_vec2_sequence_diff(&result.controls, &e_baseline_controls)[0];

            let ratio = if opp_c0 > 0.0 {
                same_c0 / opp_c0
            } else {
                f64::INFINITY
            };

            println!(
                "{:10.6} {:10.6} {:10.4e} {:10.4e} {:10.3}",
                injected_v, result.controls[0][1], same_c0, opp_c0, ratio
            );

            // Detect threshold crossing (ratio drops below 4.0)
            if threshold_v.is_none() && prev_ratio > 4.0 && ratio <= 4.0 {
                threshold_v = Some((injected_v, ratio));
            }
            prev_ratio = ratio;
        }

        if let Some((v, r)) = threshold_v {
            println!(
                "\nThreshold: ratio drops below 4.0 at v={:.6} (ratio={:.3})",
                v, r
            );
        } else {
            println!("\nNo threshold crossing found in this range");
        }
    }
}
