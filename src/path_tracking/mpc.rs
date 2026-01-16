// Model Predictive Control (MPC) for path tracking
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109)
//         Rust port
//
// Note: This is a simplified MPC implementation without a QP solver.
// Uses iterative linearization and gradient descent.

use nalgebra::{Matrix4, Vector4, Matrix4x2, Vector2, DMatrix, DVector};
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};
use std::f64::consts::PI;

// Vehicle parameters
const WB: f64 = 2.5; // wheelbase [m]
const MAX_STEER: f64 = 45.0 * PI / 180.0; // max steering angle [rad]
const MAX_DSTEER: f64 = 30.0 * PI / 180.0; // max steering rate [rad/s]
const MAX_SPEED: f64 = 55.0 / 3.6; // max speed [m/s]
const MIN_SPEED: f64 = -20.0 / 3.6; // min speed (reverse) [m/s]
const MAX_ACCEL: f64 = 1.0; // max acceleration [m/ss]

// MPC parameters
const NX: usize = 4; // state dimension [x, y, v, yaw]
const NU: usize = 2; // control dimension [accel, steer]
const T: usize = 5; // prediction horizon
const DT: f64 = 0.2; // time step [s]

// Cost weights
const Q: [f64; 4] = [1.0, 1.0, 0.5, 0.5]; // state cost [x, y, v, yaw]
const R: [f64; 2] = [0.01, 0.01]; // control cost [accel, steer]
const RD: [f64; 2] = [0.01, 1.0]; // control rate cost

const TARGET_SPEED: f64 = 10.0 / 3.6; // target speed [m/s]
const GOAL_DIS: f64 = 1.5; // goal distance threshold
const MAX_ITER: usize = 3; // max MPC iterations

const SHOW_ANIMATION: bool = false;

/// Vehicle state
#[derive(Clone, Copy)]
struct State {
    x: f64,
    y: f64,
    v: f64,
    yaw: f64,
}

impl State {
    fn new(x: f64, y: f64, v: f64, yaw: f64) -> Self {
        State { x, y, v, yaw }
    }

    fn to_vector(&self) -> Vector4<f64> {
        Vector4::new(self.x, self.y, self.v, self.yaw)
    }

    fn from_vector(v: &Vector4<f64>) -> Self {
        State::new(v[0], v[1], v[2], v[3])
    }

    /// Update state using bicycle model
    fn update(&mut self, accel: f64, steer: f64) {
        let steer = steer.clamp(-MAX_STEER, MAX_STEER);

        self.x += self.v * self.yaw.cos() * DT;
        self.y += self.v * self.yaw.sin() * DT;
        self.yaw += self.v / WB * steer.tan() * DT;
        self.yaw = normalize_angle(self.yaw);
        self.v += accel * DT;
        self.v = self.v.clamp(MIN_SPEED, MAX_SPEED);
    }
}

/// Normalize angle to [-pi, pi]
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Calculate angle difference (handles wrap-around)
fn angle_diff(a: f64, b: f64) -> f64 {
    normalize_angle(a - b)
}

/// Get linearized state-space matrices at operating point
fn get_linear_model_matrix(v: f64, phi: f64, delta: f64) -> (Matrix4<f64>, Matrix4x2<f64>, Vector4<f64>) {
    // A matrix (state transition)
    let a = Matrix4::new(
        1.0, 0.0, DT * phi.cos(), -DT * v * phi.sin(),
        0.0, 1.0, DT * phi.sin(), DT * v * phi.cos(),
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, DT * delta.tan() / WB, 1.0,
    );

    // B matrix (control input)
    let b = Matrix4x2::new(
        0.0, 0.0,
        0.0, 0.0,
        DT, 0.0,
        0.0, DT * v / (WB * delta.cos().powi(2)),
    );

    // C vector (constant term for linearization offset)
    let c = Vector4::new(
        DT * v * phi.sin() * phi,
        -DT * v * phi.cos() * phi,
        0.0,
        -DT * v * delta / (WB * delta.cos().powi(2)),
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
        let mut a = y.to_vec();
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

        CubicSpline1D { x: x.to_vec(), a, b, c, d }
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

        CubicSpline2D { s, sx, sy }
    }

    fn calc_position(&self, s: f64) -> (f64, f64) {
        (self.sx.calc(s), self.sy.calc(s))
    }

    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }

    fn calc_curvature(&self, _s: f64) -> f64 {
        // Simplified curvature calculation
        0.0
    }
}

/// Calculate reference trajectory for MPC horizon
fn calc_ref_trajectory(state: &State, csp: &CubicSpline2D, ref_s: &mut f64) -> (Vec<Vector4<f64>>, Vec<f64>) {
    let mut xref = Vec::with_capacity(T + 1);
    let mut sref = Vec::with_capacity(T + 1);

    // Find nearest point on path
    let mut min_d = f64::INFINITY;
    let mut best_s = *ref_s;
    let ds = 0.1;
    let mut s = (*ref_s - 5.0).max(0.0);
    while s <= (*ref_s + 20.0).min(*csp.s.last().unwrap_or(&0.0)) {
        let (px, py) = csp.calc_position(s);
        let d = ((state.x - px).powi(2) + (state.y - py).powi(2)).sqrt();
        if d < min_d {
            min_d = d;
            best_s = s;
        }
        s += ds;
    }
    *ref_s = best_s;

    // Generate reference trajectory over horizon
    let mut s = *ref_s;
    for _ in 0..=T {
        let (rx, ry) = csp.calc_position(s.min(*csp.s.last().unwrap_or(&0.0)));
        let ryaw = csp.calc_yaw(s.min(*csp.s.last().unwrap_or(&0.0)));

        xref.push(Vector4::new(rx, ry, TARGET_SPEED, ryaw));
        sref.push(s);

        s += TARGET_SPEED * DT;
    }

    (xref, sref)
}

/// Simplified MPC solver using gradient descent
fn mpc_solve(
    state: &State,
    xref: &[Vector4<f64>],
    u_prev: &[Vector2<f64>],
) -> Vec<Vector2<f64>> {
    let mut u = u_prev.to_vec();
    if u.len() < T {
        u.resize(T, Vector2::zeros());
    }

    let q_mat = Matrix4::from_diagonal(&Vector4::new(Q[0], Q[1], Q[2], Q[3]));
    let r_mat = nalgebra::Matrix2::from_diagonal(&Vector2::new(R[0], R[1]));

    // Iterative optimization
    for _ in 0..MAX_ITER {
        // Forward simulation
        let mut x_pred = vec![state.to_vector()];
        for i in 0..T {
            let xi = x_pred[i];
            let ui = u[i];

            let (a, b, c) = get_linear_model_matrix(xi[2], xi[3], ui[1]);
            let mut x_next = a * xi + b * ui + c;
            // Normalize yaw angle in prediction
            x_next[3] = normalize_angle(x_next[3]);
            x_pred.push(x_next);
        }

        // Backward pass - compute gradients
        let mut du = vec![Vector2::zeros(); T];

        for i in 0..T {
            // Calculate state error with proper angle wrapping for yaw
            let mut x_error = x_pred[i + 1] - xref[i + 1];
            // Handle yaw angle wrap-around (index 3 is yaw)
            x_error[3] = angle_diff(x_pred[i + 1][3], xref[i + 1][3]);

            let (_, b, _) = get_linear_model_matrix(x_pred[i][2], x_pred[i][3], u[i][1]);

            // Gradient of cost w.r.t. control
            let grad_state = b.transpose() * q_mat * x_error;
            let grad_control = r_mat * u[i];

            // Control rate penalty
            let mut grad_rate = Vector2::zeros();
            if i > 0 {
                grad_rate += nalgebra::Matrix2::from_diagonal(&Vector2::new(RD[0], RD[1])) * (u[i] - u[i - 1]);
            }
            if i < T - 1 {
                grad_rate -= nalgebra::Matrix2::from_diagonal(&Vector2::new(RD[0], RD[1])) * (u[i + 1] - u[i]);
            }

            du[i] = grad_state + grad_control + grad_rate;
        }

        // Update controls with step size
        let alpha = 0.1;
        for i in 0..T {
            u[i] -= alpha * du[i];

            // Apply constraints
            u[i][0] = u[i][0].clamp(-MAX_ACCEL, MAX_ACCEL);
            u[i][1] = u[i][1].clamp(-MAX_STEER, MAX_STEER);

            // Steering rate constraint
            if i > 0 {
                let d_steer = u[i][1] - u[i - 1][1];
                if d_steer.abs() > MAX_DSTEER * DT {
                    u[i][1] = u[i - 1][1] + MAX_DSTEER * DT * d_steer.signum();
                }
            }
        }
    }

    u
}

fn main() {
    println!("MPC Path Tracking start!");

    // Reference path waypoints
    let ax = vec![0.0, 60.0, 125.0, 50.0, 75.0, 35.0, -10.0];
    let ay = vec![0.0, 0.0, 50.0, 65.0, 30.0, -10.0, -20.0];

    // Create reference path
    let csp = CubicSpline2D::new(&ax, &ay);

    // Generate dense path for visualization
    let mut cx = Vec::new();
    let mut cy = Vec::new();
    let mut cyaw = Vec::new();
    let mut s = 0.0;
    let s_max = *csp.s.last().unwrap_or(&0.0);
    while s <= s_max {
        let (x, y) = csp.calc_position(s);
        cx.push(x);
        cy.push(y);
        cyaw.push(csp.calc_yaw(s));
        s += 0.1;
    }

    // Initial state
    let mut state = State::new(cx[0], cy[0], 0.0, cyaw[0]);
    let mut ref_s = 0.0;

    // Control history
    let mut u_history: Vec<Vector2<f64>> = vec![Vector2::zeros(); T];

    // Trajectory history
    let mut hist_x = vec![state.x];
    let mut hist_y = vec![state.y];
    let mut hist_yaw = vec![state.yaw];
    let mut hist_v = vec![state.v];

    let mut fig = Figure::new();

    // Simulation loop
    let goal_x = *cx.last().unwrap_or(&0.0);
    let goal_y = *cy.last().unwrap_or(&0.0);

    while ((state.x - goal_x).powi(2) + (state.y - goal_y).powi(2)).sqrt() > GOAL_DIS {
        // Calculate reference trajectory
        let (xref, _) = calc_ref_trajectory(&state, &csp, &mut ref_s);

        // MPC control
        let u_opt = mpc_solve(&state, &xref, &u_history);

        // Apply first control
        let accel = u_opt[0][0];
        let steer = u_opt[0][1];

        state.update(accel, steer);

        // Shift control history
        u_history = u_opt[1..].to_vec();
        u_history.push(*u_opt.last().unwrap_or(&Vector2::zeros()));

        // Store history
        hist_x.push(state.x);
        hist_y.push(state.y);
        hist_yaw.push(state.yaw);
        hist_v.push(state.v);

        // Visualization
        if SHOW_ANIMATION && hist_x.len() % 5 == 0 {
            fig.clear_axes();

            // Predicted trajectory
            let mut pred_x = vec![state.x];
            let mut pred_y = vec![state.y];
            let mut pred_state = state;
            for u in &u_opt {
                pred_state.update(u[0], u[1]);
                pred_x.push(pred_state.x);
                pred_y.push(pred_state.y);
            }

            fig.axes2d()
                .set_title(&format!("MPC Path Tracking - Speed: {:.2} m/s", state.v), &[])
                .set_x_label("x [m]", &[])
                .set_y_label("y [m]", &[])
                .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
                .lines(&cx, &cy, &[Caption("Reference"), Color("gray")])
                .lines(&hist_x, &hist_y, &[Caption("Trajectory"), Color("blue")])
                .lines(&pred_x, &pred_y, &[Caption("Prediction"), Color("green")])
                .points(&[state.x], &[state.y], &[Caption("Vehicle"), Color("red"), PointSymbol('*'), PointSize(3.0)])
                .points(&[goal_x], &[goal_y], &[Caption("Goal"), Color("magenta"), PointSymbol('O'), PointSize(2.0)]);

            fig.show_and_keep_running().unwrap();
        }

        // Safety check
        if hist_x.len() > 5000 {
            println!("Max iterations reached!");
            break;
        }
    }

    println!("Goal reached!");
    println!("Final position: ({:.2}, {:.2})", state.x, state.y);

    // Save final plot
    fig.clear_axes();

    fig.axes2d()
        .set_title("MPC Path Tracking - Complete", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
        .lines(&cx, &cy, &[Caption("Reference"), Color("gray")])
        .lines(&hist_x, &hist_y, &[Caption("Trajectory"), Color("blue")])
        .points(&[goal_x], &[goal_y], &[Caption("Goal"), Color("magenta"), PointSymbol('O'), PointSize(2.0)]);

    fig.save_to_svg("./img/path_tracking/mpc.svg", 640, 480).unwrap();
    println!("Plot saved to ./img/path_tracking/mpc.svg");

    println!("Done!");
}
