// Nonlinear MPC simulation with CGMRES (Continuation GMRES)
//
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109) - Rust port
//
// Reference:
// Shunichi09/nonlinear_control: Implementing the nonlinear model predictive
// control, sliding mode control https://github.com/Shunichi09/PythonLinearNonlinearControl

use nalgebra::{DMatrix, DVector};
use gnuplot::{Figure, Caption, Color, AxesCommon, PointSymbol, PointSize};
use std::f64::consts::PI;

// System parameters
const U_A_MAX: f64 = 1.0;              // Maximum acceleration input
const U_OMEGA_MAX: f64 = PI / 4.0;     // Maximum steering angle (45 degrees)
const PHI_V: f64 = 0.01;               // Penalty weight for acceleration
const PHI_OMEGA: f64 = 0.01;           // Penalty weight for steering
const WB: f64 = 0.25;                  // Wheelbase [m]

const SHOW_ANIMATION: bool = false;

/// Differential model for two-wheeled robot
/// Returns (dx, dy, d_yaw, dv) derivatives
fn differential_model(v: f64, yaw: f64, u_1: f64, u_2: f64) -> (f64, f64, f64, f64) {
    let dx = yaw.cos() * v;
    let dy = yaw.sin() * v;
    let dv = u_1;
    // Using sin(u_2) instead of tan(u_2) for better nonlinear optimization
    let d_yaw = v / WB * u_2.sin();

    (dx, dy, d_yaw, dv)
}

/// Two-wheeled robot system state
pub struct TwoWheeledSystem {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub history_x: Vec<f64>,
    pub history_y: Vec<f64>,
    pub history_yaw: Vec<f64>,
    pub history_v: Vec<f64>,
}

impl TwoWheeledSystem {
    pub fn new(init_x: f64, init_y: f64, init_yaw: f64, init_v: f64) -> Self {
        TwoWheeledSystem {
            x: init_x,
            y: init_y,
            yaw: init_yaw,
            v: init_v,
            history_x: vec![init_x],
            history_y: vec![init_y],
            history_yaw: vec![init_yaw],
            history_v: vec![init_v],
        }
    }

    /// Update state using Euler integration
    pub fn update_state(&mut self, u_1: f64, u_2: f64, dt: f64) {
        let (dx, dy, d_yaw, dv) = differential_model(self.v, self.yaw, u_1, u_2);

        self.x += dt * dx;
        self.y += dt * dy;
        self.yaw += dt * d_yaw;
        self.v += dt * dv;

        // Save history
        self.history_x.push(self.x);
        self.history_y.push(self.y);
        self.history_yaw.push(self.yaw);
        self.history_v.push(self.v);
    }
}

/// NMPC Simulator System for state prediction and adjoint calculation
pub struct NMPCSimulatorSystem;

impl NMPCSimulatorSystem {
    pub fn new() -> Self {
        NMPCSimulatorSystem
    }

    /// Calculate predicted states and adjoint (costate) variables
    pub fn calc_predict_and_adjoint_state(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        v: f64,
        u_1s: &[f64],
        u_2s: &[f64],
        n: usize,
        dt: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        // Forward prediction using state equations
        let (x_s, y_s, yaw_s, v_s) = self.calc_predict_states(x, y, yaw, v, u_1s, u_2s, n, dt);

        // Backward adjoint calculation using adjoint equations
        let (lam_1s, lam_2s, lam_3s, lam_4s) =
            self.calc_adjoint_states(&x_s, &y_s, &yaw_s, &v_s, u_2s, n, dt);

        (x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s)
    }

    /// Forward state prediction using Euler integration
    fn calc_predict_states(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        v: f64,
        u_1s: &[f64],
        u_2s: &[f64],
        n: usize,
        dt: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut x_s = vec![x];
        let mut y_s = vec![y];
        let mut yaw_s = vec![yaw];
        let mut v_s = vec![v];

        for i in 0..n {
            let (next_x, next_y, next_yaw, next_v) = self.predict_state_euler(
                x_s[i], y_s[i], yaw_s[i], v_s[i], u_1s[i], u_2s[i], dt,
            );
            x_s.push(next_x);
            y_s.push(next_y);
            yaw_s.push(next_yaw);
            v_s.push(next_v);
        }

        (x_s, y_s, yaw_s, v_s)
    }

    /// Backward adjoint state calculation
    fn calc_adjoint_states(
        &self,
        x_s: &[f64],
        y_s: &[f64],
        yaw_s: &[f64],
        v_s: &[f64],
        u_2s: &[f64],
        n: usize,
        dt: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        // Initialize with terminal conditions (terminal cost gradient = state)
        let mut lam_1s = vec![x_s[n]];
        let mut lam_2s = vec![y_s[n]];
        let mut lam_3s = vec![yaw_s[n]];
        let mut lam_4s = vec![v_s[n]];

        // Backward integration
        for i in (1..n).rev() {
            let (pre_lam_1, pre_lam_2, pre_lam_3, pre_lam_4) = self.adjoint_state_euler(
                yaw_s[i], v_s[i], lam_1s[0], lam_2s[0], lam_3s[0], lam_4s[0], u_2s[i], dt,
            );
            lam_1s.insert(0, pre_lam_1);
            lam_2s.insert(0, pre_lam_2);
            lam_3s.insert(0, pre_lam_3);
            lam_4s.insert(0, pre_lam_4);
        }

        (lam_1s, lam_2s, lam_3s, lam_4s)
    }

    /// Single step Euler integration for state prediction
    fn predict_state_euler(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        v: f64,
        u_1: f64,
        u_2: f64,
        dt: f64,
    ) -> (f64, f64, f64, f64) {
        let (dx, dy, d_yaw, dv) = differential_model(v, yaw, u_1, u_2);

        let next_x = x + dt * dx;
        let next_y = y + dt * dy;
        let next_yaw = yaw + dt * d_yaw;
        let next_v = v + dt * dv;

        (next_x, next_y, next_yaw, next_v)
    }

    /// Single step Euler integration for adjoint state (backward)
    /// Computes ∂H/∂x for costate equations
    fn adjoint_state_euler(
        &self,
        yaw: f64,
        v: f64,
        lam_1: f64,
        lam_2: f64,
        lam_3: f64,
        lam_4: f64,
        u_2: f64,
        dt: f64,
    ) -> (f64, f64, f64, f64) {
        // Adjoint equations: dλ/dt = -∂H/∂x

        // ∂H/∂x = 0
        let pre_lam_1 = lam_1 + dt * 0.0;

        // ∂H/∂y = 0
        let pre_lam_2 = lam_2 + dt * 0.0;

        // ∂H/∂yaw
        let tmp1 = -lam_1 * yaw.sin() * v + lam_2 * yaw.cos() * v;
        let pre_lam_3 = lam_3 + dt * tmp1;

        // ∂H/∂v
        let tmp2 = lam_1 * yaw.cos() + lam_2 * yaw.sin() + lam_3 * u_2.sin() / WB;
        let pre_lam_4 = lam_4 + dt * tmp2;

        (pre_lam_1, pre_lam_2, pre_lam_3, pre_lam_4)
    }
}

impl Default for NMPCSimulatorSystem {
    fn default() -> Self {
        Self::new()
    }
}

/// NMPC Controller using C-GMRES (Continuation GMRES) algorithm
pub struct NMPCControllerCGMRES {
    // Parameters
    pub zeta: f64,         // Stability gain for optimal answer
    pub ht: f64,           // Finite difference step size
    pub tf: f64,           // Prediction time horizon
    pub alpha: f64,        // Time scaling factor
    pub n: usize,          // Number of prediction steps
    pub threshold: f64,    // CGMRES convergence threshold
    pub input_num: usize,  // Number of inputs (u1, u2, dummy_u1, dummy_u2, raw1, raw2)
    pub max_iteration: usize,

    // Simulator
    simulator: NMPCSimulatorSystem,

    // Control inputs over horizon
    pub u_1s: Vec<f64>,
    pub u_2s: Vec<f64>,
    pub dummy_u_1s: Vec<f64>,
    pub dummy_u_2s: Vec<f64>,
    pub raw_1s: Vec<f64>,
    pub raw_2s: Vec<f64>,

    // History for plotting
    pub history_u_1: Vec<f64>,
    pub history_u_2: Vec<f64>,
    pub history_dummy_u_1: Vec<f64>,
    pub history_dummy_u_2: Vec<f64>,
    pub history_raw_1: Vec<f64>,
    pub history_raw_2: Vec<f64>,
    pub history_f: Vec<f64>,
}

impl NMPCControllerCGMRES {
    pub fn new() -> Self {
        let n = 10;  // Number of prediction steps
        let input_num = 6;

        NMPCControllerCGMRES {
            zeta: 100.0,
            ht: 0.01,
            tf: 3.0,
            alpha: 0.5,
            n,
            threshold: 0.001,
            input_num,
            max_iteration: input_num * n,

            simulator: NMPCSimulatorSystem::new(),

            // Initialize inputs as 1.0 (except raw which are 0.0)
            u_1s: vec![1.0; n],
            u_2s: vec![1.0; n],
            dummy_u_1s: vec![1.0; n],
            dummy_u_2s: vec![1.0; n],
            raw_1s: vec![0.0; n],
            raw_2s: vec![0.0; n],

            history_u_1: Vec::new(),
            history_u_2: Vec::new(),
            history_dummy_u_1: Vec::new(),
            history_dummy_u_2: Vec::new(),
            history_raw_1: Vec::new(),
            history_raw_2: Vec::new(),
            history_f: Vec::new(),
        }
    }

    /// Calculate control input using C-GMRES algorithm
    pub fn calc_input(&mut self, x: f64, y: f64, yaw: f64, v: f64, time: f64) -> (Vec<f64>, Vec<f64>) {
        // Calculate sampling time (increases with time for stability)
        let dt = self.tf * (1.0 - (-self.alpha * time).exp()) / (self.n as f64);

        // Calculate state derivatives (x_dot)
        let (x_1_dot, x_2_dot, x_3_dot, x_4_dot) =
            differential_model(v, yaw, self.u_1s[0], self.u_2s[0]);

        let dx_1 = x_1_dot * self.ht;
        let dx_2 = x_2_dot * self.ht;
        let dx_3 = x_3_dot * self.ht;
        let dx_4 = x_4_dot * self.ht;

        // F(U, x + h*x_dot, t + h) - Fxt
        let (_, _, _, v_s, _, _, lam_3s, lam_4s) = self.simulator.calc_predict_and_adjoint_state(
            x + dx_1,
            y + dx_2,
            yaw + dx_3,
            v + dx_4,
            &self.u_1s,
            &self.u_2s,
            self.n,
            dt,
        );

        let fxt = Self::calc_f(
            &v_s,
            &lam_3s,
            &lam_4s,
            &self.u_1s,
            &self.u_2s,
            &self.dummy_u_1s,
            &self.dummy_u_2s,
            &self.raw_1s,
            &self.raw_2s,
            self.n,
        );

        // F(U, x, t) - F
        let (_, _, _, v_s_current, _, _, lam_3s_current, lam_4s_current) =
            self.simulator.calc_predict_and_adjoint_state(
                x, y, yaw, v, &self.u_1s, &self.u_2s, self.n, dt,
            );

        let f = Self::calc_f(
            &v_s_current,
            &lam_3s_current,
            &lam_4s_current,
            &self.u_1s,
            &self.u_2s,
            &self.dummy_u_1s,
            &self.dummy_u_2s,
            &self.raw_1s,
            &self.raw_2s,
            self.n,
        );

        // Right hand side: -zeta * F - (Fxt - F) / ht
        let right: Vec<f64> = f
            .iter()
            .zip(fxt.iter())
            .map(|(&f_i, &fxt_i)| -self.zeta * f_i - (fxt_i - f_i) / self.ht)
            .collect();

        // Initial du for perturbation
        let du_1: Vec<f64> = self.u_1s.iter().map(|&u| u * self.ht).collect();
        let du_2: Vec<f64> = self.u_2s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_1: Vec<f64> = self.dummy_u_1s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_2: Vec<f64> = self.dummy_u_2s.iter().map(|&u| u * self.ht).collect();
        let draw_1: Vec<f64> = self.raw_1s.iter().map(|&u| u * self.ht).collect();
        let draw_2: Vec<f64> = self.raw_2s.iter().map(|&u| u * self.ht).collect();

        // F(U + h*dU(0), x + h*x_dot, t + h)
        let u_1s_perturbed: Vec<f64> = self.u_1s.iter().zip(du_1.iter()).map(|(&u, &d)| u + d).collect();
        let u_2s_perturbed: Vec<f64> = self.u_2s.iter().zip(du_2.iter()).map(|(&u, &d)| u + d).collect();

        let (_, _, _, v_s_pert, _, _, lam_3s_pert, lam_4s_pert) =
            self.simulator.calc_predict_and_adjoint_state(
                x + dx_1,
                y + dx_2,
                yaw + dx_3,
                v + dx_4,
                &u_1s_perturbed,
                &u_2s_perturbed,
                self.n,
                dt,
            );

        let dummy_u_1s_pert: Vec<f64> = self.dummy_u_1s.iter().zip(ddummy_u_1.iter()).map(|(&u, &d)| u + d).collect();
        let dummy_u_2s_pert: Vec<f64> = self.dummy_u_2s.iter().zip(ddummy_u_2.iter()).map(|(&u, &d)| u + d).collect();
        let raw_1s_pert: Vec<f64> = self.raw_1s.iter().zip(draw_1.iter()).map(|(&u, &d)| u + d).collect();
        let raw_2s_pert: Vec<f64> = self.raw_2s.iter().zip(draw_2.iter()).map(|(&u, &d)| u + d).collect();

        let fuxt = Self::calc_f(
            &v_s_pert,
            &lam_3s_pert,
            &lam_4s_pert,
            &u_1s_perturbed,
            &u_2s_perturbed,
            &dummy_u_1s_pert,
            &dummy_u_2s_pert,
            &raw_1s_pert,
            &raw_2s_pert,
            self.n,
        );

        // Left hand side: (Fuxt - Fxt) / ht
        let left: Vec<f64> = fuxt
            .iter()
            .zip(fxt.iter())
            .map(|(&fuxt_i, &fxt_i)| (fuxt_i - fxt_i) / self.ht)
            .collect();

        // Initial residual: r0 = right - left
        let r0: Vec<f64> = right.iter().zip(left.iter()).map(|(&r, &l)| r - l).collect();
        let r0_norm = vec_norm(&r0);

        // GMRES iteration
        let mut vs = DMatrix::zeros(self.max_iteration, self.max_iteration + 1);
        for i in 0..self.max_iteration {
            vs[(i, 0)] = r0[i] / r0_norm;
        }

        let mut hs = DMatrix::zeros(self.max_iteration + 1, self.max_iteration + 1);

        let mut e = DVector::zeros(self.max_iteration + 1);
        e[0] = 1.0;

        let mut ys_pre: Option<DVector<f64>> = None;

        let mut du_1_new = du_1.clone();
        let mut du_2_new = du_2.clone();
        let mut ddummy_u_1_new = ddummy_u_1.clone();
        let mut ddummy_u_2_new = ddummy_u_2.clone();
        let mut draw_1_new = draw_1.clone();
        let mut draw_2_new = draw_2.clone();

        for i in 0..self.max_iteration {
            // Extract du components from vs column i
            let mut du_1_i = vec![0.0; self.n];
            let mut du_2_i = vec![0.0; self.n];
            let mut ddummy_u_1_i = vec![0.0; self.n];
            let mut ddummy_u_2_i = vec![0.0; self.n];
            let mut draw_1_i = vec![0.0; self.n];
            let mut draw_2_i = vec![0.0; self.n];

            for k in 0..self.n {
                du_1_i[k] = vs[(k * self.input_num, i)] * self.ht;
                du_2_i[k] = vs[(k * self.input_num + 1, i)] * self.ht;
                ddummy_u_1_i[k] = vs[(k * self.input_num + 2, i)] * self.ht;
                ddummy_u_2_i[k] = vs[(k * self.input_num + 3, i)] * self.ht;
                draw_1_i[k] = vs[(k * self.input_num + 4, i)] * self.ht;
                draw_2_i[k] = vs[(k * self.input_num + 5, i)] * self.ht;
            }

            // Perturb inputs
            let u_1s_i: Vec<f64> = self.u_1s.iter().zip(du_1_i.iter()).map(|(&u, &d)| u + d).collect();
            let u_2s_i: Vec<f64> = self.u_2s.iter().zip(du_2_i.iter()).map(|(&u, &d)| u + d).collect();

            let (_, _, _, v_s_i, _, _, lam_3s_i, lam_4s_i) =
                self.simulator.calc_predict_and_adjoint_state(
                    x + dx_1,
                    y + dx_2,
                    yaw + dx_3,
                    v + dx_4,
                    &u_1s_i,
                    &u_2s_i,
                    self.n,
                    dt,
                );

            let dummy_u_1s_i: Vec<f64> = self.dummy_u_1s.iter().zip(ddummy_u_1_i.iter()).map(|(&u, &d)| u + d).collect();
            let dummy_u_2s_i: Vec<f64> = self.dummy_u_2s.iter().zip(ddummy_u_2_i.iter()).map(|(&u, &d)| u + d).collect();
            let raw_1s_i: Vec<f64> = self.raw_1s.iter().zip(draw_1_i.iter()).map(|(&u, &d)| u + d).collect();
            let raw_2s_i: Vec<f64> = self.raw_2s.iter().zip(draw_2_i.iter()).map(|(&u, &d)| u + d).collect();

            let fuxt_i = Self::calc_f(
                &v_s_i,
                &lam_3s_i,
                &lam_4s_i,
                &u_1s_i,
                &u_2s_i,
                &dummy_u_1s_i,
                &dummy_u_2s_i,
                &raw_1s_i,
                &raw_2s_i,
                self.n,
            );

            // Av = (Fuxt_i - Fxt) / ht
            let av: Vec<f64> = fuxt_i
                .iter()
                .zip(fxt.iter())
                .map(|(&f_i, &fxt_i)| (f_i - fxt_i) / self.ht)
                .collect();

            // Gram-Schmidt orthonormalization
            let mut sum_av = vec![0.0; self.max_iteration];
            for j in 0..=i {
                let mut dot_product = 0.0;
                for k in 0..self.max_iteration {
                    dot_product += av[k] * vs[(k, j)];
                }
                hs[(j, i)] = dot_product;
                for k in 0..self.max_iteration {
                    sum_av[k] += hs[(j, i)] * vs[(k, j)];
                }
            }

            // v_est = Av - sum_Av
            let v_est: Vec<f64> = av.iter().zip(sum_av.iter()).map(|(&a, &s)| a - s).collect();
            let v_est_norm = vec_norm(&v_est);
            hs[(i + 1, i)] = v_est_norm;

            // Normalize and store
            for k in 0..self.max_iteration {
                vs[(k, i + 1)] = v_est[k] / v_est_norm;
            }

            // Solve least squares: hs[:i+1, :i] * ys = r0_norm * e[:i+1]
            // Skip first iteration since we need at least 1 column
            if i == 0 {
                ys_pre = Some(DVector::zeros(1));
                continue;
            }

            let hs_sub = hs.view((0, 0), (i + 1, i));
            let e_sub = e.rows(0, i + 1) * r0_norm;

            // Use pseudo-inverse for least squares solution
            let ys = pseudo_inverse_solve(&hs_sub.clone_owned(), &e_sub);

            // Check convergence
            let hs_ys = &hs_sub * &ys.rows(0, i);
            let judge = &e_sub - &hs_ys;
            let judge_norm = judge.norm();

            let converged = judge_norm < self.threshold;
            let max_iter_reached = i == self.max_iteration - 1;

            if converged || max_iter_reached {
                if let Some(ref ys_p) = ys_pre {
                    // Compute update
                    let update_len = i.saturating_sub(1);
                    if update_len > 0 && ys_p.len() >= update_len {
                        let vs_sub = vs.view((0, 0), (self.max_iteration, update_len));
                        let ys_sub = ys_p.rows(0, update_len);
                        let update_val = &vs_sub * &ys_sub;

                        for k in 0..self.n {
                            du_1_new[k] = du_1_i[k] + update_val[k * self.input_num];
                            du_2_new[k] = du_2_i[k] + update_val[k * self.input_num + 1];
                            ddummy_u_1_new[k] = ddummy_u_1_i[k] + update_val[k * self.input_num + 2];
                            ddummy_u_2_new[k] = ddummy_u_2_i[k] + update_val[k * self.input_num + 3];
                            draw_1_new[k] = draw_1_i[k] + update_val[k * self.input_num + 4];
                            draw_2_new[k] = draw_2_i[k] + update_val[k * self.input_num + 5];
                        }
                    }
                }
                break;
            }

            ys_pre = Some(ys);
        }

        // Update control inputs
        for i in 0..self.n {
            self.u_1s[i] += du_1_new[i] * self.ht;
            self.u_2s[i] += du_2_new[i] * self.ht;
            self.dummy_u_1s[i] += ddummy_u_1_new[i] * self.ht;
            self.dummy_u_2s[i] += ddummy_u_2_new[i] * self.ht;
            self.raw_1s[i] += draw_1_new[i] * self.ht;
            self.raw_2s[i] += draw_2_new[i] * self.ht;
        }

        // Calculate final F norm for monitoring
        let (_, _, _, v_s_final, _, _, lam_3s_final, lam_4s_final) =
            self.simulator.calc_predict_and_adjoint_state(
                x, y, yaw, v, &self.u_1s, &self.u_2s, self.n, dt,
            );

        let f_final = Self::calc_f(
            &v_s_final,
            &lam_3s_final,
            &lam_4s_final,
            &self.u_1s,
            &self.u_2s,
            &self.dummy_u_1s,
            &self.dummy_u_2s,
            &self.raw_1s,
            &self.raw_2s,
            self.n,
        );

        let f_norm = vec_norm(&f_final);
        println!("norm(F) = {:.6}", f_norm);

        // Save history
        self.history_f.push(f_norm);
        self.history_u_1.push(self.u_1s[0]);
        self.history_u_2.push(self.u_2s[0]);
        self.history_dummy_u_1.push(self.dummy_u_1s[0]);
        self.history_dummy_u_2.push(self.dummy_u_2s[0]);
        self.history_raw_1.push(self.raw_1s[0]);
        self.history_raw_2.push(self.raw_2s[0]);

        (self.u_1s.clone(), self.u_2s.clone())
    }

    /// Calculate the optimality condition F (Hamiltonian gradient and constraints)
    fn calc_f(
        v_s: &[f64],
        lam_3s: &[f64],
        lam_4s: &[f64],
        u_1s: &[f64],
        u_2s: &[f64],
        dummy_u_1s: &[f64],
        dummy_u_2s: &[f64],
        raw_1s: &[f64],
        raw_2s: &[f64],
        n: usize,
    ) -> Vec<f64> {
        let mut f = Vec::with_capacity(6 * n);

        for i in 0..n {
            // ∂H/∂u_1: Hamiltonian gradient w.r.t. acceleration
            // H_u1 = u_1 + lambda_4 + 2 * raw_1 * u_1 = 0
            f.push(u_1s[i] + lam_4s[i] + 2.0 * raw_1s[i] * u_1s[i]);

            // ∂H/∂u_2: Hamiltonian gradient w.r.t. steering
            // H_u2 = u_2 + lambda_3 * v / WB * cos(u_2)^2 + 2 * raw_2 * u_2 = 0
            // Note: derivative of sin(u_2) is cos(u_2), but we have cos^2 from chain rule
            f.push(
                u_2s[i]
                    + lam_3s[i] * v_s[i] / WB * u_2s[i].cos().powi(2)
                    + 2.0 * raw_2s[i] * u_2s[i],
            );

            // ∂H/∂dummy_u_1: -phi_v + 2 * raw_1 * dummy_u_1 = 0
            f.push(-PHI_V + 2.0 * raw_1s[i] * dummy_u_1s[i]);

            // ∂H/∂dummy_u_2: -phi_omega + 2 * raw_2 * dummy_u_2 = 0
            f.push(-PHI_OMEGA + 2.0 * raw_2s[i] * dummy_u_2s[i]);

            // Constraint C_1: u_1^2 + dummy_u_1^2 - U_A_MAX^2 = 0
            f.push(u_1s[i].powi(2) + dummy_u_1s[i].powi(2) - U_A_MAX.powi(2));

            // Constraint C_2: u_2^2 + dummy_u_2^2 - U_OMEGA_MAX^2 = 0
            f.push(u_2s[i].powi(2) + dummy_u_2s[i].powi(2) - U_OMEGA_MAX.powi(2));
        }

        f
    }
}

impl Default for NMPCControllerCGMRES {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate vector L2 norm
fn vec_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// Pseudo-inverse based least squares solver
fn pseudo_inverse_solve(a: &DMatrix<f64>, b: &DVector<f64>) -> DVector<f64> {
    // Use SVD for pseudo-inverse
    let svd = a.clone().svd(true, true);
    if let Ok(solution) = svd.solve(b, 1e-10) {
        return solution;
    }

    // Fallback: return zeros if SVD fails
    DVector::zeros(a.ncols())
}

/// Plot car shape for visualization
fn plot_car(
    fig: &mut Figure,
    x: f64,
    y: f64,
    yaw: f64,
    history_x: &[f64],
    history_y: &[f64],
    time: f64,
    accel: f64,
    v: f64,
) {
    // Vehicle parameters
    let length = 0.4;
    let width = 0.2;
    let back_to_wheel = 0.1;

    // Calculate vehicle outline corners
    let corners = [
        (-back_to_wheel, width / 2.0),
        (length - back_to_wheel, width / 2.0),
        (length - back_to_wheel, -width / 2.0),
        (-back_to_wheel, -width / 2.0),
        (-back_to_wheel, width / 2.0),
    ];

    // Rotate and translate corners
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    let rotated: Vec<(f64, f64)> = corners
        .iter()
        .map(|(cx, cy)| {
            let rx = cx * cos_yaw - cy * sin_yaw + x;
            let ry = cx * sin_yaw + cy * cos_yaw + y;
            (rx, ry)
        })
        .collect();

    let car_x: Vec<f64> = rotated.iter().map(|(x, _)| *x).collect();
    let car_y: Vec<f64> = rotated.iter().map(|(_, y)| *y).collect();

    fig.clear_axes();
    fig.axes2d()
        .set_title(
            &format!(
                "CGMRES NMPC - Time: {:.2}s, Accel: {:.2} m/s², Speed: {:.2} m/s",
                time, accel, v
            ),
            &[],
        )
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
        .lines(history_x, history_y, &[Caption("Trajectory"), Color("red")])
        .lines(&car_x, &car_y, &[Caption("Vehicle"), Color("black")])
        .points(&[x], &[y], &[Color("blue"), PointSymbol('*'), PointSize(2.0)])
        .points(&[0.0], &[0.0], &[Caption("Goal"), Color("green"), PointSymbol('O'), PointSize(2.0)]);

    fig.show_and_keep_running().ok();
}

pub fn main() {
    println!("CGMRES Nonlinear MPC simulation start!");

    // Simulation parameters
    let dt = 0.1;
    let iteration_time = 150.0; // [s]

    // Initial state
    let init_x = -4.5;
    let init_y = -2.5;
    let init_yaw = PI / 4.0; // 45 degrees
    let init_v = -1.0;

    // Create plant system
    let mut plant_system = TwoWheeledSystem::new(init_x, init_y, init_yaw, init_v);

    // Create controller
    let mut controller = NMPCControllerCGMRES::new();

    let iteration_num = (iteration_time / dt) as usize;

    let mut fig = Figure::new();

    println!("Starting simulation for {} iterations...", iteration_num);

    for i in 1..iteration_num {
        let time = (i as f64) * dt;

        // Calculate control input
        let (u_1s, u_2s) = controller.calc_input(
            plant_system.x,
            plant_system.y,
            plant_system.yaw,
            plant_system.v,
            time,
        );

        // Update plant state with first control input
        plant_system.update_state(u_1s[0], u_2s[0], dt);

        // Animation (optional)
        if SHOW_ANIMATION && i % 20 == 0 {
            plot_car(
                &mut fig,
                plant_system.x,
                plant_system.y,
                plant_system.yaw,
                &plant_system.history_x,
                &plant_system.history_y,
                time,
                u_1s[0],
                plant_system.v,
            );
        }

        // Check if goal reached (near origin)
        let dist_to_goal = (plant_system.x.powi(2) + plant_system.y.powi(2)).sqrt();
        if dist_to_goal < 0.5 && plant_system.v.abs() < 0.5 {
            println!("Goal reached at iteration {}!", i);
            break;
        }
    }

    println!("\nSimulation completed!");
    println!(
        "Final position: ({:.3}, {:.3}), yaw: {:.3} rad, v: {:.3} m/s",
        plant_system.x, plant_system.y, plant_system.yaw, plant_system.v
    );

    // Save final trajectory plot
    fig.clear_axes();
    fig.axes2d()
        .set_title("CGMRES NMPC - Trajectory", &[])
        .set_x_label("x [m]", &[])
        .set_y_label("y [m]", &[])
        .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
        .lines(
            &plant_system.history_x,
            &plant_system.history_y,
            &[Caption("Trajectory"), Color("red")],
        )
        .points(
            &[plant_system.history_x[0]],
            &[plant_system.history_y[0]],
            &[Caption("Start"), Color("blue"), PointSymbol('s'), PointSize(2.0)],
        )
        .points(
            &[0.0],
            &[0.0],
            &[Caption("Goal"), Color("green"), PointSymbol('O'), PointSize(2.0)],
        );

    if let Err(e) = fig.save_to_svg("./img/path_tracking/cgmres_nmpc.svg", 800, 600) {
        eprintln!("Failed to save plot: {}", e);
    } else {
        println!("Plot saved to ./img/path_tracking/cgmres_nmpc.svg");
    }

    // Plot state history
    let mut fig_state = Figure::new();
    let time_vec: Vec<f64> = (0..plant_system.history_x.len())
        .map(|i| i as f64 * dt)
        .collect();

    fig_state.axes2d()
        .set_title("State History - Position", &[])
        .set_x_label("Time [s]", &[])
        .set_y_label("Position [m]", &[])
        .lines(&time_vec, &plant_system.history_x, &[Caption("x"), Color("red")])
        .lines(&time_vec, &plant_system.history_y, &[Caption("y"), Color("blue")]);

    if let Err(e) = fig_state.save_to_svg("./img/path_tracking/cgmres_nmpc_state.svg", 800, 600) {
        eprintln!("Failed to save state plot: {}", e);
    } else {
        println!("State plot saved to ./img/path_tracking/cgmres_nmpc_state.svg");
    }

    // Plot control history
    if !controller.history_u_1.is_empty() {
        let mut fig_control = Figure::new();
        let control_time: Vec<f64> = (0..controller.history_u_1.len())
            .map(|i| (i + 1) as f64 * dt)
            .collect();

        fig_control.axes2d()
            .set_title("Control Input History", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("Control Input", &[])
            .lines(&control_time, &controller.history_u_1, &[Caption("Acceleration"), Color("red")])
            .lines(&control_time, &controller.history_u_2, &[Caption("Steering"), Color("blue")]);

        if let Err(e) = fig_control.save_to_svg("./img/path_tracking/cgmres_nmpc_control.svg", 800, 600) {
            eprintln!("Failed to save control plot: {}", e);
        } else {
            println!("Control plot saved to ./img/path_tracking/cgmres_nmpc_control.svg");
        }

        // Plot optimality error
        let mut fig_error = Figure::new();
        fig_error.axes2d()
            .set_title("Optimality Error (norm(F))", &[])
            .set_x_label("Time [s]", &[])
            .set_y_label("norm(F)", &[])
            .lines(&control_time, &controller.history_f, &[Caption("norm(F)"), Color("red")]);

        if let Err(e) = fig_error.save_to_svg("./img/path_tracking/cgmres_nmpc_error.svg", 800, 600) {
            eprintln!("Failed to save error plot: {}", e);
        } else {
            println!("Error plot saved to ./img/path_tracking/cgmres_nmpc_error.svg");
        }
    }

    println!("\nDone!");
}
