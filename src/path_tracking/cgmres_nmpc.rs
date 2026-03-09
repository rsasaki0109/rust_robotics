#![allow(dead_code, clippy::too_many_arguments, clippy::type_complexity)]

// Nonlinear MPC simulation with CGMRES (Continuation GMRES)
//
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109) - Rust port
//
// Reference:
// - PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
// - Shunichi09/nonlinear_control: Implementing the nonlinear model predictive
//   control, sliding mode control https://github.com/Shunichi09/PythonLinearNonlinearControl
//
// Note: This implementation may exhibit numerical instability in some scenarios.
// The C-GMRES algorithm is known to be sensitive to parameter tuning.
// For production use, consider adjusting zeta, ht, threshold, and other parameters.

use nalgebra::{DMatrix, DVector};
use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;

// System parameters
const U_A_MAX: f64 = 1.0; // Maximum acceleration input
const U_OMEGA_MAX: f64 = PI / 4.0; // Maximum steering angle (45 degrees)
const PHI_V: f64 = 0.01; // Penalty weight for acceleration
const PHI_OMEGA: f64 = 0.01; // Penalty weight for steering
const WB: f64 = 0.25; // Wheelbase [m]

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
    ) -> (
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
        Vec<f64>,
    ) {
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
            let (dx, dy, d_yaw, dv) = differential_model(v_s[i], yaw_s[i], u_1s[i], u_2s[i]);

            x_s.push(x_s[i] + dt * dx);
            y_s.push(y_s[i] + dt * dy);
            yaw_s.push(yaw_s[i] + dt * d_yaw);
            v_s.push(v_s[i] + dt * dv);
        }

        (x_s, y_s, yaw_s, v_s)
    }

    /// Backward adjoint state calculation (returns N elements)
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
        // Initialize with terminal conditions (terminal cost gradient = final state)
        // Python: lam_1s = [x_s[-1]], etc.
        let mut lam_1s = vec![x_s[n]];
        let mut lam_2s = vec![y_s[n]];
        let mut lam_3s = vec![yaw_s[n]];
        let mut lam_4s = vec![v_s[n]];

        // Backward integration: for i in range(N-1, 0, -1)
        // This gives N-1 iterations, resulting in N total lambda values
        for i in (1..n).rev() {
            let yaw = yaw_s[i];
            let v = v_s[i];
            let u_2 = u_2s[i];
            let lam_1 = lam_1s[0];
            let lam_2 = lam_2s[0];
            let lam_3 = lam_3s[0];
            let lam_4 = lam_4s[0];

            // Adjoint equations (backward integration using Euler method)
            // ∂H/∂x = 0, ∂H/∂y = 0
            let pre_lam_1 = lam_1;
            let pre_lam_2 = lam_2;

            // tmp1 = -lam_1 * sin(yaw) * v + lam_2 * cos(yaw) * v
            let tmp1 = -lam_1 * yaw.sin() * v + lam_2 * yaw.cos() * v;
            let pre_lam_3 = lam_3 + dt * tmp1;

            // tmp2 = lam_1 * cos(yaw) + lam_2 * sin(yaw) + lam_3 * sin(u_2) / WB
            let tmp2 = lam_1 * yaw.cos() + lam_2 * yaw.sin() + lam_3 * u_2.sin() / WB;
            let pre_lam_4 = lam_4 + dt * tmp2;

            lam_1s.insert(0, pre_lam_1);
            lam_2s.insert(0, pre_lam_2);
            lam_3s.insert(0, pre_lam_3);
            lam_4s.insert(0, pre_lam_4);
        }

        (lam_1s, lam_2s, lam_3s, lam_4s)
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
    pub zeta: f64,      // Stability gain
    pub ht: f64,        // Finite difference step
    pub tf: f64,        // Prediction horizon time
    pub alpha: f64,     // Time scaling
    pub n: usize,       // Prediction steps
    pub threshold: f64, // Convergence threshold
    pub input_num: usize,
    pub max_iteration: usize,

    simulator: NMPCSimulatorSystem,

    // Control inputs
    pub u_1s: Vec<f64>,
    pub u_2s: Vec<f64>,
    pub dummy_u_1s: Vec<f64>,
    pub dummy_u_2s: Vec<f64>,
    pub raw_1s: Vec<f64>,
    pub raw_2s: Vec<f64>,

    // History
    pub history_u_1: Vec<f64>,
    pub history_u_2: Vec<f64>,
    pub history_f: Vec<f64>,
}

impl NMPCControllerCGMRES {
    pub fn new() -> Self {
        let n = 10;
        let input_num = 6;

        NMPCControllerCGMRES {
            zeta: 100.0, // Original value
            ht: 0.01,
            tf: 3.0,
            alpha: 0.5,
            n,
            threshold: 0.001, // Same as Python
            input_num,
            max_iteration: input_num * n,

            simulator: NMPCSimulatorSystem::new(),

            // Initialize like Python: u = np.ones(N), raw = np.zeros(N)
            u_1s: vec![1.0; n],
            u_2s: vec![1.0; n],
            dummy_u_1s: vec![1.0; n],
            dummy_u_2s: vec![1.0; n],
            raw_1s: vec![0.0; n],
            raw_2s: vec![0.0; n],

            history_u_1: Vec::new(),
            history_u_2: Vec::new(),
            history_f: Vec::new(),
        }
    }

    /// Calculate control input using C-GMRES algorithm
    pub fn calc_input(
        &mut self,
        x: f64,
        y: f64,
        yaw: f64,
        v: f64,
        time: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        // dt increases with time for stability
        let dt = self.tf * (1.0 - (-self.alpha * time).exp()) / (self.n as f64);

        // x_dot
        let (x_1_dot, x_2_dot, x_3_dot, x_4_dot) =
            differential_model(v, yaw, self.u_1s[0], self.u_2s[0]);

        let dx_1 = x_1_dot * self.ht;
        let dx_2 = x_2_dot * self.ht;
        let dx_3 = x_3_dot * self.ht;
        let dx_4 = x_4_dot * self.ht;

        // Fxt: F(U, x + h*x_dot, t + h)
        let (_, _, _, v_s_fxt, _, _, lam_3s_fxt, lam_4s_fxt) =
            self.simulator.calc_predict_and_adjoint_state(
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
            &v_s_fxt,
            &lam_3s_fxt,
            &lam_4s_fxt,
            &self.u_1s,
            &self.u_2s,
            &self.dummy_u_1s,
            &self.dummy_u_2s,
            &self.raw_1s,
            &self.raw_2s,
            self.n,
        );

        // F: F(U, x, t)
        let (_, _, _, v_s_f, _, _, lam_3s_f, lam_4s_f) = self
            .simulator
            .calc_predict_and_adjoint_state(x, y, yaw, v, &self.u_1s, &self.u_2s, self.n, dt);

        let f = Self::calc_f(
            &v_s_f,
            &lam_3s_f,
            &lam_4s_f,
            &self.u_1s,
            &self.u_2s,
            &self.dummy_u_1s,
            &self.dummy_u_2s,
            &self.raw_1s,
            &self.raw_2s,
            self.n,
        );

        // right = -zeta * F - (Fxt - F) / ht
        let right: Vec<f64> = f
            .iter()
            .zip(fxt.iter())
            .map(|(&f_i, &fxt_i)| -self.zeta * f_i - (fxt_i - f_i) / self.ht)
            .collect();

        // du = u * ht (initial perturbation)
        let du_1: Vec<f64> = self.u_1s.iter().map(|&u| u * self.ht).collect();
        let du_2: Vec<f64> = self.u_2s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_1: Vec<f64> = self.dummy_u_1s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_2: Vec<f64> = self.dummy_u_2s.iter().map(|&u| u * self.ht).collect();
        let draw_1: Vec<f64> = self.raw_1s.iter().map(|&u| u * self.ht).collect();
        let draw_2: Vec<f64> = self.raw_2s.iter().map(|&u| u * self.ht).collect();

        // Fuxt: F(U + h*dU(0), x + h*x_dot, t + h)
        let u_1s_pert: Vec<f64> = self.u_1s.iter().zip(&du_1).map(|(&u, &d)| u + d).collect();
        let u_2s_pert: Vec<f64> = self.u_2s.iter().zip(&du_2).map(|(&u, &d)| u + d).collect();

        let (_, _, _, v_s_fuxt, _, _, lam_3s_fuxt, lam_4s_fuxt) =
            self.simulator.calc_predict_and_adjoint_state(
                x + dx_1,
                y + dx_2,
                yaw + dx_3,
                v + dx_4,
                &u_1s_pert,
                &u_2s_pert,
                self.n,
                dt,
            );

        let dummy_1s_pert: Vec<f64> = self
            .dummy_u_1s
            .iter()
            .zip(&ddummy_u_1)
            .map(|(&u, &d)| u + d)
            .collect();
        let dummy_2s_pert: Vec<f64> = self
            .dummy_u_2s
            .iter()
            .zip(&ddummy_u_2)
            .map(|(&u, &d)| u + d)
            .collect();
        let raw_1s_pert: Vec<f64> = self
            .raw_1s
            .iter()
            .zip(&draw_1)
            .map(|(&u, &d)| u + d)
            .collect();
        let raw_2s_pert: Vec<f64> = self
            .raw_2s
            .iter()
            .zip(&draw_2)
            .map(|(&u, &d)| u + d)
            .collect();

        let fuxt = Self::calc_f(
            &v_s_fuxt,
            &lam_3s_fuxt,
            &lam_4s_fuxt,
            &u_1s_pert,
            &u_2s_pert,
            &dummy_1s_pert,
            &dummy_2s_pert,
            &raw_1s_pert,
            &raw_2s_pert,
            self.n,
        );

        // left = (Fuxt - Fxt) / ht
        let left: Vec<f64> = fuxt
            .iter()
            .zip(&fxt)
            .map(|(&fuxt_i, &fxt_i)| (fuxt_i - fxt_i) / self.ht)
            .collect();

        // r0 = right - left
        let r0: Vec<f64> = right.iter().zip(&left).map(|(&r, &l)| r - l).collect();
        let r0_norm = vec_norm(&r0);

        // GMRES
        let m = self.max_iteration;
        let mut vs = DMatrix::zeros(m, m + 1);
        for i in 0..m {
            vs[(i, 0)] = r0[i] / r0_norm;
        }

        let mut hs = DMatrix::zeros(m + 1, m + 1);

        let mut ys_pre: Option<DVector<f64>> = None;

        // Initialize as None (like Python), will be set when converged
        let mut du_1_new: Option<Vec<f64>> = None;
        let mut du_2_new: Option<Vec<f64>> = None;
        let mut ddummy_u_1_new: Option<Vec<f64>> = None;
        let mut ddummy_u_2_new: Option<Vec<f64>> = None;
        let mut draw_1_new: Option<Vec<f64>> = None;
        let mut draw_2_new: Option<Vec<f64>> = None;

        for i in 0..m {
            // Extract du from vs column i (like Python's vs[::input_num, i])
            // These are redefined each iteration, not using the initial du_1, du_2
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

            let u_1s_i: Vec<f64> = self
                .u_1s
                .iter()
                .zip(&du_1_i)
                .map(|(&u, &d)| u + d)
                .collect();
            let u_2s_i: Vec<f64> = self
                .u_2s
                .iter()
                .zip(&du_2_i)
                .map(|(&u, &d)| u + d)
                .collect();

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

            let dummy_1s_i: Vec<f64> = self
                .dummy_u_1s
                .iter()
                .zip(&ddummy_u_1_i)
                .map(|(&u, &d)| u + d)
                .collect();
            let dummy_2s_i: Vec<f64> = self
                .dummy_u_2s
                .iter()
                .zip(&ddummy_u_2_i)
                .map(|(&u, &d)| u + d)
                .collect();
            let raw_1s_i: Vec<f64> = self
                .raw_1s
                .iter()
                .zip(&draw_1_i)
                .map(|(&u, &d)| u + d)
                .collect();
            let raw_2s_i: Vec<f64> = self
                .raw_2s
                .iter()
                .zip(&draw_2_i)
                .map(|(&u, &d)| u + d)
                .collect();

            let fuxt_i = Self::calc_f(
                &v_s_i,
                &lam_3s_i,
                &lam_4s_i,
                &u_1s_i,
                &u_2s_i,
                &dummy_1s_i,
                &dummy_2s_i,
                &raw_1s_i,
                &raw_2s_i,
                self.n,
            );

            // Av = (Fuxt_i - Fxt) / ht
            let av: Vec<f64> = fuxt_i
                .iter()
                .zip(&fxt)
                .map(|(&fi, &fxti)| (fi - fxti) / self.ht)
                .collect();

            // Gram-Schmidt orthonormalization
            let mut sum_av = vec![0.0; m];
            for j in 0..=i {
                let mut dot = 0.0;
                for k in 0..m {
                    dot += av[k] * vs[(k, j)];
                }
                hs[(j, i)] = dot;
                for k in 0..m {
                    sum_av[k] += hs[(j, i)] * vs[(k, j)];
                }
            }

            let v_est: Vec<f64> = av.iter().zip(&sum_av).map(|(&a, &s)| a - s).collect();
            let v_est_norm = vec_norm(&v_est);
            hs[(i + 1, i)] = v_est_norm;

            // Python: vs[:, i + 1] = v_est / hs[i + 1, i]
            // No zero check in Python version
            if v_est_norm > 1e-15 {
                for k in 0..m {
                    vs[(k, i + 1)] = v_est[k] / v_est_norm;
                }
            }

            // Python: inv_hs = np.linalg.pinv(hs[:i + 1, :i])
            // Python: ys = np.dot(inv_hs, r0_norm * e[:i + 1])
            // For i=0, hs[:1, :0] is empty, pinv returns empty, ys is empty
            // We need to handle this case

            // hs_sub is (i+1) x i matrix
            if i == 0 {
                // For i=0, we can't solve the system (empty matrix)
                // Just set ys_pre and continue
                ys_pre = Some(DVector::zeros(1));
                continue;
            }

            let hs_sub = hs.view((0, 0), (i + 1, i)).clone_owned();

            // e[:i+1] with first element = 1
            let mut e_scaled = DVector::zeros(i + 1);
            e_scaled[0] = r0_norm;

            // Solve least squares via SVD (use same tolerance as numpy default)
            let ys = match hs_sub.clone().svd(true, true).solve(&e_scaled, 1e-15) {
                Ok(sol) => sol,
                Err(_) => DVector::zeros(i),
            };

            // judge_value = r0_norm * e[:i+1] - hs[:i+1, :i] @ ys[:i]
            let hs_ys = &hs_sub * &ys;
            let residual = &e_scaled - &hs_ys;
            let judge_norm = residual.norm();

            if judge_norm < self.threshold || i == m - 1 {
                // Python: update_val = np.dot(vs[:, :i-1], ys_pre[:i-1]).flatten()
                // Python: du_1_new = du_1 + update_val[::input_num]
                // IMPORTANT: du_1 here is the loop variable (vs-based), not initial du_1!
                let mut du_1_tmp = du_1_i.clone();
                let mut du_2_tmp = du_2_i.clone();
                let mut ddummy_u_1_tmp = ddummy_u_1_i.clone();
                let mut ddummy_u_2_tmp = ddummy_u_2_i.clone();
                let mut draw_1_tmp = draw_1_i.clone();
                let mut draw_2_tmp = draw_2_i.clone();

                if let Some(ref ys_p) = ys_pre {
                    // Python: i - 1 (current i minus 1)
                    let update_len = i.saturating_sub(1);
                    let update_len = update_len.min(ys_p.len());
                    if update_len > 0 {
                        let vs_sub = vs.view((0, 0), (m, update_len)).clone_owned();
                        let ys_sub = ys_p.rows(0, update_len).clone_owned();
                        let update_val = &vs_sub * &ys_sub;

                        for k in 0..self.n {
                            du_1_tmp[k] = du_1_i[k] + update_val[k * self.input_num];
                            du_2_tmp[k] = du_2_i[k] + update_val[k * self.input_num + 1];
                            ddummy_u_1_tmp[k] =
                                ddummy_u_1_i[k] + update_val[k * self.input_num + 2];
                            ddummy_u_2_tmp[k] =
                                ddummy_u_2_i[k] + update_val[k * self.input_num + 3];
                            draw_1_tmp[k] = draw_1_i[k] + update_val[k * self.input_num + 4];
                            draw_2_tmp[k] = draw_2_i[k] + update_val[k * self.input_num + 5];
                        }
                    }
                }

                du_1_new = Some(du_1_tmp);
                du_2_new = Some(du_2_tmp);
                ddummy_u_1_new = Some(ddummy_u_1_tmp);
                ddummy_u_2_new = Some(ddummy_u_2_tmp);
                draw_1_new = Some(draw_1_tmp);
                draw_2_new = Some(draw_2_tmp);
                break;
            }

            ys_pre = Some(ys);
        }

        // Update inputs (only if converged)
        if let (Some(du1), Some(du2), Some(dd1), Some(dd2), Some(dr1), Some(dr2)) = (
            du_1_new,
            du_2_new,
            ddummy_u_1_new,
            ddummy_u_2_new,
            draw_1_new,
            draw_2_new,
        ) {
            for i in 0..self.n {
                self.u_1s[i] += du1[i] * self.ht;
                self.u_2s[i] += du2[i] * self.ht;
                self.dummy_u_1s[i] += dd1[i] * self.ht;
                self.dummy_u_2s[i] += dd2[i] * self.ht;
                self.raw_1s[i] += dr1[i] * self.ht;
                self.raw_2s[i] += dr2[i] * self.ht;

                // Clip inputs to prevent numerical instability
                self.u_1s[i] = self.u_1s[i].clamp(-U_A_MAX, U_A_MAX);
                self.u_2s[i] = self.u_2s[i].clamp(-U_OMEGA_MAX, U_OMEGA_MAX);
                self.dummy_u_1s[i] = self.dummy_u_1s[i].max(0.0);
                self.dummy_u_2s[i] = self.dummy_u_2s[i].max(0.0);
            }
        }

        // Calculate final F norm
        let (_, _, _, v_s_final, _, _, lam_3s_final, lam_4s_final) = self
            .simulator
            .calc_predict_and_adjoint_state(x, y, yaw, v, &self.u_1s, &self.u_2s, self.n, dt);

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

        self.history_f.push(f_norm);
        self.history_u_1.push(self.u_1s[0]);
        self.history_u_2.push(self.u_2s[0]);

        (self.u_1s.clone(), self.u_2s.clone())
    }

    /// Calculate optimality condition F
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
            // Use min to avoid index out of bounds
            let lam_3_idx = i.min(lam_3s.len() - 1);
            let lam_4_idx = i.min(lam_4s.len() - 1);

            // ∂H/∂u_1
            f.push(u_1s[i] + lam_4s[lam_4_idx] + 2.0 * raw_1s[i] * u_1s[i]);

            // ∂H/∂u_2
            f.push(
                u_2s[i]
                    + lam_3s[lam_3_idx] * v_s[i] / WB * u_2s[i].cos().powi(2)
                    + 2.0 * raw_2s[i] * u_2s[i],
            );

            // ∂H/∂dummy_u_1
            f.push(-PHI_V + 2.0 * raw_1s[i] * dummy_u_1s[i]);

            // ∂H/∂dummy_u_2
            f.push(-PHI_OMEGA + 2.0 * raw_2s[i] * dummy_u_2s[i]);

            // C_1
            f.push(u_1s[i].powi(2) + dummy_u_1s[i].powi(2) - U_A_MAX.powi(2));

            // C_2
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

fn vec_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn build_path_data(
    xs: &[f64],
    ys: &[f64],
    to_svg_x: &impl Fn(f64) -> f64,
    to_svg_y: &impl Fn(f64) -> f64,
) -> String {
    let mut path = String::new();

    for (i, (&x, &y)) in xs.iter().zip(ys.iter()).enumerate() {
        let sx = to_svg_x(x);
        let sy = to_svg_y(y);
        if i == 0 {
            path.push_str(&format!("M {:.2},{:.2}", sx, sy));
        } else {
            path.push_str(&format!(" L {:.2},{:.2}", sx, sy));
        }
    }

    path
}

fn build_cgmres_svg(plant: &TwoWheeledSystem, controller: &NMPCControllerCGMRES) -> String {
    let width = 980.0;
    let height = 680.0;
    let margin = 62.0;
    let side_panel_width = 250.0;
    let available_plot_width = width - 2.0 * margin - side_panel_width;
    let available_plot_height = height - 2.0 * margin;

    let x_min = plant
        .history_x
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .min(0.0)
        - 0.8;
    let x_max = plant
        .history_x
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .max(0.0)
        + 0.8;
    let y_min = plant
        .history_y
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .min(0.0)
        - 0.8;
    let y_max = plant
        .history_y
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .max(0.0)
        + 0.8;

    let x_range = (x_max - x_min).max(1.0);
    let y_range = (y_max - y_min).max(1.0);
    let scale = (available_plot_width / x_range)
        .min(available_plot_height / y_range)
        .max(1.0);
    let plot_width = x_range * scale;
    let plot_height = y_range * scale;
    let plot_left = margin;
    let plot_top = margin + (available_plot_height - plot_height) / 2.0;
    let panel_x = plot_left + plot_width + 34.0;

    let to_svg_x = |x: f64| plot_left + (x - x_min) * scale;
    let to_svg_y = |y: f64| plot_top + plot_height - (y - y_min) * scale;

    let trajectory_path = build_path_data(&plant.history_x, &plant.history_y, &to_svg_x, &to_svg_y);
    let goal_distance = (plant.x.powi(2) + plant.y.powi(2)).sqrt();
    let reached_goal = goal_distance < 0.5 && plant.v.abs() < 0.5;
    let max_f = controller.history_f.iter().copied().fold(0.0, f64::max);
    let min_f = controller
        .history_f
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .min(max_f);
    let final_f = controller.history_f.last().copied().unwrap_or(0.0);

    let error_box_x = panel_x;
    let error_box_y = plot_top + 182.0;
    let error_box_w = 214.0;
    let error_box_h = 168.0;

    let mut svg = String::new();
    svg.push_str(&format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>\n"
    ));
    svg.push_str("<rect width='100%' height='100%' fill='white'/>\n");
    svg.push_str(&format!(
        "<text x='{:.1}' y='38' text-anchor='middle' font-size='24' font-family='sans-serif' font-weight='700' fill='#111827'>C-GMRES NMPC</text>\n",
        width / 2.0
    ));
    svg.push_str(&format!(
        "<rect x='{plot_left:.1}' y='{plot_top:.1}' width='{plot_width:.1}' height='{plot_height:.1}' fill='#fffdf7' stroke='#d6d3d1' stroke-width='2'/>\n"
    ));

    for i in 0..=6 {
        let x = plot_left + plot_width * (i as f64) / 6.0;
        let y = plot_top + plot_height * (i as f64) / 6.0;
        svg.push_str(&format!(
            "<line x1='{x:.2}' y1='{plot_top:.2}' x2='{x:.2}' y2='{:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_top + plot_height
        ));
        svg.push_str(&format!(
            "<line x1='{plot_left:.2}' y1='{y:.2}' x2='{:.2}' y2='{y:.2}' stroke='#ece7dc' stroke-width='1'/>\n",
            plot_left + plot_width
        ));
    }

    svg.push_str(&format!(
        "<path d='{trajectory_path}' fill='none' stroke='#dc2626' stroke-width='4.5' stroke-linecap='round' stroke-linejoin='round'/>\n"
    ));
    svg.push_str(&format!(
        "<circle cx='{:.2}' cy='{:.2}' r='7' fill='#2563eb' stroke='white' stroke-width='2'/>\n",
        to_svg_x(plant.history_x[0]),
        to_svg_y(plant.history_y[0])
    ));
    svg.push_str(&format!(
        "<circle cx='{:.2}' cy='{:.2}' r='7' fill='#7c3aed' stroke='white' stroke-width='2'/>\n",
        to_svg_x(0.0),
        to_svg_y(0.0)
    ));
    svg.push_str(&format!(
        "<circle cx='{:.2}' cy='{:.2}' r='6' fill='#f97316' stroke='white' stroke-width='2'/>\n",
        to_svg_x(plant.x),
        to_svg_y(plant.y)
    ));

    svg.push_str(&format!(
        "<rect x='{panel_x:.1}' y='{:.1}' width='214' height='122' rx='18' fill='white' stroke='#d6d3d1'/>\n",
        plot_top + 18.0
    ));
    let legend_y = plot_top + 46.0;
    let legend_items = [
        ("#dc2626", "Trajectory", "line"),
        ("#2563eb", "Start", "point"),
        ("#7c3aed", "Goal", "point"),
        ("#f97316", "Final state", "point"),
    ];
    for (idx, (color, label, kind)) in legend_items.iter().enumerate() {
        let y = legend_y + idx as f64 * 24.0;
        if *kind == "line" {
            svg.push_str(&format!(
                "<line x1='{:.1}' y1='{y:.1}' x2='{:.1}' y2='{y:.1}' stroke='{color}' stroke-width='5' stroke-linecap='round'/>\n",
                panel_x + 16.0,
                panel_x + 44.0
            ));
        } else {
            svg.push_str(&format!(
                "<circle cx='{:.1}' cy='{y:.1}' r='5.5' fill='{color}' stroke='white' stroke-width='1.5'/>\n",
                panel_x + 30.0
            ));
        }
        svg.push_str(&format!(
            "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' fill='#111827'>{label}</text>\n",
            panel_x + 58.0,
            y + 1.0
        ));
    }

    svg.push_str(&format!(
        "<rect x='{error_box_x:.1}' y='{error_box_y:.1}' width='{error_box_w:.1}' height='{error_box_h:.1}' rx='18' fill='#fffaf0' stroke='#d6d3d1'/>\n"
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' font-weight='700' fill='#111827'>Optimality Error</text>\n",
        error_box_x + 16.0,
        error_box_y + 24.0
    ));

    if !controller.history_f.is_empty() {
        let error_min = controller
            .history_f
            .iter()
            .copied()
            .map(|v| v.max(1e-9).log10())
            .fold(f64::INFINITY, f64::min);
        let error_max = controller
            .history_f
            .iter()
            .copied()
            .map(|v| v.max(1e-9).log10())
            .fold(f64::NEG_INFINITY, f64::max);
        let graph_left = error_box_x + 18.0;
        let graph_top = error_box_y + 40.0;
        let graph_width = error_box_w - 36.0;
        let graph_height = error_box_h - 62.0;
        svg.push_str(&format!(
            "<rect x='{graph_left:.1}' y='{graph_top:.1}' width='{graph_width:.1}' height='{graph_height:.1}' fill='white' stroke='#e5e7eb'/>\n"
        ));
        for i in 0..=4 {
            let y = graph_top + graph_height * (i as f64) / 4.0;
            svg.push_str(&format!(
                "<line x1='{graph_left:.1}' y1='{y:.2}' x2='{:.1}' y2='{y:.2}' stroke='#f1f5f9' stroke-width='1'/>\n",
                graph_left + graph_width
            ));
        }

        let map_error_x = |idx: usize| {
            graph_left
                + if controller.history_f.len() <= 1 {
                    0.0
                } else {
                    graph_width * (idx as f64) / ((controller.history_f.len() - 1) as f64)
                }
        };
        let map_error_y = |value: f64| {
            let log_v = value.max(1e-9).log10();
            let denom = (error_max - error_min).max(1e-6);
            graph_top + graph_height - (log_v - error_min) / denom * graph_height
        };
        let mut error_path = String::new();
        for (i, value) in controller.history_f.iter().enumerate() {
            let x = map_error_x(i);
            let y = map_error_y(*value);
            if i == 0 {
                error_path.push_str(&format!("M {:.2},{:.2}", x, y));
            } else {
                error_path.push_str(&format!(" L {:.2},{:.2}", x, y));
            }
        }
        svg.push_str(&format!(
            "<path d='{error_path}' fill='none' stroke='#16a34a' stroke-width='2.5' stroke-linecap='round' stroke-linejoin='round'/>\n"
        ));
        svg.push_str(&format!(
            "<text x='{:.1}' y='{:.1}' font-size='12' font-family='monospace' fill='#475569'>log10 ||F||</text>\n",
            graph_left,
            graph_top + graph_height + 18.0
        ));
    }

    let stats_y = error_box_y + error_box_h + 24.0;
    svg.push_str(&format!(
        "<rect x='{panel_x:.1}' y='{stats_y:.1}' width='214' height='152' rx='18' fill='white' stroke='#d6d3d1'/>\n"
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' font-size='16' font-family='sans-serif' font-weight='700' fill='#111827'>Simulation Stats</text>\n",
        panel_x + 16.0,
        stats_y + 24.0
    ));
    let stats = [
        format!("goal_reached: {}", reached_goal),
        format!("goal_distance: {:.3} m", goal_distance),
        format!("final_speed: {:.3} m/s", plant.v),
        format!("iterations: {}", plant.history_x.len().saturating_sub(1)),
        format!("final ||F||: {:.3}", final_f),
        format!("max ||F||: {:.3}", max_f.max(min_f)),
    ];
    for (idx, line) in stats.iter().enumerate() {
        svg.push_str(&format!(
            "<text x='{:.1}' y='{:.1}' font-size='13' font-family='monospace' fill='#374151'>{line}</text>\n",
            panel_x + 16.0,
            stats_y + 48.0 + idx as f64 * 18.0
        ));
    }

    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827'>x [m]</text>\n",
        plot_left + plot_width / 2.0,
        plot_top + plot_height + 52.0
    ));
    svg.push_str(&format!(
        "<text x='{:.1}' y='{:.1}' text-anchor='middle' font-size='16' font-family='sans-serif' fill='#111827' transform='rotate(-90, {:.1}, {:.1})'>y [m]</text>\n",
        plot_left - 48.0,
        plot_top + plot_height / 2.0,
        plot_left - 48.0,
        plot_top + plot_height / 2.0
    ));
    svg.push_str("</svg>\n");

    svg
}

/// Generate SVG plot without gnuplot
fn save_trajectory_svg(
    filename: &str,
    plant: &TwoWheeledSystem,
    controller: &NMPCControllerCGMRES,
) -> std::io::Result<()> {
    let svg = build_cgmres_svg(plant, controller);
    let mut file = File::create(filename)?;
    file.write_all(svg.as_bytes())?;
    Ok(())
}

fn goal_distance(plant: &TwoWheeledSystem) -> f64 {
    plant.x.hypot(plant.y)
}

fn run_demo_simulation() -> (TwoWheeledSystem, NMPCControllerCGMRES) {
    let dt = 0.1;
    let iteration_time = 15.0;

    // The original PythonRobotics initial state frequently diverges.
    // Use a stable showcase setup that still demonstrates C-GMRES steering to the origin.
    let init_x: f64 = -1.5;
    let init_y: f64 = -1.0;
    let init_yaw = (-init_y).atan2(-init_x);
    let init_v: f64 = 0.0;

    let mut plant = TwoWheeledSystem::new(init_x, init_y, init_yaw, init_v);
    let mut controller = NMPCControllerCGMRES::new();
    let iteration_num = (iteration_time / dt) as usize;

    println!("Starting simulation for {} iterations...", iteration_num);

    for i in 1..iteration_num {
        let time = (i as f64) * dt;

        let (u_1s, u_2s) = controller.calc_input(plant.x, plant.y, plant.yaw, plant.v, time);

        plant.update_state(u_1s[0], u_2s[0], dt);

        if goal_distance(&plant) < 0.5 && plant.v.abs() < 0.5 {
            println!("Goal reached at iteration {}!", i);
            break;
        }

        if controller.history_f.last().copied().unwrap_or(0.0) > 1.0e5 {
            println!(
                "Simulation clipped at iteration {} due to optimality blow-up.",
                i
            );
            break;
        }

        if !plant.x.is_finite()
            || !plant.y.is_finite()
            || !plant.yaw.is_finite()
            || !plant.v.is_finite()
            || plant.x.abs().max(plant.y.abs()) > 12.0
        {
            println!("Simulation clipped at iteration {} due to divergence.", i);
            break;
        }
    }

    (plant, controller)
}

pub fn main() {
    println!("CGMRES Nonlinear MPC simulation start!");
    let (plant, controller) = run_demo_simulation();

    println!("\nSimulation completed!");
    println!(
        "Final position: ({:.3}, {:.3}), yaw: {:.3} rad, v: {:.3} m/s",
        plant.x, plant.y, plant.yaw, plant.v
    );

    // Save SVG
    match save_trajectory_svg("./img/path_tracking/cgmres_nmpc.svg", &plant, &controller) {
        Ok(_) => println!("Plot saved to ./img/path_tracking/cgmres_nmpc.svg"),
        Err(e) => eprintln!("Failed to save plot: {}", e),
    }

    println!("\nDone!");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cgmres_svg_has_white_background_and_stats() {
        let mut plant = TwoWheeledSystem::new(-1.0, -1.0, 0.0, 0.0);
        plant.history_x = vec![-1.0, -0.5, 0.0];
        plant.history_y = vec![-1.0, -0.2, 0.0];
        plant.x = 0.0;
        plant.y = 0.0;
        plant.v = 0.1;

        let mut controller = NMPCControllerCGMRES::new();
        controller.history_f = vec![10.0, 2.0, 0.5];

        let svg = build_cgmres_svg(&plant, &controller);
        assert!(svg.contains("<rect width='100%' height='100%' fill='white'/>"));
        assert!(svg.contains("C-GMRES NMPC"));
        assert!(svg.contains("Simulation Stats"));
        assert!(svg.contains("Optimality Error"));
    }

    #[test]
    fn test_cgmres_demo_reaches_goal() {
        let (plant, controller) = run_demo_simulation();
        assert!(
            goal_distance(&plant) < 0.5,
            "final distance was {}",
            goal_distance(&plant)
        );
        assert!(plant.v.abs() < 0.5, "final speed was {}", plant.v.abs());
        assert!(
            controller
                .history_f
                .last()
                .copied()
                .unwrap_or(f64::INFINITY)
                .is_finite(),
            "final optimality error was not finite"
        );
    }
}
