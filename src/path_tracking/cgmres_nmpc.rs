// Nonlinear MPC simulation with CGMRES (Continuation GMRES)
//
// author: Atsushi Sakai (@Atsushi_twi)
//         Ryohei Sasaki (@rsasaki0109) - Rust port
//
// Reference:
// Shunichi09/nonlinear_control: Implementing the nonlinear model predictive
// control, sliding mode control https://github.com/Shunichi09/PythonLinearNonlinearControl

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

            // Adjoint equations: dλ/dt = -∂H/∂x
            // ∂H/∂x = 0, ∂H/∂y = 0
            let pre_lam_1 = lam_1;
            let pre_lam_2 = lam_2;

            // ∂H/∂yaw = -lam_1 * sin(yaw) * v + lam_2 * cos(yaw) * v
            let tmp1 = -lam_1 * yaw.sin() * v + lam_2 * yaw.cos() * v;
            let pre_lam_3 = lam_3 + dt * tmp1;

            // ∂H/∂v = lam_1 * cos(yaw) + lam_2 * sin(yaw) + lam_3 * sin(u_2) / WB
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
            zeta: 100.0,
            ht: 0.01,
            tf: 3.0,
            alpha: 0.5,
            n,
            threshold: 0.001,
            input_num,
            max_iteration: input_num * n,

            simulator: NMPCSimulatorSystem::new(),

            // Initialize with smaller values to avoid numerical instability
            u_1s: vec![0.1; n],
            u_2s: vec![0.1; n],
            dummy_u_1s: vec![0.9; n],  // sqrt(U_MAX^2 - u^2) ≈ 0.99 for u=0.1
            dummy_u_2s: vec![0.78; n], // sqrt(U_OMEGA_MAX^2 - u^2) ≈ 0.78 for u=0.1
            raw_1s: vec![0.01; n],
            raw_2s: vec![0.01; n],

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

        let mut du_1_new = du_1.clone();
        let mut du_2_new = du_2.clone();
        let mut ddummy_u_1_new = ddummy_u_1.clone();
        let mut ddummy_u_2_new = ddummy_u_2.clone();
        let mut draw_1_new = draw_1.clone();
        let mut draw_2_new = draw_2.clone();

        for i in 0..m {
            // Extract du from vs column i
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

            let u_1s_i: Vec<f64> = self.u_1s.iter().zip(&du_1_i).map(|(&u, &d)| u + d).collect();
            let u_2s_i: Vec<f64> = self.u_2s.iter().zip(&du_2_i).map(|(&u, &d)| u + d).collect();

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

            // Gram-Schmidt
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

            if v_est_norm > 1e-10 {
                for k in 0..m {
                    vs[(k, i + 1)] = v_est[k] / v_est_norm;
                }
            }

            // Solve hs[:i+2, :i+1] * ys = r0_norm * e (where e = [1, 0, 0, ...])
            let hs_sub = hs.view((0, 0), (i + 2, i + 1)).clone_owned();
            let mut e = DVector::zeros(i + 2);
            e[0] = r0_norm;

            // Use pseudoinverse to solve least squares: ys = pinv(hs_sub) * e
            let ys = match hs_sub.clone().svd(true, true).solve(&e, 1e-10) {
                Ok(sol) => sol,
                Err(_) => DVector::zeros(i + 1),
            };

            // Check convergence: judge_value = e - hs_sub * ys
            let hs_ys = &hs_sub * &ys;
            let residual = &e - &hs_ys;
            let judge_norm = residual.norm();

            if judge_norm < self.threshold || i == m - 1 {
                // Use ys_pre for final update (from previous iteration)
                if let Some(ref ys_p) = ys_pre {
                    let update_len = ys_p.len().min(i);
                    if update_len > 0 {
                        let vs_sub = vs.view((0, 0), (m, update_len)).clone_owned();
                        let ys_sub = ys_p.rows(0, update_len).clone_owned();
                        let update_val = &vs_sub * &ys_sub;

                        for k in 0..self.n {
                            du_1_new[k] = update_val[k * self.input_num];
                            du_2_new[k] = update_val[k * self.input_num + 1];
                            ddummy_u_1_new[k] = update_val[k * self.input_num + 2];
                            ddummy_u_2_new[k] = update_val[k * self.input_num + 3];
                            draw_1_new[k] = update_val[k * self.input_num + 4];
                            draw_2_new[k] = update_val[k * self.input_num + 5];
                        }
                    }
                }
                break;
            }

            ys_pre = Some(ys);
        }

        // Update inputs
        for i in 0..self.n {
            self.u_1s[i] += du_1_new[i] * self.ht;
            self.u_2s[i] += du_2_new[i] * self.ht;
            self.dummy_u_1s[i] += ddummy_u_1_new[i] * self.ht;
            self.dummy_u_2s[i] += ddummy_u_2_new[i] * self.ht;
            self.raw_1s[i] += draw_1_new[i] * self.ht;
            self.raw_2s[i] += draw_2_new[i] * self.ht;
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

/// Generate SVG plot without gnuplot
fn save_trajectory_svg(
    filename: &str,
    history_x: &[f64],
    history_y: &[f64],
    title: &str,
) -> std::io::Result<()> {
    let width = 800;
    let height = 600;
    let margin = 60;

    // Find bounds
    let x_min = history_x.iter().cloned().fold(f64::INFINITY, f64::min);
    let x_max = history_x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let y_min = history_y.iter().cloned().fold(f64::INFINITY, f64::min);
    let y_max = history_y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    // Include origin
    let x_min = x_min.min(0.0) - 0.5;
    let x_max = x_max.max(0.0) + 0.5;
    let y_min = y_min.min(0.0) - 0.5;
    let y_max = y_max.max(0.0) + 0.5;

    let plot_width = (width - 2 * margin) as f64;
    let plot_height = (height - 2 * margin) as f64;

    let scale_x = plot_width / (x_max - x_min);
    let scale_y = plot_height / (y_max - y_min);
    let scale = scale_x.min(scale_y);

    let to_svg_x = |x: f64| margin as f64 + (x - x_min) * scale;
    let to_svg_y = |y: f64| (height - margin) as f64 - (y - y_min) * scale;

    let mut file = File::create(filename)?;

    writeln!(
        file,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">
<rect width="100%" height="100%" fill="white"/>"#,
        width, height, width, height
    )?;

    // Title
    writeln!(
        file,
        r#"<text x="{}" y="30" text-anchor="middle" font-size="16" font-family="sans-serif">{}</text>"#,
        width / 2,
        title
    )?;

    // Axes
    writeln!(
        file,
        r#"<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="black" stroke-width="1"/>"#,
        margin,
        height - margin,
        width - margin,
        height - margin
    )?;
    writeln!(
        file,
        r#"<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="black" stroke-width="1"/>"#,
        margin, margin, margin, height - margin
    )?;

    // Trajectory
    if history_x.len() > 1 {
        let mut path = String::from("M");
        for (i, (&x, &y)) in history_x.iter().zip(history_y.iter()).enumerate() {
            let svg_x = to_svg_x(x);
            let svg_y = to_svg_y(y);
            if i == 0 {
                path.push_str(&format!(" {:.2},{:.2}", svg_x, svg_y));
            } else {
                path.push_str(&format!(" L {:.2},{:.2}", svg_x, svg_y));
            }
        }
        writeln!(
            file,
            r#"<path d="{}" fill="none" stroke="red" stroke-width="2"/>"#,
            path
        )?;
    }

    // Start point
    let start_x = to_svg_x(history_x[0]);
    let start_y = to_svg_y(history_y[0]);
    writeln!(
        file,
        r#"<circle cx="{:.2}" cy="{:.2}" r="6" fill="blue"/>"#,
        start_x, start_y
    )?;
    writeln!(
        file,
        r#"<text x="{:.2}" y="{:.2}" font-size="12" font-family="sans-serif">Start</text>"#,
        start_x + 10.0,
        start_y
    )?;

    // Goal (origin)
    let goal_x = to_svg_x(0.0);
    let goal_y = to_svg_y(0.0);
    writeln!(
        file,
        r#"<circle cx="{:.2}" cy="{:.2}" r="6" fill="green"/>"#,
        goal_x, goal_y
    )?;
    writeln!(
        file,
        r#"<text x="{:.2}" y="{:.2}" font-size="12" font-family="sans-serif">Goal</text>"#,
        goal_x + 10.0,
        goal_y
    )?;

    // Axis labels
    writeln!(
        file,
        r#"<text x="{}" y="{}" text-anchor="middle" font-size="12" font-family="sans-serif">x [m]</text>"#,
        width / 2,
        height - 20
    )?;
    writeln!(
        file,
        r#"<text x="20" y="{}" text-anchor="middle" font-size="12" font-family="sans-serif" transform="rotate(-90, 20, {})">y [m]</text>"#,
        height / 2,
        height / 2
    )?;

    writeln!(file, "</svg>")?;

    Ok(())
}

pub fn main() {
    println!("CGMRES Nonlinear MPC simulation start!");

    let dt = 0.1;
    let iteration_time = 150.0;

    let init_x = -4.5;
    let init_y = -2.5;
    let init_yaw = PI / 4.0;
    let init_v = -1.0;

    let mut plant = TwoWheeledSystem::new(init_x, init_y, init_yaw, init_v);
    let mut controller = NMPCControllerCGMRES::new();

    let iteration_num = (iteration_time / dt) as usize;

    println!("Starting simulation for {} iterations...", iteration_num);

    for i in 1..iteration_num {
        let time = (i as f64) * dt;

        let (u_1s, u_2s) =
            controller.calc_input(plant.x, plant.y, plant.yaw, plant.v, time);

        plant.update_state(u_1s[0], u_2s[0], dt);

        let dist = (plant.x.powi(2) + plant.y.powi(2)).sqrt();
        if dist < 0.5 && plant.v.abs() < 0.5 {
            println!("Goal reached at iteration {}!", i);
            break;
        }
    }

    println!("\nSimulation completed!");
    println!(
        "Final position: ({:.3}, {:.3}), yaw: {:.3} rad, v: {:.3} m/s",
        plant.x, plant.y, plant.yaw, plant.v
    );

    // Save SVG
    match save_trajectory_svg(
        "./img/path_tracking/cgmres_nmpc.svg",
        &plant.history_x,
        &plant.history_y,
        "CGMRES NMPC - Trajectory",
    ) {
        Ok(_) => println!("Plot saved to ./img/path_tracking/cgmres_nmpc.svg"),
        Err(e) => eprintln!("Failed to save plot: {}", e),
    }

    println!("\nDone!");
}
