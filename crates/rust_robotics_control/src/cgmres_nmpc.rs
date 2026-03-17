#![allow(dead_code, clippy::too_many_arguments, clippy::type_complexity)]

//! Nonlinear MPC simulation with CGMRES (Continuation GMRES)
//!
//! author: Atsushi Sakai (@Atsushi_twi)
//!         Ryohei Sasaki (@rsasaki0109) - Rust port
//!
//! Reference:
//! - PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
//! - Shunichi09/nonlinear_control: Implementing the nonlinear model predictive
//!   control, sliding mode control https://github.com/Shunichi09/PythonLinearNonlinearControl
//!
//! Note: This implementation may exhibit numerical instability in some scenarios.
//! The C-GMRES algorithm is known to be sensitive to parameter tuning.
//! For production use, consider adjusting zeta, ht, threshold, and other parameters.

use nalgebra::{DMatrix, DVector};
use std::f64::consts::PI;

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
        // Initialize with terminal conditions
        let mut lam_1s = vec![x_s[n]];
        let mut lam_2s = vec![y_s[n]];
        let mut lam_3s = vec![yaw_s[n]];
        let mut lam_4s = vec![v_s[n]];

        for i in (1..n).rev() {
            let yaw = yaw_s[i];
            let v = v_s[i];
            let u_2 = u_2s[i];
            let lam_1 = lam_1s[0];
            let lam_2 = lam_2s[0];
            let lam_3 = lam_3s[0];
            let lam_4 = lam_4s[0];

            let pre_lam_1 = lam_1;
            let pre_lam_2 = lam_2;

            let tmp1 = -lam_1 * yaw.sin() * v + lam_2 * yaw.cos() * v;
            let pre_lam_3 = lam_3 + dt * tmp1;

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
    pub zeta: f64,
    pub ht: f64,
    pub tf: f64,
    pub alpha: f64,
    pub n: usize,
    pub threshold: f64,
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
        let dt = self.tf * (1.0 - (-self.alpha * time).exp()) / (self.n as f64);

        let (x_1_dot, x_2_dot, x_3_dot, x_4_dot) =
            differential_model(v, yaw, self.u_1s[0], self.u_2s[0]);

        let dx_1 = x_1_dot * self.ht;
        let dx_2 = x_2_dot * self.ht;
        let dx_3 = x_3_dot * self.ht;
        let dx_4 = x_4_dot * self.ht;

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

        let right: Vec<f64> = f
            .iter()
            .zip(fxt.iter())
            .map(|(&f_i, &fxt_i)| -self.zeta * f_i - (fxt_i - f_i) / self.ht)
            .collect();

        let du_1: Vec<f64> = self.u_1s.iter().map(|&u| u * self.ht).collect();
        let du_2: Vec<f64> = self.u_2s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_1: Vec<f64> = self.dummy_u_1s.iter().map(|&u| u * self.ht).collect();
        let ddummy_u_2: Vec<f64> = self.dummy_u_2s.iter().map(|&u| u * self.ht).collect();
        let draw_1: Vec<f64> = self.raw_1s.iter().map(|&u| u * self.ht).collect();
        let draw_2: Vec<f64> = self.raw_2s.iter().map(|&u| u * self.ht).collect();

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

        let left: Vec<f64> = fuxt
            .iter()
            .zip(&fxt)
            .map(|(&fuxt_i, &fxt_i)| (fuxt_i - fxt_i) / self.ht)
            .collect();

        let r0: Vec<f64> = right.iter().zip(&left).map(|(&r, &l)| r - l).collect();
        let r0_norm = vec_norm(&r0);

        let m = self.max_iteration;
        let mut vs = DMatrix::zeros(m, m + 1);
        for i in 0..m {
            vs[(i, 0)] = r0[i] / r0_norm;
        }

        let mut hs = DMatrix::zeros(m + 1, m + 1);
        let mut ys_pre: Option<DVector<f64>> = None;

        let mut du_1_new: Option<Vec<f64>> = None;
        let mut du_2_new: Option<Vec<f64>> = None;
        let mut ddummy_u_1_new: Option<Vec<f64>> = None;
        let mut ddummy_u_2_new: Option<Vec<f64>> = None;
        let mut draw_1_new: Option<Vec<f64>> = None;
        let mut draw_2_new: Option<Vec<f64>> = None;

        for i in 0..m {
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

            let av: Vec<f64> = fuxt_i
                .iter()
                .zip(&fxt)
                .map(|(&fi, &fxti)| (fi - fxti) / self.ht)
                .collect();

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

            if v_est_norm > 1e-15 {
                for k in 0..m {
                    vs[(k, i + 1)] = v_est[k] / v_est_norm;
                }
            }

            if i == 0 {
                ys_pre = Some(DVector::zeros(1));
                continue;
            }

            let hs_sub = hs.view((0, 0), (i + 1, i)).clone_owned();

            let mut e_scaled = DVector::zeros(i + 1);
            e_scaled[0] = r0_norm;

            let ys = match hs_sub.clone().svd(true, true).solve(&e_scaled, 1e-15) {
                Ok(sol) => sol,
                Err(_) => DVector::zeros(i),
            };

            let hs_ys = &hs_sub * &ys;
            let residual = &e_scaled - &hs_ys;
            let judge_norm = residual.norm();

            if judge_norm < self.threshold || i == m - 1 {
                let mut du_1_tmp = du_1_i.clone();
                let mut du_2_tmp = du_2_i.clone();
                let mut ddummy_u_1_tmp = ddummy_u_1_i.clone();
                let mut ddummy_u_2_tmp = ddummy_u_2_i.clone();
                let mut draw_1_tmp = draw_1_i.clone();
                let mut draw_2_tmp = draw_2_i.clone();

                if let Some(ref ys_p) = ys_pre {
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
            let lam_3_idx = i.min(lam_3s.len() - 1);
            let lam_4_idx = i.min(lam_4s.len() - 1);

            f.push(u_1s[i] + lam_4s[lam_4_idx] + 2.0 * raw_1s[i] * u_1s[i]);

            f.push(
                u_2s[i]
                    + lam_3s[lam_3_idx] * v_s[i] / WB * u_2s[i].cos().powi(2)
                    + 2.0 * raw_2s[i] * u_2s[i],
            );

            f.push(-PHI_V + 2.0 * raw_1s[i] * dummy_u_1s[i]);
            f.push(-PHI_OMEGA + 2.0 * raw_2s[i] * dummy_u_2s[i]);
            f.push(u_1s[i].powi(2) + dummy_u_1s[i].powi(2) - U_A_MAX.powi(2));
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

pub fn goal_distance(plant: &TwoWheeledSystem) -> f64 {
    plant.x.hypot(plant.y)
}

pub fn run_demo_simulation() -> (TwoWheeledSystem, NMPCControllerCGMRES) {
    let dt = 0.1;
    let iteration_time = 15.0;

    let init_x: f64 = -1.5;
    let init_y: f64 = -1.0;
    let init_yaw = (-init_y).atan2(-init_x);
    let init_v: f64 = 0.0;

    let mut plant = TwoWheeledSystem::new(init_x, init_y, init_yaw, init_v);
    let mut controller = NMPCControllerCGMRES::new();
    let iteration_num = (iteration_time / dt) as usize;

    for i in 1..iteration_num {
        let time = (i as f64) * dt;

        let (u_1s, u_2s) = controller.calc_input(plant.x, plant.y, plant.yaw, plant.v, time);

        plant.update_state(u_1s[0], u_2s[0], dt);

        if goal_distance(&plant) < 0.5 && plant.v.abs() < 0.5 {
            break;
        }

        if controller.history_f.last().copied().unwrap_or(0.0) > 1.0e5 {
            break;
        }

        if !plant.x.is_finite()
            || !plant.y.is_finite()
            || !plant.yaw.is_finite()
            || !plant.v.is_finite()
            || plant.x.abs().max(plant.y.abs()) > 12.0
        {
            break;
        }
    }

    (plant, controller)
}

#[cfg(test)]
mod tests {
    use super::*;

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
