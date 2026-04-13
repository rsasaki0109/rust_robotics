//! Differential Dynamic Programming (DDP) for a car-like model.
//!
//! State: `[x, y, theta, v]`
//! Control: `[accel, steer]`

use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, Vector2, Vector4};
use std::f64::consts::PI;

const WHEEL_BASE: f64 = 2.5;
const TERMINAL_WEIGHT_SCALE: f64 = 20.0;
const REGULARIZATION: f64 = 1.0e-6;
const MAX_SPEED: f64 = 8.0;
const MIN_SPEED: f64 = -5.0;
const MAX_ACCEL: f64 = 3.0;
const MAX_STEER: f64 = 0.6;

type FeedforwardTerms = Vec<Vector2<f64>>;
type FeedbackGains = Vec<Matrix2x4<f64>>;
type StageCostDerivatives = (
    Vector4<f64>,
    Vector2<f64>,
    Matrix4<f64>,
    Matrix2<f64>,
    Matrix2x4<f64>,
);
type SecondOrderDyn = ([Matrix4<f64>; 4], [Matrix2x4<f64>; 4], [Matrix2<f64>; 4]);

/// Configuration for DDP.
#[derive(Debug, Clone, Copy)]
pub struct DDPConfig {
    /// Number of time steps in optimization horizon.
    pub horizon: usize,
    /// Integration time step \[s\].
    pub dt: f64,
    /// Maximum optimization iterations.
    pub max_iterations: usize,
    /// Convergence tolerance on cost change.
    pub tolerance: f64,
    /// Position cost weight.
    pub q_pos: f64,
    /// Heading cost weight.
    pub q_theta: f64,
    /// Velocity cost weight.
    pub q_v: f64,
    /// Acceleration effort cost.
    pub r_accel: f64,
    /// Steering effort cost.
    pub r_steer: f64,
}

impl Default for DDPConfig {
    fn default() -> Self {
        Self {
            horizon: 50,
            dt: 0.1,
            max_iterations: 50,
            tolerance: 1.0e-6,
            q_pos: 1.0,
            q_theta: 0.2,
            q_v: 0.2,
            r_accel: 0.05,
            r_steer: 0.05,
        }
    }
}

/// DDP optimization output.
#[derive(Debug, Clone)]
pub struct DDPResult {
    pub states: Vec<Vector4<f64>>,
    pub controls: Vec<Vector2<f64>>,
    pub cost: f64,
    pub iterations: usize,
    pub converged: bool,
}

/// DDP planner.
pub struct DDPPlanner {
    config: DDPConfig,
}

impl DDPPlanner {
    pub fn new(config: DDPConfig) -> Self {
        Self { config }
    }

    pub fn plan(&self, start: Vector4<f64>, goal: Vector4<f64>) -> DDPResult {
        let mut controls = self.initial_controls(start, goal);
        let mut states = self.rollout(start, &controls);
        let mut cost = self.total_cost(&states, &controls, goal);
        let mut converged = false;
        let mut iterations = 0usize;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;
            let Some((feedforward, feedback)) = self.backward_pass(&states, &controls, goal) else {
                break;
            };

            let mut accepted = None;
            for alpha in [1.0, 0.5, 0.25, 0.1, 0.05, 0.01] {
                let (candidate_states, candidate_controls) =
                    self.forward_pass(start, &states, &controls, &feedforward, &feedback, alpha);
                let candidate_cost = self.total_cost(&candidate_states, &candidate_controls, goal);
                if candidate_cost < cost {
                    accepted = Some((candidate_states, candidate_controls, candidate_cost));
                    break;
                }
            }

            let Some((candidate_states, candidate_controls, candidate_cost)) = accepted else {
                break;
            };

            let delta = (cost - candidate_cost).abs();
            states = candidate_states;
            controls = candidate_controls;
            cost = candidate_cost;
            if delta < self.config.tolerance {
                converged = true;
                break;
            }
        }

        if !converged {
            let final_state = states.last().copied().unwrap_or(start);
            let pos_error =
                ((final_state[0] - goal[0]).powi(2) + (final_state[1] - goal[1]).powi(2)).sqrt();
            let heading_error = normalize_angle(final_state[2] - goal[2]).abs();
            converged = pos_error < 0.35 && heading_error < 0.2;
        }

        DDPResult {
            states,
            controls,
            cost,
            iterations,
            converged,
        }
    }

    fn initial_controls(&self, start: Vector4<f64>, goal: Vector4<f64>) -> Vec<Vector2<f64>> {
        let distance = ((goal[0] - start[0]).powi(2) + (goal[1] - start[1]).powi(2)).sqrt();
        let total_time = (self.config.horizon as f64 * self.config.dt).max(self.config.dt);
        let desired_v = (distance / total_time).clamp(MIN_SPEED, MAX_SPEED);
        let accel = ((desired_v - start[3]) / total_time).clamp(-MAX_ACCEL, MAX_ACCEL);
        let steer = (normalize_angle(goal[2] - start[2]) / total_time).clamp(-MAX_STEER, MAX_STEER);
        vec![Vector2::new(accel, steer); self.config.horizon]
    }

    fn rollout(&self, start: Vector4<f64>, controls: &[Vector2<f64>]) -> Vec<Vector4<f64>> {
        let mut states = Vec::with_capacity(controls.len() + 1);
        let mut state = start;
        states.push(state);
        for control in controls {
            state = dynamics(&state, control, self.config.dt);
            states.push(state);
        }
        states
    }

    fn total_cost(
        &self,
        states: &[Vector4<f64>],
        controls: &[Vector2<f64>],
        goal: Vector4<f64>,
    ) -> f64 {
        let stage_cost: f64 = states
            .iter()
            .take(controls.len())
            .zip(controls.iter())
            .map(|(state, control)| self.stage_cost(state, control, goal))
            .sum();
        stage_cost + self.terminal_cost(states.last().expect("trajectory"), goal)
    }

    fn stage_cost(&self, state: &Vector4<f64>, control: &Vector2<f64>, goal: Vector4<f64>) -> f64 {
        let error = state_error(state, goal);
        self.config.q_pos * (error[0].powi(2) + error[1].powi(2))
            + self.config.q_theta * error[2].powi(2)
            + self.config.q_v * error[3].powi(2)
            + self.config.r_accel * control[0].powi(2)
            + self.config.r_steer * control[1].powi(2)
    }

    fn terminal_cost(&self, state: &Vector4<f64>, goal: Vector4<f64>) -> f64 {
        let error = state_error(state, goal);
        TERMINAL_WEIGHT_SCALE
            * (self.config.q_pos * (error[0].powi(2) + error[1].powi(2))
                + self.config.q_theta * error[2].powi(2)
                + self.config.q_v * error[3].powi(2))
    }

    fn stage_cost_derivatives(
        &self,
        state: &Vector4<f64>,
        control: &Vector2<f64>,
        goal: Vector4<f64>,
    ) -> StageCostDerivatives {
        let error = state_error(state, goal);
        let l_x = Vector4::new(
            2.0 * self.config.q_pos * error[0],
            2.0 * self.config.q_pos * error[1],
            2.0 * self.config.q_theta * error[2],
            2.0 * self.config.q_v * error[3],
        );
        let l_u = Vector2::new(
            2.0 * self.config.r_accel * control[0],
            2.0 * self.config.r_steer * control[1],
        );
        let l_xx = Matrix4::from_diagonal(&Vector4::new(
            2.0 * self.config.q_pos,
            2.0 * self.config.q_pos,
            2.0 * self.config.q_theta,
            2.0 * self.config.q_v,
        ));
        let l_uu = Matrix2::from_diagonal(&Vector2::new(
            2.0 * self.config.r_accel,
            2.0 * self.config.r_steer,
        ));
        let l_ux = Matrix2x4::zeros();
        (l_x, l_u, l_xx, l_uu, l_ux)
    }

    fn terminal_cost_derivatives(
        &self,
        state: &Vector4<f64>,
        goal: Vector4<f64>,
    ) -> (Vector4<f64>, Matrix4<f64>) {
        let error = state_error(state, goal);
        let v_x = Vector4::new(
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos * error[0],
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos * error[1],
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_theta * error[2],
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_v * error[3],
        );
        let v_xx = Matrix4::from_diagonal(&Vector4::new(
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos,
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos,
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_theta,
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_v,
        ));
        (v_x, v_xx)
    }

    fn backward_pass(
        &self,
        states: &[Vector4<f64>],
        controls: &[Vector2<f64>],
        goal: Vector4<f64>,
    ) -> Option<(FeedforwardTerms, FeedbackGains)> {
        let horizon = controls.len();
        let mut feedforward = vec![Vector2::zeros(); horizon];
        let mut feedback = vec![Matrix2x4::zeros(); horizon];

        let (mut value_x, mut value_xx) =
            self.terminal_cost_derivatives(states.last().expect("trajectory"), goal);

        for t in (0..horizon).rev() {
            let (a, b) = linearize_dynamics(&states[t], &controls[t], self.config.dt);
            let (f_xx, f_ux, f_uu) =
                second_order_dynamics(&states[t], &controls[t], self.config.dt);
            let (l_x, l_u, l_xx, l_uu, l_ux) =
                self.stage_cost_derivatives(&states[t], &controls[t], goal);

            let mut q_x = l_x + a.transpose() * value_x;
            let q_u = l_u + b.transpose() * value_x;
            let mut q_xx = l_xx + a.transpose() * value_xx * a;
            let mut q_ux = l_ux + b.transpose() * value_xx * a;
            let mut q_uu = l_uu + b.transpose() * value_xx * b;

            for i in 0..4 {
                q_xx += value_x[i] * f_xx[i];
                q_ux += value_x[i] * f_ux[i];
                q_uu += value_x[i] * f_uu[i];
            }

            q_xx = symmetrize_4x4(q_xx);
            q_uu = symmetrize_2x2(q_uu) + REGULARIZATION * Matrix2::identity();
            let q_uu_inv = q_uu.try_inverse()?;

            let k = -q_uu_inv * q_u;
            let k_feedback = -q_uu_inv * q_ux;
            feedforward[t] = k;
            feedback[t] = k_feedback;

            q_x += k_feedback.transpose() * q_uu * k
                + k_feedback.transpose() * q_u
                + q_ux.transpose() * k;
            value_x = q_x;
            value_xx = q_xx
                + k_feedback.transpose() * q_uu * k_feedback
                + k_feedback.transpose() * q_ux
                + q_ux.transpose() * k_feedback;
            value_xx = symmetrize_4x4(value_xx);
        }

        Some((feedforward, feedback))
    }

    fn forward_pass(
        &self,
        start: Vector4<f64>,
        nominal_states: &[Vector4<f64>],
        nominal_controls: &[Vector2<f64>],
        feedforward: &[Vector2<f64>],
        feedback: &[Matrix2x4<f64>],
        alpha: f64,
    ) -> (Vec<Vector4<f64>>, Vec<Vector2<f64>>) {
        let mut states = Vec::with_capacity(nominal_controls.len() + 1);
        let mut controls = Vec::with_capacity(nominal_controls.len());
        states.push(start);

        for t in 0..nominal_controls.len() {
            let delta_x = state_error(&states[t], nominal_states[t]);
            let delta_u = alpha * feedforward[t] + feedback[t] * delta_x;
            let mut control = nominal_controls[t] + delta_u;
            control[0] = control[0].clamp(-MAX_ACCEL, MAX_ACCEL);
            control[1] = control[1].clamp(-MAX_STEER, MAX_STEER);
            controls.push(control);
            states.push(dynamics(&states[t], &control, self.config.dt));
        }
        (states, controls)
    }
}

fn state_error(state: &Vector4<f64>, goal: Vector4<f64>) -> Vector4<f64> {
    Vector4::new(
        state[0] - goal[0],
        state[1] - goal[1],
        normalize_angle(state[2] - goal[2]),
        state[3] - goal[3],
    )
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

fn dynamics(state: &Vector4<f64>, control: &Vector2<f64>, dt: f64) -> Vector4<f64> {
    let accel = control[0].clamp(-MAX_ACCEL, MAX_ACCEL);
    let steer = control[1].clamp(-MAX_STEER, MAX_STEER);
    let v = (state[3] + accel * dt).clamp(MIN_SPEED, MAX_SPEED);
    let x = state[0] + state[3] * state[2].cos() * dt;
    let y = state[1] + state[3] * state[2].sin() * dt;
    let theta = normalize_angle(state[2] + (state[3] / WHEEL_BASE) * steer.tan() * dt);
    Vector4::new(x, y, theta, v)
}

fn linearize_dynamics(
    state: &Vector4<f64>,
    control: &Vector2<f64>,
    dt: f64,
) -> (Matrix4<f64>, Matrix4x2<f64>) {
    let theta = state[2];
    let v = state[3];
    let steer = control[1].clamp(-MAX_STEER, MAX_STEER);
    let sec2 = 1.0 / steer.cos().powi(2);

    let a = Matrix4::new(
        1.0,
        0.0,
        -v * theta.sin() * dt,
        theta.cos() * dt,
        0.0,
        1.0,
        v * theta.cos() * dt,
        theta.sin() * dt,
        0.0,
        0.0,
        1.0,
        (steer.tan() / WHEEL_BASE) * dt,
        0.0,
        0.0,
        0.0,
        1.0,
    );

    let b = Matrix4x2::new(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (v / WHEEL_BASE) * sec2 * dt,
        dt,
        0.0,
    );

    (a, b)
}

fn second_order_dynamics(state: &Vector4<f64>, control: &Vector2<f64>, dt: f64) -> SecondOrderDyn {
    let theta = state[2];
    let v = state[3];
    let steer = control[1].clamp(-MAX_STEER, MAX_STEER);
    let sec2 = 1.0 / steer.cos().powi(2);
    let tan = steer.tan();

    let mut f_xx = [
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
    ];
    let mut f_ux = [
        Matrix2x4::zeros(),
        Matrix2x4::zeros(),
        Matrix2x4::zeros(),
        Matrix2x4::zeros(),
    ];
    let mut f_uu = [
        Matrix2::zeros(),
        Matrix2::zeros(),
        Matrix2::zeros(),
        Matrix2::zeros(),
    ];

    // x_{k+1} second derivatives
    f_xx[0][(2, 2)] = -v * theta.cos() * dt;
    f_xx[0][(2, 3)] = -theta.sin() * dt;
    f_xx[0][(3, 2)] = -theta.sin() * dt;

    // y_{k+1} second derivatives
    f_xx[1][(2, 2)] = -v * theta.sin() * dt;
    f_xx[1][(2, 3)] = theta.cos() * dt;
    f_xx[1][(3, 2)] = theta.cos() * dt;

    // theta_{k+1} second derivatives
    f_ux[2][(1, 3)] = (sec2 / WHEEL_BASE) * dt;
    f_uu[2][(1, 1)] = (v / WHEEL_BASE) * 2.0 * sec2 * tan * dt;

    (f_xx, f_ux, f_uu)
}

fn symmetrize_2x2(matrix: Matrix2<f64>) -> Matrix2<f64> {
    0.5 * (matrix + matrix.transpose())
}

fn symmetrize_4x4(matrix: Matrix4<f64>) -> Matrix4<f64> {
    0.5 * (matrix + matrix.transpose())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ddp_config_defaults() {
        let config = DDPConfig::default();
        assert_eq!(config.horizon, 50);
        assert_eq!(config.dt, 0.1);
        assert_eq!(config.max_iterations, 50);
        assert_eq!(config.tolerance, 1.0e-6);
    }

    #[test]
    fn test_ddp_cost_decreases_from_initial_rollout() {
        let planner = DDPPlanner::new(DDPConfig::default());
        let start = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let goal = Vector4::new(5.0, 1.0, 0.2, 1.0);

        let initial_controls = planner.initial_controls(start, goal);
        let initial_states = planner.rollout(start, &initial_controls);
        let initial_cost = planner.total_cost(&initial_states, &initial_controls, goal);

        let result = planner.plan(start, goal);
        assert!(result.cost < initial_cost);
    }

    #[test]
    fn test_ddp_reaches_goal_with_turn() {
        let planner = DDPPlanner::new(DDPConfig {
            horizon: 100,
            ..DDPConfig::default()
        });
        let start = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let goal = Vector4::new(2.0, 0.5, 0.2, 0.8);
        let result = planner.plan(start, goal);
        let final_state = result.states.last().copied().expect("trajectory");
        let pos_error =
            ((final_state[0] - goal[0]).powi(2) + (final_state[1] - goal[1]).powi(2)).sqrt();

        assert!(pos_error < 0.6);
        assert!(normalize_angle(final_state[2] - goal[2]).abs() < 0.5);
    }

    #[test]
    fn test_ddp_converges_on_straight_line_case() {
        let planner = DDPPlanner::new(DDPConfig::default());
        let start = Vector4::new(0.0, 0.0, 0.0, 0.0);
        let goal = Vector4::new(5.0, 0.0, 0.0, 1.0);
        let result = planner.plan(start, goal);
        let final_state = result.states.last().copied().expect("trajectory");

        assert!(result.converged);
        assert!((final_state[0] - goal[0]).abs() < 0.4);
        assert!(final_state[1].abs() < 0.2);
    }
}
