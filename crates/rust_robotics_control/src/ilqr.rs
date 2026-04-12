//! Iterative Linear Quadratic Regulator (iLQR) for a unicycle model.
//!
//! The planner optimizes a finite-horizon state and control trajectory to
//! drive a robot from a start pose to a goal pose.
//!
//! Reference:
//! - Weiwei Li, Emanuel Todorov, "Iterative Linear Quadratic Regulator
//!   Design for Nonlinear Biological Movement Systems":
//!   <https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf>

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Matrix3x2, Vector2, Vector3};
use std::f64::consts::PI;

const MAX_LINEAR_SPEED: f64 = 5.0;
const MAX_ANGULAR_SPEED: f64 = PI;
const TERMINAL_WEIGHT_SCALE: f64 = 10.0;
const REGULARIZATION: f64 = 1.0e-6;

type FeedforwardTerms = Vec<Vector2<f64>>;
type FeedbackGains = Vec<Matrix2x3<f64>>;

/// Configuration for the iLQR planner.
#[derive(Debug, Clone, Copy)]
pub struct ILQRConfig {
    /// Number of time steps in the optimization horizon.
    pub horizon: usize,
    /// Time step \[s\].
    pub dt: f64,
    /// Maximum iLQR iterations.
    pub max_iterations: usize,
    /// Convergence threshold on cost change.
    pub tolerance: f64,
    /// Position cost weight.
    pub q_pos: f64,
    /// Heading cost weight.
    pub q_yaw: f64,
    /// Linear velocity input cost weight.
    pub r_v: f64,
    /// Angular velocity input cost weight.
    pub r_omega: f64,
}

impl Default for ILQRConfig {
    fn default() -> Self {
        Self {
            horizon: 50,
            dt: 0.1,
            max_iterations: 50,
            tolerance: 1.0e-6,
            q_pos: 1.0,
            q_yaw: 0.1,
            r_v: 0.01,
            r_omega: 0.01,
        }
    }
}

/// Optimized iLQR trajectory.
#[derive(Debug, Clone)]
pub struct ILQRResult {
    pub states: Vec<Vector3<f64>>,
    pub controls: Vec<Vector2<f64>>,
    pub cost: f64,
    pub iterations: usize,
    pub converged: bool,
}

/// iLQR planner for a unicycle model.
pub struct ILQRPlanner {
    config: ILQRConfig,
}

impl ILQRPlanner {
    /// Creates a new iLQR planner.
    pub fn new(config: ILQRConfig) -> Self {
        Self { config }
    }

    /// Plans a trajectory from `start` to `goal`.
    pub fn plan(&self, start: Vector3<f64>, goal: Vector3<f64>) -> ILQRResult {
        let mut controls = self.initial_controls(start, goal);
        let mut states = self.rollout(start, &controls);
        let mut cost = self.total_cost(&states, &controls, goal);
        let mut iterations = 0;
        let mut converged = false;

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

            let cost_change = (cost - candidate_cost).abs();
            states = candidate_states;
            controls = candidate_controls;
            cost = candidate_cost;

            if cost_change < self.config.tolerance {
                converged = true;
                break;
            }
        }

        if !converged {
            let final_state = states.last().copied().unwrap_or(start);
            let pos_error =
                ((goal[0] - final_state[0]).powi(2) + (goal[1] - final_state[1]).powi(2)).sqrt();
            let yaw_error = normalize_angle(goal[2] - final_state[2]).abs();
            converged = pos_error < 0.25 && yaw_error < 0.1;
        }

        ILQRResult {
            states,
            controls,
            cost,
            iterations,
            converged,
        }
    }

    fn initial_controls(&self, start: Vector3<f64>, goal: Vector3<f64>) -> Vec<Vector2<f64>> {
        let distance = ((goal[0] - start[0]).powi(2) + (goal[1] - start[1]).powi(2)).sqrt();
        let total_time = (self.config.horizon as f64 * self.config.dt).max(self.config.dt);
        let v = (distance / total_time).clamp(-MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        let omega = (normalize_angle(goal[2] - start[2]) / total_time)
            .clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        vec![Vector2::new(v, omega); self.config.horizon]
    }

    fn rollout(&self, start: Vector3<f64>, controls: &[Vector2<f64>]) -> Vec<Vector3<f64>> {
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
        states: &[Vector3<f64>],
        controls: &[Vector2<f64>],
        goal: Vector3<f64>,
    ) -> f64 {
        let stage_cost: f64 = states
            .iter()
            .take(controls.len())
            .zip(controls.iter())
            .map(|(state, control)| self.stage_cost(state, control, goal))
            .sum();
        stage_cost + self.terminal_cost(states.last().expect("trajectory is non-empty"), goal)
    }

    fn stage_cost(&self, state: &Vector3<f64>, control: &Vector2<f64>, goal: Vector3<f64>) -> f64 {
        let error = state_error(state, goal);
        self.config.q_pos * (error[0].powi(2) + error[1].powi(2))
            + self.config.q_yaw * error[2].powi(2)
            + self.config.r_v * control[0].powi(2)
            + self.config.r_omega * control[1].powi(2)
    }

    fn terminal_cost(&self, state: &Vector3<f64>, goal: Vector3<f64>) -> f64 {
        let error = state_error(state, goal);
        TERMINAL_WEIGHT_SCALE
            * (self.config.q_pos * (error[0].powi(2) + error[1].powi(2))
                + self.config.q_yaw * error[2].powi(2))
    }

    fn backward_pass(
        &self,
        states: &[Vector3<f64>],
        controls: &[Vector2<f64>],
        goal: Vector3<f64>,
    ) -> Option<(FeedforwardTerms, FeedbackGains)> {
        let horizon = controls.len();
        let mut feedforward = vec![Vector2::zeros(); horizon];
        let mut feedback = vec![Matrix2x3::<f64>::zeros(); horizon];

        let (mut value_x, mut value_xx) =
            self.terminal_cost_derivatives(states.last().expect("trajectory is non-empty"), goal);

        for t in (0..horizon).rev() {
            let (a, b) = linearize_dynamics(&states[t], &controls[t], self.config.dt);
            let (l_x, l_u, l_xx, l_uu) =
                self.stage_cost_derivatives(&states[t], &controls[t], goal);

            let q_x = l_x + a.transpose() * value_x;
            let q_u = l_u + b.transpose() * value_x;
            let q_xx = l_xx + a.transpose() * value_xx * a;
            let mut q_uu = l_uu + b.transpose() * value_xx * b;
            let q_ux = b.transpose() * value_xx * a;

            q_uu = symmetrize_2x2(q_uu) + REGULARIZATION * Matrix2::identity();
            let q_uu_inv = q_uu.try_inverse()?;

            let k = -q_uu_inv * q_u;
            let k_feedback = -q_uu_inv * q_ux;
            feedforward[t] = k;
            feedback[t] = k_feedback;

            value_x = q_x
                + k_feedback.transpose() * q_uu * k
                + k_feedback.transpose() * q_u
                + q_ux.transpose() * k;
            value_xx = q_xx
                + k_feedback.transpose() * q_uu * k_feedback
                + k_feedback.transpose() * q_ux
                + q_ux.transpose() * k_feedback;
            value_xx = symmetrize_3x3(value_xx);
        }

        Some((feedforward, feedback))
    }

    fn forward_pass(
        &self,
        start: Vector3<f64>,
        nominal_states: &[Vector3<f64>],
        nominal_controls: &[Vector2<f64>],
        feedforward: &[Vector2<f64>],
        feedback: &[Matrix2x3<f64>],
        alpha: f64,
    ) -> (Vec<Vector3<f64>>, Vec<Vector2<f64>>) {
        let mut states = Vec::with_capacity(nominal_controls.len() + 1);
        let mut controls = Vec::with_capacity(nominal_controls.len());
        states.push(start);

        for t in 0..nominal_controls.len() {
            let delta_state = state_error(&states[t], nominal_states[t]);
            let delta_control = alpha * feedforward[t] + feedback[t] * delta_state;
            let mut control = nominal_controls[t] + delta_control;
            control[0] = control[0].clamp(-MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
            control[1] = control[1].clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
            controls.push(control);

            let next_state = dynamics(&states[t], &control, self.config.dt);
            states.push(next_state);
        }

        (states, controls)
    }

    fn stage_cost_derivatives(
        &self,
        state: &Vector3<f64>,
        control: &Vector2<f64>,
        goal: Vector3<f64>,
    ) -> (Vector3<f64>, Vector2<f64>, Matrix3<f64>, Matrix2<f64>) {
        let error = state_error(state, goal);
        let l_x = Vector3::new(
            2.0 * self.config.q_pos * error[0],
            2.0 * self.config.q_pos * error[1],
            2.0 * self.config.q_yaw * error[2],
        );
        let l_u = Vector2::new(
            2.0 * self.config.r_v * control[0],
            2.0 * self.config.r_omega * control[1],
        );
        let l_xx = Matrix3::from_diagonal(&Vector3::new(
            2.0 * self.config.q_pos,
            2.0 * self.config.q_pos,
            2.0 * self.config.q_yaw,
        ));
        let l_uu = Matrix2::from_diagonal(&Vector2::new(
            2.0 * self.config.r_v,
            2.0 * self.config.r_omega,
        ));

        (l_x, l_u, l_xx, l_uu)
    }

    fn terminal_cost_derivatives(
        &self,
        state: &Vector3<f64>,
        goal: Vector3<f64>,
    ) -> (Vector3<f64>, Matrix3<f64>) {
        let error = state_error(state, goal);
        let v_x = Vector3::new(
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos * error[0],
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos * error[1],
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_yaw * error[2],
        );
        let v_xx = Matrix3::from_diagonal(&Vector3::new(
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos,
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_pos,
            2.0 * TERMINAL_WEIGHT_SCALE * self.config.q_yaw,
        ));

        (v_x, v_xx)
    }
}

fn state_error(state: &Vector3<f64>, goal: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        state[0] - goal[0],
        state[1] - goal[1],
        normalize_angle(state[2] - goal[2]),
    )
}

fn symmetrize_2x2(matrix: Matrix2<f64>) -> Matrix2<f64> {
    0.5 * (matrix + matrix.transpose())
}

fn symmetrize_3x3(matrix: Matrix3<f64>) -> Matrix3<f64> {
    0.5 * (matrix + matrix.transpose())
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

fn dynamics(state: &Vector3<f64>, control: &Vector2<f64>, dt: f64) -> Vector3<f64> {
    let x = state[0] + control[0] * state[2].cos() * dt;
    let y = state[1] + control[0] * state[2].sin() * dt;
    let yaw = normalize_angle(state[2] + control[1] * dt);
    Vector3::new(x, y, yaw)
}

fn linearize_dynamics(
    state: &Vector3<f64>,
    control: &Vector2<f64>,
    dt: f64,
) -> (Matrix3<f64>, Matrix3x2<f64>) {
    let yaw = state[2];
    let v = control[0];

    let a = Matrix3::new(
        1.0,
        0.0,
        -dt * v * yaw.sin(),
        0.0,
        1.0,
        dt * v * yaw.cos(),
        0.0,
        0.0,
        1.0,
    );
    let b = Matrix3x2::new(dt * yaw.cos(), 0.0, dt * yaw.sin(), 0.0, 0.0, dt);

    (a, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {expected:.12}, got {actual:.12}, tolerance {tolerance:.3e}"
        );
    }

    #[test]
    fn test_ilqr_config_defaults() {
        let config = ILQRConfig::default();
        assert_eq!(config.horizon, 50);
        assert_eq!(config.dt, 0.1);
        assert_eq!(config.max_iterations, 50);
        assert_eq!(config.tolerance, 1.0e-6);
        assert_eq!(config.q_pos, 1.0);
        assert_eq!(config.q_yaw, 0.1);
        assert_eq!(config.r_v, 0.01);
        assert_eq!(config.r_omega, 0.01);
    }

    #[test]
    fn test_ilqr_straight_line_converges() {
        let planner = ILQRPlanner::new(ILQRConfig {
            horizon: 60,
            ..ILQRConfig::default()
        });
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(5.0, 0.0, 0.0);

        let result = planner.plan(start, goal);
        let final_state = result.states.last().expect("trajectory").clone_owned();

        assert!(result.converged);
        assert!((final_state[0] - goal[0]).abs() < 0.25);
        assert!(final_state[1].abs() < 0.1);
        assert!(normalize_angle(final_state[2] - goal[2]).abs() < 0.1);
    }

    #[test]
    fn test_ilqr_turn_converges() {
        let planner = ILQRPlanner::new(ILQRConfig {
            horizon: 70,
            ..ILQRConfig::default()
        });
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(3.0, 2.0, PI / 2.0);

        let result = planner.plan(start, goal);
        let final_state = result.states.last().expect("trajectory").clone_owned();
        let pos_error =
            ((final_state[0] - goal[0]).powi(2) + (final_state[1] - goal[1]).powi(2)).sqrt();

        assert!(result.converged);
        assert!(pos_error < 0.3);
        assert!(normalize_angle(final_state[2] - goal[2]).abs() < 0.15);
    }

    #[test]
    fn test_ilqr_cost_decreases_over_iterations() {
        let planner = ILQRPlanner::new(ILQRConfig::default());
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(4.0, 1.0, 0.2);

        let initial_controls = planner.initial_controls(start, goal);
        let initial_states = planner.rollout(start, &initial_controls);
        let initial_cost = planner.total_cost(&initial_states, &initial_controls, goal);

        let result = planner.plan(start, goal);

        assert!(result.cost < initial_cost);
    }

    #[test]
    fn test_ilqr_turning_case_regression() {
        let planner = ILQRPlanner::new(ILQRConfig {
            horizon: 70,
            ..ILQRConfig::default()
        });
        let start = Vector3::new(0.0, 0.0, 0.0);
        let goal = Vector3::new(3.0, 2.0, PI / 2.0);

        let result = planner.plan(start, goal);
        let final_state = result.states.last().expect("trajectory").clone_owned();

        assert!(result.converged);
        assert_eq!(result.iterations, 10);
        assert_close(result.cost, 43.962457904378, 1.0e-6);
        assert_close(final_state[0], 2.991948883770, 1.0e-6);
        assert_close(final_state[1], 2.000000028451, 1.0e-6);
        assert_close(final_state[2], 1.570796059231, 1.0e-6);
        assert_close(result.controls[0][0], 5.0, 1.0e-12);
        assert_close(result.controls[0][1], PI, 1.0e-12);
        assert_close(result.controls[1][0], 5.0, 1.0e-12);
        assert_close(result.controls[1][1], PI, 1.0e-12);
    }
}
