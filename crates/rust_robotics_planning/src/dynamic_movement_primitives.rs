//! Dynamic Movement Primitives (DMP)
//!
//! Learns a trajectory from demonstration data by modelling the forcing
//! function as a weighted sum of Gaussian basis functions.  The learned
//! weights can then be used to recreate the trajectory with different
//! start/goal positions or durations.
//!
//! References:
//! - <https://arxiv.org/abs/2102.03861>
//! - <https://www.frontiersin.org/articles/10.3389/fncom.2013.00138/full>

/// Configuration for the DMP learner.
#[derive(Debug, Clone)]
pub struct DmpConfig {
    /// Virtual spring constant (K).
    pub spring: f64,
    /// Virtual damper coefficient (B).
    pub damper: f64,
    /// Number of Gaussian basis functions.
    pub num_basis: usize,
}

impl Default for DmpConfig {
    fn default() -> Self {
        Self {
            spring: 156.25,
            damper: 25.0,
            num_basis: 10,
        }
    }
}

/// A learned DMP model that can reproduce trajectories.
#[derive(Debug, Clone)]
pub struct Dmp {
    /// Weight matrix: `weights\[dim\]\[basis_idx\]`.
    weights: Vec<Vec<f64>>,
    /// Number of spatial dimensions.
    dimensions: usize,
    /// Number of time-steps in the original demonstration.
    timesteps: usize,
    /// Time-step size derived from the training data.
    dt: f64,
    /// Spring constant.
    spring: f64,
    /// Damper coefficient.
    damper: f64,
    /// Number of basis functions.
    num_basis: usize,
}

/// Result of trajectory recreation.
#[derive(Debug, Clone)]
pub struct DmpTrajectory {
    /// Time values for each step.
    pub time: Vec<f64>,
    /// Positions at each step: `positions\[step\]\[dim\]`.
    pub positions: Vec<Vec<f64>>,
}

impl Dmp {
    /// Learn DMP weights from demonstration data.
    ///
    /// `training_data` is a slice of points where each inner slice is a
    /// spatial position (e.g. `&[x, y]`).  `data_period` is the total time
    /// the demonstration covers.
    pub fn learn(training_data: &[Vec<f64>], data_period: f64, config: &DmpConfig) -> Self {
        let timesteps = training_data.len();
        assert!(timesteps >= 2, "need at least 2 data points");
        let dimensions = training_data[0].len();
        assert!(dimensions > 0, "need at least 1 dimension");

        let dt = data_period / timesteps as f64;
        let num_basis = config.num_basis;

        // Centres and (shared) variance of Gaussian basis functions.
        let centres: Vec<f64> = (0..num_basis)
            .map(|i| i as f64 / (num_basis - 1).max(1) as f64)
            .collect();
        let h = 0.65 / ((1.0 / (num_basis as f64 - 1.0)).powi(2).max(1e-12));

        let init_state = &training_data[0];
        let goal_state = &training_data[timesteps - 1];

        let mut all_weights: Vec<Vec<f64>> = Vec::with_capacity(dimensions);

        for dim in 0..dimensions {
            let q0 = init_state[dim];
            let g = goal_state[dim];
            let g_minus_q0 = g - q0;

            let mut q = q0;
            let mut qd_last = 0.0;

            let mut phi_matrix: Vec<Vec<f64>> = Vec::with_capacity(timesteps);
            let mut f_vals: Vec<f64> = Vec::with_capacity(timesteps);

            for i in 0..timesteps {
                let qd = if i + 1 < timesteps {
                    (training_data[i + 1][dim] - training_data[i][dim]) / dt
                } else {
                    0.0
                };

                // Normalised basis function values.
                let phase = i as f64 * dt / data_period;
                let mut phi: Vec<f64> = centres
                    .iter()
                    .map(|&c| (-0.5 * (phase - c).powi(2) * h).exp())
                    .collect();
                let phi_sum: f64 = phi.iter().sum::<f64>().max(1e-12);
                for v in &mut phi {
                    *v /= phi_sum;
                }

                let qdd = (qd - qd_last) / dt;

                let f = if g_minus_q0.abs() < 1e-12 {
                    0.0
                } else {
                    (qdd * data_period.powi(2) - config.spring * (g - q)
                        + config.damper * qd * data_period)
                        / g_minus_q0
                };

                phi_matrix.push(phi);
                f_vals.push(f);

                qd_last = qd;
                q += qd * dt;
            }

            // Solve least-squares:  phi_matrix * w = f_vals.
            let w = lstsq(&phi_matrix, &f_vals, num_basis);
            all_weights.push(w);
        }

        Self {
            weights: all_weights,
            dimensions,
            timesteps,
            dt,
            spring: config.spring,
            damper: config.damper,
            num_basis,
        }
    }

    /// Recreate a trajectory from the learned weights.
    ///
    /// * `init_state` — desired start position (one value per dimension).
    /// * `goal_state` — desired goal position.
    /// * `period`     — total time for the new trajectory.
    pub fn recreate(&self, init_state: &[f64], goal_state: &[f64], period: f64) -> DmpTrajectory {
        assert_eq!(init_state.len(), self.dimensions);
        assert_eq!(goal_state.len(), self.dimensions);

        let centres: Vec<f64> = (0..self.num_basis)
            .map(|i| i as f64 / (self.num_basis - 1).max(1) as f64)
            .collect();
        let h = 0.65 / ((1.0 / (self.num_basis as f64 - 1.0)).powi(2).max(1e-12));

        let mut q: Vec<f64> = init_state.to_vec();
        let mut qd = vec![0.0; self.dimensions];

        let mut time_vec = Vec::with_capacity(self.timesteps);
        let mut positions = Vec::with_capacity(self.timesteps);
        let mut time = 0.0;

        for _ in 0..self.timesteps {
            time += self.dt;

            let mut qdd = vec![0.0; self.dimensions];

            for dim in 0..self.dimensions {
                let f = if time <= period {
                    let phase = time / period;
                    let mut phi: Vec<f64> = centres
                        .iter()
                        .map(|&c| (-0.5 * (phase - c).powi(2) * h).exp())
                        .collect();
                    let phi_sum: f64 = phi.iter().sum::<f64>().max(1e-12);
                    for v in &mut phi {
                        *v /= phi_sum;
                    }
                    phi.iter()
                        .zip(self.weights[dim].iter())
                        .map(|(p, w)| p * w)
                        .sum::<f64>()
                } else {
                    0.0
                };

                qdd[dim] = self.spring * (goal_state[dim] - q[dim]) / period.powi(2)
                    - self.damper * qd[dim] / period
                    + (goal_state[dim] - init_state[dim]) * f / period.powi(2);
            }

            for dim in 0..self.dimensions {
                qd[dim] += qdd[dim] * self.dt;
                q[dim] += qd[dim] * self.dt;
            }

            time_vec.push(time);
            positions.push(q.clone());
        }

        DmpTrajectory {
            time: time_vec,
            positions,
        }
    }

    /// Number of spatial dimensions.
    pub fn dimensions(&self) -> usize {
        self.dimensions
    }

    /// Number of basis functions.
    pub fn num_basis(&self) -> usize {
        self.num_basis
    }

    /// Reference to the learned weight matrix.
    pub fn weights(&self) -> &[Vec<f64>] {
        &self.weights
    }
}

// ------------------------------------------------------------------
// Minimal least-squares solver (normal equations).
// ------------------------------------------------------------------

/// Solve `A * x = b` in the least-squares sense via normal equations.
/// `A` is `(m x n)`, `b` is `(m,)`, returns `x` of length `n`.
fn lstsq(a: &[Vec<f64>], b: &[f64], n: usize) -> Vec<f64> {
    let m = a.len();
    assert_eq!(b.len(), m);

    // A^T A  (n x n)
    let mut ata = vec![vec![0.0; n]; n];
    // A^T b  (n,)
    let mut atb = vec![0.0; n];

    for row in 0..m {
        for j in 0..n {
            atb[j] += a[row][j] * b[row];
            for k in j..n {
                let v = a[row][j] * a[row][k];
                ata[j][k] += v;
                if k != j {
                    ata[k][j] += v;
                }
            }
        }
    }

    // Add small regularisation for numerical stability.
    for i in 0..n {
        ata[i][i] += 1e-10;
    }

    // Solve via Cholesky-like Gaussian elimination (symmetric positive-definite).
    solve_symmetric(&mut ata, &mut atb)
}

/// Solve a symmetric positive-definite system in-place via Gaussian elimination.
fn solve_symmetric(a: &mut [Vec<f64>], b: &mut [f64]) -> Vec<f64> {
    let n = b.len();
    // Forward elimination.
    for col in 0..n {
        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution.
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for j in (i + 1)..n {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a simple sine-wave demonstration in 2-D.
    fn sine_demo() -> (Vec<Vec<f64>>, f64) {
        let n = 200;
        let period = 2.0 * std::f64::consts::PI;
        let dt = period / n as f64;
        let data: Vec<Vec<f64>> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                vec![t, t.sin()]
            })
            .collect();
        (data, period)
    }

    #[test]
    fn test_dmp_learn_dimensions() {
        let (data, period) = sine_demo();
        let config = DmpConfig::default();
        let dmp = Dmp::learn(&data, period, &config);
        assert_eq!(dmp.dimensions(), 2);
        assert_eq!(dmp.num_basis(), 10);
        assert_eq!(dmp.weights().len(), 2);
    }

    #[test]
    fn test_recreate_same_endpoints() {
        let (data, period) = sine_demo();
        let config = DmpConfig::default();
        let dmp = Dmp::learn(&data, period, &config);

        let init = &data[0];
        let goal = &data[data.len() - 1];
        let traj = dmp.recreate(init, goal, period);

        assert_eq!(traj.positions.len(), data.len());

        // First position should be close to init (after one dt step).
        let first = &traj.positions[0];
        assert!(
            (first[0] - init[0]).abs() < 1.0,
            "first x too far from init"
        );

        // Last position should converge close to goal.
        let last = &traj.positions[traj.positions.len() - 1];
        assert!(
            (last[0] - goal[0]).abs() < 2.0,
            "last x too far from goal: {} vs {}",
            last[0],
            goal[0]
        );
        assert!(
            (last[1] - goal[1]).abs() < 2.0,
            "last y too far from goal: {} vs {}",
            last[1],
            goal[1]
        );
    }

    #[test]
    fn test_recreate_shifted_goal() {
        let (data, period) = sine_demo();
        let config = DmpConfig::default();
        let dmp = Dmp::learn(&data, period, &config);

        let init = data[0].clone();
        let mut goal = data[data.len() - 1].clone();
        goal[1] += 2.0; // shift goal upward

        let traj = dmp.recreate(&init, &goal, period);
        let last = &traj.positions[traj.positions.len() - 1];
        // Should converge towards the new goal, not the original.
        assert!(
            (last[1] - goal[1]).abs() < 2.0,
            "shifted goal: last y = {}, goal y = {}",
            last[1],
            goal[1]
        );
    }

    #[test]
    fn test_recreate_different_period() {
        let (data, period) = sine_demo();
        let config = DmpConfig::default();
        let dmp = Dmp::learn(&data, period, &config);

        let init = data[0].clone();
        let goal = data[data.len() - 1].clone();

        let traj_fast = dmp.recreate(&init, &goal, period * 0.5);
        let traj_slow = dmp.recreate(&init, &goal, period * 2.0);

        // Both should have same number of steps (same timesteps / dt).
        assert_eq!(traj_fast.positions.len(), traj_slow.positions.len());

        // With a shorter period the forcing function shuts off earlier,
        // so the trajectory shapes should differ.
        let mid = traj_fast.positions.len() / 2;
        let diff = (traj_fast.positions[mid][1] - traj_slow.positions[mid][1]).abs();
        assert!(
            diff > 1e-6,
            "trajectories with different periods should differ at midpoint"
        );
    }

    #[test]
    fn test_1d_trajectory() {
        // Simple 1-D ramp.
        let data: Vec<Vec<f64>> = (0..50).map(|i| vec![i as f64 * 0.1]).collect();
        let period = 5.0;
        let config = DmpConfig::default();
        let dmp = Dmp::learn(&data, period, &config);
        assert_eq!(dmp.dimensions(), 1);

        let traj = dmp.recreate(&[0.0], &[4.9], period);
        let last = traj.positions.last().unwrap();
        assert!(
            (last[0] - 4.9).abs() < 2.0,
            "1-D ramp end: {} vs 4.9",
            last[0]
        );
    }

    #[test]
    fn test_lstsq_identity() {
        // Trivial system: I * x = b  =>  x = b.
        let a = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let b = vec![3.0, 5.0, 7.0];
        let x = lstsq(&a, &b, 3);
        for i in 0..3 {
            assert!((x[i] - b[i]).abs() < 1e-6, "lstsq identity failed at {i}");
        }
    }
}
