//! Distributed formation consensus via ADMM.
//!
//! A pure-Rust reproduction slice for the coordination layer of distributed MPC
//! (in the spirit of ACLM-style consensus control): a team of agents agrees on a
//! common formation center `z` while each agent settles at `z + offset_i`, trades
//! off staying near its own preferred position, and respects a per-agent box
//! constraint. The agreement is reached by the classic *consensus ADMM*
//! iteration — each agent solves a tiny local problem, the team averages to a
//! consensus, and scaled dual variables drive the per-agent residuals to zero.
//!
//! Minimize `sum_i (w_i/2) ||x_i - a_i||^2` subject to `x_i - offset_i = z` and
//! `lower_i <= x_i <= upper_i`. With equal weights and no box constraints the
//! consensus center has the closed form `z* = mean(a_i - offset_i)`, which the
//! solver recovers; box constraints make it iterative and is where ADMM earns its
//! keep.
//!
//! - [`AgentSpec`] is one agent's preferred position, formation offset, weight,
//!   and box bounds.
//! - [`AdmmConfig`] holds the penalty `rho`, the iteration cap, and the residual
//!   tolerance.
//! - [`solve_formation_consensus`] runs consensus ADMM and returns the agent
//!   positions, the consensus center, and the primal/dual residual history.

use rust_robotics_core::{RoboticsError, RoboticsResult};

/// One agent in the formation-consensus problem.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AgentSpec {
    /// Preferred position `a_i` the agent is pulled toward.
    pub preferred: [f64; 2],
    /// Formation offset `offset_i` from the shared center.
    pub offset: [f64; 2],
    /// Cost weight on staying near `preferred` (relative to the consensus pull).
    pub weight: f64,
    /// Lower box bound on the agent position (use `f64::NEG_INFINITY` to disable).
    pub lower: [f64; 2],
    /// Upper box bound on the agent position (use `f64::INFINITY` to disable).
    pub upper: [f64; 2],
}

impl AgentSpec {
    /// An unconstrained agent with unit weight.
    pub fn new(preferred: [f64; 2], offset: [f64; 2]) -> Self {
        Self {
            preferred,
            offset,
            weight: 1.0,
            lower: [f64::NEG_INFINITY; 2],
            upper: [f64::INFINITY; 2],
        }
    }

    /// Set a box constraint on the agent's position.
    pub fn with_box(mut self, lower: [f64; 2], upper: [f64; 2]) -> Self {
        self.lower = lower;
        self.upper = upper;
        self
    }

    /// Set the cost weight on staying near `preferred`.
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }
}

/// Consensus-ADMM solver settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdmmConfig {
    /// Augmented-Lagrangian penalty parameter.
    pub rho: f64,
    /// Maximum number of ADMM iterations.
    pub max_iters: usize,
    /// Stop once both residual norms fall below this tolerance.
    pub tol: f64,
}

impl Default for AdmmConfig {
    fn default() -> Self {
        Self {
            rho: 1.0,
            max_iters: 200,
            tol: 1e-6,
        }
    }
}

/// Result of a consensus-ADMM solve.
#[derive(Debug, Clone, PartialEq)]
pub struct AdmmReport {
    /// Final agent positions `x_i`.
    pub positions: Vec<[f64; 2]>,
    /// Consensus center `z`.
    pub center: [f64; 2],
    /// Primal residual norm per iteration.
    pub primal_residuals: Vec<f64>,
    /// Dual residual norm per iteration.
    pub dual_residuals: Vec<f64>,
    /// Iterations actually run.
    pub iterations: usize,
}

fn clamp2(v: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> [f64; 2] {
    [v[0].clamp(lo[0], hi[0]), v[1].clamp(lo[1], hi[1])]
}

/// Solve the formation-consensus problem with consensus ADMM.
pub fn solve_formation_consensus(
    config: AdmmConfig,
    agents: &[AgentSpec],
) -> RoboticsResult<AdmmReport> {
    if agents.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "consensus ADMM needs at least one agent".to_string(),
        ));
    }
    if !config.rho.is_finite() || config.rho <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "consensus ADMM rho must be finite and positive".to_string(),
        ));
    }
    if config.max_iters == 0 {
        return Err(RoboticsError::InvalidParameter(
            "consensus ADMM max_iters must be positive".to_string(),
        ));
    }
    for a in agents {
        if !a.weight.is_finite() || a.weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "consensus ADMM agent weight must be finite and positive".to_string(),
            ));
        }
        if a.lower[0] > a.upper[0] || a.lower[1] > a.upper[1] {
            return Err(RoboticsError::InvalidParameter(
                "consensus ADMM agent box lower must not exceed upper".to_string(),
            ));
        }
    }

    let n = agents.len();
    let rho = config.rho;
    let mut x = vec![[0.0; 2]; n];
    let mut u = vec![[0.0; 2]; n];
    // Initialize the center at the mean preferred-minus-offset.
    let mut z = [0.0; 2];
    for a in agents {
        z[0] += (a.preferred[0] - a.offset[0]) / n as f64;
        z[1] += (a.preferred[1] - a.offset[1]) / n as f64;
    }
    for (xi, a) in x.iter_mut().zip(agents) {
        *xi = clamp2([z[0] + a.offset[0], z[1] + a.offset[1]], a.lower, a.upper);
    }

    let mut primal_residuals = Vec::new();
    let mut dual_residuals = Vec::new();
    let mut iterations = 0;

    for _ in 0..config.max_iters {
        iterations += 1;
        // x-update: local proximal step toward preferred and the consensus, then
        // project onto the box.
        for (xi, (a, ui)) in x.iter_mut().zip(agents.iter().zip(u.iter())) {
            let w = a.weight;
            for k in 0..2 {
                let target = (w * a.preferred[k] + rho * (z[k] + a.offset[k] - ui[k])) / (w + rho);
                xi[k] = target;
            }
            *xi = clamp2(*xi, a.lower, a.upper);
        }

        // z-update: consensus average of x_i - offset_i + u_i.
        let z_prev = z;
        z = [0.0; 2];
        for (xi, (a, ui)) in x.iter().zip(agents.iter().zip(u.iter())) {
            for k in 0..2 {
                z[k] += (xi[k] - a.offset[k] + ui[k]) / n as f64;
            }
        }

        // u-update and primal residual.
        let mut primal_sq = 0.0;
        for (xi, (a, ui)) in x.iter().zip(agents.iter().zip(u.iter_mut())) {
            for k in 0..2 {
                let r = xi[k] - z[k] - a.offset[k];
                ui[k] += r;
                primal_sq += r * r;
            }
        }
        let dz0 = z[0] - z_prev[0];
        let dz1 = z[1] - z_prev[1];
        let dual = rho * ((n as f64) * (dz0 * dz0 + dz1 * dz1)).sqrt();
        let primal = primal_sq.sqrt();
        primal_residuals.push(primal);
        dual_residuals.push(dual);

        if primal < config.tol && dual < config.tol {
            break;
        }
    }

    Ok(AdmmReport {
        positions: x,
        center: z,
        primal_residuals,
        dual_residuals,
        iterations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unconstrained_recovers_closed_form_center() {
        // With equal weights and no box, z* = mean(preferred - offset).
        let agents = [
            AgentSpec::new([1.0, 0.0], [1.0, 0.0]),
            AgentSpec::new([0.0, 2.0], [-1.0, 0.0]),
            AgentSpec::new([-1.0, -1.0], [0.0, 1.0]),
        ];
        let report = solve_formation_consensus(AdmmConfig::default(), &agents).unwrap();
        let mut zx = 0.0;
        let mut zy = 0.0;
        for a in &agents {
            zx += (a.preferred[0] - a.offset[0]) / 3.0;
            zy += (a.preferred[1] - a.offset[1]) / 3.0;
        }
        assert!(
            (report.center[0] - zx).abs() < 1e-4,
            "{} vs {}",
            report.center[0],
            zx
        );
        assert!(
            (report.center[1] - zy).abs() < 1e-4,
            "{} vs {}",
            report.center[1],
            zy
        );
        // Each agent sits at center + offset.
        for (xi, a) in report.positions.iter().zip(&agents) {
            assert!((xi[0] - (report.center[0] + a.offset[0])).abs() < 1e-3);
            assert!((xi[1] - (report.center[1] + a.offset[1])).abs() < 1e-3);
        }
    }

    #[test]
    fn residuals_decrease_to_convergence() {
        let agents = [
            AgentSpec::new([2.0, 1.0], [1.0, 1.0]),
            AgentSpec::new([-2.0, 1.0], [-1.0, 1.0]),
            AgentSpec::new([-2.0, -1.0], [-1.0, -1.0]),
            AgentSpec::new([2.0, -1.0], [1.0, -1.0]),
        ];
        let report = solve_formation_consensus(AdmmConfig::default(), &agents).unwrap();
        let first = report.primal_residuals[0];
        let last = *report.primal_residuals.last().unwrap();
        assert!(last < first, "primal should shrink: {first} -> {last}");
        assert!(last < 1e-5, "should converge: {last}");
    }

    #[test]
    fn box_constraint_is_respected() {
        // A constrained agent cannot reach its formation slot, shifting consensus.
        let agents = [
            AgentSpec::new([0.0, 0.0], [1.0, 0.0]),
            AgentSpec::new([0.0, 0.0], [-1.0, 0.0]).with_box([-0.4, -1.0], [0.4, 1.0]),
        ];
        let report = solve_formation_consensus(AdmmConfig::default(), &agents).unwrap();
        let x1 = report.positions[1];
        assert!(
            x1[0] >= -0.4 - 1e-9 && x1[0] <= 0.4 + 1e-9,
            "x {x1:?} in box"
        );
    }

    #[test]
    fn is_deterministic() {
        let agents = [
            AgentSpec::new([1.0, 0.0], [0.5, 0.5]),
            AgentSpec::new([0.0, 1.0], [-0.5, 0.5]),
            AgentSpec::new([-1.0, 0.0], [-0.5, -0.5]),
        ];
        let a = solve_formation_consensus(AdmmConfig::default(), &agents).unwrap();
        let b = solve_formation_consensus(AdmmConfig::default(), &agents).unwrap();
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.center, b.center);
        assert_eq!(a.primal_residuals, b.primal_residuals);
    }

    #[test]
    fn rejects_invalid_input() {
        assert!(solve_formation_consensus(AdmmConfig::default(), &[]).is_err());
        let bad_rho = AdmmConfig {
            rho: 0.0,
            ..AdmmConfig::default()
        };
        assert!(
            solve_formation_consensus(bad_rho, &[AgentSpec::new([0.0, 0.0], [0.0, 0.0])]).is_err()
        );
    }
}
