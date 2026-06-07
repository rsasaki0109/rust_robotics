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
//! - [`solve_formation_consensus`] runs (centralized) consensus ADMM and returns
//!   the agent positions, the consensus center, and the primal/dual residuals.
//! - [`solve_graph_consensus`] runs the *decentralized* edge-based variant where
//!   agents agree only with communication-graph neighbors (no global average);
//!   the convergence rate is set by the graph connectivity.
//! - [`solve_horizon_consensus`] is the *receding-horizon* variant: agents agree
//!   on a shared center **trajectory** over a short horizon (rather than a single
//!   static center), with a temporal smoothness penalty that couples the center
//!   across time and turns the consensus z-update into a banded linear solve.

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

/// Result of a decentralized (graph) consensus-ADMM solve.
#[derive(Debug, Clone, PartialEq)]
pub struct GraphConsensusReport {
    /// Final agent positions `x_i`.
    pub positions: Vec<[f64; 2]>,
    /// Mean consensus value (the agreed formation center).
    pub center: [f64; 2],
    /// Edge-disagreement norm per iteration (`sqrt(sum_edges ||y_i - y_j||^2)`).
    pub disagreement: Vec<f64>,
    /// Iterations actually run.
    pub iterations: usize,
}

/// Solve formation consensus *decentralized* over a communication graph, where
/// each agent exchanges only with its graph neighbors (no global average).
///
/// This is the standard edge-based decentralized ADMM for
/// `minimize sum_i (w_i/2)||x_i - a_i||^2  s.t.  x_i - offset_i = x_j - offset_j`
/// for every edge `(i, j)`. Each agent keeps an aggregated dual `alpha_i` over
/// its incident edges and updates
/// `y_i = (w a_i' - alpha_i + rho * sum_{j in N_i}(y_i + y_j)) / (w + 2 rho deg_i)`
/// using only neighbor values `y_j` (with `a_i' = a_i - offset_i` and the
/// position recovered as `x_i = y_i + offset_i`, projected onto the box). On a
/// connected graph it converges to the same weighted-average consensus as the
/// centralized solver; the convergence rate is set by the graph connectivity.
#[allow(clippy::needless_range_loop)]
pub fn solve_graph_consensus(
    config: AdmmConfig,
    agents: &[AgentSpec],
    edges: &[(usize, usize)],
) -> RoboticsResult<GraphConsensusReport> {
    if agents.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "graph consensus needs at least one agent".to_string(),
        ));
    }
    if !config.rho.is_finite() || config.rho <= 0.0 || config.max_iters == 0 {
        return Err(RoboticsError::InvalidParameter(
            "graph consensus rho must be positive and max_iters > 0".to_string(),
        ));
    }
    let n = agents.len();
    for a in agents {
        if !a.weight.is_finite() || a.weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "graph consensus agent weight must be finite and positive".to_string(),
            ));
        }
    }
    let mut neighbors = vec![Vec::new(); n];
    for &(i, j) in edges {
        if i >= n || j >= n || i == j {
            return Err(RoboticsError::InvalidParameter(
                "graph consensus edge endpoints out of range or self-loop".to_string(),
            ));
        }
        neighbors[i].push(j);
        neighbors[j].push(i);
    }

    let rho = config.rho;
    // Shifted preferred values a_i' = a_i - offset_i; consensus is on y_i.
    let ashift: Vec<[f64; 2]> = agents
        .iter()
        .map(|a| [a.preferred[0] - a.offset[0], a.preferred[1] - a.offset[1]])
        .collect();
    let mut y = ashift.clone();
    let mut alpha = vec![[0.0; 2]; n];

    let project = |y_i: [f64; 2], a: &AgentSpec| -> [f64; 2] {
        let x = clamp2(
            [y_i[0] + a.offset[0], y_i[1] + a.offset[1]],
            a.lower,
            a.upper,
        );
        [x[0] - a.offset[0], x[1] - a.offset[1]]
    };

    let mut disagreement = Vec::new();
    let mut iterations = 0;

    for _ in 0..config.max_iters {
        iterations += 1;
        let y_prev = y.clone();
        // Synchronous local updates using only neighbor values.
        for i in 0..n {
            let w = agents[i].weight;
            let deg = neighbors[i].len() as f64;
            for k in 0..2 {
                let mut neigh_sum = 0.0;
                for &j in &neighbors[i] {
                    neigh_sum += y_prev[i][k] + y_prev[j][k];
                }
                y[i][k] =
                    (w * ashift[i][k] - alpha[i][k] + rho * neigh_sum) / (w + 2.0 * rho * deg);
            }
            y[i] = project(y[i], &agents[i]);
        }
        // Dual update from the updated disagreements.
        for i in 0..n {
            for k in 0..2 {
                let mut diff = 0.0;
                for &j in &neighbors[i] {
                    diff += y[i][k] - y[j][k];
                }
                alpha[i][k] += rho * diff;
            }
        }
        // Edge-disagreement and step size.
        let mut dis_sq = 0.0;
        for &(i, j) in edges {
            for k in 0..2 {
                let d = y[i][k] - y[j][k];
                dis_sq += d * d;
            }
        }
        let mut step_sq = 0.0;
        for i in 0..n {
            for k in 0..2 {
                let d = y[i][k] - y_prev[i][k];
                step_sq += d * d;
            }
        }
        disagreement.push(dis_sq.sqrt());
        if dis_sq.sqrt() < config.tol && step_sq.sqrt() < config.tol {
            break;
        }
    }

    let mut center = [0.0; 2];
    for yi in &y {
        center[0] += yi[0] / n as f64;
        center[1] += yi[1] / n as f64;
    }
    let positions: Vec<[f64; 2]> = y
        .iter()
        .zip(agents)
        .map(|(yi, a)| [yi[0] + a.offset[0], yi[1] + a.offset[1]])
        .collect();

    Ok(GraphConsensusReport {
        positions,
        center,
        disagreement,
        iterations,
    })
}

/// One agent's planning slice in the receding-horizon (trajectory) consensus
/// problem: a per-step reference trajectory, a constant formation offset, a cost
/// weight, and an optional box constraint applied at every step.
#[derive(Debug, Clone, PartialEq)]
pub struct AgentTrajectory {
    /// Per-step reference positions `a_i[t]` the agent is pulled toward.
    pub reference: Vec<[f64; 2]>,
    /// Constant formation offset `offset_i` from the shared center trajectory.
    pub offset: [f64; 2],
    /// Cost weight on staying near the reference (relative to the consensus pull).
    pub weight: f64,
    /// Lower box bound, applied to every step (`f64::NEG_INFINITY` disables).
    pub lower: [f64; 2],
    /// Upper box bound, applied to every step (`f64::INFINITY` disables).
    pub upper: [f64; 2],
}

impl AgentTrajectory {
    /// An unconstrained agent with unit weight tracking `reference`.
    pub fn new(reference: Vec<[f64; 2]>, offset: [f64; 2]) -> Self {
        Self {
            reference,
            offset,
            weight: 1.0,
            lower: [f64::NEG_INFINITY; 2],
            upper: [f64::INFINITY; 2],
        }
    }

    /// Set a per-step box constraint on the agent's position.
    pub fn with_box(mut self, lower: [f64; 2], upper: [f64; 2]) -> Self {
        self.lower = lower;
        self.upper = upper;
        self
    }

    /// Set the cost weight on tracking the reference.
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }
}

/// Result of a receding-horizon (trajectory) consensus-ADMM solve.
#[derive(Debug, Clone, PartialEq)]
pub struct HorizonConsensusReport {
    /// Shared center trajectory `z[t]`, length `H`.
    pub center: Vec<[f64; 2]>,
    /// Per-agent trajectories `x_i[t]`.
    pub trajectories: Vec<Vec<[f64; 2]>>,
    /// Primal residual norm per iteration (`sqrt(sum_{i,t} ||x_i[t]-z[t]-offset_i||^2)`).
    pub primal_residuals: Vec<f64>,
    /// Dual residual norm per iteration.
    pub dual_residuals: Vec<f64>,
    /// Iterations actually run.
    pub iterations: usize,
}

/// Dense Cholesky factorization `A = L L^T` of a symmetric positive-definite
/// matrix; returns the lower-triangular `L`. Panics only on a non-SPD matrix,
/// which the construction here precludes.
#[allow(clippy::needless_range_loop)]
fn cholesky(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let m = a.len();
    let mut l = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..=i {
            let mut sum = a[i][j];
            for k in 0..j {
                sum -= l[i][k] * l[j][k];
            }
            if i == j {
                l[i][j] = sum.max(0.0).sqrt();
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    l
}

/// Solve `L L^T x = b` in place given the Cholesky factor `L`.
fn chol_solve(l: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let m = l.len();
    let mut y = vec![0.0; m];
    for i in 0..m {
        let mut sum = b[i];
        for k in 0..i {
            sum -= l[i][k] * y[k];
        }
        y[i] = sum / l[i][i];
    }
    let mut x = vec![0.0; m];
    for i in (0..m).rev() {
        let mut sum = y[i];
        for k in (i + 1)..m {
            sum -= l[k][i] * x[k];
        }
        x[i] = sum / l[i][i];
    }
    x
}

/// Solve the *receding-horizon* formation-consensus problem: agents agree on a
/// shared center **trajectory** `z[0..H]` rather than a single static center.
///
/// Each agent tracks a per-step reference `a_i[t]`, sits at `z[t] + offset_i`,
/// and respects a per-step box; the shared center carries a temporal-smoothness
/// (acceleration) penalty `(smooth_weight/2) sum_t ||z[t+1]-2 z[t]+z[t-1]||^2`
/// that couples the center across time. The full problem is
/// `minimize sum_{i,t} (w_i/2)||x_i[t]-a_i[t]||^2 + (smooth_weight/2) sum_t
/// ||z[t+1]-2z[t]+z[t-1]||^2  s.t.  x_i[t] - offset_i = z[t]` with per-step boxes.
///
/// Consensus ADMM mirrors [`solve_formation_consensus`], but the z-update is no
/// longer a per-step average: the smoothness term makes it a banded
/// (pentadiagonal) symmetric-positive-definite linear solve
/// `(rho N I + smooth_weight D^T D) z = rho sum_i (x_i - offset_i + u_i)` per
/// coordinate, where `D` is the second-difference operator. The system matrix is
/// constant across iterations, so it is Cholesky-factorized once and back-solved
/// each iteration. An optional `anchor` fixes `z[0]` (the current team center),
/// which is what makes the solve usable as the inner step of a receding-horizon
/// MPC loop: solve over the horizon, apply `z[1]`, shift, and re-solve.
///
/// With `smooth_weight = 0` and a one-step horizon this reduces exactly to the
/// static centralized consensus of [`solve_formation_consensus`].
#[allow(clippy::needless_range_loop)]
pub fn solve_horizon_consensus(
    config: AdmmConfig,
    agents: &[AgentTrajectory],
    smooth_weight: f64,
    anchor: Option<[f64; 2]>,
) -> RoboticsResult<HorizonConsensusReport> {
    if agents.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "horizon consensus needs at least one agent".to_string(),
        ));
    }
    if !config.rho.is_finite() || config.rho <= 0.0 || config.max_iters == 0 {
        return Err(RoboticsError::InvalidParameter(
            "horizon consensus rho must be positive and max_iters > 0".to_string(),
        ));
    }
    if !smooth_weight.is_finite() || smooth_weight < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "horizon consensus smooth_weight must be finite and non-negative".to_string(),
        ));
    }
    let horizon = agents[0].reference.len();
    if horizon == 0 {
        return Err(RoboticsError::InvalidParameter(
            "horizon consensus reference trajectories must be non-empty".to_string(),
        ));
    }
    for a in agents {
        if a.reference.len() != horizon {
            return Err(RoboticsError::InvalidParameter(
                "horizon consensus reference trajectories must share one length".to_string(),
            ));
        }
        if !a.weight.is_finite() || a.weight <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "horizon consensus agent weight must be finite and positive".to_string(),
            ));
        }
        if a.lower[0] > a.upper[0] || a.lower[1] > a.upper[1] {
            return Err(RoboticsError::InvalidParameter(
                "horizon consensus agent box lower must not exceed upper".to_string(),
            ));
        }
    }
    if let Some(p) = anchor {
        if !p[0].is_finite() || !p[1].is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "horizon consensus anchor must be finite".to_string(),
            ));
        }
    }

    let n = agents.len();
    let rho = config.rho;
    let anchored = anchor.is_some();

    // Build the dense z-update system matrix A = rho N I + smooth_weight D^T D,
    // where D is the second-difference (acceleration) operator. A is constant
    // across ADMM iterations, so it is factorized once below.
    let mut a_mat = vec![vec![0.0; horizon]; horizon];
    for t in 0..horizon {
        a_mat[t][t] += rho * n as f64;
    }
    if smooth_weight > 0.0 {
        // One acceleration row per interior step t: coeffs (1,-2,1) at (t-1,t,t+1).
        for t in 1..horizon.saturating_sub(1) {
            let idx = [t - 1, t, t + 1];
            let coeff = [1.0, -2.0, 1.0];
            for (a_i, &ia) in idx.iter().enumerate() {
                for (b_i, &ib) in idx.iter().enumerate() {
                    a_mat[ia][ib] += smooth_weight * coeff[a_i] * coeff[b_i];
                }
            }
        }
    }

    // Free indices: all steps, or steps 1.. when z[0] is anchored.
    let free: Vec<usize> = if anchored {
        (1..horizon).collect()
    } else {
        (0..horizon).collect()
    };
    let m = free.len();
    // Reduced system over the free indices (anchored z[0] moves to the RHS).
    let mut a_red = vec![vec![0.0; m]; m];
    for (r, &ir) in free.iter().enumerate() {
        for (c, &ic) in free.iter().enumerate() {
            a_red[r][c] = a_mat[ir][ic];
        }
    }
    // When m == 0 (anchored, horizon == 1) the whole trajectory is the anchor.
    let chol = if m > 0 { cholesky(&a_red) } else { Vec::new() };

    // Initialize the center trajectory at the per-step mean reference-minus-offset.
    let mut z = vec![[0.0; 2]; horizon];
    for t in 0..horizon {
        for a in agents {
            z[t][0] += (a.reference[t][0] - a.offset[0]) / n as f64;
            z[t][1] += (a.reference[t][1] - a.offset[1]) / n as f64;
        }
    }
    if let Some(p) = anchor {
        z[0] = p;
    }
    let mut x: Vec<Vec<[f64; 2]>> = agents
        .iter()
        .map(|a| {
            (0..horizon)
                .map(|t| {
                    clamp2(
                        [z[t][0] + a.offset[0], z[t][1] + a.offset[1]],
                        a.lower,
                        a.upper,
                    )
                })
                .collect()
        })
        .collect();
    let mut u = vec![vec![[0.0; 2]; horizon]; n];

    let mut primal_residuals = Vec::new();
    let mut dual_residuals = Vec::new();
    let mut iterations = 0;

    for _ in 0..config.max_iters {
        iterations += 1;
        // x-update: per agent, per step — local proximal step then box projection.
        for (xi, (a, ui)) in x.iter_mut().zip(agents.iter().zip(u.iter())) {
            let w = a.weight;
            for t in 0..horizon {
                for k in 0..2 {
                    xi[t][k] = (w * a.reference[t][k] + rho * (z[t][k] + a.offset[k] - ui[t][k]))
                        / (w + rho);
                }
                xi[t] = clamp2(xi[t], a.lower, a.upper);
            }
        }

        // z-update: banded SPD solve per coordinate (couples the center in time).
        let z_prev = z.clone();
        for k in 0..2 {
            // RHS b[t] = rho * sum_i (x_i[t] - offset_i + u_i[t]).
            let mut b_full = vec![0.0; horizon];
            for t in 0..horizon {
                let mut s = 0.0;
                for (xi, (a, ui)) in x.iter().zip(agents.iter().zip(u.iter())) {
                    s += xi[t][k] - a.offset[k] + ui[t][k];
                }
                b_full[t] = rho * s;
            }
            if m == 0 {
                continue;
            }
            // Reduce: move the anchored z[0] contribution to the RHS.
            let mut b_red = vec![0.0; m];
            for (r, &ir) in free.iter().enumerate() {
                let mut v = b_full[ir];
                if anchored {
                    v -= a_mat[ir][0] * z[0][k];
                }
                b_red[r] = v;
            }
            let sol = chol_solve(&chol, &b_red);
            for (r, &ir) in free.iter().enumerate() {
                z[ir][k] = sol[r];
            }
        }

        // dual update and primal residual.
        let mut primal_sq = 0.0;
        for (xi, (a, ui)) in x.iter().zip(agents.iter().zip(u.iter_mut())) {
            for t in 0..horizon {
                for k in 0..2 {
                    let r = xi[t][k] - z[t][k] - a.offset[k];
                    ui[t][k] += r;
                    primal_sq += r * r;
                }
            }
        }
        let mut dz_sq = 0.0;
        for t in 0..horizon {
            for k in 0..2 {
                let d = z[t][k] - z_prev[t][k];
                dz_sq += d * d;
            }
        }
        let primal = primal_sq.sqrt();
        let dual = rho * (n as f64 * dz_sq).sqrt();
        primal_residuals.push(primal);
        dual_residuals.push(dual);
        if primal < config.tol && dual < config.tol {
            break;
        }
    }

    Ok(HorizonConsensusReport {
        center: z,
        trajectories: x,
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

    fn ring_edges(n: usize) -> Vec<(usize, usize)> {
        (0..n).map(|i| (i, (i + 1) % n)).collect()
    }

    fn line_edges(n: usize) -> Vec<(usize, usize)> {
        (0..n - 1).map(|i| (i, i + 1)).collect()
    }

    fn complete_edges(n: usize) -> Vec<(usize, usize)> {
        let mut e = Vec::new();
        for i in 0..n {
            for j in (i + 1)..n {
                e.push((i, j));
            }
        }
        e
    }

    #[test]
    fn graph_consensus_matches_centralized_center() {
        let agents = [
            AgentSpec::new([2.0, 1.0], [1.0, 1.0]),
            AgentSpec::new([-2.0, 1.0], [-1.0, 1.0]),
            AgentSpec::new([-2.0, -1.0], [-1.0, -1.0]),
            AgentSpec::new([2.0, -1.0], [1.0, -1.0]),
        ];
        let config = AdmmConfig {
            max_iters: 2000,
            tol: 1e-8,
            ..AdmmConfig::default()
        };
        let report = solve_graph_consensus(config, &agents, &ring_edges(4)).unwrap();
        // Closed-form weighted-average consensus center.
        let mut zx = 0.0;
        let mut zy = 0.0;
        for a in &agents {
            zx += (a.preferred[0] - a.offset[0]) / 4.0;
            zy += (a.preferred[1] - a.offset[1]) / 4.0;
        }
        assert!(
            (report.center[0] - zx).abs() < 1e-3,
            "{} vs {}",
            report.center[0],
            zx
        );
        assert!(
            (report.center[1] - zy).abs() < 1e-3,
            "{} vs {}",
            report.center[1],
            zy
        );
        assert!(*report.disagreement.last().unwrap() < 1e-6);
    }

    #[test]
    fn connectivity_speeds_convergence() {
        let agents: Vec<AgentSpec> = (0..6)
            .map(|i| {
                let f = i as f64;
                AgentSpec::new([f - 2.5, (f * 1.3).sin()], [0.0, 0.0])
            })
            .collect();
        let config = AdmmConfig {
            max_iters: 5000,
            tol: 1e-6,
            ..AdmmConfig::default()
        };
        let line = solve_graph_consensus(config, &agents, &line_edges(6)).unwrap();
        let complete = solve_graph_consensus(config, &agents, &complete_edges(6)).unwrap();
        assert!(
            complete.iterations < line.iterations,
            "complete graph ({}) should converge faster than line ({})",
            complete.iterations,
            line.iterations
        );
    }

    #[test]
    fn graph_consensus_is_deterministic() {
        let agents = [
            AgentSpec::new([1.0, 0.0], [0.5, 0.5]),
            AgentSpec::new([0.0, 1.0], [-0.5, 0.5]),
            AgentSpec::new([-1.0, 0.0], [-0.5, -0.5]),
        ];
        let e = ring_edges(3);
        let a = solve_graph_consensus(AdmmConfig::default(), &agents, &e).unwrap();
        let b = solve_graph_consensus(AdmmConfig::default(), &agents, &e).unwrap();
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.disagreement, b.disagreement);
    }

    #[test]
    fn graph_consensus_rejects_bad_edges() {
        let agents = [AgentSpec::new([0.0, 0.0], [0.0, 0.0])];
        assert!(solve_graph_consensus(AdmmConfig::default(), &agents, &[(0, 5)]).is_err());
        assert!(solve_graph_consensus(AdmmConfig::default(), &agents, &[(0, 0)]).is_err());
    }

    #[test]
    fn horizon_one_step_no_smoothing_matches_static() {
        // With smooth_weight = 0 and a one-step horizon the trajectory consensus
        // collapses to the static centralized consensus.
        let statics = [
            AgentSpec::new([2.0, 1.0], [1.0, 1.0]),
            AgentSpec::new([-2.0, 1.0], [-1.0, 1.0]),
            AgentSpec::new([-2.0, -1.0], [-1.0, -1.0]),
        ];
        let trajs: Vec<AgentTrajectory> = statics
            .iter()
            .map(|s| AgentTrajectory::new(vec![s.preferred], s.offset))
            .collect();
        let stat = solve_formation_consensus(AdmmConfig::default(), &statics).unwrap();
        let horiz = solve_horizon_consensus(AdmmConfig::default(), &trajs, 0.0, None).unwrap();
        assert!((horiz.center[0][0] - stat.center[0]).abs() < 1e-6);
        assert!((horiz.center[0][1] - stat.center[1]).abs() < 1e-6);
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn smoothing_reduces_center_acceleration() {
        // A jagged set of references; smoothing should flatten the center path.
        let h = 16;
        let refs: Vec<[f64; 2]> = (0..h)
            .map(|t| {
                let f = t as f64;
                [f * 0.3, if t % 2 == 0 { 1.0 } else { -1.0 }]
            })
            .collect();
        let agents = vec![AgentTrajectory::new(refs, [0.0, 0.0])];
        let config = AdmmConfig {
            max_iters: 2000,
            tol: 1e-9,
            ..AdmmConfig::default()
        };
        let accel = |r: &HorizonConsensusReport| {
            let z = &r.center;
            let mut s = 0.0;
            for t in 1..z.len() - 1 {
                for k in 0..2 {
                    let a = z[t + 1][k] - 2.0 * z[t][k] + z[t - 1][k];
                    s += a * a;
                }
            }
            s
        };
        let stiff = solve_horizon_consensus(config, &agents, 0.0, None).unwrap();
        let smooth = solve_horizon_consensus(config, &agents, 20.0, None).unwrap();
        assert!(
            accel(&smooth) < 0.25 * accel(&stiff),
            "smoothing should cut acceleration: {} vs {}",
            accel(&smooth),
            accel(&stiff)
        );
    }

    #[test]
    fn anchor_fixes_first_center_step() {
        let h = 8;
        let refs: Vec<[f64; 2]> = (0..h).map(|t| [t as f64, 0.0]).collect();
        let agents = vec![AgentTrajectory::new(refs, [0.0, 0.0])];
        let anchor = [-5.0, 2.0];
        let report = solve_horizon_consensus(
            AdmmConfig {
                max_iters: 500,
                ..AdmmConfig::default()
            },
            &agents,
            5.0,
            Some(anchor),
        )
        .unwrap();
        assert!((report.center[0][0] - anchor[0]).abs() < 1e-9);
        assert!((report.center[0][1] - anchor[1]).abs() < 1e-9);
    }

    #[test]
    fn horizon_box_is_respected() {
        let h = 6;
        let refs: Vec<[f64; 2]> = (0..h).map(|t| [t as f64, 3.0]).collect();
        let agents = vec![
            AgentTrajectory::new(refs.clone(), [0.0, 0.5]),
            AgentTrajectory::new(refs, [0.0, -0.5]).with_box([-100.0, -1.0], [100.0, 1.0]),
        ];
        let report = solve_horizon_consensus(AdmmConfig::default(), &agents, 1.0, None).unwrap();
        for p in &report.trajectories[1] {
            assert!(p[1] >= -1.0 - 1e-9 && p[1] <= 1.0 + 1e-9, "y {p:?} in box");
        }
    }

    #[test]
    fn horizon_consensus_is_deterministic() {
        let refs: Vec<[f64; 2]> = (0..10)
            .map(|t| [t as f64 * 0.5, (t as f64).sin()])
            .collect();
        let agents = vec![
            AgentTrajectory::new(refs.clone(), [0.5, 0.5]),
            AgentTrajectory::new(refs, [-0.5, -0.5]),
        ];
        let a =
            solve_horizon_consensus(AdmmConfig::default(), &agents, 3.0, Some([0.0, 0.0])).unwrap();
        let b =
            solve_horizon_consensus(AdmmConfig::default(), &agents, 3.0, Some([0.0, 0.0])).unwrap();
        assert_eq!(a.center, b.center);
        assert_eq!(a.trajectories, b.trajectories);
        assert_eq!(a.primal_residuals, b.primal_residuals);
    }

    #[test]
    fn horizon_rejects_invalid_input() {
        assert!(solve_horizon_consensus(AdmmConfig::default(), &[], 1.0, None).is_err());
        let a = vec![AgentTrajectory::new(vec![[0.0, 0.0]], [0.0, 0.0])];
        // Negative smoothing.
        assert!(solve_horizon_consensus(AdmmConfig::default(), &a, -1.0, None).is_err());
        // Mismatched trajectory lengths.
        let mixed = vec![
            AgentTrajectory::new(vec![[0.0, 0.0], [1.0, 0.0]], [0.0, 0.0]),
            AgentTrajectory::new(vec![[0.0, 0.0]], [0.0, 0.0]),
        ];
        assert!(solve_horizon_consensus(AdmmConfig::default(), &mixed, 1.0, None).is_err());
        // Empty trajectory.
        let empty = vec![AgentTrajectory::new(vec![], [0.0, 0.0])];
        assert!(solve_horizon_consensus(AdmmConfig::default(), &empty, 1.0, None).is_err());
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
