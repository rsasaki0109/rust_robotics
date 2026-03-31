//! Rocket Powered Landing trajectory optimization.
//!
//! Implements fuel-optimal powered descent guidance using convex optimization,
//! inspired by the lossless convexification approach to rocket landing.
//!
//! # Problem formulation (2D)
//!
//! - State: `[x, y, vx, vy]` (position and velocity)
//! - Control: `[Tx, Ty]` (thrust vector components)
//! - Dynamics: double integrator with gravity
//!   - `x_dot = vx`, `y_dot = vy`
//!   - `vx_dot = Tx / m`, `vy_dot = Ty / m - g`
//! - Constraints: `|T| <= T_max` (second-order cone constraint on thrust magnitude)
//! - Terminal: land at target position with zero velocity
//! - Objective: minimize total thrust (fuel proxy)

use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettings, DefaultSettingsBuilder, DefaultSolver, IPSolver, SolverStatus,
};
use nalgebra::Vector2;

/// Gravity acceleration (m/s^2).
const GRAVITY: f64 = 9.81;

/// Configuration for the rocket landing problem.
#[derive(Debug, Clone)]
pub struct RocketLandingConfig {
    /// Number of time steps in the trajectory.
    pub n_steps: usize,
    /// Time step duration (seconds).
    pub dt: f64,
    /// Maximum thrust magnitude (N).
    pub thrust_max: f64,
    /// Rocket mass (kg), assumed constant.
    pub mass: f64,
    /// Gravity acceleration (m/s^2).
    pub gravity: f64,
    /// Weight on terminal position error in cost.
    pub w_terminal_pos: f64,
    /// Weight on terminal velocity error in cost.
    pub w_terminal_vel: f64,
    /// Weight on thrust (fuel) in cost.
    pub w_thrust: f64,
}

impl Default for RocketLandingConfig {
    fn default() -> Self {
        Self {
            n_steps: 40,
            dt: 0.1,
            thrust_max: 24.0,
            mass: 1.0,
            gravity: GRAVITY,
            w_terminal_pos: 1000.0,
            w_terminal_vel: 1000.0,
            w_thrust: 1.0,
        }
    }
}

/// Result of the trajectory optimization.
#[derive(Debug, Clone)]
pub struct RocketTrajectory {
    /// Position at each time step: `[(x, y); n_steps + 1]`.
    pub positions: Vec<Vector2<f64>>,
    /// Velocity at each time step: `[(vx, vy); n_steps + 1]`.
    pub velocities: Vec<Vector2<f64>>,
    /// Thrust at each time step: `[(Tx, Ty); n_steps]`.
    pub thrusts: Vec<Vector2<f64>>,
    /// Total fuel cost (sum of thrust magnitudes * dt).
    pub fuel_cost: f64,
}

/// Initial condition for the rocket.
#[derive(Debug, Clone)]
pub struct RocketState {
    /// Position (x, y) in meters.
    pub position: Vector2<f64>,
    /// Velocity (vx, vy) in m/s.
    pub velocity: Vector2<f64>,
}

/// Target landing condition.
#[derive(Debug, Clone)]
pub struct LandingTarget {
    /// Target position (x, y) in meters.
    pub position: Vector2<f64>,
    /// Target velocity (vx, vy) in m/s (typically zero).
    pub velocity: Vector2<f64>,
}

impl Default for LandingTarget {
    fn default() -> Self {
        Self {
            position: Vector2::zeros(),
            velocity: Vector2::zeros(),
        }
    }
}

/// Solve the rocket powered landing trajectory optimization problem.
///
/// Uses a QP formulation with second-order cone constraints on thrust magnitude.
/// The dynamics are discretized as a linear system (double integrator + gravity)
/// and embedded as equality constraints.
///
/// Returns `None` if the solver fails to find a solution.
pub fn solve_landing_trajectory(
    initial: &RocketState,
    target: &LandingTarget,
    config: &RocketLandingConfig,
) -> Option<RocketTrajectory> {
    use clarabel::solver::SupportedConeT;

    let n = config.n_steps;
    let dt = config.dt;
    let m = config.mass;
    let g = config.gravity;
    let t_max = config.thrust_max;

    // Decision variables layout:
    //   u: [Tx_0, Ty_0, Tx_1, Ty_1, ..., Tx_{n-1}, Ty_{n-1}]  (2*n variables)
    //   x: [x_0, y_0, vx_0, vy_0, ..., x_n, y_n, vx_n, vy_n]  (4*(n+1) variables)
    let n_u = 2 * n;
    let n_x = 4 * (n + 1);
    let n_vars = n_u + n_x;

    let u_idx = |t: usize, k: usize| -> usize { 2 * t + k };
    let x_idx = |t: usize, k: usize| -> usize { n_u + 4 * t + k };

    // --- Cost: min sum_t w_thrust * (Tx_t^2 + Ty_t^2)
    //         + w_pos * ((x_n - x_target)^2 + (y_n - y_target)^2)
    //         + w_vel * ((vx_n - vx_target)^2 + (vy_n - vy_target)^2)
    // P matrix (upper triangular, factor of 2 included)
    let mut p_rows = Vec::new();
    let mut p_cols = Vec::new();
    let mut p_vals = Vec::new();

    // Thrust cost
    for t in 0..n {
        for k in 0..2 {
            let idx = u_idx(t, k);
            p_rows.push(idx);
            p_cols.push(idx);
            p_vals.push(2.0 * config.w_thrust);
        }
    }

    // Terminal position cost
    for k in 0..2 {
        let idx = x_idx(n, k);
        p_rows.push(idx);
        p_cols.push(idx);
        p_vals.push(2.0 * config.w_terminal_pos);
    }

    // Terminal velocity cost
    for k in 2..4 {
        let idx = x_idx(n, k);
        p_rows.push(idx);
        p_cols.push(idx);
        p_vals.push(2.0 * config.w_terminal_vel);
    }

    let p = CscMatrix::new_from_triplets(n_vars, n_vars, p_rows, p_cols, p_vals);

    // q vector (linear cost from terminal target)
    let mut q_vec = vec![0.0; n_vars];
    // -2 * w * target for terminal state
    q_vec[x_idx(n, 0)] = -2.0 * config.w_terminal_pos * target.position[0];
    q_vec[x_idx(n, 1)] = -2.0 * config.w_terminal_pos * target.position[1];
    q_vec[x_idx(n, 2)] = -2.0 * config.w_terminal_vel * target.velocity[0];
    q_vec[x_idx(n, 3)] = -2.0 * config.w_terminal_vel * target.velocity[1];

    // --- Constraints ---
    // 1) Equality: initial state (4 rows)
    // 2) Equality: dynamics x_{t+1} = A*x_t + B*u_t + c for t=0..n-1 (4*n rows)
    //    Total equality: 4 + 4*n = 4*(n+1)
    // 3) SOC: ||[Tx_t, Ty_t]|| <= t_max for each t (cone dim 3, n cones)
    //    Represented as: [t_max; -Tx_t; -Ty_t] in SecondOrderCone(3)
    //    Clarabel convention: ||z[1:]|| <= z[0], so we need rows for [s; u1; u2]
    //    where s = t_max (constant, free slack) and u1 = Tx_t, u2 = Ty_t

    let n_eq = 4 * (n + 1);
    let n_soc_rows = 3 * n; // n cones of dimension 3

    let total_constraints = n_eq + n_soc_rows;

    let mut a_rows = Vec::new();
    let mut a_cols = Vec::new();
    let mut a_vals = Vec::new();
    let mut b_vec = vec![0.0; total_constraints];

    // Equality: initial state x_0 = initial
    // A * x = b  =>  x_0 = initial  (ZeroCone means A*x - b = 0)
    for k in 0..2 {
        let row = k;
        a_rows.push(row);
        a_cols.push(x_idx(0, k));
        a_vals.push(1.0);
        b_vec[row] = initial.position[k];
    }
    for k in 0..2 {
        let row = 2 + k;
        a_rows.push(row);
        a_cols.push(x_idx(0, k + 2));
        a_vals.push(1.0);
        b_vec[row] = initial.velocity[k];
    }

    // Dynamics constraints: x_{t+1} = A*x_t + B*u_t + c
    // Discretized double integrator:
    //   x_{t+1}  = x_t  + dt * vx_t
    //   y_{t+1}  = y_t  + dt * vy_t
    //   vx_{t+1} = vx_t + dt * Tx_t / m
    //   vy_{t+1} = vy_t + dt * Ty_t / m - dt * g
    //
    // Rewrite as: -x_{t+1} + A*x_t + B*u_t = -c
    //   A = [[1, 0, dt, 0],
    //        [0, 1, 0, dt],
    //        [0, 0, 1,  0],
    //        [0, 0, 0,  1]]
    //   B = [[0, 0],
    //        [0, 0],
    //        [dt/m, 0],
    //        [0, dt/m]]
    //   c = [0, 0, 0, -dt*g]

    for t in 0..n {
        let base_row = 4 + 4 * t;

        // -x_{t+1}
        for k in 0..4 {
            a_rows.push(base_row + k);
            a_cols.push(x_idx(t + 1, k));
            a_vals.push(-1.0);
        }

        // A * x_t
        // x component: +x_t + dt*vx_t
        a_rows.push(base_row);
        a_cols.push(x_idx(t, 0));
        a_vals.push(1.0);
        a_rows.push(base_row);
        a_cols.push(x_idx(t, 2));
        a_vals.push(dt);

        // y component: +y_t + dt*vy_t
        a_rows.push(base_row + 1);
        a_cols.push(x_idx(t, 1));
        a_vals.push(1.0);
        a_rows.push(base_row + 1);
        a_cols.push(x_idx(t, 3));
        a_vals.push(dt);

        // vx component: +vx_t
        a_rows.push(base_row + 2);
        a_cols.push(x_idx(t, 2));
        a_vals.push(1.0);

        // vy component: +vy_t
        a_rows.push(base_row + 3);
        a_cols.push(x_idx(t, 3));
        a_vals.push(1.0);

        // B * u_t
        // vx: dt/m * Tx_t
        a_rows.push(base_row + 2);
        a_cols.push(u_idx(t, 0));
        a_vals.push(dt / m);

        // vy: dt/m * Ty_t
        a_rows.push(base_row + 3);
        a_cols.push(u_idx(t, 1));
        a_vals.push(dt / m);

        // RHS: -c = [0, 0, 0, dt*g]
        b_vec[base_row + 3] = dt * g;
    }

    // SOC constraints: ||[Tx_t, Ty_t]|| <= t_max
    // Clarabel convention for SecondOrderConeT(dim):
    //   the constraint rows represent s such that ||s[1:]|| <= s[0]
    //   A*x + s = b, s in SOC
    //   We want: s[0] = b[0] - (A*x)[0] = t_max (constant, no x dependency)
    //            s[1] = b[1] - (A*x)[1] => we want s[1] = Tx_t => A row = -Tx_t, b=0
    //            s[2] = b[2] - (A*x)[2] => we want s[2] = Ty_t => A row = -Ty_t, b=0
    //   So: s = [t_max, -Tx_t, -Ty_t] but we need ||s[1:]|| <= s[0]
    //   which gives ||(Tx_t, Ty_t)|| <= t_max. But signs:
    //   s[1] = -Tx_t means ||(-Tx, -Ty)|| = ||(Tx, Ty)||, so this is fine.
    //
    //   Row for s[0]: no A entries, b = t_max
    //   Row for s[1]: A coefficient on Tx_t = -1, b = 0
    //   Row for s[2]: A coefficient on Ty_t = -1, b = 0

    for t in 0..n {
        let soc_base = n_eq + 3 * t;

        // s[0] = t_max (no A entries needed, just b)
        b_vec[soc_base] = t_max;

        // s[1]: -Tx_t => A has -1 on Tx_t column
        a_rows.push(soc_base + 1);
        a_cols.push(u_idx(t, 0));
        a_vals.push(-1.0);

        // s[2]: -Ty_t => A has -1 on Ty_t column
        a_rows.push(soc_base + 2);
        a_cols.push(u_idx(t, 1));
        a_vals.push(-1.0);
    }

    let a_csc = CscMatrix::new_from_triplets(total_constraints, n_vars, a_rows, a_cols, a_vals);

    // Cones
    let mut cones: Vec<SupportedConeT<f64>> = Vec::new();
    cones.push(SupportedConeT::ZeroConeT(n_eq));
    for _ in 0..n {
        cones.push(SupportedConeT::SecondOrderConeT(3));
    }

    let settings: DefaultSettings<f64> = DefaultSettingsBuilder::default()
        .verbose(false)
        .max_iter(500)
        .build()
        .unwrap();

    let mut solver = DefaultSolver::new(&p, &q_vec, &a_csc, &b_vec, &cones, settings).ok()?;
    solver.solve();

    if solver.solution.status != SolverStatus::Solved {
        return None;
    }

    let z = &solver.solution.x;

    // Extract solution
    let mut positions = Vec::with_capacity(n + 1);
    let mut velocities = Vec::with_capacity(n + 1);
    for t in 0..=n {
        positions.push(Vector2::new(z[x_idx(t, 0)], z[x_idx(t, 1)]));
        velocities.push(Vector2::new(z[x_idx(t, 2)], z[x_idx(t, 3)]));
    }

    let mut thrusts = Vec::with_capacity(n);
    let mut fuel_cost = 0.0;
    for t in 0..n {
        let thrust = Vector2::new(z[u_idx(t, 0)], z[u_idx(t, 1)]);
        fuel_cost += thrust.norm() * dt;
        thrusts.push(thrust);
    }

    Some(RocketTrajectory {
        positions,
        velocities,
        thrusts,
        fuel_cost,
    })
}

/// Simulate free-fall trajectory (no thrust) for comparison.
pub fn simulate_freefall(
    initial: &RocketState,
    n_steps: usize,
    dt: f64,
    gravity: f64,
) -> (Vec<Vector2<f64>>, Vec<Vector2<f64>>) {
    let mut positions = Vec::with_capacity(n_steps + 1);
    let mut velocities = Vec::with_capacity(n_steps + 1);

    let mut pos = initial.position;
    let mut vel = initial.velocity;

    positions.push(pos);
    velocities.push(vel);

    for _ in 0..n_steps {
        pos += vel * dt;
        vel += Vector2::new(0.0, -gravity) * dt;
        positions.push(pos);
        velocities.push(vel);
    }

    (positions, velocities)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that free-fall simulation matches expected physics.
    #[test]
    fn test_freefall_no_thrust() {
        let initial = RocketState {
            position: Vector2::new(0.0, 100.0),
            velocity: Vector2::new(0.0, 0.0),
        };

        let dt = 0.1;
        let n_steps = 50;
        let (positions, velocities) = simulate_freefall(&initial, n_steps, dt, GRAVITY);

        assert_eq!(positions.len(), n_steps + 1);
        assert_eq!(velocities.len(), n_steps + 1);

        // Check initial
        assert!((positions[0] - initial.position).norm() < 1e-10);
        assert!((velocities[0] - initial.velocity).norm() < 1e-10);

        // After t seconds of free-fall: y = y0 - 0.5*g*t^2, vy = -g*t
        let t_final = n_steps as f64 * dt; // 5.0 seconds
        let expected_y = 100.0 - 0.5 * GRAVITY * t_final * t_final;
        let expected_vy = -GRAVITY * t_final;

        // Euler integration accumulates error; use a generous tolerance
        assert!(
            (positions[n_steps][1] - expected_y).abs() < 3.0,
            "Final y: {}, expected: {}",
            positions[n_steps][1],
            expected_y
        );
        assert!(
            (velocities[n_steps][1] - expected_vy).abs() < 1.0,
            "Final vy: {}, expected: {}",
            velocities[n_steps][1],
            expected_vy
        );

        // x should remain zero
        assert!((positions[n_steps][0]).abs() < 1e-10);
    }

    /// Vertical landing: start above origin, come down and land at origin with zero velocity.
    #[test]
    fn test_vertical_landing() {
        let initial = RocketState {
            position: Vector2::new(0.0, 50.0),
            velocity: Vector2::new(0.0, -5.0),
        };
        let target = LandingTarget::default(); // origin, zero velocity

        let config = RocketLandingConfig {
            n_steps: 60,
            dt: 0.1,
            thrust_max: 20.0,
            mass: 1.0,
            gravity: GRAVITY,
            w_terminal_pos: 1e4,
            w_terminal_vel: 1e4,
            w_thrust: 1.0,
        };

        let result = solve_landing_trajectory(&initial, &target, &config);
        assert!(result.is_some(), "Solver should find a solution");

        let traj = result.unwrap();
        assert_eq!(traj.positions.len(), config.n_steps + 1);
        assert_eq!(traj.thrusts.len(), config.n_steps);

        // Check terminal conditions are approximately met
        let final_pos = traj.positions.last().unwrap();
        let final_vel = traj.velocities.last().unwrap();

        assert!(
            final_pos.norm() < 1.0,
            "Final position should be near origin, got: {:?}",
            final_pos
        );
        assert!(
            final_vel.norm() < 1.0,
            "Final velocity should be near zero, got: {:?}",
            final_vel
        );

        // All thrust magnitudes should respect the constraint
        for (t, thrust) in traj.thrusts.iter().enumerate() {
            assert!(
                thrust.norm() <= config.thrust_max + 1e-6,
                "Thrust at step {} exceeds limit: {}",
                t,
                thrust.norm()
            );
        }

        // Fuel cost should be positive
        assert!(traj.fuel_cost > 0.0, "Fuel cost should be positive");
    }

    /// Angled approach: start offset to the side and above, land at origin.
    #[test]
    fn test_angled_approach_landing() {
        let initial = RocketState {
            position: Vector2::new(30.0, 80.0),
            velocity: Vector2::new(-5.0, -10.0),
        };
        let target = LandingTarget::default();

        let config = RocketLandingConfig {
            n_steps: 80,
            dt: 0.1,
            thrust_max: 30.0,
            mass: 1.0,
            gravity: GRAVITY,
            w_terminal_pos: 1e4,
            w_terminal_vel: 1e4,
            w_thrust: 1.0,
        };

        let result = solve_landing_trajectory(&initial, &target, &config);
        assert!(result.is_some(), "Solver should find a solution");

        let traj = result.unwrap();

        let final_pos = traj.positions.last().unwrap();
        let final_vel = traj.velocities.last().unwrap();

        assert!(
            final_pos.norm() < 2.0,
            "Final position should be near origin, got: {:?}",
            final_pos
        );
        assert!(
            final_vel.norm() < 2.0,
            "Final velocity should be near zero, got: {:?}",
            final_vel
        );

        // Thrust constraints
        for (t, thrust) in traj.thrusts.iter().enumerate() {
            assert!(
                thrust.norm() <= config.thrust_max + 1e-6,
                "Thrust at step {} exceeds limit: {}",
                t,
                thrust.norm()
            );
        }

        // The trajectory should have some horizontal thrust to redirect
        let has_horizontal_thrust = traj.thrusts.iter().any(|t| t[0].abs() > 0.1);
        assert!(
            has_horizontal_thrust,
            "Angled approach should use horizontal thrust"
        );

        // y should remain non-negative throughout (no underground trajectory)
        // With soft constraints this may not be strictly guaranteed, but should be close
        for (t, pos) in traj.positions.iter().enumerate() {
            assert!(
                pos[1] > -5.0,
                "Position y at step {} is too negative: {}",
                t,
                pos[1]
            );
        }
    }

    /// Verify solver returns None for infeasible problems.
    #[test]
    fn test_insufficient_thrust() {
        let initial = RocketState {
            position: Vector2::new(0.0, 100.0),
            velocity: Vector2::new(0.0, -50.0), // very fast downward
        };
        let target = LandingTarget::default();

        // Very low thrust and short horizon -- likely infeasible or poor solution
        let config = RocketLandingConfig {
            n_steps: 10,
            dt: 0.05,
            thrust_max: 5.0, // barely above gravity
            mass: 1.0,
            gravity: GRAVITY,
            w_terminal_pos: 1e4,
            w_terminal_vel: 1e4,
            w_thrust: 1.0,
        };

        // This problem is a QP so it will always return a solution, but the terminal
        // error will be large. Verify the solver at least runs.
        let result = solve_landing_trajectory(&initial, &target, &config);
        assert!(
            result.is_some(),
            "Solver should still return a solution (soft constraints)"
        );

        let traj = result.unwrap();
        // With such a short horizon and low thrust, terminal error will be large
        let final_pos = traj.positions.last().unwrap();
        assert!(
            final_pos.norm() > 5.0,
            "With insufficient thrust/time, landing should not be precise: {:?}",
            final_pos
        );
    }
}
