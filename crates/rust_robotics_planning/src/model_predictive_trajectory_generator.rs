#![allow(clippy::too_many_arguments)]

//! Model Predictive Trajectory Generator
//!
//! Generates smooth trajectories using numerical optimization (Newton's method)
//! to connect an initial state to a target state. The trajectory is parameterized
//! by arc length `s`, mid-point curvature `km`, and final curvature `kf`. A
//! quadratic curvature profile interpolated from `(k0, km, kf)` is integrated
//! via a bicycle kinematic model to produce the path. The Jacobian is computed
//! numerically and used to iteratively refine the parameters until the terminal
//! state error falls below a threshold.
//!
//! Reference: <https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/ModelPredictiveTrajectoryGenerator>

use nalgebra::{Matrix3, Vector3};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Parameters for the trajectory generator.
#[derive(Debug, Clone)]
pub struct MptgConfig {
    /// Wheel base length \[m\]
    pub wheel_base: f64,
    /// Step distance for trajectory discretization \[m\]
    pub ds: f64,
    /// Constant forward velocity used in the motion model \[m/s\]
    pub velocity: f64,
    /// Finite-difference step sizes for numerical Jacobian `(h_s, h_km, h_kf)`
    pub h: Vector3<f64>,
    /// Maximum number of optimization iterations
    pub max_iter: usize,
    /// Cost (terminal error norm) convergence threshold
    pub cost_th: f64,
}

impl Default for MptgConfig {
    fn default() -> Self {
        Self {
            wheel_base: 1.0,
            ds: 0.1,
            velocity: 10.0 / 3.6,
            h: Vector3::new(0.5, 0.02, 0.02),
            max_iter: 100,
            cost_th: 0.1,
        }
    }
}

// ---------------------------------------------------------------------------
// Target state
// ---------------------------------------------------------------------------

/// A 2-D pose used as the optimization target.
#[derive(Debug, Clone, Copy)]
pub struct TargetState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl TargetState {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of a successful trajectory optimization.
#[derive(Debug, Clone)]
pub struct MptgResult {
    /// X coordinates along the trajectory
    pub x: Vec<f64>,
    /// Y coordinates along the trajectory
    pub y: Vec<f64>,
    /// Yaw angles along the trajectory \[rad\]
    pub yaw: Vec<f64>,
    /// Optimized parameters `(s, km, kf)`
    pub params: Vector3<f64>,
}

// ---------------------------------------------------------------------------
// Internal bicycle-model state
// ---------------------------------------------------------------------------

struct BicycleState {
    x: f64,
    y: f64,
    yaw: f64,
}

impl BicycleState {
    fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        }
    }

    fn update(&mut self, v: f64, delta: f64, dt: f64, wheel_base: f64) {
        self.x += v * self.yaw.cos() * dt;
        self.y += v * self.yaw.sin() * dt;
        self.yaw += v / wheel_base * delta.tan() * dt;
        self.yaw = pi2pi(self.yaw);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn pi2pi(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

/// Fit a quadratic `a*t^2 + b*t + c` through three `(t, k)` points.
fn quad_interp(t: (f64, f64, f64), k: (f64, f64, f64)) -> (f64, f64, f64) {
    let mat = Matrix3::new(
        t.0 * t.0,
        t.0,
        1.0,
        t.1 * t.1,
        t.1,
        1.0,
        t.2 * t.2,
        t.2,
        1.0,
    );
    let rhs = Vector3::new(k.0, k.1, k.2);
    let coef = mat.try_inverse().expect("quad_interp: singular matrix") * rhs;
    (coef[0], coef[1], coef[2])
}

/// Evaluate the curvature profile `k(t) = a*t^2 + b*t + c`.
#[inline]
fn eval_curvature(coef: (f64, f64, f64), t: f64) -> f64 {
    coef.0 * t * t + coef.1 * t + coef.2
}

// ---------------------------------------------------------------------------
// Trajectory generation (forward simulation)
// ---------------------------------------------------------------------------

/// Generate a full trajectory given parameters `(s, km, kf)` and initial
/// curvature `k0`.
fn generate_trajectory(
    s: f64,
    km: f64,
    kf: f64,
    k0: f64,
    cfg: &MptgConfig,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = (s / cfg.ds).round().max(1.0) as usize;
    let time = s / cfg.velocity;
    let dt = time / n as f64;

    let coef = quad_interp((0.0, time / 2.0, time), (k0, km, kf));

    let mut state = BicycleState::new();
    let mut xs = vec![state.x];
    let mut ys = vec![state.y];
    let mut yaws = vec![state.yaw];

    for i in 0..n {
        let t = i as f64 * dt;
        let delta = eval_curvature(coef, t);
        state.update(cfg.velocity, delta, dt, cfg.wheel_base);
        xs.push(state.x);
        ys.push(state.y);
        yaws.push(state.yaw);
    }

    (xs, ys, yaws)
}

/// Generate only the last state (used in Jacobian computation for efficiency).
fn generate_last_state(s: f64, km: f64, kf: f64, k0: f64, cfg: &MptgConfig) -> (f64, f64, f64) {
    let n = (s / cfg.ds).round().max(1.0) as usize;
    let time = s / cfg.velocity;
    let dt = time / n as f64;

    let coef = quad_interp((0.0, time / 2.0, time), (k0, km, kf));

    let mut state = BicycleState::new();
    for i in 0..n {
        let t = i as f64 * dt;
        let delta = eval_curvature(coef, t);
        state.update(cfg.velocity, delta, dt, cfg.wheel_base);
    }

    (state.x, state.y, state.yaw)
}

// ---------------------------------------------------------------------------
// Optimization internals
// ---------------------------------------------------------------------------

/// Terminal state error.
fn calc_diff(target: &TargetState, x: f64, y: f64, yaw: f64) -> Vector3<f64> {
    Vector3::new(target.x - x, target.y - y, pi2pi(target.yaw - yaw))
}

/// Numerical Jacobian via central differences.
fn calc_jacobian(
    target: &TargetState,
    p: &Vector3<f64>,
    k0: f64,
    cfg: &MptgConfig,
) -> Matrix3<f64> {
    let h = &cfg.h;
    let mut cols: [Vector3<f64>; 3] = [Vector3::zeros(); 3];

    for dim in 0..3 {
        let mut pp = *p;
        let mut pn = *p;
        pp[dim] += h[dim];
        pn[dim] -= h[dim];

        let (xp, yp, yawp) = generate_last_state(pp[0], pp[1], pp[2], k0, cfg);
        let dp = calc_diff(target, xp, yp, yawp);

        let (xn, yn, yawn) = generate_last_state(pn[0], pn[1], pn[2], k0, cfg);
        let dn = calc_diff(target, xn, yn, yawn);

        cols[dim] = (dp - dn) / (2.0 * h[dim]);
    }

    Matrix3::from_columns(&cols)
}

/// Line-search to pick a learning rate that minimizes cost.
fn select_learning_rate(
    dp: &Vector3<f64>,
    p: &Vector3<f64>,
    k0: f64,
    target: &TargetState,
    cfg: &MptgConfig,
) -> f64 {
    let mut best_alpha = 1.0;
    let mut min_cost = f64::MAX;

    let mut alpha = 1.0;
    while alpha < 2.0 {
        let tp = p + alpha * dp;
        let (xc, yc, yawc) = generate_last_state(tp[0], tp[1], tp[2], k0, cfg);
        let dc = calc_diff(target, xc, yc, yawc);
        let cost = dc.norm();
        if cost < min_cost {
            best_alpha = alpha;
            min_cost = cost;
        }
        alpha += 0.5;
    }

    best_alpha
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Run the trajectory optimization.
///
/// Given a `target` pose, an initial curvature `k0`, and an initial parameter
/// guess `init_p = (s, km, kf)`, iteratively refine the parameters using
/// Newton's method until the terminal-state error is below the configured
/// threshold.
///
/// Returns `Some(MptgResult)` on success, or `None` if the optimization fails
/// to converge or encounters a singular Jacobian.
pub fn optimize_trajectory(
    target: &TargetState,
    k0: f64,
    init_p: Vector3<f64>,
    cfg: &MptgConfig,
) -> Option<MptgResult> {
    let mut p = init_p;

    for _ in 0..cfg.max_iter {
        let (xc, yc, yawc) = generate_trajectory(p[0], p[1], p[2], k0, cfg);

        let last_x = *xc.last().unwrap();
        let last_y = *yc.last().unwrap();
        let last_yaw = *yawc.last().unwrap();

        let dc = calc_diff(target, last_x, last_y, last_yaw);
        let cost = dc.norm();

        if cost <= cfg.cost_th {
            return Some(MptgResult {
                x: xc,
                y: yc,
                yaw: yawc,
                params: p,
            });
        }

        let j = calc_jacobian(target, &p, k0, cfg);
        let j_inv = j.try_inverse()?;
        let dp = -j_inv * dc;

        let alpha = select_learning_rate(&dp, &p, k0, target, cfg);
        p += alpha * dp;
    }

    // Did not converge within max_iter
    None
}

// ---------------------------------------------------------------------------
// Lookup table
// ---------------------------------------------------------------------------

/// A single entry in the lookup table: `(x, y, yaw, s, km, kf)`.
#[derive(Debug, Clone, Copy)]
pub struct LookupEntry {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub s: f64,
    pub km: f64,
    pub kf: f64,
}

/// Search the lookup table for the entry closest (in Euclidean + yaw sense)
/// to the query `(tx, ty, tyaw)`.
pub fn search_nearest_in_lookup_table(
    tx: f64,
    ty: f64,
    tyaw: f64,
    table: &[LookupEntry],
) -> Option<&LookupEntry> {
    table.iter().min_by(|a, b| {
        let da = (tx - a.x).powi(2) + (ty - a.y).powi(2) + (tyaw - a.yaw).powi(2);
        let db = (tx - b.x).powi(2) + (ty - b.y).powi(2) + (tyaw - b.yaw).powi(2);
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Generate a lookup table of pre-computed trajectory parameters.
///
/// For each combination of `(x, y, yaw)` in the given ranges, the optimizer
/// is run to find `(s, km, kf)`. The nearest existing entry is used as the
/// initial guess for each new target, bootstrapping convergence.
pub fn generate_lookup_table(
    x_range: &[f64],
    y_range: &[f64],
    yaw_range: &[f64],
    k0: f64,
    cfg: &MptgConfig,
) -> Vec<LookupEntry> {
    let mut table = vec![LookupEntry {
        x: 1.0,
        y: 0.0,
        yaw: 0.0,
        s: 1.0,
        km: 0.0,
        kf: 0.0,
    }];

    for &yaw in yaw_range {
        for &y in y_range {
            for &x in x_range {
                let best = search_nearest_in_lookup_table(x, y, yaw, &table).unwrap();
                let target = TargetState::new(x, y, yaw);
                let s_init = (x * x + y * y).sqrt();
                let init_p = Vector3::new(s_init, best.km, best.kf);

                if let Some(result) = optimize_trajectory(&target, k0, init_p, cfg) {
                    let last_x = *result.x.last().unwrap();
                    let last_y = *result.y.last().unwrap();
                    let last_yaw = *result.yaw.last().unwrap();
                    table.push(LookupEntry {
                        x: last_x,
                        y: last_y,
                        yaw: last_yaw,
                        s: result.params[0],
                        km: result.params[1],
                        kf: result.params[2],
                    });
                }
            }
        }
    }

    table
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_cfg() -> MptgConfig {
        MptgConfig::default()
    }

    #[test]
    fn test_pi2pi() {
        assert!((pi2pi(3.0 * PI) - PI).abs() < 1e-10);
        assert!((pi2pi(-3.0 * PI) - (-PI)).abs() < 1e-10);
        assert!((pi2pi(0.5) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_quad_interp_linear() {
        // f(t) = 2t + 1 => a=0, b=2, c=1
        let (a, b, c) = quad_interp((0.0, 1.0, 2.0), (1.0, 3.0, 5.0));
        assert!(a.abs() < 1e-10);
        assert!((b - 2.0).abs() < 1e-10);
        assert!((c - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_generate_trajectory_straight() {
        let cfg = default_cfg();
        // Zero curvature throughout => straight line
        let (xs, ys, yaws) = generate_trajectory(5.0, 0.0, 0.0, 0.0, &cfg);
        assert!(xs.len() > 2);
        // Final x should be approximately s=5.0
        let last_x = *xs.last().unwrap();
        assert!((last_x - 5.0).abs() < 0.5, "Expected ~5.0, got {last_x}");
        // y should stay near zero
        let last_y = ys.last().unwrap().abs();
        assert!(last_y < 0.1, "Expected ~0.0, got {last_y}");
        // yaw should stay near zero
        let last_yaw = yaws.last().unwrap().abs();
        assert!(last_yaw < 0.1, "Expected ~0.0, got {last_yaw}");
    }

    #[test]
    fn test_generate_last_state_matches_trajectory() {
        let cfg = default_cfg();
        let (xs, ys, yaws) = generate_trajectory(6.0, 0.1, -0.05, 0.0, &cfg);
        let (lx, ly, lyaw) = generate_last_state(6.0, 0.1, -0.05, 0.0, &cfg);
        assert!((xs.last().unwrap() - lx).abs() < 1e-10, "x mismatch");
        assert!((ys.last().unwrap() - ly).abs() < 1e-10, "y mismatch");
        assert!((yaws.last().unwrap() - lyaw).abs() < 1e-10, "yaw mismatch");
    }

    #[test]
    fn test_optimize_trajectory_90deg() {
        let cfg = default_cfg();
        let target = TargetState::new(5.0, 2.0, PI / 2.0);
        let k0 = 0.0;
        let init_p = Vector3::new(6.0, 0.0, 0.0);

        let result = optimize_trajectory(&target, k0, init_p, &cfg);
        assert!(result.is_some(), "Optimization should converge");

        let res = result.unwrap();
        let last_x = *res.x.last().unwrap();
        let last_y = *res.y.last().unwrap();
        let last_yaw = *res.yaw.last().unwrap();

        assert!(
            (last_x - target.x).abs() < cfg.cost_th,
            "x error too large: {last_x} vs {}",
            target.x
        );
        assert!(
            (last_y - target.y).abs() < cfg.cost_th,
            "y error too large: {last_y} vs {}",
            target.y
        );
        assert!(
            pi2pi(last_yaw - target.yaw).abs() < cfg.cost_th,
            "yaw error too large"
        );
    }

    #[test]
    fn test_optimize_trajectory_straight_ahead() {
        let cfg = default_cfg();
        let target = TargetState::new(10.0, 0.0, 0.0);
        let init_p = Vector3::new(10.0, 0.0, 0.0);

        let result = optimize_trajectory(&target, 0.0, init_p, &cfg);
        assert!(result.is_some(), "Straight-ahead should converge");
    }

    #[test]
    fn test_optimize_trajectory_negative_yaw() {
        let cfg = default_cfg();
        let target = TargetState::new(5.0, -2.0, -PI / 4.0);
        let init_p = Vector3::new(6.0, 0.0, 0.0);

        let result = optimize_trajectory(&target, 0.0, init_p, &cfg);
        assert!(result.is_some(), "Negative yaw target should converge");
    }

    #[test]
    fn test_lookup_table_generation() {
        let cfg = MptgConfig {
            max_iter: 100,
            cost_th: 0.3,
            ..Default::default()
        };

        let x_range: Vec<f64> = vec![10.0, 15.0];
        let y_range: Vec<f64> = vec![0.0, 5.0];
        let yaw_range: Vec<f64> = vec![0.0];

        let table = generate_lookup_table(&x_range, &y_range, &yaw_range, 0.0, &cfg);

        // Should have the seed entry plus some solved entries
        assert!(
            table.len() > 1,
            "Lookup table should contain more than just the seed"
        );
    }

    #[test]
    fn test_search_nearest_in_lookup_table() {
        let table = vec![
            LookupEntry {
                x: 1.0,
                y: 0.0,
                yaw: 0.0,
                s: 1.0,
                km: 0.0,
                kf: 0.0,
            },
            LookupEntry {
                x: 10.0,
                y: 5.0,
                yaw: 0.5,
                s: 11.0,
                km: 0.1,
                kf: 0.05,
            },
        ];
        let nearest = search_nearest_in_lookup_table(9.0, 4.0, 0.4, &table).unwrap();
        assert!((nearest.x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_search_nearest_empty_table() {
        let table: Vec<LookupEntry> = vec![];
        assert!(search_nearest_in_lookup_table(1.0, 0.0, 0.0, &table).is_none());
    }

    #[test]
    fn test_config_default() {
        let cfg = MptgConfig::default();
        assert!((cfg.wheel_base - 1.0).abs() < 1e-10);
        assert!((cfg.ds - 0.1).abs() < 1e-10);
        assert!(cfg.max_iter == 100);
    }
}
