//! Clothoid (Euler Spiral) Path Planner
//!
//! A clothoid is a curve whose curvature changes linearly with arc length.
//! It is the industry standard for road geometry design in highway engineering
//! and autonomous driving, providing smooth curvature transitions.
//!
//! The clothoid is parametrized by:
//!   `kappa(s) = kappa0 + kappa1 * s`
//!
//! where `kappa0` is the initial curvature, `kappa1` is the curvature rate
//! (sharpness), and `s` is the arc length.
//!
//! The position along the curve is obtained by integrating:
//!   `x(s) = integral cos(theta(t)) dt`
//!   `y(s) = integral sin(theta(t)) dt`
//!
//! where `theta(s) = theta0 + kappa0 * s + 0.5 * kappa1 * s^2`.
//!
//! # References
//!
//! * Bertolazzi, E. & Frego, M. (2015). "G1 fitting with clothoids".
//!   *Mathematical Methods in the Applied Sciences*, 38(5), 881-897.

use std::f64::consts::PI;

use rust_robotics_core::error::{RoboticsError, RoboticsResult};
use rust_robotics_core::types::{Path2D, Pose2D};

// --- Fresnel integrals -------------------------------------------------------

/// Evaluate the Fresnel integrals C(t) and S(t) defined as:
///   C(t) = integral_0^t cos(pi/2 * u^2) du
///   S(t) = integral_0^t sin(pi/2 * u^2) du
///
/// Uses a power-series expansion, which converges well for moderate |t|.
pub fn fresnel(t: f64) -> (f64, f64) {
    let t_abs = t.abs();

    if t_abs < 1e-15 {
        return (0.0, 0.0);
    }

    // For large arguments, use asymptotic approximation
    if t_abs > 6.0 {
        let (c, s) = fresnel_asymptotic(t_abs);
        if t < 0.0 {
            return (-c, -s);
        }
        return (c, s);
    }

    // Power-series expansion
    let (c, s) = fresnel_series(t_abs);
    if t < 0.0 {
        (-c, -s)
    } else {
        (c, s)
    }
}

/// Fresnel integrals via power-series expansion.
/// Good convergence for |t| <= ~6.
fn fresnel_series(t: f64) -> (f64, f64) {
    let half_pi = PI / 2.0;
    let x = half_pi * t * t;

    let mut c_sum = 0.0;
    let mut s_sum = 0.0;
    let mut term = t; // first term of C series: t
    let mut sign = 1.0_f64;

    // C(t) = sum_{k=0}^{inf} (-1)^k * (pi/2)^{2k} * t^{4k+1} / ((4k+1) * (2k)!)
    // S(t) = sum_{k=0}^{inf} (-1)^k * (pi/2)^{2k+1} * t^{4k+3} / ((4k+3) * (2k+1)!)
    for k in 0..25 {
        let c_term = sign * term / (4 * k + 1) as f64;
        c_sum += c_term;

        let s_term = sign * term * x / (4 * k + 3) as f64 / (2 * k + 1) as f64;
        s_sum += s_term;

        if c_term.abs() < 1e-16 && s_term.abs() < 1e-16 {
            break;
        }

        // Update term: multiply by (pi/2)^2 * t^4 / ((2k+1)*(2k+2))
        term *= x * x / ((2 * k + 1) as f64 * (2 * k + 2) as f64);
        sign = -sign;
    }

    (c_sum, s_sum)
}

/// Asymptotic expansion for large arguments.
fn fresnel_asymptotic(t: f64) -> (f64, f64) {
    let x = PI * t * t;
    let sin_x = (x / 2.0).sin();
    let cos_x = (x / 2.0).cos();
    let inv_x = 1.0 / (PI * t);

    // f(t) ~ 1/(pi*t) * (1 - 3/(pi*t)^4 + ...)
    // g(t) ~ 1/(pi*t)^2 * (1 - 15/(pi*t)^4 + ...)
    let pi_t2 = (PI * t).powi(2);
    let f = inv_x * (1.0 - 3.0 / (pi_t2 * pi_t2));
    let g = inv_x / (PI * t) * (1.0 - 15.0 / (pi_t2 * pi_t2));

    let c = 0.5 + f * sin_x - g * cos_x;
    let s = 0.5 - f * cos_x - g * sin_x;
    (c, s)
}

// --- Clothoid primitives -----------------------------------------------------

/// Evaluate a clothoid curve at arc length `s`, starting from the origin
/// with heading `theta0`, initial curvature `kappa0`, and sharpness `kappa1`.
///
/// Returns `(x, y, theta)` at arc length `s`.
///
/// Integration is performed via Simpson's rule for numerical stability.
pub fn clothoid_eval(theta0: f64, kappa0: f64, kappa1: f64, s: f64) -> (f64, f64, f64) {
    // Integrate position by numerical quadrature (Simpson's rule)
    let n = 100; // number of intervals (must be even)
    let ds = s / n as f64;
    let mut x = 0.0;
    let mut y = 0.0;

    for i in 0..=n {
        let si = ds * i as f64;
        let theta = theta0 + kappa0 * si + 0.5 * kappa1 * si * si;
        let weight = if i == 0 || i == n {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        x += weight * theta.cos();
        y += weight * theta.sin();
    }
    x *= ds / 3.0;
    y *= ds / 3.0;

    let theta_end = theta0 + kappa0 * s + 0.5 * kappa1 * s * s;
    (x, y, theta_end)
}

/// Sample `n_points` along a clothoid starting at pose `(x0, y0, theta0)`,
/// with initial curvature `kappa0`, sharpness `kappa1`, and total arc length `length`.
///
/// Returns `(xs, ys, thetas, kappas)` vectors at each sample point.
pub fn clothoid_sample(
    x0: f64,
    y0: f64,
    theta0: f64,
    kappa0: f64,
    kappa1: f64,
    length: f64,
    n_points: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut xs = Vec::with_capacity(n_points);
    let mut ys = Vec::with_capacity(n_points);
    let mut thetas = Vec::with_capacity(n_points);
    let mut kappas = Vec::with_capacity(n_points);

    let cos0 = theta0.cos();
    let sin0 = theta0.sin();

    for i in 0..n_points {
        let s = if n_points > 1 {
            length * i as f64 / (n_points - 1) as f64
        } else {
            0.0
        };

        let (dx, dy, theta) = clothoid_eval(0.0, kappa0, kappa1, s);
        // Rotate by theta0 and translate
        let x = x0 + cos0 * dx - sin0 * dy;
        let y = y0 + sin0 * dx + cos0 * dy;
        let kappa = kappa0 + kappa1 * s;

        xs.push(x);
        ys.push(y);
        thetas.push(theta0 + theta);
        kappas.push(kappa);
    }

    (xs, ys, thetas, kappas)
}

// --- G1 Hermite interpolation (clothoid fitting) ----------------------------

/// Normalize angle to [-pi, pi].
fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

/// Solve the G1 Hermite interpolation problem: find a clothoid connecting
/// two poses `(x0, y0, theta0)` and `(x1, y1, theta1)`.
///
/// Returns `(kappa0, kappa1, length)` -- initial curvature, sharpness, arc length.
///
/// Uses an iterative Newton-Raphson method on the clothoid parameters.
pub fn clothoid_g1_fit(start: &Pose2D, goal: &Pose2D) -> RoboticsResult<(f64, f64, f64)> {
    let dx = goal.x - start.x;
    let dy = goal.y - start.y;
    let chord = (dx * dx + dy * dy).sqrt();

    if chord < 1e-12 {
        return Err(RoboticsError::InvalidParameter(
            "start and goal poses are coincident".to_string(),
        ));
    }

    let phi = dy.atan2(dx);
    let alpha = normalize_angle(start.yaw - phi);
    let beta = normalize_angle(goal.yaw - phi);

    // Initial guess for the length
    let mut length = chord;

    // Newton iteration to find kappa0, kappa1, and length.
    // In the rotated frame the clothoid starts at (0,0) with heading alpha
    // and must reach (chord, 0) with heading beta.

    let max_iter = 50;
    let tol = 1e-12;

    let mut k0 = (alpha + beta) / length;
    let mut k1 = 2.0 * (beta - alpha) / (length * length);

    for _iter in 0..max_iter {
        // Evaluate clothoid at current parameters
        let (ex, ey, _etheta) = clothoid_eval(alpha, k0, k1, length);

        // Residuals: endpoint should be at (chord, 0) with heading beta
        let rx = ex - chord;
        let ry = ey;
        let r_theta = alpha + k0 * length + 0.5 * k1 * length * length - beta;

        if rx.abs() < tol && ry.abs() < tol && r_theta.abs() < tol {
            break;
        }

        // Compute Jacobian numerically
        let eps = 1e-8;

        let (ex_dk0, ey_dk0, _) = clothoid_eval(alpha, k0 + eps, k1, length);
        let dex_dk0 = (ex_dk0 - ex) / eps;
        let dey_dk0 = (ey_dk0 - ey) / eps;

        let (ex_dk1, ey_dk1, _) = clothoid_eval(alpha, k0, k1 + eps, length);
        let dex_dk1 = (ex_dk1 - ex) / eps;
        let dey_dk1 = (ey_dk1 - ey) / eps;

        let (ex_dl, ey_dl, _) = clothoid_eval(alpha, k0, k1, length + eps);
        let dex_dl = (ex_dl - ex) / eps;
        let dey_dl = (ey_dl - ey) / eps;

        // 3x3 Jacobian for (rx, ry, r_theta) w.r.t. (k0, k1, length)
        let dtheta_dk0 = length;
        let dtheta_dk1 = 0.5 * length * length;
        let dtheta_dl = k0 + k1 * length;

        // Solve 3x3 system using nalgebra
        let jac = nalgebra::Matrix3::new(
            dex_dk0, dex_dk1, dex_dl, dey_dk0, dey_dk1, dey_dl, dtheta_dk0, dtheta_dk1, dtheta_dl,
        );

        let residual = nalgebra::Vector3::new(rx, ry, r_theta);

        let Some(delta) = jac.try_inverse().map(|inv| inv * residual) else {
            return Err(RoboticsError::NumericalError(
                "Jacobian is singular during clothoid fitting".to_string(),
            ));
        };

        k0 -= delta[0];
        k1 -= delta[1];
        length -= delta[2];

        // Keep length positive
        if length < 1e-10 {
            length = chord * 0.5;
        }
    }

    // Verify solution
    let (ex, ey, _) = clothoid_eval(alpha, k0, k1, length);
    let err = ((ex - chord).powi(2) + ey.powi(2)).sqrt();
    if err > 1e-6 {
        return Err(RoboticsError::NumericalError(format!(
            "clothoid G1 fitting did not converge (residual={err:.6e})"
        )));
    }

    Ok((k0, k1, length))
}

// --- ClothoidPath struct -----------------------------------------------------

/// A clothoid (Euler spiral) path connecting two poses.
#[derive(Debug, Clone)]
pub struct ClothoidPath {
    /// Start pose
    pub start: Pose2D,
    /// End pose
    pub goal: Pose2D,
    /// Initial curvature
    pub kappa0: f64,
    /// Curvature rate (sharpness)
    pub kappa1: f64,
    /// Total arc length
    pub length: f64,
    /// Sampled path points
    pub path: Path2D,
    /// Heading at each sample point
    pub yaw: Vec<f64>,
    /// Curvature at each sample point
    pub curvature: Vec<f64>,
}

/// Configuration for the clothoid path planner.
#[derive(Debug, Clone)]
pub struct ClothoidConfig {
    /// Number of sample points along the path
    pub n_points: usize,
}

impl Default for ClothoidConfig {
    fn default() -> Self {
        Self { n_points: 100 }
    }
}

/// Clothoid path planner.
///
/// Computes a clothoid curve (Euler spiral) that connects two poses with
/// G1 continuity (matching position and heading at both endpoints).
#[derive(Debug, Clone)]
pub struct ClothoidPlanner {
    pub config: ClothoidConfig,
}

impl ClothoidPlanner {
    pub fn new(config: ClothoidConfig) -> Self {
        Self { config }
    }

    /// Plan a clothoid path from `start` to `goal`.
    pub fn plan(&self, start: &Pose2D, goal: &Pose2D) -> RoboticsResult<ClothoidPath> {
        let (kappa0, kappa1, length) = clothoid_g1_fit(start, goal)?;

        let (xs, ys, yaw, curvature) = clothoid_sample(
            start.x,
            start.y,
            start.yaw,
            kappa0,
            kappa1,
            length,
            self.config.n_points,
        );

        let path = Path2D::from_xy(&xs, &ys);

        Ok(ClothoidPath {
            start: *start,
            goal: *goal,
            kappa0,
            kappa1,
            length,
            path,
            yaw,
            curvature,
        })
    }

    /// Plan a clothoid path from raw coordinates.
    pub fn plan_from_xy(
        &self,
        x0: f64,
        y0: f64,
        yaw0: f64,
        x1: f64,
        y1: f64,
        yaw1: f64,
    ) -> RoboticsResult<ClothoidPath> {
        self.plan(&Pose2D::new(x0, y0, yaw0), &Pose2D::new(x1, y1, yaw1))
    }
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const TOL: f64 = 1e-6;

    fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
        assert!(
            (actual - expected).abs() < tol,
            "{label}: actual={actual}, expected={expected}, diff={}",
            (actual - expected).abs()
        );
    }

    // -- Fresnel integral tests -----------------------------------------------

    #[test]
    fn test_fresnel_zero() {
        let (c, s) = fresnel(0.0);
        assert_close(c, 0.0, 1e-15, "C(0)");
        assert_close(s, 0.0, 1e-15, "S(0)");
    }

    #[test]
    fn test_fresnel_known_values() {
        // C(1) ~ 0.7798934, S(1) ~ 0.4382591
        let (c, s) = fresnel(1.0);
        assert_close(c, 0.7798934003768228, 1e-10, "C(1)");
        assert_close(s, 0.4382591473903548, 1e-10, "S(1)");
    }

    #[test]
    fn test_fresnel_odd_symmetry() {
        let (c_pos, s_pos) = fresnel(2.0);
        let (c_neg, s_neg) = fresnel(-2.0);
        assert_close(c_neg, -c_pos, 1e-14, "C(-t) = -C(t)");
        assert_close(s_neg, -s_pos, 1e-14, "S(-t) = -S(t)");
    }

    #[test]
    fn test_fresnel_large_argument_limit() {
        // As t -> inf, C(t) -> 0.5, S(t) -> 0.5
        let (c, s) = fresnel(100.0);
        assert_close(c, 0.5, 0.005, "C(inf)");
        assert_close(s, 0.5, 0.005, "S(inf)");
    }

    // -- Clothoid evaluation tests --------------------------------------------

    #[test]
    fn test_clothoid_eval_straight_line() {
        // Zero curvature, heading = 0 -> straight line along x
        let (x, y, theta) = clothoid_eval(0.0, 0.0, 0.0, 5.0);
        assert_close(x, 5.0, TOL, "x straight");
        assert_close(y, 0.0, TOL, "y straight");
        assert_close(theta, 0.0, TOL, "theta straight");
    }

    #[test]
    fn test_clothoid_eval_circular_arc() {
        // Constant curvature (kappa1 = 0) -> circular arc
        let radius = 10.0;
        let kappa0 = 1.0 / radius;
        let arc_length = FRAC_PI_2 * radius; // quarter circle

        let (x, y, theta) = clothoid_eval(0.0, kappa0, 0.0, arc_length);
        // Quarter circle: endpoint should be at (R, R) approximately
        assert_close(x, radius, 0.01, "x circular arc");
        assert_close(y, radius, 0.01, "y circular arc");
        assert_close(theta, FRAC_PI_2, TOL, "theta circular arc");
    }

    #[test]
    fn test_clothoid_eval_with_sharpness() {
        // Non-zero kappa1: curvature grows linearly
        let (x, y, theta) = clothoid_eval(0.0, 0.0, 0.1, 3.0);
        // theta(3) = 0 + 0*3 + 0.5*0.1*9 = 0.45
        assert_close(theta, 0.45, TOL, "theta with sharpness");
        assert!(x.is_finite(), "x should be finite");
        assert!(y.is_finite(), "y should be finite");
        assert!(y > 0.0, "curve should bend left (positive y)");
    }

    // -- Clothoid sampling tests ----------------------------------------------

    #[test]
    fn test_clothoid_sample_straight() {
        let (xs, ys, thetas, kappas) = clothoid_sample(0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11);

        assert_eq!(xs.len(), 11);
        for i in 0..11 {
            let s = i as f64;
            assert_close(xs[i], s, TOL, &format!("x[{i}]"));
            assert_close(ys[i], 0.0, TOL, &format!("y[{i}]"));
            assert_close(thetas[i], 0.0, TOL, &format!("theta[{i}]"));
            assert_close(kappas[i], 0.0, TOL, &format!("kappa[{i}]"));
        }
    }

    #[test]
    fn test_clothoid_sample_rotated_straight() {
        // Straight line at 45 degrees
        let angle = PI / 4.0;
        let length = 10.0;
        let (xs, ys, _thetas, _kappas) = clothoid_sample(1.0, 2.0, angle, 0.0, 0.0, length, 11);

        let end_x = 1.0 + length * angle.cos();
        let end_y = 2.0 + length * angle.sin();
        assert_close(*xs.last().unwrap(), end_x, TOL, "end x");
        assert_close(*ys.last().unwrap(), end_y, TOL, "end y");
    }

    // -- G1 fitting tests -----------------------------------------------------

    #[test]
    fn test_g1_fit_straight_line() {
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(10.0, 0.0, 0.0);

        let (kappa0, kappa1, length) = clothoid_g1_fit(&start, &goal).unwrap();
        assert_close(kappa0, 0.0, 1e-4, "kappa0 straight");
        assert_close(kappa1, 0.0, 1e-4, "kappa1 straight");
        assert_close(length, 10.0, 0.1, "length straight");
    }

    #[test]
    fn test_g1_fit_simple_curve() {
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(5.0, 3.0, PI / 4.0);

        let result = clothoid_g1_fit(&start, &goal);
        assert!(result.is_ok(), "G1 fit should converge");

        let (kappa0, kappa1, length) = result.unwrap();
        assert!(length > 0.0, "length should be positive");

        // Verify the endpoint matches
        let (xs, ys, thetas, _) =
            clothoid_sample(start.x, start.y, start.yaw, kappa0, kappa1, length, 100);

        let end_x = *xs.last().unwrap();
        let end_y = *ys.last().unwrap();
        let end_theta = *thetas.last().unwrap();

        assert_close(end_x, goal.x, 0.01, "fitted end x");
        assert_close(end_y, goal.y, 0.01, "fitted end y");
        assert_close(
            normalize_angle(end_theta),
            normalize_angle(goal.yaw),
            0.01,
            "fitted end yaw",
        );
    }

    #[test]
    fn test_g1_fit_coincident_points_error() {
        let start = Pose2D::new(1.0, 2.0, 0.0);
        let goal = Pose2D::new(1.0, 2.0, PI / 2.0);

        let result = clothoid_g1_fit(&start, &goal);
        assert!(result.is_err(), "coincident points should fail");
    }

    #[test]
    fn test_g1_fit_symmetry() {
        // Symmetric case: start at (0,0,0), end at (10,0,0)
        // Should produce a straight line with near-zero curvature
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(10.0, 0.0, 0.0);

        let (kappa0, kappa1, length) = clothoid_g1_fit(&start, &goal).unwrap();

        assert_close(kappa0, 0.0, 1e-4, "symmetric kappa0");
        assert_close(kappa1, 0.0, 1e-4, "symmetric kappa1");
        assert!(length > 0.0, "length positive");
    }

    // -- Planner integration tests --------------------------------------------

    #[test]
    fn test_planner_basic() {
        let planner = ClothoidPlanner::new(ClothoidConfig { n_points: 50 });
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(8.0, 4.0, PI / 3.0);

        let result = planner.plan(&start, &goal);
        assert!(result.is_ok(), "planner should succeed");

        let clothoid = result.unwrap();
        assert_eq!(clothoid.path.len(), 50);
        assert_eq!(clothoid.yaw.len(), 50);
        assert_eq!(clothoid.curvature.len(), 50);

        // First point should match start
        assert_close(clothoid.path.points[0].x, start.x, TOL, "start x");
        assert_close(clothoid.path.points[0].y, start.y, TOL, "start y");

        // Curvature should change linearly
        let first_k = clothoid.curvature[0];
        let last_k = *clothoid.curvature.last().unwrap();
        let mid_k = clothoid.curvature[25];
        let expected_mid = (first_k + last_k) / 2.0;
        assert_close(mid_k, expected_mid, 0.01, "linear curvature at midpoint");
    }

    #[test]
    fn test_planner_from_xy() {
        let planner = ClothoidPlanner::new(ClothoidConfig::default());
        let result = planner.plan_from_xy(0.0, 0.0, 0.0, 5.0, 0.0, 0.0);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().path.len(), 100);
    }

    #[test]
    fn test_curvature_linearity() {
        let planner = ClothoidPlanner::new(ClothoidConfig { n_points: 101 });
        let start = Pose2D::new(0.0, 0.0, 0.0);
        let goal = Pose2D::new(6.0, 2.0, 0.5);

        let clothoid = planner.plan(&start, &goal).unwrap();

        // The curvature profile should be linear: kappa(s) = kappa0 + kappa1 * s
        // Check that equally spaced samples form an arithmetic progression
        let n = clothoid.curvature.len();
        let step = (clothoid.curvature[n - 1] - clothoid.curvature[0]) / (n - 1) as f64;
        for i in 0..n {
            let expected = clothoid.curvature[0] + step * i as f64;
            assert_close(
                clothoid.curvature[i],
                expected,
                1e-10,
                &format!("curvature linearity at {i}"),
            );
        }
    }
}
