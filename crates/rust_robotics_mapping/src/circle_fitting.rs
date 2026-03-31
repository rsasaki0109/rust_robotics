//! Circle fitting using a least-squares approach.
//!
//! Fits a circle (center + radius) to a set of 2D points by solving
//! a 3x3 linear system derived from minimising the algebraic distance.
//!
//! Reference: PythonRobotics circle_fitting by Atsushi Sakai.

use nalgebra::{Matrix3, Vector3};

/// Result of a circle fit.
#[derive(Debug, Clone, PartialEq)]
pub struct CircleFitResult {
    /// X coordinate of the fitted circle centre.
    pub cx: f64,
    /// Y coordinate of the fitted circle centre.
    pub cy: f64,
    /// Radius of the fitted circle.
    pub radius: f64,
    /// Sum of signed residuals (distance to circle boundary).
    pub error: f64,
}

/// Fit a circle to a set of 2D points using the least-squares method.
///
/// The algorithm builds the normal equations from the point sums and solves
/// for the circle parameters `(cx, cy, r)` that minimise the algebraic
/// distance.
///
/// # Panics
///
/// Panics if `x` and `y` have different lengths or contain fewer than 3
/// points (a circle cannot be uniquely determined with fewer than 3 points).
pub fn circle_fitting(x: &[f64], y: &[f64]) -> CircleFitResult {
    assert_eq!(x.len(), y.len(), "x and y must have the same length");
    assert!(
        x.len() >= 3,
        "at least 3 points are required for circle fitting"
    );

    let n = x.len() as f64;

    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_x2: f64 = x.iter().map(|v| v * v).sum();
    let sum_y2: f64 = y.iter().map(|v| v * v).sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();

    // F * T = G  =>  T = F^{-1} * G
    let f = Matrix3::new(
        sum_x2, sum_xy, sum_x, sum_xy, sum_y2, sum_y, sum_x, sum_y, n,
    );

    let g = Vector3::new(
        -x.iter()
            .zip(y.iter())
            .map(|(xi, yi)| xi.powi(3) + xi * yi.powi(2))
            .sum::<f64>(),
        -x.iter()
            .zip(y.iter())
            .map(|(xi, yi)| xi.powi(2) * yi + yi.powi(3))
            .sum::<f64>(),
        -x.iter()
            .zip(y.iter())
            .map(|(xi, yi)| xi.powi(2) + yi.powi(2))
            .sum::<f64>(),
    );

    let f_inv = f
        .try_inverse()
        .expect("singular matrix – points may be collinear");
    let t = f_inv * g;

    let cx = t[0] / -2.0;
    let cy = t[1] / -2.0;
    let radius = (cx * cx + cy * cy - t[2]).sqrt();

    let error: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| ((cx - xi).powi(2) + (cy - yi).powi(2)).sqrt() - radius)
        .sum();

    CircleFitResult {
        cx,
        cy,
        radius,
        error,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Generate points on a circle with optional noise.
    fn sample_circle(cx: f64, cy: f64, r: f64, n: usize) -> (Vec<f64>, Vec<f64>) {
        let mut xs = Vec::with_capacity(n);
        let mut ys = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * i as f64 / n as f64;
            xs.push(cx + r * theta.cos());
            ys.push(cy + r * theta.sin());
        }
        (xs, ys)
    }

    #[test]
    fn test_exact_circle() {
        let (x, y) = sample_circle(3.0, -2.0, 5.0, 36);
        let res = circle_fitting(&x, &y);
        assert!((res.cx - 3.0).abs() < 1e-10, "cx mismatch: {}", res.cx);
        assert!((res.cy - (-2.0)).abs() < 1e-10, "cy mismatch: {}", res.cy);
        assert!(
            (res.radius - 5.0).abs() < 1e-10,
            "radius mismatch: {}",
            res.radius
        );
        assert!(res.error.abs() < 1e-10, "error should be ~0: {}", res.error);
    }

    #[test]
    fn test_unit_circle_at_origin() {
        let (x, y) = sample_circle(0.0, 0.0, 1.0, 100);
        let res = circle_fitting(&x, &y);
        assert!((res.cx).abs() < 1e-10);
        assert!((res.cy).abs() < 1e-10);
        assert!((res.radius - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_small_circle() {
        let (x, y) = sample_circle(-1.0, 2.0, 0.5, 20);
        let res = circle_fitting(&x, &y);
        assert!((res.cx - (-1.0)).abs() < 1e-9);
        assert!((res.cy - 2.0).abs() < 1e-9);
        assert!((res.radius - 0.5).abs() < 1e-9);
    }

    #[test]
    #[should_panic(expected = "at least 3 points")]
    fn test_too_few_points() {
        circle_fitting(&[1.0, 2.0], &[3.0, 4.0]);
    }

    #[test]
    #[should_panic(expected = "same length")]
    fn test_mismatched_lengths() {
        circle_fitting(&[1.0, 2.0, 3.0], &[4.0, 5.0]);
    }
}
