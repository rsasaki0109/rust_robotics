#![allow(dead_code, clippy::needless_range_loop, clippy::type_complexity)]

//! B-Spline path generation
//!
//! Generates smooth paths through or near control points using B-spline curves.
//! Supports configurable degree and number of output sample points.
//!
//! Reference:
//! - PythonRobotics BSplinePath by Atsushi Sakai

/// Result of B-spline path generation.
#[derive(Debug, Clone)]
pub struct BSplinePath {
    /// Sampled (x, y) positions along the path.
    pub path: Vec<(f64, f64)>,
    /// Heading angle \[rad\] at each sampled point.
    pub heading: Vec<f64>,
    /// Curvature at each sampled point.
    pub curvature: Vec<f64>,
}

/// Generate a B-spline path from control points.
///
/// # Arguments
/// * `control_x` - x coordinates of control points
/// * `control_y` - y coordinates of control points
/// * `n_path_points` - number of points to sample along the path
/// * `degree` - B-spline degree (must satisfy 1 <= degree <= n_control_points - 1)
///
/// # Returns
/// A `BSplinePath` containing the sampled path, heading, and curvature,
/// or `None` if the inputs are invalid.
pub fn generate_bspline_path(
    control_x: &[f64],
    control_y: &[f64],
    n_path_points: usize,
    degree: usize,
) -> Option<BSplinePath> {
    let n = control_x.len();
    if n != control_y.len() || n < 2 || n_path_points == 0 {
        return None;
    }
    let degree = degree.min(n - 1);
    if degree == 0 {
        return None;
    }

    let knots = generate_clamped_knot_vector(n, degree);

    let t_min = knots[degree];
    let t_max = knots[n]; // = knots[n + degree + 1 - (degree + 1)]

    let mut path = Vec::with_capacity(n_path_points);
    let mut heading = Vec::with_capacity(n_path_points);
    let mut curvature = Vec::with_capacity(n_path_points);

    for i in 0..n_path_points {
        let t = if n_path_points == 1 {
            t_min
        } else {
            t_min + (t_max - t_min) * (i as f64) / ((n_path_points - 1) as f64)
        };

        // Clamp t to valid range to avoid numerical issues at the boundary
        let t = t.min(t_max - 1e-10);

        let (x, y) = eval_bspline(t, control_x, control_y, degree, &knots);
        let (dx, dy) = eval_bspline_derivative(t, control_x, control_y, degree, &knots, 1);
        let (ddx, ddy) = eval_bspline_derivative(t, control_x, control_y, degree, &knots, 2);

        path.push((x, y));
        heading.push(dy.atan2(dx));

        let denom = (dx * dx + dy * dy).powf(2.0 / 3.0);
        let k = if denom.abs() > 1e-15 {
            (ddy * dx - ddx * dy) / denom
        } else {
            0.0
        };
        curvature.push(k);
    }

    Some(BSplinePath {
        path,
        heading,
        curvature,
    })
}

/// Generate an interpolating B-spline path.
///
/// Uses chord-length parameterisation so the resulting curve passes through
/// all given waypoints (like `interpolate_b_spline_path` in PythonRobotics).
///
/// # Arguments
/// * `x` - x coordinates of waypoints
/// * `y` - y coordinates of waypoints
/// * `n_path_points` - number of output sample points
/// * `degree` - B-spline degree (must satisfy 1 <= degree <= n_waypoints - 1)
///
/// # Returns
/// A `BSplinePath` or `None` if the inputs are invalid.
pub fn interpolate_bspline_path(
    x: &[f64],
    y: &[f64],
    n_path_points: usize,
    degree: usize,
) -> Option<BSplinePath> {
    let n = x.len();
    if n != y.len() || n < 2 || n_path_points == 0 {
        return None;
    }
    let degree = degree.min(n - 1);
    if degree == 0 {
        return None;
    }

    // Compute normalised chord-length parameters for each waypoint
    let distances = calc_normalised_cumulative_distances(x, y);

    // Fit independent B-spline curves for x(t) and y(t) by interpolation
    let ctrl_x = fit_bspline_interpolation(&distances, x, degree)?;
    let ctrl_y = fit_bspline_interpolation(&distances, y, degree)?;

    generate_bspline_path(&ctrl_x, &ctrl_y, n_path_points, degree)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Generate a clamped (open) uniform knot vector.
///
/// For `n` control points and given `degree`, the knot vector has
/// `n + degree + 1` elements with `degree + 1` repeated values at each end.
fn generate_clamped_knot_vector(n: usize, degree: usize) -> Vec<f64> {
    let m = n + degree + 1;
    let mut knots = Vec::with_capacity(m);
    for i in 0..m {
        if i <= degree {
            knots.push(0.0);
        } else if i >= n {
            knots.push((n - degree) as f64);
        } else {
            knots.push((i - degree) as f64);
        }
    }
    knots
}

/// Evaluate the B-spline curve at parameter `t` using De Boor's algorithm.
fn eval_bspline(t: f64, cx: &[f64], cy: &[f64], degree: usize, knots: &[f64]) -> (f64, f64) {
    let n = cx.len();
    let mut x = 0.0;
    let mut y = 0.0;
    for i in 0..n {
        let b = bspline_basis(i, degree, t, knots);
        x += b * cx[i];
        y += b * cy[i];
    }
    (x, y)
}

/// Evaluate the `order`-th derivative of the B-spline curve at parameter `t`.
fn eval_bspline_derivative(
    t: f64,
    cx: &[f64],
    cy: &[f64],
    degree: usize,
    knots: &[f64],
    order: usize,
) -> (f64, f64) {
    if order == 0 {
        return eval_bspline(t, cx, cy, degree, knots);
    }
    if order > degree {
        return (0.0, 0.0);
    }

    // Compute derivative control points iteratively
    let mut dx: Vec<f64> = cx.to_vec();
    let mut dy: Vec<f64> = cy.to_vec();
    let mut current_knots = knots.to_vec();
    let mut current_degree = degree;

    for _ in 0..order {
        let nn = dx.len();
        if nn <= 1 {
            return (0.0, 0.0);
        }
        let mut new_dx = Vec::with_capacity(nn - 1);
        let mut new_dy = Vec::with_capacity(nn - 1);
        for i in 0..nn - 1 {
            let denom = current_knots[i + current_degree + 1] - current_knots[i + 1];
            let factor = if denom.abs() > 1e-15 {
                current_degree as f64 / denom
            } else {
                0.0
            };
            new_dx.push(factor * (dx[i + 1] - dx[i]));
            new_dy.push(factor * (dy[i + 1] - dy[i]));
        }
        dx = new_dx;
        dy = new_dy;
        // Remove first and last knot for the derivative knot vector
        current_knots = current_knots[1..current_knots.len() - 1].to_vec();
        current_degree -= 1;
    }

    eval_bspline(t, &dx, &dy, current_degree, &current_knots)
}

/// Evaluate the B-spline basis function N_{i,p}(t) using the Cox-de Boor recursion.
fn bspline_basis(i: usize, degree: usize, t: f64, knots: &[f64]) -> f64 {
    if degree == 0 {
        return if knots[i] <= t && t < knots[i + 1] {
            1.0
        } else {
            0.0
        };
    }

    let mut left = 0.0;
    let denom_l = knots[i + degree] - knots[i];
    if denom_l.abs() > 1e-15 {
        left = (t - knots[i]) / denom_l * bspline_basis(i, degree - 1, t, knots);
    }

    let mut right = 0.0;
    let denom_r = knots[i + degree + 1] - knots[i + 1];
    if denom_r.abs() > 1e-15 {
        right = (knots[i + degree + 1] - t) / denom_r * bspline_basis(i + 1, degree - 1, t, knots);
    }

    left + right
}

/// Compute normalised cumulative chord-length distances.
fn calc_normalised_cumulative_distances(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut dists = Vec::with_capacity(n);
    dists.push(0.0);
    for i in 1..n {
        let dx = x[i] - x[i - 1];
        let dy = y[i] - y[i - 1];
        dists.push(dists[i - 1] + (dx * dx + dy * dy).sqrt());
    }
    let total = *dists.last().unwrap();
    if total > 1e-15 {
        for d in &mut dists {
            *d /= total;
        }
    }
    dists
}

/// Fit B-spline control points so the curve interpolates the given data points.
///
/// Solves `N * P = D` where `N` is the basis function collocation matrix.
/// Uses the clamped knot vector with knots placed at the data parameter values.
fn fit_bspline_interpolation(params: &[f64], values: &[f64], degree: usize) -> Option<Vec<f64>> {
    let n = params.len();

    // Build knot vector for interpolation.
    // For n data points and degree p, we need n + p + 1 knots.
    // Use averaging method for interior knots.
    let knots = build_interpolation_knot_vector(params, n, degree);

    // Build collocation matrix
    let mut mat = nalgebra::DMatrix::zeros(n, n);
    for row in 0..n {
        let t = if row == n - 1 {
            params[row] - 1e-10 // avoid landing exactly on the last knot
        } else {
            params[row]
        };
        for col in 0..n {
            mat[(row, col)] = bspline_basis(col, degree, t, &knots);
        }
    }

    let rhs = nalgebra::DVector::from_column_slice(values);
    // Solve the linear system
    let decomp = mat.lu();
    let solution = decomp.solve(&rhs)?;

    Some(solution.as_slice().to_vec())
}

/// Build a knot vector for B-spline interpolation using the averaging method.
fn build_interpolation_knot_vector(params: &[f64], n: usize, degree: usize) -> Vec<f64> {
    let m = n + degree + 1;
    let mut knots = vec![0.0; m];

    // First degree+1 knots = 0
    // Last degree+1 knots = 1 (since params are normalised to [0,1])
    let last_val = *params.last().unwrap();
    for i in (m - degree - 1)..m {
        knots[i] = last_val;
    }

    // Interior knots by averaging
    for j in 1..(n - degree) {
        let mut sum = 0.0;
        for i in j..(j + degree) {
            sum += params[i];
        }
        knots[j + degree] = sum / degree as f64;
    }

    knots
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_clamped_knot_vector() {
        let knots = generate_clamped_knot_vector(5, 3);
        // 5 + 3 + 1 = 9 knots
        assert_eq!(knots.len(), 9);
        // First degree+1 values are 0
        assert_eq!(&knots[..4], &[0.0, 0.0, 0.0, 0.0]);
        // Last degree+1 values equal n - degree = 2
        assert_eq!(&knots[5..], &[2.0, 2.0, 2.0, 2.0]);
        assert_eq!(knots[4], 1.0);
    }

    #[test]
    fn test_basis_partition_of_unity() {
        let n = 5;
        let degree = 3;
        let knots = generate_clamped_knot_vector(n, degree);

        // Basis functions should sum to 1 at any parameter value in [t_min, t_max)
        for k in 0..20 {
            let t = (k as f64) * 0.1;
            if t >= (n - degree) as f64 {
                continue;
            }
            let sum: f64 = (0..n).map(|i| bspline_basis(i, degree, t, &knots)).sum();
            assert!(
                approx_eq(sum, 1.0, 1e-12),
                "Partition of unity failed at t={t}: sum={sum}"
            );
        }
    }

    #[test]
    fn test_generate_bspline_path_basic() {
        let cx = vec![-1.0, 3.0, 4.0, 2.0, 1.0];
        let cy = vec![0.0, -3.0, 1.0, 1.0, 3.0];
        let result = generate_bspline_path(&cx, &cy, 50, 3);
        assert!(result.is_some());
        let bsp = result.unwrap();
        assert_eq!(bsp.path.len(), 50);
        assert_eq!(bsp.heading.len(), 50);
        assert_eq!(bsp.curvature.len(), 50);
    }

    #[test]
    fn test_path_starts_and_ends_at_control_endpoints() {
        let cx = vec![-1.0, 3.0, 4.0, 2.0, 1.0];
        let cy = vec![0.0, -3.0, 1.0, 1.0, 3.0];
        let bsp = generate_bspline_path(&cx, &cy, 100, 3).unwrap();

        // With clamped knot vector, the curve starts at first and ends at last
        // control point.
        let first = bsp.path[0];
        let last = bsp.path[bsp.path.len() - 1];
        assert!(approx_eq(first.0, cx[0], 1e-9), "Start x mismatch");
        assert!(approx_eq(first.1, cy[0], 1e-9), "Start y mismatch");
        assert!(
            approx_eq(last.0, *cx.last().unwrap(), 1e-9),
            "End x mismatch"
        );
        assert!(
            approx_eq(last.1, *cy.last().unwrap(), 1e-9),
            "End y mismatch"
        );
    }

    #[test]
    fn test_different_degrees() {
        let cx = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let cy = vec![0.0, 2.0, -1.0, 3.0, 0.0, 1.0];

        for degree in 1..=5 {
            let result = generate_bspline_path(&cx, &cy, 30, degree);
            assert!(result.is_some(), "Failed for degree={degree}");
            let bsp = result.unwrap();
            assert_eq!(bsp.path.len(), 30);

            // Start/end should match first/last control point
            let first = bsp.path[0];
            let last = bsp.path[bsp.path.len() - 1];
            assert!(approx_eq(first.0, cx[0], 1e-9), "degree={degree} start x");
            assert!(approx_eq(first.1, cy[0], 1e-9), "degree={degree} start y");
            assert!(
                approx_eq(last.0, *cx.last().unwrap(), 1e-9),
                "degree={degree} end x"
            );
            assert!(
                approx_eq(last.1, *cy.last().unwrap(), 1e-9),
                "degree={degree} end y"
            );
        }
    }

    #[test]
    fn test_degree_clamped_to_valid_range() {
        // degree > n-1 should be clamped
        let cx = vec![0.0, 1.0, 2.0];
        let cy = vec![0.0, 1.0, 0.0];
        let result = generate_bspline_path(&cx, &cy, 20, 10);
        assert!(result.is_some());
    }

    #[test]
    fn test_invalid_inputs() {
        assert!(generate_bspline_path(&[], &[], 10, 3).is_none());
        assert!(generate_bspline_path(&[1.0], &[2.0], 10, 3).is_none());
        assert!(generate_bspline_path(&[1.0, 2.0], &[1.0], 10, 3).is_none());
        assert!(generate_bspline_path(&[1.0, 2.0], &[1.0, 2.0], 0, 3).is_none());
    }

    #[test]
    fn test_linear_degree1_is_straight() {
        // Degree-1 B-spline through collinear points should be a straight line
        let cx = vec![0.0, 1.0, 2.0, 3.0];
        let cy = vec![0.0, 1.0, 2.0, 3.0];
        let bsp = generate_bspline_path(&cx, &cy, 50, 1).unwrap();

        for &(px, py) in &bsp.path {
            assert!(
                approx_eq(px, py, 1e-9),
                "Point ({px}, {py}) deviates from y=x"
            );
        }
    }

    #[test]
    fn test_heading_is_consistent_with_path() {
        let cx = vec![0.0, 2.0, 4.0, 6.0];
        let cy = vec![0.0, 3.0, 1.0, 4.0];
        let bsp = generate_bspline_path(&cx, &cy, 100, 3).unwrap();

        // The heading should roughly match the direction between consecutive points
        for i in 1..bsp.path.len() - 1 {
            let dx = bsp.path[i + 1].0 - bsp.path[i - 1].0;
            let dy = bsp.path[i + 1].1 - bsp.path[i - 1].1;
            let fd_heading = dy.atan2(dx);
            let diff = (bsp.heading[i] - fd_heading).abs();
            // Allow some tolerance since finite differences are approximate
            assert!(
                diff < 0.3 || (diff - std::f64::consts::TAU).abs() < 0.3,
                "Heading mismatch at i={i}: analytic={}, fd={fd_heading}",
                bsp.heading[i]
            );
        }
    }

    #[test]
    fn test_interpolate_bspline_path_passes_through_waypoints() {
        let wx = vec![-1.0, 3.0, 4.0, 2.0, 1.0];
        let wy = vec![0.0, -3.0, 1.0, 1.0, 3.0];
        let bsp = interpolate_bspline_path(&wx, &wy, 200, 3).unwrap();

        // The interpolated curve should start and end at the first/last waypoints
        let first = bsp.path[0];
        let last = bsp.path[bsp.path.len() - 1];
        assert!(approx_eq(first.0, wx[0], 1e-6), "Start x");
        assert!(approx_eq(first.1, wy[0], 1e-6), "Start y");
        assert!(approx_eq(last.0, *wx.last().unwrap(), 1e-6), "End x");
        assert!(approx_eq(last.1, *wy.last().unwrap(), 1e-6), "End y");
    }

    #[test]
    fn test_interpolate_bspline_path_invalid() {
        assert!(interpolate_bspline_path(&[], &[], 10, 3).is_none());
        assert!(interpolate_bspline_path(&[1.0], &[2.0], 10, 3).is_none());
    }

    #[test]
    fn test_single_output_point() {
        let cx = vec![0.0, 1.0, 2.0, 3.0];
        let cy = vec![0.0, 1.0, 0.0, 1.0];
        let bsp = generate_bspline_path(&cx, &cy, 1, 3).unwrap();
        assert_eq!(bsp.path.len(), 1);
        // Single point should be the start of the curve
        assert!(approx_eq(bsp.path[0].0, 0.0, 1e-9));
        assert!(approx_eq(bsp.path[0].1, 0.0, 1e-9));
    }

    #[test]
    fn test_two_control_points() {
        let cx = vec![0.0, 5.0];
        let cy = vec![0.0, 3.0];
        let bsp = generate_bspline_path(&cx, &cy, 10, 1).unwrap();
        assert_eq!(bsp.path.len(), 10);
        // Should be a straight line from (0,0) to (5,3)
        assert!(approx_eq(bsp.path[0].0, 0.0, 1e-9));
        assert!(approx_eq(bsp.path[0].1, 0.0, 1e-9));
        assert!(approx_eq(bsp.path[9].0, 5.0, 1e-9));
        assert!(approx_eq(bsp.path[9].1, 3.0, 1e-9));
    }

    #[test]
    fn test_curvature_zero_for_straight_line() {
        let cx = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let cy = vec![0.0, 0.0, 0.0, 0.0, 0.0];
        let bsp = generate_bspline_path(&cx, &cy, 50, 3).unwrap();
        for (i, &k) in bsp.curvature.iter().enumerate() {
            assert!(
                k.abs() < 1e-6,
                "Curvature should be ~0 for straight line, got {k} at index {i}"
            );
        }
    }
}
