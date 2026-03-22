#![allow(dead_code, clippy::too_many_arguments, clippy::type_complexity)]

//! Bezier curve path planning
//!
//! Path planner using Bezier curves with curvature computation.

/// Binomial coefficient calculation
fn binomial_coefficient(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    if k == 0 || k == n {
        return 1.0;
    }

    let k = if k > n - k { n - k } else { k };

    let mut result = 1.0;
    for i in 0..k {
        result *= (n - i) as f64;
        result /= (i + 1) as f64;
    }
    result
}

/// Bernstein polynomial
fn bernstein_poly(n: usize, i: usize, t: f64) -> f64 {
    binomial_coefficient(n, i) * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
}

/// Return one point on the bezier curve
fn bezier(t: f64, control_points: &[(f64, f64)]) -> (f64, f64) {
    let n = control_points.len() - 1;
    let mut x = 0.0;
    let mut y = 0.0;

    for (i, &(px, py)) in control_points.iter().enumerate() {
        let basis = bernstein_poly(n, i, t);
        x += basis * px;
        y += basis * py;
    }

    (x, y)
}

/// Compute bezier path (trajectory) given control points
pub fn calc_bezier_path(control_points: &[(f64, f64)], n_points: usize) -> Vec<(f64, f64)> {
    let mut trajectory = Vec::new();

    for i in 0..n_points {
        let t = i as f64 / (n_points - 1) as f64;
        trajectory.push(bezier(t, control_points));
    }

    trajectory
}

/// Compute control points and path given start and end position
pub fn calc_4points_bezier_path(
    sx: f64,
    sy: f64,
    syaw: f64,
    ex: f64,
    ey: f64,
    eyaw: f64,
    offset: f64,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    let dist = ((sx - ex).powi(2) + (sy - ey).powi(2)).sqrt() / offset;

    let control_points = vec![
        (sx, sy),
        (sx + dist * syaw.cos(), sy + dist * syaw.sin()),
        (ex - dist * eyaw.cos(), ey - dist * eyaw.sin()),
        (ex, ey),
    ];

    let path = calc_bezier_path(&control_points, 100);

    (path, control_points)
}

/// Compute control points of the successive derivatives of a given bezier curve
pub fn bezier_derivatives_control_points(
    control_points: &[(f64, f64)],
    n_derivatives: usize,
) -> Vec<Vec<(f64, f64)>> {
    let mut derivatives = vec![control_points.to_vec()];

    for i in 0..n_derivatives {
        let current = &derivatives[i];
        let n = current.len();

        if n <= 1 {
            break;
        }

        let mut next_derivative = Vec::new();
        for j in 0..n - 1 {
            let dx = (n - 1) as f64 * (current[j + 1].0 - current[j].0);
            let dy = (n - 1) as f64 * (current[j + 1].1 - current[j].1);
            next_derivative.push((dx, dy));
        }
        derivatives.push(next_derivative);
    }

    derivatives
}

/// Calculate curvature at parameter t
pub fn calc_curvature(control_points: &[(f64, f64)], t: f64) -> f64 {
    let derivatives = bezier_derivatives_control_points(control_points, 2);

    if derivatives.len() < 3 {
        return 0.0;
    }

    let first_deriv = bezier(t, &derivatives[1]);
    let second_deriv = bezier(t, &derivatives[2]);

    let dx = first_deriv.0;
    let dy = first_deriv.1;
    let ddx = second_deriv.0;
    let ddy = second_deriv.1;

    let numerator = dx * ddy - dy * ddx;
    let denominator = (dx * dx + dy * dy).powf(1.5);

    if denominator.abs() < 1e-10 {
        0.0
    } else {
        numerator / denominator
    }
}

pub struct BezierPathPlanner {
    pub path: Vec<(f64, f64)>,
    pub control_points: Vec<(f64, f64)>,
    pub curvature: Vec<f64>,
}

impl BezierPathPlanner {
    pub fn new() -> Self {
        BezierPathPlanner {
            path: Vec::new(),
            control_points: Vec::new(),
            curvature: Vec::new(),
        }
    }

    pub fn planning(
        &mut self,
        sx: f64,
        sy: f64,
        syaw: f64,
        ex: f64,
        ey: f64,
        eyaw: f64,
        offset: f64,
    ) -> bool {
        let (path, control_points) = calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset);

        let mut curvature = Vec::new();
        for i in 0..path.len() {
            let t = i as f64 / (path.len() - 1) as f64;
            curvature.push(calc_curvature(&control_points, t));
        }

        self.path = path;
        self.control_points = control_points;
        self.curvature = curvature;

        true
    }

    pub fn planning_with_control_points(
        &mut self,
        control_points: Vec<(f64, f64)>,
        n_points: usize,
    ) -> bool {
        if control_points.len() < 2 {
            return false;
        }

        let path = calc_bezier_path(&control_points, n_points);

        let mut curvature = Vec::new();
        for i in 0..path.len() {
            let t = i as f64 / (path.len() - 1) as f64;
            curvature.push(calc_curvature(&control_points, t));
        }

        self.path = path;
        self.control_points = control_points;
        self.curvature = curvature;

        true
    }
}

impl Default for BezierPathPlanner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier_path_planning() {
        let mut planner = BezierPathPlanner::new();
        let result = planner.planning(
            10.0,
            1.0,
            180.0_f64.to_radians(),
            0.0,
            -3.0,
            45.0_f64.to_radians(),
            3.0,
        );
        assert!(result);
        assert!(!planner.path.is_empty());
        assert!(!planner.curvature.is_empty());
    }

    #[test]
    fn test_bezier_with_control_points() {
        let mut planner = BezierPathPlanner::new();
        let cp = vec![(-1.0, 0.0), (3.0, -3.0), (4.0, 1.0), (2.0, 1.0), (1.0, 3.0)];
        let result = planner.planning_with_control_points(cp, 100);
        assert!(result);
        assert!(!planner.path.is_empty());
    }
}
