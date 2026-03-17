#![allow(dead_code, clippy::needless_range_loop, clippy::type_complexity)]

//! Bezier curve path computation (basic functions)
//!
//! Provides fundamental Bezier curve computation including
//! Bernstein polynomials, control point manipulation, and curvature.

pub fn calc_4points_bezier_path(
    x_start: (f64, f64, f64),
    x_goal: (f64, f64, f64),
    offset: f64,
) -> (Vec<(f64, f64)>, [(f64, f64); 4]) {
    let x_diff = (x_start.0 - x_goal.0, x_start.1 - x_goal.1);
    let dist = ((x_diff.0).powi(2) + (x_diff.1).powi(2)).sqrt() / offset;
    let control_points = [
        (x_start.0, x_start.1),
        (
            x_start.0 + dist * (x_start.2).cos(),
            x_start.1 + dist * (x_start.2).sin(),
        ),
        (
            x_goal.0 - dist * (x_goal.2).cos(),
            x_goal.1 - dist * (x_goal.2).sin(),
        ),
        (x_goal.0, x_goal.1),
    ];
    let path = calc_bezier_path(control_points, 100);
    (path, control_points)
}

pub fn calc_bezier_path(control_points: [(f64, f64); 4], n_points: usize) -> Vec<(f64, f64)> {
    let mut traj: Vec<(f64, f64)> = Vec::with_capacity(n_points);
    for i in 0..n_points - 1 {
        let t = (i as f64) / ((n_points as f64) - 1.);
        traj.push(bezier(t, control_points));
    }
    traj
}

pub fn bernstein_poly(n: i32, i: i32, t: f64) -> f64 {
    (binom(n, i) as f64) * t.powi(i) * (1. - t).powi(n - i)
}

pub fn bezier(t: f64, control_points: [(f64, f64); 4]) -> (f64, f64) {
    let n = control_points.len() - 1;
    let mut point = (0., 0.);
    for i in 0..n + 1 {
        let ber_poly = bernstein_poly(n as i32, i as i32, t);
        point.0 += ber_poly * control_points[i].0;
        point.1 += ber_poly * control_points[i].1;
    }
    point
}

/// Ref: Donald E. Knuth, "The Art of Computer Programming (2)"
pub fn binom(n: i32, k: i32) -> i32 {
    (0..n + 1).rev().zip(1..k + 1).fold(1, |mut r, (n, d)| {
        r *= n;
        r /= d;
        r
    })
}

pub fn bezier_derivatives_control_points(
    control_points: [(f64, f64); 4],
    n_derivatives: usize,
) -> Vec<[(f64, f64); 4]> {
    let mut w = vec![control_points];
    let mut tmp: [(f64, f64); 4] = Default::default();
    let n = w.len();
    for i in 0..n_derivatives - 1 {
        for j in 0..n - 1 {
            tmp[j].0 = ((n - 1) as f64) * (w[i][j + 1].0 - w[i][j].0);
            tmp[j].1 = ((n - 1) as f64) * (w[i][j + 1].1 - w[i][j].1);
        }
        w.push(tmp);
    }
    w
}

pub fn curvature(dx: f64, dy: f64, ddx: f64, ddy: f64) -> f64 {
    (dx * ddy - dy * ddx) / (dx.powi(2) + dy.powi(2)).powf(3. / 2.)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier_path() {
        let start = (10., 1., std::f64::consts::PI);
        let end = (0., -3., -45.0 / 180.0 * std::f64::consts::PI);
        let (path, cp) = calc_4points_bezier_path(start, end, 3.0);
        assert!(!path.is_empty());
        assert_eq!(cp.len(), 4);
    }

    #[test]
    fn test_binom() {
        assert_eq!(binom(4, 2), 6);
        assert_eq!(binom(3, 0), 1);
        assert_eq!(binom(3, 3), 1);
    }
}
