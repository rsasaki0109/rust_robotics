//! Arc-length cubic spline courses shared by the path-tracking controllers
//! (Stanley, LQR Steer, LQR Speed+Steer, Rear Wheel Feedback).

use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// f64 math via libm on no_std targets; on std hosts the inherent methods win
use num_traits::Float;

/// `(x, y, yaw, curvature, s)` sampled every `ds` along the course.
pub(crate) type SplineCourse = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>);

/// Waypoints closer together than this \[m\] count as one.
const DUPLICATE_DISTANCE: f64 = 1.0e-9;

/// Samples a cubic spline through the waypoints every `ds` of arc length.
///
/// Repeated waypoints are dropped (they would give zero-length spline
/// intervals and NaN everywhere); fewer than two distinct waypoints give an
/// empty course.
pub(crate) fn calc_spline_course(x: &[f64], y: &[f64], ds: f64) -> SplineCourse {
    let mut course: SplineCourse = (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let Some(sp) = CubicSpline2D::new(x, y) else {
        return course;
    };
    let s_max = sp.s[sp.s.len() - 1] - ds;
    let mut s = 0.0;
    while s < s_max {
        let (ix, iy) = sp.calc_position(s);
        course.0.push(ix);
        course.1.push(iy);
        course.2.push(sp.calc_yaw(s));
        course.3.push(sp.calc_curvature(s));
        course.4.push(s);
        s += ds;
    }
    course
}

/// Natural cubic spline `y(x)` over strictly increasing knots `x`.
struct CubicSpline {
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
    x: Vec<f64>,
}

impl CubicSpline {
    /// `x` must be strictly increasing with at least two knots.
    fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        let h: Vec<f64> = x.windows(2).map(|w| w[1] - w[0]).collect();
        let a = y.to_vec();
        let mut b = vec![0.0; n];
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];

        let mut alpha = vec![0.0; n - 1];
        for i in 1..n - 1 {
            alpha[i] = 3.0 * (a[i + 1] - a[i]) / h[i] - 3.0 * (a[i] - a[i - 1]) / h[i - 1];
        }

        let mut l = vec![1.0; n];
        let mut mu = vec![0.0; n];
        let mut z = vec![0.0; n];
        for i in 1..n - 1 {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        for j in (0..n - 1).rev() {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        CubicSpline {
            a,
            b,
            c,
            d,
            x: x.to_vec(),
        }
    }

    /// Interval index and offset for `t`, or `None` outside the knots.
    fn locate(&self, t: f64) -> Option<(usize, f64)> {
        let last = self.x.len() - 1;
        if t < self.x[0] || t > self.x[last] {
            return None;
        }
        let i = (0..last)
            .find(|&i| self.x[i] <= t && t <= self.x[i + 1])
            .unwrap_or(last - 1);
        Some((i, t - self.x[i]))
    }

    fn calc(&self, t: f64) -> f64 {
        match self.locate(t) {
            Some((i, dx)) => {
                self.a[i] + self.b[i] * dx + self.c[i] * dx * dx + self.d[i] * dx * dx * dx
            }
            None if t < self.x[0] => self.a[0],
            None => self.a[self.a.len() - 1],
        }
    }

    fn calc_d(&self, t: f64) -> f64 {
        match self.locate(t) {
            Some((i, dx)) => self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx * dx,
            None if t < self.x[0] => self.b[0],
            None => self.b[self.b.len() - 1],
        }
    }

    fn calc_dd(&self, t: f64) -> f64 {
        match self.locate(t) {
            Some((i, dx)) => 2.0 * self.c[i] + 6.0 * self.d[i] * dx,
            None if t < self.x[0] => 2.0 * self.c[0],
            None => 2.0 * self.c[self.c.len() - 1],
        }
    }
}

/// `(x(s), y(s))` splines over cumulative chord length `s`.
struct CubicSpline2D {
    s: Vec<f64>,
    sx: CubicSpline,
    sy: CubicSpline,
}

impl CubicSpline2D {
    /// `None` with fewer than two distinct waypoints.
    fn new(x: &[f64], y: &[f64]) -> Option<Self> {
        let mut xs: Vec<f64> = Vec::with_capacity(x.len());
        let mut ys: Vec<f64> = Vec::with_capacity(y.len());
        let mut s: Vec<f64> = Vec::with_capacity(x.len());
        for (&px, &py) in x.iter().zip(y) {
            match (xs.last(), ys.last(), s.last()) {
                (Some(&lx), Some(&ly), Some(&ls)) => {
                    let step = (px - lx).hypot(py - ly);
                    if step < DUPLICATE_DISTANCE {
                        continue;
                    }
                    s.push(ls + step);
                }
                _ => s.push(0.0),
            }
            xs.push(px);
            ys.push(py);
        }
        if s.len() < 2 {
            return None;
        }
        let sx = CubicSpline::new(&s, &xs);
        let sy = CubicSpline::new(&s, &ys);
        Some(CubicSpline2D { s, sx, sy })
    }

    fn calc_position(&self, s: f64) -> (f64, f64) {
        (self.sx.calc(s), self.sy.calc(s))
    }

    fn calc_curvature(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let ddx = self.sx.calc_dd(s);
        let dy = self.sy.calc_d(s);
        let ddy = self.sy.calc_dd(s);
        let denom = (dx * dx + dy * dy).powf(1.5);
        if denom.abs() > 1e-6 {
            (ddy * dx - ddx * dy) / denom
        } else {
            0.0
        }
    }

    fn calc_yaw(&self, s: f64) -> f64 {
        let dx = self.sx.calc_d(s);
        let dy = self.sy.calc_d(s);
        dy.atan2(dx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_waypoints_do_not_produce_nan() {
        let x = [0.0, 5.0, 5.0, 10.0, 15.0, 15.0];
        let y = [0.0, 1.0, 1.0, -1.0, 0.0, 0.0];
        let (cx, cy, cyaw, ck, cs) = calc_spline_course(&x, &y, 0.1);
        assert!(cx.len() > 100);
        for v in cx.iter().chain(&cy).chain(&cyaw).chain(&ck).chain(&cs) {
            assert!(v.is_finite());
        }
        // Same course as without the repeats.
        let (dx, dy, ..) = calc_spline_course(&[0.0, 5.0, 10.0, 15.0], &[0.0, 1.0, -1.0, 0.0], 0.1);
        assert_eq!(cx, dx);
        assert_eq!(cy, dy);
    }

    #[test]
    fn fewer_than_two_distinct_waypoints_give_an_empty_course() {
        assert!(calc_spline_course(&[], &[], 0.1).0.is_empty());
        assert!(calc_spline_course(&[1.0], &[2.0], 0.1).0.is_empty());
        assert!(calc_spline_course(&[1.0, 1.0], &[2.0, 2.0], 0.1)
            .0
            .is_empty());
    }

    #[test]
    fn a_straight_line_has_zero_curvature_and_constant_yaw() {
        let (cx, _, cyaw, ck, _) = calc_spline_course(&[0.0, 1.0, 2.0], &[0.0, 1.0, 2.0], 0.05);
        assert!(!cx.is_empty());
        for (yaw, k) in cyaw.iter().zip(&ck) {
            assert!((yaw - core::f64::consts::FRAC_PI_4).abs() < 1e-9);
            assert!(k.abs() < 1e-9);
        }
    }
}
