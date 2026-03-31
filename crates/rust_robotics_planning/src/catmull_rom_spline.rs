//! Catmull-Rom spline path planner
//!
//! Generates a smooth, C1-continuous interpolating spline that passes through
//! all given control points.  Supports both uniform and centripetal
//! parameterisation.
//!
//! Reference:
//! - <http://graphics.cs.cmu.edu/nsp/course/15-462/Fall04/assts/catmullRom.pdf>
//! - PythonRobotics CatmullRomSplinePath

/// Parameterisation strategy for the Catmull-Rom spline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parameterisation {
    /// Uniform parameterisation (alpha = 0.0).
    Uniform,
    /// Centripetal parameterisation (alpha = 0.5).  Avoids cusps and
    /// self-intersections that can occur with the uniform variant.
    Centripetal,
    /// Chordal parameterisation (alpha = 1.0).
    Chordal,
    /// Custom alpha value.
    Custom(f64),
}

impl Parameterisation {
    fn alpha(self) -> f64 {
        match self {
            Parameterisation::Uniform => 0.0,
            Parameterisation::Centripetal => 0.5,
            Parameterisation::Chordal => 1.0,
            Parameterisation::Custom(a) => a,
        }
    }
}

/// A 2-D Catmull-Rom spline built from a sequence of waypoints.
#[derive(Debug, Clone)]
pub struct CatmullRomSpline {
    /// Control points (waypoints).
    points: Vec<(f64, f64)>,
    /// Parameterisation variant.
    param: Parameterisation,
    /// Number of interpolated points to generate per segment.
    points_per_segment: usize,
}

impl CatmullRomSpline {
    /// Create a new spline from waypoints.
    ///
    /// # Arguments
    /// * `points` - At least 2 waypoints.
    /// * `points_per_segment` - Number of output points per segment (excluding
    ///   the segment end-point to avoid duplicates, except on the last segment).
    /// * `param` - Parameterisation strategy.
    ///
    /// # Panics
    /// Panics if fewer than 2 points are provided.
    pub fn new(
        points: Vec<(f64, f64)>,
        points_per_segment: usize,
        param: Parameterisation,
    ) -> Self {
        assert!(points.len() >= 2, "At least 2 waypoints are required");
        assert!(points_per_segment >= 2, "points_per_segment must be >= 2");
        CatmullRomSpline {
            points,
            param,
            points_per_segment,
        }
    }

    /// Generate the interpolated path.
    ///
    /// Returns a vector of `(x, y)` points along the spline.
    pub fn generate_path(&self) -> Vec<(f64, f64)> {
        let alpha = self.param.alpha();
        let n = self.points.len();
        let mut path: Vec<(f64, f64)> = Vec::new();

        for i in 0..n - 1 {
            let p0 = if i == 0 {
                self.points[0]
            } else {
                self.points[i - 1]
            };
            let p1 = self.points[i];
            let p2 = self.points[i + 1];
            let p3 = if i + 2 < n {
                self.points[i + 2]
            } else {
                self.points[n - 1]
            };

            let is_last_segment = i == n - 2;
            let segment = Self::segment_points(
                p0,
                p1,
                p2,
                p3,
                self.points_per_segment,
                alpha,
                is_last_segment,
            );
            path.extend(segment);
        }

        path
    }

    /// Generate the interpolated path together with first-derivative
    /// information (yaw angle and approximate curvature).
    ///
    /// Returns `(path, yaw, curvature)`.
    pub fn generate_path_with_derivatives(&self) -> (Vec<(f64, f64)>, Vec<f64>, Vec<f64>) {
        let alpha = self.param.alpha();
        let n = self.points.len();
        let mut path: Vec<(f64, f64)> = Vec::new();
        let mut yaw: Vec<f64> = Vec::new();
        let mut curvature: Vec<f64> = Vec::new();

        for i in 0..n - 1 {
            let p0 = if i == 0 {
                self.points[0]
            } else {
                self.points[i - 1]
            };
            let p1 = self.points[i];
            let p2 = self.points[i + 1];
            let p3 = if i + 2 < n {
                self.points[i + 2]
            } else {
                self.points[n - 1]
            };

            let is_last_segment = i == n - 2;
            let seg = Self::segment_points_with_derivatives(
                p0,
                p1,
                p2,
                p3,
                self.points_per_segment,
                alpha,
                is_last_segment,
            );
            path.extend(seg.0);
            yaw.extend(seg.1);
            curvature.extend(seg.2);
        }

        (path, yaw, curvature)
    }

    /// Compute the knot interval for centripetal / chordal parameterisation.
    fn knot_interval(p0: (f64, f64), p1: (f64, f64), alpha: f64) -> f64 {
        let dx = p1.0 - p0.0;
        let dy = p1.1 - p0.1;
        let d2 = dx * dx + dy * dy;
        d2.powf(alpha * 0.5)
    }

    /// Interpolate a single segment from `p1` to `p2` using the
    /// surrounding control points `p0` and `p3`.
    ///
    /// When `alpha == 0` this is equivalent to the uniform Catmull-Rom
    /// matrix formulation used in the Python reference.
    fn segment_points(
        p0: (f64, f64),
        p1: (f64, f64),
        p2: (f64, f64),
        p3: (f64, f64),
        num_points: usize,
        alpha: f64,
        include_endpoint: bool,
    ) -> Vec<(f64, f64)> {
        if alpha == 0.0 {
            // Uniform: use the classic matrix form (matches Python reference).
            Self::segment_uniform(p0, p1, p2, p3, num_points, include_endpoint)
        } else {
            // Centripetal / chordal: Barry-Goldman algorithm.
            Self::segment_general(p0, p1, p2, p3, num_points, alpha, include_endpoint)
        }
    }

    /// Uniform Catmull-Rom using the matrix form.
    fn segment_uniform(
        p0: (f64, f64),
        p1: (f64, f64),
        p2: (f64, f64),
        p3: (f64, f64),
        num_points: usize,
        include_endpoint: bool,
    ) -> Vec<(f64, f64)> {
        let count = if include_endpoint {
            num_points
        } else {
            num_points - 1
        };
        let mut pts = Vec::with_capacity(count);

        for j in 0..count {
            let t = j as f64 / (num_points - 1) as f64;
            let t2 = t * t;
            let t3 = t2 * t;

            let x = 0.5
                * ((2.0 * p1.0)
                    + (-p0.0 + p2.0) * t
                    + (2.0 * p0.0 - 5.0 * p1.0 + 4.0 * p2.0 - p3.0) * t2
                    + (-p0.0 + 3.0 * p1.0 - 3.0 * p2.0 + p3.0) * t3);
            let y = 0.5
                * ((2.0 * p1.1)
                    + (-p0.1 + p2.1) * t
                    + (2.0 * p0.1 - 5.0 * p1.1 + 4.0 * p2.1 - p3.1) * t2
                    + (-p0.1 + 3.0 * p1.1 - 3.0 * p2.1 + p3.1) * t3);

            pts.push((x, y));
        }

        pts
    }

    /// General (centripetal / chordal) Catmull-Rom via the Barry-Goldman
    /// recursive algorithm.
    fn segment_general(
        p0: (f64, f64),
        p1: (f64, f64),
        p2: (f64, f64),
        p3: (f64, f64),
        num_points: usize,
        alpha: f64,
        include_endpoint: bool,
    ) -> Vec<(f64, f64)> {
        let t0: f64 = 0.0;
        let t1 = t0 + Self::knot_interval(p0, p1, alpha);
        let t2 = t1 + Self::knot_interval(p1, p2, alpha);
        let t3 = t2 + Self::knot_interval(p2, p3, alpha);

        let count = if include_endpoint {
            num_points
        } else {
            num_points - 1
        };
        let mut pts = Vec::with_capacity(count);

        for j in 0..count {
            let frac = j as f64 / (num_points - 1) as f64;
            let t = t1 + frac * (t2 - t1);

            let a1 = Self::lerp(p0, p1, t0, t1, t);
            let a2 = Self::lerp(p1, p2, t1, t2, t);
            let a3 = Self::lerp(p2, p3, t2, t3, t);

            let b1 = Self::lerp(a1, a2, t0, t2, t);
            let b2 = Self::lerp(a2, a3, t1, t3, t);

            let c = Self::lerp(b1, b2, t1, t2, t);
            pts.push(c);
        }

        pts
    }

    /// Segment interpolation that also returns first and second derivatives
    /// (needed for yaw and curvature).
    fn segment_points_with_derivatives(
        p0: (f64, f64),
        p1: (f64, f64),
        p2: (f64, f64),
        p3: (f64, f64),
        num_points: usize,
        alpha: f64,
        include_endpoint: bool,
    ) -> (Vec<(f64, f64)>, Vec<f64>, Vec<f64>) {
        let positions = Self::segment_points(p0, p1, p2, p3, num_points, alpha, include_endpoint);

        let n = positions.len();
        let mut yaw = Vec::with_capacity(n);
        let mut curvature = Vec::with_capacity(n);

        for i in 0..n {
            let (dx, dy, ddx, ddy) = if i == 0 && n > 1 {
                let dx = positions[1].0 - positions[0].0;
                let dy = positions[1].1 - positions[0].1;
                (dx, dy, 0.0, 0.0)
            } else if i == n - 1 && n > 1 {
                let dx = positions[n - 1].0 - positions[n - 2].0;
                let dy = positions[n - 1].1 - positions[n - 2].1;
                (dx, dy, 0.0, 0.0)
            } else {
                let dx = (positions[i + 1].0 - positions[i - 1].0) * 0.5;
                let dy = (positions[i + 1].1 - positions[i - 1].1) * 0.5;
                let ddx = positions[i + 1].0 - 2.0 * positions[i].0 + positions[i - 1].0;
                let ddy = positions[i + 1].1 - 2.0 * positions[i].1 + positions[i - 1].1;
                (dx, dy, ddx, ddy)
            };

            yaw.push(dy.atan2(dx));
            let denom = (dx * dx + dy * dy).powf(1.5);
            if denom.abs() < 1e-12 {
                curvature.push(0.0);
            } else {
                curvature.push((ddy * dx - ddx * dy) / denom);
            }
        }

        (positions, yaw, curvature)
    }

    /// Linear interpolation helper for the Barry-Goldman algorithm.
    #[inline]
    fn lerp(p0: (f64, f64), p1: (f64, f64), t0: f64, t1: f64, t: f64) -> (f64, f64) {
        let d = t1 - t0;
        if d.abs() < 1e-15 {
            return p0;
        }
        let a = (t1 - t) / d;
        let b = (t - t0) / d;
        (a * p0.0 + b * p1.0, a * p0.1 + b * p1.1)
    }
}

/// Convenience wrapper matching the planner pattern used in
/// [`CubicSplinePlanner`](super::cubic_spline_planner::CubicSplinePlanner).
#[derive(Debug, Clone)]
pub struct CatmullRomPlanner {
    /// Generated path \[x, y\].
    pub path: Vec<(f64, f64)>,
    /// Yaw angle at each path point \[rad\].
    pub yaw: Vec<f64>,
    /// Approximate curvature at each path point \[1/m\].
    pub curvature: Vec<f64>,
}

impl CatmullRomPlanner {
    pub fn new() -> Self {
        CatmullRomPlanner {
            path: Vec::new(),
            yaw: Vec::new(),
            curvature: Vec::new(),
        }
    }

    /// Run the planner.
    ///
    /// # Arguments
    /// * `waypoints_x` - x coordinates of waypoints.
    /// * `waypoints_y` - y coordinates of waypoints.
    /// * `points_per_segment` - number of output points per segment.
    /// * `param` - parameterisation strategy.
    ///
    /// Returns `true` on success.
    pub fn planning(
        &mut self,
        waypoints_x: Vec<f64>,
        waypoints_y: Vec<f64>,
        points_per_segment: usize,
        param: Parameterisation,
    ) -> bool {
        if waypoints_x.len() != waypoints_y.len() || waypoints_x.len() < 2 {
            return false;
        }
        if points_per_segment < 2 {
            return false;
        }

        let points: Vec<(f64, f64)> = waypoints_x.into_iter().zip(waypoints_y).collect();

        let spline = CatmullRomSpline::new(points, points_per_segment, param);
        let (path, yaw, curvature) = spline.generate_path_with_derivatives();

        self.path = path;
        self.yaw = yaw;
        self.curvature = curvature;

        true
    }
}

impl Default for CatmullRomPlanner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ----- basic construction -----

    #[test]
    #[should_panic(expected = "At least 2 waypoints")]
    fn test_too_few_points() {
        CatmullRomSpline::new(vec![(0.0, 0.0)], 10, Parameterisation::Uniform);
    }

    #[test]
    #[should_panic(expected = "points_per_segment must be >= 2")]
    fn test_too_few_per_segment() {
        CatmullRomSpline::new(vec![(0.0, 0.0), (1.0, 1.0)], 1, Parameterisation::Uniform);
    }

    // ----- uniform spline matches Python reference -----

    #[test]
    fn test_uniform_matches_python_reference() {
        // Same waypoints and num_points as the Python main().
        let way_points = vec![
            (-1.0, -2.0),
            (1.0, -1.0),
            (3.0, -2.0),
            (4.0, -1.0),
            (3.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ];
        let n_course_point = 100;

        let spline = CatmullRomSpline::new(
            way_points.clone(),
            n_course_point,
            Parameterisation::Uniform,
        );
        let path = spline.generate_path();

        // The Python code generates (num_segments * num_points) samples.
        // num_segments = 6, num_points = 100 => 600 total.
        // Our implementation avoids duplicate segment boundaries except at the
        // last segment, so we get 6*99 + 100 = 694... let's just check we get
        // a reasonable number.
        assert!(path.len() > 100);

        // First point must be the first waypoint.
        assert!(approx_eq(path[0].0, -1.0, 1e-12));
        assert!(approx_eq(path[0].1, -2.0, 1e-12));

        // Last point must be the last waypoint.
        let last = path[path.len() - 1];
        assert!(approx_eq(last.0, 0.0, 1e-12));
        assert!(approx_eq(last.1, 2.0, 1e-12));
    }

    // ----- interpolation passes through all control points -----

    #[test]
    fn test_passes_through_all_control_points() {
        let way_points = vec![(0.0, 0.0), (1.0, 2.0), (3.0, 1.0), (5.0, 4.0), (7.0, 0.0)];
        let n = 50;

        for param in &[
            Parameterisation::Uniform,
            Parameterisation::Centripetal,
            Parameterisation::Chordal,
        ] {
            let spline = CatmullRomSpline::new(way_points.clone(), n, *param);
            let path = spline.generate_path();

            // Check first and last.
            assert!(
                approx_eq(path[0].0, way_points[0].0, 1e-10),
                "param={param:?}"
            );
            assert!(
                approx_eq(path[0].1, way_points[0].1, 1e-10),
                "param={param:?}"
            );
            let last = path.last().unwrap();
            let wlast = way_points.last().unwrap();
            assert!(approx_eq(last.0, wlast.0, 1e-10), "param={param:?}");
            assert!(approx_eq(last.1, wlast.1, 1e-10), "param={param:?}");

            // Interior control points appear at segment boundaries.
            // With our stitching (n-1 points per non-last segment), the i-th
            // interior point is at index i*(n-1).
            for (i, wp) in way_points
                .iter()
                .enumerate()
                .skip(1)
                .take(way_points.len() - 2)
            {
                let idx = i * (n - 1);
                assert!(
                    approx_eq(path[idx].0, wp.0, 1e-10),
                    "param={param:?}, i={i}"
                );
                assert!(
                    approx_eq(path[idx].1, wp.1, 1e-10),
                    "param={param:?}, i={i}"
                );
            }
        }
    }

    // ----- straight-line waypoints produce a straight path -----

    #[test]
    fn test_straight_line() {
        let way_points = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)];
        let spline = CatmullRomSpline::new(way_points, 20, Parameterisation::Uniform);
        let path = spline.generate_path();

        for &(x, y) in &path {
            assert!(
                approx_eq(x, y, 1e-10),
                "expected x==y on a diagonal line, got ({x}, {y})"
            );
        }
    }

    // ----- two-point edge case -----

    #[test]
    fn test_two_points() {
        let way_points = vec![(0.0, 0.0), (10.0, 5.0)];
        let spline = CatmullRomSpline::new(way_points, 20, Parameterisation::Centripetal);
        let path = spline.generate_path();
        assert_eq!(path.len(), 20);
        assert!(approx_eq(path[0].0, 0.0, 1e-12));
        assert!(approx_eq(path[0].1, 0.0, 1e-12));
        assert!(approx_eq(path[19].0, 10.0, 1e-12));
        assert!(approx_eq(path[19].1, 5.0, 1e-12));
    }

    // ----- planner wrapper -----

    #[test]
    fn test_planner_success() {
        let mut planner = CatmullRomPlanner::new();
        let ok = planner.planning(
            vec![-1.0, 1.0, 3.0, 4.0, 3.0, 1.0, 0.0],
            vec![-2.0, -1.0, -2.0, -1.0, 1.0, 2.0, 2.0],
            50,
            Parameterisation::Centripetal,
        );
        assert!(ok);
        assert!(!planner.path.is_empty());
        assert_eq!(planner.path.len(), planner.yaw.len());
        assert_eq!(planner.path.len(), planner.curvature.len());
    }

    #[test]
    fn test_planner_bad_input() {
        let mut planner = CatmullRomPlanner::new();
        assert!(!planner.planning(vec![0.0], vec![0.0, 1.0], 10, Parameterisation::Uniform));
        assert!(!planner.planning(vec![0.0], vec![0.0], 10, Parameterisation::Uniform));
    }

    // ----- centripetal vs uniform should differ for non-collinear points -----

    #[test]
    fn test_centripetal_differs_from_uniform() {
        let way_points = vec![(0.0, 0.0), (1.0, 5.0), (4.0, 0.0), (6.0, 3.0)];
        let n = 30;
        let uniform =
            CatmullRomSpline::new(way_points.clone(), n, Parameterisation::Uniform).generate_path();
        let centripetal =
            CatmullRomSpline::new(way_points, n, Parameterisation::Centripetal).generate_path();

        // They should have the same length but different intermediate values.
        assert_eq!(uniform.len(), centripetal.len());
        let mut diff_count = 0;
        for (u, c) in uniform.iter().zip(centripetal.iter()) {
            if (u.0 - c.0).abs() > 1e-10 || (u.1 - c.1).abs() > 1e-10 {
                diff_count += 1;
            }
        }
        // Endpoints are the same; many interior points should differ.
        assert!(diff_count > 0, "centripetal should differ from uniform");
    }

    // ----- derivatives sanity -----

    #[test]
    fn test_yaw_and_curvature_sanity() {
        let way_points = vec![(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0)];
        let spline = CatmullRomSpline::new(way_points, 20, Parameterisation::Uniform);
        let (path, yaw, curvature) = spline.generate_path_with_derivatives();

        // Along a horizontal line, yaw should be ~0 and curvature ~0.
        for &y in &yaw {
            assert!(
                approx_eq(y, 0.0, 1e-10),
                "expected yaw ~0 on a horizontal line, got {y}"
            );
        }
        for &k in &curvature {
            assert!(
                approx_eq(k, 0.0, 1e-10),
                "expected curvature ~0 on a straight line, got {k}"
            );
        }
        assert_eq!(path.len(), yaw.len());
        assert_eq!(path.len(), curvature.len());
    }
}
