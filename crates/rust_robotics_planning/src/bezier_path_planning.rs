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
#[allow(clippy::excessive_precision)]
mod tests {
    use super::*;

    type PathFingerprint = (f64, f64, f64, f64);
    type CurvatureFingerprint = (f64, f64, f64);

    fn assert_point_close(actual: (f64, f64), expected: (f64, f64), tolerance: f64) {
        assert!(
            (actual.0 - expected.0).abs() < tolerance,
            "x mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.1 - expected.1).abs() < tolerance,
            "y mismatch: actual={actual:?}, expected={expected:?}"
        );
    }

    fn path_fingerprint(path: &[(f64, f64)]) -> PathFingerprint {
        let sum_x = path.iter().map(|point| point.0).sum();
        let sum_y = path.iter().map(|point| point.1).sum();
        let weighted_sum_x = path
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.0)
            .sum();
        let weighted_sum_y = path
            .iter()
            .enumerate()
            .map(|(index, point)| (index + 1) as f64 * point.1)
            .sum();
        (sum_x, sum_y, weighted_sum_x, weighted_sum_y)
    }

    fn curvature_fingerprint(curvature: &[f64]) -> CurvatureFingerprint {
        let sum = curvature.iter().copied().sum();
        let weighted_sum = curvature
            .iter()
            .enumerate()
            .map(|(index, value)| (index + 1) as f64 * value)
            .sum();
        let sum_sq = curvature.iter().map(|value| value * value).sum();
        (sum, weighted_sum, sum_sq)
    }

    fn assert_path_fingerprint_close(
        actual: PathFingerprint,
        expected: PathFingerprint,
        tolerance: f64,
    ) {
        assert!(
            (actual.0 - expected.0).abs() < tolerance,
            "sum_x mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.1 - expected.1).abs() < tolerance,
            "sum_y mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.2 - expected.2).abs() < tolerance,
            "weighted_sum_x mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.3 - expected.3).abs() < tolerance,
            "weighted_sum_y mismatch: actual={actual:?}, expected={expected:?}"
        );
    }

    fn assert_curvature_fingerprint_close(
        actual: CurvatureFingerprint,
        expected: CurvatureFingerprint,
        tolerance: f64,
    ) {
        assert!(
            (actual.0 - expected.0).abs() < tolerance,
            "sum mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.1 - expected.1).abs() < tolerance,
            "weighted_sum mismatch: actual={actual:?}, expected={expected:?}"
        );
        assert!(
            (actual.2 - expected.2).abs() < tolerance,
            "sum_sq mismatch: actual={actual:?}, expected={expected:?}"
        );
    }

    #[test]
    fn test_bezier_path_planning() {
        let mut planner = BezierPathPlanner::new();
        let result = planner.planning(
            10.0,
            1.0,
            180.0_f64.to_radians(),
            0.0,
            -3.0,
            (-45.0_f64).to_radians(),
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

    #[test]
    fn test_planning_matches_upstream_main_example() {
        let mut planner = BezierPathPlanner::new();
        planner.planning(
            10.0,
            1.0,
            180.0_f64.to_radians(),
            0.0,
            -3.0,
            (-45.0_f64).to_radians(),
            3.0,
        );

        let expected_control_points = [
            (10.0, 1.0),
            (6.409_890_128_576_997, 1.0),
            (-2.538_591_035_287_97, -0.461_408_964_712_031),
            (0.0, -3.0),
        ];
        let expected_samples = [
            (0usize, 10.0, 1.0),
            (25, 6.526_392_761_056_504, 0.726_609_535_974_175),
            (50, 2.630_199_272_944_397, -0.068_812_597_489_713),
            (75, -0.060_978_738_129_225, -1.349_142_512_471_283),
            (99, 0.0, -3.0),
        ];

        assert_eq!(planner.path.len(), 100);
        for (actual, expected) in planner
            .control_points
            .iter()
            .copied()
            .zip(expected_control_points.iter().copied())
        {
            assert_point_close(actual, expected, 1e-12);
        }
        for (index, x, y) in expected_samples {
            assert_point_close(planner.path[index], (x, y), 1e-12);
        }
    }

    #[test]
    fn test_calc_curvature_matches_upstream_main_example() {
        let control_points = [
            (10.0, 1.0),
            (6.409_890_128_576_997, 1.0),
            (-2.538_591_035_287_97, -0.461_408_964_712_031),
            (0.0, -3.0),
        ];

        let curvature = calc_curvature(&control_points, 0.86);
        assert!((curvature - 1.203_883_311_167_986).abs() < 1e-12);
    }

    #[test]
    fn test_planning_with_control_points_matches_upstream_reference() {
        let mut planner = BezierPathPlanner::new();
        planner.planning_with_control_points(
            vec![(-1.0, 0.0), (3.0, -3.0), (4.0, 1.0), (2.0, 1.0), (1.0, 3.0)],
            100,
        );

        let expected_path_samples = [
            (0usize, -1.0, 0.0),
            (25, 1.908_827_926_528_655, -0.991_419_119_052_972),
            (50, 2.749_694_941_997_521, 0.090_314_761_977_827),
            (75, 2.108_174_996_479_529, 1.482_624_053_372_864),
            (99, 1.0, 3.0),
        ];
        let expected_curvature_samples = [
            (0usize, 0.114),
            (25, 0.686_973_069_873_466),
            (50, 0.778_736_996_781_883),
            (75, 0.123_309_580_949_893),
            (99, -0.268_328_157_299_975),
        ];

        assert_eq!(planner.path.len(), 100);
        assert_eq!(planner.curvature.len(), 100);
        for (index, x, y) in expected_path_samples {
            assert_point_close(planner.path[index], (x, y), 1e-12);
        }
        for (index, expected) in expected_curvature_samples {
            assert!(
                (planner.curvature[index] - expected).abs() < 1e-12,
                "curvature mismatch at {index}: actual={}, expected={expected}",
                planner.curvature[index]
            );
        }
        assert_path_fingerprint_close(
            path_fingerprint(&planner.path),
            (
                178.183_164_845_750_3,
                41.116_834_432_822_614,
                10_028.516_464_168_946,
                5_417.733_506_201_01,
            ),
            1e-9,
        );
        assert_curvature_fingerprint_close(
            curvature_fingerprint(&planner.curvature),
            (
                39.037_977_305_450_994,
                1_413.299_260_366_335_1,
                30.142_637_421_645_173,
            ),
            1e-9,
        );
    }
}
