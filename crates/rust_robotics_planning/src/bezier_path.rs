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
    if n_points == 0 {
        return Vec::new();
    }

    if n_points == 1 {
        return vec![bezier(0.0, control_points)];
    }

    let mut traj: Vec<(f64, f64)> = Vec::with_capacity(n_points);
    for i in 0..n_points {
        let t = (i as f64) / ((n_points as f64) - 1.0);
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
) -> Vec<Vec<(f64, f64)>> {
    let mut w = vec![control_points.to_vec()];
    for i in 0..n_derivatives {
        let current = &w[i];
        let n = current.len();
        if n <= 1 {
            break;
        }
        let next = (0..n - 1)
            .map(|j| {
                (
                    (n - 1) as f64 * (current[j + 1].0 - current[j].0),
                    (n - 1) as f64 * (current[j + 1].1 - current[j].1),
                )
            })
            .collect();
        w.push(next);
    }
    w
}

pub fn curvature(dx: f64, dy: f64, ddx: f64, ddy: f64) -> f64 {
    (dx * ddy - dy * ddx) / (dx.powi(2) + dy.powi(2)).powf(3. / 2.)
}

#[cfg(test)]
#[allow(clippy::excessive_precision)]
mod tests {
    use super::*;

    type PathFingerprint = (f64, f64, f64, f64);

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

    #[test]
    fn test_calc_4points_bezier_path_matches_upstream_main_example() {
        let start = (10.0, 1.0, std::f64::consts::PI);
        let end = (0.0, -3.0, -45.0_f64.to_radians());
        let (path, control_points) = calc_4points_bezier_path(start, end, 3.0);

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

        assert_eq!(path.len(), 100);
        for (actual, expected) in control_points
            .iter()
            .copied()
            .zip(expected_control_points.iter().copied())
        {
            assert_point_close(actual, expected, 1e-12);
        }
        for (index, x, y) in expected_samples {
            assert_point_close(path[index], (x, y), 1e-12);
        }
    }

    #[test]
    fn test_bezier_derivatives_control_points_match_upstream_main_example() {
        let control_points = [
            (10.0, 1.0),
            (6.409_890_128_576_997, 1.0),
            (-2.538_591_035_287_97, -0.461_408_964_712_031),
            (0.0, -3.0),
        ];
        let derivatives = bezier_derivatives_control_points(control_points, 2);
        let expected_first = [
            (-10.770_329_614_269_009, 0.0),
            (-26.845_443_491_594_9, -4.384_226_894_136_093),
            (7.615_773_105_863_909, -7.615_773_105_863_908),
        ];
        let expected_second = [
            (-32.150_227_754_651_78, -8.768_453_788_272_19),
            (68.922_433_194_917_62, -6.463_092_423_455_629),
        ];

        assert_eq!(derivatives.len(), 3);
        assert_eq!(derivatives[1].len(), expected_first.len());
        assert_eq!(derivatives[2].len(), expected_second.len());

        for (actual, expected) in derivatives[1]
            .iter()
            .copied()
            .zip(expected_first.iter().copied())
        {
            assert_point_close(actual, expected, 1e-12);
        }
        for (actual, expected) in derivatives[2]
            .iter()
            .copied()
            .zip(expected_second.iter().copied())
        {
            assert_point_close(actual, expected, 1e-12);
        }
    }

    #[test]
    fn test_calc_4points_bezier_path_matches_upstream_main2_offset_sweep() {
        let start = (10.0, 1.0, std::f64::consts::PI);
        let end = (0.0, -3.0, -45.0_f64.to_radians());
        let cases = [
            (
                1.0,
                [
                    (0usize, 10.0, 1.0),
                    (25, 2.761_187_127_865_844, 1.452_632_145_801_054),
                    (50, -1.957_892_182_702_414, 1.854_166_206_916_617),
                    (75, -3.139_220_823_564_846, 0.770_058_618_915_284),
                    (99, 0.0, -3.0),
                ],
                (
                    44.990_387_229_033_885,
                    88.471_152_619_864_35,
                    -7_004.767_925_192_914,
                    2_313.162_758_091_821,
                ),
            ),
            (
                2.0,
                [
                    (0usize, 10.0, 1.0),
                    (25, 5.585_091_352_758_839, 0.908_115_188_430_895),
                    (50, 1.483_176_409_032_695, 0.411_932_103_611_869),
                    (75, -0.830_539_259_488_13, -0.819_342_229_624_641),
                    (99, 0.0, -3.0),
                ],
                (
                    272.495_193_614_516_95,
                    -5.764_423_690_067_827,
                    4_097.949_336_726_745,
                    -3_378.285_301_224_811_5,
                ),
            ),
            (
                3.0,
                [
                    (0usize, 10.0, 1.0),
                    (25, 6.526_392_761_056_504, 0.726_609_535_974_175),
                    (50, 2.630_199_272_944_397, -0.068_812_597_489_713),
                    (75, -0.060_978_738_129_225, -1.349_142_512_471_283),
                    (99, 0.0, -3.0),
                ],
                (
                    348.330_129_076_344_6,
                    -37.176_282_460_045_215,
                    7_798.855_090_699_961,
                    -5_275.434_654_330_354,
                ),
            ),
            (
                4.0,
                [
                    (0usize, 10.0, 1.0),
                    (25, 6.997_043_465_205_337, 0.635_856_709_745_815),
                    (50, 3.203_710_704_900_249, -0.309_184_948_040_505),
                    (75, 0.323_801_522_550_227, -1.614_042_653_894_604),
                    (99, 0.0, -3.0),
                ],
                (
                    386.247_596_807_258_45,
                    -52.882_211_845_033_93,
                    9_649.307_967_686_569,
                    -6_224.009_330_883_126_5,
                ),
            ),
        ];

        for (offset, expected_samples, expected_fingerprint) in cases {
            let (path, _) = calc_4points_bezier_path(start, end, offset);
            assert_eq!(path.len(), 100);
            for (index, x, y) in expected_samples {
                assert_point_close(path[index], (x, y), 1e-12);
            }
            assert_path_fingerprint_close(path_fingerprint(&path), expected_fingerprint, 1e-9);
        }
    }
}
