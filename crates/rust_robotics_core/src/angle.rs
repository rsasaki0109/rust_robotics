//! Angle helpers.

use core::f64::consts::{PI, TAU};

/// Wraps `angle` \[rad\] into `[-π, π]`.
///
/// Angles already in range (including both ends) come back unchanged, and
/// any other finite angle is reduced with one exact remainder, so the result
/// matches the classic "subtract 2π while above π" loop without its
/// unbounded running time. Non-finite input is returned as is.
pub fn normalize_angle(angle: f64) -> f64 {
    if (-PI..=PI).contains(&angle) || !angle.is_finite() {
        return angle;
    }
    let wrapped = angle % TAU;
    if wrapped > PI {
        wrapped - TAU
    } else if wrapped < -PI {
        wrapped + TAU
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The loop every module used to carry.
    fn reference(mut angle: f64) -> f64 {
        while angle > PI {
            angle -= TAU;
        }
        while angle < -PI {
            angle += TAU;
        }
        angle
    }

    #[test]
    fn matches_the_subtraction_loop() {
        for i in -2000..=2000 {
            let angle = i as f64 * 0.00731;
            assert_eq!(normalize_angle(angle), reference(angle), "angle {angle}");
        }
        for angle in [
            PI,
            -PI,
            0.0,
            -0.0,
            PI + 1e-12,
            -PI - 1e-12,
            3.0 * PI,
            -3.0 * PI,
        ] {
            assert_eq!(normalize_angle(angle), reference(angle), "angle {angle}");
        }
    }

    #[test]
    fn large_and_non_finite_angles_return() {
        let wrapped = normalize_angle(1.0e12);
        assert!((-PI..=PI).contains(&wrapped));
        assert_eq!(normalize_angle(f64::INFINITY), f64::INFINITY);
        assert!(normalize_angle(f64::NAN).is_nan());
    }
}
