//! Lie-group utilities used by estimation and optimization algorithms.
//!
//! Tangent vectors use translation-first ordering throughout this module:
//! `[vx, vy, omega]` for SE(2), and
//! `[vx, vy, vz, omega_x, omega_y, omega_z]` for SE(3).
//!
//! The implementation follows the closed-form exponential, logarithm, and
//! Jacobian expressions used by MathematicalRobotics, with explicit series
//! expansions around zero and a stable SO(3) logarithm around pi.

use nalgebra::{Matrix2, Matrix3, Matrix4, SMatrix, SVector, Vector2, Vector3};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// A six-dimensional SE(3) tangent vector in translation-first order.
pub type Vector6 = SVector<f64, 6>;
/// A 6x6 matrix acting on an SE(3) tangent vector.
pub type Matrix6 = SMatrix<f64, 6, 6>;

const SMALL_ANGLE_SQUARED: f64 = 1.0e-12;
const NEAR_PI: f64 = 1.0e-6;

/// Returns the 3x3 skew-symmetric matrix for a vector.
pub fn skew(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

/// Extracts a vector from the skew-symmetric part of a 3x3 matrix.
pub fn unskew(matrix: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::new(matrix[(2, 1)], matrix[(0, 2)], matrix[(1, 0)])
}

/// Exponential map from so(2) to SO(2).
pub fn so2_exp(angle: f64) -> Matrix2<f64> {
    let (sin_angle, cos_angle) = angle.sin_cos();
    Matrix2::new(cos_angle, -sin_angle, sin_angle, cos_angle)
}

/// Logarithm map from SO(2) to so(2), returning an angle in `[-pi, pi]`.
pub fn so2_log(rotation: &Matrix2<f64>) -> f64 {
    rotation[(1, 0)].atan2(rotation[(0, 0)])
}

/// Exponential map from so(3) to SO(3).
pub fn so3_exp(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta_squared = omega.norm_squared();
    let omega_hat = skew(omega);
    let omega_hat_squared = omega_hat * omega_hat;
    let (a, b) = rodrigues_coefficients(theta_squared);
    Matrix3::identity() + a * omega_hat + b * omega_hat_squared
}

/// Logarithm map from SO(3) to the principal so(3) tangent vector.
pub fn so3_log(rotation: &Matrix3<f64>) -> Vector3<f64> {
    let cos_theta = ((rotation.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();

    if theta * theta < SMALL_ANGLE_SQUARED {
        return 0.5 * unskew(&(rotation - rotation.transpose()));
    }

    if core::f64::consts::PI - theta < NEAR_PI {
        return theta * axis_near_pi(rotation);
    }

    let scale = theta / (2.0 * theta.sin());
    scale * unskew(&(rotation - rotation.transpose()))
}

/// Left Jacobian of SO(3).
pub fn so3_left_jacobian(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta_squared = omega.norm_squared();
    let omega_hat = skew(omega);
    let omega_hat_squared = omega_hat * omega_hat;
    let (b, c) = jacobian_coefficients(theta_squared);
    Matrix3::identity() + b * omega_hat + c * omega_hat_squared
}

/// Inverse of the SO(3) left Jacobian.
pub fn so3_left_jacobian_inverse(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta_squared = omega.norm_squared();
    let omega_hat = skew(omega);
    let omega_hat_squared = omega_hat * omega_hat;
    let coefficient = if theta_squared < SMALL_ANGLE_SQUARED {
        1.0 / 12.0 + theta_squared / 720.0 + theta_squared * theta_squared / 30_240.0
    } else {
        let theta = theta_squared.sqrt();
        1.0 / theta_squared - (1.0 + theta.cos()) / (2.0 * theta * theta.sin())
    };
    Matrix3::identity() - 0.5 * omega_hat + coefficient * omega_hat_squared
}

/// Exponential map from se(2) to a homogeneous SE(2) matrix.
pub fn se2_exp(tangent: &Vector3<f64>) -> Matrix3<f64> {
    let translation_tangent = Vector2::new(tangent.x, tangent.y);
    let angle = tangent.z;
    let jacobian = so2_left_jacobian(angle);
    let translation = jacobian * translation_tangent;
    let rotation = so2_exp(angle);

    Matrix3::new(
        rotation[(0, 0)],
        rotation[(0, 1)],
        translation.x,
        rotation[(1, 0)],
        rotation[(1, 1)],
        translation.y,
        0.0,
        0.0,
        1.0,
    )
}

/// Logarithm map from a homogeneous SE(2) matrix.
pub fn se2_log(transform: &Matrix3<f64>) -> Vector3<f64> {
    let rotation = transform.fixed_view::<2, 2>(0, 0).into_owned();
    let translation = Vector2::new(transform[(0, 2)], transform[(1, 2)]);
    let angle = so2_log(&rotation);
    let translation_tangent = so2_left_jacobian_inverse(angle) * translation;
    Vector3::new(translation_tangent.x, translation_tangent.y, angle)
}

/// Inverse of a homogeneous SE(2) matrix.
pub fn se2_inverse(transform: &Matrix3<f64>) -> Matrix3<f64> {
    let rotation = transform.fixed_view::<2, 2>(0, 0).into_owned();
    let translation = Vector2::new(transform[(0, 2)], transform[(1, 2)]);
    let inverse_rotation = rotation.transpose();
    let inverse_translation = -(inverse_rotation * translation);
    Matrix3::new(
        inverse_rotation[(0, 0)],
        inverse_rotation[(0, 1)],
        inverse_translation.x,
        inverse_rotation[(1, 0)],
        inverse_rotation[(1, 1)],
        inverse_translation.y,
        0.0,
        0.0,
        1.0,
    )
}

/// Adjoint matrix of an SE(2) transform for translation-first tangents.
pub fn se2_adjoint(transform: &Matrix3<f64>) -> Matrix3<f64> {
    let rotation = transform.fixed_view::<2, 2>(0, 0);
    let tx = transform[(0, 2)];
    let ty = transform[(1, 2)];
    Matrix3::new(
        rotation[(0, 0)],
        rotation[(0, 1)],
        ty,
        rotation[(1, 0)],
        rotation[(1, 1)],
        -tx,
        0.0,
        0.0,
        1.0,
    )
}

/// Exponential map from se(3) to a homogeneous SE(3) matrix.
pub fn se3_exp(tangent: &Vector6) -> Matrix4<f64> {
    let velocity = tangent.fixed_rows::<3>(0).into_owned();
    let omega = tangent.fixed_rows::<3>(3).into_owned();
    let rotation = so3_exp(&omega);
    let translation = so3_left_jacobian(&omega) * velocity;

    Matrix4::new(
        rotation[(0, 0)],
        rotation[(0, 1)],
        rotation[(0, 2)],
        translation.x,
        rotation[(1, 0)],
        rotation[(1, 1)],
        rotation[(1, 2)],
        translation.y,
        rotation[(2, 0)],
        rotation[(2, 1)],
        rotation[(2, 2)],
        translation.z,
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

/// Logarithm map from a homogeneous SE(3) matrix.
pub fn se3_log(transform: &Matrix4<f64>) -> Vector6 {
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    let translation = transform.fixed_view::<3, 1>(0, 3).into_owned();
    let omega = so3_log(&rotation);
    let velocity = so3_left_jacobian_inverse(&omega) * translation;
    Vector6::new(
        velocity.x, velocity.y, velocity.z, omega.x, omega.y, omega.z,
    )
}

/// Inverse of a homogeneous SE(3) matrix.
pub fn se3_inverse(transform: &Matrix4<f64>) -> Matrix4<f64> {
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    let translation = transform.fixed_view::<3, 1>(0, 3).into_owned();
    let inverse_rotation = rotation.transpose();
    let inverse_translation = -(inverse_rotation * translation);
    Matrix4::new(
        inverse_rotation[(0, 0)],
        inverse_rotation[(0, 1)],
        inverse_rotation[(0, 2)],
        inverse_translation.x,
        inverse_rotation[(1, 0)],
        inverse_rotation[(1, 1)],
        inverse_rotation[(1, 2)],
        inverse_translation.y,
        inverse_rotation[(2, 0)],
        inverse_rotation[(2, 1)],
        inverse_rotation[(2, 2)],
        inverse_translation.z,
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

/// Adjoint matrix of an SE(3) transform for translation-first tangents.
pub fn se3_adjoint(transform: &Matrix4<f64>) -> Matrix6 {
    let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
    let translation = transform.fixed_view::<3, 1>(0, 3).into_owned();
    let cross_rotation = skew(&translation) * rotation;
    let mut adjoint = Matrix6::zeros();
    adjoint.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    adjoint
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&cross_rotation);
    adjoint.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
    adjoint
}

fn so2_left_jacobian(angle: f64) -> Matrix2<f64> {
    let angle_squared = angle * angle;
    let (a, b) = if angle_squared < SMALL_ANGLE_SQUARED {
        (
            1.0 - angle_squared / 6.0 + angle_squared * angle_squared / 120.0,
            angle * (0.5 - angle_squared / 24.0 + angle_squared * angle_squared / 720.0),
        )
    } else {
        (angle.sin() / angle, (1.0 - angle.cos()) / angle)
    };
    Matrix2::new(a, -b, b, a)
}

fn so2_left_jacobian_inverse(angle: f64) -> Matrix2<f64> {
    let half_angle = 0.5 * angle;
    let diagonal = if angle * angle < SMALL_ANGLE_SQUARED {
        1.0 - angle * angle / 12.0 - angle.powi(4) / 720.0
    } else {
        half_angle / half_angle.tan()
    };
    Matrix2::new(diagonal, half_angle, -half_angle, diagonal)
}

fn rodrigues_coefficients(theta_squared: f64) -> (f64, f64) {
    if theta_squared < SMALL_ANGLE_SQUARED {
        (
            1.0 - theta_squared / 6.0 + theta_squared * theta_squared / 120.0,
            0.5 - theta_squared / 24.0 + theta_squared * theta_squared / 720.0,
        )
    } else {
        let theta = theta_squared.sqrt();
        (theta.sin() / theta, (1.0 - theta.cos()) / theta_squared)
    }
}

fn jacobian_coefficients(theta_squared: f64) -> (f64, f64) {
    if theta_squared < SMALL_ANGLE_SQUARED {
        (
            0.5 - theta_squared / 24.0 + theta_squared * theta_squared / 720.0,
            1.0 / 6.0 - theta_squared / 120.0 + theta_squared * theta_squared / 5_040.0,
        )
    } else {
        let theta = theta_squared.sqrt();
        (
            (1.0 - theta.cos()) / theta_squared,
            (theta - theta.sin()) / (theta_squared * theta),
        )
    }
}

fn axis_near_pi(rotation: &Matrix3<f64>) -> Vector3<f64> {
    let xx = ((rotation[(0, 0)] + 1.0) * 0.5).max(0.0).sqrt();
    let yy = ((rotation[(1, 1)] + 1.0) * 0.5).max(0.0).sqrt();
    let zz = ((rotation[(2, 2)] + 1.0) * 0.5).max(0.0).sqrt();

    let mut axis = if xx >= yy && xx >= zz && xx > f64::EPSILON {
        Vector3::new(
            xx,
            (rotation[(0, 1)] + rotation[(1, 0)]) / (4.0 * xx),
            (rotation[(0, 2)] + rotation[(2, 0)]) / (4.0 * xx),
        )
    } else if yy >= zz && yy > f64::EPSILON {
        Vector3::new(
            (rotation[(0, 1)] + rotation[(1, 0)]) / (4.0 * yy),
            yy,
            (rotation[(1, 2)] + rotation[(2, 1)]) / (4.0 * yy),
        )
    } else if zz > f64::EPSILON {
        Vector3::new(
            (rotation[(0, 2)] + rotation[(2, 0)]) / (4.0 * zz),
            (rotation[(1, 2)] + rotation[(2, 1)]) / (4.0 * zz),
            zz,
        )
    } else {
        Vector3::x()
    };

    let antisymmetric = unskew(&(rotation - rotation.transpose()));
    if axis.dot(&antisymmetric) < 0.0 {
        axis = -axis;
    }
    axis.normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1.0e-9;

    fn assert_matrix_close<const R: usize, const C: usize>(
        actual: &SMatrix<f64, R, C>,
        expected: &SMatrix<f64, R, C>,
        tolerance: f64,
    ) {
        let error = (actual - expected).norm();
        assert!(
            error <= tolerance,
            "matrix error {error} exceeded tolerance {tolerance}\nactual={actual}\nexpected={expected}"
        );
    }

    #[test]
    fn skew_matches_cross_product() {
        let a = Vector3::new(0.3, -0.7, 1.2);
        let b = Vector3::new(-0.4, 0.9, 0.2);
        assert_matrix_close(&(skew(&a) * b), &a.cross(&b), TOLERANCE);
    }

    #[test]
    fn so3_round_trips_regular_and_small_rotations() {
        for omega in [
            Vector3::new(0.2, -0.3, 0.4),
            Vector3::new(1.0e-10, -2.0e-10, 3.0e-10),
            Vector3::zeros(),
        ] {
            let recovered = so3_log(&so3_exp(&omega));
            assert_matrix_close(&recovered, &omega, TOLERANCE);
        }
    }

    #[test]
    fn so3_log_is_stable_near_pi() {
        let axis = Vector3::new(1.0, -2.0, 0.5).normalize();
        let omega = axis * (core::f64::consts::PI - 1.0e-8);
        let recovered_rotation = so3_exp(&so3_log(&so3_exp(&omega)));
        assert_matrix_close(&recovered_rotation, &so3_exp(&omega), 1.0e-7);
    }

    #[test]
    fn so3_jacobian_and_inverse_cancel() {
        for omega in [
            Vector3::new(0.4, -0.2, 0.1),
            Vector3::new(1.0e-10, 2.0e-10, -1.0e-10),
        ] {
            let product = so3_left_jacobian(&omega) * so3_left_jacobian_inverse(&omega);
            assert_matrix_close(&product, &Matrix3::identity(), TOLERANCE);
        }
    }

    #[test]
    fn se2_round_trip_and_inverse() {
        let tangent = Vector3::new(1.2, -0.7, 0.8);
        let transform = se2_exp(&tangent);
        assert_matrix_close(&se2_log(&transform), &tangent, TOLERANCE);
        assert_matrix_close(
            &(transform * se2_inverse(&transform)),
            &Matrix3::identity(),
            TOLERANCE,
        );
    }

    #[test]
    fn se3_round_trip_and_inverse() {
        let tangent = Vector6::new(1.2, -0.7, 0.3, 0.2, -0.4, 0.1);
        let transform = se3_exp(&tangent);
        assert_matrix_close(&se3_log(&transform), &tangent, TOLERANCE);
        assert_matrix_close(
            &(transform * se3_inverse(&transform)),
            &Matrix4::identity(),
            TOLERANCE,
        );
    }

    #[test]
    fn adjoints_match_group_conjugation() {
        let se2_pose = se2_exp(&Vector3::new(0.4, -0.2, 0.7));
        let se2_delta = Vector3::new(0.2, 0.1, -0.3);
        assert_matrix_close(
            &(se2_pose * se2_exp(&se2_delta) * se2_inverse(&se2_pose)),
            &se2_exp(&(se2_adjoint(&se2_pose) * se2_delta)),
            TOLERANCE,
        );

        let se3_pose = se3_exp(&Vector6::new(0.4, -0.2, 0.7, 0.1, 0.3, -0.2));
        let se3_delta = Vector6::new(0.2, 0.1, -0.3, -0.2, 0.15, 0.1);
        assert_matrix_close(
            &(se3_pose * se3_exp(&se3_delta) * se3_inverse(&se3_pose)),
            &se3_exp(&(se3_adjoint(&se3_pose) * se3_delta)),
            1.0e-8,
        );
    }
}
