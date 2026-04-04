#![allow(dead_code, clippy::new_without_default)]

//! 3D N-joint arm control
//!
//! Extension of the planar N-link arm to 3D space. Uses alternating
//! yaw/pitch joints to reach arbitrary 3D positions.
//!
//! Joint convention:
//! - Even-indexed joints (0, 2, 4, ...) rotate around the Z axis (yaw)
//! - Odd-indexed joints (1, 3, 5, ...) rotate around the Y axis (pitch)
//!
//! Each link extends along the local X direction after the accumulated
//! rotations.

use nalgebra::{Matrix3, Matrix3xX, Vector3};

/// Epsilon for numerical Jacobian computation.
const JACOBIAN_EPSILON: f64 = 1e-6;

/// Damping factor for Levenberg-Marquardt IK.
const DAMPING_LAMBDA: f64 = 0.5;

/// A 3D N-link arm with alternating yaw/pitch joints.
#[derive(Debug, Clone)]
pub struct NLinkArm3D {
    link_lengths: Vec<f64>,
    joint_angles: Vec<f64>,
}

impl NLinkArm3D {
    /// Creates a new arm with all joint angles initialized to zero.
    pub fn new(link_lengths: Vec<f64>) -> Self {
        let n = link_lengths.len();
        Self {
            link_lengths,
            joint_angles: vec![0.0; n],
        }
    }

    /// Creates a new arm with specified joint angles.
    ///
    /// # Panics
    ///
    /// Panics if `link_lengths` and `joint_angles` have different lengths.
    pub fn with_angles(link_lengths: Vec<f64>, joint_angles: Vec<f64>) -> Self {
        assert_eq!(
            link_lengths.len(),
            joint_angles.len(),
            "link_lengths and joint_angles must have the same length"
        );
        Self {
            link_lengths,
            joint_angles,
        }
    }

    /// Computes forward kinematics, returning positions of base, all joints,
    /// and the end effector (n+1 points total).
    ///
    /// For each joint i:
    /// - If i is even: rotate around Z by `joint_angles\[i\]` (yaw)
    /// - If i is odd: rotate around Y by `joint_angles\[i\]` (pitch)
    ///
    /// Then advance by `link_lengths\[i\]` along the current local X direction.
    pub fn forward_kinematics(&self) -> Vec<Vector3<f64>> {
        let n = self.link_lengths.len();
        let mut points = Vec::with_capacity(n + 1);
        let mut pos = Vector3::zeros();
        let mut rot = Matrix3::identity();

        points.push(pos);

        for i in 0..n {
            let angle = self.joint_angles[i];
            let local_rot = if i % 2 == 0 {
                rotation_z(angle)
            } else {
                rotation_y(angle)
            };
            rot *= local_rot;
            let direction = rot * Vector3::new(self.link_lengths[i], 0.0, 0.0);
            pos += direction;
            points.push(pos);
        }

        points
    }

    /// Returns the end-effector position.
    pub fn end_effector(&self) -> Vector3<f64> {
        let points = self.forward_kinematics();
        // forward_kinematics always pushes at least the base position
        *points
            .last()
            .expect("forward_kinematics always returns at least one point")
    }

    /// Computes the 3-by-n numerical Jacobian via finite differences.
    ///
    /// Each column j is `(end_effector(q + eps*e_j) - end_effector(q - eps*e_j)) / (2*eps)`.
    pub fn jacobian(&self) -> Matrix3xX<f64> {
        let n = self.joint_angles.len();
        let mut jac = Matrix3xX::zeros(n);

        for j in 0..n {
            let mut angles_plus = self.joint_angles.clone();
            let mut angles_minus = self.joint_angles.clone();
            angles_plus[j] += JACOBIAN_EPSILON;
            angles_minus[j] -= JACOBIAN_EPSILON;

            let arm_plus = NLinkArm3D {
                link_lengths: self.link_lengths.clone(),
                joint_angles: angles_plus,
            };
            let arm_minus = NLinkArm3D {
                link_lengths: self.link_lengths.clone(),
                joint_angles: angles_minus,
            };

            let diff = (arm_plus.end_effector() - arm_minus.end_effector())
                / (2.0 * JACOBIAN_EPSILON);
            jac.set_column(j, &diff);
        }

        jac
    }

    /// Solves inverse kinematics using damped least squares (Levenberg-Marquardt).
    ///
    /// `dq = J^T * (J * J^T + lambda * I)^{-1} * error`
    ///
    /// Returns `true` if the end effector converged to within `tolerance` of
    /// the target.
    pub fn inverse_kinematics(
        &mut self,
        target: Vector3<f64>,
        max_iter: usize,
        tolerance: f64,
    ) -> bool {
        for _ in 0..max_iter {
            let ee = self.end_effector();
            let error = target - ee;

            if error.norm() < tolerance {
                return true;
            }

            let j = self.jacobian();
            // J * J^T is 3x3
            let jjt = &j * j.transpose();
            let damped = jjt + Matrix3::identity() * DAMPING_LAMBDA;

            // Solve (J*J^T + lambda*I) * x = error for x, then dq = J^T * x
            if let Some(decomp) = damped.try_inverse() {
                let x = decomp * error;
                let dq = j.transpose() * x;

                for i in 0..self.joint_angles.len() {
                    self.joint_angles[i] += dq[i];
                }
            } else {
                // Singular; cannot proceed
                return false;
            }
        }

        // Check final convergence
        let ee = self.end_effector();
        (target - ee).norm() < tolerance
    }

    /// Returns a reference to the current joint angles.
    pub fn joint_angles(&self) -> &[f64] {
        &self.joint_angles
    }

    /// Returns a reference to the link lengths.
    pub fn link_lengths(&self) -> &[f64] {
        &self.link_lengths
    }
}

/// Rotation matrix around the Z axis by angle theta.
fn rotation_z(theta: f64) -> Matrix3<f64> {
    let (s, c) = theta.sin_cos();
    Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

/// Rotation matrix around the Y axis by angle theta.
fn rotation_y(theta: f64) -> Matrix3<f64> {
    let (s, c) = theta.sin_cos();
    Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_3d_arm_creation() {
        let arm = NLinkArm3D::new(vec![1.0, 1.0, 1.0]);
        assert_eq!(arm.link_lengths().len(), 3);
        assert_eq!(arm.joint_angles().len(), 3);
        assert!(arm.joint_angles().iter().all(|&a| a == 0.0));

        let arm2 = NLinkArm3D::with_angles(vec![1.0, 2.0], vec![0.5, -0.3]);
        assert_eq!(arm2.link_lengths(), &[1.0, 2.0]);
        assert_eq!(arm2.joint_angles(), &[0.5, -0.3]);
    }

    #[test]
    fn test_forward_kinematics_straight() {
        // All angles zero: every link extends along X.
        let arm = NLinkArm3D::new(vec![1.0, 1.5, 0.5]);
        let ee = arm.end_effector();
        let total: f64 = arm.link_lengths().iter().sum();
        assert!(
            (ee.x - total).abs() < 1e-10,
            "Expected x={total}, got x={}",
            ee.x
        );
        assert!(ee.y.abs() < 1e-10, "Expected y=0, got y={}", ee.y);
        assert!(ee.z.abs() < 1e-10, "Expected z=0, got z={}", ee.z);

        // Also check that we get n+1 points
        let points = arm.forward_kinematics();
        assert_eq!(points.len(), 4);
        assert!((points[0] - Vector3::zeros()).norm() < 1e-10);
    }

    #[test]
    fn test_inverse_kinematics_reachable() {
        // 4-link arm, total reach = 4.0. Target on X-axis at distance 3.0.
        let mut arm = NLinkArm3D::with_angles(vec![1.0; 4], vec![0.1, 0.1, 0.1, 0.1]);
        let target = Vector3::new(3.0, 0.0, 0.0);
        let converged = arm.inverse_kinematics(target, 500, 0.05);
        assert!(converged, "IK should converge for reachable in-plane target");

        let ee = arm.end_effector();
        let dist = (ee - target).norm();
        assert!(
            dist < 0.05,
            "End effector should be within tolerance, got dist={dist}"
        );
    }

    #[test]
    fn test_inverse_kinematics_3d_target() {
        // 6-link arm reaching a point above the base plane.
        // Total reach = 6.0. Target at (2, 1, 2) has distance ~3.0.
        let mut arm = NLinkArm3D::with_angles(vec![1.0; 6], vec![0.1; 6]);
        let target = Vector3::new(2.0, 1.0, 2.0);
        let converged = arm.inverse_kinematics(target, 1000, 0.05);
        assert!(converged, "IK should converge for reachable 3D target");

        let ee = arm.end_effector();
        let dist = (ee - target).norm();
        assert!(
            dist < 0.05,
            "End effector should reach 3D target, got dist={dist}"
        );
    }
}
