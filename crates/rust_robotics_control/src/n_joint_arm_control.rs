#![allow(
    dead_code,
    clippy::needless_borrows_for_generic_args,
    clippy::new_without_default
)]

//! Generalized N-joint planar arm inverse kinematics and control.
//!
//! Uses the Jacobian pseudo-inverse method to solve inverse kinematics
//! for a planar arm with an arbitrary number of revolute joints.
//!
//! Reference: PythonRobotics ArmNavigation/n\_joint\_arm\_to\_point\_control

use std::f64::consts::PI;

/// Simulation parameters
const KP: f64 = 2.0;
const DT: f64 = 0.1;
const DEFAULT_N_ITERATIONS: usize = 10000;
const DEFAULT_GOAL_THRESHOLD: f64 = 0.1;

/// A planar N-link arm with forward and inverse kinematics.
#[derive(Debug, Clone)]
pub struct NLinkArm {
    /// Length of each link
    pub link_lengths: Vec<f64>,
    /// Current joint angles \[rad\]
    pub joint_angles: Vec<f64>,
    /// Computed joint positions (n\_links + 1 points, including the base at origin)
    pub points: Vec<[f64; 2]>,
    /// End-effector position
    pub end_effector: [f64; 2],
    /// Number of links
    pub n_links: usize,
}

impl NLinkArm {
    /// Creates a new N-link arm.
    ///
    /// # Panics
    ///
    /// Panics if `link_lengths` and `joint_angles` have different lengths.
    pub fn new(link_lengths: Vec<f64>, joint_angles: Vec<f64>) -> Self {
        assert_eq!(
            link_lengths.len(),
            joint_angles.len(),
            "link_lengths and joint_angles must have the same length"
        );
        let n_links = link_lengths.len();
        let mut arm = NLinkArm {
            link_lengths,
            joint_angles,
            points: vec![[0.0, 0.0]; n_links + 1],
            end_effector: [0.0, 0.0],
            n_links,
        };
        arm.update_points();
        arm
    }

    /// Updates joint angles and recomputes link positions.
    pub fn update_joints(&mut self, joint_angles: &[f64]) {
        assert_eq!(joint_angles.len(), self.n_links);
        self.joint_angles = joint_angles.to_vec();
        self.update_points();
    }

    /// Recomputes all joint positions from current angles.
    pub fn update_points(&mut self) {
        self.points[0] = [0.0, 0.0];
        for i in 1..=self.n_links {
            let cumulative_angle: f64 = self.joint_angles[..i].iter().sum();
            self.points[i][0] =
                self.points[i - 1][0] + self.link_lengths[i - 1] * cumulative_angle.cos();
            self.points[i][1] =
                self.points[i - 1][1] + self.link_lengths[i - 1] * cumulative_angle.sin();
        }
        self.end_effector = self.points[self.n_links];
    }

    /// Total workspace radius (sum of all link lengths).
    pub fn workspace_radius(&self) -> f64 {
        self.link_lengths.iter().sum()
    }
}

/// Computes the forward kinematics for given link lengths and joint angles.
///
/// Returns the end-effector position \[x, y\].
pub fn forward_kinematics(link_lengths: &[f64], joint_angles: &[f64]) -> [f64; 2] {
    let n = link_lengths.len();
    let mut x = 0.0;
    let mut y = 0.0;
    for i in 1..=n {
        let cumulative_angle: f64 = joint_angles[..i].iter().sum();
        x += link_lengths[i - 1] * cumulative_angle.cos();
        y += link_lengths[i - 1] * cumulative_angle.sin();
    }
    [x, y]
}

/// Computes the Jacobian matrix for the planar arm.
///
/// Returns a 2-by-n matrix stored as `[row0, row1]` where each row has `n` elements.
/// The Jacobian relates joint velocities to end-effector velocity.
fn jacobian(link_lengths: &[f64], joint_angles: &[f64]) -> Vec<Vec<f64>> {
    let n = link_lengths.len();
    let mut j = vec![vec![0.0; n]; 2];
    for i in 0..n {
        for k in i..n {
            let cumulative_angle: f64 = joint_angles[..=k].iter().sum();
            j[0][i] -= link_lengths[k] * cumulative_angle.sin();
            j[1][i] += link_lengths[k] * cumulative_angle.cos();
        }
    }
    j
}

/// Computes the pseudo-inverse of a 2-by-n matrix using the formula:
/// `J^+ = J^T (J J^T)^{-1}`
fn pseudo_inverse_2xn(j: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = j[0].len();

    // Compute J * J^T (2x2 matrix)
    let mut jjt = [[0.0f64; 2]; 2];
    for row in 0..2 {
        for col in 0..2 {
            for k in 0..n {
                jjt[row][col] += j[row][k] * j[col][k];
            }
        }
    }

    // Invert the 2x2 matrix
    let det = jjt[0][0] * jjt[1][1] - jjt[0][1] * jjt[1][0];
    if det.abs() < 1e-12 {
        // Near-singular: return zero matrix (damped least squares fallback)
        return vec![vec![0.0; 2]; n];
    }
    let inv_det = 1.0 / det;
    let inv = [
        [jjt[1][1] * inv_det, -jjt[0][1] * inv_det],
        [-jjt[1][0] * inv_det, jjt[0][0] * inv_det],
    ];

    // J^T * (J J^T)^{-1}  => n x 2
    let mut result = vec![vec![0.0; 2]; n];
    for i in 0..n {
        for col in 0..2 {
            for k in 0..2 {
                result[i][col] += j[k][i] * inv[k][col];
            }
        }
    }
    result
}

/// Computes the distance and error vector from current position to goal.
pub fn distance_to_goal(current: &[f64; 2], goal: &[f64; 2]) -> ([f64; 2], f64) {
    let dx = goal[0] - current[0];
    let dy = goal[1] - current[1];
    ([dx, dy], (dx * dx + dy * dy).sqrt())
}

/// Normalizes an angle to the range \[-pi, pi\].
pub fn angle_mod(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Computes element-wise angle difference, each wrapped to \[-pi, pi\].
fn ang_diff(theta1: &[f64], theta2: &[f64]) -> Vec<f64> {
    theta1
        .iter()
        .zip(theta2.iter())
        .map(|(a, b)| angle_mod(a - b))
        .collect()
}

/// Solves inverse kinematics using the Jacobian pseudo-inverse method.
///
/// Returns `Some(joint_angles)` if a solution is found within tolerance,
/// or `None` if the maximum number of iterations is exceeded.
pub fn inverse_kinematics(
    link_lengths: &[f64],
    initial_joint_angles: &[f64],
    goal_pos: &[f64; 2],
) -> Option<Vec<f64>> {
    inverse_kinematics_with_params(
        link_lengths,
        initial_joint_angles,
        goal_pos,
        DEFAULT_N_ITERATIONS,
        DEFAULT_GOAL_THRESHOLD,
    )
}

/// Solves inverse kinematics with configurable iteration limit and threshold.
pub fn inverse_kinematics_with_params(
    link_lengths: &[f64],
    initial_joint_angles: &[f64],
    goal_pos: &[f64; 2],
    max_iterations: usize,
    goal_threshold: f64,
) -> Option<Vec<f64>> {
    let mut joint_angles = initial_joint_angles.to_vec();

    for _iteration in 0..max_iterations {
        let current_pos = forward_kinematics(link_lengths, &joint_angles);
        let (errors, distance) = distance_to_goal(&current_pos, goal_pos);
        if distance < goal_threshold {
            return Some(joint_angles);
        }

        let j = jacobian(link_lengths, &joint_angles);
        let j_pinv = pseudo_inverse_2xn(&j);

        // joint_angles += J_pinv * errors
        for i in 0..joint_angles.len() {
            joint_angles[i] += j_pinv[i][0] * errors[0] + j_pinv[i][1] * errors[1];
        }
    }
    None
}

/// Runs the proportional-control IK loop: smoothly moves from current angles
/// toward goal angles.
///
/// Returns the trajectory of joint angle snapshots.
pub fn move_to_goal(
    arm: &mut NLinkArm,
    goal_angles: &[f64],
    goal_pos: &[f64; 2],
    goal_threshold: f64,
) -> Vec<Vec<f64>> {
    let mut trajectory = Vec::new();

    for _ in 0..10000 {
        let (_, distance) = distance_to_goal(&arm.end_effector, goal_pos);
        if distance < goal_threshold {
            break;
        }
        let diff = ang_diff(goal_angles, &arm.joint_angles);
        let new_angles: Vec<f64> = arm
            .joint_angles
            .iter()
            .zip(diff.iter())
            .map(|(a, d)| a + KP * d * DT)
            .collect();
        arm.update_joints(&new_angles);
        trajectory.push(arm.joint_angles.clone());
    }
    trajectory
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_4;

    #[test]
    fn test_forward_kinematics_straight() {
        let links = vec![1.0, 1.0, 1.0];
        let angles = vec![0.0, 0.0, 0.0];
        let [x, y] = forward_kinematics(&links, &angles);
        assert!((x - 3.0).abs() < 1e-10);
        assert!(y.abs() < 1e-10);
    }

    #[test]
    fn test_forward_kinematics_bent() {
        let links = vec![1.0, 1.0];
        let angles = vec![0.0, FRAC_PI_4];
        let [x, y] = forward_kinematics(&links, &angles);
        // link1 at angle 0: (1, 0)
        // link2 at cumulative angle pi/4: (1 + cos(pi/4), sin(pi/4))
        let expected_x = 1.0 + (FRAC_PI_4).cos();
        let expected_y = (FRAC_PI_4).sin();
        assert!((x - expected_x).abs() < 1e-10);
        assert!((y - expected_y).abs() < 1e-10);
    }

    #[test]
    fn test_nlink_arm_new() {
        let arm = NLinkArm::new(vec![1.0, 1.0], vec![0.0, 0.0]);
        assert_eq!(arm.n_links, 2);
        assert!((arm.end_effector[0] - 2.0).abs() < 1e-10);
        assert!(arm.end_effector[1].abs() < 1e-10);
        assert_eq!(arm.points.len(), 3);
    }

    #[test]
    fn test_nlink_arm_update_joints() {
        let mut arm = NLinkArm::new(vec![1.0, 1.0], vec![0.0, 0.0]);
        arm.update_joints(&[PI / 2.0, 0.0]);
        // Both links point up
        assert!(arm.end_effector[0].abs() < 1e-10);
        assert!((arm.end_effector[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_inverse_kinematics_reachable() {
        // Use a 2-link arm with initial angles that break symmetry
        let links = vec![1.0, 1.0];
        let initial = vec![0.3, 0.3]; // avoid singular straight config
        let goal = [1.0, 1.0]; // well within reach (max reach = 2.0)

        let result = inverse_kinematics(&links, &initial, &goal);
        assert!(result.is_some(), "IK should converge for reachable target");

        let angles = result.unwrap();
        let pos = forward_kinematics(&links, &angles);
        let dist = ((pos[0] - goal[0]).powi(2) + (pos[1] - goal[1]).powi(2)).sqrt();
        assert!(dist < DEFAULT_GOAL_THRESHOLD);
    }

    #[test]
    fn test_inverse_kinematics_unreachable() {
        let links = vec![1.0, 1.0];
        let initial = vec![0.0, 0.0];
        // Way outside workspace radius of 2.0
        let goal = [10.0, 10.0];

        let result = inverse_kinematics_with_params(&links, &initial, &goal, 500, 0.01);
        assert!(result.is_none());
    }

    #[test]
    fn test_angle_mod() {
        assert!((angle_mod(0.0)).abs() < 1e-10);
        assert!((angle_mod(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((angle_mod(PI + 0.1) - (-PI + 0.1)).abs() < 1e-10);
        assert!((angle_mod(-PI - 0.1) - (PI - 0.1)).abs() < 1e-10);
    }

    #[test]
    fn test_distance_to_goal() {
        let (errors, dist) = distance_to_goal(&[0.0, 0.0], &[3.0, 4.0]);
        assert!((errors[0] - 3.0).abs() < 1e-10);
        assert!((errors[1] - 4.0).abs() < 1e-10);
        assert!((dist - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_jacobian_shape() {
        let links = vec![1.0, 1.0, 1.0];
        let angles = vec![0.0, 0.0, 0.0];
        let j = jacobian(&links, &angles);
        assert_eq!(j.len(), 2);
        assert_eq!(j[0].len(), 3);
        assert_eq!(j[1].len(), 3);
    }

    #[test]
    fn test_move_to_goal_converges() {
        let mut arm = NLinkArm::new(vec![1.0, 1.0], vec![0.5, 0.5]);
        let goal = [1.5, 0.5];
        let goal_angles =
            inverse_kinematics(&arm.link_lengths.clone(), &arm.joint_angles.clone(), &goal)
                .unwrap();
        let traj = move_to_goal(&mut arm, &goal_angles, &goal, 0.1);
        assert!(!traj.is_empty());
        let (_, dist) = distance_to_goal(&arm.end_effector, &goal);
        assert!(dist < 0.2);
    }

    #[test]
    fn test_10_link_arm() {
        let n = 10;
        let links = vec![0.5; n]; // total reach = 5.0
        let initial = vec![0.1; n]; // slight spread to avoid singular configuration
        let goal = [3.0, 1.0]; // well within reach

        let result = inverse_kinematics(&links, &initial, &goal);
        assert!(result.is_some(), "10-link IK should converge");

        let angles = result.unwrap();
        let pos = forward_kinematics(&links, &angles);
        let dist = ((pos[0] - goal[0]).powi(2) + (pos[1] - goal[1]).powi(2)).sqrt();
        assert!(dist < DEFAULT_GOAL_THRESHOLD);
    }

    #[test]
    fn test_workspace_radius() {
        let arm = NLinkArm::new(vec![1.0, 2.0, 0.5], vec![0.0, 0.0, 0.0]);
        assert!((arm.workspace_radius() - 3.5).abs() < 1e-10);
    }

    #[test]
    fn test_pseudo_inverse_identity_like() {
        // For a 2x2 non-singular Jacobian, pinv should be close to the true inverse
        let j = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let pinv = pseudo_inverse_2xn(&j);
        assert!((pinv[0][0] - 1.0).abs() < 1e-10);
        assert!((pinv[0][1] - 0.0).abs() < 1e-10);
        assert!((pinv[1][0] - 0.0).abs() < 1e-10);
        assert!((pinv[1][1] - 1.0).abs() < 1e-10);
    }
}
