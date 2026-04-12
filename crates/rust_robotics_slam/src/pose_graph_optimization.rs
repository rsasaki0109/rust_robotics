//! 2D pose graph optimization with Gauss-Newton iteration.
//!
//! The optimizer refines a set of initial poses from relative pose
//! constraints represented as edges in an SE(2) graph.
//!
//! Reference:
//! - Giorgio Grisetti, Rainer Kümmerle, Cyrill Stachniss, Wolfram Burgard,
//!   "A Tutorial on Graph-Based SLAM":
//!   <https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti10titsmag.pdf>

use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, Vector2, Vector3};
use std::f64::consts::PI;

/// Configuration for pose graph optimization.
#[derive(Debug, Clone, Copy)]
pub struct PoseGraphConfig {
    /// Maximum number of Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Convergence threshold on the update vector norm.
    pub tolerance: f64,
}

impl Default for PoseGraphConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            tolerance: 1.0e-5,
        }
    }
}

/// A 2D pose node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2DNode {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2DNode {
    /// Creates a pose node.
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }
}

/// A relative pose constraint between two nodes.
#[derive(Debug, Clone)]
pub struct Edge2D {
    pub from: usize,
    pub to: usize,
    pub measurement: Pose2DNode,
    pub information: Matrix3<f64>,
}

/// Optimization result.
#[derive(Debug, Clone)]
pub struct PoseGraphResult {
    pub poses: Vec<Pose2DNode>,
    pub iterations: usize,
    pub converged: bool,
}

/// Optimizes a 2D pose graph using Gauss-Newton iteration.
pub fn optimize_pose_graph(
    initial_poses: &[Pose2DNode],
    edges: &[Edge2D],
    config: &PoseGraphConfig,
) -> PoseGraphResult {
    let mut poses = initial_poses.to_vec();

    if poses.len() <= 1 || edges.is_empty() {
        return PoseGraphResult {
            poses,
            iterations: 0,
            converged: true,
        };
    }

    let state_size = 3 * poses.len();
    let anchor_weight = 1.0e6;
    let mut converged = false;
    let mut iterations = 0;

    for iter in 0..config.max_iterations {
        iterations = iter + 1;
        let mut h = DMatrix::<f64>::zeros(state_size, state_size);
        let mut b = DVector::<f64>::zeros(state_size);

        for edge in edges {
            if edge.from >= poses.len() || edge.to >= poses.len() {
                continue;
            }

            let (error, j_i, j_j) =
                edge_error_and_jacobians(&poses[edge.from], &poses[edge.to], &edge.measurement);

            accumulate_block(&mut h, &mut b, edge, &error, &j_i, &j_j);
        }

        for index in 0..3 {
            h[(index, index)] += anchor_weight;
        }

        let rhs = -&b;
        let Some(delta) = h.lu().solve(&rhs) else {
            return PoseGraphResult {
                poses,
                iterations,
                converged: false,
            };
        };

        for (node_index, pose) in poses.iter_mut().enumerate() {
            let base = 3 * node_index;
            pose.x += delta[base];
            pose.y += delta[base + 1];
            pose.yaw = normalize_angle(pose.yaw + delta[base + 2]);
        }

        if delta.norm() < config.tolerance {
            converged = true;
            break;
        }
    }

    PoseGraphResult {
        poses,
        iterations,
        converged,
    }
}

fn accumulate_block(
    h: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    edge: &Edge2D,
    error: &Vector3<f64>,
    j_i: &Matrix3<f64>,
    j_j: &Matrix3<f64>,
) {
    let from_offset = 3 * edge.from;
    let to_offset = 3 * edge.to;

    let h_ii = j_i.transpose() * edge.information * j_i;
    let h_ij = j_i.transpose() * edge.information * j_j;
    let h_ji = j_j.transpose() * edge.information * j_i;
    let h_jj = j_j.transpose() * edge.information * j_j;

    let b_i = j_i.transpose() * edge.information * error;
    let b_j = j_j.transpose() * edge.information * error;

    for row in 0..3 {
        b[from_offset + row] += b_i[row];
        b[to_offset + row] += b_j[row];
        for col in 0..3 {
            h[(from_offset + row, from_offset + col)] += h_ii[(row, col)];
            h[(from_offset + row, to_offset + col)] += h_ij[(row, col)];
            h[(to_offset + row, from_offset + col)] += h_ji[(row, col)];
            h[(to_offset + row, to_offset + col)] += h_jj[(row, col)];
        }
    }
}

fn edge_error_and_jacobians(
    from: &Pose2DNode,
    to: &Pose2DNode,
    measurement: &Pose2DNode,
) -> (Vector3<f64>, Matrix3<f64>, Matrix3<f64>) {
    let t_i = Vector2::new(from.x, from.y);
    let t_j = Vector2::new(to.x, to.y);
    let t_ij = Vector2::new(measurement.x, measurement.y);

    let r_i_t = rotation_matrix(from.yaw).transpose();
    let r_ij_t = rotation_matrix(measurement.yaw).transpose();
    let delta_t = t_j - t_i;

    let translational_error = r_ij_t * (r_i_t * delta_t - t_ij);
    let error = Vector3::new(
        translational_error[0],
        translational_error[1],
        normalize_angle(to.yaw - from.yaw - measurement.yaw),
    );

    let dr_i_t = rotation_matrix_transpose_derivative(from.yaw);
    let a_translation = -r_ij_t * r_i_t;
    let a_heading = r_ij_t * (dr_i_t * delta_t);
    let b_translation = r_ij_t * r_i_t;

    let j_i = Matrix3::new(
        a_translation[(0, 0)],
        a_translation[(0, 1)],
        a_heading[0],
        a_translation[(1, 0)],
        a_translation[(1, 1)],
        a_heading[1],
        0.0,
        0.0,
        -1.0,
    );
    let j_j = Matrix3::new(
        b_translation[(0, 0)],
        b_translation[(0, 1)],
        0.0,
        b_translation[(1, 0)],
        b_translation[(1, 1)],
        0.0,
        0.0,
        0.0,
        1.0,
    );

    (error, j_i, j_j)
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

#[cfg(test)]
fn relative_transform(from: &Pose2DNode, to: &Pose2DNode) -> Pose2DNode {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let cos_yaw = from.yaw.cos();
    let sin_yaw = from.yaw.sin();

    Pose2DNode {
        x: cos_yaw * dx + sin_yaw * dy,
        y: -sin_yaw * dx + cos_yaw * dy,
        yaw: normalize_angle(to.yaw - from.yaw),
    }
}

fn rotation_matrix(yaw: f64) -> Matrix2<f64> {
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();
    Matrix2::new(cos_yaw, -sin_yaw, sin_yaw, cos_yaw)
}

fn rotation_matrix_transpose_derivative(yaw: f64) -> Matrix2<f64> {
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();
    Matrix2::new(-sin_yaw, cos_yaw, -cos_yaw, -sin_yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge_from_truth(poses: &[Pose2DNode], from: usize, to: usize) -> Edge2D {
        Edge2D {
            from,
            to,
            measurement: relative_transform(&poses[from], &poses[to]),
            information: Matrix3::identity(),
        }
    }

    fn perturb_pose(pose: &Pose2DNode, axis: usize, delta: f64) -> Pose2DNode {
        let mut perturbed = *pose;
        match axis {
            0 => perturbed.x += delta,
            1 => perturbed.y += delta,
            2 => perturbed.yaw = normalize_angle(perturbed.yaw + delta),
            _ => unreachable!("axis out of bounds"),
        }
        perturbed
    }

    fn error_difference(a: &Vector3<f64>, b: &Vector3<f64>) -> Vector3<f64> {
        Vector3::new(a[0] - b[0], a[1] - b[1], normalize_angle(a[2] - b[2]))
    }

    fn numerical_jacobian(
        from: &Pose2DNode,
        to: &Pose2DNode,
        measurement: &Pose2DNode,
        perturb_from: bool,
    ) -> Matrix3<f64> {
        let epsilon = 1.0e-6;
        let mut jacobian = Matrix3::zeros();

        for axis in 0..3 {
            let (from_plus, to_plus, from_minus, to_minus) = if perturb_from {
                (
                    perturb_pose(from, axis, epsilon),
                    *to,
                    perturb_pose(from, axis, -epsilon),
                    *to,
                )
            } else {
                (
                    *from,
                    perturb_pose(to, axis, epsilon),
                    *from,
                    perturb_pose(to, axis, -epsilon),
                )
            };

            let error_plus = edge_error_and_jacobians(&from_plus, &to_plus, measurement).0;
            let error_minus = edge_error_and_jacobians(&from_minus, &to_minus, measurement).0;
            let column = error_difference(&error_plus, &error_minus) / (2.0 * epsilon);
            jacobian.set_column(axis, &column);
        }

        jacobian
    }

    #[test]
    fn test_pose_graph_config_defaults() {
        let config = PoseGraphConfig::default();
        assert_eq!(config.max_iterations, 20);
        assert_eq!(config.tolerance, 1.0e-5);
    }

    #[test]
    fn test_pose_graph_identity_solution_stays_unchanged() {
        let poses = vec![
            Pose2DNode::new(0.0, 0.0, 0.0),
            Pose2DNode::new(1.0, 0.0, 0.0),
            Pose2DNode::new(1.0, 1.0, PI / 2.0),
        ];
        let edges = vec![
            edge_from_truth(&poses, 0, 1),
            edge_from_truth(&poses, 1, 2),
            edge_from_truth(&poses, 0, 2),
        ];

        let result = optimize_pose_graph(&poses, &edges, &PoseGraphConfig::default());

        assert!(result.converged);
        for (expected, actual) in poses.iter().zip(result.poses.iter()) {
            assert!((expected.x - actual.x).abs() < 1.0e-9);
            assert!((expected.y - actual.y).abs() < 1.0e-9);
            assert!(normalize_angle(expected.yaw - actual.yaw).abs() < 1.0e-9);
        }
    }

    #[test]
    fn test_pose_graph_triangle_optimization_converges() {
        let truth = vec![
            Pose2DNode::new(0.0, 0.0, 0.0),
            Pose2DNode::new(1.0, 0.0, 0.0),
            Pose2DNode::new(1.0, 1.0, PI / 2.0),
        ];
        let initial = vec![
            Pose2DNode::new(0.0, 0.0, 0.0),
            Pose2DNode::new(1.15, -0.1, 0.08),
            Pose2DNode::new(0.85, 1.2, PI / 2.0 - 0.12),
        ];
        let edges = vec![
            edge_from_truth(&truth, 0, 1),
            edge_from_truth(&truth, 1, 2),
            edge_from_truth(&truth, 0, 2),
        ];

        let result = optimize_pose_graph(&initial, &edges, &PoseGraphConfig::default());

        assert!(result.converged);
        for (expected, actual) in truth.iter().zip(result.poses.iter()) {
            assert!((expected.x - actual.x).abs() < 5.0e-2);
            assert!((expected.y - actual.y).abs() < 5.0e-2);
            assert!(normalize_angle(expected.yaw - actual.yaw).abs() < 5.0e-2);
        }
    }

    #[test]
    fn test_pose_graph_jacobians_match_finite_difference() {
        let from = Pose2DNode::new(0.3, -0.4, 0.35);
        let to = Pose2DNode::new(1.1, 0.7, -0.25);
        let measurement = Pose2DNode::new(0.9, 0.8, -0.5);

        let (_, j_i, j_j) = edge_error_and_jacobians(&from, &to, &measurement);
        let numerical_i = numerical_jacobian(&from, &to, &measurement, true);
        let numerical_j = numerical_jacobian(&from, &to, &measurement, false);

        for row in 0..3 {
            for col in 0..3 {
                assert!((j_i[(row, col)] - numerical_i[(row, col)]).abs() < 1.0e-5);
                assert!((j_j[(row, col)] - numerical_j[(row, col)]).abs() < 1.0e-5);
            }
        }
    }
}
