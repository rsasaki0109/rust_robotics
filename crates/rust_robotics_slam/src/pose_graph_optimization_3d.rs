//! SE(3) pose-graph optimization on the shared factor-graph backend.

use nalgebra::{DMatrix, DVector, Matrix4};
use rust_robotics_core::{se3_exp, se3_inverse, se3_log, Matrix6, Vector6};
use rust_robotics_optimization::{
    solve, Factor, FactorEvaluation, OptimizationResult, Problem, SolverConfig, SolverMethod,
    TerminationReason, Variable, VariableId,
};

use crate::pose_graph_optimization::PoseGraphConfig;

/// A 3D pose represented by a homogeneous SE(3) matrix.
#[derive(Debug, Clone, PartialEq)]
pub struct Pose3DNode {
    pub transform: Matrix4<f64>,
}

impl Pose3DNode {
    pub fn identity() -> Self {
        Self {
            transform: Matrix4::identity(),
        }
    }

    pub fn from_tangent(tangent: &Vector6) -> Self {
        Self {
            transform: se3_exp(tangent),
        }
    }

    pub fn tangent(&self) -> Vector6 {
        se3_log(&self.transform)
    }
}

/// A relative SE(3) constraint between two pose nodes.
#[derive(Debug, Clone)]
pub struct Edge3D {
    pub from: usize,
    pub to: usize,
    pub measurement: Pose3DNode,
    pub information: Matrix6,
}

/// Result of an SE(3) pose-graph solve.
#[derive(Debug, Clone)]
pub struct PoseGraphResult3D {
    pub poses: Vec<Pose3DNode>,
    pub iterations: usize,
    pub converged: bool,
}

/// Optimizes an SE(3) pose graph, fixing the first pose as the gauge anchor.
pub fn optimize_pose_graph_3d(
    initial_poses: &[Pose3DNode],
    edges: &[Edge3D],
    config: &PoseGraphConfig,
) -> PoseGraphResult3D {
    if initial_poses.len() <= 1 || edges.is_empty() {
        return PoseGraphResult3D {
            poses: initial_poses.to_vec(),
            iterations: 0,
            converged: true,
        };
    }

    let mut problem = Problem::new();
    let ids = initial_poses
        .iter()
        .map(|pose| {
            problem.add_variable(
                Variable::with_retraction(
                    DVector::from_column_slice(pose.tangent().as_slice()),
                    6,
                    retract_pose_3d,
                )
                .expect("SE(3) dimensions are valid"),
            )
        })
        .collect::<Vec<_>>();
    problem
        .variable_mut(ids[0])
        .expect("first pose exists")
        .set_fixed(true);

    for edge in edges {
        if edge.from < initial_poses.len() && edge.to < initial_poses.len() {
            let _ = problem.add_factor(PoseGraphFactor3D {
                variables: [ids[edge.from], ids[edge.to]],
                measurement: edge.measurement.transform,
                information: edge.information,
            });
        }
    }

    let solver_config = SolverConfig {
        method: SolverMethod::LevenbergMarquardt,
        max_iterations: config.max_iterations.max(1),
        gradient_tolerance: config.tolerance,
        step_tolerance: config.tolerance,
        cost_tolerance: config.tolerance * config.tolerance,
        ..SolverConfig::default()
    };
    let summary = solve(&mut problem, &solver_config).ok();
    let poses = ids
        .iter()
        .map(|id| {
            let value = problem.variable(*id).expect("pose exists").value();
            Pose3DNode::from_tangent(&Vector6::from_column_slice(value.as_slice()))
        })
        .collect();

    PoseGraphResult3D {
        poses,
        iterations: summary.as_ref().map_or(0, |result| result.iterations),
        converged: summary
            .is_some_and(|result| result.termination != TerminationReason::MaxIterations),
    }
}

struct PoseGraphFactor3D {
    variables: [VariableId; 2],
    measurement: Matrix4<f64>,
    information: Matrix6,
}

impl Factor for PoseGraphFactor3D {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        let from = se3_exp(&Vector6::from_column_slice(values[0].as_slice()));
        let to = se3_exp(&Vector6::from_column_slice(values[1].as_slice()));
        let residual = edge_residual(&from, &to, &self.measurement);
        let jacobians = vec![
            numerical_jacobian(&from, &to, &self.measurement, true),
            numerical_jacobian(&from, &to, &self.measurement, false),
        ];
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice(residual.as_slice()),
            information: DMatrix::from_column_slice(6, 6, self.information.as_slice()),
            jacobians,
        })
    }
}

fn edge_residual(from: &Matrix4<f64>, to: &Matrix4<f64>, measurement: &Matrix4<f64>) -> Vector6 {
    se3_log(&(se3_inverse(measurement) * se3_inverse(from) * to))
}

fn numerical_jacobian(
    from: &Matrix4<f64>,
    to: &Matrix4<f64>,
    measurement: &Matrix4<f64>,
    perturb_from: bool,
) -> DMatrix<f64> {
    let epsilon = 1.0e-6;
    let mut jacobian = DMatrix::zeros(6, 6);
    for column in 0..6 {
        let mut delta = Vector6::zeros();
        delta[column] = epsilon;
        let increment = se3_exp(&delta);
        let decrement = se3_exp(&(-delta));
        let (plus, minus) = if perturb_from {
            (
                edge_residual(&(from * increment), to, measurement),
                edge_residual(&(from * decrement), to, measurement),
            )
        } else {
            (
                edge_residual(from, &(to * increment), measurement),
                edge_residual(from, &(to * decrement), measurement),
            )
        };
        jacobian.set_column(column, &((plus - minus) / (2.0 * epsilon)));
    }
    jacobian
}

fn retract_pose_3d(
    value: &DVector<f64>,
    increment: &DVector<f64>,
) -> OptimizationResult<DVector<f64>> {
    let pose = se3_exp(&Vector6::from_column_slice(value.as_slice()));
    let delta = se3_exp(&Vector6::from_column_slice(increment.as_slice()));
    Ok(DVector::from_column_slice(
        se3_log(&(pose * delta)).as_slice(),
    ))
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    fn relative(from: &Pose3DNode, to: &Pose3DNode) -> Pose3DNode {
        Pose3DNode {
            transform: se3_inverse(&from.transform) * to.transform,
        }
    }

    #[test]
    fn se3_triangle_converges() {
        let truth = [
            Pose3DNode::identity(),
            Pose3DNode::from_tangent(&Vector6::new(1.0, 0.0, 0.0, 0.1, 0.0, 0.0)),
            Pose3DNode::from_tangent(&Vector6::new(1.0, 1.0, 0.2, 0.1, -0.2, 0.3)),
        ];
        let initial = vec![
            truth[0].clone(),
            Pose3DNode::from_tangent(&Vector6::new(1.1, -0.1, 0.1, 0.15, 0.02, -0.03)),
            Pose3DNode::from_tangent(&Vector6::new(0.8, 1.2, 0.1, 0.05, -0.1, 0.4)),
        ];
        let edges = [(0, 1), (1, 2), (0, 2)]
            .into_iter()
            .map(|(from, to)| Edge3D {
                from,
                to,
                measurement: relative(&truth[from], &truth[to]),
                information: Matrix6::identity(),
            })
            .collect::<Vec<_>>();

        let result = optimize_pose_graph_3d(&initial, &edges, &PoseGraphConfig::default());
        assert!(result.converged);
        for (expected, actual) in truth.iter().zip(result.poses.iter()) {
            let error = se3_log(&(se3_inverse(&expected.transform) * actual.transform));
            assert!(error.fixed_rows::<3>(0).norm() < 1.0e-5);
            assert!(Vector3::from(error.fixed_rows::<3>(3)).norm() < 1.0e-5);
        }
    }
}
