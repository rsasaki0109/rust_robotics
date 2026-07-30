//! SE(3) pose-graph optimization on the shared factor-graph backend.

use nalgebra::{DMatrix, DVector, Matrix4};
use rust_robotics_core::{se3_adjoint, se3_exp, se3_inverse, se3_log, skew, Matrix6, Vector6};
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
        linear_solver: config.linear_solver,
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
        let left_inverse = se3_left_jacobian_inverse(&residual);
        let right_inverse = se3_left_jacobian_inverse(&(-residual));
        let jacobians = vec![
            DMatrix::from_column_slice(
                6,
                6,
                (-left_inverse * se3_adjoint(&se3_inverse(&self.measurement))).as_slice(),
            ),
            DMatrix::from_column_slice(6, 6, right_inverse.as_slice()),
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

fn se3_left_jacobian_inverse(tangent: &Vector6) -> Matrix6 {
    let velocity = tangent.fixed_rows::<3>(0).into_owned();
    let omega = tangent.fixed_rows::<3>(3).into_owned();
    let mut ad = Matrix6::zeros();
    ad.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew(&omega));
    ad.fixed_view_mut::<3, 3>(0, 3).copy_from(&skew(&velocity));
    ad.fixed_view_mut::<3, 3>(3, 3).copy_from(&skew(&omega));

    // J_l(x) = sum ad(x)^n / (n + 1)!. Evaluating the matrix series and
    // inverting it avoids finite-differencing the nonlinear factor while
    // remaining stable at the identity.
    let mut jacobian = Matrix6::identity();
    let mut power = Matrix6::identity();
    let mut factorial = 1.0;
    for order in 1..=24 {
        power *= ad;
        factorial *= (order + 1) as f64;
        jacobian += power / factorial;
    }
    jacobian
        .try_inverse()
        .expect("the SE(3) left Jacobian is nonsingular on the principal branch")
}

#[cfg(test)]
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

    #[test]
    fn analytic_se3_jacobians_match_finite_differences() {
        let from = se3_exp(&Vector6::new(0.4, -0.2, 0.1, 0.2, -0.1, 0.3));
        let to = se3_exp(&Vector6::new(1.1, 0.5, -0.3, -0.1, 0.25, 0.15));
        let measurement = se3_exp(&Vector6::new(0.6, 0.4, -0.2, 0.1, 0.2, -0.1));
        let residual = edge_residual(&from, &to, &measurement);
        let analytic_from =
            -se3_left_jacobian_inverse(&residual) * se3_adjoint(&se3_inverse(&measurement));
        let analytic_to = se3_left_jacobian_inverse(&(-residual));
        let numeric_from = numerical_jacobian(&from, &to, &measurement, true);
        let numeric_to = numerical_jacobian(&from, &to, &measurement, false);
        assert!((analytic_from - numeric_from).norm() < 1.0e-7);
        assert!((analytic_to - numeric_to).norm() < 1.0e-7);
    }
}
