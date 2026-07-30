//! Point-to-line and point-to-plane ICP using the shared robust optimizer.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3};
use rust_robotics_core::{
    se2_exp, se2_inverse, se2_log, se3_exp, se3_inverse, se3_log, skew, Vector6,
};
use rust_robotics_optimization::{
    solve, Factor, FactorEvaluation, OptimizationResult, Problem, RobustKernel, SolverConfig,
    Variable, VariableId,
};

/// Shared outer-loop and robust-loss settings.
#[derive(Debug, Clone, Copy)]
pub struct GeometricIcpConfig {
    pub max_outer_iterations: usize,
    pub max_inner_iterations: usize,
    pub correspondence_distance: f64,
    pub tolerance: f64,
    pub robust_kernel: RobustKernel,
}

impl Default for GeometricIcpConfig {
    fn default() -> Self {
        Self {
            max_outer_iterations: 20,
            max_inner_iterations: 10,
            correspondence_distance: 1.0,
            tolerance: 1.0e-7,
            robust_kernel: RobustKernel::Huber { delta: 0.1 },
        }
    }
}

#[derive(Debug, Clone)]
pub struct PointToLineIcpResult {
    pub transform: Matrix3<f64>,
    pub iterations: usize,
    pub mean_absolute_error: f64,
    pub converged: bool,
}

#[derive(Debug, Clone)]
pub struct PointToPlaneIcpResult {
    pub transform: Matrix4<f64>,
    pub iterations: usize,
    pub mean_absolute_error: f64,
    pub converged: bool,
}

/// Aligns 2D source points to target tangent lines.
pub fn point_to_line_icp_2d(
    source: &[Vector2<f64>],
    target: &[Vector2<f64>],
    target_normals: &[Vector2<f64>],
    initial: Matrix3<f64>,
    config: &GeometricIcpConfig,
) -> PointToLineIcpResult {
    if target.len() != target_normals.len() || source.is_empty() || target.is_empty() {
        return PointToLineIcpResult {
            transform: initial,
            iterations: 0,
            mean_absolute_error: f64::INFINITY,
            converged: false,
        };
    }
    let mut transform = initial;
    let mut last_error = f64::INFINITY;
    for iteration in 0..config.max_outer_iterations {
        let correspondences = source
            .iter()
            .filter_map(|source_point| {
                let transformed = transform.fixed_view::<2, 2>(0, 0) * source_point
                    + transform.fixed_view::<2, 1>(0, 2);
                nearest_2d(&transformed, target).and_then(|(index, distance)| {
                    (distance <= config.correspondence_distance).then_some((
                        *source_point,
                        target[index],
                        target_normals[index],
                    ))
                })
            })
            .collect::<Vec<_>>();
        if correspondences.len() < 2 {
            break;
        }
        let previous = transform;
        let mut problem = Problem::new();
        let pose = problem.add_variable(
            Variable::with_retraction(
                DVector::from_column_slice(se2_log(&transform).as_slice()),
                3,
                retract_se2,
            )
            .expect("SE(2) dimensions are valid"),
        );
        for (source, target, normal) in &correspondences {
            let _ = problem.add_factor(PointToLineFactor {
                variables: [pose],
                source: *source,
                target: *target,
                normal: normal.normalize(),
                kernel: config.robust_kernel,
            });
        }
        let _ = solve(
            &mut problem,
            &SolverConfig {
                max_iterations: config.max_inner_iterations.max(1),
                step_tolerance: config.tolerance,
                gradient_tolerance: config.tolerance,
                ..SolverConfig::default()
            },
        );
        let value = problem.variable(pose).expect("pose exists").value();
        transform = se2_exp(&Vector3::from_column_slice(value.as_slice()));
        let error = correspondences
            .iter()
            .map(|(source, target, normal)| {
                let transformed = transform.fixed_view::<2, 2>(0, 0) * source
                    + transform.fixed_view::<2, 1>(0, 2);
                normal.dot(&(transformed - target)).abs()
            })
            .sum::<f64>()
            / correspondences.len() as f64;
        let step = se2_log(&(se2_inverse(&previous) * transform)).norm();
        if step <= config.tolerance || (last_error - error).abs() <= config.tolerance {
            return PointToLineIcpResult {
                transform,
                iterations: iteration + 1,
                mean_absolute_error: error,
                converged: true,
            };
        }
        last_error = error;
    }
    PointToLineIcpResult {
        transform,
        iterations: config.max_outer_iterations,
        mean_absolute_error: last_error,
        converged: false,
    }
}

/// Aligns 3D source points to target tangent planes.
pub fn point_to_plane_icp_3d(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    target_normals: &[Vector3<f64>],
    initial: Matrix4<f64>,
    config: &GeometricIcpConfig,
) -> PointToPlaneIcpResult {
    if target.len() != target_normals.len() || source.is_empty() || target.is_empty() {
        return PointToPlaneIcpResult {
            transform: initial,
            iterations: 0,
            mean_absolute_error: f64::INFINITY,
            converged: false,
        };
    }
    let mut transform = initial;
    let mut last_error = f64::INFINITY;
    for iteration in 0..config.max_outer_iterations {
        let correspondences = source
            .iter()
            .filter_map(|source_point| {
                let transformed = transform.fixed_view::<3, 3>(0, 0) * source_point
                    + transform.fixed_view::<3, 1>(0, 3);
                nearest_3d(&transformed, target).and_then(|(index, distance)| {
                    (distance <= config.correspondence_distance).then_some((
                        *source_point,
                        target[index],
                        target_normals[index],
                    ))
                })
            })
            .collect::<Vec<_>>();
        if correspondences.len() < 3 {
            break;
        }
        let previous = transform;
        let mut problem = Problem::new();
        let pose = problem.add_variable(
            Variable::with_retraction(
                DVector::from_column_slice(se3_log(&transform).as_slice()),
                6,
                retract_se3,
            )
            .expect("SE(3) dimensions are valid"),
        );
        for (source, target, normal) in &correspondences {
            let _ = problem.add_factor(PointToPlaneFactor {
                variables: [pose],
                source: *source,
                target: *target,
                normal: normal.normalize(),
                kernel: config.robust_kernel,
            });
        }
        let _ = solve(
            &mut problem,
            &SolverConfig {
                max_iterations: config.max_inner_iterations.max(1),
                step_tolerance: config.tolerance,
                gradient_tolerance: config.tolerance,
                ..SolverConfig::default()
            },
        );
        let value = problem.variable(pose).expect("pose exists").value();
        transform = se3_exp(&Vector6::from_column_slice(value.as_slice()));
        let error = correspondences
            .iter()
            .map(|(source, target, normal)| {
                let transformed = transform.fixed_view::<3, 3>(0, 0) * source
                    + transform.fixed_view::<3, 1>(0, 3);
                normal.dot(&(transformed - target)).abs()
            })
            .sum::<f64>()
            / correspondences.len() as f64;
        let step = se3_log(&(se3_inverse(&previous) * transform)).norm();
        if step <= config.tolerance || (last_error - error).abs() <= config.tolerance {
            return PointToPlaneIcpResult {
                transform,
                iterations: iteration + 1,
                mean_absolute_error: error,
                converged: true,
            };
        }
        last_error = error;
    }
    PointToPlaneIcpResult {
        transform,
        iterations: config.max_outer_iterations,
        mean_absolute_error: last_error,
        converged: false,
    }
}

struct PointToLineFactor {
    variables: [VariableId; 1],
    source: Vector2<f64>,
    target: Vector2<f64>,
    normal: Vector2<f64>,
    kernel: RobustKernel,
}

impl Factor for PointToLineFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn robust_kernel(&self) -> RobustKernel {
        self.kernel
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        let transform = se2_exp(&Vector3::from_column_slice(values[0].as_slice()));
        let rotation = transform.fixed_view::<2, 2>(0, 0).into_owned();
        let transformed = rotation * self.source + transform.fixed_view::<2, 1>(0, 2);
        let angular = rotation * Vector2::new(-self.source.y, self.source.x);
        let jacobian = DMatrix::from_row_slice(
            1,
            3,
            &[
                self.normal.dot(&rotation.column(0)),
                self.normal.dot(&rotation.column(1)),
                self.normal.dot(&angular),
            ],
        );
        Ok(FactorEvaluation::unit_weighted(
            DVector::from_element(1, self.normal.dot(&(transformed - self.target))),
            vec![jacobian],
        ))
    }
}

struct PointToPlaneFactor {
    variables: [VariableId; 1],
    source: Vector3<f64>,
    target: Vector3<f64>,
    normal: Vector3<f64>,
    kernel: RobustKernel,
}

impl Factor for PointToPlaneFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn robust_kernel(&self) -> RobustKernel {
        self.kernel
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        let transform = se3_exp(&Vector6::from_column_slice(values[0].as_slice()));
        let rotation = transform.fixed_view::<3, 3>(0, 0).into_owned();
        let transformed = rotation * self.source + transform.fixed_view::<3, 1>(0, 3);
        let rotation_jacobian = -rotation * skew(&self.source);
        let mut jacobian = DMatrix::zeros(1, 6);
        for column in 0..3 {
            jacobian[(0, column)] = self.normal.dot(&rotation.column(column));
            jacobian[(0, column + 3)] = self.normal.dot(&rotation_jacobian.column(column));
        }
        Ok(FactorEvaluation::unit_weighted(
            DVector::from_element(1, self.normal.dot(&(transformed - self.target))),
            vec![jacobian],
        ))
    }
}

fn retract_se2(value: &DVector<f64>, increment: &DVector<f64>) -> OptimizationResult<DVector<f64>> {
    let transform = se2_exp(&Vector3::from_column_slice(value.as_slice()));
    let delta = se2_exp(&Vector3::from_column_slice(increment.as_slice()));
    Ok(DVector::from_column_slice(
        se2_log(&(transform * delta)).as_slice(),
    ))
}

fn retract_se3(value: &DVector<f64>, increment: &DVector<f64>) -> OptimizationResult<DVector<f64>> {
    let transform = se3_exp(&Vector6::from_column_slice(value.as_slice()));
    let delta = se3_exp(&Vector6::from_column_slice(increment.as_slice()));
    Ok(DVector::from_column_slice(
        se3_log(&(transform * delta)).as_slice(),
    ))
}

fn nearest_2d(query: &Vector2<f64>, points: &[Vector2<f64>]) -> Option<(usize, f64)> {
    points
        .iter()
        .enumerate()
        .map(|(index, point)| (index, (query - point).norm()))
        .min_by(|left, right| left.1.total_cmp(&right.1))
}

fn nearest_3d(query: &Vector3<f64>, points: &[Vector3<f64>]) -> Option<(usize, f64)> {
    points
        .iter()
        .enumerate()
        .map(|(index, point)| (index, (query - point).norm()))
        .min_by(|left, right| left.1.total_cmp(&right.1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_to_line_recovers_normal_translation() {
        let target = (-10..=10)
            .map(|index| Vector2::new(index as f64 * 0.1, 0.0))
            .collect::<Vec<_>>();
        let source = target
            .iter()
            .map(|point| point + Vector2::new(0.0, 0.3))
            .collect::<Vec<_>>();
        let normals = vec![Vector2::y(); target.len()];
        let result = point_to_line_icp_2d(
            &source,
            &target,
            &normals,
            Matrix3::identity(),
            &GeometricIcpConfig::default(),
        );
        assert!(result.converged);
        assert!((result.transform[(1, 2)] + 0.3).abs() < 1.0e-6);
    }

    #[test]
    fn point_to_plane_recovers_normal_translation() {
        let target = (-2..=2)
            .flat_map(|x| (-2..=2).map(move |y| Vector3::new(x as f64 * 0.2, y as f64 * 0.2, 0.0)))
            .collect::<Vec<_>>();
        let source = target
            .iter()
            .map(|point| point + Vector3::new(0.0, 0.0, 0.4))
            .collect::<Vec<_>>();
        let normals = vec![Vector3::z(); target.len()];
        let result = point_to_plane_icp_3d(
            &source,
            &target,
            &normals,
            Matrix4::identity(),
            &GeometricIcpConfig::default(),
        );
        assert!(result.converged);
        assert!((result.transform[(2, 3)] + 0.4).abs() < 1.0e-6);
    }
}
