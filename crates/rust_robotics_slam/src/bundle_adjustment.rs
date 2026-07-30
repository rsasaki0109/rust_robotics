//! Pinhole reprojection factors and dense bundle adjustment.

use nalgebra::{DMatrix, DVector, Matrix2, Matrix4, SMatrix, Vector2, Vector3};
use rust_robotics_core::{se3_exp, se3_inverse, se3_log, skew, Vector6};
use rust_robotics_optimization::{
    solve, Factor, FactorEvaluation, OptimizationError, OptimizationResult, Problem, RobustKernel,
    SolverConfig, SolverSummary, Variable, VariableId,
};

/// Pinhole camera calibration without distortion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl CameraIntrinsics {
    /// Projects a camera-frame point onto the image plane.
    pub fn project(&self, point: &Vector3<f64>) -> OptimizationResult<Vector2<f64>> {
        if point.z <= 1.0e-9 || !point.iter().all(|value| value.is_finite()) {
            return Err(OptimizationError::InvalidFactor(
                "reprojected point must be finite and in front of the camera".into(),
            ));
        }
        Ok(Vector2::new(
            self.fx * point.x / point.z + self.cx,
            self.fy * point.y / point.z + self.cy,
        ))
    }

    fn projection_jacobian(&self, point: &Vector3<f64>) -> SMatrix<f64, 2, 3> {
        let inverse_z = 1.0 / point.z;
        let inverse_z_squared = inverse_z * inverse_z;
        SMatrix::<f64, 2, 3>::new(
            self.fx * inverse_z,
            0.0,
            -self.fx * point.x * inverse_z_squared,
            0.0,
            self.fy * inverse_z,
            -self.fy * point.y * inverse_z_squared,
        )
    }
}

/// One observed landmark pixel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReprojectionObservation {
    pub camera: usize,
    pub point: usize,
    pub pixel: Vector2<f64>,
    pub information: Matrix2<f64>,
}

/// Inputs to a bundle-adjustment solve. Camera poses are world-from-camera.
#[derive(Debug, Clone)]
pub struct BundleAdjustmentProblem {
    pub cameras: Vec<Matrix4<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub observations: Vec<ReprojectionObservation>,
    pub intrinsics: CameraIntrinsics,
}

/// Bundle-adjustment configuration.
#[derive(Debug, Clone)]
pub struct BundleAdjustmentConfig {
    pub solver: SolverConfig,
    /// Number of leading camera poses kept constant to remove gauge freedom.
    pub fixed_cameras: usize,
    pub robust_kernel: RobustKernel,
}

impl Default for BundleAdjustmentConfig {
    fn default() -> Self {
        Self {
            solver: SolverConfig::default(),
            fixed_cameras: 1,
            robust_kernel: RobustKernel::Huber { delta: 2.0 },
        }
    }
}

/// Optimized camera poses and landmarks.
#[derive(Debug, Clone)]
pub struct BundleAdjustmentResult {
    pub cameras: Vec<Matrix4<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub summary: SolverSummary,
}

/// Reprojects a world point into a world-from-camera pose.
pub fn reproject_world_point(
    camera_world_from_camera: &Matrix4<f64>,
    point_world: &Vector3<f64>,
    intrinsics: &CameraIntrinsics,
) -> OptimizationResult<Vector2<f64>> {
    let inverse = se3_inverse(camera_world_from_camera);
    let point_camera =
        inverse.fixed_view::<3, 3>(0, 0) * point_world + inverse.fixed_view::<3, 1>(0, 3);
    intrinsics.project(&point_camera)
}

/// Runs joint camera/landmark bundle adjustment.
pub fn bundle_adjust(
    input: &BundleAdjustmentProblem,
    config: &BundleAdjustmentConfig,
) -> OptimizationResult<BundleAdjustmentResult> {
    if input.cameras.is_empty() || input.points.is_empty() {
        return Err(OptimizationError::InvalidParameter(
            "bundle adjustment needs cameras and points".into(),
        ));
    }
    let mut problem = Problem::new();
    let camera_ids = input
        .cameras
        .iter()
        .map(|camera| {
            problem.add_variable(
                Variable::with_retraction(
                    DVector::from_column_slice(se3_log(camera).as_slice()),
                    6,
                    retract_camera,
                )
                .expect("camera dimensions are valid"),
            )
        })
        .collect::<Vec<_>>();
    for id in camera_ids.iter().take(config.fixed_cameras) {
        problem.variable_mut(*id)?.set_fixed(true);
    }
    let point_ids = input
        .points
        .iter()
        .map(|point| {
            problem.add_variable(Variable::euclidean(DVector::from_column_slice(
                point.as_slice(),
            )))
        })
        .collect::<Vec<_>>();

    for observation in &input.observations {
        let camera = *camera_ids.get(observation.camera).ok_or_else(|| {
            OptimizationError::InvalidParameter("observation camera index is out of range".into())
        })?;
        let point = *point_ids.get(observation.point).ok_or_else(|| {
            OptimizationError::InvalidParameter("observation point index is out of range".into())
        })?;
        problem.add_factor(ReprojectionFactor {
            variables: [camera, point],
            pixel: observation.pixel,
            information: observation.information,
            intrinsics: input.intrinsics,
            kernel: config.robust_kernel,
        })?;
    }

    let summary = solve(&mut problem, &config.solver)?;
    let cameras = camera_ids
        .iter()
        .map(|id| {
            let value = problem.variable(*id).expect("camera exists").value();
            se3_exp(&Vector6::from_column_slice(value.as_slice()))
        })
        .collect();
    let points = point_ids
        .iter()
        .map(|id| {
            let value = problem.variable(*id).expect("point exists").value();
            Vector3::new(value[0], value[1], value[2])
        })
        .collect();
    Ok(BundleAdjustmentResult {
        cameras,
        points,
        summary,
    })
}

struct ReprojectionFactor {
    variables: [VariableId; 2],
    pixel: Vector2<f64>,
    information: Matrix2<f64>,
    intrinsics: CameraIntrinsics,
    kernel: RobustKernel,
}

impl Factor for ReprojectionFactor {
    fn variable_ids(&self) -> &[VariableId] {
        &self.variables
    }

    fn robust_kernel(&self) -> RobustKernel {
        self.kernel
    }

    fn evaluate(&self, values: &[&DVector<f64>]) -> OptimizationResult<FactorEvaluation> {
        let camera = se3_exp(&Vector6::from_column_slice(values[0].as_slice()));
        let point_world = Vector3::new(values[1][0], values[1][1], values[1][2]);
        let inverse = se3_inverse(&camera);
        let rotation_camera_from_world = inverse.fixed_view::<3, 3>(0, 0).into_owned();
        let point_camera =
            rotation_camera_from_world * point_world + inverse.fixed_view::<3, 1>(0, 3);
        let projection = self.intrinsics.project(&point_camera)?;
        let projection_jacobian = self.intrinsics.projection_jacobian(&point_camera);

        // Right perturbation of T_wc gives p_c' ~= p_c - v + [p_c]x w.
        let camera_point_jacobian = SMatrix::<f64, 3, 6>::from_columns(&[
            -Vector3::x(),
            -Vector3::y(),
            -Vector3::z(),
            skew(&point_camera).column(0).into_owned(),
            skew(&point_camera).column(1).into_owned(),
            skew(&point_camera).column(2).into_owned(),
        ]);
        let camera_jacobian = projection_jacobian * camera_point_jacobian;
        let point_jacobian = projection_jacobian * rotation_camera_from_world;
        Ok(FactorEvaluation {
            residual: DVector::from_column_slice((projection - self.pixel).as_slice()),
            information: DMatrix::from_column_slice(2, 2, self.information.as_slice()),
            jacobians: vec![
                DMatrix::from_column_slice(2, 6, camera_jacobian.as_slice()),
                DMatrix::from_column_slice(2, 3, point_jacobian.as_slice()),
            ],
        })
    }
}

fn retract_camera(
    value: &DVector<f64>,
    increment: &DVector<f64>,
) -> OptimizationResult<DVector<f64>> {
    let camera = se3_exp(&Vector6::from_column_slice(value.as_slice()));
    let delta = se3_exp(&Vector6::from_column_slice(increment.as_slice()));
    Ok(DVector::from_column_slice(
        se3_log(&(camera * delta)).as_slice(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reprojection_matches_pinhole_geometry() {
        let intrinsics = CameraIntrinsics {
            fx: 400.0,
            fy: 400.0,
            cx: 320.0,
            cy: 240.0,
        };
        let pixel = reproject_world_point(
            &Matrix4::identity(),
            &Vector3::new(1.0, 0.5, 2.0),
            &intrinsics,
        )
        .unwrap();
        assert!((pixel.x - 520.0).abs() < 1.0e-12);
        assert!((pixel.y - 340.0).abs() < 1.0e-12);
    }

    #[test]
    fn reprojection_jacobians_match_finite_differences() {
        let intrinsics = CameraIntrinsics {
            fx: 400.0,
            fy: 380.0,
            cx: 320.0,
            cy: 240.0,
        };
        let camera =
            DVector::from_column_slice(Vector6::new(0.2, -0.1, 0.3, 0.05, -0.08, 0.03).as_slice());
        let point = DVector::from_vec(vec![0.5, -0.2, 4.0]);
        let factor = ReprojectionFactor {
            variables: [VariableId(0), VariableId(1)],
            pixel: Vector2::new(330.0, 220.0),
            information: Matrix2::identity(),
            intrinsics,
            kernel: RobustKernel::L2,
        };
        let analytic = factor.evaluate(&[&camera, &point]).unwrap();
        let epsilon = 1.0e-6;

        for variable in 0..2 {
            let dimension = if variable == 0 { 6 } else { 3 };
            for column in 0..dimension {
                let mut delta = DVector::zeros(dimension);
                delta[column] = epsilon;
                let (plus, minus) = if variable == 0 {
                    (
                        retract_camera(&camera, &delta).unwrap(),
                        retract_camera(&camera, &(-&delta)).unwrap(),
                    )
                } else {
                    (&point + &delta, &point - &delta)
                };
                let (plus_values, minus_values) = if variable == 0 {
                    ([&plus, &point], [&minus, &point])
                } else {
                    ([&camera, &plus], [&camera, &minus])
                };
                let plus_residual = factor.evaluate(&plus_values).unwrap().residual;
                let minus_residual = factor.evaluate(&minus_values).unwrap().residual;
                let numerical = (plus_residual - minus_residual) / (2.0 * epsilon);
                assert!((analytic.jacobians[variable].column(column) - numerical).norm() < 1.0e-5);
            }
        }
    }

    #[test]
    fn bundle_adjustment_recovers_points_with_fixed_cameras() {
        let intrinsics = CameraIntrinsics {
            fx: 300.0,
            fy: 300.0,
            cx: 320.0,
            cy: 240.0,
        };
        let cameras = vec![
            Matrix4::identity(),
            se3_exp(&Vector6::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ];
        let truth = [
            Vector3::new(0.2, 0.1, 4.0),
            Vector3::new(-0.5, 0.3, 5.0),
            Vector3::new(0.7, -0.4, 6.0),
        ];
        let observations = cameras
            .iter()
            .enumerate()
            .flat_map(|(camera, pose)| {
                truth
                    .iter()
                    .enumerate()
                    .map(move |(point, landmark)| ReprojectionObservation {
                        camera,
                        point,
                        pixel: reproject_world_point(pose, landmark, &intrinsics).unwrap(),
                        information: Matrix2::identity(),
                    })
            })
            .collect();
        let problem = BundleAdjustmentProblem {
            cameras,
            points: truth
                .iter()
                .map(|point| point + Vector3::new(0.15, -0.1, 0.2))
                .collect(),
            observations,
            intrinsics,
        };
        let result = bundle_adjust(
            &problem,
            &BundleAdjustmentConfig {
                fixed_cameras: 2,
                ..BundleAdjustmentConfig::default()
            },
        )
        .unwrap();
        for (actual, expected) in result.points.iter().zip(truth) {
            assert!((actual - expected).norm() < 1.0e-6);
        }
        assert!(result.summary.final_cost < result.summary.initial_cost);
    }
}
