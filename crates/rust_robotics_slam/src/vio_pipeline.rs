//! Offline visual-inertial pipeline built from the shared estimation modules.

use nalgebra::{Matrix2, Matrix4, Vector3};
use rust_robotics_core::{se3_inverse, se3_log, Matrix6};
use rust_robotics_optimization::{LinearSolver, OptimizationError, OptimizationResult};

use crate::bundle_adjustment::{
    bundle_adjust, BundleAdjustmentConfig, BundleAdjustmentProblem, CameraIntrinsics,
    ReprojectionObservation,
};
use crate::dataset::{EurocDataset, EurocFeatureTracks, ImuSample};
use crate::imu_preintegration::{ImuBias, ImuNoise, NavState, PreintegratedImuMeasurement};
use crate::pose_graph_optimization::PoseGraphConfig;
use crate::pose_graph_optimization_3d::{
    optimize_pose_graph_3d, Edge3D, Pose3DNode, PoseGraphResult3D,
};

/// Inputs for an offline visual-inertial solve.
#[derive(Debug, Clone)]
pub struct VioPipelineInput {
    pub keyframe_timestamps_ns: Vec<i64>,
    pub imu_samples: Vec<ImuSample>,
    pub initial_state: NavState,
    pub bias: ImuBias,
    pub gravity: Vector3<f64>,
    /// Body-from-camera extrinsic transform.
    pub body_from_camera: Matrix4<f64>,
    pub landmarks: Vec<Vector3<f64>>,
    pub observations: Vec<ReprojectionObservation>,
    pub intrinsics: CameraIntrinsics,
}

/// Configuration for the visual-inertial stages.
#[derive(Debug, Clone)]
pub struct VioPipelineConfig {
    pub imu_noise: ImuNoise,
    pub bundle_adjustment: BundleAdjustmentConfig,
    pub pose_graph: PoseGraphConfig,
    pub imu_information: Matrix6,
    pub visual_information: Matrix6,
}

impl Default for VioPipelineConfig {
    fn default() -> Self {
        Self {
            imu_noise: ImuNoise::default(),
            bundle_adjustment: BundleAdjustmentConfig::default(),
            pose_graph: PoseGraphConfig {
                linear_solver: LinearSolver::BlockSparsePcg {
                    max_iterations: 1_000,
                    tolerance: 1.0e-10,
                },
                ..PoseGraphConfig::default()
            },
            imu_information: Matrix6::identity() * 100.0,
            visual_information: Matrix6::identity() * 50.0,
        }
    }
}

/// Output and stage diagnostics from [`run_vio_pipeline`].
#[derive(Debug, Clone)]
pub struct VioPipelineResult {
    pub imu_states: Vec<NavState>,
    pub bundle_adjusted_cameras: Vec<Matrix4<f64>>,
    pub landmarks: Vec<Vector3<f64>>,
    pub trajectory: Vec<Matrix4<f64>>,
    pub pose_graph_iterations: usize,
    pub pose_graph_converged: bool,
    pub bundle_adjustment_iterations: usize,
}

/// Converts a loaded EuRoC sequence and pre-extracted track sidecar into VIO
/// inputs. Ground truth is used only for the initial state and acceptance
/// evaluation; later states are not passed to the optimizer.
pub fn euroc_vio_input(
    dataset: &EurocDataset,
    tracks: &EurocFeatureTracks,
    gravity: Vector3<f64>,
) -> OptimizationResult<VioPipelineInput> {
    let first_truth = dataset.ground_truth.first().ok_or_else(|| {
        OptimizationError::InvalidParameter("EuRoC VIO initialization needs ground truth".into())
    })?;
    let observations = tracks
        .observations
        .iter()
        .map(|observation| {
            let camera = dataset
                .camera_frames
                .binary_search_by_key(&observation.timestamp_ns, |frame| frame.timestamp_ns)
                .map_err(|_| {
                    OptimizationError::InvalidParameter(
                        "feature timestamp has no matching camera frame".into(),
                    )
                })?;
            Ok(ReprojectionObservation {
                camera,
                point: observation.landmark_id,
                pixel: observation.pixel,
                information: Matrix2::identity(),
            })
        })
        .collect::<OptimizationResult<Vec<_>>>()?;
    Ok(VioPipelineInput {
        keyframe_timestamps_ns: dataset
            .camera_frames
            .iter()
            .map(|frame| frame.timestamp_ns)
            .collect(),
        imu_samples: dataset.imu.clone(),
        initial_state: NavState {
            rotation: first_truth
                .world_from_body
                .fixed_view::<3, 3>(0, 0)
                .into_owned(),
            position: first_truth
                .world_from_body
                .fixed_view::<3, 1>(0, 3)
                .into_owned(),
            velocity: first_truth.velocity,
        },
        bias: ImuBias {
            accelerometer: first_truth.accelerometer_bias,
            gyroscope: first_truth.gyroscope_bias,
        },
        gravity,
        body_from_camera: dataset.camera.body_from_sensor,
        landmarks: tracks
            .landmarks
            .iter()
            .map(|landmark| landmark.position)
            .collect(),
        observations,
        intrinsics: CameraIntrinsics {
            fx: dataset.camera.intrinsics[0],
            fy: dataset.camera.intrinsics[1],
            cx: dataset.camera.intrinsics[2],
            cy: dataset.camera.intrinsics[3],
        },
    })
}

/// Runs IMU initialization, joint camera/landmark BA, then SE(3) graph fusion.
pub fn run_vio_pipeline(
    input: &VioPipelineInput,
    config: &VioPipelineConfig,
) -> OptimizationResult<VioPipelineResult> {
    validate_input(input)?;
    let (imu_states, preintegrated) = initialize_from_imu(input, config)?;
    let imu_cameras = imu_states
        .iter()
        .map(|state| nav_state_pose(state) * input.body_from_camera)
        .collect::<Vec<_>>();
    let bundle_adjustment = bundle_adjust(
        &BundleAdjustmentProblem {
            cameras: imu_cameras,
            points: input.landmarks.clone(),
            observations: input.observations.clone(),
            intrinsics: input.intrinsics,
        },
        &config.bundle_adjustment,
    )?;

    let graph = fuse_pose_graph(
        &bundle_adjustment.cameras,
        &imu_states,
        &preintegrated,
        &input.body_from_camera,
        config,
    );
    Ok(VioPipelineResult {
        imu_states,
        bundle_adjusted_cameras: bundle_adjustment.cameras,
        landmarks: bundle_adjustment.points,
        trajectory: graph.poses.iter().map(|pose| pose.transform).collect(),
        pose_graph_iterations: graph.iterations,
        pose_graph_converged: graph.converged,
        bundle_adjustment_iterations: bundle_adjustment.summary.iterations,
    })
}

fn validate_input(input: &VioPipelineInput) -> OptimizationResult<()> {
    if input.keyframe_timestamps_ns.len() < 2 {
        return Err(OptimizationError::InvalidParameter(
            "VIO requires at least two keyframes".into(),
        ));
    }
    if input
        .keyframe_timestamps_ns
        .windows(2)
        .any(|window| window[1] <= window[0])
    {
        return Err(OptimizationError::InvalidParameter(
            "VIO keyframe timestamps must be strictly increasing".into(),
        ));
    }
    if input.imu_samples.is_empty() || input.landmarks.is_empty() {
        return Err(OptimizationError::InvalidParameter(
            "VIO requires IMU samples and landmarks".into(),
        ));
    }
    if input.observations.iter().any(|observation| {
        observation.camera >= input.keyframe_timestamps_ns.len()
            || observation.point >= input.landmarks.len()
    }) {
        return Err(OptimizationError::InvalidParameter(
            "VIO observation index is out of range".into(),
        ));
    }
    Ok(())
}

fn initialize_from_imu(
    input: &VioPipelineInput,
    config: &VioPipelineConfig,
) -> OptimizationResult<(Vec<NavState>, Vec<PreintegratedImuMeasurement>)> {
    let mut states = vec![input.initial_state.clone()];
    let mut measurements = Vec::with_capacity(input.keyframe_timestamps_ns.len() - 1);
    for interval in input.keyframe_timestamps_ns.windows(2) {
        let measurement = preintegrate_interval(
            &input.imu_samples,
            interval[0],
            interval[1],
            input.bias,
            config.imu_noise,
        )?;
        let next = measurement.predict(
            states.last().expect("initial state exists"),
            &input.bias,
            input.gravity,
        );
        states.push(next);
        measurements.push(measurement);
    }
    Ok((states, measurements))
}

fn preintegrate_interval(
    samples: &[ImuSample],
    start_ns: i64,
    end_ns: i64,
    bias: ImuBias,
    noise: ImuNoise,
) -> OptimizationResult<PreintegratedImuMeasurement> {
    let mut measurement = PreintegratedImuMeasurement::new(bias, noise);
    let mut previous_ns = start_ns;
    let mut current = samples
        .iter()
        .rev()
        .find(|sample| sample.timestamp_ns <= start_ns)
        .or_else(|| samples.iter().find(|sample| sample.timestamp_ns < end_ns))
        .ok_or_else(|| {
            OptimizationError::InvalidParameter("no IMU sample covers keyframe interval".into())
        })?;
    for sample in samples
        .iter()
        .filter(|sample| sample.timestamp_ns > start_ns && sample.timestamp_ns < end_ns)
    {
        let dt = (sample.timestamp_ns - previous_ns) as f64 * 1.0e-9;
        measurement.integrate(current.acceleration, current.angular_velocity, dt)?;
        previous_ns = sample.timestamp_ns;
        current = sample;
    }
    let final_dt = (end_ns - previous_ns) as f64 * 1.0e-9;
    measurement.integrate(current.acceleration, current.angular_velocity, final_dt)?;
    Ok(measurement)
}

fn nav_state_pose(state: &NavState) -> Matrix4<f64> {
    let mut pose = Matrix4::identity();
    pose.fixed_view_mut::<3, 3>(0, 0).copy_from(&state.rotation);
    pose.fixed_view_mut::<3, 1>(0, 3).copy_from(&state.position);
    pose
}

fn fuse_pose_graph(
    cameras: &[Matrix4<f64>],
    imu_states: &[NavState],
    preintegrated: &[PreintegratedImuMeasurement],
    body_from_camera: &Matrix4<f64>,
    config: &VioPipelineConfig,
) -> PoseGraphResult3D {
    let initial = cameras
        .iter()
        .map(|transform| Pose3DNode {
            transform: *transform,
        })
        .collect::<Vec<_>>();
    let mut edges = preintegrated
        .iter()
        .enumerate()
        .map(|(index, _)| {
            let from = nav_state_pose(&imu_states[index]) * body_from_camera;
            let to = nav_state_pose(&imu_states[index + 1]) * body_from_camera;
            Edge3D {
                from: index,
                to: index + 1,
                measurement: Pose3DNode {
                    transform: se3_inverse(&from) * to,
                },
                information: config.imu_information,
            }
        })
        .collect::<Vec<_>>();
    let last = cameras.len() - 1;
    edges.push(Edge3D {
        from: 0,
        to: last,
        measurement: Pose3DNode {
            transform: se3_inverse(&cameras[0]) * cameras[last],
        },
        information: config.visual_information,
    });
    optimize_pose_graph_3d(&initial, &edges, &config.pose_graph)
}

/// Translation plus rotation-vector error between two poses.
pub fn pose_error(actual: &Matrix4<f64>, expected: &Matrix4<f64>) -> f64 {
    se3_log(&(se3_inverse(expected) * actual)).norm()
}

#[cfg(test)]
mod tests {
    use nalgebra::Matrix2;

    use super::*;
    use crate::bundle_adjustment::reproject_world_point;

    #[test]
    fn connected_vio_pipeline_recovers_synthetic_sequence() {
        let timestamps = (0..5).map(|index| index * 200_000_000).collect::<Vec<_>>();
        let imu_samples = (0..=80)
            .map(|index| ImuSample {
                timestamp_ns: index * 10_000_000,
                angular_velocity: Vector3::zeros(),
                acceleration: Vector3::new(0.0, 0.0, 9.81),
            })
            .collect::<Vec<_>>();
        let intrinsics = CameraIntrinsics {
            fx: 320.0,
            fy: 320.0,
            cx: 320.0,
            cy: 240.0,
        };
        let truth_cameras = (0..5)
            .map(|index| {
                let mut pose = Matrix4::identity();
                pose[(0, 3)] = index as f64 * 0.2;
                pose
            })
            .collect::<Vec<_>>();
        let truth_points = vec![
            Vector3::new(-0.4, -0.2, 4.0),
            Vector3::new(0.2, 0.3, 4.5),
            Vector3::new(0.8, -0.1, 5.0),
            Vector3::new(-0.6, 0.4, 5.5),
        ];
        let observations = truth_cameras
            .iter()
            .enumerate()
            .flat_map(|(camera, pose)| {
                truth_points
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
        let input = VioPipelineInput {
            keyframe_timestamps_ns: timestamps,
            imu_samples,
            initial_state: NavState {
                velocity: Vector3::x(),
                ..NavState::identity()
            },
            bias: ImuBias::zero(),
            gravity: Vector3::new(0.0, 0.0, -9.81),
            body_from_camera: Matrix4::identity(),
            landmarks: truth_points
                .iter()
                .map(|point| point + Vector3::new(0.08, -0.05, 0.12))
                .collect(),
            observations,
            intrinsics,
        };
        let result = run_vio_pipeline(&input, &VioPipelineConfig::default()).unwrap();
        assert!(result.pose_graph_converged);
        assert_eq!(result.trajectory.len(), truth_cameras.len());
        let terminal_error = pose_error(&result.trajectory[4], &truth_cameras[4]);
        assert!(terminal_error < 2.0e-2, "terminal error: {terminal_error}");
        let landmark_error = result
            .landmarks
            .iter()
            .zip(&truth_points)
            .map(|(actual, expected)| (actual - expected).norm())
            .sum::<f64>()
            / truth_points.len() as f64;
        assert!(landmark_error < 0.2, "landmark error: {landmark_error}");
    }
}
