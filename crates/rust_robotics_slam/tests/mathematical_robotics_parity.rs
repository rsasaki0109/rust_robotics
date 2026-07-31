//! Golden-value parity with scomup/MathematicalRobotics.
//!
//! Reference commit: 79600010f0c86179905a6960e5fce2bb7cc85d77.

use nalgebra::{Matrix4, Vector2, Vector3};
use rust_robotics_core::{se3_exp, se3_inverse, se3_log, Matrix6, Vector6};
use rust_robotics_slam::bundle_adjustment::{reproject_world_point, CameraIntrinsics};
use rust_robotics_slam::imu_preintegration::{
    ImuBias, ImuExtrinsics, ImuMeasurement, ImuNoise, NavState, PreintegratedImuMeasurement,
};
use rust_robotics_slam::pose_graph_optimization::PoseGraphConfig;
use rust_robotics_slam::pose_graph_optimization_3d::{optimize_pose_graph_3d, Edge3D, Pose3DNode};

const TOLERANCE: f64 = 2.0e-12;
// MathR intentionally uses I + skew(w) below its 1e-5 small-angle threshold,
// while RustRobotics keeps the second-order term. Ten IMU steps therefore
// differ by a bounded sub-microradian amount.
const IMU_TOLERANCE: f64 = 1.0e-6;

#[test]
fn se3_exponential_matches_mathr_math_tools() {
    let tangent = Vector6::new(0.1, 0.3, 0.5, 0.1, 0.2, 0.3);
    let expected = Matrix4::from_row_slice(&[
        0.9357548032779188,
        -0.28316496056507373,
        0.21019170595074288,
        0.10626596926008507,
        0.3029327134026371,
        0.9505806179060915,
        -0.06803131640494002,
        0.29044713134389183,
        -0.18054007669439776,
        0.12733457491763028,
        0.9752903089530457,
        0.5042799226840438,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    let actual = se3_exp(&tangent);
    assert!((actual - expected).norm() < TOLERANCE);
    assert!((se3_log(&actual) - tangent).norm() < TOLERANCE);
}

#[test]
fn imu_prediction_matches_mathr_preintegration() {
    let mut measurement = PreintegratedImuMeasurement::new(ImuBias::zero(), ImuNoise::default());
    for _ in 0..10 {
        measurement
            .integrate(
                Vector3::new(0.2, -0.1, 9.81),
                Vector3::new(0.01, 0.02, -0.01),
                0.01,
            )
            .unwrap();
    }
    let rotation = se3_exp(&Vector6::new(0.0, 0.0, 0.0, 0.2, -0.1, 0.3))
        .fixed_view::<3, 3>(0, 0)
        .into_owned();
    let predicted = measurement.predict(
        &NavState {
            rotation,
            position: Vector3::new(1.0, -2.0, 0.5),
            velocity: Vector3::new(0.4, 0.1, -0.2),
        },
        &ImuBias::zero(),
        Vector3::new(0.0, 0.0, -9.81),
    );
    let expected_rotation = nalgebra::Matrix3::from_row_slice(&[
        0.951017232105154,
        -0.3020489753467215,
        -0.06582722609404616,
        0.28264988918911005,
        0.9358273783217808,
        -0.21056162723898253,
        0.12520289286023054,
        0.18164177513840812,
        0.9753620237836668,
    ]);
    let rotation_error = (predicted.rotation - expected_rotation).norm();
    assert!(
        rotation_error < IMU_TOLERANCE,
        "rotation error {rotation_error}, actual {:?}",
        predicted.rotation
    );
    assert!(
        (predicted.position
            - Vector3::new(1.037795921461914, -2.0004999400430425, 0.47882531174414916))
        .norm()
            < IMU_TOLERANCE
    );
    assert!(
        (predicted.velocity
            - Vector3::new(
                0.3562751781804948,
                -0.11006048891807174,
                -0.22349096589265452,
            ))
        .norm()
            < IMU_TOLERANCE
    );
}

#[test]
fn imu_extrinsic_transform_matches_mathr_kinematics() {
    let body_from_sensor = Matrix4::from_row_slice(&[
        0.9019840683889034,
        -0.15288908060872075,
        0.40379409282853806,
        -0.6466681045654054,
        0.231301825897598,
        0.9607936273555614,
        -0.15288908060872075,
        0.7059522051667105,
        -0.36458772018409946,
        0.231301825897598,
        0.9019840683889034,
        2.2347636942319853,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    let transformed = ImuExtrinsics::from_homogeneous(&body_from_sensor)
        .unwrap()
        .transform(
            ImuMeasurement {
                acceleration: Vector3::new(0.3, -0.1, 9.7),
                angular_velocity: Vector3::new(0.2, -0.3, 0.4),
            },
            Vector3::new(0.05, 0.02, -0.04),
        );
    assert!(
        (transformed.acceleration
            - Vector3::new(3.8885115178902527, -1.272134031029738, 9.21754932157626,))
        .norm()
            < TOLERANCE
    );
    assert!(
        (transformed.angular_velocity
            - Vector3::new(0.3877811749918122, -0.3031333552706371, 0.2184855355494621,))
        .norm()
            < TOLERANCE
    );
}

#[test]
fn reprojection_matches_mathr_projection() {
    let camera = se3_exp(&Vector6::new(0.3, -0.2, 0.1, 0.05, -0.08, 0.12));
    let actual = reproject_world_point(
        &camera,
        &Vector3::new(0.4, -0.1, 4.2),
        &CameraIntrinsics {
            fx: 400.0,
            fy: 380.0,
            cx: 320.0,
            cy: 240.0,
        },
    )
    .unwrap();
    let expected = Vector2::new(363.33698085267736, 263.9976156778255);
    assert!((actual - expected).norm() < TOLERANCE);
}

#[test]
fn pose_graph_loop_closure_matches_mathr_demo_optimum() {
    let count = 12;
    let odometry = se3_exp(&Vector6::new(0.2, 0.0, 0.0, 0.05, 0.0, 0.5));
    let mut current = Matrix4::identity();
    let mut initial = Vec::new();
    for _ in 0..count {
        initial.push(Pose3DNode { transform: current });
        current *= odometry;
    }
    let mut edges = (0..count - 1)
        .map(|from| Edge3D {
            from,
            to: from + 1,
            measurement: Pose3DNode {
                transform: odometry,
            },
            information: Matrix6::identity(),
        })
        .collect::<Vec<_>>();
    edges.push(Edge3D {
        from: count - 1,
        to: 0,
        measurement: Pose3DNode {
            transform: odometry,
        },
        information: Matrix6::identity(),
    });
    let result = optimize_pose_graph_3d(
        &initial,
        &edges,
        &PoseGraphConfig {
            max_iterations: 100,
            tolerance: 1.0e-12,
            ..PoseGraphConfig::default()
        },
    );
    assert!(result.converged);
    let terminal =
        se3_log(&(se3_inverse(&result.poses[0].transform) * result.poses[count - 1].transform));
    let mathr_terminal = Vector6::new(
        -0.1986667985723552,
        0.0020608056529116897,
        0.017304152502988637,
        -0.045405885167849665,
        0.0017092521462982,
        -0.5216215677345143,
    );
    assert!(
        (terminal - mathr_terminal).norm() < 2.0e-4,
        "Rust {terminal:?}, MathR {mathr_terminal:?}"
    );
}
