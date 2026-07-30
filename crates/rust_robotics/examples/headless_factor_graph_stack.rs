//! Headless end-to-end smoke test for the Lie-group and factor-graph stack.

use nalgebra::{Matrix2, Matrix4, Vector3};
use rust_robotics::core::{se3_exp, Vector6};
use rust_robotics::slam::bundle_adjustment::{
    bundle_adjust, reproject_world_point, BundleAdjustmentConfig, BundleAdjustmentProblem,
    CameraIntrinsics, ReprojectionObservation,
};
use rust_robotics::slam::g2o::{parse_g2o, G2oGraph};
use rust_robotics::slam::geometric_icp::{point_to_plane_icp_3d, GeometricIcpConfig};
use rust_robotics::slam::imu_preintegration::{
    ImuBias, ImuNoise, NavState, PreintegratedImuMeasurement,
};
use rust_robotics::slam::pose_graph_optimization::{optimize_pose_graph, PoseGraphConfig};

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

fn run_pose_graph() -> DemoResult<f64> {
    let graph = parse_g2o(
        "\
VERTEX_SE2 0 0 0 0
VERTEX_SE2 1 1.12 -0.08 0.05
VERTEX_SE2 2 2.15 0.12 -0.04
EDGE_SE2 0 1 1 0 0 100 0 0 100 0 100
EDGE_SE2 1 2 1 0 0 100 0 0 100 0 100
EDGE_SE2 0 2 2 0 0 100 0 0 100 0 100
",
    )?;
    let G2oGraph::Se2 { vertices, edges } = graph else {
        return Err("expected an SE(2) graph".into());
    };
    let initial = vertices.values().copied().collect::<Vec<_>>();
    let optimized = optimize_pose_graph(&initial, &edges, &PoseGraphConfig::default());
    if !optimized.converged {
        return Err("SE(2) pose graph did not converge".into());
    }
    let terminal = optimized.poses.last().ok_or("pose graph is empty")?;
    let error = (terminal.x - 2.0).hypot(terminal.y);
    if error > 1.0e-4 {
        return Err(format!("pose graph terminal error is {error}").into());
    }
    Ok(error)
}

fn run_imu_preintegration() -> DemoResult<f64> {
    let mut measurement = PreintegratedImuMeasurement::new(ImuBias::zero(), ImuNoise::default());
    for _ in 0..100 {
        measurement.integrate(Vector3::new(0.0, 0.0, 9.81), Vector3::zeros(), 0.01)?;
    }
    let predicted = measurement.predict(
        &NavState::identity(),
        &ImuBias::zero(),
        Vector3::new(0.0, 0.0, -9.81),
    );
    let drift = predicted.position.norm() + predicted.velocity.norm();
    if drift > 1.0e-8 {
        return Err(format!("stationary IMU drift is {drift}").into());
    }
    Ok(drift)
}

fn run_bundle_adjustment() -> DemoResult<f64> {
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
        Vector3::new(-0.4, 0.3, 5.0),
        Vector3::new(0.7, -0.2, 6.0),
    ];
    let observations = cameras
        .iter()
        .enumerate()
        .flat_map(|(camera, pose)| {
            truth
                .iter()
                .enumerate()
                .map(move |(point, landmark)| -> DemoResult<_> {
                    Ok(ReprojectionObservation {
                        camera,
                        point,
                        pixel: reproject_world_point(pose, landmark, &intrinsics)?,
                        information: Matrix2::identity(),
                    })
                })
        })
        .collect::<DemoResult<Vec<_>>>()?;
    let problem = BundleAdjustmentProblem {
        cameras,
        points: truth
            .iter()
            .map(|point| point + Vector3::new(0.12, -0.08, 0.2))
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
    )?;
    let error = result
        .points
        .iter()
        .zip(truth)
        .map(|(actual, expected)| (actual - expected).norm())
        .sum::<f64>()
        / truth.len() as f64;
    if error > 1.0e-5 {
        return Err(format!("bundle-adjustment point error is {error}").into());
    }
    Ok(error)
}

fn run_point_to_plane_icp() -> DemoResult<f64> {
    let target = (-2..=2)
        .flat_map(|x| (-2..=2).map(move |y| Vector3::new(x as f64 * 0.2, y as f64 * 0.2, 0.0)))
        .collect::<Vec<_>>();
    let source = target
        .iter()
        .map(|point| point + Vector3::new(0.0, 0.0, 0.35))
        .collect::<Vec<_>>();
    let normals = vec![Vector3::z(); target.len()];
    let result = point_to_plane_icp_3d(
        &source,
        &target,
        &normals,
        Matrix4::identity(),
        &GeometricIcpConfig::default(),
    );
    if !result.converged {
        return Err("point-to-plane ICP did not converge".into());
    }
    let error = (result.transform[(2, 3)] + 0.35).abs();
    if error > 1.0e-5 {
        return Err(format!("point-to-plane ICP transform error is {error}").into());
    }
    Ok(error)
}

fn main() -> DemoResult<()> {
    let pose_graph_error = run_pose_graph()?;
    let imu_drift = run_imu_preintegration()?;
    let bundle_adjustment_error = run_bundle_adjustment()?;
    let icp_error = run_point_to_plane_icp()?;

    println!("factor-graph stack:");
    println!("  pose_graph_terminal_error={pose_graph_error:.3e}");
    println!("  stationary_imu_drift={imu_drift:.3e}");
    println!("  bundle_adjustment_mean_point_error={bundle_adjustment_error:.3e}");
    println!("  point_to_plane_icp_transform_error={icp_error:.3e}");
    println!("  status=ok");
    Ok(())
}
