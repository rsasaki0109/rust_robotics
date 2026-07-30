//! Replay an EuRoC-layout sequence through IMU preintegration, BA, and SE(3).
//!
//! With no argument this uses the tiny checked-in fixture. Pass an extracted
//! sequence directory containing the optional `mav0/rust_robotics` feature
//! sidecar to validate a larger sequence.

use std::path::PathBuf;

use nalgebra::Vector3;
use rust_robotics::slam::dataset::EurocDataset;
use rust_robotics::slam::vio_pipeline::{
    euroc_vio_input, pose_error, run_vio_pipeline, VioPipelineConfig,
};

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../rust_robotics_slam/tests/fixtures/euroc_mini")
}

fn main() -> DemoResult<()> {
    let root = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(fixture_path);
    let dataset = EurocDataset::load(&root)?;
    let tracks = dataset.load_feature_tracks()?;
    if dataset.ground_truth.len() != dataset.camera_frames.len() {
        return Err("the replay validator needs ground truth at each camera timestamp".into());
    }
    let mut input = euroc_vio_input(&dataset, &tracks, Vector3::new(0.0, 0.0, -9.81))?;
    for landmark in &mut input.landmarks {
        *landmark += Vector3::new(0.02, -0.01, 0.04);
    }
    let result = run_vio_pipeline(&input, &VioPipelineConfig::default())?;
    let expected_terminal = dataset
        .ground_truth
        .last()
        .expect("ground truth is nonempty")
        .world_from_body
        * dataset.camera.body_from_sensor;
    let terminal_error = pose_error(
        result.trajectory.last().expect("trajectory is nonempty"),
        &expected_terminal,
    );
    println!("EuRoC VIO replay: {}", root.display());
    println!("  keyframes={}", result.trajectory.len());
    println!("  imu_samples={}", dataset.imu.len());
    println!("  feature_observations={}", input.observations.len());
    println!("  ba_iterations={}", result.bundle_adjustment_iterations);
    println!("  pose_graph_iterations={}", result.pose_graph_iterations);
    println!("  terminal_se3_error={terminal_error:.6e}");
    println!("  status=ok");
    if !result.pose_graph_converged || terminal_error > 5.0e-2 {
        return Err("EuRoC VIO replay did not meet its acceptance threshold".into());
    }
    Ok(())
}
