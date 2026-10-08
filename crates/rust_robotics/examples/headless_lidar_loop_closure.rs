//! Headless loop-closure demo: scan-to-map odometry with and without the
//! pose-graph back end on a deterministic corridor loop.
//!
//! The robot drives slightly more than one lap of a 3 m wide rectangular
//! corridor (about 85 m per lap) with pillars along the walls. Wheel odometry
//! over-reports distance by 3 % and drifts 1 deg per meter in yaw; the 2D
//! LiDAR has 8 m range and 2 cm range noise. The top corridor has no pillars,
//! so along it the scan-to-map front end cannot observe forward motion and
//! inherits the odometry scale error; revisiting the start closes the loop.
//! The scenario lives in `rust_robotics_slam::lidar_loop_scenario`.

use rust_robotics::core::Pose2D;
use rust_robotics::slam::lidar_graph_slam::LidarGraphSlamConfig;
use rust_robotics::slam::lidar_loop_scenario::{
    position_rmse, run_corridor_loop, CorridorLoopConfig,
};
use rust_robotics::slam::scan_to_map::relative_pose;

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

fn main() -> DemoResult<()> {
    let run = run_corridor_loop(
        &CorridorLoopConfig::default(),
        LidarGraphSlamConfig::default(),
    );
    let last = run.frames.last().ok_or("empty run")?;
    let final_error = |pose: Pose2D| {
        let delta = relative_pose(last.truth, pose);
        (delta.x.hypot(delta.y), delta.yaw.abs().to_degrees())
    };
    let (odom_final, odom_yaw) = final_error(last.odometry);
    let (front_final, front_yaw) = final_error(last.front_end);
    let (slam_final, slam_yaw) = final_error(run.final_estimate);

    let first_closure_node = run.loop_closures.first().map(|closure| closure.to);
    println!(
        "LiDAR loop closure: {:.1} m driven ({:.1} m lap), {} nodes, {} loop closures \
         (first at node {})",
        run.distance,
        run.lap_length,
        run.final_node_poses.len(),
        run.loop_closures.len(),
        first_closure_node.map_or("-".to_string(), |node| node.to_string()),
    );
    println!(
        "{:<22} {:>14} {:>12} {:>14}",
        "estimator", "node_rmse_m", "final_xy_m", "final_yaw_deg"
    );
    let odom_rmse = position_rmse(&run.node_odometry, &run.node_truth);
    let front_rmse = position_rmse(&run.node_front_end, &run.node_truth);
    let slam_rmse = position_rmse(&run.final_node_poses, &run.node_truth);
    for (name, node_rmse, final_xy, final_yaw) in [
        ("raw_odometry", odom_rmse, odom_final, odom_yaw),
        ("scan_to_map", front_rmse, front_final, front_yaw),
        ("scan_to_map+loop", slam_rmse, slam_final, slam_yaw),
    ] {
        println!("{name:<22} {node_rmse:>14.4} {final_xy:>12.4} {final_yaw:>14.3}");
    }

    if run.loop_closures.is_empty() {
        return Err("no loop closure was accepted".into());
    }
    if slam_rmse >= front_rmse || front_rmse >= odom_rmse {
        return Err("expected raw odometry > scan-to-map > scan-to-map+loop node RMSE".into());
    }
    if slam_final >= front_final {
        return Err("loop closure did not reduce the final position error".into());
    }
    Ok(())
}
