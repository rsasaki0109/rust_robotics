//! Headless perceptual-aliasing demo: identical pillars every 2.5 m make
//! neighboring places look the same, so a loop match can lock onto the wrong
//! pillar. The ambiguity check re-registers each accepted loop match from
//! shifted seeds and rejects it when a distinct alignment scores nearly as
//! well.
//!
//! Same corridor loop as `headless_lidar_loop_closure`, but the bottom
//! corridor has periodic pillars and the robot starts in its middle, where
//! no corner is in LiDAR range.

use rust_robotics::slam::lidar_graph_slam::LidarGraphSlamConfig;
use rust_robotics::slam::lidar_loop_scenario::{
    aliased_corridor_config, position_rmse, run_corridor_loop,
};
use rust_robotics::slam::scan_to_map::relative_pose;

type DemoResult<T> = Result<T, Box<dyn std::error::Error>>;

/// A loop edge more than this far from ground truth is a false closure \[m\].
const WRONG_LOOP_TOLERANCE: f64 = 0.5;

fn main() -> DemoResult<()> {
    let scenario = aliased_corridor_config();
    println!("Perceptual aliasing: periodic pillars every 2.5 m, start mid-corridor");
    println!(
        "{:<16} {:>9} {:>7} {:>10} {:>13} {:>12} {:>11}",
        "ambiguity_check",
        "closures",
        "wrong",
        "ambiguous",
        "front_rmse_m",
        "slam_rmse_m",
        "final_xy_m"
    );

    let mut results = Vec::new();
    for check in [false, true] {
        let run = run_corridor_loop(
            &scenario,
            LidarGraphSlamConfig {
                loop_ambiguity_check: check,
                ..LidarGraphSlamConfig::default()
            },
        );
        let last = run.frames.last().ok_or("empty run")?;
        let final_error = relative_pose(last.truth, run.final_estimate);
        let front_rmse = position_rmse(&run.node_front_end, &run.node_truth);
        let slam_rmse = position_rmse(&run.final_node_poses, &run.node_truth);
        let wrong = run.wrong_loop_closures(WRONG_LOOP_TOLERANCE);
        println!(
            "{:<16} {:>9} {:>7} {:>10} {:>13.4} {:>12.4} {:>11.4}",
            if check { "on" } else { "off" },
            run.loop_closures.len(),
            wrong,
            run.ambiguous_loop_rejections,
            front_rmse,
            slam_rmse,
            final_error.x.hypot(final_error.y),
        );
        results.push((wrong, front_rmse, slam_rmse, run.loop_closures.len()));
    }

    let (wrong_off, front_off, slam_off, _) = results[0];
    let (wrong_on, front_on, slam_on, closures_on) = results[1];
    if wrong_off == 0 || slam_off <= front_off {
        return Err("expected aliasing to corrupt the map without the ambiguity check".into());
    }
    if wrong_on != 0 || closures_on == 0 || slam_on >= front_on {
        return Err("expected only correct loop closures with the ambiguity check".into());
    }
    Ok(())
}
