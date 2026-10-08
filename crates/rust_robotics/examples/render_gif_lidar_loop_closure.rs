//! Render LiDAR graph SLAM loop closure on the corridor-loop scenario as an
//! animated GIF: `media/gallery/lidar_loop_closure.gif`.
//!
//! Gray: ground truth. Orange: scan-to-map front end (drifts in the pillar-free
//! top corridor). Green: pose-graph nodes. Magenta: loop-closure edges. Blue:
//! map rendered from node scans at the current node poses. Red: current scan.
//! The animation pauses on each re-optimization so the correction is visible.
//!
//! ```bash
//! cargo run --release -p rust_robotics --example render_gif_lidar_loop_closure --features "slam,gif"
//! ```

use rust_robotics::core::Pose2D;
use rust_robotics::slam::lidar_graph_slam::LidarGraphSlamConfig;
use rust_robotics::slam::lidar_loop_scenario::{
    run_corridor_loop, CorridorLoopConfig, CorridorLoopRun,
};
use rust_robotics::slam::scan_to_map::transform_scan_to_world;
use rust_robotics::viz::{GifCanvasConfig, GifFrame, GifRecorder, Rgb};

const OUTPUT: &str = "media/gallery/lidar_loop_closure.gif";
const FRAME_EVERY: usize = 5;
const WALL: Rgb = (70, 70, 70);
const MAP: Rgb = (110, 155, 225);
const TRUTH: Rgb = (175, 175, 175);
const FRONT_END: Rgb = (232, 131, 58);
const SLAM: Rgb = (53, 170, 110);
const LOOP: Rgb = (205, 60, 190);
const SCAN: Rgb = (221, 51, 85);

fn draw_poses(frame: &mut GifFrame, poses: &[Pose2D], color: Rgb, width: f64) {
    let xs: Vec<f64> = poses.iter().map(|pose| pose.x).collect();
    let ys: Vec<f64> = poses.iter().map(|pose| pose.y).collect();
    frame.draw_path_xy(&xs, &ys, color, width);
}

fn render(
    run: &CorridorLoopRun,
    cfg: &GifCanvasConfig,
    index: usize,
    final_frame: bool,
) -> GifFrame {
    let state = &run.frames[index];
    let node_poses = if final_frame {
        &run.final_node_poses
    } else {
        &state.node_poses
    };
    let mut frame = GifFrame::new(cfg);

    for wall in &run.walls {
        frame.draw_segment(
            (wall.start.x, wall.start.y),
            (wall.end.x, wall.end.y),
            WALL,
            1.4,
        );
    }
    for (pose, scan) in node_poses.iter().zip(&run.node_scans) {
        for point in transform_scan_to_world(scan, *pose).iter().step_by(2) {
            frame.draw_point(point.x, point.y, MAP, 0.7);
        }
    }

    let history = &run.frames[..=index];
    let truth: Vec<Pose2D> = history.iter().map(|f| f.truth).collect();
    let front_end: Vec<Pose2D> = history.iter().map(|f| f.front_end).collect();
    draw_poses(&mut frame, &truth, TRUTH, 1.2);
    draw_poses(&mut frame, &front_end, FRONT_END, 1.6);
    draw_poses(&mut frame, node_poses, SLAM, 2.0);
    for pose in node_poses {
        frame.draw_point(pose.x, pose.y, SLAM, 1.6);
    }
    for closure in run
        .loop_closures
        .iter()
        .filter(|closure| closure.to < node_poses.len())
    {
        let (a, b) = (node_poses[closure.from], node_poses[closure.to]);
        frame.draw_segment((a.x, a.y), (b.x, b.y), LOOP, 1.6);
    }

    let estimate = if final_frame {
        run.final_estimate
    } else {
        state.estimate
    };
    for point in transform_scan_to_world(&state.scan, estimate) {
        frame.draw_point(point.x, point.y, SCAN, 0.9);
    }
    frame.draw_robot(&state.front_end, 0.45, FRONT_END);
    frame.draw_robot(&estimate, 0.45, SLAM);
    frame
}

fn main() {
    let run = run_corridor_loop(
        &CorridorLoopConfig::default(),
        LidarGraphSlamConfig::default(),
    );
    let cfg = GifCanvasConfig::new(640, 440, (-16.0, 16.0), (-11.0, 11.0))
        .with_delay_cs(6)
        .with_grid_step(None);
    let mut recorder = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let last = run.frames.len() - 1;
    for index in 0..last {
        let state = &run.frames[index];
        if state.optimized {
            // Show the drifted graph, then hold on the corrected one.
            if index > 0 {
                recorder
                    .add_frame_with_delay(render(&run, &cfg, index - 1, false), 80)
                    .expect("write frame");
            }
            recorder
                .add_frame_with_delay(render(&run, &cfg, index, false), 150)
                .expect("write frame");
        } else if index % FRAME_EVERY == 0 {
            recorder
                .add_frame(render(&run, &cfg, index, false))
                .expect("write frame");
        }
    }
    recorder
        .add_frame_with_delay(render(&run, &cfg, last, true), 300)
        .expect("write frame");
    recorder.finish().expect("finalize gif");
    println!(
        "Saved {OUTPUT} ({} steps, {} loop closures)",
        run.frames.len(),
        run.loop_closures.len()
    );
}
