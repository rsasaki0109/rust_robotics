//! Render RRT tree growth as a looping animated GIF.
//!
//! The planner runs once, then the recorded tree is replayed node by node,
//! finishing with the found path. Output: `media/gallery/rrt.gif`.
//!
//! ```bash
//! cargo run -p rust_robotics --example render_gif_rrt --features "planning,gif"
//! ```

use rust_robotics::planning::RRTPlanner;
use rust_robotics::viz::{colors, rgb, GifCanvasConfig, GifFrame, GifRecorder};

const OUTPUT: &str = "media/gallery/rrt.gif";
const NODES_PER_FRAME: usize = 2;

fn main() {
    let obstacles = vec![
        (5.0, 5.0, 1.0),
        (3.0, 6.0, 2.0),
        (3.0, 8.0, 2.0),
        (3.0, 10.0, 2.0),
        (7.0, 5.0, 2.0),
        (9.0, 5.0, 2.0),
        (8.0, 10.0, 1.0),
    ];
    let start = [0.0, 0.0];
    let goal = [6.0, 10.0];

    let mut planner =
        RRTPlanner::from_obstacles(obstacles.clone(), [-2.0, 15.0], 3.0, 0.5, 5, 500, None, 0.8);
    let path = planner.planning(start, goal).expect("RRT found a path");
    let tree = planner.get_tree();

    let cfg = GifCanvasConfig::new(480, 480, (-2.5, 15.5), (-2.5, 15.5))
        .with_delay_cs(8)
        .with_grid_step(Some(5.0));
    let mut rec = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let draw_static = |frame: &mut GifFrame| {
        for &(x, y, r) in &obstacles {
            frame.fill_circle(x, y, r, (60, 60, 60));
        }
        frame.draw_point(start[0], start[1], rgb(colors::START), 5.0);
        frame.draw_cross(goal[0], goal[1], rgb(colors::GOAL), 6.0);
    };

    // Replay tree growth.
    let mut shown = 1;
    while shown < tree.len() {
        shown = (shown + NODES_PER_FRAME).min(tree.len());
        let mut frame = GifFrame::new(&cfg);
        draw_static(&mut frame);
        for node in &tree[..shown] {
            if node.parent.is_some() && node.path_x.len() >= 2 {
                frame.draw_path_xy(&node.path_x, &node.path_y, (120, 200, 120), 1.0);
            }
        }
        rec.add_frame(frame).expect("write frame");
    }

    // Final frames: highlight the path.
    let px: Vec<f64> = path.iter().map(|p| p[0]).collect();
    let py: Vec<f64> = path.iter().map(|p| p[1]).collect();
    let mut frame = GifFrame::new(&cfg);
    draw_static(&mut frame);
    for node in tree {
        if node.parent.is_some() && node.path_x.len() >= 2 {
            frame.draw_path_xy(&node.path_x, &node.path_y, (120, 200, 120), 1.0);
        }
    }
    frame.draw_path_xy(&px, &py, rgb(colors::PATH), 2.5);
    rec.add_frame_with_delay(frame, 200).expect("write frame");

    rec.finish().expect("finalize gif");
    println!("Saved {} ({} tree nodes)", OUTPUT, tree.len());
}
