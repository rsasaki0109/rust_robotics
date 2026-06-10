//! Render the Dynamic Window Approach demo as a looping animated GIF.
//!
//! A robot navigates an obstacle field to the goal; the best predicted
//! trajectory of each step is drawn ahead of the robot.
//! Output: `media/gallery/dwa.gif`.
//!
//! ```bash
//! cargo run -p rust_robotics --example render_gif_dwa --features "planning,gif"
//! ```

use rust_robotics::core::{Point2D, Pose2D, State2D};
use rust_robotics::planning::{DWAConfig, DWAPlanner};
use rust_robotics::viz::{colors, rgb, GifCanvasConfig, GifFrame, GifRecorder};

const MAX_STEPS: usize = 500;
const FRAME_EVERY: usize = 3;
const OUTPUT: &str = "media/gallery/dwa.gif";

fn main() {
    let obstacles = vec![
        (-1.0, -1.0),
        (0.0, 2.0),
        (4.0, 2.0),
        (5.0, 4.0),
        (5.0, 5.0),
        (5.0, 6.0),
        (5.0, 9.0),
        (8.0, 9.0),
        (7.0, 9.0),
        (8.0, 10.0),
        (9.0, 11.0),
        (12.0, 13.0),
        (12.0, 12.0),
        (15.0, 15.0),
        (13.0, 13.0),
    ];

    let mut planner = DWAPlanner::new(DWAConfig {
        robot_radius: 0.6,
        ..DWAConfig::default()
    });
    planner.set_state_from_2d(
        &State2D::new(0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0),
        0.0,
    );
    planner.set_goal(Point2D::new(10.0, 10.0));
    planner.set_obstacles(obstacles.iter().map(|&(x, y)| Point2D::new(x, y)).collect());

    let cfg = GifCanvasConfig::new(480, 480, (-3.0, 16.0), (-3.0, 16.0))
        .with_delay_cs(6)
        .with_grid_step(Some(5.0));
    let mut rec = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let mut trail = (Vec::new(), Vec::new());

    for k in 0..MAX_STEPS {
        planner.step();
        let state = planner.state_2d();
        trail.0.push(state.x);
        trail.1.push(state.y);

        let reached = planner.is_goal_reached();
        let last = reached || k + 1 == MAX_STEPS;
        if k % FRAME_EVERY == 0 || last {
            let mut frame = GifFrame::new(&cfg);
            for &(x, y) in &obstacles {
                frame.fill_circle(x, y, 0.3, rgb(colors::OBSTACLE));
            }
            frame.draw_cross(10.0, 10.0, rgb(colors::GOAL), 6.0);
            frame.draw_path(
                &planner.get_best_trajectory().to_path(),
                (120, 220, 120),
                1.6,
            );
            frame.draw_path_xy(&trail.0, &trail.1, rgb(colors::ESTIMATED), 2.0);
            frame.draw_robot(
                &Pose2D::new(state.x, state.y, state.yaw),
                0.6,
                rgb(colors::PATH),
            );

            rec.add_frame_with_delay(frame, if last { 150 } else { cfg.delay_cs })
                .expect("write frame");
        }

        if reached {
            break;
        }
    }

    rec.finish().expect("finalize gif");
    println!("Saved {}", OUTPUT);
}
