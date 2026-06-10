//! Render the pure pursuit path tracking demo as a looping animated GIF.
//!
//! A bicycle-model vehicle tracks a sinusoidal course with the pure pursuit
//! steering law. Output: `media/gallery/pure_pursuit.gif`.
//!
//! ```bash
//! cargo run -p rust_robotics --example render_gif_pure_pursuit --features "control,gif"
//! ```

use rust_robotics::control::pure_pursuit::{PurePursuitController, VehicleState};
use rust_robotics::core::{Path2D, Point2D, Pose2D};
use rust_robotics::viz::{colors, rgb, GifCanvasConfig, GifFrame, GifRecorder};

const DT: f64 = 0.1;
const MAX_STEPS: usize = 600;
const FRAME_EVERY: usize = 3;
const TARGET_SPEED: f64 = 10.0 / 3.6;
const OUTPUT: &str = "media/gallery/pure_pursuit.gif";

fn main() {
    // Classic PythonRobotics course: y = sin(x / 5) * x / 2
    let course_x: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
    let course_y: Vec<f64> = course_x.iter().map(|x| (x / 5.0).sin() * x / 2.0).collect();

    let mut controller = PurePursuitController::with_params(0.1, 2.0, 2.9);
    controller.set_path(Path2D::from_points(
        course_x
            .iter()
            .zip(course_y.iter())
            .map(|(&x, &y)| Point2D::new(x, y))
            .collect(),
    ));

    let mut state = VehicleState::new(0.0, -3.0, 0.0, 0.0, 2.9);

    let cfg = GifCanvasConfig::new(560, 420, (-5.0, 53.0), (-22.0, 27.0)).with_delay_cs(6);
    let mut rec = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let mut trail = (Vec::new(), Vec::new());

    for k in 0..MAX_STEPS {
        let accel = controller.compute_acceleration(TARGET_SPEED, state.v);
        let steer = controller.compute_steering(&state);
        state.update(accel, steer, DT);
        trail.0.push(state.x);
        trail.1.push(state.y);

        let reached = controller.is_goal_reached(&state);

        let last = reached || k + 1 == MAX_STEPS;
        if k % FRAME_EVERY == 0 || last {
            let mut frame = GifFrame::new(&cfg);
            frame.draw_path_xy(&course_x, &course_y, (190, 190, 190), 1.6);
            frame.draw_path_xy(&trail.0, &trail.1, rgb(colors::ESTIMATED), 2.0);
            frame.draw_robot(
                &Pose2D::new(state.x, state.y, state.yaw),
                1.1,
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
