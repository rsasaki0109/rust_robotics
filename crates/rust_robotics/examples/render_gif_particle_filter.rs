//! Render the particle filter localization demo as a looping animated GIF.
//!
//! A robot drives among range-only landmarks; the particle cloud, landmark
//! beacons, and true vs. estimated trajectories are drawn each step.
//! Output: `media/gallery/particle_filter.gif`.
//!
//! ```bash
//! cargo run -p rust_robotics --example render_gif_particle_filter --features "localization,gif"
//! ```

use rand::{Rng, SeedableRng};
use rust_robotics::core::{ControlInput, Obstacles, Point2D, Pose2D, State2D};
use rust_robotics::localization::{ParticleFilterConfig, ParticleFilterLocalizer};
use rust_robotics::viz::{colors, rgb, GifCanvasConfig, GifFrame, GifRecorder};

const DT: f64 = 0.1;
const STEPS: usize = 300;
const FRAME_EVERY: usize = 3;
const OUTPUT: &str = "media/gallery/particle_filter.gif";

fn main() {
    let mut rng = rand::rngs::StdRng::seed_from_u64(42);
    let normal = rand_distr::Normal::new(0.0, 0.15).unwrap();

    let landmarks = Obstacles::from_points(vec![
        Point2D::new(2.0, 2.0),
        Point2D::new(10.0, 2.0),
        Point2D::new(2.0, 8.0),
        Point2D::new(10.0, 8.0),
        Point2D::new(6.0, 5.0),
    ]);

    let mut pf = ParticleFilterLocalizer::with_initial_state_2d(
        State2D::new(5.0, 5.0, 0.0, 0.0),
        ParticleFilterConfig {
            n_particles: 150,
            dt: DT,
            range_noise: 0.25,
            ..ParticleFilterConfig::default()
        },
    )
    .expect("valid PF config");
    pf.set_landmarks_from_obstacles(&landmarks)
        .expect("valid landmarks");

    let mut truth = State2D::new(5.0, 5.0, 0.0, 0.0);

    let cfg = GifCanvasConfig::new(480, 480, (-1.0, 13.0), (-1.0, 11.0))
        .with_delay_cs(6)
        .with_grid_step(Some(2.0));
    let mut rec = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let mut gt = (Vec::new(), Vec::new());
    let mut est = (Vec::new(), Vec::new());

    for k in 0..STEPS {
        // Drive a rounded rectangle: straight, then turn at the corners.
        let control = if (k / 25) % 2 == 0 {
            ControlInput::new(1.1, 0.0)
        } else {
            ControlInput::new(0.5, 0.63)
        };

        truth.x += control.v * truth.yaw.cos() * DT;
        truth.y += control.v * truth.yaw.sin() * DT;
        truth.yaw += control.omega * DT;

        let obs: Vec<(f64, f64, f64)> = landmarks
            .points
            .iter()
            .map(|lm| {
                let range = ((truth.x - lm.x).powi(2) + (truth.y - lm.y).powi(2)).sqrt();
                ((range + rng.sample(normal)).max(0.0), lm.x, lm.y)
            })
            .collect();

        let state = pf
            .try_step_state(control, &obs)
            .unwrap_or_else(|_| pf.state_2d());

        gt.0.push(truth.x);
        gt.1.push(truth.y);
        est.0.push(state.x);
        est.1.push(state.y);

        if k % FRAME_EVERY != 0 {
            continue;
        }

        let mut frame = GifFrame::new(&cfg);
        for lm in &landmarks.points {
            frame.fill_circle(lm.x, lm.y, 0.16, (220, 180, 40));
            frame.draw_circle(lm.x, lm.y, 0.34, (220, 180, 40), 1.5);
        }

        for particle in pf.get_particles() {
            frame.draw_point(particle.x, particle.y, (140, 190, 255), 1.4);
        }

        frame.draw_path_xy(&gt.0, &gt.1, rgb(colors::GROUND_TRUTH), 1.6);
        frame.draw_path_xy(&est.0, &est.1, rgb(colors::ESTIMATED), 2.0);
        frame.draw_robot(
            &Pose2D::new(truth.x, truth.y, truth.yaw),
            0.35,
            rgb(colors::GROUND_TRUTH),
        );
        frame.draw_robot(
            &Pose2D::new(state.x, state.y, state.yaw),
            0.35,
            rgb(colors::ESTIMATED),
        );

        let last = k + FRAME_EVERY >= STEPS;
        rec.add_frame_with_delay(frame, if last { 150 } else { cfg.delay_cs })
            .expect("write frame");
    }

    rec.finish().expect("finalize gif");
    println!("Saved {}", OUTPUT);
}
