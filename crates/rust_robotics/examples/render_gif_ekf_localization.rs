//! Render the EKF localization demo as a looping animated GIF.
//!
//! A robot drives a circle with noisy odometry and a noisy GPS-like position
//! sensor; the EKF estimate and its 2-sigma covariance ellipse are drawn each
//! step. Output: `media/gallery/ekf_localization.gif`.
//!
//! ```bash
//! cargo run -p rust_robotics --example render_gif_ekf_localization --features "localization,gif"
//! ```

use nalgebra::Matrix2;
use rust_robotics::core::{ControlInput, Point2D, State2D};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::viz::{colors, rgb, GifCanvasConfig, GifFrame, GifRecorder};

const DT: f64 = 0.1;
const STEPS: usize = 330;
const FRAME_EVERY: usize = 3;
const OUTPUT: &str = "media/gallery/ekf_localization.gif";

fn deterministic_noise(k: usize, scale: f64, phase: f64) -> f64 {
    let t = k as f64;
    scale * (0.13 * t + phase).sin() + 0.5 * scale * (0.07 * t + 1.3 * phase).cos()
}

fn covariance_ellipse(cov: &Matrix2<f64>) -> (f64, f64, f64) {
    let eigen = cov.symmetric_eigen();
    let l0 = eigen.eigenvalues[0].max(0.0);
    let l1 = eigen.eigenvalues[1].max(0.0);
    let v0 = eigen.eigenvectors.column(0);
    let k = 2.0;
    (k * l0.sqrt(), k * l1.sqrt(), v0[1].atan2(v0[0]))
}

fn main() {
    let mut ekf = EKFLocalizer::with_initial_state_2d(
        State2D::new(10.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0),
        EKFConfig::default(),
    )
    .expect("valid EKF config");

    let mut truth = State2D::new(10.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0);
    let (v_true, w_true) = (1.0, 0.1);

    let cfg = GifCanvasConfig::new(480, 480, (-14.0, 14.0), (-4.0, 24.0)).with_delay_cs(6);
    let mut rec = GifRecorder::new(OUTPUT, cfg.clone()).expect("create recorder");

    let mut gt = (Vec::new(), Vec::new());
    let mut est = (Vec::new(), Vec::new());
    let mut meas = (Vec::new(), Vec::new());

    for k in 0..STEPS {
        truth.x += v_true * truth.yaw.cos() * DT;
        truth.y += v_true * truth.yaw.sin() * DT;
        truth.yaw += w_true * DT;

        let control = ControlInput::new(
            v_true + deterministic_noise(k, 0.12, 0.2),
            w_true + deterministic_noise(k, 0.04, 1.0),
        );
        let z = Point2D::new(
            truth.x + deterministic_noise(k, 0.6, 2.0),
            truth.y + deterministic_noise(k, 0.6, 2.7),
        );

        let state = ekf
            .estimate_state(z, control, DT)
            .unwrap_or_else(|_| ekf.state_2d());

        gt.0.push(truth.x);
        gt.1.push(truth.y);
        est.0.push(state.x);
        est.1.push(state.y);
        meas.0.push(z.x);
        meas.1.push(z.y);

        if k % FRAME_EVERY != 0 {
            continue;
        }

        let mut frame = GifFrame::new(&cfg);
        frame.draw_points_xy(&meas.0, &meas.1, (200, 200, 200), 1.6);
        frame.draw_path_xy(&gt.0, &gt.1, rgb(colors::GROUND_TRUTH), 1.6);
        frame.draw_path_xy(&est.0, &est.1, rgb(colors::ESTIMATED), 2.0);

        let cov = ekf.get_covariance_matrix();
        let pos_cov = Matrix2::new(cov[(0, 0)], cov[(0, 1)], cov[(1, 0)], cov[(1, 1)]);
        let (a, b, angle) = covariance_ellipse(&pos_cov);
        frame.draw_ellipse(state.x, state.y, a, b, angle, rgb(colors::ESTIMATED), 1.5);

        frame.draw_robot(
            &rust_robotics::core::Pose2D::new(truth.x, truth.y, truth.yaw),
            0.6,
            rgb(colors::GROUND_TRUTH),
        );
        frame.draw_robot(
            &rust_robotics::core::Pose2D::new(state.x, state.y, state.yaw),
            0.6,
            rgb(colors::ESTIMATED),
        );

        let last = k + FRAME_EVERY >= STEPS;
        rec.add_frame_with_delay(frame, if last { 150 } else { cfg.delay_cs })
            .expect("write frame");
    }

    rec.finish().expect("finalize gif");
    println!("Saved {}", OUTPUT);
}
