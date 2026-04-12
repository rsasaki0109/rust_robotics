//! Localization filter comparison visualization.
//!
//! Simulates noisy circular motion and compares EKF, IEKF, and
//! Complementary Filter trajectories. Output:
//! `img/localization/filter_comparison.png`.

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSymbol};
use nalgebra::Vector2;
use rust_robotics::core::StateEstimator;
use rust_robotics::localization::complementary_filter::{
    CFControl, CFMeasurement, ComplementaryFilter, ComplementaryFilterConfig,
};
use rust_robotics::localization::ekf::{EKFControl, EKFLocalizer, EKFMeasurement};
use rust_robotics::localization::iterated_ekf::{IEKFControl, IEKFLocalizer, IEKFMeasurement};

const DT: f64 = 0.1;
const STEPS: usize = 360;
const V_TRUE: f64 = 1.0;
const W_TRUE: f64 = 0.2;

#[derive(Clone, Copy)]
struct TrueState {
    x: f64,
    y: f64,
    yaw: f64,
}

fn deterministic_noise(k: usize, scale: f64, phase: f64) -> f64 {
    let t = k as f64;
    scale * (0.13 * t + phase).sin() + 0.5 * scale * (0.07 * t + 1.3 * phase).cos()
}

fn main() {
    std::fs::create_dir_all("img/localization")
        .expect("failed to create img/localization directory");

    let mut ekf = EKFLocalizer::with_defaults();
    let mut iekf = IEKFLocalizer::new(Default::default());
    let mut cf = ComplementaryFilter::new(ComplementaryFilterConfig {
        alpha: 0.92,
        dt: DT,
    });

    let mut truth = TrueState {
        x: 5.0,
        y: 0.0,
        yaw: std::f64::consts::FRAC_PI_2,
    };

    let mut gt_x = Vec::with_capacity(STEPS);
    let mut gt_y = Vec::with_capacity(STEPS);
    let mut meas_x = Vec::with_capacity(STEPS);
    let mut meas_y = Vec::with_capacity(STEPS);
    let mut ekf_x = Vec::with_capacity(STEPS);
    let mut ekf_y = Vec::with_capacity(STEPS);
    let mut iekf_x = Vec::with_capacity(STEPS);
    let mut iekf_y = Vec::with_capacity(STEPS);
    let mut cf_x = Vec::with_capacity(STEPS);
    let mut cf_y = Vec::with_capacity(STEPS);

    for k in 0..STEPS {
        truth.x += V_TRUE * truth.yaw.cos() * DT;
        truth.y += V_TRUE * truth.yaw.sin() * DT;
        truth.yaw += W_TRUE * DT;

        let noisy_v = V_TRUE + deterministic_noise(k, 0.10, 0.2);
        let noisy_w = W_TRUE + deterministic_noise(k, 0.03, 1.0);
        let z_x = truth.x + deterministic_noise(k, 0.45, 2.0);
        let z_y = truth.y + deterministic_noise(k, 0.45, 2.7);

        let control = Vector2::new(noisy_v, noisy_w);
        let measurement = Vector2::new(z_x, z_y);

        let _ = ekf.estimate(
            &EKFMeasurement::new(measurement[0], measurement[1]),
            &EKFControl::new(control[0], control[1]),
            DT,
        );
        let _ = iekf.estimate(
            &IEKFMeasurement::new(measurement[0], measurement[1]),
            &IEKFControl::new(control[0], control[1]),
            DT,
        );
        cf.predict(&CFControl::new(control[0], control[1]), DT);
        cf.update(&CFMeasurement::new(measurement[0], measurement[1]));

        let ekf_state = ekf.state_2d();
        let iekf_state = iekf.state_2d();
        let cf_state = cf.state_2d();

        gt_x.push(truth.x);
        gt_y.push(truth.y);
        meas_x.push(z_x);
        meas_y.push(z_y);
        ekf_x.push(ekf_state.x);
        ekf_y.push(ekf_state.y);
        iekf_x.push(iekf_state.x);
        iekf_y.push(iekf_state.y);
        cf_x.push(cf_state.x);
        cf_y.push(cf_state.y);
    }

    let mut fg = Figure::new();
    {
        fg.axes2d()
            .set_title("Filter Comparison: Circular Trajectory", &[])
            .set_x_label("x [m]", &[])
            .set_y_label("y [m]", &[])
            .set_aspect_ratio(gnuplot::AutoOption::Fix(1.0))
            .points(
                &meas_x,
                &meas_y,
                &[
                    Caption("Noisy Measurements"),
                    Color("gray"),
                    PointSymbol('.'),
                ],
            )
            .lines(&gt_x, &gt_y, &[Caption("Ground Truth"), Color("black")])
            .lines(&ekf_x, &ekf_y, &[Caption("EKF"), Color("blue")])
            .lines(&iekf_x, &iekf_y, &[Caption("IEKF"), Color("red")])
            .lines(
                &cf_x,
                &cf_y,
                &[Caption("Complementary Filter"), Color("green")],
            );
    }

    let output_path = "img/localization/filter_comparison.png";
    fg.set_terminal("pngcairo", output_path);
    fg.show()
        .expect("failed to render gnuplot figure for filter comparison");
    println!("Saved {}", output_path);
}
