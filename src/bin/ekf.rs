use std::fs;

use gnuplot::{AxesCommon, Caption, Color, Figure, PointSymbol};
use rust_robotics::localization::{EKFControl, EKFLocalizer, EKFMeasurement, EKFState};
use rust_robotics::StateEstimator;

fn propagate(state: &mut EKFState, control: &EKFControl, dt: f64) {
    let yaw = state[2];
    state[0] += control[0] * yaw.cos() * dt;
    state[1] += control[0] * yaw.sin() * dt;
    state[2] += control[1] * dt;
    state[3] = control[0];
}

fn main() {
    println!("EKF localization demo start");

    fs::create_dir_all("img/localization").unwrap();

    let dt = 0.1;
    let n_steps = 100;
    let control = EKFControl::new(1.0, 0.1);

    let mut ekf = EKFLocalizer::with_defaults();
    let mut true_state = EKFState::zeros();
    let mut dead_reckoning = EKFState::zeros();

    let mut true_x = Vec::with_capacity(n_steps + 1);
    let mut true_y = Vec::with_capacity(n_steps + 1);
    let mut est_x = Vec::with_capacity(n_steps + 1);
    let mut est_y = Vec::with_capacity(n_steps + 1);
    let mut gps_x = Vec::with_capacity(n_steps + 1);
    let mut gps_y = Vec::with_capacity(n_steps + 1);
    let mut dr_x = Vec::with_capacity(n_steps + 1);
    let mut dr_y = Vec::with_capacity(n_steps + 1);

    true_x.push(true_state[0]);
    true_y.push(true_state[1]);
    est_x.push(ekf.get_state()[0]);
    est_y.push(ekf.get_state()[1]);
    gps_x.push(true_state[0]);
    gps_y.push(true_state[1]);
    dr_x.push(dead_reckoning[0]);
    dr_y.push(dead_reckoning[1]);

    for step in 0..n_steps {
        propagate(&mut true_state, &control, dt);

        let gps_noise_x = ((step % 9) as f64 - 4.0) * 0.05;
        let gps_noise_y = (((step * 2) % 11) as f64 - 5.0) * 0.04;
        let measurement =
            EKFMeasurement::new(true_state[0] + gps_noise_x, true_state[1] + gps_noise_y);

        let dr_control = EKFControl::new(
            control[0] + (((step * 3) % 7) as f64 - 3.0) * 0.01,
            control[1] + (((step * 5) % 9) as f64 - 4.0) * 0.002,
        );
        propagate(&mut dead_reckoning, &dr_control, dt);

        ekf.estimate(&measurement, &control, dt).unwrap();

        true_x.push(true_state[0]);
        true_y.push(true_state[1]);
        est_x.push(ekf.get_state()[0]);
        est_y.push(ekf.get_state()[1]);
        gps_x.push(measurement[0]);
        gps_y.push(measurement[1]);
        dr_x.push(dead_reckoning[0]);
        dr_y.push(dead_reckoning[1]);
    }

    let mut fig = Figure::new();
    {
        let axes = fig.axes2d();
        axes.lines(&gps_x, &gps_y, &[Caption("GPS"), Color("red")]);
        axes.lines(&true_x, &true_y, &[Caption("Ground Truth"), Color("blue")]);
        axes.lines(&est_x, &est_y, &[Caption("EKF"), Color("green")]);
        axes.lines(&dr_x, &dr_y, &[Caption("Dead Reckoning"), Color("yellow")]);
        axes.points(
            [true_x[0]],
            [true_y[0]],
            &[Caption("Start"), Color("black"), PointSymbol('O')],
        );
        axes.set_title("Extended Kalman Filter Localization", &[])
            .set_x_label("X [m]", &[])
            .set_y_label("Y [m]", &[])
            .set_aspect_ratio(gnuplot::Fix(1.0));
    }

    fig.save_to_svg("./img/localization/ekf.svg", 640, 480)
        .unwrap();
    println!("Saved plot to ./img/localization/ekf.svg");
}
