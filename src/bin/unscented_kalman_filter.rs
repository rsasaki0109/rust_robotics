use rust_robotics::localization::{UKFLocalizer, UKFState};
use rust_robotics::localization::unscented_kalman_filter::{calc_input, observation};
use rust_robotics::StateEstimator;

fn main() {
    let mut ukf = UKFLocalizer::with_defaults();
    let mut x_true = UKFState::zeros();
    let mut x_dr = UKFState::zeros();

    for _ in 0..50 {
        let u = calc_input();
        let (z, u_noisy) = observation(&mut x_true, &mut x_dr, &u);
        ukf.predict(&u_noisy, 0.1);
        ukf.update(&z);
    }

    let estimate = ukf.estimate();
    println!(
        "UKF estimate: x={:.3}, y={:.3}, yaw={:.3}, v={:.3}",
        estimate[0], estimate[1], estimate[2], estimate[3]
    );
}
