use proptest::prelude::*;
use rust_robotics_localization::ekf::{EKFConfig, EKFLocalizer};
use rust_robotics_localization::particle_filter::{ParticleFilterConfig, ParticleFilterLocalizer};
use rust_robotics_localization::unscented_kalman_filter::{UKFConfig, UKFLocalizer};

use nalgebra::Vector2;

const DT: f64 = 0.1;

fn run_ekf_steps(steps: usize, v: f64, yaw_rate: f64) -> (f64, f64) {
    let config = EKFConfig::default();
    let mut ekf = EKFLocalizer::new(config);
    let control = Vector2::new(v, yaw_rate);
    let measurement = Vector2::new(0.0, 0.0);

    let mut last_state = Vector2::new(0.0, 0.0);
    for _ in 0..steps {
        if let Ok(state) = ekf.estimate(&measurement, &control, DT) {
            last_state = Vector2::new(state[0], state[1]);
        }
    }

    (last_state[0], last_state[1])
}

fn run_ukf_steps(steps: usize, v: f64, yaw_rate: f64) -> (f64, f64) {
    let config = UKFConfig::default();
    let mut ukf = UKFLocalizer::new(config);
    let control = Vector2::new(v, yaw_rate);
    let measurement = Vector2::new(0.0, 0.0);

    for _ in 0..steps {
        ukf.step(&control, &measurement);
    }

    let state = ukf.estimate();
    (state[0], state[1])
}

fn run_pf_steps(steps: usize, v: f64, yaw_rate: f64) -> (f64, f64) {
    let config = ParticleFilterConfig::default();
    let mut pf = ParticleFilterLocalizer::new(config);
    let control = Vector2::new(v, yaw_rate);
    let observations: Vec<(f64, f64, f64)> = vec![];

    for _ in 0..steps {
        pf.step(&control, &observations);
    }

    let state = pf.estimate();
    (state[0], state[1])
}

proptest! {
    #[test]
    fn ekf_does_not_diverge(
        steps in 1..50usize,
        v in -2.0..2.0f64,
        yaw_rate in -1.0..1.0f64,
    ) {
        let (x, y) = run_ekf_steps(steps, v, yaw_rate);
        prop_assert!(x.is_finite(), "EKF x diverged: {}", x);
        prop_assert!(y.is_finite(), "EKF y diverged: {}", y);
    }

    #[test]
    fn ukf_does_not_diverge(
        steps in 1..50usize,
        v in -2.0..2.0f64,
        yaw_rate in -1.0..1.0f64,
    ) {
        let (x, y) = run_ukf_steps(steps, v, yaw_rate);
        prop_assert!(x.is_finite(), "UKF x diverged: {}", x);
        prop_assert!(y.is_finite(), "UKF y diverged: {}", y);
    }

    #[test]
    fn pf_does_not_diverge(
        steps in 1..50usize,
        v in -2.0..2.0f64,
        yaw_rate in -1.0..1.0f64,
    ) {
        let (x, y) = run_pf_steps(steps, v, yaw_rate);
        prop_assert!(x.is_finite(), "PF x diverged: {}", x);
        prop_assert!(y.is_finite(), "PF y diverged: {}", y);
    }

    #[test]
    fn ekf_bounded_state(
        steps in 1..30usize,
        v in 0.5..1.5f64,
        yaw_rate in -0.5..0.5f64,
    ) {
        let (x, y) = run_ekf_steps(steps, v, yaw_rate);
        let distance = (x * x + y * y).sqrt();
        let max_travel = v * DT * steps as f64 * 2.0;
        prop_assert!(
            distance <= max_travel + 10.0,
            "EKF state too far: distance={}, max_travel={}",
            distance, max_travel
        );
    }
}
