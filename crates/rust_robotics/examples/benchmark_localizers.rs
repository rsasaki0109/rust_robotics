//! Benchmark localization filters on a circular trajectory.
//!
//! Run with:
//!   cargo run -p rust_robotics --example benchmark_localizers --features localization --release

use std::time::Instant;

use rust_robotics::core::StateEstimator;
use rust_robotics::localization::complementary_filter::{
    CFControl, CFMeasurement, ComplementaryFilter, ComplementaryFilterConfig,
};
use rust_robotics::localization::information_filter::{
    IFControl, IFMeasurement, InformationFilter, InformationFilterConfig,
};
use rust_robotics::localization::iterated_ekf::{
    IEKFConfig, IEKFControl, IEKFLocalizer, IEKFMeasurement,
};
use rust_robotics::localization::square_root_ukf::{
    SRUKFConfig, SRUKFControl, SRUKFLocalizer, SRUKFMeasurement,
};
use rust_robotics::localization::{EKFConfig, EKFControl, EKFLocalizer, EKFMeasurement};
use rust_robotics::localization::{UKFConfig, UKFControl, UKFLocalizer, UKFMeasurement};
use rust_robotics::prelude::*;

const STEPS: usize = 1000;
const DT: f64 = 0.05;

#[derive(Debug)]
struct LocalizerRow {
    name: &'static str,
    final_pos_error: f64,
    avg_step_time_us: f64,
}

fn measurement_noise(step: usize) -> (f64, f64) {
    let t = step as f64;
    (0.03 * (t * 0.31).sin(), 0.03 * (t * 0.27).cos())
}

fn run_truth(control: ControlInput) -> Vec<State2D> {
    let mut states = Vec::with_capacity(STEPS);
    let mut state = State2D::origin();
    for _ in 0..STEPS {
        state.x += control.v * state.yaw.cos() * DT;
        state.y += control.v * state.yaw.sin() * DT;
        state.yaw += control.omega * DT;
        state.v = control.v;
        states.push(state);
    }
    states
}

fn position_error(estimate: State2D, truth: State2D) -> f64 {
    let dx = estimate.x - truth.x;
    let dy = estimate.y - truth.y;
    (dx * dx + dy * dy).sqrt()
}

fn main() -> RoboticsResult<()> {
    let control = ControlInput::new(1.0, 2.0 * std::f64::consts::PI / (STEPS as f64 * DT));
    let truth = run_truth(control);
    let final_truth = *truth
        .last()
        .ok_or_else(|| RoboticsError::PlanningError("empty truth trajectory".to_string()))?;

    let mut rows = Vec::new();

    {
        let mut ekf = EKFLocalizer::new(EKFConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            let measurement = EKFMeasurement::new(true_state.x + nx, true_state.y + ny);
            let control_v = EKFControl::new(control.v, control.omega);
            let _ = ekf.estimate(&measurement, &control_v, DT)?;
        }
        rows.push(LocalizerRow {
            name: "EKF",
            final_pos_error: position_error(ekf.state_2d(), final_truth),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    {
        let mut ukf = UKFLocalizer::new(UKFConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            let measurement = UKFMeasurement::new(true_state.x + nx, true_state.y + ny);
            let control_v = UKFControl::new(control.v, control.omega);
            let _ = ukf.try_step(&control_v, &measurement)?;
        }
        rows.push(LocalizerRow {
            name: "UKF",
            final_pos_error: position_error(ukf.state_2d(), final_truth),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    {
        let mut iekf = IEKFLocalizer::new(IEKFConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            let measurement = IEKFMeasurement::new(true_state.x + nx, true_state.y + ny);
            let control_v = IEKFControl::new(control.v, control.omega);
            let _ = iekf.estimate(&measurement, &control_v, DT)?;
        }
        rows.push(LocalizerRow {
            name: "IEKF",
            final_pos_error: position_error(iekf.state_2d(), final_truth),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    {
        let mut cf = ComplementaryFilter::new(ComplementaryFilterConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            cf.predict(&CFControl::new(control.v, control.omega), DT);
            cf.update(&CFMeasurement::new(true_state.x + nx, true_state.y + ny));
        }
        rows.push(LocalizerRow {
            name: "Complementary Filter",
            final_pos_error: position_error(cf.state_2d(), final_truth),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    {
        let mut info = InformationFilter::new(InformationFilterConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            info.predict(&IFControl::new(control.v, control.omega), DT);
            info.update(&IFMeasurement::new(true_state.x + nx, true_state.y + ny));
        }
        let info_state = info.get_decoded_state();
        rows.push(LocalizerRow {
            name: "Information Filter",
            final_pos_error: position_error(
                State2D::new(info_state[0], info_state[1], info_state[2], info_state[3]),
                final_truth,
            ),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    {
        let mut sr_ukf = SRUKFLocalizer::new(SRUKFConfig::default());
        let t0 = Instant::now();
        for (step, true_state) in truth.iter().enumerate() {
            let (nx, ny) = measurement_noise(step);
            sr_ukf.predict(&SRUKFControl::new(control.v, control.omega), DT);
            sr_ukf.update(&SRUKFMeasurement::new(true_state.x + nx, true_state.y + ny));
        }
        rows.push(LocalizerRow {
            name: "SR-UKF",
            final_pos_error: position_error(sr_ukf.state_2d(), final_truth),
            avg_step_time_us: t0.elapsed().as_secs_f64() * 1_000_000.0 / STEPS as f64,
        });
    }

    println!("| Localizer | Final Position Error (m) | Avg Step Time (us) |");
    println!("|-----------|-------------------------:|-------------------:|");
    for row in rows {
        println!(
            "| {} | {:.4} | {:.2} |",
            row.name, row.final_pos_error, row.avg_step_time_us
        );
    }

    Ok(())
}
