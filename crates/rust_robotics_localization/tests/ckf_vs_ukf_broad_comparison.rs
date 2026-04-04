//! CKF vs UKF broad comparison across multiple noise levels and scenarios.
//!
//! Tests the hypothesis that CKF achieves similar accuracy to UKF across
//! a wide range of conditions while being faster and requiring no tuning
//! parameters (alpha, beta, kappa).
//!
//! Scenarios:
//! 1. Low noise (benign conditions)
//! 2. High process noise
//! 3. High observation noise
//! 4. High nonlinearity (tight circle)
//! 5. Fast motion (high velocity)
//! 6. Long horizon (500 steps)
//! 7. Asymmetric noise (x >> y)
//! 8. Near-linear motion (straight line)
//! 9. Mixed: high process + low observation
//! 10. Mixed: low process + high observation

use std::time::Instant;

use nalgebra::{Matrix2, Matrix4, Vector2, Vector4};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

use rust_robotics_localization::{
    cubature_kalman_filter::{CKFConfig, CKFLocalizer},
    unscented_kalman_filter::{UKFConfig, UKFLocalizer, UKFParams},
};

const DT: f64 = 0.1;

struct ScenarioConfig {
    name: &'static str,
    num_steps: usize,
    velocity: f64,
    yaw_rate: f64,
    process_noise_v: f64,
    process_noise_yaw_deg: f64,
    obs_noise_x: f64,
    obs_noise_y: f64,
}

struct SimData {
    ground_truth: Vec<Vector4<f64>>,
    noisy_controls: Vec<Vector2<f64>>,
    noisy_observations: Vec<Vector2<f64>>,
}

fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>, dt: f64) -> Vector4<f64> {
    let yaw = x[2];
    Vector4::new(
        x[0] + dt * u[0] * yaw.cos(),
        x[1] + dt * u[0] * yaw.sin(),
        x[2] + dt * u[1],
        u[0],
    )
}

fn generate_sim_data(config: &ScenarioConfig, seed: u64) -> SimData {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let noise_yaw_rad = config.process_noise_yaw_deg.to_radians();

    let noise_v = Normal::new(0.0, config.process_noise_v).unwrap();
    let noise_yaw = Normal::new(0.0, noise_yaw_rad).unwrap();
    let noise_obs_x = Normal::new(0.0, config.obs_noise_x).unwrap();
    let noise_obs_y = Normal::new(0.0, config.obs_noise_y).unwrap();

    let u_true = Vector2::new(config.velocity, config.yaw_rate);
    let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);

    let mut ground_truth = Vec::with_capacity(config.num_steps);
    let mut noisy_controls = Vec::with_capacity(config.num_steps);
    let mut noisy_observations = Vec::with_capacity(config.num_steps);

    for _ in 0..config.num_steps {
        x_true = motion_model(&x_true, &u_true, DT);
        let u_noisy = Vector2::new(
            u_true[0] + noise_v.sample(&mut rng),
            u_true[1] + noise_yaw.sample(&mut rng),
        );
        let z = Vector2::new(
            x_true[0] + noise_obs_x.sample(&mut rng),
            x_true[1] + noise_obs_y.sample(&mut rng),
        );

        ground_truth.push(x_true);
        noisy_controls.push(u_noisy);
        noisy_observations.push(z);
    }

    SimData {
        ground_truth,
        noisy_controls,
        noisy_observations,
    }
}

struct FilterResult {
    #[allow(dead_code)]
    name: &'static str,
    rmse: f64,
    elapsed_us: u128,
    final_error: f64,
}

fn compute_rmse(positions: &[(f64, f64)], ground_truth: &[Vector4<f64>]) -> f64 {
    let n = positions.len().min(ground_truth.len());
    if n == 0 {
        return f64::NAN;
    }
    let sum_sq: f64 = positions
        .iter()
        .zip(ground_truth.iter())
        .map(|((ex, ey), gt)| (ex - gt[0]).powi(2) + (ey - gt[1]).powi(2))
        .sum();
    (sum_sq / n as f64).sqrt()
}

fn final_error(positions: &[(f64, f64)], ground_truth: &[Vector4<f64>]) -> f64 {
    if let (Some(&(ex, ey)), Some(gt)) = (positions.last(), ground_truth.last()) {
        ((ex - gt[0]).powi(2) + (ey - gt[1]).powi(2)).sqrt()
    } else {
        f64::NAN
    }
}

fn run_ckf(data: &SimData, config: &ScenarioConfig) -> FilterResult {
    let q = Matrix4::from_diagonal(&Vector4::new(
        0.1_f64.powi(2),
        0.1_f64.powi(2),
        1.0_f64.to_radians().powi(2),
        1.0_f64.powi(2),
    ));
    let mut r = Matrix2::<f64>::identity();
    r[(0, 0)] = config.obs_noise_x.powi(2);
    r[(1, 1)] = config.obs_noise_y.powi(2);

    let ckf_config = CKFConfig { q, r };
    let mut ckf = CKFLocalizer::new(ckf_config);

    let mut positions = Vec::with_capacity(config.num_steps);
    let start = Instant::now();

    for i in 0..config.num_steps {
        ckf.step(&data.noisy_controls[i], &data.noisy_observations[i], DT);
        let s = ckf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "CKF",
        rmse: compute_rmse(&positions, &data.ground_truth),
        elapsed_us,
        final_error: final_error(&positions, &data.ground_truth),
    }
}

fn run_ukf(data: &SimData, config: &ScenarioConfig) -> FilterResult {
    let ukf_config = UKFConfig {
        params: UKFParams::default(),
        process_noise: Vector4::new(
            0.1_f64.powi(2),
            0.1_f64.powi(2),
            1.0_f64.to_radians().powi(2),
            1.0_f64.powi(2),
        ),
        observation_noise: Vector2::new(
            config.obs_noise_x.powi(2),
            config.obs_noise_y.powi(2),
        ),
        dt: DT,
    };
    let mut ukf = UKFLocalizer::new(ukf_config);

    let mut positions = Vec::with_capacity(config.num_steps);
    let start = Instant::now();

    for i in 0..config.num_steps {
        ukf.step(&data.noisy_controls[i], &data.noisy_observations[i]);
        let s = ukf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "UKF",
        rmse: compute_rmse(&positions, &data.ground_truth),
        elapsed_us,
        final_error: final_error(&positions, &data.ground_truth),
    }
}

fn scenarios() -> Vec<ScenarioConfig> {
    vec![
        ScenarioConfig {
            name: "1. Low noise",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 0.1,
            process_noise_yaw_deg: 1.0,
            obs_noise_x: 0.2,
            obs_noise_y: 0.2,
        },
        ScenarioConfig {
            name: "2. High process noise",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 1.0,
            process_noise_yaw_deg: 15.0,
            obs_noise_x: 0.5,
            obs_noise_y: 0.5,
        },
        ScenarioConfig {
            name: "3. High obs noise",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 0.3,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 2.0,
            obs_noise_y: 2.0,
        },
        ScenarioConfig {
            name: "4. Tight circle",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.5,
            process_noise_v: 0.3,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 0.5,
            obs_noise_y: 0.5,
        },
        ScenarioConfig {
            name: "5. Fast motion",
            num_steps: 100,
            velocity: 5.0,
            yaw_rate: 0.1,
            process_noise_v: 0.5,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 0.5,
            obs_noise_y: 0.5,
        },
        ScenarioConfig {
            name: "6. Long horizon",
            num_steps: 500,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 0.3,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 0.5,
            obs_noise_y: 0.5,
        },
        ScenarioConfig {
            name: "7. Asymmetric noise",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 0.3,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 2.0,
            obs_noise_y: 0.2,
        },
        ScenarioConfig {
            name: "8. Near-linear",
            num_steps: 100,
            velocity: 2.0,
            yaw_rate: 0.01,
            process_noise_v: 0.3,
            process_noise_yaw_deg: 5.0,
            obs_noise_x: 0.5,
            obs_noise_y: 0.5,
        },
        ScenarioConfig {
            name: "9. High proc + low obs",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 1.0,
            process_noise_yaw_deg: 15.0,
            obs_noise_x: 0.1,
            obs_noise_y: 0.1,
        },
        ScenarioConfig {
            name: "10. Low proc + high obs",
            num_steps: 100,
            velocity: 1.0,
            yaw_rate: 0.1,
            process_noise_v: 0.05,
            process_noise_yaw_deg: 0.5,
            obs_noise_x: 3.0,
            obs_noise_y: 3.0,
        },
    ]
}

#[test]
fn ckf_vs_ukf_broad_comparison() {
    let all_scenarios = scenarios();

    println!();
    println!(
        "{:<28} | {:>10} {:>10} {:>10} | {:>10} {:>10} {:>10} | {:>8} {:>8}",
        "Scenario", "CKF RMSE", "CKF Time", "CKF Final",
        "UKF RMSE", "UKF Time", "UKF Final",
        "RMSE %", "Time %"
    );
    println!("{}", "-".repeat(130));

    let mut ckf_wins_rmse = 0;
    let mut ukf_wins_rmse = 0;
    let mut ckf_total_time = 0u128;
    let mut ukf_total_time = 0u128;

    for scenario in &all_scenarios {
        let data = generate_sim_data(scenario, 42);
        let ckf_result = run_ckf(&data, scenario);
        let ukf_result = run_ukf(&data, scenario);

        let rmse_ratio = (ckf_result.rmse / ukf_result.rmse - 1.0) * 100.0;
        let time_ratio = (ckf_result.elapsed_us as f64 / ukf_result.elapsed_us as f64 - 1.0) * 100.0;

        println!(
            "{:<28} | {:>10.4} {:>9.1}us {:>10.4} | {:>10.4} {:>9.1}us {:>10.4} | {:>+7.1}% {:>+7.1}%",
            scenario.name,
            ckf_result.rmse,
            ckf_result.elapsed_us as f64,
            ckf_result.final_error,
            ukf_result.rmse,
            ukf_result.elapsed_us as f64,
            ukf_result.final_error,
            rmse_ratio,
            time_ratio,
        );

        if ckf_result.rmse <= ukf_result.rmse {
            ckf_wins_rmse += 1;
        } else {
            ukf_wins_rmse += 1;
        }
        ckf_total_time += ckf_result.elapsed_us;
        ukf_total_time += ukf_result.elapsed_us;

        // Both filters should produce reasonable results
        assert!(
            ckf_result.rmse.is_finite() && ckf_result.rmse < 10.0,
            "CKF diverged on {}: RMSE = {:.4}",
            scenario.name,
            ckf_result.rmse
        );
        assert!(
            ukf_result.rmse.is_finite() && ukf_result.rmse < 10.0,
            "UKF diverged on {}: RMSE = {:.4}",
            scenario.name,
            ukf_result.rmse
        );

        // CKF should be within 20% of UKF accuracy on all scenarios
        let accuracy_ratio = ckf_result.rmse / ukf_result.rmse;
        assert!(
            accuracy_ratio < 1.2,
            "CKF RMSE ({:.4}) more than 20% worse than UKF ({:.4}) on {}",
            ckf_result.rmse,
            ukf_result.rmse,
            scenario.name
        );
    }

    println!();
    println!("Summary:");
    println!(
        "  CKF wins RMSE: {}/{}, UKF wins RMSE: {}/{}",
        ckf_wins_rmse,
        all_scenarios.len(),
        ukf_wins_rmse,
        all_scenarios.len()
    );
    println!(
        "  CKF total time: {:.1}ms, UKF total time: {:.1}ms (CKF is {:.0}% of UKF)",
        ckf_total_time as f64 / 1000.0,
        ukf_total_time as f64 / 1000.0,
        ckf_total_time as f64 / ukf_total_time as f64 * 100.0,
    );
    println!(
        "  CKF tuning params: 0, UKF tuning params: 3 (alpha, beta, kappa)"
    );
    println!();
}
