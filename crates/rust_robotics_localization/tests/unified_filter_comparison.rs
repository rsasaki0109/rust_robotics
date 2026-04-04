//! Unified benchmark comparing 6 localization filters on the same scenario.
//!
//! Simulates a vehicle driving in a circle, adds noise to controls and
//! observations, then runs each filter for 100 steps. Reports RMSE, total
//! computation time, and final position error in a comparison table.

use std::time::Instant;

use nalgebra::{Matrix2, Matrix4, Vector2, Vector4};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

use rust_robotics_localization::{
    cubature_kalman_filter::{CKFConfig, CKFLocalizer},
    ekf::{EKFConfig, EKFLocalizer},
    ensemble_kalman_filter::{EnKFConfig, EnKFLocalizer},
    histogram_filter::HistogramFilter,
    particle_filter::{ParticleFilterConfig, ParticleFilterLocalizer},
    unscented_kalman_filter::{UKFConfig, UKFLocalizer, UKFParams},
};

// ---------------------------------------------------------------------------
// Simulation parameters
// ---------------------------------------------------------------------------

const DT: f64 = 0.1;
const NUM_STEPS: usize = 100;

/// Velocity for circular motion [m/s]
const VELOCITY: f64 = 1.0;
/// Yaw rate for circular motion [rad/s]
const YAW_RATE: f64 = 0.1;

// Noise standard deviations
const INPUT_NOISE_V: f64 = 0.3;
const INPUT_NOISE_YAW: f64 = 5.0_f64; // degrees, converted below
const OBS_NOISE_X: f64 = 0.5;
const OBS_NOISE_Y: f64 = 0.5;

// Landmark positions (used by particle filter and histogram filter)
const LANDMARKS: [(f64, f64); 4] = [(10.0, 0.0), (0.0, 15.0), (-5.0, 20.0), (10.0, 10.0)];

// ---------------------------------------------------------------------------
// Ground truth + noisy data generation (deterministic seed)
// ---------------------------------------------------------------------------

struct SimData {
    ground_truth: Vec<Vector4<f64>>,                  // [x, y, yaw, v]
    noisy_controls: Vec<Vector2<f64>>,                // [v, yaw_rate] with noise
    noisy_observations: Vec<Vector2<f64>>,            // [x, y] with noise
    landmark_observations: Vec<Vec<(f64, f64, f64)>>, // (distance, lm_x, lm_y)
}

/// Motion model shared across filters: x_{k+1} = f(x_k, u_k)
fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>, dt: f64) -> Vector4<f64> {
    let yaw = x[2];
    Vector4::new(
        x[0] + dt * u[0] * yaw.cos(),
        x[1] + dt * u[0] * yaw.sin(),
        x[2] + dt * u[1],
        u[0],
    )
}

fn generate_sim_data() -> SimData {
    let mut rng = rand::rngs::StdRng::seed_from_u64(42);
    let input_noise_yaw_rad = INPUT_NOISE_YAW.to_radians();

    let noise_v = Normal::new(0.0, INPUT_NOISE_V).unwrap();
    let noise_yaw = Normal::new(0.0, input_noise_yaw_rad).unwrap();
    let noise_obs_x = Normal::new(0.0, OBS_NOISE_X).unwrap();
    let noise_obs_y = Normal::new(0.0, OBS_NOISE_Y).unwrap();
    let noise_range = Normal::new(0.0, 0.5).unwrap();

    let u_true = Vector2::new(VELOCITY, YAW_RATE);
    let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);

    let mut ground_truth = Vec::with_capacity(NUM_STEPS);
    let mut noisy_controls = Vec::with_capacity(NUM_STEPS);
    let mut noisy_observations = Vec::with_capacity(NUM_STEPS);
    let mut landmark_observations = Vec::with_capacity(NUM_STEPS);

    for _ in 0..NUM_STEPS {
        // Advance ground truth
        x_true = motion_model(&x_true, &u_true, DT);

        // Noisy control
        let u_noisy = Vector2::new(
            u_true[0] + noise_v.sample(&mut rng),
            u_true[1] + noise_yaw.sample(&mut rng),
        );

        // Noisy position observation
        let z = Vector2::new(
            x_true[0] + noise_obs_x.sample(&mut rng),
            x_true[1] + noise_obs_y.sample(&mut rng),
        );

        // Landmark range observations
        let mut lm_obs = Vec::new();
        for &(lx, ly) in &LANDMARKS {
            let d_true = ((x_true[0] - lx).powi(2) + (x_true[1] - ly).powi(2)).sqrt();
            let d_noisy = (d_true + noise_range.sample(&mut rng)).max(0.0);
            lm_obs.push((d_noisy, lx, ly));
        }

        ground_truth.push(x_true);
        noisy_controls.push(u_noisy);
        noisy_observations.push(z);
        landmark_observations.push(lm_obs);
    }

    SimData {
        ground_truth,
        noisy_controls,
        noisy_observations,
        landmark_observations,
    }
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

struct FilterResult {
    name: &'static str,
    positions: Vec<(f64, f64)>,
    elapsed_us: u128,
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

fn final_position_error(positions: &[(f64, f64)], ground_truth: &[Vector4<f64>]) -> f64 {
    if let (Some(&(ex, ey)), Some(gt)) = (positions.last(), ground_truth.last()) {
        ((ex - gt[0]).powi(2) + (ey - gt[1]).powi(2)).sqrt()
    } else {
        f64::NAN
    }
}

// ---------------------------------------------------------------------------
// Run each filter
// ---------------------------------------------------------------------------

fn run_ekf(data: &SimData) -> FilterResult {
    let mut q = Matrix4::<f64>::identity();
    q[(0, 0)] = 0.1_f64.powi(2);
    q[(1, 1)] = 0.1_f64.powi(2);
    q[(2, 2)] = 1.0_f64.to_radians().powi(2);
    q[(3, 3)] = 1.0_f64.powi(2);

    let mut r = Matrix2::<f64>::identity();
    r[(0, 0)] = OBS_NOISE_X.powi(2);
    r[(1, 1)] = OBS_NOISE_Y.powi(2);

    let config = EKFConfig { q, r };
    let mut ekf = EKFLocalizer::new(config);

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        let _ = ekf.estimate(&data.noisy_observations[i], &data.noisy_controls[i], DT);
        let s = ekf.state_2d();
        positions.push((s.x, s.y));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "EKF",
        positions,
        elapsed_us,
    }
}

fn run_ukf(data: &SimData) -> FilterResult {
    let config = UKFConfig {
        params: UKFParams::default(),
        process_noise: Vector4::new(
            0.1_f64.powi(2),
            0.1_f64.powi(2),
            1.0_f64.to_radians().powi(2),
            1.0_f64.powi(2),
        ),
        observation_noise: Vector2::new(OBS_NOISE_X.powi(2), OBS_NOISE_Y.powi(2)),
        dt: DT,
    };
    let mut ukf = UKFLocalizer::new(config);

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        ukf.step(&data.noisy_controls[i], &data.noisy_observations[i]);
        let s = ukf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "UKF",
        positions,
        elapsed_us,
    }
}

fn run_ckf(data: &SimData) -> FilterResult {
    let q = Matrix4::from_diagonal(&Vector4::new(
        0.1_f64.powi(2),
        0.1_f64.powi(2),
        1.0_f64.to_radians().powi(2),
        1.0_f64.powi(2),
    ));

    let mut r = Matrix2::<f64>::identity();
    r[(0, 0)] = OBS_NOISE_X.powi(2);
    r[(1, 1)] = OBS_NOISE_Y.powi(2);

    let config = CKFConfig { q, r };
    let mut ckf = CKFLocalizer::new(config);

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        ckf.step(&data.noisy_controls[i], &data.noisy_observations[i], DT);
        let s = ckf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "CKF",
        positions,
        elapsed_us,
    }
}

fn run_enkf(data: &SimData) -> FilterResult {
    let config = EnKFConfig {
        num_particles: 50,
        process_noise_std: Vector2::new(INPUT_NOISE_V, INPUT_NOISE_YAW.to_radians()),
        measurement_noise_std: Vector2::new(OBS_NOISE_X, OBS_NOISE_Y),
    };
    let mut enkf = EnKFLocalizer::new(config);

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        enkf.step(&data.noisy_controls[i], &data.noisy_observations[i], DT);
        let s = enkf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "EnKF",
        positions,
        elapsed_us,
    }
}

fn run_particle_filter(data: &SimData) -> FilterResult {
    let config = ParticleFilterConfig {
        n_particles: 200,
        resample_threshold: 0.5,
        range_noise: 0.5,
        velocity_noise: INPUT_NOISE_V,
        yaw_rate_noise: INPUT_NOISE_YAW.to_radians(),
        dt: DT,
    };
    let mut pf = ParticleFilterLocalizer::new(config);

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        pf.step(&data.noisy_controls[i], &data.landmark_observations[i]);
        let s = pf.estimate();
        positions.push((s[0], s[1]));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "PF",
        positions,
        elapsed_us,
    }
}

fn run_histogram_filter(data: &SimData) -> FilterResult {
    let rfid: Vec<(f64, f64)> = LANDMARKS.to_vec();

    // Grid covers the motion area with some margin
    let mut hf = HistogramFilter::new_with_initial_pos(
        -5.0, -5.0, 15.0, 15.0, 0.5, // grid bounds and resolution
        0.0, 0.0, 1.0, // initial position and std
    );

    let mut positions = Vec::with_capacity(NUM_STEPS);
    let mut yaw = 0.0_f64;
    let start = Instant::now();

    for i in 0..NUM_STEPS {
        let u = data.noisy_controls[i];
        yaw += u[1] * DT;

        // Motion update
        hf.motion_update(u, yaw, DT);

        // Observation update with landmark distances
        hf.observation_update(&data.landmark_observations[i], &rfid);

        let (x, y) = hf.get_estimated_position();
        positions.push((x, y));
    }

    let elapsed_us = start.elapsed().as_micros();
    FilterResult {
        name: "Histogram",
        positions,
        elapsed_us,
    }
}

// ---------------------------------------------------------------------------
// Test
// ---------------------------------------------------------------------------

#[test]
fn unified_filter_comparison() {
    let data = generate_sim_data();

    let results = vec![
        run_ekf(&data),
        run_ukf(&data),
        run_ckf(&data),
        run_enkf(&data),
        run_particle_filter(&data),
        run_histogram_filter(&data),
    ];

    // Print comparison table
    println!();
    println!(
        "{:<12} | {:>10} | {:>10} | {:>18}",
        "Filter", "RMSE (m)", "Time (ms)", "Final Error (m)"
    );
    println!("{}", "-".repeat(58));

    for r in &results {
        let rmse = compute_rmse(&r.positions, &data.ground_truth);
        let final_err = final_position_error(&r.positions, &data.ground_truth);
        let time_ms = r.elapsed_us as f64 / 1000.0;

        println!(
            "{:<12} | {:>10.4} | {:>10.3} | {:>18.4}",
            r.name, rmse, time_ms, final_err
        );
    }
    println!();

    // Sanity checks: Kalman-family filters (EKF, UKF, CKF) should achieve
    // reasonable RMSE on this scenario (direct position observations).
    for r in &results[..3] {
        let rmse = compute_rmse(&r.positions, &data.ground_truth);
        assert!(rmse < 2.0, "{} RMSE too high: {:.4}", r.name, rmse);
    }

    // EnKF uses stochastic ensemble, allow slightly larger RMSE
    {
        let rmse = compute_rmse(&results[3].positions, &data.ground_truth);
        assert!(rmse < 3.0, "EnKF RMSE too high: {:.4}", rmse);
    }

    // PF and Histogram use landmark observations only, so they have a
    // fundamentally different observation model. We just check they don't
    // diverge to infinity.
    for r in &results[4..] {
        let rmse = compute_rmse(&r.positions, &data.ground_truth);
        assert!(rmse.is_finite(), "{} produced non-finite RMSE", r.name);
    }
}
