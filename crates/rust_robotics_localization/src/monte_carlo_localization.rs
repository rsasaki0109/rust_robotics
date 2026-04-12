//! Adaptive Monte Carlo Localization (MCL) with KLD-sampling.
//!
//! This module implements a particle filter variant where the particle count
//! is adapted online using a KLD bound.

use nalgebra::{Matrix4, Vector2, Vector4};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::collections::HashSet;
use std::f64::consts::PI;

use rust_robotics_core::{RoboticsError, RoboticsResult, State2D, StateEstimator};

/// State representation for MCL (x, y, yaw, velocity).
pub type MCLState = Vector4<f64>;

/// Control input (velocity, yaw rate).
pub type MCLControl = Vector2<f64>;

/// Landmark observation (distance, landmark_x, landmark_y).
pub type MCLMeasurement = Vec<(f64, f64, f64)>;

const X_BIN_SIZE: f64 = 0.5;
const Y_BIN_SIZE: f64 = 0.5;
const YAW_BIN_SIZE: f64 = 15.0_f64.to_radians();

/// Single particle.
#[derive(Debug, Clone)]
pub struct Particle {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub w: f64,
}

impl Particle {
    fn new(x: f64, y: f64, yaw: f64, v: f64, n_particles: usize) -> Self {
        Self {
            x,
            y,
            yaw,
            v,
            w: 1.0 / n_particles as f64,
        }
    }
}

/// Configuration for adaptive MCL.
#[derive(Debug, Clone)]
pub struct MonteCarloLocalizationConfig {
    /// Minimum number of particles.
    pub min_particles: usize,
    /// Maximum number of particles.
    pub max_particles: usize,
    /// KLD error bound (epsilon).
    pub kld_epsilon: f64,
    /// Standard normal quantile (z) used in KLD bound.
    pub kld_z: f64,
    /// Range measurement noise (sigma).
    pub range_noise: f64,
    /// Velocity process noise (sigma).
    pub velocity_noise: f64,
    /// Yaw-rate process noise (sigma).
    pub yaw_rate_noise: f64,
    /// Time step.
    pub dt: f64,
}

impl Default for MonteCarloLocalizationConfig {
    fn default() -> Self {
        Self {
            min_particles: 100,
            max_particles: 5000,
            kld_epsilon: 0.05,
            kld_z: 2.326,
            range_noise: 0.2,
            velocity_noise: 2.0,
            yaw_rate_noise: 40.0_f64.to_radians(),
            dt: 0.1,
        }
    }
}

impl MonteCarloLocalizationConfig {
    /// Validate configuration.
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.min_particles == 0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL min_particles must be greater than zero".to_string(),
            ));
        }
        if self.max_particles < self.min_particles {
            return Err(RoboticsError::InvalidParameter(
                "MCL max_particles must be greater than or equal to min_particles".to_string(),
            ));
        }
        if !self.kld_epsilon.is_finite() || self.kld_epsilon <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL kld_epsilon must be positive and finite".to_string(),
            ));
        }
        if !self.kld_z.is_finite() || self.kld_z <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL kld_z must be positive and finite".to_string(),
            ));
        }
        if !self.range_noise.is_finite() || self.range_noise <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL range_noise must be positive and finite".to_string(),
            ));
        }
        if !self.velocity_noise.is_finite() || self.velocity_noise < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL velocity_noise must be non-negative and finite".to_string(),
            ));
        }
        if !self.yaw_rate_noise.is_finite() || self.yaw_rate_noise < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL yaw_rate_noise must be non-negative and finite".to_string(),
            ));
        }
        if !self.dt.is_finite() || self.dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "MCL dt must be positive and finite".to_string(),
            ));
        }

        Ok(())
    }
}

/// Adaptive Monte Carlo localizer.
pub struct MonteCarloLocalizer {
    particles: Vec<Particle>,
    config: MonteCarloLocalizationConfig,
    state_estimate: MCLState,
    covariance_dyn: nalgebra::DMatrix<f64>,
}

impl MonteCarloLocalizer {
    /// Create a new localizer; panics on invalid config.
    pub fn new(config: MonteCarloLocalizationConfig) -> Self {
        Self::try_new(config).expect(
            "invalid MCL configuration: particle bounds, KLD params, noises, and dt must be valid",
        )
    }

    /// Create a new validated localizer.
    pub fn try_new(config: MonteCarloLocalizationConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let n = config.min_particles;
        let particles = (0..n)
            .map(|_| Particle::new(0.0, 0.0, 0.0, 0.0, n))
            .collect();
        let mut mcl = Self {
            particles,
            config,
            state_estimate: MCLState::zeros(),
            covariance_dyn: nalgebra::DMatrix::zeros(4, 4),
        };
        mcl.refresh_cache();
        Ok(mcl)
    }

    /// Create with initial state; panics on invalid inputs.
    pub fn with_initial_state(
        initial_state: MCLState,
        config: MonteCarloLocalizationConfig,
    ) -> Self {
        Self::try_with_initial_state(initial_state, config)
            .expect("invalid MCL initialization: state and configuration must be valid")
    }

    /// Create with validated initial state.
    pub fn try_with_initial_state(
        initial_state: MCLState,
        config: MonteCarloLocalizationConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        if initial_state.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "MCL initial state must contain only finite values".to_string(),
            ));
        }

        let mut rng = rand::thread_rng();
        let n = config.min_particles;
        let mut particles = Vec::with_capacity(n);
        for _ in 0..n {
            let x = initial_state[0] + rng.gen_range(-1.0..1.0);
            let y = initial_state[1] + rng.gen_range(-1.0..1.0);
            let yaw = initial_state[2] + rng.gen_range(-0.25..0.25);
            let v = initial_state[3] + rng.gen_range(-0.5..0.5);
            particles.push(Particle::new(x, y, yaw, v, n));
        }

        let mut mcl = Self {
            particles,
            config,
            state_estimate: MCLState::zeros(),
            covariance_dyn: nalgebra::DMatrix::zeros(4, 4),
        };
        mcl.refresh_cache();
        Ok(mcl)
    }

    /// Predict particles with the same motion model as `particle_filter.rs`.
    pub fn try_predict_with_control(&mut self, control: &MCLControl) -> RoboticsResult<()> {
        if control.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "MCL control input must contain only finite values".to_string(),
            ));
        }

        let mut rng = rand::thread_rng();
        let normal_v = if self.config.velocity_noise > 0.0 {
            Some(Normal::new(0.0, self.config.velocity_noise).map_err(|_| {
                RoboticsError::InvalidParameter(
                    "MCL velocity_noise must be a valid standard deviation".to_string(),
                )
            })?)
        } else {
            None
        };
        let normal_yaw = if self.config.yaw_rate_noise > 0.0 {
            Some(Normal::new(0.0, self.config.yaw_rate_noise).map_err(|_| {
                RoboticsError::InvalidParameter(
                    "MCL yaw_rate_noise must be a valid standard deviation".to_string(),
                )
            })?)
        } else {
            None
        };

        for particle in &mut self.particles {
            let v_noise = normal_v
                .as_ref()
                .map(|normal| normal.sample(&mut rng))
                .unwrap_or(0.0);
            let yaw_noise = normal_yaw
                .as_ref()
                .map(|normal| normal.sample(&mut rng))
                .unwrap_or(0.0);

            let v_noisy = control[0] + v_noise;
            let yaw_rate_noisy = control[1] + yaw_noise;

            particle.x += v_noisy * particle.yaw.cos() * self.config.dt;
            particle.y += v_noisy * particle.yaw.sin() * self.config.dt;
            particle.yaw += yaw_rate_noisy * self.config.dt;
            particle.v = v_noisy;
        }

        self.refresh_cache();
        Ok(())
    }

    /// Update particle weights with range observations.
    pub fn try_update_with_observations(
        &mut self,
        observations: &MCLMeasurement,
    ) -> RoboticsResult<()> {
        if observations
            .iter()
            .any(|(d, x, y)| !d.is_finite() || !x.is_finite() || !y.is_finite() || *d < 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "MCL observations must have finite, non-negative distances".to_string(),
            ));
        }

        for particle in &mut self.particles {
            let mut w = 1.0;
            for &(d_obs, lx, ly) in observations {
                let dx = particle.x - lx;
                let dy = particle.y - ly;
                let d_pred = (dx * dx + dy * dy).sqrt();
                let diff = d_obs - d_pred;
                w *= Self::gauss_likelihood(diff, self.config.range_noise);
            }
            particle.w = w;
        }

        self.normalize_weights();
        self.refresh_cache();
        Ok(())
    }

    /// Full estimation step.
    pub fn try_step(
        &mut self,
        control: &MCLControl,
        observations: &MCLMeasurement,
    ) -> RoboticsResult<MCLState> {
        self.try_predict_with_control(control)?;
        self.try_update_with_observations(observations)?;
        self.resample_adaptive();
        Ok(self.estimate())
    }

    /// Get estimated state.
    pub fn estimate(&self) -> MCLState {
        self.state_estimate
    }

    /// Get estimated state as common type.
    pub fn state_2d(&self) -> State2D {
        State2D::new(
            self.state_estimate[0],
            self.state_estimate[1],
            self.state_estimate[2],
            self.state_estimate[3],
        )
    }

    /// Current number of particles.
    pub fn particle_count(&self) -> usize {
        self.particles.len()
    }

    fn resample_adaptive(&mut self) {
        let n_current = self.particles.len();
        if n_current == 0 {
            return;
        }

        let mut cumulative_weights = Vec::with_capacity(n_current);
        let mut cum_sum = 0.0;
        for particle in &self.particles {
            cum_sum += particle.w;
            cumulative_weights.push(cum_sum);
        }
        if let Some(last) = cumulative_weights.last_mut() {
            *last = 1.0;
        }

        let mut rng = rand::thread_rng();
        let mut bins: HashSet<(i32, i32, i32)> = HashSet::new();
        let mut new_particles = Vec::with_capacity(self.config.max_particles);
        let mut required = self.config.min_particles;

        while new_particles.len() < self.config.max_particles {
            let idx = Self::sample_index(&cumulative_weights, rng.gen::<f64>());
            let sampled = &self.particles[idx];

            bins.insert(Self::quantize_particle(sampled));
            let k = bins.len();
            required = required.max(self.kld_required_particles(k));

            new_particles.push(sampled.clone());
            if new_particles.len() >= self.config.min_particles && new_particles.len() >= required {
                break;
            }
        }

        let n_new = new_particles.len();
        let uniform_weight = 1.0 / n_new as f64;
        for particle in &mut new_particles {
            particle.w = uniform_weight;
        }

        self.particles = new_particles;
        self.refresh_cache();
    }

    fn kld_required_particles(&self, k_bins: usize) -> usize {
        if k_bins <= 1 {
            return self.config.min_particles;
        }

        let k_minus_one = (k_bins - 1) as f64;
        let term = 1.0 - 2.0 / (9.0 * k_minus_one)
            + self.config.kld_z * (2.0 / (9.0 * k_minus_one)).sqrt();
        let n = (k_minus_one / (2.0 * self.config.kld_epsilon)) * term.powi(3);

        (n.ceil() as usize).clamp(self.config.min_particles, self.config.max_particles)
    }

    fn quantize_particle(particle: &Particle) -> (i32, i32, i32) {
        let x_bin = (particle.x / X_BIN_SIZE).floor() as i32;
        let y_bin = (particle.y / Y_BIN_SIZE).floor() as i32;
        let yaw_bin = (particle.yaw / YAW_BIN_SIZE).floor() as i32;
        (x_bin, y_bin, yaw_bin)
    }

    fn sample_index(cumulative_weights: &[f64], r: f64) -> usize {
        cumulative_weights
            .iter()
            .position(|&w| r <= w)
            .unwrap_or(cumulative_weights.len() - 1)
    }

    fn normalize_weights(&mut self) {
        let sum_w: f64 = self.particles.iter().map(|p| p.w).sum();
        if sum_w > 0.0 {
            for particle in &mut self.particles {
                particle.w /= sum_w;
            }
        } else {
            let uniform_weight = 1.0 / self.particles.len() as f64;
            for particle in &mut self.particles {
                particle.w = uniform_weight;
            }
        }
    }

    fn gauss_likelihood(x: f64, sigma: f64) -> f64 {
        let coeff = 1.0 / (2.0 * PI * sigma.powi(2)).sqrt();
        coeff * (-x.powi(2) / (2.0 * sigma.powi(2))).exp()
    }

    fn compute_estimate(&self) -> MCLState {
        let mut x_est = 0.0;
        let mut y_est = 0.0;
        let mut yaw_est = 0.0;
        let mut v_est = 0.0;

        for particle in &self.particles {
            x_est += particle.w * particle.x;
            y_est += particle.w * particle.y;
            yaw_est += particle.w * particle.yaw;
            v_est += particle.w * particle.v;
        }

        Vector4::new(x_est, y_est, yaw_est, v_est)
    }

    fn compute_covariance(&self, x_est: &MCLState) -> Matrix4<f64> {
        let mut cov = Matrix4::zeros();
        for particle in &self.particles {
            let dx = Vector4::new(
                particle.x - x_est[0],
                particle.y - x_est[1],
                particle.yaw - x_est[2],
                particle.v - x_est[3],
            );
            cov += particle.w * dx * dx.transpose();
        }
        cov
    }

    fn refresh_cache(&mut self) {
        self.state_estimate = self.compute_estimate();
        let covariance = self.compute_covariance(&self.state_estimate);
        self.covariance_dyn = nalgebra::DMatrix::from_fn(4, 4, |i, j| covariance[(i, j)]);
    }
}

impl StateEstimator for MonteCarloLocalizer {
    type State = MCLState;
    type Measurement = MCLMeasurement;
    type Control = MCLControl;

    fn predict(&mut self, control: &Self::Control, _dt: f64) {
        let _ = self.try_predict_with_control(control);
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        let _ = self.try_update_with_observations(measurement);
        self.resample_adaptive();
    }

    fn get_state(&self) -> &Self::State {
        &self.state_estimate
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        Some(&self.covariance_dyn)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_observations(state: &MCLState, landmarks: &[(f64, f64)]) -> MCLMeasurement {
        landmarks
            .iter()
            .map(|&(lx, ly)| {
                let dx = state[0] - lx;
                let dy = state[1] - ly;
                ((dx * dx + dy * dy).sqrt(), lx, ly)
            })
            .collect()
    }

    #[test]
    fn test_mcl_converges() {
        let config = MonteCarloLocalizationConfig {
            min_particles: 250,
            max_particles: 1200,
            range_noise: 0.25,
            velocity_noise: 0.05,
            yaw_rate_noise: 0.02,
            dt: 0.1,
            ..Default::default()
        };
        let mut mcl = MonteCarloLocalizer::with_initial_state(MCLState::zeros(), config);
        let landmarks = vec![(0.0, 0.0), (10.0, 0.0), (5.0, 8.0)];
        let control = MCLControl::new(1.0, 0.03);
        let mut truth = MCLState::zeros();

        for _ in 0..60 {
            truth[0] += control[0] * truth[2].cos() * 0.1;
            truth[1] += control[0] * truth[2].sin() * 0.1;
            truth[2] += control[1] * 0.1;
            truth[3] = control[0];
            let obs = synthetic_observations(&truth, &landmarks);
            let _ = mcl.try_step(&control, &obs).unwrap();
        }

        let est = mcl.estimate();
        let pos_err = ((est[0] - truth[0]).powi(2) + (est[1] - truth[1]).powi(2)).sqrt();
        assert!(pos_err < 1.0, "position error too high: {pos_err}");
    }

    #[test]
    fn test_mcl_particle_count_adapts() {
        let config = MonteCarloLocalizationConfig {
            min_particles: 100,
            max_particles: 1500,
            ..Default::default()
        };
        let mut mcl = MonteCarloLocalizer::new(config);

        mcl.particles.clear();
        let multimodal_n = 800;
        for i in 0..multimodal_n {
            let cluster = i % 4;
            let base_x = cluster as f64 * 3.0;
            let base_y = cluster as f64 * 2.0;
            let mut p = Particle::new(base_x + (i as f64) * 0.002, base_y, 0.0, 0.0, multimodal_n);
            p.w = 1.0 / multimodal_n as f64;
            mcl.particles.push(p);
        }
        mcl.resample_adaptive();
        let expanded_count = mcl.particle_count();
        assert!(expanded_count > mcl.config.min_particles);

        let concentrated_n = 600;
        mcl.particles.clear();
        for _ in 0..concentrated_n {
            let mut p = Particle::new(1.0, 1.0, 0.1, 0.0, concentrated_n);
            p.w = 1.0 / concentrated_n as f64;
            mcl.particles.push(p);
        }
        mcl.resample_adaptive();
        let reduced_count = mcl.particle_count();
        assert!(reduced_count <= expanded_count);
    }

    #[test]
    fn test_mcl_particle_count_stays_within_bounds() {
        let config = MonteCarloLocalizationConfig {
            min_particles: 120,
            max_particles: 600,
            velocity_noise: 0.5,
            yaw_rate_noise: 0.2,
            ..Default::default()
        };
        let mut mcl = MonteCarloLocalizer::new(config);
        let landmarks = vec![(0.0, 0.0), (15.0, 0.0), (8.0, 12.0)];
        let control = MCLControl::new(0.8, 0.05);
        let mut truth = MCLState::zeros();

        for _ in 0..40 {
            truth[0] += control[0] * truth[2].cos() * 0.1;
            truth[1] += control[0] * truth[2].sin() * 0.1;
            truth[2] += control[1] * 0.1;
            let obs = synthetic_observations(&truth, &landmarks);
            let _ = mcl.try_step(&control, &obs).unwrap();
            assert!(mcl.particle_count() >= mcl.config.min_particles);
            assert!(mcl.particle_count() <= mcl.config.max_particles);
        }
    }

    #[test]
    fn test_mcl_config_defaults() {
        let config = MonteCarloLocalizationConfig::default();
        assert_eq!(config.min_particles, 100);
        assert_eq!(config.max_particles, 5000);
        assert!((config.kld_epsilon - 0.05).abs() < f64::EPSILON);
        assert!((config.kld_z - 2.326).abs() < f64::EPSILON);
    }
}
