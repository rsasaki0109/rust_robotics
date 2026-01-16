//! Particle Filter (PF) localization
//!
//! Implements state estimation using Sequential Monte Carlo method
//! for robot localization with landmark observations.

use nalgebra::{Matrix4, Vector4, Vector2};
use rand::Rng;
use rand_distr::{Normal, Distribution};
use std::f64::consts::PI;

use crate::common::{StateEstimator, Point2D};

/// State representation for Particle Filter (x, y, yaw, velocity)
pub type PFState = Vector4<f64>;

/// Control input (velocity, yaw rate)
pub type PFControl = Vector2<f64>;

/// Landmark observation (distance, landmark_x, landmark_y)
pub type PFMeasurement = Vec<(f64, f64, f64)>;

/// Single particle representing a hypothesis
#[derive(Debug, Clone)]
pub struct Particle {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub w: f64, // weight
}

impl Particle {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, n_particles: usize) -> Self {
        Particle {
            x,
            y,
            yaw,
            v,
            w: 1.0 / n_particles as f64,
        }
    }

    pub fn to_state(&self) -> PFState {
        Vector4::new(self.x, self.y, self.yaw, self.v)
    }
}

/// Configuration for Particle Filter
#[derive(Debug, Clone)]
pub struct ParticleFilterConfig {
    /// Number of particles
    pub n_particles: usize,
    /// Effective particle threshold ratio for resampling (0.0-1.0)
    pub resample_threshold: f64,
    /// Range measurement noise (sigma)
    pub range_noise: f64,
    /// Velocity noise (sigma)
    pub velocity_noise: f64,
    /// Yaw rate noise (sigma)
    pub yaw_rate_noise: f64,
    /// Time step
    pub dt: f64,
}

impl Default for ParticleFilterConfig {
    fn default() -> Self {
        Self {
            n_particles: 100,
            resample_threshold: 0.5,
            range_noise: 0.2,
            velocity_noise: 2.0,
            yaw_rate_noise: 40.0_f64.to_radians(),
            dt: 0.1,
        }
    }
}

/// Particle Filter for robot localization
pub struct ParticleFilterLocalizer {
    particles: Vec<Particle>,
    config: ParticleFilterConfig,
    landmarks: Vec<Point2D>,
    last_control: PFControl,
}

impl ParticleFilterLocalizer {
    /// Create a new Particle Filter localizer
    pub fn new(config: ParticleFilterConfig) -> Self {
        let n = config.n_particles;
        let particles = (0..n)
            .map(|_| Particle::new(0.0, 0.0, 0.0, 0.0, n))
            .collect();

        ParticleFilterLocalizer {
            particles,
            config,
            landmarks: Vec::new(),
            last_control: PFControl::zeros(),
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(ParticleFilterConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: PFState, config: ParticleFilterConfig) -> Self {
        let mut rng = rand::thread_rng();
        let n = config.n_particles;
        let mut particles = Vec::with_capacity(n);

        for _ in 0..n {
            let x = initial_state[0] + rng.gen::<f64>() * 2.0 - 1.0;
            let y = initial_state[1] + rng.gen::<f64>() * 2.0 - 1.0;
            let yaw = initial_state[2] + rng.gen::<f64>() * 0.5 - 0.25;
            let v = initial_state[3] + rng.gen::<f64>() * 1.0 - 0.5;
            particles.push(Particle::new(x, y, yaw, v, n));
        }

        ParticleFilterLocalizer {
            particles,
            config,
            landmarks: Vec::new(),
            last_control: PFControl::zeros(),
        }
    }

    /// Set landmarks for observation
    pub fn set_landmarks(&mut self, landmarks: Vec<Point2D>) {
        self.landmarks = landmarks;
    }

    /// Get landmarks
    pub fn get_landmarks(&self) -> &[Point2D] {
        &self.landmarks
    }

    /// Get current particles
    pub fn get_particles(&self) -> &[Particle] {
        &self.particles
    }

    /// Prediction step: propagate particles with motion model
    pub fn predict_with_control(&mut self, control: &PFControl) {
        let mut rng = rand::thread_rng();
        let normal_v = Normal::new(0.0, self.config.velocity_noise.powi(2)).unwrap();
        let normal_yaw = Normal::new(0.0, self.config.yaw_rate_noise.powi(2)).unwrap();
        let dt = self.config.dt;

        for particle in &mut self.particles {
            let v_noise = normal_v.sample(&mut rng);
            let yaw_noise = normal_yaw.sample(&mut rng);

            let v_noisy = control[0] + v_noise;
            let yaw_rate_noisy = control[1] + yaw_noise;

            particle.x += v_noisy * particle.yaw.cos() * dt;
            particle.y += v_noisy * particle.yaw.sin() * dt;
            particle.yaw += yaw_rate_noisy * dt;
            particle.v = v_noisy;
        }

        self.last_control = *control;
    }

    /// Update step: update weights based on observations
    pub fn update_with_observations(&mut self, observations: &PFMeasurement) {
        for particle in &mut self.particles {
            let mut w = 1.0;

            for &(d_obs, landmark_x, landmark_y) in observations {
                let dx = particle.x - landmark_x;
                let dy = particle.y - landmark_y;
                let d_pred = (dx * dx + dy * dy).sqrt();

                let diff = d_obs - d_pred;
                w *= Self::gauss_likelihood(diff, self.config.range_noise);
            }

            particle.w = w;
        }

        self.normalize_weights();
    }

    /// Resample particles if effective particle count is low
    pub fn resample(&mut self) {
        let n_eff = self.calc_n_eff();
        let threshold = self.config.n_particles as f64 * self.config.resample_threshold;

        if n_eff < threshold {
            self.resample_particles();
        }
    }

    /// Compute weighted state estimate
    pub fn estimate(&self) -> PFState {
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

    /// Compute state covariance
    pub fn calc_covariance(&self) -> Matrix4<f64> {
        let x_est = self.estimate();
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

    /// Calculate effective particle count
    fn calc_n_eff(&self) -> f64 {
        let sum_w_squared: f64 = self.particles.iter().map(|p| p.w * p.w).sum();
        if sum_w_squared > 0.0 {
            1.0 / sum_w_squared
        } else {
            0.0
        }
    }

    /// Normalize particle weights
    fn normalize_weights(&mut self) {
        let sum_w: f64 = self.particles.iter().map(|p| p.w).sum();

        if sum_w > 0.0 {
            for particle in &mut self.particles {
                particle.w /= sum_w;
            }
        }
    }

    /// Systematic resampling
    fn resample_particles(&mut self) {
        let mut rng = rand::thread_rng();
        let n = self.config.n_particles;
        let mut new_particles = Vec::with_capacity(n);

        // Calculate cumulative weights
        let mut cumulative_weights = Vec::with_capacity(n);
        let mut cum_sum = 0.0;
        for particle in &self.particles {
            cum_sum += particle.w;
            cumulative_weights.push(cum_sum);
        }

        for _ in 0..n {
            let r = rng.gen::<f64>();

            // Find particle to resample
            let mut index = 0;
            for (i, &cum_w) in cumulative_weights.iter().enumerate() {
                if r <= cum_w {
                    index = i;
                    break;
                }
            }

            let mut new_particle = self.particles[index].clone();
            new_particle.w = 1.0 / n as f64;
            new_particles.push(new_particle);
        }

        self.particles = new_particles;
    }

    /// Gaussian likelihood function
    fn gauss_likelihood(x: f64, sigma: f64) -> f64 {
        let coeff = 1.0 / (2.0 * PI * sigma.powi(2)).sqrt();
        coeff * (-x.powi(2) / (2.0 * sigma.powi(2))).exp()
    }

    /// Full estimation step (predict + update + resample)
    pub fn step(&mut self, control: &PFControl, observations: &PFMeasurement) -> PFState {
        self.predict_with_control(control);
        self.update_with_observations(observations);
        self.resample();
        self.estimate()
    }
}

impl StateEstimator for ParticleFilterLocalizer {
    type State = PFState;
    type Measurement = PFMeasurement;
    type Control = PFControl;

    fn predict(&mut self, control: &Self::Control, _dt: f64) {
        self.predict_with_control(control);
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        self.update_with_observations(measurement);
        self.resample();
    }

    fn get_state(&self) -> &Self::State {
        // Note: We need to return a reference, but estimate() computes on the fly
        // This is a limitation of the trait design
        // For now, we'll use a workaround
        unimplemented!("Use estimate() method instead for ParticleFilter")
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        None // Use calc_covariance() for fixed-size covariance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_particle_filter_creation() {
        let pf = ParticleFilterLocalizer::with_defaults();
        assert_eq!(pf.particles.len(), 100);
    }

    #[test]
    fn test_particle_filter_with_initial_state() {
        let initial = PFState::new(1.0, 2.0, 0.5, 1.0);
        let config = ParticleFilterConfig::default();
        let pf = ParticleFilterLocalizer::with_initial_state(initial, config);

        let estimate = pf.estimate();
        // Should be roughly around initial state (with some spread)
        assert!((estimate[0] - 1.0).abs() < 2.0);
        assert!((estimate[1] - 2.0).abs() < 2.0);
    }

    #[test]
    fn test_particle_filter_predict() {
        let mut pf = ParticleFilterLocalizer::with_defaults();
        let control = PFControl::new(1.0, 0.0); // move forward

        let estimate_before = pf.estimate();
        pf.predict_with_control(&control);
        let estimate_after = pf.estimate();

        // Particles should have moved
        assert!(estimate_after[0] != estimate_before[0] || estimate_after[1] != estimate_before[1]);
    }

    #[test]
    fn test_particle_filter_update() {
        let initial = PFState::new(5.0, 5.0, 0.0, 0.0);
        let config = ParticleFilterConfig::default();
        let mut pf = ParticleFilterLocalizer::with_initial_state(initial, config);

        // Observation: distance 5.0 to landmark at (0, 5)
        let observations = vec![(5.0, 0.0, 5.0)];
        pf.update_with_observations(&observations);

        // Weights should be updated
        let total_weight: f64 = pf.particles.iter().map(|p| p.w).sum();
        assert!((total_weight - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_particle_filter_step() {
        let mut pf = ParticleFilterLocalizer::with_defaults();
        let control = PFControl::new(1.0, 0.1);
        let observations = vec![(10.0, 10.0, 0.0)];

        let estimate = pf.step(&control, &observations);

        // Should return a valid state
        assert!(estimate[0].is_finite());
        assert!(estimate[1].is_finite());
    }

    #[test]
    fn test_particle_filter_covariance() {
        let pf = ParticleFilterLocalizer::with_defaults();
        let cov = pf.calc_covariance();

        // Covariance should be positive semi-definite (diagonal > 0)
        assert!(cov[(0, 0)] >= 0.0);
        assert!(cov[(1, 1)] >= 0.0);
    }
}
