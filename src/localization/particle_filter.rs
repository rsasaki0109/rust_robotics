//! Particle Filter (PF) localization
//!
//! Implements state estimation using Sequential Monte Carlo method
//! for robot localization with landmark observations.

use nalgebra::{Matrix4, Vector2, Vector4};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;

use crate::common::{
    ControlInput, Obstacles, Point2D, RoboticsError, RoboticsResult, State2D, StateEstimator,
};

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

impl ParticleFilterConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.n_particles == 0 {
            return Err(RoboticsError::InvalidParameter(
                "particle filter requires at least one particle".to_string(),
            ));
        }
        if !self.resample_threshold.is_finite()
            || self.resample_threshold < 0.0
            || self.resample_threshold > 1.0
        {
            return Err(RoboticsError::InvalidParameter(
                "particle filter resample_threshold must be within [0.0, 1.0]".to_string(),
            ));
        }
        if !self.range_noise.is_finite() || self.range_noise <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "particle filter range_noise must be positive and finite".to_string(),
            ));
        }
        if !self.velocity_noise.is_finite() || self.velocity_noise < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "particle filter velocity_noise must be non-negative and finite".to_string(),
            ));
        }
        if !self.yaw_rate_noise.is_finite() || self.yaw_rate_noise < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "particle filter yaw_rate_noise must be non-negative and finite".to_string(),
            ));
        }
        if !self.dt.is_finite() || self.dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "particle filter dt must be positive and finite".to_string(),
            ));
        }

        Ok(())
    }
}

/// Particle Filter for robot localization
pub struct ParticleFilterLocalizer {
    particles: Vec<Particle>,
    config: ParticleFilterConfig,
    landmarks: Vec<Point2D>,
    last_control: PFControl,
    state_estimate: PFState,
    covariance_dyn: nalgebra::DMatrix<f64>,
}

impl ParticleFilterLocalizer {
    /// Create a new Particle Filter localizer
    pub fn new(config: ParticleFilterConfig) -> Self {
        Self::try_new(config).expect(
            "invalid particle filter configuration: particle count, noises, and dt must be valid",
        )
    }

    /// Create a validated Particle Filter localizer
    pub fn try_new(config: ParticleFilterConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let n = config.n_particles;
        let particles = (0..n)
            .map(|_| Particle::new(0.0, 0.0, 0.0, 0.0, n))
            .collect();

        let mut pf = ParticleFilterLocalizer {
            particles,
            config,
            landmarks: Vec::new(),
            last_control: PFControl::zeros(),
            state_estimate: PFState::zeros(),
            covariance_dyn: nalgebra::DMatrix::zeros(4, 4),
        };
        pf.refresh_cache();
        Ok(pf)
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(ParticleFilterConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: PFState, config: ParticleFilterConfig) -> Self {
        Self::try_with_initial_state(initial_state, config)
            .expect("invalid particle filter initialization: state and configuration must be valid")
    }

    /// Create with validated initial state
    pub fn try_with_initial_state(
        initial_state: PFState,
        config: ParticleFilterConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        Self::validate_state(&initial_state)?;

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

        let mut pf = ParticleFilterLocalizer {
            particles,
            config,
            landmarks: Vec::new(),
            last_control: PFControl::zeros(),
            state_estimate: PFState::zeros(),
            covariance_dyn: nalgebra::DMatrix::zeros(4, 4),
        };
        pf.refresh_cache();
        Ok(pf)
    }

    /// Create with common State2D type
    pub fn with_initial_state_2d(
        initial_state: State2D,
        config: ParticleFilterConfig,
    ) -> RoboticsResult<Self> {
        Self::try_with_initial_state(initial_state.to_vector(), config)
    }

    /// Set landmarks for observation
    pub fn set_landmarks(&mut self, landmarks: Vec<Point2D>) {
        self.try_set_landmarks(landmarks)
            .expect("particle filter landmarks must contain only finite values")
    }

    /// Set landmarks with validation
    pub fn try_set_landmarks(&mut self, landmarks: Vec<Point2D>) -> RoboticsResult<()> {
        Self::validate_points(&landmarks, "particle filter landmarks")?;
        self.landmarks = landmarks;
        Ok(())
    }

    /// Set landmarks from typed point container
    pub fn set_landmarks_from_obstacles(&mut self, landmarks: &Obstacles) -> RoboticsResult<()> {
        self.try_set_landmarks(landmarks.points.clone())
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
        self.try_predict_with_control(control)
            .expect("invalid particle filter prediction input")
    }

    /// Prediction step with validation
    pub fn try_predict_with_control(&mut self, control: &PFControl) -> RoboticsResult<()> {
        Self::validate_control(control)?;

        let mut rng = rand::thread_rng();
        let normal_v = if self.config.velocity_noise > 0.0 {
            Some(Normal::new(0.0, self.config.velocity_noise).map_err(|_| {
                RoboticsError::InvalidParameter(
                    "particle filter velocity_noise must be a valid standard deviation".to_string(),
                )
            })?)
        } else {
            None
        };
        let normal_yaw = if self.config.yaw_rate_noise > 0.0 {
            Some(Normal::new(0.0, self.config.yaw_rate_noise).map_err(|_| {
                RoboticsError::InvalidParameter(
                    "particle filter yaw_rate_noise must be a valid standard deviation".to_string(),
                )
            })?)
        } else {
            None
        };
        let dt = self.config.dt;

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

            particle.x += v_noisy * particle.yaw.cos() * dt;
            particle.y += v_noisy * particle.yaw.sin() * dt;
            particle.yaw += yaw_rate_noisy * dt;
            particle.v = v_noisy;
        }

        self.last_control = *control;
        self.refresh_cache();
        Ok(())
    }

    /// Update step: update weights based on observations
    pub fn update_with_observations(&mut self, observations: &PFMeasurement) {
        self.try_update_with_observations(observations)
            .expect("invalid particle filter observations")
    }

    /// Update step with validation
    pub fn try_update_with_observations(
        &mut self,
        observations: &PFMeasurement,
    ) -> RoboticsResult<()> {
        Self::validate_observations(observations)?;

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
        self.refresh_cache();
        Ok(())
    }

    /// Resample particles if effective particle count is low
    pub fn resample(&mut self) {
        let n_eff = self.calc_n_eff();
        let threshold = self.config.n_particles as f64 * self.config.resample_threshold;

        if n_eff < threshold {
            self.resample_particles();
            self.refresh_cache();
        }
    }

    /// Compute weighted state estimate
    pub fn estimate(&self) -> PFState {
        self.state_estimate
    }

    /// Get current estimate as State2D
    pub fn state_2d(&self) -> State2D {
        State2D::new(
            self.state_estimate[0],
            self.state_estimate[1],
            self.state_estimate[2],
            self.state_estimate[3],
        )
    }

    /// Compute state covariance
    pub fn calc_covariance(&self) -> Matrix4<f64> {
        Matrix4::from_fn(|i, j| self.covariance_dyn[(i, j)])
    }

    /// Predict with common control type
    pub fn try_predict_input(&mut self, control: ControlInput) -> RoboticsResult<()> {
        self.try_predict_with_control(&control.to_vector())
    }

    /// Full step with common control type returning State2D
    pub fn try_step_state(
        &mut self,
        control: ControlInput,
        observations: &PFMeasurement,
    ) -> RoboticsResult<State2D> {
        self.try_step(&control.to_vector(), observations)?;
        Ok(self.state_2d())
    }

    fn compute_estimate(&self) -> PFState {
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

    fn compute_covariance(&self, x_est: &PFState) -> Matrix4<f64> {
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
        } else {
            let uniform_weight = 1.0 / self.particles.len() as f64;
            for particle in &mut self.particles {
                particle.w = uniform_weight;
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
        self.try_step(control, observations)
            .expect("invalid particle filter step input")
    }

    /// Full validated estimation step (predict + update + resample)
    pub fn try_step(
        &mut self,
        control: &PFControl,
        observations: &PFMeasurement,
    ) -> RoboticsResult<PFState> {
        self.try_predict_with_control(control)?;
        self.try_update_with_observations(observations)?;
        self.resample();
        Ok(self.estimate())
    }

    fn refresh_cache(&mut self) {
        self.state_estimate = self.compute_estimate();
        let covariance = self.compute_covariance(&self.state_estimate);
        self.covariance_dyn = nalgebra::DMatrix::from_fn(4, 4, |i, j| covariance[(i, j)]);
    }

    fn validate_state(state: &PFState) -> RoboticsResult<()> {
        if state.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "particle filter state must contain only finite values".to_string(),
            ));
        }

        Ok(())
    }

    fn validate_control(control: &PFControl) -> RoboticsResult<()> {
        if control.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "particle filter control input must contain only finite values".to_string(),
            ));
        }

        Ok(())
    }

    fn validate_points(points: &[Point2D], label: &str) -> RoboticsResult<()> {
        if points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must contain only finite values"
            )));
        }

        Ok(())
    }

    fn validate_observations(observations: &PFMeasurement) -> RoboticsResult<()> {
        if observations
            .iter()
            .any(|(d, x, y)| !d.is_finite() || !x.is_finite() || !y.is_finite() || *d < 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "particle filter observations must have finite, non-negative distances".to_string(),
            ));
        }

        Ok(())
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
        &self.state_estimate
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        Some(&self.covariance_dyn)
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

    #[test]
    fn test_particle_filter_try_new_rejects_invalid_config() {
        let config = ParticleFilterConfig {
            n_particles: 0,
            ..Default::default()
        };

        let err = match ParticleFilterLocalizer::try_new(config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_particle_filter_with_initial_state_2d() {
        let pf = ParticleFilterLocalizer::with_initial_state_2d(
            State2D::new(1.0, 2.0, 0.3, 0.4),
            ParticleFilterConfig::default(),
        )
        .unwrap();

        let state = pf.state_2d();
        assert!((state.x - 1.0).abs() < 2.0);
        assert!((state.y - 2.0).abs() < 2.0);
    }

    #[test]
    fn test_particle_filter_set_landmarks_from_obstacles() {
        let mut pf = ParticleFilterLocalizer::with_defaults();
        let landmarks =
            Obstacles::from_points(vec![Point2D::new(1.0, 1.0), Point2D::new(2.0, 2.0)]);

        pf.set_landmarks_from_obstacles(&landmarks).unwrap();
        assert_eq!(pf.get_landmarks().len(), 2);
    }

    #[test]
    fn test_particle_filter_try_step_state() {
        let mut pf = ParticleFilterLocalizer::with_defaults();
        let state = pf
            .try_step_state(ControlInput::new(1.0, 0.1), &vec![(10.0, 10.0, 0.0)])
            .unwrap();

        assert!(state.x.is_finite());
        assert!(state.y.is_finite());
    }

    #[test]
    fn test_particle_filter_state_estimator_trait_access() {
        let mut pf = ParticleFilterLocalizer::with_defaults();
        pf.predict(&PFControl::new(1.0, 0.0), 0.1);
        pf.update(&vec![(10.0, 0.0, 0.0)]);

        let state = pf.get_state();
        let covariance = pf.get_covariance().unwrap();
        assert!(state[0].is_finite());
        assert_eq!(covariance.nrows(), 4);
    }
}
