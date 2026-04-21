//! Ensemble Kalman Filter (EnKF) localization
//!
//! Implements state estimation using an ensemble (collection) of state particles.
//! Instead of propagating a covariance matrix analytically, EnKF maintains an
//! ensemble of state samples and derives the covariance from their spread.
//!
//! Reference: Evensen, "Ensemble Kalman filtering",
//! Quarterly Journal of the Royal Meteorological Society, 2003.

use nalgebra::{DMatrix, DVector, Matrix4, Vector2, Vector4};
use rand_distr::{Distribution, Normal};

use rust_robotics_core::{
    ControlInput, Point2D, RoboticsError, RoboticsResult, State2D, StateEstimator,
};

/// State representation for EnKF \[x, y, yaw, v\]
pub type EnKFState = Vector4<f64>;

/// Control input \[velocity, yaw_rate\]
pub type EnKFControl = Vector2<f64>;

/// Measurement \[x, y\]
pub type EnKFMeasurement = Vector2<f64>;

/// Configuration for EnKF localizer
#[derive(Debug, Clone)]
pub struct EnKFConfig {
    /// Number of ensemble members
    pub num_particles: usize,
    /// Process noise standard deviations \[v_noise, yaw_rate_noise\]
    pub process_noise_std: Vector2<f64>,
    /// Measurement noise standard deviations \[x_noise, y_noise\]
    pub measurement_noise_std: Vector2<f64>,
}

impl Default for EnKFConfig {
    fn default() -> Self {
        Self {
            num_particles: 20,
            process_noise_std: Vector2::new(0.2, 1.0_f64.to_radians()),
            measurement_noise_std: Vector2::new(0.5, 0.5),
        }
    }
}

impl EnKFConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.num_particles < 2 {
            return Err(RoboticsError::InvalidParameter(
                "EnKF requires at least 2 ensemble members".to_string(),
            ));
        }
        if self
            .process_noise_std
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "EnKF process noise std must be non-negative and finite".to_string(),
            ));
        }
        if self
            .measurement_noise_std
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "EnKF measurement noise std must be non-negative and finite".to_string(),
            ));
        }
        Ok(())
    }
}

/// Ensemble Kalman Filter for robot localization
///
/// Maintains an ensemble of state particles. At each step:
/// 1. Predict: propagate each ensemble member through the motion model with
///    perturbed control inputs.
/// 2. Update: compute sample cross-covariance and observation covariance,
///    then apply the Kalman gain to each ensemble member.
#[derive(Debug)]
pub struct EnKFLocalizer {
    /// Current state estimate (ensemble mean) \[x, y, yaw, v\]
    x: EnKFState,
    /// State covariance (derived from ensemble spread)
    p: Matrix4<f64>,
    /// Ensemble matrix: each column is one ensemble member (4 x N)
    ensemble: DMatrix<f64>,
    /// Configuration
    config: EnKFConfig,
    /// Dynamic covariance cache for trait-based access
    covariance_dyn: DMatrix<f64>,
    /// Last control input (stored for trait predict/update split)
    last_control: EnKFControl,
}

impl EnKFLocalizer {
    /// Create a new EnKF localizer
    pub fn new(config: EnKFConfig) -> Self {
        Self::try_new(config).expect("invalid EnKF configuration")
    }

    /// Create a validated EnKF localizer
    pub fn try_new(config: EnKFConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let np = config.num_particles;
        // Initialize ensemble with small random perturbations to avoid degeneracy
        let mut rng = rand::rng();
        let noise = Normal::new(0.0, 0.1).unwrap();
        let mut ensemble = DMatrix::zeros(4, np);
        for j in 0..np {
            for i in 0..4 {
                ensemble[(i, j)] = noise.sample(&mut rng);
            }
        }
        let mut enkf = Self {
            x: EnKFState::zeros(),
            p: Matrix4::identity(),
            ensemble,
            config,
            covariance_dyn: DMatrix::identity(4, 4),
            last_control: EnKFControl::zeros(),
        };
        enkf.refresh_covariance_cache();
        Ok(enkf)
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(EnKFConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: EnKFState, config: EnKFConfig) -> Self {
        Self::try_with_initial_state(initial_state, config).expect("invalid EnKF initialization")
    }

    /// Create with validated initial state
    pub fn try_with_initial_state(
        initial_state: EnKFState,
        config: EnKFConfig,
    ) -> RoboticsResult<Self> {
        Self::validate_state(&initial_state)?;
        let mut enkf = Self::try_new(config)?;
        enkf.x = initial_state;
        // Initialize ensemble around the initial state with small perturbations
        let mut rng = rand::rng();
        let noise = Normal::new(0.0, 0.1).unwrap();
        for j in 0..enkf.ensemble.ncols() {
            for i in 0..4 {
                enkf.ensemble[(i, j)] = initial_state[i] + noise.sample(&mut rng);
            }
        }
        Ok(enkf)
    }

    /// Create with common State2D type
    pub fn with_initial_state_2d(
        initial_state: State2D,
        config: EnKFConfig,
    ) -> RoboticsResult<Self> {
        Self::try_with_initial_state(initial_state.to_vector(), config)
    }

    /// Get current state estimate (ensemble mean)
    pub fn estimate(&self) -> EnKFState {
        self.x
    }

    /// Get current estimate as State2D
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.x[0], self.x[1], self.x[2], self.x[3])
    }

    /// Get state covariance (derived from ensemble)
    pub fn covariance(&self) -> &Matrix4<f64> {
        &self.p
    }

    /// Get ensemble matrix (4 x N)
    pub fn ensemble(&self) -> &DMatrix<f64> {
        &self.ensemble
    }

    /// Number of ensemble members
    pub fn num_particles(&self) -> usize {
        self.config.num_particles
    }

    // ========================================================================
    // Motion model
    // ========================================================================

    /// Motion model: state transition with control input
    fn motion_model(x: &EnKFState, u: &EnKFControl, dt: f64) -> EnKFState {
        let yaw = x[2];
        Vector4::new(
            x[0] + dt * x[3] * yaw.cos(),
            x[1] + dt * x[3] * yaw.sin(),
            x[2] + dt * u[1],
            u[0],
        )
    }

    /// Observation model: extract position from state
    fn observation_model(x: &EnKFState) -> EnKFMeasurement {
        Vector2::new(x[0], x[1])
    }

    // ========================================================================
    // Prediction
    // ========================================================================

    /// Prediction step: propagate ensemble with perturbed control
    pub fn predict_with_control(&mut self, control: &EnKFControl, dt: f64) {
        self.try_predict_with_control(control, dt)
            .expect("invalid EnKF prediction input")
    }

    /// Prediction step with validation
    pub fn try_predict_with_control(
        &mut self,
        control: &EnKFControl,
        dt: f64,
    ) -> RoboticsResult<()> {
        Self::validate_control(control)?;
        Self::validate_dt(dt)?;

        let mut rng = rand::rng();
        let v_noise = Normal::new(0.0, self.config.process_noise_std[0]).map_err(|e| {
            RoboticsError::InvalidParameter(format!("Invalid velocity noise distribution: {}", e))
        })?;
        let yaw_noise = Normal::new(0.0, self.config.process_noise_std[1]).map_err(|e| {
            RoboticsError::InvalidParameter(format!("Invalid yaw rate noise distribution: {}", e))
        })?;

        let np = self.config.num_particles;
        for j in 0..np {
            let xi = EnKFState::new(
                self.ensemble[(0, j)],
                self.ensemble[(1, j)],
                self.ensemble[(2, j)],
                self.ensemble[(3, j)],
            );

            // Perturbed control
            let ud = EnKFControl::new(
                control[0] + v_noise.sample(&mut rng),
                control[1] + yaw_noise.sample(&mut rng),
            );

            let xi_pred = Self::motion_model(&xi, &ud, dt);
            for i in 0..4 {
                self.ensemble[(i, j)] = xi_pred[i];
            }
        }

        self.update_mean_and_covariance();
        self.last_control = *control;
        Ok(())
    }

    // ========================================================================
    // Update
    // ========================================================================

    /// Update step with position measurement
    pub fn update_with_measurement(&mut self, z: &EnKFMeasurement) {
        self.try_update_with_measurement(z)
            .expect("invalid EnKF update input")
    }

    /// Update step with validation
    pub fn try_update_with_measurement(&mut self, z: &EnKFMeasurement) -> RoboticsResult<()> {
        Self::validate_measurement(z)?;

        let np = self.config.num_particles;
        let nz = 2_usize;

        // Compute predicted observations for each ensemble member
        let mut pz = DMatrix::zeros(nz, np);
        for j in 0..np {
            let xi = EnKFState::new(
                self.ensemble[(0, j)],
                self.ensemble[(1, j)],
                self.ensemble[(2, j)],
                self.ensemble[(3, j)],
            );
            let zi = Self::observation_model(&xi);
            pz[(0, j)] = zi[0];
            pz[(1, j)] = zi[1];
        }

        // Ensemble means
        let x_mean = DVector::from_fn(4, |i, _| {
            (0..np).map(|j| self.ensemble[(i, j)]).sum::<f64>() / np as f64
        });
        let z_mean = DVector::from_fn(nz, |i, _| {
            (0..np).map(|j| pz[(i, j)]).sum::<f64>() / np as f64
        });

        // Deviations from mean
        let mut x_dif = DMatrix::zeros(4, np);
        let mut z_dif = DMatrix::zeros(nz, np);
        for j in 0..np {
            for i in 0..4 {
                x_dif[(i, j)] = self.ensemble[(i, j)] - x_mean[i];
            }
            for i in 0..nz {
                z_dif[(i, j)] = pz[(i, j)] - z_mean[i];
            }
        }

        // Cross-covariance U = (1/(N-1)) * X_dif * Z_dif^T
        let scale = 1.0 / (np as f64 - 1.0);
        let u_mat = scale * &x_dif * z_dif.transpose();

        // Observation covariance V = (1/(N-1)) * Z_dif * Z_dif^T
        let v_mat = scale * &z_dif * z_dif.transpose();

        // Kalman gain K = U * V^{-1}
        let v_inv = v_mat.clone().try_inverse().ok_or_else(|| {
            RoboticsError::NumericalError(
                "Failed to invert EnKF observation covariance".to_string(),
            )
        })?;
        let k = &u_mat * &v_inv;

        // Update each ensemble member
        let z_vec = DVector::from_vec(vec![z[0], z[1]]);
        for j in 0..np {
            let pz_j = DVector::from_fn(nz, |i, _| pz[(i, j)]);
            let innovation = &z_vec - &pz_j;
            let correction = &k * &innovation;
            for i in 0..4 {
                self.ensemble[(i, j)] += correction[i];
            }
        }

        self.update_mean_and_covariance();
        Ok(())
    }

    // ========================================================================
    // Combined step
    // ========================================================================

    /// Full estimation step (predict + update)
    pub fn step(
        &mut self,
        control: &EnKFControl,
        measurement: &EnKFMeasurement,
        dt: f64,
    ) -> EnKFState {
        self.try_step(control, measurement, dt)
            .expect("invalid EnKF step input or numerical failure")
    }

    /// Full validated estimation step
    pub fn try_step(
        &mut self,
        control: &EnKFControl,
        measurement: &EnKFMeasurement,
        dt: f64,
    ) -> RoboticsResult<EnKFState> {
        self.try_predict_with_control(control, dt)?;
        self.try_update_with_measurement(measurement)?;
        Ok(self.x)
    }

    /// Full estimation step using common crate types
    pub fn estimate_state(
        &mut self,
        control: ControlInput,
        measurement: Point2D,
        dt: f64,
    ) -> RoboticsResult<State2D> {
        self.try_step(&control.to_vector(), &measurement.to_vector(), dt)?;
        Ok(self.state_2d())
    }

    /// Calculate position uncertainty (2-sigma ellipse parameters)
    pub fn position_uncertainty(&self) -> (f64, f64, f64) {
        let p_xy = self.p.fixed_view::<2, 2>(0, 0);
        let eigen = p_xy.symmetric_eigen();

        let (big_idx, small_idx) = if eigen.eigenvalues[0] >= eigen.eigenvalues[1] {
            (0, 1)
        } else {
            (1, 0)
        };

        let a = 2.0 * eigen.eigenvalues[big_idx].abs().sqrt();
        let b = 2.0 * eigen.eigenvalues[small_idx].abs().sqrt();
        let angle = eigen.eigenvectors[(1, big_idx)].atan2(eigen.eigenvectors[(0, big_idx)]);

        (a, b, angle)
    }

    // ========================================================================
    // Internal helpers
    // ========================================================================

    /// Recompute ensemble mean and covariance from the current ensemble
    fn update_mean_and_covariance(&mut self) {
        let np = self.config.num_particles;

        // Mean
        let mut mean = EnKFState::zeros();
        for j in 0..np {
            for i in 0..4 {
                mean[i] += self.ensemble[(i, j)];
            }
        }
        mean /= np as f64;
        self.x = mean;

        // Covariance P = (1/N) * sum( (x_i - mean)(x_i - mean)^T )
        let mut cov = Matrix4::<f64>::zeros();
        for j in 0..np {
            let diff = Vector4::new(
                self.ensemble[(0, j)] - mean[0],
                self.ensemble[(1, j)] - mean[1],
                self.ensemble[(2, j)] - mean[2],
                self.ensemble[(3, j)] - mean[3],
            );
            cov += diff * diff.transpose();
        }
        cov /= np as f64;
        self.p = cov;
        self.refresh_covariance_cache();
    }

    fn refresh_covariance_cache(&mut self) {
        self.covariance_dyn = DMatrix::from_fn(4, 4, |i, j| self.p[(i, j)]);
    }

    fn validate_state(state: &EnKFState) -> RoboticsResult<()> {
        if state.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EnKF state must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_control(control: &EnKFControl) -> RoboticsResult<()> {
        if control.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EnKF control input must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_measurement(measurement: &EnKFMeasurement) -> RoboticsResult<()> {
        if measurement.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EnKF measurement must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_dt(dt: f64) -> RoboticsResult<()> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "EnKF dt must be positive and finite, got {}",
                dt
            )));
        }
        Ok(())
    }
}

impl StateEstimator for EnKFLocalizer {
    type State = EnKFState;
    type Measurement = EnKFMeasurement;
    type Control = EnKFControl;

    fn predict(&mut self, control: &Self::Control, dt: f64) {
        self.predict_with_control(control, dt);
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        self.update_with_measurement(measurement);
    }

    fn get_state(&self) -> &Self::State {
        &self.x
    }

    fn get_covariance(&self) -> Option<&DMatrix<f64>> {
        Some(&self.covariance_dyn)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enkf_creation() {
        let enkf = EnKFLocalizer::with_defaults();
        assert_eq!(enkf.x, EnKFState::zeros());
        assert_eq!(enkf.num_particles(), 20);
    }

    #[test]
    fn test_enkf_with_initial_state() {
        let initial = EnKFState::new(1.0, 2.0, 0.5, 1.0);
        let enkf = EnKFLocalizer::with_initial_state(initial, EnKFConfig::default());
        assert_eq!(enkf.estimate()[0], 1.0);
        assert_eq!(enkf.estimate()[1], 2.0);
    }

    #[test]
    fn test_enkf_ensemble_initialized() {
        let initial = EnKFState::new(3.0, 4.0, 0.0, 1.0);
        let enkf = EnKFLocalizer::with_initial_state(initial, EnKFConfig::default());
        let ens = enkf.ensemble();
        assert_eq!(ens.nrows(), 4);
        assert_eq!(ens.ncols(), 20);
        // Ensemble members should be near initial state (with small noise)
        for j in 0..20 {
            assert!((ens[(0, j)] - 3.0).abs() < 1.0, "x member {j} too far");
            assert!((ens[(1, j)] - 4.0).abs() < 1.0, "y member {j} too far");
        }
    }

    #[test]
    fn test_enkf_predict() {
        let initial = EnKFState::new(0.0, 0.0, 0.0, 0.0);
        let mut enkf = EnKFLocalizer::with_initial_state(initial, EnKFConfig::default());
        let control = EnKFControl::new(1.0, 0.0);
        enkf.predict_with_control(&control, 0.1);
        // After prediction, ensemble should have spread
        let p = enkf.covariance();
        // Covariance should be non-zero from process noise
        assert!(p[(0, 0)] > 0.0 || p[(3, 3)] > 0.0);
    }

    #[test]
    fn test_enkf_update() {
        let initial = EnKFState::new(5.0, 5.0, 0.0, 1.0);
        let config = EnKFConfig {
            num_particles: 50,
            ..Default::default()
        };
        let mut enkf = EnKFLocalizer::with_initial_state(initial, config);
        // Predict first to create ensemble spread
        let control = EnKFControl::new(1.0, 0.0);
        enkf.predict_with_control(&control, 0.1);

        let z = EnKFMeasurement::new(5.1, 5.0);
        enkf.update_with_measurement(&z);
        assert!((enkf.x[0] - 5.1).abs() < 2.0);
    }

    #[test]
    fn test_enkf_step() {
        let mut enkf = EnKFLocalizer::with_defaults();
        let control = EnKFControl::new(1.0, 0.1);
        let measurement = EnKFMeasurement::new(0.1, 0.01);
        let est = enkf.step(&control, &measurement, 0.1);
        assert!(est[0].is_finite());
        assert!(est[1].is_finite());
    }

    #[test]
    fn test_enkf_convergence() {
        let config = EnKFConfig {
            num_particles: 50,
            process_noise_std: Vector2::new(0.1, 0.5_f64.to_radians()),
            measurement_noise_std: Vector2::new(0.3, 0.3),
        };
        let mut enkf = EnKFLocalizer::new(config);
        let control = EnKFControl::new(1.0, 0.0);

        for i in 0..30 {
            let true_x = 0.1 * (i + 1) as f64;
            let z = EnKFMeasurement::new(true_x, 0.0);
            enkf.step(&control, &z, 0.1);
        }

        // Estimate should be in reasonable range of true_x=3.0
        assert!(
            (enkf.x[0] - 3.0).abs() < 2.0,
            "EnKF did not converge: x = {}",
            enkf.x[0]
        );
    }

    #[test]
    fn test_enkf_position_uncertainty() {
        let mut enkf = EnKFLocalizer::with_defaults();
        let control = EnKFControl::new(1.0, 0.0);
        enkf.predict_with_control(&control, 0.1);

        let (a, b, angle) = enkf.position_uncertainty();
        assert!(a >= 0.0);
        assert!(b >= 0.0);
        assert!(angle.is_finite());
    }

    #[test]
    fn test_state_estimator_trait() {
        let mut enkf = EnKFLocalizer::with_defaults();
        let control = EnKFControl::new(1.0, 0.1);
        let measurement = EnKFMeasurement::new(0.1, 0.01);

        enkf.predict(&control, 0.1);
        enkf.update(&measurement);

        let state = enkf.get_state();
        assert!(state[0].is_finite());
    }

    #[test]
    fn test_enkf_try_new_rejects_too_few_particles() {
        let config = EnKFConfig {
            num_particles: 1,
            ..Default::default()
        };
        let err = EnKFLocalizer::try_new(config).unwrap_err();
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_enkf_rejects_invalid_dt() {
        let mut enkf = EnKFLocalizer::with_defaults();
        let err = enkf
            .try_predict_with_control(&EnKFControl::new(1.0, 0.0), -0.1)
            .unwrap_err();
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_enkf_with_initial_state_2d() {
        let enkf = EnKFLocalizer::with_initial_state_2d(
            State2D::new(1.0, 2.0, 0.3, 0.5),
            EnKFConfig::default(),
        )
        .unwrap();
        let state = enkf.state_2d();
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
    }

    #[test]
    fn test_enkf_estimate_state_with_common_types() {
        let mut enkf = EnKFLocalizer::with_defaults();
        let state = enkf
            .estimate_state(ControlInput::new(1.0, 0.1), Point2D::new(0.1, 0.01), 0.1)
            .unwrap();
        assert!(state.x.is_finite());
    }

    #[test]
    fn test_enkf_get_covariance_trait_access() {
        let enkf = EnKFLocalizer::with_defaults();
        let cov = enkf.get_covariance().unwrap();
        assert_eq!(cov.nrows(), 4);
        assert_eq!(cov.ncols(), 4);
    }
}
