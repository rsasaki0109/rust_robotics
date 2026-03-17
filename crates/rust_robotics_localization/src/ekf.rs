//! Extended Kalman Filter (EKF) localization
//!
//! Implements state estimation using the Extended Kalman Filter algorithm
//! for robot localization with nonlinear motion and observation models.

use rust_robotics_core::{
    ControlInput, Point2D, RoboticsError, RoboticsResult, State2D, StateEstimator,
};
use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, Vector2, Vector4};

/// State representation for EKF (x, y, yaw, velocity)
pub type EKFState = Vector4<f64>;

/// Measurement representation (x, y position)
pub type EKFMeasurement = Vector2<f64>;

/// Control input (velocity, yaw rate)
pub type EKFControl = Vector2<f64>;

/// Configuration for EKF
#[derive(Debug, Clone)]
pub struct EKFConfig {
    /// Process noise covariance matrix
    pub q: Matrix4<f64>,
    /// Measurement noise covariance matrix
    pub r: Matrix2<f64>,
}

impl Default for EKFConfig {
    fn default() -> Self {
        let mut q = Matrix4::<f64>::identity();
        q[(0, 0)] = 0.1_f64.powi(2);
        q[(1, 1)] = (1.0_f64.to_radians()).powi(2);
        q[(2, 2)] = 0.1_f64.powi(2);
        q[(3, 3)] = 0.1_f64.powi(2);

        let r = Matrix2::<f64>::identity();

        Self { q, r }
    }
}

impl EKFConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.q.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF process noise matrix must contain only finite values".to_string(),
            ));
        }
        if self.r.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..4 {
            if self.q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "EKF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        for i in 0..2 {
            if self.r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "EKF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }

        Ok(())
    }
}

/// Extended Kalman Filter for robot localization
pub struct EKFLocalizer {
    /// Current state estimate [x, y, yaw, v]
    state: EKFState,
    /// State covariance matrix
    covariance: Matrix4<f64>,
    /// Configuration
    config: EKFConfig,
}

impl EKFLocalizer {
    /// Create a new EKF localizer
    pub fn new(config: EKFConfig) -> Self {
        Self::try_new(config).expect(
            "invalid EKF configuration: noise matrices must be finite and have non-negative diagonals",
        )
    }

    /// Create a new validated EKF localizer
    pub fn try_new(config: EKFConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(EKFLocalizer {
            state: EKFState::zeros(),
            covariance: Matrix4::identity(),
            config,
        })
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(EKFConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: EKFState, config: EKFConfig) -> Self {
        Self::try_with_initial_state(initial_state, config)
            .expect("invalid EKF initialization: state must be finite and config must be valid")
    }

    /// Create with validated initial state
    pub fn try_with_initial_state(
        initial_state: EKFState,
        config: EKFConfig,
    ) -> RoboticsResult<Self> {
        config.validate()?;
        Self::validate_state_vector(&initial_state)?;
        Ok(EKFLocalizer {
            state: initial_state,
            covariance: Matrix4::identity(),
            config,
        })
    }

    /// Create with common State2D type
    pub fn with_initial_state_2d(
        initial_state: State2D,
        config: EKFConfig,
    ) -> RoboticsResult<Self> {
        Self::try_with_initial_state(initial_state.to_vector(), config)
    }

    /// Get reference to state covariance
    pub fn get_covariance_matrix(&self) -> &Matrix4<f64> {
        &self.covariance
    }

    /// Get current estimate as State2D
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.state[0], self.state[1], self.state[2], self.state[3])
    }

    /// Set process noise covariance
    pub fn set_process_noise(&mut self, q: Matrix4<f64>) {
        self.try_set_process_noise(q)
            .expect("invalid EKF process noise matrix")
    }

    /// Set process noise covariance with validation
    pub fn try_set_process_noise(&mut self, q: Matrix4<f64>) -> RoboticsResult<()> {
        if q.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF process noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..4 {
            if q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "EKF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }

        self.config.q = q;
        Ok(())
    }

    /// Set measurement noise covariance
    pub fn set_measurement_noise(&mut self, r: Matrix2<f64>) {
        self.try_set_measurement_noise(r)
            .expect("invalid EKF measurement noise matrix")
    }

    /// Set measurement noise covariance with validation
    pub fn try_set_measurement_noise(&mut self, r: Matrix2<f64>) -> RoboticsResult<()> {
        if r.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..2 {
            if r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "EKF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }

        self.config.r = r;
        Ok(())
    }

    /// Motion model: predict state based on control input
    fn motion_model(x: &EKFState, u: &EKFControl, dt: f64) -> EKFState {
        let yaw = x[2];
        let f = Matrix4::new(
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        );
        let b = Matrix4x2::new(dt * yaw.cos(), 0., dt * yaw.sin(), 0., 0., dt, 1., 0.);
        f * x + b * u
    }

    /// Jacobian of motion model with respect to state
    fn jacobian_f(x: &EKFState, u: &EKFControl, dt: f64) -> Matrix4<f64> {
        let yaw = x[2];
        let v = u[0];
        Matrix4::new(
            1.,
            0.,
            -dt * v * yaw.sin(),
            dt * yaw.cos(),
            0.,
            1.,
            dt * v * yaw.cos(),
            dt * yaw.sin(),
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            1.,
        )
    }

    /// Observation model: predict measurement from state
    fn observation_model(x: &EKFState) -> EKFMeasurement {
        let h = Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.);
        h * x
    }

    /// Jacobian of observation model
    fn jacobian_h() -> Matrix2x4<f64> {
        Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.)
    }

    /// Full EKF estimation step (predict + update)
    pub fn estimate(
        &mut self,
        measurement: &EKFMeasurement,
        control: &EKFControl,
        dt: f64,
    ) -> Result<&EKFState, RoboticsError> {
        Self::validate_measurement_vector(measurement)?;
        Self::validate_control_vector(control)?;
        Self::validate_dt(dt)?;

        // Predict
        let x_pred = Self::motion_model(&self.state, control, dt);
        let j_f = Self::jacobian_f(&x_pred, control, dt);
        let p_pred = j_f * self.covariance * j_f.transpose() + self.config.q;

        // Update
        let j_h = Self::jacobian_h();
        let z_pred = Self::observation_model(&x_pred);
        let y = measurement - z_pred;
        let s = j_h * p_pred * j_h.transpose() + self.config.r;

        let s_inv = s.try_inverse().ok_or_else(|| {
            RoboticsError::NumericalError("Failed to invert S matrix".to_string())
        })?;

        let k = p_pred * j_h.transpose() * s_inv;
        self.state = x_pred + k * y;
        self.covariance = (Matrix4::identity() - k * j_h) * p_pred;

        Ok(&self.state)
    }

    /// EKF estimate step using common crate types
    pub fn estimate_state(
        &mut self,
        measurement: Point2D,
        control: ControlInput,
        dt: f64,
    ) -> RoboticsResult<State2D> {
        self.estimate(&measurement.to_vector(), &control.to_vector(), dt)?;
        Ok(self.state_2d())
    }

    /// Legacy interface for EKF estimation (standalone function style)
    pub fn ekf_estimation(
        x_est: EKFState,
        p_est: Matrix4<f64>,
        z: EKFMeasurement,
        u: EKFControl,
        q: Matrix4<f64>,
        r: Matrix2<f64>,
        dt: f64,
    ) -> (EKFState, Matrix4<f64>) {
        let x_pred = Self::motion_model(&x_est, &u, dt);
        let j_f = Self::jacobian_f(&x_pred, &u, dt);
        let p_pred = j_f * p_est * j_f.transpose() + q;

        let j_h = Self::jacobian_h();
        let z_pred = Self::observation_model(&x_pred);
        let y = z - z_pred;
        let s = j_h * p_pred * j_h.transpose() + r;
        let k = p_pred * j_h.transpose() * s.try_inverse().unwrap();
        let new_x_est = x_pred + k * y;
        let new_p_est = (Matrix4::identity() - k * j_h) * p_pred;

        (new_x_est, new_p_est)
    }

    fn validate_state_vector(state: &EKFState) -> RoboticsResult<()> {
        if state.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF state must contain only finite values".to_string(),
            ));
        }

        Ok(())
    }

    fn validate_measurement_vector(measurement: &EKFMeasurement) -> RoboticsResult<()> {
        if measurement.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF measurement must contain only finite values".to_string(),
            ));
        }

        Ok(())
    }

    fn validate_control_vector(control: &EKFControl) -> RoboticsResult<()> {
        if control.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "EKF control input must contain only finite values".to_string(),
            ));
        }

        Ok(())
    }

    fn validate_dt(dt: f64) -> RoboticsResult<()> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "EKF dt must be positive and finite, got {}",
                dt
            )));
        }

        Ok(())
    }
}

impl StateEstimator for EKFLocalizer {
    type State = EKFState;
    type Measurement = EKFMeasurement;
    type Control = EKFControl;

    fn predict(&mut self, control: &Self::Control, dt: f64) {
        let x_pred = Self::motion_model(&self.state, control, dt);
        let j_f = Self::jacobian_f(&x_pred, control, dt);
        self.covariance = j_f * self.covariance * j_f.transpose() + self.config.q;
        self.state = x_pred;
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        let j_h = Self::jacobian_h();
        let z_pred = Self::observation_model(&self.state);
        let y = measurement - z_pred;
        let s = j_h * self.covariance * j_h.transpose() + self.config.r;

        if let Some(s_inv) = s.try_inverse() {
            let k = self.covariance * j_h.transpose() * s_inv;
            self.state += k * y;
            self.covariance = (Matrix4::identity() - k * j_h) * self.covariance;
        }
    }

    fn get_state(&self) -> &Self::State {
        &self.state
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        // Note: We use fixed-size matrix internally, so we don't implement this
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ekf_creation() {
        let ekf = EKFLocalizer::with_defaults();
        let state = ekf.get_state();
        assert_eq!(state[0], 0.0);
        assert_eq!(state[1], 0.0);
    }

    #[test]
    fn test_ekf_predict() {
        let mut ekf = EKFLocalizer::with_defaults();
        let control = EKFControl::new(1.0, 0.0); // move forward at 1 m/s
        ekf.predict(&control, 0.1);
        let state = ekf.get_state();
        // Should move in x direction (yaw is 0)
        assert!(state[0] > 0.0);
        assert!(state[1].abs() < 0.001);
    }

    #[test]
    fn test_ekf_update() {
        let mut ekf = EKFLocalizer::with_defaults();
        let measurement = EKFMeasurement::new(1.0, 1.0);
        ekf.update(&measurement);
        let state = ekf.get_state();
        // State should move towards measurement
        assert!(state[0] > 0.0);
        assert!(state[1] > 0.0);
    }

    #[test]
    fn test_ekf_estimate() {
        let mut ekf = EKFLocalizer::with_defaults();
        let control = EKFControl::new(1.0, 0.1);
        let measurement = EKFMeasurement::new(0.1, 0.01);

        let result = ekf.estimate(&measurement, &control, 0.1);
        assert!(result.is_ok());
    }

    #[test]
    fn test_ekf_legacy_interface() {
        let x_est = EKFState::zeros();
        let p_est = Matrix4::identity();
        let z = EKFMeasurement::new(0.1, 0.0);
        let u = EKFControl::new(1.0, 0.0);
        let q = EKFConfig::default().q;
        let r = EKFConfig::default().r;

        let (new_x, new_p) = EKFLocalizer::ekf_estimation(x_est, p_est, z, u, q, r, 0.1);

        assert!(new_x[0] > 0.0);
        assert!(new_p[(0, 0)] > 0.0);
    }

    #[test]
    fn test_ekf_try_new_rejects_invalid_config() {
        let mut config = EKFConfig::default();
        config.q[(0, 0)] = -1.0;

        let err = match EKFLocalizer::try_new(config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_ekf_with_initial_state_2d() {
        let ekf = EKFLocalizer::with_initial_state_2d(
            State2D::new(1.0, 2.0, 0.3, 0.5),
            EKFConfig::default(),
        )
        .unwrap();

        let state = ekf.state_2d();
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.3);
        assert_eq!(state.v, 0.5);
    }

    #[test]
    fn test_ekf_estimate_state_with_common_types() {
        let mut ekf = EKFLocalizer::with_defaults();
        let state = ekf
            .estimate_state(Point2D::new(0.1, 0.0), ControlInput::new(1.0, 0.0), 0.1)
            .unwrap();

        assert!(state.x > 0.0);
    }

    #[test]
    fn test_ekf_estimate_rejects_invalid_dt() {
        let mut ekf = EKFLocalizer::with_defaults();
        let err = match ekf.estimate(&EKFMeasurement::new(0.0, 0.0), &EKFControl::zeros(), 0.0) {
            Ok(_) => panic!("expected invalid dt to be rejected"),
            Err(err) => err,
        };

        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }
}
