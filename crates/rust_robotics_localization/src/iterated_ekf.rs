//! Iterated Extended Kalman Filter (IEKF) localization
//!
//! Improves on the standard EKF by iterating the linearization point during
//! the update step, which gives better accuracy for highly nonlinear models.

use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, Vector2, Vector4};
use rust_robotics_core::{RoboticsError, RoboticsResult, State2D, StateEstimator};

/// State representation for IEKF (x, y, yaw, velocity)
pub type IEKFState = Vector4<f64>;

/// Measurement representation (x, y position)
pub type IEKFMeasurement = Vector2<f64>;

/// Control input (velocity, yaw rate)
pub type IEKFControl = Vector2<f64>;

/// Configuration for IEKF
#[derive(Debug, Clone)]
pub struct IEKFConfig {
    /// Process noise covariance matrix
    pub q: Matrix4<f64>,
    /// Measurement noise covariance matrix
    pub r: Matrix2<f64>,
    /// Maximum number of linearization iterations in the update step
    pub max_iterations: usize,
    /// Convergence tolerance for the iteration (norm of state change)
    pub tolerance: f64,
}

impl Default for IEKFConfig {
    fn default() -> Self {
        let mut q = Matrix4::<f64>::identity();
        q[(0, 0)] = 0.1_f64.powi(2);
        q[(1, 1)] = (1.0_f64.to_radians()).powi(2);
        q[(2, 2)] = 0.1_f64.powi(2);
        q[(3, 3)] = 0.1_f64.powi(2);

        let r = Matrix2::<f64>::identity();

        Self {
            q,
            r,
            max_iterations: 5,
            tolerance: 1e-6,
        }
    }
}

impl IEKFConfig {
    /// Validate that the configuration is numerically sane.
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.q.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IEKF process noise matrix must contain only finite values".to_string(),
            ));
        }
        if self.r.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IEKF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..4 {
            if self.q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "IEKF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        for i in 0..2 {
            if self.r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "IEKF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        if self.max_iterations == 0 {
            return Err(RoboticsError::InvalidParameter(
                "IEKF max_iterations must be greater than zero".to_string(),
            ));
        }
        Ok(())
    }
}

/// Iterated Extended Kalman Filter for robot localization
pub struct IEKFLocalizer {
    /// Current state estimate \[x, y, yaw, v\]
    pub state: IEKFState,
    /// State covariance matrix
    pub covariance: Matrix4<f64>,
    /// Configuration
    pub config: IEKFConfig,
}

impl IEKFLocalizer {
    /// Create a new IEKF localizer; panics on invalid config.
    pub fn new(config: IEKFConfig) -> Self {
        Self::try_new(config).expect(
            "invalid IEKF configuration: noise matrices must be finite and have non-negative diagonals, max_iterations > 0",
        )
    }

    /// Create a new validated IEKF localizer.
    pub fn try_new(config: IEKFConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(Self {
            state: IEKFState::zeros(),
            covariance: Matrix4::identity(),
            config,
        })
    }

    /// Create with an explicit initial state; panics on invalid inputs.
    pub fn with_initial_state(initial_state: IEKFState, config: IEKFConfig) -> Self {
        config.validate().expect("invalid IEKF configuration");
        Self {
            state: initial_state,
            covariance: Matrix4::identity(),
            config,
        }
    }

    /// Get the current estimate as a [`State2D`].
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.state[0], self.state[1], self.state[2], self.state[3])
    }

    // -----------------------------------------------------------------------
    // Models (identical to EKF)
    // -----------------------------------------------------------------------

    fn motion_model(x: &IEKFState, u: &IEKFControl, dt: f64) -> IEKFState {
        let yaw = x[2];
        let f = Matrix4::new(
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        );
        let b = Matrix4x2::new(dt * yaw.cos(), 0., dt * yaw.sin(), 0., 0., dt, 1., 0.);
        f * x + b * u
    }

    fn jacobian_f(x: &IEKFState, u: &IEKFControl, dt: f64) -> Matrix4<f64> {
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

    fn observation_model(x: &IEKFState) -> IEKFMeasurement {
        let h = Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.);
        h * x
    }

    fn jacobian_h() -> Matrix2x4<f64> {
        Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.)
    }

    // -----------------------------------------------------------------------
    // Iterated update (with saved predicted state / covariance)
    // -----------------------------------------------------------------------

    fn iterated_update(
        &mut self,
        measurement: &IEKFMeasurement,
        x_pred: IEKFState,
        p_pred: Matrix4<f64>,
    ) -> RoboticsResult<()> {
        let mut x_i = x_pred;
        let mut k = Matrix4x2::zeros();
        let mut j_h = Self::jacobian_h();

        for _ in 0..self.config.max_iterations {
            j_h = Self::jacobian_h();
            let z_pred = Self::observation_model(&x_i);
            // Innovation linearized around x_i rather than x_pred
            let y = measurement - z_pred - j_h * (x_pred - x_i);
            let s = j_h * p_pred * j_h.transpose() + self.config.r;

            let s_inv = s.try_inverse().ok_or_else(|| {
                RoboticsError::NumericalError(
                    "Failed to invert innovation covariance S".to_string(),
                )
            })?;

            k = p_pred * j_h.transpose() * s_inv;
            let x_new = x_pred + k * y;

            if (x_new - x_i).norm() < self.config.tolerance {
                x_i = x_new;
                break;
            }
            x_i = x_new;
        }

        self.state = x_i;
        self.covariance = (Matrix4::identity() - k * j_h) * p_pred;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Public combined estimate
    // -----------------------------------------------------------------------

    /// Full IEKF estimation step (predict + iterated update).
    pub fn estimate(
        &mut self,
        measurement: &IEKFMeasurement,
        control: &IEKFControl,
        dt: f64,
    ) -> Result<&IEKFState, RoboticsError> {
        if measurement.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IEKF measurement must contain only finite values".to_string(),
            ));
        }
        if control.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IEKF control input must contain only finite values".to_string(),
            ));
        }
        if !dt.is_finite() || dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "IEKF dt must be positive and finite, got {}",
                dt
            )));
        }

        let x_pred = Self::motion_model(&self.state, control, dt);
        let j_f = Self::jacobian_f(&x_pred, control, dt);
        let p_pred = j_f * self.covariance * j_f.transpose() + self.config.q;

        self.iterated_update(measurement, x_pred, p_pred)?;
        Ok(&self.state)
    }
}

// ---------------------------------------------------------------------------
// StateEstimator trait
// ---------------------------------------------------------------------------

impl StateEstimator for IEKFLocalizer {
    type State = IEKFState;
    type Measurement = IEKFMeasurement;
    type Control = IEKFControl;

    /// Prediction step — identical to EKF predict.
    fn predict(&mut self, control: &Self::Control, dt: f64) {
        let x_pred = Self::motion_model(&self.state, control, dt);
        let j_f = Self::jacobian_f(&x_pred, control, dt);
        self.covariance = j_f * self.covariance * j_f.transpose() + self.config.q;
        self.state = x_pred;
    }

    /// Update step using iterated linearization.
    ///
    /// Silently skips the update if the innovation covariance S is singular.
    fn update(&mut self, measurement: &Self::Measurement) {
        let x_pred = self.state;
        let p_pred = self.covariance;
        // Ignore numerical errors in the trait update (no return type)
        let _ = self.iterated_update(measurement, x_pred, p_pred);
    }

    fn get_state(&self) -> &Self::State {
        &self.state
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        None
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iekf_creation_with_defaults() {
        let iekf = IEKFLocalizer::new(IEKFConfig::default());
        let state = iekf.get_state();
        assert_eq!(state[0], 0.0);
        assert_eq!(state[1], 0.0);
        assert_eq!(state[2], 0.0);
        assert_eq!(state[3], 0.0);
        assert_eq!(iekf.config.max_iterations, 5);
        assert!((iekf.config.tolerance - 1e-6).abs() < 1e-12);
    }

    #[test]
    fn test_iekf_predict_moves_state() {
        let mut iekf = IEKFLocalizer::new(IEKFConfig::default());
        let control = IEKFControl::new(1.0, 0.0); // 1 m/s forward, yaw rate 0
        iekf.predict(&control, 0.1);
        let state = iekf.get_state();
        // With zero initial yaw the robot moves along x
        assert!(state[0] > 0.0, "x should increase after forward motion");
        assert!(state[1].abs() < 1e-10, "y should remain ~0");
    }

    #[test]
    fn test_iekf_update_toward_measurement() {
        let mut iekf = IEKFLocalizer::new(IEKFConfig::default());
        // Start at origin; update with a measurement at (1, 1)
        let measurement = IEKFMeasurement::new(1.0, 1.0);
        iekf.update(&measurement);
        let state = iekf.get_state();
        assert!(state[0] > 0.0, "x should move toward measurement");
        assert!(state[1] > 0.0, "y should move toward measurement");
    }

    #[test]
    fn test_iekf_estimate_combined() {
        let mut iekf = IEKFLocalizer::new(IEKFConfig::default());
        let control = IEKFControl::new(1.0, 0.1);
        let measurement = IEKFMeasurement::new(0.1, 0.01);

        let result = iekf.estimate(&measurement, &control, 0.1);
        assert!(result.is_ok(), "estimate should succeed with valid inputs");
        let state = result.unwrap();
        assert!(
            state.iter().all(|v| v.is_finite()),
            "state should be finite"
        );
    }

    #[test]
    fn test_iekf_try_new_rejects_invalid_config() {
        // Negative diagonal in Q
        let mut bad_q = IEKFConfig::default();
        bad_q.q[(0, 0)] = -1.0;
        assert!(matches!(
            IEKFLocalizer::try_new(bad_q),
            Err(RoboticsError::InvalidParameter(_))
        ));

        // max_iterations == 0
        let bad_iter = IEKFConfig {
            max_iterations: 0,
            ..IEKFConfig::default()
        };
        assert!(matches!(
            IEKFLocalizer::try_new(bad_iter),
            Err(RoboticsError::InvalidParameter(_))
        ));
    }
}
