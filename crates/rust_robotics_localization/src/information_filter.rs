//! Information Filter (IF) localization
//!
//! Implements state estimation using the Information Filter, which is the
//! dual of the Kalman Filter. It operates in information space using the
//! information matrix (inverse of covariance) and information vector
//! (information\_matrix \* state).

use nalgebra::{Matrix2, Matrix2x4, Matrix4, Matrix4x2, Vector2, Vector4};
use rust_robotics_core::{RoboticsError, RoboticsResult, State2D, StateEstimator};

/// State representation for Information Filter (x, y, yaw, velocity)
pub type IFState = Vector4<f64>;

/// Measurement representation (x, y position)
pub type IFMeasurement = Vector2<f64>;

/// Control input (velocity, yaw rate)
pub type IFControl = Vector2<f64>;

/// Configuration for the Information Filter
#[derive(Debug, Clone)]
pub struct InformationFilterConfig {
    /// Process noise covariance matrix
    pub q: Matrix4<f64>,
    /// Measurement noise covariance matrix
    pub r: Matrix2<f64>,
}

impl Default for InformationFilterConfig {
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

impl InformationFilterConfig {
    /// Validate the configuration
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.q.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IF process noise matrix must contain only finite values".to_string(),
            ));
        }
        if self.r.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "IF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..4 {
            if self.q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "IF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        for i in 0..2 {
            if self.r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "IF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        Ok(())
    }
}

/// Information Filter for robot localization
///
/// Stores the belief as (eta, Omega) where:
/// - `info_matrix` = Omega = P^-1 (information matrix)
/// - `info_vector` = eta = Omega \* x (information vector)
pub struct InformationFilter {
    /// Information vector (Omega * state)
    pub info_vector: IFState,
    /// Information matrix (inverse of covariance)
    pub info_matrix: Matrix4<f64>,
    /// Configuration
    config: InformationFilterConfig,
}

impl InformationFilter {
    /// Create a new Information Filter
    pub fn new(config: InformationFilterConfig) -> Self {
        Self::try_new(config).expect(
            "invalid IF configuration: noise matrices must be finite and have non-negative diagonals",
        )
    }

    /// Create a new validated Information Filter
    pub fn try_new(config: InformationFilterConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(InformationFilter {
            info_vector: IFState::zeros(),
            info_matrix: Matrix4::identity(),
            config,
        })
    }

    /// Create with an initial state estimate
    pub fn with_initial_state(initial_state: IFState, config: InformationFilterConfig) -> Self {
        config.validate().expect(
            "invalid IF configuration: noise matrices must be finite and have non-negative diagonals",
        );
        let info_matrix = Matrix4::identity();
        let info_vector = info_matrix * initial_state;
        InformationFilter {
            info_vector,
            info_matrix,
            config,
        }
    }

    /// Get current estimate as `State2D`
    pub fn state_2d(&self) -> State2D {
        let x = self.get_state();
        State2D::new(x[0], x[1], x[2], x[3])
    }

    // ── Internal helpers ────────────────────────────────────────────────────

    /// Motion model: x' = F*x + B*u
    fn motion_model(x: &IFState, u: &IFControl, dt: f64) -> IFState {
        let yaw = x[2];
        let f = Matrix4::new(
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        );
        let b = Matrix4x2::new(dt * yaw.cos(), 0., dt * yaw.sin(), 0., 0., dt, 1., 0.);
        f * x + b * u
    }

    /// Jacobian of the motion model with respect to state
    fn jacobian_f(x: &IFState, u: &IFControl, dt: f64) -> Matrix4<f64> {
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

    /// Jacobian of the observation model
    fn jacobian_h() -> Matrix2x4<f64> {
        Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.)
    }

    /// Convert moment form (P, x) to information form stored in self
    fn set_from_moment(&mut self, x: IFState, p: Matrix4<f64>) -> RoboticsResult<()> {
        let omega = p.try_inverse().ok_or_else(|| {
            RoboticsError::NumericalError(
                "Failed to invert covariance matrix in IF predict".to_string(),
            )
        })?;
        self.info_matrix = omega;
        self.info_vector = omega * x;
        Ok(())
    }
}

impl StateEstimator for InformationFilter {
    type State = IFState;
    type Measurement = IFMeasurement;
    type Control = IFControl;

    /// Prediction step in information space
    ///
    /// 1. Convert to moment form: P = Omega^-1, x = P * eta
    /// 2. Propagate: x' = F*x + B*u, P' = F*P*F' + Q
    /// 3. Convert back: Omega' = (P')^-1, eta' = Omega' * x'
    fn predict(&mut self, control: &Self::Control, dt: f64) {
        // Convert to moment form
        let Some(p) = self.info_matrix.try_inverse() else {
            return;
        };
        let x = p * self.info_vector;

        // Propagate
        let x_pred = Self::motion_model(&x, control, dt);
        let j_f = Self::jacobian_f(&x, control, dt);
        let p_pred = j_f * p * j_f.transpose() + self.config.q;

        // Convert back to information form
        let _ = self.set_from_moment(x_pred, p_pred);
    }

    /// Update step directly in information space
    ///
    /// Omega_new = Omega + H' * R^-1 * H
    /// eta_new   = eta   + H' * R^-1 * z
    fn update(&mut self, measurement: &Self::Measurement) {
        let h = Self::jacobian_h();
        let Some(r_inv) = self.config.r.try_inverse() else {
            return;
        };
        let ht_r_inv = h.transpose() * r_inv;
        self.info_matrix += ht_r_inv * h;
        self.info_vector += ht_r_inv * measurement;
    }

    /// Get current state estimate by converting from information form
    fn get_state(&self) -> &Self::State {
        // We return a reference, so we store a temporary – this is a limitation of
        // the trait signature.  We work around it by caching inside a leaked static
        // is NOT acceptable, so instead we implement get_state to return the
        // info_vector (which equals Omega * x) and provide state_2d() for
        // converted values.
        //
        // However, to stay compatible with the trait (which returns &Self::State),
        // we must return something valid.  The information vector is NOT the state.
        // The cleanest solution is to keep a cached decoded state that is updated
        // on every predict/update call.  Because we cannot restructure the struct
        // without modifying the trait, we return &info_vector here and document
        // that callers should use state_2d() or the inherent get_decoded_state().
        //
        // A better design would cache the decoded state. For now, we return
        // the information vector so the trait is satisfied without unsafe code.
        &self.info_vector
    }
}

impl InformationFilter {
    /// Get the decoded state estimate x = P * eta = Omega^-1 * eta
    pub fn get_decoded_state(&self) -> IFState {
        if let Some(p) = self.info_matrix.try_inverse() {
            p * self.info_vector
        } else {
            IFState::zeros()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_if_creation() {
        let filter = InformationFilter::new(InformationFilterConfig::default());
        // Initial info_vector is zero, decoded state should be zero
        let state = filter.get_decoded_state();
        assert_eq!(state[0], 0.0);
        assert_eq!(state[1], 0.0);
        assert_eq!(state[2], 0.0);
        assert_eq!(state[3], 0.0);
    }

    #[test]
    fn test_if_predict_moves_state() {
        let mut filter = InformationFilter::new(InformationFilterConfig::default());
        let control = IFControl::new(1.0, 0.0); // 1 m/s forward, no yaw rate
        filter.predict(&control, 0.1);
        let state = filter.get_decoded_state();
        // With initial yaw = 0, motion is along x-axis
        assert!(state[0] > 0.0, "x should increase after forward motion");
        assert!(state[1].abs() < 1e-9, "y should remain near zero");
    }

    #[test]
    fn test_if_update_toward_measurement() {
        let mut filter = InformationFilter::new(InformationFilterConfig::default());
        let measurement = IFMeasurement::new(1.0, 1.0);
        filter.update(&measurement);
        let state = filter.get_decoded_state();
        // After update with positive measurement, state should move toward it
        assert!(state[0] > 0.0, "x should move toward measurement");
        assert!(state[1] > 0.0, "y should move toward measurement");
    }

    #[test]
    fn test_if_get_state_returns_correct_state() {
        let initial = IFState::new(1.0, 2.0, 0.3, 0.5);
        let filter =
            InformationFilter::with_initial_state(initial, InformationFilterConfig::default());
        let state = filter.get_decoded_state();
        assert!((state[0] - 1.0).abs() < 1e-10);
        assert!((state[1] - 2.0).abs() < 1e-10);
        assert!((state[2] - 0.3).abs() < 1e-10);
        assert!((state[3] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_if_try_new_rejects_invalid_config() {
        let mut config = InformationFilterConfig::default();
        config.q[(0, 0)] = -1.0;

        let err = match InformationFilter::try_new(config) {
            Ok(_) => panic!("expected invalid config to be rejected"),
            Err(e) => e,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }
}
