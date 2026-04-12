//! Complementary Filter for sensor fusion
//!
//! Implements a simple complementary filter that blends high-frequency
//! gyroscope/odometry predictions with low-frequency position measurements.

use nalgebra::{Vector2, Vector4};
use rust_robotics_core::{RoboticsError, RoboticsResult, State2D, StateEstimator};

/// State representation for Complementary Filter \[x, y, yaw, v\]
pub type CFState = Vector4<f64>;

/// Measurement representation \[x, y\] position
pub type CFMeasurement = Vector2<f64>;

/// Control input \[v, yaw_rate\]
pub type CFControl = Vector2<f64>;

/// Configuration for the Complementary Filter
#[derive(Debug, Clone)]
pub struct ComplementaryFilterConfig {
    /// Blend factor: weight given to the prediction (0..=1).
    /// `alpha = 1.0` means pure prediction; `alpha = 0.0` means pure measurement.
    pub alpha: f64,
    /// Time step \[s\]
    pub dt: f64,
}

impl Default for ComplementaryFilterConfig {
    fn default() -> Self {
        Self {
            alpha: 0.98,
            dt: 0.1,
        }
    }
}

impl ComplementaryFilterConfig {
    /// Validate the configuration.
    ///
    /// Returns an error if `alpha` is outside \[0, 1\] or `dt` is not positive.
    pub fn validate(&self) -> RoboticsResult<()> {
        if !(0.0..=1.0).contains(&self.alpha) {
            return Err(RoboticsError::InvalidParameter(format!(
                "ComplementaryFilter alpha must be in [0, 1], got {}",
                self.alpha
            )));
        }
        if !self.dt.is_finite() || self.dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "ComplementaryFilter dt must be positive and finite, got {}",
                self.dt
            )));
        }
        Ok(())
    }
}

/// Complementary Filter for robot localization via sensor fusion
pub struct ComplementaryFilter {
    /// Current state estimate \[x, y, yaw, v\]
    state: CFState,
    /// Configuration
    config: ComplementaryFilterConfig,
}

impl ComplementaryFilter {
    /// Create a new filter with zero initial state.
    ///
    /// # Panics
    ///
    /// Panics if the configuration is invalid.
    pub fn new(config: ComplementaryFilterConfig) -> Self {
        Self::try_new(config)
            .expect("invalid ComplementaryFilter configuration: alpha must be in [0,1] and dt > 0")
    }

    /// Create a new validated filter with zero initial state.
    pub fn try_new(config: ComplementaryFilterConfig) -> RoboticsResult<Self> {
        config.validate()?;
        Ok(Self {
            state: CFState::zeros(),
            config,
        })
    }

    /// Create with the given initial state.
    ///
    /// # Panics
    ///
    /// Panics if the configuration is invalid.
    pub fn with_initial_state(state: CFState, config: ComplementaryFilterConfig) -> Self {
        config
            .validate()
            .expect("invalid ComplementaryFilter configuration");
        Self { state, config }
    }

    /// Get the current estimate as a [`State2D`].
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.state[0], self.state[1], self.state[2], self.state[3])
    }
}

impl StateEstimator for ComplementaryFilter {
    type State = CFState;
    type Measurement = CFMeasurement;
    type Control = CFControl;

    /// Prediction step using the unicycle motion model.
    ///
    /// - `x  += v * cos(yaw) * dt`
    /// - `y  += v * sin(yaw) * dt`
    /// - `yaw += omega * dt`
    /// - `v   = control\[0\]`
    fn predict(&mut self, control: &Self::Control, dt: f64) {
        let yaw = self.state[2];
        let v = control[0];
        let omega = control[1];

        self.state[0] += v * yaw.cos() * dt;
        self.state[1] += v * yaw.sin() * dt;
        self.state[2] += omega * dt;
        self.state[3] = v;
    }

    /// Update step: blend position estimate with measurement.
    ///
    /// `x_new = alpha * x_pred + (1 - alpha) * measurement`
    fn update(&mut self, measurement: &Self::Measurement) {
        let alpha = self.config.alpha;
        self.state[0] = alpha * self.state[0] + (1.0 - alpha) * measurement[0];
        self.state[1] = alpha * self.state[1] + (1.0 - alpha) * measurement[1];
    }

    fn get_state(&self) -> &Self::State {
        &self.state
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cf_pure_prediction() {
        // alpha=1 -> update ignores measurement entirely
        let config = ComplementaryFilterConfig {
            alpha: 1.0,
            dt: 0.1,
        };
        let mut cf = ComplementaryFilter::new(config);

        let control = CFControl::new(1.0, 0.0); // 1 m/s forward, no rotation
        cf.predict(&control, 0.1);

        // After predict, x should have moved, y stays ~0
        let state = cf.get_state();
        assert!((state[0] - 0.1).abs() < 1e-9);
        assert!(state[1].abs() < 1e-9);

        // update with far-away measurement; alpha=1 means state is unchanged
        let measurement = CFMeasurement::new(999.0, 999.0);
        cf.update(&measurement);
        let state = cf.get_state();
        assert!((state[0] - 0.1).abs() < 1e-9);
        assert!(state[1].abs() < 1e-9);
    }

    #[test]
    fn test_cf_pure_measurement() {
        // alpha=0 -> update replaces state with measurement entirely
        let config = ComplementaryFilterConfig {
            alpha: 0.0,
            dt: 0.1,
        };
        let mut cf = ComplementaryFilter::new(config);

        let control = CFControl::new(1.0, 0.0);
        cf.predict(&control, 0.1);

        let measurement = CFMeasurement::new(5.0, 3.0);
        cf.update(&measurement);
        let state = cf.get_state();
        assert!((state[0] - 5.0).abs() < 1e-9);
        assert!((state[1] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_cf_blended() {
        // alpha=0.5 -> equal blend of prediction and measurement
        let config = ComplementaryFilterConfig {
            alpha: 0.5,
            dt: 0.1,
        };
        let mut cf = ComplementaryFilter::new(config);

        let control = CFControl::new(1.0, 0.0);
        cf.predict(&control, 0.1);
        // After predict: x = 0.1, y = 0.0

        let measurement = CFMeasurement::new(0.5, 0.4);
        cf.update(&measurement);
        let state = cf.get_state();

        let expected_x = 0.5 * 0.1 + 0.5 * 0.5;
        let expected_y = 0.5 * 0.0 + 0.5 * 0.4;
        assert!((state[0] - expected_x).abs() < 1e-9);
        assert!((state[1] - expected_y).abs() < 1e-9);
    }

    #[test]
    fn test_cf_validate_rejects_invalid_alpha() {
        let bad_config = ComplementaryFilterConfig {
            alpha: 1.5,
            dt: 0.1,
        };
        let result = ComplementaryFilter::try_new(bad_config);
        assert!(
            matches!(result, Err(RoboticsError::InvalidParameter(_))),
            "expected InvalidParameter for alpha > 1"
        );

        let negative_config = ComplementaryFilterConfig {
            alpha: -0.1,
            dt: 0.1,
        };
        let result2 = ComplementaryFilter::try_new(negative_config);
        assert!(
            matches!(result2, Err(RoboticsError::InvalidParameter(_))),
            "expected InvalidParameter for alpha < 0"
        );
    }
}
