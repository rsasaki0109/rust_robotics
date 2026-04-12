//! PID Controller with anti-windup and output clamping.
use rust_robotics_core::Controller;

/// Configuration for [`PIDController`].
#[derive(Debug, Clone, Copy)]
pub struct PIDConfig {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Time step \[s\].
    pub dt: f64,
    /// Maximum absolute value of the integral term (anti-windup).
    pub max_integral: f64,
    /// Maximum absolute value of the controller output.
    pub max_output: f64,
}

impl Default for PIDConfig {
    fn default() -> Self {
        Self {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            dt: 0.01,
            max_integral: 10.0,
            max_output: f64::INFINITY,
        }
    }
}

/// Discrete-time PID controller.
///
/// Implements the [`Controller`] trait with `State = f64`, `Reference = f64`,
/// and `Output = f64`.
///
/// # Example
///
/// ```
/// use rust_robotics_control::pid_controller::{PIDConfig, PIDController};
/// use rust_robotics_core::Controller;
///
/// let mut pid = PIDController::with_gains(1.0, 0.0, 0.0, 0.01);
/// let output = pid.compute(&0.0, &1.0);
/// assert!((output - 1.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone)]
pub struct PIDController {
    /// Controller configuration.
    pub config: PIDConfig,
    integral: f64,
    prev_error: f64,
}

impl PIDController {
    /// Create a new `PIDController` with the given configuration.
    pub fn new(config: PIDConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    /// Create a `PIDController` with common gains and time step.
    pub fn with_gains(kp: f64, ki: f64, kd: f64, dt: f64) -> Self {
        Self::new(PIDConfig {
            kp,
            ki,
            kd,
            dt,
            ..PIDConfig::default()
        })
    }

    /// Advance the controller by one step given a pre-computed `error`.
    ///
    /// This is a convenience wrapper around [`Controller::compute`] for
    /// situations where the error is already known.
    pub fn step(&mut self, error: f64) -> f64 {
        let state = 0.0_f64;
        let reference = error;
        self.compute(&state, &reference)
    }
}

impl Controller for PIDController {
    type State = f64;
    type Reference = f64;
    type Output = f64;

    /// Compute the PID control output.
    ///
    /// `error = reference - state`.  The integral is clamped to
    /// `±max_integral` (anti-windup) and the output is clamped to
    /// `±max_output`.
    fn compute(&mut self, state: &f64, reference: &f64) -> f64 {
        let error = reference - state;

        self.integral = (self.integral + error * self.config.dt)
            .clamp(-self.config.max_integral, self.config.max_integral);

        let derivative = (error - self.prev_error) / self.config.dt;
        self.prev_error = error;

        let output =
            self.config.kp * error + self.config.ki * self.integral + self.config.kd * derivative;

        output.clamp(-self.config.max_output, self.config.max_output)
    }

    /// Reset the controller's internal state (integral and previous error).
    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rust_robotics_core::Controller;

    #[test]
    fn test_proportional_only() {
        let mut pid = PIDController::with_gains(2.0, 0.0, 0.0, 0.01);
        let output = pid.compute(&0.0, &3.0);
        assert!((output - 6.0).abs() < 1e-10, "expected 6.0, got {output}");
    }

    #[test]
    fn test_integral_accumulation() {
        let mut pid = PIDController::with_gains(0.0, 1.0, 0.0, 0.1);
        // Each step: integral += error * dt = 1.0 * 0.1 = 0.1
        // output = ki * integral
        for _ in 0..5 {
            pid.compute(&0.0, &1.0);
        }
        let output = pid.compute(&0.0, &1.0);
        // After 6 steps integral = 0.6, output = 0.6
        assert!((output - 0.6).abs() < 1e-10, "expected 0.6, got {output}");
    }

    #[test]
    fn test_anti_windup() {
        let config = PIDConfig {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
            dt: 1.0,
            max_integral: 5.0,
            max_output: f64::INFINITY,
        };
        let mut pid = PIDController::new(config);
        // Drive integral far beyond the limit
        for _ in 0..20 {
            pid.compute(&0.0, &1.0);
        }
        // Output = ki * integral, integral clamped to max_integral = 5.0
        let output = pid.compute(&0.0, &0.0);
        assert!(
            output.abs() <= 5.0 + 1e-10,
            "anti-windup failed: output {output} exceeded max_integral"
        );
    }

    #[test]
    fn test_derivative() {
        let mut pid = PIDController::with_gains(0.0, 0.0, 1.0, 0.1);
        // First step: prev_error = 0, error = 1 → derivative = (1 - 0) / 0.1 = 10
        let output = pid.compute(&0.0, &1.0);
        assert!((output - 10.0).abs() < 1e-10, "expected 10.0, got {output}");
        // Second step: error stays 1 → derivative = 0
        let output2 = pid.compute(&0.0, &1.0);
        assert!(output2.abs() < 1e-10, "expected 0.0, got {output2}");
    }

    #[test]
    fn test_reset() {
        let mut pid = PIDController::with_gains(1.0, 1.0, 1.0, 0.1);
        for _ in 0..10 {
            pid.compute(&0.0, &1.0);
        }
        pid.reset();
        // After reset integral=0, prev_error=0 → pure P on next call
        let output = pid.compute(&0.0, &2.0);
        // P = kp * error = 1.0 * 2.0 = 2.0
        // I = ki * (0 + 2.0 * 0.1) = 0.2
        // D = kd * (2.0 - 0) / 0.1 = 20.0
        let expected = 1.0 * 2.0 + 1.0 * (2.0 * 0.1) + 1.0 * (2.0 / 0.1);
        assert!(
            (output - expected).abs() < 1e-10,
            "expected {expected}, got {output}"
        );
    }
}
