//! Adaptive localization filter
//!
//! Automatically switches between EKF (fast, linear-ish) and CKF (robust,
//! nonlinear) based on innovation magnitude. When the EKF's prediction
//! residual exceeds a threshold, the filter switches to CKF for better
//! nonlinearity handling. When residuals are small, it switches back to
//! EKF for speed.
//!
//! The switching metric is the normalized innovation squared (NIS):
//! NIS = innovation^T * S^{-1} * innovation
//! where S is the innovation covariance. Under correct filter operation,
//! NIS follows a chi-squared distribution with dim(z) degrees of freedom.

use nalgebra::{Matrix2, Matrix4, Vector2, Vector4};
use rust_robotics_core::{RoboticsResult, State2D};

use crate::cubature_kalman_filter::{CKFConfig, CKFLocalizer};
use crate::ekf::{EKFConfig, EKFLocalizer};

/// Which filter is currently active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveFilter {
    EKF,
    CKF,
}

/// Configuration for the adaptive filter.
#[derive(Debug, Clone)]
pub struct AdaptiveFilterConfig {
    /// EKF configuration
    pub ekf_config: EKFConfig,
    /// CKF configuration
    pub ckf_config: CKFConfig,
    /// NIS threshold to switch from EKF to CKF.
    /// When NIS exceeds this, nonlinearity is high → use CKF.
    /// Default: 9.21 (chi-squared 99th percentile for 2 DOF)
    pub nis_switch_to_ckf: f64,
    /// NIS threshold to switch back from CKF to EKF.
    /// When NIS stays below this for `switch_back_count` steps → use EKF.
    /// Default: 5.99 (chi-squared 95th percentile for 2 DOF)
    pub nis_switch_to_ekf: f64,
    /// Number of consecutive low-NIS steps before switching back to EKF.
    /// Default: 5
    pub switch_back_count: usize,
}

impl Default for AdaptiveFilterConfig {
    fn default() -> Self {
        let q = Matrix4::from_diagonal(&Vector4::new(
            0.1_f64.powi(2),
            0.1_f64.powi(2),
            1.0_f64.to_radians().powi(2),
            1.0_f64.powi(2),
        ));
        let mut r = Matrix2::<f64>::identity();
        r[(0, 0)] = 0.5_f64.powi(2);
        r[(1, 1)] = 0.5_f64.powi(2);

        Self {
            ekf_config: EKFConfig { q, r },
            ckf_config: CKFConfig { q, r },
            nis_switch_to_ckf: 9.21,
            nis_switch_to_ekf: 5.99,
            switch_back_count: 5,
        }
    }
}

/// Adaptive filter that switches between EKF and CKF.
pub struct AdaptiveFilterLocalizer {
    ekf: EKFLocalizer,
    ckf: CKFLocalizer,
    active: ActiveFilter,
    config: AdaptiveFilterConfig,
    /// Counter for consecutive low-NIS steps (used for switch-back to EKF)
    low_nis_count: usize,
    /// Last computed NIS value
    last_nis: f64,
    /// Total switch count
    switch_count: usize,
}

impl AdaptiveFilterLocalizer {
    pub fn new(config: AdaptiveFilterConfig) -> Self {
        let ekf = EKFLocalizer::new(config.ekf_config.clone());
        let ckf = CKFLocalizer::new(config.ckf_config.clone());
        Self {
            ekf,
            ckf,
            active: ActiveFilter::EKF,
            config,
            low_nis_count: 0,
            last_nis: 0.0,
            switch_count: 0,
        }
    }

    pub fn with_defaults() -> Self {
        Self::new(AdaptiveFilterConfig::default())
    }

    /// Run one predict-update cycle and return the state estimate.
    ///
    /// Internally computes the innovation and NIS to decide whether
    /// to switch filters.
    pub fn step(
        &mut self,
        control: &Vector2<f64>,
        measurement: &Vector2<f64>,
        dt: f64,
    ) -> RoboticsResult<State2D> {
        // Run both filters in parallel to keep states synchronized
        let _ = self.ekf.estimate(measurement, control, dt);
        self.ckf.step(control, measurement, dt);

        // Compute NIS from the active filter's innovation
        let nis = self.compute_nis(measurement);
        self.last_nis = nis;

        // Switching logic
        match self.active {
            ActiveFilter::EKF => {
                if nis > self.config.nis_switch_to_ckf {
                    self.active = ActiveFilter::CKF;
                    self.switch_count += 1;
                    self.low_nis_count = 0;
                }
            }
            ActiveFilter::CKF => {
                if nis < self.config.nis_switch_to_ekf {
                    self.low_nis_count += 1;
                    if self.low_nis_count >= self.config.switch_back_count {
                        self.active = ActiveFilter::EKF;
                        self.switch_count += 1;
                        self.low_nis_count = 0;
                    }
                } else {
                    self.low_nis_count = 0;
                }
            }
        }

        Ok(self.state_2d())
    }

    /// Get current state from the active filter.
    pub fn estimate(&self) -> Vector4<f64> {
        match self.active {
            ActiveFilter::EKF => {
                let s = self.ekf.state_2d();
                Vector4::new(s.x, s.y, s.yaw, s.v)
            }
            ActiveFilter::CKF => self.ckf.estimate(),
        }
    }

    /// Get current state as State2D.
    pub fn state_2d(&self) -> State2D {
        match self.active {
            ActiveFilter::EKF => self.ekf.state_2d(),
            ActiveFilter::CKF => self.ckf.state_2d(),
        }
    }

    /// Get which filter is currently active.
    pub fn active_filter(&self) -> ActiveFilter {
        self.active
    }

    /// Get the last computed NIS value.
    pub fn last_nis(&self) -> f64 {
        self.last_nis
    }

    /// Get total number of filter switches.
    pub fn switch_count(&self) -> usize {
        self.switch_count
    }

    /// Get the covariance from the active filter.
    pub fn covariance(&self) -> Matrix4<f64> {
        match self.active {
            ActiveFilter::EKF => *self.ekf.get_covariance_matrix(),
            ActiveFilter::CKF => *self.ckf.covariance(),
        }
    }

    /// Compute Normalized Innovation Squared (NIS).
    ///
    /// NIS = (z - z_pred)^T * S^{-1} * (z - z_pred)
    /// where z_pred = H * x_hat (observation model: observe x, y)
    /// and S = H * P * H^T + R
    fn compute_nis(&self, measurement: &Vector2<f64>) -> f64 {
        let state = self.estimate();
        let z_pred = Vector2::new(state[0], state[1]);
        let innovation = measurement - z_pred;

        let p = self.covariance();
        // H = [1 0 0 0; 0 1 0 0] — observe x, y
        let h_p = Matrix2::new(p[(0, 0)], p[(0, 1)], p[(1, 0)], p[(1, 1)]);

        let r = match self.active {
            ActiveFilter::EKF => self.config.ekf_config.r,
            ActiveFilter::CKF => self.config.ckf_config.r,
        };
        let s = h_p + r;

        match s.try_inverse() {
            Some(s_inv) => (innovation.transpose() * s_inv * innovation)[(0, 0)],
            None => f64::MAX, // Degenerate covariance → force switch to CKF
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand_distr::{Distribution, Normal};

    fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>, dt: f64) -> Vector4<f64> {
        let yaw = x[2];
        Vector4::new(
            x[0] + dt * u[0] * yaw.cos(),
            x[1] + dt * u[0] * yaw.sin(),
            x[2] + dt * u[1],
            u[0],
        )
    }

    #[test]
    fn test_adaptive_filter_basic() {
        let mut af = AdaptiveFilterLocalizer::with_defaults();
        let dt = 0.1;
        let u = Vector2::new(1.0, 0.1);
        let z = Vector2::new(0.1, 0.1);

        let state = af.step(&u, &z, dt).unwrap();
        assert!(state.x.is_finite());
        assert!(state.y.is_finite());
    }

    #[test]
    fn test_adaptive_filter_starts_with_ekf() {
        let af = AdaptiveFilterLocalizer::with_defaults();
        assert_eq!(af.active_filter(), ActiveFilter::EKF);
    }

    #[test]
    fn test_adaptive_filter_tracks_circular_motion() {
        let mut af = AdaptiveFilterLocalizer::with_defaults();
        let dt = 0.1;
        let u_true = Vector2::new(1.0, 0.1);
        let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);

        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let noise_obs = Normal::new(0.0, 0.5).unwrap();

        for _ in 0..100 {
            x_true = motion_model(&x_true, &u_true, dt);
            let z = Vector2::new(
                x_true[0] + noise_obs.sample(&mut rng),
                x_true[1] + noise_obs.sample(&mut rng),
            );
            let _ = af.step(&u_true, &z, dt);
        }

        let state = af.state_2d();
        let err = ((state.x - x_true[0]).powi(2) + (state.y - x_true[1]).powi(2)).sqrt();
        assert!(
            err < 2.0,
            "adaptive filter should track circular motion, error = {:.4}",
            err
        );
    }

    #[test]
    fn test_adaptive_filter_switches_on_high_noise() {
        let config = AdaptiveFilterConfig {
            nis_switch_to_ckf: 3.0, // Lower threshold → easier to trigger
            nis_switch_to_ekf: 2.0,
            switch_back_count: 3,
            ..Default::default()
        };
        let mut af = AdaptiveFilterLocalizer::new(config);
        let dt = 0.1;
        let u = Vector2::new(1.0, 0.5); // Tight turn → high nonlinearity

        let mut rng = rand::rngs::StdRng::seed_from_u64(99);
        let noise_high = Normal::new(0.0, 3.0).unwrap();
        let mut x_true = Vector4::new(0.0, 0.0, 0.0, 0.0);

        for _ in 0..50 {
            x_true = motion_model(&x_true, &u, dt);
            let z = Vector2::new(
                x_true[0] + noise_high.sample(&mut rng),
                x_true[1] + noise_high.sample(&mut rng),
            );
            let _ = af.step(&u, &z, dt);
        }

        // Should have switched at least once
        assert!(
            af.switch_count() > 0,
            "adaptive filter should switch under high noise, switches = {}",
            af.switch_count()
        );
    }
}
