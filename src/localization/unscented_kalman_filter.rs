//! Unscented Kalman Filter (UKF) localization
//!
//! Implements state estimation using the Unscented Transform for nonlinear systems.
//! UKF propagates sigma points through nonlinear functions for more accurate estimation
//! compared to EKF linearization.

use nalgebra::{Matrix4, Matrix2, Vector4, Vector2, DMatrix, DVector};
use rand::Rng;
use std::f64::consts::PI;

use crate::common::{StateEstimator, Point2D};

/// State representation for UKF (x, y, yaw, velocity)
pub type UKFState = Vector4<f64>;

/// Control input (velocity, yaw rate)
pub type UKFControl = Vector2<f64>;

/// Measurement (x, y position)
pub type UKFMeasurement = Vector2<f64>;

/// UKF scaling parameters
#[derive(Debug, Clone)]
pub struct UKFParams {
    /// Alpha: spread of sigma points (default: 0.001)
    pub alpha: f64,
    /// Beta: prior knowledge (Gaussian = 2.0)
    pub beta: f64,
    /// Kappa: secondary scaling (default: 0.0)
    pub kappa: f64,
}

impl Default for UKFParams {
    fn default() -> Self {
        Self {
            alpha: 0.001,
            beta: 2.0,
            kappa: 0.0,
        }
    }
}

/// Configuration for UKF localizer
#[derive(Debug, Clone)]
pub struct UKFConfig {
    /// UKF scaling parameters
    pub params: UKFParams,
    /// Process noise (x, y, yaw, v)
    pub process_noise: Vector4<f64>,
    /// Observation noise (x, y)
    pub observation_noise: Vector2<f64>,
    /// Time step
    pub dt: f64,
}

impl Default for UKFConfig {
    fn default() -> Self {
        Self {
            params: UKFParams::default(),
            process_noise: Vector4::new(
                0.1_f64.powi(2),           // x variance
                0.1_f64.powi(2),           // y variance
                1.0_f64.to_radians().powi(2), // yaw variance
                1.0_f64.powi(2),           // velocity variance
            ),
            observation_noise: Vector2::new(
                1.0_f64.powi(2), // x position variance
                1.0_f64.powi(2), // y position variance
            ),
            dt: 0.1,
        }
    }
}

/// UKF weights for mean and covariance computation
#[derive(Debug, Clone)]
pub struct UKFWeights {
    /// Mean weights
    pub wm: DVector<f64>,
    /// Covariance weights
    pub wc: DVector<f64>,
    /// Scaling parameter (gamma)
    pub gamma: f64,
}

impl UKFWeights {
    /// Create weights for given state dimension and parameters
    pub fn new(nx: usize, params: &UKFParams) -> Self {
        let lambda = params.alpha.powi(2) * (nx as f64 + params.kappa) - nx as f64;
        let n_sigma = 2 * nx + 1;

        let mut wm = Vec::with_capacity(n_sigma);
        let mut wc = Vec::with_capacity(n_sigma);

        // First weight (mean point)
        wm.push(lambda / (lambda + nx as f64));
        wc.push((lambda / (lambda + nx as f64)) + (1.0 - params.alpha.powi(2) + params.beta));

        // Remaining weights (sigma points)
        let weight = 1.0 / (2.0 * (nx as f64 + lambda));
        for _ in 0..(2 * nx) {
            wm.push(weight);
            wc.push(weight);
        }

        let gamma = (nx as f64 + lambda).sqrt();

        Self {
            wm: DVector::from_vec(wm),
            wc: DVector::from_vec(wc),
            gamma,
        }
    }
}

/// Unscented Kalman Filter for robot localization
pub struct UKFLocalizer {
    /// Current state estimate [x, y, yaw, v]
    x: UKFState,
    /// State covariance matrix
    p: Matrix4<f64>,
    /// Process noise covariance
    q: Matrix4<f64>,
    /// Observation noise covariance
    r: Matrix2<f64>,
    /// UKF weights
    weights: UKFWeights,
    /// Configuration
    config: UKFConfig,
    /// Last control input
    last_control: UKFControl,
}

impl UKFLocalizer {
    /// Create a new UKF localizer
    pub fn new(config: UKFConfig) -> Self {
        let weights = UKFWeights::new(4, &config.params);

        let q = Matrix4::from_diagonal(&config.process_noise);
        let r = Matrix2::from_diagonal(&config.observation_noise);

        Self {
            x: UKFState::zeros(),
            p: Matrix4::identity(),
            q,
            r,
            weights,
            config,
            last_control: UKFControl::zeros(),
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(UKFConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: UKFState, config: UKFConfig) -> Self {
        let mut ukf = Self::new(config);
        ukf.x = initial_state;
        ukf
    }

    /// Get current state estimate
    pub fn estimate(&self) -> UKFState {
        self.x
    }

    /// Get state covariance
    pub fn covariance(&self) -> &Matrix4<f64> {
        &self.p
    }

    /// Motion model: state transition
    fn motion_model(&self, x: &UKFState, u: &UKFControl) -> UKFState {
        let dt = self.config.dt;
        Vector4::new(
            x[0] + dt * x[3] * x[2].cos(),  // x += dt * v * cos(yaw)
            x[1] + dt * x[3] * x[2].sin(),  // y += dt * v * sin(yaw)
            x[2] + dt * u[1],                // yaw += dt * yaw_rate
            u[0],                            // v = input_v
        )
    }

    /// Observation model: state to measurement
    fn observation_model(&self, x: &UKFState) -> UKFMeasurement {
        Vector2::new(x[0], x[1])
    }

    /// Generate sigma points
    fn generate_sigma_points(&self, x: &UKFState, p: &Matrix4<f64>) -> DMatrix<f64> {
        let nx = 4;
        let n_sigma = 2 * nx + 1;
        let gamma = self.weights.gamma;

        let mut sigma = DMatrix::zeros(nx, n_sigma);

        // Mean point
        sigma.set_column(0, x);

        // Matrix square root using Cholesky decomposition
        if let Some(chol) = p.cholesky() {
            let sqrt_p = chol.l();

            // Positive direction
            for i in 0..nx {
                let col = x + gamma * sqrt_p.column(i);
                sigma.set_column(i + 1, &col);
            }

            // Negative direction
            for i in 0..nx {
                let col = x - gamma * sqrt_p.column(i);
                sigma.set_column(i + 1 + nx, &col);
            }
        } else {
            // Fallback if Cholesky fails
            for i in 0..nx {
                let mut col = *x;
                let std_dev = (gamma * p[(i, i)].sqrt()).max(1e-6);
                col[i] += std_dev;
                sigma.set_column(i + 1, &col);

                col = *x;
                col[i] -= std_dev;
                sigma.set_column(i + 1 + nx, &col);
            }
        }

        sigma
    }

    /// Predict sigma points through motion model
    fn predict_sigma_motion(&self, sigma: &DMatrix<f64>, u: &UKFControl) -> DMatrix<f64> {
        let mut sigma_pred = DMatrix::zeros(sigma.nrows(), sigma.ncols());

        for i in 0..sigma.ncols() {
            let x_sigma = Vector4::new(
                sigma[(0, i)], sigma[(1, i)], sigma[(2, i)], sigma[(3, i)]
            );
            let x_pred = self.motion_model(&x_sigma, u);

            for j in 0..4 {
                sigma_pred[(j, i)] = x_pred[j];
            }
        }

        sigma_pred
    }

    /// Predict sigma points through observation model
    fn predict_sigma_observation(&self, sigma: &DMatrix<f64>) -> DMatrix<f64> {
        let mut z_sigma = DMatrix::zeros(2, sigma.ncols());

        for i in 0..sigma.ncols() {
            let x_sigma = Vector4::new(
                sigma[(0, i)], sigma[(1, i)], sigma[(2, i)], sigma[(3, i)]
            );
            let z_pred = self.observation_model(&x_sigma);

            z_sigma[(0, i)] = z_pred[0];
            z_sigma[(1, i)] = z_pred[1];
        }

        z_sigma
    }

    /// Calculate weighted mean from sigma points
    fn calc_weighted_mean(&self, sigma: &DMatrix<f64>) -> DVector<f64> {
        let mut mean = DVector::zeros(sigma.nrows());
        for i in 0..sigma.ncols() {
            mean += self.weights.wm[i] * sigma.column(i);
        }
        mean
    }

    /// Calculate covariance from sigma points
    fn calc_covariance(
        &self,
        mean: &DVector<f64>,
        sigma: &DMatrix<f64>,
        noise: &DMatrix<f64>,
    ) -> DMatrix<f64> {
        let mut cov = noise.clone();

        for i in 0..sigma.ncols() {
            let diff = sigma.column(i) - mean;
            cov += self.weights.wc[i] * &diff * diff.transpose();
        }

        cov
    }

    /// Calculate cross-covariance
    fn calc_cross_covariance(
        &self,
        x_sigma: &DMatrix<f64>,
        x_mean: &DVector<f64>,
        z_sigma: &DMatrix<f64>,
        z_mean: &DVector<f64>,
    ) -> DMatrix<f64> {
        let nx = x_mean.len();
        let nz = z_mean.len();
        let mut pxz = DMatrix::zeros(nx, nz);

        for i in 0..x_sigma.ncols() {
            let dx = x_sigma.column(i) - x_mean;
            let dz = z_sigma.column(i) - z_mean;
            pxz += self.weights.wc[i] * &dx * dz.transpose();
        }

        pxz
    }

    /// Prediction step with control input
    pub fn predict_with_control(&mut self, control: &UKFControl) {
        // Generate sigma points
        let sigma = self.generate_sigma_points(&self.x, &self.p);

        // Propagate sigma points through motion model
        let sigma_pred = self.predict_sigma_motion(&sigma, control);

        // Calculate predicted mean
        let x_pred = self.calc_weighted_mean(&sigma_pred);

        // Calculate predicted covariance
        let q_dyn = DMatrix::from_fn(4, 4, |i, j| self.q[(i, j)]);
        let p_pred = self.calc_covariance(&x_pred, &sigma_pred, &q_dyn);

        // Update state
        self.x = Vector4::new(x_pred[0], x_pred[1], x_pred[2], x_pred[3]);
        self.p = Matrix4::from_fn(|i, j| p_pred[(i, j)]);
        self.last_control = *control;
    }

    /// Update step with measurement
    pub fn update_with_measurement(&mut self, z: &UKFMeasurement) {
        // Generate sigma points from current state
        let sigma = self.generate_sigma_points(&self.x, &self.p);

        // Predict observations from sigma points
        let z_sigma = self.predict_sigma_observation(&sigma);

        // Calculate predicted observation mean
        let z_pred = self.calc_weighted_mean(&z_sigma);

        // Calculate innovation covariance
        let r_dyn = DMatrix::from_fn(2, 2, |i, j| self.r[(i, j)]);
        let s = self.calc_covariance(&z_pred, &z_sigma, &r_dyn);

        // Convert sigma to DMatrix for cross-covariance
        let x_mean = DVector::from_vec(vec![self.x[0], self.x[1], self.x[2], self.x[3]]);
        let sigma_dyn = DMatrix::from_fn(4, sigma.ncols(), |i, j| sigma[(i, j)]);

        // Calculate cross-covariance
        let pxz = self.calc_cross_covariance(&sigma_dyn, &x_mean, &z_sigma, &z_pred);

        // Kalman gain
        let s_inv = s.clone().try_inverse().unwrap_or_else(|| DMatrix::identity(2, 2));
        let k = &pxz * &s_inv;

        // Innovation
        let innovation = DVector::from_vec(vec![z[0] - z_pred[0], z[1] - z_pred[1]]);

        // Update state
        let x_update = &x_mean + &k * &innovation;
        self.x = Vector4::new(x_update[0], x_update[1], x_update[2], x_update[3]);

        // Update covariance
        let p_dyn = DMatrix::from_fn(4, 4, |i, j| self.p[(i, j)]);
        let p_update = &p_dyn - &k * &s * k.transpose();
        self.p = Matrix4::from_fn(|i, j| p_update[(i, j)]);
    }

    /// Full estimation step (predict + update)
    pub fn step(&mut self, control: &UKFControl, measurement: &UKFMeasurement) -> UKFState {
        self.predict_with_control(control);
        self.update_with_measurement(measurement);
        self.x
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

        let a = 2.0 * eigen.eigenvalues[big_idx].sqrt();
        let b = 2.0 * eigen.eigenvalues[small_idx].sqrt();
        let angle = eigen.eigenvectors[(1, big_idx)].atan2(eigen.eigenvectors[(0, big_idx)]);

        (a, b, angle)
    }
}

impl StateEstimator for UKFLocalizer {
    type State = UKFState;
    type Measurement = UKFMeasurement;
    type Control = UKFControl;

    fn predict(&mut self, control: &Self::Control, _dt: f64) {
        self.predict_with_control(control);
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        self.update_with_measurement(measurement);
    }

    fn get_state(&self) -> &Self::State {
        &self.x
    }

    fn get_covariance(&self) -> Option<&nalgebra::DMatrix<f64>> {
        None // Use covariance() method for fixed-size covariance
    }
}

// ============================================================================
// Legacy interface for backward compatibility
// ============================================================================

/// Legacy UKF weights structure (deprecated, use UKFWeights)
#[derive(Debug, Clone)]
#[deprecated(note = "Use UKFWeights instead")]
pub struct LegacyUKFWeights {
    pub wm: DVector<f64>,
    pub wc: DVector<f64>,
    pub gamma: f64,
}

/// Legacy UKF state structure (deprecated, use UKFLocalizer)
#[derive(Debug, Clone)]
#[deprecated(note = "Use UKFLocalizer instead")]
pub struct LegacyUKFState {
    pub x: Vector4<f64>,
    pub p: Matrix4<f64>,
    pub q: Matrix4<f64>,
    pub r: Matrix2<f64>,
}

#[allow(deprecated)]
impl LegacyUKFState {
    pub fn new() -> Self {
        let x = Vector4::zeros();
        let p = Matrix4::identity();

        let q = Matrix4::from_diagonal(&Vector4::new(
            0.1_f64.powi(2),
            0.1_f64.powi(2),
            1.0_f64.to_radians().powi(2),
            1.0_f64.powi(2),
        ));

        let r = Matrix2::from_diagonal(&Vector2::new(1.0, 1.0));

        Self { x, p, q, r }
    }
}

/// Legacy motion model function
pub fn motion_model(x: &Vector4<f64>, u: &Vector2<f64>) -> Vector4<f64> {
    const DT: f64 = 0.1;
    Vector4::new(
        x[0] + DT * x[3] * x[2].cos(),
        x[1] + DT * x[3] * x[2].sin(),
        x[2] + DT * u[1],
        u[0],
    )
}

/// Legacy observation model function
pub fn observation_model(x: &Vector4<f64>) -> Vector2<f64> {
    Vector2::new(x[0], x[1])
}

/// Legacy setup UKF function
pub fn setup_ukf(nx: usize) -> UKFWeights {
    UKFWeights::new(nx, &UKFParams::default())
}

/// Legacy UKF estimation function
#[allow(deprecated)]
pub fn ukf_estimation(
    state: &mut LegacyUKFState,
    z: &Vector2<f64>,
    u: &Vector2<f64>,
    weights: &UKFWeights,
) {
    let config = UKFConfig::default();
    let mut ukf = UKFLocalizer::new(config);
    ukf.x = state.x;
    ukf.p = state.p;
    ukf.q = state.q;
    ukf.r = state.r;

    // Use custom weights
    let custom_weights = UKFWeights {
        wm: weights.wm.clone(),
        wc: weights.wc.clone(),
        gamma: weights.gamma,
    };
    ukf.weights = custom_weights;

    ukf.step(u, z);

    state.x = ukf.x;
    state.p = ukf.p;
}

/// Generate control input (for demo)
pub fn calc_input() -> Vector2<f64> {
    Vector2::new(1.0, 0.1)
}

/// Simulate observation with noise (for demo)
pub fn observation(
    x_true: &mut Vector4<f64>,
    x_dr: &mut Vector4<f64>,
    u: &Vector2<f64>,
) -> (Vector2<f64>, Vector2<f64>) {
    const INPUT_NOISE_V: f64 = 1.0;
    const INPUT_NOISE_YAW: f64 = 0.5236; // 30 degrees
    const GPS_NOISE_X: f64 = 0.5;
    const GPS_NOISE_Y: f64 = 0.5;

    let mut rng = rand::thread_rng();

    *x_true = motion_model(x_true, u);

    let z = observation_model(x_true) + Vector2::new(
        GPS_NOISE_X * rng.gen::<f64>() - GPS_NOISE_X / 2.0,
        GPS_NOISE_Y * rng.gen::<f64>() - GPS_NOISE_Y / 2.0,
    );

    let u_noisy = u + Vector2::new(
        INPUT_NOISE_V * rng.gen::<f64>() - INPUT_NOISE_V / 2.0,
        INPUT_NOISE_YAW * rng.gen::<f64>() - INPUT_NOISE_YAW / 2.0,
    );

    *x_dr = motion_model(x_dr, &u_noisy);

    (z, u_noisy)
}

/// Plot covariance ellipse points
pub fn plot_covariance_ellipse(
    x_est: &Vector4<f64>,
    p_est: &Matrix4<f64>,
    x_data: &mut Vec<f64>,
    y_data: &mut Vec<f64>,
) {
    let p_xy = p_est.fixed_view::<2, 2>(0, 0);
    let eigen = p_xy.symmetric_eigen();

    let (big_idx, small_idx) = if eigen.eigenvalues[0] >= eigen.eigenvalues[1] {
        (0, 1)
    } else {
        (1, 0)
    };

    let a = eigen.eigenvalues[big_idx].sqrt();
    let b = eigen.eigenvalues[small_idx].sqrt();
    let angle = eigen.eigenvectors[(1, big_idx)].atan2(eigen.eigenvectors[(0, big_idx)]);

    let n_points = 50;
    x_data.clear();
    y_data.clear();

    for i in 0..=n_points {
        let t = 2.0 * PI * i as f64 / n_points as f64;
        let x_local = a * t.cos();
        let y_local = b * t.sin();

        let x_global = x_local * angle.cos() - y_local * angle.sin() + x_est[0];
        let y_global = x_local * angle.sin() + y_local * angle.cos() + x_est[1];

        x_data.push(x_global);
        y_data.push(y_global);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ukf_creation() {
        let ukf = UKFLocalizer::with_defaults();
        assert_eq!(ukf.x, UKFState::zeros());
    }

    #[test]
    fn test_ukf_with_initial_state() {
        let initial = UKFState::new(1.0, 2.0, 0.5, 1.0);
        let ukf = UKFLocalizer::with_initial_state(initial, UKFConfig::default());

        assert_eq!(ukf.estimate()[0], 1.0);
        assert_eq!(ukf.estimate()[1], 2.0);
    }

    #[test]
    fn test_ukf_weights() {
        let weights = UKFWeights::new(4, &UKFParams::default());

        // Check that mean weights sum to approximately 1
        let wm_sum: f64 = weights.wm.iter().sum();
        assert!((wm_sum - 1.0).abs() < 1e-6);

        // Should have 2*n + 1 = 9 weights
        assert_eq!(weights.wm.len(), 9);
        assert_eq!(weights.wc.len(), 9);
    }

    #[test]
    fn test_ukf_sigma_points() {
        let ukf = UKFLocalizer::with_defaults();
        let sigma = ukf.generate_sigma_points(&ukf.x, &ukf.p);

        // Should have 2*4 + 1 = 9 sigma points
        assert_eq!(sigma.ncols(), 9);
        assert_eq!(sigma.nrows(), 4);
    }

    #[test]
    fn test_ukf_predict() {
        let mut ukf = UKFLocalizer::with_defaults();
        let control = UKFControl::new(1.0, 0.0);

        ukf.predict_with_control(&control);

        // State should change after prediction
        // With v=1.0, yaw=0, dt=0.1: x should increase by approximately 0.1
        assert!(ukf.x[0].abs() < 0.2); // Within reasonable range
    }

    #[test]
    fn test_ukf_update() {
        let initial = UKFState::new(5.0, 5.0, 0.0, 1.0);
        let mut ukf = UKFLocalizer::with_initial_state(initial, UKFConfig::default());

        // Measurement at true position
        let z = UKFMeasurement::new(5.0, 5.0);
        ukf.update_with_measurement(&z);

        // State should stay close to initial
        assert!((ukf.x[0] - 5.0).abs() < 1.0);
        assert!((ukf.x[1] - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_ukf_step() {
        let mut ukf = UKFLocalizer::with_defaults();
        let control = UKFControl::new(1.0, 0.1);
        let measurement = UKFMeasurement::new(0.1, 0.01);

        let estimate = ukf.step(&control, &measurement);

        assert!(estimate[0].is_finite());
        assert!(estimate[1].is_finite());
        assert!(estimate[2].is_finite());
        assert!(estimate[3].is_finite());
    }

    #[test]
    fn test_ukf_position_uncertainty() {
        let ukf = UKFLocalizer::with_defaults();
        let (a, b, angle) = ukf.position_uncertainty();

        assert!(a >= 0.0);
        assert!(b >= 0.0);
        assert!(a >= b); // Major axis >= minor axis
        assert!(angle.is_finite());
    }

    #[test]
    fn test_state_estimator_trait() {
        let mut ukf = UKFLocalizer::with_defaults();
        let control = UKFControl::new(1.0, 0.1);
        let measurement = UKFMeasurement::new(0.1, 0.01);

        // Test trait methods
        ukf.predict(&control, 0.1);
        ukf.update(&measurement);

        let state = ukf.get_state();
        assert!(state[0].is_finite());
    }

    #[test]
    fn test_motion_model() {
        let x = Vector4::new(0.0, 0.0, 0.0, 1.0);
        let u = Vector2::new(1.0, 0.1);
        let x_next = motion_model(&x, &u);

        // x should increase (moving forward with yaw=0)
        assert!(x_next[0] > 0.0);
        // Velocity should be input velocity
        assert_eq!(x_next[3], 1.0);
    }

    #[test]
    fn test_observation_model() {
        let x = Vector4::new(1.0, 2.0, 0.5, 1.0);
        let z = observation_model(&x);

        assert_eq!(z[0], 1.0);
        assert_eq!(z[1], 2.0);
    }
}
