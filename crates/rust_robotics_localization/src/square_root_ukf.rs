//! Square Root Unscented Kalman Filter (SR-UKF) localization
//!
//! SR-UKF propagates the Cholesky factor of covariance for improved numerical stability.

use nalgebra::{DMatrix, DVector, Matrix2, Matrix4, Vector2, Vector4};
use rust_robotics_core::{RoboticsError, RoboticsResult, State2D, StateEstimator};

/// State representation for SR-UKF (x, y, yaw, velocity)
pub type SRUKFState = Vector4<f64>;
/// Measurement representation (x, y)
pub type SRUKFMeasurement = Vector2<f64>;
/// Control input (velocity, yaw rate)
pub type SRUKFControl = Vector2<f64>;

const STATE_DIM: usize = 4;
const MEAS_DIM: usize = 2;
const SIGMA_COUNT: usize = 2 * STATE_DIM + 1;
const NUMERICAL_EPS: f64 = 1e-12;

/// Configuration for SR-UKF localizer.
#[derive(Debug, Clone)]
pub struct SRUKFConfig {
    /// Process noise covariance.
    pub q: Matrix4<f64>,
    /// Measurement noise covariance.
    pub r: Matrix2<f64>,
    /// Sigma point spread parameter.
    pub alpha: f64,
    /// Prior distribution parameter.
    pub beta: f64,
    /// Secondary scaling parameter.
    pub kappa: f64,
}

impl Default for SRUKFConfig {
    fn default() -> Self {
        let mut q = Matrix4::<f64>::identity();
        q[(0, 0)] = 0.1_f64.powi(2);
        q[(1, 1)] = (1.0_f64.to_radians()).powi(2);
        q[(2, 2)] = 0.1_f64.powi(2);
        q[(3, 3)] = 0.1_f64.powi(2);

        Self {
            q,
            r: Matrix2::identity(),
            alpha: 1e-3,
            beta: 2.0,
            kappa: 0.0,
        }
    }
}

impl SRUKFConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.q.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF process noise matrix must contain only finite values".to_string(),
            ));
        }
        if self.r.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..STATE_DIM {
            if self.q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "SR-UKF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        for i in 0..MEAS_DIM {
            if self.r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "SR-UKF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        if !self.alpha.is_finite() || self.alpha <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF alpha must be positive and finite".to_string(),
            ));
        }
        if !self.beta.is_finite() || self.beta < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF beta must be non-negative and finite".to_string(),
            ));
        }
        if !self.kappa.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF kappa must be finite".to_string(),
            ));
        }

        let lambda = self.alpha.powi(2) * (STATE_DIM as f64 + self.kappa) - STATE_DIM as f64;
        let scale = lambda + STATE_DIM as f64;
        if !scale.is_finite() || scale <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF scaling parameters produce a non-positive sigma spread".to_string(),
            ));
        }

        Ok(())
    }
}

/// Square Root Unscented Kalman Filter localizer.
pub struct SRUKFLocalizer {
    /// Current state estimate [x, y, yaw, v].
    state: SRUKFState,
    /// Lower-triangular Cholesky factor of covariance.
    sqrt_covariance: Matrix4<f64>,
    /// Configuration.
    config: SRUKFConfig,
    wm: DVector<f64>,
    wc: DVector<f64>,
    gamma: f64,
    covariance_dyn: DMatrix<f64>,
}

impl SRUKFLocalizer {
    /// Create a new SR-UKF localizer.
    pub fn new(config: SRUKFConfig) -> Self {
        Self::try_new(config).expect(
            "invalid SR-UKF configuration: noise matrices and sigma point parameters must be valid",
        )
    }

    /// Create a validated SR-UKF localizer.
    pub fn try_new(config: SRUKFConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let (wm, wc, gamma) = Self::compute_weights(&config)?;
        let sqrt_covariance = Matrix4::identity();
        let covariance_dyn = DMatrix::identity(STATE_DIM, STATE_DIM);

        Ok(Self {
            state: SRUKFState::zeros(),
            sqrt_covariance,
            config,
            wm,
            wc,
            gamma,
            covariance_dyn,
        })
    }

    /// Create with an initial state.
    pub fn with_initial_state(state: SRUKFState, config: SRUKFConfig) -> Self {
        Self::try_with_initial_state(state, config)
            .expect("invalid SR-UKF initialization: state must be finite and config must be valid")
    }

    fn try_with_initial_state(state: SRUKFState, config: SRUKFConfig) -> RoboticsResult<Self> {
        Self::validate_state(&state)?;
        let mut localizer = Self::try_new(config)?;
        localizer.state = state;
        localizer.refresh_covariance_cache();
        Ok(localizer)
    }

    /// Return the current estimate as [`State2D`].
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.state[0], self.state[1], self.state[2], self.state[3])
    }

    /// Motion model (copied from EKF): f(x, u, dt).
    fn motion_model(x: &SRUKFState, u: &SRUKFControl, dt: f64) -> SRUKFState {
        let yaw = x[2];
        let f = Matrix4::new(
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        );
        let b = nalgebra::Matrix4x2::new(dt * yaw.cos(), 0., dt * yaw.sin(), 0., 0., dt, 1., 0.);
        f * x + b * u
    }

    /// Observation model (copied from EKF): h(x).
    fn observation_model(x: &SRUKFState) -> SRUKFMeasurement {
        let h = nalgebra::Matrix2x4::new(1., 0., 0., 0., 0., 1., 0., 0.);
        h * x
    }

    fn try_predict(&mut self, control: &SRUKFControl, dt: f64) -> RoboticsResult<()> {
        Self::validate_control(control)?;
        Self::validate_dt(dt)?;

        let sigma = self.generate_sigma_points();
        let sigma_pred = std::array::from_fn(|i| Self::motion_model(&sigma[i], control, dt));
        let x_pred = self.weighted_state_mean(&sigma_pred);

        let sqrt_q = Self::cholesky_lower_4(&self.config.q)?;
        let mut cols = DMatrix::zeros(STATE_DIM, 2 * STATE_DIM + STATE_DIM);
        for (i, sigma_point) in sigma_pred.iter().enumerate().skip(1) {
            let dx = *sigma_point - x_pred;
            let scaled = dx * self.wc[i].sqrt();
            cols.set_column(i - 1, &scaled);
        }
        for j in 0..STATE_DIM {
            cols.set_column(2 * STATE_DIM + j, &sqrt_q.column(j).into_owned());
        }

        let mut sqrt_pred = Self::qr_lower_root_4(&cols)?;
        let diff0 = sigma_pred[0] - x_pred;
        let w0 = self.wc[0];
        let scaled0 = diff0 * w0.abs().sqrt();
        sqrt_pred = Self::chol_rank1_4(sqrt_pred, scaled0, w0.signum())?;

        self.state = x_pred;
        self.sqrt_covariance = sqrt_pred;
        self.refresh_covariance_cache();
        Ok(())
    }

    fn try_update(&mut self, measurement: &SRUKFMeasurement) -> RoboticsResult<()> {
        Self::validate_measurement(measurement)?;

        let sigma = self.generate_sigma_points();
        let z_sigma = std::array::from_fn(|i| Self::observation_model(&sigma[i]));
        let z_pred = self.weighted_measurement_mean(&z_sigma);

        let sqrt_r = Self::cholesky_lower_2(&self.config.r)?;
        let mut cols = DMatrix::zeros(MEAS_DIM, 2 * STATE_DIM + MEAS_DIM);
        for (i, sigma_point) in z_sigma.iter().enumerate().skip(1) {
            let dz = *sigma_point - z_pred;
            let scaled = dz * self.wc[i].sqrt();
            cols.set_column(i - 1, &scaled);
        }
        for j in 0..MEAS_DIM {
            cols.set_column(2 * STATE_DIM + j, &sqrt_r.column(j).into_owned());
        }

        let mut sqrt_innovation = Self::qr_lower_root_2(&cols)?;
        let dz0 = z_sigma[0] - z_pred;
        let w0 = self.wc[0];
        let scaled0 = dz0 * w0.abs().sqrt();
        sqrt_innovation = Self::chol_rank1_2(sqrt_innovation, scaled0, w0.signum())?;

        let mut pxz = DMatrix::zeros(STATE_DIM, MEAS_DIM);
        for i in 0..SIGMA_COUNT {
            let dx = sigma[i] - self.state;
            let dz = z_sigma[i] - z_pred;
            pxz += self.wc[i] * dx * dz.transpose();
        }

        let s_cov = sqrt_innovation * sqrt_innovation.transpose();
        let s_inv = s_cov.try_inverse().ok_or_else(|| {
            RoboticsError::NumericalError(
                "Failed to invert SR-UKF innovation covariance".to_string(),
            )
        })?;
        let k = &pxz * s_inv;

        let innovation =
            DVector::from_vec(vec![measurement[0] - z_pred[0], measurement[1] - z_pred[1]]);
        let x_update = DVector::from_vec(vec![
            self.state[0],
            self.state[1],
            self.state[2],
            self.state[3],
        ]) + &k * innovation;
        self.state = SRUKFState::new(x_update[0], x_update[1], x_update[2], x_update[3]);

        let u = &k * sqrt_innovation;
        let mut sqrt_post = self.sqrt_covariance;
        for j in 0..MEAS_DIM {
            let downdate_col = Vector4::new(u[(0, j)], u[(1, j)], u[(2, j)], u[(3, j)]);
            sqrt_post = Self::chol_rank1_4(sqrt_post, downdate_col, -1.0)?;
        }
        self.sqrt_covariance = sqrt_post;
        self.refresh_covariance_cache();
        Ok(())
    }

    fn compute_weights(config: &SRUKFConfig) -> RoboticsResult<(DVector<f64>, DVector<f64>, f64)> {
        let lambda = config.alpha.powi(2) * (STATE_DIM as f64 + config.kappa) - STATE_DIM as f64;
        let scale = lambda + STATE_DIM as f64;
        if !scale.is_finite() || scale <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF scaling parameters produce a non-positive sigma spread".to_string(),
            ));
        }

        let mut wm = vec![0.0; SIGMA_COUNT];
        let mut wc = vec![0.0; SIGMA_COUNT];
        wm[0] = lambda / scale;
        wc[0] = wm[0] + (1.0 - config.alpha.powi(2) + config.beta);
        let tail_weight = 1.0 / (2.0 * scale);
        for i in 1..SIGMA_COUNT {
            wm[i] = tail_weight;
            wc[i] = tail_weight;
        }

        let gamma = scale.sqrt();
        if !gamma.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF gamma must be finite".to_string(),
            ));
        }

        Ok((DVector::from_vec(wm), DVector::from_vec(wc), gamma))
    }

    fn generate_sigma_points(&self) -> [SRUKFState; SIGMA_COUNT] {
        let mut sigma = [self.state; SIGMA_COUNT];
        sigma[0] = self.state;
        for i in 0..STATE_DIM {
            let offset = self.sqrt_covariance.column(i).into_owned() * self.gamma;
            sigma[i + 1] = self.state + offset;
            sigma[i + 1 + STATE_DIM] = self.state - offset;
        }
        sigma
    }

    fn weighted_state_mean(&self, sigma: &[SRUKFState; SIGMA_COUNT]) -> SRUKFState {
        let mut mean = SRUKFState::zeros();
        for (i, point) in sigma.iter().enumerate() {
            mean += *point * self.wm[i];
        }
        mean
    }

    fn weighted_measurement_mean(
        &self,
        sigma: &[SRUKFMeasurement; SIGMA_COUNT],
    ) -> SRUKFMeasurement {
        let mut mean = SRUKFMeasurement::zeros();
        for (i, point) in sigma.iter().enumerate() {
            mean += *point * self.wm[i];
        }
        mean
    }

    fn qr_lower_root_4(cols: &DMatrix<f64>) -> RoboticsResult<Matrix4<f64>> {
        let qr = cols.transpose().qr();
        let r = qr.r();
        if r.nrows() < STATE_DIM || r.ncols() < STATE_DIM {
            return Err(RoboticsError::NumericalError(
                "QR decomposition returned unexpected dimensions for state root".to_string(),
            ));
        }

        let mut lower = Matrix4::zeros();
        for i in 0..STATE_DIM {
            for j in 0..=i {
                lower[(i, j)] = r[(j, i)];
            }
        }
        Self::enforce_positive_diagonal_4(lower)
    }

    fn qr_lower_root_2(cols: &DMatrix<f64>) -> RoboticsResult<Matrix2<f64>> {
        let qr = cols.transpose().qr();
        let r = qr.r();
        if r.nrows() < MEAS_DIM || r.ncols() < MEAS_DIM {
            return Err(RoboticsError::NumericalError(
                "QR decomposition returned unexpected dimensions for innovation root".to_string(),
            ));
        }

        let mut lower = Matrix2::zeros();
        for i in 0..MEAS_DIM {
            for j in 0..=i {
                lower[(i, j)] = r[(j, i)];
            }
        }
        Self::enforce_positive_diagonal_2(lower)
    }

    fn enforce_positive_diagonal_4(mut lower: Matrix4<f64>) -> RoboticsResult<Matrix4<f64>> {
        for i in 0..STATE_DIM {
            if !lower[(i, i)].is_finite() || lower[(i, i)].abs() <= NUMERICAL_EPS {
                return Err(RoboticsError::NumericalError(
                    "State Cholesky factor has a near-zero diagonal".to_string(),
                ));
            }
            if lower[(i, i)] < 0.0 {
                for row in i..STATE_DIM {
                    lower[(row, i)] = -lower[(row, i)];
                }
            }
        }
        Ok(lower)
    }

    fn enforce_positive_diagonal_2(mut lower: Matrix2<f64>) -> RoboticsResult<Matrix2<f64>> {
        for i in 0..MEAS_DIM {
            if !lower[(i, i)].is_finite() || lower[(i, i)].abs() <= NUMERICAL_EPS {
                return Err(RoboticsError::NumericalError(
                    "Innovation Cholesky factor has a near-zero diagonal".to_string(),
                ));
            }
            if lower[(i, i)] < 0.0 {
                for row in i..MEAS_DIM {
                    lower[(row, i)] = -lower[(row, i)];
                }
            }
        }
        Ok(lower)
    }

    fn cholesky_lower_4(cov: &Matrix4<f64>) -> RoboticsResult<Matrix4<f64>> {
        if let Some(chol) = cov.cholesky() {
            return Ok(chol.l());
        }
        let jittered = *cov + Matrix4::identity() * NUMERICAL_EPS;
        jittered.cholesky().map(|chol| chol.l()).ok_or_else(|| {
            RoboticsError::NumericalError(
                "Failed to compute Cholesky factor for SR-UKF process covariance".to_string(),
            )
        })
    }

    fn cholesky_lower_2(cov: &Matrix2<f64>) -> RoboticsResult<Matrix2<f64>> {
        if let Some(chol) = cov.cholesky() {
            return Ok(chol.l());
        }
        let jittered = *cov + Matrix2::identity() * NUMERICAL_EPS;
        jittered.cholesky().map(|chol| chol.l()).ok_or_else(|| {
            RoboticsError::NumericalError(
                "Failed to compute Cholesky factor for SR-UKF measurement covariance".to_string(),
            )
        })
    }

    fn chol_rank1_4(
        mut lower: Matrix4<f64>,
        mut vector: Vector4<f64>,
        sign: f64,
    ) -> RoboticsResult<Matrix4<f64>> {
        if sign != 1.0 && sign != -1.0 {
            return Err(RoboticsError::InvalidParameter(
                "Cholesky rank-1 update sign must be +1 or -1".to_string(),
            ));
        }
        for k in 0..STATE_DIM {
            let lkk = lower[(k, k)];
            let xk = vector[k];
            let radicand = lkk * lkk + sign * xk * xk;
            if !radicand.is_finite() || radicand <= NUMERICAL_EPS {
                return Err(RoboticsError::NumericalError(
                    "Cholesky rank-1 update/downdate became non-positive definite".to_string(),
                ));
            }
            let r = radicand.sqrt();
            let c = r / lkk;
            let s = xk / lkk;
            lower[(k, k)] = r;

            for j in (k + 1)..STATE_DIM {
                let updated = (lower[(j, k)] + sign * s * vector[j]) / c;
                lower[(j, k)] = updated;
                vector[j] = c * vector[j] - s * updated;
            }
        }
        Ok(lower)
    }

    fn chol_rank1_2(
        mut lower: Matrix2<f64>,
        mut vector: Vector2<f64>,
        sign: f64,
    ) -> RoboticsResult<Matrix2<f64>> {
        if sign != 1.0 && sign != -1.0 {
            return Err(RoboticsError::InvalidParameter(
                "Cholesky rank-1 update sign must be +1 or -1".to_string(),
            ));
        }
        for k in 0..MEAS_DIM {
            let lkk = lower[(k, k)];
            let xk = vector[k];
            let radicand = lkk * lkk + sign * xk * xk;
            if !radicand.is_finite() || radicand <= NUMERICAL_EPS {
                return Err(RoboticsError::NumericalError(
                    "Innovation Cholesky rank-1 update/downdate became non-positive definite"
                        .to_string(),
                ));
            }
            let r = radicand.sqrt();
            let c = r / lkk;
            let s = xk / lkk;
            lower[(k, k)] = r;

            for j in (k + 1)..MEAS_DIM {
                let updated = (lower[(j, k)] + sign * s * vector[j]) / c;
                lower[(j, k)] = updated;
                vector[j] = c * vector[j] - s * updated;
            }
        }
        Ok(lower)
    }

    fn refresh_covariance_cache(&mut self) {
        let covariance = self.sqrt_covariance * self.sqrt_covariance.transpose();
        self.covariance_dyn = DMatrix::from_fn(STATE_DIM, STATE_DIM, |i, j| covariance[(i, j)]);
    }

    fn validate_state(state: &SRUKFState) -> RoboticsResult<()> {
        if state.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF state must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_control(control: &SRUKFControl) -> RoboticsResult<()> {
        if control.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF control input must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_measurement(measurement: &SRUKFMeasurement) -> RoboticsResult<()> {
        if measurement.iter().any(|value| !value.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF measurement must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_dt(dt: f64) -> RoboticsResult<()> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "SR-UKF dt must be positive and finite".to_string(),
            ));
        }
        Ok(())
    }
}

impl StateEstimator for SRUKFLocalizer {
    type State = SRUKFState;
    type Measurement = SRUKFMeasurement;
    type Control = SRUKFControl;

    fn predict(&mut self, control: &Self::Control, dt: f64) {
        let _ = self.try_predict(control, dt);
    }

    fn update(&mut self, measurement: &Self::Measurement) {
        let _ = self.try_update(measurement);
    }

    fn get_state(&self) -> &Self::State {
        &self.state
    }

    fn get_covariance(&self) -> Option<&DMatrix<f64>> {
        Some(&self.covariance_dyn)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_root_ukf_creation() {
        let localizer = SRUKFLocalizer::new(SRUKFConfig::default());
        assert_eq!(*localizer.get_state(), SRUKFState::zeros());
        assert!(localizer.sqrt_covariance[(0, 0)] > 0.0);
    }

    #[test]
    fn test_square_root_ukf_predict_moves_state() {
        let mut localizer = SRUKFLocalizer::new(SRUKFConfig::default());
        localizer.predict(&SRUKFControl::new(1.0, 0.0), 0.1);
        assert!(localizer.get_state()[0] > 0.0);
    }

    #[test]
    fn test_square_root_ukf_update_toward_measurement() {
        let mut localizer = SRUKFLocalizer::with_initial_state(
            SRUKFState::new(5.0, 5.0, 0.0, 1.0),
            SRUKFConfig::default(),
        );
        localizer.update(&SRUKFMeasurement::new(0.0, 0.0));
        assert!(localizer.get_state()[0] < 5.0);
        assert!(localizer.get_state()[1] < 5.0);
    }

    #[test]
    fn test_square_root_ukf_state_2d_helper() {
        let localizer = SRUKFLocalizer::with_initial_state(
            SRUKFState::new(1.0, 2.0, 0.3, 0.4),
            SRUKFConfig::default(),
        );
        let state = localizer.state_2d();
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.3);
        assert_eq!(state.v, 0.4);
    }

    #[test]
    fn test_square_root_ukf_try_new_rejects_invalid() {
        let config = SRUKFConfig {
            alpha: 0.0,
            ..Default::default()
        };
        let err = match SRUKFLocalizer::try_new(config) {
            Ok(_) => panic!("expected invalid configuration to fail"),
            Err(err) => err,
        };
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }
}
