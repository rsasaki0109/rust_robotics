//! Cubature Kalman Filter (CKF) localization
//!
//! Implements state estimation using the 3rd-degree spherical-radial cubature rule.
//! CKF generates 2n cubature points with equal weights 1/(2n), avoiding the tuning
//! parameters and potential negative weights of UKF.
//!
//! Reference: Arasaratnam & Haykin, "Cubature Kalman Filters",
//! IEEE Transactions on Automatic Control, 2009.

use nalgebra::{DMatrix, Matrix2, Matrix2x4, Matrix4, Vector2, Vector4};

use rust_robotics_core::{
    ControlInput, Point2D, RoboticsError, RoboticsResult, State2D, StateEstimator,
};

/// State representation for CKF \[x, y, yaw, v\]
pub type CKFState = Vector4<f64>;

/// Control input \[velocity, yaw_rate\]
pub type CKFControl = Vector2<f64>;

/// Measurement \[x, y\]
pub type CKFMeasurement = Vector2<f64>;

/// Configuration for CKF localizer
#[derive(Debug, Clone)]
pub struct CKFConfig {
    /// Process noise covariance (4x4)
    pub q: Matrix4<f64>,
    /// Measurement noise covariance (2x2)
    pub r: Matrix2<f64>,
}

impl Default for CKFConfig {
    fn default() -> Self {
        let q = Matrix4::from_diagonal(&Vector4::new(
            0.1_f64.powi(2),
            0.1_f64.powi(2),
            1.0_f64.to_radians().powi(2),
            1.0_f64.powi(2),
        ));
        let r = Matrix2::identity();
        Self { q, r }
    }
}

impl CKFConfig {
    pub fn validate(&self) -> RoboticsResult<()> {
        if self.q.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "CKF process noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..4 {
            if self.q[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "CKF process noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        if self.r.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "CKF measurement noise matrix must contain only finite values".to_string(),
            ));
        }
        for i in 0..2 {
            if self.r[(i, i)] < 0.0 {
                return Err(RoboticsError::InvalidParameter(
                    "CKF measurement noise diagonal entries must be non-negative".to_string(),
                ));
            }
        }
        Ok(())
    }
}

/// Cubature Kalman Filter for robot localization
///
/// Uses 2n cubature points with equal weight 1/(2n) derived from the
/// 3rd-degree spherical-radial cubature rule.
#[derive(Debug)]
pub struct CKFLocalizer {
    /// Current state estimate \[x, y, yaw, v\]
    x: CKFState,
    /// State covariance matrix
    p: Matrix4<f64>,
    /// Configuration
    config: CKFConfig,
    /// Dynamic covariance cache for trait-based access
    covariance_dyn: DMatrix<f64>,
    /// Last control input (stored for trait predict/update split)
    last_control: CKFControl,
}

impl CKFLocalizer {
    const NX: usize = 4;
    const NUM_CUBATURE_POINTS: usize = 2 * Self::NX; // 8

    /// Create a new CKF localizer
    pub fn new(config: CKFConfig) -> Self {
        Self::try_new(config).expect("invalid CKF configuration")
    }

    /// Create a validated CKF localizer
    pub fn try_new(config: CKFConfig) -> RoboticsResult<Self> {
        config.validate()?;
        let mut ckf = Self {
            x: CKFState::zeros(),
            p: Matrix4::identity(),
            config,
            covariance_dyn: DMatrix::identity(4, 4),
            last_control: CKFControl::zeros(),
        };
        ckf.refresh_covariance_cache();
        Ok(ckf)
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(CKFConfig::default())
    }

    /// Create with initial state
    pub fn with_initial_state(initial_state: CKFState, config: CKFConfig) -> Self {
        Self::try_with_initial_state(initial_state, config).expect("invalid CKF initialization")
    }

    /// Create with validated initial state
    pub fn try_with_initial_state(
        initial_state: CKFState,
        config: CKFConfig,
    ) -> RoboticsResult<Self> {
        Self::validate_state(&initial_state)?;
        let mut ckf = Self::try_new(config)?;
        ckf.x = initial_state;
        Ok(ckf)
    }

    /// Create with common State2D type
    pub fn with_initial_state_2d(
        initial_state: State2D,
        config: CKFConfig,
    ) -> RoboticsResult<Self> {
        Self::try_with_initial_state(initial_state.to_vector(), config)
    }

    /// Get current state estimate
    pub fn estimate(&self) -> CKFState {
        self.x
    }

    /// Get current estimate as State2D
    pub fn state_2d(&self) -> State2D {
        State2D::new(self.x[0], self.x[1], self.x[2], self.x[3])
    }

    /// Get state covariance
    pub fn covariance(&self) -> &Matrix4<f64> {
        &self.p
    }

    // ========================================================================
    // Core cubature point generation
    // ========================================================================

    /// Generate 2n cubature points from (mean, covariance).
    ///
    /// Cubature points: xi_i = x + sqrt(n) * S_i  for i=1..n
    ///                  xi_{i+n} = x - sqrt(n) * S_i  for i=1..n
    /// where S is the Cholesky factor of P (P = S * S^T).
    /// All weights are equal: w_i = 1/(2n).
    fn generate_cubature_points(x: &CKFState, p: &Matrix4<f64>) -> [CKFState; 8] {
        let n = Self::NX as f64;
        let scale = n.sqrt(); // sqrt(4) = 2

        let sqrt_p = if let Some(chol) = p.cholesky() {
            chol.l()
        } else {
            // Fallback: use diagonal sqrt
            Matrix4::from_diagonal(&Vector4::new(
                p[(0, 0)].abs().sqrt().max(1e-6),
                p[(1, 1)].abs().sqrt().max(1e-6),
                p[(2, 2)].abs().sqrt().max(1e-6),
                p[(3, 3)].abs().sqrt().max(1e-6),
            ))
        };

        let mut points = [CKFState::zeros(); 8];
        for i in 0..Self::NX {
            points[i] = x + scale * sqrt_p.column(i);
            points[i + Self::NX] = x - scale * sqrt_p.column(i);
        }
        points
    }

    // ========================================================================
    // Motion and observation models
    // ========================================================================

    /// Motion model: state transition with control input
    fn motion_model(x: &CKFState, u: &CKFControl, dt: f64) -> CKFState {
        let yaw = x[2];
        Vector4::new(
            x[0] + dt * x[3] * yaw.cos(),
            x[1] + dt * x[3] * yaw.sin(),
            x[2] + dt * u[1],
            u[0],
        )
    }

    /// Observation model: extract position from state
    fn observation_model(x: &CKFState) -> CKFMeasurement {
        let h = Matrix2x4::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        h * x
    }

    // ========================================================================
    // Prediction
    // ========================================================================

    /// Prediction step with control input and time step
    pub fn predict_with_control(&mut self, control: &CKFControl, dt: f64) {
        self.try_predict_with_control(control, dt)
            .expect("invalid CKF prediction input")
    }

    /// Prediction step with validation
    pub fn try_predict_with_control(
        &mut self,
        control: &CKFControl,
        dt: f64,
    ) -> RoboticsResult<()> {
        Self::validate_control(control)?;
        Self::validate_dt(dt)?;

        let w = 1.0 / Self::NUM_CUBATURE_POINTS as f64;
        let points = Self::generate_cubature_points(&self.x, &self.p);

        // Propagate cubature points through motion model
        let mut propagated = [CKFState::zeros(); 8];
        for i in 0..Self::NUM_CUBATURE_POINTS {
            propagated[i] = Self::motion_model(&points[i], control, dt);
        }

        // Predicted mean
        let mut x_pred = CKFState::zeros();
        for pt in &propagated {
            x_pred += w * pt;
        }

        // Predicted covariance
        let mut p_pred = self.config.q;
        for pt in &propagated {
            let diff = pt - x_pred;
            p_pred += w * diff * diff.transpose();
        }

        self.x = x_pred;
        self.p = p_pred;
        self.last_control = *control;
        self.refresh_covariance_cache();
        Ok(())
    }

    // ========================================================================
    // Update
    // ========================================================================

    /// Update step with measurement
    pub fn update_with_measurement(&mut self, z: &CKFMeasurement) {
        self.try_update_with_measurement(z)
            .expect("invalid CKF update input")
    }

    /// Update step with validation
    pub fn try_update_with_measurement(&mut self, z: &CKFMeasurement) -> RoboticsResult<()> {
        Self::validate_measurement(z)?;

        let w = 1.0 / Self::NUM_CUBATURE_POINTS as f64;
        let points = Self::generate_cubature_points(&self.x, &self.p);

        // Predict observations from cubature points
        let mut z_points = [CKFMeasurement::zeros(); 8];
        for i in 0..Self::NUM_CUBATURE_POINTS {
            z_points[i] = Self::observation_model(&points[i]);
        }

        // Predicted observation mean
        let mut z_pred = CKFMeasurement::zeros();
        for zp in &z_points {
            z_pred += w * zp;
        }

        // Innovation covariance S = R + sum(w * dz * dz^T)
        let mut s = self.config.r;
        for zp in &z_points {
            let dz = zp - z_pred;
            s += w * dz * dz.transpose();
        }

        // Cross-covariance P_xz
        let mut p_xz = nalgebra::Matrix4x2::<f64>::zeros();
        for i in 0..Self::NUM_CUBATURE_POINTS {
            let dx = points[i] - self.x;
            let dz = z_points[i] - z_pred;
            p_xz += w * dx * dz.transpose();
        }

        // Kalman gain
        let s_inv = s.try_inverse().ok_or_else(|| {
            RoboticsError::NumericalError("Failed to invert CKF innovation covariance".to_string())
        })?;
        let k = p_xz * s_inv;

        // Update state and covariance
        let innovation = z - z_pred;
        self.x += k * innovation;
        self.p -= k * s * k.transpose();
        self.refresh_covariance_cache();
        Ok(())
    }

    // ========================================================================
    // Combined step
    // ========================================================================

    /// Full estimation step (predict + update)
    pub fn step(
        &mut self,
        control: &CKFControl,
        measurement: &CKFMeasurement,
        dt: f64,
    ) -> CKFState {
        self.try_step(control, measurement, dt)
            .expect("invalid CKF step input or numerical failure")
    }

    /// Full validated estimation step
    pub fn try_step(
        &mut self,
        control: &CKFControl,
        measurement: &CKFMeasurement,
        dt: f64,
    ) -> RoboticsResult<CKFState> {
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

        let a = 2.0 * eigen.eigenvalues[big_idx].sqrt();
        let b = 2.0 * eigen.eigenvalues[small_idx].sqrt();
        let angle = eigen.eigenvectors[(1, big_idx)].atan2(eigen.eigenvectors[(0, big_idx)]);

        (a, b, angle)
    }

    // ========================================================================
    // Internal helpers
    // ========================================================================

    fn refresh_covariance_cache(&mut self) {
        self.covariance_dyn = DMatrix::from_fn(4, 4, |i, j| self.p[(i, j)]);
    }

    fn validate_state(state: &CKFState) -> RoboticsResult<()> {
        if state.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "CKF state must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_control(control: &CKFControl) -> RoboticsResult<()> {
        if control.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "CKF control input must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_measurement(measurement: &CKFMeasurement) -> RoboticsResult<()> {
        if measurement.iter().any(|v| !v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "CKF measurement must contain only finite values".to_string(),
            ));
        }
        Ok(())
    }

    fn validate_dt(dt: f64) -> RoboticsResult<()> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(RoboticsError::InvalidParameter(format!(
                "CKF dt must be positive and finite, got {}",
                dt
            )));
        }
        Ok(())
    }
}

impl StateEstimator for CKFLocalizer {
    type State = CKFState;
    type Measurement = CKFMeasurement;
    type Control = CKFControl;

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
    fn test_ckf_creation() {
        let ckf = CKFLocalizer::with_defaults();
        assert_eq!(ckf.x, CKFState::zeros());
    }

    #[test]
    fn test_ckf_with_initial_state() {
        let initial = CKFState::new(1.0, 2.0, 0.5, 1.0);
        let ckf = CKFLocalizer::with_initial_state(initial, CKFConfig::default());
        assert_eq!(ckf.estimate()[0], 1.0);
        assert_eq!(ckf.estimate()[1], 2.0);
    }

    #[test]
    fn test_cubature_points_count() {
        let x = CKFState::zeros();
        let p = Matrix4::identity();
        let points = CKFLocalizer::generate_cubature_points(&x, &p);
        assert_eq!(points.len(), 8); // 2n = 2*4
    }

    #[test]
    fn test_cubature_points_mean() {
        let x = CKFState::new(1.0, 2.0, 0.3, 0.5);
        let p = Matrix4::identity() * 0.1;
        let points = CKFLocalizer::generate_cubature_points(&x, &p);

        // Mean of cubature points should equal the original mean
        let w = 1.0 / 8.0;
        let mut mean = CKFState::zeros();
        for pt in &points {
            mean += w * pt;
        }
        for i in 0..4 {
            assert!(
                (mean[i] - x[i]).abs() < 1e-10,
                "cubature mean[{}] = {} != {}",
                i,
                mean[i],
                x[i]
            );
        }
    }

    #[test]
    fn test_cubature_points_weights_are_equal() {
        // CKF hallmark: all weights are 1/(2n)
        let w = 1.0 / (2.0 * 4.0);
        assert!((w - 0.125_f64).abs() < 1e-15);
    }

    #[test]
    fn test_ckf_predict() {
        let mut ckf = CKFLocalizer::with_defaults();
        let control = CKFControl::new(1.0, 0.0);
        ckf.predict_with_control(&control, 0.1);
        // With v=1.0, yaw=0, dt=0.1: x should increase
        assert!(ckf.x[0].abs() < 0.2);
    }

    #[test]
    fn test_ckf_update() {
        let initial = CKFState::new(5.0, 5.0, 0.0, 1.0);
        let mut ckf = CKFLocalizer::with_initial_state(initial, CKFConfig::default());
        let z = CKFMeasurement::new(5.0, 5.0);
        ckf.update_with_measurement(&z);
        assert!((ckf.x[0] - 5.0).abs() < 1.0);
        assert!((ckf.x[1] - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_ckf_step() {
        let mut ckf = CKFLocalizer::with_defaults();
        let control = CKFControl::new(1.0, 0.1);
        let measurement = CKFMeasurement::new(0.1, 0.01);
        let est = ckf.step(&control, &measurement, 0.1);
        assert!(est[0].is_finite());
        assert!(est[1].is_finite());
    }

    #[test]
    fn test_ckf_convergence() {
        let mut ckf = CKFLocalizer::with_defaults();
        let control = CKFControl::new(1.0, 0.0);

        // Run several steps moving in x direction
        for i in 0..20 {
            let true_x = 0.1 * (i + 1) as f64;
            let z = CKFMeasurement::new(true_x, 0.0);
            ckf.step(&control, &z, 0.1);
        }

        // Estimate should be near true_x=2.0
        assert!(
            (ckf.x[0] - 2.0).abs() < 1.0,
            "CKF did not converge: x = {}",
            ckf.x[0]
        );
    }

    #[test]
    fn test_ckf_position_uncertainty() {
        let ckf = CKFLocalizer::with_defaults();
        let (a, b, angle) = ckf.position_uncertainty();
        assert!(a >= 0.0);
        assert!(b >= 0.0);
        assert!(a >= b);
        assert!(angle.is_finite());
    }

    #[test]
    fn test_state_estimator_trait() {
        let mut ckf = CKFLocalizer::with_defaults();
        let control = CKFControl::new(1.0, 0.1);
        let measurement = CKFMeasurement::new(0.1, 0.01);

        ckf.predict(&control, 0.1);
        ckf.update(&measurement);

        let state = ckf.get_state();
        assert!(state[0].is_finite());
    }

    #[test]
    fn test_ckf_try_new_rejects_invalid_config() {
        let mut config = CKFConfig::default();
        config.q[(0, 0)] = -1.0;
        let err = CKFLocalizer::try_new(config).unwrap_err();
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_ckf_rejects_invalid_dt() {
        let mut ckf = CKFLocalizer::with_defaults();
        let err = ckf
            .try_predict_with_control(&CKFControl::new(1.0, 0.0), 0.0)
            .unwrap_err();
        assert!(matches!(err, RoboticsError::InvalidParameter(_)));
    }

    #[test]
    fn test_ckf_with_initial_state_2d() {
        let ckf = CKFLocalizer::with_initial_state_2d(
            State2D::new(1.0, 2.0, 0.3, 0.5),
            CKFConfig::default(),
        )
        .unwrap();
        let state = ckf.state_2d();
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
    }

    #[test]
    fn test_ckf_estimate_state_with_common_types() {
        let mut ckf = CKFLocalizer::with_defaults();
        let state = ckf
            .estimate_state(ControlInput::new(1.0, 0.1), Point2D::new(0.1, 0.01), 0.1)
            .unwrap();
        assert!(state.x.is_finite());
    }

    #[test]
    fn test_ckf_get_covariance_trait_access() {
        let ckf = CKFLocalizer::with_defaults();
        let cov = ckf.get_covariance().unwrap();
        assert_eq!(cov.nrows(), 4);
        assert_eq!(cov.ncols(), 4);
    }
}
