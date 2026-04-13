//! Gaussian Process regression for 2D-to-1D surface mapping.

use nalgebra::{DMatrix, DVector};

use rust_robotics_core::RoboticsError;

/// Configuration for Gaussian Process regression.
#[derive(Debug, Clone, Copy)]
pub struct GaussianProcessConfig {
    /// RBF kernel length scale.
    pub length_scale: f64,
    /// Kernel signal variance.
    pub signal_variance: f64,
    /// Observation noise variance.
    pub noise_variance: f64,
}

impl Default for GaussianProcessConfig {
    fn default() -> Self {
        Self {
            length_scale: 1.0,
            signal_variance: 1.0,
            noise_variance: 0.01,
        }
    }
}

/// Trained Gaussian Process model.
#[derive(Debug, Clone)]
pub struct GaussianProcessRegressor {
    config: GaussianProcessConfig,
    train_xy: Vec<(f64, f64)>,
    alpha: Option<DVector<f64>>,
    k_inv: Option<DMatrix<f64>>,
}

impl GaussianProcessRegressor {
    pub fn new(config: GaussianProcessConfig) -> Self {
        Self {
            config,
            train_xy: Vec::new(),
            alpha: None,
            k_inv: None,
        }
    }

    /// Fit GP model from `(x, y, z)` samples.
    pub fn fit(&mut self, samples: &[(f64, f64, f64)]) -> Result<(), RoboticsError> {
        if samples.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "GaussianProcess: at least one training sample is required".to_string(),
            ));
        }
        if self.config.length_scale <= 0.0 || self.config.signal_variance <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "GaussianProcess: length_scale and signal_variance must be positive".to_string(),
            ));
        }
        if self.config.noise_variance <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "GaussianProcess: noise_variance must be positive".to_string(),
            ));
        }

        let n = samples.len();
        self.train_xy = samples.iter().map(|(x, y, _)| (*x, *y)).collect();
        let z = DVector::from_iterator(n, samples.iter().map(|(_, _, z)| *z));

        let mut k = self.kernel_matrix(&self.train_xy, &self.train_xy);
        for i in 0..n {
            k[(i, i)] += self.config.noise_variance;
        }

        let Some(cholesky) = k.clone().cholesky() else {
            return Err(RoboticsError::NumericalError(
                "GaussianProcess: kernel matrix is not positive definite".to_string(),
            ));
        };

        let alpha = cholesky.solve(&z);
        let identity = DMatrix::<f64>::identity(n, n);
        let k_inv = cholesky.solve(&identity);

        self.alpha = Some(alpha);
        self.k_inv = Some(k_inv);
        Ok(())
    }

    /// Predict `(mean, variance)` for query `(x, y)` points.
    pub fn predict(
        &self,
        query_points: &[(f64, f64)],
    ) -> Result<(Vec<f64>, Vec<f64>), RoboticsError> {
        if query_points.is_empty() {
            return Ok((Vec::new(), Vec::new()));
        }
        let Some(alpha) = self.alpha.as_ref() else {
            return Err(RoboticsError::InvalidParameter(
                "GaussianProcess: model must be fitted before predict".to_string(),
            ));
        };
        let Some(k_inv) = self.k_inv.as_ref() else {
            return Err(RoboticsError::InvalidParameter(
                "GaussianProcess: inverse kernel state missing; call fit first".to_string(),
            ));
        };

        let k_star = self.kernel_matrix(query_points, &self.train_xy); // m x n
        let mean_vec = &k_star * alpha; // m x 1

        let mut means = Vec::with_capacity(query_points.len());
        let mut variances = Vec::with_capacity(query_points.len());

        for i in 0..query_points.len() {
            let query = query_points[i];
            let k_ss = self.kernel(query, query);
            let k_row = k_star.row(i).transpose();
            let variance = (k_ss - (k_row.transpose() * k_inv * k_row)[(0, 0)]).max(0.0);
            means.push(mean_vec[i]);
            variances.push(variance);
        }

        Ok((means, variances))
    }

    fn kernel(&self, a: (f64, f64), b: (f64, f64)) -> f64 {
        let dx = a.0 - b.0;
        let dy = a.1 - b.1;
        let r2 = dx * dx + dy * dy;
        self.config.signal_variance
            * (-0.5 * r2 / (self.config.length_scale * self.config.length_scale)).exp()
    }

    fn kernel_matrix(&self, lhs: &[(f64, f64)], rhs: &[(f64, f64)]) -> DMatrix<f64> {
        let mut matrix = DMatrix::zeros(lhs.len(), rhs.len());
        for i in 0..lhs.len() {
            for j in 0..rhs.len() {
                matrix[(i, j)] = self.kernel(lhs[i], rhs[j]);
            }
        }
        matrix
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaussian_process_config_defaults() {
        let config = GaussianProcessConfig::default();
        assert_eq!(config.length_scale, 1.0);
        assert_eq!(config.signal_variance, 1.0);
        assert_eq!(config.noise_variance, 0.01);
    }

    #[test]
    fn test_gaussian_process_predicts_known_function() {
        let mut gp = GaussianProcessRegressor::new(GaussianProcessConfig::default());
        let train = vec![
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 1.8414709848),
            (0.0, 1.0, 0.5403023059),
            (1.0, 1.0, 1.3817732907),
            (0.5, 0.5, 1.3570081005),
        ];
        gp.fit(&train).expect("fit should succeed");

        let query = vec![(0.5, 0.5), (0.8, 0.2)];
        let (mean, _) = gp.predict(&query).expect("predict should succeed");

        assert!((mean[0] - 1.3570081005).abs() < 0.1);
        let expected = 0.8f64.sin() + 0.2f64.cos();
        assert!((mean[1] - expected).abs() < 0.2);
    }

    #[test]
    fn test_gaussian_process_uncertainty_increases_away_from_data() {
        let mut gp = GaussianProcessRegressor::new(GaussianProcessConfig::default());
        let train = vec![
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.5),
            (0.0, 0.5, 0.5),
            (0.5, 0.5, 1.0),
        ];
        gp.fit(&train).expect("fit should succeed");

        let (_, var_near) = gp.predict(&[(0.25, 0.25)]).expect("predict near");
        let (_, var_far) = gp.predict(&[(3.0, 3.0)]).expect("predict far");

        assert!(var_far[0] > var_near[0]);
    }
}
