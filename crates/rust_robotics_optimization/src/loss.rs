/// Value and first two derivatives of a robust loss `rho(s)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LossEvaluation {
    pub value: f64,
    pub first_derivative: f64,
    pub second_derivative: f64,
}

/// Robust loss applied to a squared Mahalanobis residual.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum RobustKernel {
    #[default]
    L2,
    Huber {
        delta: f64,
    },
    PseudoHuber {
        delta: f64,
    },
    Cauchy {
        delta: f64,
    },
}

impl RobustKernel {
    /// Evaluates `rho(s)`, `rho'(s)`, and `rho''(s)`.
    pub fn evaluate(self, squared_error: f64) -> LossEvaluation {
        let squared_error = squared_error.max(0.0);
        match self {
            Self::L2 => LossEvaluation {
                value: squared_error,
                first_derivative: 1.0,
                second_derivative: 0.0,
            },
            Self::Huber { delta } => {
                let delta = delta.abs().max(f64::EPSILON);
                let delta_squared = delta * delta;
                if squared_error <= delta_squared {
                    Self::L2.evaluate(squared_error)
                } else {
                    let root = squared_error.sqrt();
                    let first_derivative = delta / root;
                    LossEvaluation {
                        value: 2.0 * delta * root - delta_squared,
                        first_derivative,
                        second_derivative: -0.5 * first_derivative / squared_error,
                    }
                }
            }
            Self::PseudoHuber { delta } => {
                let delta = delta.abs().max(f64::EPSILON);
                let delta_squared = delta * delta;
                let auxiliary = 1.0 + squared_error / delta_squared;
                let root = auxiliary.sqrt();
                let first_derivative = 1.0 / root;
                LossEvaluation {
                    value: 2.0 * delta_squared * (root - 1.0),
                    first_derivative,
                    second_derivative: -0.5 * first_derivative / (delta_squared * auxiliary),
                }
            }
            Self::Cauchy { delta } => {
                let delta = delta.abs().max(f64::EPSILON);
                let delta_squared = delta * delta;
                let auxiliary = 1.0 + squared_error / delta_squared;
                let first_derivative = 1.0 / auxiliary;
                LossEvaluation {
                    value: delta_squared * auxiliary.ln(),
                    first_derivative,
                    second_derivative: -first_derivative * first_derivative / delta_squared,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robust_kernels_reduce_outlier_influence() {
        let error = 100.0;
        for kernel in [
            RobustKernel::Huber { delta: 1.0 },
            RobustKernel::PseudoHuber { delta: 1.0 },
            RobustKernel::Cauchy { delta: 1.0 },
        ] {
            let evaluation = kernel.evaluate(error);
            assert!(evaluation.value < error);
            assert!(evaluation.first_derivative < 1.0);
            assert!(evaluation.first_derivative > 0.0);
        }
    }

    #[test]
    fn huber_is_continuous_at_threshold() {
        let kernel = RobustKernel::Huber { delta: 2.0 };
        let at_threshold = kernel.evaluate(4.0);
        let just_above = kernel.evaluate(4.0 + 1.0e-10);
        assert!((at_threshold.value - just_above.value).abs() < 1.0e-9);
        assert!((at_threshold.first_derivative - just_above.first_derivative).abs() < 1.0e-9);
    }
}
