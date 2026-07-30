use core::fmt;

/// Result type used by optimization operations.
pub type OptimizationResult<T> = Result<T, OptimizationError>;

/// Errors produced while building or solving an optimization problem.
#[derive(Debug, Clone, PartialEq)]
pub enum OptimizationError {
    InvalidParameter(String),
    InvalidFactor(String),
    LinearSolveFailed,
    NonFiniteValue,
}

impl fmt::Display for OptimizationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter(message) => write!(formatter, "invalid parameter: {message}"),
            Self::InvalidFactor(message) => write!(formatter, "invalid factor: {message}"),
            Self::LinearSolveFailed => formatter.write_str("linear system could not be solved"),
            Self::NonFiniteValue => formatter.write_str("optimization produced a non-finite value"),
        }
    }
}

impl std::error::Error for OptimizationError {}
