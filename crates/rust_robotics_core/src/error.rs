//! Error types for rust_robotics

use std::fmt;

/// Main error type for robotics algorithms
#[derive(Debug)]
pub enum RoboticsError {
    /// Path planning failed
    PlanningError(String),
    /// State estimation failed
    EstimationError(String),
    /// Control computation failed
    ControlError(String),
    /// Invalid parameter
    InvalidParameter(String),
    /// Numerical computation failed (matrix inversion, etc.)
    NumericalError(String),
    /// I/O error
    IoError(std::io::Error),
    /// Visualization error
    VisualizationError(String),
}

impl fmt::Display for RoboticsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RoboticsError::PlanningError(msg) => write!(f, "Planning error: {}", msg),
            RoboticsError::EstimationError(msg) => write!(f, "Estimation error: {}", msg),
            RoboticsError::ControlError(msg) => write!(f, "Control error: {}", msg),
            RoboticsError::InvalidParameter(msg) => write!(f, "Invalid parameter: {}", msg),
            RoboticsError::NumericalError(msg) => write!(f, "Numerical error: {}", msg),
            RoboticsError::IoError(e) => write!(f, "I/O error: {}", e),
            RoboticsError::VisualizationError(msg) => write!(f, "Visualization error: {}", msg),
        }
    }
}

impl std::error::Error for RoboticsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            RoboticsError::IoError(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for RoboticsError {
    fn from(e: std::io::Error) -> Self {
        RoboticsError::IoError(e)
    }
}

/// Result type alias for robotics operations
pub type RoboticsResult<T> = Result<T, RoboticsError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = RoboticsError::PlanningError("No path found".to_string());
        assert_eq!(format!("{}", err), "Planning error: No path found");
    }

    #[test]
    fn test_error_from_io() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let err: RoboticsError = io_err.into();
        assert!(matches!(err, RoboticsError::IoError(_)));
    }
}
