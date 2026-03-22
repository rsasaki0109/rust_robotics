#![forbid(unsafe_code)]
//! RustRobotics — Rust implementation of robotics algorithms.
//!
//! This umbrella crate provides feature-gated re-exports of all domain crates.

// Always available
pub use rust_robotics_core as core;

#[cfg(feature = "planning")]
pub use rust_robotics_planning as planning;

#[cfg(feature = "localization")]
pub use rust_robotics_localization as localization;

#[cfg(feature = "control")]
pub use rust_robotics_control as control;

#[cfg(feature = "mapping")]
pub use rust_robotics_mapping as mapping;

#[cfg(feature = "slam")]
pub use rust_robotics_slam as slam;

#[cfg(feature = "viz")]
pub use rust_robotics_viz as viz;

/// Prelude — import commonly used types with `use rust_robotics::prelude::*`.
pub mod prelude {
    pub use rust_robotics_core::*;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_types_accessible() {
        let p = core::Point2D::new(1.0, 2.0);
        assert!((p.x - 1.0).abs() < f64::EPSILON);
        assert!((p.y - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn prelude_reexports_work() {
        use crate::prelude::*;
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        assert!((pose.yaw).abs() < f64::EPSILON);
    }

    #[test]
    #[cfg(feature = "planning")]
    fn planning_module_accessible() {
        let _ = planning::grid::GridMap::new(&[0.0, 10.0], &[0.0, 10.0], 1.0, 0.5);
    }

    #[test]
    #[cfg(feature = "localization")]
    fn localization_module_accessible() {
        let _config = localization::EKFConfig::default();
    }

    #[test]
    #[cfg(feature = "control")]
    fn control_module_accessible() {
        let _config = control::rear_wheel_feedback::RearWheelFeedbackConfig::default();
    }
}
