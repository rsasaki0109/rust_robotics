//! RustRobotics - Rust implementation of robotics algorithms
//!
//! This crate provides implementations of common robotics algorithms
//! for localization, mapping, path planning, path tracking, and control.

// Core modules
pub mod common;
pub mod utils;

// Algorithm modules
pub mod localization;
pub mod mapping;
pub mod slam;
pub mod path_planning;
pub mod path_tracking;
pub mod control;
pub mod arm_navigation;
pub mod aerial_navigation;
pub mod mission_planning;

// Re-export common types for convenience
pub use common::{Point2D, Pose2D, State2D, Path2D, ControlInput, Obstacles};
pub use common::{PathPlanner, StateEstimator, PathTracker};
pub use common::{RoboticsError, RoboticsResult};
