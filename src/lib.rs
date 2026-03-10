//! RustRobotics - Rust implementation of robotics algorithms
//!
//! This crate provides implementations of common robotics algorithms
//! for localization, mapping, path planning, path tracking, and control.

extern crate self as rust_robotics;

// Core modules
pub mod common;
pub mod utils;

// Algorithm modules
pub mod aerial_navigation;
pub mod arm_navigation;
pub mod control;
pub mod localization;
pub mod mapping;
pub mod mission_planning;
pub mod path_planning;
pub mod path_tracking;
pub mod slam;

// Re-export common types for convenience
pub use common::{ControlInput, Obstacles, Path2D, Point2D, Pose2D, State2D};
pub use common::{PathPlanner, PathTracker, StateEstimator};
pub use common::{RoboticsError, RoboticsResult};
