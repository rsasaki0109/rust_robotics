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
