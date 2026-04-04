#![forbid(unsafe_code)]
//! Localization algorithms for the RustRobotics workspace.

pub mod adaptive_filter;
pub mod cubature_kalman_filter;
pub mod ekf;
pub mod ensemble_kalman_filter;
pub mod experiments;
pub mod histogram_filter;
pub mod particle_filter;
pub mod unscented_kalman_filter;

// Re-exports
pub use ekf::{EKFConfig, EKFControl, EKFLocalizer, EKFMeasurement, EKFState};
pub use histogram_filter::{GridMap, HistogramFilter};
pub use particle_filter::{
    PFControl, PFMeasurement, PFState, Particle, ParticleFilterConfig, ParticleFilterLocalizer,
};
pub use unscented_kalman_filter::{
    UKFConfig, UKFControl, UKFLocalizer, UKFMeasurement, UKFParams, UKFState, UKFWeights,
};
