#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]
//! Localization algorithms for the RustRobotics workspace.
//!
//! With default features disabled this crate is `no_std` (plus `alloc`) and
//! the Kalman-family filters (EKF, IEKF, UKF, CKF, SR-UKF, information,
//! complementary, histogram, adaptive) build for bare-metal targets such as
//! `thumbv7em-none-eabihf`. The sampling-based localizers (particle filter,
//! Monte Carlo localization, ensemble KF) need `std` for random number
//! generation.

extern crate alloc;

pub mod adaptive_filter;
pub mod cubature_kalman_filter;
pub mod ekf;
#[cfg(feature = "std")]
pub mod ensemble_kalman_filter;
#[cfg(feature = "std")]
pub mod experiments;
pub mod histogram_filter;
#[cfg(feature = "std")]
pub mod monte_carlo_localization;
#[cfg(feature = "std")]
pub mod particle_filter;
pub mod unscented_kalman_filter;

pub mod complementary_filter;
pub mod information_filter;
pub mod iterated_ekf;
pub mod square_root_ukf;

// Re-exports
pub use ekf::{EKFConfig, EKFControl, EKFLocalizer, EKFMeasurement, EKFState};
pub use histogram_filter::{GridMap, HistogramFilter};
#[cfg(feature = "std")]
pub use monte_carlo_localization::{
    MCLControl, MCLMeasurement, MCLState, MonteCarloLocalizationConfig, MonteCarloLocalizer,
};
#[cfg(feature = "std")]
pub use particle_filter::{
    PFControl, PFMeasurement, PFState, Particle, ParticleFilterConfig, ParticleFilterLocalizer,
};
pub use unscented_kalman_filter::{
    UKFConfig, UKFControl, UKFLocalizer, UKFMeasurement, UKFParams, UKFState, UKFWeights,
};
