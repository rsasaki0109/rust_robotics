#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]
//! Core types, traits, and error definitions for the RustRobotics workspace.
//!
//! Builds without `std` (default feature `std` disabled) for embedded
//! targets; only the `experiments` module requires the standard library.

extern crate alloc;

/// Error types and definitions for the RustRobotics workspace.
pub mod error;
#[cfg(feature = "std")]
pub mod experiments;
pub mod traits;
pub mod types;

pub use error::*;
#[cfg(feature = "std")]
pub use experiments::*;
pub use traits::*;
pub use types::*;
