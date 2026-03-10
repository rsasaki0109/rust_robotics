//! Common types, traits, and error definitions for rust_robotics
//!
//! This module provides the foundational building blocks used across
//! all robotics algorithms in this crate.

pub mod error;
pub mod traits;
pub mod types;

pub use error::*;
pub use traits::*;
pub use types::*;
