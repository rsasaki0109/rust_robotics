//! Common types, traits, and error definitions for rust_robotics
//!
//! This module provides the foundational building blocks used across
//! all robotics algorithms in this crate.

pub mod types;
pub mod traits;
pub mod error;

pub use types::*;
pub use traits::*;
pub use error::*;
