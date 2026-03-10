//! Control algorithms module
//!
//! Includes various control algorithms such as LQR and MPC.

#[cfg(feature = "viz")]
pub mod lqr_control;
#[cfg(feature = "viz")]
pub mod mpc_control;

// Re-exports will be added after refactoring
