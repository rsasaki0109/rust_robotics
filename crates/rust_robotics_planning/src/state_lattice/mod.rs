//! State Lattice Planner Module
//!
//! This module implements state lattice planning for path planning.
//! State lattice planning generates a set of trajectories by sampling
//! in the state space and uses model predictive trajectory generation
//! to create smooth, kinematically feasible paths.
//!
//! # Components
//!
//! - `motion_model`: Bicycle kinematic model for vehicle motion
//! - `trajectory_generator`: Model predictive trajectory optimization
//! - `state_lattice_planner`: Main planner with state sampling strategies
//!
//! # References
//!
//! - PythonRobotics State Lattice Planner by Atsushi Sakai
//! - "State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation"

pub mod motion_model;
pub mod state_lattice_planner;
pub mod trajectory_generator;

// Re-exports
pub use motion_model::{MotionModel, MotionModelConfig, VehicleState};
pub use state_lattice_planner::{StateLattice, StateLatticeConfig, TargetPose, Trajectory};
pub use trajectory_generator::{
    LookupTable, LookupTableEntry, TargetState, TrajectoryGenerator, TrajectoryGeneratorConfig,
    TrajectoryParams,
};
