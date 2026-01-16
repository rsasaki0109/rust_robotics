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
//! # Example
//!
//! ```no_run
//! use rust_robotics::path_planning::state_lattice::{StateLattice, StateLatticeConfig};
//! use rust_robotics::common::Point2D;
//!
//! // Create planner with default config
//! let planner = StateLattice::with_defaults();
//!
//! // Generate paths using uniform polar sampling
//! let paths = planner.plan_uniform_polar();
//!
//! // Or plan to a specific goal
//! let start = Point2D::new(0.0, 0.0);
//! let goal = Point2D::new(20.0, 5.0);
//! let path = planner.plan(start, goal);
//! ```
//!
//! # References
//!
//! - PythonRobotics State Lattice Planner by Atsushi Sakai
//! - "State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation"

pub mod motion_model;
pub mod trajectory_generator;
pub mod state_lattice_planner;

// Re-exports
pub use motion_model::{MotionModel, MotionModelConfig, VehicleState};
pub use trajectory_generator::{
    LookupTable, LookupTableEntry, TargetState, TrajectoryGenerator, TrajectoryGeneratorConfig,
    TrajectoryParams,
};
pub use state_lattice_planner::{
    StateLattice, StateLatticeConfig, TargetPose, Trajectory,
};
