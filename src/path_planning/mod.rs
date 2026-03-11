//! Path Planning algorithms module
//!
//! This module contains various path planning algorithms including:
//! - Grid-based: A*, Dijkstra, D* Lite
//! - Sampling-based: RRT, RRT*, Informed RRT*, PRM
//! - Potential field methods
//! - Curve-based: Bezier, Cubic Spline, Quintic Polynomials, Reeds-Shepp
//! - State Lattice Planner

// Grid-based planners
pub mod a_star;
#[cfg(feature = "viz")]
pub mod d_star_lite;
#[cfg(feature = "viz")]
pub mod dijkstra;
pub mod jps;
pub mod theta_star;

// Sampling-based planners
#[cfg(feature = "viz")]
pub mod informed_rrt_star;
#[cfg(feature = "viz")]
pub mod prm;
pub mod rrt;
pub mod rrt_star;
#[cfg(feature = "viz")]
pub mod voronoi_road_map;

// Other planners
pub mod dwa;
#[cfg(feature = "viz")]
pub mod frenet_optimal_trajectory;
#[cfg(feature = "viz")]
pub mod potential_field;

// State Lattice Planner
pub mod state_lattice;

// Curve generation
#[cfg(feature = "viz")]
pub mod bezier_path;
#[cfg(feature = "viz")]
pub mod bezier_path_planning;
#[cfg(feature = "viz")]
pub mod csp;
#[cfg(feature = "viz")]
pub mod cubic_spline_planner;
#[cfg(feature = "viz")]
pub mod quintic_polynomials;
#[cfg(feature = "viz")]
pub mod reeds_shepp_path;

// Re-exports for convenience
pub use a_star::{AStarConfig, AStarPlanner};
pub use dwa::{DWAConfig, DWAControl, DWAPlanner, DWAState, Trajectory as DWATrajectory};
pub use jps::{JPSConfig, JPSPlanner};
pub use rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTNode, RRTPlanner};
pub use state_lattice::{StateLattice, StateLatticeConfig};
pub use theta_star::{ThetaStarConfig, ThetaStarPlanner};
