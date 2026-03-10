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
pub mod d_star_lite;
pub mod dijkstra;
pub mod jps;
pub mod theta_star;

// Sampling-based planners
pub mod informed_rrt_star;
pub mod prm;
pub mod rrt;
pub mod rrt_star;
pub mod voronoi_road_map;

// Other planners
pub mod dwa;
pub mod frenet_optimal_trajectory;
pub mod potential_field;

// State Lattice Planner
pub mod state_lattice;

// Curve generation
pub mod bezier_path;
pub mod bezier_path_planning;
pub mod csp;
pub mod cubic_spline_planner;
pub mod quintic_polynomials;
pub mod reeds_shepp_path;

// Re-exports for convenience
pub use a_star::{AStarConfig, AStarPlanner};
pub use dwa::{DWAConfig, DWAControl, DWAPlanner, DWAState, Trajectory as DWATrajectory};
pub use jps::{JPSConfig, JPSPlanner};
pub use rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTNode, RRTPlanner};
pub use state_lattice::{StateLattice, StateLatticeConfig};
pub use theta_star::{ThetaStarConfig, ThetaStarPlanner};
