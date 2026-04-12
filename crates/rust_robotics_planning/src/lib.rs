#![forbid(unsafe_code)]
//! Path planning algorithms for the RustRobotics workspace.

pub mod experiments;
pub mod grid;
pub mod grid_nalgebra;

// Grid-based planners
pub mod a_star;
pub mod a_star_variants;
pub mod anya;
pub mod ara_star;
pub mod bidirectional_a_star;
pub mod bidirectional_bfs;
pub mod bidirectional_rrt;
pub mod bipedal_planner;
pub mod breadth_first_search;
pub mod bug_planning;
pub mod d_star;
pub mod d_star_lite;
pub mod depth_first_search;
pub mod dijkstra;
pub mod enhanced_lazy_theta_star;
pub mod flow_field;
pub mod fringe_search;
pub mod greedy_best_first_search;
pub mod hybrid_a_star;
pub mod ida_star;
pub mod jps;
pub mod lazy_theta_star;
pub mod lpa_star;
pub mod sipp;
pub mod tangent_bug;
pub mod theta_star;

// Sampling-based planners
pub mod batch_informed_rrt_star;
pub mod bit_star;
pub mod chomp;
pub mod closed_loop_rrt_star;
pub mod fmt_star;
pub mod informed_rrt_star;
pub mod lqr_rrt_star;
pub mod prm;
pub mod prm_star;
pub mod rrg;
pub mod rrt;
pub mod rrt_connect;
pub mod rrt_dubins;
pub mod rrt_path_smoothing;
pub mod rrt_sobol;
pub mod rrt_star;
pub mod rrt_star_dubins;
pub mod rrt_star_reeds_shepp;
pub mod voronoi_road_map;

// Optimization-based planners
pub mod path_smoothing;

pub mod dwa;
pub mod dynamic_movement_primitives;
pub mod elastic_bands;
pub mod frenet_optimal_trajectory;
pub mod lqr_planner;
pub mod model_predictive_trajectory_generator;
pub mod moving_ai;
pub mod particle_swarm_optimization;
pub mod potential_field;
pub mod time_based_path_planning;

// Curve-based planners
pub mod bezier_path;
pub mod bezier_path_planning;
pub mod bspline_path;
pub mod catmull_rom_spline;
pub mod clothoid_path;
pub mod cubic_spline_planner;
pub mod dubins_path;
pub mod eta3_spline;
pub mod quintic_polynomials;
pub mod reeds_shepp_path;

// State lattice planner
pub mod state_lattice;

// Coverage planning
pub mod coverage_planning;
pub mod grid_based_sweep_cpp;
pub mod spiral_spanning_tree_cpp;
pub mod wavefront_cpp;

// Visibility graph
pub mod visibility_road_map;

// 3D planning
pub mod grid_a_star_3d;

// Re-exports
pub use a_star::{AStarConfig, AStarPlanner};
pub use a_star_variants::{AStarVariantConfig, AStarVariantMode, AStarVariantPlanner};
pub use bidirectional_a_star::{BidirectionalAStarConfig, BidirectionalAStarPlanner};
pub use bidirectional_bfs::{BidirectionalBFSConfig, BidirectionalBFSPlanner};
pub use bit_star::{BITStar, BITStarConfig};
pub use breadth_first_search::{BFSConfig, BFSPlanner};
pub use clothoid_path::{ClothoidConfig, ClothoidPath, ClothoidPlanner};
pub use cubic_spline_planner::{CubicSplinePlanner, Spline2D};
pub use d_star_lite::DStarLite;
pub use depth_first_search::{DFSConfig, DFSPlanner};
pub use dubins_path::{DubinsPath, DubinsPlanner};
pub use dwa::{DWAConfig, DWAPlanner};
pub use enhanced_lazy_theta_star::{EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner};
pub use flow_field::{FlowFieldConfig, FlowFieldPlanner};
pub use fringe_search::{FringeSearchConfig, FringeSearchPlanner};
pub use greedy_best_first_search::{GreedyBestFirstConfig, GreedyBestFirstPlanner};
pub use grid::GridMap;
pub use grid_a_star_3d::{GridAStar3DConfig, GridAStar3DPlanner, Path3D};
pub use hybrid_a_star::{HybridAStarConfig, HybridAStarPath, HybridAStarPlanner};
pub use ida_star::{IDAStarConfig, IDAStarPlanner};
pub use informed_rrt_star::InformedRRTStar;
pub use jps::{JPSConfig, JPSPlanner};
pub use lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
pub use potential_field::PotentialFieldPlanner;
pub use prm::PRMPlanner;
pub use prm_star::{PRMStarConfig, PRMStarPlanner};
pub use quintic_polynomials::{QuinticPolynomial, QuinticPolynomialsPlanner};
pub use reeds_shepp_path::ReedsSheppPlanner;
pub use rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};
pub use rrt_path_smoothing::{RRTPathSmoothingConfig, RRTPathSmoothingPlanner};
pub use rrt_sobol::RRTSobolPlanner;
pub use rrt_star::RRTStar;
pub use sipp::{SippConfig, SippPlanner};
pub use state_lattice::{ObstacleAwarePlanResult, StateLattice, StateLatticeConfig};
pub use theta_star::{ThetaStarConfig, ThetaStarPlanner};
pub use voronoi_road_map::VoronoiPlanner;
