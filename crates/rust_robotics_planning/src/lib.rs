#![forbid(unsafe_code)]
//! Path planning algorithms for the RustRobotics workspace.

pub mod experiments;
pub mod grid;
pub mod grid_nalgebra;

// Grid-based planners
pub mod a_star;
pub mod a_star_variants;
pub mod adaptive_costmap_namo;
pub mod anya;
pub mod ara_star;
pub mod bidirectional_a_star;
pub mod bidirectional_bfs;
pub mod bidirectional_rrt;
pub mod bipedal_planner;
pub mod branchout_multimodal;
pub mod breadth_first_search;
pub mod bug_planning;
pub mod conformal_sipp;
pub mod d_star;
pub mod d_star_lite;
pub mod depth_first_search;
pub mod dijkstra;
pub mod enhanced_lazy_theta_star;
pub mod flow_field;
pub mod fringe_search;
pub mod frontier_navigator;
pub mod greedy_best_first_search;
pub mod hierarchical_mapf;
pub mod hybrid_a_star;
pub mod ida_star;
pub mod jps;
pub mod kinodynamic_stl_cbs;
pub mod lazy_theta_star;
pub mod lpa_star;
pub mod rigid_body_mip;
pub mod safe_decode_nav;
pub mod sipp;
pub mod stl_cbs;
pub mod tangent_bug;
pub mod theta_star;
pub mod traversal_risk_graph;

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
pub use adaptive_costmap_namo::{
    AdaptiveCostmapCell, AdaptiveCostmapCellState, AdaptiveCostmapNamo, AdaptiveCostmapNamoConfig,
    MotionProgressObservation,
};
pub use bidirectional_a_star::{BidirectionalAStarConfig, BidirectionalAStarPlanner};
pub use bidirectional_bfs::{BidirectionalBFSConfig, BidirectionalBFSPlanner};
pub use bit_star::{BITStar, BITStarConfig};
pub use branchout_multimodal::{
    BranchOutClosedLoopConfig2D, BranchOutClosedLoopMetrics2D, BranchOutDecisionMode2D,
    BranchOutDrivingScene2D, BranchOutMultimodalMetrics2D, BranchOutObstacle2D, BranchOutPlan2D,
    BranchOutPlanner2D, BranchOutPlannerConfig2D, BranchOutPose2D, BranchOutTrajectory2D,
};
pub use breadth_first_search::{BFSConfig, BFSPlanner};
pub use clothoid_path::{ClothoidConfig, ClothoidPath, ClothoidPlanner};
pub use conformal_sipp::{
    ConformalSippConfig, ConformalSippPlan, ConformalSippPlanner, PredictedObstaclePoint,
    PredictedObstacleTrajectory,
};
pub use cubic_spline_planner::{CubicSplinePlanner, Spline2D};
pub use d_star_lite::DStarLite;
pub use depth_first_search::{DFSConfig, DFSPlanner};
pub use dubins_path::{DubinsPath, DubinsPlanner};
pub use dwa::{DWAConfig, DWAPlanner};
pub use enhanced_lazy_theta_star::{EnhancedLazyThetaStarConfig, EnhancedLazyThetaStarPlanner};
pub use flow_field::{FlowFieldConfig, FlowFieldPlanner};
pub use fringe_search::{FringeSearchConfig, FringeSearchPlanner};
pub use frontier_navigator::{
    simulate_frontier_navigation, FrontierChoice, FrontierNavConfig, FrontierNavReport,
    FrontierNavWorld, Knowledge,
};
pub use greedy_best_first_search::{GreedyBestFirstConfig, GreedyBestFirstPlanner};
pub use grid::GridMap;
pub use grid_a_star_3d::{GridAStar3DConfig, GridAStar3DPlanner, Path3D};
pub use hierarchical_mapf::{
    cell_conflict_count, HierarchicalMapfAgent2D, HierarchicalMapfConfig2D, HierarchicalMapfPlan2D,
    HierarchicalMapfPlanner2D, HierarchicalMapfRegion2D, HierarchicalMapfRegionConflict2D,
    HierarchicalMapfRegionRoute2D, HierarchicalMapfReplannedGroup2D,
};
pub use hybrid_a_star::{HybridAStarConfig, HybridAStarPath, HybridAStarPlanner};
pub use ida_star::{IDAStarConfig, IDAStarPlanner};
pub use informed_rrt_star::InformedRRTStar;
pub use jps::{JPSConfig, JPSPlanner};
pub use kinodynamic_stl_cbs::{
    first_kinodynamic_continuous_conflict, kinodynamic_continuous_pairwise_separation_robustness,
    KinodynamicContinuousConflict2D, KinodynamicContinuousPose2D, KinodynamicHeading2D,
    KinodynamicStlCbsAgent2D, KinodynamicStlCbsConfig2D, KinodynamicStlCbsPath2D,
    KinodynamicStlCbsPlan2D, KinodynamicStlCbsPlanner2D, KinodynamicTimedPose2D,
};
pub use lazy_theta_star::{LazyThetaStarConfig, LazyThetaStarPlanner};
pub use potential_field::PotentialFieldPlanner;
pub use prm::PRMPlanner;
pub use prm_star::{PRMStarConfig, PRMStarPlanner};
pub use quintic_polynomials::{QuinticPolynomial, QuinticPolynomialsPlanner};
pub use reeds_shepp_path::ReedsSheppPlanner;
pub use rigid_body_mip::{
    RigidBodyConvexObstacle2D, RigidBodyHalfspace2D, RigidBodyMipConfig2D, RigidBodyMipPlan2D,
    RigidBodyMipPlanner2D, RigidBodyMipSeparationCertificate2D, RigidBodyPlanOutcome2D,
    RigidBodyPlanningBackend, RigidBodyPoint2D, RigidBodyPose2D, RigidBodyRrtBackend2D,
    RigidBodyRrtConfig2D,
};
pub use rrt::{AreaBounds, CircleObstacle, RRTConfig, RRTPlanner};
pub use rrt_path_smoothing::{RRTPathSmoothingConfig, RRTPathSmoothingPlanner};
pub use rrt_sobol::RRTSobolPlanner;
pub use rrt_star::RRTStar;
pub use safe_decode_nav::{SafeDecodePlan, SafeDecoder, SafeNavConfig, TimedRegion};
pub use sipp::{SippConfig, SippPlanner};
pub use state_lattice::{ObstacleAwarePlanResult, StateLattice, StateLatticeConfig};
pub use stl_cbs::{
    first_conflict, stl_always_avoid_robustness, stl_eventually_reach_robustness,
    stl_pairwise_separation_robustness, StlCbsAgent, StlCbsConfig, StlCbsConflict,
    StlCbsConflictKind, StlCbsPath, StlCbsPlan, StlCbsPlanner, StlRectangle2D, StlTimeInterval,
    StlTimedCell,
};
pub use theta_star::{ThetaStarConfig, ThetaStarPlanner};
pub use traversal_risk_graph::{
    add_clearance_exposure_risk, clearance_map, inflate_blocked_cells,
    inflate_blocked_cells_by_radius, smooth_terrain_risk, sweep_traversal_risk_weights,
    terrain_risk_from_elevation_map, ClearanceRiskConfig, ElevationRiskConfig,
    RiskMapSmoothingConfig, RiskWaypoint, TerrainRiskCell, TraversalRiskGraphConfig,
    TraversalRiskGraphPlanner, TraversalRiskPath, TraversalRiskWeightSample,
};
pub use voronoi_road_map::VoronoiPlanner;
