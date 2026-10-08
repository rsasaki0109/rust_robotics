#![forbid(unsafe_code)]
//! SLAM algorithms for the RustRobotics workspace.

pub mod bundle_adjustment;
pub mod correlative_scan_matching;
pub mod dataset;
pub mod ekf_slam;
pub mod fastslam1;
pub mod fastslam2;
pub mod g2o;
pub mod geometric_icp;
pub mod graph_based_slam;
pub mod icp_matching;
pub mod imu_preintegration;
pub mod lidar_graph_slam;
pub mod lidar_loop_scenario;
pub mod pose_graph_optimization;
pub mod pose_graph_optimization_3d;
pub mod robust_icp;
pub mod scan_to_map;
pub mod vio_pipeline;
pub mod visual_frontend;

// Re-exports
pub use ekf_slam::EKFSLAMState;
pub use icp_matching::ICPResult;
pub use lidar_graph_slam::{LidarGraphSlam, LidarGraphSlamConfig, LoopClosure};
pub use robust_icp::{RobustICPResult, RobustIcp2D, Transform2D};
pub use scan_to_map::{MatchStatus, ScanToMapConfig, ScanToMapMatcher, ScanToMapUpdate};
