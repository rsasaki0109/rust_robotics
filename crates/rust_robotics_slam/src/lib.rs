#![forbid(unsafe_code)]
//! SLAM algorithms for the RustRobotics workspace.

pub mod correlative_scan_matching;
pub mod ekf_slam;
pub mod fastslam1;
pub mod fastslam2;
pub mod graph_based_slam;
pub mod icp_matching;
pub mod pose_graph_optimization;
pub mod robust_icp;

// Re-exports
pub use ekf_slam::EKFSLAMState;
pub use icp_matching::ICPResult;
pub use robust_icp::{RobustICPResult, RobustIcp2D, Transform2D};
