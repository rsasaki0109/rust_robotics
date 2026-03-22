#![forbid(unsafe_code)]
//! SLAM algorithms for the RustRobotics workspace.

pub mod ekf_slam;
pub mod fastslam1;
pub mod graph_based_slam;
pub mod icp_matching;

// Re-exports
pub use ekf_slam::EKFSLAMState;
pub use icp_matching::ICPResult;
