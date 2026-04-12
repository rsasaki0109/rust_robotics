#![forbid(unsafe_code)]
//! Control and path tracking algorithms for the RustRobotics workspace.

pub mod experiments;

pub mod arm_obstacle_navigation;
pub mod behavior_tree;
pub mod cgmres_nmpc;
pub mod ddp;
pub mod drone_3d_trajectory;
pub mod lqr_control;
pub mod lqr_speed_steer_control;
pub mod lqr_steer_control;
pub mod minimum_snap_trajectory;
pub mod model_predictive_trajectory_generator;
pub mod move_to_pose;
pub mod mpc;
pub mod mpc_control;
pub mod n_joint_arm_3d;
pub mod n_joint_arm_control;
pub mod pure_pursuit;
pub mod rear_wheel_feedback;
pub mod rocket_landing;
pub mod rrt_star_seven_joint_arm;
pub mod stanley_controller;
pub mod state_machine;
pub mod two_joint_arm_control;

pub mod backstepping_control;
pub mod feedback_linearization;
pub mod ilqr;
pub mod pid_controller;
pub mod sliding_mode_control;

// Re-exports
pub use behavior_tree::{BehaviorStatus, BehaviorTree, Blackboard};
pub use lqr_control::InvertedPendulumLQR;
pub use lqr_steer_control::{LQRSteerConfig, LQRSteerController};
pub use move_to_pose::{MoveToPoseConfig, MoveToPoseController};
pub use pure_pursuit::{PurePursuitConfig, PurePursuitController};
pub use rear_wheel_feedback::{RearWheelFeedbackConfig, RearWheelFeedbackController};
pub use stanley_controller::{StanleyConfig, StanleyController};
pub use state_machine::StateMachine;
pub use two_joint_arm_control::TwoJointArm;
