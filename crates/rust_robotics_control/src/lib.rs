#![forbid(unsafe_code)]
//! Control and path tracking algorithms for the RustRobotics workspace.

pub mod behavior_tree;
pub mod cgmres_nmpc;
pub mod lqr_control;
pub mod lqr_steer_control;
pub mod model_predictive_trajectory_generator;
pub mod move_to_pose;
pub mod mpc;
pub mod mpc_control;
pub mod pure_pursuit;
pub mod rear_wheel_feedback;
pub mod stanley_controller;
pub mod state_machine;
pub mod two_joint_arm_control;
