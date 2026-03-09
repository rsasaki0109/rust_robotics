// Path Tracking algorithms module

pub mod cgmres_nmpc;
pub mod lqr_steer_control;
pub mod model_predictive_trajectory_generator;
pub mod move_to_pose;
pub mod mpc;
pub mod pure_pursuit;
pub mod rear_wheel_feedback;
pub mod stanley_controller;

// Re-export main controllers with explicit types to avoid name conflicts
pub use lqr_steer_control::{LQRSteerConfig, LQRSteerController, LQRVehicleState};
pub use move_to_pose::{MoveToPoseConfig, MoveToPoseController, MoveToPoseResult};
pub use mpc::*;
pub use pure_pursuit::VehicleState as PurePursuitVehicleState;
pub use pure_pursuit::{PurePursuitConfig, PurePursuitController};
pub use rear_wheel_feedback::VehicleState as RearWheelFeedbackVehicleState;
pub use rear_wheel_feedback::{RearWheelFeedbackConfig, RearWheelFeedbackController};
pub use stanley_controller::VehicleState as StanleyVehicleState;
pub use stanley_controller::{StanleyConfig, StanleyController};
