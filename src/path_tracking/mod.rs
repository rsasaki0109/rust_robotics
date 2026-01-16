// Path Tracking algorithms module

pub mod pure_pursuit;
pub mod stanley_controller;
pub mod lqr_steer_control;
pub mod move_to_pose;
pub mod model_predictive_trajectory_generator;
pub mod mpc;
pub mod rear_wheel_feedback;
pub mod cgmres_nmpc;

// Re-export main controllers with explicit types to avoid name conflicts
pub use pure_pursuit::{PurePursuitController, PurePursuitConfig};
pub use pure_pursuit::VehicleState as PurePursuitVehicleState;
pub use stanley_controller::{StanleyController, StanleyConfig};
pub use stanley_controller::VehicleState as StanleyVehicleState;
pub use lqr_steer_control::{LQRSteerController, LQRSteerConfig, LQRVehicleState};
pub use rear_wheel_feedback::{RearWheelFeedbackController, RearWheelFeedbackConfig};
pub use rear_wheel_feedback::VehicleState as RearWheelFeedbackVehicleState;
pub use mpc::*;
