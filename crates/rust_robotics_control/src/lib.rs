#![forbid(unsafe_code)]
//! Control and path tracking algorithms for the RustRobotics workspace.

pub mod experiments;

pub mod arm_obstacle_navigation;
pub mod behavior_tree;
pub mod cbf_safety_filter;
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
pub mod mppi;
pub mod n_joint_arm_3d;
pub mod n_joint_arm_control;
pub mod person_following_mppi;
pub mod pure_pursuit;
pub mod racing_mppi_3d;
pub mod racing_mppi_motor;
pub mod racing_mppi_powertrain;
pub mod racing_mppi_quadrotor;
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
pub use cbf_safety_filter::{
    simulate_cbf_navigation, CbfConvexObstacle2D, CbfFilterResult, CbfHalfspace2D, CbfNavConfig,
    CbfNavReport, CbfSafetyFilter,
};
pub use lqr_control::InvertedPendulumLQR;
pub use lqr_steer_control::{LQRSteerConfig, LQRSteerController};
pub use move_to_pose::{MoveToPoseConfig, MoveToPoseController};
pub use mppi::{
    MppiCircularObstacle2D, MppiConfig, MppiControl2D, MppiController2D, MppiGateRace2D,
    MppiGateRaceReport2D, MppiMovingObstacle2D, MppiPlan2D, MppiRacingGate2D,
    MppiSamplingDiagnostics2D, MppiState2D, MppiTerminalValueGrid2D,
    MppiTerminalValueReplayBuffer2D, MppiTerminalValueReplayUpdateReport2D,
    MppiTerminalValueUpdateConfig2D, MppiTerminalValueUpdateReport2D, MppiTerminalValueUpdater2D,
    MppiTrackProjection2D, MppiWaypointTrack2D,
};
pub use person_following_mppi::{
    MppiPersonFollowingCandidate2D, MppiPersonFollowingConfig2D, MppiPersonFollowingSampler2D,
    PersonFollowingMetricsAccumulator2D, PersonFollowingMetricsConfig2D,
    PersonFollowingRolloutMetrics2D,
};
pub use pure_pursuit::{PurePursuitConfig, PurePursuitController};
pub use racing_mppi_3d::{
    simulate_lap_race, RacingDroneControl3D, RacingDroneDynamics3D, RacingDroneState3D,
    RacingGateLap3D, RacingGatePlane3D, RacingLapReport3D, RacingMppi3DConfig,
    RacingMppi3DController, RacingMppi3DPlan,
};
pub use racing_mppi_motor::{
    simulate_motor_race, MotorCommand, MotorMppiConfig, MotorMppiController, MotorMppiPlan,
    MotorQuadParams, MotorQuadState, MotorRacingLapReport,
};
pub use racing_mppi_powertrain::{
    simulate_powertrain_race, simulate_powertrain_race_aware, simulate_powertrain_race_budgeted,
    ChargeBudget, PowertrainLapReport, PowertrainMppiController, PowertrainParams, PowertrainState,
};
pub use racing_mppi_quadrotor::{
    simulate_quadrotor_race, QuadrotorControl, QuadrotorLapReport, QuadrotorMppiConfig,
    QuadrotorMppiController, QuadrotorMppiPlan, QuadrotorParams, QuadrotorState,
};
pub use rear_wheel_feedback::{RearWheelFeedbackConfig, RearWheelFeedbackController};
pub use stanley_controller::{StanleyConfig, StanleyController};
pub use state_machine::StateMachine;
pub use two_joint_arm_control::TwoJointArm;
