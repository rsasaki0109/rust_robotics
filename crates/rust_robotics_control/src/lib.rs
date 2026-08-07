#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]
//! Control and path tracking algorithms for the RustRobotics workspace.
//!
//! With default features disabled this crate is `no_std` (plus `alloc`) and
//! the Tier 1 controllers — PID, Pure Pursuit, Stanley, and LQR Steer — build
//! for bare-metal targets such as `thumbv7em-none-eabihf`. The sampling and
//! optimization-based controllers (MPPI, MPC, cgmres, pusher-slider, racing,
//! consensus ADMM, ...) need `std` for random number generation or the clarabel
//! solver and are compiled only when the `std` feature is enabled.

#[macro_use]
extern crate alloc;

#[cfg(feature = "std")]
pub mod experiments;

#[cfg(feature = "std")]
pub mod admm_consensus;
#[cfg(feature = "std")]
pub mod arm_obstacle_navigation;
#[cfg(feature = "std")]
pub mod backstepping_control;
#[cfg(feature = "std")]
pub mod behavior_tree;
#[cfg(feature = "std")]
pub mod cbf_safety_filter;
#[cfg(feature = "std")]
pub mod cgmres_nmpc;
#[cfg(feature = "std")]
pub mod controller_arena;
#[cfg(feature = "std")]
pub mod ddp;
#[cfg(feature = "std")]
pub mod drone_3d_trajectory;
#[cfg(feature = "std")]
pub mod feedback_linearization;
#[cfg(feature = "std")]
pub mod ilqr;
#[cfg(feature = "std")]
pub mod lqr_control;
#[cfg(feature = "std")]
pub mod lqr_speed_steer_control;
pub mod lqr_steer_control;
#[cfg(feature = "std")]
pub mod meta_control;
#[cfg(feature = "std")]
pub mod minimum_snap_trajectory;
#[cfg(feature = "std")]
pub mod model_predictive_trajectory_generator;
#[cfg(feature = "std")]
pub mod move_to_pose;
#[cfg(feature = "std")]
pub mod mpc;
#[cfg(feature = "std")]
pub mod mpc_control;
#[cfg(feature = "std")]
pub mod mppi;
#[cfg(feature = "std")]
pub mod n_joint_arm_3d;
#[cfg(feature = "std")]
pub mod n_joint_arm_control;
#[cfg(feature = "std")]
pub mod person_following_mppi;
pub mod pid_controller;
pub mod pure_pursuit;
#[cfg(feature = "std")]
pub mod pusher_slider;
#[cfg(feature = "std")]
pub mod racing_mppi_3d;
#[cfg(feature = "std")]
pub mod racing_mppi_motor;
#[cfg(feature = "std")]
pub mod racing_mppi_powertrain;
#[cfg(feature = "std")]
pub mod racing_mppi_quadrotor;
#[cfg(feature = "std")]
pub mod rear_wheel_feedback;
#[cfg(feature = "std")]
pub mod rocket_landing;
#[cfg(feature = "std")]
pub mod rrt_star_seven_joint_arm;
#[cfg(feature = "std")]
pub mod sliding_mode_control;
pub mod stanley_controller;
#[cfg(feature = "std")]
pub mod state_machine;
#[cfg(feature = "std")]
pub mod two_joint_arm_control;

// Re-exports
#[cfg(feature = "std")]
pub use admm_consensus::{
    solve_formation_consensus, solve_graph_consensus, solve_horizon_consensus, AdmmConfig,
    AdmmReport, AgentSpec, AgentTrajectory, GraphConsensusReport, HorizonConsensusReport,
};
#[cfg(feature = "std")]
pub use behavior_tree::{BehaviorStatus, BehaviorTree, Blackboard};
#[cfg(feature = "std")]
pub use cbf_safety_filter::{
    simulate_cbf_navigation, CbfConvexObstacle2D, CbfFilterResult, CbfHalfspace2D, CbfNavConfig,
    CbfNavReport, CbfSafetyFilter,
};
#[cfg(feature = "std")]
pub use controller_arena::{
    metrics_from_samples, run_controller_arena, ArenaControllerKind, ArenaMetrics, ArenaPreset,
    ArenaRun, ArenaSample, ArenaScenario,
};
#[cfg(feature = "std")]
pub use lqr_control::InvertedPendulumLQR;
pub use lqr_steer_control::{LQRSteerConfig, LQRSteerController};
#[cfg(feature = "std")]
pub use meta_control::{
    run_meta_control, MetaControlPolicy, MetaControlRun, MetaControlSample, MetaController,
};
#[cfg(feature = "std")]
pub use move_to_pose::{MoveToPoseConfig, MoveToPoseController};
#[cfg(feature = "std")]
pub use mppi::{
    MppiCircularObstacle2D, MppiConfig, MppiControl2D, MppiController2D, MppiGateRace2D,
    MppiGateRaceReport2D, MppiMovingObstacle2D, MppiPlan2D, MppiRacingGate2D,
    MppiSamplingDiagnostics2D, MppiState2D, MppiTerminalValueGrid2D,
    MppiTerminalValueReplayBuffer2D, MppiTerminalValueReplayUpdateReport2D,
    MppiTerminalValueUpdateConfig2D, MppiTerminalValueUpdateReport2D, MppiTerminalValueUpdater2D,
    MppiTrackProjection2D, MppiWaypointTrack2D,
};
#[cfg(feature = "std")]
pub use person_following_mppi::{
    MppiPersonFollowingCandidate2D, MppiPersonFollowingConfig2D, MppiPersonFollowingSampler2D,
    PersonFollowingMetricsAccumulator2D, PersonFollowingMetricsConfig2D,
    PersonFollowingRolloutMetrics2D,
};
pub use pid_controller::{PIDConfig, PIDController};
pub use pure_pursuit::{PurePursuitConfig, PurePursuitController};
#[cfg(feature = "std")]
pub use pusher_slider::{
    simulate_multi_push, simulate_push, simulate_push_with_obstacles, ContactMode, MultiPushReport,
    PushReport, PusherCommand, PusherMppiConfig, PusherMppiPlan, PusherSliderMppiController,
    PusherSliderParams, SliderState,
};
#[cfg(feature = "std")]
pub use racing_mppi_3d::{
    simulate_lap_race, RacingDroneControl3D, RacingDroneDynamics3D, RacingDroneState3D,
    RacingGateLap3D, RacingGatePlane3D, RacingLapReport3D, RacingMppi3DConfig,
    RacingMppi3DController, RacingMppi3DPlan,
};
#[cfg(feature = "std")]
pub use racing_mppi_motor::{
    simulate_motor_race, MotorCommand, MotorMppiConfig, MotorMppiController, MotorMppiPlan,
    MotorQuadParams, MotorQuadState, MotorRacingLapReport,
};
#[cfg(feature = "std")]
pub use racing_mppi_powertrain::{
    simulate_powertrain_race, simulate_powertrain_race_aware, simulate_powertrain_race_budgeted,
    ChargeBudget, PowertrainLapReport, PowertrainMppiController, PowertrainParams, PowertrainState,
};
#[cfg(feature = "std")]
pub use racing_mppi_quadrotor::{
    simulate_quadrotor_race, QuadrotorControl, QuadrotorLapReport, QuadrotorMppiConfig,
    QuadrotorMppiController, QuadrotorMppiPlan, QuadrotorParams, QuadrotorState,
};
#[cfg(feature = "std")]
pub use rear_wheel_feedback::{RearWheelFeedbackConfig, RearWheelFeedbackController};
pub use stanley_controller::{StanleyConfig, StanleyController};
#[cfg(feature = "std")]
pub use state_machine::StateMachine;
#[cfg(feature = "std")]
pub use two_joint_arm_control::TwoJointArm;
