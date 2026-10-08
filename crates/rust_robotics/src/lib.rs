#![forbid(unsafe_code)]
//! RustRobotics — classical robotics algorithms in Rust, from a browser tab to
//! a bare-metal microcontroller.
//!
//! This umbrella crate re-exports the domain crates behind feature flags, so one
//! dependency gives you planning, localization, control, mapping, and SLAM:
//!
//! ```toml
//! [dependencies]
//! rust_robotics = "0.2"   # default: planning, localization, control, mapping, slam
//! ```
//!
//! Try the algorithms interactively first in the
//! [browser playground](https://rsasaki0109.github.io/rust_robotics/playground/),
//! or browse the [gallery](https://rsasaki0109.github.io/rust_robotics/).
//!
//! # Module map
//!
//! | Module | Feature | Crate | Highlights |
//! | --- | --- | --- | --- |
//! | [`core`] / [`prelude`] | always | `rust_robotics_core` | `Point2D`, `Pose2D`, `Path2D`, `Obstacles`, error types, shared traits, SO(2)/SE(2)/SO(3)/SE(3) |
//! | [`optimization`] | always | `rust_robotics_optimization` | Robust Gauss-Newton / Levenberg-Marquardt factor graphs, block-sparse solvers |
//! | `planning` | `planning` | `rust_robotics_planning` | A\*, JPS, Theta\*, D\* Lite, RRT family, PRM, DWA, Hybrid A\*, Frenet, MAPF |
//! | `localization` | `localization` | `rust_robotics_localization` | EKF, UKF, CKF, particle filter, MCL, histogram filter |
//! | `control` | `control` | `rust_robotics_control` | PID, Pure Pursuit, Stanley, LQR, MPC, MPPI, iLQR, Controller Arena |
//! | `mapping` | `mapping` | `rust_robotics_mapping` | Occupancy / Gaussian grid maps, NDT, clustering, shape fitting |
//! | `slam` | `slam` | `rust_robotics_slam` | EKF-SLAM, FastSLAM, ICP, scan-to-map odometry, LiDAR loop closure, pose graphs, IMU preintegration, bundle adjustment |
//! | `viz` | `viz` | `rust_robotics_viz` | gnuplot plotting and pure-Rust GIF recording (`gif`) |
//!
//! # Feature flags
//!
//! | Feature | Enables |
//! | --- | --- |
//! | `planning`, `localization`, `control`, `mapping`, `slam` | The matching domain crate (all on by default). |
//! | `viz` | gnuplot-based visualization (needs a gnuplot binary at run time). |
//! | `gif` | Pure-Rust animated GIF output, no system packages. Implies `viz`. |
//! | `full` | All domain crates plus `viz`. |
//!
//! Use `default-features = false` and pick only the domains you need to keep
//! compile times down. For `no_std` targets depend on `rust_robotics_core`,
//! `rust_robotics_localization`, and `rust_robotics_control` directly with
//! `default-features = false`; see the repository's `docs/embedded_demo.md`.
//!
//! # Examples
//!
//! Plan a grid path with A\*:
//!
//! ```
//! # #[cfg(feature = "planning")]
//! # fn main() -> rust_robotics::prelude::RoboticsResult<()> {
//! use rust_robotics::planning::a_star::{AStarConfig, AStarPlanner};
//! use rust_robotics::prelude::*;
//!
//! let mut obstacles = Obstacles::new();
//! for i in 0..=10 {
//!     let i = i as f64;
//!     obstacles.push(Point2D::new(i, 0.0));
//!     obstacles.push(Point2D::new(i, 10.0));
//!     obstacles.push(Point2D::new(0.0, i));
//!     obstacles.push(Point2D::new(10.0, i));
//! }
//! let planner = AStarPlanner::from_obstacle_points(
//!     &obstacles,
//!     AStarConfig { resolution: 1.0, robot_radius: 0.5, heuristic_weight: 1.0 },
//! )?;
//! let path = planner.plan(Point2D::new(2.0, 2.0), Point2D::new(8.0, 8.0))?;
//! assert!(!path.is_empty());
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "planning"))]
//! # fn main() {}
//! ```
//!
//! Track a position with an EKF:
//!
//! ```
//! # #[cfg(feature = "localization")]
//! # fn main() -> rust_robotics::prelude::RoboticsResult<()> {
//! use rust_robotics::localization::{EKFConfig, EKFLocalizer};
//! use rust_robotics::prelude::*;
//!
//! let mut ekf = EKFLocalizer::with_initial_state_2d(
//!     State2D::new(0.0, 0.0, 0.0, 0.0),
//!     EKFConfig::default(),
//! )?;
//! // Drive forward at 1 m/s and observe the position once per 0.1 s step.
//! for step in 1..=10 {
//!     let measurement = Point2D::new(0.1 * step as f64, 0.0);
//!     ekf.estimate_state(measurement, ControlInput::new(1.0, 0.0), 0.1)?;
//! }
//! assert!((ekf.state_2d().x - 1.0).abs() < 0.2);
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "localization"))]
//! # fn main() {}
//! ```
//!
//! Correct biased odometry with scan-to-map LiDAR matching:
//!
//! ```
//! # #[cfg(feature = "slam")]
//! # fn main() {
//! use nalgebra::Vector2;
//! use rust_robotics::prelude::*;
//! use rust_robotics::slam::scan_to_map::{
//!     ranges_to_points, ray_cast_ranges, LineSegment, ScanToMapConfig, ScanToMapMatcher,
//! };
//!
//! let room = [
//!     ((-5.0, -3.0), (5.0, -3.0)),
//!     ((5.0, -3.0), (5.0, 3.0)),
//!     ((5.0, 3.0), (-5.0, 3.0)),
//!     ((-5.0, 3.0), (-5.0, -3.0)),
//!     ((1.0, 0.5), (2.0, 1.5)),
//! ]
//! .map(|((ax, ay), (bx, by))| LineSegment::new(Vector2::new(ax, ay), Vector2::new(bx, by)));
//! let scan = |pose| ranges_to_points(&ray_cast_ranges(pose, &room, 360, 10.0));
//!
//! let mut matcher = ScanToMapMatcher::new(ScanToMapConfig::default(), Pose2D::origin());
//! matcher.update(Pose2D::origin(), &scan(Pose2D::origin()));
//!
//! // Odometry over-reports a 0.3 m step by 0.1 m; the scan fixes it.
//! let truth = Pose2D::new(0.3, 0.0, 0.0);
//! let update = matcher.update(Pose2D::new(0.4, 0.0, 0.0), &scan(truth));
//! assert!(update.status.is_accepted());
//! assert!((update.corrected_pose.x - truth.x).abs() < 0.01);
//! # }
//! # #[cfg(not(feature = "slam"))]
//! # fn main() {}
//! ```
//!
//! More end-to-end programs live in the repository's `examples/` directory;
//! every `headless_*` example runs without a display and is exercised in CI.

// Always available
pub use rust_robotics_core as core;
pub use rust_robotics_optimization as optimization;

#[cfg(feature = "planning")]
pub use rust_robotics_planning as planning;

#[cfg(feature = "localization")]
pub use rust_robotics_localization as localization;

#[cfg(feature = "control")]
pub use rust_robotics_control as control;

#[cfg(feature = "mapping")]
pub use rust_robotics_mapping as mapping;

#[cfg(feature = "slam")]
pub use rust_robotics_slam as slam;

#[cfg(feature = "viz")]
pub use rust_robotics_viz as viz;

/// Prelude — import commonly used types with `use rust_robotics::prelude::*`.
pub mod prelude {
    pub use rust_robotics_core::*;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_types_accessible() {
        let p = core::Point2D::new(1.0, 2.0);
        assert!((p.x - 1.0).abs() < f64::EPSILON);
        assert!((p.y - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn prelude_reexports_work() {
        use crate::prelude::*;
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        assert!((pose.yaw).abs() < f64::EPSILON);
    }

    #[test]
    #[cfg(feature = "planning")]
    fn planning_module_accessible() {
        let _ = planning::grid::GridMap::new(&[0.0, 10.0], &[0.0, 10.0], 1.0, 0.5);
    }

    #[test]
    #[cfg(feature = "localization")]
    fn localization_module_accessible() {
        let _config = localization::EKFConfig::default();
    }

    #[test]
    #[cfg(feature = "control")]
    fn control_module_accessible() {
        let _config = control::rear_wheel_feedback::RearWheelFeedbackConfig::default();
    }
}
