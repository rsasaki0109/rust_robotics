# RustRobotics

[![CI](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/rsasaki0109/rust_robotics/branch/main/graph/badge.svg)](https://codecov.io/gh/rsasaki0109/rust_robotics)
[![crates.io](https://img.shields.io/crates/v/rust_robotics.svg)](https://crates.io/crates/rust_robotics)
[![docs.rs](https://docs.rs/rust_robotics/badge.svg)](https://docs.rs/rust_robotics)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/rust_robotics/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Robotics algorithms in Rust: planning, localization, SLAM, control, and manipulation.
Try them in your browser, then use the same library in your own project.

<p align="center">
  <a href="https://rsasaki0109.github.io/rust_robotics/playground/"><b>Open the playground</b></a>
  ·
  <a href="#quick-start"><b>Run locally</b></a>
  ·
  <a href="#algorithm-gallery"><b>Algorithm gallery</b></a>
  ·
  <a href="https://docs.rs/rust_robotics"><b>API docs</b></a>
</p>

## Try in your browser

Edit a map, move a robot, or inspect an algorithm step by step.
Pause, replay, and share your experiment. No installation or account required.

<table>
  <tr>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=grid"><img src="docs/assets/playground/grid-search.png" width="260" alt="Grid planner search replay"/></a><br/><b>Grid planners</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=localization&amp;loc_view=belief&amp;belief_event=resample"><img src="docs/assets/playground/belief-lab.png" width="260" alt="Particle filter belief update"/></a><br/><b>Localization</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=slam&amp;algorithm=loop"><img src="docs/assets/playground/slam-loop.png" width="260" alt="SLAM loop closure comparison"/></a><br/><b>SLAM</b></td>
  </tr>
  <tr>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=arena"><img src="docs/assets/playground/arena.png" width="260" alt="Path tracking controller comparison"/></a><br/><b>Controller Arena</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=arm"><img src="docs/assets/playground/arm.png" width="260" alt="Two-link robot arm inverse kinematics"/></a><br/><b>Arm Lab</b></td>
    <td align="center"><a href="https://rsasaki0109.github.io/rust_robotics/playground/?tab=drone&amp;mode=manual&amp;course=climb&amp;cinema=1&amp;intro=1"><img src="docs/assets/playground/drone-v1.png" width="260" alt="Manual drone flight beside an autonomous ghost"/></a><br/><b>Flight Lab</b></td>
  </tr>
</table>

[Explore all experiments](https://rsasaki0109.github.io/rust_robotics/)
· [Playground guide](crates/rust_robotics_playground/README.md)

### Flight Lab

[![Play Flight Lab v1 against an autonomous drone ghost](docs/assets/playground/drone-v1.png)](https://rsasaki0109.github.io/rust_robotics/playground/?tab=drone&mode=manual&course=climb&cinema=1&intro=1)

[Flight recording](docs/assets/drone-manual.mp4) ·
[Course editor](docs/assets/drone-editor.mp4) ·
[Flight photograph](docs/assets/drone-photo-wide.png)

## Quick Start

Clone the repository and launch the native playground:

```bash
git clone https://github.com/rsasaki0109/rust_robotics.git
cd rust_robotics
cargo run -p rust_robotics_playground
```

### Rust navigation loop

Run planning, localization, and path tracking together without a GUI:

```bash
cargo run -p rust_robotics --example headless_navigation_loop \
  --features "planning,localization,control"
```

<img src="./docs/assets/playground/arena.png" width="900" alt="Controller Arena inspecting Pure Pursuit, Stanley, and LQR Steer at the same simulation time">

[Controller inspector guide](docs/controller_inspector.md)

## Use As A Library

```bash
cargo add rust_robotics --features "planning,localization,control"
```

```rust
use rust_robotics::planning::{AStarConfig, AStarPlanner};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::control::PurePursuitController;
```

See the [Rust guide](docs/rust_guide.md) for workspace crates, more examples,
build instructions, and benchmarks.

## Embedded / no_std

Core types and Kalman filters support `no_std` with `alloc`:

```bash
rustup target add thumbv7em-none-eabihf
cargo build -p rust_robotics_core -p rust_robotics_localization \
  --no-default-features --target thumbv7em-none-eabihf
```

[Run the embedded demo](docs/embedded_demo.md) with EKF, Pure Pursuit, and PID
on Cortex-M under QEMU.

## Algorithm gallery

Robotics algorithms in action. [Explanations, source, and commands](docs/algorithm_gallery.md).

<table>
  <tr>
    <td align="center"><a href="#rapidly-exploring-random-trees-rrt"><img src="./media/gallery/rrt.gif" width="260" alt="RRT tree growth"/></a><br/><b>RRT</b></td>
    <td align="center"><a href="#dynamic-window-approach"><img src="./media/gallery/dwa.gif" width="260" alt="Dynamic Window Approach"/></a><br/><b>Dynamic Window Approach</b></td>
    <td align="center"><a href="#dijkstra-algorithm"><img src="./media/dijkstra-motion-planner.gif" width="260" alt="Dijkstra motion planner"/></a><br/><b>Dijkstra</b></td>
  </tr>
  <tr>
    <td align="center"><a href="#extended-kalman-filter-localization"><img src="./media/gallery/ekf_localization.gif" width="260" alt="EKF localization"/></a><br/><b>EKF Localization</b></td>
    <td align="center"><a href="#particle-filter-localization"><img src="./media/gallery/particle_filter.gif" width="260" alt="Particle filter localization"/></a><br/><b>Particle Filter</b></td>
    <td align="center"><a href="#pure-pursuit"><img src="./media/gallery/pure_pursuit.gif" width="260" alt="Pure pursuit path tracking"/></a><br/><b>Pure Pursuit</b></td>
  </tr>
  <tr>
    <td align="center"><a href="#ekf-slam"><img src="./media/gallery/ekf_slam.gif" width="260" alt="EKF SLAM"/></a><br/><b>EKF SLAM</b></td>
    <td align="center"><a href="#fastslam-10"><img src="./media/gallery/fastslam.gif" width="260" alt="FastSLAM 1.0"/></a><br/><b>FastSLAM 1.0</b></td>
    <td align="center"><a href="#iterative-closest-point-icp-matching"><img src="./media/gallery/icp_matching.gif" width="260" alt="ICP scan matching"/></a><br/><b>ICP Matching</b></td>
  </tr>
  <tr>
    <td colspan="3" align="center"><a href="#pose-graph-optimization"><img src="./media/gallery/factor_graph_optimization.gif" width="540" alt="Block-sparse factor graph optimization"/></a><br/><b>Block-sparse Factor Graph Optimization</b></td>
  </tr>
  <tr>
    <td colspan="3" align="center"><a href="#loop-closure-lidar-graph-slam"><img src="./media/gallery/lidar_loop_closure.gif" width="540" alt="LiDAR graph SLAM loop closure on a corridor loop"/></a><br/><b>LiDAR Loop Closure (scan-to-map + pose graph)</b></td>
  </tr>
</table>

[Regenerate the animations](scripts/generate_gallery_gifs.sh).

### Localization

#### Extended Kalman Filter Localization

<img src="./media/gallery/ekf_localization.gif" width="640px">

[Source](./crates/rust_robotics_localization/src/ekf.rs) · [Details](docs/algorithm_gallery.md#extended-kalman-filter-localization)

#### Particle Filter Localization

<img src="./media/gallery/particle_filter.gif" width="640px">

[Source](./crates/rust_robotics_localization/src/particle_filter.rs) · [Details](docs/algorithm_gallery.md#particle-filter-localization)

#### Unscented Kalman Filter Localization

<img src="./img/localization/ukf_result.png" width="640px">

[Source](./crates/rust_robotics_localization/src/unscented_kalman_filter.rs) · [Details](docs/algorithm_gallery.md#unscented-kalman-filter-localization)

#### Histogram Filter Localization

<img src="./img/localization/histogram_filter.svg" width="640px">

[Source](./crates/rust_robotics_localization/src/histogram_filter.rs) · [Details](docs/algorithm_gallery.md#histogram-filter-localization)

### Mapping

#### NDT Map

<img src="./img/mapping/ndt.svg" width="640px">

[Source](./crates/rust_robotics_mapping/src/ndt.rs) · [Details](docs/algorithm_gallery.md#ndt-map)

#### Gaussian Grid Map

<img src="./img/mapping/gaussian_grid_map.svg" width="640px">

[Source](./crates/rust_robotics_mapping/src/gaussian_grid_map.rs) · [Details](docs/algorithm_gallery.md#gaussian-grid-map)

#### Ray Casting Grid Map

<img src="./img/mapping/ray_casting_grid_map.svg" width="640px">

[Source](./crates/rust_robotics_mapping/src/ray_casting_grid_map.rs) · [Details](docs/algorithm_gallery.md#ray-casting-grid-map)

### SLAM

#### Iterative Closest Point (ICP) Matching

<img src="./media/gallery/icp_matching.gif" width="640px">

[Source](./crates/rust_robotics_slam/src/icp_matching.rs) · [Details](docs/algorithm_gallery.md#iterative-closest-point-icp-matching)

#### Scan-to-Map LiDAR Odometry

##### Loop Closure (LiDAR Graph SLAM)

<img src="./media/gallery/lidar_loop_closure.gif" width="640px">

[Details](docs/algorithm_gallery.md#loop-closure-lidar-graph-slam)

#### FastSLAM 1.0

<img src="./media/gallery/fastslam.gif" width="640px">

[Source](./crates/rust_robotics_slam/src/fastslam1.rs) · [Details](docs/algorithm_gallery.md#fastslam-10)

#### EKF SLAM

<img src="./media/gallery/ekf_slam.gif" width="640px">

[Source](./crates/rust_robotics_slam/src/ekf_slam.rs) · [Details](docs/algorithm_gallery.md#ekf-slam)

#### Graph-Based SLAM

<img src="./img/slam/graph_based_slam.svg" width="640px">

[Source](./crates/rust_robotics_slam/src/graph_based_slam.rs) · [Details](docs/algorithm_gallery.md#graph-based-slam)

#### Pose Graph Optimization

<img src="./docs/assets/factor-graph-optimization.svg" width="1000px" alt="Pose graph before and after block-sparse optimization">

[Source](./crates/rust_robotics_slam/src/pose_graph_optimization.rs) · [Details](docs/algorithm_gallery.md#pose-graph-optimization)

#### EuRoC / KITTI Dataset Replay

<img src="./docs/assets/euroc-vio-pipeline.svg" width="1000px" alt="EuRoC IMU, bundle adjustment, and SE3 pose graph pipeline">

[Details](docs/algorithm_gallery.md#euroc--kitti-dataset-replay)

### Path Planning

#### A* Algorithm

<img src="./img/path_planning/a_star_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/a_star.rs) · [Details](docs/algorithm_gallery.md#a-algorithm)

#### Theta* Algorithm

<img src="./img/path_planning/theta_star_result.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/theta_star.rs) · [Details](docs/algorithm_gallery.md#theta-algorithm)

#### Jump Point Search (JPS)

<img src="./img/path_planning/jps_result.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/jps.rs) · [Details](docs/algorithm_gallery.md#jump-point-search-jps)

#### Bezier Path Planning

<img src="./img/path_planning/bezier_path_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/bezier_path_planning.rs) · [Details](docs/algorithm_gallery.md#bezier-path-planning)

#### Cubic Spline

<img src="./img/path_planning/csp.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/cubic_spline_planner.rs) · [Details](docs/algorithm_gallery.md#cubic-spline)

#### Dynamic Window Approach

<img src="./media/gallery/dwa.gif" width="640px">

[Source](./crates/rust_robotics_planning/src/dwa.rs) · [Details](docs/algorithm_gallery.md#dynamic-window-approach)

#### D* Lite

<img src="./img/path_planning/d_star_lite_result.png" width="640">

[Source](./crates/rust_robotics_planning/src/d_star_lite.rs) · [Details](docs/algorithm_gallery.md#d-lite)

#### Dijkstra Algorithm

<img src="./media/dijkstra-motion-planner.gif" width="640px">

[Source](./crates/rust_robotics_planning/src/dijkstra.rs) · [Details](docs/algorithm_gallery.md#dijkstra-algorithm)

#### Informed RRT*

<img src="./img/path_planning/informed_rrt_star_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/informed_rrt_star.rs) · [Details](docs/algorithm_gallery.md#informed-rrt)

#### Model Predictive Trajectory Generator

<img src="./img/path_tracking/model_predictive_trajectory_generator.svg" width="640px">

[Source](./crates/rust_robotics_control/src/model_predictive_trajectory_generator.rs) · [Details](docs/algorithm_gallery.md#model-predictive-trajectory-generator)

#### Potential Field Algorithm

<img src="./img/path_planning/potential_field_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/potential_field.rs) · [Details](docs/algorithm_gallery.md#potential-field-algorithm)

#### Quintic Polynomials

<img src="./img/path_planning/quintic_polynomials_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/quintic_polynomials.rs) · [Details](docs/algorithm_gallery.md#quintic-polynomials)

#### Rapidly-Exploring Random Trees (RRT)

<img src="./media/gallery/rrt.gif" width="640px">

[Source](./crates/rust_robotics_planning/src/rrt.rs) · [Details](docs/algorithm_gallery.md#rapidly-exploring-random-trees-rrt)

#### RRT*

<img src="./img/path_planning/rrt_star_result.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/rrt_star.rs) · [Details](docs/algorithm_gallery.md#rrt)

#### Hybrid A*

<img src="./docs/assets/playground/parking.png" width="640px" alt="Hybrid A* parking a car in the playground">

[Source](./crates/rust_robotics_planning/src/hybrid_a_star.rs) · [Details](docs/algorithm_gallery.md#hybrid-a)

#### Reeds-Shepp Path

<img src="./img/path_planning/reeds_shepp_result.png" width="640px">

[Source](./crates/rust_robotics_planning/src/reeds_shepp_path.rs) · [Details](docs/algorithm_gallery.md#reeds-shepp-path)

#### PRM (Probabilistic Road-Map)

<img src="./img/path_planning/prm.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/prm.rs) · [Details](docs/algorithm_gallery.md#prm-probabilistic-road-map)

#### Voronoi Road-Map

<img src="./img/path_planning/voronoi_road_map.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/voronoi_road_map.rs) · [Details](docs/algorithm_gallery.md#voronoi-road-map)

#### Frenet Optimal Trajectory

<img src="./img/path_planning/frenet_optimal_trajectory.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/frenet_optimal_trajectory.rs) · [Details](docs/algorithm_gallery.md#frenet-optimal-trajectory)

#### State Lattice Planner

<img src="./img/path_planning/state_lattice_plan.svg" width="640px">

[Source](./crates/rust_robotics_planning/src/state_lattice/) · [Details](docs/algorithm_gallery.md#state-lattice-planner)

### Path Tracking

#### LQR Steer Control

<img src="./img/path_tracking/lqr_steer_control.png" width="640px">

[Source](./crates/rust_robotics_control/src/lqr_steer_control.rs) · [Details](docs/algorithm_gallery.md#lqr-steer-control)

#### Move to Pose

<img src="./img/path_tracking/move_to_pose.png" width="640px">

[Source](./crates/rust_robotics_control/src/move_to_pose.rs) · [Details](docs/algorithm_gallery.md#move-to-pose)

#### Pure Pursuit

<img src="./media/gallery/pure_pursuit.gif" width="640px">

[Source](./crates/rust_robotics_control/src/pure_pursuit.rs) · [Details](docs/algorithm_gallery.md#pure-pursuit)

#### Stanley Control

<img src="./img/path_tracking/stanley_controller.png" width="640px">

[Source](./crates/rust_robotics_control/src/stanley_controller.rs) · [Details](docs/algorithm_gallery.md#stanley-control)

#### Rear Wheel Feedback Control

<img src="./img/path_tracking/rear_wheel_feedback.svg" width="640px">

[Source](./crates/rust_robotics_control/src/rear_wheel_feedback.rs) · [Details](docs/algorithm_gallery.md#rear-wheel-feedback-control)

#### MPC (Model Predictive Control)

<img src="./img/path_tracking/mpc.svg" width="640px">

[Source](./crates/rust_robotics_control/src/mpc.rs) · [Details](docs/algorithm_gallery.md#mpc-model-predictive-control)

### Inverted Pendulum

#### LQR Control

<img src="./img/inverted_pendulum/inverted_pendulum_lqr.png" width="640px">

[Source](./crates/rust_robotics_control/src/lqr_control.rs) · [Details](docs/algorithm_gallery.md#lqr-control)

### Arm Navigation

#### Two Joint Arm Control

<img src="./img/arm_navigation/two_joint_arm_control.png" width="640px">

[Source](./crates/rust_robotics_control/src/two_joint_arm_control.rs) · [Details](docs/algorithm_gallery.md#two-joint-arm-control)

### Mission Planning

#### State Machine

<img src="./img/mission_planning/state_machine_diagram.svg" width="640px">

[Source](./crates/rust_robotics_control/src/state_machine.rs) · [Details](docs/algorithm_gallery.md#state-machine)

## Workspace

```
crates/
├── rust_robotics_core/          — Core types, traits, errors, Lie groups
├── rust_robotics_optimization/  — Factor graphs, robust losses, Gauss-Newton/LM
├── rust_robotics_planning/      — Path planning (A*, DWA, RRT, PRM, etc.)
├── rust_robotics_localization/  — Localization (EKF, UKF, PF, Histogram)
├── rust_robotics_control/       — Control & path tracking (Pure Pursuit, LQR, MPC, etc.)
├── rust_robotics_mapping/       — Mapping (NDT, Gaussian Grid, IMLS)
├── rust_robotics_slam/          — SLAM, pose graphs, IMU, BA, geometric ICP
├── rust_robotics_viz/           — Visualization (gnuplot wrapper)
├── rust_robotics_playground/    — Native and browser playground
└── rust_robotics/               — Umbrella crate (feature-gated re-exports)

ros2_nodes/                     — ROS2 navigation nodes (safe_drive-based)
```

## ROS2 Integration

```text
                    TurtleBot3 Gazebo
                 /scan  /odom  /cmd_vel
                    |      |       ^
                    v      |       |
               +-----------+       |
               | slam_node | ----- +
               +-----------+    /map
                      |
                      v
             +-------------------+
             | path_planner_node | ---> /planned_path
             +-------------------+             |
                ^           ^                  v
                |           |           +--------------+
         /ekf_odom     /goal_pose ---> | dwa_planner |
                ^                      +--------------+
                |
      +----------------------+
      | ekf_localizer_node   |
      +----------------------+
                ^
                |
      +-------------------------+
      | waypoint_navigator_node |
      +-------------------------+
```

[Gazebo demo](docs/gazebo_demo.mp4) · [Setup and topics](docs/ros2_integration.md)

## Documentation

- [Algorithm gallery](docs/algorithm_gallery.md) — animations, source, and commands.
- [Rust guide](docs/rust_guide.md) — library APIs, examples, and benchmarks.
- [Playground guide](crates/rust_robotics_playground/README.md) — controls and browser builds.
- [ROS2 / Gazebo](docs/ros2_integration.md) · [dora-rs](docs/dora_integration.md).
- [Datasets and VIO](docs/datasets.md) · [Interfaces](docs/interfaces.md).
- [Pose-observation loss experiment](experiments/pose_observation_loss/README.md).

## Community

[Contributions](CONTRIBUTING.md) and [algorithm requests](.github/ISSUE_TEMPLATE/algorithm_request.yml)
are welcome. See the [security policy](SECURITY.md) and [MIT license](LICENSE).

Inspired by [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and
[MathematicalRobotics](https://github.com/scomup/MathematicalRobotics).
Attributions are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
