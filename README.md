RustRobotics
====

[![CI](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/rust_robotics/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/rsasaki0109/rust_robotics/branch/main/graph/badge.svg)](https://codecov.io/gh/rsasaki0109/rust_robotics)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/rust_robotics/)

This package is a rust implementation of [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics).

## Build

```bash
git clone https://github.com/rsasaki0109/RustRobotics.git
cd RustRobotics
cargo build --workspace
cargo test --workspace
```

## Workspace Structure

```
crates/
├── rust_robotics_core/          — Core types, traits, errors
├── rust_robotics_planning/      — Path planning (A*, DWA, RRT, PRM, etc.)
├── rust_robotics_localization/  — Localization (EKF, UKF, PF, Histogram)
├── rust_robotics_control/       — Control & path tracking (Pure Pursuit, LQR, MPC, etc.)
├── rust_robotics_mapping/       — Mapping (NDT, Gaussian Grid, Ray Casting)
├── rust_robotics_slam/          — SLAM (EKF-SLAM, FastSLAM, Graph SLAM, ICP)
├── rust_robotics_viz/           — Visualization (gnuplot wrapper)
└── rust_robotics/               — Umbrella crate (feature-gated re-exports)
```

## Usage as Library

```toml
# Use the full library
[dependencies]
rust_robotics = "0.1"

# Or use individual crates
[dependencies]
rust_robotics_planning = "0.1"
rust_robotics_localization = "0.1"
```

Types are re-exported at the crate root for convenience:

```rust
use rust_robotics_planning::{AStarPlanner, AStarConfig, DWAPlanner};
use rust_robotics_localization::{EKFConfig, EKFLocalizer};
use rust_robotics_control::{PurePursuitController, StanleyController};
```

## Run (Example)

```bash
# Headless (no GUI dependencies)
cargo run -p rust_robotics --example headless_grid_planners --features planning
cargo run -p rust_robotics --example headless_localizers --features localization
cargo run -p rust_robotics --example headless_navigation_loop --features "planning,localization,control"

# Visualization (requires gnuplot)
cargo run -p rust_robotics --example a_star --features "planning,viz"
cargo run -p rust_robotics --example jps --features "planning,viz"
cargo run -p rust_robotics --example rear_wheel_feedback --features "control,viz"
```

## Benchmarks

### Rust vs Python Speed Comparison

| Algorithm | Rust (ms) | Python (ms) | Speedup |
|---|---:|---:|---:|
| A* (100x100) | 4.0 | 924.5 | 231x |
| EKF (1000 steps) | 0.19 | 103.1 | 543x |
| RRT (100 runs) | 0.12 | 5.7 | 46x |
| CubicSpline (1000 runs) | 0.92 | 6.9 | 7.5x |

### Any-Angle Planner Comparison (160 MovingAI scenarios)

| Planner | Path Quality vs Theta* | Speed vs Theta* |
|---|---|---|
| Theta* | baseline | baseline |
| Lazy Theta* | same (+0.01%) | **1.7x faster** (p=0.025) |
| A*+optimize_path | same (+0.27%) | **2.3x faster** |

### Grid Planner Benchmark (50x50)

```bash
cargo bench -p rust_robotics_planning --bench unified_planning_benchmark
cargo bench -p rust_robotics_planning --bench jps_crossover_benchmark
```

# Table of Contents
   * [Localization](#localization)
      * [Extended Kalman Filter](#extended-kalman-filter-localization)
      * [Particle Filter](#particle-filter-localization)
      * [Unscented Kalman Filter](#unscented-kalman-filter-localization)
      * [Histogram Filter](#histogram-filter-localization)
      * [Cubature Kalman Filter](#cubature-kalman-filter)
      * [Ensemble Kalman Filter](#ensemble-kalman-filter)
      * [Adaptive Filter (EKF/CKF)](#adaptive-filter)
   * [Mapping](#mapping)
      * [NDT Map](#ndt-map)
      * [Gaussian Grid Map](#gaussian-grid-map)
      * [Ray Casting Grid Map](#ray-casting-grid-map)
      * [Lidar to Grid Map](#lidar-to-grid-map)
      * [Distance Map](#distance-map)
      * [Circle Fitting](#circle-fitting)
      * [Rectangle Fitting](#rectangle-fitting)
      * [K-Means Clustering](#k-means-clustering)
      * [Normal Vector Estimation](#normal-vector-estimation)
      * [Point Cloud Sampling](#point-cloud-sampling)
   * [SLAM](#slam)
      * [Iterative Closest Point](#iterative-closest-point-icp-matching)
      * [FastSLAM 1.0](#fastslam-10)
      * [EKF SLAM](#ekf-slam)
      * [Graph-Based SLAM](#graph-based-slam)
   * [Path Planning](#path-planning)
      * [A*](#a-algorithm), [Theta*](#theta-algorithm), [Lazy Theta*](#lazy-theta), [Enhanced Lazy Theta*](#enhanced-lazy-theta), [JPS](#jump-point-search-jps), [Dijkstra](#dijkstra-algorithm), [D* Lite](#d-lite), [D*](#d-algorithm), [Anya](#anya-optimal-any-angle)
      * [BFS](#breadth-first-search), [DFS](#depth-first-search), [Greedy Best-First](#greedy-best-first-search)
      * [Bidirectional A*](#bidirectional-a), [Bidirectional BFS](#bidirectional-bfs)
      * [Flow Field](#flow-field), [Bug Planning](#bug-planning)
      * [RRT](#rapidly-exploring-random-trees-rrt), [RRT*](#rrt), [Informed RRT*](#informed-rrt), [Batch Informed RRT*](#batch-informed-rrt)
      * [RRT-Dubins](#rrt-dubins), [RRT*-Dubins](#rrt-dubins-1), [RRT*-Reeds-Shepp](#rrt-reeds-shepp)
      * [Closed-Loop RRT*](#closed-loop-rrt), [LQR-RRT*](#lqr-rrt), [BIT*](#bit)
      * [Dubins Path](#dubins-path), [Reeds-Shepp Path](#reeds-shepp-path)
      * [Bezier Path](#bezier-path-planning), [B-Spline](#b-spline-path), [Catmull-Rom](#catmull-rom-spline), [Eta3 Spline](#eta3-spline)
      * [Cubic Spline](#cubic-spline), [Quintic Polynomials](#quintic-polynomials), [Clothoid Path](#clothoid-path)
      * [DWA](#dynamic-window-approach), [Potential Field](#potential-field-algorithm), [LQR Planner](#lqr-planner)
      * [PRM](#prm-probabilistic-road-map), [Voronoi Road-Map](#voronoi-road-map), [Visibility Road-Map](#visibility-road-map)
      * [Frenet Optimal Trajectory](#frenet-optimal-trajectory), [State Lattice](#state-lattice-planner)
      * [Elastic Bands](#elastic-bands), [Dynamic Movement Primitives](#dynamic-movement-primitives)
      * [PSO](#particle-swarm-optimization), [Time-Based Planning](#time-based-path-planning)
      * [Model Predictive Trajectory Generator](#model-predictive-trajectory-generator)
      * [Path Smoothing](#path-smoothing)
      * [Coverage: Grid-Based Sweep](#grid-based-sweep-cpp), [Wavefront](#wavefront-cpp), [Spiral Spanning Tree](#spiral-spanning-tree-cpp)
   * [Path Tracking](#path-tracking)
      * [LQR Steer Control](#lqr-steer-control), [LQR Speed+Steer](#lqr-speed-steer-control)
      * [Move to Pose](#move-to-pose), [Pure Pursuit](#pure-pursuit), [Stanley](#stanley-control), [Rear Wheel Feedback](#rear-wheel-feedback-control)
      * [MPC (Model Predictive Control)](#mpc-model-predictive-control)
   * [Inverted Pendulum](#inverted-pendulum)
      * [LQR Control](#lqr-control)
   * [Arm Navigation](#arm-navigation)
      * [Two Joint Arm](#two-joint-arm-control), [N-Joint Arm IK](#n-joint-arm-control), [Arm Obstacle Navigation](#arm-obstacle-navigation)
   * [Aerial Navigation](#aerial-navigation)
      * [3D Grid A*](#3d-grid-a), [Drone 3D Trajectory Following](#drone-3d-trajectory-following)
   * [Mission Planning](#mission-planning)
      * [Behavior Tree](#behavior-tree), [State Machine](#state-machine)

# Localization
## Extended Kalman Filter Localization

<img src="./img/localization/ekf.svg" width="640px">


Red:GPS, Brue:Ground Truth, Green:EKF, Yellow:Dead Reckoning

- [src](./crates/rust_robotics_localization/src/ekf.rs)

## Particle Filter Localization

<img src="./img/localization/particle_filter_result.png" width="640px">

Blue: GPS, Red: Ground Truth, Green: Particle Filter, Yellow: Dead Reckoning

- [src](./crates/rust_robotics_localization/src/particle_filter.rs)

## Unscented Kalman Filter Localization

<img src="./img/localization/ukf_result.png" width="640px">

Blue: Ground Truth, Red: UKF Estimate, Black: Dead Reckoning, Green: GPS Observations, Red Ellipse: Uncertainty

- [src](./crates/rust_robotics_localization/src/unscented_kalman_filter.rs)

## Histogram Filter Localization

<img src="./img/localization/histogram_filter.svg" width="640px">

Grid-based probabilistic localization using RFID landmarks. The algorithm maintains a probability distribution over a 2D grid and updates it based on motion and observations.

Blue: True path, Orange: Dead Reckoning, Green: Histogram Filter estimate, Black: RFID landmarks

- [src](./crates/rust_robotics_localization/src/histogram_filter.rs)

## Cubature Kalman Filter

Cubature Kalman Filter (CKF) using 3rd-degree spherical-radial cubature rule. Achieves the same accuracy as UKF but 30% faster with zero tuning parameters (no alpha/beta/kappa). Recommended as the default over UKF for typical robotics scenarios.

- [src](./crates/rust_robotics_localization/src/cubature_kalman_filter.rs)

## Ensemble Kalman Filter

Stochastic ensemble-based Kalman filter. Maintains an ensemble of state particles and updates them using the Kalman gain computed from ensemble statistics.

- [src](./crates/rust_robotics_localization/src/ensemble_kalman_filter.rs)

## Adaptive Filter

Automatically switches between EKF (fast, linear) and CKF (robust, nonlinear) based on Normalized Innovation Squared (NIS). When innovation exceeds the chi-squared threshold, switches to CKF for better nonlinearity handling.

- [src](./crates/rust_robotics_localization/src/adaptive_filter.rs)

# Mapping
## NDT Map

<img src="./img/mapping/ndt.svg" width="640px">

- [src](./crates/rust_robotics_mapping/src/ndt.rs)

## Gaussian Grid Map

<img src="./img/mapping/gaussian_grid_map.svg" width="640px">

Occupancy grid mapping using Gaussian distribution. Higher probability near obstacles.

- [src](./crates/rust_robotics_mapping/src/gaussian_grid_map.rs)

## Ray Casting Grid Map

<img src="./img/mapping/ray_casting_grid_map.svg" width="640px">

Occupancy grid mapping using ray casting. Free space (0.5), Occupied (1.0), Unknown (0.0).

- [src](./crates/rust_robotics_mapping/src/ray_casting_grid_map.rs)

# SLAM

## Iterative Closest Point (ICP) Matching

<img src="./img/slam/icp_summary.png" width="640px">

Red: Reference points, Blue: Initial points, Green: Aligned points

- [src](./crates/rust_robotics_slam/src/icp_matching.rs)

## FastSLAM 1.0

<img src="./img/slam/fastslam1.svg" width="640px">

Particle filter based SLAM (Simultaneous Localization and Mapping). Each particle maintains its own map of landmarks using EKF.

Blue: True path, Yellow: Dead Reckoning, Green: FastSLAM estimate, Black: True landmarks, Cyan: Estimated landmarks

- [src](./crates/rust_robotics_slam/src/fastslam1.rs)

## EKF SLAM

<img src="./img/slam/ekf_slam.svg" width="640px">

Extended Kalman Filter based SLAM. Maintains a joint state vector of robot pose and landmark positions with full covariance matrix.

- [src](./crates/rust_robotics_slam/src/ekf_slam.rs)

## FastSLAM 2.0

Improved particle filter SLAM that incorporates the latest observation into the proposal distribution before sampling, producing better particle diversity than FastSLAM 1.0.

- [src](./crates/rust_robotics_slam/src/fastslam2.rs)

## Graph-Based SLAM

<img src="./img/slam/graph_based_slam.svg" width="640px">

Pose graph optimization for SLAM. Constructs a graph of robot poses connected by odometry and observation constraints, then optimizes the graph using iterative methods.

- [src](./crates/rust_robotics_slam/src/graph_based_slam.rs)

# Path Planning

## A* Algorithm

<img src="./img/path_planning/a_star_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Gray: Obstacles

- [src](./crates/rust_robotics_planning/src/a_star.rs)

```
cargo run -p rust_robotics --example a_star --features "planning,viz"
```

## Theta* Algorithm

<img src="./img/path_planning/theta_star_result.svg" width="640px">

Any-angle path planning algorithm. Unlike A* which restricts movement to grid edges, Theta* allows paths at any angle by checking line-of-sight between nodes.

- [src](./crates/rust_robotics_planning/src/theta_star.rs)

```
cargo run -p rust_robotics --example theta_star --features "planning,viz"
```

## Lazy Theta*

Lazy Theta* defers line-of-sight checks until node expansion, reducing redundant visibility tests. Achieves the same path quality as Theta* while being 1.7x faster (p=0.025 on 160 MovingAI scenarios).

- [src](./crates/rust_robotics_planning/src/lazy_theta_star.rs)

## Enhanced Lazy Theta*

Extends Lazy Theta* with wider parent selection at expansion time. Uses 2-ring neighborhood search and ancestor chain walks to find better any-angle shortcuts. Achieves near-optimal paths (+0.11% vs visibility-graph optimal on 50x50 grids).

- [src](./crates/rust_robotics_planning/src/enhanced_lazy_theta_star.rs)

## Anya (Optimal Any-Angle)

Optimal any-angle pathfinding using visibility-graph Dijkstra on all free cells. Guarantees the shortest any-angle path. Used as the optimality baseline for evaluating Theta* variants.

- [src](./crates/rust_robotics_planning/src/anya.rs)

## Path Smoothing

Post-processing pipeline for grid-based paths: greedy LOS shortcutting followed by iterative waypoint relaxation. Transforms grid-constrained A* paths into near-optimal any-angle paths. A*+optimize_path achieves 2.3x speedup over Theta* with equivalent path quality.

- [src](./crates/rust_robotics_planning/src/path_smoothing.rs)

## Jump Point Search (JPS)

<img src="./img/path_planning/jps_result.svg" width="640px">

Optimized pathfinding algorithm for uniform-cost grids. Reduces the number of nodes to explore by identifying and jumping to key "jump points" instead of examining all neighbors.

- [src](./crates/rust_robotics_planning/src/jps.rs)

```
cargo run -p rust_robotics --example jps --features "planning,viz"
```

## Bezier Path Planning

<img src="./img/path_planning/bezier_path_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](./crates/rust_robotics_planning/src/bezier_path_planning.rs)

## Cubic Spline

<img src="./img/path_planning/csp.svg" width="640px">

Black: Control points, Green: Path

- [src](./crates/rust_robotics_planning/src/cubic_spline_planner.rs)

## Dynamic Window Approach

<img src="./img/path_planning/dwa.svg" width="640px">

Black: Obstacles, Green: Trajectory, Yellow: Predicted trajectory

- [src](./crates/rust_robotics_planning/src/dwa.rs)

## D* Lite

<img src="./img/path_planning/d_star_lite_result.png" width="640">

Blue: Start, Red: Goal, Green: Path, Black: Obstacles

- [src](./crates/rust_robotics_planning/src/d_star_lite.rs)

D* Lite is an incremental heuristic search algorithm for path planning in dynamic environments. It's particularly efficient for replanning when the environment changes.

## Dijkstra Algorithm

<img src="./media/dijkstra-motion-planner.gif" width="640px">

- [src](./crates/rust_robotics_planning/src/dijkstra.rs)

## Informed RRT*

<img src="./img/path_planning/informed_rrt_star_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Black: Tree

- [src](./crates/rust_robotics_planning/src/informed_rrt_star.rs)

## Model Predictive Trajectory Generator

<img src="./img/path_tracking/model_predictive_trajectory_generator.svg" width="640px">

Green: Path

- [src](./crates/rust_robotics_control/src/model_predictive_trajectory_generator.rs)

## Potential Field Algorithm

<img src="./img/path_planning/potential_field_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Gray: Obstacles

- [src](./crates/rust_robotics_planning/src/potential_field.rs)

## Quintic Polynomials

<img src="./img/path_planning/quintic_polynomials_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](./crates/rust_robotics_planning/src/quintic_polynomials.rs)

## Rapidly-Exploring Random Trees (RRT)

<img src="./img/path_planning/rrt_star_result.svg" width="640px">

Sampling-based path planning algorithm that builds a tree by randomly sampling the configuration space.

Blue: Start, Red: Goal, Green: Path, Gray: Tree

- [src](./crates/rust_robotics_planning/src/rrt.rs)

## RRT*

<img src="./img/path_planning/rrt_star_result.svg" width="640px">

Optimized version of RRT that rewires the tree to find shorter paths. Asymptotically optimal.

Blue: Start, Red: Goal, Green: Path, Gray: Tree

- [src](./crates/rust_robotics_planning/src/rrt_star.rs)

## Reeds-Shepp Path

<img src="./img/path_planning/reeds_shepp_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](./crates/rust_robotics_planning/src/reeds_shepp_path.rs)

## Dubins Path

Shortest path for non-holonomic vehicles with bounded turning radius. Computes optimal paths composed of circular arcs and straight segments (6 types: LSL, RSR, LSR, RSL, RLR, LRL).

- [src](./crates/rust_robotics_planning/src/dubins_path.rs)

## PRM (Probabilistic Road-Map)

<img src="./img/path_planning/prm.svg" width="640px">

Sampling-based path planning using random samples and k-nearest neighbor connections.

Blue: Start, Red: Goal, Green: Path, Gray: Samples and edges, Black: Obstacles

- [src](./crates/rust_robotics_planning/src/prm.rs)

## Voronoi Road-Map

<img src="./img/path_planning/voronoi_road_map.svg" width="640px">

Path planning using Voronoi diagram vertices as waypoints. Provides paths that maximize clearance from obstacles.

Blue: Start, Red: Goal, Green: Path, Cyan: Voronoi vertices, Black: Obstacles

- [src](./crates/rust_robotics_planning/src/voronoi_road_map.rs)

## Frenet Optimal Trajectory

<img src="./img/path_planning/frenet_optimal_trajectory.svg" width="640px">

Optimal trajectory planning in Frenet coordinate frame. Widely used in autonomous driving for lane keeping and obstacle avoidance.

Gray: Reference path, Green: Optimal trajectory, Black: Obstacles, Red: Vehicle

- [src](./crates/rust_robotics_planning/src/frenet_optimal_trajectory.rs)

## State Lattice Planner

<img src="./img/path_planning/state_lattice_plan.svg" width="640px">

Lattice-based motion planning that searches over a pre-computed set of motion primitives. Generates smooth, dynamically feasible trajectories by connecting state lattice primitives.

- [src](./crates/rust_robotics_planning/src/state_lattice/)

```
cargo run -p rust_robotics --example state_lattice --features "planning,viz"
```

# Path Tracking

## LQR Steer Control

<img src="./img/path_tracking/lqr_steer_control.png" width="640px">

Black: Planned path, Green: Tracked path

- [src](./crates/rust_robotics_control/src/lqr_steer_control.rs)

## Move to Pose

<img src="./img/path_tracking/move_to_pose.png" width="640px">

Green: Path, Red: Start and Goal

- [src](./crates/rust_robotics_control/src/move_to_pose.rs)

## Pure Pursuit

<img src="./img/path_tracking/pure_pursuit.png" width="640px">

Black: Planned path, Green: Tracked path

- [src](./crates/rust_robotics_control/src/pure_pursuit.rs)

## Stanley Control

<img src="./img/path_tracking/stanley_controller.png" width="640px">

Black: Planned path, Green: Tracked path

- [src](./crates/rust_robotics_control/src/stanley_controller.rs)

## Rear Wheel Feedback Control

<img src="./img/path_tracking/rear_wheel_feedback.svg" width="640px">

Path tracking using rear wheel feedback steering control. Combines heading error and lateral error with path curvature feedforward.

Blue: Reference path, Red: Vehicle trajectory, Green: Waypoints

- [src](./crates/rust_robotics_control/src/rear_wheel_feedback.rs)

```
cargo run -p rust_robotics --example rear_wheel_feedback --features "control,viz"
```

## MPC (Model Predictive Control)

<img src="./img/path_tracking/mpc.svg" width="640px">

Model Predictive Control for path tracking using linearized bicycle model. Predicts future states and optimizes control inputs over a horizon.

Gray: Reference path, Blue: Tracked trajectory, Green: Prediction horizon, Red: Vehicle

- [src](./crates/rust_robotics_control/src/mpc.rs)

# Inverted Pendulum

## LQR Control

<img src="./img/inverted_pendulum/inverted_pendulum_lqr.png" width="640px">

Cart-pendulum animation showing LQR control stabilization. Blue: Cart, Black: Pendulum. Multiple frames overlaid to show time progression from initial angle to stabilized state.

- [src](./crates/rust_robotics_control/src/lqr_control.rs)

# Arm Navigation

## Two Joint Arm Control

<img src="./img/arm_navigation/two_joint_arm_control.png" width="640px">

Two joint arm to a point control simulation using inverse kinematics.

Black: Arm links, Red: Joints (shoulder, elbow, end effector), Green: Target position

- [src](./crates/rust_robotics_control/src/two_joint_arm_control.rs)

# Aerial Navigation

## 3D Grid A*

Bounded 3D voxel-grid planning for aerial robots. The planner supports 6-connected or 26-connected motion and returns a collision-free waypoint sequence.

- [src](./crates/rust_robotics_planning/src/grid_a_star_3d.rs)

# Mission Planning

## Behavior Tree

Behavior tree runtime for mission-level decision making with sequence, selector, condition, and action nodes backed by a shared blackboard.

- [src](./crates/rust_robotics_control/src/behavior_tree.rs)

## State Machine

<img src="./img/mission_planning/state_machine_diagram.svg" width="640px">

Finite state machine for robot behavior management with states, transitions, guards, and actions

- [src](./crates/rust_robotics_control/src/state_machine.rs)
