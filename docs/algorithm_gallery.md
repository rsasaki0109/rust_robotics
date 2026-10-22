# Algorithm gallery

[Back to the README](../README.md) · [Browser playground](https://rsasaki0109.github.io/rust_robotics/playground/)

Browse animations, source code, and runnable examples. Run commands from the
repository root.

[Localization](#localization) · [Mapping](#mapping) · [SLAM](#slam) ·
[Path planning](#path-planning) · [Path tracking](#path-tracking) ·
[Inverted pendulum](#inverted-pendulum) · [Arm navigation](#arm-navigation) ·
[Aerial navigation](#aerial-navigation) · [Mission planning](#mission-planning)

## Interactive lab guides

| Topic | Guides |
| --- | --- |
| Planning | [Grid search](grid_search_replay.md) · [RRT*](rrt_star_replay.md) · [Parking](parking_maneuver_lab.md) · [Time-aware routes](time_aware_route_lab.md) |
| Localization & mapping | [Sensor observations](localization_observations.md) · [Belief updates](localization_belief_lab.md) · [Occupancy evidence](occupancy_evidence_lab.md) |
| SLAM | [Scan matching](scan_matching_lab.md) · [Loop closure](slam_loop_lab.md) · [Exploration](frontier_exploration_lab.md) |
| Control | [Controllers](controller_inspector.md) · [DWA](dwa_velocity_lab.md) · [MPPI](mppi_decision_lab.md) · [Safety filter](safety_filter_lab.md) · [Formation](formation_lab.md) |
| Manipulation | [Arm IK](arm_lab.md) · [Arm tracking](arm_tracking.md) · [Pushing contact](pushing_contact_lab.md) |
| Flight | [Flight Lab](drone_race_project.md) |

## Localization

### Extended Kalman Filter Localization

<img src="../media/gallery/ekf_localization.gif" width="640px">

Gray: GPS measurements, Blue: Ground truth, Green: EKF estimate with 2-sigma covariance ellipse

- [src](../crates/rust_robotics_localization/src/ekf.rs)

```
cargo run -p rust_robotics --example render_gif_ekf_localization --features "localization,gif"
```

### Particle Filter Localization

<img src="../media/gallery/particle_filter.gif" width="640px">

Yellow: Range landmarks, Light blue: Particles, Blue: Ground truth, Green: Particle filter estimate

- [src](../crates/rust_robotics_localization/src/particle_filter.rs)

```
cargo run -p rust_robotics --example render_gif_particle_filter --features "localization,gif"
```

### Unscented Kalman Filter Localization

<img src="../img/localization/ukf_result.png" width="640px">

Blue: Ground Truth, Red: UKF Estimate, Black: Dead Reckoning, Green: GPS Observations, Red Ellipse: Uncertainty

- [src](../crates/rust_robotics_localization/src/unscented_kalman_filter.rs)

### Histogram Filter Localization

<img src="../img/localization/histogram_filter.svg" width="640px">

Grid-based probabilistic localization using RFID landmarks. The algorithm maintains a probability distribution over a 2D grid and updates it based on motion and observations.

Blue: True path, Orange: Dead Reckoning, Green: Histogram Filter estimate, Black: RFID landmarks

- [src](../crates/rust_robotics_localization/src/histogram_filter.rs)

### Cubature Kalman Filter

Cubature Kalman Filter (CKF) using 3rd-degree spherical-radial cubature rule. Achieves the same accuracy as UKF but 30% faster with zero tuning parameters (no alpha/beta/kappa). Recommended as the default over UKF for typical robotics scenarios.

- [src](../crates/rust_robotics_localization/src/cubature_kalman_filter.rs)

### Ensemble Kalman Filter

Stochastic ensemble-based Kalman filter. Maintains an ensemble of state particles and updates them using the Kalman gain computed from ensemble statistics.

- [src](../crates/rust_robotics_localization/src/ensemble_kalman_filter.rs)

### Adaptive Filter

Automatically switches between EKF (fast, linear) and CKF (robust, nonlinear) based on Normalized Innovation Squared (NIS). When innovation exceeds the chi-squared threshold, switches to CKF for better nonlinearity handling.

- [src](../crates/rust_robotics_localization/src/adaptive_filter.rs)

### Complementary Filter

Fuses high-frequency prediction (control/gyro) with low-frequency measurement (position sensor) using a tunable blending factor alpha. Simple, fast, and effective for IMU fusion.

- [src](../crates/rust_robotics_localization/src/complementary_filter.rs)

### Iterated EKF

Improves EKF accuracy by iterating the update step linearization. Re-linearizes the observation model around the updated state estimate multiple times until convergence.

- [src](../crates/rust_robotics_localization/src/iterated_ekf.rs)

### Information Filter

Dual of the Kalman Filter operating in information space (inverse covariance). Update step is additive in information form, making multi-sensor fusion natural.

- [src](../crates/rust_robotics_localization/src/information_filter.rs)

### Square Root UKF

UKF variant that propagates Cholesky factors instead of full covariance matrices. Improves numerical stability and guarantees positive semi-definiteness.

- [src](../crates/rust_robotics_localization/src/square_root_ukf.rs)

### Monte Carlo Localization

Adaptive Particle Filter with KLD-sampling. Automatically adjusts particle count based on posterior complexity — more particles for multi-modal distributions, fewer after convergence.

- [src](../crates/rust_robotics_localization/src/monte_carlo_localization.rs)

## Mapping

### NDT Map

<img src="../img/mapping/ndt.svg" width="640px">

- [src](../crates/rust_robotics_mapping/src/ndt.rs)

### Gaussian Grid Map

<img src="../img/mapping/gaussian_grid_map.svg" width="640px">

Occupancy grid mapping using Gaussian distribution. Higher probability near obstacles.

- [src](../crates/rust_robotics_mapping/src/gaussian_grid_map.rs)

### Ray Casting Grid Map

<img src="../img/mapping/ray_casting_grid_map.svg" width="640px">

Occupancy grid mapping using ray casting. Free space (0.5), Occupied (1.0), Unknown (0.0).

- [src](../crates/rust_robotics_mapping/src/ray_casting_grid_map.rs)

### DBSCAN Clustering

Density-based spatial clustering that finds arbitrary-shaped clusters and identifies outliers (noise). No need to specify number of clusters in advance.

- [src](../crates/rust_robotics_mapping/src/dbscan_clustering.rs)

### Line Extraction

Extracts line segments from 2D scan data using the Split-and-Merge (Iterative End Point Fit) algorithm. Used for feature extraction in indoor environments.

- [src](../crates/rust_robotics_mapping/src/line_extraction.rs)

### Implicit Moving Least Squares (IMLS)

PCA-based local normal estimation and Gaussian-weighted implicit surface
projection for 2D point sets. The API reports signed distance, surface normal,
projected point, and local support count.

- [src](../crates/rust_robotics_mapping/src/imls.rs)

### Occupancy Grid Map

Probabilistic occupancy grid using log-odds representation. Updates cells via Bresenham ray casting — free along rays, occupied at endpoints.

- [src](../crates/rust_robotics_mapping/src/occupancy_grid_map.rs)

### Gaussian Process Regression

GP regression with RBF kernel for terrain/surface mapping from sparse measurements. Provides predictions with uncertainty estimates.

- [src](../crates/rust_robotics_mapping/src/gaussian_process.rs)

## SLAM

### Iterative Closest Point (ICP) Matching

<img src="../media/gallery/icp_matching.gif" width="640px">

Blue: Previous scan, Red: Current scan, Green: Current scan aligned by ICP

- [src](../crates/rust_robotics_slam/src/icp_matching.rs)

```
cargo run -p rust_robotics --example render_gif_slam --features "slam,gif"
```

The SLAM crate also provides robust optimizer-backed point-to-line ICP in 2D
and point-to-plane ICP in 3D:

- [src](../crates/rust_robotics_slam/src/geometric_icp.rs)

### Scan-to-Map LiDAR Odometry

Registers each 2D scan against a bounded local submap of recent keyframes kept
in the corrected world frame, seeded by the odometry prediction, with
point-to-line Gauss-Newton, distance-gated correspondences, and
correction/residual gates. On a deterministic 28 m run with 3 % odometry scale
error and 1 deg/m yaw drift:

| estimator | position RMSE \[m\] | final yaw error \[deg\] |
| --- | ---: | ---: |
| raw odometry | 0.631 | 29.2 |
| seeded scan-to-scan | 0.011 | 0.36 |
| scan-to-map | 0.002 | 0.03 |

- [src](../crates/rust_robotics_slam/src/scan_to_map.rs)
- [design](scan_to_map_icp_design.md)

```
cargo run -p rust_robotics --example headless_scan_to_map --no-default-features --features slam
```

#### Loop Closure (LiDAR Graph SLAM)

<img src="../media/gallery/lidar_loop_closure.gif" width="640px">

Gray: ground truth, orange: scan-to-map front end, green: pose-graph nodes,
magenta: loop edges, blue: map from node scans, red: current scan. The front
end drifts in the pillar-free top corridor; the animation pauses on each
re-optimization so the correction is visible. In the playground's SLAM tab,
**LiDAR Loop Closure** replays this run and **Drive LiDAR SLAM** lets you
drive it yourself:
<https://rsasaki0109.github.io/rust_robotics/playground/?tab=slam&algorithm=drive&auto=1>

##### Perceptual aliasing

Identical pillars every 2.5 m make neighboring places look the same, so a loop
match can lock onto the pillar next door — and a run of such matches agrees
with itself, so no consistency check catches it. `LidarGraphSlam`'s
**ambiguity check** re-registers every accepted loop match from seeds shifted
±1/2/3 m and rejects it when a distinct alignment fits nearly as well:

| ambiguity check | loop closures | false closures | node RMSE \[m\] |
| --- | ---: | ---: | ---: |
| off | 16 | 12 | 1.812 (worse than scan-to-map alone, 0.292) |
| on | 7 | 0 | 0.032 |

```
cargo run -p rust_robotics --example headless_lidar_aliasing --no-default-features --features slam
```

The playground replays both runs (false loop edges in yellow) and offers an
**Aliased corridor** world in Drive LiDAR SLAM with the check switchable.

##### Real logs and the SLAM benchmark metric

`carmen_lidar_slam` runs `LidarGraphSlam` on CARMEN laser logs (Intel Research
Lab, Freiburg, …) and scores it with the relative-pose metric of the Kümmerle
et al. SLAM benchmark; without arguments it uses a synthetic log. See
[docs/datasets.md](datasets.md#carmen-2d-laser-logs-slam-benchmark).

`LidarGraphSlam` adds a pose-graph back end on top of the scan-to-map front
end: nodes every 1 m, coarse-to-fine scan-to-submap loop verification, and
**degeneracy-aware odometry edges** — the front end discards Gauss-Newton
steps along directions its Hessian cannot observe (a featureless corridor),
and those edges get uncertainty that grows with the distance travelled along
them, so the loop correction lands where the drift actually happened. On a
deterministic 98 m corridor loop with a pillar-free 24 m corridor:

| estimator | node RMSE \[m\] | final position error \[m\] |
| --- | ---: | ---: |
| raw odometry | 10.90 | 8.92 |
| scan-to-map | 0.302 | 0.381 |
| scan-to-map + loop closure | 0.012 | 0.020 |

- [src](../crates/rust_robotics_slam/src/lidar_graph_slam.rs)

```
cargo run -p rust_robotics --example headless_lidar_loop_closure --no-default-features --features slam
```

### FastSLAM 1.0

<img src="../media/gallery/fastslam.gif" width="640px">

Particle filter based SLAM (Simultaneous Localization and Mapping). Each particle maintains its own map of landmarks using EKF.

Yellow: True landmarks, Red cross: Estimated landmarks, Light blue: Particles, Blue: True path, Green: FastSLAM estimate

- [src](../crates/rust_robotics_slam/src/fastslam1.rs)

### EKF SLAM

<img src="../media/gallery/ekf_slam.gif" width="640px">

Extended Kalman Filter based SLAM. Maintains a joint state vector of robot pose and landmark positions with full covariance matrix.

Yellow: True landmarks, Red cross: Estimated landmarks, Blue: True path, Green: EKF-SLAM estimate

- [src](../crates/rust_robotics_slam/src/ekf_slam.rs)

### FastSLAM 2.0

Improved particle filter SLAM that incorporates the latest observation into the proposal distribution before sampling, producing better particle diversity than FastSLAM 1.0.

- [src](../crates/rust_robotics_slam/src/fastslam2.rs)

### Graph-Based SLAM

<img src="../img/slam/graph_based_slam.svg" width="640px">

Pose graph optimization for SLAM. Constructs a graph of robot poses connected by odometry and observation constraints, then optimizes the graph using iterative methods.

- [src](../crates/rust_robotics_slam/src/graph_based_slam.rs)

### Pose Graph Optimization

<img src="assets/factor-graph-optimization.svg" width="1000px" alt="Pose graph before and after block-sparse optimization">

SE(2) and SE(3) pose graph optimization using the shared
Levenberg-Marquardt factor-graph backend. Dense LU and block-sparse
preconditioned conjugate-gradient linear solvers are available; SE(3) factors
use analytic Jacobians. The module fixes the first pose to remove gauge freedom
and supports standard g2o SE2/SE3 quaternion text I/O.

- [src](../crates/rust_robotics_slam/src/pose_graph_optimization.rs)
- [SE(3) src](../crates/rust_robotics_slam/src/pose_graph_optimization_3d.rs)
- [g2o I/O](../crates/rust_robotics_slam/src/g2o.rs)
- [headless integration demo](../crates/rust_robotics/examples/headless_factor_graph_stack.rs)
- [scaling benchmark](../crates/rust_robotics/examples/benchmark_factor_graph_scaling.rs)

Representative `--release` results at size 200 on a development workstation:

| Problem / solver | Parameters | Dense matrix | Stored blocks | Iterations | Time | RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Pose graph / dense LU | 597 | 2.72 MiB | 29.2 KiB | 9 | 188.7 ms | 8.11e-7 |
| Pose graph / block-sparse PCG | 597 | 2.72 MiB | 29.2 KiB | 9 | 79.1 ms | 8.11e-7 |
| BA / dense LU | 600 | 2.75 MiB | 14.1 KiB | 4 | 80.9 ms | 8.42e-13 |
| BA / Schur complement | 600 | 2.75 MiB | 14.1 KiB | 4 | 1.86 ms | 8.42e-13 |

Reproduce the full 10/50/100/200 sweep:

```bash
cargo run --release -p rust_robotics --example benchmark_factor_graph_scaling \
  --no-default-features --features slam
```

For genuinely large graphs, `benchmark_large_pose_graph` runs only the
block-sparse path and fails if convergence or RMSE acceptance is missed:

| Poses | Parameters | Dense equivalent | Stored blocks | Iterations | Time | RMSE |
|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 2,997 | 68.5 MiB | 141.0 KiB | 8 | 0.64 s | 2.20e-3 |
| 5,000 | 14,997 | 1.68 GiB | 706.3 KiB | 9 | 4.54 s | 1.13e-3 |
| 10,000 | 29,997 | 6.70 GiB | 1.38 MiB | 9 | 8.62 s | 3.52e-4 |

```bash
cargo run --release -p rust_robotics --example benchmark_large_pose_graph \
  --no-default-features --features slam
```

### EuRoC / KITTI Dataset Replay

<img src="assets/euroc-vio-pipeline.svg" width="1000px" alt="EuRoC IMU, bundle adjustment, and SE3 pose graph pipeline">

The SLAM crate loads official EuRoC MAV and KITTI odometry directory layouts
without imposing an image decoder on the library. A companion CLI decodes
EuRoC PNGs, tracks Shi-Tomasi corners with forward/backward pyramidal
Lucas-Kanade, and triangulates them from the IMU-predicted metric trajectory.
The replay then jointly optimizes cameras and landmarks with Schur elimination
then refines navigation states and time-varying IMU biases before fusing metric
IMU edges with the visual closure in an SE(3) pose graph. EuRoC `imu0` `T_BS`
extrinsics include centripetal and tangential lever-arm correction.

- [Dataset and sidecar guide](datasets.md)
- [loader source](../crates/rust_robotics_slam/src/dataset.rs)
- [visual frontend source](../crates/rust_robotics_slam/src/visual_frontend.rs)
- [VIO pipeline source](../crates/rust_robotics_slam/src/vio_pipeline.rs)
- [MathematicalRobotics porting matrix](mathematical-robotics-porting.md)
- [MathematicalRobotics numerical parity](mathematical-robotics-parity.md)

```bash
cargo run --release -p rust_robotics \
  --example generate_euroc_feature_tracks \
  --no-default-features --features slam -- /datasets/EuRoC/MH_01_easy

cargo run -p rust_robotics --example headless_euroc_vio \
  --no-default-features --features slam
```

### IMU Preintegration

Bias-aware accelerometer/gyroscope preintegration with SO(3) updates, 9-state
error covariance propagation, 9x6 bias Jacobians, navigation-state prediction,
and an optimizer-ready IMU factor with analytic state and bias Jacobians.

- [src](../crates/rust_robotics_slam/src/imu_preintegration.rs)

### Bundle Adjustment

Pinhole projection, analytic camera/landmark reprojection Jacobians, robust
losses, configurable gauge anchoring, and joint camera/landmark optimization
with landmark Schur elimination enabled by default. Camera poses use
world-from-camera SE(3) matrices.

- [src](../crates/rust_robotics_slam/src/bundle_adjustment.rs)

### Correlative Scan Matching

Brute-force correlative scan matcher that searches over a discretized pose space. More robust to initial pose errors than ICP — useful as a SLAM front-end.

- [src](../crates/rust_robotics_slam/src/correlative_scan_matching.rs)

## Path Planning

### A* Algorithm

<img src="../img/path_planning/a_star_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Gray: Obstacles

- [src](../crates/rust_robotics_planning/src/a_star.rs)

```
cargo run -p rust_robotics --example a_star --features "planning,viz"
```

### Theta* Algorithm

<img src="../img/path_planning/theta_star_result.svg" width="640px">

Any-angle path planning algorithm. Unlike A* which restricts movement to grid edges, Theta* allows paths at any angle by checking line-of-sight between nodes.

- [src](../crates/rust_robotics_planning/src/theta_star.rs)

```
cargo run -p rust_robotics --example theta_star --features "planning,viz"
```

### Lazy Theta*

Lazy Theta* defers line-of-sight checks until node expansion, reducing redundant visibility tests. Achieves the same path quality as Theta* while being 1.7x faster (p=0.025 on 160 MovingAI scenarios).

- [src](../crates/rust_robotics_planning/src/lazy_theta_star.rs)

### Enhanced Lazy Theta*

Extends Lazy Theta* with wider parent selection at expansion time. Uses 2-ring neighborhood search and ancestor chain walks to find better any-angle shortcuts. Achieves near-optimal paths (+0.11% vs visibility-graph optimal on 50x50 grids).

- [src](../crates/rust_robotics_planning/src/enhanced_lazy_theta_star.rs)

### Anya (Optimal Any-Angle)

Optimal any-angle pathfinding using visibility-graph Dijkstra on all free cells. Guarantees the shortest any-angle path. Used as the optimality baseline for evaluating Theta* variants.

- [src](../crates/rust_robotics_planning/src/anya.rs)

### Path Smoothing

Post-processing pipeline for grid-based paths: greedy LOS shortcutting followed by iterative waypoint relaxation. Transforms grid-constrained A* paths into near-optimal any-angle paths. A*+optimize_path achieves 2.3x speedup over Theta* with equivalent path quality.

- [src](../crates/rust_robotics_planning/src/path_smoothing.rs)

### Jump Point Search (JPS)

<img src="../img/path_planning/jps_result.svg" width="640px">

Optimized pathfinding algorithm for uniform-cost grids. Reduces the number of nodes to explore by identifying and jumping to key "jump points" instead of examining all neighbors.

- [src](../crates/rust_robotics_planning/src/jps.rs)

```
cargo run -p rust_robotics --example jps --features "planning,viz"
```

### Bezier Path Planning

<img src="../img/path_planning/bezier_path_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](../crates/rust_robotics_planning/src/bezier_path_planning.rs)

### Cubic Spline

<img src="../img/path_planning/csp.svg" width="640px">

Black: Control points, Green: Path

- [src](../crates/rust_robotics_planning/src/cubic_spline_planner.rs)

### Dynamic Window Approach

<img src="../media/gallery/dwa.gif" width="640px">

Black: Obstacles, Blue cross: Goal, Green: Traveled trajectory, Light green: Best predicted trajectory

- [src](../crates/rust_robotics_planning/src/dwa.rs)

```
cargo run -p rust_robotics --example render_gif_dwa --features "planning,gif"
```

### D* Lite

<img src="../img/path_planning/d_star_lite_result.png" width="640">

Blue: Start, Red: Goal, Green: Path, Black: Obstacles

- [src](../crates/rust_robotics_planning/src/d_star_lite.rs)

D* Lite is an incremental heuristic search algorithm for path planning in dynamic environments. It's particularly efficient for replanning when the environment changes.

### Dijkstra Algorithm

<img src="../media/dijkstra-motion-planner.gif" width="640px">

- [src](../crates/rust_robotics_planning/src/dijkstra.rs)

### Informed RRT*

<img src="../img/path_planning/informed_rrt_star_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Black: Tree

- [src](../crates/rust_robotics_planning/src/informed_rrt_star.rs)

### Model Predictive Trajectory Generator

<img src="../img/path_tracking/model_predictive_trajectory_generator.svg" width="640px">

Green: Path

- [src](../crates/rust_robotics_control/src/model_predictive_trajectory_generator.rs)

### Potential Field Algorithm

<img src="../img/path_planning/potential_field_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path, Gray: Obstacles

- [src](../crates/rust_robotics_planning/src/potential_field.rs)

### Quintic Polynomials

<img src="../img/path_planning/quintic_polynomials_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](../crates/rust_robotics_planning/src/quintic_polynomials.rs)

### Rapidly-Exploring Random Trees (RRT)

<img src="../media/gallery/rrt.gif" width="640px">

Sampling-based path planning algorithm that builds a tree by randomly sampling the configuration space.

Green: Start and tree, Blue cross: Goal, Red: Found path, Black: Obstacles

- [src](../crates/rust_robotics_planning/src/rrt.rs)

```
cargo run -p rust_robotics --example render_gif_rrt --features "planning,gif"
```

### RRT*

<img src="../img/path_planning/rrt_star_result.svg" width="640px">

Optimized version of RRT that rewires the tree to find shorter paths. Asymptotically optimal.

Blue: Start, Red: Goal, Green: Path, Gray: Tree

- [src](../crates/rust_robotics_planning/src/rrt_star.rs)

### Hybrid A*

<img src="assets/playground/parking.png" width="640px" alt="Hybrid A* parking a car in the playground">

Kinematic search over `(x, y, yaw)` for a car-like robot, with Reeds-Shepp
analytic expansions to the goal, so plans can reverse. `with_vehicle` checks a
rectangular `VehicleFootprint` (covering circles on a fine collision grid)
instead of a single circle; `with_max_expansions` bounds the search so an
unreachable goal fails fast. Try it in the playground's **Parking** tab.

- [src](../crates/rust_robotics_planning/src/hybrid_a_star.rs)

### Reeds-Shepp Path

<img src="../img/path_planning/reeds_shepp_result.png" width="640px">

Blue: Start, Red: Goal, Green: Path

- [src](../crates/rust_robotics_planning/src/reeds_shepp_path.rs)

### Dubins Path

Shortest path for non-holonomic vehicles with bounded turning radius. Computes optimal paths composed of circular arcs and straight segments (6 types: LSL, RSR, LSR, RSL, RLR, LRL).

- [src](../crates/rust_robotics_planning/src/dubins_path.rs)

### PRM (Probabilistic Road-Map)

<img src="../img/path_planning/prm.svg" width="640px">

Sampling-based path planning using random samples and k-nearest neighbor connections.

Blue: Start, Red: Goal, Green: Path, Gray: Samples and edges, Black: Obstacles

- [src](../crates/rust_robotics_planning/src/prm.rs)

### Voronoi Road-Map

<img src="../img/path_planning/voronoi_road_map.svg" width="640px">

Path planning using Voronoi diagram vertices as waypoints. Provides paths that maximize clearance from obstacles.

Blue: Start, Red: Goal, Green: Path, Cyan: Voronoi vertices, Black: Obstacles

- [src](../crates/rust_robotics_planning/src/voronoi_road_map.rs)

### Frenet Optimal Trajectory

<img src="../img/path_planning/frenet_optimal_trajectory.svg" width="640px">

Optimal trajectory planning in Frenet coordinate frame. Widely used in autonomous driving for lane keeping and obstacle avoidance.

Gray: Reference path, Green: Optimal trajectory, Black: Obstacles, Red: Vehicle

- [src](../crates/rust_robotics_planning/src/frenet_optimal_trajectory.rs)

### State Lattice Planner

<img src="../img/path_planning/state_lattice_plan.svg" width="640px">

Lattice-based motion planning that searches over a pre-computed set of motion primitives. Generates smooth, dynamically feasible trajectories by connecting state lattice primitives.

- [src](../crates/rust_robotics_planning/src/state_lattice/)

```
cargo run -p rust_robotics --example state_lattice --features "planning,viz"
```

## Path Tracking

### LQR Steer Control

<img src="../img/path_tracking/lqr_steer_control.png" width="640px">

Black: Planned path, Green: Tracked path

- [src](../crates/rust_robotics_control/src/lqr_steer_control.rs)

### Move to Pose

<img src="../img/path_tracking/move_to_pose.png" width="640px">

Green: Path, Red: Start and Goal

- [src](../crates/rust_robotics_control/src/move_to_pose.rs)

### Pure Pursuit

<img src="../media/gallery/pure_pursuit.gif" width="640px">

Gray: Planned course, Green: Tracked path, Red: Vehicle

- [src](../crates/rust_robotics_control/src/pure_pursuit.rs)

```
cargo run -p rust_robotics --example render_gif_pure_pursuit --features "control,gif"
```

### Stanley Control

<img src="../img/path_tracking/stanley_controller.png" width="640px">

Black: Planned path, Green: Tracked path

- [src](../crates/rust_robotics_control/src/stanley_controller.rs)

### Rear Wheel Feedback Control

<img src="../img/path_tracking/rear_wheel_feedback.svg" width="640px">

Path tracking using rear wheel feedback steering control. Combines heading error and lateral error with path curvature feedforward.

Blue: Reference path, Red: Vehicle trajectory, Green: Waypoints

- [src](../crates/rust_robotics_control/src/rear_wheel_feedback.rs)

```
cargo run -p rust_robotics --example rear_wheel_feedback --features "control,viz"
```

### MPC (Model Predictive Control)

<img src="../img/path_tracking/mpc.svg" width="640px">

Model Predictive Control for path tracking using linearized bicycle model. Predicts future states and optimizes control inputs over a horizon.

Gray: Reference path, Blue: Tracked trajectory, Green: Prediction horizon, Red: Vehicle

- [src](../crates/rust_robotics_control/src/mpc.rs)

## Inverted Pendulum

### LQR Control

<img src="../img/inverted_pendulum/inverted_pendulum_lqr.png" width="640px">

Cart-pendulum animation showing LQR control stabilization. Blue: Cart, Black: Pendulum. Multiple frames overlaid to show time progression from initial angle to stabilized state.

- [src](../crates/rust_robotics_control/src/lqr_control.rs)

## Arm Navigation

### Two Joint Arm Control

<img src="../img/arm_navigation/two_joint_arm_control.png" width="640px">

Two joint arm to a point control simulation using inverse kinematics.

Black: Arm links, Red: Joints (shoulder, elbow, end effector), Green: Target position

- [src](../crates/rust_robotics_control/src/two_joint_arm_control.rs)

## Aerial Navigation

### 3D Grid A*

Bounded 3D voxel-grid planning for aerial robots. The planner supports 6-connected or 26-connected motion and returns a collision-free waypoint sequence.

- [src](../crates/rust_robotics_planning/src/grid_a_star_3d.rs)

### Drone 3D Trajectory Following

Closed-loop quadrotor waypoint tracking with quintic segments, PD thrust/attitude control, and Euler-integrated rigid-body dynamics.

- [src](../crates/rust_robotics_control/src/drone_3d_trajectory.rs)

### Drone Minimum-Snap Trajectory

Seventh-order minimum-snap segment generation for drone waypoint loops. The module exposes piecewise segment generation, desired-state sampling, and a direct path into the existing quadrotor tracker.

- [src](../crates/rust_robotics_control/src/minimum_snap_trajectory.rs)

## Mission Planning

### Behavior Tree

Behavior tree runtime for mission-level decision making with sequence, selector, condition, and action nodes backed by a shared blackboard.

- [src](../crates/rust_robotics_control/src/behavior_tree.rs)

### State Machine

<img src="../img/mission_planning/state_machine_diagram.svg" width="640px">

Finite state machine for robot behavior management with states, transitions, guards, and actions

- [src](../crates/rust_robotics_control/src/state_machine.rs)
