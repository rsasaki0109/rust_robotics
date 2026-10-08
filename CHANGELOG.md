# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `DijkstraPlanner` / `DijkstraConfig` (`rust_robotics_planning::dijkstra`): a
  world-coordinate Dijkstra implementing `PathPlanner` (A\* grid with a zero
  heuristic); `RRTStar` implements `PathPlanner` (and derives `Clone`, `Debug`);
  `Path2D` derives `PartialEq`.
- Tier 1 contract tests driving the planners as `Box<dyn PathPlanner>`, the
  path trackers as `Box<dyn PathTracker>`, and DWA through its inherent API
  (see `docs/api_traits.md`).
- Playground **Pushing** tab: interactive pusher-slider with face-switching
  MPPI (`rust_robotics_control::pusher_slider`) — drag/turn the goal, add
  obstacles, tune pusher friction, presets, contact-mode coloring, and share
  links (`tab=pushing&goal_x=…&mu=…&obstacles=…`).
- `rust_robotics_slam::carmen`: CARMEN log reader (`FLASER`, `ROBOTLASER1`)
  and `FLASER` writer; `rust_robotics_slam::slam_benchmark`: relations parser /
  writer and the Kümmerle et al. relative-pose metric. The
  `carmen_lidar_slam` example runs LiDAR graph SLAM on a log (real or a
  generated 180° front-laser log, CI-gated), scores odometry / scan-to-map /
  graph SLAM against relations, and can write a PNG map.
- `scan_to_map::ray_cast_ranges_fov` for limited field-of-view lasers.
- Perceptual-aliasing guard: `LidarGraphSlamConfig::loop_ambiguity_check`
  (default on) re-registers accepted loop matches from shifted seeds and
  rejects ambiguous ones; `LidarGraphSlam::ambiguous_loop_rejections`.
  `lidar_loop_scenario` gains `CorridorLayout::Periodic`, `start_offset`,
  `aliased_corridor_config`, and ground-truth `is_wrong_loop`. The
  `headless_lidar_aliasing` example (CI-gated) shows 12 false closures and a
  1.81 m node RMSE without the check vs none and 0.032 m with it.
- Playground: aliased-corridor scenarios in the loop-closure replay (false
  loop edges in yellow) and an **Aliased corridor** world plus a "Reject
  ambiguous loops" toggle in Drive LiDAR SLAM.
- `rust_robotics_slam::lidar_loop_scenario`: the deterministic corridor-loop
  scenario (walls, centerline, biased odometry, noisy LiDAR) with per-step
  recording, shared by the headless example, the gallery GIF, and the
  playground.
- Playground SLAM tab **LiDAR Loop Closure** mode: timeline over the corridor
  loop, jump to the first closure, toggle the map between front-end and
  loop-closed poses, shareable links (`algorithm=loop&frontend_map=…`).
- Playground SLAM tab **Drive LiDAR SLAM** mode: drive the corridor loop with
  the arrow keys or auto-drive while `LidarGraphSlam` runs live (block-sparse
  PCG back end), with odometry scale-error / yaw-drift / range-noise sliders,
  wall collisions, and share links (`algorithm=drive&odom_scale=…&auto=…`).
- Drive LiDAR SLAM course editing: world presets (corridor loop, pillar hall,
  empty box), drag-to-draw walls snapped to 0.1 m (undo / clear, up to 64),
  and share links that carry the world (`world=hall&walls=x1,y1,x2,y2;…`).
- Gallery GIF `media/gallery/lidar_loop_closure.gif`
  (`render_gif_lidar_loop_closure`).
- `rust_robotics_slam::lidar_graph_slam`: `LidarGraphSlam` adds pose-graph
  loop closure on top of scan-to-map odometry — distance-spaced nodes,
  coarse-to-fine scan-to-submap loop verification with inlier/residual/
  correction gates, innovation-gated re-optimization, and degeneracy-aware
  odometry edges whose uncertainty grows along directions the front end
  cannot observe. The `headless_lidar_loop_closure` example (CI-gated) cuts
  node RMSE on a 98 m corridor loop from 0.30 m to 0.012 m.
- Scan-to-map degeneracy handling: `ScanToMapConfig::degeneracy_ratio`
  discards Gauss-Newton steps along unobservable translation directions, and
  registrations expose their Hessian (`ScanRegistration::hessian`,
  `translational_observability`). `register_point_to_line` and
  `scan_points_with_normals` are public for one-shot registration.
- `rust_robotics_slam::scan_to_map`: scan-to-map 2D LiDAR odometry with a
  bounded keyframe submap in the corrected world frame, odometry-seeded
  point-to-line Gauss-Newton, distance-gated correspondences, and
  correction/residual gates. A seeded scan-to-scan preset shares the same API.
  The `headless_scan_to_map` example compares raw odometry, scan-to-scan, and
  scan-to-map on a deterministic biased-odometry run and is gated in CI.
- `no_std` support for `rust_robotics_control` Tier 1 controllers (PID, Pure
  Pursuit, Stanley, LQR Steer); the remaining controllers stay behind `std`.
- `rust_robotics_embedded_demo`: EKF + Pure Pursuit / PID closed loop on an
  emulated STM32F405 under QEMU, gated by the `embedded-demo` CI job.
- `meta_control`: deterministic controller switching over Pure Pursuit /
  Stanley / LQR Steer under the shared Controller Arena engine.
- Benchmark regression gate (`scripts/check_benchmark_gate.sh`, `BENCHMARKS.md`)
  over 11 deterministic benchmark examples.
- Playground onboarding and recent-experiment history.
- A streaming EuRoC visual frontend with Shi-Tomasi detection, pyramidal
  Lucas-Kanade tracking, forward/backward checks, IMU-seeded triangulation,
  protected sidecar output, and a PNG CLI.
- MathematicalRobotics-compatible IMU extrinsic/lever-arm transforms,
  navigation-state and bias factor families, EuRoC `imu0` `T_BS` ingestion,
  and visual-pose-constrained state/bias refinement in the VIO pipeline.

### Changed
- `AStarConfig::heuristic_weight` accepts `0.0` (uniform-cost search).
- Dev/test builds compile `rust_robotics_slam` and `rust_robotics_optimization`
  with `opt-level = 2`; the SLAM headless examples run 20–50× faster in CI
  (e.g. `headless_lidar_loop_closure` 21 s → 0.6 s) with identical output.

### Removed
- `GridPathPlanner`, `SamplingBasedPlanner`, and `TrajectoryTracker` from
  `rust_robotics_core::traits`: never implemented, and their roles are covered
  by `PathPlanner`, `PathTracker`, and `Controller` (see `docs/api_traits.md`).

### Fixed
- `PathTracker` impls of Pure Pursuit, Stanley, LQR Steer, LQR Speed-Steer, and
  Rear Wheel Feedback ignored a new path with the same number of points as the
  old one; they now compare path contents.
- Rear Wheel Feedback dropped its lateral-error term whenever the heading error
  was exactly zero (`sin(θe)/θe` evaluated as 0), so a parallel offset was never
  corrected; small-signal denominators also lost their sign.
- `RobustIcp2D::estimate` applied the current transform twice and mixed a
  right-perturbation Jacobian with a left-composed update; non-identity seeds
  did not converge and identity-seeded results were off by up to ~0.2 m.

## [0.2.0] - 2026-07-31

### Added
- **`no_std` localization stack.** `rust_robotics_core` and
  `rust_robotics_localization` build without the standard library (`alloc`
  only), so the EKF / Iterated EKF / UKF / Cubature KF / Square-Root UKF /
  Information Filter / Complementary Filter / Histogram Filter / Adaptive Filter
  cross-compile to bare-metal targets (e.g. `thumbv7em-none-eabihf`). Math is
  routed through `libm`; verified on every commit by the `embedded-check` CI
  job. Sampling-based localizers (PF, MCL, EnKF) stay behind the default `std`
  feature because they need an entropy source.
- **Animated GIF gallery rendered in pure Rust** (`gif` feature) — every gallery
  animation is regenerated by the library with no gnuplot/system dependency via
  `./scripts/generate_gallery_gifs.sh`.
- crates.io publish readiness: docs.rs metadata on all first-release crates, CI
  `package-check` and `wasm-check` jobs, and `.cargo/config.toml` wasm RUSTFLAGS.
- `rust_robotics_playground`: native egui grid-planner demo (A\*, Dijkstra, JPS,
  Theta\*) with click-to-edit obstacles, draggable start/goal, and compare-all
  timing table — first slice of the Phase 2 WASM playground.
- WASM playground deploy: Trunk builds to GitHub Pages at `/playground/` with
  gallery links (**Run in browser**).
- Playground **Localization** tab: arrow-key unicycle driving with Particle
  Filter (120 particles) and EKF tabs, noise slider, trails, and steps/sec.
- Playground **SLAM** tab: canned trajectory replay with timeline scrubber for
  EKF-SLAM, FastSLAM 1.0, and ICP scan matching (landmarks, particles, aligned scans).
- Playground **ADMM Formation** tab: receding-horizon consensus ADMM with four
  agents, stiff vs smoothed overlay, noise slider, and jerk/tracking metrics.
- Reproducible playground links: the selected tab is shareable, and grid-planner
  links preserve the planner, start/goal cells, and complete obstacle map.
- Guarded release workflow for dependency-ordered workspace publishing,
  crates.io propagation checks, tagging, and GitHub Release creation.
- Reusable Lie-group/factor-graph stack inspired by MathematicalRobotics:
  stable SO(2)/SE(2)/SO(3)/SE(3), robust Gauss-Newton/Levenberg-Marquardt,
  g2o pose graphs, bias-aware IMU preintegration, bundle adjustment, and
  point-to-line/point-to-plane ICP.
- Block-sparse PCG and bundle-adjustment Schur-complement linear solvers, with
  analytic SE(3), IMU, and reprojection Jacobians validated against finite
  differences.
- Reproducible factor-graph integration/scaling examples plus pure-Rust SVG/GIF
  gallery assets.
- EuRoC MAV and KITTI odometry loaders, optional pre-extracted EuRoC feature
  sidecars, and an IMU preintegration → Schur BA → block-sparse SE(3) replay.
- Golden-value regression tests against MathematicalRobotics commit
  `79600010f0c86179905a6960e5fce2bb7cc85d77`.
- Convergent 1k/5k/10k pose benchmarks and precomputed block-Jacobi PCG
  preconditioners.

### Fixed
- `RRTPlanner::planning()` now records the search tree so `get_tree()` exposes
  the explored edges (previously returned an empty tree).

## [0.1.0] - 2026-03-23

### Added
- Cargo workspace with 8 domain crates:
  - `rust_robotics_core` — shared types, traits, error handling
  - `rust_robotics_planning` — path planning algorithms (A\*, JPS, Theta\*, RRT, DWA, etc.)
  - `rust_robotics_localization` — EKF, UKF, Particle Filter, Histogram Filter
  - `rust_robotics_control` — path tracking, LQR, MPC, behavior tree, state machine
  - `rust_robotics_mapping` — NDT, Gaussian Grid, Ray Casting
  - `rust_robotics_slam` — EKF-SLAM, FastSLAM, Graph SLAM, ICP
  - `rust_robotics_viz` — Visualizer with gnuplot backend
  - `rust_robotics` — umbrella crate with feature-gated re-exports
- **Dubins Path** planner (6 path types: LSL, RSR, LSR, RSL, RLR, LRL)
- Top-level re-exports for planning (19 types), control (9), localization (7), mapping (3), slam (2)
- Unified planner APIs: `plan_from()` for RRT\*/Informed RRT\*, `plan_with_obstacles()` for Potential Field
- `#[must_use]` on core types (Point2D, Point3D, Pose2D, State2D, ControlInput, GridNode)
- Headless examples (grid planners, localizers, navigation loop)
- Criterion benchmarks for A\* vs JPS vs Theta\*
- Proptest property-based tests for EKF/UKF/PF non-divergence
- 230 unit tests across all crates
- CI pipeline: build, test, clippy (-D warnings), rustdoc (-D warnings), fmt, cargo-deny, cargo-tarpaulin coverage, cargo-semver-checks (PRs)
- GitHub Pages with showcase gallery and API docs
- README badges (CI, codecov, docs)
- CLAUDE.md with build/test/feature documentation
- `deny.toml` for dependency auditing
- Workspace dependency versions for crates.io publish readiness

### Fixed
- Clippy warnings (identical if blocks in ICP, collapsible else-if in JPS)
- Rustdoc warnings (escaped `[m]`/`[rad]` in doc comments, bare URLs)
- Broken README example commands (30 removed, 5 updated with `--features`)
- Showcase `--bin` commands updated to `--example`

### Removed
- Unused `plotlib` dependency from `rust_robotics_viz`
