# Rust guide

[Back to the README](../README.md) · [Algorithm gallery](algorithm_gallery.md)

Use the library, run examples, or reproduce a benchmark. Run commands from the
repository root unless a section says otherwise.

[Library](#use-as-a-library) · [Build and test](#build-and-test) ·
[Embedded](#embedded--no_std) · [Workspace](#what-is-inside) ·
[Examples](#more-examples) · [ROS2](#ros2-integration) · [Benchmarks](#benchmarks)

## Use As A Library

Add the latest published umbrella crate from crates.io:

```bash
cargo add rust_robotics --features "planning,localization,control"
```

If you need changes that have not reached crates.io yet:

```toml
[dependencies]
rust_robotics = { git = "https://github.com/rsasaki0109/rust_robotics" }
```

Import the domain crate you need:

```rust
use rust_robotics::planning::{AStarConfig, AStarPlanner, DWAPlanner};
use rust_robotics::localization::{EKFConfig, EKFLocalizer};
use rust_robotics::control::{PurePursuitController, StanleyController};
```

Lie-group and optimization primitives are always available from the umbrella
crate:

```rust
use rust_robotics::core::{se3_exp, se3_log, Vector6};
use rust_robotics::optimization::{RobustKernel, SolverConfig};

let tangent = Vector6::new(1.0, 0.0, 0.0, 0.1, -0.2, 0.3);
let transform = se3_exp(&tangent);
assert!((se3_log(&transform) - tangent).norm() < 1.0e-9);

let _config = SolverConfig::default();
let _loss = RobustKernel::Huber { delta: 1.0 };
```

## Build and test

```bash
cargo build --workspace --lib --tests --examples
cargo test --workspace --lib --tests
```

## Embedded / no_std

The core types and the Kalman-family localizers build without the standard
library, so the same EKF you simulate on a workstation runs on a $5
microcontroller (Raspberry Pi Pico 2, STM32, ESP32-C3, ...):

```bash
rustup target add thumbv7em-none-eabihf
cargo build -p rust_robotics_core -p rust_robotics_localization \
  --no-default-features --target thumbv7em-none-eabihf
```

Available in `no_std` mode (with `alloc`): **EKF, Iterated EKF, UKF, Cubature KF,
Square-Root UKF, Information Filter, Complementary Filter, Histogram Filter, and
the EKF/CKF Adaptive Filter**. Math is routed through pure-Rust
[`libm`](https://crates.io/crates/libm), no FPU or OS required. The
sampling-based localizers (particle filter, MCL, ensemble KF) stay behind the
default `std` feature because they need an entropy source. This is verified on
every commit by the `embedded-check` CI job.

## What Is Inside

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

## More Examples

The [pose-observation loss experiment](../experiments/pose_observation_loss/README.md)
compares Pure Pursuit pause/recovery behavior with paired simulation inputs and
records both contacts and mission completion.

```bash
# Regenerate every animated GIF in the gallery (pure Rust, no system deps)
./scripts/generate_gallery_gifs.sh
```

<details>
<summary><b>Show all example commands (headless, benchmarks, SVG/GIF renderers, visualization)</b></summary>

```bash
# Animated GIF renderers (media/gallery/, pure Rust)
cargo run -p rust_robotics --example render_gif_ekf_localization --features "localization,gif"
cargo run -p rust_robotics --example render_gif_particle_filter --features "localization,gif"
cargo run -p rust_robotics --example render_gif_pure_pursuit --features "control,gif"
cargo run -p rust_robotics --example render_gif_dwa --features "planning,gif"
cargo run -p rust_robotics --example render_gif_rrt --features "planning,gif"
cargo run -p rust_robotics --example render_gif_slam --features "slam,gif"
cargo run -p rust_robotics --example render_factor_graph_optimization --features "slam,gif"

# Headless (no GUI dependencies)
cargo run -p rust_robotics --example headless_grid_planners --features planning
cargo run -p rust_robotics --example headless_conformal_sipp --no-default-features --features planning
cargo run -p rust_robotics --example headless_traversal_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example headless_elevation_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example headless_risk_map_smoothing --no-default-features --features planning
cargo run -p rust_robotics --example headless_clearance_risk_graph --no-default-features --features planning
cargo run -p rust_robotics --example headless_adaptive_costmap_namo --no-default-features --features planning
cargo run -p rust_robotics --example render_traversal_risk_graph_svg --no-default-features --features planning
cargo run -p rust_robotics --example headless_localizers --features localization
cargo run -p rust_robotics --example headless_mppi_double_integrator --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_constraint_discount --no-default-features --features control
cargo run -p rust_robotics --example benchmark_mppi_unified --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_terminal_value --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_value_learning --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_replay_value_learning --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_value_grid_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_adaptive_temperature --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_track_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_track_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_mppi_racing_gate_progress --no-default-features --features control
cargo run -p rust_robotics --example render_mppi_racing_gate_progress_svg --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_mppi_3d --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_quadrotor --no-default-features --features control
cargo run -p rust_robotics --example benchmark_cbf_safety_filter --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_motor --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_aware --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_budget --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_recovery --no-default-features --features control
cargo run -p rust_robotics --example benchmark_racing_powertrain_endurance --no-default-features --features control
cargo run -p rust_robotics --example benchmark_pusher_slider --no-default-features --features control
cargo run -p rust_robotics --example benchmark_pusher_slider_multi --no-default-features --features control
cargo run -p rust_robotics --example benchmark_pusher_slider_two_contact --no-default-features --features control
cargo run -p rust_robotics --example benchmark_admm_formation --no-default-features --features control
cargo run -p rust_robotics --example benchmark_admm_graph_consensus --no-default-features --features control
cargo run -p rust_robotics --example benchmark_admm_horizon_consensus --no-default-features --features control
cargo run -p rust_robotics --example headless_adap_rpf_mppi --no-default-features --features control
cargo run -p rust_robotics --example render_adap_rpf_mppi_svg --no-default-features --features control
cargo run -p rust_robotics --example headless_branchout_multimodal_driving --no-default-features --features planning
cargo run -p rust_robotics --example render_branchout_multimodal_driving_svg --no-default-features --features planning
cargo run -p rust_robotics --example headless_stl_cbs_multi_robot --no-default-features --features planning
cargo run -p rust_robotics --example render_stl_cbs_multi_robot_svg --no-default-features --features planning
cargo run -p rust_robotics --example headless_kinodynamic_stl_cbs --no-default-features --features planning
cargo run -p rust_robotics --example render_kinodynamic_stl_cbs_svg --no-default-features --features planning
cargo run -p rust_robotics --example render_safe_decode_nav_svg --no-default-features --features planning
cargo run -p rust_robotics --example render_frontier_navigator_svg --no-default-features --features planning
cargo run -p rust_robotics --example headless_rigid_body_mip_planning --no-default-features --features planning
cargo run -p rust_robotics --example render_rigid_body_mip_planning_svg --no-default-features --features planning
cargo run -p rust_robotics --example headless_hierarchical_mapf_replanning --no-default-features --features planning
cargo run -p rust_robotics --example render_hierarchical_mapf_replanning_svg --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_hierarchical_mapf_scale --no-default-features --features planning
cargo run -p rust_robotics --example headless_navigation_loop --features "planning,localization,control"
cargo run -p rust_robotics --example headless_mission_recovery --features "planning,localization,control"
cargo run --release -p rust_robotics --example generate_euroc_feature_tracks --no-default-features --features slam -- /datasets/EuRoC/MH_01_easy
cargo run -p rust_robotics --example headless_euroc_vio --no-default-features --features slam
cargo run -p rust_robotics --example render_euroc_vio_svg --no-default-features --features slam
cargo run -p rust_robotics --example benchmark_conformal_sipp --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_conformal_coverage --no-default-features --features planning
cargo run -p rust_robotics --example benchmark_traversal_risk_sweep --no-default-features --features planning

# Visualization (requires gnuplot)
cargo run -p rust_robotics --example a_star --features "planning,viz"
cargo run -p rust_robotics --example jps --features "planning,viz"
cargo run -p rust_robotics --example rear_wheel_feedback --features "control,viz"
```

</details>

### dora-rs dataflow example

The workspace also includes a minimal `dora-rs` planning demo that wraps the existing headless A* planner in a dora node and sends a structured JSON path report to a sink node.

```bash
dora run crates/rust_robotics/examples/dora_path_planning_dataflow.yml
```

This example requires the `dora` runtime/CLI to be installed and uses the feature-gated `dora` support in `crates/rust_robotics`.

## ROS2 Integration

The workspace includes ready-to-use ROS2 navigation nodes built with safe_drive (Rust ROS2 bindings).

- Path Planner (A*)
- DWA Local Planner
- SLAM Node
- SLAM Corrected Odom (optional)
- EKF Localizer
- Waypoint Navigator

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

Demo video: [docs/gazebo_demo.mp4](gazebo_demo.mp4)

See [docs/ros2_integration.md](ros2_integration.md) for details.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42  # optional but recommended if other ROS graphs are already running

cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/ekf_localizer_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/waypoint_navigator_node/Cargo.toml

export TURTLEBOT3_MODEL=burger
./ros2_nodes/launch/run_gazebo_demo.sh

# Multi-goal mission demo
WAYPOINT_NAV_FRAME=relative_start \
WAYPOINT_NAV_WAYPOINTS="0.4,0.0;0.1,0.4" \
  ./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

`run_gazebo_mission_demo.sh` defaults to `WAYPOINT_NAV_FRAME=relative_start`, so the mission waypoints above are interpreted as offsets from the first odom pose observed by `waypoint_navigator_node`. The wrapper's default mission is a conservative two-waypoint route that was verified in TurtleBot3 world: `(0.4, 0.0) -> (0.1, 0.4)`.

`waypoint_navigator_node` now includes a simple recovery state machine. If the active waypoint stays outside tolerance without measurable odom progress for `WAYPOINT_NAV_STUCK_TIMEOUT` seconds, it issues `navigation_cancel`, waits briefly, rotates in place, backs off, then republishes the active waypoint. The main tuning knobs are `WAYPOINT_NAV_MAX_RECOVERY_ATTEMPTS`, `WAYPOINT_NAV_RECOVERY_ROTATE_SECONDS`, `WAYPOINT_NAV_RECOVERY_BACKOFF_SECONDS`, and `WAYPOINT_NAV_RECOVERY_BACKOFF_SPEED`.

For observability, `navigation_demo.launch.py` now also exposes:

- `ENABLE_RVIZ=true ./ros2_nodes/launch/run_gazebo_demo.sh` to open RViz with [navigation_demo.rviz](../ros2_nodes/launch/navigation_demo.rviz)
- `ENABLE_GAZEBO_GUI=false ./ros2_nodes/launch/run_gazebo_demo.sh` for a headless Gazebo server run
- dynamic TF from the selected nav odom topic to the robot base frame via [odom_tf_broadcaster.py](../ros2_nodes/launch/odom_tf_broadcaster.py)
- `/mission_status` (`std_msgs/String`) for mission / recovery state summaries
- `/mission_markers` (`visualization_msgs/MarkerArray`) for the route, active goal, and status text

By default, the current Gazebo demo uses `odom` as its honest global frame: `slam_node` publishes `/map` in the raw odom frame it actually integrates against, `path_planner_node` republishes `/planned_path` in that same frame, and `waypoint_navigator_node` publishes `/goal_pose` plus `/mission_markers` in `RUST_NAV_GLOBAL_FRAME=odom`. If you still want the old RViz alias, you can opt into `PUBLISH_MAP_ODOM_TF=true` to add a legacy identity `map -> odom` transform.

If you want the experimental corrected SLAM frame, enable:

```bash
ENABLE_SLAM_CORRECTED_FRAME=true ./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

That switches the mission stack to `NAV_ODOM_TOPIC=/slam_odom` and `NAV_GLOBAL_FRAME=map`. In this mode, `slam_node` publishes `/slam_pose` plus `/slam_odom`, the map is integrated in `map`, and [map_odom_tf_broadcaster.py](../ros2_nodes/launch/map_odom_tf_broadcaster.py) estimates a dynamic `map -> odom` transform from `/slam_odom` against the raw odom stream. The corrected pose update is quality-gated: high-error, low-motion, or outlier ICP deltas fall back to pure odom, and medium-quality matches are attenuated instead of applied at the full blend factor.

Corrected mode also exposes two extra observability topics:

- `/slam_diagnostics` (`std_msgs/String`) with per-scan ICP convergence, error distribution, inlier ratio, `blend_alpha`, `gate_reason`, and applied correction deltas
- `/slam_ground_truth_status` (`std_msgs/String`) with relative-start Gazebo ground-truth error metrics derived from `gz topic -e -t /world/default/dynamic_pose/info --json-output`

The ground-truth monitor defaults to the spawned model name (`GROUND_TRUTH_ENTITY_NAME=$TURTLEBOT3_MODEL`) and compares `/ekf_odom` plus `/slam_odom` against the model pose after subtracting the first ground-truth sample. You can override the Gazebo source with `GROUND_TRUTH_GZ_POSE_TOPIC` if needed.

For a local ROS2/Gazebo regression check, run:

```bash
ROS_DOMAIN_ID=89 ENABLE_RVIZ=false ENABLE_GAZEBO_GUI=false ./ros2_nodes/launch/run_navigation_smoke_test.sh
```

The smoke script launches the mission demo, verifies `/map`, `/planned_path`, and typed `/mission_markers` all match the nav odom frame, checks the dynamic nav odom TF, and waits for `mission complete -> goal cleared -> stop command` in the navigation logs.

To smoke-test the corrected SLAM frame as well:

```bash
ROS_DOMAIN_ID=90 ENABLE_RVIZ=false ENABLE_GAZEBO_GUI=false ENABLE_SLAM_CORRECTED_FRAME=true \
  ./ros2_nodes/launch/run_navigation_smoke_test.sh
```

In corrected mode, the same script also verifies dynamic `map -> odom`.

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
