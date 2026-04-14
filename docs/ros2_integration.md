# ROS2 Integration with `safe_drive`

## Prerequisites

- ROS2 Jazzy installed and sourced before build or run:
  - `source /opt/ros/jazzy/setup.bash`
- `safe_drive` repository available at:
  - `/media/sasaki/aiueo/ai_coding_ws/rust_robo_ws/safe_drive`
- Rust toolchain (stable) with `cargo`
- Gazebo demo packages installed:
  - `ros-jazzy-turtlebot3-gazebo`
  - `ros-jazzy-turtlebot3-bringup`
  - `ros-jazzy-turtlebot3-description`
  - `ros-jazzy-nav2-bringup`
  - `ros-jazzy-ros-gz`

If `cargo check` or `cargo build` fails with ROS2 type support errors, source ROS2 first and retry.

## Build Instructions

### Cargo (per node)

```bash
source /opt/ros/jazzy/setup.bash

cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/ekf_localizer_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/waypoint_navigator_node/Cargo.toml
```

### Core library workspace

```bash
cargo build --workspace
cargo test --workspace --lib --tests
```

## Node Descriptions and Topics

### `path_planner_node`

- **Role**: Global path planning with A*
- **Subscriptions**:
  - `/goal_pose` (`geometry_msgs/PoseStamped`)
  - `RUST_NAV_ODOM_TOPIC` (`nav_msgs/Odometry`, default `/odom`)
  - `/map` (`nav_msgs/OccupancyGrid`)
- **Publications**:
  - `/planned_path` (`nav_msgs/Path`)
- **Configuration**:
  - `RUST_NAV_ODOM_TOPIC`
  - `DWA_GOAL_THRESHOLD`
  - `DWA_ROBOT_RADIUS`
- **Notes**:
  - Rebuilds its A* planner whenever `/map` updates
  - Converts occupied grid cells into A* obstacles
  - Uses the selected odometry topic pose as the robot start pose

### `dwa_planner_node`

- **Role**: Local trajectory planning and velocity command generation
- **Subscriptions**:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `RUST_NAV_ODOM_TOPIC` (`nav_msgs/Odometry`, default `/odom`)
  - `/planned_path` (`nav_msgs/Path`)
- **Publications**:
  - `/cmd_vel` (`geometry_msgs/Twist`)
- **Configuration**:
  - `RUST_NAV_ODOM_TOPIC`

### `slam_node`

- **Role**: Scan matching and occupancy-grid mapping
- **Subscriptions**:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/odom` (`nav_msgs/Odometry`)
- **Publications**:
  - `/map` (`nav_msgs/OccupancyGrid`)

### `ekf_localizer_node`

- **Role**: Filter wheel odometry into a smoother navigation odom stream
- **Subscriptions**:
  - `/odom` (`nav_msgs/Odometry`) by default
- **Publications**:
  - `/ekf_pose` (`geometry_msgs/PoseStamped`)
  - `/ekf_odom` (`nav_msgs/Odometry`)
- **Configuration**:
  - `EKF_INPUT_ODOM_TOPIC`
  - `EKF_OUTPUT_ODOM_TOPIC`
  - `EKF_OUTPUT_POSE_TOPIC`
- **Notes**:
  - Wraps `rust_robotics_localization::EKFLocalizer`
  - Uses raw odometry pose as measurement and raw twist as control input
  - Provides a filtered odom source for the planner / mission nodes

### `waypoint_navigator_node`

- **Role**: Mission-level sequencing of multiple 2D goals
- **Subscriptions**:
  - `RUST_NAV_ODOM_TOPIC` (`nav_msgs/Odometry`, default `/odom`)
- **Publications**:
  - `/goal_pose` (`geometry_msgs/PoseStamped`)
  - `/mission_status` (`std_msgs/String`)
  - `/mission_markers` (`visualization_msgs/MarkerArray`)
- **Configuration**:
  - `RUST_NAV_ODOM_TOPIC`
  - `WAYPOINT_NAV_WAYPOINTS`: semicolon-delimited `x,y` mission string
  - `WAYPOINT_NAV_FRAME`: `map` or `relative_start`
  - `WAYPOINT_NAV_GOAL_TOLERANCE`: waypoint reach tolerance \[m\]
  - `WAYPOINT_NAV_LOOP`: whether to restart after the last waypoint
- **Notes**:
  - Waits for the selected odometry topic before sending the first goal
  - Republishes the active `/goal_pose` every 2 seconds until the planner reacts
  - `relative_start` resolves waypoints as offsets from the robot pose seen on the first odom sample
  - Publishes human-readable mission / recovery summaries on `/mission_status`
  - Publishes RViz markers for the resolved route, active goal, and current mission state on `/mission_markers`

## Navigation Stack Architecture

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
                |                             |
      +----------------------+                v
      | ekf_localizer_node   |             /cmd_vel
      +----------------------+
                ^
                |
      +-------------------------+
      | waypoint_navigator_node |
      +-------------------------+
```

## Gazebo Demo

### Launch the full stack

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42  # recommended if another ROS graph is already active on the machine
export TURTLEBOT3_MODEL=burger

./ros2_nodes/launch/run_gazebo_demo.sh
```

The wrapper script starts [navigation_demo.launch.py](../ros2_nodes/launch/navigation_demo.launch.py), which:

- launches `turtlebot3_gazebo/turtlebot3_world.launch.py`
- forwards the upstream TurtleBot3 world spawn defaults (`x=-2.0`, `y=-0.5`) as launch arguments
- adds an extra `ros_gz_bridge` subscription so `/cmd_vel` accepts `geometry_msgs/Twist`
- publishes an identity `map -> odom` static transform for RViz / debugging tools
- waits 5 seconds for Gazebo topics to appear
- starts `slam_node`, `path_planner_node`, and `dwa_planner_node` from `target/release`

If you want RViz alongside Gazebo:

```bash
ENABLE_RVIZ=true ./ros2_nodes/launch/run_gazebo_demo.sh
```

The RViz layout is stored at [navigation_demo.rviz](../ros2_nodes/launch/navigation_demo.rviz).

Demo video: [gazebo_demo.mp4](./gazebo_demo.mp4)

### Launch the mission demo

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=burger
export NAV_ODOM_TOPIC=/ekf_odom
export WAYPOINT_NAV_FRAME=relative_start
export WAYPOINT_NAV_WAYPOINTS="0.4,0.0;0.1,0.4"

./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

The mission wrapper uses `WAYPOINT_NAV_FRAME=relative_start` by default, so the demo waypoints above are interpreted as offsets from the first odom pose observed by `waypoint_navigator_node`. Its default mission is a verified two-waypoint route, `(0.4, 0.0) -> (0.1, 0.4)`. `navigation_demo.launch.py` still exposes `spawn_x` / `spawn_y` and forwards the upstream TurtleBot3 world defaults (`x=-2.0`, `y=-0.5`) for reproducibility.

The mission wrapper reuses [navigation_demo.launch.py](../ros2_nodes/launch/navigation_demo.launch.py) and enables `waypoint_navigator_node` with:

- `enable_ekf_localizer:=true`
- `enable_rviz:=true` by default in `run_gazebo_mission_demo.sh` (override with `ENABLE_RVIZ=false`)
- `nav_odom_topic:=/ekf_odom`
- `dwa_goal_threshold:=0.3`
- `waypoint_frame:=relative_start`
- `WAYPOINT_NAV_WAYPOINTS`: semicolon-delimited 2D mission
- `WAYPOINT_NAV_LOOP`: `true` or `false`
- `WAYPOINT_NAV_GOAL_TOLERANCE`: waypoint completion radius

`waypoint_navigator_node` also watches for stalled progress on the active waypoint. When odom does not move by at least `WAYPOINT_NAV_STUCK_PROGRESS_DISTANCE` within `WAYPOINT_NAV_STUCK_TIMEOUT`, it transitions into a simple recovery sequence: `cancel -> settle -> rotate -> backoff -> reissue goal`.

For observability during the mission demo:

- `/mission_status` reports the current mission state, active waypoint, and recovery phase
- `/mission_markers` visualizes the resolved route, active goal, and a text status overlay in RViz
- the launch-time `map -> odom` static transform lets RViz show `/map`, `/planned_path`, `/ekf_odom`, and `/mission_markers` in a single `map` fixed frame

Example looping square:

```bash
export WAYPOINT_NAV_FRAME=relative_start
export WAYPOINT_NAV_WAYPOINTS="0.4,0.0;0.0,0.4;0.4,0.0"
export WAYPOINT_NAV_LOOP=true
./ros2_nodes/launch/run_gazebo_mission_demo.sh
```

### Run the launch smoke test

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=89
export TURTLEBOT3_MODEL=burger
export ENABLE_RVIZ=false

./ros2_nodes/launch/run_navigation_smoke_test.sh
```

The smoke script wraps [run_gazebo_mission_demo.sh](../ros2_nodes/launch/run_gazebo_mission_demo.sh) and verifies:

- `waypoint_navigator_node` exposes `/mission_status`
- typed `ros2 topic echo /mission_markers visualization_msgs/msg/MarkerArray --once` succeeds
- `tf2_echo map odom` resolves the identity static transform published by `navigation_demo.launch.py`
- the mission finishes with `mission complete`, `cleared active navigation goal`, `planned path cleared`, and `published stop command after path clear`

The script defaults to `WAYPOINT_NAV_FRAME=relative_start` and the verified two-waypoint route `(0.4, 0.0) -> (0.1, 0.4)`, but it respects the same mission tuning environment variables as `run_gazebo_mission_demo.sh`.

### Send a goal

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}"
```

### Expected data flow

1. `slam_node` receives `/scan` and `/odom`, then publishes `/map`.
2. `ekf_localizer_node` converts `/odom` into `/ekf_odom` and `/ekf_pose`.
3. `waypoint_navigator_node` publishes the current mission waypoint on `/goal_pose`.
4. `path_planner_node` rebuilds A* from `/map` and publishes `/planned_path`.
5. `dwa_planner_node` follows `/planned_path` while using `/ekf_odom` for state and publishes `/cmd_vel`.
6. If waypoint progress stalls, `waypoint_navigator_node` clears the active goal, publishes a short recovery maneuver on `/cmd_vel`, and then republishes the same waypoint.
7. TurtleBot3 moves toward the requested goal in Gazebo.

### Verify topic wiring

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /ekf_odom --once
ros2 topic echo /map --once
ros2 topic echo /planned_path --qos-durability transient_local --once
ros2 topic echo /mission_status --once
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once
```

## Parameters

### `path_planner_node`

- `robot_radius` (default `0.4`): obstacle inflation radius \[m\]
- `map_occupancy_threshold` (compiled default `60`): occupied-cell threshold in the SLAM-produced `OccupancyGrid`
- `map_frame_id` (default `"map"`): output path frame

### `dwa_planner_node`

- `DWA_GOAL_THRESHOLD` (default `0.3`): terminal tolerance \[m\]
- `DWA_ROBOT_RADIUS` (default `0.35`): collision radius \[m\]
- `look_ahead_points` (default `10`): target index offset on global path
- `predict_time`, `dt`, `max_speed`, `max_yaw_rate`: standard DWA tuning terms

### `slam_node`

- `map_resolution` (default `0.2`): occupancy-grid cell size \[m\]
- `map_width` (default `300`): number of cells in x
- `map_height` (default `300`): number of cells in y
- `occupied_log_odds` / `free_log_odds`: Bayesian update weights
- `icp_max_iterations` (from algorithm defaults): ICP optimization limit

### `ekf_localizer_node`

- `EKF_INPUT_ODOM_TOPIC` (default `"/odom"`): raw odom input topic
- `EKF_OUTPUT_ODOM_TOPIC` (default `"/ekf_odom"`): filtered odom output topic
- `EKF_OUTPUT_POSE_TOPIC` (default `"/ekf_pose"`): filtered pose output topic

### `waypoint_navigator_node`

- `RUST_NAV_ODOM_TOPIC` (default `"/odom"` unless launch overrides it): mission odom input topic
- `WAYPOINT_NAV_WAYPOINTS` (default `"0.5,0.0;0.5,0.5;0.0,0.5"`): mission waypoints interpreted in the frame selected by `WAYPOINT_NAV_FRAME`
- `WAYPOINT_NAV_FRAME` (default `"map"`): `map` for absolute goals, `relative_start` for offsets from the initial odom pose
- `WAYPOINT_NAV_GOAL_TOLERANCE` (default `0.35`): distance threshold for waypoint completion \[m\]
- `WAYPOINT_NAV_LOOP` (default `false`): restart the mission after the last waypoint
- `WAYPOINT_NAV_STUCK_TIMEOUT` (default `6.0`): no-progress timeout before recovery begins \[s\]
- `WAYPOINT_NAV_STUCK_PROGRESS_DISTANCE` (default `0.05`): minimum odom motion that resets the stuck timer \[m\]
- `WAYPOINT_NAV_MAX_RECOVERY_ATTEMPTS` (default `2`): recovery retries allowed per waypoint before the mission fails
- `WAYPOINT_NAV_RECOVERY_SETTLE_SECONDS` (default `0.5`): stop-and-settle time after sending `navigation_cancel` \[s\]
- `WAYPOINT_NAV_RECOVERY_ROTATE_SECONDS` (default `1.4`): in-place rotation duration \[s\]
- `WAYPOINT_NAV_RECOVERY_BACKOFF_SECONDS` (default `0.9`): reverse-motion duration \[s\]
- `WAYPOINT_NAV_RECOVERY_ROTATE_SPEED` (default `0.7`): angular velocity during the rotate phase \[rad/s\]
- `WAYPOINT_NAV_RECOVERY_BACKOFF_SPEED` (default `0.08`): reverse linear speed magnitude during the backoff phase \[m/s\]

## Troubleshooting

- `run_gazebo_demo.sh` reports missing binaries:
  - Rebuild all ROS2 nodes in release mode. The launch flow intentionally uses `target/release` only.
- Topics appear to come from unrelated robots or old simulations:
  - Set a dedicated `ROS_DOMAIN_ID` before launch, for example `export ROS_DOMAIN_ID=42`.
- `path_planner_node` logs `planner is not ready yet; waiting for /map`:
  - Confirm `slam_node` is running and `/map` is being published.
- `path_planner_node` logs `robot pose from /odom is not available yet`:
  - Confirm the selected odom topic is publishing (`/odom` or `/ekf_odom` depending on launch mode).
- `A* failed for start=... goal=...`:
  - The goal may be outside the current map bounds or inside an occupied cell.
- No robot motion despite `/planned_path`:
  - Check `/cmd_vel` output from `dwa_planner_node` and verify TurtleBot3 Gazebo is subscribed to `/cmd_vel`.
- `ekf_localizer_node` is running but `/ekf_odom` stays silent:
  - Confirm raw `/odom` exists and that `EKF_INPUT_ODOM_TOPIC` matches it.
- Mission demo does not advance to the next waypoint:
  - Check the selected mission odom topic and confirm the robot is entering `WAYPOINT_NAV_GOAL_TOLERANCE` around the active waypoint.
- Mission demo repeatedly logs recovery attempts:
  - Lower `WAYPOINT_NAV_STUCK_TIMEOUT` only if odom is stable enough; otherwise increase it or reduce `WAYPOINT_NAV_STUCK_PROGRESS_DISTANCE` so slow-but-valid motion is not classified as stuck.
- RViz shows `/map` but not the robot or path:
  - Confirm the launch still includes the `map -> odom` static transform and that RViz is using the `map` fixed frame from [navigation_demo.rviz](../ros2_nodes/launch/navigation_demo.rviz).
