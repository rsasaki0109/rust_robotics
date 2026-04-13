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
  - `/odom` (`nav_msgs/Odometry`)
  - `/map` (`nav_msgs/OccupancyGrid`)
- **Publications**:
  - `/planned_path` (`nav_msgs/Path`)
- **Notes**:
  - Rebuilds its A* planner whenever `/map` updates
  - Converts occupied grid cells into A* obstacles
  - Uses `/odom.pose.pose.position` as the robot start pose

### `dwa_planner_node`

- **Role**: Local trajectory planning and velocity command generation
- **Subscriptions**:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/odom` (`nav_msgs/Odometry`)
  - `/planned_path` (`nav_msgs/Path`)
- **Publications**:
  - `/cmd_vel` (`geometry_msgs/Twist`)

### `slam_node`

- **Role**: Scan matching and occupancy-grid mapping
- **Subscriptions**:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/odom` (`nav_msgs/Odometry`)
- **Publications**:
  - `/map` (`nav_msgs/OccupancyGrid`)

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
           /odom     /goal_pose ---> | dwa_planner |
                                     +--------------+
                                            |
                                            v
                                         /cmd_vel
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
- adds an extra `ros_gz_bridge` subscription so `/cmd_vel` accepts `geometry_msgs/Twist`
- waits 5 seconds for Gazebo topics to appear
- starts `slam_node`, `path_planner_node`, and `dwa_planner_node` from `target/release`

Demo video: [gazebo_demo.mp4](./gazebo_demo.mp4)

### Send a goal

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}"
```

### Expected data flow

1. `slam_node` receives `/scan` and `/odom`, then publishes `/map`.
2. `path_planner_node` rebuilds A* from `/map` and publishes `/planned_path`.
3. `dwa_planner_node` follows `/planned_path` while avoiding live laser obstacles and publishes `/cmd_vel`.
4. TurtleBot3 moves toward the requested goal in Gazebo.

### Verify topic wiring

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 topic echo /planned_path --qos-durability transient_local --once
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once
```

## Parameters

### `path_planner_node`

- `robot_radius` (default `0.4`): obstacle inflation radius \[m\]
- `map_occupancy_threshold` (compiled default `60`): occupied-cell threshold in the SLAM-produced `OccupancyGrid`
- `map_frame_id` (default `"map"`): output path frame

### `dwa_planner_node`

- `goal_threshold` (default `0.3`): terminal tolerance \[m\]
- `robot_radius` (default `0.35`): collision radius \[m\]
- `look_ahead_points` (default `10`): target index offset on global path
- `predict_time`, `dt`, `max_speed`, `max_yaw_rate`: standard DWA tuning terms

### `slam_node`

- `map_resolution` (default `0.2`): occupancy-grid cell size \[m\]
- `map_width` (default `300`): number of cells in x
- `map_height` (default `300`): number of cells in y
- `occupied_log_odds` / `free_log_odds`: Bayesian update weights
- `icp_max_iterations` (from algorithm defaults): ICP optimization limit

## Troubleshooting

- `run_gazebo_demo.sh` reports missing binaries:
  - Rebuild all ROS2 nodes in release mode. The launch flow intentionally uses `target/release` only.
- Topics appear to come from unrelated robots or old simulations:
  - Set a dedicated `ROS_DOMAIN_ID` before launch, for example `export ROS_DOMAIN_ID=42`.
- `path_planner_node` logs `planner is not ready yet; waiting for /map`:
  - Confirm `slam_node` is running and `/map` is being published.
- `path_planner_node` logs `robot pose from /odom is not available yet`:
  - Confirm TurtleBot3 Gazebo is publishing `/odom`.
- `A* failed for start=... goal=...`:
  - The goal may be outside the current map bounds or inside an occupied cell.
- No robot motion despite `/planned_path`:
  - Check `/cmd_vel` output from `dwa_planner_node` and verify TurtleBot3 Gazebo is subscribed to `/cmd_vel`.
