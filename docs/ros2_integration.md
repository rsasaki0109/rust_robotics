# ROS2 Integration with `safe_drive`

## Prerequisites

- ROS2 installation (`jazzy` feature is enabled in these node manifests)
- ROS2 environment sourced before build/run:
  - `source /opt/ros/jazzy/setup.bash`
- `safe_drive` repository available at:
  - `/workspace/ai_coding_ws/rust_robo_ws/safe_drive`
- Rust toolchain (stable) with `cargo`

If `cargo check` fails with ROS2 type support or environment errors, source ROS2 first and retry.

## Build Instructions

### Cargo (per node)

```bash
cargo check --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo check --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo check --manifest-path ros2_nodes/slam_node/Cargo.toml
```

### Colcon (workspace-style ROS2 integration)

Place these Rust packages in a colcon workspace and invoke:

```bash
colcon build
```

## Node Descriptions and Topics

### `path_planner_node`

- **Role**: Global path planning with A*
- **Subscriptions**:
  - `/goal_pose` (`geometry_msgs/PoseStamped`)
  - `/robot_pose` (`geometry_msgs/PoseStamped`)
- **Publications**:
  - `/planned_path` (`nav_msgs/Path`)

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
 /scan + /odom
      |
      v
  +-----------+          /map
  | slam_node | --------------------------+
  +-----------+                           |
                                          v
                                   +-------------------+
 /robot_pose + /goal_pose -------> | path_planner_node | ---> /planned_path
                                   +-------------------+            |
                                                                    v
 /scan + /odom ---------------------------------------------> +------------+
                                                              | dwa_planner|
                                                              +------------+
                                                                    |
                                                                    v
                                                                 /cmd_vel
```

## Parameters

### `path_planner_node`

- `grid_resolution` (default `0.5`): A* discretization resolution \[m\]
- `robot_radius` (default `0.4`): obstacle inflation radius \[m\]
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
