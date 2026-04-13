# RustRobotics — Phase 4: Gazebo Navigation Demo

## Project Status

100+ robotics algorithms implemented in Rust. ROS2 nodes (A* planner, DWA, SLAM) built with safe_drive and verified working. CI all green on main.

## Workspace Layout

```
rust_robo_ws/                         ← NOT a git repo, just a directory
├── rust_robotics/                    ← git repo (main project, THIS repo)
│   ├── crates/                       ← algorithm library (100+ algorithms)
│   │   ├── rust_robotics_core/
│   │   ├── rust_robotics_planning/
│   │   ├── rust_robotics_localization/
│   │   ├── rust_robotics_control/
│   │   ├── rust_robotics_mapping/
│   │   ├── rust_robotics_slam/
│   │   ├── rust_robotics_viz/
│   │   └── rust_robotics/            ← umbrella crate
│   ├── ros2_nodes/                   ← ROS2 nodes (NOT in workspace Cargo.toml)
│   │   ├── path_planner_node/        ← A* global planner (VERIFIED WORKING)
│   │   ├── dwa_planner_node/         ← DWA local planner (VERIFIED WORKING)
│   │   ├── slam_node/                ← ICP + occupancy grid (VERIFIED WORKING)
│   │   └── launch/
│   │       └── run_gazebo_demo.sh
│   ├── vendor/                       ← vendored deps (nearest_neighbor)
│   ├── docs/
│   └── .github/workflows/ci.yml
├── safe_drive/                       ← separate git repo (TIER IV ROS2 bindings)
├── safe_drive_msg/                   ← separate git repo (message transpiler)
├── safe_drive_tutorial/              ← separate git repo
├── ekf_localizer_ros2/               ← separate git repo (existing EKF node)
└── notes/
```

### Important: ros2_nodes are standalone Cargo packages
- They are NOT members of the rust_robotics workspace (not in root Cargo.toml)
- Each has its own Cargo.toml with `[workspace]` (empty, to prevent auto-detection)
- They depend on safe_drive via relative path `../../../safe_drive`
- They depend on rust_robotics crates via relative path `../../crates/rust_robotics_*`
- Build each individually: `cargo build --manifest-path ros2_nodes/*/Cargo.toml`
- Requires `source /opt/ros/jazzy/setup.bash` before building

---

## What's Already Working

### ROS2 Node Test Results (2026-04-14)

| Node | Build | Start | Functional Test |
|------|-------|-------|-----------------|
| path_planner_node | ✓ | ✓ | ✓ Generated 11-point path from (0,0) to (5,3) |
| dwa_planner_node | ✓ | ✓ | ✓ Subscribes to /scan, /odom, /planned_path → publishes /cmd_vel |
| slam_node | ✓ | ✓ | ✓ Subscribes to /scan, /odom → publishes /map |

### Integration Test (fake_node)
- turtlebot3_fake_node + path_planner_node + dwa_planner_node ran together
- Path planner received goal and produced path
- DWA received the path (with slight timing delay)

---

## TODO: Gazebo Full Navigation Demo

### System Requirements (ALREADY INSTALLED)
- ROS2 Jazzy
- ros-jazzy-turtlebot3-gazebo
- ros-jazzy-turtlebot3-bringup
- ros-jazzy-turtlebot3-description
- ros-jazzy-nav2-bringup
- ros-jazzy-ros-gz

### Task 1: Create a Proper Launch File

**File**: `ros2_nodes/launch/navigation_demo.launch.py`

Create a Python ROS2 launch file that starts everything:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    turtlebot3_model = 'burger'
    
    # Get paths to our rust nodes
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    return LaunchDescription([
        # 1. TurtleBot3 Gazebo world
        # Use: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        # OR inline the gazebo launch here
        
        # 2. Path planner node (our Rust A* planner)
        ExecuteProcess(
            cmd=[os.path.join(pkg_dir, 'path_planner_node/target/release/path_planner_node')],
            name='rust_path_planner',
            output='screen',
        ),
        
        # 3. DWA planner node (our Rust DWA)
        ExecuteProcess(
            cmd=[os.path.join(pkg_dir, 'dwa_planner_node/target/release/dwa_planner_node')],
            name='rust_dwa_planner',
            output='screen',
        ),
        
        # 4. SLAM node (our Rust SLAM)
        ExecuteProcess(
            cmd=[os.path.join(pkg_dir, 'slam_node/target/release/slam_node')],
            name='rust_slam',
            output='screen',
        ),
    ])
```

**Important**: Build in release mode for real-time performance:
```bash
source /opt/ros/jazzy/setup.bash
cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
```

### Task 2: Fix Topic Remapping for TurtleBot3

TurtleBot3 in Gazebo publishes these topics:
- `/scan` — LaserScan from LIDAR
- `/odom` — Odometry from wheel encoders
- `/cmd_vel` — Velocity commands (what we publish)
- `/imu` — IMU data

Our nodes already use these topic names, so no remapping needed. But verify:

1. Start Gazebo with TurtleBot3:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. In another terminal, check topics:
```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

3. Verify our nodes connect to the right topics.

### Task 3: Adapt Path Planner to Use /map from SLAM

Current path_planner_node uses a hardcoded static obstacle map. It should instead:
1. Subscribe to `/map` (nav_msgs/OccupancyGrid) from slam_node
2. Convert OccupancyGrid cells to obstacle points (ox, oy)
3. Rebuild A* planner when map updates

**Modify**: `ros2_nodes/path_planner_node/src/main.rs`

Change `create_static_obstacles()` to a map callback that converts OccupancyGrid:

```rust
fn occupancy_grid_to_obstacles(
    map: &nav_msgs::msg::OccupancyGrid,
    threshold: i8,  // cells above this value are obstacles (e.g., 50)
) -> (Vec<f64>, Vec<f64>) {
    let mut ox = Vec::new();
    let mut oy = Vec::new();
    let info = &map.info;
    let resolution = info.resolution as f64;
    let origin_x = info.origin.position.x;
    let origin_y = info.origin.position.y;
    let width = info.width as usize;
    
    for (i, &cell) in map.data.iter().enumerate() {
        if cell >= threshold {
            let gx = (i % width) as f64 * resolution + origin_x;
            let gy = (i / width) as f64 * resolution + origin_y;
            ox.push(gx);
            oy.push(gy);
        }
    }
    (ox, oy)
}
```

Add a `/map` subscriber and rebuild the planner when map arrives.

### Task 4: Add Robot Pose from /odom Instead of /robot_pose

Current path_planner_node subscribes to `/robot_pose` (PoseStamped). TurtleBot3 publishes `/odom` (Odometry). Change the subscription:

**Modify**: `ros2_nodes/path_planner_node/src/main.rs`

- Subscribe to `/odom` (nav_msgs/msg/Odometry) instead of `/robot_pose`
- Extract pose from `odom.pose.pose`

### Task 5: Connect Everything and Test in Gazebo

Run sequence:
```bash
# Terminal 1: Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Our SLAM node
./ros2_nodes/slam_node/target/release/slam_node

# Terminal 3: Our path planner
./ros2_nodes/path_planner_node/target/release/path_planner_node

# Terminal 4: Our DWA planner
./ros2_nodes/dwa_planner_node/target/release/dwa_planner_node

# Terminal 5: Send goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
```

Expected result:
1. SLAM node builds a map from /scan data
2. Path planner receives map + goal, plans A* path
3. DWA follows the path, publishes /cmd_vel
4. TurtleBot3 moves in Gazebo

### Task 6: Record a Demo GIF/Video

Use Gazebo screenshot or screen recording to capture:
1. TurtleBot3 in the Gazebo world
2. Goal published
3. Robot navigating to the goal

Save as `docs/gazebo_demo.gif` or `docs/gazebo_demo.mp4`.
Add to README.md under the ROS2 Integration section.

### Task 7: Update Documentation

Update `docs/ros2_integration.md`:
- Add Gazebo demo instructions
- Add screenshot/gif
- Add troubleshooting section

Update `README.md`:
- Add Gazebo demo gif/screenshot to the ROS2 section

---

## Node Architecture (after Task 3-4 modifications)

```
                    TurtleBot3 (Gazebo)
                    ┌──────────────────┐
                    │  /scan (LIDAR)   │──────────┐
                    │  /odom (wheels)  │────────┐  │
                    │  /cmd_vel (in)   │◄──┐    │  │
                    └──────────────────┘   │    │  │
                                           │    │  │
┌─────────────┐    /map                    │    │  │
│  slam_node  │◄───────────────────────────┼────┼──┘  /scan
│  (Rust ICP  │    /odom                   │    │
│  + OccGrid) │◄───────────────────────────┼────┘
│             │──────┐                     │
└─────────────┘      │ /map                │
                     ▼                     │
┌──────────────────────┐                   │
│  path_planner_node   │                   │
│  (Rust A*)           │                   │
│                      │   /planned_path   │
│  /odom ──► robot pos │──────────┐        │
│  /goal_pose ──► goal │          │        │
└──────────────────────┘          │        │
                                  ▼        │
                    ┌──────────────────┐   │
                    │ dwa_planner_node │   │
                    │ (Rust DWA)       │   │
                    │                  │───┘  /cmd_vel
                    │ /scan ──► obs    │
                    │ /odom ──► state  │
                    └──────────────────┘
```

---

## Build Commands

```bash
# Source ROS2 first!
source /opt/ros/jazzy/setup.bash

# Build all nodes in release mode
cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml

# Build the algorithm library (does NOT require ROS2)
cargo build --workspace
cargo test --workspace --lib --tests
```

---

## Conventions

### For ROS2 nodes:
- Use safe_drive for all ROS2 communication
- Feature: `features = ["jazzy"]` in Cargo.toml for safe_drive
- Each node is a standalone binary with its own Cargo.toml
- Use `pr_info!`, `pr_warn!`, `pr_error!` from safe_drive for logging
- Follow the pattern in `ekf_localizer_ros2/src/main.rs`

### For algorithm crates:
- Same as before: Config + Default + validate(), no unsafe, doc comments, inline tests
- Use `rust_robotics_core` types (Point2D, Path2D, etc.)
- Use `nalgebra` for matrix operations

---

## Reference Files

| What | File |
|------|------|
| safe_drive node pattern | `../ekf_localizer_ros2/src/main.rs` |
| Path planner node | `ros2_nodes/path_planner_node/src/main.rs` |
| DWA planner node | `ros2_nodes/dwa_planner_node/src/main.rs` |
| SLAM node | `ros2_nodes/slam_node/src/main.rs` |
| safe_drive API | `../safe_drive/src/` |
| A* planner API | `crates/rust_robotics_planning/src/a_star.rs` |
| DWA planner API | `crates/rust_robotics_planning/src/dwa.rs` |
| ICP API | `crates/rust_robotics_slam/src/icp_matching.rs` |
| Occupancy grid API | `crates/rust_robotics_mapping/src/occupancy_grid_map.rs` |
| ROS2 integration docs | `docs/ros2_integration.md` |
| Gazebo launch script | `ros2_nodes/launch/run_gazebo_demo.sh` |
