# RustRobotics — Phase 3: Publish & ROS2 Integration

## Overview

All algorithm work is complete (32+ new algorithms merged to main). This phase focuses on:

1. **Task A: Publish to crates.io** — make the library available to the Rust ecosystem
2. **Task B: ROS2 Integration** — create practical ROS2 nodes using safe_drive

---

## Task A: Publish to crates.io

### Prerequisites (already done)
- [x] LICENSE file exists (MIT)
- [x] description field in all Cargo.toml
- [x] repository URL set
- [x] version 0.1.0

### Steps

#### A1: Clean up for publishing

1. Check all crates compile independently:
```bash
for c in core planning localization control mapping slam viz; do
    cargo check -p "rust_robotics_${c}"
done
```

2. Ensure `vendor/nearest_neighbor` is NOT published — it's a local dependency. Check if any published crate depends on it directly. If so, make it optional or vendor it properly.

3. Check for any `path = ` dependencies that need to be version-only for crates.io:
```bash
grep -r 'path = ' crates/*/Cargo.toml
```
For crates.io, workspace path dependencies need both `path` AND `version`.

4. Run `cargo publish --dry-run` for each crate in dependency order:
```bash
cargo publish -p rust_robotics_core --dry-run
cargo publish -p rust_robotics_planning --dry-run
cargo publish -p rust_robotics_localization --dry-run
cargo publish -p rust_robotics_control --dry-run
cargo publish -p rust_robotics_mapping --dry-run
cargo publish -p rust_robotics_slam --dry-run
cargo publish -p rust_robotics_viz --dry-run
cargo publish -p rust_robotics --dry-run
```

#### A2: Fix any dry-run issues

Common issues:
- Missing `version` on workspace path dependencies
- `vendor/` crates not on crates.io — either publish them separately, vendor inline, or make optional
- Files excluded by `.gitignore` that are needed (e.g., test data)

#### A3: Publish (in dependency order)

```bash
cargo publish -p rust_robotics_core
# wait for index update
cargo publish -p rust_robotics_planning
cargo publish -p rust_robotics_localization
cargo publish -p rust_robotics_control
cargo publish -p rust_robotics_mapping
cargo publish -p rust_robotics_slam
cargo publish -p rust_robotics_viz
cargo publish -p rust_robotics
```

#### A4: Verify

```bash
# Test from a fresh project
cargo init /tmp/test_rust_robotics
cd /tmp/test_rust_robotics
echo 'rust_robotics = "0.1"' >> Cargo.toml
cargo build
```

---

## Task B: ROS2 Integration with safe_drive

### Context

The workspace already has:
- `safe_drive/` — Rust ROS2 bindings (by TIER IV)
- `ekf_localizer_ros2/` — working EKF ROS2 node example
- `safe_drive_msg/` — message transpiler
- `safe_drive_tutorial/` — tutorials

### Goal
Create a practical ROS2 navigation stack using rust_robotics algorithms:

#### B1: Path Planner ROS2 Node
**File**: `ros2_nodes/path_planner_node/`

A ROS2 node that:
- Subscribes to `/map` (nav_msgs/OccupancyGrid)
- Subscribes to `/goal_pose` (geometry_msgs/PoseStamped)
- Subscribes to `/robot_pose` (geometry_msgs/PoseStamped)
- Publishes `/planned_path` (nav_msgs/Path)
- Uses A* or DWA from rust_robotics_planning
- Replans on new goal or map update

```rust
// Pseudo structure
use safe_drive::node::Node;
use rust_robotics_planning::{AStarPlanner, AStarConfig};

fn main() {
    let node = Node::new("path_planner")?;
    let map_sub = node.create_subscriber("/map")?;
    let goal_sub = node.create_subscriber("/goal_pose")?;
    let path_pub = node.create_publisher("/planned_path")?;
    
    // On goal received: plan path using A*
    // Convert OccupancyGrid → obstacle points
    // Run planner
    // Convert Path2D → nav_msgs/Path
    // Publish
}
```

#### B2: Localization ROS2 Node (EKF)
**File**: Already exists at `ekf_localizer_ros2/`

Enhance the existing EKF node:
- Add UKF/IEKF as alternative filters (selectable via parameter)
- Add covariance visualization output
- Publish tf2 transforms

#### B3: DWA Local Planner Node
**File**: `ros2_nodes/dwa_planner_node/`

A ROS2 node for local planning:
- Subscribes to `/scan` (sensor_msgs/LaserScan)
- Subscribes to `/planned_path` (nav_msgs/Path)
- Subscribes to `/odom` (nav_msgs/Odometry)
- Publishes `/cmd_vel` (geometry_msgs/Twist)
- Uses DWA from rust_robotics_planning

#### B4: SLAM ROS2 Node
**File**: `ros2_nodes/slam_node/`

A ROS2 node for SLAM:
- Subscribes to `/scan` (sensor_msgs/LaserScan)
- Subscribes to `/odom` (nav_msgs/Odometry)
- Publishes `/map` (nav_msgs/OccupancyGrid)
- Uses ICP scan matching + occupancy grid mapping

#### B5: Launch File
**File**: `ros2_nodes/launch/navigation.launch.py`

Launch file connecting all nodes:
```
scan → slam_node → map → path_planner_node → planned_path → dwa_planner_node → cmd_vel
                          ↑                                        ↑
                     goal_pose                                    scan
```

#### B6: Documentation
**File**: `docs/ros2_integration.md`

- How to build the ROS2 nodes
- How to run the navigation stack
- Architecture diagram
- Parameter descriptions

### Build System for ROS2 Nodes

Each ROS2 node should be a separate Cargo package:
```
ros2_nodes/
├── path_planner_node/
│   ├── Cargo.toml  (depends on safe_drive, rust_robotics_planning)
│   └── src/main.rs
├── dwa_planner_node/
│   ├── Cargo.toml
│   └── src/main.rs
├── slam_node/
│   ├── Cargo.toml
│   └── src/main.rs
└── launch/
    └── navigation.launch.py
```

Use `colcon build` for ROS2 integration, or standalone `cargo build` for development.

---

## Priority Order

1. **A1-A3**: Publish to crates.io (can be done immediately)
2. **B1**: Path planner ROS2 node (highest practical value)
3. **B3**: DWA local planner (pairs with B1)
4. **B2**: Enhanced EKF node
5. **B4**: SLAM node
6. **B5-B6**: Launch file and docs

---

## Conventions

Same as before:
- Doc comments on public items, escape brackets `\[m\]`
- Config struct with Default + validate()
- No unsafe code in algorithm crates
- safe_drive nodes may use unsafe where required by FFI
