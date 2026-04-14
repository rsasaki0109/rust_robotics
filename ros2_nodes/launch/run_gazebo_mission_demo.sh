#!/bin/bash
# Launch TurtleBot3 Gazebo, the Rust navigation stack, and the waypoint mission node.
# Usage:
#   WAYPOINT_NAV_WAYPOINTS="0.5,0.0;0.5,0.5;0.0,0.5" ./ros2_nodes/launch/run_gazebo_mission_demo.sh

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
LAUNCH_FILE="$ROOT_DIR/ros2_nodes/launch/navigation_demo.launch.py"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export WAYPOINT_NAV_WAYPOINTS="${WAYPOINT_NAV_WAYPOINTS:-0.5,0.0;0.5,0.5;0.0,0.5}"
export WAYPOINT_NAV_LOOP="${WAYPOINT_NAV_LOOP:-false}"
export WAYPOINT_NAV_GOAL_TOLERANCE="${WAYPOINT_NAV_GOAL_TOLERANCE:-0.35}"

required_bins=(
  "$ROOT_DIR/ros2_nodes/path_planner_node/target/release/path_planner_node"
  "$ROOT_DIR/ros2_nodes/dwa_planner_node/target/release/dwa_planner_node"
  "$ROOT_DIR/ros2_nodes/slam_node/target/release/slam_node"
  "$ROOT_DIR/ros2_nodes/waypoint_navigator_node/target/release/waypoint_navigator_node"
)

missing=0
for bin in "${required_bins[@]}"; do
  if [[ ! -x "$bin" ]]; then
    echo "Missing release binary: $bin" >&2
    missing=1
  fi
done

if [[ "$missing" -ne 0 ]]; then
  cat >&2 <<'EOF'
Build the ROS2 nodes in release mode first:
  cargo build --release --manifest-path ros2_nodes/path_planner_node/Cargo.toml
  cargo build --release --manifest-path ros2_nodes/dwa_planner_node/Cargo.toml
  cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
  cargo build --release --manifest-path ros2_nodes/waypoint_navigator_node/Cargo.toml
EOF
  exit 1
fi

exec python3 "$LAUNCH_FILE" \
  "turtlebot3_model:=${TURTLEBOT3_MODEL}" \
  "enable_waypoint_navigator:=true" \
  "waypoint_mission:=${WAYPOINT_NAV_WAYPOINTS}" \
  "waypoint_loop:=${WAYPOINT_NAV_LOOP}" \
  "waypoint_goal_tolerance:=${WAYPOINT_NAV_GOAL_TOLERANCE}"
