#!/bin/bash
# Launch TurtleBot3 Gazebo, the Rust navigation stack, and the waypoint mission node.
# Usage:
#   WAYPOINT_NAV_WAYPOINTS="0.4,0.0;0.1,0.4" ./ros2_nodes/launch/run_gazebo_mission_demo.sh
# Waypoints default to offsets from the first odom pose seen by waypoint_navigator_node.

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
LAUNCH_FILE="$ROOT_DIR/ros2_nodes/launch/navigation_demo.launch.py"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export NAV_ODOM_TOPIC="${NAV_ODOM_TOPIC:-/ekf_odom}"
export WAYPOINT_NAV_WAYPOINTS="${WAYPOINT_NAV_WAYPOINTS:-0.4,0.0;0.1,0.4}"
export WAYPOINT_NAV_FRAME="${WAYPOINT_NAV_FRAME:-relative_start}"
export WAYPOINT_NAV_LOOP="${WAYPOINT_NAV_LOOP:-false}"
export WAYPOINT_NAV_GOAL_TOLERANCE="${WAYPOINT_NAV_GOAL_TOLERANCE:-0.35}"
export DWA_GOAL_THRESHOLD="${DWA_GOAL_THRESHOLD:-0.3}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
export ENABLE_GAZEBO_GUI="${ENABLE_GAZEBO_GUI:-true}"
export PUBLISH_MAP_ODOM_TF="${PUBLISH_MAP_ODOM_TF:-false}"
export NAV_GLOBAL_FRAME="${NAV_GLOBAL_FRAME:-odom}"

required_bins=(
  "$ROOT_DIR/ros2_nodes/path_planner_node/target/release/path_planner_node"
  "$ROOT_DIR/ros2_nodes/dwa_planner_node/target/release/dwa_planner_node"
  "$ROOT_DIR/ros2_nodes/slam_node/target/release/slam_node"
  "$ROOT_DIR/ros2_nodes/ekf_localizer_node/target/release/ekf_localizer_node"
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
  cargo build --release --manifest-path ros2_nodes/ekf_localizer_node/Cargo.toml
  cargo build --release --manifest-path ros2_nodes/waypoint_navigator_node/Cargo.toml
EOF
  exit 1
fi

exec ros2 launch "$LAUNCH_FILE" \
  "turtlebot3_model:=${TURTLEBOT3_MODEL}" \
  "enable_gazebo_gui:=${ENABLE_GAZEBO_GUI}" \
  "publish_map_odom_tf:=${PUBLISH_MAP_ODOM_TF}" \
  "nav_global_frame:=${NAV_GLOBAL_FRAME}" \
  "enable_rviz:=${ENABLE_RVIZ}" \
  "enable_ekf_localizer:=true" \
  "nav_odom_topic:=${NAV_ODOM_TOPIC}" \
  "dwa_goal_threshold:=${DWA_GOAL_THRESHOLD}" \
  "enable_waypoint_navigator:=true" \
  "waypoint_mission:=${WAYPOINT_NAV_WAYPOINTS}" \
  "waypoint_frame:=${WAYPOINT_NAV_FRAME}" \
  "waypoint_loop:=${WAYPOINT_NAV_LOOP}" \
  "waypoint_goal_tolerance:=${WAYPOINT_NAV_GOAL_TOLERANCE}"
