#!/bin/bash
# Launch TurtleBot3 Gazebo and the Rust navigation stack.
# Usage: ./ros2_nodes/launch/run_gazebo_demo.sh

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
LAUNCH_FILE="$ROOT_DIR/ros2_nodes/launch/navigation_demo.launch.py"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
export ENABLE_GAZEBO_GUI="${ENABLE_GAZEBO_GUI:-true}"
export PUBLISH_MAP_ODOM_TF="${PUBLISH_MAP_ODOM_TF:-false}"
export ENABLE_SLAM_CORRECTED_FRAME="${ENABLE_SLAM_CORRECTED_FRAME:-false}"
export ENABLE_SLAM_MAP_ODOM_TF="${ENABLE_SLAM_MAP_ODOM_TF:-$ENABLE_SLAM_CORRECTED_FRAME}"
export ENABLE_SLAM_GROUND_TRUTH_MONITOR="${ENABLE_SLAM_GROUND_TRUTH_MONITOR:-$ENABLE_SLAM_CORRECTED_FRAME}"
export RAW_ODOM_TOPIC="${RAW_ODOM_TOPIC:-/odom}"
export ENABLE_SLAM_INPUT_ODOM_BIAS="${ENABLE_SLAM_INPUT_ODOM_BIAS:-false}"
export BIASED_SLAM_ODOM_TOPIC="${BIASED_SLAM_ODOM_TOPIC:-/slam_input_odom}"
if [[ -z "${SLAM_INPUT_ODOM_TOPIC:-}" ]]; then
  if [[ "$ENABLE_SLAM_INPUT_ODOM_BIAS" == "true" ]]; then
    export SLAM_INPUT_ODOM_TOPIC="$BIASED_SLAM_ODOM_TOPIC"
  else
    export SLAM_INPUT_ODOM_TOPIC="$RAW_ODOM_TOPIC"
  fi
fi
export SLAM_INPUT_ODOM_XY_SCALE="${SLAM_INPUT_ODOM_XY_SCALE:-1.0}"
export SLAM_INPUT_ODOM_YAW_SCALE="${SLAM_INPUT_ODOM_YAW_SCALE:-1.0}"
export SLAM_INPUT_ODOM_YAW_BIAS_RAD="${SLAM_INPUT_ODOM_YAW_BIAS_RAD:-0.0}"
export SLAM_INPUT_ODOM_YAW_BIAS_PER_METER="${SLAM_INPUT_ODOM_YAW_BIAS_PER_METER:-0.0}"
export BASE_TF_ODOM_TOPIC="${BASE_TF_ODOM_TOPIC:-$RAW_ODOM_TOPIC}"
export SLAM_POSE_TOPIC="${SLAM_POSE_TOPIC:-/slam_pose}"
export SLAM_ODOM_TOPIC="${SLAM_ODOM_TOPIC:-/slam_odom}"
export SLAM_DIAGNOSTICS_TOPIC="${SLAM_DIAGNOSTICS_TOPIC:-/slam_diagnostics}"
export SLAM_GROUND_TRUTH_STATUS_TOPIC="${SLAM_GROUND_TRUTH_STATUS_TOPIC:-/slam_ground_truth_status}"
export GROUND_TRUTH_GZ_POSE_TOPIC="${GROUND_TRUTH_GZ_POSE_TOPIC:-/world/default/dynamic_pose/info}"
export GROUND_TRUTH_ENTITY_NAME="${GROUND_TRUTH_ENTITY_NAME:-${TURTLEBOT3_MODEL}}"
if [[ -z "${NAV_ODOM_TOPIC:-}" ]]; then
  if [[ "$ENABLE_SLAM_CORRECTED_FRAME" == "true" ]]; then
    export NAV_ODOM_TOPIC="$SLAM_ODOM_TOPIC"
  else
    export NAV_ODOM_TOPIC="$RAW_ODOM_TOPIC"
  fi
fi
if [[ -z "${NAV_GLOBAL_FRAME:-}" ]]; then
  if [[ "$ENABLE_SLAM_CORRECTED_FRAME" == "true" ]]; then
    export NAV_GLOBAL_FRAME="map"
  else
    export NAV_GLOBAL_FRAME="odom"
  fi
fi

required_bins=(
  "$ROOT_DIR/ros2_nodes/path_planner_node/target/release/path_planner_node"
  "$ROOT_DIR/ros2_nodes/dwa_planner_node/target/release/dwa_planner_node"
  "$ROOT_DIR/ros2_nodes/slam_node/target/release/slam_node"
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
EOF
  exit 1
fi

exec ros2 launch "$LAUNCH_FILE" \
  "turtlebot3_model:=${TURTLEBOT3_MODEL}" \
  "enable_gazebo_gui:=${ENABLE_GAZEBO_GUI}" \
  "publish_map_odom_tf:=${PUBLISH_MAP_ODOM_TF}" \
  "raw_odom_topic:=${RAW_ODOM_TOPIC}" \
  "slam_input_odom_topic:=${SLAM_INPUT_ODOM_TOPIC}" \
  "enable_slam_input_odom_bias:=${ENABLE_SLAM_INPUT_ODOM_BIAS}" \
  "slam_input_odom_xy_scale:=${SLAM_INPUT_ODOM_XY_SCALE}" \
  "slam_input_odom_yaw_scale:=${SLAM_INPUT_ODOM_YAW_SCALE}" \
  "slam_input_odom_yaw_bias_rad:=${SLAM_INPUT_ODOM_YAW_BIAS_RAD}" \
  "slam_input_odom_yaw_bias_per_meter:=${SLAM_INPUT_ODOM_YAW_BIAS_PER_METER}" \
  "base_tf_odom_topic:=${BASE_TF_ODOM_TOPIC}" \
  "nav_odom_topic:=${NAV_ODOM_TOPIC}" \
  "nav_global_frame:=${NAV_GLOBAL_FRAME}" \
  "enable_slam_corrected_frame:=${ENABLE_SLAM_CORRECTED_FRAME}" \
  "enable_slam_map_odom_tf:=${ENABLE_SLAM_MAP_ODOM_TF}" \
  "enable_slam_ground_truth_monitor:=${ENABLE_SLAM_GROUND_TRUTH_MONITOR}" \
  "slam_pose_topic:=${SLAM_POSE_TOPIC}" \
  "slam_odom_topic:=${SLAM_ODOM_TOPIC}" \
  "slam_diagnostics_topic:=${SLAM_DIAGNOSTICS_TOPIC}" \
  "ground_truth_gz_pose_topic:=${GROUND_TRUTH_GZ_POSE_TOPIC}" \
  "ground_truth_entity_name:=${GROUND_TRUTH_ENTITY_NAME}" \
  "slam_ground_truth_status_topic:=${SLAM_GROUND_TRUTH_STATUS_TOPIC}" \
  "enable_rviz:=${ENABLE_RVIZ}"
