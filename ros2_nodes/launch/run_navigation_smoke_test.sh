#!/bin/bash
# Run a local ROS2/Gazebo smoke test for the mission demo and verify the key
# observability and shutdown signals exposed by navigation_demo.launch.py.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
MISSION_WRAPPER="$ROOT_DIR/ros2_nodes/launch/run_gazebo_mission_demo.sh"

set +u
source /opt/ros/jazzy/setup.bash
set -u

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
export ENABLE_GAZEBO_GUI="${ENABLE_GAZEBO_GUI:-false}"
export ENABLE_SLAM_CORRECTED_FRAME="${ENABLE_SLAM_CORRECTED_FRAME:-false}"
export PUBLISH_MAP_ODOM_TF="${PUBLISH_MAP_ODOM_TF:-false}"
export RAW_ODOM_TOPIC="${RAW_ODOM_TOPIC:-/ekf_odom}"
export BASE_TF_ODOM_TOPIC="${BASE_TF_ODOM_TOPIC:-$RAW_ODOM_TOPIC}"
export SLAM_POSE_TOPIC="${SLAM_POSE_TOPIC:-/slam_pose}"
export SLAM_ODOM_TOPIC="${SLAM_ODOM_TOPIC:-/slam_odom}"
export ENABLE_SLAM_MAP_ODOM_TF="${ENABLE_SLAM_MAP_ODOM_TF:-$ENABLE_SLAM_CORRECTED_FRAME}"
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
export WAYPOINT_NAV_FRAME="${WAYPOINT_NAV_FRAME:-relative_start}"
export WAYPOINT_NAV_WAYPOINTS="${WAYPOINT_NAV_WAYPOINTS:-0.4,0.0;0.1,0.4}"
export WAYPOINT_NAV_LOOP="${WAYPOINT_NAV_LOOP:-false}"
export WAYPOINT_NAV_GOAL_TOLERANCE="${WAYPOINT_NAV_GOAL_TOLERANCE:-0.35}"
export DWA_GOAL_THRESHOLD="${DWA_GOAL_THRESHOLD:-0.3}"

SMOKE_STARTUP_TIMEOUT="${SMOKE_STARTUP_TIMEOUT:-90}"
SMOKE_MISSION_TIMEOUT="${SMOKE_MISSION_TIMEOUT:-120}"
SMOKE_TOPIC_CAPTURE_TIMEOUT="${SMOKE_TOPIC_CAPTURE_TIMEOUT:-20}"

TMP_DIR="$(mktemp -d)"
LAUNCH_LOG="$TMP_DIR/navigation_demo.log"
STATUS_OUT="$TMP_DIR/mission_status.txt"
MARKERS_OUT="$TMP_DIR/mission_markers.txt"
ODOM_OUT="$TMP_DIR/nav_odom.txt"
TF_NAV_OUT="$TMP_DIR/tf_nav.txt"
TF_MAP_ODOM_OUT="$TMP_DIR/tf_map_odom.txt"
MAP_OUT="$TMP_DIR/map.txt"
PATH_OUT="$TMP_DIR/planned_path.txt"
launch_pid=""

cleanup() {
  local exit_code=$?
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi

  if [[ "$exit_code" -ne 0 ]]; then
    echo
    echo "Smoke test failed. Launch log tail:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
  fi

  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

wait_for_log_pattern() {
  local pattern="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  until rg -q "$pattern" "$LAUNCH_LOG"; do
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for log pattern: $pattern" >&2
      return 1
    fi
    sleep 1
  done
}

capture_topic_once() {
  local topic="$1"
  local topic_type="$2"
  local output_file="$3"
  local timeout_seconds="$4"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if timeout "${SMOKE_TOPIC_CAPTURE_TIMEOUT}"s \
      ros2 topic echo "$topic" "$topic_type" --once >"$output_file" 2>>"$LAUNCH_LOG"; then
      if [[ -s "$output_file" ]]; then
        return 0
      fi
    fi
    sleep 1
  done

  echo "Timed out capturing topic $topic ($topic_type)" >&2
  return 1
}

capture_nav_tf() {
  local timeout_seconds="$1"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    capture_topic_once "$NAV_ODOM_TOPIC" "nav_msgs/msg/Odometry" "$ODOM_OUT" "$SMOKE_TOPIC_CAPTURE_TIMEOUT"
    local parent_frame
    local child_frame
    parent_frame="$(awk '/frame_id:/{print $2; exit}' "$ODOM_OUT" | tr -d '"' | tr -d "'")"
    child_frame="$(awk '/child_frame_id:/{print $2; exit}' "$ODOM_OUT" | tr -d '"' | tr -d "'")"

    if [[ -z "$parent_frame" || -z "$child_frame" ]]; then
      sleep 1
      continue
    fi

    timeout 8s ros2 run tf2_ros tf2_echo "$parent_frame" "$child_frame" >"$TF_NAV_OUT" 2>&1 || true
    if rg -q "Translation:" "$TF_NAV_OUT" && rg -q "Rotation: in Quaternion \(xyzw\)" "$TF_NAV_OUT"; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out resolving dynamic nav TF from $NAV_ODOM_TOPIC" >&2
  return 1
}

capture_tf_between_frames() {
  local parent_frame="$1"
  local child_frame="$2"
  local output_file="$3"
  local timeout_seconds="$4"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    timeout 8s ros2 run tf2_ros tf2_echo "$parent_frame" "$child_frame" >"$output_file" 2>&1 || true
    if rg -q "Translation:" "$output_file" && rg -q "Rotation: in Quaternion \(xyzw\)" "$output_file"; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out resolving TF $parent_frame -> $child_frame" >&2
  return 1
}

echo "Starting navigation smoke test on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
setsid "$MISSION_WRAPPER" >"$LAUNCH_LOG" 2>&1 &
launch_pid=$!

wait_for_log_pattern "observability topics: /mission_status and /mission_markers" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/map" "nav_msgs/msg/OccupancyGrid" "$MAP_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/planned_path" "nav_msgs/msg/Path" "$PATH_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/mission_status" "std_msgs/msg/String" "$STATUS_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/mission_markers" "visualization_msgs/msg/MarkerArray" "$MARKERS_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_nav_tf "$SMOKE_STARTUP_TIMEOUT"

parent_frame="$(awk '/frame_id:/{print $2; exit}' "$ODOM_OUT" | tr -d '"' | tr -d "'")"
if [[ -z "$parent_frame" ]]; then
  echo "Failed to infer nav odom parent frame from $NAV_ODOM_TOPIC" >&2
  exit 1
fi

rg -q "data: status=" "$STATUS_OUT"
rg -q "frame_id: $parent_frame" "$MAP_OUT"
rg -q "frame_id: $parent_frame" "$PATH_OUT"
rg -q "ns: mission" "$MARKERS_OUT"
rg -q "frame_id: $parent_frame" "$MARKERS_OUT"

if [[ "$ENABLE_SLAM_CORRECTED_FRAME" == "true" ]]; then
  capture_tf_between_frames "map" "odom" "$TF_MAP_ODOM_OUT" "$SMOKE_STARTUP_TIMEOUT"
fi

wait_for_log_pattern "mission complete at waypoint" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "cleared active navigation goal" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "planned path cleared" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "published stop command after path clear" "$SMOKE_MISSION_TIMEOUT"

capture_topic_once "/mission_status" "std_msgs/msg/String" "$STATUS_OUT" "$SMOKE_TOPIC_CAPTURE_TIMEOUT"
rg -q "data: status=completed" "$STATUS_OUT"

echo "Verified mission status topic:"
cat "$STATUS_OUT"
echo
echo "Verified /map, /planned_path, /mission_markers, and nav TF in frame '$parent_frame'."
if [[ "$ENABLE_SLAM_CORRECTED_FRAME" == "true" ]]; then
  echo "Verified dynamic TF map -> odom."
fi
echo "Navigation smoke test passed."
