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
export NAV_ODOM_TOPIC="${NAV_ODOM_TOPIC:-/ekf_odom}"
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
TF_OUT="$TMP_DIR/tf_map_odom.txt"
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

capture_tf_identity() {
  local timeout_seconds="$1"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    timeout 8s ros2 run tf2_ros tf2_echo map odom >"$TF_OUT" 2>&1 || true
    if rg -q "Translation: \[0\.000, 0\.000, 0\.000\]" "$TF_OUT" &&
      rg -q "Rotation: in Quaternion \(xyzw\) \[0\.000, 0\.000, 0\.000, 1\.000\]" "$TF_OUT"; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out resolving map -> odom identity transform" >&2
  return 1
}

echo "Starting navigation smoke test on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
setsid "$MISSION_WRAPPER" >"$LAUNCH_LOG" 2>&1 &
launch_pid=$!

wait_for_log_pattern "observability topics: /mission_status and /mission_markers" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/mission_status" "std_msgs/msg/String" "$STATUS_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_topic_once "/mission_markers" "visualization_msgs/msg/MarkerArray" "$MARKERS_OUT" "$SMOKE_STARTUP_TIMEOUT"
capture_tf_identity "$SMOKE_STARTUP_TIMEOUT"

rg -q "data: status=" "$STATUS_OUT"
rg -q "ns: mission" "$MARKERS_OUT"
rg -q "frame_id: map" "$MARKERS_OUT"

wait_for_log_pattern "mission complete at waypoint" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "cleared active navigation goal" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "planned path cleared" "$SMOKE_MISSION_TIMEOUT"
wait_for_log_pattern "published stop command after path clear" "$SMOKE_MISSION_TIMEOUT"

capture_topic_once "/mission_status" "std_msgs/msg/String" "$STATUS_OUT" "$SMOKE_TOPIC_CAPTURE_TIMEOUT"
rg -q "data: status=completed" "$STATUS_OUT"

echo "Verified mission status topic:"
cat "$STATUS_OUT"
echo
echo "Verified mission markers topic and map -> odom static transform."
echo "Navigation smoke test passed."
