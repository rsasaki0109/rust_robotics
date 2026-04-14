#!/bin/bash
# Re-evaluate corrected-frame navigation + ground truth across multiple waypoint missions.
# Invokes run_navigation_smoke_test.sh once per scenario (Gazebo + stack each time).
#
# Usage:
#   ./ros2_nodes/launch/run_navigation_revaluation_matrix.sh
# Optional:
#   ROS_DOMAIN_ID_BASE=210  # first run uses this ID, then increments per scenario
#
# Scenarios edit below: name|WAYPOINT_NAV_WAYPOINTS|SMOKE_MISSION_TIMEOUT

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
SMOKE="$ROOT_DIR/ros2_nodes/launch/run_navigation_smoke_test.sh"

set +u
source /opt/ros/jazzy/setup.bash
set -u

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-false}"
export ENABLE_GAZEBO_GUI="${ENABLE_GAZEBO_GUI:-false}"
export ENABLE_SLAM_CORRECTED_FRAME="${ENABLE_SLAM_CORRECTED_FRAME:-true}"
export ENABLE_SLAM_GROUND_TRUTH_MONITOR="${ENABLE_SLAM_GROUND_TRUTH_MONITOR:-true}"
export SMOKE_STARTUP_TIMEOUT="${SMOKE_STARTUP_TIMEOUT:-120}"

BASE_ID="${ROS_DOMAIN_ID_BASE:-200}"

# name|waypoints|mission_timeout_sec
# Keep segments moderate: four+ waypoints often hit smoke mission timeout (stuck/recovery) in Gazebo.
SCENARIOS=(
  "short_default|0.4,0.0;0.1,0.4|150"
  "three_hops|0.45,0.0;0.35,0.45;0.05,0.5|200"
  "long_two_legs|0.55,0.0;0.2,0.55|200"
)

echo "=== Corrected-frame revaluation matrix ==="
echo "ROOT_DIR=$ROOT_DIR"
echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo

run_idx=0
failed=0
for entry in "${SCENARIOS[@]}"; do
  IFS='|' read -r name wps mission_to <<<"$entry"
  export WAYPOINT_NAV_WAYPOINTS="$wps"
  export SMOKE_MISSION_TIMEOUT="$mission_to"
  export ROS_DOMAIN_ID=$((BASE_ID + run_idx))
  run_idx=$((run_idx + 1))

  out="$(mktemp)"
  echo ">>> Scenario: $name"
  echo "    WAYPOINT_NAV_WAYPOINTS=$WAYPOINT_NAV_WAYPOINTS"
  echo "    SMOKE_MISSION_TIMEOUT=${SMOKE_MISSION_TIMEOUT}s ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  set +e
  "$SMOKE" >"$out" 2>&1
  rc=$?
  set -e
  if [[ "$rc" -ne 0 ]]; then
    echo "    RESULT: FAIL (exit $rc)"
    failed=$((failed + 1))
    tail -25 "$out" >&2 || true
  else
    echo "    RESULT: OK"
    echo "    Ground truth (last lines with improvement_xy):"
    rg 'raw_xy_error=|improvement_xy=|slam_better_xy=' "$out" | tail -6 || true
  fi
  echo
  rm -f "$out"
done

if [[ "$failed" -eq 0 ]]; then
  echo "=== All scenarios passed ==="
  exit 0
fi
echo "=== Done: $failed scenario(s) failed ===" >&2
exit 1
