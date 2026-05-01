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
export ENABLE_SYNTHETIC_SCAN="${ENABLE_SYNTHETIC_SCAN:-false}"
export ENABLE_SLAM_CORRECTED_FRAME="${ENABLE_SLAM_CORRECTED_FRAME:-true}"
export ENABLE_SLAM_GROUND_TRUTH_MONITOR="${ENABLE_SLAM_GROUND_TRUTH_MONITOR:-true}"
export SMOKE_STARTUP_TIMEOUT="${SMOKE_STARTUP_TIMEOUT:-120}"

BASE_ID="${ROS_DOMAIN_ID_BASE:-200}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${REVALUATION_OUTPUT_DIR:-$ROOT_DIR/target/navigation_revaluation/$RUN_STAMP}"
SUMMARY_TSV="$OUTPUT_DIR/summary.tsv"

# name|waypoints|mission_timeout_sec
# Keep segments moderate: four+ waypoints often hit smoke mission timeout (stuck/recovery) in Gazebo.
if [[ "$ENABLE_SYNTHETIC_SCAN" == "true" ]]; then
  export WAYPOINT_NAV_GOAL_TOLERANCE="${WAYPOINT_NAV_GOAL_TOLERANCE:-0.25}"
  export DWA_GOAL_THRESHOLD="${DWA_GOAL_THRESHOLD:-0.22}"
  SCENARIOS=(
    "synthetic_straight|0.18,0.0|90"
    "synthetic_arc|0.15,0.0;0.1,0.15|90"
    "synthetic_corner|0.12,0.0;0.12,0.12|120"
  )
else
  SCENARIOS=(
    "short_default|0.4,0.0;0.1,0.4|150"
    "three_hops|0.45,0.0;0.35,0.45;0.05,0.5|200"
    "long_two_legs|0.55,0.0;0.2,0.55|200"
  )
fi

last_matching_line() {
  local pattern="$1"
  local file="$2"
  grep -E "$pattern" "$file" | tail -1 || true
}

extract_metric() {
  local line="$1"
  local key="$2"
  awk -v key="$key" '
    {
      for (i = 1; i <= NF; i++) {
        split($i, kv, "=")
        if (kv[1] == key) {
          print kv[2]
          exit
        }
      }
    }
  ' <<<"$line"
}

metric_delta() {
  local left="$1"
  local right="$2"
  if [[ -z "$left" || -z "$right" ]]; then
    return 0
  fi
  awk -v left="$left" -v right="$right" 'BEGIN { printf "%.3f", left - right }'
}

metric_gt() {
  local left="$1"
  local right="$2"
  awk -v left="$left" -v right="$right" 'BEGIN { exit !(left > right) }'
}

metric_lt() {
  local left="$1"
  local right="$2"
  awk -v left="$left" -v right="$right" 'BEGIN { exit !(left < right) }'
}

mkdir -p "$OUTPUT_DIR"
printf 'scenario\tros_domain_id\tresult\traw_xy_error\tslam_xy_error\traw_xy_rmse\tslam_xy_rmse\tslam_minus_raw_xy_rmse\timprovement_xy\tslam_better_xy\tlatest_gate_reason\tlatest_blend_alpha\ticp_total\ticp_accepted\ticp_acceptance_ratio\tmetric_gate\tlog\n' >"$SUMMARY_TSV"

echo "=== Corrected-frame revaluation matrix ==="
echo "ROOT_DIR=$ROOT_DIR"
echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "ENABLE_SYNTHETIC_SCAN=$ENABLE_SYNTHETIC_SCAN"
echo "WAYPOINT_NAV_GOAL_TOLERANCE=${WAYPOINT_NAV_GOAL_TOLERANCE:-default}"
echo "DWA_GOAL_THRESHOLD=${DWA_GOAL_THRESHOLD:-default}"
echo "REVALUATION_MAX_SLAM_XY_RMSE=${REVALUATION_MAX_SLAM_XY_RMSE:-unset}"
echo "REVALUATION_MAX_SLAM_MINUS_RAW_XY_RMSE=${REVALUATION_MAX_SLAM_MINUS_RAW_XY_RMSE:-unset}"
echo "REVALUATION_MIN_ICP_ACCEPTANCE_RATIO=${REVALUATION_MIN_ICP_ACCEPTANCE_RATIO:-unset}"
echo "OUTPUT_DIR=$OUTPUT_DIR"
echo

run_idx=0
failed=0
for entry in "${SCENARIOS[@]}"; do
  IFS='|' read -r name wps mission_to <<<"$entry"
  export WAYPOINT_NAV_WAYPOINTS="$wps"
  export SMOKE_MISSION_TIMEOUT="$mission_to"
  export ROS_DOMAIN_ID=$((BASE_ID + run_idx))
  run_idx=$((run_idx + 1))

  out="$OUTPUT_DIR/${name}.log"
  rm -f "$out"
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
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$name" \
      "$ROS_DOMAIN_ID" \
      "fail:$rc" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "" \
      "$out" >>"$SUMMARY_TSV"
  else
    gt_line="$(last_matching_line 'raw_xy_error=|improvement_xy=|slam_better_xy=' "$out")"
    diag_line="$(last_matching_line 'blend_alpha=|gate_reason=' "$out")"
    diag_stream_line="$(last_matching_line 'icp_acceptance_ratio=' "$out")"

    raw_xy_error="$(extract_metric "$gt_line" raw_xy_error)"
    slam_xy_error="$(extract_metric "$gt_line" slam_xy_error)"
    raw_xy_rmse="$(extract_metric "$gt_line" raw_xy_rmse)"
    slam_xy_rmse="$(extract_metric "$gt_line" slam_xy_rmse)"
    slam_minus_raw_xy_rmse="$(metric_delta "$slam_xy_rmse" "$raw_xy_rmse")"
    improvement_xy="$(extract_metric "$gt_line" improvement_xy)"
    slam_better_xy="$(extract_metric "$gt_line" slam_better_xy)"
    gate_reason="$(extract_metric "$diag_line" gate_reason)"
    blend_alpha="$(extract_metric "$diag_line" blend_alpha)"
    icp_total="$(extract_metric "$diag_stream_line" icp_total)"
    icp_accepted="$(extract_metric "$diag_stream_line" icp_accepted)"
    icp_acceptance_ratio="$(extract_metric "$diag_stream_line" icp_acceptance_ratio)"

    metric_gate="ok"
    if [[ -n "${REVALUATION_MAX_SLAM_XY_RMSE:-}" && -n "$slam_xy_rmse" ]] \
      && metric_gt "$slam_xy_rmse" "$REVALUATION_MAX_SLAM_XY_RMSE"; then
      metric_gate="slam_xy_rmse>${REVALUATION_MAX_SLAM_XY_RMSE}"
    fi
    if [[ "$metric_gate" == "ok" \
      && -n "${REVALUATION_MAX_SLAM_MINUS_RAW_XY_RMSE:-}" \
      && -n "$slam_minus_raw_xy_rmse" ]] \
      && metric_gt "$slam_minus_raw_xy_rmse" "$REVALUATION_MAX_SLAM_MINUS_RAW_XY_RMSE"; then
      metric_gate="slam_minus_raw_xy_rmse>${REVALUATION_MAX_SLAM_MINUS_RAW_XY_RMSE}"
    fi
    if [[ "$metric_gate" == "ok" && -n "${REVALUATION_MIN_ICP_ACCEPTANCE_RATIO:-}" ]]; then
      if [[ -z "$icp_acceptance_ratio" ]]; then
        metric_gate="missing_icp_acceptance_ratio"
      elif metric_lt "$icp_acceptance_ratio" "$REVALUATION_MIN_ICP_ACCEPTANCE_RATIO"; then
        metric_gate="icp_acceptance_ratio<${REVALUATION_MIN_ICP_ACCEPTANCE_RATIO}"
      fi
    fi

    if [[ "$metric_gate" == "ok" ]]; then
      echo "    RESULT: OK"
      result="ok"
    else
      echo "    RESULT: METRIC FAIL ($metric_gate)"
      result="metric_fail"
      failed=$((failed + 1))
    fi
    echo "    Final ground truth:"
    echo "      ${gt_line:-not captured}"
    echo "    Latest SLAM diagnostic:"
    echo "      ${diag_line:-not captured}"
    echo "    ICP summary:"
    echo "      ${diag_stream_line:-not captured}"
    echo "    Metric gate: $metric_gate"

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$name" \
      "$ROS_DOMAIN_ID" \
      "$result" \
      "$raw_xy_error" \
      "$slam_xy_error" \
      "$raw_xy_rmse" \
      "$slam_xy_rmse" \
      "$slam_minus_raw_xy_rmse" \
      "$improvement_xy" \
      "$slam_better_xy" \
      "$gate_reason" \
      "$blend_alpha" \
      "$icp_total" \
      "$icp_accepted" \
      "$icp_acceptance_ratio" \
      "$metric_gate" \
      "$out" >>"$SUMMARY_TSV"
  fi
  echo
done

if [[ "$failed" -eq 0 ]]; then
  echo "=== All scenarios passed ==="
  echo "Summary: $SUMMARY_TSV"
  exit 0
fi
echo "=== Done: $failed scenario(s) failed ===" >&2
echo "Summary: $SUMMARY_TSV"
exit 1
