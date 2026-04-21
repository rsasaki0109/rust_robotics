#!/bin/bash
# Re-evaluate corrected-frame navigation + ground truth across multiple waypoint missions.
# Invokes run_navigation_smoke_test.sh once per scenario (Gazebo + stack each time).
#
# Usage:
#   ./ros2_nodes/launch/run_navigation_revaluation_matrix.sh
# Optional:
#   ROS_DOMAIN_ID_BASE=210  # first run uses this ID, then increments per scenario
#   SLAM_REVALUATION_REPORT_DIR=reports/slam_revaluation
#   SLAM_REVALUATION_PROFILE_SET=tuning  # baseline (default) or tuning
#
# Scenarios edit below: name|WAYPOINT_NAV_WAYPOINTS|SMOKE_MISSION_TIMEOUT
# Profiles edit below: name|description|env assignments

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
REPORT_DIR="${SLAM_REVALUATION_REPORT_DIR:-$ROOT_DIR/reports/slam_revaluation}"
REPORT_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
CSV_REPORT="${SLAM_REVALUATION_CSV:-$REPORT_DIR/navigation_revaluation_${REPORT_STAMP}.csv}"
JSONL_REPORT="${SLAM_REVALUATION_JSONL:-$REPORT_DIR/navigation_revaluation_${REPORT_STAMP}.jsonl}"
PROFILE_SET="${SLAM_REVALUATION_PROFILE_SET:-baseline}"

# name|waypoints|mission_timeout_sec
# Keep segments moderate: four+ waypoints often hit smoke mission timeout (stuck/recovery) in Gazebo.
SCENARIOS=(
  "short_default|0.4,0.0;0.1,0.4|150"
  "three_hops|0.45,0.0;0.35,0.45;0.05,0.5|200"
  "long_two_legs|0.55,0.0;0.2,0.55|200"
)

case "$PROFILE_SET" in
  baseline)
    ICP_PROFILES=(
      "default|built-in ICP gating defaults|"
    )
    ;;
  tuning)
    ICP_PROFILES=(
      "default|built-in ICP gating defaults|"
      "low_alpha|reduce accepted ICP correction weight|SLAM_ICP_BLEND_ALPHA=0.10"
      "strict_error|reject weaker ICP matches earlier|SLAM_ICP_REJECT_ERROR=0.011"
      "strict_low_alpha|lower blend weight and reject weaker ICP matches earlier|SLAM_ICP_BLEND_ALPHA=0.10 SLAM_ICP_REJECT_ERROR=0.011"
    )
    ;;
  *)
    echo "Unknown SLAM_REVALUATION_PROFILE_SET=$PROFILE_SET (expected baseline or tuning)" >&2
    exit 2
    ;;
esac

RUN_COUNT=$((${#ICP_PROFILES[@]} * ${#SCENARIOS[@]}))
LAST_DOMAIN_ID=$((BASE_ID + RUN_COUNT - 1))
if (( BASE_ID < 0 || LAST_DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID_BASE=$BASE_ID with $RUN_COUNT run(s) would use ROS_DOMAIN_ID $BASE_ID..$LAST_DOMAIN_ID." >&2
  echo "Fast DDS commonly requires ROS_DOMAIN_ID <= 232; choose a lower ROS_DOMAIN_ID_BASE." >&2
  exit 2
fi

csv_escape() {
  local value="${1:-}"
  value="${value//\"/\"\"}"
  printf '"%s"' "$value"
}

write_csv_row() {
  local file="$1"
  shift
  local first=true
  {
    local field
    for field in "$@"; do
      if [[ "$first" == "true" ]]; then
        first=false
      else
        printf ','
      fi
      csv_escape "$field"
    done
    printf '\n'
  } >>"$file"
}

json_escape() {
  local value="${1:-}"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  value="${value//$'\n'/\\n}"
  value="${value//$'\r'/\\r}"
  value="${value//$'\t'/\\t}"
  printf '%s' "$value"
}

write_jsonl_row() {
  local file="$1"
  shift
  local first=true
  {
    printf '{'
    while [[ "$#" -gt 0 ]]; do
      local key="$1"
      local value="$2"
      shift 2
      if [[ "$first" == "true" ]]; then
        first=false
      else
        printf ','
      fi
      printf '"%s":"%s"' "$(json_escape "$key")" "$(json_escape "$value")"
    done
    printf '}\n'
  } >>"$file"
}

last_data_line_with() {
  local token="$1"
  local file="$2"
  grep -F "$token" "$file" \
    | sed -n 's/^[[:space:]]*data:[[:space:]]*//p' \
    | sed "s/^'//; s/'$//" \
    | tail -1 || true
}

kv_get() {
  local line="$1"
  local key="$2"
  local token
  for token in $line; do
    if [[ "$token" == "$key="* ]]; then
      printf '%s' "${token#*=}"
      return 0
    fi
  done
  return 0
}

count_key_values_with_token() {
  local file="$1"
  local line_token="$2"
  local key="$3"
  local values
  values="$(
    grep -F "$line_token" "$file" \
      | sed -n 's/^[[:space:]]*data:[[:space:]]*//p' \
      | sed "s/^'//; s/'$//" \
      | tr ' ' '\n' \
      | sed -n "s/^$key=//p" \
      | sort \
      | uniq -c \
      | awk '{ if (out != "") out = out ";"; out = out $2 ":" $1 } END { print out }'
  )" || true
  printf '%s' "$values"
}

mkdir -p "$(dirname "$CSV_REPORT")" "$(dirname "$JSONL_REPORT")"
write_csv_row "$CSV_REPORT" \
  "profile" \
  "profile_description" \
  "profile_env" \
  "scenario" \
  "exit_code" \
  "mission_completed" \
  "ros_domain_id" \
  "waypoints" \
  "mission_timeout_sec" \
  "samples" \
  "raw_xy_error" \
  "slam_xy_error" \
  "improvement_xy" \
  "slam_better_xy" \
  "raw_xy_rmse" \
  "slam_xy_rmse" \
  "raw_xy_max" \
  "slam_xy_max" \
  "raw_yaw_error" \
  "slam_yaw_error" \
  "improvement_yaw" \
  "slam_better_yaw" \
  "icp_status" \
  "icp_error_mean" \
  "icp_iterations" \
  "icp_converged" \
  "scan_points" \
  "blend_alpha" \
  "blend_applied" \
  "gate_reason" \
  "gate_reason_counts" \
  "diagnostics_status_counts"
: >"$JSONL_REPORT"

echo "=== Corrected-frame revaluation matrix ==="
echo "ROOT_DIR=$ROOT_DIR"
echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "PROFILE_SET=$PROFILE_SET"
echo "CSV_REPORT=$CSV_REPORT"
echo "JSONL_REPORT=$JSONL_REPORT"
echo

run_idx=0
failed=0
for profile_entry in "${ICP_PROFILES[@]}"; do
  IFS='|' read -r profile_name profile_description profile_env <<<"$profile_entry"
  profile_env_args=()
  if [[ -n "$profile_env" ]]; then
    read -r -a profile_env_args <<<"$profile_env"
  fi

  for entry in "${SCENARIOS[@]}"; do
    IFS='|' read -r name wps mission_to <<<"$entry"
    export WAYPOINT_NAV_WAYPOINTS="$wps"
    export SMOKE_MISSION_TIMEOUT="$mission_to"
    export ROS_DOMAIN_ID=$((BASE_ID + run_idx))
    run_idx=$((run_idx + 1))

    out="$(mktemp)"
    echo ">>> Profile: $profile_name"
    echo "    $profile_description"
    if [[ -n "$profile_env" ]]; then
      echo "    Profile env: $profile_env"
    fi
    echo ">>> Scenario: $name"
    echo "    WAYPOINT_NAV_WAYPOINTS=$WAYPOINT_NAV_WAYPOINTS"
    echo "    SMOKE_MISSION_TIMEOUT=${SMOKE_MISSION_TIMEOUT}s ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    set +e
    env "${profile_env_args[@]}" "$SMOKE" >"$out" 2>&1
    rc=$?
    set -e

    mission_completed=false
    if [[ "$rc" -eq 0 ]] || grep -Fq "status=completed" "$out"; then
      mission_completed=true
    fi

    gt_line="$(last_data_line_with "slam_xy_error=" "$out")"
    diag_line="$(last_data_line_with "blend_alpha=" "$out")"
    gate_reason_counts="$(count_key_values_with_token "$out" "gate_reason=" "gate_reason")"
    diagnostics_status_counts="$(count_key_values_with_token "$out" "gate_reason=" "status")"

    samples="$(kv_get "$gt_line" "samples")"
    raw_xy_error="$(kv_get "$gt_line" "raw_xy_error")"
    slam_xy_error="$(kv_get "$gt_line" "slam_xy_error")"
    improvement_xy="$(kv_get "$gt_line" "improvement_xy")"
    slam_better_xy="$(kv_get "$gt_line" "slam_better_xy")"
    raw_xy_rmse="$(kv_get "$gt_line" "raw_xy_rmse")"
    slam_xy_rmse="$(kv_get "$gt_line" "slam_xy_rmse")"
    raw_xy_max="$(kv_get "$gt_line" "raw_xy_max")"
    slam_xy_max="$(kv_get "$gt_line" "slam_xy_max")"
    raw_yaw_error="$(kv_get "$gt_line" "raw_yaw_error")"
    slam_yaw_error="$(kv_get "$gt_line" "slam_yaw_error")"
    improvement_yaw="$(kv_get "$gt_line" "improvement_yaw")"
    slam_better_yaw="$(kv_get "$gt_line" "slam_better_yaw")"

    icp_status="$(kv_get "$diag_line" "status")"
    icp_error_mean="$(kv_get "$diag_line" "icp_error_mean")"
    icp_iterations="$(kv_get "$diag_line" "icp_iterations")"
    icp_converged="$(kv_get "$diag_line" "icp_converged")"
    scan_points="$(kv_get "$diag_line" "scan_points")"
    blend_alpha="$(kv_get "$diag_line" "blend_alpha")"
    blend_applied="$(kv_get "$diag_line" "blend_applied")"
    gate_reason="$(kv_get "$diag_line" "gate_reason")"

    write_csv_row "$CSV_REPORT" \
      "$profile_name" \
      "$profile_description" \
      "$profile_env" \
      "$name" \
      "$rc" \
      "$mission_completed" \
      "$ROS_DOMAIN_ID" \
      "$WAYPOINT_NAV_WAYPOINTS" \
      "$SMOKE_MISSION_TIMEOUT" \
      "$samples" \
      "$raw_xy_error" \
      "$slam_xy_error" \
      "$improvement_xy" \
      "$slam_better_xy" \
      "$raw_xy_rmse" \
      "$slam_xy_rmse" \
      "$raw_xy_max" \
      "$slam_xy_max" \
      "$raw_yaw_error" \
      "$slam_yaw_error" \
      "$improvement_yaw" \
      "$slam_better_yaw" \
      "$icp_status" \
      "$icp_error_mean" \
      "$icp_iterations" \
      "$icp_converged" \
      "$scan_points" \
      "$blend_alpha" \
      "$blend_applied" \
      "$gate_reason" \
      "$gate_reason_counts" \
      "$diagnostics_status_counts"

    write_jsonl_row "$JSONL_REPORT" \
      "profile" "$profile_name" \
      "profile_description" "$profile_description" \
      "profile_env" "$profile_env" \
      "scenario" "$name" \
      "exit_code" "$rc" \
      "mission_completed" "$mission_completed" \
      "ros_domain_id" "$ROS_DOMAIN_ID" \
      "waypoints" "$WAYPOINT_NAV_WAYPOINTS" \
      "mission_timeout_sec" "$SMOKE_MISSION_TIMEOUT" \
      "samples" "$samples" \
      "raw_xy_error" "$raw_xy_error" \
      "slam_xy_error" "$slam_xy_error" \
      "improvement_xy" "$improvement_xy" \
      "slam_better_xy" "$slam_better_xy" \
      "raw_xy_rmse" "$raw_xy_rmse" \
      "slam_xy_rmse" "$slam_xy_rmse" \
      "raw_xy_max" "$raw_xy_max" \
      "slam_xy_max" "$slam_xy_max" \
      "raw_yaw_error" "$raw_yaw_error" \
      "slam_yaw_error" "$slam_yaw_error" \
      "improvement_yaw" "$improvement_yaw" \
      "slam_better_yaw" "$slam_better_yaw" \
      "icp_status" "$icp_status" \
      "icp_error_mean" "$icp_error_mean" \
      "icp_iterations" "$icp_iterations" \
      "icp_converged" "$icp_converged" \
      "scan_points" "$scan_points" \
      "blend_alpha" "$blend_alpha" \
      "blend_applied" "$blend_applied" \
      "gate_reason" "$gate_reason" \
      "gate_reason_counts" "$gate_reason_counts" \
      "diagnostics_status_counts" "$diagnostics_status_counts"

    if [[ "$rc" -ne 0 ]]; then
      echo "    RESULT: FAIL (exit $rc)"
      failed=$((failed + 1))
      tail -25 "$out" >&2 || true
    else
      echo "    RESULT: OK"
      echo "    Metrics: raw_xy_error=${raw_xy_error:-n/a} slam_xy_error=${slam_xy_error:-n/a} improvement_xy=${improvement_xy:-n/a} slam_better_xy=${slam_better_xy:-n/a}"
      echo "    ICP: status=${icp_status:-n/a} icp_error_mean=${icp_error_mean:-n/a} blend_alpha=${blend_alpha:-n/a} gate_reason=${gate_reason:-n/a}"
    fi
    echo
    rm -f "$out"
  done
done

echo "Reports written:"
echo "  CSV:   $CSV_REPORT"
echo "  JSONL: $JSONL_REPORT"
echo

if [[ "$failed" -eq 0 ]]; then
  echo "=== All scenarios passed ==="
  exit 0
fi
echo "=== Done: $failed scenario(s) failed ===" >&2
exit 1
