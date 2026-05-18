#!/bin/bash
# Re-evaluate corrected-frame navigation + ground truth across multiple waypoint missions.
# Invokes run_navigation_smoke_test.sh once per scenario (Gazebo + stack each time).
#
# Usage:
#   ./ros2_nodes/launch/run_navigation_revaluation_matrix.sh
# Optional:
#   ROS_DOMAIN_ID_BASE=210  # first run uses this ID, then increments per scenario
#   SLAM_REVALUATION_REPORT_DIR=reports/slam_revaluation
#   SLAM_REVALUATION_SUMMARY=reports/slam_revaluation/summary.md
#   SLAM_REVALUATION_LOG_DIR=reports/slam_revaluation/run_logs
#   SLAM_REVALUATION_PROFILE_SET=tuning  # baseline (default) or tuning
#   SLAM_REVALUATION_ODOM_PROFILE_SET=biased  # baseline (default) or biased
#   SLAM_REVALUATION_START_INDEX=0  # zero-based index into the expanded matrix
#   SLAM_REVALUATION_MAX_RUNS=0  # 0 means run all remaining rows
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
SUMMARY_REPORT="${SLAM_REVALUATION_SUMMARY:-$REPORT_DIR/navigation_revaluation_${REPORT_STAMP}.summary.md}"
RUN_LOG_DIR="${SLAM_REVALUATION_LOG_DIR:-$REPORT_DIR/navigation_revaluation_${REPORT_STAMP}_logs}"
PROFILE_SET="${SLAM_REVALUATION_PROFILE_SET:-baseline}"
ODOM_PROFILE_SET="${SLAM_REVALUATION_ODOM_PROFILE_SET:-baseline}"
START_INDEX="${SLAM_REVALUATION_START_INDEX:-0}"
MAX_RUNS="${SLAM_REVALUATION_MAX_RUNS:-0}"

# name|waypoints|mission_timeout_sec
# Keep segments moderate: four+ waypoints often hit smoke mission timeout (stuck/recovery) in Gazebo.
# rich_geometry_turns extends sustained forward motion past the LIDAR noise floor
# (~1.3 cm mean residual) so scan-to-scan ICP has something to correct. The shape
# mirrors three_hops (3 waypoints, all-diagonal segments) plus a return-home leg
# so total path is ~1.75 m versus three_hops's ~1.21 m. Earlier axis-aligned
# variants (0.55,0; 0.55,0.5; 0,0.5) made the robot oscillate before reaching
# waypoint 1; the all-diagonal version stays inside the working three_hops shape.
SCENARIOS=(
  "short_default|0.4,0.0;0.1,0.4|150"
  "three_hops|0.45,0.0;0.35,0.45;0.05,0.5|200"
  "long_two_legs|0.55,0.0;0.2,0.55|200"
  "rich_geometry_turns|0.55,0.05;0.20,0.55;0.0,0.0|240"
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
      "strict_error|reject weaker ICP matches earlier|SLAM_ICP_REJECT_ERROR=0.008"
      "strict_low_alpha|lower blend weight and reject weaker ICP matches earlier|SLAM_ICP_BLEND_ALPHA=0.10 SLAM_ICP_REJECT_ERROR=0.008"
      "yaw_loose_018|loosen yaw axis reject only; XY stays at default 0.011|SLAM_ICP_REJECT_ERROR_YAW=0.018"
      "yaw_loose_018_low_alpha|loosen yaw axis reject and lower yaw blend weight; XY default|SLAM_ICP_REJECT_ERROR_YAW=0.018 SLAM_ICP_BLEND_ALPHA_YAW=0.10"
    )
    ;;
  *)
    echo "Unknown SLAM_REVALUATION_PROFILE_SET=$PROFILE_SET (expected baseline or tuning)" >&2
    exit 2
    ;;
esac

case "$ODOM_PROFILE_SET" in
  baseline)
    ODOM_PROFILES=(
      "raw_realistic|use raw EKF odom as SLAM input|"
    )
    ;;
  biased)
    ODOM_PROFILES=(
      "raw_realistic|use raw EKF odom as SLAM input|"
      "odom_xy_scale_1pct|feed SLAM input odom with 1 percent relative XY scale drift|ENABLE_SLAM_INPUT_ODOM_BIAS=true SLAM_INPUT_ODOM_XY_SCALE=1.01"
      "odom_xy_scale_3pct|feed SLAM input odom with 3 percent relative XY scale drift|ENABLE_SLAM_INPUT_ODOM_BIAS=true SLAM_INPUT_ODOM_XY_SCALE=1.03"
      "odom_yaw_drift_1deg_per_m|feed SLAM input odom with 1 degree yaw drift per meter|ENABLE_SLAM_INPUT_ODOM_BIAS=true SLAM_INPUT_ODOM_YAW_BIAS_PER_METER=0.017453292519943295"
      "odom_yaw_drift_3deg_per_m|feed SLAM input odom with 3 degree yaw drift per meter|ENABLE_SLAM_INPUT_ODOM_BIAS=true SLAM_INPUT_ODOM_YAW_BIAS_PER_METER=0.05235987755982988"
      "odom_turn_yaw_scale_3pct|feed SLAM input odom with 3 percent relative yaw scale drift|ENABLE_SLAM_INPUT_ODOM_BIAS=true SLAM_INPUT_ODOM_YAW_SCALE=1.03"
    )
    ;;
  *)
    echo "Unknown SLAM_REVALUATION_ODOM_PROFILE_SET=$ODOM_PROFILE_SET (expected baseline or biased)" >&2
    exit 2
    ;;
esac

RUN_COUNT=$((${#ODOM_PROFILES[@]} * ${#ICP_PROFILES[@]} * ${#SCENARIOS[@]}))
if ! [[ "$START_INDEX" =~ ^[0-9]+$ && "$MAX_RUNS" =~ ^[0-9]+$ ]]; then
  echo "SLAM_REVALUATION_START_INDEX and SLAM_REVALUATION_MAX_RUNS must be non-negative integers." >&2
  exit 2
fi
if (( START_INDEX >= RUN_COUNT )); then
  echo "SLAM_REVALUATION_START_INDEX=$START_INDEX is outside the $RUN_COUNT run matrix." >&2
  exit 2
fi
SELECTED_RUN_COUNT=$((RUN_COUNT - START_INDEX))
if (( MAX_RUNS > 0 && MAX_RUNS < SELECTED_RUN_COUNT )); then
  SELECTED_RUN_COUNT=$MAX_RUNS
fi
LAST_DOMAIN_ID=$((BASE_ID + SELECTED_RUN_COUNT - 1))
if (( BASE_ID < 0 || LAST_DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID_BASE=$BASE_ID with $SELECTED_RUN_COUNT selected run(s) would use ROS_DOMAIN_ID $BASE_ID..$LAST_DOMAIN_ID." >&2
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

# Track the current in-flight run so that an EXIT/INT/TERM trap can emit a
# partial CSV/JSONL row when the script is killed mid-scenario (e.g. by a CI
# step timeout). Without this, the in-progress row never reaches the CSV.
# Note: the smoke test is launched as a background job (SMOKE_PID) and waited
# on with the `wait` builtin, because bash defers signal handling until a
# foreground child exits — `wait` is interruptible, so the trap fires promptly.
IN_FLIGHT=false
SMOKE_PID=
CUR_ODOM_PROFILE_NAME=
CUR_ODOM_PROFILE_DESC=
CUR_ODOM_PROFILE_ENV=
CUR_PROFILE_NAME=
CUR_PROFILE_DESC=
CUR_PROFILE_ENV=
CUR_SCENARIO_NAME=
CUR_WAYPOINTS=
CUR_MISSION_TIMEOUT=
CUR_DOMAIN_ID=
CUR_LOG=

emit_row_from_log() {
  # Extract metrics from $1 (log path) and append CSV+JSONL rows tagged with
  # the supplied exit_code/mission_completed. Safe to call on a partial log.
  local log_path="$1"
  local exit_code="$2"
  local mission_completed="$3"
  local gt_line diag_line gate_reason_counts diagnostics_status_counts
  if [[ -f "$log_path" ]]; then
    gt_line="$(last_data_line_with "slam_xy_error=" "$log_path")"
    diag_line="$(last_data_line_with "blend_alpha=" "$log_path")"
    gate_reason_counts="$(count_key_values_with_token "$log_path" "gate_reason=" "gate_reason")"
    diagnostics_status_counts="$(count_key_values_with_token "$log_path" "gate_reason=" "status")"
  else
    gt_line=""
    diag_line=""
    gate_reason_counts=""
    diagnostics_status_counts=""
  fi

  local samples raw_xy_error slam_xy_error improvement_xy slam_better_xy
  local raw_xy_rmse slam_xy_rmse raw_xy_max slam_xy_max
  local raw_yaw_error slam_yaw_error improvement_yaw slam_better_yaw
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

  local icp_status icp_error_mean icp_initial_error_mean icp_error_median
  local icp_error_p90 icp_inlier_ratio_5cm icp_relative_error_reduction
  local icp_iterations icp_converged scan_points blend_alpha blend_applied gate_reason
  icp_status="$(kv_get "$diag_line" "status")"
  icp_error_mean="$(kv_get "$diag_line" "icp_error_mean")"
  icp_initial_error_mean="$(kv_get "$diag_line" "icp_initial_error_mean")"
  icp_error_median="$(kv_get "$diag_line" "icp_error_median")"
  icp_error_p90="$(kv_get "$diag_line" "icp_error_p90")"
  icp_inlier_ratio_5cm="$(kv_get "$diag_line" "icp_inlier_ratio_5cm")"
  icp_relative_error_reduction="$(kv_get "$diag_line" "icp_relative_error_reduction")"
  icp_iterations="$(kv_get "$diag_line" "icp_iterations")"
  icp_converged="$(kv_get "$diag_line" "icp_converged")"
  scan_points="$(kv_get "$diag_line" "scan_points")"
  blend_alpha="$(kv_get "$diag_line" "blend_alpha")"
  blend_applied="$(kv_get "$diag_line" "blend_applied")"
  gate_reason="$(kv_get "$diag_line" "gate_reason")"

  write_csv_row "$CSV_REPORT" \
    "$CUR_ODOM_PROFILE_NAME" \
    "$CUR_ODOM_PROFILE_DESC" \
    "$CUR_ODOM_PROFILE_ENV" \
    "$CUR_PROFILE_NAME" \
    "$CUR_PROFILE_DESC" \
    "$CUR_PROFILE_ENV" \
    "$CUR_SCENARIO_NAME" \
    "$exit_code" \
    "$mission_completed" \
    "$CUR_DOMAIN_ID" \
    "$CUR_WAYPOINTS" \
    "$CUR_MISSION_TIMEOUT" \
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
    "$icp_initial_error_mean" \
    "$icp_error_median" \
    "$icp_error_p90" \
    "$icp_inlier_ratio_5cm" \
    "$icp_relative_error_reduction" \
    "$icp_iterations" \
    "$icp_converged" \
    "$scan_points" \
    "$blend_alpha" \
    "$blend_applied" \
    "$gate_reason" \
    "$gate_reason_counts" \
    "$diagnostics_status_counts"

  write_jsonl_row "$JSONL_REPORT" \
    "odom_profile" "$CUR_ODOM_PROFILE_NAME" \
    "odom_profile_description" "$CUR_ODOM_PROFILE_DESC" \
    "odom_profile_env" "$CUR_ODOM_PROFILE_ENV" \
    "profile" "$CUR_PROFILE_NAME" \
    "profile_description" "$CUR_PROFILE_DESC" \
    "profile_env" "$CUR_PROFILE_ENV" \
    "scenario" "$CUR_SCENARIO_NAME" \
    "exit_code" "$exit_code" \
    "mission_completed" "$mission_completed" \
    "ros_domain_id" "$CUR_DOMAIN_ID" \
    "waypoints" "$CUR_WAYPOINTS" \
    "mission_timeout_sec" "$CUR_MISSION_TIMEOUT" \
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
    "icp_initial_error_mean" "$icp_initial_error_mean" \
    "icp_error_median" "$icp_error_median" \
    "icp_error_p90" "$icp_error_p90" \
    "icp_inlier_ratio_5cm" "$icp_inlier_ratio_5cm" \
    "icp_relative_error_reduction" "$icp_relative_error_reduction" \
    "icp_iterations" "$icp_iterations" \
    "icp_converged" "$icp_converged" \
    "scan_points" "$scan_points" \
    "blend_alpha" "$blend_alpha" \
    "blend_applied" "$blend_applied" \
    "gate_reason" "$gate_reason" \
    "gate_reason_counts" "$gate_reason_counts" \
    "diagnostics_status_counts" "$diagnostics_status_counts"

  CUR_LAST_SAMPLES="$samples"
  CUR_LAST_RAW_XY="$raw_xy_error"
  CUR_LAST_SLAM_XY="$slam_xy_error"
  CUR_LAST_IMPROVEMENT_XY="$improvement_xy"
  CUR_LAST_SLAM_BETTER_XY="$slam_better_xy"
  CUR_LAST_ICP_STATUS="$icp_status"
  CUR_LAST_ICP_ERROR_MEAN="$icp_error_mean"
  CUR_LAST_ICP_ERROR_P90="$icp_error_p90"
  CUR_LAST_ICP_INLIER="$icp_inlier_ratio_5cm"
  CUR_LAST_BLEND_ALPHA="$blend_alpha"
  CUR_LAST_GATE_REASON="$gate_reason"
}

emit_partial_row_if_in_flight() {
  local trap_rc=$?
  if [[ "$IN_FLIGHT" != "true" ]]; then
    return 0
  fi
  IN_FLIGHT=false
  if [[ -n "${SMOKE_PID:-}" ]] && kill -0 "$SMOKE_PID" 2>/dev/null; then
    kill -TERM "$SMOKE_PID" 2>/dev/null || true
    # Give the smoke test a brief grace period to flush its logs.
    for _ in 1 2 3 4 5; do
      if ! kill -0 "$SMOKE_PID" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    if kill -0 "$SMOKE_PID" 2>/dev/null; then
      kill -KILL "$SMOKE_PID" 2>/dev/null || true
    fi
    wait "$SMOKE_PID" 2>/dev/null || true
    SMOKE_PID=
  fi
  echo "!!! Killed mid-scenario: emitting partial row for $CUR_ODOM_PROFILE_NAME/$CUR_PROFILE_NAME/$CUR_SCENARIO_NAME" >&2
  emit_row_from_log "$CUR_LOG" "killed" "false" || true
  # Disarm the trap so the explicit exit below does not re-enter this handler.
  # Then exit with the captured status so the caller (e.g. a CI step) still
  # sees the original termination cause.
  trap - EXIT INT TERM
  exit "$trap_rc"
}

trap emit_partial_row_if_in_flight EXIT INT TERM

mkdir -p "$(dirname "$CSV_REPORT")" "$(dirname "$JSONL_REPORT")" "$(dirname "$SUMMARY_REPORT")" "$RUN_LOG_DIR"
write_csv_row "$CSV_REPORT" \
  "odom_profile" \
  "odom_profile_description" \
  "odom_profile_env" \
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
  "icp_initial_error_mean" \
  "icp_error_median" \
  "icp_error_p90" \
  "icp_inlier_ratio_5cm" \
  "icp_relative_error_reduction" \
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
echo "ODOM_PROFILE_SET=$ODOM_PROFILE_SET"
echo "RUN_COUNT=$RUN_COUNT"
echo "START_INDEX=$START_INDEX"
echo "MAX_RUNS=$MAX_RUNS"
echo "SELECTED_RUN_COUNT=$SELECTED_RUN_COUNT"
echo "CSV_REPORT=$CSV_REPORT"
echo "JSONL_REPORT=$JSONL_REPORT"
echo "SUMMARY_REPORT=$SUMMARY_REPORT"
echo "RUN_LOG_DIR=$RUN_LOG_DIR"
echo

run_idx=0
selected_run_idx=0
failed=0
for odom_profile_entry in "${ODOM_PROFILES[@]}"; do
  IFS='|' read -r odom_profile_name odom_profile_description odom_profile_env <<<"$odom_profile_entry"
  odom_profile_env_args=()
  if [[ -n "$odom_profile_env" ]]; then
    read -r -a odom_profile_env_args <<<"$odom_profile_env"
  fi

  for profile_entry in "${ICP_PROFILES[@]}"; do
    IFS='|' read -r profile_name profile_description profile_env <<<"$profile_entry"
    profile_env_args=()
    if [[ -n "$profile_env" ]]; then
      read -r -a profile_env_args <<<"$profile_env"
    fi

    for entry in "${SCENARIOS[@]}"; do
      IFS='|' read -r name wps mission_to <<<"$entry"
      matrix_idx=$run_idx
      run_idx=$((run_idx + 1))
      if (( matrix_idx < START_INDEX )); then
        continue
      fi
      if (( selected_run_idx >= SELECTED_RUN_COUNT )); then
        continue
      fi

      export WAYPOINT_NAV_WAYPOINTS="$wps"
      export SMOKE_MISSION_TIMEOUT="$mission_to"
      export ROS_DOMAIN_ID=$((BASE_ID + selected_run_idx))
      printf -v run_label "%02d_%s_%s_%s" "$matrix_idx" "$odom_profile_name" "$profile_name" "$name"
      selected_run_idx=$((selected_run_idx + 1))

      out="$RUN_LOG_DIR/${run_label}.log"
      : >"$out"
      echo ">>> Odom profile: $odom_profile_name"
      echo "    $odom_profile_description"
      if [[ -n "$odom_profile_env" ]]; then
        echo "    Odom profile env: $odom_profile_env"
      fi
      echo ">>> ICP profile: $profile_name"
      echo "    $profile_description"
      if [[ -n "$profile_env" ]]; then
        echo "    ICP profile env: $profile_env"
      fi
      echo ">>> Scenario: $name"
      echo "    WAYPOINT_NAV_WAYPOINTS=$WAYPOINT_NAV_WAYPOINTS"
      echo "    SMOKE_MISSION_TIMEOUT=${SMOKE_MISSION_TIMEOUT}s ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

      CUR_ODOM_PROFILE_NAME="$odom_profile_name"
      CUR_ODOM_PROFILE_DESC="$odom_profile_description"
      CUR_ODOM_PROFILE_ENV="$odom_profile_env"
      CUR_PROFILE_NAME="$profile_name"
      CUR_PROFILE_DESC="$profile_description"
      CUR_PROFILE_ENV="$profile_env"
      CUR_SCENARIO_NAME="$name"
      CUR_WAYPOINTS="$WAYPOINT_NAV_WAYPOINTS"
      CUR_MISSION_TIMEOUT="$SMOKE_MISSION_TIMEOUT"
      CUR_DOMAIN_ID="$ROS_DOMAIN_ID"
      CUR_LOG="$out"
      IN_FLIGHT=true

      set +e
      env "${odom_profile_env_args[@]}" "${profile_env_args[@]}" "$SMOKE" >"$out" 2>&1 &
      SMOKE_PID=$!
      # Use `wait` (interruptible) so SIGTERM hitting this script during a
      # long-running smoke test fires the EXIT/TERM trap promptly. Bash
      # otherwise defers signal handling until a foreground job exits.
      wait "$SMOKE_PID"
      rc=$?
      SMOKE_PID=
      set -e

      mission_completed=false
      if [[ "$rc" -eq 0 ]] || grep -Fq "status=completed" "$out"; then
        mission_completed=true
      fi

      emit_row_from_log "$out" "$rc" "$mission_completed"
      IN_FLIGHT=false

      # Backward-compatible local aliases used by the post-run echo below.
      samples="$CUR_LAST_SAMPLES"
      raw_xy_error="$CUR_LAST_RAW_XY"
      slam_xy_error="$CUR_LAST_SLAM_XY"
      improvement_xy="$CUR_LAST_IMPROVEMENT_XY"
      slam_better_xy="$CUR_LAST_SLAM_BETTER_XY"
      icp_status="$CUR_LAST_ICP_STATUS"
      icp_error_mean="$CUR_LAST_ICP_ERROR_MEAN"
      icp_error_p90="$CUR_LAST_ICP_ERROR_P90"
      icp_inlier_ratio_5cm="$CUR_LAST_ICP_INLIER"
      blend_alpha="$CUR_LAST_BLEND_ALPHA"
      gate_reason="$CUR_LAST_GATE_REASON"

      if [[ "$rc" -ne 0 ]]; then
        echo "    RESULT: FAIL (exit $rc)"
        failed=$((failed + 1))
        tail -25 "$out" >&2 || true
      else
        echo "    RESULT: OK"
        echo "    Metrics: raw_xy_error=${raw_xy_error:-n/a} slam_xy_error=${slam_xy_error:-n/a} improvement_xy=${improvement_xy:-n/a} slam_better_xy=${slam_better_xy:-n/a}"
        echo "    ICP: status=${icp_status:-n/a} icp_error_mean=${icp_error_mean:-n/a} icp_error_p90=${icp_error_p90:-n/a} inlier_5cm=${icp_inlier_ratio_5cm:-n/a} blend_alpha=${blend_alpha:-n/a} gate_reason=${gate_reason:-n/a}"
      fi
      echo
    done
  done
done

{
  echo "# Corrected-frame SLAM revaluation summary"
  echo
  echo "- generated_at_utc: $REPORT_STAMP"
  echo "- profile_set: $PROFILE_SET"
  echo "- odom_profile_set: $ODOM_PROFILE_SET"
  echo "- total_run_count: $RUN_COUNT"
  echo "- start_index: $START_INDEX"
  echo "- max_runs: $MAX_RUNS"
  echo "- selected_run_count: $SELECTED_RUN_COUNT"
  echo "- turtlebot3_model: $TURTLEBOT3_MODEL"
  echo "- csv: $CSV_REPORT"
  echo "- jsonl: $JSONL_REPORT"
  echo "- logs: $RUN_LOG_DIR"
  echo
  echo '```text'
  python3 "$ROOT_DIR/scripts/summarize_slam_revaluation.py" "$CSV_REPORT"
  echo '```'
} >"$SUMMARY_REPORT"

echo "Reports written:"
echo "  CSV:   $CSV_REPORT"
echo "  JSONL: $JSONL_REPORT"
echo "  Summary: $SUMMARY_REPORT"
echo "  Logs:    $RUN_LOG_DIR"
echo

if [[ "$failed" -eq 0 ]]; then
  echo "=== All scenarios passed ==="
  exit 0
fi
echo "=== Done: $failed scenario(s) failed ===" >&2
exit 1
