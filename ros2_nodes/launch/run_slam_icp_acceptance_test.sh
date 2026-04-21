#!/bin/bash
# Run slam_node alone with deterministic synthetic odom/scan data and verify
# that a clean scan-to-scan match passes the corrected-frame ICP gate.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
SLAM_BIN="$ROOT_DIR/ros2_nodes/slam_node/target/release/slam_node"
FEED_SCRIPT="$ROOT_DIR/ros2_nodes/launch/synthetic_slam_icp_feed.py"

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
# Keep this single-node smoke independent of stale FastDDS shared-memory locks.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export SLAM_INPUT_ODOM_TOPIC="${SLAM_INPUT_ODOM_TOPIC:-/synthetic_odom}"
export SLAM_OUTPUT_POSE_TOPIC="${SLAM_OUTPUT_POSE_TOPIC:-/synthetic_slam_pose}"
export SLAM_OUTPUT_ODOM_TOPIC="${SLAM_OUTPUT_ODOM_TOPIC:-/synthetic_slam_odom}"
export SLAM_DIAGNOSTICS_TOPIC="${SLAM_DIAGNOSTICS_TOPIC:-/synthetic_slam_diagnostics}"
export SLAM_CORRECTED_FRAME_ID="${SLAM_CORRECTED_FRAME_ID:-map}"
export SLAM_USE_CORRECTED_FRAME=true
export SLAM_ACCEPTANCE_TIMEOUT="${SLAM_ACCEPTANCE_TIMEOUT:-15}"

if [[ ! -x "$SLAM_BIN" ]]; then
  cat >&2 <<EOF
Missing release binary: $SLAM_BIN
Build it first:
  source /opt/ros/jazzy/setup.bash
  cargo build --release --manifest-path ros2_nodes/slam_node/Cargo.toml
EOF
  exit 1
fi

TMP_DIR="$(mktemp -d)"
SLAM_LOG="$TMP_DIR/slam_node.log"
FEED_LOG="$TMP_DIR/synthetic_feed.log"
slam_pid=""

cleanup() {
  local exit_code=$?
  if [[ -n "$slam_pid" ]] && kill -0 "$slam_pid" 2>/dev/null; then
    kill -INT -- "-$slam_pid" 2>/dev/null || kill "$slam_pid" 2>/dev/null || true
    wait "$slam_pid" 2>/dev/null || true
  fi

  if [[ "$exit_code" -ne 0 ]]; then
    echo
    echo "SLAM ICP acceptance test failed. slam_node log tail:" >&2
    tail -n 120 "$SLAM_LOG" >&2 || true
    echo
    echo "Synthetic feed log:" >&2
    cat "$FEED_LOG" >&2 || true
  fi

  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

echo "Starting synthetic SLAM ICP acceptance test on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
setsid "$SLAM_BIN" >"$SLAM_LOG" 2>&1 &
slam_pid=$!

sleep 0.5
if ! kill -0 "$slam_pid" 2>/dev/null; then
  echo "slam_node exited before the synthetic feed started" >&2
  exit 1
fi

python3 "$FEED_SCRIPT" \
  --odom-topic "$SLAM_INPUT_ODOM_TOPIC" \
  --diagnostics-topic "$SLAM_DIAGNOSTICS_TOPIC" \
  --timeout "$SLAM_ACCEPTANCE_TIMEOUT" \
  | tee "$FEED_LOG"

echo "SLAM ICP acceptance test passed."
