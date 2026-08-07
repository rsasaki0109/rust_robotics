#!/usr/bin/env bash
# Benchmark regression gate.
#
# Runs a representative set of benchmark examples (default features disabled to
# keep CI lean), then compares each freshly generated CSV under docs/assets/
# against the version committed at HEAD. Functional columns must match; wall-
# clock columns are ignored by check_benchmark_gate.py.
#
# Note: running this script regenerates docs/assets/*.csv (and SVG panels) in
# the working tree. On a deterministic build these match the committed files,
# except for wall-clock columns, which the gate ignores.
#
# Usage: scripts/check_benchmark_gate.sh
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

# <example>:<csv>:<features>
BENCHMARKS=(
  "benchmark_rigid_body_backends:rigid-body-backend-benchmark.csv:planning"
  "benchmark_cbf_safety_filter:cbf-safety-filter.csv:control"
  "benchmark_hierarchical_mapf_scale:hierarchical-mapf-scale.csv:planning"
  "benchmark_mppi_unified:mppi-unified-benchmark.csv:control"
  "benchmark_racing_mppi_3d:racing-mppi-3d.csv:control"
  "benchmark_racing_quadrotor:racing-quadrotor.csv:control"
  "benchmark_admm_formation:admm-formation.csv:control"
  "benchmark_adap_rpf_metrics:adap-rpf-metrics-sweep.csv:control"
  "benchmark_branchout_closed_loop:branchout-closed-loop.csv:planning"
  "benchmark_conformal_coverage:conformal-sipp-coverage.csv:planning"
  "benchmark_traversal_risk_sweep:traversal-risk-weight-sweep.csv:planning"
  "benchmark_meta_control:meta-control.csv:control"
)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

status=0
for entry in "${BENCHMARKS[@]}"; do
  example="${entry%%:*}"
  rest="${entry#*:}"
  csv="${rest%%:*}"
  features="${rest#*:}"

  git show "HEAD:docs/assets/$csv" > "$TMP/$csv" 2>/dev/null || {
    echo "SKIP $example: no committed baseline docs/assets/$csv at HEAD"
    continue
  }

  echo "== $example (features: $features) =="
  cargo run -q -p rust_robotics --example "$example" --no-default-features --features "$features"

  if python3 scripts/check_benchmark_gate.py \
      --baseline "$TMP/$csv" --generated "docs/assets/$csv"; then
    :
  else
    echo "!! REGRESSION in $example (docs/assets/$csv)"
    status=1
  fi
done

if [ "$status" -eq 0 ]; then
  echo ""
  echo "Benchmark gate: all examples reproduced their committed baselines."
fi
exit "$status"
