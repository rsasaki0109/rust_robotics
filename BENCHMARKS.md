# Benchmark Regression Gate

The workspace ships a deterministic benchmark suite and guards it in CI so a
change cannot silently degrade a published algorithm. Reproducible functional
metrics (path length, success rate, residual norms, collisions, clearance, lap
completion) are pinned; wall-clock measurements are reported but not compared,
because they are not portable across machines.

## How it works

`scripts/check_benchmark_gate.sh` runs a representative set of benchmark
examples with default features disabled, then compares each freshly generated
`docs/assets/*.csv` against the version committed at `HEAD`:

- Header names, column order, and row count must match exactly.
- Numeric cells must match to `1e-6` (relative + absolute) tolerance.
- Columns whose header ends in `_ms` / `_us` / `_ns` (e.g. `elapsed_ms`,
  `mean_plan_time_us`) are ignored as wall-clock noise.
- Boolean / string cells must match exactly.

The CI job `benchmark-gate` in `.github/workflows/ci.yml` runs this script on
every push and pull request. `scripts/check_benchmark_gate.py` is the reusable
comparator (usable standalone: `--baseline <committed.csv> --generated
<fresh.csv>`).

## Pinned suite (12 examples)

| Example | CSV | Feature |
| --- | --- | --- |
| `benchmark_rigid_body_backends` | `rigid-body-backend-benchmark.csv` | planning |
| `benchmark_hierarchical_mapf_scale` | `hierarchical-mapf-scale.csv` | planning |
| `benchmark_branchout_closed_loop` | `branchout-closed-loop.csv` | planning |
| `benchmark_conformal_coverage` | `conformal-sipp-coverage.csv` | planning |
| `benchmark_traversal_risk_sweep` | `traversal-risk-weight-sweep.csv` | planning |
| `benchmark_cbf_safety_filter` | `cbf-safety-filter.csv` | control |
| `benchmark_mppi_unified` | `mppi-unified-benchmark.csv` | control |
| `benchmark_racing_mppi_3d` | `racing-mppi-3d.csv` | control |
| `benchmark_racing_quadrotor` | `racing-quadrotor.csv` | control |
| `benchmark_admm_formation` | `admm-formation.csv` | control |
| `benchmark_adap_rpf_metrics` | `adap-rpf-metrics-sweep.csv` | control |
| `benchmark_meta_control` | `meta-control.csv` | control |

## Running locally

```bash
./scripts/check_benchmark_gate.sh
```

Deterministic runs reproduce the committed baselines, so after a green run only
wall-clock columns in the regenerated `docs/assets/*.csv` differ from `HEAD`
(ignore those diffs, or run with a clean checkout for a bit-for-bit clean tree).

## Adding a benchmark to the gate

1. Make the example write a deterministic CSV to `docs/assets/` (seeded PRNG,
   no wall-clock-derived functional columns).
2. Add an `<example>:<csv>:<feature>` line to the `BENCHMARKS` array in
   `scripts/check_benchmark_gate.sh`.
3. Commit the CSV so `HEAD` holds the new baseline.
4. Verify with `./scripts/check_benchmark_gate.sh`.

## What this does not measure

- Wall-clock throughput (solve time, plan time) — reported by the benchmarks
  but intentionally not pinned here. A dedicated benchmark harness with
  controlled hardware is the future home for those numbers.
- Memory / binary size — tracked separately by the embedded build jobs.
