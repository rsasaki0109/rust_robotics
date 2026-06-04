# CP-SIPP Reproduction Slice

This is the first RustRobotics reproduction slice for:

- Liang et al., "Time-aware Motion Planning in Dynamic Environments with
  Conformal Prediction" (L4DC 2026)
- Project page: <https://time-aware-planning.github.io/>

The implemented slice is a discrete-grid CP-SIPP wrapper:

1. Take predicted obstacle positions over a finite time horizon.
2. Linearly interpolate obstacle positions for sparse discrete predictions.
3. Estimate empirical safety confidence from calibration nonconformity scores.
4. Mark cells as dynamically unsafe when confidence is below the requested
   threshold.
5. Delegate interval search to the existing SIPP planner.

Run the confidence sweep:

```bash
cargo run -p rust_robotics --example benchmark_conformal_sipp --no-default-features --features planning
```

The sweep uses a 9x5 corridor with a predicted obstacle at the center lane
during `t=4..6`. Calibration scores are `[0.10, 0.60, 1.20]`.

Representative output:

```text
| Planner | Arrival t | Path length | Off-center steps | Min confidence | Violation bound | Avg plan us |
|---|---:|---:|---:|---:|---:|---:|
| Exact SIPP | 10 | 10.0 | 6 | - | - | varies |
| CP-SIPP c=0.0 | 8 | 8.0 | 0 | 0.00 | 1.00 | varies |
| CP-SIPP c=0.5 | 10 | 10.0 | 6 | 0.67 | 0.33 | varies |
| CP-SIPP c=0.7 | 11 | 10.0 | 5 | 1.00 | 0.00 | varies |
| CP-SIPP c=1.0 | 11 | 10.0 | 5 | 1.00 | 0.00 | varies |
```

Interpretation:

- `c=0.0` accepts every cell and returns the shortest route, but its
  trajectory confidence is zero where it crosses the prediction.
- `c=0.5` blocks only cells with very low empirical confidence, matching the
  exact-obstacle SIPP behavior in this scenario.
- `c>=0.7` also blocks neighboring low-margin cells, so the planner waits or
  detours farther to keep every waypoint at full empirical confidence.

This is not the complete paper system. It intentionally omits learned
forecasting, continuous occupancy geometry, and full trajectory-level conformal
calibration. The current trajectory-level violation number is a capped union
bound over returned waypoint confidences. Those are natural follow-up slices once
the discrete planner interface is stable.
