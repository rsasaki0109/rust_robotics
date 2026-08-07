# Experiments

## Problem

Preset: `control-drone-trajectory-coupled-continuity`

Re-run the drone trajectory generator comparison with globally coupled closed-loop waypoint boundary synthesis to see whether multi-segment continuity changes the ranking against `minimum-snap-distance-scaled`.

## Shared Input

- families: `oval-cruise-coupled`, `figure-eight-climb-coupled`, `banked-diamond-coupled`
- variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- same quadrotor PD tracker for every variant
- boundary policy: solve closed-loop waypoint velocities globally so adjacent cubic spans preserve `C2` waypoint acceleration continuity, derive waypoint acceleration from that coupled solution, and use the adjacent coupled segment average as the waypoint jerk signal
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.47 s` across the compared variants
- metrics: generation runtime, tracking RMSE, jerk RMS, snap RMS, peak speed, peak acceleration

## Variant Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control drone_trajectory_variants_report_coupled_continuity_tradeoffs -- --nocapture`.

| Variant | Style | Mean gen us | Mean RMSE m | Mean jerk RMS | Mean snap RMS | Mean vmax m/s | Mean amax m/s^2 | Total s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `quintic-uniform` | fixed-duration-quintic | 486.49 | 15.3070 | 0.9460 | 0.0000 | 2.9209 | 2.2403 | 15.47 |
| `quintic-distance-scaled` | distance-scaled-quintic | 489.79 | 15.4363 | 0.9441 | 0.0000 | 2.8603 | 2.2047 | 15.47 |
| `minimum-snap-distance-scaled` | distance-scaled-minimum-snap | 1905.09 | 15.3624 | 0.9784 | 1.8045 | 2.8535 | 2.2047 | 15.47 |

## Case Comparison

| Family | Variant | RMSE m | Jerk RMS | Snap RMS | Max err m | vmax | amax |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `oval-cruise-coupled` | `quintic-uniform` | 23.0690 | 0.3633 | 0.0000 | 39.4315 | 2.548 | 1.169 |
| `oval-cruise-coupled` | `quintic-distance-scaled` | 22.5547 | 0.3726 | 0.0000 | 38.5740 | 2.388 | 1.102 |
| `oval-cruise-coupled` | `minimum-snap-distance-scaled` | 22.5622 | 0.3819 | 0.5700 | 38.5889 | 2.388 | 1.102 |
| `figure-eight-climb-coupled` | `quintic-uniform` | 0.7589 | 1.6901 | 0.0000 | 1.4270 | 3.200 | 3.759 |
| `figure-eight-climb-coupled` | `quintic-distance-scaled` | 1.6612 | 1.6753 | 0.0000 | 2.5964 | 3.179 | 3.720 |
| `figure-eight-climb-coupled` | `minimum-snap-distance-scaled` | 1.4187 | 1.7477 | 3.8050 | 2.1417 | 3.180 | 3.720 |
| `banked-diamond-coupled` | `quintic-uniform` | 22.0932 | 0.7846 | 0.0000 | 37.6807 | 3.014 | 1.793 |
| `banked-diamond-coupled` | `quintic-distance-scaled` | 22.0932 | 0.7846 | 0.0000 | 37.6807 | 3.014 | 1.793 |
| `banked-diamond-coupled` | `minimum-snap-distance-scaled` | 22.1063 | 0.8057 | 1.0383 | 37.7124 | 2.993 | 1.793 |

## Trade-offs

- The coupled-continuity preset sharply smooths the generated reference trajectories. Against the local accel+jerk preset, mean jerk RMS drops from `3.8070 -> 0.9441` for `quintic-distance-scaled` and from `4.5775 -> 0.9784` for `minimum-snap-distance-scaled`.
- Snap also collapses under the coupled boundary profile: the quintic branch degenerates to a cubic-equivalent solution with `0.0000` sampled snap RMS, while minimum-snap drops from `15.4840 -> 1.8045`.
- This smoother reference does **not** improve closed-loop tracking on the current PD controller. RMSE grows sharply on the oval and banked families, which means the next bottleneck is controller/feedforward mismatch rather than another local boundary heuristic.
- `minimum-snap-distance-scaled` is no longer obviously dominated on mean RMSE alone, but it remains far more expensive than the quintic branch and still does not justify promotion from experimental status.

## Core / Experiments

- This preset still promotes no new stable helper. The coupled boundary synthesis remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
