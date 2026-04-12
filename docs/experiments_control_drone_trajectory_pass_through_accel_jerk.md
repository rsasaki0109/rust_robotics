# Experiments

## Problem

Preset: `control-drone-trajectory-pass-through-accel-jerk`

Re-run the drone trajectory generator comparison with non-zero pass-through waypoint velocities, finite-difference waypoint accelerations, and finite-difference waypoint jerk to see whether a richer boundary model changes the ranking against `minimum-snap-distance-scaled`.

## Shared Input

- families: `oval-cruise-accel-jerk`, `figure-eight-climb-accel-jerk`, `banked-diamond-accel-jerk`
- variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- same quadrotor PD tracker for every variant
- boundary policy: waypoint velocities from adjacent segment chords, waypoint accelerations from local velocity finite differences, waypoint jerk from local acceleration finite differences
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.47 s` across the compared variants
- metrics: generation runtime, tracking RMSE, jerk RMS, snap RMS, peak speed, peak acceleration

## Variant Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control drone_trajectory_variants_report_pass_through_accel_jerk_tradeoffs -- --nocapture`.

| Variant | Style | Mean gen us | Mean RMSE m | Mean jerk RMS | Mean snap RMS | Mean vmax m/s | Mean amax m/s^2 | Total s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `quintic-uniform` | fixed-duration-quintic | 542.63 | 10.0424 | 3.8517 | 10.0410 | 3.6669 | 3.0111 | 15.47 |
| `quintic-distance-scaled` | distance-scaled-quintic | 542.30 | 9.7819 | 3.8070 | 9.9684 | 3.5606 | 2.9790 | 15.47 |
| `minimum-snap-distance-scaled` | distance-scaled-minimum-snap | 2211.26 | 10.3186 | 4.5775 | 15.4840 | 3.9817 | 3.7805 | 15.47 |

## Case Comparison

| Family | Variant | RMSE m | Jerk RMS | Snap RMS | Max err m | vmax | amax |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `oval-cruise-accel-jerk` | `quintic-uniform` | 15.9187 | 2.4159 | 6.2781 | 27.2263 | 3.118 | 1.938 |
| `oval-cruise-accel-jerk` | `quintic-distance-scaled` | 15.4373 | 2.4141 | 6.3892 | 26.2529 | 2.902 | 1.905 |
| `oval-cruise-accel-jerk` | `minimum-snap-distance-scaled` | 15.4852 | 2.9086 | 9.8687 | 26.4416 | 3.152 | 2.408 |
| `figure-eight-climb-accel-jerk` | `quintic-uniform` | 1.9312 | 5.1128 | 14.3285 | 4.0734 | 3.862 | 4.204 |
| `figure-eight-climb-accel-jerk` | `quintic-distance-scaled` | 1.6313 | 4.9807 | 13.9997 | 3.0296 | 3.759 | 4.141 |
| `figure-eight-climb-accel-jerk` | `minimum-snap-distance-scaled` | 2.9083 | 6.0156 | 21.7571 | 5.9450 | 4.319 | 5.297 |
| `banked-diamond-accel-jerk` | `quintic-uniform` | 12.2772 | 4.0263 | 9.5164 | 21.1921 | 4.020 | 2.891 |
| `banked-diamond-accel-jerk` | `quintic-distance-scaled` | 12.2772 | 4.0263 | 9.5164 | 21.1921 | 4.020 | 2.891 |
| `banked-diamond-accel-jerk` | `minimum-snap-distance-scaled` | 12.5623 | 4.8083 | 14.8261 | 21.5722 | 4.474 | 3.637 |

## Trade-offs

- Even with finite-difference waypoint jerk, `quintic-distance-scaled` still wins the current preset. It remains much cheaper than `minimum-snap-distance-scaled` and still leads on mean RMSE, jerk RMS, snap RMS, peak speed, and peak acceleration.
- The jerk-enriched preset changed the minimum-snap metrics slightly, but it did not improve the ranking enough to overcome its runtime cost or quality deficit on the current controller/preset pair.
- The quintic rows are effectively unchanged from the acceleration-only preset because the quintic generator only consumes waypoint position, velocity, and acceleration boundaries; the added jerk term only affects the minimum-snap generator.

## Core / Experiments

- This preset still promotes no new stable helper. The jerk synthesis remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
