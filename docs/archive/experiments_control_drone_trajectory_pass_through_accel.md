# Experiments

## Problem

Preset: `control-drone-trajectory-pass-through-accel`

Re-run the drone trajectory generator comparison with non-zero pass-through waypoint velocities and finite-difference waypoint accelerations to see whether a richer boundary model changes the ranking against `minimum-snap-distance-scaled`.

## Shared Input

- families: `oval-cruise-accel`, `figure-eight-climb-accel`, `banked-diamond-accel`
- variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- same quadrotor PD tracker for every variant
- boundary policy: waypoint velocities from adjacent segment chords, plus waypoint accelerations from local finite differences of those velocities
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.47 s` across the compared variants
- metrics: generation runtime, tracking RMSE, jerk RMS, snap RMS, peak speed, peak acceleration

## Variant Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control drone_trajectory_variants_report_pass_through_accel_tradeoffs -- --nocapture`.

| Variant | Style | Mean gen us | Mean RMSE m | Mean jerk RMS | Mean snap RMS | Mean vmax m/s | Mean amax m/s^2 | Total s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `quintic-uniform` | fixed-duration-quintic | 596.10 | 10.0424 | 3.8517 | 10.0410 | 3.6669 | 3.0111 | 15.47 |
| `quintic-distance-scaled` | distance-scaled-quintic | 600.26 | 9.7819 | 3.8070 | 9.9684 | 3.5606 | 2.9790 | 15.47 |
| `minimum-snap-distance-scaled` | distance-scaled-minimum-snap | 2672.31 | 10.3241 | 4.5766 | 15.4774 | 3.9820 | 3.7787 | 15.47 |

## Case Comparison

| Family | Variant | RMSE m | Jerk RMS | Snap RMS | Max err m | vmax | amax |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `oval-cruise-accel` | `quintic-uniform` | 15.9187 | 2.4159 | 6.2781 | 27.2263 | 3.118 | 1.938 |
| `oval-cruise-accel` | `quintic-distance-scaled` | 15.4373 | 2.4141 | 6.3892 | 26.2529 | 2.902 | 1.905 |
| `oval-cruise-accel` | `minimum-snap-distance-scaled` | 15.4867 | 2.9046 | 9.8293 | 26.4446 | 3.150 | 2.405 |
| `figure-eight-climb-accel` | `quintic-uniform` | 1.9312 | 5.1128 | 14.3285 | 4.0734 | 3.862 | 4.204 |
| `figure-eight-climb-accel` | `quintic-distance-scaled` | 1.6313 | 4.9807 | 13.9997 | 3.0296 | 3.759 | 4.141 |
| `figure-eight-climb-accel` | `minimum-snap-distance-scaled` | 2.9222 | 6.0168 | 21.7711 | 5.9772 | 4.321 | 5.296 |
| `banked-diamond-accel` | `quintic-uniform` | 12.2772 | 4.0263 | 9.5164 | 21.1921 | 4.020 | 2.891 |
| `banked-diamond-accel` | `quintic-distance-scaled` | 12.2772 | 4.0263 | 9.5164 | 21.1921 | 4.020 | 2.891 |
| `banked-diamond-accel` | `minimum-snap-distance-scaled` | 12.5634 | 4.8085 | 14.8316 | 21.5757 | 4.474 | 3.635 |

## Trade-offs

- Even with richer waypoint acceleration boundaries, `quintic-distance-scaled` still wins the current preset. It remains cheaper than `minimum-snap-distance-scaled` and is better on mean RMSE, jerk RMS, snap RMS, peak speed, and peak acceleration.
- The richer boundary model reduced jerk and snap for the quintic variants compared with the velocity-only pass-through preset, but it did not create a regime where the minimum-snap generator becomes preferable.
- `banked-diamond-accel` still produces a tie between `quintic-uniform` and `quintic-distance-scaled`, so the ranking signal again comes mostly from the other two families.

## Core / Experiments

- This preset still promotes no new stable helper. The richer acceleration boundary synthesis remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
