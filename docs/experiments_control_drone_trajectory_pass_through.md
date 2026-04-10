# Experiments

## Problem

Preset: `control-drone-trajectory-pass-through`

Re-run the drone trajectory generator comparison with non-zero waypoint pass-through velocities to see whether `minimum-snap-distance-scaled` becomes preferable once the vehicle is no longer forced into stop-go boundaries.

## Shared Input

- families: `oval-cruise`, `figure-eight-climb`, `banked-diamond`
- variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- same quadrotor PD tracker for every variant
- boundary policy: waypoint velocities derived from adjacent segment chords with a preset tangent scale
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.47 s` across the compared variants
- metrics: generation runtime, tracking RMSE, jerk RMS, snap RMS, peak speed, peak acceleration

## Variant Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control --test drone_trajectory_variant_comparison -- --nocapture`.

| Variant | Style | Mean gen us | Mean RMSE m | Mean jerk RMS | Mean snap RMS | Mean vmax m/s | Mean amax m/s^2 | Total s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `quintic-uniform` | fixed-duration-quintic | 591.93 | 10.1726 | 4.1334 | 10.7007 | 3.6923 | 3.0486 | 15.47 |
| `quintic-distance-scaled` | distance-scaled-quintic | 600.82 | 9.8653 | 4.0965 | 10.6551 | 3.5898 | 3.0517 | 15.47 |
| `minimum-snap-distance-scaled` | distance-scaled-minimum-snap | 2598.43 | 10.5532 | 4.9339 | 16.8112 | 4.0371 | 3.9516 | 15.47 |

## Case Comparison

| Family | Variant | RMSE m | Jerk RMS | Snap RMS | Max err m | vmax | amax |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `oval-cruise` | `quintic-uniform` | 15.8428 | 2.7058 | 6.9299 | 27.0913 | 3.148 | 1.986 |
| `oval-cruise` | `quintic-distance-scaled` | 15.3367 | 2.7062 | 7.0554 | 25.9608 | 2.929 | 1.973 |
| `oval-cruise` | `minimum-snap-distance-scaled` | 15.4447 | 3.2681 | 11.1313 | 26.2674 | 3.205 | 2.543 |
| `figure-eight-climb` | `quintic-uniform` | 2.3889 | 5.3577 | 14.9629 | 5.1867 | 3.868 | 4.213 |
| `figure-eight-climb` | `quintic-distance-scaled` | 1.9730 | 5.2465 | 14.7006 | 3.6468 | 3.779 | 4.235 |
| `figure-eight-climb` | `minimum-snap-distance-scaled` | 3.3191 | 6.3346 | 23.1463 | 6.6991 | 4.352 | 5.494 |
| `banked-diamond` | `quintic-uniform` | 12.2861 | 4.3368 | 10.2093 | 21.1948 | 4.061 | 2.947 |
| `banked-diamond` | `quintic-distance-scaled` | 12.2861 | 4.3368 | 10.2093 | 21.1948 | 4.061 | 2.947 |
| `banked-diamond` | `minimum-snap-distance-scaled` | 12.8958 | 5.1992 | 16.1561 | 22.1158 | 4.554 | 3.818 |

## Trade-offs

- `quintic-distance-scaled` still wins the current pass-through preset. It is cheaper than `minimum-snap-distance-scaled`, has lower mean RMSE, lower jerk RMS, lower snap RMS, and lower peak speed.
- `minimum-snap-distance-scaled` did not flip the result under this boundary policy. The pass-through velocities alone were not enough to make the seventh-order generator preferable on the current closed-loop tracker.
- `banked-diamond` shows one limitation of the current preset: `quintic-uniform` and `quintic-distance-scaled` tie there, so most of the separation comes from `oval-cruise` and `figure-eight-climb`.

## Core / Experiments

- This preset does not promote a new stable helper beyond the sampled desired-state surface already introduced by the stop-go drone preset.
- The pass-through velocity synthesis remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
