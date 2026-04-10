# Experiments

## Problem

Preset: `control-drone-trajectory-variants`

Compare concrete drone waypoint-trajectory generators on the same closed-loop quadrotor tracker before treating any new aerial helper as a practical default.

## Shared Input

- families: `box-climb`, `diamond-ascent`, `stair-zigzag`
- variants: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- same waypoint loops and same PD quadrotor tracker for every variant
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.20 s` across the compared variants
- metrics: generation runtime, tracking RMSE, jerk RMS, snap RMS, peak speed, peak acceleration

## Variant Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control -- drone_trajectory --nocapture`.

| Variant | Style | Mean gen us | Mean RMSE m | Mean jerk RMS | Mean snap RMS | Mean vmax m/s | Mean amax m/s^2 | Total s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `quintic-uniform` | fixed-duration-quintic | 523.55 | 3.5357 | 4.0267 | 8.1514 | 4.7557 | 3.8625 | 15.20 |
| `quintic-distance-scaled` | distance-scaled-quintic | 517.66 | 3.6117 | 4.1907 | 9.4321 | 3.8577 | 3.4018 | 15.20 |
| `minimum-snap-distance-scaled` | distance-scaled-minimum-snap | 2387.46 | 3.9138 | 5.1686 | 14.5485 | 4.5006 | 4.4268 | 15.20 |

## Case Comparison

| Family | Variant | RMSE m | Jerk RMS | Snap RMS | Max err m | vmax | amax |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `box-climb` | `quintic-uniform` | 2.7934 | 3.4685 | 6.6334 | 6.4260 | 3.815 | 2.937 |
| `box-climb` | `quintic-distance-scaled` | 2.7260 | 3.4063 | 6.5746 | 6.2550 | 3.783 | 2.931 |
| `box-climb` | `minimum-snap-distance-scaled` | 3.5922 | 4.2137 | 10.1163 | 8.3864 | 4.413 | 3.815 |
| `diamond-ascent` | `quintic-uniform` | 5.4775 | 4.5028 | 9.5550 | 11.9830 | 4.037 | 3.453 |
| `diamond-ascent` | `quintic-distance-scaled` | 5.2877 | 4.4423 | 9.4977 | 11.5622 | 3.966 | 3.445 |
| `diamond-ascent` | `minimum-snap-distance-scaled` | 6.9506 | 5.4465 | 14.7134 | 15.3292 | 4.627 | 4.481 |
| `stair-zigzag` | `quintic-uniform` | 2.3364 | 4.1087 | 8.2658 | 7.1073 | 6.414 | 5.198 |
| `stair-zigzag` | `quintic-distance-scaled` | 2.8213 | 4.7234 | 12.2240 | 5.2636 | 3.824 | 3.829 |
| `stair-zigzag` | `minimum-snap-distance-scaled` | 1.1985 | 5.8458 | 18.8158 | 1.8388 | 4.461 | 4.984 |

## Trade-offs

- `quintic-distance-scaled` is the current practical default for mixed-length waypoint loops: it keeps generation cost near the baseline while reducing mean peak speed from `4.7557` to `3.8577 m/s` and mean peak acceleration from `3.8625` to `3.4018 m/s^2`.
- `quintic-uniform` still has the best mean tracking RMSE on this preset, so the duration-aware choice is a control-aggressiveness trade rather than a strict accuracy win.
- `minimum-snap-distance-scaled` is useful as an experimental aerial trajectory generator, but under this stop-go preset it is slower and more aggressive than the quintic variants.

## Core / Experiments

- Stable helpers now live in `crates/rust_robotics_control/src/drone_3d_trajectory.rs`: `generate_waypoint_trajectory_with_durations`, `sample_trajectory_segments`, and `simulate_desired_states`.
- New concrete aerial generator lives in `crates/rust_robotics_control/src/minimum_snap_trajectory.rs`.
- Experimental comparison harness lives in `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/` with the executable check in `crates/rust_robotics_control/tests/drone_trajectory_variant_comparison.rs`.
