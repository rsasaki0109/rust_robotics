# Experiments

## Problem

Preset: `control-drone-trajectory-noncoupled-controllers`

Re-run the existing non-coupled drone trajectory presets with controller variants to check whether the bounded lateral-feedback controller gain survives outside the coupled-continuity boundary policy.

## Shared Input

- presets: `stop-go`, `pass-through`, `pass-through-accel`, `pass-through-accel-jerk`
- trajectory generators: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- controller variants:
  - `baseline-pd`: existing z-axis PD plus attitude feedforward from desired acceleration
  - `attitude-rate-damped-pd`: same controller plus angular-rate damping on roll, pitch, and yaw torques
  - `attitude-rate-damped-lateral-pd`: attitude-rate damping plus bounded lateral position/velocity feedback folded into the desired roll/pitch acceleration command
- simulation step: preset-local `dt` already used by the existing trajectory experiment harness
- metrics: tracking RMSE, max position error, mean thrust, max thrust, mean torque norm

## Preset Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control drone_controller_variants_report_noncoupled_tradeoffs -- --nocapture`.

| Preset | Controller | Mean RMSE m | Mean max err m | Mean thrust N | Mean max thrust N | Mean torque norm N m |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `stop-go` | `baseline-pd` | 3.6871 | 8.2391 | 2.1214 | 2.4191 | 5.4440 |
| `stop-go` | `attitude-rate-damped-pd` | 3.8431 | 8.1803 | 2.0197 | 2.2026 | 0.8113 |
| `stop-go` | `attitude-rate-damped-lateral-pd` | 0.9226 | 1.5597 | 2.0271 | 2.2053 | 0.9105 |
| `pass-through` | `baseline-pd` | 10.1970 | 17.7064 | 2.1313 | 2.4412 | 6.2290 |
| `pass-through` | `attitude-rate-damped-pd` | 9.2314 | 16.1798 | 2.0028 | 2.1140 | 0.9584 |
| `pass-through` | `attitude-rate-damped-lateral-pd` | 0.9434 | 1.6101 | 2.0091 | 2.1198 | 1.0880 |
| `pass-through-accel` | `baseline-pd` | 10.0495 | 17.4404 | 2.1064 | 2.3752 | 5.6809 |
| `pass-through-accel` | `attitude-rate-damped-pd` | 9.2911 | 16.3140 | 2.0008 | 2.1069 | 0.8869 |
| `pass-through-accel` | `attitude-rate-damped-lateral-pd` | 0.9313 | 1.5663 | 2.0070 | 2.1151 | 1.0078 |
| `pass-through-accel-jerk` | `baseline-pd` | 10.0476 | 17.4361 | 2.1064 | 2.3756 | 5.6860 |
| `pass-through-accel-jerk` | `attitude-rate-damped-pd` | 9.2909 | 16.3137 | 2.0008 | 2.1070 | 0.8870 |
| `pass-through-accel-jerk` | `attitude-rate-damped-lateral-pd` | 0.9317 | 1.5670 | 2.0070 | 2.1152 | 1.0080 |

## Highlight Cases

| Preset / Family / Trajectory | Baseline RMSE m | Damped RMSE m | Lateral+damped RMSE m | Baseline max err m | Damped max err m | Lateral+damped max err m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `stop-go / stair-zigzag / quintic-distance-scaled` | 2.8213 | 4.0654 | 0.8535 | 5.2636 | 8.3515 | 1.2933 |
| `stop-go / stair-zigzag / minimum-snap-distance-scaled` | 1.1985 | 5.1218 | 1.1208 | 1.8388 | 10.7968 | 1.7346 |
| `pass-through / figure-eight-climb / quintic-distance-scaled` | 1.9730 | 1.2738 | 0.5583 | 3.6468 | 2.3147 | 0.9677 |
| `pass-through / figure-eight-climb / minimum-snap-distance-scaled` | 3.3191 | 1.5448 | 0.7445 | 6.6991 | 3.0798 | 1.4438 |
| `pass-through-accel / figure-eight-climb-accel / quintic-distance-scaled` | 1.6313 | 1.2042 | 0.5503 | 3.0296 | 2.1592 | 0.9557 |
| `pass-through-accel-jerk / figure-eight-climb-accel-jerk / minimum-snap-distance-scaled` | 2.9083 | 1.4457 | 0.6312 | 5.9450 | 2.8158 | 1.1737 |

## Trade-offs

- The bounded lateral-feedback controller beats both the baseline PD tracker and the pure attitude-rate damping variant on mean RMSE for all four non-coupled presets.
- The same variant also lowers mean max position error sharply across every tested non-coupled preset, so the coupled-continuity win is not isolated to one boundary policy.
- Pure attitude-rate damping alone stays mixed. It helps many pass-through cases, but on `stop-go` it slightly worsens mean RMSE and it badly regresses the `stair-zigzag` distance-scaled and minimum-snap runs.
- The bounded-feedback controller keeps control effort far below the baseline on every preset, even though its torque norm remains slightly above the pure damping-only variant.

## Core / Experiments

- This preset still promotes no new stable helper. The controller variant comparison remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
