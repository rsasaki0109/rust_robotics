# Experiments

## Problem

Preset: `control-drone-trajectory-controller-generator-pairings`

Re-run the drone trajectory generators under the improved bounded lateral-feedback controller to see whether the stronger tracker changes the practical generator ranking across the existing non-coupled and coupled-continuity presets.

## Shared Input

- presets: `stop-go`, `pass-through`, `pass-through-accel`, `pass-through-accel-jerk`, `coupled-continuity`
- trajectory generators: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- controller variants compared for the pairing decision:
  - `baseline-pd`
  - `attitude-rate-damped-lateral-pd`
- metrics: mean generation time, mean tracking RMSE, mean max position error, mean thrust, mean max thrust, mean torque norm

## Best-RMSE Generator By Preset

Values below are from the latest local verification run of `cargo test -p rust_robotics_control --test drone_trajectory_variant_comparison drone_controller_generator_pairings_report_tradeoffs -- --nocapture`.

| Preset | Baseline best RMSE generator | Baseline best RMSE m | Lateral+damped best RMSE generator | Lateral+damped best RMSE m |
| --- | --- | ---: | --- | ---: |
| `stop-go` | `quintic-uniform` | 3.5357 | `quintic-distance-scaled` | 0.7631 |
| `pass-through` | `quintic-distance-scaled` | 9.8653 | `quintic-distance-scaled` | 0.9188 |
| `pass-through-accel` | `quintic-distance-scaled` | 9.7819 | `quintic-distance-scaled` | 0.9199 |
| `pass-through-accel-jerk` | `quintic-distance-scaled` | 9.7819 | `quintic-distance-scaled` | 0.9199 |
| `coupled-continuity` | `quintic-uniform` | 15.3070 | `minimum-snap-distance-scaled` | 1.7652 |

## Bounded Lateral-Feedback Pairing Summary

| Preset | Trajectory | Mean generation us | Mean RMSE m | Mean max err m | Mean torque norm N m |
| --- | --- | ---: | ---: | ---: | ---: |
| `stop-go` | `quintic-uniform` | 457.19 | 0.9144 | 1.8444 | 0.7671 |
| `stop-go` | `quintic-distance-scaled` | 450.41 | 0.7631 | 1.1648 | 0.8557 |
| `stop-go` | `minimum-snap-distance-scaled` | 2036.77 | 1.0903 | 1.6699 | 1.1087 |
| `pass-through` | `quintic-uniform` | 538.94 | 0.9194 | 1.5598 | 0.9763 |
| `pass-through` | `quintic-distance-scaled` | 532.04 | 0.9188 | 1.5520 | 0.9769 |
| `pass-through` | `minimum-snap-distance-scaled` | 2255.98 | 0.9919 | 1.7184 | 1.3110 |
| `pass-through-accel` | `quintic-uniform` | 533.59 | 0.9267 | 1.5492 | 0.9075 |
| `pass-through-accel` | `quintic-distance-scaled` | 523.69 | 0.9199 | 1.5399 | 0.9067 |
| `pass-through-accel` | `minimum-snap-distance-scaled` | 2232.66 | 0.9473 | 1.6098 | 1.2093 |
| `pass-through-accel-jerk` | `quintic-uniform` | 533.46 | 0.9267 | 1.5492 | 0.9075 |
| `pass-through-accel-jerk` | `quintic-distance-scaled` | 527.03 | 0.9199 | 1.5399 | 0.9067 |
| `pass-through-accel-jerk` | `minimum-snap-distance-scaled` | 2202.42 | 0.9486 | 1.6119 | 1.2098 |
| `coupled-continuity` | `quintic-uniform` | 551.05 | 1.7825 | 3.0935 | 0.1483 |
| `coupled-continuity` | `quintic-distance-scaled` | 555.45 | 1.7694 | 3.0756 | 0.1464 |
| `coupled-continuity` | `minimum-snap-distance-scaled` | 2265.74 | 1.7652 | 3.0633 | 0.2080 |

## Trade-offs

- The stronger controller does change some winner labels, so the generator question really was interaction-dependent rather than purely trajectory-side.
- On the four non-coupled presets, `quintic-distance-scaled` still beats `minimum-snap-distance-scaled` on both mean RMSE and mean max error under the bounded lateral-feedback controller, while also staying about `4x` cheaper to generate.
- On `stop-go`, the improved controller even shifts the best-RMSE label from `quintic-uniform` to `quintic-distance-scaled`, so the branch now has stronger practical support for the distance-scaled quintic default on that original preset.
- On `coupled-continuity`, the ranking becomes almost tied: `minimum-snap-distance-scaled` is slightly better than `quintic-distance-scaled` on mean RMSE (`1.7652` vs `1.7694`) and mean max error (`3.0633` vs `3.0756`), but the gap is tiny relative to the generation-cost jump (`2265.74 us` vs `555.45 us`) and its torque norm is still higher (`0.2080` vs `0.1464`).
- That means the improved controller narrows the old generator gap and exposes one coupled-only corner where minimum-snap becomes locally attractive, but it still does not overturn the branch-wide practical generator choice.

## Gain Sensitivity Sweep

Swept `lateral_position_gain` (0.08, 0.12, 0.15, 0.20, 0.25) x `lateral_velocity_gain` (0.25, 0.35, 0.45, 0.55, 0.65) on expanded coupled-continuity families (5 families).

| Trajectory variant | Best gain point | Best mean RMSE m | Default (pg=0.15, vg=0.45) RMSE m | Default / Best |
| --- | --- | ---: | ---: | ---: |
| `quintic-distance-scaled` | pg=0.25, vg=0.65 | 1.4691 | 1.6160 | 1.10 |
| `minimum-snap-distance-scaled` | pg=0.25, vg=0.55 | 1.4569 | 1.5878 | 1.09 |

Key observations:

- Default gains are within 10% of the best sweep point for both generators
- Higher gains consistently reduce RMSE but increase torque demand
- At the best gain points, minimum-snap achieves marginally better RMSE than quintic, but the gap is tiny (1.457 vs 1.469)
- The paired ranking is stable to mild gain retuning: no gain combination flips the overall generator choice

## Expanded Coupled-Continuity Families

Coupled-continuity families expanded from 3 to 5:

- original: `oval-cruise-coupled`, `figure-eight-climb-coupled`, `banked-diamond-coupled`
- new: `spiral-climb-coupled`, `reverse-s-coupled`

Full 5-family x 3-generator x 3-controller pairings confirm:

- bounded lateral-feedback controller wins decisively (mean RMSE 1.63 vs baseline 13.26)
- the controller advantage generalizes across all 5 coupled families
- `quintic-distance-scaled` remains the practical default: its RMSE is nearly tied with minimum-snap but at ~4x lower generation cost

## Core / Experiments

- `LateralGainSet` struct added to parameterize the bounded lateral-feedback controller gains
- `evaluate_lateral_gain_case()` function added for arbitrary gain sweep
- This preset still promotes no new stable helper or controller abstraction.
- The pairing comparison remains package-local inside `crates/rust_robotics_control/tests/drone_trajectory_variant_comparison.rs` and reuses the existing experiment harness in `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
