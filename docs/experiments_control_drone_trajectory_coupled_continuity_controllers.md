# Experiments

## Problem

Preset: `control-drone-trajectory-coupled-continuity-controllers`

Re-run the coupled-continuity drone trajectory preset with controller variants to check whether controller-side changes explain the remaining tracking gap better than another trajectory-boundary heuristic.

## Shared Input

- trajectory families: `oval-cruise-coupled`, `figure-eight-climb-coupled`, `banked-diamond-coupled`
- trajectory generators: `quintic-uniform`, `quintic-distance-scaled`, `minimum-snap-distance-scaled`
- controller variants:
  - `baseline-pd`: existing z-axis PD plus attitude feedforward from desired acceleration
  - `attitude-rate-damped-pd`: same controller plus angular-rate damping on roll, pitch, and yaw torques
  - `attitude-rate-damped-lateral-pd`: attitude-rate damping plus bounded lateral position/velocity feedback folded into the desired roll/pitch acceleration command
- boundary policy: globally solved closed-loop waypoint velocities plus coupled waypoint acceleration continuity
- simulation step: `dt = 0.05 s`
- total mission time normalized to `15.47 s` across the compared trajectory generators
- metrics: tracking RMSE, max position error, mean thrust, max thrust, mean torque norm

## Controller Summary

Values below are from the latest local verification run of `cargo test -p rust_robotics_control drone_controller_variants_report_coupled_continuity_tradeoffs -- --nocapture`.

| Controller | Style | Mean RMSE m | Mean max err m | Mean thrust N | Mean max thrust N | Mean torque norm N m |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline-pd` | z-pd-plus-attitude-feedforward | 15.3686 | 26.2037 | 2.0116 | 2.0871 | 3.4787 |
| `attitude-rate-damped-pd` | baseline-pd-plus-attitude-rate-damping | 15.1201 | 26.4761 | 1.9803 | 2.0538 | 0.1408 |
| `attitude-rate-damped-lateral-pd` | baseline-pd-plus-bounded-lateral-feedback-and-rate-damping | 1.7724 | 3.0775 | 1.9816 | 2.0592 | 0.1676 |

## Case Comparison

| Family | Trajectory | Controller | RMSE m | Max err m | Mean thrust | Max thrust | Mean torque |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `oval-cruise-coupled` | `quintic-uniform` | `baseline-pd` | 23.0690 | 39.4315 | 1.9765 | 2.0179 | 1.8133 |
| `oval-cruise-coupled` | `quintic-uniform` | `attitude-rate-damped-pd` | 22.4266 | 39.2220 | 1.9698 | 2.0132 | 0.0487 |
| `oval-cruise-coupled` | `quintic-uniform` | `attitude-rate-damped-lateral-pd` | 1.6114 | 3.2480 | 1.9702 | 2.0132 | 0.0653 |
| `oval-cruise-coupled` | `quintic-distance-scaled` | `baseline-pd` | 22.5547 | 38.5740 | 1.9768 | 2.0144 | 1.8403 |
| `oval-cruise-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-pd` | 21.9397 | 38.3967 | 1.9699 | 2.0098 | 0.0475 |
| `oval-cruise-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-lateral-pd` | 1.5818 | 3.2010 | 1.9703 | 2.0098 | 0.0623 |
| `oval-cruise-coupled` | `minimum-snap-distance-scaled` | `baseline-pd` | 22.5622 | 38.5889 | 1.9771 | 2.0147 | 1.8738 |
| `oval-cruise-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-pd` | 21.9390 | 38.3957 | 1.9699 | 2.0098 | 0.0666 |
| `oval-cruise-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-lateral-pd` | 1.5821 | 3.2012 | 1.9703 | 2.0098 | 0.0810 |
| `figure-eight-climb-coupled` | `quintic-uniform` | `baseline-pd` | 0.7589 | 1.4270 | 2.0522 | 2.1830 | 5.4786 |
| `figure-eight-climb-coupled` | `quintic-uniform` | `attitude-rate-damped-pd` | 0.7164 | 1.1851 | 1.9886 | 2.1033 | 0.2132 |
| `figure-eight-climb-coupled` | `quintic-uniform` | `attitude-rate-damped-lateral-pd` | 0.5822 | 0.9106 | 1.9954 | 2.1163 | 0.2530 |
| `figure-eight-climb-coupled` | `quintic-distance-scaled` | `baseline-pd` | 1.6612 | 2.5964 | 2.0501 | 2.1792 | 5.4106 |
| `figure-eight-climb-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-pd` | 2.5349 | 4.3605 | 1.9882 | 2.0988 | 0.2117 |
| `figure-eight-climb-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-lateral-pd` | 0.5722 | 0.9040 | 1.9949 | 2.1115 | 0.2505 |
| `figure-eight-climb-coupled` | `minimum-snap-distance-scaled` | `baseline-pd` | 1.4187 | 2.1417 | 2.0739 | 2.1937 | 6.3266 |
| `figure-eight-climb-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-pd` | 2.5374 | 4.3622 | 1.9881 | 2.0967 | 0.3301 |
| `figure-eight-climb-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-lateral-pd` | 0.5604 | 0.8703 | 1.9944 | 2.1073 | 0.3790 |
| `banked-diamond-coupled` | `quintic-uniform` | `baseline-pd` | 22.0932 | 37.6807 | 1.9986 | 2.0594 | 2.8019 |
| `banked-diamond-coupled` | `quintic-uniform` | `attitude-rate-damped-pd` | 21.3303 | 37.4568 | 1.9828 | 2.0511 | 0.1036 |
| `banked-diamond-coupled` | `quintic-uniform` | `attitude-rate-damped-lateral-pd` | 3.1540 | 5.1219 | 1.9797 | 2.0550 | 0.1265 |
| `banked-diamond-coupled` | `quintic-distance-scaled` | `baseline-pd` | 22.0932 | 37.6807 | 1.9986 | 2.0594 | 2.8019 |
| `banked-diamond-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-pd` | 21.3303 | 37.4568 | 1.9828 | 2.0511 | 0.1036 |
| `banked-diamond-coupled` | `quintic-distance-scaled` | `attitude-rate-damped-lateral-pd` | 3.1540 | 5.1219 | 1.9797 | 2.0550 | 0.1265 |
| `banked-diamond-coupled` | `minimum-snap-distance-scaled` | `baseline-pd` | 22.1063 | 37.7124 | 2.0007 | 2.0618 | 2.9613 |
| `banked-diamond-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-pd` | 21.3261 | 37.4492 | 1.9828 | 2.0508 | 0.1419 |
| `banked-diamond-coupled` | `minimum-snap-distance-scaled` | `attitude-rate-damped-lateral-pd` | 3.1531 | 5.1185 | 1.9797 | 2.0546 | 0.1641 |

## Trade-offs

- Pure attitude-rate damping still lowers control effort sharply, but it remains mixed because the distance-scaled and minimum-snap figure-eight cases regress.
- Adding bounded lateral position/velocity feedback on top of rate damping resolves that weakness on the current preset and dramatically lowers mean tracking RMSE (`15.3686 -> 1.7724`) and mean max error (`26.2037 -> 3.0775`) against the baseline controller.
- The same bounded-feedback variant also keeps control effort low: mean torque norm stays far below the baseline (`3.4787 -> 0.1676`) while mean max thrust also stays slightly lower (`2.0871 -> 2.0592`).
- On the current coupled-continuity cases, this makes `attitude-rate-damped-lateral-pd` the strongest tested controller candidate so far, though it is still only validated inside this package-local experiment harness.

## Core / Experiments

- This preset still promotes no new stable helper. The controller variant comparison remains package-local inside `crates/rust_robotics_control/src/experiments/drone_trajectory_quality/mod.rs`.
