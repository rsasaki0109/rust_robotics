# Meta-Control: Deterministic Controller Mode Selection

Status: implemented and verified

Date: 2026-08-07

## What it is

A **controller mode selector** that composes the three classical path trackers
under the repository's single `PathTracker` contract and switches between them
mid-run according to a deterministic policy. Pure Pursuit, Stanley, and LQR
Steer already share one trait; a meta controller is just another implementor of
that trait, which is the differentiator this repository can show and few others
can.

The selector runs inside the same shared Controller Arena engine — identical
path, initial state, clock, speed response, and turn-rate response — so meta
results are directly comparable to every fixed controller.

## Policies

`MetaControlPolicy` in `crates/rust_robotics_control/src/meta_control.rs`:

- `Fixed(kind)` — baseline; reproduces one fixed controller (also proves the
  meta loop reuses arena semantics exactly, see the parity test).
- `SwitchOnError { error_threshold, error_mode, cruise_mode }` — uses
  `error_mode` while the cross-track error exceeds the threshold, otherwise
  `cruise_mode`.
- `SwitchOnCurvature { curvature_threshold, tight_mode, smooth_mode }` — uses
  `tight_mode` while the local path curvature exceeds the threshold, otherwise
  `smooth_mode`.

Each sub-controller keeps its own internal state, so a switch resumes the
selected controller where its own history left off. Every run is deterministic
(no randomness).

## Key results

From `benchmark_meta_control` (3.0 m/s, 85% turn response), `docs/assets/meta-control.csv`:

| preset | policy | RMSE (m) | switches |
| --- | --- | ---: | ---: |
| hairpin | fixed: pure pursuit | 0.6229 | 0 |
| hairpin | fixed: stanley | 0.6288 | 0 |
| hairpin | fixed: lqr steer | 0.5432 | 0 |
| hairpin | **switch-on-curvature** | **0.5183** | 2 |
| hairpin | switch-on-error | 0.6625 | 15 |

`switch-on-curvature` is the win: on the hairpin it runs LQR Steer on the
6 m-radius arc (30% of samples) and Pure Pursuit on the straights, beating
every fixed controller on both RMSE (0.518 vs 0.543) and final goal distance
(0.735 vs 0.762). The same policy reduces to Pure Pursuit on the straight and
slalom presets (curvature stays under the 0.10 rad/m threshold), matching the
best smooth-tracker baseline.

`switch-on-error` demonstrates the mechanism (Stanley corrects the ~2.5 m
initial offset, then Pure Pursuit cruises) but also exposes a real artifact:
an error threshold can **flutter** on a curved course (15 switches on the
hairpin) because the error oscillates around the threshold. That trade-off —
correction authority vs mode stability — is the takeaway for tuning.

## Files

- Module: `crates/rust_robotics_control/src/meta_control.rs`
- Benchmark + SVG: `crates/rust_robotics/examples/benchmark_meta_control.rs`
- Artifacts: `docs/assets/meta-control.csv`, `docs/assets/meta-control.svg`
- Docs: this file

## Verification

```bash
cargo test -p rust_robotics_control meta_control          # 7 focused tests
cargo run -p rust_robotics --example benchmark_meta_control --no-default-features --features control
./scripts/check_benchmark_gate.sh                         # meta-control CSV is pinned
```

Tests cover: fixed-policy parity with the arena, both switch policies engage
both modes, determinism, switch-count correctness, invalid-threshold rejection,
and metrics recomputation.
