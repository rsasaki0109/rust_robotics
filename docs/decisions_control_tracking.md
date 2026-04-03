# Decisions

## Context

- Problem preset: `control-tracking-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` initial states per bucket and therefore defines the current reference labels.
- Keep `percentile-bucket` as the best current exploratory proxy. It reduces evaluation cost from `113.73 ms` to `58.59 ms`, while keeping hard agreement with `full-bucket` at `1.000` and near-tie-aware agreement at `1.000` on the current control problem.
- Keep `first-scenario` as the cheapest smoke-test strategy. It runs in `12.65 ms`; its hard agreement with `full-bucket` is `0.889` and its near-tie-aware agreement is `0.889`.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`3` families) and emit the same `TrackingObservation` records.

## Reject

- Reject widening the shared core for control-specific details. Path shape, initialization policy, and score definition stay package-local.
- Reject promoting any concrete aggregation strategy into core based on a single control preset.

## Next

- Compare this preset against the cross-problem control summary before widening the shared surface.
- Keep concrete variants under `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`.
