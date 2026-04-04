# Decisions

## Context

- Problem preset: `long-tail-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` scenarios per bucket and therefore defines the current reference labels.
- Keep `first-scenario` as the best current exploratory proxy. It reduces evaluation cost from `25824.72 ms` to `2583.85 ms`, while keeping hard agreement with `full-bucket` at `0.909` and near-tie-aware agreement at `0.909` on the current problem.
- `first-scenario` is also the cheapest smoke-test strategy on this problem. That is useful evidence, but one preset is not enough to collapse the search space into a single permanent strategy.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`5` families) and emit the same `RuntimeObservation` records.
- `RuntimeSamplingPlan` has now been reused across multiple validated problem presets without adding new core methods.
- Treat buckets inside the near-tie band (`|ratio - 1.0| < 0.020`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive planner wins.

## Reject

- Reject single-path convergence. The current comparison set shows that cheaper variants distort the winner label on some buckets even when they are useful for search.
- Reject the idea that one aggregation strategy should become the only path. The current repository should keep `full-bucket` for convergence and cheaper variants for search.

## Next

- Keep the shared comparison contract from `rust_robotics_core::experiments` fixed while it is reused by planning, localization, and control.
- If the gap between `first-scenario` and `full-bucket` remains large, add another disposable variant instead of widening the core again.
- Keep only the comparison contract in core. Leave the concrete variants under `experiments/` until at least one more independent package uses the same interface.
