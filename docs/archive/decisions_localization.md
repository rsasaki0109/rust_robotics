# Decisions

## Context

- Problem preset: `localization-noise-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` seeds per bucket and therefore defines the current reference labels.
- Keep `first-scenario` as the best current exploratory proxy. It reduces evaluation cost from `2401.06 ms` to `226.65 ms`, while keeping hard agreement with `full-bucket` at `1.000` and near-tie-aware agreement at `1.000` on the current localization problem.
- `first-scenario` is also the cheapest smoke-test strategy on this problem. That is useful evidence, but it should stay a disposable strategy until another independent localization preset says the same thing.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`3` families) and emit the same `AccuracyObservation` records.
- Treat buckets inside the near-tie band (`|ratio - 1.0| < 0.030`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive filter wins.

## Reject

- Reject one-shot convergence. Cheaper variants remain useful for search, but the reference still needs full-bucket aggregation when the localization story is ambiguous.
- Reject the idea that pathfinding-specific evidence was enough. This problem keeps the same selection pattern but changes both domain and metric.

## Next

- Compare this preset against the cross-problem localization summary before widening the shared experiment core beyond its current contract.
- If the gap between `first-scenario` and `full-bucket` remains large, add another disposable variant instead of widening the stable surface.
- Keep `ukf_ckf_accuracy` experimental until a second non-pathfinding comparison uses the same contract with minimal edits.
