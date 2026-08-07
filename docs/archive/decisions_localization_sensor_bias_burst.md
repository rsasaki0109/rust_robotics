# Decisions

## Context

- Problem preset: `localization-sensor-bias-burst-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` seeds per bucket and therefore defines the current reference labels.
- Keep `percentile-bucket` as the best current exploratory proxy. It reduces evaluation cost from `4486.11 ms` to `2234.69 ms`, while keeping hard agreement with `full-bucket` at `1.000` and near-tie-aware agreement at `1.000` on the current localization problem.
- Keep `first-scenario` as the cheapest smoke-test strategy. It runs in `447.61 ms`; its hard agreement with `full-bucket` is `0.778`, but its near-tie-aware agreement is `0.667`, so it should not drive filter-selection decisions.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`3` families) and emit the same `AccuracyObservation` records.
- Treat buckets inside the near-tie band (`|ratio - 1.0| < 0.030`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive filter wins.

## Reject

- Reject one-shot convergence. Cheaper variants remain useful for search, but the reference still needs full-bucket aggregation when the localization story is ambiguous.
- Reject the idea that pathfinding-specific evidence was enough. This problem keeps the same selection pattern but changes both domain and metric.

## Next

- Compare this preset against the cross-problem localization summary before widening the shared experiment core beyond its current contract.
- If the gap between `percentile-bucket` and `full-bucket` remains large, add another disposable variant instead of widening the stable surface.
- Keep `ukf_ckf_accuracy` experimental until a second non-pathfinding comparison uses the same contract with minimal edits.
