# Decisions

## Context

- Problem preset: `mapping-resolution-ladder-windows`

## Adopt

- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` cloud perturbations per bucket and therefore defines the current reference labels.
- Keep `percentile-bucket` as the best current exploratory proxy. It reduces evaluation cost from `0.01 ms` to `0.01 ms`, while keeping hard agreement with `full-bucket` at `1.000` and near-tie-aware agreement at `1.000` on the current mapping problem.
- Keep `sampled-bucket` as the cheapest smoke-test strategy. It runs in `0.01 ms`; its hard agreement with `full-bucket` is `0.889` and its near-tie-aware agreement is `0.889`.
- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`3` families) and emit the same `PointSamplingObservation` records.

## Reject

- Reject widening the shared core for mapping-specific quality heuristics. Cloud generation, bucket semantics, and score composition stay package-local.
- Reject promoting any concrete aggregation strategy into core based on a single mapping preset.

## Next

- Compare this preset against the cross-problem mapping summary before widening the shared surface.
- Keep concrete variants under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
