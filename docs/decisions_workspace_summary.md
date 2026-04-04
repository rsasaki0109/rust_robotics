# Decisions

## Context

- Scope: `4` packages and `21` validated presets aggregated into one workspace summary.

## Adopt

- Keep `full-bucket` as the convergence reference on every package and preset.
- Promote only the shared comparison contract in `rust_robotics_core::experiments`. Workspace-wide best-proxy rotation now spans `4` concrete variants: `first-scenario`, `percentile-bucket`, `sampled-bucket`, `variance-triggered`.
- Keep concrete variants disposable. Current package winner sets are planning -> `first-scenario`, `variance-triggered`; localization -> `first-scenario`, `percentile-bucket`, `sampled-bucket`, `variance-triggered`; control -> `percentile-bucket`; mapping -> `percentile-bucket`, `sampled-bucket`, `variance-triggered`.
- Keep the architectural stop condition machine-checked by the `rust_robotics_core` workspace-summary guard so winner convergence or docs drift becomes visible immediately.

## Reject

- Reject declaring a single concrete winner from frequency alone. The strongest current winner is only `percentile-bucket` (7/21).
- Reject treating the cheapest smoke test as a universal proxy. It disagrees with the best current proxy on `13/21` presets.
- Reject choosing one permanent exploratory proxy for the workspace. No concrete variant wins every package, and no package family is sufficient to justify core promotion by itself.
- Reject widening the stable surface to encode package-specific regimes such as MovingAI buckets, localization corruption patterns, controller mismatch families, or mapping corruption modes.

## Next

- Use this workspace summary as the current architectural stop condition: the stable core remains `SamplingPlan + descriptor + reference annotation + report metrics`, and even the best current proxy is still imperfect on `3/21` presets.
- If stronger external evidence is needed, measure downstream effects such as implementation speed, review clarity, or defect detection instead of adding more internal presets by default.
- Keep package-specific exploratory variants under `experiments/` and only extend `rust_robotics_core::experiments` when a new cross-package need survives comparison.
