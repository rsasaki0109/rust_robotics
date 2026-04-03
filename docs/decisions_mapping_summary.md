# Decisions

## Context

- Scope: mapping aggregation variants across `6` independent presets.

## Adopt

- Keep `full-bucket` as the convergence reference on every preset.
- Promote only the comparison contract. Current mapping presets map to `mapping-point-cloud-sampling-windows -> percentile-bucket; mapping-occlusion-corruption-windows -> variance-triggered; mapping-density-shift-windows -> variance-triggered; mapping-anisotropic-noise-windows -> sampled-bucket; mapping-sparse-outlier-burst-windows -> sampled-bucket; mapping-resolution-ladder-windows -> percentile-bucket`.
- Keep concrete variants disposable inside `experiments/point_cloud_sampling_quality/`. Package-local evidence still does not justify core promotion of any concrete mapping variant.

## Reject

- Reject choosing one permanent exploratory proxy for the workspace from mapping-only evidence. Current mapping presets cover `3` concrete winner set(s): `percentile-bucket`, `sampled-bucket`, `variance-triggered`.
- Reject widening the shared surface to encode mapping-specific corruption or noise regimes. Those belong to disposable presets, not to the core contract.

## Next

- Keep the shared comparison contract fixed while it is reused by planning, localization, control, and mapping.
- Prefer another mapping preset or another package before widening `rust_robotics_core::experiments` again.
- Do not promote any concrete mapping aggregation variant into core.
