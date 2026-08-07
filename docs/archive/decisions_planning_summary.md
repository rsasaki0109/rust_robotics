# Decisions

## Context

- Scope: planning aggregation variants across `3` independent presets.

## Adopt

- Keep `full-bucket` as the convergence reference on every preset.
- Promote only the comparison contract. Current planning presets map to `crossover-windows -> variance-triggered; long-tail-windows -> first-scenario; synthetic-local-grids -> first-scenario`.
- Keep concrete variants disposable inside `experiments/moving_ai_runtime/`. Planning-only evidence still does not justify core promotion of any concrete planning variant.

## Reject

- Reject choosing one permanent exploratory proxy for planning. Current planning presets cover `2` concrete winner set(s): `first-scenario`, `variance-triggered`.
- Reject widening the shared surface to encode MovingAI-specific or synthetic-grid-specific regimes. Those belong to disposable presets, not to the core contract.

## Next

- Keep the shared comparison contract fixed while it is reused by planning, localization, control, and mapping.
- Prefer cross-package synthesis before adding another planning-only preset.
- Do not promote any concrete planning aggregation variant into core.
