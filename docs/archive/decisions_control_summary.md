# Decisions

## Context

- Scope: control aggregation variants across `2` independent presets.

## Adopt

- Keep `full-bucket` as the convergence reference on every preset.
- Promote only the comparison contract. Current control presets map to `control-tracking-windows -> percentile-bucket; control-actuation-mismatch-windows -> percentile-bucket`.
- Keep concrete variants disposable inside `experiments/path_tracking_accuracy/`. Workspace-wide evidence still does not justify core promotion of any concrete control variant.

## Reject

- Reject choosing one permanent exploratory proxy for the workspace from control-only evidence. Current control presets cover `1` concrete winner set(s): `percentile-bucket`.
- Reject widening the shared surface to encode actuation mismatch or path-shape specifics. Those belong to disposable presets, not to the core contract.

## Next

- Keep the shared comparison contract fixed while it is reused by planning, localization, and control.
- Prefer a fourth package before widening `rust_robotics_core::experiments` again.
- Do not promote any concrete control aggregation variant into core.
