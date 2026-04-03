# Decisions

## Context

- Scope: localization aggregation variants across `10` independent presets.

## Adopt

- Keep `full-bucket` as the convergence reference on every preset.
- Promote only the comparison contract. The current best proxy rotates across presets: `localization-noise-windows -> first-scenario; localization-long-horizon-windows -> sampled-bucket; localization-dropout-bias-windows -> percentile-bucket; localization-outlier-burst-windows -> variance-triggered; localization-process-mismatch-windows -> first-scenario; localization-sensor-rate-mismatch-windows -> percentile-bucket; localization-control-latency-windows -> variance-triggered; localization-process-noise-anisotropy-windows -> first-scenario; localization-sensor-bias-burst-windows -> percentile-bucket; localization-actuator-saturation-windows -> first-scenario`.
- Keep concrete variants disposable inside `experiments/ukf_ckf_accuracy/`. None has won every current localization preset.

## Reject

- Reject choosing one permanent exploratory proxy for localization. The current best-proxy winners split across `4` concrete variants: `first-scenario`, `percentile-bucket`, `sampled-bucket`, `variance-triggered`.
- Reject widening the stable surface to encode dropout, bias, or horizon-specific logic. Those belong to disposable presets, not to the core contract.

## Next

- Stop preset growth for localization here. The current stop condition is met: `10` independent presets still do not yield a single winning concrete variant.
- The next move is not another preset. Keep the shared comparison contract fixed while planning, localization, and control reuse it without promoting any concrete strategy.
- Shared core is now limited to `SamplingPlan + descriptor + reference annotation`; do not promote any concrete variant.
