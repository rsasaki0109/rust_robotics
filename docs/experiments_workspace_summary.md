# Experiments

## Scope

Aggregate the package-level preset summaries from planning, localization, control, and mapping to see whether any concrete exploratory proxy deserves promotion beyond `experiments/` anywhere in the workspace.

## Package Comparison

| Package | Presets | Winning variants | Mean best speedup | Cheapest mismatch | Imperfect best-proxy | Summary docs |
| --- | ---: | --- | ---: | ---: | ---: | --- |
| `planning` | 3 | `first-scenario`, `variance-triggered` | 10.04x | 1 | 2 | [`experiments_planning_summary.md`] + [`decisions_planning_summary.md`] |
| `localization` | 10 | `first-scenario`, `percentile-bucket`, `sampled-bucket`, `variance-triggered` | 5.34x | 6 | 1 | [`experiments_localization_summary.md`] + [`decisions_localization_summary.md`] |
| `control` | 2 | `percentile-bucket` | 1.92x | 2 | 0 | [`experiments_control_summary.md`] + [`decisions_control_summary.md`] |
| `mapping` | 6 | `percentile-bucket`, `sampled-bucket`, `variance-triggered` | 0.92x | 4 | 0 | [`experiments_mapping_summary.md`] + [`decisions_mapping_summary.md`] |

## Stop-Condition Evidence

- strongest concrete winner share: `percentile-bucket` (7/21)
- presets where cheapest smoke test disagrees with best proxy: `13/21`
- presets where even the best proxy is imperfect against full-bucket: `3/21`

## Workspace Preset Comparison

| Package | Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `control` | `control-actuation-mismatch-windows` | `percentile-bucket` | 100.13 | 1.000 | 1.000 | `first-scenario` | 18.86 | 0.778 | 0.667 | 190.04 |
| `control` | `control-tracking-windows` | `percentile-bucket` | 58.59 | 1.000 | 1.000 | `first-scenario` | 12.65 | 0.889 | 0.889 | 113.73 |
| `localization` | `localization-actuator-saturation-windows` | `first-scenario` | 443.52 | 1.000 | 1.000 | `first-scenario` | 443.52 | 1.000 | 1.000 | 4490.61 |
| `localization` | `localization-control-latency-windows` | `variance-triggered` | 2434.12 | 1.000 | 1.000 | `first-scenario` | 447.99 | 0.778 | 0.778 | 4678.75 |
| `localization` | `localization-dropout-bias-windows` | `percentile-bucket` | 1979.28 | 1.000 | 1.000 | `first-scenario` | 392.89 | 0.889 | 1.000 | 3912.51 |
| `localization` | `localization-long-horizon-windows` | `sampled-bucket` | 1417.57 | 1.000 | 1.000 | `first-scenario` | 475.98 | 0.889 | 0.889 | 4810.27 |
| `localization` | `localization-noise-windows` | `first-scenario` | 226.65 | 1.000 | 1.000 | `first-scenario` | 226.65 | 1.000 | 1.000 | 2401.06 |
| `localization` | `localization-outlier-burst-windows` | `variance-triggered` | 3093.58 | 1.000 | 1.000 | `first-scenario` | 431.68 | 0.778 | 0.778 | 4279.09 |
| `localization` | `localization-process-mismatch-windows` | `first-scenario` | 448.84 | 1.000 | 1.000 | `first-scenario` | 448.84 | 1.000 | 1.000 | 4447.57 |
| `localization` | `localization-process-noise-anisotropy-windows` | `first-scenario` | 449.12 | 1.000 | 1.000 | `first-scenario` | 449.12 | 1.000 | 1.000 | 4512.68 |
| `localization` | `localization-sensor-bias-burst-windows` | `percentile-bucket` | 2234.69 | 1.000 | 1.000 | `first-scenario` | 447.61 | 0.778 | 0.667 | 4486.11 |
| `localization` | `localization-sensor-rate-mismatch-windows` | `percentile-bucket` | 2237.71 | 0.889 | 0.889 | `first-scenario` | 448.84 | 0.778 | 0.667 | 4473.06 |
| `mapping` | `mapping-anisotropic-noise-windows` | `sampled-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 1.000 | 0.01 |
| `mapping` | `mapping-density-shift-windows` | `variance-triggered` | 0.02 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 0.889 | 0.02 |
| `mapping` | `mapping-occlusion-corruption-windows` | `variance-triggered` | 0.02 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |
| `mapping` | `mapping-point-cloud-sampling-windows` | `percentile-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |
| `mapping` | `mapping-resolution-ladder-windows` | `percentile-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |
| `mapping` | `mapping-sparse-outlier-burst-windows` | `sampled-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 1.000 | 0.01 |
| `planning` | `crossover-windows` | `variance-triggered` | 16356.51 | 0.933 | 0.933 | `first-scenario` | 1896.14 | 0.533 | 0.533 | 18582.57 |
| `planning` | `long-tail-windows` | `first-scenario` | 2583.85 | 0.909 | 0.909 | `first-scenario` | 2583.85 | 0.909 | 0.909 | 25824.72 |
| `planning` | `synthetic-local-grids` | `first-scenario` | 3.61 | 1.000 | 1.000 | `first-scenario` | 3.61 | 1.000 | 1.000 | 68.56 |

## Proxy Rotation

| Variant | Best-proxy wins | Package/Preset |
| --- | ---: | --- |
| `first-scenario` | 6 | localization/localization-actuator-saturation-windows, localization/localization-noise-windows, localization/localization-process-mismatch-windows, localization/localization-process-noise-anisotropy-windows, planning/long-tail-windows, planning/synthetic-local-grids |
| `sampled-bucket` | 3 | localization/localization-long-horizon-windows, mapping/mapping-anisotropic-noise-windows, mapping/mapping-sparse-outlier-burst-windows |
| `percentile-bucket` | 7 | control/control-actuation-mismatch-windows, control/control-tracking-windows, localization/localization-dropout-bias-windows, localization/localization-sensor-bias-burst-windows, localization/localization-sensor-rate-mismatch-windows, mapping/mapping-point-cloud-sampling-windows, mapping/mapping-resolution-ladder-windows |
| `variance-triggered` | 5 | localization/localization-control-latency-windows, localization/localization-outlier-burst-windows, mapping/mapping-density-shift-windows, mapping/mapping-occlusion-corruption-windows, planning/crossover-windows |
