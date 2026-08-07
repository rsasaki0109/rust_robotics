# Experiments

## Scope

Compare the same five localization aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`, while keeping the shared contract fixed.

## Preset Comparison

| Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `localization-noise-windows` | `first-scenario` | 226.65 | 1.000 | 1.000 | `first-scenario` | 226.65 | 1.000 | 1.000 | 2401.06 |
| `localization-long-horizon-windows` | `sampled-bucket` | 1417.57 | 1.000 | 1.000 | `first-scenario` | 475.98 | 0.889 | 0.889 | 4810.27 |
| `localization-dropout-bias-windows` | `percentile-bucket` | 1979.28 | 1.000 | 1.000 | `first-scenario` | 392.89 | 0.889 | 1.000 | 3912.51 |
| `localization-outlier-burst-windows` | `variance-triggered` | 3093.58 | 1.000 | 1.000 | `first-scenario` | 431.68 | 0.778 | 0.778 | 4279.09 |
| `localization-process-mismatch-windows` | `first-scenario` | 448.84 | 1.000 | 1.000 | `first-scenario` | 448.84 | 1.000 | 1.000 | 4447.57 |
| `localization-sensor-rate-mismatch-windows` | `percentile-bucket` | 2237.71 | 0.889 | 0.889 | `first-scenario` | 448.84 | 0.778 | 0.667 | 4473.06 |
| `localization-control-latency-windows` | `variance-triggered` | 2434.12 | 1.000 | 1.000 | `first-scenario` | 447.99 | 0.778 | 0.778 | 4678.75 |
| `localization-process-noise-anisotropy-windows` | `first-scenario` | 449.12 | 1.000 | 1.000 | `first-scenario` | 449.12 | 1.000 | 1.000 | 4512.68 |
| `localization-sensor-bias-burst-windows` | `percentile-bucket` | 2234.69 | 1.000 | 1.000 | `first-scenario` | 447.61 | 0.778 | 0.667 | 4486.11 |
| `localization-actuator-saturation-windows` | `first-scenario` | 443.52 | 1.000 | 1.000 | `first-scenario` | 443.52 | 1.000 | 1.000 | 4490.61 |

## Proxy Rotation

| Variant | Best-proxy wins | Presets |
| --- | ---: | --- |
| `first-scenario` | 4 | localization-noise-windows, localization-process-mismatch-windows, localization-process-noise-anisotropy-windows, localization-actuator-saturation-windows |
| `sampled-bucket` | 1 | localization-long-horizon-windows |
| `percentile-bucket` | 3 | localization-dropout-bias-windows, localization-sensor-rate-mismatch-windows, localization-sensor-bias-burst-windows |
| `variance-triggered` | 2 | localization-outlier-burst-windows, localization-control-latency-windows |

## Current Read

- `localization-noise-windows`: best proxy is `first-scenario`; cheapest smoke test is `first-scenario`.
- `localization-long-horizon-windows`: best proxy is `sampled-bucket`; cheapest smoke test is `first-scenario`.
- `localization-dropout-bias-windows`: best proxy is `percentile-bucket`; cheapest smoke test is `first-scenario`.
- `localization-outlier-burst-windows`: best proxy is `variance-triggered`; cheapest smoke test is `first-scenario`.
- `localization-process-mismatch-windows`: best proxy is `first-scenario`; cheapest smoke test is `first-scenario`.
- `localization-sensor-rate-mismatch-windows`: best proxy is `percentile-bucket`; cheapest smoke test is `first-scenario`.
- `localization-control-latency-windows`: best proxy is `variance-triggered`; cheapest smoke test is `first-scenario`.
- `localization-process-noise-anisotropy-windows`: best proxy is `first-scenario`; cheapest smoke test is `first-scenario`.
- `localization-sensor-bias-burst-windows`: best proxy is `percentile-bucket`; cheapest smoke test is `first-scenario`.
- `localization-actuator-saturation-windows`: best proxy is `first-scenario`; cheapest smoke test is `first-scenario`.
