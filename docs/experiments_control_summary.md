# Experiments

## Scope

Compare the same five control aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`.

## Preset Comparison

| Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `control-tracking-windows` | `percentile-bucket` | 58.59 | 1.000 | 1.000 | `first-scenario` | 12.65 | 0.889 | 0.889 | 113.73 |
| `control-actuation-mismatch-windows` | `percentile-bucket` | 100.13 | 1.000 | 1.000 | `first-scenario` | 18.86 | 0.778 | 0.667 | 190.04 |

## Proxy Rotation

| Variant | Best-proxy wins | Presets |
| --- | ---: | --- |
| `first-scenario` | 0 | - |
| `sampled-bucket` | 0 | - |
| `percentile-bucket` | 2 | control-tracking-windows, control-actuation-mismatch-windows |
| `variance-triggered` | 0 | - |
