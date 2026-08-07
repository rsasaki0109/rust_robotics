# Experiments

## Scope

Compare the same five mapping aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`.

## Preset Comparison

| Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `mapping-point-cloud-sampling-windows` | `percentile-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |
| `mapping-occlusion-corruption-windows` | `variance-triggered` | 0.02 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |
| `mapping-density-shift-windows` | `variance-triggered` | 0.02 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 0.889 | 0.02 |
| `mapping-anisotropic-noise-windows` | `sampled-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 1.000 | 0.01 |
| `mapping-sparse-outlier-burst-windows` | `sampled-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 1.000 | 1.000 | 0.01 |
| `mapping-resolution-ladder-windows` | `percentile-bucket` | 0.01 | 1.000 | 1.000 | `sampled-bucket` | 0.01 | 0.889 | 0.889 | 0.01 |

## Proxy Rotation

| Variant | Best-proxy wins | Presets |
| --- | ---: | --- |
| `first-scenario` | 0 | - |
| `sampled-bucket` | 2 | mapping-anisotropic-noise-windows, mapping-sparse-outlier-burst-windows |
| `percentile-bucket` | 2 | mapping-point-cloud-sampling-windows, mapping-resolution-ladder-windows |
| `variance-triggered` | 2 | mapping-occlusion-corruption-windows, mapping-density-shift-windows |
