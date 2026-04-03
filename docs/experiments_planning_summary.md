# Experiments

## Scope

Compare the same five planning aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`.

## Preset Comparison

| Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `crossover-windows` | `variance-triggered` | 16356.51 | 0.933 | 0.933 | `first-scenario` | 1896.14 | 0.533 | 0.533 | 18582.57 |
| `long-tail-windows` | `first-scenario` | 2583.85 | 0.909 | 0.909 | `first-scenario` | 2583.85 | 0.909 | 0.909 | 25824.72 |
| `synthetic-local-grids` | `first-scenario` | 3.61 | 1.000 | 1.000 | `first-scenario` | 3.61 | 1.000 | 1.000 | 68.56 |

## Proxy Rotation

| Variant | Best-proxy wins | Presets |
| --- | ---: | --- |
| `first-scenario` | 2 | long-tail-windows, synthetic-local-grids |
| `sampled-bucket` | 0 | - |
| `percentile-bucket` | 0 | - |
| `variance-triggered` | 1 | crossover-windows |
