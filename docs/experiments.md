# Experiments

## Problem

Preset: `crossover-windows`

Compare multiple runtime-aggregation strategies on crossover-focused MovingAI bucket windows before promoting any evaluation method into the stable workflow.

## Shared Input

- `arena2` buckets: `86, 87, 88, 89, 90`
- `8room_000` buckets: `80, 85, 90`
- `random512-10-0` buckets: `130, 132, 135`
- `Berlin_0_512` buckets: `120, 140, 160, 186`
- per-scenario timing iterations: `3`
- near-tie reporting band: `|A*/JPS ratio - 1.0| < 0.020`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 1896.14 | 0.100 | 0.533 | 0.533 | 0.199 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 5514.01 | 0.300 | 0.733 | 0.667 | 0.185 | 31 | 0 | 1 | yes |
| `percentile-bucket` | functional-percentile | 9686.57 | 0.500 | 0.800 | 0.733 | 0.167 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 16356.51 | 0.907 | 0.933 | 0.933 | 0.036 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 18582.57 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw faster side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `8room_000/80` | outcome=`JPS`, ratio=`1.120`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.944`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.910`, coverage=`10/10`, escalated=`no` |
| `8room_000/85` | outcome=`A*`, ratio=`0.936`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.940`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.941`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.929`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.941`, coverage=`10/10`, escalated=`no` |
| `8room_000/90` | outcome=`near-tie`, raw=`JPS`, ratio=`1.018`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.994`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.979`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.957`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.959`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/120` | outcome=`A*`, ratio=`0.932`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.962`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.443`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.173`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.162`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/140` | outcome=`A*`, ratio=`0.905`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.970`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.972`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.613`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.679`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/160` | outcome=`JPS`, ratio=`1.726`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.153`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`JPS`, ratio=`1.009`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.213`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.224`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/186` | outcome=`A*`, ratio=`0.956`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.632`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.329`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.292`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.282`, coverage=`10/10`, escalated=`no` |
| `arena2/86` | outcome=`A*`, ratio=`0.721`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.714`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.705`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.766`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.756`, coverage=`10/10`, escalated=`no` |
| `arena2/87` | outcome=`A*`, ratio=`0.701`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.039`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.699`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.701`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.686`, coverage=`10/10`, escalated=`no` |
| `arena2/88` | outcome=`A*`, ratio=`0.669`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.759`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.748`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.715`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.718`, coverage=`10/10`, escalated=`no` |
| `arena2/89` | outcome=`JPS`, ratio=`1.110`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.109`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.066`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.734`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.747`, coverage=`10/10`, escalated=`no` |
| `arena2/90` | outcome=`near-tie`, raw=`JPS`, ratio=`1.016`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.703`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.690`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.027`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.978`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/130` | outcome=`JPS`, ratio=`1.577`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.673`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.614`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.703`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.664`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/132` | outcome=`JPS`, ratio=`1.858`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.819`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.873`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.866`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.848`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/135` | outcome=`A*`, ratio=`0.661`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.700`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.346`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.693`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.944`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Stable core: `RuntimeAggregationVariant`, `RuntimeSamplingPlan`, `RuntimeExperimentCase`, `RuntimeObservation`, `RuntimeVariantReport`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`.
