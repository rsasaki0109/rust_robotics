# Experiments

## Problem

Preset: `synthetic-local-grids`

Reuse the same runtime-aggregation contract on local synthetic grid families derived from the repository's open/maze/dense layouts instead of external MovingAI benchmark families.

## Shared Input

- `synthetic_open_50x50` buckets: `20, 60, 100`
- `synthetic_maze_50x50` buckets: `20, 60, 100`
- `synthetic_dense_50x50` buckets: `20, 60, 100`
- per-scenario timing iterations: `3`
- near-tie reporting band: `|A*/JPS ratio - 1.0| < 0.020`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 3.61 | 0.100 | 1.000 | 1.000 | 0.220 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 23.06 | 0.300 | 0.778 | 0.778 | 0.162 | 31 | 0 | 1 | yes |
| `percentile-bucket` | functional-percentile | 36.88 | 0.500 | 0.889 | 0.889 | 0.132 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 49.53 | 0.533 | 1.000 | 1.000 | 0.041 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 68.56 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw faster side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `synthetic_dense_50x50/20` | outcome=`A*`, ratio=`0.480`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.547`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.526`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.569`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.549`, coverage=`10/10`, escalated=`no` |
| `synthetic_dense_50x50/60` | outcome=`A*`, ratio=`0.622`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.340`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.728`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.632`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.722`, coverage=`10/10`, escalated=`no` |
| `synthetic_dense_50x50/100` | outcome=`A*`, ratio=`0.755`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.545`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.766`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.657`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.832`, coverage=`10/10`, escalated=`no` |
| `synthetic_maze_50x50/20` | outcome=`A*`, ratio=`0.288`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.672`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.487`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.680`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.732`, coverage=`10/10`, escalated=`no` |
| `synthetic_maze_50x50/60` | outcome=`A*`, ratio=`0.812`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.900`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.820`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.894`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.903`, coverage=`10/10`, escalated=`no` |
| `synthetic_maze_50x50/100` | outcome=`JPS`, ratio=`1.575`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.890`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.165`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.088`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.098`, coverage=`10/10`, escalated=`no` |
| `synthetic_open_50x50/20` | outcome=`A*`, ratio=`0.071`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.121`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.126`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.128`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.121`, coverage=`10/10`, escalated=`no` |
| `synthetic_open_50x50/60` | outcome=`A*`, ratio=`0.166`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.633`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.650`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.811`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.811`, coverage=`10/10`, escalated=`no` |
| `synthetic_open_50x50/100` | outcome=`JPS`, ratio=`1.130`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.205`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.085`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.096`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.104`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Stable core: `RuntimeAggregationVariant`, `RuntimeSamplingPlan`, `RuntimeExperimentCase`, `RuntimeObservation`, `RuntimeVariantReport`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`.
