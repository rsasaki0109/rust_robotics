# Experiments

## Problem

Preset: `long-tail-windows`

Reuse the same runtime-aggregation contract on long-tail and high-bucket MovingAI windows to test whether the exploratory proxy story survives away from the crossover region.

## Shared Input

- `arena2` buckets: `90`
- `8room_000` buckets: `120, 160, 213`
- `random512-10-0` buckets: `140, 177`
- `maze512-1-0` buckets: `200, 800, 1211`
- `Berlin_0_512` buckets: `160, 186`
- per-scenario timing iterations: `3`
- near-tie reporting band: `|A*/JPS ratio - 1.0| < 0.020`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 2583.85 | 0.100 | 0.909 | 0.909 | 0.116 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 7542.86 | 0.300 | 0.909 | 0.909 | 0.074 | 31 | 0 | 1 | yes |
| `percentile-bucket` | functional-percentile | 13268.98 | 0.500 | 0.909 | 0.818 | 0.064 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 18994.34 | 0.682 | 0.909 | 0.909 | 0.010 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 25824.72 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw faster side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `8room_000/120` | outcome=`A*`, ratio=`0.859`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.911`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.916`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.896`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.905`, coverage=`10/10`, escalated=`no` |
| `8room_000/160` | outcome=`A*`, ratio=`0.964`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.955`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.924`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.925`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`A*`, ratio=`0.924`, coverage=`10/10`, escalated=`no` |
| `8room_000/213` | outcome=`near-tie`, raw=`A*`, ratio=`0.987`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.994`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.986`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.992`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`near-tie`, raw=`A*`, ratio=`0.991`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/160` | outcome=`JPS`, ratio=`1.699`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.194`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`JPS`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.223`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.213`, coverage=`10/10`, escalated=`no` |
| `Berlin_0_512/186` | outcome=`A*`, ratio=`0.932`, coverage=`1/10`, escalated=`no` | outcome=`JPS`, ratio=`1.649`, coverage=`3/10`, escalated=`no` | outcome=`JPS`, ratio=`1.350`, coverage=`5/10`, escalated=`no` | outcome=`JPS`, ratio=`1.281`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.289`, coverage=`10/10`, escalated=`no` |
| `arena2/90` | outcome=`JPS`, ratio=`1.063`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.677`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.689`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`A*`, ratio=`0.995`, coverage=`3/10->10/10`, escalated=`yes` | outcome=`JPS`, ratio=`1.022`, coverage=`10/10`, escalated=`no` |
| `maze512-1-0/200` | outcome=`A*`, ratio=`0.619`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.631`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.590`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.623`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.628`, coverage=`10/10`, escalated=`no` |
| `maze512-1-0/800` | outcome=`A*`, ratio=`0.672`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.627`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.611`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.622`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.609`, coverage=`10/10`, escalated=`no` |
| `maze512-1-0/1211` | outcome=`A*`, ratio=`0.639`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.635`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.634`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.631`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.607`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/140` | outcome=`A*`, ratio=`0.963`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.768`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.764`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.771`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.766`, coverage=`10/10`, escalated=`no` |
| `random512-10-0/177` | outcome=`A*`, ratio=`0.652`, coverage=`1/10`, escalated=`no` | outcome=`A*`, ratio=`0.653`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.664`, coverage=`5/10`, escalated=`no` | outcome=`A*`, ratio=`0.658`, coverage=`3/10`, escalated=`no` | outcome=`A*`, ratio=`0.655`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Stable core: `RuntimeAggregationVariant`, `RuntimeSamplingPlan`, `RuntimeExperimentCase`, `RuntimeObservation`, `RuntimeVariantReport`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`.
