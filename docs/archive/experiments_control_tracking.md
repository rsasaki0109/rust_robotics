# Experiments

## Problem

Preset: `control-tracking-windows`

Compare multiple aggregation strategies on identical path-tracking simulations, asking the same question on every family/bucket window: which closed-loop tracker has lower tracking score, `PurePursuit` or `Stanley`?

## Shared Input

- `straight-recovery` buckets: `60, 120, 180`
- `slalom-recovery` buckets: `80, 140, 220`
- `tight-turn-recovery` buckets: `100, 180, 260`
- scenarios per bucket: `10` deterministic initial states
- bucket meaning: simulation horizon and initial perturbation scale are both tied to the bucket label
- score: path RMSE plus terminal goal-distance penalty
- near-tie reporting band: `|PurePursuit/Stanley score ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 12.65 | 0.100 | 0.889 | 0.889 | 0.116 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 33.53 | 0.300 | 0.889 | 0.889 | 0.025 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 58.59 | 0.500 | 1.000 | 1.000 | 0.008 | 40 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 101.37 | 0.922 | 1.000 | 1.000 | 0.000 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 113.73 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-score side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `slalom-recovery/80` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.003`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.005`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.003`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.003`, coverage=`10/10`, escalated=`no` |
| `slalom-recovery/140` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.014`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.016`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.026`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.019`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.019`, coverage=`10/10`, escalated=`no` |
| `slalom-recovery/220` | outcome=`Stanley`, ratio=`1.038`, coverage=`1/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.072`, coverage=`3/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.079`, coverage=`5/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.052`, coverage=`10/10`, escalated=`yes` | outcome=`Stanley`, ratio=`1.052`, coverage=`10/10`, escalated=`no` |
| `straight-recovery/60` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`10/10`, escalated=`no` |
| `straight-recovery/120` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.002`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`10/10`, escalated=`no` |
| `straight-recovery/180` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.004`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.012`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.012`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.012`, coverage=`10/10`, escalated=`no` |
| `tight-turn-recovery/100` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.020`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.022`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.024`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.029`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.029`, coverage=`10/10`, escalated=`no` |
| `tight-turn-recovery/180` | outcome=`PurePursuit`, ratio=`0.057`, coverage=`1/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.880`, coverage=`3/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.093`, coverage=`5/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.061`, coverage=`10/10`, escalated=`yes` | outcome=`Stanley`, ratio=`1.061`, coverage=`10/10`, escalated=`no` |
| `tight-turn-recovery/260` | outcome=`PurePursuit`, ratio=`0.059`, coverage=`1/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.058`, coverage=`3/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.059`, coverage=`5/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.058`, coverage=`3/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.059`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `TrackingAggregationVariant`, `TrackingExperimentCase`, `TrackingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`.
