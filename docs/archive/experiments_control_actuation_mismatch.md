# Experiments

## Problem

Preset: `control-actuation-mismatch-windows`

Reuse the same control comparison contract on delayed and attenuated actuation, asking which tracker stays more accurate when the plant lags and understeers relative to the commanded control.

## Shared Input

- `velocity-sag-straight` buckets: `80, 140, 220`
- `steering-lag-slalom` buckets: `100, 180, 260`
- `understeer-hairpin` buckets: `120, 200, 300`
- scenarios per bucket: `10` deterministic initial states
- bucket meaning: simulation horizon and initial perturbation scale are both tied to the bucket label
- score: path RMSE plus terminal goal-distance penalty
- near-tie reporting band: `|PurePursuit/Stanley score ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 18.86 | 0.100 | 0.778 | 0.667 | 0.123 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 56.56 | 0.300 | 0.778 | 0.889 | 0.008 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 100.13 | 0.500 | 1.000 | 1.000 | 0.005 | 40 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 188.47 | 1.000 | 1.000 | 1.000 | 0.000 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 190.04 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-score side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `steering-lag-slalom/100` | outcome=`PurePursuit`, ratio=`0.747`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.991`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.002`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.002`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.002`, coverage=`10/10`, escalated=`no` |
| `steering-lag-slalom/180` | outcome=`Stanley`, ratio=`1.040`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.026`, coverage=`3/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.037`, coverage=`5/10`, escalated=`no` | outcome=`Stanley`, ratio=`1.039`, coverage=`10/10`, escalated=`yes` | outcome=`Stanley`, ratio=`1.039`, coverage=`10/10`, escalated=`no` |
| `steering-lag-slalom/260` | outcome=`PurePursuit`, ratio=`0.339`, coverage=`1/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.344`, coverage=`3/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.342`, coverage=`5/10`, escalated=`no` | outcome=`PurePursuit`, ratio=`0.324`, coverage=`10/10`, escalated=`yes` | outcome=`PurePursuit`, ratio=`0.324`, coverage=`10/10`, escalated=`no` |
| `understeer-hairpin/120` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.996`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.997`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.984`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.995`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.995`, coverage=`10/10`, escalated=`no` |
| `understeer-hairpin/200` | outcome=`PurePursuit`, ratio=`0.619`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.982`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.970`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.989`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.989`, coverage=`10/10`, escalated=`no` |
| `understeer-hairpin/300` | outcome=`PurePursuit`, ratio=`0.532`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.973`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.988`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.988`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.988`, coverage=`10/10`, escalated=`no` |
| `velocity-sag-straight/80` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.001`, coverage=`10/10`, escalated=`no` |
| `velocity-sag-straight/140` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.999`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.000`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`1.000`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`1.000`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`1.000`, coverage=`10/10`, escalated=`no` |
| `velocity-sag-straight/220` | outcome=`near-tie`, raw=`Stanley`, ratio=`1.002`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`1.000`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.993`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.993`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PurePursuit`, ratio=`0.993`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `TrackingAggregationVariant`, `TrackingExperimentCase`, `TrackingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`.
