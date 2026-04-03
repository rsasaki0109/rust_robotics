# Experiments

## Problem

Preset: `localization-dropout-bias-windows`

Reuse the same localization aggregation contract on stale-observation and biased-control windows to test whether the exploratory proxy story survives under partial measurement lag and odometry bias.

## Shared Input

- `dropout-lag` buckets: `100, 180, 260`
- `biased-odometry` buckets: `120, 200, 280`
- `dropout-bias-mix` buckets: `120, 200, 280`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 392.89 | 0.100 | 0.889 | 1.000 | 0.032 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1180.28 | 0.300 | 1.000 | 0.889 | 0.030 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 1979.28 | 0.500 | 1.000 | 1.000 | 0.017 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 2025.29 | 0.611 | 1.000 | 1.000 | 0.015 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 3912.51 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `biased-odometry/120` | outcome=`UKF`, ratio=`0.892`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.908`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.918`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.918`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.918`, coverage=`10/10`, escalated=`no` |
| `biased-odometry/200` | outcome=`UKF`, ratio=`0.600`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.563`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.496`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.563`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.452`, coverage=`10/10`, escalated=`no` |
| `biased-odometry/280` | outcome=`UKF`, ratio=`0.604`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.673`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.656`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.673`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.656`, coverage=`10/10`, escalated=`no` |
| `dropout-bias-mix/120` | outcome=`UKF`, ratio=`0.941`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.911`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.920`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.951`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.951`, coverage=`10/10`, escalated=`no` |
| `dropout-bias-mix/200` | outcome=`near-tie`, raw=`CKF`, ratio=`1.002`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.919`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.985`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.980`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`UKF`, ratio=`0.980`, coverage=`10/10`, escalated=`no` |
| `dropout-bias-mix/280` | outcome=`CKF`, ratio=`1.129`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.129`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.166`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.129`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.129`, coverage=`10/10`, escalated=`no` |
| `dropout-lag/100` | outcome=`UKF`, ratio=`0.941`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.902`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.939`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.928`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.928`, coverage=`10/10`, escalated=`no` |
| `dropout-lag/180` | outcome=`UKF`, ratio=`0.876`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.876`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.892`, coverage=`10/10`, escalated=`no` |
| `dropout-lag/260` | outcome=`UKF`, ratio=`0.798`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.799`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.811`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.799`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.799`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
