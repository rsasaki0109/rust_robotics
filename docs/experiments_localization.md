# Experiments

## Problem

Preset: `localization-noise-windows`

Compare multiple bucket-aggregation strategies on a non-grid localization problem by asking the same question on every case/bucket window: which filter is more accurate on median RMSE, UKF or CKF?

## Shared Input

- `balanced-circle` buckets: `60, 100, 140`
- `fast-turn` buckets: `80, 120, 160`
- `gps-stress` buckets: `80, 120, 160`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 226.65 | 0.100 | 1.000 | 1.000 | 0.033 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 671.15 | 0.300 | 1.000 | 1.000 | 0.021 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 1132.08 | 0.500 | 1.000 | 1.000 | 0.011 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 995.32 | 0.456 | 1.000 | 1.000 | 0.018 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 2401.06 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `balanced-circle/60` | outcome=`UKF`, ratio=`0.898`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.898`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.898`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.898`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.898`, coverage=`10/10`, escalated=`no` |
| `balanced-circle/100` | outcome=`UKF`, ratio=`0.886`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.886`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.918`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.886`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.931`, coverage=`10/10`, escalated=`no` |
| `balanced-circle/140` | outcome=`UKF`, ratio=`0.907`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.895`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.922`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.895`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.922`, coverage=`10/10`, escalated=`no` |
| `fast-turn/80` | outcome=`UKF`, ratio=`0.933`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.862`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.920`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.862`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`10/10`, escalated=`no` |
| `fast-turn/120` | outcome=`UKF`, ratio=`0.831`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.879`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.856`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.879`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.857`, coverage=`10/10`, escalated=`no` |
| `fast-turn/160` | outcome=`UKF`, ratio=`0.956`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.889`, coverage=`10/10`, escalated=`no` |
| `gps-stress/80` | outcome=`UKF`, ratio=`0.954`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.919`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.949`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.931`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.931`, coverage=`10/10`, escalated=`no` |
| `gps-stress/120` | outcome=`UKF`, ratio=`0.969`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.969`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.968`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.957`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.957`, coverage=`10/10`, escalated=`no` |
| `gps-stress/160` | outcome=`UKF`, ratio=`0.850`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.873`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.890`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.873`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.915`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
