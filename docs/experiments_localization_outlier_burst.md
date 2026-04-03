# Experiments

## Problem

Preset: `localization-outlier-burst-windows`

Reuse the same localization aggregation contract on measurement outlier bursts to test whether the exploratory proxy story survives under transient observation corruption.

## Shared Input

- `sparse-outlier-burst` buckets: `120, 200, 280`
- `anisotropic-outlier-burst` buckets: `120, 220, 320`
- `burst-plus-bias` buckets: `140, 220, 300`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 431.68 | 0.100 | 0.778 | 0.778 | 0.076 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1314.43 | 0.300 | 0.778 | 0.667 | 0.074 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2264.03 | 0.500 | 0.667 | 0.778 | 0.034 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 3093.58 | 0.767 | 1.000 | 1.000 | 0.039 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4279.09 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `anisotropic-outlier-burst/120` | outcome=`CKF`, ratio=`1.061`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.021`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.983`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.036`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.036`, coverage=`10/10`, escalated=`no` |
| `anisotropic-outlier-burst/220` | outcome=`near-tie`, raw=`CKF`, ratio=`1.004`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.323`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.106`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.323`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.106`, coverage=`10/10`, escalated=`no` |
| `anisotropic-outlier-burst/320` | outcome=`CKF`, ratio=`1.140`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.140`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.155`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.140`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.070`, coverage=`10/10`, escalated=`no` |
| `burst-plus-bias/140` | outcome=`near-tie`, raw=`UKF`, ratio=`0.999`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.987`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.970`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.970`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`UKF`, ratio=`0.970`, coverage=`10/10`, escalated=`no` |
| `burst-plus-bias/220` | outcome=`UKF`, ratio=`0.888`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.913`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.964`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.002`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`CKF`, ratio=`1.002`, coverage=`10/10`, escalated=`no` |
| `burst-plus-bias/300` | outcome=`CKF`, ratio=`1.231`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.116`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.115`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.116`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.053`, coverage=`10/10`, escalated=`no` |
| `sparse-outlier-burst/120` | outcome=`near-tie`, raw=`CKF`, ratio=`1.018`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.020`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.978`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`UKF`, ratio=`0.978`, coverage=`10/10`, escalated=`no` |
| `sparse-outlier-burst/200` | outcome=`CKF`, ratio=`1.132`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.269`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.223`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.192`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.192`, coverage=`10/10`, escalated=`no` |
| `sparse-outlier-burst/280` | outcome=`CKF`, ratio=`1.040`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.029`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.089`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.106`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.106`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
