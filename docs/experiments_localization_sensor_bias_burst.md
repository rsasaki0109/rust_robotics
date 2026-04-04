# Experiments

## Problem

Preset: `localization-sensor-bias-burst-windows`

Reuse the same localization aggregation contract on deterministic sensor bias bursts, where observations stay finite but drift by a scheduled offset for short windows.

## Shared Input

- `gps-bias-burst` buckets: `120, 200, 280`
- `camera-bias-burst` buckets: `140, 220, 320`
- `anisotropic-bias-burst` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 447.61 | 0.100 | 0.778 | 0.667 | 0.078 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1336.64 | 0.300 | 1.000 | 0.889 | 0.029 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2234.69 | 0.500 | 1.000 | 1.000 | 0.025 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 3040.82 | 0.689 | 1.000 | 1.000 | 0.003 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4486.11 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `anisotropic-bias-burst/140` | outcome=`UKF`, ratio=`0.886`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.851`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.843`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.851`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.843`, coverage=`10/10`, escalated=`no` |
| `anisotropic-bias-burst/240` | outcome=`UKF`, ratio=`0.962`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.858`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.908`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.858`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.866`, coverage=`10/10`, escalated=`no` |
| `anisotropic-bias-burst/340` | outcome=`CKF`, ratio=`1.126`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.177`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.140`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.177`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.171`, coverage=`10/10`, escalated=`no` |
| `camera-bias-burst/140` | outcome=`near-tie`, raw=`UKF`, ratio=`0.980`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.924`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.949`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.949`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.949`, coverage=`10/10`, escalated=`no` |
| `camera-bias-burst/220` | outcome=`CKF`, ratio=`1.034`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.988`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.888`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.863`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.863`, coverage=`10/10`, escalated=`no` |
| `camera-bias-burst/320` | outcome=`CKF`, ratio=`1.143`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.094`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.162`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.121`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.121`, coverage=`10/10`, escalated=`no` |
| `gps-bias-burst/120` | outcome=`UKF`, ratio=`0.888`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.901`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.913`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.901`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.901`, coverage=`10/10`, escalated=`no` |
| `gps-bias-burst/200` | outcome=`UKF`, ratio=`0.860`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.860`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.896`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.860`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.864`, coverage=`10/10`, escalated=`no` |
| `gps-bias-burst/280` | outcome=`CKF`, ratio=`1.173`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.955`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.940`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.897`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.897`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
