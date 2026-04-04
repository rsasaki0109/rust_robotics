# Experiments

## Problem

Preset: `localization-sensor-rate-mismatch-windows`

Reuse the same localization aggregation contract on structured low-rate sensing, where observations only refresh on a deterministic cadence while both filters still step on every control tick.

## Shared Input

- `slow-gps-3x` buckets: `120, 200, 280`
- `anisotropic-gps-5x` buckets: `140, 220, 320`
- `turning-camera-4x` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 448.84 | 0.100 | 0.778 | 0.667 | 0.099 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1355.38 | 0.300 | 0.778 | 0.778 | 0.048 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2237.71 | 0.500 | 0.889 | 0.889 | 0.021 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 3054.42 | 0.689 | 0.889 | 0.889 | 0.031 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4473.06 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `anisotropic-gps-5x/140` | outcome=`near-tie`, raw=`UKF`, ratio=`0.984`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.003`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.969`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.991`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`UKF`, ratio=`0.991`, coverage=`10/10`, escalated=`no` |
| `anisotropic-gps-5x/220` | outcome=`CKF`, ratio=`1.177`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.177`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.001`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.177`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.977`, coverage=`10/10`, escalated=`no` |
| `anisotropic-gps-5x/320` | outcome=`CKF`, ratio=`1.068`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.067`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.105`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.093`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.093`, coverage=`10/10`, escalated=`no` |
| `slow-gps-3x/120` | outcome=`UKF`, ratio=`0.933`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.927`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.927`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.935`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.935`, coverage=`10/10`, escalated=`no` |
| `slow-gps-3x/200` | outcome=`UKF`, ratio=`0.853`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.774`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.842`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.774`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.802`, coverage=`10/10`, escalated=`no` |
| `slow-gps-3x/280` | outcome=`UKF`, ratio=`0.871`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.239`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.213`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.239`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.239`, coverage=`10/10`, escalated=`no` |
| `turning-camera-4x/140` | outcome=`near-tie`, raw=`UKF`, ratio=`0.981`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.987`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.900`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.880`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.880`, coverage=`10/10`, escalated=`no` |
| `turning-camera-4x/240` | outcome=`CKF`, ratio=`1.250`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.174`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.190`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.174`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.174`, coverage=`10/10`, escalated=`no` |
| `turning-camera-4x/340` | outcome=`CKF`, ratio=`1.181`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.172`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.104`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.172`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.121`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
