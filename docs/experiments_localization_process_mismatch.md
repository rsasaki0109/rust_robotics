# Experiments

## Problem

Preset: `localization-process-mismatch-windows`

Reuse the same localization aggregation contract on truth-dynamics mismatch, where the real platform drifts away from the commanded control while both filters still ingest the same noisy commanded odometry.

## Shared Input

- `slip-understeer` buckets: `120, 200, 280`
- `actuator-scale-drift` buckets: `140, 220, 320`
- `oscillatory-wheel-slip` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 448.84 | 0.100 | 1.000 | 1.000 | 0.045 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1334.99 | 0.300 | 1.000 | 1.000 | 0.026 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2228.40 | 0.500 | 0.889 | 0.889 | 0.025 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 2462.58 | 0.611 | 1.000 | 1.000 | 0.015 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4447.57 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `actuator-scale-drift/140` | outcome=`UKF`, ratio=`0.883`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.902`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.881`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.903`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.903`, coverage=`10/10`, escalated=`no` |
| `actuator-scale-drift/220` | outcome=`UKF`, ratio=`0.870`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.838`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.870`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.838`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.838`, coverage=`10/10`, escalated=`no` |
| `actuator-scale-drift/320` | outcome=`CKF`, ratio=`1.167`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.098`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.098`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.099`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.099`, coverage=`10/10`, escalated=`no` |
| `oscillatory-wheel-slip/140` | outcome=`UKF`, ratio=`0.889`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.905`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.830`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.819`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.819`, coverage=`10/10`, escalated=`no` |
| `oscillatory-wheel-slip/240` | outcome=`UKF`, ratio=`0.762`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.762`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.741`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.762`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.708`, coverage=`10/10`, escalated=`no` |
| `oscillatory-wheel-slip/340` | outcome=`CKF`, ratio=`1.268`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.104`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.162`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.104`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.146`, coverage=`10/10`, escalated=`no` |
| `slip-understeer/120` | outcome=`UKF`, ratio=`0.930`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.930`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.026`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.940`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.940`, coverage=`10/10`, escalated=`no` |
| `slip-understeer/200` | outcome=`UKF`, ratio=`0.524`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.485`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.497`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.485`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.517`, coverage=`10/10`, escalated=`no` |
| `slip-understeer/280` | outcome=`UKF`, ratio=`0.599`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.581`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.577`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.581`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.574`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
