# Experiments

## Problem

Preset: `localization-long-horizon-windows`

Reuse the same localization aggregation contract on longer trajectories and higher-noise windows to test whether the exploratory proxy story survives away from the moderate-noise regime.

## Shared Input

- `long-horizon-turn` buckets: `180, 240, 320`
- `high-curvature` buckets: `140, 220, 300`
- `anisotropic-gps` buckets: `160, 240, 320`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 475.98 | 0.100 | 0.889 | 0.889 | 0.090 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1417.57 | 0.300 | 1.000 | 1.000 | 0.039 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2557.57 | 0.500 | 0.889 | 0.889 | 0.052 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 2672.73 | 0.533 | 1.000 | 1.000 | 0.035 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4810.27 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `anisotropic-gps/160` | outcome=`UKF`, ratio=`0.893`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.890`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.919`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.890`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.921`, coverage=`10/10`, escalated=`no` |
| `anisotropic-gps/240` | outcome=`UKF`, ratio=`0.835`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.835`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.808`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.835`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.790`, coverage=`10/10`, escalated=`no` |
| `anisotropic-gps/320` | outcome=`CKF`, ratio=`1.172`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.233`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.180`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.233`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.233`, coverage=`10/10`, escalated=`no` |
| `high-curvature/140` | outcome=`CKF`, ratio=`1.149`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.149`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.136`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.136`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.136`, coverage=`10/10`, escalated=`no` |
| `high-curvature/220` | outcome=`CKF`, ratio=`1.126`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.137`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.218`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.137`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.159`, coverage=`10/10`, escalated=`no` |
| `high-curvature/300` | outcome=`CKF`, ratio=`1.056`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.093`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.060`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.116`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.116`, coverage=`10/10`, escalated=`no` |
| `long-horizon-turn/180` | outcome=`UKF`, ratio=`0.805`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.092`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.980`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.092`, coverage=`10/10`, escalated=`yes` | outcome=`CKF`, ratio=`1.092`, coverage=`10/10`, escalated=`no` |
| `long-horizon-turn/240` | outcome=`CKF`, ratio=`1.619`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.617`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.547`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.617`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.447`, coverage=`10/10`, escalated=`no` |
| `long-horizon-turn/320` | outcome=`CKF`, ratio=`1.387`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.546`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.565`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.546`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.498`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
