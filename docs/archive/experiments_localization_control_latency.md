# Experiments

## Problem

Preset: `localization-control-latency-windows`

Reuse the same localization aggregation contract on delayed actuation, where truth follows a lagged command stream while both filters still ingest the current noisy control input.

## Shared Input

- `steering-lag-2` buckets: `120, 200, 280`
- `drive-lag-4` buckets: `140, 220, 320`
- `oscillatory-command-lag-3` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 447.99 | 0.100 | 0.778 | 0.778 | 0.099 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1365.39 | 0.300 | 0.778 | 0.889 | 0.059 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2252.77 | 0.500 | 0.778 | 0.778 | 0.058 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 2434.12 | 0.611 | 1.000 | 1.000 | 0.024 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4678.75 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `drive-lag-4/140` | outcome=`UKF`, ratio=`0.961`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.948`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.948`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.930`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.930`, coverage=`10/10`, escalated=`no` |
| `drive-lag-4/220` | outcome=`UKF`, ratio=`0.915`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.980`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.931`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.011`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`CKF`, ratio=`1.011`, coverage=`10/10`, escalated=`no` |
| `drive-lag-4/320` | outcome=`CKF`, ratio=`1.220`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.184`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.155`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.184`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.176`, coverage=`10/10`, escalated=`no` |
| `oscillatory-command-lag-3/140` | outcome=`UKF`, ratio=`0.778`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.689`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.719`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.689`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.721`, coverage=`10/10`, escalated=`no` |
| `oscillatory-command-lag-3/240` | outcome=`CKF`, ratio=`1.487`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.486`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.452`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.486`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.465`, coverage=`10/10`, escalated=`no` |
| `oscillatory-command-lag-3/340` | outcome=`CKF`, ratio=`1.271`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.316`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.299`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.316`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.299`, coverage=`10/10`, escalated=`no` |
| `steering-lag-2/120` | outcome=`UKF`, ratio=`0.891`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.910`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.882`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.910`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.910`, coverage=`10/10`, escalated=`no` |
| `steering-lag-2/200` | outcome=`CKF`, ratio=`1.131`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.031`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.049`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.767`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.767`, coverage=`10/10`, escalated=`no` |
| `steering-lag-2/280` | outcome=`CKF`, ratio=`1.545`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.456`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.394`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.456`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.314`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
