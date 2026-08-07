# Experiments

## Problem

Preset: `localization-actuator-saturation-windows`

Reuse the same localization aggregation contract on actuator clipping, where truth follows saturated commands while both filters still ingest the unsaturated noisy control stream.

## Shared Input

- `velocity-clipping` buckets: `120, 200, 280`
- `steering-clipping` buckets: `140, 220, 320`
- `coupled-saturation` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 443.52 | 0.100 | 1.000 | 1.000 | 0.043 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1342.00 | 0.300 | 1.000 | 1.000 | 0.050 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2243.19 | 0.500 | 1.000 | 1.000 | 0.046 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 1524.15 | 0.378 | 1.000 | 1.000 | 0.048 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4490.61 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `coupled-saturation/140` | outcome=`UKF`, ratio=`0.812`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.709`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.654`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.709`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.721`, coverage=`10/10`, escalated=`no` |
| `coupled-saturation/240` | outcome=`CKF`, ratio=`1.605`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.535`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.629`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.535`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.596`, coverage=`10/10`, escalated=`no` |
| `coupled-saturation/340` | outcome=`CKF`, ratio=`1.674`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.674`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.735`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.674`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.693`, coverage=`10/10`, escalated=`no` |
| `steering-clipping/140` | outcome=`UKF`, ratio=`0.752`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.673`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.716`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.673`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.728`, coverage=`10/10`, escalated=`no` |
| `steering-clipping/220` | outcome=`CKF`, ratio=`1.397`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.336`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.413`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.336`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.463`, coverage=`10/10`, escalated=`no` |
| `steering-clipping/320` | outcome=`CKF`, ratio=`1.816`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.734`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.754`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.734`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.664`, coverage=`10/10`, escalated=`no` |
| `velocity-clipping/120` | outcome=`UKF`, ratio=`0.928`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.960`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.943`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.945`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.945`, coverage=`10/10`, escalated=`no` |
| `velocity-clipping/200` | outcome=`CKF`, ratio=`1.084`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.106`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.171`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.106`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.086`, coverage=`10/10`, escalated=`no` |
| `velocity-clipping/280` | outcome=`CKF`, ratio=`1.091`, coverage=`1/10`, escalated=`no` | outcome=`CKF`, ratio=`1.147`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.046`, coverage=`5/10`, escalated=`no` | outcome=`CKF`, ratio=`1.147`, coverage=`3/10`, escalated=`no` | outcome=`CKF`, ratio=`1.082`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
