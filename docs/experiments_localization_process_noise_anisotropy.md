# Experiments

## Problem

Preset: `localization-process-noise-anisotropy-windows`

Reuse the same localization aggregation contract on anisotropic process disturbance, where truth picks up unequal longitudinal, lateral, and yaw perturbations while both filters keep the same control-driven process model.

## Shared Input

- `crosswind-corridor` buckets: `120, 200, 280`
- `traction-slip-turn` buckets: `140, 220, 320`
- `yaw-dominant-drift` buckets: `140, 240, 340`
- scenarios per bucket: `10` deterministic seeds
- bucket meaning: trajectory steps and noise scale are both tied to the bucket label
- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 449.12 | 0.100 | 1.000 | 1.000 | 0.038 | 29 | 1 | 0 | no |
| `sampled-bucket` | configurable-pipeline | 1353.96 | 0.300 | 0.889 | 1.000 | 0.050 | 39 | 1 | 1 | no |
| `percentile-bucket` | functional-percentile | 2257.87 | 0.500 | 0.889 | 1.000 | 0.059 | 43 | 2 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 2817.32 | 0.611 | 1.000 | 1.000 | 0.041 | 51 | 1 | 2 | yes |
| `full-bucket` | collector-aggregate | 4512.68 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `crosswind-corridor/120` | outcome=`UKF`, ratio=`0.934`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.934`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.934`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.934`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.934`, coverage=`10/10`, escalated=`no` |
| `crosswind-corridor/200` | outcome=`UKF`, ratio=`0.887`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.794`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.627`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.794`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.857`, coverage=`10/10`, escalated=`no` |
| `crosswind-corridor/280` | outcome=`UKF`, ratio=`0.714`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.791`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.739`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.791`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.692`, coverage=`10/10`, escalated=`no` |
| `traction-slip-turn/140` | outcome=`UKF`, ratio=`0.963`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.934`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.963`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.920`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.920`, coverage=`10/10`, escalated=`no` |
| `traction-slip-turn/220` | outcome=`UKF`, ratio=`0.728`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.723`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.755`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.723`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.833`, coverage=`10/10`, escalated=`no` |
| `traction-slip-turn/320` | outcome=`near-tie`, raw=`CKF`, ratio=`1.018`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.993`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`UKF`, ratio=`0.973`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`CKF`, ratio=`1.023`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`CKF`, ratio=`1.023`, coverage=`10/10`, escalated=`no` |
| `yaw-dominant-drift/140` | outcome=`UKF`, ratio=`0.815`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.815`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.815`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.815`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.840`, coverage=`10/10`, escalated=`no` |
| `yaw-dominant-drift/240` | outcome=`UKF`, ratio=`0.848`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.848`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.839`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.848`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.781`, coverage=`10/10`, escalated=`no` |
| `yaw-dominant-drift/340` | outcome=`UKF`, ratio=`0.964`, coverage=`1/10`, escalated=`no` | outcome=`UKF`, ratio=`0.877`, coverage=`3/10`, escalated=`no` | outcome=`UKF`, ratio=`0.917`, coverage=`5/10`, escalated=`no` | outcome=`UKF`, ratio=`0.917`, coverage=`10/10`, escalated=`yes` | outcome=`UKF`, ratio=`0.917`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`.
- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`.
