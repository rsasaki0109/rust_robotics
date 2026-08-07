# Experiments

## Problem

Preset: `mapping-resolution-ladder-windows`

Reuse the same mapping comparison contract on slot-dependent resolution ladders, asking which sampler preserves geometry best when mid-bucket slots degrade local density and anisotropy while the endpoints stay comparatively clean.

## Shared Input

- `plane-resolution-ladder` buckets: `48, 96, 144`
- `bridge-resolution-ladder` buckets: `60, 120, 180`
- `ring-resolution-ladder` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.02 | 0.100 | 0.778 | 0.778 | 0.114 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 0.889 | 0.889 | 0.114 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 1.000 | 1.000 | 0.040 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.689 | 1.000 | 1.000 | 0.088 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.01 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `bridge-resolution-ladder/60` | outcome=`PoissonDisk`, ratio=`1.040`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.026`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.008`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.027`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.027`, coverage=`10/10`, escalated=`no` |
| `bridge-resolution-ladder/120` | outcome=`FarthestPoint`, ratio=`1.147`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.025`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.252`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.221`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.221`, coverage=`10/10`, escalated=`no` |
| `bridge-resolution-ladder/180` | outcome=`FarthestPoint`, ratio=`2.077`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.474`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.992`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.474`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.764`, coverage=`10/10`, escalated=`no` |
| `plane-resolution-ladder/48` | outcome=`FarthestPoint`, ratio=`1.065`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.065`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.080`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.079`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.079`, coverage=`10/10`, escalated=`no` |
| `plane-resolution-ladder/96` | outcome=`FarthestPoint`, ratio=`1.089`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.031`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.045`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.032`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.032`, coverage=`10/10`, escalated=`no` |
| `plane-resolution-ladder/144` | outcome=`FarthestPoint`, ratio=`1.140`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.101`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.082`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.101`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.080`, coverage=`10/10`, escalated=`no` |
| `ring-resolution-ladder/72` | outcome=`FarthestPoint`, ratio=`1.114`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.003`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.016`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.020`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.020`, coverage=`10/10`, escalated=`no` |
| `ring-resolution-ladder/132` | outcome=`FarthestPoint`, ratio=`1.502`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.532`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.593`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.532`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.632`, coverage=`10/10`, escalated=`no` |
| `ring-resolution-ladder/192` | outcome=`FarthestPoint`, ratio=`2.107`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.998`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.357`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.998`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.379`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
