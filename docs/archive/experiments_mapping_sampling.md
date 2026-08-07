# Experiments

## Problem

Preset: `mapping-point-cloud-sampling-windows`

Compare multiple aggregation strategies on identical synthetic point-cloud families, asking the same question on every family/bucket window: which sampler gives the lowest geometry-retention score at the same nominal sampling budget, `Voxel`, `FarthestPoint`, or `PoissonDisk`?

## Shared Input

- `layered-plane` buckets: `48, 96, 144`
- `twin-cluster-bridge` buckets: `60, 120, 180`
- `ring-with-outliers` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.03 | 0.100 | 1.000 | 1.000 | 0.046 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 0.889 | 0.889 | 0.025 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 1.000 | 1.000 | 0.025 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.611 | 1.000 | 1.000 | 0.014 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.01 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `layered-plane/48` | outcome=`FarthestPoint`, ratio=`1.107`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.105`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.103`, coverage=`10/10`, escalated=`no` |
| `layered-plane/96` | outcome=`FarthestPoint`, ratio=`1.093`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.092`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.120`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.092`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.096`, coverage=`10/10`, escalated=`no` |
| `layered-plane/144` | outcome=`FarthestPoint`, ratio=`1.116`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.116`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.138`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.144`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.144`, coverage=`10/10`, escalated=`no` |
| `ring-with-outliers/72` | outcome=`FarthestPoint`, ratio=`1.062`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.034`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.078`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.057`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.057`, coverage=`10/10`, escalated=`no` |
| `ring-with-outliers/132` | outcome=`FarthestPoint`, ratio=`1.300`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.327`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.321`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.327`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.344`, coverage=`10/10`, escalated=`no` |
| `ring-with-outliers/192` | outcome=`FarthestPoint`, ratio=`1.918`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.978`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.995`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.978`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.015`, coverage=`10/10`, escalated=`no` |
| `twin-cluster-bridge/60` | outcome=`FarthestPoint`, ratio=`1.127`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.018`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.089`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.061`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.061`, coverage=`10/10`, escalated=`no` |
| `twin-cluster-bridge/120` | outcome=`FarthestPoint`, ratio=`1.241`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.123`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.146`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.128`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.128`, coverage=`10/10`, escalated=`no` |
| `twin-cluster-bridge/180` | outcome=`FarthestPoint`, ratio=`1.770`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.778`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.635`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.778`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.719`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
