# Experiments

## Problem

Preset: `mapping-anisotropic-noise-windows`

Reuse the same mapping comparison contract on anisotropic-noise-only clouds, asking which sampler preserves geometry best when all regions stay present but local geometry stretches differently along each axis.

## Shared Input

- `sheared-plane-noise` buckets: `48, 96, 144`
- `bridge-z-noise` buckets: `60, 120, 180`
- `radial-ring-noise` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.03 | 0.100 | 0.889 | 0.889 | 0.048 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 1.000 | 1.000 | 0.053 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 1.000 | 0.889 | 0.013 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.533 | 1.000 | 1.000 | 0.044 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.01 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `bridge-z-noise/60` | outcome=`FarthestPoint`, ratio=`1.099`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.098`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.045`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.042`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.042`, coverage=`10/10`, escalated=`no` |
| `bridge-z-noise/120` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.001`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.172`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.087`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.172`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`10/10`, escalated=`no` |
| `bridge-z-noise/180` | outcome=`FarthestPoint`, ratio=`1.660`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.635`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.601`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.635`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.634`, coverage=`10/10`, escalated=`no` |
| `radial-ring-noise/72` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.025`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.002`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.044`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.017`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.017`, coverage=`10/10`, escalated=`no` |
| `radial-ring-noise/132` | outcome=`FarthestPoint`, ratio=`1.413`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.329`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.470`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.329`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.460`, coverage=`10/10`, escalated=`no` |
| `radial-ring-noise/192` | outcome=`FarthestPoint`, ratio=`2.199`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.152`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.050`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.152`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.042`, coverage=`10/10`, escalated=`no` |
| `sheared-plane-noise/48` | outcome=`FarthestPoint`, ratio=`1.102`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.096`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.116`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.096`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.113`, coverage=`10/10`, escalated=`no` |
| `sheared-plane-noise/96` | outcome=`FarthestPoint`, ratio=`1.114`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.151`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.080`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.151`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`10/10`, escalated=`no` |
| `sheared-plane-noise/144` | outcome=`FarthestPoint`, ratio=`1.135`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.133`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.136`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.149`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.149`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
