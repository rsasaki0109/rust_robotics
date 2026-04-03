# Experiments

## Problem

Preset: `mapping-density-shift-windows`

Reuse the same mapping comparison contract on density-skewed and anisotropically noisy clouds, asking which sampler preserves geometry best when some regions become overrepresented without explicit dropout.

## Shared Input

- `density-striped-plane` buckets: `48, 96, 144`
- `left-heavy-bridge` buckets: `60, 120, 180`
- `core-heavy-ring` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.03 | 0.100 | 0.889 | 0.889 | 0.091 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 1.000 | 0.889 | 0.036 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 0.778 | 0.778 | 0.030 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.611 | 1.000 | 1.000 | 0.032 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.02 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `core-heavy-ring/72` | outcome=`PoissonDisk`, ratio=`1.058`, coverage=`1/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.051`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.015`, coverage=`5/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.035`, coverage=`10/10`, escalated=`yes` | outcome=`PoissonDisk`, ratio=`1.035`, coverage=`10/10`, escalated=`no` |
| `core-heavy-ring/132` | outcome=`FarthestPoint`, ratio=`1.273`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.560`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.596`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.560`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.585`, coverage=`10/10`, escalated=`no` |
| `core-heavy-ring/192` | outcome=`FarthestPoint`, ratio=`2.330`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.305`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.328`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.305`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.307`, coverage=`10/10`, escalated=`no` |
| `density-striped-plane/48` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.023`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.013`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.008`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.010`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.010`, coverage=`10/10`, escalated=`no` |
| `density-striped-plane/96` | outcome=`PoissonDisk`, ratio=`1.077`, coverage=`1/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.045`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.016`, coverage=`5/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.041`, coverage=`10/10`, escalated=`yes` | outcome=`PoissonDisk`, ratio=`1.041`, coverage=`10/10`, escalated=`no` |
| `density-striped-plane/144` | outcome=`PoissonDisk`, ratio=`1.131`, coverage=`1/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.059`, coverage=`3/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.131`, coverage=`5/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.059`, coverage=`3/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.081`, coverage=`10/10`, escalated=`no` |
| `left-heavy-bridge/60` | outcome=`FarthestPoint`, ratio=`1.033`, coverage=`1/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.037`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.018`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.019`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.019`, coverage=`10/10`, escalated=`no` |
| `left-heavy-bridge/120` | outcome=`FarthestPoint`, ratio=`1.540`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.540`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.516`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.540`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.442`, coverage=`10/10`, escalated=`no` |
| `left-heavy-bridge/180` | outcome=`FarthestPoint`, ratio=`2.261`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.154`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.952`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.154`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.014`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
