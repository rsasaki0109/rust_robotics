# Experiments

## Problem

Preset: `mapping-sparse-outlier-burst-windows`

Reuse the same mapping comparison contract on bursty outlier regimes, asking which sampler preserves geometry best when a few deterministic slots inject abrupt peripheral clutter that fixed early-slot samples may miss.

## Shared Input

- `plane-burst-outliers` buckets: `48, 96, 144`
- `bridge-burst-outliers` buckets: `60, 120, 180`
- `ring-burst-outliers` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.03 | 0.100 | 1.000 | 0.889 | 0.067 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 1.000 | 1.000 | 0.047 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 0.889 | 0.889 | 0.042 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.611 | 1.000 | 1.000 | 0.033 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.01 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `bridge-burst-outliers/60` | outcome=`FarthestPoint`, ratio=`1.154`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.127`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.028`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.074`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.074`, coverage=`10/10`, escalated=`no` |
| `bridge-burst-outliers/120` | outcome=`FarthestPoint`, ratio=`1.263`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.124`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.101`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.094`, coverage=`10/10`, escalated=`no` |
| `bridge-burst-outliers/180` | outcome=`FarthestPoint`, ratio=`1.561`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.742`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.370`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.742`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.567`, coverage=`10/10`, escalated=`no` |
| `plane-burst-outliers/48` | outcome=`FarthestPoint`, ratio=`1.095`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.087`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.104`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.087`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.103`, coverage=`10/10`, escalated=`no` |
| `plane-burst-outliers/96` | outcome=`FarthestPoint`, ratio=`1.055`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.088`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.123`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.089`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.089`, coverage=`10/10`, escalated=`no` |
| `plane-burst-outliers/144` | outcome=`FarthestPoint`, ratio=`1.167`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.149`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.141`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.149`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.152`, coverage=`10/10`, escalated=`no` |
| `ring-burst-outliers/72` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.014`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.106`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.106`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.059`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.059`, coverage=`10/10`, escalated=`no` |
| `ring-burst-outliers/132` | outcome=`FarthestPoint`, ratio=`1.428`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.472`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.407`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.472`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.372`, coverage=`10/10`, escalated=`no` |
| `ring-burst-outliers/192` | outcome=`FarthestPoint`, ratio=`1.856`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.051`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.051`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.051`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.051`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
