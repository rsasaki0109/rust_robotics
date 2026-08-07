# Experiments

## Problem

Preset: `mapping-occlusion-corruption-windows`

Reuse the same mapping comparison contract on anisotropically corrupted and structurally occluded clouds, asking which sampler preserves geometry best after slabs or angular sectors disappear.

## Shared Input

- `slab-occluded-plane` buckets: `48, 96, 144`
- `bridge-shadow-clusters` buckets: `60, 120, 180`
- `sector-dropout-ring` buckets: `72, 132, 192`
- scenarios per bucket: `10` deterministic cloud perturbations
- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label
- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift
- near-tie reporting band: `runner_up/best - 1.0 < 0.030`

## Variant Summary

| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `first-scenario` | direct-imperative | 0.03 | 0.100 | 0.778 | 0.778 | 0.081 | 29 | 1 | 0 | no |
| `sampled-bucket` | fixed-window | 0.01 | 0.300 | 0.889 | 0.889 | 0.049 | 39 | 1 | 1 | no |
| `percentile-bucket` | percentile-spread | 0.01 | 0.500 | 0.778 | 0.889 | 0.029 | 38 | 1 | 1 | yes |
| `variance-triggered` | adaptive-two-stage | 0.02 | 0.611 | 1.000 | 1.000 | 0.039 | 51 | 1 | 2 | yes |
| `full-bucket` | exhaustive-reference | 0.01 | 1.000 | 1.000 | 1.000 | 0.000 | 25 | 0 | 0 | yes |

## Bucket Comparison

Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability.

| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |
| --- | --- | --- | --- | --- | --- |
| `bridge-shadow-clusters/60` | outcome=`FarthestPoint`, ratio=`1.069`, coverage=`1/10`, escalated=`no` | outcome=`PoissonDisk`, ratio=`1.032`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.036`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.002`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.002`, coverage=`10/10`, escalated=`no` |
| `bridge-shadow-clusters/120` | outcome=`FarthestPoint`, ratio=`1.709`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.048`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.801`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.048`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.787`, coverage=`10/10`, escalated=`no` |
| `bridge-shadow-clusters/180` | outcome=`FarthestPoint`, ratio=`3.323`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.947`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`3.021`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.947`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.909`, coverage=`10/10`, escalated=`no` |
| `sector-dropout-ring/72` | outcome=`FarthestPoint`, ratio=`1.053`, coverage=`1/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.013`, coverage=`3/10`, escalated=`no` | outcome=`near-tie`, raw=`FarthestPoint`, ratio=`1.013`, coverage=`5/10`, escalated=`no` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.003`, coverage=`10/10`, escalated=`yes` | outcome=`near-tie`, raw=`PoissonDisk`, ratio=`1.003`, coverage=`10/10`, escalated=`no` |
| `sector-dropout-ring/132` | outcome=`FarthestPoint`, ratio=`1.656`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.665`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.625`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.665`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.667`, coverage=`10/10`, escalated=`no` |
| `sector-dropout-ring/192` | outcome=`FarthestPoint`, ratio=`2.391`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.395`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.327`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.395`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`2.347`, coverage=`10/10`, escalated=`no` |
| `slab-occluded-plane/48` | outcome=`FarthestPoint`, ratio=`1.077`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.085`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.071`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.084`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.084`, coverage=`10/10`, escalated=`no` |
| `slab-occluded-plane/96` | outcome=`FarthestPoint`, ratio=`1.069`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.052`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.109`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.097`, coverage=`10/10`, escalated=`yes` | outcome=`FarthestPoint`, ratio=`1.097`, coverage=`10/10`, escalated=`no` |
| `slab-occluded-plane/144` | outcome=`FarthestPoint`, ratio=`1.181`, coverage=`1/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.154`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.142`, coverage=`5/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.154`, coverage=`3/10`, escalated=`no` | outcome=`FarthestPoint`, ratio=`1.150`, coverage=`10/10`, escalated=`no` |

## Core / Experiments

- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`.
- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`.
- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`.
- Active variants: `first-scenario` style=`direct-imperative` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/first_scenario.rs`; `sampled-bucket` style=`fixed-window` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/sampled_bucket.rs`; `percentile-bucket` style=`percentile-spread` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/percentile_bucket.rs`; `variance-triggered` style=`adaptive-two-stage` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/variance_triggered.rs`; `full-bucket` style=`exhaustive-reference` source=`crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/full_bucket.rs`.
