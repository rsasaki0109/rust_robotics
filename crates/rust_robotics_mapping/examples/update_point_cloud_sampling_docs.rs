use std::cmp::Ordering;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use rust_robotics_mapping::experiments::point_cloud_sampling_quality::{
    default_point_sampling_variants, mapping_anisotropic_noise_process_problem,
    mapping_density_shift_process_problem, mapping_occlusion_corruption_process_problem,
    mapping_point_cloud_sampling_process_problem, mapping_resolution_ladder_process_problem,
    mapping_sparse_outlier_burst_process_problem, run_variant_suite, PointSamplingEvaluationConfig,
    PointSamplingExperimentCase, PointSamplingObservation, PointSamplingVariantReport,
};

const NEAR_TIE_RATIO_BAND: f64 = 0.03;

struct ProblemDocSpec<'a> {
    problem_label: &'a str,
    problem_summary: &'a str,
    experiments_filename: &'a str,
    decisions_filename: &'a str,
}

struct ProblemRun<'a> {
    spec: &'a ProblemDocSpec<'a>,
    reports: &'a [PointSamplingVariantReport],
}

fn main() {
    let variants = default_point_sampling_variants();
    let config = PointSamplingEvaluationConfig::default();
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root should exist");
    let docs_dir = repo_root.join("docs");

    let baseline_problem = mapping_point_cloud_sampling_process_problem();
    let occlusion_problem = mapping_occlusion_corruption_process_problem();
    let density_shift_problem = mapping_density_shift_process_problem();
    let anisotropic_noise_problem = mapping_anisotropic_noise_process_problem();
    let burst_problem = mapping_sparse_outlier_burst_process_problem();
    let resolution_ladder_problem = mapping_resolution_ladder_process_problem();
    let baseline_reports = run_variant_suite(&variants, &baseline_problem, config);
    let occlusion_reports = run_variant_suite(&variants, &occlusion_problem, config);
    let density_shift_reports = run_variant_suite(&variants, &density_shift_problem, config);
    let anisotropic_noise_reports =
        run_variant_suite(&variants, &anisotropic_noise_problem, config);
    let burst_reports = run_variant_suite(&variants, &burst_problem, config);
    let resolution_ladder_reports =
        run_variant_suite(&variants, &resolution_ladder_problem, config);

    let baseline_spec = ProblemDocSpec {
        problem_label: "mapping-point-cloud-sampling-windows",
        problem_summary: "Compare multiple aggregation strategies on identical synthetic point-cloud families, asking the same question on every family/bucket window: which sampler gives the lowest geometry-retention score at the same nominal sampling budget, `Voxel`, `FarthestPoint`, or `PoissonDisk`?",
        experiments_filename: "experiments_mapping_sampling.md",
        decisions_filename: "decisions_mapping_sampling.md",
    };
    let occlusion_spec = ProblemDocSpec {
        problem_label: "mapping-occlusion-corruption-windows",
        problem_summary: "Reuse the same mapping comparison contract on anisotropically corrupted and structurally occluded clouds, asking which sampler preserves geometry best after slabs or angular sectors disappear.",
        experiments_filename: "experiments_mapping_occlusion.md",
        decisions_filename: "decisions_mapping_occlusion.md",
    };
    let density_shift_spec = ProblemDocSpec {
        problem_label: "mapping-density-shift-windows",
        problem_summary: "Reuse the same mapping comparison contract on density-skewed and anisotropically noisy clouds, asking which sampler preserves geometry best when some regions become overrepresented without explicit dropout.",
        experiments_filename: "experiments_mapping_density_shift.md",
        decisions_filename: "decisions_mapping_density_shift.md",
    };
    let anisotropic_noise_spec = ProblemDocSpec {
        problem_label: "mapping-anisotropic-noise-windows",
        problem_summary: "Reuse the same mapping comparison contract on anisotropic-noise-only clouds, asking which sampler preserves geometry best when all regions stay present but local geometry stretches differently along each axis.",
        experiments_filename: "experiments_mapping_anisotropic_noise.md",
        decisions_filename: "decisions_mapping_anisotropic_noise.md",
    };
    let burst_spec = ProblemDocSpec {
        problem_label: "mapping-sparse-outlier-burst-windows",
        problem_summary: "Reuse the same mapping comparison contract on bursty outlier regimes, asking which sampler preserves geometry best when a few deterministic slots inject abrupt peripheral clutter that fixed early-slot samples may miss.",
        experiments_filename: "experiments_mapping_sparse_outlier_burst.md",
        decisions_filename: "decisions_mapping_sparse_outlier_burst.md",
    };
    let resolution_ladder_spec = ProblemDocSpec {
        problem_label: "mapping-resolution-ladder-windows",
        problem_summary: "Reuse the same mapping comparison contract on slot-dependent resolution ladders, asking which sampler preserves geometry best when mid-bucket slots degrade local density and anisotropy while the endpoints stay comparatively clean.",
        experiments_filename: "experiments_mapping_resolution_ladder.md",
        decisions_filename: "decisions_mapping_resolution_ladder.md",
    };

    write_problem_docs(
        &docs_dir,
        &baseline_spec,
        &baseline_reports,
        &baseline_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &occlusion_spec,
        &occlusion_reports,
        &occlusion_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &density_shift_spec,
        &density_shift_reports,
        &density_shift_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &anisotropic_noise_spec,
        &anisotropic_noise_reports,
        &anisotropic_noise_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &burst_spec,
        &burst_reports,
        &burst_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &resolution_ladder_spec,
        &resolution_ladder_reports,
        &resolution_ladder_problem,
        config,
    );

    let summary_runs = [
        ProblemRun {
            spec: &baseline_spec,
            reports: &baseline_reports,
        },
        ProblemRun {
            spec: &occlusion_spec,
            reports: &occlusion_reports,
        },
        ProblemRun {
            spec: &density_shift_spec,
            reports: &density_shift_reports,
        },
        ProblemRun {
            spec: &anisotropic_noise_spec,
            reports: &anisotropic_noise_reports,
        },
        ProblemRun {
            spec: &burst_spec,
            reports: &burst_reports,
        },
        ProblemRun {
            spec: &resolution_ladder_spec,
            reports: &resolution_ladder_reports,
        },
    ];
    fs::write(
        docs_dir.join("experiments_mapping_summary.md"),
        render_summary_experiments_md(&summary_runs),
    )
    .expect("experiments_mapping_summary.md should be writable");
    fs::write(
        docs_dir.join("decisions_mapping_summary.md"),
        render_summary_decisions_md(&summary_runs),
    )
    .expect("decisions_mapping_summary.md should be writable");

    println!(
        "updated {}",
        docs_dir.join("experiments_mapping_sampling.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_mapping_sampling.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("experiments_mapping_occlusion.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_mapping_occlusion.md").display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_mapping_density_shift.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_mapping_density_shift.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_mapping_anisotropic_noise.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_mapping_anisotropic_noise.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_mapping_sparse_outlier_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_mapping_sparse_outlier_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_mapping_resolution_ladder.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_mapping_resolution_ladder.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir.join("experiments_mapping_summary.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_mapping_summary.md").display()
    );
}

fn write_problem_docs(
    docs_dir: &Path,
    spec: &ProblemDocSpec<'_>,
    reports: &[PointSamplingVariantReport],
    problem: &[PointSamplingExperimentCase],
    config: PointSamplingEvaluationConfig,
) {
    fs::write(
        docs_dir.join(spec.experiments_filename),
        render_experiments_md(spec, reports, problem, config),
    )
    .unwrap_or_else(|_| panic!("{} should be writable", spec.experiments_filename));
    fs::write(
        docs_dir.join(spec.decisions_filename),
        render_decisions_md(spec, reports, problem),
    )
    .unwrap_or_else(|_| panic!("{} should be writable", spec.decisions_filename));
}

fn render_experiments_md(
    spec: &ProblemDocSpec<'_>,
    reports: &[PointSamplingVariantReport],
    problem: &[PointSamplingExperimentCase],
    config: PointSamplingEvaluationConfig,
) -> String {
    let reference = reference_report(reports);
    let mut md = String::new();
    writeln!(&mut md, "# Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Problem").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "Preset: `{}`\n\n{}",
        spec.problem_label, spec.problem_summary
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Shared Input").unwrap();
    writeln!(&mut md).unwrap();
    for case in problem {
        writeln!(
            &mut md,
            "- `{}` buckets: `{}`",
            case.family_name,
            join_buckets(&case.buckets)
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- scenarios per bucket: `{}` deterministic cloud perturbations",
        config.scenarios_per_bucket
    )
    .unwrap();
    writeln!(
        &mut md,
        "- bucket meaning: target retained sample budget and perturbation severity are both tied to the bucket label"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- quality score: original-to-sample coverage error + sample-to-original support error + target-count penalty + spacing penalty + centroid drift"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- near-tie reporting band: `runner_up/best - 1.0 < {:.3}`",
        NEAR_TIE_RATIO_BAND
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Variant Summary\n\n| Variant | Style | Eval ms | Coverage | Hard agreement vs full | Near-tie-aware agreement | Mean ratio error | Code LOC | Branches | Knobs | Dispersion |\n| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"
    )
    .unwrap();
    for report in reports {
        writeln!(
            &mut md,
            "| `{}` | {} | {:.2} | {:.3} | {} | {} | {} | {} | {} | {} | {} |",
            report.descriptor.id,
            report.descriptor.design_style,
            report.evaluation_runtime_ms,
            report.extensibility_metrics.average_coverage_ratio,
            format_option(report.agreement_vs_reference),
            format_option(near_tie_aware_agreement(report, reference)),
            format_option(report.mean_ratio_error_vs_reference),
            report.source_metrics.code_lines,
            report.source_metrics.branch_keywords,
            report.extensibility_metrics.knob_count,
            yes_no(report.extensibility_metrics.reports_dispersion),
        )
        .unwrap();
    }
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Bucket Comparison").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "Buckets inside the near-tie band are reported as `near-tie`; the raw best sampler is still shown for auditability."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "| Family/Bucket | first-scenario | sampled-bucket | percentile-bucket | variance-triggered | full-bucket |\n| --- | --- | --- | --- | --- | --- |"
    )
    .unwrap();
    for key in ordered_keys(reports) {
        writeln!(
            &mut md,
            "| `{}/{}` | {} | {} | {} | {} | {} |",
            key.0,
            key.1,
            observation_cell(reports, "first-scenario", key),
            observation_cell(reports, "sampled-bucket", key),
            observation_cell(reports, "percentile-bucket", key),
            observation_cell(reports, "variance-triggered", key),
            observation_cell(reports, "full-bucket", key),
        )
        .unwrap();
    }
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Core / Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Shared contract pieces live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Package-local stable surface: `PointSamplingAggregationVariant`, `PointSamplingExperimentCase`, `PointSamplingObservation`, and `run_variant_suite`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Experimental region: concrete aggregation strategies under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Active variants: {}.",
        reports
            .iter()
            .map(|report| {
                format!(
                    "`{}` style=`{}` source=`{}`",
                    report.descriptor.id,
                    report.descriptor.design_style,
                    relative_source_path(report.descriptor.source_path)
                )
            })
            .collect::<Vec<_>>()
            .join("; ")
    )
    .unwrap();
    md
}

fn render_decisions_md(
    spec: &ProblemDocSpec<'_>,
    reports: &[PointSamplingVariantReport],
    problem: &[PointSamplingExperimentCase],
) -> String {
    let full = reference_report(reports);
    let best_proxy = best_proxy_report(reports);
    let cheapest = cheapest_non_reference_report(reports);

    let mut md = String::new();
    writeln!(&mut md, "# Decisions").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Context").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "- Problem preset: `{}`", spec.problem_label).unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Adopt").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` cloud perturbations per bucket and therefore defines the current reference labels."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep `{}` as the best current exploratory proxy. It reduces evaluation cost from `{:.2} ms` to `{:.2} ms`, while keeping hard agreement with `full-bucket` at `{}` and near-tie-aware agreement at `{}` on the current mapping problem.",
        best_proxy.descriptor.id,
        full.evaluation_runtime_ms,
        best_proxy.evaluation_runtime_ms,
        format_option(best_proxy.agreement_vs_reference),
        format_option(near_tie_aware_agreement(best_proxy, full))
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep `{}` as the cheapest smoke-test strategy. It runs in `{:.2} ms`; its hard agreement with `full-bucket` is `{}` and its near-tie-aware agreement is `{}`.",
        cheapest.descriptor.id,
        cheapest.evaluation_runtime_ms,
        format_option(cheapest.agreement_vs_reference),
        format_option(near_tie_aware_agreement(cheapest, full))
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`{}` families) and emit the same `PointSamplingObservation` records.",
        problem.len()
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject widening the shared core for mapping-specific quality heuristics. Cloud generation, bucket semantics, and score composition stay package-local."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject promoting any concrete aggregation strategy into core based on a single mapping preset."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Compare this preset against the cross-problem mapping summary before widening the shared surface."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep concrete variants under `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`."
    )
    .unwrap();
    md
}

fn render_summary_experiments_md(runs: &[ProblemRun<'_>]) -> String {
    let mut md = String::new();
    writeln!(&mut md, "# Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Scope").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "Compare the same five mapping aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Preset Comparison\n\n| Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |\n| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |"
    )
    .unwrap();
    for run in runs {
        let full = reference_report(run.reports);
        let best = best_proxy_report(run.reports);
        let cheapest = cheapest_non_reference_report(run.reports);
        writeln!(
            &mut md,
            "| `{}` | `{}` | {:.2} | {} | {} | `{}` | {:.2} | {} | {} | {:.2} |",
            run.spec.problem_label,
            best.descriptor.id,
            best.evaluation_runtime_ms,
            format_option(best.agreement_vs_reference),
            format_option(near_tie_aware_agreement(best, full)),
            cheapest.descriptor.id,
            cheapest.evaluation_runtime_ms,
            format_option(cheapest.agreement_vs_reference),
            format_option(near_tie_aware_agreement(cheapest, full)),
            full.evaluation_runtime_ms,
        )
        .unwrap();
    }
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Proxy Rotation\n\n| Variant | Best-proxy wins | Presets |\n| --- | ---: | --- |"
    )
    .unwrap();
    for variant_id in [
        "first-scenario",
        "sampled-bucket",
        "percentile-bucket",
        "variance-triggered",
    ] {
        let winners = runs
            .iter()
            .filter(|run| best_proxy_report(run.reports).descriptor.id == variant_id)
            .map(|run| run.spec.problem_label)
            .collect::<Vec<_>>();
        writeln!(
            &mut md,
            "| `{}` | {} | {} |",
            variant_id,
            winners.len(),
            if winners.is_empty() {
                "-".to_string()
            } else {
                winners.join(", ")
            }
        )
        .unwrap();
    }
    md
}

fn render_summary_decisions_md(runs: &[ProblemRun<'_>]) -> String {
    let mut winning_variants = runs
        .iter()
        .map(|run| best_proxy_report(run.reports).descriptor.id)
        .collect::<Vec<_>>();
    winning_variants.sort_unstable();
    winning_variants.dedup();

    let mut md = String::new();
    writeln!(&mut md, "# Decisions").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Context").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Scope: mapping aggregation variants across `{}` independent presets.",
        runs.len()
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Adopt").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Keep `full-bucket` as the convergence reference on every preset."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Promote only the comparison contract. Current mapping presets map to `{}`.",
        runs.iter()
            .map(|run| format!(
                "{} -> {}",
                run.spec.problem_label,
                best_proxy_report(run.reports).descriptor.id
            ))
            .collect::<Vec<_>>()
            .join("; ")
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep concrete variants disposable inside `experiments/point_cloud_sampling_quality/`. Package-local evidence still does not justify core promotion of any concrete mapping variant."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject choosing one permanent exploratory proxy for the workspace from mapping-only evidence. Current mapping presets cover `{}` concrete winner set(s): {}.",
        winning_variants.len(),
        winning_variants
            .iter()
            .map(|variant_id| format!("`{variant_id}`"))
            .collect::<Vec<_>>()
            .join(", ")
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject widening the shared surface to encode mapping-specific corruption or noise regimes. Those belong to disposable presets, not to the core contract."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Keep the shared comparison contract fixed while it is reused by planning, localization, control, and mapping."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Prefer another mapping preset or another package before widening `rust_robotics_core::experiments` again."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Do not promote any concrete mapping aggregation variant into core."
    )
    .unwrap();
    md
}

fn observation_cell(
    reports: &[PointSamplingVariantReport],
    variant_id: &str,
    key: (&'static str, u32),
) -> String {
    let report = reports
        .iter()
        .find(|report| report.descriptor.id == variant_id)
        .expect("report should exist");
    let observation = report
        .observations
        .iter()
        .find(|observation| observation.family_name == key.0 && observation.bucket == key.1)
        .expect("observation should exist");
    let ratio = observation.runner_up_over_best();
    if is_near_tie(ratio) {
        format!(
            "outcome=`near-tie`, raw=`{}`, ratio=`{:.3}`, coverage=`{}`, escalated=`{}`",
            observation.winner(),
            ratio,
            coverage_cell(observation),
            yes_no(observation.escalated),
        )
    } else {
        format!(
            "outcome=`{}`, ratio=`{:.3}`, coverage=`{}`, escalated=`{}`",
            observation.winner(),
            ratio,
            coverage_cell(observation),
            yes_no(observation.escalated),
        )
    }
}

fn near_tie_aware_agreement(
    candidate: &PointSamplingVariantReport,
    reference: &PointSamplingVariantReport,
) -> Option<f64> {
    let mut compared = 0usize;
    let mut agreements = 0usize;

    for observation in &candidate.observations {
        let Some(reference_observation) = reference.observations.iter().find(|other| {
            other.family_name == observation.family_name && other.bucket == observation.bucket
        }) else {
            continue;
        };
        compared += 1;
        if outcome_label(observation.runner_up_over_best())
            == outcome_label(reference_observation.runner_up_over_best())
        {
            agreements += 1;
        }
    }

    if compared > 0 {
        Some(agreements as f64 / compared as f64)
    } else {
        None
    }
}

fn best_proxy_report(reports: &[PointSamplingVariantReport]) -> &PointSamplingVariantReport {
    let full = reference_report(reports);
    reports
        .iter()
        .filter(|report| report.descriptor.id != "full-bucket")
        .max_by(|left, right| {
            near_tie_aware_agreement(left, full)
                .partial_cmp(&near_tie_aware_agreement(right, full))
                .unwrap_or(Ordering::Equal)
                .then_with(|| {
                    left.agreement_vs_reference
                        .partial_cmp(&right.agreement_vs_reference)
                        .unwrap_or(Ordering::Equal)
                })
                .then_with(|| {
                    right
                        .evaluation_runtime_ms
                        .partial_cmp(&left.evaluation_runtime_ms)
                        .unwrap_or(Ordering::Equal)
                })
        })
        .expect("at least one non-reference report should exist")
}

fn cheapest_non_reference_report(
    reports: &[PointSamplingVariantReport],
) -> &PointSamplingVariantReport {
    reports
        .iter()
        .filter(|report| report.descriptor.id != "full-bucket")
        .min_by(|left, right| {
            left.evaluation_runtime_ms
                .partial_cmp(&right.evaluation_runtime_ms)
                .unwrap_or(Ordering::Equal)
        })
        .expect("at least one non-reference report should exist")
}

fn ordered_keys(reports: &[PointSamplingVariantReport]) -> Vec<(&'static str, u32)> {
    let mut keys = reports
        .iter()
        .flat_map(|report| {
            report
                .observations
                .iter()
                .map(|observation| (observation.family_name, observation.bucket))
        })
        .collect::<Vec<_>>();
    keys.sort_unstable();
    keys.dedup();
    keys
}

fn reference_report(reports: &[PointSamplingVariantReport]) -> &PointSamplingVariantReport {
    reports
        .iter()
        .find(|report| report.descriptor.id == "full-bucket")
        .expect("full-bucket report should exist")
}

fn outcome_label(ratio: f64) -> &'static str {
    if is_near_tie(ratio) {
        "near-tie"
    } else {
        "decisive"
    }
}

fn is_near_tie(ratio: f64) -> bool {
    (ratio - 1.0).abs() < NEAR_TIE_RATIO_BAND
}

fn coverage_cell(observation: &PointSamplingObservation) -> String {
    format!(
        "{}/{}",
        observation.selected_slots.len(),
        observation.total_scenarios
    )
}

fn relative_source_path(source_path: &str) -> String {
    let source = PathBuf::from(source_path);
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    source
        .strip_prefix(&manifest_dir)
        .map(|path| format!("crates/rust_robotics_mapping/{}", path.display()))
        .unwrap_or_else(|_| source_path.to_string())
}

fn join_buckets(buckets: &[u32]) -> String {
    buckets
        .iter()
        .map(u32::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

fn format_option(value: Option<f64>) -> String {
    value
        .map(|value| format!("{value:.3}"))
        .unwrap_or_else(|| "-".to_string())
}

fn yes_no(value: bool) -> &'static str {
    if value {
        "yes"
    } else {
        "no"
    }
}
