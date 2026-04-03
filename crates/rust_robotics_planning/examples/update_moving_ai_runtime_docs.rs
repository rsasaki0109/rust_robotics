use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use rust_robotics_planning::experiments::moving_ai_runtime::{
    current_runtime_process_problem, default_runtime_variants, long_tail_runtime_process_problem,
    run_variant_suite, synthetic_runtime_process_problem, RuntimeEvaluationConfig,
    RuntimeExperimentCase, RuntimeVariantReport,
};

const NEAR_TIE_RATIO_BAND: f64 = 0.02;

struct ProblemDocSpec<'a> {
    problem_label: &'a str,
    problem_summary: &'a str,
    experiments_filename: &'a str,
    decisions_filename: &'a str,
}

struct ProblemRun<'a> {
    spec: &'a ProblemDocSpec<'a>,
    reports: &'a [RuntimeVariantReport],
}

fn main() {
    let variants = default_runtime_variants();
    let config = RuntimeEvaluationConfig::default();
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root should exist");
    let docs_dir = repo_root.join("docs");

    let crossover_problem = current_runtime_process_problem();
    let long_tail_problem = long_tail_runtime_process_problem();
    let synthetic_problem = synthetic_runtime_process_problem();
    let crossover_reports = run_variant_suite(&variants, &crossover_problem, config);
    let long_tail_reports = run_variant_suite(&variants, &long_tail_problem, config);
    let synthetic_reports = run_variant_suite(&variants, &synthetic_problem, config);

    let crossover_spec = ProblemDocSpec {
        problem_label: "crossover-windows",
        problem_summary: "Compare multiple runtime-aggregation strategies on crossover-focused MovingAI bucket windows before promoting any evaluation method into the stable workflow.",
        experiments_filename: "experiments.md",
        decisions_filename: "decisions.md",
    };
    let long_tail_spec = ProblemDocSpec {
        problem_label: "long-tail-windows",
        problem_summary: "Reuse the same runtime-aggregation contract on long-tail and high-bucket MovingAI windows to test whether the exploratory proxy story survives away from the crossover region.",
        experiments_filename: "experiments_long_tail.md",
        decisions_filename: "decisions_long_tail.md",
    };
    let synthetic_spec = ProblemDocSpec {
        problem_label: "synthetic-local-grids",
        problem_summary: "Reuse the same runtime-aggregation contract on local synthetic grid families derived from the repository's open/maze/dense layouts instead of external MovingAI benchmark families.",
        experiments_filename: "experiments_synthetic.md",
        decisions_filename: "decisions_synthetic.md",
    };

    write_problem_docs(
        &docs_dir,
        &crossover_spec,
        &crossover_reports,
        &crossover_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &long_tail_spec,
        &long_tail_reports,
        &long_tail_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &synthetic_spec,
        &synthetic_reports,
        &synthetic_problem,
        config,
    );

    let summary_runs = [
        ProblemRun {
            spec: &crossover_spec,
            reports: &crossover_reports,
        },
        ProblemRun {
            spec: &long_tail_spec,
            reports: &long_tail_reports,
        },
        ProblemRun {
            spec: &synthetic_spec,
            reports: &synthetic_reports,
        },
    ];
    fs::write(
        docs_dir.join("experiments_planning_summary.md"),
        render_summary_experiments_md(&summary_runs),
    )
    .expect("experiments_planning_summary.md should be writable");
    fs::write(
        docs_dir.join("decisions_planning_summary.md"),
        render_summary_decisions_md(&summary_runs),
    )
    .expect("decisions_planning_summary.md should be writable");
    fs::write(
        docs_dir.join("interfaces.md"),
        render_interfaces_md(&crossover_reports),
    )
    .expect("interfaces.md should be writable");

    println!("updated {}", docs_dir.join("experiments.md").display());
    println!("updated {}", docs_dir.join("decisions.md").display());
    println!(
        "updated {}",
        docs_dir.join("experiments_long_tail.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_long_tail.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("experiments_synthetic.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_synthetic.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("experiments_planning_summary.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_planning_summary.md").display()
    );
    println!("updated {}", docs_dir.join("interfaces.md").display());
}

fn write_problem_docs(
    docs_dir: &Path,
    spec: &ProblemDocSpec<'_>,
    reports: &[RuntimeVariantReport],
    problem: &[RuntimeExperimentCase],
    config: RuntimeEvaluationConfig,
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
    reports: &[RuntimeVariantReport],
    problem: &[RuntimeExperimentCase],
    config: RuntimeEvaluationConfig,
) -> String {
    let variant_ids: Vec<&str> = reports.iter().map(|report| report.descriptor.id).collect();
    let reference = reference_report(reports);
    let mut md = String::new();
    writeln!(&mut md, "# Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Problem\n\nPreset: `{}`\n\n{}",
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
            join_buckets(case.buckets)
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- per-scenario timing iterations: `{}`",
        config.iterations_per_scenario
    )
    .unwrap();
    writeln!(
        &mut md,
        "- near-tie reporting band: `|A*/JPS ratio - 1.0| < {:.3}`",
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
        "Buckets inside the near-tie band are reported as `near-tie`; the raw faster side is still shown for auditability."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    write!(&mut md, "| Family/Bucket ").unwrap();
    for variant_id in &variant_ids {
        write!(&mut md, "| {} ", variant_id).unwrap();
    }
    writeln!(&mut md, "|").unwrap();
    write!(&mut md, "| --- ").unwrap();
    for _ in &variant_ids {
        write!(&mut md, "| --- ").unwrap();
    }
    writeln!(&mut md, "|").unwrap();
    for key in ordered_keys(reports) {
        write!(&mut md, "| `{}/{}` ", key.0, key.1).unwrap();
        for variant_id in &variant_ids {
            write!(&mut md, "| {} ", observation_cell(reports, variant_id, key)).unwrap();
        }
        writeln!(&mut md, "|").unwrap();
    }
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Core / Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Stable core: `RuntimeAggregationVariant`, `RuntimeSamplingPlan`, `RuntimeExperimentCase`, `RuntimeObservation`, `RuntimeVariantReport`, and `run_variant_suite`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Experimental region: concrete selection strategies under `crates/rust_robotics_planning/src/experiments/moving_ai_runtime/`."
    )
    .unwrap();

    md
}

fn render_decisions_md(
    spec: &ProblemDocSpec<'_>,
    reports: &[RuntimeVariantReport],
    problem: &[RuntimeExperimentCase],
) -> String {
    let full = reference_report(reports);
    let mut non_reference: Vec<&RuntimeVariantReport> = reports
        .iter()
        .filter(|report| report.descriptor.id != "full-bucket")
        .collect();
    non_reference.sort_by(|left, right| {
        near_tie_aware_agreement(right, full)
            .partial_cmp(&near_tie_aware_agreement(left, full))
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                right
                    .agreement_vs_reference
                    .partial_cmp(&left.agreement_vs_reference)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| {
                left.evaluation_runtime_ms
                    .partial_cmp(&right.evaluation_runtime_ms)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });
    let best_proxy = non_reference
        .first()
        .copied()
        .expect("at least one non-reference report should exist");
    let cheapest = non_reference
        .iter()
        .min_by(|left, right| {
            left.evaluation_runtime_ms
                .partial_cmp(&right.evaluation_runtime_ms)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .copied()
        .expect("at least one non-reference report should exist");

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
        "- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` scenarios per bucket and therefore defines the current reference labels."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep `{}` as the best current exploratory proxy. It reduces evaluation cost from `{:.2} ms` to `{:.2} ms`, while keeping hard agreement with `full-bucket` at `{}` and near-tie-aware agreement at `{}` on the current problem.",
        best_proxy.descriptor.id,
        full.evaluation_runtime_ms,
        best_proxy.evaluation_runtime_ms,
        format_option(best_proxy.agreement_vs_reference),
        format_option(near_tie_aware_agreement(best_proxy, full))
    )
    .unwrap();
    if cheapest.descriptor.id == best_proxy.descriptor.id {
        writeln!(
            &mut md,
            "- `{}` is also the cheapest smoke-test strategy on this problem. That is useful evidence, but one preset is not enough to collapse the search space into a single permanent strategy.",
            cheapest.descriptor.id
        )
        .unwrap();
    } else {
        writeln!(
            &mut md,
            "- Keep `{}` as the cheapest smoke-test strategy. It runs in `{:.2} ms`, but its hard agreement with `full-bucket` is only `{}` and its near-tie-aware agreement is `{}`, so it should not drive design decisions.",
            cheapest.descriptor.id,
            cheapest.evaluation_runtime_ms,
            format_option(cheapest.agreement_vs_reference),
            format_option(near_tie_aware_agreement(cheapest, full))
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`{}` families) and emit the same `RuntimeObservation` records.",
        problem.len()
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `RuntimeSamplingPlan` has now been reused across multiple validated problem presets without adding new core methods."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Treat buckets inside the near-tie band (`|ratio - 1.0| < {:.3}`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive planner wins.",
        NEAR_TIE_RATIO_BAND
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject single-path convergence. The current comparison set shows that cheaper variants distort the winner label on some buckets even when they are useful for search."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject the idea that one aggregation strategy should become the only path. The current repository should keep `full-bucket` for convergence and cheaper variants for search."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Keep the shared comparison contract from `rust_robotics_core::experiments` fixed while it is reused by planning, localization, and control."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- If the gap between `{}` and `full-bucket` remains large, add another disposable variant instead of widening the core again.",
        best_proxy.descriptor.id
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep only the comparison contract in core. Leave the concrete variants under `experiments/` until at least one more independent package uses the same interface."
    )
    .unwrap();

    md
}

fn render_interfaces_md(reports: &[RuntimeVariantReport]) -> String {
    let mut md = String::new();
    writeln!(&mut md, "# Interfaces").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Current Minimal Interface\n\nThe stable surface is intentionally smaller than the implementation set."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "```rust").unwrap();
    writeln!(&mut md, "pub use rust_robotics_core::experiments::{{").unwrap();
    writeln!(
        &mut md,
        "    ExperimentSamplingPlan as RuntimeSamplingPlan,"
    )
    .unwrap();
    writeln!(&mut md, "    ExperimentVariantReport,").unwrap();
    writeln!(&mut md, "    VariantDescriptor,").unwrap();
    writeln!(&mut md, "}};").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "pub trait RuntimeAggregationVariant {{").unwrap();
    writeln!(&mut md, "    fn descriptor(&self) -> VariantDescriptor;").unwrap();
    writeln!(
        &mut md,
        "    fn selected_slots(&self, total_scenarios: usize) -> Vec<usize>;"
    )
    .unwrap();
    writeln!(
        &mut md,
        "    fn sampling_plan(&self, total_scenarios: usize) -> RuntimeSamplingPlan;"
    )
    .unwrap();
    writeln!(&mut md, "}}").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "pub struct RuntimeExperimentCase {{ /* family, map, scen, buckets */ }}"
    )
    .unwrap();
    writeln!(
        &mut md,
        "pub struct RuntimeObservation {{ /* same metrics for every variant */ }}"
    )
    .unwrap();
    writeln!(
        &mut md,
        "pub type RuntimeVariantReport = ExperimentVariantReport<RuntimeObservation>;"
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "pub fn run_variant_suite(/* variants, cases, config */) -> Vec<RuntimeVariantReport>;"
    )
    .unwrap();
    writeln!(&mut md, "```").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Stability Boundary").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Shared core: `rust_robotics_core::experiments::{{VariantDescriptor, ExperimentSamplingPlan, ExperimentVariantReport, annotate_against_reference, read_source_metrics}}`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Package-local stable surface: `RuntimeAggregationVariant`, `RuntimeExperimentCase`, `RuntimeObservation`, and `run_variant_suite`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Experiments: concrete selection strategies under `src/experiments/moving_ai_runtime/`; sibling validation currently lives in `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`, `crates/rust_robotics_control/src/experiments/path_tracking_accuracy/`, and `crates/rust_robotics_mapping/src/experiments/point_cloud_sampling_quality/`."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Validated Problem Sets").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- `crossover-windows`: [`docs/experiments.md`] + [`docs/decisions.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `long-tail-windows`: [`docs/experiments_long_tail.md`] + [`docs/decisions_long_tail.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `synthetic-local-grids`: [`docs/experiments_synthetic.md`] + [`docs/decisions_synthetic.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-noise-windows`: [`docs/experiments_localization.md`] + [`docs/decisions_localization.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-long-horizon-windows`: [`docs/experiments_localization_long_horizon.md`] + [`docs/decisions_localization_long_horizon.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-dropout-bias-windows`: [`docs/experiments_localization_dropout_bias.md`] + [`docs/decisions_localization_dropout_bias.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-outlier-burst-windows`: [`docs/experiments_localization_outlier_burst.md`] + [`docs/decisions_localization_outlier_burst.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-process-mismatch-windows`: [`docs/experiments_localization_process_mismatch.md`] + [`docs/decisions_localization_process_mismatch.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-sensor-rate-mismatch-windows`: [`docs/experiments_localization_sensor_rate_mismatch.md`] + [`docs/decisions_localization_sensor_rate_mismatch.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-control-latency-windows`: [`docs/experiments_localization_control_latency.md`] + [`docs/decisions_localization_control_latency.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-process-noise-anisotropy-windows`: [`docs/experiments_localization_process_noise_anisotropy.md`] + [`docs/decisions_localization_process_noise_anisotropy.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-sensor-bias-burst-windows`: [`docs/experiments_localization_sensor_bias_burst.md`] + [`docs/decisions_localization_sensor_bias_burst.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `localization-actuator-saturation-windows`: [`docs/experiments_localization_actuator_saturation.md`] + [`docs/decisions_localization_actuator_saturation.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `control-tracking-windows`: [`docs/experiments_control_tracking.md`] + [`docs/decisions_control_tracking.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `control-actuation-mismatch-windows`: [`docs/experiments_control_actuation_mismatch.md`] + [`docs/decisions_control_actuation_mismatch.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-point-cloud-sampling-windows`: [`docs/experiments_mapping_sampling.md`] + [`docs/decisions_mapping_sampling.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-occlusion-corruption-windows`: [`docs/experiments_mapping_occlusion.md`] + [`docs/decisions_mapping_occlusion.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-density-shift-windows`: [`docs/experiments_mapping_density_shift.md`] + [`docs/decisions_mapping_density_shift.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-anisotropic-noise-windows`: [`docs/experiments_mapping_anisotropic_noise.md`] + [`docs/decisions_mapping_anisotropic_noise.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-sparse-outlier-burst-windows`: [`docs/experiments_mapping_sparse_outlier_burst.md`] + [`docs/decisions_mapping_sparse_outlier_burst.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- `mapping-resolution-ladder-windows`: [`docs/experiments_mapping_resolution_ladder.md`] + [`docs/decisions_mapping_resolution_ladder.md`]"
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Cross-Package Summaries").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- planning summary: [`docs/experiments_planning_summary.md`] + [`docs/decisions_planning_summary.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- localization summary: [`docs/experiments_localization_summary.md`] + [`docs/decisions_localization_summary.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- control summary: [`docs/experiments_control_summary.md`] + [`docs/decisions_control_summary.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- mapping summary: [`docs/experiments_mapping_summary.md`] + [`docs/decisions_mapping_summary.md`]"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- workspace summary: [`docs/experiments_workspace_summary.md`] + [`docs/decisions_workspace_summary.md`]"
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Active Variants").unwrap();
    writeln!(&mut md).unwrap();
    for report in reports {
        writeln!(
            &mut md,
            "- `{}`: style=`{}`, source=`{}`",
            report.descriptor.id,
            report.descriptor.design_style,
            relative_source_path(report.descriptor.source_path)
        )
        .unwrap();
    }

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
        "Compare the same five planning aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`."
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
        "- Scope: planning aggregation variants across `{}` independent presets.",
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
        "- Promote only the comparison contract. Current planning presets map to `{}`.",
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
        "- Keep concrete variants disposable inside `experiments/moving_ai_runtime/`. Planning-only evidence still does not justify core promotion of any concrete planning variant."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject choosing one permanent exploratory proxy for planning. Current planning presets cover `{}` concrete winner set(s): {}.",
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
        "- Reject widening the shared surface to encode MovingAI-specific or synthetic-grid-specific regimes. Those belong to disposable presets, not to the core contract."
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
        "- Prefer cross-package synthesis before adding another planning-only preset."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Do not promote any concrete planning aggregation variant into core."
    )
    .unwrap();
    md
}

fn best_proxy_report(reports: &[RuntimeVariantReport]) -> &RuntimeVariantReport {
    let full = reference_report(reports);
    reports
        .iter()
        .filter(|report| report.descriptor.id != "full-bucket")
        .max_by(|left, right| {
            near_tie_aware_agreement(left, full)
                .partial_cmp(&near_tie_aware_agreement(right, full))
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    left.agreement_vs_reference
                        .partial_cmp(&right.agreement_vs_reference)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| {
                    right
                        .evaluation_runtime_ms
                        .partial_cmp(&left.evaluation_runtime_ms)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
        })
        .expect("at least one non-reference report should exist")
}

fn cheapest_non_reference_report(reports: &[RuntimeVariantReport]) -> &RuntimeVariantReport {
    reports
        .iter()
        .filter(|report| report.descriptor.id != "full-bucket")
        .min_by(|left, right| {
            left.evaluation_runtime_ms
                .partial_cmp(&right.evaluation_runtime_ms)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("at least one non-reference report should exist")
}

fn observation_cell(
    reports: &[RuntimeVariantReport],
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
    let ratio = observation.a_over_j();
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
    candidate: &RuntimeVariantReport,
    reference: &RuntimeVariantReport,
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
        if outcome_label(observation.a_over_j()) == outcome_label(reference_observation.a_over_j())
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

fn outcome_label(ratio: f64) -> &'static str {
    if is_near_tie(ratio) {
        "near-tie"
    } else if ratio > 1.0 {
        "JPS"
    } else {
        "A*"
    }
}

fn is_near_tie(ratio: f64) -> bool {
    (ratio - 1.0).abs() < NEAR_TIE_RATIO_BAND
}

fn reference_report(reports: &[RuntimeVariantReport]) -> &RuntimeVariantReport {
    reports
        .iter()
        .find(|report| report.descriptor.id == "full-bucket")
        .expect("full-bucket report should exist")
}

fn coverage_cell(
    observation: &rust_robotics_planning::experiments::moving_ai_runtime::RuntimeObservation,
) -> String {
    if observation.escalated {
        format!(
            "{}/{}->{}/{}",
            observation.initial_slots.len(),
            observation.total_scenarios,
            observation.selected_slots.len(),
            observation.total_scenarios
        )
    } else {
        format!(
            "{}/{}",
            observation.selected_slots.len(),
            observation.total_scenarios
        )
    }
}

fn ordered_keys(reports: &[RuntimeVariantReport]) -> Vec<(&'static str, u32)> {
    let full = reports
        .iter()
        .find(|report| report.descriptor.id == "full-bucket")
        .expect("full-bucket report should exist");
    let mut keys: Vec<(&'static str, u32)> = full
        .observations
        .iter()
        .map(|observation| (observation.family_name, observation.bucket))
        .collect();
    keys.sort_by(|a, b| a.0.cmp(b.0).then(a.1.cmp(&b.1)));
    keys
}

fn relative_source_path(source_path: &str) -> String {
    let source = PathBuf::from(source_path);
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    source
        .strip_prefix(&manifest_dir)
        .map(|path| format!("crates/rust_robotics_planning/{}", path.display()))
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
