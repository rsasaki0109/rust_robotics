use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics_localization::experiments::ukf_ckf_accuracy::{
    default_accuracy_variants, localization_actuator_saturation_process_problem,
    localization_control_latency_process_problem, localization_dropout_bias_process_problem,
    localization_long_horizon_process_problem, localization_noise_process_problem,
    localization_outlier_burst_process_problem, localization_process_mismatch_process_problem,
    localization_process_noise_anisotropy_process_problem,
    localization_sensor_bias_burst_process_problem,
    localization_sensor_rate_mismatch_process_problem, run_variant_suite, AccuracyEvaluationConfig,
    AccuracyExperimentCase, AccuracyObservation, AccuracyVariantReport,
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
    reports: &'a [AccuracyVariantReport],
}

fn main() {
    let variants = default_accuracy_variants();
    let config = AccuracyEvaluationConfig::default();
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root should exist");
    let docs_dir = repo_root.join("docs");
    let baseline_problem = localization_noise_process_problem();
    let long_horizon_problem = localization_long_horizon_process_problem();
    let dropout_bias_problem = localization_dropout_bias_process_problem();
    let outlier_burst_problem = localization_outlier_burst_process_problem();
    let process_mismatch_problem = localization_process_mismatch_process_problem();
    let sensor_rate_mismatch_problem = localization_sensor_rate_mismatch_process_problem();
    let control_latency_problem = localization_control_latency_process_problem();
    let process_noise_anisotropy_problem = localization_process_noise_anisotropy_process_problem();
    let sensor_bias_burst_problem = localization_sensor_bias_burst_process_problem();
    let actuator_saturation_problem = localization_actuator_saturation_process_problem();

    let baseline_reports = run_variant_suite(&variants, &baseline_problem, config);
    let long_horizon_reports = run_variant_suite(&variants, &long_horizon_problem, config);
    let dropout_bias_reports = run_variant_suite(&variants, &dropout_bias_problem, config);
    let outlier_burst_reports = run_variant_suite(&variants, &outlier_burst_problem, config);
    let process_mismatch_reports = run_variant_suite(&variants, &process_mismatch_problem, config);
    let sensor_rate_mismatch_reports =
        run_variant_suite(&variants, &sensor_rate_mismatch_problem, config);
    let control_latency_reports = run_variant_suite(&variants, &control_latency_problem, config);
    let process_noise_anisotropy_reports =
        run_variant_suite(&variants, &process_noise_anisotropy_problem, config);
    let sensor_bias_burst_reports =
        run_variant_suite(&variants, &sensor_bias_burst_problem, config);
    let actuator_saturation_reports =
        run_variant_suite(&variants, &actuator_saturation_problem, config);
    let baseline_spec = ProblemDocSpec {
        problem_label: "localization-noise-windows",
        problem_summary: "Compare multiple bucket-aggregation strategies on a non-grid localization problem by asking the same question on every case/bucket window: which filter is more accurate on median RMSE, UKF or CKF?",
        experiments_filename: "experiments_localization.md",
        decisions_filename: "decisions_localization.md",
    };
    let long_horizon_spec = ProblemDocSpec {
        problem_label: "localization-long-horizon-windows",
        problem_summary: "Reuse the same localization aggregation contract on longer trajectories and higher-noise windows to test whether the exploratory proxy story survives away from the moderate-noise regime.",
        experiments_filename: "experiments_localization_long_horizon.md",
        decisions_filename: "decisions_localization_long_horizon.md",
    };
    let dropout_bias_spec = ProblemDocSpec {
        problem_label: "localization-dropout-bias-windows",
        problem_summary: "Reuse the same localization aggregation contract on stale-observation and biased-control windows to test whether the exploratory proxy story survives under partial measurement lag and odometry bias.",
        experiments_filename: "experiments_localization_dropout_bias.md",
        decisions_filename: "decisions_localization_dropout_bias.md",
    };
    let outlier_burst_spec = ProblemDocSpec {
        problem_label: "localization-outlier-burst-windows",
        problem_summary: "Reuse the same localization aggregation contract on measurement outlier bursts to test whether the exploratory proxy story survives under transient observation corruption.",
        experiments_filename: "experiments_localization_outlier_burst.md",
        decisions_filename: "decisions_localization_outlier_burst.md",
    };
    let process_mismatch_spec = ProblemDocSpec {
        problem_label: "localization-process-mismatch-windows",
        problem_summary: "Reuse the same localization aggregation contract on truth-dynamics mismatch, where the real platform drifts away from the commanded control while both filters still ingest the same noisy commanded odometry.",
        experiments_filename: "experiments_localization_process_mismatch.md",
        decisions_filename: "decisions_localization_process_mismatch.md",
    };
    let sensor_rate_mismatch_spec = ProblemDocSpec {
        problem_label: "localization-sensor-rate-mismatch-windows",
        problem_summary: "Reuse the same localization aggregation contract on structured low-rate sensing, where observations only refresh on a deterministic cadence while both filters still step on every control tick.",
        experiments_filename: "experiments_localization_sensor_rate_mismatch.md",
        decisions_filename: "decisions_localization_sensor_rate_mismatch.md",
    };
    let control_latency_spec = ProblemDocSpec {
        problem_label: "localization-control-latency-windows",
        problem_summary: "Reuse the same localization aggregation contract on delayed actuation, where truth follows a lagged command stream while both filters still ingest the current noisy control input.",
        experiments_filename: "experiments_localization_control_latency.md",
        decisions_filename: "decisions_localization_control_latency.md",
    };
    let process_noise_anisotropy_spec = ProblemDocSpec {
        problem_label: "localization-process-noise-anisotropy-windows",
        problem_summary: "Reuse the same localization aggregation contract on anisotropic process disturbance, where truth picks up unequal longitudinal, lateral, and yaw perturbations while both filters keep the same control-driven process model.",
        experiments_filename: "experiments_localization_process_noise_anisotropy.md",
        decisions_filename: "decisions_localization_process_noise_anisotropy.md",
    };
    let sensor_bias_burst_spec = ProblemDocSpec {
        problem_label: "localization-sensor-bias-burst-windows",
        problem_summary: "Reuse the same localization aggregation contract on deterministic sensor bias bursts, where observations stay finite but drift by a scheduled offset for short windows.",
        experiments_filename: "experiments_localization_sensor_bias_burst.md",
        decisions_filename: "decisions_localization_sensor_bias_burst.md",
    };
    let actuator_saturation_spec = ProblemDocSpec {
        problem_label: "localization-actuator-saturation-windows",
        problem_summary: "Reuse the same localization aggregation contract on actuator clipping, where truth follows saturated commands while both filters still ingest the unsaturated noisy control stream.",
        experiments_filename: "experiments_localization_actuator_saturation.md",
        decisions_filename: "decisions_localization_actuator_saturation.md",
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
        &long_horizon_spec,
        &long_horizon_reports,
        &long_horizon_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &dropout_bias_spec,
        &dropout_bias_reports,
        &dropout_bias_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &outlier_burst_spec,
        &outlier_burst_reports,
        &outlier_burst_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &process_mismatch_spec,
        &process_mismatch_reports,
        &process_mismatch_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &sensor_rate_mismatch_spec,
        &sensor_rate_mismatch_reports,
        &sensor_rate_mismatch_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &control_latency_spec,
        &control_latency_reports,
        &control_latency_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &process_noise_anisotropy_spec,
        &process_noise_anisotropy_reports,
        &process_noise_anisotropy_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &sensor_bias_burst_spec,
        &sensor_bias_burst_reports,
        &sensor_bias_burst_problem,
        config,
    );
    write_problem_docs(
        &docs_dir,
        &actuator_saturation_spec,
        &actuator_saturation_reports,
        &actuator_saturation_problem,
        config,
    );
    let summary_runs = [
        ProblemRun {
            spec: &baseline_spec,
            reports: &baseline_reports,
        },
        ProblemRun {
            spec: &long_horizon_spec,
            reports: &long_horizon_reports,
        },
        ProblemRun {
            spec: &dropout_bias_spec,
            reports: &dropout_bias_reports,
        },
        ProblemRun {
            spec: &outlier_burst_spec,
            reports: &outlier_burst_reports,
        },
        ProblemRun {
            spec: &process_mismatch_spec,
            reports: &process_mismatch_reports,
        },
        ProblemRun {
            spec: &sensor_rate_mismatch_spec,
            reports: &sensor_rate_mismatch_reports,
        },
        ProblemRun {
            spec: &control_latency_spec,
            reports: &control_latency_reports,
        },
        ProblemRun {
            spec: &process_noise_anisotropy_spec,
            reports: &process_noise_anisotropy_reports,
        },
        ProblemRun {
            spec: &sensor_bias_burst_spec,
            reports: &sensor_bias_burst_reports,
        },
        ProblemRun {
            spec: &actuator_saturation_spec,
            reports: &actuator_saturation_reports,
        },
    ];
    fs::write(
        docs_dir.join("experiments_localization_summary.md"),
        render_summary_experiments_md(&summary_runs),
    )
    .expect("experiments_localization_summary.md should be writable");
    fs::write(
        docs_dir.join("decisions_localization_summary.md"),
        render_summary_decisions_md(&summary_runs),
    )
    .expect("decisions_localization_summary.md should be writable");

    println!(
        "updated {}",
        docs_dir.join("experiments_localization.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_localization.md").display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_long_horizon.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_long_horizon.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_dropout_bias.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_dropout_bias.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_outlier_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_outlier_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_process_mismatch.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_process_mismatch.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_sensor_rate_mismatch.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_sensor_rate_mismatch.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_control_latency.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_control_latency.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_process_noise_anisotropy.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_process_noise_anisotropy.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_sensor_bias_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_sensor_bias_burst.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_actuator_saturation.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("decisions_localization_actuator_saturation.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir
            .join("experiments_localization_summary.md")
            .display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_localization_summary.md").display()
    );
}

fn write_problem_docs(
    docs_dir: &Path,
    spec: &ProblemDocSpec<'_>,
    reports: &[AccuracyVariantReport],
    problem: &[AccuracyExperimentCase],
    config: AccuracyEvaluationConfig,
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
    reports: &[AccuracyVariantReport],
    problem: &[AccuracyExperimentCase],
    config: AccuracyEvaluationConfig,
) -> String {
    let variant_ids: Vec<&str> = reports.iter().map(|report| report.descriptor.id).collect();
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
            join_buckets(case.buckets)
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- scenarios per bucket: `{}` deterministic seeds",
        config.scenarios_per_bucket
    )
    .unwrap();
    writeln!(
        &mut md,
        "- bucket meaning: trajectory steps and noise scale are both tied to the bucket label"
    )
    .unwrap();
    writeln!(
        &mut md,
        "- near-tie reporting band: `|UKF/CKF RMSE ratio - 1.0| < {:.3}`",
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
        "Buckets inside the near-tie band are reported as `near-tie`; the raw lower-RMSE side is still shown for auditability."
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
        "- Shared contract pieces now live in `rust_robotics_core::experiments`: `ExperimentSamplingPlan`, `VariantDescriptor`, `ExperimentVariantReport`, `annotate_against_reference`, and `read_source_metrics`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Package-local stable surface: `AccuracyAggregationVariant`, `AccuracyExperimentCase`, `AccuracyObservation`, and `run_variant_suite`."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Experimental region: concrete selection strategies under `crates/rust_robotics_localization/src/experiments/ukf_ckf_accuracy/`."
    )
    .unwrap();
    md
}

fn render_decisions_md(
    spec: &ProblemDocSpec<'_>,
    reports: &[AccuracyVariantReport],
    problem: &[AccuracyExperimentCase],
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
        "- Keep `full-bucket` as the convergence reference. It is the slowest strategy, but it uses all `10/10` seeds per bucket and therefore defines the current reference labels."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep `{}` as the best current exploratory proxy. It reduces evaluation cost from `{:.2} ms` to `{:.2} ms`, while keeping hard agreement with `full-bucket` at `{}` and near-tie-aware agreement at `{}` on the current localization problem.",
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
            "- `{}` is also the cheapest smoke-test strategy on this problem. That is useful evidence, but it should stay a disposable strategy until another independent localization preset says the same thing.",
            cheapest.descriptor.id
        )
        .unwrap();
    } else {
        writeln!(
            &mut md,
            "- Keep `{}` as the cheapest smoke-test strategy. It runs in `{:.2} ms`; its hard agreement with `full-bucket` is `{}`, but its near-tie-aware agreement is `{}`, so it should not drive filter-selection decisions.",
            cheapest.descriptor.id,
            cheapest.evaluation_runtime_ms,
            format_option(cheapest.agreement_vs_reference),
            format_option(near_tie_aware_agreement(cheapest, full))
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- Keep the interface stable and the implementations disposable. New variants must plug into the same input set (`{}` families) and emit the same `AccuracyObservation` records.",
        problem.len()
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Treat buckets inside the near-tie band (`|ratio - 1.0| < {:.3}`) as unstable evidence. They should stay visible in docs, but they should not be over-read as decisive filter wins.",
        NEAR_TIE_RATIO_BAND
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject one-shot convergence. Cheaper variants remain useful for search, but the reference still needs full-bucket aggregation when the localization story is ambiguous."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject the idea that pathfinding-specific evidence was enough. This problem keeps the same selection pattern but changes both domain and metric."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Compare this preset against the cross-problem localization summary before widening the shared experiment core beyond its current contract."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- If the gap between `{}` and `full-bucket` remains large, add another disposable variant instead of widening the stable surface.",
        best_proxy.descriptor.id
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep `ukf_ckf_accuracy` experimental until a second non-pathfinding comparison uses the same contract with minimal edits."
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
        "Compare the same five localization aggregation variants across multiple independent presets to see whether any concrete strategy deserves promotion beyond `experiments/`, while keeping the shared contract fixed."
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
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Current Read").unwrap();
    writeln!(&mut md).unwrap();
    for run in runs {
        let best = best_proxy_report(run.reports);
        let cheapest = cheapest_non_reference_report(run.reports);
        writeln!(
            &mut md,
            "- `{}`: best proxy is `{}`; cheapest smoke test is `{}`.",
            run.spec.problem_label, best.descriptor.id, cheapest.descriptor.id
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
        "- Scope: localization aggregation variants across `{}` independent presets.",
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
        "- Promote only the comparison contract. The current best proxy rotates across presets: `{}`.",
        runs.iter()
            .map(|run| {
                format!(
                    "{} -> {}",
                    run.spec.problem_label,
                    best_proxy_report(run.reports).descriptor.id
                )
            })
            .collect::<Vec<_>>()
            .join("; ")
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep concrete variants disposable inside `experiments/ukf_ckf_accuracy/`. None has won every current localization preset."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject choosing one permanent exploratory proxy for localization. The current best-proxy winners split across `{}` concrete variants: {}.",
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
        "- Reject widening the stable surface to encode dropout, bias, or horizon-specific logic. Those belong to disposable presets, not to the core contract."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    if runs.len() >= 10 {
        writeln!(
            &mut md,
            "- Stop preset growth for localization here. The current stop condition is met: `10` independent presets still do not yield a single winning concrete variant."
        )
        .unwrap();
        writeln!(
            &mut md,
            "- The next move is not another preset. Keep the shared comparison contract fixed while planning, localization, and control reuse it without promoting any concrete strategy."
        )
        .unwrap();
    } else {
        writeln!(
            &mut md,
            "- Add another localization preset around actuator saturation or correlated multi-sensor dropout."
        )
        .unwrap();
    }
    writeln!(
        &mut md,
        "- Shared core is now limited to `SamplingPlan + descriptor + reference annotation`; do not promote any concrete variant."
    )
    .unwrap();
    md
}

fn observation_cell(
    reports: &[AccuracyVariantReport],
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
    let ratio = observation.ukf_over_ckf();
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
    candidate: &AccuracyVariantReport,
    reference: &AccuracyVariantReport,
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
        if outcome_label(observation.ukf_over_ckf())
            == outcome_label(reference_observation.ukf_over_ckf())
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

fn best_proxy_report(reports: &[AccuracyVariantReport]) -> &AccuracyVariantReport {
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

fn cheapest_non_reference_report(reports: &[AccuracyVariantReport]) -> &AccuracyVariantReport {
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

fn ordered_keys(reports: &[AccuracyVariantReport]) -> Vec<(&'static str, u32)> {
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

fn reference_report(reports: &[AccuracyVariantReport]) -> &AccuracyVariantReport {
    reports
        .iter()
        .find(|report| report.descriptor.id == "full-bucket")
        .expect("full-bucket report should exist")
}

fn outcome_label(ratio: f64) -> &'static str {
    if is_near_tie(ratio) {
        "near-tie"
    } else if ratio > 1.0 {
        "CKF"
    } else {
        "UKF"
    }
}

fn is_near_tie(ratio: f64) -> bool {
    (ratio - 1.0).abs() < NEAR_TIE_RATIO_BAND
}

fn coverage_cell(observation: &AccuracyObservation) -> String {
    format!(
        "{}/{}",
        observation.selected_slots.len(),
        observation.total_scenarios
    )
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
