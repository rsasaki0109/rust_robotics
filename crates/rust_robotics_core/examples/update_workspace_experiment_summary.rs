use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

struct PackageSummarySource {
    package: &'static str,
    experiments_summary: &'static str,
    decisions_summary: &'static str,
}

#[derive(Debug, Clone)]
struct WorkspacePresetRow {
    package: String,
    preset: String,
    best_proxy: String,
    best_eval_ms: f64,
    hard: String,
    near_tie_aware: String,
    cheapest_smoke_test: String,
    cheapest_eval_ms: f64,
    cheapest_hard: String,
    cheapest_near_tie_aware: String,
    full_eval_ms: f64,
}

fn main() {
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root should exist");
    let docs_dir = repo_root.join("docs");
    let sources = [
        PackageSummarySource {
            package: "planning",
            experiments_summary: "experiments_planning_summary.md",
            decisions_summary: "decisions_planning_summary.md",
        },
        PackageSummarySource {
            package: "localization",
            experiments_summary: "experiments_localization_summary.md",
            decisions_summary: "decisions_localization_summary.md",
        },
        PackageSummarySource {
            package: "control",
            experiments_summary: "experiments_control_summary.md",
            decisions_summary: "decisions_control_summary.md",
        },
        PackageSummarySource {
            package: "mapping",
            experiments_summary: "experiments_mapping_summary.md",
            decisions_summary: "decisions_mapping_summary.md",
        },
    ];

    let mut rows = Vec::new();
    for source in &sources {
        let content = fs::read_to_string(docs_dir.join(source.experiments_summary))
            .unwrap_or_else(|_| panic!("{} should be readable", source.experiments_summary));
        rows.extend(parse_preset_rows(source.package, &content));
    }
    rows.sort_by(|left, right| {
        left.package
            .cmp(&right.package)
            .then(left.preset.cmp(&right.preset))
    });

    fs::write(
        docs_dir.join("experiments_workspace_summary.md"),
        render_workspace_experiments_md(&sources, &rows),
    )
    .expect("experiments_workspace_summary.md should be writable");
    fs::write(
        docs_dir.join("decisions_workspace_summary.md"),
        render_workspace_decisions_md(&sources, &rows),
    )
    .expect("decisions_workspace_summary.md should be writable");

    println!(
        "updated {}",
        docs_dir.join("experiments_workspace_summary.md").display()
    );
    println!(
        "updated {}",
        docs_dir.join("decisions_workspace_summary.md").display()
    );
}

fn parse_preset_rows(package: &str, content: &str) -> Vec<WorkspacePresetRow> {
    let mut in_table = false;
    let mut rows = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("| Preset | Best proxy | Best eval ms |") {
            in_table = true;
            continue;
        }
        if !in_table {
            continue;
        }
        if trimmed.is_empty() || !trimmed.starts_with('|') {
            break;
        }
        if trimmed.contains("| --- ") {
            continue;
        }

        let columns = trimmed
            .split('|')
            .map(str::trim)
            .filter(|column| !column.is_empty())
            .map(strip_backticks)
            .collect::<Vec<_>>();
        assert_eq!(
            columns.len(),
            10,
            "unexpected workspace preset row shape: {trimmed}"
        );

        rows.push(WorkspacePresetRow {
            package: package.to_string(),
            preset: columns[0].clone(),
            best_proxy: columns[1].clone(),
            best_eval_ms: parse_number(&columns[2]),
            hard: columns[3].clone(),
            near_tie_aware: columns[4].clone(),
            cheapest_smoke_test: columns[5].clone(),
            cheapest_eval_ms: parse_number(&columns[6]),
            cheapest_hard: columns[7].clone(),
            cheapest_near_tie_aware: columns[8].clone(),
            full_eval_ms: parse_number(&columns[9]),
        });
    }

    rows
}

fn render_workspace_experiments_md(
    sources: &[PackageSummarySource],
    rows: &[WorkspacePresetRow],
) -> String {
    let mut md = String::new();
    writeln!(&mut md, "# Experiments").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Scope").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "Aggregate the package-level preset summaries from planning, localization, control, and mapping to see whether any concrete exploratory proxy deserves promotion beyond `experiments/` anywhere in the workspace."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Package Comparison\n\n| Package | Presets | Winning variants | Mean best speedup | Cheapest mismatch | Imperfect best-proxy | Summary docs |\n| --- | ---: | --- | ---: | ---: | ---: | --- |"
    )
    .unwrap();
    for source in sources {
        let package_rows = rows
            .iter()
            .filter(|row| row.package == source.package)
            .collect::<Vec<_>>();
        let winning_variants = package_rows
            .iter()
            .map(|row| row.best_proxy.clone())
            .collect::<BTreeSet<_>>();
        let mean_best_speedup = if package_rows.is_empty() {
            0.0
        } else {
            package_rows
                .iter()
                .map(|row| row.full_eval_ms / row.best_eval_ms.max(1e-9))
                .sum::<f64>()
                / package_rows.len() as f64
        };
        let cheapest_mismatch = package_rows
            .iter()
            .filter(|row| row.cheapest_smoke_test != row.best_proxy)
            .count();
        let imperfect_best_proxy = package_rows
            .iter()
            .filter(|row| !is_perfect_score(&row.hard) || !is_perfect_score(&row.near_tie_aware))
            .count();
        writeln!(
            &mut md,
            "| `{}` | {} | {} | {:.2}x | {} | {} | [`{}`] + [`{}`] |",
            source.package,
            package_rows.len(),
            if winning_variants.is_empty() {
                "-".to_string()
            } else {
                winning_variants
                    .iter()
                    .map(|variant| format!("`{variant}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            },
            mean_best_speedup,
            cheapest_mismatch,
            imperfect_best_proxy,
            source.experiments_summary,
            source.decisions_summary,
        )
        .unwrap();
    }
    writeln!(&mut md).unwrap();
    let total_presets = rows.len();
    let cheapest_mismatches = rows
        .iter()
        .filter(|row| row.cheapest_smoke_test != row.best_proxy)
        .count();
    let imperfect_best = rows
        .iter()
        .filter(|row| !is_perfect_score(&row.hard) || !is_perfect_score(&row.near_tie_aware))
        .count();
    let strongest_variant = rows
        .iter()
        .map(|row| row.best_proxy.as_str())
        .fold(BTreeMap::<&str, usize>::new(), |mut counts, variant| {
            *counts.entry(variant).or_default() += 1;
            counts
        })
        .into_iter()
        .max_by_key(|(_, count)| *count)
        .map(|(variant, count)| format!("`{variant}` ({count}/{total_presets})"))
        .unwrap_or_else(|| "-".to_string());
    writeln!(&mut md, "## Stop-Condition Evidence").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- strongest concrete winner share: {}",
        strongest_variant
    )
    .unwrap();
    writeln!(
        &mut md,
        "- presets where cheapest smoke test disagrees with best proxy: `{}/{}`",
        cheapest_mismatches, total_presets
    )
    .unwrap();
    writeln!(
        &mut md,
        "- presets where even the best proxy is imperfect against full-bucket: `{}/{}`",
        imperfect_best, total_presets
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Workspace Preset Comparison\n\n| Package | Preset | Best proxy | Best eval ms | Hard | Near-tie-aware | Cheapest smoke test | Cheapest eval ms | Cheapest hard | Cheapest near-tie-aware | Full eval ms |\n| --- | --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |"
    )
    .unwrap();
    for row in rows {
        writeln!(
            &mut md,
            "| `{}` | `{}` | `{}` | {:.2} | {} | {} | `{}` | {:.2} | {} | {} | {:.2} |",
            row.package,
            row.preset,
            row.best_proxy,
            row.best_eval_ms,
            row.hard,
            row.near_tie_aware,
            row.cheapest_smoke_test,
            row.cheapest_eval_ms,
            row.cheapest_hard,
            row.cheapest_near_tie_aware,
            row.full_eval_ms,
        )
        .unwrap();
    }
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "## Proxy Rotation\n\n| Variant | Best-proxy wins | Package/Preset |\n| --- | ---: | --- |"
    )
    .unwrap();
    for variant in [
        "first-scenario",
        "sampled-bucket",
        "percentile-bucket",
        "variance-triggered",
    ] {
        let wins = rows
            .iter()
            .filter(|row| row.best_proxy == variant)
            .map(|row| format!("{}/{}", row.package, row.preset))
            .collect::<Vec<_>>();
        writeln!(
            &mut md,
            "| `{}` | {} | {} |",
            variant,
            wins.len(),
            if wins.is_empty() {
                "-".to_string()
            } else {
                wins.join(", ")
            }
        )
        .unwrap();
    }
    md
}

fn render_workspace_decisions_md(
    sources: &[PackageSummarySource],
    rows: &[WorkspacePresetRow],
) -> String {
    let mut variant_counts = BTreeMap::<String, usize>::new();
    let mut package_winners = BTreeMap::<String, BTreeSet<String>>::new();
    for row in rows {
        *variant_counts.entry(row.best_proxy.clone()).or_default() += 1;
        package_winners
            .entry(row.package.clone())
            .or_default()
            .insert(row.best_proxy.clone());
    }

    let unique_winners = variant_counts.keys().cloned().collect::<Vec<_>>();
    let total_presets = rows.len().max(1);
    let strongest_variant = variant_counts
        .iter()
        .max_by_key(|(_, count)| **count)
        .map(|(variant, count)| format!("`{variant}` ({count}/{total_presets})"))
        .unwrap_or_else(|| "-".to_string());
    let cheapest_mismatches = rows
        .iter()
        .filter(|row| row.cheapest_smoke_test != row.best_proxy)
        .count();
    let imperfect_best = rows
        .iter()
        .filter(|row| !is_perfect_score(&row.hard) || !is_perfect_score(&row.near_tie_aware))
        .count();

    let mut md = String::new();
    writeln!(&mut md, "# Decisions").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Context").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Scope: `{}` packages and `{}` validated presets aggregated into one workspace summary.",
        sources.len(),
        rows.len()
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Adopt").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Keep `full-bucket` as the convergence reference on every package and preset."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Promote only the shared comparison contract in `rust_robotics_core::experiments`. Workspace-wide best-proxy rotation now spans `{}` concrete variants: {}.",
        unique_winners.len(),
        unique_winners
            .iter()
            .map(|variant| format!("`{variant}`"))
            .collect::<Vec<_>>()
            .join(", ")
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep concrete variants disposable. Current package winner sets are {}.",
        sources
            .iter()
            .map(|source| {
                let winners = package_winners
                    .get(source.package)
                    .cloned()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|winner| format!("`{winner}`"))
                    .collect::<Vec<_>>()
                    .join(", ");
                format!(
                    "{} -> {}",
                    source.package,
                    if winners.is_empty() {
                        "-".to_string()
                    } else {
                        winners
                    }
                )
            })
            .collect::<Vec<_>>()
            .join("; ")
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep the architectural stop condition machine-checked by the `rust_robotics_core` workspace-summary guard so winner convergence or docs drift becomes visible immediately."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Reject").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Reject declaring a single concrete winner from frequency alone. The strongest current winner is only {}.",
        strongest_variant
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject treating the cheapest smoke test as a universal proxy. It disagrees with the best current proxy on `{}/{}` presets.",
        cheapest_mismatches,
        total_presets
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject choosing one permanent exploratory proxy for the workspace. No concrete variant wins every package, and no package family is sufficient to justify core promotion by itself."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Reject widening the stable surface to encode package-specific regimes such as MovingAI buckets, localization corruption patterns, controller mismatch families, or mapping corruption modes."
    )
    .unwrap();
    writeln!(&mut md).unwrap();
    writeln!(&mut md, "## Next").unwrap();
    writeln!(&mut md).unwrap();
    writeln!(
        &mut md,
        "- Use this workspace summary as the current architectural stop condition: the stable core remains `SamplingPlan + descriptor + reference annotation + report metrics`, and even the best current proxy is still imperfect on `{}/{}` presets.",
        imperfect_best,
        total_presets
    )
    .unwrap();
    writeln!(
        &mut md,
        "- If stronger external evidence is needed, measure downstream effects such as implementation speed, review clarity, or defect detection instead of adding more internal presets by default."
    )
    .unwrap();
    writeln!(
        &mut md,
        "- Keep package-specific exploratory variants under `experiments/` and only extend `rust_robotics_core::experiments` when a new cross-package need survives comparison."
    )
    .unwrap();
    md
}

fn strip_backticks(value: &str) -> String {
    value.trim_matches('`').to_string()
}

fn parse_number(value: &str) -> f64 {
    value
        .parse::<f64>()
        .unwrap_or_else(|_| panic!("expected number, got `{value}`"))
}

fn is_perfect_score(value: &str) -> bool {
    value == "1.000"
}
