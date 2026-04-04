use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root should exist")
}

fn docs_file(name: &str) -> String {
    fs::read_to_string(repo_root().join("docs").join(name))
        .unwrap_or_else(|_| panic!("docs/{name} should be readable"))
}

fn parse_ratio_pair(line: &str) -> Option<(usize, usize)> {
    let start = line.find('`')? + 1;
    let end = line[start..].find('`').map(|offset| start + offset)?;
    let pair = &line[start..end];
    let mut parts = pair.split('/');
    let lhs = parts.next()?.parse::<usize>().ok()?;
    let rhs = parts.next()?.parse::<usize>().ok()?;
    Some((lhs, rhs))
}

fn parse_strongest_winner_share(line: &str) -> Option<(usize, usize)> {
    let open = line.rfind('(')? + 1;
    let close = line[open..].find(')').map(|offset| open + offset)?;
    let pair = &line[open..close];
    let mut parts = pair.split('/');
    let lhs = parts.next()?.parse::<usize>().ok()?;
    let rhs = parts.next()?.parse::<usize>().ok()?;
    Some((lhs, rhs))
}

fn count_nonzero_proxy_winners(summary: &str) -> usize {
    let mut in_table = false;
    let mut count = 0usize;

    for line in summary.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("| Variant | Best-proxy wins | Package/Preset |") {
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
            .collect::<Vec<_>>();
        if columns.len() < 2 {
            continue;
        }
        let wins = columns[1]
            .parse::<usize>()
            .expect("proxy wins should be numeric");
        if wins > 0 {
            count += 1;
        }
    }

    count
}

fn workspace_stop_condition_holds(experiments: &str, decisions: &str) -> bool {
    let strongest_line = experiments
        .lines()
        .find(|line| line.contains("strongest concrete winner share"));
    let cheapest_line = experiments
        .lines()
        .find(|line| line.contains("cheapest smoke test disagrees"));
    let imperfect_line = experiments
        .lines()
        .find(|line| line.contains("best proxy is imperfect"));

    let Some((strongest_wins, total_presets)) =
        strongest_line.and_then(parse_strongest_winner_share)
    else {
        return false;
    };
    let Some((cheapest_mismatches, total_presets_from_mismatch)) =
        cheapest_line.and_then(parse_ratio_pair)
    else {
        return false;
    };
    let Some((imperfect_best, total_presets_from_imperfect)) =
        imperfect_line.and_then(parse_ratio_pair)
    else {
        return false;
    };
    let active_winner_count = count_nonzero_proxy_winners(experiments);

    total_presets == total_presets_from_mismatch
        && total_presets == total_presets_from_imperfect
        && strongest_wins < total_presets
        && cheapest_mismatches > 0
        && imperfect_best > 0
        && active_winner_count > 1
        && decisions.contains("Reject choosing one permanent exploratory proxy for the workspace.")
        && decisions.contains("machine-checked by the `rust_robotics_core` workspace-summary guard")
}

fn cross_package_refs_registered(interfaces: &str) -> bool {
    [
        "## Cross-Package Summaries",
        "docs/experiments_planning_summary.md",
        "docs/decisions_planning_summary.md",
        "docs/experiments_localization_summary.md",
        "docs/decisions_localization_summary.md",
        "docs/experiments_control_summary.md",
        "docs/decisions_control_summary.md",
        "docs/experiments_mapping_summary.md",
        "docs/decisions_mapping_summary.md",
        "docs/experiments_workspace_summary.md",
        "docs/decisions_workspace_summary.md",
    ]
    .into_iter()
    .all(|expected| interfaces.contains(expected))
}

fn synthetic_experiments_summary(
    strongest_winner_share: &str,
    cheapest_mismatch_ratio: &str,
    imperfect_ratio: &str,
    proxy_rows: &[(&str, usize)],
) -> String {
    let mut md = String::from("# Experiments\n\n## Stop-Condition Evidence\n\n");
    md.push_str(&format!(
        "- strongest concrete winner share: `percentile-bucket` ({strongest_winner_share})\n"
    ));
    md.push_str(&format!(
        "- presets where cheapest smoke test disagrees with best proxy: `{cheapest_mismatch_ratio}`\n"
    ));
    md.push_str(&format!(
        "- presets where even the best proxy is imperfect against full-bucket: `{imperfect_ratio}`\n\n"
    ));
    md.push_str(
        "## Proxy Rotation\n\n| Variant | Best-proxy wins | Package/Preset |\n| --- | ---: | --- |\n",
    );
    for (variant, wins) in proxy_rows {
        md.push_str(&format!("| `{variant}` | {wins} | synthetic/{variant} |\n"));
    }
    md
}

fn synthetic_decisions(include_guard_language: bool) -> String {
    let mut md = String::from(
        "# Decisions\n\n## Adopt\n\n- Keep `full-bucket` as the convergence reference.\n",
    );
    if include_guard_language {
        md.push_str(
            "- Keep the architectural stop condition machine-checked by the `rust_robotics_core` workspace-summary guard so winner convergence or docs drift becomes visible immediately.\n",
        );
    }
    md.push_str(
        "\n## Reject\n\n- Reject choosing one permanent exploratory proxy for the workspace.\n",
    );
    md
}

#[test]
fn workspace_stop_condition_guard_holds() {
    let experiments = docs_file("experiments_workspace_summary.md");
    let decisions = docs_file("decisions_workspace_summary.md");
    assert!(workspace_stop_condition_holds(&experiments, &decisions));
}

#[test]
fn workspace_summary_cross_package_references_stay_registered() {
    let interfaces = docs_file("interfaces.md");
    assert!(cross_package_refs_registered(&interfaces));
}

#[test]
fn workspace_stop_condition_rejects_single_winner_and_universal_smoke_test() {
    let experiments =
        synthetic_experiments_summary("21/21", "0/21", "0/21", &[("percentile-bucket", 21)]);
    let decisions = synthetic_decisions(true);
    assert!(!workspace_stop_condition_holds(&experiments, &decisions));
}

#[test]
fn workspace_stop_condition_rejects_docs_without_guard_language() {
    let experiments = synthetic_experiments_summary(
        "7/21",
        "13/21",
        "3/21",
        &[("first-scenario", 6), ("percentile-bucket", 7)],
    );
    let decisions = synthetic_decisions(false);
    assert!(!workspace_stop_condition_holds(&experiments, &decisions));
}

#[test]
fn workspace_cross_package_registry_rejects_missing_workspace_summary_refs() {
    let interfaces = "\
# Interfaces

## Cross-Package Summaries

- planning summary: [`docs/experiments_planning_summary.md`] + [`docs/decisions_planning_summary.md`]
- localization summary: [`docs/experiments_localization_summary.md`] + [`docs/decisions_localization_summary.md`]
- control summary: [`docs/experiments_control_summary.md`] + [`docs/decisions_control_summary.md`]
- mapping summary: [`docs/experiments_mapping_summary.md`] + [`docs/decisions_mapping_summary.md`]
";
    assert!(!cross_package_refs_registered(interfaces));
}
