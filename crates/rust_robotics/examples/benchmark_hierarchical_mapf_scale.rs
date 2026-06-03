//! Scale benchmark for hierarchical MAPF replanning.
//!
//! Generates many isolated two-agent corridor swaps. Each pair needs CBS, but
//! the region hierarchy should keep the repair groups local as the agent count
//! grows to hundreds.

use std::fmt::Write;
use std::path::Path;
use std::time::Instant;

use rust_robotics::planning::{
    HierarchicalMapfAgent2D, HierarchicalMapfConfig2D, HierarchicalMapfPlan2D,
    HierarchicalMapfPlanner2D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/hierarchical-mapf-scale.csv";
const SVG_OUTPUT: &str = "docs/assets/hierarchical-mapf-scale.svg";
const WIDTH: i32 = 5;
const BAND_HEIGHT: i32 = 4;
const REGION_WIDTH: i32 = 5;
const REGION_HEIGHT: i32 = 4;
const MAX_TIME: u64 = 12;
const AGENT_COUNTS: [usize; 3] = [50, 100, 200];

#[derive(Debug, Clone, PartialEq)]
struct ScaleSample {
    agents: usize,
    pairs: usize,
    width: i32,
    height: i32,
    elapsed_ms: f64,
    independent_conflicts: usize,
    final_conflicts: usize,
    region_conflicts: usize,
    replanned_groups: usize,
    max_group_size: usize,
    total_cost: u64,
    fallback_full_replan: bool,
}

fn obstacle_map(width: i32, height: i32, pairs: usize) -> Vec<Vec<bool>> {
    let mut obstacles = vec![vec![false; height as usize]; width as usize];
    for pair in 0..pairs {
        let separator_y = pair as i32 * BAND_HEIGHT + 3;
        if separator_y < height {
            for x in 0..width {
                obstacles[x as usize][separator_y as usize] = true;
            }
        }
    }
    obstacles
}

fn scenario(
    agent_count: usize,
) -> RoboticsResult<(HierarchicalMapfPlanner2D, Vec<HierarchicalMapfAgent2D>)> {
    if agent_count == 0 || agent_count % 2 != 0 {
        return Err(rust_robotics::core::RoboticsError::InvalidParameter(
            "hierarchical MAPF scale benchmark requires a positive even agent count".to_string(),
        ));
    }
    let pairs = agent_count / 2;
    let height = pairs as i32 * BAND_HEIGHT;
    let planner = HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
        obstacle_map: obstacle_map(WIDTH, height, pairs),
        max_time: MAX_TIME,
        max_cbs_nodes: 4_096,
        ..HierarchicalMapfConfig2D::new(WIDTH, height, REGION_WIDTH, REGION_HEIGHT)
    })?;
    let mut agents = Vec::with_capacity(agent_count);
    for pair in 0..pairs {
        let y = pair as i32 * BAND_HEIGHT + 1;
        let left_id = pair * 2;
        agents.push(HierarchicalMapfAgent2D::new(
            left_id,
            (0, y),
            (WIDTH - 1, y),
        ));
        agents.push(HierarchicalMapfAgent2D::new(
            left_id + 1,
            (WIDTH - 1, y),
            (0, y),
        ));
    }
    Ok((planner, agents))
}

fn summarize(
    agent_count: usize,
    planner: &HierarchicalMapfPlanner2D,
    plan: &HierarchicalMapfPlan2D,
    elapsed_ms: f64,
) -> ScaleSample {
    ScaleSample {
        agents: agent_count,
        pairs: agent_count / 2,
        width: planner.config().width,
        height: planner.config().height,
        elapsed_ms,
        independent_conflicts: plan.independent_cell_conflicts,
        final_conflicts: plan.final_cell_conflicts,
        region_conflicts: plan.region_conflicts.len(),
        replanned_groups: plan.replanned_groups.len(),
        max_group_size: plan
            .replanned_groups
            .iter()
            .map(|group| group.agent_ids.len())
            .max()
            .unwrap_or(0),
        total_cost: plan.total_cost,
        fallback_full_replan: plan.fallback_full_replan,
    }
}

fn run_sample(agent_count: usize) -> RoboticsResult<ScaleSample> {
    let (planner, agents) = scenario(agent_count)?;
    let start = Instant::now();
    let plan = planner.plan(&agents)?;
    let elapsed_ms = start.elapsed().as_secs_f64() * 1_000.0;
    Ok(summarize(agent_count, &planner, &plan, elapsed_ms))
}

fn render_csv(samples: &[ScaleSample]) -> String {
    let mut csv = String::from(
        "agents,pairs,width,height,elapsed_ms,independent_conflicts,final_conflicts,region_conflicts,replanned_groups,max_group_size,total_cost,fallback_full_replan\n",
    );
    for sample in samples {
        writeln!(
            csv,
            "{},{},{},{},{:.3},{},{},{},{},{},{},{}",
            sample.agents,
            sample.pairs,
            sample.width,
            sample.height,
            sample.elapsed_ms,
            sample.independent_conflicts,
            sample.final_conflicts,
            sample.region_conflicts,
            sample.replanned_groups,
            sample.max_group_size,
            sample.total_cost,
            sample.fallback_full_replan
        )
        .unwrap();
    }
    csv
}

fn chart_x(index: usize) -> f64 {
    112.0 + index as f64 * 176.0
}

fn chart_y(value: f64, max_value: f64) -> f64 {
    342.0 - value / max_value.max(1.0) * 214.0
}

fn render_svg(samples: &[ScaleSample]) -> String {
    let max_ms = samples
        .iter()
        .map(|sample| sample.elapsed_ms)
        .fold(0.0, f64::max)
        .max(1.0);
    let max_conflicts = samples
        .iter()
        .map(|sample| sample.region_conflicts)
        .max()
        .unwrap_or(1) as f64;

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="760" height="470" viewBox="0 0 760 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Hierarchical MAPF scale benchmark</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Runtime and region-trigger counts for hierarchical MAPF local CBS repair at 50, 100, and 200 agents.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="760" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Hierarchical MAPF scale benchmark</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Many corridor swaps stay decomposed into two-agent CBS repairs.</text>"##
    )
    .unwrap();

    for i in 0..=4 {
        let y = 342.0 - i as f64 * 53.5;
        writeln!(
            svg,
            r##"<line x1="84" y1="{y:.1}" x2="616" y2="{y:.1}" stroke="#e2e8f0" stroke-width="1"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="74" y="{:.1}" text-anchor="end" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11">{:.0}</text>"##,
            y + 4.0,
            max_ms * i as f64 / 4.0
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<line x1="84" y1="342" x2="616" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="84" y1="128" x2="84" y2="342" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="350" y="392" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">agent count</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="26" y="236" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700" transform="rotate(-90 26 236)">planning time ms</text>"##
    )
    .unwrap();

    let mut conflict_line = String::new();
    for (index, sample) in samples.iter().enumerate() {
        let x = chart_x(index);
        let bar_height = sample.elapsed_ms / max_ms * 214.0;
        let y = 342.0 - bar_height;
        writeln!(
            svg,
            r##"<rect x="{:.1}" y="{y:.1}" width="58" height="{bar_height:.1}" rx="4" fill="#2563eb" opacity="0.86"/>"##,
            x - 29.0
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" text-anchor="middle" fill="#0f172a" font-family="Consolas, Menlo, monospace" font-size="12">{:.1}ms</text>"##,
            y - 9.0,
            sample.elapsed_ms
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="365" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12" font-weight="700">{} agents</text>"##,
            sample.agents
        )
        .unwrap();
        let conflict_y = chart_y(sample.region_conflicts as f64, max_conflicts);
        if !conflict_line.is_empty() {
            conflict_line.push(' ');
        }
        write!(conflict_line, "{x:.1},{conflict_y:.1}").unwrap();
        writeln!(
            svg,
            r##"<circle cx="{x:.1}" cy="{conflict_y:.1}" r="6.5" fill="#f97316" stroke="#ffffff" stroke-width="2"/>"##
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<polyline points="{conflict_line}" fill="none" stroke="#f97316" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="638" y="126" width="12" height="12" rx="2" fill="#2563eb"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="658" y="136" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">repair time</text>"##
    )
    .unwrap();
    writeln!(svg, r##"<circle cx="644" cy="160" r="6" fill="#f97316"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="658" y="164" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">region triggers</text>"##
    )
    .unwrap();

    for (index, sample) in samples.iter().enumerate() {
        writeln!(
            svg,
            r##"<text x="638" y="{:.1}" fill="#475569" font-family="Consolas, Menlo, monospace" font-size="12">n={}: groups {}, max {}, final {}</text>"##,
            212.0 + index as f64 * 24.0,
            sample.agents,
            sample.replanned_groups,
            sample.max_group_size,
            sample.final_conflicts
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<text x="54" y="430" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Scenario: isolated 3-row corridors; every swap pair is locally repaired while global fallback remains false.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    svg
}

fn write_outputs(samples: &[ScaleSample]) -> RoboticsResult<()> {
    for path in [CSV_OUTPUT, SVG_OUTPUT] {
        if let Some(parent) = Path::new(path).parent() {
            std::fs::create_dir_all(parent)?;
        }
    }
    std::fs::write(CSV_OUTPUT, render_csv(samples))?;
    std::fs::write(SVG_OUTPUT, render_svg(samples))?;
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let mut samples = Vec::new();
    for agent_count in AGENT_COUNTS {
        let sample = run_sample(agent_count)?;
        println!(
            "hierarchical-mapf-scale: agents={} ms={:.2} independent_conflicts={} final_conflicts={} region_conflicts={} groups={} max_group={} fallback={}",
            sample.agents,
            sample.elapsed_ms,
            sample.independent_conflicts,
            sample.final_conflicts,
            sample.region_conflicts,
            sample.replanned_groups,
            sample.max_group_size,
            sample.fallback_full_replan
        );
        samples.push(sample);
    }
    write_outputs(&samples)?;
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}
