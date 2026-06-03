//! Render hierarchical MAPF replanning as a static SVG.

use std::collections::BTreeSet;
use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    HierarchicalMapfAgent2D, HierarchicalMapfConfig2D, HierarchicalMapfPlan2D,
    HierarchicalMapfPlanner2D, HierarchicalMapfRegion2D, StlCbsPath,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/hierarchical-mapf-replanning.svg";
const WIDTH: i32 = 12;
const HEIGHT: i32 = 8;
const REGION_WIDTH: i32 = 4;
const REGION_HEIGHT: i32 = 4;
const LEFT: f64 = 58.0;
const TOP: f64 = 96.0;
const CELL: f64 = 42.0;

fn agents() -> Vec<HierarchicalMapfAgent2D> {
    vec![
        HierarchicalMapfAgent2D::new(0, (0, 3), (11, 3)),
        HierarchicalMapfAgent2D::new(1, (11, 3), (0, 3)),
        HierarchicalMapfAgent2D::new(2, (5, 0), (5, 7)),
        HierarchicalMapfAgent2D::new(3, (0, 7), (3, 7)),
    ]
}

fn planner() -> RoboticsResult<HierarchicalMapfPlanner2D> {
    HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
        max_time: 24,
        max_cbs_nodes: 4_096,
        ..HierarchicalMapfConfig2D::new(WIDTH, HEIGHT, REGION_WIDTH, REGION_HEIGHT)
    })
}

fn color(agent_id: usize) -> &'static str {
    match agent_id {
        0 => "#2563eb",
        1 => "#dc2626",
        2 => "#059669",
        3 => "#7c3aed",
        _ => "#0f766e",
    }
}

fn sx(x: i32) -> f64 {
    LEFT + x as f64 * CELL + CELL / 2.0
}

fn sy(y: i32) -> f64 {
    TOP + (HEIGHT - 1 - y) as f64 * CELL + CELL / 2.0
}

fn region_rect(region: HierarchicalMapfRegion2D) -> (f64, f64, f64, f64) {
    let x0 = region.rx * REGION_WIDTH;
    let x1 = (x0 + REGION_WIDTH).min(WIDTH);
    let y0 = region.ry * REGION_HEIGHT;
    let y1 = (y0 + REGION_HEIGHT).min(HEIGHT);
    (
        LEFT + x0 as f64 * CELL,
        TOP + (HEIGHT - y1) as f64 * CELL,
        (x1 - x0) as f64 * CELL,
        (y1 - y0) as f64 * CELL,
    )
}

fn trajectory_points(path: &StlCbsPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| format!("{:.1},{:.1}", sx(waypoint.x), sy(waypoint.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_grid(svg: &mut String) {
    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            writeln!(
                svg,
                r##"<rect x="{:.1}" y="{:.1}" width="{CELL:.1}" height="{CELL:.1}" fill="#ffffff" stroke="#dbe3ee" stroke-width="1"/>"##,
                LEFT + x as f64 * CELL,
                TOP + (HEIGHT - 1 - y) as f64 * CELL
            )
            .unwrap();
        }
    }
    for x in (REGION_WIDTH..WIDTH).step_by(REGION_WIDTH as usize) {
        let xx = LEFT + x as f64 * CELL;
        writeln!(
            svg,
            r##"<line x1="{xx:.1}" y1="{TOP:.1}" x2="{xx:.1}" y2="{:.1}" stroke="#64748b" stroke-width="2.3" stroke-dasharray="7 7" opacity="0.75"/>"##,
            TOP + HEIGHT as f64 * CELL
        )
        .unwrap();
    }
    for y in (REGION_HEIGHT..HEIGHT).step_by(REGION_HEIGHT as usize) {
        let yy = TOP + (HEIGHT - y) as f64 * CELL;
        writeln!(
            svg,
            r##"<line x1="{LEFT:.1}" y1="{yy:.1}" x2="{:.1}" y2="{yy:.1}" stroke="#64748b" stroke-width="2.3" stroke-dasharray="7 7" opacity="0.75"/>"##,
            LEFT + WIDTH as f64 * CELL
        )
        .unwrap();
    }
}

fn render_conflict_regions(svg: &mut String, plan: &HierarchicalMapfPlan2D) {
    let regions = plan
        .region_conflicts
        .iter()
        .map(|conflict| conflict.region)
        .collect::<BTreeSet<_>>();
    for region in regions.into_iter().take(6) {
        let (x, y, w, h) = region_rect(region);
        writeln!(
            svg,
            r##"<rect x="{x:.1}" y="{y:.1}" width="{w:.1}" height="{h:.1}" fill="#fed7aa" stroke="#f97316" stroke-width="3" opacity="0.34"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#c2410c" font-family="Consolas, Menlo, monospace" font-size="12" font-weight="700">R{},{} conflict</text>"##,
            x + w / 2.0,
            y + 20.0,
            region.rx,
            region.ry
        )
        .unwrap();
    }
}

fn render_path(svg: &mut String, path: &StlCbsPath, primary: bool) {
    let points = trajectory_points(path);
    let stroke = color(path.agent_id);
    let width = if primary { 5.2 } else { 3.5 };
    let opacity = if primary { 0.96 } else { 0.28 };
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="{:.2}"/>"##,
        width + 5.0,
        opacity
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{stroke}" stroke-width="{width:.1}" stroke-linecap="round" stroke-linejoin="round" stroke-dasharray="{}" opacity="{opacity:.2}"/>"##,
        if primary { "none" } else { "9 8" }
    )
    .unwrap();
    for waypoint in &path.waypoints {
        if waypoint.t == 0 || waypoint.t == path.arrival_time() || waypoint.t % 4 == 0 {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{stroke}" stroke="#ffffff" stroke-width="1.4" opacity="{opacity:.2}"/>"##,
                sx(waypoint.x),
                sy(waypoint.y),
                if primary { 5.0 } else { 3.2 }
            )
            .unwrap();
        }
    }
}

fn render_start_goal(svg: &mut String, agent: HierarchicalMapfAgent2D) {
    let stroke = color(agent.id);
    writeln!(
        svg,
        r##"<rect x="{:.1}" y="{:.1}" width="17" height="17" rx="4" fill="{stroke}" stroke="#ffffff" stroke-width="2"/>"##,
        sx(agent.start.0) - 8.5,
        sy(agent.start.1) - 8.5
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{:.1}" cy="{:.1}" r="10" fill="none" stroke="{stroke}" stroke-width="3"/>"##,
        sx(agent.goal.0),
        sy(agent.goal.1)
    )
    .unwrap();
}

fn render_stats(svg: &mut String, plan: &HierarchicalMapfPlan2D, x: f64, y: f64) {
    writeln!(
        svg,
        r##"<text x="{x:.1}" y="{y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="700">Hierarchical MAPF</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("independent conflicts {}", plan.independent_cell_conflicts),
        format!("region triggers       {}", plan.region_conflicts.len()),
        format!("replanned groups      {}", plan.replanned_groups.len()),
        format!("final conflicts       {}", plan.final_cell_conflicts),
        format!("total cost            {}", plan.total_cost),
        format!("fallback full plan    {}", plan.fallback_full_replan),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            y + 32.0 + index as f64 * 23.0
        )
        .unwrap();
    }
    for (index, group) in plan.replanned_groups.iter().take(3).enumerate() {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#475569" font-family="Consolas, Menlo, monospace" font-size="12">CBS group {:?}: cost {}, resolved {}</text>"##,
            y + 196.0 + index as f64 * 22.0,
            group.agent_ids,
            group.cbs_total_cost,
            group.cbs_conflicts_resolved
        )
        .unwrap();
    }
}

fn render_legend(svg: &mut String) {
    let x = 58.0;
    let y = 472.0;
    writeln!(
        svg,
        r##"<line x1="{x:.1}" y1="{y:.1}" x2="{:.1}" y2="{y:.1}" stroke="#2563eb" stroke-width="4" stroke-dasharray="9 8" opacity="0.45"/>"##,
        x + 36.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">independent shortest paths</text>"##,
        x + 48.0,
        y + 4.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="248" y1="{y:.1}" x2="284" y2="{y:.1}" stroke="#2563eb" stroke-width="5" stroke-linecap="round"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="296" y="{:.1}" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">CBS-replanned paths</text>"##,
        y + 4.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="438" y="{:.1}" width="22" height="16" fill="#fed7aa" stroke="#f97316" opacity="0.55"/>"##,
        y - 11.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="470" y="{:.1}" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">coarse region trigger</text>"##,
        y + 4.0
    )
    .unwrap();
}

fn render_svg(
    agents: &[HierarchicalMapfAgent2D],
    plan: &HierarchicalMapfPlan2D,
) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="980" height="520" viewBox="0 0 980 520" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Hierarchical MAPF replanning</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Coarse region conflicts trigger CBS replanning for affected agents while unrelated paths remain independent.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="980" height="520" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Hierarchical MAPF replanning</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Region conflicts select a smaller CBS group; the final grid plan has zero cell conflicts.</text>"##
    )
    .unwrap();
    render_grid(&mut svg);
    render_conflict_regions(&mut svg, plan);
    for path in &plan.independent_paths {
        render_path(&mut svg, path, false);
    }
    for path in &plan.paths {
        render_path(&mut svg, path, true);
    }
    for &agent in agents {
        render_start_goal(&mut svg, agent);
    }
    render_stats(&mut svg, plan, 600.0, 126.0);
    render_legend(&mut svg);
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let agents = agents();
    let planner = planner()?;
    let plan = planner.plan(&agents)?;
    let svg = render_svg(&agents, &plan)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} independent_conflicts={} final_conflicts={} region_conflicts={} groups={}",
        plan.independent_cell_conflicts,
        plan.final_cell_conflicts,
        plan.region_conflicts.len(),
        plan.replanned_groups.len()
    );
    Ok(())
}
