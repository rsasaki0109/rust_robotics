//! Render STL-CBS multi-robot planning as a static SVG.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    first_conflict, stl_eventually_reach_robustness, stl_pairwise_separation_robustness,
    StlCbsAgent, StlCbsConfig, StlCbsConflictKind, StlCbsPath, StlCbsPlan, StlCbsPlanner,
    StlRectangle2D, StlTimeInterval, StlTimedCell,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/stl-cbs-multi-robot.svg";
const MAX_TIME: u64 = 18;
const LEFT: f64 = 58.0;
const TOP: f64 = 88.0;
const CELL: f64 = 50.0;
const WIDTH: i32 = 9;
const HEIGHT: i32 = 5;

fn agents() -> Vec<StlCbsAgent> {
    vec![
        StlCbsAgent::new(0, (0, 2), (8, 2)),
        StlCbsAgent::new(1, (8, 2), (0, 2)),
        StlCbsAgent::new(2, (4, 0), (4, 4)),
    ]
}

fn planner() -> RoboticsResult<StlCbsPlanner> {
    StlCbsPlanner::new(StlCbsConfig {
        max_time: MAX_TIME,
        max_cbs_nodes: 2_048,
        ..StlCbsConfig::new(WIDTH, HEIGHT)
    })
}

fn goal_region(goal: (i32, i32)) -> RoboticsResult<StlRectangle2D> {
    StlRectangle2D::new(
        goal.0 as f64 - 0.25,
        goal.0 as f64 + 0.25,
        goal.1 as f64 - 0.25,
        goal.1 as f64 + 0.25,
    )
}

fn color(agent_id: usize) -> &'static str {
    match agent_id {
        0 => "#2563eb",
        1 => "#dc2626",
        2 => "#059669",
        _ => "#7c3aed",
    }
}

fn sx(x: i32) -> f64 {
    LEFT + x as f64 * CELL + CELL / 2.0
}

fn sy(y: i32) -> f64 {
    TOP + (HEIGHT - 1 - y) as f64 * CELL + CELL / 2.0
}

fn trajectory_points(path: &StlCbsPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| format!("{:.1},{:.1}", sx(waypoint.x), sy(waypoint.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn conflict_cell(conflict: StlCbsConflictKind) -> Option<StlTimedCell> {
    match conflict {
        StlCbsConflictKind::Vertex { x, y, t } => Some(StlTimedCell::new(x, y, t)),
        StlCbsConflictKind::Edge { ax1, ay1, t, .. } => Some(StlTimedCell::new(ax1, ay1, t)),
    }
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
}

fn render_path(svg: &mut String, path: &StlCbsPath, primary: bool) {
    let points = trajectory_points(path);
    let stroke = color(path.agent_id);
    let width = if primary { 5.5 } else { 3.5 };
    let opacity = if primary { 0.96 } else { 0.34 };
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="{opacity:.2}"/>"##,
        width + 5.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{stroke}" stroke-width="{width:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="{opacity:.2}" stroke-dasharray="{}"/>"##,
        if primary { "none" } else { "8 8" }
    )
    .unwrap();
    for waypoint in &path.waypoints {
        if waypoint.t == 0 || waypoint.t == path.arrival_time() || waypoint.t % 3 == 0 {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{stroke}" stroke="#ffffff" stroke-width="1.4" opacity="{opacity:.2}"/>"##,
                sx(waypoint.x),
                sy(waypoint.y),
                if primary { 5.0 } else { 3.5 }
            )
            .unwrap();
        }
    }
}

fn render_start_goal(svg: &mut String, agent: StlCbsAgent) {
    let stroke = color(agent.id);
    writeln!(
        svg,
        r##"<rect x="{:.1}" y="{:.1}" width="18" height="18" rx="4" fill="{stroke}" stroke="#ffffff" stroke-width="2"/>"##,
        sx(agent.start.0) - 9.0,
        sy(agent.start.1) - 9.0
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

fn render_stats(svg: &mut String, plan: &StlCbsPlan, min_goal_robustness: f64, x: f64, y: f64) {
    writeln!(
        svg,
        r##"<text x="{x:.1}" y="{y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="700">STL-CBS plan</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("cost          {}", plan.total_cost),
        format!("resolved      {}", plan.conflicts_resolved),
        format!("expanded      {}", plan.high_level_nodes_expanded),
        format!(
            "separation    {:.2}",
            plan.min_pairwise_separation_robustness
        ),
        format!("goal robust   {:.2}", min_goal_robustness),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="{:.1}" fill="#334155" font-family="Consolas, Menlo, monospace" font-size="13">{line}</text>"##,
            y + 32.0 + index as f64 * 24.0
        )
        .unwrap();
    }
}

fn render_svg(
    agents: &[StlCbsAgent],
    independent: &[StlCbsPath],
    plan: &StlCbsPlan,
) -> RoboticsResult<String> {
    let interval = StlTimeInterval::new(0, MAX_TIME)?;
    let min_goal_robustness = plan
        .paths
        .iter()
        .map(|path| {
            let agent = agents
                .iter()
                .find(|agent| agent.id == path.agent_id)
                .expect("path references a known agent");
            stl_eventually_reach_robustness(path, goal_region(agent.goal).unwrap(), interval)
                .unwrap()
        })
        .fold(f64::INFINITY, f64::min);
    let pairwise = stl_pairwise_separation_robustness(&plan.paths, 1.0, interval)?;
    let mut plan = plan.clone();
    plan.min_pairwise_separation_robustness = pairwise;
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="900" height="470" viewBox="0 0 900 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">STL-CBS multi-robot planning</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Independent grid paths collide; CBS resolves conflicts and STL robustness remains positive.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="900" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">STL-CBS multi-robot planning</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Dashed paths are independent shortest plans; solid paths are conflict-free CBS outputs checked with STL robustness.</text>"##
    )
    .unwrap();
    render_grid(&mut svg);
    for path in independent {
        render_path(&mut svg, path, false);
    }
    if let Some(conflict) =
        first_conflict(independent, MAX_TIME).and_then(|c| conflict_cell(c.kind))
    {
        writeln!(
            svg,
            r##"<circle cx="{:.1}" cy="{:.1}" r="17" fill="none" stroke="#f97316" stroke-width="4" stroke-dasharray="5 4"/>"##,
            sx(conflict.x),
            sy(conflict.y)
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="#f97316" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="10" font-weight="800">t{}</text>"##,
            sx(conflict.x),
            sy(conflict.y),
            conflict.t
        )
        .unwrap();
    }
    for path in &plan.paths {
        render_path(&mut svg, path, true);
    }
    for &agent in agents {
        render_start_goal(&mut svg, agent);
    }
    render_stats(&mut svg, &plan, min_goal_robustness, 560.0, 120.0);
    writeln!(
        svg,
        r##"<text x="58" y="398" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Squares are starts, rings are goals, and the orange ring marks the first independent-path conflict.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let agents = agents();
    let planner = planner()?;
    let independent = planner.plan_independent(&agents)?;
    let plan = planner.plan(&agents)?;
    let svg = render_svg(&agents, &independent, &plan)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} conflicts_resolved={} separation={:.2}",
        plan.conflicts_resolved, plan.min_pairwise_separation_robustness
    );
    Ok(())
}
