//! Render kinodynamic STL-CBS planning as a static SVG.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    first_conflict, KinodynamicHeading2D, KinodynamicStlCbsAgent2D, KinodynamicStlCbsConfig2D,
    KinodynamicStlCbsPath2D, KinodynamicStlCbsPlan2D, KinodynamicStlCbsPlanner2D,
    KinodynamicTimedPose2D, StlCbsConflictKind, StlTimedCell,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/kinodynamic-stl-cbs.svg";
const WIDTH: i32 = 7;
const HEIGHT: i32 = 5;
const LEFT: f64 = 58.0;
const TOP: f64 = 96.0;
const CELL: f64 = 58.0;

fn agents() -> Vec<KinodynamicStlCbsAgent2D> {
    vec![
        KinodynamicStlCbsAgent2D::new(0, (0, 2), KinodynamicHeading2D::East, (6, 2)),
        KinodynamicStlCbsAgent2D::new(1, (6, 2), KinodynamicHeading2D::West, (0, 2)),
    ]
}

fn planner() -> RoboticsResult<KinodynamicStlCbsPlanner2D> {
    KinodynamicStlCbsPlanner2D::new(KinodynamicStlCbsConfig2D {
        max_time: 32,
        max_cbs_nodes: 4_096,
        move_duration: 2,
        turn_duration: 1,
        ..KinodynamicStlCbsConfig2D::new(WIDTH, HEIGHT)
    })
}

fn color(agent_id: usize) -> &'static str {
    match agent_id {
        0 => "#2563eb",
        1 => "#dc2626",
        _ => "#059669",
    }
}

fn sx(x: i32) -> f64 {
    LEFT + x as f64 * CELL + CELL / 2.0
}

fn sy(y: i32) -> f64 {
    TOP + (HEIGHT - 1 - y) as f64 * CELL + CELL / 2.0
}

fn trajectory_points(path: &KinodynamicStlCbsPath2D) -> String {
    path.poses
        .iter()
        .map(|pose| format!("{:.1},{:.1}", sx(pose.x), sy(pose.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn conflict_cell(kind: StlCbsConflictKind) -> Option<StlTimedCell> {
    match kind {
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

fn render_heading(svg: &mut String, pose: KinodynamicTimedPose2D, color: &str, opacity: f64) {
    let (dx, dy) = pose.heading.delta();
    let cx = sx(pose.x);
    let cy = sy(pose.y);
    let ex = cx + dx as f64 * 15.0;
    let ey = cy - dy as f64 * 15.0;
    writeln!(
        svg,
        r##"<line x1="{cx:.1}" y1="{cy:.1}" x2="{ex:.1}" y2="{ey:.1}" stroke="{color}" stroke-width="2.5" stroke-linecap="round" opacity="{opacity:.2}"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{ex:.1}" cy="{ey:.1}" r="3.2" fill="{color}" opacity="{opacity:.2}"/>"##
    )
    .unwrap();
}

fn render_path(svg: &mut String, path: &KinodynamicStlCbsPath2D, primary: bool) {
    let points = trajectory_points(path);
    let stroke = color(path.agent_id);
    let width = if primary { 5.2 } else { 3.4 };
    let opacity = if primary { 0.96 } else { 0.28 };
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="{opacity:.2}"/>"##,
        width + 5.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{stroke}" stroke-width="{width:.1}" stroke-linecap="round" stroke-linejoin="round" stroke-dasharray="{}" opacity="{opacity:.2}"/>"##,
        if primary { "none" } else { "9 8" }
    )
    .unwrap();
    for pose in &path.poses {
        if pose.t == 0 || pose.t == path.arrival_time() || pose.t % 3 == 0 {
            writeln!(
                svg,
                r##"<circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{stroke}" stroke="#ffffff" stroke-width="1.4" opacity="{opacity:.2}"/>"##,
                sx(pose.x),
                sy(pose.y),
                if primary { 5.0 } else { 3.4 }
            )
            .unwrap();
            render_heading(svg, *pose, stroke, opacity);
        }
    }
}

fn render_start_goal(svg: &mut String, agent: KinodynamicStlCbsAgent2D) {
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
        r##"<circle cx="{:.1}" cy="{:.1}" r="11" fill="none" stroke="{stroke}" stroke-width="3"/>"##,
        sx(agent.goal.0),
        sy(agent.goal.1)
    )
    .unwrap();
}

fn render_stats(svg: &mut String, plan: &KinodynamicStlCbsPlan2D, x: f64, y: f64) {
    writeln!(
        svg,
        r##"<text x="{x:.1}" y="{y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="700">Kinodynamic STL-CBS</text>"##
    )
    .unwrap();
    for (index, line) in [
        "move duration       2 ticks".to_string(),
        "turn duration       1 tick".to_string(),
        format!("resolved conflicts  {}", plan.conflicts_resolved),
        format!("expanded nodes      {}", plan.high_level_nodes_expanded),
        format!("total time cost     {}", plan.total_cost),
        format!(
            "integer robust      {:.2}",
            plan.min_pairwise_separation_robustness
        ),
        format!(
            "continuous robust   {:.2}",
            plan.min_continuous_pairwise_separation_robustness
        ),
        format!(
            "continuous conflict {}",
            plan.first_continuous_conflict.is_some()
        ),
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
}

fn render_svg(
    agents: &[KinodynamicStlCbsAgent2D],
    independent: &[KinodynamicStlCbsPath2D],
    plan: &KinodynamicStlCbsPlan2D,
) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="920" height="470" viewBox="0 0 920 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(svg, r##"<title id="title">Kinodynamic STL-CBS</title>"##).unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Heading-aware multi-robot CBS with time-consuming forward and turn primitives.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="920" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Kinodynamic STL-CBS</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Dashed paths are independent oriented plans; solid paths are CBS repairs with heading and velocity limits.</text>"##
    )
    .unwrap();
    render_grid(&mut svg);
    for path in independent {
        render_path(&mut svg, path, false);
    }
    let independent_cells = independent
        .iter()
        .map(KinodynamicStlCbsPath2D::cell_path)
        .collect::<Vec<_>>();
    if let Some(conflict) =
        first_conflict(&independent_cells, 32).and_then(|c| conflict_cell(c.kind))
    {
        writeln!(
            svg,
            r##"<circle cx="{:.1}" cy="{:.1}" r="20" fill="none" stroke="#f97316" stroke-width="4" stroke-dasharray="5 4"/>"##,
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
    render_stats(&mut svg, plan, 540.0, 122.0);
    writeln!(
        svg,
        r##"<text x="58" y="426" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Squares are starts, rings are goals, and small direction ticks show robot heading at sampled times.</text>"##
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
        "wrote {OUTPUT} resolved={} expanded={} total_cost={} separation={:.2} continuous_separation={:.2}",
        plan.conflicts_resolved,
        plan.high_level_nodes_expanded,
        plan.total_cost,
        plan.min_pairwise_separation_robustness,
        plan.min_continuous_pairwise_separation_robustness
    );
    Ok(())
}
