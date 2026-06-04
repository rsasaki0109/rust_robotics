//! Render rigid-body MIP-style planning as a static SVG.

use std::f64::consts::TAU;
use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    RigidBodyConvexObstacle2D, RigidBodyMipConfig2D, RigidBodyMipPlan2D, RigidBodyMipPlanner2D,
    RigidBodyPoint2D, RigidBodyPose2D,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/rigid-body-mip-planning.svg";
const LEFT: f64 = 58.0;
const TOP: f64 = 88.0;
const PLOT_WIDTH: f64 = 620.0;
const PLOT_HEIGHT: f64 = 372.0;
const X_MIN: f64 = 0.0;
const X_MAX: f64 = 10.0;
const Y_MIN: f64 = 0.0;
const Y_MAX: f64 = 6.0;

fn scenario() -> RoboticsResult<(RigidBodyMipPlanner2D, RigidBodyPose2D, RigidBodyPose2D)> {
    let obstacles = vec![
        RigidBodyConvexObstacle2D::convex_polygon(vec![
            RigidBodyPoint2D::new(3.0, 0.0),
            RigidBodyPoint2D::new(7.0, 0.0),
            RigidBodyPoint2D::new(7.0, 2.6),
            RigidBodyPoint2D::new(3.4, 2.6),
            RigidBodyPoint2D::new(3.0, 2.2),
        ])?,
        RigidBodyConvexObstacle2D::convex_polygon(vec![
            RigidBodyPoint2D::new(3.0, 3.8),
            RigidBodyPoint2D::new(3.4, 3.4),
            RigidBodyPoint2D::new(7.0, 3.4),
            RigidBodyPoint2D::new(7.0, 6.0),
            RigidBodyPoint2D::new(3.0, 6.0),
        ])?,
    ];
    let planner = RigidBodyMipPlanner2D::new(RigidBodyMipConfig2D {
        obstacles,
        max_expansions: 50_000,
        ..RigidBodyMipConfig2D::new(X_MIN, X_MAX, Y_MIN, Y_MAX)
    })?;
    let start = RigidBodyPose2D::new(1.0, 3.0, TAU / 4.0);
    let goal = RigidBodyPose2D::new(9.0, 3.0, 0.0);
    Ok((planner, start, goal))
}

fn sx(x: f64) -> f64 {
    LEFT + (x - X_MIN) / (X_MAX - X_MIN) * PLOT_WIDTH
}

fn sy(y: f64) -> f64 {
    TOP + (Y_MAX - y) / (Y_MAX - Y_MIN) * PLOT_HEIGHT
}

fn points(points: &[RigidBodyPoint2D]) -> String {
    points
        .iter()
        .map(|point| format!("{:.1},{:.1}", sx(point.x), sy(point.y)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_grid(svg: &mut String) {
    writeln!(
        svg,
        r##"<rect x="{LEFT:.1}" y="{TOP:.1}" width="{PLOT_WIDTH:.1}" height="{PLOT_HEIGHT:.1}" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.5"/>"##
    )
    .unwrap();
    for x in 0..=10 {
        let xx = sx(x as f64);
        writeln!(
            svg,
            r##"<line x1="{xx:.1}" y1="{TOP:.1}" x2="{xx:.1}" y2="{:.1}" stroke="#e2e8f0" stroke-width="1"/>"##,
            TOP + PLOT_HEIGHT
        )
        .unwrap();
    }
    for y in 0..=6 {
        let yy = sy(y as f64);
        writeln!(
            svg,
            r##"<line x1="{LEFT:.1}" y1="{yy:.1}" x2="{:.1}" y2="{yy:.1}" stroke="#e2e8f0" stroke-width="1"/>"##,
            LEFT + PLOT_WIDTH
        )
        .unwrap();
    }
}

fn render_obstacles(svg: &mut String, planner: &RigidBodyMipPlanner2D) {
    for obstacle in &planner.config().obstacles {
        writeln!(
            svg,
            r##"<polygon points="{}" fill="#334155" stroke="#0f172a" stroke-width="2" opacity="0.92"/>"##,
            points(&obstacle.vertices)
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="#f8fafc" stroke="#f97316" stroke-width="2" stroke-dasharray="6 5" opacity="0.85"/>"##,
        sx(3.0),
        sy(3.4),
        sx(7.0) - sx(3.0),
        sy(2.6) - sy(3.4)
    )
    .unwrap();
}

fn render_path(svg: &mut String, plan: &RigidBodyMipPlan2D) {
    let polyline = plan
        .poses
        .iter()
        .map(|pose| format!("{:.1},{:.1}", sx(pose.x), sy(pose.y)))
        .collect::<Vec<_>>()
        .join(" ");
    writeln!(
        svg,
        r##"<polyline points="{polyline}" fill="none" stroke="#ffffff" stroke-width="10" stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{polyline}" fill="none" stroke="#2563eb" stroke-width="5.5" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();
}

fn render_robot(
    svg: &mut String,
    planner: &RigidBodyMipPlanner2D,
    pose: RigidBodyPose2D,
    fill: &str,
    opacity: f64,
    label: Option<&str>,
) {
    let vertices = planner.robot_vertices(pose);
    writeln!(
        svg,
        r##"<polygon points="{}" fill="{fill}" stroke="#ffffff" stroke-width="1.8" opacity="{opacity:.2}"/>"##,
        points(&vertices)
    )
    .unwrap();
    let dx = pose.theta.cos() * 0.45;
    let dy = pose.theta.sin() * 0.45;
    writeln!(
        svg,
        r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#111827" stroke-width="2" stroke-linecap="round" opacity="{opacity:.2}"/>"##,
        sx(pose.x),
        sy(pose.y),
        sx(pose.x + dx),
        sy(pose.y + dy)
    )
    .unwrap();
    if let Some(label) = label {
        writeln!(
            svg,
            r##"<text x="{:.1}" y="{:.1}" fill="#111827" text-anchor="middle" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11" font-weight="800">{label}</text>"##,
            sx(pose.x),
            sy(pose.y) - 20.0
        )
        .unwrap();
    }
}

fn render_robots(svg: &mut String, planner: &RigidBodyMipPlanner2D, plan: &RigidBodyMipPlan2D) {
    for (index, pose) in plan.poses.iter().enumerate() {
        if index == 0 {
            render_robot(svg, planner, *pose, "#22c55e", 0.82, Some("start"));
        } else if index + 1 == plan.poses.len() {
            render_robot(svg, planner, *pose, "#dc2626", 0.82, Some("goal"));
        } else if index % 3 == 0 {
            render_robot(svg, planner, *pose, "#2563eb", 0.26, None);
        }
    }
}

fn render_stats(svg: &mut String, plan: &RigidBodyMipPlan2D, x: f64, y: f64) {
    writeln!(
        svg,
        r##"<text x="{x:.1}" y="{y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="700">MIP-style certificate plan</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("poses              {}", plan.poses.len()),
        format!("total cost         {}", plan.total_cost),
        format!("expanded states    {}", plan.expanded_states),
        format!("binary choices     {}", plan.binary_separation_choices),
        format!(
            "segment binaries    {}",
            plan.segment_binary_separation_choices
        ),
        format!("min margin         {:.3}", plan.min_separation_margin),
        format!(
            "segment margin     {:.3}",
            plan.min_segment_separation_margin
        ),
        "active disjunction  one half-space per obstacle".to_string(),
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
    planner: &RigidBodyMipPlanner2D,
    plan: &RigidBodyMipPlan2D,
) -> RoboticsResult<String> {
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="980" height="520" viewBox="0 0 980 520" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Rigid-body MIP-style planning</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">A rectangular rigid body passes a narrow slot using half-space separation certificates.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="980" height="520" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="42" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Rigid-body MIP-style planning</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="58" y="68" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">A discretized SE(2) search keeps MILP-like binary half-space certificates for every obstacle at every pose.</text>"##
    )
    .unwrap();
    render_grid(&mut svg);
    render_obstacles(&mut svg, planner);
    render_path(&mut svg, plan);
    render_robots(&mut svg, planner, plan);
    render_stats(&mut svg, plan, 710.0, 126.0);
    writeln!(
        svg,
        r##"<text x="58" y="492" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Dark blocks are convex obstacles; the orange slot is only feasible when the rectangle is aligned horizontally.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let (planner, start, goal) = scenario()?;
    let plan = planner.plan(start, goal, true)?;
    let svg = render_svg(&planner, &plan)?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!(
        "wrote {OUTPUT} poses={} cost={} expanded={} binaries={} segment_binaries={} min_margin={:.3} min_segment_margin={:.3}",
        plan.poses.len(),
        plan.total_cost,
        plan.expanded_states,
        plan.binary_separation_choices,
        plan.segment_binary_separation_choices,
        plan.min_separation_margin,
        plan.min_segment_separation_margin
    );
    Ok(())
}
