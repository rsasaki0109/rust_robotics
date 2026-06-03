//! Render the traversal-risk graph demo as a static SVG.
//!
//! This keeps the visualization dependency-free so it can run in CI and update
//! the docs gallery asset with a single command.

use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    TerrainRiskCell, TraversalRiskGraphConfig, TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

const WIDTH: i32 = 9;
const HEIGHT: i32 = 5;
const CELL: f64 = 58.0;
const LEFT: f64 = 54.0;
const TOP: f64 = 94.0;
const OUTPUT: &str = "docs/assets/traversal-risk-graph-demo.svg";

fn terrain() -> Vec<Vec<TerrainRiskCell>> {
    let mut cells = vec![vec![TerrainRiskCell::free(); HEIGHT as usize]; WIDTH as usize];

    for x in 3..=5 {
        cells[x][2] = TerrainRiskCell::with_risk(5.0, 0.5, 0.0);
    }
    cells[4][1] = TerrainRiskCell::with_risk(2.5, 3.5, 0.0);
    cells[4][3] = TerrainRiskCell::with_risk(2.5, 3.5, 0.0);
    cells[2][0] = TerrainRiskCell::blocked();
    cells[6][4] = TerrainRiskCell::blocked();

    cells
}

fn planner_with_weight(risk_weight: f64) -> RoboticsResult<TraversalRiskGraphPlanner> {
    let config = TraversalRiskGraphConfig {
        risk_weight,
        traversability_weight: 1.0,
        stability_weight: 1.0,
        exposure_weight: 0.5,
        ..TraversalRiskGraphConfig::new(WIDTH, HEIGHT, terrain())
    };
    TraversalRiskGraphPlanner::new(config)
}

fn plan_with_weight(risk_weight: f64) -> RoboticsResult<TraversalRiskPath> {
    planner_with_weight(risk_weight)?.plan(0, 2, WIDTH - 1, 2)
}

fn cell_center(x: i32, y: i32) -> (f64, f64) {
    (
        LEFT + x as f64 * CELL + CELL / 2.0,
        TOP + y as f64 * CELL + CELL / 2.0,
    )
}

fn risk_color(risk: f64, blocked: bool) -> &'static str {
    if blocked {
        return "#202124";
    }

    match risk {
        r if r >= 5.5 => "#c2410c",
        r if r >= 4.0 => "#f97316",
        r if r >= 2.0 => "#facc15",
        r if r > 0.0 => "#fde68a",
        _ => "#ecfdf5",
    }
}

fn path_points(path: &TraversalRiskPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| {
            let (cx, cy) = cell_center(waypoint.x, waypoint.y);
            format!("{cx:.1},{cy:.1}")
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_path(svg: &mut String, path: &TraversalRiskPath, color: &str, width: f64) {
    let points = path_points(path);
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="#ffffff" stroke-width="{:.1}" stroke-linecap="round" stroke-linejoin="round" opacity="0.86"/>"##,
        width + 5.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width:.1}" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();

    for waypoint in &path.waypoints {
        let (cx, cy) = cell_center(waypoint.x, waypoint.y);
        writeln!(
            svg,
            r##"<circle cx="{cx:.1}" cy="{cy:.1}" r="6.2" fill="{color}" stroke="#ffffff" stroke-width="2"/>"##
        )
        .unwrap();
    }
}

fn render_svg() -> RoboticsResult<String> {
    let shortest = plan_with_weight(0.0)?;
    let risk_aware = plan_with_weight(2.0)?;
    let display_planner = planner_with_weight(1.0)?;
    let cells = terrain();

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="760" height="470" viewBox="0 0 760 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Traversal-risk graph planner demo</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Terrain risk heatmap comparing a shortest path through hazardous cells with a longer safer detour.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="760" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Traversal-risk graph</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Distance-only A* crosses the risk band; weighted terrain risk selects the safer detour.</text>"##
    )
    .unwrap();

    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            let cell = cells[x as usize][y as usize];
            let risk = display_planner.cell_risk(x, y)?;
            let fill = risk_color(risk, cell.blocked);
            let rect_x = LEFT + x as f64 * CELL;
            let rect_y = TOP + y as f64 * CELL;
            writeln!(
                svg,
                r##"<rect x="{rect_x:.1}" y="{rect_y:.1}" width="{CELL:.1}" height="{CELL:.1}" rx="6" fill="{fill}" stroke="#ffffff" stroke-width="3"/>"##
            )
            .unwrap();

            if cell.blocked {
                writeln!(
                    svg,
                    r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="800">X</text>"##,
                    rect_x + CELL / 2.0,
                    rect_y + CELL / 2.0
                )
                .unwrap();
            } else if risk > 0.0 {
                let text_color = if risk >= 4.0 { "#ffffff" } else { "#7c2d12" };
                writeln!(
                    svg,
                    r##"<text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="central" fill="{text_color}" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">{risk:.1}</text>"##,
                    rect_x + CELL / 2.0,
                    rect_y + CELL / 2.0
                )
                .unwrap();
            }
        }
    }

    render_path(&mut svg, &shortest, "#2563eb", 7.0);
    render_path(&mut svg, &risk_aware, "#059669", 7.0);

    let (sx, sy) = cell_center(0, 2);
    let (gx, gy) = cell_center(WIDTH - 1, 2);
    writeln!(
        svg,
        r##"<circle cx="{sx:.1}" cy="{sy:.1}" r="13" fill="#111827" stroke="#ffffff" stroke-width="3"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="{gx:.1}" cy="{gy:.1}" r="13" fill="#111827" stroke="#ffffff" stroke-width="3"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{sx:.1}" y="{sy:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11" font-weight="800">S</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{gx:.1}" y="{gy:.1}" text-anchor="middle" dominant-baseline="central" fill="#ffffff" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11" font-weight="800">G</text>"##
    )
    .unwrap();

    let legend_x = 580.0;
    let legend_y = 108.0;
    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="{legend_y:.1}" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">Routes</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="{legend_x:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#2563eb" stroke-width="7" stroke-linecap="round"/>"##,
        legend_y + 24.0,
        legend_x + 46.0,
        legend_y + 24.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">distance-only</text>"##,
        legend_x + 58.0,
        legend_y + 29.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="{legend_x:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#059669" stroke-width="7" stroke-linecap="round"/>"##,
        legend_y + 55.0,
        legend_x + 46.0,
        legend_y + 55.0
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{:.1}" y="{:.1}" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">risk-aware</text>"##,
        legend_x + 58.0,
        legend_y + 60.0
    )
    .unwrap();

    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="218" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">Cost split</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="244" fill="#2563eb" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">distance-only</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="265" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">distance {short_dist:.1} / risk {short_risk:.1}</text>"##,
        short_dist = shortest.distance_cost,
        short_risk = shortest.risk_cost
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="299" fill="#059669" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">risk-aware</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="{legend_x:.1}" y="320" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">distance {safe_dist:.1} / risk {safe_risk:.1}</text>"##,
        safe_dist = risk_aware.distance_cost,
        safe_risk = risk_aware.risk_cost
    )
    .unwrap();

    writeln!(
        svg,
        r##"<text x="54" y="420" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Cell labels show weighted terrain risk. Dark cells are blocked.</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();

    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let svg = render_svg()?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!("wrote {OUTPUT}");
    Ok(())
}
