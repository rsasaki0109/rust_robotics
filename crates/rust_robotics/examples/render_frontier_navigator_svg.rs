//! Render Long Range Navigator-lite frontier navigation as a static SVG.
//!
//! A robot with a limited, occlusion-aware sensor cannot see the distant goal.
//! It repeatedly picks the frontier (known-free cell bordering unknown space)
//! with the best affordance — goal progress, known-free travel cost, line of
//! sight, and bordered unknown space — and hands off to a local Dijkstra planner
//! to drive there, revealing more of the world until a path to the goal appears.
//! This draws the final known map, the traversed path, and the frontier choices.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::planning::{
    simulate_frontier_navigation, FrontierNavConfig, FrontierNavWorld, Knowledge,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/frontier-navigator.svg";
const CELL: f64 = 18.0;
const LEFT: f64 = 24.0;
const TOP: f64 = 92.0;

fn world() -> RoboticsResult<FrontierNavWorld> {
    // Two offset walls force the robot to route around occlusions in an S.
    let mut world = FrontierNavWorld::new(34, 20, (1, 10), (32, 10))?;
    world.fill_rect(11, 12, 0, 13); // wall from the bottom, gap at the top
    world.fill_rect(21, 22, 6, 19); // wall from the top, gap at the bottom
    Ok(world)
}

fn cell_fill(state: Knowledge) -> &'static str {
    match state {
        Knowledge::Unknown => "#d9d9de",
        Knowledge::Free => "#ffffff",
        Knowledge::Occupied => "#3a3a3c",
    }
}

fn render_svg(
    world: &FrontierNavWorld,
    report: &rust_robotics::planning::FrontierNavReport,
) -> String {
    let w = world.width;
    let h = world.height;
    let width = LEFT * 2.0 + w as f64 * CELL;
    let height = TOP + h as f64 * CELL + 40.0;

    let mut svg = String::new();
    let _ = writeln!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width:.0}\" height=\"{height:.0}\" viewBox=\"0 0 {width:.0} {height:.0}\">"
    );
    let _ = writeln!(
        svg,
        "<rect width=\"{width:.0}\" height=\"{height:.0}\" fill=\"#fbfbfd\"/>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"36\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Long Range Navigator-lite: affordance-scored frontier navigation</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"58\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">white = known free, gray = unknown, dark = obstacle; orange dots = chosen frontiers; blue = traversed path</text>"
    );

    // Known-map cells.
    for x in 0..w {
        for y in 0..h {
            let state = report.known[x as usize][y as usize];
            let px = LEFT + x as f64 * CELL;
            let py = TOP + y as f64 * CELL;
            let _ = writeln!(
                svg,
                "<rect x=\"{px:.1}\" y=\"{py:.1}\" width=\"{CELL}\" height=\"{CELL}\" fill=\"{}\" stroke=\"#eef0f2\" stroke-width=\"0.5\"/>",
                cell_fill(state)
            );
        }
    }

    let cx = |x: i32| LEFT + x as f64 * CELL + CELL / 2.0;
    let cy = |y: i32| TOP + y as f64 * CELL + CELL / 2.0;

    // Traversed path.
    let mut points = String::new();
    for &(x, y) in &report.path {
        let _ = write!(points, "{:.1},{:.1} ", cx(x), cy(y));
    }
    let _ = writeln!(
        svg,
        "<polyline points=\"{}\" fill=\"none\" stroke=\"#0a84ff\" stroke-width=\"2.5\" stroke-linejoin=\"round\"/>",
        points.trim()
    );

    // Frontier choices.
    for choice in &report.selected_frontiers {
        let color = if choice.line_of_sight {
            "#ff9f0a"
        } else {
            "#ff3b30"
        };
        let _ = writeln!(
            svg,
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"4\" fill=\"{color}\"/>",
            cx(choice.frontier.0),
            cy(choice.frontier.1)
        );
    }

    // Start and goal.
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"6\" fill=\"#1d1d1f\"/>",
        cx(world.start.0),
        cy(world.start.1)
    );
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"6\" fill=\"#34c759\"/>",
        cx(world.goal.0),
        cy(world.goal.1)
    );

    let footer = TOP + h as f64 * CELL + 26.0;
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"{footer:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\">reached goal: {} · {} frontier selections · path length {:.1} · {} cells revealed</text>",
        report.reached_goal, report.frontier_selections, report.path_length, report.known_free_cells
    );

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let world = world()?;
    let config = FrontierNavConfig::default();
    let report = simulate_frontier_navigation(&world, &config)?;

    println!("Long Range Navigator-lite frontier navigation");
    println!(
        "  reached_goal={} selections={} iterations={} path_length={:.1} known_free={}",
        report.reached_goal,
        report.frontier_selections,
        report.iterations,
        report.path_length,
        report.known_free_cells
    );

    if let Some(parent) = Path::new(OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(OUTPUT, render_svg(&world, &report)).ok();
    println!("wrote {OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reaches_goal_and_is_deterministic() {
        let world = world().unwrap();
        let config = FrontierNavConfig::default();
        let first = simulate_frontier_navigation(&world, &config).unwrap();
        let second = simulate_frontier_navigation(&world, &config).unwrap();
        assert!(first.reached_goal);
        assert_eq!(first.path, second.path);
        assert!(first.frontier_selections >= 1);
    }
}
