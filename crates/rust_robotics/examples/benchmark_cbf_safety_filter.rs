//! PolyMerge-lite CBF safety filter over convex polytope obstacles.
//!
//! Each scenario drives a proportional go-to-goal controller through a field of
//! convex obstacles twice: once with the raw nominal control (which drives into
//! obstacles) and once through the control-barrier-function safety filter
//! (which minimally corrects the velocity to stay clear). Results report
//! clearance, collisions, and intervention counts to CSV, and draw the nominal
//! and filtered trajectories per scenario to an SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_cbf_navigation, CbfConvexObstacle2D, CbfNavConfig, CbfNavReport, CbfSafetyFilter,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/cbf-safety-filter.csv";
const SVG_OUTPUT: &str = "docs/assets/cbf-safety-filter.svg";

const ALPHA: f64 = 2.0;
const ROBOT_RADIUS: f64 = 0.2;

struct Scenario {
    name: &'static str,
    obstacles: Vec<CbfConvexObstacle2D>,
    start: [f64; 2],
    goal: [f64; 2],
}

fn box_at(cx: f64, cy: f64, half_w: f64, half_h: f64) -> RoboticsResult<CbfConvexObstacle2D> {
    CbfConvexObstacle2D::aabb(cx - half_w, cx + half_w, cy - half_h, cy + half_h)
}

// Obstacles are placed so their blocking face sits just below (or above) the
// straight-line path. The raw policy clips the face and collides, while the
// filter meets the obstacle's near corner and is deflected along the face,
// avoiding the head-on deadlock a reactive CBF would otherwise hit when a flat
// face is square to the goal pull.
fn scenarios() -> RoboticsResult<Vec<Scenario>> {
    // 1. A single obstacle whose top edge just clips the path; robot rides over.
    let graze = Scenario {
        name: "graze-box",
        obstacles: vec![box_at(3.0, -0.85, 0.8, 0.8)?], // top at y = -0.05
        start: [0.0, 0.0],
        goal: [6.0, 0.0],
    };

    // 2. Two staggered obstacles (one below, one above) forcing a slalom.
    let slalom = Scenario {
        name: "slalom",
        obstacles: vec![
            box_at(2.2, -0.85, 0.55, 0.8)?, // top at y = -0.05, deflect up
            box_at(4.2, 0.85, 0.55, 0.8)?,  // bottom at y = 0.05, deflect down
        ],
        start: [0.0, 0.0],
        goal: [6.0, 0.0],
    };

    // 3. A ridge of small obstacles the robot rides over (all below the path,
    //    so it is never lifted into a side face).
    let cluster = Scenario {
        name: "ridge",
        obstacles: vec![
            box_at(2.0, -0.8, 0.42, 0.75)?, // top at -0.05
            box_at(3.2, -0.8, 0.42, 0.75)?, // top at -0.05
            box_at(4.4, -0.8, 0.42, 0.75)?, // top at -0.05
        ],
        start: [0.0, 0.0],
        goal: [6.0, 0.0],
    };

    // 4. A wide obstacle the robot must ride over for a sustained stretch.
    let wide = Scenario {
        name: "wide-box",
        obstacles: vec![box_at(3.0, -1.05, 1.3, 1.0)?], // top at y = -0.05
        start: [0.0, 0.0],
        goal: [6.0, 0.0],
    };

    Ok(vec![graze, slalom, cluster, wide])
}

struct Row {
    name: &'static str,
    obstacles: Vec<CbfConvexObstacle2D>,
    start: [f64; 2],
    goal: [f64; 2],
    nominal: CbfNavReport,
    filtered: CbfNavReport,
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let config = CbfNavConfig::default();
    let mut rows = Vec::new();
    for scenario in scenarios()? {
        let filter = CbfSafetyFilter::new(scenario.obstacles.clone(), ALPHA, ROBOT_RADIUS)?
            .with_margin(0.08)?;
        let nominal =
            simulate_cbf_navigation(&filter, scenario.start, scenario.goal, &config, false)?;
        let filtered =
            simulate_cbf_navigation(&filter, scenario.start, scenario.goal, &config, true)?;
        rows.push(Row {
            name: scenario.name,
            obstacles: scenario.obstacles,
            start: scenario.start,
            goal: scenario.goal,
            nominal,
            filtered,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,nominal_collided,nominal_min_clearance,filtered_collided,filtered_min_clearance,filtered_reached,interventions,mean_correction,filtered_path_length\n",
    );
    for row in rows {
        let _ = writeln!(
            csv,
            "{},{},{:.3},{},{:.3},{},{},{:.3},{:.3}",
            row.name,
            row.nominal.collided,
            row.nominal.min_clearance,
            row.filtered.collided,
            row.filtered.min_clearance,
            row.filtered.reached_goal,
            row.filtered.interventions,
            row.filtered.mean_correction,
            row.filtered.path_length
        );
    }
    csv
}

// --- SVG panel rendering -------------------------------------------------

const PANEL_W: f64 = 380.0;
const PANEL_H: f64 = 240.0;
const WORLD_MIN_X: f64 = -0.6;
const WORLD_MAX_X: f64 = 6.6;
const WORLD_MIN_Y: f64 = -2.2;
const WORLD_MAX_Y: f64 = 2.2;

fn sx(x: f64, ox: f64) -> f64 {
    ox + (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X) * PANEL_W
}

fn sy(y: f64, oy: f64) -> f64 {
    // Flip y so up is up.
    oy + (WORLD_MAX_Y - y) / (WORLD_MAX_Y - WORLD_MIN_Y) * PANEL_H
}

fn panel(row: &Row, ox: f64, oy: f64) -> String {
    let mut svg = String::new();
    let _ = writeln!(
        svg,
        "<rect x=\"{ox:.1}\" y=\"{oy:.1}\" width=\"{PANEL_W}\" height=\"{PANEL_H}\" fill=\"#ffffff\" stroke=\"#e5e5ea\"/>"
    );
    // Obstacles.
    for obstacle in &row.obstacles {
        let mut points = String::new();
        for v in obstacle.vertices() {
            let _ = write!(points, "{:.1},{:.1} ", sx(v[0], ox), sy(v[1], oy));
        }
        let _ = writeln!(
            svg,
            "<polygon points=\"{}\" fill=\"#ff3b30\" fill-opacity=\"0.18\" stroke=\"#ff3b30\" stroke-width=\"1.5\"/>",
            points.trim()
        );
    }
    // Nominal path (orange, may enter obstacles).
    svg.push_str(&path_polyline(
        &row.nominal.path,
        ox,
        oy,
        "#ff9f0a",
        "stroke-dasharray=\"5 4\"",
    ));
    // Filtered path (blue).
    svg.push_str(&path_polyline(&row.filtered.path, ox, oy, "#0a84ff", ""));
    // Start and goal.
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" fill=\"#1d1d1f\"/>",
        sx(row.start[0], ox),
        sy(row.start[1], oy)
    );
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" fill=\"#34c759\"/>",
        sx(row.goal[0], ox),
        sy(row.goal[1], oy)
    );
    // Title and metrics.
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#1d1d1f\">{}</text>",
        ox + 10.0,
        oy + 20.0,
        row.name
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\">nominal clearance {:.2}{} · filtered {:.2} ({} interventions)</text>",
        ox + 10.0,
        oy + PANEL_H - 12.0,
        row.nominal.min_clearance,
        if row.nominal.collided { " (hit)" } else { "" },
        row.filtered.min_clearance,
        row.filtered.interventions
    );
    svg
}

fn path_polyline(path: &[[f64; 2]], ox: f64, oy: f64, color: &str, dash: &str) -> String {
    let mut points = String::new();
    for p in path {
        let _ = write!(points, "{:.1},{:.1} ", sx(p[0], ox), sy(p[1], oy));
    }
    format!(
        "<polyline points=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"2.5\" {dash}/>\n",
        points.trim()
    )
}

fn render_svg(rows: &[Row]) -> String {
    let cols = 2;
    let rows_count = rows.len().div_ceil(cols);
    let margin = 24.0;
    let gap = 24.0;
    let top = 84.0;
    let width = margin * 2.0 + cols as f64 * PANEL_W + (cols as f64 - 1.0) * gap;
    let height = top + rows_count as f64 * (PANEL_H + gap) + margin;

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
        "<text x=\"{margin}\" y=\"38\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">CBF safety filter over convex polytope obstacles</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{margin}\" y=\"60\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">orange = raw go-to-goal (drives into obstacles); blue = CBF-filtered (minimal correction, stays clear)</text>"
    );

    for (index, row) in rows.iter().enumerate() {
        let col = index % cols;
        let r = index / cols;
        let ox = margin + col as f64 * (PANEL_W + gap);
        let oy = top + r as f64 * (PANEL_H + gap);
        svg.push_str(&panel(row, ox, oy));
    }

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("CBF safety filter over convex polytope obstacles");
    for row in rows {
        println!(
            "  {:<10} nominal: collided={} clearance={:.2} | filtered: collided={} clearance={:.2} reached={} interventions={}",
            row.name,
            row.nominal.collided,
            row.nominal.min_clearance,
            row.filtered.collided,
            row.filtered.min_clearance,
            row.filtered.reached_goal,
            row.filtered.interventions
        );
    }
}

fn main() -> RoboticsResult<()> {
    let rows = collect_rows()?;
    print_summary(&rows);

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&rows)).ok();
    fs::write(SVG_OUTPUT, render_svg(&rows)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sweep_is_deterministic() {
        let first = collect_rows().unwrap();
        let second = collect_rows().unwrap();
        for (a, b) in first.iter().zip(&second) {
            assert_eq!(a.filtered.path, b.filtered.path);
        }
    }

    #[test]
    fn filter_prevents_every_collision() {
        for row in collect_rows().unwrap() {
            assert!(
                !row.filtered.collided,
                "{} collided under the filter (clearance {})",
                row.name, row.filtered.min_clearance
            );
            assert!(
                row.filtered.reached_goal,
                "{} did not reach the goal",
                row.name
            );
        }
    }

    #[test]
    fn at_least_one_nominal_collides() {
        let rows = collect_rows().unwrap();
        assert!(
            rows.iter().any(|r| r.nominal.collided),
            "no scenario shows an unfiltered collision"
        );
    }
}
