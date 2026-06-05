//! Render SafeDec-lite STL-shielded navigation as a static SVG.
//!
//! A greedy goal-seeking policy would drive straight through a hazard geofence;
//! the STL shield reroutes it via constrained beam search. This draws both
//! paths over the grid — the greedy path cutting the hazard in orange, the
//! shielded detour in blue — and prints the robustness gain and intervention
//! count.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::planning::{
    SafeDecodePlan, SafeDecoder, SafeNavConfig, StlRectangle2D, StlTimeInterval, StlTimedCell,
    TimedRegion,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/safe-decode-nav.svg";
const LEFT: f64 = 50.0;
const TOP: f64 = 90.0;
const CELL: f64 = 34.0;
const WIDTH: i32 = 15;
const HEIGHT: i32 = 11;
const START: (i32, i32) = (0, 5);

fn rect(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> RoboticsResult<StlRectangle2D> {
    StlRectangle2D::new(min_x, max_x, min_y, max_y)
}

fn build() -> RoboticsResult<(SafeDecoder, Vec<TimedRegion>)> {
    let goal = (14, 5);
    let reach = TimedRegion::new(rect(13.0, 15.0, 4.0, 6.0)?, StlTimeInterval::new(0, 60)?);
    let mut config = SafeNavConfig::new(WIDTH, HEIGHT, goal, reach)?;
    config.horizon = 44;
    config.beam_width = 24;
    config.safety_margin = 0.5;
    // Two hazard geofences straddling the straight-line corridor at y = 5.
    config.avoid = vec![
        TimedRegion::new(rect(4.0, 6.0, 3.0, 8.0)?, StlTimeInterval::new(0, 60)?),
        TimedRegion::new(rect(8.0, 10.0, 2.0, 7.0)?, StlTimeInterval::new(0, 60)?),
    ];
    let avoid = config.avoid.clone();
    Ok((SafeDecoder::new(config)?, avoid))
}

fn px(x: i32) -> f64 {
    LEFT + x as f64 * CELL + CELL / 2.0
}

fn py(y: i32) -> f64 {
    TOP + y as f64 * CELL + CELL / 2.0
}

fn polyline(path: &[StlTimedCell], color: &str, dash: &str) -> String {
    let mut points = String::new();
    for cell in path {
        let _ = write!(points, "{:.1},{:.1} ", px(cell.x), py(cell.y));
    }
    format!(
        "<polyline points=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"3\" {dash} stroke-linejoin=\"round\"/>\n",
        points.trim()
    )
}

fn render_svg(plan: &SafeDecodePlan, avoid: &[TimedRegion]) -> String {
    let width = LEFT * 2.0 + WIDTH as f64 * CELL;
    let height = TOP + HEIGHT as f64 * CELL + 70.0;
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
        "<text x=\"{LEFT}\" y=\"36\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">SafeDec-lite: STL-shielded navigation decoding</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"58\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">greedy policy cuts through the geofences (orange); the STL shield reroutes via constrained beam search (blue)</text>"
    );

    // Grid.
    for x in 0..=WIDTH {
        let gx = LEFT + x as f64 * CELL;
        let _ = writeln!(
            svg,
            "<line x1=\"{gx:.1}\" y1=\"{TOP:.1}\" x2=\"{gx:.1}\" y2=\"{:.1}\" stroke=\"#e5e5ea\" stroke-width=\"1\"/>",
            TOP + HEIGHT as f64 * CELL
        );
    }
    for y in 0..=HEIGHT {
        let gy = TOP + y as f64 * CELL;
        let _ = writeln!(
            svg,
            "<line x1=\"{LEFT:.1}\" y1=\"{gy:.1}\" x2=\"{:.1}\" y2=\"{gy:.1}\" stroke=\"#e5e5ea\" stroke-width=\"1\"/>",
            LEFT + WIDTH as f64 * CELL
        );
    }

    // Hazard geofences.
    for spec in avoid {
        let r = spec.region;
        let x = LEFT + (r.min_x + 0.5) * CELL;
        let y = TOP + (r.min_y + 0.5) * CELL;
        let w = (r.max_x - r.min_x) * CELL;
        let h = (r.max_y - r.min_y) * CELL;
        let _ = writeln!(
            svg,
            "<rect x=\"{x:.1}\" y=\"{y:.1}\" width=\"{w:.1}\" height=\"{h:.1}\" fill=\"#ff3b30\" fill-opacity=\"0.16\" stroke=\"#ff3b30\" stroke-dasharray=\"5 4\"/>"
        );
    }

    // Paths.
    svg.push_str(&polyline(
        &plan.greedy_path,
        "#ff9f0a",
        "stroke-dasharray=\"6 5\"",
    ));
    svg.push_str(&polyline(&plan.shielded_path, "#0a84ff", ""));

    // Start and goal.
    let start = plan.shielded_path.first().copied().unwrap();
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"7\" fill=\"#1d1d1f\"/>",
        px(start.x),
        py(start.y)
    );
    let goal = plan.shielded_path.last().copied().unwrap();
    let _ = writeln!(
        svg,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"7\" fill=\"#34c759\"/>",
        px(goal.x),
        py(goal.y)
    );

    // Footer metrics.
    let footer = TOP + HEIGHT as f64 * CELL + 30.0;
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"{footer:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\">shielded: avoid robustness {:.2} (safe), reach {:.2}, {} interventions vs greedy</text>",
        plan.avoid_robustness, plan.reach_robustness, plan.interventions
    );
    let _ = writeln!(
        svg,
        "<text x=\"{LEFT}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">greedy: avoid robustness {:.2} (violates geofence)</text>",
        footer + 20.0,
        plan.greedy_avoid_robustness
    );

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let (decoder, avoid) = build()?;
    let plan = decoder.decode(START)?;

    println!("SafeDec-lite STL-shielded navigation decoding");
    println!(
        "  greedy : avoid_robustness={:.2} reach_robustness={:.2} (cuts through hazard)",
        plan.greedy_avoid_robustness, plan.greedy_reach_robustness
    );
    println!(
        "  shield : avoid_robustness={:.2} reach_robustness={:.2} reach_ok={} avoid_ok={} interventions={}",
        plan.avoid_robustness,
        plan.reach_robustness,
        plan.reach_satisfied,
        plan.avoid_satisfied,
        plan.interventions
    );

    if let Some(parent) = Path::new(OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(OUTPUT, render_svg(&plan, &avoid)).ok();
    println!("wrote {OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn render_is_deterministic_and_safe() {
        let (decoder, _) = build().unwrap();
        let first = decoder.decode(START).unwrap();
        let second = decoder.decode(START).unwrap();
        assert_eq!(first.shielded_path, second.shielded_path);
        assert!(first.avoid_satisfied);
        assert!(first.reach_satisfied);
        assert!(first.greedy_avoid_robustness < 0.0);
        assert!(first.interventions > 0);
    }
}
