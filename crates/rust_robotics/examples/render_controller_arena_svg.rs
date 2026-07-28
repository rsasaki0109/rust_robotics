//! Render the deterministic Controller Arena comparison as a gallery SVG.

use std::fmt::Write as _;
use std::path::Path;

use rust_robotics::control::{
    run_controller_arena, ArenaControllerKind, ArenaPreset, ArenaRun, ArenaScenario,
};
use rust_robotics::prelude::*;

const OUTPUT: &str = "docs/assets/controller-arena.svg";
const WIDTH: f64 = 1100.0;
const HEIGHT: f64 = 620.0;
const PLOT_X: f64 = 54.0;
const PLOT_Y: f64 = 112.0;
const PLOT_W: f64 = 680.0;
const PLOT_H: f64 = 430.0;

fn color(kind: ArenaControllerKind) -> &'static str {
    match kind {
        ArenaControllerKind::PurePursuit => "#3b82f6",
        ArenaControllerKind::Stanley => "#f97316",
        ArenaControllerKind::LqrSteer => "#22c55e",
    }
}

fn bounds(scenario: &ArenaScenario, runs: &[ArenaRun]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for (x, y) in scenario
        .path
        .points
        .iter()
        .map(|point| (point.x, point.y))
        .chain(
            runs.iter()
                .flat_map(|run| run.samples.iter())
                .map(|sample| (sample.state.x, sample.state.y)),
        )
    {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    (min_x - 2.0, max_x + 2.0, min_y - 2.0, max_y + 2.0)
}

fn screen_point(x: f64, y: f64, (min_x, max_x, min_y, max_y): (f64, f64, f64, f64)) -> (f64, f64) {
    let scale = (PLOT_W / (max_x - min_x)).min(PLOT_H / (max_y - min_y));
    let drawn_w = (max_x - min_x) * scale;
    let drawn_h = (max_y - min_y) * scale;
    let left = PLOT_X + (PLOT_W - drawn_w) / 2.0;
    let top = PLOT_Y + (PLOT_H - drawn_h) / 2.0;
    (
        left + (x - min_x) * scale,
        top + drawn_h - (y - min_y) * scale,
    )
}

fn path_points(points: impl Iterator<Item = (f64, f64)>, world: (f64, f64, f64, f64)) -> String {
    points
        .map(|(x, y)| {
            let (sx, sy) = screen_point(x, y, world);
            format!("{sx:.1},{sy:.1}")
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn render() -> RoboticsResult<String> {
    let scenario = ArenaPreset::SlalomRecovery.scenario(3.0, 0.85);
    let runs = run_controller_arena(&scenario, &ArenaControllerKind::ALL)?;
    let world = bounds(&scenario, &runs);
    let mut svg = String::new();

    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH:.0}" height="{HEIGHT:.0}" viewBox="0 0 {WIDTH:.0} {HEIGHT:.0}" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">RustRobotics Controller Arena</title><desc id="desc">Pure Pursuit, Stanley, and LQR Steer following the same slalom path with a table of error and smoothness metrics.</desc>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect width="{WIDTH:.0}" height="{HEIGHT:.0}" fill="#111318"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="48" fill="#f8fafc" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="27" font-weight="700">Controller Arena</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="76" fill="#a8b2c1" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Pure Pursuit · Stanley · LQR Steer — same slalom, initial state, 3.0 m/s clock, and 85% turn response</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect x="{PLOT_X}" y="{PLOT_Y}" width="{PLOT_W}" height="{PLOT_H}" rx="10" fill="#171b22" stroke="#293241"/>"##
    )
    .unwrap();

    let reference = path_points(
        scenario.path.points.iter().map(|point| (point.x, point.y)),
        world,
    );
    writeln!(
        svg,
        r##"<polyline points="{reference}" fill="none" stroke="#b8c0cc" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();

    for run in &runs {
        let trace = path_points(
            run.samples
                .iter()
                .step_by(2)
                .map(|sample| (sample.state.x, sample.state.y)),
            world,
        );
        let run_color = color(run.controller);
        writeln!(
            svg,
            r##"<polyline points="{trace}" fill="none" stroke="{run_color}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" opacity="0.94"/>"##
        )
        .unwrap();
        let last = run.samples.last().expect("arena run is non-empty").state;
        let (x, y) = screen_point(last.x, last.y, world);
        writeln!(
            svg,
            r##"<circle cx="{x:.1}" cy="{y:.1}" r="6" fill="{run_color}" stroke="#ffffff" stroke-width="2"/>"##
        )
        .unwrap();
    }

    let panel_x = 770.0;
    writeln!(
        svg,
        r##"<rect x="{panel_x}" y="112" width="276" height="430" rx="10" fill="#171b22" stroke="#293241"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="794" y="148" fill="#f8fafc" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="17" font-weight="700">Identical-condition metrics</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="794" y="176" fill="#7f8b9b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">RMSE / final / max error (m)</text>"##
    )
    .unwrap();

    for (index, run) in runs.iter().enumerate() {
        let y = 222.0 + index as f64 * 92.0;
        let run_color = color(run.controller);
        writeln!(
            svg,
            r##"<circle cx="803" cy="{:.1}" r="5" fill="{run_color}"/><text x="818" y="{:.1}" fill="{run_color}" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">{}</text>"##,
            y - 5.0,
            y,
            run.controller.label()
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="794" y="{:.1}" fill="#d9e0ea" font-family="ui-monospace, Consolas, monospace" font-size="13">{:.3} / {:.3} / {:.3}</text>"##,
            y + 25.0,
            run.metrics.cross_track_rmse,
            run.metrics.final_goal_distance,
            run.metrics.max_cross_track_error
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="794" y="{:.1}" fill="#7f8b9b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11">Δω RMS {:.3} rad/s</text>"##,
            y + 45.0,
            run.metrics.angular_command_smoothness
        )
        .unwrap();
    }

    writeln!(
        svg,
        r##"<text x="54" y="582" fill="#7f8b9b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">Deterministic headless traces replayed by the Rust/WASM Playground · comparative evidence, not a universal ranking</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let svg = render()?;
    let output = Path::new(OUTPUT);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, svg)?;
    println!("wrote {OUTPUT}");
    Ok(())
}
