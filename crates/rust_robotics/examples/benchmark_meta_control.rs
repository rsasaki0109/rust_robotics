//! Meta-Control benchmark: controller mode selection vs fixed controllers.
//!
//! Runs every Controller Arena preset under identical conditions with the two
//! deterministic meta policies (switch-on-error, switch-on-curvature) and each
//! fixed controller. Writes a CSV of metrics plus an SVG showing where each
//! policy switches modes along the path.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    run_meta_control, ArenaControllerKind, ArenaPreset, MetaControlPolicy, MetaControlRun,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/meta-control.csv";
const SVG_OUTPUT: &str = "docs/assets/meta-control.svg";

const ERROR_POLICY: MetaControlPolicy = MetaControlPolicy::SwitchOnError {
    error_threshold: 0.5,
    error_mode: ArenaControllerKind::Stanley,
    cruise_mode: ArenaControllerKind::PurePursuit,
};

const CURVATURE_POLICY: MetaControlPolicy = MetaControlPolicy::SwitchOnCurvature {
    curvature_threshold: 0.10,
    tight_mode: ArenaControllerKind::LqrSteer,
    smooth_mode: ArenaControllerKind::PurePursuit,
};

fn meta_policies() -> [(MetaControlPolicy, &'static str); 2] {
    [
        (ERROR_POLICY, "switch-on-error"),
        (CURVATURE_POLICY, "switch-on-curvature"),
    ]
}

struct PolicyRow {
    preset: &'static str,
    policy: String,
    run: MetaControlRun,
}

fn collect_rows() -> RoboticsResult<Vec<PolicyRow>> {
    let mut rows = Vec::new();
    for preset in ArenaPreset::ALL {
        let scenario = preset.scenario(3.0, 0.85);
        for kind in ArenaControllerKind::ALL {
            let run = run_meta_control(&scenario, MetaControlPolicy::Fixed(kind))?;
            rows.push(PolicyRow {
                preset: preset.slug(),
                policy: format!("fixed: {}", kind.label().to_lowercase()),
                run,
            });
        }
        for (policy, slug) in meta_policies() {
            let run = run_meta_control(&scenario, policy)?;
            rows.push(PolicyRow {
                preset: preset.slug(),
                policy: slug.to_string(),
                run,
            });
        }
    }
    Ok(rows)
}

fn render_csv(rows: &[PolicyRow]) -> String {
    let mut csv = String::from(
        "preset,policy,cross_track_rmse,final_goal_distance,max_cross_track_error,angular_smoothness,switch_count,samples,mode_share_pure_pursuit,mode_share_stanley,mode_share_lqr_steer\n",
    );
    for row in rows {
        let counts = row.run.mode_counts();
        let _ = writeln!(
            csv,
            "{},{},{:.4},{:.4},{:.4},{:.4},{},{},{:.3},{:.3},{:.3}",
            row.preset,
            row.policy,
            row.run.metrics.cross_track_rmse,
            row.run.metrics.final_goal_distance,
            row.run.metrics.max_cross_track_error,
            row.run.metrics.angular_command_smoothness,
            row.run.switch_count,
            row.run.samples.len(),
            counts[0] as f64 / row.run.samples.len() as f64,
            counts[1] as f64 / row.run.samples.len() as f64,
            counts[2] as f64 / row.run.samples.len() as f64,
        );
    }
    csv
}

// --- SVG rendering ---------------------------------------------------------

const WIDTH: f64 = 1200.0;
const HEIGHT: f64 = 940.0;
const PLOT_X: f64 = 54.0;
const PLOT_Y: f64 = 112.0;
const PLOT_W: f64 = 1090.0;
const PLOT_H: f64 = 220.0;
const PANEL_GAP: f64 = 20.0;

fn mode_color(mode: ArenaControllerKind) -> &'static str {
    match mode {
        ArenaControllerKind::PurePursuit => "#3b82f6",
        ArenaControllerKind::Stanley => "#f97316",
        ArenaControllerKind::LqrSteer => "#22c55e",
    }
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

fn world_bounds(rows: &[PolicyRow], preset: &'static str) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for row in rows.iter().filter(|row| row.preset == preset) {
        for sample in &row.run.samples {
            min_x = min_x.min(sample.state.x);
            max_x = max_x.max(sample.state.x);
            min_y = min_y.min(sample.state.y);
            max_y = max_y.max(sample.state.y);
        }
    }
    (min_x - 1.5, max_x + 1.5, min_y - 1.5, max_y + 1.5)
}

fn render() -> RoboticsResult<String> {
    let rows = collect_rows()?;
    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH:.0}" height="{HEIGHT:.0}" viewBox="0 0 {WIDTH:.0} {HEIGHT:.0}" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">RustRobotics Meta-Control</title><desc id="desc">Controller mode selection switching between Pure Pursuit, Stanley, and LQR Steer mid-run on straight, slalom, and hairpin paths.</desc>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<rect width="{WIDTH:.0}" height="{HEIGHT:.0}" fill="#111318"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="48" fill="#f8fafc" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="27" font-weight="700">Meta-Control: mode selection</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="76" fill="#a8b2c1" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Same 3.0 m/s clock and 85% turn response. Segment color = active controller mode.</text>"##
    )
    .unwrap();

    let legend = [
        ("Pure Pursuit", mode_color(ArenaControllerKind::PurePursuit)),
        ("Stanley", mode_color(ArenaControllerKind::Stanley)),
        ("LQR Steer", mode_color(ArenaControllerKind::LqrSteer)),
    ];
    for (index, (label, color)) in legend.iter().enumerate() {
        let x = 800.0 + index as f64 * 140.0;
        writeln!(
            svg,
            r##"<rect x="{x:.0}" y="58" width="16" height="16" rx="3" fill="{color}"/><text x="{:.0}" y="71" fill="#e2e8f0" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">{}</text>"##,
            x + 22.0,
            label
        )
        .unwrap();
    }

    for (panel, preset) in ArenaPreset::ALL.iter().enumerate() {
        let offset = panel as f64 * (PLOT_H + PANEL_GAP);
        let scenario = preset.scenario(3.0, 0.85);
        let world = world_bounds(&rows, preset.slug());
        writeln!(
            svg,
            r##"<rect x="{PLOT_X}" y="{:.1}" width="{PLOT_W}" height="{PLOT_H}" rx="10" fill="#171b22" stroke="#293241"/>"##,
            PLOT_Y + offset
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="68" y="{:.1}" fill="#f8fafc" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="16" font-weight="700">{}</text>"##,
            PLOT_Y + offset + 30.0,
            preset.label()
        )
        .unwrap();

        // Reference path.
        let path_points = scenario
            .path
            .points
            .iter()
            .map(|point| {
                let (sx, sy) = screen_point(point.x, point.y, world);
                format!("{sx:.1},{sy:.1}")
            })
            .collect::<Vec<_>>()
            .join(" ");
        writeln!(
            svg,
            r##"<polyline points="{path_points}" fill="none" stroke="#b8c0cc" stroke-width="3.5" stroke-linecap="round" stroke-linejoin="round"/>"##
        )
        .unwrap();

        // Meta-policy trajectories, colored by active mode per segment.
        for row in rows.iter().filter(|row| row.preset == preset.slug()) {
            for pair in row.run.samples.windows(2) {
                let (x0, y0) = screen_point(pair[0].state.x, pair[0].state.y, world);
                let (x1, y1) = screen_point(pair[1].state.x, pair[1].state.y, world);
                let color = mode_color(pair[1].active_mode);
                writeln!(
                    svg,
                    r##"<line x1="{x0:.1}" y1="{y0:.1}" x2="{x1:.1}" y2="{y1:.1}" stroke="{color}" stroke-width="2.5" stroke-linecap="round" opacity="0.85"/>"##
                )
                .unwrap();
            }
        }
    }

    // Metrics table.
    let table_x = 54.0;
    let table_y = PLOT_Y + 3.0 * (PLOT_H + PANEL_GAP) + 6.0;
    writeln!(
        svg,
        r##"<text x="{table_x:.0}" y="{:.1}" fill="#f8fafc" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="17" font-weight="700">Metrics (same speed, same turn response)</text>"##,
        table_y
    )
    .unwrap();
    let headers = [
        "preset",
        "policy",
        "RMSE (m)",
        "final (m)",
        "max cte (m)",
        "smooth",
        "switches",
    ];
    for (index, header) in headers.iter().enumerate() {
        let x = table_x + index as f64 * 165.0;
        writeln!(
            svg,
            r##"<text x="{x:.0}" y="{:.1}" fill="#7f8b9b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="12">{header}</text>"##,
            table_y + 24.0
        )
        .unwrap();
    }
    for (index, row) in rows.iter().enumerate() {
        let y = table_y + 50.0 + index as f64 * 22.0;
        let values = [
            row.preset.to_string(),
            row.policy.to_string(),
            format!("{:.3}", row.run.metrics.cross_track_rmse),
            format!("{:.3}", row.run.metrics.final_goal_distance),
            format!("{:.3}", row.run.metrics.max_cross_track_error),
            format!("{:.3}", row.run.metrics.angular_command_smoothness),
            format!("{}", row.run.switch_count),
        ];
        for (column, value) in values.iter().enumerate() {
            let x = table_x + column as f64 * 165.0;
            writeln!(
                svg,
                r##"<text x="{x:.0}" y="{y:.1}" fill="#d9e0ea" font-family="ui-monospace, Consolas, monospace" font-size="13">{value}</text>"##
            )
            .unwrap();
        }
    }
    writeln!(svg, "</svg>").unwrap();
    Ok(svg)
}

fn main() -> RoboticsResult<()> {
    let rows = collect_rows()?;
    let path = Path::new(CSV_OUTPUT);
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, render_csv(&rows))?;
    fs::write(SVG_OUTPUT, render()?)?;
    println!(
        "wrote {} and {} ({} meta-control runs)",
        CSV_OUTPUT,
        SVG_OUTPUT,
        rows.len()
    );
    Ok(())
}
