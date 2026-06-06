//! Quasi-static planar pushing: drive a square slider to goal poses with MPPI.
//!
//! Each scenario pushes a square slider from the origin to a goal pose using a
//! single point pusher on the slider's back face, under the quasi-static
//! ellipsoidal limit-surface model with stick/slide contact modes. The benchmark
//! reports the final pose error and the stick/slide split, and renders the start,
//! goal, and final slider boxes with the CoM path to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_push, PushReport, PusherMppiConfig, PusherSliderParams, SliderState,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/pusher-slider.csv";
const SVG_OUTPUT: &str = "docs/assets/pusher-slider.svg";

struct Scenario {
    name: &'static str,
    start: SliderState,
    goal: SliderState,
    max_steps: usize,
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "forward",
            start: SliderState::new(0.0, 0.0, 0.0),
            goal: SliderState::new(0.30, 0.0, 0.0),
            max_steps: 160,
        },
        Scenario {
            name: "veer",
            start: SliderState::new(0.0, 0.0, 0.0),
            goal: SliderState::new(0.26, 0.07, 0.0),
            max_steps: 200,
        },
        Scenario {
            name: "reorient",
            start: SliderState::new(0.0, 0.0, 0.0),
            // Rotation paired with the lateral drift it naturally induces, so a
            // single back-face pusher can realize it.
            goal: SliderState::new(0.24, 0.07, 0.4),
            max_steps: 220,
        },
        Scenario {
            name: "park",
            start: SliderState::new(0.0, 0.0, 0.0),
            goal: SliderState::new(0.22, -0.06, -0.3),
            max_steps: 220,
        },
    ]
}

struct Row {
    name: &'static str,
    start: SliderState,
    goal: SliderState,
    report: PushReport,
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let params = PusherSliderParams::default();
    let mut rows = Vec::new();
    for s in scenarios() {
        let report = simulate_push(
            PusherMppiConfig::default(),
            params,
            s.start,
            s.goal,
            s.max_steps,
        )?;
        rows.push(Row {
            name: s.name,
            start: s.start,
            goal: s.goal,
            report,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,goal_x,goal_y,goal_theta,final_x,final_y,final_theta,position_error,heading_error_deg,stick_fraction,slide_fraction,steps\n",
    );
    for row in rows {
        let r = &row.report;
        let _ = writeln!(
            csv,
            "{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.2},{:.3},{:.3},{}",
            row.name,
            row.goal.x(),
            row.goal.y(),
            row.goal.theta(),
            r.final_pose[0],
            r.final_pose[1],
            r.final_pose[2],
            r.position_error,
            r.heading_error.to_degrees(),
            r.stick_fraction,
            r.slide_fraction,
            r.steps
        );
    }
    csv
}

fn box_corners(pose: [f64; 3], b: f64) -> [[f64; 2]; 4] {
    let (s, c) = pose[2].sin_cos();
    let local = [[-b, -b], [b, -b], [b, b], [-b, b]];
    let mut out = [[0.0; 2]; 4];
    for (o, l) in out.iter_mut().zip(local.iter()) {
        o[0] = pose[0] + c * l[0] - s * l[1];
        o[1] = pose[1] + s * l[0] + c * l[1];
    }
    out
}

fn render_svg(rows: &[Row]) -> String {
    let b = PusherSliderParams::default().half_extent;
    let panel_w = 440.0;
    let panel_h = 210.0;
    let cols = 2.0;
    let rows_n = 2.0;
    let width = panel_w * cols;
    let height = panel_h * rows_n + 40.0;

    // World bounds across all scenarios.
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for row in rows {
        for p in row.report.path.iter() {
            min_x = min_x.min(p[0]);
            max_x = max_x.max(p[0]);
            min_y = min_y.min(p[1]);
            max_y = max_y.max(p[1]);
        }
    }
    min_x -= 2.0 * b;
    max_x += 2.0 * b;
    min_y -= 2.0 * b;
    max_y += 2.0 * b;
    let span_x = (max_x - min_x).max(1e-6);
    let span_y = (max_y - min_y).max(1e-6);

    let mut svg = String::new();
    let _ = writeln!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\" viewBox=\"0 0 {width} {height}\">"
    );
    let _ = writeln!(
        svg,
        "<rect width=\"{width}\" height=\"{height}\" fill=\"#fbfbfd\"/>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"16\" y=\"26\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1d1d1f\">Quasi-static planar pushing: slider driven to goal poses by an MPPI pusher</text>"
    );

    let poly = |svg: &mut String, pts: &[[f64; 2]], px: f64, py: f64, fill: &str, stroke: &str| {
        let map = |w: [f64; 2]| {
            let sx = px + ((w[0] - min_x) / span_x) * (panel_w - 40.0) + 20.0;
            let sy = py + (panel_h - 30.0) - ((w[1] - min_y) / span_y) * (panel_h - 50.0);
            (sx, sy)
        };
        let mut s = String::new();
        for p in pts {
            let (sx, sy) = map(*p);
            let _ = write!(s, "{sx:.1},{sy:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polygon points=\"{}\" fill=\"{fill}\" stroke=\"{stroke}\" stroke-width=\"1.5\"/>",
            s.trim()
        );
    };

    for (i, row) in rows.iter().enumerate() {
        let px = (i % 2) as f64 * panel_w;
        let py = 40.0 + (i / 2) as f64 * panel_h;
        let map = |w: [f64; 2]| {
            let sx = px + ((w[0] - min_x) / span_x) * (panel_w - 40.0) + 20.0;
            let sy = py + (panel_h - 30.0) - ((w[1] - min_y) / span_y) * (panel_h - 50.0);
            (sx, sy)
        };

        // CoM path.
        let mut path = String::new();
        for p in row.report.path.iter() {
            let (sx, sy) = map([p[0], p[1]]);
            let _ = write!(path, "{sx:.1},{sy:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"#c7c7cc\" stroke-width=\"1.5\"/>",
            path.trim()
        );

        // Start (gray), goal (green outline), final (blue).
        poly(
            &mut svg,
            &box_corners(row.start.pose, b),
            px,
            py,
            "#e5e5ea",
            "#8e8e93",
        );
        poly(
            &mut svg,
            &box_corners(row.goal.pose, b),
            px,
            py,
            "none",
            "#34c759",
        );
        poly(
            &mut svg,
            &box_corners(row.report.final_pose, b),
            px,
            py,
            "#0a84ff33",
            "#0a84ff",
        );

        let r = &row.report;
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\">{}</text>",
            px + 20.0,
            py + 16.0,
            row.name
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\">err {:.1} mm · {:.0} deg · stick {:.0}%</text>",
            px + 20.0,
            py + 30.0,
            r.position_error * 1000.0,
            r.heading_error.to_degrees(),
            r.stick_fraction * 100.0
        );
    }

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("quasi-static planar pushing (single back-face MPPI pusher)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<10} err={:.1}mm heading_err={:.1}deg stick={:.0}% slide={:.0}% steps={}",
            row.name,
            r.position_error * 1000.0,
            r.heading_error.to_degrees(),
            r.stick_fraction * 100.0,
            r.slide_fraction * 100.0,
            r.steps
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
        let a = collect_rows().unwrap();
        let b = collect_rows().unwrap();
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.report.path, y.report.path);
            assert_eq!(x.report.position_error, y.report.position_error);
        }
    }

    #[test]
    fn every_scenario_makes_clear_progress() {
        for row in collect_rows().unwrap() {
            let start_err = ((row.start.x() - row.goal.x()).powi(2)
                + (row.start.y() - row.goal.y()).powi(2))
            .sqrt();
            assert!(
                row.report.position_error < 0.5 * start_err,
                "{}: error {} vs start {}",
                row.name,
                row.report.position_error,
                start_err
            );
        }
    }
}
