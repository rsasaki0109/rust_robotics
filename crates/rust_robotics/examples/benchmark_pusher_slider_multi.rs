//! Multi-object planar pushing: arrange several sliders into goal slots.
//!
//! Reproduces the multi-object setting of "Push Anything" with the face-aware
//! quasi-static pusher. The sliders are pushed one at a time; while one is being
//! pushed, the others (at their current poses) are keep-out obstacles, so the
//! active slider is routed around blocks that sit in its straight-line path
//! instead of plowing through them. The benchmark renders every slider's start,
//! goal, and final box with its CoM path to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_multi_push, MultiPushReport, PusherMppiConfig, PusherSliderParams, SliderState,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/pusher-slider-multi.csv";
const SVG_OUTPUT: &str = "docs/assets/pusher-slider-multi.svg";

const MAX_STEPS: usize = 240;

fn params() -> PusherSliderParams {
    PusherSliderParams::default()
}

fn config() -> PusherMppiConfig {
    let b = params().half_extent;
    PusherMppiConfig {
        horizon: 16,
        samples: 320,
        obstacle_weight: 500.0,
        obstacle_radius: 2.6 * b,
        ..PusherMppiConfig::default()
    }
}

fn starts() -> Vec<SliderState> {
    vec![
        SliderState::new(0.0, 0.0, 0.0),
        SliderState::new(0.16, 0.0, 0.0),
        SliderState::new(0.0, -0.14, 0.0),
    ]
}

fn goals() -> Vec<SliderState> {
    // Object 0 must travel straight through where object 1 currently rests, so it
    // detours around it; object 1 then moves up and object 2 slides across.
    vec![
        SliderState::new(0.34, 0.0, 0.0),
        SliderState::new(0.16, 0.15, 0.0),
        SliderState::new(0.34, -0.14, 0.0),
    ]
}

fn run() -> RoboticsResult<MultiPushReport> {
    simulate_multi_push(config(), params(), &starts(), &goals(), MAX_STEPS)
}

fn render_csv(report: &MultiPushReport) -> String {
    let mut csv = String::from(
        "object,goal_x,goal_y,goal_theta,final_x,final_y,final_theta,position_error,heading_error_deg,steps\n",
    );
    let goals = goals();
    for (i, r) in report.objects.iter().enumerate() {
        let _ = writeln!(
            csv,
            "{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.2},{}",
            i,
            goals[i].x(),
            goals[i].y(),
            goals[i].theta(),
            r.final_pose[0],
            r.final_pose[1],
            r.final_pose[2],
            r.position_error,
            r.heading_error.to_degrees(),
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

const COLORS: [&str; 3] = ["#0a84ff", "#ff9f0a", "#34c759"];

fn render_svg(report: &MultiPushReport) -> String {
    let b = params().half_extent;
    let width = 880.0;
    let height = 420.0;
    let left = 40.0;
    let right = width - 40.0;
    let top = 70.0;
    let bottom = height - 40.0;

    let starts = starts();
    let goals = goals();

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for r in &report.objects {
        for p in &r.path {
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
    let map = |w: [f64; 2]| {
        let sx = left + ((w[0] - min_x) / span_x) * (right - left);
        let sy = bottom - ((w[1] - min_y) / span_y) * (bottom - top);
        (sx, sy)
    };

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
        "<text x=\"{left}\" y=\"32\" font-family=\"sans-serif\" font-size=\"18\" fill=\"#1d1d1f\">Multi-object pushing: sliders arranged into a row, routed around each other</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"52\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">gray = start, dashed = goal, solid = final; each object avoids the others while it is pushed</text>"
    );

    let poly = |svg: &mut String, pts: &[[f64; 2]], fill: &str, stroke: &str, dash: &str| {
        let mut s = String::new();
        for p in pts {
            let (sx, sy) = map(*p);
            let _ = write!(s, "{sx:.1},{sy:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polygon points=\"{}\" fill=\"{fill}\" stroke=\"{stroke}\" stroke-width=\"1.5\"{dash}/>",
            s.trim()
        );
    };

    for (i, r) in report.objects.iter().enumerate() {
        let color = COLORS[i % COLORS.len()];
        let mut path = String::new();
        for p in &r.path {
            let (sx, sy) = map([p[0], p[1]]);
            let _ = write!(path, "{sx:.1},{sy:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"1.5\" opacity=\"0.5\"/>",
            path.trim()
        );
        poly(
            &mut svg,
            &box_corners(starts[i].pose, b),
            "#e5e5ea",
            "#8e8e93",
            "",
        );
        poly(
            &mut svg,
            &box_corners(goals[i].pose, b),
            "none",
            color,
            " stroke-dasharray=\"4 3\"",
        );
        poly(
            &mut svg,
            &box_corners(r.final_pose, b),
            "#0a84ff22",
            color,
            "",
        );
    }

    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\">max position error {:.1} mm · max heading error {:.1} deg</text>",
        height - 14.0,
        report.max_position_error * 1000.0,
        report.max_heading_error.to_degrees()
    );

    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let report = run()?;
    println!("multi-object pushing (face-aware MPPI, sequential with keep-out)");
    for (i, r) in report.objects.iter().enumerate() {
        println!(
            "  object {i}: err={:.1}mm heading_err={:.1}deg steps={}",
            r.position_error * 1000.0,
            r.heading_error.to_degrees(),
            r.steps
        );
    }
    println!(
        "  max position error {:.1}mm, max heading error {:.1}deg",
        report.max_position_error * 1000.0,
        report.max_heading_error.to_degrees()
    );

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&report)).ok();
    fs::write(SVG_OUTPUT, render_svg(&report)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_is_deterministic() {
        let a = run().unwrap();
        let b = run().unwrap();
        assert_eq!(a.final_poses, b.final_poses);
    }

    #[test]
    fn all_objects_reach_their_slots() {
        let report = run().unwrap();
        let b = params().half_extent;
        assert!(
            report.max_position_error < 3.0 * b,
            "max position error {} (b={})",
            report.max_position_error,
            b
        );
    }
}
