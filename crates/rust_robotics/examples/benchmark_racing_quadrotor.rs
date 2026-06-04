//! Reference-free racing MPPI with a full quadrotor attitude model.
//!
//! Each scenario flies a seeded, deterministic MPPI quadrotor through a course
//! of 3-D gate planes. Unlike the point-mass `benchmark_racing_mppi_3d`, the
//! drone here can only accelerate along its body z-axis, so the gate-progress
//! objective has to drive the *attitude*: the quadrotor pitches and rolls to
//! aim its thrust vector toward the next gate. Results report lap progress plus
//! attitude metrics (tilt angle and body-rate effort) to CSV and SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_quadrotor_race, QuadrotorLapReport, QuadrotorMppiConfig, QuadrotorParams,
    QuadrotorState, RacingGateLap3D, RacingGatePlane3D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-quadrotor.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-quadrotor.svg";

const HALF: f64 = 0.9;
const UP: [f64; 3] = [0.0, 0.0, 1.0];

struct Scenario {
    name: &'static str,
    lap: RacingGateLap3D,
    params: QuadrotorParams,
    start: QuadrotorState,
    max_steps: usize,
    target_laps: usize,
}

/// A gate centered at `center` facing direction `normal` (need not be unit).
fn gate(center: [f64; 3], normal: [f64; 3]) -> RoboticsResult<RacingGatePlane3D> {
    RacingGatePlane3D::new(center, normal, UP, HALF, HALF)
}

fn scenarios() -> RoboticsResult<Vec<Scenario>> {
    // 1. Planar slalom: gates weave left/right on a flat plane, flown forward.
    let slalom_gates = vec![
        gate([1.6, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([3.2, -0.7, 0.0], [1.0, -0.5, 0.0])?,
        gate([4.8, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([6.4, -0.7, 0.0], [1.0, -0.5, 0.0])?,
    ];
    let slalom = Scenario {
        name: "planar-slalom",
        lap: RacingGateLap3D::open(slalom_gates)?,
        params: QuadrotorParams::default(),
        start: QuadrotorState::at(0.0, 0.0, 0.0),
        max_steps: 220,
        target_laps: 1,
    };

    // 2. Ascending course: gates climb in z, so the drone must hold extra thrust
    //    while tilting forward.
    let climb_gates = vec![
        gate([1.6, 0.0, 0.4], [1.0, 0.0, 0.4])?,
        gate([3.2, 0.0, 1.1], [1.0, 0.0, 0.5])?,
        gate([4.8, 0.0, 1.8], [1.0, 0.0, 0.5])?,
    ];
    let climb = Scenario {
        name: "ascending-gates",
        lap: RacingGateLap3D::open(climb_gates)?,
        params: QuadrotorParams::default(),
        start: QuadrotorState::at(0.0, 0.0, 0.0),
        max_steps: 200,
        target_laps: 1,
    };

    // 3. Closed square lap: the canonical lap, now with attitude dynamics.
    let s = 2.4;
    let square_gates = vec![
        gate([s, 0.0, 0.0], [0.0, 1.0, 0.0])?,
        gate([0.0, s, 0.0], [-1.0, 0.0, 0.0])?,
        gate([-s, 0.0, 0.0], [0.0, -1.0, 0.0])?,
        gate([0.0, -s, 0.0], [1.0, 0.0, 0.0])?,
    ];
    let square = Scenario {
        name: "square-lap",
        lap: RacingGateLap3D::closed_loop(square_gates)?,
        params: QuadrotorParams::default(),
        start: QuadrotorState::at(s, -1.5, 0.0),
        max_steps: 320,
        target_laps: 1,
    };

    // 4. Heavy agile drone: stronger gravity and more drag on the same slalom,
    //    so the controller must tilt harder and flies the course slower.
    let heavy_gates = vec![
        gate([1.6, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([3.2, -0.7, 0.0], [1.0, -0.5, 0.0])?,
        gate([4.8, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([6.4, -0.7, 0.0], [1.0, -0.5, 0.0])?,
    ];
    let heavy = Scenario {
        name: "heavy-slalom",
        lap: RacingGateLap3D::open(heavy_gates)?,
        params: QuadrotorParams::new(12.0, 0.6, 0.0, 26.0, 6.0, 5.0)?,
        start: QuadrotorState::at(0.0, 0.0, 0.0),
        max_steps: 240,
        target_laps: 1,
    };

    Ok(vec![slalom, climb, square, heavy])
}

struct Row {
    name: &'static str,
    gate_count: usize,
    report: QuadrotorLapReport,
}

fn lap_progress(report: &QuadrotorLapReport) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let mut rows = Vec::new();
    for scenario in scenarios()? {
        let gate_count = scenario.lap.gate_count();
        let report = simulate_quadrotor_race(
            QuadrotorMppiConfig::default(),
            scenario.params,
            &scenario.lap,
            scenario.start,
            scenario.max_steps,
            scenario.target_laps,
        )?;
        rows.push(Row {
            name: scenario.name,
            gate_count,
            report,
        });
    }
    Ok(rows)
}

fn format_lap_time(time: Option<f64>) -> String {
    match time {
        Some(seconds) => format!("{seconds:.1}s"),
        None => "-".to_string(),
    }
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,gates,gates_passed,laps_completed,lap_fraction,first_lap_time,mean_speed,max_speed,min_aperture_margin,mean_tilt_deg,max_tilt_deg,mean_body_rate,path_length\n",
    );
    for row in rows {
        let r = &row.report;
        let first_lap = r
            .first_lap_time
            .map(|t| format!("{t:.2}"))
            .unwrap_or_default();
        let _ = writeln!(
            csv,
            "{},{},{},{},{:.3},{},{:.3},{:.3},{:.3},{:.2},{:.2},{:.3},{:.3}",
            row.name,
            row.gate_count,
            r.gates_passed,
            r.laps_completed,
            r.lap_fraction,
            first_lap,
            r.mean_speed,
            r.max_speed,
            r.min_aperture_margin,
            r.mean_tilt.to_degrees(),
            r.max_tilt.to_degrees(),
            r.mean_body_rate,
            r.path_length
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let width = 820.0;
    let height = 400.0;
    let left = 70.0;
    let bottom = 300.0;
    let plot_width = 680.0;
    let plot_height = bottom - 100.0;
    let max_progress = rows
        .iter()
        .map(|row| lap_progress(&row.report))
        .fold(1.0_f64, f64::max)
        .max(1.0);

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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Quadrotor racing MPPI: lap progress and attitude by course</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">thrust + body-rate inputs; bar = laps + lap fraction; labels show mean speed, first-lap time, and peak tilt</text>"
    );

    for step in 0..=2 {
        let value = max_progress * step as f64 / 2.0;
        let y = bottom - (value / max_progress) * plot_height;
        let _ = writeln!(
            svg,
            "<line x1=\"{left}\" y1=\"{y:.1}\" x2=\"{:.1}\" y2=\"{y:.1}\" stroke=\"#e5e5ea\" stroke-width=\"1\"/>",
            left + plot_width
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"end\">{value:.1}</text>",
            left - 8.0,
            y + 4.0
        );
    }

    let group_width = plot_width / rows.len() as f64;
    let bar_width = group_width * 0.42;
    for (index, row) in rows.iter().enumerate() {
        let r = &row.report;
        let progress = lap_progress(r);
        let center = left + index as f64 * group_width + group_width / 2.0;
        let bar_height = (progress / max_progress) * plot_height;
        let bar_x = center - bar_width / 2.0;
        let bar_y = bottom - bar_height;
        let completed = r.laps_completed >= 1 || r.gates_passed >= row.gate_count;
        let color = if completed { "#0a84ff" } else { "#ff9f0a" };
        let _ = writeln!(
            svg,
            "<rect x=\"{bar_x:.1}\" y=\"{bar_y:.1}\" width=\"{bar_width:.1}\" height=\"{bar_height:.1}\" fill=\"{color}\" rx=\"3\"/>"
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{progress:.2}</text>",
            bar_y - 8.0
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            row.name
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">{}/{} gates · {:.1} m/s</text>",
            bottom + 38.0,
            r.gates_passed,
            row.gate_count,
            r.mean_speed
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">{} · peak tilt {:.0}°</text>",
            bottom + 54.0,
            format_lap_time(r.first_lap_time),
            r.max_tilt.to_degrees()
        );
    }

    // Legend.
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"70\" width=\"14\" height=\"14\" fill=\"#0a84ff\" rx=\"3\"/>",
        left + plot_width - 180.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"82\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">finished the course / lap</text>",
        left + plot_width - 160.0
    );
    let _ = writeln!(
        svg,
        "<rect x=\"{}\" y=\"90\" width=\"14\" height=\"14\" fill=\"#ff9f0a\" rx=\"3\"/>",
        left + plot_width - 180.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{}\" y=\"102\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">partial progress</text>",
        left + plot_width - 160.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("quadrotor racing MPPI (thrust + body-rate, reference-free lap progress)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<15} gates={}/{} laps={}+{:.2} first_lap={} mean_speed={:.2} peak_tilt={:.0}deg mean_rate={:.2} steps={}",
            row.name,
            r.gates_passed,
            row.gate_count,
            r.laps_completed,
            r.lap_fraction,
            format_lap_time(r.first_lap_time),
            r.mean_speed,
            r.max_tilt.to_degrees(),
            r.mean_body_rate,
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
        let first = collect_rows().unwrap();
        let second = collect_rows().unwrap();
        for (a, b) in first.iter().zip(&second) {
            assert_eq!(a.report.path, b.report.path);
            assert_eq!(a.report.gates_passed, b.report.gates_passed);
        }
    }

    #[test]
    fn every_course_makes_progress() {
        for row in collect_rows().unwrap() {
            assert!(row.report.gates_passed >= 1, "{} passed no gates", row.name);
        }
    }

    #[test]
    fn flying_requires_tilt() {
        for row in collect_rows().unwrap() {
            assert!(
                row.report.max_tilt > 0.05,
                "{} never tilted (max_tilt {})",
                row.name,
                row.report.max_tilt
            );
        }
    }
}
