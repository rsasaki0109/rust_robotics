//! Reference-free racing MPPI through 3-D gate planes.
//!
//! Each scenario flies a seeded, deterministic MPPI drone through a course of
//! 3-D gate planes — planar closed laps, vertically undulating closed laps, and
//! open climbing courses — reporting lap-progress metrics: laps completed, the
//! fraction of the current lap, first-lap time, mean speed, and the minimum
//! aperture margin at any gate crossing. Results are written to CSV and SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_lap_race, RacingDroneDynamics3D, RacingDroneState3D, RacingGateLap3D,
    RacingGatePlane3D, RacingLapReport3D, RacingMppi3DConfig,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-mppi-3d.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-mppi-3d.svg";

const HALF: f64 = 0.8;
const UP: [f64; 3] = [0.0, 0.0, 1.0];

struct Scenario {
    name: &'static str,
    lap: RacingGateLap3D,
    dynamics: RacingDroneDynamics3D,
    start: RacingDroneState3D,
    max_steps: usize,
    target_laps: usize,
}

/// A gate centered at `center` facing direction `normal` (need not be unit).
fn gate(center: [f64; 3], normal: [f64; 3]) -> RoboticsResult<RacingGatePlane3D> {
    RacingGatePlane3D::new(center, normal, UP, HALF, HALF)
}

/// Closed square lap of edge-midpoint gates, with an optional per-gate height.
fn square_lap(heights: [f64; 4]) -> RoboticsResult<RacingGateLap3D> {
    let s = 2.2;
    let gates = vec![
        gate([s, 0.0, heights[0]], [0.0, 1.0, 0.0])?,
        gate([0.0, s, heights[1]], [-1.0, 0.0, 0.0])?,
        gate([-s, 0.0, heights[2]], [0.0, -1.0, 0.0])?,
        gate([0.0, -s, heights[3]], [1.0, 0.0, 0.0])?,
    ];
    RacingGateLap3D::closed_loop(gates)
}

fn scenarios() -> RoboticsResult<Vec<Scenario>> {
    // 1. Planar closed lap: two full laps around a flat square.
    let planar = Scenario {
        name: "planar-square",
        lap: square_lap([0.0, 0.0, 0.0, 0.0])?,
        dynamics: RacingDroneDynamics3D::default(),
        start: RacingDroneState3D::at(2.2, -1.4, 0.0),
        max_steps: 150,
        target_laps: 2,
    };

    // 2. Undulating closed lap: the same square, but gates rise and fall, so the
    //    drone must climb and descend each lap.
    let wavy = Scenario {
        name: "wavy-square",
        lap: square_lap([0.0, 0.9, 0.0, 0.9])?,
        dynamics: RacingDroneDynamics3D::default(),
        start: RacingDroneState3D::at(2.2, -1.4, 0.0),
        max_steps: 170,
        target_laps: 2,
    };

    // 3. Open climbing course: five gates spiralling upward.
    let climb_gates = vec![
        gate([1.2, 0.0, 0.0], [1.0, 0.3, 0.4])?,
        gate([2.2, 0.8, 0.5], [1.0, 0.8, 0.5])?,
        gate([2.8, 1.8, 1.1], [0.4, 1.0, 0.6])?,
        gate([2.4, 2.9, 1.7], [-0.4, 1.0, 0.6])?,
        gate([1.4, 3.5, 2.3], [-1.0, 0.4, 0.6])?,
    ];
    let climb = Scenario {
        name: "ascending-helix",
        lap: RacingGateLap3D::open(climb_gates)?,
        dynamics: RacingDroneDynamics3D::default(),
        start: RacingDroneState3D::at(0.0, 0.0, 0.0),
        max_steps: 90,
        target_laps: 1,
    };

    // 4. High-drag agile slalom: gates weave in y while climbing in z, flown by
    //    a draggier, lower-top-speed drone.
    let slalom_gates = vec![
        gate([1.0, 0.8, 0.2], [1.0, 0.8, 0.2])?,
        gate([2.0, -0.8, 0.6], [1.0, -0.8, 0.4])?,
        gate([3.0, 0.8, 1.0], [1.0, 0.8, 0.4])?,
        gate([4.0, -0.8, 1.4], [1.0, -0.8, 0.4])?,
    ];
    let slalom = Scenario {
        name: "drafty-slalom",
        lap: RacingGateLap3D::open(slalom_gates)?,
        dynamics: RacingDroneDynamics3D::new(0.7, 0.0, 4.0, 8.0)?,
        start: RacingDroneState3D::at(0.0, 0.0, 0.0),
        max_steps: 90,
        target_laps: 1,
    };

    Ok(vec![planar, wavy, climb, slalom])
}

struct Row {
    name: &'static str,
    gate_count: usize,
    report: RacingLapReport3D,
}

fn lap_progress(report: &RacingLapReport3D) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let mut rows = Vec::new();
    for scenario in scenarios()? {
        let gate_count = scenario.lap.gate_count();
        let report = simulate_lap_race(
            RacingMppi3DConfig::default(),
            scenario.dynamics,
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
        "scenario,gates,gates_passed,laps_completed,lap_fraction,first_lap_time,mean_speed,max_speed,min_aperture_margin,path_length,mean_control_effort\n",
    );
    for row in rows {
        let r = &row.report;
        let first_lap = r
            .first_lap_time
            .map(|t| format!("{t:.2}"))
            .unwrap_or_else(|| "".to_string());
        let _ = writeln!(
            csv,
            "{},{},{},{},{:.3},{},{:.3},{:.3},{:.3},{:.3},{:.3}",
            row.name,
            row.gate_count,
            r.gates_passed,
            r.laps_completed,
            r.lap_fraction,
            first_lap,
            r.mean_speed,
            r.max_speed,
            r.min_aperture_margin,
            r.path_length,
            r.mean_control_effort
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let width = 780.0;
    let height = 380.0;
    let left = 70.0;
    let bottom = 296.0;
    let plot_width = 640.0;
    let plot_height = bottom - 96.0;
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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Racing MPPI through 3-D gates: lap progress by course</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">reference-free gate progress; bar = laps completed + lap fraction; labels show mean speed and first-lap time</text>"
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
        let completed_full_lap = r.laps_completed >= 1;
        let color = if completed_full_lap {
            "#0a84ff"
        } else {
            "#ff9f0a"
        };
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
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">lap {} · margin {:.2}</text>",
            bottom + 54.0,
            format_lap_time(r.first_lap_time),
            r.min_aperture_margin
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
        "<text x=\"{}\" y=\"82\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\">completed a lap / course</text>",
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
    println!("racing MPPI through 3-D gate planes (reference-free lap progress)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<15} gates={}/{} laps={}+{:.2} first_lap={} mean_speed={:.2} max_speed={:.2} margin={:.2} steps={}",
            row.name,
            r.gates_passed,
            row.gate_count,
            r.laps_completed,
            r.lap_fraction,
            format_lap_time(r.first_lap_time),
            r.mean_speed,
            r.max_speed,
            r.min_aperture_margin,
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
    fn planar_square_completes_a_lap() {
        let rows = collect_rows().unwrap();
        let planar = rows.iter().find(|row| row.name == "planar-square").unwrap();
        assert!(
            planar.report.laps_completed >= 1,
            "planar lap completed {} laps",
            planar.report.laps_completed
        );
        assert!(planar.report.first_lap_time.is_some());
    }
}
