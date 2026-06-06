//! Powertrain-aware vs powertrain-unaware racing MPPI.
//!
//! Both controllers fly the same slalom through the same lagging, sagging
//! powertrain; the only difference is what each one plans against:
//!
//! - `unaware` rolls candidates out through the *ideal* motor model, so it keeps
//!   commanding thrust the battery cannot deliver and wastes authority against
//!   the ceiling.
//! - `aware` rolls candidates out through the full powertrain, so its samples
//!   already feel the motor lag and the battery-limited thrust ceiling. It plans
//!   within the authority the pack can actually supply.
//!
//! The contrast is run on a fresh full battery (where authority is ample and the
//! two are close) and on a pack drained to 25% (where authority is scarce and
//! the aware controller's budgeting matters). Results go to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_powertrain_race, simulate_powertrain_race_aware, MotorMppiConfig, MotorQuadParams,
    PowertrainLapReport, PowertrainParams, PowertrainState, RacingGateLap3D, RacingGatePlane3D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-powertrain-aware.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-powertrain-aware.svg";

const HALF: f64 = 0.9;
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const MAX_STEPS: usize = 180;

fn config() -> MotorMppiConfig {
    MotorMppiConfig {
        horizon: 24,
        samples: 500,
        dt: 0.05,
        rotor_sigma: 2.6,
        ..MotorMppiConfig::default()
    }
}

fn gate(center: [f64; 3], normal: [f64; 3]) -> RoboticsResult<RacingGatePlane3D> {
    RacingGatePlane3D::new(center, normal, UP, HALF, HALF)
}

fn slalom() -> RoboticsResult<RacingGateLap3D> {
    let gates = vec![
        gate([1.6, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([3.2, -0.7, 0.0], [1.0, -0.5, 0.0])?,
        gate([4.8, 0.7, 0.0], [1.0, 0.5, 0.0])?,
        gate([6.4, -0.7, 0.0], [1.0, -0.5, 0.0])?,
    ];
    RacingGateLap3D::open(gates)
}

fn racing_powertrain() -> RoboticsResult<PowertrainParams> {
    PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.06, 0.18, 0.55)
}

struct Scenario {
    name: &'static str,
    start_soc: f64,
    aware: bool,
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "fresh · unaware",
            start_soc: 1.0,
            aware: false,
        },
        Scenario {
            name: "fresh · aware",
            start_soc: 1.0,
            aware: true,
        },
        Scenario {
            name: "drained · unaware",
            start_soc: 0.25,
            aware: false,
        },
        Scenario {
            name: "drained · aware",
            start_soc: 0.25,
            aware: true,
        },
    ]
}

struct Row {
    name: &'static str,
    aware: bool,
    gate_count: usize,
    report: PowertrainLapReport,
}

fn lap_progress(report: &PowertrainLapReport) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let lap = slalom()?;
    let gate_count = lap.gate_count();
    let params = racing_powertrain()?;
    let base = params.base;
    let mut rows = Vec::new();
    for scenario in scenarios() {
        let start = PowertrainState::at_soc(0.0, 0.0, 0.0, base.gravity, scenario.start_soc);
        let report = if scenario.aware {
            simulate_powertrain_race_aware(config(), params, &lap, start, MAX_STEPS, 1)?
        } else {
            simulate_powertrain_race(config(), params, &lap, start, MAX_STEPS, 1)?
        };
        rows.push(Row {
            name: scenario.name,
            aware: scenario.aware,
            gate_count,
            report,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,aware,gates,gates_passed,lap_fraction,mean_speed,ceiling_saturation,min_battery_soc,final_battery_soc,steps\n",
    );
    for row in rows {
        let r = &row.report;
        let _ = writeln!(
            csv,
            "{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{}",
            row.name,
            row.aware,
            row.gate_count,
            r.gates_passed,
            r.lap_fraction,
            r.mean_speed,
            r.ceiling_saturation_fraction,
            r.min_battery_soc,
            r.final_battery_soc,
            r.steps
        );
    }
    csv
}

fn controller_color(aware: bool) -> &'static str {
    if aware {
        "#0a84ff"
    } else {
        "#ff9f0a"
    }
}

fn render_svg(rows: &[Row]) -> String {
    let width = 920.0;
    let height = 420.0;
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
        "<text x=\"40\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Powertrain-aware vs unaware MPPI: same lagging, sagging powertrain</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"40\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">left: lap progress with ceiling-saturation labels; right: battery state of charge (blue = aware, orange = unaware)</text>"
    );

    // --- Left panel: lap-progress bars -----------------------------------
    let left = 60.0;
    let bottom = 320.0;
    let plot_width = 360.0;
    let plot_height = bottom - 110.0;
    let max_progress = rows
        .iter()
        .map(|row| lap_progress(&row.report))
        .fold(1.0_f64, f64::max)
        .max(1.0);
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
    let bar_width = group_width * 0.5;
    for (index, row) in rows.iter().enumerate() {
        let r = &row.report;
        let progress = lap_progress(r);
        let center = left + index as f64 * group_width + group_width / 2.0;
        let bar_height = (progress / max_progress) * plot_height;
        let bar_x = center - bar_width / 2.0;
        let bar_y = bottom - bar_height;
        let _ = writeln!(
            svg,
            "<rect x=\"{bar_x:.1}\" y=\"{bar_y:.1}\" width=\"{bar_width:.1}\" height=\"{bar_height:.1}\" fill=\"{}\" rx=\"3\"/>",
            controller_color(row.aware)
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}/{}</text>",
            bar_y - 8.0,
            r.gates_passed,
            row.gate_count
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            row.name
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\" text-anchor=\"middle\">ceil {:.0}%</text>",
            bottom + 38.0,
            r.ceiling_saturation_fraction * 100.0
        );
    }

    // --- Right panel: battery traces -------------------------------------
    let rleft = 520.0;
    let rwidth = 360.0;
    let rtop = 90.0;
    let rbottom = 320.0;
    let rheight = rbottom - rtop;
    let _ = writeln!(
        svg,
        "<rect x=\"{rleft}\" y=\"{rtop}\" width=\"{rwidth}\" height=\"{rheight}\" fill=\"#ffffff\" stroke=\"#e5e5ea\"/>"
    );
    for step in 0..=4 {
        let frac = step as f64 / 4.0;
        let y = rbottom - frac * rheight;
        let _ = writeln!(
            svg,
            "<line x1=\"{rleft}\" y1=\"{y:.1}\" x2=\"{:.1}\" y2=\"{y:.1}\" stroke=\"#f0f0f2\" stroke-width=\"1\"/>",
            rleft + rwidth
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\" text-anchor=\"end\">{:.0}%</text>",
            rleft - 6.0,
            y + 3.0,
            frac * 100.0
        );
    }
    let max_len = rows
        .iter()
        .map(|row| row.report.battery_trace.len())
        .max()
        .unwrap_or(1)
        .max(1);
    for row in rows {
        let trace = &row.report.battery_trace;
        if trace.is_empty() {
            continue;
        }
        // Drained runs are solid, fresh runs dashed, so the four lines read
        // apart even where blue/orange overlap.
        let dash = if row.report.battery_trace.first().copied().unwrap_or(1.0) > 0.5 {
            " stroke-dasharray=\"5 3\""
        } else {
            ""
        };
        let mut points = String::new();
        for (i, &soc) in trace.iter().enumerate() {
            let x = rleft + (i as f64 / (max_len - 1).max(1) as f64) * rwidth;
            let y = rbottom - soc.clamp(0.0, 1.0) * rheight;
            let _ = write!(points, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"2\"{}/>",
            points.trim(),
            controller_color(row.aware),
            dash
        );
    }
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">battery state of charge vs step (dashed = fresh, solid = drained)</text>",
        rleft + rwidth / 2.0,
        rbottom + 22.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("powertrain-aware vs unaware racing MPPI (same lagging, sagging powertrain)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<18} gates={}/{} mean_speed={:.2} ceil_sat={:.0}% min_soc={:.2} final_soc={:.2} steps={}",
            row.name,
            r.gates_passed,
            row.gate_count,
            r.mean_speed,
            r.ceiling_saturation_fraction * 100.0,
            r.min_battery_soc,
            r.final_battery_soc,
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
            assert_eq!(a.report.battery_trace, b.report.battery_trace);
        }
    }

    #[test]
    fn aware_helps_on_a_drained_pack() {
        let rows = collect_rows().unwrap();
        let find = |name| rows.iter().find(|r| r.name == name).unwrap();
        let unaware = find("drained · unaware");
        let aware = find("drained · aware");

        // Both still make some progress.
        assert!(unaware.report.gates_passed >= 1);

        // Planning within the battery-limited authority gets the aware
        // controller strictly further on the drained pack — it finishes the
        // course where the unaware controller stalls.
        assert!(
            aware.report.gates_passed > unaware.report.gates_passed,
            "aware {} should beat unaware {} gates",
            aware.report.gates_passed,
            unaware.report.gates_passed
        );
        assert_eq!(
            aware.report.gates_passed, aware.gate_count,
            "aware should finish the drained course"
        );
        // ...and it conserves authority, ending with more charge in the pack.
        assert!(
            aware.report.final_battery_soc > unaware.report.final_battery_soc,
            "aware final soc {} should exceed unaware {}",
            aware.report.final_battery_soc,
            unaware.report.final_battery_soc
        );
    }
}
