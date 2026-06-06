//! Reference-free racing MPPI through a motor-lag and battery-sag powertrain.
//!
//! Every scenario flies the *same* seeded MPPI controller down the *same*
//! slalom; only the powertrain underneath changes. The controller plans rotor
//! commands assuming ideal actuators, then those commands are executed through
//! a powertrain with first-order motor lag and a battery whose thrust ceiling
//! droops with load and state of charge. Isolating one effect at a time:
//!
//! - `ideal`        — no lag, no battery droop (reduces to the motor model).
//! - `motor-lag`    — first-order spin-up lag only.
//! - `lag+battery`  — lag plus a fresh, full battery that sags under load.
//! - `drained-pack` — the same powertrain starting at 25% state of charge, so
//!   the same plan keeps hitting a lowered thrust ceiling.
//!
//! The benchmark reports lap progress, the fraction of steps that hit the
//! battery ceiling, and the battery trace to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_powertrain_race, MotorMppiConfig, MotorQuadParams, PowertrainLapReport,
    PowertrainParams, PowertrainState, RacingGateLap3D, RacingGatePlane3D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-powertrain.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-powertrain.svg";

const HALF: f64 = 0.9;
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const MAX_STEPS: usize = 170;

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

struct Scenario {
    name: &'static str,
    params: PowertrainParams,
    start_soc: f64,
}

fn scenarios() -> RoboticsResult<Vec<Scenario>> {
    let base = MotorQuadParams::default();
    Ok(vec![
        Scenario {
            name: "ideal",
            params: PowertrainParams::ideal(base),
            start_soc: 1.0,
        },
        Scenario {
            name: "motor-lag",
            params: PowertrainParams::new(base, 0.08, 0.0, 0.0, 1.0)?,
            start_soc: 1.0,
        },
        Scenario {
            name: "lag+battery",
            params: PowertrainParams::new(base, 0.08, 0.06, 0.18, 0.55)?,
            start_soc: 1.0,
        },
        Scenario {
            name: "drained-pack",
            params: PowertrainParams::new(base, 0.08, 0.06, 0.18, 0.55)?,
            start_soc: 0.25,
        },
    ])
}

struct Row {
    name: &'static str,
    gate_count: usize,
    report: PowertrainLapReport,
}

fn lap_progress(report: &PowertrainLapReport) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let lap = slalom()?;
    let gate_count = lap.gate_count();
    let base = MotorQuadParams::default();
    let mut rows = Vec::new();
    for scenario in scenarios()? {
        let start = PowertrainState::at_soc(0.0, 0.0, 0.0, base.gravity, scenario.start_soc);
        let report =
            simulate_powertrain_race(config(), scenario.params, &lap, start, MAX_STEPS, 1)?;
        rows.push(Row {
            name: scenario.name,
            gate_count,
            report,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,gates,gates_passed,lap_fraction,mean_speed,ceiling_saturation,min_battery_soc,final_battery_soc,mean_voltage_scale,max_tilt_deg,steps\n",
    );
    for row in rows {
        let r = &row.report;
        let _ = writeln!(
            csv,
            "{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{}",
            row.name,
            row.gate_count,
            r.gates_passed,
            r.lap_fraction,
            r.mean_speed,
            r.ceiling_saturation_fraction,
            r.min_battery_soc,
            r.final_battery_soc,
            r.mean_voltage_scale,
            r.max_tilt.to_degrees(),
            r.steps
        );
    }
    csv
}

const COLORS: [&str; 4] = ["#0a84ff", "#30b0c7", "#ff9f0a", "#ff453a"];

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
        "<text x=\"40\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Motor-lag and battery-sag powertrain: same plan, four powertrains</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"40\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">left: lap progress (bar) with ceiling-saturation labels; right: battery state of charge over the run</text>"
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
            COLORS[index % COLORS.len()]
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
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
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
    for (index, row) in rows.iter().enumerate() {
        let trace = &row.report.battery_trace;
        if trace.is_empty() {
            continue;
        }
        let mut points = String::new();
        for (i, &soc) in trace.iter().enumerate() {
            let x = rleft + (i as f64 / (max_len - 1).max(1) as f64) * rwidth;
            let y = rbottom - soc.clamp(0.0, 1.0) * rheight;
            let _ = write!(points, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"2\"/>",
            points.trim(),
            COLORS[index % COLORS.len()]
        );
    }
    // Legend for the trace panel.
    for (index, row) in rows.iter().enumerate() {
        let ly = rtop + 14.0 + index as f64 * 16.0;
        let _ = writeln!(
            svg,
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"12\" height=\"12\" fill=\"{}\" rx=\"2\"/>",
            rleft + rwidth - 120.0,
            ly - 10.0,
            COLORS[index % COLORS.len()]
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#1d1d1f\">{}</text>",
            rleft + rwidth - 104.0,
            ly,
            row.name
        );
    }
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">battery state of charge vs step</text>",
        rleft + rwidth / 2.0,
        rbottom + 22.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("powertrain racing MPPI (motor lag + battery sag, reference-free lap progress)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<13} gates={}/{} mean_speed={:.2} ceil_sat={:.0}% min_soc={:.2} final_soc={:.2} v_scale={:.2} peak_tilt={:.0}deg steps={}",
            row.name,
            r.gates_passed,
            row.gate_count,
            r.mean_speed,
            r.ceiling_saturation_fraction * 100.0,
            r.min_battery_soc,
            r.final_battery_soc,
            r.mean_voltage_scale,
            r.max_tilt.to_degrees(),
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
    fn powertrain_effects_are_ordered() {
        let rows = collect_rows().unwrap();
        let find = |name| rows.iter().find(|r| r.name == name).unwrap();
        let ideal = find("ideal");
        let battery = find("lag+battery");
        let drained = find("drained-pack");

        // Every powertrain still threads at least one gate.
        for row in &rows {
            assert!(row.report.gates_passed >= 1, "{} passed no gates", row.name);
        }

        // The ideal powertrain never sags and never drains.
        assert_eq!(ideal.report.final_battery_soc, 1.0);
        assert_eq!(ideal.report.ceiling_saturation_fraction, 0.0);
        assert_eq!(ideal.report.mean_voltage_scale, 1.0);

        // Battery-enabled runs deplete the pack.
        assert!(battery.report.final_battery_soc < 1.0);
        assert!(drained.report.final_battery_soc < 0.25);

        // A drained pack runs at a lower terminal voltage throughout.
        assert!(
            drained.report.mean_voltage_scale < battery.report.mean_voltage_scale,
            "drained v_scale {} should be below fresh {}",
            drained.report.mean_voltage_scale,
            battery.report.mean_voltage_scale
        );

        // The ideal plan is realizable; the drained pack hits its ceiling more.
        assert!(
            drained.report.ceiling_saturation_fraction >= ideal.report.ceiling_saturation_fraction
        );
    }
}
