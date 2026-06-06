//! Charge-budgeted powertrain-aware racing MPPI on a draining multi-lap course.
//!
//! All four runs fly the same closed square lap through the same fast-draining
//! powertrain, starting from a 70% pack. They differ only in the charge-budget
//! weight handed to the powertrain-aware controller: a step's rollout cost gains
//! `weight * max(0, reserve - soc)`, so a heavier weight makes the controller
//! ease off as the pack nears the protected reserve.
//!
//! The sweep traces the energy/progress trade-off as one Pareto frontier: with
//! no budget the controller flies hard, completes the most lap progress, and
//! drains the pack to empty; as the budget weight rises it eases off below the
//! reserve, flies slower, and ends with progressively more charge in reserve at
//! the cost of lap progress. (On a hover-dominated quad with no regeneration,
//! pacing buys reserve, not extra laps — covering distance faster is the more
//! energy-efficient way to spend a fixed pack.) Results go to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_powertrain_race_budgeted, ChargeBudget, MotorMppiConfig, MotorQuadParams,
    PowertrainLapReport, PowertrainParams, PowertrainState, RacingGateLap3D, RacingGatePlane3D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-powertrain-budget.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-powertrain-budget.svg";

const HALF: f64 = 0.9;
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const MAX_STEPS: usize = 240;
const TARGET_LAPS: usize = 8;
const RESERVE: f64 = 0.4;
const START_SOC: f64 = 0.7;
const WEIGHTS: [f64; 4] = [0.0, 50.0, 150.0, 400.0];

fn config() -> MotorMppiConfig {
    MotorMppiConfig {
        horizon: 22,
        samples: 450,
        dt: 0.05,
        rotor_sigma: 2.6,
        ..MotorMppiConfig::default()
    }
}

fn gate(center: [f64; 3], normal: [f64; 3]) -> RoboticsResult<RacingGatePlane3D> {
    RacingGatePlane3D::new(center, normal, UP, HALF, HALF)
}

fn square_lap() -> RoboticsResult<RacingGateLap3D> {
    let s = 2.0;
    let gates = vec![
        gate([s, 0.0, 0.0], [0.0, 1.0, 0.0])?,
        gate([0.0, s, 0.0], [-1.0, 0.0, 0.0])?,
        gate([-s, 0.0, 0.0], [0.0, -1.0, 0.0])?,
        gate([0.0, -s, 0.0], [1.0, 0.0, 0.0])?,
    ];
    RacingGateLap3D::closed_loop(gates)
}

fn powertrain() -> RoboticsResult<PowertrainParams> {
    // A fast-draining pack so the multi-lap energy budget bites within the run.
    PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.12, 0.18, 0.55)
}

struct Row {
    weight: f64,
    gate_count: usize,
    report: PowertrainLapReport,
}

fn lap_progress(report: &PowertrainLapReport) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let lap = square_lap()?;
    let gate_count = lap.gate_count();
    let params = powertrain()?;
    let start = PowertrainState::at_soc(2.0, -1.5, 0.0, params.base.gravity, START_SOC);
    let mut rows = Vec::new();
    for &weight in &WEIGHTS {
        let report = simulate_powertrain_race_budgeted(
            config(),
            params,
            ChargeBudget::new(weight, RESERVE),
            &lap,
            start,
            MAX_STEPS,
            TARGET_LAPS,
        )?;
        rows.push(Row {
            weight,
            gate_count,
            report,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "charge_weight,gates,gates_passed,laps_completed,lap_fraction,mean_speed,ceiling_saturation,min_battery_soc,final_battery_soc,steps\n",
    );
    for row in rows {
        let r = &row.report;
        let _ = writeln!(
            csv,
            "{:.0},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{}",
            row.weight,
            row.gate_count,
            r.gates_passed,
            r.laps_completed,
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

const COLORS: [&str; 4] = ["#ff453a", "#ff9f0a", "#30b0c7", "#0a84ff"];

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
        "<text x=\"40\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Charge-budget sweep: pacing a draining pack over a multi-lap race</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"40\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">left: lap progress (bar) with final-charge labels; right: battery state of charge over the run (darker = heavier budget)</text>"
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
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\" text-anchor=\"middle\">{progress:.2}</text>",
            bar_y - 8.0
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">w={:.0}</text>",
            bottom + 22.0,
            row.weight
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\" text-anchor=\"middle\">end soc {:.0}%</text>",
            bottom + 38.0,
            r.final_battery_soc * 100.0
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
    // Reserve line.
    let reserve_y = rbottom - RESERVE * rheight;
    let _ = writeln!(
        svg,
        "<line x1=\"{rleft}\" y1=\"{reserve_y:.1}\" x2=\"{:.1}\" y2=\"{reserve_y:.1}\" stroke=\"#c7c7cc\" stroke-width=\"1\" stroke-dasharray=\"4 3\"/>",
        rleft + rwidth
    );
    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#8e8e93\">reserve</text>",
        rleft + 4.0,
        reserve_y - 4.0
    );
    for step in 0..=4 {
        let frac = step as f64 / 4.0;
        let y = rbottom - frac * rheight;
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
    for (index, row) in rows.iter().enumerate() {
        let ly = rtop + 14.0 + index as f64 * 16.0;
        let _ = writeln!(
            svg,
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"12\" height=\"12\" fill=\"{}\" rx=\"2\"/>",
            rleft + rwidth - 96.0,
            ly - 10.0,
            COLORS[index % COLORS.len()]
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#1d1d1f\">w={:.0}</text>",
            rleft + rwidth - 80.0,
            ly,
            row.weight
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
    println!("charge-budget sweep (powertrain-aware MPPI on a draining multi-lap course)");
    for row in rows {
        let r = &row.report;
        println!(
            "  w={:<4.0} laps={}+{:.2} gates={}/{} mean_speed={:.2} ceil_sat={:.0}% min_soc={:.2} final_soc={:.2} steps={}",
            row.weight,
            r.laps_completed,
            r.lap_fraction,
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
            assert_eq!(a.report.battery_trace, b.report.battery_trace);
            assert_eq!(a.report.gates_passed, b.report.gates_passed);
        }
    }

    #[test]
    fn heavier_budget_ends_with_more_charge() {
        let rows = collect_rows().unwrap();
        let greedy = rows.iter().find(|r| r.weight == 0.0).unwrap();
        let heaviest = rows
            .iter()
            .max_by(|a, b| a.weight.total_cmp(&b.weight))
            .unwrap();
        assert!(
            heaviest.report.final_battery_soc > greedy.report.final_battery_soc,
            "budgeted end soc {} should exceed greedy {}",
            heaviest.report.final_battery_soc,
            greedy.report.final_battery_soc
        );
    }
}
