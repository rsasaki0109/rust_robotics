//! Charge budget x battery recovery: when does pacing actually buy laps?
//!
//! The charge-budget sweep showed that on a pack with no recovery, pacing only
//! buys reserve, not laps — covering distance faster is the more efficient way to
//! spend a fixed pack. The battery-recovery model changes the physics: flying
//! hard pins the relaxation overpotential high, which collapses the thrust
//! ceiling, so a greedy drone *loses flyability at a higher state of charge* and
//! strands usable charge it can no longer reach. A paced drone keeps the
//! overpotential low, holds its ceiling, and flies the pack down further.
//!
//! This benchmark runs the 2x2 of {greedy, budgeted} x {recovery off, recovery
//! on} on the same draining, undulating multi-lap square, so the budget's value
//! can be read against the recovery model. The honest headline is a *reversal of
//! sign*, not a clean lap win: with no recovery the budget is strictly worse (it
//! gives up a full lap), but once the pack recovers, the budget pulls *even* on
//! laps while flying faster and ending with more charge in reserve — pacing goes
//! from a clear loss to a Pareto win. A strict "more laps" flip stays out of
//! reach here because on a *climbing* course the same throttling that lowers the
//! relaxation overpotential also limits the thrust needed to climb to the high
//! gates. Results go to CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{
    simulate_powertrain_race_budgeted, ChargeBudget, MotorMppiConfig, MotorQuadParams,
    PowertrainLapReport, PowertrainParams, PowertrainState, RacingGateLap3D, RacingGatePlane3D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-powertrain-endurance.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-powertrain-endurance.svg";

const HALF: f64 = 0.9;
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const MAX_STEPS: usize = 260;
const TARGET_LAPS: usize = 8;
const START_SOC: f64 = 0.7;
const RESERVE: f64 = 0.45;
const BUDGET_WEIGHT: f64 = 180.0;

fn config() -> MotorMppiConfig {
    MotorMppiConfig {
        horizon: 20,
        samples: 380,
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
        gate([s, 0.0, 1.4], [0.0, 1.0, 0.0])?,
        gate([0.0, s, 0.0], [-1.0, 0.0, 0.0])?,
        gate([-s, 0.0, 1.4], [0.0, -1.0, 0.0])?,
        gate([0.0, -s, 0.0], [1.0, 0.0, 0.0])?,
    ];
    RacingGateLap3D::closed_loop(gates)
}

fn base_powertrain() -> RoboticsResult<PowertrainParams> {
    PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.1, 0.15, 0.48)
}

fn recovery_powertrain() -> RoboticsResult<PowertrainParams> {
    Ok(base_powertrain()?.with_recovery(0.7, 1.1, 0.32))
}

struct Row {
    name: &'static str,
    recovery: bool,
    budgeted: bool,
    gate_count: usize,
    report: PowertrainLapReport,
}

fn lap_progress(report: &PowertrainLapReport) -> f64 {
    report.laps_completed as f64 + report.lap_fraction
}

fn collect_rows() -> RoboticsResult<Vec<Row>> {
    let lap = square_lap()?;
    let gate_count = lap.gate_count();
    let mut rows = Vec::new();
    let specs = [
        ("greedy, no-recovery", false, false),
        ("budget, no-recovery", false, true),
        ("greedy, recovery", true, false),
        ("budget, recovery", true, true),
    ];
    for (name, recovery, budgeted) in specs {
        let params = if recovery {
            recovery_powertrain()?
        } else {
            base_powertrain()?
        };
        let budget = if budgeted {
            ChargeBudget::new(BUDGET_WEIGHT, RESERVE)
        } else {
            ChargeBudget::none()
        };
        let start = PowertrainState::at_soc(2.0, -1.5, 0.0, params.base.gravity, START_SOC);
        let report = simulate_powertrain_race_budgeted(
            config(),
            params,
            budget,
            &lap,
            start,
            MAX_STEPS,
            TARGET_LAPS,
        )?;
        rows.push(Row {
            name,
            recovery,
            budgeted,
            gate_count,
            report,
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "scenario,recovery,budgeted,gates,gates_passed,laps,mean_speed,mean_voltage_scale,ceiling_saturation,min_battery_soc,final_battery_soc,steps\n",
    );
    for row in rows {
        let r = &row.report;
        let _ = writeln!(
            csv,
            "{},{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{}",
            row.name,
            row.recovery,
            row.budgeted,
            row.gate_count,
            r.gates_passed,
            lap_progress(r),
            r.mean_speed,
            r.mean_voltage_scale,
            r.ceiling_saturation_fraction,
            r.min_battery_soc,
            r.final_battery_soc,
            r.steps
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let width = 900.0;
    let height = 420.0;
    let left = 70.0;
    let bottom = 310.0;
    let plot_width = 760.0;
    let plot_height = bottom - 110.0;
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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Charge budget x battery recovery: recovery flips pacing from a loss to a Pareto win</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">bar = lap progress; blue = budgeted, orange = greedy; labels show end charge and mean voltage scale</text>"
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
    let bar_width = group_width * 0.46;
    for (index, row) in rows.iter().enumerate() {
        let r = &row.report;
        let progress = lap_progress(r);
        let center = left + index as f64 * group_width + group_width / 2.0;
        let bar_height = (progress / max_progress) * plot_height;
        let bar_x = center - bar_width / 2.0;
        let bar_y = bottom - bar_height;
        let color = if row.budgeted { "#0a84ff" } else { "#ff9f0a" };
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
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d1d1f\" text-anchor=\"middle\">{}</text>",
            bottom + 22.0,
            row.name
        );
        let _ = writeln!(
            svg,
            "<text x=\"{center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\" text-anchor=\"middle\">end soc {:.0}% · v {:.2}</text>",
            bottom + 38.0,
            r.final_battery_soc * 100.0,
            r.mean_voltage_scale
        );
    }

    // Annotate the two regimes.
    let no_rec_center = left + 1.0 * group_width;
    let rec_center = left + 3.0 * group_width;
    let _ = writeln!(
        svg,
        "<text x=\"{no_rec_center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#ff9f0a\" text-anchor=\"middle\">no recovery: budget gives up a lap</text>",
        bottom + 58.0
    );
    let _ = writeln!(
        svg,
        "<text x=\"{rec_center:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#0a84ff\" text-anchor=\"middle\">recovery: budget ties on laps, faster, more reserve</text>",
        bottom + 58.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(rows: &[Row]) {
    println!("charge budget x battery recovery (draining multi-lap square)");
    for row in rows {
        let r = &row.report;
        println!(
            "  {:<22} laps={:.2} gates={}/{} mean_speed={:.2} v_scale={:.2} ceil_sat={:.0}% final_soc={:.2} steps={}",
            row.name,
            lap_progress(r),
            r.gates_passed,
            row.gate_count,
            r.mean_speed,
            r.mean_voltage_scale,
            r.ceiling_saturation_fraction * 100.0,
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

    fn find<'a>(rows: &'a [Row], name: &str) -> &'a Row {
        rows.iter().find(|r| r.name == name).unwrap()
    }

    #[test]
    fn recovery_reverses_the_sign_of_the_budget() {
        let rows = collect_rows().unwrap();
        let greedy_off = find(&rows, "greedy, no-recovery");
        let budget_off = find(&rows, "budget, no-recovery");
        let greedy_on = find(&rows, "greedy, recovery");
        let budget_on = find(&rows, "budget, recovery");

        // Every configuration flies at least one lap.
        for row in &rows {
            assert!(lap_progress(&row.report) >= 1.0, "{} did not lap", row.name);
        }

        // Without recovery, pacing is strictly worse — it gives up lap progress.
        assert!(
            lap_progress(&budget_off.report) < lap_progress(&greedy_off.report),
            "no-recovery: budget {} should trail greedy {}",
            lap_progress(&budget_off.report),
            lap_progress(&greedy_off.report)
        );

        // Recovery depresses the ceiling, so it costs the greedy run laps.
        assert!(
            lap_progress(&greedy_on.report) < lap_progress(&greedy_off.report),
            "recovery should cost greedy laps: {} vs {}",
            lap_progress(&greedy_on.report),
            lap_progress(&greedy_off.report)
        );

        // With recovery the budget no longer costs laps, and it becomes a Pareto
        // win over greedy: at least even on laps, but faster and with more charge.
        assert!(
            lap_progress(&budget_on.report) >= lap_progress(&greedy_on.report),
            "recovery: budget {} should not trail greedy {}",
            lap_progress(&budget_on.report),
            lap_progress(&greedy_on.report)
        );
        assert!(
            budget_on.report.mean_speed > greedy_on.report.mean_speed,
            "recovery: budget should fly faster ({} vs {})",
            budget_on.report.mean_speed,
            greedy_on.report.mean_speed
        );
        assert!(
            budget_on.report.final_battery_soc >= greedy_on.report.final_battery_soc,
            "recovery: budget should end with at least as much charge ({} vs {})",
            budget_on.report.final_battery_soc,
            greedy_on.report.final_battery_soc
        );
    }
}
