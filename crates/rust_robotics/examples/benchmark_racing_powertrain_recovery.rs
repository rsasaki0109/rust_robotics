//! Battery-recovery (relaxation-overpotential) demonstration for the powertrain.
//!
//! This is a dynamics-only benchmark (no MPPI): it drives the powertrain through
//! a scripted load profile — alternating hard (full throttle) and rest (hover)
//! phases — and records the battery state of charge, the relaxation
//! overpotential, and the terminal-voltage scale at each step. Two powertrains
//! see the identical command sequence from the identical start: one with the
//! relaxation-recovery model enabled, one without.
//!
//! The point is the *slow* recovery. The instantaneous sag term relaxes the
//! moment the load drops, so both powertrains' terminal voltage jumps up at the
//! start of each rest. But with recovery enabled, the overpotential that built
//! up during the hard phase keeps the voltage depressed and then *climbs back*
//! across the rest as the overpotential decays — the pack recovers usable
//! authority even though its state of charge only ever falls. Results go to
//! CSV/SVG.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use rust_robotics::control::{MotorCommand, MotorQuadParams, PowertrainParams, PowertrainState};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/racing-powertrain-recovery.csv";
const SVG_OUTPUT: &str = "docs/assets/racing-powertrain-recovery.svg";

const DT: f64 = 0.05;
const PHASE_STEPS: usize = 60;
const PHASES: usize = 4;
const START_SOC: f64 = 0.85;

/// Hard (full throttle) on even phases, rest (hover) on odd phases.
fn command_for_step(step: usize, params: PowertrainParams) -> MotorCommand {
    let phase = step / PHASE_STEPS;
    if phase % 2 == 0 {
        MotorCommand::new([params.base.max_rotor_thrust; 4])
    } else {
        MotorCommand::hover(params.base.gravity)
    }
}

#[derive(Clone)]
struct Trace {
    soc: Vec<f64>,
    relaxation: Vec<f64>,
    terminal_voltage: Vec<f64>,
    load: Vec<f64>,
}

fn run(params: PowertrainParams) -> Trace {
    let mut state = PowertrainState::at_soc(0.0, 0.0, 1.0, params.base.gravity, START_SOC);
    let steps = PHASE_STEPS * PHASES;
    let mut trace = Trace {
        soc: Vec::with_capacity(steps),
        relaxation: Vec::with_capacity(steps),
        terminal_voltage: Vec::with_capacity(steps),
        load: Vec::with_capacity(steps),
    };
    for step in 0..steps {
        let command = command_for_step(step, params);
        state = params.step(state, command, DT);
        let load = params.load(state.rotor_thrust);
        trace.soc.push(state.battery_soc);
        trace.relaxation.push(state.relaxation);
        trace.terminal_voltage.push(params.terminal_voltage_scale(
            state.battery_soc,
            load,
            state.relaxation,
        ));
        trace.load.push(load);
    }
    trace
}

fn base_powertrain() -> RoboticsResult<PowertrainParams> {
    PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.06, 0.18, 0.5)
}

fn render_csv(no_recovery: &Trace, recovery: &Trace) -> String {
    let mut csv = String::from(
        "step,time,load,soc,terminal_voltage_no_recovery,terminal_voltage_recovery,relaxation_recovery\n",
    );
    for i in 0..recovery.soc.len() {
        let _ = writeln!(
            csv,
            "{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            i,
            i as f64 * DT,
            recovery.load[i],
            recovery.soc[i],
            no_recovery.terminal_voltage[i],
            recovery.terminal_voltage[i],
            recovery.relaxation[i],
        );
    }
    csv
}

fn render_svg(no_recovery: &Trace, recovery: &Trace) -> String {
    let width = 920.0;
    let height = 440.0;
    let left = 60.0;
    let right = 760.0;
    let top = 80.0;
    let bottom = 360.0;
    let plot_w = right - left;
    let plot_h = bottom - top;
    let steps = recovery.soc.len().max(1);
    let x_of = |i: usize| left + (i as f64 / (steps - 1).max(1) as f64) * plot_w;
    let y_of = |v: f64| bottom - v.clamp(0.0, 1.0) * plot_h;

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
        "<text x=\"{left}\" y=\"34\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#1d1d1f\">Battery recovery: terminal voltage relaxes back on ease-off as charge keeps falling</text>"
    );
    let _ = writeln!(
        svg,
        "<text x=\"{left}\" y=\"56\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">scripted hard/rest load profile; shaded = hard (full throttle) phases</text>"
    );

    // Hard-phase shading.
    for phase in (0..PHASES).step_by(2) {
        let x0 = x_of(phase * PHASE_STEPS);
        let x1 = x_of(((phase + 1) * PHASE_STEPS).min(steps - 1));
        let _ = writeln!(
            svg,
            "<rect x=\"{x0:.1}\" y=\"{top:.1}\" width=\"{:.1}\" height=\"{plot_h:.1}\" fill=\"#f0f0f2\"/>",
            (x1 - x0).max(0.0)
        );
    }

    // Gridlines.
    for step in 0..=4 {
        let frac = step as f64 / 4.0;
        let y = bottom - frac * plot_h;
        let _ = writeln!(
            svg,
            "<line x1=\"{left}\" y1=\"{y:.1}\" x2=\"{right}\" y2=\"{y:.1}\" stroke=\"#e5e5ea\" stroke-width=\"1\"/>"
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#6e6e73\" text-anchor=\"end\">{:.0}%</text>",
            left - 6.0,
            y + 3.0,
            frac * 100.0
        );
    }

    let polyline = |svg: &mut String, data: &[f64], color: &str, dash: &str| {
        let mut points = String::new();
        for (i, &v) in data.iter().enumerate() {
            let _ = write!(points, "{:.1},{:.1} ", x_of(i), y_of(v));
        }
        let _ = writeln!(
            svg,
            "<polyline points=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"2\"{dash}/>",
            points.trim()
        );
    };

    // State of charge (gray), terminal voltage without/with recovery, relaxation.
    polyline(&mut svg, &recovery.soc, "#8e8e93", "");
    polyline(&mut svg, &no_recovery.terminal_voltage, "#ff9f0a", "");
    polyline(&mut svg, &recovery.terminal_voltage, "#0a84ff", "");
    polyline(
        &mut svg,
        &recovery.relaxation,
        "#34c759",
        " stroke-dasharray=\"4 3\"",
    );

    // Legend.
    let legend = [
        ("#8e8e93", "state of charge"),
        ("#ff9f0a", "terminal voltage (no recovery)"),
        ("#0a84ff", "terminal voltage (recovery)"),
        ("#34c759", "relaxation overpotential"),
    ];
    for (i, (color, label)) in legend.iter().enumerate() {
        let ly = top + 6.0 + i as f64 * 18.0;
        let _ = writeln!(
            svg,
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"12\" height=\"12\" fill=\"{color}\" rx=\"2\"/>",
            right + 12.0,
            ly - 10.0
        );
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\">{label}</text>",
            right + 28.0,
            ly
        );
    }

    let _ = writeln!(
        svg,
        "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\" text-anchor=\"middle\">time [s]</text>",
        (left + right) / 2.0,
        bottom + 30.0
    );

    svg.push_str("</svg>\n");
    svg
}

fn print_summary(no_recovery: &Trace, recovery: &Trace) {
    // Sample the value at the end of the first rest phase to show recovery.
    let rest_end = (2 * PHASE_STEPS - 1).min(recovery.soc.len() - 1);
    let hard_end = (PHASE_STEPS - 1).min(recovery.soc.len() - 1);
    println!("battery recovery demo (scripted hard/rest load profile)");
    println!(
        "  end of first hard phase:  soc={:.2} relax={:.2} vterm_recovery={:.3} vterm_no_recovery={:.3}",
        recovery.soc[hard_end],
        recovery.relaxation[hard_end],
        recovery.terminal_voltage[hard_end],
        no_recovery.terminal_voltage[hard_end],
    );
    println!(
        "  end of first rest phase:  soc={:.2} relax={:.2} vterm_recovery={:.3} vterm_no_recovery={:.3}",
        recovery.soc[rest_end],
        recovery.relaxation[rest_end],
        recovery.terminal_voltage[rest_end],
        no_recovery.terminal_voltage[rest_end],
    );
}

fn main() -> RoboticsResult<()> {
    let base = base_powertrain()?;
    let recovery = base.with_recovery(0.9, 0.6, 0.22);
    let no_recovery_trace = run(base);
    let recovery_trace = run(recovery);
    print_summary(&no_recovery_trace, &recovery_trace);

    if let Some(parent) = Path::new(CSV_OUTPUT).parent() {
        fs::create_dir_all(parent).ok();
    }
    fs::write(CSV_OUTPUT, render_csv(&no_recovery_trace, &recovery_trace)).ok();
    fs::write(SVG_OUTPUT, render_svg(&no_recovery_trace, &recovery_trace)).ok();
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_is_deterministic() {
        let base = base_powertrain().unwrap();
        let recovery = base.with_recovery(0.9, 0.6, 0.22);
        let a = run(recovery);
        let b = run(recovery);
        assert_eq!(a.terminal_voltage, b.terminal_voltage);
        assert_eq!(a.relaxation, b.relaxation);
    }

    #[test]
    fn soc_is_monotone_nonincreasing() {
        let base = base_powertrain().unwrap();
        let trace = run(base.with_recovery(0.9, 0.6, 0.22));
        for pair in trace.soc.windows(2) {
            assert!(
                pair[1] <= pair[0] + 1e-12,
                "soc rose: {} -> {}",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn terminal_voltage_recovers_across_a_rest_phase() {
        let base = base_powertrain().unwrap();
        let recovery = run(base.with_recovery(0.9, 0.6, 0.22));
        // Start vs end of the first rest phase (steps PHASE_STEPS..2*PHASE_STEPS).
        let rest_start = PHASE_STEPS;
        let rest_end = 2 * PHASE_STEPS - 1;
        assert!(
            recovery.relaxation[rest_end] < recovery.relaxation[rest_start],
            "relaxation should decay over the rest: {} -> {}",
            recovery.relaxation[rest_start],
            recovery.relaxation[rest_end]
        );
        // Terminal voltage climbs across the rest even though charge falls.
        assert!(
            recovery.terminal_voltage[rest_end] > recovery.terminal_voltage[rest_start],
            "terminal voltage should recover: {} -> {}",
            recovery.terminal_voltage[rest_start],
            recovery.terminal_voltage[rest_end]
        );
        assert!(recovery.soc[rest_end] <= recovery.soc[rest_start]);
    }
}
