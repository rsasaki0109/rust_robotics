//! Unified, deterministic benchmark for the 2-D MPPI family.
//!
//! Four controller configurations solve the same point-to-point task with a
//! circular obstacle. Aggregate metrics are written to CSV and SVG so changes
//! to the controller can be compared with one command.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;
use std::time::Instant;

use rust_robotics::control::{
    MppiCircularObstacle2D, MppiConfig, MppiController2D, MppiState2D, MppiTerminalValueGrid2D,
};
use rust_robotics::prelude::*;

const CSV_OUTPUT: &str = "docs/assets/mppi-unified-benchmark.csv";
const SVG_OUTPUT: &str = "docs/assets/mppi-unified-benchmark.svg";
const GOAL: (f64, f64) = (2.4, 0.8);
const DT: f64 = 0.1;
const STEPS: usize = 32;
const SEEDS: [u64; 5] = [11, 23, 37, 53, 71];

#[derive(Clone, Copy)]
enum Variant {
    Vanilla,
    ConstraintDiscounted,
    TerminalValue,
    AdaptiveLambda,
}

impl Variant {
    fn name(self) -> &'static str {
        match self {
            Self::Vanilla => "vanilla",
            Self::ConstraintDiscounted => "constraint-discounted",
            Self::TerminalValue => "terminal-value",
            Self::AdaptiveLambda => "adaptive-lambda",
        }
    }
}

#[derive(Debug)]
struct Summary {
    variant: &'static str,
    runs: usize,
    successes: usize,
    mean_terminal_error: f64,
    mean_terminal_cost: f64,
    max_constraint_violation: f64,
    mean_plan_time_us: f64,
}

fn obstacle() -> MppiCircularObstacle2D {
    MppiCircularObstacle2D::new(1.15, 0.3, 0.32)
}

fn config(variant: Variant, seed: u64) -> RoboticsResult<MppiConfig> {
    let base = MppiConfig {
        horizon: 18,
        samples: 240,
        dt: DT,
        lambda: 1.0,
        noise_sigma: 1.1,
        control_limit: 2.5,
        goal_weight: 1.5,
        terminal_weight: 8.0,
        obstacles: vec![obstacle()],
        safety_margin: 0.12,
        seed,
        ..MppiConfig::default()
    };
    Ok(match variant {
        Variant::Vanilla => MppiConfig {
            constraint_weight: 0.0,
            ..base
        },
        Variant::ConstraintDiscounted => MppiConfig {
            constraint_weight: 5_000.0,
            constraint_discount: 0.90,
            ..base
        },
        Variant::TerminalValue => MppiConfig {
            constraint_weight: 5_000.0,
            constraint_discount: 0.90,
            terminal_value_weight: 8.0,
            terminal_value_grid: Some(MppiTerminalValueGrid2D::from_goal_distance(
                70, 55, -0.5, -1.0, 0.06, GOAL,
            )?),
            ..base
        },
        Variant::AdaptiveLambda => MppiConfig {
            constraint_weight: 5_000.0,
            constraint_discount: 0.90,
            adaptive_lambda: true,
            min_lambda: 0.05,
            max_lambda: 8.0,
            target_effective_sample_ratio: 0.25,
            ..base
        },
    })
}

fn distance(state: MppiState2D, point: (f64, f64)) -> f64 {
    ((state.x - point.0).powi(2) + (state.y - point.1).powi(2)).sqrt()
}

fn violation(state: MppiState2D) -> f64 {
    let o = obstacle();
    (o.radius + 0.12 - distance(state, (o.x, o.y))).max(0.0)
}

fn run_variant(variant: Variant) -> RoboticsResult<Summary> {
    let mut successes = 0;
    let mut terminal_error_sum = 0.0;
    let mut terminal_cost_sum = 0.0;
    let mut max_violation: f64 = 0.0;
    let mut elapsed_us = 0.0;
    let mut plan_count = 0;

    for seed in SEEDS {
        let mut controller = MppiController2D::new(config(variant, seed)?)?;
        let mut state = MppiState2D::new(0.0, 0.0, 0.0, 0.0);
        let mut run_max_violation: f64 = 0.0;
        for _ in 0..STEPS {
            let started = Instant::now();
            let plan = controller.plan(state, GOAL)?;
            elapsed_us += started.elapsed().as_secs_f64() * 1_000_000.0;
            plan_count += 1;
            state = state.step(plan.first_control, DT);
            run_max_violation = run_max_violation.max(violation(state));
        }
        let terminal_error = distance(state, GOAL);
        successes += usize::from(terminal_error <= 0.35 && run_max_violation <= 1e-9);
        terminal_error_sum += terminal_error;
        terminal_cost_sum += terminal_error * terminal_error;
        max_violation = max_violation.max(run_max_violation);
    }

    Ok(Summary {
        variant: variant.name(),
        runs: SEEDS.len(),
        successes,
        mean_terminal_error: terminal_error_sum / SEEDS.len() as f64,
        mean_terminal_cost: terminal_cost_sum / SEEDS.len() as f64,
        max_constraint_violation: max_violation,
        mean_plan_time_us: elapsed_us / plan_count as f64,
    })
}

fn collect() -> RoboticsResult<Vec<Summary>> {
    [
        Variant::Vanilla,
        Variant::ConstraintDiscounted,
        Variant::TerminalValue,
        Variant::AdaptiveLambda,
    ]
    .into_iter()
    .map(run_variant)
    .collect()
}

fn render_csv(rows: &[Summary]) -> String {
    let mut csv = String::from(
        "variant,runs,success_rate,mean_terminal_error,mean_terminal_cost,max_constraint_violation,mean_plan_time_us\n",
    );
    for row in rows {
        let _ = writeln!(
            csv,
            "{},{},{:.3},{:.6},{:.6},{:.6},{:.1}",
            row.variant,
            row.runs,
            row.successes as f64 / row.runs as f64,
            row.mean_terminal_error,
            row.mean_terminal_cost,
            row.max_constraint_violation,
            row.mean_plan_time_us
        );
    }
    csv
}

fn render_svg(rows: &[Summary]) -> String {
    let colors = ["#8e8e93", "#0a84ff", "#30b0c7", "#bf5af2"];
    let mut svg = String::new();
    let _ = writeln!(svg, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" height=\"430\" viewBox=\"0 0 960 430\">");
    let _ = writeln!(svg, "<rect width=\"960\" height=\"430\" fill=\"#fbfbfd\"/>");
    let _ = writeln!(svg, "<text x=\"40\" y=\"36\" font-family=\"sans-serif\" font-size=\"22\" fill=\"#1d1d1f\">Unified MPPI benchmark</text>");
    let _ = writeln!(svg, "<text x=\"40\" y=\"59\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">5 fixed seeds · same dynamics, horizon, samples, goal, and obstacle</text>");

    let panels = [
        ("Success rate", 70.0),
        ("Terminal error (m)", 360.0),
        ("Max violation (m)", 650.0),
    ];
    for (panel_index, (title, left)) in panels.iter().enumerate() {
        let _ = writeln!(svg, "<text x=\"{left}\" y=\"96\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#1d1d1f\">{title}</text>");
        let bottom = 330.0;
        let height = 190.0;
        let max_value = match panel_index {
            0 => 1.0,
            1 => {
                rows.iter()
                    .map(|r| r.mean_terminal_error)
                    .fold(0.01, f64::max)
                    * 1.1
            }
            _ => {
                rows.iter()
                    .map(|r| r.max_constraint_violation)
                    .fold(0.01, f64::max)
                    * 1.1
            }
        };
        for (i, row) in rows.iter().enumerate() {
            let value = match panel_index {
                0 => row.successes as f64 / row.runs as f64,
                1 => row.mean_terminal_error,
                _ => row.max_constraint_violation,
            };
            let x = left + i as f64 * 61.0;
            let bar_height = height * value / max_value;
            let y = bottom - bar_height;
            let _ = writeln!(svg, "<rect x=\"{x:.1}\" y=\"{y:.1}\" width=\"42\" height=\"{bar_height:.1}\" rx=\"3\" fill=\"{}\"/>", colors[i]);
            let _ = writeln!(svg, "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"10\" text-anchor=\"middle\" fill=\"#1d1d1f\">{value:.2}</text>", x + 21.0, (y - 7.0).max(112.0));
            let short = ["base", "disc", "value", "adapt"][i];
            let _ = writeln!(svg, "<text x=\"{:.1}\" y=\"349\" font-family=\"sans-serif\" font-size=\"10\" text-anchor=\"middle\" fill=\"#6e6e73\">{short}</text>", x + 21.0);
        }
        let _ = writeln!(
            svg,
            "<line x1=\"{left}\" y1=\"330\" x2=\"{:.1}\" y2=\"330\" stroke=\"#d1d1d6\"/>",
            left + 246.0
        );
    }
    let _ = writeln!(svg, "<text x=\"40\" y=\"395\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#6e6e73\">Planning latency is recorded in the companion CSV; wall-clock values depend on the machine.</text>");
    svg.push_str("</svg>\n");
    svg
}

fn main() -> RoboticsResult<()> {
    let rows = collect()?;
    fs::create_dir_all(Path::new(CSV_OUTPUT).parent().expect("asset parent"))?;
    fs::write(CSV_OUTPUT, render_csv(&rows))?;
    fs::write(SVG_OUTPUT, render_svg(&rows))?;
    for row in &rows {
        println!(
            "{:>21}: success={}/{} error={:.3} violation={:.3} plan={:.0} us",
            row.variant,
            row.successes,
            row.runs,
            row.mean_terminal_error,
            row.max_constraint_violation,
            row.mean_plan_time_us
        );
    }
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn outputs_have_all_variants_and_metrics() {
        let rows = collect().expect("benchmark runs");
        let csv = render_csv(&rows);
        let svg = render_svg(&rows);
        assert_eq!(rows.len(), 4);
        assert!(csv.contains("mean_terminal_cost,max_constraint_violation,mean_plan_time_us"));
        for variant in [
            "vanilla",
            "constraint-discounted",
            "terminal-value",
            "adaptive-lambda",
        ] {
            assert!(csv.contains(variant));
        }
        assert!(svg.contains("Unified MPPI benchmark"));
    }
}
