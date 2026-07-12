//! Calibration coverage versus planning-delay benchmark for CP-SIPP.

use std::fmt::Write as _;
use std::fs;

use rust_robotics::planning::{
    calibration_errors_from_trajectories, ConformalSippConfig, ConformalSippPlanner,
    PredictedObstaclePoint, PredictedObstacleTrajectory,
};
use rust_robotics::prelude::*;

const HORIZON: u64 = 16;
const CSV_OUTPUT: &str = "docs/assets/conformal-sipp-coverage.csv";
const SVG_OUTPUT: &str = "docs/assets/conformal-sipp-coverage.svg";

#[derive(Debug)]
struct Row {
    confidence: f64,
    radius: f64,
    test_coverage: f64,
    arrival: u64,
    delay: u64,
    path_length: f64,
}

fn trajectory(dx: f64, dy: f64) -> PredictedObstacleTrajectory {
    PredictedObstacleTrajectory::new(vec![
        PredictedObstaclePoint::new(0, 4.0 + dx, 2.0 + dy),
        PredictedObstaclePoint::new(HORIZON, 4.0 + dx, 2.0 + dy),
    ])
}

fn error(index: usize, count: usize) -> (f64, f64) {
    let fraction = (index + 1) as f64 / count as f64;
    let radius = 0.05 + 1.35 * fraction.powf(1.7);
    let angle = index as f64 * 2.399_963_229_728_653;
    (radius * angle.cos(), radius * angle.sin())
}

fn calibration_pairs(
    count: usize,
) -> Vec<(PredictedObstacleTrajectory, PredictedObstacleTrajectory)> {
    (0..count)
        .map(|index| {
            let (dx, dy) = error(index, count);
            (trajectory(0.0, 0.0), trajectory(dx, dy))
        })
        .collect()
}

fn path_length(path: &[rust_robotics::planning::sipp::TimedWaypoint]) -> f64 {
    path.windows(2)
        .map(|pair| {
            let dx = (pair[1].x - pair[0].x) as f64;
            let dy = (pair[1].y - pair[0].y) as f64;
            dx.hypot(dy)
        })
        .sum()
}

fn collect() -> RoboticsResult<Vec<Row>> {
    let scores = calibration_errors_from_trajectories(&calibration_pairs(60), HORIZON)?;
    let test_errors = (0..400).map(|i| error(i, 400)).collect::<Vec<_>>();
    let mut rows = Vec::new();
    for confidence in [0.0, 0.5, 0.7, 0.8, 0.9, 0.95, 1.0] {
        let planner = ConformalSippPlanner::new(ConformalSippConfig {
            width: 9,
            height: 5,
            obstacle_map: vec![vec![false; 5]; 9],
            predicted_obstacles: vec![trajectory(0.0, 0.0)],
            calibration_errors_by_time: scores.clone(),
            time_horizon: HORIZON,
            required_confidence: confidence,
            obstacle_radius: 0.0,
            allow_diagonal: false,
        })?;
        let plan = planner.plan(0, 2, 8, 2)?;
        let radius = planner.conformal_radius_at(4)?;
        let covered = test_errors
            .iter()
            .filter(|(dx, dy)| dx.hypot(*dy) <= radius)
            .count();
        let arrival = plan.path.last().expect("path has goal").t;
        rows.push(Row {
            confidence,
            radius,
            test_coverage: covered as f64 / test_errors.len() as f64,
            arrival,
            delay: arrival.saturating_sub(8),
            path_length: path_length(&plan.path),
        });
    }
    Ok(rows)
}

fn render_csv(rows: &[Row]) -> String {
    let mut csv = String::from(
        "required_confidence,conformal_radius,test_coverage,arrival_time,delay,path_length\n",
    );
    for row in rows {
        let _ = writeln!(
            csv,
            "{:.2},{:.4},{:.4},{},{},{:.3}",
            row.confidence, row.radius, row.test_coverage, row.arrival, row.delay, row.path_length
        );
    }
    csv
}

fn render_svg(rows: &[Row]) -> String {
    let mut svg = String::from(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" height=\"430\" viewBox=\"0 0 900 430\">\n<rect width=\"900\" height=\"430\" fill=\"#fbfbfd\"/>\n",
    );
    let _ = writeln!(svg, "<text x=\"45\" y=\"38\" font-family=\"sans-serif\" font-size=\"22\" fill=\"#1d1d1f\">CP-SIPP: coverage versus delay</text>");
    let _ = writeln!(svg, "<text x=\"45\" y=\"62\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#6e6e73\">60 calibration episodes · 400 held-out errors · deterministic dataset</text>");
    let (left, top, width, height) = (75.0, 100.0, 740.0, 250.0);
    for tick in 0..=5 {
        let y = top + height - tick as f64 * height / 5.0;
        let _ = writeln!(svg, "<line x1=\"{left}\" y1=\"{y}\" x2=\"{}\" y2=\"{y}\" stroke=\"#e5e5ea\"/><text x=\"62\" y=\"{}\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#6e6e73\">{:.1}</text>", left + width, y + 4.0, tick as f64 / 5.0);
    }
    let points = rows
        .iter()
        .map(|row| {
            let x = left + row.confidence * width;
            let y = top + height - row.test_coverage * height;
            format!("{x:.1},{y:.1}")
        })
        .collect::<Vec<_>>()
        .join(" ");
    let _ = writeln!(
        svg,
        "<polyline points=\"{points}\" fill=\"none\" stroke=\"#0a84ff\" stroke-width=\"3\"/>"
    );
    for row in rows {
        let x = left + row.confidence * width;
        let y = top + height - row.test_coverage * height;
        let _ = writeln!(svg, "<circle cx=\"{x:.1}\" cy=\"{y:.1}\" r=\"5\" fill=\"#0a84ff\"/><text x=\"{x:.1}\" y=\"{:.1}\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#1d1d1f\">+{}t</text>", y - 11.0, row.delay);
    }
    let _ = writeln!(svg, "<text x=\"445\" y=\"390\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#1d1d1f\">Required confidence (labels show planning delay)</text></svg>");
    svg
}

fn main() -> RoboticsResult<()> {
    let rows = collect()?;
    fs::write(CSV_OUTPUT, render_csv(&rows))?;
    fs::write(SVG_OUTPUT, render_svg(&rows))?;
    for row in &rows {
        println!(
            "c={:.2}: radius={:.3} coverage={:.3} arrival={} delay={}",
            row.confidence, row.radius, row.test_coverage, row.arrival, row.delay
        );
    }
    println!("wrote {CSV_OUTPUT} and {SVG_OUTPUT}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coverage_and_radius_are_monotone() {
        let rows = collect().unwrap();
        assert!(rows.windows(2).all(|pair| pair[0].radius <= pair[1].radius));
        assert!(rows
            .windows(2)
            .all(|pair| pair[0].test_coverage <= pair[1].test_coverage));
        assert!(render_csv(&rows).contains("test_coverage,arrival_time,delay"));
        assert!(render_svg(&rows).contains("coverage versus delay"));
    }
}
