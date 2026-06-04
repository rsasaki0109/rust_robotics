//! Risk-weight sweep for elevation-derived traversal-risk planning.
//!
//! Writes a CSV table and an SVG Pareto chart for distance-vs-terrain-risk
//! tradeoffs over several `risk_weight` values.

use std::collections::HashSet;
use std::fmt::Write;
use std::path::Path;

use rust_robotics::planning::{
    inflate_blocked_cells_by_radius, sweep_traversal_risk_weights, terrain_risk_from_elevation_map,
    ElevationRiskConfig, TerrainRiskCell, TraversalRiskGraphConfig, TraversalRiskPath,
    TraversalRiskWeightSample,
};
use rust_robotics::prelude::*;

const WIDTH: usize = 15;
const HEIGHT: usize = 9;
const CELL_SIZE: f64 = 0.5;
const CSV_OUTPUT: &str = "docs/assets/traversal-risk-weight-sweep.csv";
const SVG_OUTPUT: &str = "docs/assets/traversal-risk-weight-sweep.svg";

fn elevation_map() -> Vec<Vec<f64>> {
    let mut elevation = vec![vec![0.0; HEIGHT]; WIDTH];

    #[allow(clippy::needless_range_loop)]
    for x in 5..=9 {
        elevation[x][4] = 0.55;
    }
    elevation[2][1] = 1.25;
    elevation[12][7] = 1.25;

    elevation
}

fn terrain() -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    let raw = terrain_risk_from_elevation_map(
        &elevation_map(),
        &ElevationRiskConfig {
            cell_size: CELL_SIZE,
            slope_risk_scale: 4.0,
            roughness_risk_scale: 6.0,
            max_risk: 8.0,
            blocking_step_height: Some(1.0),
        },
    )?;
    inflate_blocked_cells_by_radius(&raw, 0.5, CELL_SIZE)
}

fn base_config() -> RoboticsResult<TraversalRiskGraphConfig> {
    Ok(TraversalRiskGraphConfig {
        risk_weight: 1.0,
        traversability_weight: 1.0,
        stability_weight: 0.8,
        exposure_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH as i32, HEIGHT as i32, terrain()?)
    })
}

fn path_key(path: &TraversalRiskPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| format!("{}:{}", waypoint.x, waypoint.y))
        .collect::<Vec<_>>()
        .join("|")
}

fn pareto_flags(samples: &[TraversalRiskWeightSample]) -> Vec<bool> {
    samples
        .iter()
        .enumerate()
        .map(|(index, sample)| {
            !samples.iter().enumerate().any(|(other_index, other)| {
                let dominates = other.path.distance_cost <= sample.path.distance_cost
                    && other.terrain_risk_cost <= sample.terrain_risk_cost
                    && (other.path.distance_cost < sample.path.distance_cost
                        || other.terrain_risk_cost < sample.terrain_risk_cost);
                let duplicate_before = other_index < index
                    && (other.path.distance_cost - sample.path.distance_cost).abs() < 1e-9
                    && (other.terrain_risk_cost - sample.terrain_risk_cost).abs() < 1e-9;
                dominates || duplicate_before
            })
        })
        .collect()
}

fn render_csv(samples: &[TraversalRiskWeightSample], pareto: &[bool]) -> String {
    let mut csv = String::from(
        "risk_weight,distance_cost,terrain_risk_cost,weighted_risk_cost,total_cost,waypoints,pareto,path\n",
    );
    for (sample, is_pareto) in samples.iter().zip(pareto) {
        writeln!(
            csv,
            "{:.3},{:.3},{:.3},{:.3},{:.3},{},{},{}",
            sample.risk_weight,
            sample.path.distance_cost,
            sample.terrain_risk_cost,
            sample.path.risk_cost,
            sample.path.total_cost,
            sample.path.waypoints.len(),
            is_pareto,
            path_key(&sample.path)
        )
        .unwrap();
    }
    csv
}

fn chart_x(distance: f64, min_distance: f64, max_distance: f64) -> f64 {
    if (max_distance - min_distance).abs() < 1e-9 {
        300.0
    } else {
        84.0 + (distance - min_distance) / (max_distance - min_distance) * 430.0
    }
}

fn chart_y(risk: f64, max_risk: f64) -> f64 {
    if max_risk.abs() < 1e-9 {
        335.0
    } else {
        335.0 - risk / max_risk * 235.0
    }
}

fn render_svg(samples: &[TraversalRiskWeightSample], pareto: &[bool]) -> String {
    let min_distance = samples
        .iter()
        .map(|sample| sample.path.distance_cost)
        .fold(f64::INFINITY, f64::min);
    let max_distance = samples
        .iter()
        .map(|sample| sample.path.distance_cost)
        .fold(f64::NEG_INFINITY, f64::max);
    let max_risk = samples
        .iter()
        .map(|sample| sample.terrain_risk_cost)
        .fold(0.0, f64::max)
        .max(1.0);

    let mut svg = String::new();
    writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="760" height="470" viewBox="0 0 760 470" role="img" aria-labelledby="title desc">"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<title id="title">Traversal-risk weight sweep Pareto chart</title>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<desc id="desc">Chart of path distance versus unweighted terrain risk for several risk-weight values.</desc>"##
    )
    .unwrap();
    writeln!(svg, r##"<rect width="760" height="470" fill="#f8fafc"/>"##).unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="44" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="26" font-weight="700">Risk-weight Pareto sweep</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="54" y="70" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="14">Each point is one TRG plan on the same elevation-derived terrain.</text>"##
    )
    .unwrap();

    for i in 0..=4 {
        let y = 335.0 - i as f64 * 58.75;
        writeln!(
            svg,
            r##"<line x1="84" y1="{y:.1}" x2="514" y2="{y:.1}" stroke="#e2e8f0" stroke-width="1"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="72" y="{:.1}" text-anchor="end" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11">{:.1}</text>"##,
            y + 4.0,
            max_risk * i as f64 / 4.0
        )
        .unwrap();
    }
    for i in 0..=4 {
        let x = 84.0 + i as f64 * 107.5;
        writeln!(
            svg,
            r##"<line x1="{x:.1}" y1="100" x2="{x:.1}" y2="335" stroke="#e2e8f0" stroke-width="1"/>"##
        )
        .unwrap();
        writeln!(
            svg,
            r##"<text x="{x:.1}" y="355" text-anchor="middle" fill="#64748b" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11">{:.1}</text>"##,
            min_distance + (max_distance - min_distance) * i as f64 / 4.0
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<line x1="84" y1="335" x2="514" y2="335" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<line x1="84" y1="100" x2="84" y2="335" stroke="#475569" stroke-width="1.5"/>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="299" y="391" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700">distance cost</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<text x="24" y="218" text-anchor="middle" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13" font-weight="700" transform="rotate(-90 24 218)">terrain risk</text>"##
    )
    .unwrap();

    let pareto_points = samples
        .iter()
        .zip(pareto)
        .filter(|(_, is_pareto)| **is_pareto)
        .map(|(sample, _)| {
            format!(
                "{:.1},{:.1}",
                chart_x(sample.path.distance_cost, min_distance, max_distance),
                chart_y(sample.terrain_risk_cost, max_risk)
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    writeln!(
        svg,
        r##"<polyline points="{pareto_points}" fill="none" stroke="#0f766e" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>"##
    )
    .unwrap();

    for (sample, is_pareto) in samples.iter().zip(pareto) {
        let x = chart_x(sample.path.distance_cost, min_distance, max_distance);
        let y = chart_y(sample.terrain_risk_cost, max_risk);
        let fill = if *is_pareto { "#0f766e" } else { "#94a3b8" };
        let radius = if *is_pareto { 7.0 } else { 5.0 };
        writeln!(
            svg,
            r##"<circle cx="{x:.1}" cy="{y:.1}" r="{radius:.1}" fill="{fill}" stroke="#ffffff" stroke-width="2"/>"##
        )
        .unwrap();
        if *is_pareto {
            writeln!(
                svg,
                r##"<text x="{:.1}" y="{:.1}" fill="#0f172a" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="11" font-weight="700">w={:.2}</text>"##,
                x + 9.0,
                y - 8.0,
                sample.risk_weight
            )
            .unwrap();
        }
    }

    let unique_paths = samples
        .iter()
        .map(|sample| path_key(&sample.path))
        .collect::<HashSet<_>>()
        .len();
    let pareto_count = pareto.iter().filter(|flag| **flag).count();
    let min_risk = samples
        .iter()
        .map(|sample| sample.terrain_risk_cost)
        .fold(f64::INFINITY, f64::min);
    let max_total = samples
        .iter()
        .map(|sample| sample.path.total_cost)
        .fold(f64::NEG_INFINITY, f64::max);
    writeln!(
        svg,
        r##"<text x="560" y="112" fill="#111827" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="15" font-weight="700">Sweep summary</text>"##
    )
    .unwrap();
    for (index, line) in [
        format!("samples: {}", samples.len()),
        format!("unique paths: {unique_paths}"),
        format!("Pareto points: {pareto_count}"),
        format!("min risk: {min_risk:.2}"),
        format!("max total: {max_total:.2}"),
    ]
    .iter()
    .enumerate()
    {
        writeln!(
            svg,
            r##"<text x="560" y="{:.1}" fill="#475569" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">{line}</text>"##,
            139.0 + index as f64 * 24.0
        )
        .unwrap();
    }
    writeln!(
        svg,
        r##"<circle cx="566" cy="278" r="6" fill="#0f766e" stroke="#ffffff" stroke-width="2"/><text x="582" y="282" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">non-dominated</text>"##
    )
    .unwrap();
    writeln!(
        svg,
        r##"<circle cx="566" cy="306" r="5" fill="#94a3b8" stroke="#ffffff" stroke-width="2"/><text x="582" y="310" fill="#334155" font-family="Inter, Segoe UI, Arial, sans-serif" font-size="13">dominated / duplicate</text>"##
    )
    .unwrap();
    writeln!(svg, "</svg>").unwrap();

    svg
}

fn write_text(path: &str, text: &str) -> RoboticsResult<()> {
    let output = Path::new(path);
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(output, text)?;
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let risk_weights = [0.0, 0.05, 0.1, 0.2, 0.4, 0.8, 1.2, 1.6, 2.4, 3.2];
    let samples =
        sweep_traversal_risk_weights(base_config()?, &risk_weights, 0, 4, (WIDTH - 1) as i32, 4)?;
    let pareto = pareto_flags(&samples);
    let csv = render_csv(&samples, &pareto);
    let svg = render_svg(&samples, &pareto);

    write_text(CSV_OUTPUT, &csv)?;
    write_text(SVG_OUTPUT, &svg)?;

    let unique_paths = samples
        .iter()
        .map(|sample| path_key(&sample.path))
        .collect::<HashSet<_>>()
        .len();
    println!(
        "wrote {CSV_OUTPUT} and {SVG_OUTPUT} ({} samples, {} unique paths)",
        samples.len(),
        unique_paths
    );
    for sample in &samples {
        println!(
            "w={:.2}: distance={:.2} terrain_risk={:.2} weighted_risk={:.2} total={:.2} waypoints={}",
            sample.risk_weight,
            sample.path.distance_cost,
            sample.terrain_risk_cost,
            sample.path.risk_cost,
            sample.path.total_cost,
            sample.path.waypoints.len()
        );
    }

    Ok(())
}
