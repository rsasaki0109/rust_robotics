//! Risk-map smoothing example for traversal-risk planning.
//!
//! Isolated high-risk cells can make a risk-aware graph planner zigzag around
//! sensor noise. This demo smooths the risk map before planning and compares
//! path shape plus cost summaries.

use rust_robotics::planning::{
    smooth_terrain_risk, RiskMapSmoothingConfig, TerrainRiskCell, TraversalRiskGraphConfig,
    TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

const WIDTH: usize = 13;
const HEIGHT: usize = 7;

fn noisy_terrain() -> Vec<Vec<TerrainRiskCell>> {
    let mut cells = vec![vec![TerrainRiskCell::free(); HEIGHT]; WIDTH];

    for &(x, y, risk) in &[
        (3, 3, 8.0),
        (6, 3, 8.0),
        (9, 3, 8.0),
        (5, 2, 3.0),
        (7, 4, 3.0),
    ] {
        cells[x][y] = TerrainRiskCell::with_risk(risk, 0.0, 0.0);
    }
    cells[2][0] = TerrainRiskCell::blocked();
    cells[10][6] = TerrainRiskCell::blocked();

    cells
}

fn plan(
    cells: Vec<Vec<TerrainRiskCell>>,
) -> RoboticsResult<(TraversalRiskGraphPlanner, TraversalRiskPath)> {
    let planner = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
        risk_weight: 0.2,
        traversability_weight: 1.0,
        stability_weight: 0.0,
        exposure_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH as i32, HEIGHT as i32, cells)
    })?;
    let path = planner.plan(0, 3, (WIDTH - 1) as i32, 3)?;
    Ok((planner, path))
}

fn risk_stats(cells: &[Vec<TerrainRiskCell>]) -> (f64, f64) {
    let mut max_risk: f64 = 0.0;
    let mut sum = 0.0;
    let mut count = 0;
    for column in cells {
        for cell in column {
            if cell.blocked {
                continue;
            }
            max_risk = max_risk.max(cell.traversability_risk);
            sum += cell.traversability_risk;
            count += 1;
        }
    }
    (max_risk, sum / count as f64)
}

fn turn_count(path: &TraversalRiskPath) -> usize {
    path.waypoints
        .windows(3)
        .filter(|window| {
            let first = (window[1].x - window[0].x, window[1].y - window[0].y);
            let second = (window[2].x - window[1].x, window[2].y - window[1].y);
            first != second
        })
        .count()
}

fn print_report(
    label: &str,
    cells: &[Vec<TerrainRiskCell>],
    planner: &TraversalRiskGraphPlanner,
    path: &TraversalRiskPath,
) -> RoboticsResult<()> {
    let (max_risk, mean_risk) = risk_stats(cells);
    let terrain_risk = planner.path_terrain_risk(&path.waypoints)?;
    let cells = path
        .waypoints
        .iter()
        .map(|waypoint| format!("({}, {})", waypoint.x, waypoint.y))
        .collect::<Vec<_>>()
        .join(" -> ");
    println!(
        "{label}: max_risk={max_risk:.2} mean_risk={mean_risk:.2} distance={:.2} terrain_risk={terrain_risk:.2} total={:.2} turns={} waypoints={}",
        path.distance_cost,
        path.total_cost,
        turn_count(path),
        path.waypoints.len()
    );
    println!("  {cells}");
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let raw = noisy_terrain();
    let smoothed = smooth_terrain_risk(
        &raw,
        &RiskMapSmoothingConfig {
            radius_cells: 1,
            iterations: 2,
            sigma_cells: 1.0,
            smooth_blocked_cells: false,
        },
    )?;

    let (raw_planner, raw_path) = plan(raw.clone())?;
    let (smoothed_planner, smoothed_path) = plan(smoothed.clone())?;

    print_report("raw risk map", &raw, &raw_planner, &raw_path)?;
    print_report(
        "smoothed risk map",
        &smoothed,
        &smoothed_planner,
        &smoothed_path,
    )?;

    Ok(())
}
