//! Elevation-derived traversal-risk graph example.
//!
//! Builds terrain risk from a synthetic elevation map, inflates blocked cells by
//! a robot footprint, then compares shortest-path and risk-aware plans.

use rust_robotics::planning::{
    inflate_blocked_cells_by_radius, terrain_risk_from_elevation_map, ElevationRiskConfig,
    TerrainRiskCell, TraversalRiskGraphConfig, TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

const WIDTH: usize = 15;
const HEIGHT: usize = 9;
const CELL_SIZE: f64 = 0.5;

fn elevation_map() -> Vec<Vec<f64>> {
    let mut elevation = vec![vec![0.0; HEIGHT]; WIDTH];

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

fn planner_with_weight(risk_weight: f64) -> RoboticsResult<TraversalRiskGraphPlanner> {
    let config = TraversalRiskGraphConfig {
        risk_weight,
        traversability_weight: 1.0,
        stability_weight: 0.8,
        exposure_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH as i32, HEIGHT as i32, terrain()?)
    };
    TraversalRiskGraphPlanner::new(config)
}

fn plan_with_weight(risk_weight: f64) -> RoboticsResult<TraversalRiskPath> {
    planner_with_weight(risk_weight)?.plan(0, 4, (WIDTH - 1) as i32, 4)
}

fn terrain_stats(cells: &[Vec<TerrainRiskCell>]) -> RoboticsResult<(usize, f64)> {
    let planner = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
        risk_weight: 1.0,
        traversability_weight: 1.0,
        stability_weight: 0.8,
        exposure_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH as i32, HEIGHT as i32, cells.to_vec())
    })?;

    let mut blocked = 0;
    let mut max_risk: f64 = 0.0;
    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            if cells[x][y].blocked {
                blocked += 1;
            }
            max_risk = max_risk.max(planner.cell_risk(x as i32, y as i32)?);
        }
    }

    Ok((blocked, max_risk))
}

fn print_path(label: &str, path: &TraversalRiskPath) {
    let cells = path
        .waypoints
        .iter()
        .map(|waypoint| format!("({}, {})", waypoint.x, waypoint.y))
        .collect::<Vec<_>>()
        .join(" -> ");
    println!(
        "{label}: total={:.2} distance={:.2} risk={:.2} waypoints={}",
        path.total_cost,
        path.distance_cost,
        path.risk_cost,
        path.waypoints.len()
    );
    println!("  {cells}");
}

fn main() -> RoboticsResult<()> {
    let cells = terrain()?;
    let (blocked, max_risk) = terrain_stats(&cells)?;
    let shortest = plan_with_weight(0.0)?;
    let risk_aware = plan_with_weight(1.4)?;

    println!(
        "elevation-derived terrain: {}x{} cells, blocked={}, max_risk={:.2}",
        WIDTH, HEIGHT, blocked, max_risk
    );
    print_path("distance-only", &shortest);
    print_path("risk-aware", &risk_aware);

    Ok(())
}
