//! Clearance-aware traversal-risk graph example.
//!
//! The center corridor is geometrically traversable, but it runs between two
//! obstacle rails. Clearance exposure risk makes the planner prefer a wider
//! detour when safety margin matters.

use rust_robotics::planning::{
    add_clearance_exposure_risk, clearance_map, ClearanceRiskConfig, TerrainRiskCell,
    TraversalRiskGraphConfig, TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

const WIDTH: usize = 9;
const HEIGHT: usize = 7;
const CELL_SIZE: f64 = 1.0;

fn corridor_terrain() -> Vec<Vec<TerrainRiskCell>> {
    let mut cells = vec![vec![TerrainRiskCell::free(); HEIGHT]; WIDTH];
    for x in 2..=6 {
        cells[x][2] = TerrainRiskCell::blocked();
        cells[x][4] = TerrainRiskCell::blocked();
    }
    cells
}

fn clearance_risk_terrain() -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    add_clearance_exposure_risk(
        &corridor_terrain(),
        &ClearanceRiskConfig {
            cell_size: CELL_SIZE,
            minimum_clearance: 2.0,
            risk_scale: 8.0,
            max_risk: 8.0,
            additive: true,
        },
    )
}

fn plan_with_weight(
    risk_weight: f64,
) -> RoboticsResult<(TraversalRiskGraphPlanner, TraversalRiskPath)> {
    let planner = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
        risk_weight,
        exposure_weight: 1.0,
        traversability_weight: 0.0,
        stability_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH as i32, HEIGHT as i32, clearance_risk_terrain()?)
    })?;
    let path = planner.plan(0, 3, (WIDTH - 1) as i32, 3)?;
    Ok((planner, path))
}

fn min_path_clearance(path: &TraversalRiskPath) -> RoboticsResult<f64> {
    let clearances = clearance_map(&corridor_terrain(), CELL_SIZE)?;
    Ok(path
        .waypoints
        .iter()
        .map(|waypoint| clearances[waypoint.x as usize][waypoint.y as usize])
        .fold(f64::INFINITY, f64::min))
}

fn print_path(
    label: &str,
    planner: &TraversalRiskGraphPlanner,
    path: &TraversalRiskPath,
) -> RoboticsResult<()> {
    let terrain_risk = planner.path_terrain_risk(&path.waypoints)?;
    let min_clearance = min_path_clearance(path)?;
    let cells = path
        .waypoints
        .iter()
        .map(|waypoint| format!("({}, {})", waypoint.x, waypoint.y))
        .collect::<Vec<_>>()
        .join(" -> ");
    println!(
        "{label}: distance={:.2} terrain_risk={terrain_risk:.2} min_clearance={min_clearance:.2} total={:.2} waypoints={}",
        path.distance_cost,
        path.total_cost,
        path.waypoints.len()
    );
    println!("  {cells}");
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let (distance_planner, distance_path) = plan_with_weight(0.0)?;
    let (clearance_planner, clearance_path) = plan_with_weight(1.0)?;

    print_path("distance-only", &distance_planner, &distance_path)?;
    print_path("clearance-aware", &clearance_planner, &clearance_path)?;

    Ok(())
}
