//! Headless traversal-risk graph example.
//!
//! Shows how raising terrain-risk weight changes a route from the shortest path
//! through a hazardous band to a longer, safer detour.

use rust_robotics::planning::{
    TerrainRiskCell, TraversalRiskGraphConfig, TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

fn terrain() -> Vec<Vec<TerrainRiskCell>> {
    let width = 9;
    let height = 5;
    let mut cells = vec![vec![TerrainRiskCell::free(); height]; width];

    for x in 3..=5 {
        cells[x][2] = TerrainRiskCell::with_risk(5.0, 0.5, 0.0);
    }
    cells[4][1] = TerrainRiskCell::with_risk(2.5, 3.5, 0.0);
    cells[4][3] = TerrainRiskCell::with_risk(2.5, 3.5, 0.0);
    cells[2][0] = TerrainRiskCell::blocked();
    cells[6][4] = TerrainRiskCell::blocked();

    cells
}

fn plan_with_weight(risk_weight: f64) -> RoboticsResult<TraversalRiskPath> {
    let config = TraversalRiskGraphConfig {
        risk_weight,
        traversability_weight: 1.0,
        stability_weight: 1.0,
        exposure_weight: 0.5,
        ..TraversalRiskGraphConfig::new(9, 5, terrain())
    };
    let planner = TraversalRiskGraphPlanner::new(config)?;
    planner.plan(0, 2, 8, 2)
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
    let shortest = plan_with_weight(0.0)?;
    let risk_aware = plan_with_weight(2.0)?;

    print_path("distance-only", &shortest);
    print_path("risk-aware", &risk_aware);

    Ok(())
}
