//! Adaptive Costmap-NAMO style example.
//!
//! A movable obstacle starts as a soft cost, so the planner still takes the
//! short center corridor. Repeated stuck observations raise that cost to lethal
//! and the same query replans around the movable obstacle.

use rust_robotics::planning::{
    AdaptiveCostmapNamo, AdaptiveCostmapNamoConfig, MotionProgressObservation,
    TraversalRiskGraphConfig, TraversalRiskGraphPlanner, TraversalRiskPath,
};
use rust_robotics::prelude::*;

const WIDTH: i32 = 9;
const HEIGHT: i32 = 5;
const MOVABLE: &[(i32, i32)] = &[(3, 2), (4, 2), (5, 2)];

fn adaptive_map() -> RoboticsResult<AdaptiveCostmapNamo> {
    let mut map = AdaptiveCostmapNamo::new(AdaptiveCostmapNamoConfig {
        movable_initial_cost: 15.0,
        movable_cost_increment: 35.0,
        movable_cost_decrement: 20.0,
        ..AdaptiveCostmapNamoConfig::new(WIDTH, HEIGHT)
    })?;
    for &(x, y) in MOVABLE {
        map.mark_movable_obstacle(x, y)?;
    }
    map.mark_static_obstacle(1, 0)?;
    map.mark_static_obstacle(7, 4)?;
    Ok(map)
}

fn plan(
    map: &AdaptiveCostmapNamo,
) -> RoboticsResult<(TraversalRiskGraphPlanner, TraversalRiskPath)> {
    let planner = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
        risk_weight: 1.0,
        exposure_weight: 1.0,
        traversability_weight: 0.0,
        stability_weight: 0.0,
        ..TraversalRiskGraphConfig::new(WIDTH, HEIGHT, map.to_traversal_risk_cells(true))
    })?;
    let path = planner.plan(0, 2, 8, 2)?;
    Ok((planner, path))
}

fn print_path(
    label: &str,
    planner: &TraversalRiskGraphPlanner,
    path: &TraversalRiskPath,
    map: &AdaptiveCostmapNamo,
) -> RoboticsResult<()> {
    let cells = path
        .waypoints
        .iter()
        .map(|waypoint| format!("({}, {})", waypoint.x, waypoint.y))
        .collect::<Vec<_>>()
        .join(" -> ");
    let movable_costs = MOVABLE
        .iter()
        .map(|&(x, y)| format!("{:.0}", map.cell(x, y).unwrap().cost))
        .collect::<Vec<_>>()
        .join("/");
    println!(
        "{label}: movable_costs={movable_costs} distance={:.2} terrain_risk={:.2} total={:.2} waypoints={}",
        path.distance_cost,
        planner.path_terrain_risk(&path.waypoints)?,
        path.total_cost,
        path.waypoints.len()
    );
    println!("  {cells}");
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let mut map = adaptive_map()?;
    let (initial_planner, initial_path) = plan(&map)?;
    print_path("before stuck", &initial_planner, &initial_path, &map)?;

    for _ in 0..3 {
        map.update_movable_costs(MOVABLE, MotionProgressObservation::stuck(0.4))?;
    }
    let (stuck_planner, stuck_path) = plan(&map)?;
    print_path("after stuck", &stuck_planner, &stuck_path, &map)?;

    map.update_movable_costs(MOVABLE, MotionProgressObservation::moving(0.4, 0.35, 0.2))?;
    let (recovered_planner, recovered_path) = plan(&map)?;
    print_path("after progress", &recovered_planner, &recovered_path, &map)?;

    Ok(())
}
