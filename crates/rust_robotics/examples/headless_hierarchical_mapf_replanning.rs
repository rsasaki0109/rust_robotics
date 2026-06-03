//! Hierarchical MAPF replanning example.
//!
//! Independent paths are cheap but collide near the center. The hierarchical
//! layer detects coarse region conflicts, replans the affected group with CBS,
//! and leaves unrelated agents untouched.

use rust_robotics::planning::{
    HierarchicalMapfAgent2D, HierarchicalMapfConfig2D, HierarchicalMapfPlanner2D, StlCbsPath,
};
use rust_robotics::prelude::*;

fn agents() -> Vec<HierarchicalMapfAgent2D> {
    vec![
        HierarchicalMapfAgent2D::new(0, (0, 3), (11, 3)),
        HierarchicalMapfAgent2D::new(1, (11, 3), (0, 3)),
        HierarchicalMapfAgent2D::new(2, (5, 0), (5, 7)),
        HierarchicalMapfAgent2D::new(3, (0, 7), (3, 7)),
    ]
}

fn planner() -> RoboticsResult<HierarchicalMapfPlanner2D> {
    HierarchicalMapfPlanner2D::new(HierarchicalMapfConfig2D {
        max_time: 24,
        max_cbs_nodes: 4_096,
        ..HierarchicalMapfConfig2D::new(12, 8, 4, 4)
    })
}

fn path_string(path: &StlCbsPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| format!("({}, {}, t={})", waypoint.x, waypoint.y, waypoint.t))
        .collect::<Vec<_>>()
        .join(" -> ")
}

fn main() -> RoboticsResult<()> {
    let planner = planner()?;
    let plan = planner.plan(&agents())?;

    println!(
        "hierarchical-mapf: independent_conflicts={} final_conflicts={} region_conflicts={} groups={} fallback_full={} total_cost={}",
        plan.independent_cell_conflicts,
        plan.final_cell_conflicts,
        plan.region_conflicts.len(),
        plan.replanned_groups.len(),
        plan.fallback_full_replan,
        plan.total_cost
    );
    for route in &plan.region_routes {
        let route_text = route
            .regions
            .iter()
            .map(|region| format!("({}, {})", region.rx, region.ry))
            .collect::<Vec<_>>()
            .join(" -> ");
        println!("  region route agent {}: {route_text}", route.agent_id);
    }
    for group in &plan.replanned_groups {
        println!(
            "  replanned group {:?}: cbs_cost={} cbs_resolved={}",
            group.agent_ids, group.cbs_total_cost, group.cbs_conflicts_resolved
        );
    }
    for path in &plan.paths {
        println!("  final agent {}: {}", path.agent_id, path_string(path));
    }

    Ok(())
}
