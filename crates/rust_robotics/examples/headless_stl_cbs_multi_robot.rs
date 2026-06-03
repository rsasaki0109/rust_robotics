//! STL-CBS multi-robot planning example.
//!
//! Independent shortest paths collide in a small crossing scene. CBS adds
//! time-indexed vertex/edge constraints, replans the affected agent, and the
//! final plan is checked with STL robustness primitives.

use rust_robotics::planning::{
    first_conflict, stl_eventually_reach_robustness, stl_pairwise_separation_robustness,
    StlCbsAgent, StlCbsConfig, StlCbsPath, StlCbsPlanner, StlRectangle2D, StlTimeInterval,
};
use rust_robotics::prelude::*;

const MAX_TIME: u64 = 18;

fn agents() -> Vec<StlCbsAgent> {
    vec![
        StlCbsAgent::new(0, (0, 2), (8, 2)),
        StlCbsAgent::new(1, (8, 2), (0, 2)),
        StlCbsAgent::new(2, (4, 0), (4, 4)),
    ]
}

fn planner() -> RoboticsResult<StlCbsPlanner> {
    StlCbsPlanner::new(StlCbsConfig {
        max_time: MAX_TIME,
        max_cbs_nodes: 2_048,
        ..StlCbsConfig::new(9, 5)
    })
}

fn path_string(path: &StlCbsPath) -> String {
    path.waypoints
        .iter()
        .map(|waypoint| format!("({}, {}, t={})", waypoint.x, waypoint.y, waypoint.t))
        .collect::<Vec<_>>()
        .join(" -> ")
}

fn goal_region(goal: (i32, i32)) -> RoboticsResult<StlRectangle2D> {
    StlRectangle2D::new(
        goal.0 as f64 - 0.25,
        goal.0 as f64 + 0.25,
        goal.1 as f64 - 0.25,
        goal.1 as f64 + 0.25,
    )
}

fn print_goal_robustness(paths: &[StlCbsPath], agents: &[StlCbsAgent]) -> RoboticsResult<()> {
    let interval = StlTimeInterval::new(0, MAX_TIME)?;
    for path in paths {
        let agent = agents
            .iter()
            .find(|agent| agent.id == path.agent_id)
            .expect("path references a known agent");
        let robustness = stl_eventually_reach_robustness(path, goal_region(agent.goal)?, interval)?;
        println!(
            "  agent {} eventually-goal robustness={:.2} arrival_t={}",
            path.agent_id,
            robustness,
            path.arrival_time()
        );
    }
    Ok(())
}

fn main() -> RoboticsResult<()> {
    let agents = agents();
    let planner = planner()?;
    let independent = planner.plan_independent(&agents)?;
    let independent_conflict = first_conflict(&independent, MAX_TIME);
    println!("independent first_conflict={independent_conflict:?}");
    for path in &independent {
        println!(
            "  independent agent {}: {}",
            path.agent_id,
            path_string(path)
        );
    }

    let plan = planner.plan(&agents)?;
    println!(
        "stl-cbs: total_cost={} conflicts_resolved={} expanded={} separation_robustness={:.2}",
        plan.total_cost,
        plan.conflicts_resolved,
        plan.high_level_nodes_expanded,
        plan.min_pairwise_separation_robustness
    );
    for path in &plan.paths {
        println!("  cbs agent {}: {}", path.agent_id, path_string(path));
    }
    print_goal_robustness(&plan.paths, &agents)?;
    println!(
        "  pairwise separation robustness={:.2}",
        stl_pairwise_separation_robustness(&plan.paths, 1.0, StlTimeInterval::new(0, MAX_TIME)?)?
    );
    println!(
        "  final_conflict={:?}",
        first_conflict(&plan.paths, MAX_TIME)
    );

    Ok(())
}
