//! Headless kinodynamic STL-CBS example.

use rust_robotics::planning::{
    first_conflict, KinodynamicHeading2D, KinodynamicStlCbsAgent2D, KinodynamicStlCbsConfig2D,
    KinodynamicStlCbsPath2D, KinodynamicStlCbsPlanner2D,
};
use rust_robotics::prelude::*;

fn agents() -> Vec<KinodynamicStlCbsAgent2D> {
    vec![
        KinodynamicStlCbsAgent2D::new(0, (0, 2), KinodynamicHeading2D::East, (6, 2)),
        KinodynamicStlCbsAgent2D::new(1, (6, 2), KinodynamicHeading2D::West, (0, 2)),
    ]
}

fn planner() -> RoboticsResult<KinodynamicStlCbsPlanner2D> {
    KinodynamicStlCbsPlanner2D::new(KinodynamicStlCbsConfig2D {
        max_time: 32,
        max_cbs_nodes: 4_096,
        move_duration: 2,
        turn_duration: 1,
        ..KinodynamicStlCbsConfig2D::new(7, 5)
    })
}

fn path_string(path: &KinodynamicStlCbsPath2D) -> String {
    path.poses
        .iter()
        .filter(|pose| pose.t == 0 || pose.t == path.arrival_time() || pose.t % 2 == 0)
        .map(|pose| {
            format!(
                "({}, {}, {}, t={})",
                pose.x,
                pose.y,
                pose.heading.label(),
                pose.t
            )
        })
        .collect::<Vec<_>>()
        .join(" -> ")
}

fn main() -> RoboticsResult<()> {
    let planner = planner()?;
    let agents = agents();
    let independent = planner.plan_independent(&agents)?;
    let independent_cells = independent
        .iter()
        .map(KinodynamicStlCbsPath2D::cell_path)
        .collect::<Vec<_>>();
    let plan = planner.plan(&agents)?;

    println!(
        "kinodynamic-stl-cbs: independent_conflict={} final_conflict={} continuous_conflict={} resolved={} expanded={} total_cost={} separation={:.2} continuous_separation={:.2}",
        first_conflict(&independent_cells, planner.config().max_time).is_some(),
        first_conflict(&plan.cell_paths, planner.config().max_time).is_some(),
        plan.first_continuous_conflict.is_some(),
        plan.conflicts_resolved,
        plan.high_level_nodes_expanded,
        plan.total_cost,
        plan.min_pairwise_separation_robustness,
        plan.min_continuous_pairwise_separation_robustness
    );
    for path in &plan.paths {
        println!(
            "  agent {} arrival={} path={}",
            path.agent_id,
            path.arrival_time(),
            path_string(path)
        );
    }

    Ok(())
}
