//! Headless rigid-body MIP-style planning example.

use std::f64::consts::TAU;

use rust_robotics::planning::{
    RigidBodyConvexObstacle2D, RigidBodyMipConfig2D, RigidBodyMipPlanner2D, RigidBodyPoint2D,
    RigidBodyPose2D,
};
use rust_robotics::prelude::*;

fn scenario() -> RoboticsResult<(RigidBodyMipPlanner2D, RigidBodyPose2D, RigidBodyPose2D)> {
    let obstacles = vec![
        RigidBodyConvexObstacle2D::convex_polygon(vec![
            RigidBodyPoint2D::new(3.0, 0.0),
            RigidBodyPoint2D::new(7.0, 0.0),
            RigidBodyPoint2D::new(7.0, 2.6),
            RigidBodyPoint2D::new(3.4, 2.6),
            RigidBodyPoint2D::new(3.0, 2.2),
        ])?,
        RigidBodyConvexObstacle2D::convex_polygon(vec![
            RigidBodyPoint2D::new(3.0, 3.8),
            RigidBodyPoint2D::new(3.4, 3.4),
            RigidBodyPoint2D::new(7.0, 3.4),
            RigidBodyPoint2D::new(7.0, 6.0),
            RigidBodyPoint2D::new(3.0, 6.0),
        ])?,
    ];
    let planner = RigidBodyMipPlanner2D::new(RigidBodyMipConfig2D {
        obstacles,
        max_expansions: 50_000,
        ..RigidBodyMipConfig2D::new(0.0, 10.0, 0.0, 6.0)
    })?;
    let start = RigidBodyPose2D::new(1.0, 3.0, TAU / 4.0);
    let goal = RigidBodyPose2D::new(9.0, 3.0, 0.0);
    Ok((planner, start, goal))
}

fn heading_degrees(theta: f64) -> i32 {
    theta.to_degrees().round() as i32
}

fn main() -> RoboticsResult<()> {
    let (planner, start, goal) = scenario()?;
    let plan = planner.plan(start, goal, true)?;

    println!(
        "rigid-body-mip: poses={} total_cost={} expanded={} binary_choices={} segment_choices={} min_margin={:.3} min_segment_margin={:.3}",
        plan.poses.len(),
        plan.total_cost,
        plan.expanded_states,
        plan.binary_separation_choices,
        plan.segment_binary_separation_choices,
        plan.min_separation_margin,
        plan.min_segment_separation_margin
    );
    for (index, pose) in plan.poses.iter().enumerate() {
        if index == 0 || index + 1 == plan.poses.len() || index % 3 == 0 {
            let certs = &plan.certificates[index];
            let cert_text = certs
                .iter()
                .map(|cert| {
                    format!(
                        "obs{}:h{}:{:.2}",
                        cert.obstacle_index, cert.halfspace_index, cert.margin
                    )
                })
                .collect::<Vec<_>>()
                .join(",");
            println!(
                "  {:02}: x={:.1} y={:.1} theta={}deg certs=[{}]",
                index,
                pose.x,
                pose.y,
                heading_degrees(pose.theta),
                cert_text
            );
        }
    }

    Ok(())
}
