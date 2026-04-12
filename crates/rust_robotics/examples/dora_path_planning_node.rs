//! Minimal dora-rs planning node example.
//!
//! The node waits for a timer tick, runs a headless A* path planning query,
//! and emits a structured JSON report of the generated path.

#[path = "support/dora_path_payload.rs"]
mod dora_path_payload;

use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use dora_path_payload::PathPlanningReport;
use eyre::{bail, Result};
use rust_robotics::planning::a_star::{AStarConfig, AStarPlanner};
use rust_robotics::prelude::*;

fn create_obstacles() -> Obstacles {
    let mut obstacles = Obstacles::new();

    for i in 0..=20 {
        obstacles.push(Point2D::new(i as f64, 0.0));
        obstacles.push(Point2D::new(i as f64, 20.0));
        obstacles.push(Point2D::new(0.0, i as f64));
        obstacles.push(Point2D::new(20.0, i as f64));
    }

    for i in 5..15 {
        obstacles.push(Point2D::new(10.0, i as f64));
    }

    obstacles
}

fn main() -> Result<()> {
    let output_id = DataId::from("path-report".to_owned());
    let obstacles = create_obstacles();
    let start = Point2D::new(2.0, 10.0);
    let goal = Point2D::new(18.0, 10.0);
    let planner = AStarPlanner::from_obstacle_points(
        &obstacles,
        AStarConfig {
            resolution: 1.0,
            robot_radius: 0.5,
            heuristic_weight: 1.0,
        },
    )?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut sent_summary = false;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                if sent_summary {
                    break;
                } else {
                    let path = planner.plan(start, goal)?;
                    let report = PathPlanningReport::from_path(&path, start, goal);
                    let payload = report.to_json()?;
                    node.send_output(output_id.clone(), metadata.parameters, payload.into_arrow())?;
                    sent_summary = true;
                }
            }
            Event::Input { id, .. } => eprintln!("ignoring unexpected input `{id}`"),
            Event::InputClosed { id } => {
                if id.as_str() == "tick" {
                    if sent_summary {
                        break;
                    }
                    bail!("tick input closed before any path report was sent");
                }
            }
            Event::Stop(_) => break,
            Event::Reload { .. } | Event::Error(_) => {}
            _ => {}
        }
    }

    if !sent_summary {
        bail!("planner exited before sending a path report");
    }

    Ok(())
}
