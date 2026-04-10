//! Minimal dora-rs sink for the planning example.

#[path = "support/dora_path_payload.rs"]
mod dora_path_payload;

use dora_node_api::{DoraNode, Event};
use dora_path_payload::PathPlanningReport;
use eyre::{bail, Context, Result};

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut received_summary = false;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "path" => {
                let payload: &str =
                    TryFrom::try_from(&data).context("expected UTF-8 path report")?;
                if payload.is_empty() {
                    bail!("received empty path report");
                }
                let report = PathPlanningReport::from_json(payload)?;
                received_summary = true;
                println!(
                    "received path report: planner={} waypoints={} segments={} length={:.2} direct={:.2} start=({:.1},{:.1}) goal=({:.1},{:.1})",
                    report.planner,
                    report.waypoint_count,
                    report.segment_count(),
                    report.path_length,
                    report.direct_distance(),
                    report.start.x,
                    report.start.y,
                    report.goal.x,
                    report.goal.y,
                );
                break;
            }
            Event::Input { id, .. } => eprintln!("ignoring unexpected input `{id}`"),
            Event::InputClosed { id } => {
                if id.as_str() == "path" {
                    if received_summary {
                        break;
                    }
                    bail!("path input closed before any path report was received");
                }
            }
            Event::Stop(_) => break,
            Event::Reload { .. } | Event::Error(_) => {}
            _ => {}
        }
    }

    if !received_summary {
        bail!("sink exited before receiving a path report");
    }

    Ok(())
}
