//! Dora-rs node that consumes a path report semantically.
//!
//! Receives a `PathPlanningReport` from the planner node, computes per-segment
//! quality metrics (segment lengths, turning angles, path efficiency), and
//! emits a JSON metrics report as output.

#[path = "support/dora_path_payload.rs"]
mod dora_path_payload;

use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use dora_path_payload::{PathPlanningReport, PathWaypoint};
use eyre::{bail, Context, Result};
use serde::Serialize;

#[derive(Debug, Serialize)]
struct PathMetricsReport {
    planner: String,
    waypoint_count: usize,
    segment_count: usize,
    path_length: f64,
    direct_distance: f64,
    efficiency: f64,
    mean_segment_length: f64,
    min_segment_length: f64,
    max_segment_length: f64,
    mean_turning_angle_deg: f64,
    max_turning_angle_deg: f64,
}

fn segment_length(a: PathWaypoint, b: PathWaypoint) -> f64 {
    ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
}

fn turning_angle_deg(a: PathWaypoint, b: PathWaypoint, c: PathWaypoint) -> f64 {
    let v1x = b.x - a.x;
    let v1y = b.y - a.y;
    let v2x = c.x - b.x;
    let v2y = c.y - b.y;
    let dot = v1x * v2x + v1y * v2y;
    let cross = v1x * v2y - v1y * v2x;
    cross.atan2(dot).abs().to_degrees()
}

fn compute_metrics(report: &PathPlanningReport) -> PathMetricsReport {
    let segments: Vec<f64> = report
        .waypoints
        .windows(2)
        .map(|w| segment_length(w[0], w[1]))
        .collect();

    let segment_count = segments.len();
    let mean_segment_length = if segment_count > 0 {
        segments.iter().sum::<f64>() / segment_count as f64
    } else {
        0.0
    };
    let min_segment_length = segments.iter().copied().fold(f64::INFINITY, f64::min);
    let max_segment_length = segments.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    let turning_angles: Vec<f64> = report
        .waypoints
        .windows(3)
        .map(|w| turning_angle_deg(w[0], w[1], w[2]))
        .collect();

    let mean_turning_angle_deg = if turning_angles.is_empty() {
        0.0
    } else {
        turning_angles.iter().sum::<f64>() / turning_angles.len() as f64
    };
    let max_turning_angle_deg = turning_angles.iter().copied().fold(0.0_f64, f64::max);

    let direct_distance = report.direct_distance();
    let efficiency = if report.path_length > 0.0 {
        direct_distance / report.path_length
    } else {
        0.0
    };

    PathMetricsReport {
        planner: report.planner.clone(),
        waypoint_count: report.waypoint_count,
        segment_count,
        path_length: report.path_length,
        direct_distance,
        efficiency,
        mean_segment_length,
        min_segment_length,
        max_segment_length,
        mean_turning_angle_deg,
        max_turning_angle_deg,
    }
}

fn main() -> Result<()> {
    let output_id = DataId::from("path-metrics".to_owned());
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut processed = false;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } if id.as_str() == "path" => {
                let payload: &str =
                    TryFrom::try_from(&data).context("expected UTF-8 path report")?;
                if payload.is_empty() {
                    bail!("received empty path report");
                }
                let report = PathPlanningReport::from_json(payload)?;
                let metrics = compute_metrics(&report);

                println!(
                    "path metrics: efficiency={:.4} segments={} mean_seg={:.2} turns={:.1}deg",
                    metrics.efficiency,
                    metrics.segment_count,
                    metrics.mean_segment_length,
                    metrics.mean_turning_angle_deg,
                );

                let json =
                    serde_json::to_string(&metrics).context("failed to serialize path metrics")?;
                node.send_output(output_id.clone(), metadata.parameters, json.into_arrow())?;
                processed = true;
                break;
            }
            Event::Input { id, .. } => eprintln!("ignoring unexpected input `{id}`"),
            Event::InputClosed { id } => {
                if id.as_str() == "path" {
                    if processed {
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

    if !processed {
        bail!("metrics node exited before processing a path report");
    }

    Ok(())
}
