use eyre::{bail, Context, Result};
use rust_robotics::prelude::*;
use serde::{Deserialize, Serialize};

const PAYLOAD_VERSION: u32 = 1;
const COORD_EPSILON: f64 = 1e-9;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PathWaypoint {
    pub x: f64,
    pub y: f64,
}

impl PathWaypoint {
    pub fn from_point(point: Point2D) -> Self {
        Self {
            x: point.x,
            y: point.y,
        }
    }

    fn approx_eq(self, other: Self) -> bool {
        (self.x - other.x).abs() <= COORD_EPSILON && (self.y - other.y).abs() <= COORD_EPSILON
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathPlanningReport {
    pub version: u32,
    pub planner: String,
    pub start: PathWaypoint,
    pub goal: PathWaypoint,
    pub waypoint_count: usize,
    pub path_length: f64,
    pub waypoints: Vec<PathWaypoint>,
}

#[allow(dead_code)]
impl PathPlanningReport {
    pub fn from_path(path: &Path2D, start: Point2D, goal: Point2D) -> Self {
        Self {
            version: PAYLOAD_VERSION,
            planner: "A*".to_owned(),
            start: PathWaypoint::from_point(start),
            goal: PathWaypoint::from_point(goal),
            waypoint_count: path.len(),
            path_length: path.total_length(),
            waypoints: path
                .points
                .iter()
                .copied()
                .map(PathWaypoint::from_point)
                .collect(),
        }
    }

    pub fn to_json(&self) -> Result<String> {
        self.validate()?;
        serde_json::to_string(self).context("failed to serialize path planning report")
    }

    pub fn from_json(raw: &str) -> Result<Self> {
        let report: Self =
            serde_json::from_str(raw).context("failed to deserialize path planning report")?;
        report.validate()?;
        Ok(report)
    }

    pub fn validate(&self) -> Result<()> {
        if self.version != PAYLOAD_VERSION {
            bail!(
                "unsupported path planning report version {}, expected {}",
                self.version,
                PAYLOAD_VERSION
            );
        }
        if self.planner.is_empty() {
            bail!("path planning report is missing planner name");
        }
        if !self.path_length.is_finite() || self.path_length < 0.0 {
            bail!(
                "path planning report has invalid path_length {}",
                self.path_length
            );
        }
        if self.waypoints.is_empty() {
            bail!("path planning report contains no waypoints");
        }
        if self.waypoint_count != self.waypoints.len() {
            bail!(
                "path planning report waypoint_count {} does not match payload length {}",
                self.waypoint_count,
                self.waypoints.len()
            );
        }
        let first = self.waypoints[0];
        if !first.approx_eq(self.start) {
            bail!("path planning report start does not match first waypoint");
        }
        let last = self.waypoints[self.waypoints.len() - 1];
        if !last.approx_eq(self.goal) {
            bail!("path planning report goal does not match last waypoint");
        }
        Ok(())
    }

    pub fn segment_count(&self) -> usize {
        self.waypoint_count.saturating_sub(1)
    }

    pub fn direct_distance(&self) -> f64 {
        ((self.goal.x - self.start.x).powi(2) + (self.goal.y - self.start.y).powi(2)).sqrt()
    }
}
