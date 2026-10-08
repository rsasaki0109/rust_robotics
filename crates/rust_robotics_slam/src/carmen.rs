//! CARMEN log reader for 2D laser datasets (e.g. the Intel Research Lab and
//! Freiburg logs of the Kümmerle et al. SLAM benchmark).
//!
//! Supported lines:
//!
//! ```text
//! FLASER n r1 … rn x y theta odom_x odom_y odom_theta ipc_timestamp ipc_hostname logger_timestamp
//! ROBOTLASER1 type start_angle fov resolution max_range accuracy remission_mode
//!             n r1 … rn m e1 … em laser_x laser_y laser_theta robot_x robot_y robot_theta
//!             tv rv forward_safety side_safety turn_axis ipc_timestamp ipc_hostname logger_timestamp
//! ```
//!
//! `FLASER` carries no beam geometry; the CARMEN convention for front lasers
//! is used: `angle_min = -π/2`, `angle_increment = π / n`. Other message types
//! (`ODOM`, `PARAM`, comments, …) are skipped.

use nalgebra::Vector2;
use rust_robotics_core::Pose2D;

/// One laser scan from a CARMEN log.
#[derive(Debug, Clone, PartialEq)]
pub struct CarmenScan {
    /// IPC timestamp \[s\] (the time the relations of the SLAM benchmark use).
    pub timestamp: f64,
    /// Bearing of the first beam in the laser frame \[rad\].
    pub angle_min: f64,
    /// Angle between consecutive beams \[rad\].
    pub angle_increment: f64,
    /// Maximum range of the sensor \[m\] (`f64::INFINITY` when unknown).
    pub max_range: f64,
    /// Measured ranges \[m\], in beam order.
    pub ranges: Vec<f64>,
    /// Laser pose from odometry (the pose of the scan in the odometry frame).
    pub laser_odometry: Pose2D,
}

impl CarmenScan {
    /// Beam-ordered points in the laser frame, keeping ranges in
    /// `(min_range, max_usable_range)` that are also below the sensor maximum.
    pub fn points(&self, min_range: f64, max_usable_range: f64) -> Vec<Vector2<f64>> {
        let limit = max_usable_range.min(self.max_range);
        self.ranges
            .iter()
            .enumerate()
            .filter(|(_, range)| range.is_finite() && **range > min_range && **range < limit)
            .map(|(beam, range)| {
                let angle = self.angle_min + self.angle_increment * beam as f64;
                Vector2::new(range * angle.cos(), range * angle.sin())
            })
            .collect()
    }
}

/// Error raised for a malformed laser line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CarmenParseError {
    /// 1-based line number.
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for CarmenParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for CarmenParseError {}

struct Fields<'a> {
    tokens: Vec<&'a str>,
    next: usize,
    line: usize,
}

impl<'a> Fields<'a> {
    fn error(&self, message: impl Into<String>) -> CarmenParseError {
        CarmenParseError {
            line: self.line,
            message: message.into(),
        }
    }

    fn token(&mut self, what: &str) -> Result<&'a str, CarmenParseError> {
        let token = self
            .tokens
            .get(self.next)
            .copied()
            .ok_or_else(|| self.error(format!("missing {what}")))?;
        self.next += 1;
        Ok(token)
    }

    fn number(&mut self, what: &str) -> Result<f64, CarmenParseError> {
        let token = self.token(what)?;
        token
            .parse::<f64>()
            .map_err(|_| self.error(format!("invalid {what} `{token}`")))
    }

    fn count(&mut self, what: &str) -> Result<usize, CarmenParseError> {
        let value = self.number(what)?;
        if value < 0.0 || value.fract() != 0.0 || value > 100_000.0 {
            return Err(self.error(format!("invalid {what} `{value}`")));
        }
        Ok(value as usize)
    }

    fn numbers(&mut self, count: usize, what: &str) -> Result<Vec<f64>, CarmenParseError> {
        (0..count).map(|_| self.number(what)).collect()
    }

    fn pose(&mut self, what: &str) -> Result<Pose2D, CarmenParseError> {
        Ok(Pose2D::new(
            self.number(what)?,
            self.number(what)?,
            self.number(what)?,
        ))
    }
}

/// Parses every `FLASER` and `ROBOTLASER1` line of a CARMEN log.
pub fn parse_carmen_log(text: &str) -> Result<Vec<CarmenScan>, CarmenParseError> {
    let mut scans = Vec::new();
    for (index, line) in text.lines().enumerate() {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        let Some(kind) = tokens.first().copied() else {
            continue;
        };
        let mut fields = Fields {
            tokens,
            next: 1,
            line: index + 1,
        };
        let scan = match kind {
            "FLASER" => parse_flaser(&mut fields)?,
            "ROBOTLASER1" => parse_robotlaser(&mut fields)?,
            _ => continue,
        };
        scans.push(scan);
    }
    Ok(scans)
}

fn parse_flaser(fields: &mut Fields<'_>) -> Result<CarmenScan, CarmenParseError> {
    let count = fields.count("number of readings")?;
    let ranges = fields.numbers(count, "range")?;
    let laser_odometry = fields.pose("laser pose")?;
    let _robot_odometry = fields.pose("odometry pose")?;
    let timestamp = fields.number("ipc timestamp")?;
    Ok(CarmenScan {
        timestamp,
        angle_min: -std::f64::consts::FRAC_PI_2,
        angle_increment: std::f64::consts::PI / count.max(1) as f64,
        max_range: f64::INFINITY,
        ranges,
        laser_odometry,
    })
}

fn parse_robotlaser(fields: &mut Fields<'_>) -> Result<CarmenScan, CarmenParseError> {
    let _laser_type = fields.number("laser type")?;
    let angle_min = fields.number("start angle")?;
    let _field_of_view = fields.number("field of view")?;
    let angle_increment = fields.number("angular resolution")?;
    let max_range = fields.number("maximum range")?;
    let _accuracy = fields.number("accuracy")?;
    let _remission_mode = fields.number("remission mode")?;
    let count = fields.count("number of readings")?;
    let ranges = fields.numbers(count, "range")?;
    let remissions = fields.count("number of remissions")?;
    fields.numbers(remissions, "remission")?;
    let laser_odometry = fields.pose("laser pose")?;
    let _robot_odometry = fields.pose("robot pose")?;
    fields.numbers(5, "velocity / safety field")?;
    let timestamp = fields.number("ipc timestamp")?;
    Ok(CarmenScan {
        timestamp,
        angle_min,
        angle_increment,
        max_range,
        ranges,
        laser_odometry,
    })
}

/// Formats a scan as a CARMEN `FLASER` line (robot odometry = laser pose).
///
/// The scan must follow the `FLASER` beam convention (`-π/2`, `π / n`).
pub fn format_flaser(scan: &CarmenScan) -> String {
    let pose = scan.laser_odometry;
    let ranges: Vec<String> = scan
        .ranges
        .iter()
        .map(|range| format!("{range:.3}"))
        .collect();
    format!(
        "FLASER {} {} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} sim {:.6}",
        scan.ranges.len(),
        ranges.join(" "),
        pose.x,
        pose.y,
        pose.yaw,
        pose.x,
        pose.y,
        pose.yaw,
        scan.timestamp,
        scan.timestamp,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_flaser_and_robotlaser_and_skips_other_lines() {
        let log = "\
# comment
PARAM robot_front_laser_max 81.9
ODOM 0.0 0.0 0.0 0.0 0.0 0.0 10.0 host 10.0
FLASER 4 1.0 2.0 3.0 4.0 0.5 -0.5 0.1 0.6 -0.4 0.2 12.5 host 12.6
ROBOTLASER1 0 -1.5708 3.1416 0.0175 50.0 0.01 0 3 1.0 60.0 2.0 0 1.0 2.0 0.3 1.1 2.1 0.31 0 0 0 0 0 13.0 host 13.1
";
        let scans = parse_carmen_log(log).expect("valid log");
        assert_eq!(scans.len(), 2);

        let flaser = &scans[0];
        assert_eq!(flaser.ranges, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(flaser.laser_odometry, Pose2D::new(0.5, -0.5, 0.1));
        assert_eq!(flaser.timestamp, 12.5);
        assert!((flaser.angle_increment - std::f64::consts::PI / 4.0).abs() < 1e-12);

        let robotlaser = &scans[1];
        assert_eq!(robotlaser.ranges, vec![1.0, 60.0, 2.0]);
        assert_eq!(robotlaser.max_range, 50.0);
        assert_eq!(robotlaser.laser_odometry, Pose2D::new(1.0, 2.0, 0.3));
        assert_eq!(robotlaser.timestamp, 13.0);
        // The 60 m reading exceeds the 50 m sensor maximum.
        assert_eq!(robotlaser.points(0.05, 80.0).len(), 2);
    }

    #[test]
    fn points_follow_the_beam_geometry() {
        let scan = CarmenScan {
            timestamp: 0.0,
            angle_min: -std::f64::consts::FRAC_PI_2,
            angle_increment: std::f64::consts::FRAC_PI_2,
            max_range: f64::INFINITY,
            ranges: vec![1.0, 2.0, 3.0],
            laser_odometry: Pose2D::origin(),
        };
        let points = scan.points(0.0, 10.0);
        assert!((points[0] - Vector2::new(0.0, -1.0)).norm() < 1e-12);
        assert!((points[1] - Vector2::new(2.0, 0.0)).norm() < 1e-12);
        assert!((points[2] - Vector2::new(0.0, 3.0)).norm() < 1e-12);
        assert_eq!(scan.points(0.0, 2.5).len(), 2);
    }

    #[test]
    fn flaser_round_trips_through_format_and_parse() {
        let scan = CarmenScan {
            timestamp: 42.125,
            angle_min: -std::f64::consts::FRAC_PI_2,
            angle_increment: std::f64::consts::PI / 3.0,
            max_range: f64::INFINITY,
            ranges: vec![1.25, 2.5, 3.75],
            laser_odometry: Pose2D::new(1.0, -2.0, 0.5),
        };
        let parsed = parse_carmen_log(&format_flaser(&scan)).expect("valid line");
        assert_eq!(parsed, vec![scan]);
    }

    #[test]
    fn reports_truncated_lines() {
        let error = parse_carmen_log("ODOM 1 2 3\nFLASER 3 1.0 2.0\n").unwrap_err();
        assert_eq!(error.line, 2);
        assert!(error.message.contains("range"), "{}", error.message);
    }
}
