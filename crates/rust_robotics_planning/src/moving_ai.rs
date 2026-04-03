//! Loader utilities for the Moving AI Lab grid benchmark formats.
//!
//! These helpers parse `.map` and `.scen` files and convert them into the
//! occupancy-grid obstacle format used by the existing planners.

use std::fs;
use std::path::Path;

use rust_robotics_core::{Obstacles, Point2D, RoboticsError, RoboticsResult};

/// Parsed Moving AI Lab map.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MovingAiMap {
    pub width: usize,
    pub height: usize,
    tiles: Vec<Vec<char>>,
}

impl MovingAiMap {
    /// Parse a MovingAI `.map` payload.
    pub fn parse_str(input: &str) -> RoboticsResult<Self> {
        let mut lines = input
            .lines()
            .map(str::trim_end)
            .filter(|line| !line.trim().is_empty());

        let map_type = lines.next().ok_or_else(|| {
            RoboticsError::InvalidParameter("MovingAI map is missing the type header".to_string())
        })?;
        if map_type.trim() != "type octile" {
            return Err(RoboticsError::InvalidParameter(format!(
                "unsupported MovingAI map type {:?}, expected \"type octile\"",
                map_type
            )));
        }

        let height = parse_dimension(
            lines.next().ok_or_else(|| {
                RoboticsError::InvalidParameter(
                    "MovingAI map is missing the height header".to_string(),
                )
            })?,
            "height",
        )?;
        let width = parse_dimension(
            lines.next().ok_or_else(|| {
                RoboticsError::InvalidParameter(
                    "MovingAI map is missing the width header".to_string(),
                )
            })?,
            "width",
        )?;

        let map_marker = lines.next().ok_or_else(|| {
            RoboticsError::InvalidParameter("MovingAI map is missing the map marker".to_string())
        })?;
        if map_marker.trim() != "map" {
            return Err(RoboticsError::InvalidParameter(format!(
                "expected \"map\" marker, got {:?}",
                map_marker
            )));
        }

        let mut tiles = Vec::with_capacity(height);
        for row_index in 0..height {
            let row = lines.next().ok_or_else(|| {
                RoboticsError::InvalidParameter(format!(
                    "MovingAI map ended early at row {} of {}",
                    row_index, height
                ))
            })?;
            let chars: Vec<char> = row.chars().collect();
            if chars.len() != width {
                return Err(RoboticsError::InvalidParameter(format!(
                    "row {} has width {}, expected {}",
                    row_index,
                    chars.len(),
                    width
                )));
            }
            for tile in &chars {
                if !is_supported_tile(*tile) {
                    return Err(RoboticsError::InvalidParameter(format!(
                        "unsupported MovingAI tile {:?} at row {}",
                        tile, row_index
                    )));
                }
            }
            tiles.push(chars);
        }

        Ok(Self {
            width,
            height,
            tiles,
        })
    }

    /// Load a MovingAI `.map` file from disk.
    pub fn from_file(path: impl AsRef<Path>) -> RoboticsResult<Self> {
        let content = fs::read_to_string(path)?;
        Self::parse_str(&content)
    }

    /// Return whether a tile is passable in the occupancy-only approximation.
    pub fn is_passable(&self, x: usize, y: usize) -> RoboticsResult<bool> {
        let tile = self.tile_at(x, y)?;
        Ok(matches!(tile, '.' | 'G' | 'S' | 'W'))
    }

    /// Convert the map into obstacle points for the existing planners.
    ///
    /// The generated obstacle set adds a one-cell border so edge cells remain
    /// addressable while outside-the-map coordinates stay invalid.
    pub fn to_obstacles(&self) -> Obstacles {
        let mut obstacles = Obstacles::new();
        let width = self.width as i32;
        let height = self.height as i32;

        for x in 0..=width + 1 {
            obstacles.push(Point2D::new(x as f64, 0.0));
            obstacles.push(Point2D::new(x as f64, (height + 1) as f64));
        }
        for y in 0..=height + 1 {
            obstacles.push(Point2D::new(0.0, y as f64));
            obstacles.push(Point2D::new((width + 1) as f64, y as f64));
        }

        for y in 0..self.height {
            for x in 0..self.width {
                if !self.is_passable(x, y).expect("bounds checked by iteration") {
                    obstacles.push(Point2D::new((x + 1) as f64, (y + 1) as f64));
                }
            }
        }

        obstacles
    }

    /// Translate a MovingAI grid coordinate into planner world coordinates.
    pub fn planning_point(&self, x: usize, y: usize) -> RoboticsResult<Point2D> {
        if !self.is_passable(x, y)? {
            return Err(RoboticsError::InvalidParameter(format!(
                "scenario point ({}, {}) is not traversable",
                x, y
            )));
        }
        Ok(Point2D::new((x + 1) as f64, (y + 1) as f64))
    }

    fn tile_at(&self, x: usize, y: usize) -> RoboticsResult<char> {
        if x >= self.width || y >= self.height {
            return Err(RoboticsError::InvalidParameter(format!(
                "tile coordinate ({}, {}) is out of bounds for {}x{} map",
                x, y, self.width, self.height
            )));
        }
        Ok(self.tiles[y][x])
    }
}

/// One scenario row from a MovingAI `.scen` file.
#[derive(Debug, Clone, PartialEq)]
pub struct MovingAiScenario {
    pub bucket: u32,
    pub map: String,
    pub width: usize,
    pub height: usize,
    pub start_x: usize,
    pub start_y: usize,
    pub goal_x: usize,
    pub goal_y: usize,
    pub optimal_length: f64,
}

impl MovingAiScenario {
    /// Parse a MovingAI `.scen` payload.
    pub fn parse_str(input: &str) -> RoboticsResult<Vec<Self>> {
        let mut lines = input.lines().map(str::trim).filter(|line| !line.is_empty());

        let version = lines.next().ok_or_else(|| {
            RoboticsError::InvalidParameter(
                "MovingAI scenario file is missing the version header".to_string(),
            )
        })?;
        if !version.starts_with("version ") {
            return Err(RoboticsError::InvalidParameter(format!(
                "expected scenario version header, got {:?}",
                version
            )));
        }

        let mut scenarios = Vec::new();
        for (line_index, line) in lines.enumerate() {
            let fields: Vec<&str> = line.split_whitespace().collect();
            if fields.len() != 9 {
                return Err(RoboticsError::InvalidParameter(format!(
                    "scenario line {} has {} fields, expected 9",
                    line_index + 2,
                    fields.len()
                )));
            }

            let scenario = Self {
                bucket: parse_integer(fields[0], "bucket", line_index)?,
                map: fields[1].to_string(),
                width: parse_integer(fields[2], "map width", line_index)?,
                height: parse_integer(fields[3], "map height", line_index)?,
                start_x: parse_integer(fields[4], "start x", line_index)?,
                start_y: parse_integer(fields[5], "start y", line_index)?,
                goal_x: parse_integer(fields[6], "goal x", line_index)?,
                goal_y: parse_integer(fields[7], "goal y", line_index)?,
                optimal_length: parse_float(fields[8], "optimal length", line_index)?,
            };
            scenarios.push(scenario);
        }

        if scenarios.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "scenario file does not contain any scenario rows".to_string(),
            ));
        }

        Ok(scenarios)
    }

    /// Load MovingAI scenarios from disk.
    pub fn from_file(path: impl AsRef<Path>) -> RoboticsResult<Vec<Self>> {
        let content = fs::read_to_string(path)?;
        Self::parse_str(&content)
    }
}

fn parse_dimension(line: &str, field: &str) -> RoboticsResult<usize> {
    let value = line
        .strip_prefix(&format!("{field} "))
        .ok_or_else(|| {
            RoboticsError::InvalidParameter(format!("expected {:?} header, got {:?}", field, line))
        })?
        .trim();
    value.parse::<usize>().map_err(|err| {
        RoboticsError::InvalidParameter(format!(
            "failed to parse {} from {:?}: {}",
            field, line, err
        ))
    })
}

fn parse_integer<T>(value: &str, field: &str, line_index: usize) -> RoboticsResult<T>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    value.parse::<T>().map_err(|err| {
        RoboticsError::InvalidParameter(format!(
            "failed to parse {} on scenario line {}: {}",
            field,
            line_index + 2,
            err
        ))
    })
}

fn parse_float(value: &str, field: &str, line_index: usize) -> RoboticsResult<f64> {
    let parsed = value.parse::<f64>().map_err(|err| {
        RoboticsError::InvalidParameter(format!(
            "failed to parse {} on scenario line {}: {}",
            field,
            line_index + 2,
            err
        ))
    })?;
    if !parsed.is_finite() {
        return Err(RoboticsError::InvalidParameter(format!(
            "{} on scenario line {} must be finite",
            field,
            line_index + 2
        )));
    }
    Ok(parsed)
}

fn is_supported_tile(tile: char) -> bool {
    matches!(tile, '.' | 'G' | '@' | 'O' | 'T' | 'S' | 'W')
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::a_star::{AStarConfig, AStarPlanner};

    const SAMPLE_MAP: &str = include_str!("testdata/moving_ai/sample.map");
    const SAMPLE_SCEN: &str = include_str!("testdata/moving_ai/sample.map.scen");

    #[test]
    fn parses_sample_map() {
        let map = MovingAiMap::parse_str(SAMPLE_MAP).expect("sample map should parse");

        assert_eq!(map.width, 5);
        assert_eq!(map.height, 5);
        assert!(map.is_passable(0, 0).unwrap());
        assert!(!map.is_passable(2, 0).unwrap());
        assert!(!map.is_passable(2, 1).unwrap());
        assert!(!map.is_passable(2, 2).unwrap());
        assert!(map.is_passable(1, 1).unwrap());
        assert!(map.is_passable(2, 3).unwrap());
    }

    #[test]
    fn parses_sample_scenarios() {
        let scenarios =
            MovingAiScenario::parse_str(SAMPLE_SCEN).expect("sample scenario should parse");

        assert_eq!(scenarios.len(), 2);
        assert_eq!(scenarios[0].map, "sample.map");
        assert_eq!(scenarios[0].start_x, 0);
        assert_eq!(scenarios[0].goal_y, 4);
        assert!((scenarios[1].optimal_length - std::f64::consts::SQRT_2).abs() < 1e-9);
    }

    #[test]
    fn sample_scenario_is_solved_by_astar() {
        let map = MovingAiMap::parse_str(SAMPLE_MAP).expect("sample map should parse");
        let scenario = MovingAiScenario::parse_str(SAMPLE_SCEN)
            .expect("sample scenario should parse")
            .into_iter()
            .next()
            .expect("sample scenario should contain at least one row");

        let planner = AStarPlanner::from_obstacle_points(
            &map.to_obstacles(),
            AStarConfig {
                resolution: 1.0,
                robot_radius: 0.5,
                heuristic_weight: 1.0,
            },
        )
        .expect("A* planner should build from sample map");

        let path = planner
            .plan(
                map.planning_point(scenario.start_x, scenario.start_y)
                    .expect("sample start should be valid"),
                map.planning_point(scenario.goal_x, scenario.goal_y)
                    .expect("sample goal should be valid"),
            )
            .expect("sample scenario should be solvable");

        assert!(!path.is_empty());
        assert!((path.total_length() - scenario.optimal_length).abs() < 1e-9);
    }
}
