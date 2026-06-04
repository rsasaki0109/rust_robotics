//! Adaptive movable-obstacle costmap slice inspired by Costmap-NAMO.
//!
//! This module models the core planning-side behavior without ROS2/Nav2:
//! partially known cells can be labeled movable, their cost is adapted from
//! commanded-vs-actual progress, and the result can be converted into a
//! traversal-risk grid for graph planning.

use rust_robotics_core::{RoboticsError, RoboticsResult};

use crate::traversal_risk_graph::TerrainRiskCell;

/// Semantic state of one adaptive costmap cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdaptiveCostmapCellState {
    Free,
    Unknown,
    StaticObstacle,
    MovableObstacle,
}

/// One adaptive costmap cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveCostmapCell {
    pub state: AdaptiveCostmapCellState,
    pub cost: f64,
}

impl AdaptiveCostmapCell {
    pub fn free() -> Self {
        Self {
            state: AdaptiveCostmapCellState::Free,
            cost: 0.0,
        }
    }
}

/// Configuration for adaptive movable-obstacle cost updates.
#[derive(Debug, Clone, PartialEq)]
pub struct AdaptiveCostmapNamoConfig {
    pub width: i32,
    pub height: i32,
    pub unknown_cost: f64,
    pub movable_initial_cost: f64,
    pub movable_cost_increment: f64,
    pub movable_cost_decrement: f64,
    pub static_obstacle_cost: f64,
    pub lethal_cost: f64,
    pub stuck_command_speed: f64,
    pub stuck_actual_speed_ratio: f64,
    pub progress_distance: f64,
}

impl AdaptiveCostmapNamoConfig {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            width,
            height,
            unknown_cost: 25.0,
            movable_initial_cost: 20.0,
            movable_cost_increment: 30.0,
            movable_cost_decrement: 15.0,
            static_obstacle_cost: 100.0,
            lethal_cost: 100.0,
            stuck_command_speed: 0.05,
            stuck_actual_speed_ratio: 0.2,
            progress_distance: 0.02,
        }
    }
}

/// Commanded/observed motion over one update window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionProgressObservation {
    pub commanded_speed: f64,
    pub actual_speed: f64,
    pub odom_delta: f64,
}

impl MotionProgressObservation {
    pub fn stuck(commanded_speed: f64) -> Self {
        Self {
            commanded_speed,
            actual_speed: 0.0,
            odom_delta: 0.0,
        }
    }

    pub fn moving(commanded_speed: f64, actual_speed: f64, odom_delta: f64) -> Self {
        Self {
            commanded_speed,
            actual_speed,
            odom_delta,
        }
    }
}

/// Adaptive costmap state.
#[derive(Debug, Clone, PartialEq)]
pub struct AdaptiveCostmapNamo {
    config: AdaptiveCostmapNamoConfig,
    cells: Vec<Vec<AdaptiveCostmapCell>>,
}

impl AdaptiveCostmapNamo {
    pub fn new(config: AdaptiveCostmapNamoConfig) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let cells =
            vec![vec![AdaptiveCostmapCell::free(); config.height as usize]; config.width as usize];
        Ok(Self { config, cells })
    }

    pub fn width(&self) -> i32 {
        self.config.width
    }

    pub fn height(&self) -> i32 {
        self.config.height
    }

    pub fn cell(&self, x: i32, y: i32) -> RoboticsResult<AdaptiveCostmapCell> {
        self.validate_cell(x, y)?;
        Ok(self.cells[x as usize][y as usize])
    }

    pub fn set_cell_state(
        &mut self,
        x: i32,
        y: i32,
        state: AdaptiveCostmapCellState,
    ) -> RoboticsResult<()> {
        self.validate_cell(x, y)?;
        let cost = match state {
            AdaptiveCostmapCellState::Free => 0.0,
            AdaptiveCostmapCellState::Unknown => self.config.unknown_cost,
            AdaptiveCostmapCellState::StaticObstacle => self.config.static_obstacle_cost,
            AdaptiveCostmapCellState::MovableObstacle => self.config.movable_initial_cost,
        };
        self.cells[x as usize][y as usize] = AdaptiveCostmapCell { state, cost };
        Ok(())
    }

    pub fn mark_movable_obstacle(&mut self, x: i32, y: i32) -> RoboticsResult<()> {
        self.set_cell_state(x, y, AdaptiveCostmapCellState::MovableObstacle)
    }

    pub fn mark_static_obstacle(&mut self, x: i32, y: i32) -> RoboticsResult<()> {
        self.set_cell_state(x, y, AdaptiveCostmapCellState::StaticObstacle)
    }

    pub fn mark_unknown(&mut self, x: i32, y: i32) -> RoboticsResult<()> {
        self.set_cell_state(x, y, AdaptiveCostmapCellState::Unknown)
    }

    /// Update selected movable-obstacle cells from a progress observation.
    ///
    /// Stuck observations increase cost toward lethal. Good progress decreases
    /// cost back toward `movable_initial_cost`.
    pub fn update_movable_costs(
        &mut self,
        movable_cells: &[(i32, i32)],
        observation: MotionProgressObservation,
    ) -> RoboticsResult<usize> {
        validate_observation(&observation)?;
        let stuck = self.is_stuck(observation);
        let progressing = self.is_progressing(observation);
        let mut changed = 0;

        for &(x, y) in movable_cells {
            self.validate_cell(x, y)?;
            let cell = &mut self.cells[x as usize][y as usize];
            if cell.state != AdaptiveCostmapCellState::MovableObstacle {
                continue;
            }

            let old_cost = cell.cost;
            if stuck {
                cell.cost =
                    (cell.cost + self.config.movable_cost_increment).min(self.config.lethal_cost);
            } else if progressing {
                cell.cost = (cell.cost - self.config.movable_cost_decrement)
                    .max(self.config.movable_initial_cost);
            }
            if (cell.cost - old_cost).abs() > 1e-9 {
                changed += 1;
            }
        }

        Ok(changed)
    }

    /// Convert the adaptive costmap into traversal-risk cells.
    pub fn to_traversal_risk_cells(&self, block_lethal_movable: bool) -> Vec<Vec<TerrainRiskCell>> {
        let mut terrain = vec![
            vec![TerrainRiskCell::free(); self.config.height as usize];
            self.config.width as usize
        ];
        for x in 0..self.config.width as usize {
            for y in 0..self.config.height as usize {
                let cell = self.cells[x][y];
                terrain[x][y] = match cell.state {
                    AdaptiveCostmapCellState::Free => TerrainRiskCell::free(),
                    AdaptiveCostmapCellState::Unknown => TerrainRiskCell {
                        blocked: false,
                        traversability_risk: 0.0,
                        stability_risk: 0.0,
                        exposure_risk: normalize_cost(cell.cost, self.config.lethal_cost),
                    },
                    AdaptiveCostmapCellState::StaticObstacle => TerrainRiskCell::blocked(),
                    AdaptiveCostmapCellState::MovableObstacle => TerrainRiskCell {
                        blocked: block_lethal_movable && cell.cost >= self.config.lethal_cost,
                        traversability_risk: 0.0,
                        stability_risk: 0.0,
                        exposure_risk: normalize_cost(cell.cost, self.config.lethal_cost),
                    },
                };
            }
        }
        terrain
    }

    fn validate_cell(&self, x: i32, y: i32) -> RoboticsResult<()> {
        if x >= 0 && y >= 0 && x < self.config.width && y < self.config.height {
            Ok(())
        } else {
            Err(RoboticsError::InvalidParameter(format!(
                "cell ({}, {}) is out of bounds ({}x{})",
                x, y, self.config.width, self.config.height
            )))
        }
    }

    fn is_stuck(&self, observation: MotionProgressObservation) -> bool {
        observation.commanded_speed >= self.config.stuck_command_speed
            && observation.actual_speed
                <= observation.commanded_speed * self.config.stuck_actual_speed_ratio
            && observation.odom_delta < self.config.progress_distance
    }

    fn is_progressing(&self, observation: MotionProgressObservation) -> bool {
        observation.commanded_speed >= self.config.stuck_command_speed
            && (observation.actual_speed
                > observation.commanded_speed * self.config.stuck_actual_speed_ratio
                || observation.odom_delta >= self.config.progress_distance)
    }
}

fn validate_config(config: &AdaptiveCostmapNamoConfig) -> RoboticsResult<()> {
    if config.width <= 0 || config.height <= 0 {
        return Err(RoboticsError::InvalidParameter(
            "width and height must be positive".to_string(),
        ));
    }
    for (label, value, allow_zero) in [
        ("unknown_cost", config.unknown_cost, true),
        ("movable_initial_cost", config.movable_initial_cost, true),
        (
            "movable_cost_increment",
            config.movable_cost_increment,
            false,
        ),
        (
            "movable_cost_decrement",
            config.movable_cost_decrement,
            false,
        ),
        ("static_obstacle_cost", config.static_obstacle_cost, false),
        ("lethal_cost", config.lethal_cost, false),
        ("stuck_command_speed", config.stuck_command_speed, false),
        (
            "stuck_actual_speed_ratio",
            config.stuck_actual_speed_ratio,
            true,
        ),
        ("progress_distance", config.progress_distance, true),
    ] {
        if !value.is_finite() || value < 0.0 || (!allow_zero && value == 0.0) {
            return Err(RoboticsError::InvalidParameter(format!(
                "{} must be finite and {}",
                label,
                if allow_zero {
                    "non-negative"
                } else {
                    "positive"
                }
            )));
        }
    }
    if config.movable_initial_cost > config.lethal_cost
        || config.unknown_cost > config.lethal_cost
        || config.static_obstacle_cost > config.lethal_cost
    {
        return Err(RoboticsError::InvalidParameter(
            "initial/static/unknown costs must not exceed lethal_cost".to_string(),
        ));
    }
    Ok(())
}

fn validate_observation(observation: &MotionProgressObservation) -> RoboticsResult<()> {
    for (label, value) in [
        ("commanded_speed", observation.commanded_speed),
        ("actual_speed", observation.actual_speed),
        ("odom_delta", observation.odom_delta),
    ] {
        if value < 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{} must be finite and non-negative",
                label
            )));
        }
    }
    Ok(())
}

fn normalize_cost(cost: f64, lethal_cost: f64) -> f64 {
    if lethal_cost <= 0.0 {
        0.0
    } else {
        (cost / lethal_cost).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn costmap() -> AdaptiveCostmapNamo {
        AdaptiveCostmapNamo::new(AdaptiveCostmapNamoConfig::new(6, 4)).unwrap()
    }

    #[test]
    fn stuck_observation_raises_movable_cost() {
        let mut map = costmap();
        map.mark_movable_obstacle(2, 1).unwrap();

        let changed = map
            .update_movable_costs(&[(2, 1)], MotionProgressObservation::stuck(0.5))
            .unwrap();

        assert_eq!(changed, 1);
        assert_eq!(map.cell(2, 1).unwrap().cost, 50.0);
    }

    #[test]
    fn cost_caps_at_lethal() {
        let mut map = costmap();
        map.mark_movable_obstacle(2, 1).unwrap();
        for _ in 0..5 {
            map.update_movable_costs(&[(2, 1)], MotionProgressObservation::stuck(0.5))
                .unwrap();
        }

        assert_eq!(map.cell(2, 1).unwrap().cost, 100.0);
    }

    #[test]
    fn progress_observation_decays_to_initial_cost() {
        let mut map = costmap();
        map.mark_movable_obstacle(2, 1).unwrap();
        map.update_movable_costs(&[(2, 1)], MotionProgressObservation::stuck(0.5))
            .unwrap();
        map.update_movable_costs(&[(2, 1)], MotionProgressObservation::moving(0.5, 0.4, 0.2))
            .unwrap();
        map.update_movable_costs(&[(2, 1)], MotionProgressObservation::moving(0.5, 0.4, 0.2))
            .unwrap();
        map.update_movable_costs(&[(2, 1)], MotionProgressObservation::moving(0.5, 0.4, 0.2))
            .unwrap();

        assert_eq!(map.cell(2, 1).unwrap().cost, 20.0);
    }

    #[test]
    fn non_movable_cells_are_not_adapted() {
        let mut map = costmap();
        map.mark_unknown(2, 1).unwrap();
        let changed = map
            .update_movable_costs(&[(2, 1)], MotionProgressObservation::stuck(0.5))
            .unwrap();

        assert_eq!(changed, 0);
        assert_eq!(
            map.cell(2, 1).unwrap().state,
            AdaptiveCostmapCellState::Unknown
        );
    }

    #[test]
    fn traversal_risk_conversion_blocks_static_and_lethal_movable() {
        let mut map = costmap();
        map.mark_static_obstacle(1, 1).unwrap();
        map.mark_movable_obstacle(2, 1).unwrap();
        for _ in 0..3 {
            map.update_movable_costs(&[(2, 1)], MotionProgressObservation::stuck(0.5))
                .unwrap();
        }
        let terrain = map.to_traversal_risk_cells(true);

        assert!(terrain[1][1].blocked);
        assert!(terrain[2][1].blocked);
        assert_eq!(terrain[2][1].exposure_risk, 1.0);
    }

    #[test]
    fn rejects_invalid_config_and_coordinates() {
        let config = AdaptiveCostmapNamoConfig::new(0, 4);
        assert!(matches!(
            AdaptiveCostmapNamo::new(config),
            Err(RoboticsError::InvalidParameter(_))
        ));
        let mut map = costmap();
        assert!(matches!(
            map.mark_movable_obstacle(10, 1),
            Err(RoboticsError::InvalidParameter(_))
        ));
    }
}
