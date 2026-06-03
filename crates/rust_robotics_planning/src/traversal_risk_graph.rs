//! Traversal-risk graph planning over a 2-D grid.
//!
//! This is a compact pure-Rust planner inspired by traversal-risk graph
//! planners for unstructured terrain.  Each cell has independent risk channels
//! and the planner searches for the minimum distance-plus-risk path.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use rust_robotics_core::{RoboticsError, RoboticsResult};

/// Risk attributes for one terrain cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainRiskCell {
    pub blocked: bool,
    pub traversability_risk: f64,
    pub stability_risk: f64,
    pub exposure_risk: f64,
}

impl TerrainRiskCell {
    pub fn free() -> Self {
        Self {
            blocked: false,
            traversability_risk: 0.0,
            stability_risk: 0.0,
            exposure_risk: 0.0,
        }
    }

    pub fn blocked() -> Self {
        Self {
            blocked: true,
            ..Self::free()
        }
    }

    pub fn with_risk(traversability_risk: f64, stability_risk: f64, exposure_risk: f64) -> Self {
        Self {
            blocked: false,
            traversability_risk,
            stability_risk,
            exposure_risk,
        }
    }
}

/// Converts an elevation grid into traversal-risk cells.
///
/// Elevation storage is indexed as `elevation[x][y]`, matching
/// [`TraversalRiskGraphConfig::cells`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevationRiskConfig {
    pub cell_size: f64,
    pub slope_risk_scale: f64,
    pub roughness_risk_scale: f64,
    pub max_risk: f64,
    pub blocking_step_height: Option<f64>,
}

impl ElevationRiskConfig {
    pub fn new(cell_size: f64) -> Self {
        Self {
            cell_size,
            ..Self::default()
        }
    }
}

impl Default for ElevationRiskConfig {
    fn default() -> Self {
        Self {
            cell_size: 1.0,
            slope_risk_scale: 8.0,
            roughness_risk_scale: 10.0,
            max_risk: 10.0,
            blocking_step_height: None,
        }
    }
}

/// Gaussian-style local smoothing for terrain-risk channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiskMapSmoothingConfig {
    pub radius_cells: i32,
    pub iterations: usize,
    pub sigma_cells: f64,
    pub smooth_blocked_cells: bool,
}

impl RiskMapSmoothingConfig {
    pub fn new(radius_cells: i32) -> Self {
        Self {
            radius_cells,
            ..Self::default()
        }
    }
}

impl Default for RiskMapSmoothingConfig {
    fn default() -> Self {
        Self {
            radius_cells: 1,
            iterations: 1,
            sigma_cells: 1.0,
            smooth_blocked_cells: false,
        }
    }
}

/// Converts clearance from blocked cells into exposure risk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClearanceRiskConfig {
    pub cell_size: f64,
    pub minimum_clearance: f64,
    pub risk_scale: f64,
    pub max_risk: f64,
    pub additive: bool,
}

impl ClearanceRiskConfig {
    pub fn new(cell_size: f64, minimum_clearance: f64) -> Self {
        Self {
            cell_size,
            minimum_clearance,
            ..Self::default()
        }
    }
}

impl Default for ClearanceRiskConfig {
    fn default() -> Self {
        Self {
            cell_size: 1.0,
            minimum_clearance: 1.0,
            risk_scale: 8.0,
            max_risk: 8.0,
            additive: true,
        }
    }
}

/// Build traversal-risk cells from an elevation grid.
///
/// The traversability channel receives local slope risk. The stability channel
/// receives local roughness risk from the largest neighboring height step.
/// Cells whose local step exceeds `blocking_step_height`, when configured, are
/// marked blocked.
pub fn terrain_risk_from_elevation_map(
    elevation: &[Vec<f64>],
    config: &ElevationRiskConfig,
) -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    let (width, height) = validate_elevation_map(elevation)?;
    validate_elevation_config(config)?;

    let mut cells = vec![vec![TerrainRiskCell::free(); height]; width];
    for x in 0..width {
        for y in 0..height {
            let dzdx = elevation_gradient(elevation, x, y, 0, config.cell_size);
            let dzdy = elevation_gradient(elevation, x, y, 1, config.cell_size);
            let slope = (dzdx * dzdx + dzdy * dzdy).sqrt();
            let roughness = local_roughness(elevation, x, y);
            let traversability_risk = (slope * config.slope_risk_scale).min(config.max_risk);
            let stability_risk = (roughness * config.roughness_risk_scale).min(config.max_risk);
            let blocked = config
                .blocking_step_height
                .map_or(false, |threshold| roughness >= threshold);

            cells[x][y] = TerrainRiskCell {
                blocked,
                traversability_risk,
                stability_risk,
                exposure_risk: 0.0,
            };
        }
    }

    Ok(cells)
}

/// Smooth terrain-risk channels while preserving blocked-cell topology.
///
/// Blocked cells remain blocked. When `smooth_blocked_cells` is false, their
/// risk values are left untouched, but they still contribute to neighboring
/// risk smoothing.
pub fn smooth_terrain_risk(
    cells: &[Vec<TerrainRiskCell>],
    config: &RiskMapSmoothingConfig,
) -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    let (width, height) = validate_cell_grid(cells)?;
    validate_smoothing_config(config)?;
    if config.iterations == 0 || config.radius_cells == 0 {
        return Ok(cells.to_vec());
    }

    let mut current = cells.to_vec();
    let radius = config.radius_cells;
    let radius_sq = radius * radius;
    let sigma_sq = config.sigma_cells * config.sigma_cells;

    for _ in 0..config.iterations {
        let mut next = current.clone();
        for x in 0..width {
            for y in 0..height {
                if current[x][y].blocked && !config.smooth_blocked_cells {
                    continue;
                }

                let mut weight_sum = 0.0;
                let mut traversability_sum = 0.0;
                let mut stability_sum = 0.0;
                let mut exposure_sum = 0.0;

                for dx in -radius..=radius {
                    for dy in -radius..=radius {
                        let distance_sq = dx * dx + dy * dy;
                        if distance_sq > radius_sq {
                            continue;
                        }
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;
                        if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                            continue;
                        }

                        let neighbor = current[nx as usize][ny as usize];
                        let weight = (-(distance_sq as f64) / (2.0 * sigma_sq)).exp();
                        weight_sum += weight;
                        traversability_sum += weight * neighbor.traversability_risk;
                        stability_sum += weight * neighbor.stability_risk;
                        exposure_sum += weight * neighbor.exposure_risk;
                    }
                }

                next[x][y] = TerrainRiskCell {
                    blocked: current[x][y].blocked,
                    traversability_risk: traversability_sum / weight_sum,
                    stability_risk: stability_sum / weight_sum,
                    exposure_risk: exposure_sum / weight_sum,
                };
            }
        }
        current = next;
    }

    Ok(current)
}

/// Compute Euclidean clearance from every cell to the nearest blocked cell.
///
/// Distances are returned in metric units using `cell_size`. If the grid has no
/// blocked cells, every clearance value is `f64::INFINITY`.
pub fn clearance_map(
    cells: &[Vec<TerrainRiskCell>],
    cell_size: f64,
) -> RoboticsResult<Vec<Vec<f64>>> {
    let (width, height) = validate_cell_grid(cells)?;
    if cell_size <= 0.0 || !cell_size.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "cell_size must be finite and positive".to_string(),
        ));
    }

    let blocked = blocked_cell_positions(cells);
    let mut clearances = vec![vec![f64::INFINITY; height]; width];
    if blocked.is_empty() {
        return Ok(clearances);
    }

    for x in 0..width {
        for y in 0..height {
            let min_distance = blocked
                .iter()
                .map(|&(bx, by)| {
                    let dx = x as f64 - bx as f64;
                    let dy = y as f64 - by as f64;
                    (dx * dx + dy * dy).sqrt() * cell_size
                })
                .fold(f64::INFINITY, f64::min);
            clearances[x][y] = min_distance;
        }
    }

    Ok(clearances)
}

/// Add low-clearance exposure risk while preserving existing terrain channels.
///
/// A cell at or above `minimum_clearance` receives no additional exposure risk.
/// Closer cells receive linearly increasing risk up to `max_risk`.
pub fn add_clearance_exposure_risk(
    cells: &[Vec<TerrainRiskCell>],
    config: &ClearanceRiskConfig,
) -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    let (width, height) = validate_cell_grid(cells)?;
    validate_clearance_config(config)?;
    let clearances = clearance_map(cells, config.cell_size)?;
    let mut result = cells.to_vec();

    for x in 0..width {
        for y in 0..height {
            if result[x][y].blocked {
                continue;
            }
            let clearance = clearances[x][y];
            let clearance_risk =
                clearance_exposure_risk(clearance, config.minimum_clearance, config.risk_scale)
                    .min(config.max_risk);
            result[x][y].exposure_risk = if config.additive {
                (result[x][y].exposure_risk + clearance_risk).min(config.max_risk)
            } else {
                clearance_risk
            };
        }
    }

    Ok(result)
}

/// Inflate blocked cells by a Euclidean radius expressed in grid cells.
///
/// This is useful for applying a circular robot footprint before graph search.
pub fn inflate_blocked_cells(
    cells: &[Vec<TerrainRiskCell>],
    radius_cells: i32,
) -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    if radius_cells < 0 {
        return Err(RoboticsError::InvalidParameter(
            "radius_cells must be non-negative".to_string(),
        ));
    }
    let (width, height) = validate_cell_grid(cells)?;
    if radius_cells == 0 {
        return Ok(cells.to_vec());
    }

    let mut inflated = cells.to_vec();
    let radius_sq = radius_cells * radius_cells;
    for x in 0..width {
        for y in 0..height {
            if !cells[x][y].blocked {
                continue;
            }
            let x = x as i32;
            let y = y as i32;
            for dx in -radius_cells..=radius_cells {
                for dy in -radius_cells..=radius_cells {
                    if dx * dx + dy * dy > radius_sq {
                        continue;
                    }
                    let nx = x + dx;
                    let ny = y + dy;
                    if nx >= 0 && ny >= 0 && nx < width as i32 && ny < height as i32 {
                        inflated[nx as usize][ny as usize].blocked = true;
                    }
                }
            }
        }
    }

    Ok(inflated)
}

/// Inflate blocked cells by a metric footprint radius.
pub fn inflate_blocked_cells_by_radius(
    cells: &[Vec<TerrainRiskCell>],
    radius: f64,
    cell_size: f64,
) -> RoboticsResult<Vec<Vec<TerrainRiskCell>>> {
    if radius < 0.0 || !radius.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "radius must be finite and non-negative".to_string(),
        ));
    }
    if cell_size <= 0.0 || !cell_size.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "cell_size must be finite and positive".to_string(),
        ));
    }
    inflate_blocked_cells(cells, (radius / cell_size).ceil() as i32)
}

/// Planner configuration.
#[derive(Debug, Clone)]
pub struct TraversalRiskGraphConfig {
    pub width: i32,
    pub height: i32,
    /// Cell storage indexed as `cells[x][y]`.
    pub cells: Vec<Vec<TerrainRiskCell>>,
    pub allow_diagonal: bool,
    pub distance_weight: f64,
    pub risk_weight: f64,
    pub traversability_weight: f64,
    pub stability_weight: f64,
    pub exposure_weight: f64,
}

impl TraversalRiskGraphConfig {
    pub fn new(width: i32, height: i32, cells: Vec<Vec<TerrainRiskCell>>) -> Self {
        Self {
            width,
            height,
            cells,
            allow_diagonal: false,
            distance_weight: 1.0,
            risk_weight: 1.0,
            traversability_weight: 1.0,
            stability_weight: 1.0,
            exposure_weight: 1.0,
        }
    }
}

/// Plan the same query for several `risk_weight` values.
///
/// Each sample includes the weighted path returned by the planner plus
/// `terrain_risk_cost`, which is the unweighted integrated terrain risk along
/// the path. That makes distance-vs-risk tradeoffs directly comparable across
/// different weights.
pub fn sweep_traversal_risk_weights(
    config: TraversalRiskGraphConfig,
    risk_weights: &[f64],
    sx: i32,
    sy: i32,
    gx: i32,
    gy: i32,
) -> RoboticsResult<Vec<TraversalRiskWeightSample>> {
    let mut samples = Vec::with_capacity(risk_weights.len());
    for &risk_weight in risk_weights {
        let mut config = config.clone();
        config.risk_weight = risk_weight;
        let planner = TraversalRiskGraphPlanner::new(config)?;
        let path = planner.plan(sx, sy, gx, gy)?;
        let terrain_risk_cost = planner.path_terrain_risk(&path.waypoints)?;
        samples.push(TraversalRiskWeightSample {
            risk_weight,
            terrain_risk_cost,
            path,
        });
    }
    Ok(samples)
}

fn validate_cell_grid(cells: &[Vec<TerrainRiskCell>]) -> RoboticsResult<(usize, usize)> {
    if cells.is_empty() || cells[0].is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "cell grid must be non-empty".to_string(),
        ));
    }
    let height = cells[0].len();
    for column in cells {
        if column.len() != height {
            return Err(RoboticsError::InvalidParameter(
                "cell grid must be rectangular".to_string(),
            ));
        }
        for cell in column {
            for risk in [
                cell.traversability_risk,
                cell.stability_risk,
                cell.exposure_risk,
            ] {
                if risk < 0.0 || !risk.is_finite() {
                    return Err(RoboticsError::InvalidParameter(
                        "terrain risks must be finite and non-negative".to_string(),
                    ));
                }
            }
        }
    }
    Ok((cells.len(), height))
}

fn validate_elevation_map(elevation: &[Vec<f64>]) -> RoboticsResult<(usize, usize)> {
    if elevation.is_empty() || elevation[0].is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "elevation map must be non-empty".to_string(),
        ));
    }
    let height = elevation[0].len();
    for column in elevation {
        if column.len() != height {
            return Err(RoboticsError::InvalidParameter(
                "elevation map must be rectangular".to_string(),
            ));
        }
        if column.iter().any(|height| !height.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "elevation values must be finite".to_string(),
            ));
        }
    }
    Ok((elevation.len(), height))
}

fn validate_elevation_config(config: &ElevationRiskConfig) -> RoboticsResult<()> {
    for (label, value, allow_zero) in [
        ("cell_size", config.cell_size, false),
        ("slope_risk_scale", config.slope_risk_scale, true),
        ("roughness_risk_scale", config.roughness_risk_scale, true),
        ("max_risk", config.max_risk, false),
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
    if let Some(threshold) = config.blocking_step_height {
        if threshold <= 0.0 || !threshold.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "blocking_step_height must be finite and positive".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_smoothing_config(config: &RiskMapSmoothingConfig) -> RoboticsResult<()> {
    if config.radius_cells < 0 {
        return Err(RoboticsError::InvalidParameter(
            "radius_cells must be non-negative".to_string(),
        ));
    }
    if config.sigma_cells <= 0.0 || !config.sigma_cells.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "sigma_cells must be finite and positive".to_string(),
        ));
    }
    Ok(())
}

fn validate_clearance_config(config: &ClearanceRiskConfig) -> RoboticsResult<()> {
    for (label, value, allow_zero) in [
        ("cell_size", config.cell_size, false),
        ("minimum_clearance", config.minimum_clearance, false),
        ("risk_scale", config.risk_scale, true),
        ("max_risk", config.max_risk, true),
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
    Ok(())
}

fn blocked_cell_positions(cells: &[Vec<TerrainRiskCell>]) -> Vec<(usize, usize)> {
    let mut blocked = Vec::new();
    for (x, column) in cells.iter().enumerate() {
        for (y, cell) in column.iter().enumerate() {
            if cell.blocked {
                blocked.push((x, y));
            }
        }
    }
    blocked
}

fn clearance_exposure_risk(clearance: f64, minimum_clearance: f64, risk_scale: f64) -> f64 {
    if !clearance.is_finite() || clearance >= minimum_clearance {
        0.0
    } else {
        (1.0 - clearance / minimum_clearance) * risk_scale
    }
}

fn elevation_gradient(
    elevation: &[Vec<f64>],
    x: usize,
    y: usize,
    axis: usize,
    cell_size: f64,
) -> f64 {
    let (len, index) = if axis == 0 {
        (elevation.len(), x)
    } else {
        (elevation[0].len(), y)
    };
    if len == 1 {
        return 0.0;
    }

    let prev = index.saturating_sub(1);
    let next = (index + 1).min(len - 1);
    let prev_height = if axis == 0 {
        elevation[prev][y]
    } else {
        elevation[x][prev]
    };
    let next_height = if axis == 0 {
        elevation[next][y]
    } else {
        elevation[x][next]
    };
    let distance = (next - prev) as f64 * cell_size;
    if distance == 0.0 {
        0.0
    } else {
        (next_height - prev_height) / distance
    }
}

fn local_roughness(elevation: &[Vec<f64>], x: usize, y: usize) -> f64 {
    let center = elevation[x][y];
    let width = elevation.len() as i32;
    let height = elevation[0].len() as i32;
    let mut roughness: f64 = 0.0;

    for dx in -1..=1 {
        for dy in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let nx = x as i32 + dx;
            let ny = y as i32 + dy;
            if nx >= 0 && ny >= 0 && nx < width && ny < height {
                roughness = roughness.max((center - elevation[nx as usize][ny as usize]).abs());
            }
        }
    }

    roughness
}

/// One cell in a planned risk path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RiskWaypoint {
    pub x: i32,
    pub y: i32,
}

impl RiskWaypoint {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Planned path plus decomposed costs.
#[derive(Debug, Clone, PartialEq)]
pub struct TraversalRiskPath {
    pub waypoints: Vec<RiskWaypoint>,
    pub distance_cost: f64,
    pub risk_cost: f64,
    pub total_cost: f64,
}

/// One result from a risk-weight sweep.
#[derive(Debug, Clone, PartialEq)]
pub struct TraversalRiskWeightSample {
    pub risk_weight: f64,
    pub terrain_risk_cost: f64,
    pub path: TraversalRiskPath,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CellKey {
    x: i32,
    y: i32,
}

#[derive(Debug, Clone, Copy)]
struct OpenEntry {
    key: CellKey,
    f_cost: f64,
}

impl Eq for OpenEntry {}

impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost && self.key == other.key
    }
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_cost
            .total_cmp(&self.f_cost)
            .then_with(|| self.key.x.cmp(&other.key.x))
            .then_with(|| self.key.y.cmp(&other.key.y))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Traversal-risk graph planner.
pub struct TraversalRiskGraphPlanner {
    config: TraversalRiskGraphConfig,
    motions: Vec<(i32, i32, f64)>,
}

impl TraversalRiskGraphPlanner {
    pub fn new(config: TraversalRiskGraphConfig) -> RoboticsResult<Self> {
        Self::validate_config(&config)?;
        let motions = if config.allow_diagonal {
            vec![
                (1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, std::f64::consts::SQRT_2),
                (1, -1, std::f64::consts::SQRT_2),
                (-1, 1, std::f64::consts::SQRT_2),
                (-1, -1, std::f64::consts::SQRT_2),
            ]
        } else {
            vec![(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0)]
        };
        Ok(Self { config, motions })
    }

    /// Plan a minimum distance-plus-risk path on the configured grid.
    pub fn plan(&self, sx: i32, sy: i32, gx: i32, gy: i32) -> RoboticsResult<TraversalRiskPath> {
        self.validate_free_cell(sx, sy, "start")?;
        self.validate_free_cell(gx, gy, "goal")?;

        let start = CellKey { x: sx, y: sy };
        let goal = CellKey { x: gx, y: gy };
        let mut open = BinaryHeap::new();
        let mut g_cost: HashMap<CellKey, f64> = HashMap::new();
        let mut parent: HashMap<CellKey, CellKey> = HashMap::new();

        g_cost.insert(start, 0.0);
        open.push(OpenEntry {
            key: start,
            f_cost: self.heuristic(start, goal),
        });

        while let Some(current) = open.pop() {
            if current.key == goal {
                return self.reconstruct_path(start, goal, &parent);
            }

            let Some(&current_g) = g_cost.get(&current.key) else {
                continue;
            };
            if current.f_cost > current_g + self.heuristic(current.key, goal) + 1e-9 {
                continue;
            }

            for &(dx, dy, distance) in &self.motions {
                let next = CellKey {
                    x: current.key.x + dx,
                    y: current.key.y + dy,
                };
                if !self.is_free(next.x, next.y) {
                    continue;
                }

                let step_cost = self.edge_cost(current.key, next, distance);
                let next_g = current_g + step_cost;
                if g_cost.get(&next).map_or(false, |old| next_g >= *old) {
                    continue;
                }

                g_cost.insert(next, next_g);
                parent.insert(next, current.key);
                open.push(OpenEntry {
                    key: next,
                    f_cost: next_g + self.heuristic(next, goal),
                });
            }
        }

        Err(RoboticsError::PlanningError(
            "no traversal-risk path found".to_string(),
        ))
    }

    /// Weighted terrain risk for a single cell.
    pub fn cell_risk(&self, x: i32, y: i32) -> RoboticsResult<f64> {
        self.validate_cell(x, y, "cell")?;
        Ok(self.cell_risk_unchecked(CellKey { x, y }))
    }

    /// Integrated terrain risk along a waypoint path, before `risk_weight`.
    pub fn path_terrain_risk(&self, waypoints: &[RiskWaypoint]) -> RoboticsResult<f64> {
        if waypoints.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "waypoints must be non-empty".to_string(),
            ));
        }
        for waypoint in waypoints {
            self.validate_free_cell(waypoint.x, waypoint.y, "path waypoint")?;
        }

        Ok(waypoints
            .windows(2)
            .map(|window| {
                let from = CellKey {
                    x: window[0].x,
                    y: window[0].y,
                };
                let to = CellKey {
                    x: window[1].x,
                    y: window[1].y,
                };
                let dx = (to.x - from.x) as f64;
                let dy = (to.y - from.y) as f64;
                let distance = (dx * dx + dy * dy).sqrt();
                0.5 * (self.cell_risk_unchecked(from) + self.cell_risk_unchecked(to)) * distance
            })
            .sum())
    }

    fn validate_config(config: &TraversalRiskGraphConfig) -> RoboticsResult<()> {
        if config.width <= 0 || config.height <= 0 {
            return Err(RoboticsError::InvalidParameter(
                "width and height must be positive".to_string(),
            ));
        }
        if config.cells.len() != config.width as usize {
            return Err(RoboticsError::InvalidParameter(
                "cells x-dimension must match width".to_string(),
            ));
        }
        for column in &config.cells {
            if column.len() != config.height as usize {
                return Err(RoboticsError::InvalidParameter(
                    "cells y-dimension must match height".to_string(),
                ));
            }
            for cell in column {
                for risk in [
                    cell.traversability_risk,
                    cell.stability_risk,
                    cell.exposure_risk,
                ] {
                    if risk < 0.0 || !risk.is_finite() {
                        return Err(RoboticsError::InvalidParameter(
                            "terrain risks must be finite and non-negative".to_string(),
                        ));
                    }
                }
            }
        }
        for (label, value) in [
            ("distance_weight", config.distance_weight),
            ("risk_weight", config.risk_weight),
            ("traversability_weight", config.traversability_weight),
            ("stability_weight", config.stability_weight),
            ("exposure_weight", config.exposure_weight),
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

    fn validate_free_cell(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        self.validate_cell(x, y, label)?;
        if self.config.cells[x as usize][y as usize].blocked {
            return Err(RoboticsError::PlanningError(format!(
                "{} ({}, {}) is blocked",
                label, x, y
            )));
        }
        Ok(())
    }

    fn validate_cell(&self, x: i32, y: i32, label: &str) -> RoboticsResult<()> {
        if self.in_bounds(x, y) {
            Ok(())
        } else {
            Err(RoboticsError::InvalidParameter(format!(
                "{} ({}, {}) is out of bounds ({}x{})",
                label, x, y, self.config.width, self.config.height
            )))
        }
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.config.width && y < self.config.height
    }

    fn is_free(&self, x: i32, y: i32) -> bool {
        self.in_bounds(x, y) && !self.config.cells[x as usize][y as usize].blocked
    }

    fn cell_risk_unchecked(&self, key: CellKey) -> f64 {
        let cell = self.config.cells[key.x as usize][key.y as usize];
        self.config.traversability_weight * cell.traversability_risk
            + self.config.stability_weight * cell.stability_risk
            + self.config.exposure_weight * cell.exposure_risk
    }

    fn edge_cost(&self, from: CellKey, to: CellKey, distance: f64) -> f64 {
        let from_risk = self.cell_risk_unchecked(from);
        let to_risk = self.cell_risk_unchecked(to);
        let risk = 0.5 * (from_risk + to_risk) * distance;
        self.config.distance_weight * distance + self.config.risk_weight * risk
    }

    fn heuristic(&self, from: CellKey, goal: CellKey) -> f64 {
        let dx = (from.x - goal.x) as f64;
        let dy = (from.y - goal.y) as f64;
        self.config.distance_weight * (dx * dx + dy * dy).sqrt()
    }

    fn reconstruct_path(
        &self,
        start: CellKey,
        goal: CellKey,
        parent: &HashMap<CellKey, CellKey>,
    ) -> RoboticsResult<TraversalRiskPath> {
        let mut keys = vec![goal];
        let mut current = goal;
        while current != start {
            current = *parent.get(&current).ok_or_else(|| {
                RoboticsError::PlanningError("failed to reconstruct risk path".to_string())
            })?;
            keys.push(current);
        }
        keys.reverse();

        let waypoints: Vec<RiskWaypoint> = keys
            .iter()
            .map(|key| RiskWaypoint::new(key.x, key.y))
            .collect();
        let mut distance_cost = 0.0;
        let mut risk_cost = 0.0;
        for pair in keys.windows(2) {
            let dx = (pair[1].x - pair[0].x) as f64;
            let dy = (pair[1].y - pair[0].y) as f64;
            let distance = (dx * dx + dy * dy).sqrt();
            distance_cost += self.config.distance_weight * distance;
            let step_risk = 0.5
                * (self.cell_risk_unchecked(pair[0]) + self.cell_risk_unchecked(pair[1]))
                * distance;
            risk_cost += self.config.risk_weight * step_risk;
        }

        Ok(TraversalRiskPath {
            waypoints,
            distance_cost,
            risk_cost,
            total_cost: distance_cost + risk_cost,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_cells(width: i32, height: i32) -> Vec<Vec<TerrainRiskCell>> {
        vec![vec![TerrainRiskCell::free(); height as usize]; width as usize]
    }

    fn planner_with_risk(
        width: i32,
        height: i32,
        high_risk: &[(i32, i32)],
        risk_weight: f64,
    ) -> TraversalRiskGraphPlanner {
        let mut cells = free_cells(width, height);
        for &(x, y) in high_risk {
            cells[x as usize][y as usize] = TerrainRiskCell::with_risk(5.0, 0.0, 0.0);
        }
        TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
            risk_weight,
            ..TraversalRiskGraphConfig::new(width, height, cells)
        })
        .unwrap()
    }

    #[test]
    fn straight_line_when_risk_is_zero() {
        let planner = planner_with_risk(5, 3, &[], 1.0);
        let path = planner.plan(0, 1, 4, 1).unwrap();

        assert_eq!(path.waypoints.len(), 5);
        assert!(path.waypoints.iter().all(|waypoint| waypoint.y == 1));
        assert!((path.distance_cost - 4.0).abs() < 1e-9);
        assert_eq!(path.risk_cost, 0.0);
    }

    #[test]
    fn risk_weight_changes_route_choice() {
        let risky_corridor = vec![(2, 1), (3, 1), (4, 1)];
        let distance_only = planner_with_risk(7, 3, &risky_corridor, 0.0);
        let risk_aware = planner_with_risk(7, 3, &risky_corridor, 2.0);

        let short_path = distance_only.plan(0, 1, 6, 1).unwrap();
        let safe_path = risk_aware.plan(0, 1, 6, 1).unwrap();

        assert!(short_path.waypoints.iter().any(|waypoint| waypoint.y == 1));
        assert!(
            safe_path
                .waypoints
                .iter()
                .filter(|waypoint| risky_corridor.contains(&(waypoint.x, waypoint.y)))
                .count()
                < short_path
                    .waypoints
                    .iter()
                    .filter(|waypoint| risky_corridor.contains(&(waypoint.x, waypoint.y)))
                    .count()
        );
        assert!(safe_path.distance_cost > short_path.distance_cost);
        assert!(safe_path.total_cost < risk_aware.edge_costs_along(&short_path.waypoints));
    }

    #[test]
    fn blocked_cells_force_detour() {
        let mut cells = free_cells(5, 3);
        cells[2][1] = TerrainRiskCell::blocked();
        let planner =
            TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig::new(5, 3, cells)).unwrap();
        let path = planner.plan(0, 1, 4, 1).unwrap();

        assert!(!path
            .waypoints
            .iter()
            .any(|waypoint| waypoint.x == 2 && waypoint.y == 1));
        assert!(path.distance_cost > 4.0);
    }

    #[test]
    fn rejects_invalid_risk_values() {
        let mut cells = free_cells(2, 2);
        cells[1][1].traversability_risk = f64::NAN;
        let result = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig::new(2, 2, cells));

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn elevation_map_generates_slope_and_roughness_risks() {
        let elevation = vec![
            vec![0.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0],
        ];
        let cells = terrain_risk_from_elevation_map(
            &elevation,
            &ElevationRiskConfig {
                slope_risk_scale: 4.0,
                roughness_risk_scale: 3.0,
                max_risk: 20.0,
                ..ElevationRiskConfig::new(1.0)
            },
        )
        .unwrap();

        assert!(cells[0][1].traversability_risk > 0.0);
        assert_eq!(cells[1][1].traversability_risk, 0.0);
        assert!((cells[1][1].stability_risk - 3.0).abs() < 1e-9);
        assert!(!cells[1][1].blocked);
    }

    #[test]
    fn elevation_step_can_mark_cells_blocked() {
        let elevation = vec![
            vec![0.0, 0.0, 0.0],
            vec![0.0, 1.2, 0.0],
            vec![0.0, 0.0, 0.0],
        ];
        let cells = terrain_risk_from_elevation_map(
            &elevation,
            &ElevationRiskConfig {
                blocking_step_height: Some(1.0),
                ..ElevationRiskConfig::new(1.0)
            },
        )
        .unwrap();

        assert!(cells[1][1].blocked);
        assert!(cells[0][1].blocked);
    }

    #[test]
    fn footprint_inflation_expands_blocked_cells() {
        let mut cells = free_cells(5, 5);
        cells[2][2] = TerrainRiskCell::blocked();
        let inflated = inflate_blocked_cells(&cells, 1).unwrap();

        assert!(inflated[2][2].blocked);
        assert!(inflated[1][2].blocked);
        assert!(inflated[2][1].blocked);
        assert!(inflated[3][2].blocked);
        assert!(inflated[2][3].blocked);
        assert!(!inflated[1][1].blocked);
    }

    #[test]
    fn metric_footprint_radius_is_converted_to_grid_radius() {
        let mut cells = free_cells(5, 5);
        cells[2][2] = TerrainRiskCell::blocked();
        let inflated = inflate_blocked_cells_by_radius(&cells, 0.75, 0.5).unwrap();

        assert!(inflated[0][2].blocked);
        assert!(inflated[4][2].blocked);
        assert!(!inflated[0][0].blocked);
    }

    #[test]
    fn risk_smoothing_spreads_impulse_risk() {
        let mut cells = free_cells(5, 5);
        cells[2][2] = TerrainRiskCell::with_risk(9.0, 0.0, 0.0);
        let smoothed = smooth_terrain_risk(
            &cells,
            &RiskMapSmoothingConfig {
                radius_cells: 1,
                iterations: 1,
                sigma_cells: 1.0,
                smooth_blocked_cells: false,
            },
        )
        .unwrap();

        assert!(smoothed[2][2].traversability_risk < 9.0);
        assert!(smoothed[2][1].traversability_risk > 0.0);
        assert_eq!(smoothed[0][0].traversability_risk, 0.0);
    }

    #[test]
    fn risk_smoothing_preserves_blocked_cells_by_default() {
        let mut cells = free_cells(3, 3);
        cells[1][1] = TerrainRiskCell {
            blocked: true,
            traversability_risk: 6.0,
            stability_risk: 2.0,
            exposure_risk: 1.0,
        };
        let smoothed = smooth_terrain_risk(&cells, &RiskMapSmoothingConfig::new(1)).unwrap();

        assert_eq!(smoothed[1][1], cells[1][1]);
        assert!(smoothed[1][0].traversability_risk > 0.0);
    }

    #[test]
    fn zero_iteration_smoothing_returns_original_grid() {
        let mut cells = free_cells(3, 3);
        cells[1][1] = TerrainRiskCell::with_risk(4.0, 3.0, 2.0);
        let smoothed = smooth_terrain_risk(
            &cells,
            &RiskMapSmoothingConfig {
                iterations: 0,
                ..RiskMapSmoothingConfig::new(1)
            },
        )
        .unwrap();

        assert_eq!(smoothed, cells);
    }

    #[test]
    fn rejects_invalid_smoothing_config() {
        let cells = free_cells(3, 3);
        let result = smooth_terrain_risk(
            &cells,
            &RiskMapSmoothingConfig {
                sigma_cells: 0.0,
                ..RiskMapSmoothingConfig::new(1)
            },
        );

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn clearance_map_reports_metric_distance_to_blocked_cells() {
        let mut cells = free_cells(3, 3);
        cells[1][1] = TerrainRiskCell::blocked();
        let clearances = clearance_map(&cells, 0.5).unwrap();

        assert_eq!(clearances[1][1], 0.0);
        assert!((clearances[1][0] - 0.5).abs() < 1e-9);
        assert!((clearances[0][0] - std::f64::consts::SQRT_2 * 0.5).abs() < 1e-9);
    }

    #[test]
    fn clearance_map_without_blocked_cells_is_infinite() {
        let cells = free_cells(3, 3);
        let clearances = clearance_map(&cells, 1.0).unwrap();

        assert!(clearances
            .iter()
            .flatten()
            .all(|clearance| clearance.is_infinite()));
    }

    #[test]
    fn clearance_exposure_risk_is_added_to_existing_exposure() {
        let mut cells = free_cells(3, 3);
        cells[1][1] = TerrainRiskCell::blocked();
        cells[1][0].exposure_risk = 1.0;
        let risked = add_clearance_exposure_risk(
            &cells,
            &ClearanceRiskConfig {
                cell_size: 1.0,
                minimum_clearance: 2.0,
                risk_scale: 4.0,
                max_risk: 8.0,
                additive: true,
            },
        )
        .unwrap();

        assert!(risked[1][1].blocked);
        assert!((risked[1][0].exposure_risk - 3.0).abs() < 1e-9);
        assert!(risked[0][0].exposure_risk > 0.0);
        assert!(risked[0][0].exposure_risk < risked[1][0].exposure_risk);
    }

    #[test]
    fn clearance_risk_moves_path_away_from_narrow_corridor() {
        let mut cells = free_cells(9, 7);
        for x in 2..=6 {
            cells[x][2] = TerrainRiskCell::blocked();
            cells[x][4] = TerrainRiskCell::blocked();
        }
        let risked = add_clearance_exposure_risk(
            &cells,
            &ClearanceRiskConfig {
                minimum_clearance: 2.0,
                risk_scale: 8.0,
                max_risk: 8.0,
                ..ClearanceRiskConfig::new(1.0, 2.0)
            },
        )
        .unwrap();
        let distance_only = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
            risk_weight: 0.0,
            exposure_weight: 1.0,
            ..TraversalRiskGraphConfig::new(9, 7, risked.clone())
        })
        .unwrap();
        let clearance_aware = TraversalRiskGraphPlanner::new(TraversalRiskGraphConfig {
            risk_weight: 1.0,
            exposure_weight: 1.0,
            ..TraversalRiskGraphConfig::new(9, 7, risked)
        })
        .unwrap();

        let short_path = distance_only.plan(0, 3, 8, 3).unwrap();
        let wide_path = clearance_aware.plan(0, 3, 8, 3).unwrap();

        assert!(short_path.waypoints.iter().any(|waypoint| waypoint.y == 3));
        assert!(wide_path.distance_cost > short_path.distance_cost);
        assert!(wide_path
            .waypoints
            .iter()
            .any(|waypoint| waypoint.y == 0 || waypoint.y == 6));
        assert!(
            clearance_aware
                .path_terrain_risk(&wide_path.waypoints)
                .unwrap()
                < clearance_aware
                    .path_terrain_risk(&short_path.waypoints)
                    .unwrap()
        );
    }

    #[test]
    fn rejects_invalid_clearance_config() {
        let cells = free_cells(3, 3);
        let result = add_clearance_exposure_risk(
            &cells,
            &ClearanceRiskConfig {
                minimum_clearance: 0.0,
                ..ClearanceRiskConfig::default()
            },
        );

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn path_terrain_risk_is_unweighted() {
        let planner = planner_with_risk(5, 3, &[(2, 1), (3, 1)], 3.0);
        let path = vec![
            RiskWaypoint::new(0, 1),
            RiskWaypoint::new(1, 1),
            RiskWaypoint::new(2, 1),
            RiskWaypoint::new(3, 1),
            RiskWaypoint::new(4, 1),
        ];
        let terrain_risk = planner.path_terrain_risk(&path).unwrap();

        assert!((terrain_risk - 10.0).abs() < 1e-9);
    }

    #[test]
    fn sweep_risk_weights_captures_distance_risk_tradeoff() {
        let mut cells = free_cells(7, 3);
        for x in 2..=4 {
            cells[x][1] = TerrainRiskCell::with_risk(5.0, 0.0, 0.0);
        }
        let config = TraversalRiskGraphConfig::new(7, 3, cells);
        let samples = sweep_traversal_risk_weights(config, &[0.0, 2.0], 0, 1, 6, 1).unwrap();

        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0].risk_weight, 0.0);
        assert_eq!(samples[1].risk_weight, 2.0);
        assert!(samples[1].path.distance_cost > samples[0].path.distance_cost);
        assert!(samples[1].terrain_risk_cost < samples[0].terrain_risk_cost);
    }

    #[test]
    fn path_terrain_risk_rejects_empty_paths() {
        let planner = planner_with_risk(3, 3, &[], 1.0);
        let result = planner.path_terrain_risk(&[]);

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn rejects_invalid_elevation_maps() {
        let ragged = vec![vec![0.0, 1.0], vec![0.0]];
        let result = terrain_risk_from_elevation_map(&ragged, &ElevationRiskConfig::default());

        assert!(matches!(result, Err(RoboticsError::InvalidParameter(_))));
    }

    impl TraversalRiskGraphPlanner {
        fn edge_costs_along(&self, waypoints: &[RiskWaypoint]) -> f64 {
            waypoints
                .windows(2)
                .map(|window| {
                    let from = CellKey {
                        x: window[0].x,
                        y: window[0].y,
                    };
                    let to = CellKey {
                        x: window[1].x,
                        y: window[1].y,
                    };
                    let dx = (to.x - from.x) as f64;
                    let dy = (to.y - from.y) as f64;
                    self.edge_cost(from, to, (dx * dx + dy * dy).sqrt())
                })
                .sum()
        }
    }
}
