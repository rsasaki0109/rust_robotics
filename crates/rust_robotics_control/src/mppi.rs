//! Vanilla MPPI controller for a 2-D double integrator.
//!
//! This is the dependency-light baseline needed before TD-CD-MPPI-style
//! terminal value and constraint-discounted extensions.

use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};
use std::collections::VecDeque;

/// State for a planar double-integrator model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiState2D {
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
}

impl MppiState2D {
    pub fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Self { x, y, vx, vy }
    }

    pub fn step(self, control: MppiControl2D, dt: f64) -> Self {
        Self {
            x: self.x + self.vx * dt + 0.5 * control.ax * dt * dt,
            y: self.y + self.vy * dt + 0.5 * control.ay * dt * dt,
            vx: self.vx + control.ax * dt,
            vy: self.vy + control.ay * dt,
        }
    }
}

/// Acceleration command for the double-integrator model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiControl2D {
    pub ax: f64,
    pub ay: f64,
}

impl MppiControl2D {
    pub fn new(ax: f64, ay: f64) -> Self {
        Self { ax, ay }
    }
}

/// Circular obstacle used by constraint-discounted MPPI rollouts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiCircularObstacle2D {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl MppiCircularObstacle2D {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Linearly predicted circular obstacle for prediction-aware MPPI rollouts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiMovingObstacle2D {
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
}

impl MppiMovingObstacle2D {
    pub fn new(x: f64, y: f64, vx: f64, vy: f64, radius: f64) -> Self {
        Self {
            x,
            y,
            vx,
            vy,
            radius,
        }
    }

    pub fn predict(self, time: f64) -> MppiCircularObstacle2D {
        MppiCircularObstacle2D::new(
            self.x + self.vx * time,
            self.y + self.vy * time,
            self.radius,
        )
    }
}

/// 2-D racing gate for reference-free MPPI progress objectives.
///
/// The normal points in the race direction. A gate is crossed when a rollout
/// segment moves from the negative to the positive side of the gate plane and
/// intersects the gate within `half_width`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiRacingGate2D {
    pub center_x: f64,
    pub center_y: f64,
    pub normal_x: f64,
    pub normal_y: f64,
    pub half_width: f64,
}

impl MppiRacingGate2D {
    pub fn new(
        center_x: f64,
        center_y: f64,
        normal_x: f64,
        normal_y: f64,
        half_width: f64,
    ) -> RoboticsResult<Self> {
        let norm = (normal_x * normal_x + normal_y * normal_y).sqrt();
        if !center_x.is_finite()
            || !center_y.is_finite()
            || !normal_x.is_finite()
            || !normal_y.is_finite()
            || norm <= 0.0
        {
            return Err(RoboticsError::InvalidParameter(
                "racing gate center and normal must be finite".to_string(),
            ));
        }
        if half_width <= 0.0 || !half_width.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "racing gate half_width must be finite and positive".to_string(),
            ));
        }
        Ok(Self {
            center_x,
            center_y,
            normal_x: normal_x / norm,
            normal_y: normal_y / norm,
            half_width,
        })
    }

    pub fn center(&self) -> (f64, f64) {
        (self.center_x, self.center_y)
    }

    pub fn signed_distance(&self, x: f64, y: f64) -> f64 {
        (x - self.center_x) * self.normal_x + (y - self.center_y) * self.normal_y
    }

    pub fn lateral_distance(&self, x: f64, y: f64) -> f64 {
        let tangent_x = -self.normal_y;
        let tangent_y = self.normal_x;
        ((x - self.center_x) * tangent_x + (y - self.center_y) * tangent_y).abs()
    }

    pub fn squared_distance(&self, x: f64, y: f64) -> f64 {
        (x - self.center_x).powi(2) + (y - self.center_y).powi(2)
    }

    pub fn crosses(&self, from: MppiState2D, to: MppiState2D, tolerance: f64) -> bool {
        let from_signed = self.signed_distance(from.x, from.y);
        let to_signed = self.signed_distance(to.x, to.y);
        if from_signed > tolerance || to_signed < -tolerance {
            return false;
        }
        let denom = to_signed - from_signed;
        let t = if denom.abs() > 1e-9 {
            (-from_signed / denom).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let cross_x = from.x + t * (to.x - from.x);
        let cross_y = from.y + t * (to.y - from.y);
        self.lateral_distance(cross_x, cross_y) <= self.half_width + tolerance
    }

    pub fn state_is_past_gate(&self, state: MppiState2D, tolerance: f64) -> bool {
        self.signed_distance(state.x, state.y) >= -tolerance
            && self.lateral_distance(state.x, state.y) <= self.half_width + tolerance
    }
}

/// Reference-free gate-progress objective for race-style MPPI.
///
/// The per-step progress term follows the racing-MPPI idea of minimizing the
/// change in squared distance to the active gate. Moving toward the gate creates
/// negative cost, moving away creates positive cost, and a valid gate crossing
/// switches the rollout to the next gate.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiGateRace2D {
    pub gates: Vec<MppiRacingGate2D>,
    pub progress_weight: f64,
    pub lateral_weight: f64,
    pub pass_bonus: f64,
    pub miss_penalty: f64,
    pub terminal_gate_weight: f64,
    pub crossing_tolerance: f64,
    active_gate_index: usize,
}

/// Summary for scoring a trajectory against a gate race.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiGateRaceReport2D {
    pub passed_gates: usize,
    pub active_gate_index: usize,
    pub cost: f64,
    pub final_gate_distance: f64,
    pub final_lateral_error: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct MppiGateRaceStep2D {
    cost: f64,
    active_gate_index: usize,
    passed_gate: bool,
}

impl MppiGateRace2D {
    pub fn new(gates: Vec<MppiRacingGate2D>) -> RoboticsResult<Self> {
        let race = Self {
            gates,
            progress_weight: 1.0,
            lateral_weight: 0.08,
            pass_bonus: 4.0,
            miss_penalty: 18.0,
            terminal_gate_weight: 0.6,
            crossing_tolerance: 0.03,
            active_gate_index: 0,
        };
        validate_gate_race(&race)?;
        Ok(race)
    }

    pub fn gate_count(&self) -> usize {
        self.gates.len()
    }

    pub fn active_gate_index(&self) -> usize {
        self.active_gate_index
    }

    pub fn reset(&mut self) {
        self.active_gate_index = 0;
    }

    pub fn set_active_gate_index(&mut self, active_gate_index: usize) -> RoboticsResult<()> {
        if active_gate_index > self.gates.len() {
            return Err(RoboticsError::InvalidParameter(
                "active gate index must be within the gate list".to_string(),
            ));
        }
        self.active_gate_index = active_gate_index;
        Ok(())
    }

    pub fn advance_active_gate_from_state(&mut self, state: MppiState2D) -> usize {
        while let Some(gate) = self.gates.get(self.active_gate_index) {
            if !gate.state_is_past_gate(state, self.crossing_tolerance) {
                break;
            }
            self.active_gate_index += 1;
        }
        self.active_gate_index
    }

    pub fn score_trajectory(&self, states: &[MppiState2D]) -> RoboticsResult<MppiGateRaceReport2D> {
        if states.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "gate race trajectory must contain at least one state".to_string(),
            ));
        }
        for &state in states {
            validate_state(state)?;
        }
        let mut active_gate_index = 0;
        let mut cost = 0.0;
        let mut passed_gates = 0;
        for pair in states.windows(2) {
            let step = self.transition_cost(pair[0], pair[1], active_gate_index);
            cost += step.cost;
            active_gate_index = step.active_gate_index;
            if step.passed_gate {
                passed_gates += 1;
            }
        }
        let final_state = *states
            .last()
            .expect("validated gate race trajectory is non-empty");
        cost += self.terminal_cost(final_state, active_gate_index);
        let (final_gate_distance, final_lateral_error) =
            self.final_gate_errors(final_state, active_gate_index);
        Ok(MppiGateRaceReport2D {
            passed_gates,
            active_gate_index,
            cost,
            final_gate_distance,
            final_lateral_error,
        })
    }

    fn transition_cost(
        &self,
        from: MppiState2D,
        to: MppiState2D,
        active_gate_index: usize,
    ) -> MppiGateRaceStep2D {
        let Some(gate) = self.gates.get(active_gate_index) else {
            return MppiGateRaceStep2D {
                cost: 0.0,
                active_gate_index,
                passed_gate: false,
            };
        };

        let before_distance_sq = gate.squared_distance(from.x, from.y);
        let after_distance_sq = gate.squared_distance(to.x, to.y);
        let after_lateral = gate.lateral_distance(to.x, to.y);
        let after_distance = after_distance_sq.sqrt();
        let lateral_proximity = 1.0 / (1.0 + after_distance);
        let mut cost = self.progress_weight * (after_distance_sq - before_distance_sq)
            + self.lateral_weight * lateral_proximity * after_lateral * after_lateral;
        let mut next_active = active_gate_index;
        let mut passed_gate = false;

        if gate.crosses(from, to, self.crossing_tolerance) {
            cost -= self.pass_bonus;
            next_active += 1;
            passed_gate = true;
        } else if gate.signed_distance(to.x, to.y) >= 0.0 && after_lateral > gate.half_width {
            let miss = after_lateral - gate.half_width;
            cost += self.miss_penalty * miss * miss;
        }

        MppiGateRaceStep2D {
            cost,
            active_gate_index: next_active,
            passed_gate,
        }
    }

    fn terminal_cost(&self, state: MppiState2D, active_gate_index: usize) -> f64 {
        let Some(gate) = self.gates.get(active_gate_index) else {
            return 0.0;
        };
        self.terminal_gate_weight * gate.squared_distance(state.x, state.y)
            + self.pass_bonus * (self.gates.len() - active_gate_index) as f64
    }

    fn final_gate_errors(&self, state: MppiState2D, active_gate_index: usize) -> (f64, f64) {
        let Some(gate) = self.gates.get(active_gate_index) else {
            return (0.0, 0.0);
        };
        (
            gate.squared_distance(state.x, state.y).sqrt(),
            gate.lateral_distance(state.x, state.y),
        )
    }
}

/// Grid-based terminal value estimate for MPPI rollouts.
///
/// Values are indexed as `values[x][y]` and queried with bilinear
/// interpolation. Out-of-bounds queries clamp to the nearest grid edge.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiTerminalValueGrid2D {
    pub origin_x: f64,
    pub origin_y: f64,
    pub resolution: f64,
    pub values: Vec<Vec<f64>>,
}

impl MppiTerminalValueGrid2D {
    pub fn new(
        origin_x: f64,
        origin_y: f64,
        resolution: f64,
        values: Vec<Vec<f64>>,
    ) -> RoboticsResult<Self> {
        let grid = Self {
            origin_x,
            origin_y,
            resolution,
            values,
        };
        validate_terminal_value_grid(&grid)?;
        Ok(grid)
    }

    pub fn from_goal_distance(
        width: usize,
        height: usize,
        origin_x: f64,
        origin_y: f64,
        resolution: f64,
        goal: (f64, f64),
    ) -> RoboticsResult<Self> {
        if width == 0 || height == 0 {
            return Err(RoboticsError::InvalidParameter(
                "terminal value grid dimensions must be non-empty".to_string(),
            ));
        }
        let mut values = vec![vec![0.0; height]; width];
        for (x, column) in values.iter_mut().enumerate() {
            for (y, value) in column.iter_mut().enumerate() {
                let wx = origin_x + x as f64 * resolution;
                let wy = origin_y + y as f64 * resolution;
                let dx = wx - goal.0;
                let dy = wy - goal.1;
                *value = (dx * dx + dy * dy).sqrt();
            }
        }
        Self::new(origin_x, origin_y, resolution, values)
    }

    pub fn value_at_state(&self, state: MppiState2D) -> f64 {
        self.value_at(state.x, state.y)
    }

    pub fn value_at(&self, x: f64, y: f64) -> f64 {
        let width = self.values.len();
        let height = self.values[0].len();
        let gx = ((x - self.origin_x) / self.resolution).clamp(0.0, (width - 1) as f64);
        let gy = ((y - self.origin_y) / self.resolution).clamp(0.0, (height - 1) as f64);
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let x1 = (x0 + 1).min(width - 1);
        let y1 = (y0 + 1).min(height - 1);
        let tx = gx - x0 as f64;
        let ty = gy - y0 as f64;

        let v00 = self.values[x0][y0];
        let v10 = self.values[x1][y0];
        let v01 = self.values[x0][y1];
        let v11 = self.values[x1][y1];
        let vx0 = v00 * (1.0 - tx) + v10 * tx;
        let vx1 = v01 * (1.0 - tx) + v11 * tx;
        vx0 * (1.0 - ty) + vx1 * ty
    }

    pub fn width(&self) -> usize {
        self.values.len()
    }

    pub fn height(&self) -> usize {
        self.values[0].len()
    }

    pub fn nearest_cell_indices(&self, x: f64, y: f64) -> (usize, usize) {
        let gx = ((x - self.origin_x) / self.resolution)
            .round()
            .clamp(0.0, (self.width() - 1) as f64);
        let gy = ((y - self.origin_y) / self.resolution)
            .round()
            .clamp(0.0, (self.height() - 1) as f64);
        (gx as usize, gy as usize)
    }

    pub fn value_at_cell(&self, x: usize, y: usize) -> RoboticsResult<f64> {
        if x >= self.width() || y >= self.height() {
            return Err(RoboticsError::InvalidParameter(
                "terminal value grid cell is out of bounds".to_string(),
            ));
        }
        Ok(self.values[x][y])
    }

    pub fn update_cell_toward(
        &mut self,
        x: usize,
        y: usize,
        target: f64,
        learning_rate: f64,
    ) -> RoboticsResult<f64> {
        if x >= self.width() || y >= self.height() {
            return Err(RoboticsError::InvalidParameter(
                "terminal value grid cell is out of bounds".to_string(),
            ));
        }
        if target < 0.0 || !target.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "terminal value target must be finite and non-negative".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&learning_rate) || learning_rate == 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "terminal value learning_rate must be in (0, 1]".to_string(),
            ));
        }
        let old = self.values[x][y];
        let new_value = old + learning_rate * (target - old);
        self.values[x][y] = new_value.max(0.0);
        Ok((self.values[x][y] - old).abs())
    }
}

/// Projection of a point onto a waypoint track.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiTrackProjection2D {
    pub segment_index: usize,
    pub progress: f64,
    pub lateral_error: f64,
    pub closest_x: f64,
    pub closest_y: f64,
}

/// Polyline track helper for MPPI progress-based terminal values.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiWaypointTrack2D {
    pub waypoints: Vec<(f64, f64)>,
    cumulative_lengths: Vec<f64>,
}

impl MppiWaypointTrack2D {
    pub fn new(waypoints: Vec<(f64, f64)>) -> RoboticsResult<Self> {
        validate_track_waypoints(&waypoints)?;
        let mut cumulative_lengths = vec![0.0; waypoints.len()];
        for index in 1..waypoints.len() {
            cumulative_lengths[index] = cumulative_lengths[index - 1]
                + point_distance(waypoints[index - 1], waypoints[index]);
        }
        Ok(Self {
            waypoints,
            cumulative_lengths,
        })
    }

    pub fn total_length(&self) -> f64 {
        *self
            .cumulative_lengths
            .last()
            .expect("validated track has at least two waypoints")
    }

    pub fn goal(&self) -> (f64, f64) {
        *self
            .waypoints
            .last()
            .expect("validated track has at least two waypoints")
    }

    pub fn point_at_progress(&self, progress: f64) -> RoboticsResult<(f64, f64)> {
        if !progress.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "track progress query must be finite".to_string(),
            ));
        }
        let clamped = progress.clamp(0.0, self.total_length());
        for index in 0..self.waypoints.len() - 1 {
            let segment_start = self.cumulative_lengths[index];
            let segment_end = self.cumulative_lengths[index + 1];
            if clamped <= segment_end || index + 2 == self.waypoints.len() {
                let a = self.waypoints[index];
                let b = self.waypoints[index + 1];
                let segment_length = segment_end - segment_start;
                let t = if segment_length > 0.0 {
                    (clamped - segment_start) / segment_length
                } else {
                    0.0
                };
                return Ok((a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)));
            }
        }
        Ok(self.goal())
    }

    pub fn project(&self, x: f64, y: f64) -> RoboticsResult<MppiTrackProjection2D> {
        if !x.is_finite() || !y.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "track projection point must be finite".to_string(),
            ));
        }

        let mut best = MppiTrackProjection2D {
            segment_index: 0,
            progress: 0.0,
            lateral_error: f64::INFINITY,
            closest_x: self.waypoints[0].0,
            closest_y: self.waypoints[0].1,
        };
        for index in 0..self.waypoints.len() - 1 {
            let a = self.waypoints[index];
            let b = self.waypoints[index + 1];
            let dx = b.0 - a.0;
            let dy = b.1 - a.1;
            let segment_length_sq = dx * dx + dy * dy;
            let t = (((x - a.0) * dx + (y - a.1) * dy) / segment_length_sq).clamp(0.0, 1.0);
            let closest_x = a.0 + t * dx;
            let closest_y = a.1 + t * dy;
            let lateral_error = ((x - closest_x).powi(2) + (y - closest_y).powi(2)).sqrt();
            if lateral_error < best.lateral_error {
                best = MppiTrackProjection2D {
                    segment_index: index,
                    progress: self.cumulative_lengths[index] + t * segment_length_sq.sqrt(),
                    lateral_error,
                    closest_x,
                    closest_y,
                };
            }
        }
        Ok(best)
    }

    pub fn remaining_distance(&self, x: f64, y: f64) -> RoboticsResult<f64> {
        Ok((self.total_length() - self.project(x, y)?.progress).max(0.0))
    }

    pub fn terminal_value_grid(
        &self,
        width: usize,
        height: usize,
        origin_x: f64,
        origin_y: f64,
        resolution: f64,
        progress_weight: f64,
        lateral_weight: f64,
    ) -> RoboticsResult<MppiTerminalValueGrid2D> {
        if width == 0 || height == 0 {
            return Err(RoboticsError::InvalidParameter(
                "track terminal value grid dimensions must be non-empty".to_string(),
            ));
        }
        if progress_weight < 0.0 || lateral_weight < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "track terminal value weights must be non-negative".to_string(),
            ));
        }
        if !progress_weight.is_finite() || !lateral_weight.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "track terminal value weights must be finite".to_string(),
            ));
        }

        let mut values = vec![vec![0.0; height]; width];
        for (grid_x, column) in values.iter_mut().enumerate() {
            for (grid_y, value) in column.iter_mut().enumerate() {
                let wx = origin_x + grid_x as f64 * resolution;
                let wy = origin_y + grid_y as f64 * resolution;
                let projection = self.project(wx, wy)?;
                let remaining = (self.total_length() - projection.progress).max(0.0);
                *value = progress_weight * remaining + lateral_weight * projection.lateral_error;
            }
        }
        MppiTerminalValueGrid2D::new(origin_x, origin_y, resolution, values)
    }
}

/// Configuration for online terminal-value updates from MPPI rollouts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiTerminalValueUpdateConfig2D {
    pub learning_rate: f64,
    pub discount: f64,
}

impl Default for MppiTerminalValueUpdateConfig2D {
    fn default() -> Self {
        Self {
            learning_rate: 0.25,
            discount: 0.98,
        }
    }
}

/// Summary returned after updating a terminal value grid from one rollout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiTerminalValueUpdateReport2D {
    pub updates: usize,
    pub mean_abs_delta: f64,
    pub max_abs_delta: f64,
    pub start_target: f64,
    pub terminal_target: f64,
}

/// Online updater for terminal value grids.
pub struct MppiTerminalValueUpdater2D {
    config: MppiTerminalValueUpdateConfig2D,
}

impl MppiTerminalValueUpdater2D {
    pub fn new(config: MppiTerminalValueUpdateConfig2D) -> RoboticsResult<Self> {
        validate_terminal_value_update_config(&config)?;
        Ok(Self { config })
    }

    pub fn update_from_rollout(
        &self,
        grid: &mut MppiTerminalValueGrid2D,
        rollout: &MppiRollout2D,
    ) -> RoboticsResult<MppiTerminalValueUpdateReport2D> {
        validate_rollout_for_terminal_value_update(rollout)?;
        let targets = discounted_cost_to_go(&rollout.stage_costs, self.config.discount);
        let mut updates = 0;
        let mut total_abs_delta = 0.0;
        let mut max_abs_delta: f64 = 0.0;

        for (&state, &target) in rollout.states.iter().zip(&targets) {
            let (x, y) = grid.nearest_cell_indices(state.x, state.y);
            let delta = grid.update_cell_toward(x, y, target, self.config.learning_rate)?;
            updates += 1;
            total_abs_delta += delta;
            max_abs_delta = max_abs_delta.max(delta);
        }

        Ok(MppiTerminalValueUpdateReport2D {
            updates,
            mean_abs_delta: total_abs_delta / updates as f64,
            max_abs_delta,
            start_target: targets[0],
            terminal_target: *targets
                .last()
                .expect("validated rollout has at least one stage cost"),
        })
    }
}

/// Replay buffer for reusing recent MPPI rollouts in terminal-value learning.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiTerminalValueReplayBuffer2D {
    capacity: usize,
    rollouts: VecDeque<MppiRollout2D>,
}

/// Summary returned after replay-buffer terminal-value updates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiTerminalValueReplayUpdateReport2D {
    pub rollouts: usize,
    pub updates: usize,
    pub mean_abs_delta: f64,
    pub max_abs_delta: f64,
}

impl MppiTerminalValueReplayBuffer2D {
    pub fn new(capacity: usize) -> RoboticsResult<Self> {
        if capacity == 0 {
            return Err(RoboticsError::InvalidParameter(
                "terminal value replay capacity must be positive".to_string(),
            ));
        }
        Ok(Self {
            capacity,
            rollouts: VecDeque::with_capacity(capacity),
        })
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }

    pub fn len(&self) -> usize {
        self.rollouts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.rollouts.is_empty()
    }

    pub fn push(&mut self, rollout: MppiRollout2D) -> RoboticsResult<()> {
        validate_rollout_for_terminal_value_update(&rollout)?;
        if self.rollouts.len() == self.capacity {
            self.rollouts.pop_front();
        }
        self.rollouts.push_back(rollout);
        Ok(())
    }

    pub fn update_grid(
        &self,
        grid: &mut MppiTerminalValueGrid2D,
        updater: &MppiTerminalValueUpdater2D,
    ) -> RoboticsResult<MppiTerminalValueReplayUpdateReport2D> {
        if self.rollouts.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "terminal value replay buffer is empty".to_string(),
            ));
        }

        let mut updates = 0;
        let mut weighted_delta_sum = 0.0;
        let mut max_abs_delta: f64 = 0.0;
        for rollout in &self.rollouts {
            let report = updater.update_from_rollout(grid, rollout)?;
            updates += report.updates;
            weighted_delta_sum += report.mean_abs_delta * report.updates as f64;
            max_abs_delta = max_abs_delta.max(report.max_abs_delta);
        }

        Ok(MppiTerminalValueReplayUpdateReport2D {
            rollouts: self.rollouts.len(),
            updates,
            mean_abs_delta: weighted_delta_sum / updates as f64,
            max_abs_delta,
        })
    }
}

/// MPPI configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiConfig {
    pub horizon: usize,
    pub samples: usize,
    pub dt: f64,
    pub lambda: f64,
    pub adaptive_lambda: bool,
    pub min_lambda: f64,
    pub max_lambda: f64,
    pub target_effective_sample_ratio: f64,
    pub adaptive_lambda_iterations: usize,
    pub noise_sigma: f64,
    pub control_limit: f64,
    pub goal_weight: f64,
    pub velocity_weight: f64,
    pub control_weight: f64,
    pub terminal_weight: f64,
    pub constraint_weight: f64,
    pub constraint_discount: f64,
    pub safety_margin: f64,
    pub obstacles: Vec<MppiCircularObstacle2D>,
    pub moving_obstacles: Vec<MppiMovingObstacle2D>,
    pub terminal_value_weight: f64,
    pub terminal_value_grid: Option<MppiTerminalValueGrid2D>,
    pub goal_trajectory: Option<Vec<(f64, f64)>>,
    pub gate_race: Option<MppiGateRace2D>,
    pub seed: u64,
}

impl Default for MppiConfig {
    fn default() -> Self {
        Self {
            horizon: 20,
            samples: 128,
            dt: 0.1,
            lambda: 1.0,
            adaptive_lambda: false,
            min_lambda: 0.05,
            max_lambda: 10.0,
            target_effective_sample_ratio: 0.25,
            adaptive_lambda_iterations: 20,
            noise_sigma: 0.8,
            control_limit: 2.0,
            goal_weight: 2.0,
            velocity_weight: 0.1,
            control_weight: 0.02,
            terminal_weight: 8.0,
            constraint_weight: 40.0,
            constraint_discount: 0.95,
            safety_margin: 0.05,
            obstacles: Vec::new(),
            moving_obstacles: Vec::new(),
            terminal_value_weight: 1.0,
            terminal_value_grid: None,
            goal_trajectory: None,
            gate_race: None,
            seed: 7,
        }
    }
}

/// Diagnostics for MPPI path-integral sample weights.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MppiSamplingDiagnostics2D {
    pub sample_count: usize,
    pub lambda: f64,
    pub min_cost: f64,
    pub mean_cost: f64,
    pub max_cost: f64,
    pub effective_sample_size: f64,
    pub normalized_effective_sample_size: f64,
    pub weight_entropy: f64,
    pub normalized_weight_entropy: f64,
}

/// One MPPI rollout result.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiRollout2D {
    pub cost: f64,
    pub model_cost: f64,
    pub constraint_cost: f64,
    pub terminal_value_cost: f64,
    pub max_constraint_violation: f64,
    pub states: Vec<MppiState2D>,
    pub controls: Vec<MppiControl2D>,
    pub stage_costs: Vec<f64>,
}

/// Result for one MPPI optimization step.
#[derive(Debug, Clone, PartialEq)]
pub struct MppiPlan2D {
    pub first_control: MppiControl2D,
    pub nominal_controls: Vec<MppiControl2D>,
    pub best_rollout: MppiRollout2D,
    pub sampling_diagnostics: MppiSamplingDiagnostics2D,
}

/// Vanilla MPPI controller.
pub struct MppiController2D {
    config: MppiConfig,
    nominal_controls: Vec<MppiControl2D>,
    rng: StdRng,
}

impl MppiController2D {
    pub fn new(config: MppiConfig) -> RoboticsResult<Self> {
        validate_config(&config)?;
        Ok(Self {
            nominal_controls: vec![MppiControl2D::new(0.0, 0.0); config.horizon],
            rng: StdRng::seed_from_u64(config.seed),
            config,
        })
    }

    pub fn plan(&mut self, start: MppiState2D, goal: (f64, f64)) -> RoboticsResult<MppiPlan2D> {
        validate_state(start)?;
        validate_goal(goal)?;
        if let Some(gate_race) = &mut self.config.gate_race {
            gate_race.advance_active_gate_from_state(start);
        }
        let normal = Normal::new(0.0, self.config.noise_sigma).map_err(|err| {
            RoboticsError::InvalidParameter(format!("invalid MPPI noise distribution: {err}"))
        })?;
        let mut candidates = Vec::with_capacity(self.config.samples + 1);

        candidates.push(self.rollout(start, goal, &self.nominal_controls));
        for _ in 0..self.config.samples {
            let mut controls = self.nominal_controls.clone();
            for control in &mut controls {
                control.ax = clamp_control(control.ax + normal.sample(&mut self.rng), &self.config);
                control.ay = clamp_control(control.ay + normal.sample(&mut self.rng), &self.config);
            }
            candidates.push(self.rollout(start, goal, &controls));
        }

        let costs = candidates
            .iter()
            .map(|rollout| rollout.cost)
            .collect::<Vec<_>>();
        let sampling_lambda = select_sampling_lambda(&costs, &self.config)?;
        let sampling_diagnostics = sampling_diagnostics_for_costs(&costs, sampling_lambda)?;
        let beta = sampling_diagnostics.min_cost;
        let mut weight_sum = 0.0;
        let mut next_nominal = vec![MppiControl2D::new(0.0, 0.0); self.config.horizon];
        for rollout in &candidates {
            let weight = sampling_weight(rollout.cost, beta, sampling_lambda);
            weight_sum += weight;
            for (accumulator, control) in next_nominal.iter_mut().zip(&rollout.controls) {
                accumulator.ax += weight * control.ax;
                accumulator.ay += weight * control.ay;
            }
        }
        if weight_sum <= 0.0 || !weight_sum.is_finite() {
            return Err(RoboticsError::PlanningError(
                "MPPI sample weights collapsed".to_string(),
            ));
        }
        for control in &mut next_nominal {
            control.ax = clamp_control(control.ax / weight_sum, &self.config);
            control.ay = clamp_control(control.ay / weight_sum, &self.config);
        }

        let first_control = next_nominal[0];
        self.nominal_controls = shift_controls(&next_nominal);
        let best_rollout = candidates
            .into_iter()
            .min_by(|a, b| a.cost.total_cmp(&b.cost))
            .expect("at least one candidate rollout");

        Ok(MppiPlan2D {
            first_control,
            nominal_controls: next_nominal,
            best_rollout,
            sampling_diagnostics,
        })
    }

    pub fn terminal_value_grid(&self) -> Option<&MppiTerminalValueGrid2D> {
        self.config.terminal_value_grid.as_ref()
    }

    pub fn terminal_value_grid_mut(&mut self) -> Option<&mut MppiTerminalValueGrid2D> {
        self.config.terminal_value_grid.as_mut()
    }

    pub fn set_terminal_value_grid(
        &mut self,
        grid: Option<MppiTerminalValueGrid2D>,
    ) -> RoboticsResult<()> {
        if let Some(grid) = &grid {
            validate_terminal_value_grid(grid)?;
        }
        self.config.terminal_value_grid = grid;
        Ok(())
    }

    pub fn set_moving_obstacles(
        &mut self,
        moving_obstacles: Vec<MppiMovingObstacle2D>,
    ) -> RoboticsResult<()> {
        validate_moving_obstacles(&moving_obstacles)?;
        self.config.moving_obstacles = moving_obstacles;
        Ok(())
    }

    pub fn set_goal_trajectory(
        &mut self,
        goal_trajectory: Option<Vec<(f64, f64)>>,
    ) -> RoboticsResult<()> {
        if let Some(goal_trajectory) = &goal_trajectory {
            validate_goal_trajectory(goal_trajectory)?;
        }
        self.config.goal_trajectory = goal_trajectory;
        Ok(())
    }

    fn rollout(
        &self,
        start: MppiState2D,
        goal: (f64, f64),
        controls: &[MppiControl2D],
    ) -> MppiRollout2D {
        let mut state = start;
        let mut states = Vec::with_capacity(controls.len() + 1);
        let mut model_cost = 0.0;
        let mut constraint_cost = 0.0;
        let mut max_constraint_violation: f64 = 0.0;
        let mut stage_costs = Vec::with_capacity(controls.len() + 1);
        let mut active_gate_index = self
            .config
            .gate_race
            .as_ref()
            .map_or(0, MppiGateRace2D::active_gate_index);
        states.push(state);
        for (step, &control) in controls.iter().enumerate() {
            let (step_constraint_cost, step_violation) = self.constraint_cost(state, step);
            constraint_cost += step_constraint_cost;
            max_constraint_violation = max_constraint_violation.max(step_violation);
            let next_state = state.step(control, self.config.dt);
            let gate_step_cost = if let Some(gate_race) = &self.config.gate_race {
                let gate_step = gate_race.transition_cost(state, next_state, active_gate_index);
                active_gate_index = gate_step.active_gate_index;
                gate_step.cost
            } else {
                0.0
            };
            let step_goal = self.goal_at(step, goal);
            let step_cost = self.running_cost(state, control, step_goal)
                + step_constraint_cost
                + gate_step_cost;
            model_cost += step_cost;
            stage_costs.push(step_cost);
            state = next_state;
            states.push(state);
        }
        let (terminal_constraint_cost, terminal_violation) =
            self.constraint_cost(state, controls.len());
        constraint_cost += terminal_constraint_cost;
        max_constraint_violation = max_constraint_violation.max(terminal_violation);
        let terminal_value_cost = self.terminal_value_cost(state);
        let terminal_gate_cost = self.config.gate_race.as_ref().map_or(0.0, |gate_race| {
            gate_race.terminal_cost(state, active_gate_index)
        });
        let terminal_goal = self.goal_at(controls.len(), goal);
        let terminal_model_cost = self.config.terminal_weight
            * squared_goal_distance(state, terminal_goal)
            + terminal_constraint_cost
            + terminal_gate_cost;
        model_cost += terminal_model_cost;
        stage_costs.push(terminal_model_cost);
        MppiRollout2D {
            cost: model_cost + terminal_value_cost,
            model_cost,
            constraint_cost,
            terminal_value_cost,
            max_constraint_violation,
            states,
            controls: controls.to_vec(),
            stage_costs,
        }
    }

    fn goal_at(&self, step: usize, fallback_goal: (f64, f64)) -> (f64, f64) {
        self.config
            .goal_trajectory
            .as_ref()
            .and_then(|goals| goals.get(step).copied().or_else(|| goals.last().copied()))
            .unwrap_or(fallback_goal)
    }

    fn running_cost(&self, state: MppiState2D, control: MppiControl2D, goal: (f64, f64)) -> f64 {
        self.config.goal_weight * squared_goal_distance(state, goal)
            + self.config.velocity_weight * (state.vx * state.vx + state.vy * state.vy)
            + self.config.control_weight * (control.ax * control.ax + control.ay * control.ay)
    }

    fn constraint_cost(&self, state: MppiState2D, step: usize) -> (f64, f64) {
        let mut penalty = 0.0;
        let mut max_violation: f64 = 0.0;
        for obstacle in &self.config.obstacles {
            let violation =
                circular_obstacle_violation(state, *obstacle, self.config.safety_margin);
            max_violation = max_violation.max(violation);
            penalty += violation * violation;
        }
        let predicted_time = step as f64 * self.config.dt;
        for obstacle in &self.config.moving_obstacles {
            let predicted_obstacle = obstacle.predict(predicted_time);
            let violation =
                circular_obstacle_violation(state, predicted_obstacle, self.config.safety_margin);
            max_violation = max_violation.max(violation);
            penalty += violation * violation;
        }
        let discount = self.config.constraint_discount.powi(step as i32);
        (
            self.config.constraint_weight * discount * penalty,
            max_violation,
        )
    }

    fn terminal_value_cost(&self, state: MppiState2D) -> f64 {
        self.config
            .terminal_value_grid
            .as_ref()
            .map_or(0.0, |grid| {
                self.config.terminal_value_weight * grid.value_at_state(state)
            })
    }
}

fn validate_config(config: &MppiConfig) -> RoboticsResult<()> {
    if config.horizon == 0 || config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "horizon and samples must be positive".to_string(),
        ));
    }
    for (label, value) in [
        ("dt", config.dt),
        ("lambda", config.lambda),
        ("min_lambda", config.min_lambda),
        ("max_lambda", config.max_lambda),
        ("noise_sigma", config.noise_sigma),
        ("control_limit", config.control_limit),
    ] {
        if value <= 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and positive"
            )));
        }
    }
    for (label, value) in [
        ("goal_weight", config.goal_weight),
        ("velocity_weight", config.velocity_weight),
        ("control_weight", config.control_weight),
        ("terminal_weight", config.terminal_weight),
        ("constraint_weight", config.constraint_weight),
        ("constraint_discount", config.constraint_discount),
        ("terminal_value_weight", config.terminal_value_weight),
    ] {
        if value < 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and non-negative"
            )));
        }
    }
    if config.max_lambda < config.min_lambda {
        return Err(RoboticsError::InvalidParameter(
            "max_lambda must be at least min_lambda".to_string(),
        ));
    }
    if !(0.0..=1.0).contains(&config.target_effective_sample_ratio)
        || config.target_effective_sample_ratio == 0.0
    {
        return Err(RoboticsError::InvalidParameter(
            "target_effective_sample_ratio must be in (0, 1]".to_string(),
        ));
    }
    if config.adaptive_lambda_iterations == 0 {
        return Err(RoboticsError::InvalidParameter(
            "adaptive_lambda_iterations must be positive".to_string(),
        ));
    }
    if config.constraint_discount > 1.0 {
        return Err(RoboticsError::InvalidParameter(
            "constraint_discount must be at most 1.0".to_string(),
        ));
    }
    if config.safety_margin < 0.0 || !config.safety_margin.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "safety_margin must be finite and non-negative".to_string(),
        ));
    }
    for obstacle in &config.obstacles {
        if !obstacle.x.is_finite() || !obstacle.y.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "obstacle coordinates must be finite".to_string(),
            ));
        }
        if obstacle.radius <= 0.0 || !obstacle.radius.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "obstacle radius must be finite and positive".to_string(),
            ));
        }
    }
    validate_moving_obstacles(&config.moving_obstacles)?;
    if let Some(goal_trajectory) = &config.goal_trajectory {
        validate_goal_trajectory(goal_trajectory)?;
    }
    if let Some(grid) = &config.terminal_value_grid {
        validate_terminal_value_grid(grid)?;
    }
    if let Some(gate_race) = &config.gate_race {
        validate_gate_race(gate_race)?;
    }
    Ok(())
}

fn validate_moving_obstacles(obstacles: &[MppiMovingObstacle2D]) -> RoboticsResult<()> {
    for obstacle in obstacles {
        if !obstacle.x.is_finite()
            || !obstacle.y.is_finite()
            || !obstacle.vx.is_finite()
            || !obstacle.vy.is_finite()
        {
            return Err(RoboticsError::InvalidParameter(
                "moving obstacle state must be finite".to_string(),
            ));
        }
        if obstacle.radius <= 0.0 || !obstacle.radius.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "moving obstacle radius must be finite and positive".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_goal_trajectory(goal_trajectory: &[(f64, f64)]) -> RoboticsResult<()> {
    if goal_trajectory.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "MPPI goal trajectory must be non-empty".to_string(),
        ));
    }
    for &goal in goal_trajectory {
        validate_goal(goal)?;
    }
    Ok(())
}

fn validate_gate_race(gate_race: &MppiGateRace2D) -> RoboticsResult<()> {
    if gate_race.gates.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "gate race must contain at least one gate".to_string(),
        ));
    }
    if gate_race.active_gate_index > gate_race.gates.len() {
        return Err(RoboticsError::InvalidParameter(
            "active gate index must be within the gate list".to_string(),
        ));
    }
    for (label, value) in [
        ("gate race progress_weight", gate_race.progress_weight),
        ("gate race pass_bonus", gate_race.pass_bonus),
        ("gate race miss_penalty", gate_race.miss_penalty),
        (
            "gate race terminal_gate_weight",
            gate_race.terminal_gate_weight,
        ),
    ] {
        if value <= 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and positive"
            )));
        }
    }
    for (label, value) in [
        ("gate race lateral_weight", gate_race.lateral_weight),
        ("gate race crossing_tolerance", gate_race.crossing_tolerance),
    ] {
        if value < 0.0 || !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite and non-negative"
            )));
        }
    }
    for gate in &gate_race.gates {
        let normal_norm = (gate.normal_x * gate.normal_x + gate.normal_y * gate.normal_y).sqrt();
        if !gate.center_x.is_finite()
            || !gate.center_y.is_finite()
            || !gate.normal_x.is_finite()
            || !gate.normal_y.is_finite()
            || normal_norm <= 0.0
        {
            return Err(RoboticsError::InvalidParameter(
                "racing gate center and normal must be finite".to_string(),
            ));
        }
        if gate.half_width <= 0.0 || !gate.half_width.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "racing gate half_width must be finite and positive".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_track_waypoints(waypoints: &[(f64, f64)]) -> RoboticsResult<()> {
    if waypoints.len() < 2 {
        return Err(RoboticsError::InvalidParameter(
            "track must contain at least two waypoints".to_string(),
        ));
    }
    for &(x, y) in waypoints {
        if !x.is_finite() || !y.is_finite() {
            return Err(RoboticsError::InvalidParameter(
                "track waypoints must be finite".to_string(),
            ));
        }
    }
    for pair in waypoints.windows(2) {
        if point_distance(pair[0], pair[1]) <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "track segments must have positive length".to_string(),
            ));
        }
    }
    Ok(())
}

fn point_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

fn sampling_weight(cost: f64, beta: f64, lambda: f64) -> f64 {
    (-(cost - beta) / lambda).exp()
}

fn sampling_diagnostics_for_costs(
    costs: &[f64],
    lambda: f64,
) -> RoboticsResult<MppiSamplingDiagnostics2D> {
    if costs.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "MPPI sampling costs must be non-empty".to_string(),
        ));
    }
    if lambda <= 0.0 || !lambda.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "MPPI sampling lambda must be finite and positive".to_string(),
        ));
    }
    if costs.iter().any(|cost| !cost.is_finite()) {
        return Err(RoboticsError::InvalidParameter(
            "MPPI sampling costs must be finite".to_string(),
        ));
    }

    let sample_count = costs.len();
    let min_cost = costs.iter().copied().fold(f64::INFINITY, f64::min);
    let max_cost = costs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mean_cost = costs.iter().sum::<f64>() / sample_count as f64;
    let mut weight_sum = 0.0;
    let mut squared_weight_sum = 0.0;
    let mut weights = Vec::with_capacity(sample_count);
    for &cost in costs {
        let weight = sampling_weight(cost, min_cost, lambda);
        weight_sum += weight;
        squared_weight_sum += weight * weight;
        weights.push(weight);
    }
    if weight_sum <= 0.0 || squared_weight_sum <= 0.0 || !weight_sum.is_finite() {
        return Err(RoboticsError::PlanningError(
            "MPPI sample weights collapsed".to_string(),
        ));
    }

    let effective_sample_size = weight_sum * weight_sum / squared_weight_sum;
    let mut weight_entropy = 0.0;
    for weight in weights {
        let probability = weight / weight_sum;
        if probability > 0.0 {
            weight_entropy -= probability * probability.ln();
        }
    }
    let sample_count_f = sample_count as f64;
    let normalized_weight_entropy = if sample_count > 1 {
        weight_entropy / sample_count_f.ln()
    } else {
        1.0
    };

    Ok(MppiSamplingDiagnostics2D {
        sample_count,
        lambda,
        min_cost,
        mean_cost,
        max_cost,
        effective_sample_size,
        normalized_effective_sample_size: effective_sample_size / sample_count_f,
        weight_entropy,
        normalized_weight_entropy,
    })
}

fn select_sampling_lambda(costs: &[f64], config: &MppiConfig) -> RoboticsResult<f64> {
    if !config.adaptive_lambda {
        return Ok(config.lambda);
    }

    let target_effective_sample_size = config.target_effective_sample_ratio * costs.len() as f64;
    let min_diag = sampling_diagnostics_for_costs(costs, config.min_lambda)?;
    if min_diag.effective_sample_size >= target_effective_sample_size {
        return Ok(config.min_lambda);
    }
    let max_diag = sampling_diagnostics_for_costs(costs, config.max_lambda)?;
    if max_diag.effective_sample_size <= target_effective_sample_size {
        return Ok(config.max_lambda);
    }

    let mut low = config.min_lambda;
    let mut high = config.max_lambda;
    for _ in 0..config.adaptive_lambda_iterations {
        let mid = 0.5 * (low + high);
        let diagnostics = sampling_diagnostics_for_costs(costs, mid)?;
        if diagnostics.effective_sample_size < target_effective_sample_size {
            low = mid;
        } else {
            high = mid;
        }
    }
    Ok(high)
}

fn validate_terminal_value_grid(grid: &MppiTerminalValueGrid2D) -> RoboticsResult<()> {
    if !grid.origin_x.is_finite() || !grid.origin_y.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "terminal value grid origin must be finite".to_string(),
        ));
    }
    if grid.resolution <= 0.0 || !grid.resolution.is_finite() {
        return Err(RoboticsError::InvalidParameter(
            "terminal value grid resolution must be finite and positive".to_string(),
        ));
    }
    if grid.values.is_empty() || grid.values[0].is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "terminal value grid values must be non-empty".to_string(),
        ));
    }
    let height = grid.values[0].len();
    for column in &grid.values {
        if column.len() != height {
            return Err(RoboticsError::InvalidParameter(
                "terminal value grid must be rectangular".to_string(),
            ));
        }
        if column
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(RoboticsError::InvalidParameter(
                "terminal value grid values must be finite and non-negative".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_terminal_value_update_config(
    config: &MppiTerminalValueUpdateConfig2D,
) -> RoboticsResult<()> {
    if !(0.0..=1.0).contains(&config.learning_rate) || config.learning_rate == 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "terminal value update learning_rate must be in (0, 1]".to_string(),
        ));
    }
    if !(0.0..=1.0).contains(&config.discount) {
        return Err(RoboticsError::InvalidParameter(
            "terminal value update discount must be in [0, 1]".to_string(),
        ));
    }
    Ok(())
}

fn validate_rollout_for_terminal_value_update(rollout: &MppiRollout2D) -> RoboticsResult<()> {
    if rollout.states.is_empty() || rollout.stage_costs.is_empty() {
        return Err(RoboticsError::InvalidParameter(
            "rollout states and stage_costs must be non-empty".to_string(),
        ));
    }
    if rollout.states.len() != rollout.stage_costs.len() {
        return Err(RoboticsError::InvalidParameter(
            "rollout states and stage_costs must have the same length".to_string(),
        ));
    }
    for &state in &rollout.states {
        validate_state(state)?;
    }
    if rollout
        .stage_costs
        .iter()
        .any(|value| !value.is_finite() || *value < 0.0)
    {
        return Err(RoboticsError::InvalidParameter(
            "rollout stage_costs must be finite and non-negative".to_string(),
        ));
    }
    Ok(())
}

fn discounted_cost_to_go(stage_costs: &[f64], discount: f64) -> Vec<f64> {
    let mut targets = vec![0.0; stage_costs.len()];
    let mut target = 0.0;
    for index in (0..stage_costs.len()).rev() {
        target = stage_costs[index] + discount * target;
        targets[index] = target;
    }
    targets
}

fn validate_state(state: MppiState2D) -> RoboticsResult<()> {
    for (label, value) in [
        ("x", state.x),
        ("y", state.y),
        ("vx", state.vx),
        ("vy", state.vy),
    ] {
        if !value.is_finite() {
            return Err(RoboticsError::InvalidParameter(format!(
                "{label} must be finite"
            )));
        }
    }
    Ok(())
}

fn validate_goal(goal: (f64, f64)) -> RoboticsResult<()> {
    if goal.0.is_finite() && goal.1.is_finite() {
        Ok(())
    } else {
        Err(RoboticsError::InvalidParameter(
            "goal coordinates must be finite".to_string(),
        ))
    }
}

fn squared_goal_distance(state: MppiState2D, goal: (f64, f64)) -> f64 {
    let dx = state.x - goal.0;
    let dy = state.y - goal.1;
    dx * dx + dy * dy
}

fn circular_obstacle_violation(
    state: MppiState2D,
    obstacle: MppiCircularObstacle2D,
    safety_margin: f64,
) -> f64 {
    let dx = state.x - obstacle.x;
    let dy = state.y - obstacle.y;
    let clearance = (dx * dx + dy * dy).sqrt() - obstacle.radius - safety_margin;
    (-clearance).max(0.0)
}

fn clamp_control(value: f64, config: &MppiConfig) -> f64 {
    value.clamp(-config.control_limit, config.control_limit)
}

fn shift_controls(controls: &[MppiControl2D]) -> Vec<MppiControl2D> {
    let mut shifted = controls[1..].to_vec();
    shifted.push(MppiControl2D::new(0.0, 0.0));
    shifted
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_config(seed: u64) -> MppiConfig {
        MppiConfig {
            horizon: 12,
            samples: 64,
            dt: 0.1,
            seed,
            ..MppiConfig::default()
        }
    }

    #[test]
    fn mppi_accelerates_toward_goal() {
        let mut controller = MppiController2D::new(small_config(3)).unwrap();
        let plan = controller
            .plan(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (2.0, 0.0))
            .unwrap();

        assert!(plan.first_control.ax > 0.0);
        assert!(plan.first_control.ay.abs() < 0.8);
        assert_eq!(plan.nominal_controls.len(), 12);
    }

    #[test]
    fn rollout_best_cost_is_finite() {
        let mut controller = MppiController2D::new(small_config(4)).unwrap();
        let plan = controller
            .plan(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (1.0, 1.0))
            .unwrap();

        assert!(plan.best_rollout.cost.is_finite());
        assert_eq!(plan.best_rollout.states.len(), 13);
    }

    #[test]
    fn plan_reports_sampling_diagnostics() {
        let mut controller = MppiController2D::new(small_config(8)).unwrap();
        let plan = controller
            .plan(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (1.5, 0.0))
            .unwrap();

        let diagnostics = plan.sampling_diagnostics;
        assert_eq!(diagnostics.sample_count, 65);
        assert_eq!(diagnostics.lambda, 1.0);
        assert!(diagnostics.min_cost <= diagnostics.mean_cost);
        assert!(diagnostics.mean_cost <= diagnostics.max_cost);
        assert!(diagnostics.effective_sample_size >= 1.0);
        assert!(diagnostics.effective_sample_size <= diagnostics.sample_count as f64);
        assert!(diagnostics.normalized_effective_sample_size > 0.0);
        assert!(diagnostics.normalized_effective_sample_size <= 1.0);
        assert!(diagnostics.normalized_weight_entropy >= 0.0);
        assert!(diagnostics.normalized_weight_entropy <= 1.0);
    }

    #[test]
    fn adaptive_lambda_raises_effective_sample_size() {
        let costs = vec![0.0, 10.0, 20.0, 30.0, 40.0];
        let fixed = sampling_diagnostics_for_costs(&costs, 0.05).unwrap();
        let config = MppiConfig {
            adaptive_lambda: true,
            min_lambda: 0.05,
            max_lambda: 100.0,
            target_effective_sample_ratio: 0.6,
            adaptive_lambda_iterations: 32,
            ..MppiConfig::default()
        };
        let selected_lambda = select_sampling_lambda(&costs, &config).unwrap();
        let adaptive = sampling_diagnostics_for_costs(&costs, selected_lambda).unwrap();

        assert!(selected_lambda > config.min_lambda);
        assert!(adaptive.effective_sample_size > fixed.effective_sample_size);
        assert!(adaptive.normalized_effective_sample_size >= 0.55);
    }

    #[test]
    fn circular_obstacle_violation_respects_safety_margin() {
        let obstacle = MppiCircularObstacle2D::new(1.0, 0.0, 0.5);

        let outside =
            circular_obstacle_violation(MppiState2D::new(2.0, 0.0, 0.0, 0.0), obstacle, 0.1);
        let inside =
            circular_obstacle_violation(MppiState2D::new(1.2, 0.0, 0.0, 0.0), obstacle, 0.1);

        assert_eq!(outside, 0.0);
        assert!((inside - 0.4).abs() < 1e-9);
    }

    #[test]
    fn constraint_cost_is_discounted_over_horizon() {
        let config = MppiConfig {
            obstacles: vec![MppiCircularObstacle2D::new(1.0, 0.0, 0.5)],
            constraint_weight: 10.0,
            constraint_discount: 0.5,
            safety_margin: 0.0,
            ..small_config(5)
        };
        let controller = MppiController2D::new(config).unwrap();
        let state = MppiState2D::new(1.25, 0.0, 0.0, 0.0);

        let (early_cost, early_violation) = controller.constraint_cost(state, 0);
        let (late_cost, late_violation) = controller.constraint_cost(state, 3);

        assert!((early_violation - 0.25).abs() < 1e-9);
        assert_eq!(early_violation, late_violation);
        assert!((early_cost - 0.625).abs() < 1e-9);
        assert!((late_cost - 0.078125).abs() < 1e-9);
    }

    #[test]
    fn rollout_reports_constraint_cost_and_violation() {
        let config = MppiConfig {
            obstacles: vec![MppiCircularObstacle2D::new(0.0, 0.0, 0.5)],
            constraint_weight: 10.0,
            safety_margin: 0.0,
            ..small_config(6)
        };
        let controller = MppiController2D::new(config).unwrap();
        let controls = vec![MppiControl2D::new(0.0, 0.0); 12];
        let rollout =
            controller.rollout(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (1.0, 0.0), &controls);

        assert!(rollout.constraint_cost > 0.0);
        assert!((rollout.max_constraint_violation - 0.5).abs() < 1e-9);
    }

    #[test]
    fn terminal_value_grid_interpolates_and_clamps() {
        let grid =
            MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![0.0, 1.0], vec![1.0, 2.0]])
                .unwrap();

        assert!((grid.value_at(0.5, 0.5) - 1.0).abs() < 1e-9);
        assert!((grid.value_at(-10.0, 0.5) - 0.5).abs() < 1e-9);
        assert!((grid.value_at(10.0, 10.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn waypoint_track_projects_points_to_progress() {
        let track = MppiWaypointTrack2D::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]).unwrap();

        let first = track.project(0.5, 0.2).unwrap();
        let second = track.project(1.2, 0.6).unwrap();

        assert_eq!(first.segment_index, 0);
        assert!((first.progress - 0.5).abs() < 1e-9);
        assert!((first.lateral_error - 0.2).abs() < 1e-9);
        assert_eq!(second.segment_index, 1);
        assert!((second.progress - 1.6).abs() < 1e-9);
        assert!((track.total_length() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn waypoint_track_returns_point_at_progress() {
        let track = MppiWaypointTrack2D::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]).unwrap();

        let first = track.point_at_progress(0.5).unwrap();
        let second = track.point_at_progress(2.0).unwrap();
        let clamped = track.point_at_progress(10.0).unwrap();

        assert!((first.0 - 0.5).abs() < 1e-9);
        assert!((first.1 - 0.0).abs() < 1e-9);
        assert!((second.0 - 1.0).abs() < 1e-9);
        assert!((second.1 - 1.0).abs() < 1e-9);
        assert_eq!(clamped, track.goal());
    }

    #[test]
    fn waypoint_track_terminal_grid_prefers_progress_and_low_lateral_error() {
        let track = MppiWaypointTrack2D::new(vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]).unwrap();
        let grid = track
            .terminal_value_grid(5, 5, 0.0, -1.0, 0.5, 1.0, 2.0)
            .unwrap();

        let start_value = grid.value_at(0.0, 0.0);
        let middle_value = grid.value_at(1.0, 0.0);
        let goal_value = grid.value_at(2.0, 0.0);
        let off_track_value = grid.value_at(1.0, 1.0);

        assert!(start_value > middle_value);
        assert!(middle_value > goal_value);
        assert!(off_track_value > middle_value);
    }

    #[test]
    fn moving_obstacle_is_predicted_in_constraint_cost() {
        let config = MppiConfig {
            moving_obstacles: vec![MppiMovingObstacle2D::new(0.0, 0.0, 1.0, 0.0, 0.4)],
            constraint_weight: 10.0,
            safety_margin: 0.0,
            ..small_config(5)
        };
        let controller = MppiController2D::new(config).unwrap();
        let state = MppiState2D::new(0.2, 0.0, 0.0, 0.0);
        let future_state = MppiState2D::new(0.5, 0.0, 0.0, 0.0);

        let (early_cost, early_violation) = controller.constraint_cost(state, 0);
        let (future_cost, future_violation) = controller.constraint_cost(future_state, 5);

        assert!(early_cost > 0.0);
        assert!((early_violation - 0.2).abs() < 1e-9);
        assert!(future_cost > 0.0);
        assert!((future_violation - 0.4).abs() < 1e-9);
    }

    #[test]
    fn rollout_uses_horizon_goal_trajectory_when_present() {
        let config = MppiConfig {
            horizon: 1,
            samples: 1,
            goal_weight: 1.0,
            velocity_weight: 0.0,
            control_weight: 0.0,
            terminal_weight: 1.0,
            goal_trajectory: Some(vec![(0.0, 0.0), (2.0, 0.0)]),
            ..MppiConfig::default()
        };
        let controller = MppiController2D::new(config).unwrap();
        let controls = vec![MppiControl2D::new(0.0, 0.0)];
        let rollout =
            controller.rollout(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (10.0, 0.0), &controls);

        assert!((rollout.stage_costs[0] - 0.0).abs() < 1e-9);
        assert!((rollout.stage_costs[1] - 4.0).abs() < 1e-9);
        assert!((rollout.model_cost - 4.0).abs() < 1e-9);
    }

    #[test]
    fn racing_gate_normalizes_and_detects_crossing() {
        let gate = MppiRacingGate2D::new(1.0, 0.0, 2.0, 0.0, 0.35).unwrap();
        let from = MppiState2D::new(0.8, 0.1, 0.0, 0.0);
        let through = MppiState2D::new(1.2, 0.1, 0.0, 0.0);
        let wide = MppiState2D::new(1.2, 0.8, 0.0, 0.0);

        assert!((gate.normal_x - 1.0).abs() < 1e-9);
        assert!(gate.crosses(from, through, 0.0));
        assert!(!gate.crosses(from, wide, 0.0));
        assert!(gate.state_is_past_gate(through, 0.0));
        assert!(!gate.state_is_past_gate(wide, 0.0));
    }

    #[test]
    fn gate_race_scores_progress_and_switches_gates() {
        let race = MppiGateRace2D::new(vec![
            MppiRacingGate2D::new(1.0, 0.0, 1.0, 0.0, 0.35).unwrap(),
            MppiRacingGate2D::new(2.0, 0.0, 1.0, 0.0, 0.35).unwrap(),
        ])
        .unwrap();
        let states = vec![
            MppiState2D::new(0.0, 0.0, 0.0, 0.0),
            MppiState2D::new(1.1, 0.0, 0.0, 0.0),
            MppiState2D::new(2.1, 0.0, 0.0, 0.0),
        ];

        let report = race.score_trajectory(&states).unwrap();

        assert_eq!(report.passed_gates, 2);
        assert_eq!(report.active_gate_index, 2);
        assert_eq!(report.final_gate_distance, 0.0);
        assert!(report.cost < 0.0);
    }

    #[test]
    fn gate_race_rollout_can_run_without_goal_tracking_cost() {
        let race = MppiGateRace2D::new(vec![
            MppiRacingGate2D::new(0.01, 0.0, 1.0, 0.0, 0.3).unwrap()
        ])
        .unwrap();
        let config = MppiConfig {
            horizon: 1,
            samples: 1,
            goal_weight: 0.0,
            velocity_weight: 0.0,
            control_weight: 0.0,
            terminal_weight: 0.0,
            gate_race: Some(race),
            ..MppiConfig::default()
        };
        let controller = MppiController2D::new(config).unwrap();
        let controls = vec![MppiControl2D::new(2.0, 0.0)];
        let rollout = controller.rollout(
            MppiState2D::new(0.0, 0.0, 0.0, 0.0),
            (10.0, 10.0),
            &controls,
        );

        assert!(rollout.cost < 0.0);
        assert_eq!(rollout.constraint_cost, 0.0);
        assert_eq!(rollout.terminal_value_cost, 0.0);
    }

    #[test]
    fn waypoint_track_rejects_invalid_waypoints() {
        assert!(MppiWaypointTrack2D::new(vec![(0.0, 0.0)]).is_err());
        assert!(MppiWaypointTrack2D::new(vec![(0.0, 0.0), (0.0, 0.0)]).is_err());
        assert!(MppiWaypointTrack2D::new(vec![(0.0, 0.0), (f64::NAN, 1.0)]).is_err());
    }

    #[test]
    fn terminal_value_cost_is_added_to_rollout() {
        let grid = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![3.0]]).unwrap();
        let config = MppiConfig {
            horizon: 1,
            samples: 1,
            terminal_value_weight: 2.0,
            terminal_value_grid: Some(grid),
            ..MppiConfig::default()
        };
        let controller = MppiController2D::new(config).unwrap();
        let controls = vec![MppiControl2D::new(0.0, 0.0)];
        let rollout =
            controller.rollout(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (0.0, 0.0), &controls);

        assert!((rollout.terminal_value_cost - 6.0).abs() < 1e-9);
        assert!((rollout.cost - 6.0).abs() < 1e-9);
        assert!((rollout.model_cost - 0.0).abs() < 1e-9);
        assert!((rollout.cost - rollout.model_cost - rollout.terminal_value_cost).abs() < 1e-9);
        assert_eq!(rollout.stage_costs.len(), rollout.states.len());
    }

    #[test]
    fn rejects_invalid_terminal_value_grid() {
        let ragged = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![1.0], vec![]]);
        assert!(matches!(ragged, Err(RoboticsError::InvalidParameter(_))));

        let negative = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![-1.0]]);
        assert!(matches!(negative, Err(RoboticsError::InvalidParameter(_))));
    }

    #[test]
    fn terminal_value_updater_learns_discounted_rollout_costs() {
        let mut grid = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![0.0, 0.0]]).unwrap();
        let updater = MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D {
            learning_rate: 1.0,
            discount: 0.5,
        })
        .unwrap();
        let rollout = MppiRollout2D {
            cost: 3.0,
            model_cost: 3.0,
            constraint_cost: 0.0,
            terminal_value_cost: 0.0,
            max_constraint_violation: 0.0,
            states: vec![
                MppiState2D::new(0.0, 0.0, 0.0, 0.0),
                MppiState2D::new(0.0, 1.0, 0.0, 0.0),
            ],
            controls: vec![MppiControl2D::new(0.0, 0.0)],
            stage_costs: vec![2.0, 1.0],
        };

        let report = updater.update_from_rollout(&mut grid, &rollout).unwrap();

        assert_eq!(report.updates, 2);
        assert!((report.start_target - 2.5).abs() < 1e-9);
        assert!((report.terminal_target - 1.0).abs() < 1e-9);
        assert!((grid.value_at_cell(0, 0).unwrap() - 2.5).abs() < 1e-9);
        assert!((grid.value_at_cell(0, 1).unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn terminal_value_updater_rejects_mismatched_rollout() {
        let mut grid = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![0.0]]).unwrap();
        let updater =
            MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D::default()).unwrap();
        let rollout = MppiRollout2D {
            cost: 0.0,
            model_cost: 0.0,
            constraint_cost: 0.0,
            terminal_value_cost: 0.0,
            max_constraint_violation: 0.0,
            states: vec![MppiState2D::new(0.0, 0.0, 0.0, 0.0)],
            controls: Vec::new(),
            stage_costs: Vec::new(),
        };

        assert!(matches!(
            updater.update_from_rollout(&mut grid, &rollout),
            Err(RoboticsError::InvalidParameter(_))
        ));
    }

    fn synthetic_terminal_rollout(y: f64, stage_cost: f64) -> MppiRollout2D {
        MppiRollout2D {
            cost: stage_cost,
            model_cost: stage_cost,
            constraint_cost: 0.0,
            terminal_value_cost: 0.0,
            max_constraint_violation: 0.0,
            states: vec![MppiState2D::new(0.0, y, 0.0, 0.0)],
            controls: Vec::new(),
            stage_costs: vec![stage_cost],
        }
    }

    #[test]
    fn terminal_value_replay_buffer_caps_capacity_and_updates_grid() {
        let updater = MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D {
            learning_rate: 1.0,
            discount: 0.0,
        })
        .unwrap();
        let mut buffer = MppiTerminalValueReplayBuffer2D::new(2).unwrap();
        buffer.push(synthetic_terminal_rollout(0.0, 9.0)).unwrap();
        buffer.push(synthetic_terminal_rollout(1.0, 2.0)).unwrap();
        buffer.push(synthetic_terminal_rollout(2.0, 3.0)).unwrap();

        let mut grid =
            MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![0.0, 0.0, 0.0]]).unwrap();
        let report = buffer.update_grid(&mut grid, &updater).unwrap();

        assert_eq!(buffer.capacity(), 2);
        assert_eq!(buffer.len(), 2);
        assert_eq!(report.rollouts, 2);
        assert_eq!(report.updates, 2);
        assert_eq!(grid.value_at_cell(0, 0).unwrap(), 0.0);
        assert_eq!(grid.value_at_cell(0, 1).unwrap(), 2.0);
        assert_eq!(grid.value_at_cell(0, 2).unwrap(), 3.0);
    }

    #[test]
    fn terminal_value_replay_buffer_rejects_empty_updates() {
        let updater =
            MppiTerminalValueUpdater2D::new(MppiTerminalValueUpdateConfig2D::default()).unwrap();
        let buffer = MppiTerminalValueReplayBuffer2D::new(2).unwrap();
        let mut grid = MppiTerminalValueGrid2D::new(0.0, 0.0, 1.0, vec![vec![0.0]]).unwrap();

        assert!(MppiTerminalValueReplayBuffer2D::new(0).is_err());
        assert!(buffer.is_empty());
        assert!(matches!(
            buffer.update_grid(&mut grid, &updater),
            Err(RoboticsError::InvalidParameter(_))
        ));
    }

    #[test]
    fn repeated_seed_is_reproducible() {
        let mut first = MppiController2D::new(small_config(11)).unwrap();
        let mut second = MppiController2D::new(small_config(11)).unwrap();
        let a = first
            .plan(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (1.0, 0.0))
            .unwrap();
        let b = second
            .plan(MppiState2D::new(0.0, 0.0, 0.0, 0.0), (1.0, 0.0))
            .unwrap();

        assert_eq!(a.first_control, b.first_control);
    }

    #[test]
    fn rejects_invalid_config() {
        let config = MppiConfig {
            horizon: 0,
            ..MppiConfig::default()
        };
        assert!(matches!(
            MppiController2D::new(config),
            Err(RoboticsError::InvalidParameter(_))
        ));
    }
}
