//! Reference-free racing MPPI through 3-D gate planes.
//!
//! This extends the 2-D gate-progress objective in [`crate::mppi`] to a 3-D
//! drone-racing setting:
//!
//! - [`RacingGatePlane3D`] is a rectangular gate aperture in 3-D defined by a
//!   center, a race-direction normal, and an in-plane up axis. A rollout passes
//!   the gate when a segment crosses the gate plane from behind to in front and
//!   the crossing point lies inside the rectangular aperture.
//! - [`RacingDroneDynamics3D`] is a point-mass drone with linear aerodynamic
//!   drag, optional gravity, a speed cap, and an acceleration-magnitude cap —
//!   richer than the pure 2-D double integrator.
//! - [`RacingGateLap3D`] arranges gates into an open course or a closed lap and
//!   scores reference-free gate progress.
//! - [`simulate_lap_race`] drives a deterministic, seeded MPPI controller around
//!   the lap and reports [`RacingLapReport3D`] lap-progress metrics.

use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// State for a 3-D point-mass drone: position \[m\] and velocity \[m/s\].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacingDroneState3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl RacingDroneState3D {
    pub fn new(x: f64, y: f64, z: f64, vx: f64, vy: f64, vz: f64) -> Self {
        Self {
            x,
            y,
            z,
            vx,
            vy,
            vz,
        }
    }

    /// Drone at rest at the given position.
    pub fn at(x: f64, y: f64, z: f64) -> Self {
        Self::new(x, y, z, 0.0, 0.0, 0.0)
    }

    pub fn position(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    pub fn velocity(self) -> [f64; 3] {
        [self.vx, self.vy, self.vz]
    }

    pub fn speed(self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }
}

/// Commanded acceleration \[m/s^2\] for the drone model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacingDroneControl3D {
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
}

impl RacingDroneControl3D {
    pub fn new(ax: f64, ay: f64, az: f64) -> Self {
        Self { ax, ay, az }
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn magnitude(self) -> f64 {
        (self.ax * self.ax + self.ay * self.ay + self.az * self.az).sqrt()
    }
}

/// Point-mass drone dynamics with linear drag, gravity, and actuation limits.
///
/// The discrete update is semi-implicit: the commanded acceleration (capped to
/// `accel_limit` in magnitude) and gravity update the velocity, linear drag
/// decays it, the speed is capped to `max_speed`, and the position integrates
/// the new velocity. This stays stable for the step sizes used by the racing
/// MPPI rollouts while modeling drag and saturation effects a pure double
/// integrator ignores.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacingDroneDynamics3D {
    /// Linear drag coefficient \[1/s\].
    pub drag: f64,
    /// Downward gravitational acceleration \[m/s^2\] applied to `vz`.
    pub gravity: f64,
    /// Maximum speed \[m/s\] the drone body can reach.
    pub max_speed: f64,
    /// Maximum commanded acceleration magnitude \[m/s^2\].
    pub accel_limit: f64,
}

impl Default for RacingDroneDynamics3D {
    fn default() -> Self {
        Self {
            drag: 0.4,
            gravity: 0.0,
            max_speed: 5.5,
            accel_limit: 9.0,
        }
    }
}

impl RacingDroneDynamics3D {
    pub fn new(drag: f64, gravity: f64, max_speed: f64, accel_limit: f64) -> RoboticsResult<Self> {
        if !drag.is_finite() || drag < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "drone drag must be finite and non-negative".to_string(),
            ));
        }
        if !gravity.is_finite() || gravity < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "drone gravity must be finite and non-negative".to_string(),
            ));
        }
        if !max_speed.is_finite() || max_speed <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "drone max_speed must be finite and positive".to_string(),
            ));
        }
        if !accel_limit.is_finite() || accel_limit <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "drone accel_limit must be finite and positive".to_string(),
            ));
        }
        Ok(Self {
            drag,
            gravity,
            max_speed,
            accel_limit,
        })
    }

    /// Clamp a control's acceleration magnitude to `accel_limit`.
    pub fn saturate(self, control: RacingDroneControl3D) -> RacingDroneControl3D {
        let magnitude = control.magnitude();
        if magnitude <= self.accel_limit || magnitude == 0.0 {
            return control;
        }
        let scale = self.accel_limit / magnitude;
        RacingDroneControl3D::new(control.ax * scale, control.ay * scale, control.az * scale)
    }

    /// Advance the drone one step under the commanded acceleration.
    pub fn step(
        self,
        state: RacingDroneState3D,
        control: RacingDroneControl3D,
        dt: f64,
    ) -> RacingDroneState3D {
        let control = self.saturate(control);
        let mut vx = state.vx + control.ax * dt;
        let mut vy = state.vy + control.ay * dt;
        let mut vz = state.vz + (control.az - self.gravity) * dt;

        let decay = (1.0 - self.drag * dt).clamp(0.0, 1.0);
        vx *= decay;
        vy *= decay;
        vz *= decay;

        let speed = (vx * vx + vy * vy + vz * vz).sqrt();
        if speed > self.max_speed && speed > 0.0 {
            let scale = self.max_speed / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }

        RacingDroneState3D {
            x: state.x + vx * dt,
            y: state.y + vy * dt,
            z: state.z + vz * dt,
            vx,
            vy,
            vz,
        }
    }
}

/// A rectangular racing gate plane in 3-D.
///
/// The gate is defined by its `center`, a unit `normal` pointing in the race
/// direction, and an in-plane `up` axis. The `right` axis is derived as
/// `normal x up`. A point is inside the aperture when its in-plane offset along
/// `right`/`up` is within `half_width`/`half_height` respectively.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacingGatePlane3D {
    center: [f64; 3],
    normal: [f64; 3],
    up: [f64; 3],
    right: [f64; 3],
    half_width: f64,
    half_height: f64,
}

impl RacingGatePlane3D {
    /// Build a gate from a center, race-direction normal, and up hint.
    ///
    /// `up_hint` need not be perpendicular to the normal or unit length; it is
    /// orthonormalized against the normal. The width axis is `normal x up`.
    pub fn new(
        center: [f64; 3],
        normal: [f64; 3],
        up_hint: [f64; 3],
        half_width: f64,
        half_height: f64,
    ) -> RoboticsResult<Self> {
        if !center.iter().all(|v| v.is_finite()) {
            return Err(RoboticsError::InvalidParameter(
                "racing gate center must be finite".to_string(),
            ));
        }
        let normal = normalize(normal).ok_or_else(|| {
            RoboticsError::InvalidParameter(
                "racing gate normal must be finite and non-zero".to_string(),
            )
        })?;
        // Remove the normal component from the up hint, then normalize.
        let up_projected = sub(up_hint, scale(normal, dot(up_hint, normal)));
        let up = normalize(up_projected).ok_or_else(|| {
            RoboticsError::InvalidParameter(
                "racing gate up hint must not be parallel to the normal".to_string(),
            )
        })?;
        let right = normalize(cross(normal, up)).ok_or_else(|| {
            RoboticsError::InvalidParameter(
                "racing gate axes must be linearly independent".to_string(),
            )
        })?;
        if !half_width.is_finite() || half_width <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "racing gate half_width must be finite and positive".to_string(),
            ));
        }
        if !half_height.is_finite() || half_height <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "racing gate half_height must be finite and positive".to_string(),
            ));
        }
        Ok(Self {
            center,
            normal,
            up,
            right,
            half_width,
            half_height,
        })
    }

    pub fn center(&self) -> [f64; 3] {
        self.center
    }

    pub fn normal(&self) -> [f64; 3] {
        self.normal
    }

    pub fn half_width(&self) -> f64 {
        self.half_width
    }

    pub fn half_height(&self) -> f64 {
        self.half_height
    }

    /// Signed distance to the gate plane; positive is in front (race direction).
    pub fn signed_distance(&self, point: [f64; 3]) -> f64 {
        dot(sub(point, self.center), self.normal)
    }

    /// In-plane offsets `(right, up)` of a point relative to the gate center.
    pub fn plane_offsets(&self, point: [f64; 3]) -> (f64, f64) {
        let delta = sub(point, self.center);
        (dot(delta, self.right), dot(delta, self.up))
    }

    pub fn squared_distance(&self, point: [f64; 3]) -> f64 {
        let delta = sub(point, self.center);
        dot(delta, delta)
    }

    /// Whether a point projects inside the rectangular aperture (with tolerance).
    pub fn within_aperture(&self, point: [f64; 3], tolerance: f64) -> bool {
        let (right_off, up_off) = self.plane_offsets(point);
        right_off.abs() <= self.half_width + tolerance
            && up_off.abs() <= self.half_height + tolerance
    }

    /// Whether the segment `from -> to` passes through the gate aperture.
    ///
    /// The segment must cross the plane from behind (negative side) to in front
    /// (non-negative side), and the plane-crossing point must lie inside the
    /// aperture.
    pub fn crosses(&self, from: [f64; 3], to: [f64; 3], tolerance: f64) -> bool {
        let from_signed = self.signed_distance(from);
        let to_signed = self.signed_distance(to);
        if from_signed > tolerance || to_signed < -tolerance {
            return false;
        }
        let denom = to_signed - from_signed;
        let t = if denom.abs() > 1e-9 {
            (-from_signed / denom).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let crossing = [
            from[0] + t * (to[0] - from[0]),
            from[1] + t * (to[1] - from[1]),
            from[2] + t * (to[2] - from[2]),
        ];
        self.within_aperture(crossing, tolerance)
    }

    /// Normalized aperture margin of a crossing point in \[0, 1\]: 1 at the gate
    /// center, 0 at the aperture edge. Used as a clearance metric.
    pub fn aperture_margin(&self, point: [f64; 3]) -> f64 {
        let (right_off, up_off) = self.plane_offsets(point);
        let right_margin = 1.0 - right_off.abs() / self.half_width;
        let up_margin = 1.0 - up_off.abs() / self.half_height;
        right_margin.min(up_margin)
    }
}

/// A sequence of gates forming an open course or a closed racing lap.
///
/// Holds the reference-free gate-progress objective weights, mirroring
/// [`crate::mppi::MppiGateRace2D`] but over 3-D gate planes.
#[derive(Debug, Clone, PartialEq)]
pub struct RacingGateLap3D {
    pub gates: Vec<RacingGatePlane3D>,
    /// When true, after the last gate the race wraps back to the first gate.
    pub closed: bool,
    pub progress_weight: f64,
    pub lateral_weight: f64,
    pub pass_bonus: f64,
    pub miss_penalty: f64,
    pub terminal_gate_weight: f64,
    pub crossing_tolerance: f64,
}

/// Per-step outcome of scoring one rollout transition against the active gate.
#[derive(Debug, Clone, Copy, PartialEq)]
struct GateTransition {
    cost: f64,
    advanced: bool,
}

impl RacingGateLap3D {
    fn with_defaults(gates: Vec<RacingGatePlane3D>, closed: bool) -> RoboticsResult<Self> {
        if gates.is_empty() {
            return Err(RoboticsError::InvalidParameter(
                "racing lap must contain at least one gate".to_string(),
            ));
        }
        Ok(Self {
            gates,
            closed,
            progress_weight: 6.0,
            lateral_weight: 0.4,
            pass_bonus: 9.0,
            miss_penalty: 36.0,
            terminal_gate_weight: 1.4,
            crossing_tolerance: 0.04,
        })
    }

    /// Open course: the race ends after the last gate is passed.
    pub fn open(gates: Vec<RacingGatePlane3D>) -> RoboticsResult<Self> {
        Self::with_defaults(gates, false)
    }

    /// Closed lap: after the last gate the race wraps back to the first gate.
    pub fn closed_loop(gates: Vec<RacingGatePlane3D>) -> RoboticsResult<Self> {
        Self::with_defaults(gates, true)
    }

    pub fn gate_count(&self) -> usize {
        self.gates.len()
    }

    /// Gate the race is chasing given how many gates have already been passed.
    ///
    /// For a closed lap this wraps modulo the gate count; for an open course it
    /// saturates at the last gate.
    pub fn gate_for_pass_count(&self, passed: usize) -> &RacingGatePlane3D {
        let index = if self.closed {
            passed % self.gates.len()
        } else {
            passed.min(self.gates.len() - 1)
        };
        &self.gates[index]
    }

    /// Length of the polyline through the gate centers; closed laps add the
    /// return leg from the last gate back to the first.
    pub fn centerline_length(&self) -> f64 {
        let mut length = 0.0;
        for pair in self.gates.windows(2) {
            length += distance(pair[0].center, pair[1].center);
        }
        if self.closed && self.gates.len() > 1 {
            length += distance(
                self.gates[self.gates.len() - 1].center,
                self.gates[0].center,
            );
        }
        length
    }

    fn transition_cost(&self, from: [f64; 3], to: [f64; 3], passed: usize) -> GateTransition {
        let gate = self.gate_for_pass_count(passed);
        let before_sq = gate.squared_distance(from);
        let after_sq = gate.squared_distance(to);
        let (right_off, up_off) = gate.plane_offsets(to);
        let after_distance = after_sq.sqrt();
        let lateral_proximity = 1.0 / (1.0 + after_distance);
        let lateral = right_off * right_off + up_off * up_off;
        let mut cost = self.progress_weight * (after_sq - before_sq)
            + self.lateral_weight * lateral_proximity * lateral;
        let mut advanced = false;

        if gate.crosses(from, to, self.crossing_tolerance) {
            cost -= self.pass_bonus;
            advanced = true;
        } else if gate.signed_distance(to) >= 0.0 && !gate.within_aperture(to, 0.0) {
            let right_miss = (right_off.abs() - gate.half_width).max(0.0);
            let up_miss = (up_off.abs() - gate.half_height).max(0.0);
            cost += self.miss_penalty * (right_miss * right_miss + up_miss * up_miss);
        }

        GateTransition { cost, advanced }
    }

    fn terminal_cost(&self, point: [f64; 3], passed: usize) -> f64 {
        let gate = self.gate_for_pass_count(passed);
        let remaining = if self.closed {
            // Encourage progress within the current lap.
            (self.gates.len() - passed % self.gates.len()) as f64
        } else {
            (self.gates.len() - passed.min(self.gates.len())) as f64
        };
        self.terminal_gate_weight * gate.squared_distance(point) + self.pass_bonus * remaining
    }

    /// Reference-free gate-progress cost of a position rollout starting from a
    /// pass count, returning `(cost, gates_passed_in_rollout)`.
    pub fn score_positions(&self, positions: &[[f64; 3]], start_passed: usize) -> (f64, usize) {
        if positions.len() < 2 {
            return (0.0, 0);
        }
        let mut passed = start_passed;
        let mut passed_here = 0;
        let mut cost = 0.0;
        for pair in positions.windows(2) {
            let transition = self.transition_cost(pair[0], pair[1], passed);
            cost += transition.cost;
            if transition.advanced {
                passed += 1;
                passed_here += 1;
            }
        }
        cost += self.terminal_cost(positions[positions.len() - 1], passed);
        (cost, passed_here)
    }
}

/// Lap-progress metrics from a closed-loop racing rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct RacingLapReport3D {
    /// Control steps executed.
    pub steps: usize,
    /// Total gate passes across the whole rollout (counts repeated laps).
    pub gates_passed: usize,
    /// Whole laps completed.
    pub laps_completed: usize,
    /// Fraction of the current (partial) lap completed, in \[0, 1).
    pub lap_fraction: f64,
    /// Mean drone speed \[m/s\] over the rollout.
    pub mean_speed: f64,
    /// Peak drone speed \[m/s\] over the rollout.
    pub max_speed: f64,
    /// Executed path length \[m\].
    pub path_length: f64,
    /// Distance \[m\] to the currently active gate at the end of the rollout.
    pub gate_distance: f64,
    /// Smallest normalized aperture margin seen at any gate crossing (1 = dead
    /// center, 0 = edge); `INFINITY` if no gate was crossed.
    pub min_aperture_margin: f64,
    /// Mean commanded acceleration magnitude \[m/s^2\].
    pub mean_control_effort: f64,
    /// Time \[s\] of the first completed lap, or `None` if no lap finished.
    pub first_lap_time: Option<f64>,
    /// Executed drone positions, including the start.
    pub path: Vec<[f64; 3]>,
}

/// Configuration for the deterministic 3-D racing MPPI controller.
#[derive(Debug, Clone, PartialEq)]
pub struct RacingMppi3DConfig {
    pub horizon: usize,
    pub samples: usize,
    pub dt: f64,
    pub noise_sigma: f64,
    pub velocity_weight: f64,
    pub control_weight: f64,
    pub lambda: f64,
    pub seed: u64,
}

impl Default for RacingMppi3DConfig {
    fn default() -> Self {
        Self {
            horizon: 20,
            samples: 480,
            dt: 0.1,
            noise_sigma: 3.2,
            velocity_weight: 0.01,
            control_weight: 0.01,
            lambda: 1.0,
            seed: 7,
        }
    }
}

fn validate_config(config: &RacingMppi3DConfig) -> RoboticsResult<()> {
    if config.horizon == 0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI horizon must be positive".to_string(),
        ));
    }
    if config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI samples must be positive".to_string(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI dt must be finite and positive".to_string(),
        ));
    }
    if !config.noise_sigma.is_finite() || config.noise_sigma <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI noise_sigma must be finite and positive".to_string(),
        ));
    }
    if !config.lambda.is_finite() || config.lambda <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI lambda must be finite and positive".to_string(),
        ));
    }
    if config.velocity_weight < 0.0 || config.control_weight < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "racing MPPI weights must be non-negative".to_string(),
        ));
    }
    Ok(())
}

/// Deterministic, seeded MPPI controller for 3-D gate racing.
///
/// Each `plan` call samples Gaussian acceleration perturbations around the
/// warm-started nominal control sequence, rolls them out through the drone
/// dynamics, scores reference-free gate progress, and returns the softmin-
/// weighted first control. The controller owns no obstacles — the gate
/// apertures themselves are the constraints.
#[derive(Debug, Clone)]
pub struct RacingMppi3DController {
    config: RacingMppi3DConfig,
    dynamics: RacingDroneDynamics3D,
    nominal: Vec<RacingDroneControl3D>,
    rng: StdRng,
}

/// Result of one `plan` call.
#[derive(Debug, Clone, PartialEq)]
pub struct RacingMppi3DPlan {
    pub control: RacingDroneControl3D,
    /// Lowest rollout cost among the samples (diagnostic).
    pub best_cost: f64,
    /// Normalized effective sample size in \[0, 1\] (diagnostic).
    pub normalized_effective_sample_size: f64,
}

impl RacingMppi3DController {
    pub fn new(
        config: RacingMppi3DConfig,
        dynamics: RacingDroneDynamics3D,
    ) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nominal = vec![RacingDroneControl3D::zero(); config.horizon];
        let rng = StdRng::seed_from_u64(config.seed);
        Ok(Self {
            config,
            dynamics,
            nominal,
            rng,
        })
    }

    pub fn config(&self) -> &RacingMppi3DConfig {
        &self.config
    }

    fn rollout_positions(
        &self,
        start: RacingDroneState3D,
        controls: &[RacingDroneControl3D],
    ) -> (Vec<[f64; 3]>, f64) {
        let mut state = start;
        let mut positions = Vec::with_capacity(controls.len() + 1);
        positions.push(state.position());
        let mut regularizer = 0.0;
        for &control in controls {
            state = self.dynamics.step(state, control, self.config.dt);
            positions.push(state.position());
            regularizer += self.config.velocity_weight * state.speed() * state.speed()
                + self.config.control_weight * control.magnitude() * control.magnitude();
        }
        (positions, regularizer)
    }

    /// Plan the next control given the current state, lap, and pass count.
    pub fn plan(
        &mut self,
        start: RacingDroneState3D,
        lap: &RacingGateLap3D,
        passed: usize,
    ) -> RoboticsResult<RacingMppi3DPlan> {
        let normal = Normal::new(0.0, self.config.noise_sigma).map_err(|_| {
            RoboticsError::InvalidParameter("invalid racing MPPI noise distribution".to_string())
        })?;

        let mut costs = Vec::with_capacity(self.config.samples);
        let mut perturbed_sequences = Vec::with_capacity(self.config.samples);
        let mut best_cost = f64::INFINITY;

        for _ in 0..self.config.samples {
            let mut controls = Vec::with_capacity(self.config.horizon);
            for &base in &self.nominal {
                let noisy = RacingDroneControl3D::new(
                    base.ax + normal.sample(&mut self.rng),
                    base.ay + normal.sample(&mut self.rng),
                    base.az + normal.sample(&mut self.rng),
                );
                controls.push(self.dynamics.saturate(noisy));
            }
            let (positions, regularizer) = self.rollout_positions(start, &controls);
            let (gate_cost, _) = lap.score_positions(&positions, passed);
            let cost = gate_cost + regularizer;
            best_cost = best_cost.min(cost);
            costs.push(cost);
            perturbed_sequences.push(controls);
        }

        let min_cost = costs.iter().copied().fold(f64::INFINITY, f64::min);
        let mut weights = Vec::with_capacity(costs.len());
        let mut weight_sum = 0.0;
        for &cost in &costs {
            let weight = (-(cost - min_cost) / self.config.lambda).exp();
            weight_sum += weight;
            weights.push(weight);
        }
        if weight_sum <= 0.0 || !weight_sum.is_finite() {
            // Degenerate weighting: fall back to the lowest-cost sample.
            let best_index = costs
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(index, _)| index)
                .unwrap_or(0);
            let control = perturbed_sequences[best_index][0];
            return Ok(RacingMppi3DPlan {
                control,
                best_cost,
                normalized_effective_sample_size: 0.0,
            });
        }

        let mut updated = vec![RacingDroneControl3D::zero(); self.config.horizon];
        let mut sum_sq = 0.0;
        for (weight, controls) in weights.iter().zip(&perturbed_sequences) {
            let normalized = weight / weight_sum;
            sum_sq += normalized * normalized;
            for (acc, control) in updated.iter_mut().zip(controls) {
                acc.ax += normalized * control.ax;
                acc.ay += normalized * control.ay;
                acc.az += normalized * control.az;
            }
        }
        for control in &mut updated {
            *control = self.dynamics.saturate(*control);
        }

        let first_control = updated[0];
        // Warm-start: shift the nominal sequence forward by one step.
        self.nominal.clear();
        self.nominal.extend_from_slice(&updated[1..]);
        self.nominal.push(*updated.last().unwrap());

        let normalized_effective_sample_size = if sum_sq > 0.0 {
            1.0 / (sum_sq * self.config.samples as f64)
        } else {
            0.0
        };

        Ok(RacingMppi3DPlan {
            control: first_control,
            best_cost,
            normalized_effective_sample_size,
        })
    }
}

/// Drive the racing MPPI controller around a lap and report lap-progress
/// metrics.
///
/// The rollout runs for at most `max_steps` control steps and stops early once
/// `target_laps` whole laps have been completed. Everything is deterministic
/// for a fixed `config.seed`.
pub fn simulate_lap_race(
    config: RacingMppi3DConfig,
    dynamics: RacingDroneDynamics3D,
    lap: &RacingGateLap3D,
    start: RacingDroneState3D,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<RacingLapReport3D> {
    let dt = config.dt;
    let gate_count = lap.gate_count();
    let mut controller = RacingMppi3DController::new(config, dynamics)?;

    let mut state = start;
    let mut passed = 0usize;
    let mut path = vec![state.position()];
    let mut sum_speed = 0.0;
    let mut max_speed = 0.0_f64;
    let mut path_length = 0.0;
    let mut control_effort = 0.0;
    let mut min_aperture_margin = f64::INFINITY;
    let mut first_lap_time = None;
    let mut executed_steps = 0usize;

    for step in 0..max_steps {
        let plan = controller.plan(state, lap, passed)?;
        let next = dynamics.step(state, plan.control, dt);
        let from = state.position();
        let to = next.position();

        // Resolve any gate crossings produced by this segment (usually one).
        loop {
            let gate = lap.gate_for_pass_count(passed);
            if !gate.crosses(from, to, lap.crossing_tolerance) {
                break;
            }
            let margin = aperture_crossing_margin(gate, from, to);
            min_aperture_margin = min_aperture_margin.min(margin);
            passed += 1;
            if passed == gate_count && first_lap_time.is_none() {
                first_lap_time = Some((step + 1) as f64 * dt);
            }
            // Open courses only have one gate sequence; stop once exhausted.
            if !lap.closed && passed >= gate_count {
                break;
            }
        }

        path_length += distance(from, to);
        control_effort += plan.control.magnitude();
        let speed = next.speed();
        sum_speed += speed;
        max_speed = max_speed.max(speed);
        state = next;
        path.push(to);
        executed_steps += 1;

        if target_laps > 0 && passed >= target_laps * gate_count {
            break;
        }
        if !lap.closed && passed >= gate_count {
            break;
        }
    }

    let laps_completed = passed / gate_count;
    let lap_fraction = (passed % gate_count) as f64 / gate_count as f64;
    let active_gate = lap.gate_for_pass_count(passed);
    let gate_distance = active_gate.squared_distance(state.position()).sqrt();
    let mean_speed = if executed_steps > 0 {
        sum_speed / executed_steps as f64
    } else {
        0.0
    };
    let mean_control_effort = if executed_steps > 0 {
        control_effort / executed_steps as f64
    } else {
        0.0
    };

    Ok(RacingLapReport3D {
        steps: executed_steps,
        gates_passed: passed,
        laps_completed,
        lap_fraction,
        mean_speed,
        max_speed,
        path_length,
        gate_distance,
        min_aperture_margin,
        mean_control_effort,
        first_lap_time,
        path,
    })
}

fn aperture_crossing_margin(gate: &RacingGatePlane3D, from: [f64; 3], to: [f64; 3]) -> f64 {
    let from_signed = gate.signed_distance(from);
    let to_signed = gate.signed_distance(to);
    let denom = to_signed - from_signed;
    let t = if denom.abs() > 1e-9 {
        (-from_signed / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let crossing = [
        from[0] + t * (to[0] - from[0]),
        from[1] + t * (to[1] - from[1]),
        from[2] + t * (to[2] - from[2]),
    ];
    gate.aperture_margin(crossing)
}

// --- Small 3-D vector helpers (kept private to avoid a math dependency). ---

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(a: [f64; 3]) -> Option<[f64; 3]> {
    let norm = dot(a, a).sqrt();
    if !norm.is_finite() || norm <= 0.0 {
        return None;
    }
    Some(scale(a, 1.0 / norm))
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(a, b);
    dot(d, d).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_gate(center: [f64; 3]) -> RacingGatePlane3D {
        RacingGatePlane3D::new(center, [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.5, 0.5).unwrap()
    }

    #[test]
    fn gate_orthonormalizes_axes() {
        let gate =
            RacingGatePlane3D::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.3, 0.0, 1.0], 0.5, 0.4)
                .unwrap();
        // up hint had a normal component; it must be removed.
        assert!(dot(gate.normal(), [1.0, 0.0, 0.0]) > 0.999);
        let (right, up) = (
            gate.plane_offsets([0.0, 1.0, 0.0]),
            gate.signed_distance([0.0, 1.0, 0.0]),
        );
        assert!(up.abs() < 1e-9); // point in the plane has zero signed distance
        assert!(right.0.abs() > 0.0 || right.1.abs() > 0.0);
    }

    #[test]
    fn gate_rejects_parallel_up_hint() {
        let result =
            RacingGatePlane3D::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0], 0.5, 0.5);
        assert!(result.is_err());
    }

    #[test]
    fn gate_detects_central_crossing_and_rejects_miss() {
        let gate = axis_gate([1.0, 0.0, 0.0]);
        assert!(gate.crosses([0.5, 0.0, 0.0], [1.5, 0.0, 0.0], 0.04));
        // Crosses the plane but far outside the aperture in y.
        assert!(!gate.crosses([0.5, 3.0, 0.0], [1.5, 3.0, 0.0], 0.04));
        // Moving away from the gate (wrong direction) is not a crossing.
        assert!(!gate.crosses([1.5, 0.0, 0.0], [0.5, 0.0, 0.0], 0.04));
    }

    #[test]
    fn aperture_margin_is_one_at_center() {
        let gate = axis_gate([1.0, 0.0, 0.0]);
        assert!((gate.aperture_margin([1.0, 0.0, 0.0]) - 1.0).abs() < 1e-9);
        assert!(gate.aperture_margin([1.0, 0.5, 0.0]).abs() < 1e-9);
    }

    #[test]
    fn drag_decays_velocity_without_control() {
        let dynamics = RacingDroneDynamics3D::default();
        let state = RacingDroneState3D::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let next = dynamics.step(state, RacingDroneControl3D::zero(), 0.1);
        assert!(next.vx < state.vx);
        assert!(next.vx > 0.0);
    }

    #[test]
    fn dynamics_caps_speed() {
        let dynamics = RacingDroneDynamics3D::new(0.0, 0.0, 3.0, 100.0).unwrap();
        let state = RacingDroneState3D::at(0.0, 0.0, 0.0);
        let next = dynamics.step(state, RacingDroneControl3D::new(100.0, 0.0, 0.0), 0.1);
        assert!(next.speed() <= 3.0 + 1e-9);
    }

    #[test]
    fn closed_lap_centerline_includes_return_leg() {
        let gates = vec![
            axis_gate([0.0, 0.0, 0.0]),
            axis_gate([1.0, 0.0, 0.0]),
            axis_gate([1.0, 1.0, 0.0]),
        ];
        let open = RacingGateLap3D::open(gates.clone()).unwrap();
        let closed = RacingGateLap3D::closed_loop(gates).unwrap();
        assert!(closed.centerline_length() > open.centerline_length());
    }

    #[test]
    fn closed_lap_wraps_active_gate() {
        let gates = vec![axis_gate([0.0, 0.0, 0.0]), axis_gate([1.0, 0.0, 0.0])];
        let lap = RacingGateLap3D::closed_loop(gates).unwrap();
        assert_eq!(lap.gate_for_pass_count(0).center(), [0.0, 0.0, 0.0]);
        assert_eq!(lap.gate_for_pass_count(2).center(), [0.0, 0.0, 0.0]);
        assert_eq!(lap.gate_for_pass_count(3).center(), [1.0, 0.0, 0.0]);
    }

    fn straight_lap() -> RacingGateLap3D {
        let gates = vec![
            axis_gate([1.0, 0.0, 0.0]),
            axis_gate([2.0, 0.0, 0.3]),
            axis_gate([3.0, 0.0, 0.6]),
        ];
        RacingGateLap3D::open(gates).unwrap()
    }

    #[test]
    fn controller_makes_gate_progress() {
        let lap = straight_lap();
        let report = simulate_lap_race(
            RacingMppi3DConfig::default(),
            RacingDroneDynamics3D::default(),
            &lap,
            RacingDroneState3D::at(0.0, 0.0, 0.0),
            60,
            1,
        )
        .unwrap();
        assert!(
            report.gates_passed >= 1,
            "expected at least one gate, got {}",
            report.gates_passed
        );
        assert!(report.path_length > 0.0);
    }

    #[test]
    fn simulation_is_deterministic() {
        let lap = straight_lap();
        let run = || {
            simulate_lap_race(
                RacingMppi3DConfig::default(),
                RacingDroneDynamics3D::default(),
                &lap,
                RacingDroneState3D::at(0.0, 0.0, 0.0),
                60,
                1,
            )
            .unwrap()
        };
        let first = run();
        let second = run();
        assert_eq!(first.path, second.path);
        assert_eq!(first.gates_passed, second.gates_passed);
    }

    #[test]
    fn invalid_config_is_rejected() {
        let config = RacingMppi3DConfig {
            horizon: 0,
            ..RacingMppi3DConfig::default()
        };
        assert!(RacingMppi3DController::new(config, RacingDroneDynamics3D::default()).is_err());
    }
}
