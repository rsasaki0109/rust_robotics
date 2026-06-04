//! Reference-free racing MPPI with a full quadrotor attitude model.
//!
//! This deepens [`crate::racing_mppi_3d`] from a point-mass drone to a quadrotor
//! whose orientation matters. The control input is the standard low-level agile-
//! racing abstraction used by differential-flatness controllers: a mass-
//! normalized collective thrust along the body z-axis plus three body rates.
//!
//! - [`QuadrotorState`] carries position, velocity, and a unit-quaternion
//!   attitude.
//! - [`QuadrotorParams`] integrates the rigid-body translational dynamics
//!   (`a = thrust * body_z - gravity - drag * v`) together with quaternion
//!   attitude kinematics driven by the commanded body rates.
//! - [`QuadrotorMppiController`] samples thrust/body-rate perturbations around a
//!   hover nominal, rolls them through the attitude dynamics, and scores the
//!   resulting positions with the reference-free gate-progress objective from
//!   [`crate::racing_mppi_3d`]. Because horizontal motion can only come from
//!   tilting the thrust vector, the position objective drives the *attitude*:
//!   the drone learns to pitch and roll toward the next gate.
//! - [`simulate_quadrotor_race`] flies a deterministic, seeded controller around
//!   a lap and reports [`QuadrotorLapReport`], which adds attitude metrics (tilt
//!   angle and body-rate effort) to the lap-progress metrics.
//!
//! The gate geometry ([`RacingGatePlane3D`], [`RacingGateLap3D`]) is reused
//! directly from [`crate::racing_mppi_3d`].

use crate::racing_mppi_3d::{RacingGateLap3D, RacingGatePlane3D};
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// State of the quadrotor: position \[m\], velocity \[m/s\], and a unit
/// quaternion attitude `[w, x, y, z]` (body-to-world rotation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadrotorState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4],
}

impl QuadrotorState {
    /// Level hover state at the given position (identity attitude, zero
    /// velocity).
    pub fn at(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: [x, y, z],
            velocity: [0.0, 0.0, 0.0],
            attitude: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn speed(self) -> f64 {
        norm(self.velocity)
    }

    /// The body z-axis (thrust direction) expressed in the world frame.
    pub fn thrust_axis(self) -> [f64; 3] {
        rotate(self.attitude, [0.0, 0.0, 1.0])
    }

    /// Tilt angle \[rad\] between the thrust axis and world up, in `[0, pi]`.
    pub fn tilt_angle(self) -> f64 {
        self.thrust_axis()[2].clamp(-1.0, 1.0).acos()
    }
}

/// Commanded collective thrust and body rates for the quadrotor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadrotorControl {
    /// Mass-normalized collective thrust \[m/s^2\] along the body z-axis.
    pub thrust: f64,
    /// Body angular rates `[wx, wy, wz]` \[rad/s\].
    pub body_rates: [f64; 3],
}

impl QuadrotorControl {
    pub fn new(thrust: f64, body_rates: [f64; 3]) -> Self {
        Self { thrust, body_rates }
    }

    /// Hover command: thrust balancing gravity, zero body rates.
    pub fn hover(gravity: f64) -> Self {
        Self {
            thrust: gravity,
            body_rates: [0.0, 0.0, 0.0],
        }
    }

    /// Body-rate magnitude \[rad/s\].
    pub fn body_rate_magnitude(self) -> f64 {
        norm(self.body_rates)
    }
}

/// Quadrotor rigid-body parameters and actuation limits.
///
/// Translational dynamics are semi-implicit: the collective thrust along the
/// current body z-axis, minus gravity and linear drag, updates the velocity,
/// the speed is capped, and the position integrates the new velocity. Attitude
/// is integrated from the commanded body rates and renormalized each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadrotorParams {
    /// Downward gravitational acceleration \[m/s^2\].
    pub gravity: f64,
    /// Linear aerodynamic drag coefficient \[1/s\].
    pub drag: f64,
    /// Minimum mass-normalized collective thrust \[m/s^2\].
    pub min_thrust: f64,
    /// Maximum mass-normalized collective thrust \[m/s^2\].
    pub max_thrust: f64,
    /// Maximum body-rate magnitude \[rad/s\].
    pub max_body_rate: f64,
    /// Maximum speed \[m/s\] the body can reach.
    pub max_speed: f64,
}

impl Default for QuadrotorParams {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            drag: 0.3,
            min_thrust: 0.0,
            max_thrust: 24.0,
            max_body_rate: 6.0,
            max_speed: 7.0,
        }
    }
}

impl QuadrotorParams {
    pub fn new(
        gravity: f64,
        drag: f64,
        min_thrust: f64,
        max_thrust: f64,
        max_body_rate: f64,
        max_speed: f64,
    ) -> RoboticsResult<Self> {
        if !gravity.is_finite() || gravity < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor gravity must be finite and non-negative".to_string(),
            ));
        }
        if !drag.is_finite() || drag < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor drag must be finite and non-negative".to_string(),
            ));
        }
        if !min_thrust.is_finite() || min_thrust < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor min_thrust must be finite and non-negative".to_string(),
            ));
        }
        if !max_thrust.is_finite() || max_thrust <= min_thrust {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor max_thrust must be finite and exceed min_thrust".to_string(),
            ));
        }
        if !max_body_rate.is_finite() || max_body_rate <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor max_body_rate must be finite and positive".to_string(),
            ));
        }
        if !max_speed.is_finite() || max_speed <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "quadrotor max_speed must be finite and positive".to_string(),
            ));
        }
        Ok(Self {
            gravity,
            drag,
            min_thrust,
            max_thrust,
            max_body_rate,
            max_speed,
        })
    }

    /// Clamp a command to the thrust and body-rate limits.
    pub fn saturate(self, control: QuadrotorControl) -> QuadrotorControl {
        let thrust = control.thrust.clamp(self.min_thrust, self.max_thrust);
        let mut rates = control.body_rates;
        let magnitude = norm(rates);
        if magnitude > self.max_body_rate && magnitude > 0.0 {
            let scale = self.max_body_rate / magnitude;
            rates = [rates[0] * scale, rates[1] * scale, rates[2] * scale];
        }
        QuadrotorControl::new(thrust, rates)
    }

    /// Advance the quadrotor one step under the commanded thrust and body rates.
    pub fn step(self, state: QuadrotorState, control: QuadrotorControl, dt: f64) -> QuadrotorState {
        let control = self.saturate(control);

        // Attitude kinematics: q_dot = 0.5 * q (x) [0, wx, wy, wz].
        let attitude = normalize_quat(integrate_attitude(state.attitude, control.body_rates, dt));

        // Translational dynamics in the world frame.
        let thrust_axis = rotate(attitude, [0.0, 0.0, 1.0]);
        let accel = [
            control.thrust * thrust_axis[0] - self.drag * state.velocity[0],
            control.thrust * thrust_axis[1] - self.drag * state.velocity[1],
            control.thrust * thrust_axis[2] - self.gravity - self.drag * state.velocity[2],
        ];
        let mut velocity = [
            state.velocity[0] + accel[0] * dt,
            state.velocity[1] + accel[1] * dt,
            state.velocity[2] + accel[2] * dt,
        ];
        let speed = norm(velocity);
        if speed > self.max_speed && speed > 0.0 {
            let scale = self.max_speed / speed;
            velocity = [
                velocity[0] * scale,
                velocity[1] * scale,
                velocity[2] * scale,
            ];
        }
        let position = [
            state.position[0] + velocity[0] * dt,
            state.position[1] + velocity[1] * dt,
            state.position[2] + velocity[2] * dt,
        ];

        QuadrotorState {
            position,
            velocity,
            attitude,
        }
    }
}

/// Lap-progress and attitude metrics from a quadrotor racing rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct QuadrotorLapReport {
    /// Control steps executed.
    pub steps: usize,
    /// Total gate passes across the whole rollout (counts repeated laps).
    pub gates_passed: usize,
    /// Whole laps completed.
    pub laps_completed: usize,
    /// Fraction of the current (partial) lap completed, in \[0, 1).
    pub lap_fraction: f64,
    /// Mean speed \[m/s\] over the rollout.
    pub mean_speed: f64,
    /// Peak speed \[m/s\] over the rollout.
    pub max_speed: f64,
    /// Executed path length \[m\].
    pub path_length: f64,
    /// Distance \[m\] to the active gate at the end of the rollout.
    pub gate_distance: f64,
    /// Smallest normalized aperture margin at any gate crossing (1 = center,
    /// 0 = edge); `INFINITY` if no gate was crossed.
    pub min_aperture_margin: f64,
    /// Mean tilt angle \[rad\] of the thrust axis from world up.
    pub mean_tilt: f64,
    /// Peak tilt angle \[rad\] of the thrust axis from world up.
    pub max_tilt: f64,
    /// Mean commanded body-rate magnitude \[rad/s\].
    pub mean_body_rate: f64,
    /// Time \[s\] of the first completed lap, or `None`.
    pub first_lap_time: Option<f64>,
    /// Executed positions, including the start.
    pub path: Vec<[f64; 3]>,
}

/// Configuration for the deterministic quadrotor racing MPPI controller.
#[derive(Debug, Clone, PartialEq)]
pub struct QuadrotorMppiConfig {
    pub horizon: usize,
    pub samples: usize,
    pub dt: f64,
    /// Std-dev of thrust perturbations \[m/s^2\].
    pub thrust_sigma: f64,
    /// Std-dev of body-rate perturbations \[rad/s\].
    pub rate_sigma: f64,
    /// Weight on the squared speed regularizer.
    pub velocity_weight: f64,
    /// Weight on the squared body-rate regularizer.
    pub rate_weight: f64,
    /// Weight penalizing tilt (keeps the drone from flipping under noise).
    pub level_weight: f64,
    pub lambda: f64,
    pub seed: u64,
}

impl Default for QuadrotorMppiConfig {
    fn default() -> Self {
        Self {
            horizon: 28,
            samples: 900,
            dt: 0.05,
            thrust_sigma: 5.0,
            rate_sigma: 5.0,
            velocity_weight: 0.01,
            rate_weight: 0.01,
            level_weight: 0.6,
            lambda: 1.0,
            seed: 7,
        }
    }
}

fn validate_config(config: &QuadrotorMppiConfig) -> RoboticsResult<()> {
    if config.horizon == 0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI horizon must be positive".to_string(),
        ));
    }
    if config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI samples must be positive".to_string(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI dt must be finite and positive".to_string(),
        ));
    }
    if !config.thrust_sigma.is_finite() || config.thrust_sigma <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI thrust_sigma must be finite and positive".to_string(),
        ));
    }
    if !config.rate_sigma.is_finite() || config.rate_sigma <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI rate_sigma must be finite and positive".to_string(),
        ));
    }
    if !config.lambda.is_finite() || config.lambda <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI lambda must be finite and positive".to_string(),
        ));
    }
    if config.velocity_weight < 0.0 || config.rate_weight < 0.0 || config.level_weight < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "quadrotor MPPI weights must be non-negative".to_string(),
        ));
    }
    Ok(())
}

/// Result of one `plan` call.
#[derive(Debug, Clone, PartialEq)]
pub struct QuadrotorMppiPlan {
    pub control: QuadrotorControl,
    /// Lowest rollout cost among the samples (diagnostic).
    pub best_cost: f64,
    /// Normalized effective sample size in \[0, 1\] (diagnostic).
    pub normalized_effective_sample_size: f64,
}

/// Deterministic, seeded MPPI controller for quadrotor gate racing.
#[derive(Debug, Clone)]
pub struct QuadrotorMppiController {
    config: QuadrotorMppiConfig,
    params: QuadrotorParams,
    nominal: Vec<QuadrotorControl>,
    rng: StdRng,
}

impl QuadrotorMppiController {
    pub fn new(config: QuadrotorMppiConfig, params: QuadrotorParams) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nominal = vec![QuadrotorControl::hover(params.gravity); config.horizon];
        let rng = StdRng::seed_from_u64(config.seed);
        Ok(Self {
            config,
            params,
            nominal,
            rng,
        })
    }

    pub fn config(&self) -> &QuadrotorMppiConfig {
        &self.config
    }

    fn rollout(
        &self,
        start: QuadrotorState,
        controls: &[QuadrotorControl],
    ) -> (Vec<[f64; 3]>, f64) {
        let mut state = start;
        let mut positions = Vec::with_capacity(controls.len() + 1);
        positions.push(state.position);
        let mut regularizer = 0.0;
        for &control in controls {
            state = self.params.step(state, control, self.config.dt);
            let speed = state.speed();
            let rate = control.body_rate_magnitude();
            // 1 - cos(tilt) is 0 when level and grows as the drone tips over.
            let tilt_cost = 1.0 - state.thrust_axis()[2];
            regularizer += self.config.velocity_weight * speed * speed
                + self.config.rate_weight * rate * rate
                + self.config.level_weight * tilt_cost;
            positions.push(state.position);
        }
        (positions, regularizer)
    }

    /// Plan the next control given the current state, lap, and pass count.
    pub fn plan(
        &mut self,
        start: QuadrotorState,
        lap: &RacingGateLap3D,
        passed: usize,
    ) -> RoboticsResult<QuadrotorMppiPlan> {
        let thrust_noise = Normal::new(0.0, self.config.thrust_sigma).map_err(|_| {
            RoboticsError::InvalidParameter("invalid quadrotor thrust distribution".to_string())
        })?;
        let rate_noise = Normal::new(0.0, self.config.rate_sigma).map_err(|_| {
            RoboticsError::InvalidParameter("invalid quadrotor rate distribution".to_string())
        })?;

        let mut costs = Vec::with_capacity(self.config.samples);
        let mut sequences = Vec::with_capacity(self.config.samples);
        let mut best_cost = f64::INFINITY;

        for _ in 0..self.config.samples {
            let mut controls = Vec::with_capacity(self.config.horizon);
            for &base in &self.nominal {
                let noisy = QuadrotorControl::new(
                    base.thrust + thrust_noise.sample(&mut self.rng),
                    [
                        base.body_rates[0] + rate_noise.sample(&mut self.rng),
                        base.body_rates[1] + rate_noise.sample(&mut self.rng),
                        base.body_rates[2] + rate_noise.sample(&mut self.rng),
                    ],
                );
                controls.push(self.params.saturate(noisy));
            }
            let (positions, regularizer) = self.rollout(start, &controls);
            let (gate_cost, _) = lap.score_positions(&positions, passed);
            let cost = gate_cost + regularizer;
            best_cost = best_cost.min(cost);
            costs.push(cost);
            sequences.push(controls);
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
            let best_index = costs
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(index, _)| index)
                .unwrap_or(0);
            let control = sequences[best_index][0];
            return Ok(QuadrotorMppiPlan {
                control,
                best_cost,
                normalized_effective_sample_size: 0.0,
            });
        }

        let mut updated = vec![QuadrotorControl::new(0.0, [0.0, 0.0, 0.0]); self.config.horizon];
        let mut sum_sq = 0.0;
        for (weight, controls) in weights.iter().zip(&sequences) {
            let normalized = weight / weight_sum;
            sum_sq += normalized * normalized;
            for (acc, control) in updated.iter_mut().zip(controls) {
                acc.thrust += normalized * control.thrust;
                acc.body_rates[0] += normalized * control.body_rates[0];
                acc.body_rates[1] += normalized * control.body_rates[1];
                acc.body_rates[2] += normalized * control.body_rates[2];
            }
        }
        for control in &mut updated {
            *control = self.params.saturate(*control);
        }

        let first_control = updated[0];
        // Warm-start: shift the nominal forward, repeat the last command.
        self.nominal.clear();
        self.nominal.extend_from_slice(&updated[1..]);
        self.nominal.push(*updated.last().unwrap());

        let normalized_effective_sample_size = if sum_sq > 0.0 {
            1.0 / (sum_sq * self.config.samples as f64)
        } else {
            0.0
        };

        Ok(QuadrotorMppiPlan {
            control: first_control,
            best_cost,
            normalized_effective_sample_size,
        })
    }
}

/// Drive the quadrotor racing MPPI controller around a lap and report
/// lap-progress and attitude metrics.
///
/// The rollout runs for at most `max_steps` control steps and stops early once
/// `target_laps` whole laps have been completed. Everything is deterministic for
/// a fixed `config.seed`.
pub fn simulate_quadrotor_race(
    config: QuadrotorMppiConfig,
    params: QuadrotorParams,
    lap: &RacingGateLap3D,
    start: QuadrotorState,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<QuadrotorLapReport> {
    let dt = config.dt;
    let gate_count = lap.gate_count();
    let mut controller = QuadrotorMppiController::new(config, params)?;

    let mut state = start;
    let mut passed = 0usize;
    let mut path = vec![state.position];
    let mut sum_speed = 0.0;
    let mut max_speed = 0.0_f64;
    let mut sum_tilt = 0.0;
    let mut max_tilt = 0.0_f64;
    let mut sum_body_rate = 0.0;
    let mut path_length = 0.0;
    let mut min_aperture_margin = f64::INFINITY;
    let mut first_lap_time = None;
    let mut executed_steps = 0usize;

    for step in 0..max_steps {
        let plan = controller.plan(state, lap, passed)?;
        let next = params.step(state, plan.control, dt);
        let from = state.position;
        let to = next.position;

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
            if !lap.closed && passed >= gate_count {
                break;
            }
        }

        path_length += distance(from, to);
        let speed = next.speed();
        sum_speed += speed;
        max_speed = max_speed.max(speed);
        let tilt = next.tilt_angle();
        sum_tilt += tilt;
        max_tilt = max_tilt.max(tilt);
        sum_body_rate += plan.control.body_rate_magnitude();
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
    let gate_distance = distance(active_gate.center(), state.position);
    let denom = executed_steps.max(1) as f64;

    Ok(QuadrotorLapReport {
        steps: executed_steps,
        gates_passed: passed,
        laps_completed,
        lap_fraction,
        mean_speed: sum_speed / denom,
        max_speed,
        path_length,
        gate_distance,
        min_aperture_margin,
        mean_tilt: sum_tilt / denom,
        max_tilt,
        mean_body_rate: sum_body_rate / denom,
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

// --- Quaternion and vector helpers (kept private to avoid a math dependency).

/// Rotate a vector by a unit quaternion `[w, x, y, z]` (body to world).
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    // v + 2 * qv x (qv x v + w v), with qv = (x, y, z).
    let qv = [x, y, z];
    let t = scale(cross(qv, v), 2.0);
    let cross_t = cross(qv, t);
    [
        v[0] + w * t[0] + cross_t[0],
        v[1] + w * t[1] + cross_t[1],
        v[2] + w * t[2] + cross_t[2],
    ]
}

/// Integrate attitude one step from body rates: q + 0.5 * (q (x) [0, w]) * dt.
fn integrate_attitude(q: [f64; 4], rates: [f64; 3], dt: f64) -> [f64; 4] {
    let [qw, qx, qy, qz] = q;
    let [wx, wy, wz] = rates;
    // Quaternion product q (x) [0, wx, wy, wz].
    let dw = -(qx * wx + qy * wy + qz * wz);
    let dx = qw * wx + qy * wz - qz * wy;
    let dy = qw * wy + qz * wx - qx * wz;
    let dz = qw * wz + qx * wy - qy * wx;
    let half_dt = 0.5 * dt;
    [
        qw + half_dt * dw,
        qx + half_dt * dx,
        qy + half_dt * dy,
        qz + half_dt * dz,
    ]
}

fn normalize_quat(q: [f64; 4]) -> [f64; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if !n.is_finite() || n <= 0.0 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    norm(d)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gate(center: [f64; 3], normal: [f64; 3]) -> RacingGatePlane3D {
        RacingGatePlane3D::new(center, normal, [0.0, 0.0, 1.0], 0.8, 0.8).unwrap()
    }

    fn straight_lap() -> RacingGateLap3D {
        let gates = vec![
            gate([1.5, 0.0, 0.0], [1.0, 0.0, 0.0]),
            gate([3.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            gate([4.5, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ];
        RacingGateLap3D::open(gates).unwrap()
    }

    #[test]
    fn hover_holds_position() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::at(0.0, 0.0, 1.0);
        for _ in 0..20 {
            state = params.step(state, QuadrotorControl::hover(params.gravity), 0.05);
        }
        // Thrust balances gravity, so it stays put.
        assert!(state.speed() < 1e-6);
        assert!((state.position[2] - 1.0).abs() < 1e-6);
        assert!(state.tilt_angle() < 1e-9);
    }

    #[test]
    fn body_rate_tilts_then_thrust_moves_horizontally() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::at(0.0, 0.0, 0.0);
        // Pitch about the body y-axis to tilt the thrust vector toward +x.
        for _ in 0..6 {
            state = params.step(
                state,
                QuadrotorControl::new(params.gravity, [0.0, 1.0, 0.0]),
                0.05,
            );
        }
        assert!(state.tilt_angle() > 0.1, "tilt {}", state.tilt_angle());
        // Now hold attitude and apply extra thrust; horizontal velocity grows.
        for _ in 0..6 {
            state = params.step(
                state,
                QuadrotorControl::new(params.gravity + 4.0, [0.0, 0.0, 0.0]),
                0.05,
            );
        }
        assert!(state.velocity[0].abs() > 0.1, "vx {}", state.velocity[0]);
    }

    #[test]
    fn attitude_stays_normalized() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::at(0.0, 0.0, 0.0);
        for _ in 0..40 {
            state = params.step(state, QuadrotorControl::new(11.0, [2.0, -1.5, 0.8]), 0.05);
        }
        let q = state.attitude;
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((n - 1.0).abs() < 1e-9, "quat norm {n}");
    }

    #[test]
    fn saturation_clamps_thrust_and_rates() {
        let params = QuadrotorParams::default();
        let saturated = params.saturate(QuadrotorControl::new(1000.0, [100.0, 0.0, 0.0]));
        assert!((saturated.thrust - params.max_thrust).abs() < 1e-9);
        assert!((saturated.body_rate_magnitude() - params.max_body_rate).abs() < 1e-9);
    }

    #[test]
    fn controller_makes_gate_progress() {
        let lap = straight_lap();
        let report = simulate_quadrotor_race(
            QuadrotorMppiConfig::default(),
            QuadrotorParams::default(),
            &lap,
            QuadrotorState::at(0.0, 0.0, 0.0),
            120,
            1,
        )
        .unwrap();
        assert!(
            report.gates_passed >= 1,
            "expected at least one gate, got {}",
            report.gates_passed
        );
        // Flying through gates requires tilting the thrust vector.
        assert!(report.max_tilt > 0.05, "max tilt {}", report.max_tilt);
        assert!(report.path_length > 0.0);
    }

    #[test]
    fn simulation_is_deterministic() {
        let lap = straight_lap();
        let run = || {
            simulate_quadrotor_race(
                QuadrotorMppiConfig::default(),
                QuadrotorParams::default(),
                &lap,
                QuadrotorState::at(0.0, 0.0, 0.0),
                80,
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
        let config = QuadrotorMppiConfig {
            horizon: 0,
            ..QuadrotorMppiConfig::default()
        };
        assert!(QuadrotorMppiController::new(config, QuadrotorParams::default()).is_err());
    }

    #[test]
    fn invalid_params_are_rejected() {
        assert!(QuadrotorParams::new(9.81, 0.3, 5.0, 4.0, 6.0, 7.0).is_err());
    }
}
