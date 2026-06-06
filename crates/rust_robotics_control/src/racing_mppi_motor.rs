//! Reference-free racing MPPI with a motor-level rotor-mixing quadrotor.
//!
//! This deepens [`crate::racing_mppi_quadrotor`] one more level: instead of
//! commanding body rates directly, the control input is the four rotor thrusts.
//! A rotor-mixing map turns them into a collective thrust and body torques, the
//! body angular velocity becomes a *state* driven by those torques (with light
//! aerodynamic rate damping), and every rotor saturates at a maximum thrust.
//!
//! That saturation is the point: a rotor cannot supply maximum collective lift
//! and a large differential torque at the same time, so demanding an aggressive
//! attitude change *steals thrust authority*. A thrust-limited drone therefore
//! saturates its rotors more often and is genuinely less agile — a trade-off the
//! body-rate model (which commands rates for free) cannot express.
//!
//! - [`MotorQuadState`] carries position, velocity, a unit-quaternion attitude,
//!   and the body angular velocity.
//! - [`MotorQuadParams`] integrates the rotor-mixing rotational dynamics
//!   (control-affine, with the inertia folded into roll/pitch/yaw gains and a
//!   rate-damping term) and the rigid-body translational dynamics.
//! - [`MotorMppiController`] samples per-rotor thrust perturbations around a
//!   hover nominal and scores the resulting positions with the reference-free
//!   gate-progress objective from [`crate::racing_mppi_3d`].
//! - [`simulate_motor_race`] reports [`MotorRacingLapReport`], which adds a rotor
//!   saturation fraction to the lap-progress and attitude metrics.

use crate::racing_mppi_3d::{RacingGateLap3D, RacingGatePlane3D};
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// State of the motor-level quadrotor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorQuadState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// Unit quaternion `[w, x, y, z]` (body-to-world).
    pub attitude: [f64; 4],
    /// Body angular velocity `[wx, wy, wz]` \[rad/s\].
    pub body_rates: [f64; 3],
}

impl MotorQuadState {
    /// Level hover state at the given position.
    pub fn at(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: [x, y, z],
            velocity: [0.0, 0.0, 0.0],
            attitude: [1.0, 0.0, 0.0, 0.0],
            body_rates: [0.0, 0.0, 0.0],
        }
    }

    pub fn speed(self) -> f64 {
        norm(self.velocity)
    }

    /// Body z-axis (thrust direction) in the world frame.
    pub fn thrust_axis(self) -> [f64; 3] {
        rotate(self.attitude, [0.0, 0.0, 1.0])
    }

    /// Tilt angle \[rad\] of the thrust axis from world up, in `[0, pi]`.
    pub fn tilt_angle(self) -> f64 {
        self.thrust_axis()[2].clamp(-1.0, 1.0).acos()
    }
}

/// Four rotor thrust commands `[f0, f1, f2, f3]` in mass-normalized accel units.
///
/// Rotors are laid out in an X configuration: `f0` front-right, `f1`
/// front-left, `f2` rear-left, `f3` rear-right (the exact labeling only matters
/// through the fixed mixing signs below).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCommand {
    pub rotors: [f64; 4],
}

impl MotorCommand {
    pub fn new(rotors: [f64; 4]) -> Self {
        Self { rotors }
    }

    /// Even hover command: every rotor carries a quarter of gravity.
    pub fn hover(gravity: f64) -> Self {
        Self {
            rotors: [gravity / 4.0; 4],
        }
    }

    /// Collective (mass-normalized) thrust \[m/s^2\].
    pub fn collective(self) -> f64 {
        self.rotors.iter().sum()
    }
}

/// Motor-level quadrotor parameters and actuation limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorQuadParams {
    /// Downward gravitational acceleration \[m/s^2\].
    pub gravity: f64,
    /// Linear aerodynamic drag coefficient \[1/s\].
    pub drag: f64,
    /// Maximum thrust per rotor \[m/s^2\] (mass-normalized).
    pub max_rotor_thrust: f64,
    /// Roll/pitch angular-acceleration gain per unit rotor-thrust difference.
    pub torque_gain: f64,
    /// Yaw angular-acceleration gain per unit rotor-thrust difference.
    pub yaw_gain: f64,
    /// Aerodynamic body-rate damping \[1/s\].
    pub rate_damping: f64,
    /// Maximum speed \[m/s\].
    pub max_speed: f64,
}

impl Default for MotorQuadParams {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            drag: 0.3,
            max_rotor_thrust: 6.0,
            torque_gain: 9.0,
            yaw_gain: 2.0,
            rate_damping: 1.2,
            max_speed: 7.0,
        }
    }
}

impl MotorQuadParams {
    pub fn new(
        gravity: f64,
        drag: f64,
        max_rotor_thrust: f64,
        torque_gain: f64,
        yaw_gain: f64,
        rate_damping: f64,
        max_speed: f64,
    ) -> RoboticsResult<Self> {
        let finite_nonneg = |v: f64| v.is_finite() && v >= 0.0;
        if !finite_nonneg(gravity) || !finite_nonneg(drag) || !finite_nonneg(rate_damping) {
            return Err(RoboticsError::InvalidParameter(
                "motor quad gravity/drag/rate_damping must be finite and non-negative".to_string(),
            ));
        }
        if !max_rotor_thrust.is_finite() || max_rotor_thrust <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "motor quad max_rotor_thrust must be finite and positive".to_string(),
            ));
        }
        if !torque_gain.is_finite()
            || torque_gain <= 0.0
            || !yaw_gain.is_finite()
            || yaw_gain <= 0.0
        {
            return Err(RoboticsError::InvalidParameter(
                "motor quad torque gains must be finite and positive".to_string(),
            ));
        }
        if !max_speed.is_finite() || max_speed <= 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "motor quad max_speed must be finite and positive".to_string(),
            ));
        }
        // Gravity must be hoverable within the rotor limit.
        if gravity > 4.0 * max_rotor_thrust {
            return Err(RoboticsError::InvalidParameter(
                "motor quad cannot hover: gravity exceeds 4 * max_rotor_thrust".to_string(),
            ));
        }
        Ok(Self {
            gravity,
            drag,
            max_rotor_thrust,
            torque_gain,
            yaw_gain,
            rate_damping,
            max_speed,
        })
    }

    /// Clamp each rotor thrust to `[0, max_rotor_thrust]`.
    pub fn saturate(self, command: MotorCommand) -> MotorCommand {
        let mut rotors = command.rotors;
        for r in &mut rotors {
            *r = r.clamp(0.0, self.max_rotor_thrust);
        }
        MotorCommand::new(rotors)
    }

    /// Whether any rotor of the command is at (or above) its thrust limit.
    pub fn is_saturated(self, command: MotorCommand) -> bool {
        command
            .rotors
            .iter()
            .any(|&r| r >= self.max_rotor_thrust - 1e-9 || r <= 1e-9)
    }

    /// Body torques (as angular accelerations) from the rotor mix.
    fn angular_accel_from_rotors(self, rotors: [f64; 4]) -> [f64; 3] {
        let [f0, f1, f2, f3] = rotors;
        // X-mixer (inertia folded into the gains):
        //   roll  : left rotors (f1,f2) vs right rotors (f0,f3)
        //   pitch : front rotors (f0,f1) vs rear rotors (f2,f3)
        //   yaw   : one diagonal (f0,f2) vs the other (f1,f3)
        let roll = self.torque_gain * ((f1 + f2) - (f0 + f3));
        let pitch = self.torque_gain * ((f0 + f1) - (f2 + f3));
        let yaw = self.yaw_gain * ((f0 + f2) - (f1 + f3));
        [roll, pitch, yaw]
    }

    /// Advance the quadrotor one step under the rotor command.
    pub fn step(self, state: MotorQuadState, command: MotorCommand, dt: f64) -> MotorQuadState {
        let command = self.saturate(command);

        // Rotational dynamics: rate = rate + (torque - damping*rate) * dt.
        let torque = self.angular_accel_from_rotors(command.rotors);
        let mut body_rates = [0.0; 3];
        for i in 0..3 {
            let rate_dot = torque[i] - self.rate_damping * state.body_rates[i];
            body_rates[i] = state.body_rates[i] + rate_dot * dt;
        }

        // Attitude kinematics from the (updated) body rates.
        let attitude = normalize_quat(integrate_attitude(state.attitude, body_rates, dt));

        // Translational dynamics.
        let thrust = command.collective();
        let thrust_axis = rotate(attitude, [0.0, 0.0, 1.0]);
        let accel = [
            thrust * thrust_axis[0] - self.drag * state.velocity[0],
            thrust * thrust_axis[1] - self.drag * state.velocity[1],
            thrust * thrust_axis[2] - self.gravity - self.drag * state.velocity[2],
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

        MotorQuadState {
            position,
            velocity,
            attitude,
            body_rates,
        }
    }
}

/// Lap-progress, attitude, and motor-saturation metrics.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorRacingLapReport {
    pub steps: usize,
    pub gates_passed: usize,
    pub laps_completed: usize,
    pub lap_fraction: f64,
    pub mean_speed: f64,
    pub max_speed: f64,
    pub path_length: f64,
    pub gate_distance: f64,
    pub min_aperture_margin: f64,
    pub mean_tilt: f64,
    pub max_tilt: f64,
    pub max_body_rate: f64,
    /// Fraction of executed steps whose rotor command hit a thrust limit.
    pub saturation_fraction: f64,
    pub first_lap_time: Option<f64>,
    pub path: Vec<[f64; 3]>,
}

/// Configuration for the motor-level racing MPPI controller.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorMppiConfig {
    pub horizon: usize,
    pub samples: usize,
    pub dt: f64,
    /// Std-dev of per-rotor thrust perturbations \[m/s^2\].
    pub rotor_sigma: f64,
    pub velocity_weight: f64,
    pub rotor_weight: f64,
    pub level_weight: f64,
    pub lambda: f64,
    pub seed: u64,
}

impl Default for MotorMppiConfig {
    fn default() -> Self {
        Self {
            horizon: 28,
            samples: 800,
            dt: 0.05,
            rotor_sigma: 2.6,
            velocity_weight: 0.01,
            rotor_weight: 0.004,
            level_weight: 0.6,
            lambda: 1.0,
            seed: 7,
        }
    }
}

fn validate_config(config: &MotorMppiConfig) -> RoboticsResult<()> {
    if config.horizon == 0 || config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "motor MPPI horizon and samples must be positive".to_string(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "motor MPPI dt must be finite and positive".to_string(),
        ));
    }
    if !config.rotor_sigma.is_finite() || config.rotor_sigma <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "motor MPPI rotor_sigma must be finite and positive".to_string(),
        ));
    }
    if !config.lambda.is_finite() || config.lambda <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "motor MPPI lambda must be finite and positive".to_string(),
        ));
    }
    if config.velocity_weight < 0.0 || config.rotor_weight < 0.0 || config.level_weight < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "motor MPPI weights must be non-negative".to_string(),
        ));
    }
    Ok(())
}

/// Result of one `plan` call.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorMppiPlan {
    pub command: MotorCommand,
    pub best_cost: f64,
    pub normalized_effective_sample_size: f64,
}

/// Deterministic, seeded motor-level MPPI controller for gate racing.
#[derive(Debug, Clone)]
pub struct MotorMppiController {
    config: MotorMppiConfig,
    params: MotorQuadParams,
    nominal: Vec<MotorCommand>,
    rng: StdRng,
}

impl MotorMppiController {
    pub fn new(config: MotorMppiConfig, params: MotorQuadParams) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nominal = vec![MotorCommand::hover(params.gravity); config.horizon];
        let rng = StdRng::seed_from_u64(config.seed);
        Ok(Self {
            config,
            params,
            nominal,
            rng,
        })
    }

    pub fn config(&self) -> &MotorMppiConfig {
        &self.config
    }

    fn rollout(&self, start: MotorQuadState, controls: &[MotorCommand]) -> (Vec<[f64; 3]>, f64) {
        let mut state = start;
        let mut positions = Vec::with_capacity(controls.len() + 1);
        positions.push(state.position);
        let mut regularizer = 0.0;
        for &command in controls {
            state = self.params.step(state, command, self.config.dt);
            let speed = state.speed();
            let tilt_cost = 1.0 - state.thrust_axis()[2];
            // Penalize rotor effort away from the even hover thrust.
            let hover = self.params.gravity / 4.0;
            let rotor_effort: f64 = command.rotors.iter().map(|&r| (r - hover).powi(2)).sum();
            regularizer += self.config.velocity_weight * speed * speed
                + self.config.rotor_weight * rotor_effort
                + self.config.level_weight * tilt_cost;
            positions.push(state.position);
        }
        (positions, regularizer)
    }

    /// Plan the next rotor command given state, lap, and pass count.
    pub fn plan(
        &mut self,
        start: MotorQuadState,
        lap: &RacingGateLap3D,
        passed: usize,
    ) -> RoboticsResult<MotorMppiPlan> {
        let noise = Normal::new(0.0, self.config.rotor_sigma).map_err(|_| {
            RoboticsError::InvalidParameter("invalid motor MPPI rotor distribution".to_string())
        })?;

        let mut costs = Vec::with_capacity(self.config.samples);
        let mut sequences = Vec::with_capacity(self.config.samples);
        let mut best_cost = f64::INFINITY;

        for _ in 0..self.config.samples {
            let mut controls = Vec::with_capacity(self.config.horizon);
            for &base in &self.nominal {
                let mut rotors = base.rotors;
                for r in &mut rotors {
                    *r += noise.sample(&mut self.rng);
                }
                controls.push(self.params.saturate(MotorCommand::new(rotors)));
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
            let command = sequences[best_index][0];
            return Ok(MotorMppiPlan {
                command,
                best_cost,
                normalized_effective_sample_size: 0.0,
            });
        }

        let mut updated = vec![MotorCommand::new([0.0; 4]); self.config.horizon];
        let mut sum_sq = 0.0;
        for (weight, controls) in weights.iter().zip(&sequences) {
            let normalized = weight / weight_sum;
            sum_sq += normalized * normalized;
            for (acc, command) in updated.iter_mut().zip(controls) {
                for k in 0..4 {
                    acc.rotors[k] += normalized * command.rotors[k];
                }
            }
        }
        for command in &mut updated {
            *command = self.params.saturate(*command);
        }

        let first_command = updated[0];
        self.nominal.clear();
        self.nominal.extend_from_slice(&updated[1..]);
        self.nominal.push(*updated.last().unwrap());

        let normalized_effective_sample_size = if sum_sq > 0.0 {
            1.0 / (sum_sq * self.config.samples as f64)
        } else {
            0.0
        };

        Ok(MotorMppiPlan {
            command: first_command,
            best_cost,
            normalized_effective_sample_size,
        })
    }
}

/// Drive the motor-level racing MPPI controller around a lap and report metrics.
pub fn simulate_motor_race(
    config: MotorMppiConfig,
    params: MotorQuadParams,
    lap: &RacingGateLap3D,
    start: MotorQuadState,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<MotorRacingLapReport> {
    let dt = config.dt;
    let gate_count = lap.gate_count();
    let mut controller = MotorMppiController::new(config, params)?;

    let mut state = start;
    let mut passed = 0usize;
    let mut path = vec![state.position];
    let mut sum_speed = 0.0;
    let mut max_speed = 0.0_f64;
    let mut sum_tilt = 0.0;
    let mut max_tilt = 0.0_f64;
    let mut max_body_rate = 0.0_f64;
    let mut saturated_steps = 0usize;
    let mut path_length = 0.0;
    let mut min_aperture_margin = f64::INFINITY;
    let mut first_lap_time = None;
    let mut executed_steps = 0usize;

    for step in 0..max_steps {
        let plan = controller.plan(state, lap, passed)?;
        let next = params.step(state, plan.command, dt);
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
        max_body_rate = max_body_rate.max(norm(next.body_rates));
        if params.is_saturated(plan.command) {
            saturated_steps += 1;
        }
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

    Ok(MotorRacingLapReport {
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
        max_body_rate,
        saturation_fraction: saturated_steps as f64 / denom,
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

// --- Quaternion and vector helpers ---------------------------------------

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let qv = [x, y, z];
    let t = scale(cross(qv, v), 2.0);
    let cross_t = cross(qv, t);
    [
        v[0] + w * t[0] + cross_t[0],
        v[1] + w * t[1] + cross_t[1],
        v[2] + w * t[2] + cross_t[2],
    ]
}

fn integrate_attitude(q: [f64; 4], rates: [f64; 3], dt: f64) -> [f64; 4] {
    let [qw, qx, qy, qz] = q;
    let [wx, wy, wz] = rates;
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
        RacingGatePlane3D::new(center, normal, [0.0, 0.0, 1.0], 0.9, 0.9).unwrap()
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
        let params = MotorQuadParams::default();
        let mut state = MotorQuadState::at(0.0, 0.0, 1.0);
        for _ in 0..30 {
            state = params.step(state, MotorCommand::hover(params.gravity), 0.05);
        }
        assert!(state.speed() < 1e-6, "speed {}", state.speed());
        assert!((state.position[2] - 1.0).abs() < 1e-6);
        assert!(state.tilt_angle() < 1e-9);
    }

    #[test]
    fn differential_rotors_build_body_rate_and_tilt() {
        let params = MotorQuadParams::default();
        let mut state = MotorQuadState::at(0.0, 0.0, 0.0);
        let hover = params.gravity / 4.0;
        // Pitch command: front rotors high, rear rotors low.
        let command = MotorCommand::new([hover + 1.5, hover + 1.5, hover - 1.5, hover - 1.5]);
        for _ in 0..6 {
            state = params.step(state, command, 0.05);
        }
        assert!(
            norm(state.body_rates) > 0.1,
            "body rate {:?}",
            state.body_rates
        );
        assert!(state.tilt_angle() > 0.05, "tilt {}", state.tilt_angle());
    }

    #[test]
    fn rate_damping_bounds_spin() {
        let params = MotorQuadParams::default();
        let mut state = MotorQuadState::at(0.0, 0.0, 0.0);
        let hover = params.gravity / 4.0;
        let command = MotorCommand::new([hover + 2.0, hover + 2.0, hover - 2.0, hover - 2.0]);
        for _ in 0..200 {
            state = params.step(state, command, 0.05);
        }
        // Damping caps the steady-state rate at torque / damping, not infinity.
        assert!(norm(state.body_rates).is_finite());
        assert!(norm(state.body_rates) < 200.0);
    }

    #[test]
    fn saturation_is_detected_and_clamped() {
        let params = MotorQuadParams::default();
        let command = MotorCommand::new([100.0, 0.0, 1.0, 1.0]);
        assert!(params.is_saturated(command));
        let saturated = params.saturate(command);
        assert!((saturated.rotors[0] - params.max_rotor_thrust).abs() < 1e-9);
        assert!(saturated.rotors[1] >= 0.0);
    }

    #[test]
    fn rejects_unhoverable_params() {
        // gravity 9.81 needs 4 * max_rotor >= 9.81 -> max_rotor >= 2.45.
        assert!(MotorQuadParams::new(9.81, 0.3, 2.0, 9.0, 2.0, 1.2, 7.0).is_err());
    }

    #[test]
    fn controller_makes_gate_progress() {
        let lap = straight_lap();
        let report = simulate_motor_race(
            MotorMppiConfig::default(),
            MotorQuadParams::default(),
            &lap,
            MotorQuadState::at(0.0, 0.0, 0.0),
            140,
            1,
        )
        .unwrap();
        assert!(
            report.gates_passed >= 1,
            "expected at least one gate, got {}",
            report.gates_passed
        );
        assert!(report.max_tilt > 0.05, "max tilt {}", report.max_tilt);
        assert!((0.0..=1.0).contains(&report.saturation_fraction));
    }

    #[test]
    fn simulation_is_deterministic() {
        let lap = straight_lap();
        let run = || {
            simulate_motor_race(
                MotorMppiConfig::default(),
                MotorQuadParams::default(),
                &lap,
                MotorQuadState::at(0.0, 0.0, 0.0),
                100,
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
        let config = MotorMppiConfig {
            samples: 0,
            ..MotorMppiConfig::default()
        };
        assert!(MotorMppiController::new(config, MotorQuadParams::default()).is_err());
    }
}
