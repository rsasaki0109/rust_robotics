//! Reference-free racing MPPI with a motor-lag and battery-sag powertrain.
//!
//! This deepens [`crate::racing_mppi_motor`] one more level: between the
//! commanded rotor thrusts and the rigid-body physics it inserts a *powertrain*
//! with two unmodeled-by-the-planner effects that every real racing quad has.
//!
//! 1. **First-order motor lag.** A rotor cannot change thrust instantly; the
//!    actual thrust tracks the command through a first-order spin-up lag with
//!    time constant `motor_tau`. Aggressive attitude flicks are therefore
//!    *smeared* in time — the torque the body actually feels lags the command.
//! 2. **Battery sag.** The available per-rotor thrust ceiling is not constant.
//!    It droops instantaneously with electrical load (internal resistance) and
//!    permanently as the pack's state of charge falls over the run. A drone
//!    that flies hard early has less authority late in the race.
//!
//! The powertrain is built by *composition*: it reuses
//! [`MotorQuadParams::step`] verbatim for the translational and rotational
//! physics and only adds the actuator front-end (lagged rotor thrusts plus a
//! battery state). With the [`PowertrainParams::ideal`] constructor — zero lag,
//! no discharge, no sag — it reduces exactly to the motor-level model, which is
//! the baseline the benchmark compares against.
//!
//! - [`PowertrainState`] wraps a [`MotorQuadState`] with the actual (lagged)
//!   rotor thrusts and the battery state of charge.
//! - [`PowertrainParams`] holds the base [`MotorQuadParams`] plus the lag,
//!   discharge, and sag coefficients, and exposes [`PowertrainParams::step`].
//! - [`simulate_powertrain_race`] drives the *powertrain-unaware*
//!   [`MotorMppiController`] (which plans assuming ideal actuators) through the
//!   real powertrain and reports [`PowertrainLapReport`], adding battery and
//!   ceiling-saturation metrics. The honest result is how much an idealized
//!   plan costs when it meets a laggy, sagging powertrain.
//! - [`PowertrainMppiController`] is the *powertrain-aware* answer: it rolls
//!   candidates out through [`PowertrainParams::step`], so it plans within the
//!   deliverable authority and conserves charge for later gates.
//!   [`simulate_powertrain_race_aware`] runs it through the same closed loop.
//! - [`ChargeBudget`] adds an explicit energy term to the aware controller,
//!   penalizing below-reserve current draw so it can trade lap progress for
//!   charge held back; [`simulate_powertrain_race_budgeted`] runs it.
//! - [`PowertrainParams::with_recovery`] adds a relaxation overpotential so the
//!   terminal voltage (and thus the thrust ceiling) recovers when the load eases,
//!   even as the state of charge only ever falls.

use crate::racing_mppi_3d::{RacingGateLap3D, RacingGatePlane3D};
use crate::racing_mppi_motor::{
    MotorCommand, MotorMppiConfig, MotorMppiController, MotorMppiPlan, MotorQuadParams,
    MotorQuadState,
};
use rand::{rngs::StdRng, SeedableRng};
use rand_distr::{Distribution, Normal};
use rust_robotics_core::{RoboticsError, RoboticsResult};

/// State of the powertrain-level quadrotor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowertrainState {
    /// Rigid-body state (position, velocity, attitude, body rates).
    pub motor: MotorQuadState,
    /// Actual (lagged) per-rotor thrusts \[m/s^2\], mass-normalized.
    pub rotor_thrust: [f64; 4],
    /// Battery state of charge in `[0, 1]`.
    pub battery_soc: f64,
    /// Battery relaxation overpotential in `[0, 1]`: a recoverable voltage
    /// depression that builds under sustained load and decays (the terminal
    /// voltage recovers) when the load eases. Distinct from `battery_soc`, which
    /// is the irreversible charge consumed.
    pub relaxation: f64,
}

impl PowertrainState {
    /// Level hover state with a full battery and rotors already at hover thrust.
    pub fn at(x: f64, y: f64, z: f64, gravity: f64) -> Self {
        Self::at_soc(x, y, z, gravity, 1.0)
    }

    /// Level hover state at a given starting state of charge.
    pub fn at_soc(x: f64, y: f64, z: f64, gravity: f64, soc: f64) -> Self {
        Self {
            motor: MotorQuadState::at(x, y, z),
            rotor_thrust: [gravity / 4.0; 4],
            battery_soc: soc.clamp(0.0, 1.0),
            relaxation: 0.0,
        }
    }

    pub fn speed(self) -> f64 {
        self.motor.speed()
    }

    pub fn tilt_angle(self) -> f64 {
        self.motor.tilt_angle()
    }
}

/// Powertrain parameters layered on top of the base motor model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowertrainParams {
    /// Base rigid-body and rotor-mixing parameters.
    pub base: MotorQuadParams,
    /// First-order motor spin-up time constant \[s\]; `0` means instant.
    pub motor_tau: f64,
    /// Battery discharge rate per unit load per second \[1/s\].
    pub discharge_rate: f64,
    /// Instantaneous voltage droop fraction at full electrical load, in `[0, 1)`.
    pub sag_coeff: f64,
    /// Open-circuit voltage scale at an empty pack, in `(0, 1]`.
    pub min_voltage_scale: f64,
    /// Rate \[1/s\] at which the relaxation overpotential builds toward the load.
    pub relax_build: f64,
    /// Rate \[1/s\] at which the relaxation overpotential decays (recovers).
    pub relax_recover: f64,
    /// Voltage-scale depression per unit relaxation overpotential, in `[0, 1)`.
    pub relax_coeff: f64,
}

impl PowertrainParams {
    /// Build a powertrain with explicit lag/battery coefficients.
    pub fn new(
        base: MotorQuadParams,
        motor_tau: f64,
        discharge_rate: f64,
        sag_coeff: f64,
        min_voltage_scale: f64,
    ) -> RoboticsResult<Self> {
        if !motor_tau.is_finite() || motor_tau < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "powertrain motor_tau must be finite and non-negative".to_string(),
            ));
        }
        if !discharge_rate.is_finite() || discharge_rate < 0.0 {
            return Err(RoboticsError::InvalidParameter(
                "powertrain discharge_rate must be finite and non-negative".to_string(),
            ));
        }
        if !sag_coeff.is_finite() || !(0.0..1.0).contains(&sag_coeff) {
            return Err(RoboticsError::InvalidParameter(
                "powertrain sag_coeff must be in [0, 1)".to_string(),
            ));
        }
        if !min_voltage_scale.is_finite()
            || !(0.0..=1.0).contains(&min_voltage_scale)
            || min_voltage_scale <= 0.0
        {
            return Err(RoboticsError::InvalidParameter(
                "powertrain min_voltage_scale must be in (0, 1]".to_string(),
            ));
        }
        Ok(Self {
            base,
            motor_tau,
            discharge_rate,
            sag_coeff,
            min_voltage_scale,
            relax_build: 0.0,
            relax_recover: 0.0,
            relax_coeff: 0.0,
        })
    }

    /// Add a battery-recovery (relaxation-overpotential) model: the relaxation
    /// state builds toward the load at `build` \[1/s\] and decays at `recover`
    /// \[1/s\], depressing the terminal voltage by `coeff` per unit. Easing off
    /// therefore lets the terminal voltage — and thus the thrust ceiling —
    /// recover even though the state of charge keeps falling. All arguments are
    /// sanitized (`build`/`recover >= 0` and finite, `coeff` in `[0, 1)`); the
    /// defaults of zero leave the pack with no recovery dynamics.
    pub fn with_recovery(mut self, build: f64, recover: f64, coeff: f64) -> Self {
        let nonneg = |v: f64| if v.is_finite() && v > 0.0 { v } else { 0.0 };
        self.relax_build = nonneg(build);
        self.relax_recover = nonneg(recover);
        self.relax_coeff = if coeff.is_finite() && (0.0..1.0).contains(&coeff) {
            coeff
        } else {
            0.0
        };
        self
    }

    /// Idealized powertrain: no lag, no discharge, no sag. Reduces exactly to
    /// the base [`MotorQuadParams`] dynamics — the benchmark baseline.
    pub fn ideal(base: MotorQuadParams) -> Self {
        Self {
            base,
            motor_tau: 0.0,
            discharge_rate: 0.0,
            sag_coeff: 0.0,
            min_voltage_scale: 1.0,
            relax_build: 0.0,
            relax_recover: 0.0,
            relax_coeff: 0.0,
        }
    }

    /// Electrical load in `[0, 1]`: total rotor thrust over the absolute max.
    pub fn load(self, rotor_thrust: [f64; 4]) -> f64 {
        let total: f64 = rotor_thrust.iter().sum();
        (total / (4.0 * self.base.max_rotor_thrust)).clamp(0.0, 1.0)
    }

    /// Terminal-voltage scale in `[0, 1]` from state of charge and load.
    ///
    /// Open-circuit voltage falls linearly with state of charge from `1` (full)
    /// to `min_voltage_scale` (empty); instantaneous sag subtracts
    /// `sag_coeff * load` on top of that.
    pub fn voltage_scale(self, soc: f64, load: f64) -> f64 {
        let soc = soc.clamp(0.0, 1.0);
        let ocv = self.min_voltage_scale + (1.0 - self.min_voltage_scale) * soc;
        (ocv - self.sag_coeff * load).clamp(0.0, 1.0)
    }

    /// Terminal-voltage scale including the relaxation overpotential: the
    /// open-circuit-minus-sag [`Self::voltage_scale`] less `relax_coeff *
    /// relaxation`. This is the voltage actually available at the rotors.
    pub fn terminal_voltage_scale(self, soc: f64, load: f64, relaxation: f64) -> f64 {
        (self.voltage_scale(soc, load) - self.relax_coeff * relaxation.clamp(0.0, 1.0))
            .clamp(0.0, 1.0)
    }

    /// Battery-limited per-rotor thrust ceiling for the current state, including
    /// the relaxation overpotential.
    pub fn effective_max_rotor(self, state: PowertrainState) -> f64 {
        let load = self.load(state.rotor_thrust);
        self.base.max_rotor_thrust
            * self.terminal_voltage_scale(state.battery_soc, load, state.relaxation)
    }

    /// Per-step lag blend factor `alpha = 1 - exp(-dt / tau)`, or `1` if instant.
    fn lag_alpha(self, dt: f64) -> f64 {
        if self.motor_tau > 0.0 {
            1.0 - (-dt / self.motor_tau).exp()
        } else {
            1.0
        }
    }

    /// Advance the powertrain one step under the commanded rotor thrusts.
    pub fn step(self, state: PowertrainState, command: MotorCommand, dt: f64) -> PowertrainState {
        // Battery-limited ceiling from the current (causal) load, including the
        // relaxation overpotential.
        let load = self.load(state.rotor_thrust);
        let eff_max = self.base.max_rotor_thrust
            * self.terminal_voltage_scale(state.battery_soc, load, state.relaxation);

        // First-order motor lag toward the ceiling-clamped command.
        let alpha = self.lag_alpha(dt);
        let mut rotor_thrust = state.rotor_thrust;
        for (actual, &commanded) in rotor_thrust.iter_mut().zip(command.rotors.iter()) {
            let target = commanded.clamp(0.0, eff_max);
            *actual += (target - *actual) * alpha;
        }

        // Reuse the base rigid-body + rotor-mixing physics with the *actual*
        // (lagged) rotor thrusts.
        let motor = self
            .base
            .step(state.motor, MotorCommand::new(rotor_thrust), dt);

        // Discharge from the new actual load; monotone, no regeneration.
        let new_load = self.load(rotor_thrust);
        let battery_soc = (state.battery_soc - self.discharge_rate * new_load * dt).clamp(0.0, 1.0);

        // Relaxation overpotential builds toward the load and decays when idle,
        // so easing off lets the terminal voltage recover.
        let relax_dot = self.relax_build * new_load - self.relax_recover * state.relaxation;
        let relaxation = (state.relaxation + relax_dot * dt).clamp(0.0, 1.0);

        PowertrainState {
            motor,
            rotor_thrust,
            battery_soc,
            relaxation,
        }
    }
}

/// Lap-progress, attitude, and powertrain (lag + battery) metrics.
#[derive(Debug, Clone, PartialEq)]
pub struct PowertrainLapReport {
    pub steps: usize,
    pub gates_passed: usize,
    pub laps_completed: usize,
    pub lap_fraction: f64,
    pub mean_speed: f64,
    pub max_speed: f64,
    pub path_length: f64,
    pub gate_distance: f64,
    pub min_aperture_margin: f64,
    pub max_tilt: f64,
    pub max_body_rate: f64,
    /// State of charge at the end of the run.
    pub final_battery_soc: f64,
    /// Lowest battery state of charge reached.
    pub min_battery_soc: f64,
    /// Mean terminal-voltage scale over the run (1 = no sag).
    pub mean_voltage_scale: f64,
    /// Fraction of steps whose commanded rotor exceeded the battery ceiling.
    pub ceiling_saturation_fraction: f64,
    pub first_lap_time: Option<f64>,
    pub path: Vec<[f64; 3]>,
    /// Battery state of charge after each executed step.
    pub battery_trace: Vec<f64>,
}

/// Validate the MPPI config fields shared by both powertrain controllers.
fn validate_config(config: &MotorMppiConfig) -> RoboticsResult<()> {
    if config.horizon == 0 || config.samples == 0 {
        return Err(RoboticsError::InvalidParameter(
            "powertrain MPPI horizon and samples must be positive".to_string(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "powertrain MPPI dt must be finite and positive".to_string(),
        ));
    }
    if !config.rotor_sigma.is_finite() || config.rotor_sigma <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "powertrain MPPI rotor_sigma must be finite and positive".to_string(),
        ));
    }
    if !config.lambda.is_finite() || config.lambda <= 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "powertrain MPPI lambda must be finite and positive".to_string(),
        ));
    }
    if config.velocity_weight < 0.0 || config.rotor_weight < 0.0 || config.level_weight < 0.0 {
        return Err(RoboticsError::InvalidParameter(
            "powertrain MPPI weights must be non-negative".to_string(),
        ));
    }
    Ok(())
}

/// Powertrain-*aware* MPPI controller.
///
/// Unlike [`MotorMppiController`], which rolls candidate commands out through
/// the ideal motor model, this controller rolls them out through the full
/// [`PowertrainParams::step`] — so its samples already feel the motor lag and
/// the battery-limited thrust ceiling. Commands above the (causal) ceiling are
/// clamped inside the rollout, so over-commanding earns no gate progress while
/// still paying the rotor-effort penalty; the controller is therefore pushed to
/// plan within the authority the battery can actually deliver, and it sees the
/// pack drain over the horizon rather than assuming infinite charge.
/// A charge-budget term for the powertrain-aware controller.
///
/// Once a rollout step's state of charge falls below `reserve`, the rollout cost
/// gains `weight * (reserve - soc) * load`, where `load` is the electrical draw.
/// Because the penalty scales with `load` it gives the rollout an actionable
/// gradient — throttle back when low — that a slow-moving state-of-charge term
/// cannot. `weight = 0` recovers the plain aware controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChargeBudget {
    /// Penalty scale on below-reserve current draw; clamped to `>= 0`.
    pub weight: f64,
    /// State-of-charge floor to protect, in `[0, 1]`.
    pub reserve: f64,
}

impl ChargeBudget {
    /// A charge budget with the given weight and reserve, sanitized to sane
    /// ranges (`weight >= 0` and finite, `reserve` in `[0, 1]`).
    pub fn new(weight: f64, reserve: f64) -> Self {
        Self {
            weight: if weight.is_finite() && weight > 0.0 {
                weight
            } else {
                0.0
            },
            reserve: reserve.clamp(0.0, 1.0),
        }
    }

    /// The no-op budget (`weight = 0`): the plain aware controller.
    pub fn none() -> Self {
        Self {
            weight: 0.0,
            reserve: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PowertrainMppiController {
    config: MotorMppiConfig,
    params: PowertrainParams,
    nominal: Vec<MotorCommand>,
    rng: StdRng,
    budget: ChargeBudget,
}

impl PowertrainMppiController {
    pub fn new(config: MotorMppiConfig, params: PowertrainParams) -> RoboticsResult<Self> {
        validate_config(&config)?;
        let nominal = vec![MotorCommand::hover(params.base.gravity); config.horizon];
        let rng = StdRng::seed_from_u64(config.seed);
        Ok(Self {
            config,
            params,
            nominal,
            rng,
            budget: ChargeBudget::none(),
        })
    }

    /// Attach a [`ChargeBudget`] so the controller eases off below the reserve
    /// and trades raw gate progress for charge held back for later laps.
    pub fn with_charge_budget(mut self, budget: ChargeBudget) -> Self {
        self.budget = ChargeBudget::new(budget.weight, budget.reserve);
        self
    }

    pub fn config(&self) -> &MotorMppiConfig {
        &self.config
    }

    fn rollout(&self, start: PowertrainState, controls: &[MotorCommand]) -> (Vec<[f64; 3]>, f64) {
        let mut state = start;
        let mut positions = Vec::with_capacity(controls.len() + 1);
        positions.push(state.motor.position);
        let mut regularizer = 0.0;
        let hover = self.params.base.gravity / 4.0;
        for &command in controls {
            state = self.params.step(state, command, self.config.dt);
            let speed = state.speed();
            let tilt_cost = 1.0 - state.motor.thrust_axis()[2];
            // The effort penalty is on the *commanded* thrust, so commanding
            // past the battery ceiling (which the rollout clamps away) costs
            // effort for no benefit.
            let rotor_effort: f64 = command.rotors.iter().map(|&r| (r - hover).powi(2)).sum();
            // Charge-budget term: once below the protected reserve, penalize the
            // electrical load (current draw) itself, scaled by how deep below the
            // reserve the pack is. Because load couples directly to thrust each
            // step, this gives the rollout an actionable gradient — throttle back
            // when low — that a slow-moving state-of-charge penalty cannot.
            let below_reserve = (self.budget.reserve - state.battery_soc).max(0.0);
            let load = self.params.load(state.rotor_thrust);
            regularizer += self.config.velocity_weight * speed * speed
                + self.config.rotor_weight * rotor_effort
                + self.config.level_weight * tilt_cost
                + self.budget.weight * below_reserve * load;
            positions.push(state.motor.position);
        }
        (positions, regularizer)
    }

    /// Plan the next rotor command given the full powertrain state, lap, and
    /// pass count.
    pub fn plan(
        &mut self,
        start: PowertrainState,
        lap: &RacingGateLap3D,
        passed: usize,
    ) -> RoboticsResult<MotorMppiPlan> {
        let noise = Normal::new(0.0, self.config.rotor_sigma).map_err(|_| {
            RoboticsError::InvalidParameter(
                "invalid powertrain MPPI rotor distribution".to_string(),
            )
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
                controls.push(self.params.base.saturate(MotorCommand::new(rotors)));
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
            *command = self.params.base.saturate(*command);
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

/// Shared closed-loop driver: step the powertrain under whatever command the
/// `plan_command` closure returns each step, accumulating the lap report.
fn run_powertrain_race<F>(
    dt: f64,
    params: PowertrainParams,
    lap: &RacingGateLap3D,
    start: PowertrainState,
    max_steps: usize,
    target_laps: usize,
    mut plan_command: F,
) -> RoboticsResult<PowertrainLapReport>
where
    F: FnMut(&PowertrainState, usize) -> RoboticsResult<MotorCommand>,
{
    let gate_count = lap.gate_count();
    let mut state = start;
    let mut passed = 0usize;
    let mut path = vec![state.motor.position];
    let mut battery_trace = Vec::new();
    let mut sum_speed = 0.0;
    let mut max_speed = 0.0_f64;
    let mut max_tilt = 0.0_f64;
    let mut max_body_rate = 0.0_f64;
    let mut sum_voltage_scale = 0.0;
    let mut ceiling_saturated = 0usize;
    let mut path_length = 0.0;
    let mut min_aperture_margin = f64::INFINITY;
    let mut min_battery_soc = state.battery_soc;
    let mut first_lap_time = None;
    let mut executed_steps = 0usize;

    for step in 0..max_steps {
        let command = plan_command(&state, passed)?;

        // Diagnose whether this command will hit the battery ceiling.
        let eff_max = params.effective_max_rotor(state);
        let load = params.load(state.rotor_thrust);
        sum_voltage_scale +=
            params.terminal_voltage_scale(state.battery_soc, load, state.relaxation);
        if command.rotors.iter().any(|&r| r > eff_max + 1e-9) {
            ceiling_saturated += 1;
        }

        // Execute through the real powertrain.
        let next = params.step(state, command, dt);
        let from = state.motor.position;
        let to = next.motor.position;

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
        max_tilt = max_tilt.max(next.tilt_angle());
        max_body_rate = max_body_rate.max(norm(next.motor.body_rates));
        min_battery_soc = min_battery_soc.min(next.battery_soc);
        state = next;
        path.push(to);
        battery_trace.push(state.battery_soc);
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
    let gate_distance = distance(active_gate.center(), state.motor.position);
    let denom = executed_steps.max(1) as f64;

    Ok(PowertrainLapReport {
        steps: executed_steps,
        gates_passed: passed,
        laps_completed,
        lap_fraction,
        mean_speed: sum_speed / denom,
        max_speed,
        path_length,
        gate_distance,
        min_aperture_margin,
        max_tilt,
        max_body_rate,
        final_battery_soc: state.battery_soc,
        min_battery_soc,
        mean_voltage_scale: sum_voltage_scale / denom,
        ceiling_saturation_fraction: ceiling_saturated as f64 / denom,
        first_lap_time,
        path,
        battery_trace,
    })
}

/// Drive the powertrain-*unaware* MPPI controller through the real powertrain.
///
/// The controller plans rotor commands with `config` and `params.base`,
/// assuming ideal actuators; each command is then executed through the lagging,
/// sagging powertrain. The gap between plan and execution is the point.
pub fn simulate_powertrain_race(
    config: MotorMppiConfig,
    params: PowertrainParams,
    lap: &RacingGateLap3D,
    start: PowertrainState,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<PowertrainLapReport> {
    let dt = config.dt;
    let mut controller = MotorMppiController::new(config, params.base)?;
    run_powertrain_race(
        dt,
        params,
        lap,
        start,
        max_steps,
        target_laps,
        |state, passed| controller.plan(state.motor, lap, passed).map(|p| p.command),
    )
}

/// Drive the powertrain-*aware* MPPI controller through the real powertrain.
///
/// The controller rolls candidates out through [`PowertrainParams::step`], so it
/// plans within the lag and battery-limited authority instead of assuming ideal
/// actuators. On a drained pack this conserves the authority the unaware
/// controller wastes against the ceiling.
pub fn simulate_powertrain_race_aware(
    config: MotorMppiConfig,
    params: PowertrainParams,
    lap: &RacingGateLap3D,
    start: PowertrainState,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<PowertrainLapReport> {
    let dt = config.dt;
    let mut controller = PowertrainMppiController::new(config, params)?;
    run_powertrain_race(
        dt,
        params,
        lap,
        start,
        max_steps,
        target_laps,
        |state, passed| controller.plan(*state, lap, passed).map(|p| p.command),
    )
}

/// Drive a *charge-budgeted* powertrain-aware controller through the powertrain.
///
/// Like [`simulate_powertrain_race_aware`] but the controller carries a
/// [`ChargeBudget`] that penalizes below-reserve current draw. Over a multi-lap
/// race the budgeted controller eases off as it nears the reserve, trading raw
/// lap progress for charge held back in the pack. A zero-weight budget recovers
/// the plain aware controller.
pub fn simulate_powertrain_race_budgeted(
    config: MotorMppiConfig,
    params: PowertrainParams,
    budget: ChargeBudget,
    lap: &RacingGateLap3D,
    start: PowertrainState,
    max_steps: usize,
    target_laps: usize,
) -> RoboticsResult<PowertrainLapReport> {
    let dt = config.dt;
    let mut controller = PowertrainMppiController::new(config, params)?.with_charge_budget(budget);
    run_powertrain_race(
        dt,
        params,
        lap,
        start,
        max_steps,
        target_laps,
        |state, passed| controller.plan(*state, lap, passed).map(|p| p.command),
    )
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

    fn racing_params() -> PowertrainParams {
        PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.05, 0.18, 0.55).unwrap()
    }

    #[test]
    fn ideal_powertrain_matches_base_motor_step() {
        let base = MotorQuadParams::default();
        let pt = PowertrainParams::ideal(base);
        let mut pstate = PowertrainState::at(0.0, 0.0, 0.0, base.gravity);
        let mut mstate = MotorQuadState::at(0.0, 0.0, 0.0);
        let hover = base.gravity / 4.0;
        let command = MotorCommand::new([hover + 1.0, hover + 1.0, hover - 1.0, hover - 1.0]);
        for _ in 0..20 {
            pstate = pt.step(pstate, command, 0.05);
            mstate = base.step(mstate, command, 0.05);
        }
        assert_eq!(pstate.motor.position, mstate.position);
        assert_eq!(pstate.motor.attitude, mstate.attitude);
        assert_eq!(pstate.motor.body_rates, mstate.body_rates);
        assert!((pstate.battery_soc - 1.0).abs() < 1e-12);
    }

    #[test]
    fn motor_lag_delays_thrust_response() {
        let pt = racing_params();
        let base = pt.base;
        let mut state = PowertrainState::at(0.0, 0.0, 1.0, base.gravity);
        let hover = base.gravity / 4.0;
        // Step every rotor up toward the ceiling.
        let command = MotorCommand::new([hover + 2.0; 4]);
        let target = (hover + 2.0).min(pt.effective_max_rotor(state));
        let before = state.rotor_thrust[0];
        state = pt.step(state, command, 0.05);
        let after_one = state.rotor_thrust[0];
        // After one step the actual thrust has moved toward the target but not
        // reached it (first-order lag).
        assert!(
            after_one > before,
            "lag should ramp up: {after_one} vs {before}"
        );
        assert!(
            after_one < target - 1e-6,
            "lag should not snap: {after_one} vs {target}"
        );
        // Many steps later it has essentially converged to the ceiling target.
        for _ in 0..200 {
            state = pt.step(state, command, 0.05);
        }
        let converged = pt.effective_max_rotor(state);
        assert!((state.rotor_thrust[0] - converged).abs() < 0.2);
    }

    #[test]
    fn battery_depletes_under_load() {
        let pt = racing_params();
        let base = pt.base;
        let mut state = PowertrainState::at(0.0, 0.0, 1.0, base.gravity);
        let start_soc = state.battery_soc;
        for _ in 0..200 {
            state = pt.step(state, MotorCommand::hover(base.gravity), 0.05);
        }
        assert!(state.battery_soc < start_soc, "soc {} ", state.battery_soc);
        assert!(state.battery_soc >= 0.0);
    }

    #[test]
    fn sag_lowers_ceiling_as_battery_drains() {
        let pt = racing_params();
        let base = pt.base;
        let full = PowertrainState::at_soc(0.0, 0.0, 0.0, base.gravity, 1.0);
        let empty = PowertrainState::at_soc(0.0, 0.0, 0.0, base.gravity, 0.0);
        assert!(
            pt.effective_max_rotor(full) > pt.effective_max_rotor(empty),
            "full {} empty {}",
            pt.effective_max_rotor(full),
            pt.effective_max_rotor(empty)
        );
        // Even empty, the drone can still hover (ceiling*4 >= gravity).
        assert!(4.0 * pt.effective_max_rotor(empty) >= base.gravity);
    }

    #[test]
    fn instantaneous_sag_drops_with_load() {
        let pt = racing_params();
        let high = pt.voltage_scale(1.0, 1.0);
        let low = pt.voltage_scale(1.0, 0.2);
        assert!(low > high, "more load should sag more: {low} vs {high}");
    }

    fn recovery_params() -> PowertrainParams {
        racing_params().with_recovery(0.8, 0.5, 0.25)
    }

    #[test]
    fn recovery_is_off_by_default() {
        // Without with_recovery, the relaxation state never leaves zero.
        let pt = racing_params();
        let mut state = PowertrainState::at(0.0, 0.0, 1.0, pt.base.gravity);
        for _ in 0..40 {
            state = pt.step(
                state,
                MotorCommand::new([pt.base.max_rotor_thrust; 4]),
                0.05,
            );
        }
        assert_eq!(state.relaxation, 0.0);
    }

    #[test]
    fn relaxation_depresses_terminal_voltage() {
        let pt = recovery_params();
        // Same charge and load, but a relaxed pack delivers less terminal voltage.
        let fresh = pt.terminal_voltage_scale(0.8, 0.5, 0.0);
        let relaxed = pt.terminal_voltage_scale(0.8, 0.5, 0.6);
        assert!(
            relaxed < fresh,
            "relaxed {relaxed} should be below fresh {fresh}"
        );
        // With recovery off, relaxation has no effect.
        let no_recovery = racing_params();
        assert_eq!(
            no_recovery.terminal_voltage_scale(0.8, 0.5, 0.6),
            no_recovery.voltage_scale(0.8, 0.5)
        );
    }

    #[test]
    fn terminal_voltage_recovers_when_load_eases() {
        let pt = recovery_params();
        let mut state = PowertrainState::at(0.0, 0.0, 1.0, pt.base.gravity);
        // Drive hard: the relaxation overpotential builds up.
        for _ in 0..40 {
            state = pt.step(
                state,
                MotorCommand::new([pt.base.max_rotor_thrust; 4]),
                0.05,
            );
        }
        let relax_hot = state.relaxation;
        let soc_hot = state.battery_soc;
        assert!(relax_hot > 0.1, "relaxation should build: {relax_hot}");
        // Ease off to hover: the overpotential decays and voltage recovers.
        for _ in 0..80 {
            state = pt.step(state, MotorCommand::hover(pt.base.gravity), 0.05);
        }
        assert!(
            state.relaxation < relax_hot,
            "relaxation should recover: {} vs {relax_hot}",
            state.relaxation
        );
        // ...but the state of charge keeps falling — recovery is not free energy.
        assert!(
            state.battery_soc < soc_hot,
            "soc should keep dropping: {} vs {soc_hot}",
            state.battery_soc
        );
    }

    #[test]
    fn rejects_invalid_params() {
        let base = MotorQuadParams::default();
        assert!(PowertrainParams::new(base, -1.0, 0.05, 0.18, 0.55).is_err());
        assert!(PowertrainParams::new(base, 0.08, -0.1, 0.18, 0.55).is_err());
        assert!(PowertrainParams::new(base, 0.08, 0.05, 1.0, 0.55).is_err());
        assert!(PowertrainParams::new(base, 0.08, 0.05, 0.18, 0.0).is_err());
    }

    #[test]
    fn controller_makes_gate_progress_through_powertrain() {
        let lap = straight_lap();
        let pt = racing_params();
        let report = simulate_powertrain_race(
            MotorMppiConfig::default(),
            pt,
            &lap,
            PowertrainState::at(0.0, 0.0, 0.0, pt.base.gravity),
            160,
            1,
        )
        .unwrap();
        assert!(
            report.gates_passed >= 1,
            "expected gate progress, got {}",
            report.gates_passed
        );
        assert!(report.final_battery_soc < 1.0);
        assert!((0.0..=1.0).contains(&report.ceiling_saturation_fraction));
        assert_eq!(report.battery_trace.len(), report.steps);
    }

    #[test]
    fn simulation_is_deterministic() {
        let lap = straight_lap();
        let pt = racing_params();
        let run = || {
            simulate_powertrain_race(
                MotorMppiConfig::default(),
                pt,
                &lap,
                PowertrainState::at(0.0, 0.0, 0.0, pt.base.gravity),
                120,
                1,
            )
            .unwrap()
        };
        let first = run();
        let second = run();
        assert_eq!(first.path, second.path);
        assert_eq!(first.battery_trace, second.battery_trace);
        assert_eq!(first.gates_passed, second.gates_passed);
    }

    #[test]
    fn aware_on_ideal_matches_motor_controller() {
        // On an ideal powertrain the aware rollout equals the base motor model,
        // so with the same seed the aware controller must plan the same command.
        let base = MotorQuadParams::default();
        let lap = straight_lap();
        let config = MotorMppiConfig::default();
        let mut motor = MotorMppiController::new(config.clone(), base).unwrap();
        let mut aware =
            PowertrainMppiController::new(config, PowertrainParams::ideal(base)).unwrap();
        let motor_plan = motor
            .plan(MotorQuadState::at(0.0, 0.0, 0.0), &lap, 0)
            .unwrap();
        let aware_plan = aware
            .plan(PowertrainState::at(0.0, 0.0, 0.0, base.gravity), &lap, 0)
            .unwrap();
        // Equal up to the float rounding of the (no-op) lag blend `h + (t - h)`.
        for (m, a) in motor_plan
            .command
            .rotors
            .iter()
            .zip(aware_plan.command.rotors.iter())
        {
            assert!((m - a).abs() < 1e-9, "{m} vs {a}");
        }
    }

    #[test]
    fn aware_controller_makes_gate_progress() {
        let lap = straight_lap();
        let pt = racing_params();
        let report = simulate_powertrain_race_aware(
            MotorMppiConfig::default(),
            pt,
            &lap,
            PowertrainState::at(0.0, 0.0, 0.0, pt.base.gravity),
            160,
            1,
        )
        .unwrap();
        assert!(
            report.gates_passed >= 1,
            "expected gate progress, got {}",
            report.gates_passed
        );
        assert_eq!(report.battery_trace.len(), report.steps);
    }

    #[test]
    fn budget_zero_weight_matches_aware() {
        // A zero-weight charge budget must recover the plain aware controller.
        let lap = straight_lap();
        let pt = racing_params();
        let start = PowertrainState::at_soc(0.0, 0.0, 0.0, pt.base.gravity, 0.6);
        let aware =
            simulate_powertrain_race_aware(MotorMppiConfig::default(), pt, &lap, start, 120, 1)
                .unwrap();
        let budgeted = simulate_powertrain_race_budgeted(
            MotorMppiConfig::default(),
            pt,
            ChargeBudget::new(0.0, 0.4),
            &lap,
            start,
            120,
            1,
        )
        .unwrap();
        assert_eq!(aware.path, budgeted.path);
        assert_eq!(aware.battery_trace, budgeted.battery_trace);
    }

    #[test]
    fn budget_protects_the_reserve() {
        // On a draining closed lap, a charge budget ends with more charge than
        // the greedy aware controller flying the same course.
        let s = 2.0;
        let gates = vec![
            gate([s, 0.0, 0.0], [0.0, 1.0, 0.0]),
            gate([0.0, s, 0.0], [-1.0, 0.0, 0.0]),
            gate([-s, 0.0, 0.0], [0.0, -1.0, 0.0]),
            gate([0.0, -s, 0.0], [1.0, 0.0, 0.0]),
        ];
        let lap = RacingGateLap3D::closed_loop(gates).unwrap();
        let pt = PowertrainParams::new(MotorQuadParams::default(), 0.08, 0.12, 0.18, 0.55).unwrap();
        let start = PowertrainState::at_soc(s, -1.5, 0.0, pt.base.gravity, 0.7);
        let greedy = simulate_powertrain_race_budgeted(
            MotorMppiConfig::default(),
            pt,
            ChargeBudget::none(),
            &lap,
            start,
            220,
            6,
        )
        .unwrap();
        let budgeted = simulate_powertrain_race_budgeted(
            MotorMppiConfig::default(),
            pt,
            ChargeBudget::new(40.0, 0.4),
            &lap,
            start,
            220,
            6,
        )
        .unwrap();
        assert!(
            budgeted.final_battery_soc > greedy.final_battery_soc,
            "budgeted soc {} should exceed greedy {}",
            budgeted.final_battery_soc,
            greedy.final_battery_soc
        );
    }

    #[test]
    fn aware_beats_unaware_on_drained_pack() {
        // A reach-up climb under a drained pack: the unaware controller wastes
        // authority against the ceiling, the aware one budgets within it.
        let gates = vec![
            gate([1.4, 0.0, 0.5], [1.0, 0.0, 0.4]),
            gate([2.8, 0.0, 1.1], [1.0, 0.0, 0.4]),
        ];
        let lap = RacingGateLap3D::open(gates).unwrap();
        let pt = racing_params();
        let start = PowertrainState::at_soc(0.0, 0.0, 0.0, pt.base.gravity, 0.22);
        let unaware =
            simulate_powertrain_race(MotorMppiConfig::default(), pt, &lap, start, 200, 1).unwrap();
        let aware =
            simulate_powertrain_race_aware(MotorMppiConfig::default(), pt, &lap, start, 200, 1)
                .unwrap();
        assert!(
            aware.gates_passed >= unaware.gates_passed,
            "aware {} should reach >= unaware {}",
            aware.gates_passed,
            unaware.gates_passed
        );
    }
}
