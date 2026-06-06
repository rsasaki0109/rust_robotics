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

use crate::racing_mppi_3d::{RacingGateLap3D, RacingGatePlane3D};
use crate::racing_mppi_motor::{
    MotorCommand, MotorMppiConfig, MotorMppiController, MotorQuadParams, MotorQuadState,
};
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
        })
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

    /// Battery-limited per-rotor thrust ceiling for the current state.
    pub fn effective_max_rotor(self, state: PowertrainState) -> f64 {
        let load = self.load(state.rotor_thrust);
        self.base.max_rotor_thrust * self.voltage_scale(state.battery_soc, load)
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
        // Battery-limited ceiling from the current (causal) load.
        let load = self.load(state.rotor_thrust);
        let eff_max = self.base.max_rotor_thrust * self.voltage_scale(state.battery_soc, load);

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

        PowertrainState {
            motor,
            rotor_thrust,
            battery_soc,
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

/// Drive the powertrain-unaware MPPI controller through the real powertrain.
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
    let gate_count = lap.gate_count();
    let mut controller = MotorMppiController::new(config, params.base)?;

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
        // Plan as if actuators were ideal (controller sees only the base model).
        let plan = controller.plan(state.motor, lap, passed)?;

        // Diagnose whether this command will hit the battery ceiling.
        let eff_max = params.effective_max_rotor(state);
        let load = params.load(state.rotor_thrust);
        sum_voltage_scale += params.voltage_scale(state.battery_soc, load);
        if plan.command.rotors.iter().any(|&r| r > eff_max + 1e-9) {
            ceiling_saturated += 1;
        }

        // Execute through the real powertrain.
        let next = params.step(state, plan.command, dt);
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
}
