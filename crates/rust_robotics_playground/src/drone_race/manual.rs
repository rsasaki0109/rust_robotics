//! Assisted human commands. Inputs request velocity; only thrust and body rates
//! are applied to the same quadrotor dynamics used by the autonomous pilot.

use rust_robotics_control::{QuadrotorControl, QuadrotorParams, QuadrotorState};

use super::flight::{dot, unit, MAX_STEPS};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PilotInput {
    pub forward: i8,
    pub right: i8,
    pub lift: i8,
}

impl PilotInput {
    pub fn new(forward: i8, right: i8, lift: i8) -> Self {
        Self {
            forward: forward.clamp(-1, 1),
            right: right.clamp(-1, 1),
            lift: lift.clamp(-1, 1),
        }
    }

    pub fn active(self) -> bool {
        self != Self::default()
    }

    fn code(self) -> char {
        char::from_digit(
            ((self.forward + 1) * 9 + (self.right + 1) * 3 + self.lift + 1) as u32,
            36,
        )
        .expect("three ternary axes fit in one base36 digit")
    }
}

/// Versioned, URL-safe recording: one character per 0.05 s physical update.
/// Even a full 30 s flight uses only 603 characters, without external storage.
pub fn encode(inputs: &[PilotInput]) -> String {
    let mut encoded = String::with_capacity(inputs.len() + 3);
    encoded.push_str("v1.");
    encoded.extend(inputs.iter().map(|input| input.code()));
    encoded
}

pub fn decode(encoded: &str) -> Option<Vec<PilotInput>> {
    let body = encoded.strip_prefix("v1.")?;
    if body.len() > MAX_STEPS {
        return None;
    }
    body.chars()
        .map(|ch| {
            let code = ch.to_digit(36)?;
            // Accept only the canonical lowercase alphabet and 27 states.
            if code >= 27 || char::from_digit(code, 36)? != ch {
                return None;
            }
            Some(PilotInput::new(
                (code / 9) as i8 - 1,
                ((code / 3) % 3) as i8 - 1,
                (code % 3) as i8 - 1,
            ))
        })
        .collect()
}

pub fn command(
    state: QuadrotorState,
    input: PilotInput,
    params: QuadrotorParams,
) -> QuadrotorControl {
    // The courses face +x; screen-right in Chase is -y. Holding no input
    // requests zero velocity, so the assist brakes and levels the aircraft.
    let target = [
        f64::from(input.forward) * 5.0,
        -f64::from(input.right) * 3.0,
        f64::from(input.lift) * 2.5,
    ];
    let force = std::array::from_fn::<_, 3, _>(|i| {
        let acceleration = ((target[i] - state.velocity[i]) * 3.5).clamp(-8.0, 8.0);
        acceleration + params.drag * state.velocity[i] + if i == 2 { params.gravity } else { 0.0 }
    });
    let up = unit(force);
    let roll = (-up[1]).clamp(-1.0, 1.0).asin();
    let pitch = up[0].atan2(up[2]);
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let desired = [cp * cr, cp * sr, sp * cr, -sp * sr];
    let [w, x, y, z] = state.attitude;
    // conjugate(current) * desired: attitude error in body coordinates.
    let error = [
        w * desired[0] + x * desired[1] + y * desired[2] + z * desired[3],
        w * desired[1] - x * desired[0] - y * desired[3] + z * desired[2],
        w * desired[2] + x * desired[3] - y * desired[0] - z * desired[1],
        w * desired[3] - x * desired[2] + y * desired[1] - z * desired[0],
    ];
    let sign = if error[0] < 0.0 { -1.0 } else { 1.0 };
    params.saturate(QuadrotorControl::new(
        dot(force, force).sqrt(),
        [error[1], error[2], error[3]].map(|value| value * sign * 12.0),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drone_race::flight::DT;

    #[test]
    fn input_tilts_the_real_aircraft_and_release_brakes_it() {
        let params = QuadrotorParams::default();
        let mut state = QuadrotorState::at(0.0, 0.0, 2.0);
        for _ in 0..40 {
            state = params.step(state, command(state, PilotInput::default(), params), DT);
        }
        assert_eq!(state, QuadrotorState::at(0.0, 0.0, 2.0));
        let input = PilotInput::new(1, 1, 1);
        let tilted = params.step(state, command(state, input, params), DT);
        assert!(tilted.tilt_angle() > 0.05);
        for _ in 0..40 {
            let control = command(state, input, params);
            assert!(control.thrust <= params.max_thrust);
            assert!(control.body_rate_magnitude() <= params.max_body_rate + 1e-12);
            state = params.step(state, control, DT);
        }
        assert!(state.position[0] > 5.0 && state.position[1] < -3.0 && state.position[2] > 5.0);
        assert!(state.speed() <= params.max_speed);
        let release_position = state.position;
        for _ in 0..100 {
            state = params.step(state, command(state, PilotInput::default(), params), DT);
        }
        assert!(
            state.speed() < 0.01,
            "release must brake: {:?}",
            state.velocity
        );
        assert!(state.tilt_angle() < 0.01);
        assert!(state.position[0] - release_position[0] < 2.5);
        assert!((state.attitude.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn recording_roundtrips_every_command_and_rejects_unbounded_or_invalid_input() {
        let inputs: Vec<_> = (-1..=1)
            .flat_map(|f| {
                (-1..=1).flat_map(move |r| (-1..=1).map(move |z| PilotInput::new(f, r, z)))
            })
            .collect();
        assert_eq!(decode(&encode(&inputs)), Some(inputs));
        let longest = vec![PilotInput::default(); MAX_STEPS];
        assert_eq!(decode(&encode(&longest)), Some(longest));
        for invalid in ["v2.d", "d", "v1.z", "v1.D", "v1.é", "v1.d&frame=5"] {
            assert!(decode(invalid).is_none(), "{invalid}");
        }
        assert!(decode(&format!("v1.{}", "d".repeat(MAX_STEPS + 1))).is_none());
    }
}
