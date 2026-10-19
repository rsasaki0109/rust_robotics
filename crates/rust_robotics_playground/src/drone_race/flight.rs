//! Bounded, deterministic flight history shared by live playback and seeking.

use rust_robotics_control::{
    QuadrotorMppiConfig, QuadrotorMppiController, QuadrotorParams, QuadrotorState, RacingGateLap3D,
};
use rust_robotics_core::RoboticsResult;

use super::manual::{self, PilotInput};

pub const DT: f64 = 0.05;
pub const MAX_STEPS: usize = 600;

pub use super::course::Course;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Flying,
    Finished,
    GroundContact,
    TimeLimit,
}

impl Status {
    #[cfg(target_arch = "wasm32")]
    pub fn slug(self) -> &'static str {
        match self {
            Self::Flying => "flying",
            Self::Finished => "finished",
            Self::GroundContact => "ground-contact",
            Self::TimeLimit => "time-limit",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FlightFrame {
    pub state: QuadrotorState,
    pub passed: usize,
    pub status: Status,
    pub prediction: Vec<[f64; 3]>,
}

pub struct Flight {
    pub course: Course,
    pub lap: RacingGateLap3D,
    pub frames: Vec<FlightFrame>,
    pub inputs: Vec<PilotInput>,
    controller: Option<QuadrotorMppiController>,
    params: QuadrotorParams,
}

impl Flight {
    pub fn new(course: Course) -> Self {
        let params = QuadrotorParams::default();
        let config = QuadrotorMppiConfig::default();
        let controller = QuadrotorMppiController::new(config, params)
            .expect("built-in race configuration is valid");
        Self {
            course,
            lap: course.lap(),
            frames: vec![FlightFrame {
                state: QuadrotorState::at(0.0, 0.0, 2.0),
                passed: 0,
                status: Status::Flying,
                prediction: Vec::new(),
            }],
            inputs: Vec::new(),
            controller: Some(controller),
            params,
        }
    }

    pub fn manual(course: Course) -> Self {
        let mut flight = Self::new(course);
        flight.controller = None;
        flight
    }

    pub fn is_manual(&self) -> bool {
        self.controller.is_none()
    }

    pub fn last_index(&self) -> usize {
        self.frames.len() - 1
    }

    pub fn done(&self) -> bool {
        self.frames[self.last_index()].status != Status::Flying
    }

    /// Append exactly one physical update. A terminal history never grows.
    pub fn step(&mut self) -> RoboticsResult<bool> {
        self.step_with_input(PilotInput::default())
    }

    pub fn step_with_input(&mut self, input: PilotInput) -> RoboticsResult<bool> {
        if self.done() {
            return Ok(false);
        }
        let current = &self.frames[self.last_index()];
        let control = if let Some(controller) = &mut self.controller {
            controller
                .plan(current.state, &self.lap, current.passed)?
                .control
        } else {
            manual::command(current.state, input, self.params)
        };
        let state = self.params.step(current.state, control, DT);
        let mut passed = current.passed;
        while passed < self.lap.gate_count()
            && self.lap.gates[passed].crosses(
                current.state.position,
                state.position,
                self.lap.crossing_tolerance,
            )
        {
            passed += 1;
        }
        let status = if state.position[2] <= 0.1 {
            Status::GroundContact
        } else if passed == self.lap.gate_count() {
            Status::Finished
        } else if self.frames.len() >= MAX_STEPS {
            Status::TimeLimit
        } else {
            Status::Flying
        };
        self.frames.push(FlightFrame {
            state,
            passed,
            status,
            prediction: self
                .controller
                .as_ref()
                .map_or_else(Vec::new, |c| c.predicted_positions(state)),
        });
        if self.is_manual() {
            self.inputs.push(input);
        }
        Ok(true)
    }
}

pub fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn unit(a: [f64; 3]) -> [f64; 3] {
    let norm = dot(a, a).sqrt();
    if norm < 1e-9 {
        [1.0, 0.0, 0.0]
    } else {
        scale(a, 1.0 / norm)
    }
}

pub fn rotate(q: [f64; 4], p: [f64; 3]) -> [f64; 3] {
    let v = [q[1], q[2], q[3]];
    let t = scale(cross(v, p), 2.0);
    add(p, add(scale(t, q[0]), cross(v, t)))
}

pub fn interpolate(a: QuadrotorState, b: QuadrotorState, alpha: f64) -> QuadrotorState {
    let alpha = alpha.clamp(0.0, 1.0);
    let sign = if a
        .attitude
        .iter()
        .zip(b.attitude)
        .map(|(a, b)| a * b)
        .sum::<f64>()
        < 0.0
    {
        -1.0
    } else {
        1.0
    };
    let mut attitude =
        std::array::from_fn(|i| a.attitude[i] * (1.0 - alpha) + b.attitude[i] * alpha * sign);
    let norm = attitude.iter().map(|v| v * v).sum::<f64>().sqrt();
    for value in &mut attitude {
        *value /= norm;
    }
    QuadrotorState {
        position: add(scale(a.position, 1.0 - alpha), scale(b.position, alpha)),
        velocity: add(scale(a.velocity, 1.0 - alpha), scale(b.velocity, alpha)),
        attitude,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_and_a_custom_layout_finish_by_actual_ordered_gate_crossings() {
        for course in Course::ALL
            .into_iter()
            .chain([Course::CLIMB.moved(1, [8.4, 0.2, 4.0])])
        {
            let mut flight = Flight::new(course);
            while !flight.done() {
                flight.step().unwrap();
            }
            let last = flight.frames.last().unwrap();
            assert_eq!(
                last.status,
                Status::Finished,
                "{} at {:?}, passed {}",
                course.slug(),
                last.state.position,
                last.passed
            );
            assert_eq!(last.passed, flight.lap.gate_count());
            for pair in flight.frames.windows(2) {
                assert!(pair[1].passed >= pair[0].passed);
                for gate in &flight.lap.gates[pair[0].passed..pair[1].passed] {
                    assert!(gate.crosses(
                        pair[0].state.position,
                        pair[1].state.position,
                        flight.lap.crossing_tolerance
                    ));
                }
                assert!(pair[1].state.position.iter().all(|v| v.is_finite()));
            }
            let count = flight.frames.len();
            let passed = last.passed;
            assert!(!flight.step().unwrap());
            assert_eq!(flight.frames.len(), count);
            println!(
                "{}: {} gates, {:.2}s, {} steps",
                course.slug(),
                passed,
                (count - 1) as f64 * DT,
                count - 1
            );
        }
    }

    #[test]
    fn independent_histories_and_prediction_are_identical() {
        let (mut a, mut b) = (Flight::new(Course::NEON), Flight::new(Course::NEON));
        for _ in 0..12 {
            a.step().unwrap();
            b.step().unwrap();
        }
        assert_eq!(a.frames, b.frames);
        let last = a.frames.last().unwrap();
        assert_eq!(
            last.prediction.len(),
            QuadrotorMppiConfig::default().horizon + 1
        );
        assert_eq!(last.prediction[0], last.state.position);
    }

    #[test]
    fn manual_commands_cross_all_gates_and_the_recording_reproduces_every_state() {
        let mut flight = Flight::manual(Course::CLIMB);
        // This test driver selects held keys from the next gate. It never
        // changes position, attitude, or progress outside the flight model.
        while !flight.done() {
            let current = flight.frames.last().unwrap();
            let target = flight.lap.gates[current.passed].center();
            let delta = sub(
                target,
                add(current.state.position, scale(current.state.velocity, 0.4)),
            );
            let axis = |error: f64| {
                if error > 0.25 {
                    1
                } else if error < -0.25 {
                    -1
                } else {
                    0
                }
            };
            let forward = if target[0] - current.state.position[0] < -0.5 {
                -1
            } else if target[0] - current.state.position[0] < 1.5
                && (delta[1].abs() > 0.6 || delta[2].abs() > 0.6)
            {
                0
            } else {
                1
            };
            flight
                .step_with_input(PilotInput::new(forward, -axis(delta[1]), axis(delta[2])))
                .unwrap();
        }
        assert_eq!(flight.frames.last().unwrap().status, Status::Finished);
        assert_eq!(flight.frames.last().unwrap().passed, 5);
        for pair in flight.frames.windows(2) {
            for gate in &flight.lap.gates[pair[0].passed..pair[1].passed] {
                assert!(gate.crosses(
                    pair[0].state.position,
                    pair[1].state.position,
                    flight.lap.crossing_tolerance
                ));
            }
            assert!(
                pair[1].prediction.is_empty(),
                "manual flight has no MPPI prediction"
            );
        }
        let mut replay = Flight::manual(Course::CLIMB);
        for input in manual::decode(&manual::encode(&flight.inputs)).unwrap() {
            replay.step_with_input(input).unwrap();
        }
        assert_eq!(replay.frames, flight.frames);
        assert_eq!(replay.inputs.len(), replay.last_index());
        assert!(!replay.step_with_input(PilotInput::new(-1, 1, 1)).unwrap());
        assert_eq!(replay.frames, flight.frames);
    }

    #[test]
    fn manual_hover_times_out_and_descent_stops_at_ground_contact() {
        let mut hover = Flight::manual(Course::NEON);
        for _ in 0..MAX_STEPS {
            hover.step().unwrap();
        }
        assert_eq!(hover.last_index(), MAX_STEPS);
        assert_eq!(hover.frames.last().unwrap().status, Status::TimeLimit);
        assert_eq!(hover.frames.last().unwrap().passed, 0);
        assert!(!hover.step().unwrap());
        let mut descent = Flight::manual(Course::NEON);
        while !descent.done() {
            descent.step_with_input(PilotInput::new(0, 0, -1)).unwrap();
        }
        assert_eq!(descent.frames.last().unwrap().status, Status::GroundContact);
        assert!(descent.last_index() < MAX_STEPS);
        assert_eq!(descent.frames.last().unwrap().passed, 0);
    }

    #[test]
    fn interpolation_handles_opposite_quaternion_signs_without_shrinking() {
        let a = QuadrotorState::at(0.0, 0.0, 2.0);
        let mut b = a;
        b.position = [2.0, 4.0, 6.0];
        b.attitude = [-1.0, 0.0, 0.0, 0.0];
        let mid = interpolate(a, b, 0.5);
        assert_eq!(mid.position, [1.0, 2.0, 4.0]);
        assert_eq!(mid.attitude, a.attitude);
        assert_eq!(interpolate(a, b, -1.0), a);
    }
}
