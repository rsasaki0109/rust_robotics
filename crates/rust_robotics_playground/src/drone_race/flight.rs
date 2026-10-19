//! Bounded, deterministic flight history shared by live playback and seeking.

use rust_robotics_control::{
    QuadrotorMppiConfig, QuadrotorMppiController, QuadrotorParams, QuadrotorState, RacingGateLap3D,
    RacingGatePlane3D,
};
use rust_robotics_core::RoboticsResult;

pub const DT: f64 = 0.05;
pub const MAX_STEPS: usize = 600;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Course {
    #[default]
    Neon,
    Climb,
}

impl Course {
    pub const ALL: [Self; 2] = [Self::Neon, Self::Climb];

    pub fn slug(self) -> &'static str {
        match self {
            Self::Neon => "neon",
            Self::Climb => "climb",
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            Self::Neon => "Neon slalom",
            Self::Climb => "Sky climb",
        }
    }

    pub fn from_slug(value: &str) -> Option<Self> {
        match value {
            "neon" => Some(Self::Neon),
            "climb" => Some(Self::Climb),
            _ => None,
        }
    }

    pub fn lap(self) -> RacingGateLap3D {
        let centers = match self {
            Self::Neon => vec![
                [4.0, 1.2, 2.4],
                [8.0, -1.2, 3.4],
                [12.0, 1.2, 2.5],
                [16.0, -1.2, 3.4],
                [20.0, 0.0, 2.4],
            ],
            Self::Climb => vec![
                [4.0, 0.0, 2.8],
                [8.0, 0.0, 4.0],
                [12.0, 0.8, 5.2],
                [16.0, -0.6, 5.6],
                [20.0, 0.0, 4.0],
            ],
        };
        let mut previous = [0.0, 0.0, 2.0];
        let gates = centers
            .into_iter()
            .map(|center| {
                let direction = sub(center, previous);
                previous = center;
                RacingGatePlane3D::new(center, direction, [0.0, 0.0, 1.0], 0.9, 0.9)
                    .expect("built-in race gate is valid")
            })
            .collect();
        RacingGateLap3D::open(gates).expect("built-in race has gates")
    }
}

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
    controller: QuadrotorMppiController,
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
            controller,
            params,
        }
    }

    pub fn last_index(&self) -> usize {
        self.frames.len() - 1
    }

    pub fn done(&self) -> bool {
        self.frames[self.last_index()].status != Status::Flying
    }

    /// Append exactly one physical update. A terminal history never grows.
    pub fn step(&mut self) -> RoboticsResult<bool> {
        if self.done() {
            return Ok(false);
        }
        let current = &self.frames[self.last_index()];
        let plan = self
            .controller
            .plan(current.state, &self.lap, current.passed)?;
        let state = self.params.step(current.state, plan.control, DT);
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
            prediction: self.controller.predicted_positions(state),
        });
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
    fn both_courses_finish_by_actual_ordered_gate_crossings() {
        for course in Course::ALL {
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
        let (mut a, mut b) = (Flight::new(Course::Neon), Flight::new(Course::Neon));
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
