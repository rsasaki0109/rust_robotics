//! Local personal bests, keyed by the complete course geometry.

use super::flight::{Course, Flight, Status};
#[cfg(any(target_arch = "wasm32", test))]
use super::manual;
use super::manual::PilotInput;

#[cfg(target_arch = "wasm32")]
const STORAGE_KEY: &str = "rust-robotics.drone-personal-bests.v1";
const MAX_COURSES: usize = 12;
#[cfg(any(target_arch = "wasm32", test))]
const MAX_BYTES: usize = 9_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Best {
    pub course: Course,
    pub inputs: Vec<PilotInput>,
}

impl Best {
    pub fn steps(&self) -> usize {
        self.inputs.len()
    }
}

pub struct Improvement {
    pub previous_steps: Option<usize>,
}

#[derive(Default)]
pub struct RecordBook {
    entries: Vec<Best>,
    pub persistent: bool,
}

impl RecordBook {
    pub fn load() -> Self {
        #[cfg(target_arch = "wasm32")]
        if let Some(storage) = storage() {
            if let Ok(value) = storage.get_item(STORAGE_KEY) {
                return Self {
                    entries: value.as_deref().map(decode).unwrap_or_default(),
                    persistent: true,
                };
            }
        }
        Self::default()
    }

    pub fn best(&self, course: Course) -> Option<&Best> {
        self.entries.iter().find(|entry| entry.course == course)
    }

    /// The caller supplies only live manual attempts, never received replays.
    pub fn record(&mut self, flight: &Flight) -> Option<Improvement> {
        if !flight.is_manual()
            || flight.frames.last()?.status != Status::Finished
            || flight.inputs.is_empty()
            || flight.inputs.len() != flight.last_index()
        {
            return None;
        }
        // Pick up a faster result saved by another tab before writing.
        #[cfg(target_arch = "wasm32")]
        {
            let mut stored = Self::load();
            for entry in self.entries.drain(..) {
                stored.merge(entry);
            }
            self.entries = stored.entries;
        }
        let previous_steps = self.best(flight.course).map(Best::steps);
        if previous_steps.is_some_and(|steps| steps <= flight.last_index()) {
            return None;
        }
        self.entries.retain(|entry| entry.course != flight.course);
        self.entries.insert(
            0,
            Best {
                course: flight.course,
                inputs: flight.inputs.clone(),
            },
        );
        self.entries.truncate(MAX_COURSES);
        #[cfg(target_arch = "wasm32")]
        {
            self.persistent = storage().is_some_and(|storage| {
                storage
                    .set_item(STORAGE_KEY, &encode(&self.entries))
                    .is_ok()
            });
        }
        Some(Improvement { previous_steps })
    }

    #[cfg(any(target_arch = "wasm32", test))]
    fn merge(&mut self, entry: Best) {
        if self
            .best(entry.course)
            .is_some_and(|old| old.steps() <= entry.steps())
        {
            return;
        }
        self.entries.retain(|old| old.course != entry.course);
        self.entries.push(entry);
        self.entries.truncate(MAX_COURSES);
    }
}

#[cfg(target_arch = "wasm32")]
fn storage() -> Option<web_sys::Storage> {
    web_sys::window()?.local_storage().ok().flatten()
}

#[cfg(any(target_arch = "wasm32", test))]
fn encode(entries: &[Best]) -> String {
    let mut value = "v1".to_owned();
    for entry in entries.iter().take(MAX_COURSES) {
        value.push('\n');
        value.push_str(&entry.course.encode());
        value.push('\t');
        value.push_str(&manual::encode(&entry.inputs));
    }
    value
}

#[cfg(any(target_arch = "wasm32", test))]
fn decode(value: &str) -> Vec<Best> {
    if value.len() > MAX_BYTES {
        return Vec::new();
    }
    let mut rows = value.lines();
    if rows.next() != Some("v1") {
        return Vec::new();
    }
    let mut book = RecordBook::default();
    for row in rows.take(MAX_COURSES) {
        let Some((course, inputs)) = row.split_once('\t') else {
            continue;
        };
        let Some(course) = Course::decode(course) else {
            continue;
        };
        let Some(inputs) = manual::decode(inputs) else {
            continue;
        };
        // A valid encoding alone is insufficient: retain only a real finish,
        // with no commands after the terminal state. Manual physics is cheap;
        // no autonomous planning is run while loading these bounded records.
        let mut flight = Flight::manual(course);
        let valid = inputs
            .iter()
            .all(|&input| flight.step_with_input(input).unwrap_or(false));
        if valid && flight.frames[flight.last_index()].status == Status::Finished {
            book.merge(Best { course, inputs });
        }
    }
    book.entries
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    pub fn finish(course: Course, delay: usize) -> Flight {
        let mut flight = Flight::manual(course);
        for _ in 0..delay {
            flight.step().unwrap();
        }
        while !flight.done() {
            let frame = &flight.frames[flight.last_index()];
            let target = flight.lap.gates[frame.passed].center();
            let delta = super::super::flight::sub(
                target,
                super::super::flight::add(
                    frame.state.position,
                    super::super::flight::scale(frame.state.velocity, 0.4),
                ),
            );
            let axis = |value: f64| {
                if value > 0.25 {
                    1
                } else if value < -0.25 {
                    -1
                } else {
                    0
                }
            };
            let forward = if target[0] - frame.state.position[0] < -0.5 {
                -1
            } else if target[0] - frame.state.position[0] < 1.5
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
        assert_eq!(flight.frames[flight.last_index()].status, Status::Finished);
        flight
    }

    #[test]
    fn keeps_only_strict_improvements_and_complete_manual_flights() {
        let fast = finish(Course::CLIMB, 0);
        let slow = finish(Course::CLIMB, 12);
        assert!(fast.last_index() < slow.last_index());
        let mut book = RecordBook::default();
        assert_eq!(book.record(&slow).unwrap().previous_steps, None);
        assert_eq!(
            book.record(&fast).unwrap().previous_steps,
            Some(slow.last_index())
        );
        assert!(book.record(&fast).is_none());
        assert!(book.record(&slow).is_none());
        assert!(book.record(&Flight::manual(Course::NEON)).is_none());
        let mut ground = Flight::manual(Course::NEON);
        while !ground.done() {
            ground.step_with_input(PilotInput::new(0, 0, -1)).unwrap();
        }
        assert!(book.record(&ground).is_none());
        assert_eq!(book.best(Course::CLIMB).unwrap().inputs, fast.inputs);
    }

    #[test]
    fn codec_restores_the_real_flight_and_rejects_false_or_corrupt_results() {
        let flight = finish(Course::CLIMB, 0);
        let mut book = RecordBook::default();
        book.record(&flight).unwrap();
        let encoded = encode(&book.entries);
        let restored = decode(&encoded);
        assert_eq!(restored, book.entries);
        let mut replay = Flight::manual(restored[0].course);
        for &input in &restored[0].inputs {
            replay.step_with_input(input).unwrap();
        }
        assert_eq!(replay.frames, flight.frames);
        for bad in [
            encoded.replacen("v1\n", "v99\n", 1),
            format!("{encoded}a"),
            format!("v1\n{}\tv1.n", Course::CLIMB.encode()),
            format!("v1\n{}\tv99.nnn", Course::CLIMB.encode()),
            "x".repeat(MAX_BYTES + 1),
        ] {
            assert!(decode(&bad).is_empty());
        }
    }

    #[test]
    fn course_geometry_separates_records_and_duplicate_rows_keep_the_fastest() {
        let mut book = RecordBook::default();
        let course = Course::CLIMB.moved(1, [8.4, 0.2, 4.0]);
        let original = finish(Course::CLIMB, 0);
        let custom = finish(course, 0);
        book.record(&original).unwrap();
        book.record(&custom).unwrap();
        assert_eq!(book.best(Course::CLIMB).unwrap().inputs, original.inputs);
        assert_eq!(book.best(course).unwrap().inputs, custom.inputs);
        let slow = finish(Course::CLIMB, 12);
        let duplicate = format!(
            "{}\n{}\t{}",
            encode(&book.entries),
            Course::CLIMB.encode(),
            manual::encode(&slow.inputs)
        );
        assert_eq!(decode(&duplicate).len(), 2);
        assert_eq!(
            decode(&duplicate)
                .iter()
                .find(|entry| entry.course == Course::CLIMB)
                .unwrap()
                .inputs,
            original.inputs
        );
    }
}
