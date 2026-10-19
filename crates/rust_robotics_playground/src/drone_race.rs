//! Interactive 3-D quadrotor race, with bounded incremental replay generation.

mod checkpoint;
mod course;
mod editor;
mod flight;
mod guide;
mod manual;
mod pad;
mod photo;
mod records;
mod renderer;
mod results;
mod start;

use editor::{CourseEditor, EditorAction};
use egui::{Color32, FontId, Rect, RichText, Stroke, Vec2};
use flight::{Course, Flight, Status, DT, MAX_STEPS};
use manual::PilotInput;
use photo::PhotoMode;
use renderer::{CameraMode, GhostView, ViewOptions, CYAN, GHOST, GOLD};
use web_time::Instant;

pub struct DroneRaceDemo {
    flight: Flight,
    editor: Option<CourseEditor>,
    photo: Option<PhotoMode>,
    guide: bool,
    menu_open: bool,
    records: records::RecordBook,
    new_best: Option<records::Improvement>,
    best_checked: bool,
    // The autonomous cache also supplies Watch autopilot. Keep a separate,
    // immutable best-flight snapshot for the duration of each attempt.
    ghost: Option<Flight>,
    best_ghost: Option<Flight>,
    race_best: bool,
    recording: Option<Vec<PilotInput>>,
    input: PilotInput,
    pad: pad::FlightPad,
    show_ghost: bool,
    has_started: bool,
    start: Option<start::StartSequence>,
    cursor: usize,
    seek: Option<usize>,
    playing: bool,
    phase: f64,
    last_time: Option<f64>,
    camera: CameraMode,
    trail: bool,
    prediction: bool,
    slow: bool,
    cinema: bool,
    last_plan_ms: f64,
    error: Option<String>,
}

impl Default for DroneRaceDemo {
    fn default() -> Self {
        Self {
            flight: Flight::new(Course::NEON),
            editor: None,
            photo: None,
            guide: false,
            menu_open: false,
            records: records::RecordBook::load(),
            new_best: None,
            best_checked: false,
            ghost: None,
            best_ghost: None,
            race_best: true,
            recording: None,
            input: PilotInput::default(),
            pad: pad::FlightPad::default(),
            show_ghost: true,
            has_started: false,
            start: None,
            cursor: 0,
            seek: None,
            playing: true,
            phase: 0.0,
            last_time: None,
            camera: CameraMode::Chase,
            trail: true,
            prediction: true,
            slow: false,
            cinema: false,
            last_plan_ms: 0.0,
            error: None,
        }
    }
}

impl DroneRaceDemo {
    pub fn apply_share_query(&mut self, query: &str) {
        if crate::share::value(query, "tab") != Some("drone") {
            return;
        }
        let preset = crate::share::value(query, "course")
            .and_then(Course::from_slug)
            .unwrap_or(Course::NEON);
        let encoded = crate::share::value(query, "gates");
        let custom = encoded.and_then(Course::decode);
        let invalid_course = (encoded.is_some() && custom.is_none())
            || (crate::share::value(query, "course") == Some("custom") && encoded.is_none());
        let course = custom.unwrap_or(preset);
        let manual = crate::share::value(query, "mode") == Some("manual");
        self.reset_course(course, manual);
        if invalid_course {
            self.error = Some(
                "This course link is invalid. Choose Retry to start a built-in course.".to_string(),
            );
        }
        if manual && !invalid_course {
            if let Some(encoded) = crate::share::value(query, "inputs") {
                match manual::decode(encoded) {
                    Some(inputs) => self.recording = Some(inputs),
                    None => {
                        self.error = Some(
                            "This flight recording is invalid. Choose New attempt to fly."
                                .to_string(),
                        );
                    }
                }
            }
            // Older replay links always raced the autopilot. A best rival is
            // carried by the link itself, independent of the receiver's store.
            self.race_best = crate::share::value(query, "rival")
                .map_or(self.recording.is_none(), |rival| rival == "best");
            if crate::share::value(query, "rival") == Some("best") {
                self.best_ghost = crate::share::value(query, "best_inputs")
                    .and_then(manual::decode)
                    .and_then(|inputs| records::Best { course, inputs }.flight());
                if self.best_ghost.is_none() {
                    self.error = Some(
                        "This best-flight ghost is invalid. Choose New attempt to fly.".to_owned(),
                    );
                }
            }
        }
        if let Some(camera) = crate::share::value(query, "camera").and_then(CameraMode::from_slug) {
            self.camera = camera;
        }
        self.trail = crate::share::boolean(query, "trail").unwrap_or(true);
        self.prediction = crate::share::boolean(query, "prediction").unwrap_or(true);
        self.show_ghost = crate::share::boolean(query, "ghost").unwrap_or(true);
        self.slow = crate::share::boolean(query, "slow").unwrap_or(false);
        self.cinema = crate::share::boolean(query, "cinema").unwrap_or(false);
        self.playing =
            crate::share::boolean(query, "playing").unwrap_or(!manual) && self.error.is_none();
        self.has_started = self.playing;
        let frame = if self.error.is_some() {
            0
        } else {
            crate::share::bounded_usize(query, "frame", MAX_STEPS).unwrap_or(0)
        };
        self.seek_to(frame);
        if self.playing && self.fresh_manual() {
            self.begin_start();
        }
        if self.error.is_none() && crate::share::boolean(query, "edit").unwrap_or(false) {
            self.begin_editor();
        } else if self.error.is_none() && crate::share::boolean(query, "photo").unwrap_or(false) {
            self.begin_photo();
            if let Some(photo) = &mut self.photo {
                photo.apply_query(query);
            }
        } else if frame == 0
            && self.recording.is_none()
            && crate::share::boolean(query, "intro").unwrap_or(false)
        {
            self.begin_guide();
        }
    }

    pub fn share_query(&self) -> String {
        let course = self
            .editor
            .as_ref()
            .map_or(self.flight.course, CourseEditor::course);
        let mut query = format!(
            "tab=drone&course={}&camera={}&frame={}&playing={}&trail={}&prediction={}&slow={}&cinema={}&mode={}&ghost={}&edit={}",
            course.slug(),
            self.camera.slug(),
            if self.editor.is_some() { 0 } else { self.photo.as_ref().map_or(self.seek.unwrap_or(self.cursor), |p| p.frame) },
            u8::from(self.playing && !self.counting_down()),
            u8::from(self.photo.as_ref().map_or(self.trail, |p| p.trail)),
            u8::from(self.prediction),
            u8::from(self.slow),
            u8::from(self.cinema),
            if self.flight.is_manual() { "manual" } else { "auto" },
            u8::from(self.photo.as_ref().map_or(self.show_ghost, |p| p.ghost)),
            u8::from(self.editor.is_some())
        );
        if course.is_custom() {
            query.push_str("&gates=");
            query.push_str(&course.encode());
        }
        if self.flight.is_manual() && self.editor.is_none() {
            let inputs = self.recording.as_deref().unwrap_or(&self.flight.inputs);
            if !inputs.is_empty() {
                query.push_str("&inputs=");
                query.push_str(&manual::encode(inputs));
            }
            if self.racing_best() {
                query.push_str("&rival=best&best_inputs=");
                query.push_str(&manual::encode(&self.best_ghost.as_ref().unwrap().inputs));
            } else {
                query.push_str("&rival=auto");
            }
        }
        if let Some(photo) = &self.photo {
            query.push_str(&photo.query());
        }
        query
    }

    pub fn cinema(&self) -> bool {
        self.cinema || self.editor.is_some() || self.photo.is_some() || self.guide
    }

    fn begin_guide(&mut self) {
        if self.editor.is_none()
            && self.photo.is_none()
            && self.seek.is_none()
            && self.error.is_none()
        {
            self.guide = true;
            self.menu_open = false;
            self.playing = false;
            self.reset_clock();
        }
    }

    fn finish_guide(&mut self, action: guide::Action) {
        if action == guide::Action::Open {
            return;
        }
        self.guide = false;
        match action {
            guide::Action::Fly => {
                self.choose_pilot(true);
                self.playing = false;
                if self.cursor == 0 {
                    self.has_started = false;
                }
            }
            guide::Action::Watch => {
                self.choose_pilot(false);
                if self.flight.frames[self.cursor].status != Status::Flying {
                    self.replay_run();
                } else {
                    self.playing = true;
                    self.has_started = true;
                }
            }
            guide::Action::Build => self.begin_editor(),
            guide::Action::Open | guide::Action::Close => {}
        }
        self.reset_clock();
    }

    fn begin_photo(&mut self) {
        if self.editor.is_none() && self.photo.is_none() && self.error.is_none() {
            self.menu_open = false;
            self.photo = Some(PhotoMode::new(
                self.seek.unwrap_or(self.cursor),
                self.trail,
                self.show_ghost,
            ));
            self.playing = false;
            self.reset_clock();
        }
    }

    fn close_photo(&mut self) {
        self.photo = None;
        self.reset_clock();
    }

    fn begin_editor(&mut self) {
        if self.editor.is_none() {
            self.menu_open = false;
            self.editor = Some(CourseEditor::new(self.flight.course));
            self.playing = false;
            self.seek = None;
            self.reset_clock();
        }
    }

    fn finish_editor(&mut self, action: EditorAction) {
        match action {
            EditorAction::Editing => {}
            EditorAction::Cancel => {
                self.editor = None;
                self.reset_clock();
            }
            EditorAction::Apply(course) => {
                let manual = self.flight.is_manual();
                self.reset_course(course, manual);
            }
        }
    }

    fn reset_clock(&mut self) {
        self.start = None;
        self.phase = 0.0;
        self.last_time = None;
        self.input = PilotInput::default();
        self.pad.clear();
    }

    fn reset_course(&mut self, course: Course, manual: bool) {
        self.new_best = None;
        self.best_checked = false;
        self.editor = None;
        self.photo = None;
        self.guide = false;
        self.menu_open = false;
        self.flight = if manual {
            Flight::manual(course)
        } else {
            Flight::new(course)
        };
        self.ghost = manual.then(|| Flight::new(course));
        self.capture_best();
        self.recording = None;
        self.cursor = 0;
        self.seek = None;
        self.error = None;
        self.playing = !manual;
        self.has_started = false;
        self.reset_clock();
    }

    fn choose_pilot(&mut self, manual: bool) {
        if self.flight.is_manual() == manual {
            return;
        }
        self.new_best = None;
        self.best_checked = false;
        let course = self.flight.course;
        if manual {
            self.ghost = Some(std::mem::replace(&mut self.flight, Flight::manual(course)));
        } else {
            self.flight = self.ghost.take().unwrap_or_else(|| Flight::new(course));
        }
        self.capture_best();
        self.recording = None;
        self.cursor = 0;
        self.seek = None;
        self.error = None;
        self.playing = !manual;
        self.has_started = false;
        self.reset_clock();
    }

    fn seek_to(&mut self, target: usize) {
        let end = if self.flight.is_manual() {
            self.recording
                .as_ref()
                .map_or(self.flight.last_index(), Vec::len)
        } else {
            MAX_STEPS
        };
        let target = target.min(end);
        if target <= self.flight.last_index() || self.flight.done() {
            self.cursor = target.min(self.flight.last_index());
            self.seek = None;
        } else {
            self.seek = Some(target);
        }
        self.phase = 0.0;
        self.last_time = None;
    }

    fn rewind(&mut self) {
        self.new_best = None;
        self.best_checked = false;
        if self.flight.is_manual() {
            self.flight = Flight::manual(self.flight.course);
            self.capture_best();
            self.recording = None;
            self.error = None;
        } else if self.error.is_some() {
            self.flight = Flight::new(self.flight.course);
            self.error = None;
        }
        self.seek_to(0);
        self.playing = !self.flight.is_manual();
        self.has_started = false;
        self.reset_clock();
    }

    fn replay_run(&mut self) {
        self.new_best = None;
        if self.flight.is_manual()
            && self
                .recording
                .as_deref()
                .unwrap_or(&self.flight.inputs)
                .is_empty()
        {
            return;
        }
        if self.flight.is_manual() && self.recording.is_none() {
            self.recording = Some(self.flight.inputs.clone());
        }
        self.seek_to(0);
        self.playing = self
            .recording
            .as_ref()
            .map_or(true, |inputs| !inputs.is_empty());
        self.has_started = true;
        self.reset_clock();
    }

    fn toggle_playing(&mut self) {
        if self.error.is_some() {
            return;
        }
        self.has_started = true;
        if self.flight.frames[self.cursor].status != Status::Flying || self.replay_finished() {
            self.replay_run();
        } else {
            let start = !self.playing && self.fresh_manual();
            self.playing = !self.playing;
            self.reset_clock();
            if start {
                self.begin_start();
            }
        }
    }

    fn fresh_manual(&self) -> bool {
        self.flight.is_manual()
            && self.recording.is_none()
            && self.cursor == 0
            && self.flight.last_index() == 0
            && self.seek.is_none()
            && self.error.is_none()
    }

    fn counting_down(&self) -> bool {
        self.start
            .as_ref()
            .is_some_and(start::StartSequence::waiting)
    }

    fn begin_start(&mut self) {
        self.start = Some(start::StartSequence::default());
        self.has_started = true;
        self.playing = true;
        self.phase = 0.0;
        self.last_time = None;
        // Keep held keys and touch contacts: the pilot can line up their
        // controls during the countdown and carry them across GO.
    }

    fn replay_finished(&self) -> bool {
        self.recording
            .as_ref()
            .is_some_and(|inputs| self.cursor >= inputs.len())
    }

    fn step_flight(&mut self) -> bool {
        let begin = Instant::now();
        let input = if let Some(inputs) = &self.recording {
            let Some(input) = inputs.get(self.flight.last_index()) else {
                self.playing = false;
                return false;
            };
            *input
        } else {
            self.input
        };
        // The opponent uses this exact simulation tick. It may be cached
        // from an earlier attempt, but its visible clock never runs ahead.
        let result = (|| {
            if let Some(ghost) = self
                .ghost
                .as_mut()
                .filter(|_| !self.race_best || self.best_ghost.is_none())
            {
                if ghost.last_index() <= self.flight.last_index() && !ghost.done() {
                    ghost.step()?;
                }
            }
            self.flight.step_with_input(input)
        })();
        self.last_plan_ms = begin.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(stepped) => stepped,
            Err(error) => {
                self.error = Some(error.to_string());
                self.playing = false;
                self.seek = None;
                false
            }
        }
    }

    fn advance(&mut self, now: f64) {
        let countdown_elapsed = self.last_time.map_or(0.0, |last| {
            let delta = now - last;
            // Use foreground elapsed time for the start lights, independently
            // of the bounded physics budget. Slow painting must not stretch
            // three seconds into dozens of simulated updates.
            if (0.0..1.0).contains(&delta) {
                delta
            } else {
                0.0
            }
        });
        let elapsed = self.last_time.map_or(0.0, |last| {
            let delta = now - last;
            // Changing tabs or restoring a background browser never creates
            // an unbounded catch-up loop.
            if (0.0..0.25).contains(&delta) {
                delta.min(0.1)
            } else {
                0.0
            }
        });
        self.last_time = Some(now);
        if self.error.is_some() {
            return;
        }
        if let Some(target) = self.seek {
            let begin = Instant::now();
            // At most two controller calls per paint. A long share-link seek
            // yields to the browser between batches, even on slow devices.
            for _ in 0..2 {
                if self.flight.last_index() >= target || self.flight.done() {
                    break;
                }
                if !self.step_flight() || begin.elapsed().as_secs_f64() >= 0.008 {
                    break;
                }
            }
            if self.flight.last_index() >= target || self.flight.done() {
                self.cursor = target.min(self.flight.last_index());
                self.seek = None;
            }
            self.phase = 0.0;
            return;
        }
        if !self.playing {
            return;
        }
        if let Some(start) = &mut self.start {
            let waiting = start.waiting();
            start.advance(countdown_elapsed);
            if start.finished() {
                self.start = None;
            }
            if waiting {
                self.phase = 0.0;
                return;
            }
        }
        if self.flight.frames[self.cursor].status != Status::Flying {
            self.playing = false;
            return;
        }
        self.phase = (self.phase + elapsed * if self.slow { 0.5 } else { 1.0 }).min(2.0 * DT);
        for _ in 0..2 {
            if self.phase < DT {
                break;
            }
            if self.cursor == self.flight.last_index() && !self.step_flight() {
                break;
            }
            if self.cursor < self.flight.last_index() {
                self.cursor += 1;
                self.phase -= DT;
            }
            if self.flight.frames[self.cursor].status != Status::Flying {
                self.playing = false;
                self.phase = 0.0;
                crate::engagement::track("drone_race_completed");
                break;
            }
            if self.replay_finished() {
                self.playing = false;
                self.phase = 0.0;
                break;
            }
        }
        self.record_finish();
    }

    fn record_finish(&mut self) {
        if !self.best_checked
            && self.flight.is_manual()
            && self.recording.is_none()
            && self.seek.is_none()
            && self.error.is_none()
            && self.flight.done()
            && self.cursor == self.flight.last_index()
        {
            self.best_checked = true;
            self.new_best = self.records.record(&self.flight);
        }
    }

    fn replay_best(&mut self) {
        let Some(best) = self.records.best(self.flight.course).cloned() else {
            return;
        };
        self.choose_pilot(true);
        self.rewind();
        self.recording = Some(best.inputs);
        self.playing = true;
        self.has_started = true;
    }

    fn capture_best(&mut self) {
        self.best_ghost = self
            .flight
            .is_manual()
            .then(|| {
                self.records
                    .best(self.flight.course)
                    .and_then(records::Best::flight)
            })
            .flatten();
    }

    fn racing_best(&self) -> bool {
        self.flight.is_manual() && self.race_best && self.best_ghost.is_some()
    }

    fn rival(&self) -> Option<&Flight> {
        if self.racing_best() {
            self.best_ghost.as_ref()
        } else {
            self.ghost.as_ref()
        }
    }

    fn rival_color(&self) -> Color32 {
        if self.racing_best() {
            GOLD
        } else {
            GHOST
        }
    }

    fn choose_rival(&mut self) {
        self.race_best = !self.racing_best();
        self.rewind();
    }

    /// Difference at the latest gate both flights actually crossed. This is a
    /// checkpoint split, not a distance converted to an estimated finish time.
    fn best_split(&self) -> Option<(usize, f64)> {
        self.best_split_at(self.cursor)
    }

    fn best_split_at(&self, cursor: usize) -> Option<(usize, f64)> {
        if !self.racing_best() {
            return None;
        }
        let gate = self.flight.frames[cursor].passed;
        if gate == 0 {
            return None;
        }
        let own = self.flight.frames[..=cursor]
            .iter()
            .position(|frame| frame.passed >= gate)?;
        let best = self
            .best_ghost
            .as_ref()?
            .frames
            .iter()
            .position(|frame| frame.passed >= gate)?;
        Some((gate, (own as f64 - best as f64) * DT))
    }

    fn rival_score(&self) -> String {
        let passed = self.flight.frames[self.cursor].passed;
        self.rival().map_or_else(
            || format!("GATES {passed}/5"),
            |ghost| {
                let label = if self.racing_best() { "BEST" } else { "AUTO" };
                let split = self
                    .best_split()
                    .map_or_else(String::new, |(_, delta)| format!("  {delta:+.2}s"));
                format!(
                    "YOU {passed}/5  /  {label} {}/5{split}",
                    ghost.frames[self.cursor.min(ghost.last_index())].passed
                )
            },
        )
    }

    fn finish_ghost(&mut self) {
        if self.racing_best() {
            return;
        }
        if self.flight.frames[self.cursor].status == Status::Flying && !self.replay_finished() {
            return;
        }
        let Some(ghost) = &mut self.ghost else {
            return;
        };
        let begin = Instant::now();
        for _ in 0..2 {
            if ghost.done() {
                break;
            }
            if let Err(error) = ghost.step() {
                self.error = Some(error.to_string());
                break;
            }
            if begin.elapsed().as_secs_f64() >= 0.008 {
                break;
            }
        }
    }

    pub fn controls(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "WHO'S FLYING?");
        let mut manual = self.flight.is_manual();
        ui.horizontal(|ui| {
            ui.selectable_value(&mut manual, false, "Autopilot");
            ui.selectable_value(&mut manual, true, "You fly");
        });
        self.choose_pilot(manual);
        if manual {
            if let Some(best) = self.records.best(self.flight.course) {
                crate::ui_kit::section(ui, "YOUR PERSONAL BEST");
                ui.colored_label(
                    GOLD,
                    format!("{:.2} s · five gates", best.steps() as f64 * DT),
                );
                crate::ui_kit::hint(
                    ui,
                    if self.records.persistent {
                        "Saved in this browser for this course."
                    } else {
                        "Saved for this session for this course."
                    },
                );
                if ui.button("Replay best").clicked() {
                    self.replay_best();
                }
            }
        }
        if manual {
            ui.label(RichText::new("Your drone. Same five gates.").color(CYAN));
            crate::ui_kit::hint(ui, "Hold a flight control or press Play for 3-2-1. Keep holding through GO. Space / Pause cancels the countdown.");
            crate::ui_kit::hint(ui, "W / S: forward / back · A / D: left / right\nE / Q: climb / descend. Arrow keys also steer.");
            crate::ui_kit::hint(ui, "Hold the screen buttons on a phone. Release to brake; the assist keeps the aircraft level. Fly forward through each numbered gate in order.");
            ui.checkbox(&mut self.show_ghost, "Show rival ghost");
            if self.best_ghost.is_some() || self.records.best(self.flight.course).is_some() {
                if ui
                    .button(if self.racing_best() {
                        "Race autopilot"
                    } else {
                        "Race your best"
                    })
                    .clicked()
                {
                    self.choose_rival();
                }
                crate::ui_kit::hint(ui, "Changing the rival starts a new attempt.");
            }
        }
        crate::ui_kit::section(ui, "THE CIRCUIT");
        let mut course = self.flight.course;
        for candidate in Course::ALL {
            ui.selectable_value(&mut course, candidate, candidate.title());
        }
        if course != self.flight.course {
            self.reset_course(course, manual);
        }
        if self.flight.course.is_custom() {
            ui.colored_label(CYAN, "Your circuit / 5 gates");
        }
        let edit = ui.button("Edit course");
        self.publish_editor_entry(edit.rect);
        if edit.clicked() {
            self.begin_editor();
            ctx.request_repaint();
        }
        ui.add_space(8.0);
        if !manual {
            ui.label(RichText::new("Five gates. One autonomous pilot.").color(CYAN));
            crate::ui_kit::hint(
                ui,
                "Watch the quadrotor lean into turns and climb through the course.",
            );
        }

        crate::ui_kit::section(ui, "CAMERA");
        ui.horizontal(|ui| {
            for camera in CameraMode::ALL {
                ui.selectable_value(&mut self.camera, camera, camera.title());
            }
        });
        crate::ui_kit::hint(
            ui,
            match self.camera {
                CameraMode::Chase => "Follow the drone from behind.",
                CameraMode::Pilot => "See through the drone's eyes, including its pitch and roll.",
                CameraMode::Course => "Watch the entire circuit from the grandstand.",
            },
        );

        if ui.button("Cinema view").clicked() {
            self.cinema = true;
        }
        let photo = ui.add_enabled(
            self.seek.is_none() && self.error.is_none(),
            egui::Button::new("Photo mode"),
        );
        self.publish_photo_entry(photo.rect);
        if photo.clicked() {
            self.begin_photo();
            ctx.request_repaint();
        }
        let help = ui.add_enabled(
            self.seek.is_none() && self.error.is_none(),
            egui::Button::new("How to play"),
        );
        self.publish_guide_entry(help.rect);
        if help.clicked() {
            self.begin_guide();
            ctx.request_repaint();
        }

        crate::ui_kit::section(ui, "PLAYBACK");
        ui.horizontal(|ui| {
            if ui
                .button(if self.playing {
                    "Pause"
                } else if manual && self.recording.is_none() && self.cursor == 0 {
                    "Start race"
                } else {
                    "Play"
                })
                .clicked()
            {
                self.toggle_playing();
            }
            if ui
                .add_enabled(
                    !manual
                        || !self.flight.inputs.is_empty()
                        || self.recording.as_ref().is_some_and(|r| !r.is_empty()),
                    egui::Button::new("Replay"),
                )
                .clicked()
            {
                self.replay_run();
            }
            if !manual && ui.button("Step").clicked() {
                self.playing = false;
                self.seek_to(self.cursor + 1);
            }
        });
        if manual && ui.button("New attempt").clicked() {
            self.rewind();
        }
        ui.checkbox(&mut self.slow, "Slow motion");
        let mut frame = self.seek.unwrap_or(self.cursor);
        // Full-range seeking builds only the needed prefix, incrementally.
        let end = if manual {
            self.recording
                .as_ref()
                .map_or(self.flight.last_index(), Vec::len)
        } else if self.flight.done() {
            self.flight.last_index()
        } else {
            MAX_STEPS
        };
        if ui
            .add(egui::Slider::new(&mut frame, 0..=end).text("Frame"))
            .changed()
        {
            self.playing = false;
            if manual && self.recording.is_none() {
                self.recording = Some(self.flight.inputs.clone());
            }
            self.seek_to(frame);
        }
        if let Some(target) = self.seek {
            ui.label(format!(
                "Preparing replay · {} / {target}",
                self.flight.last_index()
            ));
        }
        crate::ui_kit::hint(
            ui,
            if manual {
                "Space: start / pause · R: new attempt · 1 / 2 / 3: camera · P: photo"
            } else {
                "Space: pause · R: replay · 1 / 2 / 3: camera · P: photo"
            },
        );

        crate::ui_kit::section(ui, "FLIGHT LINES");
        ui.checkbox(&mut self.trail, "Flight trail");
        if !manual {
            ui.checkbox(&mut self.prediction, "Predicted flight");
        }
        ui.horizontal(|ui| {
            for (color, label) in if manual {
                [
                    (CYAN, "You"),
                    (
                        self.rival_color(),
                        if self.racing_best() {
                            "Best flight"
                        } else {
                            "Autopilot"
                        },
                    ),
                ]
            } else {
                [(CYAN, "Flown"), (GOLD, "Predicted")]
            } {
                let (line, _) = ui.allocate_exact_size(Vec2::new(18.0, 10.0), egui::Sense::hover());
                ui.painter().line_segment(
                    [line.left_center(), line.right_center()],
                    Stroke::new(2.0_f32, color),
                );
                ui.colored_label(color, label);
            }
        });
        ui.add_space(12.0);
        ui.collapsing("Inside the pilot", |ui| {
            if manual {
                ui.label("Your keys request velocity. A flight assist converts that request into thrust and body rates; position and attitude still follow the same quadrotor physics as the opponent.");
                ui.label("A gold ghost replays your best flight for this exact course. Without a saved best, the purple ghost is the seeded autopilot. Both use your simulation clock and cannot collide with you. The signed split compares gate crossing times: negative means you crossed sooner.");
                ui.label("Share includes both flights' recorded inputs, camera and current frame. The receiving browser replays the same rival without saving a personal best; choose New attempt to take control.");
            }
            ui.label("MPPI samples 900 possible command sequences, each 1.4 seconds long, before every flight update.");
            ui.label("The quadrotor accelerates along its own thrust axis: it must tilt to turn. The dotted line is the controller's remaining predicted horizon.");
            ui.label(format!("Latest planning call: {:.1} ms", self.last_plan_ms));
            ui.label("Fixed seed: 7 · flight step: 0.05 s");
            ui.label("Replay speed and camera do not affect physics. A flight stops at ground contact or after 30 simulated seconds.");
        });
        if let Some(error) = &self.error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
    }

    pub fn scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        if self.guide {
            let action = guide::show(
                ctx,
                ui,
                &self.flight,
                self.cursor,
                self.rival(),
                self.recording.is_some(),
            );
            self.finish_guide(action);
            self.publish_status();
            if action != guide::Action::Open {
                ctx.request_repaint();
            }
            return;
        }
        if self.photo.is_some() {
            if self.seek.is_some() {
                self.advance(ctx.input(|input| input.time));
                if self.seek.is_none() {
                    self.photo.as_mut().unwrap().frame = self.cursor;
                }
            }
            if self.error.is_some() {
                self.close_photo();
            } else {
                if self.seek.is_some() {
                    ui.centered_and_justified(|ui| {
                        ui.colored_label(CYAN, "PREPARING SHARED PHOTO")
                    });
                    ctx.request_repaint();
                } else {
                    let ghost = if self.race_best {
                        self.best_ghost.as_ref().or(self.ghost.as_ref())
                    } else {
                        self.ghost.as_ref()
                    };
                    let close = egui::Frame::new()
                        .inner_margin(12)
                        .show(ui, |ui| {
                            self.photo
                                .as_mut()
                                .unwrap()
                                .show(ctx, ui, &self.flight, ghost)
                        })
                        .inner;
                    if close {
                        self.close_photo();
                        ctx.request_repaint();
                    }
                }
                self.publish_status();
                return;
            }
        }
        if let Some(editor) = &mut self.editor {
            let action = egui::Frame::new()
                .inner_margin(12)
                .show(ui, |ui| editor.show(ctx, ui))
                .inner;
            let finished = !matches!(action, EditorAction::Editing);
            self.finish_editor(action);
            self.publish_status();
            if finished {
                ctx.request_repaint();
            }
            return;
        }
        let shortcuts = !ctx.wants_keyboard_input()
            && !self.menu_open
            && !ctx.memory(|memory| memory.any_popup_open());
        if self.menu_open && ctx.input(|input| input.key_pressed(egui::Key::Escape)) {
            ctx.request_repaint();
        }
        if shortcuts
            && ctx.input(|input| input.key_pressed(egui::Key::H))
            && self.seek.is_none()
            && self.error.is_none()
        {
            self.begin_guide();
            ctx.request_repaint();
            return;
        }
        if shortcuts
            && ctx.input(|input| input.key_pressed(egui::Key::P))
            && self.seek.is_none()
            && self.error.is_none()
        {
            self.begin_photo();
            ctx.request_repaint();
            return;
        }
        if shortcuts {
            ctx.input(|input| {
                if input.key_pressed(egui::Key::Space) {
                    self.toggle_playing();
                }
                if input.key_pressed(egui::Key::R) {
                    self.rewind();
                }
                if input.key_pressed(egui::Key::Escape) {
                    self.cinema = false;
                }
                for (key, camera) in [
                    (egui::Key::Num1, CameraMode::Chase),
                    (egui::Key::Num2, CameraMode::Pilot),
                    (egui::Key::Num3, CameraMode::Course),
                ] {
                    if input.key_pressed(key) {
                        self.camera = camera;
                    }
                }
            });
        }
        let available = ui.available_size();
        let height = if self.cinema {
            available.y.max(200.0)
        } else if ctx.screen_rect().width() < 700.0 {
            if self.flight.is_manual() {
                (available.x * 1.5).clamp(540.0, 620.0)
            } else {
                (available.x * 0.95).clamp(300.0, 520.0)
            }
        } else {
            available.y.max(300.0)
        };
        let (rect, _) = ui.allocate_exact_size(
            Vec2::new(available.x.max(120.0), height),
            egui::Sense::click(),
        );
        if !self.cinema || !pad::compact(rect) {
            self.menu_open = false;
        }
        if self.flight.is_manual() {
            if self
                .start
                .as_ref()
                .is_some_and(start::StartSequence::interrupted)
                || ctx.input(|input| {
                    !input.focused
                        || input
                            .events
                            .iter()
                            .any(|event| matches!(event, egui::Event::WindowFocused(false)))
                })
            {
                self.playing = false;
                self.reset_clock();
            }
            self.input = self.pad.read(
                ctx,
                rect,
                self.recording.is_none()
                    && self.seek.is_none()
                    && !self.flight.done()
                    && self.error.is_none()
                    && !self.menu_open,
            );
            if self.cursor == 0
                && !self.has_started
                && self.input.active()
                && self.recording.is_none()
                && self.error.is_none()
            {
                self.begin_start();
            }
        }
        let was_playing = self.playing;
        let was_seeking = self.seek.is_some();
        self.advance(ctx.input(|input| input.time));
        if !was_playing
            && !was_seeking
            && self.seek.is_none()
            && self.flight.last_index() > 0
            && self.error.is_none()
        {
            self.finish_ghost();
        }
        let painter = ui.painter_at(rect);
        let rival_action = (self.flight.is_manual()
            && (self.best_ghost.is_some() || self.records.best(self.flight.course).is_some()))
        .then_some(if self.racing_best() {
            "Race autopilot"
        } else {
            "Race your best"
        });
        let current = &self.flight.frames[self.cursor];
        let (draw_frame, state, visual_time) =
            if self.playing && self.cursor > 0 && self.seek.is_none() {
                let previous = self.cursor - 1;
                let alpha = self.phase / DT;
                (
                    previous,
                    flight::interpolate(self.flight.frames[previous].state, current.state, alpha),
                    (previous as f64 + alpha) * DT,
                )
            } else {
                (self.cursor, current.state, self.cursor as f64 * DT)
            };
        renderer::render(
            &painter,
            rect,
            &self.flight,
            draw_frame,
            state,
            visual_time,
            &ViewOptions {
                camera: self.camera,
                orbit: None,
                trail: self.trail,
                prediction: self.prediction && !self.flight.is_manual(),
                edit_gate: None,
                ghost: self.rival().filter(|_| self.show_ghost).map(|ghost| {
                    let frame = draw_frame.min(ghost.last_index());
                    let next = (frame + 1).min(ghost.last_index());
                    GhostView {
                        flight: ghost,
                        frame,
                        state: flight::interpolate(
                            ghost.frames[frame].state,
                            ghost.frames[next].state,
                            visual_time / DT - frame as f64,
                        ),
                    }
                }),
            },
        );
        self.hud(&painter, rect, state.speed(), visual_time);
        if current.status == Status::Flying && self.error.is_none() {
            if let Some(checkpoint) =
                checkpoint::Checkpoint::at(&self.flight, draw_frame, visual_time)
            {
                checkpoint.paint(
                    &painter,
                    rect,
                    self.flight.lap.gate_count(),
                    self.best_split_at(draw_frame).map(|(_, split)| split),
                );
            }
        }
        let mut switch_pilot = false;
        let mut edit_course = false;
        let mut take_photo = false;
        let mut show_guide = false;
        let mut replay_best = false;
        let mut switch_rival = false;
        if self.cinema && pad::compact(rect) {
            let help = Rect::from_min_size(
                rect.right_top() + Vec2::new(-228.0, 12.0),
                Vec2::new(108.0, 28.0),
            );
            show_guide = ui
                .put(
                    help,
                    egui::Button::new(RichText::new("How to play").size(11.0)),
                )
                .clicked();
            self.publish_guide_entry(help);
            let menu = Rect::from_min_size(
                rect.right_top() + Vec2::new(-114.0, 12.0),
                Vec2::new(98.0, 28.0),
            );
            let result = ui
                .scope_builder(egui::UiBuilder::new().max_rect(menu), |ui| {
                    let response = ui
                        .menu_button("Options", |ui| {
                            for camera in CameraMode::ALL {
                                let choice =
                                    ui.selectable_value(&mut self.camera, camera, camera.title());
                                Self::publish_menu_camera(camera, choice.rect);
                                if choice.clicked() {
                                    ui.close_menu();
                                }
                            }
                            ui.separator();
                            if let Some(label) = rival_action {
                                let choice = ui.button(label);
                                Self::publish_rival_button(choice.rect);
                                if choice.clicked() {
                                    switch_rival = true;
                                    ui.close_menu();
                                }
                            }
                            if ui.button("Photo mode").clicked() {
                                take_photo = true;
                                ui.close_menu();
                            }
                            if ui.button("Edit course").clicked() {
                                edit_course = true;
                                ui.close_menu();
                            }
                            if ui
                                .button(if self.flight.is_manual() {
                                    "Watch autopilot"
                                } else {
                                    "Take controls"
                                })
                                .clicked()
                            {
                                switch_pilot = true;
                                ui.close_menu();
                            }
                            if ui.button("Show controls").clicked() {
                                self.cinema = false;
                                ui.close_menu();
                            }
                        })
                        .response;
                    let open = egui::menu::BarState::load(ctx, ui.id()).is_some();
                    (response, open)
                })
                .inner;
            if self.menu_open != result.1 {
                ctx.request_repaint();
            }
            self.menu_open = result.1;
            self.publish_compact_menu(result.0.rect);
        } else if self.cinema {
            let top = if rect.width() >= 600.0 {
                rect.right_top() + Vec2::new(-286.0, 50.0)
            } else {
                rect.left_top() + Vec2::new(16.0, 108.0)
            };
            for (i, camera) in CameraMode::ALL.into_iter().enumerate() {
                let button = Rect::from_min_size(
                    top + Vec2::new(i as f32 * 74.0, 0.0),
                    Vec2::new(68.0, 28.0),
                );
                if ui
                    .put(
                        button,
                        egui::Button::new(RichText::new(camera.title()).size(11.0))
                            .wrap_mode(egui::TextWrapMode::Extend)
                            .selected(self.camera == camera),
                    )
                    .clicked()
                {
                    self.camera = camera;
                }
            }
            let exit = Rect::from_min_size(top + Vec2::new(0.0, 36.0), Vec2::new(98.0, 24.0));
            if ui
                .put(
                    exit,
                    egui::Button::new(RichText::new("Show controls").size(11.0))
                        .wrap_mode(egui::TextWrapMode::Extend),
                )
                .clicked()
            {
                self.cinema = false;
            }
            let pilot = Rect::from_min_size(top + Vec2::new(104.0, 36.0), Vec2::new(110.0, 24.0));
            if ui
                .put(
                    pilot,
                    egui::Button::new(
                        RichText::new(rival_action.unwrap_or(if self.flight.is_manual() {
                            "Watch autopilot"
                        } else {
                            "Take controls"
                        }))
                        .size(11.0),
                    )
                    .wrap_mode(egui::TextWrapMode::Extend),
                )
                .clicked()
            {
                if rival_action.is_some() {
                    switch_rival = true;
                } else {
                    switch_pilot = true;
                }
            }
            if rival_action.is_some() {
                Self::publish_rival_button(pilot);
            }
            let edit = Rect::from_min_size(top + Vec2::new(220.0, 36.0), Vec2::new(54.0, 24.0));
            edit_course = ui
                .put(
                    edit,
                    egui::Button::new(RichText::new("Edit").size(11.0))
                        .wrap_mode(egui::TextWrapMode::Extend),
                )
                .clicked();
            self.publish_editor_entry(edit);
            let photo = Rect::from_min_size(
                if rect.width() >= 600.0 {
                    top + Vec2::new(0.0, -32.0)
                } else {
                    rect.right_top() + Vec2::new(-124.0, 16.0)
                },
                Vec2::new(108.0, 26.0),
            );
            take_photo = ui
                .put(
                    photo,
                    egui::Button::new(RichText::new("Photo mode").size(11.0))
                        .wrap_mode(egui::TextWrapMode::Extend),
                )
                .clicked();
            self.publish_photo_entry(photo);
            let help = Rect::from_min_size(
                if rect.width() >= 600.0 {
                    top + Vec2::new(114.0, -32.0)
                } else {
                    rect.right_top() + Vec2::new(-124.0, 52.0)
                },
                Vec2::new(108.0, 26.0),
            );
            show_guide = ui
                .put(
                    help,
                    egui::Button::new(RichText::new("How to play").size(11.0))
                        .wrap_mode(egui::TextWrapMode::Extend),
                )
                .clicked();
            self.publish_guide_entry(help);
        }
        let mut retry = false;
        let mut replay = false;
        let mut toggle = false;
        if self.flight.is_manual() {
            let best_ready = self.recording.is_none()
                && self.flight.inputs.is_empty()
                && self.records.best(self.flight.course).is_some();
            pad::FlightPad::paint(
                ui,
                rect,
                self.input,
                self.recording.is_none() && current.status == Status::Flying,
            );
            for (i, (label, action)) in [
                (if self.playing { "Pause" } else { "Play" }, &mut toggle),
                ("Retry", &mut retry),
                (
                    if best_ready { "Replay best" } else { "Replay" },
                    &mut replay,
                ),
            ]
            .into_iter()
            .enumerate()
            {
                let button = pad::actions(rect, self.cinema)[i];
                *action = ui
                    .put(
                        button,
                        egui::Button::new(RichText::new(label).size(11.0))
                            .wrap_mode(egui::TextWrapMode::Extend),
                    )
                    .clicked();
            }
            if replay && best_ready {
                replay_best = true;
                replay = false;
            }
        }
        if self.error.is_some() {
            let center = rect.center();
            let card =
                Rect::from_center_size(center, Vec2::new((rect.width() - 28.0).min(350.0), 140.0));
            painter.rect_filled(card, 12.0, Color32::from_rgba_unmultiplied(10, 18, 34, 245));
            painter.text(
                center + Vec2::new(0.0, -40.0),
                egui::Align2::CENTER_CENTER,
                "FLIGHT UNAVAILABLE",
                FontId::proportional(20.0),
                Color32::LIGHT_RED,
            );
            painter.text(
                center + Vec2::new(0.0, -8.0),
                egui::Align2::CENTER_CENTER,
                "Open controls for details, or try again.",
                FontId::proportional(12.0),
                Color32::WHITE,
            );
            if ui
                .put(
                    Rect::from_center_size(center + Vec2::new(0.0, 36.0), Vec2::new(100.0, 30.0)),
                    egui::Button::new("Retry"),
                )
                .clicked()
            {
                retry = true;
            }
        } else if self.seek.is_some() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "PREPARING REPLAY",
                FontId::monospace(16.0),
                CYAN,
            );
        } else if current.status != Status::Flying || (self.replay_finished() && self.cursor > 0) {
            match results::show(self, ui, rect) {
                results::Action::Again => retry = true,
                results::Action::Best => replay_best = true,
                results::Action::None => {}
            }
        } else if self.flight.is_manual() && self.cursor == 0 && !self.playing {
            let center = rect.center() + Vec2::new(0.0, -10.0);
            painter.text(
                center + Vec2::new(0.0, -26.0),
                egui::Align2::CENTER_CENTER,
                if self.racing_best() {
                    "YOU VS YOUR BEST"
                } else {
                    "YOU VS AUTOPILOT"
                },
                FontId::proportional(if rect.width() < 500.0 { 22.0 } else { 32.0 }),
                CYAN,
            );
            painter.text(
                center + Vec2::new(0.0, 9.0),
                egui::Align2::CENTER_CENTER,
                if self.recording.is_some() {
                    "PLAY THE RECORDED ATTEMPT"
                } else {
                    if self.has_started {
                        "PRESS PLAY FOR 3 - 2 - 1"
                    } else {
                        "HOLD A FLIGHT CONTROL FOR 3 - 2 - 1"
                    }
                },
                FontId::monospace(10.0),
                Color32::WHITE,
            );
            if let Some(best) = self.records.best(self.flight.course) {
                painter.text(
                    center + Vec2::new(0.0, 35.0),
                    egui::Align2::CENTER_CENTER,
                    format!(
                        "YOUR BEST {:.2} s  /  REPLAY BEST TO WATCH",
                        best.steps() as f64 * DT
                    ),
                    FontId::monospace(if rect.width() < 500.0 { 9.0 } else { 11.0 }),
                    GOLD,
                );
            }
        }
        if let Some(start) = &self.start {
            start.paint(&painter, rect);
        }
        if self.menu_open && self.counting_down() {
            self.playing = false;
            self.reset_clock();
            ctx.request_repaint();
        }
        if switch_rival || switch_pilot || replay_best || retry || replay || toggle {
            ctx.request_repaint();
        }
        if show_guide {
            self.begin_guide();
            ctx.request_repaint();
            return;
        } else if take_photo {
            self.begin_photo();
            ctx.request_repaint();
            return;
        } else if edit_course {
            self.begin_editor();
            ctx.request_repaint();
            return;
        } else if switch_rival {
            self.choose_rival();
        } else if switch_pilot {
            self.choose_pilot(!self.flight.is_manual());
        } else if replay_best {
            self.replay_best();
        } else if retry {
            self.rewind();
        } else if replay {
            self.replay_run();
        } else if toggle {
            self.toggle_playing();
        }
        if self.flight.is_manual() {
            self.publish_pad(rect);
        }
        self.publish_status();
        let preparing_result = self.flight.last_index() > 0
            && (self.flight.done() || self.replay_finished())
            && self.rival().is_some_and(|g| !g.done())
            && self.error.is_none();
        if self.playing || self.seek.is_some() || preparing_result {
            ctx.request_repaint();
        }
    }

    fn hud(&self, painter: &egui::Painter, rect: Rect, speed: f64, time: f64) {
        if pad::compact(rect) {
            let left = rect.left_top() + Vec2::new(16.0, 12.0);
            painter.text(
                left,
                egui::Align2::LEFT_TOP,
                self.flight.course.title().to_uppercase(),
                FontId::proportional(15.0),
                Color32::WHITE,
            );
            painter.text(
                left + Vec2::new(0.0, 22.0),
                egui::Align2::LEFT_TOP,
                if self.counting_down() {
                    "GET READY"
                } else if self.playing {
                    if self.flight.is_manual() {
                        "YOU ARE FLYING"
                    } else {
                        "AUTOPILOT"
                    }
                } else {
                    "PAUSED"
                },
                FontId::monospace(10.0),
                CYAN,
            );
            let score = self.rival_score();
            painter.text(
                rect.center_bottom() + Vec2::new(0.0, -10.0),
                egui::Align2::CENTER_BOTTOM,
                format!("{score}   /   {speed:.1} m/s   /   {time:.2} s"),
                FontId::monospace(11.0),
                CYAN,
            );
            if self.flight.is_manual() {
                self.next_gate_cue(painter, rect, 112.0);
            }
            return;
        }
        let pad = if rect.width() < 500.0 { 16.0 } else { 26.0 };
        let left = rect.left() + pad;
        let top = rect.top() + pad;
        let small = Color32::from_rgb(158, 180, 201);
        painter.text(
            egui::pos2(left, top),
            egui::Align2::LEFT_TOP,
            "RUST / FLIGHT LAB",
            FontId::monospace(10.0),
            CYAN,
        );
        painter.text(
            egui::pos2(left, top + 20.0),
            egui::Align2::LEFT_TOP,
            self.flight.course.title().to_uppercase(),
            FontId::proportional(if rect.width() < 500.0 { 22.0 } else { 30.0 }),
            Color32::WHITE,
        );
        let mode = if self.counting_down() {
            "GET READY / FLIGHT CLOCK HELD"
        } else if self.seek.is_some() {
            "BUILDING REPLAY"
        } else if self.flight.is_manual() && self.recording.is_some() {
            "RECORDED ATTEMPT"
        } else if !self.playing {
            "PAUSED"
        } else if self.flight.is_manual() {
            "YOU ARE FLYING"
        } else {
            "AUTONOMOUS FLIGHT"
        };
        painter.text(
            egui::pos2(left, top + 59.0),
            egui::Align2::LEFT_TOP,
            mode,
            FontId::monospace(10.0),
            small,
        );
        if self.rival().is_some() {
            painter.text(
                egui::pos2(left, top + 82.0),
                egui::Align2::LEFT_TOP,
                self.rival_score(),
                FontId::monospace(11.0),
                self.rival_color(),
            );
            self.next_gate_cue(
                painter,
                rect,
                if rect.width() < 600.0 { 236.0 } else { 126.0 },
            );
        }
        if rect.width() >= 500.0 && !self.cinema {
            painter.text(
                rect.right_top() + Vec2::new(-pad, pad),
                egui::Align2::RIGHT_TOP,
                format!("{} CAM", self.camera.title().to_uppercase()),
                FontId::monospace(11.0),
                small,
            );
        }
        let bottom = rect.bottom() - pad;
        let width = (rect.width() - pad * 2.0) / 3.0;
        let current = &self.flight.frames[self.cursor];
        for (i, (label, value, suffix)) in [
            (
                "GATES",
                format!(
                    "{:02} / {:02}",
                    current.passed,
                    self.flight.lap.gate_count()
                ),
                "",
            ),
            ("SPEED", format!("{speed:.1}"), "m/s"),
            ("FLIGHT TIME", format!("{time:05.2}"), "s"),
        ]
        .into_iter()
        .enumerate()
        {
            let x = left + width * i as f32;
            painter.text(
                egui::pos2(x, bottom - 47.0),
                egui::Align2::LEFT_TOP,
                label,
                FontId::monospace(10.0),
                small,
            );
            painter.text(
                egui::pos2(x, bottom - 28.0),
                egui::Align2::LEFT_TOP,
                value,
                FontId::monospace(if rect.width() < 500.0 { 21.0 } else { 29.0 }),
                Color32::WHITE,
            );
            if !suffix.is_empty() {
                painter.text(
                    egui::pos2(x + width - 20.0, bottom - 7.0),
                    egui::Align2::RIGHT_BOTTOM,
                    suffix,
                    FontId::monospace(11.0),
                    small,
                );
            }
        }
        let track_y = bottom - 68.0;
        let count = self.flight.lap.gate_count();
        for i in 0..count {
            let gap = 6.0;
            let segment = (rect.width() - pad * 2.0 - gap * (count - 1) as f32) / count as f32;
            let x = left + i as f32 * (segment + gap);
            let color = if i < current.passed {
                CYAN
            } else if i == current.passed {
                GOLD
            } else {
                Color32::from_rgb(50, 67, 84)
            };
            painter.rect_filled(
                Rect::from_min_size(egui::pos2(x, track_y), Vec2::new(segment, 3.0)),
                2.0,
                color,
            );
        }
    }

    fn next_gate_cue(&self, painter: &egui::Painter, rect: Rect, height: f32) {
        let current = &self.flight.frames[self.cursor];
        if let Some(gate) = self.flight.lap.gates.get(current.passed) {
            let delta = flight::sub(gate.center(), current.state.position);
            let center = rect.center_top() + Vec2::new(0.0, height);
            painter.rect_filled(
                Rect::from_center_size(center, Vec2::new((rect.width() - 28.0).min(450.0), 28.0)),
                8.0,
                Color32::from_rgba_unmultiplied(10, 18, 34, 215),
            );
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                format!(
                    "NEXT {:02} / {:.1}m {} / {:.1}m {} / {:.1}m {}",
                    current.passed + 1,
                    delta[0].abs(),
                    if delta[0] >= 0.0 { "fwd" } else { "back" },
                    delta[1].abs(),
                    if delta[1] >= 0.0 { "left" } else { "right" },
                    delta[2].abs(),
                    if delta[2] >= 0.0 { "up" } else { "down" }
                ),
                FontId::monospace(if rect.width() < 500.0 { 8.5 } else { 11.0 }),
                GOLD,
            );
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn publish_menu_camera(camera: CameraMode, rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                &format!("data-drone-menu-{}", camera.slug()),
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_menu_camera(_camera: CameraMode, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_rival_button(rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                "data-drone-rival-button",
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_rival_button(_rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_compact_menu(&self, rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                "data-drone-menu",
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_compact_menu(&self, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_guide_entry(&self, rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                "data-drone-guide-open",
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_guide_entry(&self, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_photo_entry(&self, rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                "data-drone-photo-open",
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_photo_entry(&self, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_editor_entry(&self, rect: Rect) {
        if let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        {
            let _ = canvas.set_attribute(
                "data-drone-editor-open",
                &format!("[{:.1},{:.1}]", rect.center().x, rect.center().y),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_editor_entry(&self, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_pad(&self, rect: Rect) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let names = ["forward", "back", "left", "right", "up", "down"];
        let fields = pad::buttons(rect)
            .into_iter()
            .zip(names)
            .map(|((_, button), name)| {
                format!(
                    "\"{name}\":[{:.1},{:.1}]",
                    button.center().x,
                    button.center().y
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let _ = canvas.set_attribute("data-drone-pad", &format!("{{{fields}}}"));
        let fields = pad::actions(rect, self.cinema)
            .into_iter()
            .zip(["pause", "retry", "replay"])
            .map(|(button, name)| {
                format!(
                    "\"{name}\":[{:.1},{:.1}]",
                    button.center().x,
                    button.center().y
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let _ = canvas.set_attribute("data-drone-actions", &format!("{{{fields}}}"));
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_pad(&self, _rect: Rect) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_status(&self) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let (flight, cursor) = self.editor.as_ref().map_or(
            (
                &self.flight,
                self.photo
                    .as_ref()
                    .map_or(self.cursor, |p| p.frame.min(self.flight.last_index())),
            ),
            |editor| (editor.preview(), 0),
        );
        let frame = &flight.frames[cursor];
        let checkpoint = checkpoint::Checkpoint::at(flight, cursor, cursor as f64 * DT);
        // Accessible scene summary also permits functional browser checks of
        // real flight progress, paused replays and camera changes.
        for (key, value) in [
            ("data-drone-frame", cursor.to_string()),
            (
                "data-drone-clear-gate",
                checkpoint.map_or_else(String::new, |c| c.gate.to_string()),
            ),
            (
                "data-drone-clear-time",
                checkpoint.map_or_else(String::new, |c| format!("{:.2}", c.time())),
            ),
            (
                "data-drone-clear-age",
                checkpoint.map_or_else(String::new, |c| format!("{:.3}", c.age)),
            ),
            (
                "data-drone-clear-finish",
                checkpoint.map_or_else(String::new, |c| u8::from(c.finish).to_string()),
            ),
            (
                "data-drone-start-age",
                self.start
                    .as_ref()
                    .map_or_else(String::new, |s| format!("{:.3}", s.age)),
            ),
            (
                "data-drone-start",
                self.start
                    .as_ref()
                    .map_or("", start::StartSequence::label)
                    .to_owned(),
            ),
            ("data-drone-guide", u8::from(self.guide).to_string()),
            ("data-drone-menu-open", u8::from(self.menu_open).to_string()),
            (
                "data-drone-best-steps",
                self.records
                    .best(flight.course)
                    .map_or(0, records::Best::steps)
                    .to_string(),
            ),
            (
                "data-drone-new-best",
                u8::from(self.new_best.is_some()).to_string(),
            ),
            (
                "data-drone-best-persistent",
                u8::from(self.records.persistent).to_string(),
            ),
            (
                "data-drone-photo",
                u8::from(self.photo.is_some()).to_string(),
            ),
            (
                "data-drone-editor",
                u8::from(self.editor.is_some()).to_string(),
            ),
            ("data-drone-layout", flight.course.encode()),
            ("data-drone-generated", flight.last_index().to_string()),
            ("data-drone-gates", frame.passed.to_string()),
            (
                "data-drone-status",
                self.error
                    .as_ref()
                    .map_or(frame.status.slug(), |_| "error")
                    .to_string(),
            ),
            ("data-drone-camera", self.camera.slug().to_string()),
            ("data-drone-course", flight.course.slug().to_string()),
            (
                "data-drone-mode",
                if self.flight.is_manual() {
                    "manual"
                } else {
                    "auto"
                }
                .to_string(),
            ),
            (
                "data-drone-replay",
                u8::from(self.recording.is_some() && self.editor.is_none()).to_string(),
            ),
            (
                "data-drone-input",
                format!(
                    "{},{},{}",
                    self.input.forward, self.input.right, self.input.lift
                ),
            ),
            ("data-drone-share", self.share_query()),
            (
                "data-drone-velocity",
                frame
                    .state
                    .velocity
                    .iter()
                    .map(|v| format!("{v:.6}"))
                    .collect::<Vec<_>>()
                    .join(","),
            ),
            (
                "data-drone-ghost-frame",
                self.rival()
                    .map_or(0, |g| cursor.min(g.last_index()))
                    .to_string(),
            ),
            (
                "data-drone-ghost-gates",
                self.rival()
                    .map_or(0, |g| g.frames[cursor.min(g.last_index())].passed)
                    .to_string(),
            ),
            (
                "data-drone-ghost-finish",
                self.rival()
                    .filter(|g| g.done() && g.frames[g.last_index()].status == Status::Finished)
                    .map_or_else(String::new, |g| {
                        format!("{:.2}", g.last_index() as f64 * DT)
                    }),
            ),
            (
                "data-drone-rival",
                if self.racing_best() { "best" } else { "auto" }.to_owned(),
            ),
            (
                "data-drone-ghost-position",
                self.rival().map_or_else(String::new, |g| {
                    g.frames[cursor.min(g.last_index())]
                        .state
                        .position
                        .iter()
                        .map(|v| format!("{v:.6}"))
                        .collect::<Vec<_>>()
                        .join(",")
                }),
            ),
            (
                "data-drone-best-split",
                self.best_split_at(cursor)
                    .map_or_else(String::new, |(gate, delta)| format!("{gate},{delta:.2}")),
            ),
            (
                "data-drone-position",
                frame
                    .state
                    .position
                    .iter()
                    .map(|v| format!("{v:.6}"))
                    .collect::<Vec<_>>()
                    .join(","),
            ),
            (
                "data-drone-planning-ms",
                format!("{:.2}", self.last_plan_ms),
            ),
            ("data-drone-playing", u8::from(self.playing).to_string()),
            ("data-drone-cinema", u8::from(self.cinema).to_string()),
            (
                "data-drone-seeking",
                u8::from(self.seek.is_some()).to_string(),
            ),
            (
                "aria-label",
                format!(
                    "Drone race, {}: {} of {} gates, {:.2} seconds, {} camera",
                    if flight.is_manual() {
                        if self.recording.is_some() {
                            "recorded human flight"
                        } else {
                            "you are the pilot"
                        }
                    } else {
                        "autopilot"
                    },
                    frame.passed,
                    flight.lap.gate_count(),
                    cursor as f64 * DT,
                    self.camera.title()
                ),
            ),
        ] {
            let _ = canvas.set_attribute(key, &value);
        }
        if let Some(editor) = &self.editor {
            let _ = canvas.set_attribute(
                "aria-label",
                &format!(
                    "Course editor: {}, five ordered gates. Drag in Top or Side view to edit.",
                    editor.course().title()
                ),
            );
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_status(&self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_live_finishes_set_bests_and_retry_preserves_the_record() {
        let mut demo = DroneRaceDemo::default();
        demo.flight = records::tests::finish(Course::CLIMB, 0);
        demo.cursor = demo.flight.last_index();
        demo.record_finish();
        assert!(demo.new_best.is_some());
        let steps = demo.cursor;
        let inputs = demo.flight.inputs.clone();
        demo.record_finish();
        assert!(
            demo.new_best.is_some(),
            "repainting must retain the celebration"
        );
        demo.rewind();
        assert!(demo.new_best.is_none());
        assert!(demo.recording.is_none());
        assert!(!demo.playing);
        assert_eq!(demo.records.best(Course::CLIMB).unwrap().steps(), steps);
        demo.replay_best();
        assert_eq!(demo.recording.as_ref().unwrap(), &inputs);
        assert!(demo.playing);
        assert!(demo.new_best.is_none());
        assert_eq!(demo.cursor, 0);

        let mut receiver = DroneRaceDemo::default();
        let query = demo
            .share_query()
            .replacen("frame=0", &format!("frame={steps}"), 1);
        receiver.apply_share_query(&query);
        for step in 0..steps + 2 {
            receiver.advance(step as f64 * 0.01);
        }
        assert_eq!(receiver.cursor, steps);
        receiver.record_finish();
        assert!(receiver.records.best(Course::CLIMB).is_none());
        assert!(receiver.new_best.is_none());
    }

    #[test]
    fn share_links_restore_paused_flight_and_all_view_settings() {
        let mut original = DroneRaceDemo::default();
        for _ in 0..12 {
            original.flight.step().unwrap();
        }
        original.cursor = 12;
        original.playing = false;
        original.camera = CameraMode::Pilot;
        original.slow = true;
        original.cinema = true;
        original.trail = false;
        let query = original.share_query();
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&query);
        for i in 0..20 {
            restored.advance(i as f64 * 0.01);
        }
        assert!(restored.seek.is_none());
        assert_eq!(restored.share_query(), query);
        assert_eq!(restored.flight.frames, original.flight.frames);
        let count = restored.flight.frames.len();
        restored.advance(1000.0);
        assert_eq!(restored.cursor, 12);
        assert_eq!(restored.flight.frames.len(), count);
    }

    #[test]
    fn bounded_seek_yields_and_replay_uses_cached_history() {
        let mut demo = DroneRaceDemo::default();
        demo.apply_share_query("tab=drone&frame=600&playing=0");
        demo.advance(1.0);
        assert!(demo.flight.last_index() <= 2);
        assert!(demo.seek.is_some());
        demo.seek_to(0);
        demo.playing = true;
        let cached = demo.flight.frames.clone();
        demo.advance(2.0);
        demo.advance(200.0);
        assert_eq!(demo.cursor, 0, "background time must not advance the race");
        assert_eq!(demo.flight.frames, cached);
    }

    #[test]
    fn unrelated_and_invalid_links_do_not_change_or_allocate_flight_history() {
        let mut demo = DroneRaceDemo::default();
        let before = demo.share_query();
        demo.apply_share_query("tab=arena&course=climb&frame=600");
        assert_eq!(demo.share_query(), before);
        demo.apply_share_query("tab=drone&frame=999999&camera=invalid&course=unknown");
        assert_eq!(demo.cursor, 0);
        assert!(demo.seek.is_none());
        assert_eq!(demo.flight.frames.len(), 1);
        assert_eq!(demo.camera, CameraMode::Chase);
        assert_eq!(demo.flight.course, Course::NEON);
    }

    #[test]
    fn manual_share_restores_the_inputs_pose_and_synchronized_opponent_then_stops() {
        let mut original = DroneRaceDemo::default();
        original.choose_pilot(true);
        for i in 0..24 {
            original.input =
                PilotInput::new(1, if i < 12 { -1 } else { 0 }, if i < 8 { 1 } else { 0 });
            assert!(original.step_flight());
        }
        original.cursor = original.flight.last_index();
        original.playing = false;
        original.camera = CameraMode::Course;
        original.cinema = true;
        original.show_ghost = false;
        let query = original.share_query();
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&query);
        for i in 0..32 {
            restored.advance(i as f64 * 0.01);
            assert!(restored.ghost.as_ref().unwrap().last_index() <= restored.flight.last_index());
        }
        assert!(restored.seek.is_none());
        assert_eq!(restored.share_query(), query);
        assert_eq!(restored.flight.frames, original.flight.frames);
        assert_eq!(restored.flight.inputs, original.flight.inputs);
        assert_eq!(
            restored.ghost.as_ref().unwrap().frames,
            original.ghost.as_ref().unwrap().frames
        );
        restored.replay_run();
        restored.input = PilotInput::new(-1, 1, -1);
        for i in 0..60 {
            restored.advance(1.0 + i as f64 * DT);
        }
        assert_eq!(restored.cursor, 24);
        assert!(!restored.playing);
        assert_eq!(
            restored.flight.frames, original.flight.frames,
            "new keys cannot change a recorded attempt"
        );
        restored.rewind();
        assert!(!restored.playing);
        assert!(restored.recording.is_none());
        assert!(restored.flight.inputs.is_empty());
        assert_eq!(restored.cursor, 0);
        assert_eq!(restored.flight.frames.len(), 1);
        assert!(!restored.has_started);
    }

    #[test]
    fn invalid_manual_recordings_cannot_generate_physics_and_can_be_restarted() {
        let mut demo = DroneRaceDemo::default();
        demo.apply_share_query("tab=drone&mode=manual&inputs=v99.zzz&frame=600&playing=1");
        for i in 0..10 {
            demo.advance(i as f64 * DT);
        }
        assert!(demo.error.is_some());
        assert!(!demo.playing);
        assert_eq!(demo.flight.frames.len(), 1);
        assert_eq!(demo.ghost.as_ref().unwrap().frames.len(), 1);
        demo.rewind();
        assert!(demo.error.is_none());
        assert!(demo.flight.is_manual());
        assert!(demo.recording.is_none());
    }

    #[test]
    fn editing_preserves_the_attempt_until_apply_and_custom_recordings_roundtrip() {
        let custom = Course::CLIMB.moved(1, [8.4, 0.2, 4.0]);
        let mut demo = DroneRaceDemo::default();
        demo.reset_course(Course::CLIMB, true);
        for _ in 0..4 {
            demo.input = PilotInput::new(1, 0, 1);
            demo.step_flight();
        }
        demo.cursor = 4;
        let history = demo.flight.frames.clone();
        demo.begin_editor();
        let draft = demo.share_query();
        assert!(!draft.contains("inputs="));
        assert!(draft.contains("edit=1"));
        demo.finish_editor(EditorAction::Cancel);
        assert_eq!(demo.flight.frames, history);
        assert_eq!(demo.cursor, 4);
        assert!(!demo.playing);
        demo.begin_editor();
        demo.finish_editor(EditorAction::Apply(custom));
        assert!(demo.editor.is_none());
        assert_eq!(demo.flight.course, custom);
        assert_eq!(demo.ghost.as_ref().unwrap().course, custom);
        assert!(demo.flight.inputs.is_empty());
        assert_eq!(demo.cursor, 0);
        assert!(!demo.playing);
        demo.begin_editor();
        let mut editor_link = DroneRaceDemo::default();
        editor_link.apply_share_query(&demo.share_query());
        assert_eq!(editor_link.editor.as_ref().unwrap().course(), custom);
        assert_eq!(editor_link.flight.last_index(), 0);
        demo.finish_editor(EditorAction::Cancel);
        for _ in 0..8 {
            demo.input = PilotInput::new(1, 0, 1);
            assert!(demo.step_flight());
        }
        demo.cursor = 8;
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&demo.share_query());
        for i in 0..16 {
            restored.advance(i as f64 * 0.01);
        }
        assert_eq!(restored.flight.course, custom);
        assert_eq!(restored.flight.frames, demo.flight.frames);
        assert_eq!(
            restored.ghost.as_ref().unwrap().frames,
            demo.ghost.as_ref().unwrap().frames
        );
    }

    #[test]
    fn malformed_course_links_stop_before_seeking_or_replaying_an_attempt() {
        for query in [
            "tab=drone&course=custom&frame=600&playing=1",
            "tab=drone&gates=v9.bad&frame=600&playing=1&edit=1",
            "tab=drone&mode=manual&gates=v1.40_0_28&inputs=v1.nnn&frame=3",
        ] {
            let mut demo = DroneRaceDemo::default();
            demo.apply_share_query(query);
            for i in 0..5 {
                demo.advance(i as f64 * DT);
            }
            assert!(demo.error.is_some());
            assert!(demo.seek.is_none());
            assert!(demo.editor.is_none());
            assert!(!demo.playing);
            assert_eq!(demo.flight.last_index(), 0);
        }
    }

    #[test]
    fn photo_links_restore_a_real_manual_moment_without_changing_the_original_attempt() {
        let mut demo = DroneRaceDemo::default();
        demo.reset_course(Course::CLIMB.moved(1, [8.4, 0.2, 4.0]), true);
        for _ in 0..6 {
            demo.input = PilotInput::new(1, 0, 1);
            assert!(demo.step_flight());
        }
        demo.cursor = 6;
        let history = demo.flight.frames.clone();
        let inputs = demo.flight.inputs.clone();
        let opponent = demo.ghost.as_ref().unwrap().frames.clone();
        demo.playing = true;
        demo.begin_photo();
        assert!(!demo.playing);
        assert_eq!(demo.input, PilotInput::default());
        let photo = demo.photo.as_mut().unwrap();
        photo.frame = 3;
        photo.apply_query("yaw=156.25&pitch=37.5&distance=4.125&format=portrait&caption=0");
        let query = demo.share_query();
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&query);
        for i in 0..8 {
            restored.advance(i as f64 * DT);
        }
        assert_eq!(restored.photo.as_ref().unwrap().frame, 3);
        assert_eq!(restored.flight.frames[3], history[3]);
        assert_eq!(restored.share_query(), query);
        assert!(!restored.playing);
        demo.close_photo();
        assert_eq!(demo.cursor, 6);
        assert_eq!(demo.flight.frames, history);
        assert_eq!(demo.flight.inputs, inputs);
        assert_eq!(demo.ghost.as_ref().unwrap().frames, opponent);
        assert!(demo.recording.is_none());
        assert!(!demo.playing);
    }

    #[test]
    fn guide_freezes_manual_history_and_held_keys_then_returns_to_the_attempt() {
        let mut demo = DroneRaceDemo::default();
        demo.reset_course(Course::CLIMB, true);
        for _ in 0..3 {
            demo.input = PilotInput::new(1, 0, 1);
            assert!(demo.step_flight());
        }
        demo.cursor = 3;
        demo.playing = true;
        let original = demo.flight.frames.clone();
        let opponent = demo.ghost.as_ref().unwrap().frames.clone();
        demo.begin_guide();
        let ctx = egui::Context::default();
        for (i, size) in [Vec2::new(320.0, 740.0), Vec2::new(667.0, 375.0)]
            .into_iter()
            .enumerate()
        {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(egui::Pos2::ZERO, size)),
                    time: Some(100.0 + i as f64),
                    focused: true,
                    events: [egui::Key::Space, egui::Key::R, egui::Key::W]
                        .into_iter()
                        .map(|key| egui::Event::Key {
                            key,
                            physical_key: None,
                            pressed: true,
                            repeat: false,
                            modifiers: egui::Modifiers::default(),
                        })
                        .collect(),
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| demo.scene(ctx, ui));
                },
            );
        }
        assert!(demo.guide && !demo.playing);
        assert_eq!(demo.flight.frames, original);
        assert_eq!(demo.ghost.as_ref().unwrap().frames, opponent);
        assert_eq!(demo.input, PilotInput::default());
        demo.finish_guide(guide::Action::Fly);
        assert!(!demo.guide && !demo.playing);
        assert_eq!(demo.cursor, 3);
        assert_eq!(demo.flight.frames, original);
        assert!(demo.recording.is_none());
    }

    #[test]
    fn intro_links_are_opt_in_and_do_not_intercept_replays_photos_or_invalid_courses() {
        let mut demo = DroneRaceDemo::default();
        demo.apply_share_query("tab=drone&mode=manual&course=climb&cinema=1&intro=1");
        assert!(demo.guide && !demo.playing && demo.cinema());
        assert!(!demo.share_query().contains("intro="));
        demo.finish_guide(guide::Action::Watch);
        assert!(!demo.flight.is_manual() && demo.playing && !demo.guide);
        demo.begin_guide();
        demo.finish_guide(guide::Action::Build);
        assert!(demo.editor.is_some() && !demo.guide && !demo.playing);
        for query in [
            "tab=drone&mode=manual",
            "tab=drone&mode=manual&inputs=v1.nnn&frame=0&intro=1",
            "tab=drone&photo=1&intro=1",
            "tab=drone&edit=1&intro=1",
            "tab=drone&course=custom&intro=1",
        ] {
            demo.apply_share_query(query);
            assert!(!demo.guide, "{query}");
        }
    }

    #[test]
    fn best_ghost_is_a_fixed_physical_flight_until_the_next_attempt() {
        let slow = records::tests::finish(Course::CLIMB, 40);
        let fast = records::tests::finish(Course::CLIMB, 0);
        let mut demo = DroneRaceDemo::default();
        demo.records.record(&slow).unwrap();
        demo.reset_course(Course::CLIMB, true);
        assert_eq!(demo.rival().unwrap().frames, slow.frames);
        for &input in &fast.inputs {
            demo.input = input;
            assert!(demo.step_flight());
        }
        demo.cursor = demo.flight.last_index();
        assert_eq!(demo.flight.frames, fast.frames);
        assert_eq!(
            demo.best_split(),
            Some((
                5,
                (fast.last_index() as f64 - slow.last_index() as f64) * DT
            ))
        );
        demo.record_finish();
        assert_eq!(
            demo.records.best(Course::CLIMB).unwrap().steps(),
            fast.last_index()
        );
        assert_eq!(
            demo.rival().unwrap().frames,
            slow.frames,
            "a new record replaced the opponent mid-attempt"
        );
        assert_eq!(
            demo.ghost.as_ref().unwrap().last_index(),
            0,
            "the unused autopilot should not plan"
        );
        demo.rewind();
        assert_eq!(demo.rival().unwrap().frames, fast.frames);
        assert!(demo.best_split().is_none());
        demo.choose_rival();
        assert!(!demo.racing_best());
        assert!(!demo.rival().unwrap().is_manual());
        demo.choose_pilot(false);
        assert!(!demo.flight.is_manual());
        demo.choose_pilot(true);
        demo.choose_rival();
        assert!(demo.racing_best());
        demo.reset_course(Course::NEON, true);
        assert!(
            !demo.racing_best(),
            "a different course used the previous course's ghost"
        );
    }

    #[test]
    fn shared_best_ghost_reconstructs_without_claiming_a_receivers_record() {
        let slow = records::tests::finish(Course::CLIMB, 20);
        let fast = records::tests::finish(Course::CLIMB, 0);
        let mut original = DroneRaceDemo::default();
        original.records.record(&slow).unwrap();
        original.reset_course(Course::CLIMB, true);
        for &input in &fast.inputs {
            original.input = input;
            original.step_flight();
        }
        original.cursor = original.flight.last_index();
        original.playing = false;
        let query = original.share_query();
        assert!(query.contains("&rival=best&best_inputs=v1."));
        let mut restored = DroneRaceDemo::default();
        restored.apply_share_query(&query);
        for i in 0..MAX_STEPS {
            restored.advance(i as f64 * DT);
            if restored.seek.is_none() {
                break;
            }
        }
        assert!(restored.error.is_none());
        assert_eq!(restored.flight.frames, fast.frames);
        assert_eq!(restored.rival().unwrap().frames, slow.frames);
        assert_eq!(restored.best_split(), original.best_split());
        assert_eq!(restored.share_query(), query);
        restored.record_finish();
        assert!(restored.records.best(Course::CLIMB).is_none());
        restored.rewind();
        assert!(!restored.racing_best());
        assert!(restored.recording.is_none());
        let mut extra = slow.inputs.clone();
        extra.push(PilotInput::default());
        for encoded in ["v2.n".to_owned(), "v1.n".to_owned(), manual::encode(&extra)] {
            restored.apply_share_query(&format!(
                "tab=drone&course=climb&mode=manual&rival=best&best_inputs={encoded}"
            ));
            assert!(restored.error.is_some());
            assert!(!restored.playing);
            assert_eq!(restored.flight.last_index(), 0);
        }
    }

    #[test]
    fn countdown_holds_both_flights_and_keeps_the_first_command_across_go() {
        let mut demo = DroneRaceDemo::default();
        demo.reset_course(Course::CLIMB, true);
        demo.slow = true;
        let input = PilotInput::new(1, 0, 1);
        demo.input = input;
        demo.begin_start();
        demo.advance(0.0);
        demo.advance(0.1);
        let age = demo.start.as_ref().unwrap().age;
        // A background-sized time gap contributes no countdown or flight time.
        demo.advance(30.0);
        assert_eq!(demo.start.as_ref().unwrap().age, age);
        let mut now = 30.0;
        // Even at 2.5 paints per second (below the physics gap threshold),
        // the start lights use real elapsed time and still reach GO.
        for _ in 0..12 {
            now += 0.4;
            demo.advance(now);
            assert_eq!(demo.flight.last_index(), 0);
            assert_eq!(demo.ghost.as_ref().unwrap().last_index(), 0);
            assert_eq!(demo.input, input);
            if !demo.counting_down() {
                break;
            }
        }
        assert_eq!(demo.start.as_ref().unwrap().label(), "go");
        for _ in 0..3 {
            now += 0.1;
            demo.advance(now);
            if demo.flight.last_index() > 0 {
                break;
            }
        }
        assert_eq!(demo.flight.inputs.first(), Some(&input));
        assert_eq!(demo.flight.last_index(), 1);
        assert_eq!(demo.ghost.as_ref().unwrap().last_index(), 1);
        assert_eq!(demo.cursor, 1);
    }

    #[test]
    fn countdown_cancellation_and_sharing_return_to_a_stationary_start() {
        let mut demo = DroneRaceDemo::default();
        demo.reset_course(Course::CLIMB, true);
        demo.toggle_playing();
        assert!(demo.counting_down() && demo.playing);
        let query = demo.share_query();
        assert!(query.contains("&playing=0&"));
        let mut visitor = DroneRaceDemo::default();
        visitor.apply_share_query(&query);
        assert!(!visitor.playing && visitor.start.is_none());
        assert_eq!(visitor.flight.last_index(), 0);
        demo.toggle_playing();
        assert!(!demo.playing && demo.start.is_none());
        demo.toggle_playing();
        assert_eq!(demo.start.as_ref().unwrap().label(), "3");
        demo.begin_guide();
        assert!(!demo.playing && demo.start.is_none());
        demo.finish_guide(guide::Action::Fly);
        demo.toggle_playing();
        demo.begin_photo();
        assert!(!demo.playing && demo.start.is_none());
        demo.close_photo();
        demo.toggle_playing();
        demo.rewind();
        assert!(!demo.playing && demo.start.is_none() && !demo.has_started);
        assert_eq!(demo.flight.last_index(), 0);
        assert!(demo.flight.inputs.is_empty());
        assert!(demo.records.best(Course::CLIMB).is_none());
        demo.toggle_playing();
        let ctx = egui::Context::default();
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(
                    egui::Pos2::ZERO,
                    Vec2::new(844.0, 390.0),
                )),
                focused: true,
                events: vec![
                    egui::Event::WindowFocused(false),
                    egui::Event::WindowFocused(true),
                ],
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| demo.scene(ctx, ui));
            },
        );
        assert!(
            !demo.playing && demo.start.is_none(),
            "a hidden tab resumed its countdown after focus returned"
        );
        assert_eq!(demo.flight.last_index(), 0);
    }
}
