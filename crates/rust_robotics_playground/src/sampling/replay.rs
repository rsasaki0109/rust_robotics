//! Actual RRT* iterations: additions, rejected attempts, rewires, and solutions.
use super::{to_screen, SamplingDemo, GOAL, OBSTACLE, PATH, START, STAR_ITERATIONS, TREE, WORLD};
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use rust_robotics_planning::rrt_star::RrtStarTraceNode;
use std::collections::BTreeMap;

const REWIRE: Color32 = Color32::from_rgb(95, 235, 190);
const REMOVED: Color32 = Color32::from_rgb(225, 100, 120);
const SAMPLE: Color32 = Color32::from_rgb(255, 220, 95);

pub(super) struct SearchReplay {
    pub frame: usize,
    pub playing: bool,
    pub rate: f64,
    nodes: Vec<RrtStarTraceNode>,
    last_time: Option<f64>,
    pending: f64,
    actions: BTreeMap<&'static str, [f32; 2]>,
    hits: BTreeMap<&'static str, bool>,
    count: u64,
    last_action: &'static str,
    revision: u64,
    rect: Option<Rect>,
    plot: Option<Rect>,
}

impl Default for SearchReplay {
    fn default() -> Self {
        Self {
            frame: 0,
            playing: true,
            rate: 100.0,
            nodes: Vec::new(),
            last_time: None,
            pending: 0.0,
            actions: BTreeMap::new(),
            hits: BTreeMap::new(),
            count: 0,
            last_action: "",
            revision: 0,
            rect: None,
            plot: None,
        }
    }
}

fn length(path: &[[f64; 2]]) -> Option<f64> {
    (path.len() >= 2).then(|| {
        path.windows(2)
            .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
            .sum()
    })
}

impl SamplingDemo {
    pub(super) fn seek_search(&mut self, frame: usize) {
        self.replay.frame = frame.min(STAR_ITERATIONS);
        self.replay.playing = false;
        self.replay.last_time = None;
        self.replay.pending = 0.0;
        if let Some(trace) = self.result.as_ref().and_then(|r| r.trace.as_ref()) {
            self.replay.frame = self.replay.frame.min(trace.steps.len());
            self.replay.nodes = trace.tree_at(self.replay.frame);
        } else {
            self.replay.nodes.clear();
        }
    }

    fn advance_search(&mut self, now: f64) {
        let delta = self
            .replay
            .last_time
            .replace(now)
            .map_or(0.0, |last| now - last);
        if !self.replay.playing || !(0.0..1.0).contains(&delta) {
            self.replay.pending = 0.0;
            return;
        }
        self.replay.pending += delta * self.replay.rate;
        let steps = self.replay.pending.floor() as usize;
        if steps == 0 {
            return;
        }
        let remaining = self.replay.pending - steps as f64;
        self.seek_search(self.replay.frame + steps);
        self.replay.playing = self.replay.frame < STAR_ITERATIONS;
        self.replay.pending = remaining;
        self.replay.last_time = Some(now);
    }

    fn path_at(&self, frame: usize) -> &[[f64; 2]] {
        self.result
            .as_ref()
            .and_then(|r| r.trace.as_ref())
            .and_then(|t| frame.checked_sub(1).and_then(|i| t.steps.get(i)))
            .map_or(&[], |s| s.path.as_slice())
    }

    fn first_solution(&self) -> Option<usize> {
        self.result
            .as_ref()?
            .trace
            .as_ref()?
            .steps
            .iter()
            .position(|s| !s.path.is_empty())
            .map(|i| i + 1)
    }

    fn next_improvement(&self) -> Option<usize> {
        let trace = self.result.as_ref()?.trace.as_ref()?;
        let current = length(self.path_at(self.replay.frame)).unwrap_or(f64::INFINITY);
        trace
            .steps
            .iter()
            .enumerate()
            .skip(self.replay.frame)
            .find_map(|(i, s)| {
                length(&s.path)
                    .filter(|cost| *cost < current - 1e-7)
                    .map(|_| i + 1)
            })
    }

    fn next_rewire(&self) -> Option<usize> {
        self.result
            .as_ref()?
            .trace
            .as_ref()?
            .steps
            .iter()
            .enumerate()
            .skip(self.replay.frame)
            .find_map(|(i, s)| {
                s.updates
                    .iter()
                    .any(|u| {
                        u.before
                            .as_ref()
                            .is_some_and(|b| b.parent != u.after.parent)
                    })
                    .then_some(i + 1)
            })
    }

    pub(super) fn search_action(&mut self, name: &'static str, r: &egui::Response) {
        self.replay.actions.insert(name, r.rect.center().into());
        self.replay.hits.insert(name, r.hovered());
        if r.clicked() {
            self.ack_search(name);
            r.ctx.request_repaint();
        }
    }

    pub(super) fn ack_search(&mut self, name: &'static str) {
        self.replay.count += 1;
        self.replay.last_action = name;
    }

    pub(super) fn search_controls(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "RRT* · recorded iterations");
        ui.horizontal_wrapped(|ui| {
            ui.label("Seed");
            if ui
                .add(egui::DragValue::new(&mut self.seed).range(0..=1_000_000))
                .changed()
            {
                self.compare.clear();
                self.replan();
            }
        });
        ui.horizontal_wrapped(|ui| {
            let r = ui.button(if self.replay.playing { "Pause" } else { "Play" });
            self.search_action("play", &r);
            if r.clicked() {
                if !self.replay.playing && self.replay.frame == STAR_ITERATIONS {
                    self.seek_search(0);
                }
                self.replay.playing = !self.replay.playing;
                self.replay.last_time = None;
            }
            for (key, label, frame) in [
                ("start", "Start", 0),
                ("back", "Back", self.replay.frame.saturating_sub(1)),
                ("step", "Next", self.replay.frame + 1),
                ("end", "Result", STAR_ITERATIONS),
            ] {
                let r = ui.button(label);
                self.search_action(key, &r);
                if r.clicked() {
                    self.seek_search(frame);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (key, label, target) in [
                ("first", "First path", self.first_solution()),
                ("shorter", "Next shorter", self.next_improvement()),
                ("rewire", "Next rewire", self.next_rewire()),
            ] {
                let r = ui.add_enabled(target.is_some(), egui::Button::new(label));
                self.search_action(key, &r);
                if r.clicked() {
                    self.seek_search(target.unwrap());
                }
            }
        });
        let mut frame = self.replay.frame;
        ui.label(format!(
            "Iteration {} / {STAR_ITERATIONS}",
            self.replay.frame
        ));
        if ui
            .add(
                egui::Slider::new(&mut frame, 0..=STAR_ITERATIONS)
                    .show_value(false)
                    .clamping(egui::SliderClamping::Edits),
            )
            .changed()
        {
            self.seek_search(frame);
        }
        ui.horizontal_wrapped(|ui| {
            ui.small("iter/s");
            for (key, label, rate) in [
                ("slow", "25", 25.0),
                ("normal", "100", 100.0),
                ("fast", "250", 250.0),
            ] {
                let r = ui.selectable_label(self.replay.rate == rate, label);
                self.search_action(key, &r);
                if r.clicked() {
                    self.replay.rate = rate;
                    self.replay.pending = 0.0;
                    self.replay.last_time = None;
                }
            }
        });
        self.cost_plot(ui);
        let cost = length(self.path_at(self.replay.frame));
        ui.label(match cost {
            Some(c) => format!("Known path: {c:.2} m"),
            None => "No solution yet".into(),
        });
        ui.label(format!(
            "{} nodes · {} rewires so far",
            self.replay.nodes.len(),
            self.rewire_count()
        ));
        crate::ui_kit::legend(
            ui,
            &[
                (SAMPLE, "sample"),
                (REWIRE, "new connection"),
                (REMOVED, "removed connection"),
            ],
        );
        crate::ui_kit::hint(ui,"Tap the graph to seek. The graph includes only iterations already reached. Compare all four shows completed runs; their budgets differ.");
    }

    fn rewire_count(&self) -> usize {
        self.result
            .as_ref()
            .and_then(|r| r.trace.as_ref())
            .map_or(0, |t| {
                t.steps
                    .iter()
                    .take(self.replay.frame)
                    .flat_map(|s| &s.updates)
                    .filter(|u| {
                        u.before
                            .as_ref()
                            .is_some_and(|b| b.parent != u.after.parent)
                    })
                    .count()
            })
    }

    fn cost_plot(&mut self, ui: &mut egui::Ui) {
        crate::ui_kit::section(ui, "Known path length · tap to inspect");
        let (rect, r) = ui.allocate_exact_size(
            Vec2::new(ui.available_width(), 100.0),
            Sense::click_and_drag(),
        );
        self.replay.plot = Some(rect);
        self.replay
            .actions
            .insert("cost-plot", rect.center().into());
        self.replay.hits.insert("cost-plot", r.hovered());
        if r.clicked() || r.dragged() {
            if let Some(p) = r.interact_pointer_pos() {
                let fraction = ((p.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
                self.seek_search((fraction * STAR_ITERATIONS as f32).round() as usize);
            }
        }
        if r.clicked() || r.drag_stopped() {
            self.ack_search("cost-plot");
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 6.0, Color32::from_rgb(22, 29, 39));
        let floor = (self.goal[0] - self.start[0]).hypot(self.goal[1] - self.start[1]);
        let known: Vec<_> = (0..=self.replay.frame)
            .filter_map(|i| length(self.path_at(i)).map(|c| (i, c)))
            .collect();
        let upper = known
            .first()
            .map_or(floor + 1.0, |(_, c)| *c + 1.0)
            .max(floor + 1.0);
        let points: Vec<_> = known
            .iter()
            .map(|&(i, c)| {
                Pos2::new(
                    rect.left() + i as f32 / STAR_ITERATIONS as f32 * rect.width(),
                    rect.bottom()
                        - 8.0
                        - ((c - floor) / (upper - floor)) as f32 * (rect.height() - 16.0),
                )
            })
            .collect();
        if let Some(&last) = points.last() {
            painter.circle_filled(last, 3.0, PATH);
        }
        if points.len() > 1 {
            painter.add(Shape::line(points, Stroke::new(2.0_f32, PATH)));
        }
        let x = rect.left() + self.replay.frame as f32 / STAR_ITERATIONS as f32 * rect.width();
        painter.line_segment(
            [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
            Stroke::new(1.0_f32, Color32::WHITE),
        );
        painter.text(
            rect.left_top() + Vec2::new(5.0, 4.0),
            egui::Align2::LEFT_TOP,
            if known.is_empty() {
                "No path discovered"
            } else {
                "Known path length (m)"
            },
            egui::FontId::proportional(12.0),
            Color32::LIGHT_GRAY,
        );
    }

    pub(super) fn search_scene(&mut self, ctx: &egui::Context, ui: &mut egui::Ui, rect: Rect) {
        self.advance_search(ctx.input(|i| i.time));
        if self.replay.playing {
            ctx.request_repaint();
        }
        self.replay.rect = Some(rect);
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 8.0, Color32::from_rgb(18, 25, 34));
        let scale = rect.width() / WORLD as f32;
        for i in 0..=10 {
            let a = f64::from(i) * 2.0;
            p.line_segment(
                [to_screen(rect, [a, 0.0]), to_screen(rect, [a, WORLD])],
                Stroke::new(0.5_f32, Color32::from_gray(38)),
            );
            p.line_segment(
                [to_screen(rect, [0.0, a]), to_screen(rect, [WORLD, a])],
                Stroke::new(0.5_f32, Color32::from_gray(38)),
            );
        }
        for &(x, y, r) in &self.obstacles {
            p.circle_filled(to_screen(rect, [x, y]), r as f32 * scale, OBSTACLE);
        }
        for n in &self.replay.nodes {
            if let Some(parent) = n.parent {
                p.line_segment(
                    [
                        to_screen(rect, n.point),
                        to_screen(rect, self.replay.nodes[parent].point),
                    ],
                    Stroke::new(1.0_f32, TREE),
                );
            }
        }
        if let Some(trace) = self.result.as_ref().and_then(|r| r.trace.as_ref()) {
            if let Some(step) = self
                .replay
                .frame
                .checked_sub(1)
                .and_then(|i| trace.steps.get(i))
            {
                let previous = trace.tree_at(self.replay.frame - 1);
                for u in &step.updates {
                    if u.before
                        .as_ref()
                        .is_some_and(|b| b.parent != u.after.parent)
                    {
                        if let Some(before) = &u.before {
                            if let Some(parent) = before.parent {
                                let a = to_screen(rect, before.point);
                                let b = to_screen(rect, previous[parent].point);
                                for k in (0..12).step_by(2) {
                                    let t = k as f32 / 12.0;
                                    let v = (k + 1) as f32 / 12.0;
                                    p.line_segment(
                                        [a + (b - a) * t, a + (b - a) * v],
                                        Stroke::new(2.0_f32, REMOVED),
                                    );
                                }
                            }
                        }
                    }
                    if u.before.is_none()
                        || u.before
                            .as_ref()
                            .is_some_and(|b| b.parent != u.after.parent)
                    {
                        if let Some(parent) = u.after.parent {
                            p.line_segment(
                                [
                                    to_screen(rect, u.after.point),
                                    to_screen(rect, self.replay.nodes[parent].point),
                                ],
                                Stroke::new(2.5_f32, REWIRE),
                            );
                        }
                    }
                }
                let sample = to_screen(rect, step.sample);
                let nearest = to_screen(rect, previous[step.nearest].point);
                p.line_segment(
                    [nearest, sample],
                    Stroke::new(0.8_f32, SAMPLE.gamma_multiply(0.5)),
                );
                p.circle_stroke(nearest, 5.0, Stroke::new(1.5_f32, SAMPLE));
                p.line_segment(
                    [sample - Vec2::new(5.0, 0.0), sample + Vec2::new(5.0, 0.0)],
                    Stroke::new(2.0_f32, SAMPLE),
                );
                p.line_segment(
                    [sample - Vec2::new(0.0, 5.0), sample + Vec2::new(0.0, 5.0)],
                    Stroke::new(2.0_f32, SAMPLE),
                );
                if let Some(candidate) = step.candidate {
                    let pos = to_screen(rect, candidate);
                    let color = if step.accepted { REWIRE } else { REMOVED };
                    p.circle_filled(pos, 4.0, color);
                }
            }
        }
        let path = self.path_at(self.replay.frame);
        if path.len() >= 2 {
            p.add(Shape::line(
                path.iter().map(|p| to_screen(rect, *p)).collect(),
                Stroke::new(3.5_f32, PATH),
            ));
        }
        p.circle_filled(to_screen(rect, self.start), 7.0, START);
        p.circle_filled(to_screen(rect, self.goal), 7.0, GOAL);
        crate::ui_kit::overlay_text(
            ui,
            rect,
            &if self.grab.is_some() {
                "Release to replan".to_owned()
            } else {
                format!("RRT* · iteration {} / {STAR_ITERATIONS}", self.replay.frame)
            },
        );
        let step = self
            .result
            .as_ref()
            .and_then(|r| r.trace.as_ref())
            .and_then(|t| {
                self.replay
                    .frame
                    .checked_sub(1)
                    .and_then(|i| t.steps.get(i))
            });
        let event = step.map_or("Root only", |s| {
            if s.accepted {
                "Node accepted"
            } else {
                "Candidate rejected"
            }
        });
        ui.label(format!(
            "{event} · {} nodes · {}",
            self.replay.nodes.len(),
            length(path).map_or_else(
                || "no path found yet".into(),
                |c| format!("known path {c:.2} m")
            )
        ));
    }

    pub(super) fn finish_search(&mut self, ctx: &egui::Context) {
        self.replay.revision += 1;
        self.publish_search(ctx);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn publish_search(&self, _ctx: &egui::Context) {}

    #[cfg(target_arch = "wasm32")]
    fn publish_search(&self, ctx: &egui::Context) {
        let Some(canvas) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("rust_robotics_playground"))
        else {
            return;
        };
        let actions = self
            .replay
            .actions
            .iter()
            .map(|(k, p)| format!("\"{k}\":[{},{}]", p[0], p[1]))
            .collect::<Vec<_>>()
            .join(",");
        let hits = self
            .replay
            .hits
            .iter()
            .map(|(k, v)| format!("\"{k}\":{v}"))
            .collect::<Vec<_>>()
            .join(",");
        let encode_point = |p: &[f64; 2]| format!("[{},{}]", p[0], p[1]);
        let point_list = |v: &[[f64; 2]]| {
            format!(
                "[{}]",
                v.iter().map(encode_point).collect::<Vec<_>>().join(",")
            )
        };
        let rect_json = |r: Option<Rect>| {
            r.map_or_else(
                || "null".into(),
                |r| format!("[{},{},{},{}]", r.left(), r.top(), r.width(), r.height()),
            )
        };
        let nodes = self
            .replay
            .nodes
            .iter()
            .map(|n| {
                format!(
                    "[{},{},{},{}]",
                    n.point[0],
                    n.point[1],
                    n.parent.map_or_else(|| "null".into(), |p| p.to_string()),
                    n.cost
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let step = self
            .result
            .as_ref()
            .and_then(|r| r.trace.as_ref())
            .and_then(|t| {
                self.replay
                    .frame
                    .checked_sub(1)
                    .and_then(|i| t.steps.get(i))
            });
        let rewires = step.map_or_else(
            || "[]".into(),
            |s| {
                format!(
                    "[{}]",
                    s.updates
                        .iter()
                        .filter_map(|u| u
                            .before
                            .as_ref()
                            .filter(|b| b.parent != u.after.parent)
                            .map(|b| format!(
                                "[{},{},{}]",
                                u.index,
                                b.parent.map_or_else(|| "null".into(), |p| p.to_string()),
                                u.after
                                    .parent
                                    .map_or_else(|| "null".into(), |p| p.to_string())
                            )))
                        .collect::<Vec<_>>()
                        .join(",")
                )
            },
        );
        let costs = (0..=self.replay.frame)
            .map(|i| length(self.path_at(i)).map_or_else(|| "null".into(), |v| v.to_string()))
            .collect::<Vec<_>>()
            .join(",");
        let pointer = ctx
            .input(|i| i.pointer.hover_pos())
            .map_or_else(|| "null".into(), |p| format!("[{},{}]", p.x, p.y));
        for (key, value) in [
            ("kind", self.kind.slug().into()),
            ("editing", u8::from(self.grab.is_some()).to_string()),
            ("frame", self.replay.frame.to_string()),
            ("total", STAR_ITERATIONS.to_string()),
            ("playing", u8::from(self.replay.playing).to_string()),
            ("rate", self.replay.rate.to_string()),
            ("seed", self.seed.to_string()),
            ("nodes", format!("[{nodes}]")),
            ("path", point_list(self.path_at(self.replay.frame))),
            ("costs", format!("[{costs}]")),
            ("rewires", rewires),
            ("rewire-count", self.rewire_count().to_string()),
            (
                "sample",
                step.map_or_else(|| "null".into(), |s| encode_point(&s.sample)),
            ),
            (
                "accepted",
                step.map_or_else(|| "null".into(), |s| u8::from(s.accepted).to_string()),
            ),
            (
                "first",
                self.first_solution()
                    .map_or_else(|| "null".into(), |i| i.to_string()),
            ),
            (
                "shorter",
                self.next_improvement()
                    .map_or_else(|| "null".into(), |i| i.to_string()),
            ),
            ("actions", format!("{{{actions}}}")),
            ("hits", format!("{{{hits}}}")),
            ("pointer", pointer),
            ("action-count", self.replay.count.to_string()),
            ("last-action", self.replay.last_action.into()),
            ("ui-revision", self.replay.revision.to_string()),
            ("rect", rect_json(self.replay.rect)),
            ("plot", rect_json(self.replay.plot)),
            ("share", self.share_query()),
            (
                "obstacles",
                format!(
                    "[{}]",
                    self.obstacles
                        .iter()
                        .map(|(x, y, r)| format!("[{x},{y},{r}]"))
                        .collect::<Vec<_>>()
                        .join(",")
                ),
            ),
            ("robot-radius", super::ROBOT_RADIUS.to_string()),
            ("start", encode_point(&self.start)),
            ("goal", encode_point(&self.goal)),
        ] {
            let _ = canvas.set_attribute(&format!("data-rrt-{key}"), &value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_trace_discovers_and_improves_a_path_without_future_metrics() {
        let mut demo = SamplingDemo::default();
        demo.replan();
        demo.seek_search(0);
        assert_eq!(demo.replay.nodes.len(), 1);
        assert!(demo.path_at(0).is_empty());
        let first = demo
            .first_solution()
            .expect("default seeded field has a route");
        demo.seek_search(first - 1);
        assert!(demo.path_at(demo.replay.frame).is_empty());
        demo.seek_search(first);
        let cost = length(demo.path_at(first)).unwrap();
        let shorter = demo
            .next_improvement()
            .expect("default run improves its route");
        demo.seek_search(shorter);
        assert!(length(demo.path_at(shorter)).unwrap() < cost - 1e-7);
        let rewired = demo.next_rewire().unwrap();
        demo.seek_search(rewired);
        assert!(
            demo.result.as_ref().unwrap().trace.as_ref().unwrap().steps[rewired - 1]
                .updates
                .iter()
                .any(|u| u
                    .before
                    .as_ref()
                    .is_some_and(|b| b.parent != u.after.parent))
        );
        demo.seek_search(STAR_ITERATIONS);
        assert!(demo.rewire_count() > 0);
        assert_eq!(
            demo.path_at(STAR_ITERATIONS),
            demo.result.as_ref().unwrap().path
        );
        println!(
            "default seed7 first={first} shorter={shorter} next_rewire={rewired} rewires={} final={:?}",
            demo.rewire_count(),
            length(demo.path_at(STAR_ITERATIONS))
        );
    }

    #[test]
    fn seeded_share_keeps_precise_map_exact_prefix_and_playback_settings() {
        let mut demo = SamplingDemo {
            seed: 19,
            start: [1.5123456789, 1.523456789],
            ..SamplingDemo::default()
        };
        demo.obstacles[0].2 = 1.5123456789;
        demo.replan();
        demo.seek_search(213);
        demo.replay.rate = 37.1234567;
        let query = demo.share_query();
        let mut restored = SamplingDemo::default();
        restored.apply_share_query(&query);
        assert_eq!(restored.share_query(), query);
        assert_eq!(restored.replay.nodes, demo.replay.nodes);
        assert_eq!(restored.path_at(213), demo.path_at(213));
        assert_eq!(restored.rewire_count(), demo.rewire_count());
        let ctx = egui::Context::default();
        for _ in 0..3 {
            let _ = ctx.run(egui::RawInput::default(), |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| restored.controls(ctx, ui));
            });
            assert_eq!(restored.share_query(), query);
        }
    }

    #[test]
    fn playback_uses_elapsed_time_pauses_and_discards_background_gaps() {
        let mut demo = SamplingDemo::default();
        demo.replan();
        demo.seek_search(0);
        demo.replay.playing = true;
        demo.advance_search(0.0);
        for i in 1..=5 {
            demo.advance_search(f64::from(i) * 0.02);
        }
        assert_eq!(demo.replay.frame, 10);
        demo.seek_search(10);
        demo.advance_search(1.0);
        demo.advance_search(1.5);
        assert_eq!(demo.replay.frame, 10);
        demo.replay.playing = true;
        demo.advance_search(5.0);
        assert_eq!(demo.replay.frame, 10);
        demo.advance_search(5.1);
        assert!((19..=20).contains(&demo.replay.frame));
        demo.seek_search(STAR_ITERATIONS - 1);
        demo.replay.playing = true;
        demo.advance_search(6.0);
        demo.advance_search(6.1);
        assert_eq!(demo.replay.frame, STAR_ITERATIONS);
        assert!(!demo.replay.playing);
    }

    #[test]
    fn blocked_field_has_no_invented_solution_or_costs() {
        let mut demo = SamplingDemo {
            obstacles: (0..=10).map(|i| (10.0, 2.0 * f64::from(i), 1.2)).collect(),
            ..SamplingDemo::default()
        };
        demo.replan();
        assert!(demo.first_solution().is_none());
        assert!(demo.next_improvement().is_none());
        demo.seek_search(STAR_ITERATIONS);
        assert!(demo.path_at(STAR_ITERATIONS).is_empty());
        assert!(demo
            .result
            .as_ref()
            .unwrap()
            .trace
            .as_ref()
            .unwrap()
            .steps
            .iter()
            .all(|s| s.path.is_empty()));
    }

    #[test]
    fn invalid_shared_replay_values_are_ignored_and_other_tabs_are_unchanged() {
        let mut demo = SamplingDemo::default();
        demo.apply_share_query("tab=grid&seed=9&rrt_frame=200");
        assert_eq!(demo.seed, 7);
        assert!(demo.result.is_none());
        demo.apply_share_query(
            "tab=sampling&seed=1000001&rrt_frame=999999&rrt_rate=NaN&rrt_playing=nonsense",
        );
        assert_eq!(demo.seed, 7);
        assert_eq!(demo.replay.frame, 0);
        assert_eq!(demo.replay.rate, 100.0);
        assert!(demo.replay.playing);
    }
}
